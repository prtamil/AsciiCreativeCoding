/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * raster/deferred_rendering_pipeline.c  --  Deferred Rendering Pipeline
 *
 * -----------------------------------------------------------------------
 * WHAT IS DEFERRED RENDERING?
 * -----------------------------------------------------------------------
 * In a game engine, the GPU must answer: "what colour should this pixel be?"
 * The answer depends on surface properties (colour, normal) AND every light
 * that shines on it.  Doing that in one pass -- called FORWARD rendering --
 * means every light must process every object:
 *
 *   Forward cost:  O(objects × lights)   -- 3 objects × 4 lights = 12 passes
 *
 * DEFERRED rendering splits the work into two independent passes:
 *
 *   Pass 1 -- GEOMETRY (render_gbuffer):
 *     Rasterize every object exactly ONCE.  For each screen pixel, store the
 *     world-space surface data (position, normal, colour) in a set of buffers
 *     called the G-buffer.  No shading is done here.  Cost: O(objects).
 *
 *   Pass 2 -- LIGHTING (render_lightpass):
 *     Loop over every pixel in the G-buffer.  Apply Blinn-Phong shading from
 *     every active light using the stored surface data.  No geometry is
 *     re-processed.  Cost: O(lit_pixels × lights).
 *
 *   Deferred cost:  O(objects) + O(pixels × lights)   -- far cheaper when
 *   many lights illuminate the same geometry because each surface is only
 *   rasterized once regardless of how many lights exist.
 *
 * This is exactly how modern engines work:
 *   - Unity HDRP:      "Deferred Rendering Path" with tiled lighting
 *   - Unreal Engine 5: "Deferred Shading" default rendering path
 *   - OpenGL:          Multiple Render Targets (MRT) FBO for geometry pass,
 *                      full-screen quad draw for lighting pass
 *
 * -----------------------------------------------------------------------
 * INTERACTIVE G-BUFFER VISUALIZATION ('g' key cycles through layers)
 * -----------------------------------------------------------------------
 *
 *   POSITION  -- depth-coded characters and colour.
 *                Cyan=near, green=mid, blue=far.  '#' close, '.' far.
 *                Shows exactly where each surface sits in 3-D world space.
 *
 *   NORMAL    -- surface orientation encoded as colour:
 *                +X red | -X cyan | +Y green | -Y magenta | +Z blue | -Z yellow
 *                The sphere shows a smooth colour gradient (smooth normals).
 *                The cube shows flat constant colours per face (flat normals).
 *
 *   ALBEDO    -- raw surface colour with ZERO lighting contribution.
 *                This is exactly what the geometry pass wrote into the buffer.
 *                Press 'l' to add more lights -- this view never changes.
 *                That is the G-buffer guarantee: geometry pass is light-agnostic.
 *
 *   LIGHTING  -- Blinn-Phong shading accumulated from all active point lights.
 *                Press 'l' repeatedly: notice POSITION / NORMAL / ALBEDO stay
 *                constant while LIGHTING updates.  The geometry pass was done
 *                only once; the lighting pass reads its output many times.
 *
 * -----------------------------------------------------------------------
 * SCENE OBJECTS
 * -----------------------------------------------------------------------
 *   Cube   (orange, left)   12 triangles, flat per-face normals (hard edges)
 *   Sphere (blue, right)    UV sphere, smooth per-vertex normals (soft look)
 *   Floor  (grey, below)     2 triangles, single upward normal
 *
 * -----------------------------------------------------------------------
 * KEYS
 * -----------------------------------------------------------------------
 *   g / G   cycle G-buffer display mode (POSITION / NORMAL / ALBEDO / LIGHTING)
 *   l / L   add one point light (wraps back to 1 after MAX_LIGHTS)
 *   space   pause / resume animation
 *   r / R   reset scene (all objects and lights restart)
 *   + / =   zoom in  (decrease camera distance)
 *   -       zoom out (increase camera distance)
 *   q / ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       raster/deferred_rendering_pipeline.c -o deferred -lncurses -lm
 *
 * -----------------------------------------------------------------------
 * SECTION MAP
 * -----------------------------------------------------------------------
 *  S1  config      -- all tunable constants with explanation of each value
 *  S2  clock       -- monotonic nanosecond timer + sleep helper
 *  S3  color       -- ncurses color pairs for every G-buffer layer and HUD
 *  S4  math        -- Vec3 Vec4 Mat4; projection, lookat, normal matrix
 *  S5  mesh        -- vertex/triangle types; cube / sphere / floor builders
 *  S6  gbuffer     -- G-buffer arrays + geometry pass (render_gbuffer)
 *  S7  lightpass   -- PointLight type + Blinn-Phong lighting pass
 *  S8  scene       -- Object, Scene, tick, and all rendering functions
 *  S9  app         -- ncurses lifecycle, signals, input handler, main loop
 * -----------------------------------------------------------------------
 */

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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* S1  config                                                             */
/* ===================================================================== */
/*
 * All simulation constants live here.  Changing any of these values lets
 * you explore how the algorithm and visuals respond -- no code changes
 * needed elsewhere.
 *
 * Why keep constants together?
 *   It mirrors how Unity/Unreal expose "project settings" and "material
 *   properties" -- one place to tune, no hunting through function bodies.
 */

enum {
    /*
     * FPS_TARGET -- how many frames per second the render loop aims for.
     * 60 matches most monitor refresh rates.  Lower (e.g. 30) gives more
     * CPU budget per frame; higher (e.g. 120) makes motion smoother but
     * burns more power.  The main loop sleeps the remaining budget.
     */
    FPS_TARGET    = 60,

    /*
     * FPS_UPDATE_MS -- how often the fps counter is recalculated (ms).
     * 500 ms means the displayed fps updates twice per second, which is
     * readable without flickering.  Lower = more responsive, higher = stable.
     */
    FPS_UPDATE_MS = 500,

    /*
     * HUD_ROWS -- terminal rows reserved at the bottom for the overlay.
     * The scene renders into (total_rows - HUD_ROWS) rows.  Increase this
     * if you add more HUD lines; decrease for a taller viewport.
     */
    HUD_ROWS = 9,

    /*
     * MAX_OBJECTS -- upper bound on renderable objects in the scene.
     * The G-buffer and model-matrix arrays are statically allocated to
     * this size (no dynamic realloc during the render loop).
     */
    MAX_OBJECTS = 4,

    /*
     * MAX_LIGHTS -- maximum point lights the 'l' key can activate.
     * Each additional light adds one full G-buffer read pass in S7.
     * 6 lights demonstrate the cost difference between deferred and
     * forward rendering on the HUD (fwd=18 vs def=9 for 3 objects).
     */
    MAX_LIGHTS = 6,
};

/*
 * G-buffer array dimensions -- sized for the largest expected terminal.
 * Static allocation avoids malloc/free in the hot render path.
 * If you resize your terminal wider than GBUF_MAX_COLS or taller than
 * GBUF_MAX_ROWS, pixels outside these bounds are simply skipped.
 *
 * Memory budget (rough):
 *   g_pos + g_normal + g_albedo + g_light = 4 × Vec3  = 48 bytes/pixel
 *   g_zbuf = 4 bytes, g_valid = 1 byte → total ~53 bytes × 300×80 ≈ 1.3 MB
 */
#define GBUF_MAX_COLS  300
#define GBUF_MAX_ROWS   80

/*
 * Camera field of view (radians).
 * 55° is a comfortable "game camera" FoV -- wider than a telephoto lens
 * (30°) but tighter than a fish-eye (90°+).  Increasing it makes objects
 * look smaller and adds perspective distortion at the edges.
 *
 * OpenGL equivalent: gluPerspective(55.0, aspect, near, far)
 */
#define CAM_FOV  (55.0f * (float)M_PI / 180.0f)

/*
 * CAM_NEAR -- near clipping plane distance from the camera.
 * Any geometry closer than this is clipped.  Too small (e.g. 0.001) causes
 * z-fighting (depth precision errors).  Too large (e.g. 1.0) clips objects
 * that are close to the camera.  0.1 is a common game-engine default.
 */
#define CAM_NEAR  0.1f

/*
 * CAM_FAR -- far clipping plane distance.
 * Objects beyond this are clipped.  A large ratio (far/near = 200 here)
 * reduces depth buffer precision.  For this small scene 20 world units is
 * more than enough.
 */
#define CAM_FAR   20.0f

/*
 * CAM_DIST_DEF / MIN / MAX / ZOOM_STEP -- camera zoom controls.
 * The eye is placed at (0, 0.5, cam_dist), looking at the origin.
 * Zooming moves the eye along +Z; objects never change their world position.
 */
#define CAM_DIST_DEF   3.8f
#define CAM_DIST_MIN   1.8f
#define CAM_DIST_MAX   8.0f
#define CAM_ZOOM_STEP  0.25f

/*
 * CELL_W / CELL_H -- terminal character cell dimensions in pixels.
 * A typical terminal cell is 8 wide × 16 tall (2:1 aspect ratio).
 * Without this correction the perspective matrix would use 1:1 aspect,
 * making the cube and sphere appear vertically stretched by 2×.
 *
 * How it's used: aspect = (cols × CELL_W) / (rows × CELL_H)
 * is passed to m4_perspective so the projection matches the actual
 * pixel footprint of the terminal window.
 */
#define CELL_W   8
#define CELL_H  16

/*
 * SHININESS -- Blinn-Phong specular exponent.
 * Controls the width of the specular highlight (the bright spot from a light):
 *   8   = broad, matte/rough surface (chalk, rubber)
 *   32  = medium, plastic-like (default here -- looks good at ASCII resolution)
 *   128 = tight, metallic surface (polished steel)
 *   512 = very tight, mirror-like
 * Doubling this value halves the highlight size.
 */
#define SHININESS   32.0f

/*
 * AMBIENT_STR -- strength of the ambient (omnidirectional) light term.
 * Ambient prevents completely unlit faces from going pitch black.
 * 0.06 is low enough that you can clearly see the contribution of each
 * point light when added with 'l', but high enough to still see the floor.
 * Unity's ambient intensity is typically 0.05-0.3 depending on scene mood.
 */
#define AMBIENT_STR  0.06f

/*
 * Sphere tessellation resolution.
 * SPHERE_RINGS = latitude bands from pole to pole (more = rounder top/bottom)
 * SPHERE_SEGS  = longitude segments around the equator (more = smoother silhouette)
 * 10×16 gives 320 triangles -- visible roundness without too many pixels per tri.
 * Try RINGS=6 SEGS=8 for a faceted "gem" look, or RINGS=20 SEGS=32 for very smooth.
 */
#define SPHERE_RINGS   10
#define SPHERE_SEGS    16
#define SPHERE_RADIUS  0.55f  /* world-space radius -- matches CUBE_HALF for visual parity */

/*
 * CUBE_HALF -- half-extent of the cube along each axis.
 * The cube spans [-CUBE_HALF, +CUBE_HALF] in X, Y, Z.
 * 0.55 makes the cube slightly larger than its unit size (0.5) for a
 * bolder look at terminal resolution.
 */
#define CUBE_HALF  0.55f

/*
 * Floor geometry.
 * FLOOR_Y     = world-space Y position of the floor surface (below objects)
 * FLOOR_HALF  = half-extent in X and Z; makes the floor wider than the objects
 *               so light spill on the floor is visible from the camera angle.
 */
#define FLOOR_HALF  2.2f
#define FLOOR_Y    -0.80f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/*
 * Paul Bourke's ASCII density ramp (94 printable characters, dark to bright).
 * Source: paulbourke.net/dataformats/asciiart/
 *
 * Maps a [0,1] luminance value to a character whose visual "weight" (ink
 * coverage) corresponds to that brightness.  ' ' = empty (darkest),
 * '@' = dense (brightest).  BOURKE_LEN = 94 distinct levels.
 *
 * Used in: lighting_cell() (LIGHTING mode) and POSITION mode char selection.
 */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/*
 * Bayer 4×4 ordered dither matrix (values normalised to [0,1]).
 *
 * What is ordered dithering?
 *   When mapping continuous luminance to a small set of discrete characters
 *   (only 94 in the Bourke ramp), quantization causes visible "banding" --
 *   sudden jumps between brightness levels.  Dithering adds a per-pixel
 *   offset before quantizing so adjacent pixels alternate across the
 *   threshold, creating the visual illusion of intermediate brightness.
 *
 * The 4×4 Bayer matrix tiles with period 4 in X and Y.  At pixel (px, py)
 * use k_bayer[py & 3][px & 3].  The 16 different offsets prevent all pixels
 * in a 4×4 block from being pushed the same direction.
 *
 * Real engines use blue-noise dither for better perceptual quality, but
 * Bayer is cheap to compute and a classic teaching example.
 */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};

/* ===================================================================== */
/* S2  clock                                                              */
/* ===================================================================== */
/*
 * A high-resolution monotonic clock and a sleep helper.
 *
 * Why monotonic?
 *   CLOCK_MONOTONIC never jumps backward (unlike CLOCK_REALTIME which can be
 *   adjusted by NTP).  Frame delta time must always be positive; a backward
 *   jump would give negative dt and make objects spin the wrong way.
 *
 * clock_ns()   -- current time in nanoseconds (absolute, epoch doesn't matter)
 * clock_sleep_ns() -- sleep for a precise number of nanoseconds (frame cap)
 */

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

/* ===================================================================== */
/* S3  color                                                              */
/* ===================================================================== */
/*
 * ncurses COLOR_PAIR layout for every rendering mode.
 *
 * ncurses requires pairs to be defined once at startup (color_init) and
 * then referenced by integer ID throughout rendering.  We give each pair a
 * named constant so render functions read like "use the +X-normal colour"
 * rather than "use pair 8".
 *
 * Pair groups:
 *   CP_LIT_1..7   -- 7-level luma ramp for LIGHTING mode (dark-red to white)
 *   CP_NRM_PX..NZ -- six cardinal direction tints for NORMAL mode
 *   CP_OBJ_*      -- per-object hue for ALBEDO mode
 *   CP_HUD_*      -- overlay text colours
 *   CP_MODE_*     -- mode label highlight colours in the HUD status bar
 *
 * 256-colour terminals: exact xterm-256 palette indices (e.g. 196=red,
 * 226=yellow, 51=cyan) for vivid, unambiguous colours.
 * 8-colour fallback: the standard COLOR_* macros which every terminal supports.
 */
enum {
    /* LIGHTING mode: luma ramp dark red -> bright white */
    CP_LIT_1 = 1, CP_LIT_2, CP_LIT_3, CP_LIT_4, CP_LIT_5, CP_LIT_6, CP_LIT_7,

    /* NORMAL mode: one pair per cardinal axis direction */
    CP_NRM_PX,    /* +X axis: red      */
    CP_NRM_NX,    /* -X axis: cyan     */
    CP_NRM_PY,    /* +Y axis: green    */
    CP_NRM_NY,    /* -Y axis: magenta  */
    CP_NRM_PZ,    /* +Z axis: blue     */
    CP_NRM_NZ,    /* -Z axis: yellow   */

    /* ALBEDO mode: object-type hues */
    CP_OBJ_CUBE,     /* warm orange -- matches cube albedo (0.90, 0.52, 0.12) */
    CP_OBJ_SPHERE,   /* steel blue  -- matches sphere albedo (0.18, 0.48, 0.90) */
    CP_OBJ_FLOOR,    /* medium grey -- matches floor albedo (0.48, 0.48, 0.50) */

    /* HUD text tiers */
    CP_HUD,       /* body text: light grey              */
    CP_HUD_VAL,   /* numeric values: cyan (stands out)  */
    CP_HUD_HDR,   /* header / title: yellow             */
    CP_HUD_EXP,   /* explanatory text: dim grey         */

    /* Mode label tags in the G-buffer selector row */
    CP_MODE_POS,  /* POSITION label: light blue  */
    CP_MODE_NRM,  /* NORMAL label:   light green */
    CP_MODE_ALB,  /* ALBEDO label:   pale yellow */
    CP_MODE_LIT,  /* LIGHTING label: bright white */
};

static void color_init(void)
{
    start_color();
    use_default_colors();          /* -1 = terminal background (transparent) */
    if (COLORS >= 256) {
        /* xterm-256 palette: precise colours */
        init_pair(CP_LIT_1, 196, -1);  /* red         */
        init_pair(CP_LIT_2, 208, -1);  /* orange      */
        init_pair(CP_LIT_3, 226, -1);  /* yellow      */
        init_pair(CP_LIT_4,  46, -1);  /* green       */
        init_pair(CP_LIT_5,  51, -1);  /* cyan        */
        init_pair(CP_LIT_6, 231, -1);  /* bright white */
        init_pair(CP_LIT_7, 255, -1);  /* near-white  */

        init_pair(CP_NRM_PX, 196, -1);  /* +X: red      */
        init_pair(CP_NRM_NX,  51, -1);  /* -X: cyan     */
        init_pair(CP_NRM_PY,  46, -1);  /* +Y: green    */
        init_pair(CP_NRM_NY, 201, -1);  /* -Y: magenta  */
        init_pair(CP_NRM_PZ,  21, -1);  /* +Z: blue     */
        init_pair(CP_NRM_NZ, 226, -1);  /* -Z: yellow   */

        init_pair(CP_OBJ_CUBE,   208, -1);  /* orange */
        init_pair(CP_OBJ_SPHERE,  39, -1);  /* blue   */
        init_pair(CP_OBJ_FLOOR,  244, -1);  /* grey   */

        init_pair(CP_HUD,     252, -1);  /* light grey */
        init_pair(CP_HUD_VAL,  51, -1);  /* cyan       */
        init_pair(CP_HUD_HDR, 226, -1);  /* yellow     */
        init_pair(CP_HUD_EXP, 244, -1);  /* dim grey   */

        init_pair(CP_MODE_POS, 159, -1);  /* sky blue    */
        init_pair(CP_MODE_NRM, 120, -1);  /* light green */
        init_pair(CP_MODE_ALB, 222, -1);  /* pale yellow */
        init_pair(CP_MODE_LIT, 231, -1);  /* white       */
    } else {
        /* 8-colour fallback */
        init_pair(CP_LIT_1, COLOR_RED,     -1);
        init_pair(CP_LIT_2, COLOR_RED,     -1);
        init_pair(CP_LIT_3, COLOR_YELLOW,  -1);
        init_pair(CP_LIT_4, COLOR_GREEN,   -1);
        init_pair(CP_LIT_5, COLOR_CYAN,    -1);
        init_pair(CP_LIT_6, COLOR_WHITE,   -1);
        init_pair(CP_LIT_7, COLOR_WHITE,   -1);

        init_pair(CP_NRM_PX, COLOR_RED,     -1);
        init_pair(CP_NRM_NX, COLOR_CYAN,    -1);
        init_pair(CP_NRM_PY, COLOR_GREEN,   -1);
        init_pair(CP_NRM_NY, COLOR_MAGENTA, -1);
        init_pair(CP_NRM_PZ, COLOR_BLUE,    -1);
        init_pair(CP_NRM_NZ, COLOR_YELLOW,  -1);

        init_pair(CP_OBJ_CUBE,   COLOR_RED,   -1);
        init_pair(CP_OBJ_SPHERE, COLOR_BLUE,  -1);
        init_pair(CP_OBJ_FLOOR,  COLOR_WHITE, -1);

        init_pair(CP_HUD,     COLOR_WHITE,  -1);
        init_pair(CP_HUD_VAL, COLOR_CYAN,   -1);
        init_pair(CP_HUD_HDR, COLOR_YELLOW, -1);
        init_pair(CP_HUD_EXP, COLOR_WHITE,  -1);

        init_pair(CP_MODE_POS, COLOR_CYAN,   -1);
        init_pair(CP_MODE_NRM, COLOR_GREEN,  -1);
        init_pair(CP_MODE_ALB, COLOR_YELLOW, -1);
        init_pair(CP_MODE_LIT, COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* S4  math  --  Vec3, Vec4, Mat4                                         */
/* ===================================================================== */
/*
 * A minimal 3-D math library sufficient for a software rasterizer.
 *
 * Coordinate convention: right-handed, Y-up (same as OpenGL).
 *   +X = right,  +Y = up,  +Z = toward camera (out of screen)
 *
 * Column-major matrix layout (same as GLSL / OpenGL):
 *   m[row][col]  -- so translation is m[0][3], m[1][3], m[2][3]
 *
 * Functions named m4_pt  transform a POINT  (w=1 → translation applies)
 * Functions named m4_dir transform a DIRECTION (w=0 → translation ignored)
 */

typedef struct { float x, y, z;    } Vec3;
typedef struct { float x, y, z, w; } Vec4;
typedef struct { float m[4][4];    } Mat4;

static inline Vec3 v3(float x, float y, float z)         { return (Vec3){x,y,z}; }
static inline Vec4 v4(float x, float y, float z, float w){ return (Vec4){x,y,z,w}; }

static inline Vec3 v3_add  (Vec3 a, Vec3 b)   { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3 v3_sub  (Vec3 a, Vec3 b)   { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3 v3_scale(Vec3 a, float s)  { return v3(a.x*s, a.y*s, a.z*s); }
static inline float v3_dot (Vec3 a, Vec3 b)   { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3_len (Vec3 a)           { return sqrtf(v3_dot(a, a)); }

/* v3_norm -- return the unit vector of a.  Safe: returns (0,1,0) for zero-length. */
static inline Vec3 v3_norm(Vec3 a)
{
    float l = v3_len(a);
    return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}

/*
 * v3_bary -- barycentric interpolation of three Vec3 values.
 *
 * Given three vertex values p0, p1, p2 and per-pixel barycentric weights
 * b0+b1+b2 = 1, this computes the weighted blend at a point inside the
 * triangle.  Used to interpolate world_pos and world_nrm for every pixel.
 *
 * This is the software equivalent of the GPU's "varying" interpolation:
 *   in GLSL, outputs declared as "out vec3 vPos" in the vertex shader are
 *   automatically interpolated per-fragment using barycentric weights.
 */
static inline Vec3 v3_bary(Vec3 p0, Vec3 p1, Vec3 p2, float b0, float b1, float b2)
{
    return v3(b0*p0.x + b1*p1.x + b2*p2.x,
              b0*p0.y + b1*p1.y + b2*p2.y,
              b0*p0.z + b1*p1.z + b2*p2.z);
}

static inline Mat4 m4_identity(void)
{
    Mat4 m = {{{0}}};
    m.m[0][0] = m.m[1][1] = m.m[2][2] = m.m[3][3] = 1.f;
    return m;
}

/*
 * m4_mul_v4 -- multiply a 4×4 matrix by a 4-component column vector.
 * Used by m4_pt (w=1) and m4_dir (w=0) to apply transforms to geometry.
 */
static inline Vec4 m4_mul_v4(Mat4 m, Vec4 v)
{
    return v4(m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z + m.m[0][3]*v.w,
              m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z + m.m[1][3]*v.w,
              m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z + m.m[2][3]*v.w,
              m.m[3][0]*v.x + m.m[3][1]*v.y + m.m[3][2]*v.z + m.m[3][3]*v.w);
}

/* m4_mul -- compose two transforms: A×B applies B first, then A. */
static inline Mat4 m4_mul(Mat4 a, Mat4 b)
{
    Mat4 r = {{{0}}};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

/* m4_pt  -- transform a POINT  (w=1, translation IS applied) */
static inline Vec3 m4_pt(Mat4 m, Vec3 p)
{
    Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
    return v3(r.x, r.y, r.z);
}

/* m4_dir -- transform a DIRECTION (w=0, translation is NOT applied) */
static inline Vec3 m4_dir(Mat4 m, Vec3 d)
{
    Vec4 r = m4_mul_v4(m, v4(d.x, d.y, d.z, 0.f));
    return v3(r.x, r.y, r.z);
}

static Mat4 m4_translate(Vec3 t)
{
    Mat4 m = m4_identity();
    m.m[0][3] = t.x; m.m[1][3] = t.y; m.m[2][3] = t.z;
    return m;
}

static Mat4 m4_rotate_y(float a)
{
    Mat4 m = m4_identity();
    m.m[0][0] =  cosf(a); m.m[0][2] = sinf(a);
    m.m[2][0] = -sinf(a); m.m[2][2] = cosf(a);
    return m;
}

static Mat4 m4_rotate_x(float a)
{
    Mat4 m = m4_identity();
    m.m[1][1] =  cosf(a); m.m[1][2] = -sinf(a);
    m.m[2][1] =  sinf(a); m.m[2][2] =  cosf(a);
    return m;
}

/*
 * m4_perspective -- build a perspective projection matrix.
 *
 * Maps the view frustum to NDC (Normalized Device Coordinates):
 *   X, Y: [-1, +1]   Z: [-1, +1] (OpenGL convention; Vulkan uses [0,1])
 *
 * Key entries:
 *   m[0][0] = f/aspect    -- horizontal scale (wider aspect = smaller number)
 *   m[1][1] = f           -- vertical scale (f = 1/tan(fovy/2))
 *   m[2][2] = (far+near)/(near-far)  -- depth remapping
 *   m[2][3] = 2*far*near/(near-far)  -- depth bias term
 *   m[3][2] = -1          -- perspective divide trigger: output w = -z_view
 *             (after m4_mul_v4 the clip.w = -z_view; dividing x,y,z by w
 *              is the perspective divide that makes far things smaller)
 *
 * GLSL equivalent:
 *   mat4 proj = perspective(radians(55.0), aspect, 0.1, 20.0);
 */
static Mat4 m4_perspective(float fovy, float aspect, float near, float far)
{
    Mat4 m = {{{0}}};
    float f = 1.f / tanf(fovy * 0.5f);   /* f = cot(fovy/2) */
    m.m[0][0] = f / aspect;
    m.m[1][1] = f;
    m.m[2][2] = (far + near) / (near - far);
    m.m[2][3] = (2.f * far * near) / (near - far);
    m.m[3][2] = -1.f;   /* w = -z_view --> enables perspective divide */
    return m;
}

/*
 * m4_lookat -- build a view matrix from eye position, target, and up hint.
 *
 * The view matrix rotates and translates world space so the camera is at
 * the origin, looking down -Z.  Three orthonormal basis vectors:
 *   f = forward (eye -> target, then negated for -Z convention)
 *   r = right   = cross(forward, up)
 *   u = up      = cross(right, forward)  (corrected up, perpendicular to r and f)
 *
 * Matrix rows are [r, u, -f] plus their dot products with eye position
 * to encode the inverse translation.
 *
 * GLSL equivalent: mat4 view = lookAt(eye, target, vec3(0,1,0));
 */
static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up)
{
    Vec3 f = v3_norm(v3_sub(at, eye));                /* forward vector          */
    Vec3 r = v3_norm(v3(f.z*up.y - f.y*up.z,          /* right = f × up          */
                        f.x*up.z - f.z*up.x,
                        f.y*up.x - f.x*up.y));
    Vec3 u = v3(r.y*f.z - r.z*f.y,                    /* corrected up = r × f    */
                r.z*f.x - r.x*f.z,
                r.x*f.y - r.y*f.x);
    Mat4 m = m4_identity();
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-v3_dot(r, eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-v3_dot(u, eye);
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]= v3_dot(f, eye);
    return m;
}

/*
 * m4_normal_mat -- cofactor (adjugate) of the upper-left 3×3 of a model matrix.
 *
 * WHY NOT JUST USE THE MODEL MATRIX FOR NORMALS?
 *   Under non-uniform scale, the model matrix distorts normals so they are
 *   no longer perpendicular to the surface.  The correct transform is the
 *   INVERSE TRANSPOSE of the upper 3×3.
 *
 * For a 3×3 matrix, inverse-transpose = cofactor matrix / determinant.
 * Since we only need the direction (we re-normalize anyway), we skip the
 * division by determinant and just compute the cofactor.
 *
 * For pure rotation (no scale) the cofactor equals the rotation matrix,
 * so this is always correct and costs only a few multiplies.
 *
 * OpenGL equivalent: mat3 normalMatrix = transpose(inverse(mat3(model)));
 */
static Mat4 m4_normal_mat(Mat4 m)
{
    Mat4 n = m4_identity();
    /* Cofactors of the 3×3 submatrix */
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

/* ===================================================================== */
/* S5  mesh                                                               */
/* ===================================================================== */
/*
 * Geometry representation:
 *
 *   Vertex   -- a corner of a triangle with position, surface normal, and UV.
 *   Triangle -- three integer indices into a Vertex array (index buffer).
 *   Mesh     -- heap-allocated arrays of vertices and triangles.
 *
 * This mirrors the GPU's Vertex Buffer Object (VBO) + Index Buffer Object (IBO)
 * design.  The rasterizer (rasterize_object in S6) iterates the triangle list
 * and looks up vertices by index -- just like the GPU's "draw call".
 *
 * FLAT vs SMOOTH NORMALS (important for visual appearance):
 *
 *   FLAT normals (cube):
 *     Each face of the cube has 4 unique vertices that all share the same
 *     outward normal (e.g., {1,0,0} for the +X face).  When these normals
 *     are stored in the G-buffer and displayed in NORMAL mode, each face
 *     shows one solid colour.  This makes cube edges appear hard/sharp.
 *
 *   SMOOTH normals (sphere):
 *     For a UV sphere, each vertex's normal equals its position divided by
 *     the radius: the outward unit vector of the sphere surface.  Adjacent
 *     vertices have slightly different normals; the barycentric interpolation
 *     in rasterize_object blends them across each triangle, producing the
 *     smooth colour gradient visible in NORMAL mode.
 */

typedef struct { Vec3 pos; Vec3 normal; float u, v; } Vertex;
typedef struct { int  v[3]; }                          Triangle;
typedef struct { Vertex *verts; Triangle *tris; int nvert, ntri; } Mesh;

static void mesh_free(Mesh *m) { free(m->verts); free(m->tris); *m = (Mesh){0}; }

/*
 * tessellate_cube -- 6 faces × 4 unique vertices = 24 verts, 12 triangles.
 *
 * Why 4 unique vertices per face (24 total) instead of 8 shared corners?
 *   Each face needs its own outward normal.  If the corner vertices were
 *   shared between three faces, we'd have to average the normals (giving
 *   smooth shading).  Duplicating gives each face a single flat normal,
 *   which produces the hard-edge look typical of a box.
 *
 * Winding order: counter-clockwise (CCW) viewed from outside the face.
 *   The back-face cull in rasterize_object skips triangles whose screen-
 *   space signed area is <= 0 (clockwise), so only outward-facing tris render.
 *
 * Local arrays (not static const) because CUBE_HALF is a non-constant
 * expression and C11 disallows non-constants in static initializers.
 * This function is called only once so stack allocation is fine.
 */
static Mesh tessellate_cube(void)
{
    float h = CUBE_HALF;

    float face_nrm[6][3] = {
        { 1,0,0},{-1,0,0},{0, 1,0},{0,-1,0},{0,0, 1},{0,0,-1}
    };
    float face_vtx[6][4][3] = {
        /* +X face */ {{ h,-h, h},{ h, h, h},{ h, h,-h},{ h,-h,-h}},
        /* -X face */ {{-h,-h,-h},{-h, h,-h},{-h, h, h},{-h,-h, h}},
        /* +Y face */ {{-h, h, h},{ h, h, h},{ h, h,-h},{-h, h,-h}},
        /* -Y face */ {{-h,-h,-h},{ h,-h,-h},{ h,-h, h},{-h,-h, h}},
        /* +Z face */ {{-h,-h, h},{ h,-h, h},{ h, h, h},{-h, h, h}},
        /* -Z face */ {{ h,-h,-h},{-h,-h,-h},{-h, h,-h},{ h, h,-h}},
    };
    float face_uv[4][2] = {{0,1},{1,1},{1,0},{0,0}};  /* CCW quad UV corners */

    Mesh m;
    m.verts = malloc(24 * sizeof(Vertex));
    m.tris  = malloc(12 * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;

    for (int f = 0; f < 6; f++) {
        Vec3 n   = v3(face_nrm[f][0], face_nrm[f][1], face_nrm[f][2]);
        int  base = m.nvert;
        for (int i = 0; i < 4; i++) {
            Vec3 p = v3(face_vtx[f][i][0], face_vtx[f][i][1], face_vtx[f][i][2]);
            m.verts[m.nvert++] = (Vertex){ p, n, face_uv[i][0], face_uv[i][1] };
        }
        /* Each quad = two CCW triangles sharing the 0-2 diagonal */
        m.tris[m.ntri++] = (Triangle){{ base, base+1, base+2 }};
        m.tris[m.ntri++] = (Triangle){{ base, base+2, base+3 }};
    }
    return m;
}

/*
 * tessellate_sphere -- UV sphere parameterization.
 *
 * Vertices are laid out on a (rings+1) × (segs+1) grid in spherical coords:
 *
 *   theta = pi * i / rings         -- polar angle:    0 (north) to pi (south)
 *   phi   = 2*pi * j / segs        -- azimuth angle:  0 to 2*pi
 *
 *   pos = radius * ( sin(theta)*cos(phi),   -- X
 *                    cos(theta),            -- Y (poles at +Y and -Y)
 *                    sin(theta)*sin(phi) )  -- Z
 *
 *   normal = pos / radius   (the outward unit normal for a perfect sphere
 *                            equals the normalized position vector)
 *
 * This gives SMOOTH normals: two adjacent quads share edge vertices whose
 * normals point in slightly different directions.  Barycentric interpolation
 * across each triangle blends them, producing the smooth shading gradient
 * visible in NORMAL mode and the soft specular highlights in LIGHTING mode.
 *
 * Contrast with the cube: change SPHERE_RINGS to 3 and SPHERE_SEGS to 4
 * to see a low-poly octahedron-like shape where the smooth normals still
 * make it look rounder than it geometrically is.
 */
static Mesh tessellate_sphere(float radius, int rings, int segs)
{
    int n_verts = (rings + 1) * (segs + 1);
    int n_tris  = rings * segs * 2;
    Mesh m;
    m.verts = malloc((size_t)n_verts * sizeof(Vertex));
    m.tris  = malloc((size_t)n_tris  * sizeof(Triangle));
    m.nvert = 0; m.ntri = 0;

    for (int i = 0; i <= rings; i++) {
        float theta = (float)M_PI * (float)i / (float)rings;
        float sin_t = sinf(theta);
        float cos_t = cosf(theta);
        for (int j = 0; j <= segs; j++) {
            float phi = 2.f * (float)M_PI * (float)j / (float)segs;
            float x   = radius * sin_t * cosf(phi);
            float y   = radius * cos_t;
            float z   = radius * sin_t * sinf(phi);
            Vec3 pos  = v3(x, y, z);
            Vec3 nrm  = v3(x/radius, y/radius, z/radius);  /* unit outward normal */
            float u   = (float)j / (float)segs;
            float v_  = (float)i / (float)rings;
            m.verts[m.nvert++] = (Vertex){ pos, nrm, u, v_ };
        }
    }

    /* Connect the vertex grid into quads, each split into two CCW triangles */
    for (int i = 0; i < rings; i++) {
        for (int j = 0; j < segs; j++) {
            int v00 =  i    * (segs+1) + j;      /* top-left of quad  */
            int v10 = (i+1) * (segs+1) + j;      /* bottom-left       */
            int v11 = (i+1) * (segs+1) + (j+1);  /* bottom-right      */
            int v01 =  i    * (segs+1) + (j+1);  /* top-right         */
            m.tris[m.ntri++] = (Triangle){{ v00, v10, v01 }};
            m.tris[m.ntri++] = (Triangle){{ v10, v11, v01 }};
        }
    }
    return m;
}

/*
 * tessellate_floor -- a flat axis-aligned quad at a fixed Y height.
 *
 * The floor uses a single upward normal (+Y) for all four vertices.
 * This means the Blinn-Phong diffuse term is at maximum when a light is
 * directly above (dot(N,L) = 1 with N = (0,1,0) and L pointing up).
 * With orbit_plane=0 (horizontal ring), point lights pass both above and
 * at the same height as the floor, letting you watch the diffuse term change.
 */
static Mesh tessellate_floor(float y, float half)
{
    Mesh m;
    m.verts = malloc(4 * sizeof(Vertex));
    m.tris  = malloc(2 * sizeof(Triangle));
    m.nvert = 4; m.ntri = 2;
    Vec3 n = v3(0, 1, 0);   /* normal: upward (+Y) */
    m.verts[0] = (Vertex){ v3(-half, y,  half), n, 0, 0 };
    m.verts[1] = (Vertex){ v3( half, y,  half), n, 1, 0 };
    m.verts[2] = (Vertex){ v3( half, y, -half), n, 1, 1 };
    m.verts[3] = (Vertex){ v3(-half, y, -half), n, 0, 1 };
    m.tris[0] = (Triangle){{ 0, 1, 2 }};
    m.tris[1] = (Triangle){{ 0, 2, 3 }};
    return m;
}

/* ===================================================================== */
/* S6  gbuffer  --  geometry pass                                         */
/* ===================================================================== */
/*
 * The G-buffer is a set of 2-D arrays -- one slot per screen pixel -- that
 * store surface properties for the CLOSEST geometry at that pixel.
 *
 * In a real GPU engine (OpenGL / Vulkan / DirectX 12):
 *   The G-buffer is a collection of "render targets" (textures).
 *   The geometry pass renders to them using "Multiple Render Targets" (MRT):
 *   glBindFramebuffer + glDrawBuffers([RT0, RT1, RT2, ...])
 *   The fragment shader writes: layout(location=0) out vec4 gPosition;
 *                               layout(location=1) out vec4 gNormal;
 *                               layout(location=2) out vec4 gAlbedo;
 *
 * UNITY HDRP G-BUFFER LAYOUT (for reference):
 *   RT0: Albedo (RGB) + Roughness (A)
 *   RT1: Normal (RGB) packed as octahedron, Metallic (A)
 *   RT2: Specular colour (RGB), Occlusion (A)
 *   RT3: Emissive light (RGB), unused (A)
 *
 * UNREAL ENGINE 5 G-BUFFER LAYOUT (simplified):
 *   GBufferA: Worldspace Normal (RGB) + per-object data (A)
 *   GBufferB: Metallic/Specular/Roughness (packed into R,G,B) + ShadingModel (A)
 *   GBufferC: BaseColor (RGB) + AO (A)
 *
 * OUR SIMPLIFIED LAYOUT (one layer = one concept for clarity):
 *   g_pos    -- world XYZ of the frontmost surface at each pixel
 *   g_normal -- world-space unit normal of that surface
 *   g_albedo -- flat RGB colour (texture / material; no lighting)
 *   g_zbuf   -- NDC-z depth value used for the depth test
 *   g_valid  -- 1 once any geometry has touched this pixel
 *
 * After the geometry pass, render_lightpass reads these arrays to shade
 * each pixel -- never touching the mesh/triangle data again.
 * The output of the lighting pass is stored in:
 *   g_light  -- final Blinn-Phong shaded colour per pixel
 */

static Vec3    g_pos   [GBUF_MAX_ROWS][GBUF_MAX_COLS];
static Vec3    g_normal[GBUF_MAX_ROWS][GBUF_MAX_COLS];
static Vec3    g_albedo[GBUF_MAX_ROWS][GBUF_MAX_COLS];
static float   g_zbuf  [GBUF_MAX_ROWS][GBUF_MAX_COLS];
static uint8_t g_valid [GBUF_MAX_ROWS][GBUF_MAX_COLS];
static Vec3    g_light [GBUF_MAX_ROWS][GBUF_MAX_COLS];  /* output of lighting pass */

/*
 * gbuffer_clear -- reset the depth buffer and valid flags before each frame.
 *
 * g_zbuf is initialised to +1.0 (the far plane in NDC).  Every geometry write
 * must pass the test z < g_zbuf to overwrite it, ensuring the closest surface
 * wins.  g_pos/g_normal/g_albedo don't need clearing because g_valid gates all
 * reads in render_scene and render_lightpass.
 *
 * OpenGL equivalent: glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
 */
static void gbuffer_clear(int cols, int rows)
{
    for (int r = 0; r < rows && r < GBUF_MAX_ROWS; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_COLS; c++) {
            g_zbuf [r][c] = 1.0f;  /* NDC far plane (+1) -- everything is "behind" this */
            g_valid[r][c] = 0;
        }
    }
}

/*
 * barycentric -- compute interpolation weights for point (px,py) inside a
 * screen-space triangle defined by vertices (sx[0..2], sy[0..2]).
 *
 * Output: b[0], b[1], b[2] such that b[0]+b[1]+b[2] = 1.
 *
 *   If all three >= 0: point is INSIDE the triangle.
 *   If any <  0:       point is OUTSIDE -- skip this pixel.
 *
 * The formula uses the ratio of sub-triangle areas to the full triangle area.
 * The denominator 'd' is the signed area of the full triangle; if it's near
 * zero the triangle is degenerate (line or point) and we set all weights to
 * -1 to signal "skip this pixel".
 *
 * These weights are then used in v3_bary to interpolate world position and
 * surface normals -- the same interpolation the GPU does automatically for
 * "varying" / "in" fragment shader inputs.
 */
static void barycentric(const float sx[3], const float sy[3],
                        float px, float py, float b[3])
{
    float d = (sy[1]-sy[2])*(sx[0]-sx[2]) + (sx[2]-sx[1])*(sy[0]-sy[2]);
    if (fabsf(d) < 1e-6f) { b[0] = b[1] = b[2] = -1.f; return; }
    b[0] = ((sy[1]-sy[2])*(px-sx[2]) + (sx[2]-sx[1])*(py-sy[2])) / d;
    b[1] = ((sy[2]-sy[0])*(px-sx[2]) + (sx[0]-sx[2])*(py-sy[2])) / d;
    b[2] = 1.f - b[0] - b[1];
}

/*
 * rasterize_object -- scan-convert one mesh into the G-buffer.
 *
 * This function implements the core of the GPU vertex + rasterization pipeline
 * in software.  There are three conceptual stages, matching how the GPU works:
 *
 * ┌───────────────────────────────────────────────────────────────────────┐
 * │ STAGE 1: VERTEX TRANSFORM  (GPU: Vertex Shader)                       │
 * │                                                                       │
 * │ Each vertex position is multiplied by the MVP matrix                  │
 * │ (Model × View × Projection) to produce a clip-space Vec4.            │
 * │                                                                       │
 * │ The normal is transformed separately by the NORMAL MATRIX             │
 * │ (cofactor of the model matrix upper 3×3) to maintain perpendicularity │
 * │ under non-uniform scale.  The result is renormalized.                 │
 * │                                                                       │
 * │ World-space position and normal are also saved (from model matrix)    │
 * │ for later G-buffer writes -- the lighting pass needs world-space data. │
 * └───────────────────────────────────────────────────────────────────────┘
 * ┌───────────────────────────────────────────────────────────────────────┐
 * │ STAGE 2: PERSPECTIVE DIVIDE + RASTERIZATION SETUP  (GPU: fixed fn)   │
 * │                                                                       │
 * │ Clip-space → NDC: divide X,Y,Z by W (perspective divide).            │
 * │   NDC X: -1=left edge    +1=right edge                               │
 * │   NDC Y: -1=bottom edge  +1=top edge  (note: Y-flip to screen below)  │
 * │   NDC Z: -1=near plane   +1=far plane                                 │
 * │                                                                       │
 * │ NDC → screen pixels:                                                  │
 * │   sx = (ndcX + 1) * 0.5 * cols                                       │
 * │   sy = (-ndcY + 1) * 0.5 * rows   ← Y is flipped (NDC Y-up,          │
 * │                                      screen Y-down)                   │
 * │                                                                       │
 * │ BACK-FACE CULL:                                                       │
 * │   Compute screen-space signed area of the triangle.                   │
 * │   Positive area = CCW winding = front face → render it.               │
 * │   Zero or negative area = CW winding = back face → skip it.          │
 * │   Formula: (sx[1]-sx[0])*(sy[2]-sy[0]) - (sx[2]-sx[0])*(sy[1]-sy[0]) │
 * └───────────────────────────────────────────────────────────────────────┘
 * ┌───────────────────────────────────────────────────────────────────────┐
 * │ STAGE 3: FRAGMENT / PIXEL FILL  (GPU: Fragment Shader → MRT write)   │
 * │                                                                       │
 * │ For each pixel (px,py) in the triangle's axis-aligned bounding box:   │
 * │   1. Compute barycentric weights (b0, b1, b2).                        │
 * │   2. Reject pixels outside the triangle (any weight < 0).             │
 * │   3. Interpolate NDC-z for the depth test.                            │
 * │   4. DEPTH TEST: if z >= g_zbuf[py][px], a closer surface already     │
 * │      occupies this pixel -- skip.                                     │
 * │   5. DEPTH WRITE: g_zbuf[py][px] = z.                                 │
 * │   6. G-BUFFER WRITE:                                                  │
 * │        g_pos    ← interpolated world position                         │
 * │        g_normal ← interpolated + renormalized world normal            │
 * │        g_albedo ← flat surface colour (constant per object)           │
 * │        g_valid  ← 1 (this pixel has geometry)                         │
 * └───────────────────────────────────────────────────────────────────────┘
 */
static void rasterize_object(
    const Mesh *mesh, Vec3 albedo,
    Mat4 mvp, Mat4 model, Mat4 norm_mat,
    int cols, int rows)
{
    for (int ti = 0; ti < mesh->ntri; ti++) {
        const Triangle *tri = &mesh->tris[ti];

        /* --- STAGE 1: Vertex Transform ---------------------------------- */
        Vec4 clip[3];
        Vec3 wpos[3];  /* world-space positions (for G-buffer pos layer)  */
        Vec3 wnrm[3];  /* world-space normals   (for G-buffer nrm layer)  */
        for (int vi = 0; vi < 3; vi++) {
            const Vertex *v = &mesh->verts[tri->v[vi]];
            clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
            wpos[vi] = m4_pt (model,    v->pos);
            wnrm[vi] = v3_norm(m4_dir(norm_mat, v->normal));
        }

        /* Skip triangles entirely behind the near plane (all w negative) */
        if (clip[0].w < 0.001f && clip[1].w < 0.001f && clip[2].w < 0.001f)
            continue;

        /* --- STAGE 2: Perspective Divide → Screen Space ----------------- */
        float sx[3], sy[3], sz[3];
        for (int vi = 0; vi < 3; vi++) {
            float w = clip[vi].w;
            if (fabsf(w) < 1e-6f) w = 1e-6f;
            sx[vi] = ( clip[vi].x / w + 1.f) * 0.5f * (float)cols;
            sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;  /* Y-flip */
            sz[vi] =   clip[vi].z / w;                               /* NDC-z  */
        }

        /* Back-face cull: signed screen-space area; skip CW triangles */
        float area = (sx[1]-sx[0])*(sy[2]-sy[0]) - (sx[2]-sx[0])*(sy[1]-sy[0]);
        if (area <= 0.f) continue;

        /* Tight bounding box over the triangle, clamped to the render target */
        int x0 = (int)fmaxf(0.f,        floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
        int x1 = (int)fminf(cols - 1.f,  ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
        int y0 = (int)fmaxf(0.f,        floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
        int y1 = (int)fminf(rows - 1.f,  ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

        /* --- STAGE 3: Fragment / Pixel Fill ----------------------------- */
        for (int py = y0; py <= y1 && py < GBUF_MAX_ROWS; py++) {
            for (int px = x0; px <= x1 && px < GBUF_MAX_COLS; px++) {
                float b[3];
                barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
                if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f) continue;

                /* Interpolated NDC-z at this pixel */
                float z = b[0]*sz[0] + b[1]*sz[1] + b[2]*sz[2];
                if (z >= g_zbuf[py][px]) continue;  /* depth test: farther? skip */

                /* Depth + G-buffer write (frontmost surface wins) */
                g_zbuf  [py][px] = z;
                g_pos   [py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
                g_normal[py][px] = v3_norm(
                                   v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
                g_albedo[py][px] = albedo;  /* flat colour -- no lighting computed here */
                g_valid [py][px] = 1;
            }
        }
    }
}

/*
 * render_gbuffer -- GEOMETRY PASS: rasterize all scene objects into the G-buffer.
 *
 * This is the software equivalent of OpenGL's geometry pass:
 *
 *   glBindFramebuffer(GL_FRAMEBUFFER, gBuffer);            // bind G-buffer FBO
 *   glDrawBuffers(3, {GL_COLOR_ATTACHMENT0,1,2});          // MRT targets
 *   for each object:
 *       glUniformMatrix4fv(uMVP, ...);
 *       glDrawElements(GL_TRIANGLES, mesh.indexCount, ...); // renders to G-buffer
 *   glBindFramebuffer(GL_FRAMEBUFFER, 0);
 *
 * After this function returns, the G-buffer arrays hold the frontmost surface
 * data at every pixel -- INDEPENDENT of how many lights exist.  Adding lights
 * does NOT require re-running this function.
 */
static void render_gbuffer(
    const Mesh *meshes, const Vec3 *albedos,
    const Mat4 *models, int n_objects,
    const Mat4 *view, const Mat4 *proj,
    int cols, int rows)
{
    gbuffer_clear(cols, rows);

    for (int oi = 0; oi < n_objects; oi++) {
        Mat4 mv   = m4_mul(*view, models[oi]);     /* view space transform */
        Mat4 mvp  = m4_mul(*proj, mv);             /* full MVP for clip space */
        Mat4 nmat = m4_normal_mat(models[oi]);     /* normal transform (model only) */
        rasterize_object(&meshes[oi], albedos[oi], mvp, models[oi], nmat, cols, rows);
    }
}

/* ===================================================================== */
/* S7  lightpass  --  Blinn-Phong shading pass                            */
/* ===================================================================== */
/*
 * The LIGHTING PASS reads the G-buffer and applies shading.
 *
 * In OpenGL this is done by binding the G-buffer textures as uniforms, then
 * drawing a full-screen quad with a fragment shader that reads them:
 *
 *   glBindFramebuffer(GL_FRAMEBUFFER, 0);           // render to screen
 *   glBindTexture(GL_TEXTURE_2D, gPosition);        // bind G-buffer layers
 *   glBindTexture(GL_TEXTURE_2D, gNormal);
 *   glBindTexture(GL_TEXTURE_2D, gAlbedo);
 *   drawFullscreenQuad();  // fragment shader runs once per pixel
 *
 * In compute-shader engines (Unreal 5's Lumen, Unity HDRP tiled lighting):
 *   The screen is divided into tiles (e.g. 8×8 pixels).  Each tile finds
 *   which lights overlap it (frustum culling), then shades all pixels in
 *   the tile using only those lights -- skipping lights that don't reach.
 *   This is "tiled deferred shading", much faster for hundreds of lights.
 *
 * Our approach is the simplest possible: every pixel × every light.
 * Still a major win over forward rendering when objects outnumber lights.
 */

/*
 * PointLight -- a coloured point light source that orbits the scene.
 *
 * orbit_plane chooses which 2-D plane the light circles in:
 *   0 = XZ plane: horizontal ring around the floor -- good for ground light
 *   1 = YZ plane: vertical ring on the right -- cross-lights objects from side
 *   2 = XY plane: vertical ring on the front -- front-to-back illumination
 *
 * Each light's world-space pos is recomputed each tick from orbit_angle
 * so it is always correct regardless of animation state.
 */
typedef struct {
    Vec3  color;
    float orbit_radius;
    float orbit_speed;   /* radians per second: 0.4 = slow, 1.5 = fast */
    int   orbit_plane;   /* 0=XZ, 1=YZ, 2=XY                           */
    float height;        /* Y offset (XZ/XY plane) or Z offset (YZ plane) */
    float orbit_angle;   /* accumulated angle from orbit_speed * dt sum  */
    Vec3  pos;           /* world-space position computed each tick       */
} PointLight;

/*
 * blinn_phong -- compute one light's shading contribution at a G-buffer pixel.
 *
 * BLINN-PHONG MODEL -- the standard real-time shading equation:
 *
 *    Inputs at a pixel:
 *      P = world-space surface position  (from g_pos)
 *      N = unit surface normal           (from g_normal)
 *      albedo = flat surface colour      (from g_albedo)
 *      light_pos, light_col = point light data
 *      cam_pos = camera/eye position
 *
 *    Derived unit vectors:
 *      L = normalize(light_pos - P)   ← "light direction": surface toward light
 *      V = normalize(cam_pos   - P)   ← "view direction":  surface toward camera
 *      H = normalize(L + V)           ← "halfway vector":  bisects L and V
 *
 *    DIFFUSE (Lambertian):
 *      diff = max(0, dot(N, L))
 *      Represents: how much the surface faces the light source.
 *      Range: 0 (surface faces away) to 1 (surface perpendicular to light).
 *      This is why a sphere is brightest where the normal points at the light.
 *
 *    SPECULAR (Blinn):
 *      spec = max(0, dot(N, H)) ^ SHININESS
 *      Represents: how much the surface normal aligns with the halfway vector.
 *      When H == N, the reflection angle equals the view angle: maximum shine.
 *      SHININESS controls the falloff:
 *        low  = broad, soft highlight (rough surface)
 *        high = tight, sharp highlight (mirror surface)
 *
 *      WHY BLINN instead of Phong?
 *        Phong uses reflect(L, N) which requires the dot product cos(2θ).
 *        Blinn replaces it with H = normalize(L+V), eliminating an extra
 *        normalize per pixel.  The results are similar; Blinn was the
 *        standard in GPU hardware fixed-function pipelines (OpenGL 1.x).
 *
 *    TOTAL contribution from one light:
 *      result.rgb = albedo * light_col * diff  +  spec * 0.35
 *      (albedo modulates diffuse but not specular -- spec shines the
 *       light colour back, not the surface colour; 0.35 controls spec strength)
 *
 *    GLSL equivalent (what the lighting-pass fragment shader would contain):
 *      vec3 L    = normalize(light.pos - fragPos);
 *      vec3 V    = normalize(camPos    - fragPos);
 *      vec3 H    = normalize(L + V);
 *      float diff = max(dot(N, L), 0.0);
 *      float spec = pow(max(dot(N, H), 0.0), shininess);
 *      color += albedo * light.color * diff + spec * 0.35;
 */
static Vec3 blinn_phong(
    Vec3 P, Vec3 N, Vec3 albedo,
    Vec3 light_pos, Vec3 light_col,
    Vec3 cam_pos)
{
    Vec3 L = v3_norm(v3_sub(light_pos, P));  /* surface → light   */
    Vec3 V = v3_norm(v3_sub(cam_pos,   P));  /* surface → camera  */
    Vec3 H = v3_norm(v3_add(L, V));          /* halfway vector     */

    float diff = fmaxf(0.f, v3_dot(N, L));
    float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS);

    return v3(albedo.x * light_col.x * diff + spec * 0.35f,
              albedo.y * light_col.y * diff + spec * 0.35f,
              albedo.z * light_col.z * diff + spec * 0.35f);
}

/*
 * render_lightpass -- LIGHTING PASS: shade every G-buffer pixel.
 *
 * Reads only from the G-buffer arrays written by render_gbuffer.
 * Never touches mesh data -- the geometry has been reduced to per-pixel
 * surface properties, so adding or removing lights costs nothing extra here.
 *
 * Algorithm per pixel:
 *   1. If g_valid == 0 (no geometry here), output black and skip.
 *   2. Start with ambient = AMBIENT_STR × albedo
 *      (ambient = small uniform illumination so unlit faces aren't pitch black)
 *   3. For each active light: add blinn_phong contribution.
 *   4. Clamp result to [0,1] -- multiple lights can sum above 1 (HDR).
 *      (Real engines use tone-mapping here; we clamp for simplicity.)
 *   5. Store in g_light[r][c] for render_scene to display.
 *
 * This function IS the full-screen quad fragment shader in software.
 * Each iteration of the innermost loop is one "fragment invocation".
 */
static void render_lightpass(
    const PointLight *lights, int n_lights,
    Vec3 cam_pos, int cols, int rows)
{
    Vec3 ambient = v3(AMBIENT_STR, AMBIENT_STR, AMBIENT_STR * 1.1f);
    /* Slight blue tint on ambient mimics sky bounce light (common in engines) */

    for (int r = 0; r < rows && r < GBUF_MAX_ROWS; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_COLS; c++) {
            if (!g_valid[r][c]) { g_light[r][c] = v3(0, 0, 0); continue; }

            Vec3 P      = g_pos   [r][c];
            Vec3 N      = g_normal[r][c];
            Vec3 albedo = g_albedo[r][c];

            /* Ambient: uniform base illumination */
            Vec3 lit = v3(ambient.x * albedo.x,
                          ambient.y * albedo.y,
                          ambient.z * albedo.z);

            /* Accumulate each point light (Blinn-Phong per light) */
            for (int li = 0; li < n_lights; li++) {
                Vec3 contrib = blinn_phong(P, N, albedo,
                                           lights[li].pos, lights[li].color,
                                           cam_pos);
                lit.x += contrib.x;
                lit.y += contrib.y;
                lit.z += contrib.z;
            }

            /* Clamp to [0,1] -- sum of lights can exceed 1.0 (overexposure) */
            g_light[r][c] = v3(fminf(1.f, lit.x),
                               fminf(1.f, lit.y),
                               fminf(1.f, lit.z));
        }
    }
}

/* ===================================================================== */
/* S8  scene  +  render                                                   */
/* ===================================================================== */
/*
 * This section owns:
 *   - GBufMode enum: which G-buffer layer to display
 *   - Object struct: one renderable entity with transform and albedo
 *   - Scene struct: all mutable simulation state
 *   - scene_init / scene_tick: setup and per-frame simulation update
 *   - render_scene / render_overlay: ASCII display of G-buffer layers + HUD
 */

/*
 * GBufMode -- the currently displayed G-buffer layer.
 *
 * Cycling through modes with 'g' is a debugging tool used in game engines too:
 * Unity's "Scene View" render modes and Unreal's "Buffer Visualization" overlays
 * serve exactly this purpose -- inspect what each buffer contains independently.
 */
typedef enum {
    MODE_POSITION = 0,  /* world-depth coded character density + colour */
    MODE_NORMAL,        /* dominant surface normal axis → colour tint    */
    MODE_ALBEDO,        /* flat object colour, no lighting               */
    MODE_LIGHTING,      /* final Blinn-Phong shaded result               */
    MODE_COUNT
} GBufMode;

static const char *k_mode_names[MODE_COUNT] = {
    "POSITION", "NORMAL", "ALBEDO", "LIGHTING"
};
static const int k_mode_cp[MODE_COUNT] = {
    CP_MODE_POS, CP_MODE_NRM, CP_MODE_ALB, CP_MODE_LIT
};

/*
 * Object -- one renderable entity in the scene.
 *
 * angle_x / angle_y: accumulated rotation around each axis (radians)
 * spin_x  / spin_y:  rotation rate (radians per second)
 * position:          world-space translation of the mesh origin
 *
 * The model matrix is rebuilt each tick from angle + position:
 *   model = translate(position) × rotate_y(angle_y) × rotate_x(angle_x)
 *
 * Storing angles separately (not matrices) avoids floating-point drift that
 * would accumulate if we multiplied rotation matrices every frame.
 */
typedef struct {
    Mesh  mesh;
    Vec3  albedo;
    Vec3  position;
    float angle_x, angle_y;
    float spin_x,  spin_y;
} Object;

/*
 * LIGHT_PRESETS -- preset parameters for the six point lights.
 *
 * Lights are added one at a time with 'l'.  All six are pre-initialised from
 * this table at scene_init; only n_lights controls how many are active.
 *
 * Colour choices:
 *   Light 0 -- warm white (1.0, 0.95, 0.85): sun-like, the "key light"
 *   Light 1 -- cool blue  (0.4, 0.65, 1.0):  sky bounce, fills shadows
 *   Light 2 -- amber      (1.0, 0.70, 0.30): sunset / fire -- warm contrast
 *   Light 3 -- purple     (0.7, 0.30, 1.0):  neon / creative fill
 *   Light 4 -- green      (0.3, 1.00, 0.55): alien / underglow
 *   Light 5 -- coral      (1.0, 0.40, 0.40): warm accent
 *
 * The HUD LIGHTING row shows a coloured '@' swatch + name for each active
 * light so you can identify which colour is which while they orbit.
 */
static const struct {
    Vec3  color;
    float orbit_radius, orbit_speed, height;
    int   orbit_plane;
    float angle_start;
} LIGHT_PRESETS[MAX_LIGHTS] = {
    { {1.00f,0.95f,0.85f}, 2.6f, 0.55f, 1.8f, 0, 0.0f },  /* warm white, XZ ring */
    { {0.40f,0.65f,1.00f}, 2.3f, 0.90f, 0.0f, 1, 1.0f },  /* cool blue,  YZ ring */
    { {1.00f,0.70f,0.30f}, 2.9f, 0.40f, 1.5f, 0, 2.1f },  /* amber,      XZ ring */
    { {0.70f,0.30f,1.00f}, 2.2f, 1.20f, 2.0f, 2, 0.8f },  /* purple,     XY ring */
    { {0.30f,1.00f,0.55f}, 2.5f, 0.70f, 0.0f, 1, 3.5f },  /* green,      YZ ring */
    { {1.00f,0.40f,0.40f}, 3.1f, 0.50f, 2.2f, 0, 1.5f },  /* coral,      XZ ring */
};

/*
 * Scene -- all mutable simulation state, grouped so a single pointer
 * is all that is needed to pass the full state to any function.
 *
 * objects[]:  cube (index 0), sphere (index 1), floor (index 2)
 * lights[]:   all MAX_LIGHTS lights initialised from LIGHT_PRESETS;
 *             only n_lights of them are active in the render loop
 * models[]:   model matrices recomputed from object angles each tick
 * albedos[]:  flat copy of each object's albedo for render_gbuffer
 * meshes[]:   shallow copy of mesh struct header (verts/tris pointers)
 *
 * scene_cols = total terminal columns (scene render width)
 * scene_rows = total terminal rows minus HUD_ROWS (scene render height)
 */
typedef struct {
    Object      objects[MAX_OBJECTS];
    int         n_objects;
    PointLight  lights[MAX_LIGHTS];
    int         n_lights;

    Mat4  models [MAX_OBJECTS];   /* rebuilt by scene_tick each frame   */
    Vec3  albedos[MAX_OBJECTS];   /* copy of object albedo colours      */
    Mesh  meshes [MAX_OBJECTS];   /* shallow Mesh header copies         */

    GBufMode  mode;
    bool      paused;
    float     cam_dist;
    Mat4      view, proj;
    Vec3      cam_pos;
    int       scene_cols;
    int       scene_rows;   /* total_rows - HUD_ROWS */
} Scene;

/* Rebuild the perspective matrix after zoom or terminal resize */
static void scene_rebuild_proj(Scene *s, int cols, int rows)
{
    /*
     * Terminal cells are taller than they are wide (typically 8px wide, 16px tall).
     * Computing aspect from pixel counts rather than cell counts corrects for this.
     * Without CELL_W/CELL_H the cube appears squashed to half height.
     */
    float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
    s->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_init(Scene *s, int total_cols, int total_rows)
{
    /* Free any heap memory from a previous scene (reset / resize) */
    for (int i = 0; i < s->n_objects; i++) mesh_free(&s->objects[i].mesh);

    memset(s, 0, sizeof *s);
    s->scene_cols = total_cols;
    s->scene_rows = total_rows - HUD_ROWS;
    s->mode       = MODE_LIGHTING;   /* start with the most interesting view */
    s->cam_dist   = CAM_DIST_DEF;

    /* Camera: eye at (0, 0.5, cam_dist) looking at origin with Y up */
    s->cam_pos = v3(0.f, 0.5f, s->cam_dist);
    s->view    = m4_lookat(s->cam_pos, v3(0, 0, 0), v3(0, 1, 0));
    scene_rebuild_proj(s, total_cols, s->scene_rows);

    /* ---- Objects ------------------------------------------------------ */

    /*
     * CUBE: orange, placed left of center at X=-0.75.
     * Flat normals (each face one colour in NORMAL mode -- hard edge look).
     * Counter-rotating on both axes to show different faces over time.
     * Orange albedo (0.90, 0.52, 0.12) contrasts well with the blue sphere.
     */
    s->objects[0].mesh     = tessellate_cube();
    s->objects[0].albedo   = v3(0.90f, 0.52f, 0.12f);   /* warm orange */
    s->objects[0].position = v3(-0.75f, 0.f, 0.f);
    s->objects[0].spin_x   = 0.37f;
    s->objects[0].spin_y   = 0.55f;

    /*
     * SPHERE: blue, placed right of center at X=+0.75.
     * Smooth per-vertex normals (NORMAL mode shows soft colour gradient).
     * Negative spin_y gives opposite rotation to the cube -- visually varied.
     * Blue albedo (0.18, 0.48, 0.90) is complementary to the cube orange.
     */
    s->objects[1].mesh     = tessellate_sphere(SPHERE_RADIUS, SPHERE_RINGS, SPHERE_SEGS);
    s->objects[1].albedo   = v3(0.18f, 0.48f, 0.90f);   /* steel blue  */
    s->objects[1].position = v3( 0.75f, 0.f, 0.f);
    s->objects[1].spin_x   = 0.28f;
    s->objects[1].spin_y   = -0.44f;

    /*
     * FLOOR: neutral grey, static (spin=0), centred at origin.
     * Placed at Y=FLOOR_Y = -0.80, slightly below the cube/sphere bottoms.
     * Grey albedo (0.48, 0.48, 0.50) ensures light colour is clearly visible
     * on the floor without being dominated by a strong surface hue.
     */
    s->objects[2].mesh     = tessellate_floor(FLOOR_Y, FLOOR_HALF);
    s->objects[2].albedo   = v3(0.48f, 0.48f, 0.50f);   /* neutral grey */
    s->objects[2].position = v3(0.f, 0.f, 0.f);
    s->objects[2].spin_x   = 0.f;
    s->objects[2].spin_y   = 0.f;

    s->n_objects = 3;

    /* ---- Lights ------------------------------------------------------- */
    /*
     * Start with ONE light active; press 'l' to add more up to MAX_LIGHTS.
     * All six are initialised from LIGHT_PRESETS now so their orbit_angle
     * starts at the preset value and they don't all begin at the same position.
     */
    s->n_lights = 1;
    for (int li = 0; li < MAX_LIGHTS; li++) {
        PointLight *l   = &s->lights[li];
        l->color        = LIGHT_PRESETS[li].color;
        l->orbit_radius = LIGHT_PRESETS[li].orbit_radius;
        l->orbit_speed  = LIGHT_PRESETS[li].orbit_speed;
        l->height       = LIGHT_PRESETS[li].height;
        l->orbit_plane  = LIGHT_PRESETS[li].orbit_plane;
        l->orbit_angle  = LIGHT_PRESETS[li].angle_start;
    }
}

/*
 * scene_tick -- advance the simulation by dt seconds.
 *
 * Two things happen:
 *   1. Objects rotate: angle_x/y += spin_x/y * dt
 *   2. Lights orbit: orbit_angle += orbit_speed * dt
 *
 * Then the model matrices and light positions are rebuilt from the updated
 * angles.  Building from angles (not incrementing matrices) prevents drift.
 *
 * All updates skip if paused=true but model/light positions are still
 * recomputed from the frozen angles -- needed after resize.
 */
static void scene_tick(Scene *s, float dt)
{
    if (!s->paused) {
        for (int oi = 0; oi < s->n_objects; oi++) {
            s->objects[oi].angle_x += s->objects[oi].spin_x * dt;
            s->objects[oi].angle_y += s->objects[oi].spin_y * dt;
        }
        for (int li = 0; li < MAX_LIGHTS; li++)
            s->lights[li].orbit_angle += s->lights[li].orbit_speed * dt;
    }

    /* Rebuild model matrices: translate(pos) × rotate_y × rotate_x */
    for (int oi = 0; oi < s->n_objects; oi++) {
        Object *o    = &s->objects[oi];
        Mat4 rot     = m4_mul(m4_rotate_y(o->angle_y), m4_rotate_x(o->angle_x));
        Mat4 trans   = m4_translate(o->position);
        s->models [oi] = m4_mul(trans, rot);
        s->albedos[oi] = o->albedo;
        s->meshes [oi] = o->mesh;   /* Mesh is a small struct; safe to copy header */
    }

    /* Compute world-space light positions from orbit parameters */
    for (int li = 0; li < MAX_LIGHTS; li++) {
        PointLight *l = &s->lights[li];
        float ca = cosf(l->orbit_angle);
        float sa = sinf(l->orbit_angle);
        float r  = l->orbit_radius;
        float h  = l->height;
        switch (l->orbit_plane) {
        case 0: l->pos = v3(ca*r, h,    sa*r); break;  /* XZ: horizontal ring */
        case 1: l->pos = v3(h,    ca*r, sa*r); break;  /* YZ: right-side ring */
        case 2: l->pos = v3(ca*r, sa*r, h   ); break;  /* XY: front-side ring */
        default: break;
        }
    }
}

/* ---- display helpers (called by render_scene) ----------------------------- */

/*
 * normal_color_pair -- map a surface normal to the nearest cardinal-axis colour.
 *
 * Finds which of the three axes (X, Y, Z) has the largest absolute component,
 * then checks the sign of that component to pick the pair.
 *
 * Example reading: if the NORMAL buffer shows a bright-red region, the surface
 * at those pixels faces mostly in the +X direction.  Green = faces +Y (upward).
 * This is how surface orientation debugging works in game engines too
 * (Unity's "Buffer Visualization > World Space Normals").
 */
static int normal_color_pair(Vec3 n)
{
    float ax = fabsf(n.x), ay = fabsf(n.y), az = fabsf(n.z);
    if (ax >= ay && ax >= az) return n.x > 0.f ? CP_NRM_PX : CP_NRM_NX;
    if (ay >= ax && ay >= az) return n.y > 0.f ? CP_NRM_PY : CP_NRM_NY;
    return n.z > 0.f ? CP_NRM_PZ : CP_NRM_NZ;
}

/*
 * albedo_color_pair -- map flat surface colour to a ncurses pair for ALBEDO mode.
 *
 * Uses simple channel comparisons to identify orange (cube), blue (sphere),
 * or grey (floor) without storing object identity in the G-buffer.
 * (In a real engine you'd store a material ID in a G-buffer channel.)
 */
static int albedo_color_pair(Vec3 alb)
{
    if (alb.x > alb.z + 0.15f && alb.x > alb.y) return CP_OBJ_CUBE;    /* orange */
    if (alb.z > alb.x + 0.15f)                   return CP_OBJ_SPHERE;  /* blue   */
    return CP_OBJ_FLOOR;                                                  /* grey   */
}

/*
 * lighting_cell -- map a lit RGB pixel to an ASCII character + colour pair.
 *
 * Steps:
 *   1. Compute perceptual luminance from ITU-R BT.709 weights:
 *        luma = 0.2126*R + 0.7152*G + 0.0722*B
 *      These weights account for the human eye being most sensitive to green.
 *
 *   2. Add Bayer dither offset to soften quantization banding.
 *
 *   3. Map dithered luma to a Bourke ramp character (94 levels).
 *
 *   4. Map luma to one of 7 colour pairs (dark-red through white luma ramp).
 *
 *   5. Set bold=true for bright pixels so they appear extra-vivid.
 */
static void lighting_cell(Vec3 lit, int px, int py, char *ch_out, int *cp_out, bool *bold_out)
{
    float luma   = 0.2126f*lit.x + 0.7152f*lit.y + 0.0722f*lit.z;
    float dither = k_bayer[py & 3][px & 3];
    float d      = fmaxf(0.f, fminf(1.f, luma + (dither - 0.5f) * 0.15f));
    int   idx    = (int)(d * (float)(BOURKE_LEN - 1));
    int   cp     = CP_LIT_1 + (int)(d * 6.f);
    if (cp > CP_LIT_7) cp = CP_LIT_7;
    *ch_out   = k_bourke[idx];
    *cp_out   = cp;
    *bold_out = d > 0.6f;
}

/*
 * render_scene -- display the currently active G-buffer layer as ASCII art.
 *
 * Reads from the static G-buffer arrays (filled by render_gbuffer and
 * render_lightpass) and maps each pixel to an ASCII character + colour:
 *
 *   POSITION: depth → character density (close='#', far='.') + depth colour tint
 *   NORMAL:   dominant normal direction → one of six colour tints
 *   ALBEDO:   albedo luma → character density; channel hue → colour pair
 *   LIGHTING: Bourke ramp + Bayer dither; luma → 7-step colour pair
 *
 * Key insight to reinforce the deferred concept:
 *   In ALBEDO mode, press 'l' several times to add lights.
 *   The ALBEDO view does not change AT ALL -- it only reads g_albedo which
 *   was written by the geometry pass before any lights were considered.
 *   This proves the geometry pass is completely light-independent.
 */
static void render_scene(const Scene *s)
{
    int cols = s->scene_cols;
    int rows = s->scene_rows;

    for (int r = 0; r < rows && r < GBUF_MAX_ROWS; r++) {
        for (int c = 0; c < cols && c < GBUF_MAX_COLS; c++) {
            if (!g_valid[r][c]) continue;   /* sky / background -- leave blank */

            char ch;
            int  cp;
            bool bold;

            switch (s->mode) {

            case MODE_POSITION: {
                /*
                 * NDC-z range is (-1, +1) from near to far.
                 * Remap to t=[0,1] where 0=close and 1=far:
                 *   t = (z + 1) * 0.5
                 * Character density: '#' for very close, '.' for far away.
                 * Colour: cyan=near, green=mid, blue=far.
                 */
                float t = (g_zbuf[r][c] + 1.f) * 0.5f;
                cp   = (t < 0.35f) ? CP_NRM_NX : (t < 0.65f) ? CP_NRM_PY : CP_NRM_PZ;
                ch   = (t < 0.12f) ? '#' : (t < 0.28f) ? 'X' :
                       (t < 0.50f) ? 'o' : (t < 0.70f) ? '+' : '.';
                bold = t < 0.3f;
                break;
            }

            case MODE_NORMAL: {
                Vec3  n   = g_normal[r][c];
                float dom = fmaxf(fabsf(n.x), fmaxf(fabsf(n.y), fabsf(n.z)));
                cp   = normal_color_pair(n);
                /* Denser char for sharper-aligned normals (flat cube faces → '#') */
                ch   = dom > 0.85f ? '#' : dom > 0.55f ? 'o' : '+';
                bold = dom > 0.80f;
                break;
            }

            case MODE_ALBEDO: {
                Vec3  alb  = g_albedo[r][c];
                float luma = 0.2126f*alb.x + 0.7152f*alb.y + 0.0722f*alb.z;
                cp   = albedo_color_pair(alb);
                ch   = luma > 0.72f ? '#' : luma > 0.50f ? 'X' :
                       luma > 0.30f ? 'o' : '+';
                bold = luma > 0.65f;
                break;
            }

            case MODE_LIGHTING:
                lighting_cell(g_light[r][c], c, r, &ch, &cp, &bold);
                break;

            default:
                continue;
            }

            attr_t attr = COLOR_PAIR(cp) | (bold ? (attr_t)A_BOLD : 0u);
            attron(attr);
            mvaddch(r, c, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/*
 * light_color_pair -- map a light's RGB to the closest available ncurses pair.
 *
 * Priority order matters: check mixed colours (purple = r+b) BEFORE checking
 * single dominant channels, otherwise a purple light (r=0.7, b=1.0, g=0.3)
 * would be misidentified as "blue" by the dominant-channel test.
 *
 * Used by render_overlay to tint the "@" swatch for each active light.
 */
static int light_color_pair(Vec3 c)
{
    float r = c.x, g = c.y, b = c.z;
    /* Mixed hues first */
    if (r > 0.5f && b > 0.5f && g < 0.45f) return CP_NRM_NY;     /* purple/magenta */
    /* Single dominant channel */
    if (b > r + 0.2f && b > g + 0.2f)      return CP_NRM_PZ;     /* blue           */
    if (g > r + 0.2f && g > b + 0.2f)      return CP_NRM_PY;     /* green          */
    if (r > g + 0.2f && r > b + 0.2f)
        return (g > 0.55f) ? CP_LIT_2 : CP_NRM_PX;               /* orange / red   */
    /* Near-equal channels */
    if (r > 0.7f && g > 0.7f && b < 0.5f)  return CP_NRM_NZ;     /* yellow         */
    if (g > 0.7f && b > 0.7f && r < 0.5f)  return CP_NRM_NX;     /* cyan           */
    return CP_LIT_6;                                               /* white          */
}

/* Human-readable name for each light's colour (used in HUD light list) */
static const char *light_color_name(Vec3 c)
{
    switch (light_color_pair(c)) {
    case CP_NRM_NY: return "purple";
    case CP_NRM_PZ: return "blue";
    case CP_NRM_PY: return "green";
    case CP_LIT_2:  return "orange";
    case CP_NRM_PX: return "red";
    case CP_NRM_NZ: return "yellow";
    case CP_NRM_NX: return "cyan";
    default:        return "white";
    }
}

/*
 * render_overlay -- HUD rows drawn below the scene viewport.
 *
 * Row +0: title bar + key hints + fps counter
 * Row +1: separator line
 * Row +2: G-buffer mode selector (active mode highlighted, others dimmed)
 * Row +3: geometry pass stats (object count, triangle count)
 * Row +4: lighting pass -- coloured "@" swatch + colour name for each light
 *          followed by forward vs deferred draw-call comparison
 * Row +5: one-line explanation of what the current mode shows
 * Row +6: geometry pass algorithm summary
 * Row +7: lighting pass algorithm summary
 * Row +8: forward rendering comparison summary
 */
static void render_overlay(const Scene *s, double fps)
{
    int hr   = s->scene_rows;   /* first HUD row = one past the last scene row */
    int cols = s->scene_cols;

    int total_tris = 0;
    for (int i = 0; i < s->n_objects; i++) total_tris += s->objects[i].mesh.ntri;

    /* forward = one draw per object per light; deferred = one geometry + one light */
    int fwd_calls   = s->n_objects * s->n_lights;
    int defer_calls = s->n_objects + s->n_lights;

    /* Row +0: title */
    attron(COLOR_PAIR(CP_HUD_HDR) | A_BOLD);
    mvprintw(hr, 0,
        " Deferred Rendering Pipeline  [g=buffer | l=light | spc=pause | +/-=zoom | q=quit]"
        "  %4.1f fps", fps);
    attroff(COLOR_PAIR(CP_HUD_HDR) | A_BOLD);

    /* Row +1: separator */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hr+1, 0, " %.*s", cols - 2,
        "------------------------------------------------------------------------"
        "------------------------------------------------------------------------");
    attroff(COLOR_PAIR(CP_HUD));

    /* Row +2: G-buffer mode selector -- active mode is bold, others dim */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hr+2, 1, "G-Buffer view: ");
    attroff(COLOR_PAIR(CP_HUD));
    int cx = 16;
    for (int m = 0; m < MODE_COUNT; m++) {
        bool   active = (m == (int)s->mode);
        attr_t a = COLOR_PAIR(k_mode_cp[m]) | (active ? (attr_t)A_BOLD : (attr_t)A_DIM);
        attron(a);
        mvprintw(hr+2, cx, active ? "[%s]" : " %s ", k_mode_names[m]);
        attroff(a);
        cx += (int)strlen(k_mode_names[m]) + 3;
    }

    /* Row +3: geometry pass stats */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hr+3, 1, "Geometry pass: ");
    attroff(COLOR_PAIR(CP_HUD));
    attron(COLOR_PAIR(CP_HUD_VAL) | A_BOLD);
    mvprintw(hr+3, 16, "%d objects  %d triangles", s->n_objects, total_tris);
    attroff(COLOR_PAIR(CP_HUD_VAL) | A_BOLD);

    /*
     * Row +4: lighting pass -- one coloured "@" swatch per active light.
     *
     * The "@" character is rendered in the ncurses pair closest to the
     * light's actual RGB (via light_color_pair), immediately followed by
     * a dim colour-name label.  This lets you read the light colour from
     * the terminal even on terminals where position is hard to track.
     *
     * After all light swatches: the "fwd= def= saved=%" performance summary
     * which updates as you press 'l' to add more lights.
     */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hr+4, 1, "Lighting pass: ");
    attroff(COLOR_PAIR(CP_HUD));

    int lx = 16;
    for (int li = 0; li < s->n_lights && lx < cols - 20; li++) {
        Vec3        lc   = s->lights[li].color;
        int         cp   = light_color_pair(lc);
        const char *name = light_color_name(lc);

        /* Bold "@" swatch in the light's colour */
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvaddch(hr+4, lx, '@');
        attroff(COLOR_PAIR(cp) | A_BOLD);
        lx++;

        /* Dim colour name label */
        attron(COLOR_PAIR(cp) | A_DIM);
        mvprintw(hr+4, lx, "%s", name);
        attroff(COLOR_PAIR(cp) | A_DIM);
        lx += (int)strlen(name);

        /* Separator between lights */
        if (li < s->n_lights - 1) {
            attron(COLOR_PAIR(CP_HUD) | A_DIM);
            mvprintw(hr+4, lx, " | ");
            attroff(COLOR_PAIR(CP_HUD) | A_DIM);
            lx += 3;
        }
    }

    /* Forward vs deferred draw-call count comparison */
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(hr+4, lx + 2, "fwd=%d def=%d saved=%d%%",
             fwd_calls, defer_calls,
             fwd_calls > 0 ? 100 * (fwd_calls - defer_calls) / fwd_calls : 0);
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    /* Row +5: context-sensitive explanation of the current G-buffer mode */
    attron(COLOR_PAIR(CP_HUD_EXP));
    switch (s->mode) {
    case MODE_POSITION:
        mvprintw(hr+5, 1,
            "POSITION: depth-coded. Cyan=near, green=mid, blue=far."
            " '#' close, '.' far. Shows 3-D layout before any lighting.");
        break;
    case MODE_NORMAL:
        mvprintw(hr+5, 1,
            "NORMAL: +X=red, -X=cyan, +Y=green, -Y=magenta, +Z=blue, -Z=yellow."
            " Sphere smooth gradient; cube solid per-face colour.");
        break;
    case MODE_ALBEDO:
        mvprintw(hr+5, 1,
            "ALBEDO: flat surface colour written in geometry pass."
            " Press 'l' to add lights -- this view NEVER changes: geometry is light-agnostic.");
        break;
    case MODE_LIGHTING:
        mvprintw(hr+5, 1,
            "LIGHTING: Blinn-Phong from all active lights over G-buffer."
            " Press 'l' to add a light -- geometry pass does NOT re-run.");
        break;
    default: break;
    }
    attroff(COLOR_PAIR(CP_HUD_EXP));

    /* Rows +6..+8: algorithm summary (always visible) */
    attron(COLOR_PAIR(CP_HUD_EXP) | A_DIM);
    mvprintw(hr+6, 1,
        "Geometry pass: each object rasterized ONCE into G-buffer (pos/nrm/albedo). Cost: O(objects).");
    mvprintw(hr+7, 1,
        "Lighting pass: reads G-buffer pixels, applies Blinn-Phong per light. Cost: O(pixels x lights).");
    mvprintw(hr+8, 1,
        "Forward rendering: O(objects x lights). Deferred wins when many lights share the same geometry.");
    attroff(COLOR_PAIR(CP_HUD_EXP) | A_DIM);
}

/* ===================================================================== */
/* S9  app                                                                */
/* ===================================================================== */
/*
 * Application lifecycle: ncurses init, OS signal handlers, input, main loop.
 *
 * The App struct wraps the Scene and adds OS-level fields:
 *   running     -- set to 0 by SIGINT/SIGTERM to exit the main loop cleanly
 *   need_resize -- set to 1 by SIGWINCH; causes a full reinit next frame
 *
 * Signal safety: the signal handlers only write to volatile sig_atomic_t
 * variables -- the minimum safe operation from a signal context.  All real
 * work (endwin, getmaxyx, scene_init) happens in the main loop body.
 */

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
static void cleanup         (void)   { endwin(); }

static void screen_init(void)
{
    initscr();
    noecho();          /* don't echo key presses to the screen */
    cbreak();          /* read keys immediately, no line buffering */
    curs_set(0);       /* hide the text cursor */
    nodelay(stdscr, TRUE);   /* getch() returns ERR immediately if no key */
    keypad(stdscr, TRUE);    /* enable function keys and arrow keys */
    typeahead(-1);     /* disable typeahead detection (avoids screen flicker) */
    color_init();
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/*
 * app_do_resize -- handle a SIGWINCH terminal resize event.
 *
 * endwin()+refresh() forces ncurses to re-query the terminal size.
 * scene_init fully reinitialises the scene (meshes freed, reallocated, camera
 * rebuilt) for the new dimensions.  This resets animation state -- an
 * acceptable trade-off for simplicity.
 */
static void app_do_resize(App *app)
{
    endwin(); refresh();
    getmaxyx(stdscr, app->total_rows, app->total_cols);
    scene_init(&app->scene, app->total_cols, app->total_rows);
    app->need_resize = 0;
}

/*
 * app_handle_key -- process one key press.
 * Returns false if the application should exit, true to continue.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;  /* ESC = quit */
    case ' ':
        s->paused = !s->paused;
        break;
    case 'r': case 'R':
        scene_init(&app->scene, app->total_cols, app->total_rows);
        break;
    case 'g': case 'G':
        s->mode = (GBufMode)((s->mode + 1) % MODE_COUNT);
        break;
    case 'l': case 'L':
        /* Cycle from MAX_LIGHTS back to 1 so you can watch the algorithm reset */
        s->n_lights = (s->n_lights >= MAX_LIGHTS) ? 1 : s->n_lights + 1;
        break;
    case '=': case '+':
        s->cam_dist -= CAM_ZOOM_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        s->cam_pos = v3(0.f, 0.5f, s->cam_dist);
        s->view    = m4_lookat(s->cam_pos, v3(0,0,0), v3(0,1,0));
        break;
    case '-':
        s->cam_dist += CAM_ZOOM_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        s->cam_pos = v3(0.f, 0.5f, s->cam_dist);
        s->view    = m4_lookat(s->cam_pos, v3(0,0,0), v3(0,1,0));
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);    /* Ctrl-C */
    signal(SIGTERM,  on_exit_signal);    /* kill   */
    signal(SIGWINCH, on_resize_signal);  /* terminal resize */

    App *app  = &g_app;
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

        int64_t now    = clock_ns();
        int64_t dt     = now - frame_time;
        frame_time     = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;  /* clamp after pause/sleep */

        float dt_sec = (float)dt / (float)NS_PER_SEC;

        scene_tick(&app->scene, dt_sec);

        fps_cnt++;
        fps_acc += dt;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
            fps_cnt = 0; fps_acc = 0;
        }

        /* Frame cap: sleep any remaining budget so we don't spin the CPU */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

        /*
         * THE FULL DEFERRED RENDERING PIPELINE -- every frame:
         *
         *   Step 1 -- render_gbuffer:   geometry pass → writes G-buffer layers
         *             (only reads mesh data; lights are irrelevant here)
         *
         *   Step 2 -- render_lightpass: lighting pass → reads G-buffer, applies
         *             Blinn-Phong from every active light, writes g_light
         *
         *   Step 3 -- render_scene:     display the currently active G-buffer
         *             layer as ASCII characters with colour pairs
         *
         *   Step 4 -- render_overlay:   draw the HUD (stats, mode selector,
         *             algorithm explanation) below the scene rows
         */
        Scene *s = &app->scene;
        erase();
        render_gbuffer(s->meshes, s->albedos, s->models, s->n_objects,
                       &s->view, &s->proj, s->scene_cols, s->scene_rows);
        render_lightpass(s->lights, s->n_lights, s->cam_pos,
                         s->scene_cols, s->scene_rows);
        render_scene(s);
        render_overlay(s, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    for (int i = 0; i < app->scene.n_objects; i++)
        mesh_free(&app->scene.objects[i].mesh);

    endwin();
    return 0;
}
