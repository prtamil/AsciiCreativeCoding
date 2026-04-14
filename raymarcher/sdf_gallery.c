/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sdf_gallery.c  —  SDF Sculpture Gallery
 *
 * Five raymarched scenes showcasing SDF composition techniques:
 *   1  blend    animated smooth-union of two spheres + torus
 *   2  boolean  union / intersection / subtraction side-by-side
 *   3  twist    rounded-box with animated twist deformation
 *   4  repeat   domain-repeated sphere field
 *   5  sculpt   organic figure assembled from smin-blended primitives
 *
 * Keys:
 *   1-5     select preset scene
 *   t       cycle colour theme (5 themes)
 *   l       cycle lighting mode (N·V → Phong → Flat)
 *   p       pause / resume
 *   s       toggle soft shadows
 *   o       toggle ambient occlusion
 *   + / -   increase / decrease orbit speed
 *   r       reset camera
 *   q/ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/sdf_gallery.c \
 *       -o sdf_gallery -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config    — every tunable constant
 *   §2  clock     — monotonic clock
 *   §3  color     — 5 themes, palette init, pixel→char
 *   §4  math      — Vec3 inline ops
 *   §5  sdf       — primitives, boolean ops, smooth-union, twist, domain-rep,
 *                   5 scene functions + dispatcher
 *   §6  march     — normal, shadow, AO, ray march
 *   §7  shade     — 3-point lighting + cast_pixel
 *   §8  canvas    — framebuffer, progressive render, draw
 *   §9  app       — state, main loop, input, ncurses
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* canvas */
#define CANVAS_MAX_W  220
#define CANVAS_MAX_H   55
#define ROWS_PER_TICK   4
#define CELL_ASPECT   2.1f   /* terminal cell height / width                  */
#define FOV_HALF_TAN  0.57f  /* tan(30°): 60° vertical FOV                    */

/* march — tighter epsilon gives crisper edges; more steps handles complex scenes */
#define MARCH_MAX     120
#define MARCH_EPS    0.0015f
#define MARCH_FAR    18.0f
#define MARCH_STEP   0.85f   /* conservative: prevents over-stepping          */
#define MARCH_TW     0.60f   /* twist scene: DE is approximate, step smaller  */

/* normals, shadow, AO */
#define NORM_H       0.007f  /* smaller H → sharper surface detail            */
#define SH_STEPS       24
#define SH_K          12.0f
#define SH_FLOOR      0.12f  /* never fully-black shadows                     */
#define AO_STEPS        5
#define AO_STEP       0.12f
#define AO_DECAY      0.72f

/* lighting */
#define KA    0.10f   /* ambient floor: lower = more contrast dark↔lit        */
#define KD    0.72f   /* diffuse coefficient                                  */
#define KS    0.26f   /* specular coefficient                                 */
#define SHIN  24.0f   /* Phong shininess                                      */
#define FILL_STR  0.14f  /* fill light strength                               */
#define RIM_STR   0.09f  /* rim light strength                                */

/* camera defaults */
#define CAM_DIST_DEF   2.8f  /* closer camera → objects fill more screen     */
#define CAM_THETA_DEF  0.38f
#define CAM_PHI_DEF    0.0f
#define CAM_ORBIT_DEF  0.30f

/* color */
#define GRAD_N    8
#define N_THEMES  5
#define CP_BASE  20   /* first gradient color pair                            */
#define CP_HUD    1   /* HUD text pair                                        */

/*
 * 8-char density ramp — every step is visually distinct at arm's length.
 * Same set as mandelbulb_explorer: no ambiguous pairs (,:  ;!  |%).
 *   index 0 = ' ' background (no-hit)
 *   index 1 = '.' darkest hit
 *   index 7 = '@' brightest hit
 */
#define BOURKE_LEN  8
static const char k_bourke[BOURKE_LEN + 1] = " .:=+*#@";

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static double clock_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Five themes × GRAD_N=8 foreground xterm-256 color indices.
 *
 * Design rule: every step must be bright enough to read on a dark background.
 * Old themes ramped from near-black → bright; low gradient steps were invisible.
 * New themes vary HUE across the gradient while keeping every color fully lit —
 * minimum brightness is a saturated mid-tone, maximum is near-white.
 *
 * xterm-256 cube:  color = 16 + 36*r + 6*g + b   (r,g,b ∈ 0..5)
 * All entries below have at least one channel at 4 or 5 → always visible.
 */
static const short k_palette[N_THEMES][GRAD_N] = {
    /* 0  Studio:  gold → yellow → warm white  (all r=4..5) */
    {214, 220, 221, 226, 227, 228, 230, 231},
    /* 1  Ember:   red → orange → yellow fire  (r=5 throughout) */
    {196, 202, 203, 208, 209, 214, 220, 226},
    /* 2  Arctic:  cyan → sky → lavender → white */
    { 51,  87, 123, 159, 153, 189, 225, 231},
    /* 3  Toxic:   bright green → lime → yellow-green */
    { 46,  82, 118, 119, 154, 155, 190, 226},
    /* 4  Neon:    magenta → violet → pink → white */
    {201, 165, 171, 207, 177, 213, 219, 231},
};

static int g_theme        = 0;
static int g_color_offset = 0;   /* rotated each tick for palette animation */

static void colors_init(void)
{
    start_color();
    use_default_colors();

    /* HUD pair: bright white on default */
    init_pair(CP_HUD, 231, -1);

    /* gradient pairs: CP_BASE + theme*GRAD_N + step */
    for (int th = 0; th < N_THEMES; th++)
        for (int i = 0; i < GRAD_N; i++)
            init_pair((short)(CP_BASE + th * GRAD_N + i),
                      k_palette[th][i], -1);
}

/* Map a pixel's (luma, col) → (char, attr) using current theme + offset */
static void pixel_to_cell(float luma, float col,
                           char *ch_out, attr_t *attr_out)
{
    /*
     * Gamma encode before char selection (T.15).
     * Phong/N·V luma is computed in linear light space.  The Bourke ramp is
     * drawn for perceptual uniformity — each step looks equally spaced to the
     * eye.  Without gamma, linear values cluster in the top 2-3 chars (#, @)
     * and most of the ramp sits empty.  powf(luma, 0.45) redistributes values
     * so the full 8-step ramp is used and contrast is visually maximised.
     */
    float lg = powf(luma > 0.0f ? luma : 0.0f, 0.45f);
    int ri = (int)(lg * (float)(BOURKE_LEN - 1) + 0.5f);
    if (ri < 1)            ri = 1;
    if (ri >= BOURKE_LEN)  ri = BOURKE_LEN - 1;
    *ch_out = k_bourke[ri];

    /* col ∈ [0,1] → gradient index, shifted by animated color_offset */
    int gi = (int)(col * (float)(GRAD_N - 1) + 0.5f + g_color_offset) % GRAD_N;
    if (gi < 0) gi += GRAD_N;
    *attr_out = COLOR_PAIR(CP_BASE + g_theme * GRAD_N + gi) | A_BOLD;
}

/* ===================================================================== */
/* §4  math                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3(float x, float y, float z) { return (Vec3){x,y,z}; }
static inline Vec3  v3add(Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3sub(Vec3 a, Vec3 b)  { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3mul(Vec3 a, float s) { return v3(a.x*s,   a.y*s,   a.z*s);   }
static inline float v3dot(Vec3 a, Vec3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z;   }
static inline float v3len(Vec3 a)          { return sqrtf(v3dot(a,a));               }
static inline Vec3  v3neg(Vec3 a)          { return v3(-a.x,-a.y,-a.z);             }
static inline Vec3  v3cross(Vec3 a, Vec3 b)
{
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline Vec3 v3norm(Vec3 a)
{
    float l = v3len(a);
    return l > 1e-9f ? v3mul(a, 1.0f / l) : v3(0, 1, 0);
}

/* ===================================================================== */
/* §5  sdf — primitives, ops, scene functions                             */
/* ===================================================================== */

/* SDF result: d = signed distance, col ∈ [0,1] material color signal */
typedef struct { float d; float col; } SDF2;

/* ── Primitives ────────────────────────────────────────────────────── */

/* Sphere centered at origin, radius r */
static float sdSphere(Vec3 p, float r)
{
    return v3len(p) - r;
}

/* Infinite capsule: segment a→b, radius r */
static float sdCapsule(Vec3 p, Vec3 a, Vec3 b, float r)
{
    Vec3  ab = v3sub(b, a);
    Vec3  ap = v3sub(p, a);
    float t  = v3dot(ap, ab) / v3dot(ab, ab);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return v3len(v3sub(p, v3add(a, v3mul(ab, t)))) - r;
}

/* Torus in the xz plane: major radius R, tube radius r */
static float sdTorus(Vec3 p, float R, float r)
{
    float qx = sqrtf(p.x*p.x + p.z*p.z) - R;
    return sqrtf(qx*qx + p.y*p.y) - r;
}

/* Rounded box: half-extents b, corner radius r */
static float sdRoundBox(Vec3 p, Vec3 b, float r)
{
    float qx = fabsf(p.x) - b.x;
    float qy = fabsf(p.y) - b.y;
    float qz = fabsf(p.z) - b.z;
    float ex  = qx > 0.0f ? qx : 0.0f;
    float ey  = qy > 0.0f ? qy : 0.0f;
    float ez  = qz > 0.0f ? qz : 0.0f;
    float ins = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
    return sqrtf(ex*ex + ey*ey + ez*ez) + ins - r;
}

/* ── Boolean ops ────────────────────────────────────────────────────── */

/* Union (min), intersection (max), subtraction max(a,-b) are inlined
   at call sites for clarity.  Only smin needs a helper. */

/*
 * Smooth union (polynomial blend, Inigo Quilez):
 *   smin(a,b,k) blends the two fields over a region of width k.
 *   k=0 → hard min.  k=0.2 → noticeably smooth.  k=1.0 → very bulgy.
 */
static float smin(float a, float b, float k)
{
    if (k < 1e-6f) return a < b ? a : b;
    float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
    return fminf(a, b) - h * h * k * 0.25f;
}

/* ── Deformations ────────────────────────────────────────────────────── */

/* Twist: rotate p.xz by angle = p.y * k (applied in object space) */
static Vec3 twist(Vec3 p, float k)
{
    float angle = p.y * k;
    float c = cosf(angle), s = sinf(angle);
    return v3(c*p.x - s*p.z, p.y, s*p.x + c*p.z);
}

/* Domain repetition in xz (light must stay in original space) */
static Vec3 domain_rep_xz(Vec3 p, float cell)
{
    p.x -= cell * roundf(p.x / cell);
    p.z -= cell * roundf(p.z / cell);
    return p;
}

/* ── Scene functions ──────────────────────────────────────────────── */

/*
 * scene1_blend — animated smooth-union of sphere+sphere+torus.
 * Demonstrates: smin with time-varying k and separation.
 * col encodes radial distance from origin (shows blend transition).
 */
static SDF2 scene1_blend(Vec3 p, float t)
{
    float sep = 0.55f + 0.38f * cosf(t * 0.70f);
    float k   = 0.14f + 0.11f * sinf(t * 0.50f);

    float s1 = sdSphere(v3sub(p, v3(-sep,  0.0f, 0.0f)), 0.48f);
    float s2 = sdSphere(v3sub(p, v3( sep,  0.0f, 0.0f)), 0.48f);
    float to = sdTorus(p, 0.62f, 0.17f);

    float d = smin(smin(s1, s2, k), to, k * 0.75f);

    /* col: normalized radial distance 0 (center) → 1 (edge) */
    float r   = v3len(p) / 1.3f;
    float col = r > 1.0f ? 1.0f : (r < 0.0f ? 0.0f : r);
    return (SDF2){d, col};
}

/*
 * scene2_boolean — three columns, one per boolean operation.
 *   Left  (col≈0.15): union       = min(sphere, box)
 *   Center(col≈0.50): intersection= max(sphere, box)
 *   Right (col≈0.85): subtraction = max(sphere, -capsule)
 * All rotate slowly on the Y axis so the ops are visible from all sides.
 */
static SDF2 scene2_boolean(Vec3 p, float t)
{
    /* slow Y rotation applied to the whole scene */
    float ang = t * 0.28f;
    float c = cosf(ang), s = sinf(ang);
    Vec3  pr = v3(c*p.x - s*p.z, p.y, s*p.x + c*p.z);

    /* Left: union */
    Vec3  pl  = v3sub(pr, v3(-2.0f, 0.0f, 0.0f));
    float la  = sdSphere(pl, 0.60f);
    float lb  = sdRoundBox(v3sub(pl, v3(0.25f,0.25f,0.25f)),
                            v3(0.38f,0.38f,0.38f), 0.05f);
    float ld  = fminf(la, lb);

    /* Center: intersection */
    float ca  = sdSphere(pr, 0.65f);
    float cb  = sdRoundBox(pr, v3(0.46f,0.46f,0.46f), 0.05f);
    float cd  = fmaxf(ca, cb);

    /* Right: subtraction sphere minus capsule */
    Vec3  prr = v3sub(pr, v3(2.0f, 0.0f, 0.0f));
    float ra  = sdSphere(prr, 0.62f);
    float rb  = sdCapsule(prr, v3(0.0f,-0.75f,0.0f), v3(0.0f,0.75f,0.0f), 0.24f);
    float rd  = fmaxf(ra, -rb);

    /* pick closest and assign per-column color */
    float d;  float col;
    if (ld <= cd && ld <= rd) { d = ld; col = 0.15f; }
    else if (cd <= rd)        { d = cd; col = 0.50f; }
    else                      { d = rd; col = 0.85f; }

    return (SDF2){d, col};
}

/*
 * scene3_twist — rounded box with animated twist deformation.
 * Demonstrates: applying a pre-warp to p before evaluating an SDF.
 * NOTE: twist makes the DE inexact → use MARCH_TW conservative step.
 * col encodes vertical height so the twist gradient is visible.
 */
static SDF2 scene3_twist(Vec3 p, float t)
{
    float twist_k = 1.85f * sinf(t * 0.48f);
    Vec3  tp      = twist(p, twist_k);
    float d       = sdRoundBox(tp, v3(0.33f, 1.10f, 0.33f), 0.08f);

    float col = (p.y + 1.1f) / 2.2f;
    col = col > 1.0f ? 1.0f : (col < 0.0f ? 0.0f : col);
    return (SDF2){d, col};
}

/*
 * scene4_repeat — domain-repeated sphere lattice.
 * Demonstrates: one SDF repeated infinitely via fmod in xz.
 * Light stays in original space; col comes from original xz angle
 * so the field has a rainbow hue variation across the grid.
 */
static SDF2 scene4_repeat(Vec3 p, float t)
{
    (void)t;
    float cell = 2.2f;

    /* remember original angle for color before repeating */
    float ang  = atan2f(p.z, p.x) / (2.0f * (float)M_PI) + 0.5f;

    /* repeat in xz; keep y unrepeated (only one layer) */
    Vec3 pr = domain_rep_xz(p, cell);

    float d = sdSphere(pr, 0.72f);
    return (SDF2){d, ang};
}

/*
 * scene5_sculpt — organic figure assembled entirely from smin.
 * Demonstrates: how smooth-union can produce continuous organic surfaces
 * from completely distinct geometric primitives.
 * Rotates slowly so all four arms are visible.
 */
static SDF2 scene5_sculpt(Vec3 p, float t)
{
    /* slow Y rotation */
    float ang = t * 0.38f;
    float c = cosf(ang), s = sinf(ang);
    Vec3  pr = v3(c*p.x - s*p.z, p.y, s*p.x + c*p.z);

    float k  = 0.10f;
    float k2 = 0.06f;   /* tighter blend for belt + arms */

    /* body */
    float body = sdRoundBox(v3sub(pr, v3(0.0f, -0.35f, 0.0f)),
                             v3(0.40f, 0.44f, 0.30f), 0.14f);
    /* head */
    float head = sdSphere(v3sub(pr, v3(0.0f, 0.82f, 0.0f)), 0.27f);
    /* neck capsule */
    float neck = sdCapsule(pr, v3(0.0f, 0.20f, 0.0f),
                                v3(0.0f, 0.58f, 0.0f), 0.13f);
    /* belt torus */
    float belt = sdTorus(v3sub(pr, v3(0.0f, -0.62f, 0.0f)), 0.43f, 0.075f);
    /* four arms */
    float armL = sdCapsule(pr, v3(-0.42f,-0.10f, 0.0f),
                                v3(-0.86f,-0.52f, 0.0f), 0.10f);
    float armR = sdCapsule(pr, v3( 0.42f,-0.10f, 0.0f),
                                v3( 0.86f,-0.52f, 0.0f), 0.10f);
    float armF = sdCapsule(pr, v3( 0.0f,-0.10f,  0.40f),
                                v3( 0.0f,-0.52f,  0.82f), 0.09f);
    float armB = sdCapsule(pr, v3( 0.0f,-0.10f, -0.40f),
                                v3( 0.0f,-0.52f, -0.82f), 0.09f);

    float d = smin(body, head, k);
    d = smin(d, neck, k);
    d = smin(d, belt, k2);
    d = smin(d, armL, k);
    d = smin(d, armR, k);
    d = smin(d, armF, k);
    d = smin(d, armB, k);

    /* col: height maps to gradient from feet→head */
    float col = (pr.y + 1.0f) / 2.2f;
    col = col > 1.0f ? 1.0f : (col < 0.0f ? 0.0f : col);
    return (SDF2){d, col};
}

/* Dispatch by preset index (0-based) */
static SDF2 scene_map(int preset, Vec3 p, float t)
{
    switch (preset) {
    case 0: return scene1_blend(p, t);
    case 1: return scene2_boolean(p, t);
    case 2: return scene3_twist(p, t);
    case 3: return scene4_repeat(p, t);
    default: return scene5_sculpt(p, t);
    }
}

/* ===================================================================== */
/* §6  march — normal, shadow, AO, ray march                              */
/* ===================================================================== */

/*
 * Surface normal via 6-tap central differences (T.2).
 * H=0.010 averages over a patch ~one terminal cell wide so normals
 * are smooth rather than speckling at fractal-resolution bumps.
 */
static Vec3 sdf_normal(int preset, Vec3 p, float t)
{
    float h  = NORM_H;
    float dx = scene_map(preset, v3(p.x+h, p.y,   p.z  ), t).d
             - scene_map(preset, v3(p.x-h, p.y,   p.z  ), t).d;
    float dy = scene_map(preset, v3(p.x,   p.y+h, p.z  ), t).d
             - scene_map(preset, v3(p.x,   p.y-h, p.z  ), t).d;
    float dz = scene_map(preset, v3(p.x,   p.y,   p.z+h), t).d
             - scene_map(preset, v3(p.x,   p.y,   p.z-h), t).d;
    return v3norm(v3(dx, dy, dz));
}

/*
 * Soft shadow (T.8): march from surface point toward light.
 * Returns penumbra ratio ∈ [SH_FLOOR, 1].  SH_FLOOR prevents
 * fully-shadowed cells from collapsing to the space character.
 */
static float sdf_shadow(int preset, Vec3 ro, Vec3 rd, float t)
{
    float res = 1.0f;
    float tm  = 0.025f;
    for (int i = 0; i < SH_STEPS && tm < MARCH_FAR; i++) {
        float d = scene_map(preset, v3add(ro, v3mul(rd, tm)), t).d;
        if (d < MARCH_EPS) return SH_FLOOR;
        float r = SH_K * d / tm;
        if (r < res) res = r;
        tm += d;
    }
    return res < SH_FLOOR ? SH_FLOOR : res;
}

/*
 * Ambient occlusion: 5-step march along the normal.
 * Accumulated DE deficit indicates how much geometry is nearby.
 */
static float sdf_ao(int preset, Vec3 p, Vec3 N, float t)
{
    float occ = 0.0f, wt = 1.0f;
    for (int i = 1; i <= AO_STEPS; i++) {
        float dist = (float)i * AO_STEP;
        float d    = scene_map(preset, v3add(p, v3mul(N, dist)), t).d;
        occ += wt * (dist - d);
        wt  *= AO_DECAY;
    }
    float ao = 1.0f - 2.0f * occ;
    return ao < 0.0f ? 0.0f : (ao > 1.0f ? 1.0f : ao);
}

/*
 * Ray march: returns hit distance (≥0) or -1 on miss.
 * Uses conservative step MARCH_TW for preset 2 (twist) since the
 * twist-deformed DE is an approximation and can over-step.
 */
static float sdf_march(int preset, Vec3 ro, Vec3 rd, float t, float *col_out)
{
    float step_scale = (preset == 2) ? MARCH_TW : MARCH_STEP;
    float tm = 0.0f;
    for (int i = 0; i < MARCH_MAX; i++) {
        Vec3 p  = v3add(ro, v3mul(rd, tm));
        SDF2 s  = scene_map(preset, p, t);
        if (s.d < MARCH_EPS) { *col_out = s.col; return tm; }
        if (tm  > MARCH_FAR) break;
        tm += s.d * step_scale;
    }
    return -1.0f;
}

/* ===================================================================== */
/* §7  shade — 3-point lighting + cast_pixel                              */
/* ===================================================================== */

/*
 * Two shading modes selected by light_mode:
 *
 *   Phong (light_mode=1):
 *     3-point rig: orbiting key + fixed fill + back rim.
 *     Shadow and AO darken occluded regions.  KA floor keeps cavities visible.
 *
 *   N·V (light_mode=0, default):
 *     Brightness = N·V — how directly the surface faces the camera.
 *     No global light direction means no brightness gradient competing with
 *     the material color (col field).  Shape and depth become the primary
 *     visual signals instead of lighting.  AO still applied so cavities read.
 *
 *   Flat (light_mode=2):
 *     luma = 1.0 for every hit pixel.  No shadows, no AO, no directionality.
 *     Shape vanishes; only the theme color (col field) varies across the surface.
 *     Useful for judging color gradients without lighting distraction.
 */
static float shade_luma(int preset, Vec3 hit, Vec3 N, Vec3 V,
                         Vec3 key_L, Vec3 fill_L, Vec3 rim_L,
                         float t, bool do_shadow, bool do_ao, int light_mode)
{
    /* Flat mode: maximum brightness, no computation needed */
    if (light_mode == 2) return 1.0f;

    /* Ambient occlusion — shared by N·V and Phong */
    float ao = 1.0f;
    if (do_ao) ao = sdf_ao(preset, hit, N, t);

    if (light_mode == 0) {
        /* N·V mode: brightness from how directly surface faces camera */
        float ndv  = fmaxf(0.0f, v3dot(N, V));
        float luma = KA + KD * ndv * ao;
        return luma > 1.0f ? 1.0f : luma;
    }

    /* Phong mode (light_mode == 1) */
    float ndl_key  = fmaxf(0.0f, v3dot(N, key_L));
    float ndl_fill = fmaxf(0.0f, v3dot(N, fill_L));
    float ndl_rim  = fmaxf(0.0f, v3dot(N, rim_L));

    Vec3  R    = v3sub(v3mul(N, 2.0f * ndl_key), key_L);
    float spec = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);

    float sh = 1.0f;
    if (do_shadow) {
        Vec3 sro = v3add(hit, v3mul(N, 0.010f));
        sh = sdf_shadow(preset, sro, key_L, t);
    }

    float luma = KA
               + (KD * ndl_key + KS * spec) * sh * ao
               + FILL_STR * KD * ndl_fill * ao
               + RIM_STR  * KD * ndl_rim;
    return luma > 1.0f ? 1.0f : luma;
}

/* Result stored in framebuffer */
typedef struct {
    float luma;
    float col;
    bool  hit;
} Pixel;

static Pixel cast_pixel(int px, int py, int cw, int ch,
                          Vec3 cam_pos, Vec3 fwd, Vec3 right, Vec3 up,
                          int preset, float scene_t,
                          Vec3 key_L, Vec3 fill_L, Vec3 rim_L,
                          bool do_shadow, bool do_ao, int light_mode)
{
    Pixel out = {0.0f, 0.0f, false};

    float u      = ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
    float v      = -(((float)py + 0.5f) / (float)ch * 2.0f - 1.0f);
    /* T.14 aspect correction: terminal cells are ~2× taller than wide */
    float aspect = ((float)ch * CELL_ASPECT) / (float)cw;

    Vec3 rd = v3norm(v3add(
        v3add(v3mul(right, u * FOV_HALF_TAN),
              v3mul(up,    v * FOV_HALF_TAN * aspect)),
        fwd));

    float col_val = 0.0f;
    float hit_t   = sdf_march(preset, cam_pos, rd, scene_t, &col_val);
    if (hit_t < 0.0f) return out;

    out.hit = true;
    out.col = col_val;

    Vec3 hit = v3add(cam_pos, v3mul(rd, hit_t));
    Vec3 N   = sdf_normal(preset, hit, scene_t);
    Vec3 V   = v3norm(v3sub(cam_pos, hit));

    out.luma = shade_luma(preset, hit, N, V,
                           key_L, fill_L, rim_L,
                           scene_t, do_shadow, do_ao, light_mode);
    return out;
}

/* ===================================================================== */
/* §8  canvas — framebuffer, progressive render, draw                     */
/* ===================================================================== */

static Pixel g_fbuf  [CANVAS_MAX_H][CANVAS_MAX_W];
static Pixel g_stable[CANVAS_MAX_H][CANVAS_MAX_W];
static int   g_render_row = 0;
static bool  g_dirty      = true;

/* Camera snapshot frozen at row 0 so all rows share one viewpoint (T.11) */
static Vec3 g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up;

static void camera_basis(float cam_dist, float cam_theta, float cam_phi,
                           Vec3 *cam_pos, Vec3 *fwd, Vec3 *right, Vec3 *up)
{
    float ct = cosf(cam_theta), st = sinf(cam_theta);
    float cp = cosf(cam_phi),   sp = sinf(cam_phi);
    *cam_pos = v3mul(v3(ct*cp, st, ct*sp), cam_dist);
    *fwd     = v3norm(v3neg(*cam_pos));
    /* gimbal-lock guard: swap world-up when nearly aligned with fwd */
    Vec3 wup = (fabsf(v3dot(*fwd, v3(0,1,0))) > 0.99f) ? v3(0,0,1) : v3(0,1,0);
    *right   = v3norm(v3cross(*fwd, wup));
    *up      = v3cross(*right, *fwd);
}

/*
 * Render ROWS_PER_TICK rows into g_fbuf.
 * Returns true when a full frame is complete (g_stable updated).
 *
 * T.11 camera snapshot: cam_dist/theta/phi are captured into g_snap_* ONLY
 * when g_render_row == 0 (start of a new frame pass).  All rows in the same
 * pass share one frozen camera so auto-orbit mid-frame cannot shear geometry.
 */
static bool canvas_render_rows(int cw, int ch,
                                 int preset, float scene_t,
                                 float cam_dist, float cam_theta, float cam_phi,
                                 Vec3 key_L, Vec3 fill_L, Vec3 rim_L,
                                 bool do_shadow, bool do_ao, int light_mode)
{
    if (g_render_row == 0)
        camera_basis(cam_dist, cam_theta, cam_phi,
                     &g_snap_cam, &g_snap_fwd, &g_snap_right, &g_snap_up);

    for (int k = 0; k < ROWS_PER_TICK; k++) {
        int py = g_render_row;
        for (int px = 0; px < cw; px++) {
            g_fbuf[py][px] = cast_pixel(px, py, cw, ch,
                                         g_snap_cam,
                                         g_snap_fwd, g_snap_right, g_snap_up,
                                         preset, scene_t,
                                         key_L, fill_L, rim_L,
                                         do_shadow, do_ao, light_mode);
        }
        if (++g_render_row >= ch) {
            g_render_row = 0;
            /* T.10 stable buffer: save completed frame */
            memcpy(g_stable, g_fbuf, sizeof g_stable);
            return true;
        }
    }
    return false;
}

static void canvas_draw(int cw, int ch, int cols, int rows)
{
    int off_x = (cols - cw) / 2;
    int off_y = (rows - 1 - ch) / 2;   /* reserve 1 row for HUD */

    for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
        /* T.10 stable buffer: fresh rows from g_fbuf, pending from g_stable */
        Pixel *row_src = (vy < g_render_row) ? g_fbuf[vy] : g_stable[vy];
        for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
            Pixel *px = &row_src[vx];
            if (!px->hit) continue;

            char   ch_c;
            attr_t attr;
            pixel_to_cell(px->luma, px->col, &ch_c, &attr);

            int tx = off_x + vx;
            int ty = off_y + vy;
            if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1) continue;
            attron(attr);
            mvaddch(ty, tx, (chtype)(unsigned char)ch_c);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

static const char *k_preset_names[5] = {
    "1:Blend", "2:Boolean", "3:Twist", "4:Repeat", "5:Sculpt"
};
static const char *k_theme_names[N_THEMES] = {
    "Studio", "Ember", "Arctic", "Toxic", "Neon"
};

typedef struct {
    int   cw, ch;        /* canvas dimensions in characters */
    int   preset;        /* current scene index 0-4 */

    float cam_dist;
    float cam_theta;
    float cam_phi;
    float orbit_spd;

    bool  paused;
    bool  do_shadow;
    bool  do_ao;
    int   light_mode;    /* 0=N·V  1=Phong 3-point  2=Flat (luma=1)           */

    float scene_t;       /* time fed to scene functions (frozen when paused) */
    float color_phase;   /* accumulates for palette animation                */

    bool  quit;
} App;

static volatile sig_atomic_t g_resize = 0;
static void on_sigwinch(int sig) { (void)sig; g_resize = 1; }

static void app_init(App *a, int cols, int rows)
{
    memset(a, 0, sizeof *a);
    a->cw         = (cols > CANVAS_MAX_W) ? CANVAS_MAX_W : cols;
    a->ch         = (rows - 1 > CANVAS_MAX_H) ? CANVAS_MAX_H : rows - 1;
    a->preset     = 0;
    a->cam_dist   = CAM_DIST_DEF;
    a->cam_theta  = CAM_THETA_DEF;
    a->cam_phi    = CAM_PHI_DEF;
    a->orbit_spd  = CAM_ORBIT_DEF;
    a->do_ao       = true;
    a->do_shadow   = false;        /* off by default for speed; 's' to enable */
    a->light_mode = 0;           /* N·V mode by default: shapes read clearly */

    memset(g_fbuf,   0, sizeof g_fbuf);
    memset(g_stable, 0, sizeof g_stable);
    g_render_row = 0;
    g_dirty      = true;

    /* initialise snapshot so render_rows can use it immediately */
    camera_basis(a->cam_dist, a->cam_theta, a->cam_phi,
                 &g_snap_cam, &g_snap_fwd, &g_snap_right, &g_snap_up);
}

static void app_resize(App *a, int cols, int rows)
{
    a->cw = (cols > CANVAS_MAX_W) ? CANVAS_MAX_W : cols;
    a->ch = (rows - 1 > CANVAS_MAX_H) ? CANVAS_MAX_H : rows - 1;
    g_render_row = 0;
    g_dirty      = true;
}

static void app_handle_key(App *a, int ch)
{
    switch (ch) {
    case '1': case '2': case '3': case '4': case '5':
        if (a->preset != ch - '1') {
            a->preset    = ch - '1';
            g_dirty      = true;
            a->scene_t   = 0.0f;  /* fresh time for new preset */
        }
        break;
    case 't':
        g_theme = (g_theme + 1) % N_THEMES;
        break;
    case 'p': case ' ':
        a->paused = !a->paused;
        break;
    case 's':
        a->do_shadow = !a->do_shadow;
        g_dirty = true;
        break;
    case 'o':
        a->do_ao = !a->do_ao;
        g_dirty  = true;
        break;
    case 'l':
        a->light_mode = (a->light_mode + 1) % 3;   /* N·V → Phong → Flat → N·V */
        g_dirty       = true;
        break;
    case '+': case '=':
        a->orbit_spd += 0.05f;
        break;
    case '-':
        a->orbit_spd -= 0.05f;
        if (a->orbit_spd < 0.0f) a->orbit_spd = 0.0f;
        break;
    case 'r':
        a->cam_dist  = CAM_DIST_DEF;
        a->cam_theta = CAM_THETA_DEF;
        a->cam_phi   = CAM_PHI_DEF;
        a->orbit_spd = CAM_ORBIT_DEF;
        g_dirty      = true;
        break;
    case 'q': case 27:   /* ESC */
        a->quit = true;
        break;
    default:
        break;
    }
}

static void app_tick(App *a, float dt)
{
    if (a->paused) return;

    a->scene_t  += dt;
    a->cam_phi  += a->orbit_spd * dt;

    /* palette animation: shift color_offset slowly */
    a->color_phase += 0.4f * dt;
    g_color_offset  = (int)(a->color_phase) % GRAD_N;
}

static void draw_hud(const App *a, int cols, int rows)
{
    attron(COLOR_PAIR(CP_HUD));
    /* bottom row */
    mvprintw(rows - 1, 0,
             " Scene:%-8s  Theme:%-6s  Light:%-5s  Shadow:%-3s  AO:%-3s  "
             "Orbit:%.2f  [1-5 t l s o +- r p q]",
             k_preset_names[a->preset],
             k_theme_names[g_theme],
             a->light_mode == 1 ? "Phong" : (a->light_mode == 2 ? "Flat" : "N·V"),
             a->do_shadow ? "on" : "off",
             a->do_ao     ? "on" : "off",
             (double)a->orbit_spd);
    /* truncate to terminal width */
    clrtoeol();
    attroff(COLOR_PAIR(CP_HUD));
    (void)cols;
}

int main(void)
{
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

    while (!a.quit) {
        /* ── handle resize ─────────────────────────────────────────── */
        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            app_resize(&a, cols, rows);
        }

        /* ── input ─────────────────────────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR)
            app_handle_key(&a, ch);

        /* ── time step ─────────────────────────────────────────────── */
        double t_now = clock_now();
        float  dt    = (float)(t_now - t_prev);
        if (dt > 0.1f) dt = 0.1f;   /* cap on resume after pause/resize */
        t_prev = t_now;

        app_tick(&a, dt);

        /* ── 3-point light directions ──────────────────────────────── */
        /* Key light orbits in xz at a fixed elevation */
        float kang  = a.scene_t * 0.40f;
        Vec3  key_L = v3norm(v3( cosf(kang) * 0.70f, 0.65f, sinf(kang) * 0.70f));
        Vec3 fill_L = v3norm(v3(-1.5f,  0.50f, -1.2f));  /* fixed cool fill   */
        Vec3  rim_L = v3norm(v3( 0.0f, -0.40f, -1.0f));  /* back-edge rim     */

        /* ── progressive render ─────────────────────────────────────── */
        if (g_dirty) {
            g_render_row = 0;
            memset(g_fbuf, 0, sizeof(Pixel) * (size_t)a.ch * CANVAS_MAX_W);
            g_dirty = false;
        }

        canvas_render_rows(a.cw, a.ch,
                           a.preset, a.scene_t,
                           a.cam_dist, a.cam_theta, a.cam_phi,
                           key_L, fill_L, rim_L,
                           a.do_shadow, a.do_ao, a.light_mode);

        /* ── draw ──────────────────────────────────────────────────── */
        erase();
        canvas_draw(a.cw, a.ch, cols, rows);
        draw_hud(&a, cols, rows);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── frame cap: ~60 fps ─────────────────────────────────────── */
        struct timespec sl = {0, 16000000L};
        nanosleep(&sl, NULL);
    }

    endwin();
    return 0;
}
