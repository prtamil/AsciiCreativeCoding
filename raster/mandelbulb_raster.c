/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb_raster.c — Mandelbulb software rasterizer
 *
 * Approach: mesh the Mandelbulb surface ONCE at startup via sphere projection
 * (march each UV-sphere ray inward to the implicit surface), then rotate and
 * rasterize in real time through the same triangle pipeline as cube_raster.c.
 *
 * Unlike the raymarcher (one DE march per pixel per frame), the rasterizer:
 *   - Pays mesh cost ONCE at startup (<0.2 s) — ~1800 triangles
 *   - Projects triangles to screen via MVP transform + z-buffer per frame
 *   - Interpolates per-vertex attributes (smooth iter, normal) barycentrically
 *   - Runs real vertex + fragment shaders, not per-pixel ray math
 *
 * The tradeoff: inner concavities and back-facing pods are hidden (sphere
 * projection only captures the outermost surface skin).  That is the
 * fundamental difference between rasterisation and raymarching.
 *
 * 3 shaders (cycle with 's'):
 *   phong_hue  — Blinn-Phong lighting, smooth → HSV hue depth bands
 *   normals    — world normal azimuth → hue (orientation visualisation)
 *   depth_hue  — smooth → hue, no lighting  (direct compare to raymarcher)
 *
 * Keys:
 *   s / S     cycle shader
 *   c         toggle back-face culling
 *   p / P     power +1 / -1  (2..16, rebuilds mesh)
 *   b         cycle hue bands (1..5 rainbow cycles)
 *   + / =     zoom in
 *   -         zoom out
 *   space     pause / resume rotation
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/mandelbulb_raster.c \
 *       -o mandelbulb_raster -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  math        Vec3 Vec4 Mat4 + hsv_to_rgb
 *   §3  shaders     phong_hue / normals / depth_hue
 *   §4  mesh        Mandelbulb DE + sphere-projection tessellation
 *   §5  framebuffer zbuf cbuf color_to_cell blit
 *   §6  pipeline    vertex transform · rasterize · draw
 *   §7  scene
 *   §8  screen / HUD
 *   §9  app / main
 */

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
#include <float.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    FPS_TARGET    = 30,
    FPS_UPDATE_MS = 500,
};

/* Camera */
#define CAM_FOV       (50.0f * (float)M_PI / 180.0f)
#define CAM_NEAR      0.1f
#define CAM_FAR       100.0f
#define CAM_DIST      3.2f
#define CAM_DIST_MIN  1.4f
#define CAM_DIST_MAX  8.0f
#define CAM_ZOOM_STEP 0.2f

/* Rotation speed */
#define ROT_Y  0.45f   /* rad/s */
#define ROT_X  0.28f

/* Terminal cell aspect — same ratio as cube_raster.c */
#define CELL_W  8
#define CELL_H  16

/* Mandelbulb mesh: UV sphere grid resolution.
 * NLAT × NLON rays shot inward → vertices where they hit the surface. */
#define NLAT   28        /* latitude steps  (north–south)  */
#define NLON   56        /* longitude steps (east–west)    */

/* Mandelbulb DE parameters */
#define MB_BAIL        8.0f    /* escape radius                         */
#define MB_ITERS       48      /* DE iterations (mesh build only)       */
#define MB_AUX_ITERS   24      /* normal estimation (cheaper)           */
#define MB_HIT_EPS     0.003f  /* surface threshold                     */
#define MB_MARCH_STEPS 220     /* max steps per ray during tessellation */

/* Default fractal power */
#define MB_POWER_DEFAULT  8
#define MB_POWER_MIN      2
#define MB_POWER_MAX     16

/* Hue color pairs */
#define HUE_N    12          /* 12-step rainbow wheel (30° each)   */
#define CP_HUD  (HUE_N + 1)  /* bright white for HUD overlay        */

/* Default hue bands: how many full rainbow cycles span smooth [0,1] */
#define HUE_BANDS_DEFAULT  3
#define HUE_BANDS_MAX      5

/* Paul Bourke ASCII density ramp: space (dark) → '@' (bright) */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* Bayer 4×4 ordered dither — softens ramp banding */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* ===================================================================== */
/* §2  math                                                               */
/* ===================================================================== */

typedef struct { float x, y, z;    } Vec3;
typedef struct { float x, y, z, w; } Vec4;
typedef struct { float m[4][4];     } Mat4;

static inline Vec3  v3(float x,float y,float z){ return (Vec3){x,y,z}; }
static inline Vec3  v3_add(Vec3 a,Vec3 b)  { return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline Vec3  v3_sub(Vec3 a,Vec3 b)  { return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline Vec3  v3_scale(Vec3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline float v3_dot(Vec3 a,Vec3 b)  { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3_len(Vec3 a)         { return sqrtf(v3_dot(a,a)); }
static inline Vec3  v3_norm(Vec3 a){
    float l=v3_len(a); return l>1e-7f?v3_scale(a,1.f/l):v3(0,1,0);
}
static inline Vec3 v3_bary(Vec3 a,Vec3 b,Vec3 c,float u,float v,float w){
    return v3(u*a.x+v*b.x+w*c.x, u*a.y+v*b.y+w*c.y, u*a.z+v*b.z+w*c.z);
}
static inline Vec4 v4(float x,float y,float z,float w){ return (Vec4){x,y,z,w}; }

static inline Mat4 m4_identity(void){
    Mat4 m={{{0}}}; m.m[0][0]=m.m[1][1]=m.m[2][2]=m.m[3][3]=1.f; return m;
}
static inline Vec4 m4_mul_v4(Mat4 m,Vec4 v){
    return v4(m.m[0][0]*v.x+m.m[0][1]*v.y+m.m[0][2]*v.z+m.m[0][3]*v.w,
              m.m[1][0]*v.x+m.m[1][1]*v.y+m.m[1][2]*v.z+m.m[1][3]*v.w,
              m.m[2][0]*v.x+m.m[2][1]*v.y+m.m[2][2]*v.z+m.m[2][3]*v.w,
              m.m[3][0]*v.x+m.m[3][1]*v.y+m.m[3][2]*v.z+m.m[3][3]*v.w);
}
static inline Mat4 m4_mul(Mat4 a,Mat4 b){
    Mat4 r={{{0}}};
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++)
        r.m[i][j]+=a.m[i][k]*b.m[k][j];
    return r;
}
static inline Vec3 m4_pt (Mat4 m,Vec3 p){
    Vec4 r=m4_mul_v4(m,v4(p.x,p.y,p.z,1.f)); return v3(r.x,r.y,r.z);
}
static inline Vec3 m4_dir(Mat4 m,Vec3 d){
    Vec4 r=m4_mul_v4(m,v4(d.x,d.y,d.z,0.f)); return v3(r.x,r.y,r.z);
}
static Mat4 m4_rotate_y(float a){
    Mat4 m=m4_identity();
    m.m[0][0]= cosf(a); m.m[0][2]=sinf(a);
    m.m[2][0]=-sinf(a); m.m[2][2]=cosf(a);
    return m;
}
static Mat4 m4_rotate_x(float a){
    Mat4 m=m4_identity();
    m.m[1][1]= cosf(a); m.m[1][2]=-sinf(a);
    m.m[2][1]= sinf(a); m.m[2][2]= cosf(a);
    return m;
}
static Mat4 m4_perspective(float fovy,float aspect,float near,float far){
    Mat4 m={{{0}}};
    float f=1.f/tanf(fovy*.5f);
    m.m[0][0]=f/aspect; m.m[1][1]=f;
    m.m[2][2]=(far+near)/(near-far);
    m.m[2][3]=(2.f*far*near)/(near-far);
    m.m[3][2]=-1.f;
    return m;
}
static Mat4 m4_lookat(Vec3 eye,Vec3 at,Vec3 up){
    Vec3 f=v3_norm(v3_sub(at,eye));
    Vec3 r=v3_norm(v3(f.z*up.y-f.y*up.z, f.x*up.z-f.z*up.x, f.y*up.x-f.x*up.y));
    Vec3 u=v3(r.y*f.z-r.z*f.y, r.z*f.x-r.x*f.z, r.x*f.y-r.y*f.x);
    Mat4 m=m4_identity();
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-v3_dot(r,eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-v3_dot(u,eye);
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]= v3_dot(f,eye);
    return m;
}
static Mat4 m4_normal_mat(Mat4 m){
    Mat4 n=m4_identity();
    n.m[0][0]=m.m[1][1]*m.m[2][2]-m.m[1][2]*m.m[2][1];
    n.m[0][1]=m.m[1][2]*m.m[2][0]-m.m[1][0]*m.m[2][2];
    n.m[0][2]=m.m[1][0]*m.m[2][1]-m.m[1][1]*m.m[2][0];
    n.m[1][0]=m.m[0][2]*m.m[2][1]-m.m[0][1]*m.m[2][2];
    n.m[1][1]=m.m[0][0]*m.m[2][2]-m.m[0][2]*m.m[2][0];
    n.m[1][2]=m.m[0][1]*m.m[2][0]-m.m[0][0]*m.m[2][1];
    n.m[2][0]=m.m[0][1]*m.m[1][2]-m.m[0][2]*m.m[1][1];
    n.m[2][1]=m.m[0][2]*m.m[1][0]-m.m[0][0]*m.m[1][2];
    n.m[2][2]=m.m[0][0]*m.m[1][1]-m.m[0][1]*m.m[1][0];
    return n;
}

/*
 * hsv_to_rgb — convert HSV [0,1]³ to RGB [0,1]³.
 * h=0 → red, h=0.33 → green, h=0.66 → blue.
 * Used by all three fragment shaders to map smooth/normal → vivid colour.
 */
static Vec3 hsv_to_rgb(float h, float s, float v)
{
    h *= 6.0f;
    int   i = (int)h;
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    switch (i % 6) {
    case 0: return v3(v, t, p);
    case 1: return v3(q, v, p);
    case 2: return v3(p, v, t);
    case 3: return v3(p, q, v);
    case 4: return v3(t, p, v);
    default:return v3(v, p, q);
    }
}

/* ===================================================================== */
/* §3  shaders                                                            */
/* ===================================================================== */

typedef struct { Vec3 pos; Vec3 normal; float u, v; } VSIn;
typedef struct {
    Vec4  clip_pos;
    Vec3  world_pos, world_nrm;
    float u, v;
    float custom[4];
} VSOut;
typedef struct {
    Vec3  world_pos, world_nrm;
    float u, v;
    float custom[4];
    int   px, py;
} FSIn;
typedef struct { Vec3 color; bool discard; } FSOut;

typedef void (*VertShaderFn)(const VSIn*, VSOut*, const void*);
typedef void (*FragShaderFn)(const FSIn*, FSOut*, const void*);
typedef struct { VertShaderFn vert; FragShaderFn frag;
                 const void *vert_uni; const void *frag_uni; } ShaderProgram;

/* Uniforms passed to all shaders */
typedef struct {
    Mat4  model, view, proj, mvp, norm_mat;
    Vec3  light_pos, cam_pos;
    float shininess;
    float hue_bands;   /* how many rainbow cycles across smooth [0,1] */
} Uniforms;

/* ── vertex shader — same for all 3 pipelines ──────────────────────── */
static void vert_mb(const VSIn *in, VSOut *out, const void *u_)
{
    const Uniforms *u = (const Uniforms *)u_;
    out->clip_pos  = m4_mul_v4(u->mvp, v4(in->pos.x,in->pos.y,in->pos.z,1.f));
    out->world_pos = m4_pt (u->model,    in->pos);
    out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
    out->u = in->u; out->v = in->v;
    /* pass smooth iteration value through to the fragment stage */
    out->custom[0] = in->u;   /* u stores smooth at mesh build time */
    out->custom[1] = out->custom[2] = out->custom[3] = 0.f;
}

/* ── frag_phong_hue ────────────────────────────────────────────────── */
/*
 * Blinn-Phong lighting combined with HSV hue from smooth iteration.
 *
 * smooth [0,1] → hue via hue_bands full rainbow cycles.
 * Phong luma → HSV value (V).  Saturation fixed at 0.82.
 *
 * Effect: each concentric depth shell of the Mandelbulb has a different
 * hue, AND the lighting gives proper 3-D shape cues simultaneously.
 * The two signals (depth = hue, shape = brightness) are orthogonal —
 * unlike the raymarcher where they competed.
 */
static void frag_phong_hue(const FSIn *in, FSOut *out, const void *u_)
{
    const Uniforms *u = (const Uniforms *)u_;
    Vec3 N = v3_norm(in->world_nrm);
    Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
    Vec3 V = v3_norm(v3_sub(u->cam_pos,   in->world_pos));
    Vec3 H = v3_norm(v3_add(L, V));

    float diff = fmaxf(0.f, v3_dot(N, L));
    float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess);
    float luma = 0.10f + 0.72f * diff + 0.28f * spec;
    if (luma > 1.f) luma = 1.f;

    /* smooth → hue: hue_bands full cycles from outer shell to inner */
    float hue = fmodf(in->custom[0] * u->hue_bands, 1.0f);
    Vec3 c = hsv_to_rgb(hue, 0.82f, luma);

    /* gamma encode: Phong is linear, display is perceptual */
    out->color.x = powf(c.x, 1.f/2.2f);
    out->color.y = powf(c.y, 1.f/2.2f);
    out->color.z = powf(c.z, 1.f/2.2f);
    out->discard = false;
}

/* ── frag_normals ──────────────────────────────────────────────────── */
/*
 * Map world-space surface normal to hue via its azimuth angle.
 * Elevation modulates brightness.
 *
 * Effect: each face direction shows a distinct colour — rotating the
 * Mandelbulb creates a colour-wheel sweep that reveals the orientation
 * of every pod and concavity on the visible surface.
 */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_)
{
    (void)u_;
    Vec3 N = v3_norm(in->world_nrm);
    /* azimuth of the normal around Y axis → hue */
    float azimuth = atan2f(N.z, N.x) / (2.f * (float)M_PI) + 0.5f;
    float value   = 0.45f + 0.55f * (N.y * 0.5f + 0.5f);
    out->color  = hsv_to_rgb(azimuth, 0.80f, value);
    out->discard = false;
}

/* ── frag_depth_hue ────────────────────────────────────────────────── */
/*
 * Pure smooth → hue, no lighting at all.
 * Direct rasterised equivalent of mandelbulb_raymarcher.c depth mode:
 * both encode depth as rainbow hue, both use '.' as the character.
 * The visual difference between the two programs comes from mesh coverage
 * (rasteriser misses inner pods) not from the colour formula.
 */
static void frag_depth_hue(const FSIn *in, FSOut *out, const void *u_)
{
    const Uniforms *u = (const Uniforms *)u_;
    float hue = fmodf(in->custom[0] * u->hue_bands, 1.0f);
    out->color  = hsv_to_rgb(hue, 0.90f, 0.88f);
    out->discard = false;
}

typedef enum { SH_PHONG_HUE=0, SH_NORMALS, SH_DEPTH_HUE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = { "phong_hue", "normals  ", "depth_hue" };

/* ===================================================================== */
/* §4  mesh — Mandelbulb DE + sphere-projection tessellation             */
/* ===================================================================== */

typedef struct { Vec3 pos; Vec3 normal; float u, v; } Vertex;
typedef struct { int v[3]; } Triangle;
typedef struct { Vertex *verts; int nvert; Triangle *tris; int ntri; } Mesh;

static void mesh_free(Mesh *m){ free(m->verts); free(m->tris); *m=(Mesh){0}; }

/*
 * mb_de() — Mandelbulb distance estimator + smooth iteration count.
 *
 * Spherical power formula:
 *   z^p → r^p · (sin(pθ)cos(pφ), sin(pθ)sin(pφ), cos(pθ))
 * DE:   dr_{n+1} = p · r^(p-1) · dr + 1;   de = 0.5·log(r)·r/dr
 * Smooth: mu = esc_i + 1 − log(log(r)/log(bail)) / log(power)
 *
 * out_smooth may be NULL (used in the normal estimation loop).
 */
static float mb_de(Vec3 pos, float power, int max_iter, float *out_smooth)
{
    Vec3  z  = pos;
    float dr = 1.0f, r = 0.0f;
    int   esc = max_iter;

    for (int i = 0; i < max_iter; i++) {
        r = v3_len(z);
        if (r > MB_BAIL) { esc = i; break; }

        float theta = acosf(z.z / (r + 1e-8f));
        float phi   = atan2f(z.y, z.x);
        float rp    = powf(r, power);
        dr = rp / r * power * dr + 1.0f;

        float st = sinf(power * theta), ct = cosf(power * theta);
        float sp = sinf(power * phi),   cp = cosf(power * phi);
        z = v3_add(v3_scale(v3(st*cp, st*sp, ct), rp), pos);
    }

    if (out_smooth) {
        if (r > MB_BAIL && esc < max_iter) {
            float mu = (float)esc + 1.0f
                     - logf(logf(r) / logf(MB_BAIL)) / logf(power + 1e-6f);
            mu /= (float)max_iter;
            *out_smooth = mu < 0.f ? 0.f : (mu > 1.f ? 1.f : mu);
        } else {
            *out_smooth = 1.0f;
        }
    }

    if (r < 1e-7f) return 0.0f;
    return 0.5f * logf(r) * r / dr;
}

/*
 * mb_normal() — surface normal from DE gradient (central differences).
 * Uses MB_AUX_ITERS for speed — lower than full quality but fine for normals.
 */
static Vec3 mb_normal(Vec3 p, float power)
{
    const float H = 0.012f;
    float d0, d1;
    float dx = mb_de(v3(p.x+H,p.y,p.z), power, MB_AUX_ITERS, NULL)
             - mb_de(v3(p.x-H,p.y,p.z), power, MB_AUX_ITERS, NULL);
    float dy = mb_de(v3(p.x,p.y+H,p.z), power, MB_AUX_ITERS, NULL)
             - mb_de(v3(p.x,p.y-H,p.z), power, MB_AUX_ITERS, NULL);
    float dz = mb_de(v3(p.x,p.y,p.z+H), power, MB_AUX_ITERS, NULL)
             - mb_de(v3(p.x,p.y,p.z-H), power, MB_AUX_ITERS, NULL);
    (void)d0; (void)d1;
    return v3_norm(v3(dx, dy, dz));
}

/*
 * tessellate_mandelbulb() — generate a triangle mesh for the Mandelbulb.
 *
 * Strategy: sphere projection.
 *   1. Lay a NLAT × NLON UV grid over a unit sphere.
 *   2. For each grid point (lat θ, lon φ), shoot a ray from radius 1.5
 *      inward along direction (cos θ cos φ, sin θ, cos θ sin φ).
 *   3. March inward using the DE as a safe step: r -= DE * 0.85.
 *   4. On hit (DE < MB_HIT_EPS): record position, SDF-gradient normal,
 *      and smooth iteration count → vertex.
 *   5. On miss (r < 0 or max steps): mark invalid, skip adjacent quads.
 *   6. Build quads from valid neighbouring grid vertices → triangles.
 *
 * Limitation: only the outermost surface visible from each radial
 * direction is captured.  Inner pods and concave fold interiors are
 * hidden — that is the fundamental rasterisation vs raymarching trade-off.
 *
 * The vertex 'u' field carries the smooth iteration value so that
 * fragment shaders can colour by fractal depth.
 */
static Mesh tessellate_mandelbulb(float power)
{
    int max_v = NLAT * NLON;
    int max_t = (NLAT - 1) * NLON * 2;

    Vertex   *verts = malloc((size_t)max_v * sizeof(Vertex));
    Triangle *tris  = malloc((size_t)max_t * sizeof(Triangle));
    int      *vidx  = malloc((size_t)max_v * sizeof(int));  /* grid → vert idx or -1 */

    int nvert = 0, ntri = 0;
    float smooth_dummy;

    for (int i = 0; i < NLAT; i++) {
        for (int j = 0; j < NLON; j++) {
            /* UV sphere direction for this grid point */
            float theta = -(float)M_PI * 0.5f
                        + ((float)i / (float)(NLAT - 1)) * (float)M_PI;
            float phi   = ((float)j / (float)NLON) * 2.0f * (float)M_PI;

            float ct = cosf(theta);
            Vec3 dir = v3(ct * cosf(phi), sinf(theta), ct * sinf(phi));

            /* March inward from r=1.5 until surface hit or miss */
            float r   = 1.5f;
            bool  hit = false;
            Vec3  hit_pos = v3(0,0,0);
            float smooth  = 0.f;

            for (int step = 0; step < MB_MARCH_STEPS; step++) {
                Vec3  p = v3_scale(dir, r);
                float d = mb_de(p, power, MB_ITERS, NULL);

                if (d < MB_HIT_EPS) {
                    /* Refine once for smooth coloring */
                    mb_de(p, power, MB_ITERS, &smooth);
                    hit_pos = p;
                    hit = true;
                    break;
                }
                r -= fmaxf(d * 0.85f, 0.004f);
                if (r < 0.01f) break;
            }

            if (hit) {
                vidx[i * NLON + j] = nvert;
                verts[nvert].pos    = hit_pos;
                verts[nvert].normal = mb_normal(hit_pos, power);
                verts[nvert].u      = smooth;   /* smooth → hue in shaders */
                verts[nvert].v      = (float)j / (float)NLON;
                nvert++;
            } else {
                vidx[i * NLON + j] = -1;
            }
        }
    }

    /* Build triangles from valid neighbouring grid quads.
     * CCW winding: matches cube_raster.c convention (area > 0 = front face). */
    for (int i = 0; i < NLAT - 1; i++) {
        for (int j = 0; j < NLON; j++) {
            int j1 = (j + 1) % NLON;
            int v00 = vidx[ i      * NLON + j  ];
            int v01 = vidx[ i      * NLON + j1 ];
            int v10 = vidx[(i + 1) * NLON + j  ];
            int v11 = vidx[(i + 1) * NLON + j1 ];

            if (v00 >= 0 && v01 >= 0 && v11 >= 0)
                tris[ntri++] = (Triangle){{v00, v11, v01}};
            if (v00 >= 0 && v10 >= 0 && v11 >= 0)
                tris[ntri++] = (Triangle){{v00, v10, v11}};
        }
    }

    free(vidx);
    (void)smooth_dummy;

    return (Mesh){ verts, nvert, tris, ntri };
}

/* ===================================================================== */
/* §5  framebuffer                                                        */
/* ===================================================================== */

typedef struct { char ch; int color_pair; bool bold; } Cell;
typedef struct { float *zbuf; Cell *cbuf; int cols, rows; } Framebuffer;

static void fb_alloc(Framebuffer *fb, int cols, int rows){
    fb->cols = cols; fb->rows = rows;
    fb->zbuf = malloc((size_t)(cols * rows) * sizeof(float));
    fb->cbuf = malloc((size_t)(cols * rows) * sizeof(Cell));
}
static void fb_free(Framebuffer *fb){
    free(fb->zbuf); free(fb->cbuf); *fb = (Framebuffer){0};
}
static void fb_clear(Framebuffer *fb){
    for (int i = 0; i < fb->cols * fb->rows; i++) fb->zbuf[i] = FLT_MAX;
    memset(fb->cbuf, 0, (size_t)(fb->cols * fb->rows) * sizeof(Cell));
}

/*
 * 12-step hue wheel — same concept as mandelbulb_raymarcher.c but with
 * only 12 pairs (vs 30).  Enough to distinguish all hue bands the
 * fragment shader produces.
 */
static const int k_hue256[HUE_N] = {
    196, 202, 208, 226, 154, 46, 48, 51, 39, 21, 93, 201
/*  red  org  amb  yel  lim  grn teal cyn sky  blu vio  mag */
};
static const int k_hue8[HUE_N] = {
    COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN,
    COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA
};

static void color_init(void){
    start_color();
    use_default_colors();
    for (int i = 0; i < HUE_N; i++) {
        if (COLORS >= 256)
            init_pair(i + 1, k_hue256[i], COLOR_BLACK);
        else
            init_pair(i + 1, k_hue8[i], COLOR_BLACK);
    }
    if (COLORS >= 256) init_pair(CP_HUD, 255, -1);
    else               init_pair(CP_HUD, COLOR_WHITE, -1);
}

/*
 * rgb_to_pair() — map a fragment's RGB colour to the nearest hue pair.
 *
 * Extracts the hue angle of the output colour and maps it to one of the
 * 12 pairs.  This is the key difference from cube_raster's luma_to_cell:
 * the terminal colour represents the actual hue of the shaded surface,
 * not just a brightness gradient.
 */
static int rgb_to_pair(Vec3 c)
{
    float r = c.x, g = c.y, b = c.z;
    float cmax = r > g ? (r > b ? r : b) : (g > b ? g : b);
    float cmin = r < g ? (r < b ? r : b) : (g < b ? g : b);
    float delta = cmax - cmin;
    if (delta < 0.04f) return 1;   /* near-grey → red pair (any pair works) */

    float h;
    if      (cmax == r) h = fmodf((g - b) / delta, 6.0f);
    else if (cmax == g) h = (b - r) / delta + 2.0f;
    else                h = (r - g) / delta + 4.0f;
    if (h < 0.0f) h += 6.0f;
    h /= 6.0f;
    int idx = (int)(h * (float)HUE_N) % HUE_N;
    return idx + 1;
}

/*
 * color_to_cell() — convert fragment RGB to a terminal Cell.
 *
 * character  ← Bourke ramp from luma (brightness → ASCII density)
 * color pair ← rgb_to_pair (hue of the shaded surface)
 * bold       ← luma > 0.5  (full terminal colour intensity for bright surfaces)
 *
 * Dither with Bayer 4×4 to soften the 92-level Bourke ramp banding.
 */
static Cell color_to_cell(Vec3 color, int px, int py)
{
    float luma = 0.2126f*color.x + 0.7152f*color.y + 0.0722f*color.z;
    float d = luma + (k_bayer[py & 3][px & 3] - 0.5f) * 0.15f;
    d = d < 0.f ? 0.f : d > 1.f ? 1.f : d;
    char ch = k_bourke[(int)(d * (BOURKE_LEN - 1))];
    int  cp = rgb_to_pair(color);
    return (Cell){ ch, cp, luma > 0.5f };
}

static void fb_blit(const Framebuffer *fb){
    for (int y = 0; y < fb->rows; y++){
        for (int x = 0; x < fb->cols; x++){
            Cell c = fb->cbuf[y * fb->cols + x];
            if (!c.ch) continue;
            attr_t a = COLOR_PAIR(c.color_pair) | (c.bold ? A_BOLD : 0);
            attron(a); mvaddch(y, x, (chtype)(unsigned char)c.ch); attroff(a);
        }
    }
}

/* ===================================================================== */
/* §6  pipeline                                                           */
/* ===================================================================== */

static void barycentric(const float sx[3], const float sy[3],
                        float px, float py, float b[3])
{
    float d = (sy[1]-sy[2])*(sx[0]-sx[2]) + (sx[2]-sx[1])*(sy[0]-sy[2]);
    if (fabsf(d) < 1e-6f){ b[0]=b[1]=b[2]=-1.f; return; }
    b[0] = ((sy[1]-sy[2])*(px-sx[2]) + (sx[2]-sx[1])*(py-sy[2])) / d;
    b[1] = ((sy[2]-sy[0])*(px-sx[2]) + (sx[0]-sx[2])*(py-sy[2])) / d;
    b[2] = 1.f - b[0] - b[1];
}

/*
 * pipeline_draw_mesh() — vertex transform → rasterize → fragment shade.
 * Identical to cube_raster.c except is_wire is always false here.
 */
static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                                ShaderProgram *sh, bool cull_backface)
{
    int cols = fb->cols, rows = fb->rows;

    for (int ti = 0; ti < mesh->ntri; ti++){
        const Triangle *tri = &mesh->tris[ti];
        VSOut vo[3];

        for (int vi = 0; vi < 3; vi++){
            const Vertex *vtx = &mesh->verts[tri->v[vi]];
            VSIn in = { vtx->pos, vtx->normal, vtx->u, vtx->v };
            memset(&vo[vi], 0, sizeof vo[vi]);
            sh->vert(&in, &vo[vi], sh->vert_uni);
        }

        /* near-clip reject */
        if (vo[0].clip_pos.w < 0.001f &&
            vo[1].clip_pos.w < 0.001f &&
            vo[2].clip_pos.w < 0.001f) continue;

        /* perspective divide → screen coords */
        float sx[3], sy[3], sz[3];
        for (int vi = 0; vi < 3; vi++){
            float w = vo[vi].clip_pos.w; if (fabsf(w) < 1e-6f) w = 1e-6f;
            sx[vi] = ( vo[vi].clip_pos.x / w + 1.f) * 0.5f * (float)cols;
            sy[vi] = (-vo[vi].clip_pos.y / w + 1.f) * 0.5f * (float)rows;
            sz[vi] =   vo[vi].clip_pos.z / w;
        }

        /* back-face cull */
        float area = (sx[1]-sx[0])*(sy[2]-sy[0]) - (sx[2]-sx[0])*(sy[1]-sy[0]);
        if (cull_backface && area <= 0.f) continue;

        /* bounding box */
        int x0 = (int)fmaxf(0.f,      floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
        int x1 = (int)fminf(cols-1.f,  ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
        int y0 = (int)fmaxf(0.f,      floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
        int y1 = (int)fminf(rows-1.f,  ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

        for (int py = y0; py <= y1; py++){
            for (int px = x0; px <= x1; px++){
                float b[3];
                barycentric(sx, sy, px + 0.5f, py + 0.5f, b);
                if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f) continue;

                float z = b[0]*sz[0] + b[1]*sz[1] + b[2]*sz[2];
                int idx = py * cols + px;
                if (z >= fb->zbuf[idx]) continue;
                fb->zbuf[idx] = z;

                FSIn fsin;
                fsin.world_pos = v3_bary(vo[0].world_pos, vo[1].world_pos,
                                          vo[2].world_pos, b[0], b[1], b[2]);
                fsin.world_nrm = v3_norm(
                                 v3_bary(vo[0].world_nrm, vo[1].world_nrm,
                                          vo[2].world_nrm, b[0], b[1], b[2]));
                fsin.u  = b[0]*vo[0].u  + b[1]*vo[1].u  + b[2]*vo[2].u;
                fsin.v  = b[0]*vo[0].v  + b[1]*vo[1].v  + b[2]*vo[2].v;
                fsin.px = px; fsin.py = py;
                for (int ci = 0; ci < 4; ci++)
                    fsin.custom[ci] = b[0]*vo[0].custom[ci]
                                    + b[1]*vo[1].custom[ci]
                                    + b[2]*vo[2].custom[ci];

                FSOut fsout; fsout.discard = false;
                sh->frag(&fsin, &fsout, sh->frag_uni);
                if (fsout.discard) continue;

                fb->cbuf[idx] = color_to_cell(fsout.color, px, py);
            }
        }
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Mesh          mesh;
    float         angle_x, angle_y;
    float         cam_dist;
    bool          paused;
    bool          cull_backface;
    ShaderIdx     shade_idx;
    ShaderProgram shader;
    Uniforms      uni;
    float         power;
    int           hue_bands;
} Scene;

static void scene_build_shader(Scene *s)
{
    switch (s->shade_idx) {
    case SH_PHONG_HUE: s->shader = (ShaderProgram){ vert_mb, frag_phong_hue, &s->uni, &s->uni }; break;
    case SH_NORMALS:   s->shader = (ShaderProgram){ vert_mb, frag_normals,   &s->uni, &s->uni }; break;
    case SH_DEPTH_HUE: s->shader = (ShaderProgram){ vert_mb, frag_depth_hue, &s->uni, &s->uni }; break;
    default: break;
    }
}

static void scene_rebuild_mesh(Scene *s)
{
    mesh_free(&s->mesh);
    s->mesh = tessellate_mandelbulb(s->power);
}

static void scene_set_zoom(Scene *s)
{
    s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
    s->uni.view    = m4_lookat(s->uni.cam_pos, v3(0,0,0), v3(0,1,0));
}

static void scene_rebuild_proj(Scene *s, int cols, int rows)
{
    float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
    s->uni.proj  = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->power         = (float)MB_POWER_DEFAULT;
    s->shade_idx     = SH_PHONG_HUE;
    s->cam_dist      = CAM_DIST;
    s->cull_backface = true;
    s->hue_bands     = HUE_BANDS_DEFAULT;

    s->uni.light_pos  = v3(3.f, 4.f, 3.f);
    s->uni.shininess  = 56.f;
    s->uni.hue_bands  = (float)s->hue_bands;

    scene_set_zoom(s);
    scene_rebuild_proj(s, cols, rows);
    scene_build_shader(s);
    scene_rebuild_mesh(s);   /* tessellate — pays mesh cost once here */
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->angle_y += ROT_Y * dt;
    s->angle_x += ROT_X * dt;
    Mat4 ry = m4_rotate_y(s->angle_y);
    Mat4 rx = m4_rotate_x(s->angle_x);
    s->uni.model    = m4_mul(ry, rx);
    s->uni.mvp      = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
    s->uni.norm_mat = m4_normal_mat(s->uni.model);
}

static void scene_draw(Scene *s, Framebuffer *fb)
{
    fb_clear(fb);
    pipeline_draw_mesh(fb, &s->mesh, &s->shader, s->cull_backface);
    fb_blit(fb);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s){
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s){ (void)s; endwin(); }
static void screen_resize(Screen *s){ endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

static void screen_draw_hud(const Screen *sc, const Scene *s, double fps)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
             " Mandelbulb Raster  %.1f fps  tris:%d  power:%.0f  "
             "bands:%d  [%s]%s%s ",
             fps, s->mesh.ntri, s->power, s->hue_bands,
             k_shader_names[s->shade_idx],
             s->cull_backface ? "  cull" : "  show",
             s->paused ? "  PAUSED" : "");
    mvprintw(sc->rows - 1, 0,
             "s=shader  c=cull  p/P=power(rebuild)  b=bands  +/-=zoom  spc=pause  q=quit");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void screen_present(void){ wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    Framebuffer           fb;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit  (int sig){ (void)sig; g_app.running     = 0; }
static void on_resize(int sig){ (void)sig; g_app.need_resize = 1; }
static void cleanup  (void)   { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    fb_free(&app->fb);
    fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
    scene_rebuild_proj(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static int64_t clock_ns(void){
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns){
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                          .tv_nsec = (long)  (ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': s->paused = !s->paused; break;
    case 's': case 'S':
        s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
        scene_build_shader(s);
        break;
    case 'c': case 'C':
        s->cull_backface = !s->cull_backface;
        break;
    case 'p':
        s->power += 1.0f;
        if (s->power > (float)MB_POWER_MAX) s->power = (float)MB_POWER_MAX;
        scene_rebuild_mesh(s);
        break;
    case 'P':
        s->power -= 1.0f;
        if (s->power < (float)MB_POWER_MIN) s->power = (float)MB_POWER_MIN;
        scene_rebuild_mesh(s);
        break;
    case 'b': case 'B':
        s->hue_bands = s->hue_bands % HUE_BANDS_MAX + 1;
        s->uni.hue_bands = (float)s->hue_bands;
        break;
    case '=': case '+':
        s->cam_dist -= CAM_ZOOM_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        scene_set_zoom(s);
        break;
    case '-':
        s->cam_dist += CAM_ZOOM_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        scene_set_zoom(s);
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    fb_alloc(&app->fb, app->screen.cols, app->screen.rows);

    /* Mesh build happens here — show a "building..." message */
    mvprintw(app->screen.rows / 2, app->screen.cols / 2 - 10,
             "building mesh (power=%d)...", MB_POWER_DEFAULT);
    refresh();

    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time = clock_ns();
    int64_t fps_acc = 0;
    int     fps_cnt = 0;
    double  fps_disp = 0.0;
    const int64_t frame_ns = NS_PER_SEC / FPS_TARGET;

    while (app->running) {
        if (app->need_resize) { app_do_resize(app); frame_time = clock_ns(); }

        int64_t now  = clock_ns();
        int64_t dt   = now - frame_time;
        frame_time   = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;
        float dt_sec = (float)dt / (float)NS_PER_SEC;

        int key;
        while ((key = getch()) != ERR)
            if (!app_handle_key(app, key)) { app->running = 0; break; }
        if (!app->running) break;

        scene_tick(&app->scene, dt_sec);

        fps_cnt++; fps_acc += dt;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
            fps_cnt = 0; fps_acc = 0;
        }

        erase();
        scene_draw(&app->scene, &app->fb);
        screen_draw_hud(&app->screen, &app->scene, fps_disp);
        screen_present();

        clock_sleep_ns(frame_ns - (clock_ns() - now));
    }

    mesh_free(&app->scene.mesh);
    fb_free(&app->fb);
    screen_free(&app->screen);
    return 0;
}
