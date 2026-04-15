/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * displace_raster.c  —  ncurses software rasterizer, vertex displacement demo
 *
 * Primitive: UV sphere, tessellated once at init.
 * Vertex shader displaces each vertex along its normal by a time-varying
 * function — the mesh breathes, ripples, pulses, and spikes in real time.
 *
 * Four displacement modes cycled with  d:
 *   RIPPLE   concentric rings from equator — sin(time + r*freq)
 *   WAVE     diagonal travelling wave     — sin(time + x*f + y*f)
 *   PULSE    whole sphere breathes        — sin(time) * exp(-r*falloff)
 *   SPIKY    spiky ball                   — |sin(x*f)*sin(y*f)*sin(z*f)|
 *
 * Four shaders cycled with  s:
 *   phong / toon / normals / wireframe
 *
 * Normal recomputation — central difference method:
 *   The mesh normal is correct for the undisplaced sphere.
 *   After displacement the surface has moved, so the normal must be
 *   recomputed from the actual deformed surface geometry.
 *
 *   Method: sample the displacement function at two nearby points
 *   along each of two tangent directions (central difference), then
 *   reconstruct the displaced tangent vectors and take their cross product.
 *
 *   displace(pos) → scalar offset along normal
 *   eps = small step along tangent
 *
 *   d_u = displace(pos + eps*Tu) - displace(pos - eps*Tu)
 *   d_v = displace(pos + eps*Tv) - displace(pos - eps*Tv)
 *   T'  = Tu*(2*eps) + N*d_u        ← displaced tangent in u direction
 *   B'  = Tv*(2*eps) + N*d_v        ← displaced tangent in v direction
 *   N'  = normalize(cross(T', B'))  ← new surface normal
 *
 *   This matches the lighting to the actual deformed surface exactly,
 *   not the original sphere.  Without recomputation normals would point
 *   in the wrong direction and lighting would look completely wrong.
 *
 * Keys:
 *   d / D     cycle displacement mode  (ripple → wave → pulse → spiky)
 *   s / S     cycle shader             (phong → toon → normals → wire)
 *   c / C     toggle back-face culling
 *   + / =     zoom in
 *   -         zoom out
 *   space     pause / resume animation
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra displace_raster.c -o displace -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  math         Vec3 Vec4 Mat4
 *   §3  displacement  4 mode functions + tangent basis + normal recompute
 *   §4  shaders       VSIn VSOut FSIn FSOut + vert_displace + 3 frag shaders
 *   §5  mesh          UV sphere tessellation
 *   §6  framebuffer   zbuf cbuf dither paulbourke blit
 *   §7  pipeline      vertex transform · rasterize · draw
 *   §8  scene         uniforms · tick · draw · mode/shader swap
 *   §9  screen        ncurses init/resize/hud/present
 *   §10 app           dt loop · input · resize · cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Vertex displacement shader — modifies mesh geometry per-frame.
 *                  Unlike cube/sphere/torus_raster.c (fixed mesh), here the
 *                  vertex positions are re-calculated each frame by displacing
 *                  the base sphere vertices along their normals.
 *
 * Math           : Displacement: p' = p + N · f(p, t)  where f is the mode function.
 *                  RIPPLE: f = A·sin(ω·t + k·|p.xz|) — cylindrical wave from equator.
 *                  WAVE:   f = A·sin(ω·t + k·p.x + k·p.y) — diagonal plane wave.
 *                  PULSE:  f = A·sin(ω·t) · exp(−γ·|p.y|) — breathing along y-axis.
 *                  SPIKY:  f = |sin(kx·p.x)·sin(ky·p.y)·sin(kz·p.z)| — sharp spikes.
 *
 *                  Normal recomputation (central difference):
 *                    N_new = normalise(∂p'/∂u × ∂p'/∂v)
 *                  where ∂p'/∂u ≈ (displaced(u+ε,v) − displaced(u−ε,v)) / (2ε).
 *                  This ensures shading remains correct for the deformed surface.
 *
 * Performance    : O(N_verts) displacement per frame.  Normal recomputation is
 *                  the expensive step: 4 extra displacement evaluations per vertex.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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
    FPS_TARGET    = 60,
    FPS_UPDATE_MS = 500,
    HUD_COLS      = 52,

    /*
     * Tessellation resolution.
     * Higher = smoother base sphere, better displacement detail.
     * 48×32 gives 3072 triangles — fast enough at 60fps in a terminal,
     * detailed enough that the displacement waves look smooth.
     * Drop to 36×24 if fps is low on your terminal.
     */
    TESS_U = 48,
    TESS_V = 32,
};

#define CAM_FOV       (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR      0.1f
#define CAM_FAR       100.0f
#define CAM_DIST      3.2f
#define CAM_DIST_MIN  1.2f
#define CAM_DIST_MAX  8.0f
#define CAM_ZOOM_STEP 0.2f

#define SPHERE_R  1.0f

/* Rotation — slow tumble so displacement detail is visible */
#define ROT_Y  0.30f
#define ROT_X  0.12f

/* Wireframe threshold — sphere triangles are small, needs 0.09+ */
#define WIRE_THRESH  0.09f

/*
 * Central difference epsilon for normal recomputation.
 * Too small → floating point noise in the normal.
 * Too large → normal lags the actual surface curvature.
 * 0.03 * SPHERE_R is a good balance for all four displacement modes.
 */
#define CD_EPS  (0.03f * SPHERE_R)

/* Paul Bourke ramp */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};

#define CELL_W  8
#define CELL_H  16

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define PI  3.14159265f

/* ===================================================================== */
/* §2  math                                                               */
/* ===================================================================== */

typedef struct { float x,y,z;    } Vec3;
typedef struct { float x,y,z,w;  } Vec4;
typedef struct { float m[4][4];  } Mat4;

static inline Vec3  v3(float x,float y,float z){ return (Vec3){x,y,z}; }
static inline Vec3  v3_add(Vec3 a,Vec3 b){ return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline Vec3  v3_sub(Vec3 a,Vec3 b){ return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline Vec3  v3_scale(Vec3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline float v3_dot(Vec3 a,Vec3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3_len(Vec3 a){ return sqrtf(v3_dot(a,a)); }
static inline Vec3  v3_norm(Vec3 a){
    float l=v3_len(a); return l>1e-7f?v3_scale(a,1.f/l):v3(0,1,0);
}
static inline Vec3 v3_cross(Vec3 a,Vec3 b){
    return v3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
static inline Vec3 v3_bary(Vec3 a,Vec3 b,Vec3 c,float u,float v,float w){
    return v3(u*a.x+v*b.x+w*c.x, u*a.y+v*b.y+w*c.y, u*a.z+v*b.z+w*c.z);
}

static inline Vec4 v4(float x,float y,float z,float w){ return (Vec4){x,y,z,w}; }

static inline Mat4 m4_identity(void){
    Mat4 m={{{0}}}; m.m[0][0]=m.m[1][1]=m.m[2][2]=m.m[3][3]=1.f; return m;
}
static inline Vec4 m4_mul_v4(Mat4 m,Vec4 v){
    return v4(
        m.m[0][0]*v.x+m.m[0][1]*v.y+m.m[0][2]*v.z+m.m[0][3]*v.w,
        m.m[1][0]*v.x+m.m[1][1]*v.y+m.m[1][2]*v.z+m.m[1][3]*v.w,
        m.m[2][0]*v.x+m.m[2][1]*v.y+m.m[2][2]*v.z+m.m[2][3]*v.w,
        m.m[3][0]*v.x+m.m[3][1]*v.y+m.m[3][2]*v.z+m.m[3][3]*v.w);
}
static inline Mat4 m4_mul(Mat4 a,Mat4 b){
    Mat4 r={{{0}}};
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            for(int k=0;k<4;k++)
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
    m.m[0][0]=cosf(a); m.m[0][2]=sinf(a);
    m.m[2][0]=-sinf(a);m.m[2][2]=cosf(a);
    return m;
}
static Mat4 m4_rotate_x(float a){
    Mat4 m=m4_identity();
    m.m[1][1]=cosf(a); m.m[1][2]=-sinf(a);
    m.m[2][1]=sinf(a); m.m[2][2]=cosf(a);
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
    Vec3 r=v3_norm(v3(f.z*up.y-f.y*up.z,
                       f.x*up.z-f.z*up.x,
                       f.y*up.x-f.x*up.y));
    Vec3 u=v3(r.y*f.z-r.z*f.y,r.z*f.x-r.x*f.z,r.x*f.y-r.y*f.x);
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

/* ===================================================================== */
/* §3  displacement                                                       */
/* ===================================================================== */

/*
 * Displacement modes — each returns a scalar offset to apply along
 * the surface normal.  Positive = push outward, negative = push inward.
 *
 * All functions receive:
 *   pos   — point on unit sphere in model space
 *   time  — seconds since start
 *   amp   — amplitude scale (from uniforms)
 *   freq  — frequency scale (from uniforms)
 *
 * They are pure functions — same inputs always produce same output.
 * This is required for the central difference normal recomputation
 * to work correctly: we call them at pos±eps and the delta must be
 * a genuine derivative of the function, not a stateful value.
 */

typedef enum {
    DM_RIPPLE = 0,
    DM_WAVE,
    DM_PULSE,
    DM_SPIKY,
    DM_COUNT
} DispMode;

static const char *k_disp_names[] = { "ripple","wave","pulse","spiky" };

/*
 * displace_ripple — concentric rings radiating from the equator.
 *
 * r = distance from Y axis in XZ plane.
 * sin(time + r*freq) produces rings that travel inward over time.
 * Multiplied by (1 - |y|) to taper off at the poles — prevents
 * the poles from having large displacements that look broken.
 */
static float displace_ripple(Vec3 pos, float time, float amp, float freq)
{
    float r    = sqrtf(pos.x*pos.x + pos.z*pos.z);
    float taper= 1.f - fabsf(pos.y) * 0.6f;
    return sinf(time * 2.5f + r * freq) * amp * taper;
}

/*
 * displace_wave — diagonal travelling wave across the surface.
 *
 * Uses a combination of X and Y position as the wave phase, producing
 * a wave that travels diagonally across the sphere.
 * Adding 0.7*pos.z gives the wave a slight depth twist so it does
 * not look flat from any viewing angle.
 */
static float displace_wave(Vec3 pos, float time, float amp, float freq)
{
    float phase = pos.x * freq + pos.y * freq * 0.8f + pos.z * freq * 0.5f;
    return sinf(time * 2.0f + phase) * amp;
}

/*
 * displace_pulse — whole sphere breathes in and out.
 *
 * sin(time) gives a smooth oscillation at ~0.5 Hz.
 * Adding a secondary sin at 3x frequency gives the breath a slight
 * "catch" — it does not feel perfectly mechanical.
 * exp(-r * falloff) concentrates the pulse at the equator, making
 * the poles stable anchors that contrast the heaving equatorial band.
 */
static float displace_pulse(Vec3 pos, float time, float amp, float freq)
{
    float r       = sqrtf(pos.x*pos.x + pos.z*pos.z);
    float breathe = sinf(time * 1.5f) * 0.85f
                  + sinf(time * 4.5f) * 0.15f;
    float falloff = expf(-r * freq * 0.4f);
    return breathe * amp * falloff;
}

/*
 * displace_spiky — spiky ball driven by product of three sine waves.
 *
 * |sin(x*f) * sin(y*f) * sin(z*f)| produces spikes at positions where
 * all three waves are simultaneously at their peaks.
 * The time term slowly rotates the spike pattern so it animates.
 * powf(..., 0.6) softens the product so spikes have a smooth base
 * rather than a pinched needle tip.
 */
static float displace_spiky(Vec3 pos, float time, float amp, float freq)
{
    float f   = freq * 1.4f;
    float t   = time * 0.8f;
    float val = fabsf(sinf(pos.x*f + t)
                    * sinf(pos.y*f + t*0.7f)
                    * sinf(pos.z*f + t*1.3f));
    return powf(val, 0.6f) * amp;
}

/* Dispatch table — indexed by DispMode */
typedef float (*DispFn)(Vec3, float, float, float);
static const DispFn k_disp_fn[DM_COUNT] = {
    displace_ripple,
    displace_wave,
    displace_pulse,
    displace_spiky,
};

/*
 * make_tangent_basis — compute two orthogonal tangent vectors for a
 * point on the sphere given its outward normal.
 *
 * We need two vectors tangent to the sphere surface so we can step
 * along them for the central difference normal computation.
 *
 * Method: pick an arbitrary "up" vector that is not parallel to N,
 * then use cross products to build an orthonormal frame.
 *
 *   T = normalize(cross(N, up))     ← tangent
 *   B = cross(N, T)                 ← bitangent (already unit length)
 *
 * The choice of "up" does not matter for correctness — it only affects
 * the orientation of the tangent frame, not the resulting normal.
 * We avoid (0,1,0) when N is nearly vertical to prevent degenerate cross.
 */
static void make_tangent_basis(Vec3 N, Vec3 *T, Vec3 *B)
{
    Vec3 up = (fabsf(N.y) < 0.9f) ? v3(0,1,0) : v3(1,0,0);
    *T = v3_norm(v3_cross(up, N));
    *B = v3_cross(N, *T);              /* already unit since N and T are */
}

/*
 * displaced_normal — recompute surface normal after displacement.
 *
 * Central difference:
 *   Sample displacement at pos ± eps*T and pos ± eps*B.
 *   The delta d_t and d_b give how much the surface rises/falls
 *   as we move along each tangent direction.
 *
 *   Reconstruct displaced tangent vectors:
 *     T' = T*(2*eps) + N*d_t   ← actual direction along surface in T
 *     B' = B*(2*eps) + N*d_b   ← actual direction along surface in B
 *
 *   New normal = normalize(cross(T', B'))
 *
 * This works for any displacement function — no need to derive an
 * analytic gradient.  The eps=CD_EPS (~3% of radius) is small enough
 * to be a good local approximation for all four displacement modes.
 */
static Vec3 displaced_normal(Vec3 pos, Vec3 N,
                              DispFn fn, float time,
                              float amp, float freq)
{
    Vec3 T, B;
    make_tangent_basis(N, &T, &B);

    float eps = CD_EPS;

    /* sample displacement function at four nearby points */
    float d_tp = fn(v3_add(pos, v3_scale(T,  eps)), time, amp, freq);
    float d_tm = fn(v3_add(pos, v3_scale(T, -eps)), time, amp, freq);
    float d_bp = fn(v3_add(pos, v3_scale(B,  eps)), time, amp, freq);
    float d_bm = fn(v3_add(pos, v3_scale(B, -eps)), time, amp, freq);

    float d_t = d_tp - d_tm;   /* central difference along T */
    float d_b = d_bp - d_bm;   /* central difference along B */

    /* displaced tangent vectors */
    Vec3 Td = v3_add(v3_scale(T, 2.f*eps), v3_scale(N, d_t));
    Vec3 Bd = v3_add(v3_scale(B, 2.f*eps), v3_scale(N, d_b));

    return v3_norm(v3_cross(Td, Bd));
}

/* ===================================================================== */
/* §4  shaders                                                            */
/* ===================================================================== */

typedef struct { Vec3 pos; Vec3 normal; float u,v; } VSIn;

typedef struct {
    Vec4  clip_pos;
    Vec3  world_pos;
    Vec3  world_nrm;
    float u,v;
    float custom[4];
} VSOut;

typedef struct {
    Vec3  world_pos;
    Vec3  world_nrm;
    float u,v;
    float custom[4];
    int   px,py;
} FSIn;

typedef struct { Vec3 color; bool discard; } FSOut;

typedef void (*VertShaderFn)(const VSIn*, VSOut*, const void*);
typedef void (*FragShaderFn)(const FSIn*, FSOut*, const void*);

/*
 * ShaderProgram — separate uniform pointers for vertex and fragment.
 *
 * WHY SPLIT:
 *   vert_displace always needs DisplaceUniforms (disp_fn, time, amp, freq).
 *   frag_toon needs ToonUniforms (bands).  These are different structs.
 *   A single void* uniforms cannot satisfy both simultaneously without
 *   casting to the wrong type — which is exactly what caused the crash.
 *   Separate pointers cost one extra pointer and fix the entire problem.
 */
typedef struct {
    VertShaderFn  vert;
    FragShaderFn  frag;
    const void   *vert_uni;   /* passed to vert() — always &scene.disp_uni */
    const void   *frag_uni;   /* passed to frag() — &disp_uni or &toon_uni */
} ShaderProgram;

/* ── uniforms ─────────────────────────────────────────────────────── */

typedef struct {
    Mat4  model, view, proj, mvp, norm_mat;
    Vec3  light_pos, light_col, ambient, cam_pos, obj_color;
    float shininess;
} Uniforms;

typedef struct { Uniforms base; int bands; } ToonUniforms;

/*
 * DisplaceUniforms — extends Uniforms with displacement parameters.
 *
 * Leading with Uniforms base so &disp_uni casts cleanly to
 * const Uniforms* inside vert_default and all fragment shaders.
 *
 * disp_fn    — pointer to active displacement function
 * time       — seconds since start, updated every frame
 * amplitude  — displacement magnitude (fraction of sphere radius)
 * frequency  — spatial frequency of the wave pattern
 * mode       — active DispMode (for display only)
 */
typedef struct {
    Uniforms base;
    DispFn   disp_fn;
    float    time;
    float    amplitude;
    float    frequency;
    DispMode mode;
} DisplaceUniforms;

/* ── vertex shaders ──────────────────────────────────────────────── */

/*
 * vert_displace — the core of this demo.
 *
 * Steps:
 *   1. Get sphere normal (normalised position for unit sphere).
 *   2. Compute displacement scalar d = disp_fn(pos, time, amp, freq).
 *   3. Move position along normal: displaced_pos = pos + N*d
 *   4. Recompute surface normal at displaced position via central diff.
 *   5. Transform to clip space using the displaced position.
 *   6. Transform new normal to world space for lighting.
 *
 * The fragment shaders receive world_pos and world_nrm from the
 * displaced surface — they do not know or care about the original sphere.
 * All four fragment shaders (phong, toon, normals, wire) work unchanged.
 */
static void vert_displace(const VSIn *in, VSOut *out, const void *u_)
{
    const DisplaceUniforms *du = (const DisplaceUniforms*)u_;
    const Uniforms         *u  = &du->base;

    /* sphere normal == normalised position for unit sphere */
    Vec3 N   = v3_norm(in->pos);

    /* scalar displacement along normal */
    float d  = du->disp_fn(in->pos, du->time, du->amplitude, du->frequency);

    /* displaced model-space position */
    Vec3 dpos = v3_add(in->pos, v3_scale(N, d));

    /* recompute normal for the deformed surface */
    Vec3 dnrm = displaced_normal(in->pos, N,
                                  du->disp_fn, du->time,
                                  du->amplitude, du->frequency);

    /* clip space from displaced position */
    out->clip_pos  = m4_mul_v4(u->mvp, v4(dpos.x,dpos.y,dpos.z,1.f));

    /* world space for fragment lighting */
    out->world_pos = m4_pt (u->model,    dpos);
    out->world_nrm = v3_norm(m4_dir(u->norm_mat, dnrm));

    out->u = in->u;
    out->v = in->v;
    out->custom[0]=out->custom[1]=out->custom[2]=out->custom[3]=0.f;
}

/*
 * vert_displace_normals — same as vert_displace but additionally
 * packs world normal into custom[0..2] for frag_normals.
 */
static void vert_displace_normals(const VSIn *in, VSOut *out, const void *u_)
{
    vert_displace(in, out, u_);
    out->custom[0] = out->world_nrm.x;
    out->custom[1] = out->world_nrm.y;
    out->custom[2] = out->world_nrm.z;
}

/*
 * vert_displace_wire — same as vert_displace but leaves custom[0..2]
 * free for the pipeline to inject barycentric coords (wireframe).
 */
static void vert_displace_wire(const VSIn *in, VSOut *out, const void *u_)
{
    vert_displace(in, out, u_);
    /* custom[] will be overwritten by pipeline for wireframe */
}

/* ── fragment shaders ────────────────────────────────────────────── */

/*
 * frag_phong — Blinn-Phong + gamma.
 * On the displaced sphere the highlight dances across the waves
 * and spikes, making the deformation clearly visible even on the
 * lit side of the sphere.
 */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_)
{
    const Uniforms *u=(const Uniforms*)u_;
    Vec3 N=v3_norm(in->world_nrm);
    Vec3 L=v3_norm(v3_sub(u->light_pos, in->world_pos));
    Vec3 V=v3_norm(v3_sub(u->cam_pos,   in->world_pos));
    Vec3 H=v3_norm(v3_add(L,V));
    float diff=fmaxf(0.f,v3_dot(N,L));
    float spec=powf(fmaxf(0.f,v3_dot(N,H)), u->shininess);
    Vec3 c=u->obj_color;
    float r=u->ambient.x+c.x*u->light_col.x*diff+spec*0.5f;
    float g=u->ambient.y+c.y*u->light_col.y*diff+spec*0.5f;
    float b=u->ambient.z+c.z*u->light_col.z*diff+spec*0.5f;
    out->color.x=powf(fminf(r,1.f),1.f/2.2f);
    out->color.y=powf(fminf(g,1.f),1.f/2.2f);
    out->color.z=powf(fminf(b,1.f),1.f/2.2f);
    out->discard=false;
}

/*
 * frag_toon — 4-band quantised Phong.
 * On the displaced sphere the band boundaries follow the wave
 * crests and troughs — the toon shading reacts to the geometry,
 * not just the overall sphere curvature.
 */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_)
{
    const ToonUniforms *tu=(const ToonUniforms*)u_;
    const Uniforms     *u =&tu->base;
    Vec3 N=v3_norm(in->world_nrm);
    Vec3 L=v3_norm(v3_sub(u->light_pos, in->world_pos));
    Vec3 V=v3_norm(v3_sub(u->cam_pos,   in->world_pos));
    Vec3 H=v3_norm(v3_add(L,V));
    float diff  =fmaxf(0.f,v3_dot(N,L));
    float banded=floorf(diff*(float)tu->bands)/(float)tu->bands;
    float spec  =(v3_dot(N,H)>0.94f)?0.7f:0.f;
    Vec3 c=u->obj_color;
    out->color.x=fminf(c.x*(banded+0.12f)+spec,1.f);
    out->color.y=fminf(c.y*(banded+0.12f)+spec,1.f);
    out->color.z=fminf(c.z*(banded+0.12f)+spec,1.f);
    out->discard=false;
}

/*
 * frag_normals — world normal → RGB.
 * On the displaced sphere this shows the recomputed deformed normals
 * directly — the wave crests appear as rotating hue bands, making
 * the central difference calculation visually verifiable.
 */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_)
{
    (void)u_;
    Vec3 N=v3_norm(in->world_nrm);
    out->color  =v3(N.x*.5f+.5f, N.y*.5f+.5f, N.z*.5f+.5f);
    out->discard=false;
}

/*
 * frag_wire — barycentric edge detection.
 * On the displaced sphere the wireframe shows the UV grid deformed
 * by the displacement — latitude lines ripple in and out, the spiky
 * mode makes the grid look like a porcupine.
 */
static void frag_wire(const FSIn *in, FSOut *out, const void *u_)
{
    (void)u_;
    float edge=fminf(in->custom[0], fminf(in->custom[1],in->custom[2]));
    if(edge>WIRE_THRESH){ out->discard=true; return; }
    float t=edge/WIRE_THRESH;
    out->color  =v3(0.9f-t*0.3f, 0.9f-t*0.3f, 0.9f-t*0.3f);
    out->discard=false;
}

typedef enum { SH_PHONG=0, SH_TOON, SH_NORMALS, SH_WIRE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[]={"phong","toon","normals","wire"};

/* ===================================================================== */
/* §5  mesh — UV sphere                                                   */
/* ===================================================================== */

typedef struct { Vec3 pos; Vec3 normal; float u,v; } Vertex;
typedef struct { int v[3]; } Triangle;
typedef struct { Vertex *verts; int nvert; Triangle *tris; int ntri; } Mesh;

static void mesh_free(Mesh *m){ free(m->verts); free(m->tris); *m=(Mesh){0}; }

/*
 * tessellate_sphere — identical to sphere_raster.c.
 * Normal = normalised position (unit sphere).
 * Pole normals set explicitly to avoid sin(phi)=0 degenerate case.
 * Winding: (r0,r2,r1) and (r1,r2,r3) — CCW from outside.
 */
static Mesh tessellate_sphere(void)
{
    int nu=TESS_U, nv=TESS_V;
    float R=SPHERE_R, PI2=2.f*PI;
    Mesh m;
    m.verts=malloc((size_t)(nu+1)*(nv+1)*sizeof(Vertex));
    m.tris =malloc((size_t)nu*nv*2*sizeof(Triangle));
    m.nvert=0; m.ntri=0;

    for(int j=0;j<=nv;j++){
        float v=  (float)j/nv;
        float phi=v*PI;
        float sp=sinf(phi), cp=cosf(phi);
        for(int i=0;i<=nu;i++){
            float u    =(float)i/nu;
            float theta=u*PI2;
            Vec3 pos=v3(R*sp*cosf(theta), R*cp, R*sp*sinf(theta));
            Vec3 nrm=(sp<1e-6f) ? ((j==0)?v3(0,1,0):v3(0,-1,0))
                                 : v3_norm(pos);
            m.verts[m.nvert++]=(Vertex){pos,nrm,u,v};
        }
    }
    for(int j=0;j<nv;j++){
        for(int i=0;i<nu;i++){
            int r0=j*(nu+1)+i, r1=r0+1;
            int r2=r0+(nu+1), r3=r2+1;
            m.tris[m.ntri++]=(Triangle){{r0,r2,r1}};
            m.tris[m.ntri++]=(Triangle){{r1,r2,r3}};
        }
    }
    return m;
}

/* ===================================================================== */
/* §6  framebuffer                                                        */
/* ===================================================================== */

typedef struct { char ch; int color_pair; bool bold; } Cell;
typedef struct { float *zbuf; Cell *cbuf; int cols,rows; } Framebuffer;

static void fb_alloc(Framebuffer *fb,int c,int r){
    fb->cols=c; fb->rows=r;
    fb->zbuf=malloc((size_t)(c*r)*sizeof(float));
    fb->cbuf=malloc((size_t)(c*r)*sizeof(Cell));
}
static void fb_free(Framebuffer *fb){
    free(fb->zbuf); free(fb->cbuf); *fb=(Framebuffer){0};
}
static void fb_clear(Framebuffer *fb){
    for(int i=0;i<fb->cols*fb->rows;i++) fb->zbuf[i]=FLT_MAX;
    memset(fb->cbuf,0,(size_t)(fb->cols*fb->rows)*sizeof(Cell));
}

static void color_init(void){
    start_color();
    if(COLORS>=256){
        init_pair(1,196,COLOR_BLACK); init_pair(2,208,COLOR_BLACK);
        init_pair(3,226,COLOR_BLACK); init_pair(4, 46,COLOR_BLACK);
        init_pair(5, 51,COLOR_BLACK); init_pair(6, 21,COLOR_BLACK);
        init_pair(7,201,COLOR_BLACK);
    } else {
        init_pair(1,COLOR_RED,    COLOR_BLACK);
        init_pair(2,COLOR_RED,    COLOR_BLACK);
        init_pair(3,COLOR_YELLOW, COLOR_BLACK);
        init_pair(4,COLOR_GREEN,  COLOR_BLACK);
        init_pair(5,COLOR_CYAN,   COLOR_BLACK);
        init_pair(6,COLOR_BLUE,   COLOR_BLACK);
        init_pair(7,COLOR_MAGENTA,COLOR_BLACK);
    }
}

static Cell luma_to_cell(float luma,int px,int py){
    float d=luma+(k_bayer[py&3][px&3]-0.5f)*0.15f;
    d=d<0.f?0.f:d>1.f?1.f:d;
    int idx=(int)(d*(BOURKE_LEN-1));
    int cp =1+(int)(d*6.f); if(cp>7)cp=7;
    return (Cell){k_bourke[idx],cp,d>0.6f};
}

static void fb_blit(const Framebuffer *fb){
    for(int y=0;y<fb->rows;y++){
        for(int x=0;x<fb->cols;x++){
            Cell c=fb->cbuf[y*fb->cols+x];
            if(!c.ch) continue;
            attr_t a=COLOR_PAIR(c.color_pair)|(c.bold?A_BOLD:0);
            attron(a); mvaddch(y,x,(chtype)(unsigned char)c.ch); attroff(a);
        }
    }
}

/* ===================================================================== */
/* §7  pipeline                                                           */
/* ===================================================================== */

static void barycentric(const float sx[3],const float sy[3],
                        float px,float py,float b[3]){
    float d=(sy[1]-sy[2])*(sx[0]-sx[2])+(sx[2]-sx[1])*(sy[0]-sy[2]);
    if(fabsf(d)<1e-6f){b[0]=b[1]=b[2]=-1.f;return;}
    b[0]=((sy[1]-sy[2])*(px-sx[2])+(sx[2]-sx[1])*(py-sy[2]))/d;
    b[1]=((sy[2]-sy[0])*(px-sx[2])+(sx[0]-sx[2])*(py-sy[2]))/d;
    b[2]=1.f-b[0]-b[1];
}

static void pipeline_draw_mesh(Framebuffer   *fb,
                                const Mesh    *mesh,
                                ShaderProgram *sh,
                                bool           is_wire,
                                bool           cull_backface)
{
    int cols=fb->cols, rows=fb->rows;
    static const float wu[3]={1.f,0.f,0.f};
    static const float wv[3]={0.f,1.f,0.f};

    for(int ti=0;ti<mesh->ntri;ti++){
        const Triangle *tri=&mesh->tris[ti];
        VSOut vo[3];
        for(int vi=0;vi<3;vi++){
            const Vertex *vtx=&mesh->verts[tri->v[vi]];
            VSIn in;
            in.pos=vtx->pos; in.normal=vtx->normal;
            in.u=is_wire?wu[vi]:vtx->u;
            in.v=is_wire?wv[vi]:vtx->v;
            memset(&vo[vi],0,sizeof vo[vi]);
            sh->vert(&in,&vo[vi],sh->vert_uni);
            if(is_wire){
                vo[vi].custom[0]=wu[vi];
                vo[vi].custom[1]=wv[vi];
                vo[vi].custom[2]=1.f-wu[vi]-wv[vi];
            }
        }

        if(vo[0].clip_pos.w<0.001f &&
           vo[1].clip_pos.w<0.001f &&
           vo[2].clip_pos.w<0.001f) continue;

        float sx[3],sy[3],sz[3];
        for(int vi=0;vi<3;vi++){
            float w=vo[vi].clip_pos.w; if(fabsf(w)<1e-6f)w=1e-6f;
            sx[vi]=( vo[vi].clip_pos.x/w+1.f)*0.5f*(float)cols;
            sy[vi]=(-vo[vi].clip_pos.y/w+1.f)*0.5f*(float)rows;
            sz[vi]=  vo[vi].clip_pos.z/w;
        }

        float area=(sx[1]-sx[0])*(sy[2]-sy[0])
                  -(sx[2]-sx[0])*(sy[1]-sy[0]);
        if(cull_backface && area<=0.f) continue;

        int x0=(int)fmaxf(0.f,     floorf(fminf(sx[0],fminf(sx[1],sx[2]))));
        int x1=(int)fminf(cols-1.f, ceilf(fmaxf(sx[0],fmaxf(sx[1],sx[2]))));
        int y0=(int)fmaxf(0.f,     floorf(fminf(sy[0],fminf(sy[1],sy[2]))));
        int y1=(int)fminf(rows-1.f, ceilf(fmaxf(sy[0],fmaxf(sy[1],sy[2]))));

        for(int py=y0;py<=y1;py++){
            for(int px=x0;px<=x1;px++){
                float b[3];
                barycentric(sx,sy,px+.5f,py+.5f,b);
                if(b[0]<0.f||b[1]<0.f||b[2]<0.f) continue;

                float z=b[0]*sz[0]+b[1]*sz[1]+b[2]*sz[2];
                int idx=py*cols+px;
                if(z>=fb->zbuf[idx]) continue;
                fb->zbuf[idx]=z;

                FSIn fsin;
                fsin.world_pos=v3_bary(vo[0].world_pos,vo[1].world_pos,
                                        vo[2].world_pos,b[0],b[1],b[2]);
                fsin.world_nrm=v3_norm(
                               v3_bary(vo[0].world_nrm,vo[1].world_nrm,
                                        vo[2].world_nrm,b[0],b[1],b[2]));
                fsin.u =b[0]*vo[0].u+b[1]*vo[1].u+b[2]*vo[2].u;
                fsin.v =b[0]*vo[0].v+b[1]*vo[1].v+b[2]*vo[2].v;
                fsin.px=px; fsin.py=py;
                for(int c=0;c<4;c++)
                    fsin.custom[c]=b[0]*vo[0].custom[c]
                                  +b[1]*vo[1].custom[c]
                                  +b[2]*vo[2].custom[c];

                FSOut fsout; fsout.discard=false;
                sh->frag(&fsin,&fsout,sh->frag_uni);
                if(fsout.discard) continue;

                float luma=0.2126f*fsout.color.x
                          +0.7152f*fsout.color.y
                          +0.0722f*fsout.color.z;
                fb->cbuf[idx]=luma_to_cell(luma,px,py);
            }
        }
    }
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Per-mode amplitude and frequency tuning.
 * Each mode looks best at different scales — these are the values
 * that make each mode look visually distinct and dramatic.
 *
 * amp  — fraction of sphere radius to displace
 * freq — spatial frequency of the wave pattern
 */
static const float k_amp [DM_COUNT] = { 0.22f, 0.18f, 0.30f, 0.35f };
static const float k_freq[DM_COUNT] = { 8.0f,  5.0f,  4.0f,  4.5f  };

typedef struct {
    Mesh             mesh;
    float            angle_x, angle_y;
    float            cam_dist;
    bool             paused;
    bool             cull_backface;
    ShaderIdx        shade_idx;
    DispMode         disp_idx;
    ShaderProgram    shader;
    Uniforms         uni;
    ToonUniforms     toon_uni;
    DisplaceUniforms disp_uni;
    float            time;
} Scene;

static void scene_build_shader(Scene *s)
{
    /*
     * vert_uni is ALWAYS &disp_uni — every vertex shader variant is a
     * form of vert_displace and needs DisplaceUniforms to call disp_fn.
     *
     * frag_uni points to the struct the fragment shader actually needs:
     *   frag_phong   → Uniforms*    — cast from &disp_uni (base is first member)
     *   frag_toon    → ToonUniforms* — needs bands field
     *   frag_normals → unused       — (void)u_ so either pointer is fine
     *   frag_wire    → unused       — (void)u_ so either pointer is fine
     *
     * DisplaceUniforms leads with Uniforms base so &disp_uni casts cleanly
     * to const Uniforms* inside frag_phong — same zero-offset rule as before.
     */
    switch(s->shade_idx){
    case SH_PHONG:
        s->shader.vert     = vert_displace;
        s->shader.frag     = frag_phong;
        s->shader.vert_uni = &s->disp_uni;
        s->shader.frag_uni = &s->disp_uni;   /* Uniforms* cast from Disp* */
        break;
    case SH_TOON:
        s->toon_uni.base   = s->disp_uni.base;
        s->toon_uni.bands  = 4;
        s->shader.vert     = vert_displace;
        s->shader.frag     = frag_toon;
        s->shader.vert_uni = &s->disp_uni;   /* vert needs disp params    */
        s->shader.frag_uni = &s->toon_uni;   /* frag needs bands          */
        break;
    case SH_NORMALS:
        s->shader.vert     = vert_displace_normals;
        s->shader.frag     = frag_normals;
        s->shader.vert_uni = &s->disp_uni;
        s->shader.frag_uni = &s->disp_uni;
        break;
    case SH_WIRE:
        s->shader.vert     = vert_displace_wire;
        s->shader.frag     = frag_wire;
        s->shader.vert_uni = &s->disp_uni;
        s->shader.frag_uni = &s->disp_uni;
        break;
    default: break;
    }
}

static void scene_sync_disp(Scene *s)
{
    /*
     * Copy current base uniforms into disp_uni and update displacement
     * params.  Called every frame from scene_tick so time, mvp, and
     * norm_mat stay current.
     *
     * toon_uni.base is updated in scene_build_shader when the toon
     * shader is selected, and again here every frame so the lighting
     * matrices stay current when the toon shader is active.
     */
    s->disp_uni.base      = s->uni;
    s->disp_uni.disp_fn   = k_disp_fn[s->disp_idx];
    s->disp_uni.time      = s->time;
    s->disp_uni.amplitude = k_amp [s->disp_idx];
    s->disp_uni.frequency = k_freq[s->disp_idx];
    s->disp_uni.mode      = s->disp_idx;

    /* keep toon base matrices current if toon shader is active */
    if(s->shade_idx == SH_TOON)
        s->toon_uni.base = s->uni;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s,0,sizeof *s);
    s->mesh          = tessellate_sphere();
    s->shade_idx     = SH_PHONG;
    s->disp_idx      = DM_RIPPLE;
    s->cam_dist      = CAM_DIST;
    s->cull_backface = true;
    s->time          = 0.f;

    s->uni.light_pos  = v3(4.f, 5.f, 3.f);
    s->uni.light_col  = v3(1.f, 1.f, 1.f);
    s->uni.ambient    = v3(0.06f,0.06f,0.06f);
    s->uni.shininess  = 60.f;
    s->uni.cam_pos    = v3(0.f,0.f,s->cam_dist);
    s->uni.obj_color  = v3(0.2f,0.7f,0.95f);   /* ocean blue */

    s->uni.view = m4_lookat(s->uni.cam_pos,v3(0,0,0),v3(0,1,0));
    float aspect=(float)(cols*CELL_W)/(float)(rows*CELL_H);
    s->uni.proj = m4_perspective(CAM_FOV,aspect,CAM_NEAR,CAM_FAR);

    scene_sync_disp(s);
    scene_build_shader(s);
}

static void scene_set_zoom(Scene *s){
    s->uni.cam_pos=v3(0.f,0.f,s->cam_dist);
    s->uni.view   =m4_lookat(s->uni.cam_pos,v3(0,0,0),v3(0,1,0));
}

static void scene_rebuild_proj(Scene *s,int cols,int rows){
    float aspect=(float)(cols*CELL_W)/(float)(rows*CELL_H);
    s->uni.proj=m4_perspective(CAM_FOV,aspect,CAM_NEAR,CAM_FAR);
}

static void scene_tick(Scene *s, float dt)
{
    if(!s->paused){
        s->time    += dt;
        s->angle_y += ROT_Y*dt;
        s->angle_x += ROT_X*dt;
    }
    Mat4 ry=m4_rotate_y(s->angle_y);
    Mat4 rx=m4_rotate_x(s->angle_x);
    s->uni.model    = m4_mul(ry,rx);
    s->uni.mvp      = m4_mul(s->uni.proj,m4_mul(s->uni.view,s->uni.model));
    s->uni.norm_mat = m4_normal_mat(s->uni.model);
    scene_sync_disp(s);
    /*
     * No shader.uniforms fixup here — vert_uni/frag_uni are set once in
     * scene_build_shader and remain valid for the lifetime of that shader
     * selection.  scene_sync_disp keeps the pointed-to structs current.
     */
}

static void scene_draw(Scene *s, Framebuffer *fb){
    fb_clear(fb);
    pipeline_draw_mesh(fb,&s->mesh,&s->shader,
                       (s->shade_idx==SH_WIRE),s->cull_backface);
    fb_blit(fb);
}

static void scene_next_shader(Scene *s){
    s->shade_idx=(ShaderIdx)((s->shade_idx+1)%SH_COUNT);
    scene_build_shader(s);
}

static void scene_next_disp(Scene *s){
    s->disp_idx=(DispMode)((s->disp_idx+1)%DM_COUNT);
    scene_sync_disp(s);
    scene_build_shader(s);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

typedef struct { int cols,rows; } Screen;

static void screen_init(Screen *s){
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr,TRUE); keypad(stdscr,TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr,s->rows,s->cols);
}
static void screen_free(Screen *s){ (void)s; endwin(); }
static void screen_resize(Screen *s){ endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols); }

static void screen_draw_hud(const Screen *s,const Scene *sc,double fps)
{
    char buf[HUD_COLS+1];
    snprintf(buf,sizeof buf," %5.1f fps  [%s][%s]  z:%.1f  c:%s%s ",
             fps,
             k_disp_names[sc->disp_idx],
             k_shader_names[sc->shade_idx],
             sc->cam_dist,
             sc->cull_backface?"on ":"off",
             sc->paused?" PAUSED":"");
    int hx=s->cols-HUD_COLS; if(hx<0)hx=0;
    attron(COLOR_PAIR(3)|A_BOLD);
    mvprintw(0,hx,"%s",buf);
    attroff(COLOR_PAIR(3)|A_BOLD);
    attron(COLOR_PAIR(5));
    mvprintw(s->rows-1,0,"d=disp  s=shader  c=cull  +/-=zoom  space=pause  q=quit");
    attroff(COLOR_PAIR(5));
}

static void screen_present(void){ wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    Framebuffer           fb;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit  (int sig){ (void)sig; g_app.running=0;     }
static void on_resize(int sig){ (void)sig; g_app.need_resize=1; }
static void cleanup  (void)   { endwin(); }

static void app_do_resize(App *app){
    screen_resize(&app->screen);
    fb_free(&app->fb);
    fb_alloc(&app->fb,app->screen.cols,app->screen.rows);
    scene_rebuild_proj(&app->scene,app->screen.cols,app->screen.rows);
    app->need_resize=0;
}

static bool app_handle_key(App *app,int ch){
    Scene *s=&app->scene;
    switch(ch){
    case 'q': case 'Q': case 27: return false;
    case ' ': s->paused=!s->paused; break;
    case 's': case 'S': scene_next_shader(s); break;
    case 'd': case 'D': scene_next_disp(s);   break;
    case 'c': case 'C': s->cull_backface=!s->cull_backface; break;
    case '=': case '+':
        s->cam_dist-=CAM_ZOOM_STEP;
        if(s->cam_dist<CAM_DIST_MIN)s->cam_dist=CAM_DIST_MIN;
        scene_set_zoom(s); break;
    case '-':
        s->cam_dist+=CAM_ZOOM_STEP;
        if(s->cam_dist>CAM_DIST_MAX)s->cam_dist=CAM_DIST_MAX;
        scene_set_zoom(s); break;
    default: break;
    }
    return true;
}

static int64_t clock_ns(void){
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return (int64_t)t.tv_sec*NS_PER_SEC+t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns){
    if(ns<=0)return;
    struct timespec r={.tv_sec=(time_t)(ns/NS_PER_SEC),
                       .tv_nsec=(long)(ns%NS_PER_SEC)};
    nanosleep(&r,NULL);
}

int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,  on_exit);
    signal(SIGTERM, on_exit);
    signal(SIGWINCH,on_resize);

    App *app=&g_app; app->running=1;

    screen_init(&app->screen);
    fb_alloc(&app->fb,app->screen.cols,app->screen.rows);
    scene_init(&app->scene,app->screen.cols,app->screen.rows);

    int64_t frame_time=clock_ns();
    int64_t fps_acc=0; int fps_cnt=0; double fps_disp=0.0;

    while(app->running){

        if(app->need_resize){ app_do_resize(app); frame_time=clock_ns(); }

        int64_t now=clock_ns();
        int64_t dt =now-frame_time; frame_time=now;
        if(dt>100*NS_PER_MS) dt=100*NS_PER_MS;
        float dt_sec=(float)dt/(float)NS_PER_SEC;

        scene_tick(&app->scene,dt_sec);

        fps_cnt++; fps_acc+=dt;
        if(fps_acc>=FPS_UPDATE_MS*NS_PER_MS){
            fps_disp=(double)fps_cnt/((double)fps_acc/(double)NS_PER_SEC);
            fps_cnt=0; fps_acc=0;
        }

        erase();
        scene_draw(&app->scene,&app->fb);
        screen_draw_hud(&app->screen,&app->scene,fps_disp);
        screen_present();

        int ch=getch();
        if(ch!=ERR && !app_handle_key(app,ch)) app->running=0;

        int64_t elapsed=clock_ns()-frame_time+dt;
        clock_sleep_ns(NS_PER_SEC/FPS_TARGET-elapsed);
    }

    mesh_free(&app->scene.mesh);
    fb_free(&app->fb);
    screen_free(&app->screen);
    return 0;
}
