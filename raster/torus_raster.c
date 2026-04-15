/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * torus_raster.c  —  ncurses software rasterizer, torus demo
 *
 * One primitive: torus (tessellated mesh, rendered every frame).
 * Four shaders cycled with  s : phong / toon / normals / wireframe
 *
 * Pipeline:
 *   tessellate_torus()    once at init
 *       ↓
 *   scene_tick()          rotate model matrix each frame
 *       ↓
 *   pipeline_draw()       for every triangle:
 *       vert shader        VSIn → VSOut          (C function pointer)
 *       clip / NDC / screen
 *       back-face cull
 *       rasterize          barycentric coverage
 *       z-test             float zbuf
 *       frag shader        FSIn → FSOut          (C function pointer)
 *       luma → dither → Paul Bourke char → cbuf
 *       ↓
 *   fb_blit()             cbuf → stdscr → doupdate
 *
 * Shader interface — pure C, no GLSL, no parsing:
 *   VSIn / VSOut / FSIn / FSOut  plain structs
 *   VertexShaderFn / FragmentShaderFn  function pointers
 *   uniforms  void* to any C struct, cast inside shader
 *
 * Wireframe shader — barycentric edge detection:
 *   vert_wire packs per-vertex barycentric coord (1,0,0)/(0,1,0)/(0,0,1)
 *   into custom[0..2].  After interpolation, min(custom[]) = distance
 *   to nearest edge.  frag_wire discards interior fragments.
 *
 * Keys:
 *   s / S     cycle shader  (phong → toon → normals → wireframe)
 *   space     pause / resume rotation
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra torus_raster.c -o torus -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  math         Vec3 Vec4 Mat4
 *   §3  shaders      structs + 4 vert/frag pairs
 *   §4  mesh         Vertex Triangle Mesh + tessellate_torus
 *   §5  framebuffer  zbuf cbuf dither paulbourke blit
 *   §6  pipeline     vertex transform · rasterize · draw
 *   §7  scene        uniforms · tick · draw · shader swap
 *   §8  screen       ncurses init/resize/hud/present
 *   §9  app          dt loop · input · resize · cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Software rasterization pipeline (GPU-pipeline emulation in C).
 *                  The full pipeline runs every frame:
 *                  1. Tessellate mesh into triangles (done once at init).
 *                  2. Vertex shader: transform each vertex from object→world→clip space.
 *                  3. Perspective divide: clip → NDC → screen coordinates.
 *                  4. Back-face culling: discard triangles facing away from camera.
 *                  5. Rasterization: iterate over bounding box, test barycentric coords.
 *                  6. Z-test: per-fragment depth comparison against float z-buffer.
 *                  7. Fragment shader: Phong/toon/normal/wireframe shading.
 *
 * Math           : Barycentric coordinates (λ₀, λ₁, λ₂) for point p in triangle:
 *                    Area test: λᵢ = signed_area(edge_i) / total_area
 *                  All λ ∈ [0,1] and sum to 1 iff inside the triangle.
 *                  Used to interpolate normals and attributes across the face.
 *                  Perspective-correct interpolation: interpolate z⁻¹, then divide.
 *
 * Performance    : Z-buffer resolves occlusion without sorting triangles.
 *                  Back-face culling halves the triangle count for closed meshes.
 *                  Function pointers (vert_shader, frag_shader) allow swapping
 *                  shaders without changing the pipeline structure.
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
    HUD_COLS      = 38,

    /*
     * Tessellation resolution.
     * TESS_U: slices around the ring (longitude).
     * TESS_V: slices around the tube (latitude).
     * Higher = smoother torus, more triangles, slower fill.
     * At terminal resolution 32×24 is a good balance.
     */
    TESS_U = 32,
    TESS_V = 24,
};

/* Camera */
#define CAM_FOV   (60.0f * 3.14159265f / 180.0f)
#define CAM_NEAR  0.1f
#define CAM_FAR   100.0f
#define CAM_DIST  3.2f

/* Torus geometry */
#define TORUS_R   0.65f    /* major radius: centre to tube centre */
#define TORUS_r   0.28f    /* tube radius                         */

/* Rotation speed in radians per second */
#define ROT_Y  0.70f
#define ROT_X  0.28f

/*
 * Wireframe edge threshold.
 * min(barycentric coords) < WIRE_THRESH → draw edge char.
 * Larger value = thicker wireframe lines.
 * At terminal resolution 0.06–0.10 works well.
 */
#define WIRE_THRESH  0.08f

/* Paul Bourke ASCII density ramp — darkest → brightest */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* Bayer 4×4 ordered dither matrix, normalised to [0,1) */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};

/*
 * CELL_W / CELL_H — terminal cell aspect ratio correction.
 * Terminal cells are physically ~2× taller than wide.
 * The projection matrix uses (cols*CELL_W)/(rows*CELL_H) as aspect
 * so that the rendered torus appears round, not squashed.
 * Same principle as bounce.c pixel space.
 */
#define CELL_W  8
#define CELL_H  16

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* ===================================================================== */
/* §2  math                                                               */
/* ===================================================================== */

typedef struct { float x, y, z;    } Vec3;
typedef struct { float x, y, z, w; } Vec4;
typedef struct { float m[4][4];     } Mat4;

/* ── Vec3 ────────────────────────────────────────────────────────── */
static inline Vec3  v3(float x,float y,float z){ return (Vec3){x,y,z}; }
static inline Vec3  v3_add(Vec3 a,Vec3 b)  { return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline Vec3  v3_sub(Vec3 a,Vec3 b)  { return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline Vec3  v3_scale(Vec3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline Vec3  v3_neg(Vec3 a)         { return v3(-a.x,-a.y,-a.z); }
static inline float v3_dot(Vec3 a,Vec3 b)  { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3_len(Vec3 a)         { return sqrtf(v3_dot(a,a)); }
static inline Vec3  v3_norm(Vec3 a){
    float l = v3_len(a);
    return l > 1e-7f ? v3_scale(a, 1.f/l) : v3(0,1,0);
}
static inline Vec3 v3_reflect(Vec3 d, Vec3 n){
    return v3_sub(d, v3_scale(n, 2.f*v3_dot(d,n)));
}

/*
 * v3_bary — barycentric blend of three Vec3 values.
 * Used to interpolate world_pos and world_nrm across a triangle.
 */
static inline Vec3 v3_bary(Vec3 a,Vec3 b,Vec3 c,float u,float v,float w){
    return v3(u*a.x+v*b.x+w*c.x,
              u*a.y+v*b.y+w*c.y,
              u*a.z+v*b.z+w*c.z);
}

/* ── Vec4 ────────────────────────────────────────────────────────── */
static inline Vec4 v4(float x,float y,float z,float w){ return (Vec4){x,y,z,w}; }

/* ── Mat4 ────────────────────────────────────────────────────────── */
static inline Mat4 m4_identity(void){
    Mat4 m={{{0}}}; m.m[0][0]=m.m[1][1]=m.m[2][2]=m.m[3][3]=1.f; return m;
}

static inline Vec4 m4_mul_v4(Mat4 m, Vec4 v){
    return v4(
        m.m[0][0]*v.x+m.m[0][1]*v.y+m.m[0][2]*v.z+m.m[0][3]*v.w,
        m.m[1][0]*v.x+m.m[1][1]*v.y+m.m[1][2]*v.z+m.m[1][3]*v.w,
        m.m[2][0]*v.x+m.m[2][1]*v.y+m.m[2][2]*v.z+m.m[2][3]*v.w,
        m.m[3][0]*v.x+m.m[3][1]*v.y+m.m[3][2]*v.z+m.m[3][3]*v.w);
}

static inline Mat4 m4_mul(Mat4 a, Mat4 b){
    Mat4 r={{{0}}};
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            for(int k=0;k<4;k++)
                r.m[i][j]+=a.m[i][k]*b.m[k][j];
    return r;
}

/* Transform a point (w=1) — translation applies */
static inline Vec3 m4_pt(Mat4 m, Vec3 p){
    Vec4 r=m4_mul_v4(m,v4(p.x,p.y,p.z,1.f)); return v3(r.x,r.y,r.z);
}

/* Transform a direction (w=0) — translation ignored */
static inline Vec3 m4_dir(Mat4 m, Vec3 d){
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

/*
 * m4_perspective — perspective projection matrix.
 *
 * aspect_px = (cols * CELL_W) / (rows * CELL_H)
 * This is the physical pixel aspect ratio of the terminal, not just
 * the cell count ratio.  Without this correction a sphere looks like
 * a vertical ellipse because terminal cells are taller than wide.
 */
static Mat4 m4_perspective(float fovy, float aspect, float near, float far){
    Mat4 m={{{0}}};
    float f=1.f/tanf(fovy*.5f);
    m.m[0][0]=f/aspect;
    m.m[1][1]=f;
    m.m[2][2]=(far+near)/(near-far);
    m.m[2][3]=(2.f*far*near)/(near-far);
    m.m[3][2]=-1.f;
    return m;
}

static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up){
    Vec3 f=v3_norm(v3_sub(at,eye));
    Vec3 r=v3_norm((Vec3){f.z*up.y-f.y*up.z,
                           f.x*up.z-f.z*up.x,
                           f.y*up.x-f.x*up.y});
    Vec3 u=(Vec3){r.y*f.z-r.z*f.y, r.z*f.x-r.x*f.z, r.x*f.y-r.y*f.x};
    Mat4 m=m4_identity();
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-v3_dot(r,eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-v3_dot(u,eye);
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]= v3_dot(f,eye);
    return m;
}

/*
 * m4_normal_mat — matrix to correctly transform normals.
 *
 * When the model matrix has non-uniform scale, simply transforming
 * normals with the model matrix distorts them.  The correct transform
 * is transpose(inverse(upper-left 3×3)).
 *
 * This computes the cofactor matrix of the 3×3 block, which equals
 * the adjugate (= inverse * det).  Since we only care about direction
 * (normalise afterwards), the determinant factor doesn't matter.
 */
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
/* §3  shaders                                                            */
/* ===================================================================== */

/*
 * Shader data flow:
 *
 *   Mesh vertex  →  VSIn  →  [vert shader]  →  VSOut
 *                                                  ↓
 *                              barycentric interpolation across triangle
 *                                                  ↓
 *                                               FSIn  →  [frag shader]  →  FSOut
 *                                                                              ↓
 *                                                              luma_to_cell → cbuf
 *
 * custom[4] in VSOut/FSIn is a general-purpose interpolated channel.
 * Each shader pair uses it differently:
 *   phong / toon   — unused (custom not needed)
 *   normals        — custom[0..2] = world normal components
 *   wireframe      — custom[0..2] = per-vertex barycentric coord
 *
 * The pipeline interpolates custom[] barycentrically like every other field,
 * so no special casing is needed per shader.
 */

typedef struct {
    Vec3  pos;     /* model space */
    Vec3  normal;  /* model space */
    float u, v;
} VSIn;

typedef struct {
    Vec4  clip_pos;    /* REQUIRED — used by pipeline for projection */
    Vec3  world_pos;   /* interpolated → FSIn */
    Vec3  world_nrm;   /* interpolated → FSIn */
    float u, v;
    float custom[4];   /* shader-specific interpolated payload */
} VSOut;

typedef struct {
    Vec3  world_pos;
    Vec3  world_nrm;
    float u, v;
    float custom[4];
    int   px, py;      /* screen cell coordinates — for dither pattern */
} FSIn;

typedef struct {
    Vec3  color;
    bool  discard;     /* true = pipeline skips this cell entirely */
} FSOut;

typedef void (*VertShaderFn) (const VSIn *in,  VSOut *out, const void *uni);
typedef void (*FragShaderFn) (const FSIn *in,  FSOut *out, const void *uni);

typedef struct {
    VertShaderFn  vert;
    FragShaderFn  frag;
    const void   *vert_uni;   /* passed to vert() */
    const void   *frag_uni;   /* passed to frag() */
} ShaderProgram;

/* ── Uniforms ────────────────────────────────────────────────────── */

typedef struct {
    Mat4  model;
    Mat4  view;
    Mat4  proj;
    Mat4  mvp;        /* proj * view * model — precomputed each frame */
    Mat4  norm_mat;   /* transpose(inverse(model 3x3)) */
    Vec3  light_pos;
    Vec3  light_col;
    Vec3  ambient;
    Vec3  cam_pos;
    Vec3  obj_color;
    float shininess;
} Uniforms;

/* ToonUniforms embeds Uniforms as first member so &toon_uni casts
   cleanly to const Uniforms* inside frag_phong / vert_default.      */
typedef struct {
    Uniforms base;
    int      bands;
} ToonUniforms;

/* ── §3a  vertex shaders ─────────────────────────────────────────── */

/*
 * vert_default — standard MVP transform.
 * Outputs world-space position + normal for fragment lighting.
 * Used by phong and toon.
 */
static void vert_default(const VSIn *in, VSOut *out, const void *u_)
{
    const Uniforms *u = (const Uniforms *)u_;
    out->clip_pos  = m4_mul_v4(u->mvp, v4(in->pos.x,in->pos.y,in->pos.z,1.f));
    out->world_pos = m4_pt (u->model,    in->pos);
    out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
    out->u=in->u; out->v=in->v;
    /* custom unused */
    out->custom[0]=out->custom[1]=out->custom[2]=out->custom[3]=0.f;
}

/*
 * vert_normals — same transform but also packs world normal into
 * custom[0..2] for the normals fragment shader to read after
 * interpolation.  frag_normals could use world_nrm directly, but
 * packing into custom demonstrates the general interpolation channel.
 */
static void vert_normals(const VSIn *in, VSOut *out, const void *u_)
{
    const Uniforms *u = (const Uniforms *)u_;
    out->clip_pos  = m4_mul_v4(u->mvp, v4(in->pos.x,in->pos.y,in->pos.z,1.f));
    out->world_pos = m4_pt (u->model,    in->pos);
    out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
    out->u=in->u; out->v=in->v;
    out->custom[0]=out->world_nrm.x;
    out->custom[1]=out->world_nrm.y;
    out->custom[2]=out->world_nrm.z;
    out->custom[3]=0.f;
}

/*
 * vert_wire — packs one corner of the barycentric coordinate system
 * into custom[0..2] per vertex:
 *   vertex 0 → (1,0,0)
 *   vertex 1 → (0,1,0)
 *   vertex 2 → (0,0,1)
 *
 * The pipeline's barycentric interpolation then fills in the gradient
 * across the triangle.  At each interior fragment all three components
 * are > 0.  Near each edge one component approaches 0.
 *
 * The pipeline does not know which vertex index (0/1/2) a given VSOut
 * corresponds to — that information is lost after the vertex shader runs.
 * Instead we pre-assign the barycentric coord in the pipeline before
 * calling vert_wire: custom[0..2] in VSIn is set to the appropriate
 * unit vector by pipeline_draw_mesh before each vert call.
 * See §6 for the three-call pattern.
 */
static void vert_wire(const VSIn *in, VSOut *out, const void *u_)
{
    const Uniforms *u = (const Uniforms *)u_;
    out->clip_pos  = m4_mul_v4(u->mvp, v4(in->pos.x,in->pos.y,in->pos.z,1.f));
    out->world_pos = m4_pt (u->model,    in->pos);
    out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
    out->u=in->u; out->v=in->v;
    /* custom[0..2] already set by pipeline before this call (see §6) */
    out->custom[0]=in->u;   /* pipeline overwrites u/v with bary — see §6 */
    out->custom[1]=in->v;
    out->custom[2]=0.f;
    out->custom[3]=0.f;
}

/* ── §3b  fragment shaders ───────────────────────────────────────── */

/*
 * frag_phong — Phong shading with gamma correction.
 *
 * Ambient + diffuse (Lambertian) + specular (Blinn-Phong).
 * Gamma correction applied before handing off to luma_to_cell:
 *   output = linear^(1/2.2)
 * This matches the gamma correction in raymarching_primitives.c.
 */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_)
{
    const Uniforms *u = (const Uniforms *)u_;

    Vec3 N = v3_norm(in->world_nrm);
    Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
    Vec3 V = v3_norm(v3_sub(u->cam_pos,   in->world_pos));
    Vec3 H = v3_norm(v3_add(L, V));                /* half-vector (Blinn) */

    float diff = fmaxf(0.f, v3_dot(N, L));
    float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess);

    Vec3 c = u->obj_color;
    float r = u->ambient.x + c.x*u->light_col.x*diff + spec*0.5f;
    float g = u->ambient.y + c.y*u->light_col.y*diff + spec*0.5f;
    float b = u->ambient.z + c.z*u->light_col.z*diff + spec*0.5f;

    /* gamma correction */
    out->color.x = powf(fminf(r,1.f), 1.f/2.2f);
    out->color.y = powf(fminf(g,1.f), 1.f/2.2f);
    out->color.z = powf(fminf(b,1.f), 1.f/2.2f);
    out->discard  = false;
}

/*
 * frag_toon — quantised Phong into N discrete bands.
 *
 * Diffuse is floored into `bands` equal steps, producing the hard
 * boundary between light and shadow that defines cel shading.
 * Specular is binary — either fully on or fully off based on a
 * half-vector threshold — giving a hard specular highlight.
 */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_)
{
    const ToonUniforms *tu = (const ToonUniforms *)u_;
    const Uniforms     *u  = &tu->base;

    Vec3 N = v3_norm(in->world_nrm);
    Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
    Vec3 V = v3_norm(v3_sub(u->cam_pos,   in->world_pos));
    Vec3 H = v3_norm(v3_add(L, V));

    float diff   = fmaxf(0.f, v3_dot(N, L));
    float banded = floorf(diff * (float)tu->bands) / (float)tu->bands;
    float spec   = (v3_dot(N, H) > 0.94f) ? 0.7f : 0.0f;

    Vec3 c = u->obj_color;
    out->color.x = fminf(c.x*(banded+0.12f) + spec, 1.f);
    out->color.y = fminf(c.y*(banded+0.12f) + spec, 1.f);
    out->color.z = fminf(c.z*(banded+0.12f) + spec, 1.f);
    out->discard  = false;
}

/*
 * frag_normals — maps world normal [-1,1] → RGB [0,1].
 *
 * Classic debug / art view.  No lighting calculation.
 * The result maps surface orientation directly to colour:
 *   +X right   = red
 *   +Y up      = green
 *   +Z toward camera = blue
 * The torus's toroidal surface produces a smooth hue rotation
 * around the ring and around the tube that makes this shader
 * visually interesting rather than just diagnostic.
 */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_)
{
    (void)u_;
    Vec3 N = v3_norm(in->world_nrm);
    out->color   = v3(N.x*.5f+.5f, N.y*.5f+.5f, N.z*.5f+.5f);
    out->discard = false;
}

/*
 * frag_wire — barycentric edge detection.
 *
 * custom[0..2] holds the interpolated barycentric coordinates, set by
 * the pipeline before calling vert_wire for each vertex.
 *
 * min(b0, b1, b2) is the distance to the nearest triangle edge in
 * barycentric space.  When it falls below WIRE_THRESH the fragment
 * is close enough to an edge to be drawn.  Interior fragments are
 * discarded, leaving only the edge lines.
 *
 * The luma passed to luma_to_cell for wire fragments is 0.85 — a
 * bright value that maps to a dense Paul Bourke character, making
 * the wireframe lines clearly visible.
 */
static void frag_wire(const FSIn *in, FSOut *out, const void *u_)
{
    (void)u_;
    float b0 = in->custom[0];
    float b1 = in->custom[1];
    float b2 = in->custom[2];
    float edge = fminf(b0, fminf(b1, b2));

    if(edge > WIRE_THRESH){
        out->discard = true;
        return;
    }
    /* edge brightness — slightly vary by distance for anti-alias feel */
    float t     = edge / WIRE_THRESH;        /* 0 at edge centre, 1 at threshold */
    out->color   = v3(0.9f - t*0.3f, 0.9f - t*0.3f, 0.9f - t*0.3f);
    out->discard = false;
}

/* ── shader names for HUD ─────────────────────────────────────────── */
typedef enum { SH_PHONG=0, SH_TOON, SH_NORMALS, SH_WIRE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = { "phong","toon","normals","wire" };

/* ===================================================================== */
/* §4  mesh                                                               */
/* ===================================================================== */

typedef struct { Vec3 pos; Vec3 normal; float u,v; } Vertex;
typedef struct { int v[3]; } Triangle;

typedef struct {
    Vertex   *verts;
    int       nvert;
    Triangle *tris;
    int       ntri;
} Mesh;

static void mesh_free(Mesh *m){ free(m->verts); free(m->tris); *m=(Mesh){0}; }

/*
 * tessellate_torus — UV grid over the torus surface.
 *
 * Outer loop: angle theta around the ring (major circle, radius R).
 * Inner loop: angle phi around the tube (minor circle, radius r).
 *
 * Position:
 *   x = (R + r*cos(phi)) * cos(theta)
 *   y =  r * sin(phi)
 *   z = (R + r*cos(phi)) * sin(theta)
 *
 * Normal: direction from the ring centre point to the surface point.
 *   ring_centre(theta) = (R*cos(theta), 0, R*sin(theta))
 *   normal = normalise(position - ring_centre)
 *
 * This gives correct smooth normals for Phong shading without
 * needing to compute partial derivatives of the surface equation.
 */
static Mesh tessellate_torus(void)
{
    int nu = TESS_U, nv = TESS_V;
    int nvert = (nu+1)*(nv+1);
    int ntri  = nu*nv*2;

    Mesh m;
    m.verts = malloc((size_t)nvert * sizeof(Vertex));
    m.tris  = malloc((size_t)ntri  * sizeof(Triangle));
    m.nvert = 0;
    m.ntri  = 0;

    float R = TORUS_R, r = TORUS_r;
    float PI2 = 2.f * 3.14159265f;

    for(int i=0; i<=nu; i++){
        float u     = (float)i / (float)nu;
        float theta = u * PI2;
        float ct    = cosf(theta), st = sinf(theta);

        for(int j=0; j<=nv; j++){
            float v   = (float)j / (float)nv;
            float phi = v * PI2;
            float cp  = cosf(phi), sp = sinf(phi);

            Vec3 pos = v3((R + r*cp)*ct,  r*sp,  (R + r*cp)*st);
            Vec3 rc  = v3(R*ct, 0.f, R*st);        /* ring centre */
            Vec3 nrm = v3_norm(v3_sub(pos, rc));    /* outward tube normal */

            m.verts[m.nvert++] = (Vertex){ pos, nrm, u, v };
        }
    }

    for(int i=0; i<nu; i++){
        for(int j=0; j<nv; j++){
            int r0 = i*(nv+1)+j,   r1 = r0+1;
            int r2 = r0+(nv+1),    r3 = r2+1;
            m.tris[m.ntri++] = (Triangle){{r0,r2,r1}};
            m.tris[m.ntri++] = (Triangle){{r1,r2,r3}};
        }
    }

    return m;
}

/* ===================================================================== */
/* §5  framebuffer                                                        */
/* ===================================================================== */

typedef struct {
    char ch;
    int  color_pair;
    bool bold;
} Cell;

typedef struct {
    float *zbuf;   /* [cols*rows]  FLT_MAX = empty */
    Cell  *cbuf;   /* [cols*rows]  ch==0   = empty */
    int    cols, rows;
} Framebuffer;

static void fb_alloc(Framebuffer *fb, int cols, int rows){
    fb->cols=cols; fb->rows=rows;
    fb->zbuf=malloc((size_t)(cols*rows)*sizeof(float));
    fb->cbuf=malloc((size_t)(cols*rows)*sizeof(Cell));
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

/*
 * luma_to_cell — convert [0,1] luminance to a terminal Cell.
 *
 * Step 1 — Bayer 4×4 ordered dither:
 *   Adds a position-dependent threshold offset so gradients render
 *   as spatial density rather than abrupt luminance steps.
 *   The dither amplitude (0.15) is tuned to be visible but not noisy.
 *
 * Step 2 — Paul Bourke ramp:
 *   Maps dithered luma linearly into k_bourke[].
 *   Low luma → sparse character (space, dot).
 *   High luma → dense character (#, @).
 *
 * Step 3 — color pair:
 *   Bright regions get warmer color pairs (reds/yellows).
 *   Dark regions get cooler pairs (blues/magentas).
 *   This replicates the color gradient from raymarching_primitives.c.
 */
static Cell luma_to_cell(float luma, int px, int py)
{
    float thr     = k_bayer[py&3][px&3];
    float d       = luma + (thr - 0.5f)*0.15f;
    d = d<0.f?0.f: d>1.f?1.f:d;

    int  idx = (int)(d*(BOURKE_LEN-1));
    char ch  = k_bourke[idx];

    int  cp  = 1+(int)(d*6.f);
    if(cp>7) cp=7;
    bool bold = d>0.6f;
    return (Cell){ch, cp, bold};
}

/*
 * fb_blit — copy cbuf into stdscr (ncurses newscr).
 * Called inside erase()…doupdate() frame sequence.
 * Only writes non-empty cells — empty cells stay black background.
 */
static void fb_blit(const Framebuffer *fb)
{
    for(int y=0;y<fb->rows;y++){
        for(int x=0;x<fb->cols;x++){
            Cell c=fb->cbuf[y*fb->cols+x];
            if(!c.ch) continue;
            attr_t a=COLOR_PAIR(c.color_pair)|(c.bold?A_BOLD:0);
            attron(a);
            mvaddch(y,x,(chtype)(unsigned char)c.ch);
            attroff(a);
        }
    }
}

/* ===================================================================== */
/* §6  pipeline                                                           */
/* ===================================================================== */

/*
 * barycentric — compute barycentric coordinates (b[3]) for point
 * (px,py) relative to triangle with screen coords sx[3], sy[3].
 *
 * Formula: signed areas via 2D cross products.
 * If any b[i] < 0 the point is outside the triangle.
 * All b[i] >= 0 and sum to 1.0 when inside.
 */
static void barycentric(const float sx[3], const float sy[3],
                        float px, float py, float b[3])
{
    float d = (sy[1]-sy[2])*(sx[0]-sx[2]) + (sx[2]-sx[1])*(sy[0]-sy[2]);
    if(fabsf(d)<1e-6f){ b[0]=b[1]=b[2]=-1.f; return; }
    b[0]=((sy[1]-sy[2])*(px-sx[2])+(sx[2]-sx[1])*(py-sy[2]))/d;
    b[1]=((sy[2]-sy[0])*(px-sx[2])+(sx[0]-sx[2])*(py-sy[2]))/d;
    b[2]=1.f-b[0]-b[1];
}

/*
 * pipeline_draw_mesh — full rasterization pipeline for one mesh.
 *
 * For each triangle:
 *
 *   1. Vertex shader (×3) — model → clip space.
 *      For SHADER_WIRE: pipeline injects per-vertex barycentric coords
 *      into VSIn.u/v before the call (see wireframe note below).
 *
 *   2. Clip reject — if all 3 vertices are behind the near plane, skip.
 *
 *   3. Perspective divide → NDC → screen cell coordinates.
 *      Y axis is flipped: NDC +Y is up, screen +Y is down.
 *
 *   4. Back-face cull — 2D signed area of screen triangle.
 *      Negative area = triangle faces away from camera = skip.
 *
 *   5. Bounding box clamped to [0, cols-1] × [0, rows-1].
 *
 *   6. For each cell in bbox:
 *        a. Barycentric test — skip if outside triangle.
 *        b. Z-interpolate (NDC z) — z-test against zbuf.
 *           Write zbuf and continue only if closer.
 *        c. Interpolate VSOut fields → FSIn using barycentric coords.
 *           world_pos, world_nrm, u, v, custom[4] all interpolated.
 *        d. Fragment shader → FSOut.
 *        e. If not discarded: luma_to_cell → write cbuf.
 *
 * Wireframe barycentric injection:
 *   vert_wire reads the per-vertex barycentric coord from VSIn.u/v
 *   (we repurpose u/v to carry bary[0] and bary[1]; bary[2] = 1-u-v).
 *   The pipeline sets VSIn.u/v to:
 *     vertex 0 → u=1, v=0   → custom = (1, 0, 0)
 *     vertex 1 → u=0, v=1   → custom = (0, 1, 0)
 *     vertex 2 → u=0, v=0   → custom = (0, 0, 1)  (1-0-0=1 → bary[2])
 *   After barycentric interpolation, custom[0..2] holds the true
 *   barycentric coordinates of each fragment.
 */
static void pipeline_draw_mesh(Framebuffer   *fb,
                                const Mesh    *mesh,
                                ShaderProgram *sh,
                                bool           is_wire)
{
    int cols=fb->cols, rows=fb->rows;

    /* per-vertex barycentric coords for wireframe injection */
    static const float wire_u[3] = {1.f, 0.f, 0.f};
    static const float wire_v[3] = {0.f, 1.f, 0.f};

    for(int ti=0; ti<mesh->ntri; ti++){
        const Triangle *tri = &mesh->tris[ti];
        VSOut vo[3];

        for(int vi=0; vi<3; vi++){
            const Vertex *vtx = &mesh->verts[tri->v[vi]];
            VSIn in;
            in.pos    = vtx->pos;
            in.normal = vtx->normal;
            /*
             * Wireframe: inject barycentric coord into u/v.
             * vert_wire copies u→custom[0], v→custom[1],
             * and (1-u-v)→custom[2] is computed by frag_wire
             * from custom[0]+custom[1] (stored as b2=1-b0-b1).
             * For vertex 2: u=0,v=0 → custom=(0,0,1) ✓
             */
            in.u = is_wire ? wire_u[vi] : vtx->u;
            in.v = is_wire ? wire_v[vi] : vtx->v;

            memset(&vo[vi], 0, sizeof vo[vi]);
            sh->vert(&in, &vo[vi], sh->vert_uni);

            /* for wireframe: store bary coords in custom */
            if(is_wire){
                vo[vi].custom[0] = wire_u[vi];
                vo[vi].custom[1] = wire_v[vi];
                vo[vi].custom[2] = 1.f - wire_u[vi] - wire_v[vi];
            }
        }

        /* near-plane clip reject */
        if(vo[0].clip_pos.w<0.001f &&
           vo[1].clip_pos.w<0.001f &&
           vo[2].clip_pos.w<0.001f) continue;

        /* perspective divide → screen */
        float sx[3], sy[3], sz[3];
        for(int vi=0; vi<3; vi++){
            float w = vo[vi].clip_pos.w;
            if(fabsf(w)<1e-6f) w=1e-6f;
            sx[vi]=( vo[vi].clip_pos.x/w+1.f)*0.5f*(float)cols;
            sy[vi]=(-vo[vi].clip_pos.y/w+1.f)*0.5f*(float)rows;
            sz[vi]=  vo[vi].clip_pos.z/w;
        }

        /* back-face cull — skip CCW-wound triangles facing away */
        float area=(sx[1]-sx[0])*(sy[2]-sy[0])
                  -(sx[2]-sx[0])*(sy[1]-sy[0]);
        if(area<=0.f) continue;

        /* bounding box */
        float mnx=fminf(sx[0],fminf(sx[1],sx[2]));
        float mny=fminf(sy[0],fminf(sy[1],sy[2]));
        float mxx=fmaxf(sx[0],fmaxf(sx[1],sx[2]));
        float mxy=fmaxf(sy[0],fmaxf(sy[1],sy[2]));
        int x0=(int)fmaxf(0.f,        floorf(mnx));
        int x1=(int)fminf(cols-1.f,    ceilf(mxx));
        int y0=(int)fmaxf(0.f,        floorf(mny));
        int y1=(int)fminf(rows-1.f,    ceilf(mxy));

        for(int py=y0; py<=y1; py++){
            for(int px=x0; px<=x1; px++){

                float b[3];
                barycentric(sx,sy, px+0.5f, py+0.5f, b);
                if(b[0]<0.f||b[1]<0.f||b[2]<0.f) continue;

                float z=b[0]*sz[0]+b[1]*sz[1]+b[2]*sz[2];
                int   idx=py*cols+px;
                if(z>=fb->zbuf[idx]) continue;
                fb->zbuf[idx]=z;

                /* interpolate VSOut → FSIn */
                FSIn fsin;
                fsin.world_pos=v3_bary(vo[0].world_pos,
                                        vo[1].world_pos,
                                        vo[2].world_pos, b[0],b[1],b[2]);
                fsin.world_nrm=v3_norm(
                               v3_bary(vo[0].world_nrm,
                                        vo[1].world_nrm,
                                        vo[2].world_nrm, b[0],b[1],b[2]));
                fsin.u  =b[0]*vo[0].u+b[1]*vo[1].u+b[2]*vo[2].u;
                fsin.v  =b[0]*vo[0].v+b[1]*vo[1].v+b[2]*vo[2].v;
                fsin.px =px; fsin.py=py;
                for(int c=0;c<4;c++)
                    fsin.custom[c]=b[0]*vo[0].custom[c]
                                  +b[1]*vo[1].custom[c]
                                  +b[2]*vo[2].custom[c];

                /* fragment shader */
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
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Mesh          mesh;
    float         angle_x, angle_y;
    bool          paused;

    ShaderIdx     shade_idx;
    ShaderProgram shader;

    Uniforms      uni;
    ToonUniforms  toon_uni;
} Scene;

static void scene_build_shader(Scene *s)
{
    switch(s->shade_idx){
    case SH_PHONG:
        s->shader=(ShaderProgram){vert_default, frag_phong,   &s->uni,      &s->uni};
        break;
    case SH_TOON:
        s->toon_uni.base  = s->uni;
        s->toon_uni.bands = 4;
        s->shader=(ShaderProgram){vert_default, frag_toon,    &s->uni,      &s->toon_uni};
        break;
    case SH_NORMALS:
        s->shader=(ShaderProgram){vert_normals, frag_normals, &s->uni,      &s->uni};
        break;
    case SH_WIRE:
        s->shader=(ShaderProgram){vert_wire,    frag_wire,    &s->uni,      &s->uni};
        break;
    default: break;
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->mesh      = tessellate_torus();
    s->shade_idx = SH_PHONG;
    s->paused    = false;

    s->uni.light_pos  = v3(3.f, 4.f, 3.f);
    s->uni.light_col  = v3(1.f, 1.f, 1.f);
    s->uni.ambient    = v3(0.07f, 0.07f, 0.07f);
    s->uni.shininess  = 48.f;
    s->uni.cam_pos    = v3(0.f, 0.f, CAM_DIST);
    s->uni.obj_color  = v3(0.3f, 0.7f, 0.9f);   /* torus — cyan-blue */

    s->uni.view = m4_lookat(s->uni.cam_pos, v3(0,0,0), v3(0,1,0));

    float aspect = (float)(cols*CELL_W) / (float)(rows*CELL_H);
    s->uni.proj  = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);

    scene_build_shader(s);
}

static void scene_rebuild_proj(Scene *s, int cols, int rows){
    float aspect=(float)(cols*CELL_W)/(float)(rows*CELL_H);
    s->uni.proj=m4_perspective(CAM_FOV,aspect,CAM_NEAR,CAM_FAR);
}

static void scene_tick(Scene *s, float dt)
{
    if(s->paused) return;
    s->angle_y += ROT_Y * dt;
    s->angle_x += ROT_X * dt;

    Mat4 ry  = m4_rotate_y(s->angle_y);
    Mat4 rx  = m4_rotate_x(s->angle_x);
    s->uni.model     = m4_mul(ry, rx);
    s->uni.mvp       = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
    s->uni.norm_mat  = m4_normal_mat(s->uni.model);

    /* keep toon uniforms in sync with base each frame */
    s->toon_uni.base = s->uni;
}

static void scene_draw(Scene *s, Framebuffer *fb)
{
    fb_clear(fb);
    bool is_wire = (s->shade_idx == SH_WIRE);
    pipeline_draw_mesh(fb, &s->mesh, &s->shader, is_wire);
    fb_blit(fb);
}

static void scene_next_shader(Scene *s)
{
    s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
    scene_build_shader(s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s){
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr,TRUE); keypad(stdscr,TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s){ (void)s; endwin(); }
static void screen_resize(Screen *s){
    endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols);
}

static void screen_draw_hud(const Screen *s, const Scene *sc, double fps)
{
    char buf[HUD_COLS+1];
    snprintf(buf,sizeof buf," %5.1f fps  shader:[%s]%s ",
             fps, k_shader_names[sc->shade_idx],
             sc->paused?" PAUSED":"       ");
    int hx=s->cols-HUD_COLS; if(hx<0)hx=0;
    attron(COLOR_PAIR(3)|A_BOLD);
    mvprintw(0,hx,"%s",buf);
    attroff(COLOR_PAIR(3)|A_BOLD);

    attron(COLOR_PAIR(5));
    mvprintw(s->rows-1,0,"s=shader  space=pause  q=quit");
    attroff(COLOR_PAIR(5));
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
static void on_exit  (int sig){ (void)sig; g_app.running=0;     }
static void on_resize(int sig){ (void)sig; g_app.need_resize=1; }
static void cleanup  (void)   { endwin(); }

static void app_do_resize(App *app){
    screen_resize(&app->screen);
    fb_free(&app->fb);
    fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
    scene_rebuild_proj(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize=0;
}

static bool app_handle_key(App *app, int ch){
    switch(ch){
    case 'q': case 'Q': case 27: return false;
    case ' ': app->scene.paused = !app->scene.paused; break;
    case 's': case 'S': scene_next_shader(&app->scene); break;
    default: break;
    }
    return true;
}

static int64_t clock_ns(void){
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return (int64_t)t.tv_sec*NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns){
    if(ns<=0) return;
    struct timespec r={ .tv_sec=(time_t)(ns/NS_PER_SEC),
                        .tv_nsec=(long)(ns%NS_PER_SEC) };
    nanosleep(&r,NULL);
}

int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,  on_exit);
    signal(SIGTERM, on_exit);
    signal(SIGWINCH,on_resize);

    App *app=&g_app;
    app->running=1;

    screen_init(&app->screen);
    fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time=clock_ns();
    int64_t fps_acc=0;
    int     fps_cnt=0;
    double  fps_disp=0.0;

    while(app->running){

        if(app->need_resize){
            app_do_resize(app);
            frame_time=clock_ns();
        }

        int64_t now=clock_ns();
        int64_t dt =now-frame_time;
        frame_time =now;
        if(dt>100*NS_PER_MS) dt=100*NS_PER_MS;
        float dt_sec=(float)dt/(float)NS_PER_SEC;

        scene_tick(&app->scene, dt_sec);

        fps_cnt++;
        fps_acc+=dt;
        if(fps_acc>=FPS_UPDATE_MS*NS_PER_MS){
            fps_disp=(double)fps_cnt/((double)fps_acc/(double)NS_PER_SEC);
            fps_cnt=0; fps_acc=0;
        }

        erase();
        scene_draw(&app->scene, &app->fb);
        screen_draw_hud(&app->screen, &app->scene, fps_disp);
        screen_present();

        int ch=getch();
        if(ch!=ERR && !app_handle_key(app,ch)) app->running=0;

        int64_t elapsed=clock_ns()-frame_time+dt;
        clock_sleep_ns(NS_PER_SEC/FPS_TARGET - elapsed);
    }

    mesh_free(&app->scene.mesh);
    fb_free(&app->fb);
    screen_free(&app->screen);
    return 0;
}
