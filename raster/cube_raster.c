/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cube_raster.c  —  ncurses software rasterizer, cube demo
 *
 * One primitive: axis-aligned unit cube, 6 faces × 2 triangles = 12 triangles.
 * Four shaders cycled with  s : phong / toon / normals / wireframe
 *
 * The cube is a good complement to the torus demo:
 *   - Flat faces make per-face normal shading visible as hard steps
 *   - Sharp edges make the barycentric wireframe threshold very clean
 *   - Toon banding is dramatic because each face is uniformly lit
 *   - Normal shader shows 6 distinct colours — one per face direction
 *
 * Pipeline identical to torus_raster.c.
 * Only §4 (tessellation) changes — everything else is a direct copy
 * with cube-specific geometry constants.
 *
 * Keys:
 *   s / S     cycle shader  (phong → toon → normals → wireframe)
 *   c / C     toggle back-face culling
 *   + / =     zoom in
 *   -         zoom out
 *   space     pause / resume rotation
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra cube_raster.c -o cube -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  math         Vec3 Vec4 Mat4
 *   §3  shaders      VSIn VSOut FSIn FSOut + 4 vert/frag pairs
 *   §4  mesh         cube tessellation (6 faces, flat normals)
 *   §5  framebuffer  zbuf cbuf dither paulbourke blit
 *   §6  pipeline     vertex transform · rasterize · draw
 *   §7  scene        uniforms · tick · draw · shader swap
 *   §8  screen       ncurses init/resize/hud/present
 *   §9  app          dt loop · input · resize · cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Same software rasterization pipeline as torus_raster.c.
 *                  The cube demonstrates flat normals (per-face, not per-vertex).
 *                  Each face has a single outward normal; vertices at shared
 *                  edges are duplicated (3 vertices per face corner) to allow
 *                  distinct normals from each face's perspective.
 *
 * Math           : Flat normals: N is constant across each face = one of
 *                  ±(1,0,0), ±(0,1,0), ±(0,0,1) in model space.
 *                  Smooth vs flat normals: smooth normals are averaged at shared
 *                  vertices (good for organic shapes); flat normals show the
 *                  polyhedron facets (good for hard-surface models like the cube).
 *                  Toon shading: quantise the diffuse term to 3 bands —
 *                  the dramatic band at 50% simulates cel-shading.
 *
 * Rendering      : Wireframe mode via barycentric edge detection: when any
 *                  barycentric coordinate < WIRE_THRESHOLD, draw as edge colour.
 *                  This avoids separate edge rendering; the wireframe emerges from
 *                  the rasterisation loop itself.
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
    HUD_COLS      = 46,
};

#define CAM_FOV   (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR  0.1f
#define CAM_FAR   100.0f
#define CAM_DIST      2.8f   /* default camera Z distance */
#define CAM_DIST_MIN  1.0f   /* closest zoom              */
#define CAM_DIST_MAX  8.0f   /* furthest zoom             */
#define CAM_ZOOM_STEP 0.2f   /* distance change per keypress */

/*
 * Cube half-extent — vertices at ±CUBE_S on each axis.
 * 0.75 gives a cube that fills the terminal nicely without clipping.
 */
#define CUBE_S  0.75f

/*
 * Rotation speeds.
 * Different X and Y rates give a tumbling motion that shows all
 * six faces over time without ever looking stuck on one axis.
 */
#define ROT_Y  0.55f   /* radians / second */
#define ROT_X  0.37f

/*
 * Wireframe edge threshold — same barycentric detection as torus.
 * Cube triangles are larger in screen space so a slightly smaller
 * threshold (0.06) keeps lines thin and sharp.
 */
#define WIRE_THRESH  0.06f

/* Paul Bourke ASCII density ramp — darkest → brightest */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* Bayer 4×4 ordered dither matrix */
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};

/*
 * CELL_W / CELL_H — terminal cell aspect ratio correction.
 * Passed to m4_perspective as (cols*CELL_W)/(rows*CELL_H).
 * Without this the cube appears vertically stretched.
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

static inline Vec3  v3(float x,float y,float z){ return (Vec3){x,y,z}; }
static inline Vec3  v3_add(Vec3 a,Vec3 b)  { return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline Vec3  v3_sub(Vec3 a,Vec3 b)  { return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline Vec3  v3_scale(Vec3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline Vec3  v3_neg(Vec3 a)          { return v3(-a.x,-a.y,-a.z); }
static inline float v3_dot(Vec3 a,Vec3 b)   { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3_len(Vec3 a)          { return sqrtf(v3_dot(a,a)); }
static inline Vec3  v3_norm(Vec3 a){
    float l=v3_len(a); return l>1e-7f ? v3_scale(a,1.f/l) : v3(0,1,0);
}
static inline Vec3 v3_reflect(Vec3 d,Vec3 n){
    return v3_sub(d, v3_scale(n, 2.f*v3_dot(d,n)));
}
static inline Vec3 v3_bary(Vec3 a,Vec3 b,Vec3 c,float u,float v,float w){
    return v3(u*a.x+v*b.x+w*c.x,
              u*a.y+v*b.y+w*c.y,
              u*a.z+v*b.z+w*c.z);
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
    Vec3 r=v3_norm(v3(f.z*up.y-f.y*up.z,
                       f.x*up.z-f.z*up.x,
                       f.y*up.x-f.x*up.y));
    Vec3 u=v3(r.y*f.z-r.z*f.y, r.z*f.x-r.x*f.z, r.x*f.y-r.y*f.x);
    Mat4 m=m4_identity();
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-v3_dot(r,eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-v3_dot(u,eye);
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]= v3_dot(f,eye);
    return m;
}

/*
 * m4_normal_mat — cofactor of upper-left 3×3.
 * Correctly transforms normals under non-uniform scale.
 * For the cube (uniform scale) this equals the rotation part of model,
 * but computing it properly means non-uniform scale just works later.
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

typedef void (*VertShaderFn)(const VSIn *in,  VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in,  FSOut *out, const void *uni);
typedef struct { VertShaderFn vert; FragShaderFn frag; const void *vert_uni; const void *frag_uni; } ShaderProgram;

/* ── uniforms ─────────────────────────────────────────────────────── */

typedef struct {
    Mat4  model, view, proj, mvp, norm_mat;
    Vec3  light_pos, light_col, ambient, cam_pos, obj_color;
    float shininess;
} Uniforms;

typedef struct { Uniforms base; int bands; } ToonUniforms;

/* ── vertex shaders ──────────────────────────────────────────────── */

static void vert_default(const VSIn *in, VSOut *out, const void *u_)
{
    const Uniforms *u=(const Uniforms*)u_;
    out->clip_pos  = m4_mul_v4(u->mvp, v4(in->pos.x,in->pos.y,in->pos.z,1.f));
    out->world_pos = m4_pt (u->model,    in->pos);
    out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
    out->u=in->u; out->v=in->v;
    out->custom[0]=out->custom[1]=out->custom[2]=out->custom[3]=0.f;
}

static void vert_normals(const VSIn *in, VSOut *out, const void *u_)
{
    const Uniforms *u=(const Uniforms*)u_;
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
 * vert_wire — barycentric coord injection for wireframe.
 * Pipeline sets VSIn.u/v to the per-vertex barycentric identity
 * vector before calling this shader (see §6 pipeline_draw_mesh).
 * We copy u→custom[0], v→custom[1]; pipeline sets custom[2]=1-u-v.
 */
static void vert_wire(const VSIn *in, VSOut *out, const void *u_)
{
    const Uniforms *u=(const Uniforms*)u_;
    out->clip_pos  = m4_mul_v4(u->mvp, v4(in->pos.x,in->pos.y,in->pos.z,1.f));
    out->world_pos = m4_pt (u->model,    in->pos);
    out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
    out->u=in->u; out->v=in->v;
    out->custom[0]=0.f; out->custom[1]=0.f;
    out->custom[2]=0.f; out->custom[3]=0.f;
}

/* ── fragment shaders ────────────────────────────────────────────── */

/*
 * frag_phong — Blinn-Phong shading with gamma correction.
 *
 * The cube's flat-shaded faces make the lighting very clear —
 * each face receives uniform diffuse because all fragments on a
 * flat face share the same interpolated normal.
 * The specular highlight appears as a bright patch on whichever
 * face happens to be oriented toward the half-vector.
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
 * frag_toon — 4-band quantised diffuse + hard specular.
 *
 * On the cube this is especially striking: each face is entirely
 * one band — hard steps with no gradient across a face.
 * The silhouette between a lit and unlit face is a perfectly
 * sharp boundary, which is the defining look of cel shading.
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
 *
 * The cube has 6 faces with 6 distinct normals, so this shader
 * renders as 6 solid colour patches — one per face direction:
 *   +X = warm red        -X = cool teal
 *   +Y = bright green    -Y = dark purple
 *   +Z = blue            -Z = yellow-orange
 * The colours change smoothly as the cube rotates because the
 * world-space normal rotates with the model.
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
 *
 * min(custom[0..2]) is the distance to the nearest edge.
 * Below WIRE_THRESH: draw edge character.
 * Above WIRE_THRESH: discard (transparent interior).
 *
 * On the cube the wireframe shows a clean 12-edge outline.
 * The diagonal edges from the quad-to-triangle split are visible,
 * which honestly looks good — it shows the mesh structure clearly.
 * If you want to hide the diagonal, you would need to mark shared
 * edges and suppress them — unnecessary complexity for a demo.
 */
static void frag_wire(const FSIn *in, FSOut *out, const void *u_)
{
    (void)u_;
    float b0=in->custom[0], b1=in->custom[1], b2=in->custom[2];
    float edge=fminf(b0, fminf(b1,b2));
    if(edge>WIRE_THRESH){ out->discard=true; return; }
    float t=edge/WIRE_THRESH;
    out->color  =v3(0.9f-t*0.3f, 0.9f-t*0.3f, 0.9f-t*0.3f);
    out->discard=false;
}

typedef enum { SH_PHONG=0, SH_TOON, SH_NORMALS, SH_WIRE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[]={"phong","toon","normals","wire"};

/* ===================================================================== */
/* §4  mesh — cube tessellation                                           */
/* ===================================================================== */

typedef struct { Vec3 pos; Vec3 normal; float u,v; } Vertex;
typedef struct { int v[3]; } Triangle;
typedef struct { Vertex *verts; int nvert; Triangle *tris; int ntri; } Mesh;

static void mesh_free(Mesh *m){ free(m->verts); free(m->tris); *m=(Mesh){0}; }

/*
 * tessellate_cube — 6 faces, each a quad split into 2 triangles.
 *
 * Each face has 4 unique vertices with the same outward normal —
 * flat shading, no shared vertices between faces.
 * Sharing vertices would require averaged normals which would round
 * the corners — correct for a sphere, wrong for a cube.
 *
 * Winding order: CCW when viewed from outside (from the direction
 * the normal points).  The pipeline culls CW-wound back faces.
 *
 * UV layout: each face maps [0,1]² independently.
 *   v0 = bottom-left  (0,1)
 *   v1 = bottom-right (1,1)
 *   v2 = top-right    (1,0)
 *   v3 = top-left     (0,0)
 *
 * Face order: +X, -X, +Y, -Y, +Z, -Z
 * Each described by: normal + 4 corner positions in CCW order.
 */
static Mesh tessellate_cube(void)
{
    float s = CUBE_S;

    /*
     * 6 faces × 4 vertices each = 24 vertices
     * 6 faces × 2 triangles each = 12 triangles
     */
    static const float face_nrm[6][3] = {
        { 1, 0, 0}, {-1, 0, 0},
        { 0, 1, 0}, { 0,-1, 0},
        { 0, 0, 1}, { 0, 0,-1},
    };

    /*
     * Corners for each face in CCW winding viewed from outside
     * (i.e. from the direction the face normal points toward).
     *
     * Right-hand rule: curl fingers v0→v1→v2, thumb points toward viewer.
     * The pipeline's screen-space signed area test (after Y-flip) passes
     * triangles with area > 0, which corresponds to CCW in NDC.
     *
     * Verified per face:
     *   +X: looking from +X toward origin. Right = -Z, Up = +Y.
     *       v0=( 1,-1, 1) v1=( 1, 1, 1) v2=( 1, 1,-1) v3=( 1,-1,-1)  CCW ✓
     *   -X: looking from -X toward origin. Right = +Z, Up = +Y.
     *       v0=(-1,-1,-1) v1=(-1, 1,-1) v2=(-1, 1, 1) v3=(-1,-1, 1)  CCW ✓
     *   +Y: looking down from +Y. Right = +X, Forward = +Z.
     *       v0=(-1, 1, 1) v1=( 1, 1, 1) v2=( 1, 1,-1) v3=(-1, 1,-1)  CCW ✓
     *   -Y: looking up from -Y. Right = +X, Forward = -Z.
     *       v0=(-1,-1,-1) v1=( 1,-1,-1) v2=( 1,-1, 1) v3=(-1,-1, 1)  CCW ✓
     *   +Z: looking from +Z toward origin. Right = +X, Up = +Y.
     *       v0=(-1,-1, 1) v1=( 1,-1, 1) v2=( 1, 1, 1) v3=(-1, 1, 1)  CCW ✓
     *   -Z: looking from -Z toward origin. Right = -X, Up = +Y.
     *       v0=( 1,-1,-1) v1=(-1,-1,-1) v2=(-1, 1,-1) v3=( 1, 1,-1)  CCW ✓
     */
    static const float face_vtx[6][4][3] = {
        /* +X */ {{ 1,-1, 1},{ 1, 1, 1},{ 1, 1,-1},{ 1,-1,-1}},
        /* -X */ {{-1,-1,-1},{-1, 1,-1},{-1, 1, 1},{-1,-1, 1}},
        /* +Y */ {{-1, 1, 1},{ 1, 1, 1},{ 1, 1,-1},{-1, 1,-1}},
        /* -Y */ {{-1,-1,-1},{ 1,-1,-1},{ 1,-1, 1},{-1,-1, 1}},
        /* +Z */ {{-1,-1, 1},{ 1,-1, 1},{ 1, 1, 1},{-1, 1, 1}},
        /* -Z */ {{ 1,-1,-1},{-1,-1,-1},{-1, 1,-1},{ 1, 1,-1}},
    };

    static const float face_uv[4][2] = {
        {0,1},{1,1},{1,0},{0,0}
    };

    Mesh m;
    m.verts = malloc(24 * sizeof(Vertex));
    m.tris  = malloc(12 * sizeof(Triangle));
    m.nvert = 0;
    m.ntri  = 0;

    for(int f=0; f<6; f++){
        Vec3 n = v3(face_nrm[f][0], face_nrm[f][1], face_nrm[f][2]);
        int base = m.nvert;

        for(int i=0; i<4; i++){
            Vec3 p = v3(face_vtx[f][i][0]*s,
                        face_vtx[f][i][1]*s,
                        face_vtx[f][i][2]*s);
            m.verts[m.nvert++] = (Vertex){ p, n, face_uv[i][0], face_uv[i][1] };
        }

        /*
         * Split quad into 2 CCW triangles:
         *   tri 0: v0, v1, v2  (bottom-left → bottom-right → top-right)
         *   tri 1: v0, v2, v3  (bottom-left → top-right → top-left)
         */
        m.tris[m.ntri++] = (Triangle){{base,   base+1, base+2}};
        m.tris[m.ntri++] = (Triangle){{base,   base+2, base+3}};
    }

    return m;
}

/* ===================================================================== */
/* §5  framebuffer                                                        */
/* ===================================================================== */

typedef struct { char ch; int color_pair; bool bold; } Cell;
typedef struct { float *zbuf; Cell *cbuf; int cols,rows; } Framebuffer;

static void fb_alloc(Framebuffer *fb,int cols,int rows){
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

static Cell luma_to_cell(float luma,int px,int py)
{
    float thr=k_bayer[py&3][px&3];
    float d=luma+(thr-0.5f)*0.15f;
    d=d<0.f?0.f:d>1.f?1.f:d;
    int  idx=(int)(d*(BOURKE_LEN-1));
    char ch=k_bourke[idx];
    int  cp=1+(int)(d*6.f); if(cp>7)cp=7;
    bool bold=d>0.6f;
    return (Cell){ch,cp,bold};
}

static void fb_blit(const Framebuffer *fb)
{
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
/* §6  pipeline                                                           */
/* ===================================================================== */

static void barycentric(const float sx[3],const float sy[3],
                        float px,float py,float b[3])
{
    float d=(sy[1]-sy[2])*(sx[0]-sx[2])+(sx[2]-sx[1])*(sy[0]-sy[2]);
    if(fabsf(d)<1e-6f){b[0]=b[1]=b[2]=-1.f;return;}
    b[0]=((sy[1]-sy[2])*(px-sx[2])+(sx[2]-sx[1])*(py-sy[2]))/d;
    b[1]=((sy[2]-sy[0])*(px-sx[2])+(sx[0]-sx[2])*(py-sy[2]))/d;
    b[2]=1.f-b[0]-b[1];
}

/*
 * pipeline_draw_mesh — rasterization pipeline, identical to torus_raster.c.
 *
 * is_wire flag:
 *   true  → inject barycentric identity coords into VSIn.u/v before
 *            each vertex shader call, then write custom[0..2] in VSOut.
 *   false → use mesh u/v normally, custom unused.
 *
 * The cube has only 12 triangles so the inner loop is fast.
 * Each face covers a large screen area, making the per-fragment
 * cost the bottleneck — not the triangle setup.
 */
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
            in.u = is_wire ? wu[vi] : vtx->u;
            in.v = is_wire ? wv[vi] : vtx->v;
            memset(&vo[vi],0,sizeof vo[vi]);
            sh->vert(&in,&vo[vi],sh->vert_uni);
            if(is_wire){
                vo[vi].custom[0]=wu[vi];
                vo[vi].custom[1]=wv[vi];
                vo[vi].custom[2]=1.f-wu[vi]-wv[vi];
            }
        }

        /* near clip reject */
        if(vo[0].clip_pos.w<0.001f &&
           vo[1].clip_pos.w<0.001f &&
           vo[2].clip_pos.w<0.001f) continue;

        /* perspective divide → screen */
        float sx[3],sy[3],sz[3];
        for(int vi=0;vi<3;vi++){
            float w=vo[vi].clip_pos.w; if(fabsf(w)<1e-6f)w=1e-6f;
            sx[vi]=( vo[vi].clip_pos.x/w+1.f)*0.5f*(float)cols;
            sy[vi]=(-vo[vi].clip_pos.y/w+1.f)*0.5f*(float)rows;
            sz[vi]=  vo[vi].clip_pos.z/w;
        }

        /* back-face cull — skipped when cull_backface is false */
        float area=(sx[1]-sx[0])*(sy[2]-sy[0])
                  -(sx[2]-sx[0])*(sy[1]-sy[0]);
        if(cull_backface && area<=0.f) continue;

        /* bounding box */
        int x0=(int)fmaxf(0.f,      floorf(fminf(sx[0],fminf(sx[1],sx[2]))));
        int x1=(int)fminf(cols-1.f,  ceilf(fmaxf(sx[0],fmaxf(sx[1],sx[2]))));
        int y0=(int)fmaxf(0.f,      floorf(fminf(sy[0],fminf(sy[1],sy[2]))));
        int y1=(int)fminf(rows-1.f,  ceilf(fmaxf(sy[0],fmaxf(sy[1],sy[2]))));

        for(int py=y0;py<=y1;py++){
            for(int px=x0;px<=x1;px++){
                float b[3];
                barycentric(sx,sy,px+0.5f,py+0.5f,b);
                if(b[0]<0.f||b[1]<0.f||b[2]<0.f) continue;

                float z=b[0]*sz[0]+b[1]*sz[1]+b[2]*sz[2];
                int idx=py*cols+px;
                if(z>=fb->zbuf[idx]) continue;
                fb->zbuf[idx]=z;

                FSIn fsin;
                fsin.world_pos=v3_bary(vo[0].world_pos,
                                        vo[1].world_pos,
                                        vo[2].world_pos,b[0],b[1],b[2]);
                fsin.world_nrm=v3_norm(
                               v3_bary(vo[0].world_nrm,
                                        vo[1].world_nrm,
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
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Mesh          mesh;
    float         angle_x, angle_y;
    float         cam_dist;      /* current zoom distance — changed by +/- */
    bool          paused;
    bool          cull_backface; /* c toggles — false shows inner faces    */
    ShaderIdx     shade_idx;
    ShaderProgram shader;
    Uniforms      uni;
    ToonUniforms  toon_uni;
} Scene;

static void scene_build_shader(Scene *s)
{
    switch(s->shade_idx){
    case SH_PHONG:
        s->shader=(ShaderProgram){vert_default, frag_phong,   &s->uni,      &s->uni};      break;
    case SH_TOON:
        s->toon_uni.base=s->uni; s->toon_uni.bands=4;
        s->shader=(ShaderProgram){vert_default, frag_toon,    &s->uni,      &s->toon_uni}; break;
    case SH_NORMALS:
        s->shader=(ShaderProgram){vert_normals, frag_normals, &s->uni,      &s->uni};      break;
    case SH_WIRE:
        s->shader=(ShaderProgram){vert_wire,    frag_wire,    &s->uni,      &s->uni};      break;
    default: break;
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s,0,sizeof *s);
    s->mesh           = tessellate_cube();
    s->shade_idx      = SH_PHONG;
    s->cam_dist       = CAM_DIST;
    s->cull_backface  = true;

    s->uni.light_pos  = v3(4.f, 5.f, 4.f);
    s->uni.light_col  = v3(1.f, 1.f, 1.f);
    s->uni.ambient    = v3(0.07f,0.07f,0.07f);
    s->uni.shininess  = 64.f;
    s->uni.cam_pos    = v3(0.f, 0.f, s->cam_dist);
    s->uni.obj_color  = v3(0.9f, 0.55f, 0.15f);   /* warm orange */

    s->uni.view = m4_lookat(s->uni.cam_pos, v3(0,0,0), v3(0,1,0));
    float aspect=(float)(cols*CELL_W)/(float)(rows*CELL_H);
    s->uni.proj = m4_perspective(CAM_FOV,aspect,CAM_NEAR,CAM_FAR);

    scene_build_shader(s);
}

/*
 * scene_set_zoom — update camera position and view matrix.
 * Called whenever cam_dist changes.  Rebuilds cam_pos and view so
 * the change takes effect on the very next scene_tick() → mvp rebuild.
 */
static void scene_set_zoom(Scene *s)
{
    s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
    s->uni.view    = m4_lookat(s->uni.cam_pos, v3(0,0,0), v3(0,1,0));
}

static void scene_rebuild_proj(Scene *s, int cols, int rows){
    float aspect=(float)(cols*CELL_W)/(float)(rows*CELL_H);
    s->uni.proj=m4_perspective(CAM_FOV,aspect,CAM_NEAR,CAM_FAR);
}

static void scene_tick(Scene *s, float dt)
{
    if(s->paused) return;
    s->angle_y += ROT_Y*dt;
    s->angle_x += ROT_X*dt;
    Mat4 ry=m4_rotate_y(s->angle_y);
    Mat4 rx=m4_rotate_x(s->angle_x);
    s->uni.model    = m4_mul(ry,rx);
    s->uni.mvp      = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
    s->uni.norm_mat = m4_normal_mat(s->uni.model);
    s->toon_uni.base= s->uni;
}

static void scene_draw(Scene *s, Framebuffer *fb)
{
    fb_clear(fb);
    pipeline_draw_mesh(fb, &s->mesh, &s->shader,
                       (s->shade_idx==SH_WIRE), s->cull_backface);
    fb_blit(fb);
}

static void scene_next_shader(Scene *s){
    s->shade_idx=(ShaderIdx)((s->shade_idx+1)%SH_COUNT);
    scene_build_shader(s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols,rows; } Screen;

static void screen_init(Screen *s){
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr,TRUE); keypad(stdscr,TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr,s->rows,s->cols);
}
static void screen_free(Screen *s){ (void)s; endwin(); }
static void screen_resize(Screen *s){
    endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols);
}

static void screen_draw_hud(const Screen *s,const Scene *sc,double fps)
{
    char buf[HUD_COLS+1];
    snprintf(buf,sizeof buf," %5.1f fps  [%s]  z:%.1f  cull:%s%s ",
             fps, k_shader_names[sc->shade_idx],
             sc->cam_dist,
             sc->cull_backface ? "on " : "off",
             sc->paused ? " PAUSED" : "");
    int hx=s->cols-HUD_COLS; if(hx<0)hx=0;
    attron(COLOR_PAIR(3)|A_BOLD);
    mvprintw(0,hx,"%s",buf);
    attroff(COLOR_PAIR(3)|A_BOLD);
    attron(COLOR_PAIR(5));
    mvprintw(s->rows-1,0,"s=shader  c=cull  +/-=zoom  space=pause  q=quit");
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
    fb_alloc(&app->fb,app->screen.cols,app->screen.rows);
    scene_rebuild_proj(&app->scene,app->screen.cols,app->screen.rows);
    app->need_resize=0;
}

static bool app_handle_key(App *app, int ch){
    Scene *s = &app->scene;
    switch(ch){
    case 'q': case 'Q': case 27: return false;
    case ' ': s->paused = !s->paused; break;
    case 's': case 'S': scene_next_shader(s); break;
    case 'c': case 'C': s->cull_backface = !s->cull_backface; break;
    case '=': case '+':                             /* zoom in  */
        s->cam_dist -= CAM_ZOOM_STEP;
        if(s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        scene_set_zoom(s);
        break;
    case '-':                                       /* zoom out */
        s->cam_dist += CAM_ZOOM_STEP;
        if(s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        scene_set_zoom(s);
        break;
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

    App *app=&g_app;
    app->running=1;

    screen_init(&app->screen);
    fb_alloc(&app->fb,app->screen.cols,app->screen.rows);
    scene_init(&app->scene,app->screen.cols,app->screen.rows);

    int64_t frame_time=clock_ns();
    int64_t fps_acc=0; int fps_cnt=0; double fps_disp=0.0;

    while(app->running){

        if(app->need_resize){ app_do_resize(app); frame_time=clock_ns(); }

        int64_t now=clock_ns();
        int64_t dt =now-frame_time;
        frame_time =now;
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
