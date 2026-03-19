/*
 * sphere_raster.c  —  ncurses software rasterizer, sphere demo
 *
 * One primitive: UV sphere, TESS_U×TESS_V tessellation.
 * Four shaders cycled with  s : phong / toon / normals / wireframe
 *
 * The sphere is the best showcase for the pipeline:
 *   - Smooth normals produce a continuous Phong highlight
 *   - Toon banding forms clean latitudinal rings around the surface
 *   - Normal shader gives a full RGB hue map as the sphere rotates
 *   - Wireframe shows the UV grid — latitude/longitude lines
 *
 * Sphere tessellation notes:
 *   Normal == normalised position for a unit sphere centred at origin.
 *   No normal matrix needed for the normal itself, but we still pass it
 *   through the normal_mat transform so non-uniform scale works later.
 *   Pole vertices (top/bottom) share position but have unique normals
 *   per triangle fan — no degenerate normals at poles.
 *
 * Pipeline identical to cube_raster.c.
 * Only §1 config and §4 tessellation change.
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
 *   gcc -std=c11 -O2 -Wall -Wextra sphere_raster.c -o sphere -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  math         Vec3 Vec4 Mat4
 *   §3  shaders      VSIn VSOut FSIn FSOut + 4 vert/frag pairs
 *   §4  mesh         sphere tessellation (UV grid, smooth normals)
 *   §5  framebuffer  zbuf cbuf dither paulbourke blit
 *   §6  pipeline     vertex transform · rasterize · draw
 *   §7  scene        uniforms · tick · draw · shader swap
 *   §8  screen       ncurses init/resize/hud/present
 *   §9  app          dt loop · input · resize · cleanup
 */

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

#define CAM_FOV       (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR      0.1f
#define CAM_FAR       100.0f
#define CAM_DIST      2.6f    /* default camera Z distance */
#define CAM_DIST_MIN  1.0f    /* closest zoom              */
#define CAM_DIST_MAX  8.0f    /* furthest zoom             */
#define CAM_ZOOM_STEP 0.2f    /* distance change per keypress */

/*
 * Sphere radius.
 * 1.0 fills the terminal well at CAM_DIST=2.6 with FOV=55°.
 */
#define SPHERE_R  1.0f

/*
 * Tessellation resolution.
 * TESS_U: longitude slices (around the equator).
 * TESS_V: latitude  stacks (pole to pole).
 *
 * Higher = rounder sphere, more triangles, slower fill.
 * At terminal resolution 36×24 is a good balance:
 *   - 36 slices → ~10° per slice, smooth silhouette
 *   - 24 stacks → ~7.5° per stack, smooth top/bottom caps
 *   - Total triangles: 36*24*2 = 1728
 *
 * Wireframe at this resolution shows clear latitude/longitude lines
 * without being too dense to read.
 */
#define TESS_U  36
#define TESS_V  24

/*
 * Rotation speeds.
 * Slow X tilt so the poles are visible but the sphere mostly
 * shows its equatorial band — best view for Phong highlight.
 */
#define ROT_Y  0.50f   /* radians / second */
#define ROT_X  0.20f

/*
 * Wireframe edge threshold.
 * Sphere triangles are smaller than cube faces in screen space,
 * so a slightly larger threshold (0.09) keeps lines visible.
 */
#define WIRE_THRESH  0.09f

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
 * Without this the sphere appears vertically stretched.
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
 * For a sphere with uniform scale this equals the rotation block,
 * but computing it properly means adding squash/stretch later just works.
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
 * The sphere's smooth normals produce a continuous specular lobe —
 * the highlight glides across the surface as it rotates, which is
 * the classic "shiny ball" look.  At terminal resolution the dither
 * LUT gives the highlight a pleasing grain rather than a hard step.
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
 * On the sphere the bands form horizontal rings that track the
 * light direction — as the sphere rotates the rings sweep across
 * the surface.  More bands = finer steps; fewer = more graphic.
 * 4 bands at terminal resolution looks clean without being too coarse.
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
 * frag_normals — world normal [-1,1] → RGB [0,1].
 *
 * The sphere is the ideal showcase for this shader: every surface
 * direction is represented, so the full RGB colour cube appears
 * mapped continuously across the surface.  As the sphere rotates
 * the colours shift smoothly — every orientation has a unique hue.
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
 * On the sphere wireframe shows the UV latitude/longitude grid —
 * TESS_V horizontal rings and TESS_U vertical meridians.
 * The poles converge to a fan of triangles rather than a clean ring;
 * this is inherent to UV tessellation and is part of the aesthetic.
 * An icosphere would give uniform triangles but no clean lat/lon lines.
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
/* §4  mesh — sphere tessellation                                         */
/* ===================================================================== */

typedef struct { Vec3 pos; Vec3 normal; float u,v; } Vertex;
typedef struct { int v[3]; } Triangle;
typedef struct { Vertex *verts; int nvert; Triangle *tris; int ntri; } Mesh;

static void mesh_free(Mesh *m){ free(m->verts); free(m->tris); *m=(Mesh){0}; }

/*
 * tessellate_sphere — UV grid over the sphere surface.
 *
 * Parameterisation:
 *   theta ∈ [0, 2π)  — longitude, TESS_U divisions
 *   phi   ∈ [0,  π]  — latitude,  TESS_V divisions (0=north, π=south)
 *
 * Position:
 *   x = R * sin(phi) * cos(theta)
 *   y = R * cos(phi)               ← Y is up
 *   z = R * sin(phi) * sin(theta)
 *
 * Normal:
 *   For a unit sphere, normal == normalised position.
 *   normal = normalise(position) = position / R
 *   This gives perfectly smooth per-vertex normals with no discontinuity
 *   except at the poles (phi=0 and phi=π) where sin(phi)=0 and all
 *   vertices at the same stack share the same position.
 *
 * Pole handling:
 *   The top pole (phi=0) and bottom pole (phi=π) degenerate to a single
 *   point.  We still generate (TESS_U+1) vertices per pole stack so
 *   the UV grid is uniform, but each pole vertex has a unique U coordinate
 *   so the texture seam is handled correctly.  The normal at the top
 *   pole is (0,1,0) and at the bottom is (0,-1,0) regardless of theta.
 *
 * Winding:
 *   CCW when viewed from outside (normal pointing toward camera).
 *   Quad split: (r0,r2,r1) and (r1,r2,r3) — verified CCW for outward
 *   facing quads in a right-handed system with Y-up.
 *
 * Vertex count: (TESS_U+1) * (TESS_V+1)
 * Triangle count: TESS_U * TESS_V * 2
 */
static Mesh tessellate_sphere(void)
{
    int nu=TESS_U, nv=TESS_V;
    float R=SPHERE_R;
    float PI=3.14159265f, PI2=2.f*PI;

    int nvert=(nu+1)*(nv+1);
    int ntri =nu*nv*2;

    Mesh m;
    m.verts=malloc((size_t)nvert*sizeof(Vertex));
    m.tris =malloc((size_t)ntri *sizeof(Triangle));
    m.nvert=0; m.ntri=0;

    for(int j=0;j<=nv;j++){
        float v   =(float)j/(float)nv;
        float phi = v*PI;            /* 0 (north pole) → π (south pole) */
        float sp  =sinf(phi);
        float cp  =cosf(phi);

        for(int i=0;i<=nu;i++){
            float u     =(float)i/(float)nu;
            float theta = u*PI2;
            float st=sinf(theta), ct=cosf(theta);

            Vec3 pos=v3(R*sp*ct, R*cp, R*sp*st);

            /*
             * Normal = normalised position for unit sphere.
             * At poles (sp≈0) this degenerates — we set it explicitly
             * to the pure pole direction to avoid divide-by-zero.
             */
            Vec3 nrm;
            if(sp < 1e-6f){
                nrm = (j==0) ? v3(0,1,0) : v3(0,-1,0);
            } else {
                nrm = v3_norm(pos);
            }

            m.verts[m.nvert++]=(Vertex){pos,nrm,u,v};
        }
    }

    /*
     * Build quad grid, split each quad into 2 CCW triangles.
     * r0 = top-left, r1 = top-right (i+1)
     * r2 = bot-left (j+1), r3 = bot-right
     *
     * Tri 0: r0,r2,r1  (CCW from outside)
     * Tri 1: r1,r2,r3  (CCW from outside)
     */
    for(int j=0;j<nv;j++){
        for(int i=0;i<nu;i++){
            int r0=j*(nu+1)+i,   r1=r0+1;
            int r2=r0+(nu+1),    r3=r2+1;
            m.tris[m.ntri++]=(Triangle){{r0,r2,r1}};
            m.tris[m.ntri++]=(Triangle){{r1,r2,r3}};
        }
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
    bool          cull_backface; /* c toggles — false shows inner surface  */
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
    s->mesh          = tessellate_sphere();
    s->shade_idx     = SH_PHONG;
    s->cam_dist      = CAM_DIST;
    s->cull_backface = true;

    s->uni.light_pos  = v3(3.f, 4.f, 3.f);
    s->uni.light_col  = v3(1.f, 1.f, 1.f);
    s->uni.ambient    = v3(0.07f,0.07f,0.07f);
    s->uni.shininess  = 80.f;   /* higher shininess = tighter highlight */
    s->uni.cam_pos    = v3(0.f, 0.f, s->cam_dist);
    s->uni.obj_color  = v3(0.25f, 0.55f, 0.95f);   /* cool blue */

    s->uni.view = m4_lookat(s->uni.cam_pos, v3(0,0,0), v3(0,1,0));
    float aspect=(float)(cols*CELL_W)/(float)(rows*CELL_H);
    s->uni.proj = m4_perspective(CAM_FOV,aspect,CAM_NEAR,CAM_FAR);

    scene_build_shader(s);
}

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
    Scene *s=&app->scene;
    switch(ch){
    case 'q': case 'Q': case 27: return false;
    case ' ': s->paused=!s->paused; break;
    case 's': case 'S': scene_next_shader(s); break;
    case 'c': case 'C': s->cull_backface=!s->cull_backface; break;
    case '=': case '+':
        s->cam_dist-=CAM_ZOOM_STEP;
        if(s->cam_dist<CAM_DIST_MIN) s->cam_dist=CAM_DIST_MIN;
        scene_set_zoom(s);
        break;
    case '-':
        s->cam_dist+=CAM_ZOOM_STEP;
        if(s->cam_dist>CAM_DIST_MAX) s->cam_dist=CAM_DIST_MAX;
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
