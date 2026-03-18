/*
 * primitives.c  —  17 IQ SDF primitives viewer
 *
 * One primitive fills the screen at a time.
 * Tab / t = next   T = previous.
 *
 * Primitives:
 *   1 Sphere        2 Box           3 Round Box     4 Box Frame
 *   5 Torus         6 Capped Torus  7 Link          8 Cone
 *   9 Plane        10 Hex Prism    11 Capsule       12 Cylinder
 *  13 Round Cone   14 Octahedron   15 Pyramid       16 Triangle
 *  17 Quad
 *
 * Rotation: two-axis tumble (slow X tilt + fast Y spin) so every face,
 * edge, and vertex is seen without any manual camera movement.
 *
 * Keys:
 *   Tab / t / T   cycle primitives
 *   space         pause / resume
 *   ]  [          light faster / slower
 *   =  -          size larger / smaller
 *   r  R          tumble faster / slower
 *   q / ESC       quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra primitives.c -o primitives -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  vec2 / vec3
 *   §5  sdf  — 17 IQ primitives
 *   §6  wrappers + prim table
 *   §7  raymarch — tumble, march, normal, shade
 *   §8  canvas
 *   §9  scene
 *  §10  screen
 *  §11  app
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
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    N_PRIMS         = 17,
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 24,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,
    HUD_COLS        = 56,
    FPS_UPDATE_MS   = 500,
};

#define CELL_ASPECT       2.0f

#define RM_MAX_STEPS      100
#define RM_HIT_EPS        0.002f
#define RM_MAX_DIST       20.0f
#define RM_NORM_EPS       0.001f

#define CAM_Z             4.5f
#define FOV_HALF_TAN      0.65f

#define PRIM_SIZE_DEFAULT 1.0f
#define PRIM_SIZE_STEP    1.15f
#define PRIM_SIZE_MIN     0.2f
#define PRIM_SIZE_MAX     3.0f

/*
 * Two-axis tumble speeds.
 * Y spins at ROT_Y_SPD.  X tilts at ROT_X_SPD = ROT_Y_SPD * 0.37.
 * The irrational ratio makes the orbit quasi-periodic — it never repeats
 * exactly, so every face/edge/vertex is visited over time.
 */
#define ROT_Y_SPD_DEFAULT 0.60f   /* fast axis — left/right spin         */
#define ROT_X_RATIO       0.37f   /* slow axis — up/down tilt            */
#define ROT_SPD_STEP      1.3f
#define ROT_SPD_MIN       0.0f
#define ROT_SPD_MAX       5.0f

/*
 * Fixed light position — upper-left of camera, slightly above.
 * Classic "three-point" side key light that reveals shape well.
 * The object rotates; the light never moves.
 */
#define LIGHT_X   -3.0f
#define LIGHT_Y    3.5f
#define LIGHT_Z    2.5f

#define KA    0.10f
#define KD    0.78f
#define KS    0.55f
#define SHIN  40.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC/(f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

enum { LUMI_N = 8 };
static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        static const int g[LUMI_N] = {235,238,241,244,247,250,253,255};
        for (int i = 0; i < LUMI_N; i++) init_pair(i+1, g[i], COLOR_BLACK);
    } else {
        for (int i = 0; i < LUMI_N; i++) init_pair(i+1, COLOR_WHITE, COLOR_BLACK);
    }
}
static attr_t lumi_attr(int l)
{
    if (l<0) l=0; if (l>LUMI_N-1) l=LUMI_N-1;
    attr_t a = COLOR_PAIR(l+1);
    if (COLORS < 256) { if (l<3) a|=A_DIM; else if (l>=6) a|=A_BOLD; }
    return a;
}

/* ===================================================================== */
/* §4  vec2 / vec3                                                        */
/* ===================================================================== */

typedef struct { float x, y; }    V2;
typedef struct { float x, y, z; } V3;

static inline V2  v2(float x,float y)        { return (V2){x,y}; }
static inline V3  v3(float x,float y,float z) { return (V3){x,y,z}; }

static inline V2  v2add(V2 a,V2 b)  { return v2(a.x+b.x,a.y+b.y); }
static inline V2  v2sub(V2 a,V2 b)  { return v2(a.x-b.x,a.y-b.y); }
static inline V2  v2mul(V2 a,float s){ return v2(a.x*s,a.y*s); }
static inline V2  v2abs(V2 a)        { return v2(fabsf(a.x),fabsf(a.y)); }
static inline V2  v2max0(V2 a)       { return v2(fmaxf(a.x,0),fmaxf(a.y,0)); }
static inline float v2dot(V2 a,V2 b) { return a.x*b.x+a.y*b.y; }
static inline float v2dot2(V2 a)     { return a.x*a.x+a.y*a.y; }
static inline float v2len(V2 a)      { return sqrtf(v2dot2(a)); }

static inline V3  v3add(V3 a,V3 b)  { return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline V3  v3sub(V3 a,V3 b)  { return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline V3  v3mul(V3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline V3  v3abs(V3 a)        { return v3(fabsf(a.x),fabsf(a.y),fabsf(a.z)); }
static inline V3  v3max0(V3 a)       { return v3(fmaxf(a.x,0),fmaxf(a.y,0),fmaxf(a.z,0)); }
static inline float v3dot(V3 a,V3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3len(V3 a)      { return sqrtf(v3dot(a,a)); }
static inline V3  v3norm(V3 a)       { float l=v3len(a); return l>1e-7f?v3mul(a,1.f/l):v3(0,0,1); }

static inline float clmpf(float v,float lo,float hi){ return fmaxf(lo,fminf(hi,v)); }

/* Rotate p around Y axis */
static inline V3 rot_y(V3 p, float a)
{
    float c=cosf(a), s=sinf(a);
    return v3(c*p.x+s*p.z, p.y, -s*p.x+c*p.z);
}
/* Rotate p around X axis */
static inline V3 rot_x(V3 p, float a)
{
    float c=cosf(a), s=sinf(a);
    return v3(p.x, c*p.y-s*p.z, s*p.y+c*p.z);
}
/*
 * tumble() — two-axis rotation applied to the query point.
 * ry = fast Y spin, rx = slow X tilt (= ry * ROT_X_RATIO).
 * Apply Y first then X — this gives a natural tumbling motion.
 */
static inline V3 tumble(V3 p, float ry, float rx)
{
    return rot_x(rot_y(p, ry), rx);
}

/* ===================================================================== */
/* §5  sdf — 17 IQ primitives (pure math, no global state)              */
/* ===================================================================== */

/* 1. Sphere */
static float sdf_sphere(V3 p, float r)
{
    return v3len(p) - r;
}

/* 2. Box */
static float sdf_box(V3 p, V3 b)
{
    V3 q = v3sub(v3abs(p), b);
    return v3len(v3max0(q)) + fminf(fmaxf(q.x,fmaxf(q.y,q.z)), 0.f);
}

/* 3. Round Box */
static float sdf_round_box(V3 p, V3 b, float r)
{
    V3 q = v3sub(v3abs(p), b);
    return v3len(v3max0(q)) + fminf(fmaxf(q.x,fmaxf(q.y,q.z)), 0.f) - r;
}

/* 4. Box Frame */
static float sdf_box_frame(V3 p, V3 b, float e)
{
    V3 pa = v3sub(v3abs(p), b);
    V3 q  = v3sub(v3abs(v3add(pa,v3(e,e,e))), v3(e,e,e));
    float d0=v3len(v3max0(v3(pa.x,q.y,q.z)))+fminf(fmaxf(pa.x,fmaxf(q.y,q.z)),0.f);
    float d1=v3len(v3max0(v3(q.x,pa.y,q.z)))+fminf(fmaxf(q.x,fmaxf(pa.y,q.z)),0.f);
    float d2=v3len(v3max0(v3(q.x,q.y,pa.z)))+fminf(fmaxf(q.x,fmaxf(q.y,pa.z)),0.f);
    return fminf(fminf(d0,d1),d2);
}

/* 5. Torus (in XZ plane, ring visible from +Y) */
static float sdf_torus(V3 p, float R, float r)
{
    return v2len(v2(v2len(v2(p.x,p.z))-R, p.y)) - r;
}

/* 6. Capped Torus */
static float sdf_capped_torus(V3 p, float ra, float rb, float sx, float cx)
{
    p.x = fabsf(p.x);
    float k = (cx*p.x > sx*p.y) ? v2dot(v2(p.x,p.y),v2(cx,sx))
                                  : sqrtf(p.x*p.x+p.y*p.y);
    return sqrtf(v3dot(p,p)+ra*ra-2.f*ra*k) - rb;
}

/*
 * 7. Link — IQ chain-link shape.
 * le=half-length of straight section, r1=link radius, r2=tube radius.
 */
static float sdf_link(V3 p, float le, float r1, float r2)
{
    V3 q = v3(p.x, fmaxf(fabsf(p.y)-le, 0.f), p.z);
    return v2len(v2(v2len(v2(q.x,q.y))-r1, q.z)) - r2;
}

/*
 * 8. Cone — exact IQ formula, verbatim port from GLSL.
 * c = vec2(sin, cos) of the half-angle.
 * h = height. Apex at origin, base circle at y = -h.
 * Wrapper shifts p so cone is centred at origin apex-up.
 */
static float sdf_cone(V3 p, float si, float co, float h)
{
    V2 q = v2(h*si/co, -h);
    V2 w = v2(v2len(v2(p.x,p.z)), p.y);
    V2 a = v2sub(w, v2mul(q, clmpf(v2dot(w,q)/v2dot(q,q), 0.f, 1.f)));
    V2 b = v2sub(w, v2(clmpf(w.x/q.x, 0.f, 1.f)*q.x, q.y));
    float k = (q.y < 0.f) ? -1.f : 1.f;
    float d = fminf(v2dot(a,a), v2dot(b,b));
    float s = fmaxf(k*(w.x*q.y - w.y*q.x), k*(w.y - q.y));
    return sqrtf(d) * (s >= 0.f ? 1.f : -1.f);
}

/*
 * 9. Plane — rendered as a finite disc so it has a visible shape.
 * Thin disc of radius R and thickness t in the XZ plane.
 */
static float sdf_plane_disc(V3 p, float R, float t)
{
    float r2d = v2len(v2(p.x,p.z)) - R;
    float ry  = fabsf(p.y) - t;
    return fminf(fmaxf(r2d,ry), 0.f) + v2len(v2max0(v2(r2d,ry)));
}

/* 10. Hex Prism */
static float sdf_hex_prism(V3 p, float rx, float ry)
{
    const float kx=-0.8660254f, ky=0.5f, kz=0.57735f;
    V3 q=v3abs(p);
    float d=kx*q.x+ky*q.y; if(d<0.f){q.x-=2.f*d*kx;q.y-=2.f*d*ky;}
    float cx=clmpf(q.x,-kz*rx,kz*rx);
    V2 dv=v2(v2len(v2sub(v2(q.x,q.y),v2(cx,rx)))*(q.y<rx?-1.f:1.f), q.z-ry);
    return fminf(fmaxf(dv.x,dv.y),0.f)+v2len(v2max0(dv));
}

/* 11. Capsule (segment a→b, radius r) */
static float sdf_capsule(V3 p, V3 a, V3 b, float r)
{
    V3 pa=v3sub(p,a), ba=v3sub(b,a);
    float h=clmpf(v3dot(pa,ba)/v3dot(ba,ba),0.f,1.f);
    return v3len(v3sub(pa,v3mul(ba,h))) - r;
}

/* 12. Cylinder (vertical, capped) */
static float sdf_cylinder(V3 p, float h, float r)
{
    V2 d=v2sub(v2abs(v2(v2len(v2(p.x,p.z)),p.y)), v2(r,h));
    return fminf(fmaxf(d.x,d.y),0.f) + v2len(v2max0(d));
}

/* 13. Round Cone */
static float sdf_round_cone(V3 p, float r1, float r2, float h)
{
    V2 q=v2(v2len(v2(p.x,p.z)),p.y);
    float b=(r1-r2)/h, a=sqrtf(1.f-b*b), k=v2dot(q,v2(-b,a));
    if(k<0.f) return v2len(q)-r1;
    if(k>a*h) return v2len(v2sub(q,v2(0.f,h)))-r2;
    return v2dot(q,v2(a,b))-r1;
}

/* 14. Octahedron */
static float sdf_octahedron(V3 p, float s)
{
    V3 q=v3abs(p);
    float m=q.x+q.y+q.z-s;
    V3 r;
    if      (3.f*q.x<m) r=q;
    else if (3.f*q.y<m) r=v3(q.y,q.z,q.x);
    else if (3.f*q.z<m) r=v3(q.z,q.x,q.y);
    else                return m*0.57735027f;
    float k=clmpf(0.5f*(r.z-r.y+s),0.f,s);
    return v3len(v3(r.x,r.y-s+k,r.z-k));
}

/* 15. Pyramid */
static float sdf_pyramid(V3 p, float h)
{
    float m2=h*h+0.25f;
    p.x=fabsf(p.x); p.z=fabsf(p.z);
    if(p.z>p.x){float t=p.x;p.x=p.z;p.z=t;}
    p.x-=0.5f; p.z-=0.5f;
    V3 q=v3(p.z,h*p.y-0.5f*p.x,h*p.x+0.5f*p.y);
    float ss=fmaxf(-q.x,0.f);
    float t=clmpf((q.y-0.5f*p.z)/(m2+0.25f),0.f,1.f);
    float a=m2*(q.x+ss)*(q.x+ss)+q.y*q.y;
    float b=m2*(q.x+0.5f*t)*(q.x+0.5f*t)+(q.y-m2*t)*(q.y-m2*t);
    float d2=(fminf(q.y,-q.x*m2-q.y*0.5f)>0.f)?0.f:fminf(a,b);
    return sqrtf((d2+q.z*q.z)/m2)*(fmaxf(q.z,-p.y)>=0.f?1.f:-1.f);
}

/*
 * 16. Triangle — equilateral triangle as a thin solid slab.
 * sz = circumradius, thick = half-thickness along Y.
 * IQ 2D triangle SDF extruded along Y.
 */
static float sdf_triangle(V3 p, float sz, float thick)
{
    V2 a=v2( 0.f,         sz);
    V2 b=v2(-sz*0.866f,  -sz*0.5f);
    V2 c=v2( sz*0.866f,  -sz*0.5f);
    V2 q=v2(p.x,p.z);
    V2 ab=v2sub(b,a),bc=v2sub(c,b),ca=v2sub(a,c);
    V2 qa=v2sub(q,a),qb=v2sub(q,b),qc=v2sub(q,c);
    float d2=fminf(fminf(
        v2dot2(v2sub(qa,v2mul(ab,clmpf(v2dot(qa,ab)/v2dot2(ab),0.f,1.f)))),
        v2dot2(v2sub(qb,v2mul(bc,clmpf(v2dot(qb,bc)/v2dot2(bc),0.f,1.f))))),
        v2dot2(v2sub(qc,v2mul(ca,clmpf(v2dot(qc,ca)/v2dot2(ca),0.f,1.f)))));
    int inside = (ab.x*qa.y-ab.y*qa.x<0.f &&
                  bc.x*qb.y-bc.y*qb.x<0.f &&
                  ca.x*qc.y-ca.y*qc.x<0.f);
    float d_xz = (inside ? -1.f : 1.f) * sqrtf(d2);
    float d_y  = fabsf(p.y) - thick;
    return fminf(fmaxf(d_xz,d_y),0.f) + v2len(v2max0(v2(d_xz,d_y)));
}

/*
 * 17. Quad — axis-aligned rectangular slab.
 * wx/wz = half-extents in XZ, thick = half-thickness along Y.
 */
static float sdf_quad(V3 p, float wx, float wz, float thick)
{
    V2 q2=v2sub(v2abs(v2(p.x,p.z)), v2(wx,wz));
    float d_xz=v2len(v2max0(q2))+fminf(fmaxf(q2.x,q2.y),0.f);
    float d_y =fabsf(p.y)-thick;
    return fminf(fmaxf(d_xz,d_y),0.f)+v2len(v2max0(v2(d_xz,d_y)));
}

/* ===================================================================== */
/* §6  wrappers + prim table                                             */
/* ===================================================================== */

/*
 * Each wrapper maps s (user size, default 1.0) to good SDF parameters.
 * Flat shapes (plane, triangle, quad) get a tilt applied so the tumble
 * rotation shows the face naturally; their initial orientation at rot=0
 * is tilted ~30° so neither pure face-on nor pure edge-on.
 */

typedef struct {
    const char *name;
    float (*sdf)(V3 p, float s);
} Prim;

static float w_sphere    (V3 p,float s){ return sdf_sphere(p,s); }
static float w_box       (V3 p,float s){ return sdf_box(p,v3(s*.70f,s*.70f,s*.70f)); }
static float w_round_box (V3 p,float s){ return sdf_round_box(p,v3(s*.55f,s*.55f,s*.55f),s*.15f); }
static float w_box_frame (V3 p,float s){ return sdf_box_frame(p,v3(s*.65f,s*.65f,s*.65f),s*.07f); }

/* Torus in XZ plane — rotate 90° around X so ring faces camera */
static float w_torus(V3 p,float s)
{
    return sdf_torus(rot_x(p,1.5708f), s*.62f, s*.21f);
}

/* Capped torus — same tilt, ~270° arc */
static float w_cap_torus(V3 p,float s)
{
    return sdf_capped_torus(rot_x(p,1.5708f), s*.62f, s*.17f, sinf(2.4f), cosf(2.4f));
}

/* Link — stands vertical, two rounded ends visible from front */
static float w_link(V3 p,float s)
{
    return sdf_link(p, s*.32f, s*.26f, s*.11f);
}

/* Cone — apex at top, base at bottom, centred at origin.
 * sdf_cone: apex at y=0, base at y=-h.
 * Shift p up by h/2 so the cone midpoint aligns with origin. */
static float w_cone(V3 p, float s)
{
    float h = s * 0.7f;
    V3 pp = v3(p.x, p.y + h*0.5f, p.z);
    return sdf_cone(pp, sinf(0.48f), cosf(0.48f), h);
}

/* Plane — thin horizontal disc; tilt forward so face is visible */
static float w_plane(V3 p,float s)
{
    V3 q=rot_x(p, 0.52f);   /* ~30° forward tilt */
    return sdf_plane_disc(q, s*.85f, s*.04f);
}

/* Hex prism — face a hexagonal face toward camera */
static float w_hex_prism(V3 p,float s)
{
    return sdf_hex_prism(rot_x(p,1.5708f), s*.55f, s*.35f);
}

/* Capsule — vertical, endpoints above/below centre */
static float w_capsule(V3 p,float s)
{
    return sdf_capsule(p, v3(0,-s*.52f,0), v3(0,s*.52f,0), s*.27f);
}

static float w_cylinder(V3 p,float s)
{
    return sdf_cylinder(p, s*.58f, s*.36f);
}

/* Round cone — wider base at bottom, narrower top */
static float w_round_cone(V3 p,float s)
{
    V3 pp=v3(p.x, p.y+s*.38f, p.z);
    return sdf_round_cone(pp, s*.34f, s*.11f, s*.76f);
}

static float w_octahedron(V3 p,float s){ return sdf_octahedron(p, s*.82f); }

/* Pyramid — apex up, base shifted so centred at origin */
static float w_pyramid(V3 p,float s)
{
    float h=s*1.0f;
    V3 pp=v3(p.x, p.y+h*.33f, p.z);
    return sdf_pyramid(pp, h);
}

/* Triangle slab — tilt ~30° so face AND edge visible during tumble */
static float w_triangle(V3 p,float s)
{
    V3 q=rot_x(p, 0.52f);
    return sdf_triangle(q, s*.80f, s*.055f);
}

/* Quad slab — same treatment */
static float w_quad(V3 p,float s)
{
    V3 q=rot_x(p, 0.52f);
    return sdf_quad(q, s*.72f, s*.52f, s*.05f);
}

static const Prim k_prims[N_PRIMS] = {
    { "Sphere",       w_sphere     },   /*  1 */
    { "Box",          w_box        },   /*  2 */
    { "Round Box",    w_round_box  },   /*  3 */
    { "Box Frame",    w_box_frame  },   /*  4 */
    { "Torus",        w_torus      },   /*  5 */
    { "Capped Torus", w_cap_torus  },   /*  6 */
    { "Link",         w_link       },   /*  7 */
    { "Cone",         w_cone       },   /*  8 */
    { "Plane",        w_plane      },   /*  9 */
    { "Hex Prism",    w_hex_prism  },   /* 10 */
    { "Capsule",      w_capsule    },   /* 11 */
    { "Cylinder",     w_cylinder   },   /* 12 */
    { "Round Cone",   w_round_cone },   /* 13 */
    { "Octahedron",   w_octahedron },   /* 14 */
    { "Pyramid",      w_pyramid    },   /* 15 */
    { "Triangle",     w_triangle   },   /* 16 */
    { "Quad",         w_quad       },   /* 17 */
};

/* ===================================================================== */
/* §7  raymarch                                                           */
/* ===================================================================== */

/*
 * rm_march() — sphere-march the current primitive.
 *
 * The primitive SDF lives at the origin in local space.
 * We apply tumble() to the query point (inverse transform):
 *   p_local = tumble(p_world, ry, rx)
 *
 * ry = current Y-spin angle
 * rx = ry * ROT_X_RATIO  (slow X tilt, irrational ratio = full coverage)
 */
static float rm_march(V3 ro, V3 rd, int prim, float s, float ry)
{
    float rx = ry * ROT_X_RATIO;
    float t  = 0.05f;
    for (int i = 0; i < RM_MAX_STEPS; i++) {
        V3    p = tumble(v3add(ro, v3mul(rd,t)), ry, rx);
        float d = k_prims[prim].sdf(p, s);
        if (d < RM_HIT_EPS) return t;
        if (t > RM_MAX_DIST) return -1.f;
        t += d;
    }
    return -1.f;
}

/* rm_normal() — tetrahedron normal estimation */
static V3 rm_normal(V3 pos, int prim, float s, float ry)
{
    float rx = ry * ROT_X_RATIO;
    const float e = RM_NORM_EPS;
    V3 n = v3(0,0,0);
    for (int i = 0; i < 4; i++) {
        float bx=(float)(((i+3)>>1)&1),by=(float)((i>>1)&1),bz=(float)(i&1);
        V3 ev = v3mul(v3(2.f*bx-1.f,2.f*by-1.f,2.f*bz-1.f), 0.5773f);
        V3 sp = tumble(v3add(pos,v3mul(ev,e)), ry, rx);
        n = v3add(n, v3mul(ev, k_prims[prim].sdf(sp,s)));
    }
    return v3norm(n);
}

/* rm_shade() — Phong */
static float rm_shade(V3 N, V3 hit, V3 cam, V3 light)
{
    V3    L   = v3norm(v3sub(light,hit));
    V3    V   = v3norm(v3sub(cam,hit));
    float ndl = fmaxf(0.f, v3dot(N,L));
    V3    R   = v3sub(v3mul(N,2.f*ndl),L);
    float sp  = powf(fmaxf(0.f,v3dot(R,V)), SHIN);
    float I   = KA + KD*ndl + KS*sp;
    return I > 1.f ? 1.f : I;
}

/*
 * rm_cast_pixel() — one ray, returns Phong intensity or -1 on miss.
 * Aspect correction: phys_aspect on the v component of the ray.
 */
static float rm_cast_pixel(int px, int py, int cw, int ch,
                             int prim, float s, float ry, V3 light)
{
    float u =  ((float)px+0.5f)/(float)cw*2.f-1.f;
    float v = -((float)py+0.5f)/(float)ch*2.f+1.f;
    float pa = ((float)ch * CELL_ASPECT) / (float)cw;

    V3 ro = v3(0.f,0.f,CAM_Z);
    V3 rd = v3norm(v3(u*FOV_HALF_TAN, v*FOV_HALF_TAN*pa, -1.f));

    float t = rm_march(ro, rd, prim, s, ry);
    if (t < 0.f) return -1.f;

    V3 hit = v3add(ro, v3mul(rd,t));
    V3 N   = rm_normal(hit, prim, s, ry);
    return rm_shade(N, hit, ro, light);
}

/* ===================================================================== */
/* §8  canvas                                                             */
/* ===================================================================== */

#define CANVAS_MISS  -1

/*
 * Canvas — two-buffer design.
 *
 * intensity[] — raw Phong float [0,1] per pixel, or INTENSITY_MISS.
 *               Written by canvas_render. Physics lives here.
 *               Never touched by the display layer.
 *
 * pixels[]    — final ramp index per pixel, or CANVAS_MISS.
 *               Written by shade_to_terminal(). Display lives here.
 *               canvas_draw reads only this.
 *
 * Keeping them separate means the physics and the terminal translation
 * are completely decoupled. Change one without touching the other.
 */
#define INTENSITY_MISS  -1.0f

typedef struct {
    int    w, h;
    float *intensity;   /* [h*w] raw Phong or INTENSITY_MISS            */
    int   *pixels;      /* [h*w] ramp index or CANVAS_MISS              */
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w = cols; c->h = rows;
    c->intensity = calloc((size_t)(cols*rows), sizeof(float));
    c->pixels    = calloc((size_t)(cols*rows), sizeof(int));
}
static void canvas_free(Canvas *c)
{
    free(c->intensity); free(c->pixels);
    c->intensity = NULL; c->pixels = NULL; c->w = c->h = 0;
}

/*
 * Paul Bourke's full ASCII art ramp — 92 characters ordered by ink density.
 * Measured from actual font rendering, darkest (space) to brightest (@).
 * Source: paulbourke.net/dataformats/asciiart/
 *
 * With 92 levels + Floyd-Steinberg dithering the gradient resolution is
 * effectively continuous — the eye sees smooth shading with no visible bands.
 */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_N (int)(sizeof k_ramp - 1)   /* 92 */

/* ===================================================================== */
/* §8b  shade_to_terminal — the complete physics→display translation     */
/* ===================================================================== */

/*
 * shade_to_terminal() converts raw Phong intensity into terminal ramp
 * indices with three successive corrections, each fixing one class of
 * visual artifact. Physics (intensity[]) is read-only. Only pixels[]
 * is written.
 *
 * ── Step 1: Gamma correction ─────────────────────────────────────────
 *
 *   The raw Phong value is linear light. Terminal displays apply gamma
 *   ~2.2, meaning signal value 0.5 appears as 0.5^2.2 ≈ 0.22 brightness.
 *   Fix: pow(I, 1/2.2) before indexing.
 *
 * ── Step 2: LUT mapping ──────────────────────────────────────────────
 *
 *   With 92 characters the breaks are essentially linear — each step
 *   covers 1/91 of the gamma-corrected range. The LUT preserves the
 *   architecture so it can be tuned without touching the dithering.
 *
 * ── Step 3: Floyd-Steinberg dithering ────────────────────────────────
 *
 *   92 levels still quantise a continuous float. Dithering distributes
 *   the quantisation error to neighbours so smooth gradients remain
 *   smooth rather than showing ~1/92 = 1% banding.
 */

/*
 * LUT breaks — 92 evenly-spaced thresholds in gamma-corrected space.
 * Even spacing is appropriate here because the Bourke ramp was already
 * measured to have approximately uniform ink-density steps.
 */
static float k_lut_breaks[RAMP_N];   /* filled once at startup */

static void lut_init(void)
{
    for (int i = 0; i < RAMP_N; i++)
        k_lut_breaks[i] = (float)i / (float)(RAMP_N - 1);
}

/* Map a gamma-corrected float [0,1] to ramp index via LUT */
static inline int lut_index(float v)
{
    int idx = RAMP_N - 1;
    for (int i = 0; i < RAMP_N - 1; i++) {
        if (v < k_lut_breaks[i+1]) { idx = i; break; }
    }
    return idx;
}

/* Map ramp index back to float midpoint (for error calculation) */
static inline float lut_value(int idx)
{
    if (idx <= 0)         return 0.f;
    if (idx >= RAMP_N-1)  return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx+1]) * 0.5f;
}

static void shade_to_terminal(Canvas *c)
{
    int    w = c->w, h = c->h;
    int    n = w * h;
    float *buf = malloc((size_t)n * sizeof(float));
    if (!buf) {
        /* fallback: direct linear mapping */
        for (int i = 0; i < n; i++) {
            float v = c->intensity[i];
            if (v < 0.f) { c->pixels[i] = CANVAS_MISS; continue; }
            int idx = (int)(v*(float)(RAMP_N-1)+0.5f);
            c->pixels[i] = idx < RAMP_N ? idx : RAMP_N-1;
        }
        return;
    }

    /* Step 1+2: gamma correction + LUT remap into working buffer */
    for (int i = 0; i < n; i++) {
        float v = c->intensity[i];
        if (v < 0.f) { buf[i] = INTENSITY_MISS; continue; }
        v = fmaxf(0.f, fminf(1.f, v));
        v = powf(v, 1.f/2.2f);          /* gamma linearise              */
        buf[i] = v;
    }

    /* Step 3: Floyd-Steinberg dithering */
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int   i = y*w + x;
            float v = buf[i];

            if (v < INTENSITY_MISS + 0.5f) {
                c->pixels[i] = CANVAS_MISS;
                continue;
            }

            /* Quantise via LUT */
            int   idx = lut_index(v);
            c->pixels[i] = idx;

            /* Error between true value and quantised midpoint */
            float qv  = lut_value(idx);
            float err = v - qv;

            /* Distribute error to four neighbours */
            if (x+1 < w && buf[i+1] > INTENSITY_MISS + 0.5f)
                buf[i+1]   += err * (7.f/16.f);
            if (y+1 < h) {
                if (x-1 >= 0 && buf[i+w-1] > INTENSITY_MISS + 0.5f)
                    buf[i+w-1] += err * (3.f/16.f);
                if (buf[i+w] > INTENSITY_MISS + 0.5f)
                    buf[i+w]   += err * (5.f/16.f);
                if (x+1 < w && buf[i+w+1] > INTENSITY_MISS + 0.5f)
                    buf[i+w+1] += err * (1.f/16.f);
            }
        }
    }

    free(buf);
}

static void canvas_render(Canvas *c, int prim, float s, float ry, V3 light)
{
    /* Phase 1: physics — store raw Phong intensity, no display knowledge */
    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            float I = rm_cast_pixel(px, py, c->w, c->h, prim, s, ry, light);
            c->intensity[py*c->w+px] = I;   /* -1.0 on miss, [0,1] on hit */
        }
    }
    /* Phase 2: terminal translation — gamma + LUT + dithering */
    shade_to_terminal(c);
}

static void canvas_draw(const Canvas *c, int tcols, int trows)
{
    int ox=(tcols-c->w)/2, oy=(trows-c->h)/2;
    for (int y=0; y<c->h; y++) {
        for (int x=0; x<c->w; x++) {
            int idx=c->pixels[y*c->w+x];
            if (idx==CANVAS_MISS) continue;
            int tx=ox+x, ty=oy+y;
            if (tx<0||tx>=tcols||ty<0||ty>=trows) continue;
            char   ch   = k_ramp[idx];
            attr_t attr = lumi_attr((idx*LUMI_N)/RAMP_N);
            attron(attr);
            mvaddch(ty,tx,(chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §9  scene                                                              */
/* ===================================================================== */

typedef struct {
    Canvas canvas;
    int    prim;
    float  size;
    float  ry;          /* Y-spin angle (fast axis)                     */
    float  rot_spd;     /* Y speed; X speed = rot_spd * ROT_X_RATIO     */
    float  time;
    bool   paused;
} Scene;

/* Fixed light — never moves. Object rotates around it. */
static V3 scene_light(void)
{
    return v3(LIGHT_X, LIGHT_Y, LIGHT_Z);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s,0,sizeof *s);
    canvas_alloc(&s->canvas,cols,rows);
    s->prim    = 0;
    s->size    = PRIM_SIZE_DEFAULT;
    s->ry      = 0.f;
    s->rot_spd = ROT_Y_SPD_DEFAULT;
}
static void scene_free(Scene *s)  { canvas_free(&s->canvas); }
static void scene_resize(Scene *s, int cols, int rows)
{
    canvas_free(&s->canvas); canvas_alloc(&s->canvas,cols,rows);
}
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
    s->ry   += s->rot_spd * dt;
}
static void scene_render(Scene *s)
{
    canvas_render(&s->canvas, s->prim, s->size, s->ry, scene_light());
}
static void scene_draw(const Scene *s, int cols, int rows)
{
    canvas_draw(&s->canvas, cols, rows);
}

/* ===================================================================== */
/* §10 screen                                                             */
/* ===================================================================== */

typedef struct { int cols,rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr,TRUE); keypad(stdscr,TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr,s->rows,s->cols);
}
static void screen_free(Screen *s){ (void)s; endwin(); }
static void screen_resize(Screen *s){ endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols); }

static void screen_draw(Screen *s, const Scene *sc, double fps, int sfps)
{
    erase();
    scene_draw(sc,s->cols,s->rows);

    char buf[HUD_COLS+1];
    snprintf(buf,sizeof buf, "%4.1f fps  [%2d/%2d] %-14s  sim:%d",
             fps, sc->prim+1, N_PRIMS, k_prims[sc->prim].name, sfps);
    int hx=s->cols-HUD_COLS; if(hx<0)hx=0;
    attron(lumi_attr(6)|A_BOLD); mvprintw(0,hx,"%s",buf); attroff(lumi_attr(6)|A_BOLD);

    snprintf(buf,sizeof buf, "Tab=next  =/- size  r/R spin speed  space=pause");
    attron(lumi_attr(3)); mvprintw(1,hx,"%s",buf); attroff(lumi_attr(3));
}
static void screen_present(void){ wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §11 app                                                                */
/* ===================================================================== */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int sig){ (void)sig; g_app.running=0; }
static void on_resize(int sig){ (void)sig; g_app.need_resize=1; }
static void cleanup  (void)  { endwin(); }

static void app_do_resize(App *a)
{
    screen_resize(&a->screen);
    scene_resize(&a->scene,a->screen.cols,a->screen.rows);
    a->need_resize=0;
}

static bool app_handle_key(App *a, int ch)
{
    Scene *s=&a->scene;
    switch(ch){
    case 'q': case 'Q': case 27: return false;

    case '\t': case 't':
        s->prim=(s->prim+1)%N_PRIMS; s->ry=0.f; break;
    case 'T':
        s->prim=(s->prim+N_PRIMS-1)%N_PRIMS; s->ry=0.f; break;

    case ' ': s->paused=!s->paused; break;

    case '=': case '+':
        s->size*=PRIM_SIZE_STEP; if(s->size>PRIM_SIZE_MAX)s->size=PRIM_SIZE_MAX; break;
    case '-':
        s->size/=PRIM_SIZE_STEP; if(s->size<PRIM_SIZE_MIN)s->size=PRIM_SIZE_MIN; break;

    case 'r':
        s->rot_spd+=0.15f; if(s->rot_spd>ROT_SPD_MAX)s->rot_spd=ROT_SPD_MAX; break;
    case 'R':
        s->rot_spd-=0.15f; if(s->rot_spd<ROT_SPD_MIN)s->rot_spd=ROT_SPD_MIN; break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,on_exit); signal(SIGTERM,on_exit); signal(SIGWINCH,on_resize);

    App *app=&g_app; app->running=1; app->sim_fps=SIM_FPS_DEFAULT;
    lut_init();
    screen_init(&app->screen);
    scene_init(&app->scene,app->screen.cols,app->screen.rows);

    int64_t ft=clock_ns(), sa=0, fa=0; int fc=0; double fpsd=0.;

    while(app->running){
        if(app->need_resize){ app_do_resize(app); ft=clock_ns(); sa=0; }

        int64_t now=clock_ns(), dt=now-ft; ft=now;
        if(dt>100*NS_PER_MS) dt=100*NS_PER_MS;

        int64_t tick=TICK_NS(app->sim_fps);
        float   dts =(float)tick/(float)NS_PER_SEC;
        sa+=dt;
        while(sa>=tick){ scene_tick(&app->scene,dts); sa-=tick; }
        float alpha=(float)sa/(float)tick;
        (void)alpha;

        scene_render(&app->scene);

        fc++; fa+=dt;
        if(fa>=FPS_UPDATE_MS*NS_PER_MS){
            fpsd=(double)fc/((double)fa/(double)NS_PER_SEC);
            fc=0; fa=0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t el=clock_ns()-ft+dt;
        clock_sleep_ns(NS_PER_SEC/60-el);

        screen_draw(&app->screen,&app->scene,fpsd,app->sim_fps);
        screen_present();

        int ch=getch();
        if(ch!=ERR&&!app_handle_key(app,ch)) app->running=0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
