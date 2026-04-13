/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rigid_body.c — 2D Rigid Body Physics
 *
 * N circles and N rects on one floor.  Full pairwise collision for every
 * combination (circle-circle, circle-rect, rect-rect) plus floor/wall.
 *
 * Collision detection:
 *   circle-circle  — separating distance
 *   circle-rect    — closest point on OBB, world-space normal
 *   rect-rect      — Separating Axis Theorem (4 axes)
 *   floor          — fixed rigid body (inv_mass=0), same collision path as rect-rect
 *   side walls     — impulse with angular response + Coulomb friction
 *
 * Collision response (impulse method):
 *   j = (1+e)·vn / (1/mA + 1/mB + (rA×n)²/IA + (rB×n)²/IB)
 *   Wall response uses the same formula with fixed body (inv_mass = 0).
 *   Coulomb friction applied at every wall contact.
 *
 * Keys:
 *   c   add rect      s   add circle   x   remove last body
 *   q/ESC quit        p   pause        r   reset (1 rect + 1 circle)
 *   g   gravity       e/E restitution± t/T theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/rigid_body.c -o rigid_body -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 vec2  §5 body
 *           §6 framebuf  §7 physics  §8 scene  §9 draw  §10 screen  §11 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define ROWS_MAX        128
#define COLS_MAX        512
#define N_BODIES_MAX     21   /* 1 fixed floor + 20 dynamic bodies */
#define N_THEMES          5

#define GRAVITY_DEF     0.035f  /* units/step²  —  must be < SLEEP_VEL */
#define REST_DEF        0.35f   /* coefficient of restitution */
#define REST_STEP       0.05f
#define DAMPING_LIN     0.988f  /* per-step linear damping */
#define DAMPING_ANG     0.975f  /* per-step angular damping */
#define SLEEP_VEL       0.055f  /* sleep when |v| < this (> GRAVITY_DEF) */
#define SLEEP_OMEGA     0.015f
#define SLEEP_FRAMES      10    /* consecutive low-vel frames before sleeping */
#define WAKE_IMPULSE    0.05f   /* min impulse magnitude to wake a sleeping body */
#define FLOOR_FRICTION  0.40f   /* Coulomb μ at walls */
#define DENSITY         0.008f  /* mass = density × area */
#define V_REST          0.55f   /* vn below which restitution → 0 */
#define MAX_VEL        20.0f    /* velocity cap — prevents tunneling under dense stacking */
#define MAX_OMEGA       4.0f    /* angular velocity cap */

/* Default body sizes */
#define CIRC_R_DEF      5.0f
#define RECT_HW_DEF     7.0f
#define RECT_HH_DEF     5.0f

#define STEPS_DEF         3
#define SIM_FPS          20
#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC/(f))

static const float PI = 3.14159265f;

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

#define CP_FLOOR   9
#define CP_HUD    10

typedef struct { short body[8]; const char *name; } Theme;

static const Theme k_themes[N_THEMES] = {
    { { 196, 214, 226,  46,  51,  39, 129, 201 }, "Vivid"  },
    { { 160, 172, 178,  34,  43,  27,  91, 163 }, "Muted"  },
    { { 231, 229, 225, 220, 214, 208, 202, 196 }, "Fire"   },
    { {  46,  82, 118, 154, 190, 226, 220, 214 }, "Matrix" },
    { {  51,  45,  39,  33,  27,  21,  17,  57 }, "Ocean"  },
};

static bool g_has_256;
static int  g_theme = 0;

static void theme_apply(int ti)
{
    const Theme *t = &k_themes[ti];
    for (int i = 0; i < 8; i++) {
        short fg = g_has_256 ? t->body[i] : (short)(COLOR_RED + i % 6);
        init_pair(1+i, fg, COLOR_BLACK);
    }
    init_pair(CP_FLOOR, g_has_256 ? 240 : COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_HUD,   g_has_256 ? 255 : COLOR_WHITE,
                        g_has_256 ? 236 : COLOR_BLACK);
}

/* ===================================================================== */
/* §4  Vec2                                                               */
/* ===================================================================== */

typedef struct { float x, y; } Vec2;
static inline Vec2  v2(float x,float y)     { return (Vec2){x,y}; }
static inline Vec2  v2add(Vec2 a,Vec2 b)    { return v2(a.x+b.x,a.y+b.y); }
static inline Vec2  v2sub(Vec2 a,Vec2 b)    { return v2(a.x-b.x,a.y-b.y); }
static inline Vec2  v2scale(Vec2 a,float s) { return v2(a.x*s,a.y*s); }
static inline float v2dot(Vec2 a,Vec2 b)    { return a.x*b.x+a.y*b.y; }
static inline float v2cross(Vec2 a,Vec2 b)  { return a.x*b.y-a.y*b.x; }
static inline float v2len(Vec2 a)           { return sqrtf(a.x*a.x+a.y*a.y); }
static inline Vec2  v2norm(Vec2 a)
{ float l=v2len(a); return l>1e-6f?v2scale(a,1.f/l):v2(0,0); }
static inline Vec2  v2rot(Vec2 v,float ca,float sa)
{ return v2(v.x*ca-v.y*sa, v.x*sa+v.y*ca); }
static inline Vec2  v2perp(Vec2 a) { return v2(-a.y,a.x); }

/* ===================================================================== */
/* §5  Body                                                               */
/* ===================================================================== */

typedef enum { SHAPE_CIRCLE, SHAPE_RECT } Shape;

typedef struct {
    Vec2  pos, vel;
    float angle, omega;
    float mass, inv_mass;
    float inertia, inv_inertia;
    Shape shape;
    float radius;
    float hw, hh;
    int   color_pair;
    bool  fixed;
    int   sleep_ticks;  /* frames spent below sleep threshold */
    bool  sleeping;     /* true = skip gravity + integration */
} Body;

static void body_init_mass(Body *b)
{
    if (b->fixed) { b->inv_mass=0.f; b->inv_inertia=0.f; return; }
    float area = (b->shape==SHAPE_CIRCLE) ? PI*b->radius*b->radius
                                          : 4.f*b->hw*b->hh;
    b->mass      = area * DENSITY;
    b->inv_mass  = 1.f / b->mass;
    b->inertia   = (b->shape==SHAPE_CIRCLE)
                   ? 0.5f*b->mass*b->radius*b->radius
                   : b->mass*(b->hw*b->hw+b->hh*b->hh)/3.f;
    b->inv_inertia = 1.f / b->inertia;
}

static void body_corners(const Body *b, Vec2 out[4])
{
    float ca=cosf(b->angle), sa=sinf(b->angle);
    Vec2 ax=v2rot(v2(b->hw,0.f),ca,sa);
    Vec2 ay=v2rot(v2(0.f,b->hh),ca,sa);
    out[0]=v2add(v2sub(b->pos,ax),ay);
    out[1]=v2add(v2add(b->pos,ax),ay);
    out[2]=v2sub(v2add(b->pos,ax),ay);
    out[3]=v2sub(v2sub(b->pos,ax),ay);
}

/* ===================================================================== */
/* §6  framebuffer                                                        */
/* ===================================================================== */

static char g_fb[ROWS_MAX][COLS_MAX];
static int  g_cp[ROWS_MAX][COLS_MAX];
static int  g_rows, g_cols;

static inline int pcol(float x) { return (int)(x+0.5f); }
static inline int prow(float y) { return (int)(y*0.5f+0.5f); }
#define WORLD_H() ((g_rows-4)*2)   /* 2-row gap between floor and HUD */
#define WORLD_W() (g_cols)

static void fb_clear(void)
{ memset(g_fb,0,sizeof g_fb); memset(g_cp,0,sizeof g_cp); }

static void fb_set(int row,int col,char ch,int cp)
{
    if(row<0||row>=g_rows-2||col<0||col>=g_cols) return;
    g_fb[row][col]=ch; g_cp[row][col]=cp;
}

static void fb_line(int x0,int y0,int x1,int y1,char ch,int cp)
{
    int dx=abs(x1-x0),sx=x0<x1?1:-1;
    int dy=-abs(y1-y0),sy=y0<y1?1:-1;
    int err=dx+dy;
    for(;;){
        fb_set(y0,x0,ch,cp);
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>=dy){if(x0==x1)break;err+=dy;x0+=sx;}
        if(e2<=dx){if(y0==y1)break;err+=dx;y0+=sy;}
    }
}

static void fb_circle(float px,float py,float r,char ch,int cp)
{
    int steps=(int)(2.f*PI*r)+8; if(steps<8)steps=8;
    for(int i=0;i<steps;i++){
        float a=i*2.f*PI/steps;
        fb_set(prow(py+r*sinf(a)), pcol(px+r*cosf(a)), ch, cp);
    }
}

static void fb_body(const Body *b)
{
    int cp=b->color_pair;
    if(b->shape==SHAPE_CIRCLE){
        fb_circle(b->pos.x,b->pos.y,b->radius,'O',cp);
        float ca=cosf(b->angle),sa=sinf(b->angle),rr=b->radius*0.7f;
        fb_line(pcol(b->pos.x),prow(b->pos.y),
                pcol(b->pos.x+ca*rr),prow(b->pos.y+sa*rr),'-',cp);
        fb_set(prow(b->pos.y),pcol(b->pos.x),'+',cp);
    } else {
        Vec2 c[4]; body_corners(b,c);
        int s[4][2];
        for(int i=0;i<4;i++){s[i][0]=pcol(c[i].x);s[i][1]=prow(c[i].y);}
        fb_line(s[0][0],s[0][1],s[1][0],s[1][1],'#',cp);
        fb_line(s[1][0],s[1][1],s[2][0],s[2][1],'#',cp);
        fb_line(s[2][0],s[2][1],s[3][0],s[3][1],'#',cp);
        fb_line(s[3][0],s[3][1],s[0][0],s[0][1],'#',cp);
        fb_set(prow(b->pos.y),pcol(b->pos.x),'+',cp);
    }
}

static void fb_flush(void)
{
    for(int r=0;r<g_rows-2;r++)
        for(int c=0;c<g_cols;c++){
            if(!g_fb[r][c]) continue;
            attron(COLOR_PAIR(g_cp[r][c])|A_BOLD);
            mvaddch(r,c,(chtype)g_fb[r][c]);
            attroff(COLOR_PAIR(g_cp[r][c])|A_BOLD);
        }
}

/* ===================================================================== */
/* §7  physics                                                            */
/* ===================================================================== */

typedef struct {
    bool  valid;
    Vec2  normal;   /* unit, from A toward B */
    float depth;
    Vec2  point;
} Contact;

/* ── circle-circle ──────────────────────────────────────────────────── */
static Contact collide_cc(const Body *a,const Body *b)
{
    Vec2  d=v2sub(b->pos,a->pos);
    float dist=v2len(d), sum=a->radius+b->radius;
    if(dist>=sum-1e-4f) return (Contact){false};
    Vec2 n=(dist>1e-6f)?v2scale(d,1.f/dist):v2(1,0);
    return (Contact){true,n,sum-dist, v2add(a->pos,v2scale(n,a->radius))};
}

/* ── circle-rect ────────────────────────────────────────────────────── */
static Contact collide_cr(const Body *circle,const Body *rect)
{
    float ca=cosf(rect->angle),sa=sinf(rect->angle);
    Vec2  d=v2sub(circle->pos,rect->pos);
    Vec2  local=v2(d.x*ca+d.y*sa, -d.x*sa+d.y*ca);
    bool  inside=(fabsf(local.x)<rect->hw && fabsf(local.y)<rect->hh);
    Vec2  clamped=v2(fmaxf(-rect->hw,fminf(rect->hw,local.x)),
                     fmaxf(-rect->hh,fminf(rect->hh,local.y)));
    Vec2  diff=v2sub(local,clamped);
    float dist=v2len(diff);
    if(!inside && dist>=circle->radius-1e-4f) return (Contact){false};
    Vec2 local_n; float depth;
    if(inside){
        float dx=rect->hw-fabsf(local.x), dy=rect->hh-fabsf(local.y);
        if(dx<dy){ local_n=v2(local.x>0.f?1.f:-1.f,0.f); depth=dx+circle->radius; }
        else     { local_n=v2(0.f,local.y>0.f?1.f:-1.f); depth=dy+circle->radius; }
    } else {
        local_n=(dist>1e-6f)?v2scale(diff,1.f/dist):v2(1,0);
        depth=circle->radius-dist;
    }
    Vec2 n=v2(local_n.x*ca-local_n.y*sa, local_n.x*sa+local_n.y*ca);
    Vec2 cp=v2add(rect->pos,
        v2(clamped.x*ca-clamped.y*sa, clamped.x*sa+clamped.y*ca));
    return (Contact){true,n,depth,cp};
}

/* ── rect-rect (SAT) ────────────────────────────────────────────────── */
static bool sat_overlap(Vec2 axis,const Vec2 *cA,const Vec2 *cB,float *d)
{
    float minA=1e9f,maxA=-1e9f,minB=1e9f,maxB=-1e9f;
    for(int i=0;i<4;i++){
        float p=v2dot(cA[i],axis); minA=fminf(minA,p); maxA=fmaxf(maxA,p);
        p=v2dot(cB[i],axis);       minB=fminf(minB,p); maxB=fmaxf(maxB,p);
    }
    *d=fminf(maxA,maxB)-fmaxf(minA,minB);
    return *d>0.f;
}

static Contact collide_rr(const Body *a,const Body *b)
{
    Vec2 cA[4],cB[4]; body_corners(a,cA); body_corners(b,cB);
    float caA=cosf(a->angle),saA=sinf(a->angle);
    float caB=cosf(b->angle),saB=sinf(b->angle);
    Vec2 axes[4]={v2norm(v2(caA,saA)),v2norm(v2(-saA,caA)),
                  v2norm(v2(caB,saB)),v2norm(v2(-saB,caB))};
    float mind=1e9f; Vec2 mina=v2(1,0);
    for(int i=0;i<4;i++){
        float d; if(!sat_overlap(axes[i],cA,cB,&d)) return (Contact){false};
        if(d<mind){mind=d;mina=axes[i];}
    }
    if(v2dot(v2sub(b->pos,a->pos),mina)<0.f) mina=v2scale(mina,-1.f);
    return (Contact){true,mina,mind,v2scale(v2add(a->pos,b->pos),0.5f)};
}

/* ── impulse resolution (body-body) ─────────────────────────────────── */
static void resolve_contact(Body *a,Body *b,Contact c,float e)
{
    Vec2  rA=v2sub(c.point,a->pos), rB=v2sub(c.point,b->pos);
    Vec2  vA=v2add(a->vel,v2scale(v2perp(rA),a->omega));
    Vec2  vB=v2add(b->vel,v2scale(v2perp(rB),b->omega));
    float vn=v2dot(v2sub(vA,vB),c.normal);
    if(vn<0.f) return;   /* separating */
    float rAxn=v2cross(rA,c.normal), rBxn=v2cross(rB,c.normal);
    float denom=a->inv_mass+b->inv_mass
               +rAxn*rAxn*a->inv_inertia+rBxn*rBxn*b->inv_inertia;
    if(denom<1e-8f) return;
    float eff_e=(vn>V_REST)?e:0.f;
    float j=(1.f+eff_e)*vn/denom;
    a->vel=v2sub(a->vel,v2scale(c.normal,j*a->inv_mass));
    b->vel=v2add(b->vel,v2scale(c.normal,j*b->inv_mass));
    a->omega-=rAxn*j*a->inv_inertia;
    b->omega+=rBxn*j*b->inv_inertia;
    /* Wake only for real impacts — ignore gravity micro-contacts */
    if(j>WAKE_IMPULSE){
        a->sleeping=false; a->sleep_ticks=0;
        b->sleeping=false; b->sleep_ticks=0;
    }
    /* Baumgarte positional correction */
    float corr=fmaxf(c.depth-0.1f,0.f)*0.5f/(a->inv_mass+b->inv_mass);
    a->pos=v2sub(a->pos,v2scale(c.normal,corr*a->inv_mass));
    b->pos=v2add(b->pos,v2scale(c.normal,corr*b->inv_mass));
}

/* ── wall impulse (body vs infinite-mass surface) ────────────────────
 *
 * n   points FROM body TOWARD wall (penetration direction).
 * cp  is the contact point on the wall surface.
 * Positional correction already applied by the caller.
 *
 * Applies normal impulse (with V_REST suppression) then Coulomb friction.
 */
static void resolve_with_wall(Body *b,Vec2 cp,Vec2 n,float e)
{
    Vec2  r  =v2sub(cp,b->pos);
    Vec2  vel=v2add(b->vel,v2scale(v2perp(r),b->omega));
    float vn =v2dot(vel,n);
    if(vn<=0.f) return;

    float rn   =v2cross(r,n);
    float denom=b->inv_mass+rn*rn*b->inv_inertia;
    if(denom<1e-8f) return;

    float eff_e=(vn>V_REST)?e:0.f;
    float j=(1.f+eff_e)*vn/denom;
    b->vel  =v2sub(b->vel, v2scale(n,j*b->inv_mass));
    b->omega-=rn*j*b->inv_inertia;
    if(j>WAKE_IMPULSE){ b->sleeping=false; b->sleep_ticks=0; }

    /* Coulomb friction along wall surface */
    Vec2  tan=v2perp(n);
    float vt =v2dot(vel,tan);
    float rnt=v2cross(r,tan);
    float dt =b->inv_mass+rnt*rnt*b->inv_inertia;
    if(dt>1e-8f){
        float jt=-vt/dt;
        float mx=FLOOR_FRICTION*j;
        if(jt> mx) jt= mx;
        if(jt<-mx) jt=-mx;
        b->vel  =v2add(b->vel, v2scale(tan,jt*b->inv_mass));
        b->omega+=rnt*jt*b->inv_inertia;
    }
}

/*
 * Find corners of a rect penetrating one wall.
 * axis: 0 = y-axis (floor/ceiling), 1 = x-axis (side walls)
 * positive: true = body exceeds limit from below/right, false = from above/left
 * Returns penetration depth and averaged contact point.
 */
static void rect_wall_contact(const Vec2 corn[4],
                               int axis,bool positive,float limit,
                               float *pen,Vec2 *cp)
{
    float best=-1.f;
    float sum_perp=0.f; int cnt=0;
    for(int i=0;i<4;i++){
        float v=(axis==0)?corn[i].y:corn[i].x;
        float p=positive?v-limit:limit-v;
        if(p>0.f){
            sum_perp+=(axis==0)?corn[i].x:corn[i].y;
            cnt++;
            if(p>best) best=p;
        }
    }
    if(cnt>0){ *pen=best; float avg=sum_perp/cnt;
        *cp=(axis==0)?v2(avg,limit):v2(limit,avg); }
    else *pen=0.f;
}

/* ── AABB floor / wall collision ─────────────────────────────────────── */
static void wall_collide(Body *b,float e,float ww)
{
    if(b->fixed) return;

    /* Floor is now a fixed rigid body — only handle ceiling and side walls here */
    if(b->shape==SHAPE_CIRCLE){
        float r=b->radius;
        if(b->pos.y-r<0.f){ b->pos.y=r;
            resolve_with_wall(b,v2(b->pos.x,0.f),v2(0.f,-1.f),e); }
        if(b->pos.x-r<0.f){ b->pos.x=r;
            resolve_with_wall(b,v2(0.f,b->pos.y),v2(-1.f,0.f),e); }
        if(b->pos.x+r>ww){ b->pos.x=ww-r;
            resolve_with_wall(b,v2(ww,b->pos.y),v2(1.f,0.f),e); }
    } else {
        Vec2 corn[4]; float pen; Vec2 cp;

        body_corners(b,corn);
        rect_wall_contact(corn,0,false,0.f,&pen,&cp);
        if(pen>0.f){ b->pos.y+=pen;
            resolve_with_wall(b,cp,v2(0.f,-1.f),e); }

        body_corners(b,corn);
        rect_wall_contact(corn,1,false,0.f,&pen,&cp);
        if(pen>0.f){ b->pos.x+=pen;
            resolve_with_wall(b,cp,v2(-1.f,0.f),e); }

        body_corners(b,corn);
        rect_wall_contact(corn,1,true,ww,&pen,&cp);
        if(pen>0.f){ b->pos.x-=pen;
            resolve_with_wall(b,cp,v2(1.f,0.f),e); }
    }
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

static Body     g_bodies[N_BODIES_MAX];
static int      g_nbodies  = 0;
static int      g_ncircles = 0;   /* cumulative, for color cycling */
static int      g_nrects   = 0;
static float    g_gravity  = GRAVITY_DEF;
static float    g_rest     = REST_DEF;
static bool     g_gravity_on = true;
static int      g_steps    = STEPS_DEF;
static bool     g_paused   = false;
static long     g_tick     = 0;

static uint32_t g_rng = 1u;
static float rng_f(void)
{
    g_rng ^= g_rng<<13; g_rng ^= g_rng>>17; g_rng ^= g_rng<<5;
    return (float)(g_rng>>8) / (float)(1u<<24);
}

static Body make_circle(float x,float y,float r,float vx,float vy,int cp)
{
    Body b={0}; b.pos=v2(x,y); b.vel=v2(vx,vy);
    b.shape=SHAPE_CIRCLE; b.radius=r; b.color_pair=cp;
    body_init_mass(&b); return b;
}
static Body make_rect(float x,float y,float hw,float hh,
                      float angle,float vx,float vy,int cp)
{
    Body b={0}; b.pos=v2(x,y); b.vel=v2(vx,vy);
    b.shape=SHAPE_RECT; b.hw=hw; b.hh=hh; b.angle=angle;
    b.color_pair=cp; body_init_mass(&b); return b;
}

static bool scene_add_circle(void)
{
    if(g_nbodies>=N_BODIES_MAX) return false;
    float ww=(float)WORLD_W();
    float r=CIRC_R_DEF;
    float x=ww*(0.15f+rng_f()*0.70f);
    if(x<r+1.f) x=r+1.f;
    if(x>ww-r-1.f) x=ww-r-1.f;
    float y=r+1.f;   /* near top */
    float vx=(rng_f()-0.5f)*2.f, vy=0.f;
    int cp=1+(g_ncircles%8);
    g_bodies[g_nbodies++]=make_circle(x,y,r,vx,vy,cp);
    g_ncircles++;
    return true;
}

static bool scene_add_rect(void)
{
    if(g_nbodies>=N_BODIES_MAX) return false;
    float ww=(float)WORLD_W();
    float hw=RECT_HW_DEF, hh=RECT_HH_DEF;
    float x=ww*(0.15f+rng_f()*0.70f);
    if(x<hw+1.f) x=hw+1.f;
    if(x>ww-hw-1.f) x=ww-hw-1.f;
    float y=hh+1.f;   /* near top */
    float angle=(rng_f()-0.5f)*0.4f;   /* small random tilt */
    float vx=(rng_f()-0.5f)*2.f, vy=0.f;
    int cp=1+(g_nrects%8);
    g_bodies[g_nbodies++]=make_rect(x,y,hw,hh,angle,vx,vy,cp);
    g_nrects++;
    return true;
}

static void scene_remove_last(void)
{ if(g_nbodies>1) g_nbodies--; } /* index 0 is the fixed floor — never remove */

static void scene_build_floor(void)
{
    /* Fixed floor body — infinite mass, participates in body-body collision.
     * Center sits below WORLD_H() so its top surface is exactly at WORLD_H(). */
    float ww=(float)WORLD_W(), wh=(float)WORLD_H();
    float hh=20.f;   /* thick so nothing tunnels through */
    Body fl={0};
    fl.shape=SHAPE_RECT;
    fl.pos  =v2(ww*0.5f, wh+hh);
    fl.hw   =ww*0.5f+10.f;   /* slightly wider than screen */
    fl.hh   =hh;
    fl.fixed=true;
    fl.color_pair=CP_FLOOR;
    body_init_mass(&fl);
    g_bodies[0]=fl;
    if(g_nbodies<1) g_nbodies=1;
}

static void scene_init(void)
{
    g_nbodies=0; g_ncircles=0; g_nrects=0; g_tick=0;
    float ww=(float)WORLD_W(), wh=(float)WORLD_H();
    float cx=ww*0.5f;

    /* Index 0: fixed floor rigid body */
    scene_build_floor();

    /* One rect sitting on floor, centred */
    float hw=RECT_HW_DEF, hh=RECT_HH_DEF;
    g_bodies[g_nbodies++]=make_rect(cx,wh-hh,hw,hh,0.f,0.f,0.f,1);
    g_nrects++;

    /* One circle at top, centred — will fall and hit the rect */
    float r=CIRC_R_DEF;
    g_bodies[g_nbodies++]=make_circle(cx,r+1.f,r,0.f,0.f,2);
    g_ncircles++;
}

static void scene_step(void)
{
    float ww=(float)WORLD_W(), wh=(float)WORLD_H();

    /* Gravity — sleeping bodies are excluded (they are already at rest) */
    if(g_gravity_on)
        for(int i=0;i<g_nbodies;i++)
            if(!g_bodies[i].fixed && !g_bodies[i].sleeping)
                g_bodies[i].vel.y+=g_gravity;

    /* Integrate — sleeping bodies skip (no gravity accumulated, stay put) */
    for(int i=0;i<g_nbodies;i++){
        Body *b=&g_bodies[i]; if(b->fixed || b->sleeping) continue;
        b->pos  =v2add(b->pos,b->vel);
        b->angle+=b->omega;
        b->vel  =v2scale(b->vel,  DAMPING_LIN);
        b->omega*=DAMPING_ANG;
        /* Velocity cap — dense stacking can build runaway impulses */
        float vl=v2len(b->vel);
        if(vl>MAX_VEL) b->vel=v2scale(b->vel,MAX_VEL/vl);
        if(fabsf(b->omega)>MAX_OMEGA)
            b->omega=b->omega>0.f?MAX_OMEGA:-MAX_OMEGA;
    }

    /* Body-body collisions — 3 iterations for stability */
    for(int iter=0;iter<3;iter++)
        for(int i=0;i<g_nbodies;i++)
            for(int j=i+1;j<g_nbodies;j++){
                Body *a=&g_bodies[i], *b=&g_bodies[j];
                if(a->sleeping && b->sleeping) continue; /* both frozen, skip */
                Contact c={false};
                if(a->shape==SHAPE_CIRCLE&&b->shape==SHAPE_CIRCLE) c=collide_cc(a,b);
                else if(a->shape==SHAPE_CIRCLE&&b->shape==SHAPE_RECT)
                { c=collide_cr(a,b); c.normal=v2scale(c.normal,-1.f); }
                else if(a->shape==SHAPE_RECT&&b->shape==SHAPE_CIRCLE)
                { c=collide_cr(b,a); /* n already points from rect(a) to circle(b) */ }
                else c=collide_rr(a,b);
                if(c.valid) resolve_contact(a,b,c,g_rest);
            }

    /* Side-wall collision — floor is handled by the fixed floor rigid body */
    for(int i=0;i<g_nbodies;i++){
        if(g_bodies[i].sleeping) continue;
        wall_collide(&g_bodies[i],g_rest,ww);
        /* Emergency bounds clamp — catches any body that tunnelled through a wall.
         * Uses separate AABB half-extents for x and y so rects rest correctly. */
        Body *b=&g_bodies[i];
        float xm,ym;
        if(b->shape==SHAPE_CIRCLE){
            xm=ym=b->radius;
        } else {
            float ca=fabsf(cosf(b->angle)), sa=fabsf(sinf(b->angle));
            xm=b->hw*ca+b->hh*sa;   /* AABB half-width  */
            ym=b->hw*sa+b->hh*ca;   /* AABB half-height */
        }
        if(b->pos.x<xm)    { b->pos.x=xm;    b->vel.x=0.f; }
        if(b->pos.x>ww-xm) { b->pos.x=ww-xm; b->vel.x=0.f; }
        if(b->pos.y<ym)    { b->pos.y=ym;     b->vel.y=0.f; }
        if(b->pos.y>wh-ym) { b->pos.y=wh-ym;  b->vel.y=0.f; }
    }

    /* Sleep counter — body must hold low velocity for SLEEP_FRAMES consecutive
     * frames before being declared asleep.  Once asleep it skips gravity and
     * integration, staying perfectly still until woken by a collision. */
    for(int i=0;i<g_nbodies;i++){
        Body *b=&g_bodies[i]; if(b->fixed || b->sleeping) continue;
        if(v2len(b->vel)<SLEEP_VEL && fabsf(b->omega)<SLEEP_OMEGA){
            if(++b->sleep_ticks >= SLEEP_FRAMES){
                b->vel=v2(0.f,0.f); b->omega=0.f;
                b->sleeping=true;
            }
        } else {
            b->sleep_ticks=0;
        }
    }

    g_tick++;
}

/* ===================================================================== */
/* §9  draw                                                               */
/* ===================================================================== */

static void scene_draw(void)
{
    erase(); fb_clear();

    /* Floor */
    int floor_row=prow((float)WORLD_H());
    attron(COLOR_PAIR(CP_FLOOR));
    for(int c=0;c<g_cols;c++) mvaddch(floor_row,c,'=');
    attroff(COLOR_PAIR(CP_FLOOR));

    for(int i=0;i<g_nbodies;i++)
        if(!g_bodies[i].fixed) fb_body(&g_bodies[i]);
    fb_flush();

    /* Count dynamic bodies only */
    int nc=0,nr=0;
    for(int i=0;i<g_nbodies;i++){
        if(g_bodies[i].fixed) continue;
        if(g_bodies[i].shape==SHAPE_CIRCLE) nc++; else nr++;
    }

    int rows; { int cc; getmaxyx(stdscr,rows,cc); (void)cc; }
    attron(COLOR_PAIR(CP_HUD)|A_BOLD);
    mvprintw(rows-2,0,
        " [c]rect [s]circle [x]del"
        "  rects:%-2d circles:%-2d /%-2d"
        "  rest:%.2f  grav:%s  tick:%-5ld  theme:%s  %s",
        nr,nc,N_BODIES_MAX, g_rest,
        g_gravity_on?"on ":"off",g_tick,
        k_themes[g_theme].name, g_paused?"[PAUSED]":"");
    attroff(COLOR_PAIR(CP_HUD)|A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    mvaddstr(rows-1,0,
        "  [e/E]restitution  [g]gravity  [t/T]theme"
        "  [r]reset  [p]pause  [q]quit");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §10  screen                                                            */
/* ===================================================================== */

static volatile sig_atomic_t g_resize=0,g_quit=0;
static void on_sigwinch(int s){(void)s;g_resize=1;}
static void on_sigterm (int s){(void)s;g_quit=1;}

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr,TRUE); nodelay(stdscr,TRUE);
    curs_set(0); typeahead(-1);
    start_color(); g_has_256=(COLORS>=256);
    theme_apply(g_theme);
}
static void screen_resize(void)
{
    endwin(); refresh();
    int rows,cols; getmaxyx(stdscr,rows,cols);
    g_rows=(rows<ROWS_MAX)?rows:ROWS_MAX;
    g_cols=(cols<COLS_MAX)?cols:COLS_MAX;
    g_resize=0;
    scene_build_floor(); /* floor geometry depends on terminal size */
}

/* ===================================================================== */
/* §11  app                                                               */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH,on_sigwinch);
    signal(SIGTERM, on_sigterm);
    signal(SIGINT,  on_sigterm);

    g_rng=(uint32_t)time(NULL)^0xDEAD1234u;
    screen_init();
    { int rows,cols; getmaxyx(stdscr,rows,cols);
      g_rows=(rows<ROWS_MAX)?rows:ROWS_MAX;
      g_cols=(cols<COLS_MAX)?cols:COLS_MAX; }
    scene_init();
    int64_t next_tick=clock_ns();

    while(!g_quit){
        int ch;
        while((ch=getch())!=ERR){
            switch(ch){
            case 'q': case 27: g_quit=1; break;
            case 'p': case ' ': g_paused=!g_paused; break;
            case 'r': scene_init(); break;
            case 'c': scene_add_rect();   break;
            case 's': scene_add_circle(); break;
            case 'x': scene_remove_last(); break;
            case 'g': g_gravity_on=!g_gravity_on; break;
            case 'e': if(g_rest<0.95f) g_rest+=REST_STEP; break;
            case 'E': if(g_rest>0.05f) g_rest-=REST_STEP; break;
            case 't': g_theme=(g_theme+1)%N_THEMES; theme_apply(g_theme); break;
            case 'T': g_theme=(g_theme+N_THEMES-1)%N_THEMES; theme_apply(g_theme); break;
            }
        }
        if(g_resize){ screen_resize(); scene_init(); }

        int64_t now=clock_ns();
        if(!g_paused && now>=next_tick){
            for(int s=0;s<g_steps;s++) scene_step();
            next_tick=now+TICK_NS(SIM_FPS);
        }
        scene_draw();
        wnoutrefresh(stdscr); doupdate();
        clock_sleep_ns(next_tick-clock_ns()-1000000LL);
    }
    endwin(); return 0;
}
