/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * path_tracer.c — Progressive Monte Carlo path tracer · Cornell Box
 *
 * Algorithm: Unidirectional path tracing with cosine-weighted hemisphere
 *   sampling and Russian roulette termination.  Each frame adds SPP samples
 *   to a per-pixel accumulator.  Display = accumulator / sample_count,
 *   tone-mapped with Reinhard and gamma-encoded.  The image converges from
 *   noisy → clean over ~512 samples — watching it settle is the animation.
 *
 * Scene: Classic Cornell Box
 *   · Red left wall · Green right wall · White floor / ceiling / back
 *   · Warm area light (y=0.98, centred overhead)
 *   · Gold sphere (left) · Indigo sphere (right)
 *   Color bleeding: red and green walls tint nearby surfaces over many bounces.
 *
 * Color: 216-colour xterm cube (same as sphere_raytrace.c).
 *   RGB fragment → nearest 6×6×6 entry → colour pair.
 *   Luminance → Bourke ASCII density ramp.  A_BOLD for full saturation.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/path_tracer.c \
 *       -o path_tracer -lncurses -lm
 *
 * Keys:
 *   r          reset accumulator (restart convergence from sample 0)
 *   p          pause / resume sampling
 *   + / =      more samples per frame (faster convergence, lower fps)
 *   -          fewer samples per frame (higher fps, slower convergence)
 *   q / ESC    quit
 *
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config       §2  clock        §3  vec3
 *   §4  RNG          §5  scene        §6  intersection
 *   §7  path trace   §8  framebuffer  §9  screen + main
 * ─────────────────────────────────────────────────────────────────────
 */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define TARGET_FPS     30        /* 30 fps target — PT is compute-heavy    */
#define ASPECT         0.47f     /* terminal cell width / height            */
#define FOV_DEG        66.0f     /* horizontal field of view in degrees     */

#define MAX_DEPTH      7         /* maximum bounce depth per path           */
#define RR_DEPTH       3         /* Russian roulette starts at this depth   */
#define SPP_DEFAULT    2         /* samples added to accumulator per frame  */
#define SPP_MIN        1
#define SPP_MAX        8
#define ACCUM_CAP      8192      /* auto-pause accumulator after this many  */

#define MAX_W          320       /* static accumulator width bound          */
#define MAX_H          100       /* static accumulator height bound         */

/*
 * Cornell box coordinate system
 *   x ∈ [-1, 1]   left (red) → right (green)
 *   y ∈ [-1, 1]   floor → ceiling
 *   z ∈ [ 0, 2]   open front → back wall
 *   Camera at (0, 0.05, -1.5) looking toward +Z, no front wall.
 */
#define CAM_X          0.00f
#define CAM_Y          0.05f
#define CAM_Z         -1.50f

/* Bourke density ramp — darker chars on left, denser on right */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  vec3                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3     (float x,float y,float z){ return (V3){x,y,z}; }
static inline V3    v3add  (V3 a,V3 b)  { return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline V3    v3sub  (V3 a,V3 b)  { return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline V3    v3mul  (V3 a,V3 b)  { return v3(a.x*b.x,a.y*b.y,a.z*b.z); }
static inline V3    v3s    (float s,V3 a){ return v3(s*a.x,s*a.y,s*a.z); }
static inline float v3dot  (V3 a,V3 b)  { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3len  (V3 a)       { return sqrtf(v3dot(a,a)); }
static inline V3    v3norm (V3 a)       { float l=v3len(a); return l>1e-9f?v3s(1.f/l,a):v3(0,1,0); }
static inline V3    v3cross(V3 a,V3 b)  { return v3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x); }
static inline float v3maxc (V3 a)       { return a.x>a.y?(a.x>a.z?a.x:a.z):(a.y>a.z?a.y:a.z); }

/* ===================================================================== */
/* §4  RNG — xorshift32, one independent state per pixel per frame       */
/* ===================================================================== */

typedef uint32_t Rng;

/* Fast xorshift32 — returns uniform float in [0, 1) */
static float rng_f(Rng *r)
{
    *r ^= *r << 13;
    *r ^= *r >> 17;
    *r ^= *r << 5;
    return (float)(*r >> 1) * (1.f / (float)0x7FFFFFFF);
}

/* Seed from pixel + frame so adjacent pixels are decorrelated */
static Rng rng_seed(int px, int py, int frame)
{
    uint32_t s = (uint32_t)(px * 1973 + py * 9277 + frame * 26699 + 1);
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;   /* warm up xorshift */
    return s ? s : 1u;
}

/* ===================================================================== */
/* §5  scene — materials, quads, spheres                                 */
/* ===================================================================== */

/*
 * Material: purely diffuse (Lambertian).
 *   albedo — surface reflectance [0,1]^3
 *   emit   — radiance emitted; (0,0,0) for non-emitting surfaces
 *
 * Path tracing with Lambertian BRDF + cosine hemisphere sampling:
 *   BRDF = albedo / π,  PDF = cosθ / π  →  Monte Carlo weight = albedo.
 * Emission hits terminate the path and add throughput × emit to the pixel.
 */
typedef struct { V3 albedo; V3 emit; } Mat;

static const Mat k_mats[] = {
    /* 0  white  */ { {0.73f, 0.73f, 0.73f}, {0,   0,   0   } },
    /* 1  red    */ { {0.65f, 0.05f, 0.05f}, {0,   0,   0   } },
    /* 2  green  */ { {0.12f, 0.45f, 0.15f}, {0,   0,   0   } },
    /* 3  light  */ { {0,     0,     0    }, {15.f,14.f,11.f} }, /* warm white  */
    /* 4  gold   */ { {0.80f, 0.58f, 0.18f}, {0,   0,   0   } },
    /* 5  indigo */ { {0.22f, 0.28f, 0.82f}, {0,   0,   0   } },
};

/*
 * Axis-aligned quad: one axis fixed at `pos`, bounded in the other two.
 *
 *   axis=0 (X plane): lo/hi cover {Y, Z}
 *   axis=1 (Y plane): lo/hi cover {X, Z}
 *   axis=2 (Z plane): lo/hi cover {X, Y}
 */
typedef struct {
    int   axis;
    float pos;
    float lo[2], hi[2];
    int   mat;
} Quad;

/*
 * Cornell box quads — open on the front (z=0) side so the camera can see in.
 * The light quad at y=0.98 sits just below the ceiling (y=1.0); rays going
 * upward through the light's XZ footprint hit it first (smaller t) and
 * receive direct illumination.
 */
static const Quad k_quads[] = {
    /* floor    y=-1  x∈[-1,1] z∈[0,2] */ { 1,-1.0f, {-1.f,0.f}, {1.f,2.f}, 0 },
    /* ceiling  y=+1  x∈[-1,1] z∈[0,2] */ { 1, 1.0f, {-1.f,0.f}, {1.f,2.f}, 0 },
    /* back     z=+2  x∈[-1,1] y∈[-1,1]*/ { 2, 2.0f, {-1.f,-1.f},{1.f,1.f}, 0 },
    /* left     x=-1  y∈[-1,1] z∈[0,2] */ { 0,-1.0f, {-1.f,0.f}, {1.f,2.f}, 1 },
    /* right    x=+1  y∈[-1,1] z∈[0,2] */ { 0, 1.0f, {-1.f,0.f}, {1.f,2.f}, 2 },
    /* light    y=0.98 centred overhead  */ { 1, 0.98f,{-0.36f,0.62f},{0.36f,1.38f},3 },
};
#define N_QUADS ((int)(sizeof k_quads / sizeof k_quads[0]))

/* Sphere */
typedef struct { V3 c; float r; int mat; } Sphere;

/*
 * Two spheres just above the floor (bottom at y≈-0.98, floor at y=-1.0).
 * Kept 0.02 above floor to avoid numerical self-intersection at contact.
 */
static const Sphere k_spheres[] = {
    { {-0.46f,-0.60f,0.82f}, 0.38f, 4 },   /* gold   sphere, left  */
    { { 0.44f,-0.60f,1.16f}, 0.38f, 5 },   /* indigo sphere, right */
};
#define N_SPHERES ((int)(sizeof k_spheres / sizeof k_spheres[0]))

/* ===================================================================== */
/* §6  intersection                                                       */
/* ===================================================================== */

/*
 * ray_quad — intersect ray (ro, rd) with an axis-aligned quad.
 *
 * Computes t = (pos − ro[axis]) / rd[axis], then checks the hit point
 * lies within [lo, hi] in the two remaining axes.  The returned normal
 * always faces toward the incoming ray (for correct hemisphere sampling).
 */
static int ray_quad(V3 ro, V3 rd, const Quad *q,
                    float t_min, float *t_out, V3 *n_out)
{
    float dc, oc;
    switch (q->axis) {
    case 0: dc=rd.x; oc=ro.x; break;
    case 1: dc=rd.y; oc=ro.y; break;
    default:dc=rd.z; oc=ro.z; break;
    }
    if (fabsf(dc) < 1e-9f) return 0;
    float t = (q->pos - oc) / dc;
    if (t < t_min) return 0;

    float px=ro.x+t*rd.x, py=ro.y+t*rd.y, pz=ro.z+t*rd.z;
    float u, vv;
    switch (q->axis) {
    case 0: u=py; vv=pz; break;
    case 1: u=px; vv=pz; break;
    default:u=px; vv=py; break;
    }
    if (u<q->lo[0]||u>q->hi[0]||vv<q->lo[1]||vv>q->hi[1]) return 0;

    /* normal always points against the incoming ray direction */
    V3 gn = {0,0,0};
    switch (q->axis) {
    case 0: gn.x = dc>0?-1.f:1.f; break;
    case 1: gn.y = dc>0?-1.f:1.f; break;
    default:gn.z = dc>0?-1.f:1.f; break;
    }
    *t_out = t;
    *n_out = gn;
    return 1;
}

/*
 * ray_sphere — analytic sphere intersection.
 *   |ro + t·rd − c|² = r²  →  quadratic in t.
 * Returns the nearest positive t beyond t_min, or 0 on miss.
 */
static int ray_sphere(V3 ro, V3 rd, const Sphere *s,
                      float t_min, float *t_out)
{
    V3    oc   = v3sub(ro, s->c);
    float b    = v3dot(rd, oc);
    float disc = b*b - v3dot(oc,oc) + s->r*s->r;
    if (disc < 0.f) return 0;
    float sq = sqrtf(disc);
    float t  = -b - sq;
    if (t < t_min) t = -b + sq;
    if (t < t_min) return 0;
    *t_out = t;
    return 1;
}

/* Hit record: position, normal (facing ray), material index */
typedef struct { float t; V3 P, N; int mat; } Hit;

/* Test all quads and spheres, fill closest Hit, return 1 on any hit */
static int scene_hit(V3 ro, V3 rd, float t_min, Hit *h)
{
    float t_best = 1e30f;
    int   any    = 0;

    for (int i = 0; i < N_QUADS; i++) {
        float t; V3 n;
        if (ray_quad(ro, rd, &k_quads[i], t_min, &t, &n) && t < t_best) {
            t_best = t;
            h->t   = t;
            h->P   = v3add(ro, v3s(t, rd));
            h->N   = n;
            h->mat = k_quads[i].mat;
            any    = 1;
        }
    }
    for (int i = 0; i < N_SPHERES; i++) {
        float t;
        if (ray_sphere(ro, rd, &k_spheres[i], t_min, &t) && t < t_best) {
            t_best = t;
            h->t = t;
            h->P = v3add(ro, v3s(t, rd));
            V3 outN = v3norm(v3sub(h->P, k_spheres[i].c));
            /* flip outward normal to face the incoming ray */
            h->N   = v3dot(outN, rd) < 0.f ? outN : v3s(-1.f, outN);
            h->mat = k_spheres[i].mat;
            any    = 1;
        }
    }
    return any;
}

/* ===================================================================== */
/* §7  path trace                                                         */
/* ===================================================================== */

/*
 * onb — orthonormal basis around normal n.
 * Builds tangent (u) and bitangent (v) perpendicular to n using the
 * "pick a non-parallel up vector" trick.
 */
static void onb(V3 n, V3 *u, V3 *v)
{
    V3 up = fabsf(n.x) < 0.9f ? v3(1,0,0) : v3(0,1,0);
    *u = v3norm(v3cross(up, n));
    *v = v3cross(n, *u);
}

/*
 * cos_sample_hemi — sample a direction from the cosine-weighted hemisphere
 * above n.  Malley's method: uniform disk sample projected onto hemisphere.
 *
 *   r1 = uniform [0,1)  → azimuth phi = 2π·r1
 *   r2 = uniform [0,1)  → cos²θ = r2  →  sinθ = √r2, cosθ = √(1−r2)
 *
 * PDF = cosθ/π.  Paired with Lambertian BRDF = albedo/π, the weight
 * simplifies to just `albedo` (the π and cosθ terms cancel).
 */
static V3 cos_sample_hemi(V3 n, Rng *rng)
{
    float r1  = rng_f(rng);
    float r2  = rng_f(rng);
    float phi = 2.f * (float)M_PI * r1;
    float sr2 = sqrtf(r2);                        /* sinθ */
    float lx  = cosf(phi) * sr2;
    float ly  = sinf(phi) * sr2;
    float lz  = sqrtf(1.f - r2);                  /* cosθ */
    V3 u, vv;
    onb(n, &u, &vv);
    return v3norm(v3add(v3s(lx,u), v3add(v3s(ly,vv), v3s(lz,n))));
}

/*
 * path_trace — recursive path tracer returning outgoing radiance for one ray.
 *
 * Loop (not recursive to avoid stack overflow at high depth):
 *   ① scene_hit — find nearest surface
 *   ② if emissive: add throughput × emission, terminate
 *   ③ Russian roulette (depth ≥ RR_DEPTH): survive with prob p = max(throughput)
 *   ④ diffuse bounce: throughput *= albedo; sample cosine hemisphere
 *   ⑤ offset ray origin by 1e-4 × N to avoid self-intersection
 */
static V3 path_trace(V3 ro, V3 rd, Rng *rng)
{
    V3 col        = v3(0,0,0);
    V3 throughput = v3(1,1,1);

    for (int depth = 0; depth < MAX_DEPTH; depth++) {
        Hit h;
        if (!scene_hit(ro, rd, 1e-4f, &h)) break;   /* escaped: black sky */

        const Mat *m = &k_mats[h.mat];

        /* emissive surface: contribute and stop */
        if (m->emit.x > 0.f || m->emit.y > 0.f || m->emit.z > 0.f) {
            col = v3add(col, v3mul(throughput, m->emit));
            break;
        }

        /* Russian roulette — terminate dim paths probabilistically */
        if (depth >= RR_DEPTH) {
            float p = v3maxc(throughput);
            if (p < 1e-4f || rng_f(rng) > p) break;
            throughput = v3s(1.f / p, throughput);
        }

        /* Lambertian bounce: weight = albedo (cosθ/π factors cancel) */
        throughput = v3mul(throughput, m->albedo);
        rd = cos_sample_hemi(h.N, rng);
        ro = v3add(h.P, v3s(1e-4f, h.N));   /* push off surface */
    }
    return col;
}

/* ===================================================================== */
/* §8  framebuffer — progressive accumulator                             */
/* ===================================================================== */

/*
 * Accumulator: per-pixel running sum of RGB radiance from all samples.
 * Display = accum / g_samples, then Reinhard tone-map + gamma encode.
 *
 * On resize or 'r' keypress: memset to zero, g_samples = 0.
 * Sampling stops automatically at ACCUM_CAP to save CPU once converged.
 */
static float g_accum[MAX_H][MAX_W][3];
static int   g_samples = 0;

static void accum_reset(void)
{
    memset(g_accum, 0, sizeof g_accum);
    g_samples = 0;
}

static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x)
{
    return powf(x < 0.f ? 0.f : (x > 1.f ? 1.f : x), 1.f / 2.2f);
}

/*
 * accum_add_frame — cast `spp` paths per pixel and add to the accumulator.
 *
 * Camera: pinhole at (CAM_X, CAM_Y, CAM_Z) looking toward +Z.
 *   fov_tan = tan(FOV/2) — projects pixel (col,row) to ray direction.
 *   ASPECT (0.47) corrects for non-square terminal cells: terminal rows are
 *   ~1/0.47 ≈ 2.13× taller in screen pixels than terminal columns, so we
 *   must squish the vertical angular extent by ASPECT to get isotropic rays.
 *
 * Jitter: sub-pixel random offset per sample gives free anti-aliasing.
 */
static void accum_add_frame(int cols, int rows, int spp, int frame_idx)
{
    float fov_tan = tanf(FOV_DEG * (float)M_PI / 360.f);
    float cx = cols * .5f, cy = rows * .5f;

    V3 cam_pos = v3(CAM_X, CAM_Y, CAM_Z);
    V3 cam_fwd = v3(0, 0, 1);
    V3 cam_rgt = v3(1, 0, 0);
    V3 cam_up  = v3(0, 1, 0);

    for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
        for (int col = 0; col < cols && col < MAX_W; col++) {
            float sr = 0.f, sg = 0.f, sb = 0.f;

            for (int s = 0; s < spp; s++) {
                Rng rng = rng_seed(col, row, frame_idx * spp + s);

                /* jitter inside the pixel footprint → anti-aliasing */
                float jx = rng_f(&rng) - .5f;
                float jy = rng_f(&rng) - .5f;
                float pu = ((col + jx) - cx) / cx * fov_tan;
                float pv = -((row + jy) - cy) / cx * fov_tan / ASPECT;

                V3 rd = v3norm(v3add(cam_fwd,
                               v3add(v3s(pu, cam_rgt),
                                     v3s(pv, cam_up))));

                V3 c = path_trace(cam_pos, rd, &rng);
                sr += c.x; sg += c.y; sb += c.z;
            }

            g_accum[row][col][0] += sr;
            g_accum[row][col][1] += sg;
            g_accum[row][col][2] += sb;
        }
    }
    g_samples += spp;
}

/* ===================================================================== */
/* §9  screen + main                                                      */
/* ===================================================================== */

static int g_256 = 0;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);
    if (g_256) {
        /* 216-colour xterm cube: index 16+r*36+g*6+b, stored as pairs 1..216 */
        for (int i = 0; i < 216; i++)
            init_pair(i + 1, 16 + i, -1);
        init_pair(217, 15,  -1);   /* bright white — HUD           */
        init_pair(218, 11,  -1);   /* bright yellow — title accent */
    } else {
        init_pair(1,   COLOR_WHITE,  -1);
        init_pair(217, COLOR_WHITE,  -1);
        init_pair(218, COLOR_YELLOW, -1);
    }
}

/* Map gamma-corrected RGB [0,1] to xterm-256 color pair 1..216 */
static int rgb_to_pair(float r, float g, float b)
{
    int ri = (int)(r * 5.f + .5f); if(ri>5)ri=5; if(ri<0)ri=0;
    int gi = (int)(g * 5.f + .5f); if(gi>5)gi=5; if(gi<0)gi=0;
    int bi = (int)(b * 5.f + .5f); if(bi>5)bi=5; if(bi<0)bi=0;
    return ri*36 + gi*6 + bi + 1;
}

/*
 * accum_draw — tone-map and display the current accumulator.
 *
 * Per pixel:
 *   ① divide by g_samples → linear average radiance
 *   ② Reinhard per channel: x/(1+x) — compresses [0,∞) → [0,1)
 *   ③ gamma encode (1/2.2) — perceptual sRGB
 *   ④ luminance → Bourke char density ramp (space = dark, @ = bright)
 *   ⑤ RGB → nearest 6×6×6 xterm colour → colour pair + A_BOLD
 */
static void accum_draw(int cols, int rows)
{
    if (g_samples == 0) return;
    float inv = 1.f / (float)g_samples;

    for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
        for (int col = 0; col < cols && col < MAX_W; col++) {
            float r = g_accum[row][col][0] * inv;
            float g = g_accum[row][col][1] * inv;
            float b = g_accum[row][col][2] * inv;

            r = gamma_enc(reinhard(r));
            g = gamma_enc(reinhard(g));
            b = gamma_enc(reinhard(b));

            /* perceptual luminance → char density */
            float luma = 0.2126f*r + 0.7152f*g + 0.0722f*b;
            int   ri   = (int)(luma * (float)(RAMP_LEN - 1) + .5f);
            if (ri < 0) ri = 0;
            if (ri >= RAMP_LEN) ri = RAMP_LEN - 1;
            char ch = k_ramp[ri];

            if (g_256) {
                int pair = rgb_to_pair(r, g, b);
                attron(COLOR_PAIR(pair) | A_BOLD);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(pair) | A_BOLD);
            } else {
                mvaddch(row, col, (chtype)(unsigned char)ch);
            }
        }
    }
}

static void draw_hud(int cols, int rows, float fps,
                     int spp, int samples, bool paused)
{
    /* convergence progress bar — fills as samples accumulate */
    int bar_w  = cols / 3;
    int filled = (int)((float)samples / (float)ACCUM_CAP * bar_w);
    if (filled > bar_w) filled = bar_w;

    attron(COLOR_PAIR(218) | A_BOLD);
    mvprintw(0, 1, " PATH TRACER · CORNELL BOX ");
    attroff(COLOR_PAIR(218) | A_BOLD);

    attron(COLOR_PAIR(217));
    mvprintw(0, 30, " spp/frame:%-2d  samples:%-5d  fps:%-4.0f  %s ",
             spp, samples, (double)fps, paused ? "[PAUSED]" : "tracing ");
    attroff(COLOR_PAIR(217));

    /* convergence bar */
    attron(COLOR_PAIR(218) | A_DIM);
    mvprintw(rows - 1, 1, "[");
    attroff(COLOR_PAIR(218) | A_DIM);

    for (int i = 0; i < bar_w; i++) {
        int pair = (i < filled) ? 4 : 1;      /* green filled, dim empty */
        attron(COLOR_PAIR(pair) | (i < filled ? A_BOLD : A_DIM));
        mvaddch(rows - 1, 2 + i, i < filled ? '=' : '-');
        attroff(COLOR_PAIR(pair) | (i < filled ? A_BOLD : A_DIM));
    }

    attron(COLOR_PAIR(218) | A_DIM);
    mvprintw(rows - 1, 2 + bar_w, "] ");
    attroff(COLOR_PAIR(218) | A_DIM);

    attron(COLOR_PAIR(217) | A_DIM);
    mvprintw(rows - 1, 4 + bar_w,
             " r:reset  p:pause  +/-:spp  q:quit");
    attroff(COLOR_PAIR(217) | A_DIM);
}

static volatile sig_atomic_t g_run    = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint  (int s){ (void)s; g_run    = 0; }
static void on_sigwinch(int s){ (void)s; g_resize = 1; }

int main(void)
{
    signal(SIGINT,   on_sigint);
    signal(SIGWINCH, on_sigwinch);

    initscr(); cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE); typeahead(-1);

    int cols, rows;
    getmaxyx(stdscr, rows, cols);
    color_init();
    accum_reset();

    int       spp       = SPP_DEFAULT;
    bool      paused    = false;
    float     fps       = 0.f;
    long long fps_acc   = 0;
    int       fps_cnt   = 0;
    long long frame_ns  = 1000000000LL / TARGET_FPS;
    long long last      = clock_ns();
    int       frame_idx = 0;

    while (g_run) {
        /* ── handle terminal resize ───────────────────────────────── */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            accum_reset();
            frame_idx = 0;
        }

        /* ── wall-clock dt, capped at 200 ms ─────────────────────── */
        long long now = clock_ns();
        long long dt  = now - last;
        if (dt > 200000000LL) dt = 200000000LL;
        last = now;

        /* ── FPS counter (500 ms window) ─────────────────────────── */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps     = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* ── add samples — stop sampling once cap reached ─────────── */
        if (!paused && g_samples < ACCUM_CAP) {
            accum_add_frame(cols, rows, spp, frame_idx++);
        }

        /* ── render: erase → draw accum → HUD → present ─────────── */
        long long t0 = clock_ns();
        erase();
        accum_draw(cols, rows);
        draw_hud(cols, rows, fps, spp, g_samples, paused);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;              break;
        case 'p': case 'P': paused = !paused;                 break;
        case 'r': case 'R':
            accum_reset(); frame_idx = 0;                     break;
        case '+': case '=':
            if (spp < SPP_MAX) spp++;
            accum_reset(); frame_idx = 0;                     break;
        case '-':
            if (spp > SPP_MIN) spp--;
            accum_reset(); frame_idx = 0;                     break;
        default: break;
        }

        /* ── frame cap: sleep remainder of 1/30s budget ─────────── */
        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }

    endwin();
    return 0;
}
