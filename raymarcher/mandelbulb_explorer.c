/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb_explorer.c  —  ncurses ASCII Mandelbulb 3-D fractal explorer
 *
 * Features: smooth-iteration coloring, near-miss edge glow, animated
 * palette, power morphing, starfield background, rim lighting, soft
 * shadows, ambient occlusion, camera orbit, zoom fly-in.
 *
 * Keys:
 *   q / ESC     quit
 *   space       pause / resume auto-orbit
 *   arrow keys  orbit camera (theta/phi)
 *   p / P       power +1 / -1  (2..16, default 8)
 *   m           toggle power morph (2 ↔ 8 oscillation)
 *   t           cycle trap type  (point / plane / ring)
 *   c           cycle colour theme (7 themes)
 *   z / Z       zoom in / out (camera distance)
 *   a           toggle auto-zoom fly-in
 *   f           toggle fast mode (fewer march steps)
 *   o           toggle ambient occlusion
 *   s           toggle soft shadows
 *   r           reset camera
 *   [ / ]       decrease / increase orbit speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/mandelbulb_explorer.c \
 *       -o mandelbulb -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — every tunable constant
 *   §2  clock       — monotonic ns clock + sleep
 *   §3  color       — smooth-iter color mapping + animated palette
 *   §4  vec3        — inline 3-D vector math
 *   §5  mandelbulb  — DE+smooth, normal, shadow, AO, march+glow
 *   §6  canvas      — framebuffer, starfield, progressive render, draw
 *   §7  scene       — owns state + tick (morph, palette) + render + draw
 *   §8  screen      — ncurses stdscr wrapper
 *   §9  app         — dt loop, input, resize, cleanup
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
    SIM_FPS         = 30,
    FPS_UPDATE_MS   = 500,
    HUD_ROWS        = 3,
};

/* Terminal cell aspect — same as raymarcher.c */
#define CELL_W       1
#define CELL_H       1
#define CELL_ASPECT  2.0f

static inline int canvas_w_from_cols(int c) { return c / CELL_W; }
static inline int canvas_h_from_rows(int r) { return r / CELL_H; }

/* Canvas size limits for static framebuffer */
#define CANVAS_MAX_W  400
#define CANVAS_MAX_H  120

/* Raymarching */
#define MB_MAX_STEPS_FULL   80
#define MB_MAX_STEPS_FAST   40
#define MB_HIT_EPS          0.0015f
#define MB_MAX_DIST         5.0f
#define MB_MAX_ITER         12      /* DE bailout iterations            */
#define MB_MAX_ITER_AUX     6       /* normal / AO / shadow  (cheaper)  */
#define MB_BAIL             2.0f    /* escape radius                    */

/* Near-miss edge glow */
#define GLOW_THRESH  0.04f          /* DE < this → glow pixel           */

/* Soft shadow */
#define SH_STEPS    24
#define SH_K        8.0f
#define SH_MIN_T    0.02f

/* Ambient occlusion */
#define AO_SAMPLES  5
#define AO_STEP     0.08f
#define AO_DECAY    0.7f

/* Camera defaults */
#define CAM_DIST_DEFAULT   2.8f
#define CAM_DIST_MIN       1.4f
#define CAM_DIST_MAX       6.0f
#define CAM_THETA_DEFAULT  0.35f
#define CAM_PHI_DEFAULT    0.0f
#define CAM_ORBIT_SPD      0.4f
#define CAM_ZOOM_SPD       0.15f
#define CAM_STEP_THETA     0.07f
#define CAM_STEP_PHI       0.10f
#define CAM_STEP_DIST      0.12f
#define CAM_SPD_STEP       1.4f
#define CAM_SPD_MIN        0.05f
#define CAM_SPD_MAX        4.0f
#define FOV_HALF_TAN       0.55f

/* Lighting */
#define LIGHT_X   1.8f
#define LIGHT_Y   2.2f
#define LIGHT_Z   2.0f
/* Rim light direction (unit vector toward scene — normalised below) */
#define RIM_X    -1.0f
#define RIM_Y    -0.5f
#define RIM_Z    -0.8f
#define RIM_STR   0.22f   /* rim contribution scale                   */

/* Phong */
#define KA   0.10f
#define KD   0.72f
#define KS   0.42f
#define SHIN 32.0f

/* Power */
#define MB_POWER_DEFAULT   8
#define MB_POWER_MIN       2
#define MB_POWER_MAX      16

/* Power morph */
#define MORPH_SPD   0.22f   /* rad/sec of sine wave                    */
#define MORPH_LO    2.0f
#define MORPH_HI    8.0f

/* Smooth coloring: colour bands across iteration depth */
#define COLOR_BANDS     2.0f   /* full palette cycles visible at once  */
#define COLOR_PHASE_SPD 0.35f  /* palette rotation speed (pairs/sec)   */

/* Progressive rendering */
#define ROWS_PER_TICK   8    /* rows per frame; 8 = ~2× faster pass than 4  */

/* Starfield */
#define NSTARS  200

/* Trap types */
typedef enum { TRAP_POINT = 0, TRAP_PLANE, TRAP_RING, TRAP_N } TrapType;

/* Color pairs: 1..GRAD_N = gradient, CP_STAR=GRAD_N+1, CP_HUD=GRAD_N+2 */
#define GRAD_N    8
#define CP_STAR  (GRAD_N + 1)
#define CP_HUD   (GRAD_N + 2)

typedef struct {
    const char *name;
    int fg256[GRAD_N];
    int fg8[GRAD_N];
} Theme;

static const Theme k_themes[] = {
    { "Plasma",
      {17, 18, 54, 91, 128, 165, 201, 207},
      {COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA,
       COLOR_RED,  COLOR_RED,  COLOR_YELLOW,  COLOR_WHITE} },
    { "Fire",
      {232, 52, 88, 124, 160, 196, 202, 226},
      {COLOR_BLACK, COLOR_RED,    COLOR_RED,    COLOR_RED,
       COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE} },
    { "Ice",
      {17, 18, 24, 31, 39, 45, 51, 231},
      {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN,
       COLOR_CYAN, COLOR_WHITE,COLOR_WHITE,COLOR_WHITE} },
    { "Gold",
      {52, 94, 136, 178, 214, 220, 226, 231},
      {COLOR_RED,   COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW,
       COLOR_YELLOW,COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE} },
    { "Alien",
      {22, 28, 34, 40, 46, 82, 118, 154},
      {COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
       COLOR_GREEN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE} },
    { "Neon",
      {22, 46, 51, 45, 39, 33, 27, 21},
      {COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN,
       COLOR_CYAN,  COLOR_BLUE,  COLOR_BLUE, COLOR_BLUE} },
    { "Sunset",
      {52, 124, 196, 202, 208, 214, 220, 226},
      {COLOR_RED,   COLOR_RED,    COLOR_RED,   COLOR_YELLOW,
       COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE} },
};
#define THEME_N (int)(sizeof k_themes / sizeof k_themes[0])

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

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
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static int g_theme_idx    = 0;
static int g_color_offset = 0;   /* rotated each tick by scene_tick()  */

static void color_init_theme(void)
{
    start_color();
    use_default_colors();
    const Theme *th = &k_themes[g_theme_idx];
    for (int i = 0; i < GRAD_N; i++) {
        if (COLORS >= 256)
            init_pair(i + 1, th->fg256[i], COLOR_BLACK);
        else
            init_pair(i + 1, th->fg8[i], COLOR_BLACK);
    }
    /* Star pair — dim grey */
    if (COLORS >= 256) init_pair(CP_STAR, 240, -1);
    else               init_pair(CP_STAR, COLOR_WHITE, -1);
    /* HUD pair — bright white */
    if (COLORS >= 256) init_pair(CP_HUD, 255, -1);
    else               init_pair(CP_HUD, COLOR_WHITE, -1);
}

/*
 * mb_color_pair() — map smooth iteration value to an animated color pair.
 *
 * smooth [0,1] represents the fractal's depth layer (escaped early = 0,
 * escaped late = ~1, interior = 1).  We map this to COLOR_BANDS complete
 * cycles of the palette, then rotate by g_color_offset so the whole
 * palette slowly shifts over time.
 *
 * Result: pair 1..GRAD_N.
 */
static int mb_color_pair(float smooth)
{
    float banded = smooth * (float)GRAD_N * COLOR_BANDS;
    int   idx    = (int)banded % GRAD_N;
    if (idx < 0) idx = 0;
    idx = (idx + g_color_offset + GRAD_N * 16) % GRAD_N;
    return idx + 1;
}

/* ===================================================================== */
/* §4  vec3                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3(float x,float y,float z){ return (Vec3){x,y,z}; }
static inline Vec3  v3add(Vec3 a,Vec3 b){ return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline Vec3  v3sub(Vec3 a,Vec3 b){ return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline Vec3  v3mul(Vec3 a,float s){ return v3(a.x*s,a.y*s,a.z*s); }
static inline float v3dot(Vec3 a,Vec3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3len(Vec3 a){ return sqrtf(v3dot(a,a)); }
static inline Vec3  v3norm(Vec3 a){
    float l=v3len(a); return l>1e-7f?v3mul(a,1.0f/l):v3(0,0,1);
}
static inline Vec3  v3cross(Vec3 a,Vec3 b){
    return v3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
static inline float v3absf(float f){ return f<0.0f?-f:f; }

/* ===================================================================== */
/* §5  mandelbulb  (pure math — no ncurses, no global state)             */
/* ===================================================================== */

/*
 * mb_de() — Mandelbulb distance estimator + orbit trap + smooth iter.
 *
 * Spherical power formula:
 *   z^p → r^p · (sin(pθ)cos(pφ), sin(pθ)sin(pφ), cos(pθ))
 *
 * DE:  dr_{n+1} = p · r^(p-1) · dr_n + 1;   DE = 0.5·log(r)·r / dr
 *
 * Smooth coloring:
 *   mu = iter + 1 - log(log(r)/log(BAIL)) / log(power)
 *   Gives continuous value between integer escape iterations, so colour
 *   transitions are smooth across the fractal surface.
 *
 * out_smooth may be NULL (in aux calls: normal, shadow, AO).
 */
static float mb_de(Vec3 pos, float power, int max_iter,
                   float *out_trap, TrapType trap_type,
                   float *out_smooth)
{
    Vec3  z      = pos;
    float dr     = 1.0f;
    float r      = 0.0f;
    float trap   = 1e9f;
    int   esc_i  = max_iter;

    for (int i = 0; i < max_iter; i++) {
        r = v3len(z);

        /* orbit trap */
        float td;
        switch (trap_type) {
        case TRAP_PLANE: td = v3absf(z.z);    break;
        case TRAP_RING:  td = v3absf(r-0.5f); break;
        default:         td = r;              break;
        }
        if (td < trap) trap = td;

        if (r > MB_BAIL) { esc_i = i; break; }

        float theta = acosf(z.z / (r + 1e-8f));
        float phi   = atan2f(z.y, z.x);
        float rp    = powf(r, power);

        dr = rp / r * power * dr + 1.0f;

        float st = sinf(power * theta);
        float ct = cosf(power * theta);
        float sp = sinf(power * phi);
        float cp = cosf(power * phi);

        z = v3add(v3mul(v3(st*cp, st*sp, ct), rp), pos);
    }

    /* smooth iteration count — continuous value replacing integer escape */
    if (out_smooth) {
        if (r > MB_BAIL && esc_i < max_iter) {
            float log_r   = logf(r);
            float log_b   = logf(MB_BAIL);    /* log(2) ≈ 0.693 */
            float log_p   = logf(power + 1e-6f);
            float mu = (float)esc_i + 1.0f - logf(log_r / log_b) / log_p;
            mu /= (float)max_iter;
            *out_smooth = mu < 0.0f ? 0.0f : (mu > 1.0f ? 1.0f : mu);
        } else {
            *out_smooth = 1.0f;   /* interior — no escape */
        }
    }

    if (trap > 1.2f) trap = 1.2f;
    *out_trap = trap / 1.2f;

    if (r < 1e-7f) return 0.0f;
    return 0.5f * logf(r) * r / dr;
}

/*
 * mb_normal() — surface normal via central differences (6 DE calls).
 * Uses MB_MAX_ITER_AUX iterations for speed.
 */
static Vec3 mb_normal(Vec3 p, float power, TrapType trap_type)
{
    const float H = 0.002f;
    float dt, ds;
    float dx = mb_de(v3(p.x+H,p.y,p.z), power, MB_MAX_ITER_AUX, &dt, trap_type, NULL)
             - mb_de(v3(p.x-H,p.y,p.z), power, MB_MAX_ITER_AUX, &ds, trap_type, NULL);
    float dy = mb_de(v3(p.x,p.y+H,p.z), power, MB_MAX_ITER_AUX, &dt, trap_type, NULL)
             - mb_de(v3(p.x,p.y-H,p.z), power, MB_MAX_ITER_AUX, &ds, trap_type, NULL);
    float dz = mb_de(v3(p.x,p.y,p.z+H), power, MB_MAX_ITER_AUX, &dt, trap_type, NULL)
             - mb_de(v3(p.x,p.y,p.z-H), power, MB_MAX_ITER_AUX, &ds, trap_type, NULL);
    return v3norm(v3(dx, dy, dz));
}

/*
 * mb_shadow() — soft shadow [0,1].
 * penumbra = min over march steps of  k · sdf(p) / t
 */
static float mb_shadow(Vec3 ro, Vec3 rd, float power, TrapType trap_type)
{
    float sh = 1.0f;
    float t  = SH_MIN_T;
    float dt;
    for (int i = 0; i < SH_STEPS && t < MB_MAX_DIST; i++) {
        Vec3  p = v3add(ro, v3mul(rd, t));
        float d = mb_de(p, power, MB_MAX_ITER_AUX, &dt, trap_type, NULL);
        if (d < MB_HIT_EPS * 0.5f) return 0.05f;
        float pen = SH_K * d / t;
        if (pen < sh) sh = pen;
        t += d * 0.8f;
    }
    return sh < 0.0f ? 0.0f : (sh > 1.0f ? 1.0f : sh);
}

/*
 * mb_ao() — ambient occlusion [0,1].
 * 5 samples along surface normal.
 */
static float mb_ao(Vec3 p, Vec3 n, float power, TrapType trap_type)
{
    float ao    = 0.0f;
    float scale = 1.0f;
    float dt;
    for (int i = 1; i <= AO_SAMPLES; i++) {
        float delta = (float)i * AO_STEP;
        Vec3  sp    = v3add(p, v3mul(n, delta));
        float d     = mb_de(sp, power, MB_MAX_ITER_AUX, &dt, trap_type, NULL);
        ao   += (delta - d) * scale;
        scale *= AO_DECAY;
    }
    ao = 1.0f - ao * 2.0f;
    return ao < 0.0f ? 0.0f : (ao > 1.0f ? 1.0f : ao);
}

/*
 * mb_march() — main raymarch loop.
 * Tracks minimum DE for near-miss glow detection.
 * On hit: fills *out_trap, *out_smooth, returns t > 0.
 * On miss: returns -1; *out_glow_str in [0,1] if glow threshold crossed.
 */
static float mb_march(Vec3 ro, Vec3 rd, float power, int max_steps,
                       float *out_trap, float *out_smooth, float *out_glow_str,
                       TrapType trap_type)
{
    float t      = 0.02f;
    float min_d  = 1e9f;
    float dt;

    for (int i = 0; i < max_steps && t < MB_MAX_DIST; i++) {
        Vec3  p = v3add(ro, v3mul(rd, t));
        float d = mb_de(p, power, MB_MAX_ITER, &dt, trap_type, NULL);
        if (d < min_d) min_d = d;
        if (d < MB_HIT_EPS) {
            /* hit — recompute with full output for colour data */
            mb_de(p, power, MB_MAX_ITER, out_trap, trap_type, out_smooth);
            *out_glow_str = 0.0f;
            return t;
        }
        t += d * 0.85f;
    }

    /* miss — check for near-miss glow */
    *out_glow_str = (min_d < GLOW_THRESH)
                  ? (1.0f - min_d / GLOW_THRESH)
                  : 0.0f;
    *out_trap   = 1.0f;
    *out_smooth = 0.1f;   /* glow uses a vivid colour, not interior dark */
    return -1.0f;
}

/* ===================================================================== */
/* §6  canvas                                                             */
/* ===================================================================== */

/*
 * PixCell — per-pixel data stored in g_fbuf.
 *   luma     [0,1] — Phong brightness (key + rim + AO + shadow)
 *   smooth   [0,1] — smooth iteration count → drives colour pair
 *   trap     [0,1] — orbit trap value (modulates bold/dim)
 *   hit            — ray hit the fractal surface
 *   glow_str [0,1] — near-miss intensity (>0 means edge glow)
 */
typedef struct {
    float luma;
    float smooth;
    float trap;
    bool  hit;
    float glow_str;
} PixCell;

static PixCell g_fbuf[CANVAS_MAX_H][CANVAS_MAX_W];   /* render-in-progress  */
static PixCell g_stable[CANVAS_MAX_H][CANVAS_MAX_W]; /* last complete frame  */

/* Progressive render state */
static int  g_render_row = 0;
static bool g_dirty      = true;

/*
 * Per-pass camera snapshot — captured once when g_render_row wraps to 0.
 * All rows within one pass use the same camera so the fractal has no
 * horizontal shear from the orbit moving between rows.
 */
static Vec3 g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up;

/*
 * Character ramp — 14 levels (space = background, @ = max brightness).
 * Hit pixels always show at least '.' so shadowed regions remain visible.
 */
static const char k_ramp[] = " .`,:;+*=xX#$@";
#define RAMP_N (int)(sizeof k_ramp - 1)

/* Starfield — initialised once with a fixed seed */
static struct { int x, y; int bri; } g_stars[NSTARS];

static void stars_init(int cw, int ch)
{
    unsigned rng = 0xDEADBEEFu;
    for (int i = 0; i < NSTARS; i++) {
        rng = rng * 1664525u + 1013904223u;
        g_stars[i].x = (int)(rng % (unsigned)(cw > 0 ? cw : 1));
        rng = rng * 1664525u + 1013904223u;
        g_stars[i].y = (int)(rng % (unsigned)(ch > 0 ? ch : 1));
        rng = rng * 1664525u + 1013904223u;
        g_stars[i].bri = (int)(rng % 3u);   /* 0=faint 1=medium 2=bright */
    }
}

/*
 * mb_cast_pixel() — cast one ray; returns filled PixCell.
 *
 * Two-light Phong:
 *   Key light:  front-right-above  → full diffuse + specular + shadow
 *   Rim light:  back-left-below    → dim diffuse only, no shadow
 *
 * Aspect correction follows raymarcher.c §4b:
 *   phys_aspect = (canvas_h * CELL_ASPECT) / canvas_w
 *   rd = normalise(u·FOV·right + v·FOV·phys_aspect·up + fwd)
 */
static PixCell mb_cast_pixel(int px, int py, int cw, int ch,
                              Vec3 cam_pos, Vec3 fwd, Vec3 right, Vec3 up,
                              float power, int max_steps,
                              bool do_shadow, bool do_ao,
                              TrapType trap_type)
{
    float u =  ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
    float v = -((float)py + 0.5f) / (float)ch * 2.0f + 1.0f;
    float phys_aspect = ((float)ch * CELL_ASPECT) / (float)cw;

    Vec3 rd = v3norm(
        v3add(v3add(
            v3mul(right, u * FOV_HALF_TAN),
            v3mul(up,    v * FOV_HALF_TAN * phys_aspect)),
            fwd));

    float trap, smooth, glow_str;
    float t = mb_march(cam_pos, rd, power, max_steps,
                       &trap, &smooth, &glow_str, trap_type);

    PixCell cell = {0.0f, smooth, trap, false, glow_str};

    if (t < 0.0f) return cell;   /* miss (glow_str may be > 0) */

    cell.hit    = true;
    cell.smooth = smooth;
    cell.trap   = trap;

    Vec3 hit = v3add(cam_pos, v3mul(rd, t));
    Vec3 N   = mb_normal(hit, power, trap_type);
    Vec3 V   = v3norm(v3sub(cam_pos, hit));

    /* Key light */
    Vec3  L1   = v3norm(v3sub(v3(LIGHT_X, LIGHT_Y, LIGHT_Z), hit));
    float ndl1 = fmaxf(0.0f, v3dot(N, L1));
    Vec3  R1   = v3sub(v3mul(N, 2.0f * ndl1), L1);
    float spec = powf(fmaxf(0.0f, v3dot(R1, V)), SHIN);

    float sh = 1.0f;
    if (do_shadow) {
        Vec3 sro = v3add(hit, v3mul(N, 0.006f));
        sh = mb_shadow(sro, L1, power, trap_type);
    }

    /* Rim / fill light — opposite side, no specular, no shadow */
    Vec3  L2   = v3norm(v3(RIM_X, RIM_Y, RIM_Z));
    float ndl2 = fmaxf(0.0f, v3dot(N, L2));

    float ao = 1.0f;
    if (do_ao) ao = mb_ao(hit, N, power, trap_type);

    float luma = KA * ao
               + (KD * ndl1 + KS * spec) * sh * ao
               + RIM_STR * KD * ndl2 * ao;
    cell.luma = luma > 1.0f ? 1.0f : luma;
    return cell;
}

/*
 * canvas_render_rows() — render ROWS_PER_TICK rows into g_fbuf.
 * Progressive: wraps at ch; returns true on full-frame completion.
 */
static bool canvas_render_rows(int cw, int ch,
                                Vec3 cam_pos, Vec3 fwd, Vec3 right, Vec3 up,
                                float power, int max_steps,
                                bool do_shadow, bool do_ao,
                                TrapType trap_type)
{
    for (int k = 0; k < ROWS_PER_TICK; k++) {
        int py = g_render_row;
        for (int px = 0; px < cw; px++) {
            g_fbuf[py][px] = mb_cast_pixel(px, py, cw, ch,
                                            cam_pos, fwd, right, up,
                                            power, max_steps,
                                            do_shadow, do_ao, trap_type);
        }
        if (++g_render_row >= ch) {
            g_render_row = 0;
            /* pass complete — save as stable frame shown while next pass renders */
            memcpy(g_stable, g_fbuf, sizeof g_stable);
            return true;
        }
    }
    return false;
}

/*
 * canvas_draw() — starfield then fractal onto terminal.
 *
 * Drawing order:
 *   1. Stars in background (fractal pixels overwrite them)
 *   2. Glow pixels (near-miss corona)
 *   3. Hit pixels  (fractal surface, smooth-iter colour)
 *
 * Colour split:
 *   colour pair ← smooth iteration count (depth layer + palette rotation)
 *   char        ← Phong luma (surface lighting)
 *   bold        ← close orbit trap (trap < 0.35) or near max brightness
 */
static void canvas_draw(int cw, int ch, int cols, int rows,
                         int color_offset_unused)
{
    (void)color_offset_unused;  /* g_color_offset is global — used in mb_color_pair */

    int off_x = (cols - cw * CELL_W) / 2;
    int off_y = (rows - ch * CELL_H) / 2;

    /* ── 1. Starfield ─────────────────────────────────────────────── */
    static const char star_ch[] = ".`:";
    for (int i = 0; i < NSTARS; i++) {
        int sx = g_stars[i].x, sy = g_stars[i].y;
        if (sx >= cw || sy >= ch) continue;
        if (g_fbuf[sy][sx].hit || g_fbuf[sy][sx].glow_str > 0.02f) continue;
        int tx = off_x + sx * CELL_W;
        int ty = off_y + sy * CELL_H;
        if (tx < 0 || tx >= cols || ty < 0 || ty >= rows) continue;
        attr_t sa = COLOR_PAIR(CP_STAR);
        if (g_stars[i].bri == 2) sa |= A_BOLD;
        attron(sa);
        mvaddch(ty, tx, (chtype)(unsigned char)star_ch[g_stars[i].bri]);
        attroff(sa);
    }

    /* ── 2 & 3. Fractal pixels ────────────────────────────────────── */
    for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
        for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
            /*
             * Rows already rendered this pass come from g_fbuf (fresh data).
             * Rows not yet rendered come from g_stable (last complete frame).
             * Effect: top portion shows new data, bottom shows previous frame —
             * no black half, no hard scan-line, animation feels continuous.
             */
            PixCell *cell = (vy < g_render_row)
                          ? &g_fbuf[vy][vx]
                          : &g_stable[vy][vx];

            char   ch_c;
            attr_t attr;

            if (!cell->hit) {
                if (cell->glow_str < 0.02f) continue;   /* pure background */

                /* Near-miss glow corona */
                float gs = cell->glow_str;
                int   pair = mb_color_pair(cell->smooth);
                attr  = COLOR_PAIR(pair) | A_BOLD;
                if      (gs > 0.80f) ch_c = '+';
                else if (gs > 0.50f) ch_c = ':';
                else if (gs > 0.25f) ch_c = ',';
                else                 ch_c = '.';

            } else {
                /* Hit pixel: char from luma, colour from smooth iter */
                int ri = (int)(cell->luma * (float)(RAMP_N - 1) + 0.5f);
                if (ri < 0)        ri = 0;
                if (ri >= RAMP_N)  ri = RAMP_N - 1;
                if (ri == 0)       ri = 1;   /* always show at least '.' */
                ch_c = k_ramp[ri];

                int  pair = mb_color_pair(cell->smooth);
                attr = COLOR_PAIR(pair);
                /* close trap or bright surface → bold for extra pop */
                if (cell->trap < 0.35f || ri >= RAMP_N - 2) attr |= A_BOLD;
            }

            for (int by = 0; by < CELL_H; by++) {
                for (int bx = 0; bx < CELL_W; bx++) {
                    int tx = off_x + vx * CELL_W + bx;
                    int ty = off_y + vy * CELL_H + by;
                    if (tx < 0 || tx >= cols) continue;
                    if (ty < 0 || ty >= rows) continue;
                    attron(attr);
                    mvaddch(ty, tx, (chtype)(unsigned char)ch_c);
                    attroff(attr);
                }
            }
        }
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    int      cw, ch;

    float    power;
    TrapType trap_type;
    int      max_steps;

    float    cam_dist;
    float    cam_theta;
    float    cam_phi;
    float    orbit_spd;

    bool     paused;
    bool     auto_zoom;
    bool     do_shadow;
    bool     do_ao;
    bool     fast_mode;

    /* palette animation */
    float    color_phase;   /* accumulator → g_color_offset             */

    /* power morph */
    bool     morph_mode;
    float    morph_phase;   /* drives sine wave for power oscillation   */

    float    time;
} Scene;

static void scene_camera(const Scene *s,
                          Vec3 *cam_pos, Vec3 *fwd, Vec3 *right, Vec3 *up)
{
    float ct = cosf(s->cam_theta);
    float st = sinf(s->cam_theta);
    float cp = cosf(s->cam_phi);
    float sp = sinf(s->cam_phi);

    *cam_pos  = v3mul(v3(ct * cp, st, ct * sp), s->cam_dist);
    *fwd      = v3norm(v3mul(*cam_pos, -1.0f));
    Vec3 wup  = v3(0, 1, 0);
    if (v3absf(v3dot(*fwd, wup)) > 0.99f) wup = v3(0, 0, 1);
    *right    = v3norm(v3cross(*fwd, wup));
    *up       = v3cross(*right, *fwd);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cw         = canvas_w_from_cols(cols);
    s->ch         = canvas_h_from_rows(rows);
    if (s->cw > CANVAS_MAX_W) s->cw = CANVAS_MAX_W;
    if (s->ch > CANVAS_MAX_H) s->ch = CANVAS_MAX_H;
    s->power      = (float)MB_POWER_DEFAULT;
    s->trap_type  = TRAP_POINT;
    s->max_steps  = MB_MAX_STEPS_FULL;
    s->cam_dist   = CAM_DIST_DEFAULT;
    s->cam_theta  = CAM_THETA_DEFAULT;
    s->cam_phi    = CAM_PHI_DEFAULT;
    s->orbit_spd  = CAM_ORBIT_SPD;
    s->do_shadow  = true;
    s->do_ao      = true;
    memset(g_fbuf, 0, sizeof g_fbuf);
    stars_init(s->cw, s->ch);
    g_render_row  = 0;
    g_dirty       = true;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cw = canvas_w_from_cols(cols);
    s->ch = canvas_h_from_rows(rows);
    if (s->cw > CANVAS_MAX_W) s->cw = CANVAS_MAX_W;
    if (s->ch > CANVAS_MAX_H) s->ch = CANVAS_MAX_H;
    stars_init(s->cw, s->ch);
    g_render_row = 0;
    g_dirty      = true;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;

    /* auto-orbit */
    s->cam_phi += s->orbit_spd * dt;

    /* auto-zoom fly-in */
    if (s->auto_zoom) {
        s->cam_dist -= CAM_ZOOM_SPD * dt;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_DEFAULT;
    }

    /* palette animation — rotate colour offset */
    s->color_phase += COLOR_PHASE_SPD * dt;
    g_color_offset  = (int)(s->color_phase) % GRAD_N;

    /* power morph — sine oscillation between MORPH_LO and MORPH_HI */
    if (s->morph_mode) {
        s->morph_phase += MORPH_SPD * dt;
        float new_p = MORPH_LO
                    + (MORPH_HI - MORPH_LO)
                    * 0.5f * (1.0f + sinf(s->morph_phase));
        if (fabsf(new_p - s->power) > 0.06f) {
            s->power = new_p;
            /* do NOT set g_dirty — let progressive render sweep naturally;
               the top-to-bottom scan during morphing looks like a radar sweep */
        }
    }
}

static void scene_render(Scene *s)
{
    /*
     * Dirty: an explicit parameter change (key press, resize) — restart
     * the pass immediately so the user sees the new state from row 0.
     * The stable buffer fills the bottom until the new pass catches up.
     */
    if (g_dirty) {
        g_render_row = 0;
        memset(g_fbuf, 0, sizeof(PixCell) * (size_t)s->ch * CANVAS_MAX_W);
        g_dirty = false;
    }

    /*
     * Camera snapshot: capture the camera basis ONCE at the start of each
     * pass (g_render_row == 0) and reuse it for every row in that pass.
     *
     * Without this, auto-orbit moves cam_phi every tick, so rows rendered
     * at different ticks within one pass have different cameras → horizontal
     * shear bands. With the snapshot every row in a pass shares one camera
     * and the complete frame is geometrically consistent.
     */
    if (g_render_row == 0)
        scene_camera(s, &g_snap_cam, &g_snap_fwd, &g_snap_right, &g_snap_up);

    canvas_render_rows(s->cw, s->ch,
                       g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up,
                       s->power, s->max_steps,
                       s->do_shadow, s->do_ao,
                       s->trap_type);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    canvas_draw(s->cw, s->ch, cols, rows, g_color_offset);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init_theme();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static const char *trap_name(TrapType t)
{
    switch (t) {
    case TRAP_POINT: return "Point";
    case TRAP_PLANE: return "Plane";
    case TRAP_RING:  return "Ring ";
    default:         return "?    ";
    }
}

static void screen_draw(Screen *sc, const Scene *s, double fps)
{
    erase();
    int draw_rows = sc->rows - HUD_ROWS;
    if (draw_rows < 1) draw_rows = 1;
    scene_draw(s, sc->cols, draw_rows);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(sc->rows - 3, 0,
             "Mandelbulb Explorer  %.1f fps  [%dx%d]  row %d/%d",
             fps, s->cw, s->ch, g_render_row, s->ch);
    mvprintw(sc->rows - 2, 0,
             "power:%.1f  trap:%-5s  theme:%-6s  dist:%.2f  orb:%.2f"
             "  %s%s%s%s%s",
             s->power, trap_name(s->trap_type),
             k_themes[g_theme_idx].name,
             s->cam_dist, s->orbit_spd,
             s->do_shadow ? "SH " : "   ",
             s->do_ao     ? "AO " : "   ",
             s->fast_mode ? "FAST "  : "     ",
             s->auto_zoom ? "ZOOM "  : "     ",
             s->morph_mode? "MORPH"  : "     ");
    mvprintw(sc->rows - 1, 0,
             "q:quit spc:pause arrows:orbit p/P:power m:morph "
             "t:trap c:theme z/Z:dist a:zoom o:AO s:shad f:fast r:reset []:spd");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize(&app->scene,
                 app->screen.cols,
                 app->screen.rows - HUD_ROWS);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    bool   dirty = false;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':  s->paused = !s->paused;                          break;

    case KEY_LEFT:
        s->cam_phi   -= CAM_STEP_PHI;  dirty = true;           break;
    case KEY_RIGHT:
        s->cam_phi   += CAM_STEP_PHI;  dirty = true;           break;
    case KEY_UP:
        s->cam_theta += CAM_STEP_THETA;
        if (s->cam_theta >  1.5f) s->cam_theta =  1.5f;
        dirty = true;                                           break;
    case KEY_DOWN:
        s->cam_theta -= CAM_STEP_THETA;
        if (s->cam_theta < -1.5f) s->cam_theta = -1.5f;
        dirty = true;                                           break;

    case 'p':
        if (!s->morph_mode) {
            s->power += 1.0f;
            if (s->power > (float)MB_POWER_MAX) s->power = (float)MB_POWER_MAX;
            dirty = true;
        }
        break;
    case 'P':
        if (!s->morph_mode) {
            s->power -= 1.0f;
            if (s->power < (float)MB_POWER_MIN) s->power = (float)MB_POWER_MIN;
            dirty = true;
        }
        break;

    case 'm':
        s->morph_mode  = !s->morph_mode;
        s->morph_phase = 0.0f;   /* restart oscillation */
        break;

    case 't':
        s->trap_type = (TrapType)((s->trap_type + 1) % TRAP_N);
        dirty = true;                                           break;

    case 'c':
        g_theme_idx = (g_theme_idx + 1) % THEME_N;
        color_init_theme();                                     break;

    case 'z':
        s->cam_dist -= CAM_STEP_DIST;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        dirty = true;                                           break;
    case 'Z':
        s->cam_dist += CAM_STEP_DIST;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        dirty = true;                                           break;

    case 'a':  s->auto_zoom = !s->auto_zoom;                   break;
    case 'f':
        s->fast_mode = !s->fast_mode;
        s->max_steps = s->fast_mode ? MB_MAX_STEPS_FAST : MB_MAX_STEPS_FULL;
        dirty = true;                                           break;
    case 'o':  s->do_ao     = !s->do_ao;     dirty = true;     break;
    case 's':  s->do_shadow = !s->do_shadow; dirty = true;     break;

    case '[':
        s->orbit_spd /= CAM_SPD_STEP;
        if (s->orbit_spd < CAM_SPD_MIN) s->orbit_spd = CAM_SPD_MIN;
        break;
    case ']':
        s->orbit_spd *= CAM_SPD_STEP;
        if (s->orbit_spd > CAM_SPD_MAX) s->orbit_spd = CAM_SPD_MAX;
        break;

    case 'r':
        s->cam_dist  = CAM_DIST_DEFAULT;
        s->cam_theta = CAM_THETA_DEFAULT;
        s->cam_phi   = CAM_PHI_DEFAULT;
        s->auto_zoom = false;
        dirty = true;                                           break;

    default: break;
    }

    if (dirty) g_dirty = true;
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene,
               app->screen.cols,
               app->screen.rows - HUD_ROWS);

    int64_t frame_time  = clock_ns();
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    const int64_t tick_ns = NS_PER_SEC / SIM_FPS;
    const float   dt_sec  = 1.0f / (float)SIM_FPS;

    while (app->running) {

        /* ── resize ────────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        /* ── dt ────────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;

        /* ── sim ───────────────────────────────────────────────────── */
        scene_tick(&app->scene, dt_sec);

        /* ── progressive render ────────────────────────────────────── */
        scene_render(&app->scene);

        /* ── fps ───────────────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap ─────────────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(tick_ns - elapsed);

        /* ── draw + present ────────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        /* ── input ─────────────────────────────────────────────────── */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
