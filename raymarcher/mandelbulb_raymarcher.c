/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb_raymarcher.c — Mandelbulb depth-hue ASCII renderer
 *
 * No lighting of any kind — no Phong, no shadow, no AO, no surface normals.
 * Depth is encoded entirely through colour and character density:
 *
 *   smooth iteration count → hue pair  (30-step rainbow, 1–5 cycles)
 *   smooth iteration count → char density  (sparse outer → dense inner)
 *   orbit trap proximity   → A_BOLD decoration (structural geometry)
 *
 * With 3 colour-band cycles across the depth range, each concentric shell
 * of the Mandelbulb shows a distinct rainbow hue.  The self-similar pod
 * structure reads immediately as layered depth — no illumination needed.
 *
 * Keys:
 *   q / ESC     quit
 *   space       pause / resume auto-orbit
 *   arrow keys  orbit camera  (left/right = phi,  up/down = theta)
 *   p / P       power  +1 / -1   (2..16, default 8)
 *   m           power morph mode  (oscillates 2 ↔ 8)
 *   t           cycle orbit trap  (point / plane / ring)
 *   c           cycle colour bands  (1..5 rainbow cycles per depth range)
 *   n           clean mode toggle  (char from orbit trap, not smooth iter)
 *   z / Z       zoom in / out
 *   a           toggle auto-zoom fly-in
 *   f           toggle fast mode  (fewer march steps)
 *   r           reset camera
 *   [ / ]       decrease / increase orbit speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/mandelbulb_raymarcher.c \
 *       -o mandelbulb_hue -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — every tunable constant + §T technique reference
 *   §2  clock       — monotonic ns clock + sleep
 *   §3  color       — 30-step hue wheel, animated phase rotation
 *   §4  vec3        — inline 3-D vector math
 *   §5  mandelbulb  — DE + smooth coloring + march + glow  (no lighting)
 *   §6  canvas      — PixCell, starfield, render, draw
 *   §7  scene       — state, tick (morph, hue), render, draw
 *   §8  screen      — ncurses wrapper + HUD
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
/*
 * §T  Terminal Rendering Techniques  (reference — see application sites)
 * -----------------------------------------------------------------------
 * A terminal cell is a large, coarse pixel: ~8 visible brightness levels,
 * 2:1 aspect ratio, no anti-aliasing, ~30 fps budget.
 *
 * T.1  SURFACE SMOOTHING           constant: MB_HIT_EPS = 0.003
 *      Raising HIT_EPS merges micro-bumps smaller than one terminal cell
 *      into the nearest visible surface patch — smooth silhouette, no
 *      speckling.
 *
 * T.10 STABLE BUFFER               buffers: g_fbuf + g_stable
 *      Progressive rendering fills g_fbuf row-by-row.  g_stable holds the
 *      last complete frame.  canvas_draw blends fresh rows (g_fbuf) with
 *      the previous frame (g_stable) so there is never a black scan band.
 *
 * T.11 CAMERA SNAPSHOT             state: g_snap_*  (in scene_render)
 *      Auto-orbit increments cam_phi every tick.  Capturing the camera
 *      basis once at row 0 and reusing it for every row in a pass gives
 *      one consistent view — no horizontal geometry shear.
 *
 * T.12 CLEAN MODE                  toggle: 'n' key  (Scene.clean_mode)
 *      Normal mode: smooth → char density (depth encoded in both char and
 *      colour — maximum structural legibility).
 *      Clean mode:  orbit trap proximity → char density (shows attractor
 *      geometry as ASCII texture; colour still encodes depth via hue).
 *
 * T.13 NEAR-MISS GLOW              constant: GLOW_THRESH = 0.04
 *      Track min_d during the march.  min_d < GLOW_THRESH → draw a vivid
 *      char at glow intensity around the silhouette edge (luminous corona).
 *
 * T.14 ASPECT CORRECTION           constant: CELL_ASPECT = 2.0
 *      Terminal cells are ~2× taller than wide.  phys_aspect = (ch·2)/cw
 *      scales the vertical FOV so the Mandelbulb is spherical, not oblate.
 *
 * T.H  HUE DEPTH ENCODING          constant: HUE_N = 30
 *      smooth [0,1] maps to a 30-step xterm-256 hue wheel (full saturation
 *      rainbow: red → orange → yellow → green → cyan → blue → magenta →
 *      red).  COLOR_BANDS (1..5) sets how many complete rainbow cycles span
 *      the full depth range — more cycles = finer depth resolution in hue.
 *      g_hue_offset rotates the entire wheel slowly over time so the palette
 *      drifts without any camera or geometry change.  Both char density
 *      (T.9) and hue (T.H) encode the same smooth value; the double
 *      encoding makes the fractal shell structure readable at any zoom.
 */

enum {
    SIM_FPS       = 30,
    FPS_UPDATE_MS = 500,
    HUD_ROWS      = 3,
};

/* Terminal cell geometry — CELL_W/H set how many terminal characters one
   "physics pixel" occupies.  1×1 = native resolution, finest detail. */
#define CELL_W       1
#define CELL_H       1
#define CELL_ASPECT  2.0f   /* terminals are ~2× taller than wide (T.14) */

static inline int canvas_w_from_cols(int c) { return c / CELL_W; }
static inline int canvas_h_from_rows(int r) { return r / CELL_H; }

#define CANVAS_MAX_W  400
#define CANVAS_MAX_H  120

/* ── Mandelbulb math ──────────────────────────────────────────────────── */
#define MB_HIT_EPS          0.003f   /* T.1  surface smoothing: merge sub-cell bumps */
#define MB_BAIL             8.0f     /* escape radius: r > BAIL → escaped             */
#define MB_MAX_ITER         64       /* full-quality DE iterations                    */
#define MB_MAX_DIST         12.0f    /* march terminates beyond this distance         */
#define MB_MAX_STEPS_FULL   128      /* march steps — full quality                    */
#define MB_MAX_STEPS_FAST    60      /* march steps — fast mode                       */
#define MB_POWER_DEFAULT     8
#define MB_POWER_MIN         2
#define MB_POWER_MAX        16

/* ── Camera ───────────────────────────────────────────────────────────── */
#define FOV_HALF_TAN    0.5774f   /* tan(30°) ≈ 60° vertical FOV               */
#define CAM_DIST_DEFAULT  2.6f
#define CAM_DIST_MIN      1.2f
#define CAM_DIST_MAX      6.0f
#define CAM_THETA_DEFAULT 0.45f
#define CAM_PHI_DEFAULT   0.0f
#define CAM_ORBIT_SPD     0.25f   /* rad/s auto-orbit speed                    */
#define CAM_STEP_PHI      0.12f
#define CAM_STEP_THETA    0.10f
#define CAM_STEP_DIST     0.15f
#define CAM_ZOOM_SPD      0.08f

/* ── Glow (T.13) ──────────────────────────────────────────────────────── */
#define GLOW_THRESH   0.04f   /* near-miss glow: min_d below this → corona  */

/* ── Animation ────────────────────────────────────────────────────────── */
#define MORPH_SPD       0.22f   /* power morph oscillation speed (rad/s)    */
#define MORPH_LO        2.0f
#define MORPH_HI        8.0f
#define HUE_PHASE_SPD   0.25f   /* hue wheel rotation speed (pairs/sec)     */
#define ROWS_PER_TICK   8       /* T.10/T.11 progressive rows per frame tick */
#define NSTARS          200

/* ── Color (T.H) ──────────────────────────────────────────────────────── */
/* 30 hue steps around the full xterm-256 rainbow (T.H).
 * Each index is one step along the hue wheel at full saturation.
 * Pairs are 1-indexed: pair i+1 corresponds to k_hue256[i].            */
#define HUE_N       30
#define CP_STAR    (HUE_N + 1)
#define CP_HUD     (HUE_N + 2)

#define COLOR_BANDS_DEFAULT  3
#define COLOR_BANDS_MAX      5

/* Trap types */
typedef enum { TRAP_POINT = 0, TRAP_PLANE, TRAP_RING, TRAP_N } TrapType;

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

/*
 * 30-step xterm-256 hue wheel at full saturation (T.H).
 * Formula: index = 16 + 36·r + 6·g + b,  components r,g,b ∈ 0..5.
 *
 * The sequence walks around the hue wheel:
 *   red (196) → orange → yellow (226) → chartreuse → green (46)
 *   → spring-green → cyan (51) → azure → blue (21)
 *   → indigo → magenta (201) → rose → back toward red
 *
 * At COLOR_BANDS=3, smooth [0,1] cycles through this rainbow 3 times,
 * giving ~10 distinct pairs per cycle → fine-grained depth reading.
 */
static const int k_hue256[HUE_N] = {
    196, 202, 208, 214, 220, 226,   /* red → orange → yellow      (0 –5) */
    190, 154, 118,  82,  46,        /* yellow-green → lime → green (6–10) */
     47,  48,  49,  50,  51,        /* spring-green → cyan        (11–15) */
     45,  39,  33,  27,  21,        /* azure → sky-blue → blue    (16–20) */
     57,  93, 129, 165, 201,        /* indigo → violet → magenta  (21–25) */
    200, 199, 198, 197              /* rose → back toward red     (26–29) */
};

/* 8-color fallback: map 30 hue steps to the 6 ANSI hue families */
static const int k_hue8[HUE_N] = {
    COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,
    COLOR_YELLOW,  COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,
    COLOR_GREEN,   COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,
    COLOR_CYAN,    COLOR_BLUE,    COLOR_BLUE,    COLOR_BLUE,    COLOR_BLUE,
    COLOR_BLUE,    COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
    COLOR_MAGENTA, COLOR_RED,     COLOR_RED,     COLOR_RED
};

/* Global hue offset — incremented each tick for palette drift animation */
static int g_hue_offset = 0;

static void color_init(void)
{
    start_color();
    use_default_colors();
    /* Register all 30 hue pairs onto the xterm-256 hue wheel (T.H) */
    for (int i = 0; i < HUE_N; i++) {
        if (COLORS >= 256)
            init_pair(i + 1, k_hue256[i], COLOR_BLACK);
        else
            init_pair(i + 1, k_hue8[i], COLOR_BLACK);
    }
    /* Dim grey for stars — does not interfere with fractal hues */
    if (COLORS >= 256) init_pair(CP_STAR, 240, -1);
    else               init_pair(CP_STAR, COLOR_WHITE, -1);
    /* Bright white for HUD text */
    if (COLORS >= 256) init_pair(CP_HUD, 255, -1);
    else               init_pair(CP_HUD, COLOR_WHITE, -1);
}

/*
 * hue_color_pair() — map smooth iteration value to a hue pair (T.H).
 *
 * smooth [0,1]:  0 = outermost shell (escaped early),
 *                1 = innermost accessible shell (escaped late).
 *
 * Multiplying by HUE_N·bands then taking modulo gives 'bands' complete
 * rainbow cycles across the full depth range.  g_hue_offset rotates the
 * entire wheel slowly so the palette drifts over time without any scene
 * change — creates a slow "colour breathing" animation.
 *
 * Returns a pair index in 1..HUE_N.
 */
static int hue_color_pair(float smooth, int bands)
{
    float f   = smooth * (float)HUE_N * (float)bands;
    int   idx = (int)f % HUE_N;
    if (idx < 0) idx = 0;
    idx = (idx + g_hue_offset + HUE_N * 16) % HUE_N;
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
    float l = v3len(a); return l > 1e-7f ? v3mul(a, 1.0f/l) : v3(0,0,1);
}
static inline Vec3  v3cross(Vec3 a,Vec3 b){
    return v3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
static inline float v3absf(float f){ return f < 0.0f ? -f : f; }

/* ===================================================================== */
/* §5  mandelbulb  (pure math — no lighting, no surface normals)         */
/* ===================================================================== */

/*
 * mb_de() — Mandelbulb distance estimator + orbit trap + smooth coloring.
 *
 * Spherical power formula:
 *   z^p → r^p · (sin(pθ)cos(pφ), sin(pθ)sin(pφ), cos(pθ))
 *
 * Distance estimate:
 *   dr_{n+1} = p · r^(p-1) · dr_n + 1;   DE = 0.5·log(r)·r / dr
 *
 * Smooth coloring (continuous escape count):
 *   mu = iter + 1 − log(log(r)/log(BAIL)) / log(power)
 *   Normalised to [0,1] by dividing by max_iter.  Eliminates harsh
 *   integer-iteration band boundaries from the hue mapping.
 *
 * out_smooth may be NULL (inside mb_march loop: only the DE value needed).
 */
static float mb_de(Vec3 pos, float power, int max_iter,
                   float *out_trap, TrapType trap_type,
                   float *out_smooth)
{
    Vec3  z     = pos;
    float dr    = 1.0f;
    float r     = 0.0f;
    float trap  = 1e9f;
    int   esc_i = max_iter;

    for (int i = 0; i < max_iter; i++) {
        r = v3len(z);

        /* orbit trap: record closest approach to trap geometry */
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

    /* smooth coloring: continuous value replacing integer escape count */
    if (out_smooth) {
        if (r > MB_BAIL && esc_i < max_iter) {
            float log_r = logf(r);
            float log_b = logf(MB_BAIL);
            float log_p = logf(power + 1e-6f);
            float mu = (float)esc_i + 1.0f - logf(log_r / log_b) / log_p;
            mu /= (float)max_iter;
            *out_smooth = mu < 0.0f ? 0.0f : (mu > 1.0f ? 1.0f : mu);
        } else {
            *out_smooth = 1.0f;   /* interior point — deepest layer */
        }
    }

    if (trap > 1.2f) trap = 1.2f;
    *out_trap = trap / 1.2f;

    if (r < 1e-7f) return 0.0f;
    return 0.5f * logf(r) * r / dr;
}

/*
 * mb_march() — sphere-marching loop.
 *
 * Advances t along the ray using the DE as a safe step size.  Tracks
 * min_d (minimum DE seen) for near-miss glow (T.13).
 *
 * On hit (d < MB_HIT_EPS): fills *out_trap, *out_smooth, returns t > 0.
 * On miss: *out_glow_str > 0 if the ray came within GLOW_THRESH; returns -1.
 */
static float mb_march(Vec3 ro, Vec3 rd, float power, int max_steps,
                       float *out_trap, float *out_smooth, float *out_glow_str,
                       TrapType trap_type)
{
    float t     = 0.02f;
    float min_d = 1e9f;
    float dt;

    for (int i = 0; i < max_steps && t < MB_MAX_DIST; i++) {
        Vec3  p = v3add(ro, v3mul(rd, t));
        float d = mb_de(p, power, MB_MAX_ITER, &dt, trap_type, NULL);
        if (d < min_d) min_d = d;
        if (d < MB_HIT_EPS) {
            /* hit: recompute with full smooth/trap output for colour data */
            mb_de(p, power, MB_MAX_ITER, out_trap, trap_type, out_smooth);
            *out_glow_str = 0.0f;
            return t;
        }
        t += d * 0.85f;
    }

    /* miss: check near-miss glow (T.13) */
    *out_glow_str = (min_d < GLOW_THRESH)
                  ? (1.0f - min_d / GLOW_THRESH)
                  : 0.0f;
    *out_trap   = 1.0f;
    *out_smooth = 0.1f;   /* glow takes a vivid colour from the outer hue band */
    return -1.0f;
}

/* ===================================================================== */
/* §6  canvas                                                             */
/* ===================================================================== */

/*
 * PixCell — one rendered cell stored in g_fbuf / g_stable.
 *
 *   smooth    smooth escape count [0,1] — the sole depth signal.
 *             Maps to hue pair (T.H): each concentric fractal shell gets
 *             a distinct colour from the 30-step rainbow.  Character is
 *             always '.'; colour does all the depth work.
 *
 *   trap      orbit trap proximity [0,1]; 0 = closest to trap geometry.
 *             Stored for potential future use (currently unused in draw).
 *             so attractor structure appears as ASCII texture.
 *
 *   hit       true if the ray reached the fractal surface.
 *
 *   glow_str  near-miss glow intensity (T.13).  > 0 → draw corona char.
 */
typedef struct {
    float smooth;
    float trap;
    bool  hit;
    float glow_str;
} PixCell;

static PixCell g_fbuf[CANVAS_MAX_H][CANVAS_MAX_W];   /* render-in-progress  */
static PixCell g_stable[CANVAS_MAX_H][CANVAS_MAX_W]; /* last complete frame  */

static int  g_render_row = 0;
static bool g_dirty      = true;

/* T.11 camera snapshot — one consistent basis per full render pass */
static Vec3 g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up;

/* Starfield — initialised once, fixed layout */
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
        g_stars[i].bri = (int)(rng % 3u);
    }
}

/*
 * mb_cast_pixel() — cast one ray; return a PixCell with depth data only.
 *
 * No lighting computation at all: the march gives smooth + trap + glow,
 * and those three values are everything canvas_draw needs.  Surface
 * normals, shadow rays, and AO samples are not computed.
 */
static PixCell mb_cast_pixel(int px, int py, int cw, int ch,
                              Vec3 cam_pos, Vec3 fwd, Vec3 right, Vec3 up,
                              float power, int max_steps,
                              TrapType trap_type)
{
    float u =  ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
    float v = -((float)py + 0.5f) / (float)ch * 2.0f + 1.0f;

    /* T.14 aspect correction: scale vertical FOV by the physical cell ratio */
    float phys_aspect = ((float)ch * CELL_ASPECT) / (float)cw;

    Vec3 rd = v3norm(v3add(v3add(
        v3mul(right, u * FOV_HALF_TAN),
        v3mul(up,    v * FOV_HALF_TAN * phys_aspect)),
        fwd));

    float trap, smooth, glow_str;
    float t = mb_march(cam_pos, rd, power, max_steps,
                       &trap, &smooth, &glow_str, trap_type);

    PixCell cell = {smooth, trap, false, glow_str};
    if (t >= 0.0f) cell.hit = true;
    return cell;
}

/*
 * canvas_render_rows() — render ROWS_PER_TICK rows into g_fbuf.
 * Returns true when a full frame completes and g_stable is updated (T.10).
 */
static bool canvas_render_rows(int cw, int ch,
                                Vec3 cam_pos, Vec3 fwd, Vec3 right, Vec3 up,
                                float power, int max_steps, TrapType trap_type)
{
    for (int k = 0; k < ROWS_PER_TICK; k++) {
        int py = g_render_row;
        for (int px = 0; px < cw; px++)
            g_fbuf[py][px] = mb_cast_pixel(px, py, cw, ch,
                                            cam_pos, fwd, right, up,
                                            power, max_steps, trap_type);
        if (++g_render_row >= ch) {
            g_render_row = 0;
            memcpy(g_stable, g_fbuf, sizeof g_stable); /* T.10: save complete frame */
            return true;
        }
    }
    return false;
}

/*
 * canvas_draw() — starfield, then glow corona, then fractal surface.
 *
 * Hit pixel display pipeline:
 *
 *   smooth → hue pair (T.H): the sole depth signal — colour is everything.
 *   Character is always '.' for every hit pixel.  A uniform dot-field
 *   lets the colour do the talking: the rainbow bands read as clean
 *   concentric depth shells with no char-density gradient competing.
 *
 *   Glow (T.13): near-miss pixels draw '.' in the matching hue so the
 *   silhouette corona blends with the adjacent surface colour.
 *
 * A_BOLD on every hit pixel: ncurses A_NORMAL renders colour pairs at
 * ~60% terminal intensity.  A_BOLD gives full colour saturation so the
 * rainbow reads at its intended vividness.
 */
static void canvas_draw(int cw, int ch, int cols, int rows,
                         int color_bands, bool clean_mode)
{
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

    /* ── 2 & 3. Glow corona + fractal surface ────────────────────── */
    for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
        for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {

            /* T.10 stable buffer: completed rows from g_fbuf, pending from g_stable */
            PixCell *cell = (vy < g_render_row)
                          ? &g_fbuf[vy][vx]
                          : &g_stable[vy][vx];

            char   ch_c;
            attr_t attr;

            if (!cell->hit) {
                if (cell->glow_str < 0.02f) continue;

                /* T.13 near-miss glow: '.' in the matching hue so the corona
                 * blends smoothly with the adjacent surface colour.           */
                int pair = hue_color_pair(cell->smooth, color_bands);
                attr = COLOR_PAIR(pair) | A_BOLD;
                ch_c = '.';

            } else {
                /* Hit pixel: smooth → hue pair (T.H) is the only depth signal.
                 * Character is always '.' — a uniform dot-field lets the colour
                 * rings read as pure depth layers with nothing competing.      */
                (void)clean_mode;   /* no char variation — clean_mode unused here */
                int pair = hue_color_pair(cell->smooth, color_bands);
                attr = COLOR_PAIR(pair) | A_BOLD;
                ch_c = '.';
            }

            /* Stamp the character across all terminal pixels of this cell */
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
    bool     fast_mode;

    /* hue palette animation — g_hue_offset driven by hue_phase */
    float    hue_phase;

    /* power morph: sine oscillation between MORPH_LO and MORPH_HI */
    bool     morph_mode;
    float    morph_phase;

    /* clean mode (T.12): char from orbit trap, not smooth iteration */
    bool     clean_mode;

    /* color bands (T.H): 1..5 full rainbow cycles across depth range */
    int      color_bands;

    float    time;
} Scene;

static void scene_camera(const Scene *s,
                          Vec3 *cam, Vec3 *fwd, Vec3 *right, Vec3 *up)
{
    float ct = cosf(s->cam_theta), st = sinf(s->cam_theta);
    float cp = cosf(s->cam_phi),   sp = sinf(s->cam_phi);

    *cam   = v3mul(v3(ct*cp, st, ct*sp), s->cam_dist);
    *fwd   = v3norm(v3mul(*cam, -1.0f));
    Vec3 w = v3norm(v3(0, 1, 0));
    if (fabsf(v3dot(*fwd, w)) > 0.99f) w = v3(0, 0, 1);
    *right = v3norm(v3cross(*fwd, w));
    *up    = v3cross(*right, *fwd);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cw          = canvas_w_from_cols(cols);
    s->ch          = canvas_h_from_rows(rows);
    if (s->cw > CANVAS_MAX_W) s->cw = CANVAS_MAX_W;
    if (s->ch > CANVAS_MAX_H) s->ch = CANVAS_MAX_H;
    s->power       = (float)MB_POWER_DEFAULT;
    s->trap_type   = TRAP_POINT;
    s->max_steps   = MB_MAX_STEPS_FULL;
    s->cam_dist    = CAM_DIST_DEFAULT;
    s->cam_theta   = CAM_THETA_DEFAULT;
    s->cam_phi     = CAM_PHI_DEFAULT;
    s->orbit_spd   = CAM_ORBIT_SPD;
    s->color_bands = COLOR_BANDS_DEFAULT;
    memset(g_fbuf, 0, sizeof g_fbuf);
    stars_init(s->cw, s->ch);
    g_render_row   = 0;
    g_dirty        = true;
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

    /* hue rotation (T.H): slowly drift the rainbow offset so the palette
       breathes over time without any camera or geometry change             */
    s->hue_phase += HUE_PHASE_SPD * dt;
    g_hue_offset  = (int)(s->hue_phase) % HUE_N;

    /* power morph: sine oscillation between MORPH_LO and MORPH_HI */
    if (s->morph_mode) {
        s->morph_phase += MORPH_SPD * dt;
        float new_p = MORPH_LO
                    + (MORPH_HI - MORPH_LO)
                    * 0.5f * (1.0f + sinf(s->morph_phase));
        if (fabsf(new_p - s->power) > 0.06f)
            s->power = new_p;
            /* no g_dirty: progressive scan during morph looks like radar sweep */
    }
}

static void scene_render(Scene *s)
{
    /* T.10: dirty restart clears in-progress g_fbuf; g_stable fills the gap */
    if (g_dirty) {
        g_render_row = 0;
        memset(g_fbuf, 0, sizeof(PixCell) * (size_t)s->ch * CANVAS_MAX_W);
        g_dirty = false;
    }

    /* T.11: capture camera basis once at pass start so all rows share it */
    if (g_render_row == 0)
        scene_camera(s, &g_snap_cam, &g_snap_fwd, &g_snap_right, &g_snap_up);

    canvas_render_rows(s->cw, s->ch,
                       g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up,
                       s->power, s->max_steps, s->trap_type);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    canvas_draw(s->cw, s->ch, cols, rows, s->color_bands, s->clean_mode);
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
    color_init();
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
             "Mandelbulb Hue-Depth  %.1f fps  [%dx%d]  row %d/%d",
             fps, s->cw, s->ch, g_render_row, s->ch);
    mvprintw(sc->rows - 2, 0,
             "power:%.1f  trap:%-5s  bands:%d  dist:%.2f  orb:%.2f"
             "  %s%s%s%s",
             s->power, trap_name(s->trap_type),
             s->color_bands,
             s->cam_dist, s->orbit_spd,
             s->fast_mode  ? "FAST  " : "      ",
             s->auto_zoom  ? "ZOOM  " : "      ",
             s->morph_mode ? "MORPH " : "      ",
             s->clean_mode ? "CLN"    : "   ");
    mvprintw(sc->rows - 1, 0,
             "q:quit spc:pause arrows:orbit p/P:power m:morph n:clean "
             "t:trap c:bands(1-5) z/Z:dist a:zoom f:fast r:reset []:spd");
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
    Scene *s     = &app->scene;
    bool   dirty = false;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':  s->paused = !s->paused;                                     break;

    case KEY_LEFT:
        s->cam_phi   -= CAM_STEP_PHI;   dirty = true;                     break;
    case KEY_RIGHT:
        s->cam_phi   += CAM_STEP_PHI;   dirty = true;                     break;
    case KEY_UP:
        s->cam_theta += CAM_STEP_THETA;
        if (s->cam_theta >  1.5f) s->cam_theta =  1.5f;
        dirty = true;                                                      break;
    case KEY_DOWN:
        s->cam_theta -= CAM_STEP_THETA;
        if (s->cam_theta < -1.5f) s->cam_theta = -1.5f;
        dirty = true;                                                      break;

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
        s->morph_phase = 0.0f;
        break;

    case 't':
        s->trap_type = (TrapType)((s->trap_type + 1) % TRAP_N);
        dirty = true;                                                      break;

    case 'c':
        /* cycle colour bands 1→2→3→4→5→1 (T.H: rainbow cycles per depth) */
        s->color_bands = s->color_bands % COLOR_BANDS_MAX + 1;
        break;

    case 'z':
        s->cam_dist -= CAM_STEP_DIST;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        dirty = true;                                                      break;
    case 'Z':
        s->cam_dist += CAM_STEP_DIST;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        dirty = true;                                                      break;

    case 'a':  s->auto_zoom = !s->auto_zoom;                              break;

    case 'f':
        s->fast_mode = !s->fast_mode;
        s->max_steps = s->fast_mode ? MB_MAX_STEPS_FAST : MB_MAX_STEPS_FULL;
        dirty = true;                                                      break;

    case 'n':
        /* T.12 clean mode: char from orbit trap proximity, not smooth iter */
        s->clean_mode = !s->clean_mode;
        /* no g_dirty — clean_mode affects only canvas_draw, not the march */
        break;

    case '[':
        s->orbit_spd -= 0.05f;
        if (s->orbit_spd < 0.0f) s->orbit_spd = 0.0f;
        break;
    case ']':
        s->orbit_spd += 0.05f;
        if (s->orbit_spd > 2.0f) s->orbit_spd = 2.0f;
        break;

    case 'r':
        s->cam_dist  = CAM_DIST_DEFAULT;
        s->cam_theta = CAM_THETA_DEFAULT;
        s->cam_phi   = CAM_PHI_DEFAULT;
        dirty = true;                                                      break;

    default: break;
    }

    if (dirty) g_dirty = true;
    return true;
}

static void app_run(App *app)
{
    const int64_t frame_ns = NS_PER_SEC / SIM_FPS;
    int64_t  fps_acc   = 0;
    int      fps_count = 0;
    double   fps_disp  = 0.0;
    int64_t  prev      = clock_ns();

    app->running = 1;

    while (app->running) {
        int64_t now = clock_ns();
        int64_t dt_ns = now - prev;
        prev = now;
        float   dt   = (float)(dt_ns) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;

        if (app->need_resize) app_do_resize(app);

        int key;
        while ((key = getch()) != ERR) {
            if (!app_handle_key(app, key)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        scene_tick(&app->scene, dt);
        scene_render(&app->scene);

        fps_acc   += dt_ns;
        fps_count += 1;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp  = fps_count * 1e9 / (double)fps_acc;
            fps_acc   = 0;
            fps_count = 0;
        }

        screen_draw(&app->screen, &app->scene, fps_disp);
        screen_present();

        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(frame_ns - elapsed);
    }
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,  on_exit_signal);
    signal(SIGTERM, on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app = &g_app;
    screen_init(&app->screen);
    scene_init(&app->scene,
               app->screen.cols,
               app->screen.rows - HUD_ROWS);
    app_run(app);
    screen_free(&app->screen);
    return 0;
}
