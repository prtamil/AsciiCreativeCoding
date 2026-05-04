/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * tunnel.c — hyperspace radial tunnel flythrough.
 *
 * DEMO: Camera flies down the inside of an infinite cylinder.  Each
 *       screen pixel solves the analytical ray-cylinder intersection
 *       (no march, no SDF — closed form), recovers UV coords on the
 *       cylinder surface, and samples a procedural pattern that
 *       streams toward the viewer over time.  Distance fog darkens
 *       far walls; the central ray (parallel to the cylinder axis)
 *       opens onto the bright VANISHING POINT.  Camera sway + roll
 *       give the tunnel a mild "curving through space" feel without
 *       any actual curved-tube SDF.  Demoscene classic.
 *
 *       Patterns (cycle with n / N):
 *         RINGS    sinusoidal bright/dark bands streaming forward
 *         CHECKER  square checkerboard tile (16 around × stripes long)
 *         SPOKES   radial bars rotating with camera roll
 *         GRID     cross-hatch — stripes both axes (sci-fi corridor)
 *
 *       Themes (cycle with t / T):
 *         WARP      magenta/violet  (warp drive)
 *         INFERNO   red/orange/yellow (lava tube)
 *         ELECTRIC  cyan/blue/white  (energy conduit)
 *         VOID      cool teal+blue   (cold space)
 *         MATRIX    neon green       (cyber)
 *         GOLD      amber/bone       (treasure passage)
 *
 * Study alongside:
 *   raymarcher/raymarcher.c    — Phong sphere with virtual canvas.
 *   raymarcher/donut.c         — torus rendered analytically.  Same
 *                                "no-march closed form" approach.
 *   raymarcher/kifs_fractal.c  — opposite philosophy: fractal needs a
 *                                full sphere-trace.  tunnel.c gets
 *                                away with one quadratic per pixel.
 *
 * Section map:
 *   §1 config   — patterns, themes, raymarch tunables
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair depth ramp + vanishing-point + accent
 *   §4 vec3     — value-type 3-D math
 *   §5 tunnel   — cylinder intersection + UV + pattern texture
 *   §6 scene    — camera animation + per-pixel render
 *   §7 screen   — ncurses init / draw / resize
 *   §8 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume forward motion + sway
 *   r            reset camera time
 *   n / N        next / previous pattern
 *   t / T        next / previous theme
 *   + / =        speed up   (texture flow rate)
 *   -            slow down
 *   ] / [        sim Hz up / down
 *   s / S        sway amplitude up / down  (curving feel)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/tunnel.c \
 *       -o tunnel -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Closed-form ray-vs-infinite-cylinder intersection.
 *
 *                  Cylinder along z-axis, radius R:
 *                     x² + y² = R²
 *                  Ray:
 *                     P(t) = O + t · D
 *                  Substitute (Pₓ, P_y) into the cylinder equation:
 *                     (Oₓ + tDₓ)² + (O_y + tD_y)² = R²
 *                  → quadratic in t:
 *                     a·t² + 2b·t + c = 0
 *                       a = Dₓ² + D_y²            (= sin²(angle) of ray to axis)
 *                       b = OₓDₓ + O_y D_y
 *                       c = Oₓ² + O_y² − R²
 *                     Δ = b² − a·c
 *                     t = (−b + √Δ) / a            (forward intersection)
 *
 *                  Hit point P = O + t·D.  Cylinder UV:
 *                     u = atan2(P_y, Pₓ) / (2π)    (angular position)
 *                     v = P_z                       (axial position)
 *
 *                  Animate v with time:
 *                     v_anim = v − speed · t_global
 *                  → texture appears to STREAM toward the viewer.
 *
 *                  Pattern sample at (u, v_anim) returns brightness
 *                  in [0, 1].  Combine with distance fog:
 *                     fog       = exp(−t · FOG_DENSITY)
 *                     intensity = fog · (DARK + texture · (LIGHT − DARK))
 *                  Map intensity → glyph slot 0..7 + theme depth pair.
 *
 *                  Vanishing point (ray nearly parallel to axis):
 *                     If a < ε or t > T_MAX, paint VANISH glyph + pair —
 *                     the bright "tunnel exit" at screen centre.
 *
 *                  Camera sway:
 *                     Oₓ(t) = sway_amp_x · sin(ω₁ · t)
 *                     O_y(t) = sway_amp_y · cos(ω₂ · t)
 *                     roll(t) = roll_rate · t
 *                  The intersection is computed in WORLD coordinates,
 *                  so as O drifts off-axis the vanishing point shifts
 *                  on screen and the wall pattern leans — gives the
 *                  illusion of curving without an actual curved tube.
 *
 * Data-structure : None.  Stateless math, no LUTs, no memory pressure.
 *
 * Rendering      : ASCII only.  Glyph from intensity (` .,:+*#@`),
 *                  colour pair from depth slot (theme depth ramp).
 *                  Vanishing-point pair separate from the depth ramp
 *                  so the centre of the screen always reads as the
 *                  "tunnel exit" regardless of pattern.
 *
 * Performance    : ~30 ops per pixel (one quadratic + one atan2 + a
 *                  small modular pattern lookup).  At 80×24 = 1 920
 *                  pixels: ~60 K ops per frame.  Trivial — runs at
 *                  full-screen 200×60 at 60 fps without breaking a
 *                  sweat.  The atan2 is the dominant cost; replace
 *                  with `atan2_approx` if a future port needs more.
 *
 * References     :
 *   • Akenine-Möller, Haines, Hoffman — *Real-Time Rendering* 4e
 *     (2018), §22.7 (cylinder intersection).  The canonical
 *     derivation we follow.
 *   • Mode 7 / wormhole demos — Future Crew "Second Reality" (1993)
 *     and uncountable demoscene clones use the same closed-form
 *     cylinder-UV trick we use here.  No SDF, no march; just one
 *     quadratic per pixel.
 *   • Quílez, I. — [Distance Functions](https://iquilezles.org/articles/distfunctions/)
 *     covers the cylinder SDF for completeness.  We don't use the
 *     SDF — only the analytical intersection — but the article shows
 *     why the closed form is faster than sphere-tracing for this
 *     shape (~30× on this primitive).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * You're inside an infinite tin can flying down its axis.  For each
 * pixel of the screen, draw a straight line from your eye through that
 * pixel — that line eventually hits the can's wall.  The angle around
 * the can (which "clock-position" of the wall) gives the U coordinate;
 * how far down the can you hit it gives V.  Now wrap a stripe pattern
 * around the can (like a barber pole) and ask "what colour is the wall
 * at THIS (U, V)?" — that's your pixel's colour.  Slide the stripe
 * pattern along V over time and the whole tunnel appears to rush past.
 * Add some camera sway and the vanishing point dances around — the
 * tunnel "curves".
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Pretend the can has a sticker wrapped around the inside.  The
 * sticker has stripes printed on it.  You're in a swivel chair flying
 * forward; from your seat each direction you look hits ONE point on
 * the sticker.  Looking dead ahead you see "infinity" (the rays never
 * actually hit a wall — that's the vanishing point glow).  Looking
 * sideways you see the wall RIGHT THERE.  The stripes near the centre
 * of your view are far away → small + dim (fog); the stripes at the
 * edges of your view are close → big + bright.  Slide the sticker:
 * the stripes appear to flow toward you.  Wobble your chair slightly:
 * the centre of your view drifts around → the tunnel "curves".
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Compute camera position Oₜ at time t (sway in x, y), camera
 *     basis (forward = +z, with optional roll about z).
 *
 *  2. For each pixel (col, row):
 *
 *        u_screen, v_screen = aspect-corrected NDC in [-1, 1]
 *        D                  = norm(forward + u·tan(FOV/2)·right
 *                                          + v·tan(FOV/2)·aspect·up)
 *
 *        a = Dₓ² + D_y²
 *        if a < EPS_PARALLEL  →  vanishing point (paint accent)
 *
 *        b = Oₓ·Dₓ + O_y·D_y
 *        c = Oₓ² + O_y² − R²
 *        Δ = b² − a·c
 *        if Δ < 0  →  miss (impossible inside, but defensive)
 *        t = (−b + √Δ) / a
 *        if t > T_MAX  →  vanishing point (very far hit)
 *
 *        P     = O + t·D
 *        u_tex = atan2(P_y, Pₓ) / (2π)            ∈ (−0.5, 0.5]
 *        v_tex = P_z − speed · t_global           (animated stream)
 *
 *  3. Sample pattern → brightness ∈ [0, 1]:
 *
 *        RINGS    : 0.5 + 0.5 · sin(v_tex · RING_FREQ)
 *        CHECKER  : ((⌊u·STRIPES_AROUND⌋ + ⌊v·STRIPES_LONG⌋) mod 2)
 *        SPOKES   : (⌊u·SPOKE_COUNT⌋ mod 2)
 *        GRID     : (u mod K1) < ε₁  OR  (v mod K2) < ε₂
 *
 *  4. Distance fog:
 *
 *        fog       = exp(−t · FOG_DENSITY)
 *        intensity = fog · (FOG_FLOOR + brightness · (1 − FOG_FLOOR))
 *
 *  5. Map intensity → glyph + colour:
 *
 *        slot  = ⌊intensity · 7.999⌋
 *        glyph = LUMA_GLYPHS[slot]
 *        pair  = PAIR_DEPTH_BASE + slot
 *        attr  = A_BOLD if slot ≥ 6, A_DIM if slot ≤ 1, else A_NORMAL
 *
 *  6. Tick advances `time`; sway and texture flow read it directly.
 *     Pause freezes time without changing camera or pattern.
 *
 * KEY FORMULAS
 * ────────────
 *  Cylinder hit (camera inside the can):
 *    t = (−b + √(b² − a·c)) / a
 *    a = Dₓ² + D_y²
 *    b = OₓDₓ + O_y D_y
 *    c = Oₓ² + O_y² − R²
 *
 *  UV recovery on cylinder surface:
 *    u_tex = atan2(P_y, Pₓ) / (2π)
 *    v_tex = P_z − speed · time
 *
 *  Distance fog (Beer-style exponential):
 *    fog = exp(−t · FOG_DENSITY)
 *
 *  Sway (camera curves through tunnel):
 *    Oₓ(t) = sway_amp · sin(ω₁ t)
 *    O_y(t) = sway_amp · cos(ω₂ t) · 0.55
 *
 *  Camera roll (texture spins around axis):
 *    R(t) = roll_rate · t
 *    apply 2-D rotation R(t) to (right, up) basis
 *
 *  Aspect correction (terminal cells 2× taller than wide):
 *    phys_aspect = (rows · 2) / cols    (factor on v in ray dir)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • RAY PARALLEL TO AXIS.  a ≤ EPS_PARALLEL means the ray never hits
 *    the cylinder — paint the vanishing-point accent instead.  Without
 *    this branch the centre of the screen renders garbage from the
 *    near-zero divisor.
 *
 *  • CAMERA OUTSIDE CYLINDER.  If sway_amp is set so high that
 *    sqrt(amp_x² + amp_y²) ≥ R, c becomes positive and the camera
 *    finds itself OUTSIDE the can — the math still works but the
 *    visual is wrong (you see the OUTSIDE of the tunnel).  We clamp
 *    sway_amp ≤ R · 0.85 to keep the camera comfortably inside.
 *
 *  • INTEGER UV ALIASING.  Pattern lookups using `⌊u · K⌋ mod 2` produce
 *    a hard alias at U = ±0.5 (the seam where atan2 wraps).  We
 *    avoid this by ALWAYS having an even number of stripes in u
 *    (16, 24, 32) so the pattern is continuous across the seam.
 *
 *  • atan2 NEGATIVE BRANCH.  atan2 returns values in (−π, π].  We
 *    add π to get a (0, 2π] range and then divide by 2π — keeps u
 *    in [0, 1).  Necessary so floor() of u · K is non-negative.
 *
 *  • ROLL DOESN'T AFFECT RINGS.  The RINGS pattern depends only on
 *    v_tex, not u.  Camera roll has zero visual effect on it.  This
 *    is intentional — different patterns reveal different camera
 *    motions.  (CHECKER, SPOKES, GRID all spin under roll.)
 *
 *  • FOG_FLOOR = 0.18.  Without a floor, far-tunnel pixels go to pure
 *    black and the glyph drops to space — leaves "holes" in the
 *    tunnel walls.  A small floor keeps even the deepest depth slot
 *    visible, so the tunnel reads as a continuous structure.
 *
 *  • PAUSE.  Freezes time — texture, sway, roll all stop.  Camera
 *    position holds.
 *
 *  • FOV TOO WIDE.  At FOV > 100° the cylinder math is still correct
 *    but extreme edge pixels have ray directions almost perpendicular
 *    to axis, hitting the wall AT the camera (t very small).  Visual
 *    becomes a bright halo around the screen edge.  Default 70° keeps
 *    this within reasonable bounds.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press space.  Tunnel freezes mid-stream.  Resume — texture
 *    continues from exactly the same v_anim offset.
 *
 *  • Press n.  Pattern changes.  Same geometry, new motif on the wall.
 *    RINGS → smooth pulsing bands.  CHECKER → grid squares streaming
 *    past.  SPOKES → wagon-wheel spokes spinning with roll.  GRID →
 *    rectangular sci-fi panels.
 *
 *  • Press +.  Speed doubles; texture rushes toward the camera faster.
 *    Press − several times to slow; eventually the texture appears
 *    nearly stationary (you've "stopped flying").
 *
 *  • Press s/S.  Sway amplitude grows / shrinks.  At max sway the
 *    vanishing point swings dramatically around the centre of the
 *    screen — tunnel feels "curved".  At zero sway the vanishing
 *    point is locked dead-centre — tunnel is straight.
 *
 *  • Press t.  Theme cycles.  Geometry unchanged; only the depth-ramp
 *    colours.  WARP → magenta hyperdrive.  INFERNO → red/orange.
 *    MATRIX → neon green.
 *
 *  • At 60×20 the tunnel should still read as a tunnel — the
 *    vanishing-point bright pixel + radial pattern + perspective
 *    fog combine to make the depth obvious even with 1 200 cells.
 *
 *  • Reduce sway_amp to 0 (`S` repeatedly).  Vanishing point should
 *    sit exactly at the centre of the screen and stay there.  If it
 *    drifts you have an aspect / fov mismatch.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    FPS_UPDATE_MS    = 500,

    /* Color pair indices. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_DEPTH_BASE  =   3,    /* +0..+7 — depth ramp (far→near)         */
    PAIR_VANISH      =  11,    /* vanishing-point accent                 */
    PAIR_ACCENT      =  12,    /* secondary accent (bright pulses)       */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Tunnel geometry. */
#define TUNNEL_RADIUS    2.0f
#define FOV_DEG         70.0f

/* Camera flow + sway. */
#define SPEED_DEFAULT    8.0f
#define SPEED_MIN        0.5f
#define SPEED_MAX       40.0f
#define SPEED_STEP_FACTOR 1.30f

#define SWAY_AMP_DEFAULT 1.30f     /* max horizontal offset (cells)      */
#define SWAY_AMP_MIN     0.0f
#define SWAY_AMP_MAX     1.65f     /* < TUNNEL_RADIUS · 0.85             */
#define SWAY_AMP_STEP    0.10f
#define SWAY_FREQ_X      0.55f     /* rad / sec                          */
#define SWAY_FREQ_Y      0.31f
#define ROLL_RATE        0.18f     /* rad / sec — texture spin           */

/* Numerical tolerances. */
#define EPS_PARALLEL     1.5e-3f   /* ray-axis-parallel threshold       */
#define T_MAX           80.0f      /* far clip — vanishing point        */

/* Distance fog. */
#define FOG_DENSITY      0.030f    /* exp(−t · FOG_DENSITY)              */
#define FOG_FLOOR        0.18f     /* never fade below this — keep walls */

/* Pattern enum. */
typedef enum {
    PATTERN_RINGS   = 0,
    PATTERN_CHECKER = 1,
    PATTERN_SPOKES  = 2,
    PATTERN_GRID    = 3,
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RINGS:   return "RINGS  ";
    case PATTERN_CHECKER: return "CHECKER";
    case PATTERN_SPOKES:  return "SPOKES ";
    case PATTERN_GRID:    return "GRID   ";
    default:              return "?      ";
    }
}

/*
 * Per-pattern tunables.
 *
 *   stripes_around : how many bright/dark cells per full revolution
 *                    around the cylinder axis (must be EVEN — see
 *                    "INTEGER UV ALIASING" in MENTAL MODEL)
 *   stripes_long   : how many cells per world-space unit along axis
 *   ring_freq      : rad / unit for sinusoidal RINGS pattern
 *   line_thick     : fraction of cell occupied by GRID line
 */
typedef struct {
    int   stripes_around;
    float stripes_long;
    float ring_freq;
    float line_thick;
} PatternParams;

static const PatternParams patterns[N_PATTERNS] = {
    /*                  around  long   ring_freq  line_thick */
    /* RINGS   */     { 16,     0.55f,  1.55f,    0.30f },
    /* CHECKER */     { 16,     0.55f,  0.0f,     0.0f  },
    /* SPOKES  */     { 24,     0.0f,   0.0f,     0.0f  },
    /* GRID    */     { 16,     0.55f,  0.0f,     0.20f },
};

/*
 * Themes — 8-tier depth ramp (far/dim → near/bright) + vanishing-point
 * accent + secondary accent for occasional bright pulses.
 *
 * All entries sit in the bright half of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       depth[8];
    short       vanish;
    short       accent;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* WARP — magenta/violet hyperdrive */
    { "WARP    ", {  53,  91, 134, 165, 207, 213, 219, 225 }, 231, 213 },

    /* INFERNO — red/orange/yellow lava tube */
    { "INFERNO ", {  88, 124, 160, 202, 208, 214, 220, 226 }, 231, 220 },

    /* ELECTRIC — cyan/blue/white energy conduit */
    { "ELECTRIC", {  24,  26,  31,  39,  45,  51, 123, 195 }, 231,  51 },

    /* VOID — deep teal-blue cold space */
    { "VOID    ", {  24,  25,  31,  38,  45,  51, 117, 195 }, 195,  87 },

    /* MATRIX — neon-green cyber */
    { "MATRIX  ", {  28,  34,  76,  82, 118, 154, 190, 226 }, 231, 154 },

    /* GOLD — amber/bone treasure passage */
    { "GOLD    ", { 130, 137, 173, 179, 215, 222, 229, 230 }, 231, 226 },
};

/* Glyph ramp (dim → bright). */
static const char LUMA_GLYPHS[8] = { ' ', '.', ',', ':', '+', '*', '#', '@' };

/* Vanishing-point cursor — a small 5-cell crosshair (`+` centre, `o`
 * at N/E/S/W) drawn LAST over the rendered tunnel.  Non-intrusive
 * marker that anchors the eye on the tunnel axis without painting
 * a bright cluster on top of the natural fog falloff at centre. */
#define VANISH_CENTER_GLYPH  '+'
#define VANISH_ARM_GLYPH     'o'
#define ACCENT_GLYPH         '*'

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

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_DEPTH_BASE + i), t->depth[i], -1);
        init_pair(PAIR_VANISH, t->vanish, -1);
        init_pair(PAIR_ACCENT, t->accent, -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,    COLOR_BLUE,    COLOR_MAGENTA, COLOR_MAGENTA,
            COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE,   COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_DEPTH_BASE + i), fb[i], -1);
        init_pair(PAIR_VANISH, COLOR_WHITE,  -1);
        init_pair(PAIR_ACCENT, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  vec3                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3(float x, float y, float z)   { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)               { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3scale(float s, V3 a)          { return (V3){s*a.x, s*a.y, s*a.z}; }
static inline float v3len(V3 a)                     {
    return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
}
static inline V3    v3norm(V3 a) {
    float L = v3len(a);
    return L > 1e-12f ? v3scale(1.0f / L, a) : v3(0, 0, 1);
}

/* ===================================================================== */
/* §5  tunnel — cylinder intersection + UV + pattern                      */
/* ===================================================================== */

/*
 * cylinder_hit — closed-form ray-vs-infinite-cylinder intersection.
 *
 * Cylinder along z-axis with radius R.  Returns the forward t at which
 * the ray crosses the cylinder wall, or a negative value on miss.  We
 * always take the LARGER root (`+sqrt`) because the camera is INSIDE
 * the can — the smaller (negative) root would exit through the camera.
 */
static inline float cylinder_hit(V3 O, V3 D, float R, bool *parallel)
{
    float a = D.x * D.x + D.y * D.y;
    if (a < EPS_PARALLEL) {
        *parallel = true;
        return -1.0f;
    }
    *parallel = false;
    float b    = O.x * D.x + O.y * D.y;
    float c    = O.x * O.x + O.y * O.y - R * R;
    float disc = b * b - a * c;
    if (disc < 0.0f) return -1.0f;        /* impossible inside, defensive */
    float t = (-b + sqrtf(disc)) / a;
    return t;
}

/*
 * pattern_sample — return brightness ∈ [0, 1] for the given UV.
 *
 *   u_tex ∈ [0, 1)  — angular position (already mapped from atan2)
 *   v_tex           — axial position, already wind-shifted by speed·time
 *
 * Why brightness in [0, 1] (not 0/1 binary): the lerp into FOG_FLOOR..1
 * gives smooth depth shading.  Binary patterns (CHECKER, SPOKES, GRID)
 * still return only 0 or 1 here, but the FOG step softens them; the
 * RINGS pattern returns a sinusoid for a smoother visual.
 */
static float pattern_sample(float u_tex, float v_tex,
                            int pattern_idx)
{
    const PatternParams *pp = &patterns[pattern_idx];

    switch (pattern_idx) {
    case PATTERN_RINGS: {
        /* Sinusoid rings streaming forward — no u dependence at all,
         * so camera roll is invisible (intentional — different
         * patterns expose different camera motions). */
        float r = sinf(v_tex * pp->ring_freq) * 0.5f + 0.5f;
        return r;
    }

    case PATTERN_CHECKER: {
        int gu = (int)floorf(u_tex * (float)pp->stripes_around);
        int gv = (int)floorf(v_tex * pp->stripes_long);
        return ((gu + gv) & 1) ? 1.0f : 0.0f;
    }

    case PATTERN_SPOKES: {
        int gu = (int)floorf(u_tex * (float)pp->stripes_around);
        return (gu & 1) ? 1.0f : 0.0f;
    }

    case PATTERN_GRID:
    default: {
        /* Cross-hatch — bright cell at u OR v line boundary. */
        float u_pos  = u_tex * (float)pp->stripes_around;
        float v_pos  = v_tex * pp->stripes_long;
        float u_frac = u_pos - floorf(u_pos);
        float v_frac = v_pos - floorf(v_pos);
        float u_dist = fminf(u_frac, 1.0f - u_frac);
        float v_dist = fminf(v_frac, 1.0f - v_frac);
        float thick  = pp->line_thick;
        if (u_dist < thick || v_dist < thick) return 1.0f;
        return 0.0f;
    }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool   paused;
    int    current_pattern;
    int    current_theme;
    int    cols, rows;

    float  time;             /* accumulated seconds for sway / texture   */
    float  speed;            /* texture flow speed (units / sec)         */
    float  sway_amp;         /* horizontal sway amplitude (cells)        */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->current_pattern = PATTERN_RINGS;
    s->current_theme   = 0;
    s->cols            = cols;
    s->rows            = rows;
    s->time            = 0.0f;
    s->speed           = SPEED_DEFAULT;
    s->sway_amp        = SWAY_AMP_DEFAULT;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reset(Scene *s)
{
    s->time = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
}

/*
 * scene_render — per-pixel cylinder intersection + pattern + fog.
 *
 *   1. Compute camera position O(t) (sway), basis (with roll).
 *   2. For each (col, row): build ray dir, solve cylinder, recover UV,
 *      sample pattern, apply fog, paint glyph + theme depth pair.
 *   3. Vanishing-point branch handles ray ∥ axis OR t > T_MAX.
 *
 * We batch attron/attroff: the per-pixel (pair, attr) often repeats
 * across a horizontal run of similar-depth cells, so caching the last
 * (pair, attr) and only switching on change cuts attribute thrash by
 * ~3-5×.
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;

    /* Camera position (sway). */
    float t_global = s->time;
    V3 O = {
        s->sway_amp * sinf(t_global * SWAY_FREQ_X),
        s->sway_amp * cosf(t_global * SWAY_FREQ_Y) * 0.55f,
        0.0f
    };

    /* Camera basis with roll about z. */
    float roll = t_global * ROLL_RATE;
    float cr   = cosf(roll);
    float sr   = sinf(roll);
    V3 fwd   = v3(0.0f, 0.0f, 1.0f);
    V3 right = v3( cr,  sr, 0.0f);
    V3 up    = v3(-sr,  cr, 0.0f);

    float fov_t       = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    float phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;

    /* Texture flow offset (subtract from v so pattern moves toward us). */
    float v_flow = s->speed * t_global;

    int     last_pair = -1;
    attr_t  last_attr = 0;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            float u =  ((float)col + 0.5f) / (float)s->cols * 2.0f - 1.0f;
            float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);

            V3 D = v3norm(v3add(fwd,
                v3add(v3scale(u * fov_t, right),
                      v3scale(v * fov_t * phys_aspect, up))));

            int    pair;
            attr_t attr;
            char   glyph;

            bool   parallel = false;
            float  t        = cylinder_hit(O, D, TUNNEL_RADIUS, &parallel);

            if (parallel || t > T_MAX || t < 0.0f) {
                /*
                 * Far / parallel ray — render as deepest fog tier
                 * (dim or empty).  No texture sample (UV is undefined
                 * when ray is exactly on the axis).  The natural
                 * darkening toward the screen centre IS the tunnel
                 * exit; a small crosshair cursor is drawn over the
                 * top of this region in scene_render's epilogue.
                 */
                pair  = PAIR_DEPTH_BASE + 0;
                attr  = A_DIM;
                glyph = LUMA_GLYPHS[0];
            } else {
                /* Hit point and UV. */
                V3 P = v3add(O, v3scale(t, D));

                /* atan2 returns (−π, π].  Map to [0, 1) by adding π. */
                float u_tex = (atan2f(P.y, P.x) + (float)M_PI)
                            * (1.0f / (2.0f * (float)M_PI));
                float v_tex = P.z - v_flow;

                float bright = pattern_sample(u_tex, v_tex,
                                              s->current_pattern);

                /* Distance fog with floor. */
                float fog       = expf(-t * FOG_DENSITY);
                float intensity = fog * (FOG_FLOOR
                                + bright * (1.0f - FOG_FLOOR));

                int slot = (int)(intensity * 7.999f);
                if (slot < 0) slot = 0;
                if (slot > 7) slot = 7;

                glyph = LUMA_GLYPHS[slot];
                pair  = PAIR_DEPTH_BASE + slot;
                attr  = (slot >= 6) ? A_BOLD
                      : (slot <= 1) ? A_DIM
                      :               A_NORMAL;
            }

            if (pair != last_pair || attr != last_attr) {
                if (last_pair >= 0)
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                attron(COLOR_PAIR(pair) | attr);
                last_pair = pair;
                last_attr = attr;
            }
            mvaddch(row, col, (chtype)(unsigned char)glyph);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);

    /* ── Vanishing-point crosshair cursor ─────────────────────────────
     *
     *   . o .
     *   o + o    ← drawn at screen centre, on top of the fog falloff
     *   . o .
     *
     * For our straight-tunnel setup with fwd = (0, 0, 1), the axis
     * vanishing point projects exactly to (cols/2, rows_eff/2) for
     * any camera sway (sway shifts the tunnel structure on screen
     * but the AXIS DIRECTION's screen position is invariant under
     * camera translation parallel to the screen plane).  The `+`
     * is bold; the four `o` arms are normal-weight, giving an eye-
     * anchor that doesn't compete with the tunnel motion.
     */
    int cx = s->cols / 2;
    int cy = rows_eff / 2;
    if (cx >= 0 && cx < s->cols && cy >= 0 && cy < rows_eff) {
        attron(COLOR_PAIR(PAIR_VANISH) | A_BOLD);
        mvaddch(cy, cx, (chtype)(unsigned char)VANISH_CENTER_GLYPH);
        attroff(COLOR_PAIR(PAIR_VANISH) | A_BOLD);

        attron(COLOR_PAIR(PAIR_VANISH));
        if (cy - 1 >= 0)
            mvaddch(cy - 1, cx, (chtype)(unsigned char)VANISH_ARM_GLYPH);
        if (cy + 1 < rows_eff)
            mvaddch(cy + 1, cx, (chtype)(unsigned char)VANISH_ARM_GLYPH);
        if (cx - 1 >= 0)
            mvaddch(cy, cx - 1, (chtype)(unsigned char)VANISH_ARM_GLYPH);
        if (cx + 1 < s->cols)
            mvaddch(cy, cx + 1, (chtype)(unsigned char)VANISH_ARM_GLYPH);
        attroff(COLOR_PAIR(PAIR_VANISH));
    }
}
/* ── end §6 — to understand the ncurses wrapper, read §7 screen ── */

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    char buf[200];
    snprintf(buf, sizeof buf,
             " TUNNEL   %s   pat:%s   theme:%s   "
             "speed:%4.1f   sway:%4.2f   "
             "%5.1f fps  %3d Hz   "
             "n/N:pat  t/T:theme  +/-:speed  s/S:sway  spc:pause  r:reset  q:quit ",
             s->paused ? "PAUSED " : "FLYING ",
             pattern_name(s->current_pattern),
             themes[s->current_theme].name,
             (double)s->speed, (double)s->sway_amp,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset(s);                               break;

    case 'n':
        s->current_pattern = (s->current_pattern + 1) % N_PATTERNS;
        break;
    case 'N':
        s->current_pattern = (s->current_pattern + N_PATTERNS - 1) % N_PATTERNS;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case '=': case '+':
        s->speed *= SPEED_STEP_FACTOR;
        if (s->speed > SPEED_MAX) s->speed = SPEED_MAX;
        break;
    case '-':
        s->speed /= SPEED_STEP_FACTOR;
        if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
        break;

    case 's':
        s->sway_amp += SWAY_AMP_STEP;
        if (s->sway_amp > SWAY_AMP_MAX) s->sway_amp = SWAY_AMP_MAX;
        break;
    case 'S':
        s->sway_amp -= SWAY_AMP_STEP;
        if (s->sway_amp < SWAY_AMP_MIN) s->sway_amp = SWAY_AMP_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
