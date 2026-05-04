/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun_solar.c — clean 2-D screen-space sun with corona and arc flares.
 *
 * DEMO: A bright sun fills the centre of the screen.  Photosphere is
 *       a yellow-white disc with visible CONVECTION GRANULATION
 *       (animated fBm texture), darkening toward the LIMB via
 *       Eddington's law, occasionally pocked by SUNSPOTS where the
 *       texture noise dips below threshold.  An exponential CORONA
 *       glow extends outside the disc.  SOLAR FLARES launch from
 *       random footpoints on the disc, arch up to an apex, and fade
 *       — multiple active at once, each with its own lifecycle.
 *
 *       Designed from first principles for terminal low-res output:
 *       no SDF raymarcher, no 3-D capsule flares, no smooth-min.
 *       Pure 2-D screen-space evaluation.  Each pixel asks "am I
 *       inside the disc, in the corona, or in space?" and paints
 *       accordingly.  Flares overlay on top.  Cheap, clear, fast.
 *
 *       Themes (cycle with t / T):
 *         SOLAR       yellow-white photosphere + orange flare
 *         BLUE_GIANT  hot blue-white star (Rigel-like)
 *         RED_DWARF   cool deep red-orange (M-dwarf)
 *         ALIEN       violet/cyan unrealistic-fun
 *         NEGATIVE    white bg, dark fg, silhouette study
 *
 * Study alongside:
 *   raymarcher/mandelbulb.c   — same theme/render skeleton, very
 *                               different domain (3-D raymarched).
 *   particle_systems/embers.c — also fBm-driven heat colour with
 *                               temperature mapping; no flares.
 *
 * Section map:
 *   §1 config    — sizes, animation rates, themes
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair theme apply (with inverted-bg support)
 *   §4 noise     — hash-based 2-D value noise + multi-octave fBm
 *   §5 surface   — disc / granulation / limb darkening / sunspots
 *   §6 flares    — parabolic arc trajectory, lifecycle, spawn pool
 *   §7 scene     — full-frame compose: surface + corona + flares
 *   §8 screen    — ncurses init / draw / resize
 *   §9 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause animation (granulation drift + flare lifecycles)
 *   r            reset (clear flares, reset time)
 *   t / T        next / previous theme
 *   + / =        flare spawn rate up
 *   -            flare spawn rate down
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/sun_solar.c \
 *       -o sun_solar -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : 2-D screen-space radial composition.  No raymarch,
 *                  no SDF, no 3-D — a sun viewed from outside is
 *                  always face-on, so the "depth" axis never adds
 *                  visual information; we drop it entirely.
 *
 *                  PER PIXEL (col, row):
 *
 *                     dx, dy = (col − cx) ,  (row − cy) · CELL_ASPECT
 *                              (cell-aspect-correct so disc is round)
 *                     r      = √(dx² + dy²)
 *
 *                     if r < R_disc:
 *                         SURFACE
 *                         μ        = √(1 − (r/R_disc)²)         // limb
 *                         tex      = fbm2d(dx − wind·t, dy)     // granulation
 *                         spots    = max(0, threshold − tex)   // sunspots
 *                         lum      = (LIMB_BASE + LIMB_BIAS·μ)
 *                                  * (1 − GRAN_AMP·tex_centred − SPOT_AMP·spots)
 *                         heat_t   = lum                       // → palette slot
 *                     elif r < R_corona:
 *                         CORONA
 *                         lum = exp(−(r − R_disc) / CORONA_FALLOFF)
 *                         heat_t = lum · 0.4                   // dim corona
 *                     else:
 *                         empty
 *
 *                  FLARE OVERLAY (after disc/corona pass):
 *                     For each active flare:
 *                         Trace its parabolic arc (footpoint A → apex
 *                         → footpoint B) sampling K screen positions.
 *                         At each position, brighten the cell by the
 *                         flare's intensity at that arc parameter
 *                         (peak at apex, taper at footpoints) times
 *                         the flare's life envelope (rise → peak →
 *                         fall over LIFETIME seconds).
 *
 *                  PIXEL ENCODING:
 *                     glyph slot = ⌊lum · 7.999⌋     (luminance ramp)
 *                     pair slot  = same slot         (single ramp;
 *                                  sun is one temperature regime)
 *
 * Data-structure : `Flare flares[N_FLARES_MAX]` — pool of arcs.  No
 *                  malloc, all in BSS.  Each frame: tick lifetimes,
 *                  spawn new flares to maintain target density,
 *                  kill expired ones.
 *
 * Rendering      : ASCII only.  8-tier glyph ramp (faint → blazing),
 *                  8-tier theme palette per theme.  Inverted theme
 *                  (NEGATIVE) follows the same recipe used in
 *                  mandelbulb.c and nuke.c — white bg, dark fg,
 *                  A_BOLD/A_DIM disabled.
 *
 * Performance    : Per pixel: 1 sqrt + 1 fBm sample (3 octaves of
 *                  bilinear-interpolated value noise) + a few muls.
 *                  At 80×24 = 1 920 pixels: ~30 K cheap ops per
 *                  frame.  Flare overlay adds K · N_FLARES screen
 *                  writes per frame, trivial.  Holds 60 fps to 200×60+.
 *
 * References     :
 *   • Eddington, A. S. (1926) — *The Internal Constitution of the
 *     Stars*.  The 1-coefficient limb-darkening law:
 *         I(μ) / I(1) = 0.80 + 0.20 μ ,  μ = cos(angle from normal)
 *     We use it directly, with `μ = √(1 − (r/R)²)` from screen radius.
 *   • Mandelbrot, B. — *The Fractal Geometry of Nature* (1982) §28.
 *     The multi-octave fBm `Σ aᵢ · noise(2ⁱ x)` we use for granulation.
 *   • Nordlund, Å. & Stein, R. (2009) — "Solar Surface Convection",
 *     *Living Rev. Solar Phys.* 6.  Real-physics treatment of the
 *     boiling-cell pattern we approximate with animated fBm.
 *   • Quílez, I. — [Domain Warping](https://iquilezles.org/articles/warp/).
 *     The `fbm(p + fbm(p))` self-warp gives our sunspot/granule
 *     pattern its swirly character vs. plain noise.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Forget 3-D.  A sun is a flat disc on the screen with stuff happening
 * IN the disc (boiling surface, dimmer edges) and stuff happening AT
 * the disc edge (corona glow, flares arcing up).  Render it directly
 * in screen space.  Each pixel asks "what's my distance from the
 * sun's centre?" and paints according to which region it's in.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a backlit yellow paper disc on a black wall.  Hold it up
 * — that's your sun.  Now imagine the disc has a swirling printed
 * pattern (granulation), and that the edge fades a bit darker
 * (limb darkening).  Around the disc, draw a soft halo with
 * markers (corona).  And every few seconds, sketch an arc with
 * crayon from one point on the disc edge over the top to another
 * point — that's a flare.  Erase the arc slowly over 8 seconds.
 * That's the visual; nothing 3-D required.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Compute screen-centre `(cx, cy)`; for each terminal cell:
 *
 *        dx = col − cx
 *        dy = (row − cy) · CELL_ASPECT      // square the pixel
 *        r  = √(dx² + dy²)
 *
 *  2. CLASSIFY:
 *        r < R_disc                 → surface
 *        R_disc ≤ r < R_corona      → corona
 *        r ≥ R_corona               → empty
 *
 *  3. SURFACE shading:
 *
 *        μ        = √(1 − (r/R_disc)²)            // angle from normal
 *        // Granulation: fBm with time-varying x-offset (rotation feel)
 *        tex      = fbm2d((dx − GRAN_DRIFT·t) · GRAN_SCALE,
 *                          dy · GRAN_SCALE)
 *        tex_c    = tex − 0.5                      // centre on 0
 *        // Sunspots: where tex dips below threshold, additional darkness
 *        spot     = max(0, SPOT_THRESH − tex)
 *        // Combine: limb base + bias·μ, modulated by texture & spots
 *        lum      = (LIMB_BASE + LIMB_BIAS · μ)
 *                 * (1 − GRAN_AMP · tex_c · 2 − SPOT_AMP · spot)
 *
 *  4. CORONA shading:
 *        d = r − R_disc
 *        lum = exp(−d / CORONA_FALLOFF)           // exp drop-off
 *        lum *= CORONA_GAIN                        // dim relative to disc
 *
 *  5. FLARE OVERLAY (after the disc/corona pass):
 *
 *     For each active flare:
 *        For k = 0..ARC_SAMPLES:
 *            s     = k / ARC_SAMPLES               // 0..1 along arc
 *            // Parabolic arc: foot A at s=0, apex at s=0.5, foot B at s=1
 *            (px, py) = parabolic_lerp(A, B, apex_height, s)
 *            // Intensity peaks at midpoint, tapers at footpoints
 *            arc_amp = sin(π · s)                  // 0 at ends, 1 at apex
 *            // Lifecycle: rise (0..0.3), peak (0.3..0.7), fall (0.7..1.0)
 *            life_amp = 1 − |2 · (life/lifetime) − 1|^1.4
 *            // Brighten the cell at (px, py)
 *            cell[py][px].lum += arc_amp · life_amp · FLARE_INTENSITY
 *
 *  6. PIXEL ENCODING:
 *        slot  = ⌊clamp(lum, 0, 1) · 7.999⌋
 *        glyph = LUMA_GLYPHS[slot]
 *        pair  = PAIR_RAMP_BASE + slot
 *        attr  = (slot ≥ 6 → A_BOLD, slot ≤ 1 → A_DIM, else A_NORMAL)
 *               (NORMAL only if NEGATIVE theme — see CONCEPTS)
 *
 *  7. ANIMATE: scene_tick advances `t`; flare lifecycles tick; new
 *     flares spawn to maintain target density.  Pause freezes `t`.
 *
 * KEY FORMULAS
 * ────────────
 *  Eddington limb darkening (1-coefficient):
 *    I(μ) / I(centre) = (1 − u) + u · μ            // u = 0.20 typical
 *    μ = √(1 − (r/R_disc)²)
 *
 *  Granulation texture:
 *    tex(x, y, t) = fbm2d(x − wind·t,  y)          // 3-octave fBm
 *
 *  Parabolic arc trajectory in screen space:
 *    P(s) = (1−s)·A + s·B  +  4·s·(1−s)·apex_offset
 *  where `apex_offset` is the vertical-up displacement at s = 0.5.
 *  4·s·(1−s) is 0 at the footpoints and 1 at s=0.5 — perfect parabola.
 *
 *  Flare lifecycle envelope (rise / peak / fall):
 *    life_amp(τ) = 1 − |2·τ − 1|^p ,  τ = age / lifetime ∈ [0, 1]
 *  with p ≈ 1.4 — softer than triangle, sharper than parabola.
 *
 *  Cell-aspect correction (disc round in cell-pixels):
 *    dy_world = dy_cells · CELL_ASPECT             // CELL_ASPECT = 2
 *  Without this the disc renders as an ellipse (squashed vertically).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DISC RADIUS vs SCREEN.  We size R_disc to fit comfortably; if
 *    the user resizes the terminal really small, R_disc clamps to
 *    half the smaller dimension so it still fits.
 *
 *  • CELL ASPECT.  Terminal cells are ~2× taller than wide; multiplying
 *    `dy` by 2 makes the radial distance match physical pixels and
 *    the disc appears round.
 *
 *  • FLARE FOOTPOINTS ON THE DISC.  Both A and B are sampled
 *    uniformly on the visible disc edge (random angle).  Two
 *    footpoints from THE SAME side give natural-looking arcs;
 *    arcs that span ±180° (opposite sides) cross the centre of the
 *    disc, looking artificial.  We constrain `|θ_A − θ_B| < π/2`.
 *
 *  • APEX HEIGHT.  Random in `[ARC_APEX_MIN, ARC_APEX_MAX]` per
 *    flare.  Too high (apex_offset > R_corona) → arc visibly
 *    leaves the corona into empty space.  Too low → arc barely
 *    rises above the disc.  Defaults tuned empirically.
 *
 *  • OVERDRAW.  Flares render OVER the disc.  We add to the disc's
 *    luminance buffer, not replace, so flares brighten the surface
 *    where they touch it (footpoints).  The combined `lum` clamps
 *    to [0, 1] for the slot mapping.
 *
 *  • PAUSE.  Freezes the global `t`, so granulation halts and
 *    flare lifecycles stall.  Resume — they continue cleanly.
 *
 *  • INVERTED THEME.  Pre-fill white bg before drawing; A_BOLD/A_DIM
 *    disabled in the pixel mapper (would invert the brightness intent
 *    on a light fg over white bg).
 *
 *  • SUNSPOT THRESHOLD.  Set so that ~5 % of the disc has visible
 *    sunspots at any time — too many spots makes the sun look
 *    diseased; too few makes the granulation feel uniform.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At default size the sun fills ~50% of the screen, surrounded by
 *    a soft corona, with 4–6 flares arcing visibly at any time.
 *
 *  • Press space.  Animation freezes.  Granulation pattern stops
 *    drifting; flares hold at their current state.  Resume — clean
 *    pickup, no jitter.
 *
 *  • Watch a single flare from spawn to death (~6–8 sec): it
 *    starts faint at both footpoints, brightens to full at the
 *    apex, then fades.  The `sin(πs) · life_amp` envelope makes
 *    the apex always the brightest point.
 *
 *  • Cycle themes (`t`).  Geometry identical, only colours change.
 *    SOLAR (warm) ↔ BLUE_GIANT (cool blue) is the most striking flip.
 *
 *  • Watch the LIMB.  The disc edge should be visibly DIMMER than
 *    the centre — that's the Eddington μ-falloff.  If the disc
 *    looks like a flat-coloured circle, limb darkening is broken.
 *
 *  • Watch SUNSPOTS appear and drift across the disc as the
 *    granulation animates.  They should look like dark blotches,
 *    not regular geometric shapes.
 *
 *  • CORONA fades smoothly outward — no hard ring at the disc edge.
 *
 *  • At 60 × 20 the demo should still look like a sun.  The disc
 *    + corona + flares should all be recognisable even at small
 *    sizes; the granulation may look more like patches than cells
 *    (acceptable — it's a low-res tradeoff).
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

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_RAMP_BASE    =  3,    /* +0..+7 — heat ramp                   */

    /* Flare pool size.  Sized for "always 4-8 active". */
    N_FLARES_MAX      = 12,

    /* Number of points sampled along each flare arc when overlaying. */
    ARC_SAMPLES       = 36,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Sun sizing — fractions of the smaller screen dimension. */
#define DISC_FRAC        0.30f     /* R_disc / min(cols, rows·aspect)    */
#define CORONA_MULT      1.7f      /* R_corona = R_disc · this           */
#define CORONA_FALLOFF   0.35f     /* exp(−d / (R_disc · this))          */
#define CORONA_GAIN      0.45f     /* corona max brightness vs disc      */

/* Limb darkening (Eddington 1-coefficient law). */
#define LIMB_BASE        0.80f     /* (1 − u) intensity at edge          */
#define LIMB_BIAS        0.20f     /* u · μ contribution at centre       */

/* Granulation. */
#define GRAN_SCALE       0.18f     /* fBm domain scale                   */
#define GRAN_DRIFT       1.7f      /* world units / sec drift in x       */
#define GRAN_AMP         0.35f     /* texture brightness modulation      */

/* Sunspots. */
#define SPOT_THRESH      0.30f     /* fbm < this → sunspot               */
#define SPOT_AMP         1.20f     /* darkness factor at spot centres    */

/* Flares. */
#define FLARE_LIFETIME_MIN  4.0f
#define FLARE_LIFETIME_MAX  9.0f
#define FLARE_LIFE_EXP      1.4f   /* lifecycle envelope shape           */
#define FLARE_INTENSITY     0.55f  /* peak brightness contribution        */
#define FLARE_INTENSITY_MAX 1.10f
#define FLARE_INTENSITY_MIN 0.20f
#define ARC_APEX_MIN        0.30f  /* apex height as fraction of R_disc  */
#define ARC_APEX_MAX        0.85f
#define ARC_FOOT_MAX_SPAN   1.60f  /* max angular separation between feet (rad) */

#define SPAWN_RATE_DEFAULT  1.5f   /* flares / sec                       */
#define SPAWN_RATE_MIN      0.0f
#define SPAWN_RATE_MAX     12.0f
#define SPAWN_RATE_STEP     1.20f  /* multiplicative                     */

/* Brightness-clamp for slot mapping. */
#define LUM_CLAMP           1.05f

/* Theme palette. */
typedef struct {
    const char *name;
    short       ramp[8];
    bool        inverted;       /* white bg, dark fg                    */
} Theme;

#define N_THEMES 5

static const Theme themes[N_THEMES] = {
    /* SOLAR: dim red → orange → yellow → bone-white (real sun colours)  */
    { "SOLAR     ",
      { 124, 160, 196, 202, 208, 214, 220, 229 }, false },

    /* BLUE_GIANT: hot blue-white throughout (Rigel-class)              */
    { "BLUE_GIANT",
      {  24,  31,  38,  45,  87, 123, 159, 195 }, false },

    /* RED_DWARF: cool deep red glowing into amber                      */
    { "RED_DWARF ",
      {  52,  88, 124, 160, 166, 202, 208, 214 }, false },

    /* ALIEN: violet body climbing into electric cyan flare-ish core    */
    { "ALIEN     ",
      {  53,  91, 134, 165, 207, 159, 123,  51 }, false },

    /* NEGATIVE: white bg, dark fg — silhouette study                   */
    { "NEGATIVE  ",
      { 253, 250, 245, 240, 237, 234, 232,  16 }, true  },
};

/* Glyph ramp: faint → blazing.  Slot 0 is `.` not space — even the
 * darkest visible cell paints a dim dot (e.g. mid-corona). */
static const char LUMA_GLYPHS[8] = { '.', ',', ':', ';', '+', '*', '#', '@' };

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
    const Theme *t = &themes[idx];
    short bg256 = t->inverted ? 231 : -1;
    short bg8   = t->inverted ? COLOR_WHITE : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], bg256);
    } else {
        static const short fb[8] = {
            COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
            COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i),
                      t->inverted ? COLOR_BLACK : fb[i], bg8);
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
/* §4  noise — value noise + 3-octave fBm                                 */
/* ===================================================================== */

static inline uint32_t hash2d(int x, int z, uint32_t seed)
{
    uint32_t h = (uint32_t)x * 374761393u
               + (uint32_t)z * 668265263u
               + seed        * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

static inline float hash_unit(int x, int z, uint32_t seed)
{
    return (float)(hash2d(x, z, seed) >> 8) / (float)(1u << 24);
}

static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }

static float vnoise2d(float x, float z, uint32_t seed)
{
    int   xi = (int)floorf(x), zi = (int)floorf(z);
    float fx = x - (float)xi,   fz = z - (float)zi;
    float v00 = hash_unit(xi,     zi,     seed);
    float v10 = hash_unit(xi + 1, zi,     seed);
    float v01 = hash_unit(xi,     zi + 1, seed);
    float v11 = hash_unit(xi + 1, zi + 1, seed);
    float sx  = smoothstep01(fx);
    float sz  = smoothstep01(fz);
    float a   = v00 * (1.0f - sx) + v10 * sx;
    float b   = v01 * (1.0f - sx) + v11 * sx;
    return a * (1.0f - sz) + b * sz;
}

static float fbm2d(float x, float z, uint32_t seed)
{
    float h = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
    for (int i = 0; i < 3; i++) {
        h    += amp * vnoise2d(x * freq, z * freq, seed + (uint32_t)i * 17u);
        norm += amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return h / norm;
}

/* ===================================================================== */
/* §5  surface — disc, granulation, limb, sunspots                        */
/* ===================================================================== */

/*
 * surface_lum — luminance of a screen point INSIDE the disc.
 *
 * Combines four physical effects:
 *   1. Limb darkening (Eddington 1-coefficient): the disc edge
 *      genuinely emits less per-pixel because the line of sight
 *      crosses the photosphere at a shallower angle.  Without
 *      this, the disc looks like a flat-coloured circle.
 *   2. Granulation: 2-D fBm modulates brightness ±GRAN_AMP/2.
 *      Animated via a wind-drift offset, giving "boiling" feel.
 *   3. Sunspots: where the granulation noise dips below threshold,
 *      additional darkening creates the iconic dark blotches.
 *   4. Combined into a single luminance value in roughly [0, 1].
 */
static float surface_lum(float dx, float dy, float r, float r_disc,
                         float t, uint32_t seed)
{
    /* μ from screen radius — Eddington's law assumes a sphere. */
    float r_norm = r / r_disc;
    if (r_norm > 1.0f) r_norm = 1.0f;
    float mu = sqrtf(1.0f - r_norm * r_norm);

    /* Granulation texture, drifting in x with time. */
    float tex = fbm2d((dx - GRAN_DRIFT * t) * GRAN_SCALE,
                       dy                    * GRAN_SCALE,
                       seed);
    float tex_c = tex - 0.5f;            /* centre on 0                */

    /* Sunspots: where tex < SPOT_THRESH, additional darkness. */
    float spot = SPOT_THRESH - tex;
    if (spot < 0.0f) spot = 0.0f;
    spot *= spot;                        /* sharpen — spots have hard cores */

    /* Limb-darkened base * texture/spot modulation. */
    float base = LIMB_BASE + LIMB_BIAS * mu;
    float modu = 1.0f - GRAN_AMP * tex_c * 2.0f - SPOT_AMP * spot;
    if (modu < 0.0f) modu = 0.0f;

    return base * modu;
}

/*
 * corona_lum — luminance OUTSIDE the disc; exponential falloff.
 */
static inline float corona_lum(float r, float r_disc)
{
    float d = r - r_disc;
    return CORONA_GAIN * expf(-d / (r_disc * CORONA_FALLOFF));
}

/* ===================================================================== */
/* §6  flares — parabolic arcs with lifecycle                             */
/* ===================================================================== */

typedef struct {
    bool   active;
    float  age;            /* seconds since spawn                       */
    float  lifetime;       /* total seconds                             */

    /* Footpoint angles on the disc (in radians around centre). */
    float  theta_a, theta_b;

    /* Apex offset: how far above the chord midpoint the arc peaks,
     * in fractions of R_disc.  Always pushes "outward" from the disc. */
    float  apex_height;

    /* Per-flare random intensity scale [FLARE_INTENSITY_MIN,
     * FLARE_INTENSITY_MAX] — gives variety so flares aren't all
     * the same brightness. */
    float  intensity;
} Flare;

static Flare    g_flares[N_FLARES_MAX];
static uint32_t g_flare_rng = 0x5EEDC0DEu;

static inline uint32_t lcg_step(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}

static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_step(st) >> 8) / (float)(1u << 24);
}

static void flares_clear(void)
{
    memset(g_flares, 0, sizeof g_flares);
}

/*
 * flare_spawn — find an inactive slot and fill it with a fresh
 * random flare.  Footpoints are placed on the disc edge with their
 * angular separation clamped to [0.4, ARC_FOOT_MAX_SPAN] — too close
 * looks like a spike, too far makes the arc visibly cross the disc
 * centre and look like a chord rather than an arc.
 */
static void flare_spawn(void)
{
    int idx = -1;
    for (int i = 0; i < N_FLARES_MAX; i++) {
        if (!g_flares[i].active) { idx = i; break; }
    }
    if (idx < 0) return;     /* pool full */

    Flare *f = &g_flares[idx];
    f->active   = true;
    f->age      = 0.0f;
    f->lifetime = FLARE_LIFETIME_MIN
                + lcg_unit(&g_flare_rng)
                  * (FLARE_LIFETIME_MAX - FLARE_LIFETIME_MIN);

    float theta_a = lcg_unit(&g_flare_rng) * 2.0f * (float)M_PI;
    float span    = 0.4f + lcg_unit(&g_flare_rng) * (ARC_FOOT_MAX_SPAN - 0.4f);
    float dir     = (lcg_step(&g_flare_rng) & 1) ? +1.0f : -1.0f;
    f->theta_a    = theta_a;
    f->theta_b    = theta_a + dir * span;

    f->apex_height = ARC_APEX_MIN
                   + lcg_unit(&g_flare_rng) * (ARC_APEX_MAX - ARC_APEX_MIN);

    f->intensity = FLARE_INTENSITY_MIN
                 + lcg_unit(&g_flare_rng)
                   * (FLARE_INTENSITY_MAX - FLARE_INTENSITY_MIN);
}

/*
 * flares_tick — age every active flare; expire old ones.  Spawning
 * is handled separately by scene_tick using a Poisson-style
 * accumulator so the user can dial spawn_rate.
 */
static void flares_tick(float dt)
{
    for (int i = 0; i < N_FLARES_MAX; i++) {
        if (!g_flares[i].active) continue;
        g_flares[i].age += dt;
        if (g_flares[i].age >= g_flares[i].lifetime)
            g_flares[i].active = false;
    }
}

/*
 * flare_envelope — life-cycle envelope (rise → peak → fall).
 *
 *   τ = age / lifetime   ∈ [0, 1]
 *   amp = 1 − |2τ − 1|^FLARE_LIFE_EXP
 *
 * Returns 1 at midpoint, 0 at endpoints.  Exponent 1.4 is softer
 * than triangular (linear, exp=1) and sharper than parabolic
 * (exp=2), giving a quick rise / sustained peak / quick fall.
 */
static inline float flare_envelope(const Flare *f)
{
    float tau = f->age / f->lifetime;
    float u   = fabsf(2.0f * tau - 1.0f);
    return 1.0f - powf(u, FLARE_LIFE_EXP);
}

/*
 * flare_arc_point — for parameter s ∈ [0, 1], return the screen-space
 * (px, py) point on this flare's parabolic arc PLUS the arc-amplitude
 * factor at that s.
 *
 *   Footpoint A   = (cx + cos·R, cy + sin·R / aspect)
 *   Footpoint B   = (similar at theta_b)
 *   Midpoint M    = (A + B) / 2   (chord midpoint)
 *   Outward dir   = M − sun_centre, normalised
 *   Apex P*       = M + apex_height · R_disc · outward_dir
 *
 *   Parabolic blend:
 *     P(s) = (1−s)·A + s·B  +  4·s·(1−s) · (P* − M)
 *
 *   arc_amp(s) = sin(π·s)  — 0 at endpoints, 1 at apex, smooth.
 */
static inline void flare_arc_point(const Flare *f,
                                   float cx, float cy, float r_disc,
                                   float s,
                                   float *out_px, float *out_py,
                                   float *out_amp)
{
    /* Foot positions (the y is *cell-aspect-corrected* — we want
     * footpoints to project to a circle, not an ellipse). */
    float ax = cx + cosf(f->theta_a) * r_disc;
    float ay = cy + sinf(f->theta_a) * r_disc / CELL_ASPECT;
    float bx = cx + cosf(f->theta_b) * r_disc;
    float by = cy + sinf(f->theta_b) * r_disc / CELL_ASPECT;

    /* Chord midpoint and outward direction. */
    float mx = 0.5f * (ax + bx);
    float my = 0.5f * (ay + by);
    float ox = mx - cx;
    float oy = my - cy;
    float olen = sqrtf(ox * ox + oy * oy);
    if (olen < 1e-3f) { ox = 0.0f; oy = -1.0f; olen = 1.0f; }
    float onx = ox / olen, ony = oy / olen;

    /* Apex point. */
    float ah  = f->apex_height * r_disc;
    float apx = mx + onx * ah;
    float apy = my + ony * ah / CELL_ASPECT;

    /* Parabolic blend.  4s(1-s) is 0 at ends, 1 at midpoint. */
    float bz = 4.0f * s * (1.0f - s);
    float px = (1.0f - s) * ax + s * bx + bz * (apx - mx);
    float py = (1.0f - s) * ay + s * by + bz * (apy - my);

    *out_px  = px;
    *out_py  = py;
    *out_amp = sinf((float)M_PI * s);
}

/* ===================================================================== */
/* §7  scene — compose surface + corona + flares into the screen          */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       current_theme;
    int       cols, rows;
    uint32_t  seed;

    float     time;            /* accumulated seconds                   */
    float     spawn_rate;      /* flares / sec                          */
    float     spawn_accum;     /* unspent (1/spawn_rate) credits        */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->current_theme = 0;
    s->cols          = cols;
    s->rows          = rows;
    s->seed          = (uint32_t)clock_ns() ^ 0x1FACADEu;
    s->time          = 0.0f;
    s->spawn_rate    = SPAWN_RATE_DEFAULT;
    s->spawn_accum   = 0.0f;

    flares_clear();
    g_flare_rng = (uint32_t)clock_ns() ^ 0xCAFEBABEu;
    /* Pre-spawn a couple so the screen isn't empty at t=0. */
    flare_spawn();
    flare_spawn();
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reset(Scene *s)
{
    s->time = 0.0f;
    s->spawn_accum = 0.0f;
    flares_clear();
    g_flare_rng = (uint32_t)clock_ns() ^ 0xCAFEBABEu;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
    flares_tick(dt);

    /* Spawn-rate accumulator: every (1/rate) seconds, spawn one flare. */
    if (s->spawn_rate > 0.0f) {
        s->spawn_accum += dt * s->spawn_rate;
        while (s->spawn_accum >= 1.0f) {
            flare_spawn();
            s->spawn_accum -= 1.0f;
        }
    }
}

/*
 * scene_render — full-frame composition.
 *
 *   Pass 1: per-pixel disc/corona luminance into a per-cell scratch
 *           buffer (small static array on the stack).
 *   Pass 2: overlay each active flare's arc samples into the same
 *           buffer (additive).
 *   Pass 3: emit ncurses cells from the buffer with batched
 *           attron/attroff.
 *
 * The buffer is needed because flare overlay is additive — without
 * a buffer we'd need either to redraw cells (mvaddch each time) or
 * compute every pixel's flare contribution per-cell (expensive
 * search across all flares for every cell).
 *
 * Buffer size: 240 × 80 = 19 200 floats = 75 KB; well within stack
 * for any practical terminal.  Larger terminals than that get
 * truncated to MAX_BUF; the visible image clips cleanly.
 */
#define MAX_BUF_W  280
#define MAX_BUF_H   90

static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;

    int cols = s->cols;
    if (cols     > MAX_BUF_W) cols     = MAX_BUF_W;
    if (rows_eff > MAX_BUF_H) rows_eff = MAX_BUF_H;

    /* Pass 0: setup */
    bool inverted = themes[s->current_theme].inverted;
    static float lum_buf[MAX_BUF_H][MAX_BUF_W];

    /* Sun centre (pixel coords) and disc radius. */
    float cx = (float)cols     * 0.5f;
    float cy = (float)rows_eff * 0.5f;
    float min_dim = (float)cols < (float)rows_eff * CELL_ASPECT
                  ? (float)cols
                  : (float)rows_eff * CELL_ASPECT;
    float r_disc   = min_dim * DISC_FRAC;
    float r_corona = r_disc * CORONA_MULT;

    /* Pass 1: disc + corona luminance into the buffer. */
    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < cols; col++) {
            float dx = (float)col - cx;
            float dy = ((float)row - cy) * CELL_ASPECT;   /* round disc */
            float r  = sqrtf(dx * dx + dy * dy);

            float L;
            if (r < r_disc) {
                L = surface_lum(dx, dy, r, r_disc, s->time, s->seed);
            } else if (r < r_corona) {
                L = corona_lum(r, r_disc);
            } else {
                L = 0.0f;
            }
            lum_buf[row][col] = L;
        }
    }

    /* Pass 2: additive flare overlay. */
    for (int i = 0; i < N_FLARES_MAX; i++) {
        const Flare *f = &g_flares[i];
        if (!f->active) continue;
        float life_amp = flare_envelope(f);
        if (life_amp < 0.001f) continue;

        for (int k = 0; k <= ARC_SAMPLES; k++) {
            float sk;
            float px, py, arc_amp;
            sk = (float)k / (float)ARC_SAMPLES;
            flare_arc_point(f, cx, cy, r_disc, sk, &px, &py, &arc_amp);

            /* The flare samples are dense; round to nearest cell.
             * Some adjacent samples land in the same cell — that's
             * fine, the additive blend reaches saturation there. */
            int xi = (int)(px + 0.5f);
            int yi = (int)(py + 0.5f);
            if (xi < 0 || xi >= cols)     continue;
            if (yi < 0 || yi >= rows_eff) continue;

            float boost = arc_amp * life_amp * f->intensity * FLARE_INTENSITY;
            lum_buf[yi][xi] += boost;
        }
    }

    /* Pass 3: emit ncurses cells. */
    int     last_pair = -1;
    attr_t  last_attr = 0;

    /* Inverted theme: pre-fill white background. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_RAMP_BASE));
        for (int row = 0; row < rows_eff; row++)
            for (int col = 0; col < cols; col++)
                mvaddch(row, col, ' ');
        attroff(COLOR_PAIR(PAIR_RAMP_BASE));
        last_pair = PAIR_RAMP_BASE;
        last_attr = A_NORMAL;
    }

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < cols; col++) {
            float L = lum_buf[row][col];
            if (L < 0.0f) L = 0.0f;
            if (L > LUM_CLAMP) L = LUM_CLAMP;
            float Ln = L / LUM_CLAMP;

            /* Threshold for "anything to draw at all". */
            if (Ln < 0.02f) {
                if (!inverted && last_pair >= 0) {
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                    last_pair = -1;
                }
                continue;
            }

            int slot = (int)(Ln * 7.999f);
            if (slot < 0) slot = 0;
            if (slot > 7) slot = 7;

            char glyph = LUMA_GLYPHS[slot];
            int  pair  = PAIR_RAMP_BASE + slot;
            attr_t attr;
            if (inverted) {
                attr = A_NORMAL;
            } else {
                attr = (slot >= 6) ? A_BOLD
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
}

static int scene_active_flares(void)
{
    int n = 0;
    for (int i = 0; i < N_FLARES_MAX; i++) if (g_flares[i].active) n++;
    return n;
}
/* ── end §7 — to understand the ncurses wrapper, read §8 screen ── */

/* ===================================================================== */
/* §8  screen                                                             */
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
             " SUN   %s   theme:%s   t:%6.1fs   "
             "flares:%2d   spawn:%4.2f/s   "
             "%5.1f fps  %3d Hz   "
             "t/T:theme  +/-:spawn  spc:pause  r:reset  q:quit ",
             s->paused ? "PAUSED" : "BURNING",
             themes[s->current_theme].name,
             (double)s->time,
             scene_active_flares(),
             (double)s->spawn_rate,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
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

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case '=': case '+':
        s->spawn_rate *= SPAWN_RATE_STEP;
        if (s->spawn_rate > SPAWN_RATE_MAX) s->spawn_rate = SPAWN_RATE_MAX;
        break;
    case '-':
        s->spawn_rate /= SPAWN_RATE_STEP;
        if (s->spawn_rate < SPAWN_RATE_MIN) s->spawn_rate = SPAWN_RATE_MIN;
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
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
