/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perlin_landscape.c — 2-D Parallax Landscape
 *
 * Three independent Perlin noise terrain layers scroll at different
 * speeds (parallax illusion of depth):
 *
 *   Far   — distant mountains  slow  dark blue-grey silhouette
 *   Mid   — rolling hills      medium  olive green
 *   Near  — foreground ground  full speed  bright green
 *
 * Each layer is a 1-D fractal Brownian motion height profile.
 * Painter's algorithm: sky → far → mid → near (each overwrites).
 * Stars are scattered deterministically in the upper sky.
 *
 * Keys: q quit  p pause  r reset  <-/-> speed/dir  +/- zoom  t theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/worldgen/perlin_landscape.c \
 *       -o perlin_landscape -lncurses -lm
 *
 * Sections (re-cut by CONCERN — see ARCHITECTURE block below):
 *   §1 CONFIG  §2 PERFORMANCE  §3 LOGIC  §4 SIMULATION
 *   §5 EFFECTS (none)  §6 DELAYS (none)  §7 RENDER  §8 APP
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Fractal terrain via layered (octave) Perlin noise (fBm).
 *                  Sum FBM_OCTAVES copies of the noise, each octave at double
 *                  the frequency and half the amplitude of the last, then
 *                  divide by the total amplitude to normalise:
 *                      h(x,y) = ( Σ_k Aᵏ·noise(fᵏ·x, fᵏ·y) ) / ( Σ_k Aᵏ )
 *                  with gain A = 0.5 and lacunarity f = 2. The lower-amplitude
 *                  high frequencies give the 1/f-like spectrum real terrain
 *                  height maps exhibit.
 *
 * Math           : Perlin noise (Ken Perlin, 1985):
 *                  - Divide space into integer grid cells
 *                  - Assign a pseudo-random gradient vector to each corner
 *                  - Dot gradient with offset from corner
 *                  - Smoothly interpolate using fade function 6t⁵−15t⁴+10t³
 *                    (C² continuous, 2nd derivative = 0 at t=0 and t=1)
 *                  With FBM_OCTAVES=5, lacunarity f=2, gain A=0.5 the Hurst
 *                  exponent is H = −ln(A)/ln(f) = 1 — smooth, persistent fBm
 *                  (Brownian motion proper is H=0.5; higher H = smoother).
 *
 * Rendering      : Camera scrolls left in time (the noise time parameter
 *                  increases), giving the illusion of flying over landscape.
 *                  Height bins map to terrain types (water/plains/hills/peaks).
 *
 * References     :
 *
 *   Noise & fractal terrain —
 *   • Perlin, K. (1985) — "An Image Synthesizer", SIGGRAPH '85. The
 *     original gradient-noise paper that started procedural noise.
 *   • Perlin, K. (2002) — "Improving Noise", SIGGRAPH '02. The exact
 *     gradient scheme + fade 6t⁵−15t⁴+10t³ implemented in §4 (perlin/fade).
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   • Ebert, Musgrave, Peachey, Perlin & Worley (2003) — "Texturing &
 *     Modeling: A Procedural Approach" (3rd ed.). Musgrave's chapters on
 *     fBm / octave-summed noise terrain — the canonical treatment of fbm().
 *   • Fournier, Fussell & Carpenter (1982) — "Computer rendering of
 *     stochastic models", CACM 25(6). Seminal fractal (1/f) terrain.
 *   • Mandelbrot, B. (1982) — "The Fractal Geometry of Nature". The
 *     power-law / self-similarity the 1/f height spectrum imitates.
 *
 *   Rendering —
 *   • Foley, van Dam, Feiner & Hughes — "Computer Graphics: Principles
 *     and Practice". Painter's algorithm: the back-to-front compositing
 *     (sky → far → mid → near) scene_draw() uses, each layer overwriting.
 *   • Parallax via differential layer speed — the multiplane camera
 *     (Iwerks / Disney, 1937): each plane scrolls at a fraction of camera
 *     speed (SCROLL_FAR/MID/NEAR) to fake depth without 3-D.
 *   • Imhof, E. (1982) — "Cartographic Relief Presentation". Hypsometric
 *     tinting: mapping elevation bands to a colour ramp (the per-layer
 *     terrain colours here).
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into concern-layers. All program state lives in
 * named types — PerlinNoise (the gradient-noise generator), the Landscape that
 * contains it plus the camera/knobs, and the Scene that wraps the Landscape
 * with the RENDER state (theme + terminal geometry). main() owns ONE Scene
 * local and threads it (or its Landscape) by the NARROWEST type each function
 * needs (const for a read, non-const to mutate). The ONLY globals left are the
 * two async signal flags, which a signal handler cannot reach any other way.
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time tunables, the Theme table, and the
 *                    PerlinNoise / Landscape / Scene type definitions.
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers.
 *                    The 30 fps frame cap is POLICY, applied in main() (§8).
 *   LOGIC        §3  nothing — pure reads: the noise core (lattice_index,
 *                    grad_dot, fade, lerp, perlin, fbm), is_star, and the
 *                    per-column/per-cell helpers (layer_row, column_horizons,
 *                    band_cell), taking const PerlinNoise* / const Landscape*.
 *                    Terrain is recomputed, never stored, so no RENDER/EFFECTS
 *                    ordering can corrupt them.
 *   SIMULATION   §4  Landscape.scroll (the per-tick advance, inline in main);
 *                    PerlinNoise.{perm,gx,gy} via noise_init; and the camera/
 *                    knobs via landscape_init / landscape_reset. Only writers.
 *   EFFECTS      §5  (none) — no stored cosmetic state; stars + shading are
 *                    derived each frame. One-line section, not a real layer.
 *   DELAYS       §6  (none) — only Landscape.paused, a toggle read in the tick.
 *   RENDER       §7  ncurses back buffer + colour-pair table only
 *                    (theme_apply, color_init, hud_draw, scene_draw). Reads the
 *                    Scene (theme, geometry, Landscape) via §3; never writes
 *                    simulation state. Scene.theme is set by the 't' key.
 *   APP          §8  g_quit / g_resize (signal flags); Scene.{rows,cols} on
 *                    resize; owns the Scene local + the per-frame combine, and
 *                    drives the knobs (speed, zoom, paused, theme) from keys.
 *
 * PER-FRAME COMBINE (the one place state advances — main(), §8; ld = &sc.landscape):
 *
 *     getch() → key handler          // USER EVENTS (knobs, reset, quit)
 *     if (!ld->paused) ld->scroll += // SIMULATION: one scroll step
 *     erase(); scene_draw(&sc)       // RENDER (reads only)
 *     wnoutrefresh(); doupdate()
 *     clock_sleep_ns(frame cap)      // PERFORMANCE: 30 fps cap
 *
 * Nothing else advances ld->scroll. Key handling and SIGWINCH resize mutate
 * state (knobs, geometry, noise field on reset) but are USER EVENTS, not part
 * of the scroll advance.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- tunables, theme data, core types (+ signal flags)      */
/* ===================================================================== */

#define RENDER_NS  (1000000000LL / 30)
#define HUD_TOP     2    /* rows 0-1: data  (title/state + parameters) */
#define HUD_BOTTOM  1    /* last row: actions (key legend)             */

/* Parallax: fraction of camera speed each layer scrolls at */
#define SCROLL_FAR    0.12f
#define SCROLL_MID    0.38f
#define SCROLL_NEAR   1.00f

/* Noise frequency: horizontal "stretch" of each landscape layer.
 * Smaller = wider features (smoother). */
#define BASE_FREQ_FAR   0.009f
#define BASE_FREQ_MID   0.018f
#define BASE_FREQ_NEAR  0.040f

/* fBm shape — how fbm() sums octaves of Perlin noise. Each octave runs at
 * frequency ×FBM_LACUNARITY and amplitude ×FBM_GAIN of the previous one; the
 * gain/lacunarity pair fixes the fractal roughness:
 *   Hurst exponent H = −ln(FBM_GAIN)/ln(FBM_LACUNARITY) = −ln(0.5)/ln(2) = 1
 * (smooth, persistent terrain). More octaves = finer detail, diminishing. */
#define FBM_OCTAVES     5
#define FBM_LACUNARITY  2.0f
#define FBM_GAIN        0.5f

/* Terrain vertical bands (fraction of draw_rows from top).
 * Peak floats up from BASE by up to AMP * noise. */
#define BASE_FAR   0.60f
#define AMP_FAR    0.28f
#define BASE_MID   0.70f
#define AMP_MID    0.22f
#define BASE_NEAR  0.80f
#define AMP_NEAR   0.16f

/* Each layer samples a different y-slice of the noise field so the three
 * profiles are uncorrelated (otherwise the ranges would echo each other). */
#define NOISE_YOFF_FAR   0.0f
#define NOISE_YOFF_MID   8.3f
#define NOISE_YOFF_NEAR  16.7f

/* Sky shading, as a fraction of the distance from the top down to the far
 * ridge: stars only twinkle in the upper part; the gradient darkens at the
 * halfway line. */
#define SKY_STAR_MAX     0.55f   /* above this fraction → no stars   */
#define SKY_SPLIT        0.50f   /* below → bright sky, above → dim  */

/* Rows of bright near-ground fill before it fades to the dark base tier. */
#define NEAR_FILL_ROWS   3

#define SPEED_MAX  4.f

#define PERM 256

/* ── Colour-pair indices ───────────────────────────────────────────────── *
 * One ncurses pair per painter's-algorithm layer, ordered FAR → NEAR exactly
 * as scene_draw() composites them (sky behind, near ground in front). The
 * order is the lesson: the 10 terrain slots double as indices into a Theme's
 * c[]/c8[] arrays (CP_SKY_HI−1 .. CP_NEAR_B−1 = slots 0..9), so a theme just
 * lists ten colours in this same depth order. Pairs start at 1 because ncurses
 * reserves pair 0 for the default fg/bg. */
enum {
    CP_SKY_HI = 1,   /* top sky        */
    CP_SKY_LO,       /* horizon sky    */
    CP_STAR,          /* stars          */
    CP_FAR_E,         /* far edge       */
    CP_FAR_F,         /* far fill       */
    CP_MID_E,         /* mid edge       */
    CP_MID_F,         /* mid fill       */
    CP_NEAR_E,        /* near edge      */
    CP_NEAR_F,        /* near fill      */
    CP_NEAR_B,        /* near base       */
    CP_HUD,           /* top data line   */
    CP_HINT           /* bottom actions  */
};

#define N_THEMES 10

/* ── Theme ─────────────────────────────────────────────────────────────── *
 * WHAT  One named colour palette (10 ship; t cycles). A theme supplies a
 *       foreground colour for each of the ten depth slots (the CP_* layers, in
 *       FAR→NEAR order) plus the HUD; theme_apply() loads them into the ncurses
 *       pairs. Background is ALWAYS -1 (the terminal default) so unpainted sky
 *       cells show through — the parallax illusion needs a single flat sky.
 *
 * VALUE LOGIC  c[0..9] runs sky_hi, sky_lo, star, far_e, far_f, mid_e, mid_f,
 *       near_e, near_f, near_b — i.e. distance-ordered. A good theme fades the
 *       far slots toward the sky colour (low contrast) and saturates the near
 *       ones, so depth reads as ATMOSPHERIC PERSPECTIVE: distant ranges look
 *       hazy, the foreground crisp. c8[] mirrors the same ten slots with the
 *       8 base COLOR_* constants for terminals without 256-colour support;
 *       hud / hud8 are the matching HUD foregrounds.
 *
 * REFS  Aerial / atmospheric perspective as a depth cue — Imhof,
 *       "Cartographic Relief Presentation" (1982). Project palette notes —
 *       documentation/COLOR.md. */
typedef struct {
    const char *name;   /* HUD label (t cycles)                              */
    int c[10];          /* 256-colour fg, one per CP_* slot, FAR→NEAR        */
    int c8[10];         /* 8-colour fallback for the same ten slots          */
    int hud;            /* 256-colour HUD foreground                         */
    int hud8;           /* 8-colour HUD foreground                           */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Classic",
      { 17,  18, 250,  24,  17,  22,  22,  34,  28,  22 },
      { COLOR_BLUE, COLOR_CYAN,  COLOR_WHITE, COLOR_BLUE,  COLOR_BLUE,
        COLOR_GREEN,COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
      226, COLOR_YELLOW },

    { "Matrix",
      { 16,  22,  46,  28,  22,  34,  28,  46,  34,  22 },
      { COLOR_BLACK,COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN,COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
       46, COLOR_GREEN },

    { "Nova",
      { 17,  18, 231,  39,  17,  21,  17,  51,  39,  21 },
      { COLOR_BLUE, COLOR_BLUE,  COLOR_WHITE, COLOR_CYAN,  COLOR_BLUE,
        COLOR_CYAN,  COLOR_BLUE, COLOR_CYAN,  COLOR_CYAN,  COLOR_BLUE },
       51, COLOR_CYAN },

    { "Poison",
      { 16,  22, 190, 100,  22, 148, 100, 190, 148, 100 },
      { COLOR_BLACK,COLOR_GREEN, COLOR_YELLOW,COLOR_GREEN, COLOR_GREEN,
        COLOR_YELLOW,COLOR_GREEN,COLOR_YELLOW,COLOR_YELLOW,COLOR_GREEN },
      154, COLOR_YELLOW },

    { "Ocean",
      { 17,  24, 231,  38,  18,  30,  24,  45,  38,  30 },
      { COLOR_BLUE, COLOR_CYAN,  COLOR_WHITE, COLOR_CYAN,  COLOR_BLUE,
        COLOR_CYAN,  COLOR_CYAN, COLOR_CYAN,  COLOR_CYAN,  COLOR_CYAN },
       39, COLOR_CYAN },

    { "Fire",
      { 52,  88, 226, 196,  88, 208, 196, 226, 214, 208 },
      { COLOR_RED,  COLOR_RED,   COLOR_YELLOW,COLOR_RED,   COLOR_RED,
        COLOR_YELLOW,COLOR_RED,  COLOR_YELLOW,COLOR_YELLOW,COLOR_RED },
      208, COLOR_YELLOW },

    { "Gold",
      { 17,  52, 231,  94,  52, 136,  94, 220, 136,  94 },
      { COLOR_BLUE, COLOR_RED,   COLOR_WHITE, COLOR_YELLOW,COLOR_RED,
        COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW },
      226, COLOR_YELLOW },

    { "Ice",
      { 16,  17, 231,  30,  17,  23,  17, 159,  30,  23 },
      { COLOR_BLACK,COLOR_BLUE,  COLOR_WHITE, COLOR_CYAN,  COLOR_BLUE,
        COLOR_BLUE,  COLOR_BLUE, COLOR_CYAN,  COLOR_CYAN,  COLOR_BLUE },
      123, COLOR_CYAN },

    { "Nebula",
      { 16,  54, 183,  93,  54,  55,  54, 141,  93,  55 },
      { COLOR_BLACK,COLOR_MAGENTA,COLOR_WHITE,COLOR_MAGENTA,COLOR_MAGENTA,
        COLOR_MAGENTA,COLOR_MAGENTA,COLOR_CYAN,COLOR_MAGENTA,COLOR_MAGENTA },
       87, COLOR_CYAN },

    { "Lava",
      { 52,  88, 226, 124,  52, 196, 124, 214, 196, 124 },
      { COLOR_RED,  COLOR_RED,   COLOR_YELLOW,COLOR_RED,   COLOR_RED,
        COLOR_RED,   COLOR_RED,  COLOR_YELLOW,COLOR_RED,   COLOR_RED },
      214, COLOR_YELLOW },
};

/* ── data types ──────────────────────────────────────────────────────── */

/* ── PerlinNoise ───────────────────────────────────────────────────────── *
 * WHAT  A gradient-noise generator. Perlin noise assigns every integer lattice
 *       point a pseudo-random gradient vector, then for a sample (x,y) takes
 *       the dot product of each surrounding corner's gradient with the offset
 *       to that corner and smoothly blends the four — giving a continuous field
 *       that is 0 at the lattice and varies coherently between (no white-noise
 *       hash, no visible grid). fbm() stacks octaves of it into terrain.
 *
 * WHY THIS LAYOUT
 *   perm[]  A permutation of 0..255 (a hash): perm[perm[xi]+yi] turns a 2-D
 *           lattice coordinate into a stable index 0..255 with no multiply.
 *           It is built 0..255, Fisher–Yates shuffled, then the SAME 256 values
 *           are copied into [256..511]. That duplication is the trick that lets
 *           perlin() write perm[perm[xi]+yi] for neighbours xi, xi+1 (indices
 *           up to 255+255=510) without a modulo or bounds test — the upper half
 *           mirrors the lower, so a wrapped index reads the same gradient.
 *   gx,gy   The unit gradient at each entry, gx=cos θ, gy=sin θ for a random
 *           angle θ — kept parallel to perm[] (shuffled and doubled in lockstep)
 *           so a permuted index selects both a hash slot and its gradient.
 *           (Perlin's "Improving Noise" uses a fixed set of 12 gradients to
 *           avoid directional clustering; this file takes the simpler route of
 *           a continuous random direction per entry.)
 *
 * REFS  Perlin, "An Image Synthesizer" (1985) and "Improving Noise" (2002,
 *       the quintic fade 6t⁵−15t⁴+10t³ used in fade()). See the file-header
 *       References block. */
typedef struct {
    unsigned char perm[PERM * 2];   /* shuffled 0..255 hash, mirrored into the
                                       upper half so lookups wrap modulo-free */
    float         gx[PERM * 2];     /* gradient x = cos θ (unit), per entry   */
    float         gy[PERM * 2];     /* gradient y = sin θ (unit), per entry   */
} PerlinNoise;

/* ── Landscape ─────────────────────────────────────────────────────────── *
 * WHAT  The simulated world — a moving camera over a procedural terrain — as a
 *       table of contents (WHAT / WHERE / HOW + run-state). Only SIMULATION (§4)
 *       writes it; LOGIC/RENDER read it const. It is a sub-object of the Scene
 *       that main() owns, threaded down as &scene.landscape — no hidden global
 *       terrain.
 *
 * WHY NO STORED TERRAIN  The heightfield is never materialised: scene_draw()
 *       calls layer_row() per column, which samples fbm(&noise, scroll·frac +
 *       col·freq·zoom, …) on demand. So "advancing the world" is just moving
 *       the camera — incrementing one float — and the parallax depth comes from
 *       each layer reading the noise at a different fraction of `scroll`.
 *
 * VALUE LOGIC
 *   noise   the gradient field (above); fixed until 'r' regenerates it.
 *   scroll  camera x in terminal columns, a free-running accumulator (+= speed
 *           each unpaused frame); large values are fine — perlin() wraps.
 *   speed   columns/frame, in [−SPEED_MAX .. SPEED_MAX] (±4); sign is travel
 *           direction, magnitude is pace; ±0.1 per arrow-key press.
 *   zoom    horizontal stretch multiplying every layer's base frequency,
 *           in [0.125 .. 8] (×1.25 / ×0.8 per +/−); <1 = wider, smoother hills.
 *   paused  freezes the scroll advance only; rendering (and the clock) go on.
 *
 *   The colour theme is deliberately NOT a field here: it is a RENDER choice,
 *   not a property of the landscape (it lives in the Scene that wraps this).
 *
 * REFS  fBm terrain — Ebert/Musgrave et al., "Texturing & Modeling" (2003);
 *       parallax-by-differential-scroll — the multiplane camera (Disney, 1937).
 *       See the file-header References block. */
typedef struct {
    PerlinNoise noise;     /* terrain source (sampled, never stored)    */
    float       scroll;    /* camera x-position, in columns (accumulates)*/
    float       speed;     /* scroll columns/frame, [−SPEED_MAX..MAX]    */
    float       zoom;      /* base-frequency multiplier, [0.125..8]      */
    bool        paused;    /* freeze the scroll advance (render goes on) */
} Landscape;

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * The top-level state: WHAT is simulated (the Landscape) plus the RENDER state
 * that decides HOW/WHERE it is shown — the active colour theme and the current
 * terminal geometry. main() owns ONE Scene and threads it (or its Landscape
 * sub-object) to the functions that need it, so none of this is a global.
 * theme and rows/cols are kept OUT of the Landscape on purpose: a key toggles
 * the theme and a resize sets rows/cols — neither is a property of the terrain. */
typedef struct {
    Landscape landscape;   /* WHAT — the simulated world (§4)              */
    int       theme;       /* RENDER — active palette, index into k_themes */
    int       rows, cols;  /* RENDER — terminal size (set at init + resize)*/
} Scene;

/* The ONLY remaining globals: async signal flags. They must be global because
 * the signal handler (sig_h, §8) takes no parameter; all other state now lives
 * in the Scene that main() owns. */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ===================================================================== */
/* §2  PERFORMANCE  -- timing primitives (frame cap in main, §8)         */
/* ===================================================================== */

/* Timing primitives only. The 30 fps frame cap (RENDER_NS) is applied in
 * main() (§8): this program advances exactly one scroll step per frame, so
 * a simple frame cap suffices — no fixed-timestep accumulator. */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  LOGIC  -- pure decisions: no mutation, no I/O                     */
/* ===================================================================== */

/* Pure reads: each returns a value from its arguments alone — a const
 * PerlinNoise* (perlin/fbm), a const Landscape* (layer_row, column_horizons),
 * or plain scalars (band_cell, is_star) — with NO mutation and NO I/O. The
 * terrain is never stored; layer_row() recomputes a column on demand, so no
 * RENDER/EFFECTS ordering can corrupt a result. Only SIMULATION (§4) ever
 * writes the data these read. */

static float fade(float t) { return t * t * t * (t * (t * 6.f - 15.f) + 10.f); }
static float lerp(float a, float b, float t) { return a + t * (b - a); }

/* Integer lattice cell containing v, wrapped into the 0..255 table (the
 * `& (PERM-1)` is a fast modulo-256 since PERM is a power of two). */
static int lattice_index(float v) { return (int)floorf(v) & (PERM - 1); }

/* The heart of gradient noise: the corner's gradient vector dotted with the
 * offset (dx,dy) from that corner to the sample point. Zero at the corner,
 * rising along the gradient direction — this is what gives Perlin noise its
 * smooth ramps instead of value-noise blockiness. */
static float grad_dot(const PerlinNoise *n, int hash, float dx, float dy)
{
    return n->gx[hash] * dx + n->gy[hash] * dy;
}

static float perlin(const PerlinNoise *n, float x, float y)
{
    /* lattice cell + the sample's offset within it, with smoothstep weights */
    int   xi = lattice_index(x), yi = lattice_index(y);
    float xf = x - floorf(x),    yf = y - floorf(y);
    float u  = fade(xf),         v  = fade(yf);

    /* hash the four surrounding corners to gradient-table entries */
    int aa = n->perm[n->perm[xi  ] + yi  ];
    int ab = n->perm[n->perm[xi  ] + yi+1];
    int ba = n->perm[n->perm[xi+1] + yi  ];
    int bb = n->perm[n->perm[xi+1] + yi+1];

    /* bilinearly blend each corner's gradient·offset ramp */
    return lerp(lerp(grad_dot(n, aa, xf,       yf      ),
                     grad_dot(n, ba, xf - 1.f, yf      ), u),
                lerp(grad_dot(n, ab, xf,       yf - 1.f),
                     grad_dot(n, bb, xf - 1.f, yf - 1.f), u), v);
}

/* Fractal Brownian Motion: sum FBM_OCTAVES octaves of Perlin noise, each at
 * ×FBM_LACUNARITY frequency and ×FBM_GAIN amplitude of the last. Dividing by
 * the total amplitude m normalises the result to ≈ [-0.5, 0.5] — and that
 * normalisation makes the STARTING amplitude (a below) irrelevant: scale every
 * term by the same constant and it cancels in v/m, so 0.6 vs 1.0 is identical.
 * Only the gain/lacunarity RATIOS shape the terrain (see FBM_* in §1). */
static float fbm(const PerlinNoise *n, float x, float y)
{
    float v = 0.f, a = 0.6f, f = 1.f, m = 0.f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        v += a * perlin(n, x * f, y * f);
        m += a; a *= FBM_GAIN; f *= FBM_LACUNARITY;
    }
    return v / m;   /* normalised to ≈ [-0.5, 0.5] */
}

/* Deterministic star hash — no storage, no flicker */
static bool is_star(int col, int row)
{
    unsigned h = (unsigned)col * 2654435761u ^ (unsigned)row * 2246822519u;
    return (h & 0x1FF) < 3;   /* ~0.6% density */
}

/* Sample one parallax layer's peak row for this column. Reads the landscape's
 * noise field + camera scroll + zoom (three cohesive fields = "the moving
 * terrain"), so it takes const Landscape* rather than four loose scalars; it
 * is a pure read and cannot mutate. yoffset keeps layers independent in noise
 * space; scroll_frac is the layer's parallax factor. */
static int layer_row(const Landscape *ld, float scroll_frac, float base_freq,
                     float base, float amp,
                     float col, float yoffset, int draw_rows)
{
    /* Where this column samples the noise: parallax shifts the layer by a
     * fraction of the camera scroll; zoom stretches the per-column frequency. */
    float sample_x = ld->scroll * scroll_frac + col * base_freq * ld->zoom;
    float n        = fbm(&ld->noise, sample_x, yoffset);   /* ≈ [-0.5, 0.5] */

    /* Map noise to a screen row: `base` is the layer's resting line (fraction
     * from the top), `amp` how far its ridges rise above it. */
    float top_frac = base - (n + 0.5f) * amp;
    int   row      = (int)(top_frac * (float)draw_rows);
    if (row < 0)          row = 0;
    if (row >= draw_rows) row = draw_rows - 1;
    return row;
}

/* The three ridge rows (far, mid, near) for a column, then forced into depth
 * order: each nearer ridge must sit at least one row BELOW the one behind it,
 * so the parallax layers can never visually invert. Pure read of the scene. */
static void column_horizons(const Landscape *ld, int col, int draw_rows,
                            int *rf, int *rm, int *rn)
{
    *rf = layer_row(ld, SCROLL_FAR,  BASE_FREQ_FAR,  BASE_FAR,  AMP_FAR,
                    (float)col, NOISE_YOFF_FAR,  draw_rows);
    *rm = layer_row(ld, SCROLL_MID,  BASE_FREQ_MID,  BASE_MID,  AMP_MID,
                    (float)col, NOISE_YOFF_MID,  draw_rows);
    *rn = layer_row(ld, SCROLL_NEAR, BASE_FREQ_NEAR, BASE_NEAR, AMP_NEAR,
                    (float)col, NOISE_YOFF_NEAR, draw_rows);

    if (*rm < *rf + 1) *rm = *rf + 1;
    if (*rn < *rm + 1) *rn = *rm + 1;
    if (*rf >= draw_rows) *rf = draw_rows - 1;
    if (*rm >= draw_rows) *rm = draw_rows - 1;
    if (*rn >= draw_rows) *rn = draw_rows - 1;
}

/* Glyph + colour pair for one cell, decided purely by where its row sits
 * relative to the three ridge rows: sky above the far ridge (with stars and a
 * top-to-horizon gradient), then a lit edge '^'/'~' at each ridge and a fill
 * below it, near→far. Returns the glyph and writes the colour pair via *cp. */
static chtype band_cell(int r, int rf, int rm, int rn, int col, int *cp)
{
    if (r < rf) {                                    /* ── sky ── */
        float sky_t = (float)r / (float)(rf > 0 ? rf : 1);
        if (sky_t < SKY_STAR_MAX && is_star(col, r)) { *cp = CP_STAR; return '.'; }
        *cp = (sky_t < SKY_SPLIT) ? CP_SKY_HI : CP_SKY_LO;
        return ' ';
    }
    if (r == rf) { *cp = CP_FAR_E;  return '^'; }    /* far ridge edge   */
    if (r < rm)  { *cp = CP_FAR_F;  return (r == rf + 1) ? ':' : '.'; }
    if (r == rm) { *cp = CP_MID_E;  return '^'; }    /* mid ridge edge   */
    if (r < rn)  { *cp = CP_MID_F;  return (r == rm + 1) ? ':' : '#'; }
    if (r == rn) { *cp = CP_NEAR_E; return '~'; }    /* near ground edge */
    *cp = (r < rn + NEAR_FILL_ROWS) ? CP_NEAR_F : CP_NEAR_B;   /* darker base */
    return '#';
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances / builds state                            */
/* ===================================================================== */

/* The only writers of simulation state. The per-tick advance is a single line,
 *   if (!ld->paused) ld->scroll += ld->speed;   inside main()`s combine (§8) —
 * too trivial for its own function. noise_init() builds the gradient field;
 * landscape_init/landscape_reset set the camera + knobs (startup vs the 'r'
 * key, a USER EVENT). Mutates: Landscape.{scroll(tick), speed, zoom, paused},
 * PerlinNoise.{perm, gx, gy}. */

/* Fill a PerlinNoise with a fresh random gradient field: assign each lattice
 * point a unit gradient at a random angle, Fisher–Yates shuffle the table,
 * then duplicate into the upper half so perlin()'s lookups wrap freely. */
static void noise_init(PerlinNoise *n)
{
    /* 1. identity permutation + a random unit gradient (cos θ, sin θ) per entry */
    for (int i = 0; i < PERM; i++) {
        n->perm[i] = (unsigned char)i;
        float a = (float)rand() / RAND_MAX * 2.f * (float)M_PI;
        n->gx[i] = cosf(a);
        n->gy[i] = sinf(a);
    }
    /* 2. Fisher–Yates shuffle — perm and its paired gradients move together */
    for (int i = PERM - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        unsigned char t = n->perm[i]; n->perm[i] = n->perm[j]; n->perm[j] = t;
        float tx = n->gx[i]; n->gx[i] = n->gx[j]; n->gx[j] = tx;
        float ty = n->gy[i]; n->gy[i] = n->gy[j]; n->gy[j] = ty;
    }
    /* 3. mirror the lower half into the upper so perlin()'s lookups wrap free */
    for (int i = 0; i < PERM; i++) {
        n->perm[i + PERM] = n->perm[i];
        n->gx[i + PERM] = n->gx[i];
        n->gy[i + PERM] = n->gy[i];
    }
}

/* Reset the camera + knobs to defaults and regenerate the terrain. Leaves
 * `paused` untouched (the 'r' key resets the world without un-pausing). */
static void landscape_reset(Landscape *ld)
{
    ld->scroll = 0.f;
    ld->speed  = 0.6f;
    ld->zoom   = 1.0f;
    noise_init(&ld->noise);
}

/* Full startup init: a clean (running) landscape. */
static void landscape_init(Landscape *ld)
{
    ld->paused = false;
    landscape_reset(ld);
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. There is no stored cosmetic state: the stars are a pure
 * positional hash (is_star, §3) and every terrain glyph/colour is derived at
 * render time from the noise field — nothing is kept between frames to glow,
 * trail or fade. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No DELAYS layer to speak of: the only timing state is Landscape.paused, a
 * single toggle checked in main()`s tick (§8). No holds, countdowns or timers;
 * the frame cap is PERFORMANCE (§2). */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. Reads the Scene (its Landscape, theme and geometry),
 * sampling terrain via §3 LOGIC; writes ONLY the ncurses back buffer and the
 * colour-pair table. Never mutates simulation state. */

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    /* All terrain pairs use -1 background so the sky stays transparent */
    if (COLORS >= 256) {
        init_pair(CP_SKY_HI, th->c[0], -1);
        init_pair(CP_SKY_LO, th->c[1], -1);
        init_pair(CP_STAR,   th->c[2], -1);
        init_pair(CP_FAR_E,  th->c[3], -1);
        init_pair(CP_FAR_F,  th->c[4], -1);
        init_pair(CP_MID_E,  th->c[5], -1);
        init_pair(CP_MID_F,  th->c[6], -1);
        init_pair(CP_NEAR_E, th->c[7], -1);
        init_pair(CP_NEAR_F, th->c[8], -1);
        init_pair(CP_NEAR_B, th->c[9], -1);
        init_pair(CP_HUD,    th->hud,  -1);
    } else {
        init_pair(CP_SKY_HI, th->c8[0], -1);
        init_pair(CP_SKY_LO, th->c8[1], -1);
        init_pair(CP_STAR,   th->c8[2], -1);
        init_pair(CP_FAR_E,  th->c8[3], -1);
        init_pair(CP_FAR_F,  th->c8[4], -1);
        init_pair(CP_MID_E,  th->c8[5], -1);
        init_pair(CP_MID_F,  th->c8[6], -1);
        init_pair(CP_NEAR_E, th->c8[7], -1);
        init_pair(CP_NEAR_F, th->c8[8], -1);
        init_pair(CP_NEAR_B, th->c8[9], -1);
        init_pair(CP_HUD,    th->hud8,  -1);
    }
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    theme_apply(theme);
    /* Action legend stays a fixed bright cyan across all themes (HUD
     * standard); theme_apply() never touches it, so set it once here. */
    init_pair(CP_HINT, (COLORS >= 256) ? 51 : COLOR_CYAN, -1);
}

/* Draw s at (row,col) in pair|attr, clipped to `cols` (the terminal width) so
 * a long line can never wrap onto the next row and corrupt the display. */
static void hud_draw(int cols, int row, int col, int pair, int attr, const char *s)
{
    if (col < 0) col = 0;
    int avail = cols - col;
    if (avail <= 0) return;
    char buf[256];
    snprintf(buf, sizeof buf, "%s", s);
    if ((int)strlen(buf) > avail) buf[avail] = '\0';   /* truncate, no wrap */
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, col, "%s", buf);
    attroff(COLOR_PAIR(pair) | attr);
}

/* The HUD overlay: top two rows = data, bottom row = actions (project
 * standard). Every line is width-clipped by hud_draw(). */
static void draw_hud(const Scene *sc)
{
    int cols = sc->cols;

    /* Row 0 left — title (bold so row 0 stays dominant). */
    hud_draw(cols, 0, 1, CP_HUD, A_BOLD, " PARALLAX LANDSCAPE ");

    /* Row 0 right — run state, right-aligned. */
    const char *state = sc->landscape.paused ? " PAUSED " : " scrolling ";
    hud_draw(cols, 0, cols - (int)strlen(state), CP_HUD, A_BOLD, state);

    /* Row 1 — parameters (no bold). */
    char buf[256];
    snprintf(buf, sizeof buf,
             " scroll:%.0f  speed:%+.2f  zoom:%.1fx  theme:%s ",
             (double)sc->landscape.scroll, (double)sc->landscape.speed,
             (double)sc->landscape.zoom, k_themes[sc->theme].name);
    hud_draw(cols, 1, 0, CP_HUD, A_NORMAL, buf);

    /* Bottom row — action / key legend. */
    hud_draw(cols, sc->rows - 1, 0, CP_HINT, A_BOLD,
             " q:quit  p:pause  r:reset  <-/->:speed/dir  +/-:zoom  t:theme ");
}

/* Repaint the whole frame: for every column find its three parallax ridges,
 * then paint each cell by the band it falls in (painter's algorithm, far→near
 * already baked into the band order); finally lay the HUD over the top. */
static void scene_draw(const Scene *sc)
{
    int draw_rows = sc->rows - HUD_TOP - HUD_BOTTOM;
    if (draw_rows < 1) draw_rows = 1;

    for (int col = 0; col < sc->cols; col++) {
        int rf, rm, rn;
        column_horizons(&sc->landscape, col, draw_rows, &rf, &rm, &rn);

        for (int r = 0; r < draw_rows; r++) {
            int cp;
            chtype ch = band_cell(r, rf, rm, rn, col, &cp);
            attron(COLOR_PAIR(cp));
            mvaddch(r + HUD_TOP, col, ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    draw_hud(sc);
}

/* ===================================================================== */
/* §8  APP  -- signals, lifecycle, per-frame combine loop                */
/* ===================================================================== */

/* Owns the signal flags, terminal lifecycle and the main loop. main() is the
 * ONE place that combines the layers per frame, in fixed order (see the
 * ARCHITECTURE block). User events — key handling and SIGWINCH resize —
 * mutate state but are NOT part of the per-frame advance. */

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    Scene sc;
    sc.theme = 0;
    color_init(sc.theme);
    getmaxyx(stdscr, sc.rows, sc.cols);
    landscape_init(&sc.landscape);

    Landscape *ld = &sc.landscape;   /* alias for the per-frame combine below */

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, sc.rows, sc.cols);
        }

        /* USER EVENTS — mutate state but are NOT part of the scroll advance. */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': ld->paused = !ld->paused; break;
        case 'r': case 'R': landscape_reset(ld); break;
        case KEY_RIGHT:
            ld->speed += 0.1f; if (ld->speed > SPEED_MAX) ld->speed = SPEED_MAX; break;
        case KEY_LEFT:
            ld->speed -= 0.1f; if (ld->speed < -SPEED_MAX) ld->speed = -SPEED_MAX; break;
        case '+': case '=':
            ld->zoom *= 1.25f; if (ld->zoom > 8.f) ld->zoom = 8.f; break;
        case '-':
            ld->zoom *= 0.8f; if (ld->zoom < 0.125f) ld->zoom = 0.125f; break;
        case 't': case 'T':
            sc.theme = (sc.theme + 1) % N_THEMES;
            theme_apply(sc.theme);
            break;
        default: break;
        }

        long long now = clock_ns();

        if (!ld->paused) ld->scroll += ld->speed;   /* SIMULATION: one step */

        erase();
        scene_draw(&sc);                            /* RENDER (reads only) */
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
