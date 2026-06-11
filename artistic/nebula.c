/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nebula.c — drifting nebula with star births and shock illumination
 *
 * DEMO: A multi-octave fBm scalar field defines the density and tint
 *       of a glowing gas cloud that fills the screen. The field
 *       slowly scrolls across two parallax layers — a near layer
 *       (high frequency, dim) and a far layer (low frequency, brighter)
 *       — giving an illusion of cosmic depth. A static catalogue of
 *       background stars sparkles. Periodically (every ~5–8 seconds)
 *       a NEW STAR is born somewhere inside the densest gas: a sharp
 *       flash, then a slow expanding spherical shock-wave illuminates
 *       the surrounding gas for several seconds before fading.
 *
 *       The result is a still, vast, slowly evolving cosmic scene with
 *       occasional dramatic moments — a star nursery seen from a
 *       distant telescope.
 *
 * Study alongside: artistic/galaxy.c (rotating disc — opposite scale),
 *                  fluid/wave_2d.c (the radial wave-front under the
 *                  shock illumination uses similar 1/r decay),
 *                  fractal_random/perlin_landscape.c (multi-octave fBm).
 *
 * Section map (layers):
 *   §1 config            — fBm/parallax/star/shock constants, colour-pair IDs
 *   §2 performance       — monotonic clock + sleep (frame cap in §7)
 *   §3 simulation state  — Star / Shock / Nebula types
 *   §4 logic             — value-noise + fBm, gas bucketing, shock illumination
 *   §5 simulation        — PRNG, seed/reseed, shock spawn, per-tick advance
 *   §6 render            — palette, stars, fBm raster, shocks, HUD, ncurses
 *   §7 app               — signals, the per-tick combine loop, key handling
 *
 * Keys:  [/]   star count (50..400)
 *        -/+   nebula brightness threshold
 *        ,/.   parallax scroll speed
 *        b     trigger a star birth on demand
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/nebula.c \
 *       -o nebula -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : The gas is a 2-D fBm field — sum of OCTAVES value-noise
 *                 layers at doubling frequency and halving amplitude.
 *                 Two layers are sampled and added: a "near" layer at
 *                 fine spatial scale, and a "far" layer at coarse scale,
 *                 each scrolled at a different speed (`near.speed` >
 *                 `far.speed`). The combined field controls colour
 *                 (deep gas → cool, sparse → dark) and glyph density
 *                 (Bourke ramp, indexed by field magnitude).
 *
 *                 Stars are a fixed catalogue: random (x, y, brightness,
 *                 phase) generated at startup; rendered each frame with
 *                 a slow sinusoidal twinkle.
 *
 *                 Star births are events with a small expanding shock-
 *                 wave: each Shock has (x, y, age). A radius `r =
 *                 SHOCK_SPEED · age` advances; cells whose distance to
 *                 (x, y) is near `r` get extra brightness. Older shocks
 *                 fade. New shocks spawn periodically + on user demand.
 *
 * Data-structure: Nebula holds Star[N_STARS_MAX] and Shock[N_SHOCKS_MAX]
 *                 inline. fBm is evaluated on demand per cell each frame
 *                 — O(rows · cols · OCTAVES) per frame. At 80×24×4 ≈ 7k
 *                 evaluations, each ~10 multiplies — easily 30 fps.
 *
 * Rendering     : Per frame: fBm raster (every cell), then star catalogue,
 *                 then shocks (additive brightness over the gas). HUD
 *                 last.
 *
 * References    :
 *   Noise & fBm gas field (§4 fbm / value_noise, §6 raster)
 *     [1] Perlin, "An Image Synthesizer," SIGGRAPH (1985) — procedural noise;
 *         this file uses cheap value-noise in the same role.
 *     [2] Mandelbrot, "The Fractal Geometry of Nature" (1982) — fractional
 *         Brownian motion (fBm), the summed-octaves field that is the gas.
 *     [3] Musgrave, Kolb & Mace, "The Synthesis and Rendering of Eroded
 *         Fractal Terrains," SIGGRAPH (1989) — fBm gain/lacunarity choices.
 *     [4] Ebert, Musgrave, Peachey, Perlin & Worley, "Texturing & Modeling:
 *         A Procedural Approach" (Morgan Kaufmann) — value noise, octave
 *         layering, and using fBm for clouds/gas.
 *   Star birth — diffraction spikes & glow (§4 shock_brightness_at, §6 scene_draw)
 *     [5] Spencer, Shirley, Zimmerman & Greenberg, "Physically-Based Glare
 *         Effects for Digital Images," SIGGRAPH (1995) — why a bright point
 *         source shows a 4-pointed diffraction starburst + halo (the newborn).
 *   ASCII rendering substrate (§4 gas_bucket, §6 GAS_GLYPH)
 *     [6] Bourke, "Character representation of grey scale images" (1997) — the
 *         brightness → glyph ramp ('.' : ~ o #).
 *     [7] Padala, "NCURSES Programming HOWTO" (TLDP) — colour pairs and the
 *         erase → draw → refresh frame model.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── ARCHITECTURE (layer separation) ───────────────────────────────────── *
 *
 * Four real layers; two are intentionally absent.
 *
 *   LAYER        SECTION  MUTATES
 *   ────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — clock primitives (frame cap / fps in §7)
 *   SIMULATION   §3,§5    §3 holds the state; §5 advances it: the near/far
 *                           ParallaxLayer offsets and world_time (drift),
 *                           shocks[] (spawn /
 *                           age / plant star), stars[]/n_stars, next_shock_in;
 *                           advances the global PRNG (frand / rand).
 *                           (threshold, scroll speeds, theme, paused, n_stars
 *                           are changed by USER EVENTS in §7, not by the tick.)
 *   LOGIC        §4       nothing — pure: value_noise/fbm/hash01/smoothstep
 *                           (gas field), gas_bucket (quantise), and
 *                           shock_brightness_at (illumination from shock ages).
 *                           Reads state, returns values; no mutation, no I/O.
 *   RENDER       §6       the terminal + colour pairs only; READS the Nebula,
 *                           never writes it.
 *
 *   EFFECTS  : ABSENT — star twinkle and shock glow are derived at render time
 *              from world_time / shock age (star_draw, shock_brightness_at),
 *              never stored as separate cosmetic state.
 *   DELAYS   : the only pause is Nebula.paused (checked at the combine point);
 *              the shock-cadence timer next_shock_in is simulation state inside
 *              nebula_tick, not a separate module.
 *
 * LOGIC (§4) is provably uncorruptable from RENDER: it does no mutation and no
 * I/O, so deleting or reordering any draw cannot change fbm / shock_brightness.
 *
 * Per-tick combine order — main() (§7) is the only place that advances state:
 *     1. PERFORMANCE  measure dt (capped at DT_CAP_S)       §7
 *     2. SIMULATION   nebula_tick(dt)  [skipped if paused]  §5
 *     3. PERFORMANCE  smoothed fps                          §7
 *     4. RENDER       scene_draw()  (read-only)             §6
 *     5. PERFORMANCE  sleep to the frame cap                §7
 *
 * User events (keys: stars/thresh/scroll/birth/theme/reset/pause; SIGWINCH)
 * may mutate the Nebula / screen but are NOT part of the tick — handled before
 * it in §7 (the getch drain and resize block; 'b'/'r' call shock_spawn /
 * nebula_reseed from §5).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS         30        /* slow scene; 30 fps is plenty       */

/* fBm */
#define FBM_OCTAVES         4
#define FBM_FREQ_NEAR       0.10f
#define FBM_FREQ_FAR        0.04f
#define FBM_LACUNARITY      2.0f
#define FBM_GAIN            0.55f

/* Parallax scroll speeds — cell-units per second. */
#define SCROLL_NEAR_DEFAULT 0.6f
#define SCROLL_FAR_DEFAULT  0.18f
#define SCROLL_MIN          0.0f
#define SCROLL_MAX          3.0f
#define SCROLL_STEP         0.1f

/* Stars */
#define N_STARS_MAX         400
#define N_STARS_DEFAULT     180
#define N_STARS_MIN          50
#define N_STARS_STEP         50

/* Shocks (star births) */
#define N_SHOCKS_MAX        12
#define SHOCK_INTERVAL_MIN   4.0f
#define SHOCK_INTERVAL_MAX   9.0f
#define SHOCK_LIFE           5.0f
#define SHOCK_SPEED          5.0f      /* cells/sec radius growth          */
#define SHOCK_THICKNESS      1.6f
#define SHOCK_FLASH_DUR      0.3f      /* age < this: 3×3 white-hot flash   */
#define SHOCK_SPIKE_DUR      1.5f      /* age < this: 4-point spikes; also
                                        * the age the permanent star is planted */
#define SHOCK_SAMPLES        12        /* candidate cells sampled per birth */

/* Nebula brightness threshold — below this the gas is invisible. */
#define THRESH_DEFAULT       0.45f
#define THRESH_MIN           0.20f
#define THRESH_MAX           0.85f
#define THRESH_STEP          0.05f

#define ASPECT_X             2.0f
#define SCROLL_VDRIFT        0.25f     /* vertical drift = this × layer speed */

/* Gas-field blend (the two parallax layers) + shock illumination weight. */
#define GAS_NEAR_W           0.55f     /* weight of the near parallax layer   */
#define GAS_FAR_W            0.45f     /* weight of the far parallax layer    */
#define SHOCK_BOOST          0.5f      /* shock light folded into the gas     */

#define DT_CAP_S             0.10f
#define N_THEMES             4

/* Colour pairs */
#define PAIR_GAS_0    1   /* coolest gas (e.g. deep blue)                 */
#define PAIR_GAS_1    2
#define PAIR_GAS_2    3
#define PAIR_GAS_3    4
#define PAIR_GAS_4    5   /* hottest gas (white-pink)                     */
#define PAIR_STAR     6
#define PAIR_SHOCK    7
#define PAIR_HUD      8
#define PAIR_HINT     9

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  performance — timing primitives (frame cap applied in §7)           */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  simulation state — the data SIMULATION owns (mutated only by §5)     */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Star — one entry in the background catalogue: a fixed cell that twinkles.
 *
 * WHY a static catalogue: the starfield is a cheap, unchanging backdrop — each
 * star is just a position plus a sinusoidal twinkle, so hundreds cost almost
 * nothing and never need simulating.  Newborn stars (planted by a settling
 * Shock) are appended to the SAME pool, so a birth leaves a permanent mark.
 *
 * VALUE LOGIC: rendered brightness = brightness × (0.7 + 0.3·sin(t·1.5 + phase));
 * the per-star `phase` decorrelates the twinkle so the sky shimmers rather than
 * pulsing in unison. */
typedef struct {
    int   x, y;            /* fixed integer cell                            */
    float brightness;      /* base magnitude 0.3..1.0 (newborns ≈ 1.0)      */
    float phase;           /* 0..2π — per-star twinkle offset (decorrelates) */
    int   alive;           /* 0 = empty pool slot                           */
} Star;

/* Shock — a star-birth event: an expanding wavefront that lights the gas and,
 * once it settles, leaves a permanent Star behind.  Lives SHOCK_LIFE seconds.
 *
 * WHY one `age` drives everything: a star birth is inherently a timeline, so
 * rather than store per-effect state we keep only seconds-since-birth and DERIVE
 * the rest — the ring radius (SHOCK_SPEED·age), the three illumination layers
 * (ring / diffraction spikes / halo, §4 shock_brightness_at) and the three
 * render phases (flash → 4-point spikes → settle, §6).  The spikes + halo of a
 * bright point source follow the glare model of Spencer et al. (ref [5]).
 *
 * CONTEXT: x,y are WORLD space and do NOT scroll — the explosion is anchored to
 * its birthplace while the gas drifts past.  star_added is a one-shot latch: at
 * age ≥ 1.5 s it plants the permanent Star, then is never acted on again. */
typedef struct {
    float x, y;            /* birthplace cell (world space — does NOT scroll)  */
    float age;             /* seconds since birth — drives radius + every phase */
    int   alive;           /* 0 = empty pool slot                              */
    int   star_added;      /* one-shot latch: 1 once the star has been planted */
} Shock;

/* ParallaxLayer — one scrolling sample of the fBm gas (Mandelbrot, ref [2]).
 *
 * WHY two layers: motion parallax — nearer things sweep past faster.  Sampling
 * the same noise at two scales and scrolling them at different speeds (near =
 * fine & fast, far = coarse & slow), then summing, fakes cosmic depth on a flat
 * grid.
 *
 * HOW it scrolls: the offset (ox, oy) is ADDED to the cell coordinates before
 * sampling fbm — the field slides under a fixed screen; nothing is moved in
 * memory.  Each tick advances ox by speed·ASPECT_X (cells are ~2× tall, so x is
 * boosted to keep the drift visually isotropic) and oy by a gentler speed·0.25. */
typedef struct {
    float ox, oy;          /* accumulated scroll offset, cells (advanced/tick) */
    float speed;           /* drift speed, cells/s (user-tunable knob)         */
} ParallaxLayer;

/* Nebula — the whole cosmic scene, one aggregate read as a table of contents.
 *
 * WHY fixed pools + on-demand field: stars[]/shocks[] are sized to their maxima
 * once and never grown or freed (the `alive` flags mark live slots), and the
 * gas is re-evaluated per cell every frame (fbm, ref [4]) with NO stored grid —
 * so the whole scene is this one struct plus the terminal, malloc-free in the
 * hot path.  Fields are grouped by the CONCEPT they belong to, not by the key
 * that changes them: the colour `theme` is a RENDER choice, kept apart from the
 * simulation knobs.
 *
 *   WHAT        : stars[] / n_stars (catalogue), shocks[] (active births)
 *   field state : world_time + the two ParallaxLayer scroll offsets
 *   SIM knobs   : near/far layer speeds, threshold (gas visibility cutoff)
 *   RENDER knob : theme (palette)
 *   run-state   : paused, next_shock_in (auto-birth cadence timer)           */
typedef struct {
    /* WHAT is simulated */
    Star  stars[N_STARS_MAX];      /* catalogue pool; first n_stars are live   */
    Shock shocks[N_SHOCKS_MAX];    /* active star-birth events                 */
    int   n_stars;

    /* Parallax field state — advanced every tick */
    float         world_time;      /* seconds elapsed (drives star twinkle)    */
    ParallaxLayer near, far;       /* the two scrolling noise layers           */

    /* HOW the user drives the simulation — tunable knobs */
    float threshold;               /* gas visibility cutoff (higher = sparser) */

    /* RENDER knob — palette selection (a render choice, not a sim knob) */
    int   theme;

    /* run-state + auto-birth cadence */
    int   paused;
    float next_shock_in;           /* seconds until the next auto star birth   */
} Nebula;

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  logic — pure scalar fields: no mutation, no I/O                      */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Fast integer hash → [0, 1] for value-noise lattice samples. */
static float hash01(int x, int y)
{
    uint32_t h = (uint32_t)(x * 374761393) ^ (uint32_t)(y * 668265263);
    h = (h ^ (h >> 13)) * 1274126177u;
    h = h ^ (h >> 16);
    return (float)(h & 0xFFFFFF) / (float)0x1000000;
}

static float smoothstep(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

/* Value noise at fractional (x, y). Bilinear interp + smoothstep. */
static float value_noise(float x, float y)
{
    int   xi = (int)floorf(x), yi = (int)floorf(y);
    float fx = x - (float)xi,  fy = y - (float)yi;
    float v00 = hash01(xi,   yi);
    float v10 = hash01(xi+1, yi);
    float v01 = hash01(xi,   yi+1);
    float v11 = hash01(xi+1, yi+1);
    float ux = smoothstep(fx);
    float uy = smoothstep(fy);
    float a  = v00 + (v10 - v00) * ux;
    float b  = v01 + (v11 - v01) * ux;
    return a + (b - a) * uy;
}

/* fBm: sum of OCTAVES value-noise samples at increasing freq. */
static float fbm(float x, float y, float base_freq)
{
    float sum  = 0.0f;
    float amp  = 1.0f;
    float freq = base_freq;
    float norm = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        sum += amp * value_noise(x * freq, y * freq);
        norm += amp;
        amp  *= FBM_GAIN;
        freq *= FBM_LACUNARITY;
    }
    return sum / norm;
}

/* Quantise a [0,1] gas brightness to a glyph/colour bucket index 0..4. */
static int gas_bucket(float v)
{
    int b = (int)(v * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

/*
 * shock_brightness_at — extra brightness contribution from active shocks
 * at cell (sx, sy). Three layered contributions:
 *
 *   1. Expanding shockwave ring — Gaussian peak at d = r_now (was the
 *      only effect in stage 1).
 *   2. Anisotropic diffraction spikes — bright on the horizontal and
 *      vertical axes through the star centre, fading by age 1.2s.
 *      This is what gives a newborn star its iconic 4-pointed look.
 *   3. Diffuse halo — soft Gaussian glow centred on the star, slowly
 *      illuminating the surrounding gas; fades by age 2.5s.
 */
static float shock_brightness_at(const Nebula *n, float sx, float sy)
{
    float total = 0.0f;
    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        const Shock *s = &n->shocks[i];
        if (!s->alive) continue;

        float r_now = SHOCK_SPEED * s->age;
        float dx    = (sx - s->x) / ASPECT_X;
        float dy    = (sy - s->y);
        float d     = sqrtf(dx * dx + dy * dy);
        float fade  = 1.0f - (s->age / SHOCK_LIFE);
        if (fade < 0) fade = 0;

        /* Layer 1 — expanding shockwave ring. */
        float ring  = d - r_now;
        float gauss = expf(-ring * ring / (SHOCK_THICKNESS * SHOCK_THICKNESS));
        total += gauss * fade;

        /* Layer 2 — diffraction spikes. Bright thin lines along ±x
         * and ±y axes through the star, falling off radially so the
         * spikes don't dominate at large distance. Strong early. */
        float spike_t = 1.0f - (s->age / 1.2f);
        if (spike_t > 0.0f) {
            float h_spike = expf(-dy * dy / 0.6f) * expf(-d * d / 80.0f);
            float v_spike = expf(-dx * dx / 0.3f) * expf(-d * d / 80.0f);
            total += (h_spike + v_spike) * spike_t * 1.5f;
        }

        /* Layer 3 — diffuse halo. Soft Gaussian glow around the centre
         * that fades over a couple of seconds, illuminating gas. */
        float halo_t = 1.0f - (s->age / 2.5f);
        if (halo_t < 0.0f) halo_t = 0.0f;
        float halo = expf(-d * d / 12.0f);
        total += halo * halo_t * 0.6f;
    }
    return total;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  simulation — advances state (sole mutator of the Nebula)            */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

static float random_shock_interval(void)
{
    return SHOCK_INTERVAL_MIN
         + frand() * (SHOCK_INTERVAL_MAX - SHOCK_INTERVAL_MIN);
}

/* star_set — place a star at (x,y) with the given brightness; assigns a random
 * twinkle phase and marks it alive.  Shared by the catalogue seed, the +stars
 * key, and shock-planted newborns. */
static void star_set(Star *st, int x, int y, float brightness)
{
    st->x          = x;
    st->y          = y;
    st->brightness = brightness;
    st->phase      = frand() * 2.0f * (float)M_PI;
    st->alive      = 1;
}

static void nebula_seed_stars(Nebula *n, int rows, int cols)
{
    for (int i = 0; i < n->n_stars; i++) {
        int   sx = rand() % (cols > 0 ? cols : 1);
        int   sy = rand() % ((rows > 1) ? (rows - 1) : 1);
        float b  = 0.3f + 0.7f * frand();
        star_set(&n->stars[i], sx, sy, b);
    }
}

static void nebula_reseed(Nebula *n, int rows, int cols)
{
    for (int i = 0; i < N_STARS_MAX; i++) n->stars[i].alive  = 0;
    for (int i = 0; i < N_SHOCKS_MAX; i++) n->shocks[i].alive = 0;
    nebula_seed_stars(n, rows, cols);
    n->near.ox = n->near.oy = 0.0f;
    n->far.ox  = n->far.oy  = 0.0f;
    n->world_time    = 0.0f;
    n->next_shock_in = random_shock_interval();
}

/*
 * shock_spawn — find a dead slot, set position to a high-density spot
 * (sample fBm at random points, keep the brightest of K tries).
 */
/* Index of the first free shock slot, or -1 if all are in use. */
static int find_dead_shock(const Nebula *n)
{
    for (int i = 0; i < N_SHOCKS_MAX; i++)
        if (!n->shocks[i].alive) return i;
    return -1;
}

/* Pick the densest gas cell out of SHOCK_SAMPLES random candidates, so star
 * births favour bright gas.  Writes the winner to (*bx, *by). */
static void densest_cell(const Nebula *n, int rows, int cols, int *bx, int *by)
{
    *bx = cols / 2; *by = (rows - 1) / 2;
    float best_v = -1.0f;
    for (int k = 0; k < SHOCK_SAMPLES; k++) {
        int cx = rand() % cols;
        int cy = rand() % ((rows - 1) > 0 ? (rows - 1) : 1);
        float fn = fbm((float)cx + n->near.ox,
                       (float)cy + n->near.oy, FBM_FREQ_NEAR);
        float ff = fbm((float)cx + n->far.ox,
                       (float)cy + n->far.oy, FBM_FREQ_FAR);
        float v  = GAS_NEAR_W * fn + GAS_FAR_W * ff;
        if (v > best_v) { best_v = v; *bx = cx; *by = cy; }
    }
}

/* shock_spawn — start a new star birth at the densest gas cell we can find. */
static void shock_spawn(Nebula *n, int rows, int cols)
{
    int slot = find_dead_shock(n);
    if (slot < 0) return;

    int bx, by;
    densest_cell(n, rows, cols, &bx, &by);
    n->shocks[slot].x          = (float)bx;
    n->shocks[slot].y          = (float)by;
    n->shocks[slot].age        = 0.0f;
    n->shocks[slot].alive      = 1;
    n->shocks[slot].star_added = 0;
}

/* Advance the world clock and scroll both parallax layers (x boosted by the
 * cell aspect, y a gentler vertical drift). */
static void advance_parallax(Nebula *n, float dt)
{
    n->world_time += dt;
    n->near.ox += n->near.speed * ASPECT_X      * dt;
    n->far.ox  += n->far.speed  * ASPECT_X      * dt;
    n->near.oy += n->near.speed * SCROLL_VDRIFT * dt;
    n->far.oy  += n->far.speed  * SCROLL_VDRIFT * dt;
}

/* Age every active shock: when the spike phase ends, plant the permanent star
 * (the event's visible legacy); retire the shock once it outlives SHOCK_LIFE. */
static void age_shocks(Nebula *n, float dt)
{
    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        Shock *s = &n->shocks[i];
        if (!s->alive) continue;
        s->age += dt;

        if (!s->star_added && s->age >= SHOCK_SPIKE_DUR) {
            if (n->n_stars < N_STARS_MAX) {
                float b = 0.85f + 0.15f * frand();          /* bright newborn */
                star_set(&n->stars[n->n_stars++], (int)s->x, (int)s->y, b);
            }
            s->star_added = 1;
        }

        if (s->age > SHOCK_LIFE) s->alive = 0;
    }
}

static void nebula_tick(Nebula *n, float dt, int rows, int cols)
{
    advance_parallax(n, dt);
    age_shocks(n, dt);

    /* Schedule the next automatic star birth. */
    n->next_shock_in -= dt;
    if (n->next_shock_in <= 0.0f) {
        shock_spawn(n, rows, cols);
        n->next_shock_in = random_shock_interval();
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  render — state → screen; reads only, never mutates sim state         */
/* ═══════════════════════════════════════════════════════════════════════ */

/* 5-stop nebula palette per theme + a star colour and shock colour.
 * Each theme has a distinct dominant hue so cycling 't' is visibly
 * different across the sky; the dim end is brightened from near-black
 * (17/22/52) to visible mid-tones. */
static const short GAS_256[N_THEMES][5] = {
    /* 0 emission red   — pink/red → orange → white                   */
    {  88, 131, 196, 215, 231 },
    /* 1 reflection blue — deep blue → cyan → white                   */
    {  25,  33,  39,  51, 231 },
    /* 2 emerald nebula — dark green → lime → yellow-green → white     */
    {  22,  28,  82, 154, 231 },
    /* 3 horsehead pink — purple → magenta → light pink → white       */
    {  53,  91, 165, 213, 231 },
};
static const short STAR_256[N_THEMES] = { 231, 231, 231, 231 };
static const short SHOCK_256[N_THEMES] = { 226, 226, 226, 226 };

static const short GAS_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_YELLOW, COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,   COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW, COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? GAS_256[theme][i] : GAS_8[theme][i];
        init_pair((short)(PAIR_GAS_0 + i), fg, -1);
    }
    init_pair(PAIR_STAR,  x256 ? STAR_256[theme]  : COLOR_WHITE,  -1);
    init_pair(PAIR_SHOCK, x256 ? SHOCK_256[theme] : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,   x256 ? 226 : COLOR_YELLOW, -1);  /* top: bright yellow */
    init_pair(PAIR_HINT,  x256 ? 51  : COLOR_CYAN,   -1);  /* bottom: bright cyan */
}

static const char STAR_GLYPH[3] = { '.', '+', '*' };

/*
 * star_draw — bright dot with sinusoidal twinkle. brightness × twinkle
 * picks one of three glyphs and bold/dim attribute.
 */
static void star_draw(const Star *s, float world_time, int rows, int cols)
{
    if (!s->alive) return;
    if (s->x < 0 || s->x >= cols || s->y < 0 || s->y >= rows - 1) return;
    float twinkle = 0.7f + 0.3f * sinf(world_time * 1.5f + s->phase);
    float v       = s->brightness * twinkle;
    int   bucket  = (v < 0.4f) ? 0 : (v < 0.75f) ? 1 : 2;
    chtype attr   = COLOR_PAIR(PAIR_STAR);
    if (bucket == 2) attr |= A_BOLD;
    if (bucket == 0) attr |= A_DIM;
    attron(attr);
    mvaddch(s->y, s->x, (chtype)(unsigned char)STAR_GLYPH[bucket]);
    attroff(attr);
}

/* Glyph ramp by gas brightness bucket. */
static const char GAS_GLYPH[5] = { '.', ':', '~', 'o', '#' };

/* fBm gas raster — sample the two parallax layers at every cell, fold in shock
 * illumination, and emit a glyph where the brightness clears the threshold. */
static void draw_gas(const Nebula *n, int rows, int cols)
{
    for (int sr = 0; sr < rows - 1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            float fn = fbm((float)sc + n->near.ox,
                           (float)sr + n->near.oy, FBM_FREQ_NEAR);
            float ff = fbm((float)sc + n->far.ox,
                           (float)sr + n->far.oy, FBM_FREQ_FAR);
            float v = GAS_NEAR_W * fn + GAS_FAR_W * ff;
            v += shock_brightness_at(n, (float)sc, (float)sr) * SHOCK_BOOST;
            if (v < n->threshold) continue;

            float intensity = (v - n->threshold) / (1.0f - n->threshold + 0.001f);
            if (intensity < 0) intensity = 0;
            if (intensity > 1) intensity = 1;
            int bucket = gas_bucket(intensity);
            chtype attr = COLOR_PAIR(PAIR_GAS_0 + bucket);
            attr |= (bucket >= 3) ? A_BOLD : A_DIM;
            attron(attr);
            mvaddch(sr, sc, (chtype)(unsigned char)GAS_GLYPH[bucket]);
            attroff(attr);
        }
    }
}

/* The whole twinkling star catalogue, drawn over the gas. */
static void draw_stars(const Nebula *n, int rows, int cols)
{
    for (int i = 0; i < n->n_stars; i++)
        star_draw(&n->stars[i], n->world_time, rows, cols);
}

/* Shock phase 1 — flash: a 3×3 white-hot core. */
static void draw_shock_flash(int sr, int sc, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            int rr = sr + dr, cc = sc + dc;
            if (rr < 0 || rr >= rows - 1) continue;
            if (cc < 0 || cc >= cols)     continue;
            char ch = (dr == 0 && dc == 0) ? '*' : '+';
            mvaddch(rr, cc, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
}

/* Shock phase 2 — young star: 4-pointed cross, spikes two cells each way. */
static void draw_shock_spikes(int sr, int sc, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
    mvaddch(sr, sc, (chtype)'*');
    if (sc - 1 >= 0)        mvaddch(sr, sc - 1, (chtype)'-');
    if (sc - 2 >= 0)        mvaddch(sr, sc - 2, (chtype)'<');
    if (sc + 1 < cols)      mvaddch(sr, sc + 1, (chtype)'-');
    if (sc + 2 < cols)      mvaddch(sr, sc + 2, (chtype)'>');
    if (sr - 1 >= 0)        mvaddch(sr - 1, sc, (chtype)'|');
    if (sr + 1 < rows - 1)  mvaddch(sr + 1, sc, (chtype)'|');
    attroff(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
}

/* Shock phase 3 — settling: a single glyph fading toward death. */
static void draw_shock_settle(int sr, int sc, float age)
{
    float fade = 1.0f - (age / SHOCK_LIFE);
    if (fade <= 0.2f) return;
    chtype attr = COLOR_PAIR(PAIR_SHOCK) | (fade > 0.5f ? A_BOLD : 0);
    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)(fade > 0.6f ? '+' : '.'));
    attroff(attr);
}

/* Each active shock's centre, drawn by its age-driven phase:
 *   flash (3×3 core) → 4-point spikes → settling fade. */
static void draw_shocks(const Nebula *n, int rows, int cols)
{
    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        const Shock *s = &n->shocks[i];
        if (!s->alive) continue;
        int sr = (int)s->y, sc = (int)s->x;
        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;

        if      (s->age < SHOCK_FLASH_DUR) draw_shock_flash(sr, sc, rows, cols);
        else if (s->age < SHOCK_SPIKE_DUR) draw_shock_spikes(sr, sc, rows, cols);
        else                               draw_shock_settle(sr, sc, s->age);
    }
}

/* HUD — data readout top-right, action keys bottom-left. */
static void draw_hud(const Nebula *n, int rows, int cols, double fps)
{
    int n_shocks_active = 0;
    for (int i = 0; i < N_SHOCKS_MAX; i++)
        if (n->shocks[i].alive) n_shocks_active++;

    char buf[160];
    snprintf(buf, sizeof buf,
             " stars:%d  shocks:%d  thresh:%.2f  scroll:%.2f  theme:%d  "
             "%5.1f fps  %s ",
             n->n_stars, n_shocks_active, n->threshold,
             n->near.speed, n->theme, fps,
             n->paused ? "PAUSED " : "running");
    int x = cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " [/]:stars  -/+:thresh  ,/.:scroll  b:birth  t:theme  "
             "r:reset  p:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Nebula *n, double fps)
{
    erase();
    draw_gas(n, rows, cols);
    draw_stars(n, rows, cols);
    draw_shocks(n, rows, cols);
    draw_hud(n, rows, cols, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  app — combine point (per-tick order) + user events + frame cap       */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Nebula g_neb;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_neb.n_stars     = N_STARS_DEFAULT;
    g_neb.near.speed = SCROLL_NEAR_DEFAULT;
    g_neb.far.speed  = SCROLL_FAR_DEFAULT;
    g_neb.threshold   = THRESH_DEFAULT;
    g_neb.theme       = 0;

    screen_init(g_neb.theme);
    int rows = LINES, cols = COLS;
    nebula_reseed(&g_neb, rows, cols);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        /* USER EVENT (out of tick): apply resize → reseed for new bounds */
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            nebula_reseed(&g_neb, rows, cols);
        }

        /* USER EVENT (out of tick): drain input, mutate knobs / run-state */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_neb.paused ^= 1; break;
                case 'r':          nebula_reseed(&g_neb, rows, cols); break;
                case 't':          g_neb.theme = (g_neb.theme + 1) % N_THEMES;
                                   color_init(g_neb.theme); break;
                case 'b':          shock_spawn(&g_neb, rows, cols); break;
                case '[':
                    if (g_neb.n_stars - N_STARS_STEP >= N_STARS_MIN) {
                        g_neb.n_stars -= N_STARS_STEP;
                    }
                    break;
                case ']':
                    if (g_neb.n_stars + N_STARS_STEP <= N_STARS_MAX) {
                        int old = g_neb.n_stars;
                        g_neb.n_stars += N_STARS_STEP;
                        for (int i = old; i < g_neb.n_stars; i++) {
                            int   sx = rand() % cols;
                            int   sy = rand() % (rows - 1);
                            float b  = 0.3f + 0.7f * frand();
                            star_set(&g_neb.stars[i], sx, sy, b);
                        }
                    }
                    break;
                case '-':
                    if (g_neb.threshold + THRESH_STEP <= THRESH_MAX)
                        g_neb.threshold += THRESH_STEP;
                    break;
                case '+': case '=':
                    if (g_neb.threshold - THRESH_STEP >= THRESH_MIN)
                        g_neb.threshold -= THRESH_STEP;
                    break;
                case ',':
                    if (g_neb.near.speed - SCROLL_STEP >= SCROLL_MIN) {
                        g_neb.near.speed -= SCROLL_STEP;
                        g_neb.far.speed  -= SCROLL_STEP * 0.3f;
                        if (g_neb.far.speed < SCROLL_MIN) g_neb.far.speed = SCROLL_MIN;
                    }
                    break;
                case '.':
                    if (g_neb.near.speed + SCROLL_STEP <= SCROLL_MAX) {
                        g_neb.near.speed += SCROLL_STEP;
                        g_neb.far.speed  += SCROLL_STEP * 0.3f;
                    }
                    break;
            }
        }

        /* 1. PERFORMANCE — measure dt, capped to avoid spiral-of-death */
        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;

        /* 2. SIMULATION — the sole state advance (skipped while paused) */
        if (!g_neb.paused) nebula_tick(&g_neb, dt, rows, cols);

        /* 3. PERFORMANCE — smoothed fps estimate */
        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        /* 4. RENDER — read-only */
        scene_draw(rows, cols, &g_neb, fps);

        /* 5. PERFORMANCE — sleep to the frame cap */
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
