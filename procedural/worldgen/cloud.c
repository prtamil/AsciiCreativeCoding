/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cloud.c
 *   — Procedural cloud layers from a stack of fractional-Brownian-
 *     motion (fBm) noise samples.  Each cell of the sky asks "how
 *     dense is the cloud here?", the answer comes from one or more
 *     fBm evaluations, and the cell is drawn at a glyph + colour
 *     intensity that maps to that density.  Wind drifts the noise
 *     coordinates over time, so the clouds slide across the screen
 *     forever.
 *
 * DEMO: A sky.  Soft white shapes drift slowly from left to right.
 *       Cycle 15 cloud presets with n / p — every preset is one row in
 *       the `presets[]` table (noise scale + anisotropy, octaves, domain-
 *       warp, thresholds, glyphs, ramp, lightning), all driven by ONE
 *       sampler and ONE renderer:
 *
 *         CUMULUS  CIRRUS  STRATUS   — the textbook three.
 *         STORM    ANVIL             — dense + flickering '/' lightning.
 *         MACKEREL FOG     WISPS     — dappled / overcast / sparse high.
 *         MARESTAILS STREETS POPCORN — curved streaks / bands / fair-weather.
 *         NIMBUS   TURBULENT VEIL    — dark rain / churning / thin haze.
 *         BILLOWS                    — wave clouds.
 *
 *       Anisotropy (scale_y > scale_x) stretches features horizontally;
 *       domain-warp turns clean blobs into swirly, turbulent silhouettes;
 *       the threshold row sets how much sky the cloud covers.
 *
 *       'r' reseeds the Perlin permutation table — same preset, new cloud
 *       arrangement.  No phase machine, no rebuild cycle: the wind blows on.
 *
 * Study alongside:
 *   ../fields/perin_noise_flow_showcase.c
 *      — Perlin / fBm reference. The `fbm2` here is the same scaffold,
 *        parameterised by octave count so each preset picks the
 *        amount of detail it needs.
 *   ../worldgen/hydraulic.c
 *      — uses the same fBm field as a HEIGHT MAP that gets carved.
 *        Here the field is a DENSITY MAP that gets thresholded into
 *        glyphs. Two domains, one noise scaffold.
 *
 * Section map:
 *   §1 config    — wind, warp, themes, the 15-row CloudPreset table
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — HUD reserved + 10 themes (8-step ramp + accents)
 *   §5 cloud     — hash, perlin, fbm with octave count, one parameterised
 *                  density sampler, level helper
 *   §6 scene     — wind / drift state, scene_tick (just advances time)
 *   §7 screen    — one preset-driven renderer + HUD
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the wind
 *   r          reseed (new cloud arrangement, same preset)
 *   n / N      next preset  (cycles all 15)
 *   p / P      previous preset
 *   t / T      next / previous theme
 *   + / =      faster wind
 *   -          slower wind
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/cloud.c \
 *     -o cloud -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two pieces composed together.
 *
 *                  (1) FRACTIONAL BROWNIAN MOTION (fBm). A single
 *                      Perlin-noise sample is a smooth scalar field
 *                      with one characteristic feature size. STACK
 *                      several Perlin samples at doubled frequency
 *                      and halved amplitude, sum them, and you get
 *                      detail at every scale — the classic fBm
 *                      formula:
 *                          f(x, y) = Σ_k a_k · perlin(x·2^k, y·2^k)
 *                                     where a_k = 0.5^k
 *                      Normalise to roughly [0, 1] and threshold:
 *                      every cell becomes opaque (cloud) or
 *                      transparent (clear sky) depending on whether
 *                      the field exceeds a chosen threshold.
 *
 *                  (2) MORPHOLOGY VIA SAMPLING SHAPE. The cloud TYPE
 *                      is encoded in HOW you sample fBm — and ALL of it
 *                      lives in one CloudPreset row (§1), read by one
 *                      sampler (cloud_density). Four knobs span the gallery:
 *                        SCALE / ANISOTROPY — scale_x, scale_y set feature
 *                                   size per axis. scale_y ≫ scale_x squashes
 *                                   features vertically → horizontal wisps
 *                                   (CIRRUS) or flat bands (STRATUS); equal-
 *                                   ish scales → round puffs (CUMULUS).
 *                        OCTAVES  — fBm detail (smooth FOG ↔ fractal STORM).
 *                        DOMAIN WARP — warp_amt offsets the sample coords by
 *                                   a second fBm: 0 = clean streaks, high =
 *                                   swirly / turbulent (STORM, TURBULENT).
 *                        THRESHOLDS — where density becomes cloud: a low
 *                                   cut = overcast (FOG), high = sparse (WISPS).
 *                      15 rows of these four knobs = 15 distinct skies.
 *
 *                  Plus DRIFT — every sample's input coords get a
 *                  time-dependent offset (wind), so the cloud field
 *                  appears to scroll. Each preset's wind_mult scales its
 *                  drift speed, giving parallax between presets.
 *
 * Data-structure : One const CloudPreset table (15 rows) + a few Scene
 *                  floats — time, wind offset, preset index. The whole sky
 *                  is a function of (cell_x, cell_y, time); re-evaluated
 *                  every frame, no caching, no allocation. Rendering at
 *                  240×80 × 60 fps takes on the order of one Perlin call
 *                  per cell per frame, well under 1 % of a current CPU.
 *
 * Rendering      : ASCII only. Per cell: compute density, find which
 *                  of five LEVELS it falls into (clear / wispy / thin
 *                  / dense / core), pick a glyph from the active preset's
 *                  4-glyph table, render in the matching ramp tint. Glyphs
 *                  vary by preset so the same density reads differently per
 *                  sky — e.g. CUMULUS '.'/'o'/'@' vs STRATUS ','/'_'/'#'.
 *                  Presets flagged `lightning` (STORM, ANVIL) overstrike
 *                  sparse '/' bolts in PAIR_HOT at their dense cores.
 *
 * Performance    : The full screen is recomputed each frame. Warped
 *                  presets cost 3 fBm calls per cell (2-octave warp pass +
 *                  the main sample); at 4 octaves that is ~12 Perlin
 *                  samples / cell. At 19 200 cells × 60 fps × 50 ns per
 *                  sample ≈ 60 ms / sec, around 6 % of one core. Warp-free
 *                  presets (CIRRUS, FOG) are cheaper. No allocation in
 *                  steady state.
 *
 * References     :
 *   NOISE — the underlying field (§3 perlin2d / grad2 / fade_q)
 *   • Perlin, K. (1985)  "An Image Synthesizer", Computer Graphics
 *     19(3), SIGGRAPH '85.  The original gradient-noise lattice.
 *   • Perlin, K. (2002)  "Improving Noise", SIGGRAPH '02.
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *       Source of the quintic fade curve fade_q = 6t⁵−15t⁴+10t³ and
 *       the 8-gradient scheme used here.
 *
 *   fBm — detail at every scale (§3 fbm2_n + the density samplers)
 *   • Mandelbrot, B. & Van Ness, J. (1968)  "Fractional Brownian
 *     Motions, Fractional Noises and Applications", SIAM Review
 *     10:422–437.  Where fBm comes from.
 *   • Ebert, Musgrave, Peachey, Perlin & Worley — "Texturing &
 *     Modeling: A Procedural Approach", 3rd ed. (2003).  Musgrave's
 *     chapters are the canonical fBm / turbulence / noise-as-density
 *     reference behind this whole file.
 *
 *   RENDERING — turning the field into a picture
 *   • Quílez, I. — "fBm"   https://iquilezles.org/articles/fbm/
 *       octave summation + normalisation (the fbm2_n recipe.)
 *   • Quílez, I. — "Domain warping"  https://iquilezles.org/articles/warp/
 *       the warp pre-pass that makes CUMULUS / STORM swirl.
 *
 *   CLOUD DOMAIN — the subject itself
 *   • Schneider, A. & Vos, N. (2015)  "The Real-Time Volumetric
 *     Cloudscapes of Horizon: Zero Dawn", SIGGRAPH course.  How cloud
 *     TYPE maps to noise shape — the idea the presets are built on.
 *   • WMO — International Cloud Atlas.  https://cloudatlas.wmo.int/
 *       the genera (cumulus, cirrus, stratus, nimbus…) the presets
 *       are named after.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To draw a cloud, do not draw a cloud — draw the THRESHOLD of a
 * smooth scalar field. Fractional Brownian motion gives you a scalar
 * field that has interesting features at every scale; ask "is this
 * field over 0.6 here?" and the YES regions look exactly like clouds,
 * because clouds in nature are exactly that: regions where some
 * scalar (water-vapor density) exceeds a threshold. The MORPHOLOGY
 * (puffy vs wispy vs banded) is just the SHAPE you sample the field
 * at — isotropic spheres, horizontally stretched ovals, vertically
 * stacked bands.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a sheet of cotton ball stuffing. It has a lumpy density —
 * thicker in some places, thinner in others, with structure at many
 * scales (small fibres clump into tufts which clump into wads). Now
 * cut a 2-D cross-section through it. The lumps you see are the
 * density's THRESHOLD CONTOURS — places where the cotton crosses some
 * fixed threshold. Slide a window over the cross-section: it looks
 * like clouds drifting.
 *
 * That's literally the algorithm. fBm is the cotton-density function;
 * thresholding turns it into binary "cloud / not-cloud"; soft
 * thresholding (multiple bands) gives shaded glyphs. The wind is just
 * the cross-section sliding over the cotton.
 *
 * Different cloud TYPES come from how you tilt and squeeze the
 * cotton sheet. Stretch it horizontally → the lumps become long
 * streaks → CIRRUS. Squash it vertically → the lumps become flat
 * pancakes → STRATUS. Leave it alone (and add a domain-warp jiggle
 * for organicness) → CUMULUS.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. PERMUTATION TABLE — shuffle perm[256] from a seed and
 *     duplicate to 512 (Perlin's standard trick to skip the modulo
 *     in lookup).
 *  2. EACH FRAME:
 *     a. Advance wind: wind_x += WIND_X · dt · speed_mul
 *                      wind_y += WIND_Y · dt · speed_mul
 *                      warp_t += DRIFT  · dt · speed_mul
 *     b. For every screen cell (sx, sy), using the active preset row:
 *         i. Compute density with cloud_density (1 fBm call, or 3 if the
 *            preset's warp_amt > 0 — 2 for the warp offset, 1 for the main
 *            sample). Coords are scaled by the preset's scale_x / scale_y.
 *        ii. Convert density → level via the preset's 4 thresholds.
 *       iii. If level > 0:
 *              glyph = preset.glyphs[level - 1]
 *              ramp  = preset.ramp  [level - 1]
 *              attr  = A_DIM / A_NORMAL / A_BOLD by level
 *              if preset.lightning AND level == 4 AND hash-gate → '/' in HOT
 *              mvaddch (sy, sx, glyph)
 *
 * KEY FORMULAS
 * ────────────
 *  fBm with N octaves:
 *     f(x, y) = (1/A) · Σ_{k=0..N-1} a_k · perlin(x·2^k, y·2^k)
 *     A       = Σ a_k = 1 + 0.5 + 0.25 + … (so f ∈ roughly [-1, 1])
 *     We re-map to [0, 1] with f' = 0.5·f + 0.5.
 *
 *  Cloud density (one parameterised sampler; the preset row supplies
 *  scale_x, scale_y, octaves, warp_amt, wind_mult):
 *     wx = x + wind_x · wind_mult,   wy = y + wind_y
 *     if warp_amt > 0:                       // domain-warp pre-pass
 *        qx = fbm(x · WARP_SCALE,       y · WARP_SCALE + warp_t)
 *        qy = fbm(x · WARP_SCALE + 5.2, y · WARP_SCALE + 1.3 + warp_t)
 *        wx += qx · warp_amt,   wy += qy · warp_amt
 *     d = fbm(wx · scale_x, wy · scale_y, octaves)
 *   The aspect ratio is baked into each preset's scale_y (round looks set
 *   scale_y ≈ 2·scale_x; anisotropic looks push scale_y far higher).
 *
 *  Density → level (5 buckets from the preset's 4 thresholds):
 *     d < thresholds[0] → 0   (clear sky, do not draw)
 *       < thresholds[1] → 1   (wispy edge)
 *       < thresholds[2] → 2   (medium body)
 *       < thresholds[3] → 3   (dense)
 *       ≥                → 4   (core / highlight)
 *
 *  Lightning gate (presets with lightning = true):
 *     bolt = (level == 4) AND (hash3(x, y, ⌊30·t⌋) % LIGHTNING_HASH_MOD == 0)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • ASPECT RATIO. Terminal cells are ~2× taller than wide, so a round
 *    cloud wants scale_y ≈ 2·scale_x. Each preset bakes this into its
 *    scale_y directly: "round" presets (CUMULUS) use ~2×, while wispy /
 *    banded presets (CIRRUS, STRATUS) push scale_y much higher on purpose
 *    so features stretch horizontally.
 *
 *  • WIND CONTINUITY. wind_x grows monotonically with time. After
 *    days it can exceed float precision (~10^7 cells of drift
 *    before single-precision Perlin starts to bin into the same
 *    integer cells). For a real long-running demo, modulo wind_x
 *    by perm size periodically; for an interactive showcase that
 *    rarely runs more than minutes, ignore.
 *
 *  • DOMAIN WARP COST. Each warped cell does 3 fBm calls (qx, qy, main);
 *    warp-free presets (CIRRUS, FOG, VEIL) do just 1. At 60 fps × 240×80
 *    cells × 4 octaves the warped path is ~14 M Perlin samples per second.
 *    Modern hardware handles it easily; to scale to 8 K terminals, drop
 *    the warp pre-pass to fewer octaves (OCT_WARP).
 *
 *  • FLOAT-VS-INT IN FLOOR. Perlin's permutation lookup uses
 *    `(int)floorf(x) & 255`. For NEGATIVE x, (int) cast truncates
 *    toward zero — wrong direction; floorf rounds toward −∞ — right
 *    direction. Don't replace floorf with a plain (int) cast or the
 *    cloud field becomes discontinuous at x = 0.
 *
 *  • ANISOTROPIC SCALES NEED ADJUSTING THRESHOLDS. A high scale_y / scale_x
 *    ratio doesn't change fBm's output range but does change the spatial
 *    distribution of high values — wispy presets usually want slightly
 *    lower thresholds than round ones for a similar sky-fill ratio.
 *
 *  • LIGHTNING DENSITY. Bolts fire only at a preset's level-4 cores, then
 *    a 1-in-LIGHTNING_HASH_MOD hash gate keeps them sparse, so storms get
 *    a few flickering '/' glyphs per frame. The ⌊30·t⌋ in the seed
 *    regenerates them at 30 Hz so they don't stick. Lower the modulo for
 *    more strikes.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space). Clouds freeze in place — no drift, no animation.
 *    Resume: clouds slide from exactly where they froze. Verifies
 *    the wind accumulator.
 *
 *  • Press 'r'. The cloud arrangement reshapes (different perm
 *    table) but the wind speed and preset are unchanged. Verifies
 *    perm reseed.
 *
 *  • CUMULUS preset. Clouds should look ROUND-ish, with organic
 *    swirly outlines (the domain-warp signature). If they look like
 *    perfect blobs with no swirl, the warp pre-pass is missing.
 *
 *  • CIRRUS / WISPS presets. Clouds should be obviously HORIZONTAL
 *    streaks, much wider than tall. If they look round, the
 *    anisotropic scale_y is wrong.
 *
 *  • STRATUS / STREETS presets. Clouds form horizontal BANDS that span
 *    much of the screen width. If you see localised blobs instead, the
 *    y-scale isn't compressed enough.
 *
 *  • STORM / ANVIL presets. Dense churning cloud with bright cores, plus
 *    occasional '/' bolts flickering in red at the densest cells.
 *
 *  • Counter (top-left HUD). n / p step the preset i/15 and the name
 *    updates; the whole gallery should cycle and wrap.
 *
 *  • Wind direction. Watch a cloud edge for a few seconds; it
 *    should slide leftward-to-rightward at speed proportional to
 *    the user's "speed" knob. + key doubles the rate; − halves it.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (concern layers) ─────────────────────────────────────── *
 *
 * The file is cut into six concern-layers; each function lives in exactly
 * one. The table says what each layer MUTATES.
 *
 *   Layer        §   Mutates
 *   ──────────── ── ──────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — const tables (presets[], themes[]) + #defines
 *   PERFORMANCE  §2  nothing — clock read + sleep; the throttle that USES
 *                    them (fixed timestep + frame cap) runs in §6
 *   LOGIC        §3  nothing — pure reads: perlin / fBm / cloud_density /
 *                    density_to_level. Reads the perm[] seed; never writes.
 *   SIMULATION   §4  perm[] (noise seed, on init/reseed EVENTS only) and the
 *                    Scene's Drift (time/wind/warp, advanced once per tick)
 *   RENDER       §5  ncurses state only (colour pairs, Screen dims). Paints
 *                    Scene → screen; never touches simulation state.
 *   APP          §6  App (running / need_resize / sim_fps); owns the tick.
 *
 * LOGIC is provably uncorruptible: it does no mutation and no I/O, so
 * deleting or reordering RENDER cannot change a density or a level.
 *
 * No EFFECTS layer: the only cosmetic flourish — STORM / ANVIL lightning —
 * is DERIVED at render time in render_clouds (a level-4 core that passes a
 * hash gate is drawn as '/'), never stored. Nothing to advance → no section.
 *
 * No DELAYS section: the sole hold is the `paused` flag, which short-circuits
 * scene_tick (§4). One line, documented there — not worth its own layer.
 *
 * PER-TICK COMBINE ORDER — main() (§6) is the ONE place sim state advances:
 *   PERFORMANCE  measure + clamp dt
 *   SIMULATION   scene_tick × N        (fixed timestep; skipped when paused)
 *   PERFORMANCE  fps tally + frame-cap sleep
 *   RENDER       screen_draw (→ scene_draw → render_clouds → LOGIC) + present
 *   EVENTS       getch → app_handle_key / resize   (NOT part of the tick)
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

/* ===================================================================== */
/* §1  CONFIG — const tables + tunables (mutates nothing)                  */
/* ===================================================================== */

/* Named constants in three groups: the simulation-rate / speed knobs the user
 * adjusts, two HUD sizing values, and the ncurses colour-pair IDs. Kept as one
 * enum so every magic number in the file has a name at the top. */
enum {
    /* tick rate — ]/[ step it; sets the fixed timestep in main() */
    SIM_FPS_MIN         =  10,    /* floor — below this the sim feels jerky    */
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,    /* ceiling — past display refresh anyway     */
    SIM_FPS_STEP        =  10,

    /* wind speed — +/- step it; doubles/halves (see app_handle_key) */
    SPEED_MIN           =   1,
    SPEED_DEF           =   8,    /* the 1.0× reference; Scene.speed/SPEED_DEF  */
    SPEED_MAX           =  64,

    /* HUD */
    HUD_COLS            =  80,    /* status-line buffer width (chars)          */
    FPS_UPDATE_MS       = 500,    /* fps readout refresh period                */

    /* ncurses colour-pair IDs. PAIR_HUD/PAIR_HINT reserved per /CLAUDE.md;
     * PAIR_RAMP_BASE..+7 are the 8 theme-ramp tints (Theme.ramp). */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 cloud-density tints            */
    PAIR_HOT            =  11,    /* lightning bolt accent (Theme.hot)         */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))
#define RENDER_FPS_CAP       60        /* outer frame-rate cap (Hz), separate from sim tick */
#define MAX_FRAME_DT_MS     100        /* clamp a frame's dt to dodge spiral-of-death (ms)  */

/* Wind — base velocities at SPEED_DEF. The user's speed knob scales
 * these linearly. */
#define WIND_X_BASE          6.0f      /* cells / sec */
#define WIND_Y_BASE          0.4f
#define WARP_DRIFT_BASE      0.05f     /* warp-noise time evolution */

/* fBm octaves + spatial scale of the domain-WARP pre-pass (the per-preset
 * cloud octave count lives in the preset table). */
#define OCT_WARP             2
#define WARP_SCALE           0.012f

/* Storm lightning — presets with `lightning` fire a sparse hash-gated '/'
 * bolt at their brightest cells (level 4 cores). 1-in-MOD cells strike. */
#define LIGHTNING_HASH_MOD   1500u
#define LIGHTNING_HZ         30.0f    /* bolt pattern regenerates this often (Hz) */

/* Noise seed used at startup and as the salt for the 'r' reseed. */
#define BASE_SEED            0xC0FFEE

/*
 * CloudPreset — one cloud "look", encoded as PURE DATA: how to sample the fBm
 * density field and how to ink the result.
 *
 * WHY a struct: the morphology of a real cloud — puffy cumulus vs streaky
 * cirrus vs flat stratus — is NOT separate code here; it is captured entirely
 * by this row of numbers. So all 15 presets share one sampler (cloud_density,
 * §3) and one renderer (render_clouds, §5), and a new sky costs one line.
 * INTENT: keep the inner loops preset-agnostic — they take a single const
 * CloudPreset* and never branch on which sky is active.
 *
 * This "cloud TYPE = the SHAPE of the noise sampling" idea is the core lesson:
 *   • Schneider & Vos (2015) — cloud genera modelled from layered noise.
 *   • Musgrave, in Ebert et al. "Texturing & Modeling" — fBm AS a density
 *     field, and how octaves / lacunarity tune its look.
 *   • Quílez — "fBm" and "Domain warping" (the octaves and warp_amt knobs).
 */
typedef struct {
    const char *name;        /* HUD label; the WMO genus this row imitates    */

    /* --- sampling SHAPE — consumed by cloud_density (§3) ----------------- */
    float scale_x, scale_y;  /* noise frequency per axis (cycles per cell).
                              * scale_y > scale_x squashes features in y so they
                              * read as horizontal wisps/bands; scale_y ≈ 2·scale_x
                              * cancels the terminal's 2:1 cell aspect → round.  */
    int   octaves;           /* fBm octaves summed (2..5). More = finer detail,
                              * but the normalised sum also clusters nearer 0.5. */
    float warp_amt;          /* domain-warp strength, in cells (0 = clean blobs;
                              * 3–6 = swirly / turbulent). Offsets the sample
                              * coords by a second fBm — Quílez, "Domain warping". */
    float wind_mult;         /* this preset's share of the global wind drift
                              * (multiplies Drift.wind_x) → parallax per preset. */

    /* --- inking — consumed by density_to_level (§3) + render_clouds (§5) - */
    float thresholds[4];     /* ascending density cuts mapping d → level 1..4;
                              * d below thresholds[0] is clear sky. Lower [0] =
                              * more coverage (FOG ~0.30 overcast, WISPS ~0.60
                              * sparse). fBm output is ~[0,1], dense around 0.5. */
    char  glyphs[4];         /* glyph for levels 1..4 (light → dense). ASCII
                              * only — density is read through glyph weight.     */
    short ramp[4];           /* theme-ramp slot 0..7 per level (see Theme.ramp);
                              * higher slot = brighter tint for denser cloud.    */
    bool  lightning;         /* true → overstrike sparse '/' bolts at level-4
                              * cores (STORM, ANVIL); see render_clouds (§5).    */
} CloudPreset;

static const CloudPreset presets[] = {
/*   name           sx      sy     oct warp  wind  thresholds                glyphs              ramp        light */
{ "CUMULUS",    0.025f, 0.050f, 4, 3.0f, 1.0f, {0.50f,0.62f,0.74f,0.85f}, {'.','o','O','@'}, {3,5,6,7}, false },
{ "CIRRUS",     0.060f, 0.180f, 3, 0.0f, 1.4f, {0.48f,0.58f,0.66f,0.76f}, {'`','~','~','-'}, {3,4,5,6}, false },
{ "STRATUS",    0.018f, 0.060f, 3, 0.0f, 0.7f, {0.42f,0.52f,0.62f,0.74f}, {',','_','~','#'}, {3,4,5,7}, false },
{ "STORM",      0.022f, 0.045f, 5, 4.0f, 0.9f, {0.40f,0.52f,0.64f,0.74f}, {':','%','#','@'}, {4,5,7,7}, true  },
{ "MACKEREL",   0.055f, 0.110f, 4, 1.0f, 1.1f, {0.50f,0.60f,0.70f,0.80f}, {'.',':','o','O'}, {3,4,6,7}, false },
{ "FOG",        0.012f, 0.024f, 2, 0.0f, 0.3f, {0.30f,0.44f,0.58f,0.72f}, {'.',',',':','#'}, {3,4,5,6}, false },
{ "WISPS",      0.070f, 0.200f, 3, 0.5f, 1.5f, {0.60f,0.70f,0.78f,0.86f}, {'`','`','~','-'}, {3,4,5,6}, false },
{ "MARESTAILS", 0.050f, 0.150f, 3, 2.0f, 1.3f, {0.52f,0.62f,0.72f,0.82f}, {'`','~','-','='}, {3,4,5,6}, false },
{ "STREETS",    0.014f, 0.085f, 3, 0.6f, 1.0f, {0.46f,0.56f,0.66f,0.78f}, {',','-','~','#'}, {3,4,5,7}, false },
{ "POPCORN",    0.040f, 0.080f, 3, 2.0f, 1.0f, {0.56f,0.67f,0.77f,0.87f}, {'.','o','O','@'}, {3,5,6,7}, false },
{ "NIMBUS",     0.022f, 0.044f, 4, 2.5f, 0.6f, {0.34f,0.48f,0.61f,0.75f}, {':','o','#','@'}, {3,4,6,7}, false },
{ "TURBULENT",  0.028f, 0.056f, 5, 6.0f, 1.2f, {0.46f,0.58f,0.70f,0.82f}, {'~','o','%','@'}, {4,5,6,7}, false },
{ "VEIL",       0.030f, 0.060f, 2, 0.0f, 0.8f, {0.40f,0.52f,0.64f,0.78f}, {'`','.',',',':'}, {3,3,4,5}, false },
{ "BILLOWS",    0.022f, 0.075f, 3, 1.5f, 1.0f, {0.46f,0.56f,0.66f,0.78f}, {',','~','o','O'}, {3,4,6,7}, false },
{ "ANVIL",      0.032f, 0.075f, 4, 3.5f, 1.1f, {0.44f,0.56f,0.66f,0.76f}, {'.','%','#','@'}, {3,5,7,7}, true  },
};
#define N_PRESETS ((int)(sizeof presets / sizeof presets[0]))

/*
 * Theme — one named colour palette: an 8-step density gradient plus a lightning
 * accent, stored as xterm-256 colour indices (ANSI 256-colour SGR numbers).
 *
 * WHY a struct / INTENT: it decouples "how dense is this cell" (the LEVEL, via
 * CloudPreset.ramp) from "what colour is it" (the theme), so any preset reads
 * correctly under any palette. 't'/'T' swap the whole palette live by re-running
 * theme_apply (§5) over these indices — the field itself never changes.
 * CONTEXT: every entry is kept in the BRIGHT half of the cube so even A_DIM
 * cells stay legible on a black terminal — see "Theme Palette Brightness" in
 * /CLAUDE.md. Same 10-name menu as the other procedural showcases.
 */
typedef struct {
    const char *name;    /* HUD label                                         */
    short       ramp[8]; /* density gradient dim→bright (xterm-256 indices).
                          * Cloud levels map only into slots 3..7, so the four
                          * drawn tiers occupy the upper, well-separated half
                          * and never collapse toward clear-sky colour.        */
    short       hot;     /* accent for storm lightning — '/' bolts, PAIR_HOT
                          * (STORM / ANVIL presets).                           */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7   hot */
    { "DEFAULT",{ 24,  31,  67, 110, 153, 195, 231, 255 }, 226 },
    { "MATRIX", { 28,  34,  40,  76, 118, 154, 192, 230 }, 226 },
    { "NOVA",   { 60,  91, 134, 165, 207, 219, 225, 231 }, 226 },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 }, 226 },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 }, 226 },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 }, 226 },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 }, 226 },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 }, 226 },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 }, 226 },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 }, 226 },
};

/* ===================================================================== */
/* §2  PERFORMANCE — timing primitives for the fixed-timestep loop         */
/* ===================================================================== */
/* Pure helpers. The throttle that USES them — the fixed-timestep
 * accumulator + ~60 fps frame cap — lives at the one tick-combine point in
 * main() (§6), not here. */

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
/* §3  LOGIC — pure field math: hash, perlin, fBm, density, level          */
/* ===================================================================== */
/* Every function here is a pure read — no mutation, no I/O — so it is
 * provably impossible to corrupt from RENDER: reorder or delete the
 * renderer and these results are unchanged. The one shared datum is perm[]
 * (the noise seed), which LOGIC only READS; its sole writer is
 * SIMULATION's perm_shuffle (§4). */

/* hash3 — same routine as other showcases. Used by the lightning gate. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/* Perlin scaffold — same as other showcases.
 * perm[] is the noise seed table. LOGIC only READS it (perlin2d below); its
 * sole writer is SIMULATION's perm_shuffle (§4), run on init/reseed events
 * only — never in the tick. So LOGIC's output stays a pure function of
 * (coords, preset, perm[]). */
static uint8_t perm[512];

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

static float perlin2d(float x, float y)
{
    /* integer lattice cell + fractional position within it */
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);

    /* fade-weighted blend factors (smoothstep so cell seams vanish) */
    float u = fade_q(x), v = fade_q(y);

    /* hash the four corners of the cell via the permutation table */
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;

    /* gradient dot-product contribution from each corner */
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);

    /* bilinear blend of the four corners */
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/*
 * fbm2_n — fBm with a parameterised octave count. Each octave doubles
 * the frequency and halves the amplitude. Output is normalised to
 * roughly [0, 1].
 */
static float fbm2_n(float x, float y, int octaves)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* ----------------------------------------------------------------------- *
 * Density sampler — one parameterised function drives all 15 presets.      *
 * ----------------------------------------------------------------------- */

/*
 * cloud_density — sample the preset's fBm density field at cell (x, y).
 *
 * Two steps, both driven by the preset row:
 *   (1) DOMAIN WARP (only if warp_amt > 0). Two cheap fBm samples build an
 *       offset vector (qx, qy) that distorts the input coords. This breaks the
 *       "perfect blob" look of plain Perlin into swirly, organic silhouettes —
 *       the difference between CIRRUS (warp 0, clean streaks) and STORM /
 *       TURBULENT (high warp, churning).
 *   (2) ANISOTROPIC fBm. The warped, wind-drifted coords are scaled by
 *       scale_x / scale_y before the main fBm call. scale_y > scale_x squashes
 *       features vertically so they read as horizontal wisps or bands.
 */
static float cloud_density(const CloudPreset *p, float x, float y,
                           float wind_x, float wind_y, float warp_t)
{
    float wx = x + wind_x * p->wind_mult;
    float wy = y + wind_y;
    if (p->warp_amt > 0.0f) {
        float qx = fbm2_n(x * WARP_SCALE, y * WARP_SCALE + warp_t, OCT_WARP);
        float qy = fbm2_n((x + 5.2f) * WARP_SCALE,
                          (y + 1.3f) * WARP_SCALE + warp_t, OCT_WARP);
        wx += qx * p->warp_amt;
        wy += qy * p->warp_amt;
    }
    return fbm2_n(wx * p->scale_x, wy * p->scale_y, p->octaves);
}

/* ----------------------------------------------------------------------- *
 * Density → level conversion.                                             *
 * ----------------------------------------------------------------------- */

/*
 * density_to_level — bin a density value (in roughly [0, 1]) into one
 * of five levels using the active preset's thresholds. Level 0 is
 * "clear sky"; level 4 is "bright core".
 */
static inline int density_to_level(float d, const CloudPreset *p)
{
    if (d < p->thresholds[0]) return 0;
    if (d < p->thresholds[1]) return 1;
    if (d < p->thresholds[2]) return 2;
    if (d < p->thresholds[3]) return 3;
    return 4;
}

/* ===================================================================== */
/* §4  SIMULATION — the mutable state and what advances it                 */
/* ===================================================================== */
/* Mutates two things: perm[] (the noise seed in §3 — reseeded by
 * perm_shuffle on init / 'r' EVENTS, never in the tick) and the Scene
 * fields (time/wind/warp — advanced once per tick by scene_tick).
 * DELAYS: the `paused` flag short-circuits scene_tick — the only hold. */

/* perm_shuffle — reseed the noise table. The lone writer of perm[] (§3);
 * runs from scene_init and scene_reseed (init / 'r' events), NOT the tick. */
static void perm_shuffle(int seed)
{
    /* start from the identity permutation 0..255 */
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;

    /* Fisher–Yates shuffle, driven by an LCG seeded from `seed` */
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }

    /* mirror into the upper half so perlin2d can index perm[x]+y unwrapped */
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

/*
 * Drift — the cloud field's animation state: how far wind has ADVECTED the
 * noise sampling, plus the warp field's own time phase, all accumulated from
 * the master clock.
 *
 * WHY/CONTEXT: a static noise field is made to move by translating the sample
 * point a little each frame (advection) — the whole sky is f(cell − drift).
 * That is the standard way to animate Perlin/fBm; cf. ../fields/
 * perin_noise_flow_showcase.c and Quílez's noise articles. INTENT: keep this
 * apart from the wind SPEED knob (Scene.speed) — speed is the RATE the user
 * turns; Drift is the accumulated DISPLACEMENT it produces. VALUE LOGIC: all
 * four are monotonic accumulators advanced together once per tick (scene_tick,
 * §4); the renderer reads them, never writes. Floats, so after ~10^7 cells of
 * drift Perlin starts re-binning — fine for a minutes-long showcase.
 */
typedef struct {
    float time_secs;   /* master clock — seconds since start; also seeds the
                        * 30 Hz lightning flicker bucket in render_clouds.   */
    float wind_x;      /* accumulated horizontal sample offset, in cells     */
    float wind_y;      /* accumulated vertical   sample offset, in cells     */
    float warp_t;      /* domain-warp field's own evolving time offset, so the
                        * warp churns independently of the bulk wind.        */
} Drift;

/*
 * Scene — the whole simulated world, written as a table of contents so a reader
 * can see at a glance WHAT is simulated, HOW the user steers it, and the one
 * RENDER setting that rides along.
 *   WHAT   the drifting cloud field            → drift
 *   HOW    the simulation knobs the user turns → paused / speed / current_preset
 *   RENDER a render-only setting (palette), grouped APART from the sim knobs:
 *          a theme is a RENDER concept that merely happens to share a key, so
 *          it must not be mistaken for something the simulation reads.
 * Only init / tick orchestrators take Scene*; every other function takes the
 * narrowest sub-type (Drift, CloudPreset…) so the layers stay decoupled.
 */
typedef struct {
    Drift drift;               /* WHAT — the field's animation; advanced 1×/tick */
    bool  paused;              /* HOW  — DELAYS: freezes scene_tick when true    */
    int   speed;               /* HOW  — wind speed, SPEED_MIN..MAX (1..64),
                                *        def 8; scales the per-tick drift step.   */
    int   current_preset;      /* HOW  — active sky, index 0..N_PRESETS-1 of presets[] */
    int   current_theme;       /* RENDER — active palette, index 0..N_THEMES-1 of themes[] */
} Scene;

/* scene_reseed — reads the scene (its drift) to derive a seed, reshuffles the
 * noise table. Mutates only the global perm[], not the scene → const Scene*. */
static void scene_reseed(const Scene *s)
{
    /* salt the base seed with the current drift so each 'r' gives a new sky */
    int seed = (int)hash3((int)(s->drift.time_secs * 1000.0f),
                          (int)(s->drift.wind_x * 100.0f), BASE_SEED);
    perm_shuffle(seed);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_preset  = 0;
    perm_shuffle(BASE_SEED);
}

/*
 * scene_tick — advance the drift. No regen; clouds drift forever.
 * The user can press 'r' to reseed the perm table for a new layout.
 */
static void scene_tick(Scene *s, float dt)
{
    s->drift.time_secs += dt;
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->drift.wind_x += WIND_X_BASE * speed_mul * dt;
    s->drift.wind_y += WIND_Y_BASE * speed_mul * dt;
    s->drift.warp_t += WARP_DRIFT_BASE * speed_mul * dt;
}

/* ===================================================================== */
/* §5  RENDER — state → screen; reads only, never mutates sim state         */
/* ===================================================================== */
/* All ncurses I/O. Reads Scene + presets[]/themes[] and paints; the only
 * state it writes is terminal/ncurses internals (colour pairs, Screen
 * dimensions), never the simulation. EFFECTS: none stored — STORM / ANVIL
 * lightning is DERIVED here at draw time in render_clouds (a level-4 core
 * that passes a hash gate becomes a '/'), not kept as state. */

/* --- colour: HUD pairs + theme ramp → ncurses colour pairs -------------- */

/* bind one palette into ncurses pairs: the 8 ramp tints + the lightning accent */
static void set_palette_pairs(const short ramp[8], short hot)
{
    for (int i = 0; i < 8; i++)
        init_pair((short)(PAIR_RAMP_BASE + i), ramp[i], -1);
    init_pair(PAIR_HOT, hot, -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        set_palette_pairs(t->ramp, t->hot);
    } else {
        /* 8-colour terminals: a fixed cool→bright fallback ramp */
        static const short fb[8] = {
            COLOR_BLUE, COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_WHITE,COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE,
        };
        set_palette_pairs(fb, COLOR_YELLOW);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* --- terminal lifecycle + per-cell painting ----------------------------- */

/* Screen — the terminal viewport the field is painted into. WHY a type: it is
 * the bounds every render reader needs, so passing one const Screen* keeps the
 * 240×80-or-whatever size out of globals and current after a resize. Refreshed
 * from getmaxyx in screen_init / screen_resize (§5). */
typedef struct {
    int cols, rows;   /* viewport width / height in character cells, ≥ 0 */
} Screen;

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

/*
 * draw_cell — common path. Pick attr from level (DIM/NORMAL/BOLD).
 */
static inline void draw_cell(int sy, int sx, char glyph, int pair, int level)
{
    int attr;
    if      (level >= 4) attr = A_BOLD;
    else if (level >= 3) attr = A_BOLD;
    else if (level >= 2) attr = A_NORMAL;
    else                 attr = A_DIM;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/* ----------------------------------------------------------------------- *
 * Cloud renderer — one loop drives all 15 presets.                         *
 * ----------------------------------------------------------------------- */

/* lightning_strikes — is this dense core hit by a bolt on frame-bucket `t`?
 * Only level-4 cores of a lightning preset, then a sparse 1-in-MOD hash gate. */
static inline bool lightning_strikes(const CloudPreset *p, int level,
                                     int sx, int sy, int t)
{
    return p->lightning && level == 4 &&
           (hash3(sx, sy, t) % LIGHTNING_HASH_MOD) == 0u;
}

/*
 * render_clouds — for every sky cell: sample the active preset's density,
 * bin it to a level, and ink the matching glyph + ramp tint. Presets with
 * `lightning` overstrike a sparse '/' bolt (PAIR_HOT) at their level-4 cores.
 */
static void render_clouds(const Screen *sc, const Drift *drift,
                          const CloudPreset *p)
{
    int top = 2, bottom = sc->rows - 1;          /* skip the 2 HUD rows + hint row */
    int strike_bucket = (int)(drift->time_secs * LIGHTNING_HZ);

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            float d = cloud_density(p, (float)sx, (float)sy,
                                    drift->wind_x, drift->wind_y, drift->warp_t);
            int level = density_to_level(d, p);
            if (level == 0) continue;            /* clear sky — draw nothing */

            char glyph = p->glyphs[level - 1];
            int  pair  = PAIR_RAMP_BASE + p->ramp[level - 1];

            if (lightning_strikes(p, level, sx, sy, strike_bucket)) {
                glyph = '/';
                pair  = PAIR_HOT;
            }
            draw_cell(sy, sx, glyph, pair, level);
        }
    }
}

static void scene_draw(const Screen *sc, const Scene *s)
{
    render_clouds(sc, &s->drift, &presets[s->current_preset]);
}

/* Row 0 right — fps / tick-rate / run-state / wind-speed, right-justified. */
static void hud_status_line(const Screen *sc, const Scene *s,
                            double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED" : "DRIFT ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 0 left — title + preset counter (i/N) and name. */
static void hud_title(const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " CLOUD (fBm)  %2d/%d %-10s ",
             s->current_preset + 1, N_PRESETS,
             presets[s->current_preset].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — theme name, a live ramp swatch in the active palette, and the drift. */
static void hud_param_line(const Scene *s)
{
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    static const char ramp_glyphs[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };
    for (int i = 0; i < 8; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ramp_glyphs[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  wind:%6.1f,%5.1f  warp:%5.2f ",
             (double)s->drift.wind_x, (double)s->drift.wind_y,
             (double)s->drift.warp_t);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the interactive key legend. */
static void hud_key_hints(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:wind  ]/[:tickHz  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);
    hud_status_line(sc, s, fps, sim_fps);
    hud_title(s);
    hud_param_line(s);
    hud_key_hints(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §6  APP — events + the one per-tick combine point                       */
/* ===================================================================== */
/* main() is the SINGLE place that advances simulation state. User events
 * (resize, keys) also mutate state — app_do_resize, app_handle_key — but
 * they are NOT the tick: they fire on input, outside the fixed-timestep
 * loop, and never call scene_tick. */

/*
 * App — the running program: the simulated world plus the runtime that drives
 * it. WHY a type: main() owns exactly one (g_app) so the signal handlers can
 * reach the run-state flags; everything else is reached through it. It is the
 * top of the containment tree (App ▸ Scene ▸ Drift), not a behaviour layer.
 */
typedef struct {
    Scene                 scene;       /* the simulated world (§4)             */
    Screen                screen;      /* the viewport it is painted into (§5) */
    int                   sim_fps;     /* PERFORMANCE: tick rate, SIM_FPS_MIN..MAX
                                        * (10..240); sets the fixed timestep.  */
    volatile sig_atomic_t running;     /* main loop runs while non-zero; cleared
                                        * by SIGINT/SIGTERM or 'q'.            */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH, consumed next frame;
                                        * sig_atomic_t + volatile = safe to
                                        * write from a signal handler.         */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/* cycle an index forward / backward through [0, n) with wraparound */
static inline int wrap_inc(int i, int n) { return (i + 1)     % n; }
static inline int wrap_dec(int i, int n) { return (i + n - 1) % n; }

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                               break;

    case '=': case '+':                       /* double wind speed, capped */
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':                                 /* halve wind speed, floored */
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':                                 /* faster tick, capped */
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':                                 /* slower tick, floored */
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = wrap_inc(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = wrap_dec(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N': s->current_preset = wrap_inc(s->current_preset, N_PRESETS); break;
    case 'p': case 'P': s->current_preset = wrap_dec(s->current_preset, N_PRESETS); break;

    default: break;
    }
    return true;
}

/* app_init — wire up signals/cleanup, set run-state, bring up the terminal
 * and the scene. Everything needed before the first frame; not the tick. */
static void app_init(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);
}

int main(void)
{
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {       /* EVENT (not a tick): resize */
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ---- per-frame combine — the ONLY place sim state advances ----
         * PERFORMANCE: measure + clamp dt
         * SIMULATION : scene_tick × N   (fixed timestep; skipped if paused)
         * PERFORMANCE: fps tally + frame-cap sleep
         * RENDER     : screen_draw (→ scene_draw → LOGIC per cell) + present
         * EVENTS     : getch → app_handle_key   (NOT part of the tick)     */

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_DT_MS * NS_PER_MS) dt = MAX_FRAME_DT_MS * NS_PER_MS;

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
        clock_sleep_ns(NS_PER_SEC / RENDER_FPS_CAP - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
