/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke_v1.c — Volumetric Nuclear Detonation & Mushroom Cloud  (learner-annotated)
 *
 * A high-fidelity mushroom cloud rendered in the terminal via volumetric
 * raymarching. The simulation runs at internal HIGH resolution (2× vertical
 * supersampling) and is downsampled to character cells using top/bottom-
 * weighted glyphs that recover sub-cell vertical detail.
 *
 * Keys:   q/ESC quit  |  space pause  |  r restart  |  +/- speed
 *         n detonate  |  t theme  |  s toggle smoke  |  l toggle lighting
 * Themes: REALISTIC, MATRIX, OCEAN, NOVA, TOXIC  (cycle with 't')
 * Build:  gcc -std=c11 -O2 -Wall -Wextra raymarcher/nuke_v1.c -o nuke_v1 \
 *             -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — all tunable constants (start here to experiment)
 *   §2  clock       — monotonic timer + sleep
 *   §3  color       — 32-entry smoke + 16-entry fire palette × 5 themes
 *   §4  vec3 math   — 3D vector operations
 *   §5  noise/fBm   — value noise, fBm, domain warping
 *   §6  density     — full mushroom anatomy as Gaussian density field
 *   §7  particles   — debris (blast), embers (stem), ash (fall)
 *   §8  simulation  — continuous-time morph, phase HUD label, fall collapse
 *   §9  raymarcher  — Beer–Lambert integrator with 2× vertical supersampling
 *  §10  canvas      — sub-pixel pair buffer + glyph picker
 *  §11  renderer    — bridges simulation → screen, draws HUD
 *  §12  screen      — ncurses terminal setup
 *  §13  app         — main loop
 */

/* ── HOW THIS WORKS ──────────────────────────────────────────────────────── *
 *
 *  PIPELINE (one frame):
 *
 *    sim_tick()         advance time, update blob morph, spawn particles
 *         ↓
 *    renderer_render()  for every terminal cell, fire SS_Y rays:
 *         ↓                a. pixel (col, sub_row) → 3D ray direction
 *    rm_cast_one()         b. march fixed-step through the volume
 *         ↓                c. accumulate density (extinction) + heat (emission)
 *    sub-cell pair          via Beer–Lambert; bail when transmittance < 7%
 *         ↓
 *    glyph picker        top + bottom samples → top-weighted, bottom-weighted,
 *         ↓              full, or middle character
 *    canvas_draw()       final char + (smoke or fire) color attribute
 *
 *  ── KEY TECHNIQUES ───────────────────────────────────────────────────────
 *
 *  Volumetric raymarching (Beer–Lambert)
 *    No surfaces — the cloud is a 3D density field. Each ray walks fixed
 *    steps; at every step the optical depth dτ = density·step·EXTINCTION
 *    attenuates remaining transmittance T by exp(-dτ). Emitted heat is
 *    weighted by (1-exp(-dτ))·T to integrate front-to-back correctly.
 *    Reference: https://www.pbr-book.org/4ed/Volume_Scattering
 *
 *  Soft Gaussian primitives (no SDFs)
 *    Mushroom anatomy = a sum of Gaussian density blobs:
 *      gauss(d², r²·spread)  → smooth falloff, no hard edges
 *    Anisotropic distance dn² = (dx/rx)² + (dy/ry)² + (dz/rz)² lets one
 *    blob morph from sphere → oblate cap by independently scaling rx, ry.
 *
 *  Value noise + fBm + domain warping
 *    hash1() turns integer lattice points into pseudo-random floats.
 *    noise3() interpolates the 8 nearest hashes with a quintic fade.
 *    fbm() sums octaves at doubling frequency / halving amplitude.
 *    Domain warping: evaluate fbm(p + noise(p)) instead of fbm(p) — the
 *    self-offset creates swirling cauliflower turbulence that pure fBm
 *    cannot. Used for the boiling cloud surface.
 *    References: https://iquilezles.org/articles/warp/
 *                Perlin K., "Improving Noise" (SIGGRAPH 2002)
 *
 *  Continuous-time morph (no phase switch)
 *    Every animated parameter is `smoothstep(t_beg, t_end, time)` over
 *    overlapping time windows — the fireball is already rising while the
 *    cap is starting to bulge. Quintic smootherstep on the spread parameter
 *    eliminates residual acceleration at the seams (more fluid).
 *    Reference: https://en.wikipedia.org/wiki/Smoothstep
 *
 *  Vertical supersampling + glyph picker
 *    Terminal cells are ≈2:1 tall, so vertical bandwidth is the limit.
 *    Two rays per cell (top half + bottom half) drive a sub-cell glyph
 *    picker: top-weighted ('"' '`' '^'), bottom-weighted ('.' ',' '_'),
 *    full ('#' '@' '%') or middle (':' ';' '|'). Effectively 2× vertical
 *    resolution at less than 2× the cost (early bail on opaque cells).
 *
 *  Fixed-timestep loop
 *    Physics runs at SIM_FPS regardless of render rate; an accumulator
 *    drains real elapsed time in fixed dt steps. Phase timing stays
 *    consistent whether the terminal renders at 10 or 60 fps.
 *    Reference: https://gafferongames.com/post/fix_your_timestep/
 *
 * ─────────────────────────────────────────────────────────────────────────*/

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

/* ===================================================================== */
/* §1  config — all tunable constants                                     */
/*                                                                        */
/* Everything here is intentionally in one place — to experiment, change  */
/* values here without hunting through the rest of the file.              */
/*                                                                        */
/* Each block is annotated with the physical / visual meaning of the      */
/* constant and a sensible range. Numbers were chosen empirically against */
/* a 24-fps target on a typical 80×24 terminal.                           */
/* ===================================================================== */

enum {
    SIM_FPS        = 24,   /* fixed-timestep physics rate (ticks/sec)      */
    HUD_COLS       = 96,   /* status-line buffer width (chars)             */
    FPS_MS         = 500,  /* fps counter refresh interval (ms)            */
    MAX_PARTICLES  = 240,  /* total particle slots (debris + ember + ash)  */
};

/* ── Supersampling ─────────────────────────────────────────────────────
 * SS_Y=2 fires 2 rays per terminal row (top half + bottom half of each
 * cell). The two samples then drive top-weighted, bottom-weighted, or
 * full glyphs. SS_X=1 because horizontal cell resolution already matches
 * world units after CELL_ASPECT correction.
 */
#define SS_X            1
#define SS_Y            2

/* ── Raymarcher (volumetric) ─────────────────────────────────────────── *
 * Tuned for ~20+ fps at typical terminal sizes. Key wins vs the original:
 *   - cloud_density() has an early bounding-box check that skips the
 *     expensive noise + Gaussian work entirely for empty-space samples
 *     (about 60% of step samples on a typical ray).
 *   - RM_MAX_STEPS lowered from 52 to 40 with a slightly larger RM_STEP;
 *     supersampling hides the lower step density.
 *   - TRANSMIT_BAIL relaxed from 0.04 → 0.07; once a ray is 93% opaque
 *     the remaining contribution is invisible.
 */
#define RM_MAX_STEPS   40     /* max integration steps per ray             */
#define RM_STEP        0.18f  /* fixed step size (world units)             */
#define RM_MAX_DIST    18.0f  /* ray bail-out distance                     */
#define EXTINCTION     3.6f   /* smoke extinction coefficient              */
#define DENSITY_THRESH 0.012f /* skip below this                           */
#define CULL_R_BOMB    3.6f   /* per-bomb bounding sphere — tighter        */
#define TRANSMIT_BAIL  0.07f  /* stop marching once ray is mostly opaque   */

/* ── Camera ─────────────────────────────────────────────────────────── *
 * Camera at +Z looking toward origin. Visible Y window at z=0 is
 * roughly ±CAM_Z·FOV_HALF_TAN·pa ≈ ±2.65 for typical terminals — the
 * cloud is sized to fit comfortably inside that. CELL_ASPECT corrects
 * for the fact that terminal cells are ~2× tall as they are wide; the
 * ray direction's vertical component is multiplied by it so a sphere
 * actually renders round.
 */
#define CAM_Z          6.8f   /* camera Z offset (world units)             */
#define FOV_HALF_TAN   0.70f  /* tan(FOV/2); 0.70 ≈ 70° horizontal FOV     */
#define CELL_ASPECT    2.0f   /* terminal cell h/w ratio                   */

/* ── Blast geometry ─────────────────────────────────────────────────── *
 * The cloud spans roughly y ∈ [BLAST_Y, BLAST_Y + STEM_HEIGHT_MAX + CAP_R_V].
 * The camera sees roughly y ∈ [-pa·CAM_Z·FOV, +pa·CAM_Z·FOV] at z=0,
 * about ±2.6 for typical terminals. We size the cloud so the cap top sits
 * comfortably below +2.0 — earlier values had the cap + pyrocumulus
 * billows poke above the visible frame, looking like a vertical "beam".
 */
#define BLAST_Y       (-2.4f)

#define FIREBALL_R_MAX  1.05f
#define STEM_HEIGHT_MAX 2.55f  /* shorter stem so cap top stays in frame   */
#define STEM_R_BASE     0.20f
#define STEM_R_TOP      0.55f  /* stem widens significantly at the top    */
#define CAP_R_H         1.60f  /* cap horizontal radius                   */
#define CAP_R_V         0.58f  /* cap vertical radius (oblate)            */
#define VORTEX_R_MAJ    1.15f  /* torus major radius — outward roll       */
#define VORTEX_R_MIN    0.40f  /* torus tube radius                       */
#define SKIRT_R_MAJ     1.02f  /* thin condensation collar major          */
#define SKIRT_R_MIN     0.10f  /* skirt tube radius                       */
#define PYRO_COUNT      4      /* pyrocumulus billows on top of cap       */
#define PYRO_OFFSET_R   0.52f  /* horizontal offset of each billow        */
#define PYRO_OFFSET_Y   0.20f  /* vertical offset above cap centre (low!) */
#define PYRO_R          0.38f  /* billow radius                           */

/* ── Surface noise ──────────────────────────────────────────────────── *
 * The cloud sample point is offset by a 3D fBm/warped-fBm displacement
 * vector before the density Gaussian is evaluated. That turns a clean
 * mathematical ellipsoid into a turbulent cauliflower. DISP_AMP is the
 * peak displacement in world units; multiply by `disp_scale` (which
 * grows during the spread) so the smooth fireball boils into a rough
 * mushroom. RISE_BIAS advects the noise sample upward over time so the
 * turbulence appears to lift with the gas.
 */
#define DISP_AMP        0.55f  /* fBm displacement amplitude (world u)    */
#define DISP_FREQ       1.40f  /* base noise frequency on the surface     */
#define DISP_SPD        0.18f  /* surface noise evolution speed           */
#define WARP_AMP        0.50f  /* domain-warp offset strength             */
#define WARP_FREQ       0.78f  /* domain-warp spatial frequency           */
#define RISE_BIAS       0.28f  /* upward noise drift speed (rising gas)   */

/* ── Density Gaussian widths ────────────────────────────────────────── *
 * Each shape contributes  density = gauss(d², r²·SPREAD).
 *   gauss(x, s) = exp(-x/s)
 * SPREAD < 1 → tight, sharp-edged blob; SPREAD > 1 → soft, diffuse.
 * Visually: STEM/SKIRT use wider SPREAD because they're meant to read
 * as wispy connectors; FIRE/CAP use tighter values for a more defined
 * core.
 */
#define FIRE_SPREAD    0.42f   /* main morphing blob (fireball→cap)        */
#define STEM_SPREAD    1.10f   /* tapered cylinder                         */
#define CAP_SPREAD     1.05f   /* legacy — replaced by FIRE_SPREAD on blob */
#define VORTEX_SPREAD  0.85f   /* outward-rolling torus on equator         */
#define SKIRT_SPREAD   1.15f   /* wispy condensation collar at base        */
#define PYRO_SPREAD    0.80f   /* small billows on top of cap              */

/* ── Lighting ───────────────────────────────────────────────────────── *
 * Cheap pseudo-Lambertian: skip true shadow rays and approximate them
 * with a Y-axis self-shadow term (high Y = brighter, since less smoke
 * blocks the sun above) plus a fixed sun-from-the-right side term.
 * Diffuse colour = AMBIENT + DIFFUSE · mix(shadow, side, 0.4)
 */
#define LIGHT_AMBIENT     0.35f /* always-on floor brightness              */
#define LIGHT_DIFFUSE     0.65f /* sun-driven contribution (caps at 1.0)   */
#define SUN_DIR_X         0.45f /* sun unit vector — kept for future use   */
#define SUN_DIR_Y         0.78f
#define SUN_DIR_Z         0.42f

/* ── Shockwave ──────────────────────────────────────────────────────── *
 * Expanding ground ring; radius = SHOCKWAVE_SPD · (t - T_BLOB_GROW_BEG).
 */
#define SHOCKWAVE_SPD  3.6f    /* world units / second                    */

/* ── Particles ──────────────────────────────────────────────────────── *
 * Three particle classes share the same struct, distinguished by `type`.
 *   DEBRIS — fast outward burst, single batch at detonation
 *   EMBER  — slow upward drift along the stem during rise
 *   ASH    — slow downward rain during the fall phase
 * All particles share gravity (PART_GRAVITY) but ash overrides with
 * lighter gravity + air drag + terminal velocity (see particles_tick).
 */
#define DEBRIS_N       50      /* one-off batch on detonation              */
#define EMBER_N        12      /* embers per second during rise            */
#define PART_GRAVITY  (-3.2f)  /* world u/s² (negative = downward)         */
#define DEBRIS_SPD     3.2f    /* peak debris launch speed                 */
#define EMBER_SPD      0.55f   /* peak ember initial vy                    */
#define ASH_PER_SEC    55      /* spawn rate during fall phase             */
#define ASH_FALL_SPD   0.8f    /* initial downward bias on spawn           */

/* ── Continuous-time keypoints (seconds since detonation) ──────────────
 *
 * Single-blob morph model — no separate "fireball" and "cap". One
 * anisotropic ellipsoid (blob_rx, blob_ry, blob_rz=blob_rx) starts
 * spherical and DEFORMS into the cap shape:
 *
 *    rx grows monotonically:  fireball_r → CAP_R_H   (horizontal spread)
 *    ry grows then COMPRESSES: 0 → fireball_r → CAP_R_V (pancakes flat)
 *    blob_y rises smoothly during the rise window
 *    heat fades from 1.0 to ~0.18 as the gas cools
 *
 * That single morphing shape *is* the fireball-becoming-cap. The vortex
 * roll, condensation skirt, and pyrocumulus billows are accessory
 * shapes that fade in late, on top of the morphing blob.
 *
 *    t=0.00   detonation
 *    t=0.05   blob begins growing as a sphere
 *    t=0.35   flash fades out
 *    t=0.80   blob reaches fireball size (sphere)    ── FIREBALL phase
 *    t=0.90   blob begins rising (stem forms)        ── RISING phase
 *    t=2.00   horizontal spread + vertical compression begins (CAP_FORM)
 *    t=3.00   vortex roll begins fading in
 *    t=3.20   blob reaches apex (stem fully extended)
 *    t=3.50   condensation skirt begins fading in
 *    t=3.00   ball begins to pancake (longer pure-ball stage)
 *    t=4.00   vortex roll starts forming on perimeter
 *    t=4.70   condensation skirt fades in
 *    t=5.80   pyrocumulus billows fade in
 *    t=8.00   blob fully morphed into oblate cap ── MUSHROOM phase
 *    t=9.00   accessories fully extended (mushroom is "full")
 *    t=9.00–12.0   PLATEAU — 3-second hold at peak
 *    t=12.0   FALL begins — blob droops, density fades, ash rains down
 *    t=16.5   FALL ends — cloud is gone, ash continues to settle
 *    t=20.5   auto-deactivate
 */
#define T_FLASH_END        0.35f
#define T_BLOB_GROW_BEG    0.05f
#define T_BLOB_GROW_END    0.80f
#define T_RISE_BEG         0.90f
#define T_RISE_END         3.60f
#define T_SPREAD_BEG       3.00f  /* horizontal grow + vertical pancake  */
#define T_SPREAD_END       8.00f
#define T_VORTEX_BEG       4.00f
#define T_VORTEX_END       8.40f
#define T_SKIRT_BEG        4.70f
#define T_SKIRT_END        8.60f
#define T_PYRO_BEG         5.80f
#define T_PYRO_END         9.00f   /* mushroom full at this point         */
#define T_PLATEAU_BEG      9.00f
#define T_PLATEAU_END     12.00f   /* 3-second hold ends                  */
#define T_FALL_BEG        12.00f
#define T_FALL_END        16.50f
#define T_DEACTIVATE      20.50f

/* ── Timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color — themable palette                                           */
/*                                                                        */
/* Each Theme defines:                                                    */
/*   smoke[N_SMOKE]  — 8-step gradient driven by lit-smoke brightness    */
/*   fire [N_FIRE ]  — 16-step gradient driven by emission heat          */
/*   flash/shock/debris/ember/corona — accent colors                     */
/*                                                                        */
/* Cycle with the 't' key. color_init() is re-run on theme change to     */
/* rewrite the curses color pairs in place.                              */
/* ===================================================================== */

#define N_SMOKE   8
#define N_FIRE    16
#define CP_SMOKE_BASE 1
#define CP_FIRE_BASE  (CP_SMOKE_BASE + N_SMOKE)
#define CP_FLASH      25
#define CP_SHOCK      26
#define CP_DEBRIS     27
#define CP_EMBER      28
#define CP_CORONA     29
#define CP_HUD        30

typedef struct {
    const char *name;
    short smoke[N_SMOKE];
    short fire [N_FIRE ];
    short flash, shock, debris, ember, corona;
} Theme;

#define N_THEMES 5

static const Theme g_themes[N_THEMES] = {
    /* REALISTIC — gray smoke + classic orange/yellow/white fire core */
    { .name  = "REALISTIC",
      .smoke = {234, 236, 239, 242, 245, 248, 251, 254},
      .fire  = { 52,  88, 124, 160, 196, 202, 208, 214,
                220, 221, 222, 226, 228, 229, 230, 231},
      .flash = 231, .shock = 123, .debris = 214, .ember = 208, .corona = 220 },

    /* MATRIX — dark green → bright green smoke; green-white fire */
    { .name  = "MATRIX",
      .smoke = { 22,  28,  34,  40,  46,  82, 118, 154},
      .fire  = { 22,  28,  34,  40,  46,  82, 118, 154,
                190, 191, 192, 226, 228, 229, 230, 231},
      .flash = 231, .shock =  46, .debris = 154, .ember =  82, .corona = 154 },

    /* OCEAN — deep blue smoke; cyan/white "underwater" core */
    { .name  = "OCEAN",
      .smoke = { 17,  18,  19,  24,  31,  38,  45,  51},
      .fire  = { 17,  18,  19,  24,  31,  38,  45,  51,
                 87, 117, 123, 159, 195, 219, 230, 231},
      .flash = 231, .shock =  51, .debris =  51, .ember =  39, .corona = 195 },

    /* NOVA — deep purple → magenta → pink → white (stellar explosion) */
    { .name  = "NOVA",
      .smoke = { 53,  54,  55,  89,  90, 125, 161, 197},
      .fire  = { 53,  89,  90, 125, 161, 197, 198, 199,
                200, 207, 213, 219, 225, 226, 230, 231},
      .flash = 231, .shock = 219, .debris = 213, .ember = 199, .corona = 226 },

    /* TOXIC — sickly green → mustard yellow → bright yellow (radioactive) */
    { .name  = "TOXIC",
      .smoke = { 22,  28,  64, 100, 142, 178, 184, 220},
      .fire  = { 22,  28,  64, 100, 142, 178, 184, 220,
                221, 222, 154, 191, 192, 226, 228, 231},
      .flash = 231, .shock = 154, .debris = 226, .ember = 154, .corona = 226 },
};

static int g_theme_idx = 0;

static void color_init(void)
{
    start_color();
    use_default_colors();
    const Theme *th = &g_themes[g_theme_idx];
    if (COLORS >= 256) {
        for (int i = 0; i < N_SMOKE; i++)
            init_pair((short)(CP_SMOKE_BASE + i), th->smoke[i], -1);
        for (int i = 0; i < N_FIRE; i++)
            init_pair((short)(CP_FIRE_BASE + i), th->fire[i], -1);
        init_pair(CP_FLASH,  th->flash,  -1);
        init_pair(CP_SHOCK,  th->shock,  -1);
        init_pair(CP_DEBRIS, th->debris, -1);
        init_pair(CP_EMBER,  th->ember,  -1);
        init_pair(CP_CORONA, th->corona, -1);
        init_pair(CP_HUD,    231, -1);
    } else {
        /* 8-color fallback: only realistic theme makes sense, others map
         * loosely to nearest ANSI primary. */
        short fb_smoke = COLOR_WHITE, fb_fire = COLOR_RED;
        switch (g_theme_idx) {
            case 1: fb_smoke = COLOR_GREEN;   fb_fire = COLOR_GREEN;   break;
            case 2: fb_smoke = COLOR_BLUE;    fb_fire = COLOR_CYAN;    break;
            case 3: fb_smoke = COLOR_MAGENTA; fb_fire = COLOR_MAGENTA; break;
            case 4: fb_smoke = COLOR_GREEN;   fb_fire = COLOR_YELLOW;  break;
        }
        for (int i = 0; i < N_SMOKE; i++)
            init_pair((short)(CP_SMOKE_BASE + i), fb_smoke, -1);
        for (int i = 0; i < N_FIRE;  i++)
            init_pair((short)(CP_FIRE_BASE + i), fb_fire, -1);
        init_pair(CP_FLASH,  COLOR_WHITE,  -1);
        init_pair(CP_SHOCK,  COLOR_CYAN,   -1);
        init_pair(CP_DEBRIS, COLOR_YELLOW, -1);
        init_pair(CP_EMBER,  fb_fire,      -1);
        init_pair(CP_CORONA, COLOR_YELLOW, -1);
        init_pair(CP_HUD,    COLOR_WHITE,  -1);
    }
}

static void cycle_theme(void)
{
    g_theme_idx = (g_theme_idx + 1) % N_THEMES;
    color_init();
}

/* heat ∈ [0,1] picks a fire palette entry; brightness ∈ [0,1] picks the
 * smoke palette entry. The renderer chooses which palette based on heat. */
static attr_t fire_pair(float heat)
{
    if (heat < 0.f) heat = 0.f;
    if (heat > 1.f) heat = 1.f;
    int i = (int)(heat * (float)(N_FIRE - 1));
    if (i >= N_FIRE) i = N_FIRE - 1;
    attr_t a = COLOR_PAIR(CP_FIRE_BASE + i);
    if (heat > 0.55f) a |= A_BOLD;
    return a;
}

static attr_t smoke_pair(float bright)
{
    if (bright < 0.f) bright = 0.f;
    if (bright > 1.f) bright = 1.f;
    int i = (int)(bright * (float)(N_SMOKE - 1));
    if (i >= N_SMOKE) i = N_SMOKE - 1;
    attr_t a = COLOR_PAIR(CP_SMOKE_BASE + i);
    if (bright > 0.75f) a |= A_BOLD;
    return a;
}

/* ===================================================================== */
/* §4  vec3 math + scalar utilities                                       */
/*                                                                        */
/* Minimal — only what the simulation needs. The smoothstep / smootherstep*/
/* helpers are critical for the continuous-time morph: every animated     */
/* parameter is `smoothstep(t_beg, t_end, time)`, so the easing curve     */
/* directly shapes the visual motion.                                     */
/*                                                                        */
/*   smoothstep    cubic 3t²-2t³  — first derivative vanishes at endpoints*/
/*   smootherstep  quintic 6t⁵-15t⁴+10t³ — first AND second derivatives   */
/*                 vanish; motion has zero acceleration at the seams,     */
/*                 visibly more fluid than cubic.                         */
/*                                                                        */
/* Reference: https://en.wikipedia.org/wiki/Smoothstep                    */
/*            Perlin K., "Improving Noise" (SIGGRAPH 2002)                */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3(float x, float y, float z)  { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)              { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3sub(V3 a, V3 b)              { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3mul(V3 a, float s)           { return v3(a.x*s, a.y*s, a.z*s); }
static inline float v3dot(V3 a, V3 b)              { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3len(V3 a)                    { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a)
{
    float l = v3len(a); return l > 1e-7f ? v3mul(a, 1.f / l) : v3(0, 0, 1);
}

static inline float clmpf(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
static inline float lerpf(float a, float b, float t)   { return a + (b - a) * t; }

/* Cubic Hermite easing: 0 below e0, 1 above e1, S-curve in between.
 * 1st derivative is zero at the endpoints. */
static inline float smoothstep(float e0, float e1, float x)
{
    float t = clmpf((x - e0) / (e1 - e0), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

/* Quintic Hermite (Ken Perlin's "smootherstep"). Both 1st AND 2nd
 * derivatives vanish at the endpoints, so motion has zero acceleration
 * at the seams. Used on the ball-to-mushroom morph where any residual
 * acceleration reads as "snapping". */
static inline float smootherstep(float e0, float e1, float x)
{
    float t = clmpf((x - e0) / (e1 - e0), 0.f, 1.f);
    return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}

/* ===================================================================== */
/* §5  hash + value noise + fBm + domain warping                          */
/*                                                                        */
/* Builds organic, cloud-like 3D texture from integer hash functions.     */
/*                                                                        */
/*   hash1()      integer lattice → float ∈ [0,1]   (the random primitive)*/
/*   noise3()     smooth 3D value noise              (trilinear + quintic)*/
/*   fbm4 / fbm2  fractional Brownian motion         (layered noise)      */
/*   warped_fbm() domain-warped fBm                  (swirling vortices)  */
/*                                                                        */
/* References: https://iquilezles.org/articles/warp/                     */
/*             Perlin K., "Improving Noise" (SIGGRAPH 2002)              */
/*             https://en.wikipedia.org/wiki/Value_noise                  */
/* ===================================================================== */

/*
 * hash1() — integer lattice point → pseudo-random float ∈ [0, 1].
 * Multiply-xorshift pair; fast, no state, good enough distribution for
 * visual noise. Different multipliers/seeds = different noise patterns.
 */
static inline float hash1(int ix, int iy, int iz)
{
    unsigned u = (unsigned)(ix*127 + iy*311 + iz*743 + ix*iy*17 + iy*iz*29);
    u = (u ^ (u >> 16)) * 0x45d9f3bU;
    u = (u ^ (u >> 16)) * 0x45d9f3bU;
    u = u ^ (u >> 16);
    return (float)(u & 0xFFFFFF) / (float)0x1000000;
}

/*
 * noise3() — smooth 3D value noise via trilinear interpolation.
 *
 * 1. Find the 8 integer lattice corners around p.
 * 2. Hash each corner to a random scalar.
 * 3. Interpolate using a quintic fade — t³(6t²-15t+10) — which is
 *    C2-continuous, so we don't see lattice creases.
 *
 * "Value noise" (sample-then-interpolate scalar values) is cheaper than
 * Perlin's gradient noise but slightly less isotropic — fine for fBm
 * stacks where multiple octaves wash out the directional artifacts.
 */
static float noise3(V3 p)
{
    int ix=(int)floorf(p.x), iy=(int)floorf(p.y), iz=(int)floorf(p.z);
    float fx=p.x-floorf(p.x), fy=p.y-floorf(p.y), fz=p.z-floorf(p.z);
    /* Quintic fade — same curve as smootherstep, shapes the interpolation. */
    float ux=fx*fx*fx*(fx*(fx*6.f-15.f)+10.f);
    float uy=fy*fy*fy*(fy*(fy*6.f-15.f)+10.f);
    float uz=fz*fz*fz*(fz*(fz*6.f-15.f)+10.f);
    /* Hash the 8 corners of the unit cube around p. */
    float v000=hash1(ix,iy,iz),     v100=hash1(ix+1,iy,iz);
    float v010=hash1(ix,iy+1,iz),   v110=hash1(ix+1,iy+1,iz);
    float v001=hash1(ix,iy,iz+1),   v101=hash1(ix+1,iy,iz+1);
    float v011=hash1(ix,iy+1,iz+1), v111=hash1(ix+1,iy+1,iz+1);
    /* Trilinear interpolation: across x, then y, then z. */
    float x0=lerpf(v000,v100,ux), x1=lerpf(v010,v110,ux);
    float x2=lerpf(v001,v101,ux), x3=lerpf(v011,v111,ux);
    float y0=lerpf(x0,x1,uy),     y1=lerpf(x2,x3,uy);
    return lerpf(y0, y1, uz);
}

/*
 * fbm4() — 4-octave fractional Brownian motion.
 *
 * Σ amp · (noise3(p · freq) - 0.5)  for octaves 0..3
 * Each octave: freq ×= 2.1 (slightly irrational to avoid lattice
 * alignment between octaves), amp ×= 0.5 (geometric series → bounded sum).
 * Returns ≈ [-0.5, 0.5]. Higher octaves contribute fine detail; lower
 * octaves shape the large lumps. 4 octaves is the sweet spot here:
 * 2 looks too smooth, 6+ adds cost without visible gain at terminal res.
 */
static float fbm4(V3 p)
{
    float v=0.f, amp=0.5f, freq=1.f;
    for (int i=0; i<4; i++) {
        v    += amp * (noise3(v3mul(p, freq)) - 0.5f);
        amp  *= 0.5f;
        freq *= 2.1f;  /* irrational ratio prevents lattice alignment */
    }
    return v;
}

/* fbm2() — cheap 2-octave variant for the secondary Y/Z displacement
 * axes (where rough turbulence is enough; the X axis carries the
 * dominant warped fBm). Saves ~3 noise3 calls per displaced sample. */
static float fbm2(V3 p)
{
    float v = (noise3(p) - 0.5f) * 0.5f;
    v += (noise3(v3mul(p, 2.1f)) - 0.5f) * 0.25f;
    return v;
}

/*
 * warped_fbm() — two-pass domain warping.
 *
 * Pass 1: sample three independent noise3 fields at p → random offset q
 * Pass 2: evaluate fbm4 at (p + q · WARP_AMP) instead of at p
 *
 * The self-offset breaks the regular structure of pure fBm, producing
 * hurricanes and vortices that look like real turbulent flow — what
 * gives the cloud its "boiling cauliflower" character.
 *
 * The time advection (pa = p + time*DISP_SPD) makes the noise pattern
 * drift over time so the surface evolves rather than sitting frozen.
 */
static float warped_fbm(V3 p, float t)
{
    V3 pa = v3add(p, v3(t * DISP_SPD, t * DISP_SPD * 0.6f, 0.f));
    V3 warp = v3(
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(1.7f, 9.2f, 0.f))),
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(8.3f, 2.8f, 0.f))),
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(3.1f, 5.4f, 0.f)))
    );
    V3 warped = v3add(pa, v3mul(warp, WARP_AMP));
    return fbm4(v3mul(warped, DISP_FREQ));
}

/* ===================================================================== */
/* §6  density + heat — mushroom anatomy as a Gaussian volume             */
/*                                                                        */
/* The cloud is NOT a surface, NOT an SDF — it's a 3D scalar density      */
/* field. cloud_density(p) returns "how much smoke" at point p, and the   */
/* raymarcher (§9) integrates it along each ray (Beer–Lambert).           */
/*                                                                        */
/* The field is a sum of soft Gaussian blobs:                             */
/*                                                                        */
/*   1. main blob   anisotropic ellipsoid that morphs continuously        */
/*                  from sphere (fireball) to oblate cap (mushroom). This */
/*                  is THE cloud — there is no separate "fireball" that   */
/*                  disappears and "cap" that materialises.               */
/*   2. stem        Gaussian cylinder, wider at top, tapered at base      */
/*   3. vortex      torus on the equator (rolling outward edge)           */
/*   4. skirt       thin torus collar at the base of the blob             */
/*   5. pyrocumulus 4 rotating billows above the blob                     */
/*                                                                        */
/* Why Gaussians (not SDFs)? Real smoke is fuzzy and semi-transparent —   */
/* there is no surface to define. Gaussians give natural soft falloff     */
/* with cheap math (one expf per sample) and sum gracefully when shapes   */
/* overlap (you get a smooth bulge, not a hard intersection).             */
/*                                                                        */
/* The whole sample point is also pre-displaced by a 3D noise vector      */
/* before density evaluation, which turns the smooth math shape into a    */
/* turbulent cauliflower (see warped_fbm in §5).                          */
/* ===================================================================== */

/*
 * gauss() — Gaussian falloff: exp(-d² / r²·SPREAD).
 *
 * Returns 1 at the centre (d=0), ≈0.37 at d² = r²·SPREAD, near-zero far
 * out. The "spread" widening factor lets each shape choose how soft its
 * edges are without changing its nominal radius.
 */
static inline float gauss(float d2, float r2)
{
    return expf(-d2 / fmaxf(r2, 1e-6f));
}

/*
 * torus_d2() — squared distance from point p to the centreline of a
 * torus around the Y axis at height cy with major radius major_r.
 *
 * Math: project p to the XZ plane, compute distance from the centre
 * circle (sqrt(x²+z²) - major_r), combine with the Y offset. Returns
 * the squared distance so it can be plugged straight into gauss().
 */
static inline float torus_d2(V3 p, float cy, float major_r)
{
    float ax = sqrtf(p.x*p.x + p.z*p.z) - major_r;
    float ty = p.y - cy;
    return ax*ax + ty*ty;
}

/*
 * ellipsoid_dn2() — normalised squared distance from p to the centre of
 * an axis-aligned ellipsoid with radii (rx, ry, rz).
 *
 * Trick: dn² = (x/rx)² + (y/ry)² + (z/rz)² makes the surface dn²=1 for
 * ANY anisotropy. So `gauss(dn², s)` gives the same falloff shape no
 * matter how oblate the ellipsoid is — change rx/ry/rz independently
 * and the same Gaussian morphs into a new shape, no other math changes.
 * This is what lets the main blob deform from sphere to oblate cap with
 * one radii change.
 */
static inline float ellipsoid_dn2(V3 p, V3 c, float rx, float ry, float rz)
{
    float dx = (p.x - c.x) / rx;
    float dy = (p.y - c.y) / ry;
    float dz = (p.z - c.z) / rz;
    return dx*dx + dy*dy + dz*dz;
}

/*
 * noise_displace() — return the world-space displacement vector applied
 * to a sample point before density evaluation.
 *
 * Three independent 3D noise channels (one warped fBm + two cheap fBms)
 * give X / Y / Z offsets — independent axes so the noise actually swirls
 * instead of just radially puffing. The sample point is pre-advected
 * upward (RISE_BIAS) so the turbulence appears to lift with the gas.
 *
 * The Y/Z amplitudes are slightly reduced (×0.85, ×0.90) so the
 * displaced point doesn't drift quite as much vertically — keeps the
 * pancake silhouette from getting too noisy at the top.
 */
static inline V3 noise_displace(V3 p, float time, float disp_amp)
{
    V3 pa = v3(p.x, p.y - time * RISE_BIAS, p.z);
    float nx = warped_fbm(pa, time);
    float ny = fbm2(v3add(v3mul(pa, DISP_FREQ), v3(5.3f, 1.7f, 3.2f)));
    float nz = fbm2(v3add(v3mul(pa, DISP_FREQ), v3(2.9f, 8.1f, 0.7f)));
    return v3(nx * disp_amp,
              ny * disp_amp * 0.85f,
              nz * disp_amp * 0.90f);
}

/*
 * cloud_density() — total smoke/gas density at world point p.
 *
 * Single morphing blob model: the main cloud is one anisotropic ellipsoid
 * that deforms continuously from sphere-fireball to oblate cap. There is
 * no separate "fireball that disappears" and "cap that materialises" —
 * the same blob spreads sideways and pancakes vertically as the gas
 * convects outward. Vortex/skirt/pyro are accessory shapes layered on top.
 *
 *   blob_rx,ry,y  — current ellipsoid (rz = rx, axisymmetric)
 *   stem_h        — stem height above BLAST_Y
 *   vortex_scale  — 0..1, vortex roll on blob perimeter
 *   skirt_scale   — 0..1, condensation collar at cap base
 *   pyro_scale    — 0..1, pyrocumulus billows on top
 *   disp_scale    — 0..1, noise displacement amount (smooth ball → cauliflower)
 */
static float cloud_density(V3 p, float time,
                            float blob_rx, float blob_ry, float blob_y,
                            float stem_h,
                            float vortex_scale, float skirt_scale,
                            float pyro_scale, float disp_scale,
                            float cloud_fade)
{
    if (cloud_fade <= 0.f) return 0.f;
    /* ── EARLY BOUNDS CHECK ────────────────────────────────────────────
     * Roughly 60% of step samples on a hit ray are still in empty space
     * around the cloud (entering, exiting, or between shapes). Returning
     * 0 here BEFORE the noise + Gaussian work is the single biggest perf
     * win. The bounds adapt to the current shape sizes.
     */
    float disp_amp = DISP_AMP * disp_scale;
    {
        float r_horiz2 = p.x*p.x + p.z*p.z;

        /* Widest possible horizontal extent: blob, plus optional skirt
         * outflow once it's forming. */
        float r_max = blob_rx;
        if (skirt_scale > 0.05f) r_max = fmaxf(r_max, SKIRT_R_MAJ);
        if (stem_h > 0.05f)      r_max = fmaxf(r_max, STEM_R_TOP);
        r_max += disp_amp + 0.15f;
        if (r_horiz2 > r_max * r_max) return 0.f;

        /* Vertical extent: top is blob_y + ry (+ pyro offset if active);
         * bottom is the blast site. Both padded for displacement. */
        float y_top = blob_y + blob_ry;
        if (pyro_scale > 0.05f)
            y_top = fmaxf(y_top, blob_y + PYRO_OFFSET_Y * pyro_scale + PYRO_R);
        y_top += disp_amp * 0.95f;
        float y_bot = BLAST_Y - disp_amp * 0.4f;
        if (p.y > y_top || p.y < y_bot) return 0.f;
    }

    /* Pre-displace the sample point: turns a smooth ellipsoid into a
     * turbulent cauliflower. Amplitude rises with `disp_scale`, so the
     * smooth fireball boils into rough mushroom as the gas spreads. */
    V3 pp = v3add(p, noise_displace(p, time, disp_amp));

    float d = 0.f;

    /* ── 1. Main blob — anisotropic ellipsoid Gaussian.
     * ellipsoid_dn2 normalises so the surface is always dn²=1 regardless
     * of (rx, ry, rz). When ry shrinks and rx grows during the spread,
     * the same Gaussian smoothly morphs from sphere to oblate cap. */
    if (blob_rx > 0.001f && blob_ry > 0.001f) {
        V3 c = v3(0, blob_y, 0);
        d += gauss(ellipsoid_dn2(pp, c, blob_rx, blob_ry, blob_rx),
                   FIRE_SPREAD);
    }

    /* ── 2. Stem — Gaussian cylinder, wider at top ───────────────────── */
    if (stem_h > 0.05f) {
        float ax2    = pp.x*pp.x + pp.z*pp.z;
        float y_frac = clmpf((pp.y - BLAST_Y) / stem_h, 0.f, 1.f);
        float R      = lerpf(STEM_R_BASE, STEM_R_TOP, y_frac);
        /* taper at top so stem blends into blob, taper at base too */
        float ym     = smoothstep(0.f, 0.10f, y_frac)
                     * smoothstep(1.10f, 0.78f, y_frac);
        d += gauss(ax2, R * R * STEM_SPREAD) * ym;
    }

    /* ── 3. Vortex roll — torus on the blob's equator.
     * Major radius rides the blob's current horizontal extent so the roll
     * stays glued to the spreading edge instead of snapping to a fixed size. */
    if (vortex_scale > 0.01f) {
        V3 cp = v3sub(pp, v3(0, blob_y, 0));
        float vmaj  = lerpf(blob_rx * 0.85f, VORTEX_R_MAJ, vortex_scale);
        float vmin  = VORTEX_R_MIN * vortex_scale;
        float vmin2 = vmin * vmin;
        float td2   = torus_d2(cp, 0.f, vmaj);
        d += gauss(td2, vmin2 * VORTEX_SPREAD) * 0.55f * vortex_scale;
    }

    /* ── 4. Skirt — thin condensation collar at the blob's base. */
    if (skirt_scale > 0.01f) {
        float smaj  = SKIRT_R_MAJ * skirt_scale;
        float smin2 = SKIRT_R_MIN * SKIRT_R_MIN;
        V3    sp    = v3sub(pp, v3(0, blob_y - blob_ry * 0.85f, 0));
        d += gauss(torus_d2(sp, 0.f, smaj), smin2 * SKIRT_SPREAD)
           * 0.45f * skirt_scale;
    }

    /* ── 5. Pyrocumulus — small billows scattered on top of the blob. */
    if (pyro_scale > 0.01f) {
        V3 cp = v3sub(pp, v3(0, blob_y, 0));
        float pyro_r2 = PYRO_R * PYRO_R * pyro_scale * pyro_scale;
        for (int i = 0; i < PYRO_COUNT; i++) {
            float ang = (float)i * (float)(2.0 * M_PI / PYRO_COUNT)
                      + time * 0.12f;
            V3 bp = v3(cosf(ang) * PYRO_OFFSET_R * pyro_scale,
                       PYRO_OFFSET_Y * pyro_scale,
                       sinf(ang) * PYRO_OFFSET_R * pyro_scale);
            V3 dd = v3sub(cp, bp);
            d += gauss(v3dot(dd, dd), pyro_r2 * PYRO_SPREAD)
               * 0.40f * pyro_scale;
        }
    }

    return clmpf(d * cloud_fade, 0.f, 2.5f);
}

/*
 * cloud_heat() — temperature at world point p, ∈ [0,1].
 *
 * Heat lives in the core of the morphing blob and fades as the gas cools
 * (blob_heat parameter). Stem inherits a warm interior near its base.
 * Anisotropic core — same ellipsoid math as the density blob — so the
 * hot region pancakes naturally as the blob spreads.
 */
static float cloud_heat(V3 p,
                         float blob_rx, float blob_ry, float blob_y,
                         float blob_heat, float stem_h)
{
    float heat = 0.f;

    if (blob_rx > 0.001f && blob_ry > 0.001f) {
        /* Hot core occupies the inner ~55% of the blob radii. Anisotropic
         * normalisation pancakes the heat region together with the gas. */
        V3 c = v3(0, blob_y, 0);
        float dn2 = ellipsoid_dn2(p, c, blob_rx*0.55f, blob_ry*0.55f, blob_rx*0.55f);
        heat = fmaxf(heat, blob_heat * gauss(dn2, 0.35f));
    }

    if (stem_h > 0.05f) {
        float y_frac = clmpf((p.y - BLAST_Y) / stem_h, 0.f, 1.f);
        float ax2    = p.x*p.x + p.z*p.z;
        float fall   = gauss(ax2, STEM_R_TOP * STEM_R_TOP * 1.6f);
        /* Stem heat tied to blob_heat so it fades with the cooling gas. */
        heat = fmaxf(heat, lerpf(0.55f, 0.10f, y_frac) * fall * blob_heat);
    }

    return clmpf(heat, 0.f, 1.f);
}

/* ===================================================================== */
/* §7  particles — debris + embers + ash                                  */
/*                                                                        */
/* Three classes of point-particles share the same Particle struct,       */
/* distinguished by `type` field:                                         */
/*                                                                        */
/*   DEBRIS  fast outward burst from the blast site, single batch on      */
/*           detonation. Lifetime ~1–2 s. Subject to gravity.             */
/*   EMBER   slow upward drift along the stem during the rise window.    */
/*           Continuously spawned. Lifetime ~1.5–3.5 s.                   */
/*   ASH     slow downward rain during the fall phase. Continuously       */
/*           spawned from inside the blob volume. Lifetime ~2.5–5 s.      */
/*           Lighter gravity + air drag + terminal velocity (see          */
/*           particles_tick) so it drifts rather than free-falls.         */
/*                                                                        */
/* Pool: a fixed array of MAX_PARTICLES slots, allocated by linear scan   */
/* (cheap; the pool is small). New particles fail silently if the pool    */
/* is full — visually you just get fewer ash dots, no crash.              */
/*                                                                        */
/* Particles are projected to screen during draw using the same camera   */
/* parameters as the raymarcher, so they sit correctly in the scene.    */
/* ===================================================================== */

typedef enum { PART_DEBRIS = 0, PART_EMBER = 1, PART_ASH = 2 } PartType;

typedef struct {
    float    x, y, z;            /* position in world units                 */
    float    vx, vy, vz;         /* velocity in world u/s                   */
    float    life, max_life;     /* remaining / initial lifetime (seconds)  */
    PartType type;
    bool     alive;
} Particle;

static Particle g_parts[MAX_PARTICLES];
static unsigned g_pseed = 12345u;  /* LCG state — visual variety, not crypto */

/*
 * prand() — LCG pseudo-random ∈ [0, 1].
 * Numerical Recipes' parameters; tiny, no allocation, good enough for
 * particle scatter. Reference: https://en.wikipedia.org/wiki/Linear_congruential_generator
 */
static float prand(void)
{
    g_pseed = g_pseed * 1664525u + 1013904223u;
    return (float)((g_pseed >> 8) & 0xFFFF) / 65535.f;
}

/* Linear scan for a free slot in the particle pool. Returns NULL if
 * the pool is saturated — caller should silently bail out. */
static Particle *part_alloc(void)
{
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (!g_parts[i].alive) return &g_parts[i];
    return NULL;
}

static void particles_spawn_debris(void)
{
    for (int i = 0; i < DEBRIS_N; i++) {
        Particle *p = part_alloc();
        if (!p) break;
        float theta = prand() * (float)(2.0 * M_PI);
        float phi   = acosf(1.f - 2.f * prand());
        V3 dir = v3(sinf(phi)*cosf(theta), sinf(phi)*sinf(theta), cosf(phi));
        dir.y = fabsf(dir.y) * (0.3f + prand() * 0.7f);
        dir   = v3norm(dir);
        float spd = DEBRIS_SPD * (0.4f + prand() * 0.8f);
        p->x = 0; p->y = BLAST_Y; p->z = 0;
        p->vx = dir.x * spd;
        p->vy = dir.y * spd;
        p->vz = dir.z * spd;
        p->max_life = p->life = 0.8f + prand() * 1.4f;
        p->type  = PART_DEBRIS;
        p->alive = true;
    }
}

static void particles_spawn_ember(float stem_h)
{
    Particle *p = part_alloc();
    if (!p) return;
    float theta = prand() * (float)(2.0 * M_PI);
    float y_off = prand() * stem_h;
    float r = STEM_R_BASE * (0.4f + prand() * 0.8f);
    p->x = cosf(theta) * r;
    p->y = BLAST_Y + y_off;
    p->z = sinf(theta) * r;
    p->vx = (prand() - 0.5f) * 0.3f;
    p->vy = EMBER_SPD * (0.5f + prand() * 0.8f);
    p->vz = (prand() - 0.5f) * 0.3f;
    p->max_life = p->life = 1.5f + prand() * 2.0f;
    p->type  = PART_EMBER;
    p->alive = true;
}

/* Ash falls from random points within the current blob volume — gravity
 * already pulls everything down; ash just gets a small initial drift.
 * Long life so the rain stays on screen as the cloud fades. */
static void particles_spawn_ash(float blob_rx, float blob_ry, float blob_y)
{
    Particle *p = part_alloc();
    if (!p) return;
    /* Random point inside an oblate ellipsoid (rejection-free): */
    float theta = prand() * (float)(2.0 * M_PI);
    float r     = sqrtf(prand()) * blob_rx * 0.95f;
    float yfrac = (prand() * 2.f - 1.f);
    p->x = cosf(theta) * r;
    p->y = blob_y + yfrac * blob_ry * 0.95f;
    p->z = sinf(theta) * r;
    p->vx = (prand() - 0.5f) * 0.4f;
    p->vy = -ASH_FALL_SPD * (0.5f + prand() * 0.6f);
    p->vz = (prand() - 0.5f) * 0.4f;
    p->max_life = p->life = 2.5f + prand() * 2.5f;
    p->type  = PART_ASH;
    p->alive = true;
}

static void particles_tick(float dt)
{
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_parts[i];
        if (!p->alive) continue;
        /* Ash terminal velocity: lighter than debris, drag-limited. */
        float g = (p->type == PART_ASH) ? PART_GRAVITY * 0.35f : PART_GRAVITY;
        p->vy += g * dt;
        if (p->type == PART_ASH) {
            /* Air drag on horizontal motion so ash drifts then falls straight. */
            p->vx *= expf(-1.4f * dt);
            p->vz *= expf(-1.4f * dt);
            if (p->vy < -1.6f) p->vy = -1.6f;   /* terminal velocity */
        }
        p->x  += p->vx * dt;
        p->y  += p->vy * dt;
        p->z  += p->vz * dt;
        p->life -= dt;
        if (p->life <= 0.f || p->y < BLAST_Y - 2.f) p->alive = false;
    }
}

static void particles_draw(int cols, int rows, int canv_ox, int canv_oy,
                            int cw, int ch)
{
    float pa = ((float)ch * CELL_ASPECT) / (float)cw;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &g_parts[i];
        if (!p->alive) continue;
        float cam_dz = CAM_Z - p->z;
        if (cam_dz < 0.1f) continue;
        float u =  p->x / (cam_dz * FOV_HALF_TAN);
        float v = -p->y / (cam_dz * FOV_HALF_TAN * pa);
        int cx = (int)((u + 1.f) * 0.5f * (float)cw);
        int cy = (int)((v + 1.f) * 0.5f * (float)ch);
        int tx = canv_ox + cx, ty = canv_oy + cy;
        if (tx < 0 || tx >= cols || ty < 0 || ty >= rows) continue;
        float age     = 1.f - p->life / p->max_life;
        float bright  = smoothstep(0.f, 0.05f, 1.f - age) * smoothstep(1.f, 0.75f, age);
        if (bright < 0.05f) continue;
        char ch_p; attr_t a;
        if (p->type == PART_DEBRIS) {
            ch_p = (bright > 0.6f) ? '*' : '.';
            a = COLOR_PAIR(CP_DEBRIS) | (bright > 0.5f ? A_BOLD : 0);
        } else if (p->type == PART_EMBER) {
            ch_p = (bright > 0.5f) ? '+' : '.';
            a = COLOR_PAIR(CP_EMBER);
        } else { /* PART_ASH */
            /* Map brightness onto a low band of the smoke palette. */
            int idx = (int)(bright * 3.f);
            if (idx > 3) idx = 3;
            ch_p = (bright > 0.55f) ? ',' : '.';
            a = COLOR_PAIR(CP_SMOKE_BASE + idx);
        }
        attron(a); mvaddch(ty, tx, (chtype)(unsigned char)ch_p); attroff(a);
    }
}

/* ===================================================================== */
/* §8  simulation — continuous-time geometry                              */
/*                                                                        */
/* DESIGN NOTE — why no phase switch.                                     */
/*                                                                        */
/* An earlier version of this file used a hard switch(phase) state        */
/* machine: each phase set the parameters of "its" shapes and zeroed the */
/* others. That made transitions feel like sprite frames — the stem      */
/* appeared suddenly at FIREBALL → RISING; the cap snapped in at         */
/* RISING → CAP_FORM. The motion was synced to phase boundaries instead  */
/* of evolving with the gas.                                              */
/*                                                                        */
/* The current version derives every animated parameter from absolute    */
/* simulated time using overlapping `smoothstep(t_beg, t_end, time)`     */
/* windows. The fireball is already rising while the cap is starting to  */
/* bulge; nothing snaps from 0 to non-zero. The phase enum survives only */
/* as a HUD label — no game-logic branches on it.                        */
/*                                                                        */
/* CORE MORPH — single anisotropic blob (lives in NukeState):             */
/*   blob_rx (horizontal):  monotonic 0 → fireball_r → CAP_R_H            */
/*                          (the spreading-gas motion)                    */
/*   blob_ry (vertical):    grows to fireball_r, THEN compresses to       */
/*                          CAP_R_V (the pancake motion)                  */
/*   blob_y:                rises smoothly during the rise window         */
/*   blob_heat:             cools 1.0 → 0.18 across the spread            */
/*   disp_scale:            noise turbulence ramps 0.35 → 1.0             */
/*   cloud_fade:            stays 1.0 until FALL phase, then 1.0 → 0.0   */
/*                                                                        */
/* ACCESSORIES (vortex_scale, skirt_scale, pyro_scale): each is a         */
/* `smoothstep` over its own time window, so they fade in late and       */
/* decorate the morphed blob without ever snapping into existence.       */
/* ===================================================================== */

typedef enum {
    NK_FLASH, NK_FIREBALL, NK_RISING, NK_CAP_FORM, NK_MUSHROOM,
    NK_PLATEAU, NK_FALL, NK_SETTLE
} NukePhase;

static const char *k_phase_names[] = {
    "FLASH", "FIREBALL", "RISING", "CAP FORM", "MUSHROOM",
    "PLATEAU", "FALLING", "SETTLE"
};

typedef struct {
    NukePhase phase;        /* derived from time — for HUD only           */
    float     time;         /* seconds of simulated time since detonation */
    float     time_scale;
    bool      paused;
    bool      done;

    /* MAIN BLOB — single anisotropic ellipsoid that morphs from
     * sphere (fireball) to oblate spheroid (cap). No separate cap shape:
     * the blob just gets fatter and shorter as the gas spreads. */
    float     blob_rx;        /* horizontal radius (= rz, axisymmetric)  */
    float     blob_ry;        /* vertical radius — grows then compresses */
    float     blob_y;         /* Y position — rises during rise window   */
    float     blob_heat;      /* core temperature, 1.0 → ~0.18           */
    float     disp_scale;     /* noise displacement amount, 0.35 → 1.0   */

    /* ACCESSORY shapes — fade in late, decorate the morphed blob. */
    float     stem_h;         /* tracks blob_y - BLAST_Y                 */
    float     vortex_scale;   /* torus roll at cap perimeter             */
    float     skirt_scale;    /* condensation collar                     */
    float     pyro_scale;     /* pyrocumulus billows on top              */

    /* effects */
    float     flash_intensity;
    float     shockwave_r;
    float     cloud_fade;     /* 1.0 normal, → 0.0 during FALL phase     */

    /* particle bookkeeping */
    float     ember_accum;
    float     ash_accum;
    bool      debris_spawned;
} NukeState;

static NukeState g_nuke;
static float     g_global_ts    = 1.0f;
static bool      g_global_pause = false;
static bool      g_show_smoke   = true;
static bool      g_lighting     = true;

static void sim_init(NukeState *s, float ts)
{
    memset(s, 0, sizeof *s);
    s->phase      = NK_FLASH;
    s->time_scale = ts;
    s->cloud_fade = 1.f;
}

/*
 * Derive a phase label from absolute time. Used only for the HUD —
 * geometry no longer branches on phase.
 */
static NukePhase phase_for_time(float t)
{
    if (t < T_FLASH_END)     return NK_FLASH;
    if (t < T_RISE_BEG)      return NK_FIREBALL;
    if (t < T_SPREAD_BEG)    return NK_RISING;
    if (t < T_SPREAD_END)    return NK_CAP_FORM;
    if (t < T_PLATEAU_BEG)   return NK_MUSHROOM;
    if (t < T_FALL_BEG)      return NK_PLATEAU;
    if (t < T_FALL_END)      return NK_FALL;
    return NK_SETTLE;
}

/*
 * Continuous-time geometry — single morphing blob.
 *
 * THE BIG IDEA. There is no separate fireball-and-cap. There is one
 * anisotropic ellipsoid blob whose three radii morph independently:
 *
 *   blob_rx (horizontal): grows monotonically from 0 → fireball_r → CAP_R_H.
 *                         The outer edges keep moving outward — that IS the
 *                         spreading-gas motion.
 *
 *   blob_ry (vertical):   grows from 0 → fireball_r, then COMPRESSES to
 *                         CAP_R_V as the gas pancakes outward. The fact
 *                         that it shrinks while rx grows is what gives the
 *                         "boiling spread" look instead of the old "new cap
 *                         appears" lerp.
 *
 *   blob_y:               rises smoothly from BLAST_Y to apex.
 *   blob_heat:            cools as gas expands (1.0 → 0.18).
 *   disp_scale:           noise turbulence ramps up with the spread —
 *                         a smooth ball boils into cauliflower.
 *
 * Vortex/skirt/pyrocumulus are still separate accessory shapes that fade
 * in late, but they sit *on top* of the morphed blob — they're decoration,
 * not the cap itself.
 */
static void sim_update_geometry(NukeState *s)
{
    float t = s->time;

    s->flash_intensity = 1.f - smoothstep(0.f, T_FLASH_END, t);

    /* Two overlapping master parameters. The ball-to-mushroom morph
     * uses smootherstep (quintic) — its zero acceleration at the seams
     * removes the residual "snap" you can still feel with cubic
     * smoothstep, giving a properly fluid pancake motion. */
    float blob_grow = smoothstep  (T_BLOB_GROW_BEG, T_BLOB_GROW_END, t);
    float spread    = smootherstep(T_SPREAD_BEG,    T_SPREAD_END,    t);

    /* Horizontal radius: starts at 0, grows to fireball size, keeps
     * expanding to full cap width. Monotonic. */
    s->blob_rx = blob_grow * lerpf(FIREBALL_R_MAX, CAP_R_H, spread);

    /* Vertical radius: starts at 0, grows to fireball size, then
     * compresses to cap thickness. The "then compresses" bit is what
     * makes the blob deform like spreading gas instead of like a new
     * shape appearing. */
    s->blob_ry = blob_grow * lerpf(FIREBALL_R_MAX, CAP_R_V, spread);

    /* Y position: smooth rise from blast site to apex. */
    s->blob_y = BLAST_Y + STEM_HEIGHT_MAX
              * smoothstep(T_RISE_BEG, T_RISE_END, t);

    s->stem_h = fmaxf(0.f, s->blob_y - BLAST_Y);

    /* Heat: hot fireball cools as it spreads into a smoky cap. */
    s->blob_heat = lerpf(1.0f, 0.18f, spread);

    /* Turbulence: smooth ball at first, increasingly cauliflower as
     * the gas convects outward. */
    s->disp_scale = lerpf(0.35f, 1.0f, spread);

    /* Accessories fade in late, decorating the morphed blob. */
    s->vortex_scale = smoothstep(T_VORTEX_BEG, T_VORTEX_END, t);
    s->skirt_scale  = smoothstep(T_SKIRT_BEG,  T_SKIRT_END,  t);
    s->pyro_scale   = smoothstep(T_PYRO_BEG,   T_PYRO_END,   t);

    /* Gentle breathing pulse once mature, faded in smoothly. */
    float breathe = smoothstep(T_PYRO_END, T_PYRO_END + 1.0f, t);
    float pulse   = 1.f + 0.025f * sinf(t * 1.4f) * breathe;
    s->blob_rx *= pulse;
    s->blob_ry *= 1.f + (pulse - 1.f) * 0.5f;

    /* ── FALL phase ───────────────────────────────────────────────────
     * After the 3-second plateau the cloud collapses: density fades to
     * zero, the blob droops downward, the cap thins, and accessories die
     * away (skirt/vortex/pyro all multiplied by cloud_fade so they fade
     * in lockstep with the main blob). Ash particles, spawned in
     * sim_tick, take over visually as the cloud disappears. */
    float fall = smoothstep(T_FALL_BEG, T_FALL_END, t);
    s->cloud_fade = 1.f - fall;
    if (fall > 0.f) {
        s->blob_y  -= fall * 1.6f;     /* droop downward */
        s->blob_ry *= (1.f - fall * 0.55f);  /* thin cap collapses */
        s->stem_h  *= (1.f - fall);    /* stem dissipates */
        s->vortex_scale *= s->cloud_fade;
        s->skirt_scale  *= s->cloud_fade;
        s->pyro_scale   *= s->cloud_fade;
    }

    s->shockwave_r = (t > T_BLOB_GROW_BEG)
        ? SHOCKWAVE_SPD * (t - T_BLOB_GROW_BEG) : 0.f;

    s->phase = phase_for_time(t);
}

static void sim_tick(NukeState *s, float dt)
{
    if (s->paused || s->done) return;

    float adt = dt * s->time_scale;
    s->time  += adt;

    if (s->time > T_DEACTIVATE) {
        s->done = true;
        return;
    }

    sim_update_geometry(s);

    /* Spawn debris once, when the fireball first appears. */
    if (!s->debris_spawned && s->time > T_BLOB_GROW_BEG + 0.05f) {
        particles_spawn_debris();
        s->debris_spawned = true;
    }

    /* Embers drift up the stem from the moment it starts forming until
     * the cap is mature — no phase test needed. */
    if (s->time > T_RISE_BEG && s->time < T_PLATEAU_BEG && s->stem_h > 0.1f) {
        s->ember_accum += adt;
        float interval = 1.f / (float)EMBER_N;
        while (s->ember_accum >= interval) {
            particles_spawn_ember(s->stem_h);
            s->ember_accum -= interval;
        }
    }

    /* Ash rains from the cloud during the FALL phase. Spawn rate ramps
     * down as the cloud fades so we don't keep dropping particles from
     * empty space. */
    if (s->time > T_FALL_BEG && s->time < T_FALL_END + 1.0f
        && s->blob_rx > 0.1f && s->blob_ry > 0.05f) {
        s->ash_accum += adt * (float)ASH_PER_SEC * fmaxf(0.15f, s->cloud_fade);
        while (s->ash_accum >= 1.f) {
            particles_spawn_ash(s->blob_rx, s->blob_ry, s->blob_y);
            s->ash_accum -= 1.f;
        }
    }
}

static void nuke_reset(void)
{
    memset(g_parts, 0, sizeof g_parts);
    sim_init(&g_nuke, g_global_ts);
}

/* ===================================================================== */
/* §9  raymarcher — Beer–Lambert volumetric integrator                    */
/*                                                                        */
/* For each ray, we walk fixed steps through the volume and accumulate:  */
/*                                                                        */
/*   transmittance T  starts at 1.0 ("light gets through")                */
/*                    multiplied by exp(-σ·dt) per step (Beer–Lambert)    */
/*   heat_acc     emission integrated front-to-back, weighted by T      */
/*   bright       lit-smoke contribution, also weighted by T            */
/*                                                                        */
/* Beer–Lambert law:                                                      */
/*   T(x→x+dx) = T(x) · exp(-σ(x) dx)                                     */
/* where σ = density · EXTINCTION is the "optical depth per unit length". */
/* The integral is approximated as a fixed-step Riemann sum. Reference: */
/*   https://www.pbr-book.org/4ed/Volume_Scattering                       */
/*   https://en.wikipedia.org/wiki/Beer%E2%80%93Lambert_law              */
/*                                                                        */
/* PERFORMANCE wins:                                                      */
/*   1. Per-bomb cull sphere — reject rays that miss the cloud entirely. */
/*   2. Per-sample cull — fast 1.6× step through clearly-empty space.    */
/*   3. Early bounds check inside cloud_density (huge — see §6).         */
/*   4. Transmittance bail at < 7%: remaining contribution is invisible.  */
/*                                                                        */
/* SUPERSAMPLING (SS_Y = 2): the render loop in §10 calls rm_cast_one()  */
/* twice per terminal cell (top half + bottom half) and combines the two */
/* samples into a glyph. Effectively 2× vertical resolution at less than */
/* 2× cost (early bail kicks in on opaque cells).                         */
/* ===================================================================== */

typedef struct {
    float heat;       /* emission accumulated along ray, ∈ [0,1]         */
    float opacity;    /* 1 - transmittance; ∈ [0,1]                       */
    float bright;     /* lit-smoke brightness (extinction × diffuse)      */
} Sample;

/*
 * Cast one ray through the volume.
 *
 *   sub_row ∈ [0..SS_Y-1] — which vertical sub-row within the cell
 *
 * Returns emission (heat), opacity (smoke density), and lit brightness.
 */
static Sample rm_cast_one(int px_col, int py_row, int sub_row,
                           int cw, int ch_cells, const NukeState *s)
{
    /* Sub-pixel vertical jitter: split each cell into SS_Y horizontal
     * stripes; the ray for sub_row passes through the centre of stripe i. */
    float v_offset = ((float)sub_row + 0.5f) / (float)SS_Y;
    float hires_h  = (float)(ch_cells * SS_Y);

    float u  =  ((float)px_col + 0.5f) / (float)cw * 2.f - 1.f;
    float v  = -((float)py_row + v_offset) / (float)ch_cells * 2.f + 1.f;

    /* Aspect compensation: account for SS_Y so vertical world-units stay
     * proportional to horizontal world-units when supersampling. */
    float pa = ((float)ch_cells * CELL_ASPECT) / (float)cw;
    (void)hires_h;

    V3 ro = v3(0.f, 0.f, CAM_Z);
    V3 rd = v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * pa, -1.f));

    Sample out = {0};

    /* Cloud is centered around (0, cloud_cy, 0). Bound the cull sphere
     * on that center — earlier code used the origin which made the test
     * loose for the elongated mushroom shape. */
    float cloud_cy = (BLAST_Y + s->blob_y + s->blob_ry) * 0.5f;
    V3    cloud_c  = v3(0.f, cloud_cy, 0.f);

    /* Ray-sphere bounds test — bail if the ray misses the cloud entirely. */
    {
        V3 oc = v3sub(ro, cloud_c);
        float b = v3dot(oc, rd);
        float c = v3dot(oc, oc) - CULL_R_BOMB * CULL_R_BOMB;
        if (b*b - c < 0.f) return out;
    }

    float transmit = 1.f;
    float heat_acc = 0.f;
    float t        = 0.2f;
    const float cull_r2 = CULL_R_BOMB * CULL_R_BOMB;

    for (int i = 0; i < RM_MAX_STEPS; i++) {
        V3 p = v3add(ro, v3mul(rd, t));

        /* Per-step cull against the cloud bounding sphere. Take a larger
         * step when clearly outside — gets us through empty space faster. */
        float dx = p.x, dy = p.y - cloud_cy, dz = p.z;
        if (dx*dx + dy*dy + dz*dz > cull_r2) {
            t += RM_STEP * 1.6f;
            if (t > RM_MAX_DIST) break;
            continue;
        }

        float d = cloud_density(p, s->time,
                                s->blob_rx, s->blob_ry, s->blob_y,
                                s->stem_h,
                                s->vortex_scale, s->skirt_scale,
                                s->pyro_scale, s->disp_scale,
                                s->cloud_fade);

        if (d > DENSITY_THRESH) {
            float h = cloud_heat(p, s->blob_rx, s->blob_ry, s->blob_y,
                                 s->blob_heat, s->stem_h);

            float sigma = d * RM_STEP * EXTINCTION;
            float dT    = expf(-sigma);

            /* Cheap self-shadow: smoke high in the column is brighter
             * because less stuff is between it and the sun. */
            float shadow = smoothstep(BLAST_Y - 0.5f,
                                      BLAST_Y + STEM_HEIGHT_MAX + 1.0f, p.y);
            /* Sun angle from the right: brighten right side of cloud. */
            float side   = smoothstep(-1.5f, 1.5f, p.x);
            float light  = LIGHT_AMBIENT
                         + LIGHT_DIFFUSE * lerpf(shadow, side, 0.4f);

            heat_acc      += transmit * (1.f - dT) * h;
            out.bright    += transmit * (1.f - dT) * light;
            transmit      *= dT;

            if (transmit < TRANSMIT_BAIL) break;
        }

        t += RM_STEP;
        if (t > RM_MAX_DIST) break;
    }

    out.opacity = 1.f - transmit;
    out.heat    = clmpf(heat_acc, 0.f, 1.f);
    return out;
}

/* ===================================================================== */
/* §10  canvas — sub-pixel pair buffer + glyph picker                     */
/*                                                                        */
/* CONCEPT — sub-cell ASCII shading.                                      */
/*                                                                        */
/* Terminal cells are roughly 2× tall as they are wide, so vertical       */
/* resolution is the bottleneck. SS_Y = 2 fires two rays per cell — one  */
/* through the top half, one through the bottom half — and a glyph       */
/* picker maps the (top, bot) opacity pair to an ASCII character whose   */
/* visual shape encodes which half is denser:                             */
/*                                                                        */
/*   both strong → full-body  '#' '@' '%' '8'                             */
/*   top  only   → ascender   '"' '`' '^' '~'                             */
/*   bottom only → descender  '.' ',' '_' 'u'                             */
/*   both medium → middle     ':' ';' '|' 'o'                             */
/*                                                                        */
/* Effectively 2× vertical resolution. The same idea is used by braille- */
/* dot renderers, just at a coarser glyph alphabet.                       */
/* ===================================================================== */

typedef struct { Sample top, bot; } CellPair;

typedef struct { int w, h; CellPair *cells; } Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w = cols; c->h = rows;
    c->cells = calloc((size_t)(cols * rows), sizeof(CellPair));
}

static void canvas_free(Canvas *c)
{
    free(c->cells); c->cells = NULL; c->w = c->h = 0;
}

static void canvas_fill(Canvas *c, const NukeState *s)
{
    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            CellPair *cp = &c->cells[py*c->w + px];
            cp->top = rm_cast_one(px, py, 0, c->w, c->h, s);
            cp->bot = rm_cast_one(px, py, 1, c->w, c->h, s);
        }
    }
}

/*
 * pick_glyph — choose ASCII char from (top opacity, bottom opacity).
 *
 * The glyph encodes which half of the cell is denser:
 *   both strong  → full-body chars     '#' '@' '%' '8'
 *   top  only    → ascender chars      '"' '`' '^' '~'
 *   bottom only  → descender chars     '.' ',' '_' 'u'
 *   both medium  → middle chars        ':' ';' '|' '|'
 *   sparse       → '.' or ' '
 */
static char pick_glyph(float top_op, float bot_op)
{
    /* Quantise opacities into [0..3]: 0 empty, 1 light, 2 mid, 3 dense. */
    int qt = (int)(clmpf(top_op, 0.f, 1.f) * 3.999f);
    int qb = (int)(clmpf(bot_op, 0.f, 1.f) * 3.999f);
    if (qt > 3) qt = 3;
    if (qb > 3) qb = 3;

    static const char full[4]    = { ' ', '+', '%', '#' };  /* by max(qt,qb) */
    static const char top_w[4]   = { ' ', '`', '"', '^' };  /* top-only */
    static const char bot_w[4]   = { ' ', '.', ',', 'u' };  /* bottom-only */
    static const char middle[4]  = { ' ', ':', ';', '|' };  /* both medium */

    if (qt == 0 && qb == 0) return ' ';

    /* Asymmetric: one half clearly heavier than the other → weighted glyph. */
    if (qt > qb + 1) return top_w[qt];
    if (qb > qt + 1) return bot_w[qb];

    /* Roughly symmetric. Pick full body if anything is dense, else middle. */
    int q = (qt > qb) ? qt : qb;
    if (q >= 3) return full[q];
    return middle[q];
}

/*
 * canvas_draw() — map cell pairs to ncurses, then overlay flash + particles.
 *
 * Per cell: pick glyph from the (top, bot) opacity pair, then pick color
 * from heat (fire) or brightness (smoke). When heat is high we use the fire
 * palette and bias towards bold; otherwise we use the smoke palette so cold
 * smoke renders as gray instead of being culled.
 */
static void canvas_draw(const Canvas *c, int tcols, int trows,
                         const NukeState *s)
{
    int ox = (tcols - c->w) / 2;
    int oy = (trows - c->h) / 2;

    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int tx = ox + x, ty = oy + y;
            if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows) continue;
            const CellPair *cp = &c->cells[y*c->w + x];

            float top_op = cp->top.opacity;
            float bot_op = cp->bot.opacity;
            float avg_op = (top_op + bot_op) * 0.5f;
            if (avg_op < 0.04f) continue;

            char g = pick_glyph(top_op, bot_op);
            if (g == ' ') continue;

            float avg_heat = (cp->top.heat + cp->bot.heat) * 0.5f;
            float avg_lit  = (cp->top.bright + cp->bot.bright) * 0.5f;

            attr_t a;
            if (avg_heat > 0.18f) {
                /* Hot region — use fire palette driven by heat. */
                a = fire_pair(avg_heat);
            } else if (g_show_smoke) {
                /* Cold smoke — use smoke palette driven by lit brightness. */
                float b = g_lighting
                        ? (avg_lit / fmaxf(avg_op, 0.1f))
                        : avg_op;
                /* Slight per-cell random dither to break up banding. */
                b += (prand() - 0.5f) * 0.05f;
                a = smoke_pair(b);
            } else {
                continue;
            }
            attron(a);
            mvaddch(ty, tx, (chtype)(unsigned char)g);
            attroff(a);
        }
    }

    /* Particles drawn on top. */
    particles_draw(tcols, trows, ox, oy, c->w, c->h);

    /* Initial-flash overlay — sparse white sparkles across the screen. */
    if (s->flash_intensity > 0.01f) {
        static const char flash_chars[] = "@#$%+*";
        int    ncov = (int)(s->flash_intensity * 5.9f);
        char   fch  = flash_chars[ncov > 5 ? 5 : ncov];
        attr_t fa   = COLOR_PAIR(CP_FLASH) | A_BOLD;
        attron(fa);
        for (int y = 0; y < trows - 2; y++)
            for (int x = 0; x < tcols; x++)
                if (prand() < s->flash_intensity)
                    mvaddch(y, x, (chtype)(unsigned char)fch);
        attroff(fa);
    }
}

/* ===================================================================== */
/* §11  renderer                                                          */
/* ===================================================================== */

typedef struct { Canvas canvas; } Renderer;

static void renderer_init(Renderer *r, int cols, int rows)
{
    canvas_alloc(&r->canvas, cols, rows);
}

static void renderer_free(Renderer *r) { canvas_free(&r->canvas); }

static void renderer_resize(Renderer *r, int cols, int rows)
{
    canvas_free(&r->canvas);
    canvas_alloc(&r->canvas, cols, rows);
}

static void renderer_render(Renderer *r) { canvas_fill(&r->canvas, &g_nuke); }

static void renderer_draw(const Renderer *r, int cols, int rows, double fps)
{
    canvas_draw(&r->canvas, cols, rows, &g_nuke);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%.1ffps  %s  t=%.1f  spd:%.1fx  theme:%s  smoke:%s  light:%s  %s",
             fps, k_phase_names[g_nuke.phase], g_nuke.time,
             g_global_ts,
             g_themes[g_theme_idx].name,
             g_show_smoke ? "on" : "off",
             g_lighting   ? "on" : "off",
             g_global_pause ? "PAUSED" : "");
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%.*s", cols - 1, buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  +/-:speed  n:redo  t:theme  s:smoke  l:light");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §12  screen — ncurses                                                  */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §13  app — main loop                                                   */
/* ===================================================================== */

typedef struct {
    Renderer  rend;
    Screen    screen;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int sig) { (void)sig; g_app.running = 0; }
static void on_resize(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup  (void)   { endwin(); }

static void app_do_resize(App *a)
{
    screen_resize(&a->screen);
    renderer_resize(&a->rend, a->screen.cols, a->screen.rows);
    a->need_resize = 0;
}

static bool app_handle_key(App *a, int ch)
{
    (void)a;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':
        g_global_pause = !g_global_pause;
        g_nuke.paused  = g_global_pause;
        break;
    case 'r': case 'R':
    case 'n': case 'N':
        nuke_reset();
        break;
    case '+': case '=':
        g_global_ts = fminf(g_global_ts + 0.5f, 16.0f);
        g_nuke.time_scale = g_global_ts;
        break;
    case '-':
        g_global_ts = fmaxf(g_global_ts - 0.5f, 0.25f);
        g_nuke.time_scale = g_global_ts;
        break;
    case 's': case 'S': g_show_smoke = !g_show_smoke; break;
    case 'l': case 'L': g_lighting   = !g_lighting;   break;
    case 't': case 'T': cycle_theme(); break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app  = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    nuke_reset();
    renderer_init(&app->rend, app->screen.cols, app->screen.rows);

    int64_t frame_t = clock_ns();
    int64_t sim_acc = 0;
    int64_t fps_acc = 0;
    int     fps_cnt = 0;
    double  fpsd    = 0.0;

    while (app->running) {
        if (app->need_resize) { app_do_resize(app); frame_t = clock_ns(); sim_acc = 0; }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_t;
        frame_t     = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick   = TICK_NS(SIM_FPS);
        float   tick_s = (float)tick / (float)NS_PER_SEC;
        sim_acc += dt;
        while (sim_acc >= tick) {
            sim_tick(&g_nuke, tick_s);
            particles_tick(tick_s);
            sim_acc -= tick;
        }

        renderer_render(&app->rend);

        erase();
        renderer_draw(&app->rend, app->screen.cols, app->screen.rows, fpsd);
        screen_present();

        fps_cnt++;
        fps_acc += dt;
        if (fps_acc >= FPS_MS * NS_PER_MS) {
            fpsd    = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
            fps_cnt = 0;
            fps_acc = 0;
        }

        int64_t elapsed = clock_ns() - frame_t + dt;
        clock_sleep_ns(NS_PER_SEC / 24 - elapsed);

        int key = getch();
        if (key != ERR && !app_handle_key(app, key)) app->running = 0;
    }

    renderer_free(&app->rend);
    screen_free(&app->screen);
    return 0;
}
