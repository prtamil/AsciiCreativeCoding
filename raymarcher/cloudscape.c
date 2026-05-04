/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cloudscape.c — volumetric clouds with Beer–Lambert + crepuscular rays.
 *
 * DEMO: A static observer looks up at a slowly drifting field of clouds.
 *       Every pixel raymarches through a 3-D density field built from a
 *       2-D fBm slab (cheap) modulated by a vertical bell-curve profile.
 *       At each march step a SHORT shadow ray is cast toward the sun;
 *       light reaching that step's air-or-cloud parcel scatters TOWARD
 *       the camera (single-scatter Henyey-Greenstein-style phase),
 *       brighter when the ray and sun directions agree.  Where clouds
 *       partly block the sun, the unblocked gaps light up as visible
 *       crepuscular shafts — "god rays" — streaking across the sky.
 *       Wind drifts the cloud field in the +x direction so the shafts
 *       wander in real time.
 *
 *       Distinct from `nuke.c` (the existing volumetric demo): nuke
 *       is a transient buoyant-fluid explosion (rising column + cap +
 *       fall driven by Boussinesq Navier-Stokes); cloudscape is a
 *       stationary atmospheric system with no convection.  Same
 *       Beer–Lambert integrator, completely different physics.
 *
 *       Patterns / cover types (cycle with n / N):
 *         SCATTERED  broken cumulus, big gaps — best god-ray drama
 *         OVERCAST   uniform mid-density layer, soft diffused light
 *         CIRRUS     high thin streaks, faint shafts
 *         STORM      dark heavy clouds, dim shafts where sun pokes through
 *
 * Study alongside:
 *   raymarcher/nuke.c         — same volumetric raymarch skeleton.
 *                                  Cloudscape is the calm cousin.
 *   raymarcher/voxel_terrain.c   — also column-march + atmospheric
 *                                  perspective.  Different domain
 *                                  (heightfield vs. volume) but the
 *                                  haze-ramp colour idea carries over.
 *   raymarcher/sun_solar.c             — the sun's corona aesthetic.
 *
 * Section map:
 *   §1 config   — patterns, themes, raymarch tunables
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair cloud ramp + 3-pair sky + sun + ray pairs
 *   §4 vec3     — value-type 3-D math
 *   §5 noise    — hash + smoothstep value noise + 3-octave fBm (2D)
 *   §6 density  — slab + 2D fBm + vertical bell + air baseline
 *   §7 march    — Beer–Lambert primary + sun shadow ray + phase
 *   §8 scene    — camera, sun, wind, full-frame render
 *   §9 screen   — ncurses init / draw / resize
 *  §10 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume cloud drift
 *   r            reseed noise field
 *   n / N        next / previous pattern (SCATTERED → OVERCAST → CIRRUS → STORM)
 *   t / T        next / previous theme   (DAWN / NOON / DUSK / STORM_THM / MOONLIT / ALIEN)
 *   ← / →        sun azimuth left / right  (rays sweep across the sky)
 *   ↑ / ↓        sun pitch  up / down      (low sun = long shafts)
 *   + / =        wind speed up
 *   -            wind speed down
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/cloudscape.c \
 *       -o cloudscape -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Single-scattering volumetric raymarcher.
 *
 *                  PRIMARY march, per pixel:
 *                     T = 1, L = 0
 *                     for step = 1..N:
 *                         p   = origin + t · ray_dir
 *                         d   = density(p)
 *                         dtau  = d · step_size              (optical depth)
 *
 *                         // Shadow ray toward sun — short march of M steps
 *                         T_sun = exp(−Σ density(p + s·sun_dir) · sun_step)
 *
 *                         // Inscatter contribution this step
 *                         L  += T · dtau · (T_sun · phase(ω·sun) · sun_colour
 *                                         + ambient_sky)
 *
 *                         // Beer–Lambert extinction
 *                         T  *= exp(−dtau)
 *                         if T < 0.01: break              (opaque)
 *                         t  += step_size
 *
 *                  After the march:
 *                     pixel.colour = L  (cloud + scatter)
 *                     pixel.alpha  = 1 − T
 *                     blend with sky by alpha
 *
 *                  Why this gives crepuscular rays:
 *
 *                    Air has a tiny baseline density everywhere, so EVERY
 *                    step picks up some inscatter.  Steps along rays whose
 *                    sun-shadow path is blocked by a cloud get a small
 *                    T_sun → dim inscatter.  Steps along rays whose sun-
 *                    shadow path is CLEAR get T_sun ≈ 1 → bright inscatter.
 *                    The phase function `phase(ω·sun)` further amplifies
 *                    the effect when the ray points near the sun.  The net
 *                    result, viewed across thousands of pixels, is bright
 *                    BAND-LIKE shafts of light through the sky exactly
 *                    where the sun has a clear path — the "god ray" effect.
 *
 *                  Density field:
 *                     density(p) = AIR + slab(y) · vert_bell(y) · max(0, fbm2d(p.xz) − thr)
 *
 *                     - slab clamps to a fixed altitude range (CLOUD_BOTTOM..TOP)
 *                     - vert_bell peaks in the slab middle (denser at mid altitude)
 *                     - fbm2d gives the visible cloud pattern at the slab
 *                     - thr clips out below-threshold noise to make CLEAR GAPS
 *                       (no gaps → no god rays — the threshold is essential)
 *                     - AIR is a constant baseline so inscatter happens
 *                       everywhere, not only inside cloud cells
 *
 * Data-structure : Stateless density function + per-frame scene struct.
 *                  Optional per-pattern threshold/profile knobs.  No
 *                  precomputation, no LUTs — the 2D fbm is fast enough
 *                  to evaluate inline.
 *
 * Rendering      : ASCII only.  Glyph from accumulated luminance L
 *                  (` . , : ; + * #`).  Colour pair from cloud-vs-sky
 *                  classification (alpha + L).  In sky regions, high L
 *                  picks the SUN-tinted "ray" pair so god-ray shafts
 *                  visually pop out of the surrounding sky gradient.
 *
 * Performance    : ~28 primary × 4 shadow steps per pixel × 6 noise hash
 *                  calls per density eval.  At 80×24 = 1 920 pixels: ~1.3 M
 *                  hash ops per frame.  Comfortable at 30–60 fps.
 *                  Larger terminals (200×60 = 12 000 pixels) take ~6× more,
 *                  still tractable; for 4K terminals lower MAX_STEPS.
 *
 * References     :
 *   • Hillaire, S. (2015) — "Towards Unified and Physically-Based Volumetric
 *     Cloud Rendering", *SIGGRAPH 2015 Course Notes*. The canonical modern
 *     reference for sun-shadow shortcut + phase function for clouds.
 *   • Schneider, A. (2015) — "Real-Time Volumetric Cloudscapes",
 *     *GPU Pro 7* / Guerrilla Games tech talk. The Horizon Zero Dawn
 *     cloud rendering paper, where the 2-D-base + vertical-profile
 *     density trick comes from.
 *   • Henyey, L. C. & Greenstein, J. L. (1941) — "Diffuse Radiation in
 *     the Galaxy", *Astrophysical Journal* 93, 70–83.  The original
 *     phase function.  We use a cheap forward-bias substitute,
 *     `pow(max(0, ω·sun), p)`, qualitatively similar but ~5× cheaper.
 *   • Bouthors, A. et al. (2008) — "Interactive Multiple Anisotropic
 *     Scattering in Clouds", *I3D '08*.  Shows the importance of
 *     letting the sun shadow be cheap (ours: 4 steps) — multi-bounce
 *     accuracy buys little visible quality for our terminal output.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A cloud is a 3-D bag of vapour that BOTH BLOCKS light (extinction) AND
 * SCATTERS some of the sun's rays toward the eye (inscattering).  Every
 * pixel walks its viewing ray through the sky and adds up these two
 * effects step by step.  When a viewing ray happens to point near the
 * sun AND its path to the sun is unblocked, the air ALONG that ray
 * lights up brightly — that's a god ray.  When the sun's path is
 * blocked by a cloud somewhere, the air along the same ray stays dim —
 * that's the shadow shaft right next to the god ray.  Hundreds of
 * pixels resolving these conditions is what produces the iconic streaky
 * "sunbeams through clouds" look.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a hazy room with one bright window.  Dust motes everywhere
 * scatter sunlight toward your eyes — you see a visible beam from the
 * window across the room, brightest along the line where the dust is
 * AND the sun is unobstructed.  Move a hand to block part of the
 * window: the dust still scatters light in the unblocked region, but
 * the dust in the SHADOW of your hand stays dark.  Cloudscape is the
 * same room, scaled up: window = sun, dust = baseline air density,
 * hand = cloud.  Each viewing ray asks dust along its path "is the sun
 * visible from where you sit?" and brightens accordingly.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Build camera basis (looking up at angle pitch, around yaw).
 *     Compute world-space sun direction from sun_yaw, sun_pitch.
 *     Cache the cosine of (ray·sun) per pixel for the phase function.
 *
 *  2. PRIMARY march, per pixel (constant step size, cheap & sufficient
 *     for the cloud-fidelity we need at terminal resolution):
 *
 *        T = 1.0, L = 0.0
 *        for step in 1..MAX_STEPS:
 *            p   = camera + t · ray_dir
 *            d   = density(p, time)
 *            dtau  = d · STEP_SIZE
 *
 *  3. SHADOW march, only when d > tiny epsilon (most steps in sky/air
 *     are essentially zero density and skipping the shadow march is a
 *     ~3× speedup):
 *
 *            sun_T = 1.0
 *            for s in 1..SHADOW_STEPS:
 *                sp = p + s · sun_dir · SHADOW_STEP
 *                sun_T *= exp(−density(sp, time) · SHADOW_STEP)
 *
 *  4. INSCATTER + EXTINCTION:
 *
 *            phase     = forward_phase(ray·sun, eccentricity)
 *            inscatter = sun_T · phase · SUN_INTENSITY
 *                      + AMBIENT_SKY               (constant fill)
 *            L  += T · dtau · inscatter
 *            T  *= exp(−dtau)
 *
 *            if T < 0.01: break    (opaque, further steps invisible)
 *            t  += STEP_SIZE
 *
 *  5. CLASSIFY pixel:
 *
 *        alpha = 1 − T
 *        if alpha > CLOUD_THRESHOLD:                  → cloud cell
 *            slot  = ⌊L · 7.999⌋               (luminance ramp)
 *            glyph = CLOUD_GLYPHS[slot]
 *            colour= PAIR_CLOUD_BASE + min(slot, 5)
 *        else if L > RAY_THRESHOLD:                   → god-ray cell
 *            slot  = ⌊(L − thr) · 7.999⌋
 *            glyph = CLOUD_GLYPHS[slot]
 *            colour= PAIR_RAY                  (sun-tinted)
 *        else:                                        → plain sky
 *            slot  = sky_gradient_slot(row)
 *            glyph = SKY_GLYPHS[slot]
 *            colour= PAIR_SKY_BASE + slot
 *
 *  6. ANIMATE.  scene_tick advances `time`; the density function uses
 *     `wind · time` to translate the fBm sample point — clouds appear
 *     to drift in the wind direction.  Sun and camera are stationary.
 *
 * KEY FORMULAS
 * ────────────
 *  Beer–Lambert extinction (transmittance after path length L):
 *    T(L) = exp(−∫₀ᴸ density(s) ds)
 *    discretised: T *= exp(−d · ds)
 *
 *  Single-scatter inscatter (cheap form, no integral):
 *    L_step = T · dtau · (sun_visibility · phase + ambient)
 *
 *  Forward phase function (cheap H-G substitute):
 *    phase(c) = (1 − g) + g · max(0, c)^p     ,   c = ω · sun
 *    g ≈ 0.6, p ≈ 6   (tuned by eye for "silver lining + visible god
 *                       rays without blowing out forward-scatter").
 *
 *  Density field (slab + 2-D fBm + vertical bell + air baseline):
 *    fade(y) = 0  if y < BOT or y > TOP
 *            = sin(π · (y − BOT) / (TOP − BOT))   otherwise
 *
 *    n2d     = fbm2d((x + wind·t) · NOISE_SCALE,  z · NOISE_SCALE)
 *    base    = max(0,  n2d − threshold)
 *    cloud   = base · fade(y) · CLOUD_DENSITY
 *    return  AIR_DENSITY + cloud
 *
 *  Aspect correction (terminal cells 2× taller than wide):
 *    phys_aspect = (rows · 2) / cols      (factor on v in ray dir)
 *
 *  Sun pixel proximity (for sun glyph):
 *    cos_to_sun = ray · sun_dir
 *    if cos_to_sun > 1 − SUN_RADIUS_COS: paint sun core
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • THRESHOLD TOO HIGH → no clouds at all (sky everywhere).
 *  • THRESHOLD TOO LOW  → uniform overcast, no gaps for god rays.
 *    Patterns SCATTERED / OVERCAST / CIRRUS / STORM walk this knob.
 *
 *  • SUN BEHIND CAMERA.  When sun_pitch < 0 (below horizon) the dot
 *    product (ray · sun) goes negative everywhere; phase function
 *    floors to 0; god rays vanish.  We allow it, because dim red sun
 *    pre-twilight is a valid visual.
 *
 *  • SHADOW MARCH EARLY-OUT.  Skipping the sun shadow when density is
 *    tiny (`d < 1e-3`) is the dominant optimisation.  Without it, we
 *    do 4× the work in the 90 % of steps that are essentially clear
 *    air.  The cost: god rays in totally clear sky regions get the
 *    "fall-through" sun_T = 1, which is ACTUALLY CORRECT because the
 *    shadow ray would have hit no cloud anyway.
 *
 *  • OPAQUE EARLY-EXIT.  If transmittance drops below 0.01 the pixel
 *    can't see anything farther; bail out.  Critical for performance
 *    on dense overcast skies.
 *
 *  • STEP SIZE SAMPLING ERROR.  Constant step ≈ 0.5 cells.  For a slab
 *    of thickness ~12, that's ~24 samples — coarse enough for fast
 *    rendering, fine enough that aliasing is hidden by the soft cloud
 *    edges.  Tighter steps look identical at terminal resolution.
 *
 *  • ORDER OF DENSITY UPDATES.  The wind drift `(x + wind·t)` happens
 *    INSIDE density(), so wind is applied uniformly across all march
 *    steps within a frame (no spatial aliasing).
 *
 *  • PAUSE.  We freeze `time` (no wind), but the camera and sun
 *    response to keys still works.  This is intentional — useful for
 *    framing god-ray shots before resuming the drift.
 *
 *  • SUN GLYPH INSIDE CLOUD.  If a thick cloud hides the sun, alpha > 1
 *    where the sun would be.  The sun glyph is drawn IN the same pixel
 *    pass conditionally; cloud overpaints it via the `cloud first`
 *    branch.  No special compositing logic.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press `space`.  Cloud drift freezes.  Wind = 0 even when resumed
 *    if you press `-` enough times.
 *
 *  • Press `r`.  New noise field; the entire cloud pattern reshuffles.
 *
 *  • Press `→` until the sun is roughly opposite the view yaw.  God
 *    rays should now slant ACROSS the screen, not spread radially
 *    from one point.
 *
 *  • Press `↓` to lower sun pitch.  Light shafts get LONGER (sun is
 *    nearly horizontal, beams lance across the volume).  Past ~−5°
 *    they fade as the sun drops below horizon.
 *
 *  • Cycle patterns (`n`).  SCATTERED → most god rays (lots of gaps).
 *    OVERCAST → soft uniform light, almost no shafts.  CIRRUS → faint
 *    high streaks.  STORM → dim shafts breaking through dark base.
 *
 *  • Cycle themes (`t`).  Geometry identical, only colours change.
 *
 *  • At 60×20 the god-ray streaks should still read as bright bands
 *    through dimmer cloud.  If they look like noise, raise SHADOW_STEPS
 *    or sharpen the phase exponent.
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
    PAIR_CLOUD_BASE   =  3,    /* +0..+5  — 6 cloud-luminance pairs    */
    PAIR_SKY_BASE     =  9,    /* +0..+2  — 3 sky-gradient pairs       */
    PAIR_RAY          = 12,    /* god-ray sun-tinted brightness        */
    PAIR_SUN          = 13,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Cloud volume — slab in y. */
#define CLOUD_BOTTOM     6.0f
#define CLOUD_TOP       18.0f
#define NOISE_SCALE      0.060f    /* xz domain scale for fBm            */
#define CLOUD_DENSITY    1.4f      /* peak density inside slab           */
#define AIR_DENSITY      0.018f    /* baseline scatter medium            */

/* Camera.  Default points UP TOWARD THE SUN so the cloud field drifts
 * across the sun's disk — the canonical "cloud-over-sun" framing the
 * demo is built to showcase.  Use ←/→ ↑/↓ to swing the sun out of
 * the centre and watch the god-ray geometry shift. */
#define CAM_PITCH_DEFAULT 0.50f    /* radians above horizon              */
#define CAM_YAW_DEFAULT   0.0f
#define CAM_HEIGHT        1.0f     /* observer just above ground         */
#define FOV_DEG          80.0f     /* wide for sky drama                 */

/* Sun.  Defaults sit just above and slightly right of the camera's
 * view forward — the wind blows clouds in +x, so they drift past the
 * sun and you see the sun PARTIALLY occluded as cumulus passes through. */
#define SUN_PITCH_DEFAULT 0.58f
#define SUN_YAW_DEFAULT   0.10f
#define SUN_PITCH_MIN    -0.05f    /* allow just-below-horizon (no rays) */
#define SUN_PITCH_MAX     1.40f    /* near zenith                        */
#define SUN_KEY_STEP      0.06f
#define SUN_INTENSITY     2.6f     /* phase·intensity gain               */
#define AMBIENT_SKY       0.06f    /* constant ambient inscatter         */

/* Phase function (cheap forward-bias substitute for Henyey-Greenstein). */
#define PHASE_BACK        0.30f    /* 1 − g                              */
#define PHASE_FWD         0.70f    /* g                                  */
#define PHASE_EXP         6.0f     /* sharpness of forward lobe          */

/* Sun disc — three concentric rings give a readable round sun even at
 * 60×20.  Thresholds chosen so on an 80-col / 80°-FOV terminal the
 * disc is roughly:
 *   CORE  ≈ 4° half-angle  →  ~4 cells radius   (`@`)
 *   MID   ≈ 7° half-angle  →  ~7 cells radius   (`O`)
 *   GLOW  ≈ 11° half-angle →  ~11 cells radius  (`o`)
 * All three branches require alpha ≤ CLOUD_ALPHA_THR (cloud branch
 * wins), so a cloud passing in front genuinely OCCLUDES the sun. */
#define SUN_CORE_COS      0.9976f   /* cos( 4°) */
#define SUN_MID_COS       0.9925f   /* cos( 7°) */
#define SUN_GLOW_COS      0.9816f   /* cos(11°) */

/* Wind. */
#define WIND_DEFAULT      1.4f     /* cells / second drift in +x         */
#define WIND_MIN          0.0f
#define WIND_MAX          8.0f

/* Raymarch tunables. */
#define MAX_STEPS        28
#define SHADOW_STEPS      4
#define STEP_SIZE         0.55f
#define SHADOW_STEP       1.30f
#define T_NEAR            0.3f
#define T_FAR            32.0f
#define DENSITY_EPS       1.0e-3f  /* skip shadow march below this       */
#define OPAQUE_EPS        0.01f    /* primary march bail-out             */

/* Cloud / ray classification thresholds. */
#define CLOUD_ALPHA_THR   0.18f    /* alpha above this → "cloud cell"    */
#define RAY_LUM_THR       0.12f    /* sky luminance above this → ray     */

/* Pattern enum + per-pattern density tweaks. */
typedef enum {
    PATTERN_SCATTERED = 0,
    PATTERN_OVERCAST  = 1,
    PATTERN_CIRRUS    = 2,
    PATTERN_STORM     = 3,
    N_PATTERNS        = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SCATTERED: return "SCATTERED";
    case PATTERN_OVERCAST:  return "OVERCAST ";
    case PATTERN_CIRRUS:    return "CIRRUS   ";
    case PATTERN_STORM:     return "STORM    ";
    default:                return "?        ";
    }
}

/*
 * PatternParams — how the pattern shapes the density field.
 *
 *   threshold       : fbm clip; higher = fewer/smaller clouds = bigger gaps
 *   density_scale   : multiplier on cloud body density
 *   bottom, top     : altitude slab
 *   bell_exp        : vertical profile sharpness (1 = full bell, 2 = sharper)
 */
typedef struct {
    float threshold;
    float density_scale;
    float bottom, top;
    float bell_exp;
} PatternParams;

static const PatternParams patterns[N_PATTERNS] = {
    /* SCATTERED — mid threshold for big gaps; classic god-ray weather */
    /* threshold density_scale  bottom  top  bell_exp */
    { 0.50f, 1.4f,  6.0f, 18.0f, 1.0f },
    /* OVERCAST  — low threshold + thick band; soft diffuse light      */
    { 0.35f, 1.1f,  5.0f, 20.0f, 1.0f },
    /* CIRRUS    — high threshold + thin high band; faint shafts       */
    { 0.62f, 0.7f, 14.0f, 22.0f, 2.0f },
    /* STORM     — mid threshold + dense + extended slab; dark + dim   */
    { 0.42f, 2.2f,  4.0f, 22.0f, 1.0f },
};

/*
 * Themes — palette only, no physics.  Each theme defines:
 *   cloud[6]  : dark core → bright lit edge (luminance ramp)
 *   sky[3]    : top → mid → near-horizon haze
 *   ray       : god-ray sun-tinted bright accent
 *   sun       : sun glyph colour
 *
 * All entries sit in the bright half of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule, including the dimmest
 * cloud-core slot.
 */
typedef struct {
    const char *name;
    short cloud[6];
    short sky[3];
    short ray;
    short sun;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* DAWN — coral/peach clouds, pink sky, warm gold sun */
    { "DAWN     ",
      {  95, 131, 174, 217, 223, 230 },   /* cloud — peach to bone     */
      {  60, 138, 217 },                  /* sky   — purple→pink       */
      223, 220 },

    /* NOON — white/grey clouds, cobalt sky, white sun */
    { "NOON     ",
      { 240, 244, 248, 251, 254, 255 },
      {  25,  31,  39 },
      195, 231 },

    /* DUSK — orange clouds, magenta sky, red sun */
    { "DUSK     ",
      {  88, 130, 166, 209, 215, 222 },
      {  53,  90, 167 },
      215, 196 },

    /* STORM_THM — slate clouds, steel-blue sky, pale grey sun */
    { "STORM    ",
      { 238, 240, 242, 245, 247, 251 },
      {  60,  66,  68 },
      189, 254 },

    /* MOONLIT — silver clouds, deep navy sky, ice-blue moon */
    { "MOONLIT  ",
      {  60,  66,  72,  78, 152, 195 },
      {  17,  18,  24 },
      195, 195 },

    /* ALIEN — teal clouds, violet sky, cyan sun */
    { "ALIEN    ",
      {  29,  30,  31,  37,  44, 159 },
      {  53,  56,  98 },
      159,  51 },
};

/*
 * Glyph ramps — heavy on the cloud side, blank on the sky side, for
 * maximum legibility.
 *
 * CLOUD_GLYPHS skips the small-punctuation tiers (`.`/`,`/`:`) that
 * blend into sky.  Even the thinnest cloud-classified cell renders as
 * a `*`, so cloud silhouettes pop cleanly against the empty-black sky.
 *
 * SKY_GLYPHS are pure spaces — sky cells render as default-bg (black),
 * so clouds stand out as crisp shapes.  We trade the per-theme sky
 * gradient for a much more readable composition; the theme identity
 * lives in cloud + sun + ray colours, which is plenty.
 */
static const char CLOUD_GLYPHS[8] = { ' ', '*', '*', '#', '#', '#', '#', '%' };
static const char SKY_GLYPHS  [3] = { ' ', ' ', ' ' };

#define SUN_GLYPH_CORE   '@'
#define SUN_GLYPH_MID    'O'
#define SUN_GLYPH_GLOW   'o'

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
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_CLOUD_BASE + i), t->cloud[i], -1);
        for (int i = 0; i < 3; i++)
            init_pair((short)(PAIR_SKY_BASE + i), t->sky[i], -1);
        init_pair(PAIR_RAY, t->ray, -1);
        init_pair(PAIR_SUN, t->sun, -1);
    } else {
        static const short fb_cloud[6] = {
            COLOR_BLACK, COLOR_BLUE, COLOR_BLUE, COLOR_WHITE,
            COLOR_WHITE, COLOR_WHITE,
        };
        static const short fb_sky[3] = {
            COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        };
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_CLOUD_BASE + i), fb_cloud[i], -1);
        for (int i = 0; i < 3; i++)
            init_pair((short)(PAIR_SKY_BASE + i), fb_sky[i], -1);
        init_pair(PAIR_RAY, COLOR_YELLOW, -1);
        init_pair(PAIR_SUN, COLOR_YELLOW, -1);
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
static inline V3    v3sub(V3 a, V3 b)               { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale(float s, V3 a)          { return (V3){s*a.x, s*a.y, s*a.z}; }
static inline float v3dot(V3 a, V3 b)               { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    v3cross(V3 a, V3 b)             {
    return (V3){ a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
static inline float v3len(V3 a)                     { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a) {
    float L = v3len(a);
    return L > 1e-12f ? v3scale(1.0f / L, a) : v3(0, 1, 0);
}

/* ===================================================================== */
/* §5  noise — value noise + fBm                                          */
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
/* §6  density — slab + 2D fbm + vertical bell + air                      */
/* ===================================================================== */

/*
 * DensityCtx — packed per-frame density parameters so the inner loop
 * doesn't chase pointers into Scene.  Filled once per render() call.
 */
typedef struct {
    uint32_t seed;
    float    time;
    float    wind;
    float    threshold;
    float    density_scale;
    float    bottom, top;
    float    inv_height;     /* 1 / (top − bottom)                 */
    float    bell_exp;
} DensityCtx;

/*
 * cloud_density — value at world-space point p.  Returns total density
 * (cloud + air baseline).  Hot path — keep cheap.
 *
 * The vertical "bell" uses sin(π·y_norm) which is naturally 0 at the
 * slab boundaries and 1 at mid-altitude; raising it to bell_exp
 * sharpens the layer for thin clouds (CIRRUS).  We compute the
 * bell BEFORE the fbm so we can early-out on slab edges before paying
 * the noise cost — the dominant performance trick.
 */
static float cloud_density(V3 p, const DensityCtx *c)
{
    if (p.y < c->bottom || p.y > c->top) return AIR_DENSITY;

    float y_norm = (p.y - c->bottom) * c->inv_height;
    float bell   = sinf(y_norm * (float)M_PI);
    if (c->bell_exp != 1.0f) bell = powf(fmaxf(bell, 0.0f), c->bell_exp);
    if (bell < 0.001f) return AIR_DENSITY;

    float wx = p.x + c->wind * c->time;
    float wz = p.z;

    float n = fbm2d(wx * NOISE_SCALE, wz * NOISE_SCALE, c->seed);
    n -= c->threshold;
    if (n <= 0.0f) return AIR_DENSITY;

    return AIR_DENSITY + n * bell * c->density_scale * CLOUD_DENSITY;
}

/* ===================================================================== */
/* §7  march — Beer–Lambert primary + sun shadow + phase                  */
/* ===================================================================== */

/*
 * forward_phase — cheap Henyey-Greenstein-flavoured forward scatter.
 *
 * Real H-G is `(1 − g²) / (4π · (1 + g² − 2g·c)^1.5)` which costs a
 * sqrt + pow.  We use `(1 − g) + g · max(0, c)^p` instead — same
 * monotonic forward lobe, no expensive sqrt, and the visible result
 * is indistinguishable at terminal resolution.
 */
static inline float forward_phase(float cos_ray_sun)
{
    float c = cos_ray_sun;
    if (c < 0.0f) c = 0.0f;
    /* powf with a small integer exponent is the dominant cost — we
     * unroll p = 6 manually. */
    float c2 = c * c;
    float c4 = c2 * c2;
    float c6 = c4 * c2;
    return PHASE_BACK + PHASE_FWD * c6;
}

/*
 * raymarch_cloud — accumulate luminance + transmittance along the ray.
 *
 * `out_L` and `out_T` carry the inscatter (luminance) and final
 * transmittance respectively.  alpha = 1 − T; opaque cloud → alpha = 1.
 *
 * The shadow march is gated by DENSITY_EPS — we only do the 4 sun-
 * direction steps when the current point has actual cloud.  This is
 * the dominant performance optimisation; without it the demo is ~3×
 * slower on clear-sky frames.
 */
static void raymarch_cloud(V3 origin, V3 dir, V3 sun_dir,
                           const DensityCtx *c,
                           float *out_L, float *out_T)
{
    float t   = T_NEAR;
    float T   = 1.0f;
    float L   = 0.0f;
    float phase = forward_phase(v3dot(dir, sun_dir));

    for (int step = 0; step < MAX_STEPS; step++) {
        V3    p   = v3add(origin, v3scale(t, dir));
        float d   = cloud_density(p, c);
        float dtau  = d * STEP_SIZE;

        /* Sun-shadow ray (skip for thin air). */
        float sun_T = 1.0f;
        if (d > DENSITY_EPS) {
            float st = SHADOW_STEP * 0.5f;     /* offset to avoid self-step */
            for (int s = 0; s < SHADOW_STEPS; s++) {
                V3    sp = v3add(p, v3scale(st, sun_dir));
                float sd = cloud_density(sp, c);
                sun_T   *= expf(-sd * SHADOW_STEP);
                st      += SHADOW_STEP;
            }
        }

        /* Inscatter contribution this step. */
        float inscatter = sun_T * phase * SUN_INTENSITY + AMBIENT_SKY;
        L += T * dtau * inscatter;

        /* Beer–Lambert update. */
        T *= expf(-dtau);
        if (T < OPAQUE_EPS) break;
        if (t > T_FAR)      break;

        t += STEP_SIZE;
    }

    *out_L = L;
    *out_T = T;
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool    paused;
    int     current_pattern;
    int     current_theme;
    uint32_t seed;
    int     cols, rows;

    float   time;          /* drift accumulator (seconds)  */
    float   wind;          /* cells / second               */

    float   cam_pitch;
    float   cam_yaw;

    float   sun_pitch;
    float   sun_yaw;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->current_pattern = PATTERN_SCATTERED;
    s->current_theme   = 0;
    s->seed            = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    s->time            = 0.0f;
    s->wind            = WIND_DEFAULT;
    s->cam_pitch       = CAM_PITCH_DEFAULT;
    s->cam_yaw         = CAM_YAW_DEFAULT;
    s->sun_pitch       = SUN_PITCH_DEFAULT;
    s->sun_yaw         = SUN_YAW_DEFAULT;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->seed = (uint32_t)clock_ns() ^ 0xC10D5EEDu;
    s->time = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
}

/*
 * scene_render — full-frame raymarch.  One ray per terminal cell.
 *
 * Three structural notes:
 *
 *   1. We pre-pack a DensityCtx once per frame so the inner loop has a
 *      dense cache-friendly view of all the per-frame parameters.
 *
 *   2. We track last (pair, attr) and only attron when it changes,
 *      cutting attribute thrash 3-5× on uniform sky bands.
 *
 *   3. The sun glyph check piggy-backs on the per-pixel cos(ray·sun)
 *      we computed for the phase function — no extra trig per frame.
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;

    const PatternParams *pp = &patterns[s->current_pattern];

    DensityCtx ctx = {
        .seed          = s->seed,
        .time          = s->time,
        .wind          = s->wind,
        .threshold     = pp->threshold,
        .density_scale = pp->density_scale,
        .bottom        = pp->bottom,
        .top           = pp->top,
        .inv_height    = 1.0f / (pp->top - pp->bottom),
        .bell_exp      = pp->bell_exp,
    };

    /* Camera basis. */
    V3 cam_origin = v3(0.0f, CAM_HEIGHT, 0.0f);
    V3 fwd = v3(cosf(s->cam_pitch) * cosf(s->cam_yaw),
                sinf(s->cam_pitch),
                cosf(s->cam_pitch) * sinf(s->cam_yaw));
    V3 wup = v3(0, 1, 0);
    V3 right = v3norm(v3cross(fwd, wup));
    V3 up    = v3cross(right, fwd);

    V3 sun_dir = v3norm(v3(
        cosf(s->sun_pitch) * cosf(s->sun_yaw),
        sinf(s->sun_pitch),
        cosf(s->sun_pitch) * sinf(s->sun_yaw)));

    float fov_t       = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    float phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;

    int     last_pair = -1;
    attr_t  last_attr = 0;

    for (int row = 0; row < rows_eff; row++) {

        /* Sky vertical-gradient slot for this row (0 top → 2 horizon). */
        int sky_slot = (row * 3) / rows_eff;
        if (sky_slot < 0) sky_slot = 0;
        if (sky_slot > 2) sky_slot = 2;

        for (int col = 0; col < s->cols; col++) {
            float u =  ((float)col + 0.5f) / (float)s->cols * 2.0f - 1.0f;
            float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
            V3 dir = v3norm(v3add(fwd,
                v3add(v3scale(u * fov_t, right),
                      v3scale(v * fov_t * phys_aspect, up))));

            float L, T;
            raymarch_cloud(cam_origin, dir, sun_dir, &ctx, &L, &T);
            float alpha = 1.0f - T;

            float cos_to_sun = v3dot(dir, sun_dir);

            int    pair;
            attr_t attr;
            char   glyph;

            if (alpha > CLOUD_ALPHA_THR) {
                /*
                 * CLOUD CELL — takes precedence over the sun glyph.
                 * This is the "cloud OVER sun" composition: any cell
                 * the camera sees mostly cloud through, paints cloud,
                 * even if the ray happens to point near the sun.  The
                 * silver-lining glow (bright cloud edge backlit by the
                 * sun) emerges naturally from L being high there —
                 * the sun-facing far edge of the cloud has high sun_T
                 * and inherits the brightest cloud-ramp slot.
                 */
                float lum = L;
                if (lum > 1.0f) lum = 1.0f;
                if (lum < 0.0f) lum = 0.0f;
                int slot = (int)(lum * 7.999f);
                if (slot < 0) slot = 0;
                if (slot > 7) slot = 7;
                glyph = CLOUD_GLYPHS[slot];
                int cslot = slot * 6 / 8;        /* map 0..7 → 0..5     */
                if (cslot < 0) cslot = 0;
                if (cslot > 5) cslot = 5;
                pair = PAIR_CLOUD_BASE + cslot;
                attr = (slot >= 6) ? A_BOLD
                     : (slot <= 1) ? A_DIM
                     :               A_NORMAL;
            } else if (cos_to_sun > SUN_CORE_COS) {
                /* SUN CORE — innermost ring, only in clear sky        */
                pair  = PAIR_SUN;
                attr  = A_BOLD;
                glyph = SUN_GLYPH_CORE;
            } else if (cos_to_sun > SUN_MID_COS) {
                /* SUN MID — middle disc ring                          */
                pair  = PAIR_SUN;
                attr  = A_BOLD;
                glyph = SUN_GLYPH_MID;
            } else if (cos_to_sun > SUN_GLOW_COS) {
                /* SUN OUTER GLOW — outermost halo                     */
                pair  = PAIR_SUN;
                attr  = A_NORMAL;
                glyph = SUN_GLYPH_GLOW;
            } else if (L > RAY_LUM_THR) {
                /* God-ray cell — sky pixel with bright inscatter. */
                float lum = (L - RAY_LUM_THR) * 1.4f;
                if (lum > 1.0f) lum = 1.0f;
                if (lum < 0.0f) lum = 0.0f;
                int slot = (int)(lum * 7.999f);
                if (slot < 1) slot = 1;
                if (slot > 7) slot = 7;
                glyph = CLOUD_GLYPHS[slot];
                pair  = PAIR_RAY;
                attr  = (slot >= 5) ? A_BOLD : A_NORMAL;
            } else {
                /* Plain sky background. */
                pair  = PAIR_SKY_BASE + sky_slot;
                attr  = (sky_slot == 0) ? A_DIM : A_NORMAL;
                glyph = SKY_GLYPHS[sky_slot];
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
/* ── end §8 — to understand the ncurses wrapper, read §9 screen ── */

/* ===================================================================== */
/* §9  screen                                                             */
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

    char buf[220];
    snprintf(buf, sizeof buf,
             " CLOUDSCAPE   %s   pat:%s   theme:%s   "
             "wind:%4.1f   sun(%+5.2f,%+5.2f)   "
             "%5.1f fps  %3d Hz   "
             "n/N:pat  t/T:theme  arrows:sun  +/-:wind  r:reseed  spc:pause  q:quit ",
             s->paused ? "PAUSED " : "DRIFTING",
             pattern_name(s->current_pattern),
             themes[s->current_theme].name,
             (double)s->wind,
             (double)s->sun_yaw, (double)s->sun_pitch,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10 app                                                                */
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
    case 'r': case 'R': scene_reseed(s);                              break;

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

    case KEY_LEFT:  s->sun_yaw   -= SUN_KEY_STEP; break;
    case KEY_RIGHT: s->sun_yaw   += SUN_KEY_STEP; break;
    case KEY_UP:
        s->sun_pitch += SUN_KEY_STEP;
        if (s->sun_pitch > SUN_PITCH_MAX) s->sun_pitch = SUN_PITCH_MAX;
        break;
    case KEY_DOWN:
        s->sun_pitch -= SUN_KEY_STEP;
        if (s->sun_pitch < SUN_PITCH_MIN) s->sun_pitch = SUN_PITCH_MIN;
        break;

    case '=': case '+':
        s->wind += 0.5f;
        if (s->wind > WIND_MAX) s->wind = WIND_MAX;
        break;
    case '-':
        s->wind -= 0.5f;
        if (s->wind < WIND_MIN) s->wind = WIND_MIN;
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
