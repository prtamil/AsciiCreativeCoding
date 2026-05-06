/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * god_rays_silhouette.c — volumetric god rays through a 2-D silhouette
 *
 *   Continuous-RGB pipeline with blackbody-coloured sun, screen-space
 *   ray-march visibility, bloom post-process, and wind-blown dust
 *   particles with motion-blur trails.
 *
 * DEMO: A solid black silhouette dominates the lower foreground —
 *       an archway, a mountain peak, a broken column, a row of
 *       cathedral windows, or a tree. Behind it, a bright sun
 *       (whose colour is set by a Kelvin temperature) glows through
 *       dim fog. Wherever the sun can "see" the camera through a
 *       gap, a wide divergent SHAFT of light streaks outward — like
 *       dust-mote sunbeams in a dim room. Bright shaft cores BLOOM
 *       softly into surrounding cells. Tiny DUST PARTICLES drift
 *       through the beams with motion-blur trails, gusting under a
 *       slow Perlin wind. The sun itself has a CORE, a CORONA halo,
 *       and 4 LENS-FLARE STREAKS.
 *
 *       PATTERNS  (n / p):
 *         ARCHWAY    two stone pillars + curved arch top
 *         MOUNTAIN   a single peak silhouette
 *         COLUMN     a tall broken column
 *         WINDOWS    cathedral wall pierced by 4×2 arched windows
 *         TREE       single L-system tree silhouette
 *
 *       SUN TEMPERATURE  (t / T):
 *         EMBER  1500K    glowing-coal red
 *         SUNSET 2000K    orange sunset
 *         CANDLE 2500K    candle flame warm
 *         TORCH  3500K    tungsten bulb
 *         WARM   4500K    golden hour
 *         DAY    5500K    midday sun (D55)
 *         NOON   6500K    clear noon (D65)
 *         BLUE   8500K    overcast / blue hour
 *
 *       SHADE MODE  (m):
 *         LIT   — full pipeline (default)
 *         MASK  — silhouette only, no rays — see the SHAPE
 *         VIS   — grayscale visibility map — see the ALGORITHM output
 *
 * Study alongside:
 *   raytracing/path_tracer.c       — canonical RGB → 6×6×6 cube paint
 *   raytracing/saturn_with_rings.c — same continuous-RGB pipeline
 *   raytracing/sphere_raytrace.c   — entry-level analytic raytracing
 *
 * Section map:
 *   §1 config          — frame rate, march, sun, bloom, dust constants
 *   §2 clock           — monotonic timer + sleep
 *   §3 math + palette  — V3, RGB helpers, blackbody, palette_from_kelvin
 *   §4 ncurses paint   — 6×6×6 cube + density ramp + tone-map
 *   §5 silhouette      — five point-in-shape functions + dispatcher
 *   §6 noise + RNG     — Perlin + fBm + xorshift + hash3
 *   §7 scene + sun     — Scene state, sun motion, init/reseed/tick
 *   §8 raymarch        — THE CORE: visibility, sun, jitter, RGB shade
 *   §9 buffer + post   — V3 buffer, dust trails, wind gusts, bloom
 *   §10 screen         — scene_draw (4-pass LIT), HUD spec compliance
 *   §11 app            — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume sun drift
 *   r            reseed (sun phase, dust positions)
 *   n / p        next / previous pattern
 *   t / T        next / previous sun temperature preset
 *   m            cycle shade mode  (LIT → MASK → VIS)
 *   b            toggle bloom (LIT mode only)
 *   d            toggle dust   (LIT mode only)
 *   + / =        faster sun drift   (-: slower)
 *   ] / [        raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/god_rays_silhouette.c \
 *       -o god_rays_silhouette -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Screen-space volumetric light shafts via per-cell
 *                  shadow-ray accumulation. For each cell C the renderer
 *                  asks "how much light reaches the camera by scattering
 *                  off fog molecules along the line from C to the sun?"
 *                  Sample N points along the line; each lit sample
 *                  contributes weight w_i = exp(−σ·d_i); the weighted
 *                  fraction of unblocked samples is the cell's visibility
 *                  ∈ [0, 1]. The output is composed in continuous RGB:
 *                    base = lerp(fog, shaft, visibility)
 *                    sun  = sun_col · gaussian_disc + corona + flare
 *                    out  = (base + sun) · perlin_jitter
 *                  then dust-overlaid, bloom-blurred, tone-mapped, and
 *                  quantised to a 216-pair RGB cube + 16-char density
 *                  ramp at draw time. Sun colour is derived from a
 *                  Kelvin temperature via Planck's blackbody law.
 *
 * Data-structure : Per frame: V3 colour buffer g_buf[H][W] populated by
 *                  the ray-march, then mutated by dust drop-in, then
 *                  read-modified by the bloom pass before paint. Dust
 *                  is a static array of N particles with motion trails.
 *                  No malloc anywhere: all sizes are bounded at compile
 *                  time. The Palette is built ONCE per frame from one
 *                  Kelvin value — there are no preset themes, every
 *                  hue is physically derived.
 *
 * Rendering      : Reinhard tone-map L'=L/(1+L) → gamma encode 1/2.2
 *                  → quantise to xterm 6×6×6 cube → density char from
 *                  a curated 16-char open-glyph ramp. A_BOLD on the
 *                  brightest bin, A_DIM on the lowest. The ramp uses
 *                  ONLY OPEN GLYPHS (` .'`,-_:;~=+*oO0`) — no closed
 *                  blocks like `#` or `@` — so bright cells read as
 *                  POINTS OF LIGHT, not as solid pixel walls.
 *
 * Performance    : MARCH_STEPS · cols · rows silhouette evaluations
 *                  per frame, plus a 5×5 bloom pass and N_DUST cell
 *                  writes. At MARCH_STEPS=20 on 240×80, that's
 *                  ≈384 000 silhouette evals + 480 000 bloom samples
 *                  + 220 dust adds (× ~5 trail cells each) — all
 *                  simple float math, comfortably under the 33 ms
 *                  budget at 30 Hz.
 *
 * References     : Mitchell, "Volumetric Light Scattering as a Post-
 *                    Process," GPU Gems 3 ch. 13. (Screen-space god-
 *                    rays algorithm.)
 *                  Hoffman & Preetham, "Real-time Light/Atmosphere
 *                    Interactions for Outdoor Scenes," Game Programming
 *                    Gems 5.
 *                  Reinhard et al., "Photographic Tone Reproduction
 *                    for Digital Images," SIGGRAPH '02.
 *                  Tanner Helland, "How to convert temperature to RGB"
 *                    https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
 *                    (Practical blackbody approximation used in §3.)
 *                  James, "Bloom Post-Process Effects," Graphics
 *                    Programming Methods (Charles River 2003).
 *                  Wikipedia: Crepuscular rays — atmospheric phenomenon.
 *                  Wikipedia: Beer-Lambert law — extinction formula.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell asks the sun: "is there a clear line from you to me,
 * through the fog?" Walk N small steps from the cell toward the sun;
 * each step is "lit" if it's outside the silhouette, weighted by
 * Beer-Lambert. The total is how much light gets through. Bright
 * shafts emerge wherever the line from cell to sun threads a CLEAR
 * channel through the silhouette; dark fog fills the cells where
 * every step lies inside the silhouette.
 *
 * Five additive layers stack on top of that core:
 *
 *   COLOUR    Continuous RGB blending instead of preset themes. Sun
 *             colour comes from one Kelvin value via the blackbody
 *             curve; fog, dust, and sky colours are DERIVED from the
 *             sun (no separate theme tuning). Quantise to the
 *             6×6×6 cube only at paint time.
 *   SUN       Not just a Gaussian disc — CORE + CORONA halo + 4
 *             LENS-FLARE STREAKS at fixed angles. Makes the sun
 *             read as a real luminous body.
 *   BLOOM     Bright cells leak warm light into 5×5 neighbours.
 *             That is what makes shafts feel ATMOSPHERIC instead of
 *             stencilled.
 *   DUST      Sparkle particles drift across the scene with
 *             motion-blur trails, gusted by a slow Perlin wind.
 *             Brightness gated by local visibility — only sparkle
 *             inside shafts.
 *   RAMP      Curated open-glyph density ramp. Bright cells use
 *             open circles (`o`, `O`, `0`) not closed blocks (`#`,
 *             `@`). Makes bright cells read as LIGHT, not WALLS.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a dim hallway with a door at one end. Light streams through
 * a keyhole, lighting the dust hanging in the air. From the other end
 * you see a bright cone radiating from the keyhole — those are the
 * GOD RAYS. At a point on the keyhole-to-eye line every dust mote on
 * that line is lit (the keyhole sees those motes), so the shaft is
 * brightest. Off-axis only the motes near the door see the keyhole;
 * the rest sit in the door's shadow.
 *
 *   ┌────── door ──────┐         ← silhouette plane
 *   │░░░░░░░░░░░░░░░░░░│
 *   │░░░░░░░░·░░░░░░░░░│         · = keyhole (gap in silhouette)
 *   │░░░░░░·│┃·░░░░░░░░│         ┃ = dust particle in the shaft
 *   │░░░░·  │   ·░░░░░░│             with a motion-blur trail behind
 *   │░░·    │     ·░░░░│
 *   │·      │       ·░░│
 *   ↑                 ↑
 *   eye            camera
 *
 * Sun temperature changes the COLOUR of the rumour rolling through
 * the gossip chain (1500K → red, 6500K → white, 8500K → blue). All
 * the dust, fog, and shaft colour follow because they all bounce the
 * same primary light. Bloom turns shaft cores from "stencil bright"
 * into "atmospheric glow." Dust trails make the shafts feel ALIVE.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────────────────
 *  1. Compute SUN POSITION (slow horizontal drift across upper screen).
 *  2. Build PALETTE from sun temperature:
 *       sun  = blackbody(K)
 *       fog  = sun · 0.10 + cool ambient blue
 *       dust = sun · 0.92
 *  3. Pre-test: is the sun behind the silhouette? If so, skip the
 *     sun-disc halo step.
 *  4. PASS 1 — RAY-MARCH (V3 buffer):
 *       per cell:
 *         if silhouette → buf[r][c] = silhouette_rgb
 *         else:
 *           vis      = march_visibility(cell, sun)           ← §8.2
 *           sun_term = sun_disc(cell, sun, blocked)            ← §8.3
 *           jitter   = fog_jitter(uv, time·FOG_WIND)           ← §8.4
 *           buf[r][c] = shade_lit_rgb(palette, vis, sun_term,
 *                                      jitter)                  ← §8.5
 *           visbuf[r][c] = vis + sun_term                       (for dust)
 *  5. PASS 2 — DUST OVERLAY:
 *       advance dust positions with WIND-GUSTED velocity.
 *       record positions into per-particle TRAIL.
 *       paint head + 4 trail samples at falling brightness,
 *       each gated by local visibility.
 *  6. PASS 3 — BLOOM (5×5 bright-pass Gaussian):
 *       for each cell, gather contributions from neighbours
 *       whose luma > BLOOM_THRESHOLD, weighted by Gaussian.
 *       Write into a SEPARATE bloom buffer first; fold back
 *       into g_buf after the loop (avoids feedback compounding).
 *  7. PASS 4 — PAINT:
 *       per cell: paint_cell(c, r, buf[r][c]) →
 *       Reinhard → gamma → 6×6×6 cube quantise → density char.
 *
 * KEY FORMULAS
 * ────────────
 *  Aspect-correct cell → normalised:
 *    u = (2·sx + 1 − cols) / cols
 *    v = ((2·sy + 1 − rows) / rows) · (ASPECT_Y · rows / cols)
 *
 *  Beer-Lambert weighted visibility:
 *    Δsx, Δsy = (sun_sx − sx, sun_sy − sy) / MARCH_STEPS
 *    step_len = √(Δsx² + (ASPECT_Y · Δsy)²)
 *    w_i      = exp(−σ · i · step_len)
 *    vis      = (Σ_{i: lit} w_i) / (Σ_{all i} w_i)
 *
 *  Cinematic sun (CORE + CORONA + FLARE):
 *    core   = exp(−r²/CORE_FALLOFF²)
 *    corona = exp(−r²/CORONA_FALLOFF²) · CORONA_GAIN
 *    flare  = Σ_{s=0..N-1} exp(−Δa²/ANG_SIGMA²) · exp(−r²/R_FALLOFF²)
 *               where Δa = wrap(angle − s·π/N) into [-π/2, π/2]
 *    sun_term = core + corona + FLARE_GAIN · flare
 *
 *  Continuous RGB shading (LIT mode):
 *    base = lerp(palette.fog, palette.shaft, vis)
 *    sun  = palette.sun · sun_term
 *    col  = (base + sun) · jitter
 *
 *  Wind gust (Perlin-modulated dust speed):
 *    g = 1 + WIND_AMP · perlin(time · WIND_FREQ_HZ)   in ≈[0.5, 1.5]
 *    dust velocity *= g
 *
 *  Dust trail (distance-spaced):
 *    each frame, advance dist_acc += step
 *    if dist_acc ≥ TRAIL_SPACING:
 *      shift trail; push current position; reset acc
 *    paint head + 4 trail samples at 100/55/32/18/10% brightness,
 *    each multiplied by visibility[gridcell].
 *
 *  Bloom (5×5 bright-pass Gaussian):
 *    bloom(c, r) = Σ_{Δr,Δc ∈ [-2..2], luma(buf[r+Δr][c+Δc]) > T}
 *                    buf[r+Δr][c+Δc] · exp(−(Δr² + Δc²) / σ²) · GAIN
 *    output      = buf[r][c] + bloom(c, r)
 *
 *  Reinhard tone map + gamma + 6×6×6 cube:
 *    L'  = L / (1 + L)
 *    out = L'^(1/2.2)
 *    cube_pair = round(out_r · 5)·36 + round(out_g · 5)·6 + round(out_b · 5) + 1
 *
 *  Sun colour from Kelvin (Tanner Helland piecewise):
 *    K = T / 100
 *    if K ≤ 66:  R=1, G=(99.47·ln K − 161.12)/255, B=(138.52·ln(K−10) − 305.04)/255
 *    if K > 66:  R=329.7·(K−60)^−0.1332/255, G=288.12·(K−60)^−0.0755/255, B=1
 *
 * WORKED EXAMPLE  (verify by hand, NOON 6500K)
 * ────────────────────────────────────────────
 *   Sun temp = 6500K (NOON preset). Blackbody approximation:
 *     K = 65       → R = 1.0
 *     G = (99.47·ln 65 − 161.12)/255 ≈ (415.4 − 161.1)/255 ≈ 0.998
 *     B = (138.52·ln 55 − 305.04)/255 ≈ (555.0 − 305.0)/255 ≈ 0.980
 *     palette.sun = (1.000, 0.998, 0.980)        ← near-white, slight warm
 *     palette.fog = sun·0.10 + (0.04, 0.06, 0.12)
 *                 = (0.140, 0.160, 0.218)        ← dim cool haze
 *
 *   Cell at vis=0.85, sun_term=0.10, jitter=1.05:
 *     base = lerp(fog, sun, 0.85·1.20)           ← SHAFT_GAIN=1.2 → 1.0
 *          = sun                                  ← clamp at 1.0
 *          = (1.000, 0.998, 0.980)
 *     sun  = sun_col · 0.10 · 1.50 = (0.150, 0.150, 0.147)
 *     col  = (1.150, 1.148, 1.127) · 1.05 ≈ (1.21, 1.21, 1.18)
 *
 *   Reinhard:        (0.547, 0.547, 0.541)
 *   Gamma 2.2:       (0.760, 0.760, 0.755)
 *   6×6×6 quantise:  r5=4, g5=4, b5=4 → pair 4·36+4·6+4+1 = 173
 *                    (a clean off-white in the cube)
 *   Luminance:       0.21·0.76 + 0.72·0.76 + 0.07·0.76 ≈ 0.76
 *   Ramp index:      round(0.76 · 15) = 11 → '+' glyph
 *                    (with A_BOLD because luma > 0.85? no, 0.76 < 0.85,
 *                     so A_NORMAL — a bright but not BOLD `+`)
 *
 *   Now move one cell sideways (vis=0.40, sun_term=0):
 *     base = lerp(fog, sun, 0.40·1.20) = lerp(fog, sun, 0.48)
 *          ≈ (0.553, 0.562, 0.594)                ← partly shaft
 *     col  = base · 1.05 ≈ (0.581, 0.590, 0.624)
 *     Reinhard:   (0.367, 0.371, 0.384)
 *     Gamma 2.2:  (0.628, 0.632, 0.642)
 *     Cube:       r5=3, g5=3, b5=3 → pair 130 (medium warm)
 *     Luma 0.63 → ramp idx 9 → '~'
 *
 *   So adjacent cells go from `+` (bright) to `~` (medium) — a smooth
 *   gradient, NOT a banded jump. Continuous RGB makes that smoothness
 *   possible with the limited 16-char ramp.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • TONE-MAP BEFORE QUANTISE. Tone-map and gamma must run BEFORE
 *    cube quantisation. Quantising linear HDR puts every pixel into
 *    one cube cell — no headroom.
 *  • BLOOM SEPARATION. The bloom pass READS g_buf and writes to a
 *    SEPARATE g_bloom buffer, then folds back into g_buf. Single-
 *    pass would compound asymmetrically.
 *  • DUST RESPAWN ON RESEED. The 'r' key shifts the sun phase; we
 *    respawn dust at random positions so the visual flow tracks.
 *  • DUST VISIBILITY GATE. A particle in shadow contributes nothing
 *    — multiplying by `vis` ensures dust only sparkles where it
 *    would actually be illuminated.
 *  • BLOOM THRESHOLD. Set too low → entire scene blooms (washed
 *    out). Set too high → only the sun blooms (effect invisible).
 *    BLOOM_LUMA_THRESHOLD = 0.55 works well.
 *  • SUN FLARE r=0 EDGE. atan2(0,0) is undefined; skip the flare
 *    contribution when r < 0.5 — at that distance the core dominates.
 *  • BLACKBODY DOMAIN. Tanner Helland's formulas need K > 19 for
 *    the blue channel and K > 66 split for red/green. The presets
 *    (1500-8500K) all fall comfortably inside the valid range.
 *  • RAMP INDEX BOUNDS. Tone-mapped values can exceed 1.0 by tiny
 *    floating-point margins; clamp `ri` to [0, RAMP_LEN-1] before
 *    indexing.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space) and cycle TEMPERATURE (t/T): the entire scene's
 *    chromaticity should shift continuously — at EMBER 1500K the
 *    shafts are deep red and the fog warm dim; at NOON 6500K
 *    everything is near-white; at BLUE 8500K the shafts are cool
 *    blue-white. This is REAL physics — no artistic tinting.
 *  • Toggle BLOOM (b): bright shaft cores should noticeably "glow
 *    into" their neighbours when bloom is on.
 *  • Toggle DUST (d): tiny warm sparkles should drift across the
 *    scene, only visible when they pass through a shaft. With dust
 *    on, watch a single particle to see its motion-blur trail
 *    fading behind the head.
 *  • Cycle MODE (m): MASK shows just the silhouette (clean shape);
 *    VIS shows pure grayscale visibility (the algorithm output);
 *    LIT shows the full pipeline.
 *  • Watch the SUN FLARE: 4 sparkly streaks at 0°/45°/90°/135°
 *    radiating from the sun core. The streaks are STATIC angles
 *    (lens-flare-like) — they don't rotate as the sun moves.
 *  • Wind gusts: at long timescales (10+ sec), watch dust speed.
 *    It should ebb and flow visibly — sometimes streaming, sometimes
 *    drifting almost stationary.
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
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate */
enum {
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  30,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,

    SPEED_MIN       =   1,
    SPEED_DEF       =   8,
    SPEED_MAX       =  64,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))
#define DT_CAP_NS       (100 * NS_PER_MS)

/* §1.2 view geometry */
#define ASPECT_Y        2.0f          /* terminal cells ~2× taller         */

/* §1.3 ray-march dynamics */
#define MARCH_STEPS         20        /* samples per cell along ray-to-sun */
#define FOG_SIGMA           0.045f    /* Beer-Lambert extinction per cell  */
#define SHAFT_GAIN          1.20f
#define SUN_GAIN            1.50f

/* §1.4 sun motion (azimuth drift across the upper screen) */
#define SUN_DRIFT_PERIOD_S  20.0f
#define SUN_X_AMP_FRAC      0.30f     /* of cols                          */
#define SUN_Y_FRAC          0.18f     /* base row position as frac of rows*/
#define SUN_Y_AMP_FRAC      0.04f     /* slight vertical drift            */

/* §1.4b cinematic sun composition.
 *
 *   CORE    sharp Gaussian — the bright dot
 *   CORONA  wide dim Gaussian — the warm halo
 *   FLARE   narrow angular spokes — lens-flare streaks
 *
 * sun_term(cell) = core + corona + FLARE_GAIN · Σ flare_spokes
 */
#define SUN_CORE_FALLOFF      3.5f    /* sharp core radius (cells)        */
#define SUN_CORONA_FALLOFF    9.0f    /* wider warm halo radius           */
#define SUN_CORONA_GAIN       0.35f
#define SUN_FLARE_R_FALLOFF  18.0f    /* flare radial extent              */
#define SUN_FLARE_ANG_SIGMA   0.06f   /* flare angular half-width (rad)   */
#define SUN_FLARE_GAIN        0.55f
#define SUN_N_FLARES          4       /* number of equiangular spokes     */

/* §1.5 fog wind (subtle visual modulation) */
#define FOG_WIND            0.8f
#define FOG_JITTER_AMP      0.10f

/* §1.6 bloom (5×5 bright-pass Gaussian) */
#define BLOOM_RADIUS         2        /* (2R+1)² kernel = 5×5             */
#define BLOOM_GAIN           0.55f    /* contribution multiplier          */
#define BLOOM_LUMA_THRESHOLD 0.55f    /* luma above which a cell glows    */
#define BLOOM_SIGMA2         3.0f     /* Gaussian σ² in cell units        */

/* §1.7 dust particles */
#define DUST_MAX             220
#define DUST_SPEED_X         5.0f     /* cells/sec horizontal drift       */
#define DUST_SPEED_Y         0.8f     /* cells/sec vertical drift         */
#define DUST_LIFE_MEAN       4.0f     /* sec — mean particle lifetime     */
#define DUST_LIFE_VAR        2.0f     /* sec — uniform variance           */
#define DUST_BRIGHTNESS      0.55f    /* peak additive brightness         */

/* §1.7b dust motion blur — distance-spaced trail, 4 samples behind head. */
#define DUST_TRAIL_LEN        4
#define DUST_TRAIL_SPACING    0.6f    /* cells between trail samples       */
static const float DUST_TRAIL_BR[DUST_TRAIL_LEN] = {
    0.55f, 0.32f, 0.18f, 0.10f
};

/* §1.7c wind gusts — slow Perlin field modulating dust speed. */
#define WIND_GUST_FREQ_HZ     0.10f   /* gust period ≈ 10 sec              */
#define WIND_GUST_AMP         0.50f   /* speed varies in [0.5x, 1.5x]      */

/* §1.8 buffer dimensions (static, no malloc) */
#define BUF_MAX_W           400
#define BUF_MAX_H           200

/* §1.9 ncurses pair IDs */
enum {
    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_SILHOUETTE   =  3,
    PAIR_SUN          =  4,
    PAIR_MASK_FOG     =  5,
    PAIR_GRAY_BASE    =  8,    /* +0..+7  = 8 grayscale tints (VIS)       */
    PAIR_CUBE_BASE    = 20,    /* +0..+215 = 6×6×6 RGB cube (LIT)         */
};

/* §1.10 ASCII density ramp — airy atmospheric, sparse → luminous.
 *
 * Design goal: every character should read as a level of LIGHT, not
 * as a SOLID OBJECT. So the brightest end is OPEN glyphs (`o`, `O`,
 * `0`) whose silhouette has whitespace inside them — luminous-feeling
 * like points of light. Closed/filled glyphs (`#`, `%`, `@`) are
 * deliberately EXCLUDED because they read as walls or blocks, which
 * is wrong for sun-shafts and dust.
 *
 *   index 0       ' '            empty
 *   index 1-5     . ' ` , -      sparse single-mark family
 *   index 6-9     _ : ; ~        thin two-mark family
 *   index 10-12   = + *          symmetric crosses
 *   index 13-15   o O 0          OPEN-circle family (the bright end)
 */
static const char k_ramp[] = " .'`,-_:;~=+*oO0";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* For VIS-mode grayscale ramp — small 8-glyph blocky ramp. */
static const char RAMP8[8] = { '`', '.', ',', ':', ';', '-', '+', '*' };

/* §1.11 patterns */
typedef enum {
    PATTERN_ARCHWAY  = 0,
    PATTERN_MOUNTAIN = 1,
    PATTERN_COLUMN   = 2,
    PATTERN_WINDOWS  = 3,
    PATTERN_TREE     = 4,
    N_PATTERNS       = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_ARCHWAY:  return "ARCHWAY ";
    case PATTERN_MOUNTAIN: return "MOUNTAIN";
    case PATTERN_COLUMN:   return "COLUMN  ";
    case PATTERN_WINDOWS:  return "WINDOWS ";
    case PATTERN_TREE:     return "TREE    ";
    default:               return "?       ";
    }
}

/* §1.12 shade mode (cycled with 'm') */
typedef enum {
    SHADE_LIT  = 0,    /* full pipeline: RGB + bloom + dust              */
    SHADE_MASK = 1,    /* silhouette only, no rays                       */
    SHADE_VIS  = 2,    /* grayscale visibility map (no theme tint)       */
    SHADE_N    = 3,
} ShadeMode;

static const char *shade_mode_name(ShadeMode m)
{
    switch (m) {
    case SHADE_LIT:  return "LIT ";
    case SHADE_MASK: return "MASK";
    case SHADE_VIS:  return "VIS ";
    default:         return "?   ";
    }
}

/* §1.13 sun-temperature presets.
 *
 * Each entry is one Kelvin value; the entire scene chromaticity is
 * derived from it via the blackbody curve in §3. There are no
 * "themes" — chromaticity is physically grounded.
 */
typedef struct {
    const char *name;
    float       kelvin;
} TempPreset;

static const TempPreset TEMP_PRESETS[] = {
    { "EMBER ", 1500.0f },   /* glowing-coal red                         */
    { "SUNSET", 2000.0f },   /* orange sunset                             */
    { "CANDLE", 2500.0f },   /* candle flame warm                         */
    { "TORCH ", 3500.0f },   /* tungsten bulb                             */
    { "WARM  ", 4500.0f },   /* golden hour                               */
    { "DAY   ", 5500.0f },   /* midday sun (D55)                          */
    { "NOON  ", 6500.0f },   /* clear noon (D65)                          */
    { "BLUE  ", 8500.0f },   /* overcast / blue hour                      */
};
#define N_TEMP_PRESETS ((int)(sizeof TEMP_PRESETS / sizeof TEMP_PRESETS[0]))

/* ── §2 clock ────────────────────────────────────────────────────────── */

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

/* ── §3 math + palette ───────────────────────────────────────────────── */

typedef struct { float r, g, b; } V3;

static inline V3    v3_make (float r, float g, float b) { return (V3){r,g,b}; }
static inline V3    v3_add  (V3 a, V3 b)   { return v3_make(a.r+b.r, a.g+b.g, a.b+b.b); }
static inline V3    v3_scl  (V3 a, float s){ return v3_make(a.r*s, a.g*s, a.b*s);       }
static inline V3    v3_lerp (V3 a, V3 b, float t)
{ return v3_make(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t); }

static inline float clamp01(float x)
{ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

static inline float reinhard (float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

static inline float luma_of(V3 c)
{
    return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
}

/* §3.5 Palette (derived once per frame from the sun's Kelvin) ──────── */

/*
 * Palette — every colour the scene needs.
 *   sun     direct sun radiance
 *   shaft   sun light scattered into the camera (= sun, in this model)
 *   fog     ambient haze (dim sun + cool sky)
 *   dust    dust particle albedo · sun light
 *   sky     very dim background tint
 */
typedef struct {
    V3 sun;
    V3 shaft;
    V3 fog;
    V3 dust;
    V3 sky;
} Palette;

/*
 * sun_color_from_kelvin — Tanner Helland's piecewise blackbody approx.
 *
 *   T ≤ 6600 K   red dominates, green follows log curve, blue grows
 *   T  > 6600 K  red and green decay, blue saturates at 1
 *
 * Returns RGB ∈ [0, 1]. Brightness is held constant; chromaticity is
 * what matters here (the actual radiance grows ∝ T⁴ but we let SUN_GAIN
 * govern visual brightness).
 */
static V3 sun_color_from_kelvin(float kelvin)
{
    float K = kelvin / 100.0f;
    float r, g, b;

    /* Red. */
    if (K <= 66.0f) r = 1.0f;
    else            r = 329.7f * powf(K - 60.0f, -0.1332f) / 255.0f;

    /* Green. */
    if (K <= 66.0f) g = (99.47f  * logf(K)            - 161.12f) / 255.0f;
    else            g =  288.12f * powf(K - 60.0f, -0.0755f)      / 255.0f;

    /* Blue. */
    if (K >= 66.0f)      b = 1.0f;
    else if (K <= 19.0f) b = 0.0f;
    else                 b = (138.52f * logf(K - 10.0f) - 305.04f) / 255.0f;

    return v3_make(clamp01(r), clamp01(g), clamp01(b));
}

/*
 * palette_from_kelvin — derive every scene colour from one Kelvin.
 *
 * Physical reasoning:
 *   shaft   The shaft is direct sunlight scattered into the camera by
 *           fog particles — same chromaticity as the sun.
 *   sun     The on-axis sun-disc cells receive direct sun light.
 *   fog     Cells far from any shaft receive multiple-scattered sun
 *           light + a dim sky-blue ambient (Rayleigh-like residual).
 *   dust    Dust particles are illuminated by the same sun.
 *   sky     Very dim background — small fraction of fog + ambient blue.
 *
 * AMB_BLUE is the blue tint Rayleigh scattering would leave even on a
 * low-temperature sun day; it's why even sunset sees SOME blue in the
 * upper sky.
 */
static Palette palette_from_kelvin(float kelvin)
{
    V3 sun      = sun_color_from_kelvin(kelvin);
    V3 amb_blue = v3_make(0.04f, 0.06f, 0.12f);

    Palette p;
    p.sun   = sun;
    p.shaft = sun;
    p.dust  = v3_scl(sun, 0.92f);
    p.fog   = v3_add(v3_scl(sun, 0.10f), amb_blue);
    p.sky   = v3_add(v3_scl(sun, 0.03f), v3_scl(amb_blue, 0.4f));
    return p;
}

/* ── §4 ncurses paint ────────────────────────────────────────────────── */

static int g_256;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);

    if (g_256) {
        /* 6×6×6 RGB cube — 216 pairs at PAIR_CUBE_BASE..+215. */
        for (int i = 0; i < 216; i++)
            init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);

        /* Grayscale ramp for VIS mode. */
        static const short grays[8] = { 240, 243, 246, 249, 251, 253, 254, 255 };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_GRAY_BASE + i), grays[i], -1);

        init_pair(PAIR_HUD,         226, -1);    /* bright yellow */
        init_pair(PAIR_HINT,         51, -1);    /* bright cyan   */
        init_pair(PAIR_SILHOUETTE,  232, -1);    /* near-black    */
        init_pair(PAIR_SUN,         231, -1);    /* near-white    */
        init_pair(PAIR_MASK_FOG,    246, -1);    /* mid-gray      */
    } else {
        /* 8-colour fallback — coarse, no cube. Density ramp only. */
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_GRAY_BASE + i), COLOR_WHITE, -1);
        init_pair(PAIR_CUBE_BASE,  COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,        COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,   -1);
        init_pair(PAIR_SILHOUETTE, COLOR_BLACK,  -1);
        init_pair(PAIR_SUN,        COLOR_YELLOW, -1);
        init_pair(PAIR_MASK_FOG,   COLOR_WHITE,  -1);
    }
}

/*
 * paint_cell — full RGB → terminal pipeline.
 *
 *   1. Reinhard tone-map per channel:  L' = L / (1 + L)
 *   2. Gamma encode 1/2.2:               sRGB-perceptual values.
 *   3. Quantise to 6×6×6 cube → pair id.
 *   4. Compute Rec.601 luminance of encoded RGB → density glyph.
 *   5. A_BOLD on the brightest cells, A_DIM on the darkest.
 *
 * Steps 1-2 run BEFORE step 3 — quantising linear HDR puts every
 * pixel into one cube cell because the cube is uniform in [0,1]³.
 * Tone-mapping opens up the dynamic range so colours spread.
 */
static void paint_cell(int sx, int sy, V3 col)
{
    float r = gamma_enc(reinhard(col.r));
    float g = gamma_enc(reinhard(col.g));
    float b = gamma_enc(reinhard(col.b));

    if (g_256) {
        int r5 = (int)(r * 5.f + 0.5f); if (r5 > 5) r5 = 5; if (r5 < 0) r5 = 0;
        int g5 = (int)(g * 5.f + 0.5f); if (g5 > 5) g5 = 5; if (g5 < 0) g5 = 0;
        int b5 = (int)(b * 5.f + 0.5f); if (b5 > 5) b5 = 5; if (b5 < 0) b5 = 0;
        int pair = PAIR_CUBE_BASE + r5*36 + g5*6 + b5;

        float luma = 0.2126f*r + 0.7152f*g + 0.0722f*b;
        int   ri   = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
        if (ri < 0)         ri = 0;
        if (ri >= RAMP_LEN) ri = RAMP_LEN - 1;
        int attr = (luma > 0.85f) ? A_BOLD
                 : (luma < 0.15f) ? A_DIM
                 :                  A_NORMAL;

        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy, sx, (chtype)(unsigned char)k_ramp[ri]);
        attroff(COLOR_PAIR(pair) | attr);
    } else {
        float luma = 0.2126f*r + 0.7152f*g + 0.0722f*b;
        int   ri   = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
        if (ri < 0)         ri = 0;
        if (ri >= RAMP_LEN) ri = RAMP_LEN - 1;
        attron(COLOR_PAIR(PAIR_CUBE_BASE));
        mvaddch(sy, sx, (chtype)(unsigned char)k_ramp[ri]);
        attroff(COLOR_PAIR(PAIR_CUBE_BASE));
    }
}

/* paint_cell_vis — grayscale ramp paint for VIS mode (no theme tint). */
static void paint_cell_vis(int sx, int sy, float intensity, float sun_term)
{
    if (sun_term > 0.6f) {
        char sg = (sun_term > 0.85f) ? '*' : '+';
        attron(COLOR_PAIR(PAIR_GRAY_BASE + 7) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)sg);
        attroff(COLOR_PAIR(PAIR_GRAY_BASE + 7) | A_BOLD);
        return;
    }
    intensity = clamp01(intensity);
    int gi = (int)(intensity * 7.999f);
    if (gi < 0) gi = 0;
    if (gi > 7) gi = 7;
    int attr = (gi >= 6) ? A_BOLD
             : (gi <= 1) ? A_DIM
             :             A_NORMAL;
    attron(COLOR_PAIR(PAIR_GRAY_BASE + gi) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)RAMP8[gi]);
    attroff(COLOR_PAIR(PAIR_GRAY_BASE + gi) | attr);
}

/* ── §5 silhouette — five point-in-shape functions ──────────────────── */

/* All silhouette functions take normalised aspect-corrected coords:
 *   u ∈ [-1, +1]  screen-x (0 = centre)
 *   v ∈ [-1, +1]  screen-y (0 = centre, +v = down)
 * Return true if (u, v) is INSIDE the silhouette (opaque), false if
 * the point is in clear fog. Pattern dispatch is just a switch. */

/* §5.1 ARCHWAY: two pillars + curved arch top + ground */

static bool sil_archway(float u, float v)
{
    if (v > 0.70f) return true;                              /* ground   */
    /* Two pillars at u = ±0.40, half-width 0.08, from v=0 down. */
    if (fabsf(u - 0.40f) < 0.08f && v >= 0.0f) return true;
    if (fabsf(u + 0.40f) < 0.08f && v >= 0.0f) return true;
    /* Arch top — annular sector for v < 0. */
    if (v < 0.0f) {
        float r = sqrtf(u * u + v * v);
        if (r > 0.32f && r < 0.55f) return true;
    }
    /* Capstone block. */
    if (v > -0.65f && v < -0.55f && fabsf(u) < 0.55f) return true;
    return false;
}

/* §5.2 MOUNTAIN: gaussian-bump peak + side bump */

static bool sil_mountain(float u, float v)
{
    /* Main peak + small subsidiary peak to the right for character. */
    float p = 0.70f
            - 1.10f * expf(-(u * u) / 0.30f)
            - 0.35f * expf(-((u - 0.55f) * (u - 0.55f)) / 0.05f);
    return v > p;
}

/* §5.3 COLUMN: tapered shaft + capital + jagged broken top */

static bool sil_column(float u, float v)
{
    if (v > 0.80f) return true;                              /* ground   */

    /* Capital — slightly wider band at top of shaft. */
    if (fabsf(u) < 0.13f && v > -0.65f && v < -0.50f) return true;

    /* Shaft. */
    if (fabsf(u) < 0.08f && v > -0.50f && v < 0.80f) return true;

    /* Broken jagged top: cosine notches just above the capital. */
    if (fabsf(u) < 0.13f && v > -0.78f && v < -0.65f) {
        float jag = 0.5f * cosf(u * 18.0f) + 0.5f;
        if (v > -0.78f + jag * 0.13f) return true;
    }
    return false;
}

/* §5.4 WINDOWS: cathedral wall pierced by 4×2 arched openings */

static bool sil_windows(float u, float v)
{
    /* Wall outer bounds. */
    if (u < -0.90f || u > 0.90f) return false;
    if (v < -0.85f || v > 0.85f) return false;

    /* Window region: a rectangle inside the wall. Outside this region
     * is solid wall stone. */
    const int   N_COLS = 4;
    const int   N_ROWS = 2;
    const float WIN_LEFT   = -0.80f;
    const float WIN_RIGHT  =  0.80f;
    const float WIN_TOP    = -0.70f;
    const float WIN_BOTTOM =  0.55f;

    if (u < WIN_LEFT || u > WIN_RIGHT) return true;
    if (v < WIN_TOP  || v > WIN_BOTTOM) return true;

    /* Map (u, v) to (gu, gv) ∈ [0,1]² across the whole window region. */
    float gu = (u - WIN_LEFT) / (WIN_RIGHT  - WIN_LEFT);
    float gv = (v - WIN_TOP)  / (WIN_BOTTOM - WIN_TOP);

    /* Tile index — which column and row this point is in. */
    int cx = (int)(gu * (float)N_COLS); if (cx >= N_COLS) cx = N_COLS - 1;
    int cy = (int)(gv * (float)N_ROWS); if (cy >= N_ROWS) cy = N_ROWS - 1;

    /* Local position within the tile. */
    float wu = gu * (float)N_COLS - (float)cx;
    float wv = gv * (float)N_ROWS - (float)cy;

    /* Window opening — rectangle [0.18, 0.82] in wu, with a Romanesque
     * half-circle cap at the top (wv < 0.18). Outside the opening is
     * wall stone. */
    bool in_window = false;
    if (wu > 0.18f && wu < 0.82f) {
        if (wv > 0.18f && wv < 0.82f) {
            in_window = true;                                /* body  */
        } else if (wv <= 0.18f) {
            float du = (wu - 0.50f) / 0.32f;
            float dv = (wv - 0.18f) / 0.18f;
            if (du * du + dv * dv < 1.0f) in_window = true;  /* arch  */
        }
    }
    return !in_window;
}

/* §5.5 TREE: tapered trunk + 6 angled branch capsules */

static bool sil_tree(float u, float v)
{
    if (v > 0.85f) return true;                              /* ground   */

    /* Trunk — tapered band, wider toward base. */
    float trunk_w = 0.05f - 0.02f * (-v);
    if (trunk_w < 0.025f) trunk_w = 0.025f;
    if (fabsf(u) < trunk_w && v > -0.45f) return true;

    /* Branches — each a thick line (capsule) rooted on the trunk. */
    static const struct {
        float root_v;     /* trunk position (v) where branch attaches  */
        float angle_deg;  /* direction (0 = up, +ve = right)           */
        float length;     /* branch length in normalised units         */
        float thickness;  /* base half-width (tapers to half this)     */
    } BR[] = {
        { -0.40f,  -55.0f, 0.55f, 0.040f },
        { -0.30f,   60.0f, 0.55f, 0.040f },
        { -0.20f,  -45.0f, 0.45f, 0.035f },
        { -0.10f,   50.0f, 0.45f, 0.035f },
        {  0.00f,  -65.0f, 0.40f, 0.030f },
        { -0.50f,    0.0f, 0.30f, 0.030f },          /* small upward stub */
    };
    const int NB = (int)(sizeof BR / sizeof BR[0]);

    for (int i = 0; i < NB; i++) {
        float rad = BR[i].angle_deg * (float)M_PI / 180.0f;
        float bx  = sinf(rad);                       /* branch unit vec */
        float by  = -cosf(rad);
        float du  = u - 0.0f;                        /* root at u = 0   */
        float dv  = v - BR[i].root_v;
        float t   = du * bx + dv * by;               /* axial position  */
        if (t < 0.0f || t > BR[i].length) continue;
        float pdu  = du - t * bx;                    /* perpendicular   */
        float pdv  = dv - t * by;
        float perp = sqrtf(pdu * pdu + pdv * pdv);
        float taper = 1.0f - 0.5f * (t / BR[i].length);
        if (perp < BR[i].thickness * taper) return true;
    }
    return false;
}

/* §5.6 dispatcher */

static bool silhouette_at(Pattern p, float u, float v)
{
    switch (p) {
    case PATTERN_ARCHWAY:  return sil_archway (u, v);
    case PATTERN_MOUNTAIN: return sil_mountain(u, v);
    case PATTERN_COLUMN:   return sil_column  (u, v);
    case PATTERN_WINDOWS:  return sil_windows (u, v);
    case PATTERN_TREE:     return sil_tree    (u, v);
    default:               return false;
    }
}

/* ── §6 noise + RNG ──────────────────────────────────────────────────── */

/* Permutation table — reseeded on 'r' or app start. */
static uint8_t perm[512];

static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

static inline float fade_q(float t) { return t*t*t*(t*(t*6.f-15.f)+10.f); }
static inline float lerp_f(float a, float b, float t) { return a + t*(b-a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.f*v : 2.f*v);
}

static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.f,  y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.f );
    float n11 = grad2(perm[B + 1], x - 1.f,  y - 1.f );
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

static float fbm2(float x, float y)
{
    float total = 0.f, amp = 1.f, freq = 1.f, max_amp = 0.f;
    for (int o = 0; o < 3; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;             /* → [0, 1]    */
}

/* hash3 — small mix for reseeding. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16; h *= 0x85ebca6bu;
    h ^= h >> 13; h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/* ── §7 scene + sun motion ───────────────────────────────────────────── */

typedef struct {
    bool      paused;
    int       speed;
    int       temp_preset;            /* index into TEMP_PRESETS[]      */
    Pattern   current_pattern;
    ShadeMode shade_mode;
    bool      bloom_on;
    bool      dust_on;
    float     time_secs;
    float     seed_phase;             /* phase offset on sun position   */
    int       seed;
} Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->speed           = SPEED_DEF;
    s->temp_preset     = 5;            /* DAY 5500K — neutral white sun */
    s->current_pattern = PATTERN_ARCHWAY;
    s->shade_mode      = SHADE_LIT;
    s->bloom_on        = true;
    s->dust_on         = true;
    s->seed_phase      = 0.7f;
    s->seed            = 0xDECAF;
    perm_shuffle(s->seed);
}

static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->seed_phase * 100.0f), 0xC0FFEE);
    s->seed_phase = ((float)(h & 0xFFFFu) / 65536.0f) * 2.f * (float)M_PI;
    s->seed       = (int)(h ^ 0x5A5A5A5Au);
    perm_shuffle(s->seed);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

static void scene_sun_pos(const Scene *s, int cols, int rows,
                          float *out_sx, float *out_sy)
{
    float omega = 2.0f * (float)M_PI / SUN_DRIFT_PERIOD_S;
    float ph    = s->time_secs * omega + s->seed_phase;
    float fx    = sinf(ph) * SUN_X_AMP_FRAC + 0.50f;
    float fy    = SUN_Y_FRAC + cosf(ph * 0.7f) * SUN_Y_AMP_FRAC;
    *out_sx = fx * (float)cols;
    *out_sy = fy * (float)rows;
}

/* ── §8 raymarch — THE CORE ──────────────────────────────────────────── */

/* §8.1 cell_to_uv — screen cell → aspect-corrected normalised coords. */

static inline void cell_to_uv(float sx, float sy, int cols, int rows,
                              float *out_u, float *out_v)
{
    *out_u = (2.0f * sx + 1.0f - (float)cols) / (float)cols;
    *out_v = (2.0f * sy + 1.0f - (float)rows) / (float)rows
           * (ASPECT_Y * (float)rows / (float)cols);
}

/* §8.2 march_visibility — weighted unblocked-sample sum. */

/*
 * Walk MARCH_STEPS small steps from cell toward sun, summing the
 * Beer-Lambert weight at every step where the silhouette function
 * says "lit". Return the FRACTION of weighted samples that were lit
 * (∈ [0, 1]).
 *
 * Closer samples weigh more — that's why shafts spread OUT from the
 * sun rather than filling the whole screen.
 */
static float march_visibility(Pattern pattern,
                              float cell_sx, float cell_sy,
                              float sun_sx,  float sun_sy,
                              int cols, int rows)
{
    float dsx = sun_sx - cell_sx;
    float dsy = sun_sy - cell_sy;
    float step_dx = dsx / (float)MARCH_STEPS;
    float step_dy = dsy / (float)MARCH_STEPS;
    float step_len = sqrtf(step_dx * step_dx
                         + (step_dy * ASPECT_Y) * (step_dy * ASPECT_Y));

    float accum = 0.0f, total = 0.0f;
    for (int i = 1; i <= MARCH_STEPS; i++) {
        float px = cell_sx + step_dx * (float)i;
        float py = cell_sy + step_dy * (float)i;
        float uu, vv;
        cell_to_uv(px, py, cols, rows, &uu, &vv);

        float d = step_len * (float)i;
        float w = expf(-FOG_SIGMA * d);
        if (!silhouette_at(pattern, uu, vv)) accum += w;
        total += w;
    }
    return (total > 0.f) ? (accum / total) : 0.f;
}

/* §8.3 sun_disc — cinematic sun: CORE + CORONA + LENS FLARE. */

/*
 * Three additive radiance components:
 *   CORE    sharp Gaussian — the bright dot
 *   CORONA  wider, dimmer Gaussian — the warm halo
 *   FLARE   N narrow angular spokes (lens-flare streaks)
 *
 * For each spoke at angle φ_s = s · π/N, measure the angular
 * distance from the cell direction to the spoke axis (wrapped to
 * [-π/2, π/2] because spokes are bidirectional), multiply a narrow
 * angular Gaussian with a wide radial Gaussian. Sum across spokes.
 *
 * Edge: skip flare when r < 0.5 — atan2(0, 0) is undefined, and at
 * that distance the core dominates anyway.
 */
static float sun_disc(float cell_sx, float cell_sy,
                      float sun_sx,  float sun_sy,
                      bool sun_in_silhouette)
{
    if (sun_in_silhouette) return 0.f;
    float dx = cell_sx - sun_sx;
    float dy = (cell_sy - sun_sy) * ASPECT_Y;
    float r2 = dx * dx + dy * dy;

    float core   = expf(-r2 / (SUN_CORE_FALLOFF * SUN_CORE_FALLOFF));
    float corona = expf(-r2 / (SUN_CORONA_FALLOFF * SUN_CORONA_FALLOFF))
                 * SUN_CORONA_GAIN;

    float flare = 0.f;
    float r = sqrtf(r2);
    if (r > 0.5f) {
        float ang = atan2f(dy, dx);
        float rad_falloff = expf(-r2 / (SUN_FLARE_R_FALLOFF * SUN_FLARE_R_FALLOFF));
        for (int s = 0; s < SUN_N_FLARES; s++) {
            float spoke = (float)s * (float)M_PI / (float)SUN_N_FLARES;
            float da = ang - spoke;
            while (da >  (float)M_PI * 0.5f) da -= (float)M_PI;
            while (da < -(float)M_PI * 0.5f) da += (float)M_PI;
            float ang_falloff = expf(-da*da
                                     / (SUN_FLARE_ANG_SIGMA * SUN_FLARE_ANG_SIGMA));
            flare += ang_falloff * rad_falloff;
        }
        flare *= SUN_FLARE_GAIN;
    }
    return core + corona + flare;
}

/* §8.4 fog_jitter — slow Perlin shimmer multiplier. */

static float fog_jitter(float u, float v, float wind)
{
    float n = fbm2(u * 1.4f + wind, v * 1.4f);
    return 1.0f + FOG_JITTER_AMP * (n - 0.5f) * 2.0f;
}

/* §8.5 shade_lit_rgb — continuous RGB from palette + visibility. */

/*
 *    base = lerp(palette.fog, palette.shaft, vis)    ← continuous blend
 *    sun  = palette.sun · sun_term                    ← cinematic disc
 *    col  = (base + sun) · jitter                      ← wind shimmer
 *
 * Result is HDR — values can exceed 1.0 inside the sun. Tone
 * mapping in paint_cell handles compression at draw time.
 */
static V3 shade_lit_rgb(const Palette *pal, float vis, float sun_term, float jitter)
{
    V3 base = v3_lerp(pal->fog, pal->shaft, clamp01(vis * SHAFT_GAIN));
    V3 sun  = v3_scl(pal->sun, sun_term * SUN_GAIN);
    return v3_scl(v3_add(base, sun), jitter);
}

/* ── §9 buffer + post-processing ─────────────────────────────────────── */

/* §9.1 buffer (V3 + visibility cache). */

static V3    g_buf  [BUF_MAX_H][BUF_MAX_W];      /* main colour buffer */
static V3    g_bloom[BUF_MAX_H][BUF_MAX_W];      /* bloom contribution */
static float g_vis  [BUF_MAX_H][BUF_MAX_W];      /* per-cell vis + sun_term
                                                  * — used by dust as a
                                                  * gating multiplier */

/* §9.2 dust particles (with motion-blur trail) ──────────────────────── */

/*
 * Each particle drifts at WIND-MODULATED velocity (slow Perlin gusts
 * make the speed ebb and flow). When age > life or off-screen, it
 * respawns at a random position.
 *
 * MOTION TRAIL — each particle records its last DUST_TRAIL_LEN
 * positions, sampled at fixed SPATIAL spacing (TRAIL_SPACING cells)
 * rather than every frame. Spatial spacing means the trail length
 * looks consistent across frame rates and across wind speeds —
 * a fast particle has the same screen-space trail length as a slow
 * one, just travelled in less time.
 *
 * VISIBILITY GATING — every brightness contribution is multiplied
 * by g_vis[gridcell] of the cell it paints into. Particles outside
 * shafts contribute nothing — exactly the "dust caught in sunbeam"
 * effect.
 */
typedef struct {
    float x, y;                                 /* current position    */
    float age;                                  /* sec since spawn     */
    float life;                                 /* total life          */
    float trail_dist_acc;                       /* dist since last push */
    float tx[DUST_TRAIL_LEN];                   /* trail positions     */
    float ty[DUST_TRAIL_LEN];                   /* [0] = newest        */
    int   trail_n;                              /* valid trail entries */
} Dust;

static Dust g_dust[DUST_MAX];
static int  g_dust_count = 0;

/* xorshift RNG for dust spawn positions. */
static uint32_t g_dust_rng = 0x12345678u;

static inline float dust_rand(void)
{
    g_dust_rng ^= g_dust_rng << 13;
    g_dust_rng ^= g_dust_rng >> 17;
    g_dust_rng ^= g_dust_rng << 5;
    return (float)(g_dust_rng & 0xFFFFFFu) / (float)0x1000000u;
}

static void dust_respawn(Dust *d, int cols, int rows)
{
    /* Spawn anywhere on screen with random life. */
    d->x    = dust_rand() * (float)cols;
    d->y    = dust_rand() * (float)rows;
    d->age  = 0.f;
    d->life = DUST_LIFE_MEAN + (dust_rand() * 2.f - 1.f) * DUST_LIFE_VAR;
    if (d->life < 1.f) d->life = 1.f;

    /* Reset trail. */
    d->trail_dist_acc = 0.f;
    d->trail_n        = 0;
    for (int j = 0; j < DUST_TRAIL_LEN; j++) {
        d->tx[j] = d->x;
        d->ty[j] = d->y;
    }
}

static void dust_init(int n, int cols, int rows, uint32_t seed)
{
    if (n > DUST_MAX) n = DUST_MAX;
    if (n < 0)        n = 0;
    g_dust_count = n;
    g_dust_rng = seed ? seed : 0xC0FFEE;
    for (int i = 0; i < n; i++) {
        dust_respawn(&g_dust[i], cols, rows);
        /* Stagger ages so particles don't all expire together. */
        g_dust[i].age = dust_rand() * g_dust[i].life;
    }
}

/* Wind-gust multiplier — slow Perlin field over time, clamped to keep
 * direction consistent (no negative speeds). */
static float wind_gust(float time_secs)
{
    float n = perlin2d(time_secs * WIND_GUST_FREQ_HZ, 17.0f);   /* ≈[-1,1] */
    float g = 1.f + WIND_GUST_AMP * n;
    if (g < 0.3f)              g = 0.3f;
    if (g > 1.0f + WIND_GUST_AMP) g = 1.0f + WIND_GUST_AMP;
    return g;
}

/*
 * dust_tick — advance every particle by one sim step.
 *
 *   1. Compute wind-gusted velocity for this frame.
 *   2. Push current position into trail IF the particle has moved
 *      more than DUST_TRAIL_SPACING since the last push.
 *   3. Advance position; advance age.
 *   4. Respawn if dead or off-screen.
 *
 * Spatial-distance trail (rather than per-frame) keeps the trail
 * visually consistent across frame rates.
 */
static void dust_tick(float dt, float time_secs, int cols, int rows)
{
    float gust = wind_gust(time_secs);

    for (int i = 0; i < g_dust_count; i++) {
        Dust *d = &g_dust[i];

        float dx = DUST_SPEED_X * gust * dt;
        float dy = DUST_SPEED_Y * gust * dt;
        float step = sqrtf(dx*dx + dy*dy);

        /* Trail recording — distance-based. */
        d->trail_dist_acc += step;
        if (d->trail_dist_acc >= DUST_TRAIL_SPACING) {
            d->trail_dist_acc = 0.f;
            for (int j = DUST_TRAIL_LEN - 1; j > 0; j--) {
                d->tx[j] = d->tx[j - 1];
                d->ty[j] = d->ty[j - 1];
            }
            d->tx[0] = d->x;
            d->ty[0] = d->y;
            if (d->trail_n < DUST_TRAIL_LEN) d->trail_n++;
        }

        /* Advance. */
        d->x   += dx;
        d->y   += dy;
        d->age += dt;

        if (d->age >= d->life
            || d->x < 0.f || d->x >= (float)cols
            || d->y < 0.f || d->y >= (float)rows) {
            dust_respawn(d, cols, rows);
        }
    }
}

/* dust_apply — paint head + trail into g_buf. */

static inline void dust_plot_at(int gx, int gy, float br, V3 dust_col,
                                int cols, int rows)
{
    if (gx < 0 || gx >= cols || gy < 0 || gy >= rows) return;
    if (gy >= BUF_MAX_H || gx >= BUF_MAX_W) return;
    float vis = g_vis[gy][gx];
    if (vis < 0.05f) return;
    g_buf[gy][gx] = v3_add(g_buf[gy][gx], v3_scl(dust_col, br * vis));
}

static void dust_apply(int cols, int rows, V3 dust_col)
{
    for (int i = 0; i < g_dust_count; i++) {
        const Dust *d = &g_dust[i];

        /* Half-sine envelope — peaks at age = life/2. */
        float t = d->age / d->life;
        if (t < 0.f) t = 0.f;
        if (t > 1.f) t = 1.f;
        float env = sinf(t * (float)M_PI);

        /* Head — full brightness. */
        dust_plot_at((int)d->x, (int)d->y,
                     env * DUST_BRIGHTNESS, dust_col, cols, rows);

        /* Trail — fading samples behind the head. */
        for (int j = 0; j < d->trail_n; j++) {
            dust_plot_at((int)d->tx[j], (int)d->ty[j],
                         env * DUST_BRIGHTNESS * DUST_TRAIL_BR[j],
                         dust_col, cols, rows);
        }
    }
}

/* §9.3 bloom — 5×5 bright-pass Gaussian blur ────────────────────────── */

/*
 * Two-pass:
 *   1. Pre-compute Gaussian weight kernel (built once, cached).
 *   2. Per cell: gather Gaussian-weighted contributions from the 5×5
 *      neighbourhood for any neighbour whose luma > threshold. Write
 *      to g_bloom (NOT g_buf). Centre-cell weight is 0 — strictly
 *      additive contribution.
 *   3. After the pass, fold g_bloom into g_buf.
 *
 * Single-pass would compound asymmetrically as later cells read
 * already-bloomed earlier cells. The separate buffer keeps the read
 * source consistent.
 */
static void bloom_apply(int cols, int rows)
{
    const int R = BLOOM_RADIUS;
    static float w[2*BLOOM_RADIUS + 1][2*BLOOM_RADIUS + 1];
    static int   weights_built = 0;
    if (!weights_built) {
        for (int dy = -R; dy <= R; dy++) {
            for (int dx = -R; dx <= R; dx++) {
                float d2 = (float)(dy*dy + dx*dx);
                w[dy+R][dx+R] = expf(-d2 / BLOOM_SIGMA2);
            }
        }
        w[R][R] = 0.f;          /* don't add self — strictly additive */
        weights_built = 1;
    }

    /* Pass 1: gather bright contributions into g_bloom. */
    for (int r = 0; r < rows && r < BUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < BUF_MAX_W; c++) {
            V3 sum = v3_make(0,0,0);
            for (int dy = -R; dy <= R; dy++) {
                int nr = r + dy;
                if (nr < 0 || nr >= rows || nr >= BUF_MAX_H) continue;
                for (int dx = -R; dx <= R; dx++) {
                    int nc = c + dx;
                    if (nc < 0 || nc >= cols || nc >= BUF_MAX_W) continue;
                    V3 bv = g_buf[nr][nc];
                    if (luma_of(bv) < BLOOM_LUMA_THRESHOLD) continue;
                    float weight = w[dy+R][dx+R];
                    sum = v3_add(sum, v3_scl(bv, weight));
                }
            }
            g_bloom[r][c] = v3_scl(sum, BLOOM_GAIN);
        }
    }

    /* Pass 2: fold bloom into g_buf. */
    for (int r = 0; r < rows && r < BUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < BUF_MAX_W; c++) {
            g_buf[r][c] = v3_add(g_buf[r][c], g_bloom[r][c]);
        }
    }
}

/* ── §10 screen — scene_draw + HUD ───────────────────────────────────── */

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

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * scene_draw — main pixel pipeline.
 *
 *   LIT mode (default):
 *     PASS 1 — march visibility + colour into g_buf
 *     PASS 2 — fold dust particles into g_buf
 *     PASS 3 — bloom into g_buf
 *     PASS 4 — paint g_buf to screen via paint_cell
 *
 *   MASK mode:
 *     skip the whole RGB pipeline; paint silhouette + flat fog.
 *
 *   VIS mode:
 *     compute visibility + sun_term; route directly to grayscale ramp.
 *     No bloom, no dust, no theme tint — pure algorithm output.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int rows_eff = sc->rows - 2;
    int row_off  = 1;
    if (rows_eff < 4) { rows_eff = sc->rows; row_off = 0; }

    if (rows_eff > BUF_MAX_H) rows_eff = BUF_MAX_H;
    int cols_eff = sc->cols;
    if (cols_eff > BUF_MAX_W) cols_eff = BUF_MAX_W;

    float sun_sx, sun_sy;
    scene_sun_pos(s, cols_eff, rows_eff, &sun_sx, &sun_sy);
    float su, sv;
    cell_to_uv(sun_sx, sun_sy, cols_eff, rows_eff, &su, &sv);
    bool sun_blocked = silhouette_at(s->current_pattern, su, sv);
    float wind = s->time_secs * FOG_WIND;

    /* === MASK mode short-circuit ===================================== */
    if (s->shade_mode == SHADE_MASK) {
        int sun_col = (int)(sun_sx + 0.5f);
        int sun_row = (int)(sun_sy + 0.5f) + row_off;

        for (int r = 0; r < rows_eff; r++) {
            for (int c = 0; c < cols_eff; c++) {
                int sy = r + row_off;
                float u, v;
                cell_to_uv((float)c + 0.5f, (float)r + 0.5f,
                           cols_eff, rows_eff, &u, &v);

                if (silhouette_at(s->current_pattern, u, v)) {
                    attron(COLOR_PAIR(PAIR_SILHOUETTE));
                    mvaddch(sy, c, ' ');
                    attroff(COLOR_PAIR(PAIR_SILHOUETTE));
                } else if (!sun_blocked && c == sun_col && sy == sun_row) {
                    attron(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                    mvaddch(sy, c, '*');
                    attroff(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                } else {
                    attron(COLOR_PAIR(PAIR_MASK_FOG));
                    mvaddch(sy, c, '.');
                    attroff(COLOR_PAIR(PAIR_MASK_FOG));
                }
            }
        }
        return;
    }

    /* === VIS mode short-circuit ====================================== */
    if (s->shade_mode == SHADE_VIS) {
        for (int r = 0; r < rows_eff; r++) {
            for (int c = 0; c < cols_eff; c++) {
                int sy = r + row_off;
                float u, v;
                cell_to_uv((float)c + 0.5f, (float)r + 0.5f,
                           cols_eff, rows_eff, &u, &v);

                if (silhouette_at(s->current_pattern, u, v)) {
                    attron(COLOR_PAIR(PAIR_SILHOUETTE));
                    mvaddch(sy, c, ' ');
                    attroff(COLOR_PAIR(PAIR_SILHOUETTE));
                    continue;
                }
                float vis = march_visibility(s->current_pattern,
                                             (float)c + 0.5f,
                                             (float)r + 0.5f,
                                             sun_sx, sun_sy,
                                             cols_eff, rows_eff);
                float sun_term = sun_disc((float)c + 0.5f, (float)r + 0.5f,
                                          sun_sx, sun_sy, sun_blocked);
                float intensity = vis * SHAFT_GAIN + sun_term * SUN_GAIN;
                paint_cell_vis(c, sy, intensity, sun_term);
            }
        }
        return;
    }

    /* === LIT mode — full RGB pipeline ================================ */
    /* Build palette ONCE per frame from current sun temperature. */
    Palette pal = palette_from_kelvin(TEMP_PRESETS[s->temp_preset].kelvin);
    V3 silhouette_rgb = v3_make(0.02f, 0.02f, 0.02f);   /* near-black */

    /* PASS 1 — march into g_buf, also stash visibility for dust gate. */
    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols_eff; c++) {
            float u, v;
            cell_to_uv((float)c + 0.5f, (float)r + 0.5f,
                       cols_eff, rows_eff, &u, &v);

            if (silhouette_at(s->current_pattern, u, v)) {
                g_buf[r][c] = silhouette_rgb;
                g_vis[r][c] = 0.f;
                continue;
            }
            float vis      = march_visibility(s->current_pattern,
                                              (float)c + 0.5f,
                                              (float)r + 0.5f,
                                              sun_sx, sun_sy,
                                              cols_eff, rows_eff);
            float sun_term = sun_disc((float)c + 0.5f, (float)r + 0.5f,
                                      sun_sx, sun_sy, sun_blocked);
            float jitter   = fog_jitter(u, v, wind);

            g_buf[r][c] = shade_lit_rgb(&pal, vis, sun_term, jitter);
            g_vis[r][c] = vis + sun_term;
        }
    }

    /* PASS 2 — dust overlay (gated by visibility). */
    if (s->dust_on) dust_apply(cols_eff, rows_eff, pal.dust);

    /* PASS 3 — bloom (5×5 bright-pass Gaussian). */
    if (s->bloom_on) bloom_apply(cols_eff, rows_eff);

    /* PASS 4 — paint to screen. */
    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols_eff; c++) {
            paint_cell(c, r + row_off, g_buf[r][c]);
        }
    }
}

/*
 * hud_draw — yellow status row 0 + cyan hint bottom row (HUD spec).
 */
static void hud_draw(const Screen *sc, const Scene *s,
                     double fps, int sim_fps)
{
    const TempPreset *tp = &TEMP_PRESETS[s->temp_preset];
    char buf[200];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  %s %5.0fK  mode:%s  bloom:%s  dust:%s  speed:%-2d ",
             fps, sim_fps,
             s->paused ? "PAUSED " : pattern_name(s->current_pattern),
             tp->name, (double)tp->kelvin,
             shade_mode_name(s->shade_mode),
             s->bloom_on ? "on " : "off",
             s->dust_on  ? "on " : "off",
             s->speed);
    int len = (int)strlen(buf);
    if (len > sc->cols) len = sc->cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, " GOD-RAYS · BLACKBODY ");
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  n/p:pat  t/T:temp  m:mode  b:bloom  d:dust  +/-:spd  []:Hz  r:reseed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);
    hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §11 app ─────────────────────────────────────────────────────────── */

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

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reseed(s);
        dust_init(DUST_MAX, app->screen.cols, app->screen.rows,
                  (uint32_t)s->seed);
        break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-': case '_':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->temp_preset = (s->temp_preset + 1) % N_TEMP_PRESETS;
        break;
    case 'T':
        s->temp_preset = (s->temp_preset + N_TEMP_PRESETS - 1) % N_TEMP_PRESETS;
        break;

    case 'n':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        break;
    case 'p':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        break;

    case 'm': case 'M':
        s->shade_mode = (ShadeMode)(((int)s->shade_mode + 1) % SHADE_N);
        break;

    case 'b': case 'B':
        s->bloom_on = !s->bloom_on;
        break;
    case 'd': case 'D':
        s->dust_on = !s->dust_on;
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
    scene_init (&app->scene);
    dust_init  (DUST_MAX, app->screen.cols, app->screen.rows,
                (uint32_t)app->scene.seed);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* §11.1 resize. */
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->need_resize = 0;
            dust_init(DUST_MAX, app->screen.cols, app->screen.rows,
                      (uint32_t)app->scene.seed);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* §11.2 timing. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* §11.3 fixed-step physics. */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            if (!app->scene.paused && app->scene.dust_on)
                dust_tick(dt_sec, app->scene.time_secs,
                          app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* §11.4 fps rolling average. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* §11.5 paint. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* §11.6 input. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;

        /* §11.7 frame cap (~60 fps render). */
        int64_t target_ns = NS_PER_SEC / 60;
        int64_t elapsed   = clock_ns() - now;
        clock_sleep_ns(target_ns - elapsed);
    }

    return 0;
}
