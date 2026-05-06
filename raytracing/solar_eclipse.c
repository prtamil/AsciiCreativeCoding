/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * solar_eclipse.c — total / partial / annular / transit eclipse renderer
 *
 *   Two-sphere ray-trace with a continuous-RGB pipeline, blackbody-
 *   coloured photosphere, granulation texture, multi-streamer corona,
 *   chromosphere ring, Bailey's beads at the totality boundary, and
 *   5×5 Gaussian bloom on bright cells.
 *
 * DEMO: A glowing sun fills ~40 cells across the screen — limb-darkened
 *       at its edge, brightest at centre, with FAINT GRANULATION
 *       texture from the convection cells of the photosphere. A dark
 *       moon drifts in from one side. As the moon slides over the sun:
 *
 *         - the bright photosphere shrinks to a crescent
 *         - just at totality threshold, BAILEY'S BEADS flash —
 *           multiple bright spots around the lunar limb where
 *           sunlight passes through valleys on the moon's edge
 *         - TOTALITY: the photosphere is gone. A soft RED CHROMOSPHERE
 *           ring hugs the moon's silhouette, surrounded by a
 *           STREAMER-STRUCTURED CORONA (polar plumes + helmet
 *           streamers) — not a uniform halo
 *         - more Bailey's beads as the moon exits
 *         - the sun fades back to crescent → partial → full
 *
 *       The cycle loops continuously over ~30 sec at default speed.
 *
 *       PATTERNS  (n / p):
 *         TOTAL    moon angular size > sun (1.10×). Full totality with
 *                  corona + chromosphere + Bailey's beads.
 *         PARTIAL  moon offset above so it never centres. Crescent only.
 *         ANNULAR  moon angular size < sun (0.80×). Bright RING of
 *                  photosphere remains visible; no corona (drowned out).
 *         TRANSIT  moon size << sun (0.12×). Tiny dot crosses the disc
 *                  — Mercury or Venus transit.
 *
 *       STAR PRESETS  (t / T):
 *         SUN-LIKE   5778K  the actual sun (default)
 *         RED-G      3500K  red-giant cool star
 *         ORANGE     4500K  warm orange star
 *         GIANT      8000K  blue giant
 *         WHITE-D   10000K  near-white blue dwarf
 *
 *       'r' reseeds (orbit phase + corona streamer pattern).
 *
 * Study alongside:
 *   raytracing/path_tracer.c        — canonical RGB → cube + tone-map
 *   raytracing/saturn_with_rings.c  — same RGB pipeline + bloom
 *   raytracing/god_rays_silhouette.c — same blackbody palette + bloom
 *
 * Section map:
 *   §1 config        — frame rate, geometry, granulation, corona, beads
 *   §2 clock         — monotonic timer + sleep
 *   §3 math+palette  — V3, RGB, blackbody, palette_from_kelvin
 *   §4 noise+RNG     — Perlin + fBm + hash3
 *   §5 ncurses paint — 6×6×6 cube + airy ramp + paint_cell
 *   §6 ray-sphere    — analytic intersection (used twice per pixel)
 *   §7 occlusion     — sun-moon angular overlap fraction
 *   §8 scene+orbit   — Scene state, pattern params, moon orbit
 *   §9 shading       — RGB shaders for sun, corona, chromosphere, beads
 *                      §9.1 shade_sun         (limb + granulation)
 *                      §9.2 shade_corona      (streamers via fBm)
 *                      §9.3 shade_chromosphere (Hα ring at totality)
 *                      §9.4 shade_beads       (multi-bead totality flash)
 *   §10 buffer+post  — V3 buffer + 5×5 Gaussian bloom
 *   §11 screen+draw  — scene_draw (3-pass) + HUD
 *   §12 app          — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume orbit
 *   r            reseed (orbit phase + streamers)
 *   n / p        next / previous pattern
 *   t / T        next / previous star preset
 *   + / =        faster orbit   (-: slower)
 *   ] / [        raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/solar_eclipse.c \
 *       -o solar_eclipse -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Per-pixel ray-tracing of TWO spheres (sun + moon) with
 *                  a continuous-RGB shader composing five additive layers:
 *                    PHOTOSPHERE  limb darkening · fBm granulation
 *                    CORONA       exp(-Δα·K) · fBm streamers · occlusion^γ
 *                    CHROMOSPHERE thin red ring at d_out ≈ 0 (totality)
 *                    BEADS        N Gaussian beads around lunar limb
 *                                 within DIAMOND_W of |moon_α − sun_α|
 *                    BLOOM        5×5 bright-pass Gaussian over the buffer
 *                  Output is tone-mapped (Reinhard), gamma-encoded, and
 *                  quantised to a 216-pair RGB cube + 16-char airy ramp.
 *                  Sun colour comes from one Kelvin via Planck's blackbody
 *                  law — the entire scene chromaticity follows.
 *
 * Data-structure : Per frame: V3 colour buffer g_buf[H][W] populated by
 *                  the per-pixel shader, then read-modified by the bloom
 *                  pass. Themes are STAR TEMPERATURES (not preset
 *                  palettes) — every hue derives from one Kelvin.
 *                  No malloc anywhere; all sizes are bounded at compile
 *                  time.
 *
 * Rendering      : Reinhard tone-map L'=L/(1+L) → gamma encode 1/2.2 →
 *                  quantise to xterm 6×6×6 cube → density char from a
 *                  curated 16-char open-glyph ramp. A_BOLD on bright
 *                  cells, A_DIM on dark. The ramp uses ONLY OPEN GLYPHS
 *                  (` .'`,-_:;~=+*oO0`) — no closed blocks — so bright
 *                  cells read as POINTS OF LIGHT not solid pixels.
 *
 * Performance    : 2 ray-sphere tests + 2-3 transcendentals + a 3-octave
 *                  fBm per cell, plus a 5×5 bloom pass. At 240×80 / 30 fps
 *                  ≈ 12 ms shading per frame on modern CPU, comfortable
 *                  inside the 33 ms budget.
 *
 * References     : Wikipedia: Solar eclipse, Corona, Baily's beads,
 *                    Solar granulation, Chromosphere.
 *                  Eddington, "The Internal Constitution of the Stars"
 *                    (1926) — limb-darkening derivation.
 *                  Tanner Helland, "How to convert temperature to RGB"
 *                    https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
 *                  Reinhard et al., "Photographic Tone Reproduction for
 *                    Digital Images," SIGGRAPH '02.
 *                  Espenak & Anderson, eclipse photographs (visual
 *                    reference for streamer structure and bead count).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two spheres in 3-space; whichever is closer along the view ray covers
 * the other. The moon is much CLOSER to the camera than the sun, so when
 * its angular footprint (atan(R/Z)) overlaps the sun's, it wins the
 * depth test and produces an occluding silhouette. The remarkable visual
 * — corona, chromosphere, beads — is what becomes visible when the
 * moon's angular size JUST exceeds the sun's: the bright photosphere
 * vanishes, and the much fainter outer atmosphere remains.
 *
 * Five additive shading layers make the eclipse cinematic:
 *
 *   COLOUR        Continuous RGB from blackbody temperature. One Kelvin
 *                 number governs the entire scene's chromaticity.
 *   GRANULATION   The photosphere has a faint mottled texture from
 *                 convection cells. fBm sampled in spherical surface
 *                 coords so the texture stays glued to the sun.
 *   STREAMERS     The corona isn't a uniform halo — it has polar
 *                 plumes and helmet streamers. fBm in (angle, radius)
 *                 space gives radial structure.
 *   BEADS         Bailey's beads — 5 bright spots around the lunar
 *                 limb at the totality threshold, where sunlight
 *                 passes through valleys on the moon's edge.
 *   BLOOM         Bright cells leak warm light into 5×5 neighbours.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The sun is a desk lamp behind a paper-thin halo of mist (corona).
 * Normally the bulb is too bright to see the mist. Slide a small piece
 * of card in front of the bulb — it blocks the bulb but the mist around
 * the silhouette becomes visible. That visibility ratio is the SUN
 * OCCLUSION FACTOR. The mist isn't smooth — there are streaky patches
 * (streamers). At the precise moment the card just barely covers the
 * bulb, light leaks around its edges in a few specific spots (beads).
 * The bulb itself isn't uniformly bright — it has a granular texture.
 *
 *      ┌───── corona streamers ─────────┐
 *     ╱  ╲╲                          ╱╱  ╲
 *    ╱  ╲ ╲╲   ●●●● bailey's beads ╱╱ ╱   ╲
 *   │   ╲  ╲╲╲╲╲▓▓▓▓▓▓▓▓▓▓╱╱╱╱╱   ╱   │
 *   │     ╲╲╲╲▓▓▓▓ moon ▓▓▓╱╱╱╱       │
 *   │       ╲▓▓ silhouette ▓▓╱        │
 *   │     ╱╱╱╱▓▓▓▓▓▓▓▓▓▓╲╲╲╲          │
 *   │   ╱╱ ╱   ●●●● beads   ╲╲ ╲      │
 *    ╲   ╱╱                  ╲╲╲ ╲   ╱
 *     ╲ ╱╱   chromosphere     ╲╲╲ ╲ ╱
 *      └── (thin red ring just outside the moon) ──┘
 *
 * Switching STAR PRESETS changes the photospheric blackbody temperature.
 * 3500K red-giant gives a dim red sun; 10000K white-dwarf gives a near-
 * white blue. The geometry stays identical — only chromaticity changes.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────────────────
 *  1. Build PALETTE from current star temperature K via blackbody:
 *       sun       = blackbody(K)
 *       corona    = cool plasma white (≈ neutral, slight blue)
 *       chromos   = warm hydrogen-red (Hα emission)
 *       bead      = mix of chromos + sun
 *       moon      = very dim cool blue (earthshine)
 *       sky       = near-black with subtle blue
 *  2. Compute moon position from orbit phase + sun-moon angular
 *     separation `sep` and totality boundary sep_T = |moon_α − sun_α|.
 *  3. PASS 1 — RAY-SHADE (fill g_buf):
 *       per cell:
 *         ray-sphere sun + moon, depth-sort
 *         if sun visible: shade_sun         (limb + granulation)
 *         if moon visible: earthshine tint  (not pure black)
 *         if space + corona on: shade_corona + shade_chromosphere
 *         shade_beads if at totality boundary
 *         additive into g_buf[r][c]
 *  4. PASS 2 — BLOOM (5×5 Gaussian bright-pass into g_buf).
 *  5. PASS 3 — PAINT (Reinhard → gamma → cube → density char).
 *
 * KEY FORMULAS
 * ────────────
 *  Apparent angular radii:
 *    sun_α  = atan(SUN_R  / SUN_Z)
 *    moon_α = atan(MOON_R / MOON_Z)
 *
 *  Sun-moon angular separation:
 *    sep    = acos(normalize(sun_pos) · normalize(moon_pos))
 *
 *  Sun occlusion fraction:
 *    if sep ≥ sun_α + moon_α : 0 (no overlap)
 *    if sep ≤ |sun_α − moon_α|: 1 (TOTAL) or (m/s)² (ANNULAR ring)
 *    else: linear interpolation between
 *
 *  Limb darkening (Eddington-style):
 *    μ      = max(0, N · (-view_dir))    ← view-cosine, 0 at limb
 *    L_limb = LIMB_AMBIENT + LIMB_GAIN · μ
 *
 *  Granulation (sphere-stable):
 *    surface_u = atan2(N.x, N.z) · GRAN_FREQ    ← longitude
 *    surface_v = N.y                · GRAN_FREQ ← latitude
 *    granul    = 1 + GRAN_AMP · (fbm(surface_u, surface_v) − 0.5) · 2
 *
 *  Corona base + streamers:
 *    α_view    = acos(ray_d · sun_dir)
 *    d_out     = max(0, α_view − sun_α)
 *    c_local   = exp(-d_out · CORONA_K)
 *    streamer  = (1 + AMP · (fbm(angle · FA, d · FR + drift) − 0.5)·2)^POW
 *    global    = occlusion^GAMMA
 *    corona_I  = c_local · streamer · global · BOOST
 *
 *  Chromosphere ring (red Hα band just outside the photosphere):
 *    in_band = (0 ≤ d_out ≤ CHROMO_W)
 *    profile = sin(π · d_out / CHROMO_W)            (0..1..0)
 *    ramp    = clamp((occ − THRESHOLD) / (1 − THRESHOLD))
 *    chromos = INTENSITY · profile · ramp · chromos_col
 *
 *  Bailey's beads (N beads at angles around limb):
 *    sep_T    = |moon_α − sun_α|
 *    window   = exp(-((sep − sep_T) / DIAMOND_W)²)
 *    for each bead k:
 *      φ_k     = anti_moon_angle + jitter[k]
 *      bead_w  = exp(-distance² / RADIUS²)
 *      contrib = window · bead_w · brightness[k] · bead_col
 *
 *  Reinhard + gamma + 6×6×6 cube paint: as in path_tracer.c §8.3.
 *
 *  Blackbody (Tanner Helland piecewise — see god_rays §3.5).
 *
 * WORKED EXAMPLE  (verify by hand, SUN-LIKE 5778K, mid-totality)
 * ─────────────────────────────────────────────────────────────
 *   5778 K blackbody:
 *     K = 57.78
 *     R = 1.000
 *     G = (99.47·ln 57.78 − 161.12)/255 ≈ (404.0 − 161.1)/255 ≈ 0.953
 *     B = (138.52·ln 47.78 − 305.04)/255 ≈ (537.7 − 305.0)/255 ≈ 0.913
 *   palette.sun     = (1.000, 0.953, 0.913)        ← warm white
 *   palette.corona  = (0.85, 0.92, 1.00)            ← cool plasma
 *   palette.chromos = (1.00, 0.32, 0.18)            ← Hα red
 *
 *   At mid-totality (occlusion=1.0, sep=0):
 *     Space cell at d_out = 0.005 rad (just outside the moon limb):
 *       c_local  = exp(-0.005 · 16) = exp(-0.08) ≈ 0.923
 *       streamer = (1 + 0.55 · 0.3)² ≈ 1.36         ← noise sample
 *       global   = 1.0^4 = 1.0
 *       corona_I = 0.923 · 1.36 · 1.0 · 1.40 ≈ 1.756
 *     col = corona_I · corona_col ≈ (1.49, 1.62, 1.76)   ← HDR
 *
 *     Reinhard:    (0.598, 0.618, 0.638)
 *     Gamma 2.2:   (0.792, 0.807, 0.821)
 *     Cube quant:  r5=4, g5=4, b5=4 → cool-near-white
 *     Luma 0.81 → ramp idx 12 → '*' glyph (with A_NORMAL — just under
 *                                          A_BOLD threshold of 0.85)
 *
 *   At Bailey's-bead moment (sep ≈ sep_T):
 *     window  = exp(0) = 1.0
 *     bead_w  = 1.0 at bead centre
 *     contrib = 1.0 · 1.0 · 0.85 · (1.00, 0.65, 0.45) = (0.85, 0.55, 0.38)
 *     Reinhard + gamma → bright orange-red pixel at the limb angle.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DEPTH ORDER. The moon MUST be at smaller z than the sun, otherwise
 *    the depth-sort never lets the moon win. MOON_Z=5, SUN_Z=50.
 *  • OCCLUSION CASES. ANNULAR (moon_α < sun_α) caps occlusion at
 *    (m/s)² < 1, so corona never reaches full brightness — exactly
 *    right (real annular eclipses don't show corona).
 *  • CORONA CLIPPED INSIDE MOON. The corona is computed only for cells
 *    where neither sphere is hit. Cells inside the moon silhouette
 *    skip corona — corona is the OUTER atmosphere.
 *  • GRANULATION COORDS. Sample fBm in SURFACE coords (atan2(N.x,N.z),
 *    N.y) NOT screen coords, or the texture appears to "slide" as the
 *    sun's apparent position changes.
 *  • STREAMER ROTATION. Adding `time · drift` to the radial fBm coord
 *    makes streamers slowly evolve. DON'T let the angle coord drift —
 *    streamers don't rotate around the sun in real life (their
 *    structure is tied to the sun's magnetic field, evolving on
 *    months-to-years timescale, not seconds).
 *  • CHROMOSPHERE GATE. The chromosphere ring is invisible whenever
 *    the photosphere is visible — drowned out. Gate by `occ > 0.92`
 *    so it only appears in the last seconds before totality and the
 *    first after.
 *  • BEADS GATE. Beads only fire when moon_α > sun_α (TOTAL pattern).
 *    For ANNULAR / PARTIAL / TRANSIT, the totality boundary is never
 *    crossed, so beads never light up. Gating is by
 *    `s->pp.totality_possible AND moon_α > sun_α`.
 *  • TONE-MAP BEFORE QUANTISE. Sums of corona + chromos + beads + bloom
 *    can exceed 1.0 by 2-3×. Tone-mapping before cube quantisation
 *    spreads colours across the cube; quantising HDR puts them all
 *    into one cell.
 *  • MOON EARTHSHINE. The moon during totality isn't pitch black —
 *    earthshine illuminates it dimly. We tint the silhouette
 *    (0.04, 0.05, 0.08) cool blue rather than zero.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space) at totality. Look for:
 *      - The corona has visible STREAKY structure (polar plumes,
 *        helmet streamers) — not a uniform fuzzy halo.
 *      - A thin RED RING just outside the moon (chromosphere).
 *      - The moon silhouette has a faint cool-blue tint
 *        (earthshine) — not solid black.
 *  • At first/last contact (entering / leaving totality):
 *      - Multiple BAILEY'S BEADS appear at angular positions around
 *        the lunar limb, not just a single bright dot at the
 *        sun's far edge.
 *  • Rotate STAR PRESET (t/T):
 *      - SUN-LIKE 5778K: warm-white sun, cool-blue corona.
 *      - RED-G 3500K: deep-red sun, more crimson everything.
 *      - WHITE-D 10000K: near-white sun, cooler corona.
 *  • Look at the SUN (not eclipsed): the disc has a faint mottled
 *    texture — convection cells.
 *  • PARTIAL pattern: crescent forms but no totality — no corona,
 *    no beads, no chromosphere.
 *  • ANNULAR pattern: bright RING of sun visible at maximum, no
 *    corona — the bright ring outshines it. No beads.
 *  • TRANSIT pattern: tiny dot crawls across; sun barely changes.
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
#define FOV_H           0.20f         /* tan of half horizontal FOV        */

/* §1.3 eclipse geometry — world units.
 *
 * Sun is far (z=50, R=5); moon is close (z=5, R varies by pattern).
 * The moon's WORLD radius is small but its angular size (atan R/Z) is
 * comparable to the sun's because it's much closer. That's the geometric
 * trick that makes the moon able to occlude a vastly larger sun. */
#define SUN_Z              50.0f
#define MOON_Z              5.0f
#define SUN_R               5.0f
#define MOON_BASE_R_TOTAL   0.55f     /* angular = 1.10 × sun (TOTAL)     */
#define MOON_BASE_R_ANNULAR 0.40f     /* angular = 0.80 × sun (ANNULAR)   */
#define MOON_BASE_R_TRANSIT 0.06f     /* angular = 0.12 × sun (TRANSIT)   */

#define ECLIPSE_PERIOD_S    30.0f
#define MOON_ORBIT_X_FRAC   1.5f      /* orbit amplitude fraction          */
#define PARTIAL_Y_OFFSET    0.025f    /* radians offset for PARTIAL        */

/* §1.4 photosphere — limb darkening + granulation.
 *
 *   limb darkening  Eddington: L = AMBIENT + GAIN · μ where μ=N·V.
 *                    Real solar limbs darken to ~30% of centre brightness
 *                    at the visible spectrum; AMBIENT 0.40 + GAIN 0.60
 *                    gives 0.40..1.00 across the disc, close enough.
 *   granulation     fBm in surface coordinates. Sphere-stable: the
 *                    texture moves with the sun, doesn't slide.
 */
#define LIMB_AMBIENT        0.40f
#define LIMB_GAIN           0.60f
#define GRAN_FREQ           14.0f     /* fBm sample frequency              */
#define GRAN_AMP            0.18f     /* depth of the mottle               */

/* §1.5 corona — radial decay × streamer pattern × occlusion gate.
 *
 *   c_local   = exp(-Δα · CORONA_K)               radial Gaussian decay
 *   streamer  = (1 + AMP · fbm(...))^POW          radial-ray structure
 *   global    = occlusion^GAMMA                   "fades in at totality"
 *   total     = c_local · streamer · global · BOOST
 *
 * STREAMER fBm samples (polar_angle, d_out + time·drift). The angle
 * coord creates RADIAL streaks (each fBm "ridge" along the angle axis
 * makes one streamer). The radial coord adds variation along each
 * streak and makes them slowly evolve.
 */
#define CORONA_K            16.0f
#define CORONA_GAMMA         4.0f
#define CORONA_BOOST         1.40f
#define STREAM_FREQ_A        2.5f     /* fBm frequency in angular axis     */
#define STREAM_FREQ_R        4.0f     /* fBm frequency in radial axis      */
#define STREAM_AMP           0.55f    /* streamer amplitude                */
#define STREAM_POW           1.8f     /* contrast exponent                 */
#define STREAM_DRIFT_HZ      0.04f    /* slow streamer evolution rate      */

/* §1.5b chromosphere — thin Hα-emission ring just outside photosphere.
 * Visible only near total occlusion (otherwise drowned by photosphere). */
#define CHROMO_W              0.0040f /* angular ring half-width (rad)     */
#define CHROMO_OCC_THRESHOLD  0.92f   /* occ above which chromos fades in  */
#define CHROMO_INTENSITY      0.85f

/* §1.6 Bailey's beads — multi-bead totality flash.
 *
 * Real Bailey's beads are 3-7 distinct spots of light at the lunar limb
 * where sunlight passes through valleys between mountains. We fake them
 * with N beads at fixed (per-seed) angular positions around the sun's
 * limb, each with its own brightness multiplier. They only fire within
 * DIAMOND_W of the totality boundary.
 */
#define DIAMOND_W            0.0050f  /* angular window around sep_T       */
#define BEAD_COUNT           5
#define BEAD_RADIUS_CELLS    1.4f     /* bead size in screen cells         */
#define BEAD_BRIGHTNESS      1.6f     /* peak gain                         */

/* §1.7 bloom (5×5 bright-pass Gaussian) */
#define BLOOM_RADIUS         2
#define BLOOM_GAIN           0.50f
#define BLOOM_LUMA_THRESHOLD 0.55f
#define BLOOM_SIGMA2         3.0f

/* §1.8 buffer dimensions (static, no malloc) */
#define BUF_MAX_W           400
#define BUF_MAX_H           200

/* §1.9 stars — sparse points of light visible during totality only. */
#define STAR_DENSITY        300
#define STAR_OCC_VISIBLE    0.85f

/* §1.10 ncurses pair IDs */
enum {
    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_SUN_FALLBACK =  3,    /* used in 8-colour fallback only          */
    PAIR_CUBE_BASE    = 16,    /* +0..+215 = 6×6×6 RGB cube              */
};

/* §1.11 ASCII density ramp — airy atmospheric, no closed blocks.
 *
 * 16 chars from empty to luminous. The brightest end is OPEN circles
 * (`o`, `O`, `0`) — they read as POINTS OF LIGHT, not solid blocks.
 * `#` and `@` are deliberately excluded. */
static const char k_ramp[] = " .'`,-_:;~=+*oO0";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.12 patterns */
typedef enum {
    PATTERN_TOTAL    = 0,
    PATTERN_PARTIAL  = 1,
    PATTERN_ANNULAR  = 2,
    PATTERN_TRANSIT  = 3,
    N_PATTERNS       = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_TOTAL:   return "TOTAL  ";
    case PATTERN_PARTIAL: return "PARTIAL";
    case PATTERN_ANNULAR: return "ANNULAR";
    case PATTERN_TRANSIT: return "TRANSIT";
    default:              return "?      ";
    }
}

/* §1.13 star presets — different blackbody temperatures.
 *
 * Each preset produces a different photospheric chromaticity. The
 * eclipse geometry stays the same; only the colour palette changes.
 * Cycling through these is "what if our sun were a different star."
 */
typedef struct {
    const char *name;
    float       kelvin;
} StarPreset;

static const StarPreset STAR_PRESETS[] = {
    { "SUN-LIKE", 5778.0f },   /* the actual sun (default)                 */
    { "RED-G   ", 3500.0f },   /* red giant — cool                         */
    { "ORANGE  ", 4500.0f },   /* warm orange star                         */
    { "GIANT   ", 8000.0f },   /* hot blue giant                           */
    { "WHITE-D ", 10000.0f },  /* near-white blue dwarf                    */
};
#define N_STAR_PRESETS ((int)(sizeof STAR_PRESETS / sizeof STAR_PRESETS[0]))

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

typedef struct { float x, y, z; } V3;
typedef struct { float r, g, b; } RGB;

static inline V3    v3 (float x, float y, float z) { return (V3){x,y,z}; }
static inline V3    v3_add(V3 a, V3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3_sub(V3 a, V3 b)  { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3_scl(V3 a, float s){ return v3(a.x*s, a.y*s, a.z*s);     }
static inline float v3_dot(V3 a, V3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z;  }
static inline V3    v3_norm(V3 a)
{
    float l = sqrtf(v3_dot(a, a));
    if (l < 1e-12f) return v3(0, 0, 0);
    return v3_scl(a, 1.0f / l);
}

static inline RGB rgb_make(float r, float g, float b) { return (RGB){r,g,b}; }
static inline RGB rgb_add (RGB a, RGB b)   { return rgb_make(a.r+b.r, a.g+b.g, a.b+b.b); }
static inline RGB rgb_scl (RGB a, float s) { return rgb_make(a.r*s, a.g*s, a.b*s);       }

static inline float clamp01 (float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float reinhard (float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }
static inline float luma_of  (RGB c)
{ return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; }

/* §3.5 Palette + blackbody colour ──────────────────────────────────── */

/*
 * Palette — the entire scene's colours, derived from one Kelvin value:
 *   sun       photosphere base = blackbody(K)
 *   corona    cool plasma white (~1MK plasma reads as neutral-blue)
 *   chromos   hydrogen-emission red (Hα fixed wavelength, K-independent)
 *   bead      Bailey's bead = mix of chromos red + photosphere
 *   moon      earthshine — extremely dim cool blue
 *   sky       background space — near-black with subtle blue
 */
typedef struct {
    RGB sun, corona, chromos, bead, moon, sky;
} Palette;

/*
 * sun_color_from_kelvin — Tanner Helland's piecewise blackbody approx.
 *
 *   T ≤ 6600 K  red dominates, green follows log curve, blue grows
 *   T  > 6600 K  red and green decay, blue saturates at 1
 *
 * Returns RGB ∈ [0, 1] approximating the chromaticity of a blackbody.
 */
static RGB sun_color_from_kelvin(float kelvin)
{
    float K = kelvin / 100.0f;
    float r, g, b;

    if (K <= 66.0f) r = 1.0f;
    else            r = 329.7f * powf(K - 60.0f, -0.1332f) / 255.0f;

    if (K <= 66.0f) g = (99.47f  * logf(K)            - 161.12f) / 255.0f;
    else            g =  288.12f * powf(K - 60.0f, -0.0755f)      / 255.0f;

    if (K >= 66.0f)      b = 1.0f;
    else if (K <= 19.0f) b = 0.0f;
    else                 b = (138.52f * logf(K - 10.0f) - 305.04f) / 255.0f;

    return rgb_make(clamp01(r), clamp01(g), clamp01(b));
}

/*
 * palette_from_kelvin — derive every eclipse colour from K.
 *
 * Physical reasoning: corona and chromosphere colours are LARGELY
 * independent of K because their light comes from very different
 * physical processes (corona = 1MK plasma emission, chromos = Hα at
 * 656.3 nm). The photosphere IS the K-dependent blackbody. So:
 *   - sun     = blackbody(K)              ← varies with preset
 *   - corona  = neutral cool plasma        ← fixed
 *   - chromos = saturated red (Hα)         ← fixed
 *   - bead    = mix of chromos + sun       ← partly varies
 *   - moon    = very dim cool blue         ← fixed (earthshine)
 *   - sky     = near-black                 ← fixed
 */
static Palette palette_from_kelvin(float kelvin)
{
    RGB sun     = sun_color_from_kelvin(kelvin);
    RGB corona  = rgb_make(0.85f, 0.92f, 1.00f);
    RGB chromos = rgb_make(1.00f, 0.32f, 0.18f);
    /* Bead colour: 70% chromos + 30% photosphere. */
    RGB bead    = rgb_make(0.70f * chromos.r + 0.30f * sun.r,
                           0.70f * chromos.g + 0.30f * sun.g,
                           0.70f * chromos.b + 0.30f * sun.b);
    RGB moon    = rgb_make(0.04f, 0.05f, 0.08f);
    RGB sky     = rgb_make(0.02f, 0.02f, 0.04f);

    Palette p = { sun, corona, chromos, bead, moon, sky };
    return p;
}

/* ── §4 noise + RNG ──────────────────────────────────────────────────── */

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

static inline float hash01(uint32_t h)
{
    return (float)(h & 0xFFFFFFu) * (1.f / (float)0x1000000u);
}

/* ── §5 ncurses paint ────────────────────────────────────────────────── */

static int g_256;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);

    if (g_256) {
        for (int i = 0; i < 216; i++)
            init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_SUN_FALLBACK, COLOR_YELLOW, -1);
        init_pair(PAIR_HUD,          COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,         COLOR_CYAN,   -1);
    }
}

/*
 * paint_cell — full RGB → terminal pipeline.
 *
 *   1. Reinhard tone-map per channel.
 *   2. Gamma encode 1/2.2.
 *   3. Quantise to 6×6×6 cube → pair id.
 *   4. Compute Rec.601 luma → density glyph.
 *   5. A_BOLD on bright cells, A_DIM on dark.
 *
 * Steps 1-2 must run BEFORE step 3 — quantising linear HDR puts every
 * pixel into one cube cell. Tone-mapping opens up the dynamic range.
 */
static void paint_cell(int sx, int sy, RGB col)
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
        attron(COLOR_PAIR(PAIR_SUN_FALLBACK));
        mvaddch(sy, sx, (chtype)(unsigned char)k_ramp[ri]);
        attroff(COLOR_PAIR(PAIR_SUN_FALLBACK));
    }
}

/* ── §6 ray-sphere ───────────────────────────────────────────────────── */

/*
 * ray_sphere — analytic ray vs sphere intersection (canonical form).
 *
 *   |ro + t·rd − c|² = r²
 *   → (oc·rd)·t·t + 2(oc·rd)·t + (oc·oc − r²) = 0     where oc = ro − c
 *   For unit |rd|, the 't²' coefficient is 1, simplifying to
 *   t = -b ± √(b² − c) where b = oc·rd, c = oc·oc − r².
 *
 * Returns true on hit. out_t = nearest t > epsilon.
 */
static bool ray_sphere(V3 ro, V3 rd, V3 center, float r, float *out_t)
{
    V3    oc   = v3_sub(ro, center);
    float b    = v3_dot(oc, rd);
    float c    = v3_dot(oc, oc) - r * r;
    float disc = b * b - c;
    if (disc < 0) return false;
    float sq = sqrtf(disc);
    float t  = -b - sq;
    if (t < 1e-3f) t = -b + sq;
    if (t < 1e-3f) return false;
    *out_t = t;
    return true;
}

/* ── §7 occlusion ────────────────────────────────────────────────────── */

/*
 * occlusion_factor — fraction of sun's apparent area covered by the
 * moon at angular separation `sep`.
 *
 *   sep ≥ sun_α + moon_α  → 0 (no overlap)
 *   sep ≤ |sun_α − moon_α|  → fully nested:
 *       moon_α ≥ sun_α (TOTAL):    1
 *       moon_α < sun_α (ANNULAR):  (m/s)²  ← max occlusion of an annular
 *   else: linear interpolation between the bounds
 *
 * The ANNULAR case caps below 1 because the moon is angularly smaller
 * than the sun; even at maximum nesting the bright "ring of sun"
 * remains visible, so the corona never reaches full visibility.
 */
static float occlusion_factor(float sep, float sun_a, float moon_a)
{
    float sum = sun_a + moon_a;
    float dif = fabsf(sun_a - moon_a);
    if (sep >= sum) return 0.0f;
    if (sep <= dif) {
        if (moon_a >= sun_a) return 1.0f;
        return (moon_a / sun_a) * (moon_a / sun_a);
    }
    float t = (sum - sep) / (2.0f * fminf(sun_a, moon_a));
    if (moon_a >= sun_a) return t;
    return t * (moon_a / sun_a) * (moon_a / sun_a);
}

/* ── §8 scene + orbit ────────────────────────────────────────────────── */

typedef struct {
    float moon_world_r;        /* world radius (sets moon_α)             */
    float moon_y_world;        /* y offset (PARTIAL only)                */
    bool  totality_possible;   /* TOTAL only — gates beads + chromos     */
    bool  annular_mode;        /* ANNULAR — boost limb for visible ring  */
    bool  show_corona;         /* TOTAL + PARTIAL                        */
} PatternParams;

typedef struct {
    bool          paused;
    int           speed;
    int           star_preset;
    Pattern       current_pattern;
    PatternParams pp;
    float         time_secs;
    float         seed_phase;
    int           seed;
} Scene;

static void pattern_set(Scene *s, Pattern p)
{
    s->current_pattern = p;
    PatternParams *pp = &s->pp;
    pp->moon_world_r      = MOON_BASE_R_TOTAL;
    pp->moon_y_world      = 0.0f;
    pp->totality_possible = false;
    pp->annular_mode      = false;
    pp->show_corona       = false;

    switch (p) {
    case PATTERN_TOTAL:
        pp->moon_world_r      = MOON_BASE_R_TOTAL;
        pp->totality_possible = true;
        pp->show_corona       = true;
        break;
    case PATTERN_PARTIAL:
        pp->moon_world_r      = MOON_BASE_R_TOTAL;
        pp->moon_y_world      = PARTIAL_Y_OFFSET * MOON_Z;
        pp->show_corona       = true;
        break;
    case PATTERN_ANNULAR:
        pp->moon_world_r      = MOON_BASE_R_ANNULAR;
        pp->annular_mode      = true;
        break;
    case PATTERN_TRANSIT:
        pp->moon_world_r      = MOON_BASE_R_TRANSIT;
        break;
    case N_PATTERNS: break;
    }
}

static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->seed_phase * 100.0f), 0xC0FFEE);
    s->seed_phase = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->seed       = (int)(h ^ 0x5A5A5A5Au);
    perm_shuffle(s->seed);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused      = false;
    s->speed       = SPEED_DEF;
    s->star_preset = 0;            /* SUN-LIKE 5778K                    */
    s->seed_phase  = 0.0f;
    s->seed        = 0xDECAF;
    perm_shuffle(s->seed);
    pattern_set(s, PATTERN_TOTAL);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* Compute moon world position at the current frame.
 * Moon orbits on a sin-curve of amplitude orbit_x_world, with optional
 * y offset for PARTIAL pattern. */
static V3 scene_moon_pos(const Scene *s)
{
    float sun_a  = atanf(SUN_R              / SUN_Z );
    float moon_a = atanf(s->pp.moon_world_r / MOON_Z);
    float orbit_x_world = MOON_Z * (sun_a + moon_a) * MOON_ORBIT_X_FRAC;

    float omega = 2.0f * (float)M_PI / ECLIPSE_PERIOD_S;
    float phase = omega * s->time_secs + s->seed_phase;
    float mx = orbit_x_world * sinf(phase);
    return v3(mx, s->pp.moon_y_world, MOON_Z);
}

/* Phase string for HUD — derived from current angular separation. */
static const char *phase_str_for(const Scene *s)
{
    V3 sun_pos  = v3(0, 0, SUN_Z);
    V3 moon_pos = scene_moon_pos(s);
    V3 sun_dir  = v3_norm(sun_pos);
    V3 moon_dir = v3_norm(moon_pos);
    float cos_b = v3_dot(sun_dir, moon_dir);
    if (cos_b > 1.f)  cos_b = 1.f;
    if (cos_b < -1.f) cos_b = -1.f;
    float sep = acosf(cos_b);

    float sun_a  = atanf(SUN_R               / SUN_Z );
    float moon_a = atanf(s->pp.moon_world_r  / MOON_Z);
    float sum    = sun_a + moon_a;
    float dif    = fabsf(sun_a - moon_a);

    if (sep >= sum) return "PRE/POST  ";
    if (sep <= dif) {
        if (s->pp.annular_mode)  return "ANNULARITY";
        if (moon_a >= sun_a)     return "TOTALITY  ";
        return "MAX-OCC   ";
    }
    float omega = 2.0f * (float)M_PI / ECLIPSE_PERIOD_S;
    float phase = omega * s->time_secs + s->seed_phase;
    float vx    = cosf(phase);
    float mx    = sinf(phase);
    bool  departing = (vx * mx) > 0;
    return departing ? "DEPART    " : "APPROACH  ";
}

/* ── §9 shading ──────────────────────────────────────────────────────── */

/* §9.1 shade_sun — limb darkening + granulation ─────────────────────── */

/*
 * Photospheric radiance at a sphere hit point.
 *
 *   N      = (hit − sun_centre) / SUN_R
 *   μ      = N · (-view_dir)               ← cosine of view angle
 *   limb   = LIMB_AMBIENT + LIMB_GAIN · μ  ← Eddington-style darkening
 *
 *   gran   = 1 + GRAN_AMP · (fbm_octaves − 0.5) · 2
 *
 * Granulation is sampled in SURFACE coordinates so the texture stays
 * glued to the sun:
 *   surface_u = atan2(N.x, N.z) · GRAN_FREQ      ← longitude
 *   surface_v = N.y                · GRAN_FREQ   ← latitude
 *
 * Annular boost: in ANNULAR mode at the limb (μ < 0.4), boost slightly
 * so the visible "ring of sun" reads as a bright ring.
 */
static RGB shade_sun(V3 N, V3 view_dir, RGB sun_col, bool annular_boost)
{
    float mu = -v3_dot(N, view_dir);
    if (mu < 0.f) mu = 0.f;
    float limb = LIMB_AMBIENT + LIMB_GAIN * mu;

    float u = atan2f(N.x, N.z) * GRAN_FREQ;
    float v = N.y * GRAN_FREQ;
    float gran = 1.0f + GRAN_AMP * (fbm2(u, v) - 0.5f) * 2.0f;

    float intensity = limb * gran;
    if (annular_boost && mu < 0.4f) intensity *= 1.20f;

    return rgb_scl(sun_col, intensity);
}

/* §9.2 shade_corona — radial decay × streamers × occlusion ──────────── */

/*
 * Corona radiance at angular distance d_out outside the sun's limb.
 *
 *   c_local   = exp(-d_out · CORONA_K)            radial Gaussian
 *   streamer  = (1 + AMP · (fbm − 0.5) · 2)^POW   radial-ray structure
 *   global    = occlusion^GAMMA                   visibility gate
 *   total     = c_local · streamer · global · BOOST
 *
 * The fBm sampled in (polar_angle, d_out) space produces visible polar
 * plumes and helmet streamers. The radial coord is offset by `time ·
 * STREAM_DRIFT_HZ` so streamers slowly evolve. The angle coord is NOT
 * time-driven — real streamers don't rotate around the sun.
 */
static RGB shade_corona(float d_out, float polar_angle, float occlusion,
                        float time_secs, RGB corona_col)
{
    if (occlusion <= 0.f) return rgb_make(0,0,0);

    float c_local = expf(-d_out * CORONA_K);

    float drift = time_secs * STREAM_DRIFT_HZ;
    float n = fbm2(polar_angle * STREAM_FREQ_A,
                   d_out * STREAM_FREQ_R + drift);
    float streamer = powf(1.0f + STREAM_AMP * (n - 0.5f) * 2.0f, STREAM_POW);
    if (streamer < 0.f) streamer = 0.f;

    float global = powf(occlusion, CORONA_GAMMA);

    float intensity = c_local * streamer * global * CORONA_BOOST;
    return rgb_scl(corona_col, intensity);
}

/* §9.3 shade_chromosphere — thin Hα ring at totality boundary ───────── */

/*
 * The chromosphere is invisible outside totality (drowned by photo-
 * sphere). We gate by `occlusion >= CHROMO_OCC_THRESHOLD` so the
 * red ring fades in only in the last seconds before totality.
 *
 *   profile = sin(π · d_out / W)        ← 0 at edges, 1 at centre
 *   ramp    = (occ − THRESHOLD) / (1 − THRESHOLD)   ← clamped 0..1
 *   col     = INTENSITY · profile · ramp · chromos_col
 */
static RGB shade_chromosphere(float d_out, float occlusion, RGB chromos_col)
{
    if (occlusion < CHROMO_OCC_THRESHOLD) return rgb_make(0,0,0);
    if (d_out < 0.f || d_out > CHROMO_W)  return rgb_make(0,0,0);

    float band = sinf((d_out / CHROMO_W) * (float)M_PI);
    float occ_ramp = (occlusion - CHROMO_OCC_THRESHOLD)
                    / (1.0f - CHROMO_OCC_THRESHOLD);
    if (occ_ramp < 0.f) occ_ramp = 0.f;
    if (occ_ramp > 1.f) occ_ramp = 1.f;

    return rgb_scl(chromos_col, CHROMO_INTENSITY * band * occ_ramp);
}

/* §9.4 shade_beads — multi-bead totality flash ──────────────────────── */

/*
 * Bailey's beads — multiple bright spots around the lunar limb at the
 * totality threshold. Real eclipses show 3-7 distinct beads; we use 5.
 *
 * Window: Gaussian centred on |sep − sep_T| = 0, width DIAMOND_W. So
 * beads ramp up just before totality and fade out just after.
 *
 * Each bead is at a fixed (per-seed) angular position around the sun's
 * apparent limb, biased toward the side OPPOSITE the moon (where the
 * limb is most visible). Each bead has its own brightness multiplier
 * so they don't all flash equally — characteristic of real beads,
 * which depend on which lunar valleys are aligned with the photosphere.
 *
 * Returns the accumulated bead radiance contribution for the cell.
 */
static RGB shade_beads(float sx, float sy,
                       float sun_sx, float sun_sy, float sun_r_screen,
                       float moon_sx, float moon_sy,
                       float sep, float sep_T,
                       int seed, RGB bead_col)
{
    /* Window: how close are we to the totality boundary? */
    float dw = (sep - sep_T) / DIAMOND_W;
    float window = expf(-dw * dw);
    if (window < 0.01f) return rgb_make(0,0,0);

    /* Direction from sun to moon (in screen space). Beads cluster on
     * the side OPPOSITE the moon (the just-uncovered limb). */
    float to_moon_x = moon_sx - sun_sx;
    float to_moon_y = (moon_sy - sun_sy) * ASPECT_Y;
    float to_moon_l = sqrtf(to_moon_x*to_moon_x + to_moon_y*to_moon_y);
    if (to_moon_l < 1e-6f) to_moon_l = 1.f;
    float anti_angle = atan2f(-to_moon_y / to_moon_l,
                              -to_moon_x / to_moon_l);

    RGB acc = rgb_make(0, 0, 0);
    for (int k = 0; k < BEAD_COUNT; k++) {
        uint32_t h = hash3(k, seed, 0xBEAD);
        /* Per-bead angle: anti_angle + random offset in ±π/2. */
        float angle_jitter = (hash01(h) - 0.5f) * (float)M_PI;
        float bead_phi     = anti_angle + angle_jitter;
        /* Per-bead intensity: 0.4..1.0. */
        float intensity    = 0.4f + hash01(h ^ 0x12345678u) * 0.6f;

        /* Bead position on sun's apparent limb (screen coords). */
        float bx = sun_sx + cosf(bead_phi) * sun_r_screen;
        float by = sun_sy + sinf(bead_phi) * sun_r_screen / ASPECT_Y;

        float dx = sx - bx;
        float dy = (sy - by) * ASPECT_Y;
        float r2 = dx*dx + dy*dy;
        float bead_w = expf(-r2 / (BEAD_RADIUS_CELLS * BEAD_RADIUS_CELLS));
        if (bead_w < 0.01f) continue;

        float br = window * bead_w * intensity * BEAD_BRIGHTNESS;
        acc = rgb_add(acc, rgb_scl(bead_col, br));
    }
    return acc;
}

/* ── §10 buffer + post-processing ────────────────────────────────────── */

/* §10.1 colour buffer + bloom buffer (no malloc). */

static RGB g_buf  [BUF_MAX_H][BUF_MAX_W];
static RGB g_bloom[BUF_MAX_H][BUF_MAX_W];

/* §10.2 bloom — 5×5 bright-pass Gaussian blur ──────────────────────── */

/*
 * For each cell, gather Gaussian-weighted contributions from neighbours
 * whose luma > threshold. Write to a SEPARATE bloom buffer (NOT g_buf
 * — single-pass would compound asymmetrically). After the pass, fold
 * g_bloom into g_buf.
 *
 * Result: bright shaft cores leak warm light into 5×5 neighbours,
 * turning sharp bright pixels into atmospheric glow.
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
            RGB sum = rgb_make(0,0,0);
            for (int dy = -R; dy <= R; dy++) {
                int nr = r + dy;
                if (nr < 0 || nr >= rows || nr >= BUF_MAX_H) continue;
                for (int dx = -R; dx <= R; dx++) {
                    int nc = c + dx;
                    if (nc < 0 || nc >= cols || nc >= BUF_MAX_W) continue;
                    RGB bv = g_buf[nr][nc];
                    if (luma_of(bv) < BLOOM_LUMA_THRESHOLD) continue;
                    float weight = w[dy+R][dx+R];
                    sum = rgb_add(sum, rgb_scl(bv, weight));
                }
            }
            g_bloom[r][c] = rgb_scl(sum, BLOOM_GAIN);
        }
    }

    /* Pass 2: fold bloom into g_buf. */
    for (int r = 0; r < rows && r < BUF_MAX_H; r++) {
        for (int c = 0; c < cols && c < BUF_MAX_W; c++) {
            g_buf[r][c] = rgb_add(g_buf[r][c], g_bloom[r][c]);
        }
    }
}

/* ── §11 screen + scene_draw + HUD ───────────────────────────────────── */

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

/* §11.1 Camera helpers. */

typedef struct {
    V3   pos;
    float fov_h, fov_v;
    int  cols, rows;
} Camera;

static void camera_make(Camera *c, int cols, int rows)
{
    c->cols  = cols;
    c->rows  = rows;
    c->pos   = v3(0, 0, 0);
    c->fov_h = FOV_H;
    /* Vertical FOV adjusted by terminal cell aspect so spheres render
     * round on screen rather than as vertical ellipses. */
    c->fov_v = FOV_H * (float)rows * ASPECT_Y / (float)cols;
}

static V3 camera_ray(const Camera *c, int sx, int sy)
{
    float u = ( (2.0f * (float)sx + 1.0f) - (float)c->cols)
            / (float)c->cols * c->fov_h;
    float v = -((2.0f * (float)sy + 1.0f) - (float)c->rows)
            / (float)c->rows * c->fov_v;
    return v3_norm(v3(u, v, 1.0f));
}

/* World point → screen cell — used for bead position computation. */
static void world_to_screen(const Camera *c, V3 p, float *sx_f, float *sy_f)
{
    float u = p.x / p.z;
    float v = p.y / p.z;
    *sx_f = ((u / c->fov_h) + 1.0f) * 0.5f * (float)c->cols - 0.5f;
    *sy_f = ((-v / c->fov_v) + 1.0f) * 0.5f * (float)c->rows - 0.5f;
}

/*
 * scene_draw — full eclipse pipeline, three passes.
 *
 *   PASS 1  per-cell RGB shader → g_buf[r][c]
 *           - ray-sphere sun + moon, depth-sort
 *           - sun cell    : shade_sun  (limb + granulation)
 *           - moon cell   : earthshine (palette.moon)
 *           - space cell  : shade_corona + shade_chromosphere
 *           - beads (gated by totality_possible) added to space cells
 *           - sparse stars during totality
 *   PASS 2  bloom (5×5 bright-pass Gaussian) → g_buf
 *   PASS 3  paint g_buf → screen via paint_cell
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int rows_eff = sc->rows - 2;
    int row_off  = 1;
    if (rows_eff < 4) { rows_eff = sc->rows; row_off = 0; }
    if (rows_eff > BUF_MAX_H) rows_eff = BUF_MAX_H;
    int cols_eff = sc->cols;
    if (cols_eff > BUF_MAX_W) cols_eff = BUF_MAX_W;

    Camera cam;
    camera_make(&cam, cols_eff, rows_eff);

    V3 sun_pos  = v3(0, 0, SUN_Z);
    V3 moon_pos = scene_moon_pos(s);

    V3    sun_dir  = v3_norm(sun_pos);
    V3    moon_dir = v3_norm(moon_pos);
    float cos_b    = v3_dot(sun_dir, moon_dir);
    if (cos_b > 1.f)  cos_b = 1.f;
    if (cos_b < -1.f) cos_b = -1.f;
    float sep    = acosf(cos_b);

    float sun_a  = atanf(SUN_R              / SUN_Z );
    float moon_a = atanf(s->pp.moon_world_r / MOON_Z);
    float occ    = occlusion_factor(sep, sun_a, moon_a);
    float sep_T  = fabsf(moon_a - sun_a);

    /* Build palette ONCE per frame from current star temperature. */
    Palette pal = palette_from_kelvin(STAR_PRESETS[s->star_preset].kelvin);

    /* Sun + moon screen positions for streamers and beads. */
    float sun_sx, sun_sy, moon_sx, moon_sy;
    world_to_screen(&cam, sun_pos,  &sun_sx,  &sun_sy);
    world_to_screen(&cam, moon_pos, &moon_sx, &moon_sy);
    float sun_r_screen = sun_a / cam.fov_h * (float)cam.cols * 0.5f;

    /* Beads only fire in TOTAL-pattern-like configurations. */
    bool beads_active = s->pp.totality_possible && (moon_a > sun_a);

    /* PASS 1 — per-cell RGB shader =================================== */
    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols_eff; c++) {
            V3 ray_d = camera_ray(&cam, c, r);

            float t_sun = 0, t_moon = 0;
            bool sun_h  = ray_sphere(cam.pos, ray_d, sun_pos,  SUN_R,
                                     &t_sun);
            bool moon_h = ray_sphere(cam.pos, ray_d, moon_pos,
                                     s->pp.moon_world_r, &t_moon);

            RGB col = pal.sky;
            bool drew_sphere = false;

            if (moon_h && (!sun_h || t_moon < t_sun)) {
                /* Moon body — earthshine tint, not pure black. */
                col = pal.moon;
                drew_sphere = true;
            } else if (sun_h) {
                /* Sun visible — limb darkening + granulation. */
                V3 hit = v3_add(cam.pos, v3_scl(ray_d, t_sun));
                V3 N   = v3_norm(v3_sub(hit, sun_pos));
                bool ann_boost = s->pp.annular_mode && occ > 0.5f;
                col = shade_sun(N, ray_d, pal.sun, ann_boost);
                drew_sphere = true;
            }

            /* Corona + chromosphere — only on space cells. */
            if (!drew_sphere && s->pp.show_corona) {
                /* Angular distance from view ray to sun direction. */
                float cv = v3_dot(ray_d, sun_dir);
                if (cv > 1.f)  cv = 1.f;
                if (cv < -1.f) cv = -1.f;
                float a_view = acosf(cv);
                float d_out  = a_view - sun_a;

                if (d_out >= 0.f) {
                    /* Polar angle of cell relative to sun's screen
                     * centre — drives streamer structure. */
                    float dx = (float)c + 0.5f - sun_sx;
                    float dy = ((float)r + 0.5f - sun_sy) * ASPECT_Y;
                    float pa = atan2f(dy, dx);

                    RGB cor = shade_corona(d_out, pa, occ, s->time_secs,
                                           pal.corona);
                    col = rgb_add(col, cor);

                    RGB chr = shade_chromosphere(d_out, occ, pal.chromos);
                    col = rgb_add(col, chr);
                }

                /* Stars — sparse, visible only during totality. */
                if (occ > STAR_OCC_VISIBLE) {
                    uint32_t h = hash3(c, r, s->seed);
                    if ((h % STAR_DENSITY) == 0u) {
                        float br = 0.6f + hash01(h >> 8) * 0.4f;
                        col = rgb_add(col, rgb_scl(rgb_make(0.9f, 0.95f, 1.0f),
                                                   br));
                    }
                }
            }

            /* Bailey's beads — additive at the totality boundary. */
            if (beads_active && !drew_sphere) {
                RGB beads = shade_beads((float)c + 0.5f, (float)r + 0.5f,
                                        sun_sx, sun_sy, sun_r_screen,
                                        moon_sx, moon_sy,
                                        sep, sep_T,
                                        s->seed, pal.bead);
                col = rgb_add(col, beads);
            }

            g_buf[r][c] = col;
        }
    }

    /* PASS 2 — bloom. */
    bloom_apply(cols_eff, rows_eff);

    /* PASS 3 — paint to screen. */
    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols_eff; c++) {
            paint_cell(c, r + row_off, g_buf[r][c]);
        }
    }
}

/* §11.2 HUD — yellow status row 0, cyan hint bottom row (HUD spec). */

static void hud_draw(const Screen *sc, const Scene *s,
                     double fps, int sim_fps)
{
    const StarPreset *sp = &STAR_PRESETS[s->star_preset];
    char buf[200];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  %s %5.0fK  phase:%s  speed:%-2d ",
             fps, sim_fps,
             s->paused ? "PAUSED " : pattern_name(s->current_pattern),
             sp->name, (double)sp->kelvin,
             phase_str_for(s),
             s->speed);
    int len = (int)strlen(buf);
    if (len > sc->cols) len = sc->cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, " SOLAR-ECLIPSE · BLACKBODY ");
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  n/p:pat  t/T:star  +/-:spd  []:Hz  r:reseed ");
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

/* ── §12 app ─────────────────────────────────────────────────────────── */

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
    case 'r': case 'R': scene_reseed(s); break;

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
        s->star_preset = (s->star_preset + 1) % N_STAR_PRESETS;
        break;
    case 'T':
        s->star_preset = (s->star_preset + N_STAR_PRESETS - 1) % N_STAR_PRESETS;
        break;

    case 'n': case 'N':
        pattern_set(s, (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS));
        break;
    case 'p': case 'P':
        pattern_set(s, (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS));
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

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* §12.1 resize. */
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* §12.2 timing. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* §12.3 fixed-step physics. */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* §12.4 fps rolling average over half-second windows. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* §12.5 paint. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* §12.6 input. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;

        /* §12.7 frame cap (~60 fps render). */
        int64_t target_ns = NS_PER_SEC / 60;
        int64_t elapsed   = clock_ns() - now;
        clock_sleep_ns(target_ns - elapsed);
    }

    return 0;
}
