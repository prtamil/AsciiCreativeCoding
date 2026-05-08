/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * saturn_with_rings.c — analytic raytraced ringed planet, RGB pipeline
 *
 * DEMO: A huge planet (40-ish cells across) sits in the middle of a
 *       starfield. A flat ring system extends ~80 cells horizontally,
 *       an ellipse in screen space because we view the rings at a
 *       small tilt. The planet has a clean DAY/NIGHT terminator;
 *       atmosphere darkens its silhouette and a warm rim glows where
 *       sunlit edge meets space. The ring system disappears BEHIND
 *       the planet, the planet casts a SOFT SHADOW BAND across the
 *       far side, and the RINGS in turn cast their own shadow back —
 *       parallel dim STREAKS across the planet's lit hemisphere,
 *       brightest where the Cassini gap lets full sunlight through.
 *       Where the sun sits BEHIND the ring (relative to the camera)
 *       the sparse ring sections GLOW from forward-scattered light —
 *       the famous Cassini "lit-from-behind" look. The sun orbits
 *       slowly so all of these effects sweep together around the scene.
 *
 *       PATTERNS  (n / p):
 *         SATURN     banded cream planet, broad rings, Cassini gap
 *         URANUS     pale smooth planet, thin rings, near edge-on
 *         EARTH-R    blue-green continent map planet + speculative rings
 *         EXO        per-seed random tints + rings
 *
 *       'r' reseeds (sun phase, exo tints, continent shuffle).
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c  — ray-sphere math, three-light shading
 *   raytracing/path_tracer.c      — RGB-cube pipeline + tone mapping
 *
 * Section map:
 *   §1 config        — frame rate, planet/ring geometry, shading consts
 *   §2 clock         — monotonic timer + sleep
 *   §3 math          — V3 + RGB helpers (mix, clamp, reinhard, gamma)
 *   §4 noise         — Perlin + fBm (continents + EXO randomness)
 *   §5 themes        — RGB-triplet palettes per role (NOT pair indices)
 *   §6 color         — cube pair init + paint_cell (RGB → cube + ramp)
 *   §7 patterns      — pattern → params (sphere/ring/colour mapping)
 *   §8 raytrace      — V3 ray primitives + shadow utilities
 *                      §8.1 ray_sphere   §8.2 ray_ring
 *                      §8.3 hard_shadow  §8.4 soft_shadow
 *   §9 shading       — RGB shaders for each scene element
 *                      §9.1 limb darkening + atmospheric rim
 *                      §9.2 shade_planet
 *                      §9.3 shade_ring (forward-scatter + soft shadow)
 *                      §9.4 shade_space (stars)
 *   §10 scene        — Scene state, sun motion, init/reseed/tick
 *   §11 render       — camera, per-cell ray, depth-sort, paint
 *   §12 hud          — yellow row 0 status, cyan bottom hint strip
 *   §13 app          — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume
 *   r            reseed
 *   n / p        next / prev pattern
 *   t / T        next / prev theme
 *   + / =        faster sun rotation     (-: slower)
 *   ] / [        raise / lower tick Hz
 *   s            cycle SPP (1, 2, 4) — anti-alias quality
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/saturn_with_rings.c \
 *       -o saturn_with_rings -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order as
 *      prose before touching code.
 *   2. §1 config — every constant has a unit-bearing comment, so a
 *      glance is the fastest tour of what's tunable.
 *   3. §8 raytrace primitives (ray_sphere, ray_ring, soft_shadow) —
 *      read AFTER tutorials T2.
 *   4. §9 shading (the heart of the file) — read AFTER tutorials
 *      T3-T8. Each shader is a small layer; together they are the
 *      six realism stages.
 *   5. §11 render and §10 scene wire it all together. §5 themes is
 *      pure data — skim.
 *   6. §4 noise + §6 paint + §12/§13 are infrastructure; skip unless
 *      curious.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Long descriptive names everywhere. The file uses these:
 *     ray_origin / ray_dir   the ray as (point, unit direction)
 *     hit_planet / hit_ring  surface points on the two primitives
 *     N / view_dir / sun_dir surface normal and light/view directions
 *     NdotL / NdotV          dot products for shading math
 *     albedo / sun_col       material × light (Lambertian setup)
 *     density (rings)        radial × azimuthal × Cassini falloff
 *
 *   When a one-letter name appears it's a tight-loop index (`i`, `s`)
 *   or a math symbol whose meaning is on the line above.
 *
 * Background you need
 * ───────────────────
 *   - 3-D vectors (dot, cross, normalise).
 *   - The quadratic formula and what its discriminant means
 *     geometrically (positive → two real roots → ray hits sphere).
 *   - Lambertian diffuse: max(0, N · L) is the "fraction of incoming
 *     light at this surface point".
 *   - 6×6×6 RGB colour cube on xterm-256 (16 + 36·r + 6·g + b).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Recursive raytracing — every ray here is one bounce only.
 *   - PDFs, importance sampling — soft shadow uses a small uniform
 *     cone, that's it.
 *   - Path tracing or Monte Carlo theory — see path_tracer.c for that.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Per-pixel analytic raytracing of two primitives — a
 *                  SPHERE (the planet) and a flat ANNULUS (the rings).
 *                  For each cell:
 *                    (1) build a view ray from the camera through that
 *                        cell;
 *                    (2) intersect ray with sphere → t_sphere;
 *                    (3) intersect ray with ring plane (clipped to
 *                        annulus radii) → t_ring;
 *                    (4) depth-sort: closer wins;
 *                    (5) shade the winner in continuous RGB space with
 *                        Lambert + limb darkening + atmospheric rim
 *                        (planet) or with forward-scattering glow + soft
 *                        shadow (rings);
 *                    (6) tone-map and quantise to xterm 6×6×6 colour
 *                        cube + 92-char density ramp.
 *
 *                  Sub-pixel jitter (SPP rays per cell) anti-aliases the
 *                  silhouette. Soft shadow uses N rays into a small
 *                  angular cone around the sun for penumbra.
 *
 * Data-structure : Per frame: a Scene, a Camera, a sun direction. Themes
 *                  are RGB triplets, not pre-baked colour pairs — this
 *                  is the key change from the old ramp-slot pipeline.
 *                  216 ncurses pairs are pre-allocated as a 6×6×6 RGB
 *                  cube, and any computed RGB ∈ [0,1]³ quantises into
 *                  one of those 216 pairs at draw time.
 *
 * Rendering      : Continuous RGB through the shader, Reinhard tone-map
 *                  L'=L/(1+L), gamma encode L^(1/2.2), then quantise to
 *                  xterm cube. Density character chosen from rec601
 *                  luminance via Bourke 92-char ramp. Both the colour
 *                  and the glyph density carry shading information —
 *                  same trick as `path_tracer.c` and the other
 *                  raytrace files.
 *
 * Performance    : 1 ray-sphere + 1 ray-ring + (rings only) up to 8
 *                  shadow rays per pixel per sample. SPP=2 default.
 *                  At 240×80 cells × 30 fps × 2 spp ≈ 1.2 M rays/s,
 *                  ~6 ms shading per frame on modern hardware. SPP=4
 *                  doubles it to ~12 ms — comfortable inside the 33 ms
 *                  budget.
 *
 * References     : Shirley, "Ray Tracing in One Weekend"
 *                    https://raytracing.github.io/books/RayTracingInOneWeekend.html
 *                  Inigo Quílez, "Sphere — intersection"
 *                    https://iquilezles.org/articles/intersectors/
 *                  Reinhard et al., "Photographic Tone Reproduction
 *                    for Digital Images," SIGGRAPH '02.
 *                  Hapke, "Theory of Reflectance and Emittance
 *                    Spectroscopy" 2e — particle scattering and the
 *                    forward/backward asymmetry that makes ring
 *                    backlight glow brighter than ring frontlight.
 *                  Wikipedia: Saturn's Rings + Cassini Division
 *                    https://en.wikipedia.org/wiki/Rings_of_Saturn
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two analytic primitives + six realism layers stacked on top.
 * The primitives (sphere + annulus) decide WHAT each pixel sees;
 * the six shading layers decide what COLOUR it should be:
 *
 *      LAMBERT          gives  light/dark hemispheres
 *      LIMB DARKENING   gives  spherical "fall-off" toward silhouette
 *      ATMOSPHERIC RIM  gives  warm glow on the sunlit edge
 *      FORWARD SCATTER  gives  rings glow when sun is behind them
 *      SOFT SHADOW      gives  ring shadow with a real penumbra
 *      RING SHADOW      gives  dim streaks across the sunlit hemisphere
 *                              of the planet, drawn by the ring's
 *                              density (banding + Cassini gap visible)
 *
 * Each layer is a few lines of math added to the colour vector. None of
 * them require a new ray-trace; they're all derivable from quantities
 * the primary intersection already gave us (N, P, t, sun_dir,
 * view_dir).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *
 *      sun → → →                     ╲
 *                ⌒⌒⌒⌒                 ╲     ← view ray
 *              ⌒        ⌒              ╲
 *      ─────●─    PLANET    ─●─────────●→  camera
 *              ⌒        ⌒              ╱
 *                ⌒⌒⌒⌒                 ╱
 *                  │ │ │              ╱
 *                  ▼ ▼ ▼   ← penumbra of soft shadow on rings
 *
 * Imagine the planet as a chocolate-coated marble, the rings as a paper
 * plate, photographed from one corner of a dim room with a flashlight
 * on the other side. The marble's lit half faces the flashlight (Lambert).
 * Near the marble's edge the chocolate looks darker because the light
 * grazes there at a steep angle (limb darkening). A faint warm halo
 * hugs the edge where the atmosphere catches sunlight (rim). On the
 * back of the paper plate the marble casts a soft-edged shadow because
 * the flashlight isn't a point — it's a small disc, and parts of the
 * shadow can still see SOME of the disc (penumbra). And on the SIDE of
 * the plate that's BETWEEN you and the flashlight, the dust on the
 * paper LIGHTS UP because the dust scatters forward; sparse parts of
 * the plate (low density of dust) glow brightest because you can SEE
 * the bright background through them.
 *
 * That's everything this demo simulates. The RGB pipeline (xterm 6×6×6
 * cube + 92-char density ramp on top of it) gives us ~20 000 effective
 * shades per cell — enough to actually CARRY all of those subtle
 * gradients across the screen without quantisation banding.
 *
 * ALGORITHM IN STEPS  (per pixel per sample)
 * ─────────────────────────────────────────
 *  1. CAMERA → primary view ray rd = normalize(fwd + u·right + v·up)
 *     with sub-pixel jitter (u,v offset by ±0.5).
 *  2. INTERSECT ray vs sphere → t_sphere. Vs ring plane clipped to
 *     annulus → t_ring. Pick the smaller positive t.
 *  3. SHADE THE WINNER in RGB (one of the §9 functions):
 *     PLANET                                     RING
 *     ───────                                    ──────
 *     N = normalize(P)                           density = profile(r)
 *     NdotL  = max(0, N·sun)                     bands   = sin(r·k+θ·...)
 *     NdotV  = |N·view|                          density *= bands
 *     Lambert: albedo · NdotL · sun_col          If Cassini-ish radius:
 *     Fill   : albedo · NdotF · fill_col           density *= dim
 *     Ambient: albedo · 0.05                     soft = soft_shadow(P,sun,8)
 *     Limb   : pow(NdotV, 0.5)                   diffuse = |sun.y|·soft
 *     Rim    : if NdotV < .25 & NdotL > 0:        backlight = max(0,-sun·view)
 *                col += (1−NdotV/.25)²            forward = backlight³·(1−density)
 *                       · NdotL · warm·rim         col = density·base·diffuse·sun
 *     Spec   : pow(R·V, 24)·NdotL·0.4 · spec           + forward·sun·0.8
 *                                                      + ambient
 *     col = limb · (ambient + diffuse + fill +
 *                    spec)  +  rim
 *  4. NO HIT → space (very dim sky + maybe a hashed star with twinkle).
 *  5. PAINT cell:
 *       reinhard tone-map → gamma → quantise to 6×6×6 cube pair
 *       luminance → 92-char ramp glyph
 *       attr: A_BOLD if luma > 0.85, A_DIM if < 0.15, else A_NORMAL
 *
 * KEY FORMULAS
 * ────────────
 *   Lambert (diffuse):
 *     N   = normalize(P − origin_of_planet)
 *     L   = normalize(sun_pos − P)         (or just sun_dir if directional)
 *     diffuse = max(0, N · L) · albedo · sun_colour
 *
 *   Limb darkening (atmospheric absorption — applies to REFLECTED
 *   light only, NOT to ambient/fill):
 *     NdotV = |N · view_dir|
 *     limb  = pow(NdotV, k)        k ≈ 0.5 (gentle), > 1 = aggressive
 *     reflected = (diffuse + specular) · limb        ← only these
 *     col       = ambient + fill + reflected         ← keep ambient/fill
 *
 *   Why only reflected light: limb darkening models atmosphere
 *   absorbing light that bounces off the surface and travels to the
 *   camera at grazing angles. Ambient + fill represent skylight
 *   reaching the surface from many directions, so they don't depend
 *   on the camera's outgoing path. Multiplying everything by limb
 *   would make the silhouette go pitch-black; this form leaves the
 *   ambient + fill intact at the edge so the silhouette has a
 *   visible body colour before the rim glow takes over.
 *
 *   Atmospheric rim glow (warm glow on lit silhouette edge):
 *     if NdotV < RIM_WIDTH and NdotL > 0:
 *       t = 1 − NdotV / RIM_WIDTH        (1 at silhouette, 0 inside)
 *       rim = t² · NdotL · RIM_STRENGTH
 *       col += rim · WARM_RIM_TINT
 *
 *   Forward-scattering ring glow:
 *     backlight = max(0, −sun_dir · view_dir)        (1 when sun "behind")
 *     scatter   = backlight^GLOW_SHARP · (1 − density) · GLOW_GAIN
 *     col      += scatter · sun_colour
 *
 *   Soft shadow (N samples in a small cone around sun_dir):
 *     for i in 0..N-1:
 *        offset_i = uniform_disc_2d() · SUN_ANGULAR_RADIUS
 *        sample_dir = normalize(sun_dir + tangent_basis · offset_i)
 *        if not sphere_hit(P, sample_dir): n_lit++
 *     soft = n_lit / N                                 ∈ [0, 1]
 *
 *   Reinhard tone map + gamma:
 *     L'  = L / (1 + L)
 *     out = L'^(1/2.2)
 *
 *   RGB → 6×6×6 cube pair (xterm 16..231):
 *     r5 = clamp(round(out_r · 5), 0, 5)
 *     g5 = clamp(round(out_g · 5), 0, 5)
 *     b5 = clamp(round(out_b · 5), 0, 5)
 *     pair_id = PAIR_CUBE_BASE + r5·36 + g5·6 + b5
 *
 * WORKED EXAMPLE  (verify by hand)
 * ────────────────────────────────
 *   At a pixel near the lit equator on SATURN (current constants:
 *   AMBIENT_K=0.18, FILL_K=0.40, SUN_ELEV_Y=0.45, LIMB_K=0.5):
 *     N        = (0.95, 0, 0.31)         (front-right of planet)
 *     sun_dir  = (0.89, 0.45, 0)         (right + raised elevation)
 *     view_dir = (0, 0.22, -0.97)        (toward camera, slight down)
 *     fill_dir = -sun_dir = (-0.89, -0.45, 0)
 *     albedo   = (0.95, 0.86, 0.65)      (cream)
 *     sun_col  = (1.0, 0.92, 0.78)       (warm white)
 *     fill_col = (0.32, 0.42, 0.58)      (cool blue)
 *
 *     NdotL = 0.95·0.89 + 0·0.45 + 0.31·0          = 0.846
 *     NdotF = max(0, N·fill_dir)
 *           = max(0, 0.95·(-0.89) + 0)              = 0    (back face)
 *     NdotV = |0.95·0 + 0·0.22 + 0.31·(−0.97)|     = 0.301
 *
 *     limb        = pow(0.301, 0.5)                 ≈ 0.549
 *     ambient     = albedo · 0.18
 *                 ≈ (0.171, 0.155, 0.117)
 *     diffuse_RGB = albedo · sun_col · NdotL
 *                 = (0.95·1.0·0.846, 0.86·0.92·0.846, 0.65·0.78·0.846)
 *                 ≈ (0.804, 0.669, 0.429)
 *     fill_RGB    = albedo · fill_col · NdotF · 0.40 = 0  (NdotF=0 here)
 *     specular    ≈ 0   (sun and view directions ≈ orthogonal here)
 *
 *     reflected   = (diffuse + specular) · limb
 *                 ≈ (0.804, 0.669, 0.429) · 0.549
 *                 ≈ (0.441, 0.367, 0.236)
 *     col         = ambient + fill + reflected
 *                 ≈ (0.612, 0.522, 0.353)
 *
 *     Rim?  NdotV=0.301 > 0.28 → no rim contribution at this pixel.
 *
 *     Reinhard:    (0.380, 0.343, 0.261)
 *     Gamma 2.2:   (0.654, 0.622, 0.531)
 *
 *     6×6×6 quantise:
 *       r5=round(0.654·5)=3, g5=round(0.622·5)=3, b5=round(0.531·5)=3
 *       pair = 3·36 + 3·6 + 3 + 1 = 130  (warm cream)
 *
 *     Luminance = 0.21·0.654 + 0.72·0.622 + 0.07·0.531 ≈ 0.624
 *     Ramp index = round(0.624 · 91) = 57 → glyph 'Z'
 *
 *   Now move one pixel toward the silhouette: NdotV drops below 0.28,
 *   the rim term kicks in, ambient + fill survive (limb only attenuates
 *   diffuse + specular), and the silhouette retains a faint cream body
 *   colour before the warm rim glow overlays it.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • RIM ONLY ON LIT SIDE. The rim term must be gated by NdotL > 0 —
 *    a rim glow on the dark hemisphere would just be a halo around
 *    the entire silhouette and look fake.
 *  • SOFT-SHADOW DETERMINISM. Use a hash of (sx, sy, frame, sample)
 *    to seed the cone-jitter offsets so the same pixel jitters the
 *    same way each frame. Otherwise the penumbra "boils".
 *  • SPP COST IS LINEAR. SPP=4 doubles the runtime of SPP=2. Soft
 *    shadow cost is N_SHADOW per ring pixel; total scaling is
 *    O(SPP × (1 + N_SHADOW · ring_fill_fraction)).
 *  • TONE-MAP BEFORE QUANTISE. We must Reinhard + gamma BEFORE the
 *    6×6×6 quantisation. Quantising linear HDR puts most pixels in
 *    one cube cell because the cube is uniform in [0,1]³ — no headroom.
 *  • RING BACKLIGHT vs FRONTLIGHT. backlight = max(0, −sun·view) is
 *    nonzero only when sun is roughly OPPOSITE the view (sun behind
 *    the rings from camera's POV). When sun is on the camera side,
 *    backlight = 0 and rings just look diffuse-lit — exactly right.
 *  • CASSINI GAP IS SMOOTH. The old code used a hard `< CASSINI_W`
 *    test; we now use a smoothstep-shaped dip so the gap edges
 *    don't pixel-step.
 *  • RING SHADOW SAME-SIDE GUARD. ring_transmittance returns 1.0
 *    (no shadow) when sun and planet hit point are on the SAME side
 *    of the ring plane (sun_dir.y · P.y >= 0). Without this guard,
 *    the chord toward the sun never crosses the ring plane in
 *    forward t and the shadow streaks would be wrong (or appear on
 *    the wrong hemisphere).
 *  • RING SHADOW MULTIPLIES DIRECT-SUN ONLY. Diffuse + specular +
 *    rim get multiplied by ring_transmittance; ambient + fill do
 *    NOT (they're skylight, not direct sun). Multiplying everything
 *    would over-darken the streaks and lose the sense of an
 *    ambient-lit planet with sharp shadow lines drawn across it.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE → CYCLE PATTERN. Each pattern keeps lighting consistent
 *    while the planet/ring colour and band frequency change.
 *  • LIMB DARKENING. The planet should look noticeably darker AT
 *    THE SILHOUETTE than at its centre, even on the lit side. Toggle
 *    the limb factor to 1.0 in code and you'll see the difference.
 *  • ATMOSPHERIC RIM. On the lit-side silhouette there should be
 *    a warm orange-gold halo, brightest at the very edge and fading
 *    inward over a few cells.
 *  • FORWARD-SCATTER. Wait until the sun has rotated to be roughly
 *    behind the planet relative to the camera. The ring sections
 *    BETWEEN sun and camera should look noticeably brighter than the
 *    front sections — sparse Cassini-gap region especially.
 *  • SOFT SHADOW. Increase ring opacity (or pause and zoom in
 *    mentally) — the shadow's edge is GRADIENT, not stair-step. As
 *    the sun rotates, the shadow band sweeps with a soft leading
 *    edge.
 *  • RING SHADOW STREAKS. Cycle to the SATURN pattern (which has the
 *    Cassini gap) and pause when the sun is roughly OFF-AXIS but
 *    still on the camera-side of the rings. The lit hemisphere of
 *    the planet should show parallel dim BANDS — the projection of
 *    the ring's density profile along the sun direction. The CASSINI
 *    GAP appears as a brighter STRIPE in the middle of those bands
 *    (where ring density is near zero, light passes freely). As the
 *    sun rotates, the streaks sweep across the planet.
 *  • SUB-PIXEL AA. Press 's' to cycle SPP=1/2/4. SPP=1 should show
 *    visible jaggies on the planet silhouette; SPP=2/4 should smooth
 *    them out.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Eleven short tutorials that build the renderer from first principles.
 * Read in order; each builds on the previous.
 *
 *   T1  Two primitives + six layers — the pedagogical handle
 *   T2  Ray-sphere + ray-annulus intersection
 *   T3  Layer 1: Three-light Lambertian shading (sun + fill + ambient)
 *   T4  Layer 2: Limb darkening — and why it applies only to
 *               REFLECTED light (the "atmospheric absorption" subtlety)
 *   T5  Layer 3: Atmospheric rim glow at the lit silhouette
 *   T6  Layer 4: Forward-scatter ring glow (Cassini's lit-from-behind)
 *   T7  Layer 5: Soft shadows — planet on rings via cone-jittered samples
 *   T8  Layer 6: Ring shadows — rings on planet via chord-to-sun
 *   T9  Surface texture — bands, continents, smooth Cassini gap
 *   T10 Continuous-RGB pipeline → 6×6×6 cube + 92-char ramp
 *   T11 Three shade modes — LIT vs FLAT vs NORMAL as debug overlays
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  TWO PRIMITIVES + SIX LAYERS — THE PEDAGOGICAL HANDLE
 * ────────────────────────────────────────────────────────
 * Most "ringed planet" demos look complicated. This one isn't —
 * it's two analytic intersections plus six INDEPENDENT shading
 * layers. Each layer is a few lines of math; together they give
 * the photorealistic look.
 *
 * Two primitives:
 *   1. SPHERE       the planet (one ray-quadratic per pixel)
 *   2. ANNULUS      the rings (one ray-plane test, then radial
 *                   bound check between r_in and r_out)
 *
 * Six additive shading layers (per pixel of either primitive):
 *   1. LAMBERT          three-light setup (sun + fill + ambient)
 *   2. LIMB DARKENING   spherical brightness fall-off near silhouette
 *   3. ATMOSPHERIC RIM  warm halo at the lit edge of the silhouette
 *   4. FORWARD SCATTER  rings glow when sun is behind them
 *   5. SOFT SHADOW      planet's shadow on rings has real penumbra
 *   6. RING SHADOW      rings cast streak-shadows on planet's lit side
 *
 * Layers 1-4 derive from quantities the primary intersection already
 * gave us (N, P, sun_dir, view_dir) — no extra rays. Layer 5 needs
 * N=8 cheap sphere tests per ring pixel for penumbra. Layer 6 is the
 * symmetric counterpart of layer 5: from each PLANET hit point, one
 * cheap ray-plane crossing + ring-density lookup tells us how much
 * sunlight is occluded by the rings (no extra full ray trace).
 *
 * Layers 5 and 6 are PHYSICALLY DUAL: planet-occludes-sun-from-rings
 * (shadow band on rings) and rings-occlude-sun-from-planet (shadow
 * streaks on planet), respectively. Real Saturn imagery shows BOTH
 * — the file's pedagogical spine wouldn't be complete without it.
 *
 * The "two primitives + six layers" framing is the file's reading
 * key: each tutorial T2-T8 covers one of these pieces in isolation.
 * Once you understand each piece, putting them together is bookkeeping.
 *
 * T2  RAY-SPHERE + RAY-ANNULUS INTERSECTION
 * ──────────────────────────────────────────
 * RAY-SPHERE — textbook quadratic. Substitute the parametric ray
 * |O + t·D − C|² = r² and expand:
 *
 *     t² + 2·b·t + c = 0,    b = D·(O−C), c = |O−C|² − r²
 *     disc = b² − c
 *     if disc < 0: MISS
 *     t = −b − √disc   (front face)
 *
 * Read §8.1 ray_sphere — that IS this algebra in code.
 *
 * RAY-ANNULUS — even simpler. The rings are a flat disc in the
 * plane y=0, clipped to two radii r_in ≤ r ≤ r_out:
 *
 *     1. Skip if ray is parallel to the plane (|D.y| < ε).
 *     2. Find t where the ray crosses y=0:    t = −O.y / D.y
 *     3. If t < 0:                            MISS (behind camera).
 *     4. Compute hit point  P = O + t·D
 *        and its 2-D radius r = √(P.x² + P.z²).
 *     5. If r < r_in or r > r_out:            MISS (inside hole or
 *                                              outside outer rim).
 *     6. Otherwise: HIT.
 *
 * Read §8.2 ray_ring — six lines of code; that's the entire ring
 * geometry. The fancy stuff (density profile, Cassini gap, bands,
 * forward scatter, soft shadow) is all SHADING layered on top of
 * this trivially-cheap intersection.
 *
 * T3  LAYER 1: THREE-LIGHT LAMBERTIAN SHADING
 * ────────────────────────────────────────────
 * For a Lambertian surface, the incoming-radiance integral
 * collapses to:
 *
 *     L_o = albedo · (ambient + diffuse_lights)
 *
 * Three lights here:
 *
 *   AMBIENT        constant fraction of albedo
 *                  ambient = AMBIENT_K · albedo
 *                  Models indirect light reaching the surface from
 *                  every direction. Without this, the dark hemisphere
 *                  goes pitch-black.
 *
 *   SUN (KEY)      directional Lambertian
 *                  diffuse = max(0, N · sun_dir) · albedo · sun_col
 *                  Standard Lambert. The sun's WARM tint biases the
 *                  lit hemisphere toward orange-cream.
 *
 *   FILL (COOL)    fake-sky from the OPPOSITE direction
 *                  fill_dir = -sun_dir
 *                  fill = max(0, N · fill_dir) · FILL_K · albedo · fill_col
 *                  Models bluish skylight reaching the dark side.
 *                  Without this, the dark hemisphere reads as a flat
 *                  ambient blob; the cool fill differentiates the
 *                  shadow side from the deep-space background.
 *
 * Three-light setup (KEY warm + FILL cool + AMBIENT) is the
 * cinematic standard — same convention as portrait photography.
 *
 * Read §9.2 shade_planet → ambient + diffuse + fill blocks.
 *
 * T4  LAYER 2: LIMB DARKENING — REFLECTED-ONLY ATTENUATION
 * ─────────────────────────────────────────────────────────
 * Look at any photo of the Sun, Saturn, or Mars: the disc is
 * BRIGHTER in the centre and DARKER at the edge. This is LIMB
 * DARKENING, and physically it's atmospheric absorption: light
 * reflected at grazing angles travels through more atmosphere on
 * its way out to the camera, so more gets absorbed.
 *
 * Math: parameterise by NdotV = |N · view_dir|.
 *   NdotV = 1 at the disc CENTRE  (camera looks straight in)
 *   NdotV = 0 at the SILHOUETTE   (camera looks along the surface)
 *
 *     limb = pow(NdotV, k),    k ≈ 0.5  (gentle, square-root curve)
 *
 * Subtle but important: which contributions does limb darkening
 * APPLY to?
 *
 *   ✗ WRONG   `col *= limb` for everything → silhouette goes BLACK
 *             (limb=0 at NdotV=0 wipes ambient + fill too)
 *
 *   ✓ RIGHT   `col = ambient + fill + (diffuse + specular) · limb`
 *             reflected light gets attenuated, ambient + fill survive
 *
 * Why "reflected only": atmospheric absorption depends on the
 * OUTGOING path (surface to camera). Diffuse and specular are
 * reflected light bouncing OUT toward the camera — they take that
 * path. Ambient and fill represent skylight reaching the surface
 * from MANY directions — they don't depend on the outgoing path.
 *
 * Visual consequence: with the wrong formulation the silhouette
 * goes pitch-black and the planet looks like it has a black
 * outline. With the right formulation, the silhouette retains a
 * faint body colour from ambient + fill, and the rim glow then
 * overlays the lit edge naturally.
 *
 * Read §9.2 shade_planet for this version.
 *
 * T5  LAYER 3: ATMOSPHERIC RIM GLOW
 * ──────────────────────────────────
 * On Saturn or Earth photographs, the LIT silhouette has a warm
 * orange-gold halo. That's atmospheric scattering — when sunlight
 * grazes the atmosphere tangentially (relative to the camera),
 * Rayleigh-scattered red wavelengths travel toward the camera at
 * the silhouette.
 *
 * We approximate the effect with a triangular falloff cone:
 *
 *     if NdotV < RIM_WIDTH and NdotL > 0:
 *       t = 1 − NdotV / RIM_WIDTH        ← 1 at silhouette, 0 inside
 *       rim = t² · NdotL · RIM_STRENGTH
 *       col += rim · WARM_RIM_TINT
 *
 * Two gates:
 *   - NdotV < RIM_WIDTH    only near the silhouette
 *   - NdotL > 0            only on the LIT half (a halo on the dark
 *                          half would just be a fake aura)
 *
 * The squared-t gives a smooth falloff: at the very edge (t=1)
 * the rim is FULL strength × NdotL; a few cells inside (t=0.5)
 * it's a quarter strength; well inside (t=0) it's zero.
 *
 * Read §9.1 atmospheric_rim — five lines of code, instantly
 * recognisable visual feature.
 *
 * T6  LAYER 4: FORWARD-SCATTER RING GLOW
 * ───────────────────────────────────────
 * The famous Cassini "lit-from-behind" Saturn photographs show the
 * RING SECTIONS BETWEEN THE SUN AND THE CAMERA glowing brightly,
 * even where the rings are sparse (Cassini gap). Why?
 *
 * Ring particles are mostly transparent ice. When the sun is
 * BEHIND the ring (relative to the camera), light reaches the
 * camera by scattering FORWARD through the dust — same physics as
 * sunbeams in a misty room. SPARSE ring sections (low density) let
 * MORE light through; that's why the Cassini gap actually looks
 * BRIGHTER than the dense main bands when backlit.
 *
 * Pseudocode (matches §9.3 shade_ring):
 *
 *     backlight = max(0, -sun_dir · view_dir)        ← 1 if sun
 *                                                       OPPOSITE camera
 *     glow = backlight^GLOW_SHARP · (1 - density) · GLOW_GAIN
 *     col += glow · sun_col
 *
 * Two key ideas:
 *   - backlight is non-zero ONLY when the sun is roughly opposite
 *     the camera (sun-behind-rings configuration).
 *   - (1 - density) means SPARSE sections scatter more — the
 *     Cassini gap glows BRIGHTEST when backlit.
 *
 * GLOW_SHARP ≈ 3 makes the falloff steep (only a narrow angular
 * range gets the glow). GLOW_GAIN sets peak brightness.
 *
 * Visual: pause and watch the sun rotate slowly. About every 30
 * seconds (one ROTATION_PERIOD_S) the glow sweeps across the rings
 * — most dramatic when the sun is roughly behind the planet from
 * the camera's POV.
 *
 * T7  LAYER 5: SOFT SHADOWS ON RINGS VIA CONE JITTER
 * ───────────────────────────────────────────────────
 * The planet casts a shadow band on the rings. A naïve test —
 * "shoot one ray from ring point P toward the sun; if blocked by
 * the planet, this point is shadowed" — gives a HARD-EDGED
 * shadow. Real shadows have penumbra because the sun is a SMALL
 * DISC, not a point.
 *
 *      ┌──── sun (disc) ────┐
 *       ╲     ╲    ╲    ╱   ╱     ← rays from POINT P fan out across
 *        ╲     ╲    ╲  ╱   ╱        the sun's ANGULAR SIZE
 *         ╲     ╲    ╲╱   ╱
 *          ╲     ╲   ●   ╱        ● = planet (some rays are blocked,
 *           ╲     ╲     ╱             some pass freely)
 *            ╲     ╲   ╱
 *             ╲     ╲ ╱
 *              ╲     P            P = ring sample point
 *
 *  N samples in the cone:
 *    n_lit = 0
 *    for i = 0 .. N − 1:
 *      offset_i = uniform_disc_2d() · SUN_ANGULAR_RADIUS
 *      sample_dir = normalize(sun_dir + tangent_basis · offset_i)
 *      if not sphere_hit(P, sample_dir): n_lit++
 *    soft = n_lit / N                              ∈ [0, 1]
 *
 * `tangent_basis` is two perpendicular vectors orthogonal to
 * sun_dir — they span the disc the sun's angular radius traces.
 *
 * Result: 0 (fully shadowed) → 1 (fully lit) with a SMOOTH
 * gradient over the penumbra width.
 *
 * Critical detail: the jitter sequence must be DETERMINISTIC for
 * each pixel and frame. We hash (sx, sy, frame) to seed the
 * sample offsets. Without that, the penumbra "boils" frame to
 * frame as random noise rolls through it.
 *
 * Cost: SOFT_SHADOW_SAMPLES = 8 sphere tests per ring pixel. Linear
 * in N; an obvious knob if you need more performance.
 *
 * Read §8.4 soft_shadow.
 *
 * T8  LAYER 6: RING SHADOWS — RINGS ON PLANET VIA CHORD-TO-SUN
 * ─────────────────────────────────────────────────────────────
 * The dual of layer 5. Layer 5 said: "planet blocks the sun from
 * ring points → ring has a soft shadow band". Layer 6 is the
 * symmetric statement: "rings block the sun from planet points →
 * planet's lit hemisphere has dim STREAKS where the ring's bands
 * sit between it and the sun".
 *
 * Real Saturn imagery (Cassini probe shots, March 2006) shows this
 * clearly: parallel dark bands streaking across the lit cloud
 * tops, brightest at the latitudes farthest from the ring shadow,
 * darkest where the dense B ring sits between sun and planet, and
 * with a brighter STRIPE at the latitude where the Cassini gap
 * lets full sunlight through. The shadow streak is literally a
 * projection of the ring density profile onto the planet's surface
 * along the sun direction.
 *
 *      ☀ sun
 *        ╲
 *         ╲
 *      ════╪═══   ← ring annulus (y = 0)
 *           ╲╲     ← chord from sun, intercepted by ring
 *            ╲╲
 *             ╲╲
 *              ●  ← planet hit point P (y < 0, below ring plane)
 *              │
 *           shadow
 *           streak
 *
 * Geometric question: from planet point P, follow the chord toward
 * the sun. Where does it cross the ring plane (y = 0)? If that
 * crossing is INSIDE the annulus, the ring blocks part of the sun;
 * the planet's diffuse + specular at P should be multiplied by
 * (1 - ring_density_at_crossing).
 *
 *   Pseudocode:
 *     if sun_dir.y · P.y >= 0:    sun on same side as P → no shadow
 *     t = -P.y / sun_dir.y                       chord travel to plane
 *     if t < ε:                   ring plane is behind us → no shadow
 *     crossing = P + t · sun_dir
 *     r = sqrt(crossing.x² + crossing.z²)
 *     if r outside [r_in, r_out]: chord misses annulus → no shadow
 *     density = ring_density_at(r, θ)            same formula as the
 *                                                visible ring (§9.3a)
 *     ring_through = 1 − density                 ∈ [0, 1]
 *
 * Then in shade_planet (§9.2):
 *
 *     diffuse  *= ring_through
 *     specular *= ring_through
 *     rim      *= ring_through                   (rim is direct sun too)
 *     ambient + fill UNTOUCHED                   (those are skylight)
 *
 * Why ambient + fill aren't multiplied: those represent diffuse sky
 * radiance reaching P from MANY directions, not the single chord
 * blocked by the rings. Multiplying them would over-darken the
 * shadow streaks and lose the sense of a still-illuminated planet
 * with sharp shadow lines drawn across it.
 *
 * Cost: ONE ray-plane crossing + ONE density lookup per planet
 * pixel. Cheaper than layer 5 (which costs N=8 sphere tests per
 * ring pixel). The shadow's quality matches the ring's density
 * formula automatically — Cassini gap shows up as a brighter stripe
 * in the shadow streaks because density there is close to zero.
 *
 * Read §9.3a ring_density_at + §9.3b ring_transmittance.
 *
 * T9  SURFACE TEXTURE — BANDS, CONTINENTS, SMOOTH CASSINI GAP
 * ────────────────────────────────────────────────────────────
 * The shading we've described so far gives a solid sphere with no
 * surface detail. Three texture layers add believability:
 *
 *   LATITUDE BANDS  modulate albedo by sin(N.y · band_freq + phase).
 *                   On Saturn this is the cream/gold-stripe pattern;
 *                   on Uranus it's the faint blue stratification.
 *                   Frequency and amplitude per pattern.
 *
 *   CONTINENTS      EARTH-R only. Map (longitude, latitude) using
 *                   atan2(N.x, N.z) and N.y, then sample fBm noise.
 *                   Threshold > 0.55 → land, else sea.
 *                   Phase animates so the planet appears to rotate.
 *
 *   CASSINI GAP     Saturn only. A dip in ring density at radius
 *                   ≈ 2.10. We use SMOOTHSTEP, not a hard threshold:
 *                     gap = smoothstep(0, w, |r − cassini_r|)
 *                     density *= CASSINI_DIM + (1 − CASSINI_DIM)·gap
 *                   gap=0 INSIDE the dip → density·0.18 (dim).
 *                   gap=1 OUTSIDE      → density·1.0 (full).
 *                   The smoothstep curve avoids hard pixel-step at the
 *                   gap edges.
 *
 * Read §9.2 (continents + bands) and §9.3 (Cassini smoothstep).
 *
 * T10 CONTINUOUS-RGB PIPELINE → 6×6×6 CUBE + 92-CHAR RAMP
 * ────────────────────────────────────────────────────────
 * The naïve approach: tint each shader output with one of N preset
 * colour-pair indices, switch between them. That's BANDED — smooth
 * gradients show obvious stripes between palette entries.
 *
 * Better: do all colour math in CONTINUOUS RGB (3 floats), tone-
 * map + gamma, THEN quantise to a uniform 6×6×6 RGB cube at draw
 * time. Same approach as `path_tracer.c` and `god_rays_silhouette.c`.
 *
 *   1. Per-pixel shading produces an HDR linear RGB triplet.
 *   2. Reinhard tone-map per channel:  L' = L / (1 + L)
 *      Compresses [0, ∞) into [0, 1) without saturation.
 *   3. Gamma encode 1/2.2: brightens midtones perceptually.
 *   4. Quantise each channel to {0..5}: 6 levels per axis = 216 cube.
 *   5. Pair index = 16 + 36·R5 + 6·G5 + B5  (xterm cube convention).
 *   6. Density character = Rec.601 luma → ramp[index] in 92-char ramp.
 *   7. A_BOLD on luma > 0.85, A_DIM on luma < 0.15.
 *
 * Tone-mapping BEFORE quantisation is critical (T7 in
 * path_tracer.c covers this in detail). Quantising linear HDR
 * puts every pixel above ~0.85 into the same cube cell — no
 * headroom for the sun, the rim glow, the forward-scatter glow.
 * Reinhard spreads the dynamic range over the full cube uniformly.
 *
 * Read §6 paint_cell.
 *
 * T11 THREE SHADE MODES — LIT VS FLAT VS NORMAL
 * ──────────────────────────────────────────────
 * Cycling `m` switches between three shading modes. Each is a
 * different pedagogical lens on the same scene:
 *
 *   LIT       The full pipeline (default). All six shading layers
 *             active. Use this to admire the result.
 *   FLAT      Raw albedo only — no lighting, no rim, no glow, no
 *             shadow. Bands and continents still appear (they're
 *             surface colour, not lighting). Use this to inspect
 *             the texture WITHOUT shading effects. "What does the
 *             planet look like under perfectly diffuse light from
 *             every direction?"
 *   NORMAL    RGB-encoded surface normal: N → (N+1)/2. Each
 *             component remapped from [-1, +1] → [0, 1] so RGB
 *             channels visualise N.x, N.y, N.z.
 *             For the rings (which are flat in y=0) we substitute
 *             (cosθ, density, sinθ) instead — flat green would tell
 *             us nothing; this maps theta around the annulus to a
 *             rainbow hue and density to brightness.
 *             Diagnostic mode: a correct sphere intersection produces
 *             smooth radial gradients; a correct ring produces a
 *             rainbow donut.
 *
 * The two debug modes converge in ONE FRAME because no randomness
 * is involved (NORMAL/FLAT don't call soft_shadow). Switching INTO
 * them is instant; switching back to LIT shows the noise smooth
 * out over a few frames as soft-shadow jitter averages.
 *
 * Pedagogical contrast:
 *   FLAT    → "this is what the materials assert their colour is"
 *   NORMAL  → "this is what the geometry pipeline gives us"
 *   LIT     → "this is the result of all six shading layers"
 *
 * ─────────────────────────────────────────────────────────────────── */

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
#define ASPECT_Y        2.0f       /* terminal cells ~2× taller        */
#define FOV_H           0.55f      /* tan of half horizontal FOV       */

/* §1.3 planet / ring geometry (world units; planet radius = 1) */
#define PLANET_RADIUS         1.00f
#define RING_R_IN_DEFAULT     1.45f
#define RING_R_OUT_DEFAULT    2.55f
#define CASSINI_R_DEFAULT     2.10f
#define CASSINI_W_DEFAULT     0.06f
#define CASSINI_DIM           0.18f
#define RING_BAND_FREQ        18.0f

#define CAM_DIST_DEFAULT      4.7f
#define CAM_HEIGHT_DEFAULT    1.10f

/* §1.4 sun motion */
#define ROTATION_PERIOD_S     30.0f
#define SUN_ELEV_Y            0.45f       /* sun elevation; raise for brighter rings */

/* §1.5 shading constants */
#define AMBIENT_K             0.18f       /* fraction of albedo as ambient   */
#define FILL_K                0.40f       /* cool fill light strength        */
#define SPEC_SHININESS        24.0f
#define SPEC_K                0.40f       /* specular gain on planet         */
#define LIMB_K                0.50f       /* limb-darkening exponent         */
#define RIM_WIDTH             0.28f       /* atmosphere rim cone half-width  */
#define RIM_STRENGTH          0.85f       /* rim brightness multiplier       */
#define GLOW_SHARP            3.0f        /* forward-scatter exponent        */
#define GLOW_GAIN             1.20f       /* forward-scatter brightness      */

/* §1.6 soft shadow on rings */
#define SOFT_SHADOW_SAMPLES   8
#define SUN_ANGULAR_RADIUS    0.05f       /* radians of sun cone (≈ 3°)      */

/* §1.7 sub-pixel AA */
enum { SPP_MIN_VAL = 1, SPP_MAX_VAL = 4, SPP_DEF_VAL = 2 };

/* §1.7b shade mode (cycled with 'm').
 *   LIT     — full RGB pipeline: ambient + diffuse + fill + spec
 *             + limb darkening + atmospheric rim + forward-scatter
 *             + soft shadow. Default.
 *   FLAT    — raw albedo only, no lighting. Bands and continents
 *             still appear (they're surface colour, not lighting).
 *             Use this to inspect the texture without shading
 *             effects — easiest way to "see the planet without
 *             lighting."
 *   NORMAL  — RGB-encoded surface normal (diagnostic). Each
 *             component remapped from [-1,+1] → [0,1] so the three
 *             RGB channels visualise N.x, N.y, N.z. Standard
 *             debugging view across the raytracing folder.
 */
typedef enum {
    SHADE_LIT    = 0,
    SHADE_FLAT   = 1,
    SHADE_NORMAL = 2,
    SHADE_N      = 3,
} ShadeMode;

static const char *shade_mode_name(ShadeMode m)
{
    switch (m) {
    case SHADE_LIT:    return "LIT   ";
    case SHADE_FLAT:   return "FLAT  ";
    case SHADE_NORMAL: return "NORMAL";
    default:           return "?     ";
    }
}

/* §1.8 stars */
#define STAR_DENSITY          180
#define STAR_TWINKLE_HZ       0.4f

/* §1.9 ncurses pair IDs */
enum {
    PAIR_HUD          = 1,
    PAIR_HINT         = 2,
    PAIR_FLASH        = 3,
    PAIR_CUBE_BASE    = 8,            /* + 0..215 = 6×6×6 cube           */
};

/* §1.10 ASCII density ramp — Bourke 92 chars sparse → dense */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

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

/* ── §3 math (V3 + RGB helpers) ──────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3 (float x, float y, float z) { return (V3){x,y,z}; }
static inline V3    v3_add  (V3 a, V3 b)   { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3_sub  (V3 a, V3 b)   { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3_scl  (V3 a, float s){ return v3(a.x*s,   a.y*s,   a.z*s);   }
static inline V3    v3_mul  (V3 a, V3 b)   { return v3(a.x*b.x, a.y*b.y, a.z*b.z); }
static inline float v3_dot  (V3 a, V3 b)   { return a.x*b.x + a.y*b.y + a.z*b.z;   }
static inline V3    v3_cross(V3 a, V3 b)
{
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline V3 v3_norm(V3 a)
{
    float l = sqrtf(v3_dot(a,a));
    if (l < 1e-12f) return v3(0,0,0);
    return v3_scl(a, 1.f / l);
}
static inline V3 v3_lerp(V3 a, V3 b, float t)
{
    return v3_add(v3_scl(a, 1.f - t), v3_scl(b, t));
}
static inline V3 v3_max0(V3 a)
{
    return v3(a.x<0?0:a.x, a.y<0?0:a.y, a.z<0?0:a.z);
}

static inline float clamp01(float x)
{
    if (x < 0.f) return 0.f;
    if (x > 1.f) return 1.f;
    return x;
}

static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* Smoothstep — 0 below e0, 1 above e1, smooth Hermite curve in between. */
static inline float smoothstep(float e0, float e1, float x)
{
    float t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.f - 2.f * t);
}

/* ── §4 noise (Perlin + fBm) ─────────────────────────────────────────── */

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
    float total = 0, amp = 1, freq = 1, max_amp = 0;
    for (int o = 0; o < 4; o++) {
        total += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp *= 0.5f; freq *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* hash3 — drives star placement + per-frame randomness. */
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

/* hash → uniform float in [0,1) */
static inline float hash01(uint32_t h)
{
    return (float)(h & 0xFFFFFFu) * (1.f / (float)0x1000000u);
}

/* ── §5 themes (RGB triplets, NOT pair indices) ─────────────────────── */

/*
 * Each theme provides RGB colours per role. The shader blends them in
 * continuous RGB space; the §6 paint function quantises to the 6×6×6
 * xterm cube at draw time. This is the central change vs the old
 * ramp-slot pipeline — it gives ~20 000 effective shades per cell
 * (216 cube pairs × 92 ramp glyphs) instead of 8.
 */
typedef struct {
    const char *name;
    V3 planet_base;       /* base albedo                                  */
    V3 planet_band_tint;  /* tint added to dark/light bands               */
    V3 ring_base;         /* ring base albedo                             */
    V3 sun_col;           /* warm key light                               */
    V3 fill_col;          /* cool fill light from −sun (skylight)         */
    V3 rim_col;           /* atmospheric-rim warm tint                    */
    V3 spec_col;          /* specular tint                                */
    V3 sky_col;           /* dim background space colour                  */
    V3 land_col;          /* RINGED-EARTH only                            */
    V3 sea_col;           /* RINGED-EARTH only                            */
} Theme;

#define N_THEMES 8

static const Theme themes[N_THEMES] = {
    /* SATURN — cream + golden bands, ivory rings */
    { "SATURN",
      {0.92f, 0.84f, 0.66f}, {0.55f, 0.45f, 0.28f},
      {0.85f, 0.79f, 0.66f},
      {1.00f, 0.92f, 0.78f}, {0.32f, 0.42f, 0.58f},
      {1.00f, 0.62f, 0.30f}, {1.00f, 0.95f, 0.85f},
      {0.04f, 0.05f, 0.08f},
      {0.50f, 0.45f, 0.30f}, {0.10f, 0.20f, 0.45f} },

    /* MARS — rust planet, dust rings */
    { "MARS",
      {0.78f, 0.45f, 0.28f}, {0.50f, 0.22f, 0.12f},
      {0.72f, 0.55f, 0.42f},
      {1.00f, 0.86f, 0.65f}, {0.32f, 0.22f, 0.40f},
      {1.00f, 0.40f, 0.18f}, {1.00f, 0.92f, 0.78f},
      {0.05f, 0.04f, 0.06f},
      {0.55f, 0.30f, 0.18f}, {0.20f, 0.15f, 0.10f} },

    /* OCEAN — blue planet, silver rings */
    { "OCEAN",
      {0.32f, 0.55f, 0.85f}, {0.10f, 0.28f, 0.55f},
      {0.78f, 0.85f, 0.92f},
      {1.00f, 0.95f, 0.85f}, {0.55f, 0.65f, 0.85f},
      {0.45f, 0.85f, 1.00f}, {1.00f, 1.00f, 1.00f},
      {0.04f, 0.05f, 0.10f},
      {0.30f, 0.65f, 0.30f}, {0.10f, 0.30f, 0.65f} },

    /* FOREST — green planet, pale rings */
    { "FOREST",
      {0.28f, 0.62f, 0.30f}, {0.10f, 0.32f, 0.12f},
      {0.78f, 0.85f, 0.62f},
      {1.00f, 0.92f, 0.72f}, {0.42f, 0.62f, 0.55f},
      {0.55f, 1.00f, 0.45f}, {1.00f, 0.95f, 0.80f},
      {0.04f, 0.06f, 0.05f},
      {0.40f, 0.62f, 0.20f}, {0.10f, 0.28f, 0.55f} },

    /* FIRE — molten planet, ember rings */
    { "FIRE",
      {0.95f, 0.42f, 0.18f}, {0.65f, 0.10f, 0.05f},
      {0.92f, 0.55f, 0.28f},
      {1.00f, 0.78f, 0.40f}, {0.45f, 0.18f, 0.40f},
      {1.00f, 0.30f, 0.10f}, {1.00f, 0.85f, 0.65f},
      {0.06f, 0.04f, 0.04f},
      {0.85f, 0.42f, 0.15f}, {0.40f, 0.10f, 0.05f} },

    /* ARCTIC — pale icy blue planet, white rings */
    { "ARCTIC",
      {0.78f, 0.88f, 0.95f}, {0.45f, 0.62f, 0.78f},
      {0.92f, 0.96f, 1.00f},
      {1.00f, 0.95f, 0.88f}, {0.55f, 0.70f, 0.92f},
      {0.85f, 0.95f, 1.00f}, {1.00f, 1.00f, 1.00f},
      {0.04f, 0.06f, 0.10f},
      {0.78f, 0.88f, 0.92f}, {0.30f, 0.55f, 0.78f} },

    /* VIOLET — magenta gas giant, pink rings */
    { "VIOLET",
      {0.62f, 0.32f, 0.78f}, {0.30f, 0.10f, 0.45f},
      {0.85f, 0.62f, 0.92f},
      {1.00f, 0.85f, 1.00f}, {0.42f, 0.55f, 0.78f},
      {1.00f, 0.45f, 0.85f}, {1.00f, 0.92f, 1.00f},
      {0.05f, 0.03f, 0.08f},
      {0.55f, 0.30f, 0.62f}, {0.18f, 0.10f, 0.30f} },

    /* GOLD — molten gold planet, brass rings */
    { "GOLD",
      {0.95f, 0.78f, 0.32f}, {0.60f, 0.42f, 0.10f},
      {0.92f, 0.78f, 0.42f},
      {1.00f, 0.92f, 0.65f}, {0.42f, 0.30f, 0.55f},
      {1.00f, 0.55f, 0.18f}, {1.00f, 0.95f, 0.78f},
      {0.05f, 0.04f, 0.05f},
      {0.78f, 0.62f, 0.22f}, {0.30f, 0.20f, 0.08f} },
};

/* ── §6 color (cube allocation + RGB → cube paint) ───────────────────── */

static int g_256;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);
    if (g_256) {
        /* Pre-allocate the 6×6×6 RGB cube as 216 pairs. */
        for (int i = 0; i < 216; i++)
            init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        /* 8-colour fallback — coarse, no cube. */
        init_pair(PAIR_CUBE_BASE, COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,       COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH,     COLOR_YELLOW, -1);
    }
}

/*
 * paint_cell — full RGB → terminal pipeline.
 *
 *   1. Reinhard tone-map per channel:  L' = L / (1 + L)
 *   2. Gamma encode 1/2.2:               sRGB-perceptual.
 *   3. Quantise to 6×6×6 cube → pair id.
 *   4. Compute Rec.601 luminance of the encoded RGB.
 *   5. Pick density glyph from luminance.
 *   6. Pick A_BOLD / A_DIM / A_NORMAL from luminance.
 *
 * Steps 1-2 must run BEFORE step 3 — quantising linear HDR puts
 * almost every pixel in one cube cell. Tone-mapping first opens up
 * the dynamic range so colours spread across the cube.
 */
static void paint_cell(int sx, int sy, V3 col)
{
    /* Tone-map + gamma. */
    float r = gamma_enc(reinhard(col.x));
    float g = gamma_enc(reinhard(col.y));
    float b = gamma_enc(reinhard(col.z));

    if (g_256) {
        int r5 = (int)(r * 5.f + 0.5f); if (r5 > 5) r5 = 5; if (r5 < 0) r5 = 0;
        int g5 = (int)(g * 5.f + 0.5f); if (g5 > 5) g5 = 5; if (g5 < 0) g5 = 0;
        int b5 = (int)(b * 5.f + 0.5f); if (b5 > 5) b5 = 5; if (b5 < 0) b5 = 0;
        int pair = PAIR_CUBE_BASE + r5*36 + g5*6 + b5;

        float luma = 0.2126f*r + 0.7152f*g + 0.0722f*b;
        int   ri   = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
        if (ri < 0)         ri = 0;
        if (ri >= RAMP_LEN) ri = RAMP_LEN - 1;
        char ch = k_ramp[ri];

        int attr = (luma > 0.85f) ? A_BOLD
                 : (luma < 0.15f) ? A_DIM
                 :                  A_NORMAL;

        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy, sx, (chtype)(unsigned char)ch);
        attroff(COLOR_PAIR(pair) | attr);
    } else {
        /* 8-colour fallback: density only. */
        float luma = 0.2126f*r + 0.7152f*g + 0.0722f*b;
        int   ri   = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
        if (ri < 0)         ri = 0;
        if (ri >= RAMP_LEN) ri = RAMP_LEN - 1;
        attron(COLOR_PAIR(PAIR_CUBE_BASE));
        mvaddch(sy, sx, (chtype)(unsigned char)k_ramp[ri]);
        attroff(COLOR_PAIR(PAIR_CUBE_BASE));
    }
}

/* ── §7 patterns ─────────────────────────────────────────────────────── */

typedef enum {
    PATTERN_SATURN = 0,
    PATTERN_URANUS = 1,
    PATTERN_EARTH  = 2,
    PATTERN_EXO    = 3,
    N_PATTERNS     = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SATURN: return "SATURN ";
    case PATTERN_URANUS: return "URANUS ";
    case PATTERN_EARTH:  return "EARTH-R";
    case PATTERN_EXO:    return "EXO    ";
    default:             return "?      ";
    }
}

typedef struct {
    float ring_r_in, ring_r_out;
    float cam_height;
    float band_freq, band_amp;
    bool  has_continents, has_cassini;
    float cassini_r, cassini_w;
    float ring_density_floor;       /* base opacity baseline 0..1 */
} PatternParams;

/* ── §8 raytrace primitives ─────────────────────────────────────────── */

/* §8.1 ── ray-sphere ──────────────────────────────────────────────── */

/*
 * ray_sphere — analytic intersection via the textbook quadratic.
 *
 * Setup: |O + t·D − C|² = r². With unit-length ray direction D this
 * expands to t² + 2·b·t + c = 0 where:
 *
 *     b = D · (O − C)
 *     c = |O − C|² − r²
 *
 * Discriminant disc = b² − c. Roots t = −b ± √disc.
 *
 * We pick the nearest root that's beyond a small epsilon (front face
 * for an outside ray; far root only if the ray STARTS inside the
 * sphere — can't happen for the planet's primary view ray, but the
 * logic is here for safety).
 *
 * Inputs : ray_origin, ray_dir   (unit-length), centre, radius.
 * Output : *out_t = first valid t > ε.
 * Returns: true on hit, false on miss (disc < 0 or all t ≤ ε).
 *
 * See tutorial T2.
 */
static bool ray_sphere(V3 ro, V3 rd, V3 center, float r, float *out_t)
{
    V3    oc   = v3_sub(ro, center);
    float b    = v3_dot(oc, rd);
    float c    = v3_dot(oc, oc) - r * r;
    float disc = b * b - c;
    if (disc < 0.f) return false;
    float sq = sqrtf(disc);
    float t  = -b - sq;
    if (t < 1e-3f) t = -b + sq;
    if (t < 1e-3f) return false;
    *out_t = t;
    return true;
}

/* §8.2 ── ray-ring (plane y=0 clipped to annulus) ─────────────────── */

/*
 * ray_ring — analytic intersection of a ray with a flat annulus in
 *            the plane y=0, clipped to inner/outer radii.
 *
 * Pseudocode:
 *   1. If |ray_dir.y| < ε:   parallel to plane → MISS.
 *   2. t = -ray_origin.y / ray_dir.y           (where ray crosses y=0)
 *   3. If t < ε:             behind camera → MISS.
 *   4. hit = ray_origin + t · ray_dir
 *   5. r² = hit.x² + hit.z²                    (squared 2-D radius)
 *   6. If r² < r_in² or r² > r_out²:           outside annulus → MISS.
 *   7. Otherwise: HIT, write *out_t and *out_hit.
 *
 * The squared-radius bound check avoids a sqrt; it's algebraically
 * equivalent to comparing r against r_in and r_out.
 *
 * The annulus spans [r_in, r_out] on the y=0 plane — that's the
 * entire ring system geometry. Density profile, Cassini gap, bands,
 * forward scatter, and soft shadow are all SHADING applied on top of
 * this trivially cheap intersection (§9.3 shade_ring).
 *
 * See tutorial T2.
 */
static bool ray_ring(V3 ro, V3 rd, float r_in, float r_out,
                     float *out_t, V3 *out_hit)
{
    if (fabsf(rd.y) < 1e-5f) return false;
    float t = -ro.y / rd.y;
    if (t < 1e-3f) return false;
    V3 hit = v3_add(ro, v3_scl(rd, t));
    float r2 = hit.x * hit.x + hit.z * hit.z;
    if (r2 < r_in  * r_in)  return false;
    if (r2 > r_out * r_out) return false;
    *out_t   = t;
    *out_hit = hit;
    return true;
}

/* §8.3 ── hard shadow (cheap, used for planet self-shadow) ────────── */

static bool hard_shadow_sphere(V3 origin, V3 dir, V3 sphere_c, float sphere_r)
{
    V3    o    = v3_add(origin, v3_scl(dir, 1e-3f));
    V3    oc   = v3_sub(o, sphere_c);
    float b    = v3_dot(oc, dir);
    float c    = v3_dot(oc, oc) - sphere_r * sphere_r;
    float disc = b * b - c;
    if (disc < 0.f) return false;
    float t = -b - sqrtf(disc);
    return t > 1e-3f;
}

/* §8.4 ── soft shadow (N rays in a cone around the sun) ──────────── */

/*
 * Sample N directions in a small angular cone around `sun_dir` (sun
 * is treated as a small disc, not a point), test each against the
 * planet, and return the FRACTION that reach the sun unobstructed.
 *
 * Result is 0 (fully shadowed) → 1 (fully lit) with a smooth gradient
 * in between → that's the penumbra. The jitter sequence is a
 * deterministic function of (sx, sy, frame) so the same pixel jitters
 * the same way each frame and the penumbra doesn't "boil".
 */
static float soft_shadow(V3 origin, V3 sun_dir, V3 sphere_c, float sphere_r,
                         int sx, int sy, int frame)
{
    /* Build a tangent basis around sun_dir to offset within the cone. */
    V3 up_seed = (fabsf(sun_dir.y) < 0.9f) ? v3(0,1,0) : v3(1,0,0);
    V3 tx = v3_norm(v3_cross(up_seed, sun_dir));
    V3 ty = v3_cross(sun_dir, tx);

    int n_lit = 0;
    for (int i = 0; i < SOFT_SHADOW_SAMPLES; i++) {
        uint32_t h = hash3(sx, sy, frame * SOFT_SHADOW_SAMPLES + i);
        float r1 = hash01(h);
        float r2 = hash01(h ^ 0xA5A5A5A5u);
        float ang = 2.f * (float)M_PI * r1;
        float rad = sqrtf(r2) * SUN_ANGULAR_RADIUS;
        V3 offset = v3_add(v3_scl(tx, cosf(ang) * rad),
                           v3_scl(ty, sinf(ang) * rad));
        V3 dir = v3_norm(v3_add(sun_dir, offset));
        if (!hard_shadow_sphere(origin, dir, sphere_c, sphere_r))
            n_lit++;
    }
    return (float)n_lit / (float)SOFT_SHADOW_SAMPLES;
}

/* ── §9 shading ──────────────────────────────────────────────────────── */

/* §9.1 ── limb darkening + atmospheric rim helpers ──────────────────── */

/*
 * Limb darkening: surfaces seen at grazing angles return less radiance
 * because the atmosphere absorbs more of the path through it. Modelled
 * as `pow(NdotV, k)` with k ∈ [0.4, 0.8] (gentle real atmospheres).
 *
 * Atmospheric rim: at very small NdotV (silhouette), the lit edge of
 * the planet glows warmly because sunlight scatters tangentially
 * through the atmosphere. Triangle-shaped falloff parameterised by
 * RIM_WIDTH (cone half-width) and RIM_STRENGTH (peak brightness).
 *
 * Both are pure functions of N, V, L — no extra rays.
 */
static float limb_darken(float NdotV)
{
    return powf(clamp01(NdotV), LIMB_K);
}

static V3 atmospheric_rim(float NdotV, float NdotL, V3 rim_col)
{
    if (NdotL <= 0.f || NdotV >= RIM_WIDTH) return v3(0,0,0);
    float t = 1.f - NdotV / RIM_WIDTH;       /* 1 at silhouette, 0 inside */
    float rim = t * t * NdotL * RIM_STRENGTH;
    return v3_scl(rim_col, rim);
}

/* Forward declarations for ring helpers used below (defined in §9.3a/b). */
static float ring_density_at  (float r, float theta,
                               const PatternParams *pp, float seed_phase);
static float ring_transmittance(V3 P, V3 sun_dir,
                                const PatternParams *pp, float seed_phase);

/* §9.2 ── shade_planet ───────────────────────────────────────────────── */

/*
 * Full planet shader in continuous RGB:
 *   ambient                 — albedo × small constant
 *   diffuse (Lambert)       — albedo × NdotL × sun_col
 *   fill (cool sky)         — albedo × NdotF × fill_col × FILL_K
 *   specular (subtle)       — pow(R·V, n) × spec_col × NdotL × SPEC_K
 *   limb darkening          — multiply final by pow(NdotV, LIMB_K)
 *   atmospheric rim         — add warm rim at silhouette
 *   bands / continents      — modulate albedo before all of the above
 */
static V3 shade_planet(const Theme *th, const PatternParams *pp,
                       float continent_phase, float seed_phase,
                       V3 hit, V3 view_dir, V3 sun_dir, Pattern pat,
                       ShadeMode mode)
{
    V3 N = v3_norm(hit);
    V3 fill_dir = v3_norm(v3_scl(sun_dir, -1.f));   /* opposite to sun  */

    /* Compute albedo: latitude bands or continent map. */
    V3 albedo = th->planet_base;
    if (pp->band_amp > 0.001f) {
        float band = sinf(N.y * pp->band_freq + seed_phase * 1.7f);
        float t = 0.5f + 0.5f * band;       /* 0..1 mix factor */
        t *= pp->band_amp * 2.0f;           /* amplitude       */
        if (t > 1.f) t = 1.f;
        albedo = v3_lerp(albedo, th->planet_band_tint, t);
    }
    if (pp->has_continents) {
        float u = atan2f(N.x, N.z) / (float)M_PI;
        float v = N.y;
        float land = fbm2(u * 3.5f + continent_phase,
                          v * 2.5f + continent_phase * 0.7f);
        bool is_land = (land > 0.55f);
        albedo = (is_land || pat != PATTERN_EARTH)
               ? th->land_col : th->sea_col;
    }

    /* Diagnostic / non-lit modes — return early. */
    if (mode == SHADE_NORMAL) {
        return v3(N.x*0.5f + 0.5f, N.y*0.5f + 0.5f, N.z*0.5f + 0.5f);
    }
    if (mode == SHADE_FLAT) {
        /* Raw albedo at uniform brightness. Bands and continents
         * still differentiate surface; no lighting at all. */
        return albedo;
    }

    /* Light geometry. */
    float NdotL = v3_dot(N, sun_dir);    if (NdotL < 0.f) NdotL = 0.f;
    float NdotF = v3_dot(N, fill_dir);   if (NdotF < 0.f) NdotF = 0.f;
    float NdotV = fabsf(v3_dot(N, view_dir));

    /* Phong specular. R = reflect(-L, N). */
    V3 R = v3_sub(v3_scl(N, 2.f * v3_dot(N, sun_dir)), sun_dir);
    float spec = powf(fmaxf(0.f, v3_dot(R, view_dir)), SPEC_SHININESS)
               * (NdotL > 0.f ? 1.f : 0.f);

    V3 ambient  = v3_scl(albedo, AMBIENT_K);
    V3 diffuse  = v3_scl(v3_mul(albedo, th->sun_col), NdotL);
    V3 fill     = v3_scl(v3_mul(albedo, th->fill_col), NdotF * FILL_K);
    V3 specular = v3_scl(th->spec_col, spec * SPEC_K);

    /* LAYER 6 (T8) — RINGS CAST SHADOW ON PLANET.
     *
     * From the planet hit point, the chord to the sun may pass through
     * the ring annulus. If it does, the planet sees a dimmer sun:
     * direct-sun components (diffuse, specular, atmospheric rim) get
     * multiplied by the ring transmittance. Ambient and fill are
     * skylight, not direct sun, so they're NOT multiplied. */
    float ring_through = ring_transmittance(hit, sun_dir, pp, seed_phase);
    diffuse  = v3_scl(diffuse,  ring_through);
    specular = v3_scl(specular, ring_through);

    /* Limb darkening — apply ONLY to the reflected components (diffuse +
     * specular). Physically, limb darkening models atmosphere absorbing
     * light that REFLECTS off the surface as it travels to the camera at
     * grazing angles. Ambient and fill represent skylight reaching the
     * surface from any direction, independent of the outgoing path —
     * those should NOT be attenuated. Multiplying everything (the older
     * formulation) made the silhouette go pitch-black; this form lets
     * ambient + fill survive at the edge so the silhouette has a visible
     * body colour before the rim glow takes over. */
    float limb = limb_darken(NdotV);
    V3 atmo_attenuated = v3_scl(v3_add(diffuse, specular), limb);
    V3 col = v3_add(v3_add(ambient, fill), atmo_attenuated);

    /* Atmospheric rim — additive warm halo on lit silhouette edge.
     * Also dimmed by ring_through (the rim is direct sun scattered
     * tangentially through the atmosphere — same source as diffuse). */
    V3 rim = atmospheric_rim(NdotV, NdotL, th->rim_col);
    col = v3_add(col, v3_scl(rim, ring_through));

    return col;
}

/* §9.3a ── ring_density_at (single source of truth) ──────────────── */

/*
 * Smooth ring density at polar coords (r, θ) on the ring plane. ∈ [0,1].
 *   r ∈ [0, ∞)        radial distance from planet centre
 *   θ ∈ [-π, π]       azimuth around the ring
 *
 * Composes three profiles:
 *   1. Soft radial edges    — annulus fades over 0.04 m at r_in / r_out
 *                              instead of cutting hard at the boundary.
 *   2. Bands                — sin(r · BAND_FREQ + θ · 0.4 + phase) drift
 *                              the density over the ring's radial extent.
 *   3. Smooth Cassini gap   — smoothstep dip at the gap radius.
 *
 * Used by BOTH shade_ring (for direct ring colour, §9.3) AND
 * ring_transmittance (for shadow cast onto the planet, §9.3b). Single
 * source of truth — the planet's shadow streaks have the same banding
 * and Cassini dip as the visible ring.
 */
static float ring_density_at(float r, float theta,
                             const PatternParams *pp, float seed_phase)
{
    float edge_in  = smoothstep(pp->ring_r_in,  pp->ring_r_in  + 0.04f, r);
    float edge_out = 1.f - smoothstep(pp->ring_r_out - 0.04f, pp->ring_r_out, r);
    float density  = pp->ring_density_floor
                   + (1.0f - pp->ring_density_floor)
                     * (0.5f + 0.5f * sinf(r * RING_BAND_FREQ
                                           + theta * 0.4f
                                           + seed_phase));
    density *= edge_in * edge_out;

    if (pp->has_cassini) {
        float gap = smoothstep(0.f, pp->cassini_w, fabsf(r - pp->cassini_r));
        density *= CASSINI_DIM + (1.f - CASSINI_DIM) * gap;
    }
    return density;
}

/* §9.3b ── ring_transmittance (rings cast shadow on planet, T8) ──── */

/*
 * Fraction of direct sunlight reaching planet hit point P after
 * passing through (or missing) the ring annulus.
 *
 * Pseudocode:
 *   if sun and P on the SAME side of the ring plane:   return 1.0
 *     (chord toward sun never crosses the ring)
 *   if sun_dir.y near zero:                            return 1.0
 *     (chord parallel to the rings — degenerate)
 *   t = -P.y / sun_dir.y                  distance from P to ring plane
 *   if t < ε:                              return 1.0
 *   crossing = P + t · sun_dir
 *   r² = crossing.x² + crossing.z²
 *   if r outside annulus:                  return 1.0
 *   density = ring_density_at(r, θ, ...)   same formula as shade_ring
 *   return 1 - density
 *
 * ∈ [0, 1].  1.0 = unshadowed, 0.0 = fully blocked by an opaque band.
 *
 * Multiplies the planet's diffuse + specular + rim — all three are
 * direct-sun components and the ring sits between sun and P along
 * the sun chord. Ambient and fill are NOT multiplied (they don't
 * trace back through the rings).
 */
static float ring_transmittance(V3 P, V3 sun_dir,
                                const PatternParams *pp,
                                float seed_phase)
{
    if (sun_dir.y * P.y >= 0.f)            return 1.f;
    if (fabsf(sun_dir.y) < 1e-5f)          return 1.f;

    float t = -P.y / sun_dir.y;
    if (t < 1e-3f)                         return 1.f;

    V3 crossing = v3_add(P, v3_scl(sun_dir, t));
    float r2 = crossing.x * crossing.x + crossing.z * crossing.z;
    float r_in2  = pp->ring_r_in  * pp->ring_r_in;
    float r_out2 = pp->ring_r_out * pp->ring_r_out;
    if (r2 < r_in2 || r2 > r_out2)         return 1.f;

    float r     = sqrtf(r2);
    float theta = atan2f(crossing.z, crossing.x);
    return 1.f - ring_density_at(r, theta, pp, seed_phase);
}

/* §9.3 ── shade_ring (forward-scatter + soft shadow + smooth Cassini) ─ */

/*
 * Continuous ring shader:
 *   density(r, θ)           — via ring_density_at (§9.3a)
 *   soft shadow             — N-sample penumbra by the planet
 *   diffuse (|sun.y|·shad)  — flat-plate Lambertian, double-sided
 *   forward-scatter glow    — backlight × (1-density) × sun_col
 *   ambient                 — base brightness floor
 */
static V3 shade_ring(const Theme *th, const PatternParams *pp,
                     float seed_phase,
                     V3 hit, V3 view_dir, V3 sun_dir,
                     int sx, int sy, int frame, ShadeMode mode)
{
    float r       = sqrtf(hit.x * hit.x + hit.z * hit.z);
    float theta   = atan2f(hit.z, hit.x);
    float density = ring_density_at(r, theta, pp, seed_phase);

    /* Diagnostic / non-lit modes — return early.
     *
     * The flat ring normal (0,1,0) would render as one solid colour, so
     * for NORMAL mode we encode (cosθ, density, sinθ) instead — that
     * gives a rainbow around the annulus (theta sweeping the hue) plus
     * brightness gradient by density. Much more useful for "see the
     * ring structure" than a flat green strip. */
    if (mode == SHADE_NORMAL) {
        return v3(cosf(theta) * 0.5f + 0.5f,
                  density,
                  sinf(theta) * 0.5f + 0.5f);
    }
    if (mode == SHADE_FLAT) {
        return v3_scl(th->ring_base, density);
    }

    /* Soft shadow — penumbra of planet on this ring point. */
    V3 planet_c = v3(0,0,0);
    float shadow = soft_shadow(hit, sun_dir, planet_c, PLANET_RADIUS,
                               sx, sy, frame);

    /* Diffuse (rings are flat, double-sided) — abs of sun.y component.
     * Multiply by shadow factor. The (0.25 + 0.75·|sun.y|) shape gives
     * the rings a 25% diffuse FLOOR even when the sun is near the ring
     * plane, so they never go pitch-dark. Real ring particles aren't
     * paper-thin perfect Lambertians; they pick up some glow from the
     * lit sides of neighbouring particles regardless of sun elevation. */
    float diffuse = (0.25f + 0.75f * fabsf(sun_dir.y)) * shadow;

    /* Forward-scatter glow — sparse parts glow when sun is roughly
     * behind the ring relative to the camera. */
    float backlight = -v3_dot(sun_dir, view_dir);   /* > 0 when sun behind */
    if (backlight < 0.f) backlight = 0.f;
    float glow = powf(backlight, GLOW_SHARP) * (1.f - density) * GLOW_GAIN;

    /* Combine into RGB. */
    V3 base    = v3_scl(th->ring_base, density);
    V3 ambient = v3_scl(base, AMBIENT_K * 1.5f);
    V3 lit     = v3_scl(v3_mul(base, th->sun_col), diffuse);
    V3 fwd     = v3_scl(th->sun_col, glow * density);  /* glow shows ring dust */

    /* Plus a faint forward-scatter even where density is zero (haze
     * around the ring) — kept very dim. */
    V3 haze    = v3_scl(th->sun_col, glow * 0.15f);

    return v3_add(ambient, v3_add(lit, v3_add(fwd, haze)));
}

/* §9.4 ── shade_space (background + stars) ──────────────────────────── */

static V3 shade_space(const Theme *th, int sx, int sy,
                      int star_seed, float time_secs)
{
    V3 base = th->sky_col;
    uint32_t h = hash3(sx, sy, star_seed);
    if ((h % STAR_DENSITY) == 0u) {
        float phase = hash01(h >> 16) * 2.f * (float)M_PI;
        float tw = 0.5f + 0.5f * sinf(2.f * (float)M_PI
                                       * STAR_TWINKLE_HZ * time_secs + phase);
        if (tw > 0.40f) {
            /* Star RGB hue per hash — warm, cool, or pure white.
             * Brightness scales with twinkle phase. */
            float warm = hash01(h ^ 0xC0FFEEu);
            V3 star_col;
            if      (warm > 0.66f) star_col = v3(1.0f, 0.85f, 0.65f); /* warm */
            else if (warm > 0.33f) star_col = v3(0.85f, 0.92f, 1.0f); /* cool */
            else                    star_col = v3(1.0f, 1.0f, 1.0f);   /* pure */
            float br = 0.4f + 0.7f * tw;
            return v3_add(base, v3_scl(star_col, br));
        }
    }
    return base;
}

/* ── §10 scene ───────────────────────────────────────────────────────── */

typedef struct {
    bool          paused;
    int           speed;
    int           current_theme;
    Pattern       current_pattern;
    PatternParams pp;
    float         time_secs;
    float         seed_phase;
    float         continent_phase;
    int           star_seed;
    int           frame;
    int           spp;
    ShadeMode     shade_mode;
} Scene;

static void pattern_set(Scene *s, Pattern p)
{
    s->current_pattern = p;
    PatternParams *pp = &s->pp;
    pp->ring_r_in        = RING_R_IN_DEFAULT;
    pp->ring_r_out       = RING_R_OUT_DEFAULT;
    pp->cam_height       = CAM_HEIGHT_DEFAULT;
    pp->band_freq        = 8.0f;
    pp->band_amp         = 0.18f;
    pp->has_continents   = false;
    pp->has_cassini      = false;
    pp->cassini_r        = CASSINI_R_DEFAULT;
    pp->cassini_w        = CASSINI_W_DEFAULT;
    pp->ring_density_floor = 0.55f;

    switch (p) {
    case PATTERN_SATURN:
        pp->band_freq = 12.0f;
        pp->band_amp  = 0.20f;
        pp->has_cassini = true;
        pp->ring_r_in   = 1.45f;
        pp->ring_r_out  = 2.65f;
        pp->cassini_r   = 2.10f;
        pp->cassini_w   = 0.06f;
        pp->cam_height  = 1.10f;
        break;
    case PATTERN_URANUS:
        pp->band_freq = 4.0f;
        pp->band_amp  = 0.06f;
        pp->ring_r_in  = 1.30f;
        pp->ring_r_out = 1.90f;
        pp->ring_density_floor = 0.35f;
        pp->cam_height = 0.45f;
        break;
    case PATTERN_EARTH:
        pp->band_freq = 0.0f;
        pp->band_amp  = 0.0f;
        pp->has_continents = true;
        pp->ring_r_in  = 1.40f;
        pp->ring_r_out = 2.00f;
        pp->ring_density_floor = 0.45f;
        pp->cam_height = 1.20f;
        break;
    case PATTERN_EXO: {
        int hsp = (int)(s->seed_phase * 100.0f);
        pp->band_freq      = 4.0f  + ((float)(hsp        % 100) * 0.18f);
        pp->band_amp       = 0.10f + ((float)((hsp >> 2) &  7) * 0.04f);
        pp->ring_r_in      = 1.35f + ((float)((hsp >> 4) &  7) * 0.04f);
        pp->ring_r_out     = pp->ring_r_in + 0.6f
                           + ((float)((hsp >> 6) & 7) * 0.10f);
        pp->has_cassini    = (hsp & 1) != 0;
        pp->cassini_r      = (pp->ring_r_in + pp->ring_r_out) * 0.5f;
        pp->ring_density_floor = 0.40f;
        pp->cam_height     = 0.7f + ((float)((hsp >> 8) & 7) * 0.10f);
        break;
    }
    case N_PATTERNS: break;
    }
}

static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->seed_phase * 100.0f), 0xC0FFEE);
    s->seed_phase      = hash01(h) * 2.f * (float)M_PI;
    s->continent_phase = hash01(h >> 16) * 8.0f;
    s->star_seed       = (int)(h ^ 0x5A5A5A5Au);
    perm_shuffle(s->star_seed);
    pattern_set(s, s->current_pattern);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->speed           = SPEED_DEF;
    s->seed_phase      = 1.0f;
    s->continent_phase = 3.0f;
    s->star_seed       = 0xDECAF;
    s->spp             = SPP_DEF_VAL;
    s->shade_mode      = SHADE_LIT;
    perm_shuffle(s->star_seed);
    pattern_set(s, PATTERN_SATURN);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

static V3 scene_sun_dir(const Scene *s)
{
    float omega = 2.f * (float)M_PI / ROTATION_PERIOD_S;
    float az    = s->time_secs * omega + s->seed_phase;
    return v3_norm(v3(cosf(az), SUN_ELEV_Y, sinf(az)));
}

/* ── §11 render ──────────────────────────────────────────────────────── */

typedef struct {
    V3 pos, fwd, right, up;
    float fov_h, fov_v;
    int cols, rows;
} Camera;

/*
 * camera_make — build the camera basis (right, up, forward) from a
 *               look-at + height parameterisation.
 *
 * The camera sits at (0, cam_height, -CAM_DIST_DEFAULT) and looks
 * toward the origin. We compute the basis as:
 *   forward = normalize(target − camera_pos)
 *   right   = normalize(forward × world_up)        (world_up = +Y)
 *   up      = right × forward
 *
 * Vertical FOV is derived from horizontal FOV using the
 * (rows × ASPECT_Y / cols) ratio so terminal cells render as
 * physical SQUARES — without that correction, a circle would look
 * like a vertical ellipse.
 *
 * The cam_height parameter varies per pattern (Uranus tilts more
 * edge-on, Saturn shows the rings more open) — see §10 pattern_set.
 */
static void camera_make(Camera *c, int cols, int rows, float cam_height)
{
    c->cols = cols;
    c->rows = rows;
    c->pos  = v3(0.f, cam_height, -CAM_DIST_DEFAULT);
    V3 target  = v3(0.f, 0.f, 0.f);
    V3 worldup = v3(0.f, 1.f, 0.f);
    c->fwd   = v3_norm(v3_sub(target, c->pos));
    c->right = v3_norm(v3_cross(c->fwd, worldup));
    c->up    = v3_cross(c->right, c->fwd);
    c->fov_h = FOV_H;
    c->fov_v = FOV_H * (float)rows * ASPECT_Y / (float)cols;
}

/*
 * camera_ray — build a normalised view ray from a sub-pixel screen
 *              coordinate (fx, fy).
 *
 * The (2·fx + 1 − cols) / cols form maps integer pixel column 0..cols
 * to the centred range [-1, +1] with a half-pixel offset (so we
 * sample the cell CENTRE, not corner). Multiply by fov_h for
 * horizontal angular extent.
 *
 * The vertical axis is NEGATED because terminal y grows downward
 * but world y (up) grows upward.
 *
 *     ray_dir = normalize(forward + u · right + v · up)
 */
static V3 camera_ray(const Camera *c, float fx, float fy)
{
    float u =  ( (2.f * fx + 1.f) - (float)c->cols ) / (float)c->cols * c->fov_h;
    float v = -( (2.f * fy + 1.f) - (float)c->rows ) / (float)c->rows * c->fov_v;
    return v3_norm(v3_add(c->fwd,
                          v3_add(v3_scl(c->right, u),
                                 v3_scl(c->up,    v))));
}

/*
 * trace_one — primary ray dispatcher for one sub-pixel sample.
 *
 * Algorithm:
 *   1. Build a view ray through screen coord (fx, fy).
 *   2. Test against both primitives (sphere + ring annulus).
 *   3. Depth-sort: pick the smaller positive t.
 *      If sphere wins  → shade_planet() (§9.2)
 *      If ring wins    → shade_ring()   (§9.3)
 *      If both miss    → shade_space()  (§9.4 — sky + stars)
 *
 * The (sx, sy, frame) integer triple is passed to ring shading so
 * its soft-shadow jitter is deterministic per pixel per frame —
 * preventing the penumbra from "boiling" frame to frame.
 */
static V3 trace_one(const Scene *s, const Theme *th, const Camera *cam,
                    V3 sun_dir, float fx, float fy, int sx, int sy)
{
    V3 rd = camera_ray(cam, fx, fy);

    float t_sphere = 0.f, t_ring = 0.f;
    V3    hit_ring = v3(0,0,0);
    bool  hit_s = ray_sphere(cam->pos, rd, v3(0,0,0), PLANET_RADIUS, &t_sphere);
    bool  hit_r = ray_ring  (cam->pos, rd,
                             s->pp.ring_r_in, s->pp.ring_r_out,
                             &t_ring, &hit_ring);

    if (hit_s && (!hit_r || t_sphere < t_ring)) {
        V3 hit  = v3_add(cam->pos, v3_scl(rd, t_sphere));
        V3 view = v3_norm(v3_scl(rd, -1.f));
        return shade_planet(th, &s->pp, s->continent_phase, s->seed_phase,
                            hit, view, sun_dir, s->current_pattern,
                            s->shade_mode);
    }
    if (hit_r) {
        V3 view = v3_norm(v3_scl(rd, -1.f));
        return shade_ring(th, &s->pp, s->seed_phase,
                          hit_ring, view, sun_dir,
                          sx, sy, s->frame, s->shade_mode);
    }
    return shade_space(th, sx, sy, s->star_seed, s->time_secs);
}

static void scene_draw(int cols, int rows, const Scene *s)
{
    Camera cam;
    camera_make(&cam, cols, rows - 2, s->pp.cam_height);
    V3 sun_dir = scene_sun_dir(s);
    const Theme *th = &themes[s->current_theme];

    int top = 1;                          /* row 0 = HUD                 */
    int bot = rows - 1;                   /* rows-1 = hint               */
    if (rows < 5) { top = 0; bot = rows; }

    int spp = s->spp;
    if (spp < SPP_MIN_VAL) spp = SPP_MIN_VAL;
    if (spp > SPP_MAX_VAL) spp = SPP_MAX_VAL;

    for (int sy = top; sy < bot; sy++) {
        for (int sx = 0; sx < cols; sx++) {
            V3 acc = v3(0,0,0);
            for (int p = 0; p < spp; p++) {
                /* Deterministic sub-pixel jitter from hash. */
                uint32_t h = hash3(sx, sy, s->frame * spp + p);
                float jx = hash01(h)              - 0.5f;
                float jy = hash01(h ^ 0xDEADBEEFu) - 0.5f;
                float fx = (float)sx + 0.5f + jx;
                float fy = (float)(sy - top) + 0.5f + jy;
                V3 c = trace_one(s, th, &cam, sun_dir, fx, fy, sx, sy);
                acc = v3_add(acc, c);
            }
            paint_cell(sx, sy, v3_scl(acc, 1.f / (float)spp));
        }
    }
}

/* ── §12 hud ─────────────────────────────────────────────────────────── */

static void hud_draw(int cols, int rows, const Scene *s,
                     double fps, int sim_fps)
{
    /* §12.1 status — top-right (yellow + BOLD per HUD spec). */
    V3 sd = scene_sun_dir(s);
    float az = atan2f(sd.z, sd.x) * 180.f / (float)M_PI;

    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  %-8s  %s  spp:%d  sun:%+5.0f°  speed:%-2d ",
             fps, sim_fps,
             s->paused ? "PAUSED " : pattern_name(s->current_pattern),
             themes[s->current_theme].name,
             shade_mode_name(s->shade_mode),
             s->spp, (double)az, s->speed);
    int len = (int)strlen(buf);
    if (len > cols) len = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* §12.2 title — top-left (yellow no bold = secondary). */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, " SATURN-WITH-RINGS · RGB ");
    attroff(COLOR_PAIR(PAIR_HUD));

    /* §12.3 hint — bottom row (cyan + BOLD). */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  n/P:pattern  t/T:theme  m:mode  s:spp  +/-:speed  []:Hz  r:reseed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §13 app ─────────────────────────────────────────────────────────── */

typedef struct {
    Scene                 scene;
    int                   cols, rows;
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
    case ' ': case 'p':           s->paused = !s->paused; break;
    case 'r': case 'R':           scene_reseed(s); break;

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
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        break;

    case 'n':  case 'N':
        pattern_set(s, (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS));
        break;
    case 'P':  /* prev pattern (capital P, since lowercase p = pause) */
        pattern_set(s, (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS));
        break;

    case 's': case 'S': {
        int next = s->spp == 1 ? 2 : (s->spp == 2 ? 4 : 1);
        s->spp = next;
        break;
    }
    case 'm': case 'M':
        s->shade_mode = (ShadeMode)(((int)s->shade_mode + 1) % SHADE_N);
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

    App *app = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    initscr();
    noecho(); cbreak(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, app->rows, app->cols);

    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* §13.1 resize. */
        if (app->need_resize) {
            app->need_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, app->rows, app->cols);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* §13.2 timing. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* §13.3 fixed-step physics. */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* §13.4 fps rolling. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* §13.5 paint. */
        long long t0 = clock_ns();
        erase();
        scene_draw(app->cols, app->rows, &app->scene);
        hud_draw(app->cols, app->rows, &app->scene, fps_display, app->sim_fps);
        wnoutrefresh(stdscr);
        doupdate();

        /* §13.6 input. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;

        /* §13.7 frame cap. */
        app->scene.frame++;
        clock_sleep_ns(NS_PER_SEC / 60 - (clock_ns() - t0));
    }

    return 0;
}
