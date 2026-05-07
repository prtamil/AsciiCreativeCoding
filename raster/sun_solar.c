/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun_solar.c — a 2-D screen-space sun with corona and arc flares
 *
 * DEMO: A sun fills the centre of the terminal.  The disc has visible
 *       CONVECTION GRANULATION (fBm noise that drifts over time),
 *       darkens toward its LIMB (Eddington's 1-coefficient law), and
 *       is occasionally pocked by SUNSPOTS where the noise dips below
 *       threshold.  An exponential CORONA glow extends past the disc
 *       edge.  SOLAR FLARES launch from random footpoints, arc up to
 *       an apex, and fade — multiple active at once, each with its
 *       own lifecycle.  Five colour themes (real-sun yellow, blue
 *       giant, red dwarf, alien, photographic negative).
 *
 *       NOT a raymarcher.  This file lives in raymarcher/ for filing
 *       reasons but the algorithm is purely 2-D SCREEN-SPACE — for
 *       each terminal cell, compute distance from sun centre, decide
 *       which layer (disc / corona / outside) produces the cell's
 *       luminance, overlay flare arcs additively, quantise to glyph
 *       + theme colour.  No rays, no SDFs, no march loop.  See T1.
 *
 * Study alongside: raster/donut.c (point-cloud rasterisation — also
 *       2-D screen-space, also lives in raster/ for the same reason
 *       this file does), and raymarcher/raymarcher.c (a true 3-D
 *       surface raymarcher).  Rendering the same scene as a 3-D
 *       sphere with raymarched lighting would be 10× slower and the
 *       "sun" character (granulation, flares) would have to be
 *       invented anyway as 2-D textures.  This file is what you get
 *       when you accept that the SHAPE is trivial (a circle on the
 *       screen) and put all the work into TEXTURE + PHYSICAL EFFECTS
 *       layered onto that circle.
 *
 * Section map:
 *   §1   config       — every tunable named, no magic numbers later
 *   §2   clock        — monotonic timer + sleep
 *   §3   color        — themes + 2-pair HUD spec
 *   §4   hash + smoothstep — building blocks for value noise
 *   §5   value noise  — bilinear-blended hash grid
 *   §6   fBm          — 3-octave fractional Brownian motion
 *   §7   surface_lum  — disc luminance: limb + granulation + sunspots
 *   §8   corona_lum   — exponential glow outside the disc
 *   §9   flare struct + RNG
 *   §10  flare spawn + tick
 *   §11  flare envelope (rise / peak / fall)
 *   §12  flare arc geometry (parabolic blend of footpoints + apex)
 *   §13  scene state (tick, reset, zoom)
 *   §14  scene_render — luminance buffer + flare overlay + emit
 *   §15  screen — ncurses init / 2-row HUD / present
 *   §16  app — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause animation (granulation drift + flare lifecycles)
 *   r            reset (clear flares, reset time, reset zoom)
 *   t / T        next / previous theme
 *                  (SOLAR / BLUE_GIANT / RED_DWARF / ALIEN / NEGATIVE)
 *   + / =        flare spawn rate up
 *   -            flare spawn rate down
 *   z / Z        zoom in / out (sun disc grows / shrinks)
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/sun_solar.c \
 *       -o sun_solar -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is its own textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         what the algorithm is + references.
 *   • MENTAL MODEL     intuition + an ASCII diagram of the radial
 *                      luminance profile (disc → corona → space).
 *   • GUIDED TUTORIAL  ten short answers — a per-pixel walkthrough:
 *                      distance to centre → limb darkening → fBm
 *                      granulation → sunspots → corona → flare
 *                      lifecycle → flare arc geometry → compositing
 *                      → glyph + colour quantisation.
 *   • §1..§16          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in code:
 *      r            radial distance from sun centre (pixel units, with
 *                   y * CELL_ASPECT to compensate for tall cells)
 *      r_disc       disc radius, scaled by user zoom
 *      r_corona     outer corona radius (= r_disc · CORONA_MULT)
 *      μ (mu)       cosine of angle between line of sight and surface
 *                   normal — the variable in Eddington's limb-darkening
 *                   law.  Computed from r_norm = r / r_disc.
 *      L            per-cell luminance, accumulated into lum_buf
 *
 * Background you need:
 *   • basic floating-point arithmetic + sqrt + exp + sin/cos
 *   • familiarity with VALUE NOISE (random value at integer grid
 *     points, bilinearly blended) — Tutorials T4-T6 derive it
 *   • Eddington's limb darkening (Tutorial T3 explains briefly)
 *   • this file is NOT a raymarcher — see Tutorial T1
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : 2-D SCREEN-SPACE COMPOSITION.  No ray casting, no
 *                SDFs, no march loop.  For each terminal cell at
 *                pixel (col, row) we compute one number L (luminance)
 *                by ASKING THREE QUESTIONS in order:
 *
 *                  1. Where am I relative to the sun centre?
 *                     r = sqrt(dx² + (dy · CELL_ASPECT)²)
 *
 *                  2. Which radial REGION am I in?
 *                     r < r_disc        → surface_lum (limb + texture)
 *                     r < r_corona      → corona_lum  (exponential glow)
 *                     r ≥ r_corona      → 0           (empty space)
 *
 *                  3. Are any active FLARE ARCS passing through this
 *                     cell?  Each active flare contributes additive
 *                     luminance to a small set of cells along its
 *                     parabolic arc.
 *
 *                Combine, clamp, quantise to a 0..7 luma slot, look up
 *                glyph + theme colour, paint the cell.  Done.
 *
 *                Fancier than it sounds: surface_lum implements
 *                Eddington's limb-darkening law, multi-octave fBm
 *                granulation, and a noise-threshold sunspot dip,
 *                producing visibly "solar" behaviour from one number
 *                per cell.
 *
 * Data         : Stateless math (hash + value noise + fBm + the four
 *                radial-region functions) + a fixed-size pool of
 *                Flare structs (12 slots, struct-of-arrays would be
 *                tidier but irrelevant at this scale).  ONE per-frame
 *                static lum_buf[h][w] — luminance accumulation buffer
 *                for additive flare overlay.
 *
 * Rendering    : One pass over the canvas computes disc + corona;
 *                a second pass overlays each active flare's ARC
 *                samples additively into lum_buf; a third pass walks
 *                lum_buf and emits glyph + colour pair per cell with
 *                attron/attroff batching.  Top and bottom rows are
 *                reserved for the two-row HUD.
 *
 * Performance  : O(cols · rows) for disc + corona + emit (one fBm
 *                evaluation per disc cell — ~3 noise hashes and
 *                bilinear blends per fBm).  O(N_FLARES_MAX ·
 *                ARC_SAMPLES) for flare overlay (~ 12 × 36 = 432
 *                samples per frame, each writing ONE cell).  At 60
 *                fps on an 80×24 terminal: ~115 K disc cells per
 *                second + ~26 K flare samples — comfortable.
 *
 * References   :
 *   • Eddington, A. S. (1926) — "The Internal Constitution of the
 *     Stars", Cambridge University Press.  The 1-coefficient limb-
 *     darkening law (T3) is the canonical "first approximation" to
 *     stellar disc brightness.
 *   • Perlin, K. (1985) — "An Image Synthesizer", *SIGGRAPH '85*,
 *     pp. 287-296.  Background for value/gradient noise; our fBm
 *     (T4-T6) follows the same multi-octave layering pattern.
 *   • Quílez, I. — "Value noise" + "fbm"
 *     https://iquilezles.org/articles/morenoise/
 *     The pragmatic value-noise + fBm formulation we use directly.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * For any cell on the terminal, ASK ITS RADIAL DISTANCE from the sun
 * centre and look up which physical layer it belongs to.  Each layer
 * has a closed-form luminance function — there is no iteration, no
 * marching, no convergence.  Animation comes from time-shifting the
 * fBm input and ageing the flare lifecycle.  The whole renderer is
 * "for every cell, evaluate four short formulas and add their
 * contributions."  Cheap, deterministic, infinitely re-renderable.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a SOLAR CROSS-SECTION drawn radially: a bright disc, then
 * an exponentially-fading corona, then space.  The disc is decorated
 * with PROCEDURAL TEXTURE (granulation noise + sunspots) modulated
 * by Eddington's limb law.  Atop everything, a few PARABOLIC ARCS
 * (the flares) erupt and fade, each launching from one point on the
 * disc edge, peaking at an apex point above the disc, and curving
 * down to land at another disc-edge point.  The renderer paints the
 * cross-section once per frame; the sun "lives" because the noise
 * drifts and the flares come and go.
 *
 *      L
 *      ▲
 *      │ ╱──╲ surface_lum            corona_lum
 * 1.0 ─┤╱    ╲ (limb + granulation                CORONA_GAIN · exp(−d/falloff)
 *      │      ╲  + sunspots)                 ╲
 *      │       ╲                              ╲
 * 0.45 ┤        │╲                             ╲___
 *      │        │ ╲                                 ─────
 *      │        │  ╲                                       ─────
 *      ┼──────────────────────────────────────────────────────────► r
 *      0     r_disc·zoom        r_corona = r_disc · CORONA_MULT
 *
 *      ▲
 *      │  ●───── flare arc (additive on top of L) ─────●
 *      │  │ apex                                       │
 *      │  │   ╱──╲                                     │
 *      │  │  ╱    ╲                                    │
 *      │  ●        ●  footpoints on disc edge          │
 *      └──────────────────────────────────────────────────────────►
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per tick:
 *   1. animate         scene_t accumulates dt; fBm input shifts by
 *                      GRAN_DRIFT in x → granulation appears to flow
 *   2. flares_tick     age every active flare; expire at lifetime
 *   3. spawn           Poisson-style accumulator: every (1/spawn_rate)
 *                      seconds, call flare_spawn() to fill an open slot
 *
 * Per pass over the canvas (full frame each tick):
 *   4. PASS 1 — compute disc + corona luminance into lum_buf:
 *        for (row, col) in canvas:
 *           dx = col - cx
 *           dy = (row - cy) · CELL_ASPECT       (round disc on tall cells)
 *           r  = sqrt(dx² + dy²)
 *           if r < r_disc:    L = surface_lum(dx, dy, r, r_disc, t, seed)
 *           elif r < r_corona: L = corona_lum(r, r_disc)
 *           else:              L = 0
 *           lum_buf[row][col] = L
 *
 *   5. PASS 2 — additive flare overlay:
 *        for each active flare f:
 *           amp_life = flare_envelope(f)         rise/peak/fall
 *           for s in 0..ARC_SAMPLES:
 *               (px, py, amp_arc) = flare_arc_point(f, ..., s)
 *               lum_buf[py][px] += amp_life · amp_arc · f->intensity · FLARE_INTENSITY
 *
 *   6. PASS 3 — emit glyph + colour:
 *        for (row, col) in canvas:
 *           L = clamp(lum_buf[row][col], 0, LUM_CLAMP)
 *           Ln = L / LUM_CLAMP                     in [0, 1]
 *           if Ln < 0.02: skip cell (background)
 *           slot = floor(Ln · 7.999)               in {0..7}
 *           glyph = LUMA_GLYPHS[slot]
 *           pair  = PAIR_RAMP_BASE + slot
 *           attr  = A_BOLD (top), A_DIM (bottom), or A_NORMAL
 *           emit with attron/attroff batched on (pair, attr) change
 *
 * Per draw frame:
 *   7. HUD overlay     yellow row 0 (title + fps + status), cyan
 *                      hint at row rows-1
 *
 * KEY FORMULAS
 * ────────────
 * Radial distance with cell aspect compensation:
 *      r = sqrt(dx² + (dy · CELL_ASPECT)²)
 *      where dx = col − cx, dy = row − cy
 *
 * Eddington 1-coefficient limb darkening:
 *      μ = sqrt(1 − (r/r_disc)²)
 *      base = LIMB_BASE + LIMB_BIAS · μ
 *      → LIMB_BASE at r = r_disc (limb), full LIMB_BASE+LIMB_BIAS at centre
 *
 * Granulation modulation (drifting fBm):
 *      tex_c = fbm2d((dx − GRAN_DRIFT · t) · GRAN_SCALE,
 *                     dy · GRAN_SCALE,  seed) − 0.5
 *      → ±0.5 around 0; multiplied by GRAN_AMP and 2 to set full swing
 *
 * Sunspot darkening:
 *      spot = max(SPOT_THRESH − tex, 0)²
 *      → smooth-shouldered "spot strength" wherever fBm dips below threshold
 *
 * Surface luminance:
 *      modu = 1 − GRAN_AMP · 2 · tex_c − SPOT_AMP · spot   (clamped ≥ 0)
 *      L_surface = base · modu
 *
 * Corona (exponential decay outside disc):
 *      d = r − r_disc
 *      L_corona = CORONA_GAIN · exp(−d / (r_disc · CORONA_FALLOFF))
 *
 * Flare lifecycle envelope:
 *      τ = age / lifetime           in [0, 1]
 *      amp_life = 1 − |2τ − 1|^FLARE_LIFE_EXP
 *      → 0 at endpoints, 1 at midpoint, exp 1.4 = quick rise / sustained / quick fall
 *
 * Flare parabolic arc point (parameter s ∈ [0, 1]):
 *      A, B   footpoints on disc edge (theta_a, theta_b)
 *      M      chord midpoint (A+B)/2
 *      P*     apex = M + (M − sun_centre, normalised) · apex_height · r_disc
 *      P(s)   = (1−s)·A + s·B + 4·s·(1−s) · (P* − M)
 *      amp_arc(s) = sin(π · s)        // 0 at endpoints, 1 at apex
 *
 * Glyph quantisation:
 *      Ln   = clamp(L, 0, LUM_CLAMP) / LUM_CLAMP
 *      slot = floor(Ln · 7.999)
 *      glyph = LUMA_GLYPHS[slot]
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Cell aspect: terminal cells are ~2× taller than wide.  Without
 *     `dy · CELL_ASPECT` in the radial-distance formula, the "disc"
 *     would render as a tall ellipse.  Same correction applies inside
 *     flare_arc_point so footpoints sit on a circle, not an ellipse.
 *
 *   • Granulation drift: tex(t + dt) has the same shape as tex(t)
 *     translated horizontally — there is no actual "boiling".  At
 *     small drift speeds it reads as wind across the photosphere; at
 *     large speeds it looks unnaturally fast (uncanny valley).
 *     GRAN_DRIFT = 1.7 cell/sec is tuned to the natural "convection"
 *     impression on a 24-row terminal.
 *
 *   • Sunspot threshold: SPOT_THRESH = 0.30 means about 25% of the
 *     disc reads as "spot-darkened" at any instant (driven by fBm
 *     spending time below 0.30).  Set SPOT_THRESH to 0 → no spots;
 *     set to 0.5 → most of the disc is dark.
 *
 *   • Flare arcs use ARC_SAMPLES = 36 dense points along each arc.
 *     Adjacent samples sometimes round to the same cell — the
 *     additive accumulation reaches saturation there, which actually
 *     reads correctly as "the arc is brightest near the apex".
 *
 *   • The flare pool is fixed-size (N_FLARES_MAX = 12).  At very high
 *     spawn_rate values (12/sec) and average lifetime ~6 s, the pool
 *     fills and flare_spawn becomes a no-op.  Visible cap: arcs
 *     plateau at ~12 simultaneous regardless of how high spawn_rate
 *     goes.
 *
 *   • Inverted theme (NEGATIVE): pre-fills the canvas with white
 *     before painting darker glyphs over it.  A_BOLD/A_DIM are
 *     disabled in this mode (they invert their visual meaning
 *     against a light bg).
 *
 *   • Zoom × disc: r_disc = min_dim · DISC_FRAC · zoom.  At ZOOM_MAX
 *     (2.5) and a small terminal, r_corona can exceed the canvas —
 *     corners of the screen still show corona luminance, which is
 *     usually fine but can wash the HUD; the HUD's own bg pair stays
 *     readable due to bold + bright yellow / cyan.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • At default settings, the disc fills ~30% of the smaller
 *     dimension and is clearly bordered by a soft glow that fades
 *     to 0 within roughly r_disc · 0.7 outward.
 *
 *   • Pause (space): granulation FREEZES, flares FREEZE in place
 *     (they don't disappear, just stop ageing).  Confirms that
 *     animation is purely time-driven — pause kills both.
 *
 *   • Press t through all 5 themes.  Geometry stays identical; the
 *     colour ramp changes.  NEGATIVE flips bg/fg — verify that the
 *     HUD remains readable.
 *
 *   • Press z several times to zoom in.  Disc grows, corona scales
 *     proportionally (since r_corona = r_disc · CORONA_MULT).
 *     Spawning rate of flares is unchanged but each flare's arc spans
 *     more cells.
 *
 *   • Press + repeatedly to crank spawn rate up to 12/s.  Active
 *     flare count saturates at ~12 (pool full).  Press - to back
 *     off; eventually no flares.
 *
 *   • Press r to reset.  Time goes back to 0; granulation phase
 *     resets to t = 0; flare pool clears; zoom resets to 1.0.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — ten short answers ─────────────────────────────── *
 *
 *
 * T1: Why is this NOT a raymarcher?
 * ─────────────────────────────────
 * Most files in raymarcher/ render 3-D objects by tracing rays
 * through space until they hit a surface.  That's the right tool
 * when the "thing" you're rendering has 3-D structure that VARIES
 * along the ray (a fractal, a metaball cluster, a textured cube).
 *
 * A sun is different.  Geometrically it's a sphere; on screen it
 * projects to a circle.  Almost everything visually interesting
 * about a sun lives in the SURFACE TEXTURE — granulation, sunspots,
 * limb darkening — and those are 2-D patterns.  Plus the corona is
 * a glow you'd composite on top anyway, and the flares are 2-D
 * curves drawn over the disc.
 *
 * If we raymarched a 3-D sphere and applied procedural texture, we'd
 * pay the cost of ray casting (~30 SDF evals per pixel) AND still
 * compute the same 2-D textures.  Skip the rays: project the disc
 * directly to screen coordinates and ask "for this cell, what's the
 * luminance?".  Order-of-magnitude faster, identical visual result.
 *
 *
 * T2: For one cell, where am I in the sun?
 * ────────────────────────────────────────
 * The sun has a centre at the middle of the canvas, (cx, cy).  For a
 * cell at (col, row), compute:
 *
 *      dx = col − cx
 *      dy = (row − cy) · CELL_ASPECT          (tall-cell compensation)
 *      r  = sqrt(dx² + dy²)
 *
 * The CELL_ASPECT factor on dy is critical.  Terminal cells are
 * ~twice as tall as wide; without compensation, identical pixel
 * counts give different physical distances on screen and the disc
 * renders as a vertical ellipse.  Multiplying dy by 2 makes the
 * radial distance match what the eye sees as "round".
 *
 * Three regions decide how the cell is lit:
 *
 *      r < r_disc           → INSIDE the photosphere
 *      r_disc ≤ r < r_corona → IN the corona
 *      r ≥ r_corona          → OUTSIDE everything (empty space)
 *
 *
 * T3: Limb darkening — Eddington's law in 2-D
 * ───────────────────────────────────────────
 * On a real star, the disc edge appears DIMMER than the centre, even
 * though the surface is essentially uniform-emitting.  Why: at the
 * limb, the line of sight ENTERS the photosphere at a shallow angle
 * → it samples cooler, higher layers of plasma → dimmer.  The
 * Eddington 1-coefficient law captures this:
 *
 *      I(μ) = I₀ · (1 − u + u·μ)
 *
 * where μ is the cosine of the angle between line-of-sight and the
 * surface normal.  In our 2-D screen-space approximation, μ comes
 * from the projected radial distance:
 *
 *      μ = sqrt(1 − (r/r_disc)²)
 *
 * (For a sphere viewed in projection, points at radial distance r/R
 * subtend angle arccos(r/R) from the centre — so μ = sqrt(1 − (r/R)²).)
 *
 *      μ = 1 at r = 0       (disc centre, looking straight in)
 *      μ = 0 at r = r_disc  (limb, looking tangent to surface)
 *
 * In code:
 *      base = LIMB_BASE + LIMB_BIAS · μ
 *      with LIMB_BASE = 0.80, LIMB_BIAS = 0.20.
 *
 * That's the Eddington law's `(1 − u + u·μ)` reorganised: limb (μ=0)
 * has brightness 0.80; centre (μ=1) has 1.00.
 *
 *
 * T4: Building blocks for noise — hash + smoothstep
 * ─────────────────────────────────────────────────
 * For granulation we want a SMOOTH, DETERMINISTIC, REPEATABLE
 * pattern across the disc.  We build it from two pieces:
 *
 *   HASH.  A function that maps integer coordinates (xi, zi) to a
 *   pseudo-random number in [0, 1].  Three multiplications and two
 *   xors:
 *
 *      hash(x, z, seed) = mix prime-multiply-xor of (x, z, seed)
 *      → 32-bit integer, take top 24 bits, divide by 2^24
 *
 *   SMOOTHSTEP.  A function that maps t ∈ [0, 1] to a smooth
 *   "ease-in-ease-out" curve:
 *
 *      smoothstep(t) = t² · (3 − 2t)
 *      smoothstep(0) = 0, smoothstep(0.5) = 0.5, smoothstep(1) = 1
 *      derivative is 0 at both endpoints
 *
 * These two ingredients combine in T5 to produce VALUE NOISE.
 *
 *
 * T5: Value noise — bilinearly-blended hash grid
 * ──────────────────────────────────────────────
 * Take the HASH output at every integer grid point.  For a query at
 * fractional position (x, z), interpolate the four corners of the
 * unit square containing (x, z):
 *
 *      xi, zi   = floor(x), floor(z)
 *      fx, fz   = x − xi, z − zi          ∈ [0, 1]
 *
 *      v00, v10 = hash(xi, zi),     hash(xi+1, zi)
 *      v01, v11 = hash(xi, zi+1),   hash(xi+1, zi+1)
 *
 *      sx, sz   = smoothstep(fx), smoothstep(fz)
 *      a        = v00·(1−sx) + v10·sx     (bottom row)
 *      b        = v01·(1−sx) + v11·sx     (top row)
 *      v(x, z)  = a·(1−sz) + b·sz         (vertical blend)
 *
 * smoothstep on the blend factors gives C¹-smooth noise (no visible
 * grid lines).  The result is the canonical value noise — random-
 * looking but smoothly varying.
 *
 *
 * T6: fBm — three octaves stacked
 * ───────────────────────────────
 * Plain value noise has a single characteristic length scale (the
 * grid spacing).  Real granulation has detail at MULTIPLE scales —
 * big convection cells, smaller bubbles inside them, tiny flickers.
 * Add several layers of value noise, each at half the amplitude and
 * twice the frequency of the previous:
 *
 *      h = 0
 *      amp = 1; freq = 1; norm = 0
 *      for i in 0..2:
 *          h    += amp · vnoise2d(x · freq, z · freq, seed + i·17)
 *          norm += amp
 *          amp  *= 0.5         (each layer half as strong)
 *          freq *= 2.0         (each layer twice as detailed)
 *      return h / norm
 *
 * Three octaves is enough to give visible "structure within
 * structure" without paying for finer detail than the terminal can
 * resolve.  Each octave uses a different seed offset so the layers
 * don't align.
 *
 *
 * T7: Surface luminance — limb + granulation + sunspots
 * ─────────────────────────────────────────────────────
 * Combine T3 (limb darkening) and T6 (fBm) into one number L for
 * each disc cell.  surface_lum walks four steps:
 *
 *      1. μ from screen radius     (T3)
 *      2. tex = fbm2d(...)         (T6) — drifting in x with time
 *      3. spot = max(SPOT_THRESH − tex, 0)²
 *               → "spot strength" wherever fBm dips below threshold
 *      4. base = LIMB_BASE + LIMB_BIAS · μ
 *         modu = 1 − GRAN_AMP · 2 · (tex − 0.5) − SPOT_AMP · spot
 *         L    = base · modu              (clamped ≥ 0)
 *
 * That single product `base · modu` produces all the visible disc
 * features.  The granulation is a SUBTLE modulation (~30%); the
 * sunspots are a SHARP modulation (squared term) — so spots punch
 * through the granulation as distinct dark blotches, while the
 * granulation itself reads as a soft "boiling" texture.
 *
 *
 * T8: Corona — exponential glow outside the disc
 * ──────────────────────────────────────────────
 * Outside the disc, luminance falls off exponentially:
 *
 *      d = r − r_disc                            (positive in corona)
 *      L_corona = CORONA_GAIN · exp(−d / (r_disc · CORONA_FALLOFF))
 *
 * At r = r_disc, L = CORONA_GAIN = 0.45 (matches the dim end of the
 * disc).  At r = r_disc · (1 + CORONA_FALLOFF), L falls to 1/e ≈
 * 0.37 of the gain.  By r_corona = r_disc · 1.7, L is ~4.5% of the
 * disc luminance — visible as a faint halo, not lit ("glow") at the
 * outer edge.
 *
 * No lighting integral, no scattering simulation — just an
 * exponential.  Reads as a corona because the EYE expects "fades to
 * nothing" outside a bright disc, and exponentials match that
 * expectation.
 *
 *
 * T9: Flare arc geometry — parabolic blend of three points
 * ────────────────────────────────────────────────────────
 * A flare is defined by three random screen points + a lifecycle:
 *
 *      A   footpoint A   = sun_centre + (cos θ_a, sin θ_a / aspect) · r_disc
 *      B   footpoint B   = same with θ_b
 *      M   chord midpoint = (A + B) / 2
 *      P*  apex          = M + outward(M − sun_centre) · apex_height · r_disc
 *
 * The arc is a PARABOLIC BLEND of the chord and the apex:
 *
 *      P(s) = (1 − s) · A + s · B  +  4 · s · (1 − s) · (P* − M)
 *      arc_amp(s) = sin(π · s)        // 0 at endpoints, 1 at apex
 *
 * The 4·s·(1−s) term is the standard "Bezier middle bump": 0 at
 * endpoints (s = 0 or 1), 1 at midpoint (s = 0.5).  At s = 0 we land
 * at A; at s = 1 we land at B; at s = 0.5 we land at the apex P*.
 *
 * arc_amp(s) is a half-sine envelope along the arc — it ensures the
 * arc is FAINT at the footpoints (where the real flare is closest
 * to the photosphere and gets washed out by surface luminance) and
 * BRIGHTEST at the apex (where it stands out against dark space).
 *
 * Sample s at ARC_SAMPLES = 36 evenly-spaced points; for each
 * sample, additively boost lum_buf at the rounded cell coordinates.
 *
 *
 * T10: Quantising luminance to glyph + theme colour
 * ─────────────────────────────────────────────────
 * After PASS 1 (disc + corona) and PASS 2 (flares), every cell has
 * a luminance L in lum_buf.  The final pass:
 *
 *      L  = clamp(L, 0, LUM_CLAMP)        (LUM_CLAMP = 1.05)
 *      Ln = L / LUM_CLAMP                  ∈ [0, 1]
 *
 *      if Ln < 0.02:          skip cell (background)
 *      else:
 *          slot  = floor(Ln · 7.999)       ∈ {0..7}
 *          glyph = LUMA_GLYPHS[slot]       '.', ',', ':', ';', '+', '*', '#', '@'
 *          pair  = PAIR_RAMP_BASE + slot
 *          attr  = A_BOLD (slot ≥ 6) | A_DIM (slot ≤ 1) | A_NORMAL else
 *          mvaddch(row, col, glyph) with batched attron/attroff
 *
 * The 8 colour slots come from the active theme — SOLAR is a warm
 * crimson → orange → yellow → bone ramp matching real-sun colours;
 * BLUE_GIANT replaces it with a blue-white ramp; etc.
 *
 * Slot 0 maps to '.' (not space) — even the dimmest visible cell
 * paints SOMETHING, so the corona's outer fade reads as a gentle
 * wash of dim dots rather than abruptly disappearing.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* End of textbook.  The rest of the file is the worked exercises. */

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

/* ── §1 config ───────────────────────────────────────────────────────── */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,    /* yellow + bold — top status row     */
    PAIR_HINT         =  2,    /* cyan   + bold — bottom hint row    */
    PAIR_RAMP_BASE    =  3,    /* +0..+7 — luma ramp                 */

    /* Flare pool.  Sized for "always 4-8 active". */
    N_FLARES_MAX      = 12,

    /* Number of points sampled along each flare arc. */
    ARC_SAMPLES       = 36,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h / w                */

/* Sun sizing — fractions of the smaller screen dimension. */
#define DISC_FRAC        0.30f     /* R_disc / min(cols, rows·aspect)    */
#define CORONA_MULT      1.7f      /* R_corona = R_disc · this           */
#define CORONA_FALLOFF   0.35f     /* exp(−d / (R_disc · this))          */
#define CORONA_GAIN      0.45f     /* corona max brightness vs disc      */

/* Zoom — multiplier on DISC_FRAC.  z key zooms in (bigger disc), Z out. */
#define ZOOM_DEFAULT     1.0f
#define ZOOM_MIN         0.40f
#define ZOOM_MAX         2.50f
#define ZOOM_STEP        0.15f

/* Limb darkening (Eddington 1-coefficient law). */
#define LIMB_BASE        0.80f     /* (1 − u) intensity at the limb      */
#define LIMB_BIAS        0.20f     /* u · μ contribution at the centre   */

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

/* Brightness clamp for slot mapping. */
#define LUM_CLAMP           1.05f

/* Theme palette.  Five themes × 8 luma tiers.  inverted = white bg. */
typedef struct {
    const char *name;
    short       ramp[8];
    bool        inverted;
} Theme;

#define N_THEMES 5

static const Theme themes[N_THEMES] = {
    /* SOLAR: dim red → orange → yellow → bone-white (real-sun colours) */
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

/* Glyph ramp: faint → blazing.  Slot 0 is `.` not ' ' — even the
 * darkest visible cell paints a dim dot. */
static const char LUMA_GLYPHS[8] = { '.', ',', ':', ';', '+', '*', '#', '@' };

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

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

/* ── §3 color — themes + 2-pair HUD spec ─────────────────────────────── *
 *
 * 8 luma-ramp pairs (PAIR_RAMP_BASE..+7) hold the active theme's
 * colours.  Two HUD pairs (PAIR_HUD yellow, PAIR_HINT cyan) are
 * theme-independent per the CLAUDE.md HUD spec.
 *
 * Inverted themes use a white bg colour code (231 in 256-cube,
 * COLOR_WHITE in 8-cube) so the canvas pre-fill in scene_render
 * paints a white background under darker glyphs.
 */
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
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── §4 hash + smoothstep — building blocks for value noise (T4) ─────── */

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

/* ── §5 value noise — bilinearly-blended hash grid (T5) ──────────────── */

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

/* ── §6 fBm — three octaves of value noise stacked (T6) ──────────────── */

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

/* ── §7 surface_lum — disc luminance: limb + granulation + sunspots ──── *
 *
 * Tutorial T7 derived this.  Four steps in linear order:
 *      1. μ from screen radius      (Eddington's law, T3)
 *      2. fBm texture, drifting in x with time (T6)
 *      3. sunspot strength = max(threshold − texture, 0)²
 *      4. L = base · (1 − granulation_term − spot_term)
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
    float tex_c = tex - 0.5f;            /* centre on 0                  */

    /* Sunspots: where tex < SPOT_THRESH, additional darkness. */
    float spot = SPOT_THRESH - tex;
    if (spot < 0.0f) spot = 0.0f;
    spot *= spot;                        /* sharpen — spots have hard cores */

    /* Limb-darkened base × texture/spot modulation. */
    float base = LIMB_BASE + LIMB_BIAS * mu;
    float modu = 1.0f - GRAN_AMP * tex_c * 2.0f - SPOT_AMP * spot;
    if (modu < 0.0f) modu = 0.0f;

    return base * modu;
}

/* ── §8 corona_lum — exponential glow outside the disc (T8) ──────────── */

static inline float corona_lum(float r, float r_disc)
{
    float d = r - r_disc;
    return CORONA_GAIN * expf(-d / (r_disc * CORONA_FALLOFF));
}

/* ── §9 flare struct + RNG ───────────────────────────────────────────── *
 *
 * A flare is fully defined by:
 *   - active flag + age + lifetime (T11)
 *   - two footpoint angles on the disc (theta_a, theta_b)
 *   - apex height as a fraction of r_disc
 *   - a per-flare intensity scalar in [MIN, MAX]
 *
 * RNG: a tiny linear-congruential generator (Numerical Recipes
 * constants).  Determinism doesn't matter here — we just want random
 * variety.
 */
typedef struct {
    bool   active;
    float  age;
    float  lifetime;

    float  theta_a, theta_b;     /* footpoint angles on the disc        */
    float  apex_height;          /* fraction of r_disc                  */
    float  intensity;            /* [FLARE_INTENSITY_MIN, MAX]          */
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

/* ── §10 flare spawn + tick ──────────────────────────────────────────── *
 *
 * flares_clear   reset the entire pool.
 * flare_spawn    fill an inactive slot with fresh random parameters.
 * flares_tick    age every active flare; expire when age ≥ lifetime.
 *
 * Spawning RATE is handled separately by scene_tick using a Poisson-
 * style accumulator so the user can dial spawn_rate continuously.
 */

static void flares_clear(void)
{
    memset(g_flares, 0, sizeof g_flares);
}

/*
 * flare_spawn — find an inactive slot, fill with random parameters.
 *
 * Footpoints are spaced 0.4..ARC_FOOT_MAX_SPAN radians apart.  Too
 * close looks like a vertical spike; too far makes the chord cross
 * the disc centre and the arc reads as a chord rather than an arc.
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

static void flares_tick(float dt)
{
    for (int i = 0; i < N_FLARES_MAX; i++) {
        if (!g_flares[i].active) continue;
        g_flares[i].age += dt;
        if (g_flares[i].age >= g_flares[i].lifetime)
            g_flares[i].active = false;
    }
}

/* ── §11 flare envelope (rise / peak / fall) ─────────────────────────── *
 *
 *      τ = age / lifetime           in [0, 1]
 *      amp = 1 − |2τ − 1|^FLARE_LIFE_EXP
 *
 *      τ = 0   → amp = 0     (just spawned)
 *      τ = 0.5 → amp = 1     (peak)
 *      τ = 1   → amp = 0     (about to expire)
 *
 * Exponent 1.4 sits between linear (1.0) and parabolic (2.0): faster
 * rise + sustained peak + faster fall than a linear triangle.  Reads
 * naturally as "flare brightens, holds briefly, fades".
 */
static inline float flare_envelope(const Flare *f)
{
    float tau = f->age / f->lifetime;
    float u   = fabsf(2.0f * tau - 1.0f);
    return 1.0f - powf(u, FLARE_LIFE_EXP);
}

/* ── §12 flare arc geometry — parabolic blend (T9) ───────────────────── *
 *
 *   A   footpoint A on the disc edge   (cos θ_a, sin θ_a / aspect) · R
 *   B   footpoint B on the disc edge   (similar at θ_b)
 *   M   chord midpoint                  (A + B) / 2
 *   P*  apex                            M + outward · apex_height · R
 *
 *   P(s) = (1 − s) · A + s · B + 4·s·(1 − s) · (P* − M)
 *
 *   At s=0: P = A.  At s=1: P = B.  At s=0.5: P = P*.
 *   amp_arc(s) = sin(π · s)  — half-sine, 0 at endpoints, 1 at apex.
 *
 * The y components are pre-divided by CELL_ASPECT so the FOOTPOINTS
 * project to a circle on the screen (matching what the eye sees as
 * "the disc edge").  Without that division the footpoints would lie
 * on an ellipse.
 */
static inline void flare_arc_point(const Flare *f,
                                   float cx, float cy, float r_disc,
                                   float s,
                                   float *out_px, float *out_py,
                                   float *out_amp)
{
    /* Foot positions (cell-aspect-corrected). */
    float ax = cx + cosf(f->theta_a) * r_disc;
    float ay = cy + sinf(f->theta_a) * r_disc / CELL_ASPECT;
    float bx = cx + cosf(f->theta_b) * r_disc;
    float by = cy + sinf(f->theta_b) * r_disc / CELL_ASPECT;

    /* Chord midpoint and outward-from-centre direction. */
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

    /* Parabolic blend.  4·s·(1 − s) is 0 at endpoints, 1 at midpoint. */
    float bz = 4.0f * s * (1.0f - s);
    float px = (1.0f - s) * ax + s * bx + bz * (apx - mx);
    float py = (1.0f - s) * ay + s * by + bz * (apy - my);

    *out_px  = px;
    *out_py  = py;
    *out_amp = sinf((float)M_PI * s);
}

/* ── §13 scene state — tick, reset, zoom ─────────────────────────────── */

typedef struct {
    bool      paused;
    int       current_theme;
    int       cols, rows;
    uint32_t  seed;

    float     time;            /* accumulated seconds                   */
    float     spawn_rate;      /* flares / sec                          */
    float     spawn_accum;     /* unspent (1/spawn_rate) credits        */
    float     zoom;            /* disc size multiplier (z/Z keys)       */
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
    s->zoom          = ZOOM_DEFAULT;

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
    s->zoom = ZOOM_DEFAULT;
    flares_clear();
    g_flare_rng = (uint32_t)clock_ns() ^ 0xCAFEBABEu;
}

/*
 * scene_tick — advance time, age flares, spawn new ones.
 *
 * Spawn-rate accumulator: each tick adds dt · spawn_rate to a
 * "credit" counter.  Whenever the counter passes 1.0, spawn a flare
 * and subtract 1.0.  Equivalent to a Poisson process at rate
 * spawn_rate (integer-time approximation).
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
    flares_tick(dt);

    if (s->spawn_rate > 0.0f) {
        s->spawn_accum += dt * s->spawn_rate;
        while (s->spawn_accum >= 1.0f) {
            flare_spawn();
            s->spawn_accum -= 1.0f;
        }
    }
}

/* ── §14 scene_render — luminance buffer + flare overlay + emit ──────── *
 *
 * Three-pass composition (T2 + T7 + T8 + T9 + T10):
 *   Pass 1: per-cell disc / corona luminance into lum_buf.
 *   Pass 2: additive overlay of every active flare's arc samples.
 *   Pass 3: emit ncurses cells from lum_buf with batched attron/attroff.
 *
 * Buffer is needed because flare overlay is ADDITIVE — without a
 * buffer we'd have to either redraw cells (mvaddch each time) or
 * search across all flares for every cell.
 *
 * Buffer size: MAX_BUF_W × MAX_BUF_H = 280 × 90 = 25 200 floats ≈
 * 100 KB.  Comfortably within stack for any practical terminal.
 * Larger terminals than that get truncated to MAX_BUF; the visible
 * image clips cleanly.
 *
 * Two HUD rows reserved (top: yellow status, bottom: cyan hint).
 * The sun renders into rows 1..rows-2, controlled by y_offset = 1.
 */

#define MAX_BUF_W  280
#define MAX_BUF_H   90

static void scene_render(const Scene *s)
{
    /* Reserve top and bottom HUD rows.  Sun renders into rows 1..rows-2. */
    int rows_eff = s->rows - 2;
    int y_offset = 1;
    if (rows_eff < 1) return;

    int cols = s->cols;
    if (cols     > MAX_BUF_W) cols     = MAX_BUF_W;
    if (rows_eff > MAX_BUF_H) rows_eff = MAX_BUF_H;

    bool inverted = themes[s->current_theme].inverted;
    static float lum_buf[MAX_BUF_H][MAX_BUF_W];

    /* Sun centre (pixel coords) and disc radius — zoom-scaled. */
    float cx = (float)cols     * 0.5f;
    float cy = (float)rows_eff * 0.5f;
    float min_dim = (float)cols < (float)rows_eff * CELL_ASPECT
                  ? (float)cols
                  : (float)rows_eff * CELL_ASPECT;
    float r_disc   = min_dim * DISC_FRAC * s->zoom;
    float r_corona = r_disc * CORONA_MULT;

    /* Pass 1: disc + corona luminance. */
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

            /* Round to nearest cell.  Dense samples near the apex
             * pile into the same cell — the additive blend saturates
             * there, which reads as "arc is brightest at apex". */
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
                mvaddch(row + y_offset, col, ' ');
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
            mvaddch(row + y_offset, col, (chtype)(unsigned char)glyph);
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

/* ── §15 screen — ncurses init / 2-row HUD / present ─────────────────── */

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

/*
 * screen_draw — CLAUDE.md HUD spec.
 *   row 0          PAIR_HUD  (yellow + bold) — title + fps left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 *
 * FPS lives in the LEFT label so it stays visible even when the
 * settings status string overflows the terminal width.
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    /* Top row — title + fps left, settings status right (truncated). */
    char left[48];
    snprintf(left, sizeof left, " SUN  %5.1f fps ", fps);
    int llen = (int)strlen(left);

    char status[200];
    snprintf(status, sizeof status,
             " %s  theme:%s  zoom:%.2f  flares:%2d  spawn:%4.2f/s  "
             "t:%6.1fs  sim:%3dHz ",
             s->paused ? "PAUSED" : "BURNING",
             themes[s->current_theme].name,
             (double)s->zoom,
             scene_active_flares(),
             (double)s->spawn_rate,
             (double)s->time,
             sim_fps);
    int slen = (int)strlen(status);
    int max_slen = sc->cols - llen;
    if (max_slen < 0)    max_slen = 0;
    if (slen > max_slen) slen     = max_slen;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", left);
    if (slen > 0)
        mvprintw(0, sc->cols - slen, "%.*s", slen, status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom row — cyan key hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  t/T:theme  +/-:spawn  "
             "z/Z:zoom  ]/[:fps ");
    clrtoeol();
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §16 app — main loop, signals, key handling ──────────────────────── */

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

    case 'z':
        /* zoom IN — bigger sun disc */
        s->zoom += ZOOM_STEP;
        if (s->zoom > ZOOM_MAX) s->zoom = ZOOM_MAX;
        break;
    case 'Z':
        /* zoom OUT — smaller sun disc */
        s->zoom -= ZOOM_STEP;
        if (s->zoom < ZOOM_MIN) s->zoom = ZOOM_MIN;
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
