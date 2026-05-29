/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * buddhabrot.c — Buddhabrot / Anti-Buddhabrot density-accumulator fractal.
 *
 * The Buddhabrot is the "trajectory image" of the Mandelbrot iteration.
 * For every randomly sampled complex number c we iterate z → z² + c
 * starting from z = 0 and ask: does |z| ever exceed 2 (escape) within
 * `max_iter` steps?  Then we accumulate a hit counter on every screen
 * cell visited by that orbit, conditional on the escape result:
 *
 *   • BUDDHA mode — plot orbits whose c ESCAPES.  Out-of-set c values
 *     produce trajectories that visit cells around the boundary of the
 *     Mandelbrot set.  The aggregate looks like a luminous meditating
 *     Buddha figure (rotated 90° here so head is on top).
 *
 *   • ANTI  mode — plot orbits whose c does NOT escape.  In-set c
 *     values produce bounded orbits that converge to attractor cycles
 *     inside the Mandelbrot set.  The aggregate reveals the interior
 *     structure: cardioid, bulbs, period-doubling cycles.
 *
 * One preset is shipped (anti 1k); the buddha engine code is kept
 * intact for easy re-enable.  Cycle with `n`/`N`.
 *
 * Algorithm per tick:
 *   1. Sample SAMPLES_PER_TICK random c values from the bounding box.
 *   2. In BUDDHA mode, fast-skip the main cardioid and period-2 bulb
 *      (always-bounded regions — no escape to find).
 *   3. Pass 1 — iterate z from 0 to determine the escape status of c.
 *   4. If escape status matches the mode's filter, run Pass 2: re-
 *      iterate and increment counts[row][col] at every visited cell.
 *   5. Track the running maximum hit count for log-tone normalisation.
 *
 * Density → glyph (5-tier log-tone mapping):
 *   COL_C1   '.'   sparse (rim / faint halo)
 *   COL_C2   ':'
 *   COL_C3   '+'
 *   COL_C4   '#'
 *   COL_C5   '@'   peak (bold)
 *
 * Keys:
 *   q / ESC   quit
 *   n / r     reset / advance preset
 *   p / spc   pause / resume
 *   ] / [     faster / slower simulation
 *   t / T     next / previous theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra buddhabrot.c -o buddhabrot -lncurses -lm
 *
 * REFERENCES — for the concepts and the rendering.
 *
 *   Mandelbrot dynamics & the Buddhabrot
 *   ── Mandelbrot, B.B., "The Fractal Geometry of Nature", Freeman,
 *      1982.  The foundational treatise on fractals; introduces the
 *      Mandelbrot set and the z → z² + c iteration (§6 here).
 *   ── Devaney, R., "An Introduction to Chaotic Dynamical Systems",
 *      Westview, 1989.  Rigorous background on escape dynamics and
 *      the Julia/Mandelbrot family — informs the "orbit" mental
 *      model used in mandelbrot_trace_orbit().
 *   ── Green, M., "The Buddhabrot Technique", 1993.
 *      http://superliminal.com/fractals/bbrot/
 *      Original description of plotting Mandelbrot ORBITS rather
 *      than escape times — exactly what §6 does.
 *
 *   Random number generation
 *   ── Knuth, D., "The Art of Computer Programming, Vol. 2:
 *      Seminumerical Algorithms", 3rd ed., §3.2.1.  LCG theory:
 *      spectral test, period analysis, why the low bits are bad,
 *      and why the high bits (the LCG_HIGH_BITS_SHIFT trick in
 *      lcg_unit_float) recover decent uniformity.
 *   ── Graham, Knuth & Patashnik, "Concrete Mathematics", 2nd ed.,
 *      §3.3.3.  Source of LCG_MULTIPLIER (1664525) and
 *      LCG_INCREMENT (1013904223) used in §2.
 *
 *   Rendering
 *   ── Foley, van Dam, Feiner & Hughes, "Computer Graphics:
 *      Principles and Practice", 3rd ed., 2013.  §6 on aspect-
 *      preserving window-to-viewport mappings — the basis for
 *      Viewport (§4) and viewport_project().
 *   ── Draves, S. & Reckase, E., "The Fractal Flame Algorithm",
 *      2003.  https://flam3.com/flame_draves.pdf
 *      Canonical reference for accumulating fractal hits into a
 *      density buffer and rendering with log-tone mapping.
 *      DensityGrid + log_tone_map() in §5/§7 implement exactly
 *      that pattern, reduced to 5 ASCII tiers.
 *   ── Bourke, P., "Character representation of greyscale images",
 *      1997.  https://paulbourke.net/dataformats/asciiart/
 *      Origin of the ASCII density-ramp glyph ordering
 *      ('.' ':' '+' '#' '@') used by k_tier_glyphs[] in §7.
 *   ── Padala, P., "NCURSES Programming HOWTO", 2005.
 *      https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *      Practical reference for init_pair, mvaddch, frame pacing,
 *      resize handling — every ncurses API used in §4 / §7 / §8.
 *
 * §1 types & data   §2 random          §3 time
 * §4 view & palette §5 density grid    §6 Mandelbrot & chaos game
 * §7 render & HUD   §8 scene & main
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>          /* memset */
#include <time.h>

/* ===================================================================== */
/* §1  types & data                                                       */
/* ===================================================================== */

/* ---- grid + HUD sizing ------------------------------------------ */
#define GRID_ROWS_MAX        80
#define GRID_COLS_MAX       300
#define HUD_TOP_ROWS          1     /* data row at top                    */
#define HUD_BOTTOM_ROWS       1     /* hint row at bottom                 */

/* ---- iteration / pacing ----------------------------------------- */
#define SIM_FPS_MIN          10
#define SIM_FPS_DEFAULT      30
#define SIM_FPS_MAX          60
#define SIM_FPS_STEP          5

#define SAMPLES_PER_TICK   2000     /* random c samples per simulation tick */
#define TOTAL_SAMPLES    500000     /* target accumulation per preset       */

/* ---- timing ----------------------------------------------------- */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define FRAME_NS_60FPS (NS_PER_SEC / 60)       /* outer-loop pacing target */
#define FPS_WINDOW_MS         500              /* rolling FPS window       */
#define MAX_DT_NS    (100 * NS_PER_MS)         /* cap dt to avoid spirals  */

/* ---- Mandelbrot iteration parameters ---------------------------- */
#define ESCAPE_RADIUS_SQ   4.0f     /* |z|² > 4  ⇔  |z| > 2 (escaped)     */
#define MANDELBROT_Z0_RE   0.0f     /* iteration starts at z = 0          */
#define MANDELBROT_Z0_IM   0.0f
#define CARDIOID_CUSP_X    0.25f    /* main cardioid cusp position        */
#define PERIOD2_BULB_R2    0.0625f  /* (1/4)² — bulb radius squared       */
#define PERIOD2_BULB_CX   -1.0f     /* period-2 bulb centre x             */

/* ---- aspect-correct projection ---------------------------------- */
#define ASPECT_CELL_HEIGHT    2.0f    /* cells are 2× taller than wide   */
#define ASPECT_INV            0.5f    /* 1 / ASPECT_CELL_HEIGHT          */

/* ---- density tiers ---------------------------------------------- */
#define N_DENSITY_TIERS       5     /* COL_C1..COL_C5                  */
#define DENSITY_FLOOR_BUDDHA  0.08f /* below → invisible (rim noise)   */
#define DENSITY_FLOOR_ANTI    0.25f /* anti has 10× max so floor is higher */
#define EPS_LOG_MAX           1e-12f

/* ---- view rectangle --------------------------------------------- */
#define VIEW_RE_HALF       1.75f   /* covers Mandelbrot's real extent (~3.5) */
#define VIEW_CENTER_RE    -0.5f    /* Buddhabrot's gravitational centre      */
#define VIEW_CENTER_IM     0.0f    /* Mandelbrot is symmetric across im=0    */

/* ---- HUD ------------------------------------------------------- */
#define HUD_PCT_FULL         100   /* "100%" — sample target reached         */

/* ---- LCG constants (Knuth's "minimal standard") ----------------- */
#define LCG_MULTIPLIER     1664525u
#define LCG_INCREMENT   1013904223u
#define LCG_HIGH_BITS_SHIFT     8u      /* drop low 8 bits (worse stats)   */
#define LCG_UNIT_DENOM     (1u << 24)   /* 2^24 — divisor for [0, 1)       */
#define LCG_SEED_TIME_MIX   123456789LL

/* ---- defaults & input ------------------------------------------- */
#define KEY_ESCAPE          27
#define DEFAULT_PRESET_IDX   0
#define DEFAULT_PALETTE_IDX  0

/* ---- glyphs ------------------------------------------------------ */
#define GLYPH_L1    '.'
#define GLYPH_L2    ':'
#define GLYPH_L3    '+'
#define GLYPH_L4    '#'
#define GLYPH_L5    '@'

/* ---- HUD --------------------------------------------------------- */
#define HUD_BUFFER_BYTES    160

/* ---- Complex — a point in the complex plane ℂ ------------------- *
 *
 * What it is: a complex number z = re + im·i, stored as two floats.
 * The whole Buddhabrot algorithm lives on the complex plane —
 * sample points c, the iteration variable z, every intermediate —
 * they're all Complex values.
 *
 * Why it exists — the Mandelbrot iteration is naturally complex:
 *
 *     z → z² + c              (z and c are complex numbers)
 *
 * That one line of math is the heart of the program.  Wrapping
 * (re, im) into a named type lets every helper take and return a
 * Complex by value and read like the math —
 *     `mandelbrot_step(z, c)`     vs   `mandelbrot_step(zr, zi, cr, ci)`
 * — keeping the inner loop short and focused on the algorithm
 * instead of the bookkeeping.  Modern compilers pass small structs
 * in registers, so there's no measurable cost over loose floats.
 *
 * Why two floats (not doubles): single precision gives ~7 decimal
 * digits, plenty for terminal-resolution rendering where the
 * smallest meaningful gap is one cell.  Doubles would double the
 * memory traffic in the hot loop for no visible gain.
 *
 * Fields:
 *   re — real part (the horizontal coordinate in the ℂ plane).
 *   im — imaginary part (the vertical coordinate).
 *
 * References:
 *   ── Mandelbrot, B.B., "The Fractal Geometry of Nature", 1982 —
 *      the foundational treatment of the z → z² + c iteration.
 *   ── Devaney, R., "An Introduction to Chaotic Dynamical
 *      Systems", ch. 4 — complex dynamics and the rigorous
 *      definition of orbits in ℂ. */
typedef struct {
    float re, im;
} Complex;

/* ---- ComplexRect — a rectangle of the complex plane ------------- *
 *
 * What it is: an axis-aligned rectangle of ℂ described by its real-
 * axis interval [re_min, re_max] and its imaginary-axis interval
 * [im_min, im_max].
 *
 * Why it exists — the Buddhabrot sampler picks each c value
 * uniformly at random from a rectangle.  Bundling the four bounds
 * into one type gives us three things:
 *
 *   1. SHORT signatures.  `sampler_pick_c(rng, &g_sample_box)`
 *      instead of `sampler_pick_c(rng, re_min, re_max, im_min, im_max)`.
 *   2. ONE place to change the region.  Want to zoom in on the
 *      cardioid?  Edit g_sample_box.  No other code touches the
 *      bounds.
 *   3. NAMED bounds.  `re_min` is unambiguous; `bbox[0]` is not.
 *
 * The single instance in this program is `g_sample_box`, set to
 * (-2.5, 1.0) × (-1.25, 1.25) — generous enough to enclose the
 * entire Mandelbrot set plus the regions where escaping orbits
 * begin their trajectories.
 *
 * Fields:
 *   re_min, re_max — real-axis bounds (left and right edges of
 *                     the rectangle).
 *   im_min, im_max — imaginary-axis bounds (bottom and top edges). */
typedef struct {
    float re_min, re_max;
    float im_min, im_max;
} ComplexRect;

/* ---- BuddhMode — orbit-filter selector --------------------------- *
 *
 * What it is: an enum naming the two flavours of orbit filtering
 * the Buddhabrot algorithm supports.
 *
 *   • MODE_BUDDHA — KEEP orbits whose c ESCAPES the iteration
 *                   (|z| > 2 within max_iter steps).  Out-of-set c
 *                   values produce trajectories that visit cells
 *                   around the Mandelbrot BOUNDARY.  Aggregated,
 *                   they paint the iconic luminous "Buddha figure".
 *
 *   • MODE_ANTI   — KEEP orbits whose c does NOT escape (bounded).
 *                   In-set c values produce trajectories that
 *                   converge to attractor cycles INSIDE the
 *                   Mandelbrot set.  Aggregated, they reveal its
 *                   interior structure: cardioid, period bulbs,
 *                   period-doubling cycles, internal filaments.
 *
 * Why it exists — the two modes share 99% of the code; they
 * differ in exactly one comparison:
 *
 *     if (anti_mode == escaped) return;   // skip wrong filter
 *
 * Naming the flavours lets MandelbrotPreset DECLARE its intent in
 * its struct field, instead of passing a magic boolean through
 * the algorithm.  It also documents the design: the engine is
 * built to support both, and a third flavour (e.g. a Nebulabrot
 * iteration-window filter) would extend this enum without
 * rewriting the iteration code.
 *
 * References:
 *   ── Green, M., "The Buddhabrot Technique" (1993) — original
 *      description of the escape-filter approach.
 *   ── Wikipedia, "Buddhabrot" — accessible side-by-side of the
 *      buddha and anti flavours. */
typedef enum {
    MODE_BUDDHA,
    MODE_ANTI,
} BuddhMode;

/* ---- MandelbrotPreset — a named Buddhabrot variant -------------- *
 *
 * What it is: everything you need to describe ONE flavour of the
 * fractal — a name (for the HUD), the iteration depth `max_iter`,
 * and the orbit-filter mode.  The chaos-game driver reads these
 * three fields and produces the corresponding image.
 *
 * Why it exists — the "engine + data" pattern.  The math is the
 * SAME for every preset (Mandelbrot iteration + density
 * accumulation); only `max_iter` and `mode` differ.  Wrapping
 * those two knobs into one named type makes the engine invariant
 * under the choice of variant — pick a preset → pick a flavour.
 * Adding a new variant is one entry in g_presets[]; no algorithm
 * code changes.
 *
 * The `max_iter` trade-off:
 *
 *     ~100      coarse approximation; main shape only
 *     1000      reasonable terminal-resolution detail (this file)
 *     10000     fine threads visible (much slower, hardly resolvable
 *               at terminal scale)
 *     100000+   ultra-fine; only meaningful at print resolution
 *
 * The `mode` selector — see BuddhMode for the buddha/anti story.
 *
 * Fields:
 *   name      — short label shown in the HUD.  Keep ≤ 11 chars so
 *               it fits cleanly under the `%-11s` HUD column.
 *   max_iter  — escape-iteration cap.  Pass-1 (the escape test)
 *               runs up to max_iter steps; Pass-2 (the orbit
 *               trace) also runs up to max_iter.  Cost per
 *               sample is roughly proportional to max_iter.
 *   mode      — buddha (plot escaping orbits) or anti (plot
 *               bounded orbits).  Selects how Pass-1's escape
 *               result decides whether Pass-2 runs.
 *
 * References:
 *   ── Green, M., "The Buddhabrot Technique" (1993) — first
 *      description of plotting Mandelbrot orbits filtered by
 *      escape status. */
typedef struct {
    const char *name;
    int         max_iter;
    BuddhMode   mode;
} MandelbrotPreset;

/* ---- Palette — a named density-tier colour theme ---------------- *
 *
 * What it is: a named five-entry colour table.  Index by density
 * tier (L1..L5) and get back an xterm 256-colour code — or an
 * 8-colour fallback for older terminals.
 *
 * Why it exists — keeps THEMING separate from DRAWING.  The
 * renderer never sees raw colour numbers; it asks for the ncurses
 * colour pair matching a tier and draws.  Adding a new theme is
 * one entry in g_palettes[]; the rest of the program doesn't
 * notice.  The `t` / `T` keys cycle palettes by reloading just
 * the five COL_C1..COL_C5 pairs (a single init_pair per tier).
 *
 * TIER-COLOUR CONVENTION:
 *   C1 (sparsest)  — a DARKER variant of the theme's starting hue.
 *                    Painted on rim/halo cells where only a few
 *                    orbits land.  Making C1 dim keeps the outer
 *                    halo subtle so the bright body stands out.
 *   C2..C4         — progressively brighter mid-tones across
 *                    multiple HUES (not just shades of one) so
 *                    density layers contrast strongly.
 *   C5 (densest)   — peak highlight (white or hottest hue).
 *                    Drawn with A_BOLD for extra punch.
 *
 * WHY MULTI-HUE: a single-hue ramp (e.g. all greens, dim → bright)
 * still works, but it reads as flat.  Walking the ramp across
 * different hues (purple → cyan → green → yellow → white, for
 * example) makes each density layer visually distinct, and the
 * resulting fractal looks gorgeous instead of monochromatic.
 *
 * Fields:
 *   name      — short label shown in the HUD (≤ 6 chars keeps the
 *               HUD line compact).
 *   fg256[]   — xterm-256 colour codes for L1..L5.  Used when
 *               COLORS ≥ 256.  Indexed [0..4] = [L1..L5].
 *   fg8[]     — fallback colour codes for 8-colour terminals.
 *               Used when COLORS < 256.  Same indexing.
 *
 * References:
 *   ── XTerm 256-colour palette spec:
 *      https://en.wikipedia.org/wiki/ANSI_escape_code#8-bit */
typedef struct {
    const char *name;
    int  fg256[N_DENSITY_TIERS];
    int  fg8[N_DENSITY_TIERS];
} Palette;

/* ---- RandLCG — a fast linear congruential RNG ------------------- *
 *
 * What it is: a tiny random-number generator that holds 32 bits of
 * state and ticks forward with one multiply and one add:
 *
 *     state = state · LCG_MULTIPLIER + LCG_INCREMENT
 *
 * That's it.  Two operations per draw.
 *
 * Why it exists — SPEED.  The Buddhabrot needs two random floats
 * per sample (one for re, one for im) and does thousands of
 * samples per frame.  At ~10⁵-10⁶ random draws per second, any
 * function-call RNG (rand(), drand48(), etc.) would dominate the
 * inner loop.  An inline LCG is a couple of cycles per draw and
 * the compiler hoists its state into a register.
 *
 * QUALITY vs. SPEED TRADE-OFF: we do NOT need cryptographic
 * randomness.  We need uniform [0, 1) values to pick random c
 * points in the sample box.  The LCG's well-known weakness is
 * correlation in the LOW bits — see Knuth TAOCP §3.2.1.  We
 * sidestep it by extracting the HIGH 24 BITS in lcg_unit_float
 * (shift right by 8), which behave much better statistically.
 * Period is 2³² (full period since LCG_MULTIPLIER ≡ 1 mod 4 and
 * gcd(LCG_INCREMENT, 2³²) = 1).
 *
 * About the LCG parameters: the pair (1664525, 1013904223) is the
 * "minimal standard" from Knuth's "Concrete Mathematics" §3.3.3.
 * Well-studied, full period, reasonable uniformity for non-
 * statistical applications.
 *
 * Fields:
 *   state — current 32-bit LCG state.  MUST be non-zero — a zero
 *           state stays zero forever under the LCG recurrence.
 *           Seeded once by lcg_seed_from_clock(); updated in place
 *           by lcg_next_u32().
 *
 * References:
 *   ── Knuth, TAOCP Vol. 2, §3.2.1 — full LCG theory, spectral
 *      test, and the choice of multiplier.
 *   ── Graham, Knuth & Patashnik, "Concrete Mathematics", §3.3.3 —
 *      the specific multiplier and increment used here.
 *   ── O'Neill, "PCG: A Family of Simple Fast Space-Efficient
 *      Statistically Good Algorithms for Random Number Generation"
 *      (2014).  Modern replacement if quality matters — we don't
 *      use it, but the paper explains LCG weaknesses cleanly. */
typedef struct {
    uint32_t state;
} RandLCG;

/* ---- DensityGrid — chaos-game hit accumulator ------------------ *
 *
 * What it is: a 2-D array of 32-bit counters, one per terminal
 * cell.  Each counter holds how many sampled orbits visited that
 * cell.  Think of it as a 2-D HISTOGRAM of orbit positions in
 * screen space.
 *
 * Why it exists — DENSITY is what makes the Buddhabrot visible.
 * A single orbit visits dozens to thousands of cells; the iconic
 * figure only emerges after MILLIONS of orbits accumulate.
 * Counting hits per cell turns "did any orbit visit this cell?"
 * (binary) into "how many?" (continuous), which is what lets us
 * paint smooth brightness gradients via log-tone mapping.
 *
 * LOG-TONE MAPPING (used by grid_render):
 *
 *     normalised_t = log(1 + hits) / log(1 + max_count)
 *
 * Why log instead of linear: the brightest cells get hit 10⁵-10⁶
 * times more often than the faintest.  Linear mapping makes
 * everything below the top 1% invisible.  Log compresses that
 * dynamic range so the entire gradient is visible — the same
 * trick fractal-flame renderers use (Draves & Reckase 2003).
 *
 * RUNNING max_count: we update max_count INCREMENTALLY on each
 * grid_hit, which avoids a per-frame O(rows × cols) scan.
 * Trade-off: this only works because counts only ever GROW.  If
 * we ever decremented a cell, the running max could go stale and
 * we'd need a periodic rescan.
 *
 * Why FIXED size: we know the upper bound on terminal size at
 * compile time (GRID_ROWS_MAX × GRID_COLS_MAX), so the whole grid
 * lives in BSS — no allocation, no resize hassle.  At 80 × 300
 * cells with 4 bytes per counter, that's ~96 KB — fits in L2
 * cache, plenty fast.
 *
 * Lifecycle:
 *   • grid_clear()  — zero every counter + reset max_count.
 *                     Called on init, resize, 'r' reset,
 *                     and preset change.
 *   • grid_hit()    — increment one cell, update max_count.
 *                     Called once per visited cell during Pass 2
 *                     of every accepted orbit.
 *
 * Fields:
 *   counts[][] — hit counters indexed [row][col] in TERMINAL
 *                coordinates (not ℂ-space).  Row 0 is the top of
 *                the screen.  Counter type is uint32_t — far
 *                above any realistic max_count (~10⁶) so the
 *                4-byte saturation limit is never reached.
 *   max_count  — running maximum across counts[][].  Used as the
 *                denominator in log-tone mapping.  Reset to 0 by
 *                grid_clear().
 *
 * References:
 *   ── Draves, S. & Reckase, E., "The Fractal Flame Algorithm",
 *      2003 — canonical density-buffer + log-tone-mapping pattern
 *      for IFS-style fractal rendering.  Buddhabrot uses the same
 *      accumulator pattern.
 *   ── Bourke, P., "Character representation of greyscale images",
 *      1997 — ASCII glyph ramps for density display. */
typedef struct {
    uint32_t counts[GRID_ROWS_MAX][GRID_COLS_MAX];
    uint32_t max_count;
} DensityGrid;

/* ---- Viewport — ℂ-plane → terminal cells ----------------------- *
 *
 * What it is: a precomputed RECIPE for projecting a point in the
 * complex plane (a Complex z) onto a terminal cell (a col, row
 * pair).  Just a handful of numbers that, together, describe "how
 * to place the figure on screen".
 *
 * Why it exists — keep the projection IN ONE PLACE.  The chaos
 * game's inner loop calls viewport_project once per orbit point
 * and doesn't need to know about terminal size, HUD rows, or
 * cell aspect.  Resize?  Recompute the Viewport once,
 * everything downstream automatically reprojects on the next
 * frame.  One struct, one source of projection truth.
 *
 * PORTRAIT ORIENTATION (rotated 90° from traditional Buddhabrot).
 * The natural figure (head, body, shoulders, tendrils) is
 * wider-than-tall in landscape orientation.  We rotate the
 * projection so the head sits at the TOP and the body extends
 * DOWNWARD — much better fit in a vertical-aspect view:
 *
 *     col ← imaginary axis    (im_min → LEFT,  im_max → RIGHT)
 *     row ← real axis         (re_min → TOP,   re_max → BOTTOM)
 *
 * TWO ASPECT TRANSFORMS happen on the way to the screen,
 * captured by the single field `scale`:
 *
 *   1) ASPECT CORRECTION.  Terminal cells are ~2× taller than
 *      wide, so distances measured in rows and columns aren't
 *      directly comparable.  We work in "cell-WIDTH units"
 *      everywhere; vertical distances get multiplied by 0.5
 *      (ASPECT_INV) when converted to row counts.
 *
 *   2) ASPECT FIT.  The figure has a natural bounding box in ℂ.
 *      We pick the TIGHTER of (horizontal cell budget) vs
 *      (vertical cell budget in width-units) so the figure fits
 *      inside the terminal both ways while keeping its TRUE
 *      shape.
 *
 * One number, `scale`, captures both: visual cells (width-units)
 * per ℂ-unit.  Bigger terminal → bigger scale → bigger drawing.
 *
 * THE ARITHMETIC, in plain prose:
 *
 *     col = center_col  +  (z.im − view_mid_im) · scale
 *     row = center_row  +  (z.re − view_mid_re) · scale · 0.5
 *
 *   • shift z so the view midpoint sits at the screen centre
 *   • multiply by `scale` (width-units per ℂ-unit) for col
 *   • multiply by `scale · 0.5` for row (the aspect correction)
 *   • add the screen-centre offset
 *
 * Lifecycle: rebuilt by viewport_fit() on init and on SIGWINCH
 * resize.  Constant for many frames otherwise.
 *
 * Fields:
 *   rows, cols   — current terminal dimensions in cells.
 *   play_rows    — number of usable content rows
 *                  (= rows − HUD_TOP_ROWS − HUD_BOTTOM_ROWS).
 *                  Cached so neither the renderer nor the chaos
 *                  game has to redo the subtraction.
 *   center_col   — column where the figure's centre lands.
 *                  Equals cols / 2.
 *   center_row   — row where the figure's centre lands.  Centred
 *                  between the HUD rows.
 *   scale        — visual cells (width-units) per ℂ-unit.  Bigger
 *                  terminal → bigger scale → bigger drawing.
 *                  This is the single "size" knob.
 *   view_mid_re  — real part of the ℂ-point at the screen centre.
 *                  Set to -0.5 (the Buddhabrot's gravitational
 *                  centre).
 *   view_mid_im  — imaginary part at the screen centre.  Set to
 *                  0 (the Mandelbrot set is symmetric about the
 *                  real axis).
 *
 * References:
 *   ── Foley, van Dam, Feiner & Hughes, "Computer Graphics:
 *      Principles and Practice", 3rd ed., §6 — window-to-viewport
 *      mappings with aspect preservation. */
typedef struct {
    int    rows, cols;
    int    play_rows;
    int    center_col, center_row;
    float  scale;
    float  view_mid_re, view_mid_im;
} Viewport;

/* ---- FpsCounter — rolling FPS measurement ----------------------- *
 *
 * What it is: a tiny stopwatch that counts frames and time inside
 * a ROLLING WINDOW (default 500 ms), then divides them to produce
 * a stable frames-per-second number for the HUD.
 *
 * Why it exists — RAW per-frame FPS jitters wildly.  One frame
 * might take 30 ms because of a scheduling hiccup, the next 5 ms,
 * the next 50 ms while the kernel is paging.  A naive `1 / dt`
 * reading flickers through 20–200 fps every frame and is useless
 * to look at.  Averaging across half a second smooths that out
 * into one stable number that actually reflects sustained
 * performance.
 *
 * How the windowing works:
 *
 *   each frame:
 *      accum_ns += frame_duration
 *      frames   += 1
 *      if accum_ns ≥ window:                  // ~500 ms passed
 *          display = frames / (accum_ns / 1e9)
 *          accum_ns = 0
 *          frames   = 0
 *
 * We refresh the HUD's `display` only at window boundaries, so
 * the number on screen updates ~2× per second.  Between updates
 * the HUD keeps showing the LAST computed value, which is exactly
 * what we want for stability.
 *
 * Fields:
 *   accum_ns — running sum of frame durations in the current
 *              window.  Reset to 0 at each window boundary.
 *   frames   — number of frames accumulated in the current
 *              window.  Reset to 0 at each window boundary.
 *   display  — last computed FPS value.  This is what the HUD
 *              actually reads; updated only at window boundaries.
 *
 * References:
 *   ── Standard rolling-average pattern; no specific paper. */
typedef struct {
    long long accum_ns;
    int       frames;
    double    display;
} FpsCounter;

/* ---- Scene — the whole program's state -------------------------- *
 *
 * What it is: ONE struct holding everything the running program
 * needs — the density grid, the orbit RNG, the projection, the
 * theme, animation timing, and the user's control toggles.
 *
 * Why it exists — collapses what would otherwise be a DOZEN
 * scattered globals into one named container.  The main loop ends
 * up reading as a tiny sequence of verbs on a single noun:
 *
 *     scene_process_input(s);   // react to keys
 *     scene_tick(s);            // run one frame of the chaos game
 *     frame_render(s);          // draw grid + HUD
 *
 * Every helper declares its dependency by taking the right
 * sub-pointer:
 *   • `buddhabrot_iterate(s)`     — needs grid + view + preset + rng
 *   • `grid_render(grid, view, mode)` — only the things it draws from
 *   • `viewport_project(view, z, ...)` — only the projection
 *
 * Function signatures spell out which parts of the world each
 * function touches — NO MYSTERY GLOBALS to chase.  If you ever
 * needed to checkpoint, replay, or run two instances side-by-side,
 * the whole runtime state is one POD struct you could memcpy.
 *
 * Fields, grouped by responsibility:
 *
 *   What we're computing (the fractal):
 *     grid          — density accumulator.  Cleared by grid_clear
 *                     on init / resize / reset / preset change.
 *     preset        — index into g_presets[] of the active variant.
 *                     Cycled by 'n' / 'N' / 'r' (rebuilds grid).
 *     samples_done  — count of c samples processed for this preset.
 *                     When ≥ TOTAL_SAMPLES, scene_tick stops the
 *                     chaos game and the image holds indefinitely.
 *     rng           — random source for the sampler.  Seeded once
 *                     from the wall clock by lcg_seed_from_clock.
 *
 *   How we view it (presentation):
 *     view          — ℂ → cell projection.  Re-fit on resize.
 *     palette       — index into g_palettes[] of the active theme.
 *                     Cycled by 't' / 'T'; reloads the five
 *                     COL_C1..COL_C5 ncurses pairs.  Doesn't touch
 *                     the grid.
 *
 *   Control:
 *     paused        — chaos game freezes; HUD still updates.
 *                     Toggled by 'p' or space.
 *     sim_fps       — simulation tick rate (SIM_FPS_MIN .. MAX, in
 *                     SIM_FPS_STEP increments).  Adjusted by '[' /
 *                     ']'.  Higher → more samples per second →
 *                     faster build but more CPU.
 *
 *   Timing:
 *     fps           — rolling FPS counter for the HUD. */
typedef struct {
    /* what we're computing */
    DensityGrid    grid;
    int            preset;
    int            samples_done;
    RandLCG        rng;

    /* how we view it */
    Viewport       view;
    int            palette;

    /* control */
    bool           paused;
    int            sim_fps;

    /* timing */
    FpsCounter     fps;
} Scene;

/* ---- colour-pair slots ------------------------------------------ */
enum {
    COL_C1   = 1,   /* sparsest density tier (rim)                   */
    COL_C2   = 2,
    COL_C3   = 3,
    COL_C4   = 4,
    COL_C5   = 5,   /* densest tier (peak, drawn bold)               */
    COL_HUD  = 6,   /* top data row — bright yellow                  */
    COL_HINT = 7,   /* bottom hint row — bright cyan                 */
};

/* ---- sample box (constant config) ------------------------------- */
static const ComplexRect g_sample_box = {
    .re_min = -2.50f, .re_max =  1.00f,
    .im_min = -1.25f, .im_max =  1.25f,
};

/* ---- presets (constant config) ---------------------------------- *
 * One preset shipped.  The buddha-mode engine code is kept intact
 * for future re-enable; no current preset uses MODE_BUDDHA. */
#define N_PRESETS 1
static const MandelbrotPreset g_presets[N_PRESETS] = {
    { "anti     1k", 1000, MODE_ANTI },
};

/* ---- palettes (constant config) --------------------------------- *
 * Each theme walks across multiple bright hues; C1 (sparsest tier)
 * is a darker variant so the rim reads as a subtle introductory
 * glow.  Per CLAUDE.md the 24-29 dim cube range is OK only at the
 * lowest tier; all C2..C5 codes are ≥ 30. */
#define N_PALETTES 8
static const Palette g_palettes[N_PALETTES] = {
/*                       C1   C2   C3   C4   C5                     C1   C2   C3   C4   C5                                                  walk */
    { "Aurora",     {    53,  51,  46, 226, 231 },   { COLOR_BLUE, COLOR_CYAN, COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE } },  /* dark plum → cyan → lime → yellow → white */
    { "Galaxy",     {    54, 165,  51, 213, 231 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN, COLOR_MAGENTA, COLOR_WHITE } },  /* dim indigo → magenta → cyan → pink → white */
    { "Sunset",     {    24, 201, 196, 214, 226 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_RED, COLOR_RED, COLOR_YELLOW } },  /* deep blue → magenta → red → orange → yellow */
    { "Spectrum",   {    25,  51,  46, 226, 196 },   { COLOR_BLUE, COLOR_CYAN, COLOR_GREEN, COLOR_YELLOW, COLOR_RED } },  /* dim blue → cyan → green → yellow → red */
    { "Vapor",      {    53, 213,  51, 159, 231 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },  /* dark plum → pink → cyan → lt cyan → white */
    { "Tropical",   {    28,  46, 226, 208, 196 },   { COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED } },  /* deep green → lime → yellow → orange → red */
    { "Phoenix",    {    53, 201, 196, 214, 226 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_RED, COLOR_RED, COLOR_YELLOW } },  /* dark violet → magenta → red → orange → yellow */
    { "Neon",       {    89, 201,  39,  51, 231 },   { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE } },  /* dim magenta → magenta → blue → cyan → white */
};

/* ===================================================================== */
/* §2  random — fast LCG for sample selection                             */
/* ===================================================================== */

/* Seed the LCG from the wall clock.  Any non-zero seed works; zero
 * stays zero forever under the LCG recurrence. */
static void lcg_seed_from_clock(RandLCG *r)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    r->state = (uint32_t)(ts.tv_nsec ^ (ts.tv_sec * LCG_SEED_TIME_MIX));
    if (r->state == 0) r->state = 1u;
}

/* Step the LCG and return the new 32-bit state. */
static inline uint32_t lcg_next_u32(RandLCG *r)
{
    r->state = r->state * LCG_MULTIPLIER + LCG_INCREMENT;
    return r->state;
}

/* Uniform float in [0, 1).  Uses the high 24 bits of the LCG output;
 * the low bits of an LCG have worse statistical properties. */
static inline float lcg_unit_float(RandLCG *r)
{
    uint32_t high_bits = lcg_next_u32(r) >> LCG_HIGH_BITS_SHIFT;
    return (float)high_bits / (float)LCG_UNIT_DENOM;
}

/* Uniform float in [lo, hi). */
static inline float lcg_in_range(RandLCG *r, float lo, float hi)
{
    return lo + lcg_unit_float(r) * (hi - lo);
}

/* ===================================================================== */
/* §3  time — monotonic clock + rolling FPS                               */
/* ===================================================================== */

static long long clock_ns_now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* Add a frame's duration to the rolling counter; refresh the
 * displayed FPS at each window boundary so the HUD reading is
 * stable. */
static void fps_tick(FpsCounter *fps, long long frame_ns)
{
    fps->accum_ns += frame_ns;
    fps->frames   += 1;
    if (fps->accum_ns >= FPS_WINDOW_MS * NS_PER_MS) {
        fps->display = (double)fps->frames
                     / ((double)fps->accum_ns / (double)NS_PER_SEC);
        fps->accum_ns = 0;
        fps->frames   = 0;
    }
}

/* ===================================================================== */
/* §4  view & palette                                                     */
/* ===================================================================== */

/* ---- viewport ---------------------------------------------------- */

/* How many usable play-area rows remain after reserving HUD rows. */
static int viewport_play_rows(int rows)
{
    int n = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    return n < 1 ? 1 : n;
}

/* Choose the single aspect-correct scale.  Cells are 2× taller than
 * wide; vertical budget is play_rows × ASPECT_CELL_HEIGHT cell-widths.
 * Pick the tighter axis so the view rectangle fits both ways. */
static float viewport_pick_scale(int cols, int play_rows,
                                 float re_range, float im_range)
{
    float horizontal = (float)cols      / im_range;
    float vertical   = (float)play_rows * ASPECT_CELL_HEIGHT / re_range;
    return horizontal < vertical ? horizontal : vertical;
}

/* Clamp & store the terminal dimensions in the viewport.  Anything
 * larger than the static grid limits gets capped. */
static void viewport_set_dimensions(Viewport *v, int rows, int cols)
{
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    v->rows      = rows;
    v->cols      = cols;
    v->play_rows = viewport_play_rows(rows);
}

/* Choose the ℂ-space rectangle that bounds the figure.  re_half is
 * sized to cover the Mandelbrot's real extent (~3.5 units, so
 * 2·re_half ≈ 3.5).  im_half is derived from terminal aspect so
 * the projection is aspect-correct. */
static void viewport_compute_view_rectangle(const Viewport *v,
                                            float *re_half_out,
                                            float *im_half_out)
{
    *re_half_out = VIEW_RE_HALF;
    *im_half_out = VIEW_RE_HALF * (float)v->cols
                                / ((float)v->play_rows * ASPECT_CELL_HEIGHT);
}

/* Place the figure's centre at the screen centre.  The Buddhabrot
 * is symmetric about im = 0 (the real axis), and its mass centres
 * around re = -0.5. */
static void viewport_anchor_center(Viewport *v)
{
    v->center_col  = v->cols / 2;
    v->center_row  = HUD_TOP_ROWS + v->play_rows / 2;
    v->view_mid_re = VIEW_CENTER_RE;
    v->view_mid_im = VIEW_CENTER_IM;
}

/* Recompute the viewport for the current terminal size.
 *
 *     1. Store dimensions      — viewport_set_dimensions
 *     2. Pick view rectangle   — viewport_compute_view_rectangle
 *     3. Pick aspect-fit scale — viewport_pick_scale
 *     4. Anchor figure centre  — viewport_anchor_center
 */
static void viewport_fit(Viewport *v, int rows, int cols)
{
    viewport_set_dimensions(v, rows, cols);

    float re_half, im_half;
    viewport_compute_view_rectangle(v, &re_half, &im_half);

    v->scale = viewport_pick_scale(v->cols, v->play_rows,
                                   2.0f * re_half, 2.0f * im_half);

    viewport_anchor_center(v);
}

/* Project a ℂ-space point to a terminal cell (portrait orientation).
 *   col ← imaginary axis (small im → left,  large im → right)
 *   row ← real axis      (small re → top,   large re → bottom)
 * Vertical projection is multiplied by ASPECT_INV because cells are
 * 2× taller than wide. */
static inline bool viewport_project(const Viewport *v, Complex z,
                                    int *col_out, int *row_out)
{
    int col = v->center_col + (int)((z.im - v->view_mid_im) * v->scale);
    int row = v->center_row + (int)((z.re - v->view_mid_re) * v->scale * ASPECT_INV);

    if (col < 0 || col >= v->cols)                              return false;
    if (row < HUD_TOP_ROWS || row >= v->rows - HUD_BOTTOM_ROWS) return false;

    *col_out = col;
    *row_out = row;
    return true;
}

/* ---- palette ---------------------------------------------------- */

/* Tier index → ncurses colour-pair slot. */
static const int k_tier_pair_slots[N_DENSITY_TIERS] = {
    COL_C1, COL_C2, COL_C3, COL_C4, COL_C5,
};

/* Install the five density-tier pairs from the active palette.
 * Called at startup and whenever the user cycles `t` / `T`. */
static void palette_apply(int idx)
{
    const Palette *p = &g_palettes[idx];
    if (COLORS >= 256) {
        for (int tier = 0; tier < N_DENSITY_TIERS; tier++)
            init_pair(k_tier_pair_slots[tier], p->fg256[tier], COLOR_BLACK);
    } else {
        for (int tier = 0; tier < N_DENSITY_TIERS; tier++)
            init_pair(k_tier_pair_slots[tier], p->fg8[tier], COLOR_BLACK);
    }
}

/* HUD/HINT pairs are theme-independent — bright yellow data line +
 * bright cyan hint line stay legible against any palette. */
static void palette_init_static(void)
{
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(COL_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §5  density grid                                                       */
/* ===================================================================== */

static void grid_clear(DensityGrid *g)
{
    memset(g->counts, 0, sizeof g->counts);
    g->max_count = 0;
}

/* Bump one cell and update the running maximum.  Caller is
 * responsible for bounds-checking col/row (the viewport projector
 * does that). */
static inline void grid_hit(DensityGrid *g, int row, int col)
{
    uint32_t v = ++g->counts[row][col];
    if (v > g->max_count) g->max_count = v;
}

/* Tier-boundary tables, indexed [0..3] for the four breaks between
 * L1/L2, L2/L3, L3/L4, L4/L5.  t < entry[0] → L1, ..., t ≥ entry[3] → L5.
 *
 * Anti uses a wider spread (0.45 .. 0.90) because its dynamic range
 * is bunched at the high end (only attractor cells approach max).
 * Buddha uses a tighter spread so MORE of the figure registers in
 * mid-tones (without this the figure collapses into a flat blob;
 * with it, the gradient rim → body → core reads cleanly). */
static const float k_anti_density_thresholds[4]   = { 0.45f, 0.62f, 0.78f, 0.90f };
static const float k_buddha_density_thresholds[4] = { 0.30f, 0.45f, 0.60f, 0.78f };

/* Map a normalised log-density value `t` ∈ [0, 1] to a tier index
 * 0..4 (L1..L5), or -1 to skip (below the mode-dependent floor). */
static int density_tier(float t, BuddhMode mode)
{
    float floor = (mode == MODE_ANTI) ? DENSITY_FLOOR_ANTI : DENSITY_FLOOR_BUDDHA;
    if (t < floor) return -1;

    const float *thresholds = (mode == MODE_ANTI)
                                ? k_anti_density_thresholds
                                : k_buddha_density_thresholds;

    for (int tier = 0; tier < N_DENSITY_TIERS - 1; tier++)
        if (t < thresholds[tier]) return tier;
    return N_DENSITY_TIERS - 1;
}

/* ===================================================================== */
/* §6  Mandelbrot iteration & chaos game                                  */
/* ===================================================================== */

/* ---- complex arithmetic ----------------------------------------- */

static inline Complex complex_make(float re, float im)
{
    return (Complex){ re, im };
}

/* One Mandelbrot step: z → z² + c.  The squared modulus |z|² is
 * also returned so the caller can do its escape test without a
 * second multiply. */
static inline Complex mandelbrot_step(Complex z, Complex c, float *abs_sq_out)
{
    float new_re = z.re * z.re - z.im * z.im + c.re;
    float new_im = 2.0f * z.re * z.im        + c.im;
    *abs_sq_out  = new_re * new_re + new_im * new_im;
    return complex_make(new_re, new_im);
}

/* ---- Mandelbrot fast-skip regions ------------------------------- *
 *
 * The interior of the Mandelbrot set has two large always-bounded
 * regions for which we can skip the full escape iteration in
 * BUDDHA mode (those c values never escape).  Standard formulas
 * from Mandelbrot-set rendering literature.
 *
 *   Main cardioid:  q(q + cre − 1/4) < 1/4 · cim²
 *                   where q = (cre − 1/4)² + cim²
 *   Period-2 bulb:  |c + 1|² < 1/16
 */
static inline bool in_main_cardioid(Complex c)
{
    float q = (c.re - CARDIOID_CUSP_X) * (c.re - CARDIOID_CUSP_X)
            + c.im * c.im;
    return q * (q + c.re - CARDIOID_CUSP_X) < 0.25f * c.im * c.im;
}

static inline bool in_period2_bulb(Complex c)
{
    float dx = c.re - PERIOD2_BULB_CX;
    return dx * dx + c.im * c.im < PERIOD2_BULB_R2;
}

/* True if c lies in either always-bounded region — fast skip for
 * BUDDHA mode (no escape to find). */
static inline bool in_known_interior(Complex c)
{
    return in_main_cardioid(c) || in_period2_bulb(c);
}

/* ---- escape test (Pass 1) --------------------------------------- *
 *
 * Iterate z from 0 under z → z² + c.  Return true if |z|² exceeds
 * ESCAPE_RADIUS_SQ within max_iter steps; false otherwise. */
static bool mandelbrot_orbit_escapes(Complex c, int max_iter)
{
    Complex z      = complex_make(MANDELBROT_Z0_RE, MANDELBROT_Z0_IM);
    float   abs_sq = 0.0f;
    for (int i = 0; i < max_iter; i++) {
        z = mandelbrot_step(z, c, &abs_sq);
        if (abs_sq > ESCAPE_RADIUS_SQ) return true;
    }
    return false;
}

/* ---- orbit trace + accumulate (Pass 2) -------------------------- */

/* Project one orbit point onto the density grid; if it lands inside
 * the play area, bump its cell's hit counter. */
static inline void plot_orbit_point(DensityGrid *grid, const Viewport *view, Complex z)
{
    int col, row;
    if (viewport_project(view, z, &col, &row))
        grid_hit(grid, row, col);
}

/* Re-iterate z from 0, plotting each orbit point onto the grid.
 *
 *     for each iteration:
 *         plot the current orbit point to the grid
 *         z ← mandelbrot_step(z, c)
 *         (BUDDHA only) stop if the orbit has escaped
 *
 * In BUDDHA mode we terminate early once the orbit escapes —
 * further points fly off to infinity and don't contribute useful
 * structure.  In ANTI mode the orbit is bounded by construction
 * so we always iterate to max_iter. */
static void mandelbrot_trace_orbit(Complex c, int max_iter, bool anti_mode,
                                   DensityGrid *grid, const Viewport *view)
{
    Complex z      = complex_make(MANDELBROT_Z0_RE, MANDELBROT_Z0_IM);
    float   abs_sq = 0.0f;

    for (int i = 0; i < max_iter; i++) {
        plot_orbit_point(grid, view, z);
        z = mandelbrot_step(z, c, &abs_sq);
        if (!anti_mode && abs_sq > ESCAPE_RADIUS_SQ) break;
    }
}

/* ---- sampler ---------------------------------------------------- */

/* Pick one uniformly-random c value from the sample box. */
static inline Complex sampler_pick_c(RandLCG *rng, const ComplexRect *box)
{
    return complex_make(lcg_in_range(rng, box->re_min, box->re_max),
                        lcg_in_range(rng, box->im_min, box->im_max));
}

/* ---- the per-sample driver -------------------------------------- *
 *
 * Reads as the algorithm itself:
 *
 *     1. pick a random c
 *     2. fast-skip if c is in a known-interior region (buddha only)
 *     3. classify c's orbit (Pass 1: does it escape?)
 *     4. accept c if its escape status matches the mode's filter
 *     5. trace the orbit and accumulate density (Pass 2)
 */
static void buddhabrot_run_one_sample(Scene *s)
{
    const MandelbrotPreset *preset = &g_presets[s->preset];
    bool anti_mode = (preset->mode == MODE_ANTI);

    Complex c = sampler_pick_c(&s->rng, &g_sample_box);

    if (!anti_mode && in_known_interior(c)) return;

    bool escaped = mandelbrot_orbit_escapes(c, preset->max_iter);

    /* anti_mode XOR escaped: skip when escape status fails the filter */
    if (anti_mode == escaped) return;

    mandelbrot_trace_orbit(c, preset->max_iter, anti_mode,
                           &s->grid, &s->view);
}

/* Run SAMPLES_PER_TICK samples this tick.  Stops short if the
 * preset's accumulation target has been reached. */
static void buddhabrot_iterate(Scene *s)
{
    for (int i = 0; i < SAMPLES_PER_TICK; i++) {
        if (s->samples_done >= TOTAL_SAMPLES) return;
        buddhabrot_run_one_sample(s);
        s->samples_done++;
    }
}

/* ===================================================================== */
/* §7  render & HUD                                                       */
/* ===================================================================== */

/* Density tier → glyph lookup.  (Tier → colour-pair slot lives in §4
 * with palette_apply as k_tier_pair_slots[].) */
static const chtype k_tier_glyphs[N_DENSITY_TIERS] = {
    GLYPH_L1, GLYPH_L2, GLYPH_L3, GLYPH_L4, GLYPH_L5,
};

/* Plot one density tier into a screen cell.  Peak tier (L5) gets
 * A_BOLD so the hot-spots really pop. */
static void plot_density_cell(int row, int col, int tier)
{
    chtype glyph = k_tier_glyphs[tier];
    int    pair  = k_tier_pair_slots[tier];
    attr_t bold  = (tier == N_DENSITY_TIERS - 1) ? A_BOLD : 0;

    attron(COLOR_PAIR(pair) | bold);
    mvaddch(row, col, glyph);
    attroff(COLOR_PAIR(pair) | bold);
}

/* Clamp log1p output so the log-tone denominator never goes ~0. */
static inline float log1p_safe(float x)
{
    float v = logf(1.0f + x);
    return v < EPS_LOG_MAX ? EPS_LOG_MAX : v;
}

/* Log-tone normalisation: hits → t ∈ [0, 1]. */
static inline float log_tone_map(uint32_t hits, float log_max)
{
    return logf(1.0f + (float)hits) / log_max;
}

/* Walk-bounds for the render pass.  Clipped to both the play area
 * (between the HUD rows) and the static grid limits. */
static void grid_render_walk_bounds(const Viewport *v,
                                    int *row_top, int *row_lim, int *col_lim)
{
    *row_top = HUD_TOP_ROWS;
    *row_lim = v->rows - HUD_BOTTOM_ROWS;
    *col_lim = v->cols;
    if (*row_lim > GRID_ROWS_MAX) *row_lim = GRID_ROWS_MAX;
    if (*col_lim > GRID_COLS_MAX) *col_lim = GRID_COLS_MAX;
}

/* Convert one cell's hit count to a tier and plot it.  Returns
 * silently if the cell is empty or below the mode-dependent floor. */
static void grid_render_one_cell(uint32_t hits, float log_max, BuddhMode mode,
                                 int row, int col)
{
    if (hits == 0) return;
    int tier = density_tier(log_tone_map(hits, log_max), mode);
    if (tier < 0) return;
    plot_density_cell(row, col, tier);
}

/* Render the density grid as glyph-tier ASCII.
 *
 *     1. Find the play-area bounds (clip to grid limits + HUD rows).
 *     2. Compute log of the running max for normalisation.
 *     3. Walk every cell; map hits → tier via log-tone, plot.
 */
static void grid_render(const DensityGrid *g, const Viewport *v, BuddhMode mode)
{
    int row_top, row_lim, col_lim;
    grid_render_walk_bounds(v, &row_top, &row_lim, &col_lim);

    float log_max = log1p_safe((float)g->max_count);

    for (int row = row_top; row < row_lim; row++)
        for (int col = 0; col < col_lim; col++)
            grid_render_one_cell(g->counts[row][col], log_max, mode, row, col);
}

/* ---- HUD --------------------------------------------------------- */

/* Percentage of the preset's sample target reached so far. */
static int hud_compute_pct_done(const Scene *s)
{
    if (TOTAL_SAMPLES <= 0) return HUD_PCT_FULL;
    int pct = s->samples_done * HUD_PCT_FULL / TOTAL_SAMPLES;
    return pct > HUD_PCT_FULL ? HUD_PCT_FULL : pct;
}

/* Format the top HUD data line into `buf`.  Returns string length. */
static int hud_format_data_line(char *buf, int bufsize, const Scene *s, int pct)
{
    return snprintf(buf, bufsize,
        " [%d/%d] %-11s  thm:%-6s  spd:%d  %3d%%  %5.1f fps  %s ",
        s->preset + 1, N_PRESETS,
        g_presets[s->preset].name,
        g_palettes[s->palette].name,
        s->sim_fps, pct, s->fps.display,
        s->paused ? "PAUSED" : "      ");
}

/* Draw a string right-aligned at row 0 with HUD attrs, clipped to
 * the terminal width so we never write past the right edge. */
static void hud_draw_top_right_aligned(const char *str, int len, int cols)
{
    if (len > cols) len = cols;
    int x = cols - len;
    if (x < 0) x = 0;
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, x, "%.*s", len, str);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

/* Top HUD row: compute progress, format line, draw right-aligned. */
static void hud_draw_data(const Scene *s)
{
    int pct = hud_compute_pct_done(s);

    char buf[HUD_BUFFER_BYTES];
    int  len = hud_format_data_line(buf, sizeof buf, s, pct);

    hud_draw_top_right_aligned(buf, len, s->view.cols);
}

static void hud_draw_hint(const Scene *s)
{
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    mvprintw(s->view.rows - 1, 0,
        " q:quit  p:pause  r:reset  [/]:speed  t/T:theme ");
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

static void hud_draw(const Scene *s)
{
    hud_draw_data(s);
    hud_draw_hint(s);
}

/* ===================================================================== */
/* §8  scene & main                                                       */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void on_exit_cleanup(void) { endwin(); }

/* ---- scene action helpers (named for what the user sees) -------- */

/* Clear the grid and reset the sample counter — start the chaos
 * game fresh.  Doesn't change preset, palette, or RNG state. */
static void scene_rebuild(Scene *s)
{
    grid_clear(&s->grid);
    s->samples_done = 0;
}

static void scene_cycle_preset(Scene *s, int dir)
{
    s->preset = (s->preset + dir + N_PRESETS) % N_PRESETS;
    scene_rebuild(s);
}

static void scene_cycle_palette(Scene *s, int dir)
{
    s->palette = (s->palette + dir + N_PALETTES) % N_PALETTES;
    palette_apply(s->palette);
}

static void scene_change_speed(Scene *s, int delta)
{
    int n = s->sim_fps + delta;
    if (n < SIM_FPS_MIN) n = SIM_FPS_MIN;
    if (n > SIM_FPS_MAX) n = SIM_FPS_MAX;
    s->sim_fps = n;
}

/* ---- scene lifecycle -------------------------------------------- */

static void scene_read_term_size(int *rows_out, int *cols_out)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    *rows_out = rows;
    *cols_out = cols;
}

static void scene_init_options(Scene *s)
{
    s->preset       = DEFAULT_PRESET_IDX;
    s->palette      = DEFAULT_PALETTE_IDX;
    s->paused       = false;
    s->sim_fps      = SIM_FPS_DEFAULT;
    s->samples_done = 0;
    s->fps          = (FpsCounter){ .accum_ns = 0, .frames = 0, .display = 0.0 };
}

static void scene_init_rng(Scene *s)
{
    lcg_seed_from_clock(&s->rng);
}

static void scene_init_view_and_grid(Scene *s)
{
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols);
    grid_clear(&s->grid);
}

/* Set the scene to its starting state.
 *
 *     1. options  — preset, palette, pause, FPS, sample counter
 *     2. RNG      — seed from clock
 *     3. view/grid — viewport from terminal; clear grid */
static void scene_init(Scene *s)
{
    scene_init_options(s);
    scene_init_rng(s);
    scene_init_view_and_grid(s);
}

static void scene_resize(Scene *s)
{
    endwin();
    refresh();
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols);
    scene_rebuild(s);
}

/* Handle one keystroke.  Each case maps to a named scene operation. */
static void scene_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESCAPE:  g_quit = 1;                          break;
    case 'p': case 'P': case ' ':         s->paused = !s->paused;              break;
    case 'r': case 'R': case 'n': case 'N': scene_cycle_preset(s, +1);         break;
    case ']':                             scene_change_speed(s, +SIM_FPS_STEP); break;
    case '[':                             scene_change_speed(s, -SIM_FPS_STEP); break;
    case 't':                             scene_cycle_palette(s, +1);          break;
    case 'T':                             scene_cycle_palette(s, -1);          break;
    default: break;
    }
}

/* Drain all pending keystrokes (nodelay() returns ERR when none). */
static void scene_process_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR)
        scene_handle_key(s, ch);
}

/* Advance the chaos game by one tick's worth of samples.  Skips the
 * work entirely while paused or once the accumulation target is hit. */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    if (s->samples_done >= TOTAL_SAMPLES) return;
    buddhabrot_iterate(s);
}

/* ---- frame ------------------------------------------------------- */

static void frame_render(const Scene *s)
{
    erase();
    grid_render(&s->grid, &s->view, g_presets[s->preset].mode);
    hud_draw(s);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ---- ncurses + signal setup ------------------------------------ */

static void register_signal_handlers(void)
{
    atexit(on_exit_cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
}

static void ncurses_setup(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
}

static void colors_setup(void)
{
    start_color();
    palette_init_static();
}

static void app_init(void)
{
    register_signal_handlers();
    ncurses_setup();
    colors_setup();
}

/* Cap dt so a long stall (process suspended/resumed, kernel hiccup)
 * doesn't trigger a spiral of catch-up ticks. */
static long long clamp_dt(long long dt)
{
    return dt > MAX_DT_NS ? MAX_DT_NS : dt;
}

/* Fixed-step simulation accumulator.  Adds dt to the accumulator
 * and ticks the scene every tick_ns interval, leaving the
 * remainder for the next frame.  Decouples the chaos game's
 * sim_fps Hz from the render rate. */
static void run_simulation_ticks(Scene *s, long long *sim_accum, long long dt)
{
    long long tick_ns = NS_PER_SEC / s->sim_fps;
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(s);
        *sim_accum -= tick_ns;
    }
}

/* Sleep what's left of the frame budget so the render loop holds
 * its target rate. */
static void frame_pace_to_target(long long frame_start)
{
    long long elapsed = clock_ns_now() - frame_start;
    clock_sleep_ns(FRAME_NS_60FPS - elapsed);
}

/* ---- main loop --------------------------------------------------- *
 *
 * Each iteration reads as the algorithm itself: process input, run
 * one or more simulation ticks, render the frame, update FPS, sleep
 * to target.  The simulation is decoupled from rendering via the
 * fixed-step accumulator inside `run_simulation_ticks`, so the
 * chaos game ticks at sim_fps Hz regardless of render rate. */
int main(void)
{
    app_init();

    static Scene scene;
    scene_init(&scene);
    palette_apply(scene.palette);

    long long t_prev    = clock_ns_now();
    long long sim_accum = 0;

    while (!g_quit) {
        if (g_resize) { g_resize = 0; scene_resize(&scene); }

        long long t_now = clock_ns_now();
        long long dt    = clamp_dt(t_now - t_prev);
        t_prev = t_now;

        scene_process_input(&scene);
        run_simulation_ticks(&scene, &sim_accum, dt);
        frame_render(&scene);
        fps_tick(&scene.fps, dt);

        frame_pace_to_target(t_now);
    }
    return 0;
}
