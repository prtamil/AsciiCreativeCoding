/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * barnsley.c — Barnsley IFS fractals via the chaos game.
 *
 * An Iterated Function System (IFS) is a small set of contractive
 * affine maps {T_1, T_2, …, T_n} on the plane, each with a selection
 * probability p_i.  Barnsley's theorem guarantees a unique compact
 * attractor — the fractal — which we visualise with the CHAOS GAME:
 *
 *     start anywhere on the plane
 *     loop:
 *         pick a map T_i with probability p_i
 *         (x, y) = T_i(x, y)
 *         plot (x, y)
 *
 * After a brief burn-in the orbit lives on the attractor.  Rather
 * than plot one pixel per iteration we ACCUMULATE hits on a density
 * grid and render the log-normalised density as 4 ASCII tiers.  This
 * lets millions of iterations per second feed naturally into a
 * 30 fps display.
 *
 * 30 built-in presets, grouped by family:
 *   Ferns & leaves (1-5):
 *      Barnsley Fern, Mirror Fern, Cyclosorus, Slim Fern, Maple Leaf
 *   Trees (6-13):
 *      Fractal Tree, Pythagoras, Pine, Wide, Narrow, Asym, Bushy, Tilted
 *   Sierpinski family (14-21):
 *      Sierpinski, Pentagon, Hexagon, Carpet, T-Square, Vicsek,
 *      Cantor Dust, Inverted Sierpinski
 *   Dragons (22-25):
 *      Heighway Dragon, Twindragon, Terdragon, Lévy C
 *   Curves & forms (26-30):
 *      Koch, Lévy Tapestry, Cesàro, Spiral, Crystal
 *
 * Keys:
 *   q  quit       p / space  pause      r  reset (clear grid)
 *   n / N         next / prev preset
 *   t / T         next / prev theme
 *   +/-           adjust iterations per frame
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra barnsley.c -o barnsley -lncurses -lm
 *
 * REFERENCES — for the concepts behind this program.
 *
 *   IFS & chaos game
 *   ── Barnsley, M.F., "Fractals Everywhere", 2nd ed., Academic
 *      Press, 1993.  The canonical book on IFS, the chaos game,
 *      and the fern coefficients we use.
 *   ── Hutchinson, J.E., "Fractals and Self-Similarity", Indiana
 *      University Mathematics Journal 30 (1981), 713–747.  The
 *      paper that introduced IFS rigorously; Barnsley's theorem
 *      on the existence and uniqueness of the attractor.
 *   ── Falconer, K., "Fractal Geometry: Mathematical Foundations
 *      and Applications", 3rd ed., Wiley, 2014.  Modern treatment
 *      of attractor theory and chaos-game convergence.
 *
 *   Random number generation
 *   ── Knuth, D., "The Art of Computer Programming, Vol. 2:
 *      Seminumerical Algorithms", 3rd ed., §3.2.1.  LCG theory.
 *   ── Graham, Knuth & Patashnik, "Concrete Mathematics", 2nd ed.,
 *      §3.3.3.  Source of the multiplier 1664525 and increment
 *      1013904223 used in our LCG.
 *
 *   Rendering
 *   ── Foley, van Dam, Feiner & Hughes, "Computer Graphics:
 *      Principles and Practice", 3rd ed., Addison-Wesley, 2013.
 *      §6 on aspect-preserving window-to-viewport mappings —
 *      Viewport (§4) is a 2D restriction of that pipeline.
 *   ── Draves, S. & Reckase, E., "The Fractal Flame Algorithm",
 *      2003.  https://flam3.com/flame_draves.pdf
 *      The canonical reference for accumulating IFS hits into a
 *      density buffer and rendering with LOG-TONE mapping
 *      (log(1 + hits) / log(1 + max_hits)).  Our 4-tier mapping
 *      in density_tier() is the same technique reduced to ASCII.
 *   ── Bourke, P., "Character representation of greyscale images",
 *      1997.  https://paulbourke.net/dataformats/asciiart/
 *      Origin of the density-ramp glyph ordering ('.' ':' '+' '@').
 *   ── Padala, P., "NCURSES Programming HOWTO", 2005.
 *      https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *      Reference for init_pair, mvaddch, frame pacing, resize
 *      handling — every ncurses API in §4–§8.
 *
 * §1 types & data    §2 random          §3 time
 * §4 view & palette  §5 density grid    §6 IFS & chaos game
 * §7 render & HUD    §8 scene & main
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>          /* memset                                  */
#include <time.h>

/* ===================================================================== */
/* §1  types & data                                                       */
/* ===================================================================== */

/* ---- grid + HUD sizing -------------------------------------------- */
#define GRID_ROWS_MAX        80
#define GRID_COLS_MAX       300
#define HUD_TOP_ROWS          1     /* data row at top                    */
#define HUD_BOTTOM_ROWS       1     /* hint row at bottom                 */

/* ---- iteration limits -------------------------------------------- */
#define ITERS_MIN          1000
#define ITERS_MAX         30000
#define ITERS_DEFAULT      8000
#define ITERS_STEP         1000

/* ---- per-cell hit saturation ------------------------------------- */
#define HITS_CAP          60000u

/* ---- timing ------------------------------------------------------ */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define FRAME_NS         (NS_PER_SEC / 30)        /* 30 fps target      */
#define FPS_WINDOW_MS         500                 /* rolling FPS window */

/* ---- IFS limits -------------------------------------------------- */
#define MAX_IFS_MAPS          8     /* Sierpinski Carpet needs 8        */
#define N_DENSITY_TIERS       4     /* L1..L4                          */
#define N_PRESETS            30
#define N_PALETTES            5

/* ---- LCG parameters (Knuth's minimal standard) ------------------- */
#define LCG_MULTIPLIER  1664525u
#define LCG_INCREMENT   1013904223u

/* ---- density-tier thresholds (normalised log-density in [0, 1]) -- */
#define DENSITY_BG_MAX     0.15
#define DENSITY_L1_MAX     0.35
#define DENSITY_L2_MAX     0.55
#define DENSITY_L3_MAX     0.75

/* ---- density-tier glyphs (Bourke ramp, 4-step) ------------------- */
#define GLYPH_L1    '.'
#define GLYPH_L2    ':'
#define GLYPH_L3    '+'
#define GLYPH_L4    '@'

/* ---- aspect & projection ----------------------------------------- */
#define ASPECT_CELL_HEIGHT      2.0f      /* cells are 2× taller than wide   */
#define ASPECT_INV              0.5f      /* 1 / ASPECT_CELL_HEIGHT          */
#define VIEW_MIDPOINT_FRACTION  0.5f      /* (min + max) / 2 = midpoint      */
#define EDGE_MARGIN_CELLS       1         /* keep 1-cell margin on each side */

/* ---- LCG output extraction --------------------------------------- */
#define LCG_HIGH_BITS_SHIFT     8u        /* drop low 8 bits (worse stats)   */
#define LCG_UNIT_DENOM          (1u << 24)/* 2^24 — divisor for [0, 1)       */
#define LCG_SEED_TIME_MIX       123456789LL  /* mix sec component into seed  */

/* ---- defaults ---------------------------------------------------- */
#define DEFAULT_PRESET_IDX      0
#define DEFAULT_PALETTE_IDX     0

/* ---- misc -------------------------------------------------------- */
#define KEY_ESCAPE          27
#define EPS_LOG_MAX     1e-12         /* guard against log(0)            */
#define EPS_RANGE       1e-6f         /* guard against degenerate view   */
#define ORBIT_SEED_X    0.1f          /* burn-in starting point          */
#define ORBIT_SEED_Y    0.0f


/* ---- AffineMap — one IFS transformation -------------------------- *
 *
 * What it is: one of the affine maps the chaos game iterates.
 * Each map takes a point (x, y) and moves it to a new point:
 *
 *     T(x, y) = ( a·x + b·y + e ,
 *                 c·x + d·y + f )
 *
 * Together with the cumulative probability of being chosen, this is
 * the SMALLEST UNIT the chaos game touches in its inner loop.
 *
 * Why it exists: an IFS is just a list of such maps; the smallest
 * useful list is one entry per map.  Bundling the six coefficients
 * with the selection probability means every chaos step is a single
 * lookup + 6 multiplies + 4 adds.  No global state, no special
 * cases, no math hidden in the inner loop.
 *
 * Worked example — the Barnsley fern's dominant leaflet map (T2):
 *
 *     a =  0.85   b =  0.04
 *     c = -0.04   d =  0.85         cum = 0.86  (85% probability)
 *     e =  0.00   f =  1.60
 *
 * This is a slight rotation (the 0.04 / -0.04 cross-terms) combined
 * with a 0.85× shrink and an upward shift of 1.60.  Iterating it
 * 85% of the time builds the leaflet structure.
 *
 * Why CUMULATIVE probability and not raw p_i: the INVERSE-CDF method
 * (Knuth TAOCP §3.4.1) is the standard trick for sampling from a
 * discrete distribution.  Sorting the maps in monotone-increasing
 * `cum` order lets us draw a uniform rnd ∈ [0, 1) and pick the first
 * map with `rnd ≤ cum` — one branch, no division, no table search.
 *
 * Fields:
 *   a, b, c, d  — the 2×2 matrix coefficients.  Together they
 *                 control rotation, scaling, and shearing.  All
 *                 four are read on every iteration.
 *   e, f        — the translation (offset) added after the matrix
 *                 multiplication.  These shift the map's fixed
 *                 point off the origin.
 *   cum         — CUMULATIVE selection probability in [0, 1].
 *                 Maps inside a preset are listed in monotone
 *                 order; the last entry's cum is exactly 1.0 so a
 *                 rnd at the boundary still matches.  Example for
 *                 the fern (p₁=1%, p₂=85%, p₃=7%, p₄=7%):
 *                       cum₁=0.01, cum₂=0.86, cum₃=0.93, cum₄=1.00.
 *
 * Tip for adding a new fractal: probability should roughly track
 * |det(A)| = |a·d − b·c| (the area-scaling factor of the matrix).
 * Tiny-det maps (like the fern's stem) cover little area and need
 * tiny probability; near-1 dets need large probability for uniform
 * density across the attractor.
 *
 * References:
 *   ── Barnsley, "Fractals Everywhere", ch. 3 — IFS theory and
 *      Hutchinson operators.
 *   ── Knuth, TAOCP Vol. 2, §3.4.1 — the inverse-CDF method for
 *      sampling a discrete distribution. */
typedef struct {
    float a, b, c, d, e, f;
    float cum;
} AffineMap;

/* ---- IFSPreset — a named IFS configuration ----------------------- *
 *
 * What it is: everything you need to describe ONE fractal:
 *   • a name (for the HUD),
 *   • a list of affine maps,
 *   • the gasket-space rectangle where the attractor lives.
 *
 * The chaos-game engine reads exactly these fields; the same engine
 * draws all 30 fractals.  Pick a preset → pick a fractal.
 *
 * Why it exists — the "engine + data" pattern.  The math is the
 * same for every IFS: pick a map, apply, plot.  Different fractals
 * differ only in the LIST OF MAPS and the VIEW BOUNDS.  Wrapping
 * those into one struct lets the rest of the program treat fractals
 * as INTERCHANGEABLE DATA:
 *
 *     g_presets[0]  — the Barnsley fern
 *     g_presets[14] — the Sierpinski triangle
 *     g_presets[22] — the Heighway dragon
 *
 * Changing fractal at runtime is just `s->preset = …;` plus a
 * grid clear and viewport refit.  No new code per fractal.
 *
 * About `n_maps` and the trailing slots: the `maps[]` array is a
 * fixed size (MAX_IFS_MAPS = 8 — Sierpinski Carpet needs 8).
 * Presets with fewer maps leave the trailing slots zero, and
 * `n_maps` tells the chaos game where to stop reading.  No tagged
 * union, no variable-length array, no allocation.
 *
 * About the view rectangle: each fractal has a natural bounding
 * box in (x, y) gasket coords — for the fern roughly
 * [−2.6, 2.6] × [−0.2, 10.2]; for the Sierpinski triangle
 * [−0.1, 1.1] × [−0.1, 1.1].  Viewport (§4) projects this
 * rectangle onto the terminal, aspect-corrected so circles look
 * round and the disc fits inside the play area.
 *
 * Fields:
 *   name          — short identifier shown in the HUD (≤ 14 chars
 *                   keeps the HUD line compact).
 *   n_maps        — count of valid maps in `maps[]` (1..MAX_IFS_MAPS).
 *                   The chaos game reads maps[0..n_maps-1] only.
 *   maps[]        — affine maps in CUMULATIVE PROBABILITY order
 *                   (last entry has cum = 1.0).  Trailing slots are
 *                   zero-initialised by C99 array-init rules and
 *                   ignored by the chaos game.
 *   x_min, x_max,
 *   y_min, y_max  — gasket-space view rectangle that contains the
 *                   attractor.  Used by Viewport to compute the
 *                   projection scale.  Tighter bounds = larger
 *                   on-screen fractal; too tight = clipped tips.
 *
 * References:
 *   ── Barnsley, "Fractals Everywhere", §3.6 — the collage theorem
 *      (how to design an IFS that matches a target shape).
 *   ── Falconer, "Fractal Geometry", ch. 9 — uniqueness and
 *      convergence of the attractor.
 *   ── Wikipedia, "Iterated function system" — accessible summary
 *      with concrete coefficient tables. */
typedef struct {
    const char *name;
    int         n_maps;
    AffineMap   maps[MAX_IFS_MAPS];
    float       x_min, x_max;
    float       y_min, y_max;
} IFSPreset;

/* ---- Palette — a named density-tier colour theme ---------------- *
 *
 * What it is: four colours, one per density tier.  Selecting a
 * palette re-skins the fractal without changing anything else:
 *
 *     L1 (sparsest)  — used for the faintest dots `.`
 *     L2             — for moderate density `:`
 *     L3             — for dense regions    `+`
 *     L4 (densest)   — for the brightest hot-spots `@`
 *
 * Why it exists — keep THEMING SEPARATE from DRAWING.  The renderer
 * never sees raw colour codes; it asks for the colour pair matching
 * a tier index, and that's all it knows.  Adding a new theme is one
 * entry in g_palettes[]; the rendering code is unchanged.  The `t`
 * key just bumps an index and reloads four ncurses pairs.
 *
 * About fg256 vs fg8: modern terminals support 256 colours and we
 * use that path.  Older 8-colour terminals get a graceful fallback
 * (a roughly-similar 8-colour scheme) selected by palette_apply at
 * runtime based on COLORS.
 *
 * About the tier order — outermost (sparsest) → innermost (densest)
 * mirrors how density grows around an attractor.  A "Fire" theme
 * goes dark-red → red → orange → yellow as density rises; an "Ice"
 * theme goes dark-blue → blue → cyan → light-cyan.  The deeper the
 * tier index, the closer to the attractor's CORE the cell is.
 *
 * Fields:
 *   name      — short identifier shown in the HUD (≤ 7 chars keeps
 *               the HUD line compact).
 *   fg256[]   — colour codes for L1..L4 in 256-colour terminals.
 *               XTerm palette positions: 0-15 ANSI + 16-231 RGB cube
 *               + 232-255 grayscale.
 *   fg8[]     — fallback colour codes for 8-colour terminals
 *               (COLOR_BLACK .. COLOR_WHITE).  Used when COLORS < 256.
 *
 * References:
 *   ── XTerm 256-colour palette specification:
 *      https://en.wikipedia.org/wiki/ANSI_escape_code#8-bit
 *   ── ncurses' init_pair() / start_color() (Padala HOWTO §8). */
typedef struct {
    const char *name;
    int  fg256[N_DENSITY_TIERS];
    int  fg8[N_DENSITY_TIERS];
} Palette;

/* ---- RandLCG — a fast linear congruential RNG -------------------- *
 *
 * What it is: a tiny random-number generator that holds 32 bits of
 * state and ticks forward with one multiply and one add:
 *
 *     state = state * LCG_MULTIPLIER + LCG_INCREMENT
 *
 * That's it.  Two operations per random number.
 *
 * Why it exists — speed.  The chaos game makes ONE random draw per
 * iteration.  At 8 000 iterations per frame × 30 fps that's 240 000
 * draws per second.  A call into rand() (or any stdlib RNG with a
 * function-call boundary) would dominate the inner loop.  An inline
 * LCG is a couple of cycles per draw and the compiler can pipeline
 * it inside the chaos game's hot loop.
 *
 * Quality vs. speed trade-off: we DO NOT need cryptographic
 * randomness.  We need uniform [0, 1) values to feed the inverse-CDF
 * sampler for picking which AffineMap to apply.  The LCG's main
 * statistical sin is correlation between successive low bits — see
 * Knuth TAOCP Vol. 2, §3.2.1.  We dodge that by extracting the HIGH
 * 24 BITS in lcg_unit_float (shift right by 8), which behave much
 * better.  Period is 2³² (full period since gcd(LCG_INCREMENT, 2³²)
 * = 1 and LCG_MULTIPLIER ≡ 1 mod 4).
 *
 * About the LCG parameters: the pair (1664525, 1013904223) is the
 * "minimal standard" from Knuth's "Concrete Mathematics" §3.3.3.
 * It's been studied extensively and is known to give reasonable
 * uniform sampling for non-statistical use.
 *
 * Fields:
 *   state — current 32-bit LCG state.  Updated in place by
 *           lcg_next_u32().  MUST be non-zero — a zero state stays
 *           zero forever under the LCG recurrence (lcg_seed_from_clock
 *           guards against this).
 *
 * References:
 *   ── Knuth, TAOCP Vol. 2, §3.2.1 — full LCG theory, spectral test,
 *      and the "minimal standard" choice of multiplier.
 *   ── Graham, Knuth & Patashnik, "Concrete Mathematics", §3.3.3 —
 *      the multiplier 1664525 and increment 1013904223 specifically.
 *   ── O'Neill, "PCG: A Family of Simple Fast Space-Efficient
 *      Statistically Good Algorithms for Random Number Generation"
 *      (2014).  Modern replacement if quality matters — we don't
 *      use it, but the paper explains LCG weaknesses cleanly. */
typedef struct {
    uint32_t state;
} RandLCG;

/* ---- Orbit — chaos-game trajectory state ------------------------- *
 *
 * What it is: the CURRENT POINT being iterated by the chaos game,
 * plus the RNG that picks which map to apply next.  One single
 * point — not a list, not a history.  Each chaos step replaces the
 * point with its image under a randomly-chosen map.
 *
 * The word "orbit" comes from DYNAMICAL SYSTEMS.  An orbit is the
 * sequence of points visited by repeated application of a map
 * (deterministic case) or randomly-chosen maps (our stochastic
 * case).  Barnsley's theorem says: regardless of starting point,
 * the orbit visits every part of the attractor with frequency
 * proportional to the map probabilities.
 *
 * Why it exists — keep the trajectory state in one named bundle.
 * The chaos game needs both the position AND its random source on
 * EVERY iteration.  Bundling them means the hot loop reads from one
 * cache line, and that the iteration is a pure function of one
 * Orbit (+ one IFSPreset reference).
 *
 * About BURN-IN: the orbit doesn't have to START on the attractor.
 * If we begin at (0.1, 0.0) — outside the fern, say — the first
 * dozen or so iterations PULL the point onto the attractor (because
 * every map is a contraction).  After the burn-in, every iterate is
 * effectively a sample from the attractor itself.  We just plot
 * them all — the first dozen out-of-attractor hits are lost in the
 * sea of millions of correct hits and don't affect the rendering.
 *
 * Lifecycle:
 *   • orbit_reset()        — re-seed the position to (0.1, 0).
 *                            Called on grid reset and preset change.
 *   • lcg_seed_from_clock(&o.rng) — non-zero random seed from wall
 *                            clock.  Called once at scene_init.
 *   • chaos_iterate()      — reads + updates (cx, cy) and steps rng
 *                            for each iteration.
 *
 * Fields:
 *   cx, cy — current orbit position in gasket coordinates (not
 *            screen coordinates).  Updated on every chaos step.
 *   rng    — random source used to pick the next map.  Lives
 *            inside the orbit so the inner loop touches only one
 *            struct.
 *
 * References:
 *   ── Barnsley, "Fractals Everywhere", §3.4 — the chaos game /
 *      random iteration algorithm and its convergence.
 *   ── Elton, J.H., "An ergodic theorem for iterated maps",
 *      Ergodic Theory and Dynamical Systems 7 (1987), 481–488.
 *      The rigorous proof that the orbit visits each part of the
 *      attractor with the right frequency. */
typedef struct {
    float    cx, cy;
    RandLCG  rng;
} Orbit;

/* ---- DensityGrid — accumulator for chaos-game hits --------------- *
 *
 * What it is: a 2D array of 16-bit counters, one per terminal cell.
 * Each counter holds how many orbit iterations have landed on that
 * cell.  Think of it as a HISTOGRAM of orbit positions in screen
 * space.
 *
 * Why it exists — DENSITY is the visual signal, not individual
 * dots.  If we just plotted one pixel per iteration, the chaos game
 * would over-paint the same handful of cells thousands of times
 * with no way to tell DENSE regions (lots of hits) from SPARSE ones
 * (few hits).  Counting hits per cell turns "where the attractor
 * is" into "how many times the orbit visited this cell" — which is
 * exactly the invariant measure of the attractor we want to draw.
 *
 * LOG-TONE MAPPING (in grid_render):
 *
 *     normalised_t = log(1 + hits)  /  log(1 + max_hits)
 *
 * Why log: the highest-density cells get hit MILLIONS of times more
 * often than the faintest.  Linear scaling makes everything below
 * the brightest 1% invisible.  Log compresses that dynamic range so
 * the whole density gradient is visible — same trick fractal-flame
 * renderers use.  See Draves & Reckase (2003).
 *
 * SATURATION at HITS_CAP: cells are 16-bit (max 65 535).  We
 * saturate at HITS_CAP = 60 000 to avoid overflow.  Doesn't change
 * the look — because we normalise against per-frame `max_hits`,
 * saturation just clamps the top of the dynamic range; log-tone
 * mapping flattens out at the top anyway.
 *
 * FIXED SIZE: we know the upper bound on terminal size
 * (GRID_ROWS_MAX × GRID_COLS_MAX) at compile time, so the whole
 * grid lives in BSS — no allocation, no resize hassle.  A 80 × 300
 * grid at 2 bytes per cell is only ~48 KB, fits in L1/L2 cache.
 *
 * Lifecycle:
 *   • grid_clear()  — zero every cell.  Called on init, resize,
 *                     'r' reset, and preset change.
 *   • grid_hit()    — increment one cell, saturating.  Called once
 *                     per chaos-game iteration that lands in bounds.
 *
 * Fields:
 *   cells[][] — saturating 16-bit hit counters indexed [row][col]
 *               in TERMINAL coordinates (not gasket coordinates).
 *               row 0 is the top of the screen.
 *
 * References:
 *   ── Draves, S. & Reckase, E., "The Fractal Flame Algorithm",
 *      2003 — the canonical reference for density-buffer
 *      accumulation + log-tone mapping in IFS rendering.
 *   ── Bourke, P., "Character representation of greyscale images",
 *      1997 — origin of the ASCII density-ramp glyphs we use.
 *   ── Hutchinson, J.E., "Fractals and Self-Similarity" (1981) —
 *      §6 introduces the INVARIANT MEASURE supported on the
 *      attractor, which is what our density buffer estimates. */
typedef struct {
    uint16_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
} DensityGrid;

/* ---- Viewport — gasket coords → terminal cells ------------------- *
 *
 * What it is: a precomputed RECIPE for turning a point in the
 * fractal's gasket-space coordinates into a (col, row) cell in the
 * terminal.  Just a handful of numbers that, together, describe
 * "how to place the fractal on screen".
 *
 * Why it exists — keep the projection IN ONE PLACE.  The chaos
 * game's inner loop calls viewport_project once per iteration and
 * doesn't need to know anything about terminal size, HUD rows, or
 * cell aspect ratio.  Resize?  Recompute the Viewport once,
 * everything downstream is automatically reprojected on the next
 * frame.  Switch preset?  Recompute the Viewport.  One struct,
 * one source of projection truth.
 *
 * TWO TRANSFORMS happen on the way to the screen — handled together
 * by `scale`:
 *
 *   1) ASPECT CORRECTION.  Terminal cells aren't square — they're
 *      about 2× taller than wide.  If we treated 1 row = 1 column,
 *      every circle would look squashed and every tree would look
 *      squat.  We work in "cell-WIDTH units" everywhere and
 *      multiply vertical distances by 0.5 when converting to row
 *      counts.
 *
 *   2) ASPECT FIT.  Each preset has a natural view rectangle (e.g.
 *      the fern's [−2.6, 2.6] × [−0.2, 10.2]).  We want it to fit
 *      INSIDE the terminal regardless of whether the terminal is
 *      wide-and-short or tall-and-narrow.  Pick the TIGHTER of
 *      (horizontal budget) vs (vertical budget converted to width
 *      units), and use that single value for `scale`.  The fractal
 *      stays correctly proportioned but might leave empty margin
 *      on the unconstrained axis.
 *
 * THE ARITHMETIC, in plain prose:
 *
 *     col = center_col + ( gasket_x − view_mid_x ) · scale
 *     row = center_row − ( gasket_y − view_mid_y ) · scale · 0.5
 *
 *   • shift gasket-coords so the view midpoint sits at the origin
 *   • multiply by scale (width-units per gasket unit) for x
 *   • multiply by scale·0.5 for y (the aspect correction)
 *   • flip y because gasket-y points UP, terminal row points DOWN
 *   • add the screen-centre offset
 *
 * Lifecycle: rebuilt by viewport_fit() on init, SIGWINCH resize,
 * and preset change.  Otherwise constant for many frames.
 *
 * Fields:
 *   rows, cols   — current terminal dimensions in cells.
 *   play_rows    — number of usable content rows
 *                  (= rows − HUD_TOP_ROWS − HUD_BOTTOM_ROWS).
 *                  Cached here so neither the renderer nor the
 *                  chaos game has to redo the subtraction.
 *   center_col   — column index where the view midpoint lands.
 *                  Equals cols/2 (we always centre horizontally).
 *   center_row   — row index where the view midpoint lands.
 *                  Centred between the HUD rows.
 *   scale        — VISUAL CELLS PER GASKET UNIT, in cell-width
 *                  terms.  Bigger terminal ⇒ bigger scale ⇒ bigger
 *                  drawing.  This is the single "size" knob.
 *   view_mid_x   — gasket-space x at the screen centre.
 *                  Equals (x_min + x_max) / 2 of the preset.
 *   view_mid_y   — gasket-space y at the screen centre.
 *                  Equals (y_min + y_max) / 2 of the preset.
 *
 * References:
 *   ── Foley, van Dam, Feiner & Hughes, "Computer Graphics:
 *      Principles and Practice", 3rd ed., §6 — window-to-viewport
 *      mappings with aspect preservation.  The Viewport here is
 *      the 2D restriction of the full 3D projection pipeline
 *      described in that chapter. */
typedef struct {
    int    rows, cols;
    int    play_rows;
    int    center_col, center_row;
    float  scale;
    float  view_mid_x, view_mid_y;
} Viewport;

/* ---- FpsCounter — rolling FPS measurement ----------------------- *
 *
 * What it is: a tiny stopwatch that counts frames and time inside a
 * rolling WINDOW (default 500 ms), then divides to produce a stable
 * frames-per-second number for the HUD.
 *
 * Why it exists — RAW per-frame FPS jitters wildly.  One frame
 * might take 30 ms because of a scheduling hiccup, the next 5 ms,
 * the next 50 ms while the kernel is paging.  A naive `1 / dt`
 * reading flickers through 20–200 fps every frame and is useless
 * to look at.  Averaging across half a second smooths that out into
 * one stable number that actually reflects sustained performance.
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
 * We refresh the HUD's `display` only at window boundaries, so the
 * number on screen updates ~2× per second.  Between updates the
 * HUD shows the LAST computed value, which is exactly what we
 * want for stability.
 *
 * Fields:
 *   accum_ns — running sum of frame durations within the current
 *              window.  Reset to 0 each window boundary.
 *   frames   — number of frames accumulated within the current
 *              window.  Reset to 0 each window boundary.
 *   display  — last computed FPS value.  This is the number the HUD
 *              actually reads; updated only at window boundaries.
 *
 * References:
 *   ── Standard rolling-average technique; no specific paper. */
typedef struct {
    long long accum_ns;
    int       frames;
    double    display;
} FpsCounter;

/* ---- Scene — the whole program's state -------------------------- *
 *
 * What it is: ONE struct holding everything the running program
 * needs — the density grid, the orbit, the projection, the theme,
 * the iteration count, the pause flag, and the FPS counter.
 *
 * Why it exists — collapse what would otherwise be a dozen
 * scattered globals into one named container.  The main loop ends
 * up reading as a tiny sequence of verbs on a single noun:
 *
 *     scene_process_input(s);   // react to keys
 *     scene_tick(s);            // run one frame of the chaos game
 *     frame_render(s);          // draw grid + HUD
 *
 * Every helper declares its dependency by taking the right
 * sub-pointer.  `chaos_iterate` takes a Scene because it needs the
 * orbit, grid, view, preset, and iters all at once.  `grid_render`
 * takes only the grid and viewport.  `palette_apply` takes only the
 * palette index.  The function signatures spell out which parts of
 * the world each function touches — NO MYSTERY GLOBALS.
 *
 * If you ever needed to checkpoint, replay, or run multiple
 * instances side-by-side, you could because the whole runtime
 * state is one POD struct.
 *
 * FIELDS, GROUPED BY RESPONSIBILITY:
 *
 *   What we're computing (the fractal):
 *     grid         — accumulator for chaos-game hits.  Cleared by
 *                    grid_clear (in scene_rebuild).
 *     orbit        — chaos-game trajectory + its RNG.  Reset by
 *                    orbit_reset (in scene_rebuild); RNG seeded
 *                    once at startup.
 *     preset       — index into g_presets[] of the active IFS.
 *                    Cycled by 'n' / 'N'; refits viewport and
 *                    rebuilds grid + orbit.
 *     iters        — chaos-game iterations to run per frame.
 *                    Adjusted by '+' / '-' in ITERS_STEP increments,
 *                    clamped to [ITERS_MIN, ITERS_MAX].  Higher =
 *                    denser per frame but more CPU.
 *
 *   How we view it (presentation):
 *     view         — gasket→cell projection.  Re-fit on resize and
 *                    on preset change.
 *     palette      — index into g_palettes[] of the active theme.
 *                    Cycled by 't' / 'T'; reloads the four
 *                    CP_L1..CP_L4 colour pairs.  Doesn't touch the
 *                    grid or orbit.
 *
 *   Control:
 *     paused       — chaos game freezes; HUD still updates.
 *                    Toggled by 'p' or space.  Useful for staring
 *                    at the picture without it changing.
 *
 *   Timing:
 *     fps          — rolling FPS counter shown in the HUD.  Updated
 *                    each frame; redisplayed every 500 ms.
 *
 * References:
 *   ── No specific paper — this is just the "passing state by
 *      pointer instead of using globals" pattern, applied
 *      consistently.  See Madhav, "Game Programming in C++"
 *      (Addison-Wesley) ch. 1 for the same idea applied to a
 *      game-engine "Game" class. */
typedef struct {
    /* what we're computing */
    DensityGrid  grid;
    Orbit        orbit;
    int          preset;
    int          iters;

    /* how we view it */
    Viewport     view;
    int          palette;

    /* control */
    bool         paused;

    /* timing */
    FpsCounter   fps;
} Scene;

/* ---- colour-pair slots ------------------------------------------- */
enum {
    CP_HUD  = 1,    /* bright yellow HUD data line                       */
    CP_HINT = 2,    /* bright cyan key hint line                         */
    CP_L1   = 3,    /* density tier 1 (sparsest)                         */
    CP_L2   = 4,
    CP_L3   = 5,
    CP_L4   = 6,    /* density tier 4 (densest)                          */
};

/* ---- palettes (constant config) ---------------------------------- */
static const Palette g_palettes[N_PALETTES] = {
    { "Fern",   { 22, 34, 46, 154 },
                { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN } },
    { "Fire",   { 124, 196, 208, 226 },
                { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW } },
    { "Ice",    { 17, 27, 51, 123 },
                { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN } },
    { "Plasma", { 54, 129, 201, 231 },
                { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN, COLOR_WHITE } },
    { "Mono",   { 240, 245, 250, 255 },
                { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE } },
};

/* ---- presets (constant config) ----------------------------------- *
 * 30 entries, grouped by family.  See file header for the list. */
static const IFSPreset g_presets[N_PRESETS] = {

    /* ────── ferns & leaves ────── */

    /*  1. Barnsley Fern — the canonical IFS.  T1 (1%) makes the stem,
     *  T2 (85%) the dominant leaflets, T3/T4 the side branches. */
    { "Barnsley Fern", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.16f, 0.00f, 0.00f, 0.01f },
        { 0.85f,  0.04f, -0.04f, 0.85f, 0.00f, 1.60f, 0.86f },
        { 0.20f, -0.26f,  0.23f, 0.22f, 0.00f, 1.60f, 0.93f },
        {-0.15f,  0.28f,  0.26f, 0.24f, 0.00f, 0.44f, 1.00f },
      },
      -2.6f, 2.6f,  -0.2f, 10.2f,
    },

    /*  2. Mirror Fern — Barnsley fern reflected across the y-axis. */
    { "Mirror Fern", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.16f, 0.00f, 0.00f, 0.01f },
        { 0.85f, -0.04f,  0.04f, 0.85f, 0.00f, 1.60f, 0.86f },
        { 0.20f,  0.26f, -0.23f, 0.22f, 0.00f, 1.60f, 0.93f },
        {-0.15f, -0.28f, -0.26f, 0.24f, 0.00f, 0.44f, 1.00f },
      },
      -2.6f, 2.6f,  -0.2f, 10.2f,
    },

    /*  3. Cyclosorus Fern — a more spiral-stemmed fern variant. */
    { "Cyclosorus", 4,
      {
        {  0.000f,  0.000f,  0.000f, 0.250f,  0.000f,-0.400f, 0.02f },
        {  0.950f,  0.005f, -0.005f, 0.930f, -0.002f, 0.500f, 0.86f },
        {  0.035f, -0.200f,  0.160f, 0.040f, -0.090f, 0.020f, 0.93f },
        { -0.040f,  0.200f,  0.160f, 0.040f,  0.083f, 0.120f, 1.00f },
      },
      -2.6f, 2.6f,  -0.5f, 6.5f,
    },

    /*  4. Slim Fern — tall narrow fern with longer stem. */
    { "Slim Fern", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.30f, 0.00f, 0.00f, 0.02f },
        { 0.85f,  0.02f, -0.02f, 0.90f, 0.00f, 1.80f, 0.85f },
        { 0.10f, -0.15f,  0.13f, 0.15f, 0.00f, 1.60f, 0.93f },
        {-0.10f,  0.15f,  0.13f, 0.15f, 0.00f, 0.44f, 1.00f },
      },
      -1.5f, 1.5f,  -0.2f, 12.0f,
    },

    /*  5. Maple Leaf — Barnsley's published maple-leaf coefficients. */
    { "Maple Leaf", 4,
      {
        { 0.14f,  0.01f,  0.00f, 0.51f, -0.08f, -1.31f, 0.10f },
        { 0.43f,  0.52f, -0.45f, 0.50f,  1.49f, -0.75f, 0.45f },
        { 0.45f, -0.49f,  0.47f, 0.47f, -1.62f, -0.74f, 0.80f },
        { 0.49f,  0.00f,  0.00f, 0.51f,  0.02f,  1.62f, 1.00f },
      },
      -4.0f, 4.0f,  -2.0f, 4.0f,
    },

    /* ────── trees ────── */

    /*  6. Fractal Tree — symmetric binary tree, 45° branches. */
    { "Fractal Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.42f, -0.42f,  0.42f, 0.42f, 0.00f, 0.20f, 0.45f },
        { 0.42f,  0.42f, -0.42f, 0.42f, 0.00f, 0.20f, 0.85f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -1.1f, 1.1f,   0.0f, 2.0f,
    },

    /*  7. Pythagoras Tree — classic 1/√2 scaling at ±45°. */
    { "Pythagoras", 3,
      {
        { 0.05f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.10f },   /* trunk */
        { 0.50f, -0.50f,  0.50f, 0.50f, 0.00f, 1.00f, 0.55f },   /* left  */
        { 0.50f,  0.50f, -0.50f, 0.50f, 0.50f, 1.50f, 1.00f },   /* right */
      },
      -2.0f, 2.5f,   0.0f, 4.0f,
    },

    /*  8. Pine Tree — tall narrow conifer (small branch angle). */
    { "Pine Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.60f, 0.00f, 0.00f, 0.10f },
        { 0.35f, -0.10f,  0.10f, 0.35f, 0.00f, 0.40f, 0.50f },
        { 0.35f,  0.10f, -0.10f, 0.35f, 0.00f, 0.40f, 0.90f },
        { 0.00f,  0.00f,  0.00f, 0.80f, 0.00f, 0.20f, 1.00f },
      },
      -0.6f, 0.6f,   0.0f, 3.5f,
    },

    /*  9. Wide Tree — broad 60° branch angle. */
    { "Wide Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.35f, -0.70f,  0.70f, 0.35f, 0.00f, 0.20f, 0.45f },
        { 0.35f,  0.70f, -0.70f, 0.35f, 0.00f, 0.20f, 0.85f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -2.0f, 2.0f,   0.0f, 2.5f,
    },

    /* 10. Narrow Tree — tight 15° branch angle. */
    { "Narrow Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.50f, -0.15f,  0.15f, 0.50f, 0.00f, 0.20f, 0.45f },
        { 0.50f,  0.15f, -0.15f, 0.50f, 0.00f, 0.20f, 0.85f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -0.8f, 0.8f,   0.0f, 2.5f,
    },

    /* 11. Asym Tree — unequal branch sizes / angles. */
    { "Asym Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.50f, -0.30f,  0.30f, 0.50f, 0.00f, 0.20f, 0.50f },
        { 0.30f,  0.40f, -0.40f, 0.30f, 0.00f, 0.20f, 0.80f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -1.5f, 1.5f,   0.0f, 2.5f,
    },

    /* 12. Bushy Tree — extra branch maps for a fuller crown. */
    { "Bushy Tree", 5,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.10f },
        { 0.40f, -0.30f,  0.30f, 0.40f, 0.00f, 0.30f, 0.35f },
        { 0.40f,  0.30f, -0.30f, 0.40f, 0.00f, 0.30f, 0.60f },
        { 0.50f, -0.50f,  0.50f, 0.50f, 0.00f, 0.30f, 0.80f },
        { 0.50f,  0.50f, -0.50f, 0.50f, 0.00f, 0.30f, 1.00f },
      },
      -1.5f, 1.5f,   0.0f, 2.5f,
    },

    /* 13. Tilted Tree — trunk slightly leaning. */
    { "Tilted Tree", 4,
      {
        { 0.05f,  0.00f,  0.00f, 0.45f, 0.10f, 0.05f, 0.05f },
        { 0.40f, -0.50f,  0.50f, 0.40f, 0.20f, 0.20f, 0.45f },
        { 0.45f,  0.40f, -0.40f, 0.45f,-0.20f, 0.20f, 0.85f },
        { 0.05f,  0.00f,  0.00f, 0.70f, 0.10f, 0.20f, 1.00f },
      },
      -1.5f, 1.5f,   0.0f, 2.5f,
    },

    /* ────── Sierpinski family ────── */

    /* 14. Sierpinski Triangle — three half-scale copies at the 3 vertices. */
    { "Sierpinski", 3,
      {
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f, 0.00f, 0.333f },
        { 0.5f, 0.f, 0.f, 0.5f, 1.0f, 0.00f, 0.667f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, 0.87f, 1.000f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 15. Sierpinski Pentagon — five copies scaled by 1/(1+φ) ≈ 0.382. */
    { "Pentagon", 5,
      {
        { 0.382f, 0.f, 0.f, 0.382f,  0.000f,  0.618f, 0.20f },
        { 0.382f, 0.f, 0.f, 0.382f, -0.588f,  0.191f, 0.40f },
        { 0.382f, 0.f, 0.f, 0.382f, -0.363f, -0.500f, 0.60f },
        { 0.382f, 0.f, 0.f, 0.382f,  0.363f, -0.500f, 0.80f },
        { 0.382f, 0.f, 0.f, 0.382f,  0.588f,  0.191f, 1.00f },
      },
      -1.1f, 1.1f,  -1.1f, 1.1f,
    },

    /* 16. Sierpinski Hexagon — six 1/3-scale copies at hexagon vertices. */
    { "Hexagon", 6,
      {
        { 0.333f, 0.f, 0.f, 0.333f,  0.000f,  0.667f, 1.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f, -0.577f,  0.333f, 2.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f, -0.577f, -0.333f, 3.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f,  0.000f, -0.667f, 4.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f,  0.577f, -0.333f, 5.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f,  0.577f,  0.333f, 1.000f  },
      },
      -1.1f, 1.1f,  -1.1f, 1.1f,
    },

    /* 17. Sierpinski Carpet — 8 of 9 unit-square subcells at scale 1/3. */
    { "Carpet", 8,
      {
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.000f, 0.125f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.000f, 0.250f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.000f, 0.375f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.333f, 0.500f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.333f, 0.625f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.667f, 0.750f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.667f, 0.875f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.667f, 1.000f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 18. T-Square — four 1/2-scale copies at the 4 corners of [0,1]². */
    { "T-Square", 4,
      {
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f, 0.0f, 0.25f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, 0.0f, 0.50f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f, 0.5f, 0.75f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, 0.5f, 1.00f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 19. Vicsek — five 1/3-scale copies in a plus pattern. */
    { "Vicsek", 5,
      {
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.333f, 0.20f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.333f, 0.40f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.333f, 0.60f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.000f, 0.80f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.667f, 1.00f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 20. Cantor Dust — 4 corners only, scale 1/3. */
    { "Cantor Dust", 4,
      {
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.000f, 0.25f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.000f, 0.50f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.667f, 0.75f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.667f, 1.00f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 21. Inverted Sierpinski — apex points down. */
    { "Inv Sierp", 3,
      {
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f,  0.00f, 0.333f },
        { 0.5f, 0.f, 0.f, 0.5f, 1.0f,  0.00f, 0.667f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, -0.87f, 1.000f },
      },
      -0.1f, 1.1f, -1.1f, 0.1f,
    },

    /* ────── dragons ────── */

    /* 22. Heighway Dragon — two 1/√2 rotations, one inverted. */
    { "Dragon", 2,
      {
        {  0.5f, -0.5f,  0.5f,  0.5f, 0.0f, 0.0f, 0.5f },
        { -0.5f,  0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 1.0f },
      },
      -0.2f, 1.3f, -0.7f, 0.7f,
    },

    /* 23. Twindragon — two Heighway dragons fitted edge-to-edge. */
    { "Twindragon", 2,
      {
        { 0.5f, -0.5f,  0.5f, 0.5f, 0.0f,  0.0f, 0.5f },
        { 0.5f,  0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 1.0f },
      },
      -0.5f, 1.5f, -1.2f, 0.7f,
    },

    /* 24. Terdragon — three rotations of 1/√3 at ±30° / 90°. */
    { "Terdragon", 3,
      {
        { 0.5f, -0.289f,  0.289f, 0.5f, 0.000f, 0.000f, 0.333f },
        { 0.0f, -0.577f,  0.577f, 0.0f, 0.500f, 0.289f, 0.667f },
        { 0.5f, -0.289f,  0.289f, 0.5f, 0.500f,-0.289f, 1.000f },
      },
      -0.2f, 1.2f, -0.5f, 0.7f,
    },

    /* 25. Lévy C Curve — recursive 90° turns at 1/√2 scale. */
    { "Levy C", 2,
      {
        { 0.5f, -0.5f,  0.5f, 0.5f, 0.0f, 0.0f, 0.5f },
        { 0.5f,  0.5f, -0.5f, 0.5f, 0.5f, 0.5f, 1.0f },
      },
      -0.5f, 1.5f, -0.5f, 1.5f,
    },

    /* ────── curves & forms ────── */

    /* 26. Koch Curve — 4 segments at scale 1/3, rotations ±60° in
     *     the middle pair generate the snowflake-edge "bump". */
    { "Koch Curve", 4,
      {
        { 0.333f,  0.000f,  0.000f, 0.333f, 0.000f, 0.000f, 0.25f },
        { 0.167f, -0.289f,  0.289f, 0.167f, 0.333f, 0.000f, 0.50f },
        { 0.167f,  0.289f, -0.289f, 0.167f, 0.500f, 0.289f, 0.75f },
        { 0.333f,  0.000f,  0.000f, 0.333f, 0.667f, 0.000f, 1.00f },
      },
      -0.1f, 1.1f, -0.1f, 0.5f,
    },

    /* 27. Lévy Tapestry — Lévy C generalised to 4 maps, fills a square. */
    { "Levy Tap", 4,
      {
        { 0.50f, -0.20f,  0.20f, 0.50f, 0.0f, 0.0f, 0.25f },
        { 0.50f, -0.20f,  0.20f, 0.50f, 0.5f, 0.5f, 0.50f },
        { 0.50f,  0.20f, -0.20f, 0.50f, 0.0f, 0.5f, 0.75f },
        { 0.50f,  0.20f, -0.20f, 0.50f, 0.5f, 0.0f, 1.00f },
      },
      -0.5f, 1.5f, -0.5f, 1.5f,
    },

    /* 28. Cesàro Curve — two contractive rotations sweep a Cesàro shape. */
    { "Cesaro", 2,
      {
        { 0.4f, -0.3f,  0.3f, 0.4f, 0.0f, 0.0f, 0.5f },
        { 0.4f,  0.3f, -0.3f, 0.4f, 0.6f, 0.0f, 1.0f },
      },
      -0.2f, 1.2f, -0.3f, 0.5f,
    },

    /* 29. Spiral — Barnsley's published 3-map spiral (highly probable
     *     T1 does most of the inward spiraling). */
    { "Spiral", 3,
      {
        { 0.787879f, -0.424242f, 0.242424f, 0.859848f,  1.758647f, 1.408065f, 0.895f },
        {-0.121212f,  0.257576f, 0.151515f, 0.053030f, -6.721654f, 1.377236f, 0.950f },
        { 0.181818f, -0.136364f, 0.090909f, 0.181818f,  6.086107f, 1.568035f, 1.000f },
      },
      -8.0f, 8.0f, -1.0f, 8.0f,
    },

    /* 30. Crystal — four 0.4-scale copies at the cardinal directions;
     *     four-pointed star-like attractor. */
    { "Crystal", 4,
      {
        { 0.4f, 0.f, 0.f, 0.4f,  0.0f,  0.6f, 0.25f },
        { 0.4f, 0.f, 0.f, 0.4f,  0.6f,  0.0f, 0.50f },
        { 0.4f, 0.f, 0.f, 0.4f,  0.0f, -0.6f, 0.75f },
        { 0.4f, 0.f, 0.f, 0.4f, -0.6f,  0.0f, 1.00f },
      },
      -1.1f, 1.1f, -1.1f, 1.1f,
    },
};

/* ===================================================================== */
/* §2  random — fast LCG for chaos-game map selection                     */
/* ===================================================================== */

/* Seed the LCG state from the monotonic clock.  Any non-zero seed
 * works; zero would stay zero forever under the LCG recurrence. */
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

/* ===================================================================== */
/* §3  time — monotonic clock + rolling FPS                               */
/* ===================================================================== */

static long long clock_ns_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* Accumulate one frame's duration; refresh the displayed FPS at
 * each FPS_WINDOW_MS boundary so the HUD reading is stable. */
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

/* ---- preset geometry queries ------------------------------------ */

static float preset_x_range(const IFSPreset *p) {
    float r = p->x_max - p->x_min;
    return r < EPS_RANGE ? EPS_RANGE : r;
}
static float preset_y_range(const IFSPreset *p) {
    float r = p->y_max - p->y_min;
    return r < EPS_RANGE ? EPS_RANGE : r;
}
static float preset_mid_x(const IFSPreset *p) {
    return (p->x_min + p->x_max) * VIEW_MIDPOINT_FRACTION;
}
static float preset_mid_y(const IFSPreset *p) {
    return (p->y_min + p->y_max) * VIEW_MIDPOINT_FRACTION;
}

/* ---- viewport_fit decomposition --------------------------------- */

/* How many rows of play area remain after reserving HUD rows. */
static int viewport_play_rows(int rows) {
    int n = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    return n < 1 ? 1 : n;
}

/* Aspect-correct scale: pick the tighter of (horizontal cell budget)
 * vs (vertical cell budget × 2 for cell aspect), so the view
 * rectangle fits both ways while preserving shape. */
static float viewport_aspect_fit_scale(const Viewport *v, const IFSPreset *p)
{
    float horizontal_budget =  (float)(v->cols      - EDGE_MARGIN_CELLS)
                                 / preset_x_range(p);
    float vertical_budget   =  (float)(v->play_rows - EDGE_MARGIN_CELLS)
                                 * ASPECT_CELL_HEIGHT / preset_y_range(p);
    return horizontal_budget < vertical_budget ? horizontal_budget : vertical_budget;
}

/* Place the gasket origin at the centre of the available play area. */
static void viewport_place_center(Viewport *v, const IFSPreset *p)
{
    v->center_col = v->cols / 2;
    v->center_row = HUD_TOP_ROWS + v->play_rows / 2;
    v->view_mid_x = preset_mid_x(p);
    v->view_mid_y = preset_mid_y(p);
}

/* Recompute the viewport for the current terminal size and active
 * preset.  Called on startup, resize, and any preset change. */
static void viewport_fit(Viewport *v, int rows, int cols, const IFSPreset *p)
{
    /* Step 1 — store the terminal dimensions and play-area height. */
    v->rows      = rows;
    v->cols      = cols;
    v->play_rows = viewport_play_rows(rows);

    /* Step 2 — choose the single scale that fits both axes. */
    v->scale = viewport_aspect_fit_scale(v, p);

    /* Step 3 — anchor the gasket origin at the screen centre. */
    viewport_place_center(v, p);
}

/* ---- viewport_project decomposition ----------------------------- */

/* Convert a gasket-x to a terminal column. */
static inline int viewport_project_col(const Viewport *v, float x)
{
    return v->center_col + (int)((x - v->view_mid_x) * v->scale);
}

/* Convert a gasket-y to a terminal row.  The 0.5× factor is the
 * cell-aspect correction; the minus sign flips gasket-y (points up)
 * to terminal row (points down). */
static inline int viewport_project_row(const Viewport *v, float y)
{
    return v->center_row - (int)((y - v->view_mid_y) * v->scale * ASPECT_INV);
}

/* Is a cell inside the play area (between the two HUD rows)? */
static inline bool viewport_cell_in_play_area(const Viewport *v, int col, int row)
{
    if (col < 0 || col >= v->cols) return false;
    if (row < HUD_TOP_ROWS || row >= v->rows - HUD_BOTTOM_ROWS) return false;
    return true;
}

/* Project a gasket-space point to a terminal cell.  Returns true if
 * the cell is inside the play area; false if it falls outside. */
static inline bool viewport_project(const Viewport *v,
                                    float x, float y,
                                    int *col_out, int *row_out)
{
    int col = viewport_project_col(v, x);
    int row = viewport_project_row(v, y);
    if (!viewport_cell_in_play_area(v, col, row)) return false;
    *col_out = col;
    *row_out = row;
    return true;
}

/* Map from density tier index (0..3) to the ncurses colour-pair slot. */
static const int k_tier_pair_slots[N_DENSITY_TIERS] = {
    CP_L1, CP_L2, CP_L3, CP_L4,
};

/* Pick the foreground colour for one tier, with 8-colour fallback. */
static int palette_tier_fg(const Palette *p, int tier)
{
    return (COLORS >= 256) ? p->fg256[tier] : p->fg8[tier];
}

/* Install the four density-tier pairs from the active palette.
 * Called at startup and on every palette cycle. */
static void palette_apply(int idx)
{
    const Palette *p = &g_palettes[idx];
    for (int tier = 0; tier < N_DENSITY_TIERS; tier++)
        init_pair(k_tier_pair_slots[tier], palette_tier_fg(p, tier), -1);
}

/* HUD pairs are theme-independent — bright yellow data + bright cyan
 * hints so the bars remain legible against any palette. */
static void palette_init_static(void)
{
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, -1);
        init_pair(CP_HINT,  51, -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §5  density grid                                                       */
/* ===================================================================== */

static void grid_clear(DensityGrid *g)
{
    memset(g->cells, 0, sizeof g->cells);
}

/* Bump one cell, saturating at HITS_CAP.  Out-of-range coordinates
 * are ignored (the caller is the viewport; clipping is a defence in
 * depth). */
static inline void grid_hit(DensityGrid *g, int row, int col)
{
    if (row < 0 || row >= GRID_ROWS_MAX) return;
    if (col < 0 || col >= GRID_COLS_MAX) return;
    uint16_t h = g->cells[row][col];
    if (h < (uint16_t)HITS_CAP)
        g->cells[row][col] = h + 1u;
}

/* Find the highest hit count in the visible region — used to
 * normalise log density into [0, 1] each frame. */
static uint32_t grid_max_hits(const DensityGrid *g,
                              int row_top, int row_bot, int cols)
{
    uint32_t max_hits = 1;
    int row_lim = row_bot < GRID_ROWS_MAX ? row_bot : GRID_ROWS_MAX;
    int col_lim = cols    < GRID_COLS_MAX ? cols    : GRID_COLS_MAX;
    for (int r = row_top; r < row_lim; r++)
        for (int c = 0; c < col_lim; c++)
            if (g->cells[r][c] > max_hits)
                max_hits = g->cells[r][c];
    return max_hits;
}

/* Map a normalised log-density value in [0, 1] to a tier index 0..3
 * (or -1 for background, signalling "don't draw"). */
static int density_tier(double t)
{
    if (t < DENSITY_BG_MAX) return -1;
    if (t < DENSITY_L1_MAX) return 0;
    if (t < DENSITY_L2_MAX) return 1;
    if (t < DENSITY_L3_MAX) return 2;
    return 3;
}

/* ===================================================================== */
/* §6  IFS & chaos game                                                   */
/* ===================================================================== */

/* Pick which affine map to apply next.  Iterates the preset's maps
 * in cumulative-probability order and returns the first whose cum is
 * ≥ rnd.  Linear scan is fine because n_maps ≤ 8. */
static const AffineMap *ifs_pick_map(const IFSPreset *p, float rnd)
{
    for (int k = 0; k < p->n_maps; k++)
        if (rnd <= p->maps[k].cum) return &p->maps[k];
    /* Fallback for float-rounding edge cases at rnd ≈ 1.0 */
    return &p->maps[p->n_maps - 1];
}

/* Apply one affine map: (x, y) → (a·x + b·y + e, c·x + d·y + f). */
static inline void affine_apply(const AffineMap *m,
                                float x, float y,
                                float *nx, float *ny)
{
    *nx = m->a * x + m->b * y + m->e;
    *ny = m->c * x + m->d * y + m->f;
}

/* Reset the orbit to a neutral starting point inside the basin of
 * attraction.  Burn-in of the first few iterations brings it onto
 * the actual attractor. */
static void orbit_reset(Orbit *o)
{
    o->cx = ORBIT_SEED_X;
    o->cy = ORBIT_SEED_Y;
}

/* One chaos-game step: draw a random map and apply it in place to
 * the orbit position.  Updates (*x, *y) with the new orbit point. */
static inline void chaos_step_one(const IFSPreset *preset, RandLCG *rng,
                                  float *x, float *y)
{
    const AffineMap *map = ifs_pick_map(preset, lcg_unit_float(rng));
    float nx, ny;
    affine_apply(map, *x, *y, &nx, &ny);
    *x = nx;
    *y = ny;
}

/* Project the orbit's current point to a grid cell and bump its
 * hit counter, if the point lands inside the play area. */
static inline void chaos_plot_orbit_hit(DensityGrid *grid, const Viewport *view,
                                        float x, float y)
{
    int col, row;
    if (viewport_project(view, x, y, &col, &row))
        grid_hit(grid, row, col);
}

/* Run iters chaos-game steps on the orbit, plotting each into the
 * density grid.  This is the program's hot loop — every iteration
 * is one map application + one viewport projection + one grid bump.
 *
 *     for each iteration:
 *         orbit ← T_i(orbit) for a randomly-chosen map i
 *         plot the new orbit point to the grid
 */
static void chaos_iterate(Scene *s)
{
    const IFSPreset *preset = &g_presets[s->preset];
    Orbit           *orbit  = &s->orbit;
    float x = orbit->cx;
    float y = orbit->cy;

    for (int i = 0; i < s->iters; i++) {
        chaos_step_one(preset, &orbit->rng, &x, &y);
        chaos_plot_orbit_hit(&s->grid, &s->view, x, y);
    }

    orbit->cx = x;
    orbit->cy = y;
}

/* ===================================================================== */
/* §7  render & HUD                                                       */
/* ===================================================================== */

/* Density tier → glyph lookup.  (The tier → colour-pair lookup
 * `k_tier_pair_slots[]` lives in §4 with palette_apply.) */
static const chtype k_tier_glyphs[N_DENSITY_TIERS] = {
    GLYPH_L1, GLYPH_L2, GLYPH_L3, GLYPH_L4,
};

/* Plot a single density tier into a screen cell.  The densest tier
 * gets A_BOLD so the peaks of the fractal stand out. */
static void plot_density_cell(int row, int col, int tier)
{
    chtype glyph = k_tier_glyphs[tier];
    int    pair  = k_tier_pair_slots[tier];
    attr_t bold  = (tier == N_DENSITY_TIERS - 1) ? A_BOLD : 0;

    attron(COLOR_PAIR(pair) | bold);
    mvaddch(row, col, glyph);
    attroff(COLOR_PAIR(pair) | bold);
}

/* Clamp log1p output so the denominator can never be ~0. */
static inline double log1p_safe(double x)
{
    double v = log1p(x);
    return v < EPS_LOG_MAX ? EPS_LOG_MAX : v;
}

/* Log-tone map: hits → normalised brightness in [0, 1].
 * See Draves & Reckase, "The Fractal Flame Algorithm" (2003) §4. */
static inline double log_tone_map(uint16_t hits, double log_max)
{
    return log1p((double)hits) / log_max;
}

/* Compute the rectangle of grid cells we should walk this frame.
 * Clipped to both the play area (between HUD rows) and the static
 * grid bounds. */
static void grid_render_walk_bounds(const Viewport *v,
                                    int *row_top, int *row_lim, int *col_lim)
{
    int row_bot = v->rows - HUD_BOTTOM_ROWS;
    *row_top    = HUD_TOP_ROWS;
    *row_lim    = row_bot < GRID_ROWS_MAX ? row_bot : GRID_ROWS_MAX;
    *col_lim    = v->cols < GRID_COLS_MAX ? v->cols : GRID_COLS_MAX;
}

/* Convert one cell's hit count into a density tier + plot.  Returns
 * silently if the cell is empty or below the background threshold. */
static void grid_render_one_cell(uint16_t hits, double log_max, int row, int col)
{
    if (hits == 0) return;
    int tier = density_tier(log_tone_map(hits, log_max));
    if (tier < 0) return;
    plot_density_cell(row, col, tier);
}

/* Render the density grid as glyph-tier ASCII.
 *
 *     1. Find this frame's peak density (for normalisation).
 *     2. Walk every cell in the play area.
 *     3. Map each cell's hit count → tier via log-tone, plot. */
static void grid_render(const DensityGrid *g, const Viewport *v)
{
    int row_top, row_lim, col_lim;
    grid_render_walk_bounds(v, &row_top, &row_lim, &col_lim);

    uint32_t peak    = grid_max_hits(g, row_top, row_lim, v->cols);
    double   log_max = log1p_safe((double)peak);

    for (int row = row_top; row < row_lim; row++)
        for (int col = 0; col < col_lim; col++)
            grid_render_one_cell(g->cells[row][col], log_max, row, col);
}

/* Top HUD row — bright yellow data line. */
static void hud_draw_data(const Scene *s)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
        " [%d/%d] %-14s   theme:%-7s   iters:%5d   %5.1f fps   %s",
        s->preset + 1, N_PRESETS,
        g_presets[s->preset].name,
        g_palettes[s->palette].name,
        s->iters,
        s->fps.display,
        s->paused ? "PAUSED" : "      ");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom HUD row — bright cyan key-hint line. */
static void hud_draw_hint(const Scene *s)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->view.rows - 1, 0,
        " q:quit  p:pause  r:reset  n/N:preset  t/T:theme  +/-:iters ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
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

/* ---- scene action helpers (named for what the user sees) --------- */

/* Clear the density grid and reseed the orbit — restart the chaos
 * game from scratch without changing preset or palette. */
static void scene_rebuild(Scene *s)
{
    grid_clear(&s->grid);
    orbit_reset(&s->orbit);
}

/* Cycle preset by ±1 (wraps); refit viewport; restart the chaos game. */
static void scene_cycle_preset(Scene *s, int dir)
{
    s->preset = (s->preset + dir + N_PRESETS) % N_PRESETS;
    viewport_fit(&s->view, s->view.rows, s->view.cols, &g_presets[s->preset]);
    scene_rebuild(s);
}

/* Cycle palette by ±1 (wraps).  Doesn't touch the grid or orbit. */
static void scene_cycle_palette(Scene *s, int dir)
{
    s->palette = (s->palette + dir + N_PALETTES) % N_PALETTES;
    palette_apply(s->palette);
}

/* Step iters by `delta`, clamped to [ITERS_MIN, ITERS_MAX]. */
static void scene_change_iters(Scene *s, int delta)
{
    int n = s->iters + delta;
    if (n < ITERS_MIN) n = ITERS_MIN;
    if (n > ITERS_MAX) n = ITERS_MAX;
    s->iters = n;
}

/* ---- scene lifecycle --------------------------------------------- */

/* Read terminal dimensions, clamp to grid limits. */
static void scene_read_term_size(int *rows_out, int *cols_out)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    *rows_out = rows;
    *cols_out = cols;
}

/* Defaults for a fresh scene — first preset, first palette, default
 * iteration count, not paused, FPS counter zeroed. */
static void scene_init_options(Scene *s)
{
    s->preset  = DEFAULT_PRESET_IDX;
    s->palette = DEFAULT_PALETTE_IDX;
    s->iters   = ITERS_DEFAULT;
    s->paused  = false;
    s->fps     = (FpsCounter){ .accum_ns = 0, .frames = 0, .display = 0.0 };
}

/* Seed the orbit's RNG from the wall clock and position the orbit
 * at its burn-in starting point. */
static void scene_init_orbit(Scene *s)
{
    lcg_seed_from_clock(&s->orbit.rng);
    orbit_reset(&s->orbit);
}

/* Read the terminal size, fit the viewport to the active preset,
 * and clear the density grid for a fresh draw. */
static void scene_init_view_and_grid(Scene *s)
{
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols, &g_presets[s->preset]);
    grid_clear(&s->grid);
}

/* Set the scene to its starting state.
 *
 *     1. options    — preset, palette, iters, pause, FPS
 *     2. orbit      — RNG seed + start position
 *     3. view/grid  — viewport from terminal + preset; clear grid */
static void scene_init(Scene *s)
{
    scene_init_options(s);
    scene_init_orbit(s);
    scene_init_view_and_grid(s);
}

/* Refit the viewport after SIGWINCH and restart the chaos game. */
static void scene_resize(Scene *s)
{
    endwin();
    refresh();
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols, &g_presets[s->preset]);
    scene_rebuild(s);
}

/* React to a single key.  Each case maps to a named scene operation,
 * so the dispatch table reads as a mapping from input to intent. */
static void scene_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESCAPE:  g_quit = 1;                          break;
    case 'p': case ' ':                   s->paused = !s->paused;              break;
    case 'r': case 'R':                   scene_rebuild(s);                    break;
    case 'n':                             scene_cycle_preset(s,  +1);          break;
    case 'N':                             scene_cycle_preset(s,  -1);          break;
    case 't':                             scene_cycle_palette(s, +1);          break;
    case 'T':                             scene_cycle_palette(s, -1);          break;
    case '+': case '=':                   scene_change_iters(s, +ITERS_STEP);  break;
    case '-':                             scene_change_iters(s, -ITERS_STEP);  break;
    default: break;
    }
}

/* Drain all pending keystrokes (nodelay returns ERR when none). */
static void scene_process_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR)
        scene_handle_key(s, ch);
}

/* Advance the chaos game by one frame's worth of iterations.  Skips
 * the work entirely while paused. */
static void scene_tick(Scene *s)
{
    if (!s->paused) chaos_iterate(s);
}

/* ---- frame ------------------------------------------------------- */

/* Composite the grid + HUD and flush to the terminal. */
static void frame_render(const Scene *s)
{
    erase();
    grid_render(&s->grid, &s->view);
    hud_draw(s);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Pace the frame: update the rolling FPS counter with the elapsed
 * duration, then sleep what's left of the frame budget so we hold
 * a stable rate. */
static void frame_pace_to_target(long long frame_start, FpsCounter *fps)
{
    long long frame_dur = clock_ns_now() - frame_start;
    fps_tick(fps, frame_dur);
    clock_sleep_ns(FRAME_NS - frame_dur);
}

/* ---- ncurses + signal setup -------------------------------------- */

static void app_init(void)
{
    atexit(on_exit_cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);

    start_color();
    use_default_colors();
    palette_init_static();
}

/* ---- main loop --------------------------------------------------- */

int main(void)
{
    app_init();

    static Scene scene;
    scene_init(&scene);
    palette_apply(scene.palette);

    while (!g_quit) {
        if (g_resize) { g_resize = 0; scene_resize(&scene); }

        long long frame_start = clock_ns_now();
        scene_process_input(&scene);
        scene_tick(&scene);
        frame_render(&scene);
        frame_pace_to_target(frame_start, &scene.fps);
    }
    return 0;
}
