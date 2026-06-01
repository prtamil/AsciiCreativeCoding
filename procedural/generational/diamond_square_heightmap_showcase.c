/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diamond_square_heightmap_showcase.c
 *   — Diamond-Square fractal heightmap, animated.
 *
 * DEMO: Watch a fractal terrain grow from four random corners. The
 *       grid starts blue (uncomputed = water level) with four bright
 *       seeded corners. Each tick, a few more cells are computed via
 *       alternating "diamond" and "square" passes — the standard
 *       midpoint-displacement recurrence. Each cell appears in its
 *       colour band by height: water → beach → grass → hills →
 *       mountain → snow. Heights are normalised at render time so all
 *       six bands are always represented. Once every cell is computed,
 *       the finished terrain holds on screen until you ask for a new one
 *       (r = same preset, n/p = new pattern, t/T = recolour).
 *
 * Study alongside: ./bsp_dungeon_showcase.c — another "structured
 *       partition" generator. BSP partitions space recursively for
 *       discrete rooms; Diamond-Square recursively interpolates a
 *       continuous height field. Same recursive-halving philosophy,
 *       very different output.
 *
 * Layer map (see ARCHITECTURE below):
 *   §1 CONFIG & DATA   — constants + all data types (no behaviour)
 *   §2 PERFORMANCE     — timing primitives (monotonic clock, sleep)
 *   §3 LOGIC           — pure decisions: const/value in, value out
 *   §4 SIMULATION      — advances HeightField/Subdivision/Scene; the combine point
 *   §5 EFFECTS         — cosmetic-only state (currently none)
 *   §6 RENDER          — state → screen, read-only
 *   §7 APP             — input, signals, fixed-timestep loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume
 *   r            reset (immediate restart with new seed)
 *   w/b/g/h/m/s  filter to one biome      a  show all biomes
 *   t / T        next / previous theme  (colour only, recolours live)
 *   n / p        next / previous preset (terrain pattern only, regenerates)
 *   + / =  / -   more / fewer cells per tick
 *   ] / [        raise / lower sim tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra diamond_square_heightmap_showcase.c \
 *       -o diamond_square -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Diamond-Square (Fournier, Fussell & Carpenter 1982).
 *                  Generates a fractal 2D height field on a
 *                  (2^n + 1) × (2^n + 1) grid by midpoint displacement:
 *                    1. Seed the four corners with random heights.
 *                    2. DIAMOND step. For every square sub-region of the
 *                       current step size, the centre cell becomes the
 *                       AVERAGE of the four corners plus a random offset.
 *                    3. SQUARE step. For every diamond sub-region just
 *                       formed, each of its four edge midpoints becomes
 *                       the average of its (up to four) cardinal
 *                       neighbours plus a random offset.
 *                    4. Halve the step size; halve the random magnitude
 *                       (the "roughness"); repeat from 2.
 *                  Stops when step size = 1 — every cell has a value.
 *                  The result is a self-similar fractal — at any zoom
 *                  level the terrain looks statistically the same.
 *
 *                  The roughness decay (R *= persistence each level,
 *                  typically persistence = 0.5) is what makes the
 *                  output look like terrain rather than noise: large
 *                  features dominate, with progressively smaller
 *                  features layered on top.
 *
 * Data-structure : `HeightField` — a flat float height[] field with a parallel
 *                  computed[] mask (the renderer paints only computed
 *                  cells; the rest are flat water). `Subdivision` — the
 *                  recurrence cursor (current step / roughness / phase)
 *                  plus a queue of pending (x, y, kind) ops; each
 *                  scene_tick drains a few ops and computes them, which
 *                  animates the reveal.
 *
 * Rendering      : Cells map to ASCII terrain glyphs by height (after
 *                  normalisation against the per-frame min/max):
 *                    h < 0.30 water       '~'  (sky blue)
 *                    h < 0.40 beach       '.'  (yellow)
 *                    h < 0.55 grass       ','  (green)
 *                    h < 0.72 hills       ';'  (forest green)
 *                    h < 0.88 mountain    '#'  (grey)
 *                    else     snow        '@'  (white)
 *                  Without normalisation, heights would drift outside
 *                  [0, 1] from accumulated jitter, clamp at the extremes,
 *                  and the middle bands would rarely render.
 *
 * Performance    : O(N²) where N = 2^n + 1. The work per phase grows
 *                  geometrically (1, 4, 4, 16, 16, …, 4ⁿ) but the inner
 *                  per-cell work is constant (3 adds + a divide + a
 *                  random). We throttle via ops_per_tick so the
 *                  spectacle plays out over ~6–10 seconds regardless
 *                  of grid size.
 *
 * References     : ALGORITHM & FRACTAL THEORY
 *                  • Fournier, Fussell & Carpenter (1982) — "Computer
 *                    rendering of stochastic models", CACM 25(6):371-384.
 *                    The original diamond-square paper; start here.
 *                  • Miller, G.S.P. (1986) — "The definition and rendering
 *                    of terrain maps", SIGGRAPH '86, Computer Graphics
 *                    20(4):39-48. Refines the recurrence and names the
 *                    "creasing" artifact that motivates the EDGE CASES.
 *                  • Saupe, D. — "Algorithms for random fractals", ch. 2 of
 *                    Peitgen & Saupe (eds.), "The Science of Fractal
 *                    Images", Springer 1988. The clearest derivation of
 *                    midpoint displacement and the persistence/roughness
 *                    decay used by the presets.
 *                  • Mandelbrot, B. (1982) — "The Fractal Geometry of
 *                    Nature", W.H. Freeman. Foundational: self-similarity
 *                    and fractional Brownian motion, the "why it looks
 *                    like terrain" theory.
 *                  • Ebert, Musgrave, Peachey, Perlin & Worley (2003) —
 *                    "Texturing & Modeling: A Procedural Approach", 3rd ed.,
 *                    Morgan Kaufmann. Musgrave's terrain chapters are the
 *                    standard practitioner reference (multifractals, erosion).
 *
 *                  PRACTICAL WALKTHROUGHS
 *                  • Hunter Loftis — "Realistic terrain in 130 lines":
 *                    https://www.playfuljs.com/realistic-terrain-in-130-lines/
 *                  • Wikipedia — "Diamond-square algorithm" (quick reference,
 *                    good diagrams): https://en.wikipedia.org/wiki/Diamond-square_algorithm
 *
 *                  RENDERING (height -> glyph/colour, terminal)
 *                  • Paul Bourke — "Character representation of grey scale
 *                    images" (1997): http://paulbourke.net/dataformats/asciiart/
 *                    The luminance->ASCII ramp behind §6's band glyphs.
 *                  • Padala, P. — "NCURSES Programming HOWTO" (TLDP), and
 *                    Gookin, D. (2007) "Programmer's Guide to NCurses",
 *                    Wiley. The terminal-rendering API used in §6.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a height field that looks like real terrain, don't sample
 * random heights independently — neighbouring points must be similar.
 * Diamond-Square gets that by INTERPOLATING between known points and
 * adding only a small RANDOM jitter. The jitter shrinks at every
 * recursion depth, so big features (continents, mountain ranges) get
 * shaped first, and tiny details (boulders, ripples) get layered on
 * later. The whole thing is just "average your neighbours, jitter a
 * bit, recurse smaller".
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a stretched rubber sheet pinned at the four corners. Pinch
 * the centre upward with a small random offset — that's the first
 * DIAMOND step. Now pinch the four edge-midpoints (the points at the
 * middle of each side) — that's the first SQUARE step. The sheet now
 * has 9 fixed points: 4 corners, 4 edge-midpoints, 1 centre. Subdivide
 * each of the 4 quadrants and repeat (with HALF the pinch strength).
 * Eventually every grid point is pinned at a height.
 *
 * Two key observations:
 *   • DIAMOND points see four corners arranged in a SQUARE shape
 *     around them. SQUARE points see (up to) four neighbours arranged
 *     in a DIAMOND shape (cross / +). Hence the names.
 *   • Halving the random magnitude every level is what makes the
 *     output FRACTAL — without that decay you just get noise. The
 *     ratio of decay is called "persistence"; 0.5 is the textbook
 *     value, lower → smoother, higher → noisier.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Allocate a (2^n+1) × (2^n+1) grid of floats. Mark all
 *     cells uncomputed. Set the 4 CORNERS — (0,0), (N,0), (0,N), (N,N)
 *     where N = 2^n — to random values in [0, 1]; mark them computed.
 *  2. step = N, roughness = R0 (≈ 0.5).
 *  3. DIAMOND PHASE at this step:
 *     for y in [step/2, step/2 + step, step/2 + 2·step, …, N - step/2]:
 *       for x in same range:
 *         h = mean(grid[x±step/2][y±step/2]) + uniform(-R, +R)
 *         set grid[x][y] = h
 *  4. SQUARE PHASE at this step:
 *     for every "edge midpoint" (cells where exactly one of x, y is a
 *     multiple of step and the other is offset by step/2):
 *       h = mean of (up to four) cardinal neighbours at distance step/2
 *           + uniform(-R, +R)
 *       set grid[x][y] = h
 *  5. step ← step / 2; R ← R · 0.5; if step ≥ 1 goto 3.
 *  6. Done. Every cell has a height; clamp to [0, 1] for rendering.
 *
 * KEY FORMULAS
 * ────────────
 *  N (grid side − 1)             : N = 2^n
 *  Number of levels              : log₂(N)
 *  Diamond cell coords           : (i + ½)·step, (j + ½)·step  ∈ {1..⌊N/step⌋}²
 *  Square cell coords            : where (x ÷ half + y ÷ half) is odd,
 *                                  with both x, y multiples of half.
 *                                  Equivalently: every other lattice
 *                                  point on the half-step grid that is
 *                                  NOT a diamond point.
 *  Diamond average               : (NW + NE + SW + SE) / 4
 *  Square average                : (N + E + S + W) / count_in_grid
 *                                  (omit out-of-grid neighbours)
 *  Random offset                 : uniform(-R, +R)
 *  Persistence (roughness decay) : R_{level+1} = R_level · 0.5
 *  Total ops                     : N² − 4 (every cell except corners)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • OUT-OF-GRID NEIGHBOURS in the SQUARE step. Edge cells along the
 *    map borders only have 3 valid cardinal neighbours (corners only
 *    have 2). The average must use only the in-bounds neighbours,
 *    NOT pretend missing ones are 0 — using 0 makes the entire border
 *    drop into "deep water" regardless of corner seeds. Always count
 *    contributors and divide by the actual count.
 *
 *  • DIAMOND BEFORE SQUARE. The order matters: every level's SQUARE
 *    phase reads the DIAMOND points it just created. Running square
 *    before diamond at the same level produces noise (the square
 *    averages have nothing to average).
 *
 *  • SEAM AT THE WRAP. If you accidentally treat the grid as periodic
 *    you'll see a sharp seam at the boundary because the corners are
 *    independent random samples. The standard algorithm is NON-periodic.
 *    For a wrap-around terrain (e.g. a sphere), seed corners equal
 *    pairwise — but that's a different algorithm.
 *
 *  • CLAMPING THE OUTPUT. Heights drift outside [0, 1] because of the
 *    accumulated random offsets. For rendering we clamp on read, but
 *    the underlying float can go negative or > 1. Don't clamp during
 *    the algorithm — clipping mid-computation flattens features.
 *
 *  • POWER-OF-TWO PLUS ONE. The grid MUST be (2^n + 1) wide. 33, 65,
 *    129, 257 are the practical sizes; non-power-of-2 widths break
 *    the recursion (step halves and never lands on integer cells).
 *
 *  • PHASE TRANSITION. The animation queue must be fully drained
 *    BEFORE moving to the next phase. If you advance to the next
 *    halving while DIAMOND cells from this level haven't been
 *    computed yet, the SQUARE step reads zeros and produces flat
 *    triangles instead of fractal noise.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At init, exactly 4 cells are computed (the corners). If 0 or 5+,
 *    the seed loop is wrong.
 *  • After every DIAMOND phase, the count of computed cells equals
 *    the previous count + (N/step)². For step=N (level 0) that's +1.
 *  • The final terrain has features at multiple scales — if it looks
 *    like a single big triangle/cone, the random offsets are too
 *    small or the persistence is too aggressive.
 *  • Histogram: a healthy run produces a roughly bell-shaped height
 *    distribution centered near the mean of the corner seeds. A flat
 *    or U-shaped histogram means roughness is wrong.
 *  • Visual: continents, coastlines, mountain interiors should be
 *    obviously distinguishable. If everything is one colour band the
 *    height range is collapsed (corners too similar OR roughness
 *    decay too steep).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Six concern-layers, each with its own §section. The SIGNATURE CONVENTION
 * makes state-change visible at the call site: a pure read takes a `const`
 * pointer (or a plain value); anything that mutates takes a NON-const
 * pointer.
 *
 *   LAYER        §    LIVES IN                       MUTATES
 *   ─────        ─    ───────                        ───────
 *   SIMULATION   §4   grid_* , subdiv_* , scene_*    HeightField, Subdivision,
 *                                                    Scene.state
 *   LOGIC        §3   grid_idx / grid_in_bounds,     nothing — value/const
 *                     band_* , pick_grid_N           in, value out
 *   EFFECTS      §5   (none at present)              cosmetic-only state —
 *                                                    reserved home
 *   RENDER       §6   grid_draw, screen_*, colour    nothing in program
 *                     setup                          state; terminal only
 *   DELAYS       —    paused gate (§4 scene_tick),   gate advance; no
 *                     SCENE_HOLD, frame sleep (§7)   timers, no auto-reset
 *   PERFORMANCE  §2   clock_*; fixed-timestep loop   accumulator, dt cap,
 *                     + ops_per_tick (§4 / §7)       fps sleep, work/tick
 *
 * DATA AGGREGATION vs NARROW FUNCTIONS. All run state hangs off ONE Scene
 * (§1), arranged to read as: WHAT is simulated (HeightField grid, Subdivision
 * subdiv) · HOW the user drives it (Controls controls) · WHERE we are in
 * the lifecycle (SceneState state). But FUNCTIONS still take the NARROWEST
 * type they need — grid_compute_*(HeightField*), grid_draw(const HeightField*),
 * subdiv_step(Subdivision*, HeightField*) — never the whole Scene. So aggregating
 * data on Scene does NOT re-couple the layers: a RENDER function cannot
 * reach the Subdivision, and LOGIC reads only through `const`.
 *
 * ONE COMBINE POINT. scene_tick() (§4) is the only function that advances
 * the simulation per tick, in explicit order: DELAYS gate → PERFORMANCE
 * throttle → SIMULATION step → state transition. The §7 main loop calls it
 * inside the fixed-timestep accumulator; nothing in RENDER or input
 * advances state.
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
/* §1  CONFIG & DATA — constants and all data types (no behaviour)        */
/* ===================================================================== */

enum {
    /* Maximum grid side − 1. We support (2^n + 1) sizes up to 129 — at
     * 129×129 the heightmap holds 16641 cells, which fits a wide
     * terminal and stays under 100KB of state. */
    GRID_N_MAX        = 128,        /* N = 2^n; grid is (N+1) × (N+1) */
    GRID_N_MIN        =   8,        /* minimum useful — gives a 9×9   */

    /* Terminal columns drawn per heightmap cell. Terminal cells are
     * roughly twice as tall as wide, so rendering each cell 2 columns
     * wide makes it visually square AND fills the horizontal space —
     * the terrain reads as a proper map instead of a small skinny
     * block. The grid stays square in CELL units. 3 fills more of a
     * wide terminal; the auto-sizer trades resolution if a terminal is
     * too narrow to fit. */
    CELL_COLS         =   3,

    GRID_W_MAX        = GRID_N_MAX + 1,
    CELLS_MAX         = GRID_W_MAX * GRID_W_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =   8,        /* cells computed per scene_tick */
    OPS_PER_TICK_MAX  = 512,

    FPS_UPDATE_MS     = 500,

    /* HUD layout — the map is centred in the rows BETWEEN these reserves
     * (see grid_draw / pick_grid_N). */
    HUD_TOP_ROWS      =   2,        /* rows 0,1: the two HUD data lines  */
    HUD_BOTTOM_ROWS   =   1,        /* last row: the action-key hint     */

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WATER        =   3,        /* water — sky blue              */
    PAIR_BEACH        =   4,        /* beach — sand yellow           */
    PAIR_GRASS        =   5,        /* grass — green                 */
    PAIR_HILL         =   6,        /* hills — forest green          */
    PAIR_MOUNTAIN     =   7,        /* mountain — grey               */
    PAIR_SNOW         =   8,        /* snow — bright white           */
};

/* Diamond-Square parameters are per-preset now (see presets[] below):
 *   roughness   — initial random offset magnitude
 *   persistence — roughness *= persistence each level (jaggedness)
 *   bias        — biome-mix gamma applied to normalised height
 * Defaults live in the CLASSIC preset (roughness 0.55, persistence 0.5,
 * bias 1.0). */

/* Corner seed heights are drawn from [CORNER_MIN, CORNER_MIN+CORNER_RANGE]
 * = [0.2, 0.8]: a starting spread that is neither trivially flat nor
 * saturated, so midpoint displacement has room to move both ways. */
#define CORNER_MIN       0.2f
#define CORNER_RANGE     0.6f

/* Terrain band thresholds — applied to NORMALIZED heights ([0, 1] after
 * scaling by min/max of the computed grid) so every band is exercised
 * regardless of where the raw float values landed. */
#define BAND_WATER       0.30f
#define BAND_BEACH       0.40f
#define BAND_GRASS       0.55f
#define BAND_HILL        0.72f
#define BAND_MOUNTAIN    0.88f

/* A computed-height span narrower than this is treated as flat — we divide
 * by 1 instead of the span, avoiding a divide-by-zero when only the (equal)
 * corners or a perfectly flat patch have been computed. */
#define HSPAN_EPSILON    1e-6f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Frame pacing (RENDERING) — distinct from the simulation tick rate. The
 * sim advances at controls.sim_fps; the terminal is repainted at RENDER_HZ. */
#define RENDER_HZ        60
#define RENDER_PERIOD_NS (NS_PER_SEC / RENDER_HZ)

/* Largest frame delta the loop will believe. A debugger pause, terminal
 * suspend, or slow resize can produce a huge dt; without this cap the
 * fixed-timestep accumulator would try to "catch up" with a burst of ticks
 * (the classic spiral of death). 100 ms = at most ~6 ticks at 60 Hz. */
#define MAX_FRAME_DT_NS  (100 * NS_PER_MS)

/*
 * Phase kinds — the two alternating steps of the diamond-square recurrence
 * (Fournier, Fussell & Carpenter 1982). Stored in Op.kind and
 * Subdivision.phase, they select which neighbour stencil is averaged:
 *   OP_DIAMOND : set a square's CENTRE from its 4 corner points (the
 *                sources form a SQUARE around the target) + jitter
 *   OP_SQUARE  : set an edge MIDPOINT from its up-to-4 cardinal neighbours
 *                (the sources form a DIAMOND / "+" around the target) + jitter
 * The names describe the shape the SOURCE points make, not the target —
 * the classic source of confusion when first reading the algorithm.
 */
enum { OP_DIAMOND = 0, OP_SQUARE = 1 };

/*
 * Preset — one table row carrying BOTH a colour palette and a set of
 * terrain-generation parameters, but the two are cycled INDEPENDENTLY:
 *   • t / T pick the COLOUR (a row's colors[] becomes the live palette),
 *     recolouring in place with no regeneration.
 *   • n / p pick the terrain PATTERN (a row's roughness/persistence/bias)
 *     and regenerate, leaving the colour untouched.
 * So the colour you see comes from one row and the land shape from
 * another — mix freely (e.g. FIRE colours on ALPINE terrain).
 *
 * The three terrain params are the knobs of fractional Brownian motion
 * via midpoint displacement (Mandelbrot 1982; Saupe, in Peitgen & Saupe
 * 1988). The value RANGES below are those actually spanned by the 15
 * presets, chosen by eye to bracket "calm" to "extreme".
 *
 *   colors[6]   : [WATER, BEACH, GRASS, HILL, MOUNTAIN, SNOW]. Every
 *                 entry sits in the visible half of the 256-colour space
 *                 (≥24, avoiding the 16-23 cube floor and 232-239 dark
 *                 greys, which render near-black — see CLAUDE.md Theme
 *                 Palette Brightness). PAIR_HUD/HINT stay palette-
 *                 independent so the UI is always legible.
 *   roughness   : INITIAL jitter magnitude R0 at the coarsest level —
 *                 the elevation contrast of the largest landforms.
 *                 Presets span ~0.35 (gentle) … 0.75 (dramatic relief).
 *   persistence : the per-level decay ratio R_{k+1} = R_k · persistence
 *                 (the fBm "Hurst" knob). Higher (→0.62) keeps small-scale
 *                 jitter alive → JAGGED, busy coastlines; lower (→0.42)
 *                 damps it fast → SMOOTH, rounded hills. 0.5 is textbook.
 *   bias        : biome-mix gamma applied at render time to the
 *                 NORMALISED height (v -> v^bias; v∈[0,1] so it stays in
 *                 range). >1 pushes heights DOWN → more low bands,
 *                 water-heavy archipelago (presets to 1.55); <1 pushes UP
 *                 → more high bands, peak-heavy alpine (presets to 0.62);
 *                 1.0 leaves the even spread. Purely cosmetic — it changes
 *                 which BAND a height lands in, not the height itself.
 */
typedef struct {
    const char *name;
    short       colors[6];
    float       roughness;
    float       persistence;
    float       bias;
} Preset;

#define N_PRESETS 15

static const Preset presets[N_PRESETS] = {
    /* name        WATER BEACH GRASS HILL  MTN   SNOW    rough  persist bias  */
    { "CLASSIC",  {  33, 221,  34,  28, 244, 231 }, 0.55f, 0.50f, 1.00f },  /* balanced default        */
    { "ATOLL",    {  24,  25,  31,  38,  45,  51 }, 0.45f, 0.45f, 1.55f },  /* smooth, water-heavy     */
    { "ALPINE",   {  25,  31,  39,  51, 189, 231 }, 0.70f, 0.60f, 0.62f },  /* jagged, peak-heavy ice  */
    { "CANYON",   {  52, 124, 160, 196, 208, 226 }, 0.72f, 0.60f, 0.95f },  /* sharp red mesas         */
    { "DUNES",    {  24, 222, 178, 137,  94, 230 }, 0.35f, 0.42f, 1.05f },  /* very smooth sand        */
    { "MATRIX",   {  28,  34,  40,  46,  82, 118 }, 0.55f, 0.55f, 1.00f },  /* digital greens          */
    { "MONO",     { 240, 244, 247, 250, 253, 255 }, 0.60f, 0.55f, 1.00f },  /* greyscale relief        */
    { "NOVA",     {  53,  92, 129, 165, 201, 219 }, 0.58f, 0.52f, 0.90f },  /* purple → pink           */
    { "EARTH",    {  58, 180,  64,  94, 137, 187 }, 0.50f, 0.50f, 1.00f },  /* brown → cream           */
    { "FOREST",   {  28,  64, 100, 136, 172, 187 }, 0.52f, 0.48f, 1.10f },  /* green → olive → tan     */
    { "VOLCANIC", {  52,  88, 124, 166, 202, 220 }, 0.68f, 0.62f, 0.80f },  /* jagged peak-heavy fire  */
    { "GLACIER",  {  31,  38,  45,  51, 159, 231 }, 0.40f, 0.45f, 1.30f },  /* smooth ice, water-heavy */
    { "SUNSET",   {  54,  96, 168, 204, 215, 229 }, 0.50f, 0.50f, 1.00f },  /* warm dusk gradient      */
    { "TROPIC",   {  30,  37,  42,  76, 148, 229 }, 0.48f, 0.50f, 1.40f },  /* lagoon, water-heavy     */
    { "INFERNO",  {  52, 124, 166, 202, 208, 231 }, 0.75f, 0.62f, 0.70f },  /* extreme jagged peaks    */
};

/* Biome filter — the band selected by the w/b/g/h/m/s keys.
 * FILTER_ALL (0) means "show everything". */
enum {
    FILTER_ALL      = 0,
    FILTER_WATER    = 1,
    FILTER_BEACH    = 2,
    FILTER_GRASS    = 3,
    FILTER_HILLS    = 4,
    FILTER_MOUNTAIN = 5,
    FILTER_SNOW     = 6,
};

/*
 * Tile — the visual appearance of one terrain band: the (colour pair,
 * ncurses attribute, ASCII glyph) a normalised height renders as.
 *
 * WHY bundle the three: it lets band_for_height (§3) be a PURE function
 * returning ONE value instead of writing three out-parameters — the
 * classification can't be half-applied, and the call site reads as "this
 * height looks like THIS". The glyph ramp (~ . , ; # @) is a coarse
 * luminance ramp ordered low→high terrain, in the spirit of Bourke's
 * ASCII-art intensity mapping (see References).
 *
 *   pair  : ncurses colour-pair id (one of PAIR_WATER..PAIR_SNOW)
 *   attr  : ncurses attribute — A_NORMAL, or A_BOLD to brighten snow so
 *           the peaks read against any palette
 *   glyph : the ASCII character drawn for this band (0x20–0x7E, ASCII-only
 *           per the project's no-UTF-8 rule)
 */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
} Tile;

/*
 * HeightRange — the bottom and width of the currently-computed height
 * window. RENDER normalises raw heights into [0,1] against THIS window (not
 * a fixed range), so the visible terrain always spans all six bands as the
 * field fills in and the extremes drift.
 *   min  : smallest computed height this frame (the window's floor)
 *   span : max-min, floored at HSPAN_EPSILON → 1.0, so we never divide by 0
 */
typedef struct {
    float min, span;
} HeightRange;

/*
 * Op — one queued cell-computation: the unit of work the animated reveal
 * drains. WHY a queue of ops at all: the diamond-square recurrence would
 * otherwise fill the whole field in one burst; instead each phase enqueues
 * its cells and scene_tick pops a few per frame, so the viewer watches the
 * fractal emerge (Fournier, Fussell & Carpenter 1982).
 *
 * WHY each op is SELF-CONTAINED: `step` and `rough` are SNAPSHOTTED from
 * the recurrence cursor when the op is scheduled, not read live when it is
 * computed. That decouples scheduling from computation — the cursor
 * (Subdivision) may advance to the next level while earlier ops are still
 * pending, and their stencil radius and jitter won't change underneath
 * them. (Read the live cursor instead and a partly-drained phase corrupts.)
 *
 *   x, y  : grid coordinates of the cell to set (each in 0..N)
 *   kind  : OP_DIAMOND (square centre) or OP_SQUARE (edge midpoint) —
 *           selects the neighbour stencil grid_compute_* applies
 *   step  : full step size S at this op's level; the stencil samples at ±S/2
 *   rough : jitter magnitude R at this op's level; the random offset added
 *           is U(-1,1)·R. Snapshotted so per-level decay can't disturb it.
 */
typedef struct {
    int     x, y;
    uint8_t kind;     /* OP_DIAMOND / OP_SQUARE */
    int     step;     /* full step size at scheduling time */
    float   rough;    /* random-offset magnitude at this level */
} Op;

/*
 * HeightField — the 2D scalar field diamond-square fills (the OUTPUT of the
 * algorithm), stored flat and row-major: cell (x,y) is at index y·w + x.
 * Ref: Fournier, Fussell & Carpenter, "Computer rendering of stochastic
 * models", CACM 25(6), 1982 — midpoint displacement on a height field.
 *
 * WHY (2^n + 1) square (w == h == N+1, N a power of two): the algorithm
 * works by repeatedly HALVING the step (N, N/2, … 1). Only a side of
 * 2^n + 1 keeps every midpoint on an integer cell at every level — a
 * non-power-of-two side would, after a few halvings, ask for a cell at a
 * fractional coordinate and the recurrence breaks (see EDGE CASES).
 *
 * WHY a separate computed[] mask rather than a sentinel height: heights are
 * an UNBOUNDED running sum of jittered averages, so every value (incl. 0,
 * negatives) is a legal height — "is this cell set yet?" simply can't be
 * read from height[]. The mask lets RENDER leave un-revealed cells blank
 * during the animated fill.
 *
 * Ownership: SIMULATION (§4) mutates it via HeightField*; RENDER (§6) reads
 * it via const HeightField* and never writes (the signature enforces it).
 *
 *   w, h, N        : grid is (N+1)×(N+1); w == h == N+1; N a power of two
 *   height[]       : per-cell height. RAW and UNBOUNDED — the algorithm
 *                    never clamps (clamping mid-run flattens features);
 *                    RENDER normalises against the live min/max and clamps
 *                    on read instead. CELLS_MAX-sized (fits the largest N)
 *   computed[]     : per-cell "has a value yet" flag (see WHY above)
 */
typedef struct {
    int   w, h, N;
    float height  [CELLS_MAX];
    bool  computed[CELLS_MAX];
} HeightField;

/*
 * Subdivision — the diamond-square recurrence IN PROGRESS: a cursor over
 * the halving levels, plus the queue of cells each level still owes. This
 * is the algorithm's working memory; the HeightField is its output.
 * Ref: midpoint displacement — Fournier/Fussell/Carpenter 1982; the
 * persistence / roughness-decay view is clearest in Saupe, "Algorithms for
 * random fractals" (Peitgen & Saupe, "The Science of Fractal Images", 1988).
 *
 * The recurrence walks step = N, N/2, N/4, … 2, and at each level runs a
 * DIAMOND phase then a SQUARE phase — order matters: square reads the
 * diamond points just created (see EDGE CASES). Each phase ENQUEUES its
 * cells; subdiv_step drains a few per tick, which is what animates the
 * reveal. SIMULATION (§4) owns and mutates it.
 *
 * WHY one append-only queue with two cursors (consumed ops are never
 * removed): the slice [qhead, qtail) is "pending". Leaving drained ops in
 * place lets subdiv_step peek at queue[qhead-1] to recover the LAST phase
 * run (diamond vs square) and pick the next transition — so no separate
 * phase-history variable is needed. queue[] is CELLS_MAX-sized because the
 * field has at most that many cells, so a level's ops always fit.
 *
 *   queue[]      : pending Ops in execution order (a phase is fully queued
 *                  before the next is scheduled)
 *   qhead, qtail : drain cursor (next op to compute) / fill cursor (next
 *                  free slot). qhead == qtail ⇒ current phase exhausted
 *   level_step   : current step S — "how big are the squares we're
 *                  subdividing now". Halves each level
 *   level_rough  : current jitter magnitude R; *= persistence each level.
 *                  Big early (continents), tiny late (ripples) — this DECAY
 *                  is precisely what makes the result fractal, not white noise
 *   persistence  : the decay ratio (copied from the active preset, so a
 *                  preset change only takes effect on the next reset)
 *   phase        : OP_DIAMOND or OP_SQUARE — which stencil this level is on
 *   done         : set true once the next step would drop below 2. We stop
 *                  at step 2 (half = 1): at step 1 the half would be 0 —
 *                  a divide-by-zero AND an infinite loop on `y += half`
 */
typedef struct {
    Op    queue[CELLS_MAX];
    int   qhead, qtail;
    int   level_step;
    float level_rough;
    float persistence;
    int   phase;
    bool  done;
} Subdivision;

/*
 * Controls — the tunable knobs the user drives at runtime: the HOW axis of
 * Scene. WHY its own struct: it separates USER INTENT from ALGORITHM STATE.
 * Nothing here changes except via app_handle_key, and the simulation is a
 * pure function of these knobs plus the RNG seed — so grouping them
 * documents exactly what a viewer can influence, and keeps presentation
 * choices (filter, theme) out of the HeightField/Subdivision that model the
 * maths. Two of the knobs pace the work (ops_per_tick, sim_fps); two pick
 * the look (preset, theme); the rest gate or filter.
 *
 *   paused         : freeze the sim. The DELAYS gate at the top of
 *                    scene_tick early-returns while set (render still runs)
 *   ops_per_tick   : cells computed per tick — the PERFORMANCE throttle
 *                    that paces the reveal. Stepped ×2 / ÷2 by +/- and
 *                    clamped to [OPS_PER_TICK_MIN..MAX] = [1..512]
 *   sim_fps        : simulation tick rate in Hz; the fixed-timestep loop
 *                    runs scene_tick this many times per second. ]/[ adjust
 *                    by SIM_FPS_STEP within [10..240]. Effective reveal
 *                    speed = ops_per_tick × sim_fps cells/second
 *   filter_band    : FILTER_ALL, or one biome (w/b/g/h/m/s) — render only
 *                    that band so a biome can be isolated; 'a' clears it
 *   current_preset : index into presets[] (n/p) — supplies the TERRAIN
 *                    params (roughness/persistence/bias) ONLY
 *   current_theme  : index into presets[] (t/T) — supplies the PALETTE
 *                    ONLY. Independent of current_preset, so any colour
 *                    scheme can dress any terrain shape
 */
typedef struct {
    bool paused;
    int  ops_per_tick;     /* cells computed per tick (+/-)          */
    int  sim_fps;          /* simulation tick rate, Hz (] / [)       */
    int  filter_band;      /* FILTER_ALL or one biome (w/b/g/h/m/s/a) */
    int  current_preset;   /* terrain pattern index, presets[] (n/p) */
    int  current_theme;    /* palette index, presets[] (t/T)         */
} Controls;

/*
 * SceneState — the WHERE-in-lifecycle axis of Scene; a two-state machine.
 *   COMPUTING — the reveal is running: each tick drains ops_per_tick cells;
 *               transitions to HOLD when subdiv_step reports the field done.
 *   HOLD      — the algorithm has finished; the completed terrain stays on
 *               screen indefinitely. Regeneration is USER-driven only
 *               (r / n / p) — there is deliberately no timer or auto-reset,
 *               so a finished map can be studied for as long as you like.
 */
typedef enum {
    SCENE_COMPUTING = 0,
    SCENE_HOLD      = 1,
} SceneState;

/*
 * Scene — the whole simulation in one aggregate, deliberately ordered so it
 * reads top-to-bottom as the three questions you'd ask about any sim:
 *   WHAT is simulated  — a HeightField filled by a Subdivision process
 *   HOW the user drives — the Controls knobs
 *   WHERE in lifecycle  — the SceneState (+ the grid side for this run)
 *
 * CRITICAL: data aggregates HERE, but functions still take the narrowest
 * sub-type they need (grid_compute_*(HeightField*), subdiv_step(...),
 * grid_draw(const HeightField*)) — never Scene* unless they genuinely
 * orchestrate (scene_tick) or report across layers (screen_draw). So
 * aggregation gives one obvious home for state WITHOUT re-coupling the
 * layers §4 kept apart (see ARCHITECTURE).
 */
typedef struct {
    /* WHAT is simulated */
    HeightField grid;            /* the height field being filled        */
    Subdivision subdiv;          /* the recurrence cursor + reveal queue  */

    /* HOW the user drives it */
    Controls    controls;        /* the tunable knobs                     */

    /* WHERE we are in the lifecycle */
    SceneState  state;           /* COMPUTING → HOLD                      */
    int         grid_N;          /* Side N chosen for THIS run from the
                                  * terminal size (pick_grid_N). Distinct
                                  * from HeightField.N: this is the TARGET
                                  * the next scene_reset seeds to. It is
                                  * held on Scene (not just inside the
                                  * field) so a SIGWINCH can record the new
                                  * size, and the following reset rebuilds
                                  * the field to match it. */
} Scene;

/*
 * Screen — the terminal's current size in character cells, and the WHOLE
 * render context the program keeps. WHY so little: ncurses owns the actual
 * cell framebuffer and double-buffers internally, so we only need to cache
 * the dimensions — to centre the map and place the HUD rows. Refreshed from
 * getmaxyx() at init and after every SIGWINCH resize.
 *   cols, rows : terminal width / height, in character cells
 */
typedef struct { int cols, rows; } Screen;

/* ===================================================================== */
/* §2  PERFORMANCE — timing primitives (the fixed-timestep loop is §7)    */
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

/*
 * frame_delta_ns — nanoseconds since the previous frame, CAPPED at
 * MAX_FRAME_DT_NS so one long stall can't spiral the sim. Advances *prev
 * to now (so it is the running "last frame" timestamp the loop owns).
 */
static int64_t frame_delta_ns(int64_t *prev)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *prev;
    *prev = now;
    if (dt > MAX_FRAME_DT_NS) dt = MAX_FRAME_DT_NS;
    return dt;
}

/*
 * pace_frame — sleep so the whole frame lasts one RENDER_PERIOD_NS. Sleeps
 * before terminal I/O so the frame cap holds regardless of draw time.
 * `frame_start` is when this frame's work began; `dt` is that frame's delta.
 */
static void pace_frame(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(RENDER_PERIOD_NS - elapsed);
}

/* ===================================================================== */
/* §3  LOGIC — pure decisions. No state, no I/O; reads only through       */
/*     `const`, returns a value. Cannot be corrupted by §5/§6.            */
/* ===================================================================== */

static inline int grid_idx(const HeightField *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds(const HeightField *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * band_index_for_norm — given a normalised height in [0, 1], return a
 * band index 1..6 (FILTER_WATER..FILTER_SNOW). Same thresholds as
 * band_for_height — kept in lock-step. Used for filtering.
 */
static inline int band_index_for_norm(float v)
{
    if (v < BAND_WATER)    return FILTER_WATER;
    if (v < BAND_BEACH)    return FILTER_BEACH;
    if (v < BAND_GRASS)    return FILTER_GRASS;
    if (v < BAND_HILL)     return FILTER_HILLS;
    if (v < BAND_MOUNTAIN) return FILTER_MOUNTAIN;
    return FILTER_SNOW;
}

static const char *band_name(int filter)
{
    switch (filter) {
    case FILTER_WATER:    return "WATER";
    case FILTER_BEACH:    return "BEACH";
    case FILTER_GRASS:    return "GRASS";
    case FILTER_HILLS:    return "HILLS";
    case FILTER_MOUNTAIN: return "MOUNTAIN";
    case FILTER_SNOW:     return "SNOW";
    default:              return "ALL";
    }
}

static int band_color_pair(int filter)
{
    switch (filter) {
    case FILTER_WATER:    return PAIR_WATER;
    case FILTER_BEACH:    return PAIR_BEACH;
    case FILTER_GRASS:    return PAIR_GRASS;
    case FILTER_HILLS:    return PAIR_HILL;
    case FILTER_MOUNTAIN: return PAIR_MOUNTAIN;
    case FILTER_SNOW:     return PAIR_SNOW;
    default:              return PAIR_HUD;
    }
}

/*
 * band_for_height — classify a (clamped) [0,1] height into the Tile it
 * renders as (colour pair + attribute + ASCII glyph). Pure: height in,
 * Tile out. Bands are ordered by altitude; boundaries (BAND_*) are in §1.
 */
static Tile band_for_height(float h)
{
    if (h < BAND_WATER)    return (Tile){ PAIR_WATER,    A_NORMAL, '~' };
    if (h < BAND_BEACH)    return (Tile){ PAIR_BEACH,    A_NORMAL, '.' };
    if (h < BAND_GRASS)    return (Tile){ PAIR_GRASS,    A_NORMAL, ',' };
    if (h < BAND_HILL)     return (Tile){ PAIR_HILL,     A_NORMAL, ';' };
    if (h < BAND_MOUNTAIN) return (Tile){ PAIR_MOUNTAIN, A_NORMAL, '#' };
    return (Tile){ PAIR_SNOW, A_BOLD, '@' };
}

/*
 * normalize01 — map a raw height into [0,1] against [min, min+span], then
 * clamp. This is the per-cell "where in the visible range does this height
 * sit?" — the input to band classification.
 */
static inline float normalize01(float value, float min, float span)
{
    float v = (value - min) / span;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

/*
 * computed_height_range — the [min,max] window over the cells set so far,
 * span floored at HSPAN_EPSILON. Pure scan; RENDER normalises against this
 * each frame so all six bands stay in use as the extremes drift.
 */
static HeightRange computed_height_range(const HeightField *g)
{
    float min = 1.0f, max = 0.0f;
    bool any = false;
    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        if (!g->computed[i]) continue;
        float v = g->height[i];
        if (!any) { min = v; max = v; any = true; }
        else { if (v < min) min = v; if (v > max) max = v; }
    }
    float span = (max - min > HSPAN_EPSILON) ? (max - min) : 1.0f;
    return (HeightRange){ min, span };
}

/*
 * cardinal_neighbour_mean — mean of the in-bounds N/E/S/W neighbours at
 * distance `half` (the SQUARE-step stencil). Returns false (leaving *mean)
 * if NONE are in bounds; counts only the in-bounds ones so a border cell
 * with 3 neighbours averages by 3, never by a phantom 4 (see EDGE CASES).
 */
static bool cardinal_neighbour_mean(const HeightField *g, int x, int y,
                                    int half, float *mean)
{
    float sum = 0.0f;
    int count = 0;
    if (grid_in_bounds(g, x - half, y)) { sum += g->height[grid_idx(g, x - half, y)]; count++; }
    if (grid_in_bounds(g, x + half, y)) { sum += g->height[grid_idx(g, x + half, y)]; count++; }
    if (grid_in_bounds(g, x, y - half)) { sum += g->height[grid_idx(g, x, y - half)]; count++; }
    if (grid_in_bounds(g, x, y + half)) { sum += g->height[grid_idx(g, x, y + half)]; count++; }
    if (count == 0) return false;
    *mean = sum / (float)count;
    return true;
}

/*
 * subdivision_progress — which halving level the recurrence is on, and how
 * many there are in total. total = log2(N); current = halvings done + 1.
 * Both N and level_step are powers of two, so plain right-shift counting.
 */
static void subdivision_progress(const HeightField *g, const Subdivision *sd,
                                 int *current, int *total)
{
    int tot = 0;
    for (int v = g->N; v > 1; v >>= 1) tot++;
    int cur = 1;
    for (int v = g->N; v > sd->level_step && v > 1; v >>= 1) cur++;
    if (cur > tot) cur = tot;
    *current = cur;
    *total   = tot;
}

/*
 * pick_grid_N — choose the largest N=2^n such that the resulting
 * (N+1) × (N+1) grid fits inside the terminal with HUD margins.
 *
 * Falls back to GRID_N_MIN if the terminal is too small.
 */
static int pick_grid_N(int cols, int rows)
{
    int avail_w = cols / CELL_COLS;                       /* each cell is CELL_COLS wide */
    int avail_h = rows - (HUD_TOP_ROWS + HUD_BOTTOM_ROWS); /* rows left for the map      */
    int min_dim = (avail_w < avail_h) ? avail_w : avail_h;
    int N = 1;
    while ((N * 2) + 1 <= min_dim && (N * 2) <= GRID_N_MAX) {
        N *= 2;
    }
    if (N < GRID_N_MIN) N = GRID_N_MIN;
    if (N > GRID_N_MAX) N = GRID_N_MAX;
    return N;
}

/* ===================================================================== */
/* §4  SIMULATION — advances state. Each function takes the narrowest     */
/*     mutable type it needs (HeightField* / Subdivision*); scene_tick()   */
/*     is the one combine point — nothing else advances the simulation.   */
/* ===================================================================== */

/*
 * rand_signed — uniform float in [-1, 1]. The random offset per cell
 * (scaled by the level roughness). NOT pure: it advances rand()'s hidden
 * state, so it is the stochastic SOURCE for the simulation, not a LOGIC
 * helper. rand() quality is fine — artifacts would need statistics to spot.
 */
static inline float rand_unit(void)   /* uniform in [0, 1] */
{
    return (float)rand() / (float)RAND_MAX;
}
static inline float rand_signed(void) /* uniform in [-1, 1] */
{
    return rand_unit() * 2.0f - 1.0f;
}

/*
 * grid_set — commit a freshly-computed height into the field: store it and
 * mark the cell computed. The one place a cell value is written (called by
 * both compute stencils and the corner seeding).
 */
static void grid_set(HeightField *g, int idx, float value)
{
    g->height[idx]   = value;
    g->computed[idx] = true;
}

/*
 * subdiv_schedule_diamond — append every diamond cell of the current
 * level to the queue (mutates Subdivision; reads the grid side N).
 * Ordering: row-major raster sweep. Diamond cells sit at the centres of
 * squares of the current step: (S/2 + i*S, S/2 + j*S).
 */
static void subdiv_schedule_diamond(Subdivision *sd, const HeightField *g)
{
    sd->phase = OP_DIAMOND;
    int S = sd->level_step;
    int half = S / 2;
    for (int y = half; y < g->N; y += S) {
        for (int x = half; x < g->N; x += S) {
            sd->queue[sd->qtail++] = (Op){
                .x = x, .y = y, .kind = OP_DIAMOND,
                .step = S, .rough = sd->level_rough
            };
        }
    }
}

/*
 * subdiv_schedule_square — append every square (cross-midpoint) cell of
 * the current level (mutates Subdivision; reads the grid side N). Square
 * cells are the "edge midpoints" of the diamond regions: lattice points
 * on the half-step grid where (x/half + y/half) is ODD.
 */
static void subdiv_schedule_square(Subdivision *sd, const HeightField *g)
{
    sd->phase = OP_SQUARE;
    int S = sd->level_step;
    int half = S / 2;
    for (int y = 0; y <= g->N; y += half) {
        /* On row y: if y/half is even, x_start = half (offset);
         * if odd, x_start = 0 (aligned). Both sequences step by S. */
        int x_start = ((y / half) & 1) ? 0 : half;
        for (int x = x_start; x <= g->N; x += S) {
            sd->queue[sd->qtail++] = (Op){
                .x = x, .y = y, .kind = OP_SQUARE,
                .step = S, .rough = sd->level_rough
            };
        }
    }
}

/*
 * grid_compute_diamond — the DIAMOND step: set a square's centre to the
 * mean of its four diagonal corners (at ±half) plus a random displacement.
 * Mutates only the HeightField; the Op carries step+rough, so this never
 * touches the Subdivision.
 */
static void grid_compute_diamond(HeightField *g, const Op *op)
{
    int half = op->step / 2;
    int x = op->x, y = op->y;

    float corner_mean = 0.25f * (
        g->height[grid_idx(g, x - half, y - half)] +
        g->height[grid_idx(g, x + half, y - half)] +
        g->height[grid_idx(g, x - half, y + half)] +
        g->height[grid_idx(g, x + half, y + half)]);
    float displacement = rand_signed() * op->rough;   /* midpoint displacement */

    grid_set(g, grid_idx(g, x, y), corner_mean + displacement);
}

/*
 * grid_compute_square — the SQUARE step: set an edge midpoint to the mean
 * of its in-bounds cardinal neighbours (at ±half) plus a random
 * displacement. Border cells average over fewer than 4 (see EDGE CASES).
 */
static void grid_compute_square(HeightField *g, const Op *op)
{
    int half = op->step / 2;
    int x = op->x, y = op->y;

    float neighbour_mean;
    if (!cardinal_neighbour_mean(g, x, y, half, &neighbour_mean))
        return;                                        /* no neighbours — defensive */
    float displacement = rand_signed() * op->rough;    /* midpoint displacement */

    grid_set(g, grid_idx(g, x, y), neighbour_mean + displacement);
}

/*
 * subdiv_advance_level — move to the next level: halve step, decay
 * roughness, schedule the next DIAMOND phase. The smallest useful step
 * is 2 (half=1); at step=1 the half would be 0 and schedule_square would
 * divide by zero AND infinite-loop on `y += half=0`. So we stop when the
 * next step would be < 2.
 */
static void subdiv_advance_level(Subdivision *sd, const HeightField *g)
{
    int next = sd->level_step / 2;
    if (next < 2) {
        sd->done = true;
        return;
    }
    sd->level_step  = next;
    sd->level_rough *= sd->persistence;
    subdiv_schedule_diamond(sd, g);
}

/*
 * grid_compute_op — apply the stencil for this op's phase (DIAMOND or
 * SQUARE) to the field. The dispatch the stepper uses to drain one cell.
 */
static void grid_compute_op(HeightField *g, const Op *op)
{
    if (op->kind == OP_DIAMOND) grid_compute_diamond(g, op);
    else                        grid_compute_square (g, op);
}

/*
 * subdiv_open_next — the current phase's queue is drained; schedule the
 * next one. After a DIAMOND phase comes the SQUARE phase at the SAME step;
 * after a SQUARE phase, drop to the next finer level. The last-run phase is
 * recovered from queue[qhead-1] (the queue is never re-shrunk). Returns
 * false when the recurrence is complete (nothing more to schedule).
 */
static bool subdiv_open_next(Subdivision *sd, const HeightField *g)
{
    if (sd->qhead == 0) return false;   /* pre-init: nothing was ever scheduled */

    uint8_t last_phase = sd->queue[sd->qhead - 1].kind;
    if (last_phase == OP_DIAMOND) subdiv_schedule_square(sd, g);
    else                          subdiv_advance_level(sd, g);

    if (sd->done) return false;
    return sd->qhead < sd->qtail;       /* true iff something is now queued */
}

/*
 * subdiv_step — advance the recurrence by ONE cell. Returns false when the
 * algorithm is finished. Reads as: stop if done → if the phase is drained,
 * open the next (or finish) → pop one op → compute it.
 */
static bool subdiv_step(Subdivision *sd, HeightField *g)
{
    if (sd->done) return false;
    if (sd->qhead >= sd->qtail && !subdiv_open_next(sd, g)) return false;

    Op op = sd->queue[sd->qhead++];
    grid_compute_op(g, &op);
    return true;
}

/*
 * grid_clear — size the field to (N+1)² and mark every cell uncomputed.
 * w MUST equal N + 1; the caller picks N as a power of 2.
 */
static void grid_clear(HeightField *g, int N)
{
    g->N = N;
    g->w = N + 1;
    g->h = N + 1;

    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        g->height[i]   = 0.0f;
        g->computed[i] = false;
    }
}

/*
 * grid_seed_corners — plant the four corner heights, the fixed points the
 * whole recurrence interpolates between. Each is uniform in
 * [CORNER_MIN, CORNER_MIN+CORNER_RANGE].
 */
static void grid_seed_corners(HeightField *g)
{
    int N = g->N;
    int corners[4][2] = {{0, 0}, {N, 0}, {0, N}, {N, N}};
    for (int i = 0; i < 4; i++) {
        int idx = grid_idx(g, corners[i][0], corners[i][1]);
        grid_set(g, idx, CORNER_MIN + rand_unit() * CORNER_RANGE);
    }
}

/* grid_seed — fresh field: clear, then plant the corner fixed points. */
static void grid_seed(HeightField *g, int N)
{
    grid_clear(g, N);
    grid_seed_corners(g);
}

/*
 * subdiv_begin — start the recurrence over the (already-seeded) grid:
 * reset the cursor to step=N with the preset's roughness/persistence and
 * schedule the first DIAMOND phase. Reads the grid side; mutates the
 * Subdivision.
 */
static void subdiv_begin(Subdivision *sd, const HeightField *g,
                         float roughness, float persistence)
{
    sd->qhead = 0;
    sd->qtail = 0;
    sd->level_step  = g->N;
    sd->level_rough = roughness;
    sd->persistence = persistence;
    sd->done = false;
    subdiv_schedule_diamond(sd, g);   /* sets phase = OP_DIAMOND */
}

static void scene_reset(Scene *s)
{
    const Preset *p = &presets[s->controls.current_preset];
    grid_seed(&s->grid, s->grid_N);
    subdiv_begin(&s->subdiv, &s->grid, p->roughness, p->persistence);
    s->state = SCENE_COMPUTING;
    /* controls and grid_N are preserved across resets — pressing 'r' or a
     * filter key gives a fresh heightmap with the same knobs applied. */
}

static void scene_init(Scene *s, int grid_N)
{
    memset(s, 0, sizeof *s);
    s->controls.paused         = false;
    s->controls.ops_per_tick   = OPS_PER_TICK_DEF;
    s->controls.sim_fps        = SIM_FPS_DEFAULT;
    s->controls.filter_band    = FILTER_ALL;
    s->controls.current_preset = 0;
    s->controls.current_theme  = 0;
    s->grid_N = grid_N;
    scene_reset(s);
}

/*
 * scene_tick — THE COMBINE POINT. The only function that advances the
 * simulation per tick, in explicit layer order:
 *   1. DELAYS      — paused gate: nothing advances while paused.
 *   2. PERFORMANCE — ops_per_tick bounds the work done this tick.
 *   3. SIMULATION  — subdiv_step advances Subdivision+HeightField one cell at a time.
 *   4. (state transition COMPUTING → HOLD when the algorithm finishes.)
 * EFFECTS (§5) would run here too if any existed (none currently).
 * It takes the whole Scene because it ORCHESTRATES the layers — but the
 * functions it calls each take the narrow Subdivision* / HeightField*.
 */
static void scene_tick(Scene *s)
{
    /* 1. DELAYS — paused gate. */
    if (s->controls.paused) return;

    switch (s->state) {

    case SCENE_COMPUTING:
        /* 2. PERFORMANCE throttle → 3. SIMULATION step. */
        for (int i = 0; i < s->controls.ops_per_tick; i++) {
            if (!subdiv_step(&s->subdiv, &s->grid)) {
                s->state = SCENE_HOLD;   /* 4. done — hold the finished map */
                break;
            }
        }
        break;

    case SCENE_HOLD:
        /* DELAYS — finished terrain stays put; a new one only on demand. */
        break;
    }
}

/* ===================================================================== */
/* §5  EFFECTS — cosmetic-only state (glows, trails, flashes).            */
/* ===================================================================== */
/*
 * Currently EMPTY. The earlier gold compute-flash and yellow supernova
 * reset-flash lived here (per-cell decaying glow buffers — a `Glow` type
 * advanced in the tick alongside SIMULATION, read by RENDER). They were
 * removed. This is the reserved home if a purely cosmetic layer returns:
 * such state must be advanced inside scene_tick (§4) AFTER the simulation
 * step and read-only from RENDER (§6) — never feeding back into
 * SIMULATION or LOGIC.
 */

/* ===================================================================== */
/* §6  RENDER — state → screen. Draw functions take the narrowest `const` */
/*     state they need (grid_draw → const HeightField*) and mutate only    */
/*     the terminal. Colour setup lives here (it writes ncurses pairs).   */
/* ===================================================================== */

/*
 * palette_apply — install one entry's 6 terrain colour pairs (its
 * "theme"). Cycled independently of the terrain by t/T; n/p does not call
 * it (a preset change leaves the colour alone). HUD/HINT pairs stay the
 * same so the UI remains legible.
 *
 * Safe to call any time; ncurses' init_pair updates the live pair
 * definition, so cells already on screen redraw with the new colours.
 *
 * On 8-colour terminals all palettes degrade to the same fallback set —
 * 256-colour palettes can't render with 8 colours, and remapping each
 * one to ANSI primaries would just produce indistinguishable variants.
 */
static void palette_apply(int idx)
{
    if (idx < 0 || idx >= N_PRESETS) idx = 0;
    if (COLORS >= 256) {
        const Preset *t = &presets[idx];
        init_pair(PAIR_WATER,    t->colors[0], -1);
        init_pair(PAIR_BEACH,    t->colors[1], -1);
        init_pair(PAIR_GRASS,    t->colors[2], -1);
        init_pair(PAIR_HILL,     t->colors[3], -1);
        init_pair(PAIR_MOUNTAIN, t->colors[4], -1);
        init_pair(PAIR_SNOW,     t->colors[5], -1);
    } else {
        init_pair(PAIR_WATER,    COLOR_BLUE,    -1);
        init_pair(PAIR_BEACH,    COLOR_YELLOW,  -1);
        init_pair(PAIR_GRASS,    COLOR_GREEN,   -1);
        init_pair(PAIR_HILL,     COLOR_GREEN,   -1);
        init_pair(PAIR_MOUNTAIN, COLOR_WHITE,   -1);
        init_pair(PAIR_SNOW,     COLOR_WHITE,   -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);   /* reserved bright yellow */
        init_pair(PAIR_HINT,        51, -1);   /* reserved bright cyan   */
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    palette_apply(0);   /* CLASSIC palette on startup */
}

/*
 * draw_cell — paint one terrain cell as a CELL_COLS-wide run of its glyph,
 * at screen row sy starting at column sx. Cells are widened because
 * terminal characters are ~twice as tall as wide; one column per cell would
 * squash the square map. Columns past the screen edge are skipped.
 */
static void draw_cell(int sy, int sx, int cols, Tile t)
{
    attron(COLOR_PAIR(t.pair) | t.attr);
    for (int c = 0; c < CELL_COLS; c++) {
        int xx = sx + c;
        if (xx >= 0 && xx < cols)
            mvaddch(sy, xx, (chtype)(unsigned char)t.glyph);
    }
    attroff(COLOR_PAIR(t.pair) | t.attr);
}

/*
 * grid_draw — paint the height field. Reads ONLY the HeightField (const)
 * plus two render parameters (the active `filter_band` and the preset
 * `bias`); it cannot see the Subdivision or Controls — the narrowest type
 * it needs. Reads top-to-bottom as: centre the map, find the visible height
 * window, then for each computed cell normalise → bias → filter → draw.
 */
static void grid_draw(const HeightField *g, int filter_band, float bias,
                      int cols, int rows)
{
    /* Centre the (g->w*CELL_COLS)-wide × g->h-tall map block in the rows
     * left between the top HUD lines and the bottom hint. */
    int map_rows = rows - (HUD_TOP_ROWS + HUD_BOTTOM_ROWS);
    int gx0 = (cols - g->w * CELL_COLS) / 2;
    int gy0 = (map_rows - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;

    HeightRange range = computed_height_range(g);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x * CELL_COLS;
            if (sx >= cols) break;             /* rest of row is off-screen */

            int idx = grid_idx(g, x, y);
            if (!g->computed[idx]) continue;   /* uncomputed — blank */

            float v = normalize01(g->height[idx], range.min, range.span);
            if (bias != 1.0f) v = powf(v, bias);   /* biome-mix gamma */

            /* filter mode: render only the selected biome */
            if (filter_band != FILTER_ALL && band_index_for_norm(v) != filter_band)
                continue;

            draw_cell(sy, sx, cols, band_for_height(v));
        }
    }
}

/*
 * hud_print — draw one HUD segment at (row, x) in (pair, attr), clipped
 * so it never passes max_x or wraps onto the next line. (ncurses wraps a
 * too-long string by default, which would corrupt the data row below.)
 * Returns the column just past the drawn text so segments chain
 * left-to-right; a no-op once x has reached max_x.
 */
static int hud_print(int row, int x, int max_x, int pair, int attr,
                     const char *str)
{
    if (x >= max_x) return x;
    int avail = max_x - x;
    int len   = (int)strlen(str);
    int n     = (len < avail) ? len : avail;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", n, str);
    attroff(COLOR_PAIR(pair) | attr);
    return x + n;
}

/*
 * hud_draw_status — HUD row 0 (bold, the dominant line): title, run state,
 * the two visual axes (theme = palette via t/T, preset = terrain via n/p),
 * and the frame rate.
 */
static void hud_draw_status(const Screen *sc, const Scene *s, double fps)
{
    const Controls *c = &s->controls;
    const char *state_str =
        c->paused                     ? "PAUSED"    :
        (s->state == SCENE_COMPUTING) ? "COMPUTING" :
                                        "HOLD";
    char seg[128];
    snprintf(seg, sizeof seg,
             " DIAMOND-SQUARE  %-9s  theme:%-8s  preset:%-8s  %.1f fps ",
             state_str, presets[c->current_theme].name,
             presets[c->current_preset].name, fps);
    hud_print(0, 0, sc->cols, PAIR_HUD, A_BOLD, seg);
}

/*
 * hud_draw_detail — HUD row 1 (not bold, so row 0 stays dominant):
 * recurrence internals that advance as the fractal fills in (level, step,
 * phase, roughness), the live sim settings (cells/tick, Hz), then the
 * active biome filter, colour-coded to its own tile colour.
 */
static void hud_draw_detail(const Screen *sc, const Scene *s)
{
    const Subdivision *sd = &s->subdiv;
    const Controls    *c  = &s->controls;
    const char *phase_str =
        c->paused                 ? "(paused)" :
        (s->state == SCENE_HOLD)  ? "(done)"   :
        (sd->phase == OP_DIAMOND) ? "DIAMOND"  :
                                    "SQUARE";
    int level, total_levels;
    subdivision_progress(&s->grid, sd, &level, &total_levels);

    char row1[128];
    snprintf(row1, sizeof row1,
             " level %d/%d  step:%-3d  %-8s  rough:%.3f  ops:%-3d  hz:%-3d  show:",
             level, total_levels, sd->level_step, phase_str,
             sd->level_rough, c->ops_per_tick, c->sim_fps);
    int rx = hud_print(1, 0, sc->cols, PAIR_HUD, A_NORMAL, row1);
    hud_print(1, rx, sc->cols, band_color_pair(c->filter_band), A_NORMAL,
              band_name(c->filter_band));
}

/* hud_draw_actions — bottom row: every interactive key, clipped to width. */
static void hud_draw_actions(const Screen *sc)
{
    hud_print(sc->rows - 1, 0, sc->cols, PAIR_HINT, A_BOLD,
              " w/b/g/h/m/s a:all  t/T:theme  n/p:preset  +/-:ops  [/]:rate  spc:pause  r:reset  q:quit ");
}

/*
 * screen_draw — one frame: clear, paint the map, then the three HUD rows.
 * The dashboard reads the whole Scene (const) because the HUD reports across
 * layers, but the map is painted via grid_draw with the narrow arguments.
 */
static void screen_draw(const Screen *sc, const Scene *s, double fps)
{
    const Controls *c = &s->controls;

    erase();
    grid_draw(&s->grid, c->filter_band, presets[c->current_preset].bias,
              sc->cols, sc->rows);
    hud_draw_status (sc, s, fps);
    hud_draw_detail (sc, s);
    hud_draw_actions(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* Screen lifecycle — these MUTATE Screen (terminal dimensions), so they
 * take a non-const pointer. Setup/teardown, not per-frame render. */
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

/* ===================================================================== */
/* §7  APP — input, signals, and the fixed-timestep loop. PERFORMANCE     */
/*     (accumulator, caps, frame sleep) and the input layer live here;    */
/*     simulation advances only via scene_tick.                          */
/* ===================================================================== */

/*
 * App — top-level container wiring the simulation to the terminal and the
 * run loop. Exactly ONE static instance (g_app) exists: POSIX signal
 * handlers take no user argument, so they must reach the run flags through
 * a global. Everything else is passed by pointer; the global is only here
 * for the handlers.
 *
 *   scene       : the whole simulation (WHAT / HOW / WHERE)
 *   screen      : cached terminal dimensions
 *   running     : main-loop flag; cleared by SIGINT/SIGTERM to exit cleanly
 *   need_resize : set by the SIGWINCH handler, then serviced and cleared at
 *                 the top of the loop
 *
 * WHY both flags are volatile sig_atomic_t: they are written inside a
 * signal handler and read in the main loop. sig_atomic_t is the only type
 * the C standard guarantees is read/written atomically w.r.t. a signal,
 * and `volatile` stops the compiler caching the value across the loop. The
 * handlers do nothing but set a flag — the real work (resize, teardown)
 * happens on the main thread where it is safe.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->scene.grid_N = pick_grid_N(app->screen.cols, app->screen.rows);
    scene_reset(&app->scene);
    app->need_resize = 0;
}

/*
 * app_handle_key — input layer. Translates a keypress into a change of
 * Controls (pause, filter, preset/theme, throttles) or a reset request.
 * It does NOT step the simulation — that stays in scene_tick.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene    *s = &app->scene;
    Controls *c = &s->controls;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     c->paused = !c->paused; break;
    case 'r': case 'R':
        scene_reset(s);
        break;

    /* Biome filter keys — set the filter to one band and reset the
     * heightmap so the user sees a fresh reveal showing only that
     * biome's cells. 'a' clears the filter back to "show all". */
    case 'w': case 'W': c->filter_band = FILTER_WATER;    scene_reset(s); break;
    case 'b': case 'B': c->filter_band = FILTER_BEACH;    scene_reset(s); break;
    case 'g': case 'G': c->filter_band = FILTER_GRASS;    scene_reset(s); break;
    case 'h': case 'H': c->filter_band = FILTER_HILLS;    scene_reset(s); break;
    case 'm': case 'M': c->filter_band = FILTER_MOUNTAIN; scene_reset(s); break;
    case 's': case 'S': c->filter_band = FILTER_SNOW;     scene_reset(s); break;
    case 'a': case 'A': c->filter_band = FILTER_ALL;      scene_reset(s); break;

    /* Theme cycling — 't' next, 'T' previous. Swaps ONLY the colour
     * palette of the current view; the terrain is untouched (no regen),
     * so you can recolour the land you're looking at. */
    case 't':
        c->current_theme = (c->current_theme + 1) % N_PRESETS;
        palette_apply(c->current_theme);
        break;
    case 'T':
        c->current_theme = (c->current_theme + N_PRESETS - 1) % N_PRESETS;
        palette_apply(c->current_theme);
        break;

    /* Preset cycling — 'n' next, 'p' previous. Changes ONLY the terrain
     * pattern (roughness/persistence/bias) and regenerates. The colour
     * is left untouched — recolour separately with t/T. */
    case 'n':
        c->current_preset = (c->current_preset + 1) % N_PRESETS;
        scene_reset(s);
        break;
    case 'p':
        c->current_preset = (c->current_preset + N_PRESETS - 1) % N_PRESETS;
        scene_reset(s);
        break;

    case '=': case '+':
        if (c->ops_per_tick < OPS_PER_TICK_MAX) c->ops_per_tick *= 2;
        if (c->ops_per_tick > OPS_PER_TICK_MAX) c->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        c->ops_per_tick /= 2;
        if (c->ops_per_tick < OPS_PER_TICK_MIN) c->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        c->sim_fps += SIM_FPS_STEP;
        if (c->sim_fps > SIM_FPS_MAX) c->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        c->sim_fps -= SIM_FPS_STEP;
        if (c->sim_fps < SIM_FPS_MIN) c->sim_fps = SIM_FPS_MIN;
        break;
    default: break;
    }
    return true;
}

/*
 * run_fixed_ticks — drain the fixed-timestep accumulator: add this frame's
 * dt, then run one scene_tick per whole tick_ns of accumulated time. Decouples
 * simulation rate from frame rate; scene_tick (§4) is the single combine
 * point, so this only decides HOW MANY ticks the frame owes.
 */
static void run_fixed_ticks(Scene *s, int64_t *accum, int64_t tick_ns, int64_t dt)
{
    *accum += dt;
    while (*accum >= tick_ns) {
        scene_tick(s);
        *accum -= tick_ns;
    }
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

    screen_init(&app->screen);
    scene_init(&app->scene, pick_grid_N(app->screen.cols, app->screen.rows));

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

        int64_t dt = frame_delta_ns(&frame_time);

        /* SIMULATION: run the ticks this frame owes. */
        run_fixed_ticks(&app->scene, &sim_accum,
                        TICK_NS(app->scene.controls.sim_fps), dt);

        /* measure the displayed fps over FPS_UPDATE_MS windows */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        pace_frame(frame_time, dt);      /* PERFORMANCE/DELAYS: hold ~RENDER_HZ */

        /* RENDER then INPUT. */
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
