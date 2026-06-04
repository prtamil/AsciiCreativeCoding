/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * truchet_tiles.c
 *   — Truchet tilings: take ONE simple tile (e.g., a square split
 *     diagonally), randomly rotate it at every cell of a grid, and
 *     watch a complex emergent pattern arise from a single primitive.
 *     Sébastien Truchet's 1704 observation, in ASCII.
 *
 * DEMO: A grid of '/' and '\' — randomly per cell — fills the
 *       screen. The diagonals join across cell boundaries to form
 *       swirls, mazes, and meandering ribbons that look anything
 *       but random; that emergent structure is the Truchet effect.
 *       The pattern is fully STATIC — it changes only when you reseed
 *       ('r') or switch pattern / glyph set / theme.
 *
 *       PATTERN and GLYPH SET are independent axes.
 *
 *       PATTERN controls HOW the orientation field is DISTRIBUTED
 *       across the grid — i.e., the "noise" the tiling is sampled
 *       from. Cycle with n / p:
 *
 *         RANDOM   uniform per-cell hash. Adjacent cells uncorrelated.
 *                  The classic Truchet "white-noise" look.
 *         NOISE    fBm-correlated. Smooth flowing regions of same
 *                  orientation, like clouds carved into Truchet.
 *         BANDS    sinusoidal field — diagonal stripes alternate
 *                  between orientations.
 *         VORONOI  jittered-seed regions. Irregular blocks share
 *                  one orientation; boundaries follow the Voronoi
 *                  diagram of the random seeds.
 *
 *       GLYPH SET controls WHICH characters represent each
 *       orientation, and how many orientations are available. Cycle
 *       with g / G — 12 sets across three structural families:
 *
 *         2-orient × 1-cell : diag(/\), lens(()), brkt([]), wave(~-)
 *         4-orient × 1-cell : axis(/\_|), cross(/\+X),
 *                             arrow(<>^v), dots(oO#@)
 *         2-orient × 2-cell : slope(,'), tri(<>),
 *                             wcurv(()), wbrkt([])
 *
 *       'r' reseeds — same algorithm, fresh arrangement of the
 *       chosen pattern.
 *
 * Study alongside:
 *   ../worldgen/cloud.c
 *      — uses the same Perlin/fBm noise this file's NOISE pattern
 *        samples, but as a flowing animated field rather than a
 *        frozen orientation map.
 *   ../fractal_random/automaton_2d.c
 *      — also "complexity from a simple per-cell rule applied
 *        uniformly". Truchet is the static analogue: no time
 *        evolution, just per-cell random orientation.
 *
 * Section map (cut by CONCERN, not by subsystem — see ARCHITECTURE below):
 *   §1 CONFIG       — constants, pattern/glyph/theme tables (data only)
 *   §2 PERFORMANCE  — monotonic timer + sleep (accumulator/frame-cap in §6 main)
 *   §3 LOGIC        — pure decisions: pattern name, spatial hash
 *   §4 SIMULATION   — noise field, the STATIC Truchet pattern, Scene, scene_tick
 *   §5 RENDER       — colour/themes, screen, orientation-shaded glyph draw + HUD
 *   §6 APP          — signals, resize, input, fixed-step combine (main)
 *
 * Keys:
 *   q / ESC    quit
 *   r          reseed (new tile arrangement)
 *   n / N      next pattern  (RANDOM → NOISE → BANDS → VORONOI)
 *   p / P      previous pattern
 *   g / G      next / previous glyph set
 *   t / T      next / previous theme
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/patterns/truchet_tiles.c \
 *     -o truchet -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : TRUCHET TILING — Sébastien Truchet noticed
 *                  (1704) that a single decorative tile can generate
 *                  effectively unlimited PATTERNS just by being
 *                  randomly rotated at every cell of a grid. Pick a
 *                  tile (a square divided diagonally black/white).
 *                  Pick a per-cell rotation from its symmetry group
 *                  (C4 for a 4-rotation tile, or just the 2-rotation
 *                  flip for the diagonal-split version). The resulting
 *                  tiling has structure at every scale — no two cells
 *                  decide the global pattern, but their collective
 *                  choices reveal swirls, mazes, labyrinths and
 *                  Voronoi-like blobs.
 *
 *                  For ASCII: a "tile" is one or more screen cells;
 *                  the rotation is encoded in WHICH glyph(s) we draw
 *                  there. Per-cell rotation is picked deterministically
 *                  from a hash of (cell_x, cell_y, seed) — same seed
 *                  always reproduces the same pattern; reseed = new
 *                  pattern. The PATTERN knob varies how that rotation
 *                  is DISTRIBUTED (white-noise, fBm, bands, Voronoi).
 *
 *                  The render is STATIC: a pure function of
 *                  (cell_x, cell_y, seed, pattern, glyph_set).  Nothing
 *                  animates; the image holds until a key event.
 *
 * Data-structure : NONE. Like the cloud and parallax-star showcases,
 *                  every render is a pure function of (cell_x,
 *                  cell_y, seed). No grid is stored.
 *
 * Rendering      : ASCII only. For each cell:
 *                    glyph, orient = truchet_glyph(sx, sy, seed, pattern, set)
 *                    tier  = spread orient across [TIER_LO, TIER_HI]
 *                    render at PAIR_RAMP_BASE + tier, A_BOLD
 *                  Cells are SHADED by tile orientation (static, pattern-
 *                  derived contrast) so /- and \-regions read light vs dark.
 *
 * Performance    : Per cell, a single hash (RANDOM/VORONOI), a sinf
 *                  (BANDS), or one 3-octave Perlin-fBm (NOISE) — well
 *                  under a microsecond. The image is static, so ncurses
 *                  re-emits almost nothing after the first frame. No
 *                  allocations.
 *
 * References     : The tiling first, then the noise fields and the drawing.
 *
 *   CONCEPTS — Truchet tilings
 *   • Smith, C.S. (1987) — "The Tiling Patterns of Sébastien Truchet
 *     and the Topology of Structural Hierarchy", Leonardo 20(4):373-385.
 *     The modern revival (with a translation of Truchet's memoir); the
 *     best single starting point.
 *   • Truchet, S. (1704) — "Mémoire sur les combinaisons", Mémoires de
 *     l'Académie Royale des Sciences. The original observation.
 *   • Browne, C. (2008) — "Truchet curves and surfaces", Computers &
 *     Graphics 32(2):268-281.  The curved (quarter-circle) variants.
 *   • Wikipedia — "Truchet tiles"
 *     https://en.wikipedia.org/wiki/Truchet_tiles  Quick figures of the
 *     diagonal, arc, and multi-tile forms.
 *
 *   PATTERN FIELDS — how the orientations are DISTRIBUTED (n/p)
 *   • Perlin, K. (1985) — "An Image Synthesizer", SIGGRAPH Computer
 *     Graphics 19(3):287-296.  Gradient noise — the fBm behind the
 *     NOISE pattern (and this file's perlin2d/fbm2).
 *   • Worley, S. (1996) — "A Cellular Texture Basis Function",
 *     SIGGRAPH '96:291-294.  Nearest-seed / Voronoi cellular noise —
 *     the basis of the VORONOI pattern.
 *
 *   RENDERING — building the field and inking it
 *   • Quilez, I. — "fBm" https://iquilezles.org/articles/fbm/  Practical
 *     octave-sum noise; the recipe fbm2 follows.
 *   • Bourke, P. — "Character representation of grey scale images",
 *     https://paulbourke.net/dataformats/asciiart/  The intensity→ASCII
 *     ramp idea behind the theme ramp tiers and the HUD swatch.
 *   • Bridges Math/Art — https://archive.bridgesmathart.org/  A large
 *     gallery of Truchet-based artwork and papers.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A complex pattern need not come from a complex rule. Take ONE tiny
 * design — half a square shaded one way, half the other. Cover a
 * grid with copies of it, rotating each one at random. The resulting
 * surface is anything but trivial: paths, swirls, mazes, knots all
 * appear out of pure local randomness. The cells don't communicate;
 * they each just flip a coin. The pattern is in the EYE — your
 * visual cortex stitches the per-cell diagonals into globally
 * meaningful curves. That's the Truchet effect.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a stack of identical playing cards, each one bearing a
 * single diagonal line from corner to corner. Spread them on a
 * checkerboard, randomly choosing which corner each card's line
 * starts from. Step back. Where two adjacent cards have lines that
 * share an edge endpoint, those lines APPEAR continuous to the eye.
 * Long curves emerge — sometimes closing into circles, sometimes
 * meandering across the whole grid. Variations:
 *   - Cards with TWO diagonals (an X) on some flipped orientation
 *     → cross-junctions in the pattern
 *   - Multi-cell cards (one card spans 2 squares)
 *     → patterns at multiple scales
 *
 * NO ANIMATION: once seeded, the surface is FIXED. Nothing drifts or
 * fades; the picture changes only on a key event — reseed, or switch
 * pattern / glyph set / theme. The art is the static tiling itself.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SEED. Pick a 32-bit hash seed.  Reseed reshuffles all tiles.
 *  2. EACH FRAME (identical until a key is pressed):
 *     For every screen cell (sx, sy):
 *        i.   v      = pattern_value(pattern, tile_x(sx), sy, seed) ∈ [0,1]
 *        ii.  orient = ⌊v · n_orient⌋        (n_orient from the glyph set)
 *        iii. glyph  = gs->glyphs[orient · tile_w + sub_x]
 *        iv.  tier   = TIER_HI − orient·(TIER_HI−TIER_LO)/(n_orient−1)
 *        v.   draw glyph at PAIR_RAMP_BASE + tier, A_BOLD
 *  3. HUD on top.
 *
 * KEY FORMULAS
 * ────────────
 *  Tile-cell coordinate (tile_w from the active glyph set):
 *     tile_x = sx / tile_w      // 2 for the 2-cell glyph sets
 *     sub    = sx mod tile_w
 *
 *  Orientation (the PATTERN sets how v is distributed across the grid):
 *     v      = pattern_value(pattern, tile_x, sy, seed)  ∈ [0, 1]
 *     orient = clamp(⌊v · n_orient⌋, 0, n_orient − 1)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • TILE_W != 1 INDEXING. WAVE has TILE_W = 2; the LEFT cell of a
 *    tile uses sub = 0, the RIGHT cell uses sub = 1. With negative
 *    sx, sx % 2 may give -1 in C; bias by adding TILE_W and taking
 *    modulo again. We render starting at sx = 0 only, so this is
 *    moot in this file; flag it if you ever scroll into negatives.
 *
 *  • HASH STABILITY. The orientation uses (tile_x, sy, seed). seed is
 *    fixed until 'r' reseeds, and the coords are integer screen cells
 *    with NO time term — so the pattern is fully static. (Feeding a
 *    moving offset into the coords WOULD scroll the pattern; that is a
 *    different look we deliberately don't do here.)
 *
 *  • NOISE PATTERN NEEDS Scene.perm. PATTERN_NOISE samples Perlin fBm for
 *    its orientation field, so perm_shuffle must run on every reseed /
 *    pattern change (apply_perm) or the field goes stale.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • STATIC. The image never changes on its own — no drift, no fade.
 *    Only a key event redraws it differently.
 *
 *  • Press 'r'. The tile arrangement REORGANISES — same algorithm, a
 *    new seed → new orientations.
 *
 *  • RANDOM pattern, diag glyphs. Trace a continuous diagonal across
 *    the screen — it connects across cells, bending where neighbours
 *    flip. Long ribbons and closed loops are the Truchet signature.
 *
 *  • NOISE pattern. Orientations form smooth flowing regions (fBm-
 *    correlated), not white-noise speckle.
 *
 *  • BANDS pattern. Diagonal stripes of one orientation alternate with
 *    the other. VORONOI: irregular blocks share one orientation,
 *    boundaries following the jittered-seed Voronoi diagram.
 *
 *  • Glyph sets (g / G). axis adds '_' '|' (maze channels); cross adds
 *    '+' 'X' intersections; the 2-cell sets span column pairs.
 *
 *  • Theme cycle (t / T). The whole pattern recolours; every theme's
 *    tier-6 colour should read clearly against default-black.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into six concern-layers.  The whole point this structure
 * EXPOSES: the Truchet pattern is a PURE, STATIC function — it never moves.
 * There are NO cosmetic effects; nothing animates between key presses.
 *
 *   LAYER         SECTION   MUTATES
 *   ───────────   ───────   ─────────────────────────────────────────────
 *   CONFIG        §1        nothing — const tables (patterns, glyphs, themes)
 *   PERFORMANCE   §2 + §6   main-local timing only (frame_time, sim_accum,
 *                           fps_accum, frame_count); never Scene
 *   LOGIC         §3        nothing — pure functions (args → value)
 *   SIMULATION    §4        Scene.perm (the Permutation, rebuilt by perm_shuffle
 *                           on reseed/pattern change) and Scene.time_secs (the
 *                           wall clock — the ONLY thing scene_tick advances)
 *   RENDER        §5        ncurses screen + colour pairs only; Screen.{cols,
 *                           rows} on init/resize.  NEVER mutates Scene
 *   APP/EVENTS    §6        Scene.{seed, current_*}, App.{sim_fps, running,
 *                           need_resize} via keys/signals
 *
 * WHICH CONCERNS THIS PROGRAM ACTUALLY HAS — it is a STATIC pattern renderer,
 * so only four of the six layers are real; the rest are named-and-dismissed
 * here rather than given hollow sections:
 *   • CONFIG, PERFORMANCE, LOGIC, RENDER — the genuine layers.
 *   • SIMULATION — trivial.  scene_tick advances only a wall clock (reseed
 *     entropy) and nothing it touches is drawn, so §4 is really the pattern
 *     (pure LOGIC) + the scene state, NOT a per-tick simulation.
 *   • EFFECTS — none.  No glow / trail / flash state exists.
 *   • DELAYS  — none.  No pause / hold / timer (the frame cap is PERFORMANCE).
 *   • pattern_value + truchet_glyph (§4) are pure LOGIC: glyph = f(perm, sx, sy,
 *     seed, pattern, glyph_set), with NO time term — frozen until a key event
 *     picks a new seed/pattern.  They take a const Permutation* and read no
 *     globals; they sit in §4 grouped with the Scene/pattern machinery.
 *
 * PER-TICK COMBINE ORDER (the ONE place state advances — main, §6):
 *   1. PERFORMANCE  measure real dt (clock_ns), clamp to 100 ms
 *   2. SIMULATION   drain the fixed-step accumulator → scene_tick(dt) ×K
 *                   (advances only the wall clock — the pattern is static)
 *   3. PERFORMANCE  fps accounting + sleep to the 60 fps frame cap
 *   4. RENDER       screen_draw (the static Truchet pattern + HUD)
 * User events — resize (SIGWINCH), keys (app_handle_key) — mutate state but
 * are NOT part of the tick; they run before/after it, never inside scene_tick.
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
/* §1  CONFIG — constants, pattern/glyph/theme tables (data only)         */
/* ===================================================================== */

enum {
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* Render-loop pacing (main).  Nothing animates, but the loop still runs
     * for input/resize; cap the redraw rate and clamp a stalled frame. */
    RENDER_FPS_CAP      =  60,    /* hard frame-rate cap (Hz) — per-frame sleep target  */
    MAX_FRAME_DT_MS     = 100,    /* clamp a stalled frame's dt to dodge the spiral of death */

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 theme tints           */

    /* Cells are shaded by their tile ORIENTATION (static, pattern-derived
     * contrast — no brightness field): the orientations spread across the
     * bright tiers [TIER_LO, TIER_HI] so /-regions and \-regions read as
     * light vs dark, making the pattern's structure visible. */
    TIER_HI             =   7,
    TIER_LO             =   4,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* fBm octaves — used by the NOISE pattern's orientation field. */
#define FBM_OCTAVES          3

/*
 * Pattern — the four orientation DISTRIBUTIONS (cycled by n/p).  A Truchet
 * tiling needs one orientation per tile; Pattern decides HOW those orientations
 * are spread across the grid.  Each value maps (tile_x, tile_y, seed) → a scalar
 * in [0,1] (see pattern_value); the active GlyphSet then quantises that scalar
 * into its orientation count.  So the two axes are INDEPENDENT: Pattern controls
 * the STRUCTURE (uncorrelated / smooth / banded / blocky), GlyphSet controls the
 * ALPHABET (which glyphs, how many orientations).
 *
 *   RANDOM  — white noise: one integer hash per tile, tiles uncorrelated; the
 *             classic Truchet look (Truchet 1704; Smith 1987).
 *   NOISE   — fractional-Brownian Perlin noise → smooth correlated regions
 *             (Perlin 1985).  The only distribution that reads Scene.perm.
 *   BANDS   — a 2-D sinusoid 0.5 + 0.5·sin(x·fx + y·fy) → diagonal stripes.
 *   VORONOI — nearest jittered seed in a coarse grid → irregular blocks of one
 *             orientation, boundaries on the Voronoi diagram (Worley 1996).
 * See the References block for citations.
 */
typedef enum {
    PATTERN_RANDOM  = 0,    /* white-noise hash — uncorrelated tiles       */
    PATTERN_NOISE   = 1,    /* Perlin fBm — smooth flowing regions         */
    PATTERN_BANDS   = 2,    /* sinusoid — diagonal stripes                 */
    PATTERN_VORONOI = 3,    /* jittered-seed cells — irregular blocks      */
    N_PATTERNS      = 4,    /* count — bounds the n/p cycle, not a pattern */
} Pattern;

/* Pattern-specific tuning constants. */
#define NOISE_SCALE_X        0.045f     /* fBm spatial scale          */
#define NOISE_SCALE_Y        0.090f
#define BANDS_FREQ_X         0.25f      /* sinusoid spatial frequency */
#define BANDS_FREQ_Y         0.45f
#define VORONOI_GRID_X       6          /* coarse-cell width  (cols)  */
#define VORONOI_GRID_Y       3          /* coarse-cell height (rows)  */

/*
 * GlyphSet — pairs a glyph ALPHABET with the tile metadata to render it.  Where
 * Pattern decides how orientations are distributed, GlyphSet decides WHICH glyph
 * each orientation draws and HOW MANY orientations exist.  The pattern's [0,1]
 * value is quantised into n_orient buckets to pick the rotation, and the chosen
 * row of glyphs inks the tile's cells (see truchet_glyph).
 *
 * 12 sets across three structural families:
 *   2-orient × 1-cell : diag, lens, brkt, wave
 *   4-orient × 1-cell : axis, cross, arrow, dots
 *   2-orient × 2-cell : slope, tri, wcurv, wbrkt
 *
 *   name     — HUD label, cycled by g/G.
 *   n_orient — orientation count (2 or 4).  Sets the quantisation AND the
 *              orientation→tier shade spread in scene_draw.
 *   tile_w   — tile width in screen cells (1, or 2 for multi-cell sets); for
 *              tile_w=2 the sub-cell x picks which of the row's two glyphs.
 *   glyphs   — n_orient × tile_w chars in ROW-MAJOR order by orientation,
 *              indexed glyphs[orient·tile_w + sub_x]; trailing slots (to 8)
 *              are unused 0s.
 */
typedef struct {
    const char *name;   /* HUD label (g/G)                                    */
    int  n_orient;      /* orientations: 2 or 4 — quantisation + shade spread */
    int  tile_w;        /* tile width in cells (1, or 2 for multi-cell sets)  */
    char glyphs[8];     /* orient×tile_w chars, indexed [orient·tile_w+sub_x] */
} GlyphSet;

static const GlyphSet GLYPH_SETS[] = {
    /*  name      orient tile glyphs                                       */
    {  "diag ",    2,    1,   { '/', '\\',                  0, 0, 0, 0, 0, 0 } },
    {  "lens ",    2,    1,   { '(', ')',                   0, 0, 0, 0, 0, 0 } },
    {  "brkt ",    2,    1,   { '[', ']',                   0, 0, 0, 0, 0, 0 } },
    {  "wave ",    2,    1,   { '~', '-',                   0, 0, 0, 0, 0, 0 } },
    {  "axis ",    4,    1,   { '/', '\\', '_', '|',        0, 0, 0, 0 } },
    {  "cross",    4,    1,   { '/', '\\', '+', 'X',        0, 0, 0, 0 } },
    {  "arrow",    4,    1,   { '<', '>',  '^', 'v',        0, 0, 0, 0 } },
    {  "dots ",    4,    1,   { 'o', 'O',  '#', '@',        0, 0, 0, 0 } },
    {  "slope",    2,    2,   { ',', '\'', '\'', ',',       0, 0, 0, 0 } },
    {  "tri  ",    2,    2,   { '<', '>',  '>',  '<',       0, 0, 0, 0 } },
    {  "wcurv",    2,    2,   { '(', ')',  ')',  '(',       0, 0, 0, 0 } },
    {  "wbrkt",    2,    2,   { '[', ']',  ']',  '[',       0, 0, 0, 0 } },
};
#define N_GLYPH_SETS ((int)(sizeof GLYPH_SETS / sizeof GLYPH_SETS[0]))

/*
 * Theme — one named colour palette, cycled by t/T.  WHY a struct (not loose
 * arrays): a theme is the whole look in one row of the themes[] table, so
 * adding/editing a palette touches exactly one line and theme_apply just blits
 * the row into ncurses colour pairs.  Every value is a 256-colour-cube index;
 * the CLAUDE.md "Theme Palette Brightness" rule keeps them all in the bright
 * half so even A_DIM cells stay legible on a default-black terminal.
 *
 *   name — HUD label, the only part the user reads.
 *   ramp — 8-tier dark→bright gradient, loaded into pairs PAIR_RAMP_BASE+0..7.
 *          Cells are shaded by tile orientation across tiers TIER_LO..TIER_HI
 *          (4..7), so those four bright tiers actually carry the picture; the
 *          full 8 still appear in the HUD swatch.
 */
typedef struct {
    const char *name;      /* HUD label, cycled by t/T                       */
    short       ramp[8];   /* 8-tier dark→bright gradient (256-cube indices)  */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7  */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 } },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 } },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 } },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 } },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 } },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 } },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 } },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 } },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 } },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 } },
};

/* ===================================================================== */
/* §2  PERFORMANCE — timing primitives                                    */
/* ===================================================================== */
/* Monotonic clock + sleep.  The fixed-timestep accumulator, fps counter   */
/* and 60 fps frame cap that USE them live at the combine point in §6 main. */

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
/* §3  LOGIC — pure decisions (no mutation, no I/O)                       */
/* ===================================================================== */
/* Results depend only on arguments; these write no globals and touch no    */
/* screen.  The Truchet pattern samplers (pattern_value, truchet_glyph) and  */
/* the Perlin stack are ALSO pure (they take a const Permutation*, read no    */
/* globals), but live in §4 with the Scene/pattern machinery (see ARCHITECTURE).*/

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RANDOM:  return "RANDOM ";
    case PATTERN_NOISE:   return "NOISE  ";
    case PATTERN_BANDS:   return "BANDS  ";
    case PATTERN_VORONOI: return "VORONOI";
    default:              return "?      ";
    }
}

/* hash3 — same routine as other showcases. */
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

/* hash_unit — map a 32-bit hash to a float in [0, 1): the top 24 bits over 2^24
 * (24 bits being the exact mantissa precision of a float). */
static inline float hash_unit(uint32_t h)
{
    return (float)(h & 0xFFFFFFu) / 16777216.0f;
}

/* wrap_inc / wrap_dec — step an index forward/backward through [0, n) with
 * wraparound: the "next/previous in a cyclic list" the key handler uses to cycle
 * pattern / glyph set / theme.  wrap_dec adds n before the modulo so the result
 * is never negative. */
static inline int wrap_inc(int v, int n) { return (v + 1) % n; }
static inline int wrap_dec(int v, int n) { return (v + n - 1) % n; }

/* ===================================================================== */
/* §4  SIMULATION — noise field, the STATIC Truchet pattern, Scene, tick   */
/* ===================================================================== */
/* The Truchet pattern is fully STATIC — nothing animates.  perm_shuffle      */
/* rebuilds Scene.perm on reseed/pattern change; scene_tick only advances the  */
/* wall clock (entropy for the next reseed).  pattern_value / truchet_glyph    */
/* are PURE and STATIC (no time term) — LOGIC by behaviour (they take a const   */
/* Permutation*), grouped here with the Scene/noise machinery they belong to.   */

/*
 * Permutation — the Perlin-noise permutation table: a random shuffle of the
 * bytes 0..255 used to hash integer lattice points into gradient indices
 * (Perlin 1985 / "Improving Noise" 2002).  It is stored DOUBLED (512 entries,
 * the second half repeating the first) so a lattice lookup can index
 * table[x & 255] + y up to 511 without a wrap test.  Rebuilt by perm_shuffle
 * whenever the seed/pattern changes; only the NOISE distribution reads it (via
 * perlin2d).  Perlin scaffold copied inline per the self-contained-file rule.
 */
typedef struct {
    uint8_t table[512];   /* shuffled 0..255, duplicated into [256,512) */
} Permutation;

static void perm_shuffle(Permutation *p, int seed)
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
        p->table[i      ] = base[i];
        p->table[i + 256] = base[i];
    }
}

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

static float perlin2d(const Permutation *pm, float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = pm->table[X    ] + Y;
    int B = pm->table[X + 1] + Y;
    float n00 = grad2(pm->table[A    ], x,        y       );
    float n10 = grad2(pm->table[B    ], x - 1.0f, y       );
    float n01 = grad2(pm->table[A + 1], x,        y - 1.0f);
    float n11 = grad2(pm->table[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

static float fbm2(const Permutation *pm, float x, float y)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(pm, x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;     /* → [0, 1] */
}

/* ----------------------------------------------------------------------- *
 * Truchet glyph picker.                                                   *
 * ----------------------------------------------------------------------- */

/*
 * nearest_seed_cell — the Voronoi core.  Each coarse cell of the VORONOI_GRID
 * holds one jittered seed point (its position hashed from the cell coords).  Of
 * the 3×3 block of coarse cells around tile (tx,ty), find the one whose seed is
 * closest — distance squared, with dy weighted ×2 because terminal cells are
 * ~2× taller than wide.  Returns that winning cell in (*best_cx,*best_cy).
 */
static void nearest_seed_cell(int tx, int ty, int seed,
                              int *best_cx, int *best_cy)
{
    int cx = tx / VORONOI_GRID_X;
    int cy = ty / VORONOI_GRID_Y;
    *best_cx = cx; *best_cy = cy;
    long best_d = (long)1 << 60;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int gcx = cx + dx, gcy = cy + dy;
            uint32_t h = hash3(gcx, gcy, seed);
            int sx = gcx * VORONOI_GRID_X + (int)(h        % (uint32_t)VORONOI_GRID_X);
            int sy = gcy * VORONOI_GRID_Y + (int)((h >> 8) % (uint32_t)VORONOI_GRID_Y);
            long ddx = (long)(sx - tx);
            long ddy = (long)(sy - ty) * 2;     /* aspect */
            long d   = ddx * ddx + ddy * ddy;
            if (d < best_d) { best_d = d; *best_cx = gcx; *best_cy = gcy; }
        }
    }
}

/*
 * pattern_value — map (tile_x, tile_y, seed) to a value in [0, 1].  Each pattern
 * is a different DISTRIBUTION of that value across the grid; the caller quantises
 * it into the active glyph set's orientation count to pick the rotation.
 */
static float pattern_value(const Permutation *pm, Pattern p,
                           int tx, int ty, int seed)
{
    switch (p) {
    case PATTERN_RANDOM:                          /* white noise: per-tile hash */
        return hash_unit(hash3(tx, ty, seed));

    case PATTERN_NOISE: {                          /* Perlin fBm: smooth regions */
        float ox = (float)((seed >> 16) & 0xFFFF) * 0.001f;   /* seed → field offset */
        float oy = (float)( seed        & 0xFFFF) * 0.001f;
        return fbm2(pm, (float)tx * NOISE_SCALE_X + ox,
                    (float)ty * NOISE_SCALE_Y + oy);
    }
    case PATTERN_BANDS: {                          /* sinusoid: diagonal stripes */
        float phase = (float)(seed & 0xFFFF) * (1.0f / 65536.0f) * 6.2832f;
        float a     = (float)tx * BANDS_FREQ_X + (float)ty * BANDS_FREQ_Y + phase;
        return 0.5f + 0.5f * sinf(a);
    }
    case PATTERN_VORONOI: {                        /* nearest jittered seed cell */
        int best_cx, best_cy;
        nearest_seed_cell(tx, ty, seed, &best_cx, &best_cy);
        return hash_unit(hash3(best_cx, best_cy, seed ^ 0x9E3779B9));
    }
    case N_PATTERNS: break;
    }
    return 0.5f;
}

/*
 * truchet_glyph — deterministic glyph for one screen cell.
 *
 * 1. Resolve the active glyph set: gs->n_orient is the orientation
 *    count, gs->tile_w is the tile width.
 * 2. Convert screen x to (tile_x, sub_x) using gs->tile_w.
 * 3. Sample the pattern field at (tile_x, sy) → v ∈ [0, 1].
 * 4. Quantise v → orient ∈ [0, n_orient).
 * 5. Look up gs->glyphs[orient · tile_w + sub_x].
 *
 * Pure function of (perm, sx, sy, seed, pattern, set_idx); no time dependence.
 * The chosen orientation is written to *out_orient (the renderer shades by it).
 * `pm` is only consulted by the NOISE distribution; the others ignore it.
 */
static char truchet_glyph(const Permutation *pm, int sx, int sy, int seed,
                          Pattern p, int set_idx, int *out_orient)
{
    if (set_idx < 0 || set_idx >= N_GLYPH_SETS) set_idx = 0;
    const GlyphSet *gs = &GLYPH_SETS[set_idx];
    int tile_w = gs->tile_w;
    int tile_x = (sx >= 0) ? (sx / tile_w) : -((-sx + tile_w - 1) / tile_w);
    int sub_x  = sx - tile_x * tile_w;

    float v = pattern_value(pm, p, tile_x, sy, seed);
    int orient = (int)(v * (float)gs->n_orient);
    if (orient < 0)              orient = 0;
    if (orient >= gs->n_orient)  orient = gs->n_orient - 1;

    *out_orient = orient;
    return gs->glyphs[orient * tile_w + sub_x];
}

/*
 * Scene — the whole drawn frame in one struct, laid out as a table of contents.
 * Render/HUD read it; only the orchestrators (init / reseed / tick) take a
 * Scene* — every other function takes the narrowest sub-type (const
 * Permutation*, const Screen*, …).
 */
typedef struct {
    /* WHAT defines the pattern — the noise table + the seed that built it. */
    Permutation perm;            /* Perlin table for the NOISE distribution */
    int      seed;               /* drives every tile's orientation         */

    /* HOW the user picks the pattern (the genuinely tunable knobs). */
    Pattern  current_pattern;    /* which orientation DISTRIBUTION (n/p)    */
    int      current_glyph_set;  /* which glyphs + orientation count (g/G)  */

    /* WHEN — wall clock, only entropy for the next reseed. */
    float    time_secs;

    /* Display option — a RENDER concept, merely toggled by a key. */
    int      current_theme;      /* active Theme index (t/T)                */
} Scene;

/*
 * apply_perm — rebuild the Perlin permutation for the current (seed, pattern)
 * pair, so the NOISE pattern's orientation field looks different per pattern.
 * The pattern index is XOR-mixed into the seed so the same (seed, pattern)
 * always shuffles the same way (deterministic), but cycling patterns gives a
 * fresh field each time.  Mutates s->perm.
 */
static void apply_perm(Scene *s)
{
    perm_shuffle(&s->perm, s->seed ^ ((int)s->current_pattern * 0xA5A5A5));
}

static void scene_reseed(Scene *s)
{
    /* Mix the wall clock with the previous seed so each 'r' press differs. */
    s->seed = (int)hash3((int)(s->time_secs * 1000.0f), s->seed, 0xC0FFEE);
    apply_perm(s);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->current_theme     = 0;
    s->current_pattern   = PATTERN_NOISE;   /* flowing regions read clearly; */
    s->current_glyph_set = 0;               /* RANDOM is white-noise, dull at boot */
    s->seed              = 0xC0FFEE;
    apply_perm(s);
}

/*
 * scene_tick — the Truchet pattern is fully STATIC, so a tick advances only
 * the wall clock, which is read by scene_reseed for fresh entropy on 'r'.
 * Nothing visible moves between frames.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
}

/* ===================================================================== */
/* §5  RENDER — state → screen (reads only, never mutates the model)      */
/* ===================================================================== */
/* Colour/theme setup, screen geometry, then the per-cell draw (the static     */
/* Truchet glyph, shaded by orientation) and HUD.  Reads Scene; writes only     */
/* ncurses (screen + colour pairs) and Screen.{cols,rows}.  Never mutates Scene.*/

/* ---- colour: load a theme's ramp + accents into ncurses pairs ----------- */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
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

/* ---- screen: terminal viewport ------------------------------------------ */

/*
 * Screen — the terminal viewport, just its size in character cells.  WHY a type
 * for two ints: it is the narrowest read-only handle the render functions
 * (scene_draw, screen_draw) need for geometry, so they take `const Screen*`
 * instead of the whole App — keeping RENDER decoupled from the rest.  Captured
 * by getmaxyx at init and refreshed on every SIGWINCH resize; scene_draw scans
 * rows 2..rows-2 (row 0/1 = HUD, last row = key hint).
 */
typedef struct {
    int cols, rows;   /* current terminal width / height in cells */
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

/* orient_to_tier — spread orientation 0..n_orient-1 across the bright shade
 * tiers [TIER_LO, TIER_HI] (orientation 0 = brightest), so the /- and \-leaning
 * regions read as light vs dark.  This is the static, pattern-derived contrast
 * that replaced the old brightness field. */
static inline int orient_to_tier(int orient, int n_orient)
{
    int span = (n_orient > 1) ? n_orient - 1 : 1;
    return TIER_HI - (orient * (TIER_HI - TIER_LO)) / span;
}

/*
 * scene_draw — render the static Truchet pattern.  Every cell is the pure
 * Truchet glyph (a hash of sx, sy, seed), shaded by its tile orientation so the
 * structure is visible — all static, no brightness field, no overlays.  The
 * image is unchanged frame to frame until a key reseeds or switches knobs.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int top = 2, bottom = sc->rows - 1;
    int n_orient = GLYPH_SETS[s->current_glyph_set].n_orient;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            int  orient;
            char glyph = truchet_glyph(&s->perm, sx, sy, s->seed,
                                       s->current_pattern,
                                       s->current_glyph_set, &orient);

            int pair = PAIR_RAMP_BASE + orient_to_tier(orient, n_orient);
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/*
 * hud_draw_status_line — row 0: title on the left, fps / tick-Hz on the right.
 * (No run-state or speed: the pattern is static, so there is nothing to pause.)
 */
static void hud_draw_status_line(const Screen *sc, double fps, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz ", fps, sim_fps);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);            /* right-aligned status */
    mvprintw(0, 1, " TRUCHET TILES ");     /* left title          */
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/*
 * hud_draw_param_line — row 1: the knobs the keys control — pattern, glyph set,
 * theme, a live swatch of the 8 ramp colours, and the current seed.  Each field
 * prints, then `x` advances past its fixed column width.
 */
static void hud_draw_param_line(const Screen *sc, const Scene *s)
{
    (void)sc;
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", GLYPH_SETS[s->current_glyph_set].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    /* Ramp swatch — paint each of the 8 gradient tiers in its own pair. */
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
    mvprintw(1, x, "  seed:%08x ", (unsigned)s->seed);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* hud_draw_key_hints — bottom row: the full interactive key legend. */
static void hud_draw_key_hints(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);                      /* the static pattern */
    hud_draw_status_line(sc, fps, sim_fps); /* row 0: title + fps/Hz */
    hud_draw_param_line(sc, s);             /* row 1: pattern/glyph/theme/ramp/seed */
    hud_draw_key_hints(sc);                 /* bottom row: key legend */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §6  APP — signals, resize, input + the per-tick combine                */
/* ===================================================================== */
/* main is the ONE place the layers combine per tick (PERFORMANCE →         */
/* SIMULATION → PERFORMANCE → RENDER; see ARCHITECTURE).  Signals and        */
/* app_handle_key mutate state OUTSIDE the tick, never inside scene_tick.    */

/*
 * App — the running program: the Scene plus the screen and the loop runtime the
 * tick and the signal handlers share.  It is the root main owns; sub-layers
 * still take the narrowest type, so bundling everything here does not re-couple
 * them.
 *   scene / screen — WHAT is drawn + WHERE (see those types).
 *   sim_fps        — fixed-timestep tick rate (Hz), [ / ] keys; sets TICK_NS.
 *   running        — 0 = quit; cleared by SIGINT/SIGTERM.
 *   need_resize    — 1 = a SIGWINCH is pending; serviced before the next tick.
 * running/need_resize are written from async signal handlers, so they are
 * `volatile sig_atomic_t` — the only type a handler may portably touch and the
 * only way the compiler won't hoist the flag-read out of the loop.
 */
typedef struct {
    Scene                 scene;        /* WHAT is drawn — the simulated frame */
    Screen                screen;       /* WHERE — viewport size               */
    int                   sim_fps;      /* fixed tick rate (Hz), [ / ] keys    */
    volatile sig_atomic_t running;      /* 0 = quit; set by SIGINT/SIGTERM     */
    volatile sig_atomic_t need_resize;  /* 1 = SIGWINCH pending; serviced in loop */
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

/* app_handle_key — user EVENT, not part of the tick.  May mutate Scene knobs
 * and App fields directly; it never advances simulation state (no scene_tick). */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case 'r': case 'R': scene_reseed(s);                               break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
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

    case 'n': case 'N':
        s->current_pattern = (Pattern)wrap_inc((int)s->current_pattern, N_PATTERNS);
        apply_perm(s);                 /* fresh noise field for the new pattern */
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)wrap_dec((int)s->current_pattern, N_PATTERNS);
        apply_perm(s);
        break;

    case 'g':
        s->current_glyph_set = wrap_inc(s->current_glyph_set, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph_set = wrap_dec(s->current_glyph_set, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/* app_init — bring the program up: seed the RNG, install signal handlers and
 * the endwin() atexit hook, set the loop's starting state, then open the screen
 * and the scene.  Everything that happens once, before the per-frame loop. */
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

    const int64_t max_dt_ns    = (int64_t)MAX_FRAME_DT_MS * NS_PER_MS;
    const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

    while (app->running) {

        /* EVENT (not the tick): service a pending resize before timing. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 1. PERFORMANCE — measure real elapsed time, clamp a stall. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > max_dt_ns) dt = max_dt_ns;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* 2. SIMULATION — drain the fixed-step accumulator.  The pattern is
         *    static; scene_tick only advances the wall clock for reseed. */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* 3. PERFORMANCE — fps accounting + sleep to the frame cap. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(frame_cap_ns - elapsed);

        /* 4. RENDER — read-only draw of the static Truchet pattern. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* EVENT (not the tick): drain one key. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
