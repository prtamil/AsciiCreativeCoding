/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wang_tiles.c
 *   — Wang tilings: a finite set of square tiles with coloured edges
 *     are placed on a grid such that adjacent tiles share matching
 *     edge colours.  Hao Wang's 1961 construction; the constraint
 *     graph forces continuous "veins" of colour to flow across
 *     tile boundaries.  16-tile complete set with 2 edge colours.
 *
 * DEMO: A grid of multi-cell tiles fills the screen.  Each tile is
 *       a small rectangle whose four edges (north/east/south/west)
 *       carry one of two colours.  Where two tiles meet, the
 *       SHARED edge has the same colour on both sides — that's the
 *       Wang constraint.  The eye reads adjacent same-colour edges
 *       as continuous BANDS flowing across the screen, even though
 *       each tile chose its colour independently.
 *
 *       PATTERN (n / N) — what BIASES the per-cell tile choice
 *       beyond the Wang constraint:
 *
 *         RANDOM    uniform over all valid tiles → maximally varied
 *         NOISE     fBm bias → large blobby same-colour regions
 *         STRIPES   sinusoidal y-bias → horizontal coloured bands
 *         SWIRL     polar bias around screen centre → quartered
 *                   colour map
 *
 *       GLYPH (g / G) — how each tile is rendered:
 *
 *         EDGES     thin '-' '_' '|' borders only
 *         BLOCKS    bold '#' borders + softly-tinted interior
 *         WIRES     a circuit mesh (default) — every matching edge
 *                   routes a coloured spoke to a central + junction
 *
 *       'r' reseeds the tile placement; pattern and glyph cycling
 *       reshuffle the underlying noise too so the screen feels
 *       genuinely different per pattern.  The tiling is fully STATIC —
 *       it changes only on a key event, never on its own.
 *
 * Study alongside:
 *   ../patterns/truchet_tiles.c
 *      — same idea (per-cell rotation of a tile family) but with no
 *        edge constraint.  Wang tiles are Truchet's with the
 *        adjacency rule turned on.
 *   ../generational/wfc_showcase.c
 *      — wave-function collapse generalises Wang-tile constraint
 *        propagation to arbitrary tile sets and supports backtracking;
 *        this file is the simpler "pick first valid candidate"
 *        version.
 *
 * Section map (cut by CONCERN, not by subsystem — see ARCHITECTURE below):
 *   §1 CONFIG       — constants, edge-ramp, pattern/glyph/theme tables (data)
 *   §2 PERFORMANCE  — monotonic timer + sleep (accumulator/frame-cap in §6 main)
 *   §3 LOGIC        — pure decisions: pattern/glyph names, spatial hash
 *   §4 SIMULATION   — noise + tile set + constraint solver (the STATIC grid)
 *   §5 RENDER       — colour/themes, screen, per-cell tile draw + HUD
 *   §6 APP          — signals, resize, input, fixed-step combine (main)
 *
 * Keys:
 *   q / ESC    quit
 *   r          reseed tile arrangement
 *   n / N      next pattern  (RANDOM → NOISE → STRIPES → SWIRL)
 *   p / P      previous pattern
 *   g / G      next / previous glyph set (EDGES → BLOCKS → WIRES)
 *   t / T      next / previous theme
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/patterns/wang_tiles.c \
 *     -o wang -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three stages.
 *
 *                  (1) TILE SET. With 2 edge colours per axis there
 *                      are 2⁴ = 16 distinct tiles — every combination
 *                      of (N, E, S, W) ∈ {0, 1}⁴.  This is a
 *                      "complete" set: for any pair of (W-constraint,
 *                      N-constraint) at least one tile is valid, so
 *                      placement never gets stuck.
 *
 *                  (2) PLACEMENT (constraint solver).  Iterate row-
 *                      major over the grid.  For each cell:
 *                          expected_w = (left  cell exists) ? left.E  : ANY
 *                          expected_n = (above cell exists) ? above.S : ANY
 *                          valid     = filter(tile_set, by W & N)
 *                          chosen    = pick(valid, x, y, seed, pattern)
 *                          grid[x,y] = chosen
 *                      The PATTERN influences which of several valid
 *                      tiles to pick — RANDOM is uniform; NOISE biases
 *                      toward whichever colour an fBm field prefers
 *                      at this point; STRIPES biases by sin(y);
 *                      SWIRL biases by atan2 around the centre.
 *                      The Wang constraint itself is identical
 *                      across patterns; only the bias differs.
 *
 *                  (3) RENDERING.  Each tile is TILE_W × TILE_H
 *                      cells (default 6×3 → roughly square on
 *                      terminals where cells are 2× taller than
 *                      wide).  The active GLYPH set determines how
 *                      to draw each cell within the tile:
 *                        EDGES  : '-' top, '_' bottom, '|' sides,
 *                                 blank interior — the minimum
 *                                 visualisation of the constraint.
 *                        BLOCKS : '#' on edges (BOLD), '.' tinted
 *                                 interior — heavier visual weight.
 *                        WIRES  : a wire mesh (the default) — each
 *                                 edge whose colour is shared routes a
 *                                 coloured spoke to a central '+'
 *                                 junction, so same-colour edges
 *                                 connect as straights, bends, T's and
 *                                 crosses across tile boundaries.
 *
 *                  The render is STATIC: once placed, the tiling is a
 *                  pure function of (grid cell, seed, pattern, glyph).
 *                  Nothing animates; the image holds until a key event.
 *
 * Data-structure : 16-entry tile_set[] plus a flat WangGrid of
 *                  uint8_t tile-indices, one per tile cell. For a
 *                  240×80 screen with 6×3 tiles, the grid is at
 *                  most 40×26 = 1 040 bytes. No allocation.
 *
 * Rendering      : ASCII only. Each cell's glyph + colour come from its
 *                  tile's edge colours — the two edge colours map to two
 *                  ramp slots (mid + bright), drawn at a fixed A_BOLD so
 *                  the matching-edge "veins" read clearly.
 *
 * Performance    : Generation is O(grid_w · grid_h · N_TILES) — about
 *                  1 040 × 16 = 17 000 ops, microseconds. Run only on
 *                  reseed / pattern change.  The image is static, so
 *                  ncurses re-emits almost nothing after the first frame.
 *
 * References     : The tiles and tiling theory first, then graphics use.
 *
 *   CONCEPTS — Wang tiles & aperiodic tiling
 *   • Wang, H. (1961) — "Proving Theorems by Pattern Recognition II",
 *     Bell System Technical Journal 40:1-41.  The original Wang tile
 *     paper; best starting point for the edge-matching idea.
 *   • Berger, R. (1966) — "The Undecidability of the Domino Problem",
 *     Memoirs of the AMS 66.  Disproved Wang's periodicity conjecture
 *     with the first APERIODIC set (20 426 tiles).
 *   • Jeandel, E. & Rao, M. (2021) — "An aperiodic set of 11 Wang
 *     tiles", Advances in Combinatorics 2021:1.  The MINIMAL aperiodic
 *     set (11 tiles, 4 colours), proven minimal by computer search.
 *   • Grünbaum, B. & Shephard, G.C. (1987) — "Tilings and Patterns",
 *     W.H. Freeman.  Definitive treatment of Wang tiles & the domino
 *     problem.
 *   • Wikipedia — "Wang tile"
 *     https://en.wikipedia.org/wiki/Wang_tile  Quick figures of the
 *     edge-coloured tiles and aperiodic sets.
 *
 *   RENDERING — Wang tiles in graphics
 *   • Cohen, M.F., Shade, J., Hiller, S. & Deussen, O. (2003) — "Wang
 *     Tiles for Image and Texture Generation", SIGGRAPH '03:287-294.
 *     The modern CG revival; the "pick a valid neighbour" placement
 *     this file uses.
 *   • Stam, J. (1997) — "Aperiodic Texture Mapping", ERCIM R046.  An
 *     early use of Wang tiles to tile a plane without visible repeats.
 *   • Lagae, A. & Dutré, P. (2006) — "An Alternative for Wang Tiles:
 *     Colored Edges versus Colored Corners", ACM TOG 25(4):1442-1459.
 *     Directly about the edge-coloured scheme this file renders.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a small library of square tiles. Each tile has FOUR edges,
 * each painted in one of a few colours. Rule: when you place tile
 * A next to tile B, the edge they SHARE must match in colour. Now
 * fill a grid by repeatedly picking any tile that satisfies the rule
 * at this position. The rule is local — only the immediate north
 * and west neighbours influence the choice — but the consequence is
 * GLOBAL: same-colour edges line up across many tile boundaries to
 * form continuous bands of colour flowing through the picture. The
 * pattern looks designed; nobody designed it.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a child's wooden block set. Each block is a cube with
 * coloured stripes painted on its four side faces. You stack the
 * blocks on a table such that any TWO TOUCHING SIDE FACES wear the
 * same paint colour. Doing this strictly forces stripes that span
 * many blocks — a red stripe carrying through five blocks,
 * separated from a blue stripe by a clean dividing line. Step back:
 * what looks like one elaborate continuous painting is actually 25
 * independently-chosen blocks held together by a single matching
 * rule.
 *
 * The PATTERN (n / N) is what makes the random tile choice prefer
 * red OR blue at each spot — but the matching rule is the same.
 * Pattern bias produces large red regions vs random patches vs
 * banded stripes; the underlying machinery is identical.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Build the 16-tile complete set. tile_set[i] has edges
 *     (N, E, S, W) = (i bits 3..0).
 *  2. GENERATE the grid (whenever seed or pattern changes):
 *     for ty in 0..H:
 *       for tx in 0..W:
 *         if (tx > 0): expected_w = grid[ty][tx-1].E
 *         if (ty > 0): expected_n = grid[ty-1][tx].S
 *         valid = { tile : tile.W matches and tile.N matches }
 *         scored = score each valid tile by pattern's preference
 *                  (RANDOM: 0 for all; NOISE: prefer if E,S match
 *                  fBm-biased colour; STRIPES: prefer if S matches
 *                  sin(y)-biased colour; SWIRL: prefer if E,S match
 *                  atan2(y - cy, x - cx)-biased colour)
 *         chosen = highest-scored, with hash-based tiebreak
 *         grid[ty][tx] = chosen
 *  3. RENDER (the same image every frame until a key is pressed):
 *     for sy in screen_rows:
 *       for sx in screen_cols:
 *         tx = (sx − gx0) / TILE_W
 *         ty = (sy − gy0) / TILE_H
 *         dx = sx mod TILE_W, dy = sy mod TILE_H
 *         tile = grid[ty][tx]
 *         (glyph, pair) = glyph_set_render(tile, dx, dy)
 *         mvaddch(sy, sx, glyph) in pair, A_BOLD
 *
 * KEY FORMULAS
 * ────────────
 *  Tile index encoding:
 *    tile_set[i].n = (i >> 3) & 1
 *    tile_set[i].e = (i >> 2) & 1
 *    tile_set[i].s = (i >> 1) & 1
 *    tile_set[i].w = (i >> 0) & 1
 *
 *  Wang constraint at cell (tx, ty):
 *    valid = { tile : (tx == 0 OR tile.W = grid[ty][tx-1].E) AND
 *                     (ty == 0 OR tile.N = grid[ty-1][tx].S) }
 *
 *  Pattern preference for edges to bias:
 *    RANDOM   : no preference
 *    NOISE    : prefer_e = prefer_s = (fbm(tx·s, ty·s) > 0.5)
 *    STRIPES  : prefer_s = (sin(ty·k) > 0)
 *    SWIRL    : θ        = atan2((ty − cy)·2, tx − cx)
 *               prefer_e = prefer_s = (θ > 0)
 *
 *  Score and pick:
 *    score(tile)     = (tile.E matches prefer_e ? 1 : 0)
 *                    + (tile.S matches prefer_s ? 1 : 0)
 *    chosen = uniform random over { valid : score = max_score }
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • COMPLETE SET. With 2 colours per axis and 16 tiles, EVERY
 *    (W, N) constraint pair has exactly 4 valid tiles (free
 *    choice over E and S). Placement never gets stuck. If you cut
 *    the set down to fewer than 16 tiles — to make the visual
 *    more constrained — you must verify that for ALL 4 possible
 *    (W, N) pairs, at least one tile remains valid.
 *
 *  • EDGE OF GRID. The leftmost column has no W neighbour; the
 *    top row has no N neighbour. Treat the missing constraint as
 *    "any" (tile can have either edge value). The corner tile
 *    (0, 0) is unconstrained and picks freely from all 16.
 *
 *  • PREFERENCE ≠ HARD CONSTRAINT. The pattern's preference biases
 *    the choice but never overrides the Wang constraint. NOISE
 *    might want a tile with E = 1, but if no valid tile has E = 1
 *    given the W/N constraint, the highest-scoring valid tile (E = 0)
 *    is taken instead. This keeps placement always succeeding.
 *
 *  • GRID SIZE VS SCREEN SIZE. The grid is rounded down to whole
 *    tiles: grid_w = screen_w / TILE_W, grid_h = (screen_h − HUD)
 *    / TILE_H. Cells at the screen edge that fall outside the
 *    last tile boundary are not drawn (left blank).
 *
 *  • TILE_W / TILE_H ASPECT. With cells 2× taller than wide, a 6×3
 *    tile is 6 cells horizontally and 6 "cell-widths" vertically
 *    (3 cells × 2) — visually square. Smaller tiles look
 *    horizontally squashed.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • EDGES pattern. Trace any vertical line where two tiles meet.
 *    The right border ('|') of one tile and the left border ('|')
 *    of the next tile are adjacent CELLS on screen — and by Wang
 *    constraint they're the same colour. You should never see a
 *    sharp colour change at a tile boundary; only at the
 *    ENCAPSULATED edges between tiles where the colour MATCHES.
 *
 *  • RANDOM pattern. Tile diversity is maximum; no large regions
 *    of same colour. The whole grid is uniformly speckled.
 *
 *  • NOISE pattern. Same constraint, different bias: now you should
 *    see large continuous regions of "mostly blue" and "mostly
 *    yellow" (or whatever the theme calls them) separated by
 *    irregular boundaries. Like fBm clouds.
 *
 *  • STRIPES pattern. The S edge bias creates horizontal BANDS:
 *    rows alternate between mostly-colour-0 and mostly-colour-1.
 *    The bands move slightly when you reseed because the sin
 *    phase is seeded.
 *
 *  • SWIRL pattern. Bias angles around the centre into 4 quadrants;
 *    you should see two halves of the grid favour different colours,
 *    splitting roughly along a horizontal-ish line through the
 *    middle.
 *
 *  • Switch GLYPH (g) within a fixed pattern. The TILE LAYOUT does
 *    not change; only the rendering (glyphs and interior fills) does.
 *    Tile boundaries are at the same positions; constraint matching
 *    is still visible (just with different glyphs).
 *
 *  • STATIC. The image never changes on its own — no drift, no fade.
 *    Only a key event (reseed, or switch pattern/glyph/theme) redraws
 *    it differently.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into six concern-layers.  The whole tiling is STATIC: it is
 * placed once (per seed/pattern) and then only re-drawn — nothing animates, so
 * SIMULATION is thin and EFFECTS/DELAYS are absent.
 *
 *   LAYER         SECTION   MUTATES
 *   ───────────   ───────   ─────────────────────────────────────────────
 *   CONFIG        §1        nothing — const tables (edge-ramp, patterns,
 *                           glyphs, themes) + the fixed 16-tile set's shape
 *   PERFORMANCE   §2 + §6   main-local timing only (frame_time, sim_accum,
 *                           fps_accum, frame_count); never Scene
 *   LOGIC         §3        nothing — pure functions (args → value)
 *   SIMULATION    §4        Scene.perm (the Permutation, via perm_shuffle) and
 *                           tile_set[] (tile_set_init), both on init/reseed/
 *                           pattern change; the WangGrid cells (grid_generate);
 *                           Scene.{seed, current_*, grid_*_cap, time_secs}
 *   RENDER        §5        ncurses screen + colour pairs only; Screen.{cols,
 *                           rows, gx0, gy0, grid_w, grid_h}.  NEVER mutates Scene
 *   APP/EVENTS    §6        Scene.{seed, current_*} + grid via keys; App.{sim_fps,
 *                           running, need_resize} via keys/signals
 *
 * WHICH CONCERNS ACTUALLY EXIST — it is a static generator + renderer:
 *   • SIMULATION is event-driven, not tick-driven.  grid_generate (the Wang
 *     constraint solver) and perm_shuffle / tile_set_init run on INIT, reseed,
 *     pattern change and resize — NOT every frame.  scene_tick advances only a
 *     wall clock (entropy for the next reseed); nothing it touches is drawn.
 *   • LOGIC (§3) is pure: pattern_name, glyph_set_name, hash3 depend only on
 *     their arguments.  grid_pick and tile_cell_render are ALSO pure decisions
 *     but live with the machinery they serve (grid_pick in §4 reads the
 *     Permutation; tile_cell_render in §5 is the per-cell draw decision).
 *   • EFFECTS — none.  No glow / trail / flash state exists.
 *   • DELAYS  — none.  No pause / hold / timer (the frame cap is PERFORMANCE).
 *
 * PER-TICK COMBINE ORDER (the ONE place state advances — main, §6):
 *   1. PERFORMANCE  measure real dt (clock_ns), clamp to 100 ms
 *   2. SIMULATION   drain the fixed-step accumulator → scene_tick(dt) ×K
 *                   (advances only the wall clock — the grid is static)
 *   3. PERFORMANCE  fps accounting + sleep to the 60 fps frame cap
 *   4. RENDER       screen_draw (the static tile grid + HUD)
 * User events — resize (SIGWINCH, which RE-GENERATES the grid), keys
 * (app_handle_key) — mutate state but are NOT part of the tick; they run
 * before/after it, never inside scene_tick.
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
/* §1  CONFIG — constants, edge-ramp, pattern/glyph/theme tables (data)   */
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

    /* Tile dimensions in screen cells. 6 × 3 → ~square on terminals
     * with 2:1 cell aspect. */
    TILE_W              =   6,
    TILE_H              =   3,

    /* 16-tile complete set: 2 edge colours per axis. */
    N_EDGE_COLORS       =   2,
    N_TILES             =  16,

    /* Hard upper bounds on the tile grid. With TILE_W=6 and TILE_H=3
     * across a 240×80 screen, max grid is 40 × 26. */
    MAX_SCREEN_W        = 256,
    MAX_SCREEN_H        =  96,
    MAX_GRID_W          = MAX_SCREEN_W / TILE_W,
    MAX_GRID_H          = MAX_SCREEN_H / TILE_H,
    MAX_GRID_CELLS      = MAX_GRID_W * MAX_GRID_H,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 theme tints            */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Cell aspect: terminal cells are ~2× taller than wide.  Used by the SWIRL
 * pattern so its polar bias reads round, not squashed. */
#define ASPECT_Y_F           2.0f

/* fBm octaves — used by the NOISE pattern's tile bias. */
#define FBM_OCTAVES          3

/* EDGE_ANY — sentinel edge constraint meaning "no neighbour, any colour goes"
 * (the grid's first row/column).  Passed to the constraint filter in place of
 * a real edge colour. */
#define EDGE_ANY (-1)

/* Edge-color → ramp-slot mapping. With 2 edge colours and a 0..7
 * ramp, pick two well-separated slots so the colours read distinctly:
 *   colour 0 → ramp[3] (mid-bright)
 *   colour 1 → ramp[6] (highlight)
 */
static const int EDGE_RAMP[N_EDGE_COLORS] = { 3, 6 };

/*
 * Pattern — the four BIASES applied to the tile choice (cycled by n/p).  The
 * Wang constraint already restricts each cell to the tiles whose W/N edges
 * match its left/above neighbours (see grid_generate); Pattern only breaks the
 * tie AMONG those legal candidates by preferring a colour for the E and/or S
 * edge.  So the constraint (the structure) is identical across patterns — only
 * the large-scale colour DISTRIBUTION differs.
 *
 *   RANDOM  — no bias: uniform over the legal candidates.  Maximally varied.
 *   NOISE   — prefer the colour an fBm field favours here → large blobby
 *             same-colour regions (Perlin 1985; reads Scene.perm).
 *   STRIPES — prefer the S colour a sin(y) field favours → horizontal bands.
 *   SWIRL   — prefer the colour an atan2(y,x) angle favours around the centre
 *             → the grid splits into colour quadrants.
 * Scoring is in grid_pick.  Ref: Cohen et al. 2003 (the placement model).
 */
typedef enum {
    PATTERN_RANDOM  = 0,    /* uniform over legal tiles                    */
    PATTERN_NOISE   = 1,    /* Perlin-fBm colour bias → blobs              */
    PATTERN_STRIPES = 2,    /* sinusoidal y-bias → horizontal bands        */
    PATTERN_SWIRL   = 3,    /* polar-angle bias → colour quadrants         */
    N_PATTERNS      = 4,    /* count — bounds the n/p cycle, not a pattern */
} Pattern;

/*
 * GlyphSet — how each tile cell is DRAWN (cycled by g/G).  Pure RENDER choice:
 * it changes nothing in the tile placement, only the glyphs.  All three colour
 * each cell by the tile's EDGE colours (EDGE_RAMP); they differ in what shape
 * they draw (decoded by tile_cell_render, §5).
 */
typedef enum {
    GLYPH_EDGES  = 0,    /* thin '-' '_' '|' borders, blank interior */
    GLYPH_BLOCKS = 1,    /* bold '#' borders + parity-tinted '.' fill */
    GLYPH_WIRES  = 2,    /* circuit mesh: spokes → '+' junctions (default) */
    N_GLYPH_SETS = 3,    /* count — bounds the g/G cycle             */
} GlyphSet;

/* Pattern bias parameters. */
#define NOISE_SCALE_X        0.06f
#define NOISE_SCALE_Y        0.12f
#define STRIPES_FREQ_Y       0.45f

/*
 * Theme — one named colour palette, cycled by t/T.  WHY a struct: a theme is
 * the whole look in one row of the themes[] table, so adding/editing a palette
 * touches one line and theme_apply just blits the row into ncurses pairs.
 * Every value is a 256-colour-cube index; the CLAUDE.md "Theme Palette
 * Brightness" rule keeps them in the bright half so cells stay legible on a
 * default-black terminal.
 *
 *   name — HUD label, the only part the user reads.
 *   ramp — 8-tier dark→bright gradient, loaded into pairs PAIR_RAMP_BASE+0..7.
 *          The two edge colours map to ramp[3] and ramp[6] (EDGE_RAMP), so
 *          those two tiers actually carry the tiling; the full 8 appear in the
 *          HUD swatch.
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
/* screen.  grid_pick and tile_cell_render are also pure decisions but live  */
/* with the machinery they serve (§4 / §5 — see ARCHITECTURE).               */

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RANDOM:  return "RANDOM ";
    case PATTERN_NOISE:   return "NOISE  ";
    case PATTERN_STRIPES: return "STRIPES";
    case PATTERN_SWIRL:   return "SWIRL  ";
    default:              return "?      ";
    }
}

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_EDGES:  return "edge ";
    case GLYPH_BLOCKS: return "block";
    case GLYPH_WIRES:  return "wire ";
    default:           return "?    ";
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

/* wrap_inc / wrap_dec — step an index forward/backward through [0, n) with
 * wraparound: the "next/previous in a cyclic list" the key handler uses to
 * cycle pattern / glyph set / theme.  wrap_dec adds n before the modulo so the
 * result is never negative. */
static inline int wrap_inc(int v, int n) { return (v + 1) % n; }
static inline int wrap_dec(int v, int n) { return (v + n - 1) % n; }

/* ===================================================================== */
/* §4  SIMULATION — noise + tile set + constraint solver (the STATIC grid) */
/* ===================================================================== */
/* The only state.  perm_shuffle / tile_set_init build the noise table and   */
/* the 16-tile set; grid_generate (the Wang constraint solver) fills the grid */
/* — ALL on init / reseed / pattern change / resize, never per tick.  The      */
/* per-tick scene_tick advances only the wall clock (reseed entropy).          */

/*
 * Permutation — the Perlin-noise permutation table: a random shuffle of the
 * bytes 0..255 used to hash integer lattice points into gradient indices
 * (Perlin 1985 / "Improving Noise" 2002).  Stored DOUBLED (512 entries, the
 * second half repeating the first) so a lattice lookup can index
 * table[x & 255] + y up to 511 without a wrap test.  Rebuilt by perm_shuffle on
 * every seed/pattern change; only the NOISE pattern reads it (via perlin2d).
 * Perlin scaffold copied inline per the self-contained-file rule.
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
 * Tile set + grid.                                                         *
 * ----------------------------------------------------------------------- */

/*
 * WangTile — one Wang tile: a unit square with a COLOURED EDGE on each side
 * (north / east / south / west), each colour in {0, 1} (Hao Wang, 1961).  The
 * tiling rule is that abutting tiles must share the same colour on their
 * touching edge, so a tile is fully described by its four edge colours.  The
 * 16-tile "complete" set (tile_set) is exactly every (n,e,s,w) ∈ {0,1}^4:
 * tile_set[i] encodes i's four low bits (n = bit 3, e = bit 2, s = bit 1,
 * w = bit 0), which is why ANY (W,N) constraint always has valid tiles — the
 * placement can never get stuck.  Ref: Wang 1961; Cohen et al. 2003.
 */
typedef struct {
    uint8_t n, e, s, w;   /* north/east/south/west edge colour, each 0 or 1 */
} WangTile;

static WangTile tile_set[N_TILES];

static void tile_set_init(void)
{
    for (int i = 0; i < N_TILES; i++) {
        tile_set[i].n = (uint8_t)((i >> 3) & 1);
        tile_set[i].e = (uint8_t)((i >> 2) & 1);
        tile_set[i].s = (uint8_t)((i >> 1) & 1);
        tile_set[i].w = (uint8_t)((i >> 0) & 1);
    }
}

/*
 * WangGrid — the placed tiling: a w×h grid of tile INDICES into tile_set,
 * stored as a flat ROW-MAJOR array (cell (tx,ty) = cells[ty*w + tx]).  Filled
 * by the constraint solver grid_generate so every shared edge matches its
 * neighbour; read by scene_draw to look up each tile's edge colours.  Pre-sized
 * to MAX_GRID_CELLS and held inline so generation never allocates.
 *   w, h  — grid dimensions in TILES (not screen cells)
 *   cells — tile indices in [0, N_TILES); the live region is [0, w*h)
 */
typedef struct {
    int     w, h;                    /* grid dimensions in TILES (not cells)  */
    uint8_t cells[MAX_GRID_CELLS];   /* row-major tile indices: cells[ty*w+tx] */
} WangGrid;

/*
 * pick_preferred — among the valid candidates, choose one whose E/S edges best
 * match the requested preferred colours (prefer = -1 means "no preference for
 * that edge", giving every candidate the same score).  Ties are broken
 * uniformly by a hash, so equally-good tiles stay varied.
 */
static int pick_preferred(int prefer_e, int prefer_s,
                          const uint8_t *valid, int n_valid,
                          int tx, int ty, int seed)
{
    uint8_t best[N_TILES];
    int best_score = -1;
    int n_best     =  0;
    for (int i = 0; i < n_valid; i++) {
        const WangTile *t = &tile_set[valid[i]];
        int score = 0;
        if (prefer_e >= 0 && (int)t->e == prefer_e) score++;
        if (prefer_s >= 0 && (int)t->s == prefer_s) score++;
        if (score > best_score) {
            best_score = score;
            best[0]    = valid[i];
            n_best     = 1;
        } else if (score == best_score) {
            best[n_best++] = valid[i];
        }
    }
    return best[hash3(tx, ty, seed ^ 0x9E3779B9) % (uint32_t)n_best];
}

/*
 * grid_pick — pick one of the Wang-legal candidates (`valid`) per the active
 * pattern.  RANDOM picks uniformly; the others compute a PREFERRED edge colour
 * from a field (fBm / sin / polar angle) and then take the best-matching tile.
 * The Wang constraint (already applied to `valid`) is never violated.
 */
static int grid_pick(const Permutation *pm, int tx, int ty, int seed, Pattern p,
                     const uint8_t *valid, int n_valid,
                     int grid_w, int grid_h)
{
    if (n_valid <= 0) return 0;

    int prefer_e = -1, prefer_s = -1;   /* -1 = no preference for that edge */

    switch (p) {
    case PATTERN_RANDOM:
        return valid[hash3(tx, ty, seed) % (uint32_t)n_valid];
    case PATTERN_NOISE: {                /* fBm field → blobby colour regions */
        float ox = (float)((seed >> 16) & 0xFFFF) * 0.001f;
        float oy = (float)( seed        & 0xFFFF) * 0.001f;
        float v  = fbm2(pm, (float)tx * NOISE_SCALE_X + ox,
                        (float)ty * NOISE_SCALE_Y + oy);
        int prefer = (v > 0.5f) ? 1 : 0;
        prefer_e = prefer; prefer_s = prefer;
        break;
    }
    case PATTERN_STRIPES: {              /* sin(y) field → horizontal bands */
        float phase = (float)(seed & 0xFFFF) * (1.0f / 65536.0f) * 6.2832f;
        float v     = 0.5f + 0.5f * sinf((float)ty * STRIPES_FREQ_Y + phase);
        prefer_s = (v > 0.5f) ? 1 : 0;
        break;
    }
    case PATTERN_SWIRL: {                /* polar angle → colour quadrants */
        float cx = (float)grid_w * 0.5f;
        float cy = (float)grid_h * 0.5f;
        float dx = (float)tx - cx;
        float dy = ((float)ty - cy) * ASPECT_Y_F;   /* aspect-correct y */
        float a  = atan2f(dy, dx);
        int prefer = (a > 0) ? 1 : 0;
        prefer_e = prefer; prefer_s = prefer;
        break;
    }
    case N_PATTERNS: break;
    }

    return pick_preferred(prefer_e, prefer_s, valid, n_valid, tx, ty, seed);
}

/*
 * grid_generate — fill the grid cell-by-cell, respecting the Wang
 * constraint between adjacent tiles. Iterate row-major; left + above
 * neighbour determine the W and N edge constraints respectively.
 * The first column drops the W constraint; the first row drops N.
 *
 * Time complexity: O(grid_w · grid_h · N_TILES) where N_TILES = 16 —
 * trivial. Run on reseed and on pattern change.
 */
/* collect_valid_tiles — fill `valid` with the indices of every tile in the
 * complete set whose W and N edges satisfy the given constraints (EDGE_ANY =
 * unconstrained): the Wang-legal candidates for a cell.  Returns the count. */
static int collect_valid_tiles(int expected_w, int expected_n, uint8_t *valid)
{
    int n_valid = 0;
    for (int i = 0; i < N_TILES; i++) {
        if (expected_w != EDGE_ANY && (int)tile_set[i].w != expected_w) continue;
        if (expected_n != EDGE_ANY && (int)tile_set[i].n != expected_n) continue;
        valid[n_valid++] = (uint8_t)i;
    }
    return n_valid;
}

static void grid_generate(WangGrid *g, const Permutation *pm, int seed,
                          Pattern p, int grid_w, int grid_h)
{
    if (grid_w > MAX_GRID_W) grid_w = MAX_GRID_W;
    if (grid_h > MAX_GRID_H) grid_h = MAX_GRID_H;
    g->w = grid_w;
    g->h = grid_h;

    /* Row-major sweep: each cell inherits its W edge from the left neighbour
     * and its N edge from the one above (the first row/column are free). */
    for (int ty = 0; ty < grid_h; ty++) {
        for (int tx = 0; tx < grid_w; tx++) {
            int expected_w = (tx > 0) ? tile_set[g->cells[ty * grid_w + (tx - 1)]].e
                                      : EDGE_ANY;
            int expected_n = (ty > 0) ? tile_set[g->cells[(ty - 1) * grid_w + tx]].s
                                      : EDGE_ANY;

            uint8_t valid[N_TILES];
            int n_valid = collect_valid_tiles(expected_w, expected_n, valid);

            g->cells[ty * grid_w + tx] = (uint8_t)grid_pick(pm, tx, ty, seed, p,
                                            valid, n_valid, grid_w, grid_h);
        }
    }
}

/*
 * Scene — the whole drawn frame in one struct, laid out as a table of contents.
 * Render/HUD read it; only the orchestrators (init / regenerate / reseed /
 * resize_to / tick) take a Scene* — every other function takes the narrowest
 * sub-type (const Permutation*, const WangGrid*, const Screen*, …).
 */
typedef struct {
    /* WHAT is generated — the tile grid + the noise/seed that built it. */
    WangGrid    grid;
    Permutation perm;            /* Perlin table for the NOISE bias       */
    int         seed;            /* drives the whole placement            */

    /* HOW the user drives the GENERATION (regenerates on change). */
    Pattern     current_pattern; /* which placement BIAS (n/p)            */
    int         grid_w_cap;      /* grid bounds (tiles) from the screen,  */
    int         grid_h_cap;      /*   the size grid_generate last built   */

    /* WHEN — wall clock, only entropy for the next reseed. */
    float       time_secs;

    /* Display options — RENDER concepts, merely toggled by keys. */
    GlyphSet    current_glyph;   /* how each tile cell is drawn (g/G)     */
    int         current_theme;   /* active Theme index (t/T)              */
} Scene;

/*
 * apply_perm — reshuffle the Permutation for the current (seed, pattern) pair
 * so the NOISE pattern's fBm bias looks different per pattern / reseed.
 * Mutates s->perm.
 */
static void apply_perm(Scene *s)
{
    perm_shuffle(&s->perm, s->seed ^ ((int)s->current_pattern * 0xA5A5A5));
}

static void scene_regenerate(Scene *s)
{
    apply_perm(s);
    grid_generate(&s->grid, &s->perm, s->seed, s->current_pattern,
                  s->grid_w_cap, s->grid_h_cap);
}

static void scene_reseed(Scene *s)
{
    /* Mix the wall clock with the previous seed so each 'r' press differs. */
    s->seed = (int)hash3((int)(s->time_secs * 1000.0f), s->seed, 0xABCDEF);
    scene_regenerate(s);
}

static void scene_init(Scene *s, int grid_w, int grid_h)
{
    memset(s, 0, sizeof *s);
    s->current_theme   = 0;
    s->current_pattern = PATTERN_RANDOM;
    s->current_glyph   = GLYPH_WIRES;   /* wire mesh reads best by default */
    s->seed            = 0xDEADBEEF;
    s->grid_w_cap      = grid_w;
    s->grid_h_cap      = grid_h;
    tile_set_init();
    scene_regenerate(s);
}

static void scene_resize_to(Scene *s, int grid_w, int grid_h)
{
    s->grid_w_cap = grid_w;
    s->grid_h_cap = grid_h;
    scene_regenerate(s);
}

/*
 * scene_tick — the tile placement is fully STATIC, so a tick advances only the
 * wall clock, which scene_reseed reads for fresh entropy on 'r'.  Nothing
 * visible moves between frames.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
}

/* ===================================================================== */
/* §5  RENDER — state → screen (reads only, never mutates the model)      */
/* ===================================================================== */
/* Colour/theme setup, screen geometry, the per-cell tile-draw DECISION      */
/* (tile_cell_render — pure, but it IS the render policy so it lives here),   */
/* then the grid draw + HUD.  Reads Scene; writes only ncurses (screen +      */
/* colour pairs) and Screen.  Never mutates Scene.                            */

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

/* ---- per-cell tile-draw decision ---------------------------------------- */

/*
 * cell_glyph_wires — the WIRES glyph set for one cell.  A wire mesh: an edge is
 * "wired" when its colour appears on >=2 of the tile's edges (it has a partner
 * to route to through the centre).  Each wired edge draws a coloured spoke (|
 * vertical, - horizontal) to the centre, where spokes meet at a junction (|
 * straight-vertical, - straight-horizontal, + a bend/T/cross).  This connects
 * ANY same-colour edges, so the tiling reads as a circuit of coloured wires.
 * (With 2 colours over 4 edges every tile is a 3- or 4-way junction.)
 */
static int cell_glyph_wires(const WangTile *t, int dx, int dy,
                            char *out_glyph, bool *visible)
{
    *visible = true;
    int  ramp_idx = 0;
    char glyph    = ' ';

    int cx = TILE_W / 2, cy = TILE_H / 2;
    int cnt[N_EDGE_COLORS] = { 0 };
    cnt[t->n]++; cnt[t->e]++; cnt[t->s]++; cnt[t->w]++;
    bool route_n = cnt[t->n] >= 2;
    bool route_e = cnt[t->e] >= 2;
    bool route_s = cnt[t->s] >= 2;
    bool route_w = cnt[t->w] >= 2;

    if (dx == cx && dy == cy) {                  /* centre junction */
        if      (!route_n && !route_e && !route_s && !route_w) *visible = false;
        else if (route_n && route_s && !route_e && !route_w)   glyph = '|';
        else if (route_e && route_w && !route_n && !route_s)   glyph = '-';
        else                                                   glyph = '+';
        ramp_idx = EDGE_RAMP[(cnt[1] > cnt[0]) ? 1 : 0];   /* majority colour */
    } else if (dx == cx && dy < cy) {            /* north spoke */
        if (route_n) { glyph = '|'; ramp_idx = EDGE_RAMP[t->n]; }
        else         *visible = false;
    } else if (dx == cx && dy > cy) {            /* south spoke */
        if (route_s) { glyph = '|'; ramp_idx = EDGE_RAMP[t->s]; }
        else         *visible = false;
    } else if (dy == cy && dx < cx) {            /* west spoke */
        if (route_w) { glyph = '-'; ramp_idx = EDGE_RAMP[t->w]; }
        else         *visible = false;
    } else if (dy == cy && dx > cx) {            /* east spoke */
        if (route_e) { glyph = '-'; ramp_idx = EDGE_RAMP[t->e]; }
        else         *visible = false;
    } else {
        *visible = false;
    }

    *out_glyph = glyph;
    return ramp_idx;
}

/*
 * tile_cell_render — return (glyph, ramp_idx, *visible) for one cell within a
 * tile, given the active glyph set and the cell's local (dx, dy) coordinates
 * inside the tile [0..TILE_W) × [0..TILE_H).  PURE (a function of its args
 * only); it is the per-cell RENDER policy, so it lives in §5.  EDGES/BLOCKS
 * colour the four border cells; WIRES routes the circuit mesh (see above).
 *
 * For invisible cells (e.g., interior of EDGES set), `visible` is set to false;
 * the caller skips drawing.
 */
static int tile_cell_render(const WangTile *t, int dx, int dy,
                            GlyphSet g, char *out_glyph, bool *visible)
{
    *visible = true;
    int ramp_idx = 0;
    char glyph = ' ';

    bool is_top    = (dy == 0);
    bool is_bot    = (dy == TILE_H - 1);
    bool is_left   = (dx == 0);
    bool is_right  = (dx == TILE_W - 1);

    if (g == GLYPH_EDGES) {
        if (is_top) {
            glyph = '-'; ramp_idx = EDGE_RAMP[t->n];
        } else if (is_bot) {
            glyph = '_'; ramp_idx = EDGE_RAMP[t->s];
        } else if (is_left) {
            glyph = '|'; ramp_idx = EDGE_RAMP[t->w];
        } else if (is_right) {
            glyph = '|'; ramp_idx = EDGE_RAMP[t->e];
        } else {
            *visible = false;
        }
    }
    else if (g == GLYPH_BLOCKS) {
        if (is_top || is_bot || is_left || is_right) {
            glyph = '#';
            ramp_idx = is_top  ? EDGE_RAMP[t->n]
                     : is_bot  ? EDGE_RAMP[t->s]
                     : is_left ? EDGE_RAMP[t->w]
                     :           EDGE_RAMP[t->e];
        } else {
            /* Interior — softly tinted by the parity of the four
             * edge colours so each tile has a recognisable shade. */
            int parity = (t->n ^ t->e ^ t->s ^ t->w) & 1;
            glyph = '.';
            ramp_idx = parity ? 5 : 4;
        }
    }
    else if (g == GLYPH_WIRES) {
        return cell_glyph_wires(t, dx, dy, out_glyph, visible);
    }

    *out_glyph = glyph;
    return ramp_idx;
}

/* ---- screen: terminal viewport + tile-grid geometry --------------------- */

/*
 * Screen — the terminal viewport AND where the tile grid sits inside it.  WHY
 * a type: the render functions take a const Screen* for all geometry, keeping
 * RENDER decoupled from App.  screen_layout derives the centred placement once
 * per resize; scene_draw then maps tile (tx,ty) → screen cell (gx0 + tx·TILE_W,
 * gy0 + ty·TILE_H).
 *   cols, rows     — terminal size in character cells
 *   gx0, gy0       — screen coords of the grid's top-left corner (centred)
 *   grid_w, grid_h — grid size in TILES (cols/TILE_W, rows/TILE_H, then capped)
 */
typedef struct {
    int cols, rows;       /* terminal size in cells                     */
    int gx0, gy0;         /* top-left of the tile grid in screen coords */
    int grid_w, grid_h;   /* grid dimensions in TILES                   */
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;
    int avail_w = s->cols;

    int gw = avail_w / TILE_W;
    int gh = avail_h / TILE_H;
    if (gw > MAX_GRID_W) gw = MAX_GRID_W;
    if (gh > MAX_GRID_H) gh = MAX_GRID_H;
    if (gw < 4) gw = 4;
    if (gh < 4) gh = 4;

    s->grid_w = gw;
    s->grid_h = gh;
    /* Centre the grid on screen so partial tiles don't leak at edges. */
    s->gx0 = (avail_w - gw * TILE_W) / 2;
    s->gy0 = top + (avail_h - gh * TILE_H) / 2;
    if (s->gx0 < 0)   s->gx0 = 0;
    if (s->gy0 < top) s->gy0 = top;
}

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
    screen_layout(s);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_layout(s);
}

/*
 * draw_one_tile — ink the TILE_W×TILE_H cells of tile `t` at grid position
 * (tx,ty), each through tile_cell_render in the active glyph set, at a fixed
 * bright A_BOLD.  Cells off-screen or under the HUD rows are skipped.
 */
static void draw_one_tile(const Screen *sc, const WangTile *t,
                          int tx, int ty, GlyphSet g)
{
    for (int dy = 0; dy < TILE_H; dy++) {
        int sy = sc->gy0 + ty * TILE_H + dy;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int dx = 0; dx < TILE_W; dx++) {
            int sx = sc->gx0 + tx * TILE_W + dx;
            if (sx < 0 || sx >= sc->cols) continue;

            char glyph;
            bool visible;
            int ramp_idx = tile_cell_render(t, dx, dy, g, &glyph, &visible);
            if (!visible) continue;

            int pair = PAIR_RAMP_BASE + ramp_idx;
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/*
 * scene_draw — render the static Wang tiling: draw every tile in the grid.
 * Each cell's glyph and colour come from its tile's edge colours, so the
 * matching-edge "veins" of colour ARE the structure — no overlays, no
 * animation.  The image is unchanged until a key reseeds or switches a knob.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    const WangGrid *g = &s->grid;
    for (int ty = 0; ty < g->h; ty++)
        for (int tx = 0; tx < g->w; tx++)
            draw_one_tile(sc, &tile_set[g->cells[ty * g->w + tx]],
                          tx, ty, s->current_glyph);
}

/*
 * hud_draw_status_line — row 0: title on the left, fps / tick-Hz on the right.
 * (No run-state: the tiling is static, so there is nothing to pause.)
 */
static void hud_draw_status_line(const Screen *sc, double fps, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz ", fps, sim_fps);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);            /* right-aligned status */
    mvprintw(0, 1, " WANG TILES ");        /* left title          */
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/*
 * hud_draw_param_line — row 1: the knobs the keys control — pattern, glyph set,
 * theme, a live swatch of the 8 ramp colours, and the grid size.  Each field
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
    mvprintw(1, x, " glyph:%-5s ", glyph_set_name(s->current_glyph));
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
    mvprintw(1, x, "  grid:%dx%d  tiles:%d ",
             s->grid.w, s->grid.h, s->grid.w * s->grid.h);
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
    scene_draw(sc, s);                      /* the static tiling */
    hud_draw_status_line(sc, fps, sim_fps); /* row 0: title + fps/Hz */
    hud_draw_param_line(sc, s);             /* row 1: pattern/glyph/theme/ramp/grid */
    hud_draw_key_hints(sc);                 /* bottom row: key legend */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §6  APP — signals, resize, input + the per-tick combine                */
/* ===================================================================== */
/* main is the ONE place the layers combine per tick (PERFORMANCE →         */
/* SIMULATION → PERFORMANCE → RENDER; see ARCHITECTURE).  Signals and        */
/* app_handle_key mutate state OUTSIDE the tick, never inside scene_tick;    */
/* a resize/reseed/pattern change RE-GENERATES the grid (also outside it).   */

/*
 * App — the running program: the Scene plus the screen and the loop runtime the
 * tick and the signal handlers share.  It is the root main owns; sub-layers
 * still take the narrowest type, so bundling everything here does not re-couple
 * them.
 *   scene / screen — WHAT is generated/drawn + WHERE (see those types).
 *   sim_fps        — fixed-timestep tick rate (Hz), [ / ] keys; sets TICK_NS.
 *   running        — 0 = quit; cleared by SIGINT/SIGTERM.
 *   need_resize    — 1 = a SIGWINCH is pending; serviced (re-generates the grid)
 *                    before the next tick.
 * running/need_resize are written from async signal handlers, so they are
 * `volatile sig_atomic_t` — the only type a handler may portably touch.
 */
typedef struct {
    Scene                 scene;        /* WHAT is generated/drawn             */
    Screen                screen;       /* WHERE — viewport + grid geometry    */
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
    scene_resize_to(&app->scene, app->screen.grid_w, app->screen.grid_h);
    app->need_resize = 0;
}

/* app_handle_key — user EVENT, not part of the tick.  May mutate Scene knobs
 * and re-generate the grid directly (reseed / pattern change); it never
 * advances simulation state inside the tick (no scene_tick). */
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
        scene_regenerate(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)wrap_dec((int)s->current_pattern, N_PATTERNS);
        scene_regenerate(s);
        break;

    /* Glyph cycling — pure rendering change, no regen. */
    case 'g':
        s->current_glyph = (GlyphSet)wrap_inc((int)s->current_glyph, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)wrap_dec((int)s->current_glyph, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/* app_init — bring the program up: seed the RNG, install signal handlers and
 * the endwin() atexit hook, set the loop's starting state, then open the screen
 * and build the initial tiling.  Everything that happens once, before the loop. */
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
    scene_init(&app->scene, app->screen.grid_w, app->screen.grid_h);
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

        /* EVENT (not the tick): service a pending resize (regenerates grid). */
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

        /* 2. SIMULATION — drain the fixed-step accumulator.  The grid is
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

        /* 4. RENDER — read-only draw of the static tile grid + HUD. */
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
