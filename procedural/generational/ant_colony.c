/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ant_colony.c — Ant Colony Optimisation: stigmergic shortest-path
 *                pheromone trails between a nest and food sources.
 *
 * DEMO: A colony of ~80 ants wanders a terminal grid.  Searching ants
 *       move with a slight bias toward the nearest food source and
 *       deposit a faint pheromone trail.  When an ant reaches food,
 *       it switches to RETURNING state and bee-lines back to the
 *       nest, depositing a HEAVY pheromone trail.  Future ants
 *       follow strong trails preferentially → positive feedback
 *       (autocatalysis) → the colony converges on the shortest
 *       routes between nest and food without any global knowledge.
 *
 *       Pheromone evaporates each tick, so disused paths fade and
 *       the colony adapts when food is moved (press `r`).  Classical
 *       Deneubourg double-bridge experiment, run live in your terminal.
 *
 *       Patterns (cycle with n / N) — different food-source layouts:
 *         DOUBLE     2 sources, opposite sides (canonical Deneubourg)
 *         SINGLE     1 source — simplest trail pattern
 *         QUAD       4 cardinal sources around nest
 *         LINE       3 collinear sources on one edge
 *         HEXAGON    6 sources at hex-arrangement angles
 *         CROSS      4 sources, cardinal directions, mid-distance
 *         TRIANGLE   3 sources at equilateral-ish triangle vertices
 *         CIRCLE     8 sources on a ring around the nest
 *         DIAGONAL   4 sources along the main diagonal
 *         CLUSTER    5 sources tightly clumped on one side
 *         DISTANT    2 sources at extreme opposite corners
 *         PERIMETER  6 sources spread along screen edges
 *         GRID       9 sources in a 3×3 grid
 *         RANDOM     7 sources at fixed-seed random positions
 *
 *       Themes (cycle with t / T):
 *         CLASSIC   blue trails, yellow searchers, magenta returners
 *         INFRARED  warm red/orange "heat" trails
 *         FOREST    green trails, brown ants
 *         NEON      vivid magenta/cyan pop
 *         TWILIGHT  violet/purple, dusk vibe
 *         SUMI_E    white-paper inverted (Japanese-ink aesthetic)
 *
 * Section map:
 *   §1 config     — tunables, palettes, food patterns (every literal named)
 *   §2 clock      — monotonic timer + sleep (delays live here + main loop)
 *   §3 color      — theme → ncurses colour pairs (a terminal effect)
 *   §4 geometry   — Cell / Step / DIR8: grid-coordinate + direction maths
 *   §5 field      — PheromoneField: the τ trail substrate + its operations
 *   §6 agents     — Ant / Colony: the foragers + their data model
 *   §7 scene      — Environment + Settings + Scene; layout, reset, ONE tick
 *   §8 render     — PURE: const Scene → screen; mutates no state
 *   §9 app        — orchestration: sequences input, effects, delay, render
 *
 * Keys:
 *   q / ESC      quit
 *   space / p    pause / resume simulation
 *   r            reset (clear pheromones, respawn ants)
 *   n / N        next / previous food pattern
 *   t / T        next / previous theme
 *   + / =        sim speed up (next rung of the speed ladder)
 *   -            sim speed down (down to 0.25x slow motion)
 *   ] / [        sim Hz up / down (render pacing, advanced)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/ant_colony.c \
 *       -o ant_colony -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Ant Colony Optimisation (ACO) — stigmergic path
 *                  finding via pheromone field reinforcement.
 *
 *                  ANT STATE MACHINE:
 *                     SEARCHING:
 *                       — pick next cell from a forward-arc of three
 *                         8-directional candidates, weighted by
 *                         (pheromone_at(cell) + base) · food_bias
 *                       — deposit DEPOSIT_SEARCH on current cell
 *                       — on entering food's detection radius:
 *                         flip to RETURNING; head turns toward nest
 *                     RETURNING:
 *                       — bee-line toward nest with ±1-step random
 *                         deviation (avoids over-straight paths)
 *                       — deposit DEPOSIT_RETURN (~10× SEARCH)
 *                       — on entering nest's detection radius:
 *                         increment delivered; flip to SEARCHING
 *
 *                  PHEROMONE FIELD:
 *                     Per-cell scalar τ; each tick:
 *                       τ_new = max(0, τ × (1 − EVAP_RATE) + Δ)
 *                     where Δ is the sum of deposits this tick.
 *                     Evaporation makes disused paths fade →
 *                     the colony adapts to changes in environment.
 *
 *                  WHY SHORTEST PATHS EMERGE (autocatalysis):
 *                     Two paths from nest to food, lengths L1 < L2.
 *                     An ant on L1 returns sooner → reinforces L1
 *                     while its peer on L2 is still in transit.  Next
 *                     ant chooses biased by current trail strength →
 *                     more likely to take L1.  Positive feedback
 *                     compounds; within ~30 sec the colony converges
 *                     on L1 even though no individual ant knows it's
 *                     shorter.  Real ants do this in nature
 *                     (Deneubourg et al. 1990 double-bridge experiment).
 *
 * Data-structure : Four composable abstractions, all living in one
 *                  `Scene` (BSS, no malloc).  `PheromoneField` wraps the
 *                  τ[H][W] float buffer; `Colony` is the `Ant` pool;
 *                  `Environment` holds the nest + food list + delivered
 *                  tally; `Settings` the UI knobs.  `Cell` / `Step` are
 *                  the grid-coordinate and 8-direction maths primitives
 *                  everything else is built on.
 *
 * Rendering      : ASCII only.  Pheromone shown as 4-tier glyph ramp
 *                  (`.`/`:`/`+`/`#`) coloured by theme palette.  Ants
 *                  rendered as `o` in state-coloured pairs; nest as
 *                  `@`, food as `*` with bracket frame.
 *
 * Performance    : O(N_ants × ants_per_tick + grid_cells × evap) per
 *                  tick.  At N_ants = 80 and 200×60 grid: ~12 K cells
 *                  × evap + 80 ants × small constant work ≈ 14 K ops.
 *                  Trivial — holds well past 60 fps with multi-tick
 *                  speed multipliers.
 *
 * References     :
 *
 *   CONCEPTS — stigmergy & ant colony optimisation
 *   • Grassé, P.-P. (1959) — "La reconstruction du nid et les
 *     coordinations interindividuelles…", *Insectes Sociaux* 6:41-80.
 *     Coined "stigmergy": coordination through traces left in the
 *     environment — the mechanism the pheromone field models.
 *   • Goss, S., Aron, S., Deneubourg, J.-L. & Pasteels, J. M. (1989) —
 *     "Self-organized shortcuts in the Argentine ant",
 *     *Naturwissenschaften* 76:579-581.  The binary-bridge experiment;
 *     shortest-path emergence from local trail reinforcement.
 *   • Deneubourg, J.-L., Aron, S., Goss, S. & Pasteels, J. M. (1990) —
 *     "The self-organizing exploratory pattern of the Argentine ant",
 *     *Journal of Insect Behavior* 3(2):159-168.  The double-bridge
 *     experiment this demo replicates.
 *   • Dorigo, M., Maniezzo, V. & Colorni, A. (1996) — "Ant System:
 *     optimization by a colony of cooperating agents", *IEEE Trans.
 *     Systems, Man & Cybernetics-B* 26(1):29-41.  First formalisation
 *     of the deposit / evaporate update rule used here.
 *   • Dorigo, M. & Stützle, T. (2004) — *Ant Colony Optimization*,
 *     MIT Press.  The canonical book on the metaheuristic.
 *   • Bonabeau, E., Dorigo, M. & Theraulaz, G. (1999) — *Swarm
 *     Intelligence: From Natural to Artificial Systems*, Oxford.
 *     Stigmergy and self-organisation primer.
 *
 *   RENDERING — ASCII glyph ramps & terminal output
 *   • Bourke, P. (1997) — "Character representation of grey-scale
 *     images", paulbourke.net/dataformats/asciiart/.  The luminance →
 *     character-density mapping behind the 4-tier `. : + #` ramp.
 *   • Padala, P. (2005) — "NCURSES Programming HOWTO", TLDP
 *     (tldp.org/HOWTO/NCURSES-Programming-HOWTO/).  Reference for the
 *     colour-pair, attribute and double-buffered draw model in §3/§8.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — ABSTRACTIONS, EFFECTS, DELAYS & RENDERING ──────────── *
 *
 * The one rule that keeps this readable: a function's SIGNATURE tells you
 * whether it can change the simulation.
 *   • Takes a NON-const pointer (Scene*, PheromoneField*, Ant*, …)
 *       → it is an EFFECT; it may mutate state.
 *   • Takes a CONST pointer (const Scene*, const PheromoneField*, …)
 *       → it is a pure READ — a query or the renderer.  It cannot change
 *         what the next tick computes.
 *
 * The world is four named abstractions, each bundling its data WITH the
 * verbs that act on it (struct + operations = one little module):
 *
 *   §4 Cell / Step      grid-coordinate + the 8 Moore-neighbour steps;
 *                       pure maths (cell_step, heading_toward, …).
 *   §5 PheromoneField   τ(cell): the evaporating trail substrate.
 *                       field_deposit / field_evaporate / field_sample.
 *   §6 Colony / Ant     the foraging agents + their two-phase state.
 *   §7 Environment      the nest, the food sources, the delivered tally.
 *      Scene            composition: field + colony + environment +
 *                       Settings (UI knobs) + screen size.  Its dynamics
 *                       live here too because ONE tick (scene_step) couples
 *                       all three: ants read+write the field and env.
 *
 * DELAYS — the only time-bending, both inside main() (§9):
 *   frame sleep      clock_sleep_ns() paces rendering to cfg.sim_fps Hz.
 *   step accumulator app.tick_accum gates how many scene_steps run per
 *                    frame; speeds < 1x run one step only every few frames.
 *
 * RENDERING (§8) is PURE: render_world / render_hud read a const Scene and
 * write only the terminal.  Read main() top-to-bottom and you can see
 * exactly when each effect and delay fires — and that the final render
 * touches none of them.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Ants don't communicate.  They drop and follow chemical trails.
 * That's the whole algorithm.  An ant returning from food drops a
 * heavy trail; future searching ants prefer cells with stronger
 * trails; over time the strongest trails are the shortest paths,
 * because shorter trips reinforce faster.  The intelligence lives
 * in the FIELD, not in any ant.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the floor of your kitchen with breadcrumbs scattered around
 * a sandwich.  Drop ten kids in the room.  They wander randomly until
 * they find a crumb; once they do, they walk back to the door
 * dropping coloured chalk dust as they go.  The next kid leaving the
 * door notices the chalk and prefers to walk along it; if it leads
 * to a crumb, they follow the same path back, depositing more chalk.
 * After 5 minutes, every kid is shuttling back and forth between
 * door and crumbs along a single bright chalk path.  Nobody planned
 * the route; it self-organised.  That's ACO.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Initialise ants at the nest, all SEARCHING, random directions.
 *
 *  2. Per tick, per ant (ant_step, §7):
 *
 *       SEARCHING:
 *         deposit DEPOSIT_SEARCH at current cell
 *         pick three forward-arc 8-dir candidates (dir−1, dir, dir+1)
 *         weight each by (pheromone(cell) + base_noise) · food_bias
 *         pick proportionally to weight (choose_weighted)
 *         move; bounce if would exit grid
 *         if ant within FOOD_RADIUS of any food: flip to RETURNING
 *
 *       RETURNING:
 *         deposit DEPOSIT_RETURN at current cell
 *         pick direction = nearest 8-dir to nest ± random offset (-1, 0, +1)
 *         move; bounce if would exit grid
 *         if ant within FOOD_RADIUS of nest: delivered++, flip to SEARCHING
 *
 *  3. Per tick, evaporate the entire pheromone field (field_evaporate):
 *
 *       τ_new = τ_old · (1 − EVAP_RATE)
 *       (clip to 0 below PH_EPSILON)
 *
 *  4. Render each frame (§8): pheromone field as 4-tier glyph ramp; ants
 *     as state-coloured `o`; nest as `@`; food sources as `*`.
 *
 *  5. Cycle pattern (`n`) → reset and re-place food sources.
 *     Cycle theme (`t`) → re-init colour pairs only.
 *
 * KEY FORMULAS
 * ────────────
 *  Pheromone update:
 *    τ_{t+1} = max(0, τ_t · (1 − EVAP_RATE) + Σ_deposits)
 *    EVAP_RATE = 0.003 → e-fold decay every ~333 ticks.
 *
 *  Direction-biased weight (search step):
 *    w(d) = (τ(neighbour_d) + ε_base) · max(0, FOOD_BIAS · (n̂_d · n̂_food + 1))
 *    P(d) = w(d) / Σ_d' w(d')
 *
 *  8-direction unit steps (Moore neighbourhood), index 0..7 = N..NW:
 *    DIR8[d] = (drow, dcol)
 *            = {(-1, 0),(-1, 1),(0, 1),(1, 1),(1, 0),(1,-1),(0,-1),(-1,-1)}
 *
 *  Best-direction-toward (returning ants):
 *    ideal_dir = argmax_d (DIR8[d].dcol · Δcol + DIR8[d].drow · Δrow)
 *    where Δ = (nest − ant) (or (food − ant) for searchers)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • PHEROMONE OVERFLOW.  Clipped at PH_MAX = 1.0 to prevent runaway
 *    along well-used paths.  Without the cap, a heavily-used route
 *    saturates so much that ants would never explore alternatives.
 *
 *  • DEAD ANTS (stuck at boundary).  If an ant tries to move out of
 *    the grid, we reverse its direction.  If reversing also fails
 *    (corner case — narrow grid + bad dir), the ant stays put for
 *    one tick.  Eventually picks a new direction.
 *
 *  • CONVERGENCE TIME.  Default ~30 sec for the colony to settle
 *    onto its primary trails.  Reset (`r`) restarts; pattern change
 *    (`n`) also restarts, since food positions move.
 *
 *  • TWO PATTERNS, SAME GRID.  All patterns share the nest at
 *    centre; only food positions differ.  The pheromone field is
 *    cleared on pattern change so old trails don't bias the new
 *    layout.
 *
 *  • INPUT RESPONSIVENESS.  Input is read at 60 fps regardless of sim
 *    speed; sim ticks accumulate via a fixed-step loop (§9), so speed
 *    scaling never impacts key-poll rate.
 *
 *  • INVERTED THEME (SUMI_E).  Pre-fill white "paper" before drawing,
 *    use dark fg pairs, disable A_BOLD on ants/food/nest.  Standard
 *    repo recipe.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Watch the first 30 seconds.  Initially the pheromone trails
 *    are diffuse, ants wander broadly.  Within ~10 sec, two faint
 *    trails (nest→food[0] and nest→food[1]) emerge; within ~30 sec,
 *    they crystallise into bright `+`/`#` paths.
 *
 *  • Press `n` to switch pattern.  All trails clear; new food
 *    sources appear; the colony re-explores and finds new paths.
 *
 *  • Press `t` to cycle theme.  Trails and ants recolour instantly.
 *    Geometry unchanged.
 *
 *  • Press `+` / `-` to walk the speed ladder (0.25x … 16x).  At 0.25x
 *    the ants visibly crawl; at 16x convergence is fast.  Key presses
 *    still register immediately at any speed.
 *
 *  • Press `r`.  Reset clears trails and respawns ants at the nest.
 *    Convergence restarts.
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
    SIM_FPS_MIN      =  20,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 240,
    SIM_FPS_STEP     =  20,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_PH_BASE      =  3,    /* +0..+3 — pheromone gradient (faint→strong) */
    PAIR_ANT_S        =  7,    /* searching ant                         */
    PAIR_ANT_R        =  8,    /* returning ant                         */
    PAIR_FOOD         =  9,
    PAIR_NEST         = 10,
    PAIR_PAPER        = 11,    /* inverted-theme white bg               */

    /* Static-buffer caps. */
    GRID_W_MAX        = 320,
    GRID_H_MAX        = 100,

    /* Colony. */
    N_ANTS            =  80,
    MAX_FOOD          =  16,    /* max food sources any pattern places   */

    /* HUD: top row = data readout, bottom row = action hints.         */
    HUD_TOP_ROWS      =   1,
    HUD_BOT_ROWS      =   1,
    HUD_ROWS          = HUD_TOP_ROWS + HUD_BOT_ROWS,

    SPEED_DEFAULT_IDX =   1,    /* index into SPEED_LADDER (= 0.5x)     */

    SEARCH_ARC        =   3,    /* forward-arc candidate headings        */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))
#define DT_CAP_MS            100      /* clamp on a long frame gap       */

/* Pheromone tunables. */
#define EVAP_RATE       0.003f   /* fraction lost per tick              */
#define DEPOSIT_RETURN  0.40f
#define DEPOSIT_SEARCH  0.04f
#define PH_MAX          1.0f
#define PH_BASE_NOISE   0.05f    /* base attraction even on empty cells  */
#define PH_EPSILON      0.001f   /* below this, clip a cell to exactly 0 */
#define PH_DRAW_MIN     0.04f    /* below this, a cell draws nothing     */

#define FOOD_BIAS       3.0f     /* food-direction pull strength         */
#define FOOD_RADIUS     2        /* Manhattan radius for food/nest hit   */
#define STEER_OFFSET    1.0f     /* shifts heading·food alignment ≥0 so even
                                  * a sideways candidate keeps some weight */
#define SPAWN_RADIUS    1        /* ants spawn within ±1 cell of the nest  */
#define MIN_PLAY_ROWS   8        /* never lay out a grid shorter than this */

/* Food-layout geometry (food_layout / scene_place_food). */
#define EDGE_MARGIN_DIV   5      /* sources sit ~1/5 in from the screen edge */
#define EDGE_MARGIN_MIN_X 4      /* …but never closer than this many cells   */
#define EDGE_MARGIN_MIN_Y 3
#define RING_RADIUS_MIN_X 4      /* min ellipse radius for ring patterns     */
#define RING_RADIUS_MIN_Y 3
#define QUARTER_TURN  ((float)(M_PI / 2.0))  /* 90°: puts a ring vertex up   */

/* 256-colour anchors for the inverted (light-paper) themes. */
#define XTERM_WHITE  231         /* near-white paper background            */
#define XTERM_BLACK   16         /* black ink for glyphs on paper          */

/* Speed ladder — sim-ticks-per-render-frame multiplier.  Values < 1
 * are slow motion: the fractional accumulator in main() runs one tick
 * only every 1/value frames (0.25x → one tick per 4 frames). */
static const float SPEED_LADDER[] = {
    0.25f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 12.0f, 16.0f,
};
#define N_SPEEDS ((int)(sizeof SPEED_LADDER / sizeof SPEED_LADDER[0]))

/* Pheromone render ramp: glyphs + the lower threshold of each tier
 * above tier 0.  ramp_slot() turns an intensity into 0..3. */
static const char  PH_GLYPHS[4]  = { '.', ':', '+', '#' };
static const float PH_RAMP_T[3]  = { 0.15f, 0.40f, 0.70f };

#define ANT_GLYPH      'o'
#define FOOD_GLYPH     '*'
#define NEST_GLYPH     '@'
#define FOOD_FRAME_L   '['
#define FOOD_FRAME_R   ']'
#define NEST_FRAME_L   '('
#define NEST_FRAME_R   ')'

/* Pattern enum + per-pattern food-source layout.
 * Each pattern arranges food sources differently around the nest, so
 * the learner can watch the colony adapt its trail topology to the
 * geometry of the foraging problem.  All patterns share the same nest
 * (centre of grid) — only food layout changes. */
typedef enum {
    PAT_DOUBLE  = 0,    /* 2 sources, opposite sides — Deneubourg classic */
    PAT_SINGLE,         /* 1 source                                       */
    PAT_QUAD,           /* 4 cardinal sources                             */
    PAT_LINE,           /* 3 collinear sources on right edge              */
    PAT_HEXAGON,        /* 6 sources around nest at hex angles            */
    PAT_CROSS,          /* 4 sources at near-distance cardinals           */
    PAT_TRIANGLE,       /* 3 sources at equilateral-ish triangle          */
    PAT_CIRCLE,         /* 8 sources on a ring                            */
    PAT_DIAGONAL,       /* 4 sources along the main diagonal              */
    PAT_CLUSTER,        /* 5 sources tightly clumped on one side          */
    PAT_DISTANT,        /* 2 sources at extreme corners                   */
    PAT_PERIMETER,      /* 6 sources spread along screen edges            */
    PAT_GRID,           /* 9 sources in a 3×3 grid                        */
    PAT_RANDOM,         /* 7 sources at fixed-seed random positions       */
    N_PATTERNS,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PAT_DOUBLE:    return "DOUBLE   ";
    case PAT_SINGLE:    return "SINGLE   ";
    case PAT_QUAD:      return "QUAD     ";
    case PAT_LINE:      return "LINE     ";
    case PAT_HEXAGON:   return "HEXAGON  ";
    case PAT_CROSS:     return "CROSS    ";
    case PAT_TRIANGLE:  return "TRIANGLE ";
    case PAT_CIRCLE:    return "CIRCLE   ";
    case PAT_DIAGONAL:  return "DIAGONAL ";
    case PAT_CLUSTER:   return "CLUSTER  ";
    case PAT_DISTANT:   return "DISTANT  ";
    case PAT_PERIMETER: return "PERIMETER";
    case PAT_GRID:      return "GRID     ";
    case PAT_RANDOM:    return "RANDOM   ";
    default:            return "?        ";
    }
}

/*
 * Theme — every entry sits in the bright half of the 256-cube per
 * the CLAUDE.md "Theme Palette Brightness" rule, so even slot 0 of
 * the pheromone gradient stays visible against a black terminal.
 * NEGATIVE-flavoured "SUMI_E" theme follows the standard inverted
 * recipe (white bg + dark fg, A_BOLD disabled).
 */
typedef struct {
    const char *name;        /* 8-char fixed-width label for the HUD       */
    short       ph[4];       /* pheromone gradient, slot 0..3 faint→strong;
                              * one 256-colour per τ tier (see PH_RAMP_T),
                              * so trail strength reads as colour, not just
                              * glyph.  All entries bright-half per the
                              * "Theme Palette Brightness" rule.            */
    short       ant_s;       /* searching-ant colour                       */
    short       ant_r;       /* returning-ant colour — deliberately distinct
                              * from ant_s so the two flows are separable
                              * by eye.                                     */
    short       food;        /* food-source `[*]` marker colour            */
    short       nest;        /* nest `(@)` marker colour                    */
    bool        inverted;    /* true = light-paper recipe: white bg, dark
                              * fg, A_BOLD suppressed (SUMI_E).             */
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* CLASSIC: blue/cyan trails, warm ants.                          */
    { "CLASSIC ",
      {  39,  45,  51, 195 },    /* sky-blue → cyan → pale cyan       */
      226,                       /* searcher: yellow                  */
      201,                       /* returner: bright magenta          */
       46,                       /* food: lime                        */
      214,                       /* nest: orange                      */
      false },

    /* INFRARED: warm red→orange→yellow heat-trails.                  */
    { "INFRARED",
      { 124, 166, 208, 220 },
      226, 196,  46, 201, false },

    /* FOREST: green trails, brown ants.                              */
    { "FOREST  ",
      {  34,  70, 112, 154 },
      136, 130, 220, 196, false },

    /* NEON: vivid magenta + cyan pop.                                */
    { "NEON    ",
      { 165, 207, 213,  51 },
      226,  51,  46, 201, false },

    /* TWILIGHT: violet/lavender dusk.                                */
    { "TWILIGHT",
      {  99, 105, 147, 195 },
      226, 213,  46, 207, false },

    /* SUMI_E: white-paper bg, dark fg, ink-painting aesthetic.       */
    { "SUMI_E  ",
      { 240, 237, 234,  16 },
       60, 124,  64, 130, true  },
};

/* Wrap an index into [0, n) the long way (handles negative steps). */
static int wrap_idx(int i, int n)
{
    i %= n;
    if (i < 0) i += n;
    return i;
}

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
/* §3  color  (a terminal effect: rewrites ncurses colour pairs)          */
/* ===================================================================== */

/* Bind the theme's 256-colour palette to the pheromone / ant / marker
 * pairs.  `bg` is the shared background (paper-white or terminal default). */
static void theme_bind_truecolor(const Theme *t, short bg)
{
    for (int i = 0; i < 4; i++)
        init_pair((short)(PAIR_PH_BASE + i), t->ph[i], bg);
    init_pair(PAIR_ANT_S, t->ant_s, bg);
    init_pair(PAIR_ANT_R, t->ant_r, bg);
    init_pair(PAIR_FOOD,  t->food,  bg);
    init_pair(PAIR_NEST,  t->nest,  bg);
    init_pair(PAIR_PAPER, XTERM_BLACK, bg);
}

/* 8-colour fallback: collapse the palette to the nearest basic colours,
 * or to black-on-white ink for inverted themes. */
static void theme_bind_8color(const Theme *t)
{
    static const short fb_ph[4] = { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN };
    short bg = t->inverted ? COLOR_WHITE : -1;
    for (int i = 0; i < 4; i++)
        init_pair((short)(PAIR_PH_BASE + i), t->inverted ? COLOR_BLACK : fb_ph[i], bg);
    init_pair(PAIR_ANT_S, t->inverted ? COLOR_BLACK : COLOR_YELLOW,  bg);
    init_pair(PAIR_ANT_R, t->inverted ? COLOR_BLACK : COLOR_MAGENTA, bg);
    init_pair(PAIR_FOOD,  t->inverted ? COLOR_BLACK : COLOR_GREEN,   bg);
    init_pair(PAIR_NEST,  t->inverted ? COLOR_BLACK : COLOR_RED,     bg);
    init_pair(PAIR_PAPER, COLOR_BLACK, COLOR_WHITE);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];
    if (COLORS >= 256) theme_bind_truecolor(t, t->inverted ? XTERM_WHITE : -1);
    else               theme_bind_8color(t);
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
/* §4  geometry — Cell / Step / DIR8  (pure grid + direction maths)       */
/*                                                                        */
/* A `Cell` is a discrete grid coordinate; a `Step` is the delta to one   */
/* of the 8 Moore-neighbour cells.  DIR8 indexes headings 0..7 = N..NW.   */
/* Nothing here mutates state — these are the maths the rest is built on. */
/* ===================================================================== */

/*
 * Cell — a discrete position on the pheromone grid (one terminal
 * character cell).  Row-major to match ncurses (mvaddch takes y,x) AND
 * the τ buffer's [row][col] indexing, so a Cell drops into both with no
 * swizzling.  Integer only: the simulation is cellular — an ant either
 * occupies a cell or it doesn't, there is no sub-cell position.
 */
typedef struct {
    int row;   /* 0 .. field.h-1, top→bottom (screen y, before HUD offset) */
    int col;   /* 0 .. field.w-1, left→right (screen x)                    */
} Cell;

/*
 * Step — the integer delta from a cell to ONE of its 8 neighbours: an
 * entry of the Moore neighbourhood (the 8 cells touching a square, vs.
 * the 4-cell von Neumann neighbourhood).  Stored as a fixed table (DIR8)
 * so a heading index 0..7 turns into a move with zero branching.
 * Components ∈ {-1,0,+1}; the two are never both 0 (no null step).
 */
typedef struct {
    int drow;  /* row delta:  -1 up,   +1 down,  0 same row  */
    int dcol;  /* col delta:  -1 left, +1 right, 0 same col  */
} Step;

static const Step DIR8[8] = {
    {-1,  0},  /* 0 N  */  {-1,  1},  /* 1 NE */
    { 0,  1},  /* 2 E  */  { 1,  1},  /* 3 SE */
    { 1,  0},  /* 4 S  */  { 1, -1},  /* 5 SW */
    { 0, -1},  /* 6 W  */  {-1, -1},  /* 7 NW */
};

/* the cell reached by taking one step in `heading` (0..7). */
static Cell cell_step(Cell c, int heading)
{
    return (Cell){ c.row + DIR8[heading].drow, c.col + DIR8[heading].dcol };
}

/* the opposite heading (used to bounce off a wall). */
static int heading_reverse(int heading) { return (heading + 4) % 8; }

/* the heading whose unit step points most toward `to`. */
static int heading_toward(Cell from, Cell to)
{
    int   drow = to.row - from.row, dcol = to.col - from.col;
    float best_dot = -1e9f;
    int   best = 0;
    for (int h = 0; h < 8; h++) {
        float dot = (float)DIR8[h].dcol * dcol + (float)DIR8[h].drow * drow;
        if (dot > best_dot) { best_dot = dot; best = h; }
    }
    return best;
}

/* true when `a` is within Manhattan `radius` of `b`. */
static bool cell_in_range(Cell a, Cell b, int radius)
{
    int dr = a.row - b.row; if (dr < 0) dr = -dr;
    int dc = a.col - b.col; if (dc < 0) dc = -dc;
    return (dr + dc) <= radius;
}

/* the two compass neighbours of a heading (the search arc's outer prongs). */
static int heading_cw (int h) { return (h + 1) % 8; }   /* one step clockwise */
static int heading_ccw(int h) { return (h + 7) % 8; }   /* one step counter-cw */

/* a continuous 2D direction (dx = col/east, dy = row/south); used for the
 * sub-cell maths that integer Cells/Steps can't express, e.g. the
 * normalised food-ward vector that steers a searching ant. */
typedef struct { float dx, dy; } Vec2f;

/* unit vector pointing from `from` toward `to` (zero when they coincide). */
static Vec2f unit_toward(Cell from, Cell to)
{
    float dx = (float)(to.col - from.col), dy = (float)(to.row - from.row);
    float len = sqrtf(dx * dx + dy * dy);
    if (len > 0.0f) { dx /= len; dy /= len; }
    return (Vec2f){ dx, dy };
}

/* smaller / larger of two ints — layout clamps read better named. */
static int imin(int a, int b) { return a < b ? a : b; }
static int imax(int a, int b) { return a > b ? a : b; }

/* roulette-wheel: pick index in [0,n) with probability weight[i]/Σweight.
 * Degenerate all-zero weights fall back to the middle candidate. */
static int choose_weighted(const float *weight, int n)
{
    float total = 0.0f;
    for (int i = 0; i < n; i++) total += weight[i];
    if (total <= 0.0f) return n / 2;

    float r   = (float)rand() / (float)RAND_MAX * total;
    float acc = 0.0f;
    for (int i = 0; i < n; i++) {
        acc += weight[i];
        if (r <= acc) return i;
    }
    return n - 1;
}

/* ===================================================================== */
/* §5  field — PheromoneField: the τ trail substrate                      */
/*                                                                        */
/* The central data structure: a 2D scalar field τ(cell) the ants write   */
/* to and read from.  `field_sample` / `field_in_bounds` are pure reads   */
/* (const); the rest are EFFECTS that mutate the field.                   */
/* ===================================================================== */

/*
 * PheromoneField — the shared stigmergic medium: a scalar field τ(cell)
 * that every ant writes to (deposit) and reads from (sample).  This is
 * where ACO keeps its "memory": no ant remembers a route, the FIELD does.
 * Stigmergy = coordination through traces left in the environment
 * (Grassé 1959); the field IS that environment.
 *
 *   WHY a dense 2D array (not a sparse map): deposits and the per-tick
 *   evaporation sweep both touch cells uniformly, the grid is tiny
 *   (≤ 320×100), so a flat buffer is cache-friendly and needs no malloc.
 *   Fixed GRID_*_MAX caps let it live in BSS (~128 KB) — see "Memory
 *   Allocation" in CLAUDE.md; w/h track the sub-rectangle actually in use.
 *
 *   τ DYNAMICS (Dorigo & Stützle 2004, "Ant Colony Optimization", §1.4):
 *     deposit     τ += Δ          Δ = DEPOSIT_SEARCH (light) | DEPOSIT_RETURN (heavy)
 *     saturate    τ  = min(τ, PH_MAX)     caps runaway autocatalysis
 *     evaporate   τ *= (1 − EVAP_RATE)    forgets disused trails
 *   The cap + evaporation together are what let the colony ADAPT when
 *   food moves — without them an over-reinforced path could never fade.
 */
typedef struct {
    int   w, h;   /* active sub-rectangle in use this layout (≤ GRID_*_MAX) */
    float tau[GRID_H_MAX][GRID_W_MAX];
                  /* τ[row][col] ∈ [0, PH_MAX]: 0 = no trail, PH_MAX = a
                   * saturated highway.  [row][col] order matches Cell so
                   * field_sample(f, cell) indexes without reordering.      */
} PheromoneField;

static void field_resize(PheromoneField *f, int w, int h)
{
    f->w = w < GRID_W_MAX ? w : GRID_W_MAX;
    f->h = h < GRID_H_MAX ? h : GRID_H_MAX;
}

static bool field_in_bounds(const PheromoneField *f, Cell c)
{
    return c.row >= 0 && c.row < f->h && c.col >= 0 && c.col < f->w;
}

static float field_sample(const PheromoneField *f, Cell c)
{
    return field_in_bounds(f, c) ? f->tau[c.row][c.col] : 0.0f;
}

static void field_clear(PheromoneField *f)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            f->tau[r][c] = 0.0f;
}

/* add `amount` pheromone at a cell, saturating at PH_MAX. */
static void field_deposit(PheromoneField *f, Cell c, float amount)
{
    if (!field_in_bounds(f, c)) return;
    f->tau[c.row][c.col] += amount;
    if (f->tau[c.row][c.col] > PH_MAX) f->tau[c.row][c.col] = PH_MAX;
}

/* one tick of decay: τ ← τ·(1−EVAP_RATE), clipped to 0 below PH_EPSILON. */
static void field_evaporate(PheromoneField *f)
{
    float decay = 1.0f - EVAP_RATE;
    for (int r = 0; r < f->h; r++) {
        for (int c = 0; c < f->w; c++) {
            f->tau[r][c] *= decay;
            if (f->tau[r][c] < PH_EPSILON) f->tau[r][c] = 0.0f;
        }
    }
}

/* pheromone intensity → glyph/colour tier 0..3 (a pure render query). */
static int ramp_slot(float tau)
{
    int slot = 0;
    for (int i = 0; i < 3; i++)
        if (tau >= PH_RAMP_T[i]) slot = i + 1;
    return slot;
}

/* ===================================================================== */
/* §6  agents — Ant / Colony: the foragers                                */
/*                                                                        */
/* Pure data model + one pure query.  The agents' BEHAVIOUR (how they     */
/* move and deposit) needs the whole Scene, so it lives in §7 with the    */
/* tick that drives it.                                                   */
/* ===================================================================== */

/*
 * AntPhase — the two halves of a forager's round trip.  An ant is a
 * minimal 2-state machine (Deneubourg et al. 1990, double-bridge model):
 * outbound looking for food, then inbound carrying it home.  The phase
 * selects BOTH the move rule (explore vs. bee-line) AND the deposit
 * weight (light vs. heavy).  Heavy return-trip deposits are what bias the
 * next searchers — the engine of the autocatalytic feedback loop.
 */
typedef enum {
    ANT_SEARCHING,   /* outbound: trail-biased forward-arc walk → food  */
    ANT_RETURNING,   /* inbound:  bee-line to nest, heavy deposit       */
} AntPhase;

/*
 * Ant — one forager.  Deliberately tiny: a position, a heading, a phase.
 * An ant carries NO map and NO memory of its route; all path knowledge
 * lives in the PheromoneField.  That minimality is the whole point —
 * global shortest-path structure emerges from local, near-mindless
 * agents (Bonabeau, Dorigo & Theraulaz 1999, "Swarm Intelligence").
 */
typedef struct {
    Cell     at;       /* current cell, always kept inside the field      */
    int      heading;  /* facing, 0..7 index into DIR8.  Gives the search
                        * step a forward arc {dir-1, dir, dir+1} so an ant
                        * has momentum instead of jittering in place.      */
    AntPhase phase;    /* which move/deposit rule applies this tick        */
} Ant;

/*
 * Colony — the fixed pool of foragers.  Fixed size (N_ANTS) so it lives
 * in BSS with no allocation; ants are never created or destroyed at
 * runtime, only reset (ant_spawn) and flipped between phases.  More ants
 * ⇒ denser trails and faster convergence, at linear cost per tick.
 */
typedef struct {
    Ant ant[N_ANTS];   /* indexed pool; index has no meaning, order-free  */
} Colony;

static int colony_returning(const Colony *col)
{
    int n = 0;
    for (int i = 0; i < N_ANTS; i++)
        if (col->ant[i].phase == ANT_RETURNING) n++;
    return n;
}

/* ===================================================================== */
/* §7  scene — Environment + Settings + Scene; layout, reset, ONE tick    */
/*                                                                        */
/* `Scene` composes every abstraction into the simulated world plus its   */
/* UI knobs.  All the EFFECTS that advance a tick live here, because one   */
/* tick couples field + colony + environment.                             */
/* ===================================================================== */

/*
 * Environment — the foraging PROBLEM the colony solves: one nest (home)
 * and a set of food sources.  Pure geometry; it holds no trail state.
 * Kept separate from the PheromoneField so a pattern change can rewrite
 * food[] and clear τ independently — the problem moves, the medium
 * resets.  `delivered` is the single live scalar: the success metric.
 */
typedef struct {
    Cell nest;             /* home cell, always the grid centre; serves as
                            * both the spawn point and the return target.  */
    Cell food[MAX_FOOD];   /* source cells for the active pattern; an ant
                            * within FOOD_RADIUS of any one picks up food.  */
    int  food_n;           /* sources actually placed, 1..MAX_FOOD; varies
                            * by Pattern (SINGLE=1 … CIRCLE=8 … GRID=9).    */
    long delivered;        /* lifetime parcels returned to the nest — the
                            * HUD "food:" counter and a convergence proxy
                            * (climbs faster once trails form).  `long`
                            * because at 16x it can run up for minutes.     */
} Environment;

/*
 * Settings — the user-facing knobs, kept apart from world state so that
 * INPUT mutates these while the SIMULATION mutates field/colony/env.
 * Several are stored as indices (not live values) so cycling wraps
 * cleanly and the HUD can show "current/total"; the index→value lookup
 * happens at the point of use (SPEED_LADDER[], themes[], pattern switch).
 */
typedef struct {
    bool paused;     /* freeze stepping; render + input keep running.     */
    int  speed_idx;  /* index into SPEED_LADDER[]: 0=0.25x … N_SPEEDS-1=16x.
                      * an index, not a float, so +/- walks fixed rungs.   */
    int  pattern;    /* current Pattern, 0..N_PATTERNS-1; selects food[].  */
    int  theme;      /* current palette, 0..N_THEMES-1; colour only, no
                      * effect on geometry or dynamics.                    */
    int  sim_fps;    /* render-pacing target in Hz, SIM_FPS_MIN..MAX.  Sets
                      * the frame sleep — NOT how fast the sim runs (that's
                      * speed_idx) — so smoothness and sim rate decouple.  */
} Settings;

/*
 * Scene — the whole simulated world in one value: the trail medium, the
 * agents, the problem, the knobs, and the viewport.  This is the SINGLE
 * root of state (one global App.scene), so "what is the program doing?"
 * has exactly one answer.  The member order mirrors the data-flow of a
 * tick: ants (colony) read/write the field, guided by env; cfg/cols/rows
 * are presentation.  Effects take `Scene *` and mutate the first three;
 * input touches `cfg`; the renderer reads it all through `const`.
 */
typedef struct {
    PheromoneField field;   /* §5 — τ trail substrate (the shared memory)  */
    Colony         colony;  /* §6 — the foragers                           */
    Environment    env;     /* §7 — nest + food sources + delivered tally  */
    Settings       cfg;     /* UI knobs (pattern/theme/speed/pacing/pause) */
    int            cols;    /* terminal width  (cells); field.w ≤ cols     */
    int            rows;    /* terminal height (cells); field.h ≤ rows−HUD */
} Scene;

static float scene_speed(const Scene *s) { return SPEED_LADDER[s->cfg.speed_idx]; }

/* index of the food source nearest to `from` (squared distance is enough). */
static int nearest_food(const Environment *e, Cell from)
{
    float best = 1e18f;
    int   fi   = 0;
    for (int k = 0; k < e->food_n; k++) {
        float dr = (float)(e->food[k].row - from.row);
        float dc = (float)(e->food[k].col - from.col);
        float d2 = dr * dr + dc * dc;
        if (d2 < best) { best = d2; fi = k; }
    }
    return fi;
}

static Cell nearest_food_cell(const Environment *e, Cell from)
{
    return e->food[nearest_food(e, from)];
}

/*
 * search_weight — the ACO probabilistic transition rule for ONE candidate
 * heading, specialised to this demo's 3-way forward arc.  A heading's
 * attractiveness is trail strength × food-ward steering; proportional
 * selection over the arc (choose_weighted) is what makes short, well-trodden
 * routes win over time (Dorigo, Maniezzo & Colorni 1996).
 *
 *   trail = τ(neighbour) + PH_BASE_NOISE     ε keeps blank cells walkable
 *   steer = heading · food_dir               alignment, ±√2 for diagonals
 *   weight = trail · max(0, FOOD_BIAS·(steer + STEER_OFFSET))
 * The +STEER_OFFSET shift and the ≥0 clamp keep a sideways candidate in
 * the running while a backward-facing one drops out.
 */
static float search_weight(const PheromoneField *f, Cell at, int h, Vec2f food_dir)
{
    float trail = field_sample(f, cell_step(at, h)) + PH_BASE_NOISE;
    float steer = (float)DIR8[h].dcol * food_dir.dx
                + (float)DIR8[h].drow * food_dir.dy;
    float bias  = FOOD_BIAS * (steer + STEER_OFFSET);
    if (bias < 0.0f) bias = 0.0f;
    return trail * bias;
}

/*
 * step_with_fallback — move one cell toward `primary`; if the grid edge
 * blocks it, deflect to `fallback`; if that is blocked too, stay put.
 * Reports the heading actually adopted through *used.  This is the shared
 * wall-handling for both ant phases (search reverses, return falls back to
 * its straight aim).
 */
static Cell step_with_fallback(const PheromoneField *f, Cell at,
                               int primary, int fallback, int *used)
{
    Cell next = cell_step(at, primary);
    if (field_in_bounds(f, next)) { *used = primary; return next; }
    *used = fallback;
    next = cell_step(at, fallback);
    return field_in_bounds(f, next) ? next : at;
}

/* nudge a heading by a random −1/0/+1 — spreads return trails into a usable
 * width instead of a single ruler-straight line. */
static int heading_wobble(int h) { return (h + (rand() % 3 - 1) + 8) % 8; }

/* index of a food source the ant is standing on (within FOOD_RADIUS), else -1. */
static int food_underfoot(const Environment *e, Cell at)
{
    for (int k = 0; k < e->food_n; k++)
        if (cell_in_range(at, e->food[k], FOOD_RADIUS)) return k;
    return -1;
}

/* is the ant home — within FOOD_RADIUS of the nest? */
static bool at_nest(const Environment *e, Cell at)
{
    return cell_in_range(at, e->nest, FOOD_RADIUS);
}

/* — the ant state machine ------------------------------------------------ *
 * Each phase reads as four named steps: lay a trail, pick a heading, move
 * (deflecting off walls), and maybe flip phase on reaching the target.
 * Both take Scene* because they read+write the field and read the env. */

static void ant_search_step(Scene *s, Ant *a)
{
    PheromoneField *f = &s->field;

    /* 1. lay a faint exploratory trail at the current cell. */
    field_deposit(f, a->at, DEPOSIT_SEARCH);

    /* 2. weight the forward arc {ccw, straight, cw} by trail × food-bias. */
    int   arc[SEARCH_ARC] = { heading_ccw(a->heading), a->heading,
                              heading_cw(a->heading) };
    Vec2f food_dir = unit_toward(a->at, nearest_food_cell(&s->env, a->at));
    float weight[SEARCH_ARC];
    for (int k = 0; k < SEARCH_ARC; k++)
        weight[k] = search_weight(f, a->at, arc[k], food_dir);

    /* 3. pick proportionally and advance, deflecting off the grid edge. */
    int heading = arc[choose_weighted(weight, SEARCH_ARC)];
    a->at = step_with_fallback(f, a->at, heading, heading_reverse(heading), &heading);
    a->heading = heading;

    /* 4. on touching food, flip to RETURNING and aim at the nest. */
    if (food_underfoot(&s->env, a->at) >= 0) {
        a->phase   = ANT_RETURNING;
        a->heading = heading_toward(a->at, s->env.nest);
    }
}

static void ant_return_step(Scene *s, Ant *a)
{
    PheromoneField *f = &s->field;

    /* 1. lay a STRONG trail — this is what biases the next searchers. */
    field_deposit(f, a->at, DEPOSIT_RETURN);

    /* 2. aim at the nest with a ±1 wobble; deflect to the straight aim
     *    if the wobble would cross the grid edge. */
    int aim = heading_toward(a->at, s->env.nest);
    int heading;
    a->at = step_with_fallback(f, a->at, heading_wobble(aim), aim, &heading);
    a->heading = heading;

    /* 3. on reaching the nest, deliver and start searching afresh. */
    if (at_nest(&s->env, a->at)) {
        s->env.delivered++;
        a->phase   = ANT_SEARCHING;
        a->heading = rand() % 8;
    }
}

static void ant_step(Scene *s, Ant *a)
{
    if (a->phase == ANT_SEARCHING) ant_search_step(s, a);
    else                           ant_return_step(s, a);
}

/* a random offset in [-radius, +radius] (ant scatter around the nest). */
static int jitter(int radius) { return rand() % (2 * radius + 1) - radius; }

/* (re)spawn one ant scattered near the nest, searching, random heading. */
static void ant_spawn(Scene *s, Ant *a)
{
    a->at.row  = s->env.nest.row + jitter(SPAWN_RADIUS);
    a->at.col  = s->env.nest.col + jitter(SPAWN_RADIUS);
    a->heading = rand() % 8;
    a->phase   = ANT_SEARCHING;
}

/* ONE simulation tick: every ant moves, then the field evaporates. */
static void scene_step(Scene *s)
{
    for (int i = 0; i < N_ANTS; i++) ant_step(s, &s->colony.ant[i]);
    field_evaporate(&s->field);
}

/*
 * Derived geometry for placing food: the nest centre (cx,cy), the ellipse
 * radii (rx,ry) for ring layouts, and the edge margins — all from the grid
 * size, clamped so even a tiny terminal stays sensible.
 */
typedef struct { int cx, cy, rx, ry, margin_x, margin_y; } FoodLayout;

static FoodLayout food_layout(int cols, int rows)
{
    FoodLayout L;
    L.margin_x = imax(cols / EDGE_MARGIN_DIV, EDGE_MARGIN_MIN_X);
    L.margin_y = imax(rows / EDGE_MARGIN_DIV, EDGE_MARGIN_MIN_Y);
    L.cx = cols / 2;
    L.cy = rows / 2;
    L.rx = imax(cols / 2 - L.margin_x / 2, RING_RADIUS_MIN_X);
    L.ry = imax(rows / 2 - L.margin_y / 2, RING_RADIUS_MIN_Y);
    return L;
}

/* cell at `angle` on the ellipse (rx,ry) about the nest — shared placement
 * maths for the HEXAGON / CIRCLE / TRIANGLE ring patterns. */
static Cell ring_point(FoodLayout L, float angle)
{
    return (Cell){ L.cy + (int)(sinf(angle) * (float)L.ry),
                   L.cx + (int)(cosf(angle) * (float)L.rx) };
}

/* angle of the k-th of n points evenly spaced around a full turn. */
static float ring_angle(int k, int n) { return (float)k * (float)(2.0 * M_PI / n); }

/*
 * scene_place_food — fill env.food[] / food_n for the current pattern,
 * using the field's current extents and the centre nest.
 */
static void scene_place_food(Scene *s)
{
    Environment *e = &s->env;
    int cols = s->field.w, rows = s->field.h;
    FoodLayout L = food_layout(cols, rows);
    int cx = L.cx, cy = L.cy, rx = L.rx, ry = L.ry;
    int margin_x = L.margin_x, margin_y = L.margin_y;

    switch (s->cfg.pattern) {
    case PAT_SINGLE:
        e->food_n  = 1;
        e->food[0] = (Cell){ rows / 4, cols - margin_x };
        break;
    case PAT_QUAD:
        e->food_n  = 4;
        e->food[0] = (Cell){ margin_y,        cx };
        e->food[1] = (Cell){ rows - margin_y, cx };
        e->food[2] = (Cell){ cy, margin_x };
        e->food[3] = (Cell){ cy, cols - margin_x };
        break;
    case PAT_LINE:
        e->food_n  = 3;
        e->food[0] = (Cell){ rows / 4,     cols - margin_x };
        e->food[1] = (Cell){ cy,           cols - margin_x };
        e->food[2] = (Cell){ rows * 3 / 4, cols - margin_x };
        break;
    case PAT_HEXAGON:
        /* 6 sources evenly spaced on the ring (60° apart). */
        e->food_n = 6;
        for (int k = 0; k < 6; k++)
            e->food[k] = ring_point(L, ring_angle(k, 6));
        break;
    case PAT_CROSS:
        e->food_n  = 4;
        e->food[0] = (Cell){ cy - ry / 2, cx };
        e->food[1] = (Cell){ cy + ry / 2, cx };
        e->food[2] = (Cell){ cy, cx - rx / 2 };
        e->food[3] = (Cell){ cy, cx + rx / 2 };
        break;
    case PAT_TRIANGLE:
        /* equilateral triangle, one vertex straight up (−90° start). */
        e->food_n = 3;
        for (int k = 0; k < 3; k++)
            e->food[k] = ring_point(L, -QUARTER_TURN + ring_angle(k, 3));
        break;
    case PAT_CIRCLE:
        /* 8 sources evenly spaced on the ring (45° apart). */
        e->food_n = 8;
        for (int k = 0; k < 8; k++)
            e->food[k] = ring_point(L, ring_angle(k, 8));
        break;
    case PAT_DIAGONAL:
        e->food_n  = 4;
        e->food[0] = (Cell){ margin_y,        margin_x };
        e->food[1] = (Cell){ cy - ry / 3,     cx - rx / 3 };
        e->food[2] = (Cell){ cy + ry / 3,     cx + rx / 3 };
        e->food[3] = (Cell){ rows - margin_y, cols - margin_x };
        break;
    case PAT_CLUSTER:
        /* 5 sources tightly grouped on the right side. */
        e->food_n  = 5;
        e->food[0] = (Cell){ cy - 2, cols - margin_x };
        e->food[1] = (Cell){ cy + 2, cols - margin_x };
        e->food[2] = (Cell){ cy,     cols - margin_x - 3 };
        e->food[3] = (Cell){ cy - 1, cols - margin_x + 2 };
        e->food[4] = (Cell){ cy + 1, cols - margin_x + 2 };
        break;
    case PAT_DISTANT:
        e->food_n  = 2;
        e->food[0] = (Cell){ margin_y,        margin_x };
        e->food[1] = (Cell){ rows - margin_y, cols - margin_x };
        break;
    case PAT_PERIMETER:
        e->food_n  = 6;
        e->food[0] = (Cell){ margin_y,        cols / 4 };
        e->food[1] = (Cell){ margin_y,        cols * 3 / 4 };
        e->food[2] = (Cell){ cy,              margin_x };
        e->food[3] = (Cell){ cy,              cols - margin_x };
        e->food[4] = (Cell){ rows - margin_y, cols / 4 };
        e->food[5] = (Cell){ rows - margin_y, cols * 3 / 4 };
        break;
    case PAT_GRID: {
        /* 3×3 grid of sources, skipping the centre (the nest). */
        e->food_n = 0;
        int gx[3] = { margin_x, cx, cols - margin_x };
        int gy[3] = { margin_y, cy, rows - margin_y };
        for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++) {
                if (i == 1 && j == 1) continue;     /* skip nest cell */
                e->food[e->food_n++] = (Cell){ gy[j], gx[i] };
            }
        break;
    }
    case PAT_RANDOM: {
        /* 7 sources at fixed-seed random positions inside the play area
         * — same layout every reset, but the geometry looks irregular. */
        e->food_n = 7;
        unsigned rng = 0xC0FFEEu;
        for (int k = 0; k < 7; k++) {
            rng = rng * 1664525u + 1013904223u;
            int rc = (int)(rng >> 16) % (cols - 2 * margin_x);
            rng = rng * 1664525u + 1013904223u;
            int rr = (int)(rng >> 16) % (rows - 2 * margin_y);
            e->food[k] = (Cell){ margin_y + rr, margin_x + rc };
        }
        break;
    }
    case PAT_DOUBLE:
    default:
        e->food_n  = 2;
        e->food[0] = (Cell){ rows / 3,     margin_x };
        e->food[1] = (Cell){ rows * 2 / 3, cols - margin_x };
        break;
    }
}

/* size the field + nest from the screen, then place food for the pattern. */
static void scene_layout(Scene *s)
{
    int play_rows = imax(s->rows - HUD_ROWS, MIN_PLAY_ROWS);

    field_resize(&s->field, s->cols, play_rows);
    s->env.nest = (Cell){ s->field.h / 2, s->field.w / 2 };
    scene_place_food(s);
}

/* full restart: re-layout, clear field + tally, respawn the colony. */
static void scene_reset(Scene *s)
{
    scene_layout(s);
    s->env.delivered = 0;
    field_clear(&s->field);
    for (int i = 0; i < N_ANTS; i++) ant_spawn(s, &s->colony.ant[i]);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cols          = cols;
    s->rows          = rows;
    s->cfg.paused    = false;
    s->cfg.speed_idx = SPEED_DEFAULT_IDX;
    s->cfg.pattern   = PAT_DOUBLE;
    s->cfg.theme     = 0;
    s->cfg.sim_fps   = SIM_FPS_DEFAULT;
    scene_reset(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    scene_reset(s);
}

static void scene_cycle_pattern(Scene *s, int dir)
{
    s->cfg.pattern = wrap_idx(s->cfg.pattern + dir, N_PATTERNS);
    scene_reset(s);
}

static void scene_cycle_theme(Scene *s, int dir)
{
    s->cfg.theme = wrap_idx(s->cfg.theme + dir, N_THEMES);
    theme_apply(s->cfg.theme);
}

/* ===================================================================== */
/* §8  render — PURE: const Scene → screen                                */
/*                                                                        */
/* These read state and paint it.  They take only `const Scene *` and     */
/* never alter the scene — only the terminal.  Grid row r maps to screen  */
/* row r + HUD_TOP_ROWS, leaving row 0 free for the data bar.             */
/* ===================================================================== */

/* draw a framed glyph marker (`[*]`, `(@)`) at a cell, clipped to the grid. */
static void draw_marker(Cell c, int top, int gw, int gh,
                        char left, char mid, char right)
{
    if (c.row < 0 || c.row >= gh) return;
    int y = c.row + top;
    if (c.col - 1 >= 0 && c.col - 1 < gw) mvaddch(y, c.col - 1, (chtype)(unsigned char)left);
    if (c.col     >= 0 && c.col     < gw) mvaddch(y, c.col,     (chtype)(unsigned char)mid);
    if (c.col + 1 >= 0 && c.col + 1 < gw) mvaddch(y, c.col + 1, (chtype)(unsigned char)right);
}

/* attribute for a τ tier: strongest trail bold, faintest dim, else normal;
 * inverted (paper) themes stay normal so dark ink reads cleanly on white. */
static attr_t trail_attr(int slot, bool inverted)
{
    if (inverted)  return A_NORMAL;
    if (slot == 3) return A_BOLD;     /* tier 3 — a saturated highway   */
    if (slot == 0) return A_DIM;      /* tier 0 — a faint whisper       */
    return A_NORMAL;
}

/* fill the play area with blank "paper" for inverted (light-bg) themes. */
static void render_paper(const PheromoneField *f, int top)
{
    attron(COLOR_PAIR(PAIR_PAPER));
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            mvaddch(r + top, c, ' ');
    attroff(COLOR_PAIR(PAIR_PAPER));
}

/* paint the pheromone field as a 4-tier glyph ramp — the visible trails. */
static void render_trails(const PheromoneField *f, bool inverted, int top)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++) {
            float tau = f->tau[r][c];
            if (tau < PH_DRAW_MIN) continue;          /* nothing to show */
            int    slot = ramp_slot(tau);
            attr_t attr = trail_attr(slot, inverted);
            attron(COLOR_PAIR(PAIR_PH_BASE + slot) | attr);
            mvaddch(r + top, c, (chtype)(unsigned char)PH_GLYPHS[slot]);
            attroff(COLOR_PAIR(PAIR_PH_BASE + slot) | attr);
        }
}

/* draw all food sources as `[*]`. */
static void render_food(const Environment *e, const PheromoneField *f,
                        attr_t attr, int top)
{
    attron(COLOR_PAIR(PAIR_FOOD) | attr);
    for (int k = 0; k < e->food_n; k++)
        draw_marker(e->food[k], top, f->w, f->h,
                    FOOD_FRAME_L, FOOD_GLYPH, FOOD_FRAME_R);
    attroff(COLOR_PAIR(PAIR_FOOD) | attr);
}

/* draw the nest as `(@)`. */
static void render_nest(const Environment *e, const PheromoneField *f,
                        attr_t attr, int top)
{
    attron(COLOR_PAIR(PAIR_NEST) | attr);
    draw_marker(e->nest, top, f->w, f->h, NEST_FRAME_L, NEST_GLYPH, NEST_FRAME_R);
    attroff(COLOR_PAIR(PAIR_NEST) | attr);
}

/* draw each ant as `o`, coloured by phase (searching vs. returning). */
static void render_colony(const Colony *col, const PheromoneField *f,
                          attr_t attr, int top)
{
    for (int i = 0; i < N_ANTS; i++) {
        Cell at = col->ant[i].at;
        if (at.row < 0 || at.row >= f->h || at.col < 0 || at.col >= f->w) continue;
        int pair = (col->ant[i].phase == ANT_RETURNING) ? PAIR_ANT_R : PAIR_ANT_S;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(at.row + top, at.col, (chtype)(unsigned char)ANT_GLYPH);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* compose one frame, back to front: paper, trails, food, nest, ants. */
static void render_world(const Scene *s)
{
    bool   inverted = themes[s->cfg.theme].inverted;
    int    top      = HUD_TOP_ROWS;             /* grid row r → screen r+top */
    attr_t marker   = inverted ? A_NORMAL : A_BOLD;

    if (inverted) render_paper(&s->field, top);
    render_trails(&s->field,   inverted, top);
    render_food  (&s->env,     &s->field, marker, top);
    render_nest  (&s->env,     &s->field, marker, top);
    render_colony(&s->colony,  &s->field, marker, top);
}

/* draw one full-width HUD bar: fill the row, then write the text clipped
 * to `cols` so a long string never wraps onto the scene. */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* format the top-bar readout: state, pattern/theme (with current/total),
 * returning-ant count, delivered tally, speed, and timing. */
static void hud_status_line(char *buf, size_t n, const Scene *s, double fps)
{
    const Settings *cfg = &s->cfg;
    snprintf(buf, n,
             " ANT_COLONY  %s  pattern:%s %2d/%d  theme:%s %d/%d  "
             "ret:%2d  food:%4ld  speed:%4gx  %5.1f fps  %3d Hz ",
             cfg->paused ? "PAUSED" : "FORAGE",
             pattern_name((Pattern)cfg->pattern), cfg->pattern + 1, N_PATTERNS,
             themes[cfg->theme].name, cfg->theme + 1, N_THEMES,
             colony_returning(&s->colony), s->env.delivered,
             scene_speed(s), fps, cfg->sim_fps);
}

static void render_hud(const Scene *s, double fps)
{
    char status[200];
    hud_status_line(status, sizeof status, s, fps);

    const char *keys =
        " n/N:pattern  t/T:theme  +/-:speed  [/]:Hz  "
        "spc:pause  r:reset  q:quit ";

    hud_bar(0,           s->cols, PAIR_HUD,  status);  /* top: live data    */
    hud_bar(s->rows - 1, s->cols, PAIR_HINT, keys);    /* bottom: actions   */
}

/* ===================================================================== */
/* §9  app — orchestration                                                */
/*                                                                        */
/* The only place that sequences input → effects → delay → render.        */
/* ===================================================================== */

/*
 * App — the running PROCESS around the Scene: the simulated world plus
 * the loop's own bookkeeping.  Split from Scene because these fields
 * describe the program, not the simulation — they would not belong in a
 * saved/serialised world.  One file-scope instance (g_app) so the signal
 * handlers can reach the flags below without scattering loose globals.
 */
typedef struct {
    Scene scene;             /* the simulated world (§4-§7)                 */
    float tick_accum;        /* carry between frames for fractional speed:
                              * += speed each frame, spend whole steps,
                              * keep the remainder.  At 0.25x it crosses
                              * 1.0 once every four frames ⇒ slow motion.   */
    volatile sig_atomic_t running;     /* main-loop flag, 0 = exit.  volatile
                                        * sig_atomic_t: written from the
                                        * SIGINT/SIGTERM handlers, so it must
                                        * be async-signal-safe to touch.     */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH, drained at the top
                                        * of the loop — defer the ncurses
                                        * rebuild OUT of the handler.         */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void screen_init(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
}

/* terminal resize: re-read size, then rebuild the scene layout. */
static void app_resize(App *app)
{
    int cols, rows;
    endwin();
    refresh();
    getmaxyx(stdscr, rows, cols);
    scene_resize(&app->scene, cols, rows);
    app->need_resize = 0;
}

/* walk the speed ladder / fps target one notch, clamped to range. */
static void settings_nudge_speed(Settings *c, int delta)
{
    c->speed_idx = imax(0, imin(N_SPEEDS - 1, c->speed_idx + delta));
}
static void settings_nudge_fps(Settings *c, int delta)
{
    c->sim_fps = imax(SIM_FPS_MIN, imin(SIM_FPS_MAX, c->sim_fps + delta));
}

/* map a keypress to an intent; some intents fire §7 effects.
 * returns false only on quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene    *s   = &app->scene;
    Settings *cfg = &s->cfg;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
    case 'p': case 'P': cfg->paused = !cfg->paused;     break;
    case 'r': case 'R': scene_reset(s);                 break;

    case 'n':           scene_cycle_pattern(s, +1);     break;
    case 'N':           scene_cycle_pattern(s, -1);     break;

    case 't':           scene_cycle_theme(s, +1);       break;
    case 'T':           scene_cycle_theme(s, -1);       break;

    case '=': case '+': settings_nudge_speed(cfg, +1);          break;
    case '-':           settings_nudge_speed(cfg, -1);          break;

    case ']':           settings_nudge_fps(cfg, +SIM_FPS_STEP); break;
    case '[':           settings_nudge_fps(cfg, -SIM_FPS_STEP); break;

    default: break;
    }
    return true;
}

/* EFFECT step: advance the world by the speed multiplier this frame.
 * The fractional accumulator is what makes sub-1x slow motion possible —
 * at 0.25x one scene_step runs only every fourth frame. */
static void app_advance(App *app)
{
    if (app->scene.cfg.paused) return;
    app->tick_accum += scene_speed(&app->scene);
    while (app->tick_accum >= 1.0f) {
        scene_step(&app->scene);
        app->tick_accum -= 1.0f;
    }
}

/* RENDER step: pure paint of the current state. */
static void app_draw(const App *app, double fps)
{
    erase();
    render_world(&app->scene);
    render_hud(&app->scene, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * FpsMeter — a rolling frame-rate average over FPS_UPDATE_MS, so the HUD
 * number is readable rather than flickering every frame.  Banks elapsed
 * time + frame count, then divides once per window.
 */
typedef struct {
    int64_t accum_ns;   /* time banked since the last readout   */
    int     frames;     /* frames banked since the last readout */
    double  value;      /* last computed fps (shown in the HUD) */
} FpsMeter;

static void fps_meter_tick(FpsMeter *m, int64_t dt)
{
    m->frames++;
    m->accum_ns += dt;
    if (m->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        m->value    = (double)m->frames / ((double)m->accum_ns / (double)NS_PER_SEC);
        m->frames   = 0;
        m->accum_ns = 0;
    }
}

/* DELAY step: sleep so the frame lands on the cfg.sim_fps cadence. */
static void app_pace_frame(const App *app, int64_t frame_start, int64_t dt)
{
    int64_t target  = TICK_NS(app->scene.cfg.sim_fps);
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(target - elapsed);
}

/* measure dt since the last frame, clamped so one stall can't fast-forward
 * the whole simulation (spiral-of-death guard). */
static int64_t frame_delta(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    return dt > DT_CAP_MS * NS_PER_MS ? DT_CAP_MS * NS_PER_MS : dt;
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

    screen_init();
    int cols, rows;
    getmaxyx(stdscr, rows, cols);
    scene_init(&app->scene, cols, rows);

    int64_t  frame_time = clock_ns();
    FpsMeter fps        = { 0, 0, 0.0 };

    /*
     * Fixed-step main loop — each iteration reads as five named steps:
     *   INPUT   poll a key (may fire §7 effects: reset, pattern, theme)
     *   TIME    frame_delta() — dt since last frame, clamped
     *   EFFECTS app_advance() — fractional-speed scene stepping
     *   DELAY   app_pace_frame() — hold the cfg.sim_fps cadence
     *   RENDER  app_draw() — pure paint, mutates nothing
     * Input is polled every iteration, so responsiveness is independent
     * of the sim speed multiplier.
     */
    while (app->running) {
        if (app->need_resize) {
            app_resize(app);
            frame_time = clock_ns();
        }

        int ch = getch();                                       /* INPUT   */
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;

        int64_t dt = frame_delta(&frame_time);                  /* TIME    */

        app_advance(app);                                       /* EFFECTS */
        fps_meter_tick(&fps, dt);
        app_pace_frame(app, frame_time, dt);                    /* DELAY   */
        app_draw(app, fps.value);                               /* RENDER  */
    }

    endwin();
    return 0;
}
