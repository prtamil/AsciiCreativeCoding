/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hex_life.c — Hexagonal Game of Life
 *
 * Game of Life on a hexagonal grid using offset-rows layout.
 * Each cell has 6 neighbours.  Rule B2/S34:
 *   Born    if dead  and has exactly 2 live neighbours.
 *   Survive if alive and has 3 or 4 live neighbours.
 *
 * Hex offset-rows: odd rows are shifted right by one terminal column.
 * For even row r, neighbours of (r,c): (r-1,c-1),(r-1,c),(r,c-1),(r,c+1),(r+1,c-1),(r+1,c)
 * For odd  row r, neighbours of (r,c): (r-1,c),(r-1,c+1),(r,c-1),(r,c+1),(r+1,c),(r+1,c+1)
 *
 * Keys: q quit  SPACE random fill  1-5 density  r seed cluster
 *       c clear  t theme  p pause  +/- sim speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/hex_life.c \
 *       -o hex_life -lncurses -lm
 *
 * §1 config  §2 state  §3 performance  §4 color  §5 logic  §6 simulation
 * §7 effects  §8 seed/reset  §9 render  §10 platform  §11 driver
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Conway's Game of Life adapted for a hexagonal grid.
 *                  Each cell has 6 neighbours (vs 8 for square grids).
 *                  Rule B2/S34: born on 2 live neighbours, survives on 3–4.
 *                  Uses double-buffering: write next generation to the
 *                  inactive buffer, then swap — O(rows × cols) per step.
 *
 * Data-structure : Flat 2-D array with uint8_t age field.  Offset-rows
 *                  layout: odd rows shifted right by 0.5 cells visually.
 *                  Neighbour coordinates differ for even vs odd rows (two
 *                  lookup tables, one per parity).
 *
 * Physics        : Hexagonal CA rules produce qualitatively different
 *                  behaviour than square-grid Life: the B2/S34 rule creates
 *                  stable oscillators and gliders unique to the hex topology.
 *                  6-neighbour symmetry eliminates diagonal artefacts present
 *                  in square-grid CAs (more isotropic diffusion of patterns).
 *
 * Rendering      : Each cell is a single dot whose SIZE/brightness tracks a
 *                  per-cell "glow" (how settled it is) and whose COLOUR tracks
 *                  age along an 8-stop heat ramp (themeable via 't').  Glow
 *                  eases toward the alive/dead target every frame, so births
 *                  swell in and deaths fade out instead of snapping.  Genera-
 *                  tions advance on a fixed-timestep accumulator decoupled
 *                  from a steady 60 fps render.
 *
 * References     : Concepts —
 *                  [1] Gardner, "Mathematical Games: ...Conway's new solitaire
 *                      game 'Life'," Scientific American 223(4), 1970 — the
 *                      original Game of Life; this file is Life on a hex grid.
 *                  [2] Berlekamp, Conway & Guy, "Winning Ways for Your Mathe-
 *                      matical Plays," Vol. 4 (AK Peters, 2004) — the theory of
 *                      Life: still lifes, oscillators, gliders.
 *                  [3] Bays, "A Note on the Game of Life in Hexagonal and
 *                      Pentagonal Tessellations," Complex Systems 15 (2005) —
 *                      THE hex-Life reference; Life rules (Bn/Sn) on hex tilings.
 *                  [4] Adamatzky (ed.), "Game of Life Cellular Automata"
 *                      (Springer, 2010) — modern survey incl. non-square
 *                      lattices and the Life-like rule space.
 *                  [5] LifeWiki — conwaylife.com/wiki — the B/S rule notation,
 *                      hexagonal Life, and the pattern catalogue.
 *                  Rendering / implementation —
 *                  [6] Patel, "Hexagonal Grids," Red Blob Games
 *                      (redblobgames.com/grids/hexagons) — definitive guide to
 *                      hex layouts; the §5 (hex_count) + §9 (scene_draw) shift.
 *                  [7] Fiedler, "Fix Your Timestep!"
 *                      (gafferongames.com/post/fix_your_timestep) — the §11
 *                      fixed-timestep accumulator decoupling sim Hz from fps.
 *                  [8] Ben-Halim, Raymond et al., "Writing Programs with
 *                      NCURSES" (NCURSES HOWTO) — colour pairs, erase/doupdate;
 *                      the §4 colour + §9 render model.
 *                  [9] Moreland, "Diverging Color Maps for Scientific Visuali-
 *                      zation," Proc. ISVC 2009 — perceptually-ordered colour
 *                      ramps; the basis for the §1 heat ramp + theme palettes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Same Conway loop, different topology.  Replace the 8 neighbours of a
 * square cell with the 6 neighbours of a hexagon, and replace B3/S23
 * with B2/S34.  The hex lattice is stored as offset rows: each cell
 * has the same array indices as a square grid, but rendered with odd
 * rows shifted right one column to fake the hex stagger.  Because two
 * of the eight square-grid diagonals are "missing" in hex, life-style
 * patterns are more isotropic — gliders and oscillators don't have a
 * preferred axis the way Conway's diagonal glider does.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a beehive.  Every cell touches exactly six others — three on
 * the row above, two beside, and three on the row below — except every
 * other row is bumped half a cell sideways.  That bump is what makes
 * even rows have neighbours at (-1,-1)/(-1,0) above, but odd rows have
 * them at (-1,0)/(-1,+1).  Two lookup tables indexed by row parity is
 * how the simulation sees the lattice; visually the half-cell shift is
 * faked by printing odd rows starting one terminal column to the right
 * of even rows.  Each cell owns a 2-column slot but draws a single dot in
 * its left column, so that 1-column odd-row offset puts a row's dots in
 * the gaps of its neighbours — the perceived hex stagger.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Init grid and age arrays to zero.  Seed live cells (random ~35%
 *     density, or a centre cluster).
 *  2. For each cell (r,c):
 *       a. Pick neighbour delta table by parity: HEX_DC_EVEN or
 *          HEX_DC_ODD; HEX_DR is shared.
 *       b. Count live neighbours among 6 offsets, treating off-grid
 *          as dead (no toroidal wrap here — finite hex board).
 *       c. Apply B2/S34: alive ∧ n ∈ {3,4} → survive; dead ∧ n=2 →
 *          birth; otherwise → dead.
 *       d. Track age: survivors increment age (cap 255); newborns
 *          age=0; dead cells reset age=0.
 *  3. Swap colony.grid ← colony.next, colony.age ← colony.anext via memcpy.
 *  4. Render: every live cell is a single dot — '.' speck while faint,
 *     round 'o' once settled (size from glow); colour from age (heat
 *     ramp).  Dying cells shrink and fade to the dim '.' lattice.
 *  5. HUD: data on the top 2 rows, action keys on the bottom row;
 *     sleep until 60 fps deadline.
 *
 * KEY FORMULAS
 * ────────────
 *  hex neighbour deltas (even row r):
 *      (-1,-1) (-1, 0)  ←  upper neighbours
 *      ( 0,-1) ( 0, 1)  ←  side neighbours
 *      ( 1,-1) ( 1, 0)  ←  lower neighbours
 *  hex neighbour deltas (odd row r):
 *      (-1, 0) (-1, 1)
 *      ( 0,-1) ( 0, 1)
 *      ( 1, 0) ( 1, 1)
 *  B2/S34 rule:
 *      next = alive ? (n==3 || n==4) : (n==2)
 *  screen mapping:
 *      sc = c·2 + (r & 1)              odd rows shift right 1 col
 *      sr = r + HUD_TOP_ROWS          2 data rows above, 1 action row below
 *  glyph: a single dot per cell — speck '.' → round 'o' (size set by glow,
 *         not age, so it swells/shrinks smoothly; one column, hex-spaced)
 *  age → heat colour (younger = hotter; Ember theme shown, 't' cycles 5):
 *      0      white-hot     11..16  cyan
 *      1..3   gold/amber     17..25  teal
 *      4..10  orange/ember   26+     deep blue (settled core)
 *  size/brightness ← glow (stability): speck (fading) · 'o' · bold 'o' (settled)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Parity bug: if you forget the `r & 1` parity check and use one
 *    delta table, the hex neighbourhood is wrong on every other row
 *    and patterns drift diagonally instead of radially.
 *  • Bounded board: hex_count() does NOT wrap.  Patterns that hit a
 *    wall lose neighbours and behave as if surrounded by dead cells —
 *    expect edge die-off, not toroidal continuation.
 *  • Age overflow: colony.age is uint8_t.  Code caps at 255 explicitly.
 *    Very long-lived oscillator cells stay in the elder bucket for
 *    thousands of generations; visually fine, but don't read age as
 *    "exact generation count" beyond 255.
 *  • Render footprint: each hex cell is 2 cols + 1 col shift on odd
 *    rows, so the effective grid width is `(cols - 2) / 2`.  Resize
 *    must clamp colony.gw to that, else the right edge wraps mid-cell.
 *  • Decoupled timing: the renderer runs at 60 fps (RENDER_NS) while
 *    generations advance on a fixed-timestep accumulator at scene.speed
 *    per second (GEN_HZ_*).  Between generations the glow layer fades
 *    births in and deaths out, so the discrete CA reads as smooth
 *    motion instead of snapping.  '+'/'-' change scene.speed, not the fps.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Density 1 (sparse, '1' key) — most random seeds die out within
 *    ~50 generations, leaving a few small static survivors.
 *  • Density 5 (dense, '5' key) — large random regions tend to the
 *    "soup" attractor: a churning steady-state population around 30%.
 *  • Colour-age check: seed once, watch a stable cell's `o` dot cool in
 *    colour with age — white-hot → gold → orange → cyan → teal → deep
 *    blue.  If the colour never shifts, the age update is broken.
 *  • Hex isotropy: random-soup B2/S34 patterns should NOT show a
 *    preferred axis.  If you see horizontal stripes forming, you've
 *    accidentally treated even/odd rows the same.
 *  • Generation counter advances scene.speed times per second (not per
 *    frame); live cell counter recomputes every frame — both in the HUD.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Layers, the section that owns each, and the state each MUTATES.  All mutable
 * data hangs off one Scene (g_scene): the Colony, its GlowField, and the
 * speed/paused/theme fields; workers take the narrowest sub-type they touch.
 *
 *   Layer        §   Mutates
 *   ──────────── ─── ─────────────────────────────────────────────────────
 *   CONFIG       §1  nothing (compile-time constants, CP ids, theme table)
 *   STATE        §2  the type defs (Colony, GlowField, Scene) + g_scene
 *   PERFORMANCE  §3  nothing (clock primitives; the policy lives in §11)
 *   COLOR        §4  ncurses colour-pair table (I/O); reads scene.theme + g_has_256
 *   LOGIC        §5  nothing — pure: age→pair (heat_pair), glow→glyph
 *   SIMULATION   §6  colony.{grid,next,age,anext,gen}  (the per-tick advance;
 *                    hex_count is its own pure, no-mutation neighbour test)
 *   EFFECTS      §7  glow.{glow,fade}                    (cosmetic only)
 *   SEED/RESET   §8  colony.{grid,age,gen} + glow.{glow,fade}  (user-triggered,
 *                    NOT a tick: colony_random/clear/seed re-init the board)
 *   RENDER       §9  the screen (draw_colony/draw_hud); reads only — count_live
 *                    RETURNS the population, it does not store it
 *   PLATFORM     §10 g_quit,g_resize                     (signal flags)
 *   DRIVER       §11 g_rows,g_cols,g_live,g_fps, colony.{gh,gw},
 *                    scene.{speed,paused,theme}, accum  (setup, input, resize, combine)
 *
 * Per-tick combine (driven by advance_sim in §11: one generation per
 * fixed-timestep slot of banked time, only when not paused):
 *       colony_step(&s->colony)  advance the CA one generation     (SIMULATION)
 * Per-frame (every rendered frame, in §11):
 *       g_live = count_live(...)  recompute the HUD population       (driver stat)
 *       glow_step(&colony,&glow) ease each cell's glow/fade          (EFFECTS)
 *       erase(); scene_draw(s)   state → screen                      (RENDER)
 *       clock_sleep_ns(...)      hold to the 60 fps cap              (PERFORMANCE)
 *
 * SIMULATION advances state ONLY through colony_step at the combine point;
 * user events (key, random/clear/seed, resize) mutate state but are NOT the
 * tick.  LOGIC (§5) does no mutation and no I/O, so reordering or deleting
 * RENDER/EFFECTS can never change its results.
 *
 * No DELAYS layer: the only wait is the per-frame 60 fps cap sleep, which is a
 * PERFORMANCE throttle — not an animation hold/timer; 'p' pause is a control
 * flag read in the driver, not a timed delay.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  CONFIG — constants, colour-pair ids, theme palette table       */
/* ===================================================================== */

#define GRID_W_MAX  200
#define GRID_H_MAX   60
#define HUD_TOP_ROWS  2   /* rows 0-1 (top): live stats + age legend   */
#define HUD_BOT_ROWS  1   /* last row (bottom): action key hints       */
#define HUD_ROWS      (HUD_TOP_ROWS + HUD_BOT_ROWS)   /* total reserved */

#define RENDER_NS    (1000000000LL / 60)   /* 60 fps render — smooth screen     */
#define GEN_HZ_DEF    8     /* generations per second (sim speed), default      */
#define GEN_HZ_MIN    2
#define GEN_HZ_MAX   30
#define DT_CAP_NS    (100 * 1000000LL)      /* clamp a frame's dt → no spiral    */
#define GLOW_RATE    0.28f /* per-frame fade toward target (birth in/death out) */
#define GLOW_GONE    0.10f /* glow below this = fully faded → cold lattice dot   */
#define NS_PER_SEC   1000000000.0  /* nanoseconds in one second (timing maths)   */
#define FPS_SMOOTH   0.1f          /* EMA weight smoothing the displayed fps      */
#define DENSITY_DEFAULT  35        /* default random-fill density, % (boot/reseed)*/

/* Colour-pair ids (ncurses init_pair slots, 1-based).  CP_HEAT0..7 are the heat
 * ramp: cell age → temperature in 8 tiers, newborn (hottest) → settled-old
 * (coolest).  The slots are fixed; their actual colours are filled from the
 * active theme (k_themes) by theme_apply(), so 't' recolours the whole field
 * without touching any draw code.  CP_HEAT0..7 MUST stay consecutive —
 * theme_apply indexes them as CP_HEAT0 + i, and glow.fade stores one of these
 * ids per cell.
 *
 * WHY the age→tier boundaries (defined in heat_pair, §5) are NON-uniform — narrow
 * early (0, 1, 2-3, 4-6), wide later (7-10, 11-16, 17-25, 26+): a cell should
 * visibly "cool" in its first few generations, then settle to a steady colour
 * once it is part of a stable structure.  Equal-width buckets would either
 * change too slowly when young or keep flickering when old. */
enum {
    CP_DEAD = 1,   /* dead     — dim dot, outlines the hex lattice */
    CP_HEAT0,      /* age 0      newborn (hottest)   */
    CP_HEAT1,      /* age 1                          */
    CP_HEAT2,      /* age 2-3                        */
    CP_HEAT3,      /* age 4-6                        */
    CP_HEAT4,      /* age 7-10                       */
    CP_HEAT5,      /* age 11-16                      */
    CP_HEAT6,      /* age 17-25                      */
    CP_HEAT7,      /* age 26+    settled (coolest)   */
    CP_HUD,        /* HUD data    — bright yellow on default bg */
    CP_HINT,       /* HUD actions — bright cyan   on default bg */
};

/* Theme — one named colour gradient for the age ramp: 8 stops running hot
 * (newborn) → cool (old), one stop per heat tier CP_HEAT0..7.  Encoding a scalar
 * (here a cell's age) as a perceptually-ordered colour ramp is the standard
 * scientific-visualisation technique — see Moreland, "Diverging Color Maps for
 * Scientific Visualization," Proc. ISVC 2009.  Pressing 't' swaps the entire
 * palette by re-filling those 8 ncurses pairs (theme_apply), so the field
 * recolours instantly with no change to any draw code.
 *
 * VALUE LOGIC: every 256-colour stop sits in the BRIGHT half of the cube
 * (≥ 24 / ≥ 240) per the project brightness rule (COLOR.md) so even the dimmest
 * tier stays legible under A_DIM — grayscale 232-239 and cube indices < 24
 * render near-black and are banned.  ramp8 is the parallel 8-colour fallback,
 * chosen by g_has_256 when the terminal lacks 256-colour support. */
typedef struct {
    short ramp[8];     /* 256-colour stops, index 0 = newborn (hot) .. 7 = oldest (cool) */
    short ramp8[8];    /* 8-colour fallback, same newborn → old order                    */
    const char *name;  /* shown in the HUD                                               */
} Theme;

#define N_THEMES 5
static const Theme k_themes[N_THEMES] = {
    /* 0  Ember   — white-hot young cooling to deep blue (the classic) */
    { { 231, 226, 220, 214, 208,  51,  38,  26 },
      { COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED,
        COLOR_CYAN,  COLOR_CYAN,   COLOR_BLUE }, "Ember" },
    /* 1  Inferno — pure fire: white-hot down to dark red */
    { { 231, 228, 220, 214, 208, 202, 196, 124 },
      { COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED,
        COLOR_RED,   COLOR_RED,    COLOR_RED },  "Inferno" },
    /* 2  Ocean   — white spray through cyan to deep blue */
    { { 231, 195, 159,  87,  51,  39,  33,  26 },
      { COLOR_WHITE, COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN, COLOR_CYAN,
        COLOR_BLUE,  COLOR_BLUE,   COLOR_BLUE }, "Ocean" },
    /* 3  Forest  — pale shoots cooling to deep evergreen */
    { { 231, 230, 191, 154, 118,  78,  35,  28 },
      { COLOR_WHITE, COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN,  COLOR_GREEN }, "Forest" },
    /* 4  Plasma  — white through magenta and violet to blue */
    { { 231, 219, 213, 207, 171, 135,  99,  57 },
      { COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_BLUE,  COLOR_BLUE,    COLOR_BLUE }, "Plasma" },
};

/* ===================================================================== */
/* §2  STATE — the domain types and the one Scene that aggregates them    */
/* ===================================================================== */

/* Colony — the live state of the cellular automaton: one generation of cells on
 * the hexagonal lattice, plus everything needed to advance it.  This is hex-Life
 * — Conway's Game of Life retiled onto a 6-neighbour hex grid under rule B2/S34
 * (a dead cell is BORN on exactly 2 live neighbours; a live cell SURVIVES on 3
 * or 4).  Refs: Gardner, Sci. Am. 223(4) 1970 (the original Life); Bays, Complex
 * Systems 15 (2005) (Life on hexagonal/pentagonal tilings — the source of hex
 * Bn/Sn rules); Berlekamp-Conway-Guy, Winning Ways Vol. 4 (Life theory).
 *
 * WHY DOUBLE-BUFFERED (grid/next, age/anext): a CA step must compute the WHOLE
 * next generation from a FROZEN snapshot of the current one — every cell counts
 * its neighbours as they are NOW.  Writing in place would let a cell see
 * neighbours already advanced this tick and corrupt the rule.  So colony_step
 * writes next/anext, then memcpy-swaps grid←next, age←anext.
 *
 * WHY age (not a textbook-Life field): age = generations a cell has been
 * CONTINUOUSLY alive (survivors increment, newborns reset to 0).  It feeds the
 * heat colour ramp (young = hot, old = cool); it plays no part in the B2/S34
 * decision.  Advanced by SIMULATION (§6); re-initialised by SEED/RESET (§8). */
typedef struct {
    signed char   grid [GRID_H_MAX][GRID_W_MAX];  /* current generation — 0 dead / 1 alive (the authoritative state) */
    signed char   next [GRID_H_MAX][GRID_W_MAX];  /* scratch colony_step fills, then swaps into grid (double-buffer)  */
    unsigned char age  [GRID_H_MAX][GRID_W_MAX];  /* per-cell generations alive, 0..255 — input to the heat ramp      */
    unsigned char anext[GRID_H_MAX][GRID_W_MAX];  /* next-gen age scratch, parallel to next                           */
    int           gh, gw;                          /* LIVE size in cells (≤ _MAX) fit to the terminal; only [0,gh)×[0,gw) is simulated/drawn */
    long long     gen;                             /* generations since the last reset — the colony's clock (HUD); long long so it never overflows */
} Colony;

/* GlowField — the render-smoothing layer that shadows the Colony one entry per
 * cell.  A discrete CA "blinks": cells snap fully on/off every generation, which
 * the eye reads as flicker.  The fix is to DECOUPLE the look from the step — each
 * cell carries a brightness that EASES toward its target rather than jumping
 * (an exponential moving average / one-pole low-pass filter, the rendering
 * sibling of the sim-vs-render decoupling in Fiedler, "Fix Your Timestep!").  A
 * birth then swells in over several frames and a death fades out.
 *
 * WHY brightness tracks STABILITY, not age: glow≈1 only for cells that have been
 * steady a while, so the churning growth front stays dim/quiet and stable
 * structures read as bright — the opposite of flashing the most-changing cells.
 * Purely cosmetic — never read by the B2/S34 rule (EFFECTS, §7). */
typedef struct {
    float         glow[GRID_H_MAX][GRID_W_MAX];   /* brightness 0..1, eased toward alive=1/dead=0 by GLOW_RATE/frame; glow_glyph maps it to dot size+intensity */
    unsigned char fade[GRID_H_MAX][GRID_W_MAX];   /* the cell's last heat-pair id (CP_HEAT*) while alive, so a dying cell fades out in its OWN colour; a pair id (not a raw colour) so theme swaps recolour it for free */
} GlowField;

/* Scene — the whole program in one place, as a table of contents.  It exists so
 * the data has ONE home while the code stays layered: orchestrators that drive a
 * whole frame take the Scene, but every worker below takes only the narrowest
 * sub-type (Colony / GlowField) it actually touches — so "everything hangs off
 * Scene" never re-couples SIMULATION, EFFECTS and RENDER.  Grouped by concept:
 *   colony  WHAT   — the hex-Life colony that evolves
 *   glow    (its cosmetic glow/fade layer, drawn over the colony)
 *   speed   HOW    — the one tunable sim knob (the rule itself is fixed)
 *   paused  WHERE  — run-state
 *   theme   RENDER — which palette is active; a render concept, deliberately NOT
 *                    bundled with the sim knob even though both ride the keyboard */
typedef struct {
    Colony    colony;   /* the simulation (WHAT is computed)                        */
    GlowField glow;     /* per-cell render smoothing over `colony`                  */
    int       speed;    /* generations per second, clamped GEN_HZ_MIN..GEN_HZ_MAX   */
    bool      paused;   /* freeze the sim (render + glow ease keep running)         */
    int       theme;    /* active palette: index into k_themes                      */
} Scene;

/* The single program instance.  The big per-cell arrays live INSIDE it, so this
 * is one ~110 KB static object in BSS — no heap, matching the project's
 * no-allocation-after-init rule.  speed is the only non-zero default. */
static Scene g_scene = { .speed = GEN_HZ_DEF };

/* View, derived HUD stats, and platform flags — incidental to the simulation,
 * so they stay as module globals rather than cluttering the Scene. */
static int       g_rows, g_cols;                /* terminal size, in characters    */
static long long g_live = 0;                    /* live-cell count, recomputed/frame */
static double    g_fps  = 60.0;                 /* smoothed render frame rate        */
static bool      g_has_256 = false;             /* 256-colour terminal? (set in init)*/
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ===================================================================== */
/* §3  PERFORMANCE — monotonic clock + frame-cap sleep (policy in §11) */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §4  COLOR — palette setup; the only colour-pair I/O                */
/* ===================================================================== */

/* theme_apply — fill the 8 heat-ramp pairs (CP_HEAT0..7) from theme ti.  Pair
 * indices are reused, so re-applying with a new theme instantly recolours every
 * cell; glow.fade stores pair indices, not raw colours, so it needs no rebuild.
 * CP_DEAD/CP_HUD/CP_HINT are theme-independent and set once in color_init. */
static void theme_apply(int ti)
{
    const Theme *th = &k_themes[ti];
    for (int i = 0; i < 8; i++)
        init_pair(CP_HEAT0 + i, g_has_256 ? th->ramp[i] : th->ramp8[i], -1);
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_has_256 = (COLORS >= 256);

    /* theme-independent pairs — set once, never touched by theme_apply */
    if (g_has_256) {
        init_pair(CP_DEAD, 242, -1);   /* dim lattice dot         */
        init_pair(CP_HUD,  226, -1);   /* bright yellow — data    */
        init_pair(CP_HINT,  51, -1);   /* bright cyan   — actions */
    } else {
        init_pair(CP_DEAD, COLOR_BLACK,  -1);
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(g_scene.theme);   /* CP_HEAT0..7 from the active theme */
}

/* ===================================================================== */
/* §5  LOGIC — pure decisions: no mutation, no I/O (age→pair, glow→glyph) */
/* ===================================================================== */

/* heat_pair — a live cell's age → its colour on the heat ramp.  Young cells
 * burn hot (white→gold→orange); old cells cool (cyan→teal→deep blue).  Colour
 * is the ONLY thing age controls — glyph and weight are uniform, so a cell
 * never changes shape as it ages (that shape-popping was the blinking). */
static int heat_pair(unsigned char age)
{
    if      (age == 0)  return CP_HEAT0;   /* white-hot */
    else if (age <= 1)  return CP_HEAT1;   /* gold      */
    else if (age <= 3)  return CP_HEAT2;   /* amber     */
    else if (age <= 6)  return CP_HEAT3;   /* orange    */
    else if (age <= 10) return CP_HEAT4;   /* ember     */
    else if (age <= 16) return CP_HEAT5;   /* cyan      */
    else if (age <= 25) return CP_HEAT6;   /* teal      */
    else                return CP_HEAT7;   /* deep blue */
}

/* glow_glyph — one cell's dot from its glow (stability).  A cell mid-transition
 * (just born or dying) is a faint speck '.'; once settled it swells to a round
 * 'o', bold at full glow.  Because size tracks STABILITY, churning births and
 * deaths stay tiny and quiet while stable structures read as bright dots —
 * instead of flashing the most-rapidly-changing cells.  Glow moves smoothly
 * frame to frame, so the dot grows and shrinks instead of snapping. */
static void glow_glyph(float gl, char *ch, attr_t *at)
{
    if      (gl >= 0.55f) { *ch = 'o'; *at = A_BOLD;   }  /* settled — bright */
    else if (gl >= 0.25f) { *ch = 'o'; *at = A_NORMAL; }  /* mid              */
    else                  { *ch = '.'; *at = A_DIM;    }  /* faint speck      */
}

/* ===================================================================== */
/* §6  SIMULATION — the per-tick advance (colony_step) + its pure neighbour test */
/* ===================================================================== */

/* B2/S34 hex neighbour offsets (even row, odd row) */
static const int HEX_DR[6]    = { -1, -1,  0, 0,  1,  1 };
static const int HEX_DC_EVEN[6] = { -1,  0, -1, 1, -1,  0 };
static const int HEX_DC_ODD[6]  = {  0,  1, -1, 1,  0,  1 };

/* hex_count — live neighbours of cell (r,c): a pure read of the colony. */
static int hex_count(const Colony *col, int r, int c)
{
    const int *dc = (r & 1) ? HEX_DC_ODD : HEX_DC_EVEN;
    int cnt = 0;
    for (int k = 0; k < 6; k++) {
        int nr = r + HEX_DR[k];
        int nc = c + dc[k];
        if (nr >= 0 && nr < col->gh && nc >= 0 && nc < col->gw)
            cnt += (col->grid[nr][nc] != 0) ? 1 : 0;
    }
    return cnt;
}

/* colony_step — advance the colony one generation (the B2/S34 tick): write the
 * next buffers, bump ages, then swap.  The only mutator of the simulation. */
static void colony_step(Colony *col)
{
    for (int r = 0; r < col->gh; r++) {
        for (int c = 0; c < col->gw; c++) {
            int n = hex_count(col, r, c);
            int alive = (col->grid[r][c] != 0);
            /* B2/S34: survive on 3 or 4 neighbours, born on exactly 2, else die */
            if (alive && (n == 3 || n == 4)) {
                col->next[r][c]  = 1;
                /* survivor ages one generation, saturating at 255 */
                col->anext[r][c] = col->age[r][c] < 255 ? col->age[r][c] + 1 : 255;
            } else if (!alive && n == 2) {
                col->next[r][c]  = 1;
                col->anext[r][c] = 0;   /* newborn */
            } else {
                col->next[r][c]  = 0;
                col->anext[r][c] = 0;
            }
        }
    }
    /* commit the new generation: swap next → current, advance the clock */
    col->gen++;
    memcpy(col->grid, col->next,  sizeof col->grid);
    memcpy(col->age,  col->anext, sizeof col->age);
}

/* ===================================================================== */
/* §7  EFFECTS — glow/fade smoothing state advance (cosmetic only)    */
/* ===================================================================== */

/* glow_sync — snap every cell's glow to its current alive state (instant, no
 * fade).  Called after a reseed/clear so the new board appears immediately. */
static void glow_sync(const Colony *col, GlowField *gf)
{
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++)
            gf->glow[r][c] = col->grid[r][c] ? 1.0f : 0.0f;
}

/* ease_toward — one exponential step of `cur` toward `target` (a one-pole
 * low-pass / EMA): moves a fraction GLOW_RATE of the remaining gap each call. */
static float ease_toward(float cur, float target)
{
    return cur + (target - cur) * GLOW_RATE;
}

/* glow_step — advance every cell's brightness one render frame toward its
 * target (alive→1, dead→0).  Also stash each living cell's heat colour in
 * gf->fade so a later death fades out in its own colour.  Runs once per
 * rendered frame, independent of how often a generation ticks. */
static void glow_step(const Colony *col, GlowField *gf)
{
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++) {
            if (col->grid[r][c]) {
                gf->glow[r][c] = ease_toward(gf->glow[r][c], 1.0f);
                gf->fade[r][c] = (unsigned char)heat_pair(col->age[r][c]);
            } else {
                gf->glow[r][c] = ease_toward(gf->glow[r][c], 0.0f);
            }
        }
}

/* ===================================================================== */
/* §8  SEED / RESET — user-triggered board init (NOT part of the tick) */
/* ===================================================================== */

/* Each reset writes the colony then snaps its glow layer to match, so the new
 * board appears at once instead of fading in.  Both layers are touched, so each
 * takes the two narrow types — never the whole Scene. */

static void colony_random(Colony *col, GlowField *gf, int density_pct)
{
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++)
            col->grid[r][c] = (rand() % 100 < density_pct) ? 1 : 0;
    memset(col->age, 0, sizeof col->age);
    col->gen = 0;
    glow_sync(col, gf);
}

static void colony_clear(Colony *col, GlowField *gf)
{
    memset(col->grid, 0, sizeof col->grid);
    memset(col->age,  0, sizeof col->age);
    col->gen = 0;
    glow_sync(col, gf);
}

/* place a small seed pattern at center */
static void colony_seed(Colony *col, GlowField *gf)
{
    colony_clear(col, gf);
    int cr = col->gh / 2, cc = col->gw / 2;
    /* small dense cluster */
    for (int dr = -3; dr <= 3; dr++)
        for (int dc = -3; dc <= 3; dc++) {
            int r = cr+dr, c = cc+dc;
            if (r>=0&&r<col->gh&&c>=0&&c<col->gw)
                col->grid[r][c] = rand()%2;
        }
    glow_sync(col, gf);   /* colony_clear synced to 0; re-sync after seeding cells */
}

/* ===================================================================== */
/* §9  RENDER — state → screen; pure reads (count_live returns its count)   */
/* ===================================================================== */

/* count_live — population of the colony, for the HUD.  Pure read, returns the
 * count (the driver stores it in g_live each frame). */
static long long count_live(const Colony *col)
{
    long long live = 0;
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++)
            if (col->grid[r][c]) live++;
    return live;
}

/* Draw one HUD line: coloured text from col 1, clipped to the last column so a
 * long line never wraps onto the grid.  Default background (no bar) to match the
 * project HUD standard — bright yellow data, bright cyan hints. */
static void hud_line(int row, int pair, attr_t attr, const char *s)
{
    attron(COLOR_PAIR(pair) | attr);
    mvaddnstr(row, 1, s, g_cols - 1);
    attroff(COLOR_PAIR(pair) | attr);
}

/* draw_colony — paint the hex lattice of dots.  Each hex cell owns a 2-column
 * slot (the 1-column odd-row shift gives the hexagonal stagger) but draws a
 * SINGLE dot in its left column, so cells read as small separated points.  The
 * dot's SIZE/brightness comes from glow (speck '.' → round 'o' → bold 'o'), its
 * COLOUR from age (the cell's heat-pair); a dead cell with no glow left is the
 * dim '.' lattice underneath. */
static void draw_colony(const Scene *s)
{
    const Colony    *col = &s->colony;
    const GlowField *gf  = &s->glow;

    for (int r = 0; r < col->gh && r + HUD_TOP_ROWS < g_rows - HUD_BOT_ROWS; r++) {
        int offset = r & 1;   /* odd rows shift right 1 col */
        for (int c = 0; c < col->gw; c++) {
            int sc = c * 2 + offset;
            int sr = r + HUD_TOP_ROWS;
            if (sc >= g_cols) break;

            float  gl = gf->glow[r][c];
            int    cp;
            attr_t at;
            char   ch;

            if (col->grid[r][c] || gl > GLOW_GONE) {
                /* live, or a corpse still fading: a single dot that swells with
                 * glow ('.'→'o'), coloured by the cell's heat (live) or its
                 * last-living heat (dying), so transitions never snap */
                cp = gf->fade[r][c];
                glow_glyph(gl, &ch, &at);
            } else {
                /* fully dead: dim grey dot — the hex lattice underneath */
                cp = CP_DEAD; ch = '.'; at = A_DIM;
            }
            attron(COLOR_PAIR(cp) | at);
            mvaddch(sr, sc, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp) | at);
        }
    }
}

/* draw_hud — the two top data rows (live stats + colour legend) and the bottom
 * action-key row, per the project HUD standard (yellow data, cyan hints). */
static void draw_hud(const Scene *s)
{
    char buf[160];
    snprintf(buf, sizeof buf,
        "HexLife B2/S34  %.1f fps  gen:%lld  live:%lld  gen/s:%d  [%s]  %s",
        g_fps, s->colony.gen, g_live, s->speed,
        k_themes[s->theme].name, s->paused ? "PAUSED" : "running");
    hud_line(0, CP_HUD, A_BOLD, buf);   /* primary data — bold dominant */
    hud_line(1, CP_HUD, 0,              /* legend — same yellow, no bold */
        "o=live dot (colour=age: white-hot new .. deep-blue old)   .=dead");
    hud_line(g_rows - 1, CP_HINT, A_BOLD,
        "q:quit  spc:random  1-5:density  r:seed  c:clear  t:theme  p:pause  +/-:speed");
}

/* scene_draw — one rendered frame: the colony, then the HUD over it. */
static void scene_draw(const Scene *s)
{
    draw_colony(s);
    draw_hud(s);
}


/* ===================================================================== */
/* §10 PLATFORM — signals, RNG seed, terminal setup/cleanup           */
/* ===================================================================== */

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void seed_rng(void) { srand((unsigned)time(NULL)); }

static void install_signals(void)
{
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);
}

/* screen_init — put the terminal into raw, non-blocking, cursor-off mode for
 * full-screen animation; typeahead(-1) stops input from interrupting the draw. */
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
}

/* ===================================================================== */
/* §11 DRIVER — setup, input, resize, and the per-tick combine point  */
/* ===================================================================== */

/* fit_to_terminal — size the live grid to the current terminal: record the
 * char dimensions, then derive how many cells fit.  Cells are 2 columns wide
 * with a 1-column margin → (cols-2)/2 across; rows lose the HUD bands. */
static void fit_to_terminal(Colony *col)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    col->gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
    col->gw = ((cols - 2) / 2) < GRID_W_MAX ? ((cols - 2) / 2) : GRID_W_MAX;
}

/* handle_resize — re-read the new terminal size and reseed to fill it. */
static void handle_resize(Scene *s)
{
    endwin(); refresh();
    fit_to_terminal(&s->colony);
    colony_random(&s->colony, &s->glow, DENSITY_DEFAULT);
}

/* handle_key — apply one keypress: quit, reset/seed, cycle theme, or nudge the
 * sim speed.  Mutates the scene (and the quit flag); not part of the tick. */
static void handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1; break;
    case ' ': colony_random(&s->colony, &s->glow, DENSITY_DEFAULT); break;
    case 'p': case 'P': s->paused = !s->paused; break;
    case 'c': case 'C': colony_clear(&s->colony, &s->glow); break;
    case 'r': case 'R': colony_seed(&s->colony, &s->glow); break;
    case 't': s->theme = (s->theme + 1) % N_THEMES; theme_apply(s->theme); break;
    case 'T': s->theme = (s->theme + N_THEMES - 1) % N_THEMES; theme_apply(s->theme); break;
    case '1': colony_random(&s->colony, &s->glow, 20); break;
    case '2': colony_random(&s->colony, &s->glow, DENSITY_DEFAULT); break;
    case '3': colony_random(&s->colony, &s->glow, 50); break;
    case '4': colony_random(&s->colony, &s->glow, 65); break;
    case '5': colony_random(&s->colony, &s->glow, 80); break;
    case '+': case '=': if (s->speed < GEN_HZ_MAX) s->speed++; break;
    case '-':           if (s->speed > GEN_HZ_MIN) s->speed--; break;
    default: break;
    }
}

/* advance_sim — catch the simulation up to real time on a FIXED timestep,
 * decoupled from the render rate: bank elapsed time and run one generation per
 * 1/speed second of it, so the screen stays smooth at any sim speed. */
static void advance_sim(Scene *s, double *accum, double dt)
{
    if (s->paused) return;
    double ns_per_gen = NS_PER_SEC / (double)s->speed;
    *accum += dt;
    while (*accum >= ns_per_gen) { colony_step(&s->colony); *accum -= ns_per_gen; }
}

int main(void)
{
    Scene *s = &g_scene;

    /* ── setup ── */
    seed_rng();
    atexit(cleanup);
    install_signals();
    screen_init();
    color_init();
    fit_to_terminal(&s->colony);
    colony_random(&s->colony, &s->glow, DENSITY_DEFAULT);

    long long prev  = clock_ns();
    double    accum = 0.0;   /* nanoseconds banked toward the next generation */

    while (!g_quit) {
        if (g_resize) { g_resize = 0; handle_resize(s); }
        handle_key(s, getch());

        /* ── frame timing: measure dt, smooth the fps readout, cap the catch-up ── */
        long long now = clock_ns();
        double    dt  = (double)(now - prev);
        prev = now;
        if (dt > 0) g_fps += (NS_PER_SEC / dt - g_fps) * FPS_SMOOTH;
        if (dt > (double)DT_CAP_NS) dt = (double)DT_CAP_NS;

        advance_sim(s, &accum, dt);

        /* ── render one frame ── */
        g_live = count_live(&s->colony);
        glow_step(&s->colony, &s->glow);   /* ease cells toward the current generation */
        erase();
        scene_draw(s);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));   /* hold to 60 fps */
    }
    return 0;
}
