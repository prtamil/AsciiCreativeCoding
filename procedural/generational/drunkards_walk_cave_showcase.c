/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * drunkards_walk_cave_showcase.c
 *   — Drunkard's Walk cave generator, animated.
 *
 * DEMO: Watch a few "drunkards" stagger around a solid wall map,
 *       carving floors as they go. Each walker is a glowing '@' that
 *       picks a random cardinal direction every step, carving '.' into
 *       the wall behind it. Multiple walkers + the bias of random
 *       motion produces winding, naturally-shaped caves — the look
 *       roguelikes use for caverns and dungeons. After the carved
 *       fraction reaches the target, HOLD on the cave, then repeat
 *       forever with a fresh layout.
 *
 * Study alongside: ./bsp_dungeon_showcase.c — the geometric opposite.
 *       BSP gives you sharp rectangular rooms and L-corridors;
 *       Drunkard's Walk gives you blobby, organic caverns. Two answers
 *       to the same "make a dungeon" question.
 *
 * Section map (cut by concern — see ARCHITECTURE block):
 *   §1 config      — map/walker/fps constants, tile + direction enums, themes
 *   §2 perf/delays — monotonic clock + sleep primitives
 *   §3 state       — Walker, Cave, Scene, Screen types + globals
 *   §4 logic       — pure: index, bounds, direction delta, neighbour test
 *   §5 simulation  — walkers carve the cave; scene_tick = combine point
 *   §6 effects     — glow trails (interwoven; see note)
 *   §7 render      — ASCII map + HUD; themes
 *   §8 platform    — ncurses setup, resize, signals
 *   §9 app         — input, fixed-timestep loop, main
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (preserves theme)
 *   t / T      next / previous theme
 *   + / =      faster walkers (more steps/tick)
 *   -          slower walkers
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra drunkards_walk_cave_showcase.c \
 *       -o drunkards_cave -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Drunkard's Walk cave generation. Start with a fully
 *                  walled map. Place N "drunkards" (random walkers) at
 *                  fixed start positions. Each tick, every walker:
 *                    1. Picks a uniformly random cardinal direction.
 *                    2. Steps one cell in that direction (if in bounds).
 *                    3. Carves the cell at its new position to FLOOR.
 *                  Continue until the carved-cell fraction reaches a
 *                  target (default 45%). Result: a naturally winding
 *                  cave system with no straight lines, no rooms — the
 *                  organic look that scripted geometry can't produce.
 *
 *                  Why multiple walkers? A single drunkard makes one
 *                  long meandering tunnel. Multiple walkers (started
 *                  near each other) carve overlapping territories that
 *                  fuse into a connected cave system. Walker count
 *                  controls cave density / branching.
 *
 *                  Walkers also "respawn" after MAX_AGE steps —
 *                  teleport to a random already-floor cell — to break
 *                  up long single-corridor patterns.
 *
 * Data-structure : Tile grid (TILE_WALL or TILE_FLOOR), per-cell glow
 *                  floats (carve_glow + walker_glow),
 *                  fixed-size Walker[] array, floor_count for
 *                  termination check. No allocation post-init.
 *
 * Rendering      : ASCII only. '#' for walls — drawn ONLY where adjacent
 *                  to a floor cell, so unreached interior void stays
 *                  blank (classic roguelike look). '.' for floor.
 *                  '@' for walkers, in the theme's walker colour. Per-
 *                  walker comet trail via walker_glow that decays over
 *                  ~0.4 s — last ~3-5 cells visited each glow brightly.
 *                  Per-cell carve_glow flashes the moment a fresh wall
 *                  becomes a floor.
 *
 * Performance    : O(N · steps_to_target) = O(N · cells^1.something) per
 *                  full run. Most steps are wasted (walker re-visits a
 *                  floor cell), but the constant is tiny. We throttle
 *                  via steps_per_tick so the spectacle plays out over
 *                  ~6-10 s. No allocation post-init.
 *
 * References     : Concepts —
 *                  • RogueBasin, "Random Walk Cave Generation" — the canonical
 *                    drunkard's-walk recipe this file implements:
 *                    http://www.roguebasin.com/index.php?title=Random_Walk_Cave_Generation
 *                  • Shaker, Togelius & Nelson, "Procedural Content Generation in
 *                    Games" (Springer, 2016; free at pcgbook.com) — the field's
 *                    textbook; space / dungeon generation chapters.
 *                  • Short & Adams (eds.), "Procedural Generation in Game Design"
 *                    (CRC Press, 2017) — agent-based random-walk carving in practice.
 *                  • Spitzer, "Principles of Random Walk" (2nd ed., Springer, 1964)
 *                    — rigorous theory; the 2-D random walk is RECURRENT, which is
 *                    why walkers keep revisiting the centre and caves cluster at
 *                    the seed (the clustering the MENTAL MODEL block describes).
 *                  • Pearson, "Generate Random Cave Levels Using Cellular
 *                    Automata" (Game Developer, 2010) — the CA alternative; a
 *                    useful contrast with agent carving:
 *                    https://www.gamedeveloper.com/programming/cellular-automata-for-physical-modeling
 *                  • Wikipedia, "Random walk" — accessible intro to recurrence /
 *                    diffusion: https://en.wikipedia.org/wiki/Random_walk
 *                  Rendering —
 *                  • Ben-Halim, Raymond et al., "Writing Programs with NCURSES"
 *                    (NCURSES HOWTO) — colour pairs, erase / wnoutrefresh /
 *                    doupdate; the §3 colour + §7 screen model used here.
 *                  • Patel, "Grids" (Red Blob Games, redblobgames.com) — grid
 *                    representation and 4/8-neighbour queries, as in the §7
 *                    "a wall is drawn only where it borders a floor" visibility test.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * If you want a cave that looks NATURAL — winding, irregular, no
 * straight lines — don't try to design rooms. Just send a few
 * drunkards stumbling around in a solid block of stone and have them
 * eat the cell they're standing on. After enough stumbling, the
 * negative space they've left behind IS your cave. The randomness
 * does all the aesthetic work for you.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture an ant in a rock who eats one grain of stone per step and
 * always picks a random direction to step next. Drop several such
 * ants near each other. Wait a while. The eaten-out region becomes
 * a cave system: the ants' overlapping paths form chambers (where
 * many ants meet), corridors (where one ant wandered far), and dead
 * ends (where an ant turned back). The PROBABILITY DISTRIBUTION of
 * a random walker is well-known — it tends to revisit the centre,
 * which is why caves cluster around the start point.
 *
 * Two visible layers:
 *   1. The WALKERS '@' (theme walker colour) are the ants right now.
 *     Each tick they each move one cell in a random cardinal direction.
 *   2. The TRAIL (theme trail colour, fading) shows where each walker
 *     was over the last ~5 steps. Watch any one '@' and you'll see
 *     a short comet tail behind it.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Every cell is TILE_WALL. Place N walkers at the map
 *     centre (with small random jitter so they don't pile up). Carve
 *     their starting cells. floor_count = N.
 *  2. STEP (one operation per walker, all walkers stepped in turn):
 *     a. d = uniform random ∈ {N, E, S, W}.
 *     b. (nx, ny) = (walker.x, walker.y) + direction_delta(d).
 *     c. If (nx, ny) out of bounds → don't move this step.
 *     d. Else: walker.x = nx, walker.y = ny. If grid[nx][ny] == WALL,
 *        carve it (set FLOOR, ++floor_count). Paint carve_glow flash.
 *     e. Paint walker_glow = 1.0 on (nx, ny).
 *     f. Increment walker.age. If age > MAX_AGE: respawn the walker
 *        at a random already-FLOOR cell, age = 0.
 *  3. Repeat step 2 until floor_count ≥ target_floor (= w·h·FILL_RATIO).
 *  4. HOLD on the finished cave for HOLD_SECONDS, then reset and
 *     goto 1.
 *
 * KEY FORMULAS
 * ────────────
 *  Tile flat index               : idx = y · w + x
 *  Direction delta               : N=(0,-1), E=(+1,0), S=(0,+1), W=(-1,0)
 *  Target floor count            : target = ⌊w · h · FILL_RATIO⌋
 *  Termination                   : floor_count ≥ target
 *  Glow decay (per frame)        : glow *= exp(-rate · dt)
 *  Walker respawn condition      : walker.age ≥ MAX_AGE
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • RE-VISITING A FLOOR. The walker doesn't care if the next cell
 *    is already FLOOR — it moves in regardless. Don't double-count
 *    floor_count: only increment when the cell was WALL before.
 *
 *  • OUT-OF-BOUNDS. The simplest handling is "skip the step" — keep
 *    the walker stationary if the random direction would leave the
 *    map. That's fine but creates a slight bias toward edges (walker
 *    spends extra ticks there). For a showcase, this bias is invisible.
 *    Alternatives: bounce (reverse direction), wrap (toroidal map), or
 *    re-roll the direction; each gives subtly different cave shapes.
 *
 *  • DENSITY VS CONNECTEDNESS. With N walkers all starting at centre,
 *    the cave is GUARANTEED connected (every walker shares a starting
 *    cell). Spawning walkers at random independent positions can
 *    produce DISCONNECTED caves. Don't do that unless the design
 *    wants islands.
 *
 *  • THE MAX_AGE RESPAWN is what prevents single-walker meandering
 *    (which produces ribbon-like rather than blob-like caves).
 *    Without it, an unlucky walker can wander to a far corner and
 *    spend the whole run there. With it, walkers stay in the
 *    connected blob.
 *
 *  • THE TARGET FILL FRACTION. Below ~25% the cave looks too sparse
 *    (lots of stone, few rooms). Above ~60% it looks like swiss cheese.
 *    45% is the sweet spot for "explorable cavern".
 *
 *  • TERMINATION on small maps. If walker collisions happen often,
 *    the random walk takes much longer than expected to add new
 *    floor cells. Empirically the algorithm needs Θ(N · target² /
 *    walker_count) random steps to hit the target on an N-cell map.
 *    For 8000 cells × 0.45 = 3600 floor at 4 walkers, that's ~300K
 *    steps — fast at modern speeds, but visibly long if you crank
 *    steps_per_tick down to 1.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Initial state: every cell is WALL except the N walker start
 *    positions. floor_count == N. If the screen shows large floor
 *    regions at t=0, the wall init is broken.
 *  • Walker stays in bounds — never see '@' outside the map area
 *    when paused. If it does, the bounds check is missing.
 *  • Floor count climbs monotonically until target. If it stalls
 *    BEFORE target, walkers are stuck (e.g. respawn picks an empty
 *    floor list — but you initialised walkers AS floor, so the list
 *    is non-empty, so this can't happen).
 *  • Connectedness: BFS from any walker's position should reach
 *    every floor cell. Untested by default but easy to add.
 *  • Different themes change WALL/FLOOR/WALKER colours but the
 *    cave shape is identical for the same RNG seed (the algorithm
 *    is theme-independent).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layers) ───────────────────────────────────────────────
 *
 * The file is cut into labelled layers. Data is still global/struct-based in
 * this pass (no new types — that is step 5); the table is the contract for who
 * writes what. Each tick, scene_tick() (§5) is the ONE place that advances
 * simulation state.
 *
 *   LAYER          §   MUTATES                              KEY FUNCTIONS
 *   ------------  --   ----------------------------------   ----------------------
 *   CONFIG         1   (constants + theme table)           —
 *   PERF/DELAYS    2   (local timing only)                 clock_ns, clock_sleep_ns
 *   STATE          3   (declares Walker/Cave/Scene/...)    —
 *   LOGIC          4   nothing (pure reads / index math)   dir_dx/dy, cave_idx,
 *                                                          cave_in_bounds,
 *                                                          tile_has_floor_neighbour
 *   SIMULATION     5   Cave.tiles/floor_count/walkers,     cave_carve, cave_walker_step,
 *                      Scene.state/hold_timer; paints      cave_step, cave_reset,
 *                      the EFFECTS glow buffers            scene_tick <-- combine point
 *   EFFECTS        6   Cave.carve_glow / walker_glow       effects_decay, paint_walker_glow (§5)
 *   RENDER         7   terminal + ncurses palette only     cave_draw, screen_draw,
 *                                                          theme_apply, color_init
 *   PLATFORM       8   g_screen dims, g_running flags      screen_init/resize, signals
 *   APP            9   drives the loop (no own sim state)  main, handle_key,
 *                                                          app_resize, pick_map_size
 *
 * Per-tick combine order (scene_tick, §5), runs only when not paused:
 *   1. DELAYS     — paused gate (early return).
 *   2. EFFECTS    — decay carve_glow / walker_glow by exp(-rate·dt).
 *   3. SIMULATION — SCENE_WALKING: steps_per_tick × cave_step (walkers move,
 *                   carve tiles, paint glows); on target → enter SCENE_HOLD.
 *                 — SCENE_HOLD: count down hold_timer (a DELAY); at 0 → scene_reset.
 *   The PERFORMANCE shell (fixed-timestep accumulator + 60 fps frame cap) lives
 *   in main (§9) and merely calls scene_tick; it advances no sim state itself.
 *
 * Notes / contracts:
 *   (b) LOGIC (§4) does no mutation and no I/O — render/effects order cannot
 *       change its result.
 *   (c) scene_tick() is the sole per-tick state advance. cave_reset / scene_reset
 *       / scene_init (§5) and app_resize / handle_key (§9) also mutate state,
 *       but are INIT or USER EVENTS, not part of the tick.
 *   EFFECTS: real but interwoven — the glow buffers live in Cave, are PAINTED by
 *       §5 sim functions (cave_carve, cave_walker_step) and DECAYED in scene_tick.
 *       Not a standalone layer in this pass; step 7 may extract the decay.
 *   DELAYS: the pause gate + HOLD timer live in scene_tick (§5); the inter-frame
 *       sleep primitive is clock_sleep_ns (§2), driven by the §9 frame cap.
 * ──────────────────────────────────────────────────────────────────────── */

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
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    /* Walker count — more walkers = more cave branches in less time.
     * 4 is a good showcase default; the user can't change this from
     * the keyboard but it's tunable here. */
    WALKERS_MAX       =  16,
    WALKERS_DEF       =   4,

    /* Walkers spawn within ±JITTER of the map centre — sharing a starting
     * region is what guarantees the cave comes out connected. */
    SPAWN_JITTER_X    =   3,        /* ± columns */
    SPAWN_JITTER_Y    =   2,        /* ± rows    */

    /* Walker lifetime in steps before respawning at a random floor.
     * 200 is enough to see a coherent meander but short enough that
     * walkers don't drift to the edges and stay there. */
    WALKER_MAX_AGE    = 200,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Per-tick walker steps. Default 2 ⇒ each walker advances 120
     * cells/sec, slow enough to see a comet trail. */
    STEPS_PER_TICK_MIN =   1,
    STEPS_PER_TICK_DEF =   2,
    STEPS_PER_TICK_MAX = 256,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,
    RENDER_FPS        =  60,        /* display refresh cap (independent of sim_fps) */
    MAX_FRAME_MS      = 100,        /* clamp dt to this — avoids spiral-of-death after a stall */

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* '#' walls — theme wall colour    */
    PAIR_FLOOR        =   4,        /* '.' floors — theme floor colour  */
    PAIR_WALKER       =   5,        /* '@' walkers — theme walker color */
    PAIR_TRAIL        =   6,        /* freshly-carved glow              */
};

/* Glow decay rates. */
#define CARVE_GLOW_DECAY    2.5f
#define WALKER_GLOW_DECAY   8.0f    /* fast — only the head + ~3 cells visible */
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.0f

/* Target carved fraction. 0.45 is the sweet spot for "explorable
 * cavern" — see the MENTAL MODEL block for the band rationale. */
#define FILL_RATIO          0.45f

/* Tile kinds. */
enum { TILE_WALL = 0, TILE_FLOOR = 1 };

/* Direction encoding — same as the other procedural files.
 * (The N/E/S/W → delta helpers live in §4 logic.) */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3 };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — one named 4-colour palette for the cave, ordered as a DARK→BRIGHT
 * depth ramp so the eye reads the scene as layered: resting stone sinks back,
 * the walker pops forward. Preserving that ordering is what keeps a palette
 * legible; the absolute hues are free to vary (see the table below).
 *
 * Values are xterm-256 colour indices (0–255), all chosen in the bright half of
 * the cube so even the darkest tier stays visible against the default-black
 * background (project COLOR.md theme-brightness rule). Cycled live with 't'/'T'
 * (theme_apply); the HUD/HINT pairs stay theme-independent for UI legibility.
 */
typedef struct {
    const char *name;     /* shown in the HUD; identifies the palette          */
    short       wall;     /* resting stone — dimmest tier, low-contrast vs bg   */
    short       floor;    /* carved cells — brighter than wall, below walker    */
    short       walker;   /* the '@' cursor — brightest tier, draws the eye     */
    short       trail;    /* carve-glow flash — transient accent on a fresh carve */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      wall floor walker trail */
    { "DEFAULT",  240,   67,  231,  220 },   /* grey / blue / white / gold   */
    { "MATRIX",    22,   34,  118,   46 },   /* dark green → bright green    */
    { "NOVA",      53,  129,  219,  201 },   /* purple → pink → magenta      */
    { "MONO",     234,  244,  254,  250 },   /* greyscale gradient           */
    { "OCEAN",     17,   33,   51,   39 },   /* navy → cyan                  */
    { "FIRE",      52,  124,  226,  196 },   /* dark red → yellow walker     */
    { "EARTH",     58,  137,  230,  173 },   /* brown → cream                */
    { "FOREST",    22,   64,  144,   28 },   /* dark green → tan walker      */
    { "DESERT",    94,  222,  230,  178 },   /* brown → sand                 */
    { "ARCTIC",    18,   39,  231,  159 },   /* navy → white walker          */
};

/* ===================================================================== */
/* §2  perf / delays  —  monotonic clock + sleep primitives               */
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
/* §3  state  —  all program data (mutated by SIMULATION §5 / APP §9)      */
/* ===================================================================== */

/*
 * Walker — one "drunkard": a single random walker, the elementary agent of the
 * algorithm. Each tick it steps one cell in a uniformly-random cardinal
 * direction and carves whatever it lands on. The 2-D random walk is RECURRENT
 * (Spitzer, "Principles of Random Walk") — it keeps returning near its start —
 * which is exactly why a cluster of walkers carves one CONNECTED blob around
 * the seed instead of wandering off into disjoint tunnels.
 */
typedef struct {
    int x, y;   /* current cell, in map coordinates (0..w-1, 0..h-1)            */
    int age;    /* steps since this walker last (re)spawned. At WALKER_MAX_AGE
                 * it teleports to a random floor cell and age resets to 0 — the
                 * leash that stops one unlucky walker drifting to a far corner
                 * and stranding there (which would yield ribbon-like caves).    */
} Walker;

/*
 * Cave — the map a drunkard's walk carves out, plus the state that produces it.
 * The OUTPUT is `tiles`, the wall/floor grid a cave-gen reference calls "the
 * cave" (RogueBasin, "Random Walk Cave Generation"; Shaker/Togelius/Nelson,
 * "Procedural Content Generation in Games"). The rest is generation
 * bookkeeping: the agents, a progress counter, and a cosmetic glow overlay.
 *
 * Layout choices worth knowing: the grid is a FLAT row-major array
 * (idx = y·w + x, see cave_idx), not 2-D, so it memsets and scans in one pass;
 * everything is fixed-size at CELLS_MAX so the hot path never allocates.
 * Connectedness is structural, not checked — all walkers seed near the centre,
 * so their carved territories necessarily fuse into one blob.
 */
typedef struct {
    /* THE GRID — the cave itself (SIMULATION writes; LOGIC + RENDER read) */
    int     w, h;                  /* active extent this run (≤ MAP_W/H_MAX)    */
    int     total_cells;           /* w·h, cached so scan loops skip the multiply */
    uint8_t tiles[CELLS_MAX];      /* TILE_WALL | TILE_FLOOR, one byte per cell  */

    /* THE DRUNKARDS carving it (the algorithm's agents) */
    Walker  walkers[WALKERS_MAX];  /* fixed pool; only the first n_walkers live  */
    int     n_walkers;             /* more walkers → denser, more-branched cave  */

    /* CARVE PROGRESS / termination */
    int     floor_count;           /* cells carved so far (climbs monotonically) */
    int     target_floor;          /* = ⌊w·h·FILL_RATIO⌋; the carve stops here   */

    /* EFFECTS overlay — cosmetic per-cell glows in [0,1], painted by SIMULATION
     * and decayed each tick by exp(-rate·dt) (§6). Indexed identically to tiles
     * so they ride along with the grid; never read by SIMULATION or LOGIC. */
    float   carve_glow [CELLS_MAX];   /* fresh wall→floor flash (trail colour)   */
    float   walker_glow[CELLS_MAX];   /* walker head + its recent comet tail     */
} Cave;

/*
 * SceneState — which phase of the forever-loop the demo is in. The showcase
 * runs WALKING (carve until target) → HOLD (admire the finished cave for
 * HOLD_SECONDS) → reset → WALKING …, so the viewer always sees a cave reach
 * completion before the next one starts carving in.
 */
typedef enum {
    SCENE_WALKING = 0,   /* advancing walkers, carving toward target_floor   */
    SCENE_HOLD    = 1,   /* target met; counting hold_timer down to a reset  */
} SceneState;

/*
 * Scene — the whole simulation in one place, read like a table of contents
 * (WHAT is generated, HOW the user drives it, WHERE we are, how it is shown).
 * One global g_scene holds it; orchestrators take Scene*, workers take the
 * narrowest sub-type, so aggregating here never re-couples the layers.
 */
typedef struct {
    /* WHAT is simulated */
    Cave        cave;             /* the cave being carved (+ its agents)      */

    /* HOW the user drives the carve — two independent speed knobs:
     *   steps_per_tick = work done per simulation tick (how chunky each step),
     *   sim_fps        = how many ticks run per wall-clock second.
     * Their product is walker-steps/second; splitting them lets you slow the
     * animation (low sim_fps) without changing the per-step granularity. */
    int         steps_per_tick;   /* STEPS_PER_TICK_MIN..MAX; doubles on +/-   */
    int         sim_fps;          /* SIM_FPS_MIN..MAX; ±SIM_FPS_STEP on ]/[     */

    /* WHERE / when we are — run-state, persists across resets */
    SceneState  state;            /* WALKING | HOLD (see SceneState)           */
    float       hold_timer;       /* seconds left before HOLD → reset          */
    bool        paused;           /* user freeze: halts the tick, not RENDER   */

    /* RENDER selection — a view choice, deliberately apart from the sim knobs */
    int         theme;            /* index into themes[] (0..N_THEMES-1)       */
} Scene;

/*
 * Screen — the terminal viewport, measured in character cells as reported by
 * ncurses getmaxyx(). Re-read on every SIGWINCH; the cave is then re-sized and
 * re-centred to fit (pick_map_size, cave_draw). Platform-owned (§8), read by
 * RENDER for layout.
 */
typedef struct {
    int cols;   /* terminal width  in columns */
    int rows;   /* terminal height in rows    */
} Screen;

/* The one simulation, plus the platform handles it runs on. */
static Scene  g_scene;
static Screen g_screen;
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

/* ===================================================================== */
/* §4  logic  —  pure decisions: no mutation, no I/O, readable alone       */
/* ===================================================================== */

static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }

static inline int cave_idx(const Cave *c, int x, int y) { return y * c->w + x; }
static inline bool cave_in_bounds(const Cave *c, int x, int y)
{
    return x >= 0 && x < c->w && y >= 0 && y < c->h;
}

/*
 * tile_has_floor_neighbour — 8-connected check used to decide whether
 * a wall cell renders as '#' (visible — borders a floor) or as ' '
 * (interior void). Same trick as bsp_dungeon_showcase.c §7.
 */
static bool tile_has_floor_neighbour(const Cave *c, int x, int y)
{
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (!cave_in_bounds(c, nx, ny)) continue;
            if (c->tiles[cave_idx(c, nx, ny)] == TILE_FLOOR) return true;
        }
    }
    return false;
}

/* ===================================================================== */
/* §5  simulation  —  the only layer that advances state                  */
/* ===================================================================== */

/* paint_walker_glow — light a cell to full intensity as walker-occupied
 * (EFFECTS §6); the comet tail is just this value decaying over later ticks. */
static void paint_walker_glow(Cave *c, int x, int y)
{
    c->walker_glow[cave_idx(c, x, y)] = 1.0f;
}

/* effects_decay — fade every cosmetic glow one tick toward zero (EFFECTS §6).
 * Geometric: glow *= exp(-rate·dt), so a glow halves on a fixed wall-clock
 * schedule regardless of frame rate. */
static void effects_decay(Cave *c, float dt)
{
    float carve_d  = expf(-CARVE_GLOW_DECAY  * dt);
    float walker_d = expf(-WALKER_GLOW_DECAY * dt);
    for (int i = 0; i < c->total_cells; i++) {
        c->carve_glow[i]  *= carve_d;
        c->walker_glow[i] *= walker_d;
    }
}

/* pick_random_floor_cell — reservoir-sample one floor cell uniformly at random
 * in a single pass; returns its flat index, or -1 if no floor exists yet. */
static int pick_random_floor_cell(const Cave *c)
{
    int chosen = -1, seen = 0;
    for (int i = 0; i < c->total_cells; i++) {
        if (c->tiles[i] == TILE_FLOOR) {
            seen++;
            if ((rand() % seen) == 0) chosen = i;
        }
    }
    return chosen;
}

/*
 * cave_carve — set (x, y) to TILE_FLOOR and paint carve_glow if it was
 * a wall before. Idempotent on already-floor cells; floor_count only
 * increments on the wall→floor transition.
 */
static void cave_carve(Cave *c, int x, int y)
{
    int idx = cave_idx(c, x, y);
    if (c->tiles[idx] == TILE_WALL) {
        c->tiles[idx] = TILE_FLOOR;
        c->carve_glow[idx] = 1.0f;
        c->floor_count++;
    }
}

/*
 * cave_respawn_walker — teleport the walker to a random already-floor cell and
 * reset its age. Fired when age exceeds WALKER_MAX_AGE so no walker gets
 * stranded in a far corner. Floors always exist (seeded by spawn_walkers).
 */
static void cave_respawn_walker(Cave *c, Walker *w)
{
    int cell = pick_random_floor_cell(c);
    if (cell < 0) return;            /* shouldn't happen */
    w->x = cell % c->w;              /* unflatten the cell index → (x, y) */
    w->y = cell / c->w;
    w->age = 0;
}

/*
 * cave_walker_step — advance one walker by one random cardinal step.
 * Updates the walker_glow (head halo), carve_glow (trail), and the
 * tile grid.
 */
static void cave_walker_step(Cave *c, Walker *w)
{
    int dir = rand() & 3;            /* low 2 bits → one uniform cardinal */
    int nx  = w->x + dir_dx(dir);
    int ny  = w->y + dir_dy(dir);
    w->age++;

    if (cave_in_bounds(c, nx, ny)) {
        w->x = nx;
        w->y = ny;
        cave_carve(c, nx, ny);          /* eat the new cell if it was stone */
        paint_walker_glow(c, nx, ny);
    } else {
        /* would step off the map — stay put, but keep the walker lit so it
         * still looks alive (slight edge bias, invisible at showcase scale) */
        paint_walker_glow(c, w->x, w->y);
    }

    if (w->age >= WALKER_MAX_AGE)
        cave_respawn_walker(c, w);
}

/*
 * cave_step — advance every walker by ONE step. Caller decides how
 * many cave_step calls to do per scene_tick (steps_per_tick).
 *
 * Returns true if work was done; false once the carve target is met.
 */
static bool cave_step(Cave *c)
{
    if (c->floor_count >= c->target_floor) {
        return false;
    }
    for (int i = 0; i < c->n_walkers; i++) {
        cave_walker_step(c, &c->walkers[i]);
    }
    return true;
}

/* fill_with_walls — reset the whole grid to solid stone and clear all glows. */
static void fill_with_walls(Cave *c)
{
    for (int i = 0; i < c->total_cells; i++) {
        c->tiles[i]       = TILE_WALL;
        c->carve_glow[i]  = 0.0f;
        c->walker_glow[i] = 0.0f;
    }
}

/*
 * spawn_walkers — seed n_walkers in a small jitter window around the map centre
 * and carve their starting cells. Sharing a starting region is what GUARANTEES
 * the cave comes out connected (the walkers' territories necessarily overlap).
 */
static void spawn_walkers(Cave *c)
{
    int cx = c->w / 2;
    int cy = c->h / 2;
    for (int i = 0; i < c->n_walkers; i++) {
        int wx = cx + (rand() % (2 * SPAWN_JITTER_X + 1)) - SPAWN_JITTER_X;
        int wy = cy + (rand() % (2 * SPAWN_JITTER_Y + 1)) - SPAWN_JITTER_Y;

        /* keep the spawn off the 1-cell border ring */
        if (wx < 1) wx = 1;
        if (wy < 1) wy = 1;
        if (wx >= c->w - 1) wx = c->w - 2;
        if (wy >= c->h - 1) wy = c->h - 2;

        c->walkers[i].x = wx;
        c->walkers[i].y = wy;
        c->walkers[i].age = 0;
        cave_carve(c, wx, wy);
        paint_walker_glow(c, wx, wy);
    }
}

/* cave_reset — begin a fresh run: a solid wall map seeded with centred walkers. */
static void cave_reset(Cave *c, int w, int h, int n_walkers)
{
    c->w = w;
    c->h = h;
    c->total_cells = w * h;
    c->target_floor = (int)((float)c->total_cells * FILL_RATIO);
    c->n_walkers = (n_walkers < 1) ? 1
                 : (n_walkers > WALKERS_MAX) ? WALKERS_MAX
                 : n_walkers;
    c->floor_count = 0;

    fill_with_walls(c);
    spawn_walkers(c);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    cave_reset(&s->cave, mw, mh, WALKERS_DEF);
    s->state      = SCENE_WALKING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused         = false;
    s->steps_per_tick = STEPS_PER_TICK_DEF;
    s->sim_fps        = SIM_FPS_DEFAULT;
    s->theme          = 0;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;                  /* DELAY gate: frozen, no advance   */

    effects_decay(&s->cave, dt);            /* EFFECTS: fade the cosmetic glows */

    switch (s->state) {                     /* SIMULATION: advance the carve    */

    case SCENE_WALKING:
        for (int i = 0; i < s->steps_per_tick; i++) {
            if (!cave_step(&s->cave)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->cave.w, s->cave.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §6  effects                                                            */
/* ===================================================================== */

/*
 * EFFECTS state = Cave.carve_glow + Cave.walker_glow (§3). Its operations are
 * named (paint_walker_glow, effects_decay) but live in §5, because SIMULATION
 * invokes them and a §6 section sitting AFTER §5 could not be called from it:
 *   • PAINTED by SIMULATION — cave_carve sets carve_glow=1 on a fresh carve;
 *     paint_walker_glow lights the walker's cell.
 *   • DECAYED by effects_decay, called once per tick from scene_tick (§5).
 *   • READ by RENDER — draw_cave_cell (§7) thresholds the glows to pick glyph/colour.
 */

/* ===================================================================== */
/* §7  render  —  state → screen, reads only; palette setup               */
/* ===================================================================== */

/*
 * theme_apply — install one of the 10 named palettes. Re-initialises
 * the 4 cave colour pairs only; HUD/HINT stay theme-independent.
 * Safe to call any time — ncurses' init_pair updates the live pair
 * definition; cells already on screen redraw with the new colours on
 * the next refresh.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        init_pair(PAIR_WALL,   t->wall,   -1);
        init_pair(PAIR_FLOOR,  t->floor,  -1);
        init_pair(PAIR_WALKER, t->walker, -1);
        init_pair(PAIR_TRAIL,  t->trail,  -1);
    } else {
        /* 8-colour fallback — themes can't be distinguished. */
        init_pair(PAIR_WALL,   COLOR_WHITE,  -1);
        init_pair(PAIR_FLOOR,  COLOR_BLUE,   -1);
        init_pair(PAIR_WALKER, COLOR_WHITE,  -1);
        init_pair(PAIR_TRAIL,  COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    theme_apply(0);
}

/*
 * draw_cave_cell — pick one cell's glyph + colour and blit it at (sy, sx).
 * Priority: live walker glow > fresh-carve glow > settled floor > wall. A wall
 * is drawn ONLY where it borders a floor, so unreached interior void stays
 * blank and the cave reads as a cavern rather than a filled rectangle.
 */
static void draw_cave_cell(const Cave *c, int x, int y, int sy, int sx)
{
    int idx = cave_idx(c, x, y);
    float wg = c->walker_glow[idx];
    float cg = c->carve_glow[idx];
    uint8_t k = c->tiles[idx];

    int  pair = 0, attr = A_NORMAL;
    char glyph = ' ';

    if (wg > GLOW_THRESHOLD) {
        pair = PAIR_WALKER; attr = A_BOLD;   glyph = '@';   /* walker head / comet */
    } else if (cg > GLOW_THRESHOLD) {
        pair = PAIR_TRAIL;  attr = A_BOLD;   glyph = '.';   /* just-carved flash   */
    } else if (k == TILE_FLOOR) {
        pair = PAIR_FLOOR;  attr = A_DIM;    glyph = '.';   /* settled floor       */
    } else {
        /* WALL — only render where adjacent to floor. */
        if (tile_has_floor_neighbour(c, x, y)) {
            pair = PAIR_WALL; attr = A_NORMAL; glyph = '#'; /* visible wall face   */
        } else {
            return;                                         /* interior void — blank */
        }
    }

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

static void cave_draw(const Cave *c, int cols, int rows)
{
    /* centre the map under the 2 HUD rows, above the hint row */
    int gx0 = (cols - c->w) / 2;
    int gy0 = ((rows - 3) - c->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    for (int y = 0; y < c->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < c->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            draw_cave_cell(c, x, y, sy, sx);
        }
    }
}

/*
 * draw_glyph_legend — append a colour-keyed glyph legend after the row-1 params
 * (using the live cursor column), each symbol drawn in its current theme colour
 * so the legend doubles as a palette swatch. Drawn only when it fits, so it
 * never overlaps the params or wraps onto the map.
 */
static void draw_glyph_legend(const Screen *sc)
{
    int lx = getcurx(stdscr);
    const char *legend = "  #:wall  .:floor  @:walker ";
    if (lx + (int)strlen(legend) <= sc->cols) {
        int p = lx + 2;                       /* 2-col gap after the params */

        attron(COLOR_PAIR(PAIR_WALL));
        mvaddch(1, p++, '#');
        attroff(COLOR_PAIR(PAIR_WALL));
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(1, p, ":wall  ");  p += 7;
        attroff(COLOR_PAIR(PAIR_HUD));

        attron(COLOR_PAIR(PAIR_FLOOR) | A_DIM);
        mvaddch(1, p++, '.');
        attroff(COLOR_PAIR(PAIR_FLOOR) | A_DIM);
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(1, p, ":floor  ");  p += 8;
        attroff(COLOR_PAIR(PAIR_HUD));

        attron(COLOR_PAIR(PAIR_WALKER) | A_BOLD);
        mvaddch(1, p++, '@');
        attroff(COLOR_PAIR(PAIR_WALKER) | A_BOLD);
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(1, p, ":walker");
        attroff(COLOR_PAIR(PAIR_HUD));
    }
}

static void screen_draw(const Screen *sc, const Scene *s, double fps)
{
    erase();
    cave_draw(&s->cave, sc->cols, sc->rows);

    const Cave *c = &s->cave;
    const char *state_str =
        s->paused                     ? "PAUSED " :
        (s->state == SCENE_WALKING)   ? "WALKING" :
                                        "HOLD   ";

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    int pct = (c->target_floor > 0)
            ? (100 * c->floor_count / c->target_floor)
            : 0;
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  steps:%-3d  %s  %3d%%  %5d/%-5d ",
             fps, s->sim_fps, s->steps_per_tick, state_str,
             pct, c->floor_count, c->target_floor);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " DRUNKARD'S WALK CAVE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — theme name (bold) + walker count + map size. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " walkers:%-2d  age-cap:%-3d  fill-target:%d%%  map:%dx%d ",
             c->n_walkers, WALKER_MAX_AGE,
             (int)(FILL_RATIO * 100.0f), c->w, c->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Row 1 right — glyph legend (a live palette swatch); see draw_glyph_legend. */
    draw_glyph_legend(sc);

    /* Bottom hint — actions only (glyph legend now lives on the top HUD). */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  platform  —  ncurses setup, resize, signals                        */
/* ===================================================================== */

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

static void on_exit_signal  (int sig) { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Restore the terminal on exit and route quit/resize signals to the flags */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* ===================================================================== */
/* §9  app  —  input, fixed-timestep loop, main                           */
/* ===================================================================== */

/* seed_rng — seed libc rand() from the monotonic clock so every run differs */
static void seed_rng(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
}

/* Choose the cave's extent from the terminal size, clamped to the buffer cap */
static void pick_map_size(const Screen *scr, int *mw, int *mh)
{
    int w = scr->cols;
    int h = scr->rows - 3;     /* 2 HUD rows + 1 hint row */
    if (w < 16) w = 16;
    if (h < 8)  h = 8;
    if (w > MAP_W_MAX) w = MAP_W_MAX;
    if (h > MAP_H_MAX) h = MAP_H_MAX;
    *mw = w;
    *mh = h;
}

static void app_resize(Scene *s, Screen *scr)
{
    screen_resize(scr);
    int mw, mh;
    pick_map_size(scr, &mw, &mh);
    scene_reset(s, mw, mh);
    g_need_resize = 0;
}

static bool handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, s->cave.w, s->cave.h);
        break;
    case '=': case '+':
        if (s->steps_per_tick < STEPS_PER_TICK_MAX) s->steps_per_tick *= 2;
        if (s->steps_per_tick > STEPS_PER_TICK_MAX) s->steps_per_tick = STEPS_PER_TICK_MAX;
        break;
    case '-':
        s->steps_per_tick /= 2;
        if (s->steps_per_tick < STEPS_PER_TICK_MIN) s->steps_per_tick = STEPS_PER_TICK_MIN;
        break;
    case ']':
        s->sim_fps += SIM_FPS_STEP;
        if (s->sim_fps > SIM_FPS_MAX) s->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        s->sim_fps -= SIM_FPS_STEP;
        if (s->sim_fps < SIM_FPS_MIN) s->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    seed_rng();
    install_signals();

    screen_init(&g_screen);
    int mw, mh;
    pick_map_size(&g_screen, &mw, &mh);
    scene_init(&g_scene, mw, mh);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (g_running) {

        if (g_need_resize) {
            app_resize(&g_scene, &g_screen);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        int64_t tick_ns = TICK_NS(g_scene.sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&g_scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* throttle the display to RENDER_FPS: sleep off the frame's slack */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / RENDER_FPS - elapsed);

        screen_draw(&g_screen, &g_scene, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !handle_key(&g_scene, ch))
            g_running = 0;
    }

    screen_free(&g_screen);
    return 0;
}
