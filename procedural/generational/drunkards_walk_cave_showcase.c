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
 *       fraction reaches the target, HOLD on the cave; supernova
 *       flash; repeat forever with a fresh layout.
 *
 * Study alongside: ./bsp_dungeon_showcase.c — the geometric opposite.
 *       BSP gives you sharp rectangular rooms and L-corridors;
 *       Drunkard's Walk gives you blobby, organic caverns. Two answers
 *       to the same "make a dungeon" question.
 *
 * Section map:
 *   §1 config   — map size, walker count, fill ratio, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 cave themes
 *   §5 cave     — Walker, Cave, walker_step, restart logic
 *   §6 scene    — WALKING / HOLD state machine
 *   §7 screen   — ASCII render: # walls, . floors, @ walkers, trails
 *   §8 app      — signals, resize, main loop
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
 *                  floats (carve_glow + walker_glow + supernova_glow),
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
 * References     : • RogueBasin — "Random Walk Cave Generation":
 *                    http://www.roguebasin.com/index.php?title=Random_Walk_Cave_Generation
 *                  • Pearson — "Generate Random Cave Levels Using Cellular
 *                    Automata" (also covers drunkard's walk):
 *                    https://www.gamedeveloper.com/programming/cellular-automata-for-physical-modeling
 *                  • Wikipedia — "Random walk":
 *                    https://en.wikipedia.org/wiki/Random_walk
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
 *  4. HOLD on the finished cave for HOLD_SECONDS, then supernova
 *     reset and goto 1.
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

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* '#' walls — theme wall colour    */
    PAIR_FLOOR        =   4,        /* '.' floors — theme floor colour  */
    PAIR_WALKER       =   5,        /* '@' walkers — theme walker color */
    PAIR_TRAIL        =   6,        /* freshly-carved glow              */
    PAIR_FLASH        =   7,        /* walker's brief halo on each step */
    PAIR_SUPERNOVA    =   8,        /* yellow reset flash               */
};

/* Glow decay rates. */
#define CARVE_GLOW_DECAY    2.5f
#define WALKER_GLOW_DECAY   8.0f    /* fast — only the head + ~3 cells visible */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.0f

/* Target carved fraction. 0.45 is the sweet spot for "explorable
 * cavern" — see the MENTAL MODEL block for the band rationale. */
#define FILL_RATIO          0.45f

/* Tile kinds. */
enum { TILE_WALL = 0, TILE_FLOOR = 1 };

/* Direction encoding — same as the other procedural files. */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };
static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — 10 named palettes. Each theme defines four colours:
 *   wall   — the resting stone (dim, low-contrast against the bg)
 *   floor  — carved cells (brighter than wall, lower than walker)
 *   walker — the '@' cursor (brightest in the theme)
 *   trail  — carve_glow flash (a transitional accent colour)
 *
 * Cycle with 't' (next) and 'T' (previous). HUD/HINT/FLASH/SUPERNOVA
 * pairs stay constant across themes for UI legibility.
 */
typedef struct {
    const char *name;
    short       wall, floor, walker, trail;
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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * theme_apply — install one of the 10 named palettes. Re-initialises
 * the 4 cave colour pairs only; HUD/HINT/FLASH/SUPERNOVA stay
 * theme-independent. Safe to call any time — ncurses' init_pair updates
 * the live pair definition; cells already on screen redraw with the
 * new colours on the next refresh.
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
        init_pair(PAIR_FLASH,      220, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_FLASH,     COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  cave                                                               */
/* ===================================================================== */

/*
 * Walker — one drunkard's state.
 *
 *   x, y  : current position in map coordinates
 *   age   : steps since this walker last respawned. When age reaches
 *           WALKER_MAX_AGE the walker teleports to a random floor cell
 *           and age resets to 0 — keeps walkers in the connected blob.
 */
typedef struct {
    int x, y;
    int age;
} Walker;

/*
 * Cave — the simulation heart. Tile grid + walker array + the three
 * glow buffers and per-frame stats.
 */
typedef struct {
    int     w, h;
    int     total_cells;
    uint8_t tiles[CELLS_MAX];

    float   carve_glow    [CELLS_MAX];   /* trail colour fade */
    float   walker_glow   [CELLS_MAX];   /* per-walker comet tail */
    float   supernova_glow[CELLS_MAX];   /* reset flash */

    Walker  walkers[WALKERS_MAX];
    int     n_walkers;

    int     floor_count;
    int     target_floor;
    bool    done;
} Cave;

static inline int cave_idx(const Cave *c, int x, int y) { return y * c->w + x; }
static inline bool cave_in_bounds(const Cave *c, int x, int y)
{
    return x >= 0 && x < c->w && y >= 0 && y < c->h;
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
 * cave_respawn_walker — teleport the walker to a random already-floor
 * cell and reset its age. Used when age exceeds WALKER_MAX_AGE so the
 * walker doesn't get stuck in a far corner of the map.
 *
 * Implementation: reservoir sampling among floor cells. One pass.
 * Floors always exist (we seeded them in cave_reset).
 */
static void cave_respawn_walker(Cave *c, Walker *w)
{
    int chosen = -1, seen = 0;
    for (int i = 0; i < c->total_cells; i++) {
        if (c->tiles[i] == TILE_FLOOR) {
            seen++;
            if ((rand() % seen) == 0) chosen = i;
        }
    }
    if (chosen < 0) return;       /* shouldn't happen */
    w->x = chosen % c->w;
    w->y = chosen / c->w;
    w->age = 0;
}

/*
 * cave_walker_step — advance one walker by one random cardinal step.
 * Updates the walker_glow (head halo), carve_glow (trail), and the
 * tile grid.
 */
static void cave_walker_step(Cave *c, Walker *w)
{
    /* Pick a random direction. If the resulting cell is out of bounds,
     * skip this step — the walker stays in place this tick. (Slight
     * edge bias is invisible at showcase scale.) */
    int d  = rand() & 3;
    int nx = w->x + dir_dx(d);
    int ny = w->y + dir_dy(d);
    w->age++;

    if (cave_in_bounds(c, nx, ny)) {
        w->x = nx;
        w->y = ny;
        cave_carve(c, nx, ny);
        c->walker_glow[cave_idx(c, nx, ny)] = 1.0f;
    } else {
        /* Out of bounds — paint a glow flicker on current cell so the
         * walker still appears alive. */
        c->walker_glow[cave_idx(c, w->x, w->y)] = 1.0f;
    }

    if (w->age >= WALKER_MAX_AGE) {
        cave_respawn_walker(c, w);
    }
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
        c->done = true;
        return false;
    }
    for (int i = 0; i < c->n_walkers; i++) {
        cave_walker_step(c, &c->walkers[i]);
    }
    return true;
}

/*
 * cave_reset — solid wall map, walkers spawned near centre, supernova
 * glow set on every cell (the yellow flash that fades over the first
 * ~0.4 s).
 */
static void cave_reset(Cave *c, int w, int h, int n_walkers)
{
    c->w = w;
    c->h = h;
    c->total_cells = w * h;
    c->target_floor = (int)((float)c->total_cells * FILL_RATIO);
    c->n_walkers = (n_walkers < 1) ? 1
                 : (n_walkers > WALKERS_MAX) ? WALKERS_MAX
                 : n_walkers;
    c->done = false;
    c->floor_count = 0;

    int n = w * h;
    for (int i = 0; i < n; i++) {
        c->tiles[i]          = TILE_WALL;
        c->carve_glow[i]     = 0.0f;
        c->walker_glow[i]    = 0.0f;
        c->supernova_glow[i] = 1.0f;
    }

    /* Spawn walkers in a small jitter around the map centre — they
     * share a starting region, which guarantees the resulting cave
     * is connected. */
    int cx = w / 2;
    int cy = h / 2;
    for (int i = 0; i < c->n_walkers; i++) {
        int wx = cx + (rand() % 7) - 3;
        int wy = cy + (rand() % 5) - 2;
        if (wx < 1) wx = 1;
        if (wy < 1) wy = 1;
        if (wx >= w - 1) wx = w - 2;
        if (wy >= h - 1) wy = h - 2;
        c->walkers[i].x = wx;
        c->walkers[i].y = wy;
        c->walkers[i].age = 0;
        cave_carve(c, wx, wy);
        c->walker_glow[cave_idx(c, wx, wy)] = 1.0f;
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   WALKING — perform steps_per_tick cave steps. Each cave_step moves
 *             every walker once. Transitions to HOLD when target is
 *             reached.
 *   HOLD    — wait HOLD_SECONDS, then cave_reset and back to WALKING.
 */
typedef enum {
    SCENE_WALKING = 0,
    SCENE_HOLD    = 1,
} SceneState;

typedef struct {
    Cave        c;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         steps_per_tick;
    int         current_theme;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    cave_reset(&s->c, mw, mh, WALKERS_DEF);
    s->state      = SCENE_WALKING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused         = false;
    s->steps_per_tick = STEPS_PER_TICK_DEF;
    s->current_theme  = 0;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Decay glows. */
    float carve_d  = expf(-CARVE_GLOW_DECAY  * dt);
    float walker_d = expf(-WALKER_GLOW_DECAY * dt);
    float nova_d   = expf(-SUPERNOVA_DECAY   * dt);
    int n = s->c.total_cells;
    for (int i = 0; i < n; i++) {
        s->c.carve_glow[i]     *= carve_d;
        s->c.walker_glow[i]    *= walker_d;
        s->c.supernova_glow[i] *= nova_d;
    }

    switch (s->state) {

    case SCENE_WALKING:
        for (int i = 0; i < s->steps_per_tick; i++) {
            if (!cave_step(&s->c)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->c.w, s->c.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Cave *c = &s->c;

    int gx0 = (cols - c->w) / 2;
    int gy0 = ((rows - 3) - c->h) / 2 + 2;   /* leave row 0 + 1 + last */
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    for (int y = 0; y < c->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < c->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = cave_idx(c, x, y);
            float ng = c->supernova_glow[idx];
            float wg = c->walker_glow[idx];
            float cg = c->carve_glow[idx];
            uint8_t k = c->tiles[idx];

            int  pair = -1, attr = A_NORMAL;
            char glyph = ' ';

            if (ng > GLOW_THRESHOLD) {
                /* Whole-grid yellow flash on reset. */
                pair = PAIR_SUPERNOVA;
                attr = A_BOLD;
                glyph = '*';
            } else if (wg > GLOW_THRESHOLD) {
                /* Walker '@' or recent comet trail cell. The closer
                 * the cell is to the current walker (higher glow),
                 * the brighter — but we just use a single threshold
                 * here for simplicity. */
                pair = PAIR_WALKER;
                attr = A_BOLD;
                glyph = '@';
            } else if (cg > GLOW_THRESHOLD) {
                /* Just-carved floor — flash trail colour. */
                pair = PAIR_TRAIL;
                attr = A_BOLD;
                glyph = '.';
            } else if (k == TILE_FLOOR) {
                pair = PAIR_FLOOR;
                attr = A_DIM;
                glyph = '.';
            } else {
                /* WALL — only render where adjacent to floor. */
                if (tile_has_floor_neighbour(c, x, y)) {
                    pair = PAIR_WALL;
                    attr = A_NORMAL;
                    glyph = '#';
                } else {
                    continue;
                }
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Cave *c = &s->c;
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
             fps, sim_fps, s->steps_per_tick, state_str,
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
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " walkers:%-2d  age-cap:%-3d  fill-target:%d%%  map:%dx%d ",
             c->n_walkers, WALKER_MAX_AGE,
             (int)(FILL_RATIO * 100.0f), c->w, c->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " #:wall  .:floor  @:walker | t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 3;     /* 2 HUD rows + 1 hint row */
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
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
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
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
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
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

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
