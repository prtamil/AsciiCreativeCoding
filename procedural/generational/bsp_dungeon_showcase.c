/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bsp_dungeon_showcase.c — Binary Space Partition dungeon, animated.
 *
 * DEMO: Watch a classic roguelike dungeon assemble in three visible
 *       phases. PARTITION: a single rectangle (the whole map) is
 *       recursively split with cyan grid lines, dividing into leaf
 *       cells. CARVE: inside each leaf, a smaller pink room appears
 *       cell-by-cell. CORRIDORS: gold L-shaped passages walk between
 *       sibling rooms, joining everything into one connected dungeon.
 *       Fresh cuts glow white-hot and cool through the theme colour like
 *       embers. When the dungeon is complete, a glowing path blinks along
 *       a route from one corner room to the opposite one (BFS shortest
 *       path) and holds there — press n for a fresh map. Cycle 5 themes
 *       with t/T.
 *
 * Study alongside: ./maze_backtracker.c — the OTHER classic dungeon-
 *       like generator. DFS makes one twisty corridor; BSP makes many
 *       distinct rooms with controlled corridors between. BSP is what
 *       Rogue / NetHack / classic roguelikes use.
 *
 * Section map  (layers separated):
 *   §1 config   — tunables, palettes, named constants
 *   §2 clock    — monotonic timer + sleep (the program's DELAYS)
 *   §3 color    — theme → ncurses pairs (a terminal effect) + glow→tier
 *   §4 geometry — Point / Rect: the algorithm is rectangle subdivision
 *   §5 dungeon  — the world: Fifo queues, BSPNode/Dungeon data, pure reads,
 *                 and the generation EFFECTS (split / carve / corridor / path)
 *   §6 scene    — the phase state machine that drives those effects
 *   §7 render   — PURE: const Scene → screen (mutates nothing but the term)
 *   §8 app      — orchestration: input → effects → delay → render
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   n          generate the next map (no auto-restart; press to advance)
 *   t / T      next / previous colour theme
 *   + / =      faster phase pacing
 *   -          slower phase pacing
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra bsp_dungeon_showcase.c \
 *       -o bsp_dungeon -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Binary Space Partition for dungeon generation. Three
 *                  stages:
 *                    1. PARTITION the map by repeatedly splitting the
 *                       largest leaf rectangle along its longer axis.
 *                       Stops when leaves are below a size threshold.
 *                       Result: a binary tree of axis-aligned rectangles
 *                       tiling the map with no overlap.
 *                    2. CARVE one room inside each leaf, with random
 *                       padding so rooms don't fill their leaf entirely.
 *                    3. CONNECT sibling rooms via L-shaped corridors —
 *                       a post-order traversal of the BSP tree, joining
 *                       a representative room from each subtree at every
 *                       internal node. Result: a connected, axis-aligned
 *                       dungeon.
 *
 * Data-structure : Static array of BSPNode (max 256). Each node stores
 *                  its bounding box, child indices (-1 for leaves), and —
 *                  for leaves — the carved room rectangle inside.
 *
 *                  Map: a flat tile grid (TILE_WALL or TILE_FLOOR) plus
 *                  per-cell glow floats for the carve / partition flashes,
 *                  and an on_path flag for the HOLD-phase blinking route.
 *                  No allocation after init.
 *
 * Rendering      : ASCII only. '#' for walls (rendered only where a wall
 *                  cell touches a floor cell — interior void stays blank
 *                  for the classic roguelike look), '.' for floor.
 *                  During PARTITION, leaf rectangles are also drawn as
 *                  cyan outlines with '+' corners and '-' / '|' edges.
 *                  Per-cell carve_glow paints fresh-cut cells in their
 *                  phase colour (pink for rooms, gold for corridors).
 *
 * Performance    : O(N · log N) where N is the number of map cells —
 *                  splitting halves each region down a tree of depth
 *                  ~log₂(N). Carving and corridor planning are O(N) in
 *                  the carved area. We throttle each phase independently
 *                  (build_per_tick, carve_per_tick, corridor_per_tick)
 *                  so the spectacle plays at human-readable speed.
 *
 * References     :
 *
 *   CONCEPTS — BSP trees & procedural dungeons
 *   • Fuchs, H., Kedem, Z. & Naylor, B. (1980) — "On Visible Surface
 *     Generation by A Priori Tree Structures", *SIGGRAPH '80*,
 *     Computer Graphics 14(3):124-133.  The origin of the BSP tree
 *     (for hidden-surface removal; later repurposed for level layout).
 *   • Buck, Jamis (2014) — "Rooms and Mazes: A Procedural Dungeon
 *     Generator", journal.stuffwithstuff.com/2014/12/21/rooms-and-mazes/.
 *     The canonical BSP-dungeon write-up this demo follows.
 *   • RogueBasin — "Basic BSP Dungeon Generation",
 *     roguebasin.com/index.php?title=Basic_BSP_Dungeon_generation.
 *     The split/place/connect recipe in three steps.
 *   • Shaker, N., Togelius, J. & Nelson, M. J. (2016) — *Procedural
 *     Content Generation in Games*, Springer.  Survey placing BSP among
 *     the family of dungeon/level generators.
 *
 *   PATHFINDING — the HOLD-phase glowing route
 *   • Moore, E. F. (1959) — "The shortest path through a maze",
 *     *Proc. Int. Symp. on the Theory of Switching*, 285-292.  The origin
 *     of breadth-first shortest-path search (dun_plan_path).
 *
 *   RENDERING — ASCII intensity ramps & terminal output
 *   • Bourke, P. (1997) — "Character representation of grey-scale images",
 *     paulbourke.net/dataformats/asciiart/.  The intensity→glyph idea
 *     behind mapping a decaying glow to a brightness ramp.
 *   • Padala, P. (2005) — "NCURSES Programming HOWTO", TLDP
 *     (tldp.org/HOWTO/NCURSES-Programming-HOWTO/).  The colour-pair,
 *     attribute, and double-buffered draw model used in §3/§7.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a dungeon of distinct rooms with controlled connectivity,
 * don't try to design rooms directly. Instead, RECURSIVELY CHOP the
 * map into smaller rectangles, then drop a room inside each smallest
 * piece, then walk corridors between the rooms following the chopping
 * tree itself. The tree structure of the chops automatically becomes
 * the connection graph — every room reaches every other room because
 * the tree is connected.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a sheet of paper. Cut it in half (one slice). Cut each half
 * in half again. Repeat until pieces are small enough. Now in each
 * piece, draw a smaller rectangle (the room). To wire up the rooms,
 * for every cut you made (in REVERSE order), draw a line between
 * SOMETHING-on-the-left and SOMETHING-on-the-right of that cut.
 * That line becomes a corridor. The cut order determines the corridor
 * order; the tree of cuts becomes the connection graph.
 *
 * Three layers in the visual:
 *   1. CYAN outlined rectangles = current set of leaves in the BSP
 *      tree. As splits happen, big rectangles vanish and are replaced
 *      by their two children. By the end of PARTITION, you see ~30–80
 *      small cyan rectangles tiling the map.
 *   2. PINK '#' / '.' painting = rooms being carved, one cell at a
 *      time, inside each leaf. Each room is smaller than its leaf so
 *      walls remain on the leaf boundary.
 *   3. GOLD '#' / '.' painting = L-corridors being walked from one
 *      room's centre to a sibling's. Two segments per corridor.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. nodes[0] = the whole map. The build queue holds just the
 *     root. Map is all TILE_WALL.
 *  2. PARTITION STEP (one operation):
 *     a. Pop node N from the build queue.
 *     b. If N is too small to split (both dims < 2 × MIN_LEAF) OR a
 *        random stop fires: leave it as a leaf. Goto next iteration.
 *     c. Otherwise pick split axis: the longer dimension biased; ties
 *        broken randomly. Pick a random split position in
 *        [MIN_LEAF, dim − MIN_LEAF].
 *     d. Create two child nodes covering the two halves; record the
 *        child indices on N. Push children onto the build queue.
 *  3. When the build queue empties: place a random room inside every leaf
 *     (random margin so the room doesn't touch the leaf edges).
 *     Transition to CARVE.
 *  4. CARVE STEP: pop a room cell from the carve queue, set it to
 *     TILE_FLOOR with pink carve_glow. When the queue empties: plan
 *     corridors and transition to CORRIDORS.
 *  5. CORRIDOR PLAN: post-order traversal of the BSP tree. At each
 *     non-leaf node N, pick one room from N.left's subtree and one
 *     from N.right's subtree (random representatives), append the
 *     L-path between them to the corridor queue.
 *  6. CORRIDOR STEP: pop a cell from the corridor queue, set TILE_FLOOR
 *     with gold carve_glow. When empty: transition to HOLD.
 *  7. HOLD: BFS a route between two opposite-corner rooms and blink it
 *     forever; the user presses `n` to generate the next map (goto 1).
 *
 * KEY FORMULAS
 * ────────────
 *  Split axis bias               : if w > 1.4·h → vertical line
 *                                  if h > 1.4·w → horizontal line
 *                                  else random
 *  Split position (vertical cut) : x = MIN_LEAF + rand() %
 *                                      (w − 2·MIN_LEAF)
 *  Room placement (in leaf b)    : margin = rand() % MAX_MARGIN
 *                                  room = inset(b, margin)
 *                                  width  ≥ MIN_ROOM_W
 *                                  height ≥ MIN_ROOM_H
 *  Subtree representative        : recurse to a leaf via random child
 *                                  → return room centre
 *  L-corridor                    : (x1,y1) → (x2,y2) is two segments
 *                                  EITHER (x1,y1)→(x2,y1)→(x2,y2)
 *                                  OR     (x1,y1)→(x1,y2)→(x2,y2)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SPLIT POSITION. If MIN_LEAF is too large relative to the leaf
 *    being split, max − min in the random call goes ≤ 0 and rand() %
 *    becomes a divide-by-zero or wraps to 0/garbage. Always check
 *    `if (max <= min) return;` BEFORE the modulo.
 *
 *  • ROOM MARGIN. Likewise, MAX_MARGIN must be small enough that the
 *    leaf can fit MIN_ROOM_W × MIN_ROOM_H + 2·MAX_MARGIN. If a leaf
 *    is too narrow, fall back to placing the room with margin 0/1
 *    rather than failing — the player just gets a room that fills
 *    its leaf.
 *
 *  • CORRIDOR THROUGH WALL. An L-corridor between two rooms passes
 *    through the leaves' shared boundary by definition (it's how
 *    they connect). The corridor cells are set to TILE_FLOOR
 *    UNCONDITIONALLY — overwriting whatever was there. Don't try to
 *    "respect" the partition lines or the corridor will fail to
 *    cross.
 *
 *  • DEPTH BOUND. Without a depth cap, deeply unbalanced trees can
 *    push past MAX_NODES. Cap the recursion: stop splitting when
 *    n_nodes + 2 > MAX_NODES, treat the leaf as terminal.
 *
 *  • RAND(): for small ranges it's biased, but for dungeon variety
 *    that bias is invisible. Don't bother with rejection sampling
 *    unless you're benchmarking statistical uniformity.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • After PARTITION: every leaf has at least MIN_LEAF in both dims.
 *    Smaller leaves mean the split-stop guard is loose.
 *  • After CARVE: every leaf has exactly one TILE_FLOOR rectangle
 *    inside it; no FLOOR outside any leaf. (Corridors break this
 *    later — that's by design.)
 *  • After CORRIDORS: every room is reachable from every other room.
 *    Run a BFS from any FLOOR cell; if FLOOR cells exist that BFS
 *    doesn't reach, a corridor failed to connect.
 *  • Visual: the PARTITION cyan outlines must TILE the map exactly —
 *    no overlap, no gaps. If you see gaps, child rectangles aren't
 *    inheriting the parent's bounds correctly (off-by-one on
 *    `child.x = parent.x + split`).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — SIMULATION / EFFECTS / LOGIC / DELAYS / RENDER ─────── *
 *
 * One rule keeps the layers honest: a function's SIGNATURE says whether it
 * can change the dungeon.
 *   • non-const pointer (Dungeon*, Scene*, App*) → an EFFECT; may mutate.
 *   • const pointer / by-value                   → a pure READ (a query
 *                                                   or the renderer).
 *
 * SIMULATION / EFFECTS — the generation, all behind `Dungeon *` (§5):
 *   dun_split_step     one BSP partition: split a leaf into two children
 *   dun_place_rooms    drop a room in every leaf, queue its cells
 *   dun_carve_step     pop one room cell → TILE_FLOOR (+ glow)
 *   dun_plan_corridors queue L-paths joining sibling subtrees
 *   dun_corridor_step  pop one corridor cell → TILE_FLOOR (+ glow)
 *   dun_plan_path      BFS the HOLD-phase glowing route
 *   dun_reset          wipe back to one wall-filled leaf
 *   The Scene state machine (§6) is the ONE place that sequences these.
 *
 * LOGIC — stateless decisions (no Dungeon mutation), used by the effects:
 *   split_axis_vertical  pick the cut axis from a leaf's proportions
 *   dun_pick_room_dim    pick a room dimension inside a leaf
 *   glow_tier            map a glow magnitude to a colour-ramp tier
 *
 * DELAYS — the only time-bending, all in main() (§8):
 *   sim accumulator    advances the generation at sim_fps Hz
 *   per-tick pacing    build/carve/corridor_per_tick reveal speed (+/-)
 *   frame cap          caps the render at RENDER_FPS
 * (HOLD has no timer: the finished map holds until the user presses `n`.)
 *
 * PERFORMANCE — modest by design: everything is static (no malloc after
 *   start); the BSP is inherently O(N log N); the only per-frame scan is
 *   tile_floor_neighbours for wall visibility/shading.  There is no heavy
 *   data structure to hide behind a name here — the cost is just the grid.
 *
 * RENDERING (§7) is PURE: scene_draw / screen_draw read a const Scene and
 * write only the terminal.  Read main() top to bottom and you can see when
 * each effect and delay fires — and that the draw at the end touches none.
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
    /* Map size in cells (= terminal cells; 1:1 mapping, no §4). */
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    MIN_MAP_W         =  16,    /* smallest usable dungeon width         */
    MIN_MAP_H         =   8,    /* smallest usable dungeon height        */
    HUD_ROWS          =   2,    /* rows reserved: top data bar + hint    */

    /* Minimum leaf dimensions — leaves below this don't split.
     * Smaller MIN_LEAF → more rooms; larger → fewer, bigger rooms. */
    MIN_LEAF_W        =  10,
    MIN_LEAF_H        =   6,

    /* Room placement margin range — random inset from leaf edges. */
    MIN_ROOM_W        =   4,
    MIN_ROOM_H        =   3,
    MAX_ROOM_MARGIN   =   3,

    /* Static BSP tree array bound. With MIN_LEAF=10×6 on a 200×56 map
     * the deepest tree has ~7 levels → 256 leaves comfortably fits. */
    MAX_NODES         = 512,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,
    RENDER_FPS        =  60,    /* screen refresh cap (the frame DELAY)  */
    DT_CAP_MS         = 100,    /* clamp a long frame gap (spiral guard) */

    /* Per-tick operation rates per phase. Build is slow (you want to
     * SEE each split); carve and corridor are fast (visible reveal but
     * not a slog). User can scale via +/-. */
    BUILD_STEPS_MIN   =   1,
    BUILD_STEPS_DEF   =   1,
    BUILD_STEPS_MAX   =  64,

    CARVE_STEPS_DEF   =  12,
    CORRIDOR_STEPS_DEF=   8,
    PACE_MAX          = 256,    /* cap on carve/corridor cells per tick  */

    FPS_UPDATE_MS     = 500,

    N_THEMES          =   5,
    GLOW_TIERS        =   4,        /* glow-ramp resolution (hot→cool)   */

    /* Color pair indices.  Each glowing effect owns a GLOW_TIERS ramp so a
     * decaying glow renders as a smooth fade (white-hot → signature colour
     * → cool) instead of one flat colour.  HUD/HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* '#' walls (attr-shaded by torchlight) */
    PAIR_FLOOR        =   4,        /* '.' floors                            */
    PAIR_ROOM_0       =   5,        /* +0..+3 room-dig ramp                  */
    PAIR_COR_0        =   9,        /* +0..+3 corridor-dig ramp              */
    PAIR_PART_0       =  13,        /* +0..+3 partition-outline ramp         */
    PAIR_PATH_0       =  17,        /* +0..+3 HOLD solution-path glow ramp   */
};

/* Glow decay rates. Carve glows fast (instant feel), partition slower
 * (so the BSP outlines stay readable through the build phase). */
#define CARVE_GLOW_DECAY    3.5f
#define PART_GLOW_DECAY     1.5f
#define GLOW_THRESHOLD      0.05f

/* HOLD-phase animation: a blinking solution path + breathing torchlight.
 * (HOLD has no timer — it holds until the user presses `n` for a new map.) */
#define PATH_BLINK_RATE     6.0f      /* path pulse angular rate (rad/sec) */
#define BREATHE_RATE        2.2f      /* floor/wall breathe rate (rad/sec) */
#define BREATHE_BASE        0.65f     /* breathe = BASE + AMP·sin(...)     */
#define BREATHE_AMP         0.35f     /*   ranges [BASE-AMP, BASE+AMP]     */
#define BREATHE_FLOOR_HI    0.75f     /* breathe above this brightens floor */
#define BREATHE_WALL_LO     0.40f     /* breathe below this dims walls       */

/* Wall torchlight: a wall is brighter the more floor it hugs (depth cue). */
#define TORCH_LIT_NB        5         /* ≥ this many floor neighbours → bold */
#define TORCH_DIM_NB        2         /* < this many → dim; 0 → invisible    */
#define FLAGSTONE_MOD       11        /* ~1/this floor cells get a brighter stone */

/* Tile kinds. */
enum { TILE_WALL = 0, TILE_FLOOR = 1 };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CELLS_MAX  (MAP_W_MAX * MAP_H_MAX)

/* clamp v to [lo, hi] — keeps range-limits named at the call site. */
static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

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
 * Theme — a full palette: base wall/floor, plus a GLOW_TIERS ramp (hot→cool)
 * for each glowing effect.  A glow decays smoothly each tick; the renderer
 * maps its magnitude to a ramp tier, so a fresh cut reads white-hot and
 * cools through the theme's signature colour as it fades.  Cycle with t/T.
 * Every tier is in the bright half of the 256-cube so even the coolest is
 * legible against the default background.
 */
typedef struct {
    const char *name;         /* 6-char HUD label                          */
    short wall;               /* settled '#' wall (attr-shaded by torch)   */
    short floor;              /* settled '.' floor                         */
    short room[GLOW_TIERS];   /* room-dig glow ramp:      hot → cool       */
    short cor [GLOW_TIERS];   /* corridor-dig glow ramp                    */
    short part[GLOW_TIERS];   /* partition-outline glow ramp               */
    short path[GLOW_TIERS];   /* HOLD solution-path glow ramp              */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "EMBER ", 246, 67,
      {231,218,212,175}, {231,229,220,208}, {231,159,117, 74}, {231,227,214,202} },
    { "ARCANE", 103, 61,
      {231,219,177,134}, {231,159, 87, 45}, {231,189,147, 99}, {231,213,171,129} },
    { "TOXIC ", 101, 65,
      {231,191,154,106}, {231,229,220,178}, {231,158,119, 71}, {231,226,190,154} },
    { "ICE   ", 152, 67,
      {231,195,153,111}, {231,159,123, 81}, {231,195,159,123}, {231,159,123, 75} },
    { "MONO  ", 250, 242,
      {255,252,248,245}, {255,251,247,244}, {255,250,246,243}, {255,253,250,246} },
};

/* bind one effect's ramp to GLOW_TIERS consecutive colour pairs. */
static void theme_bind_ramp(int base_pair, const short ramp[GLOW_TIERS])
{
    for (int i = 0; i < GLOW_TIERS; i++)
        init_pair((short)(base_pair + i), ramp[i], -1);
}

/* bind the full 256-colour palette: base wall/floor + four glow ramps. */
static void theme_bind_256(const Theme *t)
{
    init_pair(PAIR_WALL,  t->wall,  -1);
    init_pair(PAIR_FLOOR, t->floor, -1);
    theme_bind_ramp(PAIR_ROOM_0, t->room);
    theme_bind_ramp(PAIR_COR_0,  t->cor);
    theme_bind_ramp(PAIR_PART_0, t->part);
    theme_bind_ramp(PAIR_PATH_0, t->path);
}

/* 8-colour fallback: one flat colour per effect, theme-independent. */
static void theme_bind_8(void)
{
    init_pair(PAIR_WALL,  COLOR_WHITE, -1);
    init_pair(PAIR_FLOOR, COLOR_BLUE,  -1);
    for (int i = 0; i < GLOW_TIERS; i++) {
        init_pair((short)(PAIR_ROOM_0 + i), COLOR_MAGENTA, -1);
        init_pair((short)(PAIR_COR_0  + i), COLOR_YELLOW,  -1);
        init_pair((short)(PAIR_PART_0 + i), COLOR_CYAN,    -1);
        init_pair((short)(PAIR_PATH_0 + i), COLOR_YELLOW,  -1);
    }
}

static void theme_apply(int idx)
{
    const Theme *t = &THEMES[idx];
    if (COLORS >= 256) theme_bind_256(t);
    else               theme_bind_8();
}

/* map a decaying glow in (0,1] to a ramp tier: 0 = freshest / hottest. */
static int glow_tier(float g)
{
    if (g > 0.66f) return 0;
    if (g > 0.40f) return 1;
    if (g > 0.18f) return 2;
    return GLOW_TIERS - 1;
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
/* §4  geometry — Point / Rect: the algorithm is rectangle subdivision     */
/* ===================================================================== */

/*
 * The BSP algorithm is pure rectangle geometry: the map is a Rect, every
 * split halves a Rect into two, and a room is a smaller Rect inset inside a
 * leaf.  Point is a single map coordinate (a room centre, a corridor
 * waypoint).  Both are value types — passed and returned by value, never
 * mutated through a pointer — so they read like the maths they model.
 */
typedef struct {
    int x, y;        /* map cell: column, row (0-based, top-left origin) */
} Point;
typedef struct {
    int x, y;        /* top-left corner cell                            */
    int w, h;        /* size in cells; spans x..x+w-1, y..y+h-1          */
} Rect;

/* centre cell of a rectangle (integer; biases toward top-left on even dims). */
static Point rect_center(Rect r) { return (Point){ r.x + r.w / 2, r.y + r.h / 2 }; }

/* split a rect into left/right halves at column offset `at` (a vertical cut). */
static void rect_split_v(Rect b, int at, Rect *lo, Rect *hi)
{
    *lo = (Rect){ b.x,      b.y, at,       b.h };
    *hi = (Rect){ b.x + at, b.y, b.w - at, b.h };
}

/* split a rect into top/bottom halves at row offset `at` (a horizontal cut). */
static void rect_split_h(Rect b, int at, Rect *lo, Rect *hi)
{
    *lo = (Rect){ b.x, b.y,      b.w, at       };
    *hi = (Rect){ b.x, b.y + at, b.w, b.h - at };
}

/* ===================================================================== */
/* §5  dungeon — data model, pure reads, and generation effects           */
/*                                                                        */
/* Signature convention: `Dungeon *` = EFFECT (mutates the world);        */
/* `const Dungeon *` = a pure read.  Structs and reads come first, then   */
/* the effects that grow the dungeon phase by phase.                      */
/* ===================================================================== */

/*
 * Fifo — a fixed-capacity first-in-first-out queue of cell/node indices.
 * The generation reveals work breadth-first (Moore 1959): leaves awaiting a
 * split, room cells awaiting carving, corridor cells awaiting digging.  All
 * three are the same shape, so one queue type serves them all (and the BFS
 * in dun_plan_path borrows a drained one as its frontier).
 *
 *   WHY no wrap-around (head/tail just advance): each index is pushed at
 *   most once per phase and the queue is cleared between phases, so the
 *   monotonic cursors never exceed CELLS_MAX — a plain bump allocator is
 *   simpler and faster than a circular buffer here.
 */
typedef struct {
    int items[CELLS_MAX]; /* backing store; sized for the largest user (carve) */
    int head;             /* next index to POP  — consumer cursor              */
    int tail;             /* next free slot to PUSH — producer cursor; count = tail-head */
} Fifo;

static void fifo_clear(Fifo *q)       { q->head = q->tail = 0; }
static bool fifo_empty(const Fifo *q) { return q->head >= q->tail; }
static int  fifo_pop  (Fifo *q)       { return q->items[q->head++]; }
static void fifo_push (Fifo *q, int v)        /* guarded: drop, never overflow */
{
    if (q->tail < CELLS_MAX) q->items[q->tail++] = v;
}

/*
 * BSPNode — one node of the Binary Space Partition tree: a rectangular
 * region that is either split into two children or left as a leaf holding
 * one room.  The tree IS the dungeon's structure — the recursive halving
 * (Fuchs, Kedem & Naylor 1980) tiles the map without overlap, and because
 * the tree is connected, joining sibling rooms at every internal node
 * yields a fully-connected dungeon (Buck 2014).
 *
 * Indices, not pointers: children are array indices into Dungeon.nodes[]
 * so the whole tree lives in one static array (no malloc, stable across
 * the run) and a child reference is just an int.
 */
typedef struct {
    Rect bounds;        /* the region this node covers (whole map at root)   */
    int  left, right;   /* child indices into nodes[]; both -1 ⇒ this is a
                         * leaf (the split test: left == -1)                  */
    bool has_room;      /* leaf only: true once dun_place_rooms drops a room  */
    Rect room;          /* the carved room, inset inside bounds; valid only
                         * when this is a leaf AND has_room (internal nodes
                         * have no room — they exist just to route corridors) */
} BSPNode;

/*
 * Dungeon — the whole generated world, data only.  WHICH phase is running
 * is held by the Scene enum (§6); this struct just carries the state those
 * phases read and mutate.  Four families of data:
 *   1. the raster   — the tile grid the player sees, plus per-cell glow
 *   2. the BSP tree — the partition that decides where rooms/corridors go
 *   3. the queues   — breadth-first work lists driving the animated reveal
 *   4. the path     — the HOLD-phase BFS route + counters for the HUD
 * Everything is static (CELLS_MAX / MAX_NODES) — no malloc after start,
 * so a reset is just re-initialising fields in place.
 */
typedef struct {
    /* ── raster (the visible map) ── */
    int      w, h;                      /* grid size in cells               */
    int      total_cells;               /* w·h, cached (loop bound)         */
    uint8_t  tiles[CELLS_MAX];          /* TILE_WALL or TILE_FLOOR per cell  */
    float    carve_glow[CELLS_MAX];     /* fresh-cut flash, decays each tick */
    float    part_glow[CELLS_MAX];      /* partition-outline glow, decays    */
    uint8_t  part_type[CELLS_MAX];      /* outline glyph: '+', '-', '|', 0   */

    /* ── BSP tree (the structure) ── */
    BSPNode  nodes[MAX_NODES];          /* the partition tree (indices, §5)  */
    int      n_nodes;                   /* nodes allocated so far (bump idx) */
    int      root_idx;                  /* whole-map node (always 0)         */

    /* Per-phase reveal queues (BFS order).  Each holds the indices still
     * to process for that phase; pop one (or several) per tick. */
    /* ── queues (the animated reveal, BFS order) ── */
    Fifo     build;      /* PARTITION: leaf node indices awaiting a split  */
    Fifo     carve;      /* CARVE:     room cell indices awaiting a floor   */
    Fifo     corridor;   /* CORRIDORS: corridor cell indices to dig        */

    /* ── HOLD-phase solution path (a blinking route across the dungeon) ── */
    uint8_t  on_path[CELLS_MAX];        /* 1 = cell lies on the route       */
    int      bfs_prev[CELLS_MAX];       /* BFS back-pointers: -2 unvisited,
                                         * -1 origin, else predecessor index */
    int      path_len;                  /* cells on the route (HUD/debug)    */

    /* ── live counters for the HUD ── */
    int      leaf_count;                /* leaves so far (grows in PARTITION)*/
    int      room_count;                /* rooms placed                      */
    int      corridor_count;            /* L-corridors planned               */
} Dungeon;

static inline int dun_idx(const Dungeon *d, int x, int y) { return y * d->w + x; }
static inline bool dun_in_bounds(const Dungeon *d, int x, int y)
{
    return x >= 0 && x < d->w && y >= 0 && y < d->h;
}

/* ── generation effects (mutate the Dungeon; sequenced by §6) ─────────── */

/* clear the visible map: every cell back to wall, no glow, no outline,
 * no path. */
static void dun_clear_raster(Dungeon *d)
{
    for (int i = 0; i < d->total_cells; i++) {
        d->tiles[i]      = TILE_WALL;
        d->carve_glow[i] = 0.0f;
        d->part_glow[i]  = 0.0f;
        d->part_type[i]  = 0;
        d->on_path[i]    = 0;
    }
    d->path_len = 0;
}

static void dun_reset(Dungeon *d, int w, int h)
{
    d->w = w;  d->h = h;  d->total_cells = w * h;

    /* 1. blank the raster. */
    dun_clear_raster(d);

    /* 2. seed the tree with one leaf covering the whole map. */
    d->n_nodes  = 0;
    d->root_idx = d->n_nodes++;
    d->nodes[d->root_idx] = (BSPNode){
        .bounds = { 0, 0, w, h }, .left = -1, .right = -1, .has_room = false
    };

    /* 3. PARTITION starts with that leaf queued; other queues empty. */
    fifo_clear(&d->build);
    fifo_push(&d->build, d->root_idx);
    fifo_clear(&d->carve);
    fifo_clear(&d->corridor);

    /* 4. reset the HUD counters (root counts as one leaf). */
    d->leaf_count     = 1;
    d->room_count     = 0;
    d->corridor_count = 0;
}

/*
 * dun_paint_outline — paint part_glow + per-cell glyph kind on the
 * boundary of node `idx`.
 *
 * Each outline cell is one of:
 *   '+' corner (one of the 4 leaf corners)
 *   '-' horizontal edge (top or bottom row, interior x)
 *   '|' vertical edge (left or right column, interior y)
 *
 * Adjacent leaves share edges; both leaves paint the same kind onto
 * the shared cell, so the type is consistent regardless of paint order.
 *
 * Called whenever a node is created so the BSP tree grows visibly in
 * cyan during the PARTITION phase.
 */
static void dun_paint_outline(Dungeon *d, int idx)
{
    Rect b = d->nodes[idx].bounds;
    int x0 = b.x;
    int y0 = b.y;
    int x1 = b.x + b.w - 1;
    int y1 = b.y + b.h - 1;
    if (!dun_in_bounds(d, x0, y0) || !dun_in_bounds(d, x1, y1)) return;

    /* Top + bottom rows: '+' on corners, '-' on the interior. */
    for (int x = x0; x <= x1; x++) {
        int top = dun_idx(d, x, y0);
        int bot = dun_idx(d, x, y1);
        d->part_glow[top] = 1.0f;
        d->part_glow[bot] = 1.0f;
        char kind = (x == x0 || x == x1) ? '+' : '-';
        d->part_type[top] = (uint8_t)kind;
        d->part_type[bot] = (uint8_t)kind;
    }
    /* Left + right columns: '+' on corners (already set above), '|' on
     * the interior. Don't overwrite '+' corners — only set if currently
     * empty or already '|'. */
    for (int y = y0 + 1; y < y1; y++) {
        int lf = dun_idx(d, x0, y);
        int rt = dun_idx(d, x1, y);
        d->part_glow[lf] = 1.0f;
        d->part_glow[rt] = 1.0f;
        d->part_type[lf] = (uint8_t)'|';
        d->part_type[rt] = (uint8_t)'|';
    }
}

/*
 * split_axis_vertical — stateless cut-axis decision (LOGIC, no mutation):
 * prefer the longer dimension (1.4× bias), forced when only one axis can
 * split, random on a near-square tie.  Returns true for a vertical cut
 * (split x → a left/right pair).
 */
static bool split_axis_vertical(int w, int h, bool can_w, bool can_h)
{
    if (can_w && !can_h) return true;
    if (can_h && !can_w) return false;
    if (w * 10 > h * 14) return true;     /* clearly wider  → cut x */
    if (h * 10 > w * 14) return false;    /* clearly taller → cut y */
    return (rand() & 1);                  /* near-square: coin flip */
}

/*
 * dun_split_step — perform one split operation on the next queued node.
 *
 * Returns true if work happened (split or rejected as final leaf),
 * false if the build queue is empty (PARTITION phase done).
 *
 * Splitting decision:
 *   • If both dims < 2·MIN_LEAF  → can't split, mark as final leaf.
 *   • Else pick axis (split_axis_vertical), then a split position in
 *     [MIN_LEAF, dim − MIN_LEAF], create two children, queue them.
 */
static bool dun_split_step(Dungeon *d)
{
    if (fifo_empty(&d->build)) return false;

    int  idx = fifo_pop(&d->build);
    Rect b   = d->nodes[idx].bounds;

    /* Refuse to split if both children would be < MIN_LEAF. */
    bool can_split_w = b.w >= 2 * MIN_LEAF_W;
    bool can_split_h = b.h >= 2 * MIN_LEAF_H;
    if (!can_split_w && !can_split_h) return true;   /* final leaf */

    /* Refuse if there's no room in the nodes[] array for two more. */
    if (d->n_nodes + 2 > MAX_NODES) return true;

    /* Decide the cut axis (pure logic), pick a split offset, halve the rect. */
    bool vertical = split_axis_vertical(b.w, b.h, can_split_w, can_split_h);
    int  span     = vertical ? b.w : b.h;
    int  min_leaf = vertical ? MIN_LEAF_W : MIN_LEAF_H;
    int  min_pos  = min_leaf, max_pos = span - min_leaf;
    if (max_pos <= min_pos) return true;             /* can't fit two leaves */
    int  at = min_pos + rand() % (max_pos - min_pos);

    Rect lo, hi;
    if (vertical) rect_split_v(b, at, &lo, &hi);
    else          rect_split_h(b, at, &lo, &hi);

    int left_idx  = d->n_nodes++;
    int right_idx = d->n_nodes++;
    d->nodes[left_idx]  = (BSPNode){ .bounds = lo, .left = -1, .right = -1, .has_room = false };
    d->nodes[right_idx] = (BSPNode){ .bounds = hi, .left = -1, .right = -1, .has_room = false };

    /* Hook children into the parent; queue them for further splitting. */
    d->nodes[idx].left  = left_idx;
    d->nodes[idx].right = right_idx;
    d->leaf_count += 1;         /* +2 children, -1 split parent → +1 net */
    fifo_push(&d->build, left_idx);
    fifo_push(&d->build, right_idx);

    /* Paint partition outlines so the BSP tree grows visibly in cyan. */
    dun_paint_outline(d, left_idx);
    dun_paint_outline(d, right_idx);

    return true;
}

/*
 * dun_place_rooms — at end of PARTITION phase, drop a room inside
 * every leaf and queue the room cells for animated carving.
 *
 * Room placement: random margin from each leaf edge in [0, MAX_ROOM_MARGIN].
 * Falls back to margin=1 / margin=0 when leaf is too tight.
 */
static int dun_pick_room_dim(int leaf_dim, int min_room, int max_margin)
{
    int margin_total = 2 * max_margin;
    int max_room = leaf_dim - margin_total;
    if (max_room < min_room) max_room = min_room;
    if (max_room > leaf_dim - 2) max_room = leaf_dim - 2;
    if (max_room < min_room) max_room = min_room;
    int range = max_room - min_room + 1;
    if (range <= 0) range = 1;
    return min_room + rand() % range;
}

/*
 * room_inside_leaf — a room Rect randomly inset inside a leaf (LOGIC, pure):
 * pick a size (≥ MIN_ROOM, leaving a ≥1-cell wall border), then a random
 * offset so the room floats anywhere inside that border.
 */
static Rect room_inside_leaf(Rect b)
{
    int rw = dun_pick_room_dim(b.w, MIN_ROOM_W, MAX_ROOM_MARGIN);
    int rh = dun_pick_room_dim(b.h, MIN_ROOM_H, MAX_ROOM_MARGIN);
    if (rw > b.w - 2) rw = b.w - 2;          /* keep a 1-cell wall border */
    if (rh > b.h - 2) rh = b.h - 2;
    if (rw < 1) rw = 1;
    if (rh < 1) rh = 1;

    int max_x_off = b.w - rw - 1; if (max_x_off < 1) max_x_off = 1;
    int max_y_off = b.h - rh - 1; if (max_y_off < 1) max_y_off = 1;
    return (Rect){ b.x + 1 + rand() % max_x_off,
                   b.y + 1 + rand() % max_y_off, rw, rh };
}

/* queue a room's cells (raster order) for the CARVE reveal. */
static void dun_queue_room_cells(Dungeon *d, Rect room)
{
    for (int ry = 0; ry < room.h; ry++)
        for (int rx = 0; rx < room.w; rx++) {
            int cx = room.x + rx, cy = room.y + ry;
            if (dun_in_bounds(d, cx, cy))
                fifo_push(&d->carve, dun_idx(d, cx, cy));
        }
}

static void dun_place_rooms(Dungeon *d)
{
    for (int i = 0; i < d->n_nodes; i++) {
        BSPNode *n = &d->nodes[i];
        if (n->left != -1) continue;        /* internal node — no room */
        n->room     = room_inside_leaf(n->bounds);
        n->has_room = true;
        d->room_count++;
        dun_queue_room_cells(d, n->room);
    }
}

/*
 * dun_carve_step — pop one cell from the carve queue, set TILE_FLOOR
 * with pink carve_glow flash. Returns true if a cell was carved.
 */
static bool dun_carve_step(Dungeon *d)
{
    if (fifo_empty(&d->carve)) return false;
    int idx = fifo_pop(&d->carve);
    d->tiles[idx]      = TILE_FLOOR;
    d->carve_glow[idx] = 1.0f;
    return true;
}

/*
 * dun_subtree_room_centre — centre of a randomly chosen room from the
 * subtree rooted at `idx` (recurses to a leaf, coin-flipping at each
 * branch).  Corridor planning joins a left-subtree room to a right one.
 */
static Point dun_subtree_room_centre(const Dungeon *d, int idx)
{
    int cur = idx;
    while (d->nodes[cur].left != -1)
        cur = (rand() & 1) ? d->nodes[cur].left : d->nodes[cur].right;
    return rect_center(d->nodes[cur].room);
}

/*
 * dun_queue_corridor_segment — queue every cell along the axis-aligned
 * segment from `a` to `b` (must share a row or a column).  Step direction
 * is the sign of the delta.
 */
static void dun_queue_corridor_segment(Dungeon *d, Point a, Point b)
{
    int dx = (b.x > a.x) ? 1 : (b.x < a.x) ? -1 : 0;
    int dy = (b.y > a.y) ? 1 : (b.y < a.y) ? -1 : 0;
    int x = a.x, y = a.y;
    while (1) {
        if (dun_in_bounds(d, x, y)) fifo_push(&d->corridor, dun_idx(d, x, y));
        if (x == b.x && y == b.y) break;
        x += dx;
        y += dy;
    }
}

/*
 * dun_plan_corridors — post-order traversal of the BSP tree; at every
 * internal node, queue an L-corridor joining a left-subtree room centre to
 * a right one.  The L bends at a random `elbow` (horizontal-then-vertical
 * or vice versa) for visual variety.
 */
static void dun_plan_corridors(Dungeon *d, int idx)
{
    const BSPNode *n = &d->nodes[idx];
    if (n->left == -1) return;

    dun_plan_corridors(d, n->left);
    dun_plan_corridors(d, n->right);

    Point a     = dun_subtree_room_centre(d, n->left);
    Point b     = dun_subtree_room_centre(d, n->right);
    Point elbow = (rand() & 1) ? (Point){ b.x, a.y }   /* go across, then down */
                               : (Point){ a.x, b.y };  /* go down, then across */
    dun_queue_corridor_segment(d, a, elbow);
    dun_queue_corridor_segment(d, elbow, b);
    d->corridor_count++;
}

/*
 * dun_corridor_step — pop one cell from the corridor queue, set TILE_FLOOR
 * with a glow.  The renderer reads the Scene phase (CORRIDORS) to colour
 * the glow gold; after it fades the cell is plain floor.  Returns true if a
 * cell was dug.
 */
static bool dun_corridor_step(Dungeon *d)
{
    if (fifo_empty(&d->corridor)) return false;
    int idx = fifo_pop(&d->corridor);
    d->tiles[idx]      = TILE_FLOOR;
    d->carve_glow[idx] = 1.0f;
    return true;
}

/*
 * dun_extreme_rooms — the two leaf-room centres nearest opposite corners
 * (min / max of room-centre x+y), the endpoints of the showcased route.
 * Returns false if there are no rooms.
 */
static bool dun_extreme_rooms(const Dungeon *d, Point *a, Point *b)
{
    int  lo = 0, hi = 0;
    bool first = true;
    for (int i = 0; i < d->n_nodes; i++) {
        const BSPNode *n = &d->nodes[i];
        if (n->left != -1 || !n->has_room) continue;   /* leaves with a room */
        Point c   = rect_center(n->room);
        int   key = c.x + c.y;
        if (first || key < lo) { lo = key; *a = c; }    /* nearest top-left  */
        if (first || key > hi) { hi = key; *b = c; }    /* nearest bot-right */
        first = false;
    }
    return !first;
}

/*
 * dun_bfs — breadth-first flood from `start` over floor cells (4-connected),
 * recording each cell's predecessor in bfs_prev[] (−2 unvisited, −1 origin).
 * Stops early once `goal` is dequeued.  Reuses the drained corridor Fifo's
 * buffer as the frontier.  This is the classic shortest-path search
 * (Moore 1959): the first time goal is reached is via a shortest route.
 */
static void dun_bfs(Dungeon *d, int start, int goal)
{
    for (int i = 0; i < d->total_cells; i++) d->bfs_prev[i] = -2;  /* unvisited */
    int *frontier = d->corridor.items;
    int qh = 0, qt = 0;
    d->bfs_prev[start] = -1;                       /* -1 = the origin */
    frontier[qt++] = start;

    static const int DX4[4] = { 1, -1, 0, 0 };
    static const int DY4[4] = { 0, 0, 1, -1 };
    while (qh < qt) {
        int cur = frontier[qh++];
        if (cur == goal) break;
        int cx = cur % d->w, cy = cur / d->w;
        for (int k = 0; k < 4; k++) {
            int nx = cx + DX4[k], ny = cy + DY4[k];
            if (!dun_in_bounds(d, nx, ny)) continue;
            int ni = dun_idx(d, nx, ny);
            if (d->tiles[ni] != TILE_FLOOR || d->bfs_prev[ni] != -2) continue;
            d->bfs_prev[ni] = cur;                 /* remember how we got here */
            frontier[qt++] = ni;
        }
    }
}

/* walk bfs_prev[] back from goal to the origin, flagging on_path[]. */
static void dun_mark_path(Dungeon *d, int goal)
{
    if (d->bfs_prev[goal] == -2) return;           /* unreachable (shouldn't happen) */
    for (int c = goal; c != -1; c = d->bfs_prev[c]) {
        d->on_path[c] = 1;
        d->path_len++;
    }
}

/*
 * dun_plan_path — flag the HOLD-phase glowing route, in three named steps:
 * pick the corner-most room pair, BFS the shortest floor route between
 * them, then trace it back into on_path[].
 */
static void dun_plan_path(Dungeon *d)
{
    for (int i = 0; i < d->total_cells; i++) d->on_path[i] = 0;
    d->path_len = 0;

    Point a, b;
    if (!dun_extreme_rooms(d, &a, &b)) return;     /* no rooms — nothing to do */

    int start = dun_idx(d, a.x, a.y);
    int goal  = dun_idx(d, b.x, b.y);
    dun_bfs(d, start, goal);
    dun_mark_path(d, goal);
}

/* ===================================================================== */
/* §6  scene — the phase state machine that sequences the effects         */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   PARTITION   — split nodes one per build step. Transitions to CARVE
 *                 when build queue empties (and dun_place_rooms is run).
 *   CARVE       — pop room cells from the carve Fifo, set TILE_FLOOR.
 *                 Transitions to CORRIDORS when the carve Fifo empties
 *                 (and dun_plan_corridors is run from root).
 *   CORRIDORS   — pop corridor cells, set TILE_FLOOR. Transitions to
 *                 HOLD when corridor queue empties.
 *   HOLD        — finished; the path blinks until the user presses `n`,
 *                 which regenerates and returns to PARTITION.
 *
 * Carve_glow gets coloured differently depending on the active phase
 * (pink in CARVE, gold in CORRIDORS). The Scene state IS the phase,
 * so the renderer reads s->state to pick the right pair.
 */
typedef enum {
    SCENE_PARTITION = 0,
    SCENE_CARVE     = 1,
    SCENE_CORRIDORS = 2,
    SCENE_HOLD      = 3,
} SceneState;

/*
 * Scene — the running instance: the generated Dungeon plus the control
 * state that drives and presents it.  `state` selects which effect the
 * tick runs; the per-tick counts are pacing knobs; theme/anim_time are
 * presentation.  This is the single thing the renderer reads (const) and
 * the loop advances (non-const).
 */
typedef struct {
    Dungeon     d;                  /* the world being generated (§5)        */
    SceneState  state;              /* which phase / which effect runs now   */
    bool        paused;             /* freeze stepping; render keeps running */
    int         build_per_tick;     /* splits   revealed per tick (slow)     */
    int         carve_per_tick;     /* room cells dug per tick (fast)        */
    int         corridor_per_tick;  /* corridor cells dug per tick (fast)    */
    float       anim_time;          /* monotonic; drives the HOLD path blink */
    int         theme;              /* index into THEMES (t/T)               */
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    dun_reset(&s->d, mw, mh);
    s->state = SCENE_PARTITION;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused           = false;
    s->build_per_tick   = BUILD_STEPS_DEF;
    s->carve_per_tick   = CARVE_STEPS_DEF;
    s->corridor_per_tick= CORRIDOR_STEPS_DEF;
    scene_reset(s, mw, mh);
}

/* exponentially fade the carve + partition glow buffers by one tick. */
static void scene_decay_glows(Dungeon *d, float dt)
{
    float carve_d = expf(-CARVE_GLOW_DECAY * dt);
    float part_d  = expf(-PART_GLOW_DECAY  * dt);
    for (int i = 0; i < d->total_cells; i++) {
        d->carve_glow[i] *= carve_d;
        d->part_glow[i]  *= part_d;
    }
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    s->anim_time += dt;          /* drives the HOLD path-blink + breathe */
    scene_decay_glows(&s->d, dt);

    switch (s->state) {

    case SCENE_PARTITION:
        for (int i = 0; i < s->build_per_tick; i++) {
            if (!dun_split_step(&s->d)) {
                /* Partition done — place rooms then move to CARVE. */
                dun_place_rooms(&s->d);
                s->state = SCENE_CARVE;
                break;
            }
        }
        break;

    case SCENE_CARVE:
        for (int i = 0; i < s->carve_per_tick; i++) {
            if (!dun_carve_step(&s->d)) {
                /* Carve done — plan corridors and move to CORRIDORS. */
                dun_plan_corridors(&s->d, s->d.root_idx);
                s->state = SCENE_CORRIDORS;
                break;
            }
        }
        break;

    case SCENE_CORRIDORS:
        for (int i = 0; i < s->corridor_per_tick; i++) {
            if (!dun_corridor_step(&s->d)) {
                /* Dungeon complete — trace the glowing path and hold here. */
                dun_plan_path(&s->d);
                s->state = SCENE_HOLD;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        /* Finished: the path blinks indefinitely (anim_time keeps advancing
         * above).  No auto-reset — the user presses `n` for the next map. */
        break;
    }
}

/* ===================================================================== */
/* §7  render — PURE: const Scene → screen (mutates nothing but the term)  */
/* ===================================================================== */

/*
 * Same canonical pattern as other procedural files §7 / framework.c:
 *   erase → scene_draw → HUD → wnoutrefresh(stdscr) → doupdate
 *
 * ASCII glyphs only:
 *   '#' wall (rendered only where adjacent to floor — interior void
 *       stays blank for the classic roguelike aesthetic),
 *   '.' floor,
 *   '+' partition outline corner,
 *   '-' partition outline horizontal,
 *   '|' partition outline vertical.
 *
 * Per-cell render priority (highest wins):
 *   blinking path (HOLD) > carve glow > partition glow > settled floor/wall
 *   Each glowing layer maps its intensity through glow_tier() to a colour
 *   ramp (white-hot → signature → cool), so the exponential glow decay
 *   reads as a fading ember.  The carve ramp is the room palette during
 *   CARVE and the corridor palette during CORRIDORS — one buffer, two
 *   phases.  During HOLD, on_path[] cells pulse through the path ramp.
 */
/* Screen — the ncurses terminal extent, refreshed on init and resize. */
typedef struct {
    int cols, rows;   /* terminal width / height in character cells */
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

/*
 * tile_floor_neighbours — how many of the 8 neighbours of (x,y) are
 * TILE_FLOOR (0..8).  Drives two things in the renderer:
 *   • visibility — a wall with 0 floor neighbours is interior void, blank.
 *   • torchlight — a wall hugging MORE floor reads as "more lit" (a corner
 *     embraced by a room glows brighter than a lone edge), giving depth.
 */
static int tile_floor_neighbours(const Dungeon *d, int x, int y)
{
    int count = 0;
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (dun_in_bounds(d, nx, ny) &&
                d->tiles[dun_idx(d, nx, ny)] == TILE_FLOOR) count++;
        }
    return count;
}

/* flagstone texture: a cheap sparse hash so ~1/FLAGSTONE_MOD floor cells
 * read a touch brighter, like worn stone. */
static bool flagstone_at(int x, int y) { return (x * 3 + y * 7) % FLAGSTONE_MOD == 0; }

/* what to paint at one cell (a "skip" flag covers the interior void). */
typedef struct { int pair; attr_t attr; char glyph; bool draw; } CellDraw;

/*
 * cell_appearance — the renderer's per-cell decision (pure), by priority:
 *   blinking path (HOLD) > fresh-cut ember > partition outline > floor > wall
 * Each glow layer maps its magnitude through glow_tier() to its ramp; the
 * HOLD `blink`/`breathe` phases modulate brightness.  draw=false ⇒ leave
 * the cell blank (interior void with no floor nearby).
 */
static CellDraw cell_appearance(const Scene *s, int x, int y,
                                bool holding, float blink, float breathe)
{
    const Dungeon *d   = &s->d;
    int     idx = dun_idx(d, x, y);
    float   cg  = d->carve_glow[idx], pg = d->part_glow[idx];
    uint8_t k   = d->tiles[idx];
    int     dig = (s->state == SCENE_CORRIDORS) ? PAIR_COR_0 : PAIR_ROOM_0;
    int     t;

    if (holding && d->on_path[idx]) {                 /* blinking path     */
        t = glow_tier(blink);
        return (CellDraw){ PAIR_PATH_0 + t, (t <= 1) ? A_BOLD : A_NORMAL, '*', true };
    }
    if (cg > GLOW_THRESHOLD) {                         /* fresh-cut ember   */
        t = glow_tier(cg);
        return (CellDraw){ dig + t, (t <= 1) ? A_BOLD : A_NORMAL,
                           (k == TILE_FLOOR) ? '.' : '#', true };
    }
    if (pg > GLOW_THRESHOLD && k == TILE_WALL && d->part_type[idx]) { /* outline */
        t = glow_tier(pg);
        return (CellDraw){ PAIR_PART_0 + t, (t == 0) ? A_BOLD : A_NORMAL,
                           (char)d->part_type[idx], true };
    }
    if (k == TILE_FLOOR) {                             /* settled floor     */
        attr_t a = flagstone_at(x, y) ? A_NORMAL : A_DIM;
        if (holding && breathe > BREATHE_FLOOR_HI) a = A_NORMAL;
        return (CellDraw){ PAIR_FLOOR, a, '.', true };
    }
    /* wall: visible only where it hugs floor, torch-shaded by how much. */
    int nb = tile_floor_neighbours(d, x, y);
    if (nb == 0) return (CellDraw){ 0, A_NORMAL, ' ', false };  /* interior void */
    attr_t a = (nb >= TORCH_LIT_NB) ? A_BOLD
             : (nb >= TORCH_DIM_NB) ? A_NORMAL : A_DIM;
    if (holding && breathe < BREATHE_WALL_LO) a = A_DIM;
    return (CellDraw){ PAIR_WALL, a, '#', true };
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Dungeon *d = &s->d;

    /* Centre the map in the play area (top + bottom rows reserved for HUD). */
    int gx0 = (cols - d->w) / 2;             if (gx0 < 0) gx0 = 0;
    int gy0 = ((rows - HUD_ROWS) - d->h) / 2 + 1; if (gy0 < 1) gy0 = 1;

    /* HOLD: the path pulses (blink) and the dungeon breathes under torchlight. */
    bool  holding = (s->state == SCENE_HOLD);
    float blink   = 0.5f + 0.5f * sinf(s->anim_time * PATH_BLINK_RATE);  /* → [0,1] */
    float breathe = holding ? BREATHE_BASE + BREATHE_AMP * sinf(s->anim_time * BREATHE_RATE)
                            : 1.0f;

    for (int y = 0; y < d->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < d->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            CellDraw c = cell_appearance(s, x, y, holding, blink, breathe);
            if (!c.draw) continue;
            attron(COLOR_PAIR(c.pair) | c.attr);
            mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
            attroff(COLOR_PAIR(c.pair) | c.attr);
        }
    }
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

/* the phase label shown in the HUD (fixed-width so the bar doesn't jiggle). */
static const char *phase_label(const Scene *s)
{
    if (s->paused)                     return "PAUSED   ";
    if (s->state == SCENE_PARTITION)   return "PARTITION";
    if (s->state == SCENE_CARVE)       return "CARVING  ";
    if (s->state == SCENE_CORRIDORS)   return "CORRIDOR ";
    return "HOLD     ";
}

/* format the top-bar readout: phase, theme (current/total), live counts,
 * the per-tick pacing, and timing. */
static void hud_status_line(char *buf, size_t n, const Scene *s,
                            double fps, int sim_fps)
{
    const Dungeon *d = &s->d;
    snprintf(buf, n,
             " BSP_DUNGEON  %s  theme:%s (%d/%d)  leaves:%-3d  rooms:%-3d  "
             "cor:%-3d  pace:%d/%d/%d  %5.1f fps  %3d Hz ",
             phase_label(s), THEMES[s->theme].name, s->theme + 1, N_THEMES,
             d->leaf_count, d->room_count, d->corridor_count,
             s->build_per_tick, s->carve_per_tick, s->corridor_per_tick,
             fps, sim_fps);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    char data[180];
    hud_status_line(data, sizeof data, s, fps, sim_fps);

    const char *keys =       /* bottom bar — every interactive key */
        " q:quit  spc:pause  n:new map  t/T:theme  +/-:speed  [/]:Hz ";

    hud_bar(0,            sc->cols, PAIR_HUD,  data);
    hud_bar(sc->rows - 1, sc->cols, PAIR_HINT, keys);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app — orchestration: input → effects → delay → render              */
/* ===================================================================== */

/*
 * FpsMeter — a rolling frame-rate average over FPS_UPDATE_MS, so the HUD
 * number is readable rather than flickering every frame.  Banks elapsed
 * time + frame count, divides once per window, then resets.
 */
typedef struct {
    int64_t accum_ns;   /* nanoseconds banked since the last readout    */
    int     frames;     /* frames banked since the last readout         */
    double  value;      /* last computed fps, shown in the HUD          */
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

/*
 * App — the running PROCESS: the Scene plus the loop's own bookkeeping
 * (terminal size, sim pacing, the timing accumulator, the fps meter, and
 * the signal flags).  Split from Scene because these describe the program,
 * not the generated dungeon.  One file-scope instance (g_app) so the async
 * signal handlers can reach the flags.
 */
typedef struct {
    Scene                 scene;       /* the running generation (§6)         */
    Screen                screen;      /* terminal extent                     */
    int                   sim_fps;     /* generation tick rate (Hz); render is
                                        * capped separately at RENDER_FPS      */
    int                   map_w, map_h;/* dungeon size (≤ terminal, capped)   */
    int64_t               sim_accum;   /* fixed-step accumulator: banks real
                                        * dt, spent in whole sim ticks (delay) */
    FpsMeter              fps;          /* rolling render-rate readout          */
    volatile sig_atomic_t running;     /* main-loop flag, 0 = quit.  volatile
                                        * sig_atomic_t: set by SIGINT/SIGTERM   */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH, drained atop loop    */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    /* dungeon = terminal minus the two HUD rows, clamped to the buffers. */
    app->map_w = clampi(app->screen.cols,            MIN_MAP_W, MAP_W_MAX);
    app->map_h = clampi(app->screen.rows - HUD_ROWS, MIN_MAP_H, MAP_H_MAX);
}

/* step the reveal pacing one notch (build slow-doubles so each split stays
 * visible; carve/corridor double freely), each clamped to its range. */
static void scene_pace_faster(Scene *s)
{
    if (s->build_per_tick < BUILD_STEPS_MAX) s->build_per_tick *= 2;
    s->build_per_tick    = clampi(s->build_per_tick,        BUILD_STEPS_MIN, BUILD_STEPS_MAX);
    s->carve_per_tick    = clampi(s->carve_per_tick    * 2, 1, PACE_MAX);
    s->corridor_per_tick = clampi(s->corridor_per_tick * 2, 1, PACE_MAX);
}
static void scene_pace_slower(Scene *s)
{
    s->build_per_tick    = clampi(s->build_per_tick    / 2, BUILD_STEPS_MIN, BUILD_STEPS_MAX);
    s->carve_per_tick    = clampi(s->carve_per_tick    / 2, 1, PACE_MAX);
    s->corridor_per_tick = clampi(s->corridor_per_tick / 2, 1, PACE_MAX);
}

/* nudge the render/sim Hz one step, clamped. */
static void app_nudge_fps(App *app, int delta)
{
    app->sim_fps = clampi(app->sim_fps + delta, SIM_FPS_MIN, SIM_FPS_MAX);
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->sim_accum   = 0;
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'n': case 'N':                 /* generate the next map */
        scene_reset(s, app->map_w, app->map_h);
        break;
    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        break;
    case 'T':
        s->theme = (s->theme - 1 + N_THEMES) % N_THEMES;
        theme_apply(s->theme);
        break;
    case '=': case '+': scene_pace_faster(s);            break;
    case '-':           scene_pace_slower(s);            break;
    case ']':           app_nudge_fps(app, +SIM_FPS_STEP); break;
    case '[':           app_nudge_fps(app, -SIM_FPS_STEP); break;
    default: break;
    }
    return true;
}

/* TIME: dt since the last frame, clamped so one stall can't fast-forward
 * the generation (spiral-of-death guard). */
static int64_t frame_delta(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    return dt > DT_CAP_MS * NS_PER_MS ? DT_CAP_MS * NS_PER_MS : dt;
}

/* EFFECTS: drain the fixed-step accumulator — advance the generation at
 * sim_fps Hz, decoupled from the render rate. */
static void app_advance(App *app, int64_t dt)
{
    int64_t tick   = TICK_NS(app->sim_fps);
    float   dt_sec = (float)tick / (float)NS_PER_SEC;
    app->sim_accum += dt;
    while (app->sim_accum >= tick) {
        scene_tick(&app->scene, dt_sec);
        app->sim_accum -= tick;
    }
}

/* DELAY: sleep so the screen refreshes at most RENDER_FPS. */
static void app_pace_frame(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(TICK_NS(RENDER_FPS) - elapsed);
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

    int64_t frame_time = clock_ns();
    app->sim_accum     = 0;

    /*
     * Fixed-step main loop — each iteration reads as five named steps:
     *   INPUT   poll a key (may fire §5 effects: next map, theme, pacing)
     *   TIME    frame_delta() — dt since last frame, clamped
     *   EFFECTS app_advance() — drain the sim accumulator at sim_fps Hz
     *   DELAY   app_pace_frame() — cap the render at RENDER_FPS
     *   RENDER  screen_draw() — pure paint, mutates nothing
     */
    while (app->running) {
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        int ch = getch();                                  /* INPUT   */
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;

        int64_t dt = frame_delta(&frame_time);             /* TIME    */

        app_advance(app, dt);                              /* EFFECTS */
        fps_meter_tick(&app->fps, dt);
        app_pace_frame(frame_time, dt);                    /* DELAY   */

        screen_draw(&app->screen, &app->scene,             /* RENDER  */
                    app->fps.value, app->sim_fps);
        screen_present();
    }

    screen_free(&app->screen);
    return 0;
}
