/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * life.c — Conway's Game of Life and Rule Variants
 *
 * Toroidal grid; rules expressed as B/S bitmasks (bit N set = birth/survive
 * when neighbour count is N).  Six variants cycle with n/p:
 *
 *   Conway    B3/S23      — the classic
 *   HighLife  B36/S23     — gliders that self-replicate
 *   Day&Night B3678/S34678— symmetric; same rule alive/dead
 *   Seeds     B2/S        — every cell dies; explosions from any pair
 *   Morley    B368/S245   — chaotic; moving structures
 *   2×2       B36/S125    — tiles of 2×2 blocks grow and divide
 *
 * Seeding:  r random · g glider · G Gosper gun · e R-pentomino · a acorn
 *
 * Layout:
 *   Row  0         — data HUD (rule, preset index, generation, speed)
 *   Rows 1 … n-5   — grid (toroidal)
 *   Rows n-4 … n-2 — scrolling population histogram (3 rows, bar chart)
 *   Row  n-1       — action HUD (key hints)
 *
 * Keys:
 *   n / p       next / previous rule
 *   t / T       next / previous colour theme (live cells coloured by crowding)
 *   r           random seed (~30 % density)
 *   g           single glider at centre
 *   G           Gosper glider gun
 *   e           R-pentomino
 *   a           Acorn
 *   + / =       more steps per frame
 *   - / _       fewer steps per frame
 *   space       pause / resume
 *   q / Q       quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/life.c -o life -lncurses
 *
 * Sections: §1 config+types  §2 performance  §3 logic
 *           §4 simulation  §5 render  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Conway's Game of Life and variants using B/S rule bitmasks.
 *                  Birth/survival encoded as bit N = 1 in a uint16 mask where
 *                  N is the neighbour count (0–8).  This allows checking any
 *                  rule with a single bitshift: born = (birth>>n)&1.
 *                  Double-buffered toroidal grid; update is O(rows × cols).
 *
 * Physics        : Emergent complexity from local rules (Conway 1970):
 *                  despite only two states and 9 neighbour values, Life
 *                  supports stable structures (still-lifes), oscillators,
 *                  spaceships (gliders), and Turing-complete computation
 *                  (Gosper gun produces infinite glider stream).
 *
 * Data-structure : Population histogram ring buffer — last HIST_LEN
 *                  generation counts stored; drawn as a 3-row bar chart
 *                  of '#' (filled) / '.' (empty) cells, taller = busier.
 *
 * Performance    : Per-step cost: O(rows × cols) neighbour-count evaluation.
 *                  Steps per frame adjustable (1–STEPS_MAX) to allow fast-fwd
 *                  to interesting evolved states (e.g., Gosper gun period 30).
 *
 * References     : Concept —
 *                  [1] Gardner, "Mathematical Games: …Conway's new solitaire
 *                      game 'life'", Scientific American 223, Oct 1970.
 *                      The origin; first published the B3/S23 rule.
 *                  [2] Berlekamp, Conway & Guy, "Winning Ways for Your
 *                      Mathematical Plays", vol. 4, 2nd ed. 2004 — Life's
 *                      theory, oscillators/spaceships, Turing-completeness.
 *                  [3] LifeWiki — conwaylife.com/wiki.  Catalogue of the
 *                      patterns seeded here: glider, Gosper glider gun,
 *                      R-pentomino, Acorn (both famous methuselahs).
 *                  [4] "Life-like cellular automaton" (LifeWiki) + Eppstein,
 *                      "Growth and Decay in Life-Like CAs", 2010 — the B/S
 *                      rule family: HighLife, Day&Night, Seeds, Morley, 2x2.
 *                  [5] Wolfram, "A New Kind of Science", 2002 — CA rule
 *                      space and how local rules yield global complexity.
 *                  Rendering —
 *                  [6] Padala, "NCURSES Programming HOWTO", TLDP.
 *                      Colour pairs, erase/refresh, non-blocking input.
 *                  [7] Bourke, "Colour Ramping for Data Visualisation",
 *                      paulbourke.net/texture_colour/colourramp — mapping a
 *                      scalar (here: neighbour count) onto a colour ramp.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell looks at its 8 neighbours, counts how many are alive, and
 * asks one question — "with this many neighbours, do I live next tick?"
 * The answer is read out of a 9-bit lookup: bit N of the BIRTH mask
 * decides what an empty cell does, bit N of the SURVIVE mask decides
 * what a live cell does.  Different masks = different universes.
 * Conway's B3/S23 is just one of countless rules; flipping bits gets
 * you HighLife, Day&Night, Seeds, Morley, 2x2 — same code, same loop,
 * different cosmologies.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine an infinite chessboard wrapped into a torus.  Each square
 * holds a stone or doesn't.  Once a tick, every square simultaneously
 * peeks at its 8 surrounding squares and decides — based on how
 * crowded it is — whether to keep its stone, drop one, or pick one up.
 * No square ever cheats by reading the new state of a neighbour; the
 * old board is read, the new board is written, and they swap.  Two
 * boards, ping-pong.  That's it.  The astonishing complexity is the
 * universe's bonus, not your code's burden.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Pick rule (Conway, HighLife, …).  Encode birth/survive as 9-bit
 *     masks: bit N set iff cell with N neighbours is born / survives.
 *  2. For each cell (r,c) in the active board: sum the 8 toroidal
 *     neighbours; (r±1, c±1) wrap with `% rows / % cols`.
 *  3. Form bit = 1u << n.  Next state =
 *        cur ? (survive & bit) ? 1 : 0
 *            : (birth   & bit) ? 1 : 0;
 *     Write into the OTHER board.
 *  4. Swap boards (`board.buf = 1 - board.buf`); increment board.gen;
 *     record population into the history ring (history_record).
 *  5. Repeat steps 2-4 `scene.steps` times per frame for fast-forward.
 *  6. Render: draw live cells coloured by neighbour count (board.h rows
 *     under the data HUD), draw the scrolling population histogram (3
 *     rows), data HUD on row 0 and action HUD on the bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  n = sum over 8 neighbours of board.cells[buf][r±1 mod R][c±1 mod C]
 *                                                   neighbour count 0..8
 *  bit = 1u << n                                    9-bit position flag
 *  next = cur ? (S & bit) != 0
 *             : (B & bit) != 0                      Conway's rule
 *  Conway:    B = (1<<3),                           B3 — birth on 3
 *             S = (1<<2)|(1<<3)                     S23 — survive on 2 or 3
 *  toroidal:  rp = (r-1+R) % R, rn = (r+1) % R      no edge cells
 *  hist bar:  level = pop / (R·C) · 2·HIST_ROWS     2 sub-levels per row
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Two-board read/write — if you forget to write into the OTHER
 *    buffer, neighbour counts read partially-updated state and the
 *    simulation collapses (gliders smear, oscillators die).
 *  • Toroidal wrap with C-style %  — `(r-1) % R` is negative for r=0;
 *    must add R before mod, hence `(r-1+R) % R`.
 *  • Seeds rule (B2/S0) — every live cell dies every tick; only
 *    explosions survive.  Looks like everything is broken until you
 *    see the rule name.
 *  • Gosper gun period — the canonical pattern emits one glider every
 *    30 ticks.  At scene.steps=3 you see a glider every 10 frames; at
 *    scene.steps=30 you see one every frame.
 *  • Steps-per-frame * grid size — at MAX_COLS=320, MAX_ROWS=128, and
 *    STEPS_MAX=30, that's ~1.2M cell updates/frame.  Still well under
 *    16ms on modern CPUs.  The histogram walks board.w columns of the
 *    ring (HIST_LEN samples), so board.w must stay ≤ HIST_LEN.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Blinker test (Conway): place 3 cells in a row.  After 1 tick it
 *    must be vertical; after 2 ticks horizontal again.  Period 2.
 *  • Glider test (Conway): seed 'g'.  The 5-cell glider must move
 *    diagonally one cell every 4 ticks.  Toroidal wrap means it
 *    eventually returns to start after gcd(R, C) · 4 ticks.
 *  • Acorn test (Conway): seed 'a'.  Should stabilise after exactly
 *    5206 generations into a soup of 633 cells (one of Life's most
 *    famous methuselahs).
 *  • R-pentomino test: seed 'e'.  Stabilises at gen 1103 with 116
 *    live cells (period-2 oscillators + still-lifes + 6 gliders).
 *  • Seeds test: seed 'r' under Seeds rule.  Population should
 *    explode then crash; histogram shows characteristic spikes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * All data hangs off one Scene (§1), but functions take the NARROWEST type
 * they need, so aggregating on Scene never re-couples the layers:
 *
 *   Layer        Section  Takes                          Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       (scalars)                      nothing — time / sleep
 *   LOGIC        §3       const Board* / count list      NOTHING — pure, no I/O
 *   SIMULATION   §4       Board*, const Rule*, Scene*    Board (cells/buf/gen)
 *   RENDER       §5       const Board/History/Scene*     the terminal only
 *   APP          §6       Scene*                         Scene; drives the above
 *
 *   EFFECTS — the PopulationHistory ring buffer (Scene.history): cosmetic
 *             state read only by draw_histogram (§5).  No logic of its own —
 *             scene_tick records one sample per step as the board advances.
 *   DELAYS  — trivial: 'space' toggles Scene.paused (a flag the tick reads);
 *             the only wait is the fixed-timestep frame cap (§2 / TICK_NS).
 *
 * PER-TICK COMBINE — one place advances state (§6 main), in order:
 *     1. SIMULATION : scene_tick()  — board_step() × steps, recording each
 *                     step's population into the EFFECTS history
 *     2. RENDER     : scene_draw()  — draw_board → draw_histogram → draw_hud
 *     3. PERFORMANCE: sleep until the next TICK_NS boundary
 *
 * USER EVENTS are NOT the tick: keys and resize mutate Scene directly in §6
 * (reseeding via the seed_* resets, rebuilding colours, refitting the board)
 * but none of them call scene_tick()/board_step().
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config + types   (the ONLY place data is declared) ────────────── */

#define TICK_NS     33333333LL
#define MAX_ROWS    128
#define MAX_COLS    320
#define HIST_LEN    512    /* population history ring buffer length        */
#define HIST_ROWS   3      /* screen rows used by histogram                */
#define HUD_TOP     1      /* row 0: data HUD                              */
#define HUD_BOT     1      /* bottom row: action HUD                       */
#define STEPS_DEF   3
#define STEPS_MAX   30
#define LIVE_CHAR   '#'
#define RAMP_LEN    9      /* live-cell colour ramp = one tier per neighbour count 0..8 */

/* ncurses colour-pair IDs.  CP_RAMP..CP_RAMP+8 are the live-cell ramp,
 * indexed by neighbour count (assigned per theme in color_apply); CP_HIST is
 * the histogram, CP_HUD/CP_HINT the two HUD rows.  Numbered from 1 because
 * ncurses reserves pair 0 for the terminal's default colours. */
enum {
    CP_RAMP = 1,                   /* ramp occupies CP_RAMP .. CP_RAMP+RAMP_LEN-1 */
    CP_HIST = CP_RAMP + RAMP_LEN,  /* pairs after the ramp                        */
    CP_HUD,
    CP_HINT
};

/* Rule — a "life-like" cellular-automaton rule in Golly's B/S notation.
 *
 * WHY bitmasks: the transition depends only on the live-neighbour count n
 * (0..8), so each of birth/survive is a 9-bit set — "is n a member?".  Storing
 * them as masks turns the per-cell decision into one shift + AND ((mask>>n)&1),
 * which is why the same inner loop runs every variant unchanged.  E.g. Conway
 * B3/S23: birth = 1<<3, survive = (1<<2)|(1<<3).
 * (Life-like CA family: LifeWiki [4]; Conway's original B3/S23: Gardner [1].) */
typedef struct {
    uint16_t birth;     /* bit n set ⇒ a DEAD cell with n live neighbours is born   */
    uint16_t survive;   /* bit n set ⇒ a LIVE cell with n live neighbours survives  */
    const char *name;   /* HUD label, e.g. "Conway B3/S23"                          */
} Rule;
#define N_RULES 6
static Rule RULES[N_RULES];   /* catalogue, built by rules_init (§4); Scene.rule_idx selects */

/* Theme — a live-cell colour palette, cycled with t/T.
 *
 * WHY a ramp, not one colour: Life cells are binary, so there is no intensity
 * to show — but a cell's live-neighbour count (0..8) is a free local scalar.
 * Mapping that count onto a colour ramp makes crowding visible: lone cells and
 * dense clusters read as different hues, so gliders and still-lifes stand out
 * from soup.  (Scalar→colour ramp: Bourke [7].)
 *
 * Every entry sits in the bright half of the 256-colour space (per project
 * palette rules) so even the lowest tier stays visible on a black terminal. */
typedef struct {
    const char *name;          /* HUD label, e.g. "OCEAN"                       */
    short       ramp[RAMP_LEN]; /* ramp[n] = xterm-256 colour for count n (0..8),
                                * dim→bright as the neighbourhood gets denser    */
} Theme;

static const Theme THEMES[] = {
    { "OCEAN",  {  25,  26,  27,  32,  38,  44,  45,  51, 159 } },
    { "FIRE",   {  88, 124, 160, 166, 196, 202, 208, 214, 226 } },
    { "MATRIX", {  28,  34,  40,  46,  82, 118, 154, 191, 194 } },
    { "AMBER",  {  94, 130, 136, 166, 172, 178, 214, 220, 229 } },
    { "MONO",   { 244, 246, 247, 249, 250, 251, 252, 253, 255 } },
};
#define N_THEMES 5

/* Board — the world: Conway's cellular grid, double-buffered and toroidal.
 *
 * WHY double-buffered: a generation is SIMULTANEOUS — every cell must see its
 * neighbours' OLD values.  So a step reads cells[buf] and writes cells[1-buf],
 * then flips buf; updating in place would let already-written cells corrupt
 * the neighbours still being computed (the classic Life bug — gliders smear).
 *
 * WHY toroidal: neighbour indices wrap modulo h/w (see board_neighbors), so
 * the edges join into a torus — a glider leaving the right side re-enters on
 * the left, with no boundary walls to perturb patterns.  (Conway 1970 [1].) */
typedef struct {
    /* the two boards: cells[buf] live, cells[1-buf] scratch; 0 = dead, 1 = live */
    uint8_t cells[2][MAX_ROWS][MAX_COLS];
    /* which buffer holds the current generation (0 or 1); flips every step */
    int     buf;
    /* generations elapsed since the last reset — shown in the HUD as gen= */
    long    gen;
    /* active region actually simulated, ≤ the fixed MAX backing store; fitted
     * to the terminal each resize so a small window doesn't iterate dead cells */
    int     h, w;
} Board;

/* PopulationHistory — the cosmetic population trail (EFFECTS), plotted by the
 * scrolling histogram.  Write-only from the simulation's view: it records the
 * live-cell count but never feeds back into the rule, so it can be cleared or
 * resized without affecting evolution.
 *
 * WHY a ring buffer: the chart shows only the most recent w columns of an
 * unbounded stream of per-step counts.  A ring overwrites the oldest sample in
 * O(1) with no shifting; `head` marks the next write slot, and the renderer
 * walks backwards from head to lay the newest sample at the right edge. */
typedef struct {
    long samples[HIST_LEN];  /* population (live-cell count) after each recorded step */
    int  head;               /* next write index; wraps modulo HIST_LEN              */
} PopulationHistory;

/* The whole simulation in one aggregate, passed to the §6 orchestrators
 * (init / reset / tick).  Read as a table of contents:
 *   WHAT is simulated — board.  EFFECTS — history (the population trail).
 *   HOW it is driven  — rule_idx + steps-per-frame + paused.
 *   RENDER selection  — theme.   WHERE — rows (terminal height).
 * Width is board.w (a board spans the full terminal width), so there is no
 * separate cols.  Knobs are loose scalars: each is too thin for its own type. */
typedef struct {
    Board             board;     /* WHAT: the cell grid being evolved              */
    PopulationHistory history;   /* EFFECTS: cosmetic population trail (the chart)  */
    int rule_idx;                /* HOW: which rule is live, 0..N_RULES-1 → RULES   */
    int steps;                   /* HOW: generations per frame, 1..STEPS_MAX (fast-fwd) */
    int paused;                  /* HOW: 0/1 — when 1, scene_tick is a no-op         */
    int theme;                   /* RENDER: active palette, 0..N_THEMES-1 → THEMES   */
    int rows;                    /* WHERE: terminal height; bottom HUD sits at rows-1 */
} Scene;

static Scene g_scene = { .steps = STEPS_DEF };

/* Signal-handler ↔ main-loop flags — not Scene data, so kept file-scope. */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

/* ── §2 performance   (frame clock + fixed-timestep cap; the only DELAY) ─ */

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

/* ── §3 logic   (pure decisions: read-only, no I/O — cannot be corrupted) ─ */

/* Toroidal index wrap: map i back into [0,n) so the grid edges join into a
 * torus.  Adding n once suffices because every caller passes i ≥ -n. */
static int wrap(int i, int n) { return (i + n) % n; }

/* Build a B/S bitmask from a list of neighbour counts (bit N per count N). */
static uint16_t rule_mask(const int *ns, int len)
{
    uint16_t m = 0;
    for (int i = 0; i < len; i++) m |= (uint16_t)(1u << ns[i]);
    return m;
}

/* Live-neighbour count (0..8) of cell (r,c) on the live buffer, toroidal.
 * The single neighbour counter — used by both the step and the renderer. */
static int board_neighbors(const Board *b, int r, int c)
{
    int rn = wrap(r + 1, b->h), rp = wrap(r - 1, b->h);
    int cn = wrap(c + 1, b->w), cp = wrap(c - 1, b->w);
    return b->cells[b->buf][rp][cp] + b->cells[b->buf][rp][c] +
           b->cells[b->buf][rp][cn] + b->cells[b->buf][r ][cp] +
           b->cells[b->buf][r ][cn] + b->cells[b->buf][rn][cp] +
           b->cells[b->buf][rn][c ] + b->cells[b->buf][rn][cn];
}

/* Apply the B/S rule: does a cell with `n` live neighbours live next gen?
 * One shift+AND tests membership in the survive set (live cell) or birth set
 * (dead cell) — the entire transition, identical across every variant. */
static int cell_next_state(const Rule *ru, int alive, int n)
{
    uint16_t bit = (uint16_t)(1u << n);
    return alive ? ((ru->survive & bit) ? 1 : 0)
                 : ((ru->birth   & bit) ? 1 : 0);
}

/* Normalise a population to a histogram bar height in [0, HIST_ROWS].  The
 * ×2 makes a half-full board reach a full bar, so the busy low range stays
 * readable (counts above 50 % all saturate). */
static int bar_level(long pop, long max_pop)
{
    int level = (int)((float)pop / (float)max_pop * HIST_ROWS * 2.0f);
    return level > HIST_ROWS ? HIST_ROWS : level;
}

/* ── §4 simulation   (advances state: mutates the Board, resets the Scene) ─ */

/* Rule-table sugar: B(3,6) → mask with bits 3 and 6 set (via rule_mask, §3). */
#define B(...)  rule_mask((int[]){__VA_ARGS__}, \
                          sizeof((int[]){__VA_ARGS__})/sizeof(int))

static void rules_init(void)
{
    RULES[0] = (Rule){ B(3),         B(2,3),         "Conway B3/S23"          };
    RULES[1] = (Rule){ B(3,6),       B(2,3),         "HighLife B36/S23"       };
    RULES[2] = (Rule){ B(3,6,7,8),   B(3,4,6,7,8),   "Day&Night B3678/S34678" };
    RULES[3] = (Rule){ B(2),         0,              "Seeds B2/S"             };
    RULES[4] = (Rule){ B(3,6,8),     B(2,4,5),       "Morley B368/S245"       };
    RULES[5] = (Rule){ B(3,6),       B(1,2,5),       "2x2 B36/S125"           };
}

/* Reset the simulation to empty: clear the board and restart the history.
 * Invoked by the seed_* user events, not by the per-tick combine. */
static void scene_clear(Scene *sc)
{
    memset(sc->board.cells, 0, sizeof sc->board.cells);
    sc->board.gen = 0;
    memset(sc->history.samples, 0, sizeof sc->history.samples);
    sc->history.head = 0;
}

/* Stamp a list of (dr,dc) live cells onto the board, toroidally, at (r0,c0). */
static void place(Board *b, const int cells[][2], int n, int r0, int c0)
{
    for (int i = 0; i < n; i++) {
        int r = wrap(r0 + cells[i][0], b->h);
        int c = wrap(c0 + cells[i][1], b->w);
        b->cells[b->buf][r][c] = 1;
    }
}

static void seed_random(Scene *sc)
{
    scene_clear(sc);
    Board *b = &sc->board;
    for (int r = 0; r < b->h; r++)
        for (int c = 0; c < b->w; c++)
            b->cells[b->buf][r][c] = (rand() % 10 < 3) ? 1 : 0;
}

static void seed_glider(Scene *sc)
{
    scene_clear(sc);
    static const int G[][2] = {{0,1},{1,2},{2,0},{2,1},{2,2}};
    place(&sc->board, G, 5, sc->board.h/2 - 1, sc->board.w/2 - 1);
}

static void seed_rpentomino(Scene *sc)
{
    scene_clear(sc);
    static const int P[][2] = {{0,1},{0,2},{1,0},{1,1},{2,1}};
    place(&sc->board, P, 5, sc->board.h/2 - 1, sc->board.w/2 - 1);
}

static void seed_acorn(Scene *sc)
{
    scene_clear(sc);
    static const int A[][2] = {{0,1},{1,3},{2,0},{2,1},{2,4},{2,5},{2,6}};
    place(&sc->board, A, 7, sc->board.h/2 - 1, sc->board.w/2 - 3);
}

static void seed_gosper(Scene *sc)
{
    scene_clear(sc);
    static const int GG[][2] = {
        {0,24},
        {1,22},{1,24},
        {2,12},{2,13},{2,20},{2,21},{2,34},{2,35},
        {3,11},{3,15},{3,20},{3,21},{3,34},{3,35},
        {4, 0},{4, 1},{4,10},{4,16},{4,20},{4,21},
        {5, 0},{5, 1},{5,10},{5,14},{5,16},{5,17},{5,22},{5,24},
        {6,10},{6,16},{6,24},
        {7,11},{7,15},
        {8,12},{8,13},
    };
    int roff = sc->board.h / 4;
    int coff = (sc->board.w > 40) ? 5 : 0;
    place(&sc->board, GG, 36, roff, coff);
}

/* One Life generation under rule `ru`: count neighbours, apply B/S, swap
 * buffers.  THE state advance.  Returns the new population. */
static long board_step(Board *b, const Rule *ru)
{
    int next = 1 - b->buf;
    long pop = 0;

    for (int r = 0; r < b->h; r++)
        for (int c = 0; c < b->w; c++) {
            int n = board_neighbors(b, r, c);                       /* live neighbours 0..8 */
            uint8_t nxt = (uint8_t)cell_next_state(ru, b->cells[b->buf][r][c], n);
            b->cells[next][r][c] = nxt;
            pop += nxt;
        }

    b->buf = next;   /* swap: the buffer we just wrote becomes live */
    b->gen++;
    return pop;
}

/* Push one population sample into the ring, advancing the write head. */
static void history_record(PopulationHistory *h, long pop)
{
    h->samples[h->head] = pop;
    h->head = (h->head + 1) % HIST_LEN;
}

/* The per-tick combine: advance `steps` generations under the active rule,
 * recording each step's population into the EFFECTS history. */
static void scene_tick(Scene *sc)
{
    if (sc->paused) return;
    const Rule *ru = &RULES[sc->rule_idx];
    for (int s = 0; s < sc->steps; s++) {
        long pop = board_step(&sc->board, ru);
        history_record(&sc->history, pop);
    }
}

/* ── §5 render   (state → screen: reads state, mutates only the terminal) ─ */

static void color_apply(const Theme *th)
{
    start_color();
    use_default_colors();
    /* live-cell ramp from the given theme (or flat green in 8-colour mode) */
    for (int i = 0; i < RAMP_LEN; i++) {
        short col = (COLORS >= 256) ? th->ramp[i] : COLOR_GREEN;
        init_pair((short)(CP_RAMP + i), col, -1);
    }
    init_pair(CP_HIST, (COLORS >= 256) ? 240 : COLOR_WHITE,  -1);
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* top data row    */
    init_pair(CP_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* bottom key hints */
}

static void draw_board(const Board *b)
{
    for (int r = 0; r < b->h; r++) {
        const uint8_t *row = b->cells[b->buf][r];
        for (int c = 0; c < b->w - 1; c++) {
            if (!row[c]) { mvaddch(r + HUD_TOP, c, ' '); continue; }
            int pair = CP_RAMP + board_neighbors(b, r, c);   /* colour by crowding */
            attron(COLOR_PAIR(pair));
            mvaddch(r + HUD_TOP, c, (chtype)(unsigned char)LIVE_CHAR);
            attroff(COLOR_PAIR(pair));
        }
    }
}

static void draw_histogram(const PopulationHistory *h, const Board *b)
{
    long max_pop = (long)b->h * b->w;
    if (max_pop == 0) return;

    attron(COLOR_PAIR(CP_HIST));
    for (int c = 0; c < b->w - 1; c++) {
        /* sample shown at column c: walk back (w-1-c) steps from the head */
        int idx = (h->head - (b->w - 1 - c) + HIST_LEN * 2) % HIST_LEN;
        int level = bar_level(h->samples[idx], max_pop);
        for (int hr = 0; hr < HIST_ROWS; hr++) {
            int sr = HUD_TOP + b->h + hr;                     /* histogram screen row */
            char ch = (level >= HIST_ROWS - hr) ? '#' : '.';  /* bar grows upward     */
            mvaddch(sr, c, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(CP_HIST));
}

/* Draw one HUD row left-aligned at `row`, clipped to `width` so it can never
 * overflow onto the grid. */
static void draw_hud_row(int row, int pair, int width, const char *text)
{
    char buf[128];
    snprintf(buf, sizeof buf, "%s", text);
    if ((int)strlen(buf) > width) buf[width] = '\0';
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Top row = live parameters (data); bottom row = key hints (actions). */
static void draw_hud(const Scene *sc)
{
    char data[128];
    snprintf(data, sizeof data,
             " Life  %-22s [%d/%d]  %s  gen=%ld  spd=%dx  %s",
             RULES[sc->rule_idx].name, sc->rule_idx + 1, N_RULES,
             THEMES[sc->theme].name, sc->board.gen, sc->steps,
             sc->paused ? "PAUSED" : "running");
    draw_hud_row(0, CP_HUD, sc->board.w, data);
    draw_hud_row(sc->rows - 1, CP_HINT, sc->board.w,
                 " q:quit  spc:pause  n/p:rule  t:theme  r:rand  g:glider  G:gun  e:pento  a:acorn  +/-:spd ");
}

/* Reads the whole scene (const → cannot mutate state, so no re-coupling). */
static void scene_draw(const Scene *sc)
{
    draw_board(&sc->board);
    draw_histogram(&sc->history, &sc->board);
    draw_hud(sc);
}

/* ── §6 app   (terminal setup, user events, and the per-tick combine) ──── */

static void screen_init(const Scene *sc)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_apply(&THEMES[sc->theme]);
}

/* USER EVENT: terminal resized — refit the board and reseed (not a tick). */
static void screen_resize(Scene *sc)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    sc->rows    = rows;
    sc->board.w = cols;
    sc->board.h = rows - HIST_ROWS - HUD_TOP - HUD_BOT;
    if (sc->board.h < 1) sc->board.h = 1;
    seed_random(sc);
    erase();
}

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

static void install_signals(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
}

/* Consume a pending resize: reset ncurses, then refit the board. */
static void handle_resize(Scene *sc)
{
    g_need_resize = 0;
    endwin(); refresh();
    screen_resize(sc);
}

/* USER EVENT: dispatch one key press — mutates the scene, never a tick. */
static void handle_key(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': g_running = 0; break;
    case ' ':           sc->paused ^= 1; break;
    case 'n':
        sc->rule_idx = (sc->rule_idx + 1) % N_RULES;
        seed_random(sc); break;
    case 'p':
        sc->rule_idx = (sc->rule_idx - 1 + N_RULES) % N_RULES;
        seed_random(sc); break;
    case 't':
        sc->theme = (sc->theme + 1) % N_THEMES;
        color_apply(&THEMES[sc->theme]); break;
    case 'T':
        sc->theme = (sc->theme - 1 + N_THEMES) % N_THEMES;
        color_apply(&THEMES[sc->theme]); break;
    case 'r':  seed_random(sc);     break;
    case 'g':  seed_glider(sc);     break;
    case 'G':  seed_gosper(sc);     break;
    /* 'p' already used for prev-rule; use 'e' for R-pentomino */
    case 'e':  seed_rpentomino(sc); break;
    case 'a':  seed_acorn(sc);      break;
    case '+': case '=':
        if (sc->steps < STEPS_MAX) sc->steps++;
        break;
    case '-': case '_':
        if (sc->steps > 1) sc->steps--;
        break;
    }
}

/* RENDER: clear, draw the scene, flush in one diff write. */
static void frame_render(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    install_signals();
    atexit(cleanup);
    rules_init();
    screen_init(&g_scene);
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    screen_resize(&g_scene);

    long long next = clock_ns();
    while (g_running) {
        if (g_need_resize) handle_resize(&g_scene);

        int ch;
        while ((ch = getch()) != ERR) handle_key(&g_scene, ch);

        scene_tick(&g_scene);                  /* advance simulation (per-tick combine) */
        frame_render(&g_scene);                /* board + histogram + HUD → screen      */

        next += TICK_NS;                       /* PERFORMANCE: fixed-timestep frame cap */
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
