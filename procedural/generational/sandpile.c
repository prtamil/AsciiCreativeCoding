/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sandpile.c — Bak-Tang-Wiesenfeld Abelian Sandpile
 *
 * Grains rain onto the centre cell. Any cell holding ≥ 4 grains topples:
 * it sheds 4 grains, one to each cardinal neighbour. Grains that fall off
 * the border are lost. Toppling cascades until every cell holds 0–3 grains.
 *
 * DEMO: A glowing point at screen centre. Grain by grain a pile grows into
 *       a perfectly 4-fold-symmetric MANDALA — concentric diamonds of
 *       intricate, self-similar texture that nobody designed: it falls out
 *       of the toppling rule alone (self-organised criticality). The pattern
 *       blooms outward, fills the screen, and settles. Press v to watch a
 *       single avalanche ripple outward in rings. Press r to start over.
 *
 * Colour by grain count (the mandala's palette):
 *   0 grains — blank
 *   1 grain  — '.'  dim   (sparse outer field)
 *   2 grains — 'o'  mid   (the body)
 *   3 grains — '#'  bright (the near-critical ridges — the skeleton)
 *   ≥4 (mid-avalanche, 'v' mode) — '*' hot flash
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         reset (re-centre, clear the pile)
 *   t / T     next / previous colour theme
 *   v         avalanche-wave view (drop one grain, watch its cascade ripple)
 *   + / =     more grains per frame (faster bloom)
 *   - / _     fewer grains per frame
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra sandpile.c -o sandpile -lncurses
 *
 * Sections: §1 config  §2 performance  §3 simulation
 *           §4 render   §5 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Abelian Sandpile Model (Bak, Tang & Wiesenfeld, 1987).
 *                  A grain is dropped on a chosen cell.  If any cell has
 *                  ≥ 4 grains, it "topples": loses 4 grains and gives 1 to
 *                  each of its 4 (von Neumann) neighbours.  Toppling can
 *                  cascade into an "avalanche" affecting many cells.
 *
 * Physics/Math   : The sandpile is a canonical example of Self-Organised
 *                  Criticality (SOC): without any tuning, the system
 *                  spontaneously evolves to a critical state where avalanche
 *                  size distributions follow a power law P(s) ∝ s^(−τ) with
 *                  τ ≈ 1.0 for 2D abelian sandpile.
 *                  "Abelian" means the final state is independent of the order
 *                  in which topplings are processed — a deep mathematical
 *                  property proved by Dhar (1990).
 *
 * Performance    : O(avalanche_size) per grain drop.  Average avalanche size
 *                  grows as the pile approaches the critical state.  At
 *                  criticality, large avalanches are rare but unbounded —
 *                  the expected cost per drop diverges logarithmically.
 *
 * References     : Concept —
 *                  [1] Bak, Tang & Wiesenfeld (1987) — "Self-Organized
 *                      Criticality: an explanation of 1/f noise", Phys. Rev.
 *                      Lett. 59, 381.  The origin of SOC and this model.
 *                  [2] Dhar (1990) — "Self-organised critical state of sandpile
 *                      automaton models", Phys. Rev. Lett. 64, 1613.  Proves
 *                      the Abelian property (order-independent final state).
 *                  [3] Bak — "How Nature Works: The Science of Self-Organised
 *                      Criticality" (1996).  The accessible book-length intro.
 *                  [4] Levine & Propp — "What is … a Sandpile?", AMS Notices
 *                      57(8), 2010.  The fractal limit shape (the mandala).
 *                  [5] Wikipedia — "Abelian sandpile model":
 *                      https://en.wikipedia.org/wiki/Abelian_sandpile_model
 *                  Rendering —
 *                  [6] Bourke — "Colour Ramping for Data Visualisation": the
 *                      grain-count → colour-tier mapping the renderer uses.
 *                      paulbourke.net/texture_colour/colourramp
 *                  [7] Padala — "NCURSES Programming HOWTO", TLDP: colour
 *                      pairs, glyph output, non-blocking input, resize.
 *                  [8] xterm 256-colour palette — the index the themes draw
 *                      from: https://jonasjacek.github.io/colors/
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * State lives on ONE Scene (§3) that reads like a table of contents: a
 * Sandpile domain object (WHAT is simulated) + the user's run knobs (HOW it
 * is driven) + render state (theme + terminal dims).  Each section owns one
 * concern; this table says what each layer mutates:
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the clock / sleeps
 *   SIMULATION   §3       the Sandpile — grain grid + avalanche work-list
 *                         + read-out counters (Scene.pile)
 *   RENDER       §4       the terminal only — reads the Sandpile, never writes
 *   APP          §5       run knobs (Scene.drops_per_frame / .paused /
 *                         .wave_view / .theme) + signal flags
 *
 *   No LOGIC layer — the SIMULATION layer's pure reads (const Sandpile*) are
 *     just queries it owns: avalanche_pending / queued_count / in_pile.  No
 *     decision lives apart from the layer that acts on it.
 *   No EFFECTS — nothing cosmetic is stored.  The avalanche '*' is read from
 *     live cells ≥ 4 during a 'v'-mode cascade; glyph + colour are derived
 *     from the grain count at render time, not kept as separate state.
 *   DELAYS — trivial: 'space' sets Scene.paused (sim_tick early-returns); the
 *     only wait is the PERFORMANCE frame cap (§2 / TICK_NS).
 *
 * PER-TICK COMBINE — sim_tick (§3) is the ONE place state advances:
 *     bloom mode : drop drops_per_frame grains, each run to stability
 *                  (avalanche_full)
 *     wave  mode : drop one grain, then one avalanche ring/tick (avalanche_wave)
 *   RENDER (scene_draw + scene_hud) then PERFORMANCE (frame cap) run each frame.
 *
 * USER EVENTS are NOT the tick: keys (reset/pause/theme/viz/rate) mutate run
 * knobs in §5, and resize re-fits + resets the grid (screen_resize, §4) — but
 * only sim_tick runs the per-frame simulation step.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ──────────────────────────────────────────────────────────── */

#define TICK_NS       33333333LL   /* ~30 fps                               */
#define MAX_ROWS      128
#define MAX_COLS      320
#define QMAX          (MAX_ROWS * MAX_COLS + 1)  /* avalanche queue size   */

#define HUD_TOP        1           /* row 0: data HUD                       */
#define HUD_BOT        1           /* bottom row: action HUD                */

#define DROPS_DEF     40           /* grains per frame (default bloom pace) */
#define DROPS_MAX     500          /* maximum grains per frame              */

/* Critical height: a cell holding ≥ TOPPLE_THRESHOLD grains is unstable and
 * topples, shedding exactly this many grains — one to each of its 4 von-Neumann
 * neighbours.  The value is the lattice coordination number; that the threshold
 * equals the neighbour count is what makes the model well-defined (refs [1],[2]). */
#define TOPPLE_THRESHOLD 4

/* Glyph per grain count: a density ramp, sparse → solid. */
static const char GRAIN_CH[4] = { ' ', '.', 'o', '#' };

enum { CP_G1 = 1, CP_G2, CP_G3, CP_TOPPLE, CP_HUD, CP_HINT };

/* ── themes ──────────────────────────────────────────────────────────────── */

#define N_THEMES 10

/*
 * Theme — the mandala's palette.
 *
 * WHY a struct: a sandpile cell carries only a tiny integer (0–3 when stable,
 * ≥4 mid-topple), so the *only* way the emergent fractal becomes visible is by
 * mapping that integer to colour — this is the rendering core, not decoration.
 * Bundling the four colours under one name lets `t`/`T` swap the entire look
 * atomically without touching the simulation.
 *
 * VALUE LOGIC: each `c[i]` is an xterm-256 index (ref [8]) chosen so the four
 * entries form a perceptual gradient dim→bright.  The ordering is deliberate
 * and mirrors the grain tiers, so brightness encodes "how close to toppling":
 *   c[0] grain 1 — darkest : the sparse outer field
 *   c[1] grain 2 — mid     : the body
 *   c[2] grain 3 — bright  : the near-critical ridges (the pile's skeleton)
 *   c[3] topple  — hottest : a cell momentarily holding ≥4 during a cascade
 * This is a discrete colour-ramp (ref [6]): four stops instead of a continuous
 * gradient because a stable cell only ever holds one of three values.
 * Every index sits in the "bright half" of the palette so even c[0] stays
 * legible against the default-black background under A_DIM-free rendering.
 *
 * `c8` is the parallel 8-colour fallback for terminals reporting COLORS < 256;
 * it preserves the dim→bright intent within the coarse 8-colour space.
 * The HUD/HINT pairs are NOT here — they are fixed in color_init() so the
 * status line stays the same legible yellow/cyan whichever theme is active.
 */
typedef struct {
    const char *name;
    short c [4];   /* 256-colour: grain 1, grain 2, grain 3, topple */
    short c8[4];   /* 8-colour fallback                             */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        g1   g2   g3  topple   8-colour fallback                         */
    { "AURORA", {  27,  45, 226, 231 }, { COLOR_BLUE,    COLOR_CYAN,   COLOR_YELLOW, COLOR_WHITE  } },
    { "MATRIX", {  28,  46, 190, 231 }, { COLOR_GREEN,   COLOR_GREEN,  COLOR_GREEN,  COLOR_WHITE  } },
    { "EMBER",  {  52, 166, 220, 231 }, { COLOR_RED,     COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE  } },
    { "NOVA",   {  55, 129, 213, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA,COLOR_MAGENTA,COLOR_WHITE  } },
    { "OCEAN",  {  24,  39,  51, 159 }, { COLOR_BLUE,    COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
    { "GOLD",   {  94, 178, 226, 231 }, { COLOR_YELLOW,  COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE  } },
    { "ICE",    {  31, 117, 195, 231 }, { COLOR_CYAN,    COLOR_CYAN,   COLOR_WHITE,  COLOR_WHITE  } },
    { "POISON", {  58, 106, 154, 190 }, { COLOR_GREEN,   COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW } },
    { "NEBULA", {  61, 141, 219, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA,COLOR_WHITE,  COLOR_WHITE  } },
    { "MONO",   { 240, 248, 255, 255 }, { COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  } },
};

/* ── §2 performance ──────────────────────────────────────────────────────── */

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

/* ── §3 simulation  (the ONLY layer that advances grid state) ───────────── */

/*
 * Sandpile — the Abelian sandpile configuration (refs [1], [2]) plus the
 * machinery that relaxes it.  The whole demo is one rule: a cell with ≥4
 * grains topples, shedding one grain to each of its 4 von-Neumann neighbours;
 * toppling cascades until every cell holds 0–3.  This struct is the state that
 * rule reads and writes, in three concept groups.
 *
 * ── the configuration ──────────────────────────────────────────────────
 * `grain[r][c]` is the height at each cell.  uint8_t (not int) because the
 *   value is bounded: stable cells hold 0–3, and a cell transiently holds at
 *   most (3 + 4 neighbours) = 7 mid-cascade — one byte is ample and keeps the
 *   40 KB grid cache-friendly during the inner topple loop.
 * `rows`/`cols` are the pile-area dimensions in cells, set by screen_resize to
 *   the terminal minus the two HUD rows; topple_one bounds-checks neighbours
 *   against them so grains that fall off the border are simply lost (an open
 *   boundary, the standard sandpile condition that lets the system shed mass).
 * `drop_r`/`drop_c` is the single cell grains rain onto — the pile centre,
 *   `rows/2, cols/2`.  Dropping always at the centre is what makes the limit
 *   shape a 4-fold-symmetric mandala (ref [4]) rather than random texture.
 *
 * ── the avalanche work-list (the cascade in progress) ───────────────────
 * An avalanche is processed as a breadth-first relaxation: only cells known to
 * be unstable are revisited, instead of rescanning the whole grid each pass.
 * Because the model is Abelian (ref [2]), the *order* of topplings cannot
 * change the final configuration, so a simple FIFO queue is a valid schedule —
 * and processing it one enqueue-layer at a time yields the visible expanding
 * ring in 'v' mode (avalanche_wave).
 * `q_r[i]`/`q_c[i]` hold the coordinates of queued cells in a ring buffer of
 *   QMAX = rows*cols+1 slots (+1 so a full buffer is distinguishable from an
 *   empty one: head==tail always means empty).
 * `queued[r][c]` is a membership bitmap: a cell unstable in several ways in one
 *   wave must be toppled once, so enqueue() consults this to avoid duplicates.
 *   It is sized like `grain` and tied to the same coordinates, which is why the
 *   work-list lives inside Sandpile rather than as a separate type.
 * `q_head`/`q_tail` are the ring-buffer read/write indices (mod QMAX).
 *
 * ── read-outs (HUD only, never steer the simulation) ────────────────────
 * `total_drops` — grains dropped since the last reset (the bloom's "age").
 * `last_avalanche` — topples in the most recent cascade; over time this grows
 *   and spikes erratically, the visible fingerprint of the power-law avalanche
 *   distribution that defines self-organised criticality (refs [1], [3]).
 */
typedef struct {
    uint8_t  grain[MAX_ROWS][MAX_COLS];   /* grains per cell                  */
    int      rows, cols;                  /* pile-area dimensions (cells)     */
    int      drop_r, drop_c;              /* drop point — pile centre         */

    int      q_r[QMAX], q_c[QMAX];        /* queued cell coordinates          */
    uint8_t  queued[MAX_ROWS][MAX_COLS];  /* membership map (avoids re-queue) */
    int      q_head, q_tail;              /* ring-buffer indices              */

    long long total_drops;                /* grains dropped since reset       */
    long      last_avalanche;             /* topples in the most recent cascade */
} Sandpile;

/*
 * Scene — the table of contents the whole program hangs off.  It exists to
 * keep the three concerns the program juggles in one place while the layers
 * stay decoupled: mutators take the narrowest sub-type (Sandpile*), and only
 * whole-tick orchestrators (sim_tick, screen_resize) ever take Scene*, so
 * "everything hangs off Scene" never re-couples simulation to rendering.
 *
 * ── WHAT is simulated ───────────────────────────────────────────────────
 * `pile` — the Sandpile domain object above; the only field the simulation
 *   layer mutates.
 *
 * ── HOW the user drives it (tunable simulation knobs) ───────────────────
 * `drops_per_frame` — grains dropped per tick in bloom mode (+/- keys, 1..
 *   DROPS_MAX).  This is purely a *pacing* knob: the Abelian property
 *   guarantees the final shape is identical at any rate, so raising it only
 *   makes the mandala bloom faster, never differently.
 * `paused` — space toggles it; sim_tick early-returns, freezing the pile while
 *   rendering continues.
 * `wave_view` — 'v' toggles the two viewing modes.  0 = bloom (drop a burst,
 *   relax each drop fully and instantly).  1 = wave (drop one grain, then
 *   advance the cascade one BFS ring per frame) so a single avalanche is
 *   watchable as an expanding ring — the same physics, slowed for the eye.
 *
 * ── RENDER state ────────────────────────────────────────────────────────
 * `theme` — index into k_themes (t/T cycle).  Lives here, not with the sim
 *   knobs, because palette is a rendering concept even though a key drives it.
 * `term_rows`/`term_cols` — current terminal size, refreshed on resize; used
 *   only to place the HUD (bottom row = term_rows-1) and clip its text to
 *   width.  Distinct from pile.rows/cols, which are the terminal minus the two
 *   HUD rows; keeping both makes the HUD-vs-pile geometry explicit.
 */
typedef struct {
    Sandpile pile;
    int      drops_per_frame;             /* grains dropped per tick (bloom)  */
    int      paused;
    int      wave_view;                   /* watch one avalanche ripple out   */
    int      theme;                       /* palette index into k_themes      */
    int      term_rows, term_cols;        /* terminal dims (HUD placement)    */
} Scene;

/* von Neumann neighbour offsets: N, E, S, W. */
static const int DR[4] = { -1,  0,  1,  0 };
static const int DC[4] = {  0,  1,  0, -1 };

/* Is (r,c) a real cell, or did toppling push a grain over the open border? */
static int in_pile(const Sandpile *p, int r, int c)
{
    return r >= 0 && r < p->rows && c >= 0 && c < p->cols;
}

/* Zero the grid + counters and re-centre the drop point on the pile area. */
static void sandpile_reset(Sandpile *p)
{
    memset(p->grain, 0, sizeof p->grain);
    p->total_drops    = 0;
    p->last_avalanche = 0;
    p->drop_r = p->rows / 2;
    p->drop_c = p->cols / 2;
}

/* ── the avalanche work-list (a FIFO ring buffer of unstable cells) ──────── */

static void enqueue(Sandpile *p, int r, int c)
{
    if (p->queued[r][c]) return;        /* already waiting — don't double-queue */
    p->queued[r][c] = 1;
    p->q_r[p->q_tail] = r;
    p->q_c[p->q_tail] = c;
    p->q_tail = (p->q_tail + 1) % QMAX;
}

/* Take the front cell off the work-list and clear its membership flag. */
static void dequeue(Sandpile *p, int *r, int *c)
{
    *r = p->q_r[p->q_head];
    *c = p->q_c[p->q_head];
    p->q_head = (p->q_head + 1) % QMAX;
    p->queued[*r][*c] = 0;
}

static int avalanche_pending(const Sandpile *p) { return p->q_head != p->q_tail; }

/* How many cells are queued right now — i.e. the size of the current BFS
 * frontier, which is exactly one expanding ring of the avalanche. */
static int queued_count(const Sandpile *p)
{
    return (p->q_tail - p->q_head + QMAX) % QMAX;
}

/* ── toppling ───────────────────────────────────────────────────────────── */

/* Hand one grain to each of the 4 neighbours that lie inside the pile, and
 * enqueue any neighbour this push tipped over the critical height.  Grains
 * pushed across the border are lost (the open boundary that sheds mass). */
static void distribute_to_neighbours(Sandpile *p, int r, int c)
{
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (in_pile(p, nr, nc)) {
            p->grain[nr][nc]++;
            if (p->grain[nr][nc] >= TOPPLE_THRESHOLD) enqueue(p, nr, nc);
        }
    }
}

/* Topple one queued cell: shed TOPPLE_THRESHOLD grains to its neighbours, then
 * re-enqueue any cell (including itself) left unstable.  Returns 1 if it
 * actually toppled, 0 if it had stabilised since being enqueued. */
static int topple_one(Sandpile *p)
{
    int r, c;
    dequeue(p, &r, &c);

    if (p->grain[r][c] < TOPPLE_THRESHOLD) return 0;   /* stabilised meanwhile */

    p->grain[r][c] -= TOPPLE_THRESHOLD;                /* shed one per neighbour */
    distribute_to_neighbours(p, r, c);
    if (p->grain[r][c] >= TOPPLE_THRESHOLD) enqueue(p, r, c);  /* still over */
    return 1;
}

/* Process exactly one BFS layer of the avalanche (the cells queued right now),
 * so the cascade renders as an expanding ring.  Returns topples this wave. */
static long avalanche_wave(Sandpile *p)
{
    long topples = 0;
    int  ring = queued_count(p);        /* snapshot: only this frontier, not refills */
    for (int i = 0; i < ring; i++)
        topples += topple_one(p);
    return topples;
}

/* Run the whole avalanche to stability.  Returns total topples. */
static long avalanche_full(Sandpile *p)
{
    long topples = 0;
    while (avalanche_pending(p)) topples += topple_one(p);
    return topples;
}

/* Start a fresh cascade seeded at (r,c): empty the work-list, then queue the
 * seed cell so the next topple_one picks it up. */
static void begin_avalanche(Sandpile *p, int r, int c)
{
    memset(p->queued, 0, sizeof p->queued);
    p->q_head = p->q_tail = 0;
    enqueue(p, r, c);
}

static void drop_grain(Sandpile *p)
{
    p->grain[p->drop_r][p->drop_c]++;
    p->total_drops++;
    if (p->grain[p->drop_r][p->drop_c] >= TOPPLE_THRESHOLD)
        begin_avalanche(p, p->drop_r, p->drop_c);
}

/* THE per-tick combine — the one place grid state advances. */
static void sim_tick(Scene *s)
{
    if (s->paused) return;
    Sandpile *p = &s->pile;

    if (s->wave_view) {
        /* one grain, then watch its avalanche ripple out ring by ring */
        if (avalanche_pending(p)) {
            p->last_avalanche += avalanche_wave(p);
        } else {
            drop_grain(p);
            p->last_avalanche = 0;
        }
    } else {
        /* bloom mode: drop a burst, run each to full stability instantly */
        for (int i = 0; i < s->drops_per_frame; i++) {
            drop_grain(p);
            if (avalanche_pending(p))
                p->last_avalanche = avalanche_full(p);
        }
    }
}

/* ── §4 render  (state → screen + palette/terminal; reads grid, never writes) */

static void theme_apply(int theme)
{
    const Theme  *th      = &k_themes[theme];
    const short  *palette = (COLORS >= 256) ? th->c : th->c8;  /* per terminal */
    init_pair(CP_G1,     palette[0], -1);   /* grain 1 — sparse field   */
    init_pair(CP_G2,     palette[1], -1);   /* grain 2 — body           */
    init_pair(CP_G3,     palette[2], -1);   /* grain 3 — critical ridges */
    init_pair(CP_TOPPLE, palette[3], -1);   /* ≥4      — topple flash    */
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* fixed top data */
    init_pair(CP_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* fixed bottom    */
    theme_apply(theme);
}

/*
 * cell_appearance — map a grain count to its on-screen look (a discrete colour
 * ramp, ref [6]).  ≥ TOPPLE_THRESHOLD is the hot topple flash, only ever seen
 * mid-cascade in 'v' view; the stable tiers 1/2/3 grow brighter and bolder as
 * the cell nears criticality, so brightness reads as "how close to toppling".
 */
static void cell_appearance(int grains, int *pair, char *glyph, attr_t *bold)
{
    if (grains >= TOPPLE_THRESHOLD) {       /* mid-avalanche */
        *pair = CP_TOPPLE; *glyph = '*'; *bold = A_BOLD;
    } else {                                /* stable tier 1/2/3 */
        *pair  = CP_G1 + (grains - 1);
        *glyph = GRAIN_CH[grains];
        *bold  = (grains >= 2) ? A_BOLD : 0;   /* body + ridges glow */
    }
}

/*
 * scene_draw — the centred top-down mandala.  The drop point sits at the pile
 * centre, which maps to screen centre, so the 4-fold-symmetric pattern is
 * always centred.  Each cell is coloured by its grain count (the emergent art
 * IS the colouring); erase() each frame leaves empty cells blank.
 */
static void scene_draw(const Sandpile *p)
{
    for (int r = 0; r < p->rows; r++) {
        for (int c = 0; c < p->cols - 1; c++) {   /* cols-1: skip last column to avoid cursor wrap */
            int grains = p->grain[r][c];
            if (grains == 0) continue;             /* empty cell stays blank */

            int    pair;
            char   glyph;
            attr_t bold;
            cell_appearance(grains, &pair, &glyph, &bold);

            attron(COLOR_PAIR(pair) | bold);
            mvaddch(HUD_TOP + r, c, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | bold);
        }
    }
}

static void scene_hud(const Scene *s)
{
    char buf[256];

    /* top row: data — title, theme, grains, last-avalanche, rate, state */
    snprintf(buf, sizeof buf,
             " Sandpile  %s  grains:%lld  avalanche:%ld  rate:%d/f  %s",
             k_themes[s->theme].name, (long long)s->pile.total_drops,
             s->pile.last_avalanche, s->drops_per_frame,
             s->paused ? "PAUSED" : (s->wave_view ? "WAVE" : "running"));
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row: actions — every interactive key */
    snprintf(buf, sizeof buf,
             " q:quit  spc:pause  r:reset  t:theme  v:avalanche  +/-:rate ");
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->term_rows - 1, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_init(const Scene *s)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(s->theme);
}

/* USER EVENT: terminal resized — re-fit the pile area and reset (not a tick). */
static void screen_resize(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    s->term_rows  = rows;
    s->term_cols  = cols;
    s->pile.cols  = cols;
    s->pile.rows  = rows - HUD_TOP - HUD_BOT;   /* pile area between the HUD rows */
    if (s->pile.rows < 1) s->pile.rows = 1;
    sandpile_reset(&s->pile);
    erase();
}

/* ── §5 app  (signals, user events, main loop) ───────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static Scene g_scene;   /* the single Scene instance (~400 KB, lives in BSS) */

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    g_scene.theme           = 0;
    g_scene.drops_per_frame = DROPS_DEF;
    g_scene.paused          = 0;
    g_scene.wave_view       = 0;

    screen_init(&g_scene);
    screen_resize(&g_scene);

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize(&g_scene);
        }

        /* USER EVENT: keys — mutate run knobs, not the tick */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0;              break;
            case ' ':           g_scene.paused ^= 1;                 break;
            case 'r': case 'R': sandpile_reset(&g_scene.pile); erase(); break;
            case 't': case 'T':
                g_scene.theme = (g_scene.theme + 1) % N_THEMES;
                theme_apply(g_scene.theme);
                break;
            case 'v': case 'V': g_scene.wave_view ^= 1;              break;
            case '+': case '=':
                if (g_scene.drops_per_frame < DROPS_MAX) g_scene.drops_per_frame++;
                break;
            case '-': case '_':
                if (g_scene.drops_per_frame > 1) g_scene.drops_per_frame--;
                break;
            }
        }

        sim_tick(&g_scene);         /* PER-TICK COMBINE (see ARCHITECTURE) */

        /* RENDER then PERFORMANCE frame cap */
        erase();
        scene_draw(&g_scene.pile);
        scene_hud(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
