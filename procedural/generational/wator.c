/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wator.c — Wa-Tor Predator-Prey World
 *
 * Fish (o) swim and breed; sharks (X) hunt fish, breed, and starve.
 * A. K. Dewdney's Wa-Tor simulation (Scientific American, December 1984).
 *
 * Rules (each tick, in randomly shuffled cell order):
 *
 *   Fish:
 *     1. age++
 *     2. Move to a random adjacent empty cell (toroidal wrap).
 *        If no empty cell exists: stay put.
 *     3. If age ≥ fish_breed: leave a new fish (age=0) at old cell; reset age.
 *
 *   Shark:
 *     1. breed_age++, hunger++
 *     2. If hunger ≥ shark_starve: die (cell → empty).
 *     3. Look for adjacent fish first.  If found: eat one, move there, hunger=0.
 *        Otherwise: move to a random adjacent empty cell.
 *        If no move possible: stay put.
 *     4. If breed_age ≥ shark_breed: leave a new shark at old cell; reset breed_age.
 *
 * Shuffled update prevents directional bias — all live cells are collected into
 * an array, Fisher-Yates shuffled, then processed in that order.  A moved[]
 * flag prevents any cell being processed twice per tick.
 *
 * Layout:
 *   Row  0                      — data HUD (tick, populations, speed, state)
 *   Rows 1 … ocean.rows         — ocean grid
 *   next 4 rows                 — dual population history graph
 *   bottom row                  — action HUD (interactive keys)
 *
 * Population graph:
 *   Upper 2 rows — fish count (cyan bars, grow downward)
 *   Lower 2 rows — shark count (red bars, grow upward)
 *   Each column = one past tick from a HIST_LEN-entry ring buffer.
 *
 * Keys:
 *   r         reseed (fresh random ocean)
 *   space     pause / resume
 *   + / =     more steps per frame  (up to 20)
 *   - / _     fewer steps per frame
 *   n / p     next / previous ecological regime preset (reseeds)
 *   q / Q     quit
 *
 * Presets (n/p):  STABLE · BLOOM · BOOM-BUST · FRAGILE
 *   Each sets fish/shark breed rates, shark starve time, and initial
 *   densities to a different point in Wa-Tor's parameter space — from
 *   steady oscillation to shark boom-and-collapse to extinction.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra wator.c -o wator -lncurses
 *
 * Sections (layers — see ARCHITECTURE block below):
 *   §1 config  §2 performance  §3 simulation-data  §4 simulation-step
 *   §5 render  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Wa-Tor predator-prey cellular automaton (Dewdney 1984).
 *                  Fish and sharks are simulated on a toroidal grid with
 *                  age-based breeding and hunger-based shark death.
 *                  Shuffled update (Fisher-Yates on active cell list) removes
 *                  directional bias — equivalent to random-order asynchronous
 *                  update, which better models spatial competition.
 *
 * Physics        : Lotka-Volterra dynamics emerge: fish and shark populations
 *                  oscillate with a phase lag (fish peak before shark peak).
 *                  Parameter space: low fish_breed + high shark_breed →
 *                  shark overpopulation and collapse; opposite → shark
 *                  starvation.  Stable cycles exist in a narrow parameter band.
 *
 * Data-structure : Grid of uint16_t cells encoding (type, age, hunger) in
 *                  bitfields.  Active cell array rebuilt each tick by scanning
 *                  the grid O(rows×cols); Fisher-Yates shuffle is O(N_active).
 *                  moved[] flag (parallel uint8 grid) prevents double-processing.
 *
 * Rendering      : Population history ring buffer (HIST_LEN entries); drawn
 *                  as a 4-row dual bar chart: fish above, sharks below.
 *
 * References     : Concept —
 *                  • Dewdney, A. K. (1984) — "Computer Recreations: Sharks
 *                    and fish wage an ecological war on the toroidal planet
 *                    Wa-Tor", Scientific American 251(6), Dec 1984. The
 *                    original specification of this exact simulation.
 *                  • Lotka (1925) / Volterra (1926) — the predator-prey
 *                    equations whose oscillations Wa-Tor reproduces from
 *                    purely local rules (the emergent dynamics in Physics).
 *                  • Grimm & Railsback (2005) — "Individual-based Modeling
 *                    and Ecology". Why per-agent rules (not population ODEs)
 *                    are the right model, and what emerges from them.
 *                  • Knuth, TAOCP vol. 2, §3.4.2 — Fisher-Yates shuffle
 *                    (Algorithm P): the unbiased shuffle behind the random
 *                    cell-update order and the per-cell direction picks.
 *                  • Wikipedia — "Wa-Tor": parameter tables and the rule
 *                    variants. https://en.wikipedia.org/wiki/Wa-Tor
 *                  Rendering —
 *                  • Tufte — "Beautiful Evidence" (2006), ch. on sparklines:
 *                    the dense, axis-free history strip the dual bar chart is.
 *                  • Padala — "NCURSES Programming HOWTO", TLDP: colour pairs,
 *                    glyph output, non-blocking input, resize handling.
 *                  • xterm 256-colour palette — the indices the fish / shark /
 *                    HUD pairs draw from: https://jonasjacek.github.io/colors/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell of the toroidal ocean owns three numbers — type (empty,
 * fish, shark), breed counter, hunger counter.  Each tick, every live
 * cell is awakened in random order and runs a tiny three-rule script:
 * "did I starve?", "is there food / open water adjacent?", "have I bred
 * yet?".  No global step, no neighbour count — just per-cell lottery
 * draws against a randomly-shuffled neighbour list.  Lotka-Volterra
 * oscillations EMERGE from this micro-rule, they aren't programmed in.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a chess board where every fish and shark gets one ticket per
 * tick.  The tickets are drawn from a hat in random order.  When your
 * ticket is called, you look in the four cardinal directions: a shark
 * checks for fish first, then for empty water; a fish checks for empty
 * water; both then check whether they've bred enough times to leave a
 * baby behind.  Sharks that haven't eaten in shark_starve turns leave
 * the board.  No central referee — the only synchronization is the
 * "moved" flag that prevents re-drawing the same ticket twice.
 *
 * ALGORITHM IN STEPS  (per sim_step)
 * ──────────────────
 *  1. Clear ocean.moved.  collect_live_cells fills ocean.order with every
 *     live cell index; Fisher-Yates shuffle it (random asynchronous order).
 *  2. For each cell index in shuffled order:
 *        if ocean.moved[r][c] continue.  (cell already acted this tick)
 *        if FISH  → fish_step
 *        if SHARK → shark_step
 *  3. fish_step:
 *        a. age++.
 *        b. find_neighbour for the first EMPTY of 4 shuffled directions.
 *        c. if breed_age >= fish_breed, leave new fish at old cell, reset.
 *        d. cell_set fish at new cell, mark moved.
 *  4. shark_step:
 *        a. breed++, hunger++.
 *        b. if hunger >= shark_starve → die, return.
 *        c. find_neighbour FISH first (eat, hunger=0); else EMPTY; else stay.
 *        d. if breed >= shark_breed, leave offspring at old cell.
 *        e. cell_set shark at new cell, mark moved.
 *  5. census recounts fish_pop/shark_pop; history_push records them, advancing
 *     PopulationHistory.head modulo HIST_LEN.
 *  6. Render: data HUD (row 0), ocean rows, 4-row dual histogram,
 *     action HUD (bottom row).
 *
 * KEY FORMULAS
 * ────────────
 *  Toroidal wrap   :  nr = (r + dr + R) mod R,   nc = (c + dc + C) mod C
 *  Histogram index :  idx = (head − (cols−1−col) + 2·HIST_LEN) mod HIST_LEN
 *  Bar height      :  fl = pop · bar_rows / scale,
 *                     scale_fish  = R·C / HIST_FISH_FULL_DIV,
 *                     scale_shark = R·C / HIST_SHARK_FULL_DIV  (sharks scaled
 *                     brighter because they're rarer)
 *  Visual age cue  :  fish "old"   when breed_age >= fish_breed − 1
 *                     shark "hungry" when hunger >= shark_starve − 1
 *  Population
 *    oscillation   :  steady-state cycle requires
 *                     fish births ≈ shark predation rate;
 *                     phase lag arises because sharks reproduce slower
 *                     than fish.  At the STABLE preset
 *                     (fish_breed=3, shark_breed=10, shark_starve=4)
 *                     the system shows visible Lotka–Volterra waves.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • A shark that just bred AND ate — the offspring at the OLD cell
 *    inherits hunger from after the meal (zero), not the parent's
 *    pre-meal hunger.  This is the classic Wa-Tor convention.
 *  • moved guard: when shark eats fish, the eaten fish's slot becomes
 *    SHARK with moved=1, so when the fish's own ticket is later drawn,
 *    the cell is no longer FISH and the if-chain falls through.
 *  • Total extinction is a stable fixed point — when sharks eat all
 *    fish, they then starve out; ocean ends empty.  Press `r` to reseed.
 *  • Histogram scale is fixed; if fish exceed 50 % of grid the bars
 *    saturate (always full); below scale they fall to 0 (empty bar).
 *  • HIST_LEN = 512 is a ring; older history scrolls off the left edge
 *    silently when the terminal is wider than 512 columns (rare).
 *  • breed and hunger are uint8_t and saturate at 255 — fish that
 *    sit trapped in a corner forever stop counting at 255 instead of
 *    overflowing.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press `r`: ocean reseeds; fish density ≈30 %, sharks ≈5 %.
 *    Count by sampling 100 random cells.
 *  • Watch the histogram: fish bars (cyan, top) and shark bars (red,
 *    bottom) should oscillate roughly out of phase — fish peak comes
 *    several ticks before shark peak.
 *  • Hold `+` to crank steps/frame to 20: oscillation period clearly
 *    visible across the screen width within seconds.
 *  • Pause (space): tick counter freezes; populations shown stay fixed.
 *  • A fully starved ocean (everything gone) should show empty bars
 *    sliding rightward as new zero-pop ticks accumulate.
 *  • A fish-only ocean (a preset with shark_pct=0) saturates with fish
 *    within ~10 ticks at fish_breed=3.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * State lives on one Scene (§3): the Ocean (the world), its PopulationHistory,
 * the user knobs, and the terminal size.  Each layer owns one concern; this
 * table says what each mutates:
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the clock / sleeps
 *   SIMULATION   §3,§4    the Ocean (type / breed / hunger / moved grids, the
 *                         shuffle order, the census fish_pop/shark_pop, tick)
 *                         and the PopulationHistory ring.  The rule params are
 *                         NOT copied — they are read straight from the active
 *                         regime PRESETS[Scene.preset].
 *   RENDER       §5       the terminal only — reads Ocean + history, never
 *                         writes them; screen_resize also re-fits Ocean.rows /
 *                         .cols + Scene.term_rows/.term_cols (a USER EVENT).
 *   APP          §6       run knobs (Scene.steps / .paused / .preset) + the
 *                         signal flags (g_running / g_need_resize)
 *
 *   No LOGIC layer — the pure decisions are small named helpers within §4
 *     (wrap, find_neighbour) called by fish_step / shark_step, not a separate
 *     layer; likewise ishuffle transforms only its array argument (no global
 *     state, no I/O).  All sit with SIMULATION.
 *   No EFFECTS — nothing cosmetic is stored.  The "old fish" / "hungry shark"
 *     colour cues are derived at render time from the breed / hunger counters;
 *     the population history is real census data, not a decorative trail.
 *   No DELAYS — 'space' sets Scene.paused (sim_tick early-returns); the only
 *     wait is the PERFORMANCE frame cap (TICK_NS).
 *
 *   Signatures land the narrowest type: fish_step/shark_step/sim_step take
 *   Ocean* (+ const Preset*); the renderers take const Ocean* / const
 *   PopulationHistory*; only the tick/reset orchestrators (sim_tick,
 *   scene_reset, screen_resize) take Scene*.
 *
 * PER-TICK COMBINE — sim_tick (§4) is the ONE place state advances.  It runs
 * Scene.steps × sim_step, and each sim_step in fixed order:
 *     1. clear moved; collect live cells into order[]; Fisher-Yates shuffle.
 *     2. process each live cell in shuffled order (fish_step / shark_step).
 *     3. census fish_pop / shark_pop; push into the history ring; tick++.
 *   RENDER (scene_ocean + scene_histogram + scene_hud) then PERFORMANCE (frame
 *   cap) run once per frame in main, OUTSIDE the tick.
 *
 * USER EVENTS are NOT the tick: keys (reseed / pause / speed / preset) and
 * resize mutate state directly in §6 — reseed and preset switching call
 * scene_reset; resize re-fits the layout and reseeds — but only sim_tick
 * advances the simulation per frame.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ──────────────────────────────────────────────────────────── */

#define TICK_NS         33333333LL   /* ~30 fps                              */
#define MAX_ROWS        128
#define MAX_COLS        320
#define HIST_ROWS       4            /* rows reserved for population graph   */
#define HIST_LEN        512          /* ring-buffer length (columns of history)*/
#define HUD_TOP         1            /* row 0: data HUD                      */
#define HUD_BOT         1            /* bottom row: action HUD               */

/* Histogram full-scale: a bar reaches full height when its population hits
 * 1/DIV of the grid.  Sharks use a smaller divisor (fill at ~1/10) because
 * they are far rarer than fish, so their bar stays visible. */
#define HIST_FISH_FULL_DIV    2
#define HIST_SHARK_FULL_DIV   10

/*
 * Preset — one named ecological regime.  Wa-Tor's behaviour is governed
 * entirely by three rate parameters and two initial densities, and that
 * parameter space splits into qualitatively distinct regions (Dewdney 1984;
 * the underlying Lotka-Volterra balance, see CONCEPTS): a narrow band of stable
 * oscillation, flanked by shark boom-and-collapse (sharks breed faster than
 * they starve → eat everything → crash) and shark starvation (they starve
 * faster than they breed → die out, fish saturate).  Each preset is one
 * labelled point in that space; n/p cycle through them (reseeding the ocean).
 *
 * VALUE LOGIC (each "every N ticks" counter compares against an entity's age):
 *   fish_breed   : lower → fish reproduce faster → more prey → more sharks.
 *   shark_breed  : lower → sharks reproduce faster → heavier predation.
 *   shark_starve : higher → sharks survive longer between meals → overrun.
 *   fish_pct,
 *   shark_pct    : starting densities — the basin the dynamics settle from.
 */
typedef struct {
    const char *name;
    int fish_breed;     /* fish reproduces every N ticks               */
    int shark_breed;    /* shark reproduces every N ticks              */
    int shark_starve;   /* shark dies after N ticks without food       */
    int fish_pct;       /* initial fish density  (% of cells)          */
    int shark_pct;      /* initial shark density (% of cells)          */
} Preset;

#define N_PRESETS 4
static const Preset PRESETS[N_PRESETS] = {
    /* name          Fbr  Sbr  Sstv   F%  S%    behaviour                    */
    { "STABLE",        3,  10,    4,  30,  5 },/* steady Lotka-Volterra waves */
    { "BLOOM",         2,  15,    3,  25,  3 },/* fish-dominant, sharks scarce*/
    { "BOOM-BUST",     4,   6,    8,  35,  8 },/* sharks overrun, then crash  */
    { "FRAGILE",       5,  12,    2,  20,  6 },/* sharks starve, extinction   */
};

#define STEPS_DEF       1
#define STEPS_MAX       20

#define EMPTY  0
#define FISH   1
#define SHARK  2

/* direction vectors: N E S W */
static const int DR[4] = {-1,  0, 1, 0};
static const int DC[4] = { 0,  1, 0,-1};

enum {
    CP_FISH_Y = 1,   /* young fish (bright cyan)   */
    CP_FISH_O,       /* old fish   (dim teal)       */
    CP_SHARK_F,      /* fed shark  (bright red)     */
    CP_SHARK_H,      /* hungry shark (orange)       */
    CP_HIST_F,       /* histogram — fish            */
    CP_HIST_S,       /* histogram — shark           */
    CP_HUD,          /* top data row (bright yellow)*/
    CP_HINT          /* bottom action row (cyan)    */
};

/* ── §2 performance — monotonic clock + sleep (the frame-cap helpers) ─────── */

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

/* ── §3 simulation-data — grid state, live params, reset/seed setup ───────── */

/*
 * Ocean — Dewdney's toroidal planet Wa-Tor (Dewdney 1984): a wrap-around grid
 * where every cell is empty, a fish, or a shark.  "Toroidal" means the edges
 * connect (top↔bottom, left↔right), so there is no boundary to distort the
 * dynamics — every cell has the same four von-Neumann neighbours.
 *
 * WHY struct-of-arrays: the four per-cell attributes are kept as separate
 * parallel grids rather than an array of Cell structs because the hot paths
 * sweep one attribute at a time — sim_step memset's `moved` wholesale and
 * scans `type` to list live cells — so contiguous same-field arrays stay
 * cache-warm.  All four share the same [r][c]; together they ARE one cell.
 *
 * VALUE LOGIC:
 *   rows, cols : live grid size in cells.  cols == terminal width; rows ==
 *                terminal height minus the HUD + histogram rows, so the grid
 *                fills exactly the band between the two HUD bars.
 *   type       : EMPTY / FISH / SHARK — the cell's occupant.
 *   breed      : dual-use age counter — a fish's age, a shark's breed timer.
 *                At >= the regime's *_breed the entity spawns offspring and
 *                resets.  uint8_t, saturated at 255 by the ++ guards, so an
 *                entity trapped forever stops counting instead of wrapping to 0
 *                (which would make it spuriously "young" again).
 *   hunger     : shark only — ticks since its last meal; at >= shark_starve the
 *                shark dies.  Also uint8_t-saturated.
 *   moved      : per-tick guard.  Cells update asynchronously in random order,
 *                so when an entity moves into a not-yet-processed cell it must
 *                not act twice this tick — `moved` marks the destination, and
 *                the later draw of that cell's ticket falls through.
 *   order      : the work-list — indices of all live cells, Fisher-Yates
 *                shuffled (Knuth TAOCP §3.4.2) so update order is unbiased
 *                (random asynchronous update; a fixed raster scan would bias
 *                motion in the scan direction).  Each entry packs (r,c) as
 *                r*MAX_COLS + c — the fixed array stride MAX_COLS, not the live
 *                `cols`, so packing/unpacking is stable across resizes.
 *   fish_pop,
 *   shark_pop  : census recomputed each tick — the two counts whose ratio
 *                oscillates (Lotka-Volterra) and that feed the HUD + history.
 *   tick       : ticks elapsed since the last reset (the world's age).
 */
typedef struct {
    int       rows, cols;                  /* grid dimensions (cells)        */
    uint8_t   type  [MAX_ROWS][MAX_COLS];  /* EMPTY / FISH / SHARK           */
    uint8_t   breed [MAX_ROWS][MAX_COLS];  /* fish: age; shark: breed_age    */
    uint8_t   hunger[MAX_ROWS][MAX_COLS];  /* shark: ticks since last meal   */
    uint8_t   moved [MAX_ROWS][MAX_COLS];  /* set after entity moves/dies    */
    int       order [MAX_ROWS * MAX_COLS]; /* shuffled processing indices    */
    long      fish_pop, shark_pop;         /* current census                 */
    long long tick;                        /* ticks since last reset         */
} Ocean;

/*
 * PopulationHistory — a ring buffer of the last HIST_LEN censuses, one fish and
 * one shark count per past tick.  WHY it exists: the whole point of Wa-Tor is
 * that the two populations trace coupled Lotka-Volterra oscillations (the fish
 * peak leads the shark peak); a single live count can't show that, so we retain
 * the recent time series and draw it as a dual sparkline (Tufte) — the bar
 * chart under the ocean.  It is real recorded data, not a cosmetic trail.
 *
 * VALUE LOGIC:
 *   fish[], shark[] : parallel ring buffers; fish[i]/shark[i] is the census in
 *                     slot i.  HIST_LEN (512) exceeds any real terminal width,
 *                     so the visible chart never runs out of history — old
 *                     entries are simply overwritten.
 *   head            : next write slot, advanced (head+1) % HIST_LEN each tick.
 *                     The renderer walks backwards from head so the newest tick
 *                     maps to the rightmost column.
 */
typedef struct {
    long fish [HIST_LEN];
    long shark[HIST_LEN];
    int  head;
} PopulationHistory;

/*
 * Scene — the running showcase, read top-to-bottom as a table of contents.
 * It aggregates the data so the layers stay decoupled: simulation functions
 * take Ocean* / const Preset*, renderers take const sub-types, and only the
 * tick/reset orchestrators take Scene* — so nothing re-couples through this
 * one struct.
 *   WHAT  — ocean: the simulated world; history: its census over time.
 *   HOW   — steps: sim_steps run per frame (the +/- speed knob, 1..STEPS_MAX);
 *           paused: freezes the tick; preset: index into PRESETS.  Storing the
 *           INDEX, not a copy of the rate params, keeps ONE source of truth —
 *           the active regime is always PRESETS[preset], read where needed.
 *   WHERE — term_rows/term_cols: terminal size.  Kept distinct from
 *           ocean.rows/cols (which exclude the HUD + histogram rows): used to
 *           place the bottom HUD (term_rows-1) and clip HUD text to width.
 */
typedef struct {
    Ocean             ocean;
    PopulationHistory history;
    int  steps;
    int  paused;
    int  preset;
    int  term_rows, term_cols;
} Scene;

static Scene g_scene;   /* the single Scene instance (~340 KB, lives in BSS) */

/* Seed the ocean grid for a fresh run, using the active regime's densities. */
static void ocean_seed(Ocean *oc, const Preset *p)
{
    memset(oc->type,   0, sizeof oc->type);
    memset(oc->breed,  0, sizeof oc->breed);
    memset(oc->hunger, 0, sizeof oc->hunger);
    oc->tick = 0;

    for (int r = 0; r < oc->rows; r++) {
        for (int c = 0; c < oc->cols; c++) {
            int roll = rand() % 100;
            if (roll < p->fish_pct) {
                oc->type [r][c] = FISH;
                oc->breed[r][c] = (uint8_t)(rand() % p->fish_breed);
            } else if (roll < p->fish_pct + p->shark_pct) {
                oc->type  [r][c] = SHARK;
                oc->breed [r][c] = (uint8_t)(rand() % p->shark_breed);
                oc->hunger[r][c] = (uint8_t)(rand() % p->shark_starve);
            }
        }
    }
}

static void history_clear(PopulationHistory *h)
{
    memset(h->fish,  0, sizeof h->fish);
    memset(h->shark, 0, sizeof h->shark);
    h->head = 0;
}

/* Reset everything for a fresh run under the current preset (a USER EVENT). */
static void scene_reset(Scene *s)
{
    ocean_seed(&s->ocean, &PRESETS[s->preset]);
    history_clear(&s->history);
}

/* ── §4 simulation-step — the tick: shuffle, fish/shark step, census ──────── */

/* Fisher-Yates shuffle */
static void ishuffle(int *arr, int n)
{
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = arr[i]; arr[i] = arr[j]; arr[j] = t;
    }
}

/* Toroidal wrap: step `coord` by `delta` on an axis of `extent` cells, wrapping
 * around the edge (the +extent keeps the result non-negative before the mod). */
static int wrap(int coord, int delta, int extent)
{
    return (coord + delta + extent) % extent;
}

/* Scan the four neighbours of (r,c) in the given (shuffled) direction order;
 * write the first one holding `want` to the nr,nc out-params and return 1, else 0.
 * Pure read — the shuffled order makes the choice unbiased among matches. */
static int find_neighbour(const Ocean *oc, int r, int c, const int dirs[4],
                          uint8_t want, int *nr, int *nc)
{
    for (int i = 0; i < 4; i++) {
        int tr = wrap(r, DR[dirs[i]], oc->rows);
        int tc = wrap(c, DC[dirs[i]], oc->cols);
        if (oc->type[tr][tc] == want) { *nr = tr; *nc = tc; return 1; }
    }
    return 0;
}

/* Put an entity (with its counters) into a cell. */
static void cell_set(Ocean *oc, int r, int c, uint8_t type, uint8_t breed, uint8_t hunger)
{
    oc->type  [r][c] = type;
    oc->breed [r][c] = breed;
    oc->hunger[r][c] = hunger;
}

/* Empty a cell (entity moved away or died). */
static void cell_clear(Ocean *oc, int r, int c)
{
    oc->type  [r][c] = EMPTY;
    oc->breed [r][c] = 0;
    oc->hunger[r][c] = 0;
}

static void fish_step(Ocean *oc, const Preset *p, int r, int c)
{
    if (oc->breed[r][c] < 255) oc->breed[r][c]++;          /* age one tick */

    int dirs[4] = {0, 1, 2, 3};
    ishuffle(dirs, 4);
    int nr, nc;
    if (!find_neighbour(oc, r, c, dirs, EMPTY, &nr, &nc))
        return;                                            /* boxed in — stay put */

    int     breed_now   = (oc->breed[r][c] >= p->fish_breed);
    uint8_t carry_breed = breed_now ? 0 : oc->breed[r][c];

    cell_set(oc, nr, nc, FISH, carry_breed, 0);            /* swim to the open cell */
    oc->moved[nr][nc] = 1;

    if (breed_now) cell_set(oc, r, c, FISH, 0, 0);         /* leave a fledgling behind */
    else           cell_clear(oc, r, c);                   /* else vacate the old cell */
}

static void shark_step(Ocean *oc, const Preset *p, int r, int c)
{
    if (oc->breed [r][c] < 255) oc->breed [r][c]++;        /* age breed timer */
    if (oc->hunger[r][c] < 255) oc->hunger[r][c]++;        /* hunger since last meal */

    if (oc->hunger[r][c] >= p->shark_starve) {             /* starved → die */
        cell_clear(oc, r, c);
        return;
    }

    int dirs[4] = {0, 1, 2, 3};
    ishuffle(dirs, 4);
    int nr, nc, ate = 0;
    if      (find_neighbour(oc, r, c, dirs, FISH,  &nr, &nc)) ate = 1;  /* hunt: eat */
    else if (!find_neighbour(oc, r, c, dirs, EMPTY, &nr, &nc)) return;  /* trapped — stay */

    uint8_t new_hunger  = ate ? 0 : oc->hunger[r][c];      /* a meal resets hunger */
    int     breed_now   = (oc->breed[r][c] >= p->shark_breed);
    uint8_t carry_breed = breed_now ? 0 : oc->breed[r][c];

    cell_set(oc, nr, nc, SHARK, carry_breed, new_hunger);  /* move to the chosen cell */
    oc->moved[nr][nc] = 1;

    /* offspring inherits the parent's post-meal hunger (the Wa-Tor convention) */
    if (breed_now) cell_set(oc, r, c, SHARK, 0, new_hunger);
    else           cell_clear(oc, r, c);
}

/* List every live cell into oc->order (packed r*MAX_COLS + c); return count. */
static int collect_live_cells(Ocean *oc)
{
    int n = 0;
    for (int r = 0; r < oc->rows; r++)
        for (int c = 0; c < oc->cols; c++)
            if (oc->type[r][c] != EMPTY)
                oc->order[n++] = r * MAX_COLS + c;
    return n;
}

/* Recount the living populations into the Ocean's census fields. */
static void census(Ocean *oc)
{
    oc->fish_pop = 0; oc->shark_pop = 0;
    for (int r = 0; r < oc->rows; r++)
        for (int c = 0; c < oc->cols; c++) {
            if      (oc->type[r][c] == FISH)  oc->fish_pop++;
            else if (oc->type[r][c] == SHARK) oc->shark_pop++;
        }
}

/* Append one tick's populations to the ring buffer. */
static void history_push(PopulationHistory *h, long fish, long shark)
{
    h->fish [h->head] = fish;
    h->shark[h->head] = shark;
    h->head = (h->head + 1) % HIST_LEN;
}

static void sim_step(Ocean *oc, PopulationHistory *h, const Preset *p)
{
    memset(oc->moved, 0, sizeof oc->moved);

    int n = collect_live_cells(oc);
    ishuffle(oc->order, n);            /* random asynchronous update order */

    for (int i = 0; i < n; i++) {
        int r = oc->order[i] / MAX_COLS;   /* unpack the packed cell index */
        int c = oc->order[i] % MAX_COLS;
        if (oc->moved[r][c]) continue;     /* already acted this tick */
        if      (oc->type[r][c] == FISH)  fish_step(oc, p, r, c);
        else if (oc->type[r][c] == SHARK) shark_step(oc, p, r, c);
    }

    census(oc);
    history_push(h, oc->fish_pop, oc->shark_pop);
    oc->tick++;
}

static void sim_tick(Scene *s)
{
    if (s->paused) return;
    const Preset *p = &PRESETS[s->preset];
    for (int i = 0; i < s->steps; i++)
        sim_step(&s->ocean, &s->history, p);
}

/* ── §5 render — palette + draw + terminal (reads state; resize reseeds) ──── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_FISH_Y,  51,  -1);  /* bright cyan — young fish         */
        init_pair(CP_FISH_O,  37,  -1);  /* dim teal    — old fish           */
        init_pair(CP_SHARK_F, 196, -1);  /* bright red  — fed shark          */
        init_pair(CP_SHARK_H, 202, -1);  /* orange      — hungry shark       */
        init_pair(CP_HIST_F,  45,  -1);  /* cyan histogram bar               */
        init_pair(CP_HIST_S,  160, -1);  /* dark red histogram bar           */
        init_pair(CP_HUD,     226, -1);  /* bright yellow — top data row     */
        init_pair(CP_HINT,    51,  -1);  /* bright cyan   — bottom action row*/
    } else {
        init_pair(CP_FISH_Y,  COLOR_CYAN,   -1);
        init_pair(CP_FISH_O,  COLOR_CYAN,   -1);
        init_pair(CP_SHARK_F, COLOR_RED,    -1);
        init_pair(CP_SHARK_H, COLOR_YELLOW, -1);
        init_pair(CP_HIST_F,  COLOR_CYAN,   -1);
        init_pair(CP_HIST_S,  COLOR_RED,    -1);
        init_pair(CP_HUD,     COLOR_YELLOW, -1);
        init_pair(CP_HINT,    COLOR_CYAN,   -1);
    }
}

static void scene_ocean(const Ocean *oc, const Preset *p)
{
    for (int r = 0; r < oc->rows; r++) {
        int sr = HUD_TOP + r;
        for (int c = 0; c < oc->cols - 1; c++) {
            uint8_t t = oc->type[r][c];
            if (t == EMPTY) { mvaddch(sr, c, ' '); continue; }
            if (t == FISH) {
                int old    = (oc->breed[r][c] >= p->fish_breed - 1);   /* one tick from breeding */
                attr_t at  = COLOR_PAIR(old ? CP_FISH_O : CP_FISH_Y);
                if (!old) at |= A_BOLD;
                attron(at);
                mvaddch(sr, c, 'o');
                attroff(at);
            } else {
                int hungry = (oc->hunger[r][c] >= p->shark_starve - 1);/* one tick from starving */
                attr_t at  = COLOR_PAIR(hungry ? CP_SHARK_H : CP_SHARK_F) | A_BOLD;
                attron(at);
                mvaddch(sr, c, 'X');
                attroff(at);
            }
        }
    }
}

/* Map chart column c to its history slot: the newest tick (head-1) sits at the
 * rightmost column, older ticks to the left.  (+2·HIST_LEN keeps it positive.) */
static int history_slot(const PopulationHistory *h, int cols, int c)
{
    return (h->head - (cols - 1 - c) + HIST_LEN * 2) % HIST_LEN;
}

/* Bar height in rows for a population, clamped to the band height. */
static int bar_height(long pop, long scale, int rows)
{
    int h = (int)((float)pop / (float)scale * rows);
    return h > rows ? rows : h;
}

static void scene_histogram(const PopulationHistory *h, const Ocean *oc)
{
    long max_pop = (long)oc->rows * (oc->cols - 1);
    if (max_pop == 0) return;

    /* full-scale populations (see HIST_*_FULL_DIV): fish bar fills at ~1/2 the
     * grid, shark bar at ~1/10 — sharks are rarer, so their bar reads brighter */
    long fish_scale  = max_pop / HIST_FISH_FULL_DIV;  if (fish_scale  < 1) fish_scale  = 1;
    long shark_scale = max_pop / HIST_SHARK_FULL_DIV; if (shark_scale < 1) shark_scale = 1;

    int fish_rows  = HIST_ROWS / 2;  /* upper 2 rows for fish   */
    int shark_rows = HIST_ROWS / 2;  /* lower 2 rows for sharks */

    for (int c = 0; c < oc->cols - 1; c++) {
        int idx = history_slot(h, oc->cols, c);

        /* fish bars — upper rows, grow downward from the top */
        int fl = bar_height(h->fish[idx], fish_scale, fish_rows);
        attron(COLOR_PAIR(CP_HIST_F));
        for (int hr = 0; hr < fish_rows; hr++) {
            int sr = HUD_TOP + oc->rows + hr;
            mvaddch(sr, c, (fl > hr) ? '#' : '.');
        }
        attroff(COLOR_PAIR(CP_HIST_F));

        /* shark bars — lower rows, grow upward from the bottom */
        int sl = bar_height(h->shark[idx], shark_scale, shark_rows);
        attron(COLOR_PAIR(CP_HIST_S));
        for (int hr = 0; hr < shark_rows; hr++) {
            int sr = HUD_TOP + oc->rows + fish_rows + hr;
            /* hr=0 is top of shark area; fill from bottom (hr=shark_rows-1) up */
            mvaddch(sr, c, (sl >= shark_rows - hr) ? '#' : '.');
        }
        attroff(COLOR_PAIR(CP_HIST_S));
    }
}

static void scene_hud(const Scene *s)
{
    char buf[MAX_COLS + 1];

    /* top row: data — preset, tick, populations, speed, run state */
    snprintf(buf, sizeof buf,
             " Wa-Tor  preset:%s %d/%d  tick:%lld  fish:%ld  sharks:%ld  spd:%d/f  %s",
             PRESETS[s->preset].name, s->preset + 1, N_PRESETS,
             (long long)s->ocean.tick, s->ocean.fish_pop, s->ocean.shark_pop,
             s->steps, s->paused ? "PAUSED" : "running");
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row: actions — every interactive key */
    snprintf(buf, sizeof buf,
             " q:quit  spc:pause  r:reseed  +/-:speed  n/p:preset ");
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->term_rows - 1, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* terminal lifecycle — screen_resize re-fits the layout and reseeds (USER EVENT) */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void screen_resize(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    s->term_rows  = rows;
    s->term_cols  = cols;
    s->ocean.cols = cols;
    s->ocean.rows = rows - HUD_TOP - HIST_ROWS - HUD_BOT;
    if (s->ocean.rows < 1) s->ocean.rows = 1;
    scene_reset(s);
    erase();
}

/* ── §6 app — signals, key/resize events, fixed-timestep main loop ────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

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

    screen_init();
    g_scene.steps  = STEPS_DEF;
    g_scene.paused = 0;
    g_scene.preset = 0;              /* default regime before first seed */
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    screen_resize(&g_scene);

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize(&g_scene);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0;               break;
            case ' ':           g_scene.paused ^= 1;          break;
            case 'r':           scene_reset(&g_scene); erase();break;
            case '+': case '=':
                if (g_scene.steps < STEPS_MAX) g_scene.steps++;
                break;
            case '-': case '_':
                if (g_scene.steps > 1) g_scene.steps--;
                break;
            case 'n': case 'N':     /* next regime preset (reseeds) */
                g_scene.preset = (g_scene.preset + 1) % N_PRESETS;
                scene_reset(&g_scene); erase();
                break;
            case 'p': case 'P':     /* previous regime preset (reseeds) */
                g_scene.preset = (g_scene.preset + N_PRESETS - 1) % N_PRESETS;
                scene_reset(&g_scene); erase();
                break;
            }
        }

        sim_tick(&g_scene);
        erase();
        scene_ocean(&g_scene.ocean, &PRESETS[g_scene.preset]);
        scene_histogram(&g_scene.history, &g_scene.ocean);
        scene_hud(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
