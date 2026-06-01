/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * langton.c — Langton's Ant and Multi-Colour Turmites
 *
 * An ant walks a toroidal grid.  The rule string encodes what to do on
 * each cell colour: 'R' = turn right, 'L' = turn left.  After turning,
 * the cell advances to the next colour and the ant steps forward.
 *
 * With rule "RL" (Langton's original):
 *   colour 0 → turn R, flip to colour 1, step
 *   colour 1 → turn L, flip to colour 0, step
 * After ~10 000 steps a diagonal "highway" emerges from chaos.
 *
 * Presets:
 *   RL        Classic Langton — highway after ~10 k steps
 *   LR        Symmetric fractal
 *   LLRR      Growing square spiral
 *   RLR       Chaotic triangle
 *   LRRL      Complex branching
 *   RRLL      Symmetric highway variant
 *   RLLR      Tiling pattern
 *   LLRRR     Irregular growth
 *
 * Multiple ants of the same rule start at spread positions and share
 * the grid — their trails interact, producing emergent structures.
 *
 * Cell colours are mapped to a palette; ants drawn as bold '@'.
 *
 * Keys:
 *   n / p     next / previous preset
 *   1-3       set number of ants (1, 2, or 3)
 *   r         reset grid and ants
 *   + / =     more steps per frame  (up to 2000)
 *   - / _     fewer steps per frame
 *   space     pause / resume
 *   q / Q     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/langton.c -o langton -lncurses
 *
 * Sections: §1 config  §2 state  §3 performance  §4 color  §5 simulation
 *           §6 render  §7 seed/reset  §8 platform  §9 driver
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Langton's Ant — a 2-D Turing machine on a grid.
 *                  State: ant position, heading (N/E/S/W), cell colours.
 *                  Each step: read cell colour → turn (R or L per rule char)
 *                  → increment cell colour (mod N_colours) → step forward.
 *                  Generalised multi-colour variant: rule string of length K
 *                  means K colours; rule[i] is turn direction for colour i.
 *
 * Physics        : Emergent order from local rules: with rule "RL", the ant
 *                  appears chaotic for ~10 000 steps, then spontaneously
 *                  builds a periodic "highway" — a striking example of
 *                  emergence: complex global structure from simple local rules.
 *
 * Math           : The ant's state space is (position, heading, grid) —
 *                  theoretically infinite but bounded by toroidal wraparound.
 *                  The system is deterministic and reversible (time-reversal
 *                  = swap R↔L in rule).  Highway period = 104 steps (RL rule).
 *
 * Performance    : Steps per frame configurable (1–2000) so user can skip
 *                  ahead past the chaotic phase to observe the highway.
 *                  O(steps_per_frame) per tick; grid update is O(1) per step.
 *
 * References     : Concepts —
 *                  [1] Langton, "Studying artificial life with cellular
 *                      automata," Physica D 22 (1986) — introduces the ant;
 *                      the rule this file simulates.
 *                  [2] Bunimovich & Troubetzkoy, "Recurrence properties of
 *                      Lorentz lattice gas cellular automata," J. Stat. Phys.
 *                      67 (1992) — proves the trajectory is UNBOUNDED (the ant
 *                      can never stay confined; it always escapes to infinity).
 *                  [3] Gajardo, Moreira & Goles, "Complexity of Langton's ant,"
 *                      Discrete Appl. Math. 117 (2002) — the ant is a computer:
 *                      it can build logic gates; predicting it is P-complete.
 *                  [4] Turing, "On Computable Numbers...," Proc. LMS (1936) —
 *                      the head/tape/program model the ant literally is, here
 *                      in 2-D (grid = tape, ant = head, rule string = program).
 *                  [5] Wolfram, "A New Kind of Science" (2002), mobile automata
 *                      — the multi-colour "turmite" generalisation (rules of
 *                      length K, K cell colours) the §4 sim implements.
 *                  Rendering / implementation —
 *                  [6] Ben-Halim, Raymond et al., "Writing Programs with
 *                      NCURSES" (NCURSES HOWTO) — colour pairs, erase/doupdate;
 *                      the §3 colour + §5 scene model.
 *                  [7] Fiedler, "Fix Your Timestep!"
 *                      (gafferongames.com/post/fix_your_timestep) — the §7 app
 *                      fixed-rate frame loop (TICK_NS accumulator, 30 fps).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The ant has a tiny brain — three integers (row, col, heading) — and a
 * memory shared with the world: each cell stores a colour 0..K-1.  At
 * every step it reads the colour beneath its feet, looks up "R" or "L"
 * in the rule string for that colour, turns that way 90°, repaints the
 * cell to the next colour (mod K), and walks forward one square.  No
 * planning, no memory of the past.  Yet for rule "RL" specifically,
 * after roughly 10 000 steps of pure chaos, the ant builds itself a
 * diagonal "highway" that repeats forever with period 104.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine an ant wandering a tiled floor where each tile changes
 * colour every time the ant steps off it, and the colour tells the
 * NEXT ant who arrives where to turn.  So the ant is leaving messages
 * for itself, and reading those messages, and the rules are just
 * "right on white, left on black" — and somehow that suffices to
 * produce a deterministic finite-state machine that paints highways,
 * spirals, fractals depending on the alphabet.  The grid is the tape;
 * the ant is the head; the rule string is the program.  It's literally
 * a 2-D Turing machine.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Pick a rule string of length K (e.g. "RL", "LLRR").  K is the
 *     number of cell colours.
 *  2. Start: grid all 0, ant at centre, heading north.
 *  3. Each step:
 *       a. state = grid[ant.r][ant.c]
 *       b. turn = rule[state]              ('R' or 'L')
 *       c. ant.dir = (turn=='R')? (dir+1)%4 : (dir+3)%4
 *       d. grid[ant.r][ant.c] = (state+1) % K
 *       e. ant.r += DR[dir];  ant.c += DC[dir]   (wrap toroidally)
 *  4. Multi-ant: same loop, N ants each step once per "step" — they
 *     share one grid, so trails interact.
 *  5. Render: each non-zero cell in its colour-pair (CP_C0+state) as
 *     '#'; ants drawn on top as bold '@' in white.
 *
 * KEY FORMULAS
 * ────────────
 *  dir ∈ {0,1,2,3}  →  N, E, S, W                 4-way heading
 *  DR = {-1, 0, +1, 0}  DC = {0, +1, 0, -1}       step deltas
 *  turn-right:  dir = (dir + 1) % 4               (+90° cw)
 *  turn-left :  dir = (dir + 3) % 4               (−90° = +270°)
 *  next state:  c = (c + 1) % K                   K = strlen(rule)
 *  toroidal :   r = (r + dr + R) % R              wrap N/S
 *               c = (c + dc + C) % C              wrap E/W
 *  Highway period (rule "RL"): 104 steps,         empirically observed
 *                              moves (2, -2)      diagonal NE
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Rule string length must be ≥ 2.  Length 1 ("R" or "L") loops the
 *    ant in a 4-cycle on a single cell forever.  Code clamps to ≥ 2.
 *  • Highway only emerges for "RL" — and only AFTER the chaotic phase.
 *    At the default 12 steps/frame it takes ~830 frames (~30 s); press
 *    '+' to step up to 2000 and see the highway form within seconds.  Do
 *    not assume every
 *    rule terminates in a highway; "RLR" is genuinely chaotic.
 *  • Toroidal wrap interferes with highway — once the highway hits a
 *    wrap boundary it can collide with its own old trail and dissolve
 *    back into chaos until a new structure forms.
 *  • Multi-ant: 2 or 3 ants on the same grid mutate each other's
 *    trails.  No collision logic — two ants on the same cell still
 *    each step.  Visually they pass through each other.
 *  • Cell-colour palette has 8 slots; rules of length > 8 would over-
 *    flow.  N_PRESETS keeps max length ≤ 5 (LLRRR).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Rule "RL", 1 ant: at ~10 000 steps the highway emerges — a recurring
 *    rectangular zigzag pattern moving diagonally.  At the default 12
 *    steps/frame that is ~830 frames; press '+' to get there faster.
 *  • Rule "LLRR", 1 ant: at ~5 000 steps you should see a clean
 *    growing square spiral, four-fold symmetric about origin.
 *  • Reverse-symmetry check: rule "LR" (vs "RL") produces the mirror
 *    pattern.  Both have identical chaotic phase length.
 *  • Speed test: '+' doubles steps/frame (12 → 24 → 48 → … → 2000).  Total
 *    step counter in HUD must rise proportionally; framerate should stay
 *    pegged at 30 fps thanks to the fixed-step accumulator.
 *  • Reset 'r' must zero the grid AND re-seed ants at preset offsets;
 *    after reset the same rule must produce the same pattern (it's
 *    deterministic — no rand involved).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Layers, the section that owns each, and the state each MUTATES.  All mutable
 * data hangs off one Scene (g_scene): the Board, the Ants, the Rule, and the
 * preset/speed/paused fields; workers take the narrowest sub-type they touch.
 *
 *   Layer        §   Mutates
 *   ──────────── ─── ─────────────────────────────────────────────────────
 *   CONFIG       §1  nothing (constants, CP ids, dir vectors, presets, palettes)
 *   STATE        §2  the type defs (Ant, Board, Rule, Scene) + g_scene
 *   PERFORMANCE  §3  nothing (clock primitives; the frame cap lives in §9)
 *   COLOR        §4  ncurses colour-pair table (I/O); reads rule.n_states
 *   SIMULATION   §5  board.cells, ants[], board.steps   (the per-tick advance)
 *   RENDER       §6  the screen only; reads board/ants/HUD state, mutates nothing
 *   SEED/RESET   §7  board.cells, ants[], board.steps, preset, rule
 *                    (user-triggered: reset / preset change — NOT the tick)
 *   PLATFORM     §8  board.rows/cols (resize); g_running, g_need_resize (signals)
 *   DRIVER       §9  speed, paused, n_ants   (setup, input, the combine)
 *
 * Per-tick combine (in §9 DRIVER, once per frame):
 *       sim_tick(s)              advance all ants speed times       (SIMULATION)
 *       erase(); scene_draw(s)   board + ants → screen              (RENDER)
 *       scene_hud(s)             HUD rows → screen                  (RENDER)
 *       clock_sleep_ns(...)      hold to the 30 fps TICK_NS cap     (PERFORMANCE)
 *
 * SIMULATION advances state ONLY through sim_tick at the combine point.  User
 * events mutate state but are NOT the tick: key handling lives in §9, reset /
 * preset run via §7, resize via §8 — all outside sim_tick.
 *
 * LOGIC: the pure per-step decisions are the helpers turn() and wrap() at the
 * top of §5 — no mutation, no I/O, so RENDER/EFFECTS cannot affect them.  wrap
 * (positive modulo) is reused by §7 to cycle the preset menu.
 * No EFFECTS layer: the coloured trails ARE the simulation state (board.cells),
 * not a separate cosmetic buffer.
 * No DELAYS layer: 'p' pause is a single control flag (scene.paused) read by
 * sim_tick; the only wait is the §3/§9 frame cap, a PERFORMANCE throttle.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 CONFIG ─ constants, CP ids, dir vectors, presets, palettes ───── */

#define TICK_NS      33333333LL
#define MAX_ROWS     128
#define MAX_COLS     320
#define STEPS_DEF    12      /* default steps/frame (the 'spd' knob, +/-)   */
#define STEPS_MAX    2000
#define MAX_ANTS     3
#define MAX_COLORS   8       /* max cell-state colours per rule           */
#define N_DIRS       4       /* cardinal headings N/E/S/W (DR/DC index space) */
#define NS_PER_SEC   1000000000LL  /* nanoseconds per second (clock maths)   */

/* Colour-pair ids (ncurses init_pair slots, 1-based).  CP_C0..C7 are the per
 * cell-state TRAIL colours: a cell painted with colour-state s draws in pair
 * CP_C0 + s, so the grid shows the ant's history as bands of colour.  CP_ANT is
 * the bold '@' head; CP_HUD / CP_HINT are the data / action HUD rows. */
enum { CP_C0=1, CP_C1, CP_C2, CP_C3, CP_C4, CP_C5, CP_C6, CP_C7,
       CP_ANT, CP_HUD, CP_HINT };

/* Step deltas indexed by heading dir: row/col change for one move N/E/S/W.
 * Kept as parallel arrays so ant_step is a branch-free `r += DR[dir]`. */
static const int DR[4] = {-1,  0, 1,  0};   /* 0=N 1=E 2=S 3=W */
static const int DC[4] = { 0,  1, 0, -1};

/* PRESETS — the menu of turmite rules ('n'/'p' cycle through it).  Each entry is
 * a turn-table string (rule) plus a display name.  The headline lesson of
 * Langton's ant / turmites is that the SAME trivial local algorithm yields wildly
 * different emergent global structure as the turn-table changes — highway,
 * fractal, spiral, chaos (Langton 1986; Wolfram, NKS 2002).  "RL" is Langton's
 * original; table lengths run 2..5 (each char a colour-state, ≤ MAX_COLORS). */
#define N_PRESETS 8
static const struct {
    const char *rule;    /* turn-table: 'R'/'L' per colour-state */
    const char *name;    /* shown in the HUD                     */
} PRESETS[N_PRESETS] = {
    { "RL",    "Langton RL (highway)"  },
    { "LR",    "LR (fractal)"          },
    { "LLRR",  "LLRR (square spiral)"  },
    { "RLR",   "RLR (chaotic)"         },
    { "LRRL",  "LRRL (complex)"        },
    { "RRLL",  "RRLL (symmetric)"      },
    { "RLLR",  "RLLR (tiling)"         },
    { "LLRRR", "LLRRR (irregular)"     },
};

/* Cell-state → colour, a QUALITATIVE palette: distinct hues, NOT a gradient —
 * the colour-states are categorical (which rule-step painted the cell), not an
 * ordered magnitude, so adjacent states get maximally-different colours.  Index
 * 0 = background: -1 means the terminal default shows through (drawn as a space);
 * indices 1..7 colour live states.  PAL256 is the 256-colour cube; PAL8 the
 * fallback for 8-colour terminals (chosen by COLORS in color_init). */
static const short PAL256[MAX_COLORS] =
    { -1, 202, 226, 82, 51, 45, 201, 196 };  /* bg, orange, yellow, green, cyan, blue, magenta, red */
static const short PAL8[MAX_COLORS] =
    { -1, COLOR_RED, COLOR_YELLOW, COLOR_GREEN,
      COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA, COLOR_WHITE };

/* ── §2 STATE ─ the domain types and the one Scene that aggregates them ─ */

/* Ant — one turmite "head": its cell (r,c) and heading dir (0=N 1=E 2=S 3=W,
 * indexing DR/DC).  Just three integers — the ant keeps NO memory of its past;
 * all memory lives in the cells it paints.  In the 2-D Turing-machine reading of
 * Langton's ant this IS the machine head (grid = tape, ant = head, rule =
 * program; cf. Turing 1936).  Famously the head's path is UNBOUNDED — it can
 * never stay confined to a finite region (Bunimovich & Troubetzkoy 1992).
 * Refs: Langton 1986; see the §References block for full citations. */
typedef struct { int r, c, dir; } Ant;

/* Board — the grid the ants crawl and paint: the "tape" of the 2-D Turing
 * machine (Langton 1986).  Each cell holds a colour-STATE; an ant reads it,
 * turns per the rule, increments it (mod n_states), and steps on — so the
 * painted cells ARE the machine's memory and the visible trail.  uint8_t is
 * plenty: at most MAX_COLORS (8) states.  rows/cols are the LIVE size (≤ the
 * static MAX_*, fit to the terminal); only [0,rows)×[0,cols) is simulated and
 * ant moves wrap toroidally over it.  steps is the run's clock — total ant-steps
 * since the last reset; it shares the sim's step units, so it lives here with
 * the grid it counts (shown as 'steps' in the HUD). */
typedef struct {
    uint8_t   cells[MAX_ROWS][MAX_COLS];  /* per-cell colour-state, 0..n_states-1   */
    int       rows, cols;                  /* live grid size in cells (≤ MAX_*)      */
    long long steps;                       /* total ant-steps taken — the run clock  */
} Board;

/* Rule — the turmite's turn-program: the "program" of the 2-D Turing machine.
 * turns[s] is 'R' or 'L', the 90° turn an ant makes on a cell in colour-state s;
 * the cell then advances to (s+1) % n_states.  "RL" is Langton's original (right
 * on 0, left on 1), which self-organises into a highway after ~10k steps; longer
 * tables are multi-colour "turmites" (Wolfram, NKS 2002), and some turn-tables
 * can provably COMPUTE — build logic gates (Gajardo, Moreira & Goles 2002).
 * n_states = strlen(turns), clamped to [2, MAX_COLORS]: length 1 just loops the
 * ant in a 4-cycle, and > 8 would overflow the colour palette. */
typedef struct {
    const char *turns;     /* turn per colour-state: 'R'/'L', length n_states     */
    int         n_states;  /* colour-state count = strlen(turns), 2..MAX_COLORS   */
} Rule;

/* Scene — the whole program in one place, as a table of contents:
 *   board   WHAT   — the grid the ants paint
 *   ants    WHAT   — the heads (entries 0..n_ants-1 are active)
 *   rule    the active turn-program governing every step
 *   preset  which PRESETS entry is selected (the n/p menu position)
 *   speed   HOW    — ant-steps advanced per frame ('+'/'-')
 *   paused  WHERE  — run-state
 * Whole-frame orchestrators (tick / reset / resize) take the Scene; every
 * worker below takes only the narrowest sub-type (Ant / Board / Rule), so the
 * data living in one place never re-couples SIMULATION, RENDER and SEED/RESET. */
typedef struct {
    Board board;           /* WHAT  — the grid the ants paint                   */
    Ant   ants[MAX_ANTS];  /* WHAT  — the heads; entries 0..n_ants-1 are active  */
    int   n_ants;          /* how many ants share the board (1..MAX_ANTS)        */
    Rule  rule;            /* the active turn-program governing every step       */
    int   preset;          /* index of the selected PRESETS entry (n/p menu)     */
    int   speed;           /* HOW   — ant-steps advanced per frame (1..STEPS_MAX)*/
    int   paused;          /* WHERE — run-state: nonzero freezes sim_tick        */
} Scene;

/* The single program instance.  The Board's cell array lives INSIDE it, so this
 * is one ~40 KB static object in BSS — no heap, per the no-allocation-after-init
 * rule.  Zero-initialised; main sets speed/n_ants and set_preset fills the rule. */
static Scene g_scene;

/* Platform signal flags — set from signal handlers, so they stay module-level
 * (volatile sig_atomic_t), not on the Scene. */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

/* ── §3 PERFORMANCE ─ monotonic clock; the frame cap lives in §9 ──────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ── §4 COLOR ─ palette setup (the only colour-pair I/O) ──────────────── */

static void color_init(const Rule *rule)
{
    start_color();
    use_default_colors();
    init_pair(CP_ANT,  (COLORS >= 256) ? 231 : COLOR_WHITE,  -1);
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* HUD data  — top    */
    init_pair(CP_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* action keys — bottom */
    /* state 0 = background (space), states 1..N-1 get colour pairs */
    for (int i = 1; i < rule->n_states && i < MAX_COLORS; i++) {
        short fg = (COLORS >= 256) ? PAL256[i] : PAL8[i];
        init_pair(CP_C0 + i, fg, -1);
    }
}

/* ── §5 SIMULATION ─ the per-tick advance: ant_step, sim_tick ─────────── */

/* Pure per-step decisions (no state, no I/O) — the LOGIC the tick is built from. */

/* wrap — positive modulo: fold v into [0, n) even when v < 0.  Models the
 * toroidal grid (an ant leaving one edge re-enters the opposite); also reused to
 * cycle the preset menu. */
static int wrap(int v, int n) { return ((v % n) + n) % n; }

/* turn — rotate a heading 90°: 'R' clockwise (+1), else counter-clockwise
 * (+N_DIRS-1 ≡ -1), modulo the 4 cardinal directions. */
static int turn(int dir, char lr)
{
    return (dir + (lr == 'R' ? 1 : N_DIRS - 1)) % N_DIRS;
}

static void ant_step(Ant *ant, Board *board, const Rule *rule)
{
    /* One Langton step: read this cell's colour, turn per the rule, repaint the
     * cell its next colour, then step forward one cell (wrapping on the torus). */
    int  state = board->cells[ant->r][ant->c];
    char lr    = rule->turns[state % rule->n_states];   /* 'R'/'L' for this colour */

    ant->dir = turn(ant->dir, lr);
    board->cells[ant->r][ant->c] = (uint8_t)((state + 1) % rule->n_states);
    ant->r = wrap(ant->r + DR[ant->dir], board->rows);
    ant->c = wrap(ant->c + DC[ant->dir], board->cols);
}

static void sim_tick(Scene *s)
{
    if (s->paused) return;
    for (int step = 0; step < s->speed; step++) {
        for (int i = 0; i < s->n_ants; i++)
            ant_step(&s->ants[i], &s->board, &s->rule);
    }
    s->board.steps += s->speed;
}

/* ── §6 RENDER ─ state → screen; reads only, never mutates ────────────── */

/* draw_trails — paint the coloured cells the ants have left behind: a cell in
 * colour-state s draws '#' in pair CP_C0 + s.  Only rows 1..rows-2 (row 0 and
 * the last row are the HUD); background state 0 draws a space. */
static void draw_trails(const Scene *s)
{
    const Board *board = &s->board;
    for (int r = 1; r < board->rows - 1; r++) {
        const uint8_t *row = board->cells[r];
        for (int c = 0; c < board->cols - 1; c++) {
            int st = row[c];
            if (st == 0) { mvaddch(r, c, ' '); continue; }
            int cp = CP_C0 + (st % MAX_COLORS);
            attron(COLOR_PAIR(cp));
            mvaddch(r, c, (chtype)(unsigned char)'#');
            attroff(COLOR_PAIR(cp));
        }
    }
}

/* draw_ants — the bold '@' heads, on top of the trails (skipping the HUD rows). */
static void draw_ants(const Scene *s)
{
    const Board *board = &s->board;
    attron(COLOR_PAIR(CP_ANT) | A_BOLD);
    for (int i = 0; i < s->n_ants; i++) {
        int r = s->ants[i].r, c = s->ants[i].c;
        if (r >= 1 && r < board->rows - 1 && c < board->cols - 1)
            mvaddch(r, c, '@');
    }
    attroff(COLOR_PAIR(CP_ANT) | A_BOLD);
}

static void scene_draw(const Scene *s)
{
    draw_trails(s);
    draw_ants(s);
}

static void scene_hud(const Scene *s)
{
    char buf[256];

    /* top row (data): rule, ant count, total steps, speed, run-state */
    snprintf(buf, sizeof buf,
             " %s   ants:%d   steps:%lld   spd:%d   %s",
             PRESETS[s->preset].name, s->n_ants, s->board.steps, s->speed,
             s->paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, s->board.cols - 1);   /* clipped — never wraps off the row */
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row (actions): every interactive key */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvaddnstr(s->board.rows - 1, 0,
              " q:quit  spc:pause  n/p:rule  1-3:ants  r:reset  +/-:speed",
              s->board.cols - 1);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §7 SEED/RESET ─ user-triggered re-init (NOT the tick) ────────────── */

/* spread_ants — place the active ants at fixed spread starting cells (centre +
 * two offset positions) so multiple ants don't all begin on the same square. */
static void spread_ants(Scene *s)
{
    const Board *board = &s->board;
    static const float ROFF[3] = {0.5f, 0.35f, 0.65f};   /* start row, fraction of grid */
    static const float COFF[3] = {0.5f, 0.35f, 0.65f};   /* start col, fraction of grid */
    static const int   DIRS[3] = {0, 1, 3};               /* starting heading per ant    */
    for (int i = 0; i < s->n_ants; i++) {
        s->ants[i].r   = (int)(ROFF[i] * board->rows) % board->rows;
        s->ants[i].c   = (int)(COFF[i] * board->cols) % board->cols;
        s->ants[i].dir = DIRS[i % 3];
    }
}

static void sim_reset(Scene *s)
{
    Board *board = &s->board;
    if (board->rows < 1 || board->cols < 1) return;
    memset(board->cells, 0, sizeof(board->cells));   /* clear every trail to background */
    board->steps = 0;
    spread_ants(s);
}

static void set_preset(Scene *s, int idx)
{
    s->preset = wrap(idx, N_PRESETS);   /* cycle, wrapping at both ends */
    s->rule.turns    = PRESETS[s->preset].rule;
    s->rule.n_states = (int)strlen(s->rule.turns);
    if (s->rule.n_states < 2) s->rule.n_states = 2;                  /* length-1 rules loop */
    if (s->rule.n_states > MAX_COLORS) s->rule.n_states = MAX_COLORS; /* cap to the palette */
    color_init(&s->rule);
    sim_reset(s);
}

/* ── §8 PLATFORM ─ terminal setup/resize + signal handlers ────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
}

static void screen_resize(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    s->board.rows = rows;
    s->board.cols = cols;
    sim_reset(s);
    erase();
}

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}

static void install_signals(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
}

static void cleanup(void) { endwin(); }

/* ── §9 DRIVER ─ setup, input, and the per-tick combine point ─────────── */

/* handle_key — apply one keypress: quit, pause, cycle rule / ant count, reset,
 * or change speed.  Mutates the scene (and the quit flag); NOT part of the tick. */
static void handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': g_running = 0; break;
    case ' ':           s->paused ^= 1; break;
    case 'n':           set_preset(s, s->preset + 1); break;
    case 'p':           set_preset(s, s->preset - 1); break;
    case 'r':           sim_reset(s);                 break;
    case '1': s->n_ants = 1; sim_reset(s); break;
    case '2': s->n_ants = 2; sim_reset(s); break;
    case '3': s->n_ants = 3; sim_reset(s); break;
    case '+': case '=':
        s->speed = (s->speed < STEPS_MAX) ? s->speed * 2 : STEPS_MAX;   /* double, cap */
        break;
    case '-': case '_':
        s->speed = (s->speed > 1) ? s->speed / 2 : 1;                   /* halve, floor 1 */
        break;
    }
}

int main(void)
{
    Scene *s = &g_scene;

    /* ── setup ── */
    install_signals();
    atexit(cleanup);
    screen_init();
    s->speed  = STEPS_DEF;
    s->paused = 0;
    s->n_ants = 1;
    screen_resize(s);
    set_preset(s, 0);

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {              /* SIGWINCH: re-fit the board and reseed */
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize(s);
        }

        int ch;
        while ((ch = getch()) != ERR)
            handle_key(s, ch);

        /* ── per-frame combine ── */
        sim_tick(s);
        erase();
        scene_draw(s);
        scene_hud(s);
        wnoutrefresh(stdscr);
        doupdate();

        /* hold to the fixed TICK_NS frame period (30 fps) */
        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
