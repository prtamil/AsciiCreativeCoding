/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * network_sim.c — SIR epidemic on a Watts-Strogatz small-world network
 *
 * SPLIT DISPLAY
 *   Left  ~60% : network ring — nodes coloured by SIR state
 *   Right ~40% : scrolling stacked epidemic curve (S/I/R over time)
 *
 * N=40 nodes (half the original; ring is readable).
 * Watts-Strogatz: K=4 ring neighbours, 15% rewiring probability.
 *
 * Node symbols
 *   S  grey  ·  susceptible — small, unobtrusive
 *   I  red   *  newly infected (flashes FLASH_TICKS ticks after transition)
 *   I  red   @  infected, settled
 *   R  green +  recovered / immune
 *
 * Edge colours
 *   dim grey      S–S and R–* edges   (background structure)
 *   bright red    any edge touching I  (shows where disease is active)
 *   bright yellow rewired shortcut edge touching I
 *
 * Epidemic curve (right panel)
 *   stacked bar per tick: R (bottom green) → I (middle red) → S (top grey)
 *   scrolls left as time advances; Y axis = node count 0..N
 *
 * R0 = β · <k> / γ — when R0 > 1 epidemic spreads; < 1 it dies
 *
 * Keys:  q quit   ↑↓ β   ←→ γ   r reset   i inject   spc pause
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/network_sim.c \
 *       -o network_sim -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 types  §5 graph
 * §6 SIR     §7 draw   §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : SIR (Susceptible-Infected-Recovered) epidemic model on
 *                  a Watts-Strogatz small-world network.
 *                  Per tick: each I node infects each S neighbour with
 *                  probability β; each I node recovers with probability γ.
 *                  R0 = β·⟨k⟩/γ: epidemic spreads when R0 > 1.
 *
 * Data-structure : Watts-Strogatz construction: start with a K=4 ring graph
 *                  (each node connected to K/2 nearest neighbours on each
 *                  side); rewire each edge with probability p=0.15, replacing
 *                  target with a random node.  Rewired "shortcut" edges create
 *                  the small-world property: short average path length + high
 *                  clustering coefficient.
 *
 * Math           : SIR basic reproduction number: R0 = β·⟨k⟩/γ where
 *                  ⟨k⟩ is the mean degree.  Epidemic threshold R0=1 marks
 *                  the phase transition between extinction and outbreak.
 *                  Node positions on ring: θ_i = 2πi/N, placed in a circle.
 *
 * Rendering      : Split display: left panel shows network ring with node
 *                  colours (S=grey, I=red, R=green) and edges; right panel
 *                  shows scrolling stacked epidemic curve bar chart.
 *
 * References
 * ──────────
 *   ── Small-world networks (§5 graph construction) ────────────────
 *   [1] Watts, D. J. & Strogatz, S. H. (1998), "Collective dynamics
 *       of small-world networks", Nature 393, pp. 440-442 — THE
 *       small-world paper.  Defines the K-ring + rewire construction
 *       implemented in §5.  ~3 pages; the best place to start.
 *   [2] Newman, M. E. J. (2010), "Networks: An Introduction", Oxford
 *       University Press — the network-science textbook.  Ch. 15
 *       covers small-world; Ch. 17 covers degree distributions
 *       relevant to the ⟨k⟩ in our R0 formula.
 *   [3] Barabási, A.-L. & Albert, R. (1999), "Emergence of scaling
 *       in random networks", Science 286, pp. 509-512 — the SCALE-
 *       FREE network model.  Read alongside [1] to see how a
 *       different rewiring rule produces a different "interesting"
 *       network topology, and how each affects epidemic spread.
 *
 *   ── SIR epidemic models (§6 SIR transitions) ────────────────────
 *   [4] Kermack, W. O. & McKendrick, A. G. (1927), "A contribution to
 *       the mathematical theory of epidemics", Proc. Royal Society A
 *       115, pp. 700-721 — the ORIGINAL SIR model.  Defines the
 *       three-compartment system and the threshold theorem (R0 > 1).
 *   [5] Anderson, R. M. & May, R. M. (1991), "Infectious Diseases of
 *       Humans: Dynamics and Control", Oxford University Press — the
 *       bible of mathematical epidemiology.  Ch. 2 derives the
 *       R0 = β·⟨k⟩/γ used in the HUD.
 *   [6] Keeling, M. J. & Rohani, P. (2008), "Modeling Infectious
 *       Diseases in Humans and Animals", Princeton University Press
 *       — modern practical textbook.  Ch. 3 covers stochastic SIR
 *       in discrete time, exactly what §6 simulates.
 *   [7] Diekmann, O. & Heesterbeek, J. A. P. (2000), "Mathematical
 *       Epidemiology of Infectious Diseases: Model Building, Analysis
 *       and Interpretation", Wiley — for the deterministic ODE limit
 *       (dS/dt = -βSI/N) that this stochastic simulation approximates
 *       at large N.
 *
 *   ── Epidemics on networks (combines §5 + §6) ────────────────────
 *   [8] Pastor-Satorras, R., Castellano, C., Van Mieghem, P. &
 *       Vespignani, A. (2015), "Epidemic processes in complex
 *       networks", Reviews of Modern Physics 87(3), pp. 925-979 — the
 *       definitive review of SIR/SIS dynamics on networks of
 *       different topologies.  Pairs [1] and [4] into one paper.
 *
 *   ── Stochastic simulation algorithm ─────────────────────────────
 *   [9] Gillespie, D. T. (1977), "Exact stochastic simulation of
 *       coupled chemical reactions", J. Physical Chemistry 81(25),
 *       pp. 2340-2361 — the EXACT continuous-time algorithm.  Our
 *       fixed-tick synchronous update is a discrete-time
 *       approximation; Gillespie is what you'd use if rates were
 *       very heterogeneous or you cared about exact timing.
 *
 *   ── Rendering ───────────────────────────────────────────────────
 *  [10] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       the line-rasterization algorithm used by the edge draw in §7
 *       (with the directional-glyph trick '\' '/' '-' '|').
 *
 *   ── Online quick references ─────────────────────────────────────
 *  [11] https://en.wikipedia.org/wiki/Small-world_network — covers
 *       W-S, Newman-Watts variants, and the clustering coefficient.
 *  [12] https://en.wikipedia.org/wiki/Compartmental_models_in_epidemiology
 *       — SIR, SIS, SEIR, SIRS variants with the standard ODE
 *       formulations and threshold derivations.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every individual is in exactly one of three boxes: S (susceptible), I
 * (infected), R (recovered).  Each tick, each I rolls the dice once
 * for recovery (probability γ) and once per infected→susceptible edge
 * for transmission (probability β).  The single most important
 * derived number is R0 = β·⟨k⟩/γ — when R0 > 1 the epidemic explodes,
 * < 1 it dies.  The network shape (small-world: K=4 ring with 15%
 * rewired shortcuts) controls how fast the wavefront circles the ring
 * versus skipping across via shortcuts.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Watts-Strogatz construction:
 *       a. Ring lattice: connect each node i to its K/2=2 nearest
 *          neighbours on each side (K=4 total).
 *       b. For each ring edge (i, i+k), with probability p=0.15,
 *          replace target i+k with a random other node, marking
 *          topology.rewired[i][new_j] = true.
 *  2. Layout: place node i on circle at angle 2π·i/N − π/2 (12
 *     o'clock start), radius RING_FRAC · half-min-extent.
 *  3. SIR tick (synchronous; staged update via nxt[]):
 *       For each I node:
 *         - With probability γ: mark nxt[i] = R.
 *         - For each neighbour j with state S: with probability β,
 *           mark nxt[j] = I, set flash[j] = FLASH_TICKS=6.
 *       commit_state(nxt → epi.state).
 *  4. Append (S_count, I_count, R_count) to history (circular
 *     buffer, capacity HIST_LEN=500); track all-time peak I.
 *  5. Render network: edges first (dim grey for S–S/R–*, hot red for
 *     any I-touching, bright yellow for hot rewired shortcuts).
 *     Nodes on top: '.' grey for S, '@' red bold for I (or '*'
 *     yellow during flash), '+' green for R.
 *  6. Render chart: each tick = one column, stacked R(bottom green
 *     '-') / I(red '#') / S(top grey '='); scrolls left as new ticks
 *     append at right edge.  Y-axis labels at 0/N/4/N/2/3N/4/N.
 *  7. HUD row: live β, γ, R0 (coloured by threshold), <k>, S/I/R
 *     counts, phase label (READY/SEEDED/GROWING/WANING/PLATEAU/EXTINCT).
 *
 * KEY FORMULAS
 * ────────────
 *  Mean degree    ⟨k⟩ = (1/N) · Σ_i deg(i)        (= K = 4 if no rewire)
 *  Reproduction   R0 = β · ⟨k⟩ / γ
 *  Threshold      R0 > 1 → epidemic;  R0 < 1 → extinction
 *  Tick recover   I → R with prob γ (uniform random)
 *  Tick transmit  for each (I,S) edge: S → I with prob β
 *  Ring node pos  (cx + r·cos θ_i,  cy + r·sin θ_i),  θ_i = 2πi/N - π/2
 *  Rewire prob    p = WS_P = 0.15 per directed edge
 *  Phase detect   prev_i = hist[(head-2+HIST_LEN) % HIST_LEN]
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first.  Read
 *      algorithms/graph_search.c first if adjacency lists are new —
 *      that file has the same Watts-Strogatz layout idea (random
 *      graph + force layout) for a different purpose.  Read
 *      flocking/flocking.c if you want a totally different
 *      "interacting agents on a graph" example.
 *   2. §6 SIR — the per-tick transition rule + staged update.
 *      THE HEART of this file.  Each driver carries a pseudocode
 *      docblock; read those before the bodies.
 *   3. §5 graph — Watts-Strogatz construction (see References [1]).
 *   4. §7 draw — split layout (network ring + epidemic curve).
 *   5. §1-§3, §4, §8 — config / clock / colour / data types / app loop.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   sc                          the Scene struct passed by pointer to
 *                               every sim / draw / input function.
 *   sc->epi.state[i]            current SIR state of node i.
 *   nxt[]                       sir_tick's local scratch — staged
 *                               next-tick state for SYNCHRONOUS update.
 *   sc->topology.adj[i][j]      true iff edge i↔j exists.  Symmetric
 *                               (undirected graph).
 *   sc->topology.rewired[i][j]  true iff this is a "shortcut" rewired
 *                               edge (used for yellow highlighting
 *                               of small-world long-range links).
 *   sc->beta, sc->gamma         transmission + recovery rates ∈ [0, 1].
 *   N_NODES                     40.
 *   WS_K = 4                    ring lattice degree (each node has
 *                               K/2 neighbours on each side).
 *   WS_P = 0.15                 rewiring probability per edge.
 *   sc->history.s/i/r[]         rolling (S, I, R) history for the
 *                               right-panel curve.  Capacity HIST_LEN=500.
 *
 * Background you need
 * ───────────────────
 *   - Adjacency-matrix graph storage.
 *   - Probability: a coin with P(heads) = β, flipped per (I, S)
 *     edge per tick.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Differential equations.  The continuous SIR ODE
 *     (dS/dt = -βSI/N etc.) is the limit of large-N stochastic
 *     simulation; we use the stochastic version directly.
 *   - Stochastic differential equations / master equations.
 *   - Real epidemiology (R_eff vs R_0, contact tracing,
 *     interventions).  We use the simplest textbook model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_NODES     40        /* ring nodes — readable without hairball    */
#define WS_K         4        /* ring degree (2 neighbours each side)      */
#define WS_P         0.15f    /* Watts-Strogatz rewiring probability       */

#define BETA_INIT    0.040f   /* infection prob per S-I edge per tick      */
#define GAMMA_INIT   0.025f   /* recovery prob per I node per tick         */
#define BETA_STEP    0.005f
#define GAMMA_STEP   0.005f

#define FLASH_TICKS  6        /* ticks a newly-infected node shows as '*'  */
#define HIST_LEN   500        /* rolling history for epidemic curve        */

#define CELL_W       8        /* pixels per terminal column                */
#define CELL_H      16        /* pixels per terminal row                   */
#define NET_FRAC     0.58f    /* fraction of screen width for network      */
#define RING_FRAC    0.44f    /* ring radius / min(half-width, half-height)*/
#define FPS         15

/* HUD layout — rows reserved at top/bottom of the screen for status bars.
 * Top bar carries DATA (β, γ, R0, S/I/R counts, SIR proportion bar);
 * bottom bar carries ACTIONS (key hints).  layout_ring + draw_network +
 * draw_chart all clip into the band between the two bars. */
#define HUD_TOP_ROWS 2        /* row 0: params  ;  row 1: SIR proportion bar */
#define HUD_BOT_ROWS 1        /* last row: key-hint action bar               */

/*
 * SIR — the three compartments of the Kermack-McKendrick model.
 *
 * Intent
 *   Each node sits in EXACTLY ONE of these states at any moment.
 *   The whole simulation is the per-tick rule for transitioning
 *   between them:
 *
 *       ┌───┐  prob_roll(β) per I-S edge   ┌───┐  prob_roll(γ)   ┌───┐
 *       │ S │ ──────────────────────────► │ I │ ──────────────► │ R │
 *       └───┘                             └───┘                 └───┘
 *
 *   S → I  is the ONLY transition driven by neighbours (graph-mediated).
 *   I → R  is independent per node — one die roll per I per tick.
 *   R → *  is ABSENT — recovered nodes are permanently immune in
 *          the basic SIR model.  Variants (SIRS, SEIR) reintroduce
 *          edges or add intermediate states; not implemented here.
 *
 * Why an enum (not int 0/1/2)
 *   Self-documenting at every comparison site: `state[i] == I_STATE`
 *   reads as English; the compiler catches typos that bare ints
 *   would silently accept.  Zero cost — the enum's underlying type
 *   is int.
 *
 * Reference: Kermack & McKendrick (1927) [4] for the three-
 *   compartment model; Anderson & May (1991) [5] §2 for the
 *   derivation of R0 = β·⟨k⟩/γ from these transition rules.
 */
typedef enum {
    S_STATE,    /* Susceptible — can be infected by an I neighbour       */
    I_STATE,    /* Infected    — transmits; rolls γ each tick to recover */
    R_STATE,    /* Recovered   — immune; absorbing state in basic SIR    */
} SIR;

enum {
    CP_S=1, CP_I, CP_I_FLASH, CP_R,
    CP_EDGE_DIM, CP_EDGE_HOT, CP_EDGE_REWIRE,
    CP_HUD,                /* top data bar  — bright yellow + A_BOLD */
    CP_HINT,               /* bottom action bar — bright cyan + A_BOLD */
    CP_BAR_S, CP_BAR_I, CP_BAR_R,
    CP_DIVIDER,
};

/* ===================================================================== */
/* §2  clock                                                              */
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
    struct timespec ts = { ns/1000000000LL, ns%1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_S,          246,  -1);  /* grey — susceptible            */
        init_pair(CP_I,          196,  -1);  /* red  — infected               */
        init_pair(CP_I_FLASH,    226,  -1);  /* bright yellow — just infected */
        init_pair(CP_R,           46,  -1);  /* green — recovered             */
        init_pair(CP_EDGE_DIM,   244,  -1);  /* very dark — background edges  */
        init_pair(CP_EDGE_HOT,   196,  -1);  /* red  — edges touching I       */
        init_pair(CP_EDGE_REWIRE,220,  -1);  /* yellow — rewired + hot        */
        init_pair(CP_HUD,        226,  -1);  /* bright yellow — top data bar  */
        init_pair(CP_HINT,        51,  -1);  /* bright cyan — bottom hint bar */
        init_pair(CP_BAR_S,      246,  -1);
        init_pair(CP_BAR_I,      196,  -1);
        init_pair(CP_BAR_R,       46,  -1);
        init_pair(CP_DIVIDER,    246,  -1);
    } else {
        init_pair(CP_S,         COLOR_WHITE,   -1);
        init_pair(CP_I,         COLOR_RED,     -1);
        init_pair(CP_I_FLASH,   COLOR_YELLOW,  -1);
        init_pair(CP_R,         COLOR_GREEN,   -1);
        init_pair(CP_EDGE_DIM,  COLOR_WHITE,   -1);
        init_pair(CP_EDGE_HOT,  COLOR_RED,     -1);
        init_pair(CP_EDGE_REWIRE,COLOR_YELLOW, -1);
        init_pair(CP_HUD,       COLOR_YELLOW,  -1);
        init_pair(CP_HINT,      COLOR_CYAN,    -1);
        init_pair(CP_BAR_S,     COLOR_WHITE,   -1);
        init_pair(CP_BAR_I,     COLOR_RED,     -1);
        init_pair(CP_BAR_R,     COLOR_GREEN,   -1);
        init_pair(CP_DIVIDER,   COLOR_WHITE,   -1);
    }
}

/* ===================================================================== */
/* §4  data types — Vec2, Topology, EpiState, EpiHistory, Scene           */
/* ===================================================================== */

/*
 * Vec2 — 2-D pixel-space position.
 *
 * Intent
 *   One node's screen position in PIXEL coordinates (CELL_W × CELL_H
 *   sub-pixels per terminal cell).  Stored as float because the ring
 *   layout uses trigonometry — integer coords would accumulate
 *   rounding error at every layout recompute and cause nodes to
 *   "jitter" on resize.
 *
 *   Used only for node positions (Scene.pos[]) and the (centre,
 *   radius) of the ring layout in §5.  Force-directed layouts in
 *   other files use Vec2 for velocities too; we don't here.
 *
 * Convention
 *   x : EASTWARD pixel coordinate  (positive → right)
 *   y : SOUTHWARD pixel coordinate (positive → DOWN — terminal idiom)
 *
 *   The y-down convention is why place_node_on_ring's angle starts
 *   at −π/2: that places node 0 at the TOP of the ring in screen
 *   space, matching the reader's natural "12 o'clock" expectation.
 *
 * Why pass-by-value
 *   8 bytes; fits in registers on every modern ABI.  No aliasing
 *   concerns when both centre and per-node positions live in the
 *   same caller frame.  place_node_on_ring takes centre by value
 *   and returns a fresh Vec2 the same way.
 */
typedef struct {
    float x;        /* eastward  pixel coordinate (right is positive)  */
    float y;        /* southward pixel coordinate (down  is positive)  */
} Vec2;

/*
 * Topology — the IMMUTABLE Watts-Strogatz graph structure.
 *
 * Intent
 *   Captures the network built ONCE at startup and frozen for the
 *   life of the program: which nodes are connected (adj), and which
 *   of those connections are RANDOM SHORTCUTS introduced by the
 *   rewire step (rewired).  Stays constant across epi_reset ('r')
 *   and epi_inject ('i'); only the SIR dynamics get wiped on reset,
 *   never the graph.
 *
 *   This separation matters pedagogically: the user can rerun the
 *   epidemic many times on the SAME graph and watch how outbreak
 *   trajectories differ purely due to stochastic rolls — isolating
 *   the variance contributed by RANDOMNESS from the variance
 *   contributed by TOPOLOGY.
 *
 * Why two parallel matrices
 *   adj[i][j]      — true iff edge (i, j) exists; symmetric (undirected).
 *   rewired[i][j]  — true iff that edge replaced a ring edge during
 *                    rewire_edges; strict SUBSET of adj.
 *
 *   Keeping `rewired` as a separate matrix lets the renderer paint
 *   shortcut edges in BRIGHT YELLOW (when hot) — the single most
 *   informative visual feature for understanding small-world
 *   acceleration.  Recomputing "is this a shortcut?" per frame
 *   would require remembering the original ring construction.
 *
 * Why ADJACENCY MATRIX (not adjacency list)
 *   At N=40 the matrix is 1600 bytes; two parallel matrices = 3200
 *   bytes total.  Adjacency-list storage would save space at large
 *   N but the matrix gives O(1) edge query (used heavily in
 *   sir_tick's "for each neighbour" loop and in draw_network_edges).
 *   For N ≫ 1000 you would switch to a list; at N=40 the matrix
 *   wins cleanly.
 *
 * Construction sequence (§5)
 *   topology_clear            — zero both matrices
 *   connect_ring_neighbours   — K-ring lattice (Step 1 of W-S)
 *   rewire_edges              — replace ring edges with prob WS_P (Step 2)
 *
 * Members
 *   adj    [i][j]   symmetric undirected adjacency
 *   rewired[i][j]   subset of adj; true iff edge is a small-world shortcut
 *
 * Invariants
 *   adj[i][j]     == adj[j][i]                   — symmetry
 *   rewired[i][j] == rewired[j][i]               — symmetry
 *   rewired[i][j] → adj[i][j]                    — subset
 *   adj[i][i]     == false                       — no self-loops
 *
 * References
 *   [1] Watts & Strogatz (1998) — the original construction.  Our
 *       implementation is literal; see also [2] §15 for the
 *       clustering-coefficient + path-length analysis.
 */
typedef struct {
    bool adj    [N_NODES][N_NODES];     /* symmetric, no self-loops      */
    bool rewired[N_NODES][N_NODES];     /* subset of adj — shortcut flag */
} Topology;

/*
 * EpiState — per-node epidemiological state.
 *
 * Intent
 *   The CURRENT compartment of each node, plus a short "just got
 *   infected" countdown that drives the bright-yellow flash glyph
 *   in the renderer.  Together these capture everything the renderer
 *   needs to draw the network panel: state[] picks the COLOUR family,
 *   flash[] picks WHICH variant (fresh '*' vs settled '@').
 *
 *   Reset on 'r' (epi_reset clears all to S except one seed I);
 *   extended on 'i' (epi_inject flips a random S → I).
 *
 * Why a SEPARATE flash counter (not derived from state)
 *   "Just infected" is a TEMPORAL property — the same I_STATE
 *   transitioning two ticks apart, just one rendered with extra
 *   emphasis.  Encoding it as a second state (e.g. I_FRESH_STATE)
 *   would mean the SIR update logic has to know to demote
 *   I_FRESH → I after the countdown, complicating sir_tick.  A
 *   separate decrement loop (decrement_flash_counters) keeps the
 *   compartment logic and the rendering hint cleanly separate.
 *
 * Why INT (not bool) for flash
 *   The flash lasts FLASH_TICKS frames, not one.  An int countdown
 *   that the renderer compares with > 0 captures both "is it
 *   flashing?" (the bool we need) and "for how many more ticks?"
 *   (the state machine that drives it).
 *
 * Members
 *   state[i]   one of S_STATE, I_STATE, R_STATE.  Mutated synchronously
 *              via the nxt[] scratch in sir_tick (never in place).
 *   flash[i]   FLASH_TICKS countdown.  Renderer maps:
 *                flash > 0  AND state == I  →  '*' bright yellow
 *                flash == 0 AND state == I  →  '@' red
 *                otherwise                  →  ignored (no flash glyph)
 *
 *   Synchronous-update scratch (`nxt[N_NODES]`) is a local in
 *   sir_tick — no cross-frame meaning, not in this struct.
 *
 * Invariants
 *   state[i] ∈ {S_STATE, I_STATE, R_STATE}
 *   flash[i] ≥ 0
 *   flash[i] > 0  →  state[i] == I_STATE        (only I nodes flash)
 */
typedef struct {
    SIR state[N_NODES];     /* current compartment per node                 */
    int flash[N_NODES];     /* FLASH_TICKS countdown — render hint for new I */
} EpiState;

/*
 * EpiHistory — circular buffer of per-tick (S, I, R) counts.
 *
 * Intent
 *   Drives the right-panel scrolling stacked bar chart.  Each tick
 *   appends one (S, I, R) triple via history_record; the chart shows
 *   the most recent `data_w` columns scrolling left as time advances.
 *   After HIST_LEN ticks the buffer wraps and overwrites the oldest
 *   sample — the chart still shows the most recent window, just with
 *   bounded memory.
 *
 *   peak_i is a MONOTONIC max of the I count over the entire run;
 *   the chart marks it with "pk" so the all-time outbreak peak is
 *   visible even after I has dropped back down.
 *
 * Why a CIRCULAR buffer (not a fixed array)
 *   Outbreak runs can last thousands of ticks (slow γ).  A linear
 *   array would either grow unboundedly or truncate on overflow.
 *   The circular form gives bounded memory + automatic forgetting
 *   of ancient ticks the chart can't show anyway.
 *
 * Why SEPARATE s/i/r arrays (not array of struct)
 *   Slightly better cache behaviour for the column-major chart
 *   render (reads all S values, then all I, then all R).  More
 *   importantly: `h->i[bi]` reads as "I count at bin bi", matching
 *   the visual idiom of the stacked bar chart.  At N=40 with three
 *   arrays of 500 ints = 6 KB total, the whole buffer fits in L1
 *   either way.
 *
 * Members
 *   s[k]    S count for tick k                 (k indexed mod HIST_LEN)
 *   i[k]    I count for tick k
 *   r[k]    R count for tick k
 *   head    NEXT write position; 0 ≤ head < HIST_LEN
 *   n       current fill count; 0 ≤ n ≤ HIST_LEN
 *   peak_i  MONOTONIC max of I across the entire run
 *
 * Invariants
 *   0 ≤ head < HIST_LEN
 *   0 ≤ n    ≤ HIST_LEN
 *   The OLDEST valid sample sits at (head − n + HIST_LEN) % HIST_LEN.
 *   The NEWEST valid sample sits at (head − 1 + HIST_LEN) % HIST_LEN.
 *   peak_i never decreases (history_record only assigns if i > peak_i).
 *   s[k] + i[k] + r[k] == N_NODES   for every valid k (each node
 *   sits in exactly one compartment per tick — partition invariant).
 */
typedef struct {
    int s[HIST_LEN];        /* S count per recorded tick                 */
    int i[HIST_LEN];        /* I count per recorded tick                 */
    int r[HIST_LEN];        /* R count per recorded tick                 */
    int head;               /* next write slot; 0 ≤ head < HIST_LEN      */
    int n;                  /* fill count;      0 ≤ n    ≤ HIST_LEN      */
    int peak_i;             /* monotonic max of I across the entire run  */
} EpiHistory;

/*
 * Scene — owns ALL persistent simulation + render state for one run.
 *
 * Intent
 *   One instance lives in main() and is passed by pointer to every
 *   simulation, draw, and input function.  No file-scope mutables
 *   for the simulation; signal-handler flags (g_quit, g_resize) in
 *   §8 stay as file-scope globals because POSIX signal handlers
 *   can't be passed a pointer.
 *
 *   The struct is partitioned into THREE LIFETIMES that map directly
 *   to the user's key bindings:
 *     (1) Topology + layout — built ONCE at startup; refreshed only
 *         on terminal resize (layout_ring); never touched by 'r'.
 *     (2) Epidemic dynamics — wiped on every 'r' (epi_reset); the
 *         user can rerun the SAME graph many times this way.
 *     (3) Terminal extent  — read each frame from LINES/COLS so
 *         resize is picked up.
 *
 * Why a struct (not a global blob)
 *   - Every function's signature now documents what it READS / WRITES
 *     (look at the const-ness of the Scene* parameter).
 *   - Two simulations could coexist (e.g. an A/B comparison panel)
 *     without aliasing.  Not used today; the door is open.
 *   - Removes ~15 file-scope globals from the previous version,
 *     keeping the module surface clean.
 *
 * Sub-structures (defined just above)
 *   Topology    immutable graph (adj + rewired)
 *   EpiState    per-node compartment + flash counter
 *   EpiHistory  circular buffer of (S, I, R) counts
 *
 * Members
 *   ── Graph (built once, survives reset) ──────────────────────────
 *   topology       Watts-Strogatz adjacency + shortcut flags
 *   pos[N_NODES]   ring layout positions in pixel space; recomputed
 *                  by layout_ring on startup and on every resize.
 *
 *   ── Epidemic dynamics (cleared on 'r' reset) ───────────────────
 *   epi            per-node SIR state + flash countdown
 *   history        rolling (S, I, R) counts for the chart
 *   beta           transmission probability per S-I edge per tick;
 *                  read in try_transmission via prob_roll(beta).
 *                  Valid range [0, 1]; clamp01 enforces in handle_input.
 *   gamma          recovery probability per I node per tick;
 *                  read in try_recovery via prob_roll(gamma).
 *                  Valid range [0, 1]; clamp01 enforces in handle_input.
 *   tick           frame counter since last reset (0 at startup and
 *                  after epi_reset); shown in HUD.
 *   paused         pause flag; sir_tick is a no-op while true.
 *                  Toggle: SPACE.
 *
 *   ── Terminal extent (refreshed each frame) ─────────────────────
 *   rows, cols     LINES, COLS snapshot for the current frame
 *
 * Derived quantities (computed on demand, NOT stored)
 *   ⟨k⟩    topology_mean_degree(&topology)
 *   R0     β · ⟨k⟩ / γ           (HUD computes each frame)
 *   S/I/R  count_state(&epi, …)  (HUD + sir_tick each compute)
 *
 *   Keeping these derived rather than cached avoids stale-cache
 *   bugs: any update to β/γ or epi.state is immediately visible
 *   in the next frame's HUD with no invalidation logic.
 *
 * Invariants
 *   0 ≤ beta  ≤ 1
 *   0 ≤ gamma ≤ 1
 *   tick    ≥ 0
 *   rows    ≥ HUD_TOP_ROWS + HUD_BOT_ROWS + 1   (else simulation panel
 *                                                is empty; layout_ring
 *                                                degenerates to radius 0)
 */
typedef struct {
    Topology    topology;       /* immutable graph                          */
    Vec2        pos[N_NODES];   /* ring layout positions, pixel space       */

    EpiState    epi;            /* per-node compartment + flash             */
    EpiHistory  history;        /* rolling (S, I, R) counts                 */
    float       beta;           /* transmission rate, [0, 1]                */
    float       gamma;          /* recovery rate,     [0, 1]                */
    int         tick;           /* ticks since last reset, ≥ 0              */
    bool        paused;         /* sir_tick is a no-op while true           */

    int         rows;           /* LINES this frame                         */
    int         cols;           /* COLS  this frame                         */
} Scene;

/* ── shared helpers ────────────────────────────────────────────── */

/*
 * prob_roll — coin flip with probability p of returning true.
 *
 *   Pseudocode:  return rand() / RAND_MAX < p
 *
 *   The single primitive that drives every probabilistic event in
 *   the file:
 *     - rewire decision (§5 graph)
 *     - recovery roll   (§6 SIR)
 *     - transmission roll (§6 SIR)
 *
 *   Naming it makes the algorithm bodies read as
 *   "if (prob_roll(γ)) recover; ..." rather than an opaque
 *   `rand()/RAND_MAX` expression.  Caller is responsible for
 *   keeping p in [0, 1] — values outside are clamped at the
 *   THRESH_MIN/MAX bounds in handle_input.
 */
static inline bool prob_roll(float p)
{
    return (float)rand() / (float)RAND_MAX < p;
}

/*
 * px_col / px_row — pixel-space → cell-space rounding.
 *   The renderer needs (col, row) cell indices; node positions live
 *   in pixel coordinates (CELL_W × CELL_H pixels per cell).  +0.5f
 *   rounds to the nearest cell.
 */
static inline int px_col(float px) { return (int)(px / (float)CELL_W + 0.5f); }
static inline int px_row(float py) { return (int)(py / (float)CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  graph — Watts-Strogatz construction + ring layout                  */
/* ===================================================================== */

/* ── Topology construction ───────────────────────────────────────── */

/*
 * topology_clear — zero adjacency and rewire flags.  Step 0 of any
 * (re)build; idempotent.
 */
static void topology_clear(Topology *t)
{
    memset(t->adj,     0, sizeof t->adj);
    memset(t->rewired, 0, sizeof t->rewired);
}

/*
 * connect_ring_neighbours — Step 1: build the K-ring lattice.
 *
 *   Pseudocode:
 *     for each node i in 0..N-1:
 *       for each k in 1..K/2:
 *         adj[i][(i+k) mod N] := adj[(i+k) mod N][i] := true
 *
 *   Each node ends up with exactly K neighbours (K/2 on each side of
 *   the ring).  This is the regular-lattice starting point: high
 *   clustering, long path lengths.  rewire_edges introduces the
 *   shortcuts that turn it into a small-world graph.
 */
static void connect_ring_neighbours(Topology *t)
{
    for (int i = 0; i < N_NODES; i++)
        for (int k = 1; k <= WS_K / 2; k++) {
            int j = (i + k) % N_NODES;
            t->adj[i][j] = t->adj[j][i] = true;
        }
}

/*
 * rewire_one_edge — Step 2.b: try to replace edge (i → old_j) with
 * (i → new_j) for some random new_j.
 *
 *   Pseudocode:
 *     for up to N tries:
 *       new_j := uniform random in [0, N)
 *       if new_j != i AND no existing edge (i, new_j):
 *         delete (i, old_j);  add (i, new_j);  mark new_j as rewired
 *         return
 *     give up — graph too dense to find a free slot
 *
 *   The retry budget caps work in pathological cases (almost-complete
 *   graphs); at WS_P=0.15 and K=4 the loop almost never fails.
 */
static void rewire_one_edge(Topology *t, int i, int old_j)
{
    for (int tries = 0; tries < N_NODES; tries++) {
        int new_j = rand() % N_NODES;
        if (new_j == i || t->adj[i][new_j]) continue;
        t->adj[i][old_j] = t->adj[old_j][i] = false;
        t->adj[i][new_j] = t->adj[new_j][i] = true;
        t->rewired[i][new_j] = t->rewired[new_j][i] = true;
        return;
    }
    /* dense graph: leave the original ring edge intact */
}

/*
 * rewire_edges — Step 2: roll WS_P for each ring edge, redirecting
 * target to a random non-self non-neighbour.
 *
 *   Pseudocode:
 *     for each i, k in (0..N-1) × (1..K/2):
 *       if prob_roll(WS_P):
 *         rewire_one_edge(t, i, (i+k) mod N)
 *
 *   Iterates over the SAME (i, i+k) pairs that connect_ring_neighbours
 *   created, ensuring each ring edge gets exactly one rewire chance.
 */
static void rewire_edges(Topology *t)
{
    for (int i = 0; i < N_NODES; i++)
        for (int k = 1; k <= WS_K / 2; k++)
            if (prob_roll(WS_P))
                rewire_one_edge(t, i, (i + k) % N_NODES);
}

/*
 * topology_build_ws — Watts-Strogatz orchestrator.
 *
 *   Pseudocode:
 *     1. topology_clear           — zero adjacency
 *     2. connect_ring_neighbours  — K-ring lattice
 *     3. rewire_edges             — replace ring edges with prob WS_P
 *
 *   Reference: Watts & Strogatz (1998) [1].
 */
static void topology_build_ws(Topology *t)
{
    topology_clear(t);
    connect_ring_neighbours(t);
    rewire_edges(t);
}

/*
 * topology_mean_degree — ⟨k⟩ over all nodes.
 *
 *   ⟨k⟩ = (1/N) · Σ_i deg(i)
 *
 *   With pure ring (no rewires) this equals K.  Rewires preserve
 *   total edges (each delete is paired with an add), so ⟨k⟩ stays
 *   at K even after rewiring.  Used in the HUD as the denominator
 *   of R0 = β·⟨k⟩/γ.
 */
static float topology_mean_degree(const Topology *t)
{
    int total = 0;
    for (int i = 0; i < N_NODES; i++)
        for (int j = 0; j < N_NODES; j++)
            if (t->adj[i][j]) total++;
    return (float)total / (float)N_NODES;
}

/* ── Ring layout (geometry) ──────────────────────────────────────── */

/*
 * compute_ring_geometry — derive (centre, radius) for the network panel.
 *
 *   The network panel occupies the left NET_FRAC of the screen, and
 *   vertically the band between HUD_TOP_ROWS and rows - HUD_BOT_ROWS.
 *   The ring centre is the centre of that rectangle; the radius is
 *   RING_FRAC × half-min-extent so the ring fits inside with margin.
 */
static void compute_ring_geometry(int rows, int cols,
                                  Vec2 *centre, float *radius)
{
    int   net_w = (int)(cols * NET_FRAC);
    float pw    = (float)(net_w * CELL_W);
    float ph    = (float)((rows - HUD_TOP_ROWS - HUD_BOT_ROWS) * CELL_H);
    centre->x   = pw * 0.5f;
    centre->y   = ph * 0.5f + (float)(HUD_TOP_ROWS * CELL_H);
    *radius     = RING_FRAC * (pw < ph ? pw : ph) * 0.5f;
}

/*
 * place_node_on_ring — pure geometry: node i's pixel position.
 *
 *   θ_i  = 2π·i/N − π/2          (start at 12 o'clock, sweep clockwise)
 *   pos  = centre + radius · (cos θ_i, sin θ_i)
 *
 *   The −π/2 phase shift puts node 0 at the TOP of the ring rather
 *   than at the right — matches the natural reader expectation.
 */
static Vec2 place_node_on_ring(int i, int n, Vec2 centre, float radius)
{
    float angle = 2.f * (float)M_PI * (float)i / (float)n
                - (float)M_PI / 2.f;
    Vec2  p = { centre.x + radius * cosf(angle),
                centre.y + radius * sinf(angle) };
    return p;
}

/*
 * layout_ring — orchestrator: compute geometry, then place each node.
 *
 *   Called at startup AND on every resize so node positions track
 *   the current terminal extent.  The TOPOLOGY (edges) is unchanged
 *   by resize — only the layout.
 */
static void layout_ring(Scene *sc)
{
    Vec2  centre;
    float radius;
    compute_ring_geometry(sc->rows, sc->cols, &centre, &radius);
    for (int i = 0; i < N_NODES; i++)
        sc->pos[i] = place_node_on_ring(i, N_NODES, centre, radius);
}

/* ===================================================================== */
/* §6  SIR dynamics                                                       */
/* ===================================================================== */

/*
 * count_state — count nodes currently in `target` compartment.
 *
 *   Single pass over EpiState.state.  The compartment is a parameter
 *   so one function covers all three counts — no per-compartment
 *   count_s / count_i / count_r split.
 */
static int count_state(const EpiState *epi, SIR target)
{
    int n = 0;
    for (int i = 0; i < N_NODES; i++)
        if (epi->state[i] == target) n++;
    return n;
}

/*
 * epi_reset — seed a fresh outbreak: all S except one random I.
 *
 *   Pseudocode:
 *     for each i:  state[i] := S;  flash[i] := 0
 *     seed := uniform random in [0, N)
 *     state[seed] := I;  flash[seed] := FLASH_TICKS
 *     clear history;  reset tick counter
 *
 *   Does NOT touch the topology — topology_build_ws runs once at
 *   startup.  Each 'r' keypress draws a fresh seed; topology stays
 *   constant.
 */
static void epi_reset(Scene *sc)
{
    for (int i = 0; i < N_NODES; i++) {
        sc->epi.state[i] = S_STATE;
        sc->epi.flash[i] = 0;
    }
    int seed = rand() % N_NODES;
    sc->epi.state[seed] = I_STATE;
    sc->epi.flash[seed] = FLASH_TICKS;
    sc->tick    = 0;
    sc->history = (EpiHistory){ 0 };
}

/*
 * epi_inject — find any S node and flip it to I (the 'i' key).
 *
 *   Manually seeds additional infections — useful when an outbreak
 *   has died out and you want to study spread again without losing
 *   the existing history.  Bounded retry budget covers the edge
 *   case where no S exists (all already I or R).
 */
static void epi_inject(EpiState *epi)
{
    for (int tries = 0; tries < N_NODES * 2; tries++) {
        int i = rand() % N_NODES;
        if (epi->state[i] == S_STATE) {
            epi->state[i] = I_STATE;
            epi->flash[i] = FLASH_TICKS;
            return;
        }
    }
}

/*
 * decrement_flash_counters — age the just-infected flash markers.
 *
 *   Every node with flash > 0 decrements by 1 each tick.  When it
 *   reaches 0 the renderer switches the glyph from '*' (yellow flash)
 *   to '@' (red settled).  Runs BEFORE the SIR update so this tick's
 *   new infections start at the full FLASH_TICKS value.
 */
static void decrement_flash_counters(EpiState *epi)
{
    for (int i = 0; i < N_NODES; i++)
        if (epi->flash[i] > 0) epi->flash[i]--;
}

/*
 * try_recovery — one I rolls γ; on success, stages I → R in nxt[].
 *
 *   Probability γ per I node per tick.  Independent of neighbours.
 */
static inline void try_recovery(int i, SIR *nxt, float gamma_rate)
{
    if (prob_roll(gamma_rate))
        nxt[i] = R_STATE;
}

/*
 * try_transmission — one (I, S) edge rolls β; on success, stages
 * S → I in nxt[] and sets the flash counter for the new infectee.
 *
 *   Probability β per (I, S) edge per tick.  Note: the source's state
 *   is read from the CURRENT state[] (the pre-tick snapshot), not
 *   nxt[] — so an I that recovers this tick still transmits this
 *   tick (synchronous semantics).
 */
static inline void try_transmission(int j, EpiState *epi, SIR *nxt,
                                    float beta_rate)
{
    if (prob_roll(beta_rate)) {
        nxt[j]        = I_STATE;
        epi->flash[j] = FLASH_TICKS;
    }
}

/*
 * tick_one_infected — apply recovery + transmission for ONE I node.
 *
 *   Pseudocode:
 *     try_recovery(i, nxt, γ)
 *     for each neighbour j with state[j] == S:
 *       try_transmission(j, epi, nxt, β)
 *
 *   Reads adjacency from the topology; reads neighbour susceptibility
 *   from epi->state (NOT from nxt — synchronous semantics).  If two
 *   I neighbours both try to infect the same S in one tick, both
 *   writes target the same nxt[j] slot and at most one transition
 *   survives — but the new I is "deserved" regardless of which
 *   neighbour's roll happened to succeed.
 */
static void tick_one_infected(int i, const Scene *sc, EpiState *epi,
                              SIR *nxt)
{
    try_recovery(i, nxt, sc->gamma);
    for (int j = 0; j < N_NODES; j++) {
        if (!sc->topology.adj[i][j])  continue;
        if (epi->state[j] != S_STATE) continue;
        try_transmission(j, epi, nxt, sc->beta);
    }
}

/*
 * history_record — append the current (S, I, R) counts; bump peak.
 *
 *   Circular write: head advances mod HIST_LEN; once n reaches
 *   HIST_LEN, head wraps and overwrites the oldest sample (the
 *   chart still shows the most recent data_w samples).
 *   peak_i is monotonic over the run — only ever increases.
 */
static void history_record(EpiHistory *h, int s, int i, int r)
{
    h->s[h->head] = s;
    h->i[h->head] = i;
    h->r[h->head] = r;
    h->head = (h->head + 1) % HIST_LEN;
    if (h->n < HIST_LEN) h->n++;
    if (i > h->peak_i)   h->peak_i = i;
}

/*
 * history_prev_infected — return the I count from the PREVIOUS tick.
 *
 *   Used for phase detection in the HUD (GROWING vs WANING vs
 *   PLATEAU): compare current I count to history_prev_infected.
 *   Returns 0 if there's fewer than 2 samples (no prior tick).
 */
static int history_prev_infected(const EpiHistory *h)
{
    if (h->n < 2) return 0;
    return h->i[(h->head - 2 + HIST_LEN) % HIST_LEN];
}

/*
 * snapshot_state — read epi.state into a scratch SIR[] buffer.
 *
 *   The "read-half" of the synchronous-update pattern: every
 *   per-node transition computed this tick will look at this
 *   immutable snapshot, NEVER at epi.state directly.  This is what
 *   makes the SIR step iteration-order-independent.
 *
 *   See cellular-automata literature on synchronous vs asynchronous
 *   update; Wolfram, "A New Kind of Science" §3 covers it.
 */
static inline void snapshot_state(const EpiState *epi, SIR *out)
{
    memcpy(out, epi->state, sizeof epi->state);
}

/*
 * commit_state — write the staged nxt[] buffer back as the new state.
 *
 *   The "write-half" of the synchronous-update pattern: after every
 *   per-node transition has been computed into nxt[], one atomic
 *   write applies all of them.  Pairs with snapshot_state above.
 */
static inline void commit_state(EpiState *epi, const SIR *in)
{
    memcpy(epi->state, in, sizeof epi->state);
}

/*
 * compute_next_state — fill nxt[] with this tick's transitions.
 *
 *   Pseudocode:
 *     for each node i:
 *       if state[i] == I_STATE:
 *         tick_one_infected(i, sc, epi, nxt)
 *
 *   Only I nodes drive transitions (recovery + transmission); S and
 *   R nodes are passive (only changed by some I neighbour writing
 *   to their nxt[] slot).  tick_one_infected reads state[] (the
 *   pre-tick snapshot from snapshot_state) and writes nxt[], so
 *   neighbour ordering is irrelevant — this is the property the
 *   snapshot/commit dance buys us.
 */
static void compute_next_state(Scene *sc, SIR *nxt)
{
    for (int i = 0; i < N_NODES; i++) {
        if (sc->epi.state[i] != I_STATE) continue;
        tick_one_infected(i, sc, &sc->epi, nxt);
    }
}

/*
 * record_current_counts — sample (S, I, R) compartment sizes into history.
 *
 *   One pass per compartment via count_state; the three results
 *   plus the all-time peak update are pushed onto the rolling
 *   history buffer by history_record.  Called after commit_state
 *   so the counts reflect THIS tick's transitions, not the
 *   pre-tick snapshot.
 */
static void record_current_counts(Scene *sc)
{
    int s = count_state(&sc->epi, S_STATE);
    int i = count_state(&sc->epi, I_STATE);
    int r = count_state(&sc->epi, R_STATE);
    history_record(&sc->history, s, i, r);
}

/*
 * sir_tick — advance the whole simulation by ONE frame.
 *
 *   Pseudocode (synchronous SIR update — cellular-automata pattern):
 *     if paused: return
 *     decrement_flash_counters       ← age render flash hints
 *     nxt := snapshot_state          ← READ HALF of sync update
 *     compute_next_state(nxt)        ← per-I transitions written to nxt
 *     commit_state(nxt)              ← WRITE HALF (atomic apply)
 *     tick++
 *     record_current_counts          ← push (S, I, R) to chart history
 *
 *   SYNCHRONOUS means every node's transition for this tick is
 *   COMPUTED from the pre-tick state, then APPLIED all at once.
 *   Without the snapshot/commit pair an I that recovered early in
 *   the loop wouldn't transmit to its neighbours, under-counting
 *   infections — a classic order-of-iteration bug in CA simulations.
 */
static void sir_tick(Scene *sc)
{
    if (sc->paused) return;

    decrement_flash_counters(&sc->epi);

    SIR nxt[N_NODES];
    snapshot_state    (&sc->epi, nxt);
    compute_next_state(sc,       nxt);
    commit_state      (&sc->epi, nxt);

    sc->tick++;
    record_current_counts(sc);
}

/* ===================================================================== */
/* §7  draw                                                               */
/* ===================================================================== */

/*
 * draw_line — Bresenham rasteriser with slope-matched ASCII glyph.
 *
 *   At each step the glyph is chosen so the line visibly slopes in
 *   the right direction:  '-' horizontal, '|' vertical, '\' '/' diagonal.
 *
 *   Reference: Bresenham (1965) [10].
 */
static void draw_line(int x0,int y0,int x1,int y1,attr_t attr,int cols,int rows)
{
    int dx=abs(x1-x0),dy=abs(y1-y0),sx=x0<x1?1:-1,sy=y0<y1?1:-1,err=dx-dy;
    for(;;){
        if(x0>=0&&x0<cols&&y0>=0&&y0<rows){
            int e2=2*err; bool bx=(e2>-dy),by=(e2<dx);
            chtype ch=(bx&&by)?(sx==sy?'\\':'/'):(bx?'-':'|');
            attron(attr); mvaddch(y0,x0,ch); attroff(attr);
        }
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>-dy){err-=dy;x0+=sx;}
        if(e2< dx){err+=dx;y0+=sy;}
    }
}

/* ── left panel: network ─────────────────────────────────────────────── */

/*
 * NodeGlyph — the rendered representation of one node's SIR state.
 *
 * Intent
 *   The output of pick_node_glyph — a 3-tuple that
 *   draw_network_nodes passes straight to ncurses.  Naming the
 *   tuple as a struct lets the classifier function return a single
 *   value and decouples "what colour / glyph corresponds to state X"
 *   from the drawing loop.
 *
 *   Same idiom as the `case_glyph[16]` table in marching_squares.c
 *   and the `NodeGlyph` in kd_tree.c — render output as data, not
 *   logic.  Adding a new visual state (e.g. an "exposed" pre-
 *   infectious node) is a one-line case in pick_node_glyph; no
 *   draw-loop changes needed.
 *
 * Why a STRUCT (not three out-params or a packed int)
 *   - Three values, three roles; the names document the call sites.
 *   - Pass-by-value: 8-12 bytes, fits in registers on any 64-bit ABI.
 *   - Adding a fourth field (e.g. background colour for an overlay)
 *     is a one-line struct edit; no signature churn elsewhere.
 *
 * Visual legend (matches pick_node_glyph cases)
 *   S            → grey   '.'        (small dot, unobtrusive)
 *   I, flash > 0 → yellow '*'  bold  (just-infected highlight)
 *   I, flash = 0 → red    '@'  bold  (settled infection)
 *   R            → green  '+'        (recovered + immune)
 *
 *   These four mappings are the LEGEND the HUD's row-1 "key:" line
 *   exposes to the user.
 *
 * Members
 *   cp       ncurses colour-pair index (CP_S, CP_I, CP_I_FLASH,
 *            CP_R) — see §1 enum.
 *   ch       glyph character ('.', '@', '*', '+').
 *   extra    additional ncurses attributes — A_BOLD for I, 0 for S/R.
 *            ORed with COLOR_PAIR(cp) at the attron / attroff call.
 */
typedef struct {
    int    cp;      /* ncurses colour-pair index (see §1 CP_* enum) */
    chtype ch;      /* glyph character drawn at the node's cell     */
    attr_t extra;   /* extra attributes ORed in (A_BOLD or 0)       */
} NodeGlyph;

/*
 * pick_node_glyph — translate (state, flash) to (cp, glyph, attrs).
 *
 *     state == S:                       grey '.'
 *     state == I  AND flash > 0:        bright yellow '*' (just infected)
 *     state == I  AND flash == 0:       red '@'           (settled)
 *     state == R:                       green '+'         (immune)
 *
 *   Extending the alphabet (e.g. an "exposed" pre-infectious state)
 *   only requires a new case here — the draw loop stays unchanged.
 */
static NodeGlyph pick_node_glyph(const EpiState *epi, int i)
{
    switch (epi->state[i]) {
    case S_STATE: return (NodeGlyph){ CP_S, '.', 0 };
    case I_STATE: return epi->flash[i] > 0
                       ? (NodeGlyph){ CP_I_FLASH, '*', A_BOLD }
                       : (NodeGlyph){ CP_I,       '@', A_BOLD };
    case R_STATE: return (NodeGlyph){ CP_R, '+', 0 };
    default:      return (NodeGlyph){ CP_S, '.', 0 };
    }
}

/*
 * pick_edge_attr — colour an edge by infection / shortcut status.
 *
 *     hot AND rewired:       bright yellow + bold  (shortcut active)
 *     hot AND ring:          bright red    + bold  (ring edge active)
 *     not hot:               dim grey              (background lattice)
 *
 *   "hot" means at least one endpoint is in state I.  The shortcut
 *   highlight is THE visualisation of small-world acceleration —
 *   bright yellow paths jumping across the ring tell you the
 *   epidemic is using a long-range link to spread.
 */
static attr_t pick_edge_attr(const Scene *sc, int i, int j)
{
    bool hot = (sc->epi.state[i] == I_STATE || sc->epi.state[j] == I_STATE);
    if (!hot)                          return COLOR_PAIR(CP_EDGE_DIM);
    if (sc->topology.rewired[i][j])    return COLOR_PAIR(CP_EDGE_REWIRE) | A_BOLD;
    return                                    COLOR_PAIR(CP_EDGE_HOT)    | A_BOLD;
}

/*
 * draw_network_edges — every adjacency, drawn first so nodes paint
 * on top.  Pass i < j only to avoid drawing each edge twice.
 */
static void draw_network_edges(const Scene *sc)
{
    int net_w = (int)(sc->cols * NET_FRAC);
    for (int i = 0; i < N_NODES; i++)
        for (int j = i + 1; j < N_NODES; j++) {
            if (!sc->topology.adj[i][j]) continue;
            draw_line(px_col(sc->pos[i].x), px_row(sc->pos[i].y),
                      px_col(sc->pos[j].x), px_row(sc->pos[j].y),
                      pick_edge_attr(sc, i, j),
                      net_w, sc->rows - HUD_BOT_ROWS);
        }
}

/*
 * draw_network_nodes — paint each node's glyph on top of its edges.
 *   Bounds-clipped to the network panel and the HUD-free band.
 */
static void draw_network_nodes(const Scene *sc)
{
    int net_w = (int)(sc->cols * NET_FRAC);
    for (int i = 0; i < N_NODES; i++) {
        int c = px_col(sc->pos[i].x);
        int r = px_row(sc->pos[i].y);
        if (c < 0 || c >= net_w)                              continue;
        if (r < HUD_TOP_ROWS || r >= sc->rows - HUD_BOT_ROWS) continue;
        NodeGlyph g = pick_node_glyph(&sc->epi, i);
        attron(COLOR_PAIR(g.cp) | g.extra);
        mvaddch(r, c, g.ch);
        attroff(COLOR_PAIR(g.cp) | g.extra);
    }
}

/* draw_network — orchestrator: edges then nodes (painter's order). */
static void draw_network(const Scene *sc)
{
    draw_network_edges(sc);
    draw_network_nodes(sc);
}

/* ── right panel: epidemic curve ─────────────────────────────────────── */

/*
 * ChartLayout — derived geometry for ONE frame's chart panel.
 *
 * Intent
 *   compute_chart_layout fills this struct from (rows, cols); the
 *   downstream draw_chart_* helpers consume it instead of recomputing
 *   each dimension at every call.
 *
 *   Same idiom as a "shader uniform block" in 3-D graphics: pre-
 *   compute the per-frame derived constants once, then pass them by
 *   const ptr to every fragment that needs them.  Saves both
 *   arithmetic AND the temptation for the draw_chart_* siblings to
 *   drift apart on geometry assumptions.
 *
 * Why a `valid` flag (not just panic on too-small terminals)
 *   When the user shrinks the terminal below the chart's minimum
 *   workable size, we want the network panel to keep rendering
 *   without an awkward partial chart appearing in slivers.
 *   compute_chart_layout sets valid = false and draw_chart bails;
 *   the rest of the frame still renders.  This pattern composes
 *   better than a global "panic" early-return at the top of
 *   scene_draw.
 *
 * Coordinate naming convention (left → right across the screen)
 *
 *       0 ─── net_w  net_w+1 ─── chart_x ─── data_x ─── cols
 *                  │           │           │           │
 *       network    │  divider  │  Y-axis   │  bar data │
 *       panel      │  (1 col)  │  labels   │  area     │
 *                  │           │  (3 col)  │           │
 *
 *   - net_w     : column index of the divider; equal to floor(cols*NET_FRAC)
 *   - chart_x   : net_w + 1 — first column past the divider
 *   - data_x    : chart_x + 3 — first column past the Y-axis label width
 *
 *   And vertically:
 *
 *       row 0 ─── HUD_TOP_ROWS = chart_top ─── data_top ─── rows-HUD_BOT_ROWS
 *
 *   - chart_top : HUD_TOP_ROWS (right after the top HUD)
 *   - data_top  : chart_top + 1 — leaves one row for the title
 *
 * Members
 *   net_w       column of the vertical divider (also last column of
 *               the network panel).
 *   chart_x     first column AFTER the divider — left edge of chart area.
 *   chart_top   first row of the chart area  (= HUD_TOP_ROWS).
 *   chart_h     height of the chart area
 *               (= rows − chart_top − HUD_BOT_ROWS).
 *   data_x      first column AFTER the Y-axis labels — left edge of
 *               the bar-chart data area.
 *   data_w      width of the data area in columns; the visible bars
 *               occupy the rightmost min(history.n, data_w) columns.
 *   data_top    chart_top + 1 (skips the title row).
 *   data_h      chart_h − 1.
 *   valid       false if any dimension check failed; draw_chart bails.
 *
 * Invariants (when valid == true)
 *   net_w + 1   == chart_x
 *   chart_top   == HUD_TOP_ROWS
 *   chart_h     ≥ 4         (else valid would be false)
 *   data_w      ≥ 4
 *   data_h      ≥ 2
 *   data_x      == chart_x + 3
 *   data_top    == chart_top + 1
 *   data_h      == chart_h − 1
 *
 *   These minima are what the early-return checks in
 *   compute_chart_layout enforce; downstream helpers can trust them.
 */
typedef struct {
    int  net_w;        /* column index of the vertical divider        */
    int  chart_x;      /* leftmost column of chart area (= net_w + 1) */
    int  chart_top;    /* first row of chart area (= HUD_TOP_ROWS)    */
    int  chart_h;      /* total height of chart area                  */
    int  data_x;       /* leftmost column of data area (past Y labels)*/
    int  data_w;       /* width of data area in columns               */
    int  data_top;     /* first row of data area (chart_top + 1)      */
    int  data_h;       /* height of data area (chart_h − 1)           */
    bool valid;        /* false if any dimension check failed         */
} ChartLayout;

/*
 * compute_chart_layout — derive all chart-panel dimensions from
 * the current terminal extent.
 *
 *   Pseudocode:
 *     net_w     := cols × NET_FRAC      ← edge of network panel
 *     chart_x   := net_w + 1            ← past divider column
 *     chart_top := HUD_TOP_ROWS
 *     chart_h   := rows - chart_top - HUD_BOT_ROWS
 *     y_lbl_w   := 3                    ← width reserved for "40 " "20 " " 0 "
 *     data_x    := chart_x + y_lbl_w
 *     data_w    := chart_w - y_lbl_w
 *     data_top  := chart_top + 1        ← one row for the title
 *     data_h    := chart_h - 1
 *     valid     := all dimensions fit
 */
/*
 * split_panels_horizontally — divide the screen between the network
 * panel (left NET_FRAC) and the chart panel (right remainder).
 *
 *   Sets L->net_w (divider column) and L->chart_x (chart's left edge);
 *   returns false if the chart would be narrower than 8 columns
 *   (too small to render any meaningful curve).
 */
static bool split_panels_horizontally(ChartLayout *L, int cols)
{
    L->net_w   = (int)(cols * NET_FRAC);
    L->chart_x = L->net_w + 1;
    int chart_w = cols - L->chart_x;
    return chart_w >= 8;
}

/*
 * set_chart_vertical_extent — clip the chart vertically to the
 * HUD-free band [HUD_TOP_ROWS, rows − HUD_BOT_ROWS).
 *
 *   Sets L->chart_top and L->chart_h.  Returns false if the band is
 *   shorter than 4 rows (no room for title + axis + bars).
 */
static bool set_chart_vertical_extent(ChartLayout *L, int rows)
{
    L->chart_top = HUD_TOP_ROWS;
    L->chart_h   = rows - L->chart_top - HUD_BOT_ROWS;
    return L->chart_h >= 4;
}

/*
 * reserve_y_axis_columns — set aside Y_LBL_W=3 columns at the left of
 * the chart for "40 ", "20 ", " 0" axis labels.
 *
 *   Sets L->data_x (left edge of the bar data area) and L->data_w
 *   (width of the bar data area).  Returns false if data_w < 4
 *   (no room for any visible bars).
 */
static bool reserve_y_axis_columns(ChartLayout *L, int cols)
{
    const int y_lbl_w = 3;          /* "40 ", "20 ", " 0" */
    L->data_x = L->chart_x + y_lbl_w;
    L->data_w = (cols - L->chart_x) - y_lbl_w;
    return L->data_w >= 4;
}

/*
 * reserve_title_row — set aside ONE row at the top of the chart for
 * the " EPIDEMIC CURVE" title; everything below is the bar data area.
 *
 *   Sets L->data_top and L->data_h.  Returns false if the data area
 *   would be shorter than 2 rows (no usable bar height).
 */
static bool reserve_title_row(ChartLayout *L)
{
    L->data_top = L->chart_top + 1;
    L->data_h   = L->chart_h - 1;
    return L->data_h >= 2;
}

/*
 * compute_chart_layout — orchestrate the four layout steps.
 *
 *   Pseudocode:
 *     split_panels_horizontally    ← divide screen N|C
 *     set_chart_vertical_extent    ← clip into HUD-free band
 *     reserve_y_axis_columns       ← left edge: axis labels
 *     reserve_title_row            ← top edge: chart title
 *     mark valid; return
 *
 *   Any step that returns false sets L.valid = false implicitly
 *   (it was initialised that way) and short-circuits the orchestrator.
 *   draw_chart's early-return on !L.valid then skips the whole panel.
 */
static ChartLayout compute_chart_layout(int rows, int cols)
{
    ChartLayout L = { .valid = false };
    if (!split_panels_horizontally(&L, cols))   return L;
    if (!set_chart_vertical_extent(&L, rows))   return L;
    if (!reserve_y_axis_columns   (&L, cols))   return L;
    if (!reserve_title_row        (&L))         return L;
    L.valid = true;
    return L;
}

/* draw_chart_divider — vertical bar separating network from chart. */
static void draw_chart_divider(const Scene *sc, const ChartLayout *L)
{
    attron(COLOR_PAIR(CP_DIVIDER));
    for (int r = L->chart_top; r < sc->rows - HUD_BOT_ROWS; r++)
        mvaddch(r, L->net_w, ACS_VLINE);
    attroff(COLOR_PAIR(CP_DIVIDER));
}

/* draw_chart_title — single yellow heading line above the data area. */
static void draw_chart_title(const ChartLayout *L)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(L->chart_top, L->chart_x, " EPIDEMIC CURVE");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* draw_chart_axis_labels — 0, N/4, N/2, 3N/4, N along the Y axis. */
static void draw_chart_axis_labels(const ChartLayout *L)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(L->data_top,                  L->chart_x, "%2d", N_NODES);
    mvprintw(L->data_top + L->data_h/4,    L->chart_x, "%2d", N_NODES*3/4);
    mvprintw(L->data_top + L->data_h/2,    L->chart_x, "%2d", N_NODES/2);
    mvprintw(L->data_top + L->data_h*3/4,  L->chart_x, "%2d", N_NODES/4);
    mvprintw(L->data_top + L->data_h - 1,  L->chart_x, " 0");
    attroff(COLOR_PAIR(CP_HUD));
}

/*
 * draw_chart_stacked_bars — one column per visible history sample,
 * stacked R (bottom green) → I (middle red) → S (top grey).
 *
 *   Pseudocode:
 *     n_show := min(history.n, data_w)              ← columns to draw
 *     for cx in 0..n_show-1:
 *       bi := circular-buffer index of sample (oldest at left)
 *       rh, ih, sh := scaled heights for R, I, S
 *       for each row from bottom up:
 *         paint R / I / S glyph depending on which segment owns this row
 */
static void draw_chart_stacked_bars(const Scene *sc, const ChartLayout *L)
{
    int n_show = sc->history.n < L->data_w ? sc->history.n : L->data_w;
    if (n_show == 0) return;

    for (int cx = 0; cx < n_show; cx++) {
        int bi = (sc->history.head - n_show + cx + HIST_LEN) % HIST_LEN;
        int hs = sc->history.s[bi];
        int hi = sc->history.i[bi];
        int hr = sc->history.r[bi];

        int scol = L->data_x + (L->data_w - n_show) + cx;
        if (scol < L->data_x || scol >= sc->cols) continue;

        int rh = hr * L->data_h / N_NODES;
        int ih = hi * L->data_h / N_NODES;
        int sh = hs * L->data_h / N_NODES;

        for (int rb = 0; rb < L->data_h; rb++) {
            int srow = L->data_top + (L->data_h - 1 - rb);
            if (srow < 0 || srow >= sc->rows) continue;

            int    cp;
            chtype ch;
            attr_t ex = 0;
            if      (rb < rh)             { cp=CP_BAR_R; ch='-';            }
            else if (rb < rh + ih)        { cp=CP_BAR_I; ch='#'; ex=A_BOLD; }
            else if (rb < rh + ih + sh)   { cp=CP_BAR_S; ch='=';            }
            else                          { continue;                       }

            attron(COLOR_PAIR(cp) | ex);
            mvaddch(srow, scol, ch);
            attroff(COLOR_PAIR(cp) | ex);
        }
    }
}

/*
 * draw_chart_peak_marker — "pk" label at the all-time outbreak peak.
 *
 *   peak_i is monotonic over the run (see history_record), so this
 *   label only moves UP (or stays put).  Drawn in dim red so it
 *   reads as an annotation, not part of the bars.
 */
static void draw_chart_peak_marker(const Scene *sc, const ChartLayout *L)
{
    if (sc->history.peak_i <= 0) return;
    int peak_row = L->data_top + L->data_h - 1
                 - sc->history.peak_i * L->data_h / N_NODES;
    if (peak_row < L->data_top || peak_row >= L->data_top + L->data_h) return;
    attron(COLOR_PAIR(CP_I) | A_DIM);
    mvprintw(peak_row, L->chart_x, "pk");
    attroff(COLOR_PAIR(CP_I) | A_DIM);
}

/*
 * draw_chart — orchestrator.
 *
 *   Pseudocode:
 *     L := compute_chart_layout(rows, cols)
 *     if !L.valid: return                      ← too small to render
 *     draw_chart_divider      (sc, &L)
 *     draw_chart_title        (&L)
 *     draw_chart_axis_labels  (&L)
 *     draw_chart_stacked_bars (sc, &L)
 *     draw_chart_peak_marker  (sc, &L)
 */
static void draw_chart(const Scene *sc)
{
    ChartLayout L = compute_chart_layout(sc->rows, sc->cols);
    if (!L.valid) return;
    draw_chart_divider     (sc, &L);
    draw_chart_title       (&L);
    draw_chart_axis_labels (&L);
    draw_chart_stacked_bars(sc, &L);
    draw_chart_peak_marker (sc, &L);
}

/* ── HUD (top rows: data, bottom row: actions) ───────────────────────── */

/*
 * detect_epidemic_phase — classify the current frame into one of
 * five readable labels: READY, SEEDED, GROWING, WANING, PLATEAU,
 * EXTINCT.
 *
 *   Pseudocode:
 *     if I == 0 AND R == 0:  READY  ← never seeded
 *     if I == 0:             EXTINCT (was infected; now all R)
 *     if no prior tick:      SEEDED
 *     compare current I to history_prev_infected:
 *       larger  →  GROWING
 *       smaller →  WANING
 *       equal   →  PLATEAU
 */
static const char *detect_epidemic_phase(const Scene *sc, int ni, int nr)
{
    if (ni == 0 && nr == 0) return "READY  ";
    if (ni == 0)            return "EXTINCT";
    if (sc->history.n < 2)  return "SEEDED ";
    int prev_i = history_prev_infected(&sc->history);
    if (ni > prev_i) return "GROWING";
    if (ni < prev_i) return "WANING ";
    return                  "PLATEAU";
}

/*
 * draw_hud_rates — column 0 of row 0: "β=0.040  γ=0.025".
 *
 *   The two probabilistic rates the user adjusts with the arrow keys.
 *   These are the INPUTS to the SIR dynamics; everything else on row
 *   0 (R0, <k>, S/I/R, phase) is derived.
 */
static void draw_hud_rates(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, " β=%.3f  γ=%.3f ", sc->beta, sc->gamma);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * draw_hud_r0 — column 18 of row 0: "R0=6.40" with threshold colouring.
 *
 *   R0 = β·⟨k⟩/γ — the BASIC REPRODUCTION NUMBER.  Threshold is 1:
 *     R0 > 1   →  epidemic spreads      (rendered red)
 *     R0 ≤ 1   →  epidemic dies         (rendered green)
 *
 *   The colour swap at R0 = 1 IS the phase-transition signal — the
 *   user can drag β/γ and watch R0 cross the threshold and the
 *   colour flip mid-frame.
 *
 *   References [4], [5] §2 for the derivation.
 */
static void draw_hud_r0(float r0)
{
    attr_t a = (r0 > 1.f) ? (COLOR_PAIR(CP_I) | A_BOLD)
                          : (COLOR_PAIR(CP_R) | A_BOLD);
    attron(a);
    mvprintw(0, 18, "R0=%-4.2f", r0);
    attroff(a);
}

/*
 * draw_hud_summary — column 27 of row 0: derived state at a glance.
 *
 *   <k>=mean_degree  tick=N  S=n I=n R=n  PHASE  [PAUSED]
 *
 *   Everything here is DERIVED from sc (counts, topology_mean_degree)
 *   or from sc plus history (phase) — no new state.
 */
static void draw_hud_summary(const Scene *sc, float mk, int ns, int ni, int nr,
                             const char *phase)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 27, " <k>=%.1f  tick=%-4d  S=%-3d I=%-3d R=%-3d  %s%s",
             mk, sc->tick, ns, ni, nr,
             phase,
             sc->paused ? "  [PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * draw_bar_segment — paint ONE coloured segment of the stacked bar.
 *
 *   Pseudocode:
 *     fill := count × bar_w / total       ← proportional length
 *     paint `glyph` in colour `cp` from column bx for `fill` cells
 *     return bx + fill                    ← new write head for caller
 *
 *   Returning the advanced cursor position lets the caller chain
 *   segments without tracking the offset itself — the classic
 *   "fold" idiom (a → b → c → … applied left-to-right).
 */
static int draw_bar_segment(int row, int bx, int count, int total, int bar_w,
                            int max_col, int cp, attr_t extra, chtype glyph)
{
    int fill = count * bar_w / total;
    attron(COLOR_PAIR(cp) | extra);
    for (int i = 0; i < fill && bx + i < max_col; i++)
        mvaddch(row, bx + i, glyph);
    attroff(COLOR_PAIR(cp) | extra);
    return bx + fill;
}

/*
 * draw_segment_legend — three coloured "S I R" letters keyed to the
 * bar segment colours.  Tells the reader what the bar's colours mean.
 */
static void draw_segment_legend(int row, int bx)
{
    attron(COLOR_PAIR(CP_BAR_S));        mvprintw(row, bx+2, "S"); attroff(COLOR_PAIR(CP_BAR_S));
    attron(COLOR_PAIR(CP_BAR_I)|A_BOLD); mvprintw(row, bx+4, "I"); attroff(COLOR_PAIR(CP_BAR_I)|A_BOLD);
    attron(COLOR_PAIR(CP_BAR_R));        mvprintw(row, bx+6, "R"); attroff(COLOR_PAIR(CP_BAR_R));
}

/*
 * draw_glyph_key — the " key: .=S @=I +=R" hint that maps the
 * NETWORK-PANEL glyphs to their meaning.  Distinct from the segment
 * legend (which keys the BAR colours).
 */
static void draw_glyph_key(int row, int bx)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(row, bx+9, " key: ");
    attroff(COLOR_PAIR(CP_HUD));
    attron(COLOR_PAIR(CP_S));        mvprintw(row, bx+15, ".=S "); attroff(COLOR_PAIR(CP_S));
    attron(COLOR_PAIR(CP_I)|A_BOLD); mvprintw(row, bx+19, "@=I "); attroff(COLOR_PAIR(CP_I)|A_BOLD);
    attron(COLOR_PAIR(CP_R));        mvprintw(row, bx+23, "+=R");  attroff(COLOR_PAIR(CP_R));
}

/*
 * draw_hud_proportion_bar — row 1: stacked-fill bar showing the
 * current S / I / R proportions of the population.
 *
 *   Pseudocode:
 *     bar_w := cols - 22                ← width budget
 *     bx    := 1                        ← writer head, after lead pad
 *     bx    := draw_bar_segment(..., S, '=')
 *     bx    := draw_bar_segment(..., I, '#')
 *     bx    := draw_bar_segment(..., R, '-')
 *     draw_segment_legend(bx)           ← " S I R " key for the bar
 *     draw_glyph_key(bx)                ← " key: .=S @=I +=R " for the network
 */
static void draw_hud_proportion_bar(const Scene *sc, int ns, int ni, int nr)
{
    int bar_w = sc->cols - 22;
    if (bar_w < 6) bar_w = 6;

    mvprintw(1, 0, " ");
    int bx = 1;
    bx = draw_bar_segment(1, bx, ns, N_NODES, bar_w, sc->cols, CP_BAR_S, 0,      '=');
    bx = draw_bar_segment(1, bx, ni, N_NODES, bar_w, sc->cols, CP_BAR_I, A_BOLD, '#');
    bx = draw_bar_segment(1, bx, nr, N_NODES, bar_w, sc->cols, CP_BAR_R, 0,      '-');

    draw_segment_legend(1, bx);
    draw_glyph_key     (1, bx);
}

/*
 * draw_hud_top — top HUD_TOP_ROWS rows: live DATA readout.
 *
 *   Pseudocode:
 *     compute ns, ni, nr, mk, r0, phase   ← derived quantities
 *     draw_hud_rates                       ← row 0 cols 0-17
 *     draw_hud_r0                          ← row 0 col 18, threshold-coloured
 *     draw_hud_summary                     ← row 0 col 27 onwards
 *     draw_hud_proportion_bar              ← row 1: stacked S/I/R bar + keys
 *
 *   Top bar follows the project HUD Standard: bright yellow + A_BOLD
 *   (CP_HUD) so the bar stays legible against any animation that
 *   intrudes from the network panel below.  R0 uses CP_I/CP_R to
 *   colour-code the epidemic-threshold crossing (see draw_hud_r0).
 */
static void draw_hud_top(const Scene *sc)
{
    int ns = count_state(&sc->epi, S_STATE);
    int ni = count_state(&sc->epi, I_STATE);
    int nr = count_state(&sc->epi, R_STATE);
    float mk = topology_mean_degree(&sc->topology);
    float r0 = (sc->gamma > 0.f) ? sc->beta * mk / sc->gamma : 0.f;
    const char *phase = detect_epidemic_phase(sc, ni, nr);

    draw_hud_rates          (sc);
    draw_hud_r0             (r0);
    draw_hud_summary        (sc, mk, ns, ni, nr, phase);
    draw_hud_proportion_bar (sc, ns, ni, nr);
}

/*
 * draw_hud_bottom — last row: ACTION key hints.
 *
 *   Bright cyan + A_BOLD per the project HUD Standard.  Lists every
 *   interactive key bound in handle_input; each key here changes a
 *   value visible in the top data bar.
 */
static void draw_hud_bottom(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HINT)|A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  i:inject  up/dn:β  lt/rt:γ ");
    attroff(COLOR_PAIR(CP_HINT)|A_BOLD);
}

/*
 * scene_draw — render one frame.
 *
 *   Pseudocode (the entire frame pipeline):
 *     1. draw_hud_top      ─ row 0+1: live data (yellow)
 *     2. draw_network      ─ left panel: ring + edges + nodes
 *     3. draw_chart        ─ right panel: scrolling epidemic curve
 *     4. draw_hud_bottom   ─ last row: key hints (cyan)
 *
 *   Step 1 is drawn first so steps 2-3 can overlap it visually only
 *   if positions are mis-clipped (defensive; should never happen with
 *   correct HUD_*_ROWS reservations).
 */
static void scene_draw(const Scene *sc)
{
    draw_hud_top   (sc);
    draw_network   (sc);
    draw_chart     (sc);
    draw_hud_bottom(sc);
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s==SIGINT||s==SIGTERM) g_quit=1;
    if (s==SIGWINCH)           g_resize=1;
}
static void cleanup(void) { endwin(); }

/*
 * clamp01 — pin a value into [0, 1].  Used to bound β and γ when the
 * user adjusts them via arrow keys.
 */
static inline float clamp01(float v)
{
    if (v < 0.f) return 0.f;
    if (v > 1.f) return 1.f;
    return v;
}

/*
 * handle_input — dispatch one keypress against the scene.
 *
 *   Pure delegation: every key maps to a small mutation of sc.
 *   Quit signals go through g_quit so SIGINT/SIGTERM take the same
 *   exit path as the 'q' key.  β and γ are clamped to [0, 1] so
 *   the probability roll in prob_roll always sees a valid value.
 */
static void handle_input(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit       = 1;             break;
    case ' ':                    sc->paused   = !sc->paused;   break;
    case 'r': case 'R':          epi_reset(sc);                break;
    case 'i': case 'I':          epi_inject(&sc->epi);         break;
    case KEY_UP:    sc->beta  = clamp01(sc->beta  + BETA_STEP);  break;
    case KEY_DOWN:  sc->beta  = clamp01(sc->beta  - BETA_STEP);  break;
    case KEY_RIGHT: sc->gamma = clamp01(sc->gamma + GAMMA_STEP); break;
    case KEY_LEFT:  sc->gamma = clamp01(sc->gamma - GAMMA_STEP); break;
    default: break;
    }
}

/* ── main loop helpers ───────────────────────────────────────────────── */

/* init_random_seed — seed the libc RNG from the monotonic clock. */
static void init_random_seed(void)
{
    srand((unsigned)(clock_ns() & 0xFFFFFFFF));
}

/*
 * register_signal_handlers — wire SIGINT/SIGTERM to g_quit and
 * SIGWINCH to g_resize, plus atexit(cleanup) so endwin always runs.
 */
static void register_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);
}

/*
 * init_ncurses_session — bring up ncurses in the mode this demo
 * needs: cbreak (raw keys), noecho, keypad (for arrow keys),
 * nodelay (non-blocking getch), hidden cursor, no typeahead,
 * colours initialised.
 */
static void init_ncurses_session(void)
{
    initscr();
    cbreak(); noecho(); keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE); curs_set(0); typeahead(-1);
    color_init();
}

/*
 * init_scene — set default rates and build the one-shot graph + layout.
 *
 *   Pseudocode:
 *     sc.beta, sc.gamma := INIT defaults    (clamped to [0, 1])
 *     sc.paused := false;  sc.tick := 0
 *     read terminal extent into sc.rows, sc.cols
 *     topology_build_ws(&sc.topology)       ← built ONCE per run
 *     layout_ring(sc)                       ← positions for current size
 *     epi_reset(sc)                         ← seed first infection
 *
 *   The TOPOLOGY survives the entire program lifetime; only the
 *   layout is rebuilt on resize and only the epi state is wiped on 'r'.
 */
static void init_scene(Scene *sc)
{
    sc->beta   = BETA_INIT;
    sc->gamma  = GAMMA_INIT;
    sc->paused = false;
    sc->tick   = 0;
    getmaxyx(stdscr, sc->rows, sc->cols);
    topology_build_ws(&sc->topology);
    layout_ring(sc);
    epi_reset(sc);
}

/*
 * handle_resize — refresh ncurses' terminal size cache and rebuild
 * the ring layout so node positions track the new extent.
 *
 *   The TOPOLOGY (edges) is unchanged on resize — only pixel
 *   positions; this keeps the simulation continuous across resizes.
 */
static void handle_resize(Scene *sc)
{
    g_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
    layout_ring(sc);
}

/*
 * render_one_frame — paint + flush one ncurses frame.
 *
 *   Pseudocode:
 *     erase                ← clear stdscr's virtual buffer
 *     scene_draw(sc)       ← paint everything into stdscr
 *     wnoutrefresh         ← stage the diff (no terminal I/O yet)
 *     doupdate             ← flush ONE diff to the terminal
 *
 *   The wnoutrefresh + doupdate split is the standard ncurses idiom
 *   for "only emit one diff per frame" — see Pradeep Padala's ncurses
 *   HOWTO, "Update vs. wnoutrefresh + doupdate".
 */
static void render_one_frame(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * sleep_to_frame_deadline — burn the remainder of the frame budget.
 *
 *   Holds the loop to FPS by sleeping (frame_ns − elapsed) since t0.
 *   If we overran the budget (negative delta) clock_sleep_ns no-ops,
 *   so a slow frame just doesn't get any sleep — no spiral.
 */
static void sleep_to_frame_deadline(long long t0, long long frame_ns)
{
    clock_sleep_ns(frame_ns - (clock_ns() - t0));
}

/*
 * main — own the Scene, build the graph once, drive the fixed-rate loop.
 *
 *   Pseudocode:
 *     init_random_seed
 *     register_signal_handlers
 *     init_ncurses_session
 *     init_scene(&scene)           ← topology + layout + first seed
 *
 *     while !g_quit:
 *       if g_resize: handle_resize
 *       handle_input(scene, getch)
 *       t0 := now
 *       sir_tick(scene)            ← advance one tick (or no-op if paused)
 *       render_one_frame(scene)
 *       sleep_to_frame_deadline(t0)
 */
int main(void)
{
    init_random_seed();
    register_signal_handlers();
    init_ncurses_session();

    Scene scene = { 0 };
    init_scene(&scene);

    long long frame_ns = 1000000000LL / FPS;

    while (!g_quit) {
        if (g_resize) handle_resize(&scene);
        handle_input(&scene, getch());

        long long t0 = clock_ns();
        sir_tick(&scene);
        render_one_frame(&scene);
        sleep_to_frame_deadline(t0, frame_ns);
    }
    return 0;
}
