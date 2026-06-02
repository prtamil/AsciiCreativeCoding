/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wfc_showcase.c — Wave Function Collapse, scaled up for spectacle.
 *
 * DEMO: Full-terminal grid (~200×60 cells), 34-tile alphabet of light /
 *       heavy / double ASCII pipes that only connect to their own
 *       weight class. Five random seed collapses at startup ripple
 *       outward, meeting in the middle with visible interference; the
 *       three weight classes form coloured "domains" (cyan-light /
 *       pink-heavy / gold-double) separated by blank gaps. After full
 *       collapse the grid holds briefly, then re-seeds and the whole
 *       thing loops forever.
 *
 * Study alongside: wfc_learn.c — same algorithm, slowed down with an
 *       entropy heatmap and a propagation queue you can single-step
 *       through. Read learn first; this file extends it for visual punch.
 *
 * Section map (layers — see ARCHITECTURE block below):
 *   §1 config       — grid/glow/timing constants, alphabet count, colour pairs
 *   §2 performance  — monotonic timer + sleep (frame-cap helpers)
 *   §3 simulation   — WFC core: 34-tile alphabet, 4-valued edges, compat, Grid
 *   §4 simulation   — scene & the ONE per-tick combine (glow decay + ops)
 *   §5 render       — 10 weight-colour themes, per-weight routing, grid, HUD
 *   §6 app          — signals, resize, key events, fixed-timestep main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (immediate re-seed, no flash)
 *   t / T      next / previous colour theme
 *   +/-        more / fewer ops per tick
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra wfc_showcase.c -o wfc_showcase \
 *       -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Tile-based Wave Function Collapse — same algorithm as
 *                  wfc_learn.c but with a 34-tile alphabet. The edge
 *                  state per side is no longer binary {connect, no} but
 *                  4-valued {NONE, LIGHT, HEAVY, DOUBLE}, encoding which
 *                  weight class connects there. Two tiles match across an
 *                  edge iff their facing edges have the SAME state — so
 *                  different weights meet only at NONE-NONE seams,
 *                  producing visible coloured domains.
 *
 * Data-structure : uint64_t bitmask per cell (34 bits used, 30 spare).
 *                  Compat table compat[34][4] of uint64_t. Flat 1-D grid;
 *                  propagation queue with per-cell in_queue[] flag (same
 *                  fix as learn — see learn.c for the bug story).
 *
 * Rendering      : ASCII glyphs only ({' ', '-', '|', '+'}) for terminal
 *                  portability. Each weight class draws in its own colour:
 *                  light=cyan, heavy=pink, double=gold — colour IS the
 *                  weight differentiator since all junctions render as '+'.
 *                  A per-cell prop_glow buffer decays slowly (rate 1.5/s) so
 *                  the propagation wave stays visible as a yellow ribbon on the
 *                  still-undecided frontier; decided cells immediately show
 *                  their weight (theme) colour, so the scene fills with theme
 *                  colour as it grows. After DONE: hold, then re-seed straight
 *                  into a fresh pattern (no full-screen wash).
 *
 * Performance    : O(N · K) per full grid collapse, where N is cells and
 *                  K is alphabet size (34). Worst case ≈ 300 M ops for a
 *                  200×60 grid; we throttle to ops_per_tick (default 32)
 *                  so the spectacle unfolds over ~5–10 s. No allocation
 *                  after init.
 *
 * References     : Concept — (see wfc_learn.c for the step-by-step walkthrough)
 *                  • Gumin (2016) — "WaveFunctionCollapse" repo + README, the
 *                    canonical reference: https://github.com/mxgmn/WaveFunctionCollapse
 *                  • Merrell (2007) — "Example-Based Model Synthesis", I3D 2007:
 *                    the tile-adjacency constraint method; the 4-valued edge
 *                    state used here is exactly its multi-label adjacency.
 *                  • Karth & Smith (2017) — "WaveFunctionCollapse is Constraint
 *                    Solving in the Wild", FDG 2017: WFC framed as a CSP.
 *                  • Mackworth (1977) — "Consistency in Networks of Relations":
 *                    AC-3 arc consistency, what the propagate step performs.
 *                  • Boris the Brave (2020) — "WFC tile-based, explained": the
 *                    adjacency-tile formulation this file scales up.
 *                  Rendering —
 *                  • Brewer — ColorBrewer (colorbrewer2.org): qualitative
 *                    palettes — choosing mutually-distinct colours for the three
 *                    weight-class domains (and the 10 themes) so they stay
 *                    readable side by side.
 *                  • Padala — "NCURSES Programming HOWTO", TLDP: colour pairs,
 *                    glyph output, non-blocking input, resize handling.
 *                  • xterm 256-colour palette — the theme + glow colour indices
 *                    the renderer draws from: https://jonasjacek.github.io/colors/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell holds a 64-bit "could be" set over 34 pipe tiles. The
 * showcase differs from learn only in alphabet size and pacing —
 * everything in this block is a delta against learn's mental model.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * A pond with five stones dropped at once. Each stone is a forced
 * collapse; each ripple is constraint propagation. Where ripples
 * meet, they don't cancel — they MERGE, because constraints are
 * monotone (once a tile is removed it never comes back). So the
 * pattern always converges, but the boundary geometry depends on
 * which ripple arrived first.
 *
 * Three ripple "colours" exist (light / heavy / double weight). At
 * a domain boundary, both sides agree on NONE — so domains are
 * separated by a thin moat of blank cells where the pipe network
 * simply stops. That blank seam is the visible signature of the
 * 4-valued edge state.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Seed: pick N_INITIAL_SEEDS random cells; collapse each to a
 *     uniformly-chosen tile from its full superposition.
 *  2. Drain the propagation queue (same as learn step 3–4).
 *  3. Pick lowest-entropy uncollapsed cell, collapse, drain.
 *  4. When every cell has entropy 1 → state = HOLD.
 *  5. After HOLD_SECONDS → reset every cell to full superposition and
 *     goto 1 (no full-screen flash — just the live propagation wave).
 *
 * KEY FORMULAS
 * ────────────
 *  Edge match (4-valued)         : a.edge[d] == b.edge[opposite(d)]
 *  Compat (precomputed)          : compat[t][d] = ⋃ {1<<t' : edges match}
 *  Mask type                     : uint64_t   (34 bits used)
 *  Popcount                      : __builtin_popcountll
 *  Lowest-bit index              : __builtin_ctzll
 *  Glow decay (per frame)        : glow *= exp(-rate * dt)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CONTRADICTIONS are MORE common with multiple seeds: two ripples
 *    of incompatible weight colliding in a tight space can leave a
 *    cell with no legal tile. Strategy here is "tolerate" — flag bad
 *    and trigger an early reset, rather than backtrack. The visual
 *    cost is one early restart every ~5 runs; backtracking would
 *    cost much more code and the showcase can afford the occasional
 *    glitch.
 *
 *  • UINT64_T MASKS need __builtin_popcountll and __builtin_ctzll
 *    (NOT the 32-bit variants). On x86-64 these are single POPCNT/
 *    TZCNT instructions; performance is fine.
 *
 *  • COLOR PAIRS: PAIR_HUD (226 yellow) and PAIR_HINT (51 cyan) are
 *    RESERVED across the whole project (see CLAUDE.md HUD Standard).
 *    Tile-light deliberately uses 117 (a bluer cyan than 51) so it
 *    is distinguishable from the hint strip.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Distinct domains visible: any rendered frame should show clear
 *    theme-coloured "regions" (three weight classes) separated by blanks,
 *    even while growing. If everything is one colour, weight isolation is broken.
 *  • Theme colour from the start: collapsed cells show their weight colour
 *    immediately (not red/yellow until completion); a thin yellow wave rides
 *    the still-undecided frontier.
 *  • Loop completes: leave the demo running 60 s. You should see at
 *    least 3 full collapse-and-reset cycles. If it freezes mid-cycle,
 *    a contradiction wasn't caught and the state machine stalled.
 *  • HUD tile counts: collapsed_count should monotonically rise to
 *    total_cells, then snap back to 0 on reset. If it
 *    overshoots total_cells, double-counting is back.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * State is module-global (typed structs arrive in step 5).  Each layer owns
 * one concern; this table says what each mutates:
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the clock / sleeps
 *   SIMULATION   §3,§4    the Grid (per-cell mask[], the propagation queue +
 *                         in_queue[], qhead/qtail, collapsed_count, done/bad)
 *                         and Scene's run state; compat[] is filled once at init.
 *   RENDER       §5       the terminal only — reads Grid + glow, never writes
 *                         them; screen_resize re-fits Screen dims (USER EVENT).
 *   APP          §6       run knobs (paused/ops_per_tick/theme, sim_fps), grid
 *                         size, signal flags.
 *
 *   LOGIC is real but NOT a separate section — the pure decisions are small
 *     helpers kept beside the data they serve: dir_dx/dir_dy/opposite,
 *     grid_idx, grid_in_bounds (no mutation, no I/O), plus the render-side pure
 *     mapping weight_pair.  Hoisting these one-liners out would only hurt
 *     locality.  (grid_pick_min_entropy / grid_seed_random are read-only scans
 *     but draw rand() for tie-breaks, so they sit in §3.)
 *   EFFECTS is real STATE, not functions: Grid.prop_glow[] (the yellow
 *     propagation wave).  grid_propagate_one WRITES it; scene_tick DECAYS it;
 *     render READS it.  It never feeds back into the algorithm, so deleting all
 *     glow code would not change the WFC result.  (The collapse red-flash and
 *     the full-grid "supernova" wash were both removed earlier.)
 *   No DELAYS layer — 'space' (paused) freezes scene_tick; the HOLD state
 *     (hold_timer) pauses on a finished pattern before re-seeding; the only
 *     real wait is the §2 frame cap.
 *
 * PER-TICK COMBINE — scene_tick (§4) is the ONE place sim state advances,
 * in fixed order:
 *     1. EFFECTS decay  — multiply prop_glow by exp(-rate·dt).
 *     2. SIMULATION     — GROWING: up to ops_per_tick grid_step calls (each a
 *                         propagation pop, else a collapse); on contradiction
 *                         re-seed, on done → HOLD.  HOLD: count down, then re-seed.
 *   RENDER (scene_draw + HUD) then PERFORMANCE (frame cap) run once per frame in
 *   main, OUTSIDE the tick.
 *
 * USER EVENTS are NOT the tick: keys (pause/reset/theme/speed/Hz) and resize
 * mutate state directly in §6 — reset / theme-change / resize call scene_reset;
 * resize re-fits the grid — but only scene_tick advances the simulation.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <locale.h>
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
    GRID_W_MAX        = 240,
    GRID_H_MAX        =  80,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =  64,        /* spectacle pace — full grid in ≈8 s */
    OPS_PER_TICK_MAX  = 512,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    N_TILES           =  34,        /* 1 blank + 11 light + 11 heavy + 11 double */
    N_INITIAL_SEEDS   =   5,        /* multi-ripple startup */

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_LIGHT        =   3,        /* tile-weight: light pipes  (themed) */
    PAIR_HEAVY        =   4,        /* tile-weight: heavy pipes  (themed) */
    PAIR_DOUBLE       =   5,        /* tile-weight: double pipes (themed) */
    PAIR_WAVE         =   6,        /* propagation wave yellow            */
};

/*
 * Glow decay rates. Slower than learn.c (4.0) because the showcase
 * wants the trail to LINGER — the goal here is spectacle, not
 * pedagogy. exp(-1.5 * 1.0) ≈ 0.22 → glow falls to ~22% in 1 second,
 * ~5% in 2 seconds. Long enough for the eye to follow the wavefront
 * across a wide grid.
 */
#define GLOW_DECAY_RATE      1.5f
#define GLOW_FLASH_THRESHOLD 0.05f

/* Hold timing — state-machine in §4 scene. */
#define HOLD_SECONDS         1.6f   /* pause on a finished pattern */

/* All bits set for our 34-tile alphabet. */
#define ALL_TILES_MASK ((N_TILES >= 64) ? (~(uint64_t)0) \
                                        : (((uint64_t)1 << N_TILES) - 1u))

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Render frame cap: the wall-clock loop never spins faster than this,
 * independent of the (possibly higher) simulation tick rate. */
#define RENDER_CAP_FPS  60
/* Spiral-of-death guard: clamp a stalled frame's elapsed time so the
 * fixed-timestep accumulator never tries to catch up forever. */
#define MAX_FRAME_NS  (100 * NS_PER_MS)

/* ===================================================================== */
/* §2  performance — monotonic clock + sleep (the frame-cap helpers)      */
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
/* §3  simulation — WFC core: 34-tile alphabet, 4-valued edges, compat, Grid */
/* ===================================================================== */

/*
 * Direction encoding: same as wfc_learn.c (N=0, E=1, S=2, W=3).
 * Read learn.c for the why.
 */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };

static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

/*
 * EdgeState — 4-valued edge label, the showcase's extension over learn.c's
 * binary {connect / no}.  This is multi-label tile adjacency (Merrell 2007):
 * the label records not just WHETHER a side connects but to WHICH weight class.
 *
 * NONE      = no connection on this side (blank, or stub-end).
 * LIGHT     = connects to a light-weight (cyan) tile on the matching side.
 * HEAVY     = connects to a heavy-weight (pink) tile.
 * DOUBLE    = connects to a double-weight (gold) tile.
 * (All three weight classes render with the same ASCII glyph set;
 * the visible difference is COLOUR.)
 *
 * Two tiles match across direction d iff
 *   tiles[a].edge[d] == tiles[b].edge[opposite(d)].
 * EQUALITY of EdgeState — not just non-zero — is the whole trick: light/heavy/
 * double never connect across each other, so they can only meet at NONE seams,
 * which is what produces the blank moats between coloured domains.  This
 * equality test is what wfc_build_compat bakes into the compat[][] table.
 */
typedef enum {
    EDGE_NONE   = 0,
    EDGE_LIGHT  = 1,
    EDGE_HEAVY  = 2,
    EDGE_DOUBLE = 3,
} EdgeState;

/*
 * Weight — a tile's visual class, deliberately separate from its edge labels.
 * EdgeState answers "what may connect on this side?"; Weight answers "what
 * colour do I draw?".  They're decoupled because all three classes share one
 * ASCII glyph set ('+' '-' '|'), so Weight (colour) is the ONLY on-screen
 * signal of which domain a pipe belongs to.  BLANK is a colour class with no
 * edges to anything (the moat tile).
 */
typedef enum {
    WEIGHT_LIGHT  = 0,
    WEIGHT_HEAVY  = 1,
    WEIGHT_DOUBLE = 2,
    WEIGHT_BLANK  = 3,   /* visual class only — blank has no edges to any */
} Weight;

/*
 * Tile — one entry in the 34-tile alphabet (Merrell-style adjacency tiles).
 * A tile is fully described by what it looks like and where it connects:
 *   glyph  : the 1-char ASCII string drawn for it (' ' '-' '|' '+').
 *   edge[d]: the 4-valued connection label on side d, indexed [N,E,S,W] —
 *            the entire adjacency rule (see EdgeState).
 *   weight : its colour / domain class (see Weight), used only by the renderer.
 * The alphabet is 1 blank + the full 11-tile junction set per weight class (×3
 * = 34), so every connectivity option exists within a class and there are no
 * dead-end stubs that would force avoidable contradictions.
 */
typedef struct {
    const char *glyph;
    EdgeState   edge[N_DIRS];   /* [N, E, S, W] */
    Weight      weight;         /* drives color choice */
} Tile;

/*
 * tiles[] — 34-entry alphabet.
 *
 *   0           : blank (no connections, weight=BLANK)
 *   1..11       : light pipes — cyan, glyphs in {' ', '-', '|', '+'}
 *   12..22      : heavy pipes — pink, same glyph set
 *   23..33      : double pipes — gold, same glyph set
 *
 * Each weight is the full 11-tile T-junction set, so the constraint
 * solver always has every connectivity option available — no dead-end
 * stubs that would force contradictions.
 */
#define E_NONE   EDGE_NONE
#define E_L      EDGE_LIGHT
#define E_H      EDGE_HEAVY
#define E_D      EDGE_DOUBLE

/* ASCII-only glyphs for terminal portability (no UTF-8/locale dependency).
 * The three weight classes use the same character set ({' ', '-', '|',
 * '+'}); the visible difference between LIGHT/HEAVY/DOUBLE comes from
 * COLOR alone (cyan / pink / gold). Connectivity is fully tracked in the
 * 4-valued edge[] flags, so the algorithm enforces "weights only meet
 * across NONE seams" exactly as before. */
static const Tile tiles[N_TILES] = {
    /* 0  blank */ { " ",  {E_NONE, E_NONE, E_NONE, E_NONE}, WEIGHT_BLANK  },

    /* light — weight LIGHT, edges in {NONE, LIGHT} */
    /* 1  ─ */ { "-", {E_NONE, E_L,    E_NONE, E_L   }, WEIGHT_LIGHT },
    /* 2  │ */ { "|", {E_L,    E_NONE, E_L,    E_NONE}, WEIGHT_LIGHT },
    /* 3  ┌ */ { "+", {E_NONE, E_L,    E_L,    E_NONE}, WEIGHT_LIGHT },
    /* 4  ┐ */ { "+", {E_NONE, E_NONE, E_L,    E_L   }, WEIGHT_LIGHT },
    /* 5  └ */ { "+", {E_L,    E_L,    E_NONE, E_NONE}, WEIGHT_LIGHT },
    /* 6  ┘ */ { "+", {E_L,    E_NONE, E_NONE, E_L   }, WEIGHT_LIGHT },
    /* 7  ├ */ { "+", {E_L,    E_L,    E_L,    E_NONE}, WEIGHT_LIGHT },
    /* 8  ┤ */ { "+", {E_L,    E_NONE, E_L,    E_L   }, WEIGHT_LIGHT },
    /* 9  ┬ */ { "+", {E_NONE, E_L,    E_L,    E_L   }, WEIGHT_LIGHT },
    /*10  ┴ */ { "+", {E_L,    E_L,    E_NONE, E_L   }, WEIGHT_LIGHT },
    /*11  ┼ */ { "+", {E_L,    E_L,    E_L,    E_L   }, WEIGHT_LIGHT },

    /* heavy — weight HEAVY, edges in {NONE, HEAVY} */
    /*12  ━ */ { "-", {E_NONE, E_H,    E_NONE, E_H   }, WEIGHT_HEAVY },
    /*13  ┃ */ { "|", {E_H,    E_NONE, E_H,    E_NONE}, WEIGHT_HEAVY },
    /*14  ┏ */ { "+", {E_NONE, E_H,    E_H,    E_NONE}, WEIGHT_HEAVY },
    /*15  ┓ */ { "+", {E_NONE, E_NONE, E_H,    E_H   }, WEIGHT_HEAVY },
    /*16  ┗ */ { "+", {E_H,    E_H,    E_NONE, E_NONE}, WEIGHT_HEAVY },
    /*17  ┛ */ { "+", {E_H,    E_NONE, E_NONE, E_H   }, WEIGHT_HEAVY },
    /*18  ┣ */ { "+", {E_H,    E_H,    E_H,    E_NONE}, WEIGHT_HEAVY },
    /*19  ┫ */ { "+", {E_H,    E_NONE, E_H,    E_H   }, WEIGHT_HEAVY },
    /*20  ┳ */ { "+", {E_NONE, E_H,    E_H,    E_H   }, WEIGHT_HEAVY },
    /*21  ┻ */ { "+", {E_H,    E_H,    E_NONE, E_H   }, WEIGHT_HEAVY },
    /*22  ╋ */ { "+", {E_H,    E_H,    E_H,    E_H   }, WEIGHT_HEAVY },

    /* double — weight DOUBLE, edges in {NONE, DOUBLE} */
    /*23  ═ */ { "-", {E_NONE, E_D,    E_NONE, E_D   }, WEIGHT_DOUBLE },
    /*24  ║ */ { "|", {E_D,    E_NONE, E_D,    E_NONE}, WEIGHT_DOUBLE },
    /*25  ╔ */ { "+", {E_NONE, E_D,    E_D,    E_NONE}, WEIGHT_DOUBLE },
    /*26  ╗ */ { "+", {E_NONE, E_NONE, E_D,    E_D   }, WEIGHT_DOUBLE },
    /*27  ╚ */ { "+", {E_D,    E_D,    E_NONE, E_NONE}, WEIGHT_DOUBLE },
    /*28  ╝ */ { "+", {E_D,    E_NONE, E_NONE, E_D   }, WEIGHT_DOUBLE },
    /*29  ╠ */ { "+", {E_D,    E_D,    E_D,    E_NONE}, WEIGHT_DOUBLE },
    /*30  ╣ */ { "+", {E_D,    E_NONE, E_D,    E_D   }, WEIGHT_DOUBLE },
    /*31  ╦ */ { "+", {E_NONE, E_D,    E_D,    E_D   }, WEIGHT_DOUBLE },
    /*32  ╩ */ { "+", {E_D,    E_D,    E_NONE, E_D   }, WEIGHT_DOUBLE },
    /*33  ╬ */ { "+", {E_D,    E_D,    E_D,    E_D   }, WEIGHT_DOUBLE },
};

#undef E_NONE
#undef E_L
#undef E_H
#undef E_D

/* compat[t][d] — see wfc_learn.c §5 for the why. uint64_t because
 * we have 34 tiles. Built once at startup. */
static uint64_t compat[N_TILES][N_DIRS];

static void wfc_build_compat(void)
{
    for (int t = 0; t < N_TILES; t++) {
        for (int d = 0; d < N_DIRS; d++) {
            uint64_t mask = 0;
            int od = opposite(d);
            for (int t2 = 0; t2 < N_TILES; t2++) {
                if (tiles[t].edge[d] == tiles[t2].edge[od])
                    mask |= ((uint64_t)1 << t2);
            }
            compat[t][d] = mask;
        }
    }
}

/*
 * Grid — the WFC working state: the "wave" (Gumin 2016) plus the machinery
 * that collapses it.  Each cell's `mask` is its SUPERPOSITION — a bitmask of
 * the tiles still possible there (bit i set ⇒ tile i allowed); popcount(mask)
 * is the cell's entropy.  The algorithm only ever CLEARS bits, never sets them
 * — monotone shrinking, which is what makes WFC terminate.  (uint64_t here, vs
 * uint16_t in learn.c, because the alphabet is 34 tiles not 12.)
 *
 * 1-D arrays addressed by idx = y*w + x:
 *   mask       — the wave (per-cell tile-possibility bitmask).
 *   prop_glow  — EFFECTS only: the yellow propagation wave; set to 1.0 by
 *                grid_propagate_one, decayed in scene_tick, read by render —
 *                never feeds back into the algorithm.
 *   queue / in_queue / qhead / qtail — the BFS propagation frontier (cells
 *                whose shrink still has to ripple outward): arc-consistency /
 *                AC-3 (Mackworth 1977) made concrete.  in_queue caps the queue
 *                at one entry per cell (see learn.c for the overflow bug story).
 *   done       — true once every cell has popcount 1.
 *   bad        — true if any cell's mask hit 0 (contradiction — no tile fits).
 *                More common here than in learn because several seed ripples of
 *                incompatible weight can collide; flagged, then re-seeded.
 *   collapsed_count / total_cells — HUD read-outs.   w, h — grid dimensions.
 */
typedef struct {
    int      w, h;
    uint64_t mask         [GRID_W_MAX * GRID_H_MAX];
    float    prop_glow    [GRID_W_MAX * GRID_H_MAX];   /* yellow growth wave */
    int      queue        [GRID_W_MAX * GRID_H_MAX];
    bool     in_queue     [GRID_W_MAX * GRID_H_MAX];
    int      qhead, qtail;

    bool     done;
    bool     bad;

    int      collapsed_count;
    int      total_cells;
} Grid;

static void grid_reset(Grid *g, int w, int h)
{
    g->w = w;
    g->h = h;
    g->total_cells = w * h;
    g->collapsed_count = 0;
    g->done  = false;
    g->bad   = false;
    g->qhead = 0;
    g->qtail = 0;

    int n = w * h;
    for (int i = 0; i < n; i++) {
        g->mask[i]          = ALL_TILES_MASK;
        g->prop_glow[i]     = 0.0f;   /* no startup wash — only the live
                                       * propagation wave paints yellow */
        g->in_queue[i]      = false;
    }
}

static inline int grid_idx(const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds(const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

static inline void grid_enqueue(Grid *g, int idx)
{
    if (g->in_queue[idx]) return;
    if (g->qtail >= g->total_cells) return;
    g->queue[g->qtail++] = idx;
    g->in_queue[idx] = true;
}

/* nth_set_tile — index of the n'th (0-based) set bit in `mask`, i.e. the n'th
 * still-possible tile.  Turns a uniform random draw into a tile choice. */
static int nth_set_tile(uint64_t mask, int n)
{
    while (mask) {
        int t = __builtin_ctzll(mask);
        if (n == 0) return t;
        n--;
        mask &= mask - 1;
    }
    return -1;   /* unreachable when n < popcount(mask) */
}

/* allowed_in_dir — the arc-consistency core (AC-3): given a cell whose options
 * are `mask`, the set of tiles a neighbour in direction d could legally be is
 * the union of compat[t][d] over every tile t still possible here. */
static uint64_t allowed_in_dir(uint64_t mask, int d)
{
    uint64_t allowed = 0;
    while (mask) {
        int t = __builtin_ctzll(mask);   /* index of lowest set bit */
        allowed |= compat[t][d];
        mask &= mask - 1;
    }
    return allowed;
}

/*
 * grid_pick_min_entropy — same heuristic as learn.c, with reservoir
 * sampling among ties to avoid row-major bias. Returns -1 when no
 * uncollapsed cell exists.
 */
static int grid_pick_min_entropy(const Grid *g)
{
    int best_n  = N_TILES + 1;
    int tied    = 0;
    int chosen  = -1;
    int n_cells = g->total_cells;

    for (int i = 0; i < n_cells; i++) {
        int entropy = __builtin_popcountll(g->mask[i]);   /* options left */
        if (entropy <= 1) continue;          /* skip collapsed / contradicted */
        if (entropy < best_n) {
            best_n = entropy;
            chosen = i;
            tied   = 1;
        } else if (entropy == best_n) {
            tied++;
            if ((rand() % tied) == 0) chosen = i;   /* reservoir sample ties */
        }
    }
    return chosen;
}

static void grid_collapse_cell(Grid *g, int idx)
{
    uint64_t m = g->mask[idx];
    int n = __builtin_popcountll(m);
    if (n <= 1) return;

    int chosen_tile = nth_set_tile(m, rand() % n);   /* uniform pick among options */

    g->mask[idx] = (uint64_t)1 << chosen_tile;       /* mask → a single tile */
    g->collapsed_count++;
    grid_enqueue(g, idx);                            /* schedule its neighbours */
}

/*
 * tighten_neighbour — apply arc-consistency to ONE neighbour: intersect its
 * options with what `my_mask` permits across direction d.  If it shrank, paint
 * the wavefront, record the effect (contradiction / implicit collapse), and
 * re-enqueue it so the constraint keeps propagating.
 */
static void tighten_neighbour(Grid *g, int nidx, uint64_t my_mask, int d)
{
    uint64_t before = g->mask[nidx];
    uint64_t after  = before & allowed_in_dir(my_mask, d);
    if (after == before) return;            /* neighbour unaffected */

    g->mask[nidx]      = after;
    g->prop_glow[nidx] = 1.0f;              /* EFFECTS: paint the wavefront */
    if (after == 0) {
        g->bad = true;                      /* contradiction — no tile fits */
    } else if (__builtin_popcountll(after) == 1
            && __builtin_popcountll(before) > 1) {
        g->collapsed_count++;               /* neighbour collapsed by elimination */
    }
    grid_enqueue(g, nidx);
}

/*
 * grid_propagate_one — pop ONE cell off the queue and tighten its four
 * neighbours.  One pop per call keeps the wavefront visible.  Returns true if
 * the queue had work.
 */
static bool grid_propagate_one(Grid *g)
{
    if (g->qhead >= g->qtail) return false;

    int idx = g->queue[g->qhead++];
    g->in_queue[idx] = false;
    int x = idx % g->w;                     /* unpack flat cell index */
    int y = idx / g->w;
    uint64_t my_mask = g->mask[idx];

    for (int d = 0; d < N_DIRS; d++) {
        int nx = x + dir_dx(d);
        int ny = y + dir_dy(d);
        if (!grid_in_bounds(g, nx, ny)) continue;
        tighten_neighbour(g, grid_idx(g, nx, ny), my_mask, d);
    }
    return true;
}

static bool grid_step(Grid *g)
{
    if (g->bad || g->done) return false;
    if (grid_propagate_one(g)) return true;

    int idx = grid_pick_min_entropy(g);
    if (idx < 0) { g->done = true; return false; }
    grid_collapse_cell(g, idx);
    return true;
}

/*
 * grid_seed_random — pick a random uncollapsed cell and force-collapse
 * it. Used at startup to drop N_INITIAL_SEEDS ripple centres.
 *
 * The "reservoir among uncollapsed" pattern means we don't have to
 * track collapse history — every call sees the current grid state.
 */
static void grid_seed_random(Grid *g)
{
    int chosen = -1;
    int tied   = 0;
    int n_cells = g->total_cells;
    for (int i = 0; i < n_cells; i++) {
        if (__builtin_popcountll(g->mask[i]) > 1) {
            tied++;
            if ((rand() % tied) == 0) chosen = i;
        }
    }
    if (chosen >= 0) grid_collapse_cell(g, chosen);
}

/* ===================================================================== */
/* §4  simulation — scene & the ONE per-tick combine (glow decay + ops)   */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   GROWING    : algorithm running normally — collapses + propagation.
 *                Enters HOLD when grid_step returns false with done=true.
 *   HOLD       : finished pattern displayed for HOLD_SECONDS.
 *                Enters GROWING via a fresh re-seed on timeout.
 *
 * Contradictions (g.bad=true) trigger an immediate reset without HOLD —
 * the user shouldn't have to stare at a stuck grid.
 *
 * `paused` is independent of the state machine: when paused, scene_tick
 * is a no-op (no ops, no glow decay, no state transitions). This freezes
 * whatever is on screen, useful for screenshots.
 */
typedef enum { SCENE_GROWING = 0, SCENE_HOLD = 1 } SceneState;

/*
 * Scene — the running showcase, read top-to-bottom as a table of contents.
 * Grouped by concept, NOT by which key changes them:
 *   WHAT   — g: the Grid being collapsed (the simulation proper).
 *   PHASE  — state + hold_timer: GROWING vs HOLD, and the HOLD countdown;
 *            paused freezes the whole tick.
 *   HOW    — ops_per_tick: grid_step calls per tick (the +/- spectacle pace).
 *   RENDER — theme: index into THEMES.  A palette choice, so it is grouped
 *            apart from the sim knob even though a key drives it.
 */
typedef struct {
    Grid       g;              /* WHAT:   the grid of superpositions          */
    SceneState state;          /* PHASE:  GROWING / HOLD                       */
    float      hold_timer;     /* PHASE:  seconds left in HOLD                 */
    bool       paused;         /* PHASE:  freeze the tick                      */
    int        ops_per_tick;   /* HOW:    grid_step calls per tick             */
    int        theme;          /* RENDER: index into THEMES (t/T cycles)       */
} Scene;

/*
 * scene_reset — full re-seed: every cell back to full superposition, then
 * drop N_INITIAL_SEEDS collapses to start a fresh pattern (which then grows
 * with its live yellow propagation wave — there is no startup wash).
 *
 * Called on first init, on user 'r', on a theme change, on contradiction,
 * and when the HOLD timer expires.
 */
static void scene_reset(Scene *s, int gw, int gh)
{
    grid_reset(&s->g, gw, gh);
    s->state      = SCENE_GROWING;
    s->hold_timer = 0.0f;
    for (int i = 0; i < N_INITIAL_SEEDS; i++) {
        grid_seed_random(&s->g);
    }
}

static void scene_init(Scene *s, int gw, int gh)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->ops_per_tick  = OPS_PER_TICK_DEF;
    scene_reset(s, gw, gh);
}

/* EFFECTS: fade the per-cell propagation wave toward 0 this tick.
 * exp(-rate*dt) is the textbook RC-circuit decay; computed directly since
 * dt is small and bounded. */
static void decay_wave(Grid *g, float dt)
{
    float decay = expf(-GLOW_DECAY_RATE * dt);
    int n = g->total_cells;
    for (int i = 0; i < n; i++) {
        g->prop_glow[i] *= decay;
    }
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    decay_wave(&s->g, dt);          /* EFFECTS: fade the wave before new ops */

    switch (s->state) {
    case SCENE_GROWING: {
        /* Run up to ops_per_tick algorithm operations. */
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!grid_step(&s->g)) break;
        }
        if (s->g.bad) {
            /* Contradiction — restart immediately. */
            scene_reset(s, s->g.w, s->g.h);
        } else if (s->g.done) {
            s->state      = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
        }
    } break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->g.w, s->g.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §5  render — palette + themes, per-weight routing, grid, HUD           */
/* ===================================================================== */

/*
 * Theme — one named palette for the three weight-class domains.  The weight
 * colour IS the visual signal here (every junction renders as '+', so colour
 * is the only differentiator between light / heavy / double regions), so a
 * theme recolours exactly those three and leaves the functional accents
 * (HUD / HINT / flash / wave / seed) fixed.  Within a theme the three tiers
 * are bright, mutually-distinct hues so the domains stay readable; theme 0
 * (DEFAULT) reproduces the original cyan / pink / gold look.
 */
typedef struct {
    const char *name;
    short c [3];   /* 256-colour: light, heavy, double */
    short c8[3];   /* 8-colour fallback                */
} Theme;

#define N_THEMES 10

static const Theme THEMES[N_THEMES] = {
    /*  name        light heavy double   8-colour fallback (light/heavy/double)   */
    { "DEFAULT", { 117, 213, 220 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_YELLOW } },
    { "AURORA",  {  51, 207, 154 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_GREEN  } },
    { "EMBER",   { 226, 208, 196 }, { COLOR_YELLOW, COLOR_YELLOW,  COLOR_RED    } },
    { "OCEAN",   { 159,  45,  33 }, { COLOR_CYAN,   COLOR_CYAN,    COLOR_BLUE   } },
    { "FOREST",  { 190, 154,  34 }, { COLOR_GREEN,  COLOR_GREEN,   COLOR_YELLOW } },
    { "NEON",    {  51, 201, 226 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_YELLOW } },
    { "CANDY",   { 159, 219, 213 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_MAGENTA} },
    { "ICE",     { 195, 159, 117 }, { COLOR_WHITE,  COLOR_CYAN,    COLOR_CYAN   } },
    { "SUNSET",  { 220, 208, 204 }, { COLOR_YELLOW, COLOR_YELLOW,  COLOR_RED    } },
    { "MONO",    { 252, 246, 255 }, { COLOR_WHITE,  COLOR_WHITE,   COLOR_WHITE  } },
};

/*
 * theme_apply — install one theme's three weight colours (light/heavy/double).
 * Re-callable at runtime (the 't'/'T' key); the fixed accent pairs set by
 * color_init are untouched.  Picks the 256-colour or 8-colour set per terminal.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &THEMES[idx];
    const short *c = (COLORS >= 256) ? t->c : t->c8;
    init_pair(PAIR_LIGHT,  c[0], -1);
    init_pair(PAIR_HEAVY,  c[1], -1);
    init_pair(PAIR_DOUBLE, c[2], -1);
}

/*
 * color_init — set the fixed accent pairs (per CLAUDE.md HUD Standard:
 * PAIR_HUD=226 bright yellow, PAIR_HINT=51 bright cyan, both A_BOLD on use;
 * WAVE=226 yellow), then lay down the DEFAULT theme's weight colours.
 * 't'/'T' later swap only the weight pairs.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_WAVE,       226, -1);
    } else {
        init_pair(PAIR_HUD,        COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,    -1);
        init_pair(PAIR_WAVE,       COLOR_YELLOW,  -1);
    }
    theme_apply(0);   /* DEFAULT weight palette — matches Scene.theme = 0 */
}

/*
 * Screen — the terminal viewport: its current size in cells.  A render-target
 * concept kept as its own narrow type so the §5 functions take Screen* alone
 * and never reach for the whole App (keeps render decoupled from app/sim).
 *
 * Per-frame pattern (framework.c §7):
 *   erase → scene_draw → HUD → wnoutrefresh(stdscr) → doupdate.
 *
 * Rendering priority per cell (theme-first):
 *   collapsed                  →  weight (theme) colour — always, immediately
 *   uncollapsed + prop wave    →  yellow bold '#' (advancing growth frontier)
 *   uncollapsed, no wave       →  blank
 * No entropy digits (a teaching crutch from learn.c) — keeps the spectacle
 * clean.  Glyphs go out via mvaddstr; setlocale in screen_init keeps output
 * byte-safe, though the current tile alphabet is ASCII (see Tile).
 *
 *   cols, rows : terminal dimensions, refreshed on resize.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    setlocale(LC_ALL, "");
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

/* Map a tile's weight class to its color pair. Blank uses LIGHT but
 * wouldn't be drawn anyway (renderer skips uncollapsed/blank cells). */
static int weight_pair(Weight w)
{
    switch (w) {
    case WEIGHT_LIGHT:  return PAIR_LIGHT;
    case WEIGHT_HEAVY:  return PAIR_HEAVY;
    case WEIGHT_DOUBLE: return PAIR_DOUBLE;
    case WEIGHT_BLANK:  return PAIR_LIGHT;   /* irrelevant — see above */
    }
    return PAIR_LIGHT;
}

/*
 * draw_cell — paint one cell at screen (sy,sx), theme-first: a DECIDED cell
 * always shows its weight (theme) colour (so the scene fills with theme colour
 * as it grows); the yellow wave is drawn only on a still-undecided cell, a thin
 * advancing frontier that never covers the theme-coloured pipes behind it.
 */
static void draw_cell(const Scene *s, int idx, int sy, int sx)
{
    const Grid *g = &s->g;
    uint64_t m = g->mask[idx];
    bool collapsed = (__builtin_popcountll(m) == 1);

    if (collapsed) {
        int t = __builtin_ctzll(m);
        if (tiles[t].weight == WEIGHT_BLANK) return;     /* blank seam — leave blank */
        int color_pair = weight_pair(tiles[t].weight);
        attron(COLOR_PAIR(color_pair) | A_BOLD);
        mvaddstr(sy, sx, tiles[t].glyph);
        attroff(COLOR_PAIR(color_pair) | A_BOLD);
    } else if (g->prop_glow[idx] > GLOW_FLASH_THRESHOLD) {
        attron(COLOR_PAIR(PAIR_WAVE) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)'#');
        attroff(COLOR_PAIR(PAIR_WAVE) | A_BOLD);
    }
    /* else: undecided and no wave — leave blank */
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid *g = &s->g;

    /* Centre the grid, leaving a 1-row HUD top + 1-row hint bottom margin. */
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - 2) - g->h) / 2 + 1;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 1) gy0 = 1;

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 1 || sy >= rows - 1) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            draw_cell(s, grid_idx(g, x, y), sy, sx);
        }
    }
}

/* mode_label() — the current run state as a fixed-width HUD label. */
static const char *mode_label(const Scene *s)
{
    if (s->paused)             return "PAUSED ";
    if (s->state == SCENE_HOLD) return "HOLD   ";
    if (s->g.bad)              return "BAD!   ";
    return                            "GROWING";
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Grid *g = &s->g;
    const char *state_str = mode_label(s);

    /* Top-right status — PAIR_HUD bold per CLAUDE.md HUD Standard. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops/tick:%3d  %s  %5d/%-5d ",
             fps, sim_fps, s->ops_per_tick, state_str,
             g->collapsed_count, g->total_cells);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Top-left title + active theme. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " WFC showcase  theme:%-7s ", THEMES[s->theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — A_BOLD, never A_DIM (CLAUDE.md). */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  t:theme  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §6  app — signals, key/resize events, fixed-timestep main loop         */
/* ===================================================================== */

/*
 * App — the running program: the Scene being animated, the terminal it draws
 * to, and the loop/lifecycle state that is neither simulation nor render.
 * Kept distinct from Scene so the render layer can take Screen* alone, and so
 * the signal handler can reach the run flags via the single g_app instance.
 *   DOMAIN    — scene + screen: the simulation and its draw target.
 *   PACING    — sim_fps: fixed-timestep rate (the [ / ] knob).
 *   GEOMETRY  — grid_w/grid_h: chosen grid size, re-fit to the terminal on
 *               resize (clamped to GRID_*_MAX so the queue can't overflow).
 *   LIFECYCLE — running/need_resize: signal-written flags (sig_atomic_t).
 */
typedef struct {
    Scene                 scene;        /* DOMAIN:    the simulation       */
    Screen                screen;       /* DOMAIN:    its draw target      */
    int                   sim_fps;      /* PACING:    fixed-timestep rate  */
    int                   grid_w, grid_h; /* GEOMETRY: chosen grid size    */
    volatile sig_atomic_t running;      /* LIFECYCLE: clear to exit        */
    volatile sig_atomic_t need_resize;  /* LIFECYCLE: set by SIGWINCH      */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_grid_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 2;     /* leave HUD top + hint bottom */
    if (mw < 8) mw = 8;
    if (mh < 4) mh = 4;
    if (mw > GRID_W_MAX) mw = GRID_W_MAX;
    if (mh > GRID_H_MAX) mh = GRID_H_MAX;
    app->grid_w = mw;
    app->grid_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_grid_size(app);
    scene_reset(&app->scene, app->grid_w, app->grid_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->grid_w, app->grid_h);
        break;
    case '=': case '+':
        if (s->ops_per_tick < OPS_PER_TICK_MAX) s->ops_per_tick *= 2;
        if (s->ops_per_tick > OPS_PER_TICK_MAX) s->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        s->ops_per_tick /= 2;
        if (s->ops_per_tick < OPS_PER_TICK_MIN) s->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        scene_reset(s, app->grid_w, app->grid_h);   /* restart in the new theme */
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme);
        scene_reset(s, app->grid_w, app->grid_h);   /* restart in the new theme */
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    wfc_build_compat();
    app_pick_grid_size(app);
    scene_init(&app->scene, app->grid_w, app->grid_h);

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
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* spiral-of-death guard */

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
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
