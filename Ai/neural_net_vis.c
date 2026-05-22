/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * neural_net_vis.c — neural network architecture visualiser (Stage 2)
 *
 * DEMO: Draws a feed-forward neural network as columns of `(O)` neurons
 *       fully connected by slope-character lines. One particle per input
 *       neuron drifts forward through the network — at each layer it
 *       picks a random target in the next layer and travels along that
 *       edge — and on reaching the output layer loops back to a random
 *       input neuron. Resize the terminal, change layer count with
 *       `[`/`]`, neurons-per-layer with `-`/`+`, line thickness with
 *       `<`/`>`; the particle pool reseeds whenever the network shape
 *       changes.
 *
 *       Stage 3 will add trails, fan-in pulse on neuron arrival, and
 *       optional weighted-random target selection.
 *
 * Study alongside: artistic/galaxy.c (similar dot-cluster + edge pattern),
 *                  artistic/graph_search.c (node + edge animation).
 *
 * Section map:
 *   §1 config    — MIN/MAX/DEFAULT layers, neurons, particle speed
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — bright + connection + particle foreground per theme
 *   §4 layout    — neuron_cell() coord seam + LINE_THICKNESS glyph table
 *   §5 net       — NetArch / RenderConfig / NetUI sub-structs
 *   §6 particle  — Particle pool: reset / tick / draw + edge-by-edge hop
 *   §7 scene     — Screen + FrameTimer + Scene container;
 *                  draw_connections + particle_draw + draw_neurons + HUD
 *   §8 screen    — ncurses init / cleanup
 *   §9 app       — signals, dt tracking, key handling, main loop
 *
 * Keys:  [/]  decrease/increase layer count (2..12)
 *        -/+  decrease/increase neurons per layer (2..16)
 *        </>  thinner/thicker connections (dot / thin / bold / heavy)
 *        t    cycle theme   r reset   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra Ai/neural_net_vis.c \
 *       -o neural_net_vis -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Pure layout — every layer is a vertical column at a
 *                 fixed x-fraction of the screen; every neuron is a cell
 *                 at a fixed y-fraction within the layer; every neuron in
 *                 layer i connects to every neuron in layer i+1. No data
 *                 flow yet — this stage only fixes the geometry.
 *
 * Data-structure: A Scene owns the whole program state, composed of
 *                 concept-sized sub-structs:
 *
 *                   NetArch       — n_layers, n_per_layer
 *                                     (architectural — mutating triggers
 *                                      particle_reset)
 *                   RenderConfig  — thickness, theme
 *                                     (purely cosmetic — no reset)
 *                   NetUI         — paused
 *                                     (keyboard flag)
 *                   ParticlePool  — pool of dots flowing along edges
 *                   Screen        — current ncurses dimensions
 *                   FrameTimer    — dt + EWMA fps + sleep budget
 *
 *                 Plus volatile sig_atomic_t flags (running, need_resize)
 *                 for signal handlers. Neuron positions are NOT stored
 *                 — they're computed on demand from (layer, idx, rows,
 *                 cols), so resize is free: the next frame reads the new
 *                 dimensions and places neurons in the right place
 *                 automatically.
 *
 * Rendering     : Per frame: erase, draw all O(L · N²) connections in a
 *                 dim colour pair, draw all O(L · N) neurons in a bright
 *                 pair over the connections, then HUD. Connections are
 *                 drawn by linear-step interpolation (cheaper than full
 *                 Bresenham, accurate enough at terminal resolution),
 *                 with the two endpoints skipped so the neuron glyph
 *                 lands cleanly on top.
 *
 * Performance   : O(layers · neurons²) line draws per frame — at the
 *                 12-layer × 16-neuron cap that's ~3000 short lines, well
 *                 under one millisecond on any modern CPU.
 *
 * References    : grouped by the concepts this file teaches. Nine
 *                  entries — one or two strong picks per topic. The
 *                  free online refs (Nielsen, Olah, Playground,
 *                  Fiedler) are the cheapest places to start.
 *
 *   ── Feed-forward neural networks (concept: the picture itself) ───
 *   Goodfellow, Bengio & Courville, "Deep Learning" (MIT Press,
 *     2016) ch. 6. The canonical textbook treatment of feed-forward
 *     networks. Section 6.1 introduces the layered-column diagram
 *     notation this file animates.
 *   Nielsen, "Neural Networks and Deep Learning" (2015, free online
 *     at http://neuralnetworksanddeeplearning.com). Chapter 1's
 *     diagram of a fully-connected feed-forward network is
 *     essentially the picture this file draws. Ideal first read for
 *     anyone new to NNs — visual, no calculus required upfront.
 *   LeCun, Bengio & Hinton, "Deep learning" (Nature 521:436–444,
 *     2015). Landmark survey; Fig. 1's columnar diagram is the
 *     canonical layered visualisation.
 *   Schmidhuber, "Deep Learning in Neural Networks: An Overview"
 *     (Neural Networks 61, 2015). Comprehensive history of NN
 *     architectures — supports the "layered feed-forward" abstraction
 *     used here with the historical context that motivated it.
 *
 *   ── Network visualisation & intuition ────────────────────────────
 *   Olah, "Neural Networks, Manifolds, and Topology"
 *     (colah.github.io, 2014). Visual / geometric intuition for what
 *     each layer DOES to the input space — the philosophical
 *     companion to this file's purely-geometric picture.
 *   Smilkov, Carter, et al., "TensorFlow Playground"
 *     (playground.tensorflow.org, 2017). Interactive in-browser NN
 *     visualiser with similar fully-connected-layer aesthetics. This
 *     file is the terminal cousin of the same idea.
 *
 *   ── Line drawing (§4 layout + §7 draw_line) ──────────────────────
 *   Bresenham, "Algorithm for computer control of a digital plotter"
 *     (IBM Systems Journal 4(1):25–30, 1965). Standard reference for
 *     line rasterisation. This file uses simpler linear-step
 *     interpolation (steps = max(|Δr|, |Δc|)) — accurate enough at
 *     terminal resolution — but the per-cell glyph-from-local-step
 *     idea generalises from Bresenham's stepped-error pattern.
 *
 *   ── Rendering (§3, §7) ───────────────────────────────────────────
 *   Padala, "NCURSES Programming HOWTO" (The Linux Documentation
 *     Project). Practical reference for the ncurses API used in
 *     §3 (color_init) and §7 (mvaddch, mvaddstr, attron).
 *     https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *   ── Game loop & numerical timing (§9 main loop) ──────────────────
 *   Fiedler ("Gaffer on Games"), "Fix Your Timestep!" (2004). The
 *     dt-cap + EWMA-fps pattern in main() — explains why measuring
 *     wall-clock dt with a DT_CAP_S guard keeps the particle flow
 *     stable at any frame rate.
 *     https://gafferongames.com/post/fix_your_timestep/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A feed-forward neural network diagram is a column of dots, then a
 * column of dots, then a column of dots, fully connected.  Geometry is
 * the whole demo: the position of every neuron is computed on demand
 * from (layer, idx, rows, cols) — nothing stored, nothing pre-baked.
 * On top of that geometry, a small pool of particles drifts forward
 * one edge at a time, picking a random next-layer target each hop.
 * No real activations or weights — but the visual unmistakably shows
 * "data flows left-to-right through fully-connected layers."
 *
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. neuron_cell(layer, idx) returns
 *       col = (layer + 1) · cols / (n_layers + 1)
 *       row = (idx   + 1) · (rows - 1) / (n_per_layer + 1)
 *     The "+1" margins distribute equal space at all four edges.
 *  2. draw_connections: for each pair of adjacent layers, for each
 *     pair (a,b) of neurons (n² edges per layer pair), call draw_line
 *     between their cells.  draw_line interpolates linearly with
 *     steps = max(|dr|, |dc|), and picks a glyph PER CELL based on
 *     the LOCAL step direction (hor/ver/down/up) — not the global
 *     slope — so a near-horizontal line renders as '------\------'
 *     instead of staircased back-slashes.
 *  3. Particle pool: one particle per input neuron.  Each holds
 *     {from_layer, from_idx, to_idx, t∈[0,1], speed} where speed has
 *     ±40% jitter so dots desync within ~3 hops.
 *  4. particle_tick: t += speed·dt.  When t≥1: subtract 1, advance
 *     to next layer (from_idx ← to_idx, pick new to_idx).  At output
 *     layer, loop back to a random input neuron.  The `while` loop
 *     handles the rare large-dt case of crossing 2 edges in one tick.
 *  5. particle_draw: linearly interpolate between source and target
 *     cells, paint '*' at sr,sc.  Drawn between connections and
 *     neurons so neurons cap the trajectory cleanly.
 *  6. Endpoints i=0 and i=steps are skipped in draw_line so the
 *     connection lines don't overdraw the '(O)' neuron sigil.
 *  7. HUD top-right (params + fps), hint strip bottom-left.
 *
 * KEY FORMULAS
 * ────────────
 *  Neuron col       col = (L + 1) · cols / (n_layers + 1)
 *  Neuron row       row = (I + 1) · (rows - 1) / (n_per_layer + 1)
 *  Edge interp      sr = ar + (br - ar) · t,  sc = ac + (bc - ac) · t
 *  Edge advance     t ← t + speed · dt
 *  Speed jitter     speed = SPEED · (1 - J + 2·J·rand01)
 *  Bresenham steps  steps = max(|Δr|, |Δc|);  step glyph from local
 *                   (dsr, dsc): hor / ver / dn / up
 *  Connection cost  per frame O(L · n²) edges, L=layers, n=per-layer
 *  Particle cost    per tick O(p) where p = n_per_layer
 *
 *
 * Background you need
 * ───────────────────
 *   - Concept of a feed-forward neural network: layers stacked
 *     left-to-right, each neuron connected to every neuron in the
 *     next layer.  No prior NN math required — this file is the
 *     PICTURE, not the math.
 *   - Linear interpolation: lerp(a, b, t) = a + t·(b - a).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Backpropagation, gradient descent, activation functions.
 *     This file does ZERO learning.  Particles pick random
 *     targets — they're a visual flow indicator, not real
 *     activations.
 *   - Tensor frameworks (TensorFlow, PyTorch).  We don't even have
 *     a weights matrix.
 *   - Convolutional / recurrent architectures.  Pure feed-forward.
 *
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <locale.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS         30

#define MIN_LAYERS          2
#define MAX_LAYERS         12
#define DEFAULT_LAYERS      3

#define MIN_NEURONS         2
#define MAX_NEURONS        16
#define DEFAULT_NEURONS     5

#define MIN_THICKNESS       0
#define MAX_THICKNESS       3
#define DEFAULT_THICKNESS   1

/* One particle per input neuron. Speed is in edges-per-second; jitter
 * gives each particle a random factor in [1−J, 1+J] times the base. */
#define MAX_PARTICLES       MAX_NEURONS
#define PARTICLE_SPEED      0.6f
#define PARTICLE_JITTER     0.4f
#define PARTICLE_GLYPH      '*'
#define DT_CAP_S            0.10f

#define N_THEMES            4

/* ── §1.1 timing & EWMA fps ─────────────────────────────────────── */

#define NS_PER_SEC          1000000000LL

/* EWMA one-pole IIR low-pass on instantaneous fps:
 *   fps = fps · EWMA_RETAIN  +  instantaneous · EWMA_NEW
 * Window ≈ 1 / (1 - EWMA_RETAIN) = 20 frames. Stable enough for the
 * HUD digits not to jitter every frame, fast enough to track real
 * fps changes within a fraction of a second. */
#define EWMA_RETAIN         0.95
#define EWMA_NEW            0.05

/* ── §1.2 neuron sigil + particle topology ──────────────────────── */

/* The '(O)' neuron rendering is three cells: left paren, the 'O',
 * right paren. NEURON_SIGIL_HALF is the half-width on either side
 * of the centre cell. */
#define NEURON_SIGIL_HALF   1
#define NEURON_GLYPH_LEFT   '('
#define NEURON_GLYPH_CENTRE 'O'
#define NEURON_GLYPH_RIGHT  ')'

/* In a network with L layers, particles can be travelling along edges
 * sourced from layers [0, L - LAYER_EDGE_TAIL]. The last layer has no
 * forward edge → its index isn't a valid `from_layer`. */
#define LAYER_EDGE_TAIL     2

/* Colour pair IDs */
#define PAIR_NEURON         1   /* bright (O) glyph                  */
#define PAIR_CONN           2   /* connection-line characters        */
#define PAIR_PARTICLE       3   /* travelling '*' dots               */
#define PAIR_HUD            4   /* status bar (top right)            */
#define PAIR_HINT           5   /* key hints (bottom left)           */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Per-theme [neuron_fg, connection_fg, particle_fg] for 256-color and
 * 8-color fallback. The particle accent is chosen to contrast with both
 * the neuron and connection tones so travelling dots stay visible
 * regardless of which line they're crossing. */
static const short THEME_FG_256[N_THEMES][3] = {
    {  51,  45, 226 },   /* cyan family    — aqua / teal   / gold      */
    {  82,  34, 220 },   /* green family   — lime / forest / amber     */
    { 220, 178, 207 },   /* amber family   — gold / honey  / pink      */
    { 207, 134,  51 },   /* magenta family — pink / orchid / aqua      */
};
static const short THEME_FG_8[N_THEMES][3] = {
    { COLOR_CYAN,    COLOR_CYAN,    COLOR_YELLOW  },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW  },
    { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN    },
};

/* CLAUDE.md HUD spec: PAIR_HUD = bright yellow on default bg, PAIR_HINT
 * = bright cyan on default bg. Both used with A_BOLD in scene_draw —
 * never A_DIM (vanishes against animation). Out-of-theme so they stay
 * legible no matter which network theme is active. */
#define HUD_FG_256          226    /* bright yellow */
#define HINT_FG_256          51    /* bright cyan   */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int   x256   = (COLORS >= 256);
    short neuron = x256 ? THEME_FG_256[theme][0] : THEME_FG_8[theme][0];
    short conn   = x256 ? THEME_FG_256[theme][1] : THEME_FG_8[theme][1];
    short part   = x256 ? THEME_FG_256[theme][2] : THEME_FG_8[theme][2];
    init_pair(PAIR_NEURON,   neuron, -1);
    init_pair(PAIR_CONN,     conn,   -1);
    init_pair(PAIR_PARTICLE, part,   -1);
    init_pair(PAIR_HUD,      x256 ? HUD_FG_256  : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,     x256 ? HINT_FG_256 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  layout — single coordinate seam                                     */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * neuron_cell — turn an abstract (layer, idx) address into a concrete
 * terminal cell. The whole network depends on this one function — change
 * it and the entire layout follows.
 *
 *   col = (layer + 1) · cols       / (n_layers + 1)
 *   row = (idx   + 1) · (rows - 1) / (n_in_layer + 1)
 *
 * The +1 in the denominator leaves equal margin at the top, bottom and
 * sides of the network. The (rows - 1) reserves the bottom row for the
 * key-hint strip.
 */
static void neuron_cell(int layer, int idx,
                        int n_layers, int n_in_layer,
                        int rows, int cols,
                        int *out_row, int *out_col)
{
    *out_col = (layer + 1) * cols       / (n_layers + 1);
    *out_row = (idx   + 1) * (rows - 1) / (n_in_layer + 1);
}

/*
 * LineStyle — one row of the connection-glyph palette ramp.
 *
 * INTENT
 *   Connection lines render edge segments cell-by-cell. At each
 *   cell, the local step direction (horizontal, vertical, down-
 *   diagonal, up-diagonal) selects which glyph to stamp from this
 *   row. The `'</'` and `'>'` keys cycle through LINE_THICKNESS[]
 *   so the user can dial visual weight from "barely there" (dot)
 *   to "double-line emphatic" (heavy) without retiling the network.
 *
 * WHY FOUR GLYPHS (not one)
 *   A single glyph for the whole line (e.g. all '-') reads as a
 *   staircased pile of dashes when the line has a real slope. By
 *   picking the glyph from the LOCAL per-cell step, shallow lines
 *   render as long horizontal runs with occasional '\' or '/' at
 *   row changes — visually faithful to the actual geometry.
 *
 * WHY MULTI-BYTE STRINGS (not chtype)
 *   The "heavy" tier uses UTF-8 box-drawing characters (═ ║ ╲ ╱).
 *   ncurses' chtype only fits 7-bit ASCII; mvaddstr handles
 *   multi-byte transparently as long as setlocale(LC_ALL, "") ran
 *   first (it does, in screen_init).
 *
 * THICKNESS RAMP (LINE_THICKNESS[0..3])
 *   0  dot    — '.' everywhere + A_DIM             faintest, exploratory
 *   1  thin   — ASCII '- | \\ /'                   default
 *   2  bold   — ASCII '- | \\ /' + A_BOLD          visible, slope-aware
 *   3  heavy  — UTF-8 '═ ║ ╲ ╱' + A_BOLD           emphatic, slope-aware
 *
 * ALGORITHM REFERENCE
 *   The per-cell glyph-from-local-step idea is a cousin of
 *   Bresenham's line algorithm: the same "what's the dominant step
 *   right here?" question, evaluated incrementally rather than from
 *   the global Δr/Δc slope. See Bresenham (1965) in References.
 */
typedef struct {
    const char *hor;    /* glyph for purely-horizontal step (dsr = 0).   *
                         * Examples: "-" or "═".                          */
    const char *ver;    /* glyph for purely-vertical step (dsc = 0).     *
                         * Examples: "|" or "║".                          */
    const char *dn;     /* glyph for down-diagonal (dsr & dsc same sign).*
                         * Examples: "\\" or "╲".                         */
    const char *up;     /* glyph for up-diagonal (dsr & dsc opposite).   *
                         * Examples: "/" or "╱".                          */
    attr_t      attr;   /* ncurses attribute OR'd into PAIR_CONN at      *
                         * draw time. 0 / A_BOLD / A_DIM only.            */
    const char *name;   /* short label shown in the HUD ("dot", "thin", *
                         * "bold", "heavy"). Lives next to attr so the   *
                         * HUD and the rendering stay in sync.            */
} LineStyle;

static const LineStyle LINE_THICKNESS[] = {
    /* 0 */ { ".", ".", ".",  ".",  A_DIM,   "dot"   },
    /* 1 */ { "-", "|", "\\", "/",  0,       "thin"  },
    /* 2 */ { "-", "|", "\\", "/",  A_BOLD,  "bold"  },
    /* 3 */ { "═", "║", "╲",  "╱",  A_BOLD,  "heavy" },
};

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  net                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * §5.1 NetArch — the architecture: how many layers, how many neurons.
 *
 * INTENT
 *   These two scalars fully define the NETWORK SHAPE. Every neuron
 *   position, every connection edge, every particle pool size — all
 *   derive from (n_layers, n_per_layer). The struct is intentionally
 *   minimal so "what shape is the network?" is a single question
 *   with one short answer.
 *
 * INVARIANT
 *   Mutating either field changes how many neurons exist and therefore
 *   how many particles the pool should hold. The caller MUST call
 *   particle_reset() after ANY write here — the '[' / ']' / '-' / '+'
 *   keys all do this. The contract is enforced by separation: the
 *   reset path takes  const NetArch *  so it always sees the new
 *   value.
 *
 * WHY UNIFORM WIDTH (every layer has n_per_layer neurons)
 *   Real feed-forward networks have variable-width layers — typically
 *   tapering from a wider input toward a narrower output. The
 *   uniform model here is a visualisation simplification: it
 *   foregrounds the LAYERED structure (the demo's pedagogical
 *   point) without burdening the reader with per-layer width
 *   bookkeeping. Easy to extend later: replace n_per_layer with an
 *   int[MAX_LAYERS] and propagate through neuron_cell().
 *
 * ALGORITHM REFERENCE
 *   Goodfellow/Bengio/Courville (2016) ch. 6 — the columnar
 *   feed-forward diagram. n_layers = depth, n_per_layer = uniform
 *   width.
 */
typedef struct {
    int n_layers;       /* number of network layers (input + hidden  *
                         * + output, counted together).               *
                         * Clamped to [MIN_LAYERS, MAX_LAYERS]        *
                         * = [2, 12]. Default DEFAULT_LAYERS = 3.    *
                         * '[' / ']' keys nudge. Each mutation       *
                         * triggers particle_reset.                  */
    int n_per_layer;    /* uniform layer width — every layer has    *
                         * this many neurons.                        *
                         * Clamped to [MIN_NEURONS, MAX_NEURONS]      *
                         * = [2, 16]. Default DEFAULT_NEURONS = 5.   *
                         * '-' / '+' keys nudge. Each mutation       *
                         * triggers particle_reset (pool size       *
                         * matches input-layer width).               */
} NetArch;

/*
 * §5.2 RenderConfig — visual choices that don't change topology.
 *
 * INTENT
 *   Cosmetic flags. Mutating these NEVER affects the network shape,
 *   the particle pool, or the neuron geometry — only the glyphs and
 *   colours used to paint them. The separation from NetArch makes
 *   the contract a TYPE-LEVEL property: a flag here cannot trigger
 *   particle_reset() because the reset path doesn't consult
 *   RenderConfig.
 *
 * INVARIANT (READER GUIDE FOR FUTURE EXTENSIONS)
 *   When adding a new visual flag, ask: does this change WHAT IS
 *   DRAWN (the shape, the count, the geometry)? If YES, it belongs
 *   in NetArch (and needs particle_reset on mutation). If NO (only
 *   changes glyphs, colours, opacity), it belongs here.
 */
typedef struct {
    int thickness;      /* index into LINE_THICKNESS[] (§4). Clamped *
                         * to [MIN_THICKNESS, MAX_THICKNESS] =       *
                         * [0, 3]. Default DEFAULT_THICKNESS = 1     *
                         * ("thin"). '<' / '>' keys nudge.           */
    int theme;          /* index into THEME_FG_256 / THEME_FG_8[].   *
                         * 0 .. N_THEMES - 1 = 0 .. 3. Default 0.    *
                         * 't' key cycles; color_init() rebinds      *
                         * PAIR_NEURON / PAIR_CONN / PAIR_PARTICLE  *
                         * on change.                                */
} RenderConfig;

/*
 * §5.3 NetUI — keyboard-toggled flags.
 *
 * INTENT
 *   Pure UI state. Separating from NetArch / RenderConfig documents
 *   at the type level that this is "the human's state", not "the
 *   network's state". Mutating these flags never affects the
 *   network shape, the renderer, or the particle pool — only how
 *   the loop paces and what label appears in the HUD.
 */
typedef struct {
    int paused;         /* 'p' — freeze particle_tick. Rendering    *
                         * continues so the user can study a frame. *
                         * HUD displays "PAUSED " in this mode.     */
} NetUI;

static void arch_reset(NetArch *a)
{
    a->n_layers    = DEFAULT_LAYERS;
    a->n_per_layer = DEFAULT_NEURONS;
}

static void render_reset(RenderConfig *r)
{
    r->thickness = DEFAULT_THICKNESS;
    r->theme     = 0;
}

static void ui_reset(NetUI *u)
{
    u->paused = 0;
}

/* §5.4 derived-from-NetArch accessors.
 *
 * Total neuron count and fully-connected edge count are not stored —
 * they're cheap to compute from (n_layers, n_per_layer). Naming them
 * here so the HUD doesn't repeat the arithmetic inline.
 *
 *   total_neurons = L · N
 *   total_edges   = (L - 1) · N²       (fully connected, every pair) */
static inline int net_total_neurons(const NetArch *a)
{
    return a->n_layers * a->n_per_layer;
}

static inline int net_total_edges(const NetArch *a)
{
    return (a->n_layers - 1) * a->n_per_layer * a->n_per_layer;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  particle                                                            */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Particle — a single bright dot in flight along ONE edge between
 * two adjacent layers.
 *
 * INTENT
 *   The particle is the FLOW indicator — it visualises "data moving
 *   forward through the network" without any actual activation math.
 *   The trajectory is: launch at an input neuron, hop one edge per
 *   "lifetime" until the output layer, then loop back to a random
 *   input. Every particle traces an endless input → output → input
 *   → output cycle.
 *
 * STATE PER PARTICLE
 *   At any instant a particle is travelling ALONG ONE EDGE — from
 *   neuron (from_layer, from_idx) to neuron (from_layer + 1, to_idx),
 *   at progress fraction t ∈ [0, 1].
 *
 *   Each tick:  t += speed · dt
 *   When t ≥ 1: hop forward (from_layer + 1, to_idx becomes new
 *               from_layer, from_idx; pick fresh random to_idx).
 *   If next layer would be past output:  loop to a random input.
 *
 * WHY PER-PARTICLE SPEED JITTER
 *   Without jitter, all particles released together would stay in
 *   lock-step forever — a single moving wavefront crossing layer by
 *   layer. With ±PARTICLE_JITTER speed variation, the dots desync
 *   within 2–3 hops and the visual reads as flow, not march.
 *
 * WHY t REMAINDER IS CARRIED (not reset to 0 on hop)
 *   particle_tick uses  while (t >= 1.0f) { t -= 1.0f; ... }  so a
 *   large dt that crosses TWO edges in one tick still advances the
 *   particle correctly. Without the carried remainder, the dt cap
 *   would have to be much tighter.
 *
 * ALGORITHM REFERENCE
 *   The straight-line linear interpolation between source and target
 *   centres is the standard parametric line equation. Random target
 *   selection in particle_tick mimics a "uniform-random successor"
 *   model — Schmidhuber (2015) calls this a "non-deterministic flow"
 *   pattern when used in actual NN training (this file's version is
 *   purely visual, no training).
 */
typedef struct {
    int   from_layer;   /* source neuron's layer index.               *
                         * Range [0, n_layers - 2] (never the last    *
                         * layer — would have no forward edge).       */
    int   from_idx;     /* source neuron's position in that layer.   *
                         * Range [0, n_per_layer - 1].                */
    int   to_idx;       /* target neuron's position in the NEXT      *
                         * layer (from_layer + 1).                    *
                         * Range [0, n_per_layer - 1].                */
    float t;            /* progress fraction along current edge.     *
                         * 0 = at source, 1 = at target.              *
                         * Advances by speed·dt each tick.            */
    float speed;        /* edges-per-second for this particular     *
                         * particle. Set at reset as                  *
                         *   PARTICLE_SPEED · (1 ± PARTICLE_JITTER)  *
                         * so dots desync within 2–3 hops.            */
} Particle;

/*
 * ParticlePool — fixed-capacity array of particles + active count.
 *
 * INTENT
 *   One particle per INPUT neuron — count = n_per_layer. Every input
 *   neuron continuously launches a dot that takes a meandering path
 *   to some output neuron, then loops back. The full network is
 *   therefore always "lit" by at least one particle per input
 *   column at all times.
 *
 * WHY FIXED CAPACITY (not dynamic alloc)
 *   MAX_PARTICLES = MAX_NEURONS = 16 — the pool never needs more
 *   slots than the largest possible input layer width. Statically
 *   allocating the upper bound means no malloc in the hot path.
 *
 * RESET TRIGGERS
 *   particle_reset() rebuilds the pool whenever the network SHAPE
 *   changes — '[' / ']' (layer count), '-' / '+' (neurons per
 *   layer), or 'r' (full scene reset). Each rebuild re-seeds every
 *   particle at t = 0, jittered speed, and a random initial target.
 */
typedef struct {
    Particle items[MAX_PARTICLES];  /* fixed slack of MAX_NEURONS = 16.*
                                     * Only items[0..count) are live.  */
    int      count;                  /* active particle count this gen *
                                     * = current NetArch.n_per_layer.  */
} ParticlePool;

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * particle_reset — one particle per input neuron, all starting at t=0
 * (right at the input neuron). Per-particle speed jitter desyncs them
 * within 2–3 edges so they don't stay in lock-step, while the t=0 start
 * makes the moment of reset visibly obvious — every particle snaps back
 * onto its input neuron and the fan-forward begins fresh.
 *
 * Called once at startup and again whenever the network shape changes
 * (layer count, neurons-per-layer) so the pool size and indices match
 * the new geometry.
 */
static void particle_reset(ParticlePool *p, const NetArch *a)
{
    p->count = a->n_per_layer;
    for (int i = 0; i < p->count; i++) {
        Particle *q  = &p->items[i];
        q->from_layer = 0;
        q->from_idx   = i;
        q->to_idx     = (int)(frand() * a->n_per_layer);
        q->t          = 0.0f;
        q->speed      = PARTICLE_SPEED *
                        (1.0f - PARTICLE_JITTER + 2.0f * PARTICLE_JITTER * frand());
    }
}

/* Uniform random neuron index in [0, n_per_layer). Wraps the rand-
 * scaling cast so the random-target idea reads as one named operation. */
static inline int random_target_in_layer(const NetArch *a)
{
    return (int)(frand() * a->n_per_layer);
}

/* True if this particle just arrived at the output layer (its current
 * source is the LAST forward-edge layer). Naming the condition makes
 * the "loop back to input" branch read as state-machine, not
 * arithmetic. */
static inline bool particle_at_output_layer(const Particle *q, const NetArch *a)
{
    return q->from_layer == a->n_layers - LAYER_EDGE_TAIL;
}

/* Advance one particle by ONE edge: either hop forward to the next
 * layer (carrying to_idx as the new from_idx), or — if already at
 * the output layer — loop back to a random input neuron. */
static void particle_hop_or_loop(Particle *q, const NetArch *a)
{
    if (particle_at_output_layer(q, a)) {
        /* output reached — re-seed at a random input neuron */
        q->from_layer = 0;
        q->from_idx   = random_target_in_layer(a);
    } else {
        /* forward hop — current target becomes the next source */
        q->from_layer += 1;
        q->from_idx    = q->to_idx;
    }
    q->to_idx = random_target_in_layer(a);
}

/*
 * particle_tick — advance every particle by speed·dt along its
 * current edge. While t ≥ 1, consume one edge's worth of progress
 * (particle_hop_or_loop). The while loop handles the rare large-dt
 * case where a particle crosses two edges in a single tick.
 */
static void particle_tick(ParticlePool *p, const NetArch *a, float dt)
{
    for (int i = 0; i < p->count; i++) {
        Particle *q = &p->items[i];
        q->t += q->speed * dt;
        while (q->t >= 1.0f) {
            q->t -= 1.0f;
            particle_hop_or_loop(q, a);
        }
    }
}

/* Linearly interpolate a particle's position along its current edge.
 * Cell-space coords:  sr = ar + (br - ar)·t ,  sc = ac + (bc - ac)·t.
 * Pulled out so the per-particle paint loop reads as "compute end-
 * points → lerp → paint" rather than four arithmetic lines.        */
static void particle_interp_cell(const Particle *q, const NetArch *a,
                                 int rows, int cols,
                                 int *out_sr, int *out_sc)
{
    int ar, ac, br, bc;
    neuron_cell(q->from_layer,     q->from_idx,
                a->n_layers, a->n_per_layer, rows, cols, &ar, &ac);
    neuron_cell(q->from_layer + 1, q->to_idx,
                a->n_layers, a->n_per_layer, rows, cols, &br, &bc);
    *out_sr = ar + (int)((br - ar) * q->t);
    *out_sc = ac + (int)((bc - ac) * q->t);
}

/* particle_draw — paint a bright '*' at each particle's interpolated
 * cell. Drawn between connections and neurons in scene_draw so the
 * neuron sigil cleanly caps the particle's trajectory at the endpoints. */
static void particle_draw(const ParticlePool *p, const NetArch *a,
                          int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sr, sc;
        particle_interp_cell(&p->items[i], a, rows, cols, &sr, &sc);
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddch(sr, sc, (chtype)PARTICLE_GLYPH);
    }
    attroff(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Pick the line glyph from the LOCAL per-cell step direction.
 *
 *   dsr == 0           → horizontal step → hor
 *   dsc == 0           → vertical step   → ver
 *   sign(dsr) == sign(dsc) → down-diagonal → dn
 *   otherwise          → up-diagonal   → up
 *
 * Choosing the glyph per cell (not from the global Δr/Δc slope)
 * gives shallow lines a long run of '-' with occasional '\' or '/'
 * at row transitions — visually faithful to the actual geometry.
 */
static const char *pick_line_glyph(int dsr, int dsc, const LineStyle *s)
{
    if (dsr == 0)                    return s->hor;
    if (dsc == 0)                    return s->ver;
    if ((dsr > 0) == (dsc > 0))      return s->dn;
    return s->up;
}

/*
 * draw_line — linear-step rasterise the segment (r0,c0)→(r1,c1).
 *
 *   steps = max(|Δr|, |Δc|)              one step per cell along
 *                                        the major axis
 *   sr_i  = r0 + Δr · i / steps           interpolated row
 *   sc_i  = c0 + Δc · i / steps           interpolated col
 *
 * Endpoints (i = 0 and i = steps) are skipped so the neuron sigil
 * (drawn afterwards) lands on a clean background — see §7 painter's
 * order in scene_draw.
 */
static void draw_line(int r0, int c0, int r1, int c1,
                      int rows, int cols, const LineStyle *s)
{
    int dr    = r1 - r0, dc = c1 - c0;
    int steps = (abs(dr) > abs(dc)) ? abs(dr) : abs(dc);
    if (steps <= 1) return;

    int prev_sr = r0, prev_sc = c0;
    for (int i = 1; i < steps; i++) {
        int sr  = r0 + dr * i / steps;
        int sc  = c0 + dc * i / steps;
        int dsr = sr - prev_sr;
        int dsc = sc - prev_sc;

        const char *glyph = pick_line_glyph(dsr, dsc, s);
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddstr(sr, sc, glyph);

        prev_sr = sr;
        prev_sc = sc;
    }
}

/*
 * draw_connections — every neuron in layer i to every neuron in layer i+1.
 * O(layers · neurons²) — cheap at our caps.
 */
static void draw_connections(const NetArch *arch, const RenderConfig *rc,
                             int rows, int cols)
{
    const LineStyle *s = &LINE_THICKNESS[rc->thickness];
    attron(COLOR_PAIR(PAIR_CONN) | s->attr);
    for (int i = 0; i < arch->n_layers - 1; i++) {
        for (int a = 0; a < arch->n_per_layer; a++) {
            int ar, ac;
            neuron_cell(i, a, arch->n_layers, arch->n_per_layer,
                        rows, cols, &ar, &ac);
            for (int b = 0; b < arch->n_per_layer; b++) {
                int br, bc;
                neuron_cell(i + 1, b, arch->n_layers, arch->n_per_layer,
                            rows, cols, &br, &bc);
                draw_line(ar, ac, br, bc, rows, cols, s);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_CONN) | s->attr);
}

/* Stamp the 3-cell '(O)' neuron sigil centred at (sr, sc). The parens
 * give the neuron a clear visual border that reads as a circle even
 * in fonts where 'O' alone looks like a regular letter. Each cell is
 * bounds-checked independently so a neuron near the screen edge
 * draws its valid cells and silently drops the off-screen ones. */
static void paint_neuron_sigil(int sr, int sc, int cols)
{
    if (sc - NEURON_SIGIL_HALF >= 0    ) mvaddch(sr, sc - NEURON_SIGIL_HALF, NEURON_GLYPH_LEFT);
    if (sc                     <  cols ) mvaddch(sr, sc,                     NEURON_GLYPH_CENTRE);
    if (sc + NEURON_SIGIL_HALF <  cols ) mvaddch(sr, sc + NEURON_SIGIL_HALF, NEURON_GLYPH_RIGHT);
}

/*
 * draw_neurons — paint every neuron in the network as a bright
 * '(O)' sigil at the cell returned by neuron_cell(). Outer loop:
 * layers (left → right); inner loop: neurons within layer (top →
 * bottom). Rows past rows-1 are clipped so the hint strip stays
 * unobstructed.
 */
static void draw_neurons(const NetArch *arch, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
    for (int i = 0; i < arch->n_layers; i++) {
        for (int j = 0; j < arch->n_per_layer; j++) {
            int sr, sc;
            neuron_cell(i, j, arch->n_layers, arch->n_per_layer,
                        rows, cols, &sr, &sc);
            if (sr < 0 || sr >= rows - 1) continue;
            paint_neuron_sigil(sr, sc, cols);
        }
    }
    attroff(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
}

/*
 * §7.1 Screen — current ncurses dimensions.
 *
 * INTENT
 *   Single point of truth for "how big is the terminal RIGHT NOW".
 *   neuron_cell() derives every neuron position from these; the
 *   renderers use them to clip each draw; the HUD reads cols to
 *   right-align the status string. Modified only at startup and on
 *   SIGWINCH — never mid-frame.
 *
 * RESIZE BEHAVIOUR
 *   Because neuron positions are NOT stored anywhere (computed on
 *   demand from (layer, idx, rows, cols)), a resize just needs to
 *   update these two ints. The next frame reads the new dimensions
 *   and lays out the whole network at the new geometry — no
 *   per-neuron-array rebuild needed.
 */
typedef struct {
    int rows;   /* terminal height in character cells.                 */
    int cols;   /* terminal width  in character cells.                 */
} Screen;

/*
 * §7.2 FrameTimer — wall-clock dt + EWMA fps + frame budget.
 *
 * INTENT
 *   Pulls the four timing concerns that used to be loose locals in
 *   main() into one named struct:
 *     (1) frame_ns    — target sleep budget per frame
 *     (2) t_tick_prev — measure dt between particle_tick calls
 *     (3) t_fps_prev  — measure dt between fps updates
 *     (4) fps         — EWMA-smoothed reading for the HUD
 *
 * EWMA SMOOTHING (the one bit of math)
 *   fps_new  =  0.95 · fps_old  +  0.05 · instantaneous
 *
 *   One-pole IIR low-pass filter. The 0.95 / 0.05 split gives a ~20-
 *   frame effective window — fast enough to track real fps changes,
 *   slow enough that the HUD's digits don't jitter every frame.
 *
 * WHY DT_CAP_S = 0.10
 *   Spiral-of-death prevention: if the process is suspended (e.g.
 *   Ctrl-Z, host sleep), `now - t_tick_prev` can jump by seconds.
 *   Without the cap, particle_tick would try to advance one particle
 *   across dozens of edges in a single call (handled by the while
 *   loop, but burns CPU pointlessly). Capping dt at 0.1 s means we
 *   skip lost time gracefully.
 *
 * ALGORITHM REFERENCE
 *   Glenn Fiedler, "Fix Your Timestep!" — the dt-cap pattern in
 *   main() is the simple-mode variant of his full sub-step recipe.
 *   No sub-stepping needed here because particle_tick is per-tick
 *   integration with the carried-remainder t (see Particle docs).
 */
typedef struct {
    int64_t frame_ns;       /* one frame budget in ns.                  *
                             * = NS_PER_SEC / TARGET_FPS = 33.3 ms.     *
                             * Used by clock_sleep_ns at frame end.     */
    int64_t t_tick_prev;    /* clock_ns() at last particle_tick. dt    *
                             * for the tick is (now - t_tick_prev),    *
                             * capped at DT_CAP_S.                      */
    int64_t t_fps_prev;     /* clock_ns() at last fps update. The      *
                             * EWMA reads (now - t_fps_prev) as its     *
                             * instantaneous sample.                    */
    double  fps;            /* EWMA-smoothed frames per second. Seeded *
                             * to TARGET_FPS at boot; shown in the HUD.*/
} FrameTimer;

/*
 * §7.3 Scene — the top-level container.
 *
 * INTENT
 *   One single struct owns every long-lived piece of state. Replaces
 *   what used to be loose Net + ParticlePool locals in main() plus
 *   two separate file-scope g_running / g_need_resize flags. A
 *   single file-static g_scene instance gives signal handlers a
 *   path to the volatile flags without API ceremony.
 *
 * SUB-STRUCT LAYERING (read these in order)
 *   1. NetArch       — what shape is the network?
 *   2. RenderConfig  — how do we paint it?
 *   3. NetUI         — what has the human toggled?
 *   4. ParticlePool  — who's flowing along the edges right now?
 *   5. Screen        — how big is the terminal?
 *   6. FrameTimer    — loop pacing
 *   + running, need_resize — signal-handler flags
 *
 * MUTATION CONTRACT (which keys / events touch which sub-struct)
 *
 *   NetArch         [ / ]      n_layers       → particle_reset
 *                   - / +      n_per_layer    → particle_reset
 *   RenderConfig    < / >      thickness      (no reset)
 *                   t          theme          + color_init
 *   NetUI           p          paused
 *   particles       per tick   t advances; on t≥1 hop forward
 *   r               full reset (arch+render+ui+particles+color)
 *   running         q / ESC, SIGINT, SIGTERM
 *   need_resize     SIGWINCH
 *
 *   NetArch writes are architectural — they invalidate the pool.
 *   RenderConfig writes are cosmetic — they never reset anything.
 *   The split is a TYPE-LEVEL guarantee, not just a comment.
 *
 * INITIALIZATION ORDER (see main())
 *   1. arch_reset / render_reset / ui_reset — defaults
 *   2. particle_reset                       — pool sized to arch
 *   3. screen_init + colour_init            — ncurses up
 *   4. screen.rows/cols                     — read LINES/COLS
 *   5. timer.frame_ns, fps, t_*             — seed FrameTimer
 */
typedef struct {
    NetArch               arch;        /* network shape (rebuild trigger) */
    RenderConfig          render;      /* cosmetic flags                  */
    NetUI                 ui;          /* keyboard-toggled flags          */
    ParticlePool          particles;   /* flowing dots, one per input     */
    Screen                screen;      /* ncurses dimensions              */
    FrameTimer            timer;       /* loop pacing                     */
    volatile sig_atomic_t running;     /* cleared by 'q'/ESC/SIGINT       */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH                 */
} Scene;

/* §7.4 HUD bars — canonical CLAUDE.md two-bar layout.
 *
 *   row 0  right     bright yellow + A_BOLD   live network parameters + fps
 *   row -1 left      bright cyan   + A_BOLD   all interactive keys
 *
 * Both pairs bind on the default terminal background and use A_BOLD so
 * they stay legible against any colour/glyph the network draws behind
 * them — never A_DIM.
 */

/* Build the top-row status string. Order chosen so the eye lands on
 * the architecture parameters first (the user's most-frequent
 * adjustments), then style, then fps + run state. */
static void hud_format_status(const Scene *scene, char *buf, size_t n)
{
    const NetArch      *arch = &scene->arch;
    const RenderConfig *rc   = &scene->render;

    snprintf(buf, n,
             " layers:%d  neurons:%d  total:%d  edges:%d  thick:%s  "
             "theme:%d  %5.1f fps  %s ",
             arch->n_layers, arch->n_per_layer,
             net_total_neurons(arch), net_total_edges(arch),
             LINE_THICKNESS[rc->thickness].name, rc->theme,
             scene->timer.fps,
             scene->ui.paused ? "PAUSED " : "running");
}

/* Paint the HUD status string right-justified on row 0. */
static void hud_paint_status(const char *buf, int cols)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Paint the hint strip — lists every interactive key. */
static void hud_paint_hint(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  r:reset  t:theme  "
             "[/]:layers  -/+:neurons  </>:thick ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * scene_draw — one frame, painter's order:
 *   1. erase
 *   2. fully-connected edges (PAIR_CONN, thickness-driven glyphs)
 *   3. travelling particles (between connections and neurons so
 *      neurons cap their trajectory cleanly)
 *   4. neuron sigils (bright '(O)')
 *   5. HUD status row 0 + hint strip row rows-1
 */
static void scene_draw(const Scene *scene)
{
    const NetArch      *arch = &scene->arch;
    const RenderConfig *rc   = &scene->render;
    int rows = scene->screen.rows, cols = scene->screen.cols;

    erase();
    draw_connections(arch, rc, rows, cols);
    particle_draw(&scene->particles, arch, rows, cols);
    draw_neurons(arch, rows, cols);

    char buf[160];
    hud_format_status(scene, buf, sizeof buf);
    hud_paint_status (buf, cols);
    hud_paint_hint   (rows);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    /* Activate the user's locale so mvaddstr renders multi-byte UTF-8
     * line characters correctly (used by the heavy thickness level). */
    setlocale(LC_ALL, "");
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

/* The single Scene instance. File-scope so signal handlers can flip
 * the volatile sig_atomic_t flags without ceremony. */
static Scene g_scene;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_scene.running     = 0;
    if (s == SIGWINCH)               g_scene.need_resize = 1;
}

/* scene_reset — full reset to boot defaults. Used by 'r' key.
 *
 * Bundles all five reset concerns into one named operation:
 *   - architecture (n_layers, n_per_layer back to defaults)
 *   - render config (thickness, theme back to defaults)
 *   - UI flags (paused cleared)
 *   - particle pool (reseeded to the new architecture)
 *   - colour pairs (rebound for the reset theme)
 *
 * Keeping this as one function documents at the TYPE LEVEL that 'r'
 * means "factory-reset the whole scene" rather than "reset these
 * three things and remember to also re-init colour". */
static void scene_reset(Scene *s)
{
    arch_reset    (&s->arch);
    render_reset  (&s->render);
    ui_reset      (&s->ui);
    particle_reset(&s->particles, &s->arch);
    color_init    (s->render.theme);
}

/* ── §9.1 setup + per-step main-loop helpers ───────────────────────── */

/* One-shot scene setup before the loop: signals, defaults, ncurses,
 * particle pool, FrameTimer seed. */
static void scene_setup(void)
{
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    /* (1) seed defaults */
    g_scene.running = 1;
    arch_reset    (&g_scene.arch);
    render_reset  (&g_scene.render);
    ui_reset      (&g_scene.ui);
    particle_reset(&g_scene.particles, &g_scene.arch);

    /* (2) ncurses up; learn rows/cols */
    screen_init(g_scene.render.theme);
    g_scene.screen.rows = LINES;
    g_scene.screen.cols = COLS;

    /* (3) seed FrameTimer */
    g_scene.timer.frame_ns    = NS_PER_SEC / TARGET_FPS;
    g_scene.timer.fps         = TARGET_FPS;
    g_scene.timer.t_fps_prev  = clock_ns();
    g_scene.timer.t_tick_prev = g_scene.timer.t_fps_prev;
}

/* SIGWINCH handler — re-sync ncurses + refresh screen dims. The
 * particle pool stays intact (neuron positions are computed on
 * demand from the new dims). */
static void scene_handle_resize(void)
{
    g_scene.need_resize = 0;
    endwin();
    refresh();
    g_scene.screen.rows = LINES;
    g_scene.screen.cols = COLS;
}

/* Wall-clock dt since last particle_tick, capped at DT_CAP_S so a
 * stalled process can't dump a giant catch-up burst into the pool. */
static float frame_measure_dt(int64_t now)
{
    float dt = (float)(now - g_scene.timer.t_tick_prev) / (float)NS_PER_SEC;
    if (dt > DT_CAP_S) dt = DT_CAP_S;
    g_scene.timer.t_tick_prev = now;
    return dt;
}

/* EWMA one-pole IIR update of fps. +1 in the denominator defends
 * against a zero-dt frame. */
static void frame_update_ewma_fps(int64_t now)
{
    int64_t dt_ns   = now - g_scene.timer.t_fps_prev + 1;
    double  instant = (double)NS_PER_SEC / (double)dt_ns;
    g_scene.timer.fps = g_scene.timer.fps * EWMA_RETAIN
                      + instant            * EWMA_NEW;
    g_scene.timer.t_fps_prev = now;
}

/* Sleep the remainder of frame_ns so we hit TARGET_FPS. */
static void frame_cap_to_target_fps(int64_t frame_start)
{
    clock_sleep_ns(g_scene.timer.frame_ns - (clock_ns() - frame_start));
}

/* ── §9.2 keyboard action handlers ─────────────────────────────────── */

/* 'p' — pause / resume particle ticks. Rendering keeps running. */
static void key_pause_toggle(void) { g_scene.ui.paused ^= 1; }

/* 't' — cycle theme and rebind PAIR_NEURON / PAIR_CONN / PAIR_PARTICLE. */
static void key_cycle_theme(void)
{
    g_scene.render.theme = (g_scene.render.theme + 1) % N_THEMES;
    color_init(g_scene.render.theme);
}

/* '[' / ']' — nudge layer count, clamped to [MIN_LAYERS, MAX_LAYERS].
 * Any architectural change triggers particle_reset (pool size depends
 * on layer count via from_layer ranges). */
static void key_change_layers(int delta)
{
    int n = g_scene.arch.n_layers + delta;
    if (n < MIN_LAYERS || n > MAX_LAYERS) return;
    g_scene.arch.n_layers = n;
    particle_reset(&g_scene.particles, &g_scene.arch);
}

/* '-' / '+' — nudge neurons per layer, clamped to [MIN, MAX].
 * Triggers particle_reset (pool size = n_per_layer). */
static void key_change_neurons(int delta)
{
    int n = g_scene.arch.n_per_layer + delta;
    if (n < MIN_NEURONS || n > MAX_NEURONS) return;
    g_scene.arch.n_per_layer = n;
    particle_reset(&g_scene.particles, &g_scene.arch);
}

/* '<' / '>' — nudge line thickness, clamped to [MIN, MAX]. No reset
 * — purely cosmetic, lives in RenderConfig. */
static void key_change_thickness(int delta)
{
    int t = g_scene.render.thickness + delta;
    if (t < MIN_THICKNESS || t > MAX_THICKNESS) return;
    g_scene.render.thickness = t;
}

/* Dispatch one keystroke to its named action. Each case is one helper
 * call; the switch reads as the keymap. */
static void scene_handle_one_keystroke(int ch)
{
    switch (ch) {
    case 'q': case 27 /* ESC */:  g_scene.running = 0;           break;
    case 'p':                     key_pause_toggle();            break;
    case 'r':                     scene_reset(&g_scene);         break;
    case 't':                     key_cycle_theme();             break;
    case '[':                     key_change_layers(-1);         break;
    case ']':                     key_change_layers(+1);         break;
    case '-':                     key_change_neurons(-1);        break;
    case '+': case '=':           key_change_neurons(+1);        break;
    case ',': case '<':           key_change_thickness(-1);      break;
    case '.': case '>':           key_change_thickness(+1);      break;
    default:                                                     break;
    }
}

/* Drain all queued keystrokes through the action dispatcher. */
static void scene_drain_input(void)
{
    int ch;
    while ((ch = getch()) != ERR) scene_handle_one_keystroke(ch);
}

/* Advance the particle pool by one wall-clock dt (no-op if paused). */
static void scene_advance_particles(float dt)
{
    if (g_scene.ui.paused) return;
    particle_tick(&g_scene.particles, &g_scene.arch, dt);
}

/* ── §9.3 main ─────────────────────────────────────────────────────── *
 *
 * Reads as the program's lifecycle:
 *
 *   SETUP:
 *     scene_setup() — signals, defaults, ncurses, pool, FrameTimer.
 *
 *   LOOP (each frame):
 *     1. handle pending resize
 *     2. drain queued keystrokes
 *     3. measure dt (capped) → advance particles
 *     4. update EWMA fps
 *     5. draw the frame
 *     6. sleep to hit TARGET_FPS
 *
 * ─────────────────────────────────────────────────────────────────── */
int main(void)
{
    scene_setup();

    while (g_scene.running) {
        /* (1) deferred resize */
        if (g_scene.need_resize) scene_handle_resize();

        /* (2) drain queued keystrokes */
        scene_drain_input();

        /* (3) advance particles by wall-clock dt */
        int64_t now = clock_ns();
        float   dt  = frame_measure_dt(now);
        scene_advance_particles(dt);

        /* (4) update EWMA fps for the HUD */
        frame_update_ewma_fps(now);

        /* (5) draw the frame */
        scene_draw(&g_scene);

        /* (6) sleep to target frame rate */
        frame_cap_to_target_fps(now);
    }
    return 0;
}
