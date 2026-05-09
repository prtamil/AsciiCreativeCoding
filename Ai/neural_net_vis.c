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
 *   §5 net       — Net struct (sizes, thickness, theme)
 *   §6 particle  — Particle pool: reset / tick / draw + edge-by-edge hop
 *   §7 scene     — draw_connections + particle_draw + draw_neurons + HUD
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
 * Data-structure: int n_layers, int n_per_layer in a Net struct. Neuron
 *                 positions are NOT stored — they're computed on demand
 *                 from (layer, idx, rows, cols). Resize is therefore
 *                 free: the next frame reads the new dimensions and
 *                 places neurons in the right place automatically.
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
 * References    :
 *   Goodfellow, Bengio & Courville, "Deep Learning" (2016) ch. 6 —
 *     standard feed-forward network diagram notation.
 *   Network architecture diagrams as visual notation —
 *     Schmidhuber, "Deep Learning in Neural Networks: An Overview"
 *     *Neural Networks* 61 (2015).
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
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of a city subway map drawn as parallel station rows.  Every
 * station in row i has a track to every station in row i+1 — a
 * complete bipartite graph between adjacent rows.  Now imagine a
 * commuter at each row-0 station who, at every junction, picks a
 * random track to the next row.  Resize the city (terminal) and the
 * stations rearrange to keep the layout balanced; change the number
 * of rows or stations-per-row and the commuter pool resizes too.
 * The single source of truth is neuron_cell(layer, idx) — change
 * that formula and every line, every neuron, every particle moves
 * accordingly.
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
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • neuron_cell uses INTEGER division — at small terminal sizes,
 *    several neurons can collapse onto the same row.  Visually
 *    they look like one dot; physics still treats them as separate.
 *  • The particle's t can exceed 1.0 in a single tick if dt is huge
 *    (e.g. paused 5 s then unpaused).  The DT_CAP_S = 0.10 clamp in
 *    main and the while loop in particle_tick together prevent both
 *    spiral-of-death and "particle teleports".
 *  • Resize re-reads LINES/COLS but does NOT call particle_reset —
 *    in-flight particles continue with their old indices, which are
 *    still valid since n_layers and n_per_layer are unchanged.
 *  • Changing n_layers or n_per_layer DOES call particle_reset
 *    because old indices may now be out of range.
 *  • thickness=3 (heavy) uses UTF-8 (═ ║ ╲ ╱); requires a UTF-8
 *    locale — setlocale(LC_ALL, "") in screen_init covers this.
 *    Levels 0-2 are pure ASCII.
 *  • particle_draw clamps to rows-1 (leaves bottom row for hint),
 *    cols-1 (no right margin).  A particle drawn at the boundary
 *    is silently skipped — visible as a tiny gap before the next
 *    neuron.
 *  • PAIR_HUD uses background COLOR_CYAN — on terminals that don't
 *    honour PAIR_HUD bg, the HUD becomes faded yellow on default.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Default 3 layers × 5 neurons → 15 neurons, 2·25 = 50 connection
 *    lines, 5 particles.  Press '+' until 16 neurons → 3·16² = 768
 *    line draws per frame.  Should still hit 30 fps easily.
 *  • Press '[' down to 2 layers — connections cut to one layer pair
 *    (n²); particles never advance past the output, just bounce
 *    back to input each hop.
 *  • Pause (p): particles freeze mid-edge; resume — they continue
 *    from same t, no jump.
 *  • Reset (r): every particle snaps back to t=0 at its input
 *    neuron (visible as a momentary alignment at column 0).
 *  • Theme cycle (t): all four colour pairs change in unison.
 *  • Thickness '<' / '>': line glyphs change between '.', '-|\/'
 *    (thin/bold), and Unicode '═║╲╱' (heavy).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose.  This is a NEURAL-NETWORK STRUCTURE VISUALIZER, not
 *      an actual NN trainer (no weights, no activation function, no
 *      backpropagation).  It's the architecture diagram brought to
 *      life with animated particles to suggest data flow.  Read
 *      Ai/genetic_rocket.c first if you want a real learning
 *      algorithm; this file is purely geometric.
 *   2. §4 layout — neuron_cell() is the SINGLE SOURCE OF TRUTH for
 *      every neuron's position.  All drawing reads from this.
 *      Read AFTER tutorials T1-T3 below.
 *   3. §6 particle — particle pool, advance-edge logic.
 *      Read AFTER tutorial T5.
 *   4. §7 scene — orchestrator: connections → particles → neurons
 *      (painter's order: dim background → moving foreground →
 *      bright neurons last).
 *   5. §1-§3, §5, §8-§9 — config / clock / colour / Net struct /
 *      screen / app loop.  Skim if seen.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   n_layers          number of layers (2..12).
 *   n_per_layer       neurons per layer (2..16).
 *   net.thickness     line-glyph style (0=dot, 1=thin, 2=bold,
 *                     3=heavy unicode).
 *   neuron_cell(L, I) returns (col, row) for neuron (L, I).
 *   Particle.t        progress along current edge ∈ [0, 1].
 *   from_layer        layer index of particle's source neuron.
 *   from_idx          neuron index in from_layer.
 *   to_idx            neuron index in (from_layer + 1) — the random
 *                     target this particle is currently heading to.
 *   speed             per-particle edge-traversal speed (jittered).
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
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build a feed-forward NN visualizer from
 * first principles.
 *
 *   T1  What a feed-forward neural network LOOKS like
 *   T2  Geometry-only — why this file omits weights and math
 *   T3  Coordinate-on-demand — neuron_cell() as single source of truth
 *   T4  Drawing N² connections — line glyph picked per-cell
 *   T5  Particles as data flow — the visual abstraction
 *   T6  From visualisation to real NN — what's missing here
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT A FEED-FORWARD NEURAL NETWORK LOOKS LIKE
 * ─────────────────────────────────────────────────
 * The standard textbook neural-network diagram has THREE features:
 *
 *   1. LAYERS — vertical columns of NEURONS (circles, "(O)" here).
 *      Conventionally drawn left-to-right: input layer leftmost,
 *      output layer rightmost, "hidden" layers between.
 *
 *   2. FULL CONNECTIVITY between adjacent layers — every neuron in
 *      layer L connects to every neuron in layer L+1.  N neurons
 *      per layer × N neurons in next layer = N² edges per layer
 *      pair.  L-1 layer pairs total.
 *
 *   3. NO connections WITHIN a layer or BACKWARD between layers
 *      (that's what "feed-forward" means).
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   layer 0    layer 1    layer 2    layer 3       │
 *      │   (input)   (hidden)   (hidden)   (output)       │
 *      │                                                  │
 *      │   (O)─────╲─(O)──╳───(O)─────────(O)             │
 *      │      ╳     ╳        ╳                            │
 *      │   (O)─────╳─(O)──╲───(O)─────────(O)             │
 *      │      ╲   ╱       ╳                               │
 *      │   (O)──── (O)────── (O)                          │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * That's the ENTIRE structural picture.  The math (weights,
 * biases, activation) lives ON TOP of this geometry but DOESN'T
 * CHANGE the geometry.  This file renders just the geometry —
 * layers + full connectivity + neurons.
 *
 * T2  GEOMETRY-ONLY — WHY THIS FILE OMITS WEIGHTS AND MATH
 * ────────────────────────────────────────────────────────
 * A real neural network forward pass:
 *
 *     for each layer L from 1 to N:
 *       for each neuron j in layer L:
 *         z_j = sum over neurons i in layer (L-1) of:
 *                 weight[L][i][j] · activation[L-1][i]
 *         activation[L][j] = sigmoid(z_j + bias[L][j])
 *
 * That requires weights (a 3D float array), biases, and an
 * activation function.  Plus TRAINING needs backpropagation,
 * loss functions, gradients.  Hundreds of lines just for the
 * math.
 *
 * This file SKIPS all of that.  It just shows the STRUCTURE.
 * Why?
 *
 *   - PEDAGOGICAL: many learners' first confusion is "what
 *     does the network LOOK like?"  The textbook diagram answer
 *     is enough to ground further reading.
 *
 *   - SCALABLE: with no weights to maintain, we can resize the
 *     network on a key press (more layers, more neurons) without
 *     blowing up memory or breaking pre-trained state.
 *
 *   - REVEALING: by showing the structure WITHOUT the math, we
 *     emphasise that "neural network" is mostly a particular
 *     CONNECTIVITY PATTERN.  The math is the same linear algebra
 *     that fits any graph.
 *
 * For the actual learning algorithm in this project, see
 * Ai/genetic_rocket.c (genetic algorithm — different family, but
 * uses real fitness evaluation + breeding).
 *
 * T3  COORDINATE-ON-DEMAND — neuron_cell() AS SINGLE SOURCE OF TRUTH
 * ─────────────────────────────────────────────────────────────────
 * Naive approach: store neuron positions in an array
 * `neuron[L][I].col, .row`.  Recompute on resize.
 *
 * Better: COMPUTE THEM ON DEMAND from a pure function of (L, I,
 * cols, rows):
 *
 *     neuron_cell(L, I) returns (col, row):
 *       col = (L + 1) · cols / (n_layers + 1)
 *       row = (I + 1) · (rows - 1) / (n_per_layer + 1)
 *
 * The "(L + 1)" gives equal LEFT margin (no neuron at col 0)
 * AND equal right margin (when L = n_layers - 1, col is at the
 * rightmost slot, leaving cols/(n_layers+1) of the right margin
 * empty).  Same logic for rows.
 *
 * Properties of "compute on demand":
 *
 *   - RESIZE IS FREE.  Next frame reads new (cols, rows), every
 *     neuron lands in the right place.  No state to update, no
 *     bookkeeping.
 *
 *   - SHAPE CHANGE IS FREE.  Press [/] to change n_layers or
 *     -/+ to change n_per_layer; the geometry recomputes from
 *     fresh values.  Particles need a reset because their
 *     in-flight indices may go out of range, but that's a
 *     reseeded particle pool — no neuron-position
 *     reorganisation.
 *
 *   - SINGLE SOURCE OF TRUTH.  Every line, every dot, every
 *     particle position derives from neuron_cell().  Change
 *     this one formula and the whole diagram redraws.
 *
 * Same pattern as a vertex shader in a GPU pipeline: per-frame
 * derive position from per-vertex parameters.  Or as the polar→
 * cell mapping in artistic/hindu_mandalas.c — a single function
 * that maps abstract index → screen cell.
 *
 * T4  DRAWING N² CONNECTIONS — LINE GLYPH PICKED PER-CELL
 * ───────────────────────────────────────────────────────
 * Between layer L and layer L+1, every (a, b) pair gets a line
 * — that's n_per_layer² lines per layer pair.
 *
 * Each line goes from neuron_cell(L, a) to neuron_cell(L+1, b)
 * — different sources, different targets.  Slopes range from
 * near-horizontal (top-of-layer to top-of-next) to steep diagonal
 * (top of layer to bottom of next).
 *
 * Line drawing: linear-step interpolation from (r1, c1) to
 * (r2, c2):
 *
 *     dr = r2 - r1; dc = c2 - c1
 *     steps = max(|dr|, |dc|)
 *     for i in 1 .. steps-1:        ← skip endpoints
 *       sr = r1 + i·dr/steps
 *       sc = c1 + i·dc/steps
 *       paint (sr, sc, glyph_for_local_step)
 *
 * "Endpoints skipped" matters: the neurons render '(O)' on top of
 * the line endpoints; the line shouldn't paint the centre cell of
 * '(O)' or it'd look messy.
 *
 * GLYPH selection per cell: at each cell, look at the LOCAL step
 * direction (dsr, dsc) — what's the difference to the PREVIOUS
 * cell?  Pick the glyph that matches that local direction:
 *
 *     dsr = 0, dsc != 0   →   horizontal '-'
 *     dsr != 0, dsc = 0   →   vertical   '|'
 *     dsr > 0, dsc > 0    →   '\\' (down-right)
 *     dsr < 0, dsc > 0    →   '/'  (up-right)
 *
 * Picking PER CELL (rather than per LINE) gives smoother near-
 * horizontal lines.  A line that's mostly horizontal but slowly
 * descends would look like staircased '\\' if the glyph were
 * fixed for the whole line; per-cell picks gives `------\------`
 * which reads correctly as "almost horizontal."
 *
 * Cost: at max settings (12 layers × 16 neurons), 11 layer pairs
 * × 16² = 2816 lines per frame, ~10 cells each = 28K paints per
 * frame at 30 fps = under 1ms.
 *
 * T5  PARTICLES AS DATA FLOW — THE VISUAL ABSTRACTION
 * ───────────────────────────────────────────────────
 * A static network diagram says "here's the structure" but
 * doesn't suggest "data flows through it."  Adding moving
 * particles fixes that without adding any actual computation.
 *
 * Particle state per-particle:
 *
 *     struct {
 *       int   from_layer, from_idx;     where am I starting from?
 *       int   to_idx;                    which neuron in next layer
 *                                        am I heading to?
 *       float t;                         progress 0..1 along edge
 *       float speed;                     per-particle (with jitter)
 *     };
 *
 * Each tick:
 *
 *     t += speed · dt
 *     while t >= 1.0:
 *       t -= 1.0
 *       // arrived at (from_layer + 1, to_idx)
 *       from_layer += 1
 *       from_idx = to_idx
 *       if from_layer == n_layers - 1:
 *         // reached output, loop back
 *         from_layer = 0
 *         from_idx = random in [0, n_per_layer)
 *       to_idx = random in [0, n_per_layer)   ← new target
 *
 * Drawing: linear interpolate between source cell and target
 * cell using `t`, paint a '*' at the resulting cell.
 *
 * Why RANDOM target selection?  Because we have no weights to
 * weight the choice by.  In a real NN, the choice would be
 * influenced by the activation values — particles would prefer
 * the connection with the highest weight × incoming activation.
 * That's T6's discussion.
 *
 * The visual effect: particles continuously stream forward
 * through the network, hopping randomly between neurons,
 * suggesting "data flowing left-to-right through fully-
 * connected layers."  Reading the picture becomes immediate.
 *
 * T6  FROM VISUALISATION TO REAL NN — WHAT'S MISSING HERE
 * ───────────────────────────────────────────────────────
 * To turn this visualizer into an actual neural network
 * trainer, you'd add:
 *
 *   1. WEIGHTS.  A 3D float array weight[L][I][J] = strength
 *      of the connection from neuron I in layer L to neuron J
 *      in layer L+1.  Initialise randomly small (Xavier or He
 *      initialisation).  About L · N² floats — at 12 layers ×
 *      16 neurons that's ~3000 floats = trivial memory.
 *
 *   2. ACTIVATION.  Per-neuron float storing the current
 *      activation value (after the activation function).  About
 *      L · N floats.
 *
 *   3. FORWARD PASS.  For each layer L, compute every neuron's
 *      activation as activation_function(sum of weight ·
 *      previous-layer activation + bias).  Common activation
 *      functions: sigmoid, tanh, ReLU.
 *
 *   4. INPUT.  Some way to set the input layer's activations
 *      (image pixels, sensor readings, ...).
 *
 *   5. LOSS FUNCTION.  Compare output layer to expected target,
 *      compute a scalar error.
 *
 *   6. BACKPROPAGATION.  Compute the gradient of the loss with
 *      respect to every weight, propagating backward through
 *      the chain rule.
 *
 *   7. WEIGHT UPDATE.  weight -= learning_rate · gradient.
 *      Or use Adam, RMSProp, etc.
 *
 *   8. TRAINING DATA.  A dataset of (input, expected output)
 *      pairs to train against.
 *
 * Each step is a substantial body of code on its own.  This
 * file gives you the canvas; turning it into a trainer is a
 * project's worth of work.
 *
 * Decision tree for "should I extend this?":
 *
 *   want a TEACHING demo of NN structure?     → this file is enough
 *   want to TRAIN a simple network?           → write a separate
 *                                                trainer; reuse the
 *                                                visualiser as
 *                                                output if desired
 *   want to USE a network for classification? → use a real ML
 *                                                framework
 *   want a different LEARNING algorithm?      → see Ai/genetic_rocket.c
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
    init_pair(PAIR_HUD,      x256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,     x256 ? 75 : COLOR_CYAN,  -1);
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
 * Connection line style — glyph set plus an ncurses attribute, indexed
 * by thickness 0..3. Each entry maps the four step types (horizontal,
 * vertical, down-diagonal, up-diagonal) to glyph strings; multi-byte
 * UTF-8 is fine because draw_line uses mvaddstr (and screen_init calls
 * setlocale).
 *
 *   0 dot   — `.` everywhere, A_DIM            (faintest, exploratory)
 *   1 thin  — ASCII `- | \ /`                  (default)
 *   2 bold  — ASCII `- | \ /` + A_BOLD         (visible but slope-aware)
 *   3 heavy — Unicode `═ ║ ╲ ╱` + A_BOLD       (double-line horizontal/
 *                                               vertical + light Unicode
 *                                               diagonals — the visual
 *                                               jumps but the slope of
 *                                               every line stays legible)
 */
typedef struct {
    const char *hor, *ver, *dn, *up;
    attr_t      attr;
    const char *name;
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

typedef struct {
    int n_layers;       /* MIN_LAYERS .. MAX_LAYERS                */
    int n_per_layer;    /* uniform width for now (extends easily)  */
    int thickness;      /* MIN_THICKNESS .. MAX_THICKNESS          */
    int theme;
    int paused;
} Net;

static void net_reset(Net *n)
{
    n->n_layers    = DEFAULT_LAYERS;
    n->n_per_layer = DEFAULT_NEURONS;
    n->thickness   = DEFAULT_THICKNESS;
    n->theme       = 0;
    n->paused      = 0;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  particle                                                            */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * A particle is a single bright dot travelling along ONE edge between
 * two adjacent layers. When it reaches the target neuron (t ≥ 1) it
 * picks a random target in the next layer and continues. When the next
 * layer would be past the output layer, it loops back to a random input
 * neuron — every particle therefore traces an endless input → output →
 * input → output trajectory.
 *
 * Pool size is fixed at the input-layer width: one particle per input
 * neuron. `particle_reset()` rebuilds the pool whenever the network
 * shape changes (layer count or neurons-per-layer).
 *
 * Each particle has a per-particle speed jitter and an initial random
 * `t` so the dots never bunch up into a single moving wavefront.
 */
typedef struct {
    int   from_layer;     /* 0 .. n_layers - 2                    */
    int   from_idx;       /* 0 .. n_per_layer - 1                  */
    int   to_idx;         /* 0 .. n_per_layer - 1 (next layer)     */
    float t;              /* progress along current edge, 0..1     */
    float speed;          /* edges per second                       */
} Particle;

typedef struct {
    Particle items[MAX_PARTICLES];
    int      count;
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
static void particle_reset(ParticlePool *p, const Net *n)
{
    p->count = n->n_per_layer;
    for (int i = 0; i < p->count; i++) {
        Particle *q  = &p->items[i];
        q->from_layer = 0;
        q->from_idx   = i;
        q->to_idx     = (int)(frand() * n->n_per_layer);
        q->t          = 0.0f;
        q->speed      = PARTICLE_SPEED *
                        (1.0f - PARTICLE_JITTER + 2.0f * PARTICLE_JITTER * frand());
    }
}

/*
 * particle_tick — advance every particle by speed·dt along its current
 * edge. On reaching t ≥ 1: hop to the next layer (carrying to_idx as
 * the new from_idx) or loop back to the input layer when at the output.
 *
 * The `while` loop catches the rare case where dt is large enough that
 * a particle traverses more than one edge in a single tick.
 */
static void particle_tick(ParticlePool *p, const Net *n, float dt)
{
    for (int i = 0; i < p->count; i++) {
        Particle *q = &p->items[i];
        q->t += q->speed * dt;
        while (q->t >= 1.0f) {
            q->t -= 1.0f;
            if (q->from_layer == n->n_layers - 2) {
                /* arrived at output — loop back to a random input neuron */
                q->from_layer = 0;
                q->from_idx   = (int)(frand() * n->n_per_layer);
                q->to_idx     = (int)(frand() * n->n_per_layer);
            } else {
                q->from_layer += 1;
                q->from_idx    = q->to_idx;
                q->to_idx      = (int)(frand() * n->n_per_layer);
            }
        }
    }
}

/*
 * particle_draw — interpolate between source and target neuron centres
 * and paint a bright glyph at the particle's current cell.
 */
static void particle_draw(const ParticlePool *p, const Net *n, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        const Particle *q = &p->items[i];
        int ar, ac, br, bc;
        neuron_cell(q->from_layer,     q->from_idx,
                    n->n_layers, n->n_per_layer, rows, cols, &ar, &ac);
        neuron_cell(q->from_layer + 1, q->to_idx,
                    n->n_layers, n->n_per_layer, rows, cols, &br, &bc);
        int sr = ar + (int)((br - ar) * q->t);
        int sc = ac + (int)((bc - ac) * q->t);
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddch(sr, sc, (chtype)PARTICLE_GLYPH);
    }
    attroff(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * draw_line — step from (r0,c0) to (r1,c1), drawing one character per
 * interior cell. The character comes from the LineStyle indexed by the
 * net's current thickness, with the slope (hor/ver/dn/up) chosen PER
 * CELL from the local step (dsr, dsc) — not from the global (dr, dc) —
 * so shallow lines render as rows of `-` linked by occasional `\` or
 * `/` at row changes, instead of `\\` pairs that read as parallel lines.
 *
 * Endpoints (i=0 and i=steps) are skipped so the neuron glyph (drawn
 * afterwards) lands on a clean background.
 */
static void draw_line(int r0, int c0, int r1, int c1,
                      int rows, int cols, const LineStyle *s)
{
    int dr = r1 - r0, dc = c1 - c0;
    int steps = (abs(dr) > abs(dc)) ? abs(dr) : abs(dc);
    if (steps <= 1) return;
    int prev_sr = r0, prev_sc = c0;
    for (int i = 1; i < steps; i++) {
        int sr = r0 + dr * i / steps;
        int sc = c0 + dc * i / steps;
        int dsr = sr - prev_sr;
        int dsc = sc - prev_sc;
        const char *ch;
        if      (dsr == 0)                ch = s->hor;
        else if (dsc == 0)                ch = s->ver;
        else if ((dsr > 0) == (dsc > 0))  ch = s->dn;
        else                              ch = s->up;
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddstr(sr, sc, ch);
        prev_sr = sr;
        prev_sc = sc;
    }
}

/*
 * draw_connections — every neuron in layer i to every neuron in layer i+1.
 * O(layers · neurons²) — cheap at our caps.
 */
static void draw_connections(const Net *n, int rows, int cols)
{
    const LineStyle *s = &LINE_THICKNESS[n->thickness];
    attron(COLOR_PAIR(PAIR_CONN) | s->attr);
    for (int i = 0; i < n->n_layers - 1; i++) {
        for (int a = 0; a < n->n_per_layer; a++) {
            int ar, ac;
            neuron_cell(i, a, n->n_layers, n->n_per_layer, rows, cols, &ar, &ac);
            for (int b = 0; b < n->n_per_layer; b++) {
                int br, bc;
                neuron_cell(i + 1, b, n->n_layers, n->n_per_layer, rows, cols, &br, &bc);
                draw_line(ar, ac, br, bc, rows, cols, s);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_CONN) | s->attr);
}

/*
 * draw_neurons — bright `(O)` 3-cell sigil at every neuron centre. The
 * parens give the neuron a clear visual border that reads as a circle
 * even in fonts where 'O' alone looks like a regular letter.
 */
static void draw_neurons(const Net *n, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
    for (int i = 0; i < n->n_layers; i++) {
        for (int j = 0; j < n->n_per_layer; j++) {
            int sr, sc;
            neuron_cell(i, j, n->n_layers, n->n_per_layer, rows, cols, &sr, &sc);
            if (sr < 0 || sr >= rows - 1) continue;
            if (sc - 1 >= 0  )  mvaddch(sr, sc - 1, '(');
            if (sc     <  cols)  mvaddch(sr, sc,     'O');
            if (sc + 1 <  cols)  mvaddch(sr, sc + 1, ')');
        }
    }
    attroff(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Net *n,
                       const ParticlePool *p, double fps)
{
    erase();
    draw_connections(n, rows, cols);
    particle_draw(p, n, rows, cols);   /* over connections, under neurons */
    draw_neurons(n, rows, cols);

    /* HUD — top right */
    char buf[112];
    snprintf(buf, sizeof buf,
             " layers:%d  neurons:%d  thick:%s  theme:%d  %5.1f fps  %s ",
             n->n_layers, n->n_per_layer,
             LINE_THICKNESS[n->thickness].name, n->theme, fps,
             n->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Key hints — bottom left */
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:layers  -/+:neurons  </>:thick  t:theme  r:reset  p:pause  q:quit  [neural_net_vis stage 1] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

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

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    Net          net;  net_reset(&net);
    ParticlePool pool; particle_reset(&pool, &net);
    screen_init(net.theme);

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           net.paused ^= 1; break;
                case 'r':           net_reset(&net); particle_reset(&pool, &net);
                                    color_init(net.theme); break;
                case 't':           net.theme = (net.theme + 1) % N_THEMES;
                                    color_init(net.theme); break;
                case '[':           if (net.n_layers > MIN_LAYERS) {
                                        net.n_layers--; particle_reset(&pool, &net);
                                    } break;
                case ']':           if (net.n_layers < MAX_LAYERS) {
                                        net.n_layers++; particle_reset(&pool, &net);
                                    } break;
                case '-':           if (net.n_per_layer > MIN_NEURONS) {
                                        net.n_per_layer--; particle_reset(&pool, &net);
                                    } break;
                case '+': case '=': if (net.n_per_layer < MAX_NEURONS) {
                                        net.n_per_layer++; particle_reset(&pool, &net);
                                    } break;
                case ',': case '<': if (net.thickness > MIN_THICKNESS) net.thickness--; break;
                case '.': case '>': if (net.thickness < MAX_THICKNESS) net.thickness++; break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;   /* spiral-of-death cap */
        t_tick_prev = now;
        if (!net.paused) particle_tick(&pool, &net, dt);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &net, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
