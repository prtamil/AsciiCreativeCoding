/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aafire_port.c — aalib 1.4 fire cellular automaton, rebuilt on the
 *                 ncurses framework with a gamma → Floyd-Steinberg →
 *                 perceptual-LUT rendering pipeline.
 *
 * DEMO: A flame stands on the bottom edge of the terminal.  Heat
 *       lives in a uint8 grid; each cell reads a five-neighbour sum
 *       from the cells BELOW it (3 from one row below, 2 from two
 *       rows below — no centre on the deeper row), runs that sum
 *       through a precomputed decay lookup, and writes the cooled
 *       result back.  Two extra rows under the visible area are
 *       FUEL: each tick they are reseeded in an arch-shaped sweep
 *       so the flame stands tall in the middle and shorter at the
 *       sides.  The heat field is rendered by gamma-correcting
 *       each byte, dithering with Floyd-Steinberg error diffusion,
 *       looking the dithered float up in a perceptual LUT to pick
 *       one of nine glyphs from ' ' (cold) to '@' (hot), then
 *       painting that glyph in the active theme's colour pair.
 *       Press `t` to cycle 10 palettes; `d` to switch among four
 *       debug overlays that expose the raw CA state, the fuel
 *       band alone, and the un-dithered quantisation.
 *
 * Study alongside:
 *   particle_systems/fire.c     Doom-style fire with a 3-neighbour
 *                               stencil but identical render pipeline.
 *                               Diff the two CAs to see why aafire
 *                               makes rounded blobs and Doom makes
 *                               sharp spires.
 *   particle_systems/embers.c   particle-based fire that uses no CA
 *                               at all.  Contrasts the grid approach
 *                               with the particle approach.
 *   particle_systems/comet.c    canonical reference for the LITERATE
 *                               doctrine this file is built under.
 *
 * Section map:
 *   §1  config             — sim constants, fuel knobs, timing
 *   §2  clock              — monotonic timer + sleep
 *   §3  LUT                — perceptual heat → ramp bucket
 *   §4  themes             — 10 colour palettes + theme_apply
 *   §5  bitmap state       — clamp helpers + Bitmap struct
 *   §6  decay table        — gentable() builds the per-row cooling LUT
 *   §7  propagation        — firemain() applies the 5-neighbour stencil
 *   §8  fuel seeding       — drawfire() seeds the bottom 2 rows in an arch
 *   §9  bitmap lifecycle   — alloc / free / init / per-tick
 *   §10 render pipeline    — gamma + Floyd-Steinberg dither + paint
 *   §11 scene              — orchestrator over Bitmap, pause, debug mode
 *   §12 debug overlay      — alternative render modes for inspection
 *   §13 screen + HUD       — ncurses init + bright HUD strip
 *   §14 app                — App struct + signal handlers + key dispatch
 *   §15 main               — fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit                     space      pause / resume
 *   t / T      next / prev theme        d / D      next / prev debug mode
 *   g / G      fuel up / fuel down      ] / [      sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/aafire_port.c \
 *       -o aafire_port -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * READING ORDER
 *   1. CONCEPTS + MENTAL MODEL (below) — the CA in plain English with the
 *      key formulas and edge cases listed.
 *   2. GUIDED TUTORIAL (below) — eleven mini-lessons that build the
 *      whole system from "why does fire LOOK like that" up to the full
 *      gamma → dither → LUT → paint pipeline, the 10 themes, and the
 *      four debug overlays.
 *   3. §1 config — every constant you would tweak while experimenting.
 *      Read this BEFORE the algorithm so each magic-name has context.
 *   4. §6 bitmap state — the Bitmap struct is the simulation's data
 *      atlas; every later helper operates on this struct.  Skim it so
 *      the §7–§10 functions read with shared context.
 *   5. §7–§9 are the CA core, in execution order: gentable() builds the
 *      decay LUT once at init/resize, drawfire() seeds the fuel rows
 *      every tick, firemain() propagates every tick.
 *   6. §10 render pipeline is three passes over the same buffer.  Read
 *      them in order; each pass has one job.
 *   7. §11–§15 are scene orchestration, the debug overlay, ncurses,
 *      signals, key dispatch, and the main loop — standard framework
 *      glue used in other files in this repo.
 *
 * NAMING (one line per significant identifier)
 *   Bitmap            simulation state struct.  Owns bmap / prev / dither
 *                     buffers, the decay table, the warm-up counter,
 *                     fuel intensity, theme + cycle scalars.
 *   bmap[]            uint8 heat grid.  Size cols × (rows + 2).  The
 *                     extra 2 rows on the bottom are FUEL — read by
 *                     firemain but never rendered.
 *   prev[]            uint8 mirror of last frame's bmap — used by the
 *                     diff-clear in pass 2 of the render pipeline.
 *   dither[]          float work buffer for Floyd-Steinberg.  Negative
 *                     value is the cold/skip sentinel.
 *   table[1280]       decay lookup — table[neighbour_sum] = decayed
 *                     output value.  Built once by gentable().
 *   ArchSweep         three counters (column, i1, i2) walking the fuel
 *                     rows.  Their min is the per-column heat ceiling.
 *   FireTheme         { name, fg256[9], fg8[9], attr8[9] } — one named
 *                     palette.  10 are declared in §3.
 *   DebugMode         enum { OFF, RAW, FUEL, NODITHER } — d/D selector.
 *   k_ramp[9]         glyph alphabet " .:+x*X#@".
 *   k_lut_breaks[9]   perceptual thresholds for each glyph bucket.
 *   PAIR_RAMP_BASE    colour pair for ramp bucket 0; bucket k uses
 *                     PAIR_RAMP_BASE + k.
 *   PAIR_HUD/HINT     bright theme-independent HUD pairs.
 *   COLD_SENTINEL     dither[] value used to mean "cell is cold; skip".
 *
 * BACKGROUND ASSUMED
 *   • Plain C, pointer arithmetic, fixed-size arrays.  No threads.
 *   • Cellular automata at a Game-of-Life level — each cell is a
 *     pure function of its neighbours.
 *   • Floyd-Steinberg dithering — distribute quantisation error to
 *     forward-unseen neighbours so banding breaks into a textured
 *     gradient.  Weights 7/3/5/1 over /16.
 *   • Perceptual gamma — eyes are non-linear in brightness.  A 1/2.2
 *     exponent on the [0, 1] linear value approximates a perceptually
 *     uniform response.
 *   • ncurses double-buffer pattern: single stdscr, frame loop is
 *     erase → draw → wnoutrefresh → doupdate.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : aafire is a 2-D cellular automaton on a uint8 heat
 *                  grid.  Each tick runs two sweeps over the grid:
 *
 *                    drawfire()    overwrites the bottom 2 rows (FUEL)
 *                                  with new random heat whose per-column
 *                                  ceiling follows an arch profile.
 *                    firemain()    for every cell, replaces its value
 *                                  with table[ Σ five neighbours below ].
 *                                  The decay table cools and averages
 *                                  in a single integer lookup.
 *
 *                  The 5-neighbour stencil reads 3 cells from y + 1 and
 *                  2 cells from y + 2 (skipping y + 2's centre).  That
 *                  asymmetric stencil is what produces the characteristic
 *                  rounded tongue shape:
 *
 *                    .   .   .         ← y         (cell being written)
 *                    a   b   c         ← y + 1
 *                    d       e         ← y + 2     (no centre)
 *
 *                  Decay table: table[s] = max(0, (s − minus) / 5)  with
 *                  minus = max(1, 800 / rows).  Subtraction BEFORE the
 *                  divide concentrates cooling on LOW sums, so dim outer
 *                  edges die fast while the bright core decays slowly.
 *
 * Data-structure : One flat uint8 array bmap[(rows + 2) × cols],
 *                  row-major bmap[y × cols + x].  The +2 phantom rows
 *                  on the bottom are the fuel band.  The decay table is
 *                  a single uint32 array of MAXTABLE = 1280 entries.
 *                  prev[] is a uint8 mirror of last frame's bmap for the
 *                  diff-clear in pass 2.  dither[] is a float work buffer
 *                  for Floyd-Steinberg.  Zero per-frame allocations —
 *                  every buffer is sized at init/resize and reused.
 *
 * Rendering      : Three sequential passes over the visible region:
 *
 *                    Pass 1   gamma-correct each heat byte into the
 *                             float dither buffer.  Cold bytes get a
 *                             COLD_SENTINEL value.
 *                    Pass 2   for each cell, quantise the gamma-corrected
 *                             value to a ramp bucket via k_lut_breaks[],
 *                             diffuse the quantisation error to forward
 *                             neighbours (Floyd-Steinberg 7/3/5/1),
 *                             paint k_ramp[bucket] in the theme's pair.
 *                             Cold cells that WERE lit last frame draw a
 *                             single ' ' to erase the stale glyph.
 *                    Pass 3   memcpy bmap → prev so next frame's pass 2
 *                             can do its diff-clear.
 *
 * Performance    : Three O(cols × rows) sweeps per tick (drawfire,
 *                  firemain, render).  At a 120 × 40 terminal that's
 *                  4 800 byte ops per sweep × 3 = ~14 k ops per tick.
 *                  60 ticks per second still leaves wall-clock budget
 *                  for ncurses I/O, which is the actual bottleneck.
 *                  No malloc/free after init; every per-tick byte
 *                  lives in a buffer allocated up front.
 *
 * References     :
 *   Jan "Yarrick" Olszak, aalib 1.4 — aafire.c (1999).  The original
 *     algorithm; this file ports the CA logic faithfully but rewrites
 *     the rendering layer.    https://aa-project.sourceforge.net/aalib/
 *   Fabien Sanglard, "How Doom Fire Was Done" (2014).  Walks through
 *     the simpler 3-neighbour Doom CA; diff against aafire's stencil
 *     to see why the silhouettes differ.
 *     http://fabiensanglard.net/doom_fire_psx/
 *   Floyd & Steinberg, "An Adaptive Algorithm for Spatial Greyscale"
 *     (SID International Symposium, 1976).  Original FS dithering paper.
 *   particle_systems/fire.c   — sibling file; same render pipeline,
 *                               Doom-style CA instead of aafire.
 *   particle_systems/comet.c  — canonical reference for the LITERATE
 *                               doctrine used by this file.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Fire is not a physics simulation — it is an upside-down convection
 * cellular automaton.  Each cell reads a smoothed-and-cooled average of
 * the cells BELOW it; fresh heat is injected at the bottom in an arch
 * shape; over a few hundred ticks the screen self-organises into the
 * familiar tongue-of-flame silhouette.  Zero floats, zero trigonometry,
 * zero physics in the hot path.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a chimney filled with smoke detectors.  Every detector reports
 * "average of the five detectors below me, minus a cooling constant."
 * The minus is gravity-as-cooling: heat loses some energy with every
 * row it climbs.  The chimney is fed at the bottom by an uneven gas
 * burner whose flame is taller in the middle and shorter at the sides.
 * Run the chimney for a few thousand frames and you get the same
 * upward-pointing dome you see in a real fire — without a single line
 * of fluid dynamics.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Allocate a (rows + 2) × cols byte grid.  The bottom 2 rows are
 *     FUEL — invisible to the renderer, readable by firemain.
 *  2. Build the decay table once: for every possible neighbour sum in
 *     [0, MAXTABLE − 1], compute max(0, (sum − cooling_per_row) / 5).
 *     cooling_per_row = max(1, 800 / rows).
 *  3. Each tick — drawfire() overwrites the two fuel rows.  An ArchSweep
 *     counter walks across cols; the per-column heat ceiling is
 *     min(i1, i2, height).  Within a burst of up to 6 cells the seed
 *     value walks ±2; the next burst picks its own base from rand()%cap.
 *  4. Each tick — firemain() sweeps the whole grid top-down.  For every
 *     (x, y), sum the 5 neighbours below, look up table[sum], write
 *     the result into bmap[y × cols + x].
 *  5. Render: gamma-correct each heat byte, diffuse Floyd-Steinberg
 *     error to forward neighbours, look up the dithered value in
 *     k_lut_breaks[] for a ramp bucket, paint k_ramp[bucket] in the
 *     theme's pair.  Cells that went hot → cold get a ' ' to erase
 *     the stale glyph.  memcpy bmap → prev for next frame's diff-clear.
 *  6. HUD: bright yellow row 0 with fps / theme / fuel / debug-mode;
 *     bright cyan row rows−1 with every key.
 *
 * KEY FORMULAS
 * ────────────
 *  decay table:    table[s] = max(0, (s − minus) / 5)
 *                  minus    = max(1, 800 / rows)
 *  arch ceiling:   cap      = min(i1, i2, height)
 *                  i1, i2 advance ±4 per cell
 *  seed value:     v        = (rand() % cap) × fuel_intensity
 *  intra-burst:    v        ← clamp(v + rand() % 6 − 2, 0, 255)
 *  perceptual:     g        = (heat / 255) ^ (1 / 2.2)
 *  FS weights:     7/16 right, 3/16 ↙, 5/16 ↓, 1/16 ↘
 *  bucket choice:  largest k such that g ≥ k_lut_breaks[k]
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • THE +2 PHANTOM ROWS.  firemain reads bmap[y + 2] for the bottom
 *    visible row, so the buffer MUST be (rows + 2) tall.  Forgetting
 *    this is the most common port bug — you get garbage flames or a
 *    segfault, depending on the malloc backing.
 *  • 800/rows UNDERFLOW.  On very tall terminals (rows > 800) the
 *    integer division gives 0 and the decay table collapses to all
 *    zeros (no cooling → flame fills the screen).  gentable() clamps
 *    cooling_per_row to ≥ 1.
 *  • CAP < 1.  The arch sweep can produce cap = 0 in early frames or
 *    on very narrow terminals.  rand() % 0 is undefined behaviour;
 *    the cap ≥ 1 floor is critical.
 *  • RESIZE REBUILDS GENTABLE.  The "minus" constant depends on rows,
 *    so a SIGWINCH must call gentable() or the cooling rate becomes
 *    wrong for the new aspect ratio.
 *  • DIFF-CLEAR vs FULL ERASE.  pass 2 writes ' ' only for cells
 *    that were lit LAST frame and cold THIS frame.  Without this,
 *    rapidly cooling cells leave stale glyphs on screen.
 *  • THEME CYCLE rebinds the same colour pair IDs to different RGB
 *    values, so a cell painted with PAIR_RAMP_BASE+5 last frame and
 *    again this frame visually changes colour.  After the cycle we
 *    set needs_clear so the screen redraws fully.
 *  • DEBUG OVERLAYS bypass parts of the pipeline; they must still
 *    set needs_clear when entered or stale glyphs from the previous
 *    render mode hang around.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Cold start: at t = 0 the grid is empty; over ~cols frames the
 *    flame grows as `height` climbs through the warm-up clamp.
 *  • G to lower fuel: the dome shrinks without changing shape.
 *  • Pause then unpause: the flame continues from its current state,
 *    NOT a fresh cold seed.
 *  • t to cycle themes: silhouette unchanged, only colour shifts.
 *  • d into DEBUG_RAW: every cell shows a hex digit 0–F from the raw
 *    heat byte — the bare CA state, no rendering pipeline at all.
 *  • d again into DEBUG_FUEL: only the bottom 2 rows are painted;
 *    you can watch the arch sweep + ±2 walk in isolation.
 *  • d again into DEBUG_NODITHER: visible banding returns; contrast
 *    against DEBUG_OFF to appreciate what dithering buys.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 * Eleven mini-lessons in dependency order.  Each opens with a question,
 * answers in plain English, and ends with the pseudocode line that maps
 * onto a real function below.  Read in sequence.
 *
 * ─── 1.  Why does fire LOOK like that?  ──────────────────────────────── *
 *   Real fire is hot gas under gravity.  The hot part is less dense, so
 *   it rises; it mixes with cooler air on the way up, so it cools; the
 *   brightest place is the base, where the reaction is still happening.
 *   We DON'T simulate any of that.  We capture the SHAPE OF THE
 *   OBSERVATION: bright base, vertical streaks, flickering edges.  A
 *   tiny cellular automaton on a byte grid does the job.
 *
 *        observation  →  byte grid  →  per-tick CA  →  rendered image
 *
 * ─── 2.  Heat as a byte grid (no particles, no floats)  ──────────────── *
 *   Each terminal cell is one uint8 in [0, 255]:
 *      0   = cold / background
 *      255 = white-hot core
 *   The whole simulation state is bmap[(rows + 2) × cols] bytes.
 *   Particles would burn memory bandwidth on per-frame heap traffic;
 *   floats would let numerical drift sneak in over thousands of ticks.
 *   Bytes give exact reproducibility and trivial memcpy archiving.
 *
 *        bmap[y, x] : uint8 ∈ [0, 255]
 *
 * ─── 3.  The 5-neighbour stencil  ────────────────────────────────────── *
 *   To make heat RISE, write each cell as the smoothed average of cells
 *   BELOW it.  aafire picks five: three in y + 1 and two in y + 2.  Note
 *   the gap — y + 2 has NO centre cell.
 *
 *                .   .   .         ← y       (cell being written)
 *                a   b   c         ← y + 1   (3 neighbours)
 *                d       e         ← y + 2   (2 neighbours, NO centre)
 *
 *   Leaving the y + 2 centre off biases the stencil toward the SIDES of
 *   the cell directly below — that's what gives aafire its rounded
 *   tongue shape instead of straight vertical streaks.
 *
 *        new[x, y] = (a + b + c + d + e) / 5     minus a cooling constant
 *
 * ─── 4.  The decay table (cool BEFORE divide)  ───────────────────────── *
 *   The division by 5 averages the [0, 1275] sum back into [0, 255].
 *   "Minus a cooling constant" is the gravity-as-cooling step:
 *
 *        minus = max(1, 800 / rows)
 *
 *   Subtraction BEFORE division concentrates cooling on LOW sums, which
 *   is precisely where the flame should die — at the cold edges.  A high
 *   sum (bright core) is barely affected.  Build the table once, never
 *   divide in the hot loop:
 *
 *        table[s] = max(0, (s − minus) / 5)    for s = 0 .. 1279
 *
 *   1280 entries covers the maximum possible sum of 5 × 255 = 1275 with
 *   5 extra slots as paranoia padding.
 *
 * ─── 5.  Fuel-row seeding — the arch  ────────────────────────────────── *
 *   The bottom two rows are FUEL.  Each tick, fill them with random heat
 *   so the CA above has new material to propagate.  A uniform fuel row
 *   gives a flat-bottomed flame; aafire produces the iconic dome by
 *   SWEEPING three counters across cols:
 *
 *      i1     = 1, 5, 9, …         (grows from the LEFT edge by 4 per cell)
 *      i2     = 4·cols+1, …, 1     (shrinks from the RIGHT edge by 4 per cell)
 *      cap    = min(i1, i2, height)
 *
 *      heat ceiling
 *          .,;+x*X#@*Xx+;,.     ← the arch (drawn with the ramp glyphs)
 *          |________________|
 *                  cols
 *
 *   min(i1, i2) creates the dome shape; min(., height) clamps during the
 *   warm-up so flames start tiny and grow over the first ~cols frames.
 *
 *        cap  = min(i1, i2, height)    (≥ 1 floor — see edge cases)
 *        seed = (rand() % cap) × fuel_intensity
 *
 * ─── 6.  Bursts + ±2 random walk  ────────────────────────────────────── *
 *   Within a burst of up to 6 consecutive cells, the seed value is NOT
 *   chosen freshly per cell.  It walks ±2 around its starting value:
 *
 *        v ← seed
 *        for k in 0..burst_length:
 *          fuel_row [col] ← clamp(v)
 *          v             ← v + rand() % 6 − 2
 *          fuel_row2[col] ← clamp(v)
 *          v             ← v + rand() % 6 − 2
 *          advance sweep
 *
 *   This gives intra-burst micro-flicker (adjacent cells differ a little)
 *   while inter-burst macro-flicker is preserved (each burst picks its
 *   own base from rand() % cap).  The flame's texture comes from these
 *   walks more than from the arch sweep itself.
 *
 * ─── 7.  Why gamma correction?  ──────────────────────────────────────── *
 *   Mapping uint8 directly to 9 glyph buckets gives banding the eye can
 *   see — humans are more sensitive to brightness changes at the low
 *   end of the scale.  Gamma correction flattens the response curve:
 *
 *        normalised  = heat / 255                  ∈ [0, 1]
 *        perceptual  = normalised ^ (1 / 2.2)      ≈ perceived brightness
 *
 *   The 1/2.2 exponent compresses bright values and expands dark ones,
 *   so equal steps in `perceptual` correspond to equal steps in perceived
 *   brightness.  Downstream LUT breakpoints can now be evenly spaced
 *   and still look smooth.
 *
 * ─── 8.  Floyd-Steinberg dithering  ──────────────────────────────────── *
 *   Even with gamma correction, 9 glyph buckets across [0, 1] leave
 *   visible bands.  Floyd-Steinberg fixes the banding by distributing
 *   each cell's quantisation error to forward neighbours:
 *
 *                ──► 7/16 to (x + 1, y)
 *      3/16 to  (x − 1, y + 1)
 *      5/16 to  (x,     y + 1)
 *      1/16 to  (x + 1, y + 1)
 *
 *   The numerator weights add to 16; dividing distributes the same
 *   total error.  Over a region the per-cell error averages out to zero,
 *   so banding becomes a textured gradient.
 *
 *        error = perceptual − bucket_midpoint
 *        diffuse(error, 7/16, 3/16, 5/16, 1/16)
 *
 * ─── 9.  Diff-clearing vs erase()  ───────────────────────────────────── *
 *   erase() retransmits the whole screen every frame — visible flicker
 *   on slow terminals.  Instead, pass 3 memcpys bmap → prev; next frame's
 *   pass 2 writes ' ' only for cells that went from prev > 0 to current
 *   == 0.  Most frames touch only a handful of cells; the terminal
 *   repaint cost stays nearly free.
 *
 *        if perceptual == COLD_SENTINEL  AND  prev > 0:  mvaddch(' ')
 *
 * ─── 10.  The ten themes  ────────────────────────────────────────────── *
 *   Each theme is just a 9-colour ramp (cold → hot) plus an 8-colour
 *   fallback table for non-256-colour terminals.  Theme cycling rebinds
 *   the same colour pair IDs to different RGB values, so the flame's
 *   silhouette is preserved exactly — only the palette shifts.
 *
 *      matrix   ocean   ice            cool palettes (green / blue / cyan)
 *      neon     plasma  aurora         saturated mixed palettes
 *      nova     fire    gold   toxic   warm palettes (red / orange / yellow)
 *
 *   `t` advances, `T` retreats.  Auto-advance every CYCLE_TICKS ticks
 *   so an idle demo keeps looking fresh.
 *
 * ─── 11.  Debug overlays  ────────────────────────────────────────────── *
 *   Four render modes selectable with `d` / `D`:
 *
 *      DEBUG_OFF        normal pipeline (default)
 *      DEBUG_RAW        each cell shows the heat byte in hex (0-F);
 *                       reveals the raw CA state without gamma / dither /
 *                       LUT.
 *      DEBUG_FUEL       only the bottom 2 fuel rows are painted; lets
 *                       you watch the arch sweep + ±2 walk in isolation.
 *      DEBUG_NODITHER   gamma + LUT + paint, no Floyd-Steinberg;
 *                       contrast against DEBUG_OFF to see what dithering
 *                       removes.
 *
 *   The debug renderer's source is in §12 and IS part of the lesson.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,
    HUD_COLS        = 52,
    FPS_UPDATE_MS   = 500,
    CYCLE_TICKS     = 300,   /* ticks before auto-cycling theme          */
};

/*
 * aalib uses a 256-entry uint8 bitmap.
 * MAXTABLE must cover the maximum possible sum of 5 neighbours × 255 = 1275.
 * Original: MAXTABLE = 256*5 = 1280.  We keep the same.
 */
#define MAXTABLE   1280

/* Fuel intensity scale [0.1, 1.0] — multiplied against the base heat */
#define FUEL_DEFAULT  1.0f
#define FUEL_STEP     0.05f
#define FUEL_MIN      0.1f
#define FUEL_MAX      1.0f

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC/(f))

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
    struct timespec r = { (time_t)(ns/NS_PER_SEC),(long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  LUT — perceptual heat → ramp bucket                                */
/* ===================================================================== */

/*
 * §3 PREAMBLE — quantising perceptual heat to a glyph bucket
 * ──────────────────────────────────────────────────────────
 *
 * The render pipeline (§10) computes a gamma-corrected float in
 * [0, 1] for each cell.  This section maps that float onto one of
 * 9 glyph buckets, then exposes the bucket's midpoint so Floyd-
 * Steinberg can compute its quantisation error.
 *
 *   k_ramp[9]         the glyph alphabet, cold → hot.
 *   k_lut_breaks[9]   perceptual thresholds, each bucket covers
 *                     roughly equal perceived brightness.
 *   lut_index(v)      which bucket does v belong in?
 *   lut_midpoint(k)   centre of bucket k — used for FS error.
 */

/*
 * Ramp — same 9-level set as fire.c.
 * ' ' = cold/background, '@' = hottest core.
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1)

/*
 * HUD/HINT color pairs sit AFTER the theme palette so theme cycling
 * (which only touches CP_BASE..CP_BASE+RAMP_N-1) never clobbers them.
 *
 *   PAIR_HUD  — bright yellow 226 (8-color fallback: COLOR_YELLOW)
 *   PAIR_HINT — bright cyan   51  (8-color fallback: COLOR_CYAN)
 *
 * Both drawn with A_BOLD per CLAUDE.md HUD standard — never A_DIM, so the
 * lines stay legible against the brightest theme.
 */
#define PAIR_HUD  (CP_BASE + RAMP_N)
#define PAIR_HINT (CP_BASE + RAMP_N + 1)

static const float k_lut_breaks[RAMP_N] = {
    0.000f, 0.080f, 0.180f, 0.290f, 0.390f,
    0.500f, 0.620f, 0.750f, 0.900f,
};

static int lut_index(float v)
{
    int idx = 0;
    for (int i = RAMP_N-1; i >= 0; i--)
        if (v >= k_lut_breaks[i]) { idx = i; break; }
    return idx;
}
static float lut_midpoint(int idx)
{
    if (idx <= 0)        return 0.f;
    if (idx >= RAMP_N-1) return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx+1]) * 0.5f;
}

/* ===================================================================== */
/* §4  themes — 10 colour palettes                                        */
/* ===================================================================== */

/*
 * §4 PREAMBLE — colour palettes
 * ─────────────────────────────
 *
 * Each FireTheme is a 9-entry ramp from cold (bucket 0) to hot
 * (bucket 8).  fg256 is the 256-colour-cube index for terminals that
 * support it; fg8 is the basic-8-colour fallback; attr8 layers
 * A_DIM / A_NORMAL / A_BOLD on top of fg8 to expand the visible
 * brightness range when the palette is small.
 *
 * theme_apply() rebinds the 9 ramp pair IDs (CP_BASE+0..+8) to the
 * active theme's colours.  HUD pairs (PAIR_HUD/HINT) are bound once
 * by color_init() and never touched again — so theme cycling can
 * never break the HUD's legibility.
 */
typedef struct {
    const char *name;
    int         fg256[RAMP_N];
    int         fg8[RAMP_N];
    attr_t      attr8[RAMP_N];
} FireTheme;

#define CP_BASE 1

/*
 * The 10 default themes per LITERATE doctrine.  Order matches the
 * doctrine's canonical list:
 *
 *   matrix  neon  nova  ocean  fire  toxic  gold  ice  aurora  plasma
 *
 * Every fg256 in indices 1..8 sits in the BRIGHT HALF of the 256-cube
 * (cube ≥ 24, greys ≥ 244) per CLAUDE.md's Theme Palette Brightness
 * rule.  Index 0 is the "cold/empty" slot — its colour is never
 * visible because k_ramp[0] is the space character, so index 0 can
 * use the otherwise-forbidden 232.
 *
 * attr8[] layers A_DIM / A_NORMAL / A_BOLD on top of the basic-8
 * palette to expand the brightness range on small-palette terminals.
 */
static const FireTheme k_themes[] = {
    { "matrix",
      { 232,  22,  28,  34,  40,  46,  82, 118, 231 },
      { COLOR_BLACK, COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN,  COLOR_GREEN,  COLOR_WHITE,  COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_DIM,    A_NORMAL,
        A_NORMAL, A_BOLD,   A_BOLD,   A_BOLD,    A_BOLD }
    },
    { "neon",
      { 232,  53,  89, 125, 161, 197, 213, 219, 231 },
      { COLOR_BLACK,   COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_BOLD,
        A_BOLD,   A_DIM,    A_NORMAL, A_BOLD,    A_BOLD }
    },
    { "nova",
      { 232,  52,  88, 124, 160, 196, 208, 220, 231 },
      { COLOR_BLACK, COLOR_RED,    COLOR_RED,    COLOR_RED,
        COLOR_RED,   COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_NORMAL,
        A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD,    A_BOLD }
    },
    { "ocean",
      { 232,  17,  19,  21,  27,  33,  39, 123, 231 },
      { COLOR_BLACK, COLOR_BLUE,   COLOR_BLUE,   COLOR_BLUE,
        COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_BOLD,
        A_BOLD,   A_DIM,    A_BOLD,   A_BOLD,    A_BOLD }
    },
    { "fire",
      { 232,  52,  88, 124, 160, 196, 202, 214, 231 },
      { COLOR_BLACK, COLOR_RED,    COLOR_RED,    COLOR_RED,
        COLOR_RED,   COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_NORMAL,
        A_BOLD,   A_DIM,    A_NORMAL, A_BOLD,    A_BOLD }
    },
    { "toxic",
      { 232,  22,  28,  34, 106, 154, 190, 226, 231 },
      { COLOR_BLACK, COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,
        COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_BOLD,
        A_DIM,    A_NORMAL, A_BOLD,   A_BOLD,    A_BOLD }
    },
    { "gold",
      { 232,  94, 130, 136, 172, 178, 214, 220, 231 },
      { COLOR_BLACK, COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
        COLOR_YELLOW,COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_NORMAL,
        A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD,    A_BOLD }
    },
    { "ice",
      { 232,  17,  19,  21,  27,  33,  51, 123, 231 },
      { COLOR_BLACK, COLOR_BLUE,   COLOR_BLUE,   COLOR_BLUE,
        COLOR_CYAN,  COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_BOLD,
        A_DIM,    A_NORMAL, A_BOLD,   A_BOLD,    A_BOLD }
    },
    { "aurora",
      { 232,  22,  28,  34, 121, 159, 207, 219, 231 },
      { COLOR_BLACK, COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,
        COLOR_CYAN,  COLOR_CYAN,   COLOR_MAGENTA,COLOR_MAGENTA,COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_BOLD,
        A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD,    A_BOLD }
    },
    { "plasma",
      { 232,  53,  91, 129, 165, 207, 213, 219, 231 },
      { COLOR_BLACK,   COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,    COLOR_CYAN,  COLOR_WHITE },
      { A_NORMAL, A_DIM,    A_NORMAL, A_NORMAL,
        A_BOLD,   A_BOLD,   A_DIM,    A_BOLD,    A_BOLD }
    },
};
#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static void theme_apply(int t)
{
    const FireTheme *th = &k_themes[t];
    for (int i = 0; i < RAMP_N; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE+i, th->fg256[i], COLOR_BLACK);
        else
            init_pair(CP_BASE+i, th->fg8[i],   COLOR_BLACK);
    }
}
static void color_init(int theme)
{
    start_color();
    use_default_colors();
    theme_apply(theme);

    /* HUD pairs are theme-independent — init ONCE, theme_apply leaves them alone. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

static attr_t ramp_attr(int i, int theme)
{
    attr_t a = COLOR_PAIR(CP_BASE+i);
    if (COLORS >= 256) {
        if (i >= RAMP_N-2) a |= A_BOLD;
    } else {
        a |= k_themes[theme].attr8[i];
    }
    return a;
}

/* ===================================================================== */
/* §5  bitmap state — clamp helpers + the Bitmap struct                   */
/* ===================================================================== */

/*
 * §5 PREAMBLE — what holds the simulation state
 * ─────────────────────────────────────────────
 *
 * Two integer-clamp inlines + the Bitmap struct that owns every byte
 * touched by the per-tick code.  Reading this section before §6-§10
 * means every later helper signature reads with full context.
 *
 *   clamp_int       — clamp an integer to a closed interval.
 *   clamp_uchar_int — specialisation for uint8 saturation arithmetic.
 *   Bitmap          — the simulation's single mutable struct.
 */
static inline int clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline int clamp_uchar_int(int v)
{
    return clamp_int(v, 0, 255);
}

/*
 * Bitmap — aalib uses a flat uint8 array of (cols × (rows+2)).
 * The extra 2 rows at the bottom are the fuel rows that get seeded.
 * `rows` here is the visible rows; `rows+2` is the full height.
 *
 * table[]  — the decay lookup table built by gentable().
 *            table[sum_of_5_neighbours] = decayed heat value.
 *
 * prev[]   — previous frame's bitmap for diff-based clearing.
 *            Only cells that went from hot to cold get erased.
 *
 * dither[] — float working buffer for Floyd-Steinberg.
 *
 * height   — frame counter; grows until fuel reaches full brightness.
 * loop     — countdown for fuel seeding bursts (original logic).
 * sloop    — counts seeding cycles.
 * fuel     — user-adjustable intensity [0.1, 1.0].
 * theme, cycle_tick — theme state.
 */
typedef struct {
    unsigned char *bmap;      /* [cols * (rows+2)] heat uint8             */
    unsigned char *prev;      /* [cols * rows]     last drawn frame        */
    float         *dither;    /* [cols * rows]     dither working buffer   */
    unsigned int   table[MAXTABLE];
    int            cols;
    int            rows;
    int            height;    /* frame counter for warm-up                 */
    int            loop;      /* fuel burst countdown                      */
    int            sloop;     /* fuel sweep counter                        */
    float          fuel;      /* intensity scale                           */
    int            theme;
    int            cycle_tick;
} Bitmap;

/* ===================================================================== */
/* §6  decay table — gentable() builds the per-row cooling LUT            */
/* ===================================================================== */

/*
 * §6 PREAMBLE — cool-before-divide as a precomputed table
 * ───────────────────────────────────────────────────────
 *
 * The propagation step in §7 sums 5 heat values from below and needs
 * to look up the cooled-and-averaged result.  Computing
 * max(0, (sum - cooling_per_row) / 5) in the inner loop would force
 * a division per cell; instead we precompute every possible answer
 * once at init / resize, then the hot loop is just an array index.
 *
 *   decay_table_entry  — single-entry calculation.
 *   gentable           — fill the entire 1280-entry table.
 *
 * The cooling rate depends on the row count, so SIGWINCH must call
 * gentable() — see §15.
 */

/*
 * decay_table_entry() — one entry of the lookup table.
 *
 *   The full sum of 5 neighbours lives in [0, 5 × 255] = [0, 1275].
 *
 *   if sum ≤ cooling_per_row:  entry = 0           (cell becomes cold)
 *   else:                       entry = (sum − cooling_per_row) / 5
 *
 *   Subtracting BEFORE dividing concentrates cooling on low sums, which
 *   is precisely where we want the flame to die out.
 */
static unsigned int decay_table_entry(int neighbour_sum, int cooling_per_row)
{
    if (neighbour_sum <= cooling_per_row) return 0;
    return (unsigned int)(neighbour_sum - cooling_per_row) / 5;
}

/*
 * gentable() — build the entire decay lookup once per resize.
 *
 *   cooling_per_row = max(1, 800 / rows)
 *      • Big terminals (many rows) → small cooling, flame burns higher.
 *      • Small terminals          → big cooling, flame stays squat.
 *      • The max(1, …) guard prevents the table collapsing to all zeros
 *        on huge terminals where 800/rows would be 0.
 *
 *   The result is a 1280-entry table; the inner loop in firemain()
 *   never performs a division.
 */
static void gentable(Bitmap *b)
{
    int cooling_per_row = 800 / b->rows;
    if (cooling_per_row == 0) cooling_per_row = 1;

    for (int neighbour_sum = 0; neighbour_sum < MAXTABLE; neighbour_sum++)
        b->table[neighbour_sum] = decay_table_entry(neighbour_sum, cooling_per_row);
}

/* ===================================================================== */
/* §7  propagation — firemain() applies the 5-neighbour stencil           */
/* ===================================================================== */

/*
 * §7 PREAMBLE — heat rises by reading the cells below
 * ───────────────────────────────────────────────────
 *
 * The CA's signature step.  For every (x, y) in the visible grid we
 * sum 5 cells below — 3 from y+1 and 2 from y+2 — look the sum up in
 * the decay table, write the result back.  Top-down sweep order is
 * important: if we went bottom-up, a freshly-written hot cell could
 * be re-read in the SAME tick as a y+1 neighbour, polluting the
 * propagation with same-tick state.
 *
 *   sample_five_neighbours_below  — one cell's neighbour sum.
 *   firemain                       — the full top-down sweep.
 */

/*
 * sample_five_neighbours_below() — read the 5 cells in the aafire stencil.
 *
 *   The five cells live BELOW (x, y) — positive y = down:
 *
 *                    .   .   .          ← y    (cell being written)
 *                    a   b   c          ← y+1  (3 neighbours)
 *                    d       e          ← y+2  (2 neighbours, NO centre)
 *
 *   Skipping the centre on y+2 biases the stencil toward the sides of
 *   the cell directly below; this is what gives aafire its rounded
 *   blob shape instead of vertical streaks.
 *
 *   Edge columns clamp left and right neighbours to the nearest valid
 *   column so the stencil never reads off-grid horizontally.  The
 *   (rows + 2) buffer height ensures the y+2 reads stay in bounds.
 */
static unsigned int sample_five_neighbours_below(const unsigned char *bmap,
                                                 int cols, int x, int y)
{
    int left_column  = clamp_int(x - 1, 0, cols - 1);
    int right_column = clamp_int(x + 1, 0, cols - 1);
    int row_below    = y + 1;
    int row_deeper   = y + 2;

    unsigned int below_left   = bmap[row_below  * cols + left_column];
    unsigned int below_centre = bmap[row_below  * cols + x];
    unsigned int below_right  = bmap[row_below  * cols + right_column];
    unsigned int deeper_left  = bmap[row_deeper * cols + left_column];
    unsigned int deeper_right = bmap[row_deeper * cols + right_column];

    return below_left + below_centre + below_right + deeper_left + deeper_right;
}

/*
 * firemain() — propagate heat upward via 5-neighbour averaging.
 *
 *   For every cell from top-left to bottom-right:
 *      neighbour_sum  ← sample the 5 cells below
 *      table_index    ← min(neighbour_sum, MAXTABLE − 1)
 *      bmap[y][x]     ← table[table_index]
 *
 *   The clamp on table_index is paranoia: the stencil's maximum sum is
 *   5 × 255 = 1275, comfortably under MAXTABLE = 1280.
 */
static void firemain(Bitmap *b)
{
    int            cols = b->cols;
    int            rows = b->rows;
    unsigned char *bmap = b->bmap;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            unsigned int neighbour_sum = sample_five_neighbours_below(bmap, cols, x, y);
            unsigned int table_index   = (neighbour_sum < MAXTABLE)
                                         ? neighbour_sum : MAXTABLE - 1;
            bmap[y * cols + x] = (unsigned char)b->table[table_index];
        }
    }
}

/* ===================================================================== */
/* §8  fuel seeding — drawfire() seeds the bottom 2 rows in an arch       */
/* ===================================================================== */

/*
 * §8 PREAMBLE — feeding the chimney an arch-shaped flame
 * ──────────────────────────────────────────────────────
 *
 * Without fresh fuel, the propagation in §7 just cools the grid
 * uniformly toward zero.  This section overwrites the bottom two
 * rows of the bitmap (the fuel rows) each tick.  The seed values
 * follow an arch profile so the flame is tall in the middle and
 * short at the edges; within bursts of up to 6 cells they walk
 * ±2 to give the fuel band micro-texture.
 *
 *   ArchSweep                       — the three counters that travel
 *                                     together across the fuel rows.
 *   arch_sweep_cap                  — per-column heat ceiling.
 *   arch_sweep_advance_one_cell     — step counters forward.
 *   seed_one_burst                  — write up to 6 consecutive cells.
 *   drawfire                        — sweep the whole row, burst by burst.
 */

/*
 * ArchSweep — three counters that travel together across the fuel row.
 *
 *   column : current column being seeded (0 .. cols)
 *   i1     : grows  from 1, +4 per cell           — small at LEFT edge
 *   i2     : shrinks from 4·cols+1, −4 per cell   — small at RIGHT edge
 *
 *   The per-column arch ceiling is min(i1, i2, height): low at both
 *   edges, peaks in the middle, capped by the warm-up counter.
 *
 *   Both i1 and i2 are advanced by 4 per cell — that's an aalib choice
 *   that makes the arch's edges curve more sharply than a linear ramp.
 */
typedef struct {
    int column;
    int i1;
    int i2;
} ArchSweep;

/*
 * arch_sweep_cap() — per-column heat ceiling = min(i1, i2, warmup).
 *
 *   The three terms encode three independent constraints:
 *      i1     — distance from the LEFT  margin (arch left half)
 *      i2     — distance from the RIGHT margin (arch right half)
 *      height — warm-up frame counter           (start-up clamp)
 *   The minimum is whichever constraint is currently tightest.
 *
 *   The floor (cap < 1 → cap = 1) is essential: rand() % 0 is
 *   undefined behaviour, so we must guarantee cap ≥ 1 before any
 *   rand() % cap.
 */
static int arch_sweep_cap(const ArchSweep *sweep, int height)
{
    int cap = (sweep->i1 < sweep->i2) ? sweep->i1 : sweep->i2;
    if (height < cap) cap = height;
    if (cap < 1)      cap = 1;
    return cap;
}

/*
 * arch_sweep_advance_one_cell() — move all three counters forward.
 *
 *   column ++   (move to next column)
 *   i1      += 4 (left-distance grows)
 *   i2      -= 4 (right-distance shrinks)
 */
static void arch_sweep_advance_one_cell(ArchSweep *sweep)
{
    sweep->column += 1;
    sweep->i1     += 4;
    sweep->i2     -= 4;
}

/*
 * seed_one_burst() — write up to 6 consecutive fuel cells.
 *
 *   Algorithm:
 *      cap          ← arch_sweep_cap(sweep, height)
 *      seed_value   ← (rand() % cap) × fuel_intensity
 *      burst_length ← rand() % 6                       (0..5)
 *      repeat burst_length+1 times (or until end of row):
 *          clamp seed_value to [0, 255]
 *          fuel_row [column] ← seed_value
 *          seed_value ← seed_value + rand() % 6 − 2    (±2 walk)
 *          clamp seed_value
 *          fuel_row2[column] ← seed_value
 *          seed_value ← seed_value + rand() % 6 − 2
 *          advance sweep one cell
 *
 *   The ±2 walk inside the burst creates micro-flicker along the fuel
 *   line — adjacent cells differ a little, but neighbouring bursts can
 *   differ a lot (each picks its own base from rand() % cap).
 */
static void seed_one_burst(Bitmap *b, ArchSweep *sweep,
                           unsigned char *fuel_row,
                           unsigned char *fuel_row2)
{
    int cap          = arch_sweep_cap(sweep, b->height);
    int seed_value   = (int)((float)(rand() % cap) * b->fuel);
    int burst_length = rand() % 6;

    for (int j = 0; j <= burst_length && sweep->column < b->cols; j++) {
        seed_value                  = clamp_uchar_int(seed_value);
        fuel_row [sweep->column]    = (unsigned char)seed_value;
        seed_value                  = clamp_uchar_int(seed_value + rand() % 6 - 2);
        fuel_row2[sweep->column]    = (unsigned char)seed_value;
        seed_value                 += rand() % 6 - 2;
        arch_sweep_advance_one_cell(sweep);
    }
}

/*
 * drawfire() — seed the two fuel rows at the bottom of the bitmap.
 *
 *   The body reads as the two algorithm steps:
 *      Step 1 — advance frame counter + burst countdown.
 *      Step 2 — sweep across cols: burst, single-cell gap, burst, gap, …
 *
 *   The burst/gap rhythm is what gives the fuel line its textured look;
 *   a uniform fill would produce a flat-bottomed wall of flame.
 */
static void drawfire(Bitmap *b)
{
    int            cols       = b->cols;
    int            rows       = b->rows;
    unsigned char *fuel_row   = b->bmap + rows       * cols;   /* row `rows`   */
    unsigned char *fuel_row_2 = b->bmap + (rows + 1) * cols;   /* row `rows+1` */

    /* Step 1 — advance frame counter and burst countdown. */
    b->height++;
    b->loop--;
    if (b->loop < 0) {
        b->loop = rand() % 3;
        b->sloop++;
    }

    /* Step 2 — sweep across columns: burst → gap → burst → gap → … */
    ArchSweep sweep = { .column = 0, .i1 = 1, .i2 = 4 * cols + 1 };
    while (sweep.column < cols) {
        seed_one_burst(b, &sweep, fuel_row, fuel_row_2);
        arch_sweep_advance_one_cell(&sweep);   /* one-cell gap between bursts */
    }
}

/* ===================================================================== */
/* §9  bitmap lifecycle — alloc / free / init / per-tick                  */
/* ===================================================================== */

/*
 * §9 PREAMBLE — buffer ownership and the tick orchestrator
 * ────────────────────────────────────────────────────────
 *
 * The Bitmap struct's three malloc'd buffers (bmap, prev, dither) are
 * allocated here, never reallocated in the hot path, and freed at
 * shutdown.  bitmap_tick() chains drawfire (§8) → firemain (§7) and
 * advances the theme auto-cycle counter; it returns true the frame
 * the theme just changed so the scene can mark needs_clear.
 *
 *   bitmap_alloc  — calloc the three buffers, store sizes.
 *   bitmap_free   — free + zero the struct.
 *   bitmap_init   — alloc + reset all scalars + gentable().
 *   bitmap_tick   — one CA step + theme auto-cycle.
 */

static void bitmap_alloc(Bitmap *b, int cols, int rows)
{
    b->cols   = cols;
    b->rows   = rows;
    /* +2 extra rows for the fuel rows that firemain reads from */
    b->bmap   = calloc((size_t)(cols * (rows + 2)), sizeof(unsigned char));
    b->prev   = calloc((size_t)(cols * rows),       sizeof(unsigned char));
    b->dither = calloc((size_t)(cols * rows),       sizeof(float));
}

static void bitmap_free(Bitmap *b)
{
    free(b->bmap); free(b->prev); free(b->dither);
    memset(b, 0, sizeof *b);
}

static void bitmap_init(Bitmap *b, int cols, int rows, int theme)
{
    bitmap_alloc(b, cols, rows);
    b->height     = 0;
    b->loop       = 0;
    b->sloop      = 0;
    b->fuel       = FUEL_DEFAULT;
    b->theme      = theme;
    b->cycle_tick = 0;
    gentable(b);
}

/*
 * bitmap_tick() — one simulation step: seed fuel rows then propagate.
 * Returns true if theme just cycled.
 */
static bool bitmap_tick(Bitmap *b)
{
    drawfire(b);
    firemain(b);

    b->cycle_tick++;
    if (b->cycle_tick >= CYCLE_TICKS) {
        b->cycle_tick = 0;
        b->theme = (b->theme + 1) % THEME_COUNT;
        theme_apply(b->theme);
        return true;
    }
    return false;
}

/* ===================================================================== */
/* §10  render pipeline — gamma + Floyd-Steinberg dither + paint          */
/* ===================================================================== */

/*
 * §10 PREAMBLE — three passes from heat to glyph
 * ──────────────────────────────────────────────
 *
 * The CA produces uint8 heat; the terminal can show 9 glyphs in any
 * of N colour pairs.  Bridging the two is a three-pass pipeline:
 *
 *   Pass 1   gamma-correct each byte into a float dither buffer.
 *            Cold bytes get COLD_SENTINEL so pass 2 can distinguish
 *            "draw a glyph" from "erase a stale glyph".
 *   Pass 2   for each cell: lut_index → bucket; quant error =
 *            perceptual − bucket midpoint; Floyd-Steinberg diffuse
 *            to the four forward neighbours; paint the glyph.
 *   Pass 3   memcpy bmap → prev so next frame's pass 2 can do its
 *            diff-clear ("was hot, now cold → write ' '").
 *
 * The 7/16, 3/16, 5/16, 1/16 weights add to 16; dividing distributes
 * the same total error across forward-unseen neighbours.  Over a
 * region, the per-cell error AVERAGES OUT to zero, so the visual
 * bands break into a textured gradient.
 *
 *   gamma_correct_byte             — perceptual conversion.
 *   COLD_SENTINEL                  — dither-buffer "skip" value.
 *   pipeline_pass1_gamma_correct   — pass 1 (per pixel).
 *   pipeline_diffuse_quant_error   — FS distribution from one cell.
 *   pipeline_draw_lit_cell         — quantise + diffuse + paint one cell.
 *   pipeline_pass2_quantise_and_draw — pass 2 (per pixel).
 *   pipeline_pass3_archive_current_frame — pass 3 (memcpy).
 *   bitmap_draw                    — three-pass orchestrator.
 */

/*
 * gamma_correct_byte() — uint8 linear heat → perceptual float in [0, 1].
 *
 *   linear     = heat / 255
 *   perceptual = linear ^ (1 / 2.2)
 *
 * Used by pass 1 of the main pipeline and by the DEBUG_FUEL /
 * DEBUG_NODITHER alternative renders in §12.  static inline so the
 * pixel loop sees the math directly.
 */
static inline float gamma_correct_byte(unsigned char heat)
{
    float linear = (float)heat / 255.f;
    return powf(linear, 1.f / 2.2f);
}

/*
 * COLD_SENTINEL — value written to the dither buffer for cells whose
 * heat is 0.  Negative so any non-negative entry can be distinguished
 * by a single (>= 0.f) check in the diffusion code.
 */
#define COLD_SENTINEL  (-1.0f)

/*
 * pipeline_pass1_gamma_correct() — fill dither_buffer with perceptual heat.
 *
 *   For every cell:
 *      linear_heat  ← bmap[i]              (uint8 in [0, 255])
 *      if cold:     dither_buffer[i] ← COLD_SENTINEL
 *      else:        dither_buffer[i] ← gamma_correct_byte(linear_heat)
 *
 *   The cold sentinel lets pass 2 distinguish "draw a glyph", "erase a
 *   stale glyph", and "do nothing" without re-reading the heat field.
 */
static void pipeline_pass1_gamma_correct(Bitmap *b)
{
    int                  total_cells   = b->cols * b->rows;
    const unsigned char *heat_bitmap   = b->bmap;
    float               *dither_buffer = b->dither;

    for (int i = 0; i < total_cells; i++) {
        unsigned char linear_heat = heat_bitmap[i];
        if (linear_heat == 0) {
            dither_buffer[i] = COLD_SENTINEL;
            continue;
        }
        dither_buffer[i] = gamma_correct_byte(linear_heat);
    }
}

/*
 * pipeline_diffuse_quant_error() — push one cell's rounding error to neighbours.
 *
 *   Floyd-Steinberg distribution (cold-tagged neighbours skipped):
 *
 *      ── current ─────► 7/16 → (x+1, y)
 *      3/16 → (x-1, y+1) ── 5/16 → (x, y+1) ── 1/16 → (x+1, y+1)
 */
static void pipeline_diffuse_quant_error(float *dither_buffer, int cols, int rows,
                                         int x, int y, float quant_error)
{
    int i = y * cols + x;

    if (x + 1 < cols && dither_buffer[i + 1] >= 0.f)
        dither_buffer[i + 1] += quant_error * (7.f / 16.f);

    if (y + 1 < rows) {
        if (x - 1 >= 0 && dither_buffer[i + cols - 1] >= 0.f)
            dither_buffer[i + cols - 1] += quant_error * (3.f / 16.f);
        if (dither_buffer[i + cols] >= 0.f)
            dither_buffer[i + cols]     += quant_error * (5.f / 16.f);
        if (x + 1 < cols && dither_buffer[i + cols + 1] >= 0.f)
            dither_buffer[i + cols + 1] += quant_error * (1.f / 16.f);
    }
}

/*
 * pipeline_draw_lit_cell() — quantise one cell and draw its glyph.
 *
 *   bucket           ← lut_index(perceptual)
 *   bucket_midpoint  ← lut_midpoint(bucket)
 *   quant_error      ← perceptual − bucket_midpoint
 *   diffuse quant_error to forward neighbours (Floyd-Steinberg)
 *   draw k_ramp[bucket] with the theme's ramp attribute
 */
static void pipeline_draw_lit_cell(Bitmap *b, int x, int y, float perceptual)
{
    int   bucket          = lut_index(perceptual);
    float bucket_midpoint = lut_midpoint(bucket);
    float quant_error     = perceptual - bucket_midpoint;

    pipeline_diffuse_quant_error(b->dither, b->cols, b->rows, x, y, quant_error);

    attr_t glyph_attr = ramp_attr(bucket, b->theme);
    attron(glyph_attr);
    mvaddch(y, x, (chtype)(unsigned char)k_ramp[bucket]);
    attroff(glyph_attr);
}

/*
 * pipeline_pass2_quantise_and_draw() — walk dither_buffer, draw or erase.
 *
 *   For every cell visible to the terminal:
 *      cold sentinel  → if previous frame was lit: write a single ' '
 *      lit            → pipeline_draw_lit_cell()
 */
static void pipeline_pass2_quantise_and_draw(Bitmap *b, int tcols, int trows)
{
    int                  cols          = b->cols;
    int                  rows          = b->rows;
    const float         *dither_buffer = b->dither;
    const unsigned char *previous_heat = b->prev;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (x >= tcols || y >= trows) continue;

            int   i           = y * cols + x;
            float perceptual  = dither_buffer[i];

            if (perceptual < 0.f) {
                bool was_lit_last_frame = (previous_heat[i] > 0);
                if (was_lit_last_frame) mvaddch(y, x, ' ');
                continue;
            }

            pipeline_draw_lit_cell(b, x, y, perceptual);
        }
    }
}

/*
 * pipeline_pass3_archive_current_frame() — record current heat as previous.
 *
 *   Next frame's pass 2 will compare prev[i] > 0 against the new heat to
 *   decide whether a cold cell needs a literal ' ' erase.
 */
static void pipeline_pass3_archive_current_frame(Bitmap *b)
{
    int total_cells = b->cols * b->rows;
    memcpy(b->prev, b->bmap, (size_t)total_cells * sizeof(unsigned char));
}

/*
 * bitmap_draw() — orchestrate the three render passes.
 *
 *   pass 1   gamma-correct heat into the dither buffer
 *   pass 2   quantise + dither + draw (or erase stale cells)
 *   pass 3   archive current heat so next frame can detect changes
 */
static void bitmap_draw(Bitmap *b, int tcols, int trows)
{
    pipeline_pass1_gamma_correct(b);
    pipeline_pass2_quantise_and_draw(b, tcols, trows);
    pipeline_pass3_archive_current_frame(b);
}

/* ===================================================================== */
/* §11  scene — orchestrator over Bitmap, pause, debug mode               */
/* ===================================================================== */

/*
 * §11 PREAMBLE — the orchestrator
 * ───────────────────────────────
 *
 * Scene owns the Bitmap plus three small scalars:
 *   paused        — set by space; freezes scene_tick.
 *   needs_clear   — set when the screen content can no longer be
 *                   diff-cleared (theme cycle, resize, debug-mode
 *                   change) so the next frame does a full erase().
 *   debug_mode    — which render path scene_draw dispatches to
 *                   (see §12 for the alternatives).
 *
 * The split keeps scene_tick small (one branch) and lets scene_draw
 * be a 4-line table of contents that picks the render path.
 */

/*
 * DebugMode — which render path scene_draw uses.
 *
 *   DEBUG_OFF        normal gamma + dither + LUT pipeline (§10).
 *   DEBUG_RAW        each cell shows the raw heat byte in hex (0-F).
 *   DEBUG_FUEL       paint only the bottom 2 fuel rows; everything
 *                    else is left as the previous frame so the user
 *                    sees the arch sweep + ±2 walk in isolation.
 *   DEBUG_NODITHER   gamma + LUT + paint, no Floyd-Steinberg.  Lets
 *                    the user see what the dither removes.
 *
 *   DEBUG_COUNT is the sentinel for the d/D cycle.
 */
typedef enum {
    DEBUG_OFF      = 0,
    DEBUG_RAW      = 1,
    DEBUG_FUEL     = 2,
    DEBUG_NODITHER = 3,
    DEBUG_COUNT    = 4,
} DebugMode;

static const char *debug_mode_name(DebugMode m)
{
    switch (m) {
    case DEBUG_OFF:      return "off     ";
    case DEBUG_RAW:      return "raw heat";
    case DEBUG_FUEL:     return "fuel    ";
    case DEBUG_NODITHER: return "nodither";
    default:             return "?       ";
    }
}

typedef struct {
    Bitmap    bmap;
    bool      paused;
    bool      needs_clear;
    DebugMode debug_mode;
} Scene;

/* Forward declaration: §12 owns the debug render functions. */
static void scene_draw_debug(Scene *s, int cols, int rows);

static void scene_init(Scene *s, int cols, int rows, int theme)
{
    memset(s, 0, sizeof *s);
    bitmap_init(&s->bmap, cols, rows, theme);
    s->debug_mode = DEBUG_OFF;
}

static void scene_free(Scene *s) { bitmap_free(&s->bmap); }

static void scene_resize(Scene *s, int cols, int rows)
{
    int       t          = s->bmap.theme;
    float     fuel       = s->bmap.fuel;
    DebugMode debug_mode = s->debug_mode;

    bitmap_free(&s->bmap);
    bitmap_init(&s->bmap, cols, rows, t);

    s->bmap.fuel   = fuel;
    s->debug_mode  = debug_mode;
    s->needs_clear = true;
    gentable(&s->bmap);   /* rebuild decay table for new rows */
}

/*
 * scene_cycle_debug_mode() — advance d/D selector and trigger a clear.
 *
 *   step = +1 for `d`, −1 for `D`.  The previous render path may have
 *   left glyphs the next path will not overpaint (e.g. DEBUG_FUEL leaves
 *   the top of the screen untouched), so we force a needs_clear.
 */
static void scene_cycle_debug_mode(Scene *s, int step)
{
    int next = ((int)s->debug_mode + step + DEBUG_COUNT) % DEBUG_COUNT;
    s->debug_mode  = (DebugMode)next;
    s->needs_clear = true;
}

static void scene_tick(Scene *s)
{
    if (!s->paused) {
        if (bitmap_tick(&s->bmap))
            s->needs_clear = true;
    }
}

/*
 * scene_draw() — dispatch to the active render path.
 *
 *   DEBUG_OFF → §10 bitmap_draw() (the full pipeline).
 *   Anything else → §12 scene_draw_debug() (alternative renders).
 */
static void scene_draw(Scene *s, int cols, int rows)
{
    if (s->debug_mode == DEBUG_OFF)
        bitmap_draw(&s->bmap, cols, rows);
    else
        scene_draw_debug(s, cols, rows);
}

/* ===================================================================== */
/* §12  debug overlay — alternative render modes for inspection           */
/* ===================================================================== */

/*
 * §12 PREAMBLE — three windows into the CA
 * ────────────────────────────────────────
 *
 * The full §10 pipeline (gamma → dither → LUT → paint) is so well-
 * tuned that the underlying state is hard to see.  This section
 * provides three alternative renders that bypass one stage each:
 *
 *   DEBUG_RAW       skip the entire pipeline.  Show the raw heat
 *                   byte in hex (0..F).  Reveals the actual CA
 *                   state without ANY post-processing.
 *
 *   DEBUG_FUEL      paint ONLY the two fuel rows at the bottom.
 *                   The rest of the screen keeps the previous
 *                   frame so the reader can watch the arch sweep
 *                   + ±2 walk in isolation.
 *
 *   DEBUG_NODITHER  run gamma + LUT + paint but skip the Floyd-
 *                   Steinberg error diffusion.  Visible banding
 *                   returns; contrast against DEBUG_OFF to
 *                   appreciate what dithering buys.
 *
 * Each branch is a small dedicated function so the dispatch in
 * scene_draw_debug() reads as a 3-way switch with no inline logic.
 */

/*
 * hex_digit_for_heat() — uint8 [0, 255] → ASCII hex digit '0'..'F'.
 *
 *   Maps the top nibble: 0..15 → '0'..'9', 'A'..'F'.  Bucket size 16
 *   gives a fair-coverage display where every hex digit covers
 *   the same heat range.
 */
static inline char hex_digit_for_heat(unsigned char heat)
{
    static const char k_hex[] = "0123456789ABCDEF";
    return k_hex[heat >> 4];
}

/*
 * debug_render_raw_heat() — print the raw heat byte as a hex digit.
 *
 *   Loops the visible region.  For every cell:
 *      glyph = hex_digit_for_heat(bmap[i])
 *      paint with the theme's ramp pair for bucket (heat >> 5)
 *      (so colour still tracks heat — only glyph + skip-dither change)
 */
static void debug_render_raw_heat(Scene *s, int tcols, int trows)
{
    Bitmap *b = &s->bmap;
    int     cols = b->cols;
    int     rows = b->rows;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (x >= tcols || y >= trows) continue;
            unsigned char heat = b->bmap[y * cols + x];

            int    bucket = heat >> 5;            /* 0..7 from top 3 bits */
            if (bucket >= RAMP_N) bucket = RAMP_N - 1;
            char   glyph  = hex_digit_for_heat(heat);
            attr_t a      = ramp_attr(bucket, b->theme);

            attron(a);
            mvaddch(y, x, (chtype)(unsigned char)glyph);
            attroff(a);
        }
    }
}

/*
 * debug_render_fuel_only() — paint only the bottom 2 fuel rows.
 *
 *   Erases the visible region first (so the previous render's flame
 *   is gone), then paints rows y = rows..rows+1 OF THE BITMAP at
 *   screen rows rows-2 and rows-1.  Each fuel cell renders through
 *   the normal LUT so the colour tracks the seed value.
 */
static void debug_render_fuel_only(Scene *s, int tcols, int trows)
{
    Bitmap *b    = &s->bmap;
    int     cols = b->cols;
    int     rows = b->rows;

    erase();

    for (int row_offset = 0; row_offset < 2; row_offset++) {
        int fuel_row    = rows + row_offset;             /* bmap row */
        int screen_row  = rows - 2 + row_offset;          /* screen row */
        if (screen_row < 0 || screen_row >= trows) continue;

        for (int x = 0; x < cols && x < tcols; x++) {
            unsigned char heat = b->bmap[fuel_row * cols + x];
            float perceptual   = (heat == 0) ? 0.f : gamma_correct_byte(heat);
            int   bucket       = lut_index(perceptual);
            char  glyph        = k_ramp[bucket];
            attr_t a           = ramp_attr(bucket, b->theme);
            attron(a);
            mvaddch(screen_row, x, (chtype)(unsigned char)glyph);
            attroff(a);
        }
    }
}

/*
 * debug_render_no_dither() — gamma + LUT + paint, NO Floyd-Steinberg.
 *
 *   Per cell:
 *      perceptual = gamma_correct_byte(heat)
 *      bucket     = lut_index(perceptual)
 *      paint k_ramp[bucket]
 *
 *   Cold cells erase a stale glyph the same way pass 2 does, so
 *   diff-clear still works inside this mode.
 */
static void debug_render_no_dither(Scene *s, int tcols, int trows)
{
    Bitmap *b    = &s->bmap;
    int     cols = b->cols;
    int     rows = b->rows;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (x >= tcols || y >= trows) continue;
            int i = y * cols + x;

            unsigned char heat = b->bmap[i];
            if (heat == 0) {
                if (b->prev[i] > 0) mvaddch(y, x, ' ');
                continue;
            }
            float  perceptual = gamma_correct_byte(heat);
            int    bucket     = lut_index(perceptual);
            char   glyph      = k_ramp[bucket];
            attr_t a          = ramp_attr(bucket, b->theme);
            attron(a);
            mvaddch(y, x, (chtype)(unsigned char)glyph);
            attroff(a);
        }
    }
    memcpy(b->prev, b->bmap, (size_t)(cols * rows) * sizeof(unsigned char));
}

/*
 * scene_draw_debug() — three-way dispatch on debug_mode.
 *
 *   The DEBUG_OFF branch never gets here (scene_draw filters it out).
 *   Default-case fall-through reuses the full pipeline as a safety
 *   net should DEBUG_COUNT ever grow without a matching case.
 */
static void scene_draw_debug(Scene *s, int cols, int rows)
{
    switch (s->debug_mode) {
    case DEBUG_RAW:      debug_render_raw_heat (s, cols, rows); break;
    case DEBUG_FUEL:     debug_render_fuel_only(s, cols, rows); break;
    case DEBUG_NODITHER: debug_render_no_dither(s, cols, rows); break;
    default:             bitmap_draw(&s->bmap, cols, rows);     break;
    }
}

/* ===================================================================== */
/* §13  screen + HUD                                                      */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols); }

/*
 * screen_draw_hud_top_right() — bright yellow status strip on row 0.
 *
 *   Shows fps · theme · fuel · debug-mode · sim-Hz.  PAIR_HUD is
 *   bound once in color_init() and is theme-independent, so the
 *   strip stays legible no matter which palette is active.
 */
static void screen_draw_hud_top_right(const Screen *s, const Scene *sc,
                                      double fps, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %4.1f fps  [%s]  fuel:%.2f  dbg:%s  sim:%d ",
             fps,
             k_themes[sc->bmap.theme].name,
             sc->bmap.fuel,
             debug_mode_name(sc->debug_mode),
             sim_fps);

    int hud_x = s->cols - (int)strlen(buf);
    if (hud_x < 0) hud_x = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/*
 * screen_draw_hint_bottom() — bright cyan key strip on the last row.
 *
 *   Lists every interactive key.  Same theme-independence story as
 *   the top HUD.  Keep this in one line so it survives narrow
 *   terminals; the strip simply right-truncates if cols is small.
 */
static void screen_draw_hint_bottom(const Screen *s)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q/ESC:quit  space:pause  t/T:theme  d/D:debug  g/G:fuel  ]/[:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_draw() — full-frame composite: clear (if needed), CA, HUD strips.
 *
 *   Step 1 — honour scene's needs_clear flag (set by theme cycle, resize,
 *            debug-mode switch).
 *   Step 2 — paint the CA via scene_draw (dispatches on debug_mode).
 *   Step 3 — paint the top HUD strip.
 *   Step 4 — paint the bottom hint strip.
 */
static void screen_draw(Screen *s, Scene *sc, double fps, int sim_fps)
{
    if (sc->needs_clear) {
        erase();
        sc->needs_clear = false;
    }
    scene_draw(sc, s->cols, s->rows);
    screen_draw_hud_top_right(s, sc, fps, sim_fps);
    screen_draw_hint_bottom (s);
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §14  app — signals + key dispatch                                      */
/* ===================================================================== */

/*
 * §14 PREAMBLE — boilerplate, slightly named
 * ──────────────────────────────────────────
 *
 * Signal-safe flag flips (running, need_resize), a small App struct
 * that bundles scene + screen + sim_fps, and the key handler.  Each
 * non-trivial mutation in the key switch delegates to a named helper
 * (app_cycle_theme, app_nudge_fuel, app_nudge_sim_fps,
 * scene_cycle_debug_mode) so the switch reads as one line per
 * behaviour.
 */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int s){ (void)s; g_app.running=0; }
static void on_resize(int s){ (void)s; g_app.need_resize=1; }
static void cleanup  (void) { endwin(); }

/*
 * app_cycle_theme() — step the theme index by ±1 and rebind ramp pairs.
 *
 *   step = +1 for `t`, −1 for `T`.  Reset the auto-cycle counter so
 *   the user's manual choice has a full CYCLE_TICKS before auto-advance.
 *   needs_clear so the previous palette's diff-clear can't bleed
 *   colour into the new palette.
 */
static void app_cycle_theme(App *a, int step)
{
    Bitmap *b = &a->scene.bmap;
    int next = (b->theme + step + THEME_COUNT) % THEME_COUNT;
    b->theme       = next;
    b->cycle_tick  = 0;
    theme_apply(next);
    a->scene.needs_clear = true;
}

/*
 * app_nudge_fuel() — bump fuel intensity by signed step, clamped to
 *                    [FUEL_MIN, FUEL_MAX].
 */
static void app_nudge_fuel(App *a, float step)
{
    Bitmap *b = &a->scene.bmap;
    float v = b->fuel + step;
    if (v > FUEL_MAX) v = FUEL_MAX;
    if (v < FUEL_MIN) v = FUEL_MIN;
    b->fuel = v;
}

/*
 * app_nudge_sim_fps() — bump simulation Hz by signed step, clamped.
 */
static void app_nudge_sim_fps(App *a, int step)
{
    int v = a->sim_fps + step;
    if (v > SIM_FPS_MAX) v = SIM_FPS_MAX;
    if (v < SIM_FPS_MIN) v = SIM_FPS_MIN;
    a->sim_fps = v;
}

/*
 * app_handle_key() — dispatch one keystroke.
 *
 *   Returns true to keep running, false to quit (q / Q / ESC).  All
 *   non-trivial mutations delegate to a named helper so the switch
 *   reads as one line per behaviour.
 */
static bool app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:  return false;
    case ' ':  a->scene.paused = !a->scene.paused;        break;

    case 't':  app_cycle_theme       (a, +1);             break;
    case 'T':  app_cycle_theme       (a, -1);             break;

    case 'd':  scene_cycle_debug_mode(&a->scene, +1);     break;
    case 'D':  scene_cycle_debug_mode(&a->scene, -1);     break;

    case 'g':  app_nudge_fuel        (a, +FUEL_STEP);     break;
    case 'G':  app_nudge_fuel        (a, -FUEL_STEP);     break;

    case ']':  app_nudge_sim_fps     (a, +SIM_FPS_STEP);  break;
    case '[':  app_nudge_sim_fps     (a, -SIM_FPS_STEP);  break;

    default:                                              break;
    }
    return true;
}

/* ===================================================================== */
/* §15  main — fixed-step loop                                            */
/* ===================================================================== */

/*
 * §15 PREAMBLE — wire it all up
 * ─────────────────────────────
 *
 * Standard framework pattern from CLAUDE.md:
 *   1. install signal handlers + atexit
 *   2. screen_init → scene_init
 *   3. while running:
 *        if need_resize: rebuild screen + scene
 *        clock_ns delta → sim_accum += dt
 *        drain sim_accum into scene_tick at the configured Hz
 *        update fps display every FPS_UPDATE_MS
 *        sleep for the remaining wall-clock budget
 *        screen_draw → screen_present
 *        getch → app_handle_key
 */
int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,on_exit); signal(SIGTERM,on_exit); signal(SIGWINCH,on_resize);

    App *app  = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0);

    int64_t ft=clock_ns(), sa=0, fa=0; int fc=0; double fpsd=0.;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            ft = clock_ns(); sa = 0;
        }

        int64_t now=clock_ns(), dt=now-ft; ft=now;
        if (dt > 100*NS_PER_MS) dt = 100*NS_PER_MS;

        int64_t tick = TICK_NS(app->sim_fps);
        sa += dt;
        while (sa >= tick) { scene_tick(&app->scene); sa -= tick; }
        float alpha = (float)sa / (float)tick;
        (void)alpha;

        fc++; fa += dt;
        if (fa >= FPS_UPDATE_MS*NS_PER_MS) {
            fpsd = (double)fc / ((double)fa/(double)NS_PER_SEC);
            fc=0; fa=0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t el = clock_ns()-ft+dt;
        clock_sleep_ns(NS_PER_SEC/60 - el);

        screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
