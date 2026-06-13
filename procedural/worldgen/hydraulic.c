/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hydraulic.c
 *   — Hydraulic erosion on a procedural heightmap. Generate fractal
 *     terrain with Perlin fBm; let thousands of water droplets flow
 *     down it, eroding the ground where they accelerate and
 *     depositing sediment where they slow. Watch the smooth Perlin
 *     hills sprout valleys, deltas, and river networks in real time.
 *
 * DEMO: A fresh, smooth heightmap appears — rolling Perlin hills with
 *       no rivers and no detail. Within a second, water droplets
 *       start flowing across it. As they fall through the height
 *       field they pick up sediment from steep slopes and drop it
 *       on flat ground; as more droplets follow the same path the
 *       slope deepens and a CHANNEL forms. After ~10 seconds the
 *       initial blobby hills have been carved into a recognisable
 *       fluvial landscape: dendritic river networks running down
 *       to broad, silt-filled deltas. Cycle fifteen views with n / p;
 *       the four analytic ones:
 *
 *         TERRAIN    biome map of the current heightmap (deep ocean
 *                    → sea → coast → plains → hills → mountains →
 *                    highlands → peaks)
 *         DROPLETS   recent water-flow visualisation: cells that a
 *                    droplet visited recently glow bright cyan,
 *                    fading over ~1 second.  Reveals the
 *                    self-organising drainage network in real time.
 *         EROSION    cut/fill diff vs the original heightmap — red
 *                    '-' where the ground was lowered, blue '+'
 *                    where sediment was deposited.
 *         SLOPE      gradient-magnitude heatmap with arrow glyphs
 *                    pointing downhill (the direction water flows).
 *
 *       The other eleven (RELIEF, CONTOUR, HYPSO, ASPECT, RIDGES,
 *       CANYONS, BATHY, PLASMA, NIGHT, FLOW, HEAT) layer hillshading,
 *       contours, structure detection and animation on the SAME three
 *       arrays — see the Pattern enum in §1 for a one-line each.
 *
 *       After ~8 000 droplets the simulation holds for a few
 *       seconds, then regenerates on a fresh map and the cycle
 *       repeats.
 *
 * Study alongside:
 *   ../worldgen/tectonic.c
 *      — same biome ramp and HUD scaffolding, but generates terrain
 *        from plate-tectonic CONSTRAINTS rather than from
 *        post-process erosion of noise. Tectonic builds geology
 *        upward; this file carves it downward.
 *   ../fields/perin_noise_flow_showcase.c
 *      — particles being steered by a noise field. Same particle-
 *        in-a-field idea; here the field is the heightmap gradient
 *        rather than a Perlin angle.
 *
 * Section map (re-cut by CONCERN — see ARCHITECTURE block below):
 *   §1 CONFIG       — constants, data tables, core state types
 *   §2 PERFORMANCE  — timing primitives (throttle policy lives in main)
 *   §3 LOGIC        — pure decisions: no mutation, no I/O
 *   §4 SIMULATION   — advances state (the only writers of sim state)
 *   §5 EFFECTS      — cosmetic-only state (one-line note: it's woven in)
 *   §6 DELAYS       — pauses, holds, timers (one-line note: woven in)
 *   §7 RENDER       — state → screen; reads only, never mutates
 *   §8 APP          — user events + per-tick combine + main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          regenerate fresh terrain + restart erosion
 *   n / N      next pattern  (cycles all 15: TERRAIN, DROPLETS,
 *              EROSION, SLOPE, RELIEF, CONTOUR, HYPSO, ASPECT,
 *              RIDGES, CANYONS, BATHY, PLASMA, NIGHT, FLOW, HEAT)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      more droplets per tick (faster erosion)
 *   -          fewer droplets
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/hydraulic.c \
 *     -o hydraulic -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle-based hydraulic erosion (Beyer 2015 / the
 *                  Sebastian Lague tutorial implementation). Each
 *                  droplet is a tiny Lagrangian particle that walks
 *                  the heightmap one cell at a time, carrying a
 *                  scalar SEDIMENT load. At every step:
 *
 *                  (1) GRADIENT — bilinear-sample the four corners of
 *                      the current cell to compute (∂h/∂x, ∂h/∂y).
 *                      The droplet's velocity vector is:
 *                          v ← inertia·v_old − (1−inertia)·∇h
 *                      then normalised to unit length. This makes
 *                      droplets prefer to keep going in the direction
 *                      they were already going (so they cut straight
 *                      channels, not chaotic zig-zags) while still
 *                      following the terrain's local slope.
 *
 *                  (2) STEP — advance one cell in the velocity
 *                      direction; bilinear-sample the new height;
 *                      compute Δh = h_new − h_old.
 *
 *                  (3) CARRYING CAPACITY — a droplet moving fast
 *                      down a steep slope can carry more sediment
 *                      than one trickling across a plain:
 *                          C = max(−Δh · speed · water · K,  C_min)
 *
 *                  (4) ERODE / DEPOSIT — compare carried sediment to
 *                      capacity:
 *                        if (sed > C  ||  Δh > 0):
 *                          DEPOSIT — drop (sed − C)·rate at the
 *                          current cell (split bilinearly across
 *                          the four-corner footprint). Going UPHILL
 *                          (Δh > 0) always deposits, capped by Δh.
 *                        else:
 *                          ERODE — pull up (C − sed)·rate, capped
 *                          by |Δh|, distributed across a small disc
 *                          (BRUSH) of cells around the droplet so
 *                          the carving is smooth, not pixel-wide.
 *
 *                  (5) ENERGETICS — a droplet going downhill
 *                      gains kinetic energy:
 *                          v² ← max(0, v² − Δh·g)
 *                      and water evaporates a little each step:
 *                          water ← water · (1 − e_rate)
 *
 *                  (6) TERMINATE after MAX_STEPS or when the droplet
 *                      walks off the map. Spawn a new one. Repeat
 *                      thousands of times.
 *
 *                  The erosion BRUSH (a disc of weighted cells, not a
 *                  single point) is what makes this method look so
 *                  good — a single-cell erosion produces 1-pixel
 *                  zig-zag canyons; a disc-weighted erosion produces
 *                  smoothly-rounded valleys.
 *
 * Data-structure : Three flat float arrays per cell —
 *                    height[]     current eroded heightmap
 *                    initial[]    pre-erosion copy (for cut/fill diff)
 *                    water_trail[] decaying counter; raised to 1.0
 *                                  every time a droplet visits a cell;
 *                                  multiplied by WATER_TRAIL_DECAY
 *                                  each tick. Drives the DROPLETS
 *                                  pattern's flow visualisation.
 *                  Plus a single transient Droplet struct per
 *                  simulated droplet — created on the stack, run to
 *                  completion, discarded. No per-droplet allocation.
 *
 * Rendering      : ASCII only.  Each pattern dispatches per cell:
 *                    TERRAIN   biome glyph by elevation bucket.
 *                    DROPLETS  terrain dimmed + bright '~' wherever
 *                              water_trail > threshold; trails fade
 *                              over ~1 s exposing the drainage net.
 *                    EROSION   '-' (red) where height < initial,
 *                              '+' (blue) where height > initial,
 *                              dim biome where unchanged.
 *                    SLOPE     gradient magnitude → ramp index;
 *                              direction of -∇h → arrow glyph.
 *
 * Performance    : Per droplet: MAX_STEPS = 32 iterations, each doing
 *                  ~30 ops (gradient + brush + bookkeeping). 32 ×
 *                  30 = ~1 K ops. At default speed ~12 droplets/tick
 *                  × 60 ticks = 720/sec ≈ 720 K ops/sec — under 1 %
 *                  of a current CPU. Per-frame render is O(W·H).
 *
 * References     :
 *
 *   Erosion algorithm —
 *   • Beyer, H. (2015) — "Implementation of a method for hydraulic
 *     erosion", Bachelor thesis (TU München). The exact droplet model
 *     this file implements (capacity, erode/deposit, brush).
 *     https://www.firespark.de/resources/downloads/implementation%20of%20a%20methode%20for%20hydraulic%20erosion.pdf
 *   • Lague, S. (2019) — "Coding Adventure: Hydraulic Erosion".
 *     The 20-minute video walkthrough of the same model.
 *     https://www.youtube.com/watch?v=eaXk97ujbPQ
 *   • Mei, X., Decaudin, P., Hu, B-G. (2007) — "Fast Hydraulic
 *     Erosion Simulation and Visualization on GPU", Pacific Graphics.
 *     The grid-based "virtual pipes" alternative.
 *   • Perlin, K. (2002) — "Improving Noise" (the fBm scaffold in §5).
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *
 *   Rendering & cartographic visualization (§7 presets) —
 *   • Imhof, E. (1982) — "Cartographic Relief Presentation" (ESRI Press
 *     reprint 2007). The canonical book on hillshading, hypsometric
 *     tints and contour lines → RELIEF, HYPSO, CONTOUR, FLOW.
 *   • Horn, B.K.P. (1981) — "Hill Shading and the Reflectance Map",
 *     Proc. IEEE 69(1):14-47. The Lambert-from-gradient hillshade in
 *     cell_shade() → RELIEF, CANYONS, BATHY, NIGHT.
 *     https://doi.org/10.1109/PROC.1981.11918
 *   • Wilson, J.P. & Gallant, J.C. (2000) — "Terrain Analysis:
 *     Principles and Applications" (Wiley). Slope, aspect and
 *     curvature from a DEM → SLOPE, ASPECT, RIDGES.
 *   • Bourke, P. (1997) — "Character representation of grey scale
 *     images". The luminance→ASCII density ramp behind ELEV_RAMP.
 *     http://paulbourke.net/dataformats/asciiart/
 *   • Vandevenne, L. — "Lode's Computer Graphics Tutorial: Plasma".
 *     Sum-of-sines colour/height fields → PLASMA, BATHY caustics.
 *     https://lodev.org/cgtutor/plasma.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To carve a realistic landscape, do not draw the rivers — let them
 * draw themselves. Drop a tiny pebble of water onto a smooth height
 * field; gravity pulls it downhill; whenever the slope steepens it
 * digs out a bit of dirt; whenever the slope flattens it drops the
 * dirt back. Repeat with thousands of pebbles. The first ones cut
 * shallow scratches; later ones, biased to follow the existing
 * scratches because those are the steepest paths, deepen them into
 * channels. The channels merge into rivers, the rivers carve valleys,
 * the valleys break into deltas at the lowlands. Nobody designed
 * any of it — every feature is a consequence of "water flows down
 * and carries dirt".
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a sheet of putty. Sprinkle marbles on top. Each marble:
 *   - Rolls in the direction of steepest descent (gravity).
 *   - Has a little scoop attached underneath.
 *   - When rolling fast (steep slope), the scoop digs a little
 *     groove into the putty.
 *   - When rolling slow (flat), the scoop is full and dribbles its
 *     load back out, raising the putty.
 *   - When it leaves the table, it disappears and a new marble is
 *     placed somewhere random.
 *
 * That is the algorithm. The putty is the heightmap, the marbles are
 * the droplets, the scoop is the carrying capacity formula, and the
 * grooves are the rivers. The marbles do not know about each other,
 * but they collectively build the drainage network because each
 * marble is steered by the grooves left by previous marbles.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INITIALISE the heightmap with Perlin fBm (4 octaves of noise
 *     summed at halved amplitude / doubled frequency). Save a copy
 *     into initial[] for the cut/fill diff later.
 *
 *  2. SPAWN a droplet at a random cell with:
 *       v = (0, 0)           — direction
 *       speed = 1
 *       water = 1
 *       sediment = 0
 *
 *  3. STEP the droplet up to MAX_STEPS times:
 *     a. xi, yi = floor(droplet.x, droplet.y)
 *     b. h00, h10, h01, h11 = height of 4 surrounding cells
 *     c. ∇h = ((h10 − h00)(1 − fy) + (h11 − h01)·fy,
 *              (h01 − h00)(1 − fx) + (h11 − h10)·fx)
 *     d. v ← inertia·v − (1 − inertia)·∇h    [steering]
 *     e. v ← v / |v|                          [unit length]
 *     f. new_pos ← pos + v.  Sample h_new bilinearly.
 *     g. Δh = h_new − h_old.
 *     h. C = max(−Δh·speed·water·K, C_min).
 *     i. if (sediment > C || Δh > 0):
 *          DEPOSIT  — split (sediment−C)·rate over the four corners
 *                     of the current cell.
 *        else:
 *          ERODE    — pull up min((C−sediment)·rate, |Δh|) using a
 *                     disc-weighted brush of radius BRUSH_R.
 *     j. v² ← max(0, v² − Δh·g);  water ← water·(1 − e_rate).
 *     k. pos ← new_pos.
 *
 *  4. After a budget DROPLETS_PER_GEN of droplets, hold for
 *     HOLD_SECONDS and then go back to step 1 with a new seed.
 *
 * KEY FORMULAS
 * ────────────
 *  Bilinear height at fractional (x, y):
 *    fx, fy = x − ⌊x⌋, y − ⌊y⌋
 *    h = (h00·(1−fx) + h10·fx)·(1−fy) + (h01·(1−fx) + h11·fx)·fy
 *
 *  Bilinear gradient at the same point:
 *    ∂h/∂x = (h10 − h00)·(1 − fy) + (h11 − h01)·fy
 *    ∂h/∂y = (h01 − h00)·(1 − fx) + (h11 − h10)·fx
 *
 *  Carrying capacity (sediment a droplet CAN hold):
 *    C = max(−Δh · speed · water · CAPACITY_K,  C_min)
 *
 *  Erosion brush (disc-weighted; ω(d) = 1 − d/R for d ≤ R):
 *    total_w = Σ_disc ω(d_i)
 *    height_i ← height_i − amount · ω(d_i) / total_w     for i in disc
 *
 *  Energetics:
 *    speed² ← max(0, speed² − Δh · gravity)
 *    water  ← water · (1 − evaporate_rate)
 *
 *  Biome buckets (same as ../worldgen/tectonic.c):
 *    e<0.15 DEEP_OCEAN  | <0.30 OCEAN   | <0.40 COAST
 *    | <0.50 PLAINS     | <0.62 HILLS   | <0.75 MOUNTAINS
 *    | <0.85 HIGHLANDS  | else PEAKS
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SINGLE-CELL EROSION = ZIG-ZAGS. If you erode JUST the cell the
 *    droplet is in, every droplet cuts a 1-pixel channel and the
 *    terrain develops a stripey checkerboard look. Always erode
 *    over a SMALL DISC (BRUSH_R = 2 cells) — the resulting valleys
 *    are smoothly rounded.
 *
 *  • UNNORMALISED VELOCITY. After v ← inertia·v − (1−inertia)·∇h
 *    the magnitude drifts; un-normalised, droplets in flat regions
 *    crawl to a halt and never leave. Always normalise to unit
 *    length so the droplet keeps moving until it walks off the map.
 *
 *  • UPHILL SAFETY. If a droplet steers uphill (Δh > 0) it must
 *    always deposit, capped at Δh, even if the capacity test would
 *    say "erode" — otherwise droplets dig holes UPHILL of where
 *    they came from, which is unphysical and visually awful.
 *
 *  • LIMIT EROSION TO Δh. Without this guard, a steep cell + a
 *    near-empty droplet can pull up more height than the slope
 *    actually offers, producing pits that water can never escape.
 *    Cap erode_amount at |Δh|.
 *
 *  • EVAPORATION VS CAPACITY. As water evaporates the carrying
 *    capacity drops; eventually the droplet must deposit anything
 *    it's carrying. This is what produces deltas at the foot of
 *    rivers — the slow, low-water section drops every grain it had.
 *    Don't make EVAPORATE_RATE too small or droplets carry sediment
 *    to the edge of the map; don't make it too large or rivers
 *    deposit before they reach the lowlands.
 *
 *  • BOUNDARY HANDLING. Bilinear sampling reads four corners
 *    (xi, yi)..(xi+1, yi+1). On the right and bottom edges the
 *    +1 index is out of bounds; terminate the droplet rather than
 *    reading garbage. The simulation still produces erosion all the
 *    way to the edge because earlier steps reach there.
 *
 *  • TRAIL DECAY. WATER_TRAIL_DECAY is per-TICK, not per-second; if
 *    you raise the sim_fps, decay-per-second changes proportionally.
 *    For a fixed visual decay length, multiply by exp(-K·dt) instead
 *    of a constant. Here we accept the tick-coupling for simplicity.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Heightmap and trails freeze. Resume: simulation
 *    continues from exactly where it stopped. Verifies the fixed-
 *    step accumulator.
 *
 *  • Switch to EROSION on a FRESH world. The map is uniform "no
 *    change" (no red, no blue) — initial == current. Watch for a
 *    few seconds; red lines appear (erosion in valleys) and blue
 *    blobs appear at the lowlands (deposition in deltas). The two
 *    should roughly balance — total cut ≈ total fill.
 *
 *  • Switch to DROPLETS during active erosion. You should see a
 *    glowing dendritic NETWORK of channels — recently-walked cells
 *    light up and fade. The network should match the valleys
 *    visible in TERRAIN — the channels follow the same valleys
 *    that have eroded.
 *
 *  • Switch to SLOPE. Steep cells (along ridges and valley walls)
 *    are bright; flat cells (plains and lake bottoms) are dim.
 *    Arrow glyphs at every cell point downhill — water flows from
 *    high to low.
 *
 *  • Press 'r'. The terrain regenerates: the HUD's "droplets" counter
 *    resets to 0, the world is fresh, and erosion starts over.
 *
 *  • Run with 'speed' = 64 (much faster); the simulation reaches
 *    8 000 droplets in a couple of seconds and holds. Drop speed to
 *    1 — droplets crawl and you can WATCH each erosion step
 *    individually as a single bright water trail.
 *
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * This file was re-cut from first principles into separated concern-layers
 * (a SEPARATION pass: RELOCATE + LABEL only — every function body is
 * byte-identical to before, nothing renamed). Layer → section → what it
 * mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time constants + const data tables
 *                    + the Heightmap struct (core state type, relocated up
 *                    so the pure §3 helpers can see it).
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers.
 *                    The frame cap + fixed-timestep accumulator are POLICY
 *                    and live in main() (§8), not here.
 *   LOGIC        §3  nothing — pure reads. Given their args (+ the read-only
 *                    perm[] / PATTERN_NAMES tables) they return a value with
 *                    no mutation and no I/O, so no RENDER/EFFECTS reordering
 *                    can corrupt them: the noise stack, the grid samplers
 *                    (hidx, h_bilinear, h_at, bed_sample, cell_gradient,
 *                    cell_shade, cell_laplacian) and the classifiers/mappers
 *                    (height_to_biome, shade_to_level, sediment_capacity,
 *                    erosion_complete, brush_weight, flow_arrow, contour_glyph,
 *                    downhill_sector, pattern_name).
 *   SIMULATION   §4  Heightmap.{height,initial,water_trail,seed,droplets_done}
 *                    and perm[]; Scene.{time_secs,hold_countdown}. The ONLY
 *                    writers of simulation state.
 *   EFFECTS      §5  (none of its own) — water_trail[] is written/decayed in
 *                    §4; every other glow is derived at render time. One-line
 *                    section; not a real layer here.
 *   DELAYS       §6  (none of its own) — hold_countdown / paused are handled
 *                    inside scene_tick (§4). One-line section.
 *   RENDER       §7  ncurses back buffer + colour-pair table only. Reads
 *                    Scene/Heightmap; never writes simulation state.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     sim_accum += dt                     // PERFORMANCE: bank elapsed time
 *     run_pending_ticks(...)              //   scene_tick() ×N (SIM+EFFECTS+DELAYS)
 *     screen_draw() ; screen_present()    // RENDER (reads only)
 *     getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick() advances simulation state. User events
 * (app_handle_key / app_do_resize) may mutate Scene/Screen in response to a
 * keypress or SIGWINCH, but they are NOT part of the tick — they run once
 * per frame, outside the accumulator loop.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants, data tables, core state types               */
/* ===================================================================== */

enum {
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,
    CELLS_MAX           = MAP_W_MAX * MAP_H_MAX,

    /* Smallest usable map — floor so a tiny terminal still gets a field. */
    MAP_W_MIN           =  16,
    MAP_H_MIN           =   8,
    HUD_TOP_ROWS        =   2,    /* rows 0–1 reserved for the HUD     */

    /* Droplet physics. */
    DROPLET_MAX_STEPS   =  32,
    EROSION_BRUSH_R     =   2,

    /* How many droplets to spawn per simulation tick at SPEED_DEF.
     * Scales linearly with the speed knob. */
    DROPLETS_PER_TICK_DEF =  12,

    /* Total budget per generation cycle. After this many droplets
     * the simulation enters HOLD; once HOLD elapses, regenerate. */
    DROPLETS_PER_GEN    = 8000,
    HOLD_TICKS          = 6 * 60,    /* ~6 s @ 60 fps                  */

    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 elevation tints       */
    PAIR_HOT            =  11,    /* eroded / convergent accent       */
    PAIR_COLD           =  12,    /* deposited / divergent accent     */
    PAIR_WATER          =  13,    /* live water trail                 */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Render frame-rate cap (Hz) — the present rate; the SIMULATION rate is
 * separate (SIM_FPS_*). */
#define FRAME_CAP_FPS   60
/* Clamp one frame's dt so a long stall (process suspended, terminal blocked)
 * can't make the accumulator simulate a huge burst at once — the classic
 * "spiral of death" guard. */
#define MAX_FRAME_NS    (100 * NS_PER_MS)

/* Cell aspect — y in noise / gradient sampling is multiplied by this
 * so terrain features look right on terminals where cells are 2x
 * taller than wide. */
#define ASPECT_Y_F           2.0f

/* fBm — initial heightmap. Lower scale = larger features. */
#define FBM_SCALE_X          0.045f
#define FBM_SCALE_Y          0.090f
#define FBM_OCTAVES          5
/* Apply gamma to fBm output so peaks/valleys are sharper. */
#define FBM_GAMMA            1.30f

/* Droplet physics constants — tuned to the Lague defaults. */
#define INERTIA              0.05f
#define SEDIMENT_CAPACITY_K  4.0f
#define MIN_CAPACITY         0.01f
#define ERODE_RATE           0.30f
#define DEPOSIT_RATE         0.30f
#define EVAPORATE_RATE       0.01f
#define GRAVITY              4.0f
#define INITIAL_WATER        1.0f
#define INITIAL_SPEED        1.0f
/* Below this steering-vector length the droplet has stalled on a flat and
 * its path ends (avoids a divide-by-~0 when normalising the heading). */
#define DROPLET_STALL_LEN    1e-4f

/* Water trail decay per tick (DROPLETS pattern). 0.92 → ~30 ticks ≈
 * 0.5 s visible after a single visit. */
#define WATER_TRAIL_DECAY     0.92f
#define WATER_TRAIL_HIGH      0.55f
#define WATER_TRAIL_MID       0.20f

/* EROSION pattern — minimum |height − initial| to colour a cell. */
#define EROSION_THRESH_LOW    0.012f
#define EROSION_THRESH_HIGH   0.05f

/* SLOPE pattern — slope magnitude to ramp-level mapping factor. */
#define SLOPE_SCALE           18.0f

/* RELIEF / CANYONS / BATHY / NIGHT — Lambert hillshade. Fixed top-left
 * light; gradient is exaggerated so even gentle relief reads as 3-D. */
#define RELIEF_LIGHT_X       -0.55f
#define RELIEF_LIGHT_Y       -0.55f
#define RELIEF_LIGHT_Z        0.63f
#define RELIEF_EXAGGERATE     7.0f

/* CONTOUR / FLOW — elevation quantised into this many height bands; a
 * cell sits on a contour where its band differs from a neighbour. */
#define CONTOUR_LEVELS        16

/* RIDGES — |Laplacian| above this marks a ridge (peak side) or valley. */
#define RIDGE_THRESH          0.006f

/* ── Biome ─────────────────────────────────────────────────────────────── *
 * Eight elevation bands, deep ocean (0) → peaks (7), assigned by
 * height_to_biome() from fixed thresholds on the [0,1] elevation (≈0.30 sea
 * level/coast, ≈0.85 snow line). Intent: a single ordinal that DOUBLES AS AN
 * INDEX — the same biome value offsets into Theme.ramp[], BIOME_GLYPHS[] and
 * the PAIR_RAMP_BASE colour pairs, so buckets, ramp colours and glyph pairs
 * always stay aligned. N_BIOMES (8) is therefore the shared length of all
 * three tables; changing it means re-tuning every one of them. */
typedef enum {
    BIOME_DEEP_OCEAN = 0,
    BIOME_OCEAN      = 1,
    BIOME_COAST      = 2,
    BIOME_PLAINS     = 3,
    BIOME_HILLS      = 4,
    BIOME_MOUNTAINS  = 5,
    BIOME_HIGHLANDS  = 6,
    BIOME_PEAKS      = 7,
    N_BIOMES         = 8,
} Biome;

/*
 * BIOME_GLYPHS[biome][0..1] — two glyphs per biome; per-cell hash
 * picks one for textural variation.
 */
static const char BIOME_GLYPHS[N_BIOMES][2] = {
    { '~', ',' },     /* DEEP_OCEAN */
    { '~', '_' },     /* OCEAN      */
    { '.', ',' },     /* COAST      */
    { '.', '_' },     /* PLAINS     */
    { '_', '^' },     /* HILLS      */
    { '^', 'A' },     /* MOUNTAINS  */
    { 'A', '#' },     /* HIGHLANDS  */
    { '#', '@' },     /* PEAKS      */
};

/* Density ramp for SLOPE / ELEVATION-style readouts. */
static const char ELEV_RAMP[N_BIOMES] = {
    '`', '.', ',', ':', '-', '^', '#', '@'
};

/* Pattern — fifteen ways to render the same heightmap. The first four
 * are the original analytic views; the rest layer shading, contours,
 * structure detection and animation on the same three float arrays. */
typedef enum {
    PATTERN_TERRAIN  = 0,   /* biome glyph map                         */
    PATTERN_DROPLETS,       /* live water trails over dim terrain      */
    PATTERN_EROSION,        /* cut/fill diff vs initial                */
    PATTERN_SLOPE,          /* gradient magnitude + flow arrows        */
    PATTERN_RELIEF,         /* colour hillshade — looks 3-D            */
    PATTERN_CONTOUR,        /* topographic isolines                    */
    PATTERN_HYPSO,          /* filled hypsometric tint                 */
    PATTERN_ASPECT,         /* slope-facing compass (8 hues)           */
    PATTERN_RIDGES,         /* ridge / valley skeleton                 */
    PATTERN_CANYONS,        /* carved channels glow over shaded land   */
    PATTERN_BATHY,          /* animated ocean caustics, calm land      */
    PATTERN_PLASMA,         /* animated colour field over the shape    */
    PATTERN_NIGHT,          /* dark sea, lit ridges, glowing peaks     */
    PATTERN_FLOW,           /* contour bands sweeping downhill         */
    PATTERN_HEAT,           /* two-tone thermal elevation map          */
    N_PATTERNS,
} Pattern;

/* 8-char padded names so the HUD field width stays fixed. */
static const char *PATTERN_NAMES[N_PATTERNS] = {
    "TERRAIN ", "DROPLETS", "EROSION ", "SLOPE   ", "RELIEF  ",
    "CONTOUR ", "HYPSO   ", "ASPECT  ", "RIDGES  ", "CANYONS ",
    "BATHY   ", "PLASMA  ", "NIGHT   ", "FLOW    ", "HEAT    ",
};

/* ── Theme ─────────────────────────────────────────────────────────────── *
 * WHAT  One named colour palette (10 ship; t/T cycles). A theme maps the
 *       abstract elevation/erosion quantities onto concrete 256-colour codes;
 *       theme_apply() loads those into ncurses colour pairs. Same 10-name
 *       menu as the sibling procedural showcases, for muscle memory.
 *
 * VALUE LOGIC  ramp[] is a hypsometric tint — a MONOTONIC gradient from the
 *       deepest/coldest (index 0) to the highest/brightest (7), one entry per
 *       Biome bucket; renderers index it by biome or by a derived 0..7 level.
 *       hot/cold are a DIVERGENT accent pair straddling the no-change point:
 *       hot = eroded cuts & peaks, cold = deposits & water. Every code sits in
 *       the BRIGHT half of the cube (≥24 / ≥244, per CLAUDE.md "Theme Palette
 *       Brightness") so even A_DIM cells stay legible on default-black.
 *
 * REFS  Hypsometric tinting + divergent relief palettes — Imhof,
 *       "Cartographic Relief Presentation" (1982). Project palette notes —
 *       documentation/COLOR.md. */
typedef struct {
    const char *name;            /* HUD label (t/T cycles)                   */
    short       ramp[N_BIOMES];  /* hypsometric tint, low→high (256-colour)  */
    short       hot;             /* divergent accent: cuts / peaks           */
    short       cold;            /* divergent accent: deposits / water       */
} Theme;

#define N_THEMES 10

/*
 * All ramp entries sit in the bright half of the 256-colour space so
 * even A_DIM cells stay legible against a default-black terminal.
 * See "Theme Palette Brightness" in /CLAUDE.md.
 */
static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7   hot cold */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 }, 196,  39 },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 }, 226,  39 },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 }, 196,  39 },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 }, 226,  39 },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 }, 196,  21 },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 }, 226,  21 },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 }, 196,  39 },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 }, 196,  39 },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 }, 196,  21 },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 }, 196,  39 },
};

/* ── Heightmap ─────────────────────────────────────────────────────────── *
 * WHAT  The terrain under simulation: a w×h grid of scalar elevations, stored
 *       row-major in flat arrays (cell (x,y) at index y*w+x — always via
 *       hidx() so the stride is never hand-rolled). It is the single domain
 *       object — each of the 15 views is just a different READING of these
 *       arrays — and SIMULATION (§4) is its ONLY writer.
 *
 * WHY STRUCT-OF-ARRAYS  One array per quantity (not an array of per-cell
 *       structs): the hot loops (erode_brush, every renderer) sweep ONE
 *       quantity at a time, so contiguous height[] / initial[] keep the cache
 *       warm. Sizes are static (CELLS_MAX) — nothing is malloc'd on the hot
 *       path (project "Memory Allocation" rule).
 *
 * REFS  Droplet erosion model — Beyer, "Implementation of a method for
 *       hydraulic erosion" (TU München, 2015); Lague, "Coding Adventure:
 *       Hydraulic Erosion" (2019). Base terrain — Perlin fBm (Perlin,
 *       "Improving Noise", 2002). Full citations in the file-header block. */
typedef struct {
    /* ── grid extent (cells); clamped to ≤ MAP_W_MAX × MAP_H_MAX at layout ─ */
    int    w, h;

    /* ── elevation: TWO snapshots, both [0,1] (fBm output, gamma-sharpened) ─
     * height[] is mutated as droplets cut and fill; initial[] is the frozen
     * pre-erosion copy taken at generation. Keeping both lets render_erosion /
     * canyons / deltas show the SIGNED difference (height − initial) as a
     * cut(−)/fill(+) map — the actual subject of an erosion demo, not just the
     * terrain. EROSION_THRESH_LOW/HIGH pick where that diff starts colouring. */
    float  height     [CELLS_MAX];   /* current eroded elevation [0,1]   */
    float  initial    [CELLS_MAX];   /* pre-erosion copy (for cut/fill)  */

    /* ── cosmetic EFFECTS overlay (§5) — NOT physics ──────────────────────
     * Stamped to 1.0 by droplet_simulate() at each visited cell, then decayed
     * ×WATER_TRAIL_DECAY (0.92) per tick → a fading "a droplet just passed
     * here" glow (~30 ticks ≈ 0.5 s). Read ONLY by render_droplets; deleting
     * it changes no elevation. WATER_TRAIL_HIGH/MID slice it into brightness
     * tiers. */
    float  water_trail[CELLS_MAX];   /* decaying recent-visit indicator  */

    /* ── this generation's erosion run ────────────────────────────────────
     * seed selects which fBm terrain (feeds perm_shuffle, re-hashed from the
     * clock at each regen so worlds differ). droplets_done counts particles
     * simulated and drives the phase machine: < DROPLETS_PER_GEN = ERODING,
     * == = SETTLED → arm HOLD (see Scene.hold_countdown / §4 scene_tick). */
    int    seed;                     /* which terrain (fBm permutation)  */
    int    droplets_done;            /* run progress [0 .. DROPLETS_PER_GEN] */
} Heightmap;

/* ===================================================================== */
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The 60 fps frame cap and the fixed-timestep
 * accumulator that decides how many scene_tick()s run per frame are
 * POLICY -- they live in main() (§8), the single per-tick combine point. */

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
/* §3  LOGIC  -- pure decisions: no mutation, no I/O                     */
/* ===================================================================== */

/* Every function below is a pure read: it takes its inputs as parameters
 * (or the read-only perm[] / PATTERN_NAMES tables) and returns a value
 * with NO mutation and NO I/O. Deleting or reordering any RENDER / EFFECTS
 * code cannot change what these return -- corruption-proof by construction.
 * Shared by SIMULATION (§4) and RENDER (§7). */

static const char *pattern_name(Pattern p)
{
    return ((int)p >= 0 && (int)p < N_PATTERNS) ? PATTERN_NAMES[p]
                                                : "?       ";
}

/* hash3 — same as other showcases. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/* Perlin scaffold — copied inline per the self-contained-file rule. */
static uint8_t perm[512];

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

static float fbm2(float x, float y)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;     /* → [0, 1] */
}

static inline int hidx(const Heightmap *hm, int x, int y) { return y * hm->w + x; }

/* Bilinear height sample at fractional (x, y). Caller must guarantee
 * 0 ≤ xi+1 < w and 0 ≤ yi+1 < h — we don't bounds-check here because
 * the droplet step does it once per iteration. */
static inline float h_bilinear(const Heightmap *hm, float x, float y)
{
    int xi = (int)x, yi = (int)y;
    float fx = x - (float)xi, fy = y - (float)yi;
    int idx = hidx(hm, xi, yi);
    float h00 = hm->height[idx];
    float h10 = hm->height[idx + 1];
    float h01 = hm->height[idx + hm->w];
    float h11 = hm->height[idx + hm->w + 1];
    return (h00 * (1.0f - fx) + h10 * fx) * (1.0f - fy)
         + (h01 * (1.0f - fx) + h11 * fx) * fy;
}

/* Height at an integer cell with edges clamped — for contour / ridge
 * renderers that sample neighbours right at the map border. */
static inline float h_at(const Heightmap *hm, int x, int y)
{
    if (x < 0) x = 0; else if (x >= hm->w) x = hm->w - 1;
    if (y < 0) y = 0; else if (y >= hm->h) y = hm->h - 1;
    return hm->height[hidx(hm, x, y)];
}

/* ----------------------------------------------------------------------- *
 * Helpers used by the renderers.                                          *
 * ----------------------------------------------------------------------- */

/* Map elevation [0, 1] (with some headroom either side) into one of
 * eight biome buckets. */
static inline int height_to_biome(float e)
{
    if (e < 0.18f) return BIOME_DEEP_OCEAN;
    if (e < 0.30f) return BIOME_OCEAN;
    if (e < 0.40f) return BIOME_COAST;
    if (e < 0.50f) return BIOME_PLAINS;
    if (e < 0.62f) return BIOME_HILLS;
    if (e < 0.75f) return BIOME_MOUNTAINS;
    if (e < 0.85f) return BIOME_HIGHLANDS;
    return BIOME_PEAKS;
}

/* Central-difference gradient at integer cell. y-difference divided
 * by ASPECT_Y_F to reflect physical-distance per cell. */
static inline void cell_gradient(const Heightmap *hm, int x, int y,
                                  float *gx, float *gy)
{
    int xm = (x > 0)        ? x - 1 : x;
    int xp = (x < hm->w - 1) ? x + 1 : x;
    int ym = (y > 0)        ? y - 1 : y;
    int yp = (y < hm->h - 1) ? y + 1 : y;
    *gx = (hm->height[hidx(hm, xp, y)] - hm->height[hidx(hm, xm, y)]) * 0.5f;
    *gy = (hm->height[hidx(hm, x, yp)] - hm->height[hidx(hm, x, ym)]) * 0.5f / ASPECT_Y_F;
}

/* Lambert hillshade in [0, 1]: surface normal (−gx, −gy, 1) dotted with
 * a fixed top-left light. Gradient is exaggerated first so subtle relief
 * still casts visible shading. Shared by RELIEF/CANYONS/BATHY/NIGHT. */
static inline float cell_shade(const Heightmap *hm, int x, int y)
{
    float gx, gy;
    cell_gradient(hm, x, y, &gx, &gy);
    gx *= RELIEF_EXAGGERATE;
    gy *= RELIEF_EXAGGERATE;
    float inv = 1.0f / sqrtf(gx * gx + gy * gy + 1.0f);
    float s = (-gx * RELIEF_LIGHT_X - gy * RELIEF_LIGHT_Y
               + RELIEF_LIGHT_Z) * inv;
    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    return s;
}

/* Shade level 0..7 from a Lambert value — index into ELEV_RAMP. */
static inline int shade_to_level(float sh)
{
    int l = (int)(sh * 7.0f + 0.5f);
    return l < 0 ? 0 : l > 7 ? 7 : l;
}

/* Disc-falloff weight of a brush cell at offset (dx,dy): 1 at the centre,
 * linearly to 0 at radius EROSION_BRUSH_R, exactly 0 outside. The erosion
 * brush spreads a droplet's bite over this disc so cuts aren't single-cell
 * spikes (Beyer 2015, "radius" brush). */
static inline float brush_weight(int dx, int dy)
{
    float d = sqrtf((float)(dx * dx + dy * dy));
    return (d > (float)EROSION_BRUSH_R) ? 0.0f
                                        : 1.0f - d / (float)EROSION_BRUSH_R;
}

/* Sediment carrying capacity of a droplet on a step of height change dh<0
 * (descending). Proportional to drop steepness × speed × water (Lague 2019;
 * Beyer 2015, eq. "c = ..."), floored at MIN_CAPACITY so a near-flat step
 * still lets a trickle of erosion happen. */
static inline float sediment_capacity(float dh, float speed, float water)
{
    float cap = (-dh) * speed * water * SEDIMENT_CAPACITY_K;
    return (cap < MIN_CAPACITY) ? MIN_CAPACITY : cap;
}

/* Sample the bed directly under a droplet at fractional (x,y): returns the
 * bilinearly interpolated height and writes the bilinear forward-difference
 * slope into (*gx,*gy). One read shared by steering and the dh calculation,
 * so both see exactly the same four corners. Caller guarantees xi+1,yi+1 are
 * in range (droplet_simulate bounds-checks each step). */
static inline float bed_sample(const Heightmap *hm, float x, float y,
                               float *gx, float *gy)
{
    int xi = (int)x, yi = (int)y;
    float fx = x - (float)xi, fy = y - (float)yi;
    int idx = hidx(hm, xi, yi);
    float h00 = hm->height[idx];
    float h10 = hm->height[idx + 1];
    float h01 = hm->height[idx + hm->w];
    float h11 = hm->height[idx + hm->w + 1];
    *gx = (h10 - h00) * (1.0f - fy) + (h11 - h01) * fy;
    *gy = (h01 - h00) * (1.0f - fx) + (h11 - h10) * fx;
    return h00 * (1 - fx) * (1 - fy) + h10 * fx * (1 - fy)
         + h01 * (1 - fx) *      fy  + h11 * fx *      fy;
}

/* True once this generation has spent its whole droplet budget — the
 * ERODING → SETTLED transition of the scene phase machine. */
static inline bool erosion_complete(const Heightmap *hm)
{
    return hm->droplets_done >= DROPLETS_PER_GEN;
}

/* Downhill flow arrow for a cell: the gradient points UPHILL, so water flows
 * the opposite way — pick the cardinal arrow of −gradient. Shared by the
 * SLOPE and CANYONS views. */
static inline char flow_arrow(float gx, float gy)
{
    if (fabsf(gx) > fabsf(gy)) return (gx > 0) ? '<' : '>';
    else                       return (gy > 0) ? '^' : 'v';
}

/* Contour-line glyph for a cell: an isoline runs PERPENDICULAR to the
 * gradient, so a mostly-horizontal gradient draws a vertical bar and vice
 * versa, with diagonals in between. Shared by the CONTOUR and FLOW views. */
static inline char contour_glyph(float gx, float gy)
{
    if      (fabsf(gx) > 2.0f * fabsf(gy)) return '|';
    else if (fabsf(gy) > 2.0f * fabsf(gx)) return '-';
    else    return (gx * gy > 0.0f) ? '\\' : '/';
}

/* Which of 8 compass sectors the downhill direction (−gradient) falls in,
 * 0..7 starting at east and turning CCW. Used by the ASPECT view to colour
 * and arrow each cell by the way its slope faces. */
static inline int downhill_sector(float gx, float gy)
{
    float ang = atan2f(-gy, -gx);
    return (int)floorf((ang + (float)M_PI) / (2.0f * (float)M_PI) * 8.0f) & 7;
}

/* Discrete Laplacian (∇²) at a cell — the 4-neighbour curvature of the
 * height field. Strongly negative = a convex ridge crest, strongly positive
 * = a concave valley floor. Used by the RIDGES view. */
static inline float cell_laplacian(const Heightmap *hm, int x, int y)
{
    return h_at(hm, x - 1, y) + h_at(hm, x + 1, y)
         + h_at(hm, x, y - 1) + h_at(hm, x, y + 1) - 4.0f * h_at(hm, x, y);
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (only writers of sim state)         */
/* ===================================================================== */

/* The ONLY writers of simulation state. Mutates: Heightmap.height[],
 * .initial[], .water_trail[], .seed, .droplets_done (and perm[] via
 * perm_shuffle); Scene.time_secs, .hold_countdown. scene_tick() is the
 * single per-tick entry point, called only from main (§8). User events
 * (key / resize) also mutate Scene but are NOT part of the tick -- see §8. */

static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

/*
 * heightmap_generate — sample fBm noise into height + initial, zero
 * out water_trail. Aspect-corrected y so blob features look round
 * on screen.
 */
static void heightmap_generate(Heightmap *hm, int seed)
{
    hm->seed = seed;
    hm->droplets_done = 0;
    perm_shuffle(seed);

    for (int y = 0; y < hm->h; y++) {
        for (int x = 0; x < hm->w; x++) {
            float nx = (float)x * FBM_SCALE_X;
            float ny = (float)y * FBM_SCALE_Y;
            float n  = fbm2(nx, ny);
            n = powf(n, FBM_GAMMA);                /* sharpen highs */
            int idx = hidx(hm, x, y);
            hm->height[idx]      = n;
            hm->initial[idx]     = n;
            hm->water_trail[idx] = 0.0f;
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Erosion brush — disc-weighted height subtract.                          *
 * ----------------------------------------------------------------------- */

static void erode_brush(Heightmap *hm, float fx, float fy, float amount)
{
    int cx = (int)fx, cy = (int)fy;

    /* First pass — total disc weight, so the bite is normalised to `amount`
     * regardless of how much of the disc is clipped at the map edge. */
    float total_w = 0.0f;
    for (int dy = -EROSION_BRUSH_R; dy <= EROSION_BRUSH_R; dy++)
        for (int dx = -EROSION_BRUSH_R; dx <= EROSION_BRUSH_R; dx++)
            total_w += brush_weight(dx, dy);
    if (total_w < 1e-6f) return;

    /* Second pass — subtract each in-bounds cell's share of `amount`. */
    for (int dy = -EROSION_BRUSH_R; dy <= EROSION_BRUSH_R; dy++) {
        int ny = cy + dy;
        if (ny < 0 || ny >= hm->h) continue;
        for (int dx = -EROSION_BRUSH_R; dx <= EROSION_BRUSH_R; dx++) {
            int nx = cx + dx;
            if (nx < 0 || nx >= hm->w) continue;
            float w = brush_weight(dx, dy);
            hm->height[hidx(hm, nx, ny)] -= amount * w / total_w;
        }
    }
}

/* ── Droplet ───────────────────────────────────────────────────────────── *
 * WHAT  One water particle in particle-based ("droplet") hydraulic erosion.
 *       The sim never carves rivers directly: it drops thousands of these and
 *       lets each trace a downhill path, eroding the bed where it accelerates
 *       and depositing where it slows or oversaturates. Stream networks EMERGE
 *       as the sum of many paths — no river is ever drawn deliberately.
 *
 * LIFETIME  Stack-local, never stored: spawned (droplet_spawn), walked for
 *       ≤ DROPLET_MAX_STEPS (32) steps (droplet_simulate), discarded. Its only
 *       lasting trace is the height[] it edited and the water_trail[] it lit.
 *
 * REFS  Heading + carrying-capacity model — Beyer (2015, "The droplet");
 *       Lague (2019). Tuning constants in §1 (INERTIA, SEDIMENT_CAPACITY_K,
 *       MIN_CAPACITY, ERODE_RATE, DEPOSIT_RATE, EVAPORATE_RATE, GRAVITY). */
typedef struct {
    /* ── where it is / where it's heading ─────────────────────────────────
     * (x,y) is a FRACTIONAL position in cell space (bilinearly sampled, never
     * snapped to a cell). (dx,dy) is the heading, renormalised to unit length
     * each step: dx = dx·INERTIA − gx·(1−INERTIA). With INERTIA = 0.05 it is
     * ~95% downhill −gradient + 5% memory — enough momentum to cross flats
     * without jittering, not enough to climb. */
    float x, y;          /* position in cell space          */
    float dx, dy;        /* unit heading (steered downhill) */

    /* ── the load it transports ───────────────────────────────────────────
     * Carrying capacity each step:  cap = max(−dh · speed · water ·
     * SEDIMENT_CAPACITY_K, MIN_CAPACITY).  sediment < cap on a descent →
     * ERODE (take min(cap−sediment)·ERODE_RATE, −dh); sediment > cap or going
     * uphill → DEPOSIT. speed grows on descents (v² += −dh·GRAVITY), boosting
     * capacity; water starts at INITIAL_WATER and decays ×(1−EVAPORATE_RATE)
     * each step, steadily shrinking capacity so the droplet must finally drop
     * its load. */
    float speed;         /* kinetic energy → carrying capacity */
    float water;         /* evaporates along the path          */
    float sediment;      /* mass currently suspended           */
} Droplet;

/* Drop `amount` of sediment back onto the bed under a droplet at fractional
 * (x,y), bilinearly split across the four corner cells — the inverse of the
 * brush bite, so a slowing droplet lays its load down smoothly. */
static void deposit_sediment(Heightmap *hm, float x, float y, float amount)
{
    int xi = (int)x, yi = (int)y;
    float fx = x - (float)xi, fy = y - (float)yi;
    int idx = hidx(hm, xi, yi);
    hm->height[idx]              += amount * (1.0f - fx) * (1.0f - fy);
    hm->height[idx + 1]          += amount *         fx  * (1.0f - fy);
    hm->height[idx + hm->w]      += amount * (1.0f - fx) *         fy;
    hm->height[idx + hm->w + 1]  += amount *         fx  *         fy;
}

/* Walk one droplet across the bed for up to DROPLET_MAX_STEPS, eroding and
 * depositing as it goes. Reads top-to-bottom as the per-step algorithm; the
 * physics of each step lives in the named helpers (bed_sample,
 * sediment_capacity, erode_brush, deposit_sediment). */
static void droplet_simulate(Heightmap *hm, Droplet *d)
{
    for (int step = 0; step < DROPLET_MAX_STEPS; step++) {
        /* Stop if the droplet (or its next cell) would leave the grid. */
        int xi = (int)d->x, yi = (int)d->y;
        if (xi < 0 || xi >= hm->w - 1 || yi < 0 || yi >= hm->h - 1) return;

        /* Mark the visited cell for the cosmetic water trail (§5). */
        hm->water_trail[hidx(hm, xi, yi)] = 1.0f;

        /* Sample the bed here: height under the droplet + downhill slope. */
        float gx, gy;
        float old_h = bed_sample(hm, d->x, d->y, &gx, &gy);

        /* Steer: blend old heading with −gradient (INERTIA), renormalise. */
        d->dx = d->dx * INERTIA - gx * (1.0f - INERTIA);
        d->dy = d->dy * INERTIA - gy * (1.0f - INERTIA);
        float len = sqrtf(d->dx * d->dx + d->dy * d->dy);
        if (len < DROPLET_STALL_LEN) return;        /* stalled on a flat */
        d->dx /= len; d->dy /= len;

        /* Take a unit step; stop if it lands off-grid. */
        float new_x = d->x + d->dx;
        float new_y = d->y + d->dy;
        int new_xi = (int)new_x, new_yi = (int)new_y;
        if (new_xi < 0 || new_xi >= hm->w - 1 ||
            new_yi < 0 || new_yi >= hm->h - 1) return;

        /* Height change over the step decides erode vs deposit. */
        float dh  = h_bilinear(hm, new_x, new_y) - old_h;
        float cap = sediment_capacity(dh, d->speed, d->water);

        if (d->sediment > cap || dh > 0.0f) {
            /* DEPOSIT: fill an uphill rise (≤ dh), or shed the excess above
             * capacity at DEPOSIT_RATE. */
            float amount = (dh > 0.0f)
                         ? (dh < d->sediment ? dh : d->sediment)
                         : (d->sediment - cap) * DEPOSIT_RATE;
            d->sediment -= amount;
            deposit_sediment(hm, d->x, d->y, amount);
        } else {
            /* ERODE: take the capacity deficit at ERODE_RATE, capped at the
             * drop so the brush never digs below the step it is descending. */
            float wanted = (cap - d->sediment) * ERODE_RATE;
            float amount = (wanted < -dh) ? wanted : -dh;
            erode_brush(hm, d->x, d->y, amount);
            d->sediment += amount;
        }

        /* Energetics: speed gains kinetic energy on descents, water evaporates. */
        float v_sq = d->speed * d->speed - dh * GRAVITY;
        d->speed = (v_sq > 0.0f) ? sqrtf(v_sq) : 0.0f;
        d->water *= (1.0f - EVAPORATE_RATE);

        d->x = new_x;
        d->y = new_y;
    }
}

/*
 * droplet_spawn — pick a random in-bounds cell and initialise. Uses
 * rand() so different droplets land in different places (rand() is
 * srand()-seeded once in main from the wall clock).
 */
static void droplet_spawn(Droplet *d, int w, int h)
{
    d->x = 1.0f + (float)(rand() % (w - 2));
    d->y = 1.0f + (float)(rand() % (h - 2));
    d->dx = 0.0f;  d->dy = 0.0f;
    d->speed    = INITIAL_SPEED;
    d->water    = INITIAL_WATER;
    d->sediment = 0.0f;
}

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * WHAT  The whole simulated-and-shown world: one eroding Heightmap plus the
 *       few knobs and clocks that drive and time it. Reads top-to-bottom like
 *       a table of contents (WHAT / HOW / view / WHEN). The loose scalars are
 *       each too thin to deserve their own type, so they live here grouped by
 *       the CONCEPT they belong to — not by which key toggles them (a colour
 *       theme is a RENDER concept even though a key flips it, NOT a sim knob).
 *
 * PHASE MACHINE  hm.droplets_done + hold_countdown encode a 3-state cycle, run
 *       once per tick by scene_tick(): ERODING (spawn droplets) → SETTLED
 *       (budget reached → arm HOLD) → HOLDING (count down) → regenerate. This
 *       is why the two counters live one level apart: progress is a property
 *       of the terrain (Heightmap), the hold timer is a property of the run. */
typedef struct {
    /* WHAT is simulated — the domain object. */
    Heightmap hm;

    /* HOW the user drives it — the one tunable simulation knob.
     * Spawn rate = DROPLETS_PER_TICK_DEF · speed / SPEED_DEF (clamped ≥ 1);
     * speed ∈ [SPEED_MIN..SPEED_MAX] = 1..64, doubled/halved by +/−. Trades
     * watchability (speed 1: follow one trail) for throughput (speed 64). */
    int       speed;            /* droplets spawned per tick (scaled)        */

    /* WHAT we are looking at — render selections (RENDER, not simulation).
     * Changing either never touches physics: same heightmap, different read. */
    Pattern   current_pattern;  /* which of the 15 views is drawn (n/p)      */
    int       current_theme;    /* index into themes[] (t/T), 0..N_THEMES-1  */

    /* WHEN we are — run clock + state (the DELAYS concept, §6, woven into §4).
     * time_secs accumulates dt every tick: it both phases all animated views
     * and is hashed into the next seed at regen, so each world differs.
     * hold_countdown is armed to HOLD_TICKS (~6 s) when erosion completes. */
    float     time_secs;        /* simulated seconds; animation + reseed     */
    int       hold_countdown;   /* ticks left in the post-erosion HOLD       */
    bool      paused;           /* simulation frozen; rendering continues    */
} Scene;

static void scene_rebuild(Scene *s)
{
    int seed = (int)hash3((int)(s->time_secs * 1000.0f),
                          s->hm.w, s->hm.h ^ 0x9E3779B9);
    heightmap_generate(&s->hm, seed);
    s->hold_countdown = 0;
}

static void scene_init(Scene *s, int map_w, int map_h)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_TERRAIN;
    s->hm.w = map_w;
    s->hm.h = map_h;
    scene_rebuild(s);
}

static void scene_resize_to(Scene *s, int map_w, int map_h)
{
    s->hm.w = map_w;
    s->hm.h = map_h;
    scene_rebuild(s);
}

/* Fade every cell's water trail one tick toward zero (EFFECTS, §5). Runs
 * whether or not we are still eroding, so trails keep decaying after the
 * last droplet lands. */
static void decay_water_trails(Heightmap *hm)
{
    int total = hm->w * hm->h;
    for (int i = 0; i < total; i++)
        hm->water_trail[i] *= WATER_TRAIL_DECAY;
}

/* Spawn and run one tick's worth of droplets. Count scales linearly with the
 * speed knob (SPEED_DEF=8 → 12/tick → ~720/s → ~11 s to spend the budget),
 * and the loop also stops the instant the per-generation budget is hit. */
static void run_erosion_batch(Heightmap *hm, int speed)
{
    int per_tick = DROPLETS_PER_TICK_DEF * speed / SPEED_DEF;
    if (per_tick < 1) per_tick = 1;
    for (int i = 0; i < per_tick && !erosion_complete(hm); i++) {
        Droplet d;
        droplet_spawn(&d, hm->w, hm->h);
        droplet_simulate(hm, &d);
        hm->droplets_done++;
    }
}

/* One simulation step = the scene phase machine (§6 DELAYS woven in):
 *   ERODING  → spawn a batch; when the budget is spent, arm the HOLD.
 *   HOLDING  → count the HOLD down.
 *   ELAPSED  → regenerate a fresh terrain and start over.
 * Paused freezes the machine but not the wall clock. */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    if (s->paused) return;

    decay_water_trails(&s->hm);

    if (!erosion_complete(&s->hm)) {
        run_erosion_batch(&s->hm, s->speed);
        if (erosion_complete(&s->hm))
            s->hold_countdown = HOLD_TICKS;     /* just settled → hold */
    } else if (s->hold_countdown > 0) {
        s->hold_countdown--;
    } else {
        scene_rebuild(s);
    }
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No separate layer -- one line, as the SEPARATION audit requires when a
 * concern is trivial. The only cosmetic-only state is Heightmap.water_trail[]
 * (the fading droplet trails behind the DROPLETS view): WRITTEN by
 * droplet_simulate() and faded each tick by decay_water_trails() (both §4),
 * READ only by render_droplets() (§7). Every other glow -- peak twinkle,
 * plasma, caustics, night glints -- is derived at render time from
 * height/time, never stored. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer -- one line. The post-erosion HOLD before regeneration
 * is a single int, Scene.hold_countdown: armed to HOLD_TICKS when the droplet
 * budget is reached and counted down inside scene_tick() (§4); at 0 the scene
 * regenerates. The pause toggle (Scene.paused) early-returns scene_tick().
 * Both are trivial and woven into the simulation tick. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. Reads Scene / Heightmap, writes ONLY the ncurses back
 * buffer (and the colour-pair table at init). Never touches simulation
 * state, so a frame may be dropped or rebuilt with no effect on the sim. */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < N_BIOMES; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_HOT,   t->hot,    -1);
        init_pair(PAIR_COLD,  t->cold,   -1);
        init_pair(PAIR_WATER, 51, -1);          /* bright cyan      */
    } else {
        static const short fb[N_BIOMES] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_GREEN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < N_BIOMES; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_HOT,   COLOR_RED,  -1);
        init_pair(PAIR_COLD,  COLOR_BLUE, -1);
        init_pair(PAIR_WATER, COLOR_CYAN, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── Screen ────────────────────────────────────────────────────────────── *
 * WHAT  The terminal viewport: its full character size, the heightmap
 *       rectangle chosen to fit inside it, and the top-left offset where that
 *       rectangle is centred. Pure presentation geometry — holds NO simulation
 *       state — recomputed by screen_layout() at startup and on every SIGWINCH.
 *
 * WHY  This is the project's ONE coordinate bridge: map cell (x,y) draws at
 *       terminal (gy0 + y, gx0 + x), computed in exactly one place so no
 *       renderer hand-rolls the offset. map_w/map_h are clamped to [16..MAP_W_MAX] ×
 *       [8..MAP_H_MAX]: the upper bound stops a huge terminal from indexing
 *       past the static CELLS_MAX store; the lower keeps a tiny one usable.
 *       Rows 0–1 are the HUD and the last row the hint line, so gy0 starts at
 *       row 2 and the field is inset to avoid overwriting them. */
typedef struct {
    int cols, rows;      /* full terminal size (getmaxyx)        */
    int map_w, map_h;    /* heightmap rectangle that fits inside */
    int gx0, gy0;        /* its top-left origin on screen        */
} Screen;

static void screen_layout(Screen *s)
{
    int top = HUD_TOP_ROWS, bottom = s->rows - 1;   /* HUD top, hint bottom */
    int avail_h = bottom - top;
    int avail_w = s->cols;

    int mw = avail_w;
    int mh = avail_h;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;

    s->map_w = mw;
    s->map_h = mh;
    s->gx0 = (avail_w - mw) / 2;
    s->gy0 = top + (avail_h - mh) / 2;
    if (s->gx0 < 0)   s->gx0 = 0;
    if (s->gy0 < top) s->gy0 = top;
}

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
    screen_layout(s);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_layout(s);
}

/* ----------------------------------------------------------------------- *
 * Pattern renderers.                                                       *
 * ----------------------------------------------------------------------- */

/* Terrain — eroded heightmap as a biome map. Peaks twinkle. */
static void render_terrain(const Screen *sc, const Heightmap *hm, float t)
{
    int twinkle_t = (int)t;

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            int   bi  = height_to_biome(e);
            uint32_t hh = hash3(x, y, hm->seed);
            char  glyph;
            int   pair = PAIR_RAMP_BASE + bi;
            int   attr = A_NORMAL;

            if (bi <= BIOME_OCEAN) {
                /* Slow water shimmer on ocean cells. */
                static const char waves[4] = { '~', '_', '~', ',' };
                int phase = (x + y * 2 + (int)(t * 3.0f)) & 3;
                glyph = waves[phase];
                if (bi == BIOME_DEEP_OCEAN) attr = A_DIM;
            } else {
                glyph = BIOME_GLYPHS[bi][hh & 1];
            }

            if (bi == BIOME_PEAKS) {
                attr = A_BOLD;
                if ((hash3(x, y, twinkle_t) % 50u) == 0u) attr |= A_BOLD;
            } else if (bi >= BIOME_HIGHLANDS) {
                attr = A_BOLD;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Droplets — terrain dimmed + bright water trails on top. */
static void render_droplets(const Screen *sc, const Heightmap *hm)
{

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float trail = hm->water_trail[hidx(hm, x, y)];
            float e     = hm->height[hidx(hm, x, y)];
            int   bi    = height_to_biome(e);
            uint32_t hh = hash3(x, y, hm->seed);
            char  glyph;
            int   pair;
            int   attr;

            if (trail > WATER_TRAIL_HIGH) {
                glyph = '~';
                pair  = PAIR_WATER;
                attr  = A_BOLD;
            } else if (trail > WATER_TRAIL_MID) {
                glyph = '~';
                pair  = PAIR_WATER;
                attr  = A_NORMAL;
            } else if (trail > 0.05f) {
                glyph = '.';
                pair  = PAIR_WATER;
                attr  = A_DIM;
            } else {
                /* Backdrop biome glyph, dimmed so trails pop. */
                glyph = BIOME_GLYPHS[bi][hh & 1];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Erosion — cut/fill diff vs initial heightmap. */
static void render_erosion(const Screen *sc, const Heightmap *hm)
{

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            int idx = hidx(hm, x, y);
            float diff = hm->height[idx] - hm->initial[idx];
            char  glyph;
            int   pair;
            int   attr;

            if (diff < -EROSION_THRESH_HIGH) {
                glyph = '-';  pair = PAIR_HOT;  attr = A_BOLD;
            } else if (diff < -EROSION_THRESH_LOW) {
                glyph = '-';  pair = PAIR_HOT;  attr = A_NORMAL;
            } else if (diff > +EROSION_THRESH_HIGH) {
                glyph = '+';  pair = PAIR_COLD; attr = A_BOLD;
            } else if (diff > +EROSION_THRESH_LOW) {
                glyph = '+';  pair = PAIR_COLD; attr = A_NORMAL;
            } else {
                /* Unchanged — dim biome backdrop. */
                int bi = height_to_biome(hm->height[idx]);
                uint32_t hh = hash3(x, y, hm->seed);
                glyph = BIOME_GLYPHS[bi][hh & 1];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Slope — gradient magnitude as a heatmap; arrow glyph for direction
 * of -gradient (downhill = the way water flows). */
static void render_slope(const Screen *sc, const Heightmap *hm)
{

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float gx, gy;
            cell_gradient(hm, x, y, &gx, &gy);
            float mag = sqrtf(gx * gx + gy * gy);
            int level = (int)(mag * SLOPE_SCALE);
            if (level < 0)         level = 0;
            if (level >= N_BIOMES) level = N_BIOMES - 1;

            char glyph = (mag < 1e-4f) ? '.' : flow_arrow(gx, gy);

            int pair = PAIR_RAMP_BASE + level;
            int attr = (level >= 6) ? A_BOLD
                     : (level <= 1) ? A_DIM
                     : A_NORMAL;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Relief — colour by elevation (biome ramp), brightness by hillshade.
 * Reads as a lit 3-D surface. */
static void render_relief(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float sh  = cell_shade(hm, x, y);
            int   bi  = height_to_biome(hm->height[hidx(hm, x, y)]);
            char  glyph = ELEV_RAMP[shade_to_level(sh)];
            int   pair  = PAIR_RAMP_BASE + bi;
            int   attr  = sh > 0.70f ? A_BOLD : sh < 0.22f ? A_DIM : A_NORMAL;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Contour — bright isolines where the quantised height band changes;
 * line glyph orients perpendicular to the gradient. Ocean faint, land
 * between lines left blank for a clean topographic-sheet look. */
static void render_contour(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = h_at(hm, x, y);
            int   lv  = (int)(e * CONTOUR_LEVELS);
            int   lvr = (int)(h_at(hm, x + 1, y) * CONTOUR_LEVELS);
            int   lvd = (int)(h_at(hm, x, y + 1) * CONTOUR_LEVELS);
            char  glyph;
            int   pair, attr;

            if (lv != lvr || lv != lvd) {
                float gx, gy;
                cell_gradient(hm, x, y, &gx, &gy);
                glyph = contour_glyph(gx, gy);
                int bi = lv * N_BIOMES / (CONTOUR_LEVELS + 1);
                if (bi > 7) bi = 7;
                pair = PAIR_RAMP_BASE + bi;
                attr = A_BOLD;
            } else if (height_to_biome(e) <= BIOME_OCEAN) {
                glyph = '~';
                pair  = PAIR_RAMP_BASE + height_to_biome(e);
                attr  = A_DIM;
            } else {
                continue;       /* land between lines: leave blank */
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Hypso — solid filled hypsometric tint: every cell a bold block in its
 * elevation colour. The clean "painted map" complement to TERRAIN. */
static void render_hypso(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            int  bi    = height_to_biome(hm->height[hidx(hm, x, y)]);
            char glyph = (bi <= BIOME_OCEAN) ? '~' : '#';
            int  pair  = PAIR_RAMP_BASE + bi;

            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* Aspect — which compass direction each slope faces. Downhill bearing is
 * binned into 8 sectors, each a distinct ramp hue + arrow glyph. */
static void render_aspect(const Screen *sc, const Heightmap *hm)
{
    static const char dirg[8] = { '>', '/', '^', '\\', '<', '/', 'v', '\\' };
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float gx, gy;
            cell_gradient(hm, x, y, &gx, &gy);
            float mag = sqrtf(gx * gx + gy * gy);
            char  glyph;
            int   pair, attr;

            if (mag < 5e-4f) {
                glyph = '.';  pair = PAIR_RAMP_BASE;  attr = A_DIM;
            } else {
                int sect = downhill_sector(gx, gy);
                glyph = dirg[sect];
                pair  = PAIR_RAMP_BASE + sect;
                attr  = mag > 0.02f ? A_BOLD : A_NORMAL;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Ridges — discrete Laplacian skeleton: strongly convex cells are ridge
 * crests (warm '^'), strongly concave are valleys (cool 'v'), the rest a
 * dim biome backdrop. Exposes the drainage structure. */
static void render_ridges(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            float lap = cell_laplacian(hm, x, y);
            char  glyph;
            int   pair, attr;

            if (lap < -RIDGE_THRESH) {
                glyph = '^';  pair = PAIR_HOT;   attr = A_BOLD;
            } else if (lap > RIDGE_THRESH) {
                glyph = 'v';  pair = PAIR_COLD;  attr = A_NORMAL;
            } else {
                int bi = height_to_biome(e);
                glyph = BIOME_GLYPHS[bi][hash3(x, y, hm->seed) & 1];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Canyons — cells the droplets carved below their initial height glow as
 * cyan river arrows (deeper cut = brighter); untouched land is a dim 3-D
 * hillshade so the channel network stands out in relief. */
static void render_canyons(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            int   idx  = hidx(hm, x, y);
            float diff = hm->height[idx] - hm->initial[idx];
            char  glyph;
            int   pair, attr;

            if (diff < -EROSION_THRESH_LOW) {
                float gx, gy;
                cell_gradient(hm, x, y, &gx, &gy);
                glyph = flow_arrow(gx, gy);
                pair = PAIR_WATER;
                attr = (diff < -EROSION_THRESH_HIGH) ? A_BOLD : A_NORMAL;
            } else {
                glyph = ELEV_RAMP[shade_to_level(cell_shade(hm, x, y))];
                pair  = PAIR_RAMP_BASE + height_to_biome(hm->height[idx]);
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Bathy — underwater focus: shallows-to-deep ocean ramp with animated
 * caustic shimmer; land is a calm dim hillshade so the eye stays on the
 * living sea. */
static void render_bathy(const Screen *sc, const Heightmap *hm, float t)
{
    static const char cg[4] = { '.', '~', '-', '=' };
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e  = hm->height[hidx(hm, x, y)];
            int   bi = height_to_biome(e);
            char  glyph;
            int   pair, attr;

            if (bi <= BIOME_COAST) {
                float caustic = sinf(x * 0.35f + y * 0.22f + t * 2.3f)
                              + sinf(x * 0.13f - y * 0.31f + t * 1.7f);
                int ci = (int)((caustic + 2.0f) / 4.0f * 3.99f);
                if (ci < 0) ci = 0;
                if (ci > 3) ci = 3;
                int depth = (int)((0.40f - e) * 16.0f);
                if (depth < 0) depth = 0;
                if (depth > 5) depth = 5;
                glyph = cg[ci];
                pair  = PAIR_RAMP_BASE + (5 - depth);   /* shallow=bright */
                attr  = ci >= 3 ? A_BOLD : ci == 0 ? A_DIM : A_NORMAL;
            } else {
                glyph = ELEV_RAMP[shade_to_level(cell_shade(hm, x, y))];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Plasma — pure eye-candy: a colour phase advances with elevation, two
 * sine waves and time, cycling the 8 ramp hues so colour bands flow
 * across the terrain shape. */
static void render_plasma(const Screen *sc, const Heightmap *hm, float t)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e = hm->height[hidx(hm, x, y)];
            float v = e * 7.0f + t * 0.8f
                    + sinf(x * 0.10f + t) * 0.8f
                    + cosf(y * 0.13f - t) * 0.8f;
            int idx = (int)v % N_BIOMES;
            if (idx < 0) idx += N_BIOMES;
            char glyph = ELEV_RAMP[idx];
            int  pair  = PAIR_RAMP_BASE + idx;

            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* Night — atmospheric mood: black sea with rare moon glints, dim land,
 * hillshade-lit ridgelines, and snow peaks that glow and twinkle. */
static void render_night(const Screen *sc, const Heightmap *hm, float t)
{
    int glint_t   = (int)(t * 2.0f);
    int twinkle_t = (int)(t * 3.0f);
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            int   bi  = height_to_biome(e);
            char  glyph;
            int   pair, attr;

            if (bi <= BIOME_OCEAN) {
                if ((hash3(x, y, glint_t) % 80u) != 0u) continue;  /* black */
                glyph = '.';  pair = PAIR_RAMP_BASE + bi;  attr = A_DIM;
            } else if (bi >= BIOME_HIGHLANDS) {
                glyph = BIOME_GLYPHS[bi][hash3(x, y, hm->seed) & 1];
                if (bi == BIOME_PEAKS && (hash3(x, y, twinkle_t) % 40u) == 0u)
                    glyph = '*';
                pair = PAIR_RAMP_BASE + 7;  attr = A_BOLD;
            } else if (cell_shade(hm, x, y) > 0.62f) {
                glyph = '^';  pair = PAIR_RAMP_BASE + 5;  attr = A_NORMAL;
            } else {
                glyph = BIOME_GLYPHS[bi][hash3(x, y, hm->seed) & 1];
                pair  = PAIR_RAMP_BASE + bi;  attr = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Flow — animated contours: the same isoline test as CONTOUR but the
 * bands scroll downhill over time, so bright lines appear to cascade
 * from peaks to sea like sheet flow. */
static void render_flow(const Screen *sc, const Heightmap *hm, float t)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e     = h_at(hm, x, y);
            float phase = e * CONTOUR_LEVELS - t * 2.0f;
            float frac  = phase - floorf(phase);
            char  glyph;
            int   pair, attr;

            if (frac < 0.18f) {
                float gx, gy;
                cell_gradient(hm, x, y, &gx, &gy);
                glyph = contour_glyph(gx, gy);
                int bi = (int)(e * 7.99f);
                if (bi > 7) bi = 7;
                pair = PAIR_RAMP_BASE + bi;  attr = A_BOLD;
            } else {
                int bi = height_to_biome(e);
                glyph = (bi <= BIOME_OCEAN) ? '~' : '.';
                pair  = PAIR_RAMP_BASE + bi;  attr = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Heat — two-tone thermal map: warm accent above mid-elevation, cool
 * below, density glyph by height. A stark cartographer's heat readout. */
static void render_heat(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            int   lvl = (int)(e * 7.99f);
            if (lvl < 0) lvl = 0;
            if (lvl > 7) lvl = 7;
            char glyph = ELEV_RAMP[lvl];
            int  pair  = (e >= 0.5f) ? PAIR_HOT : PAIR_COLD;
            int  attr  = (lvl >= 6 || lvl <= 1) ? A_BOLD : A_NORMAL;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void scene_draw(const Screen *sc, const Heightmap *hm,
                       Pattern pattern, float t)
{
    switch (pattern) {
    case PATTERN_TERRAIN:  render_terrain (sc, hm, t);  break;
    case PATTERN_DROPLETS: render_droplets(sc, hm);     break;
    case PATTERN_EROSION:  render_erosion (sc, hm);     break;
    case PATTERN_SLOPE:    render_slope   (sc, hm);     break;
    case PATTERN_RELIEF:   render_relief  (sc, hm);     break;
    case PATTERN_CONTOUR:  render_contour (sc, hm);     break;
    case PATTERN_HYPSO:    render_hypso   (sc, hm);     break;
    case PATTERN_ASPECT:   render_aspect  (sc, hm);     break;
    case PATTERN_RIDGES:   render_ridges  (sc, hm);     break;
    case PATTERN_CANYONS:  render_canyons (sc, hm);     break;
    case PATTERN_BATHY:    render_bathy   (sc, hm, t);  break;
    case PATTERN_PLASMA:   render_plasma  (sc, hm, t);  break;
    case PATTERN_NIGHT:    render_night   (sc, hm, t);  break;
    case PATTERN_FLOW:     render_flow    (sc, hm, t);  break;
    case PATTERN_HEAT:     render_heat    (sc, hm);     break;
    case N_PATTERNS:       break;       /* unreachable sentinel */
    }
}

/* Draw the 8-swatch colour-ramp legend at (row,x); returns the x past it. */
static int draw_ramp_legend(int row, int x)
{
    for (int i = 0; i < N_BIOMES; i++) {
        attron(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        mvaddch(row, x, (chtype)(unsigned char)ELEV_RAMP[i]);
        attroff(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        x++;
    }
    return x;
}

/* Row 0 right — primary status: fps, sim Hz, phase, speed. Right-aligned. */
static void draw_status_line(const Screen *sc, const Scene *s,
                             double fps, int sim_fps)
{
    const char *phase = s->paused                 ? "PAUSED  "
                      : erosion_complete(&s->hm)  ? "SETTLED "
                                                  : "ERODING ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, phase, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern (with n/N counter), theme, ramp legend, droplet progress.
 * Fixed left-aligned layout, so it needs the Scene but not the Screen. */
static void draw_param_line(const Scene *s)
{
    int x = 1;

    char pbuf[40];
    snprintf(pbuf, sizeof pbuf, " pattern:%s %2d/%-2d ",
             pattern_name(s->current_pattern),
             (int)s->current_pattern + 1, (int)N_PATTERNS);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, "%s", pbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += (int)strlen(pbuf);

    char tbuf[32];
    snprintf(tbuf, sizeof tbuf, " theme:%-8s ", themes[s->current_theme].name);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, "%s", tbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += (int)strlen(tbuf);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    x = draw_ramp_legend(1, x);

    int pct = (DROPLETS_PER_GEN > 0)
            ? (s->hm.droplets_done * 100) / DROPLETS_PER_GEN : 100;
    if (pct > 100) pct = 100;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  droplets:%5d/%-5d  %3d%% ",
             s->hm.droplets_done, (int)DROPLETS_PER_GEN, pct);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, progress, run-state.
 * A const read can't re-couple the layers; the leaf renderers stay narrow.
 * Reads as: draw the chosen view, then lay the HUD over it. */
static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, &s->hm, s->current_pattern, s->time_secs);

    draw_status_line(sc, s, fps, sim_fps);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " HYDRAULIC EROSION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    draw_param_line(s);
    draw_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main
 * loop. main() is the ONE place that combines the layers per tick, in fixed
 * order:  scene_tick (SIM + EFFECTS + DELAYS) -> screen_draw (RENDER) ->
 * screen_present -> input. app_handle_key() / app_do_resize() mutate state
 * in response to USER EVENTS and are deliberately OUTSIDE the tick. */

/* ── App ───────────────────────────────────────────────────────────────── *
 * WHAT  Top-level harness binding the simulation (scene) to the terminal
 *       (screen), plus the loop's PERFORMANCE knob and the async signal flags.
 *       A single static instance (g_app) exists ONLY so the signal handlers —
 *       which may fire between any two instructions — can reach the flags;
 *       everything else passes App explicitly. Just init + the main loop touch
 *       App whole; every other function takes the narrowest sub-type (§5).
 *
 * REFS  Fixed-timestep accumulator — sim_fps decouples the simulation rate
 *       from the render frame rate, so physics is deterministic regardless of
 *       terminal speed (Fiedler, "Fix Your Timestep!"; §8 main()). */
typedef struct {
    /* the two worlds it binds */
    Scene                 scene;        /* WHAT is simulated + shown         */
    Screen                screen;       /* WHERE it is drawn                 */

    /* loop control */
    int                   sim_fps;      /* fixed-timestep rate, SIM_FPS_* Hz */
    /* volatile sig_atomic_t: written from signal handlers, so the compiler
     * must re-read them each loop and the write must be atomic w.r.t. the
     * interrupted code. */
    volatile sig_atomic_t running;      /* cleared by SIGINT/SIGTERM → exit  */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, served next loop */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize_to(&app->scene, app->screen.map_w, app->screen.map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_rebuild(s);                              break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        break;

    default: break;
    }
    return true;
}

/* Fixed-timestep catch-up: run as many simulation ticks as the accumulated
 * real time (*sim_accum) allows, each advancing the scene by one fixed dt,
 * draining the accumulator as it goes. Decoupling the sim rate from the frame
 * rate keeps physics deterministic regardless of how fast frames are drawn
 * (Fiedler, "Fix Your Timestep!"). */
static void run_pending_ticks(Scene *s, int64_t *sim_accum,
                              int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(s, dt_sec);
        *sim_accum -= tick_ns;
    }
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
    scene_init(&app->scene, app->screen.map_w, app->screen.map_h);

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
        run_pending_ticks(&app->scene, &sim_accum, tick_ns, dt_sec);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
