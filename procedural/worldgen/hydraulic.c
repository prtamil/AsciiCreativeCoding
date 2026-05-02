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
 *       to broad, silt-filled deltas. Cycle four views with n / p:
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
 *       After ~8 000 droplets the simulation holds for a few
 *       seconds; then a flash, regenerate, and the cycle repeats
 *       on a fresh map.
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
 * Section map:
 *   §1 config     — geometry, droplet physics, themes
 *   §2 clock      — monotonic timer + sleep
 *   §3 color      — HUD reserved + 10 themes (8-step ramp + accents)
 *   §5 hydraulic  — hash, perlin/fbm, heightmap gen, droplet sim,
 *                   erode brush, gradient + biome helpers
 *   §6 scene      — phase machine (eroding → holding → regen)
 *   §7 screen     — four pattern renderers + HUD
 *   §8 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          regenerate fresh terrain + restart erosion
 *   n / N      next pattern  (TERRAIN → DROPLETS → EROSION → SLOPE)
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
 *   • Beyer, H. (2015) — "Implementation of a method for hydraulic
 *     erosion", Bachelor thesis (TU München).
 *     https://www.firespark.de/resources/downloads/implementation%20of%20a%20methode%20for%20hydraulic%20erosion.pdf
 *   • Lague, S. (2019) — "Coding Adventure: Hydraulic Erosion"
 *     https://www.youtube.com/watch?v=eaXk97ujbPQ
 *   • Mei, X., Decaudin, P., Hu, B-G. (2007) — "Fast Hydraulic
 *     Erosion Simulation and Visualization on GPU", Pacific Graphics.
 *     The grid-based "virtual pipes" alternative.
 *   • Wikipedia — Erosion / Stream power law
 *     https://en.wikipedia.org/wiki/Erosion
 *   • Perlin, K. (2002) — Improving Noise (the fBm scaffold)
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
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
 *  • Press 'r'. Flash, regen. The HUD's "droplets" counter resets
 *    to 0; the world is fresh; erosion starts over.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,
    CELLS_MAX           = MAP_W_MAX * MAP_H_MAX,

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
    PAIR_FLASH          =  14,    /* regen flash                      */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

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

/* Biome buckets — eight categories from deep ocean to peaks. */
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

/* Pattern — four ways to render the same heightmap. */
typedef enum {
    PATTERN_TERRAIN  = 0,
    PATTERN_DROPLETS = 1,
    PATTERN_EROSION  = 2,
    PATTERN_SLOPE    = 3,
    N_PATTERNS       = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_TERRAIN:  return "TERRAIN ";
    case PATTERN_DROPLETS: return "DROPLETS";
    case PATTERN_EROSION:  return "EROSION ";
    case PATTERN_SLOPE:    return "SLOPE   ";
    default:               return "?       ";
    }
}

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 * Each theme provides:
 *   ramp[8]  — gradient from deepest/coldest to highest/brightest
 *   hot      — accent for eroded cells (stream cuts) and peaks
 *   cold     — accent for deposited cells (deltas) and water trails
 *
 * All entries are picked from the brighter half of the 256-colour
 * cube so even A_DIM cells stay legible against a default-black
 * terminal.
 */
typedef struct {
    const char *name;
    short       ramp[N_BIOMES];
    short       hot;
    short       cold;
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
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

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
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  hydraulic — hash, perlin/fbm, heightmap, droplet sim               */
/* ===================================================================== */

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

/* ----------------------------------------------------------------------- *
 * Heightmap — three flat float arrays.                                    *
 * ----------------------------------------------------------------------- */

typedef struct {
    int    w, h;
    float  height     [CELLS_MAX];   /* current eroded heightmap        */
    float  initial    [CELLS_MAX];   /* pre-erosion copy (for diff)     */
    float  water_trail[CELLS_MAX];   /* decaying recent-visit indicator */
    int    seed;
    int    droplets_done;            /* progress this generation        */
} Heightmap;

static inline int hidx(const Heightmap *hm, int x, int y) { return y * hm->w + x; }

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

/* ----------------------------------------------------------------------- *
 * Erosion brush — disc-weighted height subtract.                          *
 * ----------------------------------------------------------------------- */

static void erode_brush(Heightmap *hm, float fx, float fy, float amount)
{
    int cx = (int)fx, cy = (int)fy;
    float total_w = 0.0f;

    /* First pass — sum weights inside the disc. */
    for (int dy = -EROSION_BRUSH_R; dy <= EROSION_BRUSH_R; dy++) {
        for (int dx = -EROSION_BRUSH_R; dx <= EROSION_BRUSH_R; dx++) {
            float d = sqrtf((float)(dx * dx + dy * dy));
            if (d > (float)EROSION_BRUSH_R) continue;
            total_w += 1.0f - d / (float)EROSION_BRUSH_R;
        }
    }
    if (total_w < 1e-6f) return;

    /* Second pass — subtract weighted amount from each disc cell. */
    for (int dy = -EROSION_BRUSH_R; dy <= EROSION_BRUSH_R; dy++) {
        int ny = cy + dy;
        if (ny < 0 || ny >= hm->h) continue;
        for (int dx = -EROSION_BRUSH_R; dx <= EROSION_BRUSH_R; dx++) {
            int nx = cx + dx;
            if (nx < 0 || nx >= hm->w) continue;
            float d = sqrtf((float)(dx * dx + dy * dy));
            if (d > (float)EROSION_BRUSH_R) continue;
            float w = 1.0f - d / (float)EROSION_BRUSH_R;
            hm->height[hidx(hm, nx, ny)] -= amount * w / total_w;
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Droplet — particle-based erosion.                                       *
 * ----------------------------------------------------------------------- */

typedef struct {
    float x, y;
    float dx, dy;
    float speed;
    float water;
    float sediment;
} Droplet;

static void droplet_simulate(Heightmap *hm, Droplet *d)
{
    for (int step = 0; step < DROPLET_MAX_STEPS; step++) {
        int xi = (int)d->x, yi = (int)d->y;
        if (xi < 0 || xi >= hm->w - 1 || yi < 0 || yi >= hm->h - 1) return;

        /* Stamp a water trail at the visited cell. */
        hm->water_trail[hidx(hm, xi, yi)] = 1.0f;

        /* Bilinear gradient. */
        float fx = d->x - (float)xi, fy = d->y - (float)yi;
        int idx = hidx(hm, xi, yi);
        float h00 = hm->height[idx];
        float h10 = hm->height[idx + 1];
        float h01 = hm->height[idx + hm->w];
        float h11 = hm->height[idx + hm->w + 1];
        float gx  = (h10 - h00) * (1.0f - fy) + (h11 - h01) * fy;
        float gy  = (h01 - h00) * (1.0f - fx) + (h11 - h10) * fx;

        /* Steering — keep some inertia, blend in -gradient. */
        d->dx = d->dx * INERTIA - gx * (1.0f - INERTIA);
        d->dy = d->dy * INERTIA - gy * (1.0f - INERTIA);

        /* Normalise to unit step length. */
        float len = sqrtf(d->dx * d->dx + d->dy * d->dy);
        if (len < 1e-4f) return;
        d->dx /= len; d->dy /= len;

        float new_x = d->x + d->dx;
        float new_y = d->y + d->dy;

        int new_xi = (int)new_x, new_yi = (int)new_y;
        if (new_xi < 0 || new_xi >= hm->w - 1 ||
            new_yi < 0 || new_yi >= hm->h - 1) return;

        float old_h = h00 * (1 - fx) * (1 - fy) + h10 * fx * (1 - fy)
                    + h01 * (1 - fx) *      fy  + h11 * fx *      fy;
        float new_h = h_bilinear(hm, new_x, new_y);
        float dh    = new_h - old_h;

        /* Carrying capacity. */
        float cap = (-dh) * d->speed * d->water * SEDIMENT_CAPACITY_K;
        if (cap < MIN_CAPACITY) cap = MIN_CAPACITY;

        if (d->sediment > cap || dh > 0.0f) {
            /* DEPOSIT.
             * - Going uphill (dh > 0): drop ≤ dh worth of sediment to
             *   fill the rise.
             * - Going downhill but oversaturated: shed (sed - cap)·rate. */
            float amount;
            if (dh > 0.0f) {
                amount = (dh < d->sediment) ? dh : d->sediment;
            } else {
                amount = (d->sediment - cap) * DEPOSIT_RATE;
            }
            d->sediment -= amount;

            /* Bilinear deposit at the four-corner footprint. */
            hm->height[idx]              += amount * (1.0f - fx) * (1.0f - fy);
            hm->height[idx + 1]          += amount *         fx  * (1.0f - fy);
            hm->height[idx + hm->w]      += amount * (1.0f - fx) *         fy;
            hm->height[idx + hm->w + 1]  += amount *         fx  *         fy;
        } else {
            /* ERODE — disc-weighted, capped at |dh|. */
            float wanted = (cap - d->sediment) * ERODE_RATE;
            float amount = (wanted < -dh) ? wanted : -dh;
            erode_brush(hm, d->x, d->y, amount);
            d->sediment += amount;
        }

        /* Energetics. */
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

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Heightmap hm;
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    float     time_secs;
    float     flash_t;
    int       hold_countdown;     /* set when budget reached, counts to 0 */
} Scene;

static void scene_rebuild(Scene *s)
{
    int seed = (int)hash3((int)(s->time_secs * 1000.0f),
                          s->hm.w, s->hm.h ^ 0x9E3779B9);
    heightmap_generate(&s->hm, seed);
    s->hold_countdown = 0;
    s->flash_t        = 1.0f;
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

/*
 * scene_tick — three phases per generation:
 *   ERODING  : droplets_done < DROPLETS_PER_GEN. Each tick:
 *              decay water_trail, then run K droplets.
 *   COMPLETE : droplets_done == budget. Decay continues; arm hold.
 *   HOLDING  : count down. When 0, regenerate.
 *
 * Speed knob scales droplets-per-tick linearly. With SPEED_DEF=8 →
 * 12 per tick → 720/sec → ~11 s to reach 8 000.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    s->flash_t   *= expf(-4.0f * dt);
    if (s->paused) return;

    Heightmap *hm = &s->hm;
    int total = hm->w * hm->h;

    /* Decay trails every tick (whether or not we're still eroding). */
    for (int i = 0; i < total; i++) {
        hm->water_trail[i] *= WATER_TRAIL_DECAY;
    }

    if (hm->droplets_done < DROPLETS_PER_GEN) {
        int per_tick = DROPLETS_PER_TICK_DEF * s->speed / SPEED_DEF;
        if (per_tick < 1) per_tick = 1;
        for (int i = 0; i < per_tick && hm->droplets_done < DROPLETS_PER_GEN; i++) {
            Droplet d;
            droplet_spawn(&d, hm->w, hm->h);
            droplet_simulate(hm, &d);
            hm->droplets_done++;
        }
        if (hm->droplets_done >= DROPLETS_PER_GEN) {
            s->hold_countdown = HOLD_TICKS;
        }
    } else {
        if (s->hold_countdown > 0) {
            s->hold_countdown--;
        } else {
            scene_rebuild(s);
        }
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols, rows;
    int map_w, map_h;
    int gx0, gy0;
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;
    int avail_w = s->cols;

    int mw = avail_w;
    int mh = avail_h;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;

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
static void render_terrain(const Screen *sc, const Scene *s)
{
    const Heightmap *hm = &s->hm;
    int twinkle_t = (int)s->time_secs;

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
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
                int phase = (x + y * 2 + (int)(s->time_secs * 3.0f)) & 3;
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
static void render_droplets(const Screen *sc, const Scene *s)
{
    const Heightmap *hm = &s->hm;

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
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
static void render_erosion(const Screen *sc, const Scene *s)
{
    const Heightmap *hm = &s->hm;

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
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
static void render_slope(const Screen *sc, const Scene *s)
{
    const Heightmap *hm = &s->hm;

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float gx, gy;
            cell_gradient(hm, x, y, &gx, &gy);
            float mag = sqrtf(gx * gx + gy * gy);
            int level = (int)(mag * SLOPE_SCALE);
            if (level < 0)         level = 0;
            if (level >= N_BIOMES) level = N_BIOMES - 1;

            char glyph;
            if (mag < 1e-4f) {
                glyph = '.';
            } else if (fabsf(gx) > fabsf(gy)) {
                /* gradient points uphill; water flows opposite. */
                glyph = (gx > 0) ? '<' : '>';
            } else {
                glyph = (gy > 0) ? '^' : 'v';
            }

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

static void scene_draw(const Screen *sc, const Scene *s)
{
    switch (s->current_pattern) {
    case PATTERN_TERRAIN:  render_terrain (sc, s);  break;
    case PATTERN_DROPLETS: render_droplets(sc, s);  break;
    case PATTERN_EROSION:  render_erosion (sc, s);  break;
    case PATTERN_SLOPE:    render_slope   (sc, s);  break;
    case N_PATTERNS:       break;       /* unreachable sentinel */
    }

    /* Regenerate flash. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->time_secs * 1000.0f);
        attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
        for (int sy = 2; sy < sc->rows - 1; sy += 2) {
            for (int sx = 0; sx < sc->cols; sx += 2) {
                if (((sx ^ sy ^ seed) & 7) == 0)
                    mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    const Heightmap *hm = &s->hm;
    bool eroding = (hm->droplets_done < DROPLETS_PER_GEN);
    const char *state_str;
    if (s->paused)        state_str = "PAUSED  ";
    else if (eroding)     state_str = "ERODING ";
    else                  state_str = "SETTLED ";

    /* Row 0 right — primary status. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " HYDRAULIC EROSION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + theme + ramp + counters. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-8s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 19;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    for (int i = 0; i < N_BIOMES; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ELEV_RAMP[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    int pct = (DROPLETS_PER_GEN > 0)
            ? (hm->droplets_done * 100) / DROPLETS_PER_GEN : 100;
    if (pct > 100) pct = 100;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  droplets:%5d/%-5d  %3d%% ",
             hm->droplets_done, (int)DROPLETS_PER_GEN, pct);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
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
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

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
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
