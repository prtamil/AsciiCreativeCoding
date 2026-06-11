/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lava_lamp.c — magma chamber with metaball blobs
 *
 * DEMO: A few large viscous "lava" blobs drift slowly inside a confined
 *       chamber. Each blob is a smooth metaball — a Gaussian-like
 *       scalar field centred on its position. The total field is the
 *       sum of every blob's contribution; cells whose field exceeds a
 *       threshold are rendered as molten material, with the field
 *       gradient driving glyph and colour intensity. The blobs heat
 *       at the floor (buoyant lift), cool near the ceiling (gentle
 *       sinking), and merge or split as they pass through one another
 *       — the field sums freely, so two close blobs become one larger
 *       lobe and slowly separate again. Slow time scale: a single
 *       blob rises in ~6 seconds, the whole scene is meditative
 *       rather than frenetic.
 *
 * Study alongside: raymarcher/metaballs.c (3-D version),
 *                  fluid/marching_squares.c (iso-contour extraction —
 *                  this file uses the same scalar-field idea but
 *                  threshold-fills the interior rather than tracing
 *                  the contour).
 *
 * Section map (layers):
 *   §1 config       — constants, knob ranges, colour-pair IDs
 *   §2 performance  — monotonic clock + sleep (frame cap lives in §6)
 *   §3 simulation   — PRNG, Blob + Lamp state, spawn / tick / buoyancy
 *   §4 logic        — pure field eval f(x,y)=Σ rᵢ²/dᵢ² + heat bucketing
 *   §5 render        — heat ramp, chamber walls, field raster, HUD, ncurses
 *   §6 app          — signals, the per-tick combine loop, key handling
 *
 * Keys:  [/]   blob count (3..10)
 *        -/+   field threshold (smaller = larger blobs)
 *        ,/.   buoyancy strength
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/lava_lamp.c \
 *       -o lava_lamp -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : N circular blobs, each with position (x, y), radius
 *                 r, and temperature T ∈ [0, 1]. A scalar field is
 *                 defined as
 *                     f(x, y) = Σ_i  rᵢ² / ((x - xᵢ)² + (y - yᵢ)²·k + ε²)
 *                 where `k` is the cell-aspect correction (terminal
 *                 cells are ~2× tall as wide, so the field is squashed
 *                 vertically by k = 4 = 2²). The renderer iterates
 *                 every screen cell, evaluates f, and emits a glyph
 *                 from the heat ramp when f > THRESHOLD.
 *
 *                 Buoyancy: a blob's vertical velocity is driven by
 *                 (T - T_ambient(y)), where T_ambient is cool at the
 *                 ceiling and hot at the floor. So hot blobs rise, cool
 *                 blobs sink — a slow oscillation. Damping prevents
 *                 unbounded acceleration. Blobs additionally take a
 *                 random horizontal walk to break perfect verticality.
 *
 * Data-structure: Lamp holds Blob[N_BLOBS_MAX] inline. Each Blob has
 *                 (x, y, vx, vy, r, temp). Field evaluation is on the
 *                 fly; no grid storage.
 *
 * Rendering     : Per frame: erase, raster every interior cell (sr, sc),
 *                 evaluate f, and where f > threshold pick a glyph + colour
 *                 from the field magnitude blended with the blobs' average
 *                 temperature (cell_shade → heat_bucket). The raster is
 *                 O(rows · cols · n_blobs) — at 24×80×8 ≈ 15k field
 *                 evaluations per frame; each is a few multiplies + a divide.
 *                 Easily 60 fps.
 *
 * References    :
 *   Metaball scalar field (§4 field_eval, §5 draw_field threshold-fill)
 *     [1] Blinn, "A Generalization of Algebraic Surface Drawing," ACM Trans.
 *         Graph. 1 (1982) — the original metaball / "blobby" surface paper;
 *         the f(x,y)=Σ rᵢ²/dᵢ² summation this file rasterises.
 *     [2] Wyvill, McPheeters & Wyvill, "Data Structure for Soft Objects,"
 *         The Visual Computer 2 (1986) — refined finite-support kernels.
 *     [3] Bloomenthal et al., "Introduction to Implicit Surfaces" (Morgan
 *         Kaufmann, 1997) — the textbook on blobby/implicit surfaces and how
 *         summed fields blend/merge, which is what makes blobs fuse & split.
 *     [4] Lorensen & Cline, "Marching Cubes," SIGGRAPH 1987 — the iso-surface
 *         threshold idea (marching-squares is its 2-D analog; cf. the study-
 *         alongside file).  Here we threshold-fill the interior, not trace it.
 *   Buoyancy / thermal convection (§3 blob_tick)
 *     [5] Tritton, "Physical Fluid Dynamics" (Oxford, 1988) — Rayleigh–Bénard
 *         convection and the Boussinesq buoyancy (force ∝ temperature anomaly)
 *         that a real lava lamp runs on; blob_tick is a lumped caricature.
 *   Rendering substrate
 *     [6] Bourke, "Character representation of grey scale images" (1997) —
 *         the luminance→ASCII ramp behind HEAT_GLYPH's intensity buckets (§5).
 *     [7] Fiedler, "Fix Your Timestep!", gafferongames.com — the dt-capped
 *         integration loop in §6.
 *     [8] Padala, "NCURSES Programming HOWTO" (TLDP) — colour pairs and the
 *         erase → draw → refresh frame model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ───────────────────────────────────── *
 *
 * Four real layers; two are intentionally absent.
 *
 *   LAYER        SECTION  MUTATES
 *   ────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — timing primitives (frame cap/fps in §6)
 *   SIMULATION   §3       Blob.{x,y,vx,vy,r,temp} for every live blob, and
 *                           Lamp.blobs/Lamp.n_blobs via spawn.  (Lamp.threshold/
 *                           buoyancy/theme/paused are NOT touched here — they
 *                           are user-event knobs, see below.)  Advances the
 *                           global PRNG via frand.
 *   LOGIC        §4       nothing — pure functions (field_eval, cell_shade,
 *                           heat_bucket)
 *   RENDER       §5       the terminal only (ncurses); READS Lamp, never
 *                           writes it.
 *
 *   EFFECTS  : ABSENT — glow / colour are derived at render time from the
 *              field magnitude and blob temperature (draw_field), never stored
 *              as separate cosmetic state.
 *   DELAYS   : ABSENT — the only pause is Lamp.paused, checked once at the
 *              combine point in main; there are no other timers or holds.
 *
 * LOGIC (§4) is provably uncorruptable from RENDER/EFFECTS: it does no
 * mutation and no I/O, so deleting or reordering any draw call cannot change
 * what field_eval / heat_bucket return.
 *
 * Per-tick combine order — main() (§6) is the ONLY place that advances state,
 * and lamp_tick() is the ONLY mutator of simulation state:
 *     1. PERFORMANCE  measure dt (capped at DT_CAP_S)      §6
 *     2. SIMULATION   lamp_tick(dt)  [skipped if paused]   §3
 *     3. PERFORMANCE  update smoothed fps                  §6
 *     4. RENDER       scene_draw()  (read-only)            §5
 *     5. PERFORMANCE  sleep to frame cap                   §6
 *
 * User events (key, reseed, theme, blob +/-, threshold, buoyancy, resize,
 * signal) MAY mutate Lamp / screen but are NOT part of the tick — they run in
 * the resize block and the getch() input drain at the top of the loop, before
 * the tick (main §6).
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS         30        /* slow scene; 30 fps is plenty       */

#define N_BLOBS_MAX        10
#define N_BLOBS_DEFAULT     6
#define N_BLOBS_MIN         3

#define BLOB_RADIUS_MIN     2.5f
#define BLOB_RADIUS_MAX     5.0f

/* Cell-aspect correction. y in cell space is squashed by ~2× vs x. */
#define ASPECT_K2           4.0f     /* k² where k = ASPECT_X = 2.0        */
#define FIELD_EPS2          0.5f     /* field softening ε² — caps the r²/d²
                                      * peak at a blob centre so it never
                                      * diverges (see §4 field_eval)        */

/* Field threshold — bigger threshold means smaller-looking blobs. */
#define THRESHOLD_DEFAULT   0.8f
#define THRESHOLD_MIN       0.3f
#define THRESHOLD_MAX       2.0f
#define THRESHOLD_STEP      0.1f

/* Buoyancy: vertical force = BUOYANCY · (T - T_ambient(y)). */
#define BUOYANCY_DEFAULT    8.0f
#define BUOYANCY_MIN        2.0f
#define BUOYANCY_MAX        20.0f
#define BUOYANCY_STEP       2.0f
#define DAMPING             1.0f     /* velocity damping per second        */
#define HORIZ_NOISE         3.0f     /* horizontal walk amplitude          */
#define WALL_RESTITUTION    0.5f     /* speed fraction kept on a wall bounce */

/* Temperature dynamics — heated at floor, cooled at ceiling. */
#define HEAT_GAIN           0.30f    /* temp/sec when at floor             */
#define HEAT_LOSS           0.20f    /* temp/sec when at ceiling           */
#define BOUND_MARGIN        2        /* keep blobs this many cells inside  */

#define DT_CAP_S            0.10f
#define N_THEMES            4
#define N_HEAT_STOPS        5        /* heat-ramp stops: glyph + colour buckets */

/* Cell shading (§5 draw_field) — blend weights (sum to 1) + glow cutoff. */
#define SHADE_INTENSITY_W   0.4f     /* weight of the interior field gradient */
#define SHADE_TEMP_W        0.6f     /* weight of the blobs' average temp     */
#define HOT_BUCKET          3        /* buckets ≥ this get A_BOLD (glow)      */

/* Colour pair IDs */
#define PAIR_HEAT_0  1   /* coolest                                       */
#define PAIR_HEAT_1  2
#define PAIR_HEAT_2  3
#define PAIR_HEAT_3  4
#define PAIR_HEAT_4  5   /* hottest                                       */
#define PAIR_WALL    6
#define PAIR_HUD     7
#define PAIR_HINT    8

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  performance — timing primitives (frame cap applied in §6)           */
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
/* §3  simulation — advances state (sole mutator of Blob / Lamp.blobs)      */
/* ═══════════════════════════════════════════════════════════════════════ */

/* PRNG utilities — mutate the global rand() state, so NOT pure (not §4). */
static float frand(void)        { return (float)rand() / (float)RAND_MAX; }
static float frand_signed(void) { return frand() * 2.0f - 1.0f; }

/* Blob — one metaball: a soft, summable scalar-field source (Blinn 1982 [1],
 * Wyvill 1986 [2]).
 *
 * WHY a metaball and not a sprite: each blob contributes a smooth falloff
 * field  r² / (dx² + dy²·k² + ε)  to every cell (§4 field_eval).  Because the
 * fields simply ADD, two nearby blobs merge into one lobe and separate again
 * with no special-case code — the organic "lava" merge/split is free.  Nothing
 * is rasterised onto the blob itself; its entire on-screen shape is recomputed
 * from these six numbers every frame, so there is no grid to store.
 *
 * DATA-FLOW (one blob, per tick — blob_tick, §3):
 *   y            ──▶ T_ambient(y)      cool at ceiling, hot at floor (linear)
 *   temp−T_amb   ──▶ vy                buoyancy: hotter-than-surroundings rises
 *   vy, vx       ──▶ x, y              integrate position; chamber walls bounce
 *   which half y ──▶ temp              heats near floor, cools near ceiling
 * Temperature and height thus chase each other, giving the slow rise/sink
 * oscillation — a lumped Boussinesq buoyancy (ref [5]).
 *
 * UNITS: x/y/vx/vy/r are in terminal CELLS; temp is dimensionless [0,1]. */
typedef struct {
    float x, y;     /* centre in cell space; y grows DOWNWARD (screen coords) */
    float vx, vy;   /* velocity (cells/s); vy < 0 = rising, vx = random walk   */
    float r;        /* metaball radius (cells): sets field strength r² & reach */
    float temp;     /* temperature ∈ [0,1]: 1 = hot (rises, white-hot colour),
                     * 0 = cold (sinks, dim colour).  Heated near the floor,
                     * cooled near the ceiling; also the radius-weighted colour
                     * key blended into the heat ramp in §4 field_eval / §5     */
} Blob;

static void blob_spawn(Blob *b, int rows, int cols)
{
    b->x   = (float)BOUND_MARGIN + frand() * (float)(cols - 2 * BOUND_MARGIN);
    b->y   = (float)BOUND_MARGIN + frand() * (float)(rows - 2 - 2 * BOUND_MARGIN);
    b->vx  = 0;
    b->vy  = 0;
    b->r   = BLOB_RADIUS_MIN + frand() * (BLOB_RADIUS_MAX - BLOB_RADIUS_MIN);
    b->temp = frand();
}

/* Ambient temperature target at height y: 1.0 at the floor, 0.0 at the
 * ceiling (linear).  A blob hotter than its surroundings rises, cooler sinks. */
static float ambient_temp(float y, int rows)
{
    float t = 1.0f - (y / (float)(rows - 2));
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return t;
}

/* Buoyancy + random walk: push vy toward rising when hotter than ambient
 * (Boussinesq, ref [5]); add a small horizontal jitter to break symmetry. */
static void blob_apply_buoyancy(Blob *b, float dt, float buoyancy, int rows)
{
    float t_ambient = ambient_temp(b->y, rows);
    b->vy += -buoyancy * (b->temp - t_ambient) * dt;   /* negative vy = up */
    b->vx += frand_signed() * HORIZ_NOISE * dt;
}

/* Viscous drag: shrink velocity by DAMPING per second. */
static void blob_apply_damping(Blob *b, float dt)
{
    float k = 1.0f - DAMPING * dt;
    if (k < 0) k = 0;
    b->vx *= k;
    b->vy *= k;
}

/* Explicit-Euler position update. */
static void blob_integrate(Blob *b, float dt)
{
    b->x += b->vx * dt;
    b->y += b->vy * dt;
}

/* Heat exchange with the chamber: warm in the lower half, cool in the upper. */
static void blob_exchange_heat(Blob *b, float dt, int rows)
{
    if (b->y > (float)rows * 0.5f) b->temp += HEAT_GAIN * dt;
    else                           b->temp -= HEAT_LOSS * dt;
    if (b->temp < 0) b->temp = 0;
    if (b->temp > 1) b->temp = 1;
}

/* Reflect off the chamber walls, keeping WALL_RESTITUTION of the speed. */
static void blob_bounce_walls(Blob *b, int rows, int cols)
{
    float xmin = (float)BOUND_MARGIN;
    float xmax = (float)(cols - BOUND_MARGIN);
    float ymin = (float)BOUND_MARGIN;
    float ymax = (float)(rows - 2 - BOUND_MARGIN);
    if (b->x < xmin) { b->x = xmin; b->vx = fabsf(b->vx) * WALL_RESTITUTION; }
    if (b->x > xmax) { b->x = xmax; b->vx = -fabsf(b->vx) * WALL_RESTITUTION; }
    if (b->y < ymin) { b->y = ymin; b->vy = fabsf(b->vy) * WALL_RESTITUTION; }
    if (b->y > ymax) { b->y = ymax; b->vy = -fabsf(b->vy) * WALL_RESTITUTION; }
}

/* blob_tick — one buoyancy-driven motion step, as the physics pipeline. */
static void blob_tick(Blob *b, float dt, float buoyancy, int rows, int cols)
{
    blob_apply_buoyancy(b, dt, buoyancy, rows);   /* hot rises, cool sinks */
    blob_apply_damping(b, dt);                    /* viscous drag          */
    blob_integrate(b, dt);                        /* move                  */
    blob_exchange_heat(b, dt, rows);              /* warm low, cool high   */
    blob_bounce_walls(b, rows, cols);             /* stay in the chamber   */
}

/* Lamp — the whole magma chamber in one aggregate (the WHAT + HOW + WHERE),
 * laid out as a table of contents.
 *
 * WHY a fixed pool: blobs[] is sized to N_BLOBS_MAX once and never grown or
 * freed; n_blobs is merely how many slots are live.  This keeps the hot path
 * malloc-free — the ']' key spawns into an existing slot and '[' lowers the
 * count.  Field evaluation is O(rows·cols·n_blobs) with no grid storage, so
 * the entire scene is these few structs plus the terminal.
 *
 * WHY these groups: fields are grouped by the CONCEPT they belong to, not by
 * the key that happens to change them.  theme is toggled from the same keyboard
 * as the physics knobs, but it is a RENDER choice (which palette) and so sits
 * apart from the simulation knobs.  Refs: metaballs [1][2][3]; buoyancy [5]. */
typedef struct {
    /* WHAT is simulated — the metaballs (only the first n_blobs are live) */
    Blob blobs[N_BLOBS_MAX]; /* fixed pool, never resized at runtime           */
    int  n_blobs;            /* live count ∈ [N_BLOBS_MIN..MAX] ('[' / ']' keys) */

    /* HOW the user drives the simulation — tunable physics knobs */
    float threshold;        /* field iso-level (§5 draw_field): a cell is
                             * "molten" where f > threshold, so HIGHER makes the
                             * blobs look smaller/tighter.  [THRESHOLD_MIN..MAX],
                             * stepped by the -/+ keys                          */
    float buoyancy;         /* gain on the temperature-driven lift in blob_tick
                             * (force ∝ buoyancy·(temp − T_ambient)).  Bigger =
                             * livelier rise/sink.  [BUOYANCY_MIN..MAX], , / .   */

    /* RENDER knob — palette selection (a render concept, not a sim knob) */
    int   theme;            /* index 0..N_THEMES-1 into HEAT_256 / HEAT_8 (§5);
                             * the t key cycles it and re-runs color_init       */

    /* run-state */
    int   paused;           /* nonzero → lamp_tick is skipped at the combine
                             * point, but RENDER still runs (p key toggles)     */
} Lamp;

/* User-event helper (reset/reseed/resize) — re-spawns all live blobs. */
static void lamp_reseed(Lamp *l, int rows, int cols)
{
    for (int i = 0; i < l->n_blobs; i++) blob_spawn(&l->blobs[i], rows, cols);
}

/* The ONE per-tick simulation advance: move every live blob. */
static void lamp_tick(Lamp *l, float dt, int rows, int cols)
{
    for (int i = 0; i < l->n_blobs; i++)
        blob_tick(&l->blobs[i], dt, l->buoyancy, rows, cols);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  logic — pure functions: no mutation, no I/O, readable in isolation   */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * field_eval — sum of inverse-square contributions from each blob.
 *   contribution_i = rᵢ² / ((x - xᵢ)² + (y - yᵢ)²·k² + ε²)
 * where k² is the cell-aspect squared (so the field is squashed
 * vertically and the rendered blobs read as round on screen).
 *
 * Returns total field magnitude at (x, y) in cell coordinates.
 *
 * Also fills *out_t_avg with the radius-weighted average temperature
 * of contributing blobs (for the heat-ramp colour). The bigger the
 * blob's contribution at this cell, the more its temperature dominates.
 */
static float field_eval(const Blob *blobs, int n_blobs,
                        float x, float y, float *out_t_avg)
{
    float total = 0.0f;
    float wsum  = 0.0f;
    float tw    = 0.0f;
    for (int i = 0; i < n_blobs; i++) {
        float dx = x - blobs[i].x;
        float dy = (y - blobs[i].y);
        float d2 = dx * dx + dy * dy * ASPECT_K2 + FIELD_EPS2;
        float w  = blobs[i].r * blobs[i].r / d2;
        total += w;
        wsum  += w;
        tw    += w * blobs[i].temp;
    }
    if (out_t_avg) *out_t_avg = (wsum > 1e-6f) ? (tw / wsum) : 0.0f;
    return total;
}

/* Quantise a [0,1] brightness to a heat-ramp bucket index 0..N_HEAT_STOPS-1. */
static int heat_bucket(float t)
{
    int b = (int)(t * 4.99f);   /* N_HEAT_STOPS - ε, maps [0,1] onto 0..4 */
    if (b < 0)                 b = 0;
    if (b > N_HEAT_STOPS - 1)  b = N_HEAT_STOPS - 1;
    return b;
}

/* Cell shade ∈ [0,1]: blend how deep inside a blob this cell is (interior
 * field gradient, normalised over [threshold, 3·threshold]) with the
 * contributing blobs' average temperature, so the core reads bright and cool
 * blobs read dim.  Caller has already checked f ≥ threshold. */
static float cell_shade(float f, float threshold, float t_avg)
{
    float intensity = (f - threshold) / (threshold * 2.0f);
    if (intensity < 0) intensity = 0;
    if (intensity > 1) intensity = 1;
    return SHADE_INTENSITY_W * intensity + SHADE_TEMP_W * t_avg;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  render — state → screen; reads only, never mutates sim state         */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Four distinct dominant hues. Cold end (bucket 0) is mid-bright in
 * every theme so that cool/sinking blobs stay visible without A_DIM
 * — see draw_field, which no longer dims the lower buckets. */
static const short HEAT_256[N_THEMES][N_HEAT_STOPS] = {
    /* 0 lava   — red → orange → yellow → white   */
    { 160, 196, 208, 220, 231 },
    /* 1 ocean  — sky blue → cyan → white         */
    {  39,  45,  51,  87, 231 },
    /* 2 toxic  — medium green → lime → white     */
    {  34,  82, 118, 154, 231 },
    /* 3 royal  — purple → pink → white           */
    {  91, 127, 165, 213, 231 },
};
static const short HEAT_8[N_THEMES][N_HEAT_STOPS] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < N_HEAT_STOPS; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    init_pair(PAIR_WALL, x256 ? 240 : COLOR_WHITE, -1);
    init_pair(PAIR_HUD,  x256 ? 226 : COLOR_YELLOW, -1);  /* top: bright yellow */
    init_pair(PAIR_HINT, x256 ?  51 : COLOR_CYAN,   -1);  /* bottom: bright cyan */
}

/* Heat ramp glyphs by intensity bucket (interior gradient). */
static const char HEAT_GLYPH[N_HEAT_STOPS] = { '`', '.', '*', 'o', '#' };

static void draw_walls(int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_WALL) | A_DIM);
    /* Top + bottom */
    for (int c = 0; c < cols; c++) {
        mvaddch(0,        c, (chtype)'_');
        if (rows - 2 >= 0) mvaddch(rows - 2, c, (chtype)'_');
    }
    /* Left + right */
    for (int r = 1; r < rows - 1; r++) {
        if (0 < cols)        mvaddch(r, 0,        (chtype)'|');
        if (cols - 1 < cols) mvaddch(r, cols - 1, (chtype)'|');
    }
    attroff(COLOR_PAIR(PAIR_WALL) | A_DIM);
}

static void draw_field(const Lamp *l, int rows, int cols)
{
    /* Threshold-fill every interior cell (inset 1 inside the chamber walls). */
    for (int sr = 1; sr < rows - 2; sr++) {
        for (int sc = 1; sc < cols - 1; sc++) {
            float t_avg;
            float f = field_eval(l->blobs, l->n_blobs,
                                 (float)sc, (float)sr, &t_avg);
            if (f < l->threshold) continue;          /* outside every blob */

            int bucket = heat_bucket(cell_shade(f, l->threshold, t_avg));

            /* Hot buckets glow; the colour value itself carries the gradient. */
            chtype attr = COLOR_PAIR(PAIR_HEAT_0 + bucket);
            if (bucket >= HOT_BUCKET) attr |= A_BOLD;
            attron(attr);
            mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
            attroff(attr);
        }
    }
}

/* HUD — data readout top-right, action keys bottom-left (bright + A_BOLD). */
static void draw_hud(int rows, int cols, const Lamp *l, double fps)
{
    char buf[160];
    snprintf(buf, sizeof buf,
             " blobs:%d  thresh:%.2f  buoy:%.1f  theme:%d  "
             "%5.1f fps  %s ",
             l->n_blobs, l->threshold, l->buoyancy, l->theme, fps,
             l->paused ? "PAUSED " : "running");
    int x = cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " [/]:blobs  -/+:thresh  ,/.:buoyancy  t:theme  "
             "r:reset  p:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Lamp *l, double fps)
{
    erase();
    draw_walls(rows, cols);
    draw_field(l, rows, cols);
    draw_hud(rows, cols, l, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  app — combine point (per-tick order) + user events + frame cap       */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Lamp g_lamp;

/* Apply one keypress (a user event, not part of the tick): mutate knobs /
 * run-state / palette.  Returns false on quit. */
static bool lamp_handle_key(Lamp *l, int ch, int rows, int cols)
{
    switch (ch) {
        case 'q': case 27: return false;
        case 'p': l->paused ^= 1; break;
        case 'r': lamp_reseed(l, rows, cols); break;
        case 't': l->theme = (l->theme + 1) % N_THEMES; color_init(l->theme); break;
        case '[':
            if (l->n_blobs > N_BLOBS_MIN) l->n_blobs--;
            break;
        case ']':
            if (l->n_blobs < N_BLOBS_MAX) {
                blob_spawn(&l->blobs[l->n_blobs], rows, cols);
                l->n_blobs++;
            }
            break;
        case '-':
            if (l->threshold + THRESHOLD_STEP <= THRESHOLD_MAX)
                l->threshold += THRESHOLD_STEP;
            break;
        case '+': case '=':
            if (l->threshold - THRESHOLD_STEP >= THRESHOLD_MIN)
                l->threshold -= THRESHOLD_STEP;
            break;
        case ',':
            if (l->buoyancy - BUOYANCY_STEP >= BUOYANCY_MIN)
                l->buoyancy -= BUOYANCY_STEP;
            break;
        case '.':
            if (l->buoyancy + BUOYANCY_STEP <= BUOYANCY_MAX)
                l->buoyancy += BUOYANCY_STEP;
            break;
    }
    return true;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_lamp.n_blobs         = N_BLOBS_DEFAULT;
    g_lamp.threshold = THRESHOLD_DEFAULT;
    g_lamp.buoyancy  = BUOYANCY_DEFAULT;
    g_lamp.theme     = 0;

    screen_init(g_lamp.theme);
    int rows = LINES, cols = COLS;
    lamp_reseed(&g_lamp, rows, cols);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        /* USER EVENT (out of tick): apply resize, then reseed for new bounds */
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            lamp_reseed(&g_lamp, rows, cols);
        }

        /* USER EVENT (out of tick): drain input, mutate knobs / run-state */
        int ch;
        while ((ch = getch()) != ERR)
            if (!lamp_handle_key(&g_lamp, ch, rows, cols)) g_running = 0;

        /* 1. PERFORMANCE — measure dt, capped to avoid spiral-of-death */
        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;

        /* 2. SIMULATION — the sole state advance (skipped while paused) */
        if (!g_lamp.paused) lamp_tick(&g_lamp, dt, rows, cols);

        /* 3. PERFORMANCE — smoothed fps estimate */
        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        /* 4. RENDER — read-only */
        scene_draw(rows, cols, &g_lamp, fps);

        /* 5. PERFORMANCE — sleep to the frame cap */
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
