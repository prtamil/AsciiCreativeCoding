/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_diffusion_gray_scott.c
 *   — Gray-Scott reaction-diffusion: 5 chemical-pattern presets.
 *
 * DEMO: A 2-D reaction-diffusion system simulating two chemicals U
 *       and V that diffuse at different rates and react via
 *       U + 2V → 3V. The system is seeded with a small blob of V in
 *       the centre; over a few seconds, V "eats" U outward, and the
 *       resulting front breaks up into intricate self-organising
 *       patterns. Five pattern presets — different choices of the
 *       feed and kill rates (F, k) — produce visually distinct
 *       outcomes:
 *         SPOTS   — mitosis-like spots that divide and multiply
 *         STRIPES — stable parallel stripes (zebra-like)
 *         MAZES   — connected labyrinth channels
 *         CORAL   — branching coral / dendrite growth
 *         WORMS   — slowly travelling worm-like fronts
 *       Press n/p to switch presets — the simulation resets each time
 *       so you watch the pattern self-organise from a fresh seed.
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — also a "field over the grid", but
 *       the field here EVOLVES via PDE rather than being sampled
 *       from a static noise function. Reaction-diffusion is what
 *       gives biological pigmentation patterns; Perlin/simplex give
 *       static landscape patterns.
 *   ../generational/cellular_automata_cave_4-5_rule_showcase.c — a
 *       discrete cousin: cellular automata are reaction-diffusion's
 *       binary-state equivalent. Rules are integer-state, but the
 *       morphology family is similar (spots, mazes, blobs).
 *
 * Section map:
 *   §1 config   — grid, presets, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 rd       — Gray-Scott step (5-point Laplacian)
 *   §6 patterns — 5 (F, k) presets
 *   §7 scene    — Field, scene state, sub-stepped per-frame update
 *   §8 screen   — ASCII render: density-graded glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (re-seed + perturb)
 *   n / N      next pattern preset
 *   p / P      previous pattern preset
 *   t / T      next / previous theme
 *   + / =      faster simulation (more sub-steps per frame)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra reaction_diffusion_gray_scott.c \
 *       -o gray_scott -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Gray-Scott reaction-diffusion (Gray & Scott, 1983).
 *                  Two chemical concentrations U and V on a 2-D grid,
 *                  evolving under a coupled PDE system:
 *
 *                    ∂U/∂t = D_u · ∇²U − U · V² + F · (1 − U)
 *                    ∂V/∂t = D_v · ∇²V + U · V² − (F + k) · V
 *
 *                  Reading right to left:
 *                    • Both U and V DIFFUSE (with different diffusion
 *                      constants D_u > D_v — V is heavier / slower).
 *                    • The REACTION U + 2V → 3V converts U into V at a
 *                      rate proportional to U·V². V "eats" U.
 *                    • U is FED at rate F · (1 − U) — pushed back
 *                      toward 1 wherever it dropped.
 *                    • V is KILLED at rate (F + k) · V — decays back
 *                      toward 0 wherever it isn't being created.
 *
 *                  The competition between these four processes — at
 *                  different rates F, k — produces a phase diagram
 *                  with dramatically different morphologies (Pearson
 *                  1993). We expose 5 of the named regions as
 *                  switchable presets.
 *
 *                  Numerical method: explicit forward-Euler with a
 *                  5-point Laplacian stencil. Time step dt = 1 (the
 *                  unit in which F and k are usually quoted), with
 *                  multiple sub-steps per render frame for visible
 *                  evolution. Boundary condition: NEUMANN (zero-flux,
 *                  i.e. clamp out-of-grid neighbours to the cell
 *                  itself). Periodic boundaries also work but produce
 *                  visible tile-seams when the central pattern
 *                  reaches the edges.
 *
 * Data-structure : Two pairs of float grids: (U, V) for the current
 *                  state and (U_next, V_next) for the buffer being
 *                  written. Swap (memcpy) at the end of each sub-step.
 *                  All preset-tuning lives in §6 patterns; the
 *                  simulation itself is parameter-agnostic.
 *
 * Rendering      : ASCII only. Each cell's V value drives a density
 *                  glyph ('.', '*', '#') and a 4-band colour. V
 *                  typically lives in roughly [0, 0.4] for these
 *                  presets, so we multiply by ~2.5 for visibility.
 *
 * Performance    : 5-point Laplacian + Euler step is ≈ 12 multiply-adds
 *                  per cell per sub-step. With 4 sub-steps at 60 Hz on
 *                  a 200×56 grid that's ~32 M ops/sec — well within
 *                  budget on any modern CPU.
 *
 * References     : • Gray, P. & Scott, S.K. (1983) — "Autocatalytic
 *                    reactions in the isothermal, continuous stirred
 *                    tank reactor", Chemical Engineering Science.
 *                  • Pearson, J.E. (1993) — "Complex Patterns in a
 *                    Simple System". The phase diagram showing 11
 *                    distinct morphologies; SCIENCE 261:189–192.
 *                  • Karl Sims — interactive Gray-Scott explorer
 *                    (lookup: "Karl Sims reaction diffusion") with
 *                    parameter sliders and the famous "α through κ"
 *                    pattern names.
 *                  • Robert Munafo — exhaustive Gray-Scott references
 *                    and pattern catalogue:
 *                    https://mrob.com/pub/comp/xmorphia/
 *                  • Wikipedia — "Reaction–diffusion system":
 *                    https://en.wikipedia.org/wiki/Reaction%E2%80%93diffusion_system
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two chemicals (U and V) compete on a grid. U is plentiful and
 * spreads slowly; V is rare and spreads slower still, but it can
 * "eat" U and reproduce. Add a slow "feed" of fresh U from outside
 * and a slow "kill" of V, and you have a system that perpetually
 * fights itself into self-organising patterns. Different F (feed)
 * and k (kill) rates change which morphology dominates: spots,
 * stripes, mazes, coral, worms — all from the same equations.
 *
 * Crucially, the patterns are NOT random — they emerge
 * deterministically from physical reaction kinetics. This is one of
 * Alan Turing's "morphogens": the chemical mechanism behind animal
 * pigmentation, fingerprint formation, and many biological textures.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture an ocean of U with a tiny droplet of V at the centre. V is
 * voracious — it converts nearby U into more V at a rate
 * proportional to U·V². At first the droplet just grows outward.
 * But V also dies (at rate F+k), and U gets refreshed (at rate F).
 * If V's spread is ROUGHLY in balance with V's death, the front
 * stops being a smooth circle — small fluctuations get amplified
 * because curvy regions of the V-front have more or less U to consume.
 * That's the morphogenic instability.
 *
 * Once the instability hits, the system evolves toward a stable
 * pattern that depends on F and k:
 *   F LOW, k LOW   — V wins everywhere, pattern dies
 *   F MID, k MID   — instability balances → the interesting morphologies
 *   F HIGH, k HIGH — V dies fast, pattern collapses to U-only
 *
 * The interesting region (Pearson's diagram) is a thin curving sliver
 * in (F, k) space. We pre-pick 5 well-known points in that sliver
 * and label them.
 *
 * Visible layers:
 *   • V CONCENTRATION — every cell's brightness shows local V level.
 *   • PALETTE BAND — quartile of V drives the colour band, so dim,
 *     mid, and bright cells get different colours.
 *   • Time evolution — the pattern develops over a few seconds from
 *     a tiny seed, then often reaches a quasi-stable configuration
 *     (spots/stripes) or keeps drifting (worms).
 *
 * ALGORITHM IN STEPS  (per sub-step)
 * ──────────────────
 *  For every cell (x, y):
 *    1. lap_U = U(x-1,y) + U(x+1,y) + U(x,y-1) + U(x,y+1) − 4·U(x,y)
 *    2. lap_V = same for V
 *    3. r = U(x,y) · V(x,y)²
 *    4. U_next(x,y) = U + dt · (D_u · lap_U − r + F · (1 − U))
 *    5. V_next(x,y) = V + dt · (D_v · lap_V + r − (F + k) · V)
 *  After all cells done: swap U ← U_next, V ← V_next.
 *
 *  Run several sub-steps per render frame so the chemistry evolves
 *  fast enough to be watchable but stable enough to not numerically
 *  blow up.
 *
 * KEY FORMULAS
 * ────────────
 *  Diffusion (5-point Laplacian) :
 *      ∇² f(x,y) ≈ f(x-1,y) + f(x+1,y) + f(x,y-1) + f(x,y+1) − 4f(x,y)
 *  Reaction term                : r = U · V²
 *  U update                     : U' = U + dt · (D_u · ∇²U − r + F·(1−U))
 *  V update                     : V' = V + dt · (D_v · ∇²V + r − (F+k)·V)
 *  Stability bound (CFL-ish)    : dt · (D_u + D_v) · 4 ≲ 1
 *  Steady U=1, V=0 fixed point  : trivial; perturb to escape it
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • EXPLICIT EULER STABILITY. If dt is too big, the diffusion step
 *    blows up — values go negative or to infinity. Stable choice:
 *    dt = 1.0 with D_u = 0.16, D_v = 0.08 (the standard Gray-Scott
 *    values). Cranking up sub-steps via +/- gives more simulation
 *    progress per render frame WITHOUT changing dt itself.
 *
 *  • SEEDING MATTERS. Starting from U=1, V=0 everywhere is a stable
 *    fixed point — nothing happens forever. Seed a small central
 *    region with V=0.5, U=0.5 to break the symmetry. Without the
 *    seed, the screen stays blank.
 *
 *  • BOUNDARY ARTEFACTS. NEUMANN (zero-flux) is the natural choice —
 *    it models a closed petri dish. Periodic boundaries can also
 *    work and avoid the edge "stickiness" of NEUMANN, but they
 *    introduce wrap-around symmetry that interferes with the
 *    pattern formation.
 *
 *  • SUB-STEP COUNT VS VISIBLE PROGRESS. With 1 sub-step per render
 *    frame the pattern barely moves; with 16 it moves so fast you
 *    can't watch it form. 4–8 is the sweet spot.
 *
 *  • CHANGING (F, k) MID-RUN. Switching presets without resetting
 *    can be visually interesting (the pattern morphs gradually) but
 *    confuses the showcase. We reset on every preset switch so each
 *    pattern starts from a fresh seed.
 *
 *  • GRID SIZE. Patterns scale with the simulation — bigger grid =
 *    bigger features, slower convergence. The 200×56 we use is
 *    enough to see one or two full instances of each pattern.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At t = 0, you should see a small bright blob in the centre
 *    (the V seed) on an otherwise blank field.
 *  • After ~2 seconds, the blob expands and the front becomes
 *    irregular — instability has kicked in.
 *  • SPOTS preset: dots dividing into more dots over time.
 *  • STRIPES preset: stripes form parallel to the seed's growth
 *    direction. Random noise in the seed gives random stripe
 *    orientation.
 *  • MAZES preset: looks visually similar to STRIPES but the
 *    "stripes" connect laterally into a maze topology.
 *  • CORAL preset: branching, dendritic growth from the seed
 *    outward — like coral or frost on a window.
 *  • WORMS preset: elongated regions that slowly translate across
 *    the field.
 *  • If everything goes BLANK: F or k is in the "V dies" region —
 *    the preset values are wrong.
 *  • If everything goes SOLID: F or k is in the "V wins" region.
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
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Sub-steps per render frame. More = faster pattern development.
     * 6 lets a pattern emerge over ~10 seconds of wall time. */
    SUB_STEPS_MIN     =   1,
    SUB_STEPS_DEF     =   6,
    SUB_STEPS_MAX     = 128,

    /* Hard reset cadence. Without a reset, stable patterns sit
     * forever after they form. Re-seed every ~30 s. */
    RESET_TICKS_DEF   = 30 * 60,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* Standard Gray-Scott diffusion rates. */
#define GS_DU               0.16f
#define GS_DV               0.08f
#define GS_DT               1.0f

/* Render scaling: V values typically peak around 0.35-0.45 in the
 * pattern regions; multiply for visibility before mapping to glow. */
#define V_RENDER_GAIN       2.5f

#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern presets — each is a different (F, k) point in Pearson's
 * morphology phase diagram. Cycle with n/p; switching resets the
 * simulation so the chosen pattern starts forming from a fresh seed.
 *
 * Values pulled from the canonical Pearson 1993 paper and Karl Sims'
 * interactive explorer.
 */
typedef enum {
    PATTERN_SPOTS   = 0,
    PATTERN_STRIPES = 1,
    PATTERN_MAZES   = 2,
    PATTERN_CORAL   = 3,
    PATTERN_WORMS   = 4,
    N_PATTERNS      = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SPOTS:   return "SPOTS  ";
    case PATTERN_STRIPES: return "STRIPES";
    case PATTERN_MAZES:   return "MAZES  ";
    case PATTERN_CORAL:   return "CORAL  ";
    case PATTERN_WORMS:   return "WORMS  ";
    default:              return "?      ";
    }
}

typedef struct {
    float F;
    float k;
} GSPreset;

static const GSPreset gs_presets[N_PATTERNS] = {
    /*  F        k       — well-known Gray-Scott parameter pairs */
    { 0.035f, 0.065f },   /* SPOTS    — mitosis (Pearson γ region)  */
    { 0.020f, 0.050f },   /* STRIPES  — stable stripes              */
    { 0.029f, 0.057f },   /* MAZES    — labyrinth                   */
    { 0.054f, 0.063f },   /* CORAL    — branching                   */
    { 0.078f, 0.061f },   /* WORMS    — slowly-travelling worms     */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names. Each defines 4 palette colours (used for
 * banding the V concentration) plus a flash accent.
 */
typedef struct {
    const char *name;
    short       band[4];
    short       flash;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  17,   33,  220,  231 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 234,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  52,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
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
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  rd — Gray-Scott reaction-diffusion solver                          */
/* ===================================================================== */

/*
 * Field — simulation state for the Gray-Scott system.
 *
 *   U[]     : per-cell concentration of the "fed" chemical
 *   V[]     : per-cell concentration of the "consumed" chemical
 *   U_next, V_next : double-buffer for the explicit-Euler step
 *   F, k    : feed and kill rates from the active preset
 *   step_count : total sub-steps run since seed (for HUD)
 *
 * Boundary condition: NEUMANN (zero-flux). Out-of-grid neighbours
 * are treated as equal to the central cell, so no flux leaves the
 * box. Equivalent to a sealed petri dish.
 */
typedef struct {
    int    w, h;
    int    total_cells;

    float  U     [CELLS_MAX];
    float  V     [CELLS_MAX];
    float  U_next[CELLS_MAX];
    float  V_next[CELLS_MAX];

    float  F, k;
    int    step_count;

    float  supernova_glow_t;
    int    reset_countdown;
} Field;

static inline int field_idx(const Field *f, int x, int y) { return y * f->w + x; }

/*
 * field_seed — initialise to the trivial fixed point (U = 1, V = 0)
 * everywhere, then plant a small centred V-blob with reduced U.
 * Without this seed the system never escapes its trivial steady
 * state.
 *
 * Random small jitter on U is added so the early instability has
 * something to amplify — without jitter the seeded blob stays
 * perfectly symmetric and only forms concentric rings.
 */
static void field_seed(Field *f)
{
    int n = f->total_cells;
    for (int i = 0; i < n; i++) {
        f->U[i] = 1.0f;
        f->V[i] = 0.0f;
    }
    /* Small central seed. */
    int cx = f->w / 2;
    int cy = f->h / 2;
    int rad = (f->w < f->h ? f->w : f->h) / 12;
    if (rad < 3) rad = 3;
    for (int dy = -rad; dy <= rad; dy++) {
        for (int dx = -rad; dx <= rad; dx++) {
            int x = cx + dx;
            int y = cy + dy;
            if (x < 0 || x >= f->w || y < 0 || y >= f->h) continue;
            if (dx * dx + dy * dy > rad * rad) continue;
            int idx = field_idx(f, x, y);
            f->U[idx] = 0.5f;
            f->V[idx] = 0.25f;
        }
    }
    /* Salt the field with a tiny amount of random U-jitter so symmetry
     * breaks naturally rather than producing concentric rings. */
    for (int i = 0; i < n; i++) {
        float jitter = (float)(rand() % 100 - 50) * 0.0002f;
        f->U[i] += jitter;
    }
    f->step_count = 0;
}

/*
 * field_step — run one Gray-Scott sub-step. Forward Euler with a
 * 5-point Laplacian. Neumann boundaries: out-of-grid → same value
 * as centre cell (zero-flux).
 */
static void field_step(Field *f)
{
    const float Du = GS_DU;
    const float Dv = GS_DV;
    const float dt = GS_DT;
    const float F  = f->F;
    const float k  = f->k;
    const int   w  = f->w;
    const int   h  = f->h;

    for (int y = 0; y < h; y++) {
        int yu = (y > 0)     ? y - 1 : y;
        int yd = (y < h - 1) ? y + 1 : y;
        for (int x = 0; x < w; x++) {
            int xl = (x > 0)     ? x - 1 : x;
            int xr = (x < w - 1) ? x + 1 : x;

            int idx = y  * w + x;
            int il  = y  * w + xl;
            int ir  = y  * w + xr;
            int iu  = yu * w + x;
            int id  = yd * w + x;

            float u = f->U[idx];
            float v = f->V[idx];

            float lap_u = f->U[il] + f->U[ir] + f->U[iu] + f->U[id] - 4.0f * u;
            float lap_v = f->V[il] + f->V[ir] + f->V[iu] + f->V[id] - 4.0f * v;
            float reaction = u * v * v;

            f->U_next[idx] = u + dt * (Du * lap_u - reaction + F * (1.0f - u));
            f->V_next[idx] = v + dt * (Dv * lap_v + reaction - (F + k) * v);
        }
    }
    /* Swap buffers. memcpy is fine; the arrays are only ~45 KB each
     * for our worst-case grid. */
    memcpy(f->U, f->U_next, (size_t)(w * h) * sizeof(float));
    memcpy(f->V, f->V_next, (size_t)(w * h) * sizeof(float));
    f->step_count++;
}

/* ===================================================================== */
/* §6  patterns — preset (F, k) plug-ins                                  */
/* ===================================================================== */

/*
 * field_apply_preset — copy a preset's (F, k) into the simulation.
 * Doesn't reset; caller decides whether to re-seed.
 */
static void field_apply_preset(Field *f, Pattern p)
{
    if (p < 0 || p >= N_PATTERNS) p = PATTERN_SPOTS;
    f->F = gs_presets[p].F;
    f->k = gs_presets[p].k;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Field   F;
    bool    paused;
    int     sub_steps;
    int     current_theme;
    Pattern current_pattern;
} Scene;

static void field_reset(Field *f, int w, int h)
{
    f->w = w;
    f->h = h;
    f->total_cells = w * h;
    f->reset_countdown = RESET_TICKS_DEF;
    f->supernova_glow_t = 1.0f;
    field_seed(f);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    field_apply_preset(&s->F, s->current_pattern);
    field_reset(&s->F, mw, mh);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->sub_steps       = SUB_STEPS_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_SPOTS;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;

    float decay_n = expf(-SUPERNOVA_DECAY * dt);
    f->supernova_glow_t *= decay_n;

    /* Run sub_steps Gray-Scott updates. */
    for (int i = 0; i < s->sub_steps; i++) {
        field_step(f);
    }

    f->reset_countdown--;
    if (f->reset_countdown <= 0) {
        field_reset(f, f->w, f->h);
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Field *f = &s->F;

    int gx0 = (cols - f->w) / 2;
    int gy0 = ((rows - 3) - f->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    for (int y = 0; y < f->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < f->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = field_idx(f, x, y);
            float ng = f->supernova_glow_t;
            float vraw = f->V[idx];

            /* Map V to glow [0, 1]. V typically peaks at ~0.4 for our
             * presets; multiply for visibility, then clamp. */
            float g = vraw * V_RENDER_GAIN;
            if (g < 0.0f) g = 0.0f;
            if (g > 1.0f) g = 1.0f;

            int  pair, attr;
            char glyph;

            if (ng > GLOW_THRESHOLD) {
                if (((x ^ y) & 3) != 0 && g <= GLOW_THRESHOLD) continue;
                pair  = PAIR_SUPERNOVA;
                attr  = A_BOLD;
                glyph = '*';
            } else if (g > GLYPH_HIGH_THRESH) {
                int band = (int)(g * 3.999f) & 3;
                pair  = PAIR_BAND_BASE + band;
                attr  = A_BOLD;
                glyph = '#';
            } else if (g > GLYPH_MID_THRESH) {
                int band = (int)(g * 3.999f) & 3;
                pair  = PAIR_BAND_BASE + band;
                attr  = A_BOLD;
                glyph = '*';
            } else if (g > GLOW_THRESHOLD) {
                int band = (int)(g * 3.999f) & 3;
                pair  = PAIR_BAND_BASE + band;
                attr  = A_NORMAL;
                glyph = '.';
            } else {
                continue;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Field *f = &s->F;
    const char *state_str = s->paused
                          ? "PAUSED "
                          : pattern_name(s->current_pattern);

    float reset_secs = (float)f->reset_countdown / 60.0f;
    if (reset_secs < 0.0f) reset_secs = 0.0f;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  sub:%-3d  step:%-7d ",
             fps, sim_fps, state_str, s->sub_steps, f->step_count);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " GRAY-SCOTT REACTION-DIFFUSION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — pattern + theme + parameters. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 9;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_BAND_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  F=%.4f  k=%.4f  Du=%.2f  Dv=%.2f  map:%dx%d ",
             (double)f->F, (double)f->k,
             (double)GS_DU, (double)GS_DV, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " .:low  *:mid  #:high | n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 3;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        if (s->sub_steps < SUB_STEPS_MAX) s->sub_steps *= 2;
        if (s->sub_steps > SUB_STEPS_MAX) s->sub_steps = SUB_STEPS_MAX;
        break;
    case '-':
        s->sub_steps /= 2;
        if (s->sub_steps < SUB_STEPS_MIN) s->sub_steps = SUB_STEPS_MIN;
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
        scene_reset(s, app->map_w, app->map_h);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_reset(s, app->map_w, app->map_h);
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
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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
