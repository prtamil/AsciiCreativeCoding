/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * signed_distance_field_jfa_showcase.c
 *
 * Scatter a handful of "seed" points on a grid, then for every other
 * cell figure out which seed is nearest and how far away it is. That
 * map of distances is the building block behind crisp scaled fonts,
 * Voronoi cell diagrams, glows, and outlines. We compute it the fast
 * way using the Jump Flooding Algorithm, then draw the same answer 30
 * different ways. The seeds drift around slowly so the picture moves.
 *
 * The clever part — Jump Flooding — works like rumour spreading: each
 * cell asks a few far-off neighbours "which seed do you know about?"
 * and keeps the closest one it hears. Starting with big jumps and
 * halving them each round, the right answer reaches every cell in only
 * log2(N) rounds instead of checking every seed against every cell.
 *
 * Reference for the algorithm:
 *   Rong & Tan (2006), "Jump Flooding in GPU with Applications to
 *   Voronoi Diagram and Distance Transform", I3D'06.
 *
 * Sister files worth comparing:
 *   ./worley_cellular_noise.c        — same Voronoi look, but worked
 *       out on the fly per query instead of precomputed.
 *   ../generational/voronoi_region_map.c — precomputed Voronoi by a
 *       different method (no animation).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra signed_distance_field_jfa_showcase.c \
 *       -o sdf_jfa -lncurses -lm
 */

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

    /* A "no seed here yet" marker. We need a value no real grid
     * coordinate could ever be; 32767 is way past the biggest grid. */
    JFA_SENTINEL      = INT16_MAX,

    /* How many seed points to scatter. Kept small so the cells are big
     * and you can actually watch the flooding fill them in. */
    N_SEEDS           = 20,

    /* Color pair indices. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,
};

/*
 * How the seeds drift. Each seed slowly circles a fixed home spot.
 *   WOBBLE_RADIUS_CELLS — how far it strays from home, in cells. About
 *                          5 keeps the cell boundaries visibly moving
 *                          without seeds bumping into each other.
 *   WOBBLE_RATE         — how fast it circles. Slow on purpose: faster
 *                          and you'd see the motion jump cell-to-cell
 *                          instead of gliding.
 */
#define WOBBLE_RADIUS_CELLS  5.0f
#define WOBBLE_RATE          0.5f

/* When we turn a seed's smooth (decimal) position into a whole-number
 * grid cell, add 0.5 first so it rounds to the nearest cell instead of
 * always rounding down — otherwise it would twitch by one at the edges. */
#define ROUND_TO_NEAREST_BIAS    0.5f

/* A tiny slack used when finding the "ridge" line between cells. Two
 * cells with the exact same distance shouldn't keep cancelling each
 * other out and making the line flicker, so a neighbour must be clearly
 * farther (by more than this) to count. Too small to see on screen. */
#define MEDIAL_AXIS_TOLERANCE    0.01f

/* A stand-in for "infinitely far" while a cell is still hunting for its
 * nearest seed, so the very first real candidate always beats it. Any
 * actual distance on this grid is far smaller. */
#define JFA_INFINITE_SQ_DISTANCE INT32_MAX

/* Each cell's brightness (0 to 1) gets sorted into one of 4 color
 * tiers. The 3.999 gain stops a full-brightness 1.0 from spilling into
 * a 5th tier. */
#define N_PALETTE_BANDS     4
#define PALETTE_BAND_MASK   3
#define GLOW_TO_BAND_GAIN   3.999f

/* Below this brightness a cell is left blank. */
#define GLOW_THRESHOLD      0.05f

/* The HUD reserves rows at top (info) and bottom (key reminders). */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* How fast the seeds drift, adjustable with +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      8
#define DRIFT_MULT_MAX      16

/* Brightness cutoffs for picking which ASCII character to draw. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/* The OUTLINE pattern draws a single ring at this distance from a seed. */
#define OUTLINE_DISTANCE    8.0f
#define OUTLINE_THICKNESS   1.5f    /* how wide the ring is, each side  */

/*
 * The 30 ways we draw the same distance map, grouped into 6 tiers from
 * simplest to most involved. Flip through them with n/p.
 *
 * This list must stay in the same order as noise_patterns[] in §6 —
 * the compiler checks the count matches via the [N_PATTERNS] array.
 *
 *   Tier 1 DISTANCE  : plain distance, shaded a few different ways
 *   Tier 2 BANDS     : repeating rings carved out of the distance
 *   Tier 3 VORONOI   : color each cell by which seed owns it
 *   Tier 4 CONTOURS  : draw rings/glows at chosen distances
 *   Tier 5 STRUCTURE : the cell skeleton — edges, corners, ridges
 *   Tier 6 ANIMATED  : effects that pulse and sweep over time
 */
typedef enum {
    /* Tier 1 — plain distance, shaded different ways */
    PATTERN_SDF = 0,
    PATTERN_INVERSE,
    PATTERN_SQRT,
    PATTERN_SQUARED,
    PATTERN_CAPPED,
    /* Tier 2 — repeating rings */
    PATTERN_RINGS,
    PATTERN_SAW,
    PATTERN_ZEBRA,
    PATTERN_WAVE,
    PATTERN_SHELLS,
    /* Tier 3 — color by which seed owns the cell */
    PATTERN_CELLS,
    PATTERN_CRACKLE,
    PATTERN_SPECKLE,
    PATTERN_RAINBOW,
    PATTERN_TWINKLE,
    /* Tier 4 — rings and glows at chosen distances */
    PATTERN_OUTLINE,
    PATTERN_DUAL,
    PATTERN_SHELL,
    PATTERN_HALO,
    PATTERN_AURA,
    /* Tier 5 — the cell skeleton (edges, corners, ridges) */
    PATTERN_EDGES,
    PATTERN_VERTICES,
    PATTERN_INTERIOR,
    PATTERN_WEAVE,
    PATTERN_SKELETON,
    /* Tier 6 — effects that move over time */
    PATTERN_PULSE,
    PATTERN_SCAN,
    PATTERN_ECHO,
    PATTERN_PLASMA,
    PATTERN_CHAOS,
    N_PATTERNS,
} Pattern;

/* Defined in §6 next to the pattern table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* If a frame ever takes longer than this (e.g. the program was paused
 * by the OS), pretend it was only this long so the sim doesn't try to
 * catch up all at once. See Glenn Fiedler, "Fix Your Timestep!". */
#define MAX_FRAME_DT_NS    (100 * NS_PER_MS)
#define RENDER_FPS_TARGET  60

/*
 * Theme — a named color scheme, one row of themes[] below.
 *
 * The whole point is to keep "what colors to use" separate from the
 * drawing code. The renderer just says "this cell is brightness tier
 * 0..3"; the theme decides what those four tiers actually look like.
 * Pressing t/T swaps in a different theme without touching any of the
 * math, giving the same animation ten different moods.
 *
 *   name    : short label shown in the HUD (padded to line up).
 *   band[4] : four colors, darkest first, brightest last — one for
 *             each brightness tier. These are xterm 256-color codes.
 *             All four must come from the bright half of the palette
 *             or the dimmest one vanishes against a black terminal
 *             (see CLAUDE.md "Theme Palette Brightness"). They're
 *             `short` only because the ncurses init_pair call wants
 *             that type.
 */
typedef struct {
    const char *name;          /* HUD label, padded to line up        */
    short       band[4];       /* 4 colors, darkest -> brightest      */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  17,   33,  220,  231 } },
    { "MATRIX",  {  22,   34,   46,  118 } },
    { "NOVA",    {  53,  129,  201,  219 } },
    { "MONO",    { 234,  244,  250,  254 } },
    { "OCEAN",   {  17,   33,   39,   51 } },
    { "FIRE",    {  52,  124,  208,  226 } },
    { "EARTH",   {  58,  100,  173,  230 } },
    { "FOREST",  {  22,   28,   64,  144 } },
    { "DESERT",  {  94,  130,  173,  222 } },
    { "ARCTIC",  {  18,   39,  159,  231 } },
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

/* Does this terminal have the full 256-color palette? */
static inline bool terminal_supports_256_colours(void) { return COLORS >= 256; }

/* Snap a bad theme number back to theme 0 so nothing crashes. */
static inline int theme_clamp_index(int idx)
{
    return (idx < 0 || idx >= N_THEMES) ? 0 : idx;
}

/* Load a theme's four colors into the slots the renderer draws with. */
static void theme_bind_palette_256(const Theme *t)
{
    for (int band = 0; band < N_PALETTE_BANDS; band++)
        init_pair(PAIR_BAND_BASE + band, t->band[band], -1);
}

/* For old terminals with only 8 colors: themes all look the same, but
 * we still give four distinct tiers so the picture stays readable. */
static void theme_bind_palette_fallback_8color(void)
{
    static const short ansi_band_ramp[N_PALETTE_BANDS] = {
        COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
    };
    for (int band = 0; band < N_PALETTE_BANDS; band++)
        init_pair(PAIR_BAND_BASE + band, ansi_band_ramp[band], -1);
}

/* Switch the live colors to theme number idx. Everything drawn after
 * this picks up the new palette automatically. */
static void theme_apply(int idx)
{
    int safe_idx = theme_clamp_index(idx);

    if (terminal_supports_256_colours())
        theme_bind_palette_256(&themes[safe_idx]);
    else
        theme_bind_palette_fallback_8color();
}

/* The HUD's own colors (bright yellow + cyan). Kept apart from the
 * themes because the on-screen text should always stay readable no
 * matter which theme the animation is using. */
static void hud_bind_pairs(void)
{
    if (terminal_supports_256_colours()) {
        init_pair(PAIR_HUD,   226, -1);     /* bright yellow */
        init_pair(PAIR_HINT,   51, -1);     /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* One-time color setup at startup. */
static void color_init(void)
{
    start_color();
    use_default_colors();        /* let cells keep the terminal's bg */
    hud_bind_pairs();
    theme_apply(0);
}

/* ===================================================================== */
/* §5  algorithm + data + animation + optimization                        */
/* ===================================================================== */

/*
 * §5 is split into four layers so the core algorithm reads cleanly:
 *
 *   §5a  ALGORITHM — the Jump Flooding itself, working purely on
 *                    whole-number seed positions. No motion, no colors.
 *   §5b  DATA      — the Seed: a point with a home spot plus drift state.
 *   §5c  ANIMATION — nudging each seed around its home over time.
 *   §5d  READOUT   — looking up "how far is this cell from its seed?"
 *                    with a smoothing trick for nicer-looking gradients.
 *
 * Both structs are declared first so the functions can use each other.
 */

/* ---------- §5: types ------------------------------------------------- */

/*
 * JFAGrid — the grid the Jump Flooding works on.
 *
 * For every cell it remembers the location of the nearest seed it
 * knows about so far. The flooding keeps improving that guess over a
 * handful of passes; when it's done, each cell holds the true nearest
 * seed, and the distance to it is just one square-root away.
 *
 * One of these lives on Scene (§7). It gets rebuilt every tick as the
 * seeds drift, and every drawing pattern reads from it.
 *
 * Why two copies of the data (the "ping-pong"):
 *   A flooding pass needs to read the whole grid as it was BEFORE the
 *   pass while writing the improved values somewhere else — reading and
 *   writing the same cells mid-pass would give wrong answers. So we keep
 *   two buffers: read from one, write to the other, then swap. After all
 *   the passes the final answer ends up in nearest_x/nearest_y (a
 *   copy-back at the end handles the case where it landed in scratch).
 *
 *   w, h         : grid size in cells; set before flooding starts.
 *   nearest_x[]  : for each cell, where its nearest seed is. Value
 *   nearest_y[]    JFA_SENTINEL means "no seed found yet" — you only
 *                  see that briefly at the start; once flooding finishes
 *                  every cell has a real answer. This is what the
 *                  drawing code reads.
 *   scratch_x[]  : spare buffer the flooding writes into, then swaps
 *   scratch_y[]    with the pair above. Not used outside §5a.
 *
 * Whole thing is about 89 KB, all reserved up front — no allocation
 * while running. int16_t is plenty: grids never approach 32767 a side.
 *
 * Ref: Rong & Tan (2006), "Jump Flooding in GPU with Applications
 *      to Voronoi Diagram and Distance Transform", I3D'06.
 */
typedef struct {
    int     w, h;                    /* grid size in cells                */
    int16_t nearest_x[CELLS_MAX];    /* cell -> nearest seed's x          */
    int16_t nearest_y[CELLS_MAX];    /* cell -> nearest seed's y          */
    int16_t scratch_x[CELLS_MAX];    /* spare buffer for the ping-pong    */
    int16_t scratch_y[CELLS_MAX];    /* spare buffer for the ping-pong    */
} JFAGrid;

/*
 * Seed — one of the points the whole picture is built around.
 *
 * Each seed has a fixed home spot and slowly circles around it. We keep
 * its position in two forms on purpose:
 *   - a smooth decimal position (fx, fy), so distances change gradually
 *     and the shading looks fluid;
 *   - a rounded whole-cell position (ix, iy), which the flooding uses to
 *     decide exactly which cell owns which seed.
 * That split is why the gradients glide while the cell borders still
 * snap cleanly to the grid.
 *
 * There are N_SEEDS of these on the Scene. seeds_scatter picks new homes
 * on reset; seeds_animate recomputes the positions every tick.
 *
 *   anchor_x, anchor_y : home spot (whole cells). The seed circles this.
 *   phase_x, phase_y   : where each seed starts in its circle. Giving x
 *                        and y different starting points turns the path
 *                        into a tilted oval, so the seeds don't all
 *                        drift in lockstep. In radians.
 *   fx, fy             : current smooth position. Used for distance reads.
 *   ix, iy             : current position rounded to a whole cell. Used by
 *                        the flooding to assign ownership.
 */
typedef struct {
    int16_t anchor_x, anchor_y;      /* home spot (cells)                  */
    float   phase_x,  phase_y;       /* starting point of the circle (rad) */
    float   fx, fy;                  /* smooth position                    */
    int16_t ix, iy;                  /* rounded-to-cell position           */
} Seed;

/* Turn a cell's (x, y) into its slot in the flat arrays. */
static inline int jfa_idx(const JFAGrid *g, int x, int y)
{
    return y * g->w + x;
}

/* Distance between two points, but squared (no square root). Comparing
 * squared distances gives the same "which is closer" answer, and skipping
 * the sqrt keeps the inner loop fast. */
static inline int dist_sq(int ax, int ay, int bx, int by)
{
    int dx = ax - bx;
    int dy = ay - by;
    return dx * dx + dy * dy;
}

/* ---------- §5a  ALGORITHM — the Jump Flooding itself ---------------- */

/* Starting point for the flooding: mark every cell "no seed yet", then
 * plant each seed into its own cell. From here the flooding spreads
 * that ownership outward. */
static void jfa_grid_init(JFAGrid *g, const Seed *seeds, int n_seeds)
{
    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        g->nearest_x[i] = JFA_SENTINEL;
        g->nearest_y[i] = JFA_SENTINEL;
    }
    for (int s = 0; s < n_seeds; s++) {
        int x = seeds[s].ix;
        int y = seeds[s].iy;
        if (x < 0 || x >= g->w || y < 0 || y >= g->h) continue;
        int idx = jfa_idx(g, x, y);
        g->nearest_x[idx] = (int16_t)x;
        g->nearest_y[idx] = (int16_t)y;
    }
}

/* Look at one neighbour during the flooding: if the seed that neighbour
 * knows about is closer to our cell than our current best, take it.
 * Neighbours off the grid, or ones that don't know any seed yet, are
 * simply skipped. */
static inline void jfa_consider_candidate_owner(
    const JFAGrid *g, int cx, int cy, int nx, int ny,
    const int16_t *src_x, const int16_t *src_y,
    int *best_d, int *best_sx, int *best_sy)
{
    if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) return;

    int neigh_idx = jfa_idx(g, nx, ny);
    int candidate_sx = src_x[neigh_idx];
    int candidate_sy = src_y[neigh_idx];
    if (candidate_sx == JFA_SENTINEL) return;        /* neighbour knows no seed yet */

    int candidate_d = dist_sq(cx, cy, candidate_sx, candidate_sy);
    if (candidate_d < *best_d) {                     /* closer than what we have? */
        *best_d  = candidate_d;
        *best_sx = candidate_sx;
        *best_sy = candidate_sy;
    }
}

/*
 * jfa_pass — one round of flooding using jump size k.
 *
 * Each cell looks at eight neighbours k steps away (and keeps itself in
 * the running too), and adopts whichever knows the closest seed. Reads
 * from src, writes the improved answers into dst — they must be
 * different buffers (see the ping-pong note on JFAGrid).
 */
static void jfa_pass(const JFAGrid *g, int k,
                     const int16_t *src_x, const int16_t *src_y,
                     int16_t *dst_x, int16_t *dst_y)
{
    for (int cy = 0; cy < g->h; cy++) {
        for (int cx = 0; cx < g->w; cx++) {
            int self_idx = jfa_idx(g, cx, cy);

            /* start with whatever seed this cell already knew */
            int best_sx = src_x[self_idx];
            int best_sy = src_y[self_idx];
            int best_d  = (best_sx == JFA_SENTINEL)
                        ? JFA_INFINITE_SQ_DISTANCE
                        : dist_sq(cx, cy, best_sx, best_sy);

            /* check the eight neighbours k steps away */
            for (int dy = -k; dy <= k; dy += k) {
                for (int dx = -k; dx <= k; dx += k) {
                    if (dx == 0 && dy == 0) continue;        /* skip self */
                    jfa_consider_candidate_owner(
                        g, cx, cy, cx + dx, cy + dy,
                        src_x, src_y,
                        &best_d, &best_sx, &best_sy);
                }
            }

            /* write the winner out */
            dst_x[self_idx] = (int16_t)best_sx;
            dst_y[self_idx] = (int16_t)best_sy;
        }
    }
}

/* The first (biggest) jump: half the grid's longer side, so even seeds
 * on opposite edges can reach each other in the very first pass. */
static inline int jfa_initial_jump_step(const JFAGrid *g)
{
    int n_max = (g->w > g->h) ? g->w : g->h;
    return n_max / 2;
}

/* Halve the jump for the next pass; the sequence runs down to 1. */
static inline int jfa_next_jump_step(int k) { return k >> 1; }

/* Swap the read and write buffers so the next pass reads the freshest
 * answers. Keeps each pass reading and writing separate memory. */
static inline void jfa_swap_ping_pong(int16_t **src_x, int16_t **src_y,
                                      int16_t **dst_x, int16_t **dst_y)
{
    int16_t *tx = *src_x; *src_x = *dst_x; *dst_x = tx;
    int16_t *ty = *src_y; *src_y = *dst_y; *dst_y = ty;
}

/* After all the swapping, the final answer might have landed in the
 * scratch buffer (odd number of passes). The rest of the program always
 * reads from nearest_*, so copy it back over if needed. */
static inline void jfa_finalise_into_canonical(JFAGrid *g,
                                               const int16_t *final_src_x,
                                               const int16_t *final_src_y)
{
    if (final_src_x == g->nearest_x) return;        /* already in the right place */

    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        g->nearest_x[i] = final_src_x[i];
        g->nearest_y[i] = final_src_y[i];
    }
}

/*
 * jfa_grid_run — build the whole distance map from scratch.
 *
 * Plant the seeds, then flood with ever-smaller jumps (half the grid,
 * then a quarter, ... down to 1) until every cell knows its nearest
 * seed. After this, sdf_read / sdf_seed_id work on any cell.
 */
static void jfa_grid_run(JFAGrid *g, const Seed *seeds, int n_seeds)
{
    jfa_grid_init(g, seeds, n_seeds);

    int16_t *src_x = g->nearest_x, *src_y = g->nearest_y;
    int16_t *dst_x = g->scratch_x, *dst_y = g->scratch_y;

    for (int k = jfa_initial_jump_step(g); k >= 1; k = jfa_next_jump_step(k)) {
        jfa_pass            (g, k, src_x, src_y, dst_x, dst_y);
        jfa_swap_ping_pong  (&src_x, &src_y, &dst_x, &dst_y);
    }

    jfa_finalise_into_canonical(g, src_x, src_y);
}

/* ---------- §5b  DATA — seed scatter --------------------------------- */

/* Drop the seeds at fresh random homes with random starting angles
 * (this is what 'r' does). It doesn't set the live positions — the
 * caller runs seeds_animate right after to fill those in. */
static void seeds_scatter(Seed *seeds, int n_seeds, int w, int h)
{
    for (int s = 0; s < n_seeds; s++) {
        seeds[s].anchor_x = (int16_t)(rand() % w);
        seeds[s].anchor_y = (int16_t)(rand() % h);
        seeds[s].phase_x  = (float)rand() / (float)RAND_MAX
                          * 2.0f * (float)M_PI;
        seeds[s].phase_y  = (float)rand() / (float)RAND_MAX
                          * 2.0f * (float)M_PI;
    }
}

/* ---------- §5c  ANIMATION — drifting the seeds around -------------- */

/* How far the seed is from its home right now. Using a sine for x and a
 * cosine for y, each with its own starting angle, traces a tilted oval
 * rather than a plain circle — that's what makes the drift look organic
 * instead of every seed sliding the same way. */
static inline void seed_compute_wobble_offset(const Seed *s, float t,
                                              float *wx, float *wy)
{
    *wx = WOBBLE_RADIUS_CELLS * sinf(t + s->phase_x);
    *wy = WOBBLE_RADIUS_CELLS * cosf(t + s->phase_y);
}

/* Keep a seed from drifting off the grid — one that did would vanish
 * from the picture. */
static inline void seed_clamp_to_grid(float *fx, float *fy, int w, int h)
{
    if (*fx < 0.0f)              *fx = 0.0f;
    if (*fx > (float)(w - 1))    *fx = (float)(w - 1);
    if (*fy < 0.0f)              *fy = 0.0f;
    if (*fy > (float)(h - 1))    *fy = (float)(h - 1);
}

/* Round a decimal position to the nearest whole cell. */
static inline int16_t seed_snap_to_integer_cell(float f)
{
    return (int16_t)(f + ROUND_TO_NEAREST_BIAS);
}

/* Save both forms of a seed's position at once — the smooth one for
 * shading and the rounded one for the flooding — so they never drift
 * out of agreement. */
static inline void seed_commit_position(Seed *s, float fx, float fy)
{
    s->fx = fx;
    s->fy = fy;
    s->ix = seed_snap_to_integer_cell(fx);
    s->iy = seed_snap_to_integer_cell(fy);
}

/* Move every seed to where it should be at time t: its home plus a
 * little orbit, kept on the grid. Runs once per tick. */
static void seeds_animate(Seed *seeds, int n_seeds, int w, int h, float t)
{
    for (int s = 0; s < n_seeds; s++) {
        float wx, wy;
        seed_compute_wobble_offset(&seeds[s], t, &wx, &wy);

        float fx = (float)seeds[s].anchor_x + wx;
        float fy = (float)seeds[s].anchor_y + wy;
        seed_clamp_to_grid(&fx, &fy, w, h);

        seed_commit_position(&seeds[s], fx, fy);
    }
}

/* ---------- §5d  READOUT — looking up a cell's distance to its seed -- */

/* Which seed owns this cell? Returns its index, or -1 if none. The grid
 * stores the owner's coordinates, so we just find the matching seed.
 * A plain scan is fine for only 20 seeds. */
static inline int sdf_seed_id(const JFAGrid *grid, const Seed *seeds,
                              int n_seeds, int x, int y)
{
    int idx = jfa_idx(grid, x, y);
    int sx  = grid->nearest_x[idx];
    int sy  = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) return -1;
    for (int s = 0; s < n_seeds; s++) {
        if (seeds[s].ix == sx && seeds[s].iy == sy) return s;
    }
    return -1;
}

/* How far cell (x, y) is from its nearest seed.
 *
 * It finds the owning seed through the grid (rounded positions), but
 * measures the distance against the seed's smooth position. So as a seed
 * drifts, the distance changes gradually even before its rounded cell
 * moves — that's what keeps the shaded gradients fluid instead of
 * looking like stair steps. */
static inline float sdf_read(const JFAGrid *grid, const Seed *seeds,
                             int n_seeds, int x, int y)
{
    int s = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (s < 0) return -1.0f;
    float dx = seeds[s].fx - (float)x;
    float dy = seeds[s].fy - (float)y;
    return sqrtf(dx * dx + dy * dy);
}

/* ===================================================================== */
/* §6  patterns — 30 visualisations of the same JFA output + dispatch     */
/* ===================================================================== */

/*
 * Every pattern below has the same shape: given a cell, it fills in a
 * brightness (out_glow, 0 to 1) and a color tier (out_band, 0 to 3).
 * The arguments are the finished distance grid, the seeds, the cell's
 * (x, y), and t (the drift clock, which the Tier-6 animated patterns
 * lean on). Patterns reach into the grid only through sdf_read,
 * sdf_seed_id, and the raw owner coords.
 */

/* Distances get divided by this to land in the 0..1 range for shading.
 * With 20 seeds the farthest a cell usually gets is around 20, so 25
 * keeps most of the range usable. */
#define SDF_TYPICAL_MAX     25.0f

/* How far apart the repeating rings are (Tier 2), in distance units. */
#define BAND_SPACING        4.0f

/* How quickly the glow patterns (HALO / AURA) fade with distance. */
#define HALO_SIGMA          5.0f

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Turn a 0..1 brightness into one of the 4 color tiers. */
static inline int band_from_glow(float g)
{
    return (int)(g * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK;
}

/* Scramble a number into a repeatable "random" value between 0 and 1.
 * Patterns use this to give each cell its own flavor (a color, a blink
 * rate). Same input always gives the same output. */
static inline float jfa_hash_to_unit(uint32_t v)
{
    v = (v ^ (v >> 16)) * 0x7feb352du;
    v = (v ^ (v >> 15)) * 0x846ca68bu;
    v =  v ^ (v >> 16);
    return (float)(v & 0xFFFFu) * (1.0f / 65535.0f);
}

/* A single ID for a cell's owning seed, built from its coordinates.
 * Every cell that belongs to the same seed gets the same ID, so we can
 * give a whole region one consistent random flavor. */
static inline uint32_t jfa_cell_key(int sx, int sy)
{
    return ((uint32_t)(sx & 0xFFFF) << 16) | (uint32_t)(sy & 0xFFFF);
}

/* ---------- helpers for the cell-skeleton patterns (Tier 5) ----------- *
 *
 * These peek at a cell's four neighbours to find the structure of the
 * cell layout: the borders between cells, the corners where three cells
 * meet, and the ridge lines running down the middle of each cell.
 */

/* Is this cell on a border between two cells? True when any of its four
 * neighbours belongs to a different seed. */
static bool is_voronoi_edge(const JFAGrid *g, int x, int y)
{
    int idx_me = jfa_idx(g, x, y);
    int sx = g->nearest_x[idx_me];
    int sy = g->nearest_y[idx_me];
    if (sx == JFA_SENTINEL) return false;
    static const int dx4[4] = { -1, 1,  0, 0 };
    static const int dy4[4] = {  0, 0, -1, 1 };
    for (int k = 0; k < 4; k++) {
        int nx = x + dx4[k];
        int ny = y + dy4[k];
        if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;
        int idx_n = jfa_idx(g, nx, ny);
        if (g->nearest_x[idx_n] != sx || g->nearest_y[idx_n] != sy) return true;
    }
    return false;
}

/* Have we already counted this seed in the running list? */
static inline bool owner_already_listed(const int *sx_seen, const int *sy_seen,
                                        int n, int sx, int sy)
{
    for (int i = 0; i < n; i++)
        if (sx_seen[i] == sx && sy_seen[i] == sy) return true;
    return false;
}

/* How many different seeds own this cell and its four neighbours?
 * Result is 1 deep inside a cell, 2 along a border, and 3+ right at a
 * corner where several cells touch (used to spot those corners). */
static int count_distinct_neighbours(const JFAGrid *g, int x, int y)
{
    /* the cell itself plus its four neighbours */
    static const int neighbour_dx5[5] = { 0, -1, 1,  0, 0 };
    static const int neighbour_dy5[5] = { 0,  0, 0, -1, 1 };

    int sx_seen[5];
    int sy_seen[5];
    int unique_count = 0;

    for (int k = 0; k < 5; k++) {
        int nx = x + neighbour_dx5[k];
        int ny = y + neighbour_dy5[k];
        if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;

        int idx = jfa_idx(g, nx, ny);
        int neighbour_sx = g->nearest_x[idx];
        int neighbour_sy = g->nearest_y[idx];
        if (neighbour_sx == JFA_SENTINEL) continue;        /* no seed here */

        if (owner_already_listed(sx_seen, sy_seen, unique_count,
                                 neighbour_sx, neighbour_sy)) continue;

        sx_seen[unique_count] = neighbour_sx;
        sy_seen[unique_count] = neighbour_sy;
        unique_count++;
    }
    return unique_count;
}

/* Is this cell at least as far from its seed as all four neighbours are
 * from theirs? Those high points trace the ridge running between cells —
 * roughly the skeleton of the layout. It comes out a little speckly on a
 * grid this coarse. */
static bool is_local_max(const JFAGrid *g, const Seed *seeds, int n_seeds,
                         int x, int y)
{
    float d_me = sdf_read(g, seeds, n_seeds, x, y);
    if (d_me < 0.0f) return false;
    static const int dx4[4] = { -1, 1,  0, 0 };
    static const int dy4[4] = {  0, 0, -1, 1 };
    for (int k = 0; k < 4; k++) {
        int nx = x + dx4[k];
        int ny = y + dy4[k];
        if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;
        float d_n = sdf_read(g, seeds, n_seeds, nx, ny);
        if (d_n > d_me + MEDIAL_AXIS_TOLERANCE) return false;
    }
    return true;
}

/* ---------- Tier 1 — plain distance, shaded different ways ------------ */

/* SDF — straight distance: the farther from a seed, the brighter. */
static void pattern_sdf(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                        float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* INVERSE — the flip of SDF: bright at the seeds, dark out at the edges. */
static void pattern_inverse(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = 1.0f - clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* SQRT — same as SDF but brightens the mid-range, for softer contrast. */
static void pattern_sqrt(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = sqrtf(clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f));
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* SQUARED — the opposite of SQRT: only the far cells really light up,
 * so the contrast is sharp. */
static void pattern_squared(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float lin = clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = lin * lin;
    *out_band = band_from_glow(*out_glow);
}

/* CAPPED — like SDF but everything past the halfway distance maxes out
 * flat, giving solid-looking cell interiors instead of a fade. */
static void pattern_capped(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                           float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = clampf(d / (SDF_TYPICAL_MAX * 0.5f), 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 2 — repeating rings carved out of the distance ------- */

/* RINGS — evenly spaced rings around each seed, fading in and out. */
static void pattern_rings(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float u   = fmodf(d, BAND_SPACING) / BAND_SPACING;       /* position in ring */
    float tri = (u < 0.5f) ? (u * 2.0f) : (2.0f - u * 2.0f); /* up then down    */
    *out_glow = tri;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* SAW — rings that brighten gradually then drop off sharply, so they
 * look like they're flowing outward from each seed. */
static void pattern_saw(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                        float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = fmodf(d, BAND_SPACING) / BAND_SPACING;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* ZEBRA — hard on/off rings, no shading in between. */
static void pattern_zebra(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    int ring = (int)(d / BAND_SPACING);
    *out_glow = (ring & 1) ? 0.85f : 0.15f;
    *out_band = ring & PALETTE_BAND_MASK;
}

/* WAVE — smooth rings like ripples on a pond. */
static void pattern_wave(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = sinf(d * (2.0f * (float)M_PI / BAND_SPACING)) * 0.5f + 0.5f;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* SHELLS — nested layers, each bright at its inner edge and fading out,
 * stacked from the seed outward. */
static void pattern_shells(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                           float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float u = fmodf(d, BAND_SPACING);
    *out_glow = expf(-u * 1.5f);
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* ---------- Tier 3 — color each cell by which seed owns it ------------ */

/* CELLS — each region gets its own color, a touch brighter near its
 * seed so each cell has a center. */
static void pattern_cells(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    int sid = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (sid < 0) { *out_glow = 0.0f; *out_band = 0; return; }
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = 1.0f - clampf(d * 0.06f, 0.0f, 0.4f);
    *out_band = sid & PALETTE_BAND_MASK;
}

/* CRACKLE — colored cells that darken toward their edges, like dried
 * cracked mud. */
static void pattern_crackle(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    int sid = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (sid < 0) { *out_glow = 0.0f; *out_band = 0; return; }
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = clampf(1.0f - d * 0.10f, 0.0f, 1.0f);
    *out_band = sid & PALETTE_BAND_MASK;
}

/* SPECKLE — every cell a flat random brightness of its own, no fade. */
static void pattern_speckle(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)seeds; (void)n_seeds; (void)t;
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    float v = jfa_hash_to_unit(jfa_cell_key(sx, sy));
    *out_glow = 0.3f + v * 0.7f;
    *out_band = (int)(v * 3.999f) & PALETTE_BAND_MASK;
}

/* RAINBOW — like SPECKLE, but the color also varies per cell so
 * neighbours stand apart even at the same brightness. */
static void pattern_rainbow(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    uint32_t k = jfa_cell_key(sx, sy);
    float v = jfa_hash_to_unit(k);
    /* mix in a little fade so cells aren't perfectly flat */
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = clampf((1.0f - d * 0.04f) * (0.3f + v * 0.7f), 0.0f, 1.0f);
    *out_band = (int)(jfa_hash_to_unit(k ^ 0xdeadbeefu) * 3.999f) & PALETTE_BAND_MASK;
}

/* TWINKLE — each cell slowly blinks, every one at its own pace, so they
 * never blink in sync. */
static void pattern_twinkle(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    uint32_t k    = jfa_cell_key(sx, sy);
    float    rate = 0.5f + jfa_hash_to_unit(k) * 2.5f;
    float    pulse = 0.5f + 0.5f * sinf(t * rate);
    float    d    = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = clampf((1.0f - d * 0.06f) * pulse, 0.0f, 1.0f);
    *out_band = (int)k & PALETTE_BAND_MASK;
}

/* ---------- Tier 4 — rings and glows at chosen distances -------------- */

/* OUTLINE — one soft glowing ring at a fixed distance from each seed. */
static void pattern_outline(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float diff = fabsf(d - OUTLINE_DISTANCE);
    if (diff > OUTLINE_THICKNESS) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = 1.0f - diff / OUTLINE_THICKNESS;
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* DUAL — two rings around each seed, an inner and an outer, in
 * different colors. */
static void pattern_dual(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float d_inner = 4.0f;        /* inner ring distance  */
    float d_outer = 10.0f;       /* outer ring distance  */
    float diff_i = fabsf(d - d_inner);
    float diff_o = fabsf(d - d_outer);
    if (diff_i < OUTLINE_THICKNESS) {
        *out_glow = 1.0f - diff_i / OUTLINE_THICKNESS;
        *out_band = 1;
    } else if (diff_o < OUTLINE_THICKNESS) {
        *out_glow = 1.0f - diff_o / OUTLINE_THICKNESS;
        *out_band = 3;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* SHELL — a thick filled band around each seed, like a fat OUTLINE. */
static void pattern_shell(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float d_lo = 2.0f, d_hi = 7.0f;        /* the band runs from d_lo to d_hi */
    if (d < d_lo || d > d_hi) { *out_glow = 0.0f; *out_band = 0; return; }
    float u = (d - d_lo) / (d_hi - d_lo);
    /* fade the band's two edges so it doesn't end abruptly */
    *out_glow = u * u * (3.0f - 2.0f * u) * (1.0f - u) * 4.0f;
    if (*out_glow > 1.0f) *out_glow = 1.0f;
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* HALO — a simple soft glow around each seed, bright at the center and
 * fading smoothly outward. */
static void pattern_halo(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = expf(-(d * d) / (HALO_SIGMA * HALO_SIGMA));
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* AURA — three glows of different sizes stacked, giving each seed a
 * bright core, a halo, and a faint outer haze. */
static void pattern_aura(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float s1 = HALO_SIGMA * 0.5f;        /* tight core glow  */
    float s2 = HALO_SIGMA;               /* mid halo         */
    float s3 = HALO_SIGMA * 2.0f;        /* wide outer haze  */
    float g = expf(-(d * d) / (s1 * s1)) * 0.6f
            + expf(-(d * d) / (s2 * s2)) * 0.3f
            + expf(-(d * d) / (s3 * s3)) * 0.15f;
    *out_glow = clampf(g, 0.0f, 1.0f);
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* ---------- Tier 5 — the cell skeleton: edges, corners, ridges ------- */

/* EDGES — light up only the borders between cells. */
static void pattern_edges(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    if (is_voronoi_edge(grid, x, y)) {
        *out_glow = 1.0f;
        *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* VERTICES — light up just the corners where three or more cells meet.
 * Only a few scattered bright dots. */
static void pattern_vertices(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band)
{
    (void)seeds; (void)n_seeds; (void)t;
    int n = count_distinct_neighbours(grid, x, y);
    if (n >= 3) {
        *out_glow = 1.0f;
        *out_band = 3;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* INTERIOR — the opposite of EDGES: fill the cells, leave thin dark
 * cracks along the borders. */
static void pattern_interior(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band)
{
    (void)t;
    int sid = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (sid < 0) { *out_glow = 0.0f; *out_band = 0; return; }
    if (is_voronoi_edge(grid, x, y)) {
        *out_glow = 0.0f;
        *out_band = 0;
    } else {
        float d = sdf_read(grid, seeds, n_seeds, x, y);
        *out_glow = clampf(1.0f - d * 0.05f, 0.0f, 1.0f);
        *out_band = sid & PALETTE_BAND_MASK;
    }
}

/* WEAVE — RINGS, but clipped to each cell so the borders stay dark.
 * Looks like a tiled patchwork of little ripple patches. */
static void pattern_weave(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    if (is_voronoi_edge(grid, x, y)) {
        *out_glow = 0.0f;
        *out_band = 0;
        return;
    }
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float u   = fmodf(d, BAND_SPACING) / BAND_SPACING;
    float tri = (u < 0.5f) ? (u * 2.0f) : (2.0f - u * 2.0f);
    *out_glow = tri;
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* SKELETON — light up the ridge lines running down the middle between
 * cells. Comes out a bit patchy on a coarse grid. */
static void pattern_skeleton(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band)
{
    (void)t;
    if (is_local_max(grid, seeds, n_seeds, x, y)) {
        *out_glow = 1.0f;
        *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* ---------- Tier 6 — effects that move over time --------------------- */

/* PULSE — the plain SDF, but the whole thing breathes brighter and
 * dimmer over time. */
static void pattern_pulse(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float pulse = 0.5f + 0.5f * sinf(t * 1.5f);
    float g     = clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = g * pulse;
    *out_band = band_from_glow(g);
}

/* SCAN — one ring that grows outward over time, like a radar sweep
 * expanding from every seed at once. */
static void pattern_scan(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    /* the ring's distance creeps outward, then jumps back to the start */
    float threshold = fmodf(t * 4.0f, SDF_TYPICAL_MAX);
    float diff = fabsf(d - threshold);
    if (diff > OUTLINE_THICKNESS * 1.5f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = 1.0f - diff / (OUTLINE_THICKNESS * 1.5f);
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* ECHO — like SCAN but with many rings at once, a steady train of
 * pulses spreading from each seed. */
static void pattern_echo(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float phase = d - t * 3.0f;
    float u   = fmodf(phase, BAND_SPACING);
    if (u < 0.0f) u += BAND_SPACING;
    u /= BAND_SPACING;
    *out_glow = sinf(u * 2.0f * (float)M_PI) * 0.5f + 0.5f;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* PLASMA — the classic shifting-blob plasma effect, blended from the
 * distance plus the cell's x and y, all stirred by time. */
static void pattern_plasma(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                           float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float v = sinf(d * 0.4f + t)
            + cosf((float)x * 0.15f + t * 0.7f)
            + sinf((float)y * 0.12f + t * 0.5f);
    float g = (v + 3.0f) / 6.0f;
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* CHAOS — a field of colored cells, each one breathing on its own clock. */
static void pattern_chaos(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    uint32_t k     = jfa_cell_key(sx, sy);
    float    phase = jfa_hash_to_unit(k) * 2.0f * (float)M_PI;
    float    pulse = 0.5f + 0.5f * sinf(t * 2.0f + phase);
    float    d     = sdf_read(grid, seeds, n_seeds, x, y);
    float    fade  = clampf(1.0f - d * 0.05f, 0.0f, 1.0f);
    *out_glow = pulse * fade;
    *out_band = (int)k & PALETTE_BAND_MASK;
}

/* ---------- the pattern table ---------------------------------------- */

typedef void (*JFAPatternFn)(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band);

/*
 * NoisePattern — one row in the table of all 30 patterns.
 *
 * Instead of a huge switch, each pattern is a row holding its label and
 * a pointer to its function, looked up by the Pattern enum. To add a
 * pattern you write the function, add its enum, and add a row here —
 * nothing else changes.
 *
 *   name   : label shown in the HUD, padded so the column lines up.
 *   tier   : its tier tag, like "1-DIST" or "5-STRC", also for the HUD.
 *   sample : the pattern function to call for each cell.
 */
typedef struct {
    const char   *name;         /* HUD label, padded to line up      */
    const char   *tier;         /* tier tag, padded to line up       */
    JFAPatternFn  sample;       /* the pattern function              */
} NoisePattern;

static const NoisePattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — DISTANCE */
    [PATTERN_SDF]       = { "SDF       ", "1-DIST ", pattern_sdf       },
    [PATTERN_INVERSE]   = { "INVERSE   ", "1-DIST ", pattern_inverse   },
    [PATTERN_SQRT]      = { "SQRT      ", "1-DIST ", pattern_sqrt      },
    [PATTERN_SQUARED]   = { "SQUARED   ", "1-DIST ", pattern_squared   },
    [PATTERN_CAPPED]    = { "CAPPED    ", "1-DIST ", pattern_capped    },
    /* Tier 2 — BANDS */
    [PATTERN_RINGS]     = { "RINGS     ", "2-BAND ", pattern_rings     },
    [PATTERN_SAW]       = { "SAW       ", "2-BAND ", pattern_saw       },
    [PATTERN_ZEBRA]     = { "ZEBRA     ", "2-BAND ", pattern_zebra     },
    [PATTERN_WAVE]      = { "WAVE      ", "2-BAND ", pattern_wave      },
    [PATTERN_SHELLS]    = { "SHELLS    ", "2-BAND ", pattern_shells    },
    /* Tier 3 — VORONOI */
    [PATTERN_CELLS]     = { "CELLS     ", "3-VOR  ", pattern_cells     },
    [PATTERN_CRACKLE]   = { "CRACKLE   ", "3-VOR  ", pattern_crackle   },
    [PATTERN_SPECKLE]   = { "SPECKLE   ", "3-VOR  ", pattern_speckle   },
    [PATTERN_RAINBOW]   = { "RAINBOW   ", "3-VOR  ", pattern_rainbow   },
    [PATTERN_TWINKLE]   = { "TWINKLE   ", "3-VOR  ", pattern_twinkle   },
    /* Tier 4 — CONTOURS */
    [PATTERN_OUTLINE]   = { "OUTLINE   ", "4-CONT ", pattern_outline   },
    [PATTERN_DUAL]      = { "DUAL      ", "4-CONT ", pattern_dual      },
    [PATTERN_SHELL]     = { "SHELL     ", "4-CONT ", pattern_shell     },
    [PATTERN_HALO]      = { "HALO      ", "4-CONT ", pattern_halo      },
    [PATTERN_AURA]      = { "AURA      ", "4-CONT ", pattern_aura      },
    /* Tier 5 — STRUCTURE */
    [PATTERN_EDGES]     = { "EDGES     ", "5-STRC ", pattern_edges     },
    [PATTERN_VERTICES]  = { "VERTICES  ", "5-STRC ", pattern_vertices  },
    [PATTERN_INTERIOR]  = { "INTERIOR  ", "5-STRC ", pattern_interior  },
    [PATTERN_WEAVE]     = { "WEAVE     ", "5-STRC ", pattern_weave     },
    [PATTERN_SKELETON]  = { "SKELETON  ", "5-STRC ", pattern_skeleton  },
    /* Tier 6 — ANIMATED */
    [PATTERN_PULSE]     = { "PULSE     ", "6-ANIM ", pattern_pulse     },
    [PATTERN_SCAN]      = { "SCAN      ", "6-ANIM ", pattern_scan      },
    [PATTERN_ECHO]      = { "ECHO      ", "6-ANIM ", pattern_echo      },
    [PATTERN_PLASMA]    = { "PLASMA    ", "6-ANIM ", pattern_plasma    },
    [PATTERN_CHAOS]     = { "CHAOS     ", "6-ANIM ", pattern_chaos     },
};

static const char *pattern_name(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?         ";
    return noise_patterns[p].name;
}

static const char *pattern_tier(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?      ";
    return noise_patterns[p].tier;
}

/* ===================================================================== */
/* §7  scene — JFAGrid + Seeds + GlowField + GlyphRamp                    */
/*             + PatternState + PaletteState + Scene                       */
/* ===================================================================== */

/*
 * GlowField — the finished picture, one cell at a time, ready to draw.
 *
 * It sits between two jobs: the pattern code fills it in (a brightness
 * and a color tier per cell) once per sim step, and the drawing code
 * reads it back to put characters on screen. Keeping it as a snapshot
 * means drawing and computing don't have to happen at the same moment.
 *
 *   w, h    : grid size in cells (matches the JFAGrid).
 *   count   : w * h, kept handy so the loops don't recompute it.
 *   glow[]  : each cell's brightness, 0 to 1.
 *   band[]  : each cell's color tier, 0 to 3.
 */
typedef struct {
    int      w, h;                   /* grid size in cells              */
    int      count;                  /* w * h                           */
    float    glow[CELLS_MAX];        /* brightness per cell, 0..1       */
    uint8_t  band[CELLS_MAX];        /* color tier per cell, 0..3       */
} GlowField;

static inline int glow_field_idx(const GlowField *gf, int x, int y)
{
    return y * gf->w + x;
}

static void glow_field_reset(GlowField *gf, int w, int h)
{
    gf->w     = w;
    gf->h     = h;
    gf->count = w * h;
    for (int i = 0; i < gf->count; i++) {
        gf->glow[i] = 0.0f;
        gf->band[i] = 0;
    }
}

/*
 * GlyphRamp — the rule for turning a brightness into an ASCII character.
 *
 * Brighter cells get denser characters:
 *   above thresh_high  ->  '#'  bold
 *   above thresh_mid   ->  '*'  bold
 *   above thresh_low   ->  '.'  normal
 *   below that         ->  drawn as blank
 *
 * The three cutoffs must stay in order (high > mid > low > 0). The
 * '#', '*', '.' set is a coarse slice of Paul Bourke's classic
 * dark-to-light character ramp.
 */
typedef struct {
    float thresh_high;    /* above this -> glyph_high             */
    float thresh_mid;     /* above this -> glyph_mid              */
    float thresh_low;     /* above this -> glyph_low             */
    char  glyph_high;     /* densest character                   */
    char  glyph_mid;      /* mid character                       */
    char  glyph_low;      /* faintest character                  */
} GlyphRamp;

/*
 * GlyphChoice — what glyph_ramp_pick decided for one cell. The color is
 * handled separately, so this only covers the character itself.
 *
 *   glyph   : the character to draw (only meaningful if visible).
 *   attr    : bold or normal.
 *   visible : false means leave the cell blank.
 */
typedef struct {
    char glyph;
    int  attr;
    bool visible;
} GlyphChoice;

static void glyph_ramp_init(GlyphRamp *gr)
{
    gr->thresh_high = GLYPH_HIGH_THRESH;
    gr->thresh_mid  = GLYPH_MID_THRESH;
    gr->thresh_low  = GLOW_THRESHOLD;
    gr->glyph_high  = '#';
    gr->glyph_mid   = '*';
    gr->glyph_low   = '.';
}

static GlyphChoice glyph_ramp_pick(const GlyphRamp *gr, float glow)
{
    GlyphChoice c = { .glyph = ' ', .attr = A_NORMAL, .visible = false };
    if      (glow > gr->thresh_high) { c.glyph = gr->glyph_high; c.attr = A_BOLD;   c.visible = true; }
    else if (glow > gr->thresh_mid)  { c.glyph = gr->glyph_mid;  c.attr = A_BOLD;   c.visible = true; }
    else if (glow > gr->thresh_low)  { c.glyph = gr->glyph_low;  c.attr = A_NORMAL; c.visible = true; }
    return c;
}

/*
 * PatternState — which pattern is showing and how fast the seeds drift.
 *
 *   current    : the pattern on screen now; n/p flip through them.
 *   wobble_time: the running clock that drives the drift. Ticks up a
 *                little each step; reset to zero only on 'r'.
 *   drift_mult : drift-speed dial, doubled/halved by +/- so speed
 *                changes feel like steps rather than a smooth slider.
 */
typedef struct {
    Pattern current;       /* the pattern on screen                         */
    float   wobble_time;   /* the drift clock                               */
    int     drift_mult;    /* drift-speed dial (powers of 2)                */
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current     = PATTERN_SDF;
    ps->wobble_time = 0.0f;
    ps->drift_mult  = DRIFT_MULT_DEF;
}

/*
 * PaletteState — which color theme is showing.
 *
 * Just an index into themes[], cycled by t/T. (The actual colors live in
 * ncurses; theme_apply pushes them there whenever this changes.)
 */
typedef struct {
    int current;          /* index into themes[]                 */
} PaletteState;

static void palette_state_init(PaletteState *p)
{
    p->current = 0;
}

/*
 * Scene — holds everything that changes while the program runs.
 *
 * The pieces line up in the order the data flows each step:
 *   seeds   -> drift to new spots
 *   grid    -> rebuilt from the seeds (the distance map)
 *   pattern -> turns the grid into the field
 *   field   -> brightness + color per cell
 *   ramp    -> turns brightness into characters (color comes from palette)
 *   palette -> which theme; plus the paused flag
 *
 * Three calls move it through its life:
 *   scene_init  — set up and build the first frame.
 *   scene_reset — start over with new seeds ('r' and on resize).
 *   scene_tick  — advance one step: drift, rebuild, re-evaluate.
 * No cleanup needed; it lives for the whole program.
 */
typedef struct {
    JFAGrid       grid;            /* the distance map (~90 KB)              */
    Seed          seeds[N_SEEDS];  /* the seed points                        */
    GlowField     field;           /* the finished picture (~88 KB)          */
    GlyphRamp     ramp;            /* brightness -> character rule           */
    PatternState  pattern;         /* which pattern + drift speed            */
    PaletteState  palette;         /* which theme                            */
    bool          paused;          /* true freezes the drift                 */
} Scene;

/* Sort a 0..1 brightness into one of the 4 color tiers. The 3.999 gain
 * keeps a full 1.0 from spilling past tier 3. */
static inline uint8_t glow_to_palette_band(float glow)
{
    return (uint8_t)((int)(glow * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK);
}

/* Move the seeds to their current drift positions, then rebuild the
 * whole distance map from them. Cheap enough to do every frame. */
static void scene_recompute_jfa(Scene *s)
{
    seeds_animate(s->seeds, N_SEEDS, s->grid.w, s->grid.h,
                  s->pattern.wobble_time);
    jfa_grid_run(&s->grid, s->seeds, N_SEEDS);
}

/* Fill in one cell of the field: run the active pattern for it and store
 * the brightness and color (kept in their valid ranges). */
static inline void glow_field_sample_cell(
    GlowField *field, int x, int y,
    JFAPatternFn sample_pattern,
    const JFAGrid *grid, const Seed *seeds, float t)
{
    float raw_glow = 0.0f;
    int   raw_band = 0;
    sample_pattern(grid, seeds, N_SEEDS, x, y, t, &raw_glow, &raw_band);

    int idx          = glow_field_idx(field, x, y);
    field->glow[idx] = clampf(raw_glow, 0.0f, 1.0f);
    field->band[idx] = (uint8_t)(raw_band & PALETTE_BAND_MASK);
}

/* Run the active pattern over every cell to fill the field. Expects the
 * distance map to be current already (scene_recompute_jfa did that). */
static void scene_evaluate_field(Scene *s)
{
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    JFAPatternFn   sample_pattern = noise_patterns[active].sample;
    const JFAGrid *grid           = &s->grid;
    const Seed    *seeds          = s->seeds;
    GlowField     *field          = &s->field;
    float          t              = s->pattern.wobble_time;

    for (int y = 0; y < field->h; y++) {
        for (int x = 0; x < field->w; x++) {
            glow_field_sample_cell(field, x, y,
                                   sample_pattern, grid, seeds, t);
        }
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    glow_field_reset(&s->field, mw, mh);
    s->grid.w = mw;
    s->grid.h = mh;
    s->pattern.wobble_time = 0.0f;
    seeds_scatter(s->seeds, N_SEEDS, mw, mh);
    scene_recompute_jfa  (s);
    scene_evaluate_field (s);     /* build one frame now, so a paused start still shows something */
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    glyph_ramp_init   (&s->ramp);
    pattern_state_init(&s->pattern);
    palette_state_init(&s->palette);
    s->paused = false;
    scene_reset(s, mw, mh);
}

/* One animation step: nudge the drift clock forward, then rebuild the
 * distance map and the picture. Does nothing while paused. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->pattern.wobble_time += WOBBLE_RATE * (float)s->pattern.drift_mult * dt;
    scene_recompute_jfa (s);
    scene_evaluate_field(s);
}

/* ===================================================================== */
/* §8  screen — terminal viewport + scene_draw + HUD                       */
/* ===================================================================== */

/*
 * Screen — the terminal's current size, remembered so we don't have to
 * ask ncurses for it on every cell. Refreshed whenever the window is
 * resized.
 *
 *   cols : width in cells.
 *   rows : height in cells.
 * (rows-then-cols matches the order ncurses' getmaxyx hands them back.)
 */
typedef struct {
    int cols;     /* width  in cells  */
    int rows;     /* height in cells  */
} Screen;

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

/* ---------- scene_draw: turn the picture into characters ------------- */

/*
 * GridPlacement — where on screen the picture's top-left corner goes,
 * once it's centered in the window with the HUD rows left clear.
 * Worked out once per frame so the drawing loop doesn't redo the
 * centering math for every cell.
 *
 *   origin_x : screen column of the picture's left edge (never negative).
 *   origin_y : screen row of the top edge (never above the title bar).
 * If the picture is bigger than the window it's just cropped at the edge.
 */
typedef struct {
    int origin_x;   /* screen column of the left edge  */
    int origin_y;   /* screen row of the top edge      */
} GridPlacement;

static GridPlacement compute_grid_placement(int field_w, int field_h,
                                            int cols, int rows)
{
    GridPlacement p;
    p.origin_x = (cols - field_w) / 2;
    p.origin_y = ((rows - HUD_BAND_RESERVED_ROWS) - field_h) / 2
                 + HUD_TOP_ROWS;
    if (p.origin_x < 0)            p.origin_x = 0;
    if (p.origin_y < HUD_TOP_ROWS) p.origin_y = HUD_TOP_ROWS;
    return p;
}

/* Draw one character on screen in the given color. (The cast keeps
 * ncurses from mangling characters above 127.) */
static inline void draw_glyph_at(int screen_y, int screen_x,
                                 char glyph, int color_pair, int attr)
{
    attron(COLOR_PAIR(color_pair) | attr);
    mvaddch(screen_y, screen_x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | attr);
}

/* Which color slot a brightness tier uses. */
static inline int band_to_color_pair(uint8_t band)
{
    return PAIR_BAND_BASE + (band & PALETTE_BAND_MASK);
}

/* Draw one cell: read its brightness, pick a character, and if it's not
 * blank, put it on screen in the right color. */
static inline void glow_field_paint_cell(
    const GlowField *field, const GlyphRamp *ramp,
    int x, int y, int screen_x, int screen_y)
{
    int   cell_idx = glow_field_idx(field, x, y);
    float glow     = field->glow[cell_idx];

    GlyphChoice pick = glyph_ramp_pick(ramp, glow);
    if (!pick.visible) return;

    int color_pair = band_to_color_pair(field->band[cell_idx]);
    draw_glyph_at(screen_y, screen_x, pick.glyph, color_pair, pick.attr);
}

/* Draw the whole picture: center it, then paint every cell that lands
 * on screen. Just reads the scene; doesn't change anything. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const GlowField *field = &s->field;
    const GlyphRamp *ramp  = &s->ramp;

    GridPlacement place = compute_grid_placement(field->w, field->h,
                                                 cols, rows);

    for (int y = 0; y < field->h; y++) {
        int screen_y = place.origin_y + y;
        if (screen_y < 0 || screen_y >= rows) continue;
        for (int x = 0; x < field->w; x++) {
            int screen_x = place.origin_x + x;
            if (screen_x < 0 || screen_x >= cols) continue;

            glow_field_paint_cell(field, ramp, x, y, screen_x, screen_y);
        }
    }
}

/* ---------- HUD layout widths ----------------------------------------- */

#define HUD_W_PATTERN_FIELD   21   /* " pattern:%-10s "          */
#define HUD_W_TIER_FIELD      15   /* " tier:%-7s "              */
#define HUD_W_THEME_FIELD     17   /* " theme:%-8s "             */
#define HUD_W_PALETTE_LABEL    9   /* " palette:"                */

#define HUD_TITLE_ROW          0
#define HUD_STATUS_ROW         1
#define HUD_TITLE_TEXT         " SDF / JUMP FLOOD "
#define HUD_BOTTOM_HINT_TEXT \
    " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit "

/* ---------- HUD: one drawer per chunk of the status line -------------- *
 *
 * Each one draws its piece and returns where the next piece should
 * start, so the caller can lay them out left to right.
 */

/* The program title in the top-left. */
static int hud_draw_title_chip(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, "%s", HUD_TITLE_TEXT);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + (int)strlen(HUD_TITLE_TEXT);
}

/* Top-right status: fps, tick rate, pattern, and drift speed. */
static void hud_draw_state_bar(int row, int cols,
                               double fps, int sim_fps,
                               const PatternState *ps, bool paused)
{
    const char *state_text = paused ? "PAUSED    " : pattern_name(ps->current);
    char        buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  drift:x%-2d ",
             fps, sim_fps, state_text,
             (int)ps->current + 1, N_PATTERNS,
             ps->drift_mult);
    int right_aligned_x = cols - (int)strlen(buf);
    if (right_aligned_x < 0) right_aligned_x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, right_aligned_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* "pattern:<NAME>" */
static int hud_draw_pattern_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-10s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_PATTERN_FIELD;
}

/* "tier:<N-LABEL>" */
static int hud_draw_tier_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " tier:%-7s ", pattern_tier(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_TIER_FIELD;
}

/* "theme:<NAME>" */
static int hud_draw_theme_field(int row, int x, int theme_idx)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_THEME_FIELD;
}

/* "palette:" followed by a small '#' in each of the theme's four colors,
 * so you can see at a glance what the current theme looks like. */
static int hud_draw_palette_swatches(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_W_PALETTE_LABEL;
    for (int band = 0; band < N_PALETTE_BANDS; band++) {
        int color_pair = PAIR_BAND_BASE + band;
        attron(COLOR_PAIR(color_pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(color_pair) | A_BOLD);
        x += 1;
    }
    return x;
}

/* Tail of the status line: seed count and grid size. */
static void hud_draw_stats_field(int row, int x, const GlowField *gf)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  seeds:%d  map:%dx%d ", N_SEEDS, gf->w, gf->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the list of keys you can press. */
static void hud_draw_action_hint(int row)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(row, 0, "%s", HUD_BOTTOM_HINT_TEXT);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ---------- screen_draw: scene + full HUD ----------------------------- */

/* Draw one full frame: clear, paint the picture, then lay out the HUD. */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    /* top row: title on the left, status on the right */
    hud_draw_title_chip(HUD_TITLE_ROW, HUD_LEFT_MARGIN);
    hud_draw_state_bar (HUD_TITLE_ROW, sc->cols, fps, sim_fps,
                        &s->pattern, s->paused);

    /* second row: the info chunks laid out left to right */
    int x = HUD_LEFT_MARGIN;
    x = hud_draw_pattern_field   (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_tier_field      (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_theme_field     (HUD_STATUS_ROW, x, s->palette.current);
    x = hud_draw_palette_swatches(HUD_STATUS_ROW, x);
    hud_draw_stats_field         (HUD_STATUS_ROW, x, &s->field);

    hud_draw_action_hint(sc->rows - 1);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app — App harness + FrameClock + main loop                          */
/* ===================================================================== */

/*
 * App — the top-level box that holds everything else.
 *
 * There's one global so the signal handlers can flip its flags. The
 * handlers touch only those two flags; the main loop owns the rest.
 *
 *   scene       : the simulation (see Scene).
 *   screen      : terminal size, refreshed on resize.
 *   sim_fps     : how many sim steps per second; ]/[ adjust it.
 *   map_w,      : grid size, chosen from the terminal size minus the
 *   map_h         rows the HUD needs.
 *   running     : turns false on quit (q/ESC or Ctrl-C).
 *   need_resize : set when the window is resized; the loop acts on it.
 *
 * The two flags are volatile sig_atomic_t because that's the only kind
 * of variable a signal handler may safely set, and volatile stops the
 * compiler from caching a stale copy in the loop.
 */
typedef struct {
    Scene                 scene;        /* the simulation                       */
    Screen                screen;       /* terminal size                        */
    int                   sim_fps;      /* sim steps per second (]/[)           */
    int                   map_w;        /* grid width  (cells)                  */
    int                   map_h;        /* grid height (cells)                  */
    volatile sig_atomic_t running;      /* false -> quit                        */
    volatile sig_atomic_t need_resize;  /* window was resized                   */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
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

/* ---------- one helper per key, so the keymap stays readable ---------- *
 *
 * Each takes +1 for "next" or -1 for "previous" where that applies.
 */

/* Step to the next/previous pattern, wrapping around the ends. */
static void scene_cycle_pattern(Scene *s, int direction)
{
    int next = ((int)s->pattern.current + direction + N_PATTERNS) % N_PATTERNS;
    s->pattern.current = (Pattern)next;
}

/* Step to the next/previous theme and load its colors. */
static void scene_cycle_theme(Scene *s, int direction)
{
    s->palette.current = (s->palette.current + direction + N_THEMES)
                         % N_THEMES;
    theme_apply(s->palette.current);
}

/* Double / halve the drift speed, so it changes in clear steps. */
static void scene_drift_double(PatternState *ps)
{
    if (ps->drift_mult < DRIFT_MULT_MAX) ps->drift_mult *= 2;
    if (ps->drift_mult > DRIFT_MULT_MAX) ps->drift_mult = DRIFT_MULT_MAX;
}
static void scene_drift_halve(PatternState *ps)
{
    ps->drift_mult /= 2;
    if (ps->drift_mult < DRIFT_MULT_MIN) ps->drift_mult = DRIFT_MULT_MIN;
}

/* Nudge the sim tick rate up/down, kept within its allowed range. */
static void app_adjust_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* Handle one key press. Returns false if it was a quit key. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            s->paused = !s->paused;                 break;
    case 'r': case 'R':  scene_reset(s, app->map_w, app->map_h); break;

    case '=': case '+':  scene_drift_double(&s->pattern);        break;
    case '-':            scene_drift_halve (&s->pattern);        break;

    case ']':            app_adjust_sim_fps(app, +SIM_FPS_STEP); break;
    case '[':            app_adjust_sim_fps(app, -SIM_FPS_STEP); break;

    case 't':            scene_cycle_theme  (s, +1);             break;
    case 'T':            scene_cycle_theme  (s, -1);             break;

    case 'n': case 'N':  scene_cycle_pattern(s, +1);             break;
    case 'p': case 'P':  scene_cycle_pattern(s, -1);             break;

    default:                                                     break;
    }
    return true;
}

/* ---------- FrameClock: the main loop's timekeeping ------------------ */

/*
 * FrameClock — the timekeeping the main loop needs. It does two jobs:
 *
 *   1) Keep the sim running at a steady rate. Each frame's elapsed time
 *      is added to a running total, which is then spent one fixed-size
 *      step at a time, so the sim always advances by the same amount
 *      regardless of how choppy the frame rate is.
 *      (Glenn Fiedler, "Fix Your Timestep!")
 *
 *   2) Measure the frame rate for the HUD: count frames over a short
 *      window and divide.
 *
 *   frame_time_ns   : when the current frame started.
 *   sim_accum_ns    : elapsed time collected but not yet spent on steps.
 *   fps_accum_ns    : time piled up since the last fps update.
 *   fps_frame_count : frames since the last fps update.
 *   fps_display     : the latest fps number — the only field the HUD reads.
 */
typedef struct {
    int64_t frame_time_ns;     /* when this frame started                 */
    int64_t sim_accum_ns;      /* time waiting to be spent on sim steps   */
    int64_t fps_accum_ns;      /* time since last fps update              */
    int     fps_frame_count;   /* frames since last fps update            */
    double  fps_display;       /* latest fps, shown in the HUD            */
} FrameClock;

static void frame_clock_init(FrameClock *fc)
{
    fc->frame_time_ns   = clock_ns();
    fc->sim_accum_ns    = 0;
    fc->fps_accum_ns    = 0;
    fc->fps_frame_count = 0;
    fc->fps_display     = 0.0;
}

/* Reset the clock after a pause (like a window resize) so the sim
 * doesn't try to fast-forward through time the user never saw. */
static void frame_clock_resync(FrameClock *fc)
{
    fc->frame_time_ns = clock_ns();
    fc->sim_accum_ns  = 0;
}

/* Return how long since the last frame, capped so one slow frame can't
 * make the sim try to catch up forever. */
static int64_t frame_clock_advance(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->frame_time_ns;
    fc->frame_time_ns = now;
    if (dt > MAX_FRAME_DT_NS) dt = MAX_FRAME_DT_NS;
    return dt;
}

/* Count this frame; every so often work out the frame rate and update
 * the number shown in the HUD. */
static void fps_meter_observe(FrameClock *fc, int64_t frame_dt_ns)
{
    fc->fps_frame_count++;
    fc->fps_accum_ns += frame_dt_ns;
    if (fc->fps_accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        double elapsed_sec = (double)fc->fps_accum_ns / (double)NS_PER_SEC;
        fc->fps_display     = (double)fc->fps_frame_count / elapsed_sec;
        fc->fps_frame_count = 0;
        fc->fps_accum_ns    = 0;
    }
}

/* ---------- pieces of one trip through the main loop ------------------ */

/* Add this frame's elapsed time to the pile, then run the sim one
 * fixed-size step at a time until the pile is used up. Each step gets
 * the same dt, so the sim runs the same whether frames are smooth or
 * choppy. (Glenn Fiedler, "Fix Your Timestep!") */
static void app_run_fixed_step_ticks(App *app, FrameClock *fc,
                                     int64_t frame_dt_ns)
{
    int64_t tick_ns     = TICK_NS(app->sim_fps);
    float   tick_dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    fc->sim_accum_ns += frame_dt_ns;
    while (fc->sim_accum_ns >= tick_ns) {
        scene_tick(&app->scene, tick_dt_sec);
        fc->sim_accum_ns -= tick_ns;
    }
}

/* Sleep off the leftover time so the loop runs at the target frame rate
 * instead of spinning as fast as it can. */
static void app_throttle_to_render_rate(int64_t frame_start_ns,
                                        int64_t frame_dt_ns)
{
    int64_t target_frame_period_ns = NS_PER_SEC / RENDER_FPS_TARGET;
    int64_t time_consumed_ns       = clock_ns() - frame_start_ns
                                   + frame_dt_ns;
    clock_sleep_ns(target_frame_period_ns - time_consumed_ns);
}

/* Check for a key press (without blocking) and act on it. */
static void app_pump_input(App *app)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
        app->running = 0;
}

/* Catch Ctrl-C / kill (quit) and window-resize signals. */
static void app_install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

int main(void)
{
    /* set up: random seed, cleanup-on-exit, signal handlers */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    app_install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    /* open the terminal, pick a grid size, build the simulation */
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    FrameClock clock;
    frame_clock_init(&clock);

    /* each pass: handle a resize, advance the sim, draw, read a key */
    while (app->running) {
        if (app->need_resize) {
            app_do_resize(app);
            frame_clock_resync(&clock);
        }

        int64_t frame_dt_ns = frame_clock_advance(&clock);
        app_run_fixed_step_ticks   (app, &clock, frame_dt_ns);
        fps_meter_observe          (&clock, frame_dt_ns);
        app_throttle_to_render_rate(clock.frame_time_ns, frame_dt_ns);

        screen_draw(&app->screen, &app->scene,
                    clock.fps_display, app->sim_fps);
        screen_present();
        app_pump_input(app);
    }

    screen_free(&app->screen);
    return 0;
}
