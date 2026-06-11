/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* leaf_fall.c — ASCII tree with matrix-rain leaf fall
 *
 * States: DISPLAY (2.5 s static tree) → FALLING (matrix-rain leaf drop)
 *         → RESET (brief blank) → new algorithmically varied tree
 *
 * Keys: q/ESC quit   r new tree   spc skip state
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/leaf_fall.c -o leaf_fall -lncurses -lm
 *
 * §1 config   §2 performance   §3 simulation state   §4 logic
 * §5 simulation   §6 render   §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two-phase animation: (1) stochastic recursive tree growth
 *                  using a branching stack (iterative DFS), then (2) matrix-
 *                  rain-style leaf fall where each leaf column streams downward
 *                  with a white head and green trail.
 *
 * Data-structure : Leaf pool (MAX_LEAVES=4096) of (col, row, delay) structs.
 *                  Each leaf is assigned to a column of the rendered tree;
 *                  start delay staggers the fall so leaves drop in waves.
 *                  Trail drawn by remembering last TRAIL_LEN=7 row positions.
 *
 * Rendering      : Tree drawn into a virtual grid (GRID_ROWS×GRID_COLS) using
 *                  recursive branching with angle jitter (BRANCH_SPREAD=0.50
 *                  radians range); leaf chars scattered at branch tips.
 *                  FALL_NS=55 ms per step → ~18 fps fall rate, slower than
 *                  the 30 fps render tick for smooth trailing effect.
 *
 * State machine  : DISPLAY → FALLING → RESET → (new tree) in a cycle.
 *                  Each state has a fixed time budget (DISPLAY_NS=2.5 s,
 *                  RESET_NS=0.7 s) measured with CLOCK_MONOTONIC timestamps.
 *
 * References     :
 *   Procedural tree growth (§5 grow_tree — stochastic recursive branching)
 *     [1] Prusinkiewicz & Lindenmayer, "The Algorithmic Beauty of Plants"
 *         (Springer, 1990) — the canonical procedural-plant / L-system text.
 *     [2] Lindenmayer, "Mathematical models for cellular interactions in
 *         development," J. Theoretical Biology 18 (1968) — original L-systems.
 *   Rasterisation
 *     [3] Bresenham, "Algorithm for computer control of a digital plotter,"
 *         IBM Systems Journal 4(1) (1965) — the integer line drawer used by
 *         draw_branch_line to paint branches into the TreeGrid.
 *   Effect
 *     [4] "Matrix" digital rain — per-column streamer with a bright head and a
 *         fading trail; recycled here with the column AND the start row taken
 *         from the tree's foliage (cf. the project's matrix_rain.c).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The tree is grown ONCE into a cell grid (chars + colour-pair codes),
 * then frozen.  The leaves remember their birthplace (orig_row, orig_col)
 * and a single moving HEAD row.  When fall begins, each leaf's head
 * starts at orig_row and walks downward at its own period; the trail is
 * not stored — it's the last TRAIL_LEN rows above the head, drawn live
 * each frame with the head white-bold and the trail solid green.  This
 * is the matrix-rain composition recycled: per-column streamers, but the
 * column AND the start row come from the tree's foliage layout.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Two layers stacked.  Bottom layer: a still painting (trunk + branches
 * + leaves).  Top layer: vertical waterfalls, one per leaf, each with a
 * white droplet at the bottom and a fading green tail trailing upward.
 * In DISPLAY the waterfall is invisible — you see the painting.  In
 * FALLING the waterfall switches on per-leaf as its start_delay expires;
 * the painting's leaf cells turn into droplets and the static green
 * leaves disappear from their birthplace as the head leaves it behind.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. scene_new_tree()  — reseed LCG, clear grid, run grow_tree() which
 *     pushes branch tasks to a fixed-size Branch stack and processes them
 *     iteratively (DFS).  Branch ends call place_foliage(), which
 *     scatters leaves in an aspect-corrected ellipse (1×2 axes ratio
 *     compensates for non-square cells).  Each leaf gets a random
 *     start_delay in [0, MAX_START_DELAY) and fall_period in {1,2,3}.
 *  2. PHASE_DISPLAY for DISPLAY_NS; HUD shows static painting.
 *  3. PHASE_FALLING:  every FALL_NS the fall_tick() advances each started
 *     leaf's head_row when its sub-counter hits its period.  Leaves
 *     finish (done=true) when head exceeds rows + TRAIL_LEN.
 *  4. scene_draw() during fall: trunk/branch cells are drawn from the
 *     grid as-is.  Leaves not yet started → static green at orig.
 *     Started leaves → for j in [TRAIL_LEN..0], draw cell at
 *     (head_row − j, orig_col).  j=0 is the white head; j>0 is the
 *     green tail.  Trail is clipped at orig_row so rain starts AT the
 *     leaf, not above it.
 *  5. When all_done(), enter PHASE_RESET for RESET_NS, then loop with a
 *     new seed (g_cycle increments to vary tree shape).
 *
 * KEY FORMULAS
 * ────────────
 *  Branch endpoint (screen y-up, sin uses negation):
 *      er = r − ⌊sin(θ) · len⌋,   ec = c + ⌊cos(θ) · len⌋
 *  Foliage ellipse test (aspect-correct):
 *      d  = dr² + 0.25·dc²            (so semi-axes are rad × 2·rad)
 *      keep cell if d ≤ rad²
 *  Foliage density:  thresh = 0.62 + 0.33·d/(rad²+1); rng > thresh
 *  Leaf fall trigger:  ++fall_sub >= fall_period → head_row++
 *  LCG step       :  s = s · 1664525 + 1013904223
 *  Trunk height   :  h = (TRUNK_H_MIN..MAX) · g_rows  (≈45–65 % of screen)
 *
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * Four real layers; two are intentionally absent.
 *
 *   LAYER        SECTION  MUTATES
 *   ────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — clock primitives (the RENDER_NS frame cap
 *                           and FALL_NS fall accumulator live in §7 main)
 *   SIMULATION   §3,§5    §3 holds the state; §5 builds & advances it:
 *                           g_tree.ch/g_tree.cp (the frozen tree painting),
 *                           g_leaves[]/g_leaf_count (pool + per-tick head_row),
 *                           g_seed (PRNG), g_phase/g_phase_t/g_cycle/g_fall_tick
 *   LOGIC        §4       nothing — pure predicates (in_grid, all_done)
 *   RENDER       §6       the terminal only (ncurses); READS grid/leaves/state,
 *                           never writes them
 *
 *   EFFECTS  : ABSENT — the matrix-rain trail is NOT stored; it is redrawn each
 *              frame as the last TRAIL_LEN rows above head_row (scene_draw).
 *   DELAYS   : the two timed holds (DISPLAY_NS, RESET_NS) are not a module —
 *              they are elapsed-time comparisons in the state machine at the
 *              combine point (§7).  There is no pause feature.
 *
 * GENERATION vs TICK: grow_tree() and its helpers BUILD a tree once per cycle
 * (the reset / new-tree path); fall_tick() is the only PER-TICK advance.
 *
 * LOGIC (§4) is provably uncorruptable from RENDER: it does no mutation and no
 * I/O, so reordering or deleting any draw cannot change in_grid / all_done.
 *
 * Per-tick combine order — main() (§7) is the ONLY place that advances state:
 *     1. PERFORMANCE  measure dt (capped at 100 ms)              §7
 *     2. SIMULATION   state-machine transitions (timed / all_done) §7 + §4/§5
 *     3. PERFORMANCE  fall accumulator → fall_tick() per FALL_NS  §7 → §5
 *     4. RENDER       scene_draw() + HUD (read-only)             §6 + §7
 *     5. PERFORMANCE  sleep to the RENDER_NS frame cap           §7
 *
 * User events (q/r/spc keys, SIGWINCH resize) may rebuild the tree or change
 * state but are NOT part of the tick — they are handled at the top of the loop
 * before the tick (§7): the g_resize block and the getch() branch.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── §1 config ────────────────────────────────────────────────────────────── */
#define RENDER_FPS      30
#define RENDER_NS       (1000000000LL / RENDER_FPS)
#define FALL_NS         55000000LL    /* 55 ms per fall step (~18 fps) */
#define DISPLAY_NS      2500000000LL  /* 2.5 s static display */
#define RESET_NS        700000000LL   /* 0.7 s blank between trees */
#define DT_CAP_NS       100000000LL   /* clamp frame dt to 100 ms (spiral guard) */
#define TRAIL_LEN       7             /* length of green trail behind white head */
#define MAX_LEAVES      4096
#define GRID_ROWS       128
#define GRID_COLS       320
#define MAX_DEPTH       7
#define BSTACK_MAX      512
#define MAX_START_DELAY 80            /* ticks — stagger leaf fall start */

static const float TRUNK_H_MIN  = 0.45f;
static const float TRUNK_H_MAX  = 0.65f;
static const float BRANCH_SPREAD = 0.50f;

/* Foliage scatter (place_foliage): aspect-correct ellipse + edge-sparse density */
#define FOLIAGE_ASPECT          0.25f  /* dc² weight = (1/2)² → round on screen */
#define FOLIAGE_DENSITY_BASE    0.62f  /* keep-probability floor at patch centre */
#define FOLIAGE_DENSITY_FALLOFF 0.33f  /* extra sparseness toward the edge       */

/* Colour pair IDs */
#define CP_TRUNK  1
#define CP_BRANCH 2
#define CP_LEAF   3
#define CP_FALL_H 4   /* white head */
#define CP_FALL_G 5   /* green trail body */
#define CP_HUD    6   /* top: data readout */
#define CP_HINT   7   /* bottom: action keys */

/* ── §2 performance — timing primitives (frame cap / accumulators in §7) ───── */
static long long clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 simulation state — the data SIMULATION owns (mutated only by §5) ────── */

/* TreeGrid — the tree rasterised ONCE into a fixed cell grid: a glyph plus a
 * colour-pair code per cell.
 *
 * WHY a grid (a framebuffer): the tree is produced by stochastic recursive
 * branching, but we want a STATIC image to hold on screen for 2.5 s and to
 * anchor the leaf fall.  So grow_tree() rasterises the whole branching process
 * ONCE into this grid (branch lines via Bresenham, ref [3]) and freezes it;
 * every frame then just blits cells — generation cost is paid once, not per
 * frame.  Struct-of-arrays (parallel ch/cp) keeps each cell to a char + an
 * int8_t with no struct padding, versus an array-of-structs.
 *
 * VALUE LOGIC: ch[r][c]==0 marks an empty cell (skipped by the renderer); cp
 * is the colour-pair tag (CP_TRUNK / CP_BRANCH / CP_LEAF) that doubles as a
 * cell TYPE — add_leaf refuses to overwrite trunk/branch cells, and scene_draw
 * animates CP_LEAF cells while drawing the rest verbatim. */
typedef struct {
    char   ch[GRID_ROWS][GRID_COLS];   /* glyph per cell (0 = empty)     */
    int8_t cp[GRID_ROWS][GRID_COLS];   /* colour-pair code + cell type   */
} TreeGrid;
static TreeGrid g_tree;

/* Leaf — one foliage cell that becomes a "digital-rain" streamer when it falls
 * (the Matrix-rain effect, ref [4]: a bright white head trailing a fading green
 * tail).
 *
 * WHY store so little: the trail is NOT kept anywhere — only the single moving
 * head_row is stored, and scene_draw re-derives the last TRAIL_LEN rows above
 * it each frame.  That shrinks a streamer of arbitrary length to ~16 bytes, so
 * the whole canopy (up to MAX_LEAVES) fits in a flat pool with no allocation.
 *
 * CONTEXT: leaves are created by place_foliage() at branch tips; the per-leaf
 * randomised start_delay + fall_period make the canopy drop in staggered waves
 * rather than as one sheet.
 *
 * VALUE LOGIC: the head walks from orig_row downward; clipping the trail at
 * orig_row makes the rain ORIGINATE at the leaf, not above it. */
typedef struct {
    int16_t orig_row, orig_col; /* birthplace in the grid (and trail origin)   */
    char    ch;                 /* the leaf glyph (chosen from k_lch)          */
    int16_t head_row;           /* current head row; walks down during FALLING */
    bool    started;            /* has start_delay elapsed? (fall has begun)   */
    bool    done;               /* head passed bottom + trail → fully gone     */
    int16_t start_delay;        /* ticks to wait before falling (staggers waves) */
    int16_t fall_period;        /* ticks per row step: 1=fast 2=med 3=slow     */
    int16_t fall_sub;           /* sub-tick counter toward fall_period         */
} Leaf;

/* Leaf pool + active screen extent + PRNG state. */
static Leaf  g_leaves[MAX_LEAVES];
static int   g_leaf_count;
static int   g_rows, g_cols;
static unsigned g_seed;

/* Fall sub-tick counter (advanced by fall_tick). */
static int g_fall_tick;

/* Phase — the scene's timed animation cycle, as a finite state machine: show
 * the finished tree, drop its leaves, blank briefly, then regrow.
 *
 * WHY a cycle: the demo runs forever, varying the tree each round (g_cycle
 * perturbs the seed).  Each phase owns a different exit condition, so the loop
 * stays a clean 3-state machine rather than a tangle of timers.
 *
 * VALUE LOGIC (transitions evaluated in main):
 *   DISPLAY → FALLING  once DISPLAY_NS has elapsed
 *   FALLING → RESET    when all_done() (every leaf off-screen)
 *   RESET   → DISPLAY  after RESET_NS, via scene_new_tree() (fresh seed). */
typedef enum { PHASE_DISPLAY, PHASE_FALLING, PHASE_RESET } Phase;
static Phase     g_phase;     /* current animation phase                    */
static long long g_phase_t;   /* clock_ns timestamp when this phase began   */
static unsigned  g_cycle;     /* trees shown so far (also varies the seed)  */

/* ── §4 logic — pure predicates over the state: no mutation, no I/O ─────────── */

static bool in_grid(int r, int c) {
    return r >= 0 && r < g_rows && r < GRID_ROWS &&
           c >= 0 && c < g_cols && c < GRID_COLS;
}

static bool all_done(void) {
    if (g_leaf_count == 0) return true;
    for (int i = 0; i < g_leaf_count; i++)
        if (!g_leaves[i].done) return false;
    return true;
}

/* ── §5 simulation — builds the tree (generation) and advances the fall ─────── */

/* LCG — deterministic from seed */
static unsigned lcg(void)  { g_seed = g_seed * 1664525u + 1013904223u; return g_seed; }
static float    lcgf(void) { return (float)(lcg() & 0x7fffffffu) / (float)0x7fffffffu; }

static void grid_clear(void) {
    memset(g_tree.ch, 0, sizeof(g_tree.ch));
    memset(g_tree.cp,  0, sizeof(g_tree.cp));
    g_leaf_count = 0;
}

static void set_cell(int r, int c, char ch, int8_t cp) {
    if (in_grid(r, c)) { g_tree.ch[r][c] = ch; g_tree.cp[r][c] = cp; }
}

static void add_leaf(int r, int c, char ch) {
    if (!in_grid(r, c))                                          return;
    if (g_tree.cp[r][c] == CP_TRUNK || g_tree.cp[r][c] == CP_BRANCH)   return;
    if (g_tree.cp[r][c] == CP_LEAF)                                  return;
    if (g_leaf_count >= MAX_LEAVES)                              return;
    set_cell(r, c, ch, CP_LEAF);
    g_leaves[g_leaf_count++] = (Leaf){
        .orig_row    = (int16_t)r,
        .orig_col    = (int16_t)c,
        .ch          = ch,
        .head_row    = (int16_t)r,
        .started     = false,
        .done        = false,
        .start_delay = (int16_t)(lcg() % MAX_START_DELAY),
        .fall_period = (int16_t)(1 + (int)(lcg() % 3)),  /* 1=fast 2=med 3=slow */
        .fall_sub    = 0,
    };
}

static const char k_lch[] = "*@&#%o~";

/* elliptical foliage patch — aspect-correct so it looks round on screen */
static void place_foliage(int r, int c, int rad) {
    if (rad < 1) rad = 1;
    int nlch = (int)(sizeof(k_lch) - 1);
    for (int dr = -rad; dr <= rad; dr++) {
        for (int dc = -(rad * 2); dc <= (rad * 2); dc++) {
            /* ellipse: semi-axes rad (rows) × 2*rad (cols) ≈ circle on terminal */
            float d = (float)(dr * dr) + FOLIAGE_ASPECT * (float)(dc * dc);
            if (d > (float)(rad * rad)) continue;
            /* density falls off toward edge — keep sparse */
            float thresh = FOLIAGE_DENSITY_BASE
                         + FOLIAGE_DENSITY_FALLOFF * d / (float)(rad * rad + 1);
            if (lcgf() > thresh) {
                char ch = k_lch[lcg() % (unsigned)nlch];
                add_leaf(r + dr, c + dc, ch);
            }
        }
    }
}

/* Glyph for a branch line from its run direction: vertical, horizontal, diagonal. */
static char branch_glyph(int dr, int dc, int sr, int sc) {
    return (dc == 0) ? '|' :
           (dr == 0) ? '-' :
           (sr * sc > 0) ? '\\' : '/';
}

/* Bresenham line (ref [3]) — rasterise a branch into the TreeGrid. */
static void draw_branch_line(int r0, int c0, int r1, int c1, int8_t cp) {
    int dr = abs(r1 - r0), dc = abs(c1 - c0);
    int sr = (r1 > r0) ? 1 : -1, sc = (c1 > c0) ? 1 : -1;
    int err = dc - dr;
    int r = r0, c = c0;
    char ch = branch_glyph(dr, dc, sr, sc);
    for (;;) {
        /* don't overwrite thick trunk chars with thinner branch chars */
        if (in_grid(r, c) && g_tree.cp[r][c] != CP_TRUNK) {
            g_tree.ch[r][c] = ch;
            g_tree.cp[r][c]  = cp;
        }
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -dr) { err -= dr; c += sc; }
        if (e2 <  dc) { err += dc; r += sr; }
    }
}

/* Branch — one pending branch segment on the growth stack: a turtle-graphics
 * state (position + heading + length + recursion depth).
 *
 * WHY an explicit stack instead of recursion: grow_tree() is an iterative DFS
 * — it pushes child Branches onto g_bstack and pops them in a loop.  This caps
 * memory at BSTACK_MAX (no unbounded call stack / overflow on deep trees) and
 * lets the generator bail cleanly when the stack is full.  The stochastic
 * branching itself is the procedural-plant / L-system idea (refs [1][2]).
 *
 * VALUE LOGIC: (r,c) is the segment's start cell; angle is its heading in
 * radians (M_PI/2 = straight up, since screen y is inverted); len is its length
 * in cells; depth is the recursion level — it selects thickness (CP_TRUNK when
 * ≤2, else CP_BRANCH) and triggers the switch to foliage at MAX_DEPTH. */
typedef struct { int r, c; float angle; int len; int depth; } Branch;
static Branch g_bstack[BSTACK_MAX];

/* Draw the trunk: a 2-cell column, a wider flare at the base, and root buttresses. */
static void draw_trunk(int base_row, int base_col, int trunk_h, int trunk_top) {
    /* 2-cell wide trunk */
    for (int row = base_row; row >= trunk_top; row--) {
        set_cell(row, base_col,     '|', CP_TRUNK);
        set_cell(row, base_col + 1, '|', CP_TRUNK);
    }
    /* wider flare at base (~25% of trunk height) */
    int flare = trunk_h / 4;
    for (int row = base_row; row >= base_row - flare; row--) {
        set_cell(row, base_col - 1, '|', CP_TRUNK);
        set_cell(row, base_col + 2, '|', CP_TRUNK);
    }
    /* root buttresses at very bottom */
    set_cell(base_row, base_col - 2, '/', CP_TRUNK);
    set_cell(base_row, base_col + 3, '\\', CP_TRUNK);
}

/* Seed the growth stack with 3–5 pairs of primary branches up the trunk
 * (longest at mid-canopy).  Returns the new stack top. */
static int push_trunk_branches(int base_row, int base_col, int trunk_h, int bsp) {
    int n_pts = 3 + (int)(lcgf() * 3.0f);   /* 3–5 */
    for (int i = 0; i < n_pts && bsp < BSTACK_MAX - 2; i++) {
        float hf     = (float)(i + 1) / (float)(n_pts + 1);
        int   br     = base_row - (int)(hf * (float)trunk_h * 0.95f);
        float spread = BRANCH_SPREAD + lcgf() * 0.35f;
        /* branches are longer at mid-canopy */
        float len_f  = 0.25f + 0.35f * (1.0f - fabsf(hf - 0.5f) * 2.0f) + lcgf() * 0.10f;
        int   blen   = (int)(len_f * (float)trunk_h);
        if (blen < 3) blen = 3;
        g_bstack[bsp++] = (Branch){ br, base_col,     (float)M_PI/2.0f + spread, blen, 1 };
        g_bstack[bsp++] = (Branch){ br, base_col + 1, (float)M_PI/2.0f - spread, blen, 1 };
    }
    return bsp;
}

/* Seed the top crown: two spreading branches plus one straight-up shoot. */
static int push_crown_branches(int base_col, int trunk_top, int trunk_h, int bsp) {
    if (bsp < BSTACK_MAX - 3) {
        int   top_len = trunk_h / 3;
        if (top_len < 3) top_len = 3;
        float ts = 0.55f + lcgf() * 0.30f;
        g_bstack[bsp++] = (Branch){ trunk_top, base_col,     (float)M_PI/2.0f + ts,   top_len,          1 };
        g_bstack[bsp++] = (Branch){ trunk_top, base_col + 1, (float)M_PI/2.0f - ts,   top_len,          1 };
        g_bstack[bsp++] = (Branch){ trunk_top, base_col,     (float)M_PI/2.0f,         (int)(top_len*0.8f), 1 };
    }
    return bsp;
}

/* Branch endpoint from heading θ and length (screen y inverted → sin negated). */
static void branch_endpoint(const Branch *t, int *er, int *ec) {
    *er = t->r - (int)(sinf(t->angle) * (float)t->len);
    *ec = t->c + (int)(cosf(t->angle) * (float)t->len);
}

/* Expand one popped branch: cap it with foliage (terminal / max depth), or draw
 * its line and push child branches at depth+1.  Mutates the stack via *bsp. */
static void grow_branch(Branch t, int *bsp) {
    if (t.len <= 1 || t.depth > MAX_DEPTH) {
        int frad = 2 + (MAX_DEPTH - t.depth) / 2;
        if (frad < 1) frad = 1;
        if (frad > 5) frad = 5;
        place_foliage(t.r, t.c, frad);
        return;
    }

    int er, ec;
    branch_endpoint(&t, &er, &ec);

    int8_t cp = (t.depth <= 2) ? CP_TRUNK : CP_BRANCH;
    draw_branch_line(t.r, t.c, er, ec, cp);

    /* sparse foliage along mid/deep branches */
    if (t.depth >= 4 && lcgf() < 0.40f)
        place_foliage(er, ec, 1);

    if (t.depth >= MAX_DEPTH) {
        place_foliage(er, ec, 3);
        return;
    }

    float spread = 0.30f + lcgf() * 0.30f;
    int   slen   = (int)((float)t.len * (0.50f + lcgf() * 0.20f));
    if (slen < 2) slen = 2;

    if (*bsp < BSTACK_MAX - 3) {
        g_bstack[(*bsp)++] = (Branch){ er, ec, t.angle + spread, slen, t.depth + 1 };
        g_bstack[(*bsp)++] = (Branch){ er, ec, t.angle - spread, slen, t.depth + 1 };
        /* occasional straight-ahead sub-branch */
        if (lcgf() < 0.35f && *bsp < BSTACK_MAX - 1) {
            float jitter = (lcgf() - 0.5f) * 0.20f;
            g_bstack[(*bsp)++] = (Branch){ er, ec, t.angle + jitter,
                                            (int)((float)slen * 0.80f), t.depth + 1 };
        }
    }
}

/* Grow one whole tree into the grid: trunk, then an iterative-DFS branch expand. */
static void grow_tree(int base_row, int base_col) {
    /* trunk height (clamped to fit the screen) */
    int trunk_h = (int)((TRUNK_H_MIN + lcgf() * (TRUNK_H_MAX - TRUNK_H_MIN)) * (float)g_rows);
    if (trunk_h < 8) trunk_h = 8;
    int trunk_top = base_row - trunk_h;
    if (trunk_top < 2) { trunk_top = 2; trunk_h = base_row - trunk_top; }

    draw_trunk(base_row, base_col, trunk_h, trunk_top);

    /* seed the growth stack, then expand it depth-first */
    int bsp = 0;
    bsp = push_trunk_branches(base_row, base_col, trunk_h, bsp);
    bsp = push_crown_branches(base_col, trunk_top, trunk_h, bsp);

    while (bsp > 0) {
        Branch t = g_bstack[--bsp];
        grow_branch(t, &bsp);
    }
}

/* per-tick advance: walk each started leaf's head downward at its own period */
/* Advance one leaf: wait out start_delay, then step the head down one row every
 * fall_period ticks; mark done once it clears the bottom + trail. */
static void leaf_step(Leaf *lf) {
    if (lf->done) return;
    if (!lf->started) {
        if (g_fall_tick >= lf->start_delay) lf->started = true;
        else return;
    }
    if (++lf->fall_sub >= lf->fall_period) {
        lf->fall_sub = 0;
        lf->head_row++;
        if (lf->head_row > (int16_t)(g_rows + TRAIL_LEN)) lf->done = true;
    }
}

static void fall_tick(void) {
    g_fall_tick++;
    for (int i = 0; i < g_leaf_count; i++)
        leaf_step(&g_leaves[i]);
}

static void scene_new_tree(void) {
    /* vary seed per cycle for algorithmic diversity */
    g_seed = (unsigned)clock_ns() ^ (g_cycle * 0x9e3779b9u);
    g_cycle++;
    grid_clear();
    g_fall_tick = 0;
    grow_tree(g_rows - 2, g_cols / 2 - 1);
    g_phase   = PHASE_DISPLAY;
    g_phase_t = clock_ns();
}

static void scene_resize(int rows, int cols) {
    g_rows = (rows > 4 && rows < GRID_ROWS) ? rows : GRID_ROWS - 1;
    g_cols = (cols > 8 && cols < GRID_COLS) ? cols : GRID_COLS - 1;
    scene_new_tree();
}

/* ── §6 render — state → screen; reads only, never mutates sim state ────────── */
static void color_init(void) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_TRUNK,  130, -1);  /* brown */
        init_pair(CP_BRANCH, 94,  -1);  /* dark brown */
        init_pair(CP_LEAF,   82,  -1);  /* bright green */
        init_pair(CP_FALL_H, 231, -1);  /* white */
        init_pair(CP_FALL_G, 46,  -1);  /* vivid green */
        init_pair(CP_HUD,    226, -1);  /* bright yellow */
        init_pair(CP_HINT,    51, -1);  /* bright cyan */
    } else {
        init_pair(CP_TRUNK,  COLOR_YELLOW, -1);
        init_pair(CP_BRANCH, COLOR_YELLOW, -1);
        init_pair(CP_LEAF,   COLOR_GREEN,  -1);
        init_pair(CP_FALL_H, COLOR_WHITE,  -1);
        init_pair(CP_FALL_G, COLOR_GREEN,  -1);
        init_pair(CP_HUD,    COLOR_YELLOW, -1);
        init_pair(CP_HINT,   COLOR_CYAN,   -1);
    }
}

/* Blit the frozen trunk + branch cells (leaf cells are drawn separately). */
static void draw_tree_cells(void) {
    for (int r = 0; r < g_rows && r < GRID_ROWS; r++) {
        for (int c = 0; c < g_cols && c < GRID_COLS; c++) {
            char   ch = g_tree.ch[r][c];
            int8_t cp = g_tree.cp[r][c];
            if (!ch || cp == CP_LEAF) continue;   /* empty cell or a leaf cell */
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(r, c, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }
}

/* Draw one leaf: a static green glyph at rest, or a falling matrix-rain
 * streamer (white head + green trail clipped to its birthplace). */
static void draw_leaf(const Leaf *lf, bool falling) {
    if (!falling || !lf->started) {
        /* static green leaf at original position */
        attron(COLOR_PAIR(CP_LEAF) | A_BOLD);
        mvaddch(lf->orig_row, lf->orig_col, (chtype)(unsigned char)lf->ch);
        attroff(COLOR_PAIR(CP_LEAF) | A_BOLD);
        return;
    }
    if (lf->done) return;   /* fallen off screen — erase() cleared it */

    /* matrix-rain trail: green body + white head.  Clipped to orig_row so the
     * rain originates AT the leaf position, not above it. */
    for (int j = TRAIL_LEN; j >= 0; j--) {
        int r = lf->head_row - j;
        if (r < lf->orig_row || r < 0 || r >= g_rows) continue;
        if (j == 0) {
            attron(COLOR_PAIR(CP_FALL_H) | A_BOLD);
            mvaddch(r, lf->orig_col, (chtype)(unsigned char)lf->ch);
            attroff(COLOR_PAIR(CP_FALL_H) | A_BOLD);
        } else {
            attron(COLOR_PAIR(CP_FALL_G));
            mvaddch(r, lf->orig_col, (chtype)(unsigned char)lf->ch);
            attroff(COLOR_PAIR(CP_FALL_G));
        }
    }
}

static void scene_draw(void) {
    bool falling = (g_phase == PHASE_FALLING);
    draw_tree_cells();
    for (int i = 0; i < g_leaf_count; i++)
        draw_leaf(&g_leaves[i], falling);
}

static void screen_init(void) {
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* HUD — data readout top-right, action keys bottom-left (bright + A_BOLD). */
static void draw_hud(int rows, int cols) {
    const char *phase_name = (g_phase == PHASE_DISPLAY) ? "display" :
                             (g_phase == PHASE_FALLING) ? "falling" : "reset";
    char hud[80];
    snprintf(hud, sizeof hud, " %s  cycle:%u  leaves:%d ",
             phase_name, g_cycle, g_leaf_count);
    int hud_x = cols - (int)strlen(hud);
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hud_x, "%s", hud);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(rows - 1, 0, " q:quit  r:new tree  spc:skip ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §7 app — combine point (per-tick order) + user events + frame cap ──────── */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
    if (s == SIGINT  || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)                g_resize = 1;
}
static void cleanup(void) { endwin(); }

int main(void) {
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_resize(rows, cols);

    long long fall_acc = 0;
    long long last_ns  = clock_ns();

    while (!g_quit) {
        /* USER EVENT (out of tick): apply resize → rebuild the tree */
        if (g_resize) {
            g_resize = 0;
            getmaxyx(stdscr, rows, cols);
            scene_resize(rows, cols);
            last_ns  = clock_ns();
            fall_acc = 0;
            continue;
        }

        /* USER EVENT (out of tick): keys — quit / new tree / skip state */
        int ch = getch();
        if (ch == 'q' || ch == 27) break;
        if (ch == 'r') {
            scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
        }
        if (ch == ' ') {
            if (g_phase == PHASE_DISPLAY) {
                g_phase = PHASE_FALLING; g_phase_t = clock_ns();
            } else if (g_phase == PHASE_FALLING) {
                scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
            }
        }

        /* 1. PERFORMANCE — measure dt, capped to avoid spiral-of-death */
        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        last_ns = now_ns;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        long long elapsed = now_ns - g_phase_t;

        /* 2. SIMULATION — state-machine transitions (timed holds / all_done) */
        if (g_phase == PHASE_DISPLAY && elapsed >= DISPLAY_NS) {
            g_phase = PHASE_FALLING; g_phase_t = now_ns;
        }
        if (g_phase == PHASE_FALLING && all_done()) {
            g_phase = PHASE_RESET; g_phase_t = now_ns;
        }
        if (g_phase == PHASE_RESET && elapsed >= RESET_NS) {
            scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
            continue;
        }

        /* 3. PERFORMANCE — fall accumulator drives fall_tick at FALL_NS */
        if (g_phase == PHASE_FALLING) {
            fall_acc += dt;
            while (fall_acc >= FALL_NS) { fall_tick(); fall_acc -= FALL_NS; }
        }

        /* 4. RENDER — read-only: scene then HUD on top */
        erase();
        if (g_phase != PHASE_RESET) scene_draw();
        draw_hud(rows, cols);
        screen_present();

        /* 5. PERFORMANCE — sleep to the render frame cap */
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }

    return 0;
}
