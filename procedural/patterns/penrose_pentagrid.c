/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * penrose.c — Penrose Tiling (P3 Rhombus)
 *
 * Computes a Penrose rhombus tiling per terminal cell using de Bruijn's
 * pentagrid duality.  No tiles are stored; each cell is coloured in O(1)
 * from its pentagrid indices.  The view rotates slowly so the aperiodic
 * structure is clearly visible and no period is ever found.
 *
 * DE BRUIJN PENTAGRID METHOD
 * ──────────────────────────
 * Five families of parallel lines, family j with direction 2πj/5:
 *
 *   k_j(x,y) = ⌊ x·cos(2πj/5) + y·sin(2πj/5) − γ_j ⌋
 *
 * where γ_j are offset parameters (0 here for 5-fold symmetry at origin).
 *
 * The 5-tuple (k_0,…,k_4) uniquely identifies which Penrose rhombus a
 * point lies in.  The parity of S = k_0+k_1+k_2+k_3+k_4 distinguishes
 * the two rhombus types:
 *   S even → thick rhombus  (72° acute angle)
 *   S odd  → thin  rhombus  (36° acute angle)
 *
 * Adjacent cells in the same rhombus share the same k-tuple → same colour.
 * Cells in different rhombuses have different tuples → colour changes at
 * tile boundaries; this colouring already reveals the pattern, and the cells
 * nearest a grid line are additionally drawn as edge glyphs for clarity.
 *
 * ASPECT RATIO CORRECTION
 * ────────────────────────
 * Terminal cells are CELL_H/CELL_W ≈ 2× taller than wide.
 * Cell (col, row) is converted to pixel offset (px, py) before
 * projecting to the pentagrid.  This keeps rhombus proportions correct.
 *
 * ANIMATION
 * ─────────
 * The pixel coordinate frame rotates at ROTATE_SPEED rad/s.
 * The Penrose tiling has 5-fold symmetry, so period = 2π/5 ≈ 1.26 rad.
 * At 0.04 rad/s the view completes one "distinct cycle" in ~31 s, making
 * the aperiodic nature obvious — no configuration repeats.
 *
 * COLOUR SCHEME
 * ─────────────
 * Thick rhombuses (72° wide):   '*' A_BOLD in warm yellow/gold/amber tones.
 * Thin  rhombuses (36° narrow): '.' in cool cyan/light-blue/aqua tones.
 * Grid lines get directional edge glyphs - / | \ (magenta); the screen centre
 * carries an 'O' 5-fold axis marker.
 * shade = abs(k_0·3 + k_1·7 + k_2·11 + k_3·13 + k_4·17) mod N_SHADES gives
 * N_SHADES (= 3) shades per type, so adjacent same-type tiles differ in colour.
 *
 * Keys:
 *   q/ESC quit   space pause   r reset angle   +/- speed   ] / [  sim Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra penrose.c -o penrose -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : De Bruijn dual/pentagrid method for Penrose P3 rhombuses.
 *                  Rather than recursive deflation (splitting tiles), this
 *                  method projects a 5D integer lattice onto 2D via five
 *                  directions e_j = (cos 2πj/5, sin 2πj/5).  Each rhombus
 *                  corresponds to a pair of grid lines from two directions.
 *
 * Math           : A Penrose tiling is quasiperiodic: non-periodic but with
 *                  long-range order (well-defined diffraction peaks).  The
 *                  "inflation" symmetry: each tile can be subdivided into
 *                  φ² smaller tiles of the same two types (φ = golden ratio ≈ 1.618).
 *                  Ratio of thick:thin rhombus counts → φ as tiling grows.
 *                  No translational periodicity, but 5-fold local symmetry.
 *
 * Rendering      : Each terminal cell is classified by its 5-integer k-tuple
 *                  (which pentagrid lines it sits between).  Adjacent cells
 *                  in the same tile share the same tuple → same colour.
 *                  Animation rotates the sampling frame (the view angle)
 *                  slowly, scrolling the fixed quasiperiodic tiling past the
 *                  viewport so its non-repeating structure is visible.
 *
 * References     : Concept — the pentagrid duality and tile classification.
 *   [1] N. G. de Bruijn, "Algebraic theory of Penrose's non-periodic
 *       tilings of the plane, I & II," Indagationes Mathematicae 43 (1981),
 *       39-66.  THE source for the pentagrid / dual method this file
 *       implements: k-tuples, the parity-of-sum rule, ribbon geometry.
 *   [2] R. Penrose, "The role of aesthetics in pure and applied mathematical
 *       research," Bull. Inst. Math. Appl. 10 (1974), 266-271.  Original P3
 *       thick (72°) / thin (36°) rhombus tiling.
 *   [3] M. Gardner, "Extraordinary nonperiodic tiling that enriches the
 *       theory of tiles," Scientific American 236(1) (1977), 110-121.  The
 *       accessible first introduction to Penrose tilings.
 *   [4] B. Grünbaum & G. C. Shephard, "Tilings and Patterns," W. H. Freeman
 *       (1987), ch. 10.  Standard textbook treatment of aperiodic tilings.
 *   [5] M. Senechal, "Quasicrystals and Geometry," Cambridge Univ. Press
 *       (1995).  Cut-and-project view; why the parity sum S encodes tile
 *       type, and why the pattern is quasiperiodic (sharp diffraction).
 *
 *                  Rendering — mapping pentagrid indices to the screen.
 *   [6] D. Austin, "Penrose Tiles Talk Across Miles," AMS Feature Column
 *       (2005).  Free online; walks through de Bruijn's grid method step by
 *       step — closest match to this code's per-cell classification.
 *   [7] E. Durand, "QuasiTiler," The Geometry Center (1994).  Classic
 *       generalized-dual-method generator; reference for turning pentagrid
 *       line indices into on-screen tile coordinates (the rendering step).
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * A pure-visual demo: the entire simulation state is ONE rotation angle.
 * The concerns are cut into labelled layers so each reads in isolation.
 *
 *   LAYER        SECTION         MUTATES
 *   ──────────   ──────────────  ──────────────────────────────────────────
 *   PERFORMANCE  §2 + loop §5    frame_time, sim_accum, fps_accum, dt
 *                                (locals in main); never the sim state
 *   SIMULATION   §3              Penrose.angle (penrose_tick); all Penrose
 *                                fields at init/reset (lifecycle, not a tick)
 *   RENDER       §4              nothing — reads Scene/Penrose, writes only
 *                                to the ncurses screen
 *   APP/EVENTS   §5              Penrose.{angle,speed,paused} & App.sim_fps via
 *                                keys; App.running/need_resize via signals
 *
 * Concerns ABSENT or trivial here — no hollow section is created for them:
 *   LOGIC   — the pure decisions are small pure helpers in §4: edge_glyph
 *             (grid-line slope → ASCII glyph) and rhombus_shade (k-tuple →
 *             colour variant); only the one-line tile-type parity test stays
 *             inline in penrose_draw.
 *   EFFECTS — none.  Colours and edge glyphs are derived per cell from the
 *             k-tuple at draw time; no glow/trail/flash state is stored.
 *   DELAYS  — the only hold is the `paused` flag, checked at the top of
 *             penrose_tick.  No timers, no scheduled pauses.
 *
 * PER-TICK COMBINE ORDER (the ONE place state advances — main, §5):
 *   1. PERFORMANCE  dt = now - frame_time, clamp to 100ms, sim_accum += dt
 *   2. SIMULATION   while (sim_accum >= tick_ns) scene_tick(); sim_accum -= tick_ns
 *   3. PERFORMANCE  fps bookkeeping, then sleep to cap the frame at 60 fps
 *   4. RENDER       screen_draw() + screen_present()   (reads only)
 *   User events (key / reset / resize / signal) mutate state OUTSIDE this
 *   order and are NOT part of the tick — see app_handle_key and the resize
 *   block at the top of the loop.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,   /* pure visual; 30 fps is smooth enough       */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,

    /* Rotation-speed multiplier level (+/- keys; halve / double around    */
    /* SPEED_DEF, so SPEED_DEF == 1x of ROTATE_SPEED).                     */
    SPEED_MIN       =  1,
    SPEED_DEF       =  8,
    SPEED_MAX       = 64,

    HUD_TOP_ROWS    =  2,   /* rows 0-1 = data band; tiling starts at row 2 */

    N_SHADES        =  3,   /* colour variants per tile type — sizes the WARM/  */
                            /* COOL palettes and the modulus in rhombus_shade   */
};

/* HUD colour pairs — reuse the tiling palette (see color_init). */
#define HUD_DATA   8        /* yellow — top data band                      */
#define HUD_LABEL  4        /* cyan   — title + bottom action bar          */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Pixel cell dimensions */
#define CELL_W   8
#define CELL_H   16

/*
 * SCALE_PX — pixels per pentagrid unit.
 * With CELL_W=8 and SCALE_PX=80: 1 unit = 10 terminal columns.
 * Each rhombus side spans ~10 cols so tile shapes are clearly visible.
 */
#define SCALE_PX   80.0f

/*
 * BORDER — distance (in pentagrid units) from a grid line that is
 * rendered as a tile edge character instead of tile interior.
 * 0.15 → ~1-2 cell wide border at SCALE_PX=80.
 */
#define BORDER     0.15f

/*
 * ROTATE_SPEED — angular velocity of the view in radians per second.
 * 0.04 rad/s → one 5-fold period (72°) traversed in ~31 seconds.
 */
#define ROTATE_SPEED  0.04f

/* Tile colour-pair ids used as bare numbers at draw call-sites (the 1..6
 * interior pairs are abstracted by WARM/COOL; assignments live in color_init). */
#define PAIR_CENTRE  1      /* yellow  — 5-fold axis marker (same pair as WARM[0]) */
#define PAIR_EDGE    7      /* magenta — tile-edge glyphs                          */

/*
 * Edge-glyph slope sectors (radians, in [0, π)).  edge_glyph() buckets an
 * on-screen grid-line angle into one of four ASCII slopes; these boundaries
 * approximate π/12, π/3, 2π/3 and 11π/12 so each glyph covers the band of
 * angles it visually matches best.
 */
#define SLOPE_DASH_LO  0.26f   /* below this (or above _DASH_HI): '-' horizontal */
#define SLOPE_FSLASH   1.05f   /* below this: '/'                                */
#define SLOPE_PIPE     2.09f   /* below this: '|' vertical; otherwise '\'        */
#define SLOPE_DASH_HI  2.88f   /* above this: wraps back to '-' horizontal       */

/* ===================================================================== */
/* §2  PERFORMANCE — timing primitives                                    */
/* ===================================================================== */
/* Monotonic clock + sleep.  These are the primitives; the fixed-timestep */
/* accumulator, fps counter and 60 fps frame cap that USE them live at the */
/* single combine point in main (§5) — inseparable from the loop body.    */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  SIMULATION — state + advance                                       */
/* ===================================================================== */
/* The complete simulation state is the Penrose struct (one angle plus the */
/* speed level and pause flag).  penrose_tick is the ONLY function that    */
/* advances it; init/reset set it but are lifecycle, not part of a tick.   */
/*                                                                         */
/* DELAYS: the only hold is `paused`, tested at the top of penrose_tick —  */
/* there is no separate timer/delay layer.                                 */

/*
 * Penrose — the animated tiling.  The tiling itself is FIXED and infinite (it
 * is fully determined by the Pentagrid); what evolves over time is only the
 * frame we sample it through, which we rotate slowly so the eye can confirm the
 * pattern never repeats — the whole point of showing a quasiperiodic tiling
 * ([2], [5]).  All three fields describe that single rotation, which is why a
 * piece of state and two knobs cohere in one struct.
 *
 *   angle  — current view rotation in radians; the ONLY evolving simulation
 *            state.  Folded back into [0, 2π) every tick so float error cannot
 *            accumulate over a long run (see penrose_tick).  At the base rate
 *            ROTATE_SPEED ≈ 0.04 rad/s one 5-fold period (72°) passes in ~31 s.
 *   speed  — rotation-rate multiplier stored as an integer LEVEL
 *            (SPEED_MIN..SPEED_MAX; +/- keys double/halve it).  A level rather
 *            than a raw float so steps are clean and SPEED_DEF reads as exactly
 *            1× of ROTATE_SPEED; penrose_tick applies the ratio speed/SPEED_DEF.
 *   paused — run gate: while true penrose_tick returns early and angle freezes.
 *            One bool, so it is folded into the object it gates instead of being
 *            promoted to a separate run-state type.
 */
typedef struct {
    float angle;
    int   speed;
    bool  paused;
} Penrose;

static void penrose_init(Penrose *p)
{
    p->angle  = 0.0f;
    p->speed  = SPEED_DEF;
    p->paused = false;
}

static void penrose_tick(Penrose *p, float dt)
{
    if (p->paused) return;
    float speed_mul = (float)p->speed / (float)SPEED_DEF;
    p->angle += ROTATE_SPEED * speed_mul * dt;
    /* Keep in [0, 2π) to avoid float drift */
    if (p->angle >= 2.0f * (float)M_PI)
        p->angle -= 2.0f * (float)M_PI;
}

/*
 * Scene — the aggregate of everything being simulated, and the single hand-off
 * between the simulation layer (§3) and the render layer (§4).  It exists so
 * those layers share ONE owned object instead of reaching for globals — which
 * is what keeps the simulation and render layers cleanly separated.
 *
 * For this demo the entire simulated world is a single domain object, the
 * rotating Penrose tiling, so the "table of contents" has exactly one entry.
 * A richer simulation would list its independent sub-systems here, one field
 * each; we deliberately do not invent more, to avoid a hollow wrapper.
 *   penrose — WHAT is simulated: the rotating tiling (see Penrose).
 */
typedef struct {
    Penrose penrose;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    penrose_init(&s->penrose);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    penrose_tick(&s->penrose, dt);
}

/* ===================================================================== */
/* §4  RENDER — state → screen (reads only, never mutates sim state)      */
/* ===================================================================== */
/* Penrose works in pixel space for aspect correction (CELL_W/CELL_H); the */
/* one cell→pixel→pentagrid conversion lives inline in penrose_draw.       */
/*                                                                         */
/* LOGIC: the pure decisions are small pure helpers here — edge_glyph        */
/* (grid-line slope → glyph) and rhombus_shade (k-tuple → colour variant);   */
/* only the one-line parity test (thick vs thin) stays inline in penrose_draw.*/
/* EFFECTS: none — every colour/glyph is derived per cell from the k-tuple    */
/* at draw time; no cosmetic state (glow/trail/flash) is stored.             */

/*
 * Pentagrid — de Bruijn's five line-family directions, the core object of the
 * dual ("pentagrid") method [1].  A Penrose P3 tiling is the geometric dual of
 * an arrangement of FIVE infinite families of equally-spaced parallel lines
 * whose normals point along e_j = (cos 2πj/5, sin 2πj/5), j = 0..4 — i.e. unit
 * vectors at 0°, 72°, 144°, 216°, 288°.
 *
 * WHY FIVE, 72° APART: this spacing is what gives the tiling its 5-fold
 * symmetry; the crystallographic restriction forbids 5-fold *periodic* order,
 * so the dual is quasiperiodic instead — it never repeats ([2], [5]).  Any
 * other count/spacing collapses back to an ordinary periodic grid.
 * WHY THESE NUMBERS: cos/sin of the 72° multiples are the exact golden-ratio
 * constants (cos 72° = (√5−1)/4 ≈ 0.309, cos 144° = −(√5+1)/4 ≈ −0.809, …);
 * the φ ≈ 1.618 that governs Penrose tilings enters the algorithm right here.
 *
 * HOW IT IS USED: project a point p onto e_j and floor — k_j = ⌊p·e_j⌋ is p's
 * integer index in family j (which gap between two parallel lines it lies in).
 * The 5-tuple (k_0..k_4) names the rhombus; the parity of Σk_j selects thick
 * (even) vs thin (odd).  See penrose_draw, and [6] for the per-point walkthrough.
 *
 * WHY A STRUCT / WHY const + PRECOMPUTED: the two arrays are one indivisible
 * concept (the grid), so they travel together under one name.  penrose_draw
 * evaluates p·e_j for every cell every frame, so the trig is resolved once at
 * file scope and never recomputed — and never mutated (the grid is fixed; only
 * the view that samples it rotates, see Penrose).
 *   cos[j] / sin[j] — x / y component of direction e_j (j = 0 lies along +x).
 */
typedef struct {
    float cos[5];
    float sin[5];
} Pentagrid;

static const Pentagrid PENTAGRID = {
    /*        j=0     72°           144°          216°          288°        */
    .cos = {  1.0f,  0.30901699f, -0.80901699f, -0.80901699f,  0.30901699f },
    .sin = {  0.0f,  0.95105652f,  0.58778525f, -0.58778525f, -0.95105652f },
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 226, COLOR_BLACK);   /* yellow     — thick fill / centre   */
        init_pair(2, 220, COLOR_BLACK);   /* gold       — thick fill            */
        init_pair(3, 214, COLOR_BLACK);   /* amber      — thick fill            */
        init_pair(4,  51, COLOR_BLACK);   /* cyan       — thin fill / HUD label  */
        init_pair(5,  75, COLOR_BLACK);   /* light blue — thin fill             */
        init_pair(6,  87, COLOR_BLACK);   /* aqua       — thin fill             */
        init_pair(7, 201, COLOR_BLACK);   /* magenta    — tile edges            */
        init_pair(8, 226, COLOR_BLACK);   /* yellow     — HUD data band         */
    } else {
        init_pair(1, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(2, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_CYAN,    COLOR_BLACK);
        init_pair(5, COLOR_BLUE,    COLOR_BLACK);
        init_pair(6, COLOR_CYAN,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);
    }
}

/* Interior palettes, N_SHADES variants each (selected by rhombus_shade).
 * WARM = thick-rhombus fills (yellow, gold, amber); COOL = thin-rhombus fills
 * (cyan, light blue, aqua).  Values are color_init pair ids. */
static const int WARM[N_SHADES] = { 1, 2, 3 };
static const int COOL[N_SHADES] = { 4, 5, 6 };

/*
 * put_cell — paint one glyph at (row, col) in colour `pair` with attributes
 * `attr`, restoring attribute state afterwards.  The single point where a
 * coloured character reaches the screen, so every call site reads as one
 * "paint this tile" step.  Writes the screen only; reads no sim state.
 */
static void put_cell(WINDOW *w, int row, int col, int pair, int attr, chtype ch)
{
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, row, col, ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * edge_glyph — the ASCII slope of family `near_j`'s grid lines as they appear
 * on screen once the view has rotated by `view_angle`.  Family j's lines are
 * perpendicular to e_j (direction 2πj/5), so on screen they run at
 * 2πj/5 + π/2 − view_angle; folding that into [0, π) and bucketing by the
 * SLOPE_* sectors picks one of '-', '/', '|', '\'.  Pure: depends only on its
 * arguments.  See [1] for the line-family geometry.
 */
static char edge_glyph(int near_j, float view_angle)
{
    float ang = (float)(2.0 * M_PI * near_j / 5.0 + M_PI * 0.5) - view_angle;
    ang = fmodf(ang, (float)M_PI);
    if (ang < 0.0f) ang += (float)M_PI;

    if (ang < SLOPE_DASH_LO || ang > SLOPE_DASH_HI) return '-';
    if (ang < SLOPE_FSLASH)                         return '/';
    if (ang < SLOPE_PIPE)                           return '|';
    return '\\';
}

/*
 * rhombus_shade — choose one of N_SHADES colour variants for the rhombus with
 * pentagrid indices k[].  A spatial hash of the 5-tuple with small distinct
 * primes: same-type neighbours differ in their k-tuple, so they usually hash
 * to different shades, which makes tile boundaries visible without drawing
 * edges.  Pure read of k.
 */
static int rhombus_shade(const int k[5])
{
    return abs(k[0]*3 + k[1]*7 + k[2]*11 + k[3]*13 + k[4]*17) % N_SHADES;
}

/*
 * penrose_draw — render the Penrose tiling into window w.  Per terminal cell:
 *   (1) map the cell to a rotated, scaled point in pentagrid space;
 *   (2) classify the point against the pentagrid — its 5 indices k_j, the
 *       parity that types the tile, and the distance to the nearest grid line;
 *   (3) paint: on a grid-line band an edge glyph, else the tile interior.
 */
static void penrose_draw(const Penrose *p, WINDOW *w, int cols, int rows)
{
    float cx = (float)cols * 0.5f;          /* screen centre, in cells */
    float cy = (float)rows * 0.5f;
    float ca = cosf(p->angle);              /* view-rotation cos/sin   */
    float sa = sinf(p->angle);

    for (int row = HUD_TOP_ROWS; row < rows - 1; row++) {
        float py = ((float)row - cy) * (float)CELL_H;

        for (int col = 0; col < cols; col++) {
            float px = ((float)col - cx) * (float)CELL_W;

            /* (1) cell → pentagrid point: rotate by the view angle, then
             *     scale so one pentagrid unit ≈ SCALE_PX pixels */
            float rx = px * ca - py * sa;
            float ry = px * sa + py * ca;
            float wx = rx / SCALE_PX;
            float wy = ry / SCALE_PX;

            /* (2) classify against the pentagrid: floor index k_j per family,
             *     running sum for parity, and the nearest grid-line distance */
            int   k[5], sum = 0;
            float min_dist = 1.0f;          /* distance to nearest grid line   */
            int   near_j   = 0;             /* family owning that nearest line  */

            for (int j = 0; j < 5; j++) {
                float proj = wx * PENTAGRID.cos[j] + wy * PENTAGRID.sin[j];
                k[j] = (int)floorf(proj);
                sum += k[j];
                float frac         = proj - (float)k[j];                 /* ∈ [0,1) */
                float dist_to_line = frac < 0.5f ? frac : 1.0f - frac;   /* to nearer line */
                if (dist_to_line < min_dist) { min_dist = dist_to_line; near_j = j; }
            }

            bool thick = ((sum & 1) == 0);  /* de Bruijn parity: even Σk → thick tile */

            /* (3) paint this cell */
            if (min_dist < BORDER)          /* on a grid line → directional edge glyph */
                put_cell(w, row, col, PAIR_EDGE, A_DIM,
                         (chtype)(unsigned char)edge_glyph(near_j, p->angle));
            else if (thick)                 /* thick rhombus interior — warm bold fill */
                put_cell(w, row, col, WARM[rhombus_shade(k)], A_BOLD, '*');
            else                            /* thin rhombus interior — cool fill       */
                put_cell(w, row, col, COOL[rhombus_shade(k)], 0, '.');
        }
    }

    /* 5-fold symmetry axis marker at screen centre */
    int cc = (int)cx, cr = (int)cy;
    if (cc >= 0 && cc < cols && cr >= HUD_TOP_ROWS && cr < rows - 1)
        put_cell(w, cr, cc, PAIR_CENTRE, A_BOLD, 'O');
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    penrose_draw(&s->penrose, w, cols, rows);
}

/*
 * Screen — the terminal's current size in character cells, cached as the single
 * source of truth for "how big is the canvas".  WHY CACHE: the per-cell draw
 * loop and the HUD would otherwise call getmaxyx() repeatedly; we read it once
 * and refresh only when it can actually change — at start-up and on every
 * SIGWINCH (see the resize block in main).  A stale value here would clip or
 * wrap the tiling, hence it is updated before any draw.
 *   cols / rows — width / height in cells.  The tiling spans rows
 *                 HUD_TOP_ROWS..rows-2 (top data band + bottom action bar are
 *                 reserved) and cols 0..cols-1.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * hud_print — draw a HUD string at (y, x) clipped to the terminal width, so a
 * narrow window can never wrap HUD text down into the tiling. Right-aligned
 * callers pass x = cols - len; a negative x is pinned to 0 and the text is
 * truncated to whatever space remains.
 */
static void hud_print(int y, int x, int cols, int pair, int attr, const char *s)
{
    if (x < 0) x = 0;
    int avail = cols - x;
    if (avail <= 0) return;
    char tmp[128];
    snprintf(tmp, sizeof tmp, "%s", s);
    if ((int)strlen(tmp) > avail) tmp[avail] = '\0';   /* clip to fit */
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(y, x, "%s", tmp);
    attroff(COLOR_PAIR(pair) | attr);
}

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Penrose *p = &sc->penrose;

    /* ── top band: data (rows 0-1) ──────────────────────────────────── */
    hud_print(0, 1, s->cols, HUD_LABEL, A_BOLD, " PENROSE P3 ");

    char status[80];
    snprintf(status, sizeof status, " %5.1f fps  %3d Hz  speed:%-3d  %s ",
             fps, sim_fps, p->speed, p->paused ? "PAUSED " : "running");
    hud_print(0, s->cols - (int)strlen(status), s->cols, HUD_DATA, A_BOLD, status);

    char data[80];
    snprintf(data, sizeof data,
             " angle:%5.1f deg   *=thick(warm)  .=thin(cool)  /|\\-=edges ",
             (double)(p->angle * 180.0f / (float)M_PI));
    hud_print(1, 1, s->cols, HUD_DATA, 0, data);

    /* ── bottom band: actions (last row) ────────────────────────────── */
    hud_print(s->rows - 1, 0, s->cols, HUD_LABEL, A_BOLD,
              " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §5  APP — user events + per-tick combine                              */
/* ===================================================================== */
/* main is the ONE place that combines the layers per tick (PERFORMANCE → */
/* SIMULATION → PERFORMANCE → RENDER, see ARCHITECTURE block).  Signals    */
/* and app_handle_key mutate state OUTSIDE the tick and are not part of    */
/* the combine order.                                                      */

/*
 * App — the running program: the single owner of the scene plus the loop and
 * runtime state that the main tick and the asynchronous signal handlers must
 * share.  WHY A FILE-SCOPE INSTANCE (g_app): a signal handler receives only an
 * int, so the run flags it sets have to live somewhere it can reach without a
 * parameter — a global is the standard, and here the only, way.
 *
 *   scene       — WHAT is simulated (see Scene); the only domain data.
 *   screen      — cached terminal size (see Screen).
 *   sim_fps     — simulation tick rate in Hz ([ / ] keys); the fixed timestep
 *                 the accumulator in main targets.  A LOOP knob, kept apart from
 *                 Penrose.speed (a view-rotation knob) on purpose: different
 *                 concepts that merely happen to share the keyboard.
 *   running     — main-loop guard; cleared by SIGINT/SIGTERM (or the 'q' key) so
 *                 the loop exits and the terminal is restored.
 *   need_resize — set by SIGWINCH, consumed once at the top of the loop to
 *                 re-read the terminal size.  Both flags are volatile
 *                 sig_atomic_t — the only access a signal handler may safely make.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    Penrose *p = &app->scene.penrose;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': p->paused = !p->paused; break;
    case 'r': case 'R': p->angle = 0.0f; break;
    case '=': case '+':
        if (p->speed < SPEED_MAX) p->speed *= 2;
        if (p->speed > SPEED_MAX) p->speed  = SPEED_MAX;
        break;
    case '-':
        p->speed /= 2;
        if (p->speed < SPEED_MIN) p->speed  = SPEED_MIN;
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        /* 1. apply a pending resize: re-read terminal size, reset the clock */
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 2. measure this frame's elapsed time, clamped to avoid a spiral of
         *    death after a long stall (e.g. the process was suspended) */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* 3. advance the simulation in fixed TICK steps (accumulator pattern) */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

        /* 4. refresh the fps readout about twice a second */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. sleep off the remainder of the frame to cap at 60 fps */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* 6. render the frame and flush it to the terminal */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        /* 7. handle one input event (NULL when none is pending) */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
