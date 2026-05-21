/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * murmuration.c — 800-bird starling flock rendered as a density field
 *
 * DEMO: A high-count Reynolds flock (separation + alignment + cohesion)
 *       drifts across a toroidal terminal world.  Instead of drawing
 *       one glyph per bird, the renderer bins agents into a per-cell
 *       density grid and stamps a glyph from the ramp ".,:;oO*#@" —
 *       sparse cells fade to dots, dense cells glow as `@`, the
 *       flock reads as a moving black blob with internal density
 *       waves.  A single hawk orbits the world centre; press SPACE
 *       to send it diving through the flock at 250 px/s for ~1.5 s
 *       — the flock fragments around the hawk and reforms behind
 *       it.  Press 'h' to enable auto-dive every 6 seconds.
 *
 * Study alongside: flocking/flocking.c (the boid forces this builds on)
 *                  flocking/shepherd.c (Strömbom hawk/sheep model)
 *
 * Section map:
 *   §1 config   — counts, speeds, radii, weights, glyph ramp, hash dims
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 10 Theme palettes + PAIR_HAWK + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect bridge + Vec2 + toroidal_delta +
 *                 World struct + world_from_terminal
 *   §5 boid     — Boid struct, spawn, integrate, boid_clamp_speed,
 *                 boid_wrap
 *   §6 forces   — align_force, cohere_force, hawk_flee_force primitives
 *                 (with named-intermediate math)
 *   §6.5 hash   — SpatialHash struct + spatial_hash_build +
 *                 accumulate_pair_force + boid_forces (O(N·k) scan)
 *   §7 hawk     — Hawk struct + PATROL/HOVER/PURSUE/DIVE controller;
 *                 per-mode steppers (snap/integrate helpers) +
 *                 hawk_cycle_base_mode (n/p); world_centre +
 *                 hawk_patrol_radius_for shared shape helpers
 *   §8 scene    — SimControls + Scene (= pool + n_birds + hawk + sim +
 *                 world + flock_centroid + hash); scene_init (uses
 *                 spawn_boid_pool + place_hawk_at_orbit_east); scene_tick
 *                 (tick_auto_dive_timer + hawk_step + spatial_hash_build
 *                 + compute_new_velocities + commit_step_and_wrap +
 *                 scene_centroid); scene_draw (zero_density_region +
 *                 bin_boids_into_density + render_density_field +
 *                 stamp_hawk_glyph; tint_pair_for_cell + density_glyph)
 *   §9 app      — Screen, FpsCounter, App (= scene + screen + fps +
 *                 sim_fps + sig flags); signals; app_do_resize;
 *                 screen_draw (format_hud_status + hud_paint_text +
 *                 draw_hud_status / draw_hud_hint); app_handle_key
 *                 (toggle_auto_dive + cycle_theme + adjust_bird_count);
 *                 main game loop
 *
 * Keys:
 *   q / ESC    quit                       space    hawk DIVE (overlay)
 *   n / p      cycle hawk base mode       h        toggle auto-dive
 *              (PATROL → HOVER → PURSUE)           (every AUTO_DIVE_PERIOD s)
 *   . (period) pause / resume             r / R    reset (theme preserved)
 *   t / T      next / prev theme          + / -    add / remove 100 boids
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/murmuration.c \
 *       -o murmuration -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Reynolds boids (separation, alignment, cohesion) on a
 *                  toroidal grid, plus a flee force from a single hawk
 *                  predator.  All three boid forces and the flee force
 *                  are computed in ONE single-pass O(N²) loop per tick
 *                  — toroidal_delta gives the shortest signed
 *                  displacement on each axis so neighbours across the
 *                  screen edge are correctly seen as close.
 *
 *                  The renderer is what makes this visually distinct
 *                  from `flocking/flocking.c`.  Instead of one glyph
 *                  per bird (which at 800 birds in 80x24 = 1920 cells
 *                  would be a uniform soup), each frame:
 *                    1. zero a density[rows][cols] grid;
 *                    2. for each bird, increment density[cy][cx];
 *                    3. for each non-empty cell, pick a glyph from the
 *                       ASCII ramp ".,:;oO*#@" indexed by density and
 *                       a brightness from A_DIM/NORMAL/BOLD by tier.
 *                  The flock now reads as a 2D density field — exactly
 *                  the visual signature of real starling murmurations.
 *
 *                  HAWK has two states.  PATROL: orbit the world centre
 *                  at radius 0.40·min(w,h) with angular speed 0.4 rad/s.
 *                  DIVE (triggered by SPACE or auto-dive timer): set a
 *                  straight-line velocity toward the flock centroid,
 *                  step at 250 px/s for HAWK_DIVE_DURATION (~1.5 s),
 *                  then resume PATROL from the new position.
 *
 * Data-structure : Scene owns a fixed Boid pool[N_BOIDS_MAX], the
 *                  active count `n_birds`, the Hawk, the world
 *                  dimensions, and a pair of file-scope statics for
 *                  the per-frame density buffer (avoiding a per-frame
 *                  malloc and keeping the stack small).  Boids carry
 *                  pos / prev_pos / vel and a spawn-time colour pair
 *                  index used for HUD identity (the renderer ignores
 *                  per-bird colour because it draws the density field).
 *
 * Rendering      : Painter's order — density-mapped flock first, hawk
 *                  '!' last (always on top, A_BOLD red).  The density
 *                  ramp index is `min(density − 1, RAMP_LEN − 1)`; the
 *                  brightness tier is `A_DIM` for density 1-2,
 *                  `A_NORMAL` for 3-5, `A_BOLD` for 6+.  Sub-tick
 *                  alpha lerp is omitted on purpose — the visual is
 *                  the density gradient, which is itself stable across
 *                  alpha values, and skipping the per-bird interp
 *                  saves the 800 × 2 lerp ops per frame.
 *
 * Performance    : Uniform SPATIAL HASH (§6.5) drops the inner loop
 *                  from O(N²) to O(N·k), where k is the typical number
 *                  of birds in a 3×3 cell neighbourhood (cell size =
 *                  COHESION_RADIUS).  At default density on an 80×24
 *                  terminal that's k ≈ 30; the loop does ~N·k pair
 *                  tests per tick instead of N·(N-1).
 *
 *                    N      O(N²) pairs   O(N·k) pairs   speedup
 *                    ──     ───────────   ────────────   ───────
 *                    800       639 K          ~24 K       ~27×
 *                    1500     2 250 K         ~45 K       ~50×
 *
 *                  Hash build is O(N) (one prepend per bird).  Density
 *                  binning is O(N + cells) ≈ 800 + 1920 ≈ 2700 ops per
 *                  render — unchanged.  Net result: 1500-bird sims that
 *                  used to run at 2-8 fps now sit comfortably at 60.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Original behavioural-steering papers ───────────────────────
 *   [1] Reynolds, C. W. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 Computer Graphics
 *       21(4), pp. 25-34 — the ORIGINAL boids paper.  Introduces
 *       separation + alignment + cohesion; coins "boid" (bird-oid).
 *       This file's §6 force primitives are a direct implementation.
 *   [2] Reynolds, C. W. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference 1999.  Online at
 *       https://www.red3d.com/cwr/steer/ — canonical steering primer
 *       (seek, flee, arrive, pursue, evade, wander, separate, align,
 *       cohere).  The hawk's flee force in §6 and the future-extension
 *       PURSUE base mode are taken straight from this primer.
 *
 *   ── Statistical-physics flocking models ────────────────────────
 *   [3] Vicsek, T., Czirók, A., Ben-Jacob, E., Cohen, I. & Shochet, O.
 *       (1995), "Novel type of phase transition in a system of self-
 *       driven particles", Phys. Rev. Lett. 75(6), pp. 1226-1229 —
 *       the canonical Vicsek model.  Establishes flocking as a
 *       CONTINUOUS PHASE TRANSITION between order (low noise) and
 *       disorder (high noise) — same physics that lets a real
 *       starling murmuration switch between coherent and chaotic
 *       motion in seconds.
 *   [4] Toner, J. & Tu, Y. (1995, 1998), "Long-Range Order in a
 *       Two-Dimensional Dynamical XY Model" (PRL 75, pp. 4326-4329)
 *       + "Flocks, herds, and schools" (Phys. Rev. E 58, pp. 4828-
 *       4858) — hydrodynamic continuum theory of flocking; the
 *       macroscopic field theory behind the Vicsek model.  Read
 *       after [3] when you want to understand WHY the density
 *       waves visible in §8's renderer are smooth and propagating
 *       rather than chaotic.
 *
 *   ── Collective-behaviour biology ───────────────────────────────
 *   [5] Couzin, I. D., Krause, J., James, R., Ruxton, G. D. & Franks,
 *       N. R. (2002), "Collective memory and spatial sorting in
 *       animal groups", J. Theor. Biol. 218(1), pp. 1-11 — extends
 *       boids with explicit ZONES OF REPULSION / ORIENTATION /
 *       ATTRACTION.  Justifies SEPARATION_RADIUS < ALIGN_RADIUS <
 *       COHESION_RADIUS as the canonical ordering (this file uses
 *       14 / 50 / 80 px).
 *
 *   ── Empirical starling-murmuration studies ─────────────────────
 *   [6] Hildenbrandt, H., Carere, C. & Hemelrijk, C. K. (2010),
 *       "Self-organized aerial displays of thousands of starlings:
 *       a model", Behav. Ecol. 21(6), pp. 1349-1359 — the canonical
 *       QUANTITATIVE simulation model of starling murmurations.
 *       Adds aerodynamic banked turns + roost attraction; this file
 *       omits both but the flocking core matches.
 *   [7] Cavagna, A., Cimarelli, A., Giardina, I., Parisi, G.,
 *       Santagati, R., Stefanini, F. & Viale, M. (2010), "Scale-free
 *       correlations in starling flocks", PNAS 107(26), pp. 11865-
 *       11870 — the STARFLAG project's empirical analysis of real
 *       3-D starling flocks.  Shows real flocks are CRITICAL —
 *       behaviour correlations span the entire flock at all sizes,
 *       not just within a fixed neighbourhood radius.  Context for
 *       why the density-wave visuals in this demo emerge naturally.
 *
 *   ── Implementation-oriented references ─────────────────────────
 *   [8] Shiffman, D., "The Nature of Code", Ch. 6 — reading-level
 *       walkthrough of Reynolds steering with Processing examples.
 *       The single most accessible introduction; pair with [2].
 *       https://natureofcode.com/book/chapter-6-autonomous-agents/
 *
 *   ── Game-loop / fixed-step physics ─────────────────────────────
 *   [9] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       fixed-step accumulator + cap-dt pattern implemented in §9
 *       main().  This file does NOT lerp between physics snapshots
 *       in the renderer (density-field is stable across alpha) but
 *       the dt-cap + accumulator logic is straight from this article.
 *
 *   ── Large-N optimisation: spatial hashing ──────────────────────
 *  [10] Teschner, M., Heidelberger, B., Müller, M., Pomerantes, D. &
 *       Gross, M. (2003), "Optimized spatial hashing for collision
 *       detection of deformable objects", Vision, Modeling, and
 *       Visualization 2003, pp. 47-54 — the standard reference for
 *       uniform spatial hashing in particle / agent systems.
 *       §6.5 implements this for boid neighbour search: cell size =
 *       max neighbour radius, scan the 3×3 surround.  Drops the
 *       inner loop from O(N²) to O(N·k); critical for the 1500-bird
 *       configuration.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [11] Wikipedia: "Boids", "Murmuration", "Vicsek model" —
 *       https://en.wikipedia.org/wiki/Boids
 *       https://en.wikipedia.org/wiki/Murmuration
 *       https://en.wikipedia.org/wiki/Vicsek_model
 *       concise summaries of the rules, the biological phenomenon
 *       this demo reproduces, and the phase-transition model.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     flocking/flocking.c — same Reynolds rules with FIVE switchable
 *       modes (boids / chase / Vicsek / orbit / predator-prey).  Read
 *       FIRST if you want the per-rule story before the high-N
 *       density-field rendering here.
 *     flocking/crowd.c    — single-flock crowd with six behaviour
 *       modes (WANDER / FLOCK / PANIC / GATHER / FOLLOW / QUEUE).
 *       Shows how the same primitives compose into different
 *       emergent behaviours at MEDIUM N (~150 agents).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Stop drawing birds.  Draw the LOCAL DENSITY of birds.  When 800
 * agents are clustered into ~1900 terminal cells, each cell almost
 * always has 0, 1, or 2 birds — except the heart of the flock,
 * which has 5-15.  Map count → ASCII glyph from a ramp going
 * sparse-to-dense, and the flock automatically reads as a moving
 * black cloud with a bright core, internal density waves, and
 * a clear silhouette — which is exactly how real starling
 * murmurations look at human visual scale.  The flocking forces
 * just keep the cloud cohesive; the ramp is what makes it pretty.
 *
 *
 * ALGORITHM IN STEPS  (per tick)
 * ──────────────────────────────
 *   1. Hawk: tick controller.  Pick stepper based on .mode —
 *      PATROL  → advance phase along the orbit ring.
 *      HOVER   → drift toward flock centroid; settle inside hold radius.
 *      PURSUE  → track flock centroid continuously at PURSUE_SPEED.
 *      DIVE    → integrate forward at DIVE_SPEED until dive_timer
 *                expires, then snap back to .base_mode (n/p selects).
 *   2. Hash build: O(N) — clear bucket heads, then prepend every
 *      bird to its cell's singly-linked list.  Cell size =
 *      COHESION_RADIUS so a 3×3 cell neighbourhood is sufficient.
 *   3. Boids: read OLD positions, hash-narrowed neighbour scan.
 *      For each bird, walk its own + 8 adjacent buckets; for each
 *      pair (self, neighbour) compute toroidal Δ; classify into
 *      separation / alignment / cohesion accumulators.  After the
 *      inner loop, add a flee force from the hawk (once per bird).
 *   4. Boids: write phase.  Apply force·dt to vel, clamp |vel|
 *      to [MIN_SPEED, MAX_SPEED], integrate pos, wrap toroidally.
 *   5. Render: zero density[rows][cols]; bin every bird into the
 *      grid via px_to_cell; for each cell with density > 0 stamp
 *      the corresponding glyph + brightness; over-stamp the hawk.
 *
 * KEY FORMULAS
 * ────────────
 *   Toroidal delta (shortest signed displacement on a wrapped axis):
 *     d = b − a
 *     if d >  extent/2 : d −= extent
 *     if d < −extent/2 : d += extent
 *
 *   Boid forces (per neighbour with squared toroidal distance d²):
 *     if d² < SEP_R²  : sep_force  += unit(self−nb) · (SEP_R − d)/SEP_R · BOID_SPEED
 *     if d² < ALIGN_R²: ali_vsum   += nb.vel; ali_n++
 *     if d² < COH_R²  : coh_dsum   += toroidal_delta(self, nb)  (offset, NOT pos)
 *                       coh_n++
 *
 *   After loop:
 *     ali_force  = (ali_n>0) ? (ali_vsum/ali_n − vel)             : 0
 *     coh_force  = (coh_n>0) ? unit(coh_dsum/coh_n) · BOID_SPEED  : 0
 *     hawk_flee  = if dist(self, hawk) < HAWK_FLEE_RADIUS:
 *                    unit(self − hawk) · (HAWK_FLEE_R − d)/HAWK_FLEE_R · FLEE_SPEED
 *                  else 0
 *
 *   Steering sum:
 *     force = W_SEP·sep + W_ALIGN·ali + W_COHERE·coh + W_FLEE·hawk_flee
 *
 *   Density-to-glyph mapping (renderer):
 *     idx   = min(density − 1, RAMP_LEN − 1)
 *     glyph = RAMP[idx]                     ".,:;oO*#@"
 *     attr  = density ≥ 5 ? A_BOLD : A_NORMAL
 *
 *     (No A_DIM tier — sparse cells make up most of the flock area
 *     and A_DIM would mute them to near-invisible, defeating the
 *     "tinted cloud" look.  The fade-to-edge effect comes from the
 *     glyph ramp '.'→'@', not from brightness modulation.)
 *
 *
 * Background you need
 * ───────────────────
 *   - flocking.c T1-T7 (boid + three rules + toroidal + 2-stage).
 *   - Histogram / binning — counting items into discrete buckets.
 *   - Uniform spatial hash — bucket agents by floor(pos / cell_size),
 *     query the 3×3 neighbourhood.  Standard large-N optimisation;
 *     this file uses it in §6.5 to scale to 1500 birds at 60 fps.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - kd-trees / BVH / R-trees.  Uniform hash is enough when agents
 *     are roughly uniformly distributed; flocking density doesn't
 *     fluctuate enough to need adaptive structures.
 *   - Voxel splatting / volume rendering. The density grid is 2-D,
 *     ASCII output.
 *   - Per-bird identity. The renderer doesn't track WHICH boid is
 *     in which cell — only HOW MANY.
 *
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT = 60,
    TARGET_FPS      = 60,
    HUD_COLS        = 96,
    FPS_UPDATE_MS   = 500,

    /*
     * Boid pool sizing.
     *
     * N_BOIDS_DEFAULT = 800: enough density that the binned grid
     *   shows clear `@`-core + falloff to `.` periphery on an 80x24
     *   terminal (1920 cells), without breaking sub-millisecond tick
     *   budget under O(N²) steering.
     *
     * N_BOIDS_MAX     = 1500: hard ceiling.  At 1500 the inner loop
     *   does 2.25 M pair tests/tick = 135 M/s — still well under the
     *   16 ms frame budget at 60 Hz on any modern CPU.
     *
     * BOID_STEP       = 100: chunk added/removed per `+`/`-` keypress.
     */
    N_BOIDS_DEFAULT = 800,
    N_BOIDS_MAX     = 1500,
    N_BOIDS_MIN     = 100,
    BOID_STEP       = 100,

    /*
     * Colour pair IDs (see §3 for actual colour values).
     *   Pairs 1-7 cycle the flock tint through the active theme.
     *   PAIR_HAWK is fixed (red, A_BOLD) regardless of theme so the
     *   predator always reads as the predator.
     *   PAIR_HUD/PAIR_HINT follow the project HUD spec.
     */
    N_COLORS        =   7,
    PAIR_HAWK       =   8,
    PAIR_HUD        =   9,
    PAIR_HINT       =  10,

    N_THEMES        =  10,

    /*
     * Density-grid hard cap — sized for very large terminals so
     * resize never overflows the buffer.  At MAX_ROWS=80, MAX_COLS=256
     * this is 80 × 256 × sizeof(int) = 80 KB of static BSS.
     */
    MAX_ROWS        =  80,
    MAX_COLS        = 256,

    /*
     * Spatial-hash grid cap.
     *
     * Cell size equals the LARGEST neighbour radius (COHESION_RADIUS = 80 px),
     * so a 3×3 cell neighbourhood is sufficient to find every neighbour
     * within COHESION_RADIUS of self regardless of where self sits in its
     * cell.  Grid dims grow with world size; values below cap any
     * reasonable terminal:
     *
     *   max world px = MAX_COLS·CELL_W × MAX_ROWS·CELL_H = 2048 × 1280
     *   max hash dims = ceil(2048/80) × ceil(1280/80) = 26 × 16 → fits 32×24.
     *
     * Storage: 32·24·sizeof(int) + N_BOIDS_MAX·sizeof(int) ≈ 3 KB + 6 KB
     *          = 9 KB static.  Cheap.
     */
    HASH_CELL_PX     = 80,         /* matches COHESION_RADIUS              */
    HASH_GRID_W_MAX  = 32,         /* covers worlds up to 2560 px wide     */
    HASH_GRID_H_MAX  = 24,         /* covers worlds up to 1920 px tall     */
};

/* Cell dimensions — physics in pixel space, draw in cells. */
#define CELL_W   8
#define CELL_H  16

/* ── boid speeds (px/s) ─────────────────────────────────────────────── */
#define BOID_SPEED       90.0f   /* cruise speed used by force formulas    */
#define BOID_MAX_SPEED  150.0f   /* hard cap on |vel| each tick            */
#define BOID_MIN_SPEED   30.0f   /* floor — boids never stall completely   */
#define FLEE_SPEED      180.0f   /* effective speed scaling on hawk flee   */

/* ── boid radii (px) ────────────────────────────────────────────────── */
#define SEP_RADIUS       14.0f   /* personal-space bubble                  */
#define ALIGN_RADIUS     50.0f   /* heading-match neighbourhood            */
#define COHESION_RADIUS  80.0f   /* centre-of-mass neighbourhood           */
#define HAWK_FLEE_RADIUS 110.0f  /* boids flee within this disc of hawk    */

/* ── boid force weights ─────────────────────────────────────────────── *
 *  Order of magnitude reflects priority:
 *    FLEE   (4.5) overrides everything when hawk is close.
 *    SEP    (1.8) firm enough to stop overlap during panic.
 *    ALIGN  (1.0) nominal — produces the coherent direction of travel.
 *    COHERE (0.7) softer — flock stays loose, doesn't collapse to a point.
 *    MAX_STEER caps the per-tick acceleration so smooth curves emerge
 *    rather than instant pivots even when forces stack up.            */
#define W_SEP      1.8f
#define W_ALIGN    1.0f
#define W_COHERE   0.7f
#define W_FLEE     4.5f
#define MAX_STEER 130.0f

/* ── hawk parameters ────────────────────────────────────────────────── *
 *
 *  Hawk has THREE base modes (selected with n / p cycling) and ONE
 *  overlay (DIVE, fired by SPACE on top of any base mode):
 *
 *    PATROL — orbit world centre at HAWK_PATROL_FRAC · min(w,h) at
 *             HAWK_PATROL_OMEGA rad/s.  Hawk circles, ignored by the
 *             flock except when it passes through.
 *    HOVER  — slow drift toward the flock centroid; settle once close.
 *             Creates a persistent "no-fly zone" the flock has to
 *             route around — flock self-organises into a moving torus.
 *    PURSUE — continuous tracking at HAWK_PURSUE_SPEED (just above
 *             BOID_SPEED so the hawk slowly closes).  Flock keeps
 *             running; pressure is constant rather than burst-like.
 *    DIVE   — overlay: straight-line sprint at HAWK_DIVE_SPEED toward
 *             the flock centroid for HAWK_DIVE_DURATION seconds, then
 *             return to whichever base mode is currently selected.
 *
 *  Speed ordering (px/s):
 *    BOID_SPEED 90  <  HAWK_HOVER_DRIFT 60  ≪ ...
 *    BOID_MAX_SPEED 150  >  HAWK_PURSUE_SPEED 110
 *    HAWK_DIVE_SPEED 250  ≫ everything (burst attack)
 *
 *  AUTO_DIVE_PERIOD — when auto-dive is on (`h` key), the next dive
 *  fires this many seconds after the previous one ended.            */
#define HAWK_PATROL_FRAC      0.40f
#define HAWK_PATROL_OMEGA     0.4f
#define HAWK_DIVE_SPEED     250.0f
#define HAWK_DIVE_DURATION    1.5f
#define AUTO_DIVE_PERIOD      6.0f

/* HOVER: drift toward centroid at this slow speed; settle once within
 * HAWK_HOVER_HOLD_RADIUS of centroid (vel scales linearly to 0 inside). */
#define HAWK_HOVER_DRIFT        60.0f
#define HAWK_HOVER_HOLD_RADIUS  30.0f

/* PURSUE: continuous chase speed.  Slightly faster than BOID_SPEED so
 * the hawk slowly catches up; flock keeps running and reshapes. */
#define HAWK_PURSUE_SPEED  110.0f

/* Glyph ramp for density-to-character mapping.  Index 0 = density 1
 * (sparsest), index RAMP_LEN-1 = density ≥ RAMP_LEN (densest core).
 * Pure ASCII per CLAUDE.md "ASCII-Only Rendering" — no Unicode blocks. */
static const char DENSITY_RAMP[] = ".,:;oO*#@";
#define RAMP_LEN ((int)(sizeof DENSITY_RAMP - 1))

/* Timing primitives (verbatim from framework.c). */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/* clock_ns — monotonic wall-clock in ns; never goes backward. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* clock_sleep_ns — sleep ns nanoseconds; ns ≤ 0 returns immediately. */
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
/* §3  color — 10 themes for the flock + theme-independent hawk + HUD    */
/* ===================================================================== */

/*
 * Theme — one named flock palette.
 *
 * Intent
 *   Holds the seven xterm-256 colour indices that compose ONE flock
 *   tint.  The renderer cycles through pairs 1..N_COLORS keyed by
 *   spatial hash (`(cy*7 + cx) % N_COLORS`), so adjacent cells get
 *   different pair indices and the flock reads as a cloud with a
 *   subtle interior mottle.
 *
 * Why a TIGHT CLUSTER of bright shades (not a dim-to-bright gradient)
 *   The renderer cycles through all 7 pairs spatially.  If the
 *   palette spans dim → bright, roughly half the flock cells render
 *   dim and the cloud reads as muddy.  Keeping all 7 entries in the
 *   same brightness band gives a clean tinted cloud with a subtle
 *   mottle from the spatial hash — no dim spots, no banding.
 *
 * Brightness floor
 *   All entries sit at ≥ 81 in the xterm-256 cube (or ≥ 119 in
 *   mint/peach band, or use full-saturation primaries 46/196/220/226).
 *   No values below 80 — at A_DIM they'd vanish.  We removed the
 *   A_DIM tier in density_glyph but the floor is a safety margin in
 *   case it's ever re-enabled (per CLAUDE.md palette-brightness rule).
 *
 * Members
 *   name     HUD label shown to the user (e.g. "Dusk", "Sky", "Matrix").
 *            Pointer to a static C string literal.  Read by screen_draw.
 *   body[7]  Seven xterm-256 colour indices.  Bird i renders with
 *            pair (i % N_COLORS) + 1, so each theme entry is "the
 *            colour for the i-th rotation slot".  All seven SHOULD be
 *            shades of the same tint; see "tight cluster" above.
 *
 * Invariants
 *   name != NULL.
 *   Every body[k] ∈ [0, 255] (xterm-256 colour index range).
 *   For visual quality: body[k] ≥ 80 (brightness floor).
 *
 * References
 *   None directly — the colour palette is project-specific design.
 *   xterm-256 cube layout: see CLAUDE.md §"Theme Palette Brightness".
 */
typedef struct {
    const char *name;
    int         body[N_COLORS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        bright cluster — all 7 entries are similar shades */
    {"Dusk",    { 180, 187, 188, 222, 223, 224, 230 }},  /* warm cream    */
    {"Sky",     {  81,  87, 111, 117, 123, 153, 159 }},  /* sky-blue      */
    {"Solar",   { 214, 220, 221, 222, 226, 227, 228 }},  /* warm yellow   */
    {"Aurora",  { 121, 122, 158, 159, 195, 207, 213 }},  /* mint → pink   */
    {"Ember",   { 196, 202, 208, 209, 214, 215, 220 }},  /* fire ramp     */
    {"Forest",  { 119, 120, 121, 154, 155, 156, 157 }},  /* leafy green   */
    {"Neon",    { 165, 171, 201, 207, 213, 219, 225 }},  /* hot magenta   */
    {"Sunset",  { 209, 210, 215, 216, 221, 222, 223 }},  /* warm orange   */
    {"Ghost",   { 247, 250, 252, 253, 254, 255, 231 }},  /* near-white    */
    {"Matrix",  {  46,  82, 118, 119, 120, 121, 156 }},  /* matrix green  */
};

/*
 * theme_apply — register the body palette with ncurses.  Background
 * = -1 (terminal default) per CLAUDE.md HUD spec.  Hawk and HUD pairs
 * are theme-independent and configured once in color_init().
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_COLORS] = {
            COLOR_WHITE, COLOR_WHITE, COLOR_CYAN,
            COLOR_CYAN,  COLOR_BLUE,  COLOR_BLUE,  COLOR_MAGENTA
        };
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/*
 * color_init — one-time colour setup.
 *   start_color()        — initialise ncurses colour support.
 *   use_default_colors() — allow background = -1 (terminal default).
 *   theme_apply()        — register the initial flock palette.
 *   PAIR_HAWK            — bright red 196, A_BOLD on draw.
 *   PAIR_HUD             — bright yellow 226, A_BOLD on draw.
 *   PAIR_HINT            — bright cyan   51, A_BOLD on draw.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HAWK, 196, -1);
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HAWK, COLOR_RED,    -1);
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell + Vec2 + toroidal_delta                       */
/* ===================================================================== */

/* Pixel-space world dimensions from terminal cell counts. */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * World — pixel-space extent of the simulation box.
 *
 * Intent
 *   Every physics routine needs "the size of the world" for two things:
 *
 *     1. Toroidal wrap (boid_wrap, hawk_wrap):
 *          if px <  0      → px += width
 *          if px >= width  → px -= width
 *        A bird leaving one edge re-enters at the opposite edge.
 *
 *     2. Shortest-path distance (toroidal_delta):
 *          d = b − a
 *          if |d| > extent/2 : take the wrap path instead
 *        Used by every steering force so neighbours across the wrap
 *        are correctly seen as close.
 *
 *   Before this struct, every signature carried a separate (ww, wh)
 *   pair — 10+ functions, all threading the same two floats.  Bundling
 *   them into one named type makes signatures read in the physics
 *   vocabulary: `boid_wrap(b, world)` instead of `boid_wrap(b, ww, wh)`.
 *
 * Members
 *   width   x-extent in pixels = cols × CELL_W (CELL_W = 8).
 *   height  y-extent in pixels = rows × CELL_H (CELL_H = 16, double
 *           CELL_W to compensate for the terminal cell aspect ratio).
 *
 * Invariants
 *   width > 0 AND height > 0 (set by world_from_terminal).
 *   width  == cols × CELL_W,  height == rows × CELL_H, kept in sync
 *   with Screen.cols / Screen.rows by scene_init / app_do_resize.
 */
typedef struct {
    float width;
    float height;
} World;

/*
 * world_from_terminal — derive the pixel-space World from a terminal
 * (cols, rows) reading.  Called at scene_init and again after every
 * SIGWINCH so the physics box matches the visible terminal extent.
 */
static inline World world_from_terminal(int cols, int rows)
{
    return (World){ .width = pw(cols), .height = ph(rows) };
}

/*
 * px_to_cell_x/y — round to nearest cell.  "Round half up" via
 * floor(p/dim + 0.5) avoids the half-pixel oscillation roundf()
 * would produce at exact boundaries.
 */
static inline int px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/*
 * Vec2 — 2-D float vector used everywhere physics or geometry shows up.
 *
 * Intent
 *   Position, velocity, acceleration, force, offset, centroid — every
 *   quantity with an x and a y is a Vec2.  Functions return Vec2 BY
 *   VALUE (no out-params); the v2* helpers below compose into terse
 *   chains that read like the math notation they implement:
 *
 *      Vec2 force = v2add(v2scale(sep, W_SEP), v2scale(ali, W_ALIGN));
 *
 *   reads as `force = W_SEP · sep + W_ALIGN · ali` to a maths-fluent
 *   reader.
 *
 * Why a struct (not float[2] or two scalars)
 *   • Named type makes signatures read in the algorithm's vocabulary:
 *     `boid_forces(pool, hash, self, hawk_pos, world)` rather than
 *     `boid_forces(pool, hash, self, hawk_x, hawk_y, ww, wh)`.
 *   • Return by value works without spilling to memory on x86-64 +
 *     ARM ABIs (8 bytes = one register pair); no perf penalty.
 *   • Eliminates two-out-parameter idioms in every steering helper.
 *
 * Members
 *   x, y    Cartesian components in PIXEL space (units = px).  Cell-
 *           space conversion happens only inside scene_draw via
 *           px_to_cell_x/y.  Physics never sees cells.
 *
 * References
 *   [2] Reynolds 1999 — every steering primitive returns a Vec2
 *       force; this file's §6 mirrors that API shape.
 *   [8] Shiffman, *Nature of Code* Ch. 1 — PVector pedagogy that
 *       this type is the C analogue of.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }
static inline float v2len2(Vec2 v)               { return v.x*v.x + v.y*v.y; }

/* v2norm — unit vector; returns zero if input is near-zero. */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/* v2clamp_len — cap |v| at max_len, preserve direction. */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/*
 * toroidal_delta — shortest signed displacement from a to b on a
 * single axis of length `extent`.  On a wrap-around axis there are
 * two paths between any two points; we always return the shorter
 * one with sign indicating direction.
 *
 * Example (extent=100): a=5, b=95 → direct=+90, wrap=-10 → returns -10.
 * Used by every steering force so neighbours across the wrap are
 * correctly seen as close — without this, the flock would tear at edges.
 */
static inline float toroidal_delta(float a, float b, float extent)
{
    float d = b - a;
    if (d >  extent * 0.5f) d -= extent;
    if (d < -extent * 0.5f) d += extent;
    return d;
}

static inline float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ===================================================================== */
/* §5  boid                                                               */
/* ===================================================================== */

/*
 * Boid — one bird's complete kinematic state (Reynolds [1]: "bird-oid").
 *
 * Intent
 *   The atomic unit of the murmuration.  A Boid is a point particle
 *   on a torus with position + velocity; steering forces accumulate
 *   into vel each tick, vel integrates into pos.  At N=1500 the file
 *   holds 1500 of these in Scene.pool[].
 *
 * Why these fields and no more
 *   • No mass — every bird is implicitly mass 1, so force·dt feeds
 *     directly into vel.  Reynolds [2] uses the same simplification:
 *     weights become tunable scalars rather than per-mass forces.
 *   • No personal cruise_speed — unlike flocking.c / crowd.c, this
 *     file uses one global BOID_SPEED for every agent because the
 *     density-field renderer hides per-agent variation anyway.  The
 *     visual variety comes from the density gradient, not from
 *     individual depth.
 *   • No heading angle — heading is implicit in atan2(vel.y, vel.x);
 *     never stored separately.  Eliminates desync risk between an
 *     angle and velocity components.
 *
 * Why prev_pos despite no alpha-lerp
 *   prev_pos is snapshotted each tick out of habit / future-proofing:
 *   the renderer (§8 scene_draw) does NOT currently lerp between
 *   prev_pos and pos (density rendering is stable across alpha, and
 *   skipping the 1500×2 lerp ops/frame saves real time).  prev_pos
 *   is kept so a future per-bird rendering mode could use it without
 *   a schema change.
 *
 * Why PIXEL space
 *   Storing px/py in cell coordinates would lose sub-cell motion
 *   (CELL_W=8 means a 90 px/s bird advances 0.19 cell/tick at 60 Hz;
 *   in cell space that becomes "stay in cell A for 5 ticks, jump to
 *   B" — visible stair-stepping).  Float pixel space → smooth motion.
 *
 * Members
 *   pos          current position in PIXEL space (units = px).
 *                INVARIANT: 0 ≤ x < World.width AND 0 ≤ y < World.height
 *                after every boid_wrap() call (torus wraps strays back).
 *                Density binning recovers (col, row) via px_to_cell_x/y.
 *
 *   prev_pos     position at start of this tick.  Snapshotted in
 *                commit_step_and_wrap (scene_tick stage 2) BEFORE pos
 *                is integrated.  Kept for future renderer extensions;
 *                current density renderer ignores it.
 *
 *   vel          velocity in PIXELS PER SECOND.  Integration rule:
 *                  vel += steer·dt   (steering accumulator,  §6 forces)
 *                  pos += vel ·dt   (Euler integration,     §8 commit_step_and_wrap)
 *                Magnitude clamped to [BOID_MIN_SPEED, BOID_MAX_SPEED]
 *                each tick by boid_clamp_speed (floor prevents stall
 *                when forces cancel; ceiling prevents flee-runaway).
 *
 *   color_pair   ncurses colour pair index, set once at spawn from
 *                `(id % N_COLORS) + 1`.  COSMETIC ONLY — the density
 *                renderer derives per-cell colour from spatial hash,
 *                not from per-bird color_pair.  Kept on the struct so
 *                a future per-bird draw mode (e.g. trail rendering)
 *                could use it.
 *
 * Invariants
 *   0 ≤ pos.x < World.width  AND  0 ≤ pos.y < World.height after wrap.
 *   |vel| ∈ [BOID_MIN_SPEED, BOID_MAX_SPEED] after boid_clamp_speed.
 *   color_pair ∈ [1, N_COLORS]; set once at spawn, never mutated.
 *
 * References
 *   [1] Reynolds 1987 — the original boid: point particle that
 *       accumulates steering forces.
 *   [2] Reynolds 1999 §"Vehicle Model" — same kinematic model:
 *       force → velocity → position with magnitude caps.
 *   [8] Shiffman *Nature of Code* Ch. 6 — same Vehicle layout in
 *       Processing pseudocode; great companion read.
 */
typedef struct {
    /* kinematic state — written by physics every tick */
    Vec2 pos;
    Vec2 prev_pos;     /* kept for future renderer extensions; unused now */
    Vec2 vel;

    /* identity — written once at spawn, read by NO active code path
     * (density renderer uses spatial hash instead).  Cosmetic. */
    int  color_pair;
} Boid;

/*
 * boid_spawn — random pos in world, random direction at BOID_SPEED·0.5
 * so the flock isn't frozen on frame one.
 */
static void boid_spawn(Boid *b, int id, World world)
{
    b->pos      = v2(randf() * world.width, randf() * world.height);
    b->prev_pos = b->pos;

    float ang = randf() * 2.0f * (float)M_PI;
    b->vel    = v2(cosf(ang) * BOID_SPEED * 0.5f,
                   sinf(ang) * BOID_SPEED * 0.5f);

    b->color_pair = (id % N_COLORS) + 1;
}

/*
 * boid_clamp_speed — enforce MIN_SPEED ≤ |vel| ≤ MAX_SPEED.
 *
 * Without a floor, boids stall when sep + ali + coh roughly cancel.
 * Without a ceiling, the flee force during a hawk dive can blow up
 * |vel| past anything visually useful.  Both clamps are needed.
 */
static void boid_clamp_speed(Boid *b)
{
    const float STALL_EPSILON = 0.001f;     /* below this = no direction left */
    float current_speed = v2len(b->vel);

    /* (a) full stall: forces cancelled to ~0; kick forward in a random
     *     direction so the bird never freezes in place */
    if (current_speed < STALL_EPSILON) {
        float random_heading_rad = randf() * 2.0f * (float)M_PI;
        b->vel = v2(cosf(random_heading_rad) * BOID_MIN_SPEED,
                    sinf(random_heading_rad) * BOID_MIN_SPEED);
        return;
    }

    /* (b) below floor: rescale to MIN_SPEED preserving direction */
    if (current_speed < BOID_MIN_SPEED) {
        b->vel = v2scale(v2norm(b->vel), BOID_MIN_SPEED);
        return;
    }

    /* (c) above ceiling: rescale to MAX_SPEED preserving direction */
    if (current_speed > BOID_MAX_SPEED) {
        b->vel = v2scale(v2norm(b->vel), BOID_MAX_SPEED);
    }
}

/*
 * boid_wrap — toroidal boundary.  A bird leaving one edge re-enters
 * at the opposite edge.  Uniform flocking everywhere — no "edge of
 * the world" where birds bunch up.
 */
static void boid_wrap(Boid *b, World world)
{
    if (b->pos.x <  0.0f)        b->pos.x += world.width;
    if (b->pos.x >= world.width) b->pos.x -= world.width;
    if (b->pos.y <  0.0f)        b->pos.y += world.height;
    if (b->pos.y >= world.height)b->pos.y -= world.height;
}

/* ===================================================================== */
/* §6  forces — separation + alignment + cohesion + hawk flee primitives */
/* ===================================================================== */
/* These are SHAPE helpers: take accumulators (built by §6.5 over a 3×3
 * cell neighbourhood) and return a Reynolds-style steering force.  The
 * actual neighbour scan and accumulator construction lives in §6.5
 * (spatial hash + accumulate_pair_force + boid_forces).               */

/*
 * align_force — convert alignment accumulator (sum of neighbour
 * velocities + count) into a Reynolds-style "steer toward mean
 * heading" force.  Returns zero if no neighbours were in range.
 */
static Vec2 align_force(Vec2 ali_vsum, int ali_n, Vec2 my_vel)
{
    if (ali_n == 0) return v2(0, 0);

    /* mean_neighbour_velocity = Σ neighbour.vel / count
     * force = (desired − own_velocity), where desired = mean heading
     * Reynolds rule 2: steer toward the average heading of the group */
    Vec2 mean_neighbour_velocity = v2scale(ali_vsum, 1.0f / (float)ali_n);
    return v2sub(mean_neighbour_velocity, my_vel);
}

/*
 * cohere_force — convert cohesion accumulator (sum of toroidal
 * offsets toward neighbours + count) into a "seek mean position"
 * force.  We accumulate OFFSETS, not absolute positions, because
 * naïve averaging across the toroidal wrap would put the centroid
 * in the middle of the screen even when birds are clustered at
 * opposite edges.
 */
static Vec2 cohere_force(Vec2 coh_dsum, int coh_n, Vec2 my_vel)
{
    if (coh_n == 0) return v2(0, 0);

    /* mean_offset_to_centre_of_mass = Σ (neighbour_pos − self_pos) / n
     *   (OFFSETS, not absolute positions — see cohere_force docblock)
     * toward_centre_dir = unit vector pointing at the group centroid
     * desired_velocity  = toward_centre direction at cruise speed
     * force             = desired − own_velocity (Reynolds rule 3) */
    Vec2 mean_offset_to_centre_of_mass = v2scale(coh_dsum, 1.0f / (float)coh_n);
    Vec2 toward_centre_dir             = v2norm(mean_offset_to_centre_of_mass);
    Vec2 desired_velocity              = v2scale(toward_centre_dir, BOID_SPEED);
    return v2sub(desired_velocity, my_vel);
}

/*
 * hawk_flee_force — repulsion from the hawk with linear falloff
 * inside HAWK_FLEE_RADIUS, zero outside.  Magnitude scales toward
 * FLEE_SPEED at the wall (d → 0).  Uses toroidal_delta so a hawk
 * across the wrap still scares its neighbours.
 */
static Vec2 hawk_flee_force(Vec2 me_pos, Vec2 hawk_pos, World world)
{
    const float DIST_EPSILON_SQ = 1e-6f;
    const float HAWK_FLEE_RADIUS_SQ = HAWK_FLEE_RADIUS * HAWK_FLEE_RADIUS;

    /* offset_to_hawk = self → hawk (toroidal shortest-path)
     * squared_distance compared first to skip the sqrt when hawk is
     * out of range (very common — most birds aren't near the hawk) */
    float offset_x          = toroidal_delta(me_pos.x, hawk_pos.x, world.width);
    float offset_y          = toroidal_delta(me_pos.y, hawk_pos.y, world.height);
    float squared_distance  = offset_x*offset_x + offset_y*offset_y;

    bool inside_panic_zone = squared_distance < HAWK_FLEE_RADIUS_SQ;
    bool well_separated    = squared_distance > DIST_EPSILON_SQ;
    if (!inside_panic_zone || !well_separated) return v2(0, 0);

    /* distance_to_hawk          : pull out sqrt now that we need it
     * panic_intensity ∈ (0, 1]  : closer to hawk ⇒ stronger flee
     * away_from_hawk_dir        : unit vector pointing AWAY from hawk
     *                             (negate offset which is self→hawk)
     * force = away_dir · panic_intensity · FLEE_SPEED */
    float distance_to_hawk = sqrtf(squared_distance);
    float panic_intensity  = (HAWK_FLEE_RADIUS - distance_to_hawk) / HAWK_FLEE_RADIUS;
    Vec2  away_from_hawk_dir = v2(-offset_x / distance_to_hawk,
                                  -offset_y / distance_to_hawk);
    return v2scale(away_from_hawk_dir, panic_intensity * FLEE_SPEED);
}

/* ── §6.5 spatial hash — O(N·k) neighbour search ────────────────────── *
 *
 * At large N (e.g. 1500 birds) the O(N²) inner loop dominates: 2.25 M
 * pair tests per tick × 60 Hz ≈ 135 M/s, which can drop visible fps to
 * single digits.  A uniform spatial hash brings the work down to
 * O(N · k) where k is the typical number of birds in a 3×3 cell
 * neighbourhood.  At default density that's k ≈ 20-40, giving roughly
 * an order of magnitude speedup at N=1500.
 *
 * Data structure (singly-linked-list buckets, held by Scene.hash):
 *
 *      hash->head[gy][gx]   : index of FIRST bird in cell, or −1
 *      hash->next[bird]     : index of NEXT bird in same cell, or −1
 *
 *      cell 5 occupants: head[…] = 12 ─► next[12] = 7 ─► next[7] = 3
 *                                                      ─► next[3] = −1
 *
 * Build: clear heads, then prepend each bird to its cell's list.  O(N).
 *
 * Query: for each bird, walk the 3×3 cells centred on its own cell
 * (toroidal-wrapped) and visit only the birds in those buckets.
 *
 * Cell size MUST be ≥ COHESION_RADIUS (the largest neighbour radius).
 * Smaller cells = more cells visited; larger cells = more birds per
 * cell ≈ same total work.  Matching exactly is the sweet spot.
 *
 * Small-world fix: if the world only fits 1 or 2 hash cells along an
 * axis, the toroidal-wrap of {-1, 0, +1} visits some cells twice and
 * double-counts.  In that case we sweep ALL cells along that axis
 * once each (no wrap), which is equivalent in correctness and is
 * still O(N) overall since the dimension itself is small.
 *
 * ────────────────────────────────────────────────────────────────── */

/*
 * SpatialHash — bucketed linked-list neighbour index, rebuilt every tick.
 *
 * Intent
 *   The data half of the O(N·k) neighbour search.  At N=1500 the
 *   O(N²) pair loop runs at ~5 fps; with this hash, ~60 fps.  Lives
 *   ON Scene so it travels with the simulation state rather than as
 *   loose file-scope globals.
 *
 * Why on Scene (not file-scope statics)
 *   Two reasons.  (1) Owning structure: every steering call already
 *   takes Scene; reading the hash through `s->hash` keeps the data
 *   close to its only user.  (2) Multiple Scenes: a future split-
 *   screen demo or recorded-replay viewer would need independent
 *   hashes — file-scope statics make that impossible.
 *
 * Members
 *   head[gy][gx]   index of the FIRST bird in this hash cell, or −1
 *                  if empty.  Singly-linked-list head pointer.
 *   next[i]        index of the NEXT bird in the same cell as i, or
 *                  −1 if i is the tail.  next[] is keyed by BIRD index
 *                  (parallel to Scene.pool), NOT by cell index.
 *   grid_w, grid_h Active grid dimensions for THIS tick (depend on
 *                  world extent).  ≤ HASH_GRID_W_MAX × HASH_GRID_H_MAX.
 *
 * Lifecycle
 *   spatial_hash_build()       — called once per tick, before boid_forces.
 *   boid_forces()              — reads head/next walks for the 3×3
 *                                neighbourhood of each bird.
 *   No teardown — the buffer is rebuilt from scratch every tick.
 *
 * Invariants
 *   1 ≤ grid_w ≤ HASH_GRID_W_MAX,  1 ≤ grid_h ≤ HASH_GRID_H_MAX.
 *   head[gy][gx] is either −1 (empty bucket) or a valid bird index
 *   in [0, n_birds).
 *   next[i] for i ∈ [0, n_birds) is either −1 (i is the tail of its
 *   bucket) or another valid bird index pointing further down the
 *   same bucket's list.
 *
 * References
 *  [10] Teschner et al. 2003 — "Optimized spatial hashing for
 *       collision detection of deformable objects".  Canonical
 *       reference for uniform-grid bucket hashing in particle /
 *       agent systems; this struct implements the simplest form
 *       (no hash function — direct cell index as bucket).
 */
typedef struct {
    int head[HASH_GRID_H_MAX][HASH_GRID_W_MAX];
    int next[N_BOIDS_MAX];
    int grid_w;
    int grid_h;
} SpatialHash;

/* Map a possibly-out-of-range cell index back into [0, extent) toroidally. */
static inline int hash_wrap_cell(int idx, int extent)
{
    int r = idx % extent;
    if (r < 0) r += extent;
    return r;
}

/* hash_cell_for_pos — pixel → cell coordinates (no wrap).  Caller wraps. */
static inline void hash_cell_for_pos(Vec2 pos, int *out_cx, int *out_cy)
{
    *out_cx = (int)floorf(pos.x / (float)HASH_CELL_PX);
    *out_cy = (int)floorf(pos.y / (float)HASH_CELL_PX);
}

/*
 * spatial_hash_build — populate the SpatialHash's head + next arrays
 * from the current bird positions.  Called once per tick BEFORE the
 * steering loop (so all reads inside the steering loop see a consistent
 * hash structure).
 *
 *   1. derive active grid dims from world extent, clamped to caps
 *   2. clear every head to −1 (empty)
 *   3. for each bird: prepend to its cell's linked list
 */
static void spatial_hash_build(SpatialHash *h, const Boid *pool, int n,
                               World world)
{
    /* (1) active grid dims = ceil(world / cell), clamped to compile-time cap */
    int gw = (int)ceilf(world.width  / (float)HASH_CELL_PX);
    int gh = (int)ceilf(world.height / (float)HASH_CELL_PX);
    if (gw < 1)               gw = 1;
    if (gh < 1)               gh = 1;
    if (gw > HASH_GRID_W_MAX) gw = HASH_GRID_W_MAX;
    if (gh > HASH_GRID_H_MAX) gh = HASH_GRID_H_MAX;
    h->grid_w = gw;
    h->grid_h = gh;

    /* (2) clear heads (only the active region) */
    for (int gy = 0; gy < gh; gy++)
        for (int gx = 0; gx < gw; gx++)
            h->head[gy][gx] = -1;

    /* (3) prepend each bird to its cell's singly-linked list */
    for (int i = 0; i < n; i++) {
        int cx, cy;
        hash_cell_for_pos(pool[i].pos, &cx, &cy);
        cx = hash_wrap_cell(cx, gw);
        cy = hash_wrap_cell(cy, gh);
        h->next[i]      = h->head[cy][cx];
        h->head[cy][cx] = i;
    }
}

/*
 * accumulate_pair_force — given a pair (self, neighbour=pool[j]), update
 * the per-rule accumulators for separation / alignment / cohesion if the
 * pair falls inside the relevant radius.  Skips self-pair (j == self)
 * and zero-distance overlaps.
 *
 * Caller has already done the cell-narrowing; this is the per-pair work.
 */
static inline void accumulate_pair_force(const Boid *me, const Boid *nb,
                                         int j, int self, World world,
                                         Vec2 *sep_force,
                                         Vec2 *ali_vsum, int *ali_n,
                                         Vec2 *coh_dsum, int *coh_n)
{
    const float DIST_EPSILON_SQ = 1e-6f;
    const float SEP_R2          = SEP_RADIUS      * SEP_RADIUS;
    const float ALIGN_R2        = ALIGN_RADIUS    * ALIGN_RADIUS;
    const float COH_R2          = COHESION_RADIUS * COHESION_RADIUS;
    if (j == self) return;

    /* (a) toroidal offset self → neighbour, squared distance for cheap
     *     comparisons before paying for sqrt */
    float offset_x         = toroidal_delta(me->pos.x, nb->pos.x, world.width);
    float offset_y         = toroidal_delta(me->pos.y, nb->pos.y, world.height);
    float squared_distance = offset_x*offset_x + offset_y*offset_y;

    bool inside_perception = squared_distance <= COH_R2;
    bool well_separated    = squared_distance >  DIST_EPSILON_SQ;
    if (!inside_perception || !well_separated) return;

    /* (b) COHESION sum — offsets, not absolute positions
     *     (averaging absolutes is wrong on a torus; see cohere_force) */
    coh_dsum->x += offset_x;
    coh_dsum->y += offset_y;
    (*coh_n)++;

    /* (c) ALIGNMENT sum — neighbour velocities inside the smaller align ring */
    if (squared_distance < ALIGN_R2) {
        ali_vsum->x += nb->vel.x;
        ali_vsum->y += nb->vel.y;
        (*ali_n)++;
    }

    /* (d) SEPARATION push — only neighbours inside personal-space radius
     *     distance              : NOW we pay sqrt (rare branch)
     *     personal_space_intrusion ∈ (0,1] — linear falloff: 1 at d=0, 0 at d=SEP
     *     push_away_dir         : unit vector pointing FROM neighbour
     *                             TOWARD self  (subtract offset_x = flip)
     *     contribution scaled by BOID_SPEED so the push has speed units */
    if (squared_distance < SEP_R2) {
        float distance                 = sqrtf(squared_distance);
        float personal_space_intrusion = (SEP_RADIUS - distance) / SEP_RADIUS;
        float push_away_x              = -offset_x / distance;
        float push_away_y              = -offset_y / distance;
        sep_force->x += push_away_x * personal_space_intrusion * BOID_SPEED;
        sep_force->y += push_away_y * personal_space_intrusion * BOID_SPEED;
    }
}

/*
 * boid_forces — compute the total steering force for one bird via the
 * spatial hash.
 *
 *   1. find self's hash cell
 *   2. walk the 3×3 cells centred on it (toroidal wrap; or full sweep
 *      along any axis whose grid dim < 3, to avoid double-counting)
 *   3. for every bird in those buckets, call accumulate_pair_force
 *   4. convert per-rule accumulators to forces, add hawk-flee, sum,
 *      clamp to MAX_STEER
 *
 * Reads only — no writes to pool[].  scene_tick captures all new_vel[]
 * before writing back, so the update is "simultaneous" across the flock.
 */
static Vec2 boid_forces(const Boid *pool, const SpatialHash *hash, int self,
                        Vec2 hawk_pos, World world)
{
    const Boid *me = &pool[self];

    /* Per-rule accumulators */
    Vec2 sep_force = v2(0, 0);
    Vec2 ali_vsum  = v2(0, 0); int ali_n = 0;
    Vec2 coh_dsum  = v2(0, 0); int coh_n = 0;   /* OFFSETS, not absolute */

    /* (1) self's hash cell */
    int self_cx, self_cy;
    hash_cell_for_pos(me->pos, &self_cx, &self_cy);
    self_cx = hash_wrap_cell(self_cx, hash->grid_w);
    self_cy = hash_wrap_cell(self_cy, hash->grid_h);

    /* (2) build dy/dx sweep ranges — special-case small worlds:
     *     grid_h ≥ 3: dy ∈ {-1, 0, +1} with toroidal wrap (standard 3×3)
     *     grid_h < 3: walk every row 0..grid_h-1 once (no wrap, no dup) */
    int dy_lo, dy_hi, dy_use_wrap;
    if (hash->grid_h >= 3) { dy_lo = -1;            dy_hi =  1;                 dy_use_wrap = 1; }
    else                   { dy_lo =  0;            dy_hi = hash->grid_h - 1;   dy_use_wrap = 0; }

    int dx_lo, dx_hi, dx_use_wrap;
    if (hash->grid_w >= 3) { dx_lo = -1;            dx_hi =  1;                 dx_use_wrap = 1; }
    else                   { dx_lo =  0;            dx_hi = hash->grid_w - 1;   dx_use_wrap = 0; }

    /* (3) walk 3×3 neighbourhood (or full sweep on small axis), accumulate */
    for (int dy = dy_lo; dy <= dy_hi; dy++) {
        int cy = dy_use_wrap ? hash_wrap_cell(self_cy + dy, hash->grid_h) : dy;
        for (int dx = dx_lo; dx <= dx_hi; dx++) {
            int cx = dx_use_wrap ? hash_wrap_cell(self_cx + dx, hash->grid_w) : dx;
            for (int j = hash->head[cy][cx]; j != -1; j = hash->next[j]) {
                accumulate_pair_force(me, &pool[j], j, self, world,
                                      &sep_force,
                                      &ali_vsum, &ali_n,
                                      &coh_dsum, &coh_n);
            }
        }
    }

    /* (4) Accumulators → final per-rule forces */
    Vec2 ali  = align_force    (ali_vsum, ali_n, me->vel);
    Vec2 coh  = cohere_force   (coh_dsum, coh_n, me->vel);
    Vec2 flee = hawk_flee_force(me->pos, hawk_pos, world);

    /* (5) Sum weighted forces, clamp to MAX_STEER for smooth curves */
    Vec2 total = v2add(
        v2add(v2scale(sep_force, W_SEP),
              v2scale(ali,       W_ALIGN)),
        v2add(v2scale(coh,       W_COHERE),
              v2scale(flee,      W_FLEE))
    );
    return v2clamp_len(total, MAX_STEER);
}

/* ===================================================================== */
/* §7  hawk — PATROL / DIVE controller                                    */
/* ===================================================================== */

/*
 * HawkMode — finite-state machine for the predator's behaviour.
 *
 * Intent
 *   Four discrete states + one cycle helper give the user three
 *   "base" modes (PATROL, HOVER, PURSUE) to choose from with n / p,
 *   plus one OVERLAY mode (DIVE) triggered by SPACE or the auto-dive
 *   timer.  When DIVE expires, the FSM returns to whichever base
 *   mode was last selected — never teleports.
 *
 * State diagram
 *
 *       n press  ─►    n press  ─►    n press  ─►
 *      ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
 *      │ PATROL │ │ HOVER  │ │ PURSUE │ │ PATROL │  (cycles via n/p)
 *      └───┬────┘ └───┬────┘ └───┬────┘ └────────┘
 *          ◄─ p press ◄─ p press ◄─
 *
 *          SPACE (or auto-dive timer) from ANY base mode:
 *            → enter DIVE; dive_timer = HAWK_DIVE_DURATION
 *            → vel = unit(centroid − pos) · HAWK_DIVE_SPEED
 *
 *          DIVE → base_mode  when dive_timer ≤ 0
 *            → if base_mode == PATROL:
 *                patrol_phase = atan2(pos − world_centre)
 *                (orbit resumes from current pos; no teleport)
 *
 * Per-mode behaviour
 *
 *   PATROL  Closed-form orbit around the world centre at radius
 *           HAWK_PATROL_FRAC · min(w,h), angular speed
 *           HAWK_PATROL_OMEGA = 0.4 rad/s (one orbit ≈ 15.7 s).
 *           pos is a pure function of (centre, r, phase) — no
 *           velocity integration.  Hawk reads as "watching" rather
 *           than chasing.  Reference: [2] Reynolds 1999 §"Wander" —
 *           same closed-form-circular-motion pattern.
 *
 *   HOVER   Slow drift toward flock centroid at HAWK_HOVER_DRIFT
 *           (60 px/s).  Inside HAWK_HOVER_HOLD_RADIUS (30 px) the
 *           drift speed scales linearly to 0, producing a smooth
 *           settle.  Flock forms a moving torus around the hawk —
 *           classic murmuration motif when an aerial predator is
 *           present.  Reference: [5] Couzin et al. 2002 — analogous
 *           "loaf" / "doughnut" configurations.
 *
 *   PURSUE  Continuous tracking at HAWK_PURSUE_SPEED (110 px/s,
 *           slightly above BOID_SPEED = 90).  Hawk closes in slowly;
 *           flock keeps running and reshapes.  Reference: [2]
 *           Reynolds 1999 §"Pursue" — directly relevant.
 *
 *   DIVE    Straight-line sprint at HAWK_DIVE_SPEED (250 px/s) for
 *           HAWK_DIVE_DURATION (1.5 s) toward the flock centroid.
 *           Models the "stoop" of a real raptor attack.  References:
 *           [6] Hildenbrandt et al. 2010 (hawk dive simulation),
 *           [7] Cavagna et al. 2010 (empirical evidence that real
 *           starling flocks fragment + reform around such attacks).
 *
 * Members
 *   HAWK_PATROL    = 0  Default base; orbiting watch.
 *   HAWK_HOVER     = 1  Stationary threat over the flock.
 *   HAWK_PURSUE    = 2  Continuous low-speed chase.
 *   HAWK_DIVE      = 3  Overlay; never in the n/p cycle.
 *   HAWK_MODE_COUNT_BASE  Sentinel == HAWK_DIVE (= 3 cyclable bases).
 *
 * Invariants
 *   0 ≤ mode < HAWK_DIVE+1 == 4.
 *   0 ≤ base_mode < HAWK_MODE_COUNT_BASE == 3 (DIVE never in base).
 *
 * References (collected)
 *   [1] Reynolds 1987 — boid model the prey-flock obeys.
 *   [2] Reynolds 1999 — steering primitives (PATROL=wander,
 *       HOVER=arrival, PURSUE=pursue) reused as hawk steerings.
 *   [5] Couzin et al. 2002 — torus / loaf configurations.
 *   [6] Hildenbrandt et al. 2010 — hawk-dive simulation.
 *   [7] Cavagna et al. 2010 — empirical fragment-and-reform.
 */
typedef enum {
    HAWK_PATROL = 0,
    HAWK_HOVER,
    HAWK_PURSUE,
    HAWK_DIVE,                  /* always last — overlay, not in n/p cycle */
    HAWK_MODE_COUNT_BASE = HAWK_DIVE  /* number of cyclable base modes (3) */
} HawkMode;

static const char *HAWK_MODE_NAMES[] = {
    [HAWK_PATROL] = "PATROL",
    [HAWK_HOVER ] = "HOVER ",
    [HAWK_PURSUE] = "PURSUE",
    [HAWK_DIVE  ] = "DIVE  ",
};

/*
 * Hawk — the singleton predator + its controller state.
 *
 * Intent
 *   One Hawk instance lives on Scene.  Each tick its mode-specific
 *   stepper (hawk_step_patrol / _hover / _pursue / _dive) advances
 *   pos + vel; every Boid feels a flee force via hawk_flee_force.
 *
 * Why ONE struct instead of separate state-per-mode
 *   The four modes share the same KINEMATIC variables (pos, vel) but
 *   only some use each controller variable (patrol_phase only in
 *   PATROL, dive_timer only in DIVE).  A flat layout keeps the
 *   structure size small (~36 bytes) and avoids the inheritance
 *   gymnastics of per-mode subtypes.  The few unused fields-per-
 *   mode are tolerable noise.
 *
 * Why TWO mode fields (mode + base_mode)
 *   The state machine has 3 cyclable bases plus 1 overlay (see
 *   HawkMode docblock).  `mode` tracks WHAT'S RUNNING right now
 *   (which stepper to call); `base_mode` tracks WHERE TO RETURN
 *   when DIVE ends.  Without `base_mode`, a DIVE fired while in
 *   HOVER would return to PATROL by default, losing the user's
 *   selection.  With it, DIVE is a true non-destructive overlay.
 *
 * Members — kinematic state
 *   pos          world-pixel position; clamped/wrapped by hawk_wrap.
 *                Source of the flee-vector emanation for every Boid.
 *   vel          world-pixel velocity (units = px/s).  USED by HOVER,
 *                PURSUE, DIVE.  IGNORED by PATROL (which writes pos
 *                directly from closed-form orbit equation, then
 *                leaves vel stale — no harm because PATROL is the
 *                only mode that doesn't read vel).
 *
 * Members — controller state
 *   mode         live FSM state (PATROL/HOVER/PURSUE/DIVE).  Read by
 *                hawk_step's dispatcher to pick the right stepper.
 *                Also read by screen_draw for the HUD label and
 *                glyph choice ('!' during DIVE, 'X' otherwise).
 *   base_mode    persistent "what to return to after DIVE" — set by
 *                hawk_cycle_base_mode (n/p keys).  Range: PATROL,
 *                HOVER, PURSUE.  Never HAWK_DIVE.
 *   patrol_phase Orbit angle in RADIANS.  Advanced by
 *                HAWK_PATROL_OMEGA·dt each PATROL tick.  When DIVE
 *                ends with base_mode==PATROL, this is recomputed as
 *                atan2(pos − centre) so the orbit resumes smoothly
 *                from where the dive ended (NO TELEPORT — see [2]
 *                Reynolds 1999 §"Wander" for the same trick).
 *   dive_timer   Seconds remaining in the current DIVE.  Decremented
 *                each DIVE tick by dt; on ≤ 0 the FSM transitions to
 *                base_mode.  Unused outside DIVE.
 *
 * Members — user-facing flags
 *   auto_dive       Toggled by `h` key.  When true, auto_dive_timer
 *                   accumulates dt; on reaching AUTO_DIVE_PERIOD,
 *                   scene_tick fires a dive automatically.  Lets the
 *                   user watch the cycle "cohere → split → reform"
 *                   without continuous keypresses.
 *   auto_dive_timer Seconds since the last (auto-)fired dive.  Reset
 *                   to 0 on each fire and on `h` toggle.  Only ticks
 *                   while auto_dive is true AND mode != HAWK_DIVE.
 *
 * Invariants
 *   mode ∈ {PATROL, HOVER, PURSUE, DIVE}.
 *   base_mode ∈ {PATROL, HOVER, PURSUE}  (never DIVE).
 *   dive_timer > 0  IFF  mode == HAWK_DIVE  (otherwise unread).
 *   0 ≤ pos.x < World.width  AND  0 ≤ pos.y < World.height after wrap.
 *
 * References
 *   [2] Reynolds 1999 — Pursue / Arrival / Wander steerings reused.
 *   [6] Hildenbrandt et al. 2010 — adds aerial predators to the
 *       canonical starling-murmuration simulation.
 *   [7] Cavagna et al. 2010 — empirical evidence that real flocks
 *       fragment and reform around hawk attacks.
 */
typedef struct {
    /* kinematic state */
    Vec2 pos;
    Vec2 vel;        /* used by HOVER / PURSUE / DIVE; PATROL recomputes pos */

    /* controller state */
    HawkMode mode;            /* current state — includes DIVE overlay  */
    HawkMode base_mode;       /* PATROL/HOVER/PURSUE — DIVE returns here */
    float    patrol_phase;    /* radians around world centre            */
    float    dive_timer;      /* seconds remaining in current dive      */

    /* user-facing flag (HUD + auto-dive) */
    bool     auto_dive;
    float    auto_dive_timer; /* seconds since last (auto-)fired dive */
} Hawk;

/*
 * hawk_cycle_base_mode — cycle the persistent base mode forward (dir=+1,
 * `n` key) or backward (dir=−1, `p` key) through the three cyclable
 * states {PATROL, HOVER, PURSUE}.  If the hawk is currently in DIVE,
 * the cycle changes what it will return to when the dive ends; the
 * dive itself runs to completion.  If the hawk is in any base mode,
 * the cycle also flips the live mode.
 */
static void hawk_cycle_base_mode(Hawk *h, int dir)
{
    int next = ((int)h->base_mode + dir + HAWK_MODE_COUNT_BASE)
             % HAWK_MODE_COUNT_BASE;
    h->base_mode = (HawkMode)next;
    if (h->mode != HAWK_DIVE) h->mode = h->base_mode;
}

/*
 * hawk_dive — kick off a dive toward `target` (typically the flock
 * centroid).  No-op if the hawk is already mid-dive.
 *
 *   pick the direction TARGET − POS;
 *   set vel = direction · HAWK_DIVE_SPEED;
 *   start dive_timer = HAWK_DIVE_DURATION;
 *   mode = DIVE.
 *
 * The dive moves in a straight line and IGNORES toroidal wrap on the
 * direction calculation — if the centroid wraps across an edge, the
 * hawk will fly the long way around.  This is fine because PATROL
 * resumes immediately after dive_timer expires, and the hawk's orbit
 * will re-centre.
 */
static void hawk_dive(Hawk *h, Vec2 target)
{
    if (h->mode == HAWK_DIVE) return;
    Vec2 dir = v2norm(v2sub(target, h->pos));
    if (v2len(dir) < 0.001f) return;       /* degenerate: hawk on target */

    h->vel        = v2scale(dir, HAWK_DIVE_SPEED);
    h->dive_timer = HAWK_DIVE_DURATION;
    h->mode       = HAWK_DIVE;
}

/* ── per-mode hawk steppers ─────────────────────────────────────────── */

/*
 * hawk_step_patrol — closed-form orbit position.
 *
 *   patrol_phase += OMEGA · dt
 *   pos = centre + r · (cos φ, sin φ)
 *
 * No velocity integration; pos is a pure function of (centre, r, phase).
 */
static void hawk_step_patrol(Hawk *h, Vec2 world_centre, float patrol_r, float dt)
{
    h->patrol_phase += HAWK_PATROL_OMEGA * dt;
    Vec2 orbit_offset = v2scale(v2(cosf(h->patrol_phase),
                                    sinf(h->patrol_phase)), patrol_r);
    h->pos = v2add(world_centre, orbit_offset);
}

/*
 * hawk_step_hover — slow drift toward flock centroid; settle once close.
 *
 *   offset_to_centroid  = centroid − pos  (no toroidal wrap; hover is
 *                         a local LOS approach, not a wrap-aware seek)
 *   distance            = |offset_to_centroid|
 *   if distance < HOLD_RADIUS:
 *       speed scales linearly from HAWK_HOVER_DRIFT at the edge to 0
 *       at the centroid — produces a smooth settle, not a stop-jerk
 *   else:
 *       speed = HAWK_HOVER_DRIFT (full drift toward target)
 *   vel = unit(offset) · speed; pos += vel · dt
 */
static void hawk_step_hover(Hawk *h, Vec2 centroid, float dt)
{
    Vec2  offset_to_centroid = v2sub(centroid, h->pos);
    float distance           = v2len(offset_to_centroid);
    if (distance < 0.001f) { h->vel = v2(0, 0); return; }

    float settle_fraction = (distance < HAWK_HOVER_HOLD_RADIUS)
                          ? (distance / HAWK_HOVER_HOLD_RADIUS)
                          : 1.0f;
    float drift_speed     = HAWK_HOVER_DRIFT * settle_fraction;

    Vec2 toward_centroid = v2scale(offset_to_centroid, 1.0f / distance);
    h->vel = v2scale(toward_centroid, drift_speed);
    h->pos = v2add(h->pos, v2scale(h->vel, dt));
}

/*
 * hawk_step_pursue — continuous chase at HAWK_PURSUE_SPEED toward flock
 * centroid.  Always at full speed (no settling) — the hawk is actively
 * trying to close in.  Boid flee force still applies normally.
 */
static void hawk_step_pursue(Hawk *h, Vec2 centroid, float dt)
{
    Vec2  offset_to_centroid = v2sub(centroid, h->pos);
    float distance           = v2len(offset_to_centroid);
    if (distance < 0.001f) { h->vel = v2(0, 0); return; }

    Vec2 toward_centroid = v2scale(offset_to_centroid, 1.0f / distance);
    h->vel = v2scale(toward_centroid, HAWK_PURSUE_SPEED);
    h->pos = v2add(h->pos, v2scale(h->vel, dt));
}

/*
 * hawk_step_dive — straight-line sprint at HAWK_DIVE_SPEED until the
 * dive_timer expires; on expiry, snap back to base_mode with state
 * re-derived from current position so there's no teleport.
 *
 *   pos += vel · dt
 *   dive_timer -= dt
 *   if dive_timer ≤ 0:
 *       if base_mode == PATROL:
 *           re-derive patrol_phase = atan2(pos − centre) so orbit
 *           resumes from CURRENT position (no jump back to ring)
 *       mode = base_mode
 */
static void hawk_step_dive(Hawk *h, Vec2 world_centre, float dt)
{
    h->pos = v2add(h->pos, v2scale(h->vel, dt));
    h->dive_timer -= dt;
    if (h->dive_timer > 0.0f) return;

    if (h->base_mode == HAWK_PATROL) {
        h->patrol_phase = atan2f(h->pos.y - world_centre.y,
                                  h->pos.x - world_centre.x);
    }
    h->mode = h->base_mode;
}

/* hawk_wrap — toroidal wrap for the hawk; keeps it on-screen during
 * long dives that would otherwise punch off the edge. */
static void hawk_wrap(Hawk *h, World world)
{
    if (h->pos.x <  0.0f)         h->pos.x += world.width;
    if (h->pos.x >= world.width)  h->pos.x -= world.width;
    if (h->pos.y <  0.0f)         h->pos.y += world.height;
    if (h->pos.y >= world.height) h->pos.y -= world.height;
}

/*
 * hawk_step — dispatcher: pick the right per-mode stepper, then wrap.
 *
 * Centroid is supplied by the caller (Scene refreshes it each tick).
 * For PATROL, centroid is unused; for HOVER/PURSUE/DIVE it's the target.
 */
static void hawk_step(Hawk *h, Vec2 world_centre, float patrol_r,
                      Vec2 flock_centroid, World world, float dt)
{
    switch (h->mode) {
        case HAWK_PATROL: hawk_step_patrol(h, world_centre, patrol_r,    dt); break;
        case HAWK_HOVER:  hawk_step_hover (h, flock_centroid,            dt); break;
        case HAWK_PURSUE: hawk_step_pursue(h, flock_centroid,            dt); break;
        case HAWK_DIVE:   hawk_step_dive  (h, world_centre,              dt); break;
        default: break;
    }
    hawk_wrap(h, world);
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   Bundle the two scalars that the input handler writes and the HUD +
 *   tick read — `paused` (toggled by `.`) and `theme_idx` (cycled by
 *   t / T).  Keeps app_handle_key compact and gives the Scene's top
 *   level a cleaner shape (everything user-controlled in one place).
 *
 * Members
 *   paused      true → scene_tick early-returns; HUD shows "PAUSED".
 *   theme_idx   active flock colour palette in [0, N_THEMES); cycled
 *               by t / T.  theme_apply() re-registers ncurses pairs.
 *
 * Invariants
 *   0 ≤ theme_idx < N_THEMES (clamped in app_handle_key).
 */
typedef struct {
    bool paused;
    int  theme_idx;
} SimControls;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── pool[N_BOIDS_MAX] : Boid[]       ← flock pool (first n active)
 *       ├── n_birds           : int          ← active prefix length
 *       ├── hawk              : Hawk         ← singleton predator
 *       ├── sim               : SimControls  ← paused + theme_idx
 *       ├── world             : World        ← pixel-space simulation extent
 *       ├── flock_centroid    : Vec2         ← derived (refreshed each tick)
 *       └── hash              : SpatialHash  ← per-tick neighbour structure
 *
 *   Every persistent simulation value is reachable from one Scene*.
 *   Terminal state (cols, rows, ncurses lifecycle) lives on Screen in §9;
 *   POSIX signal flags live on App in §9 because signal handlers can't
 *   receive a context pointer.
 *
 * Why this hierarchy
 *   • Replaceability: a future split-screen demo allocates multiple
 *     Scenes; file-scope globals would prevent that.
 *   • Reading aid: every helper takes `Scene *`, giving the reader ONE
 *     place to look for "what state exists".
 *   • The spatial hash sits on Scene rather than as file-scope statics
 *     so it travels with the simulation it indexes — see SpatialHash
 *     docblock above (§6.5) for the rationale.
 */
typedef struct {
    /* ── flock pool ─ first n_birds slots active ───────────────── */
    Boid  pool[N_BOIDS_MAX];
    int   n_birds;

    /* ── singleton predator ────────────────────────────────────── */
    Hawk  hawk;

    /* ── user-facing knobs (paused + theme_idx) ────────────────── */
    SimControls sim;

    /* ── pixel-space simulation extent (refreshed on resize) ───── */
    World world;

    /* ── derived: flock centre of mass (refreshed each tick) ───── */
    Vec2  flock_centroid;

    /* ── per-tick neighbour index (rebuilt each tick) ──────────── */
    SpatialHash hash;
} Scene;

/*
 * scene_init — fresh scene with random birds and a hawk parked at the
 * east edge of its patrol orbit.  Theme index is preserved across
 * reset (saved before memset, restored after).
 */
/*
 * world_centre — geometric centre of the simulation box in pixel space.
 * Used as patrol orbit centre + initial centroid + dive return point. */
static inline Vec2 world_centre(World world)
{
    const float WORLD_CENTRE_FRAC = 0.5f;
    return v2(world.width  * WORLD_CENTRE_FRAC,
              world.height * WORLD_CENTRE_FRAC);
}

/* hawk_patrol_radius_for — orbit radius = HAWK_PATROL_FRAC × min(w, h).
 * The min-dimension constraint keeps the orbit visible on tall-and-thin
 * or short-and-wide terminals where one axis would otherwise dominate. */
static inline float hawk_patrol_radius_for(World world)
{
    float min_dim = (world.width < world.height) ? world.width : world.height;
    return min_dim * HAWK_PATROL_FRAC;
}

/*
 * spawn_boid_pool — pre-spawn every bird in the pool to a random
 * position in the world.  Called once at scene_init; the '+'/'-'
 * keys just slide n_birds without re-randomising.
 */
static void spawn_boid_pool(Boid *pool, World world)
{
    for (int i = 0; i < N_BOIDS_MAX; i++)
        boid_spawn(&pool[i], i, world);
}

/*
 * place_hawk_at_orbit_east — park the hawk on the east edge of its
 * patrol orbit (phase = 0, position = centre + (r, 0)) so the first
 * frame shows the hawk away from the spawn cluster.
 */
static void place_hawk_at_orbit_east(Hawk *h, World world)
{
    Vec2  centre = world_centre(world);
    float r      = hawk_patrol_radius_for(world);
    h->pos             = v2add(centre, v2(r, 0));
    h->vel             = v2(0, 0);
    h->mode            = HAWK_PATROL;
    h->base_mode       = HAWK_PATROL;
    h->patrol_phase    = 0.0f;
    h->dive_timer      = 0.0f;
    h->auto_dive       = false;
    h->auto_dive_timer = 0.0f;
}

static void scene_init(Scene *s, int cols, int rows)
{
    /* (a) Preserve theme across reset; zero everything else */
    int saved_theme = s->sim.theme_idx;
    memset(s, 0, sizeof *s);
    s->sim.theme_idx = saved_theme;
    s->sim.paused    = false;

    /* (b) World extent in pixel space, derived from terminal extent */
    s->world   = world_from_terminal(cols, rows);
    s->n_birds = N_BOIDS_DEFAULT;

    /* (c) Pre-spawn the full pool; place the predator on the orbit */
    spawn_boid_pool(s->pool, s->world);
    place_hawk_at_orbit_east(&s->hawk, s->world);

    /* (d) Initial centroid = world centre (no birds have moved yet) */
    s->flock_centroid = world_centre(s->world);
}

/*
 * scene_centroid — toroidal-aware flock centre of mass.  Used by the
 * hawk dive to choose a target.  For a small flock on a large world
 * this is just the average position; on small terminals (where the
 * flock can wrap around) we need to pick a reference and accumulate
 * toroidal offsets relative to it.
 */
static Vec2 scene_centroid(const Boid *pool, int n, World world)
{
    /* (1) degenerate case: no birds → world centre */
    if (n <= 0) return v2(world.width * 0.5f, world.height * 0.5f);

    /* (2) pick a reference bird and sum TOROIDAL OFFSETS to every other
     *     bird.  Averaging absolute positions would give the wrong
     *     centroid when the flock straddles a wrap edge — see the
     *     cohere_force docblock for the same trick. */
    Vec2 reference_pos = pool[0].pos;
    Vec2 offset_sum    = v2(0, 0);
    for (int i = 0; i < n; i++) {
        offset_sum.x += toroidal_delta(reference_pos.x, pool[i].pos.x, world.width);
        offset_sum.y += toroidal_delta(reference_pos.y, pool[i].pos.y, world.height);
    }

    /* (3) mean offset from reference = average displacement of the flock
     *     centroid_pos = reference_pos + mean_offset */
    Vec2 mean_offset_from_reference = v2scale(offset_sum, 1.0f / (float)n);
    Vec2 centroid_pos = v2add(reference_pos, mean_offset_from_reference);

    /* (4) wrap centroid back into [0, width) × [0, height) so callers
     *     get a valid world-space coordinate */
    if (centroid_pos.x <  0.0f)         centroid_pos.x += world.width;
    if (centroid_pos.x >= world.width)  centroid_pos.x -= world.width;
    if (centroid_pos.y <  0.0f)         centroid_pos.y += world.height;
    if (centroid_pos.y >= world.height) centroid_pos.y -= world.height;
    return centroid_pos;
}

/*
 * tick_auto_dive_timer — advance the auto-dive timer if enabled, fire a
 * dive once AUTO_DIVE_PERIOD has elapsed.  Only ticks while the hawk
 * is in a base mode (not already mid-dive) so successive dives don't
 * stack.  No-op when auto_dive is off.
 */
static void tick_auto_dive_timer(Hawk *h, Vec2 flock_centroid, float dt)
{
    bool eligible = h->auto_dive && h->mode != HAWK_DIVE;
    if (!eligible) return;

    h->auto_dive_timer += dt;
    if (h->auto_dive_timer >= AUTO_DIVE_PERIOD) {
        h->auto_dive_timer = 0.0f;
        hawk_dive(h, flock_centroid);
    }
}

/*
 * compute_new_velocities — stage 1 of the two-stage simultaneous update.
 *
 *   for each active bird i:
 *     force_i  = boid_forces(...)        (hashed neighbour scan)
 *     vel'_i   = clamp(vel_i + force_i·dt, MAX_SPEED)
 *
 * Reads pool[] only (OLD positions); writes new_vel[].  No bird sees
 * any already-moved neighbour.
 */
static void compute_new_velocities(const Boid *pool, const SpatialHash *hash,
                                   int n, Vec2 hawk_pos, World world,
                                   float dt, Vec2 *new_vel)
{
    for (int i = 0; i < n; i++) {
        Vec2 force      = boid_forces(pool, hash, i, hawk_pos, world);
        Vec2 raw_new_v  = v2add(pool[i].vel, v2scale(force, dt));
        new_vel[i]      = v2clamp_len(raw_new_v, BOID_MAX_SPEED);
    }
}

/*
 * commit_step_and_wrap — stage 2 of the two-stage simultaneous update.
 *
 *   for each active bird i:
 *     vel_i       = new_vel[i]                         (write back)
 *     prev_pos_i  = pos_i                              (snapshot)
 *     pos_i      += vel_i · dt                         (Euler integrate)
 *     toroidal-wrap pos_i
 *     clamp |vel_i| ∈ [MIN_SPEED, MAX_SPEED]
 */
static void commit_step_and_wrap(Boid *pool, int n, const Vec2 *new_vel,
                                 World world, float dt)
{
    for (int i = 0; i < n; i++) {
        Boid *b      = &pool[i];
        b->vel       = new_vel[i];
        b->prev_pos  = b->pos;
        b->pos       = v2add(b->pos, v2scale(b->vel, dt));
        boid_wrap(b, world);
        boid_clamp_speed(b);
    }
}

/*
 * scene_tick — one fixed-step physics update.
 *
 *   (1) auto-dive timer  : tick_auto_dive_timer fires DIVE on schedule
 *   (2) hawk             : hawk_step dispatches by mode
 *   (3) spatial hash     : rebuild once for THIS tick's positions
 *   (4) STAGE 1 (read)   : compute_new_velocities reads OLD pool[]
 *   (5) STAGE 2 (write)  : commit_step_and_wrap writes vel + pos
 *   (6) centroid         : refresh for next-frame HUD + future dive
 *
 * The two-stage simultaneous update is the canonical fix for the
 * "boid earlier in array sees the next boid already moved" drift —
 * see flocking.c MENTAL MODEL for the same pattern with detail.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;
    World world = s->world;

    /* (1) auto-dive trigger (only when hawk is in a base mode) */
    tick_auto_dive_timer(&s->hawk, s->flock_centroid, dt);

    /* (2) hawk step — pick stepper by mode; PATROL needs centre + radius,
     *     HOVER/PURSUE/DIVE use the flock centroid */
    hawk_step(&s->hawk,
              world_centre(world),
              hawk_patrol_radius_for(world),
              s->flock_centroid, world, dt);

    /* (3) build spatial hash ONCE for this tick so every boid_forces call
     *     below sees a consistent neighbour structure.  O(N). */
    spatial_hash_build(&s->hash, s->pool, s->n_birds, world);

    /* (4) STAGE 1 — compute every new velocity from OLD positions */
    static Vec2 new_vel[N_BOIDS_MAX];   /* file-scope: keep stack small */
    compute_new_velocities(s->pool, &s->hash, s->n_birds,
                           s->hawk.pos, world, dt, new_vel);

    /* (5) STAGE 2 — write velocities, integrate, wrap, clamp speed */
    commit_step_and_wrap(s->pool, s->n_birds, new_vel, world, dt);

    /* (6) refresh centroid for next-frame HUD and any future dive */
    s->flock_centroid = scene_centroid(s->pool, s->n_birds, world);
}

/* ── render ──────────────────────────────────────────────────────────── */

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx, cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that
 * would otherwise be repeated at every mvwaddch site.  The double
 * cast prevents sign-extension on values > 127 (per CLAUDE.md
 * "Common ncurses Bugs").  Off-screen cells are silently dropped.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * Per-frame density buffer.  Static so we don't re-allocate every
 * frame and don't put 80 KB on the stack.  Sized to MAX_ROWS × MAX_COLS
 * to handle very large terminals; scene_draw zeroes only the active
 * region (rows × cols), not the whole buffer.
 */
static int g_density[MAX_ROWS][MAX_COLS];

/*
 * density_glyph — pick the glyph + brightness tier for a given count.
 *
 *   density 0      : not drawn (caller skips)
 *   density 1-4    : A_NORMAL, glyphs '.' ',' ':' ';'
 *   density 5+     : A_BOLD,   glyphs 'o' 'O' '*' '#' '@' (saturating)
 *
 * No A_DIM tier — at A_DIM most terminals render colours at ~half
 * intensity, and since most cells in the flock have density 1 or 2
 * the periphery would fade to near-black.  The visual fade-to-edge
 * comes from the GLYPH ramp ('.' for sparse → '@' for dense), not
 * from brightness modulation; using only A_NORMAL and A_BOLD keeps
 * every drawn cell legible while still giving the bright core its
 * characteristic A_BOLD glow.
 */
static void density_glyph(int density, char *out_ch, attr_t *out_attr)
{
    int idx = density - 1;
    if (idx < 0)            idx = 0;
    if (idx >= RAMP_LEN)    idx = RAMP_LEN - 1;
    *out_ch = DENSITY_RAMP[idx];

    *out_attr = (density >= 5) ? A_BOLD : A_NORMAL;
}

/*
 * scene_draw — paint the frame in painter's order:
 *
 *   1. Bin every bird into the density grid (skipping out-of-bounds).
 *   2. For each non-empty cell, stamp the corresponding glyph from the
 *      ASCII ramp with its brightness tier; the colour pair cycles
 *      through pairs 1..N_COLORS keyed by cell position so the flock
 *      reads as a coherent tinted cloud rather than monochrome.
 *   3. Hawk on top — always A_BOLD red `!`.
 *
 * The flock tint per cell uses (cy * 7 + cx) % N_COLORS — a cheap
 * spatial hash that gives the cloud a soft mottled colour without
 * recomputing per-bird identity.
 */
/*
 * zero_density_region — clear the active rectangle of the density grid.
 * Only zeros the visible region (cap_rows × cap_cols) — touching the
 * full buffer would be wasteful on small terminals.
 */
static void zero_density_region(int cap_cols, int cap_rows)
{
    for (int r = 0; r < cap_rows; r++)
        for (int c = 0; c < cap_cols; c++)
            g_density[r][c] = 0;
}

/*
 * bin_boids_into_density — histogram step: for each bird, increment
 * the count in its cell.  O(N) over birds + bounds-check per bird.
 *
 *   for each bird:
 *     (cx, cy) = px_to_cell(bird.pos)
 *     if inside the visible region:  density[cy][cx]++
 */
static void bin_boids_into_density(const Boid *pool, int n_birds,
                                   int cap_cols, int cap_rows)
{
    for (int i = 0; i < n_birds; i++) {
        int cx = px_to_cell_x(pool[i].pos.x);
        int cy = px_to_cell_y(pool[i].pos.y);
        if (cx < 0 || cx >= cap_cols || cy < 0 || cy >= cap_rows) continue;
        g_density[cy][cx]++;
    }
}

/*
 * tint_pair_for_cell — choose a flock colour pair index for the cell
 * at (cx, cy).  Uses a CHEAP SPATIAL HASH `(cy·7 + cx) mod N_COLORS`
 * so adjacent cells get DIFFERENT pairs — produces the subtle mottle
 * inside an otherwise monochrome flock cloud.  7 is coprime with
 * N_COLORS=7's typical neighbours; any odd small int gives a similar
 * effect.
 */
static inline int tint_pair_for_cell(int cx, int cy)
{
    const int SPATIAL_HASH_PRIME = 7;
    return ((cy * SPATIAL_HASH_PRIME + cx) % N_COLORS) + 1;
}

/*
 * render_density_field — for each non-empty density cell, look up the
 * glyph + brightness tier from the ramp and paint with a spatial-
 * hashed flock tint.  This is the step that makes 1500 boids read as
 * ONE cloud — see CONCEPTS §"Density binning" + tutorial T1/T2.
 */
static void render_density_field(WINDOW *w, int cap_cols, int cap_rows,
                                 int cols, int rows)
{
    for (int cy = 0; cy < cap_rows; cy++) {
        for (int cx = 0; cx < cap_cols; cx++) {
            int density_in_cell = g_density[cy][cx];
            if (density_in_cell == 0) continue;

            char   glyph;
            attr_t brightness_tier;
            density_glyph(density_in_cell, &glyph, &brightness_tier);

            int tint_pair = tint_pair_for_cell(cx, cy);
            mark_cell(w, cx, cy, glyph, tint_pair, brightness_tier, cols, rows);
        }
    }
}

/*
 * stamp_hawk_glyph — over-stamp the hawk on top of the rendered flock.
 * Glyph is '!' during DIVE (urgency), 'X' otherwise.  Always A_BOLD
 * in PAIR_HAWK (theme-independent bright red) so the predator is
 * visible regardless of which flock theme is active.
 */
static void stamp_hawk_glyph(WINDOW *w, const Hawk *hawk, int cols, int rows)
{
    int  cx       = px_to_cell_x(hawk->pos.x);
    int  cy       = px_to_cell_y(hawk->pos.y);
    char hawk_glyph = (hawk->mode == HAWK_DIVE) ? '!' : 'X';
    mark_cell(w, cx, cy, hawk_glyph, PAIR_HAWK, A_BOLD, cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    /* (1) Visible region (clip to density-buffer dims) */
    int cap_rows = (rows < MAX_ROWS) ? rows : MAX_ROWS;
    int cap_cols = (cols < MAX_COLS) ? cols : MAX_COLS;

    /* (2) Density histogram: zero, then bin every bird */
    zero_density_region(cap_cols, cap_rows);
    bin_boids_into_density(s->pool, s->n_birds, cap_cols, cap_rows);

    /* (3) Convert density → glyphs + spatial-hash tint → paint cloud */
    render_density_field(w, cap_cols, cap_rows, cols, rows);

    /* (4) Hawk on top (always painted last) */
    stamp_hawk_glyph(w, &s->hawk, cols, rows);
}

/* ===================================================================== */
/* §9  app — screen + signals + main loop                                 */
/* ===================================================================== */

/*
 * Screen — cell-space (terminal) extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Owns the TERMINAL side of the world.  Where Scene.world tracks
 *   the pixel-space simulation box, this struct tracks the cell-space
 *   terminal grid that ncurses paints onto.  The two are linked but
 *   live in DIFFERENT spaces:
 *
 *      World.width  = Screen.cols × CELL_W      (CELL_W = 8  px)
 *      World.height = Screen.rows × CELL_H      (CELL_H = 16 px)
 *
 *   Keeping them in SEPARATE structs prevents the entire class of
 *   "rendered too early in cell space" aspect-ratio bugs — physics
 *   never sees cells, the renderer never sees pixels except through
 *   the one px_to_cell_x/y bridge in §4.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_draw / screen_present touch ncurses'
 *     initscr / endwin / mvprintw.  They all take `Screen *` to
 *     make this layer explicit at the type level.
 *   • Symmetry with World: simulation and rendering each get one
 *     struct named for the space they live in.
 *   • Future-proof: a status panel / split-screen / viewport offset
 *     adds fields here without touching Scene.
 *
 * Members
 *   cols   terminal width in CELLS (read from getmaxyx).  Used by
 *          the renderer for cell-bounds checks after px → cell
 *          conversion, and by screen_draw for right-aligning the
 *          top HUD row.
 *   rows   terminal height in CELLS.  Same role on the vertical
 *          axis; bottom-row index is `rows - 1` (where the key hint
 *          paints).
 *
 * Invariants
 *   cols > 0 AND rows > 0 (set by screen_init via getmaxyx).
 *   Scene.world.width  == cols × CELL_W,
 *   Scene.world.height == rows × CELL_H,
 *   maintained in lockstep by scene_init / app_do_resize.
 *
 * References
 *   None directly — terminal extent is a rendering substrate concern.
 *   Aspect-ratio compensation (CELL_W vs CELL_H = 1:2) is project-
 *   specific; see CLAUDE.md §"Coordinate / Physics".
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int initial_theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(initial_theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — compose one frame: density-rendered flock + hawk
 * + HUD bars.  HUD top-right (PAIR_HUD bright yellow A_BOLD) shows
 * fps + sim Hz + bird count + theme + hawk mode.  Bottom-left
 * (PAIR_HINT bright cyan A_BOLD) shows the key-binding strip.
 */
/*
 * format_hud_status — write the top-row HUD text into `buf`.
 *
 *   live_mode is what's actually running right now (PATROL/HOVER/PURSUE
 *   in normal use, DIVE during the overlay).  When DIVE is firing on
 *   top of a non-PATROL base, render `hawk:DIVE→HOVER` so the user
 *   sees what mode it'll return to when the dive ends.  Otherwise
 *   show just the live mode.
 *
 *   Suffix shows PAUSED / AUTO / nothing.
 */
static void format_hud_status(const Scene *sc, double fps, int sim_fps,
                              char *buf, size_t buflen)
{
    const char *live_mode = HAWK_MODE_NAMES[sc->hawk.mode];
    const char *suffix    = sc->sim.paused      ? "  PAUSED"
                          : sc->hawk.auto_dive  ? "  AUTO"
                                                : "";
    bool dive_over_non_patrol = (sc->hawk.mode == HAWK_DIVE)
                              && (sc->hawk.base_mode != HAWK_PATROL);

    if (dive_over_non_patrol) {
        snprintf(buf, buflen,
                 " %5.1f fps  sim:%3d Hz  n:%d/%d  [%s]  hawk:%s→%s%s ",
                 fps, sim_fps, sc->n_birds, N_BOIDS_MAX,
                 THEMES[sc->sim.theme_idx].name,
                 live_mode, HAWK_MODE_NAMES[sc->hawk.base_mode], suffix);
    } else {
        snprintf(buf, buflen,
                 " %5.1f fps  sim:%3d Hz  n:%d/%d  [%s]  hawk:%s%s ",
                 fps, sim_fps, sc->n_birds, N_BOIDS_MAX,
                 THEMES[sc->sim.theme_idx].name, live_mode, suffix);
    }
}

/* hud_paint_text — attron / mvprintw / attroff sandwich; both HUD rows
 * use this so the colour-pair setup isn't duplicated at every call. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_hud_status — right-aligned top-row data line (fps + mode + count). */
static void draw_hud_status(const Screen *s, const Scene *sc,
                            double fps, int sim_fps)
{
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_COLS + 1];
    format_hud_status(sc, fps, sim_fps, buf, sizeof buf);

    int right_col = s->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip. */
static void draw_hud_hint(const Screen *s)
{
    static const char *KEY_HINT =
        " q:quit  spc:dive  n/p:mode  h:auto-dive  .:pause  r:reset  t/T:theme  +/-:birds ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    /* (1) Clear offscreen buffer */
    erase();
    /* (2) Paint the simulation (density field + hawk) */
    scene_draw(sc, stdscr, s->cols, s->rows);
    /* (3) HUD on top — data on row 0, key hints on last row */
    draw_hud_status(s, sc, fps, sim_fps);
    draw_hud_hint  (s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App + signal handlers ──────────────────────────────────────────── */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Intent
 *   Per-frame fps would jitter wildly (a single 1 ms variance swings
 *   the instantaneous reading by ~6 fps).  This struct accumulates
 *   frame_count + elapsed nanoseconds over a fixed window
 *   (FPS_UPDATE_MS = 500 ms) and emits a smoothed `display` figure
 *   each time the window fills, so the HUD reads a stable number.
 *
 * Lifecycle
 *   fps_counter_init  — zero everything at startup.
 *   fps_counter_tick  — call once per frame with frame dt (ns); emits
 *                       a fresh `display` only when the window fills.
 *
 * Members
 *   frame_count  frames seen in the current window
 *   window_ns    nanoseconds accumulated in the current window
 *   display      most recent smoothed fps figure (HUD reads this)
 *
 * Why on App (not on Scene)
 *   Render-side bookkeeping, not simulation state — keeps Scene the
 *   pure-physics container.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f)
{
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

/* fps_counter_tick — tally one frame; emit smoothed reading when window fills. */
static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = (int64_t)FPS_UPDATE_MS * NS_PER_MS;
    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display = (double)f->frame_count
               / ((double)f->window_ns / (double)NS_PER_SEC);
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value in the program.
 *
 * Layered ownership
 *
 *     App
 *       ├── scene       : Scene       ← simulation state
 *       │      ├── pool[]       : Boid[]
 *       │      ├── n_birds      : int
 *       │      ├── hawk         : Hawk
 *       │      ├── sim          : SimControls   ← paused + theme_idx
 *       │      ├── world        : World         ← pixel-space extent
 *       │      ├── flock_centroid: Vec2
 *       │      └── hash         : SpatialHash   ← per-tick neighbour idx
 *       │
 *       ├── screen      : Screen      ← terminal cell extent
 *       ├── fps         : FpsCounter  ← rolling-window frame-rate
 *       ├── sim_fps     : int         ← target physics rate (config)
 *       ├── running     : sig_atomic_t  ← cleared by on_exit_signal
 *       └── need_resize : sig_atomic_t  ← set by on_resize_signal
 *
 * Intent
 *   One static `g_app` is the sole file-scope variable in the program;
 *   every helper reaches state through `&g_app` (signal handlers) or
 *   through an `App *app` argument (everything else).  See flocking.c
 *   App docblock for the broader rationale on volatile sig_atomic_t
 *   and the "one root, everything reachable" design.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scene *sc = &app->scene;

    /* Refresh pixel-space world extent from new terminal extent. */
    sc->world = world_from_terminal(app->screen.cols, app->screen.rows);
    World world = sc->world;

    /* Clamp every bird and the hawk into the new world bounds. */
    for (int i = 0; i < sc->n_birds; i++) {
        Boid *b = &sc->pool[i];
        if (b->pos.x >= world.width)  b->pos.x = world.width  - 1.0f;
        if (b->pos.y >= world.height) b->pos.y = world.height - 1.0f;
        b->prev_pos = b->pos;
    }
    if (sc->hawk.pos.x >= world.width)  sc->hawk.pos.x = world.width  - 1.0f;
    if (sc->hawk.pos.y >= world.height) sc->hawk.pos.y = world.height - 1.0f;

    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 *
 *   q / Q / ESC    quit
 *   space          fire one hawk DIVE at the flock centroid (overlay)
 *   n              cycle hawk BASE mode forward  (PATROL → HOVER → PURSUE)
 *   p              cycle hawk BASE mode backward (PATROL ← HOVER ← PURSUE)
 *   h / H          toggle auto-dive (every AUTO_DIVE_PERIOD seconds)
 *   . (period)     pause / resume
 *   r / R          reset (theme preserved)
 *   t              cycle to next theme; T cycles backward
 *   + / =          add BOID_STEP birds (cap N_BOIDS_MAX)
 *   -              remove BOID_STEP birds (floor N_BOIDS_MIN)
 */
/*
 * toggle_auto_dive — flip the auto-dive flag and reset the timer.
 * Resetting on toggle prevents "phantom dives" where the timer was
 * mid-period when the user disabled auto-dive previously.
 */
static void toggle_auto_dive(Hawk *h)
{
    h->auto_dive       = !h->auto_dive;
    h->auto_dive_timer = 0.0f;
}

/*
 * cycle_theme — advance theme_idx by `dir` (+1 = next, −1 = prev) wrapped
 * over [0, N_THEMES); re-register ncurses pairs so the new palette
 * takes effect immediately.
 */
static void cycle_theme(SimControls *sim, int dir)
{
    sim->theme_idx = (sim->theme_idx + dir + N_THEMES) % N_THEMES;
    theme_apply(sim->theme_idx);
}

/*
 * adjust_bird_count — slide the active prefix by `delta`, clamped to
 * [N_BOIDS_MIN, N_BOIDS_MAX].  Pool is pre-spawned at scene_init so
 * positive delta reveals existing birds instantly (no re-randomise).
 */
static void adjust_bird_count(Scene *sc, int delta)
{
    int next = sc->n_birds + delta;
    if (next < N_BOIDS_MIN) next = N_BOIDS_MIN;
    if (next > N_BOIDS_MAX) next = N_BOIDS_MAX;
    sc->n_birds = next;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    /* hawk DIVE overlay */
    case ' ':  hawk_dive(&sc->hawk, sc->flock_centroid);          break;

    /* hawk BASE mode cycle */
    case 'n':  hawk_cycle_base_mode(&sc->hawk, +1);               break;
    case 'p':  hawk_cycle_base_mode(&sc->hawk, -1);               break;

    /* auto-dive toggle */
    case 'h': case 'H':  toggle_auto_dive(&sc->hawk);             break;

    /* pause / reset / theme / bird count */
    case '.':            sc->sim.paused = !sc->sim.paused;        break;
    case 'r': case 'R':  scene_init(sc, app->screen.cols, app->screen.rows); break;
    case 't':            cycle_theme(&sc->sim, +1);               break;
    case 'T':            cycle_theme(&sc->sim, -1);               break;
    case '+': case '=':  adjust_bird_count(sc, +BOID_STEP);       break;
    case '-':            adjust_bird_count(sc, -BOID_STEP);       break;

    default: break;
    }
    return true;
}

/*
 * main — game loop.  Identical structure to the project framework:
 *   ① resize check → ② measure dt → ③ fixed-step accumulator →
 *   ④ fps counter  → ⑤ frame cap (sleep BEFORE render) →
 *   ⑥ draw + present → ⑦ drain input.
 *
 * The frame cap uses `clock_ns() − frame_start` (no `+ dt`) — adding
 * dt back into elapsed cancels the cap and pegs CPU at 100 %.
 */
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
    fps_counter_init(&app->fps);

    screen_init(&app->screen, 0 /* initial theme = Dusk */);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;       /* avalanche guard */
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;

    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;

    while (app->running) {
        int64_t frame_start = clock_ns();

        /* (1) handle pending resize: reload extent, reset timers */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* (2) measure dt since last frame, capped to avoid physics avalanche */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* (3) drain accumulator: fixed-step physics until caught up */
        const int64_t tick_ns = TICK_NS(app->sim_fps);
        const float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* (4) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt);

        /* (5) sleep BEFORE render so terminal I/O stays inside the budget */
        int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
        clock_sleep_ns(budget_left);

        /* (6) draw + present */
        screen_draw(&app->screen, &app->scene, app->fps.display, app->sim_fps);
        screen_present();

        /* (7) drain input */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
