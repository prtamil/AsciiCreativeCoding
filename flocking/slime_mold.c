/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * slime_mold.c — Physarum polycephalum (Slime Mold) Simulation
 *
 * Implements the Jeff Jones (2010) agent-based model of Physarum transport
 * networks.  Thousands of agents move on a 2D trail grid, sense the trail
 * ahead with three sensors, steer toward the highest concentration, move
 * one step, deposit trail, then the grid diffuses and decays.  The emergent
 * behaviour — self-organising tubular networks that approximate minimum
 * Steiner trees between food sources — requires no central control.
 *
 * Algorithm (per tick, per agent):
 *   1. SENSE   — sample trail at FL (front-left), F (front), FR (front-right)
 *                sensor positions, each SENSOR_DIST cells ahead at ±SENSOR_ANGLE
 *   2. ROTATE  — if F > FL and F > FR: keep heading
 *                if FL > FR: turn left by ROTATE_ANGLE
 *                if FR > FL: turn right by ROTATE_ANGLE
 *                if FL == FR (and > F): random ±ROTATE_ANGLE
 *   3. MOVE    — advance STEP_SIZE cells in heading direction (wrap screen)
 *   4. DEPOSIT — add DEPOSIT_AMT to trail at current cell
 *
 * Grid update (per tick, applied to all cells simultaneously):
 *   trail_new[r][c] = lerp(trail[r][c], avg_3x3(trail, r, c), DIFFUSE_W)
 *                     × (1 − DECAY_RATE)
 *
 * Food sources: 3 fixed positions (configurable per preset).
 *   Agents within FOOD_RADIUS cells deposit FOOD_BONUS × DEPOSIT_AMT.
 *   Food sources are displayed as bright '@' markers; the network of tubes
 *   connecting them is the "optimal Steiner tree" approximation.
 *
 * Presets:
 *   0  Scatter   — agents randomised; 3 food sources in a triangle
 *   1  Ring      — agents on a ring pointing inward; food at centre
 *   2  Clusters  — two dense clusters; food at opposite edges
 *   3  Mesh      — agents on a grid; 4 corner food sources
 *
 * Keys:
 *   q / ESC    quit
 *   p / space  pause / resume
 *   r          reset current preset
 *   n / N      next / previous preset
 *   t / T      next / previous theme
 *   + / -      more / fewer agents
 *   d / D      more / less diffusion
 *   e / E      faster / slower decay
 *   f          toggle food sources
 *   ] / [      sim FPS up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/slime_mold.c \
 *       -o slime_mold -lncurses -lm
 *
 * Section map:
 *   §1 config       — tunables (grid sizes, agent counts, sim/decay/diffuse,
 *                      preset/theme constants)
 *   §2 clock        — monotonic timer + sleep
 *   §3 color/theme  — Theme struct + 5 palettes; theme-independent
 *                      HUD pairs (canonical bright yellow + cyan)
 *   §4 trail grid   — TrailField (current + diffusion buffer + active
 *                      dims); trail_sample / _deposit / _update / _clear
 *                      / _resize / _draw, plus the inner kernel helper
 *                      box_average_3x3_wrapped (3×3 Laplacian)
 *   §5 agents       — Agent + FoodSrc + Crowd + SimControls + Screen +
 *                      Scene types; agent_step pipeline split into
 *                      agent_sense_three_sensors → agent_rotate_toward_
 *                      brightest → agent_move_and_wrap → agent_food_
 *                      bonus_multiplier; agents_step_all + food_draw
 *   §6 scene        — presets (Scatter / Ring / Clusters / Mesh) +
 *                      scene_init dispatcher; crowd_alloc;
 *                      agent_spawn_random
 *   §7 screen/HUD   — screen_init; canonical two-row hud_draw
 *                      (top=data, bottom=actions)
 *   §8 app          — FpsCounter, App; clampi/clampf; signals + resize;
 *                      key dispatch (cycle_preset / cycle_theme /
 *                      adjust_agent_count / adjust_diffuse /
 *                      adjust_decay / adjust_sim_fps + clamp ranges);
 *                      main game loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Agent-based simulation — emergent behaviour from local rules.
 *                  No global coordinator; network topology arises purely from
 *                  the sense→rotate→move→deposit loop repeated N_AGENTS times
 *                  per tick. The trail grid mediates indirect communication
 *                  (stigmergy): agents respond to paths left by earlier agents.
 *
 * Biology        : Models Physarum polycephalum (Jeff Jones, 2010).
 *                  Real slime mold spans food sources with tubular networks that
 *                  approximate minimum Steiner trees — surprisingly close to
 *                  optimal transport graphs.  The model captures this using only
 *                  three sensor readings and a random tie-break rule.
 *
 * Math           : Diffusion step is a lerp toward a 3×3 box average:
 *                    trail' = lerp(trail, avg_3×3(trail), DIFFUSE_W)
 *                  This is a discrete approximation of the heat equation
 *                  (∂u/∂t = D·∇²u).  DIFFUSE_W controls the effective
 *                  diffusion coefficient D.
 *                  Decay: trail' *= (1 − DECAY_RATE) per tick — exponential
 *                  fade without diffusion would give trail lifetime ≈ 1/DECAY.
 *
 * Performance    : O(N_AGENTS + W×H) per tick.  The trail grid update is the
 *                  bottleneck: 512×128 ≈ 65K cells × 9-neighbour sum per cell
 *                  ≈ 590K ops per tick.  Agents cost N_AGENTS × ~15 ops each.
 *
 * Data-structure : TrailField struct (§4) wraps two float arrays —
 *                  TrailField.current (live) and TrailField.buffer
 *                  (Jacobi workspace) — plus active_rows/cols tracking
 *                  the in-use portion.  Agents read and deposit into
 *                  `current`; trail_update reads `current` → writes
 *                  `buffer` → copies back.  This Jacobi pattern (vs
 *                  in-place Gauss-Seidel) keeps the update READ-ONLY
 *                  on `current` so the result is iteration-order
 *                  independent and free of asymmetric scan artefacts.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Canonical Physarum agent model ─────────────────────────────
 *   [1] Jones, J. (2010), "Characteristics of pattern formation and
 *       evolution in approximations of Physarum transport networks",
 *       Artificial Life 16(2), pp. 127-153 — THE source for this
 *       file's algorithm.  Specifies the 3-sensor agent (front-left
 *       / front / front-right), the abrupt ±ROTATE_ANGLE turning,
 *       and the per-tick 3×3 diffuse + exponential decay of the
 *       pheromone field.  §5 agent_step + §4 trail_update are
 *       direct implementations.
 *
 *   ── Real Physarum biology + optimal-transport experiments ──────
 *   [2] Nakagaki, T., Yamada, H. & Tóth, Á. (2000), "Maze-solving by
 *       an amoeboid organism", Nature 407(6803), p. 470 — the original
 *       experiment showing Physarum solves shortest-path mazes.
 *       Showed the world that this organism does graph algorithms
 *       on petri-dish substrate.
 *   [3] Tero, A., Takagi, S., Saigusa, T., Ito, K., Bebber, D. P.,
 *       Fricker, M. D., Yumiki, K., Kobayashi, R. & Nakagaki, T.
 *       (2010), "Rules for biologically inspired adaptive network
 *       design", Science 327(5964), pp. 439-442 — Physarum solving
 *       the Tokyo rail network problem.  Demonstrates the network
 *       topology this file's agents are reproducing in miniature.
 *   [4] Tero, A., Kobayashi, R. & Nakagaki, T. (2007), "A
 *       mathematical model for adaptive transport network in path
 *       finding by true slime mold", J. Theor. Biol. 244(4),
 *       pp. 553-564 — the continuous-PDE model of the same
 *       phenomenon (parallel to [1]'s agent-based formulation).
 *
 *   ── Mathematical foundation: reaction-diffusion ────────────────
 *   [5] Turing, A. M. (1952), "The chemical basis of morphogenesis",
 *       Phil. Trans. R. Soc. B 237(641), pp. 37-72 — the seminal
 *       paper on reaction-diffusion pattern formation.  §4
 *       trail_update is a discrete reaction-diffusion update: agent
 *       deposit = source, diffuse + decay = the standard
 *       Laplacian + sink, on a coarse grid.
 *
 *   ── Stigmergy: indirect communication via environment ──────────
 *   [6] Grassé, P.-P. (1959), "La reconstruction du nid et les
 *       coordinations interindividuelles chez Bellicositermes
 *       natalensis et Cubitermes sp.", Insectes Sociaux 6, pp. 41-83
 *       — coined "stigmergy".  Termites coordinate nest-building
 *       solely through pheromone traces in the environment — exactly
 *       the mechanism this file's agents use.  No central planner,
 *       no agent-to-agent messaging.
 *
 *   ── Book-length treatments ─────────────────────────────────────
 *   [7] Adamatzky, A. (2010), "Physarum Machines: Computers from
 *       Slime Mould", World Scientific — the canonical reference
 *       book on slime-mould unconventional computing.  Covers
 *       biology, the agent model in [1], wet-lab experiments, and
 *       sensor / actuator designs that turn Physarum into a
 *       general-purpose substrate.
 *
 *   ── Game-loop / fixed-step physics ─────────────────────────────
 *   [8] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       fixed-step accumulator + dt-cap pattern in §8 main().
 *       Slime mold uses a single fixed sim_fps without sub-tick
 *       interpolation (the trail field is already smooth), but the
 *       same dt-cap-at-100 ms avalanche guard is from Fiedler.
 *
 *   ── Implementation-oriented references ─────────────────────────
 *   [9] Shiffman, D., "The Nature of Code", Ch. 5 (Autonomous Agents)
 *       — reading-level introduction to sense-decide-act agent loops
 *       in Processing.  Pair with [1] for the slime-mould specifics.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [10] Sage Jenson's "Physarum" interactive — https://sagejenson.com/physarum
 *       — interactive web demo of the same Jones [1] model with live
 *       parameter sliders.  The canonical "see the algorithm move"
 *       reference; pair with [1] for the math and this file for the
 *       C-on-ncurses implementation.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     flocking/flocking.c    — Reynolds-style boids; agent-based but
 *       with PEER interactions (alignment, cohesion) rather than
 *       SUBSTRATE-mediated stigmergy.
 *     flocking/murmuration.c — large-N (1500) flock with density-
 *       field rendering and spatial hashing.  Similar
 *       "render-the-field-not-the-agents" rendering philosophy.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Thousands of agents wander a 2-D grid, each leaving a TRAIL.
 * They prefer to walk where the trail is strongest (positive
 * feedback).  The trail diffuses outward and decays over time
 * (negative feedback).  The interplay of these two forces
 * produces self-organising tube networks that connect food
 * sources — without ANY central planner.
 *
 *
 * STIGMERGY
 * ─────────
 * "Stigmergy" (Pierre-Paul Grassé, 1959) = indirect
 * communication via persistent environmental modifications.
 * No agent talks to another agent.  Communication happens
 * through the ENVIRONMENT (the trail grid).  This is the
 * same mechanism real ant colonies, termite mound builders,
 * and Physarum slime molds use.
 *
 * The remarkable result: stigmergic colonies can COMPUTE.
 * Real Physarum slime molds connect oat-flake food sources
 * with tubes that approximate the MINIMUM STEINER TREE
 * (the optimal pipe network).  This file reproduces that
 * computation in software.
 *
 * ALGORITHM IN STEPS  (per tick)
 * ──────────────────────────────
 *  AGENT LOOP — agents_step_all calls agent_step for each agent:
 *    1. SENSE   — agent_sense_three_sensors → SensorReadings
 *                 sample trail at 3 sensor positions:
 *                   FL = pos + dir(heading − SENSOR_ANGLE) · SENSOR_DIST
 *                   F  = pos + dir(heading                ) · SENSOR_DIST
 *                   FR = pos + dir(heading + SENSOR_ANGLE) · SENSOR_DIST
 *    2. ROTATE  — agent_rotate_toward_brightest
 *                 F greatest      → no turn
 *                 FL > FR         → turn LEFT by ROTATE_ANGLE
 *                 FR > FL         → turn RIGHT by ROTATE_ANGLE
 *                 FL == FR > F    → random turn (tie-break)
 *    3. MOVE    — agent_move_and_wrap
 *                 pos += STEP_SIZE · (cos heading, sin heading)
 *                 toroidal wrap to trail->active_rows/cols
 *    4. DEPOSIT — bonus = agent_food_bonus_multiplier (FOOD_BONUS
 *                          if within FOOD_RADIUS of any food source,
 *                          else 1.0)
 *                 trail_deposit(trail, pos, sim.deposit · bonus)
 *
 *  GRID LOOP — trail_update over active_rows × active_cols:
 *    For each (r, c):
 *      neighbour_avg     = box_average_3x3_wrapped(trail, r, c)
 *      after_diffusion   = lerp(current[r][c], neighbour_avg, diffuse_w)
 *      after_decay       = after_diffusion · (1 − decay)
 *      buffer[r][c]      = max(after_decay, 0)
 *    Jacobi swap: current ← buffer (whole-grid memcpy)
 *
 * KEY FORMULAS
 * ────────────
 *   Diffusion: trail' = lerp(trail, neighbours_avg, w)
 *              ≈ discretised heat equation ∂u/∂t = D · ∇²u
 *
 *   Decay:     trail' = trail · (1 - DECAY)
 *              exponential fade, time constant ≈ 1/DECAY
 *
 *   Sensor:    sample at (pos + dir · SENSOR_DIST) for 3 angles
 *
 *   Steady-state trail strength at deposit point ≈
 *      DEPOSIT_AMT / DECAY    (deposit rate / decay rate)
 *
 *
 * Background you need
 * ───────────────────
 *   - Agent-based simulation (flocking.c if new).
 *   - 2-D grid as a scalar field with diffusion.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Real cell biology (mitosis, signalling).  Slime mold
 *     biology is the INSPIRATION, not the simulation target.
 *   - Steiner-tree algorithms (Kruskal, Prim).  Slime mold
 *     APPROXIMATES Steiner; we don't compute it analytically.
 *   - PDE solvers.  The diffusion is a 3×3 box-blur, not a
 *     proper diffusion solver.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

#define ROWS_MAX        128
#define COLS_MAX        512

/* Agent population */
#define N_AGENTS_DEF   2000  /* default agent count; 2000 fills ~200×60 grid visually */
#define N_AGENTS_MIN    200  /* below this, tubes fail to form (too sparse)            */
#define N_AGENTS_MAX   6000  /* above this, cost > 30fps on typical hardware           */
#define N_AGENTS_STEP   200

/* Physarum sensor parameters (Jones 2010) */
#define SENSOR_ANGLE  ((float)M_PI / 4.0f)   /* ±45° from heading; Jones found 45°
                                               * gives crisp tubes without over-steering */
#define SENSOR_DIST    4.0f                   /* cells ahead; smaller→tighter curves,
                                               * larger→straighter long-range tubes      */
#define ROTATE_ANGLE  ((float)M_PI / 4.0f)   /* 45° abrupt turn per step — discrete
                                               * steering; fractional angles give blurry
                                               * diffuse blobs instead of tubes          */
#define STEP_SIZE      1.0f                   /* cells moved per tick (1 = one cell)    */

/* Trail parameters */
#define DEPOSIT_DEF    5.0f    /* trail concentration added per agent tick; scales
                                * with FOOD_BONUS near food sources                    */
#define MAX_TRAIL     100.0f   /* saturation ceiling; prevents overflow and keeps
                                * concentration mapping in [0,100] for display         */
#define DECAY_DEF      0.08f   /* 8% removed per tick → trail lifetime ≈ 1/0.08 = 12.5
                                * ticks at 30fps ≈ 0.4s half-life without diffusion    */
#define DIFFUSE_DEF    0.35f   /* lerp weight toward 3×3 average; higher→smoother but
                                * more blurry network (tubes lose sharp boundaries)    */

/* Food sources */
#define N_FOOD          3
#define FOOD_RADIUS     3.0f   /* detection radius in cells (about 3 character widths) */
#define FOOD_BONUS      6.0f   /* ×6 deposit near food → strong attractor gradient     */
#define FOOD_MIN_TRAIL 30.0f   /* floor concentration at food cells; prevents food
                                * sites from fading after most agents move away        */

/* Simulation */
#define SIM_FPS_DEF    30
#define SIM_FPS_MIN     5
#define SIM_FPS_MAX    60
#define SIM_FPS_STEP    5

#define N_PRESETS       4
#define N_THEMES        5

#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Colour pairs:
 *   1..5  trail intensity ramp (dim → bright), theme-controlled
 *   6     food, theme-controlled
 *   7     HUD top row    — canonical bright yellow, theme-INDEPENDENT
 *   8     HUD hint row   — canonical bright cyan,   theme-INDEPENDENT
 *
 * The trail is rendered with 5 characters × 5 colours.  The two HUD
 * pairs are set ONCE in color_init() to the project-standard bright
 * yellow / bright cyan from CLAUDE.md §"HUD Standard" so the status
 * row stays legible against any theme's background.
 */
enum {
    CP_T1 = 1,  /* dimmest trail  */
    CP_T2 = 2,
    CP_T3 = 3,
    CP_T4 = 4,
    CP_T5 = 5,  /* brightest trail */
    CP_FOOD = 6,
    CP_HUD  = 7,
    CP_HINT = 8,
};

/*
 * Theme — one named colour palette for the trail intensity ramp + food.
 *
 * Intent
 *   The visual character of the simulation is dominated by the FIVE
 *   trail intensity colours plus the food marker.  Different themes
 *   make the same algorithm read as a yellow slime-mold network, a
 *   cool cyan one, a fiery red one, etc.  Bundling all theme-dependent
 *   colours into one struct lets `theme_apply()` re-register six ncurses
 *   pairs in one call when the user cycles themes with t / T.
 *
 * Why two parallel palettes (t[] vs t8[])
 *   xterm-256 terminals get fine-grained gradients (5 distinct steps
 *   of a single hue band).  8-colour terminals can't; t8[] picks the
 *   closest ANSI primary, accepting that the dim→bright gradient
 *   collapses into a smaller set of equivalence classes.
 *
 * Why NOT include HUD / hint colours
 *   PAIR_HUD and PAIR_HINT are theme-INDEPENDENT (canonical bright
 *   yellow + bright cyan per CLAUDE.md §"HUD Standard") so the status
 *   row stays legible regardless of which theme is active.  Those
 *   pairs are set ONCE in color_init() and survive every theme cycle.
 *
 * Members
 *   t[5]    five xterm-256 foreground indices, dim → bright.
 *           Bird-cell intensity bucket maps directly: density 1 → t[0],
 *           density 2 → t[1], ..., density 5+ → t[4].
 *   t8[5]   five ANSI primary fallbacks for 8-colour terminals.
 *   food    food-marker foreground (xterm-256).
 *   food8   food-marker foreground (ANSI 8).
 *   name    HUD label ("Physarum", "Cyan", "Neon", "Forest", "Lava").
 *
 * Invariants
 *   All indices in [0, 255] (256-colour) or in COLOR_BLACK..COLOR_WHITE
 *   (8-colour).  name != NULL.
 *
 * References
 *   None directly — colour palette design is project-specific.  The
 *   five-step intensity gradient mirrors the glyph ramp ".+x#@" used
 *   in §4 trail_char_pair; both encode "how strong is the pheromone
 *   here" via parallel character + colour channels.  See CLAUDE.md
 *   §"Theme Palette Brightness" for the brightness-floor rule.
 */
typedef struct {
    short t[5];        /* trail pair fg colours, dim→bright (256-color) */
    short t8[5];       /* 8-color fallbacks                             */
    short food;        /* food-marker fg, xterm-256                     */
    short food8;       /* food-marker fg, ANSI 8                        */
    const char *name;  /* HUD label                                     */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 Physarum — yellow/amber network on black, like real slime mold */
    { {22, 58, 100, 136, 220},
      {COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
      196, COLOR_RED, "Physarum" },
    /* 1 Cyan — cold network */
    { {17, 19, 27, 39, 87},
      {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
      226, COLOR_YELLOW, "Cyan" },
    /* 2 Neon — magenta/violet */
    { {53, 91, 129, 165, 207},
      {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE},
      226, COLOR_YELLOW, "Neon" },
    /* 3 Forest — organic green */
    { {22, 28, 34, 40, 118},
      {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE},
      196, COLOR_RED, "Forest" },
    /* 4 Lava — red/orange */
    { {52, 88, 124, 166, 226},
      {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
      231, COLOR_WHITE, "Lava" },
};

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    for (int i = 0; i < 5; i++) {
        if (COLORS >= 256)
            init_pair(i + 1, th->t[i],  -1);
        else
            init_pair(i + 1, th->t8[i], -1);
    }
    if (COLORS >= 256) init_pair(CP_FOOD, th->food,  -1);
    else               init_pair(CP_FOOD, th->food8, -1);

    /* CP_HUD + CP_HINT are theme-INDEPENDENT — see color_init. */
}

static void color_init(void)
{
    enum { HUD_YELLOW_256 = 226, HUD_CYAN_256 = 51 };

    start_color();
    use_default_colors();

    /* Theme-independent HUD pairs (CLAUDE.md §"HUD Standard"):
     *   PAIR_HUD  — bright yellow on default bg, used for top status row
     *   PAIR_HINT — bright cyan   on default bg, used for bottom key hint
     * Set ONCE here so they survive every theme cycle. */
    if (COLORS >= 256) {
        init_pair(CP_HUD,  HUD_YELLOW_256, -1);
        init_pair(CP_HINT, HUD_CYAN_256,   -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW,   -1);
        init_pair(CP_HINT, COLOR_CYAN,     -1);
    }

    theme_apply(0);
}

/* ===================================================================== */
/* §4  trail grid                                                         */
/* ===================================================================== */

/*
 * TrailField — the pheromone scalar field that the slime mold lives in.
 *
 * Intent
 *   Physarum's macroscopic behaviour emerges from a tiny rule: every
 *   agent SENSES the field at three points ahead, ROTATES toward the
 *   brightest, MOVES one step, then DEPOSITS pheromone at its new
 *   position.  The field itself DIFFUSES (3×3 box average, weighted)
 *   and DECAYS each tick.  This struct owns both halves of that field
 *   evolution — the live grid and the diffusion workspace.
 *
 * Why TWO grids (current + buffer)
 *   The diffusion convolution reads every cell's 8 neighbours.  If we
 *   wrote results back into the same array, cells later in the scan
 *   would see ALREADY-updated values from earlier cells — a Gauss-
 *   Seidel style update that depends on iteration order and produces
 *   asymmetric artefacts.  Writing into a separate buffer + copying
 *   back (Jacobi) keeps the update READ-ONLY on the current state
 *   and gives an order-independent result.
 *
 * Why fixed-size arrays (not malloc'd)
 *   ROWS_MAX × COLS_MAX × 4 bytes × 2 buffers ≈ 512 KB.  Static BSS
 *   allocation means no per-resize malloc, and the cache hierarchy
 *   sees a stable address for the hot inner loop.  active_rows /
 *   active_cols track the in-use portion (terminal-sized) so we don't
 *   waste cycles updating cells that won't be rendered.
 *
 * Members
 *   current[r][c]   live pheromone concentration in cell (r, c).
 *                   Read by agent sensors + the renderer; written by
 *                   agent deposits + the diffuse/decay step.
 *   buffer [r][c]   diffusion workspace — never observed by agents
 *                   or renderer.  Populated by trail_update, then
 *                   copied back to `current`.
 *   active_rows     number of rows actually in use (terminal rows − 1,
 *                   reserving the bottom row for the HUD).  Cells
 *                   beyond this are unused and left as zeros.
 *   active_cols     number of columns actually in use (= terminal cols).
 *
 * Invariants
 *   0 ≤ active_rows ≤ ROWS_MAX,  0 ≤ active_cols ≤ COLS_MAX.
 *   current[r][c] ∈ [0, MAX_TRAIL] after trail_deposit clamps.
 *   buffer[r][c]  ≥ 0 after trail_update clamps.
 *
 * References
 *   [1] Jones, J. (2010), "Characteristics of pattern formation and
 *       evolution in approximations of Physarum transport networks",
 *       Artificial Life 16(2), pp. 127-153 — the diffuse + decay rule
 *       used here, with the same 3×3 box average.
 */
typedef struct {
    float current[ROWS_MAX][COLS_MAX];
    float buffer [ROWS_MAX][COLS_MAX];
    int   active_rows;
    int   active_cols;
} TrailField;

/* Single file-scope instance — 512 KB of BSS, allocated once at
 * startup.  Scene.trail points at this so a future split-screen
 * variant could allocate per-Scene fields. */
static TrailField g_trail_field;

/* Toroidal wrap helpers on the field's active dimensions. */
static inline int wrap_r_on(const TrailField *t, int r) {
    return (r % t->active_rows + t->active_rows) % t->active_rows;
}
static inline int wrap_c_on(const TrailField *t, int c) {
    return (c % t->active_cols + t->active_cols) % t->active_cols;
}

/* trail_sample — read concentration at a float (column, row) position,
 * toroidally wrapped.  Used by agent sensors. */
static float trail_sample(const TrailField *t, float fc, float fr)
{
    int c = wrap_c_on(t, (int)(fc + 0.5f));
    int r = wrap_r_on(t, (int)(fr + 0.5f));
    return t->current[r][c];
}

/* trail_deposit — add `amount` of pheromone at a float (column, row),
 * clamped to MAX_TRAIL.  Used by agents after their move step. */
static void trail_deposit(TrailField *t, float fc, float fr, float amount)
{
    int c = wrap_c_on(t, (int)(fc + 0.5f));
    int r = wrap_r_on(t, (int)(fr + 0.5f));
    t->current[r][c] += amount;
    if (t->current[r][c] > MAX_TRAIL) t->current[r][c] = MAX_TRAIL;
}

/*
 * box_average_3x3_wrapped — mean of a cell's nine-neighbour 3×3
 * patch (including itself), with toroidal wrap on the field's
 * active extent.  This is the spatial half of the diffusion update:
 * the standard discrete Laplacian + identity kernel divided by 9.
 *
 *   kernel = (1/9) · | 1 1 1 |
 *                    | 1 1 1 |
 *                    | 1 1 1 |
 */
static float box_average_3x3_wrapped(const TrailField *t, int r, int c)
{
    enum { KERNEL_HALF = 1 };                /* 3×3 has radius 1 cell */
    const float KERNEL_CELL_COUNT = 9.0f;    /* (2·HALF+1)^2          */

    float sum = 0.0f;
    for (int dr = -KERNEL_HALF; dr <= KERNEL_HALF; dr++)
        for (int dc = -KERNEL_HALF; dc <= KERNEL_HALF; dc++)
            sum += t->current[wrap_r_on(t, r+dr)][wrap_c_on(t, c+dc)];
    return sum / KERNEL_CELL_COUNT;
}

/*
 * trail_update — diffuse + decay one tick.
 *
 *   for each (r, c):
 *     neighbour_avg     = mean of 3×3 neighbours (toroidal)
 *     after_diffusion   = lerp(current, neighbour_avg, diffuse_w)
 *     after_decay       = after_diffusion · (1 − decay)
 *     buffer[r][c]      = max(after_decay, 0)
 *   current ← buffer    (Jacobi swap: order-independent)
 *
 * The lerp + scalar decay is the discretised reaction-diffusion PDE
 * (Turing [5]):  ∂u/∂t = D·∇²u − k·u, with D ∝ diffuse_w and k ∝
 * decay.  Jacobi writes guarantee every cell sees the SAME source
 * state, so the result doesn't depend on iteration order.
 */
static void trail_update(TrailField *t, float diffuse_w, float decay)
{
    const float retain = 1.0f - decay;
    int   R = t->active_rows;
    int   C = t->active_cols;

    for (int r = 0; r < R; r++) {
        for (int c = 0; c < C; c++) {
            float neighbour_avg   = box_average_3x3_wrapped(t, r, c);
            float self            = t->current[r][c];

            /* spatial step: lerp self → neighbour_avg by diffuse_w */
            float after_diffusion = self * (1.0f - diffuse_w)
                                  + neighbour_avg * diffuse_w;

            /* temporal step: exponential decay (one tick) */
            float after_decay     = after_diffusion * retain;

            /* clamp negative floats (rounding-safety; should not occur) */
            t->buffer[r][c] = (after_decay > 0.0f) ? after_decay : 0.0f;
        }
    }

    /* Jacobi swap: copy buffer → current */
    memcpy(t->current, t->buffer, sizeof t->current);
}

/* trail_clear — zero both grids and reset the active extent. */
static void trail_clear(TrailField *t)
{
    memset(t->current, 0, sizeof t->current);
    memset(t->buffer,  0, sizeof t->buffer);
}

/* trail_resize — set the active grid extent (called from scene_init
 * after the terminal size is known).  The actual buffers are static. */
static void trail_resize(TrailField *t, int rows, int cols)
{
    if (rows > ROWS_MAX) rows = ROWS_MAX;
    if (cols > COLS_MAX) cols = COLS_MAX;
    t->active_rows = rows;
    t->active_cols = cols;
}

/*
 * trail_char_pair — map concentration to (character, color_pair).
 *
 * Five intensity levels with distinct characters so the network structure
 * is legible even on 8-color terminals:
 *   0.3–2   '.'  dim      (faint trace)
 *   2–8     '+'  medium   (moderate concentration)
 *   8–20    'x'  high
 *  20–50    '#'  dense    (major tube)
 *  50+      '@'  saturated (hub / intersection)
 */
static void trail_char_pair(float v, chtype *ch_out, int *cp_out)
{
    if      (v < 0.3f)  { *ch_out = ' '; *cp_out = CP_T1; }
    else if (v < 2.0f)  { *ch_out = '.'; *cp_out = CP_T1; }
    else if (v < 8.0f)  { *ch_out = '+'; *cp_out = CP_T2; }
    else if (v < 20.0f) { *ch_out = 'x'; *cp_out = CP_T3; }
    else if (v < 50.0f) { *ch_out = '#'; *cp_out = CP_T4; }
    else                { *ch_out = '@'; *cp_out = CP_T5; }
}

static void trail_draw(const TrailField *t, int rows, int cols)
{
    int max_r = (rows < t->active_rows) ? rows : t->active_rows;
    int max_c = (cols < t->active_cols) ? cols : t->active_cols;

    for (int r = 0; r < max_r; r++) {
        for (int c = 0; c < max_c; c++) {
            chtype ch; int cp;
            trail_char_pair(t->current[r][c], &ch, &cp);
            if (ch == ' ') continue;  /* skip background cells */
            attron(COLOR_PAIR(cp));
            mvaddch(r, c, ch);
            attroff(COLOR_PAIR(cp));
        }
    }
}

/* ===================================================================== */
/* §5  agents                                                             */
/* ===================================================================== */

/*
 * Agent — one Physarum-style particle (Jones [1]).
 *
 * Intent
 *   The atomic unit of the simulation.  Each tick every agent performs
 *   the sense-rotate-move-deposit cycle from Jones 2010 [1]:
 *
 *     1. SENSE   — sample the trail field at three forward points
 *                  (front-left, front, front-right) using cos/sin of
 *                  the current angle and SENSOR_DIST / SENSOR_ANGLE.
 *     2. ROTATE  — pick the brightest sensor; rotate the heading by
 *                  ±ROTATE_ANGLE toward that side, or keep heading if
 *                  the centre sensor wins.
 *     3. MOVE    — advance pos by STEP_SIZE in the new heading
 *                  direction; toroidal wrap to the grid extent.
 *     4. DEPOSIT — add `deposit` pheromone at the new pos (multiplied
 *                  by FOOD_BONUS if within FOOD_RADIUS of any source).
 *
 *   The agent has NO peer interactions — neighbour agents are not
 *   sampled directly.  All communication happens through the trail
 *   field: agents read the field, deposit into the field, and the
 *   field diffuses + decays.  This is STIGMERGY (Grassé [6]) — same
 *   mechanism real termites use to coordinate nest building.
 *
 * Why no mass / no velocity / no acceleration
 *   Physarum agents are kinematic, not dynamic.  Speed is a hard
 *   constant STEP_SIZE; turning is an INSTANT ±ROTATE_ANGLE snap.
 *   This produces the CRISP tube morphology characteristic of real
 *   slime-mould networks — adding smoothing forces would dampen the
 *   sharp branches into a fuzzy blob.
 *
 * Why FLOAT (x, y) on a cell grid
 *   The trail field is a discrete grid (rows × cols of float), but
 *   agents move in CONTINUOUS cell coords.  Float positions let the
 *   deposit step land between cells (via wrap_*_on's round-to-nearest)
 *   so the trail receives fractional contributions — which smooths
 *   the diffusion at low agent counts and avoids cell-quantisation
 *   "checkerboard" artefacts.
 *
 * Members
 *   x, y       Position in cell coordinates (float; range [0,
 *              active_cols) × [0, active_rows)).  Toroidally wrapped
 *              by agent_step at the end of the move step.
 *   angle      Heading in RADIANS.  Sensor positions are
 *              (cos angle, sin angle) × SENSOR_DIST; rotation snaps
 *              by ±ROTATE_ANGLE (no smoothing).  Range unbounded
 *              (atan2/cos/sin handle the wrap).
 *
 * Invariants
 *   0 ≤ x < trail->active_cols, 0 ≤ y < trail->active_rows after
 *   agent_step's wrap loop.
 *   angle has no enforced range — cos/sin are 2π-periodic so any
 *   real value is well-defined.
 *
 * References
 *   [1] Jones 2010 — the canonical agent model implemented here.
 *   [6] Grassé 1959 — the stigmergy mechanism that gives agents their
 *       "intelligence" without peer communication.
 *   [9] Shiffman, *Nature of Code* Ch. 5 — same sense-decide-act
 *       agent loop in Processing.
 */
typedef struct {
    float x, y;     /* cell-space position (float for sub-cell deposit) */
    float angle;    /* heading in radians                                */
} Agent;

/*
 * FoodSrc — one fixed food source.
 *
 * Intent
 *   A static gradient sink that the slime-mould network LEARNS to
 *   connect.  Two roles:
 *
 *     1. agent_step reads food positions to apply a FOOD_BONUS
 *        multiplier on `deposit` when the agent is within
 *        FOOD_RADIUS of a source — this creates a stronger trail
 *        near food, which feeds back via the sensors to attract more
 *        agents (positive-feedback loop = the "exploit" half of the
 *        explore/exploit dynamic).
 *
 *     2. agents_step_all keeps the trail cells AT food positions
 *        above FOOD_MIN_TRAIL — without this floor, decay eventually
 *        drains the source's pheromone to ~0 and agents lose the
 *        gradient they should be following.
 *
 *     3. food_draw paints '@' markers in PAIR_FOOD so the user can
 *        SEE which points the network is connecting.
 *
 * Why a separate struct (not just Vec2-style)
 *   Future food sources might gain attributes — strength multiplier,
 *   activation time, food-type identity for typed-trail experiments.
 *   The named type leaves room without churn.  Currently has only
 *   (x, y) but the type lives on its own line for that future-
 *   proofing.
 *
 * Members
 *   x, y   Cell-coordinate position; set by the active preset
 *          (preset_scatter / _ring / _clusters / _mesh).
 *
 * References
 *   [3] Tero et al. 2010 — Physarum solving the Tokyo rail problem;
 *       the food sources here are this file's analogue of the rail
 *       stations the slime mould must connect.
 *   [4] Tero, Kobayashi & Nakagaki 2007 — continuous-PDE model where
 *       food sources are boundary conditions for the flow PDE.
 */
typedef struct { float x, y; } FoodSrc;

/*
 * Crowd — the agent pool: a heap-allocated Agent array plus its
 * active count.
 *
 * Intent
 *   Bundles the count with the pointer so every helper that operates
 *   on agents takes ONE struct pointer instead of two scalars.
 *   `count` matches the user-tunable agent total (adjusted by +/-
 *   keys via adjust_agent_count, which clamps to [N_AGENTS_MIN,
 *   N_AGENTS_MAX] and reallocs via crowd_alloc).
 *
 * Why heap-allocated (not a fixed-size array)
 *   N_AGENTS_MAX is 6000 → 6000 × sizeof(Agent) = 72 KB.  Static
 *   allocation would tie the binary's BSS footprint to the maximum,
 *   even at the default N_AGENTS_DEF = 2000.  malloc'ing at the
 *   actual size keeps the working set tight.  The pool is
 *   reallocated only when the user adjusts count or the scene resets.
 *
 * Members
 *   agents  Pointer to malloc'd Agent array of `count` elements.
 *           Freed by do_cleanup at exit.
 *   count   Number of active agents in [N_AGENTS_MIN, N_AGENTS_MAX];
 *           adjust_agent_count clamps the range.
 *
 * Invariants
 *   agents != NULL after scene_init.
 *   N_AGENTS_MIN ≤ count ≤ N_AGENTS_MAX.
 *   agents[i] for i ∈ [0, count) is valid.
 *
 * References
 *   None directly — the agent-pool data layout is standard ABM
 *   practice (every Jones-style implementation uses an array of
 *   agent records).
 */
typedef struct {
    Agent *agents;     /* malloc'd; sized by `count`              */
    int    count;      /* active count; user-tunable via +/- keys */
} Crowd;

/*
 * SimControls — every user-tunable knob in one struct.
 *
 * Intent
 *   The HUD reads these for display; app_handle_key writes them on
 *   keypress; the tick reads `paused` + `deposit`/`decay`/`diffuse`
 *   + `food_on`.  Bundling them keeps the Scene's top level free for
 *   actual simulation state and the key handler from reaching into
 *   many scattered globals.
 *
 * Members
 *   preset    initial-condition layout: 0=Scatter, 1=Ring, 2=Clusters,
 *             3=Mesh (n / N keys cycle).
 *   theme     active colour palette index in [0, N_THEMES); t / T cycle.
 *   sim_fps   target physics rate; live-adjustable with [ / ].
 *   paused    true → skip tick; HUD shows "PAUSED".
 *   food_on   true → food-proximity bonus deposit + food markers on
 *             screen.  f / F toggles.
 *   deposit   trail amount left by each agent step (default DEPOSIT_DEF).
 *   decay     per-tick exponential decay of every trail cell.
 *   diffuse   per-tick weight of the 3×3 box average.
 *
 * Invariants
 *   0 ≤ preset < N_PRESETS, 0 ≤ theme < N_THEMES.
 *   sim_fps ∈ [SIM_FPS_MIN, SIM_FPS_MAX].
 *   deposit > 0, decay ∈ [0.01, 0.30], diffuse ∈ [0.05, 0.90]
 *   (clamped in app_handle_key on every adjustment).
 *
 * References
 *   [1] Jones 2010 — defines the three knobs that drive Physarum
 *       morphology: deposit amount per step, exponential decay rate,
 *       and diffusion weight.  Jones shows that varying ONLY these
 *       three (holding sensor + step constants fixed) sweeps the
 *       network through fragmented / branching / mesh / flooded
 *       regimes.  The d/D/e/E/D/D keys let the user explore that
 *       parameter space LIVE.
 */
typedef struct {
    int   preset;
    int   theme;
    int   sim_fps;
    bool  paused;
    bool  food_on;
    float deposit;
    float decay;
    float diffuse;
} SimControls;

/*
 * Screen — terminal cell extent.
 *
 * Intent
 *   Owns the TERMINAL side of the simulation.  Where TrailField tracks
 *   the active GRID extent (= cols × rows-1, reserving the bottom for
 *   the HUD), Screen tracks the FULL terminal extent — used by:
 *     - the renderer's bounds checks (cell-space)
 *     - the HUD layout (top row = 0; bottom row = rows - 1)
 *     - SIGWINCH handling (which re-reads getmaxyx into these fields)
 *
 *   The TWO extents are deliberately separate: trail.active_rows =
 *   screen.rows − 1 (HUD reserves the bottom row).  scene_init's
 *   trail_resize() bridges the two.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Resize is a CLEAR state change: app_do_resize reads new
 *     dimensions into Scene.screen, then re-inits the scene.
 *     Bundling makes that one assignment (`scene.screen = ...`)
 *     rather than two scattered field writes.
 *   • Consistent with the project's other files (flocking.c, crowd.c,
 *     murmuration.c, shepherd.c), which all use a Screen sub-struct.
 *
 * Members
 *   cols   Terminal width in CELLS (= getmaxyx columns).  Active grid
 *          spans columns [0, cols).
 *   rows   Terminal height in CELLS.  Active grid spans rows [0,
 *          rows−1); the bottom row (rows−1) is the HUD action strip.
 *
 * Invariants
 *   cols > 0 AND rows ≥ 2 (need at least one grid row plus the HUD).
 *   trail.active_cols == cols  AND  trail.active_rows == rows − 1
 *   after scene_init.
 *
 * References
 *   None directly — terminal extent is a rendering substrate concern.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── sim        : SimControls   ← preset/theme/paused/food/...
 *       ├── crowd      : Crowd         ← Agent pool + active count
 *       ├── food[]     : FoodSrc[]     ← N_FOOD sources placed by preset
 *       ├── trail      : TrailField *  ← points at g_trail_field
 *       └── screen     : Screen        ← terminal cell extent
 *
 *   Every persistent simulation value is reachable from one Scene*.
 *   Signal flags live on App in §8 because handlers can't take ctx.
 *
 * Why TrailField as a pointer (not embedded)
 *   sizeof(TrailField) ≈ 512 KB.  Embedding it on Scene would make
 *   every Scene-by-value copy and stack-local Scene impractical.
 *   The pointer is set once at startup to &g_trail_field.
 *
 * Why a Scene-rooted hierarchy (not file-scope globals)
 *   • Replaceability: a future split-screen or A/B-comparison demo
 *     allocates multiple Scenes — impossible with globals.
 *   • Testability: scene_tick becomes a pure transformation Scene*→
 *     Scene* with no hidden inputs.
 *   • Reading aid: every helper begins with `Scene *s` (or const),
 *     so there's ONE place to look for "what state exists".
 *
 * References
 *   [1] Jones 2010 — the simulation owns the agent vector + the
 *       pheromone field; this Scene layout mirrors that reference
 *       design.
 *   [9] Shiffman, *Nature of Code* — same single-World pattern in
 *       Processing pseudocode for autonomous-agent demos.
 */
typedef struct {
    SimControls  sim;
    Crowd        crowd;
    FoodSrc      food[N_FOOD];
    TrailField  *trail;
    Screen       screen;
} Scene;

/*
 * SensorReadings — the three forward trail samples one agent observes.
 *
 *   front_left  : sensor at heading − SENSOR_ANGLE
 *   front       : sensor straight ahead
 *   front_right : sensor at heading + SENSOR_ANGLE
 *
 * Bundling lets agent_sense_three_sensors return all three by value
 * and agent_rotate_toward_brightest take them as one struct.
 */
typedef struct {
    float front_left;
    float front;
    float front_right;
} SensorReadings;

/*
 * agent_sense_three_sensors — SENSE step.
 *
 * Sample the trail at the agent's three forward sensor offsets.
 * Sensors sit on a circle of radius SENSOR_DIST in front of the
 * agent, at angles ±SENSOR_ANGLE from the heading (cone half-width
 * = SENSOR_ANGLE).  Jones [1] §III.
 */
static SensorReadings agent_sense_three_sensors(const Agent *a,
                                                const TrailField *trail)
{
    float left_x  = a->x + cosf(a->angle - SENSOR_ANGLE) * SENSOR_DIST;
    float left_y  = a->y + sinf(a->angle - SENSOR_ANGLE) * SENSOR_DIST;
    float fwd_x   = a->x + cosf(a->angle               ) * SENSOR_DIST;
    float fwd_y   = a->y + sinf(a->angle               ) * SENSOR_DIST;
    float right_x = a->x + cosf(a->angle + SENSOR_ANGLE) * SENSOR_DIST;
    float right_y = a->y + sinf(a->angle + SENSOR_ANGLE) * SENSOR_DIST;

    return (SensorReadings){
        .front_left  = trail_sample(trail, left_x,  left_y ),
        .front       = trail_sample(trail, fwd_x,   fwd_y  ),
        .front_right = trail_sample(trail, right_x, right_y),
    };
}

/*
 * agent_rotate_toward_brightest — ROTATE step.
 *
 *     centre wins → keep heading
 *     left   wins → turn −ROTATE_ANGLE
 *     right  wins → turn +ROTATE_ANGLE
 *     tie    → random ±ROTATE_ANGLE jitter
 *
 * Note the asymmetric "≥" on the centre check: the agent keeps
 * heading ONLY if centre is tied-or-better than BOTH sides.  This
 * biases the network toward straight tubes when no gradient is
 * present.  Jones [1] §III, equation (4).
 */
static void agent_rotate_toward_brightest(Agent *a, SensorReadings r)
{
    bool centre_wins = (r.front >= r.front_left) && (r.front >= r.front_right);
    if (centre_wins) return;                                /* keep heading */

    if      (r.front_left  > r.front_right) a->angle -= ROTATE_ANGLE;
    else if (r.front_right > r.front_left ) a->angle += ROTATE_ANGLE;
    else {                                  /* equal sides, both > centre */
        bool flip_to_right = (rand() & 1) != 0;
        a->angle += flip_to_right ? ROTATE_ANGLE : -ROTATE_ANGLE;
    }
}

/*
 * agent_move_and_wrap — MOVE step.
 *
 * Advance by STEP_SIZE in the heading direction, then toroidally
 * wrap to the trail's active grid extent.  Wrap uses `while` (not
 * modulo) so the agent's x/y stay floats throughout — fractional
 * positions are preserved for the deposit step.
 */
static void agent_move_and_wrap(Agent *a, const TrailField *trail)
{
    a->x += cosf(a->angle) * STEP_SIZE;
    a->y += sinf(a->angle) * STEP_SIZE;

    float grid_w = (float)trail->active_cols;
    float grid_h = (float)trail->active_rows;
    while (a->x <  0.0f  ) a->x += grid_w;
    while (a->x >= grid_w) a->x -= grid_w;
    while (a->y <  0.0f  ) a->y += grid_h;
    while (a->y >= grid_h) a->y -= grid_h;
}

/*
 * agent_food_bonus_multiplier — return FOOD_BONUS if the agent is
 * within FOOD_RADIUS of ANY food source, or 1.0 otherwise.  Returns
 * 1.0 unconditionally when food is OFF.
 *
 * Why this is the "exploit" half of explore/exploit
 *   Boosted deposit near food → stronger trail near food → stronger
 *   sensor reading for the NEXT agent passing through → more agents
 *   converge.  Without this multiplier the food sources have no
 *   gradient and the network never finds them.
 */
static float agent_food_bonus_multiplier(const Agent *a,
                                          const FoodSrc *food,
                                          bool food_on)
{
    if (!food_on) return 1.0f;

    for (int f = 0; f < N_FOOD; f++) {
        float dx = a->x - food[f].x;
        float dy = a->y - food[f].y;
        float distance_to_food = sqrtf(dx*dx + dy*dy);
        if (distance_to_food < FOOD_RADIUS) return FOOD_BONUS;
    }
    return 1.0f;
}

/*
 * agent_step — one Physarum agent tick.
 *
 *   (1) SENSE   — read three forward trail samples
 *   (2) ROTATE  — turn toward the brightest sensor
 *   (3) MOVE    — advance STEP_SIZE + wrap to grid
 *   (4) DEPOSIT — drop pheromone (boosted near food)
 *
 * The crisp tube morphology characteristic of Physarum [1] comes
 * from the abrupt ±ROTATE_ANGLE rotation (no smoothing) — small
 * changes to SENSOR_ANGLE, SENSOR_DIST, or ROTATE_ANGLE sweep the
 * network through distinctly different topologies (Jones [1] §IV).
 */
static void agent_step(Agent *a, Scene *s)
{
    TrailField *trail = s->trail;

    /* (1) SENSE */
    SensorReadings sensors = agent_sense_three_sensors(a, trail);

    /* (2) ROTATE */
    agent_rotate_toward_brightest(a, sensors);

    /* (3) MOVE + WRAP */
    agent_move_and_wrap(a, trail);

    /* (4) DEPOSIT — base deposit × food-proximity bonus */
    float bonus  = agent_food_bonus_multiplier(a, s->food, s->sim.food_on);
    float amount = s->sim.deposit * bonus;
    trail_deposit(trail, a->x, a->y, amount);
}

static void agents_step_all(Scene *s)
{
    for (int i = 0; i < s->crowd.count; i++)
        agent_step(&s->crowd.agents[i], s);

    /* Keep food-source cells above their floor — without this, decay
     * eventually drains the source's pheromone to ~0 and agents lose
     * the gradient they should be following toward food. */
    if (s->sim.food_on) {
        TrailField *trail = s->trail;
        for (int f = 0; f < N_FOOD; f++) {
            int c = wrap_c_on(trail, (int)(s->food[f].x + 0.5f));
            int r = wrap_r_on(trail, (int)(s->food[f].y + 0.5f));
            if (trail->current[r][c] < FOOD_MIN_TRAIL)
                trail->current[r][c] = FOOD_MIN_TRAIL;
        }
    }
}

static void food_draw(const Scene *s, int rows, int cols)
{
    if (!s->sim.food_on) return;
    for (int f = 0; f < N_FOOD; f++) {
        int c = (int)(s->food[f].x + 0.5f);
        int r = (int)(s->food[f].y + 0.5f);
        if (r >= 0 && r < rows-1 && c >= 0 && c < cols) {
            attron(COLOR_PAIR(CP_FOOD) | A_BOLD);
            mvaddch(r, c, '@');
            attroff(COLOR_PAIR(CP_FOOD) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §6  scene — presets & lifecycle                                        */
/* ===================================================================== */

static const char *k_preset_names[N_PRESETS] = {
    "Scatter", "Ring", "Clusters", "Mesh"
};

/* Spawn helpers */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }
static float randf_range(float lo, float hi) { return lo + randf() * (hi - lo); }

/* crowd_alloc — (re)allocate the agent array to hold `count` agents.
 * Frees any previous allocation and writes the new count into the
 * Crowd struct; safe on first call (free(NULL) is a no-op). */
static void crowd_alloc(Crowd *crowd, int count)
{
    free(crowd->agents);
    crowd->agents = malloc((size_t)count * sizeof(Agent));
    crowd->count  = count;
}

static void agent_spawn_random(Agent *a, const TrailField *t)
{
    a->x     = randf() * (float)t->active_cols;
    a->y     = randf() * (float)t->active_rows;
    a->angle = randf() * 2.0f * (float)M_PI;
}

/* ── Preset 0: Scatter ────────────────────────────────────────────── */
/* Random positions + 3 food sources in a triangle */
static void preset_scatter(Scene *s)
{
    const TrailField *t = s->trail;
    float cx = (float)t->active_cols * 0.5f;
    float cy = (float)t->active_rows * 0.5f;
    float rx = (float)t->active_cols * 0.30f;
    float ry = (float)t->active_rows * 0.28f;

    /* triangle food sources */
    s->food[0] = (FoodSrc){ cx,            cy - ry       };
    s->food[1] = (FoodSrc){ cx - rx,       cy + ry * 0.6f};
    s->food[2] = (FoodSrc){ cx + rx,       cy + ry * 0.6f};

    for (int i = 0; i < s->crowd.count; i++)
        agent_spawn_random(&s->crowd.agents[i], t);
}

/* ── Preset 1: Ring ───────────────────────────────────────────────── */
/* Agents on a circle pointing inward; single food source at centre */
static void preset_ring(Scene *s)
{
    const TrailField *t = s->trail;
    float cx = (float)t->active_cols * 0.5f;
    float cy = (float)t->active_rows * 0.5f;
    float r  = fminf((float)t->active_cols, (float)t->active_rows * 2.0f) * 0.38f;

    s->food[0] = (FoodSrc){ cx, cy };
    s->food[1] = (FoodSrc){ cx - r * 0.5f, cy };
    s->food[2] = (FoodSrc){ cx + r * 0.5f, cy };

    for (int i = 0; i < s->crowd.count; i++) {
        float angle  = (float)i / (float)s->crowd.count * 2.0f * (float)M_PI;
        float jitter = randf_range(-0.03f, 0.03f) * 2.0f * (float)M_PI;
        s->crowd.agents[i].x = cx + cosf(angle) * r;
        s->crowd.agents[i].y = cy + sinf(angle) * r * 0.5f; /* aspect correction */
        s->crowd.agents[i].angle = angle + (float)M_PI + jitter;  /* inward + jitter */
    }
}

/* ── Preset 2: Clusters ───────────────────────────────────────────── */
/* Two dense clusters on left and right; food at screen corners */
static void preset_clusters(Scene *s)
{
    const TrailField *t = s->trail;
    float lx = (float)t->active_cols * 0.22f;
    float rx = (float)t->active_cols * 0.78f;
    float cy = (float)t->active_rows * 0.50f;
    float spread_c = (float)t->active_cols * 0.08f;
    float spread_r = (float)t->active_rows * 0.15f;

    s->food[0] = (FoodSrc){ (float)t->active_cols * 0.05f, cy };
    s->food[1] = (FoodSrc){ (float)t->active_cols * 0.95f, cy };
    s->food[2] = (FoodSrc){ (float)t->active_cols * 0.50f,
                            (float)t->active_rows * 0.20f };

    for (int i = 0; i < s->crowd.count; i++) {
        bool left = (i < s->crowd.count / 2);
        float cx  = left ? lx : rx;
        s->crowd.agents[i].x = cx + randf_range(-spread_c, spread_c);
        s->crowd.agents[i].y = cy + randf_range(-spread_r, spread_r);
        s->crowd.agents[i].angle = randf() * 2.0f * (float)M_PI;
    }
}

/* ── Preset 3: Mesh ───────────────────────────────────────────────── */
/* Agents on a regular grid; food at 4 corners */
static void preset_mesh(Scene *s)
{
    const TrailField *t = s->trail;
    float mx = (float)t->active_cols * 0.12f;
    float my = (float)t->active_rows * 0.12f;

    s->food[0] = (FoodSrc){ mx,                             my                            };
    s->food[1] = (FoodSrc){ (float)t->active_cols - mx,     my                            };
    s->food[2] = (FoodSrc){ mx,                             (float)t->active_rows - my    };
    s->food[3 % N_FOOD] = (FoodSrc){ (float)t->active_cols - mx,
                                     (float)t->active_rows - my };

    int grid_side = (int)sqrtf((float)s->crowd.count);
    int placed = 0;
    for (int gi = 0; gi < grid_side && placed < s->crowd.count; gi++) {
        for (int gj = 0; gj < grid_side && placed < s->crowd.count; gj++, placed++) {
            s->crowd.agents[placed].x = ((float)gj + 0.5f) / (float)grid_side
                                         * (float)t->active_cols;
            s->crowd.agents[placed].y = ((float)gi + 0.5f) / (float)grid_side
                                         * (float)t->active_rows;
            s->crowd.agents[placed].angle = randf() * 2.0f * (float)M_PI;
        }
    }
    for (; placed < s->crowd.count; placed++)
        agent_spawn_random(&s->crowd.agents[placed], t);
}

/*
 * scene_init — bring a Scene to a fresh start.
 *
 *   (1) update the active grid extent from the current Screen size
 *   (2) (re)allocate the agent pool to crowd.count
 *   (3) clear both trail buffers
 *   (4) zero food positions, then dispatch to the active preset
 *
 * Preserves: sim.* (all user knobs), screen.*, crowd.count (the
 * caller has already set it for +/- adjustments).
 */
static void scene_init(Scene *s, int preset)
{
    s->sim.preset = preset;

    /* (1) active grid extent: full terminal width × (rows − 1)
     *     (bottom row reserved for the HUD).  The TrailField's
     *     active_rows/cols are the source of truth for sim wrapping. */
    trail_resize(s->trail, s->screen.rows - 1, s->screen.cols);

    /* (2) agent pool — size kept in sync with crowd.count */
    crowd_alloc(&s->crowd, s->crowd.count);

    /* (3) zero trail buffers */
    trail_clear(s->trail);

    /* (4) wipe food positions and let the preset fill them */
    memset(s->food, 0, sizeof s->food);

    srand((unsigned)time(NULL));  /* fresh RNG so each reset is unique */

    switch (preset) {
        case 0: preset_scatter (s); break;
        case 1: preset_ring    (s); break;
        case 2: preset_clusters(s); break;
        case 3: preset_mesh    (s); break;
    }
}

/* ===================================================================== */
/* §7  screen / HUD                                                       */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

/*
 * hud_draw — paint the two canonical HUD rows.
 *
 *   Row 0 (top, right-aligned, PAIR_HUD bright yellow A_BOLD):
 *     DATA — fps, preset name, theme name, agent count, diffuse/decay
 *            parameters, food state, paused state.
 *
 *   Row rows-1 (bottom, left-aligned, PAIR_HINT bright cyan A_BOLD):
 *     ACTIONS — every interactive key, in the order the user reads
 *               left-to-right.
 *
 * Both rows are over-stamped LAST in the frame so the HUD always reads
 * above any trail or food glyph below it.
 */
static void hud_draw(const Scene *s, int fps)
{
    /* ── (1) top row: data, right-aligned ─────────────────────────── */
    char status[160];
    snprintf(status, sizeof status,
             " %3d fps  preset:%s  theme:%s  agents:%d  diffuse:%.2f"
             "  decay:%.2f  food:%s%s ",
             fps,
             k_preset_names[s->sim.preset],
             k_themes[s->sim.theme].name,
             s->crowd.count,
             s->sim.diffuse, s->sim.decay,
             s->sim.food_on ? "ON" : "OFF",
             s->sim.paused  ? "  PAUSED" : "");

    int right_col = s->screen.cols - (int)strlen(status);
    if (right_col < 0) right_col = 0;

    move(0, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, right_col, "%s", status);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* ── (2) bottom row: actions ──────────────────────────────────── */
    move(s->screen.rows - 1, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->screen.rows - 1, 0,
             " q:quit  spc/p:pause  r:reset  n/N:preset  t/T:theme  "
             "+/-:agents  d/D:diffuse  e/E:decay  f:food  [/]:sim-fps ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Intent
 *   Per-frame fps would jitter wildly (a single 1 ms variance swings
 *   the instantaneous reading by ~6 fps at 60 Hz, ~2 fps at 30 Hz).
 *   This struct accumulates frame_count + elapsed nanoseconds over a
 *   fixed window (500 ms) and emits a smoothed `display` value each
 *   time the window fills, so the HUD reads a stable number.
 *
 *   Same shape as the FpsCounter on every other file in this project
 *   (flocking.c, crowd.c, murmuration.c, shepherd.c) — adopting the
 *   shared pattern means a reader who knows one knows them all.
 *
 * Lifecycle
 *   fps_counter_init  — zero everything at startup.
 *   fps_counter_tick  — call once per frame with the frame's dt (ns).
 *                       Emits a fresh `display` only when the window
 *                       fills.
 *
 * Why an INT display (not double)
 *   The slime-mould HUD format is " %3d fps " — three integer digits.
 *   Other files use `double display` because their HUD shows tenths
 *   ("%5.1f fps"); this file's stripped-down format saves the divide
 *   and just stores frames-per-second as an integer.
 *
 * Members
 *   frame_count   Frames seen in the current window.
 *   window_ns     Nanoseconds accumulated in the current window.
 *   display       Smoothed FPS shown in the HUD top row.
 *
 * Invariants
 *   frame_count ≥ 0  AND  window_ns ≥ 0 between resets.
 *   display ≥ 0 (zero until the first window completes).
 *
 * References
 *   [8] Fiedler 2014 — recommends a fixed smoothing window for any
 *       in-game fps display, on the same reasoning: per-frame
 *       readings are unstable, fixed-window averaging is the
 *       simplest robust answer.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    int     display;       /* whole-fps int (HUD shows as " %3d fps ") */
} FpsCounter;

static void fps_counter_init(FpsCounter *f)
{
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;     /* 500 ms */
    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;

    /* frames per (window_ns / NS_PER_SEC) seconds → frames per second */
    f->display     = (int)(f->frame_count
                         * (double)NS_PER_SEC / (double)f->window_ns);
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 * Layered ownership
 *
 *     App
 *       ├── scene   : Scene        ← simulation state (sim+crowd+food
 *       │                             +trail+screen)
 *       ├── fps     : FpsCounter   ← rolling-window fps for HUD
 *       ├── quit    : sig_atomic_t ← cleared by SIGINT/TERM + 'q' key
 *       └── resize  : sig_atomic_t ← set by SIGWINCH; main reacts on next tick
 *
 * Intent
 *   The static `g_app` is the program's ONLY file-scope state pointer.
 *   Signal handlers reach `quit` and `resize` through `g_app` directly
 *   (POSIX handlers can't receive context); everything else flows
 *   through an `App *` argument.
 *
 * Why `volatile sig_atomic_t` for the flags
 *   sig_atomic_t (C11 §5.1.2.3) is guaranteed to be readable/writable
 *   in a single uninterruptible operation from a signal handler; any
 *   wider type risks a torn read.  `volatile` prevents the compiler
 *   from caching the flag's value in a register inside the main loop
 *   — the loop must re-read on every iteration in case a signal
 *   fired between iterations.  Together these are the canonical
 *   "flag set by signal, polled by main" pattern.
 *
 * References
 *   None directly — this is a project-structure pattern.  Stevens,
 *   APUE §10.3 describes the volatile sig_atomic_t signal-flag idiom
 *   that POSIX programs use.  Nystrom, "Game Programming Patterns"
 *   Ch. 9 (Game Loop) describes the same "one root, signal flags via
 *   globals, everything else via pointer" arrangement used here and
 *   in every other file in this project.
 */
typedef struct {
    Scene                 scene;
    FpsCounter            fps;
    volatile sig_atomic_t quit;
    volatile sig_atomic_t resize;
} App;

static App g_app;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_app.quit   = 1;
    if (s == SIGWINCH)               g_app.resize = 1;
}

static void do_cleanup(void)
{
    endwin();
    free(g_app.scene.crowd.agents);
}

/* clamp_inclusive — keep `*v` inside [lo, hi] (used by every parameter
 * adjustment in app_handle_key so the bounds aren't re-typed each case). */
static void clampf(float *v, float lo, float hi)
{
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}
static void clampi(int *v, int lo, int hi)
{
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}

/*
 * adjust_agent_count — change crowd.count by `delta`, clamp to
 * [N_AGENTS_MIN, N_AGENTS_MAX], then re-init the scene so the
 * crowd is reallocated to the new size and the preset re-runs.
 */
static void adjust_agent_count(Scene *s, int delta)
{
    int next = s->crowd.count + delta;
    clampi(&next, N_AGENTS_MIN, N_AGENTS_MAX);
    if (next == s->crowd.count) return;
    s->crowd.count = next;
    scene_init(s, s->sim.preset);
}

/* cycle_preset — n=+1 / N=-1 through presets, then re-init scene. */
static void cycle_preset(Scene *s, int dir)
{
    int next = (s->sim.preset + dir + N_PRESETS) % N_PRESETS;
    scene_init(s, next);
}

/* cycle_theme — t=+1 / T=-1 through colour themes; immediate effect. */
static void cycle_theme(Scene *s, int dir)
{
    s->sim.theme = (s->sim.theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->sim.theme);
}

/*
 * Parameter-range bounds for the live-tunable knobs.  Centralised so
 * the HUD's parameter regime + the key handler's clamp share a single
 * source of truth.  See Jones [1] §IV for why these ranges matter:
 * diffuse outside [0.05, 0.90] either fragments the network (too low)
 * or floods it into uniform haze (too high); decay outside [0.01,
 * 0.30] either preserves stale tubes forever or kills new growth
 * before it can self-reinforce.
 */
#define DIFFUSE_MIN  0.05f
#define DIFFUSE_MAX  0.90f
#define DIFFUSE_STEP 0.05f
#define DECAY_MIN    0.01f
#define DECAY_MAX    0.30f
#define DECAY_STEP   0.01f

/* adjust_diffuse — bump diffusion weight by ±DIFFUSE_STEP, clamped. */
static void adjust_diffuse(Scene *s, int dir)
{
    s->sim.diffuse += (float)dir * DIFFUSE_STEP;
    clampf(&s->sim.diffuse, DIFFUSE_MIN, DIFFUSE_MAX);
}

/* adjust_decay — bump decay rate by ±DECAY_STEP, clamped. */
static void adjust_decay(Scene *s, int dir)
{
    s->sim.decay += (float)dir * DECAY_STEP;
    clampf(&s->sim.decay, DECAY_MIN, DECAY_MAX);
}

/* adjust_sim_fps — bump physics rate by ±SIM_FPS_STEP, clamped. */
static void adjust_sim_fps(Scene *s, int dir)
{
    s->sim.sim_fps += dir * SIM_FPS_STEP;
    clampi(&s->sim.sim_fps, SIM_FPS_MIN, SIM_FPS_MAX);
}

/*
 * app_handle_key — dispatch a single keypress; return false to quit.
 *
 *   q / Q / ESC    quit
 *   space / p / P  pause / resume
 *   r / R          re-init the current preset
 *   n / N          cycle preset forward / backward
 *   t / T          cycle theme forward / backward
 *   + / =          add N_AGENTS_STEP agents (cap N_AGENTS_MAX)
 *   -              remove N_AGENTS_STEP agents (floor N_AGENTS_MIN)
 *   d / D          increase / decrease diffusion weight
 *   e / E          increase / decrease decay rate
 *   f / F          toggle food sources
 *   ] / [          increase / decrease sim fps
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;       break;

    case 'r': case 'R': scene_init(s, s->sim.preset);                   break;

    case 'n':           cycle_preset(s, +1);                            break;
    case 'N':           cycle_preset(s, -1);                            break;

    case 't':           cycle_theme(s, +1);                             break;
    case 'T':           cycle_theme(s, -1);                             break;

    case '+': case '=': adjust_agent_count(s, +N_AGENTS_STEP);          break;
    case '-':           adjust_agent_count(s, -N_AGENTS_STEP);          break;

    case 'd':           adjust_diffuse(s, +1);                          break;
    case 'D':           adjust_diffuse(s, -1);                          break;

    case 'e':           adjust_decay(s, +1);                            break;
    case 'E':           adjust_decay(s, -1);                            break;

    case 'f': case 'F': s->sim.food_on = !s->sim.food_on;               break;

    case ']':           adjust_sim_fps(s, +1);                          break;
    case '[':           adjust_sim_fps(s, -1);                          break;

    default: break;
    }
    return true;
}

/*
 * app_do_resize — handle a pending SIGWINCH.
 *
 *   (1) tear ncurses down + up so getmaxyx returns the new dims
 *   (2) read new cols/rows into Scene.screen
 *   (3) re-init the active preset so the trail field, agent pool, and
 *       food positions all match the new extent
 */
static void app_do_resize(App *app)
{
    Scene *s = &app->scene;
    endwin();
    refresh();
    getmaxyx(stdscr, s->screen.rows, s->screen.cols);
    scene_init(s, s->sim.preset);
    app->resize = 0;
}

int main(void)
{
    /* (a) RNG seed, atexit cleanup, signal handlers */
    atexit(do_cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    /* (b) Bring up the App: ncurses + initial scene config */
    App   *app   = &g_app;
    Scene *scene = &app->scene;

    /* Initial sim defaults */
    scene->sim.preset    = 0;
    scene->sim.theme     = 0;
    scene->sim.sim_fps   = SIM_FPS_DEF;
    scene->sim.paused    = false;
    scene->sim.food_on   = true;
    scene->sim.deposit   = DEPOSIT_DEF;
    scene->sim.decay     = DECAY_DEF;
    scene->sim.diffuse   = DIFFUSE_DEF;
    scene->crowd.count   = N_AGENTS_DEF;
    scene->trail         = &g_trail_field;

    fps_counter_init(&app->fps);

    screen_init();
    getmaxyx(stdscr, scene->screen.rows, scene->screen.cols);
    scene_init(scene, scene->sim.preset);

    /* (c) Game loop */
    int64_t t_last = clock_ns();

    while (!app->quit) {

        /* (1) handle SIGWINCH */
        if (app->resize) {
            app_do_resize(app);
            t_last = clock_ns();
            continue;
        }

        /* (2) drain input */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->quit = 1;
                break;
            }
        }

        /* (3) tick — physics if not paused */
        if (!scene->sim.paused) {
            agents_step_all(scene);
            trail_update(scene->trail, scene->sim.diffuse, scene->sim.decay);
        }

        /* (4) draw */
        erase();
        trail_draw(scene->trail, scene->screen.rows, scene->screen.cols);
        food_draw (scene, scene->screen.rows, scene->screen.cols);
        hud_draw  (scene, app->fps.display);
        wnoutrefresh(stdscr);
        doupdate();

        /* (5) Frame cap: sleep BEFORE measuring next frame's dt so terminal
         *     I/O doesn't eat into the budget.  dt cap at 100 ms prevents
         *     a physics avalanche if the process was suspended. */
        const int64_t DT_CAP_NS = 100000000LL;        /* 100 ms */

        int64_t t_now  = clock_ns();
        int64_t t_used = t_now - t_last;
        t_last         = t_now;
        if (t_used > DT_CAP_NS) t_used = DT_CAP_NS;

        int64_t t_sleep = TICK_NS(scene->sim.sim_fps) - (clock_ns() - t_now);
        clock_sleep_ns(t_sleep);

        /* (6) rolling-window fps counter */
        fps_counter_tick(&app->fps, t_used);
    }
    return 0;
}
