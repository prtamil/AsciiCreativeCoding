/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * constellation.c — particle constellation effect in ncurses
 *
 * Stars drift slowly across the terminal.  When two stars come within
 * CONNECT_DIST pixels of each other an ASCII line is drawn between them
 * using slope-matched characters.  Lines fade with distance via bold
 * weight and stippling, creating a live constellation map.
 *
 * ROOT FIX: same pixel-space / cell-space separation as bounce_ball.c.
 * Physics runs entirely in square pixel space; only px_to_cell_x/y()
 * converts to terminal columns/rows for drawing.
 *
 * RENDER INTERPOLATION:
 *   Stars carry prev_px/prev_py (previous tick position) so we can lerp:
 *     draw_px = prev_px + (px - prev_px) * alpha
 *   This is true interpolation, not extrapolation, which is required
 *   here because wander acceleration changes velocity each tick — forward
 *   extrapolation from the current state would drift from the real
 *   in-between position.
 *
 * CONNECTION RENDERING:
 *   For each pair (i < j) within CONNECT_DIST pixels:
 *     ratio = dist / connect_dist   (0.0 = same spot, 1.0 = edge)
 *     ratio < 0.50  → bold, every cell drawn       (close, bright)
 *     ratio < 0.75  → normal, every cell drawn      (medium)
 *     ratio < 1.00  → normal, every 2nd cell drawn  (far, stippled)
 *   Slope character is selected from pixel-space angle (physically correct):
 *     0–22.5°   '-'
 *     22.5–67.5° '\' or '/'  (sign of dx_px × dy_px)
 *     67.5–90°  '|'
 *   Thin-line rendering (one cell per major-axis step) prevents the doubled
 *   diagonal characters ('\\'  '//') that vanilla Bresenham produces.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   =  -      add / remove a star
 *   r         randomise all stars
 *   ]  [      faster / slower simulation
 *   c         cycle connection threshold (tight / normal / wide)
 *   t  T      next / previous theme  (night/aurora/nebula/winter/ember/void/mono)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra constellation.c -o constellation -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config   — every tunable constant
 *   §2  clock    — monotonic ns clock + sleep
 *   §3  color    — 7 themes × 6 star colours + line + HUD/HINT pairs
 *   §4  coords   — pixel↔cell conversion; the one aspect-ratio fix
 *   §5  star     — pixel-space physics with wander + speed cap
 *   §6  scene    — star pool; tick; Bresenham line draw
 *   §7  screen   — single stdscr buffer + HUD
 *   §8  app      — dt loop, input, resize, cleanup
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * READING ORDER
 *   1. CONCEPTS + MENTAL MODEL (below) — algorithm in plain English.
 *   2. GUIDED TUTORIAL (below) — 8 mini-lessons that build the
 *      proximity-graph constellation from "what is a star" up to the
 *      cell_used overlap-prevention grid.
 *   3. §1 config — every constant you'd tweak when experimenting,
 *      grouped: simulation / display / themes.
 *   4. §3 colour — ConstTheme table + theme_apply.  Read this if you
 *      want to add a new palette; it's the only colour code in the file.
 *   5. §5 star — pixel-space physics: wander, speed cap, wall reflect.
 *   6. §6 scene — the per-frame heart: tick + draw + the proximity
 *      pair-loop + thin-line Bresenham + cell_used grid.
 *   7. §7 screen / §8 app — ncurses + fixed-step loop boilerplate.
 *
 * NAMING
 *   Star            one drifting point with pos / vel / prev / glyph / hue
 *   Scene           the star pool + paused + connect_preset + current_theme
 *   ConstTheme      one palette entry — 6 star hues + 1 line hue
 *   k_themes[]      7-entry table of palettes
 *   k_connect_presets[]  3 distance thresholds: tight / normal / wide
 *   px, py          star position in PIXEL space (sub-cell, float)
 *   prev_px, prev_py  position at the start of the previous tick
 *                   (used for render-frame interpolation; see Tutorial #3)
 *   pw(cols), ph(rows)  cells × CELL_W/H — width/height in PIXELS
 *   px_to_cell_x/y  the ONE conversion point pixel → terminal cell
 *   draw_line()     thin Bresenham — one cell per major-axis step
 *   cell_used[][]   per-frame bool grid: "this line already drew here"
 *                   prevents mojibake when many lines overlap
 *
 * BACKGROUND ASSUMED
 *   • Bresenham's line algorithm at a "discrete pixel rasterisation"
 *     level (we use a thin variant — no doubled diagonals).
 *   • Pixel-space physics vs cell-space rendering — the project-wide
 *     pattern, also used by bounce_ball.c and the boids files.
 *   • Fixed-step accumulator + render-frame interpolation (see
 *     documentation/Architecture.md).
 *   • Object-pool pattern (Star[STARS_MAX] with `n` active count).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Star wander (Ornstein-Uhlenbeck) + proximity-graph line drawing.
 *                  Each star applies a small random velocity increment (wander)
 *                  bounded to prevent runaway speed.  For each pair of stars
 *                  within CONNECT_DIST pixels, a line is drawn with slope-matched
 *                  characters and distance-based stippling.
 *
 * Physics        : Wander: each tick, velocity perturbed by small random
 *                  δvx, δvy bounded at WANDER_FORCE; speed capped at SPEED_MAX.
 *                  Stars bounce off screen edges with velocity reflection.
 *
 * Rendering      : Render interpolation: draw position lerped between previous
 *                  and current tick position at sub-frame alpha — prevents
 *                  jitter when render rate ≠ sim rate.  Thin-line Bresenham:
 *                  one cell per major-axis step to avoid doubled diagonal chars.
 *                  Line brightness: ratio < 0.50 → bold; ratio < 0.75 → normal;
 *                  ratio < 1.00 → stippled (every 2nd cell drawn).
 *
 * Math           : Slope character selection from pixel-space angle:
 *                  0–22.5° → '─'; 22.5–67.5° → '╲'/'╱'; 67.5–90° → '│'.
 *                  CONNECT_DIST in pixels; aspect-corrected before comparison.
 *
 * Data-structure : Star[STARS_MAX] flat array with `n` ≤ STARS_MAX active.
 *                  One ConstTheme[] table for palettes; one bool VLA
 *                  `cell_used[rows][cols]` reset per frame for line
 *                  overlap-prevention.  No malloc in the hot path; the
 *                  whole simulation runs out of the static Scene struct
 *                  plus the per-frame VLA.
 *
 * Performance    : O(N²/2) pair scan per frame.  At N=30 → 435 pairs
 *                  ×  ≤ 200 px per line  = ≤ 87 000 mvaddch's per frame,
 *                  most short-circuited by the dist² cull.  60 fps is
 *                  comfortable up to N=80 (3160 pairs).  The cell_used
 *                  grid adds one bool flag per cell per frame —
 *                  negligible vs the line draws.
 *
 * References     :
 *   Bresenham, J. E. (1965) — "Algorithm for computer control of a
 *     digital plotter", *IBM Systems Journal* 4(1):25–30.  The thin-
 *     line variant here uses the same error-accumulation idea.
 *   Uhlenbeck & Ornstein (1930) — "On the Theory of the Brownian
 *     Motion", *Phys. Rev.* 36:823–841.  Source of the bounded-random-
 *     walk model used for the wander force.
 *   particle_systems/burst.c — radial-fan sibling: same pool/tick/draw
 *     framework, different motion model (no wander, explicit drag).
 *   particle_systems/comet.c — moving-emitter sibling with trails and
 *     a ground-impact blast.  Uses the same pixel-vs-cell separation.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The constellation is an O(N²) PROXIMITY GRAPH redrawn from scratch
 * every frame.  Stars are point particles wandering with a tiny
 * Ornstein–Uhlenbeck nudge in pure pixel space, bouncing off the
 * screen rect.  Each frame, every (i,j) pair within CONNECT_DIST gets
 * an ASCII line connecting them; line WEIGHT (bold / normal /
 * stippled) is a function of the inter-star distance.  Stars are
 * drawn AFTER lines so they always sit on top.  All physics is in
 * sub-pixel space; conversion to terminal cells happens ONLY in
 * scene_draw().
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture fireflies in a dark field, each carrying a faint LED.  When
 * two fireflies drift within "shouting distance" of each other, an
 * imaginary string of light snaps between them; the closer they are,
 * the brighter the string.  The graph is recomputed every blink, so
 * strings appear and dissolve as the swarm moves.  The wander force is
 * what makes fireflies look natural: each tick the velocity gets
 * bumped by a small random kick, then immediately speed-capped so it
 * cannot run away.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. scene_tick (per star): save (px,py) into (prev_px,prev_py).  Add
 *     small random acceleration (±WANDER_ACCEL)·dt to velocity.  Clamp
 *     speed magnitude to SPEED_CAP.  Integrate pos += v·dt.  Reflect
 *     off screen edges (vx → −vx if px < 0 or > max_px).
 *  2. scene_draw alpha = sim_accum/tick_ns ∈ [0,1).  Pre-compute
 *     draw_px = prev_px + (px − prev_px)·alpha for every star, plus
 *     its terminal cell (dcx, dcy).
 *  3. Connection pass: O(N²/2) loop over pairs (i<j).  Skip if
 *     dist² >= CONNECT_DIST².  Compute ratio = dist/CONNECT_DIST.
 *     Bucket: <0.50 bold solid; <0.75 normal solid; <1.00 normal
 *     stipple-2.  Call draw_line with a thin Bresenham that picks per-
 *     step glyph from local axis movement (`-`, `|`, `\\` or `/`).
 *  4. cell_used[][] grid: each line marks cells it draws on; later
 *     lines that hit those cells silently skip — prevents thick
 *     mixed-character bundles when many lines overlap.
 *  5. Star pass: each star renders its glyph (`*+o@.` cycled) at
 *     (dcx, dcy) with bold and its assigned colour, atop any line.
 *  6. HUD draws fps, star count, connect preset name, sim Hz, paused.
 *
 * KEY FORMULAS
 * ────────────
 *  Render lerp        :  draw_p = prev_p + (p − prev_p) · alpha
 *  Wander integration :  v += rand(±WANDER_ACCEL) · dt
 *  Speed cap          :  if |v| > SPEED_CAP, v *= SPEED_CAP/|v|
 *  Wall reflection    :  if p < 0 or p > max,   v = −v;  clamp p
 *  Pair distance²     :  d² = (px_j − px_i)² + (py_j − py_i)²
 *  Brightness bucket  :  ratio = d / CONNECT_DIST    (0..1)
 *  Bresenham step
 *    shallow (adx>=ady):  diagonal glyph if next_err < 0; else `-`
 *    steep             :  diagonal glyph if next_err < 0; else `|`
 *  Diagonal direction :  `\\` if sx·sy > 0 else `/`
 *  Pixel↔cell convert :  cx = ⌊px/CELL_W + 0.5⌋
 *  Pair complexity    :  C(N,2) = N(N−1)/2; default N=30 → 435 pairs
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Two stars ON THE SAME CELL: draw_line's adx=0,ady=0 branch draws
 *    the diagonal glyph and exits — the star pass then overwrites it
 *    with the star glyph, so the visual is just two stars overlapping.
 *  • cell_used grid uses VLA `bool[rows][cols]` — for very large
 *    terminals this is small (e.g. 200×60 = 12 KB) but stack-allocated;
 *    huge sizes could overflow.  Acceptable on standard terminals.
 *  • Resize: stars outside new screen are clamped; their velocities
 *    keep them moving; first-frame jitter possible.
 *  • Stipple-2 means draw `step % 2 == 0` only; visually shows ~50 %
 *    of cells.  step starts at 0, so the first cell is always drawn.
 *  • Wander uses a random ax,ay each tick — at very low SIM_FPS
 *    (<20) the kicks become large per integration step and motion
 *    becomes jittery.  SPEED_CAP and dt scaling mitigate this.
 *  • prev/curr swap is at the START of star_tick.  Skip a tick (e.g.
 *    paused) and prev tracks correctly because it isn't updated.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Default 30 stars, "normal" 200 px connect: at 60 fps you should
 *    see 5–15 active connection lines at any moment.
 *  • Press `=` to add stars to STARS_MAX=80; line count grows
 *    quadratically; CPU climbs noticeably.
 *  • Press `c` to cycle connect preset: tight (120 px) → normal
 *    (200 px) → wide (280 px).  Wide should approximately double the
 *    line count vs normal.
 *  • Pause (space): freeze; pick any pair; their distance in cells
 *    should match their drawn line length within ±1.
 *  • Sight along any line: glyphs alternate `-` for runs and `\\`
 *    /`/` for true diagonal cells; never two diagonal glyphs in a row.
 *  • Stars never escape — bounce off all four walls; verify by
 *    watching the corners.
 *  • Theme cycle (`t`/`T`): every existing star and line picks up the
 *    new palette on the very next frame — no flash, no reset.  Try
 *    cycling rapidly: the lattice never blanks.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 * Eight mini-lessons.  Read them in order; each ends with the pseudocode
 * or struct that maps onto a real symbol below.
 *
 * ─── 1.  What is a Star?  ──────────────────────────────────────────── *
 *   The atom of the simulation.  Five floats and one byte:
 *       float px, py        // position in PIXEL space (sub-cell precision)
 *       float vx, vy        // velocity, pixels/second
 *       float prev_px, prev_py  // position at the LAST tick
 *       char  glyph         // chosen at spawn from "*+o@."
 *       Hue   pair_id       // which colour pair to use (1..6)
 *
 *   No "alive" flag — the active count `Scene.n` tells you which prefix
 *   of the array matters.  The first n slots are live; the rest are
 *   unused storage.  This makes "add a star" an O(1) decrement of `n`
 *   not a pool scan.
 *
 *       Star ≡ { pos, vel, prev_pos, glyph, hue }
 *
 * ─── 2.  Pixel space vs cell space  ─────────────────────────────────── *
 *   Terminal cells are roughly 8 × 16 PIXELS in a typical monospace
 *   font — they're rectangles, not squares.  If you do physics in cell
 *   space, you have to multiply x-velocities by 2 to make motion look
 *   visually isotropic.  Easy to forget; easy to get wrong.
 *
 *   The project-wide solution: physics in SUB-CELL PIXEL space where
 *   the units ARE square, then convert to terminal cells ONLY at draw
 *   time:
 *
 *       CELL_W = 8,  CELL_H = 16          // pixels per cell
 *       pw(cols) = cols × CELL_W           // playable width in pixels
 *       ph(rows) = rows × CELL_H           // playable height in pixels
 *       px_to_cell_x(px) = round(px / CELL_W)
 *       px_to_cell_y(py) = round(py / CELL_H)
 *
 *   ONE conversion point, ONE direction.  All other geometry — wall
 *   reflection, pair distance, line drawing — happens in pixels.  Test
 *   it: a star moving at (vx=80, vy=80) traces a true 45° line on
 *   screen, not a 26.5°-flat oval.
 *
 * ─── 3.  Render interpolation (lerp, not extrapolate)  ──────────────── *
 *   The sim runs at fixed steps (e.g. 60 Hz).  The render can fire at
 *   any rate.  Without interpolation, render frames see the discrete
 *   sim state — motion JUDDERS when render is faster than sim.
 *
 *   Naive fix: extrapolate forward by alpha × velocity.  But our
 *   wander adds random acceleration each tick — forward prediction
 *   diverges from the next real tick's position.  So the lattice
 *   would visibly snap on every tick boundary.
 *
 *   Correct fix: keep PREVIOUS position too, and interpolate BACKWARDS:
 *
 *       alpha = sim_accum / tick_ns            // ∈ [0, 1)
 *       draw_px = prev_px + (px − prev_px) × alpha
 *
 *   At alpha=0 we render where the star WAS at the last tick boundary.
 *   At alpha→1 we render where it IS now.  Smooth between.  The render
 *   is always slightly BEHIND reality, but it never overshoots, so no
 *   snap-back artifact.
 *
 *       prev = curr  // at start of every star_tick
 *       integrate curr
 *
 * ─── 4.  Ornstein-Uhlenbeck wander  ─────────────────────────────────── *
 *   A constant-velocity star looks dead.  A purely-random-walking star
 *   shimmers in place.  The Ornstein-Uhlenbeck bounded random walk is
 *   the goldilocks middle:
 *
 *       each tick:
 *           v.x += uniform(±WANDER_ACCEL) × dt
 *           v.y += uniform(±WANDER_ACCEL) × dt
 *           if |v| > SPEED_CAP:
 *               v *= SPEED_CAP / |v|         // clamp magnitude, keep direction
 *
 *   The random kick keeps direction changing slowly; the speed cap
 *   prevents the cumulative kicks from blowing up.  Net effect: stars
 *   meander in gentle curves rather than straight lines or noise.
 *
 *   Why "Ornstein-Uhlenbeck"?  Their 1930 paper on Brownian motion
 *   formalised exactly this kind of bounded random walk with a
 *   restoring force (here, the speed cap acts as the restoring force).
 *
 * ─── 5.  Wall reflection (the simplest collision)  ──────────────────── *
 *   When a star reaches the edge of the playable rectangle:
 *
 *       if px < 0          : px = 0;          vx = −vx;
 *       if px > pw − 1     : px = pw − 1;     vx = −vx;
 *       if py < 0          : py = 0;          vy = −vy;
 *       if py > ph − 1     : py = ph − 1;     vy = −vy;
 *
 *   Two things at each bound: SNAP back to in-bounds, then NEGATE the
 *   normal component.  Without the snap, the star sits stuck at the
 *   bound for one extra frame; without the negate, it would tunnel.
 *
 *   This is the dumbest possible collision response and it works
 *   because the simulation is friction-free — energy never builds up
 *   to the point where the snap becomes visible.
 *
 * ─── 6.  Proximity graph rendering  ─────────────────────────────────── *
 *   Every frame, scan all C(N, 2) = N(N−1)/2 pairs:
 *
 *       for i = 0..n-2:
 *         for j = i+1..n-1:
 *           dx = stars[j].px - stars[i].px
 *           dy = stars[j].py - stars[i].py
 *           d² = dx*dx + dy*dy
 *           if d² >= CONNECT_DIST²:  continue       // cull early
 *           ratio = sqrt(d²) / CONNECT_DIST          // ∈ [0, 1]
 *           if ratio < 0.50:  draw_line(...,  A_BOLD,    stride=1)
 *           if ratio < 0.75:  draw_line(...,  A_NORMAL,  stride=1)
 *           else:             draw_line(...,  A_NORMAL,  stride=2)  // stippled
 *
 *   The three brightness BUCKETS are the cheap way to get "lines fade
 *   with distance" without subpixel rendering or anti-aliasing.  Each
 *   bucket is a different attribute combo + Bresenham stride.
 *
 *   d² < D² instead of d < D — saves a sqrt per pair, fires on 99 %
 *   of pairs, sqrt only runs for the few that actually draw a line.
 *
 * ─── 7.  Thin-line Bresenham + slope glyphs  ─────────────────────────── *
 *   Vanilla Bresenham at 45° emits TWO cells per step (one horizontal,
 *   one diagonal) — you see doubled '\\' '\\' or '//' '//' characters.
 *   Looks like a thick line, not a thin one.
 *
 *   The fix: at each step, choose ONE of three glyphs based on which
 *   axis just advanced:
 *
 *       if dx_step and dy_step:   glyph = '\\' or '/'   // (sign of dx·dy)
 *       if dx_step only:          glyph = '─'           // horizontal
 *       if dy_step only:          glyph = '│'           // vertical
 *
 *   One cell per major-axis step.  At true 45° you see a clean
 *   diagonal of '\\' or '/'; at near-flat angles you see mostly '─'
 *   with occasional diagonal "kicks" where the minor axis advances.
 *
 * ─── 8.  cell_used: prevent overlap mojibake  ────────────────────────── *
 *   With N = 30 stars and 435 candidate pairs, multiple lines often
 *   pass through the SAME cell.  If line A draws '\\' there and line B
 *   later writes '─' on top, you see '─' — but the eye reads it as a
 *   missing diagonal.  Bundles of overlapping lines visually fall apart.
 *
 *   Fix: a per-frame `bool cell_used[rows][cols]` (VLA, zeroed every
 *   frame).  When draw_line writes a cell:
 *
 *       if cell_used[row][col]:  return     // first writer wins
 *       cell_used[row][col] = true
 *       mvaddch(row, col, glyph)
 *
 *   The first line to claim a cell keeps it; later lines that pass
 *   through silently skip.  Result: overlapping lines look like one
 *   coherent shape, not garbage.  Cost: one bool per cell per frame
 *   (~12 KB on a 200×60 terminal) — trivial.
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
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    HUD_COLS         = 64,   /* widened to fit [theme] suffix             */
    FPS_UPDATE_MS    = 500,

    STARS_DEFAULT    = 30,
    STARS_MIN        =  5,
    STARS_MAX        = 80,

    N_STAR_COLORS    =  6,   /* color pairs 1..6 for stars               */
    CONN_PAIR        =  7,   /* color pair for all connection lines       */
    HUD_PAIR         =  8,   /* bright yellow 226 + A_BOLD — top-right    */
    HINT_PAIR        =  9,   /* bright cyan   51  + A_BOLD — bottom-left  */
};

/*
 * CELL_W, CELL_H — sub-pixel units per terminal cell.
 * CELL_H / CELL_W = 2 matches the typical terminal cell aspect ratio.
 * All physics lives in this square pixel space.
 */
#define CELL_W   8
#define CELL_H  16

/*
 * Star drift speed (pixels per second).
 *
 * The staircase rule (speed >= CELL_H × fps / 4 ≈ 240 px/s) is
 * intentionally relaxed here.  The visual goal is slow, drifting stars.
 * Wander force keeps motion non-axis-aligned, reducing staircase
 * visibility.  Lerp interpolation eliminates sub-cell jitter between ticks.
 */
#define SPEED_MIN   50.0f
#define SPEED_MAX  120.0f

/*
 * Wander: random acceleration (px/s²) added each tick.
 * Keeps star paths gently curving rather than straight forever.
 * SPEED_CAP prevents unlimited runaway from accumulated wander.
 */
#define WANDER_ACCEL  20.0f
#define SPEED_CAP    130.0f

/*
 * Connection distance presets (pixels).  Higher = more connections.
 * At 200 px on a 200-col × 50-row terminal (1600 × 800 px space):
 *   each star's connect circle covers ~10 % of the screen area
 *   → expect ~3 neighbours per star → ~45 lines for 30 stars.
 */
static const float k_connect_presets[] = { 120.0f, 200.0f, 280.0f };
static const char *k_connect_names[]   = { "tight", "normal", "wide" };
enum { N_CONNECT_PRESETS = 3 };

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * ConstTheme — one palette: 6 star hues + 1 connection-line hue.
 *
 * Every theme reassigns the SAME seven color pair IDs (1..7), so live
 * stars and connection lines auto-pick the new colour on the next
 * mvaddch — no needs_clear, no full redraw required.  HUD/HINT pairs
 * sit at 8/9, untouched by theme cycling.
 *
 * All entries are in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule (24+ and 244+ only).
 */
typedef struct {
    const char *name;
    int         star_fg256[6];
    int         star_fg8  [6];
    int         conn_fg256;
    int         conn_fg8;
} ConstTheme;

#define N_THEMES  7

static const ConstTheme k_themes[N_THEMES] = {
    /* The classic — was the only palette before theme support was added. */
    { "night",
      {  15,  51,  39, 201, 147, 159 },
      { COLOR_WHITE,  COLOR_CYAN,  COLOR_BLUE,
        COLOR_MAGENTA, COLOR_CYAN,  COLOR_WHITE },
      24,  COLOR_BLUE },

    /* Greens and teals — northern-lights vibe. */
    { "aurora",
      {  46,  51,  87, 119, 156, 158 },
      { COLOR_GREEN, COLOR_CYAN,  COLOR_CYAN,
        COLOR_GREEN, COLOR_GREEN, COLOR_CYAN },
      30,  COLOR_GREEN },

    /* Saturated magentas / pinks — a star-forming region. */
    { "nebula",
      { 201, 207, 213, 219, 225, 159 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN },
      91,  COLOR_MAGENTA },

    /* Cool blues + whites — clear winter sky. */
    { "winter",
      { 195, 159, 153, 117,  75,  39 },
      { COLOR_WHITE, COLOR_CYAN, COLOR_CYAN,
        COLOR_CYAN,  COLOR_BLUE, COLOR_BLUE },
      24,  COLOR_BLUE },

    /* Warm reds → oranges → yellows — campfire / brazier sky. */
    { "ember",
      { 226, 220, 214, 208, 202, 196 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_RED,    COLOR_RED,    COLOR_RED },
      130, COLOR_RED },

    /* Deep indigos + cool whites — empty-space void. */
    { "void",
      { 252, 248, 141, 105,  99,  63 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_BLUE,
        COLOR_BLUE,  COLOR_BLUE,  COLOR_BLUE },
      53,  COLOR_BLUE },

    /* Whites + light greys — minimalist / B&W photograph. */
    { "mono",
      { 255, 253, 252, 250, 248, 246 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
        COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
      244, COLOR_WHITE },
};

/*
 * theme_apply — bind the 7 rendering pair IDs (1..7) to one theme.
 * Called from color_init at startup and from the t/T key handler.
 * Pairs 8 (HUD) and 9 (HINT) are NOT touched; they live forever
 * once color_init initialises them.
 */
static void theme_apply(int theme)
{
    const ConstTheme *th = &k_themes[theme];
    if (COLORS >= 256) {
        for (int i = 0; i < 6; i++)
            init_pair(1 + i, th->star_fg256[i], COLOR_BLACK);
        init_pair(CONN_PAIR, th->conn_fg256, COLOR_BLACK);
    } else {
        for (int i = 0; i < 6; i++)
            init_pair(1 + i, th->star_fg8[i],   COLOR_BLACK);
        init_pair(CONN_PAIR, th->conn_fg8,   COLOR_BLACK);
    }
}

/*
 * color_init — start_color + theme_apply + theme-independent HUD/HINT.
 *
 * HUD/HINT pairs (8/9) get bright yellow / bright cyan per the CLAUDE.md
 * HUD standard — both drawn with A_BOLD, never A_DIM, so they remain
 * legible against any theme's star palette.
 */
static void color_init(int theme)
{
    start_color();
    use_default_colors();
    theme_apply(theme);
    if (COLORS >= 256) {
        init_pair(HUD_PAIR,  226, -1);    /* bright yellow */
        init_pair(HINT_PAIR,  51, -1);    /* bright cyan   */
    } else {
        init_pair(HUD_PAIR,  COLOR_YELLOW, -1);
        init_pair(HINT_PAIR, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §4  coords — the one place aspect ratio is handled                    */
/* ===================================================================== */

/*
 * §4 PREAMBLE — pixel ↔ cell, the project's ROOT FIX
 * ────────────────────────────────────────────────────
 *
 * Terminal cells are about 8 × 16 PIXELS in a typical monospace font —
 * a 1:2 aspect ratio.  If physics ran in CELL coordinates, every x
 * velocity would need a × 2 multiplier to make motion look isotropic.
 * That's the project-wide trap this file (and bounce_ball.c, boids,
 * comet.c, raster) all avoid the same way:
 *
 *     PHYSICS  in PIXEL space   (square units)
 *     RENDER   in CELL space    (terminal characters)
 *     ONE conversion point      px_to_cell_x / px_to_cell_y
 *
 * Below this section, NO code outside §5 star_spawn touches CELL_W or
 * CELL_H directly — everything goes through pw, ph, or the px_to_cell
 * helpers.  That discipline is what keeps the aspect-ratio fix
 * contained; break it and motion will start to look skewed.
 *
 * pw, ph        cell count → pixel count (multiplication)
 * px_to_cell_*  pixel → terminal cell (round-half-up division)
 */

/*
 * Pixel-space extents from terminal dimensions.
 * pw/ph are the only callers of CELL_W/CELL_H outside §5.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * "Round half up" avoids the banker's rounding oscillation that roundf()
 * can produce when a star sits exactly on a cell boundary.
 * See bounce_ball.c §4 for the detailed explanation.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  star                                                               */
/* ===================================================================== */

/*
 * §5 PREAMBLE — the actor
 * ────────────────────────
 *
 * A Star is the atom of the simulation: position + velocity + glyph +
 * colour.  Per CLAUDE.md's pixel-vs-cell rule (§4 preamble), position
 * lives in PIXEL space.  The prev_px / prev_py pair stores the position
 * AT THE LAST TICK so the renderer can interpolate sub-frame motion.
 *
 * Three operations:
 *   star_spawn  fill a freshly-allocated Star at startup or on resize
 *   star_tick   advance one star by dt (wander + cap + integrate + reflect)
 *   (no star_draw — that lives in §6 scene_draw alongside the pair loop)
 *
 * No allocation: STARS_MAX live in Scene.stars[]; Scene.n controls how
 * many are active.  Adding a star is `n++`; removing is `n--`.  No
 * pool scan, no free list.
 *
 * COORDINATE SYSTEMS USED HERE
 *   (px, py)            current pixel position
 *   (prev_px, prev_py)  pixel position at the last completed tick
 *   (vx, vy)            velocity, pixels/second
 *   max_px, max_py      playable rectangle bounds, pw(cols)-1, ph(rows)-1
 */

/*
 * Star — all positions and velocities live in PIXEL SPACE.
 *
 *   px, py          current tick position (pixels)
 *   prev_px, prev_py  previous tick position (pixels)
 *   vx, vy          velocity (pixels per second)
 *
 * WHY prev_px / prev_py instead of forward extrapolation:
 *   The wander acceleration changes velocity each tick, so the draw
 *   position after alpha ticks into the future cannot be predicted from
 *   the current state alone without solving the acceleration integral.
 *   Lerp between prev and current is exact for any acceleration profile:
 *
 *     draw_px = prev_px + (px - prev_px) * alpha   alpha ∈ [0, 1)
 *
 *   bounce_ball.c uses forward extrapolation only because its balls have
 *   constant velocity between ticks (no acceleration) — the two methods
 *   are equivalent in that special case.
 */
typedef struct {
    float px,      py;       /* position — current tick  (pixel space) */
    float prev_px, prev_py; /* position — previous tick (pixel space) */
    float vx,      vy;       /* velocity (px/s)                        */
    int   color;             /* color pair index (1..N_STAR_COLORS)    */
    char  ch;                /* star symbol                            */
} Star;

static const char k_star_chars[] = "*+o@.";
static const int  k_n_star_chars = (int)(sizeof k_star_chars - 1);

/*
 * star_spawn — initialise one Star at startup, on resize, or on 'r'.
 *
 * PURPOSE
 *   Place a star at a random position INSIDE the playable rectangle
 *   (with a 1-cell margin) and give it a uniformly-distributed random
 *   direction.  Glyph and colour pair are deterministic from the
 *   index — colour cycles 1..6, glyph cycles "*+o@.", so 30 stars
 *   show all 6 colours and 5 glyphs without rand() flicker.
 *
 * PSEUDOCODE
 *   pxw, pxh = pw(cols), ph(rows)                  // pixel extent
 *   px = uniform(CELL_W,  pxw − CELL_W)             // 1-cell margin
 *   py = uniform(CELL_H,  pxh − CELL_H)
 *   prev_px, prev_py = px, py                       // no past at spawn
 *   (dx, dy) = REJECTION-SAMPLE inside unit disk    // uniform angle
 *   speed    = uniform(SPEED_MIN, SPEED_MAX)
 *   v        = (dx, dy) / |(dx, dy)| · speed        // normalise to speed
 *   colour, glyph = deterministic from idx
 *
 * MENTAL MODEL
 *   The rejection-sample is the textbook "uniform direction without
 *   trig" trick: pick (dx, dy) uniformly in the unit SQUARE, reject if
 *   outside the unit DISK or too close to (0, 0).  Renormalising
 *   accepted samples gives uniform angle.  Picking vx, vy independently
 *   would over-represent diagonals (the square's corners are 41% farther
 *   from the origin than its edges — those become biased directions).
 *
 *   The margin (CELL_W on each side) prevents stars spawning ON the
 *   wall.  Without it, the first tick would already reflect them — a
 *   visible "snap into bounds" on startup.
 *
 * INPUTS / OUTPUTS
 *   s       ← Star struct to fill (mutated)
 *   idx     → which slot this star occupies; drives colour + glyph
 *   cols, rows → terminal extent (cells)
 *
 * UNITS
 *   px, py:   pixels (sub-cell precision)
 *   speed:    pixels/second (typically 50..120)
 *
 * WHY IT EXISTS (vs inlining in scene_init)
 *   The 'r' (randomise) key calls this on each star.  The resize handler
 *   calls it on out-of-bounds stars.  Three call sites, three different
 *   reasons; one shared implementation.
 */
static void star_spawn(Star *s, int idx, int cols, int rows)
{
    int pxw = pw(cols);
    int pxh = ph(rows);

    s->px = (float)(CELL_W + rand() % (pxw - 2 * CELL_W));
    s->py = (float)(CELL_H + rand() % (pxh - 2 * CELL_H));
    s->prev_px = s->px;
    s->prev_py = s->py;

    /*
     * Isotropic random direction via rejection-sample unit disk.
     * Separate random vx/vy produces too many diagonal stars; this
     * gives a uniform angle distribution.
     */
    float dx, dy, len;
    do {
        dx  = (float)(rand() % 2001 - 1000) / 1000.0f;
        dy  = (float)(rand() % 2001 - 1000) / 1000.0f;
        len = dx*dx + dy*dy;
    } while (len < 0.01f || len > 1.0f);

    float mag   = sqrtf(len);
    float speed = SPEED_MIN
                + (float)(rand() % (int)(SPEED_MAX - SPEED_MIN + 1));
    s->vx = (dx / mag) * speed;
    s->vy = (dy / mag) * speed;

    s->color = (idx % N_STAR_COLORS) + 1;
    s->ch    = k_star_chars[idx % k_n_star_chars];
}

/*
 * star_tick — advance one star one frame.
 *
 * PURPOSE
 *   The per-frame physics for ONE star.  Five steps in fixed order;
 *   getting the order wrong (e.g. integrating BEFORE the speed cap)
 *   produces visible artifacts.
 *
 * PSEUDOCODE
 *   prev_px, prev_py = px, py                       // save for lerp (Tut #3)
 *   v += uniform(±WANDER_ACCEL) · dt                 // O-U wander (Tut #4)
 *   if |v| > SPEED_CAP: v *= SPEED_CAP / |v|         // magnitude-only clamp
 *   px += vx · dt;  py += vy · dt                    // explicit Euler
 *   for each wall:                                    // reflect (Tut #5)
 *     if px out of [0, max_px]: clamp + negate vx
 *     if py out of [0, max_py]: clamp + negate vy
 *
 * MENTAL MODEL
 *   Each tick is one step of the bounded random walk.  The wander
 *   nudge is small (±20 px/s² · dt ≈ ±0.33 px/s per frame at 60 Hz),
 *   so direction changes slowly.  The speed cap is the ONLY thing
 *   keeping cumulative wander from blowing up to infinity — without
 *   it, after a few seconds every star would be pinned to a wall.
 *
 *   Wall reflection is the SIMPLEST collision response: SNAP into
 *   bounds, then NEGATE the normal component of velocity.  Two ops,
 *   one direction each.  Don't forget the snap — without it, a star
 *   that overshoots stays out-of-bounds for one frame, then bounces
 *   back next frame, then bounces again — a visible jitter at the wall.
 *
 * INPUTS / OUTPUTS
 *   s       ← Star to advance (mutated)
 *   dt      → frame time, seconds (typically 1/60)
 *   max_px  → pw(cols)-1, the right wall in pixels
 *   max_py  → ph(rows)-1, the bottom wall in pixels
 *
 * UNITS
 *   dt:        seconds
 *   wander:    pixels/second² (random kick magnitude)
 *   speed cap: pixels/second
 *
 * WHY IT EXISTS (vs inlining in scene_tick)
 *   scene_tick stays at "for each star: tick" — one line.  All the
 *   per-star physics (and there's a fair bit) lives here as one unit.
 *   If we ever wanted to swap wander for a different force model
 *   (gravity? attractor?), only this function changes.
 */
static void star_tick(Star *s, float dt, float max_px, float max_py)
{
    /* 1. save for lerp */
    s->prev_px = s->px;
    s->prev_py = s->py;

    /* 2. wander: random acceleration in [-WANDER_ACCEL, +WANDER_ACCEL] */
    float ax = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
    float ay = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
    s->vx += ax * dt;
    s->vy += ay * dt;

    /* 3. speed cap */
    float spd = sqrtf(s->vx * s->vx + s->vy * s->vy);
    if (spd > SPEED_CAP) {
        float inv = SPEED_CAP / spd;
        s->vx *= inv;
        s->vy *= inv;
    }

    /* 4. integrate */
    s->px += s->vx * dt;
    s->py += s->vy * dt;

    /* 5. elastic wall bounce */
    if (s->px < 0.0f)    { s->px = 0.0f;    s->vx = -s->vx; }
    if (s->px > max_px)  { s->px = max_px;   s->vx = -s->vx; }
    if (s->py < 0.0f)    { s->py = 0.0f;     s->vy = -s->vy; }
    if (s->py > max_py)  { s->py = max_py;   s->vy = -s->vy; }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * §6 PREAMBLE — orchestrator + the heart
 * ───────────────────────────────────────
 *
 * The Scene owns the star pool, the running configuration (paused state,
 * connect preset, current theme), and two operations:
 *
 *   scene_tick   advance every star by dt — pure state mutation, no I/O
 *   scene_draw   render one frame at sub-frame alpha — pure rendering
 *
 * Plus one helper (draw_line) that the pair loop in scene_draw uses.
 *
 * The TICK/DRAW SPLIT is the cleanest separation in this file.  Tick
 * never touches ncurses; draw never mutates state (const Scene *).
 * This means:
 *   - tick runs at fixed Hz; draw runs at render Hz; they don't fight
 *   - debug overlays can be added inside draw without touching physics
 *   - render frames can interpolate between tick boundaries (Tutorial #3)
 *
 * The pair loop in scene_draw is the algorithmic heart — O(N²/2)
 * inner-loop iterations per frame.  Skim through it with Tutorials
 * #6 (proximity graph) and #8 (cell_used) in mind; it should read
 * directly out of those pseudocode blocks.
 */

typedef struct {
    Star  stars[STARS_MAX];
    int   n;
    bool  paused;
    int   connect_preset;   /* index into k_connect_presets */
    int   current_theme;    /* index into k_themes[]        */
} Scene;

static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    sc->n              = STARS_DEFAULT;
    sc->connect_preset = 1;   /* "normal" */
    sc->current_theme  = 0;   /* "night"  */
    for (int i = 0; i < sc->n; i++)
        star_spawn(&sc->stars[i], i, cols, rows);
}

/*
 * scene_tick — advance every active star by dt.
 *
 * PURPOSE
 *   The full physics step for one frame.  Trivial because all the work
 *   is delegated to star_tick (per-star) — scene_tick just walks the
 *   active prefix of the pool and converts cell extents to pixel
 *   bounds.
 *
 * PSEUDOCODE
 *   if paused:  return
 *   max_px = pw(cols) − 1            // right wall in pixels
 *   max_py = ph(rows) − 1            // bottom wall in pixels
 *   for i = 0..n-1:
 *     star_tick(stars[i], dt, max_px, max_py)
 *
 * MENTAL MODEL
 *   The "active prefix" pattern: stars 0..n-1 are live; stars n..MAX-1
 *   are unused storage.  Adding a star with '+' bumps n by 1 (the slot
 *   already exists and was last touched at startup, so its state is
 *   whatever it was).  Removing with '-' decrements n — the star
 *   silently disappears, no resource to free.
 *
 *   No inter-star physics here.  Stars don't see each other during
 *   tick — only at DRAW time, when the proximity-graph pair scan
 *   reads their positions to decide which pairs get lines.
 *
 * INPUTS / OUTPUTS
 *   sc          ← Scene (mutated: every active star advances by dt)
 *   dt          → frame time, seconds
 *   cols, rows  → terminal extent (cells)
 *
 * UNITS
 *   dt:           seconds
 *   max_px/py:    pixels — derived from cells via pw/ph
 *
 * WHY IT EXISTS (vs inlining the loop in app's main)
 *   Same pattern as comet.c: keep app's main loop at orchestrator level
 *   (read input, tick, draw, sleep).  Encapsulating physics inside
 *   scene_tick means a future test harness or batch renderer can drive
 *   the same Scene without dragging in ncurses or signals.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    if (sc->paused) return;

    float max_px = (float)(pw(cols) - 1);
    float max_py = (float)(ph(rows) - 1);

    for (int i = 0; i < sc->n; i++)
        star_tick(&sc->stars[i], dt, max_px, max_py);
}

/*
 * draw_line — thin Bresenham with slope glyphs, stipple, and overlap skip.
 *
 * PURPOSE
 *   Rasterise one segment from (x0, y0) to (x1, y1) using ONE cell per
 *   major-axis step (Tutorial #7), pick a slope-matched glyph at every
 *   step from the LOCAL movement (not the overall angle), apply the
 *   stipple stride for distance fade (Tutorial #6 bucket 3), and respect
 *   the `used` bool grid so multiple overlapping lines don't garble
 *   each other (Tutorial #8).
 *
 * PSEUDOCODE
 *   adx, ady = |x1-x0|, |y1-y0|
 *   sx, sy   = sign(x1-x0), sign(y1-y0)
 *   diag     = (sx*sy > 0) ? '\\' : '/'      // fixed for the whole line
 *
 *   if adx == 0 and ady == 0:                // degenerate same-cell
 *     try-write 'diag' at (x0, y0);  return
 *
 *   if adx >= ady:                            // SHALLOW: one cell per column
 *     err = adx / 2
 *     for x in [x0 .. x1]  step sx:
 *       glyph = (next_err < 0) ? diag : '-'   // pick BEFORE advancing y
 *       try-write glyph at (x, y0)
 *       step++; err -= ady; if err < 0: y0 += sy; err += adx
 *   else:                                     // STEEP: one cell per row
 *     err = ady / 2
 *     for y in [y0 .. y1]  step sy:
 *       glyph = (next_err < 0) ? diag : '|'
 *       try-write glyph at (x0, y)
 *       step++; err -= adx; if err < 0: x0 += sx; err += ady
 *
 *   try-write(x, y, glyph):
 *     if out of bounds OR step % stipple != 0 OR used[y][x]: skip
 *     used[y][x] = true
 *     mvaddch(y, x, glyph)
 *
 * MENTAL MODEL
 *   Standard Bresenham emits a cell per axis step.  At 45° BOTH axes
 *   advance every step, so you get two cells per step → the doubled
 *   diagonal artifact.  Thin-Bresenham emits exactly ONE cell per
 *   MAJOR-axis step; the minor axis ride-along is folded into the
 *   glyph selection ('\\'/'/' when it advances, '-'/'|' when it doesn't).
 *
 *   Stipple: instead of antialiasing (which terminals can't do), draw
 *   only every Nth cell.  stipple=1 means every cell (solid line);
 *   stipple=2 means every other cell (fading line).  The eye reads
 *   the stippled lines as "further away" or "fading".
 *
 *   The `used` cell grid is the OVERLAP fix.  Without it: line A draws
 *   '\\' at cell (10, 5); later line B passes through and draws '─' on
 *   top — you see '─' and the diagonal is gone.  Multiplied across
 *   dozens of overlapping lines, the lattice becomes a smudged mess.
 *
 * INPUTS / OUTPUTS
 *   w           → destination WINDOW (typically stdscr)
 *   x0, y0      → segment start in CELL coords
 *   x1, y1      → segment end   in CELL coords
 *   attr        → ncurses attribute bundle (COLOR_PAIR + maybe A_BOLD)
 *   stipple     → 1 = solid; 2 = every other cell; N = every Nth
 *   cols, rows  → bounds for clipping + indexing `used`
 *   used        ← flat rows·cols bool grid; mutated (cells get marked)
 *
 * UNITS
 *   All coordinates in CELL space (caller already converted from pixels).
 *
 * WHY IT EXISTS (vs inlining one big nested loop in scene_draw)
 *   Bresenham + thin-line + stipple + overlap-skip in one function is
 *   long enough that inlining it inside the pair loop would push that
 *   loop past 100 lines and make the bucket-selection logic harder to
 *   read.  Splitting also lets the FLASH cross (if we ever add one)
 *   reuse the same thin Bresenham — just call draw_line with sample
 *   coordinates from a fixed pattern.
 */
static void draw_line(WINDOW *w,
                      int x0, int y0, int x1, int y1,
                      chtype attr, int stipple,
                      int cols, int rows,
                      bool *used)
{
    int adx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int ady = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int step = 0;
    char diag = (sx * sy > 0) ? '\\' : '/';

    if (adx == 0 && ady == 0) {
        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows
                && !used[y0 * cols + x0]) {
            used[y0 * cols + x0] = true;
            wattron(w, attr);
            mvwaddch(w, y0, x0, (chtype)(unsigned char)diag);
            wattroff(w, attr);
        }
        return;
    }

    if (adx >= ady) {
        /* shallow: one cell per column */
        int err = adx / 2;
        for (int x = x0; x != x1 + sx; x += sx) {
            int next_err = err - ady;
            char ch = (next_err < 0) ? diag : '-';
            if (x >= 0 && x < cols && y0 >= 0 && y0 < rows
                    && step % stipple == 0 && !used[y0 * cols + x]) {
                used[y0 * cols + x] = true;
                wattron(w, attr);
                mvwaddch(w, y0, x, (chtype)(unsigned char)ch);
                wattroff(w, attr);
            }
            step++;
            err = next_err;
            if (err < 0) { y0 += sy; err += adx; }
        }
    } else {
        /* steep: one cell per row */
        int err = ady / 2;
        for (int y = y0; y != y1 + sy; y += sy) {
            int next_err = err - adx;
            char ch = (next_err < 0) ? diag : '|';
            if (x0 >= 0 && x0 < cols && y >= 0 && y < rows
                    && step % stipple == 0 && !used[y * cols + x0]) {
                used[y * cols + x0] = true;
                wattron(w, attr);
                mvwaddch(w, y, x0, (chtype)(unsigned char)ch);
                wattroff(w, attr);
            }
            step++;
            err = next_err;
            if (err < 0) { x0 += sx; err += ady; }
        }
    }
}

/*
 * scene_draw — render one frame at sub-frame interpolation alpha.
 *
 * PURPOSE
 *   The full per-frame rendering: lerp every star's draw position,
 *   sweep all C(n, 2) pairs and draw connection lines for pairs
 *   within CONNECT_DIST, then paint star glyphs on top.  No state
 *   mutation — `Scene *` is const throughout.
 *
 * PSEUDOCODE
 *   PHASE A — pre-compute lerp'd positions:
 *     for i = 0..n-1:
 *       dpx[i] = prev_px + (px - prev_px) · alpha       // pixel space
 *       dpy[i] = prev_py + (py - prev_py) · alpha
 *       dcx[i] = px_to_cell_x(dpx[i])  (clamped to [0, cols))
 *       dcy[i] = px_to_cell_y(dpy[i])  (clamped to [0, rows))
 *
 *   PHASE B — connection lines (proximity graph, Tutorial #6):
 *     cell_used[rows][cols] = 0
 *     for i = 0..n-2:
 *       for j = i+1..n-1:
 *         d² = (dpx[j]-dpx[i])² + (dpy[j]-dpy[i])²
 *         if d² >= CONNECT_DIST²:  continue           // cull, no sqrt
 *         ratio = sqrt(d²) / CONNECT_DIST
 *         (attr, stipple) = bucket(ratio)             // 3-tier brightness
 *         draw_line(dcx[i], dcy[i], dcx[j], dcy[j],
 *                    attr, stipple, cell_used)
 *
 *   PHASE C — stars on top (so glyphs are never hidden by lines):
 *     for i = 0..n-1:
 *       mvaddch(dcy[i], dcx[i], stars[i].ch, stars[i].color | A_BOLD)
 *
 * MENTAL MODEL
 *   Two arrays (dpx, dpy) cache the pixel-space lerp; two more (dcx,
 *   dcy) cache the cell-space rounded positions.  Pre-computing means
 *   the pair loop only does floating-point math on the distance test,
 *   then hands integer cell coords to draw_line.  At N=80 stars,
 *   3160 pairs each doing 2 floats + 1 cmp + 1 sqrt (only if close)
 *   is well under any modern CPU's frame budget.
 *
 *   The order — lines first, stars last — is the painter's algorithm.
 *   Stars are the foreground; they must overwrite line glyphs that
 *   pass through their cell.  Reverse the order and stars would
 *   sometimes hide behind a '\\' or '-'.
 *
 *   cell_used is per-frame (zeroed at the start of phase B) and is
 *   the only allocation in the hot path — a stack VLA sized to the
 *   current terminal.  See Tutorial #8 for the "why first-writer
 *   wins" rationale.
 *
 * INPUTS / OUTPUTS
 *   sc          → Scene to read (const — never mutated)
 *   w           → destination WINDOW
 *   cols, rows  → terminal extent, used for bounds + cell_used sizing
 *   alpha       → ∈ [0.0, 1.0); 0 = at last tick, 1 = at next tick
 *
 * UNITS
 *   dpx, dpy:   pixels (interpolated)
 *   dcx, dcy:   cells (rounded)
 *   ratio:      dimensionless, ∈ [0, 1]
 *
 * WHY IT EXISTS (vs splitting into scene_draw_lines + scene_draw_stars)
 *   The cache arrays (dpx/dpy/dcx/dcy) are computed once at the top
 *   and read by both the line loop and the star loop.  Splitting
 *   would require either re-computing the lerp twice (wasted work)
 *   or threading a cached-positions struct through both halves
 *   (more typing for no clarity gain).  At ~70 lines this function
 *   reads cleanly top-to-bottom as one pipeline.
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha)
{
    float connect_dist = k_connect_presets[sc->connect_preset];
    float cdist_sq     = connect_dist * connect_dist;

    /* --- pre-compute lerp'd pixel and cell positions for all stars ---- */
    float dpx[STARS_MAX], dpy[STARS_MAX];
    int   dcx[STARS_MAX], dcy[STARS_MAX];

    for (int i = 0; i < sc->n; i++) {
        const Star *s = &sc->stars[i];
        dpx[i] = s->prev_px + (s->px - s->prev_px) * alpha;
        dpy[i] = s->prev_py + (s->py - s->prev_py) * alpha;
        dcx[i] = px_to_cell_x(dpx[i]);
        dcy[i] = px_to_cell_y(dpy[i]);
        if (dcx[i] < 0)     dcx[i] = 0;
        if (dcx[i] >= cols) dcx[i] = cols - 1;
        if (dcy[i] < 0)     dcy[i] = 0;
        if (dcy[i] >= rows) dcy[i] = rows - 1;
    }

    /* --- 1. connection lines ------------------------------------------ */

    /* One cell visited by any connection line is not redrawn by others.
     * This prevents the mixed-character thick-bundle look that appears
     * when several lines overlap in the same screen region. */
    bool cell_used[rows][cols];
    memset(cell_used, 0, sizeof cell_used);

    for (int i = 0; i < sc->n - 1; i++) {
        for (int j = i + 1; j < sc->n; j++) {

            float dx_px   = dpx[j] - dpx[i];
            float dy_px   = dpy[j] - dpy[i];
            float dist_sq = dx_px * dx_px + dy_px * dy_px;

            if (dist_sq >= cdist_sq) continue;

            float dist  = sqrtf(dist_sq);
            float ratio = dist / connect_dist;   /* 0.0 (close) → 1.0 (edge) */

            chtype attr;
            int    stipple;

            if (ratio < 0.50f) {
                attr    = COLOR_PAIR(CONN_PAIR) | A_BOLD;
                stipple = 1;
            } else if (ratio < 0.75f) {
                attr    = COLOR_PAIR(CONN_PAIR);
                stipple = 1;
            } else {
                attr    = COLOR_PAIR(CONN_PAIR);
                stipple = 2;
            }

            draw_line(w,
                      dcx[i], dcy[i], dcx[j], dcy[j],
                      attr, stipple,
                      cols, rows,
                      &cell_used[0][0]);
        }
    }

    /* --- 2. stars — drawn after lines so they are always on top ------- */
    for (int i = 0; i < sc->n; i++) {
        const Star *s = &sc->stars[i];
        wattron(w, COLOR_PAIR(s->color) | A_BOLD);
        mvwaddch(w, dcy[i], dcx[i], (chtype)(unsigned char)s->ch);
        wattroff(w, COLOR_PAIR(s->color) | A_BOLD);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen wraps the ncurses single-window model.
 *
 * One WINDOW (stdscr), one flush (doupdate).  ncurses' internal
 * curscr / newscr pair IS the double buffer — no extra WINDOW needed.
 * erase() → mvwaddch() → mvprintw() → wnoutrefresh() → doupdate().
 * HUD is drawn last so it always overlays the scene.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s, int theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);      /* never interrupt output to check for input    */
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();          /* re-reads LINES and COLS                      */
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * HUD layout per CLAUDE.md:
 *   row 0           — fps + stars + conn + spd + theme + state, bright bold yellow (top-right)
 *   row rows-1      — every interactive key listed, bright bold cyan (bottom-left)
 * Both pairs (HUD_PAIR=8, HINT_PAIR=9) are theme-independent — they live
 * outside the 1..7 range that theme_apply touches.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  stars:%-2d  conn:%-6s  spd:%-2d  [%s]  %s ",
             fps, sc->n,
             k_connect_names[sc->connect_preset],
             sim_fps,
             k_themes[sc->current_theme].name,
             sc->paused ? "PAUSED " : "running");
    int hud_x = s->cols - (int)strlen(buf);
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    attron(COLOR_PAIR(HINT_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q/ESC:quit  spc:pause  +/-:stars  c:connect"
             "  r:randomise  ]/[:speed  t/T:theme ");
    attroff(COLOR_PAIR(HINT_PAIR) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

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

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    int   cols = app->screen.cols;
    int   rows = app->screen.rows;
    float mpx  = (float)(pw(cols) - 1);
    float mpy  = (float)(ph(rows) - 1);
    for (int i = 0; i < app->scene.n; i++) {
        Star *s = &app->scene.stars[i];
        if (s->px      > mpx) s->px      = mpx;
        if (s->py      > mpy) s->py      = mpy;
        if (s->prev_px > mpx) s->prev_px = mpx;
        if (s->prev_py > mpy) s->prev_py = mpy;
    }
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *sc   = &app->scene;
    int    cols = app->screen.cols;
    int    rows = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        for (int i = 0; i < sc->n; i++)
            star_spawn(&sc->stars[i], i, cols, rows);
        break;

    case '=': case '+':
        if (sc->n < STARS_MAX) {
            star_spawn(&sc->stars[sc->n], sc->n, cols, rows);
            sc->n++;
        }
        break;

    case '-':
        if (sc->n > STARS_MIN) sc->n--;
        break;

    case 'c': case 'C':
        sc->connect_preset = (sc->connect_preset + 1) % N_CONNECT_PRESETS;
        break;

    case 't':
        sc->current_theme = (sc->current_theme + 1) % N_THEMES;
        theme_apply(sc->current_theme);
        break;
    case 'T':
        sc->current_theme = (sc->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(sc->current_theme);
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
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, app->scene.current_theme);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;  /* pause guard */

        /* ── fixed-timestep sim accumulator ─────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /*
         * alpha = fractional tick elapsed since last physics step.
         * Passed to scene_draw for lerp interpolation.
         *   alpha = 0.0 → draw at ticked position (no change)
         *   alpha = 0.9 → draw 90 % of the way toward the next tick
         */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap ───────────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ─────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
