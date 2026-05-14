/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * smoke.c  —  ncurses ASCII smoke, five physics algorithms
 *
 * ALGORITHMS (cycle with 'a'):
 *
 *   0  Particle Puffs   — Pool of MAX_PARTS Lagrangian particles born at the
 *                         source zone with upward velocity and a random
 *                         lifetime.  Density = life² (quadratic fade).
 *                         Particles bilinear-splat onto the float density
 *                         grid before the shared rendering pipeline runs.
 *
 *   Algos 1..4 are all Eulerian semi-Lagrangian advection — same back-trace
 *   + bilinear-sample + decay + arch-source pipeline; only the velocity field
 *   differs:
 *
 *   1  Vortex Advection — N_VORTS=3 orbiting point vortices generate the
 *                         velocity field via 2D Biot-Savart.  Mixed +/−
 *                         chirality gives counter-rotating eddies; visible
 *                         signature is obvious curls and swirls.
 *
 *   2  Curl Noise       — velocity = curl of a smooth scalar noise potential
 *                         (divergence-free by construction).  Smooth
 *                         distributed flow, no obvious centres; smoke
 *                         meanders organically over the whole frame.
 *
 *   3  Buoyancy Plume   — vy = −rise · density[here]  (Boussinesq: hot rises).
 *                         Density doubles as temperature; tall thin plumes
 *                         that mushroom-cap where buoyancy balances decay.
 *
 *   4  Breeze           — vx(y, t) = amp · sin(t + y · k); vy = small rise.
 *                         Closed-form laminar pattern.  Smoke rises as a
 *                         waving curtain, each row at its own sway phase.
 *
 * All five algorithms write into the same [rows × cols] float density grid.
 * The shared rendering path runs Floyd-Steinberg dithering then maps density
 * to one of 9 ASCII chars via a perceptual LUT (identical to fire.c).
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   a         next algorithm (cycle 0..4)
 *   t / T     next / previous theme
 *   g  G      source intensity up / down
 *   w  W      wind right / left
 *   0         calm (no wind)
 *   ]  [      sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra smoke.c -o smoke -lncurses -lm
 *
 * Sections
 * --------
 *   §1  presets       — all tunable constants, grouped by sub-system
 *   §2  clock
 *   §3  theme         — 10 palettes, Floyd-Steinberg + LUT pipeline
 *   §4  shared helpers— warmup_scale, clampf, bilinear_sample,
 *                       SLCtx + semi-Lagrangian kernel (sl_step_cell)
 *   §5  algo 0        — particle puffs
 *   §6  algo 1        — vortex advection
 *   §7  algo 2        — curl-noise advection
 *   §8  algo 3        — buoyancy plume
 *   §9  algo 4        — breeze advection
 *   §10 scene         — owns all state, dispatches tick / draw
 *   §11 screen        — ncurses layer + HUD
 *   §12 app           — main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm 0 — Particle System (Reeves):
 *   Particles carry (x, y, vx, vy, life).  Each tick: turbulence random
 *   walk on vx, upward vy drift, life decreases.  density = life² gives a
 *   quadratic fade so particles are bright at birth and fade out smoothly.
 *   Bilinear splat distributes density across 4 surrounding grid cells
 *   (1-pixel tent filter) for soft-edged smoke puffs.
 *
 * Algorithms 1..4 — Eulerian semi-Lagrangian (Stam):
 *   Four different ways to define the velocity field that stirs the
 *   density grid.  Each tick, for every cell:
 *       v   = velocity_field_at(x, y, t)
 *       sx  = x − vx·ADV_DT;  sy = y − vy·ADV_DT     (back-trace)
 *       new = bilinear_sample(density, sx, sy) · (1 − decay) + source(p)
 *   Differs only in how the velocity field is computed:
 *
 *   1 — Vortex Advection:
 *       N_VORTS=3 orbiting point vortices each contribute via 2D Biot-Savart
 *       vx += strength × (−dy) / (r² + VORT_EPS)
 *       vy += strength × ( dx) / (r² + VORT_EPS)
 *       Mixed +/− chirality across vortices → counter-rotating eddies.
 *
 *   2 — Curl-Noise Advection:
 *       velocity = curl of a smooth scalar noise potential.  Divergence-
 *       free by construction (∇·v ≡ 0) — density is conserved without
 *       artificial sinks.  Time-varying via t-offset on the noise coord.
 *       Visible signature: smooth distributed flow, no obvious centres.
 *
 *   3 — Buoyancy Plume (Boussinesq):
 *       vy = −BUOY_RISE · density[here]   (hot rises, cold sinks)
 *       vx = mild noise-driven horizontal turbulence
 *       Density doubles as temperature.  Visible signature: tall thin
 *       plume that mushroom-caps where buoyancy balances decay.
 *
 *   4 — Breeze Advection:
 *       vx(y, t) = BREEZE_AMP · sin(t + y · BREEZE_K)   (laminar sway)
 *       vy       = −BREEZE_RISE                          (gentle rise)
 *       Closed-form 1-D pattern, no noise table.  Visible signature:
 *       smoke rises as a waving curtain, each row at its own phase.
 *
 *   All four share the same CFL clamp (ADV_VEL_CAP=2 cells/tick) and the
 *   same arch-shaped source injection on the bottom row.  Wind is
 *   accumulated once in scene_tick() and passed to each algo — algos
 *   must NOT advance wind_acc themselves.
 *
 * Rendering (shared):
 *   Same Floyd-Steinberg + gamma-corrected LUT pipeline as fire.c.
 *   Ramp: " .,:coO0#"  (soft round chars for a smoky feel).
 *   Only cells that were non-zero last frame and are now zero get an
 *   explicit erase — same borderless diff trick as fire.c.
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  Algo 1 (Particle System) is a direct
 *       Reeves system — pool, life-decay, per-particle update with
 *       turbulence + drift.  The bilinear splat that scatters life²
 *       across 4 cells is the ASCII-cell analogue of Reeves' point-
 *       sprite rendering.
 *
 *     Stam, J. (1999)
 *       "Stable Fluids"
 *       SIGGRAPH '99 Proceedings: 121-128.
 *       THE paper for semi-Lagrangian advection.  Shared by Algos
 *       1..4 (vortex, curl, buoy, breeze) via the sl_step_cell
 *       kernel: the back-trace formula new_density(p) =
 *       bilinear(old, p − v·ΔT) and the CFL stability bound
 *       (ADV_DT·|v_max| ≤ 1.6 cells) come straight from Stam §3.
 *       The velocity-cap at ADV_VEL_CAP=2 is exactly the CFL clamp
 *       Stam recommends for unconditional stability.
 *
 *     Floyd, R. W. & Steinberg, L. (1976)
 *       "An Adaptive Algorithm for Spatial Greyscale"
 *       Proc. Society for Information Display 17(2): 75-77.
 *       The 7/16, 3/16, 5/16, 1/16 error-diffusion mask scene_draw
 *       uses to dither the continuous density grid into the 9-step
 *       ASCII ramp.  Same coefficients, two dimensions short of a
 *       printer driver.
 *
 *   BOOKS
 *     Bridson, R. — "Fluid Simulation for Computer Graphics"
 *       (2nd ed, CRC Press, 2015).
 *       Definitive textbook on grid-based fluid sim.  Chapter 3
 *       covers semi-Lagrangian advection — the shared kernel that
 *       drives Algos 1..4.  §2.3 on Biot-Savart point vortices
 *       grounds the v = strength · (-dy, dx) / (r² + ε) formula
 *       Algo 1 (vortex_tick) uses, and the role of ε (= VORT_EPS)
 *       as a regulariser preventing the singularity at the vortex
 *       centre.
 *
 *     Witkin, A. & Baraff, D. (2001)
 *       "Physically Based Modeling: Principles and Practice"
 *       SIGGRAPH course notes (online proceedings).  §1 — particle
 *       dynamics with explicit force accumulation, the pattern Algo
 *       1's per-particle update follows (vx += turbulence, vx *=
 *       0.97 damping, life -= decay).
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. — "Real-Time
 *       Rendering" (4th ed, CRC Press, 2018).
 *       §5.6 — gamma correction (the pow(density, 1/2.2) in
 *       scene_draw maps linear-light density to perceptually-uniform
 *       brightness); §13.7 — point-sprite particle rendering with
 *       soft-edged tent-filter falloff, the bilinear splat being the
 *       cell-grid analogue.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Five different ways of stirring a single shared float density grid,
 * then one shared dithered renderer that maps density to round soft
 * glyphs.  Algo 0 (particle) flies puffs upward and bilinear-splats
 * their life² onto 4 neighbouring cells (soft tent-filter blobs).
 * Algos 1..4 are all Eulerian semi-Lagrangian advections sharing the
 * same back-trace + bilinear-sample + decay + arch-source pipeline,
 * differing only in HOW THEY COMPUTE THE VELOCITY at each cell:
 *   1 (vortex) — sum of Biot-Savart contributions from 3 orbiting
 *                point vortices (curls + swirls are the streamlines)
 *   2 (curl)   — curl of a smooth scalar noise field (divergence-free
 *                distributed flow, no centres)
 *   3 (buoy)   — vy = −rise·density (hot rises, mushroom-cap plumes)
 *   4 (breeze) — vx(y,t) = amp·sin(t + y·k)  (swaying laminar curtain)
 *
 * ALGORITHM IN STEPS  (per tick)
 * ──────────────────
 *  scene_tick (always):
 *    1. wind_acc += wind; wrap at ±cols.
 *    2. dispatch by algo.
 *
 *  Algo 0 — Particle Puffs (particle_tick):
 *    a. for each active particle:
 *         vx += turb±0.06; vx *= 0.97; x+=vx; y+=vy; life-=decay
 *         deactivate if life≤0 or off-grid.
 *    b. spawn SPAWN_PER_TICK new particles via arch-rejection sampling.
 *    c. clear density grid; bilinear-splat life² into 4 cells per
 *       particle:  weight(x0,y0) = (1-tx)(1-ty), etc.
 *    d. clamp density ≤ 1.
 *
 *  Algos 1..4 — Semi-Lagrangian advection (vortex_tick / curl_tick /
 *               buoy_tick / breeze_tick).  Identical structure; only
 *               the velocity-field computation differs.
 *    a. compute decay from VORT_REACH_FRAC and rows.
 *    b. for each cell (x,y):
 *       — compute v = velocity_field_for_this_algo(x, y, t)
 *                  1 vortex : sum of Biot-Savart over N_VORTS
 *                  2 curl   : (∂P/∂y, −∂P/∂x) where P is curl_noise2d
 *                  3 buoy   : (turb·noise(x,y,t), −rise·density[here])
 *                  4 breeze : (amp·sin(t + y·k), −rise)
 *       — clamp |v| ≤ ADV_VEL_CAP=2 (CFL stability).
 *       — sx = x − vx·ADV_DT; sy = y − vy·ADV_DT (back-trace).
 *       — adv = bilinear_sample(density, sx, sy).
 *       — src = arch·intensity·jitter·wscale on bottom row only.
 *       — work[y][x] = adv·(1−decay) + src.
 *    c. memcpy work → density.
 *
 *  scene_draw (shared):
 *    1. for each density cell: if d=0 mark −1; else d = pow(d,1/2.2).
 *    2. Floyd-Steinberg dither: err = d − lut_midpoint(idx);
 *       diffuse 7/16, 3/16, 5/16, 1/16 to neighbours.
 *    3. mvaddch(y, x, k_ramp[idx]) with theme attribute.
 *    4. cells that were non-zero last frame but zero now → ' '.
 *    5. swap density ↔ prev_density.
 *
 * KEY FORMULAS
 * ────────────
 *  Arch envelope:
 *      t = (x − margin − wind_acc) / span
 *      arch = (min(t, 1−t) · 2)²                    0 at edges, 1 ctr
 *
 *  Particle bilinear splat (life² distributed across 4 cells):
 *      w00 = (1-tx)(1-ty)   w10 = tx(1-ty)
 *      w01 = (1-tx)ty       w11 = tx·ty
 *      density[y0..y1][x0..x1] += life² · w
 *
 *  Semi-Lagrangian advection (shared by algos 1..4):
 *      density'(p) = density(p − v(p)·ΔT)·(1−decay) + source(p)
 *      ΔT = ADV_DT = 0.8                            CFL: |v|·ΔT ≤ 1.6 cells
 *
 *  Velocity fields per algo:
 *      1 vortex : vx += s·(−dy)/(r² + VORT_EPS)     Biot-Savart point vortex
 *                 vy += s·( dx)/(r² + VORT_EPS)
 *                 ε = 6.0 prevents singularity at vortex centre
 *                 orbits r·{0.20, 0.30, 0.18}·cols, ω ∈ {+0.018, −0.011, +0.025}
 *                 strengths {+2.5, −1.8, +1.4} → mixed chirality
 *
 *      2 curl   : vx =  CURL_AMP · (P(x, y+h) − P(x, y-h)) / 2h
 *                 vy = −CURL_AMP · (P(x+h, y) − P(x-h, y)) / 2h
 *                 P = curl_noise2d(scale·x, scale·y + t)
 *                 vy −= CURL_UPWARD_BIAS                 (smoke rises)
 *
 *      3 buoy   : vx = (curl_noise2d(...) − 0.5) · 2·BUOY_TURB_AMP
 *                 vy = −BUOY_RISE · density[here]       Boussinesq buoyancy
 *
 *      4 breeze : vx = BREEZE_AMP · sin(t + y · BREEZE_K)
 *                 vy = −BREEZE_RISE
 *
 *  Dither + LUT:
 *      d = pow(density, 1/2.2)                      gamma to perceptual
 *      idx = lut_index(d) ∈ 0..8
 *      glyph = " .,:coO0#"[idx]
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
#include <stdio.h>

/* ===================================================================== */
/* §1  presets                                                            */
/* ===================================================================== */

/* ── loop / display ─────────────────────────────────────────────────── */
enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    HUD_COLS        = 64,
    FPS_UPDATE_MS   = 500,

    N_ALGOS         =  5,
    MAX_PARTS       = 400,   /* particle pool size                          */
    N_VORTS         =  3,    /* number of orbiting vortices                 */
};

#define WIND_MAX    3        /* max wind offset in cells/tick               */

/* ── source zone (shared by all five algos) ──────────────────────────
 * The smoke source is an arch shape along the bottom row.
 *   ARCH_MARGIN_FRAC  : fraction of cols kept empty at each side edge
 *   SRC_JITTER_BASE   : minimum random multiplier on source (prevents flat line)
 *   SRC_JITTER_RANGE  : random range added on top of BASE (0 → RANGE)
 *   WARMUP_TICKS      : linear ramp 0→1 at startup so smoke builds gradually
 *   WARMUP_CAP        : warmup counter is clamped here (prevents int overflow
 *                       and keeps warmup_scale() at exactly 1.0 after startup)
 * ─────────────────────────────────────────────────────────────────────*/
#define ARCH_MARGIN_FRAC  0.06f   /* 6% of cols kept empty at each side    */
#define SRC_JITTER_BASE   0.80f   /* min random multiplier on source       */
#define SRC_JITTER_RANGE  0.20f   /* extra random range 0→0.20            */
#define WARMUP_TICKS      80      /* ramp 0→1 over first 80 ticks         */
#define WARMUP_CAP        200     /* counter capped here; scale stays 1.0  */

/* ── algo 0: particle puffs ─────────────────────────────────────────────
 *   PART_LIFE_MIN/RANGE  : lifetime = MIN + rand×RANGE ticks
 *   PART_VY_BASE/RANGE   : upward speed = BASE + rand×RANGE (vy negative)
 *   PART_VX_SPREAD       : birth lateral kick ±SPREAD/2
 *   PART_TURB_STEP       : per-tick turbulence on vx (random ±TURB/2)
 *   PART_VX_DAMP         : vx damping per tick
 *   SPAWN_PER_TICK       : new particles each tick
 * ─────────────────────────────────────────────────────────────────────*/
#define PART_LIFE_MIN      35.f
#define PART_LIFE_RANGE    35.f
#define PART_VY_BASE       0.25f
#define PART_VY_RANGE      0.30f
#define PART_VX_SPREAD     0.4f
#define PART_TURB_STEP     0.12f
#define PART_VX_DAMP       0.97f
#define SPAWN_PER_TICK     5

/* ── shared semi-Lagrangian config (used by algos 1..4) ────────────────
 *   ADV_DT             : semi-Lagrangian time step (cells/tick); ≤1 for stability
 *   ADV_VEL_CAP        : clamp every velocity field so back-trace ≤ 2 cells away
 *   VORT_REACH_FRAC    : target smoke height as fraction of rows (for decay)
 *   VORT_DECAY_SCALE   : decay = (1/target) × this
 *   VORT_DECAY_MIN     : floor on decay so tiny terminals still dissipate
 * ─────────────────────────────────────────────────────────────────────*/
#define ADV_DT            0.8f
#define ADV_VEL_CAP       2.0f
#define VORT_REACH_FRAC   0.55f
#define VORT_DECAY_SCALE  0.9f
#define VORT_DECAY_MIN    0.010f

/* ── algo 1: vortex advection ───────────────────────────────────────────
 *   VORT_EPS           : Biot-Savart softening (avoids singularity at centre)
 *
 * Vortex orbital presets (indices match N_VORTS=3):
 *   VORT_ORB_FRACS[]   : orbit radius as fraction of cols
 *   VORT_ORB_SPDS[]    : orbital angular speed (rad/tick); negative = clockwise
 *   VORT_STRENGTHS[]   : Biot-Savart strength; positive = CCW, negative = CW
 *   VORT_INIT_ANGLES[] : starting orbital angle (radians)
 * ─────────────────────────────────────────────────────────────────────*/
#define VORT_EPS          6.0f

static const float VORT_ORB_FRACS[N_VORTS]   = { 0.20f,  0.30f,  0.18f };
static const float VORT_ORB_SPDS[N_VORTS]    = { 0.018f,-0.011f, 0.025f };
static const float VORT_STRENGTHS[N_VORTS]   = { 2.5f,  -1.8f,   1.4f  };
static const float VORT_INIT_ANGLES[N_VORTS] = { 0.0f,   2.1f,   4.3f  };

/* ── algo 2: curl-noise advection ───────────────────────────────────────
 *   CURL_SCALE         : noise frequency (cells⁻¹); higher = tighter swirls
 *   CURL_AMP           : peak curl velocity in cells/tick
 *   CURL_TIME_RATE     : how fast the noise field morphs (rad-ish/tick)
 *   CURL_UPWARD_BIAS   : constant upward push added so smoke rises
 * ─────────────────────────────────────────────────────────────────────*/
#define CURL_SCALE         0.10f
#define CURL_AMP           3.5f
#define CURL_TIME_RATE     0.012f
#define CURL_UPWARD_BIAS   0.5f

/* ── algo 3: buoyancy plume ─────────────────────────────────────────────
 *   BUOY_RISE          : upward velocity = -BUOY_RISE · density (hot rises)
 *   BUOY_TURB_AMP      : horizontal turbulence amplitude (cells/tick)
 *   BUOY_TURB_SCALE    : turbulence noise frequency
 *   BUOY_TURB_RATE     : how fast the turbulence field morphs
 * ─────────────────────────────────────────────────────────────────────*/
#define BUOY_RISE          2.5f
#define BUOY_TURB_AMP      0.6f
#define BUOY_TURB_SCALE    0.18f
#define BUOY_TURB_RATE     0.020f

/* ── algo 4: breeze advection ───────────────────────────────────────────
 *   BREEZE_AMP         : peak horizontal sway in cells/tick
 *   BREEZE_K           : sine wavelength along y (rad/cell)
 *   BREEZE_RATE        : sine phase advance per tick (rad/tick)
 *   BREEZE_RISE        : constant upward drift in cells/tick
 * ─────────────────────────────────────────────────────────────────────*/
#define BREEZE_AMP         1.4f
#define BREEZE_K           0.25f
#define BREEZE_RATE        0.05f
#define BREEZE_RISE        0.6f

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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  theme + rendering pipeline                                         */
/* ===================================================================== */

/*
 * Ramp — ASCII chars ordered light-to-dense, chosen for soft smoky shapes.
 *
 *   ' '  empty / transparent
 *   '.'  wispy trail
 *   ','  thin curl
 *   ':'  light body
 *   'c'  billow edge  (round, small)
 *   'o'  billow mid   (round)
 *   'O'  dense billow
 *   '0'  opaque core
 *   '#'  thick / black smoke
 */
static const char k_ramp[] = " .,:coO0#";
#define RAMP_N (int)(sizeof k_ramp - 1)   /* 9 */

#define CP_BASE 1                       /* CP_BASE .. CP_BASE+RAMP_N-1 = 1..9 (theme ramp) */
#define PAIR_HUD  (CP_BASE + RAMP_N)     /* 10 — top status bar, bright yellow, theme-independent */
#define PAIR_HINT (CP_BASE + RAMP_N + 1) /* 11 — bottom key hints,  bright cyan,   theme-independent */

/*
 * LUT break points — gamma-corrected density thresholds per ramp level.
 * Bunched in the 0.2–0.7 range so mid-density billows use the most chars
 * (most visible part of a smoke column).
 */
static const float k_lut_breaks[RAMP_N] = {
    0.000f,  /* ' '  empty        */
    0.060f,  /* '.'  wisp         */
    0.150f,  /* ','  thin         */
    0.260f,  /* ':'  light        */
    0.370f,  /* 'c'  billow edge  */
    0.480f,  /* 'o'  billow mid   */
    0.600f,  /* 'O'  dense        */
    0.740f,  /* '0'  opaque       */
    0.880f,  /* '#'  thick        */
};

static int lut_index(float v)
{
    int idx = 0;
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (v >= k_lut_breaks[i]) { idx = i; break; }
    return idx;
}

static float lut_midpoint(int idx)
{
    if (idx <= 0)        return 0.f;
    if (idx >= RAMP_N-1) return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx+1]) * 0.5f;
}

/*
 * Themes — 10 smoke palettes.  Each is a 9-step foreground ramp from
 * faint wispy edge (ramp[0]) to dense bright core (ramp[8]).  All
 * entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule — even the faintest
 * wisp is visible against a dark terminal background.
 *
 * Cycled by t / T (forward / back).  Default = MATRIX (idx 0).
 *
 *   0  MATRIX  — dark green → lime → cream      (digital rain smoke)
 *   1  FIRE    — dark red → orange → yellow     (fire-lit smoke)
 *   2  OCEANIC — deep teal → cyan → white       (underwater plume)
 *   3  NEON    — violet → pink → white          (retro-arcade haze)
 *   4  MONO    — gray ramp → white              (classic chimney)
 *   5  ICE     — navy → pale blue → white       (frozen vapour)
 *   6  NOVA    — deep blue → white → yellow     (supernova plume)
 *   7  FOREST  — dark green → gold → cream      (canopy mist)
 *   8  DESERT  — wine → tan → cream             (dust storm)
 *   9  ECLIPSE — dark red → peach               (bloodmoon vapour)
 */
typedef struct {
    const char *name;
    int         fg256[RAMP_N];
    int         fg8[RAMP_N];
    attr_t      attr8[RAMP_N];
} SmokeTheme;

static const SmokeTheme k_themes[] = {
    {   /* 0  MATRIX — dark green → lime → cream */
        "MATRIX",
        {  28,  34,  40,  46,  82, 118, 154, 190, 230 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
          COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE },
        { A_DIM,    A_DIM,    A_NORMAL, A_NORMAL, A_NORMAL,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 1  FIRE — dark red → orange → yellow */
        "FIRE",
        {  88, 124, 130, 166, 202, 208, 214, 220, 226 },
        { COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
          COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
        { A_NORMAL, A_NORMAL, A_NORMAL, A_BOLD,   A_NORMAL,
          A_NORMAL, A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 2  OCEANIC — deep teal → cyan → white */
        "OCEANIC",
        {  24,  31,  38,  44,  51,  87, 123, 159, 231 },
        { COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN, COLOR_CYAN,
          COLOR_CYAN,  COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        { A_DIM,    A_NORMAL, A_NORMAL, A_BOLD,   A_BOLD,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 3  NEON — violet → pink → white */
        "NEON",
        {  53,  91, 134, 165, 201, 207, 213, 219, 225 },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
          COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE },
        { A_NORMAL, A_NORMAL, A_BOLD,   A_BOLD,   A_BOLD,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 4  MONO — gray ramp → white */
        "MONO",
        { 242, 244, 245, 247, 248, 250, 251, 253, 255 },
        { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
          COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        { A_DIM,    A_DIM,    A_NORMAL, A_NORMAL, A_NORMAL,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 5  ICE — navy → pale blue → white */
        "ICE",
        {  24,  31,  67,  75, 117, 153, 195, 230, 231 },
        { COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN,  COLOR_CYAN,
          COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        { A_DIM,    A_NORMAL, A_NORMAL, A_BOLD,   A_BOLD,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 6  NOVA — deep blue → white → yellow */
        "NOVA",
        {  60,  75, 117, 159, 195, 219, 220, 226, 231 },
        { COLOR_BLUE,   COLOR_BLUE,   COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE,
          COLOR_WHITE,  COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
        { A_NORMAL, A_NORMAL, A_BOLD,   A_BOLD,   A_BOLD,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 7  FOREST — dark green → gold → cream */
        "FOREST",
        {  28,  64,  70, 112, 148, 154, 184, 220, 230 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW,
          COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
        { A_DIM,    A_NORMAL, A_NORMAL, A_BOLD,   A_BOLD,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 8  DESERT — wine → tan → cream */
        "DESERT",
        {  94, 130, 137, 173, 179, 215, 222, 229, 230 },
        { COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
          COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE },
        { A_NORMAL, A_NORMAL, A_BOLD,   A_NORMAL, A_BOLD,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
    {   /* 9  ECLIPSE — dark red → peach */
        "ECLIPSE",
        {  52,  88,  95, 131, 167, 173, 209, 215, 217 },
        { COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED,
          COLOR_RED, COLOR_RED, COLOR_RED, COLOR_WHITE },
        { A_DIM,    A_NORMAL, A_NORMAL, A_NORMAL, A_BOLD,
          A_BOLD,   A_BOLD,   A_BOLD,   A_BOLD }
    },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static void theme_apply(int t)
{
    const SmokeTheme *th = &k_themes[t];
    for (int i = 0; i < RAMP_N; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
        else
            init_pair(CP_BASE + i, th->fg8[i],   COLOR_BLACK);
    }
}

static void color_init(int theme)
{
    start_color();
    theme_apply(theme);
    /* Theme-independent HUD bars — bright high-contrast colours per
     * CLAUDE.md "HUD Standard" so they stay legible against any theme. */
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(PAIR_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(PAIR_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
}

static attr_t ramp_attr(int i, int theme)
{
    attr_t a = COLOR_PAIR(CP_BASE + i);
    if (COLORS >= 256) {
        if (i >= RAMP_N - 2) a |= A_BOLD;
    } else {
        a |= k_themes[theme].attr8[i];
    }
    return a;
}

/* ===================================================================== */
/* §4  shared helpers                                                     */
/* ===================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * warmup_scale() — linear ramp 0→1 over the first WARMUP_TICKS ticks.
 *
 * *warmup is the counter stored in the Scene; it is incremented here and
 * clamped at WARMUP_CAP so it never wraps and the scale stays at 1.0
 * after the warmup period ends.  Call once per tick per algo function.
 */
static float warmup_scale(int *warmup)
{
    float s = (*warmup < WARMUP_TICKS) ? (float)*warmup / (float)WARMUP_TICKS : 1.f;
    (*warmup)++;
    if (*warmup > WARMUP_CAP) *warmup = WARMUP_CAP;
    return s;
}

/*
 * bilinear_sample() — sample float grid at non-integer position (sx, sy).
 * Clamps at boundaries (Neumann: zero gradient at edges).
 */
static float bilinear_sample(const float *grid, float sx, float sy,
                              int cols, int rows)
{
    int x0 = (int)sx, y0 = (int)sy;
    int x1 = x0 + 1,  y1 = y0 + 1;

    if (x0 < 0)     x0 = 0;
    if (x0 >= cols) x0 = cols - 1;
    if (x1 < 0)     x1 = 0;
    if (x1 >= cols) x1 = cols - 1;
    if (y0 < 0)     y0 = 0;
    if (y0 >= rows) y0 = rows - 1;
    if (y1 < 0)     y1 = 0;
    if (y1 >= rows) y1 = rows - 1;

    float tx = sx - (float)(int)sx;
    float ty = sy - (float)(int)sy;
    if (tx < 0.f) tx = 0.f;
    if (tx > 1.f) tx = 1.f;
    if (ty < 0.f) ty = 0.f;
    if (ty > 1.f) ty = 1.f;

    float v00 = grid[y0*cols+x0], v10 = grid[y0*cols+x1];
    float v01 = grid[y1*cols+x0], v11 = grid[y1*cols+x1];

    return (1.f-tx)*(1.f-ty)*v00 + tx*(1.f-ty)*v10
         + (1.f-tx)*ty      *v01 + tx*ty      *v11;
}

/* ── Semi-Lagrangian shared kernel (used by algos 1..4) ──────────────── *
 *
 * REFERENCES:
 *   Stam, J. (1999) "Stable Fluids", SIGGRAPH '99 Proc.: 121-128.
 *     §3 — the unconditionally-stable advection scheme: for each grid
 *     cell, BACK-TRACE one Δt along the velocity field to find where the
 *     fluid that's NOW at (x, y) came FROM, then bilinear-sample the
 *     previous-frame density there.  No explicit forward step → no CFL
 *     blow-up regardless of |v|·Δt.  We still clamp velocity (Stam's
 *     §3.2 advice) so the sample point stays in a small neighbourhood
 *     where the interpolation is accurate.
 *
 *   Bridson, R. (2015) "Fluid Simulation for Computer Graphics", 2nd ed.
 *     Chapter 3 covers semi-Lagrangian advection in textbook detail.
 *     The CFL stability bound (ADV_DT · |v_max| ≤ a few cells) we use
 *     directly is from §3.5.
 *
 * SCHEMA (the inner loop body the helper executes per cell):
 *   v   = velocity_field_at(x, y, t)    THE algo-specific part
 *   v   = clamp(v, ±ADV_VEL_CAP)         CFL stability clamp
 *   sx  = x − vx · ADV_DT                back-trace one step
 *   sy  = y − vy · ADV_DT
 *   adv = bilinear_sample(density, sx, sy)
 *   src = sl_source_at(c, x, y) · intensity · jitter · wscale   (if y == fy)
 *   new = adv · (1 − decay) + src        Stam Eq. 14-ish
 *
 * The only thing that varies between vortex / curl / buoy / breeze is
 * how the velocity is computed.  Everything else — CFL clamp, back-trace,
 * bilinear sample, arch source on bottom row, blend with decay, clamp
 * to [0, 1] — is identical.  Factoring those into one helper makes each
 * algo a tight pseudocode listing where only the velocity model differs
 * (which is also where the conceptual difference between the algos lives).
 *
 * SLCtx — per-tick context object.
 *
 * INTENT: pack all the loop-invariant constants the SL inner loop needs
 * into one struct passed by pointer, instead of threading 10+ scalar
 * arguments through `sl_step_cell()` for every (x, y).  Computed once
 * per tick by `sl_make_ctx`, read read-only by the per-cell helper.
 * Reading a struct via pointer is the same cost as reading individual
 * locals — the compiler will register-allocate the fields it uses.
 *
 *   density   : READ pointer — previous-frame density field, sampled
 *               at the back-traced position via bilinear_sample.
 *               Marked const to enforce read-only access from the helper.
 *
 *   work      : WRITE pointer — buffer that receives the new-frame
 *               density.  After the per-cell loop completes, the caller
 *               memcpy's work → density so the next tick reads what we
 *               just wrote.  Aliased with scene_draw's dither scratch
 *               (Scene.work) — order of operations keeps them separate.
 *
 *   cols, rows: cached grid dimensions, used by bilinear_sample for
 *               boundary clamping and by the per-cell loop bounds.
 *
 *   fy        : bottom-row index (rows − 1).  Source injection happens
 *               ONLY at y == fy; every other row receives `0` for src.
 *               Caching avoids a `rows - 1` subtraction per cell.
 *
 *   margin    : arch envelope start column = cols × ARCH_MARGIN_FRAC.
 *               Columns [0, margin) and (cols−margin, cols) receive no
 *               source — keeps the smoke base away from screen edges.
 *
 *   span      : arch envelope width = cols − 2·margin.  Source position
 *               is parameterised as t = (x − margin − wind_acc) / span
 *               so the arch lives in t ∈ [0, 1] regardless of cols.
 *
 *   wind_acc  : current wind offset, shifts the arch horizontally so a
 *               steady wind makes the smoke base actually MOVE rather
 *               than just lean.  Advanced once per tick in scene_tick.
 *
 *   intensity : user source-intensity multiplier ∈ [0.1, 1.0] from
 *               the g / G keys.  Scales the arch envelope's peak.
 *
 *   wscale    : current warmup scale ∈ [0, 1] from warmup_scale().
 *               Multiplied into source so smoke fades IN at startup
 *               rather than appearing fully-formed on tick 1.
 *
 *   decay     : per-tick fractional density loss, computed once via
 *               sl_decay_for_grid().  After (1 − decay)^t steps, a
 *               cell loses 1−exp(−decay·t) ≈ decay·t fraction of its
 *               density.  Tuned so the smoke column lands at
 *               ~VORT_REACH_FRAC·rows under unit upward velocity.
 * ──────────────────────────────────────────────────────────────────── */
typedef struct {
    const float *density;
    float       *work;
    int          cols, rows;
    int          fy;
    float        margin;
    float        span;
    int          wind_acc;
    float        intensity;
    float        wscale;
    float        decay;
} SLCtx;

/* Decay scalar derived from screen height so the smoke column lands at
 * ~VORT_REACH_FRAC·rows.  After (1−decay)^t steps, a cell that travels
 * upward at 1 cell/tick fades to ~0.37 after `target` ticks; the source
 * keeps pushing replacement density up from below. */
static inline float sl_decay_for_grid(int rows)
{
    float target = (float)rows * VORT_REACH_FRAC;
    float d = (target > 1.f) ? (1.f / target) * VORT_DECAY_SCALE : VORT_DECAY_MIN;
    if (d < VORT_DECAY_MIN) d = VORT_DECAY_MIN;
    return d;
}

/* Pack the per-tick constants once. */
static inline SLCtx sl_make_ctx(const float *density, float *work,
                                int cols, int rows,
                                float intensity, int wind_acc, float wscale)
{
    SLCtx c;
    c.density   = density;
    c.work      = work;
    c.cols      = cols;
    c.rows      = rows;
    c.fy        = rows - 1;
    c.margin    = (float)cols * ARCH_MARGIN_FRAC;
    c.span      = (float)cols - 2.f * c.margin;
    c.wind_acc  = wind_acc;
    c.intensity = intensity;
    c.wscale    = wscale;
    c.decay     = sl_decay_for_grid(rows);
    return c;
}

/* CFL clamp: keep the back-trace within ADV_VEL_CAP·ADV_DT ≈ 1.6 cells
 * of the origin so bilinear_sample reads a well-defined neighbourhood.
 * Same clamp Stam recommends for unconditional stability. */
static inline void sl_clamp_velocity(float *vx, float *vy)
{
    if (*vx >  ADV_VEL_CAP) *vx =  ADV_VEL_CAP;
    if (*vx < -ADV_VEL_CAP) *vx = -ADV_VEL_CAP;
    if (*vy >  ADV_VEL_CAP) *vy =  ADV_VEL_CAP;
    if (*vy < -ADV_VEL_CAP) *vy = -ADV_VEL_CAP;
}

/* Arch-shaped source injection at the bottom row.  Returns the density
 * to add to (x, y); 0 for any non-source row.  Squared edge function
 * gives 0 at margins, 1 at centre, with per-tick jitter so the source
 * flickers naturally. */
static inline float sl_source_at(const SLCtx *c, int x, int y)
{
    if (y != c->fy) return 0.f;
    float ts = ((float)x - c->margin - (float)c->wind_acc)
             / (c->span > 0.f ? c->span : 1.f);
    if (ts < 0.f || ts > 1.f) return 0.f;
    float edge = (ts < 0.5f) ? ts : 1.f - ts;
    float arch = (edge * 2.f) * (edge * 2.f);
    float jit  = SRC_JITTER_BASE + SRC_JITTER_RANGE * ((float)rand() / RAND_MAX);
    return c->intensity * arch * jit * c->wscale;
}

/* One semi-Lagrangian cell step given the algo's velocity at this cell.
 * Caller hands in (vx, vy); helper does clamp + back-trace + sample +
 * source-injection + blend + clamp + write. */
static inline void sl_step_cell(const SLCtx *c, int x, int y, float vx, float vy)
{
    sl_clamp_velocity(&vx, &vy);

    float sx  = (float)x - vx * ADV_DT;
    float sy  = (float)y - vy * ADV_DT;
    float adv = bilinear_sample(c->density, sx, sy, c->cols, c->rows);
    float src = sl_source_at(c, x, y);

    float v = adv * (1.f - c->decay) + src;
    if (v < 0.f) v = 0.f;
    if (v > 1.f) v = 1.f;
    c->work[y * c->cols + x] = v;
}

/* ===================================================================== */
/* §5  algo 0 — particle puffs                                            */
/* ===================================================================== */

/*
 * Particle — one smoke puff (consumed only by Algo 0, the Particle System).
 *
 * REFERENCES:
 *   Reeves, W. T. (1983) "Particle Systems — A Technique for Modeling
 *     a Class of Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     The original.  Reeves defines the fixed-size pool, the per-particle
 *     (position, velocity, lifetime, density) state, and the stochastic
 *     update model that this struct implements.  §4 covers fire and
 *     smoke specifically — the (life-decay → density-fade) model used
 *     here is from §4.2.
 *
 *   Witkin, A. & Baraff, D. (2001) "Physically Based Modeling: Principles
 *     and Practice", SIGGRAPH course notes.  §1 — particle dynamics with
 *     EXPLICIT FORCE ACCUMULATION: clear accel, add each force, integrate.
 *     The per-tick `vx += turbulence; vx *= damp` pattern in
 *     particle_integrate is Witkin & Baraff's accumulate-then-damp idiom
 *     applied to the horizontal axis only.
 *
 *   Akenine-Möller, T. et al. (2018) "Real-Time Rendering", 4th ed.
 *     §13.7 — POINT-SPRITE particles with soft-edged falloff.  Our
 *     bilinear "tent filter" splat (particle_splat_bilinear) is the
 *     ASCII-cell analogue of Akenine-Möller's screen-space sprite blur.
 *
 * INTENT: hold every per-puff value the integrator (particle_tick) and
 * the splat pass (particles_rebuild_density) need, in a flat layout
 * that fits into a fixed-size BSS array (no malloc after init).
 * Algos 1..4 ignore this entirely — they fill `density` directly via
 * semi-Lagrangian advection.
 *
 * LIFECYCLE: spawn at the arch source zone with life = 1.0 → integrate
 * forward each tick (vx random-walk + upward drift; life -= decay) →
 * when life <= 0 or position leaves the grid, flip `active` false and
 * the slot becomes available to the next particle_spawn.
 *
 * QUADRATIC FADE: the splat pass writes density = life², not life.
 * Quadratic gives fast initial opacity (a fresh puff appears solid)
 * with a long gentle tail (the wisp lingers), instead of a harsh
 * linear cutoff at end-of-life that would look like a sudden pop.
 * This matches Reeves §4.2's "intensity ∝ remaining_life^p" recommendation
 * (he uses p ∈ [1, 3]; p = 2 is the cheap-but-good middle ground).
 *
 *   x, y    : position in GRID CELLS (float). Sub-cell precision is
 *             what lets the bilinear splat scatter density across
 *             4 surrounding cells with continuously-varying weight
 *             — without it, puffs would snap-blink between cells.
 *
 *   vx, vy  : velocity in cells/tick. Sign convention: vy NEGATIVE
 *             = rising (ncurses y increases downward). At spawn vy
 *             ≈ −0.5 cells/tick (upward drift); vx starts near zero
 *             and accumulates ±PART_TURB_STEP/2 turbulence per tick
 *             with PART_VX_DAMP = 0.97 damping.  Force-accumulation
 *             pattern (Witkin & Baraff §1).
 *
 *   life    : [1.0 → 0.0] linear lifetime counter. Decremented by
 *             `decay` each tick. The splat writes life² as density,
 *             so a half-dead puff (life=0.5) renders at 0.25 density.
 *
 *   decay   : life lost per tick.  Pre-computed at spawn as
 *             1.0 / lifetime where lifetime is randomly drawn from
 *             [PART_LIFE_MIN, PART_LIFE_MIN + PART_LIFE_RANGE].
 *             Caching it avoids a division in every tick's update —
 *             Reeves §3 specifically calls out this micro-optimisation.
 *
 *   active  : pool-slot occupancy flag. Inactive slots are skipped
 *             by both the integrator and the splat pass; spawn finds
 *             the next inactive slot via the part_idx round-robin cursor.
 */
typedef struct {
    float x, y;
    float vx, vy;
    float life;
    float decay;
    bool  active;
} Particle;

/*
 * particle_spawn() — birth one particle at the arch source zone.
 *
 * Spawn column is rejection-sampled weighted by the inline arch envelope
 * (edge² shape, zero at margins, 1 at centre) so most particles emerge
 * from the centre of the smoke base.  Up to 8 attempts before falling
 * back to the centre column — keeps spawning bounded under heavy load.
 */
static void particle_spawn(Particle *p, int cols, int rows,
                            float intensity, int wind_acc, int warmup)
{
    float wscale = (warmup < WARMUP_TICKS) ? (float)warmup / (float)WARMUP_TICKS : 1.f;
    float margin = (float)cols * ARCH_MARGIN_FRAC;
    float span   = (float)cols - 2.f * margin;

    float bx = (float)cols * 0.5f;
    for (int attempt = 0; attempt < 8; attempt++) {
        float t  = (float)rand() / RAND_MAX;
        float cx = margin + t * span + (float)wind_acc;
        float edge   = (t < 0.5f) ? t : 1.f - t;
        float arch   = (edge * 2.f) * (edge * 2.f);
        float accept = arch * intensity * wscale;
        if (((float)rand() / RAND_MAX) < accept) { bx = cx; break; }
    }

    p->x      = bx;
    p->y      = (float)(rows - 1) - 0.5f;
    p->vx     = ((float)rand() / RAND_MAX - 0.5f) * PART_VX_SPREAD;
    p->vy     = -(PART_VY_BASE + ((float)rand() / RAND_MAX) * PART_VY_RANGE);
    float life_ticks = PART_LIFE_MIN + ((float)rand() / RAND_MAX) * PART_LIFE_RANGE;
    p->life   = 1.0f;
    p->decay  = 1.0f / life_ticks;
    p->active = true;
}

/* ── Per-particle physics helpers ────────────────────────────────── */

/* One-tick integration of a single particle:
 *   vx += random turbulence kick ±PART_TURB_STEP/2  (Witkin & Baraff
 *                                                    force accumulation)
 *   vx *= PART_VX_DAMP                              viscous damping
 *   x  += vx;  y += vy                              explicit Euler position
 *   life -= decay                                   linear lifetime counter */
static inline void particle_integrate(Particle *p)
{
    p->vx += ((float)rand() / RAND_MAX - 0.5f) * PART_TURB_STEP;
    p->vx *= PART_VX_DAMP;
    p->x  += p->vx;
    p->y  += p->vy;
    p->life -= p->decay;
}

/* Return true if the particle is still alive AND on-grid after this step.
 * Caller flips active=false when this returns false. */
static inline bool particle_still_alive(const Particle *p, int cols, int rows)
{
    return p->life > 0.f
        && p->x >= 0.f && p->x < (float)cols
        && p->y >= 0.f && p->y < (float)rows;
}

/* Bilinear "tent-filter" splat of one particle's life² density across the
 * four cells surrounding (x, y).  Weights:
 *     w00 = (1-tx)(1-ty)   w10 = tx(1-ty)
 *     w01 = (1-tx)ty       w11 = tx·ty            Σ w = 1
 * Each cell that falls inside the grid receives life² · w added to it.
 * Smaller weight per cell + 4 cells per particle → soft, fuzzy puffs
 * instead of a hard one-cell stamp. */
static inline void particle_splat_bilinear(const Particle *p,
                                           float *density, int cols, int rows)
{
    float pd = p->life * p->life;          /* quadratic density fade */
    int   x0 = (int)p->x, y0 = (int)p->y;
    int   x1 = x0 + 1,    y1 = y0 + 1;
    float tx = p->x - (float)x0;
    float ty = p->y - (float)y0;

    if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows)
        density[y0*cols+x0] += pd * (1.f-tx) * (1.f-ty);
    if (x1 >= 0 && x1 < cols && y0 >= 0 && y0 < rows)
        density[y0*cols+x1] += pd * tx       * (1.f-ty);
    if (x0 >= 0 && x0 < cols && y1 >= 0 && y1 < rows)
        density[y1*cols+x0] += pd * (1.f-tx) * ty;
    if (x1 >= 0 && x1 < cols && y1 >= 0 && y1 < rows)
        density[y1*cols+x1] += pd * tx       * ty;
}

/* ── Pool sweep helpers ──────────────────────────────────────────── */

/* Integrate every alive particle; deactivate those that died this tick. */
static void particles_integrate_all(Particle *parts, int cols, int rows)
{
    for (int i = 0; i < MAX_PARTS; i++) {
        Particle *p = &parts[i];
        if (!p->active) continue;
        particle_integrate(p);
        if (!particle_still_alive(p, cols, rows))
            p->active = false;
    }
}

/* Spawn SPAWN_PER_TICK new particles via round-robin search for free slots.
 * Each attempt advances *next_idx; if the slot is taken, try the next.
 * Bounded by MAX_PARTS tries so we don't loop forever when the pool is
 * full — in that case the spawn is silently dropped (graceful saturation). */
static void particles_spawn_burst(Particle *parts, int *next_idx,
                                  int cols, int rows,
                                  float intensity, int wind_acc, int warmup)
{
    for (int s = 0; s < SPAWN_PER_TICK; s++) {
        for (int tries = 0; tries < MAX_PARTS; tries++) {
            *next_idx = (*next_idx + 1) % MAX_PARTS;
            if (!parts[*next_idx].active) {
                particle_spawn(&parts[*next_idx], cols, rows,
                               intensity, wind_acc, warmup);
                break;
            }
        }
    }
}

/* Zero the density field, then splat every alive particle.  We REBUILD
 * the field every tick (rather than incrementing) so dead-now particles
 * leave no ghost density behind. */
static void particles_rebuild_density(Particle *parts,
                                      float *density, int cols, int rows)
{
    memset(density, 0, (size_t)(cols * rows) * sizeof(float));
    for (int i = 0; i < MAX_PARTS; i++) {
        if (!parts[i].active) continue;
        particle_splat_bilinear(&parts[i], density, cols, rows);
    }
    /* Cells where multiple particles overlapped can exceed 1.0; clamp
     * so the dither LUT stays in its valid range. */
    for (int i = 0; i < cols * rows; i++)
        if (density[i] > 1.f) density[i] = 1.f;
}

/*
 * particle_tick — one step of "Reeves Lagrangian particle smoke".
 *
 * ALGORITHM (Reeves 1983 particle system + bilinear tent-filter render):
 *
 *   Unlike algos 1..4 (which work on the density grid directly), this
 *   algo maintains a POOL of discrete particle objects.  Each particle
 *   has its own (x, y, vx, vy, life) and lives until it leaves the
 *   grid or its life counter hits zero.  The density grid is recomputed
 *   from scratch every tick by splatting each alive particle.
 *
 *   Pseudocode per tick:
 *     1. particles_integrate_all — move every alive particle one step,
 *                                   reap dead ones.
 *     2. particles_spawn_burst   — birth SPAWN_PER_TICK new particles at
 *                                   the arch source zone (round-robin slot
 *                                   allocator).
 *     3. warmup_scale            — advance the shared warmup counter; the
 *                                   spawn step already read it.
 *     4. particles_rebuild_density — zero density, splat life² of every
 *                                     alive particle into 4 surrounding
 *                                     cells via bilinear weights.
 *
 * What you SEE: discrete puffs that you can almost count.  Each puff
 * is a soft 2×2 cell blob (the tent filter) that quadratically fades
 * over its lifetime.  The smoke as a whole is the union of those blobs
 * — no continuous flow field, just a moving cloud of overlapping
 * Gaussian-ish bumps.  Best for "low-density, lots of detail" smoke;
 * the Eulerian algos win at high density where particles would have to
 * overlap heavily anyway.
 */
static void particle_tick(Particle *parts, int *next_idx,
                          float *density, int cols, int rows,
                          float intensity, int wind_acc, int *warmup)
{
    particles_integrate_all(parts, cols, rows);
    particles_spawn_burst  (parts, next_idx, cols, rows,
                            intensity, wind_acc, *warmup);
    warmup_scale(warmup);         /* advance counter; return value unused */
    particles_rebuild_density(parts, density, cols, rows);
}

/* ===================================================================== */
/* §6  algo 1 — vortex advection                                          */
/* ===================================================================== */

/*
 * Vortex — a 2D point vortex (consumed only by Algo 1, Vortex Advection).
 *
 * REFERENCES:
 *   Lamb, H. (1932) "Hydrodynamics", 6th ed., Cambridge University Press.
 *     §155-158 — the classical theory of POINT VORTICES in 2D inviscid
 *     flow.  The velocity field around an isolated point vortex of
 *     circulation Γ at the origin is
 *         v_θ(r) = Γ / (2π r)
 *     tangential to the radius; equivalently in Cartesian coords
 *         v = (Γ / 2π) · (−y, x) / r²
 *     The (strength · (−dy, dx) / r²) formula below is exactly this,
 *     with `strength` absorbing the Γ/(2π) factor.
 *
 *   Bridson, R. (2015) "Fluid Simulation for Computer Graphics", 2nd ed.
 *     §2.3 — vortex-particle methods for incompressible flow.
 *     Justifies the +ε regulariser at r² → 0: real vortex cores are
 *     viscous and the velocity peaks at a finite value (Lamb-Oseen
 *     vortex); the ε approximates that smooth core cheaply.
 *
 * INTENT: hold the moving source of one rotational velocity field
 * contribution. The vortex itself isn't drawn — its only effect is
 * to STIR the density grid via Biot-Savart at every sample point.
 *
 * BIOT-SAVART CONTRIBUTION at sample point (px, py):
 *   dx = px − cx;  dy = py − cy;  r² = dx² + dy²
 *   vx += strength · (−dy) / (r² + VORT_EPS)
 *   vy += strength · ( dx) / (r² + VORT_EPS)
 * The (−dy, dx) rotation gives counter-clockwise flow for positive
 * `strength`. VORT_EPS regularises the singularity at the vortex
 * centre — without it, r² → 0 produces an infinite velocity and the
 * semi-Lagrangian back-trace reads garbage.
 *
 * ORBITING vs FIXED: each vortex has its OWN circular orbit around
 * the screen centre, advanced by orb_spd each tick.  Mixing positive
 * and negative orb_spd / strength across N_VORTS=3 gives the
 * counter-rotating-eddy look — no uniform whole-frame rotation.
 * (Mathematically this is a TIME-VARYING velocity field that's
 * divergence-free at every instant — Bridson §2.3.)
 *
 *   cx, cy   : current vortex CENTRE in grid cells (float). Updated
 *              each tick by vortex_advance_orbits as
 *              (cx, cy) = screen_centre + orb_r · (cos orb_a, sin orb_a).
 *              Read by every sample point as the (px-cx, py-cy)
 *              distance in the Biot-Savart formula.
 *
 *   strength : Biot-Savart strength constant. Positive = CCW (counter-
 *              clockwise) rotation viewed with y-axis pointing down;
 *              negative = CW. Magnitude controls how strongly the
 *              vortex stirs nearby density — typical |strength| ∈
 *              [1.4, 2.5] gives visible curls without blowing past
 *              the ADV_VEL_CAP=2 CFL clamp.
 *
 *   orb_r    : orbital radius in grid cells around the screen centre.
 *              Larger = vortex sweeps a wider arc and visits more of
 *              the frame; smaller = vortex stays near centre and
 *              produces tighter local swirls. Preset fractions in
 *              VORT_ORB_FRACS[] are multiplied by cols at init.
 *
 *   orb_a    : current orbital ANGLE in radians, advanced by orb_spd
 *              each tick. Determines where the vortex is on its
 *              orbit right now. Initialised with VORT_ORB_PHASE_OFFS
 *              so the three vortices don't start at the same angle.
 *
 *   orb_spd  : angular speed in radians/tick. Sign flips orbital
 *              direction (so a positive-strength CCW vortex can ORBIT
 *              CW around centre, giving the meta-rotation extra
 *              visual interest). Magnitude controls how quickly the
 *              swirl positions shift in the frame.
 */
typedef struct {
    float cx, cy;
    float strength;
    float orb_r;
    float orb_a;
    float orb_spd;
} Vortex;

/*
 * vortex_init() — set up N_VORTS vortices using the preset arrays from §1.
 * Radii from VORT_ORB_FRACS[] are stored as absolute grid cells.
 */
static void vortex_init(Vortex vorts[N_VORTS], int cols, int rows)
{
    float cx = (float)cols * 0.5f;
    float cy = (float)rows * 0.5f;

    for (int i = 0; i < N_VORTS; i++) {
        vorts[i].orb_r   = VORT_ORB_FRACS[i] * (float)cols;
        vorts[i].orb_spd = VORT_ORB_SPDS[i];
        vorts[i].strength= VORT_STRENGTHS[i];
        vorts[i].orb_a   = VORT_INIT_ANGLES[i];
        vorts[i].cx = cx + vorts[i].orb_r * cosf(vorts[i].orb_a);
        vorts[i].cy = cy + vorts[i].orb_r * sinf(vorts[i].orb_a);
    }
}

static void vortex_advance_orbits(Vortex vorts[N_VORTS], int cols, int rows)
{
    float cx = (float)cols * 0.5f;
    float cy = (float)rows * 0.5f;

    for (int i = 0; i < N_VORTS; i++) {
        vorts[i].orb_a += vorts[i].orb_spd;
        vorts[i].cx = cx + vorts[i].orb_r * cosf(vorts[i].orb_a);
        vorts[i].cy = cy + vorts[i].orb_r * sinf(vorts[i].orb_a);
    }
}


/* Velocity at (x, y) = Σ Biot-Savart contributions from all N_VORTS
 * vortices.  Each vortex contributes a rotating field whose strength
 * decays as 1/r² and whose direction is perpendicular to (px-cx, py-cy):
 *     dv = strength · (-dy, dx) / (r² + ε)
 * Mixed signs across vortices (some CW, some CCW) cancel in some regions
 * and reinforce in others → counter-rotating eddies. */
static inline void vortex_velocity_at(int x, int y,
                                      const Vortex vorts[N_VORTS],
                                      float *out_vx, float *out_vy)
{
    float vx = 0.f, vy = 0.f;
    for (int i = 0; i < N_VORTS; i++) {
        float dx = (float)x - vorts[i].cx;
        float dy = (float)y - vorts[i].cy;
        float r2 = dx * dx + dy * dy + VORT_EPS;
        vx += vorts[i].strength * (-dy) / r2;
        vy += vorts[i].strength * ( dx) / r2;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/*
 * vortex_tick — one step of "smoke stirred by 3 orbiting whirlpools".
 *
 * ALGORITHM (Stam 1999 semi-Lagrangian + Biot-Savart velocity field):
 *
 *   1. Advance each vortex one step along its orbit (orb_a += orb_spd).
 *      The vortex centres trace circles around the screen centre; their
 *      mixed +/− chirality is what produces multiple counter-rotating
 *      eddies in the smoke rather than a single uniform swirl.
 *
 *   2. For each cell (x, y):
 *      a. Sum Biot-Savart contributions from every vortex →
 *         a velocity vector v(x, y).  Far from any vortex centre, the
 *         field is weak (1/r² falloff).  Near a centre the velocity
 *         peaks but is bounded by the +VORT_EPS regulariser.
 *      b. CFL clamp the velocity to ADV_VEL_CAP cells/tick so the
 *         back-trace below reads a small, well-defined neighbourhood.
 *      c. SEMI-LAGRANGIAN BACK-TRACE: "where was the smoke at this
 *         cell one tick ago?"  Answer: (x − vx·dt, y − vy·dt).
 *         Bilinear-sample the previous-frame density there.
 *      d. Add arch-shaped source on the bottom row, fade by (1 − decay).
 *
 *   3. memcpy work → density.
 *
 * What you SEE: curls and swirls.  The streamlines of the velocity
 * field ARE the visible smoke patterns — anywhere the field rotates,
 * the smoke curls; anywhere it converges or diverges (numerically
 * impossible for Biot-Savart, which is divergence-free, but visually
 * possible near the regulariser cutoff), the smoke piles or thins.
 */
static void vortex_tick(float *density, float *work,
                        Vortex vorts[N_VORTS],
                        int cols, int rows,
                        float intensity, int wind_acc, int *warmup)
{
    float wscale = warmup_scale(warmup);
    vortex_advance_orbits(vorts, cols, rows);

    SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            float vx, vy;
            vortex_velocity_at(x, y, vorts, &vx, &vy);
            sl_step_cell(&c, x, y, vx, vy);
        }
    }

    memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §7  algo 2 — curl-noise advection                                      */
/* ===================================================================== */

/*
 * Curl-noise smoke: a smooth divergence-free velocity field generated
 * from a scalar Perlin-style noise potential, advected semi-Lagrangianly.
 *
 *   velocity = curl(P)   where P is a smooth scalar noise field
 *   in 2D:   vx =  ∂P/∂y      vy = -∂P/∂x
 * Curl of a scalar is automatically divergence-free (∇·v ≡ 0), so
 * density is preserved under advection — no artificial sinks/sources.
 * Different from vortex: smooth distributed flow rather than discrete
 * point vortices.  See Bridson §4 "Vortex methods and noise advection".
 */

/* Cheap integer hash → unit float [0, 1).  Avalanche-style scramble. */
static inline float curl_hash01(int ix, int iy, int seed)
{
    uint32_t h = (uint32_t)(ix * 374761393 + iy * 668265263 + seed * 1274126177);
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (float)(h & 0xFFFFFFu) / (float)0x1000000;   /* [0, 1) */
}

/* 2D smooth value noise: cosine-eased bilinear blend of 4 hashed corners. */
static inline float curl_noise2d(float x, float y, int seed)
{
    int   ix = (int)floorf(x);
    int   iy = (int)floorf(y);
    float fx = x - (float)ix;
    float fy = y - (float)iy;
    float sx = fx * fx * (3.f - 2.f * fx);   /* smoothstep ease */
    float sy = fy * fy * (3.f - 2.f * fy);
    float v00 = curl_hash01(ix,     iy,     seed);
    float v10 = curl_hash01(ix + 1, iy,     seed);
    float v01 = curl_hash01(ix,     iy + 1, seed);
    float v11 = curl_hash01(ix + 1, iy + 1, seed);
    float a = v00 + (v10 - v00) * sx;
    float b = v01 + (v11 - v01) * sx;
    return a + (b - a) * sy;
}

/* Curl of the scalar noise field — gives a divergence-free 2D velocity.
 *     vx =  ∂P/∂y     vy = −∂P/∂x          where P is the noise potential
 * Finite differences with h = 0.5 cells.  Time-varying via a t-offset on
 * the noise coordinate.  Bias the y component downward (positive vy means
 * downward in ncurses) so smoke rises rather than just swirling in place. */
static inline void curl_velocity_at(int x, int y, float t,
                                    float *out_vx, float *out_vy)
{
    float h     = 0.5f;
    float scale = CURL_SCALE;
    float fx = (float)x, fy = (float)y;
    float yp = curl_noise2d(fx * scale,           (fy + h) * scale + t, 0);
    float ym = curl_noise2d(fx * scale,           (fy - h) * scale + t, 0);
    float xp = curl_noise2d((fx + h) * scale,     fy * scale       + t, 0);
    float xm = curl_noise2d((fx - h) * scale,     fy * scale       + t, 0);
    *out_vx =  CURL_AMP * (yp - ym) / (2.f * h);
    *out_vy = -CURL_AMP * (xp - xm) / (2.f * h) - CURL_UPWARD_BIAS;
}

/*
 * curl_tick — one step of "smoke stirred by a smooth divergence-free
 * noise field".
 *
 * ALGORITHM (Stam 1999 SL + curl-of-noise velocity field, Bridson §4):
 *
 *   Mathematically: if P(x, y, t) is any smooth scalar field, then
 *   v = ∇×P (which in 2D collapses to (∂P/∂y, −∂P/∂x)) is automatically
 *   divergence-free: ∇·v = ∂²P/∂x∂y − ∂²P/∂y∂x = 0.  Advecting a density
 *   along a divergence-free field PRESERVES total density — no
 *   artificial sources or sinks.  This is the property real fluids
 *   have (incompressibility), achieved here for free without solving
 *   a pressure-projection step.
 *
 *   Each tick:
 *     1. Compute t = warmup · CURL_TIME_RATE so the noise field
 *        evolves slowly over many ticks (full re-shuffle ≈ 60 sec).
 *     2. For each cell (x, y):
 *        a. Sample the noise potential P at four offset points
 *           (h = 0.5 cells) and take the curl by finite differences.
 *           Add a constant upward bias (CURL_UPWARD_BIAS) so the
 *           smoke actually rises — pure curl has no preferred direction.
 *        b. Same CFL clamp + back-trace + bilinear sample + arch source
 *           + blend as vortex_tick (via sl_step_cell).
 *     3. memcpy work → density.
 *
 * What you SEE: smooth distributed flow, no obvious centres or curls.
 * Smoke meanders organically through the frame — visually closer to
 * real turbulence than the discrete vortex algo because the velocity
 * is continuous everywhere instead of peaking at point sources.
 */
static void curl_tick(float *density, float *work,
                      int cols, int rows,
                      float intensity, int wind_acc, int *warmup)
{
    float wscale = warmup_scale(warmup);
    SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);
    float t = (float)*warmup * CURL_TIME_RATE;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            float vx, vy;
            curl_velocity_at(x, y, t, &vx, &vy);
            sl_step_cell(&c, x, y, vx, vy);
        }
    }
    memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §8  algo 3 — buoyancy plume (Boussinesq)                               */
/* ===================================================================== */

/*
 * Buoyancy plume: the density itself drives upward velocity at each
 * cell — hot (dense) regions rise faster than cold ones.
 *
 *   vy = -BUOY_RISE · density[here]      (hot rises)
 *   vx = mild noise-driven horizontal turbulence
 *
 * This is the Boussinesq approximation collapsed onto a single field:
 * density doubles as temperature.  Different from vortex (which has
 * external stirring) and curl (which has noise-driven flow) — here the
 * smoke moves itself.  Visible signature: tall thin plume that
 * mushroom-caps at the top when buoyancy stalls against decay.
 */
/* Velocity at (x, y) for the buoyancy plume.
 *   vy = -BUOY_RISE · density[here]        Boussinesq: hot rises faster
 *   vx = (noise − 0.5) · 2·BUOY_TURB_AMP   mild horizontal turbulence
 * The local density itself drives the upward push — empty regions get
 * zero upward force, dense regions get a strong one.  That's what gives
 * buoyancy its characteristic "the smoke pulls itself up" feel. */
static inline void buoy_velocity_at(int x, int y, float t,
                                    const float *density, int cols,
                                    float *out_vx, float *out_vy)
{
    float d_here = density[y * cols + x];
    float n = curl_noise2d((float)x * BUOY_TURB_SCALE,
                           (float)y * BUOY_TURB_SCALE + t, 7);
    *out_vx = (n - 0.5f) * 2.f * BUOY_TURB_AMP;
    *out_vy = -BUOY_RISE * d_here;
}

/*
 * buoy_tick — one step of "smoke that pulls itself up because it's hot".
 *
 * ALGORITHM (Stam 1999 SL + Boussinesq density-driven buoyancy):
 *
 *   The Boussinesq approximation says that in a slightly-heated fluid,
 *   density variations enter the momentum equation as a single buoyancy
 *   term:  vertical force per unit mass = -β · (T - T_ref).  Here we
 *   collapse temperature onto the density field itself — density doubles
 *   as a temperature proxy.  Cold (empty) cells get no buoyancy; hot
 *   (full) cells rise fastest.  Strict positive feedback: dense plumes
 *   rise quickly into emptiness, the void leaves an even hotter pocket
 *   below, which then rises faster.
 *
 *   Each tick:
 *     1. Compute decay + turbulence time t.
 *     2. For each cell (x, y):
 *        a. Read density[y,x] = "local temperature".
 *        b. Velocity:  vy = −BUOY_RISE · density  (rising; sign negative
 *                                                  because ncurses y is
 *                                                  positive-down)
 *                      vx = small noise-driven horizontal sway
 *                      (without it the plume goes dead-straight up,
 *                      which doesn't look natural)
 *        c. Same CFL clamp + back-trace + arch source + blend.
 *     3. memcpy work → density.
 *
 * What you SEE: tall narrow plumes that climb quickly through empty
 * space, then MUSHROOM-CAP at the top where the buoyancy force has
 * weakened (density × rise) below the decay rate.  The cap is the
 * literal momentum-vs-dissipation balance point.  Without that decay
 * the plume would punch right through the ceiling.
 */
static void buoy_tick(float *density, float *work,
                      int cols, int rows,
                      float intensity, int wind_acc, int *warmup)
{
    float wscale = warmup_scale(warmup);
    SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);
    float t = (float)*warmup * BUOY_TURB_RATE;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            float vx, vy;
            buoy_velocity_at(x, y, t, density, cols, &vx, &vy);
            sl_step_cell(&c, x, y, vx, vy);
        }
    }
    memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §9  algo 4 — breeze advection (sinusoidal laminar flow)                */
/* ===================================================================== */

/*
 * Breeze advection: time-varying laminar horizontal flow with a small
 * upward drift.  The velocity is constant along x for each row, varies
 * with y as a sine wave, and shifts phase with time:
 *
 *     vx(y, t) = BREEZE_AMP · sin(t + y · BREEZE_K)
 *     vy       = -BREEZE_RISE                       (gentle upward)
 *
 * Visual signature: density rises as a swaying curtain — each row drifts
 * at its own phase, so the plume bends like seaweed in a current.
 * Different from curl (which has 2-D structured noise) and buoyancy
 * (which has local self-driven velocity) — here the velocity is a
 * cheap closed-form 1-D pattern, no noise table involved.
 */
/* Per-row sinusoidal sway — the velocity depends only on y and t,
 * NOT on x.  That's what makes the breeze laminar: every column in
 * the same row gets the same horizontal push.  The y-modulation makes
 * different rows sway out of phase, producing the curtain-bend look.
 *
 *     vx(y, t) = BREEZE_AMP · sin(t + y · BREEZE_K)
 *     vy       = −BREEZE_RISE                          gentle upward drift
 */
static inline void breeze_velocity_at(int y, float t,
                                      float *out_vx, float *out_vy)
{
    *out_vx = BREEZE_AMP * sinf(t + (float)y * BREEZE_K);
    *out_vy = -BREEZE_RISE;
}

/*
 * breeze_tick — one step of "smoke caught in a swaying horizontal wind".
 *
 * ALGORITHM (Stam 1999 SL + analytic laminar-flow velocity):
 *
 *   Unlike vortex/curl/buoy which use noise tables or per-cell sums,
 *   the breeze velocity is a CLOSED-FORM 1-D pattern: a sine wave whose
 *   phase advances with time and whose argument depends on row.  Two
 *   handy properties:
 *
 *     - velocity is constant along x for a fixed row  → laminar flow,
 *       no shear within a row, smoke moves as a coherent layer
 *     - the per-row phase offset (y · BREEZE_K) makes adjacent rows
 *       move at different sway phases → the column visibly BENDS
 *       like a curtain in a current.
 *
 *   Each tick:
 *     1. t = warmup · BREEZE_RATE advances the sine phase.
 *     2. For each row y, precompute vx_row = AMP · sin(t + y·k) once
 *        (it's the same for every cell in that row).
 *     3. For each cell, call sl_step_cell with (vx_row, -BREEZE_RISE).
 *     4. memcpy work → density.
 *
 * What you SEE: smoke rises as a swaying curtain.  Watching a single
 * row reveals a clean horizontal oscillation; watching a vertical
 * stripe reveals a snake-like bend as the phase walks up.  No
 * turbulence, no curls — just rhythmic laminar flow.
 */
static void breeze_tick(float *density, float *work,
                        int cols, int rows,
                        float intensity, int wind_acc, int *warmup)
{
    float wscale = warmup_scale(warmup);
    SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);
    float t = (float)*warmup * BREEZE_RATE;

    for (int y = 0; y < rows; y++) {
        /* Per-row sway, computed once outside the inner loop. */
        float vx, vy;
        breeze_velocity_at(y, t, &vx, &vy);

        for (int x = 0; x < cols; x++)
            sl_step_cell(&c, x, y, vx, vy);
    }
    memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §10 scene                                                              */
/* ===================================================================== */

/*
 * Scene — owns every piece of mutable state for the smoke simulation.
 *
 * DESIGN INTENT: cleanly separate the PHYSICS-AND-STATE (what scene_tick
 * mutates) from the RENDER-SELECTION (what scene_draw consults).  Most
 * of the file's runtime memory lives here — three cols×rows float fields,
 * the MAX_PARTS particle pool, the N_VORTS vortex array — and pulling
 * them all into one struct makes scene_alloc / scene_free / scene_resize
 * trivial (one calloc set, one free set, one re-allocation on SIGWINCH).
 *
 * Two clearly-separated halves:
 *
 *   SIMULATION half — what scene_tick reads + writes.
 *                     Owns the shared density grid (and its prev/work
 *                     scratch buffers), the active algorithm selector,
 *                     the warmup ramp, wind state, the particle pool
 *                     (Algo 0, Reeves 1983), and the vortex array
 *                     (Algo 1, Lamb's Hydrodynamics + Bridson §2.3).
 *                     Algos 2..4 are STATELESS — they only need the
 *                     shared density + work + warmup, so no extra
 *                     fields are required for them.
 *                     Mutated by scene_tick and the key handler.
 *
 *   RENDER half     — what scene_draw / screen_draw consult.
 *                     Theme selection (palette table lookup) and the
 *                     next-frame full-erase flag.  Never read inside
 *                     the physics tick — flipping the theme during a
 *                     buoy_tick wouldn't make the smoke move any
 *                     differently, only look different.
 *
 * REFERENCES:
 *   Reeves (1983), Stam (1999), Bridson (2015), Floyd & Steinberg (1976)
 *   — see the References block at the top of the file.  Each Scene
 *   member's comment cites the specific algorithm it serves.
 *
 * The Scene knows nothing about ncurses — physics writes to the
 * density grid, the render layer reads it via Floyd-Steinberg dither
 * → ramp lookup.  That separation lets the simulation be exercised
 * without a terminal (useful for headless snapshot tests).
 */
typedef struct {
    /* ──────────────────────────────────────────────────────────────
     *  SIMULATION HALF — physics tick reads + writes these
     * ────────────────────────────────────────────────────────────── */

    /* DENSITY GRID — the single shared float field all five algos
     * write into. Read by scene_draw via Floyd-Steinberg + LUT.
     * Range [0, 1]; -1 in scene_draw is a sentinel for "this cell
     * is empty, don't propagate dither error here".  Sized cols×rows. */
    float    *density;

    /* PREV-FRAME DENSITY — snapshot taken at the END of scene_draw,
     * used by the NEXT frame's clear pass to find cells that
     * transitioned from non-zero to zero (dirty-rectangle erase).
     * Same size and layout as density. */
    float    *prev_density;

    /* WORK SCRATCH — dual-use buffer reused for two different
     * purposes back-to-back per tick:
     *   (1) Algos 1..4 (semi-Lagrangian) write the new advected
     *       density here, then memcpy()s back into `density`.
     *   (2) scene_draw uses it as the gamma-corrected dither scratch.
     * Order matters: the tick runs before scene_draw, so the memcpy
     * happens first and the dither overwrites cleanly. */
    float    *work;

    /* CACHED GRID DIMENSIONS — set at scene_alloc / resize, read by
     * every loop. Avoids calling getmaxyx() in the hot path. */
    int       cols, rows;

    /* ALGORITHM SELECTOR — 0 = Particle System, 1 = Vortex Advection,
     * 2 = Curl Noise, 3 = Buoyancy Plume, 4 = Breeze. Cycled by 'a'.
     * scene_tick dispatches on this value to the correct *_tick
     * function; all five write into the same `density` field. */
    int       algo;

    /* WARMUP COUNTER — shared across all algos. Increments every tick
     * until it hits WARMUP_TICKS, then saturates. The source-row
     * intensity is multiplied by (warmup / WARMUP_TICKS) during the
     * ramp, so the smoke fades IN at start instead of appearing
     * fully-formed on the first frame.  Reset to 0 on algo switch
     * and on resize so transitions don't show stale density. */
    int       warmup;

    /* SOURCE INTENSITY — multiplier on the bottom-row arch envelope,
     * range [0.1, 1.0]. Adjusted by g / G keys. Controls how much
     * density the source injects per tick — low value = thin wisp
     * barely reaches mid-screen, full value = thick column reaching
     * the top. */
    float     source;

    /* WIND — signed step in cells/tick, positive = rightward,
     * adjusted by w / W keys, '0' resets to 0. Read once per tick
     * to advance wind_acc. */
    int       wind;

    /* WIND ACCUMULATOR — running offset (mod cols) that shifts the
     * arch envelope horizontally over time. scene_tick advances this
     * ONCE per tick by adding `wind` and wrapping — each algo then
     * reads wind_acc (without re-advancing) when computing the
     * source position. Critical: the algos must NOT increment
     * wind_acc themselves or wind would double per tick. */
    int       wind_acc;

    /* PAUSE FLAG — scene_tick is a no-op when set. Toggled by space.
     * Render keeps running so the user sees the frozen frame; for
     * Algo 1 the vortex orbital phase is preserved so resume picks
     * up exactly where pause left off. */
    bool      paused;

    /* PARTICLE POOL — Algo 0 only. Fixed-size array of smoke puffs.
     * Algos 1..4 leave this untouched. See Particle for per-slot detail. */
    Particle  parts[MAX_PARTS];

    /* PARTICLE-SPAWN CURSOR — index into parts[] hinting where the
     * next spawn should look. Advances on each spawn; wraps mod
     * MAX_PARTS. Cheap round-robin allocator that avoids a linear
     * scan on every spawn when the pool is mostly full. */
    int       part_idx;

    /* VORTEX ARRAY — Algo 1 only. Fixed-size array of N_VORTS=3
     * orbiting point vortices. See Vortex for per-slot detail.
     * Initialised by vortex_init at scene_alloc with mixed
     * chirality (some CW, some CCW) so the smoke gets multiple
     * counter-rotating eddies, not a uniform whole-frame swirl. */
    Vortex    vorts[N_VORTS];

    /* ──────────────────────────────────────────────────────────────
     *  RENDER HALF — scene_draw reads these; physics tick ignores them
     * ────────────────────────────────────────────────────────────── */

    /* THEME SELECTOR — index into k_themes[]. Cycled by t / T.
     * Selects the 9-step foreground ramp used to colour the dither
     * output. Pure render concern — smoke physics is identical
     * regardless of which theme is active.  theme_apply rewrites
     * pairs CP_BASE..CP_BASE+RAMP_N-1 when this changes. */
    int       theme;

    /* FORCE-CLEAR FLAG — set when the screen needs a full erase()
     * on the NEXT scene_draw (e.g. after algo change, theme change,
     * resize). The default render path uses a dirty-rectangle
     * compare against prev_density to keep the write count down;
     * this flag bypasses that optimisation for the one frame after
     * a state change. */
    bool      needs_clear;
} Scene;

static void scene_alloc(Scene *sc)
{
    sc->density      = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
    sc->prev_density = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
    sc->work         = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
}

static void scene_free_bufs(Scene *sc)
{
    free(sc->density);      sc->density      = NULL;
    free(sc->prev_density); sc->prev_density = NULL;
    free(sc->work);         sc->work         = NULL;
}

static void scene_init(Scene *sc, int cols, int rows, int algo, int theme)
{
    memset(sc, 0, sizeof *sc);
    sc->cols        = cols;
    sc->rows        = rows;
    sc->algo        = algo;
    sc->theme       = theme;
    sc->source      = 0.85f;
    sc->wind        = 0;
    sc->wind_acc    = 0;
    sc->warmup      = 0;
    sc->part_idx    = 0;
    scene_alloc(sc);
    vortex_init(sc->vorts, cols, rows);
}

static void scene_resize(Scene *sc, int cols, int rows)
{
    int   algo  = sc->algo;
    int   theme = sc->theme;
    float src   = sc->source;
    int   wind  = sc->wind;
    scene_free_bufs(sc);
    sc->cols        = cols;
    sc->rows        = rows;
    sc->algo        = algo;
    sc->theme       = theme;
    sc->source      = src;
    sc->wind        = wind;
    sc->wind_acc    = 0;
    sc->warmup      = 0;
    sc->needs_clear = true;
    scene_alloc(sc);
    vortex_init(sc->vorts, cols, rows);
    memset(sc->parts, 0, sizeof sc->parts);
    sc->part_idx = 0;
}

/*
 * scene_tick() — advance wind once then dispatch to the active algo.
 *
 * Wind is accumulated here, exactly once per tick, before calling the
 * algo.  No algo function should advance wind_acc independently.
 */
static void scene_tick(Scene *sc)
{
    if (sc->paused) return;

    sc->wind_acc += sc->wind;
    if (sc->wind_acc >= sc->cols || sc->wind_acc <= -sc->cols)
        sc->wind_acc = 0;

    switch (sc->algo) {
    case 0:
        particle_tick(sc->parts, &sc->part_idx,
                      sc->density, sc->cols, sc->rows,
                      sc->source, sc->wind_acc, &sc->warmup);
        break;
    case 1:
        vortex_tick(sc->density, sc->work, sc->vorts,
                    sc->cols, sc->rows,
                    sc->source, sc->wind_acc, &sc->warmup);
        break;
    case 2:
        curl_tick(sc->density, sc->work,
                  sc->cols, sc->rows,
                  sc->source, sc->wind_acc, &sc->warmup);
        break;
    case 3:
        buoy_tick(sc->density, sc->work,
                  sc->cols, sc->rows,
                  sc->source, sc->wind_acc, &sc->warmup);
        break;
    case 4:
        breeze_tick(sc->density, sc->work,
                    sc->cols, sc->rows,
                    sc->source, sc->wind_acc, &sc->warmup);
        break;
    }
}

/* ── Render-pass helpers ─────────────────────────────────────────── */

/* PASS 1 — gamma correction.  Map linear-light density [0, 1] to
 * perceptually-uniform brightness via v' = v^(1/2.2).  Empty cells
 * become −1 so the Floyd-Steinberg pass can use that as a "do not
 * propagate error here" sentinel — otherwise mid-density error would
 * leak into supposedly-black background and the empty space sparkles. */
static void render_density_to_gamma(const float *density, float *scratch,
                                    int cols, int rows)
{
    for (int i = 0; i < cols * rows; i++) {
        float v = density[i];
        scratch[i] = (v <= 0.f) ? -1.f : powf(fminf(1.f, v), 1.f / 2.2f);
    }
}

/* Diffuse the quantisation error from this cell to its four neighbours
 * using Floyd & Steinberg's 1976 mask:
 *           [      *   7 ]
 *           [  3   5   1 ]   / 16
 * Skip empty (−1) neighbours so the sentinel stays a sentinel. */
static inline void floyd_steinberg_diffuse(float *d, int i, int x, int y,
                                           int cols, int rows, float err)
{
    if (x+1 < cols && d[i+1] >= 0.f)
        d[i+1]       += err * (7.f / 16.f);
    if (y+1 < rows) {
        if (x-1 >= 0  && d[i+cols-1] >= 0.f)
            d[i+cols-1] += err * (3.f / 16.f);
        if (d[i+cols] >= 0.f)
            d[i+cols]   += err * (5.f / 16.f);
        if (x+1 < cols && d[i+cols+1] >= 0.f)
            d[i+cols+1] += err * (1.f / 16.f);
    }
}

/* PASS 2 — dither + emit one cell.  Maps the gamma-corrected scratch
 * value to a ramp index, computes the quantisation error against the
 * ramp midpoint, diffuses the error forward, and writes the glyph.
 * For empty cells (scratch < 0) emit a space ONLY if this cell was lit
 * last frame (dirty-rectangle erase). */
static inline void render_cell_emit(int x, int y, int i, float v,
                                    float *scratch, const float *prev,
                                    int cols, int rows, int theme)
{
    if (v < 0.f) {
        if (prev[i] > 0.f) mvaddch(y, x, ' ');
        return;
    }
    int   idx = lut_index(v);
    float qv  = lut_midpoint(idx);
    float err = v - qv;
    floyd_steinberg_diffuse(scratch, i, x, y, cols, rows, err);

    attr_t attr = ramp_attr(idx, theme);
    attron (attr);
    mvaddch(y, x, (chtype)(unsigned char)k_ramp[idx]);
    attroff(attr);
}

/* PASS 3 — snapshot density for next frame's dirty-cell diff.
 * Swaps the pointer (current density becomes prev_density next frame),
 * then memcpy preserves the current density values so the next tick's
 * algo still reads from a populated buffer. */
static void render_swap_density_snapshot(Scene *sc)
{
    int cols = sc->cols, rows = sc->rows;
    float *tmp       = sc->prev_density;
    sc->prev_density = sc->density;
    sc->density      = tmp;
    memcpy(sc->density, sc->prev_density, (size_t)(cols * rows) * sizeof(float));
}

/*
 * scene_draw — three-pass dithered render (same pipeline as fire.c).
 *
 * Pseudocode:
 *   1. render_density_to_gamma   — density^(1/2.2) into work[] (empty = −1)
 *   2. for each cell within the terminal viewport:
 *        render_cell_emit         — pick ramp glyph + dither + draw, OR
 *                                    erase if dropped to empty since last frame
 *   3. render_swap_density_snapshot — current density → prev_density
 *
 * Gamma correction gives mid-density billows the widest character
 * variety (linear values cluster near the dark end of the perceptual
 * curve and would crush detail).  Floyd-Steinberg dithering trades
 * a small amount of spatial noise for smoother brightness gradients
 * — without it, the 9-step ramp's banding is obvious on subtle plumes.
 */
static void scene_draw(Scene *sc, int tcols, int trows)
{
    int    cols    = sc->cols, rows = sc->rows;
    float *scratch = sc->work;        /* algo's advection already memcpy'd out */
    float *prev    = sc->prev_density;

    render_density_to_gamma(sc->density, scratch, cols, rows);

    for (int y = 0; y < rows && y < trows; y++) {
        for (int x = 0; x < cols && x < tcols; x++) {
            int   i = y * cols + x;
            float v = scratch[i];
            render_cell_emit(x, y, i, v, scratch, prev, cols, rows, sc->theme);
        }
    }

    render_swap_density_snapshot(sc);
}

/* ===================================================================== */
/* §11 screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

static const char *algo_name(int a)
{
    switch (a) {
    case 0: return "particle";
    case 1: return "vortex";
    case 2: return "curl";
    case 3: return "buoy";
    case 4: return "breeze";
    default: return "?";
    }
}

/*
 * screen_draw — render the smoke field, then paint a two-layer HUD over it:
 *
 *   Row 0          STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                  Live state: paused/run, algorithm, theme, source
 *                  intensity, wind direction (>>> / <<< / ---) +
 *                  magnitude, fps, sim Hz.
 *   Row rows-1     KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                  Every interactive key the demo accepts.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width, and drawn AFTER scene_draw so
 * smoke never bleeds through the bars.
 */
static void screen_draw(Screen *s, Scene *sc, double fps, int sfps)
{
    if (sc->needs_clear) { erase(); sc->needs_clear = false; }
    scene_draw(sc, s->cols, s->rows);

    /* ── Top row: dynamic status ─────────────────────────────── */
    const char *wstr =
        sc->wind > 0 ? ">>>" :
        sc->wind < 0 ? "<<<" : "---";

    char status[200];
    snprintf(status, sizeof status,
             " SMOKE   %s   algo:%-7s   theme:%-7s   src:%.2f   "
             "wind:%s (%+d)   %5.1f fps  %3d Hz ",
             sc->paused ? "PAUSED " : "running",
             algo_name(sc->algo),
             k_themes[sc->theme].name,
             sc->source, wstr, sc->wind,
             fps, sfps);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < s->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── Bottom row: every interactive key ───────────────────── */
    const char *hints =
        " q:quit  spc:pause  a/A:algo  t/T:theme  g/G:source  "
        "w/W:wind  0:calm  ]/[:Hz ";

    int hint_row = s->rows - 1;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int x = 0; x < s->cols; x++) mvaddch(hint_row, x, ' ');
    mvprintw(hint_row, 0, "%s", hints);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §12 app                                                                */
/* ===================================================================== */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int s) { (void)s; g_app.running    = 0; }
static void on_resize(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup  (void)  { endwin(); }

static bool app_handle_key(App *a, int ch)
{
    Scene *sc = &a->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ': sc->paused = !sc->paused; break;

    case 'a': case 'A':
        sc->algo        = (sc->algo + 1) % N_ALGOS;
        sc->warmup      = 0;
        sc->wind_acc    = 0;
        sc->needs_clear = true;
        memset(sc->parts, 0, sizeof sc->parts);
        sc->part_idx = 0;
        break;

    case 't': case 'T':
        sc->theme = (sc->theme + 1) % THEME_COUNT;
        theme_apply(sc->theme);
        sc->needs_clear = true;
        break;

    case 'g': sc->source += 0.05f; if (sc->source > 1.0f) sc->source = 1.0f; break;
    case 'G': sc->source -= 0.05f; if (sc->source < 0.1f) sc->source = 0.1f; break;

    case 'w': sc->wind++; if (sc->wind >  WIND_MAX) sc->wind =  WIND_MAX; break;
    case 'W': sc->wind--; if (sc->wind < -WIND_MAX) sc->wind = -WIND_MAX; break;
    case '0': sc->wind = 0; sc->wind_acc = 0; break;

    case ']': a->sim_fps += SIM_FPS_STEP; if (a->sim_fps > SIM_FPS_MAX) a->sim_fps = SIM_FPS_MAX; break;
    case '[': a->sim_fps -= SIM_FPS_STEP; if (a->sim_fps < SIM_FPS_MIN) a->sim_fps = SIM_FPS_MIN; break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app  = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0, 0);

    int64_t ft = clock_ns(), sa = 0, fa = 0;
    int     fc = 0;
    double  fpsd = 0.0;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            ft = clock_ns(); sa = 0;
        }

        int64_t now = clock_ns(), dt = now - ft;
        ft = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick = TICK_NS(app->sim_fps);
        sa += dt;
        while (sa >= tick) { scene_tick(&app->scene); sa -= tick; }

        fc++; fa += dt;
        if (fa >= FPS_UPDATE_MS * NS_PER_MS) {
            fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
            fc = 0; fa = 0;
        }

        int64_t el = clock_ns() - ft + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - el);

        screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
        screen_present();

        int ch;
        while ((ch = getch()) != ERR)
            if (!app_handle_key(app, ch)) { app->running = 0; break; }
    }

    scene_free_bufs(&app->scene);
    screen_free(&app->screen);
    return 0;
}
