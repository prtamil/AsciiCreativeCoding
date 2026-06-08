/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * galaxy.c — Spiral Galaxy Simulation
 *
 * Stars orbit a central mass with a flat rotation curve (every star has the
 * same tangential speed regardless of radius, just like real galaxies).
 * Because inner stars complete orbits faster than outer ones (differential
 * rotation), the arms gradually wind up — exactly what happens in real spiral
 * galaxies over hundreds of millions of years.
 *
 * Stars are NOT simulated with mutual gravity — each moves in a smooth
 * circular orbit.  The spiral structure emerges purely from the initial
 * placement on logarithmic spiral arms and the differential rotation.
 *
 * Character key (density → glyph):
 *   .  ,  :  o  O  0  @     sparse → dense
 *
 * Colour key (radial zone):
 *   CORE  — bright bulge (white / yellow-white)
 *   DISK  — spiral arms (cyan / blue)
 *   HALO  — outer disc  (grey / dim)
 *
 * Keys:
 *   q/ESC   quit         p/Space  pause/resume     r  reset current arms
 *   a       more arms (2→3→4→2)   A  fewer arms
 *   t/T     next/prev theme        +/-  orbit speed faster/slower
 *   ]/[     FPS up/down            </>  steps per frame
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/galaxy.c -o galaxy -lncurses -lm
 *
 * Sections (cut by layer — see ARCHITECTURE):  §1 config  §2 clock  §3 data
 *   §4 logic  §5 sim  §6 init  §7 render  §8 events  §9 screen  §10 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Kinematic circular orbit simulation — no N-body gravity.
 *                  Each star is placed on a logarithmic spiral arm at init
 *                  with a fixed angular speed ω = v_tan / radius.  Each
 *                  tick: θ += ω × dt.  Position = (r·cos θ, r·sin θ).
 *
 * Physics        : Flat rotation curve: all stars orbit at the same
 *                  tangential speed v_tan regardless of radius (like real
 *                  spiral galaxies, explained by dark matter halos).
 *                  Differential rotation: ω ∝ 1/r → inner stars orbit
 *                  faster → arms wind up over time (winding problem).
 *
 * Math           : Logarithmic spiral: r = a·exp(b·θ), so θ = ln(r/a)/b.
 *                  Stars seeded along N_ARMS arms at equal r-intervals;
 *                  their initial θ values are staggered by 2π/N_ARMS.
 *
 * Rendering      : Stars projected to screen (r,θ) → (col,row) with aspect
 *                  correction.  Density → glyph ramp (. , : o O 0 @).
 *                  Radial zone (CORE/DISK/HALO) determines colour pair.
 *
 * References     :
 *   Galaxy dynamics (rotation curve, differential rotation, spiral arms):
 *     Binney & Tremaine, "Galactic Dynamics" (2nd ed., Princeton 2008) — the
 *       standard text: rotation curves, differential rotation, log-spiral arms.
 *     Rubin & Ford, "Rotation of the Andromeda Nebula…" (ApJ 1970) — the flat
 *       rotation curve / dark-matter evidence; the constant-V0 law this sim uses.
 *     Lin & Shu, "On the Spiral Structure of Disk Galaxies" (ApJ 1964) —
 *       density-wave theory; why REAL arms persist instead of winding up — the
 *       counterpoint to this kinematic model, which deliberately winds up.
 *     Toomre, "Theories of Spiral Structure" (ARA&A 1977) — review including the
 *       winding dilemma this file demonstrates.
 *
 *   Rendering & numerics:
 *     Bourke, "Character representation of greyscale images" — the density →
 *       glyph ramp (. , : o O 0 @) used in scene_draw.
 *     Box & Muller, "A Note on the Generation of Random Normal Deviates"
 *       (Ann. Math. Stat. 1958) — the Gaussian sampling for the bulge (rng_gauss).
 *     Marsaglia, "Xorshift RNGs" (J. Stat. Soft. 2003) — the xorshift32 generator
 *       (rng_next), a fast deterministic PRNG for reproducible seeding.
 *
 * ─────────────────────────────────────────────────────────────────────── */


/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into LAYERS by concern. The simulation/render state lives on
 * one Galaxy aggregate (§3 DATA); the orchestrators (galaxy_init / galaxy_step /
 * scene_draw) take Galaxy* and everything else takes the narrowest value it
 * needs, so the layers never re-couple. PRNG state (g_rng), the 256-colour flag
 * (g_has_256) and the signal flags (g_resize/g_quit) stay module-global —
 * infrastructure, not galaxy data.
 *
 *   Layer        Section            Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2 clock           nothing (reads OS clock, sleeps)
 *   DATA         §3 data            — Galaxy aggregate + type declarations —
 *   LOGIC        §4 logic           g_rng only (its own PRNG word); pure maps
 *   SIMULATION   §5 galaxy_step     galaxy.stars[].theta (orbits) + .bright (trail)
 *   INIT/RESET   §6 galaxy_init     galaxy.stars[], .bright, .narms (full reseed)
 *   RENDER       §7 render          the screen, + galaxy.bright decay (see note)
 *   EVENTS       §8 events          g_resize, g_quit, galaxy.rows, galaxy.cols
 *   —            §9 screen          ncurses init / palette load
 *   —            §10 app            signals, the frame loop, key events
 *
 * EFFECTS is woven in, not a standalone layer: galaxy.bright is cosmetic-only
 * trail state (a per-cell brightness accumulator with an exponential fade), but
 * its two mutations are inline — galaxy_step ACCUMULATES star splats into it (§5)
 * and scene_draw FADES it once per frame (bright *= DECAY at the top of §7).
 * Extracting the fade into its own EFFECTS step is deferred; for now it is the
 * single, deliberate place RENDER mutates state, called out here so it is not a
 * surprise. The stars' physics (r, theta, omega) never reads bright, so the
 * trail can never affect the orbits.
 *
 * LOGIC (rng_next / rng_float / rng_gauss) does no I/O and touches only its own
 * PRNG word g_rng — it never reads the Galaxy or the screen — so reordering or
 * deleting RENDER/EFFECTS cannot change a LOGIC result.
 *
 * No DELAYS layer: pause is a single flag (galaxy.paused) tested once in main
 * before the tick. The sim-rate gate (run galaxy.steps galaxy_steps only when
 * now >= next_tick) is PERFORMANCE, living in main's loop (§10), not a hold.
 *
 * PER-TICK COMBINE — main's loop (§10) is the ONLY place sim state advances, in
 * order, and only when not paused and the sim-clock is due:
 *   1. for s in 0..galaxy.steps: galaxy_step(&galaxy)  (orbits + trail accumulate)
 *   then every frame:  scene_draw(&galaxy)             (trail fade + project + HUD)
 *
 * User events (quit, pause, reset r, arms a/A, theme t/T, speed +/-, fps [/],
 * steps </>, resize) DO mutate state but are NOT part of the tick — they run in
 * main's input/resize handling, before the gated simulate step. Reset r, arm
 * change, and resize re-invoke INIT (galaxy_init, §6).
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define ROWS_MAX      128
#define COLS_MAX      512
#define N_STARS       3000   /* total stars */
#define ARMS_DEF      2      /* starting arm count */
#define ARMS_MIN      2
#define ARMS_MAX      4
#define N_THEMES      5

#define STEPS_DEF     2      /* physics steps per rendered frame */
#define STEPS_MIN     1
#define STEPS_MAX    16
#define SIM_FPS_DEF  20
#define SIM_FPS_MIN   5
#define SIM_FPS_MAX  60
#define SIM_FPS_STEP  5

/*
 * Flat rotation curve: v_circ = V0 (constant for all r).
 * Angular velocity: omega(r) = V0 / r  →  inner orbits faster.
 * V0 is in units of (normalized_radius / step).
 */
#define V0_DEF        0.006f

/* Brightness accumulator decays by this factor once per rendered frame */
#define DECAY         0.82f

/* Log-spiral tightness (higher = more wound) */
#define WINDING       1.0f

/* Angular scatter around each arm (rad, ±half-width) */
#define ARM_SCATTER   0.25f

#define SPEED_DEF     1.0f
#define SPEED_MIN     0.1f
#define SPEED_MAX     5.0f
#define SPEED_STEP    0.1f

#define NS_PER_SEC    1000000000LL
#define TICK_NS(f)    (NS_PER_SEC / (f))

static const float PI = 3.14159265358979f;

/* ===================================================================== */
/* §2  PERFORMANCE — clock                                                 */
/* ===================================================================== *
 * Monotonic clock + sleep. Mutates nothing. The sim-rate gate (TICK_NS,
 * next_tick) and frame sleep that use these live in main's loop (§10).      */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                            .tv_nsec = (long)(ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  DATA — types, palette table & the Galaxy aggregate                 */
/* ===================================================================== *
 * Declarations only; no behaviour. The Galaxy aggregate is mutated by
 * SIMULATION (§5) / INIT (§6) / RENDER (§7, the trail fade) / EVENTS (§8) and
 * read by RENDER; the infra globals (g_has_256, g_rng) sit alongside it.       */

/* Colour-pair ids (ncurses pairs are 1-BASED — pair 0 is the reserved default).
 * The display is a TWO-CHANNEL encoding: the GLYPH carries local brightness (a
 * Bourke-style density ramp), the COLOUR carries the star's radial ZONE — so a
 * reader sees structure (bulge vs arms vs halo) and density at the same time.
 * The three star zones mirror the morphology of a real disc galaxy and split at
 * fixed fractions of the normalised radius (r_core=0.10, r_disk=0.65 in
 * scene_draw):
 *   CP_CORE — stellar bulge        (innermost < 10% of radius)
 *   CP_DISK — spiral arms and disc (10%..65%)
 *   CP_HALO — outer diffuse halo   (> 65%)
 *   CP_HUD / CP_HINT — the two HUD bars (yellow data row, cyan action row). */
enum { CP_CORE=1, CP_DISK, CP_HALO, CP_HUD, CP_HINT };

/* Theme — one named RENDER palette: a foreground colour for each of the three
 * radial zones (core / disk / halo), data-driven so the whole look swaps at
 * runtime (t/T cycles k_themes) with zero code change. Two PARALLEL sets are
 * stored — xterm-256 indices (16..231 = 6×6×6 colour cube, 232..255 = grays)
 * and an 8-colour fallback — so theme_apply can pick by terminal capability
 * (g_has_256). Foregrounds are always on a black bg (deep space). The live
 * choice is Galaxy.theme; one row per theme.
 *   core256/disk256/halo256 : 256-colour fg per zone (bulge / arms / halo)
 *   core8/disk8/halo8       : 8-colour fallback for the same three zones
 *   name                    : label shown in the HUD */
typedef struct {
    short core256, disk256, halo256;   /* 256-colour fg on black bg */
    short core8,   disk8,   halo8;     /* 8-colour fg fallback */
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 MilkyWay  — white core, cyan arms, grey halo */
    { 231,  39, 240,  COLOR_WHITE, COLOR_CYAN,    COLOR_WHITE,   "MilkyWay" },
    /* 1 Starburst — yellow core, blue arms, dark grey halo */
    { 226,  33, 244,  COLOR_YELLOW, COLOR_BLUE,   COLOR_WHITE,   "Starburst" },
    /* 2 Nebula    — white core, pink arms, purple halo */
    { 231, 207,  92,  COLOR_WHITE,  COLOR_MAGENTA, COLOR_MAGENTA, "Nebula" },
    /* 3 Infrared  — white core, red arms, dark-red halo */
    { 231, 196,  52,  COLOR_WHITE,  COLOR_RED,     COLOR_RED,     "Infrared" },
    /* 4 Aurora    — white core, bright-green arms, dark-green halo */
    { 231,  46,  28,  COLOR_WHITE,  COLOR_GREEN,   COLOR_GREEN,   "Aurora" },
};

static bool g_has_256;   /* terminal 256-colour capability (render infra, set once) */

/* Star — one point mass on a FIXED circular orbit (no N-body gravity). INTENT /
 * the physics it encodes: a FLAT ROTATION CURVE — every star has the same
 * tangential speed V0 regardless of radius, the hallmark of real spiral galaxies
 * (Rubin & Ford 1970; explained by dark-matter halos). With constant V0 the
 * angular velocity is omega = V0/r (Binney & Tremaine, "Galactic Dynamics"), so
 * inner stars sweep round faster than outer ones — DIFFERENTIAL ROTATION, which
 * is exactly why the arms wind up over time (the winding problem; Toomre 1977).
 * The spiral is NOT computed from forces: it emerges purely from the initial
 * logarithmic-spiral placement (galaxy_init) plus this radial dependence of
 * omega. omega is precomputed at spawn and never changes — only theta advances.
 *   r     : orbital radius, normalised so 1.0 = edge of the display.
 *   theta : current orbital angle (radians); the ONLY field a tick advances.
 *   omega : angular velocity (rad/step) = V0/r, fixed at spawn. */
typedef struct {
    float r;
    float theta;
    float omega;
} Star;

/*
 * Galaxy — the whole simulation in one aggregate, read like a table of
 * contents. WHY one aggregate: state lives in one place, yet the orchestrators
 * (galaxy_init / galaxy_step / scene_draw) take Galaxy* while utilities take the
 * narrowest value they need, so the layers never re-couple. (PRNG state, the
 * 256-colour flag and the signal flags stay module-global — infrastructure, not
 * galaxy data.)
 *   WHAT   — stars[]: the orbiting star pool;  bright[][]: the per-cell
 *            brightness accumulator the galaxy is drawn from — splatted each
 *            step, faded each frame (the cosmetic trail; see ARCHITECTURE).
 *   WHERE  — rows, cols: the galaxy grid extent in cells, sized from the
 *            terminal at startup/resize.
 *   HOW    — the user-tunable knobs: narms (spiral arm count), v0 (flat-
 *            rotation-curve speed), speed (orbit multiplier), steps (physics
 *            sub-steps per frame), sim_fps (simulation rate, Hz).
 *   WHEN   — paused (1 freezes the per-tick advance; rendering continues).
 *   RENDER — theme: the selected k_themes[] palette index (a render choice
 *            cycled by t/T, kept here so resize preserves it).
 */
typedef struct {
    /* WHAT — the simulated objects */
    Star  stars[N_STARS];                /* the orbiting star pool (seeded in galaxy_init)     */
    float bright[ROWS_MAX][COLS_MAX];    /* per-cell brightness accumulator (the fade trail)   */
    /* WHERE — display geometry */
    int   rows, cols;                    /* galaxy grid extent in cells (<= ROWS_MAX/COLS_MAX)  */
    /* HOW — user-tunable simulation knobs */
    int   narms;                         /* spiral arm count [ARMS_MIN..ARMS_MAX]               */
    float v0;                            /* flat rotation-curve speed (norm. radius / step)     */
    float speed;                         /* orbit-speed multiplier [SPEED_MIN..SPEED_MAX]       */
    int   steps;                         /* physics sub-steps per frame [STEPS_MIN..STEPS_MAX]  */
    int   sim_fps;                       /* simulation rate, Hz [SIM_FPS_MIN..SIM_FPS_MAX]      */
    /* WHEN — run state */
    bool  paused;                        /* 1 freezes the per-tick advance (render continues)   */
    /* RENDER — palette selection */
    int   theme;                         /* index into k_themes[]  [0..N_THEMES)                */
} Galaxy;

static Galaxy g_galaxy = {
    .narms   = ARMS_DEF,
    .v0      = V0_DEF,
    .speed   = SPEED_DEF,
    .steps   = STEPS_DEF,
    .sim_fps = SIM_FPS_DEF,
    /* bright/stars/rows/cols/paused/theme → zero-init */
};

/* xorshift32 PRNG state (seeded in main; advanced only by §4 LOGIC) */
static uint32_t g_rng = 12345u;

/* ===================================================================== */
/* §4  LOGIC — RNG & sampling (pure: no I/O, touches only g_rng)           */
/* ===================================================================== */

static inline uint32_t rng_next(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng <<  5;
    return g_rng;
}
static inline float rng_float(void)
{
    return (float)(rng_next() >> 8) / (float)(1u << 24);
}

/* Box-Muller normal sample */
static float rng_gauss(void)
{
    float u1 = rng_float() + 1e-6f;
    float u2 = rng_float();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}

/* ===================================================================== */
/* §5  SIMULATION — galaxy_step (advances orbits + accumulates the trail)  */
/* ===================================================================== *
 * The ONLY function that advances simulation state. Mutates g->stars[].theta
 * (each orbit) and splats live stars into g->bright (the EFFECTS trail). The
 * per-tick combine in main runs this g->steps times when the sim-clock is due. */

static void galaxy_step(Galaxy *g)
{
    /*
     * Screen centre and scale factors.
     * ry = rx * 0.5 corrects for terminal cells being ~2× taller than wide,
     * making the galaxy appear circular rather than vertically stretched.
     */
    int   cx = g->cols / 2, cy = g->rows / 2;
    float rx = g->cols * 0.44f;
    float ry = rx * 0.50f;

    /*
     * Weight per star per step: dividing by g->steps keeps total brightness
     * per frame constant regardless of how many physics steps we compute.
     */
    float w = 1.0f / (float)g->steps;

    for (int i = 0; i < N_STARS; i++) {
        /* Advance along circular orbit */
        g->stars[i].theta += g->stars[i].omega * g->speed;

        /* Convert polar → Cartesian → screen coordinates */
        float x = g->stars[i].r * cosf(g->stars[i].theta);
        float y = g->stars[i].r * sinf(g->stars[i].theta);
        int   sx = cx + (int)(x * rx + 0.5f);
        int   sy = cy + (int)(y * ry + 0.5f);

        if (sx >= 0 && sx < g->cols && sy >= 0 && sy < g->rows)
            g->bright[sy][sx] += w;
    }
}

/* ===================================================================== */
/* §6  INIT/RESET — galaxy_init (full reseed, NOT part of the tick)        */
/* ===================================================================== *
 * Mutates g->stars / g->bright / g->narms wholesale. Called at startup and on
 * 'r', arm change (a/A), and resize — all outside the gated simulate step.   */

static void galaxy_init(Galaxy *g, int narms)
{
    g->narms = narms;
    memset(g->bright, 0, sizeof g->bright);

    int n = 0;

    /*
     * BULGE — 20% of stars: tightly concentrated around the centre.
     * Radius drawn from a half-Gaussian (always positive) so stars pile up
     * near r=0 to form the bright nuclear region.
     */
    int n_bulge = N_STARS / 5;
    for (int i = 0; i < n_bulge; i++) {
        float r = fabsf(rng_gauss() * 0.07f);
        if (r < 0.004f) r = 0.004f;
        if (r > 0.20f)  r = 0.20f;
        g->stars[n++] = (Star){ r, rng_float() * 2.0f * PI, g->v0 / r };
    }

    /*
     * ARMS — 70% of stars: placed on logarithmic spirals.
     * For arm k (0-indexed), the initial angle at radius r is:
     *   theta = (k * 2π/narms) + WINDING * ln(r / r_min) + noise
     * This is a logarithmic spiral: r = r_min * exp((theta - start) / WINDING)
     */
    int n_arm_total = N_STARS * 7 / 10;
    for (int i = 0; i < n_arm_total; i++) {
        int   arm   = i % narms;
        float a_off = arm * (2.0f * PI / (float)narms);
        float r     = 0.08f + rng_float() * 0.87f;   /* 0.08 .. 0.95 */
        float theta = a_off + WINDING * logf(r / 0.08f)
                      + (rng_float() - 0.5f) * (2.0f * ARM_SCATTER);
        g->stars[n++] = (Star){ r, theta, g->v0 / r };
    }

    /*
     * HALO — remaining ~10%: scattered uniformly at large radii.
     * These represent field stars and the outer diffuse disc.
     */
    while (n < N_STARS) {
        float r = 0.35f + rng_float() * 0.70f;
        if (r > 1.05f) r = 1.05f;
        g->stars[n++] = (Star){ r, rng_float() * 2.0f * PI, g->v0 / r };
    }
}

/* ===================================================================== */
/* §7  RENDER — palette load + state → screen                             */
/* ===================================================================== *
 * theme_apply loads colour pairs; scene_draw projects state to the screen and
 * draws the HUD. scene_draw ALSO fades the trail (g->bright *= DECAY) at the top
 * — the single, deliberate place RENDER mutates state (see ARCHITECTURE).      */

static void theme_apply(int ti)
{
    const Theme *t = &k_themes[ti];
    if (g_has_256) {
        init_pair(CP_CORE, t->core256, COLOR_BLACK);
        init_pair(CP_DISK, t->disk256, COLOR_BLACK);
        init_pair(CP_HALO, t->halo256, COLOR_BLACK);
    } else {
        init_pair(CP_CORE, t->core8, COLOR_BLACK);
        init_pair(CP_DISK, t->disk8, COLOR_BLACK);
        init_pair(CP_HALO, t->halo8, COLOR_BLACK);
    }
    init_pair(CP_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);  /* bright yellow — top data bar    */
    init_pair(CP_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);  /* bright cyan   — bottom action bar */
}

static void scene_draw(Galaxy *g)
{
    /*
     * Decay the brightness grid once per rendered frame.
     * Steady-state brightness for a cell receiving f stars/frame:
     *   B_ss = f / (1 - DECAY)
     * With DECAY=0.82, B_ss = f * 5.56  →  bright core, faint halo.
     */
    for (int r = 0; r < g->rows; r++)
        for (int c = 0; c < g->cols; c++)
            g->bright[r][c] *= DECAY;

    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    int draw_rows = (g->rows < rows - 2) ? g->rows : rows - 2;
    int draw_cols = (g->cols < cols)     ? g->cols : cols;

    /*
     * Find peak brightness so we can normalise to [0,1].
     * This auto-calibrates the display for any star density or speed.
     */
    float b_max = 0.1f;
    for (int r = 0; r < draw_rows; r++)
        for (int c = 0; c < draw_cols; c++)
            if (g->bright[r][c] > b_max) b_max = g->bright[r][c];

    /* Radial colour zone boundaries (normalised galaxy radius) */
    int   cx = g->cols / 2, cy = g->rows / 2;
    float rx = g->cols * 0.44f, ry = rx * 0.50f;
    float r_core = 0.10f;    /* < 10% → core colour */
    float r_disk = 0.65f;    /* 10%..65% → disk colour; > 65% → halo */

    for (int r = 0; r < draw_rows; r++) {
        for (int c = 0; c < draw_cols; c++) {
            float b = g->bright[r][c];
            if (b < 0.02f * b_max) continue;   /* below noise floor — skip */

            /* Normalised brightness 0..1 → character */
            float t = b / b_max;
            char  ch;
            int   bold = 0;
            if      (t < 0.12f) ch = '.';
            else if (t < 0.25f) ch = ',';
            else if (t < 0.40f) ch = ':';
            else if (t < 0.55f) ch = 'o';
            else if (t < 0.70f) ch = 'O';
            else if (t < 0.85f) { ch = '0'; bold = A_BOLD; }
            else                { ch = '@'; bold = A_BOLD; }

            /* Normalised screen radius → colour zone */
            float dx = (float)(c - cx) / rx;
            float dy = (float)(r - cy) / ry;
            float rn = sqrtf(dx*dx + dy*dy);
            int cp = (rn < r_core) ? CP_CORE
                   : (rn < r_disk) ? CP_DISK
                                   : CP_HALO;

            attron(COLOR_PAIR(cp) | bold);
            mvaddch(r + 1, c, (chtype)ch);   /* +1: row 0 is the top HUD bar */
            attroff(COLOR_PAIR(cp) | bold);
        }
    }

    /* Galactic centre — always mark even when no stars land exactly here */
    if (cy < draw_rows && cx < draw_cols) {
        attron(COLOR_PAIR(CP_CORE) | A_BOLD);
        mvaddch(cy + 1, cx, '*');            /* +1: row 0 is the top HUD bar */
        attroff(COLOR_PAIR(CP_CORE) | A_BOLD);
    }

    /* ── HUD (standard two bars) ──────────────────────────────────────── *
     * Top row 0      DATA    — title (left) + sim stats (right-aligned, yellow).
     * Bottom rows-1  ACTIONS — the key legend (cyan).
     * Both rows are filled first, then clipped with "%.*s" so a narrow terminal
     * can neither overflow nor wrap. */
    char left[20], right[80];
    snprintf(left,  sizeof left,  " SPIRAL GALAXY ");
    snprintf(right, sizeof right,
             " arms:%d  speed:%.1fx  theme:%s  fps:%d  steps:%d  %s ",
             g->narms, g->speed, k_themes[g->theme].name,
             g->sim_fps, g->steps, g->paused ? "PAUSED" : "running");
    int stat_x = cols - (int)strlen(right);      /* right-aligned stats column */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(0, c, ' ');
    if (stat_x >= 0) {
        mvprintw(0, 0,      "%.*s", stat_x, left);  /* title clipped clear of stats */
        mvprintw(0, stat_x, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, right);    /* too narrow: data only */
    }
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(rows - 1, c, ' ');
    mvprintw(rows - 1, 0, "%.*s", cols,
             " q:quit  p:pause  r:reset  a/A:arms  t/T:theme  +/-:speed  [/]:fps  </>:steps ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §8  EVENTS — signals & resize (mutate state, NOT part of the tick)      */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_resize(Galaxy *g)
{
    endwin(); refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g->rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
    g->cols = (cols      < COLS_MAX) ? cols      : COLS_MAX;
    g_resize = 0;
}

/* ===================================================================== */
/* §9  screen — ncurses init / palette load                               */
/* ===================================================================== */

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    start_color();
    use_default_colors();
    g_has_256 = (COLORS >= 256);
    theme_apply(theme);
}

/* ===================================================================== */
/* §10  app — signals, the frame loop, key events (the per-tick combine)   */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);

    g_rng = (uint32_t)time(NULL) ^ 0xC0DE4A1Au;
    screen_init(g_galaxy.theme);

    { int rows, cols; getmaxyx(stdscr, rows, cols);
      g_galaxy.rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
      g_galaxy.cols = (cols      < COLS_MAX) ? cols      : COLS_MAX; }

    galaxy_init(&g_galaxy, g_galaxy.narms);
    int64_t next_tick = clock_ns();

    while (!g_quit) {

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case 'p': case ' ': g_galaxy.paused = !g_galaxy.paused; break;
            case 'r': galaxy_init(&g_galaxy, g_galaxy.narms); break;

            case 'a':
                g_galaxy.narms = (g_galaxy.narms < ARMS_MAX) ? g_galaxy.narms + 1 : ARMS_MIN;
                galaxy_init(&g_galaxy, g_galaxy.narms);
                break;
            case 'A':
                g_galaxy.narms = (g_galaxy.narms > ARMS_MIN) ? g_galaxy.narms - 1 : ARMS_MAX;
                galaxy_init(&g_galaxy, g_galaxy.narms);
                break;

            case 't':
                g_galaxy.theme = (g_galaxy.theme + 1) % N_THEMES;
                theme_apply(g_galaxy.theme);
                break;
            case 'T':
                g_galaxy.theme = (g_galaxy.theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_galaxy.theme);
                break;

            case '+': case '=':
                if (g_galaxy.speed < SPEED_MAX - 0.01f) g_galaxy.speed += SPEED_STEP;
                break;
            case '-':
                if (g_galaxy.speed > SPEED_MIN + 0.01f) g_galaxy.speed -= SPEED_STEP;
                break;

            case ']':
                if (g_galaxy.sim_fps < SIM_FPS_MAX) g_galaxy.sim_fps += SIM_FPS_STEP;
                break;
            case '[':
                if (g_galaxy.sim_fps > SIM_FPS_MIN) g_galaxy.sim_fps -= SIM_FPS_STEP;
                break;

            case '>':
                if (g_galaxy.steps < STEPS_MAX) g_galaxy.steps++;
                break;
            case '<':
                if (g_galaxy.steps > STEPS_MIN) g_galaxy.steps--;
                break;
            }
        }

        /* ── resize ── */
        if (g_resize) { screen_resize(&g_galaxy); galaxy_init(&g_galaxy, g_galaxy.narms); }

        /* ── simulate ── */
        int64_t now = clock_ns();
        if (!g_galaxy.paused && now >= next_tick) {
            for (int s = 0; s < g_galaxy.steps; s++) galaxy_step(&g_galaxy);
            next_tick = now + TICK_NS(g_galaxy.sim_fps);
        }

        /* ── render ── */
        scene_draw(&g_galaxy);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── sleep until next frame ── */
        clock_sleep_ns(next_tick - clock_ns() - 1000000LL);
    }

    endwin();
    return 0;
}
