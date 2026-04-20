/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fluid/cfl_stability_explorer.c — Interactive CFL Stability Visualizer
 *
 * Runs the same explicit 2-D wave equation as membrane.c but makes the
 * timestep dt a freely adjustable parameter so you can drive the solver
 * across the CFL stability boundary and WATCH the transition happen.
 *
 * The Courant-Friedrichs-Lewy (CFL) condition for 2-D explicit wave:
 *
 *   CFL_2D = c · dt · √2        must be  < 1  (stable)
 *                                         = 1  (critically stable)
 *                                         > 1  (UNSTABLE — blows up)
 *
 * where c = wave speed [cells/s],  dt = physics timestep [s].
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *  §1  config       — all tunable constants
 *  §2  clock        — monotonic nanosecond clock + sleep
 *  §3  theme        — signed-amplitude color pipeline; CFL zone colors
 *  §4  grid         — static field arrays g_u, g_v
 *  §5  solver       — init_wave, update_wave, compute_CFL, detect_instability
 *  §6  render       — render_field, render_cfl_meter, render_dt_slider,
 *                     render_sparkline, render_overlay
 *  §7  scene        — Scene struct, scene_init/tick/draw/reset/set_cfl
 *  §8  screen       — ncurses double-buffer display layer
 *  §9  app          — signals, resize, input, main loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q/ESC   quit          space  pause/resume    s    single step
 *   +/-     dt ±5%        c/C    wave speed ±    r    reset field
 *   1       CFL=0.25 (deep stable)
 *   2       CFL=0.50 (stable)
 *   3       CFL=0.70 (stable/marginal boundary)
 *   4       CFL=0.85 (marginal)
 *   5       CFL=0.95 (near-critical stable)
 *   6       CFL=0.99 (one '+' press from instability)
 *           then press '+' repeatedly to cross CFL=1.0 and watch it blow up
 *   m       resonance mode (pure eigenmode for clean analysis)
 *   d       Gaussian drop at centre (additive)
 *   t       cycle theme      [/]  sim Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/cfl_stability_explorer.c \
 *       -o cfl_explorer -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * What the CFL number measures (§5 compute_CFL):
 *   CFL = c · dt / dx.  In words: "how many grid cells does information
 *   travel in one timestep?"  When CFL < 1, numerical information propagates
 *   at most one cell per step — the scheme is causal.  When CFL > 1, the
 *   numerical domain of dependence is SMALLER than the physical one: the
 *   scheme is trying to describe physics that happened before it could
 *   "see" it.  This causality violation causes errors to amplify rather
 *   than propagate, and the solution blows up.
 *
 * Von Neumann stability analysis (§5 detect_instability):
 *   We study how a single Fourier mode u = U·exp(iωn·dt) evolves under the
 *   velocity-field update.  For the worst-case mode (shortest wavelength,
 *   k = l = π, i.e. alternating +1/-1 on every cell), the discrete
 *   Laplacian eigenvalue is -8 (with dx=1):
 *
 *     ∇²u|_{k=l=π} = -8u
 *
 *   The per-step amplification matrix A is:
 *
 *     [u_{n+1}]   [1 - 8c²dt²   dt] [u_n]
 *     [v_{n+1}] = [-8c²dt        1 ] [v_n]
 *
 *   eigenvalues μ satisfy: μ² - (2 - 8c²dt²)μ + 1 = 0.
 *   Let β = 8c²dt² = 4·CFL_2D²:
 *
 *     μ = [(2-β) ± √((2-β)²-4)] / 2
 *
 *   • If β ≤ 4  (CFL_2D ≤ 1): discriminant ≤ 0 → complex roots on the
 *     unit circle → |μ| = 1 → amplitude stays CONSTANT forever.
 *     The scheme is neutrally stable: zero numerical dissipation.
 *
 *   • If β > 4  (CFL_2D > 1): discriminant > 0 → two real roots, one
 *     larger than 1 in magnitude.  Each step multiplies amplitude by
 *     |μ_max| > 1 → EXPONENTIAL GROWTH.  The simulation "explodes".
 *
 * Growth rate formula (§5 detect_instability):
 *   Let γ = β - 2 > 2 (when β > 4):
 *     disc    = √(γ² - 4)
 *     μ_max   = (-γ - disc) / 2        (both negative, take abs)
 *     |μ_max| = (γ + disc) / 2
 *
 *   growth_per_step = |μ_max|    e.g. 1.33 means +33% per tick
 *   time_to_double  = ln(2) / (ln(|μ_max|) · TARGET_FPS)   seconds
 *
 * Why changing dt matters (relationship dt–dx–c):
 *   CFL = c · dt / dx.  Three ways to make the scheme unstable:
 *     1. Increase dt  — take bigger timesteps (what this explorer does).
 *     2. Increase c   — faster wave speed (also adjustable here).
 *     3. Decrease dx  — refine the spatial grid (finer resolution).
 *   In real simulation code, a common bug is refining the grid without
 *   proportionally reducing dt — this silently crosses the CFL boundary.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#define M_SQRT2  1.41421356237309504880

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
    TARGET_FPS      = 60,           /* render rate; also = physics steps/s */
    FPS_UPDATE_MS   = 500,
    HUD_COLS        = 80,
    AMP_HIST_LEN    = 280,          /* sparkline ring buffer; fills wide terminals */
};

#define GRID_MAX_W   300
#define GRID_MAX_H   100

/* Wave speed c (cells/s) — sets the CFL jointly with dt */
#define WAVE_SPEED_DEFAULT   35.0f   /* CFL_2D ≈ 0.825 at dt=1/60 */
#define WAVE_SPEED_MIN        5.0f
#define WAVE_SPEED_MAX       55.0f
#define WAVE_SPEED_STEP       5.0f

/*
 * Physics timestep dt — THE key control in this explorer.
 * Unlike membrane.c where dt = 1/sim_fps is fixed, here dt is a free
 * parameter that the user adjusts to cross the stability boundary.
 *
 * dt_default = 1/TARGET_FPS  (same as membrane.c baseline)
 * dt_min     = 1/TARGET_FPS / 20   (very small — always stable)
 * dt_max     = 1/TARGET_FPS * 8    (very large — always unstable)
 * dt_mult    = 1.05f               (±5% per keypress)
 *
 * CFL at defaults: 35 · (1/60) · √2 ≈ 0.825  (MARGINAL zone)
 * Critical dt for stability: 1/(c·√2) = 1/(35·1.414) ≈ 0.0202 s
 */
#define DT_DEFAULT    (1.0f / TARGET_FPS)
#define DT_MIN        (DT_DEFAULT / 20.0f)
#define DT_MAX        (DT_DEFAULT * 8.0f)
#define DT_MULT       1.05f               /* multiplicative step for +/- keys */

/* Excitation */
#define DROP_AMP      1.2f
#define DROP_RADIUS   3.5f
#define RESONANCE_AMP 1.0f

/* Display */
#define DISPLAY_RANGE  1.8f       /* |u| outside ±RANGE clipped   */
#define EXPL_THRESH  20000.0f     /* auto-reset when |u| exceeds this */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

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
    struct timespec r = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  theme — signed-amplitude color pipeline + CFL zone colors         */
/* ===================================================================== */

static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N  (int)(sizeof k_ramp - 1)

static const float k_breaks[RAMP_N] = {
    0.000f, 0.030f, 0.090f, 0.200f, 0.340f,
    0.500f, 0.660f, 0.820f, 0.940f,
};

static int lut_index(float v)
{
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return RAMP_N - 1;
    float g = powf(v, 1.0f / 2.2f);
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (g >= k_breaks[i]) return i;
    return 0;
}

/*
 * Color pair layout:
 *   Field  (3 themes × 9 levels × 2 signs):
 *     CP_POS(t,i) = crests (positive amplitude)
 *     CP_NEG(t,i) = troughs (negative amplitude)
 *   Stability zones:
 *     CP_STABLE   = green   (CFL < 0.70)
 *     CP_MARGINAL = yellow  (0.70 ≤ CFL < 1.0)
 *     CP_UNSTABLE = red     (CFL ≥ 1.0)
 *     CP_WARN     = bold bright red (blinking warning)
 *   Overlay:
 *     CP_HUD      = amber
 */
#define N_THEMES    3
#define CP_POS(t,i) (1 + (t)*(RAMP_N*2) + (i))
#define CP_NEG(t,i) (1 + (t)*(RAMP_N*2) + RAMP_N + (i))
#define CP_STABLE   (1 + N_THEMES*(RAMP_N*2))
#define CP_MARGINAL (CP_STABLE + 1)
#define CP_UNSTABLE (CP_MARGINAL + 1)
#define CP_WARN     (CP_UNSTABLE + 1)
#define CP_HUD      (CP_WARN + 1)

typedef struct {
    const char *name;
    int pos256[RAMP_N]; int neg256[RAMP_N];
    int pos8  [RAMP_N]; int neg8  [RAMP_N];
} WaveTheme;

static const WaveTheme k_themes[N_THEMES] = {
    {   /* 0  wave */
        "wave",
        {  52,  88, 124, 160, 196, 202, 208, 220, 231 },
        {  17,  19,  21,  27,  33,  39,  45,  51, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
           COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
        {  COLOR_BLUE,   COLOR_BLUE,   COLOR_BLUE,   COLOR_BLUE,
           COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,  COLOR_WHITE },
    },
    {   /* 1  thermal */
        "thermal",
        {  52,  94, 130, 166, 202, 208, 214, 220, 231 },
        {  53,  54,  91,  92, 129, 165, 201, 207, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE },
        {  COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
           COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE },
    },
    {   /* 2  ocean */
        "ocean",
        {  23,  29,  36,  43,  51,  87, 123, 159, 231 },
        {  17,  18,  19,  20,  21,  27,  33,  39,  45 },
        {  COLOR_CYAN,  COLOR_CYAN,  COLOR_CYAN,   COLOR_CYAN,
           COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE },
        {  COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
           COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN },
    },
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    for (int t = 0; t < N_THEMES; t++) {
        for (int i = 0; i < RAMP_N; i++) {
            if (COLORS >= 256) {
                init_pair(CP_POS(t,i), k_themes[t].pos256[i], COLOR_BLACK);
                init_pair(CP_NEG(t,i), k_themes[t].neg256[i], COLOR_BLACK);
            } else {
                init_pair(CP_POS(t,i), k_themes[t].pos8[i], COLOR_BLACK);
                init_pair(CP_NEG(t,i), k_themes[t].neg8[i], COLOR_BLACK);
            }
        }
    }
    if (COLORS >= 256) {
        init_pair(CP_STABLE,   82,  COLOR_BLACK);   /* lime green  */
        init_pair(CP_MARGINAL, 220, COLOR_BLACK);   /* amber       */
        init_pair(CP_UNSTABLE, 196, COLOR_BLACK);   /* bright red  */
        init_pair(CP_WARN,     231, COLOR_RED);     /* white on red */
        init_pair(CP_HUD,      220, COLOR_BLACK);   /* amber HUD   */
    } else {
        init_pair(CP_STABLE,   COLOR_GREEN,  COLOR_BLACK);
        init_pair(CP_MARGINAL, COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_UNSTABLE, COLOR_RED,    COLOR_BLACK);
        init_pair(CP_WARN,     COLOR_WHITE,  COLOR_RED);
        init_pair(CP_HUD,      COLOR_YELLOW, COLOR_BLACK);
    }
}

static attr_t wave_attr(int theme, bool positive, int level)
{
    attr_t a = positive ? COLOR_PAIR(CP_POS(theme, level))
                        : COLOR_PAIR(CP_NEG(theme, level));
    if (level >= RAMP_N - 2) a |= A_BOLD;
    return a;
}

/* CFL → color pair for meter / labels */
static int cfl_color(float cfl)
{
    if (cfl < 0.70f) return CP_STABLE;
    if (cfl < 1.00f) return CP_MARGINAL;
    return CP_UNSTABLE;
}

static const char *cfl_label(float cfl)
{
    if (cfl < 0.70f) return "STABLE  ";
    if (cfl < 1.00f) return "MARGINAL";
    return "UNSTABLE";
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * g_u — displacement field u(x,y,t)  (signed, same as membrane.c)
 * g_v — velocity field  ∂u/∂t         (updated first each tick)
 *
 * We deliberately use the IDENTICAL physics as membrane.c.  The only
 * thing that changes is how dt is controlled — here it is decoupled from
 * the rendering framerate so the user can push it past the CFL boundary.
 */
static float g_u[GRID_MAX_H][GRID_MAX_W];
static float g_v[GRID_MAX_H][GRID_MAX_W];

/* ===================================================================== */
/* §5  solver                                                             */
/* ===================================================================== */

static void grid_zero(int cols, int rows)
{
    for (int r = 0; r < rows; r++) {
        memset(g_u[r], 0, (size_t)cols * sizeof(float));
        memset(g_v[r], 0, (size_t)cols * sizeof(float));
    }
}

/*
 * update_wave() — one explicit step of the 2-D wave equation.
 *
 * This is IDENTICAL to membrane.c §5 update_wave — velocity-field
 * (symplectic Euler) with no damping (γ=0).  No damping keeps the growth
 * of unstable modes purely from the CFL condition, not from other physics.
 *
 * The two loops MUST remain separate.  The first loop reads g_u (old),
 * writes g_v (new).  The second reads g_v (new), writes g_u (new).
 * Merging them would corrupt the Laplacian of u[r+1][c] with an already-
 * updated value — a subtle data race that changes the growth exponent.
 */
static void update_wave(float dt, float wave_speed, int cols, int rows)
{
    float c2 = wave_speed * wave_speed;

    /* STEP 1: velocity update — reads g_u, writes g_v */
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            float lap = g_u[r-1][c] + g_u[r+1][c]
                      + g_u[r][c-1] + g_u[r][c+1]
                      - 4.0f * g_u[r][c];
            g_v[r][c] += c2 * lap * dt;
        }
    }

    /* STEP 2: displacement update — reads g_v (new), writes g_u */
    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            g_u[r][c] += g_v[r][c] * dt;

    /* STEP 3: Dirichlet boundary (u=v=0 at edges) */
    for (int c = 0; c < cols; c++) {
        g_u[0][c] = g_u[rows-1][c] = 0.0f;
        g_v[0][c] = g_v[rows-1][c] = 0.0f;
    }
    for (int r = 0; r < rows; r++) {
        g_u[r][0] = g_u[r][cols-1] = 0.0f;
        g_v[r][0] = g_v[r][cols-1] = 0.0f;
    }
}

/*
 * compute_CFL() — the central quantity of this explorer.
 *
 * CFL_2D = c · dt · √2
 *
 * The √2 factor comes from the 2-D diagonal — a wavefront moving at 45°
 * travels √2 cell-lengths per time it advances one cell diagonally.
 * In 1-D: CFL = c·dt/dx.  In 2-D (square grid): CFL_2D = c·dt·√(1+1) = c·dt·√2.
 *
 * Critical threshold: CFL_2D = 1  ↔  dt_crit = 1/(c·√2).
 */
static float compute_CFL(float wave_speed, float dt)
{
    return wave_speed * dt * (float)M_SQRT2;
}

/*
 * InstabilityInfo — result of Von Neumann growth-rate analysis.
 *
 * When CFL_2D ≤ 1 (stable): |μ| = 1, amplitude stays bounded, t_double = ∞.
 * When CFL_2D > 1 (unstable): |μ| > 1, amplitude grows multiplicatively.
 *
 * Derivation (see CONCEPTS block):
 *   β = 4 · CFL_2D²
 *   If β > 4:  γ = β - 2,  disc = √(γ²-4)
 *              |μ_max| = (γ + disc) / 2     (larger root by magnitude)
 *
 *   Growth per real second:
 *     — We do exactly TARGET_FPS physics steps per second (one per frame).
 *     — Each step multiplies amplitude by |μ_max|.
 *     — After N steps: amplitude = A₀ · |μ_max|^N
 *     — Time to double: N = log(2)/log(|μ_max|) steps
 *                         = log(2)/(log(|μ_max|)·TARGET_FPS) seconds
 */
typedef struct {
    float growth_per_step;   /* |μ_max|; = 1.0 when stable             */
    float time_to_double;    /* real seconds to 2× amplitude; 1e30 stable */
    float critical_dt;       /* dt value at CFL_2D = 1.0               */
    float beta;              /* 4·CFL² — stability parameter           */
} InstabilityInfo;

static InstabilityInfo detect_instability(float wave_speed, float scene_dt)
{
    InstabilityInfo info;
    float cfl  = compute_CFL(wave_speed, scene_dt);
    info.beta  = 4.0f * cfl * cfl;               /* β = 4·CFL_2D²  */
    info.critical_dt = 1.0f / (wave_speed * (float)M_SQRT2);

    if (info.beta <= 4.0f) {
        /* Stable: complex eigenvalues on the unit circle */
        info.growth_per_step   = 1.0f;
        info.time_to_double    = 1.0e30f;
    } else {
        /* Unstable: eigenvalues real, one outside unit circle.
         * β-2 > 2  (since β > 4),  so γ > 2 always here.             */
        float gamma = info.beta - 2.0f;              /* γ = β-2 > 2 */
        float disc  = sqrtf(gamma*gamma - 4.0f);     /* √(γ²-4) > 0 */
        info.growth_per_step = (gamma + disc) * 0.5f;/* (γ+disc)/2  */
        float log_mu = logf(info.growth_per_step);
        info.time_to_double = (float)log(2.0) / (log_mu * (float)TARGET_FPS);
    }
    return info;
}

static float compute_max_amplitude(int cols, int rows)
{
    float mx = 0.0f;
    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++) {
            float a = fabsf(g_u[r][c]);
            /* NaN/Inf from float overflow means the field has already blown
             * up beyond float range — treat it as exceeding EXPL_THRESH so
             * the auto-reset fires instead of the simulation freezing blank. */
            if (!isfinite(a)) return EXPL_THRESH + 1.0f;
            if (a > mx) mx = a;
        }
    return mx;
}

static void apply_drop(float cx, float cy, float amp, float radius,
                       int cols, int rows)
{
    float inv2r2 = 1.0f / (2.0f * radius * radius);
    for (int r = 0; r < rows; r++) {
        float dy = (float)r - cy;
        for (int c = 0; c < cols; c++) {
            float dx = (float)c - cx;
            g_u[r][c] += amp * expf(-(dx*dx + dy*dy) * inv2r2);
        }
    }
}

/*
 * apply_resonance_mode() — initialise a pure (nx,ny) standing wave mode.
 *
 * This excites a SINGLE eigenmode of the Laplacian.  In the stable regime,
 * its amplitude is constant and frequency is exactly ω = c·π·√(nx²/W²+ny²/H²).
 * In the unstable regime, it grows at precisely the theoretical |μ| rate —
 * making growth_per_step directly observable by watching max_amplitude.
 */
static void apply_resonance_mode(int nx, int ny, float amp, int cols, int rows)
{
    float kx = (float)nx * (float)M_PI / (float)(cols - 1);
    float ky = (float)ny * (float)M_PI / (float)(rows - 1);
    for (int r = 0; r < rows; r++) {
        float sy = sinf(ky * (float)r);
        for (int c = 0; c < cols; c++) {
            g_u[r][c] = amp * sinf(kx * (float)c) * sy;
            g_v[r][c] = 0.0f;
        }
    }
}

/* ===================================================================== */
/* §6  render                                                             */
/* ===================================================================== */

/*
 * Layout: the terminal is split into two distinct zones.
 *
 *   Rows  0 .. field_rows-1  : wave field (heatmap)
 *   Rows  field_rows .. rows-1 : CFL dashboard (full-width panel)
 *
 * field_rows = rows - DASH_H.  DASH_H = 8 rows for the dashboard.
 * This split makes the explorer look nothing like membrane.c, which uses
 * a full-screen field with a small corner overlay.
 *
 * Field coloring changes with the stability zone:
 *   CFL < 0.70  → normal signed-amplitude (warm crests, cool troughs)
 *   CFL < 1.00  → all cells shift to yellow/amber (marginal warning)
 *   CFL ≥ 1.00  → all cells red/alarm + blinking centre warning text
 *
 * The fastest-growing instability mode (k=l=π) produces an alternating
 * +1/-1 checkerboard.  In alarm mode the sign distinction is dropped and
 * everything goes red — this makes the checkerboard dramatically visible
 * as a field of alternating bright/dim red cells.
 */
#define DASH_H  8    /* rows reserved for the bottom dashboard */

static void render_field(WINDOW *w, int cols, int field_rows, int theme,
                         float cfl, float display_max, int blink_frame)
{
    if (display_max < 0.05f) display_max = 0.05f;
    float inv     = 1.0f / display_max;
    bool unstable = (cfl >= 1.0f);
    bool marginal = (cfl >= 0.7f && cfl < 1.0f);
    bool blink_on = (blink_frame / 8) % 2 == 0;

    for (int r = 0; r < field_rows; r++) {
        for (int c = 0; c < cols; c++) {
            float u    = g_u[r][c];
            bool  pos  = (u >= 0.0f);
            float norm = fabsf(u) * inv;
            if (norm > 1.0f) norm = 1.0f;
            int lvl = lut_index(norm);
            if (lvl == 0) continue;

            attr_t attr;
            if (unstable) {
                /* Alarm mode: drop sign distinction — everything red.
                 * The alternating +/- checkerboard shows up as alternating
                 * bright-red ('#'/'@') and dim-red ('.'/':') cells.     */
                if (!blink_on && lvl >= RAMP_N / 2) {
                    attr = COLOR_PAIR(CP_WARN) | A_BOLD;
                } else {
                    attr = COLOR_PAIR(CP_UNSTABLE);
                    if (lvl >= RAMP_N - 2) attr |= A_BOLD;
                }
            } else if (marginal) {
                /* Marginal: amber tint — same for both signs so the
                 * field looks uniformly warm, like a caution light.     */
                attr = COLOR_PAIR(CP_MARGINAL);
                if (lvl >= RAMP_N - 2) attr |= A_BOLD;
            } else {
                /* Stable: normal signed-amplitude coloring              */
                attr = wave_attr(theme, pos, lvl);
            }
            wattron(w, attr);
            mvwaddch(w, r, c, k_ramp[lvl]);
            wattroff(w, attr);
        }
    }

    /* Centre alarm text — overwrites field when CFL ≥ 1 */
    if (unstable && field_rows >= 5) {
        int wy = field_rows / 2;
        char line0[64], line1[64], line2[64];
        snprintf(line0, sizeof line0, "  !!!  NUMERICAL INSTABILITY  !!!  ");
        snprintf(line1, sizeof line1, "  !!!  CFL = %5.3f > 1.0      !!!  ", cfl);
        snprintf(line2, sizeof line2, "  !!!  increase dt is ILLEGAL  !!!  ");
        int wx0 = (cols - (int)strlen(line0)) / 2;
        int wx1 = (cols - (int)strlen(line1)) / 2;
        int wx2 = (cols - (int)strlen(line2)) / 2;
        if (wx0 < 0) wx0 = 0;
        if (wx1 < 0) wx1 = 0;
        if (wx2 < 0) wx2 = 0;
        if (blink_on) {
            wattron(w, COLOR_PAIR(CP_WARN) | A_BOLD);
            mvwprintw(w, wy - 1, wx0, "%s", line0);
            mvwprintw(w, wy,     wx1, "%s", line1);
            mvwprintw(w, wy + 1, wx2, "%s", line2);
            wattroff(w, COLOR_PAIR(CP_WARN) | A_BOLD);
        } else {
            wattron(w, COLOR_PAIR(CP_UNSTABLE) | A_BOLD);
            mvwprintw(w, wy - 1, wx0, "%s", line0);
            mvwprintw(w, wy,     wx1, "%s", line1);
            mvwprintw(w, wy + 1, wx2, "%s", line2);
            wattroff(w, COLOR_PAIR(CP_UNSTABLE) | A_BOLD);
        }
    }
}

/*
 * render_wide_cfl_meter() — full-width CFL bar spanning the terminal.
 *
 * Zones (proportional to bar_w):
 *   GREEN  [0 ..  0.70)  : stable       — CFL safely below 1
 *   YELLOW [0.70 .. 1.0) : marginal     — approaching the boundary
 *   RED    [1.0 .. 1.5]  : unstable     — beyond the critical threshold
 *
 * '#' = filled (current CFL), '.' = unfilled, '|' = CFL=1.0 threshold.
 * The filled red section blinks to reinforce the alarm state.
 */
static void render_wide_cfl_meter(WINDOW *w, int oy, int ox, int bar_w,
                                  float cfl, int blink_frame)
{
    const float cfl_max    = 1.5f;
    int stable_end   = (int)((0.70f / cfl_max) * bar_w);
    int critical_pos = (int)((1.00f / cfl_max) * bar_w);

    float ratio = cfl / cfl_max;
    if (ratio > 1.0f) ratio = 1.0f;
    int cur = (int)(ratio * bar_w);

    bool blink_on = (blink_frame / 8) % 2 == 0;

    for (int i = 0; i < bar_w; i++) {
        int  cp     = (i < stable_end)   ? CP_STABLE
                    : (i < critical_pos) ? CP_MARGINAL
                    :                      CP_UNSTABLE;
        bool filled = (i < cur);
        char ch     = (i == critical_pos) ? '|' : (filled ? '#' : '.');

        attr_t a = COLOR_PAIR(cp);
        if (filled && cp == CP_UNSTABLE && !blink_on)
            a = COLOR_PAIR(CP_WARN) | A_BOLD;
        else if (filled)
            a |= A_BOLD;

        wattron(w, a);
        mvwaddch(w, oy, ox + i, ch);
        wattroff(w, a);
    }
}

/*
 * render_wide_sparkline() — amplitude history spanning full terminal width.
 *
 * Flat line = stable bounded waves.
 * Rising line = unstable exponential growth.
 * The horizontal axis is time (oldest left, newest right).
 * Normalization is adaptive so both small and exploding amplitudes display.
 */
static void render_wide_sparkline(WINDOW *w, int oy, int ox, int bar_w,
                                  const float *hist, int head, int len)
{
    float mx = 0.001f;
    for (int i = 0; i < len; i++)
        if (hist[i] > mx) mx = hist[i];

    for (int i = 0; i < bar_w && i < len; i++) {
        int   idx = (head + i) % len;
        float nv  = hist[idx] / mx;
        if (nv > 1.0f) nv = 1.0f;
        int lvl   = lut_index(nv);

        int cp = (hist[idx] < EXPL_THRESH * 0.1f) ? CP_STABLE
               : (hist[idx] < EXPL_THRESH * 0.5f) ? CP_MARGINAL
               :                                     CP_UNSTABLE;
        wattron(w, COLOR_PAIR(cp));
        mvwaddch(w, oy, ox + i, k_ramp[lvl]);
        wattroff(w, COLOR_PAIR(cp));
    }
}

/*
 * render_dashboard() — full-width 8-row panel at the bottom of the screen.
 *
 * Row layout:
 *   0: separator + title
 *   1: CFL value (large, colour-coded) + dt + c + growth info
 *   2: full-width CFL meter bar
 *   3: scale labels (0.0 · stable · 0.7 · marginal · 1.0 · unstable · 1.5)
 *   4: dt slider (log-scale) + β value + dt_crit
 *   5: amplitude sparkline spanning full width
 *   6: stats (max_amp, ticks, explosions)
 *   7: key hints
 *
 * This layout occupies the entire terminal width — nothing like the
 * 36-column corner panel in membrane.c or shallow_water_solver.c.
 */
static void render_dashboard(WINDOW *w, int cols, int total_rows,
                             float scene_dt, float wave_speed, float cfl,
                             const InstabilityInfo *info, float max_amp,
                             long tick_count, int explosion_count, bool paused,
                             const float *amp_hist, int hist_head,
                             int blink_frame)
{
    int dy      = total_rows - DASH_H;
    int bar_w   = cols - 2;
    if (dy < 0 || bar_w < 10) return;

    int  cc       = cfl_color(cfl);
    bool blink_on = (blink_frame / 8) % 2 == 0;

    /* ── Row 0: separator ───────────────────────────────────────── */
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    for (int c = 0; c < cols; c++) mvwaddch(w, dy, c, '-');
    const char *title = "[ CFL STABILITY EXPLORER ]";
    mvwprintw(w, dy, (cols - (int)strlen(title)) / 2, "%s", title);
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);

    /* ── Row 1: CFL value + params + growth info ─────────────────── */
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(w, dy+1, 1, " CFL = ");
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
    wattron(w, COLOR_PAIR(cc) | A_BOLD);
    wprintw(w, "%6.4f  %-8s", cfl, cfl_label(cfl));
    wattroff(w, COLOR_PAIR(cc) | A_BOLD);
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    wprintw(w, "  dt=%8.5fs  c=%5.1f  ", scene_dt, wave_speed);
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
    if (info->growth_per_step > 1.0001f) {
        wattron(w, COLOR_PAIR(CP_UNSTABLE) | A_BOLD);
        if (info->time_to_double < 60.0f)
            wprintw(w, "|μ|=%6.4fx/step  t×2=%.3fs", info->growth_per_step,
                    info->time_to_double);
        else
            wprintw(w, "|μ|=%6.4fx/step  t×2=>60s ", info->growth_per_step);
        wattroff(w, COLOR_PAIR(CP_UNSTABLE) | A_BOLD);
    } else {
        wattron(w, COLOR_PAIR(CP_STABLE) | A_BOLD);
        wprintw(w, "|μ|=1.0000  amplitude bounded (neutral)");
        wattroff(w, COLOR_PAIR(CP_STABLE) | A_BOLD);
    }
    if (paused) {
        wattron(w, COLOR_PAIR(CP_MARGINAL) | A_BOLD);
        mvwprintw(w, dy+1, cols - 9, " PAUSED ");
        wattroff(w, COLOR_PAIR(CP_MARGINAL) | A_BOLD);
    }

    /* ── Row 2: full-width CFL meter ─────────────────────────────── */
    render_wide_cfl_meter(w, dy+2, 1, bar_w, cfl, blink_frame);

    /* ── Row 3: scale labels under the meter ─────────────────────── */
    {
        const float cfl_max = 1.5f;
        wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
        /* position labels proportional to bar_w */
        int p00  = 1 + (int)(0.00f / cfl_max * bar_w);
        int p05  = 1 + (int)(0.50f / cfl_max * bar_w) - 3;
        int p07  = 1 + (int)(0.70f / cfl_max * bar_w) - 3;
        int p10  = 1 + (int)(1.00f / cfl_max * bar_w) - 3;
        int p15  = 1 + (int)(1.50f / cfl_max * bar_w) - 3;
        if (p00 >= 0 && p00 < cols) mvwprintw(w, dy+3, p00, "0.0");
        if (p05 >= 0 && p05 < cols) mvwprintw(w, dy+3, p05, "0.5");
        if (p07 >= 0 && p07 < cols) {
            wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
            wattron(w, COLOR_PAIR(CP_MARGINAL));
            mvwprintw(w, dy+3, p07, "0.7");
            wattroff(w, COLOR_PAIR(CP_MARGINAL));
            wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
        }
        if (p10 >= 0 && p10 < cols) {
            wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
            if (blink_on && cfl >= 1.0f)
                wattron(w, COLOR_PAIR(CP_WARN) | A_BOLD);
            else
                wattron(w, COLOR_PAIR(CP_UNSTABLE) | A_BOLD);
            mvwprintw(w, dy+3, p10, "1.0");
            wattroff(w, A_BOLD | COLOR_PAIR(CP_UNSTABLE) | COLOR_PAIR(CP_WARN));
            wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
        }
        if (p15 >= 0 && p15 < cols) mvwprintw(w, dy+3, p15, "1.5");
        /* zone labels */
        int mid_stable   = 1 + (int)(0.35f / cfl_max * bar_w) - 4;
        int mid_marginal = 1 + (int)(0.85f / cfl_max * bar_w) - 4;
        int mid_unstable = 1 + (int)(1.25f / cfl_max * bar_w) - 4;
        if (mid_stable >= 0 && mid_stable + 8 < cols)
            mvwprintw(w, dy+3, mid_stable, "─STABLE─");
        if (mid_marginal >= 0 && mid_marginal + 8 < cols)
            mvwprintw(w, dy+3, mid_marginal, "MARGINAL");
        if (mid_unstable >= 0 && mid_unstable + 8 < cols)
            mvwprintw(w, dy+3, mid_unstable, "UNSTABLE");
        wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
    }

    /* ── Row 4: dt log-scale slider + β ─────────────────────────── */
    {
        float log_min  = logf(DT_MIN);
        float log_max  = logf(DT_MAX);
        float log_cur  = logf(scene_dt);
        float dt_crit  = info->critical_dt;
        float log_crit = logf(dt_crit < DT_MIN ? DT_MIN
                             : dt_crit > DT_MAX ? DT_MAX : dt_crit);
        float pos_cur  = (log_cur  - log_min) / (log_max - log_min);
        float pos_crit = (log_crit - log_min) / (log_max - log_min);
        int   slider_w = bar_w - 14;   /* leave room for labels */
        int   cur_i    = (int)(pos_cur  * (slider_w - 1));
        int   crit_i   = (int)(pos_crit * (slider_w - 1));
        if (cur_i  < 0) cur_i  = 0;
        if (cur_i  >= slider_w) cur_i  = slider_w - 1;
        if (crit_i < 0) crit_i = 0;
        if (crit_i >= slider_w) crit_i = slider_w - 1;
        int cp_cur = (scene_dt <= dt_crit) ? CP_STABLE : CP_UNSTABLE;

        wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
        mvwprintw(w, dy+4, 1, "dt[");
        for (int i = 0; i < slider_w; i++) {
            char ch = '-';
            if (i == crit_i) ch = '|';
            if (i == cur_i)  ch = 'O';
            attr_t a = (i == cur_i) ? (COLOR_PAIR(cp_cur) | A_BOLD)
                                    : (COLOR_PAIR(CP_HUD) | A_DIM);
            wattron(w, a);
            mvwaddch(w, dy+4, 4 + i, ch);
            wattroff(w, a);
        }
        wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
        wprintw(w, "]");
        wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
        wattron(w, COLOR_PAIR(cc) | A_BOLD);
        wprintw(w, " β=%6.3f", info->beta);
        wattroff(w, COLOR_PAIR(cc) | A_BOLD);
        wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
        wprintw(w, " dt_crit=%8.5fs  (+/-:adjust  1-6:jump)", dt_crit);
        wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
    }

    /* ── Row 5: full-width sparkline ─────────────────────────────── */
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(w, dy+5, 0, "t>");
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
    render_wide_sparkline(w, dy+5, 2, bar_w, amp_hist, hist_head, AMP_HIST_LEN);

    /* ── Row 6: stats ────────────────────────────────────────────── */
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(w, dy+6, 1,
              " max_amp=%10.4f  ticks=%8ld  exploded=%3d times  "
              "c·dt=%7.5f  dt_crit=%7.5f",
              max_amp, tick_count, explosion_count,
              wave_speed * scene_dt, info->critical_dt);
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);

    /* ── Row 7: key hints ────────────────────────────────────────── */
    attron(A_DIM);
    mvwprintw(w, dy+7, 0,
              " q:quit  spc:pause  s:step  +/-:dt±5%%  c/C:speed  "
              "1-6:CFL preset  m:eigenmode  d:drop  r:reset  t:theme ");
    attroff(A_DIM);
}


/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct Scene Scene;

struct Scene {
    /* THE key parameters */
    float   scene_dt;          /* physics timestep; user adjusts with +/- */
    float   wave_speed;        /* c (cells/s); adjustable with c/C        */

    /* derived (recomputed each tick) */
    float           cfl;
    InstabilityInfo instab;
    float           max_amplitude;

    /* amplitude history ring buffer for sparkline */
    float   amp_hist[AMP_HIST_LEN];
    int     amp_hist_head;    /* oldest sample position */

    /* simulation state */
    bool    paused;
    bool    step_requested;
    int     theme;
    bool    resonance_mode;   /* true = eigenmode init, false = Gaussian  */

    /* explosion tracking */
    long    tick_count;
    int     explosion_count;

    /* grid dimensions */
    int     cols;
    int     rows;
};

static void scene_reset(Scene *s)
{
    grid_zero(s->cols, s->rows);
    if (s->resonance_mode)
        apply_resonance_mode(1, 1, RESONANCE_AMP, s->cols, s->rows);
    else
        apply_drop((float)(s->cols-1)*0.5f, (float)(s->rows-1)*0.5f,
                   DROP_AMP, DROP_RADIUS, s->cols, s->rows);
    s->tick_count     = 0;
    s->max_amplitude  = 0.0f;
    memset(s->amp_hist, 0, sizeof s->amp_hist);
    s->amp_hist_head  = 0;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->scene_dt    = DT_DEFAULT;
    s->wave_speed  = WAVE_SPEED_DEFAULT;
    s->theme       = 0;
    s->cols        = cols;
    s->rows        = rows;
    scene_reset(s);
}

/*
 * scene_set_cfl() — jump to a target CFL value by adjusting scene_dt.
 *
 * Useful for preset keys (1-6): user presses a key, gets immediately placed
 * in a specific stability regime without hand-tuning dt.
 * dt = CFL_target / (c · √2).
 */
static void scene_set_cfl(Scene *s, float target_cfl)
{
    float dt = target_cfl / (s->wave_speed * (float)M_SQRT2);
    if (dt < DT_MIN) dt = DT_MIN;
    if (dt > DT_MAX) dt = DT_MAX;
    s->scene_dt = dt;
    scene_reset(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    scene_reset(s);
}

static void scene_tick(Scene *s)
{
    if (s->paused && !s->step_requested) return;
    s->step_requested = false;
    s->tick_count++;

    update_wave(s->scene_dt, s->wave_speed, s->cols, s->rows);

    s->max_amplitude = compute_max_amplitude(s->cols, s->rows);

    /* Write to sparkline ring buffer (one sample per tick) */
    s->amp_hist[s->amp_hist_head] = s->max_amplitude;
    s->amp_hist_head = (s->amp_hist_head + 1) % AMP_HIST_LEN;

    /* Auto-reset on explosion — keep parameters but restart field */
    if (s->max_amplitude > EXPL_THRESH) {
        s->explosion_count++;
        scene_reset(s);
    }

    s->cfl    = compute_CFL(s->wave_speed, s->scene_dt);
    s->instab = detect_instability(s->wave_speed, s->scene_dt);
}

static void scene_draw(const Scene *s, WINDOW *w, int blink_frame)
{
    int field_rows = s->rows - DASH_H;
    if (field_rows < 1) field_rows = 1;

    render_field(w, s->cols, field_rows, s->theme,
                 s->cfl, s->max_amplitude, blink_frame);

    render_dashboard(w, s->cols, s->rows,
                     s->scene_dt, s->wave_speed,
                     s->cfl, &s->instab,
                     s->max_amplitude, s->tick_count,
                     s->explosion_count, s->paused,
                     s->amp_hist, s->amp_hist_head,
                     blink_frame);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols; int rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int blink_frame)
{
    (void)sc;
    erase();
    scene_draw(s, stdscr, blink_frame);

    /* fps counter — top-left, small, doesn't overlap dashboard */
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(0, 0, " %.0f fps ", fps);
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                  scene;
    Screen                 screen;
    volatile sig_atomic_t  running;
    volatile sig_atomic_t  need_resize;
    int                    blink_frame;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_handle_key():
 *
 *   +/-     adjust dt by ±DT_MULT (5%) — the primary CFL control.
 *           Holding '+' for a few seconds takes you from CFL=0.5 to CFL=1.5.
 *
 *   c/C     change wave_speed — affects CFL without changing dt.
 *           Demonstrates that instability comes from their PRODUCT, not
 *           either parameter alone.
 *
 *   1-6     jump to preset CFL values:
 *             1=0.30 (well stable), 2=0.60 (stable), 3=0.85 (marginal),
 *             4=0.98 (near-critical), 5=1.05 (just unstable), 6=1.20 (explosive)
 *           Resets the wave field so you see the growth from the beginning.
 *
 *   m       toggle resonance mode — single (1,1) eigenmode vs Gaussian pulse.
 *           In eigenmode, the sparkline shows the pure exponential growth
 *           of that one mode — growth_per_step is directly measurable.
 *
 *   d       add an extra Gaussian drop at the centre (additive).
 */
static bool app_handle_key(App *app, int ch)
{
    Scene  *s  = &app->scene;
    Screen *sc = &app->screen;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 's': case 'S':
        s->paused = true;
        s->step_requested = true;
        break;

    /* ── dt adjustment — the primary stability control ──────────── *
     * Each '+' press multiplies dt by DT_MULT (5%).
     * Starting at CFL=0.825 (default), reaching CFL=1.0 takes about
     * 4 presses (+); going back takes the same number of '-' presses.
     * Notice: the closer to CFL=1, the more dramatic each step is.  */
    case '+': case '=':
        s->scene_dt *= DT_MULT;
        if (s->scene_dt > DT_MAX) s->scene_dt = DT_MAX;
        break;

    case '-': case '_':
        s->scene_dt /= DT_MULT;
        if (s->scene_dt < DT_MIN) s->scene_dt = DT_MIN;
        break;

    /* ── wave speed — affects CFL multiplicatively with dt ─────── */
    case 'c':
        s->wave_speed += WAVE_SPEED_STEP;
        if (s->wave_speed > WAVE_SPEED_MAX) s->wave_speed = WAVE_SPEED_MAX;
        break;
    case 'C':
        s->wave_speed -= WAVE_SPEED_STEP;
        if (s->wave_speed < WAVE_SPEED_MIN) s->wave_speed = WAVE_SPEED_MIN;
        break;

    /* ── CFL preset keys: all stay within stable/marginal zone ──── *
     * Press '6' to reach CFL=0.99, then '+' once or twice to cross
     * CFL=1.0 and watch the explosion unfold in real time.           */
    /* All 6 presets stay within the stable/marginal zone.
     * Push into instability manually with '+' (takes ~3 presses from '6').
     *   1: CFL=0.25 — deep stable, very smooth waves, large CFL margin
     *   2: CFL=0.50 — comfortable stable
     *   3: CFL=0.70 — exactly at the STABLE/MARGINAL boundary
     *   4: CFL=0.85 — marginal, field turns amber
     *   5: CFL=0.95 — near-critical, most energetic stable waves
     *   6: CFL=0.99 — one '+' press from instability; use to watch the
     *                  transition happen in real time                    */
    case '1': scene_set_cfl(s, 0.25f); break;
    case '2': scene_set_cfl(s, 0.50f); break;
    case '3': scene_set_cfl(s, 0.70f); break;
    case '4': scene_set_cfl(s, 0.85f); break;
    case '5': scene_set_cfl(s, 0.95f); break;
    case '6': scene_set_cfl(s, 0.99f); break;

    /* ── eigenmode toggle ───────────────────────────────────────── */
    case 'm': case 'M':
        s->resonance_mode = !s->resonance_mode;
        scene_reset(s);
        break;

    /* ── additive drop ──────────────────────────────────────────── */
    case 'd': case 'D':
        apply_drop((float)(sc->cols-1)*0.5f, (float)(sc->rows-1)*0.5f,
                   DROP_AMP, DROP_RADIUS, sc->cols, sc->rows);
        break;

    case 'r': case 'R':
        scene_reset(s);
        break;

    case 't': case 'T':
        s->theme = (s->theme + 1) % N_THEMES;
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

    Screen *sc = &app->screen;
    Scene  *s  = &app->scene;

    screen_init(sc);
    scene_init(s, sc->cols, sc->rows);

    /* FPS measurement */
    int64_t fps_window_start = clock_ns();
    int     fps_frame_count  = 0;
    double  fps_display      = 0.0;

    while (app->running) {

        if (app->need_resize) {
            screen_resize(sc);
            scene_resize(s, sc->cols, sc->rows);
            app->need_resize = 0;
        }

        /* ── input ──────────────────────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR)
            if (!app_handle_key(app, ch)) { app->running = 0; break; }
        if (!app->running) break;

        /* ── one physics step per render frame ──────────────────── *
         * Decoupling dt from the render rate is the core design of
         * this explorer.  render rate = TARGET_FPS (fixed at 60 Hz).
         * physics dt   = scene_dt (user-controlled).
         * A large dt means larger CFL — potentially unstable.        */
        int64_t frame_start = clock_ns();
        scene_tick(s);
        app->blink_frame++;

        /* ── FPS ────────────────────────────────────────────────── */
        fps_frame_count++;
        int64_t now = clock_ns();
        int64_t fps_elapsed = now - fps_window_start;
        if (fps_elapsed >= (int64_t)FPS_UPDATE_MS * NS_PER_MS) {
            fps_display      = (double)fps_frame_count /
                               ((double)fps_elapsed / (double)NS_PER_SEC);
            fps_frame_count  = 0;
            fps_window_start = now;
        }

        /* ── render ─────────────────────────────────────────────── */
        screen_draw(sc, s, fps_display, app->blink_frame);
        screen_present();

        /* ── sleep to maintain TARGET_FPS render rate ───────────── */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);
    }

    screen_free(sc);
    return 0;
}
