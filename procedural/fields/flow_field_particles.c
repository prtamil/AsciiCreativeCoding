/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * flow_field_particles.c
 *   — Particles streaming through closed-form 2-D vector fields.
 *
 * DEMO: 256 particles flow through one of five algebraically-defined
 *       2-D vector fields. Unlike the noise-based showcases in this
 *       folder, every field here is a CLOSED-FORM expression — pure
 *       (x, y, t) → (vx, vy) math, no Perlin / Worley / etc. The five
 *       patterns are the canonical examples from any vector-field
 *       textbook:
 *         VORTICES  — 4 rotating cyclones drifting around the map
 *         WAVE      — sinusoidal flow (sin(ωy+t), cos(ωx−t))
 *         SADDLE    — hyperbolic stagnation point — diverge in x,
 *                     converge in y, like a 2-D saddle
 *         MAGNET    — attractor/repeller pair — particles flow from
 *                     repeller to attractor along curved field lines
 *         TURBULENT — 20 random small vortices summed — chaotic
 *                     small-scale flow
 *       Vortex centres / magnet poles are drawn as bright '@' glyphs
 *       on top of the particle trails so you can see the field's
 *       singularities. Themes shape colour; n/p cycles patterns.
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — same particle-flow architecture,
 *       but the field source is Perlin noise. Compare the streamline
 *       SHAPES: noise gives organic eddies, this file gives clean
 *       textbook fields.
 *   ./curl_noise_vector_field.c — divergence-free curl-of-noise
 *       fields. Particles never converge there; here, MAGNET and
 *       SADDLE explicitly DO converge / diverge.
 *
 * Section map:
 *   §1 config   — grid, particles, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 fields   — 5 closed-form (vx, vy) functions
 *   §6 attractors — vortex / magnet / turbulence centre management
 *   §7 scene    — Field, Particle, scene state, per-frame update
 *   §8 screen   — ASCII render: density glyphs + attractor markers
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (re-seed particles, re-randomise field state)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   g / G      next / previous glyph set (slim → fat)
 *   + / =      faster particles
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flow_field_particles.c \
 *       -o flow_field -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle advection through a time-varying 2-D vector
 *                  field. For each particle, look up (vx, vy) at its
 *                  current position, step by (vx, vy) · dt, paint a
 *                  trail. Five patterns expose five different field
 *                  generators; the particle simulation is identical
 *                  across all five.
 *
 *                  Field generators (closed-form, no noise):
 *                    VORTICES  — sum of N rotational fields:
 *                                v_i = (−Δy, Δx) · S / r²
 *                                where (Δx, Δy) is offset from
 *                                vortex i's centre, r² is its squared
 *                                distance, S is strength.
 *                    WAVE      — vx = sin(ωy + t), vy = cos(ωx − t).
 *                    SADDLE    — v = (x − cx, −(y − cy)) · α.
 *                                Hyperbolic stagnation at (cx, cy).
 *                    MAGNET    — sum of inverse-square gravity-style
 *                                terms:
 *                                v_i = −sign · (Δx, Δy) / r²
 *                                where sign = +1 attracts, −1 repels.
 *                    TURBULENT — sum of many small random-sign mini-
 *                                vortices.
 *
 *                  Each frame the attractor/vortex POSITIONS drift
 *                  slowly (orbit around their nominal centres) so the
 *                  flow evolves without resetting.
 *
 * Data-structure : Particles array (256), attractor array (16), per-cell
 *                  trail glow + colour buffers. No allocation post-init.
 *
 * Rendering      : ASCII only. Density-graded '.', '*', '#' for
 *                  particle trails in 4 theme colours. Bright '@' over
 *                  attractor positions (vortex centres, magnet poles,
 *                  saddle centres) so the field's singularities are
 *                  visible against the streamline pattern.
 *
 * Performance    : 5–20 attractor evaluations per particle per frame
 *                  (TURBULENT is the heaviest at 20). With 256 particles
 *                  at 60 Hz that's ~300 K function calls/sec — trivial.
 *                  Closed-form math is faster than noise-based fields
 *                  by an order of magnitude.
 *
 * References     : • Strogatz, S. — "Nonlinear Dynamics and Chaos",
 *                    chapters on phase plane analysis and 2-D vector
 *                    fields. The mathematical taxonomy used here
 *                    (vortex / saddle / source / sink) comes from
 *                    dynamical systems theory.
 *                  • Inigo Quilez — "Useful little functions" gives
 *                    practical stable forms for the inverse-square
 *                    attractor with a softening epsilon:
 *                    https://iquilezles.org/articles/distfunctions2d/
 *                  • Compare ./perin_noise_flow_showcase.c (noise
 *                    field) and ./curl_noise_vector_field.c (curl
 *                    field) — same particle simulation, different
 *                    field sources.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A 2-D vector field assigns a velocity to every point. Drop particles,
 * read off their local velocity, step them forward. The choice of
 * field determines the streamlines: rotational fields make swirls,
 * radial fields make spokes, periodic fields make waves, mixed fields
 * make turbulent eddies. All five patterns here are pure math —
 * no random component except the seeding of vortex positions.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the screen as a windy plain. The "wind" pattern is what we
 * draw. Drop dust on it and watch the dust trace the wind. Each
 * pattern is a different wind:
 *   VORTICES  — four hurricanes spinning at the corners.
 *   WAVE      — wind blowing in undulating waves like ocean swell.
 *   SADDLE    — wind blowing OUT along x and IN along y (rare in
 *               nature, common in dynamical systems analysis).
 *   MAGNET    — opposing magnetic poles; particles flow from the
 *               repeller to the attractor along curved field lines.
 *   TURBULENT — many small whirlwinds chaotically interacting.
 *
 * Each pattern has its own SIGNATURE streamlines. With practice you
 * can identify the field type at a glance from the particle trails.
 *
 * Visible layers:
 *   1. PARTICLE TRAILS — fading paths (theme-coloured) showing where
 *      particles have flowed. Density encodes how often particles
 *      cross each cell.
 *   2. ATTRACTORS '@' — bright glyphs marking vortex centres or
 *      magnet poles. Visible over the trails so you can see exactly
 *      which singularities are driving the flow.
 *   3. SLOW DRIFT — vortex / magnet positions orbit around their
 *      nominal centres at ~0.2 rad/s, so the flow evolves rather
 *      than freezing.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Spawn N particles uniformly in-bounds. Initialise the
 *     pattern's attractors (4 vortex centres, 2 magnet poles, etc.).
 *  2. PER FRAME:
 *     a. Update attractor positions: each one orbits slightly around
 *        its base position to keep the flow alive.
 *     b. For each particle:
 *        i.   Look up (vx, vy) using the active pattern's field
 *             function at the particle's position.
 *        ii.  Step by (vx, vy) · speed · dt.
 *        iii. Increment age. If age ≥ max_age OR position out of
 *             bounds, respawn at a random in-bounds cell.
 *        iv.  Paint trail at the new cell.
 *     c. Render trails (density glyphs) + attractors ('@') on top.
 *  3. Periodic supernova reset every ~12 s for variety.
 *
 * KEY FORMULAS
 * ────────────
 *  Vortex contribution at (x, y) :
 *    v_i = (−(y − ay), (x − ax)) · S / max(r², ε)
 *    where (ax, ay) is vortex centre, r² = (x−ax)² + (y−ay)², S the
 *    strength, ε the softening to avoid singularity.
 *  Wave field                    :
 *    vx = sin(ωy + t),  vy = cos(ωx − t)
 *  Saddle field                  :
 *    vx = α · (x − cx),  vy = −α · (y − cy)
 *  Magnet (gravity-style)        :
 *    v_i = −sign · ((x − ax), (y − ay)) / max(r², ε)
 *    sign = +1 attracts, −1 repels.
 *  Total field                   : v(x, y) = Σᵢ v_i (for multi-source)
 *  Step                          : (Δx, Δy) = v · speed · dt
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SOFTENING EPSILON. Without ε in the divisor, particles passing
 *    through a vortex centre or magnet pole hit infinity and warp
 *    across the map. Use r² + ε with ε ≥ 1 cell² to keep the field
 *    finite everywhere.
 *
 *  • NORMALISATION. Raw vortex/magnet output magnitudes vary wildly
 *    (huge near the centre, tiny at distance). Normalise to unit
 *    direction × constant speed so particles move at consistent
 *    visual speed regardless of where they are.
 *
 *  • PARTICLE RESPAWN. SADDLE and MAGNET patterns have explicit
 *    sources / sinks: particles will pile up into the attractor or
 *    fly off the edge from the repeller / saddle. Without respawn
 *    on age + OOB, the flow goes dead in seconds.
 *
 *  • BOUNDARY BEHAVIOUR. Particles hit map edges and exit; we OOB-
 *    respawn. Wrap-around (toroidal) would also work but breaks the
 *    visual identity of bounded fields like SADDLE.
 *
 *  • PATTERN-DEPENDENT BACKGROUND BIAS. SADDLE drives particles
 *    AWAY from the centre on average; without random respawning,
 *    everything converges to one quadrant. The respawn keeps the
 *    coverage uniform.
 *
 *  • TURBULENT SCALING. With 20 mini-vortices, total field strength
 *    can far exceed any single one. We pre-scale individual
 *    strengths down to keep total motion sane.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • VORTICES: trails clearly curve around 4 visible '@' centres.
 *    If trails are straight, the strength is too low. If they're
 *    near-singularities, the softening is too small.
 *  • WAVE: streamlines look like ocean swell — sinusoidal undulations
 *    along both axes. If you see straight motion, the sin/cos is
 *    being evaluated wrong.
 *  • SADDLE: clear quadrupole structure — particles flow OUT
 *    horizontally from the centre and IN vertically. The four
 *    diagonal "exit corridors" are diagnostic.
 *  • MAGNET: '@'s visibly represent two poles; particles flow
 *    consistently from one to the other along curving paths.
 *  • TURBULENT: small-scale eddies everywhere; no single dominant
 *    structure. If you see one big swirl, the mini-vortex count
 *    is too low.
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

    /* Particle pool. */
    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    /* Particle lifetime (ticks). Different per particle to break the
     * lockstep "everyone respawns at the same time" effect. */
    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    /* Speed: cells per second. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    /* Periodic full-reset cadence. */
    RESET_TICKS_DEF   = 12 * 60,

    /* Maximum simultaneous attractors (max needed: TURBULENT = 20). */
    MAX_ATTRACTORS    =  32,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,    /* PAIR_TRAIL_BASE..+3 = 4 trail colours */
    PAIR_ATTRACT      =   7,    /* '@' attractor markers                  */
    PAIR_SUPERNOVA    =   8,
};

#define TRAIL_GLOW_DECAY    0.6f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define DRIFT_RATE          0.2f    /* attractor orbital rate (rad/s) */

/* Field strength scales — tuned so particle motion is consistent
 * across all patterns at SPEED_DEF. */
#define VORTEX_STRENGTH     80.0f
#define MAGNET_STRENGTH     50.0f
#define TURBULENT_STRENGTH  6.0f
#define WAVE_FREQ           0.15f
#define SADDLE_RATE         0.05f
#define EPS_R2              4.0f    /* softening: avoid divide-by-zero at attractor centres */

#define HOLD_SECONDS        2.5f

#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern — five algebraic vector fields. Cycle with n/p.
 *
 *   VORTICES  : 4 rotating cyclones
 *   WAVE      : sinusoidal field
 *   SADDLE    : single hyperbolic stagnation point
 *   MAGNET    : attractor/repeller pair
 *   TURBULENT : 20 random-sign mini vortices
 */
typedef enum {
    PATTERN_VORTICES  = 0,
    PATTERN_WAVE      = 1,
    PATTERN_SADDLE    = 2,
    PATTERN_MAGNET    = 3,
    PATTERN_TURBULENT = 4,
    N_PATTERNS        = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_VORTICES:  return "VORTICES ";
    case PATTERN_WAVE:      return "WAVE     ";
    case PATTERN_SADDLE:    return "SADDLE   ";
    case PATTERN_MAGNET:    return "MAGNET   ";
    case PATTERN_TURBULENT: return "TURBULENT";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names. Each defines 4 trail colours and a flash
 * accent (used for attractor markers in this file).
 */
typedef struct {
    const char *name;
    short       trail[4];
    short       flash;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      trail0 trail1 trail2 trail3 flash */
    { "DEFAULT", {  33,  117,  220,  220 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 240,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  88,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
};

/*
 * GlyphSet — three characters representing low / mid / high trail
 * density, ordered in five thickness bands (SLIM → FAT). Cycle with
 * g/G. Useful when the default '.' '*' '#' triad doesn't suit the
 * current theme (e.g. MONO looks better with the fatter HEAVY set;
 * MATRIX looks more "wireframe" with SLIM).
 *
 * The ramp goes from thinnest visible mark to densest filled mark.
 * Each set's three glyphs are visually consistent (e.g. HEAVY is
 * round-on-round-on-round; FAT is angular-on-angular).
 */
typedef struct {
    const char *name;
    char low, mid, high;
} GlyphSet;

#define N_GLYPH_SETS 5

static const GlyphSet glyph_sets[N_GLYPH_SETS] = {
    /*  name      low  mid  high   visual progression       */
    { "SLIM",    '.', '\'', ':' },   /* thinnest, sparse     */
    { "LIGHT",   '.', '*',  '+' },   /* small but visible    */
    { "MEDIUM",  '.', '*',  '#' },   /* standard (default)   */
    { "HEAVY",   'o', 'O',  '@' },   /* round, dense         */
    { "FAT",     '+', '#',  'M' },   /* maximum coverage     */
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
            init_pair(PAIR_TRAIL_BASE + i, t->trail[i], -1);
        init_pair(PAIR_ATTRACT, t->flash, -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
        init_pair(PAIR_ATTRACT, COLOR_YELLOW, -1);
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
/* §5  fields — 5 closed-form vector fields                               */
/* ===================================================================== */

/*
 * Attractor — one source/sink/vortex centre.
 *
 *   bx, by    : nominal "base" position (around which it orbits)
 *   ox, oy    : orbital radius components — the actual position is
 *               (bx + ox · cos(t · DRIFT_RATE + φ),
 *                by + oy · sin(t · DRIFT_RATE + φ))
 *   phase     : per-attractor phase offset so they drift independently
 *   strength  : multiplier
 *   sign      : +1 for attractor / CCW vortex, −1 for repeller / CW
 *
 * The "live" position is computed each frame from base + drift; we
 * don't store it.
 */
typedef struct {
    float bx, by;
    float ox, oy;
    float phase;
    float strength;
    int   sign;
} Attractor;

/*
 * Active attractor positions (computed once per frame from base+drift).
 */
typedef struct {
    float x, y;
    float strength;
    int   sign;
} ActiveAttractor;

/*
 * Forward declarations — the field functions need access to the
 * active attractor list which lives in §7's Field struct. We pass it
 * as a parameter rather than using globals.
 */

/*
 * field_vortices — sum of N vortex contributions at (x, y, t). Each
 * vortex contributes a tangential velocity proportional to 1/r²
 * around its centre.
 */
static void field_vortices(const ActiveAttractor *att, int n_att,
                           float x, float y,
                           float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/*
 * field_wave — sinusoidal flow. Streamlines are wavy and oscillate
 * with time.
 */
static void field_wave(float x, float y, float t,
                       float *out_vx, float *out_vy)
{
    *out_vx = sinf(y * WAVE_FREQ + t       ) * 5.0f;
    *out_vy = cosf(x * WAVE_FREQ - t * 0.7f) * 5.0f;
}

/*
 * field_saddle — hyperbolic stagnation point. Particles diverge
 * along x and converge along y. The classic "saddle" example from
 * 2-D dynamical systems analysis.
 */
static void field_saddle(float x, float y, float cx, float cy,
                         float *out_vx, float *out_vy)
{
    *out_vx =  (x - cx) * SADDLE_RATE;
    *out_vy = -(y - cy) * SADDLE_RATE;
}

/*
 * field_magnet — gravitational dipole. Sum of inverse-square
 * attractions/repulsions toward each pole.
 */
static void field_magnet(const ActiveAttractor *att, int n_att,
                         float x, float y,
                         float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = (float)(-att[i].sign) * att[i].strength / r2;
        vx += dx * w;
        vy += dy * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/*
 * field_turbulent — sum of 20 small vortex contributions, half CCW
 * and half CW. Net field is chaotic at small scales but locally
 * coherent — a decent approximation to fluid turbulence.
 */
static void field_turbulent(const ActiveAttractor *att, int n_att,
                            float x, float y,
                            float *out_vx, float *out_vy)
{
    /* Same shape as field_vortices but with smaller per-vortex
     * strengths. (n_att is set to 20 for this pattern.) */
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* ===================================================================== */
/* §6  attractors — pattern-specific source / sink configuration         */
/* ===================================================================== */

/*
 * pattern_init_attractors — populate Field's attractors[] for a given
 * pattern. Called once per pattern switch (and at scene reset).
 *
 * Different patterns need different attractor counts and signs:
 *   VORTICES  : 4 attractors at the corners of an inner square,
 *               all sign=+1, strength VORTEX_STRENGTH.
 *   WAVE      : 0 attractors (closed-form math, no centres).
 *   SADDLE    : 0 attractors (the centre is implicit at map midpoint).
 *   MAGNET    : 2 attractors, opposite signs.
 *   TURBULENT : 20 random attractors, mixed signs, lower strength.
 *
 * The map dims (w, h) are needed to place attractors in-bounds.
 */
static int pattern_init_attractors(Attractor out[], int max_n,
                                   Pattern p, int w, int h)
{
    int n = 0;
    switch (p) {

    case PATTERN_VORTICES: {
        float cx = (float)w * 0.5f;
        float cy = (float)h * 0.5f;
        float rx = (float)w * 0.25f;
        float ry = (float)h * 0.25f;
        for (int i = 0; i < 4 && n < max_n; i++) {
            float ang = (float)i * (float)M_PI * 0.5f;
            out[n++] = (Attractor){
                .bx = cx + rx * cosf(ang),
                .by = cy + ry * sinf(ang),
                .ox = 4.0f, .oy = 4.0f,
                .phase = (float)i * 0.7f,
                .strength = VORTEX_STRENGTH,
                .sign = (i & 1) ? -1 : +1,    /* alternate spin */
            };
        }
        break;
    }

    case PATTERN_MAGNET: {
        float cx = (float)w * 0.5f;
        float cy = (float)h * 0.5f;
        float dx = (float)w * 0.20f;
        out[n++] = (Attractor){
            .bx = cx - dx, .by = cy,
            .ox = 3.0f, .oy = 2.0f, .phase = 0.0f,
            .strength = MAGNET_STRENGTH,
            .sign = +1,                       /* attractor */
        };
        out[n++] = (Attractor){
            .bx = cx + dx, .by = cy,
            .ox = 3.0f, .oy = 2.0f, .phase = (float)M_PI,
            .strength = MAGNET_STRENGTH,
            .sign = -1,                       /* repeller */
        };
        break;
    }

    case PATTERN_TURBULENT: {
        for (int i = 0; i < 20 && n < max_n; i++) {
            out[n++] = (Attractor){
                .bx = (float)(rand() % w),
                .by = (float)(rand() % h),
                .ox = 2.0f, .oy = 2.0f,
                .phase = (float)i * 0.31f,
                .strength = TURBULENT_STRENGTH,
                .sign = (rand() & 1) ? +1 : -1,
            };
        }
        break;
    }

    default:
        /* WAVE and SADDLE have no explicit attractors. */
        break;
    }
    return n;
}

/*
 * compute_active_attractors — derive each frame's "live" attractor
 * positions from base + orbital drift.
 */
static int compute_active_attractors(const Attractor src[], int n_src,
                                     ActiveAttractor out[], float t)
{
    for (int i = 0; i < n_src; i++) {
        float ang = t * DRIFT_RATE + src[i].phase;
        out[i].x         = src[i].bx + src[i].ox * cosf(ang);
        out[i].y         = src[i].by + src[i].oy * sinf(ang);
        out[i].strength  = src[i].strength;
        out[i].sign      = src[i].sign;
    }
    return n_src;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    float x, y;
    int   color_idx;
    int   age;
    int   max_age;
} Particle;

typedef struct {
    int      w, h;
    int      total_cells;

    float    trail_glow [CELLS_MAX];
    uint8_t  trail_color[CELLS_MAX];
    float    supernova_glow_t;

    Particle particles[MAX_PARTICLES];
    int      n_particles;

    Attractor       attractors[MAX_ATTRACTORS];
    int             n_attractors;
    ActiveAttractor active_attr[MAX_ATTRACTORS];
    int             n_active_attr;

    float    field_time;
    int      reset_countdown;
} Field;

static inline int field_idx(const Field *f, int x, int y) { return y * f->w + x; }
static inline bool field_in_bounds(const Field *f, int x, int y)
{
    return x >= 0 && x < f->w && y >= 0 && y < f->h;
}

static void particle_spawn(Field *f, Particle *p)
{
    p->x         = (float)(rand() % f->w);
    p->y         = (float)(rand() % f->h);
    p->color_idx = rand() & 3;
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/*
 * field_velocity_at — dispatch on pattern, compute velocity at (x, y).
 * Reads from the per-frame ActiveAttractor list for VORTICES / MAGNET
 * / TURBULENT.
 */
static void field_velocity_at(const Field *f, Pattern pat,
                              float x, float y,
                              float *out_vx, float *out_vy)
{
    switch (pat) {
    case PATTERN_VORTICES:
        field_vortices(f->active_attr, f->n_active_attr, x, y, out_vx, out_vy);
        break;
    case PATTERN_WAVE:
        field_wave(x, y, f->field_time, out_vx, out_vy);
        break;
    case PATTERN_SADDLE: {
        float cx = (float)f->w * 0.5f;
        float cy = (float)f->h * 0.5f;
        field_saddle(x, y, cx, cy, out_vx, out_vy);
        break;
    }
    case PATTERN_MAGNET:
        field_magnet(f->active_attr, f->n_active_attr, x, y, out_vx, out_vy);
        break;
    case PATTERN_TURBULENT:
        field_turbulent(f->active_attr, f->n_active_attr, x, y, out_vx, out_vy);
        break;
    default:
        *out_vx = 0.0f;
        *out_vy = 0.0f;
        break;
    }
}

/*
 * particle_step — read the field at the particle's position, normalise
 * to a unit direction (to keep visual speed consistent regardless of
 * the field's magnitude variation), step by speed · dt. Respawn on
 * OOB or max_age.
 */
static void particle_step(Field *f, Pattern pat, Particle *p,
                          float dt, int speed)
{
    float vx, vy;
    field_velocity_at(f, pat, p->x, p->y, &vx, &vy);

    /* Normalise. Keeps motion controllable across patterns whose raw
     * magnitudes vary by orders of magnitude. */
    float m = sqrtf(vx * vx + vy * vy);
    if (m > 1e-6f) { vx /= m; vy /= m; }

    p->x += vx * (float)speed * dt;
    p->y += vy * (float)speed * dt;
    p->age++;

    int cx = (int)p->x;
    int cy = (int)p->y;
    if (field_in_bounds(f, cx, cy)) {
        int idx = field_idx(f, cx, cy);
        f->trail_glow[idx]  = 1.0f;
        f->trail_color[idx] = (uint8_t)p->color_idx;
    }

    if (p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)f->w
        || p->y < 0.0f || p->y >= (float)f->h) {
        particle_spawn(f, p);
    }
}

typedef struct {
    Field   F;
    bool    paused;
    int     speed;
    int     current_theme;
    int     current_glyph_set;
    Pattern current_pattern;
} Scene;

static void field_clear_trails(Field *f)
{
    for (int i = 0; i < f->total_cells; i++) {
        f->trail_glow[i]  = 0.0f;
        f->trail_color[i] = 0;
    }
}

static void field_reset(Field *f, int w, int h, Pattern pat)
{
    f->w = w;
    f->h = h;
    f->total_cells = w * h;
    f->n_particles = N_PARTICLES_DEF;
    f->field_time = 0.0f;
    f->reset_countdown = RESET_TICKS_DEF;
    f->supernova_glow_t = 1.0f;
    field_clear_trails(f);
    f->n_attractors = pattern_init_attractors(f->attractors, MAX_ATTRACTORS,
                                              pat, w, h);
    for (int i = 0; i < f->n_particles; i++) {
        particle_spawn(f, &f->particles[i]);
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    field_reset(&s->F, mw, mh, s->current_pattern);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused             = false;
    s->speed              = SPEED_DEF;
    s->current_theme      = 0;
    s->current_glyph_set  = 2;        /* MEDIUM — the historical default */
    s->current_pattern    = PATTERN_VORTICES;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;

    /* Decay trail glows + supernova. */
    float decay_t = expf(-TRAIL_GLOW_DECAY * dt);
    float decay_n = expf(-SUPERNOVA_DECAY  * dt);
    for (int i = 0; i < f->total_cells; i++) {
        f->trail_glow[i] *= decay_t;
    }
    f->supernova_glow_t *= decay_n;

    f->field_time += dt;

    /* Update attractor positions for this frame. */
    f->n_active_attr = compute_active_attractors(f->attractors, f->n_attractors,
                                                 f->active_attr, f->field_time);

    /* Step every particle. */
    for (int i = 0; i < f->n_particles; i++) {
        particle_step(f, s->current_pattern, &f->particles[i], dt, s->speed);
    }

    /* Periodic reset. */
    f->reset_countdown--;
    if (f->reset_countdown <= 0) {
        field_reset(f, f->w, f->h, s->current_pattern);
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

    /* Lookup the active glyph set once for the entire frame. */
    int gs_idx = s->current_glyph_set;
    if (gs_idx < 0 || gs_idx >= N_GLYPH_SETS) gs_idx = 0;
    const GlyphSet *gs = &glyph_sets[gs_idx];

    /* Pass 1 — particle trails. */
    for (int y = 0; y < f->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < f->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = field_idx(f, x, y);
            float ng = f->supernova_glow_t;
            float tg = f->trail_glow[idx];

            int  pair, attr;
            char glyph;

            if (ng > GLOW_THRESHOLD) {
                if (((x ^ y) & 3) != 0 && tg <= GLOW_THRESHOLD) continue;
                pair  = PAIR_SUPERNOVA;
                attr  = A_BOLD;
                glyph = '*';
            } else if (tg > GLYPH_HIGH_THRESH) {
                pair  = PAIR_TRAIL_BASE + (f->trail_color[idx] & 3);
                attr  = A_BOLD;
                glyph = gs->high;
            } else if (tg > GLYPH_MID_THRESH) {
                pair  = PAIR_TRAIL_BASE + (f->trail_color[idx] & 3);
                attr  = A_BOLD;
                glyph = gs->mid;
            } else if (tg > GLOW_THRESHOLD) {
                pair  = PAIR_TRAIL_BASE + (f->trail_color[idx] & 3);
                attr  = A_NORMAL;
                glyph = gs->low;
            } else {
                continue;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* Pass 2 — attractor markers '@' on top, in flash colour. */
    attron(COLOR_PAIR(PAIR_ATTRACT) | A_BOLD);
    for (int i = 0; i < f->n_active_attr; i++) {
        int sx = gx0 + (int)f->active_attr[i].x;
        int sy = gy0 + (int)f->active_attr[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)'@');
    }
    attroff(COLOR_PAIR(PAIR_ATTRACT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Field *f = &s->F;
    const char *state_str = s->paused
                          ? "PAUSED   "
                          : pattern_name(s->current_pattern);

    float reset_secs = (float)f->reset_countdown / 60.0f;
    if (reset_secs < 0.0f) reset_secs = 0.0f;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d  reset:%4.1fs ",
             fps, sim_fps, state_str, s->speed, (double)reset_secs);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " FLOW FIELD PARTICLES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 20;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 9;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_TRAIL_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  parts:%d  attr:%d  map:%dx%d ",
             f->n_particles, f->n_active_attr, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Glyph-set indicator on row 1 (after the existing detail).
     * Format: " glyph:NAME [.*#] " — live sample of the three chars
     * so you can see what each set looks like before switching. */
    int gs_idx = s->current_glyph_set;
    if (gs_idx < 0 || gs_idx >= N_GLYPH_SETS) gs_idx = 0;
    const GlyphSet *gs = &glyph_sets[gs_idx];
    /* Find current x position; mvprintw above doesn't return one, so
     * we re-derive by scanning past what we wrote. Approximate: the
     * earlier mvprintw used roughly 28 + N digits; for our typical
     * grid (~3-5 digits per number) end up at ~x+38. Use a safer
     * approach: query with getyx after one of the prints. Simpler
     * still: use a fresh mvprintw with explicit column. */
    /* (We don't track exact columns precisely; use right-aligned
     * printout below the right HUD instead. Print at row 1 right of
     * the existing detail.) */
    /* For simplicity, re-print on the right edge of the row 0 HUD's
     * column area is messy. Use end of row 1 — fits easily. */
    char gbuf[32];
    snprintf(gbuf, sizeof gbuf, " glyph:%-7s [%c%c%c] ",
             gs->name, gs->low, gs->mid, gs->high);
    int gx = sc->cols - (int)strlen(gbuf);
    if (gx < 0) gx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, gx, "%s", gbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " trails:density-graded  @:attractor | n/p:pattern  g/G:glyph  t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
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
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
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

    /* Glyph set cycling — slim → fat thickness ramp. Doesn't reset
     * the simulation; just changes how trails render. */
    case 'g':
        s->current_glyph_set = (s->current_glyph_set + 1) % N_GLYPH_SETS;
        break;
    case 'G':
        s->current_glyph_set = (s->current_glyph_set + N_GLYPH_SETS - 1) % N_GLYPH_SETS;
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
