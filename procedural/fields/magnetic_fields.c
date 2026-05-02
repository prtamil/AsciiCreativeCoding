/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_fields.c
 *   — Magnetic field visualisation: iron-filings-style flow.
 *
 * DEMO: Hundreds of particles flow along the magnetic field lines
 *       from N poles to S poles, tracing the iconic patterns you see
 *       when iron filings are sprinkled around a magnet. Pole
 *       positions are fixed by the active pattern; the field is the
 *       sum of inverse-square contributions from each pole. Five
 *       configurations are available:
 *         DIPOLE    — classic bar magnet (1 N + 1 S)
 *         QUAD      — quadrupole (4 alternating poles in a square)
 *         OCTUPOLE  — 8 alternating poles around a ring
 *         CHAIN     — 6 poles in a row (NSNSNS) — multi-magnet array
 *         PLASMA    — 14 random poles with random polarities
 *       Pole markers render as red 'N' / blue 'S' glyphs on top of
 *       the particle trails so polarity stays clear across all themes.
 *
 * Study alongside:
 *   ./flow_field_particles.c — same particle-flow architecture, but
 *       with abstract algebraic vector fields. Magnetic fields here
 *       are physically meaningful: every streamline is a real iron-
 *       filing path, every singular point is a magnetic monopole.
 *   ./curl_noise_vector_field.c — divergence-FREE flow. Magnetic
 *       fields are also divergence-free in nature (∇·B = 0, no
 *       monopoles), but our 2-D simulation uses signed monopoles
 *       as a convenient approximation; ∇·B ≠ 0 exactly at each pole.
 *
 * Section map:
 *   §1 config   — grid, particles, palette, themes, glyph sets
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes + N/S pole pairs
 *   §5 fields   — magnetic field math: B(r) = Σ qᵢ · r̂ᵢ / rᵢ²
 *   §6 patterns — 5 pole configurations
 *   §7 scene    — Field, Particle, scene state
 *   §8 screen   — ASCII render: trails + N/S pole markers
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   g / G      next / previous glyph set (slim → fat)
 *   + / =      faster particles
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra magnetic_fields.c \
 *       -o magnetic -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle advection through a 2-D magnetic field. The
 *                  field at a point r is the sum of inverse-square
 *                  contributions from N magnetic poles:
 *
 *                      B(r) = Σᵢ qᵢ · (r − rᵢ) / |r − rᵢ|³
 *
 *                  where qᵢ = +1 for north poles, −1 for south, and rᵢ
 *                  is each pole's position. The field points OUT of
 *                  positive (north) poles and INTO negative (south)
 *                  poles; field lines are continuous arcs from N to S.
 *
 *                  Particles step along the local B direction, leaving
 *                  trails. After enough particles have flowed, the
 *                  trails reveal the FIELD LINES — the same pattern
 *                  iron filings make when sprinkled around a magnet.
 *
 *                  Each pattern is a different pole configuration
 *                  (number, positions, polarities). The simulation
 *                  itself is identical across patterns; only the pole
 *                  list changes.
 *
 *                  Note on physics: real magnetic fields have ∇·B = 0
 *                  everywhere (no magnetic monopoles). Our "monopole"
 *                  poles are a convenient 2-D approximation — at exact
 *                  pole positions, ∇·B blows up, but particle flow on
 *                  the rest of the field is accurate to the physical
 *                  picture.
 *
 * Data-structure : Pole list (max 32) + particle pool (256). Particles
 *                  store float (x, y) + age + colour. Per-cell glow +
 *                  colour buffers. Five glyph sets and ten themes.
 *
 * Rendering      : ASCII only. Density-graded particle trails in 4
 *                  theme palette colours. North poles render as red 'N'
 *                  glyphs, south poles as blue 'S' glyphs — colours
 *                  are theme-INDEPENDENT so polarity stays readable on
 *                  every theme.
 *
 * Performance    : Up to 14 pole evaluations per particle per frame
 *                  (PLASMA). With 256 particles at 60 Hz that's ~210 K
 *                  function calls/sec — essentially free on modern
 *                  hardware. Each pole evaluation is one sqrtf + 5
 *                  multiplies.
 *
 * References     : • Griffiths, D.J. — "Introduction to Electrodynamics".
 *                    The standard reference for magnetic-field math.
 *                  • Visual: any high-school physics textbook's iron-
 *                    filings photo around a bar magnet. The patterns
 *                    you see there are exactly what this file produces.
 *                  • Wikipedia — "Magnetic field":
 *                    https://en.wikipedia.org/wiki/Magnetic_field
 *                  • Compare ./flow_field_particles.c — same particle
 *                    architecture, different (purely abstract) fields.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Sprinkle iron filings on paper above a magnet. Tap the paper. The
 * filings line up along invisible "field lines" connecting north and
 * south poles. Those lines aren't drawn anywhere — they emerge from
 * the local magnetic force on each filing. We do the same thing
 * computationally: drop particles, let each read the local B at its
 * position, step along that direction. After many particles flow,
 * the cumulative trails ARE the field lines.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each pole is a "fountain" or a "drain" for the field:
 *   • N pole — field flows OUT in all directions (a fountain).
 *   • S pole — field flows IN from all directions (a drain).
 * When you have both a fountain and a drain on the same plane, the
 * fountain's water curves around to fall into the drain — that's the
 * dipole field-line pattern. Add more fountains and drains and the
 * curves get more elaborate; they NEVER cross (a physical impossibility
 * in 2-D since the field has a unique direction at every point).
 *
 * Visible layers:
 *   1. PARTICLE TRAILS — fading streamlines in 4 theme colours.
 *      Density-graded glyphs ('.', '*', '#') show flux density —
 *      bright cells are where many particles converge (near pole
 *      "drains") or diverge (around pole "fountains").
 *   2. POLE MARKERS — red 'N' and blue 'S' glyphs sitting at each
 *      pole's exact position, ALWAYS visible (rendered after trails).
 *   3. The streamlines curve smoothly. No straight lines except
 *      directly along the axis between two oppositely-charged poles.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Spawn 256 particles uniformly in-bounds. Initialise the
 *     active pattern's pole list (positions + polarities).
 *  2. PER FRAME:
 *     a. Slight orbital drift: each pole's position oscillates by a
 *        few cells around its base position, so the field evolves.
 *     b. For each particle:
 *        i.   Compute B(particle.pos) = Σ qᵢ · (r − rᵢ) / |r − rᵢ|³
 *        ii.  Normalise to unit direction (so visual speed is
 *             consistent regardless of the field's wildly varying
 *             magnitude near poles vs far from them).
 *        iii. Step by direction · speed · dt.
 *        iv.  Increment age. If age ≥ max_age OR position OOB,
 *             respawn at random in-bounds.
 *        v.   Paint trail at the new cell.
 *     c. Render trails first, then pole markers on top.
 *  3. Periodic supernova reset every ~12 s for fresh particle seeding.
 *
 * KEY FORMULAS
 * ────────────
 *  Single-pole field             :
 *    B_i(r) = qᵢ · (r − rᵢ) / |r − rᵢ|³
 *    (qᵢ = ±1 signed pole charge; r̂ from pole to query point)
 *  Total field                   : B(r) = Σᵢ B_i(r)
 *  Particle step                 :
 *    direction = B / |B|;  Δr = direction · speed · dt
 *  Softening                     :
 *    |r − rᵢ|² + ε  to avoid singularities at exact pole positions
 *  Pole drift                    :
 *    pole_pos = base_pos + (Δx, Δy) · cos/sin(t · ω + φ)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SINGULARITIES AT POLES. The 1/r³ form blows up if any particle
 *    lands exactly on a pole. We add a softening ε to the denominator
 *    (ε = 4 cell² is comfortable). Without it, particles can hit
 *    infinite velocity and warp across the screen.
 *
 *  • UNNORMALISED MAGNITUDE. Raw B is huge near poles, tiny far away.
 *    If we step by raw B, particles near poles teleport away while
 *    particles far away barely move. Normalise to unit direction.
 *
 *  • PARTICLE SINKS. With S poles in the configuration, particles
 *    that approach an S pole get sucked in and stop moving (the
 *    nearest neighbour cell IS the pole). Without max_age respawn,
 *    all particles eventually pile up at the S poles. The respawn
 *    keeps coverage alive.
 *
 *  • ALL-N or ALL-S CONFIGURATIONS would have non-physical field
 *    (no closing field lines). We always balance N and S in even-pole
 *    patterns; in PLASMA we randomise but with bias toward equal
 *    counts.
 *
 *  • DRIFT VS FLOW. If pole drift is too fast, the field changes
 *    quicker than particles can flow → confused streamlines. We use
 *    DRIFT_RATE = 0.2 rad/s — much slower than typical particle
 *    transit times.
 *
 *  • POLE MARKERS OVERLAPPING TRAILS. The 'N' / 'S' glyphs are drawn
 *    LAST, on top, so even if a bright trail cell coincides with a
 *    pole the user still sees the pole marker. The trail behind it
 *    is hidden — that's intended.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • DIPOLE pattern: trails clearly arc from N to S in the textbook
 *    dipole shape. Field lines should be denser between the poles
 *    (along the axis) and along the perpendicular bisector.
 *  • QUAD: 4 'lobes' between alternating poles, each a smaller
 *    dipole arc.
 *  • OCTUPOLE: 8 lobes around a ring, each between adjacent
 *    alternating poles.
 *  • CHAIN: snake-like undulations between consecutive poles.
 *  • PLASMA: chaotic, no obvious global structure but locally
 *    clear arcs.
 *  • Switching themes preserves the field-line shapes; only colours
 *    change. Switching glyph sets only changes density-glyph
 *    appearance, not the line geometry.
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

    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    RESET_TICKS_DEF   = 12 * 60,

    MAX_POLES         =  32,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,    /* PAIR_TRAIL_BASE..+3 = 4 trail colours */
    PAIR_NORTH        =   7,    /* red 'N' marker — theme-independent   */
    PAIR_SOUTH        =   8,    /* blue 'S' marker — theme-independent  */
    PAIR_SUPERNOVA    =   9,
};

#define TRAIL_GLOW_DECAY    0.6f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Pole drift rate (rad/s) — slow oscillation of base positions so the
 * field evolves rather than freezing. */
#define DRIFT_RATE          0.2f

/* Field strengths — tuned so particle motion is visible at SPEED_DEF. */
#define DIPOLE_STRENGTH     50.0f
#define QUAD_STRENGTH       40.0f
#define OCTUPOLE_STRENGTH   25.0f
#define CHAIN_STRENGTH      35.0f
#define PLASMA_STRENGTH     25.0f
#define EPS_R2              4.0f    /* softening — avoid 1/0 at pole centres */

#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern — 5 magnetic-pole configurations.
 *
 *   DIPOLE   : 1 N + 1 S — classic bar magnet
 *   QUAD     : 4 alternating poles at corners of a square
 *   OCTUPOLE : 8 alternating poles around a ring
 *   CHAIN    : 6 alternating poles in a horizontal line
 *   PLASMA   : 14 random poles with random polarities
 */
typedef enum {
    PATTERN_DIPOLE   = 0,
    PATTERN_QUAD     = 1,
    PATTERN_OCTUPOLE = 2,
    PATTERN_CHAIN    = 3,
    PATTERN_PLASMA   = 4,
    N_PATTERNS       = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_DIPOLE:   return "DIPOLE  ";
    case PATTERN_QUAD:     return "QUAD    ";
    case PATTERN_OCTUPOLE: return "OCTUPOLE";
    case PATTERN_CHAIN:    return "CHAIN   ";
    case PATTERN_PLASMA:   return "PLASMA  ";
    default:               return "?       ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names. Each defines 4 trail colours + a flash
 * accent (used for the supernova).
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
 * GlyphSet — 3-character density ramp, slim → fat. Same idea and
 * layout as flow_field_particles.c — the glyph set is decoupled
 * from theme so they can be combined freely.
 */
typedef struct {
    const char *name;
    char low, mid, high;
} GlyphSet;

#define N_GLYPH_SETS 5

static const GlyphSet glyph_sets[N_GLYPH_SETS] = {
    { "SLIM",    '.', '\'', ':' },
    { "LIGHT",   '.', '*',  '+' },
    { "MEDIUM",  '.', '*',  '#' },
    { "HEAVY",   'o', 'O',  '@' },
    { "FAT",     '+', '#',  'M' },
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
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        /* N = red, S = blue — physics convention. Theme-independent so
         * polarity is unambiguous on every theme. */
        init_pair(PAIR_NORTH,      196, -1);   /* hot red               */
        init_pair(PAIR_SOUTH,       33, -1);   /* clear blue            */
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_NORTH,     COLOR_RED,     -1);
        init_pair(PAIR_SOUTH,     COLOR_BLUE,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  fields — magnetic field math                                       */
/* ===================================================================== */

/*
 * Pole — one magnetic monopole.
 *
 *   bx, by      : nominal "base" position (the centre of its small orbit)
 *   ox, oy      : orbital radius components
 *   phase       : orbital phase offset
 *   polarity    : +1 = N, -1 = S
 *   strength    : multiplier
 *
 * Live position each frame:
 *   x = bx + ox · cos(t · DRIFT_RATE + phase)
 *   y = by + oy · sin(t · DRIFT_RATE + phase)
 */
typedef struct {
    float bx, by;
    float ox, oy;
    float phase;
    float strength;
    int   polarity;
} Pole;

/*
 * ActivePole — one frame's evaluated pole position.
 */
typedef struct {
    float x, y;
    float strength;
    int   polarity;
} ActivePole;

/*
 * field_at — evaluate B(x, y) for the given list of poles.
 *
 *   B = Σᵢ qᵢ · (r − rᵢ) / |r − rᵢ|³
 *
 * Implementation: for each pole, compute Δr = (x − pole.x, y − pole.y),
 * its squared length r² + ε (softening), and its actual length r.
 * The vector contribution is Δr · (q · S / r³), summed over poles.
 */
static void field_at(const ActivePole *poles, int n_poles,
                     float x, float y,
                     float *out_bx, float *out_by)
{
    float bx = 0.0f, by = 0.0f;
    for (int i = 0; i < n_poles; i++) {
        float dx = x - poles[i].x;
        float dy = y - poles[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float r3 = r2 * sqrtf(r2);
        float w  = (float)poles[i].polarity * poles[i].strength / r3;
        bx += dx * w;
        by += dy * w;
    }
    *out_bx = bx;
    *out_by = by;
}

/* ===================================================================== */
/* §6  patterns — 5 pole configurations                                   */
/* ===================================================================== */

/*
 * pattern_init_poles — populate the pole list for the active pattern.
 * Called at scene reset and every pattern switch.
 *
 * w, h are the map dims (so pole positions can be placed in-bounds).
 *
 * Returns the number of poles populated.
 */
static int pattern_init_poles(Pole out[], int max_n, Pattern p, int w, int h)
{
    int n = 0;
    float cx = (float)w * 0.5f;
    float cy = (float)h * 0.5f;

    switch (p) {

    case PATTERN_DIPOLE: {
        float dx = (float)w * 0.20f;
        out[n++] = (Pole){
            .bx = cx - dx, .by = cy,
            .ox = 3.0f, .oy = 2.0f, .phase = 0.0f,
            .strength = DIPOLE_STRENGTH, .polarity = +1,    /* N */
        };
        out[n++] = (Pole){
            .bx = cx + dx, .by = cy,
            .ox = 3.0f, .oy = 2.0f, .phase = (float)M_PI,
            .strength = DIPOLE_STRENGTH, .polarity = -1,    /* S */
        };
        break;
    }

    case PATTERN_QUAD: {
        /* 4 poles at the corners of an inner square, alternating signs. */
        float ax = (float)w * 0.18f;
        float ay = (float)h * 0.22f;
        struct { float dx, dy; int sign; } qcfg[4] = {
            {-ax, -ay, +1}, {+ax, -ay, -1},
            {-ax, +ay, -1}, {+ax, +ay, +1},
        };
        for (int i = 0; i < 4 && n < max_n; i++) {
            out[n++] = (Pole){
                .bx = cx + qcfg[i].dx, .by = cy + qcfg[i].dy,
                .ox = 2.0f, .oy = 2.0f,
                .phase = (float)i * 0.5f,
                .strength = QUAD_STRENGTH, .polarity = qcfg[i].sign,
            };
        }
        break;
    }

    case PATTERN_OCTUPOLE: {
        /* 8 poles around a ring, alternating N/S. */
        float rx = (float)w * 0.28f;
        float ry = (float)h * 0.32f;
        for (int i = 0; i < 8 && n < max_n; i++) {
            float ang = (float)i * (float)M_PI / 4.0f;
            out[n++] = (Pole){
                .bx = cx + rx * cosf(ang),
                .by = cy + ry * sinf(ang),
                .ox = 1.5f, .oy = 1.5f,
                .phase = ang,
                .strength = OCTUPOLE_STRENGTH,
                .polarity = (i & 1) ? -1 : +1,
            };
        }
        break;
    }

    case PATTERN_CHAIN: {
        /* 6 poles in a horizontal line, alternating N/S. */
        int n_chain = 6;
        float spacing = (float)w * 0.12f;
        float left = cx - spacing * (float)(n_chain - 1) * 0.5f;
        for (int i = 0; i < n_chain && n < max_n; i++) {
            out[n++] = (Pole){
                .bx = left + spacing * (float)i, .by = cy,
                .ox = 2.0f, .oy = 1.0f,
                .phase = (float)i * 0.4f,
                .strength = CHAIN_STRENGTH,
                .polarity = (i & 1) ? -1 : +1,
            };
        }
        break;
    }

    case PATTERN_PLASMA: {
        /* 14 random positions, polarity alternates with a small bias
         * to keep the N/S count roughly balanced. */
        int n_plasma = 14;
        for (int i = 0; i < n_plasma && n < max_n; i++) {
            int margin_x = w / 8;
            int margin_y = h / 6;
            float px = (float)(margin_x + rand() % (w - 2 * margin_x));
            float py = (float)(margin_y + rand() % (h - 2 * margin_y));
            out[n++] = (Pole){
                .bx = px, .by = py,
                .ox = 1.5f, .oy = 1.0f,
                .phase = (float)i * 0.31f,
                .strength = PLASMA_STRENGTH,
                .polarity = ((i + (rand() & 1)) & 1) ? -1 : +1,
            };
        }
        break;
    }

    default:
        break;
    }
    return n;
}

/*
 * compute_active_poles — derive each frame's "live" pole positions
 * from base + orbital drift.
 */
static int compute_active_poles(const Pole src[], int n_src,
                                ActivePole out[], float t)
{
    for (int i = 0; i < n_src; i++) {
        float ang = t * DRIFT_RATE + src[i].phase;
        out[i].x        = src[i].bx + src[i].ox * cosf(ang);
        out[i].y        = src[i].by + src[i].oy * sinf(ang);
        out[i].strength = src[i].strength;
        out[i].polarity = src[i].polarity;
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
    int        w, h;
    int        total_cells;

    float      trail_glow [CELLS_MAX];
    uint8_t    trail_color[CELLS_MAX];
    float      supernova_glow_t;

    Particle   particles[MAX_PARTICLES];
    int        n_particles;

    Pole       poles[MAX_POLES];
    int        n_poles;
    ActivePole active_poles[MAX_POLES];
    int        n_active_poles;

    float      field_time;
    int        reset_countdown;
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
 * particle_step — read B at the particle's position, normalise to a
 * unit direction, step. Respawn on max_age or OOB.
 *
 * Normalisation is crucial: raw B is enormous near poles and tiny far
 * from them. Without it, particles near poles teleport across the
 * map while distant particles stand still.
 */
static void particle_step(Field *f, Particle *p, float dt, int speed)
{
    float bx, by;
    field_at(f->active_poles, f->n_active_poles, p->x, p->y, &bx, &by);

    float m = sqrtf(bx * bx + by * by);
    if (m > 1e-6f) { bx /= m; by /= m; }

    p->x += bx * (float)speed * dt;
    p->y += by * (float)speed * dt;
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
    f->n_poles = pattern_init_poles(f->poles, MAX_POLES, pat, w, h);
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
    s->current_glyph_set  = 2;        /* MEDIUM */
    s->current_pattern    = PATTERN_DIPOLE;
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

    f->n_active_poles = compute_active_poles(f->poles, f->n_poles,
                                             f->active_poles, f->field_time);

    for (int i = 0; i < f->n_particles; i++) {
        particle_step(f, &f->particles[i], dt, s->speed);
    }

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

    /* Lookup glyph set once per frame. */
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

    /* Pass 2 — pole markers on top, theme-independent red 'N' / blue 'S'. */
    for (int i = 0; i < f->n_active_poles; i++) {
        int sx = gx0 + (int)f->active_poles[i].x;
        int sy = gy0 + (int)f->active_poles[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        int  pair  = (f->active_poles[i].polarity > 0) ? PAIR_NORTH : PAIR_SOUTH;
        char glyph = (f->active_poles[i].polarity > 0) ? 'N' : 'S';
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Field *f = &s->F;
    const char *state_str = s->paused
                          ? "PAUSED  "
                          : pattern_name(s->current_pattern);

    float reset_secs = (float)f->reset_countdown / 60.0f;
    if (reset_secs < 0.0f) reset_secs = 0.0f;

    /* Count N and S poles for the HUD. */
    int n_north = 0, n_south = 0;
    for (int i = 0; i < f->n_poles; i++) {
        if (f->poles[i].polarity > 0) n_north++; else n_south++;
    }

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
    mvprintw(0, 1, " MAGNETIC FIELDS ");
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
    /* N/S pole counts in their respective colours. */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  ");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 2;
    attron(COLOR_PAIR(PAIR_NORTH) | A_BOLD);
    mvprintw(1, x, "N:%d", n_north);
    attroff(COLOR_PAIR(PAIR_NORTH) | A_BOLD);
    x += (n_north < 10 ? 3 : 4);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 1;
    attron(COLOR_PAIR(PAIR_SOUTH) | A_BOLD);
    mvprintw(1, x, "S:%d", n_south);
    attroff(COLOR_PAIR(PAIR_SOUTH) | A_BOLD);
    x += (n_south < 10 ? 3 : 4);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  parts:%d  map:%dx%d ",
             f->n_particles, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Glyph-set indicator on right of row 1. */
    int gs_idx = s->current_glyph_set;
    if (gs_idx < 0 || gs_idx >= N_GLYPH_SETS) gs_idx = 0;
    const GlyphSet *gs = &glyph_sets[gs_idx];
    char gbuf[32];
    snprintf(gbuf, sizeof gbuf, " glyph:%-7s [%c%c%c] ",
             gs->name, gs->low, gs->mid, gs->high);
    int grx = sc->cols - (int)strlen(gbuf);
    if (grx < 0) grx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, grx, "%s", gbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " trails:field-lines  N:north  S:south | n/p:pattern  g/G:glyph  t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
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
