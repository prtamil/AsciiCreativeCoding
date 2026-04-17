/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_centipede.c — FK Centipede with Gait-Driven Legs
 *
 * DEMO: A 16-segment centipede crawls autonomously across the terminal.
 *       The body uses a circular trail-buffer path-following FK (identical
 *       to snake_forward_kinematics.c).  Each of 7 leg-pairs is a 2-joint
 *       chain (hip → knee → foot) driven by stateless sinusoidal FK:
 *       gait phase is computed from wave_time + per-leg phase offset,
 *       producing a natural alternating-gait wave that rolls down the body.
 *
 * STUDY ALONGSIDE: snake_forward_kinematics.c (trail-buffer FK body)
 *                  framework.c (canonical loop / timing template)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — earthy/amber palette + dedicated HUD pair
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Centipede: body trail FK + stateless leg FK
 *       §5a  trail helpers      — push, index, arc-length sampler
 *       §5b  move_head          — steer + translate + wrap + record
 *       §5c  compute_joints     — body placement from trail
 *       §5d  compute_legs       — stateless FK for all leg pairs
 *       §5e  rendering helpers  — glyphs, body colors, line drawer
 *       §5f  render_centipede   — full frame composition
 *   §6  scene         — scene_init / scene_tick / scene_draw wrappers
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * TWO FK MECHANISMS — HOW THEY DIFFER AND WHY
 * ─────────────────────────────────────────────
 *
 *  MECHANISM           │ Trail-buffer FK (body)    │ Stateless sinusoidal FK (legs)
 *  ────────────────────┼───────────────────────────┼──────────────────────────────
 *  State required      │ Yes — circular trail[]    │ No — all from wave_time
 *  Memory cost         │ TRAIL_CAP × 8 B = 32 KB   │ Zero extra state
 *  Position at time t  │ Query arc-length in trail │ Pure math formula
 *  History encoded     │ Yes — all past positions  │ No — oscillator only
 *  Can follow curves   │ Yes (any path the head    │ No — fixed geometric shape
 *                      │ carves, including IK)     │ relative to body joint
 *  Good for            │ Body, tail, chain, rope   │ Legs, fins, antennae, fur
 *
 * HOW THE BODY WORKS — trail-buffer path-following FK
 * ────────────────────────────────────────────────────
 * Identical to snake_forward_kinematics.c:
 *   1. Every tick, the head's pixel position is pushed into a circular trail.
 *   2. Body joint i = trail_sample(i × SEG_LEN_PX) — arc-length interpolation
 *      backward along the recorded path.  The body follows the exact path
 *      the head carved; no per-segment angle formula is needed.
 *
 *   Key insight: arc-length interpolation means joint spacing is always
 *   exactly SEG_LEN_PX pixels regardless of the head's instantaneous speed.
 *   The body stretches/compresses correctly through tight curves.
 *
 * HOW THE LEGS WORK — stateless sinusoidal FK
 * ────────────────────────────────────────────
 * Each leg pair i (0-based) attaches to a body joint computed from the trail.
 * Left and right legs are placed symmetrically using the body direction vector.
 *
 * For each leg:
 *   gait_phase = wave_time × GAIT_FREQ + i × (π / N_LEGS)  [left]
 *              = gait_phase_left + π                          [right — antiphase]
 *
 *   upper_angle = body_dir ± LEG_SPLAY + SWING_AMP × sin(gait_phase)
 *   lower_angle = upper_angle + LEG_BEND + LOWER_SWING × sin(gait_phase + π/4)
 *
 *   hip   = body_joint ± BODY_OFFSET × normal_vector
 *   knee  = hip  + UPPER_LEN × (cos upper_angle, sin upper_angle)
 *   foot  = knee + LOWER_LEN × (cos lower_angle, sin lower_angle)
 *
 * No IK, no contact constraints — all positions are computed analytically
 * from wave_time each tick.  The gait phase offset i×π/N_LEGS creates a
 * travelling wave of leg steps from head to tail, which is how real
 * myriapods (centipedes, millipedes) coordinate their legs.
 *
 * CONTRALATERAL ANTIPHASE GAIT
 * ─────────────────────────────
 * phi_R = phi_L + π means the right leg is exactly half a cycle behind the
 * left leg.  When the left leg is at peak forward (swing), the right leg
 * is at peak backward (stance).  This is the defining trait of alternating
 * tripod / wave gait — each side forms a supporting base while the other
 * swings.  At any instant exactly half the legs are on the ground.
 *
 * Keys:
 *   q / ESC         quit
 *   space           pause / resume
 *   ↑ / ↓           move speed faster / slower
 *   ← / →           undulation frequency slower / faster
 *   w / s           amplitude wider / narrower
 *   [ / ]           simulation Hz down / up
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       fk_centipede.c -o fk_centipede -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Body uses path-following FK — the head's trail buffer
 *                  encodes positional history; each joint is placed at a
 *                  fixed arc-length distance along that history via linear
 *                  interpolation.  Legs use stateless sinusoidal FK: all
 *                  joint positions are computed analytically from wave_time
 *                  and per-leg phase offsets, producing a coordinated gait
 *                  without any explicit gait state machine.
 *
 * Data-structure : Circular trail buffer Vec2 trail[TRAIL_CAP].  Joint
 *                  arrays joint[]/prev_joint[] for body alpha-lerp.
 *                  Leg positions leg_left/leg_right[N_LEGS][3] (hip, knee,
 *                  foot) with prev_ counterparts for alpha-lerp.
 *
 * Rendering      : Body drawn dense (DRAW_STEP_PX increments).  Each leg
 *                  drawn as two line segments (hip→knee, knee→foot) using
 *                  the same dense stepping and angle-based glyph selection.
 *                  Earthy amber-to-green 7-pair gradient, head in bold red.
 *                  Alpha interpolation blends all prev/current positions for
 *                  smooth sub-tick motion at any render frame rate.
 *
 * Performance    : Fixed-step accumulator decouples physics from render Hz.
 *                  ncurses doupdate() sends only changed cells to terminal.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/*
 * M_PI is a POSIX extension, not standard C99/C11.
 * Define a fallback so the build never fails on strict-conformance toolchains.
 */
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

/*
 * All magic numbers live here.  Never scatter literals through the code.
 */
enum {
    /*
     * SIM_FPS — physics tick rate in Hz.
     * The fixed-step accumulator in §8 fires scene_tick() this many times
     * per second regardless of render frame rate.
     */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    /*
     * HUD_COLS — byte budget for the status-bar string.
     * 96 bytes covers the longest possible HUD string at max values.
     * FPS_UPDATE_MS — fps display refresh interval.
     */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /*
     * N_PAIRS — gradient body pairs (1-7 = earthy amber palette).
     * HUD_PAIR = 8 is reserved for the status bar.
     */
    N_PAIRS          =   7,
    HUD_PAIR         =   8,

    /*
     * BODY_SEGS — number of rigid body segments (= joints - 1).
     * 24 segments at 20 px each = 480 px total body length ≈ 30 terminal rows,
     * giving a long, sinuous snake-like silhouette across the screen.
     */
    BODY_SEGS        =  24,

    /*
     * N_LEGS — number of leg pairs (left + right each).
     * 10 pairs across 24 body segments → one pair every ~2.4 segments,
     * matching the dense-leg look of a real centipede.
     */
    N_LEGS           =  10,

    /*
     * TRAIL_CAP — circular head-position buffer capacity.
     * Body length = 24 × 20 = 480 px.  At min speed 10 px/s, 60 Hz:
     * 0.167 px/tick → 480 / 0.167 ≈ 2876 entries needed; 4096 = safe margin.
     */
    TRAIL_CAP        = 4096,

    /*
     * N_THEMES — number of switchable color themes.
     * Cycled with the 't' key at runtime.
     */
    N_THEMES         =  10,
};

/*
 * SEG_LEN_PX — pixel length of each body segment.
 * 20 px = 2.5 terminal columns, balancing arc-length resolution and
 * visual segment separation.  24 segs × 20 px = 480 px body length.
 *
 * DRAW_STEP_PX — step size for the dense segment renderer (pass 1).
 * 5.0 px spaces glyphs apart so joint node markers (pass 2) are visible.
 */
#define SEG_LEN_PX     20.0f
#define DRAW_STEP_PX    5.0f

/*
 * Leg geometry — all distances in square pixel space (same coord system as body).
 *
 * Schematic (one leg pair, left side, body going right →):
 *
 *    body axis ──────── body_joint ──────── (body direction)
 *                           │
 *              BODY_OFFSET  │  (lateral normal = body_dir + π/2)
 *                           │
 *                         hip_L ─── UPPER_LEN ──→ knee_L
 *                                                     │
 *                                            LOWER_LEN│
 *                                                     ↓
 *                                                  foot_L
 *
 * UPPER_LEN = 14 px: hip→knee at default speed 65 px/s and GAIT_FREQ 2 Hz,
 *   the leg swings ≈ ±SWING_AMP×UPPER_LEN = ±5.6 px → visible foot travel.
 *   14 px = 1.75 terminal columns — long enough to be clearly visible.
 *
 * LOWER_LEN = 12 px: slightly shorter than upper gives a bent-knee look.
 *   Total reach = UPPER_LEN + LOWER_LEN = 26 px = 3.25 cols from hip.
 *   Real centipedes have a similar femur/tibia length ratio (~1.2:1).
 *
 * LEG_SPLAY = 1.2 rad (≈ 69°): the base angle between body axis and upper leg.
 *   At π/2 (90°) legs would point straight out; 1.2 rad tilts them slightly
 *   forward for the characteristic "reaching" posture of real arthropods.
 *   Valid range: 0.8–1.5 rad.  Below 0.8 legs point forward; above 1.5 backward.
 *
 * SWING_AMP = 0.4 rad (≈ 23°): upper-leg fore-aft swing per half-cycle.
 *   At foot: arc ≈ SWING_AMP × UPPER_LEN = 5.6 px — clearly visible motion.
 *   Too high (>1.0) makes legs clip through the body; too low (<0.1) barely moves.
 *
 * LOWER_SWING = 0.3 rad (≈ 17°): tibia (lower leg) secondary swing.
 *   The +π/4 phase offset (vs upper leg) makes the foot follow an elliptical
 *   arc rather than a circular arc — more realistic and reduces foot clipping.
 *
 * LEG_BEND = -0.5 rad (≈ -29°): static knee-bend, negative = bent downward.
 *   Without this bend the leg would be straight; a negative offset creates the
 *   characteristic bent-leg posture of insects at rest.  The gait oscillation
 *   modulates around this base angle.
 *
 * BODY_OFFSET = 8 px = exactly 1 terminal column: lateral distance from the
 *   body centerline to the hip.  Keeps hips clearly separated from body glyphs.
 */
#define UPPER_LEN      14.0f   /* hip → knee segment length (px)            */
#define LOWER_LEN      12.0f   /* knee → foot segment length (px)           */
#define LEG_SPLAY       1.2f   /* base splay from body axis (rad, ≈69°)     */
#define SWING_AMP       0.4f   /* upper-leg gait swing amplitude (rad)      */
#define LOWER_SWING     0.3f   /* lower-leg swing amplitude, +π/4 phase     */
#define LEG_BEND       -0.5f   /* static knee-bend offset (rad, ≈-29°)      */
#define BODY_OFFSET     8.0f   /* hip lateral offset from body center (px)  */

/*
 * Gait and locomotion parameters.
 *
 * TURN_AMP/TURN_FREQ match snake_forward_kinematics.c defaults (0.52/0.95)
 * so the body carves the same smooth S-curve sinusoidal path as the snake.
 *   Peak angular velocity = TURN_AMP = 0.52 rad/s
 *   Peak heading deflection = TURN_AMP / TURN_FREQ = 0.52/0.95 ≈ 0.55 rad = 31°
 *   Period = 2π / TURN_FREQ = 2π/0.95 ≈ 6.6 s per full undulation cycle
 *
 * GAIT_FREQ = 2.0 rad/s: leg oscillation runs at 2.0 rad/s.
 *   Full leg cycle = 2π / 2.0 ≈ 3.14 s.  At 65 px/s the centipede travels
 *   65 × 3.14 ≈ 204 px per leg cycle — about 25 terminal columns per stride.
 *
 * MOVE_SPEED = 65 px/s: slightly slower than the snake (80 px/s) because the
 *   centipede body is 24 × 20 = 480 px long — it needs more time on screen.
 */
#define GAIT_FREQ       2.0f   /* leg oscillation frequency (rad/s)         */
#define MOVE_SPEED     65.0f   /* head translation speed (px/s)             */
#define MOVE_SPEED_MIN 10.0f   /* slowest speed (px/s)                      */
#define MOVE_SPEED_MAX 300.0f  /* fastest speed (px/s)                      */
#define TURN_AMP        0.52f  /* sinusoidal body turn amplitude (rad/s)    */
#define TURN_AMP_MIN    0.0f   /* straight-line movement (no undulation)    */
#define TURN_AMP_MAX    3.0f   /* extreme coiling, nearly in-place spin     */
#define TURN_FREQ       0.95f  /* body undulation frequency (rad/s)         */
#define TURN_FREQ_MIN   0.05f  /* very slow, gentle curves                  */
#define TURN_FREQ_MAX   5.0f   /* rapid S-curves, body visibly waggles      */

/*
 * Timing primitives — verbatim from framework.c.
 * NS_PER_SEC / NS_PER_MS: unit conversion constants.
 * TICK_NS(f): converts frequency f (Hz) → period in nanoseconds.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display.  1 pixel represents the same physical distance in x and y.
 * CELL_W = 8 px, CELL_H = 16 px: a typical terminal cell is 2× taller.
 */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 * CLOCK_MONOTONIC never goes backward (unlike CLOCK_REALTIME).
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for exactly ns nanoseconds.
 * Used before render to cap frame rate at 60 fps.
 * If ns ≤ 0 the frame is over-budget; skip immediately.
 */
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
/* §3  color — 10 switchable themes + theme_apply()                      */
/* ===================================================================== */

/*
 * Theme — one color scheme for the centipede.
 *
 * body[7]  xterm-256 foreground colors for pairs 1-7 (tail→head gradient).
 * hud      foreground color for HUD pair 8.
 * name     display name shown in the HUD status bar.
 *
 * All themes use COLOR_BLACK background for maximum contrast.
 * In 8-color terminals theme_apply() falls back to 8 basic colors.
 *
 * Themes (cycled with 't'):
 *   0  Amber   — earthy brown → bright red        (default, feels alive)
 *   1  Matrix  — dark green → bright green         (hacker rain)
 *   2  Fire    — deep red → bright yellow          (combustion)
 *   3  Ocean   — deep blue → bright cyan           (underwater)
 *   4  Nova    — deep purple → hot pink → white    (stellar explosion)
 *   5  Toxic   — dark olive → acid green           (poison)
 *   6  Lava    — dark brown → bright orange        (volcanic)
 *   7  Ghost   — dark grey → bright white          (ethereal)
 *   8  Aurora  — teal → lavender → pale yellow     (northern lights)
 *   9  Neon    — electric blue → hot pink          (synthwave)
 */
typedef struct {
    const char *name;
    int body[N_PAIRS];   /* 7 foreground colors: tail (index 0) → head (6) */
    int hud;             /* HUD pair color */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* 0  Amber   */ { "Amber",  {130,136,142,148,154,160,196}, 226 },
    /* 1  Matrix  */ { "Matrix", { 22, 28, 34, 40, 46, 82,118},  46 },
    /* 2  Fire    */ { "Fire",   { 52, 88,124,160,196,208,226}, 208 },
    /* 3  Ocean   */ { "Ocean",  { 17, 18, 20, 27, 33, 45, 51},  51 },
    /* 4  Nova    */ { "Nova",   { 54, 93,129,165,201,213,225}, 213 },
    /* 5  Toxic   */ { "Toxic",  { 58, 64, 70, 76, 82,118,154}, 118 },
    /* 6  Lava    */ { "Lava",   { 52, 58, 94,130,166,202,208}, 202 },
    /* 7  Ghost   */ { "Ghost",  {234,238,242,246,250,254,231}, 252 },
    /* 8  Aurora  */ { "Aurora", { 23, 29, 35, 71,107,143,221}, 143 },
    /* 9  Neon    */ { "Neon",   { 57, 93,129,165,201,199,197}, 197 },
};

/*
 * theme_apply() — (re-)define all 8 ncurses color pairs for theme idx.
 *
 * Can be called at any time after start_color(); ncurses allows redefining
 * pairs at runtime.  The next frame automatically picks up new colors
 * because wattron(COLOR_PAIR(n)) resolves at draw time, not at pair-define time.
 *
 * 8-color fallback: maps the gradient to red/yellow/green/cyan palette.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *th = &THEMES[idx];

    if (COLORS >= 256) {
        for (int i = 0; i < N_PAIRS; i++)
            init_pair(i + 1, th->body[i], COLOR_BLACK);
        init_pair(HUD_PAIR, th->hud, COLOR_BLACK);
    } else {
        /* 8-color fallback — coarse gradient */
        int basic[N_PAIRS] = {
            COLOR_RED, COLOR_RED, COLOR_YELLOW,
            COLOR_YELLOW, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE
        };
        for (int i = 0; i < N_PAIRS; i++)
            init_pair(i + 1, basic[i], COLOR_BLACK);
        init_pair(HUD_PAIR, COLOR_YELLOW, COLOR_BLACK);
    }
}

/*
 * color_init() — one-time ncurses color setup; applies theme 0 (Amber).
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * All entity positions are stored in square pixel space (1 unit = 1 px).
 * Only at draw time do these helpers convert to cell coordinates.
 * formula: cell = floor(px / CELL_DIM + 0.5) = nearest-integer rounding.
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
/* §5  entity — Centipede                                                 */
/* ===================================================================== */

/*
 * Vec2 — lightweight 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention).
 */
typedef struct { float x, y; } Vec2;

/* Convenience vector helpers */
static inline float vec2_len(Vec2 v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}
static inline Vec2 vec2_norm(Vec2 v)
{
    float l = vec2_len(v);
    if (l < 1e-6f) return (Vec2){0.0f, 0.0f};
    return (Vec2){v.x / l, v.y / l};
}

/*
 * Centipede — complete simulation state.
 *
 * TRAIL BUFFER (circular, same design as snake_forward_kinematics.c):
 *   trail[]      Head position history pushed once per tick.
 *   trail_head   Write cursor (newest entry index).
 *   trail_count  Valid entries, clamped to TRAIL_CAP.
 *
 * BODY JOINTS:
 *   joint[0]            Head — set by move_head() each tick.
 *   joint[1..BODY_SEGS] Body — set by compute_joints() from trail.
 *   prev_joint[]        Snapshot at tick start; enables alpha lerp.
 *
 * LEG POSITIONS:
 *   leg_left[i][3]   Hip(0), knee(1), foot(2) for left leg pair i.
 *   leg_right[i][3]  Same for right leg pair i.
 *   prev_leg_*[]     Snapshot at tick start; enables alpha lerp.
 *
 * MOTION:
 *   heading      Travel direction (rad).  0=east, π/2=south.
 *   wave_time    Accumulated sim time (s) driving all oscillations.
 *   move_speed   Head translation (px/s).
 *   turn_amp     Peak sinusoidal turn rate (rad/s).
 *   turn_freq    Body undulation frequency (rad/s).
 *   paused       When true physics is frozen; prev saved for clean freeze.
 */
typedef struct {
    /* body — trail buffer FK (same as snake) */
    Vec2  trail[TRAIL_CAP];
    int   trail_head;
    int   trail_count;

    Vec2  joint[BODY_SEGS + 1];
    Vec2  prev_joint[BODY_SEGS + 1];

    float heading;
    float wave_time;
    float move_speed;
    float turn_amp;
    float turn_freq;
    int   theme_idx;   /* current color theme index into THEMES[N_THEMES] */
    bool  paused;

    /* legs — computed each frame from body joints */
    /* index [i][0]=hip [i][1]=knee [i][2]=foot   */
    Vec2  leg_left [N_LEGS][3];
    Vec2  leg_right[N_LEGS][3];
    Vec2  prev_leg_left [N_LEGS][3];
    Vec2  prev_leg_right[N_LEGS][3];
} Centipede;

/* ── §5a  trail helpers ─────────────────────────────────────────────── */

/*
 * trail_push() — append the head's new position to the circular buffer.
 * trail_head advances (mod TRAIL_CAP), overwriting the oldest entry when full.
 */
static void trail_push(Centipede *c, Vec2 pos)
{
    c->trail_head = (c->trail_head + 1) % TRAIL_CAP;
    c->trail[c->trail_head] = pos;
    if (c->trail_count < TRAIL_CAP) c->trail_count++;
}

/*
 * trail_at() — retrieve the entry k steps back from newest.
 * k=0 = newest (current head), k=1 = one tick older, etc.
 * Adding TRAIL_CAP before subtraction avoids negative modulo.
 */
static inline Vec2 trail_at(const Centipede *c, int k)
{
    return c->trail[(c->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * trail_sample() — interpolated position at arc-length dist from head.
 *
 * Walks the trail accumulating Euclidean distance.  When the accumulated
 * length reaches dist, linearly interpolates between the two bounding trail
 * entries.  This is the core of path-following FK: the body literally
 * follows the exact path the head carved, encoded in the trail.
 */
static Vec2 trail_sample(const Centipede *c, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(c, 0);   /* newest = current head */

    for (int k = 1; k < c->trail_count; k++) {
        Vec2  b   = trail_at(c, k);
        float dx  = b.x - a.x;
        float dy  = b.y - a.y;
        float seg = sqrtf(dx * dx + dy * dy);

        if (accum + seg >= dist) {
            /* dist falls within this segment — interpolate */
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return (Vec2){ a.x + dx * t, a.y + dy * t };
        }

        accum += seg;
        a      = b;
    }

    /* Trail exhausted before dist — return oldest known point */
    return trail_at(c, c->trail_count - 1);
}

/* ── §5b  move_head ─────────────────────────────────────────────────── */

/*
 * move_head() — advance wave_time, update heading, translate, wrap.
 *
 * Step 1: wave_time += dt  (drives all sinusoidal motion).
 * Step 2: sinusoidal turn rate integrated into heading.
 *   turn = turn_amp * sin(turn_freq * wave_time)
 *   heading += turn * dt
 * Step 3: translate head in pixel space along heading.
 * Step 4: toroidal screen wrap (re-enters from opposite side).
 * Step 5: push head position into trail for this tick.
 */
static void move_head(Centipede *c, float dt, int cols, int rows)
{
    /* Step 1: advance the oscillator clock */
    c->wave_time += dt;

    /* Step 2: sinusoidal turn rate — integrate into heading */
    float turn = c->turn_amp * sinf(c->turn_freq * c->wave_time);
    c->heading += turn * dt;

    /* Step 3: translate head in pixel space */
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);

    c->joint[0].x += c->move_speed * cosf(c->heading) * dt;
    c->joint[0].y += c->move_speed * sinf(c->heading) * dt;

    /* Step 4: toroidal wrap */
    if (c->joint[0].x <   0.0f) c->joint[0].x += wpx;
    if (c->joint[0].x >= wpx)   c->joint[0].x -= wpx;
    if (c->joint[0].y <   0.0f) c->joint[0].y += hpx;
    if (c->joint[0].y >= hpx)   c->joint[0].y -= hpx;

    /* Step 5: record head position */
    trail_push(c, c->joint[0]);
}

/* ── §5c  compute_joints ────────────────────────────────────────────── */

/*
 * compute_joints() — place all body joints by sampling the trail.
 * joint[i] = trail_sample(i × SEG_LEN_PX).
 * joint[0] already set by move_head().
 */
static void compute_joints(Centipede *c)
{
    for (int i = 1; i <= BODY_SEGS; i++) {
        c->joint[i] = trail_sample(c, (float)i * SEG_LEN_PX);
    }
}

/* ── §5d  compute_legs ──────────────────────────────────────────────── */

/*
 * compute_legs() — stateless FK for all leg pairs.
 *
 * For each leg pair i (0 = frontmost, N_LEGS-1 = rearmost):
 *
 * 1. Determine the body joint this pair attaches to.
 *    We distribute N_LEGS attachment points evenly across joints 1..BODY_SEGS-1
 *    (skip head joint 0 and last joint for aesthetics).
 *    body_idx = 1 + i * (BODY_SEGS - 2) / (N_LEGS - 1)
 *    If N_LEGS == 1, use the midpoint.
 *
 * 2. Compute body direction at joint body_idx:
 *    dir_vec = normalize(joint[body_idx-1] - joint[body_idx+1])
 *    body_dir = atan2(dir_vec.y, dir_vec.x)
 *
 * 3. Body left and right normals (perpendicular to body direction):
 *    left_normal  = body_dir + π/2   (in terminal space: +y = down)
 *    right_normal = body_dir - π/2
 *    These point laterally out from the body.
 *
 * 4. Hip positions:
 *    hip_left  = joint[body_idx] + BODY_OFFSET * (cos left_normal,  sin left_normal)
 *    hip_right = joint[body_idx] + BODY_OFFSET * (cos right_normal, sin right_normal)
 *
 * 5. Gait phase for this leg pair:
 *    Left leg phase:  phi_L = wave_time * GAIT_FREQ + i * (π / N_LEGS)
 *    Right leg phase: phi_R = phi_L + π   (contralateral antiphase)
 *    The i*(π/N_LEGS) offset creates a travelling wave from front to rear.
 *
 * 6. Upper and lower leg angles (stateless FK):
 *    upper_angle_L = body_dir + LEG_SPLAY + SWING_AMP * sin(phi_L)
 *    upper_angle_R = body_dir - LEG_SPLAY + SWING_AMP * sin(phi_R)
 *    lower_angle_L = upper_angle_L + LEG_BEND + LOWER_SWING * sin(phi_L + π/4)
 *    lower_angle_R = upper_angle_R + LEG_BEND + LOWER_SWING * sin(phi_R + π/4)
 *
 * 7. Joint positions via forward kinematics:
 *    knee.x = hip.x + UPPER_LEN * cos(upper_angle)
 *    knee.y = hip.y + UPPER_LEN * sin(upper_angle)
 *    foot.x = knee.x + LOWER_LEN * cos(lower_angle)
 *    foot.y = knee.y + LOWER_LEN * sin(lower_angle)
 */
static void compute_legs(Centipede *c)
{
    float pi_f = (float)M_PI;

    for (int i = 0; i < N_LEGS; i++) {

        /* Step 1: body joint attachment index */
        int body_idx;
        if (N_LEGS > 1) {
            body_idx = 1 + i * (BODY_SEGS - 2) / (N_LEGS - 1);
        } else {
            body_idx = BODY_SEGS / 2;
        }
        /* Clamp to valid range [1, BODY_SEGS-1] */
        if (body_idx < 1)           body_idx = 1;
        if (body_idx > BODY_SEGS-1) body_idx = BODY_SEGS - 1;

        /* Step 2: body direction at this joint */
        /* Use neighbors to get a smooth direction even at low speed */
        int prev_idx = body_idx - 1;
        int next_idx = body_idx + 1;
        if (prev_idx < 0)          prev_idx = 0;
        if (next_idx > BODY_SEGS)  next_idx = BODY_SEGS;

        Vec2 fwd_vec = {
            c->joint[prev_idx].x - c->joint[next_idx].x,
            c->joint[prev_idx].y - c->joint[next_idx].y
        };
        fwd_vec = vec2_norm(fwd_vec);
        float body_dir = atan2f(fwd_vec.y, fwd_vec.x);

        /* Step 3: lateral normals (perpendicular to body direction) */
        float left_normal  = body_dir + pi_f * 0.5f;
        float right_normal = body_dir - pi_f * 0.5f;

        /* Step 4: hip positions */
        Vec2 hip_L = {
            c->joint[body_idx].x + BODY_OFFSET * cosf(left_normal),
            c->joint[body_idx].y + BODY_OFFSET * sinf(left_normal)
        };
        Vec2 hip_R = {
            c->joint[body_idx].x + BODY_OFFSET * cosf(right_normal),
            c->joint[body_idx].y + BODY_OFFSET * sinf(right_normal)
        };

        /* Step 5: gait phases — travelling wave from front to rear */
        float phi_L = c->wave_time * GAIT_FREQ + (float)i * (pi_f / (float)N_LEGS);
        float phi_R = phi_L + pi_f;   /* right leg: contralateral antiphase */

        /* Step 6: upper and lower leg angles (stateless FK) */
        float upper_L = body_dir + LEG_SPLAY + SWING_AMP * sinf(phi_L);
        float upper_R = body_dir - LEG_SPLAY + SWING_AMP * sinf(phi_R);
        float lower_L = upper_L + LEG_BEND + LOWER_SWING * sinf(phi_L + pi_f * 0.25f);
        float lower_R = upper_R + LEG_BEND + LOWER_SWING * sinf(phi_R + pi_f * 0.25f);

        /* Step 7: forward kinematics — compute knee and foot positions */
        Vec2 knee_L = {
            hip_L.x + UPPER_LEN * cosf(upper_L),
            hip_L.y + UPPER_LEN * sinf(upper_L)
        };
        Vec2 foot_L = {
            knee_L.x + LOWER_LEN * cosf(lower_L),
            knee_L.y + LOWER_LEN * sinf(lower_L)
        };
        Vec2 knee_R = {
            hip_R.x + UPPER_LEN * cosf(upper_R),
            hip_R.y + UPPER_LEN * sinf(upper_R)
        };
        Vec2 foot_R = {
            knee_R.x + LOWER_LEN * cosf(lower_R),
            knee_R.y + LOWER_LEN * sinf(lower_R)
        };

        /* Store: [0]=hip [1]=knee [2]=foot */
        c->leg_left[i][0]  = hip_L;
        c->leg_left[i][1]  = knee_L;
        c->leg_left[i][2]  = foot_L;
        c->leg_right[i][0] = hip_R;
        c->leg_right[i][1] = knee_R;
        c->leg_right[i][2] = foot_R;
    }
}

/* ── §5e  rendering helpers ─────────────────────────────────────────── */

/*
 * body_seg_pair() — ncurses color pair for body segment i.
 *
 * Maps segment index linearly across the N_PAIRS gradient:
 *   i = 0           → pair N_PAIRS (head color, most vivid — e.g. bright red)
 *   i = BODY_SEGS-1 → pair 1       (tail color, dimmest  — e.g. dark brown)
 *
 * Formula: p = N_PAIRS - round(i × (N_PAIRS-1) / (BODY_SEGS-1))
 * This is integer linear interpolation from N_PAIRS down to 1 as i goes
 * from 0 to BODY_SEGS-1.  Every N_PAIRS segments share the same color bucket.
 *
 * Clamping [1, N_PAIRS] guards against out-of-range i from callers that pass
 * BODY_SEGS (the tail-tip joint node marker at index BODY_SEGS).
 */
static int body_seg_pair(int i)
{
    /* i=0 → head = pair N_PAIRS; i=BODY_SEGS-1 → tail = pair 1 */
    int p = N_PAIRS - (i * (N_PAIRS - 1)) / (BODY_SEGS - 1);
    if (p < 1)       p = 1;
    if (p > N_PAIRS) p = N_PAIRS;
    return p;
}

/*
 * body_seg_attr() — ncurses text attribute for body segment i.
 *
 * Three zones create a visual depth gradient along the body:
 *   Head quarter  (i < BODY_SEGS/4)      : A_BOLD  — brightest, most attention
 *   Mid body      (BODY_SEGS/4 ≤ i ≤ 3/4): A_NORMAL — neutral
 *   Tail quarter  (i > 3×BODY_SEGS/4)    : A_DIM   — recedes visually
 *
 * Combined with the color gradient from body_seg_pair(), this produces a
 * convincing sense of the centipede tapering away into the distance.
 * A_BOLD and A_DIM are resolved by the terminal at render time — they typically
 * adjust brightness by ±1 stop on 256-color terminals.
 */
static attr_t body_seg_attr(int i)
{
    if (i < BODY_SEGS / 4)       return A_BOLD;
    if (i > 3 * BODY_SEGS / 4)   return A_DIM;
    return A_NORMAL;
}

/*
 * seg_glyph() — select the best-matching ASCII direction glyph for (dx, dy).
 *
 * The four ASCII line glyphs and their angular ranges (folded to [0°, 180°)):
 *   '-'    [  0°,  22.5°) ∪ [157.5°, 180°)  — near-horizontal
 *   '\'    [ 22.5°,  67.5°)                  — diagonal down-right (in term space)
 *   '|'    [ 67.5°, 112.5°)                  — near-vertical
 *   '/'    [112.5°, 157.5°)                  — diagonal down-left (in term space)
 *
 * Why fold to [0°, 180°)?  The glyphs are symmetric under 180° rotation
 * ('-' going right looks the same as '-' going left), so we only need half
 * the angular range.  Folding: if deg ≥ 180°, subtract 180°.
 *
 * dy negated (atan2f(-dy, dx)) converts terminal coordinates (y down) to
 * mathematical coordinates (y up) before computing the angle, so the glyph
 * matches visual direction rather than memory direction.
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <   0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;   /* fold to [0°, 180°) */

    if (deg < 22.5f || deg >= 157.5f) return (chtype)'-';
    if (deg < 67.5f)                   return (chtype)'\\';
    if (deg < 112.5f)                  return (chtype)'|';
    return                             (chtype)'/';
}

/*
 * head_glyph() — directional arrow for the centipede head.
 * Maps heading (radians) to '>' '<' '^' 'v'.
 */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg <  45.0f || deg >= 315.0f) return (chtype)'>';
    if (deg < 135.0f)                  return (chtype)'v';
    if (deg < 225.0f)                  return (chtype)'<';
    return                             (chtype)'^';
}

/*
 * draw_line_dense() — stamp a direction glyph at every terminal cell the
 * line from pixel point a to b passes through.
 *
 * WHY DENSE STEPPING?
 *   A terminal cell is CELL_W × CELL_H = 8 × 16 px.  If we drew only at joint
 *   positions (a and b), a segment DRAW_STEP_PX × 2 = 10 px long would leave a
 *   visible gap in the body.  By stepping every DRAW_STEP_PX = 5 px we guarantee
 *   at least one sample per cell the segment passes through (5 < CELL_W = 8).
 *
 * ALGORITHM:
 *   1. Compute direction vector (dx, dy) and segment length len.
 *   2. Choose glyph from seg_glyph(dx, dy) — maps angle to -\/|.
 *   3. Walk nsteps = ceil(len / DRAW_STEP_PX) + 1 sample points.
 *      At step t: u = t/nsteps; sample_pos = a + u × (b-a).
 *   4. Convert sample to cell (cx, cy) via px_to_cell_x/y.
 *   5. Deduplicate: if this (cx,cy) is the same as previous, skip.
 *      This prevents double-stamping at segment joints which would
 *      overwrite the bold joint-node marker placed by the second pass.
 *   6. Clamp: skip out-of-bounds cells (toroidal wrap edges can produce
 *      segments that partially exit the screen).
 *
 * DEDUPLICATION CURSOR (prev_cx, prev_cy):
 *   The caller passes these by pointer so the cursor persists across multiple
 *   draw_line_dense calls within a single pass.  Initialise to -9999 before
 *   the first call for a given pass to ensure the first cell always draws.
 *   For separate passes (e.g. body vs legs) use independent cursors.
 */
static void draw_line_dense(WINDOW *w,
                             Vec2 a, Vec2 b,
                             int pair, attr_t attr,
                             int cols, int rows,
                             int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* ── §5f  render_centipede ──────────────────────────────────────────── */

/*
 * render_centipede() — compose the complete centipede frame into WINDOW *w.
 *
 * DRAW ORDER (painter's algorithm — later draws win overlaps):
 *   Step 1 & 2: compute interpolated positions (no drawing)
 *   Step 3:     draw legs (deepest layer — behind body)
 *   Step 4a:    draw body segment lines tail→head
 *   Step 4b:    draw body joint node markers tail→head (on top of lines)
 *   Step 5:     draw head arrow (topmost — always visible)
 *
 * STEP 1 — alpha interpolation for body joints.
 *   rj[i] = prev_joint[i] + (joint[i] - prev_joint[i]) × alpha
 *   alpha ∈ [0,1) = fraction of the way from last physics tick to next tick.
 *   Result: motion is smooth at any render Hz; even at SIM_FPS=10 the body
 *   glides continuously because render can fire at 60 Hz between ticks.
 *
 * STEP 2 — alpha interpolation for leg positions.
 *   Same lerp formula applied to each leg's hip[0], knee[1], foot[2].
 *   Both left and right legs lerped independently.
 *
 * STEP 3 — draw legs (behind body).
 *   For each leg pair i:
 *     Left:  hip→knee drawn (pair 3, A_DIM), knee→foot drawn (pair 3, A_DIM).
 *     Right: hip→knee drawn (pair 4, A_DIM), knee→foot drawn (pair 4, A_DIM).
 *     Left and right use different pairs so they are visually distinguishable
 *     on the narrow strip where they overlap at the hip attachment.
 *     Foot tips: left='*', right='x' in pair 5 A_BOLD — bright tip markers.
 *   Each leg gets its own prev_cx/prev_cy dedup cursor (legs don't share cells).
 *
 * STEP 4a — body segment lines, drawn tail→head.
 *   A single shared dedup cursor (prev_cx, prev_cy) spans all segments.
 *   Tail-to-head order ensures the head-area glyphs overwrite tail glyphs where
 *   the body doubles back on itself during tight curves.
 *
 * STEP 4b — body joint node markers (second pass, on top of lines).
 *   Shape encodes position along body:
 *     j == BODY_SEGS  → '*'  (tail tip — rearmost point)
 *     j > 3/4 body    → '.'  (near tail — thin dot)
 *     j > 1/4 body    → 'o'  (mid body — medium dot)
 *     j > 0           → 'O'  (near head — thick dot)
 *     j == 0          → '@'  (head joint — overwritten by arrow in step 5)
 *   All rendered A_BOLD so markers dominate the underlying '-\/|' line glyphs.
 *
 * STEP 5 — head arrow (topmost layer).
 *   head_glyph(heading) → one of '>' 'v' '<' '^'.
 *   Drawn last in pair 7 A_BOLD so it is never obscured.
 */
static void render_centipede(const Centipede *c, WINDOW *w,
                              int cols, int rows, float alpha)
{
    /* Step 1 — interpolated body joint positions */
    Vec2 rj[BODY_SEGS + 1];
    for (int i = 0; i <= BODY_SEGS; i++) {
        rj[i].x = c->prev_joint[i].x + (c->joint[i].x - c->prev_joint[i].x) * alpha;
        rj[i].y = c->prev_joint[i].y + (c->joint[i].y - c->prev_joint[i].y) * alpha;
    }

    /* Step 2 — interpolated leg positions */
    Vec2 rl_left [N_LEGS][3];
    Vec2 rl_right[N_LEGS][3];
    for (int i = 0; i < N_LEGS; i++) {
        for (int j = 0; j < 3; j++) {
            rl_left[i][j].x = c->prev_leg_left[i][j].x
                            + (c->leg_left[i][j].x - c->prev_leg_left[i][j].x) * alpha;
            rl_left[i][j].y = c->prev_leg_left[i][j].y
                            + (c->leg_left[i][j].y - c->prev_leg_left[i][j].y) * alpha;

            rl_right[i][j].x = c->prev_leg_right[i][j].x
                             + (c->leg_right[i][j].x - c->prev_leg_right[i][j].x) * alpha;
            rl_right[i][j].y = c->prev_leg_right[i][j].y
                             + (c->leg_right[i][j].y - c->prev_leg_right[i][j].y) * alpha;
        }
    }

    /* Step 3 — draw legs (behind body; use independent cursors per leg segment) */
    for (int i = 0; i < N_LEGS; i++) {

        /* Left leg: pair 3 (yellow-green), A_DIM */
        int lcx = -9999, lcy = -9999;
        draw_line_dense(w, rl_left[i][0], rl_left[i][1],
                        3, A_DIM, cols, rows, &lcx, &lcy);
        draw_line_dense(w, rl_left[i][1], rl_left[i][2],
                        3, A_DIM, cols, rows, &lcx, &lcy);

        /* Left foot tip marker */
        int ftx = px_to_cell_x(rl_left[i][2].x);
        int fty = px_to_cell_y(rl_left[i][2].y);
        if (ftx >= 0 && ftx < cols && fty >= 0 && fty < rows) {
            wattron(w, COLOR_PAIR(5) | A_BOLD);
            mvwaddch(w, fty, ftx, (chtype)'*');
            wattroff(w, COLOR_PAIR(5) | A_BOLD);
        }

        /* Right leg: pair 4 (yellow), A_DIM */
        int rcx = -9999, rcy = -9999;
        draw_line_dense(w, rl_right[i][0], rl_right[i][1],
                        4, A_DIM, cols, rows, &rcx, &rcy);
        draw_line_dense(w, rl_right[i][1], rl_right[i][2],
                        4, A_DIM, cols, rows, &rcx, &rcy);

        /* Right foot tip marker */
        int frx = px_to_cell_x(rl_right[i][2].x);
        int fry = px_to_cell_y(rl_right[i][2].y);
        if (frx >= 0 && frx < cols && fry >= 0 && fry < rows) {
            wattron(w, COLOR_PAIR(5) | A_BOLD);
            mvwaddch(w, fry, frx, (chtype)'x');
            wattroff(w, COLOR_PAIR(5) | A_BOLD);
        }
    }

    /* Step 4 — body segments, drawn tail→head so head wins overlaps */
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = BODY_SEGS - 1; i >= 0; i--) {
        draw_line_dense(w,
                        rj[i + 1], rj[i],   /* rear → front */
                        body_seg_pair(i), body_seg_attr(i),
                        cols, rows,
                        &prev_cx, &prev_cy);
    }

    /* Step 4b — body joint node markers (drawn on top of segment lines).
     *
     * Same two-pass pattern as fk_tentacle_forest: segment lines first,
     * then bold node glyphs at every joint position so the chain reads as
     * distinct beads connected by spaced direction chars.
     *
     * Node shape varies with distance from head (j=0=head end):
     *   j == 0          '@'  head marker (overwritten by head arrow below)
     *   j < BODY/4      'O'  thick near-head nodes
     *   j < 3*BODY/4    'o'  mid-body nodes
     *   j < BODY_SEGS   '.'  thin near-tail nodes
     *   j == BODY_SEGS  '*'  tail tip
     * All rendered A_BOLD so they dominate the underlying line glyphs.
     */
    for (int j = BODY_SEGS; j >= 0; j--) {   /* tail → head, head drawn last */
        int cx = px_to_cell_x(rj[j].x);
        int cy = px_to_cell_y(rj[j].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        int    p = body_seg_pair(j < BODY_SEGS ? j : BODY_SEGS - 1);
        chtype marker;
        if      (j == BODY_SEGS)           marker = (chtype)'*'; /* tail tip   */
        else if (j > 3 * BODY_SEGS / 4)   marker = (chtype)'.'; /* near tail  */
        else if (j > BODY_SEGS / 4)        marker = (chtype)'o'; /* mid body   */
        else if (j > 0)                    marker = (chtype)'O'; /* near head  */
        else                               marker = (chtype)'@'; /* head joint */

        wattron(w, COLOR_PAIR(p) | A_BOLD);
        mvwaddch(w, cy, cx, marker);
        wattroff(w, COLOR_PAIR(p) | A_BOLD);
    }

    /* Step 5 — head arrow always on top */
    int hx = px_to_cell_x(rj[0].x);
    int hy = px_to_cell_y(rj[0].y);
    if (hx >= 0 && hx < cols && hy >= 0 && hy < rows) {
        wattron(w, COLOR_PAIR(7) | A_BOLD);
        mvwaddch(w, hy, hx, head_glyph(c->heading));
        wattroff(w, COLOR_PAIR(7) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Centipede centipede; } Scene;

/*
 * scene_init() — initialise centipede to a clean, immediately-animated state.
 *
 * STARTING POSITION: head at 38% from left, vertically centred.
 *   Off-centre so the initial undulation curve immediately fills the screen
 *   — centred placement would show the body off-screen on the first turn.
 *
 * STARTING HEADING: π/8 (22.5° south-east) for immediate visual interest.
 *   A zero heading (east) starts mid-cycle with no curve visible; π/8
 *   enters the screen at a slight downward angle, making the first curve
 *   appear within 1–2 seconds.
 *
 * WAVE_TIME = π/2: sin(π/2) = 1.0, the peak of the sine wave.
 *   The turn integrator begins at peak rate, so the head immediately starts
 *   curving.  Starting at 0 would give a straight-line first half-second.
 *
 * TRAIL PRE-POPULATION:
 *   Without pre-fill the centipede would have zero trail entries and
 *   trail_sample() would return the head position for all joints — the body
 *   would appear as a single point and grow tail-first over ~7 s (480 px at
 *   65 px/s).  Pre-filling TRAIL_CAP entries 1 px apart behind the starting
 *   heading gives a fully-extended body from frame 1.
 *
 *   TRAIL_CAP = 4096 >> 480 px body → no risk of overwriting fresh entries.
 *
 * COPY TO PREV ARRAYS:
 *   memcpy(prev_joint, joint) makes the very first alpha lerp a no-op
 *   (prev == curr → lerped = curr at any alpha).  Without this copy, the
 *   first frame would interpolate from zero-initialized prev to the real
 *   position, creating a one-frame "flash" of the body at the origin.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Centipede *c   = &sc->centipede;
    c->move_speed  = MOVE_SPEED;
    c->turn_amp    = TURN_AMP;
    c->turn_freq   = TURN_FREQ;
    c->theme_idx   = 0;
    c->paused      = false;

    /* Start at peak of wave so curves are visible immediately */
    c->wave_time = (float)M_PI * 0.5f;
    c->heading   = (float)M_PI / 8.0f;

    /* Head position */
    c->joint[0].x = (float)(cols * CELL_W) * 0.38f;
    c->joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /* Pre-populate trail: 1 px steps backward from head */
    float bx = cosf(c->heading + (float)M_PI);
    float by = sinf(c->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        c->trail[k].x = c->joint[0].x + (float)k * bx;
        c->trail[k].y = c->joint[0].y + (float)k * by;
    }
    c->trail_head  = 0;
    c->trail_count = TRAIL_CAP;

    /* Initial body and leg placement */
    compute_joints(c);
    compute_legs(c);

    /* Copy to prev_ arrays so first frame alpha lerp is a no-op */
    memcpy(c->prev_joint,     c->joint,     sizeof c->joint);
    memcpy(c->prev_leg_left,  c->leg_left,  sizeof c->leg_left);
    memcpy(c->prev_leg_right, c->leg_right, sizeof c->leg_right);
}

/*
 * scene_tick() — one fixed-step physics update.
 *
 * ORDER:
 *   1. Save prev_ arrays (interpolation anchors must be set BEFORE physics).
 *   2. Return early if paused (prev saved, so freeze is clean).
 *   3. move_head() — advance wave_time, heading, position, push trail.
 *   4. compute_joints() — place body joints from updated trail.
 *   5. compute_legs() — stateless FK for all leg pairs.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Centipede *c = &sc->centipede;

    /* Step 1: snapshot previous state for alpha lerp */
    memcpy(c->prev_joint,     c->joint,     sizeof c->joint);
    memcpy(c->prev_leg_left,  c->leg_left,  sizeof c->leg_left);
    memcpy(c->prev_leg_right, c->leg_right, sizeof c->leg_right);

    /* Step 2: skip physics if paused */
    if (c->paused) return;

    /* Step 3-5: physics update */
    move_head(c, dt, cols, rows);
    compute_joints(c);
    compute_legs(c);
}

/*
 * scene_draw() — render centipede; called once per render frame.
 * alpha ∈ [0,1) is the sub-tick interpolation factor.
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_centipede(&sc->centipede, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — ncurses display layer.
 * Holds current terminal dimensions, updated after each resize.
 */
typedef struct { int cols, rows; } Screen;

/*
 * screen_init() — configure terminal for animation.
 *   initscr()        initialise ncurses.
 *   noecho()         suppress key echo.
 *   cbreak()         immediate key delivery.
 *   curs_set(0)      hide hardware cursor.
 *   nodelay(TRUE)    non-blocking getch() — never stalls the render loop.
 *   keypad(TRUE)     decode arrow keys to KEY_* constants.
 *   typeahead(-1)    disable mid-output read() calls that can tear frames.
 */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(0);   /* theme 0 = Amber; re-applied when user presses 't' */
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_free() — restore terminal to pre-animation state.
 */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH resize event.
 * endwin() + refresh() forces ncurses to re-read LINES/COLS from the kernel.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose the full frame into stdscr.
 *
 * Frame composition order:
 *   1. erase()        clear newscr (no terminal I/O yet).
 *   2. scene_draw()   draw centipede body and legs.
 *   3. HUD top bar    status information (on top of any body glyph).
 *   4. Hint bar       key reference at bottom row.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Centipede *c = &sc->centipede;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %dHz  spd:%.0f  amp:%.2f  freq:%.2f  [%s]  %s ",
             fps, sim_fps,
             c->move_speed, c->turn_amp, c->turn_freq,
             THEMES[c->theme_idx].name,
             c->paused ? "PAUSED" : "crawling");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  ^v:spd  a/d:freq  w/s:amp  t:theme  [/]:Hz ");
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

/*
 * screen_present() — flush composed frame to terminal.
 * wnoutrefresh() copies newscr to pending update; doupdate() diffs and sends
 * only changed cells — the ncurses double-buffer protocol.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state; accessible from signal handlers.
 * Declared as file-scope global so signal handlers (no user argument) can
 * write running/need_resize flags.
 *
 * volatile sig_atomic_t: volatile prevents register-caching across handler
 * writes; sig_atomic_t guarantees atomic read/write on any POSIX platform.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls here */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 * Ensures endwin() is called even on unhandled signal exit paths.
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle pending SIGWINCH.
 * Re-reads terminal dimensions.  Clamps head pixel position to new bounds
 * so the centipede doesn't teleport — it continues from wherever it was.
 * Resets frame_time and sim_accum in the caller to prevent physics avalanche
 * from the large dt that accumulated during the resize.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Centipede *c = &app->scene.centipede;
    float wpx    = (float)(app->screen.cols * CELL_W);
    float hpx    = (float)(app->screen.rows * CELL_H);
    if (c->joint[0].x >= wpx) c->joint[0].x = wpx - 1.0f;
    if (c->joint[0].y >= hpx) c->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC        quit
 *   space              toggle pause
 *   ↑  / ↓             move_speed × / ÷ 1.25
 *   ← / → and a / d    turn_freq  − / + 0.15  (slower / faster undulation)
 *   w / s              turn_amp   + / − 0.10   (wider / narrower curves)
 *   [ / ]              sim_fps    − / + SIM_FPS_STEP
 *
 * Arrow keys and letter keys are aliases.  Letters are provided because
 * some terminals swallow arrow escape sequences, and to be consistent with
 * fk_tentacle_forest.c (a/d = freq there too).
 * Step sizes are generous so one keypress gives an immediately visible change.
 */
static bool app_handle_key(App *app, int ch)
{
    Centipede *c = &app->scene.centipede;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': c->paused = !c->paused; break;

    /* Cycle color theme — wraps around all N_THEMES themes */
    case 't': case 'T':
        c->theme_idx = (c->theme_idx + 1) % N_THEMES;
        theme_apply(c->theme_idx);
        break;

    /* Move speed — ^/v (arrow up/down), 25% per press */
    case KEY_UP:
        c->move_speed *= 1.25f;
        if (c->move_speed > MOVE_SPEED_MAX) c->move_speed = MOVE_SPEED_MAX;
        break;
    case KEY_DOWN:
        c->move_speed /= 1.25f;
        if (c->move_speed < MOVE_SPEED_MIN) c->move_speed = MOVE_SPEED_MIN;
        break;

    /* Undulation frequency — ←/→ and a/d */
    case KEY_LEFT:  case 'a': case 'A':
        c->turn_freq -= 0.15f;
        if (c->turn_freq < TURN_FREQ_MIN) c->turn_freq = TURN_FREQ_MIN;
        break;
    case KEY_RIGHT: case 'd': case 'D':
        c->turn_freq += 0.15f;
        if (c->turn_freq > TURN_FREQ_MAX) c->turn_freq = TURN_FREQ_MAX;
        break;

    /* Swim amplitude — w/s */
    case 'w': case 'W':
        c->turn_amp += 0.10f;
        if (c->turn_amp > TURN_AMP_MAX) c->turn_amp = TURN_AMP_MAX;
        break;
    case 's': case 'S':
        c->turn_amp -= 0.10f;
        if (c->turn_amp < TURN_AMP_MIN) c->turn_amp = TURN_AMP_MIN;
        break;

    /* Simulation Hz */
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

/*
 * main() — the game loop (structure identical to framework.c §8)
 *
 * Each frame:
 *  ① RESIZE CHECK    — handle pending SIGWINCH before touching ncurses.
 *  ② MEASURE dt      — wall-clock ns since last frame; capped at 100ms.
 *  ③ ACCUMULATOR     — fire scene_tick() once per sim_fps^-1 ns of dt.
 *  ④ ALPHA           — fractional leftover → sub-tick interpolation factor.
 *  ⑤ FPS COUNTER     — smoothed fps over a 500ms window.
 *  ⑥ FRAME CAP       — sleep budget − elapsed (before render, not after).
 *  ⑦ DRAW + PRESENT  — erase → draw → wnoutrefresh → doupdate.
 *  ⑧ DRAIN INPUT     — loop getch() until ERR; process every queued key.
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

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── ① resize ─────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② dt ─────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha — sub-tick interpolation factor ─────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ──────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
    }

    screen_free(&app->screen);
    return 0;
}
