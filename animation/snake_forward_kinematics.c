/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snake_forward_kinematics.c — a snake that swims itself around the terminal.
 *
 * Forward kinematics: you drive the head, and each body segment just follows
 * the one ahead of it (staying a fixed distance behind), so the whole snake
 * trails along the head's path. Here the head steers itself with a sine-wave
 * wiggle and curves away from the edges, so it never needs a driver.
 * Sister file: snake_inverse_kinematics.c (same body, but the head chases a
 * moving target instead of wandering).
 *
 * Build (needs -lm for the trig in the steering and trail math):
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       snake_forward_kinematics.c -o snake_fk -lncurses -lm
 */


#define _POSIX_C_SOURCE 200809L

/* M_PI isn't standard C, only POSIX. Define it ourselves if the toolchain
 * doesn't, so the build never fails. */
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

/* ── §1 config — every tunable number lives here ── */

/* Edit behaviour from this block only; no literals scattered in the code. */
enum {
    /* Physics tick rate (Hz). The §8 loop runs scene_tick() this many times
     * per second no matter the render rate; higher = smoother + more CPU.
     * Default 60 matches the 60-fps render cap (one tick per drawn frame). */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,   /* how much [/] keys nudge it              */

    /* HUD_COLS: max length of the status bar string (snprintf truncates).
     * FPS_UPDATE_MS: refresh the shown fps number this often, so it reads
     * steadily instead of flickering every frame. */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /* N_PAIRS: the body uses 7 colour pairs, head (warm) to tail (cool).
     * PAIR_HUD / PAIR_HINT: separate fixed pairs for the status and key-hint
     * bars; they never change when the body theme cycles, so the text on top
     * stays readable against any colour. N_THEMES: how many themes exist. */
    N_PAIRS          =   7,
    PAIR_HUD         =   8,   /* bright yellow — top status bar  */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hint */
    N_THEMES         =  10,

    /* Number of body segments. More segments = a finer, smoother curve.
     * 32 segments × 18 px = a 576 px body, about 72 columns wide. */
    N_SEGS           =  32,

    /* How many past head positions the trail remembers (the ring buffer).
     * The body samples this trail, so it must be long enough to cover the
     * whole 576 px body even at the slowest crawl. At 20 px/s and 60 Hz the
     * head moves ~0.33 px/tick, so ~1745 entries cover the body; 4096 is a
     * comfortable margin. 4096 × 8 bytes = 32 KB, kept in a global (BSS). */
    TRAIL_CAP        = 4096,
};

/* SEG_LEN_PX: length of one body segment in pixels (~2.25 columns wide).
 * DRAW_STEP_PX: when draw_segment_beads() draws a segment it walks along it
 *   in steps this big, stamping a glyph at each. Must stay smaller than a
 *   cell (CELL_W = 8 px) so the line never skips a cell and leaves a gap. */
#define SEG_LEN_PX    18.0f
#define DRAW_STEP_PX   3.0f

/* How fast the head travels (px/s). 72 is a calm watchable pace; the up/down
 * keys scale it within [MIN, MAX]. */
#define MOVE_SPEED_DEFAULT   72.0f   /* px/s                               */
#define MOVE_SPEED_MIN       20.0f   /* slowest crawl                      */
#define MOVE_SPEED_MAX      500.0f   /* sprint — hard to follow visually   */

/* The wiggle that drives the snake. The head's turning rate follows a sine
 * wave: it steers left, then right, then left... so the path comes out as a
 * smooth S-curve. amplitude sets how hard it turns (curve sharpness);
 * frequency sets how fast it alternates left/right (curve tightness). The
 * defaults give a gentle ~31° sway that never doubles back on itself.
 * speed_scale is just a time knob: 2 runs the same wave twice as fast. */
#define AMPLITUDE_DEFAULT    0.52f   /* peak turn rate (rad/s)             */
#define AMPLITUDE_MIN        0.0f    /* 0 → perfectly straight swim        */
#define AMPLITUDE_MAX        4.0f    /* > 2 creates tight spirals          */
#define FREQUENCY_DEFAULT    0.95f   /* how fast it sways left/right (rad/s)*/
#define FREQUENCY_MIN        0.10f   /* very long lazy curves              */
#define FREQUENCY_MAX        6.00f   /* extremely rapid zigzag             */
#define SPEED_SCALE_DEFAULT  1.0f    /* wave time multiplier (+/- keys)    */

/* The soft fence that keeps the snake on-screen (no wrapping). Within
 * EDGE_MARGIN_PX of an edge, the head feels a nudge to turn inward; the
 * closer to the wall, the stronger the nudge. Outside that band there's no
 * effect, so the snake swims freely in the middle. EDGE_TURN_GAIN scales how
 * hard that nudge is — strong enough to curve away before reaching the wall,
 * gentle enough to look like a deliberate turn, not a bounce. */
#define EDGE_MARGIN_PX     80.0f
#define EDGE_TURN_GAIN      3.5f

/* Time units. TICK_NS(f) gives the nanoseconds per tick at f Hz, e.g.
 * TICK_NS(60) ≈ 16.67 ms. Used by the §8 fixed-step loop. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Terminal cells aren't square — they're about twice as tall as wide. We
 * treat each cell as CELL_W × CELL_H "pixels" and do all the snake math in
 * that square pixel space, so a circle stays a circle. §4 has the details. */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ── §2 clock — monotonic timer and sleep ── */

/* Current time in nanoseconds, from a clock that never jumps backward (so
 * subtracting two readings always gives a sane elapsed time). int64 keeps
 * the differences signed. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep for ns nanoseconds (skip if already over budget). §8 calls this to
 * cap the frame rate; sleeping before the terminal write keeps the cap from
 * counting I/O time. */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — head-to-tail gradient themes + fixed HUD colours ── */

/*
 * Theme — one named colour scheme for the snake body.
 *   name    : shown in the HUD (e.g. "Ocean").
 *   body[7] : the 7 colours (xterm-256 numbers) running from head (slot 0,
 *             brightest, warm) to tail (slot 6, dimmest, cool). seg_pair()
 *             maps each body segment onto one of these 7.
 * The head-to-tail brightness fade is the cue for which end is the head.
 * Every colour stays in the bright half of the palette on purpose: the tail
 * is drawn dimmed, and dark colours would vanish entirely on black terminals.
 * The HUD/hint bars use their own fixed colours, not these, so the text on
 * top stays readable whatever theme is active.
 */
typedef struct {
    const char *name;            /* shown in the HUD                  */
    int         body[N_PAIRS];   /* head→tail colours; colour pair p  *
                                  * gets body[p-1]                    */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      head ←─────────────────────→ tail */
    {"Solar",  {226, 220, 214, 208, 202, 196, 160}},
    {"Matrix", { 28,  34,  40,  76,  46,  82, 118}},
    {"Ocean",  { 24,  25,  31,  33,  39,  45,  51}},
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}},
    {"Nova",   { 54,  55,  56,  57,  93, 129, 165}},
    {"Medusa", { 57,  63,  93,  99, 105, 111, 159}},
    {"Lava",   { 52,  88, 124, 160, 196, 202, 208}},
    {"Ghost",  {244, 245, 247, 249, 251, 253, 255}},
    {"Aurora", { 28,  34,  64,  71,  78, 121, 159}},
    {"Neon",   {201, 165, 129,  93,  57,  51,  45}},
};

/* Load theme idx's 7 colours into ncurses pairs 1..7. Background is -1
 * (the terminal's own). On 8-colour terminals there's no 256-colour palette,
 * so we fall back to a fixed yellow→green→cyan→blue ramp. */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_PAIRS] = {
            COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN,
            COLOR_GREEN,  COLOR_CYAN,   COLOR_CYAN, COLOR_BLUE
        };
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/* One-time colour setup: turn on colour, let -1 mean "terminal default
 * background", load the starting theme, and bind the two fixed HUD colours
 * (bright yellow for the status bar, bright cyan for the key hints). */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ── §4 coords — convert pixel space to terminal cells ── */

/*
 * The snake math runs in square "pixel" space (1 unit is the same distance
 * in x and y) so curves don't come out squashed by the tall cell shape.
 * These two helpers are the only place that converts back to cells, at draw
 * time. They round to the nearest cell with floor(px/dim + 0.5): plain
 * truncation would bias one way, and roundf()'s banker's rounding can flip
 * back and forth right on a cell boundary and cause a one-cell flicker.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the Snake: trail buffer, body placement, drawing ── */

/*
 * Vec2 — a 2-D point in pixel space (the square space from §4).
 *   x : pixels right of the left edge (positive → right).
 *   y : pixels down from the top      (positive → down).
 * Note y grows downward, so a heading of π/2 (90°) points DOWN the screen,
 * not up. Everything in §5 — head, trail, joints, forces — is a Vec2; the
 * conversion to cells happens only at draw time (§5d). Passed around by
 * value: it's just 8 bytes, and the optimiser inlines the small helpers.
 */
typedef struct {
    float x;   /* pixels right of left edge (positive → right) */
    float y;   /* pixels down from top      (positive → down)  */
} Vec2;

/*
 * Snake — everything about the creature, in one record. It has four parts:
 *
 *   1. The trail: a record of where the head has recently been. This is the
 *      whole trick of forward kinematics here — the body has no physics of
 *      its own, it just remembers the head's path and follows it.
 *   2. The joints: joint[0] is the head (move_head drives it), and the rest
 *      are placed onto the trail by compute_joints. prev_joint is last tick's
 *      pose, kept so drawing can smoothly blend between frames.
 *   3. The wave: the self-steering wiggle (wave_time, amplitude, frequency,
 *      speed_scale) that makes the head sway side to side on its own.
 *   4. Steering: heading (which way the head currently points) and move_speed
 *      (how fast it goes).
 *
 * Each tick runs straight down that list: advance the wave, steer the head,
 * record it in the trail, then place the body from the trail. No feedback.
 *
 * Two design points worth knowing:
 *   - trail is a ring buffer (writes wrap around and overwrite the oldest)
 *     so pushing a new head position is O(1), never a big array shift.
 *   - joint and prev_joint are kept separate: prev_joint is frozen at the
 *     start of the tick so drawing can interpolate prev → current and stay
 *     smooth even when the draw rate and physics rate differ. Merging them
 *     would make the blend overshoot.
 */
typedef struct {
    /* The trail: a ring buffer of past head positions. The body samples it
     * to find where to sit. trail[trail_head] is the newest entry. */
    Vec2 trail[TRAIL_CAP];
    int  trail_head;                  /* index of newest entry (wraps mod CAP)*/
    int  trail_count;                 /* how many slots are filled (caps at CAP)*/

    /* The body. joint[0] is the head; joint[1..N_SEGS] are the rest.
     * prev_joint is the same poses one tick ago, the anchor for the smooth
     * draw-time blend. */
    Vec2 joint     [N_SEGS + 1];
    Vec2 prev_joint[N_SEGS + 1];

    /* Steering. heading: current facing in radians (0 = right, π/2 = down,
     * since y points down). move_speed: head speed in px/s, constant. */
    float heading;
    float move_speed;

    /* The self-steering wave. wave_time is its own clock so speed_scale can
     * speed the wiggle up or down without changing how fast the snake moves.
     * The head's turn rate is amplitude · sin(frequency · wave_time). */
    float wave_time;                  /* wave clock (seconds · speed_scale)  */
    float amplitude;                  /* how hard it turns (rad/s)           */
    float frequency;                  /* how fast it sways left/right (rad/s)*/
    float speed_scale;                /* wave-clock speed multiplier         */

    /* Controls. paused freezes the physics (drawing keeps running);
     * theme_idx picks the §3 colour scheme. */
    int   theme_idx;                  /* index into THEMES[]; t/T cycles     */
    bool  paused;
} Snake;

/* ── §5a trail helpers — record the head's path, sample points along it ── */

/* Record one new head position. trail_head moves forward (wrapping around),
 * and once the buffer is full this overwrites the oldest entry — fine,
 * because the body never needs history older than its own length. */
static void trail_push(Snake *s, Vec2 pos)
{
    s->trail_head = (s->trail_head + 1) % TRAIL_CAP;
    s->trail[s->trail_head] = pos;
    if (s->trail_count < TRAIL_CAP) s->trail_count++;
}

/* Return the trail entry k steps back from the newest (k=0 is the current
 * head, k=1 one tick older, and so on). The "+ TRAIL_CAP" before subtracting
 * keeps the index positive — plain (head - k) % CAP can go negative in C and
 * give a wrong slot. Caller must keep k < trail_count. */
static inline Vec2 trail_at(const Snake *s, int k)
{
    return s->trail[(s->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/* Straight-line distance between two trail points. The trail is a chain of
 * straight segments, and summing these lengths walks along it. */
static inline float polyline_segment_length(Vec2 a, Vec2 b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

/* Point a fraction t of the way from a to b (t=0 → a, t=1 → b). Lets a joint
 * land at the exact distance asked for, not just at the nearest stored point. */
static inline Vec2 lerp_between_points(Vec2 a, Vec2 b, float t)
{
    return (Vec2){
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
    };
}

/*
 * The heart of the whole effect: find the point on the head's trail that is
 * exactly `dist` pixels behind the head. Walk the trail from newest to
 * oldest adding up segment lengths; once the running total reaches dist,
 * we've found the segment that contains the answer, so interpolate inside
 * it for the exact spot. compute_joints calls this once per body joint with
 * dist = i · SEG_LEN_PX, which spaces the joints evenly along the path no
 * matter how fast the head was moving when it laid the trail down.
 * The "seg > 1e-4f" guard avoids a divide-by-zero if two trail points happen
 * to land on the same spot. If the trail runs out, return its oldest point.
 */
static Vec2 trail_sample(const Snake *s, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(s, 0);              /* newest = current head */

    for (int k = 1; k < s->trail_count; k++) {
        Vec2  b   = trail_at(s, k);            /* one tick older than a */
        float seg = polyline_segment_length(a, b);

        if (accum + seg >= dist) {             /* target inside [a, b] */
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return lerp_between_points(a, b, t);
        }

        accum += seg;                          /* keep walking back     */
        a      = b;
    }

    return trail_at(s, s->trail_count - 1);    /* trail exhausted       */
}

/* ── §5b move_head — steer the head and record it in the trail ── */

/* Fold an angle into (−π, π]. Used so an angle difference becomes the
 * short way around the circle, not 350° when it should be −10°. */
static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/*
 * Build a "push me inward" vector from how close the head is to each wall.
 * For any wall within EDGE_MARGIN_PX, add a push pointing away from it that
 * ramps from 0 at the edge of the margin band up to 1 right at the wall.
 * The result points toward open space; it's zero in the screen interior and
 * grows toward a corner where two walls both push. edge_bias_turn turns this
 * vector into an actual steering rate. The linear ramp (rather than a hard
 * cutoff) is what makes the snake curve away smoothly instead of bouncing.
 */
static Vec2 inward_repulsion_from_walls(Vec2 head, float wpx, float hpx)
{
    float margin = EDGE_MARGIN_PX;
    Vec2  force  = { 0.0f, 0.0f };

    if (head.x < margin)         force.x += (margin - head.x)         / margin;
    if (head.x > wpx - margin)   force.x -= (head.x - (wpx - margin)) / margin;
    if (head.y < margin)         force.y += (margin - head.y)         / margin;
    if (head.y > hpx - margin)   force.y -= (head.y - (hpx - margin)) / margin;

    return force;
}

/*
 * How much extra turning the wall-avoidance wants this tick (rad/s), to be
 * added to the wave's turning. Get the inward-push vector; if it's zero
 * (head is clear of every wall) there's nothing to do. Otherwise figure out
 * the direction it wants (atan2), take the short-way difference from the
 * current heading (wrap_pi), and scale by how strong the push is. Returns 0
 * in the screen interior, so the wiggle is untouched there. */
static float edge_bias_turn(const Snake *s, float wpx, float hpx)
{
    Vec2 force = inward_repulsion_from_walls(s->joint[0], wpx, hpx);
    if (force.x == 0.0f && force.y == 0.0f) return 0.0f;

    float strength = sqrtf(force.x * force.x + force.y * force.y);
    float desired  = atan2f(force.y, force.x);
    float delta    = wrap_pi(desired - s->heading);
    return delta * strength * EDGE_TURN_GAIN;
}

/* Advance the wave's own clock. speed_scale lets the +/- keys run the wiggle
 * faster or slower without changing the snake's travel speed. Never
 * wrapped — sin() is happy with an ever-growing argument. */
static inline void advance_wave_clock(Snake *s, float dt)
{
    s->wave_time += dt * s->speed_scale;
}

/* The self-steering wiggle, as a turn rate (rad/s): amplitude controls how
 * hard the head turns, frequency how fast it alternates left and right. This
 * sine is the whole reason the snake wanders on its own. */
static inline float cpg_turn_rate(const Snake *s)
{
    return s->amplitude * sinf(s->frequency * s->wave_time);
}

/* Apply one step of motion: turn the heading by omega·dt first, then step
 * the head forward along the new heading. (Turn-then-move; the snake has no
 * acceleration, so this simple integration is always stable.) */
static inline void integrate_heading_then_position(Snake *s, float omega,
                                                   float dt)
{
    s->heading    += omega * dt;
    s->joint[0].x += s->move_speed * cosf(s->heading) * dt;
    s->joint[0].y += s->move_speed * sinf(s->heading) * dt;
}

/* Last-resort clamp keeping the head on screen. The edge bias normally does
 * this gently, but one giant time step (e.g. after the program was paused in
 * a debugger) could overshoot a wall before the bias reacts. */
static inline void clamp_head_to_pixel_bounds(Snake *s, float wpx, float hpx)
{
    if (s->joint[0].x < 0.0f) s->joint[0].x = 0.0f;
    if (s->joint[0].x > wpx)  s->joint[0].x = wpx;
    if (s->joint[0].y < 0.0f) s->joint[0].y = 0.0f;
    if (s->joint[0].y > hpx)  s->joint[0].y = hpx;
}

/*
 * Move the head one tick. Tick the wave clock, add up the two turn rates
 * (the wiggle plus any wall-avoidance), turn and step the head, clamp it on
 * screen, and record the new spot in the trail (compute_joints reads that
 * trail right after). heading deliberately keeps growing past ±π; sin/cos
 * don't care, and wrapping it would cause a one-frame jerk.
 */
static void move_head(Snake *s, float dt, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);

    advance_wave_clock(s, dt);

    float omega = cpg_turn_rate(s) + edge_bias_turn(s, wpx, hpx);
    integrate_heading_then_position(s, omega, dt);

    clamp_head_to_pixel_bounds(s, wpx, hpx);
    trail_push(s, s->joint[0]);
}

/* ── §5c compute_joints — place the body onto the head's trail ── */

/*
 * Place every body joint. joint[0] (the head) is already set. Joint i sits
 * i · SEG_LEN_PX of path-distance behind the head — i.e. exactly where the
 * head was a moment ago — so the body retraces the head's path and bends
 * wherever the head bent. No angles or matrices: the trail already holds the
 * shape, we just read points off it at even spacings.
 */
static void compute_joints(Snake *s)
{
    for (int i = 1; i <= N_SEGS; i++) {
        s->joint[i] = trail_sample(s, (float)i * SEG_LEN_PX);
    }
}

/* ── §5d rendering helpers — colours and glyphs for one segment ── */

/* Pick the colour pair for body segment i, spreading the 7 theme colours
 * evenly from head (pair 1) to tail (pair 7). The fade is how you tell which
 * end is the head. */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/* Pick the brightness for body segment i: bold for the front quarter (draws
 * the eye to the head), dim for the back quarter (fades the tail away),
 * normal in between. */
static attr_t seg_attr(int i)
{
    if (i < N_SEGS / 4)       return A_BOLD;
    if (i > 3 * N_SEGS / 4)   return A_DIM;
    return A_NORMAL;
}

/* Pick the bead glyph for joint i: bigger near the head ('0'), shrinking to
 * a dot ('.') near the tail, so the body visibly tapers. */
static chtype joint_node_char(int i)
{
    if (i <= (N_SEGS - 1) / 3)     return '0';
    if (i >= (N_SEGS - 1) * 2 / 3) return '.';
    return 'o';
}

/* Pick an arrow showing which way the head points. Heading is rounded to one
 * of four directions; '>' right, 'v' down, '<' left, '^' up (remember down is
 * +y here). The while-loops fold the angle into 0–360° (fmod can go negative). */
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

/* ── §5e mark_cell — stamp one glyph in colour at a cell ── */

/* Draw one character at cell (cx, cy) in the given colour, skipping cells
 * off-screen. One place for the off-screen check and the colour on/off pair.
 * The double cast (chtype)(unsigned char) stops chars above 127 turning
 * negative and corrupting the glyph. */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5f draw_segment_beads — draw the line between two joints ── */

/* Stamp 'o' beads along the line from a to b, one small step at a time so
 * the line is continuous. prev_cx/prev_cy skip repeats when several steps
 * land in the same cell (re-stamping would flicker the colour). */
static void draw_segment_beads(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx;  prev_cy = cy;
        mark_cell(w, cx, cy, 'o', pair, attr, cols, rows);
    }
}

/* ── §5g render_chain — draw the whole snake for one frame ── */

/* Fill rj[] with the positions to actually draw: a blend between last tick's
 * pose and this tick's, by fraction alpha. This smooths the motion when the
 * draw rate and physics rate don't line up. */
static void lerp_joints(const Snake *s, float alpha, Vec2 rj[N_SEGS + 1])
{
    for (int i = 0; i <= N_SEGS; i++) {
        rj[i].x = s->prev_joint[i].x
                + (s->joint[i].x - s->prev_joint[i].x) * alpha;
        rj[i].y = s->prev_joint[i].y
                + (s->joint[i].y - s->prev_joint[i].y) * alpha;
    }
}

/* Draw the line of every segment. Tail-first, so where two segments overlap
 * the head-end colour is drawn last and wins. */
static void draw_body_fill(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                           int cols, int rows)
{
    for (int i = N_SEGS - 1; i >= 0; i--) {
        draw_segment_beads(w,
                           rj[i + 1], rj[i],
                           seg_pair(i), seg_attr(i),
                           cols, rows);
    }
}

/* Draw a bead marker on top of each joint (over the segment lines), giving
 * the snake its knobbly look. Tail-first again so head colours win. */
static void draw_body_nodes(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                            int cols, int rows)
{
    for (int i = N_SEGS; i >= 1; i--) {
        int cx = px_to_cell_x(rj[i].x);
        int cy = px_to_cell_y(rj[i].y);
        char ch = (char)joint_node_char(i);
        mark_cell(w, cx, cy, ch,
                  seg_pair(i - 1), seg_attr(i - 1),
                  cols, rows);
    }
}

/* Draw the head arrow on top of everything. Its direction comes from the
 * live heading, not the blended pose, so it never flickers. */
static void draw_head(WINDOW *w, const Vec2 *head_pos, float heading,
                      int cols, int rows)
{
    int cx = px_to_cell_x(head_pos->x);
    int cy = px_to_cell_y(head_pos->y);
    char ch = (char)head_glyph(heading);
    mark_cell(w, cx, cy, ch, 1 /* PAIR_HEAD */, A_BOLD, cols, rows);
}

/* Draw the snake, back layer to front: work out the blended poses, draw the
 * segment lines, then the bead markers on top, then the head arrow on top. */
static void render_chain(const Snake *s, WINDOW *w,
                          int cols, int rows, float alpha)
{
    Vec2 rj[N_SEGS + 1];
    lerp_joints(s, alpha, rj);

    draw_body_fill (w, rj, cols, rows);
    draw_body_nodes(w, rj, cols, rows);
    draw_head      (w, &rj[0], s->heading, cols, rows);
}

/* ── §6 scene — set up, step, and draw the world ── */

/*
 * Scene — the whole simulated world. Here it's just one snake, but we keep
 * the wrapper so the scene_init / scene_tick / scene_draw loop matches every
 * other demo in the project. A future variant could add prey or obstacles
 * here as new fields without touching the main loop.
 */
typedef struct {
    Snake snake;               /* the world: just one snake for now */
} Scene;

/*
 * Set up the snake so it's already swimming on the first frame.
 *   - Head starts left-of-centre, vertically centred, with room to move.
 *   - Heading is a slight downward angle so it drifts toward the middle.
 *   - wave_time starts at π/2 (the peak of the sine) so it's already curving
 *     instead of swimming straight for the first few seconds.
 *   - The trail is pre-filled with a straight line of points stretching back
 *     behind the head. Without this the body would have no path to follow yet
 *     and would collapse to a single dot until the trail built up.
 * Finally copy joint into prev_joint so the first frame's blend is a no-op.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    int saved_theme = sc->snake.theme_idx;
    memset(sc, 0, sizeof *sc);
    Snake *s       = &sc->snake;
    s->theme_idx   = saved_theme;
    s->move_speed  = MOVE_SPEED_DEFAULT;
    s->amplitude   = AMPLITUDE_DEFAULT;
    s->frequency   = FREQUENCY_DEFAULT;
    s->speed_scale = SPEED_SCALE_DEFAULT;
    s->paused      = false;

    /* wave_time at π/2 = peak of sine → immediately curving on frame 1 */
    s->wave_time = (float)M_PI * 0.5f;

    /* Heading slightly south-east: drifts toward centre on most terminals */
    s->heading = (float)M_PI / 8.0f;

    /* Head at 38% from left, vertically centred — well inside the
     * EDGE_MARGIN_PX band so the snake starts swimming under wave-only
     * steering, with no edge bias kicking in until it actually nears a wall. */
    s->joint[0].x = (float)(cols * CELL_W) * 0.38f;
    s->joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /*
     * Pre-populate trail: TRAIL_CAP entries, 1 px apart, extending behind
     * the head in the direction opposite to heading (i.e., heading + π).
     *
     * bx, by form a unit vector pointing AWAY from the initial heading.
     * trail[0] = newest = head; trail[k] = head + k × (bx, by).
     */
    float bx = cosf(s->heading + (float)M_PI);   /* unit vec backward */
    float by = sinf(s->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        s->trail[k].x = s->joint[0].x + (float)k * bx;
        s->trail[k].y = s->joint[0].y + (float)k * by;
    }
    s->trail_head  = 0;       /* index 0 is the newest (= head) entry    */
    s->trail_count = TRAIL_CAP;

    compute_joints(s);
    memcpy(s->prev_joint, s->joint, sizeof s->joint);
}

/*
 * scene_tick() — one fixed-step physics update, called by §8 accumulator.
 *
 * dt is the fixed tick duration in seconds (= 1.0 / sim_fps).
 *
 * ORDER IS IMPORTANT:
 *   1. Save prev_joint[] FIRST — before any physics runs.
 *      This is the interpolation anchor for render_chain(); it must hold
 *      the state from the end of the PREVIOUS tick, not this one.
 *      Saving after move_head would produce a lerp that overshoots.
 *
 *   2. Return early if paused — prev_joint is saved regardless so the
 *      alpha lerp in render_chain() produces a clean freeze (prev = curr).
 *
 *   3. move_head() — advances wave_time, updates heading, translates
 *      joint[0], wraps it, pushes it into the trail.
 *
 *   4. compute_joints() — samples the now-updated trail to set joint[1..N].
 *      Must run AFTER move_head() so the body follows this tick's head.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Snake *s = &sc->snake;
    memcpy(s->prev_joint, s->joint, sizeof s->joint);   /* Step 1 */
    if (s->paused) return;                               /* Step 2 */
    move_head(s, dt, cols, rows);                        /* Step 3 */
    compute_joints(s);                                   /* Step 4 */
}

/*
 * scene_draw() — render the scene; called once per render frame.
 * alpha ∈ [0, 1) is the sub-tick interpolation factor (see §5f).
 * dt_sec is unused here (no entity needs it for interpolation).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_chain(&sc->snake, w, cols, rows, alpha);
}

/* ── §7 screen — the ncurses display layer ── */

/*
 * Screen — the ncurses display layer.
 *
 * Holds the current terminal dimensions (cols, rows), read after each
 * resize.  See framework.c §7 for the full double-buffer architecture
 * rationale (erase → draw → wnoutrefresh → doupdate).
 */
/*
 * Screen — terminal-extent snapshot in CHARACTER CELLS.
 *
 * Intent
 *   Caches the current terminal size so every §6 entry point reads
 *   (cols, rows) as plain ints rather than re-querying ncurses each
 *   frame. Refreshed only when SIGWINCH sets App::need_resize, then
 *   propagated to scene_init via app_do_resize.
 *
 * Why a separate struct (not just two ints in App)
 *   Resize logic (endwin + refresh + getmaxyx) touches NOTHING in App
 *   except this struct. Carving it out makes screen_resize pure and
 *   isolates the ncurses dependency from the simulation layer.
 *
 * Why cells, not pixels
 *   ncurses' coordinate system is cells. Pixel space (CELL_W × CELL_H
 *   sub-pixels per cell) lives only inside §5 — converted at the
 *   draw boundary, per the project's "one conversion point" rule.
 *
 * References [12] Raymond, NCURSES Programming HOWTO.
 */
typedef struct {
    int cols;   /* terminal width  in CHARACTER CELLS */
    int rows;   /* terminal height in CHARACTER CELLS */
} Screen;

/*
 * screen_init() — configure the terminal for animation.
 *
 *   initscr()         initialise ncurses; must be first.
 *   noecho()          do not echo typed characters to the screen.
 *   cbreak()          pass keys immediately, without line buffering.
 *   curs_set(0)       hide the hardware cursor (no blinking cursor).
 *   nodelay(TRUE)     getch() returns ERR immediately if no key — makes
 *                     input polling non-blocking so the render loop
 *                     never stalls waiting for input.
 *   keypad(TRUE)      decode arrow keys and function keys into single
 *                     KEY_* constants rather than multi-byte sequences.
 *   typeahead(-1)     disable ncurses' habit of calling read() mid-output
 *                     to look for escape sequences; without this, terminal
 *                     output can be interrupted and frames arrive torn.
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
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_free() — restore the terminal to its pre-animation state.
 * endwin() re-enables echo, shows the cursor, and restores the scroll region.
 */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH (terminal resize) event.
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from the
 * kernel and resize its internal virtual screens (curscr/newscr) to match
 * the new terminal dimensions.  Without this, stdscr retains the old size
 * and mvwaddch at coordinates in the newly valid area silently fails.
 *
 * Called from app_do_resize() which also clamps joint[0] to new bounds.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose the full frame into stdscr (ncurses' newscr).
 *
 * Frame composition order (must not change):
 *   1. erase()        — write spaces over the entire newscr, erasing stale
 *                       content from the previous frame.  Does NOT write to
 *                       the terminal; only modifies the in-memory newscr.
 *   2. scene_draw()   — write snake body and head glyphs.
 *   3. HUD (top)      — status bar written last so it is always on top of
 *                       any snake glyph that might occupy the same row.
 *   4. Hint bar (bottom) — key reference line, also on top.
 *
 * Nothing reaches the terminal until screen_present() calls doupdate().
 *
 * HUD content: fps · sim Hz · move speed · wave amplitude · wave frequency ·
 *              speed scale · state (swimming / PAUSED).
 * Hint bar: brief keyboard reference for all controllable parameters.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    const Snake *sn = &sc->snake;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  spd:%.0f  amp:%.2f  freq:%.1f  x%.2f  [%s]  %s ",
             fps, sim_fps,
             sn->move_speed,
             sn->amplitude,
             sn->frequency,
             sn->speed_scale,
             THEMES[sn->theme_idx].name,
             sn->paused ? "PAUSED " : "swimming");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  UD:spd  LR/ad:freq  ws:amp  +/-:wave-x  t/T:theme  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal in one write.
 *
 * wnoutrefresh(stdscr) copies the in-memory newscr model into ncurses'
 *   internal "pending update" structure.  No terminal I/O yet.
 * doupdate() diffs newscr against curscr (what is physically on screen),
 *   sends only the changed cells to the terminal fd, then sets curscr=newscr.
 *
 * This two-step sequence is the correct way to flush in ncurses.  Calling
 * refresh() (= wrefresh(stdscr) = wnoutrefresh + doupdate in one call)
 * is fine for a single window, but the two-step allows multiple windows to
 * be batched into one doupdate() for truly atomic multi-window renders.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — top-level container + main loop ── */

/*
 * App — top-level container; everything outside the world.
 *
 * Intent
 *   Bundles the simulated world (Scene), the host terminal (Screen),
 *   and the session-level loop-control flags into one record so
 *   main() reads as four-line phases: init / service signals /
 *   step+draw / shutdown. Declared file-scope (g_app) so signal
 *   handlers — which cannot take a user argument — can write
 *   `running` and `need_resize` without globals scattered through
 *   the file.
 *
 * Locality of concern
 *   ── Owned subsystems ── nouns the app composes
 *      scene       — the world being simulated (§6)
 *      screen      — the terminal extent it draws to (§7)
 *
 *   ── Session state ── settings the user controls across resets
 *      sim_fps     — physics tick rate (cycled with [ / ])
 *
 *   ── Loop control ── verbs the loop reads each frame
 *      running     — clear → loop exits; set by SIGINT/SIGTERM
 *      need_resize — set by SIGWINCH; cleared after Screen refresh
 *
 * Why volatile sig_atomic_t (not bool, not int)
 *   `volatile`    : the compiler must not cache the flag across a
 *                   signal-handler write — every loop iteration must
 *                   re-read it from memory.
 *   `sig_atomic_t`: POSIX-guaranteed atomic with respect to async
 *                   signals; a plain `int` could be observed half-
 *                   written on architectures where stores are split.
 *   See [12] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no
 *     other code path needs them.
 *   - Snake tuning values (amplitude, frequency, theme_idx) —
 *     simulation/render state in §5 Snake.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal extent (§7)                 */

    /* ── Session state ────────────────────────────────────────── */
    int    sim_fps;            /* physics tick rate (Hz)               */

    /* ── Loop control ─────────────────────────────────────────── */
    volatile sig_atomic_t running;      /* main loop predicate            */
    volatile sig_atomic_t need_resize;  /* SIGWINCH pending               */
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls here */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 * Registered with atexit() so that endwin() is always called even if the
 * program exits via an unhandled signal path that bypasses screen_free().
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * screen_resize() re-reads LINES/COLS from the kernel.  If the terminal
 * was made smaller and joint[0] now falls outside the new pixel bounds,
 * it is clamped to just inside the boundary rather than re-centred — the
 * snake continues swimming from wherever it is rather than teleporting.
 *
 * frame_time and sim_accum are reset in the main loop after this returns
 * to prevent a physics avalanche from the large dt that would otherwise
 * accumulate during the resize operation.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Snake *s   = &app->scene.snake;
    float  wpx = (float)(app->screen.cols * CELL_W);
    float  hpx = (float)(app->screen.rows * CELL_H);
    if (s->joint[0].x >= wpx) s->joint[0].x = wpx - 1.0f;
    if (s->joint[0].y >= hpx) s->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * app_handle_key() — process a single keypress; return false to quit.
 *
 * All keys adjust simulation parameters — there is no manual steering.
 * The snake's heading is driven entirely by the autonomous wave (§5b).
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         toggle paused
 *   r / R         reset simulation (theme preserved)
 *   ↑  KEY_UP     move_speed × 1.20   [MOVE_SPEED_MIN, MAX]
 *   ↓  KEY_DOWN   move_speed ÷ 1.20
 *   ← / a / A     frequency − 0.1     [FREQUENCY_MIN, MAX]
 *   → / d / D     frequency + 0.1
 *   w / W         amplitude + 0.1     [AMPLITUDE_MIN, MAX]
 *   s / S         amplitude − 0.1
 *   + / =         speed_scale × 1.25  [0.05, 8.0]
 *   -             speed_scale ÷ 1.25
 *   t             next color theme (wraps 0..N_THEMES-1)
 *   T             previous color theme
 *   ]             sim_fps + step      [SIM_FPS_MIN, MAX]
 *   [             sim_fps − step
 */
static bool app_handle_key(App *app, int ch)
{
    Snake *s = &app->scene.snake;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': s->paused = !s->paused; break;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    /* Move speed */
    case KEY_UP:
        s->move_speed *= 1.20f;
        if (s->move_speed > MOVE_SPEED_MAX) s->move_speed = MOVE_SPEED_MAX;
        break;
    case KEY_DOWN:
        s->move_speed /= 1.20f;
        if (s->move_speed < MOVE_SPEED_MIN) s->move_speed = MOVE_SPEED_MIN;
        break;

    /* Undulation frequency */
    case KEY_LEFT: case 'a': case 'A':
        s->frequency -= 0.1f;
        if (s->frequency < FREQUENCY_MIN) s->frequency = FREQUENCY_MIN;
        break;
    case KEY_RIGHT: case 'd': case 'D':
        s->frequency += 0.1f;
        if (s->frequency > FREQUENCY_MAX) s->frequency = FREQUENCY_MAX;
        break;

    /* Swim amplitude */
    case 'w': case 'W':
        s->amplitude += 0.1f;
        if (s->amplitude > AMPLITUDE_MAX) s->amplitude = AMPLITUDE_MAX;
        break;
    case 's': case 'S':
        s->amplitude -= 0.1f;
        if (s->amplitude < AMPLITUDE_MIN) s->amplitude = AMPLITUDE_MIN;
        break;

    /* Wave-time speed scale */
    case '=': case '+':
        s->speed_scale *= 1.25f;
        if (s->speed_scale > 8.0f) s->speed_scale = 8.0f;
        break;
    case '-':
        s->speed_scale /= 1.25f;
        if (s->speed_scale < 0.05f) s->speed_scale = 0.05f;
        break;

    /* Color themes */
    case 't':
        s->theme_idx = (s->theme_idx + 1) % N_THEMES;
        theme_apply(s->theme_idx);
        break;
    case 'T':
        s->theme_idx = (s->theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme_idx);
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

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop  (structure identical to framework.c §8)
 *
 * The loop body executes these seven steps every frame:
 *
 *  ① RESIZE CHECK
 *     Handle a pending SIGWINCH before touching ncurses state.
 *     Reset frame_time and sim_accum so the large dt that accumulated
 *     during the resize does not inject a physics jump.
 *
 *  ② MEASURE dt
 *     Wall-clock nanoseconds since the previous frame start.
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger)
 *     and resumed, an uncapped dt would fire hundreds of physics ticks
 *     in one frame — a physics avalanche that looks like a sudden jump.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt each frame.
 *     While sim_accum ≥ tick_ns (one physics tick duration), fire one
 *     scene_tick() and drain tick_ns from sim_accum.
 *     Result: physics runs at exactly sim_fps Hz on average, regardless
 *     of how fast or slow the render loop runs.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining, sim_accum holds the fractional leftover — how far
 *     into the next unfired tick we are.
 *       alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to render_chain() so joint positions are lerped between the
 *     last tick and the current tick, eliminating micro-stutter.
 *
 *  ⑤ FPS COUNTER
 *     Frames counted over a 500 ms sliding window.  Dividing the frame
 *     count by elapsed seconds gives a smoothed fps estimate.  This avoids
 *     per-frame division (which would oscillate wildly) and per-frame
 *     string formatting (which is slow).
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     elapsed = time spent on physics since frame_time was updated.
 *     budget  = NS_PER_SEC / 60  (one 60-fps frame).
 *     sleep   = budget − elapsed.
 *     Sleeping before terminal I/O means the I/O cost is not charged
 *     against the next frame's budget.  If sleep is negative (frame
 *     over-budget), clock_sleep_ns() returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     erase() → scene_draw() → HUD → wnoutrefresh() → doupdate().
 *     One atomic diff write; no partial frames reach the terminal.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued key event.
 *     Looping (not single-call) ensures all key-repeat events are
 *     consumed within the same frame they arrive, keeping parameter
 *     adjustments responsive when keys are held.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed RNG from monotonic clock so each run looks different */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Safety net: endwin() even if we exit via an unhandled path */
    atexit(cleanup);

    /* SIGINT / SIGTERM — graceful exit from Ctrl-C or kill */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — terminal resize; handled at the top of the next iteration */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();   /* timestamp at start of last frame */
    int64_t sim_accum   = 0;            /* nanoseconds in the physics bucket */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window  */
    int     frame_count = 0;            /* frames rendered in fps window     */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD         */

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset so dt doesn't spike         */
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per physics tick   */
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
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

        /* ── ⑥ frame cap — sleep before render ──────────────────── *
         * Budget = 1/60 s.  elapsed is wall time spent on physics +
         * accounting since frame_start; sleep the remainder so the
         * render rate sits at 60 fps regardless of sim Hz.            */
        int64_t elapsed = clock_ns() - frame_start;
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
