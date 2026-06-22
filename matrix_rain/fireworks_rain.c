/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fireworks_rain.c — fireworks whose every spark drags a shimmering
 * Matrix-rain trail of rerolling ASCII along the arc it falls through.
 *
 * Sister files: matrix_rain/matrix_rain.c (the plain vertical version of
 * the same rerolling-glyph trail — read it first if that trick is new) and
 * particle_systems/fireworks.c (the rocket-and-burst skeleton, no trails).
 * The particle model is the classic one from Reeves, "Particle Systems",
 * SIGGRAPH '83.
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* §1  config — every tunable in one place */

/* §1.1 frame rate */
enum {
    TARGET_FPS = 60,
};

/* §1.2 rocket pool */
enum {
    ROCKETS_MIN     =  1,
    ROCKETS_DEFAULT =  5,
    ROCKETS_MAX     = 16,
    MAX_ROCKETS     = ROCKETS_MAX,
};

/* Stagger the start so the rockets don't all fire at once and bunch up
 * at the top: rocket i waits i of these before its first launch. */
#define INITIAL_FUSE_STAGGER_SEC  0.13f

/* §1.3 rocket physics, in cells per second (y points DOWN, so up is negative) */

/* How hard a rocket is thrown upward. The slow end pops about 5 rows up
 * a second after launch; the fast end climbs ~30 rows over ~1.5 s. */
#define ROCKET_LAUNCH_VY_MIN_CPS  -18.0f   /* slow rocket */
#define ROCKET_LAUNCH_VY_MAX_CPS  -48.0f   /* fast rocket */

/* Pull of gravity on a rising rocket. Bigger means it runs out of climb
 * sooner and bursts lower; this value keeps bursts in the upper half. */
#define ROCKET_GRAVITY_CPS2       30.0f

/* Burst early if a rocket reaches the top edge — only matters for the
 * occasional too-fast launch. */
#define ROCKET_TOP_EXPLODE_ROW    2.0f

/* How long a spent rocket waits before launching again: 0.5 to 2.5 s. */
#define FUSE_MIN_SEC      0.5f
#define FUSE_VAR_SEC      2.0f

/* §1.4 spark physics, cells per second */
enum { PARTICLES_PER_BURST = 72 };

/* How fast each spark flies away from the burst point. */
#define SPARK_SPEED_MIN_CPS    12.0f
#define SPARK_SPEED_MAX_CPS    40.0f

/* Gravity on each spark, plus a per-spark wobble of +/-20 % so they don't
 * all fall along the exact same arc — without it a burst drops in lockstep. */
#define SPARK_GRAVITY_CPS2     32.0f
#define SPARK_GRAVITY_JITTER    0.20f

/* How long a spark lives and how fast it dims. life starts somewhere in
 * [MIN, MIN+VAR] on a 0..1 scale and drains by decay-per-second; together
 * they give a spark roughly 0.33 to 1.33 s before it dies. */
#define SPARK_LIFE_MIN          0.6f
#define SPARK_LIFE_VAR          0.4f
#define SPARK_DECAY_MIN_RPS     0.75f
#define SPARK_DECAY_VAR_RPS     1.05f

/* A little random nudge (radians) on each spark's launch angle so the 72
 * of them read as a 3-D ball of sparks, not a flat ring. */
#define BURST_ANGLE_JITTER      0.30f

/* §1.5 trail + shimmer */
enum {
    /* How many past positions each spark remembers — at 60 fps, 16 of
     * them cover about the last quarter-second of its flight. */
    TRAIL_LEN       = 16,

    /* Where the brightness bands sit along the trail (see trail_attr):
     * the first few slots are bold, the middle plain, the tail dim. */
    TRAIL_HOT_END   = 2,
    TRAIL_WARM_END  = TRAIL_LEN / 2,

    /* Each frame, every cached glyph has a 1-in-this chance of staying
     * put; the rest get rerolled. 4 means ~75 % change — the Matrix look. */
    SHIMMER_KEEP_ONE_IN = 4,
};

/* §1.6 fade threshold */
/* Once a spark's life drops below this, the whole trail goes dim so it
 * visibly fades out instead of just vanishing. */
#define FADING_LIFE_THRESHOLD  0.25f

/* §1.7 speed scale — the global slow/fast multiplier the [ and ] keys drive */
#define SPEED_SCALE_DEFAULT    1.0f
#define SPEED_SCALE_MIN        0.25f
#define SPEED_SCALE_MAX        4.0f
#define SPEED_SCALE_STEP       1.25f

/* §1.8 ncurses pair IDs */
enum {
    /* 1..7 — the seven spark colours; the theme decides the actual hues,
     * so these names only match the default "vivid" look. */
    CP_RED          = 1,
    CP_ORANGE,
    CP_YELLOW,
    CP_GREEN,
    CP_CYAN,
    CP_BLUE,
    CP_MAGENTA,

    /* 8 — the bright trail head, always white whatever the theme */
    CP_TRAIL_HEAD,

    /* 9..10 — the HUD colours, which never change with the theme */
    PAIR_HUD,
    PAIR_HINT,
};

#define N_SPARK_COLORS  7

/* §1.9 dt cap + timing units */
#define DT_CAP_SEC    0.10f
#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define HUD_BUF_LEN   96

/* §2  clock — monotonic timer + sleep */

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

/* §3  color */

/*
 * Theme — one named palette of seven spark colours.
 *
 * Picking a theme just decides what hue each spark can wear. A spark is
 * handed one of these colours when it's born and keeps it for its whole
 * life, so a burst looks like a fan of coloured rays. The `t` key flips
 * to the next theme; theme_apply() then repaints colour pairs 1..7, so
 * the pair numbers stay the same and only the colours behind them change.
 *
 * Two palettes per theme because terminals differ: colors[] is the rich
 * 256-colour version (smooth shading down each trail); fallback[] is the
 * best we can do on an old 8-colour terminal, where nearby shades blur
 * into one.
 *
 * The presets: vivid (rainbow), matrix (greens, the Matrix-rain look),
 * fire (reds/oranges/yellows), ice (blues/cyans), plasma (purple/magenta).
 *
 * Members
 *   name        the label shown in the HUD ("vivid", "matrix", ...); never NULL.
 *   colors[]    the seven 256-colour indices.
 *   fallback[]  the seven 8-colour indices for old terminals.
 *
 * Every colors[] entry stays at index 24 or higher: the very dark cube and
 * gray colours (16-23, 232-239) come out black on the default background and
 * would make sparks invisible (CLAUDE.md "Theme Palette Brightness").
 */
typedef struct {
    const char *name;
    int         colors  [N_SPARK_COLORS];
    int         fallback[N_SPARK_COLORS];
} Theme;

static const Theme k_themes[] = {
    { "vivid",
      { 196, 208, 226,  46,  51,  33, 201 },
      { COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA } },
    { "matrix",
      {  28,  34,  40,  46,  82, 118, 154 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN } },
    { "fire",
      { 196, 160, 202, 208, 214, 220,  88 },
      { COLOR_RED, COLOR_RED, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED } },
    { "ice",
      {  24,  33,  39,  45,  51,  87, 153 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE } },
    { "plasma",
      {  53,  57,  93, 129, 165, 201, 207 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/* Repaints the spark colours (and the white trail head) for one theme.
 * Leaves the HUD colours alone on purpose — they must stay readable. */
static void theme_apply(int idx)
{
    const Theme *t = &k_themes[idx];
    for (int i = 0; i < N_SPARK_COLORS; i++) {
        int fg = g_has_256 ? t->colors[i] : t->fallback[i];
        init_pair(i + 1, fg, COLOR_BLACK);
    }
    init_pair(CP_TRAIL_HEAD, g_has_256 ? 255 : COLOR_WHITE, COLOR_BLACK);
}

/* Sets up the HUD colours once at startup. They sit on the terminal's own
 * background (-1) so the status text never appears inside a black box. */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

static int color_rand(void) { return 1 + rand() % N_SPARK_COLORS; }

/* §4  spark — the matrix-rain trail effect */

/* §4.1 the glyph pool + tiny helpers */

/* The characters a shimmering trail can show — same set as matrix_rain.c. */
static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01   (void) { return (float)rand() / (float)RAND_MAX; }

/* §4.2 Vec2 + Spark types */

/*
 * Vec2 — a 2-D point or velocity.
 *
 * x and y are measured in terminal cells but kept as floats, so a spark
 * can sit between cells and only round to a whole cell when it's drawn —
 * that fractional precision is what makes the arcs look smooth.
 * Top-left is the origin and y grows downward, the usual terminal convention.
 */
typedef struct { float x, y; } Vec2;

/*
 * Spark — one piece of an explosion, dragging a Matrix-rain trail.
 *
 * A plain firework particle is just one bright dot following an arc. This
 * one also remembers where it's been: the last TRAIL_LEN positions it
 * passed through, drawn as a comet tail of ASCII characters that keep
 * randomly changing — the Matrix-rain shimmer, one per spark.
 *
 * Each frame it does four things (see spark_tick): slide a new position
 * into the trail, move under gravity, reroll most of its glyphs, and dim
 * a little. The trail is drawn newest-first so the bright head lands on
 * top wherever it overlaps an older part of the tail.
 *
 * Each spark keeps its OWN trail rather than scribbling glyphs into a
 * shared grid: the arcs are curved and they cross, so a shared grid would
 * lose track of which spark owns which glyph. And the glyphs are cached,
 * not picked fresh every frame — only some reroll each frame — so the
 * trail holds a shape and the shimmer reads as texture instead of noise.
 *
 * Members
 *   head          Where the spark is right now, in cell-space (fractional).
 *   vel           How fast it's moving, in cells per second. Updated by
 *                 plain forward-Euler each frame: head moves using the old
 *                 velocity, then gravity nudges the velocity. Good enough —
 *                 a spark only lives about a second, so any drift is unseen.
 *
 *   trail[]       The recent head positions, newest at [0], oldest at the
 *                 end. Every frame they all slide down one to make room.
 *   trail_fill    How many of those slots have real data yet. Climbs from 0
 *                 to full over the spark's first frames, so the tail grows
 *                 out of the burst point instead of popping in fully formed.
 *
 *   cache[]       One random glyph per trail position. Most get rerolled
 *                 every frame — this is the shimmer.
 *
 *   life          A 1.0-down-to-0.0 health bar; the renderer dims the trail
 *                 as it nears 0.
 *   decay_rps     How fast life drains per second. Randomised per spark so a
 *                 burst fades out raggedly instead of all dying at once.
 *   color         The hue picked when the spark was born; fixed for life.
 *   active        Goes false when life hits 0; the ticker then skips it and
 *                 the slot waits to be reused on the next burst.
 *
 * Invariants
 *   0 <= trail_fill <= TRAIL_LEN.
 *   trail[0] == head right after each trail slide.
 *   active == (life > 0).
 */
typedef struct {
    /* where it is and how it's moving */
    Vec2  head;
    Vec2  vel;

    /* the comet tail of past positions */
    Vec2  trail[TRAIL_LEN];
    int   trail_fill;

    /* one shimmering glyph per tail position */
    char  cache[TRAIL_LEN];

    /* how alive it is, and how it looks */
    float life;
    float decay_rps;
    int   color;
    bool  active;
} Spark;

/* §4.3 spark_burst_spawn — light off one whole explosion */

/* The launch direction for spark i: an even share of the full circle, plus
 * a small random skew so the sparks don't land on a perfect ring outline. */
static inline float burst_angle_for(int i, int count) {
    float ring_angle   = ((float)i / (float)count) * 2.0f * (float)M_PI;
    float random_skew  = urand01() * BURST_ANGLE_JITTER;
    return ring_angle + random_skew;
}

static inline float random_uniform_in(float lo, float hi) {
    return lo + urand01() * (hi - lo);
}

/* Fills a fresh spark's whole trail with random glyphs so it has something
 * to show on frame one, before the per-frame shimmer takes over. */
static inline void fill_glyph_cache_random(Spark *p) {
    for (int k = 0; k < TRAIL_LEN; k++)
        p->cache[k] = rand_glyph();
}

/* Sets up `count` brand-new sparks all flying out from `origin`. */
static void spark_burst_spawn(Spark *pool, int count, Vec2 origin)
{
    for (int i = 0; i < count; i++) {
        Spark *p = &pool[i];

        float angle = burst_angle_for(i, count);

        float speed = random_uniform_in(SPARK_SPEED_MIN_CPS,
                                         SPARK_SPEED_MAX_CPS);

        p->head        = origin;
        p->vel.x       = cosf(angle) * speed;
        p->vel.y       = sinf(angle) * speed;
        p->trail_fill  = 0;

        /* spread the lifespans so the burst fades out raggedly */
        p->life        = SPARK_LIFE_MIN  + urand01() * SPARK_LIFE_VAR;
        p->decay_rps   = SPARK_DECAY_MIN_RPS
                       + urand01() * SPARK_DECAY_VAR_RPS;

        p->color       = color_rand();

        fill_glyph_cache_random(p);

        p->active      = true;
    }
}

/* §4.4 the per-frame steps, one function each */

/* Remembers the current position: drops it in at the front of the trail and
 * pushes everything else back one. The loop runs back-to-front on purpose —
 * front-to-back would smear the newest value across the whole buffer. */
static void spark_advance_trail(Spark *p)
{
    for (int i = TRAIL_LEN - 1; i > 0; i--)
        p->trail[i] = p->trail[i - 1];
    p->trail[0] = p->head;
    if (p->trail_fill < TRAIL_LEN) p->trail_fill++;
}

/* Moves the spark one frame forward under gravity. Each spark gets its
 * gravity wobbled a bit so the burst spreads out instead of falling as one. */
static void spark_integrate(Spark *p, float dt)
{
    float jitter = 1.0f - SPARK_GRAVITY_JITTER + urand01() * 2.0f * SPARK_GRAVITY_JITTER;
    float g      = SPARK_GRAVITY_CPS2 * jitter;
    p->head.x += p->vel.x * dt;
    p->head.y += p->vel.y * dt;
    p->vel.y  += g * dt;
}

/* The shimmer: rerolls about three-quarters of the trail's glyphs, leaving
 * the rest so the trail keeps its shape from frame to frame. */
static void spark_shimmer(Spark *p)
{
    for (int k = 0; k < TRAIL_LEN; k++)
        if (rand() % SHIMMER_KEEP_ONE_IN != 0)
            p->cache[k] = rand_glyph();
}

/* §4.5 spark_tick — do all the steps in order */

static void spark_tick(Spark *p, float dt)
{
    if (!p->active) return;

    spark_advance_trail(p);
    spark_integrate    (p, dt);
    spark_shimmer      (p);

    p->life -= p->decay_rps * dt;
    if (p->life <= 0.0f) p->active = false;
}

/* §4.6 trail_attr — how bright each part of the trail is */

/* Picks the brightness for one trail slot: the near-head slots are bold,
 * the middle plain, the far tail dim. A dying spark (fading) goes all-dim
 * so it fades out cleanly. */
static attr_t trail_attr(int i, int cp, bool fading)
{
    if (fading)              return COLOR_PAIR(cp) | A_DIM;
    if (i <= TRAIL_HOT_END)  return COLOR_PAIR(cp) | A_BOLD;
    if (i <  TRAIL_WARM_END) return COLOR_PAIR(cp);
    return COLOR_PAIR(cp) | A_DIM;
}

/* §4.7 spark_draw — paint the tail oldest-first, then the head on top */

/* True if (x, y) lands on the visible screen. Off-screen draws are skipped
 * so ncurses never gets an out-of-bounds cell. */
static inline bool cell_in_screen(int x, int y, int cols, int rows) {
    return x >= 0 && x < cols && y >= 0 && y < rows;
}

/* Draws one character with the given attributes. The cast keeps a
 * high-value byte (> 127) from being read as a negative char by ncurses. */
static inline void paint_glyph_at(int y, int x, char glyph, attr_t attrs) {
    attron(attrs);
    mvaddch(y, x, (chtype)(unsigned char)glyph);
    attroff(attrs);
}

/* Draws one slot of the trail at its rounded cell, skipping it if off-screen. */
static inline void paint_trail_slot(const Spark *p, int slot_idx,
                                     int cols, int rows, bool fading) {
    int x = (int)roundf(p->trail[slot_idx].x);
    int y = (int)roundf(p->trail[slot_idx].y);
    if (!cell_in_screen(x, y, cols, rows)) return;
    paint_glyph_at(y, x, p->cache[slot_idx],
                   trail_attr(slot_idx, p->color, fading));
}

/* Draws the bright head. Painted last so it wins wherever it lands on top of
 * a tail slot — bold white while alive, dim in the spark's colour while dying. */
static inline void paint_live_head(const Spark *p, int cols, int rows,
                                    bool fading) {
    int hx = (int)roundf(p->head.x);
    int hy = (int)roundf(p->head.y);
    if (!cell_in_screen(hx, hy, cols, rows)) return;

    attr_t head_attrs = fading
        ? (COLOR_PAIR(p->color)      | A_DIM)
        : (COLOR_PAIR(CP_TRAIL_HEAD) | A_BOLD);
    paint_glyph_at(hy, hx, p->cache[0], head_attrs);
}

/* Draws one spark: its tail oldest-first, then its head on top. Skipped
 * entirely if the spark is dead. The oldest-first order is what lets the
 * brighter, newer parts cover the older ones where they overlap. */
static void spark_draw(const Spark *p, int cols, int rows)
{
    if (!p->active) return;

    bool fading = (p->life < FADING_LIFE_THRESHOLD);

    for (int slot = p->trail_fill - 1; slot >= 0; slot--)
        paint_trail_slot(p, slot, cols, rows, fading);

    paint_live_head(p, cols, rows, fading);
}

/* §5  rocket — its IDLE -> RISING -> EXPLODED life */

/* §5.1 RocketState enum + Rocket type */

/*
 * RocketState — which of three stages a firework is in.
 *
 *   RS_IDLE      Waiting underground, counting its fuse down; nothing drawn.
 *                When the fuse hits zero it launches.
 *   RS_RISING    Climbing. Gravity slows it; the moment it starts to fall
 *                (or reaches the top edge) it bursts into sparks and moves on.
 *   RS_EXPLODED  Its sparks are flying. Once they've all died out, the rocket
 *                goes back to IDLE with a fresh random fuse and does it again.
 *
 * IDLE is 0 on purpose: a zeroed-out rocket is automatically idle, which
 * scene_init relies on when it parks the slots it isn't using.
 */
typedef enum {
    RS_IDLE     = 0,
    RS_RISING   = 1,
    RS_EXPLODED = 2,
} RocketState;

/*
 * Rocket — one rising streak that owns the sparks it bursts into.
 *
 * A rocket handles its own whole show: the climb, the burst at the top,
 * the sparks flying out, and the wait before it goes again. Its burst of
 * sparks lives right inside it (particles[]), so a burst never has to
 * allocate anything — old sparks are just marked dead and overwritten next
 * time. That also matches the project's "no malloc after startup" rule.
 *
 * A rocket only tracks vertical speed; it flies straight up. The spread
 * comes from the random launch column and the fan of sparks at the top, so
 * a sideways drift would only blur the streak without adding anything.
 *
 * Members
 *   x, y         Where it is. y grows downward, so a climbing rocket's y
 *                is going down.
 *   vy           Vertical speed (negative while climbing). Gravity bends it
 *                upward toward zero; hitting zero means it's at the top.
 *   color        The streak's colour while rising. Ignored after it bursts —
 *                each spark then picks its own.
 *   state        Which stage it's in (see RocketState).
 *   fuse_sec     Seconds left before an idle rocket launches; reset to a new
 *                random value each time it recycles.
 *   particles[]  Its built-in batch of sparks. Reused burst after burst.
 *
 * Invariants
 *   IDLE     => fuse_sec > 0 and every particle is dead.
 *   RISING   => vy < 0 (until the tick that tips it past the top).
 *   EXPLODED => at least one spark still alive (else it flips to IDLE).
 */
typedef struct {
    float        x, y;
    float        vy;
    int          color;
    RocketState  state;
    float        fuse_sec;
    Spark        particles[PARTICLES_PER_BURST];
} Rocket;

/* §5.2 rocket_launch — send a fresh rocket up from the bottom */

static void rocket_launch(Rocket *r, int cols, int rows)
{
    r->x     = (float)(rand() % cols);
    r->y     = (float)(rows - 1);
    r->vy    = ROCKET_LAUNCH_VY_MIN_CPS
             + urand01() * (ROCKET_LAUNCH_VY_MAX_CPS - ROCKET_LAUNCH_VY_MIN_CPS);
    r->color = color_rand();
    r->state = RS_RISING;

    for (int i = 0; i < PARTICLES_PER_BURST; i++)
        r->particles[i].active = false;
}

/* §5.3 one tick function per stage */

/* Idle: burn the fuse down, and launch the moment it runs out. */
static void rocket_tick_idle(Rocket *r, float dt, int cols, int rows)
{
    r->fuse_sec -= dt;
    if (r->fuse_sec <= 0.0f)
        rocket_launch(r, cols, rows);
}

/* Rising: climb under gravity, and burst the instant it tops out (or hits
 * the top edge on a too-fast launch), scattering its sparks from there. */
static void rocket_tick_rising(Rocket *r, float dt)
{
    r->y  += r->vy * dt;
    r->vy += ROCKET_GRAVITY_CPS2 * dt;

    if (r->vy >= 0.0f || r->y < ROCKET_TOP_EXPLODE_ROW) {
        Vec2 origin = { r->x, r->y };
        spark_burst_spawn(r->particles, PARTICLES_PER_BURST, origin);
        r->state = RS_EXPLODED;
    }
}

/* Exploded: tick the sparks, and once they've all died go back to idle
 * with a new random fuse. */
static void rocket_tick_exploded(Rocket *r, float dt)
{
    bool any_alive = false;
    for (int i = 0; i < PARTICLES_PER_BURST; i++) {
        spark_tick(&r->particles[i], dt);
        if (r->particles[i].active) any_alive = true;
    }
    if (!any_alive) {
        r->fuse_sec = FUSE_MIN_SEC + urand01() * FUSE_VAR_SEC;
        r->state    = RS_IDLE;
    }
}

/* §5.4 rocket_tick — run whichever stage we're in */

static void rocket_tick(Rocket *r, float dt, int cols, int rows)
{
    switch (r->state) {
    case RS_IDLE:     rocket_tick_idle    (r, dt, cols, rows); break;
    case RS_RISING:   rocket_tick_rising  (r, dt);             break;
    case RS_EXPLODED: rocket_tick_exploded(r, dt);             break;
    }
}

/* §5.5 rocket_draw — the streak while rising, the sparks once burst */

/* The two characters that make up the rising rocket. */
enum {
    ROCKET_BODY_GLYPH = '|',   /* bright head */
    ROCKET_TAIL_GLYPH = '\'',  /* faint tail one cell below */
};

/* Draws the rocket as a bright '|' with a faint tail just below it — a cheap
 * two-cell streak that fakes a smoke trail without storing past positions. */
static void draw_rising_rocket_body(const Rocket *r, int cols, int rows) {
    int x = (int)r->x;
    int y = (int)r->y;
    if (!cell_in_screen(x, y, cols, rows)) return;

    paint_glyph_at(y, x, ROCKET_BODY_GLYPH, COLOR_PAIR(r->color) | A_BOLD);

    bool tail_row_visible = (y + 1 < rows);
    if (tail_row_visible)
        paint_glyph_at(y + 1, x, ROCKET_TAIL_GLYPH, COLOR_PAIR(r->color));
}

/* Draws every spark in the burst. No sorting needed — each spark already
 * paints its own tail back-to-front. */
static void draw_exploded_burst(const Rocket *r, int cols, int rows) {
    for (int i = 0; i < PARTICLES_PER_BURST; i++)
        spark_draw(&r->particles[i], cols, rows);
}

/* Idle rockets draw nothing — they're "below ground" waiting on the fuse. */
static void rocket_draw(const Rocket *r, int cols, int rows)
{
    switch (r->state) {
    case RS_RISING:   draw_rising_rocket_body(r, cols, rows); break;
    case RS_EXPLODED: draw_exploded_burst    (r, cols, rows); break;
    case RS_IDLE:                                              break;
    }
}

/* §6  scene — the Scene and its sub-structs */

/*
 * RocketPool — the rocket array and how many of it are switched on.
 *
 * The array is always full-sized, but only the first active_count slots are
 * live; the rest sit idle with an enormous fuse so they never fire. The '+'
 * key just bumps active_count and wakes the next slot. Keeping the live ones
 * up front (instead of tracking free slots) means there's nothing to
 * allocate and an off slot costs only a "fuse ready? no" each frame.
 *
 * Members
 *   rockets[]      The whole pool; slots 0..active_count-1 are the live ones.
 *   active_count   How many rockets are on, between ROCKETS_MIN and MAX
 *                  (the + / - keys move it).
 *
 * Invariants
 *   0 <= active_count <= MAX_ROCKETS.
 *   Every slot at or past active_count is idle with a huge fuse.
 */
typedef struct {
    Rocket rockets[MAX_ROCKETS];
    int    active_count;
} RocketPool;

/*
 * SimControls — the playback knobs the user controls.
 *
 *   paused        When true the scene freezes and the HUD says PAUSED
 *                 (space / p toggles it).
 *   speed_scale   Stretches or shrinks time for the whole show. 1.0 is
 *                 normal; the [ and ] keys scale it, kept within
 *                 [SPEED_SCALE_MIN, SPEED_SCALE_MAX].
 */
typedef struct {
    bool  paused;
    float speed_scale;
} SimControls;

/*
 * Scene — all the state for one running show, reachable from one pointer.
 *
 *   pool       the rockets and how many are on
 *   sim        the pause / speed knobs
 *   theme_idx  which colour theme is active (the t key cycles it)
 *
 * The theme lives here, not on App, because it's a per-scene look — another
 * scene could start on a different theme.
 */
typedef struct {
    RocketPool  pool;
    SimControls sim;
    int         theme_idx;
} Scene;

/* Wakes one rocket slot with a short fuse so it fires soon. Used to stagger
 * the opening volley and to switch on a slot when the user adds a rocket.
 * cols/rows are passed through so the launch can pick a column and height. */
static void launch_idle_slot(Scene *s, int slot_index, int cols, int rows,
                             float fuse_sec) {
    Rocket *r = &s->pool.rockets[slot_index];
    rocket_launch(r, cols, rows);
    r->fuse_sec = fuse_sec;
    r->state    = RS_IDLE;
}

/* Switches a slot off: a giant fuse it'll never burn through, and all its
 * sparks marked dead. Used for the slots above active_count. */
static void park_idle_slot(Scene *s, int slot_index) {
    Rocket *r = &s->pool.rockets[slot_index];
    r->state    = RS_IDLE;
    r->fuse_sec = 1e9f;        /* effectively never */
    for (int j = 0; j < PARTICLES_PER_BURST; j++)
        r->particles[j].active = false;
}

static void scene_init(Scene *s, int cols, int rows, int rocket_count) {
    s->pool.active_count = rocket_count;
    s->sim.paused        = false;
    s->sim.speed_scale   = SPEED_SCALE_DEFAULT;

    for (int i = 0; i < MAX_ROCKETS; i++) {
        if (i < rocket_count) {
            /* stagger the opening volley so they don't all pop at once */
            float staggered_fuse = (float)i * INITIAL_FUSE_STAGGER_SEC;
            launch_idle_slot(s, i, cols, rows, staggered_fuse);
        } else {
            park_idle_slot(s, i);
        }
    }
}

static void scene_free(Scene *s) { memset(s, 0, sizeof *s); }

static void scene_tick(Scene *s, float dt, int cols, int rows) {
    if (s->sim.paused) return;
    float scaled_dt = dt * s->sim.speed_scale;
    for (int i = 0; i < s->pool.active_count; i++)
        rocket_tick(&s->pool.rockets[i], scaled_dt, cols, rows);
}

static void scene_draw(const Scene *s, int cols, int rows) {
    for (int i = 0; i < s->pool.active_count; i++)
        rocket_draw(&s->pool.rockets[i], cols, rows);
}

/* Scene input helpers — driven by app_handle_key. */

/* Adds or removes rockets (clamped to the allowed range). When adding, it
 * wakes the new slot with a short fuse so it fires soon — a long wait would
 * make the key feel dead. */
static void scene_change_rockets(Scene *s, int delta, int cols, int rows) {
    int next_count = s->pool.active_count + delta;
    if (next_count < ROCKETS_MIN) next_count = ROCKETS_MIN;
    if (next_count > ROCKETS_MAX) next_count = ROCKETS_MAX;

    if (next_count > s->pool.active_count) {
        int waking_slot = s->pool.active_count;
        launch_idle_slot(s, waking_slot, cols, rows, INITIAL_FUSE_STAGGER_SEC);
    }
    s->pool.active_count = next_count;
}

/* Speeds the show up or slows it down (clamped). Multiplying rather than
 * adding means each key press feels the same whatever the current speed. */
static void scene_scale_speed(Scene *s, float factor) {
    s->sim.speed_scale *= factor;
    if (s->sim.speed_scale < SPEED_SCALE_MIN) s->sim.speed_scale = SPEED_SCALE_MIN;
    if (s->sim.speed_scale > SPEED_SCALE_MAX) s->sim.speed_scale = SPEED_SCALE_MAX;
}

/* Moves to the next colour theme and repaints the spark colours. */
static void scene_cycle_theme(Scene *s) {
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/* §7  screen — ncurses setup, present, and HUD */

/*
 * Screen — how big the terminal is, in character cells.
 *
 * Sparks and rockets live in these same cell units, so this is the one
 * source of truth for clipping draws and placing the HUD. cols/rows are
 * read from ncurses at startup and re-read on every resize.
 *
 * Members
 *   cols   width in cells.
 *   rows   height in cells; the bottom row (rows-1) holds the key hint,
 *          the top row (0) holds the status line.
 *
 * Both are > 0 after setup.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    start_color();
    use_default_colors();       /* lets HUD pairs use -1 background       */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Draws the two status lines: fps and settings along the top, the key list
 * along the bottom. Both use fixed colours so they stay readable on any theme. */
static void screen_draw_hud(const Screen *sc, double fps,
                            const Scene *scene)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  spd:%.2fx  rkt:%d  [%s] %s ",
             fps, scene->sim.speed_scale, scene->pool.active_count,
             k_themes[scene->theme_idx].name,
             scene->sim.paused ? "PAUSED " : "running");

    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  []:speed  +/-:rockets  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* §8  app — signals, resize, and the main loop */

/*
 * FpsCounter — a steady frame-rate number for the HUD.
 *
 * A per-frame rate jumps around too much to read, so this tallies frames
 * over a half-second window and only then updates the displayed value.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;     /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — everything the program keeps alive while it runs.
 *
 *   scene         the show itself (rockets, pause/speed, theme)
 *   screen        the terminal size and ncurses handle
 *   fps           the smoothed frame rate for the HUD
 *   running       cleared on Ctrl-C / kill / 'q' to end the loop
 *   need_resize   set when the terminal is resized; the loop reacts next frame
 *
 * g_app is the only global. The signal handlers need it to flip those two
 * flags; everything else is passed an App pointer instead.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Rebuilds the show at the new terminal size after a resize, keeping the
 * user's chosen theme, speed, and rocket count. */
static void app_do_resize(App *app)
{
    int   saved_n     = app->scene.pool.active_count;
    float saved_speed = app->scene.sim.speed_scale;
    int   saved_theme = app->scene.theme_idx;

    scene_free(&app->scene);
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, saved_n);
    app->scene.sim.speed_scale = saved_speed;
    app->scene.theme_idx       = saved_theme;
    app->need_resize           = 0;
}

/* Handles one keypress; returns false only when the user wants to quit.
 * Keys: q/Q/ESC quit, space/p pause, r reset, [ ] slower/faster,
 * +/= add a rocket, - remove one, t cycle theme. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    int    cols = app->screen.cols;
    int    rows = app->screen.rows;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;       break;
    case 'r': case 'R':           scene_init(s, cols, rows,
                                              s->pool.active_count);    break;

    case ']':                     scene_scale_speed(s, SPEED_SCALE_STEP);          break;
    case '[':                     scene_scale_speed(s, 1.0f / SPEED_SCALE_STEP);   break;

    case '=': case '+':           scene_change_rockets(s, +1, cols, rows); break;
    case '-':                     scene_change_rockets(s, -1, cols, rows); break;

    case 't': case 'T':           scene_cycle_theme(s);                  break;

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

    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);
    scene->theme_idx = 0;

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    theme_apply(scene->theme_idx);
    hud_pairs_init();
    scene_init(scene, app->screen.cols, app->screen.rows, ROCKETS_DEFAULT);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* handle a pending resize first so the rest of the frame sees the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* how long since the last frame, capped so one slow frame can't
         * make the catch-up step huge and snowball */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* read any keys waiting */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        scene_tick(scene, dt, app->screen.cols, app->screen.rows);

        fps_counter_tick(&app->fps, dt_ns);

        /* draw the frame and flush it */
        erase();
        scene_draw(scene, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, app->fps.display, scene);
        screen_present();

        /* wait out the rest of the frame's time budget */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    scene_free(scene);
    screen_free(&app->screen);
    return 0;
}
