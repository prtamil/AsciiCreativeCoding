/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fire_tornado.c — a swirling column of fire embers, drawn in ASCII.
 *
 * Embers spin around a central axis and rise; sparks fly off, and a strip
 * of flame flickers along the ground. Heat fades from white-hot to dim.
 * Particle/fire ideas from Reeves (SIGGRAPH 1983) and the 1-D Doom fire
 * (Sanglard, "Game Engine Black Book: DOOM"); glyph ramp from Bourke.
 */

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

/* ── §1 config — all the tunable numbers (sizes, speeds, colours) ── */

#define TARGET_FPS          60

/* The array holds the max; '[' / ']' just changes how many are active. */
#define N_EMBERS_DEFAULT    250
#define N_EMBERS_MIN         50
#define N_EMBERS_MAX        600
#define N_EMBERS_STEP        50

/* Column size, all measured in screen cells. */
#define HEIGHT_FRAC          0.85f
#define HEIGHT_FRAC_MIN      0.40f
#define HEIGHT_FRAC_MAX      0.95f
#define HEIGHT_FRAC_STEP     0.05f
#define BASE_RADIUS_CELLS    9.0f

/* Tuned so an ember takes ~3 seconds to rise — long enough to see it spin. */
#define Y_VEL_BASE           6.0f
#define OMEGA_BASE           5.0f
#define OMEGA_MULT_DEFAULT   1.0f
#define OMEGA_MULT_MIN       0.25f
#define OMEGA_MULT_MAX       4.0f
#define OMEGA_MULT_STEP      0.25f
#define RADIUS_DECAY         0.6f
#define COOL_RATE            0.32f

/* Embers closer to the axis rise and spin faster, so the column tapers
 * into a funnel. Set once when each ember is born. */
#define EMBER_RISE_AXIS      1.4f   /* rise-speed boost on the axis (× Y_VEL_BASE)  */
#define EMBER_RISE_FALLOFF   0.7f   /* outer embers rise slower by this much        */
#define EMBER_OMEGA_RSCALE   0.4f   /* turns radius into a spin rate                */
#define EMBER_OMEGA_FLOOR    0.6f   /* cap so embers near the axis don't spin wildly */
#define EMBER_TEMP_MIN       0.92f  /* birth heat starts here, plus a little jitter  */
#define EMBER_TEMP_JITTER    0.08f
#define EMBER_INIT_COOL      0.7f   /* at startup, higher embers start cooler        */

/* Terminal cells are about twice as tall as they are wide, so we stretch
 * everything horizontally to make the round column actually look round. */
#define ASPECT_X             2.0f

/* The flickering flame strip along the ground. */
#define BASE_HEAT_W          60       /* width in cells, centred on the axis */
#define BASE_HEAT_DECAY      2.0f     /* how fast each cell cools per second  */
#define BASE_INJECT_MIN      1        /* new hot spots added per frame: min   */
#define BASE_INJECT_MAX      3        /* … and max                            */
#define BASE_INJECT_HEAT     0.85f    /* heat level of a new hot spot         */
#define BASE_INJECT_SPREAD   0.35f    /* how far from the axis hot spots land */
#define BASE_INJECT_JITTER   0.15f    /* random heat wobble per hot spot      */
#define BASE_HEAT_VISIBLE    0.05f    /* below this, a cell is too dim to draw */

/* Sparks flung outward from the column. */
#define N_SPARKS_MAX         40
#define SPARK_SPAWN_HZ      10.0f     /* average sparks born per second       */
#define SPARK_SPEED_MIN      8.0f     /* launch speed range, cells/sec        */
#define SPARK_SPEED_MAX     20.0f
#define SPARK_GRAVITY       20.0f     /* downward pull, cells/sec²            */
#define SPARK_COOL           1.5f     /* sparks cool this fast (heat/sec)     */
#define SPARK_GLYPH          '*'
#define SPARK_SRC_TRIES      8        /* attempts to land on a live ember to spawn from */
#define SPARK_VY_SCALE       0.5f     /* shrinks vertical launch speed (cells are tall) */
#define SPARK_UP_BIAS        0.3f     /* nudges the launch upward             */
#define SPARK_TEMP_MIN       0.95f    /* birth heat, plus a little jitter     */
#define SPARK_TEMP_JITTER    0.05f

/* Wind makes the top of the column sway slowly side to side. */
#define WIND_AMP_DEFAULT     3.0f     /* how far the top tilts, in cells      */
#define WIND_FREQ            0.45f    /* sway speed                            */

#define DT_CAP_S             0.10f
#define N_THEMES             4

/* Colour pair IDs */
#define PAIR_HEAT_0   1   /* coolest         */
#define PAIR_HEAT_1   2
#define PAIR_HEAT_2   3
#define PAIR_HEAT_3   4
#define PAIR_HEAT_4   5   /* hottest — white */
#define PAIR_HUD      6
#define PAIR_HINT     7

/* ── §2 clock — read the time, sleep for a bit ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 data — the heat/colour tables and the ember/spark/tornado types ── */

/* Colour for each heat level, coolest (0) to hottest (4), one row per theme.
 * The same heat_bucket(temp) picks both the colour here and the glyph below,
 * so a given heat always looks consistent. Theme 0 follows real fire colours
 * (dark red to orange to yellow to white). HEAT_256 uses xterm 256-colour
 * codes; HEAT_8 is the fallback for old 8-colour terminals. */
static const short HEAT_256[N_THEMES][5] = {
    /* classic fire */ {  52, 196, 208, 226, 231 },
    /* blue fire    */ {  17,  21,  39,  51, 231 },
    /* toxic green  */ {  22,  28,  76, 154, 231 },
    /* hellfire     */ {  53, 127, 165, 213, 231 },
};
static const short HEAT_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

/* The character drawn at each heat level: faint dot when cool, solid '#'
 * when white-hot. Same heat_bucket(temp) index as the colour table above
 * (Bourke, "character representation of greyscale images"). */
static const char HEAT_GLYPH[5] = { '`', '.', '*', 'o', '#' };

/* Ember — one glowing speck in the spinning column. We track where it is in
 * cylindrical terms: how high, what angle around the central axis, and how
 * far out from that axis. Every frame it rises, turns, drifts inward, and
 * cools; when it reaches the top or burns out it is reborn at the bottom.
 * Embers near the axis are given a faster rise and faster spin, which is what
 * makes the column taper into a funnel. There is no true 3-D: which side of
 * the axis the ember faces (front or back) is read off sin(phase) and used
 * only to pick a brighter or dimmer glyph. Lives in a fixed pool; `alive`
 * marks a slot as in use. */
typedef struct {
    float y;          /* height above the ground, in cells (0 = base)        */
    float phase;      /* angle around the axis, radians                      */
    float radius;     /* distance out from the axis, in cells                */
    float y_vel;      /* rise speed, cells/sec (set when born)               */
    float omega;      /* spin speed, radians/sec (set when born)             */
    float temp;       /* heat, 0 (cold) to 1 (white-hot)                     */
    int   alive;      /* 1 while burning, 0 = free slot                      */
} Ember;

/* Spark — a bit of ember that gets flung off the column and falls back down.
 * Purely decorative; the main column never looks at sparks. Each frame it is
 * pulled down by gravity, moved, and cooled. Unlike an Ember, a spark already
 * stores its position in plain screen coordinates (the cylinder-to-screen
 * conversion happens once when it is born), so drawing it is just rounding to
 * a cell. Lives in a fixed pool; `alive` marks a slot as in use. */
typedef struct {
    float x, y;       /* position on screen, in cells (rounded when drawn)   */
    float vx, vy;     /* velocity, cells/sec (vy bends downward over time)   */
    float temp;       /* heat, 0 to 1; cools quickly                         */
    int   alive;      /* 1 while in flight, 0 = free slot                    */
} Spark;

/* FlameMat — the flame strip along the ground, stored as one heat value per
 * screen column (a 1-D line, not a 2-D grid). This is the classic 1-D Doom
 * fire: each frame the strip cools, smears sideways, and gets fresh fuel
 * sprinkled near the middle (Sanglard, "Game Engine Black Book: DOOM" ch. 11).
 * Decorative only — the column never reads it back. */
typedef struct {
    float cell[BASE_HEAT_W];  /* heat 0..1 for each column; picks glyph + colour */
} FlameMat;

/* Tornado — the entire simulation bundled into one struct: the embers, the
 * decorative sparks and ground flame, where the column sits on screen, the
 * user-adjustable knobs, the clocks, and the chosen colour theme. Keeping it
 * all in one place means reset and resize can rebuild everything cleanly.
 * Everything is fixed-size; nothing is allocated after start-up. */
typedef struct {
    /* the things being drawn */
    Ember    embers[N_EMBERS_MAX];   /* the spinning column                  */
    Spark    sparks[N_SPARKS_MAX];   /* sparks flung outward (decorative)    */
    FlameMat base_heat;              /* ground flame strip (decorative)      */

    /* where the column sits on screen (set from terminal size on resize) */
    int   axis_x;                    /* centre column, in cells              */
    int   base_y;                    /* ground row, in cells                 */
    float height;                    /* column height, in cells              */

    /* knobs the user can change live */
    int   n;                         /* embers currently active [50..600]    */
    float height_frac;               /* height as a fraction of the screen   */
    float omega_mult;                /* spin-speed multiplier [0.25..4]      */
    float wind_amp;                  /* sway strength in cells; 0 = no wind   */

    /* timing + run state */
    float world_time;                /* seconds elapsed; drives the wind sway */
    float spark_accum;               /* leftover fraction of a spark to spawn */
    int   paused;                    /* 1 freezes motion; drawing continues  */

    /* look */
    int   theme;                     /* which colour ramp is selected        */
} Tornado;

/* ── §4 logic — small pure helpers that just compute a value ── */

/* random float in [0, 1) */
static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/* Turn a heat value (0..1) into one of the 5 ramp slots (0..4). */
static int heat_bucket(float temp)
{
    int b = (int)(temp * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

/* How far the column leans sideways at height y: nothing at the ground,
 * full sway at the top, swinging back and forth over time. */
static float wind_offset(float y, float height, float world_time, float wind_amp)
{
    if (height < 1.0f) return 0.0f;
    float t_norm = y / height;
    return wind_amp * sinf(world_time * WIND_FREQ) * t_norm;
}

/* Work out where an ember lands on screen from its height/angle/radius.
 * One shared converter so embers and the sparks born from them agree. */
static void ember_to_screen(const Ember *e, int axis_x, int base_y, float height,
                            float world_time, float wind_amp, float *sx, float *sy)
{
    float w_off = wind_offset(e->y, height, world_time, wind_amp);
    *sx = (float)axis_x + w_off + ASPECT_X * e->radius * cosf(e->phase);
    *sy = (float)base_y - e->y;
}

/* ── §5 ember — the spinning column: rise, turn, cool, be reborn ── */

/* Place an ember back at the bottom with a fresh random angle and distance.
 * Embers nearer the axis get a faster rise and faster spin (the funnel shape).
 * initial=1 also scatters them up the column so the screen is full on frame 0
 * (used at startup, reset, and when the shape changes). */
static void ember_respawn(Ember *e, float height, int initial)
{
    e->y      = initial ? frand() * height : 0.0f;
    e->phase  = frand() * 2.0f * (float)M_PI;
    float u   = frand();
    e->radius = BASE_RADIUS_CELLS * sqrtf(u);   /* sqrt spreads embers evenly over the disk */
    float r_norm = e->radius / BASE_RADIUS_CELLS;
    e->y_vel  = Y_VEL_BASE * (EMBER_RISE_AXIS - EMBER_RISE_FALLOFF * r_norm);
    e->omega  = OMEGA_BASE / fmaxf(e->radius * EMBER_OMEGA_RSCALE, EMBER_OMEGA_FLOOR);
    e->temp   = EMBER_TEMP_MIN + EMBER_TEMP_JITTER * frand();
    if (initial) e->temp *= (1.0f - EMBER_INIT_COOL * (e->y / height));
    e->alive  = 1;
}

/* Move one ember forward by dt: spin a bit, rise a bit, pull inward, cool.
 * Once it tops out or burns to nothing, mark it dead so it gets reborn. */
static void ember_tick(Ember *e, float height, float omega_mult, float dt)
{
    if (!e->alive) {
        ember_respawn(e, height, 0);
        return;
    }
    e->phase  += e->omega * omega_mult * dt;
    e->y      += e->y_vel * dt;
    e->radius *= (1.0f - RADIUS_DECAY * dt);
    e->temp   -= COOL_RATE * dt;
    if (e->y >= height || e->temp <= 0.0f) e->alive = 0;
}

/* ── §6 effects — the ground flame strip and the flying sparks ── */

/* Cool every cell of the flame strip a little. */
static void flame_decay(FlameMat *mat, float dt)
{
    float keep = 1.0f - BASE_HEAT_DECAY * dt;
    if (keep < 0.0f) keep = 0.0f;
    for (int i = 0; i < BASE_HEAT_W; i++)
        mat->cell[i] *= keep;
}

/* Smear the heat sideways: each cell becomes a weighted blend of itself and
 * its two neighbours. Edges reuse themselves so we never read past the ends.
 * Writes into scratch first so each cell sees the old values, not half-blurred ones. */
static void flame_blur(FlameMat *mat)
{
    float *heat = mat->cell;
    static float tmp[BASE_HEAT_W];
    for (int i = 0; i < BASE_HEAT_W; i++) {
        float l = (i > 0)              ? heat[i - 1] : heat[i];
        float r = (i < BASE_HEAT_W - 1) ? heat[i + 1] : heat[i];
        tmp[i] = 0.25f * l + 0.50f * heat[i] + 0.25f * r;
    }
    memcpy(heat, tmp, sizeof tmp);
}

/* Drop a few fresh hot spots onto the strip, clustered near the middle.
 * Adding two random numbers favours the centre, so the flame is hottest there. */
static void flame_inject(FlameMat *mat)
{
    float *heat  = mat->cell;
    int n_inject = BASE_INJECT_MIN + (rand() % (BASE_INJECT_MAX - BASE_INJECT_MIN + 1));
    float center = BASE_HEAT_W * 0.5f;
    float spread = BASE_HEAT_W * BASE_INJECT_SPREAD;
    for (int k = 0; k < n_inject; k++) {
        float u   = frand() + frand() - 1.0f;          /* centre-weighted, in [-1, 1] */
        int   idx = (int)(center + u * spread);
        if (idx < 0)             idx = 0;
        if (idx >= BASE_HEAT_W)  idx = BASE_HEAT_W - 1;
        float h = BASE_INJECT_HEAT + BASE_INJECT_JITTER * frand();
        if (heat[idx] < h) heat[idx] = h;
    }
}

/* One frame of the ground flame: cool it, smear it, sprinkle new fuel. */
static void base_heat_tick(FlameMat *mat, float dt)
{
    flame_decay(mat, dt);
    flame_blur(mat);
    flame_inject(mat);
}

/* Move one spark forward by dt: gravity speeds its fall, then it moves and
 * cools. It dies once it goes cold or leaves the screen. */
static void spark_tick(Spark *s, float dt, int rows, int cols)
{
    if (!s->alive) return;
    s->vy  += SPARK_GRAVITY * dt;
    s->x   += s->vx * dt;
    s->y   += s->vy * dt;
    s->temp -= SPARK_COOL * dt;
    if (s->temp <= 0.0f
        || s->x < 0 || s->x >= cols
        || s->y < 0 || s->y >= rows - 1)
        s->alive = 0;
}

/* Index of a free spark slot, or -1 if all are in use. */
static int spark_free_slot(const Spark *sparks)
{
    for (int i = 0; i < N_SPARKS_MAX; i++)
        if (!sparks[i].alive) return i;
    return -1;
}

/* Pick a random live ember to throw a spark from. Tries a few times and
 * gives up (returns -1) rather than scanning, since most embers are alive. */
static int random_live_ember(const Ember *embers, int n)
{
    for (int tries = 0; tries < SPARK_SRC_TRIES; tries++) {
        int k = rand() % n;
        if (embers[k].alive) return k;
    }
    return -1;
}

/* Spawn one spark: find a free slot and a live ember, start the spark at that
 * ember's screen spot, and launch it outward with a slight upward kick. */
static void tornado_spawn_spark(Tornado *t)
{
    int slot = spark_free_slot(t->sparks);
    if (slot < 0) return;
    int src = random_live_ember(t->embers, t->n);
    if (src < 0) return;
    const Ember *e = &t->embers[src];

    float x, y;
    ember_to_screen(e, t->axis_x, t->base_y, t->height, t->world_time, t->wind_amp, &x, &y);

    Spark *s = &t->sparks[slot];
    s->x = x;
    s->y = y;
    float ang   = frand() * 2.0f * (float)M_PI;
    float speed = SPARK_SPEED_MIN + frand() * (SPARK_SPEED_MAX - SPARK_SPEED_MIN);
    s->vx = cosf(ang) * speed * ASPECT_X;
    s->vy = sinf(ang) * speed * SPARK_VY_SCALE - speed * SPARK_UP_BIAS;
    s->temp  = SPARK_TEMP_MIN + SPARK_TEMP_JITTER * frand();
    s->alive = 1;
}

/* ── §7 init — place the column on screen and (re)seed all the particles ── */

/* Centre the column and size it to the current terminal. */
static void tornado_position(Tornado *t, int rows, int cols)
{
    t->axis_x = cols / 2;
    t->base_y = rows - 2;
    t->height = (float)(rows - 1) * t->height_frac;
}

/* Start fresh: scatter the embers up the column, clear the ground flame,
 * kill all sparks, and reset the clock. Used at startup, on 'r', or whenever
 * the shape changes. */
static void tornado_reseed(Tornado *t)
{
    for (int i = 0; i < t->n; i++)
        ember_respawn(&t->embers[i], t->height, /*initial=*/1);
    for (int i = 0; i < BASE_HEAT_W; i++)
        t->base_heat.cell[i] = 0.0f;
    for (int i = 0; i < N_SPARKS_MAX; i++)
        t->sparks[i].alive = 0;
    t->spark_accum = 0.0f;
    t->world_time  = 0.0f;
}

/* ── §8 combine — advance the whole simulation by one time step ── */

/* Step everything forward by dt. Sparks spawn at a fractional rate: we add up
 * the fraction owed each frame and spawn whole sparks as it crosses 1, so the
 * spawn rate stays steady no matter the frame rate. */
static void tornado_tick(Tornado *t, float dt, int rows, int cols)
{
    for (int i = 0; i < t->n; i++)
        ember_tick(&t->embers[i], t->height, t->omega_mult, dt);

    base_heat_tick(&t->base_heat, dt);

    for (int i = 0; i < N_SPARKS_MAX; i++)
        spark_tick(&t->sparks[i], dt, rows, cols);

    t->spark_accum += SPARK_SPAWN_HZ * dt;
    while (t->spark_accum >= 1.0f) {
        t->spark_accum -= 1.0f;
        tornado_spawn_spark(t);
    }

    t->world_time += dt;
}

/* ── §9 render — turn the simulation into characters on screen ── */

/* Set up the colour pairs for the current theme (falls back to 8 colours). */
static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    init_pair(PAIR_HUD,  x256 ? 226 : COLOR_YELLOW, -1);  /* bright yellow — top data bar    */
    init_pair(PAIR_HINT, x256 ?  51 : COLOR_CYAN,   -1);  /* bright cyan   — bottom hint bar  */
}

/* Draw one ember, but only on the matching depth pass. sin(phase) tells us
 * which side of the axis it faces; front-facing embers are drawn bright and
 * back-facing ones dim, which is what sells the rotation in a flat view. */
static void ember_draw(const Ember *e,
                       int axis_x, int base_y, float height,
                       float world_time, float wind_amp,
                       int rows, int cols, int back_pass)
{
    if (!e->alive) return;
    float depth = sinf(e->phase);
    int   is_back = (depth < 0.0f);
    if ( back_pass &&  !is_back) return;   /* this pass draws only the far side */
    if (!back_pass &&   is_back) return;   /* this pass draws only the near side */

    float sx, sy;
    ember_to_screen(e, axis_x, base_y, height, world_time, wind_amp, &sx, &sy);
    int sc = (int)sx, sr = (int)sy;
    if (sr < 0 || sr >= rows - 1) return;
    if (sc < 0 || sc >= cols)     return;

    int    bucket = heat_bucket(e->temp);
    chtype attr   = COLOR_PAIR(PAIR_HEAT_0 + bucket);
    attr |= is_back ? A_DIM : A_BOLD;
    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
    attroff(attr);
}

static void spark_draw(const Spark *s, int rows, int cols)
{
    if (!s->alive) return;
    int sr = (int)s->y, sc = (int)s->x;
    if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) return;
    int bucket = heat_bucket(s->temp);
    attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    mvaddch(sr, sc, (chtype)(unsigned char)SPARK_GLYPH);
    attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
}

static void base_heat_draw(const FlameMat *mat, int axis_x, int base_y,
                           int rows, int cols)
{
    if (base_y < 0 || base_y >= rows - 1) return;
    const float *heat = mat->cell;
    int half = BASE_HEAT_W / 2;
    for (int i = 0; i < BASE_HEAT_W; i++) {
        if (heat[i] < BASE_HEAT_VISIBLE) continue;
        int sc = axis_x - half + i;
        if (sc < 0 || sc >= cols) continue;
        int bucket = heat_bucket(heat[i]);
        attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
        mvaddch(base_y, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
        attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    }
}

/* Draw the two info bars: a title plus live stats on the top row, and the key
 * legend on the bottom row. Both are clipped so a narrow terminal can't wrap. */
static void draw_hud(const Tornado *t, double fps, int rows, int cols)
{
    char left[24], right[96];
    snprintf(left,  sizeof left,  " FIRE TORNADO ");
    snprintf(right, sizeof right,
             " embers:%d  spin:%.2fx  height:%.0f%%  wind:%s  theme:%d  %.0f fps  %s ",
             t->n, t->omega_mult, t->height_frac * 100.0f,
             (t->wind_amp > 0.001f) ? "on" : "off",
             t->theme, fps, t->paused ? "PAUSED" : "running");
    int rx = cols - (int)strlen(right);          /* where the right-aligned stats start */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(0, c, ' ');
    if (rx >= 0) {
        mvprintw(0, 0,  "%.*s", rx, left);       /* clip the title so it can't touch the stats */
        mvprintw(0, rx, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, right);    /* too narrow for both: stats only */
    }
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(rows - 1, c, ' ');
    mvprintw(rows - 1, 0, "%.*s", cols,
             " [/]:embers  -/+:spin  ,/.:height  w:wind  t:theme  r:reset  p:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Tornado *t, double fps)
{
    erase();

    /* Draw back to front so nearer things cover farther ones: ground flame,
       far embers, near embers, sparks on top, then the HUD. */
    base_heat_draw(&t->base_heat, t->axis_x, t->base_y, rows, cols);
    for (int i = 0; i < t->n; i++)
        ember_draw(&t->embers[i],
                   t->axis_x, t->base_y, t->height,
                   t->world_time, t->wind_amp,
                   rows, cols, /*back_pass=*/1);
    for (int i = 0; i < t->n; i++)
        ember_draw(&t->embers[i],
                   t->axis_x, t->base_y, t->height,
                   t->world_time, t->wind_amp,
                   rows, cols, /*back_pass=*/0);
    for (int i = 0; i < N_SPARKS_MAX; i++)
        spark_draw(&t->sparks[i], rows, cols);
    draw_hud(t, fps, rows, cols);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §10 screen — start up and tear down ncurses ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §11 app — signals, the main loop, and the keyboard ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Tornado g_tornado;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_tornado.n           = N_EMBERS_DEFAULT;
    g_tornado.height_frac = HEIGHT_FRAC;
    g_tornado.omega_mult  = OMEGA_MULT_DEFAULT;
    g_tornado.wind_amp    = WIND_AMP_DEFAULT;
    g_tornado.theme       = 0;

    screen_init(g_tornado.theme);

    int rows = LINES, cols = COLS;
    tornado_position(&g_tornado, rows, cols);
    tornado_reseed(&g_tornado);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            tornado_position(&g_tornado, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_tornado.paused ^= 1; break;
                case 'r':          tornado_reseed(&g_tornado); break;
                case 't':          g_tornado.theme = (g_tornado.theme + 1) % N_THEMES;
                                   color_init(g_tornado.theme); break;
                case 'w':
                    g_tornado.wind_amp = (g_tornado.wind_amp > 0.001f)
                                       ? 0.0f : WIND_AMP_DEFAULT;
                    break;
                case '[':
                    if (g_tornado.n - N_EMBERS_STEP >= N_EMBERS_MIN)
                        g_tornado.n -= N_EMBERS_STEP;
                    break;
                case ']':
                    if (g_tornado.n + N_EMBERS_STEP <= N_EMBERS_MAX) {
                        int old_n = g_tornado.n;
                        g_tornado.n += N_EMBERS_STEP;
                        for (int i = old_n; i < g_tornado.n; i++)
                            ember_respawn(&g_tornado.embers[i],
                                          g_tornado.height, /*initial=*/1);
                    }
                    break;
                case '-':
                    if (g_tornado.omega_mult - OMEGA_MULT_STEP >= OMEGA_MULT_MIN)
                        g_tornado.omega_mult -= OMEGA_MULT_STEP;
                    break;
                case '+': case '=':
                    if (g_tornado.omega_mult + OMEGA_MULT_STEP <= OMEGA_MULT_MAX)
                        g_tornado.omega_mult += OMEGA_MULT_STEP;
                    break;
                case ',':
                    if (g_tornado.height_frac - HEIGHT_FRAC_STEP >= HEIGHT_FRAC_MIN) {
                        g_tornado.height_frac -= HEIGHT_FRAC_STEP;
                        tornado_position(&g_tornado, rows, cols);
                    }
                    break;
                case '.':
                    if (g_tornado.height_frac + HEIGHT_FRAC_STEP <= HEIGHT_FRAC_MAX) {
                        g_tornado.height_frac += HEIGHT_FRAC_STEP;
                        tornado_position(&g_tornado, rows, cols);
                    }
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_tornado.paused) tornado_tick(&g_tornado, dt, rows, cols);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_tornado, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
