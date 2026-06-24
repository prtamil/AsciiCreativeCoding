/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_star_field_parallax_noise_showcase.c
 *   An infinite scrolling star field with a fake-3D depth effect, plus
 *   15 cosmic display modes (twinkle, warp, nebula, pulsar, meteors...).
 *
 * The trick: stacked "sheets" of stars scroll at different speeds, and the
 * eye reads the speed difference as depth. The stars aren't stored anywhere
 * — each screen cell asks a hash function "is there a star here?", so the
 * world is endless and free. Perlin/fBm noise paints the nebula clouds.
 *
 * Sister file: ../fields/perin_noise_flow_showcase.c uses noise to STEER
 * particles; this file uses a hash to PLACE stars (noise only for clouds).
 *
 * References for the ideas the code can't show on its own:
 *   • Teschner et al. (2003), "Optimized Spatial Hashing" — the three big
 *     primes in hash3 and the "star exists iff hash mod density == 0" trick.
 *   • Perlin (2002), "Improving Noise" — the quintic fade used in fade_q.
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   • Wikipedia, "Parallax scrolling" — depth from per-layer scroll speed.
 *   • Lode's Computer Graphics Tutorial — the plasma / tunnel / aurora ideas.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/procedural_star_field_parallax_noise_showcase.c \
 *     -o star_field -lncurses -lm
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

/* ── §1 config — constants, data tables (layers, glyphs, themes), Pattern ── */

enum {
    /* Biggest screen area we'll draw stars over; anything past this is
     * clipped. Nothing is stored per cell, so these are just safety caps. */
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,

    /* How many star sheets stack up for the depth effect. Four looks best. */
    N_LAYERS            =   4,

    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* Foreground scroll speed (cells per second). Deeper layers move slower
     * by their LAYER_SPEEDS fraction. */
    SCROLL_SPEED_MIN    =   1,
    SCROLL_SPEED_DEF    =   8,
    SCROLL_SPEED_MAX    =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Color pair slots. HUD/HINT are reserved project-wide; the star and
     * nebula tints each take a run of 4. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_STAR_BASE      =   3,    /* +0..+3 = 4 star tints           */
    PAIR_NEBULA_BASE    =   7,    /* +0..+3 = 4 nebula tints         */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/*
 * How fast each sheet scrolls, as a fraction of the user's speed knob.
 * Sheet 0 is the front (closest, full speed); higher sheets are deeper and
 * slower. Each is roughly 0.4x the one in front — that ratio gives the
 * cleanest depth illusion. Make them all equal and the depth effect vanishes.
 */
static const float LAYER_SPEEDS[N_LAYERS] = {
    1.00f,   /* front: full speed              */
    0.45f,
    0.18f,
    0.06f,   /* deep background: barely moves  */
};

/*
 * How crowded each sheet is: "1 star per N world cells." Smaller = denser.
 * The front sheet is sparse (a few big bright stars); deeper sheets are dense
 * (lots of faint specks) — the same way distant stars pile up in real skies.
 */
static const int LAYER_DENSITY[N_LAYERS] = {
    22,    /* front: roughly 1 star per 22 cells */
    16,
    11,
     7,    /* deep field: heavy faint sprinkle   */
};

/*
 * Four possible characters per sheet; which one a star gets is picked by two
 * bits of its hash. Front sheets use heavier glyphs and deep ones use lighter
 * marks, so glyph "weight" reinforces the sense of depth.
 */
static const char LAYER_GLYPHS[N_LAYERS][4] = {
    { '*', 'O', '+', 'o' },     /* front: bright */
    { '*', '+', 'o', '.' },
    { '.', '+', '.', ',' },
    { '.', '`', '.', '\'' },    /* deep: faint   */
};

/* In WARP mode the front stars switch to these streak characters. */
static const char WARP_GLYPHS[4] = { '=', '-', '~', '>' };

#define WARP_SPEED_MULT     5.0f
#define WARP_STREAK_LEN     3

/* How fast stars pulse in TWINKLE mode. About 2s per pulse feels natural;
 * much faster and it looks like a strobe. The thresholds split a pulse into
 * off / dim / bright. */
#define TWINKLE_HZ          0.5f
#define TWINKLE_OFF_THRESH  0.30f
#define TWINKLE_BOLD_THRESH 0.65f

/*
 * Nebula cloud settings.
 *   NEBULA_SCALE  — bigger smoother clouds when smaller (0.025 ≈ 40-cell blobs)
 *   NEBULA_SCROLL — cloud drift speed; below the deepest star sheet so the
 *                   clouds read as the farthest thing in the scene
 *   FBM_OCTAVES   — noise detail layers; 4 is the usual sweet spot
 *   *_THRESH      — cutoffs turning a noise value into a glyph
 */
#define NEBULA_SCALE        0.025f
#define NEBULA_SCROLL       0.03f
#define FBM_OCTAVES         4
#define NEBULA_DOT_THRESH   0.50f
#define NEBULA_MID_THRESH   0.62f
#define NEBULA_HI_THRESH    0.74f

/* Terminal cells are about twice as tall as wide; stretch vertical distances
 * by this so circles and rings come out round instead of squashed. */
#define SF_ASPECT           2.0f

/* TUNNEL — front stars streak outward from the centre, longer near the edges. */
#define TUNNEL_STREAK_MAX   7

/* STARFALL — stars rain downward with a short trail. */
#define FALL_STREAK_LEN     4

/* WORMHOLE — how hard the field twists, how fast it spins, and how quickly the
 * twist fades as you move away from the centre. */
#define WORM_SWIRL          5.0f
#define WORM_SPIN           0.30f
#define WORM_FALLOFF        0.08f

/* PULSAR — brightness rings marching out from the centre: speed, gap, width. */
#define PULSAR_SPEED        24.0f
#define PULSAR_SPACING      14.0f
#define PULSAR_RING_W        1.6f

/* SUPERNOVA — a few spots that flash into an expanding, fading shock ring. */
#define NOVA_SITES           3
#define NOVA_PERIOD          3.6f      /* seconds between flashes */
#define NOVA_SPEED          22.0f
#define NOVA_RING_W          1.8f

/* METEORS — a handful of fast diagonal streaks. */
#define METEOR_COUNT         6
#define METEOR_LEN          12
#define METEOR_PERIOD        2.2f      /* seconds per meteor */

/* MILKYWAY — a glowing diagonal dust band: how thick, and how tilted. */
#define BAND_HALF_W          6.0f
#define BAND_SLOPE           0.5f

/* PLASMA — full-screen ripple made by adding sine waves. */
#define PLASMA_SCALE         0.13f
#define PLASMA_SPEED         1.4f

/* AURORA — flowing curtains of noise. */
#define AURORA_SCALE         0.07f
#define AURORA_SPEED         0.6f

/* QUASAR — a bright core with two vertical jets. */
#define QUASAR_CORE_R        4.0f
#define QUASAR_JET_HALF_W    1.5f

/*
 * The 15 display modes, cycled with n / p. The first four show the plain
 * scrolling field; the rest add motion, rings, meteors, a galaxy band, or
 * replace the field entirely with a full-screen effect.
 */
typedef enum {
    PATTERN_STARFIELD = 0,    /* classic parallax scroll                  */
    PATTERN_TWINKLE,          /* stars pulse in/out                       */
    PATTERN_NEBULA,           /* drifting fBm dust clouds + stars         */
    PATTERN_WARP,             /* linear hyperspace streaks                */
    PATTERN_TUNNEL,           /* radial fly-through (streaks from centre) */
    PATTERN_STARFALL,         /* vertical star rain                       */
    PATTERN_WORMHOLE,         /* spiral swirl of the whole field          */
    PATTERN_REDSHIFT,         /* stars tinted by depth (near→far gradient)*/
    PATTERN_PULSAR,           /* expanding concentric brightness rings    */
    PATTERN_SUPERNOVA,        /* detonating, fading shock rings           */
    PATTERN_METEORS,          /* sparse fast diagonal meteor streaks      */
    PATTERN_MILKYWAY,         /* luminous diagonal dust band              */
    PATTERN_PLASMA,           /* full-screen interference plasma          */
    PATTERN_AURORA,           /* flowing horizontal noise curtains        */
    PATTERN_QUASAR,           /* nucleus + vertical relativistic jets     */
    N_PATTERNS,
} Pattern;

/*
 * One colour scheme. Each theme carries four star colours (the hash picks one
 * per star) and four nebula colours (one per cloud-density band). The HUD
 * colours don't change with the theme.
 *   name   — shown in the HUD
 *   star   — the four star tints
 *   nebula — the four cloud tints, faint to dense
 */
typedef struct {
    const char *name;
    short       star  [4];
    short       nebula[4];
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name      star{0,1,2,3}              nebula{0,1,2,3}            */
    { "DEFAULT", { 231, 226, 117,  33 }, {  17,  19,  62, 105 } },
    { "MATRIX",  { 230, 226, 118,  46 }, {  22,  28,  34,  40 } },
    { "NOVA",    { 231, 219, 201, 129 }, {  53,  54,  91, 165 } },
    { "MONO",    { 254, 250, 244, 240 }, { 234, 236, 238, 240 } },
    { "OCEAN",   { 231, 159,  51,  39 }, {  17,  18,  24,  31 } },
    { "FIRE",    { 231, 226, 208, 196 }, {  52,  88, 124, 160 } },
    { "EARTH",   { 230, 222, 173, 100 }, {  58,  64, 100, 137 } },
    { "FOREST",  { 231, 156, 118,  64 }, {  22,  28,  29,  65 } },
    { "DESERT",  { 230, 222, 173, 130 }, {  94,  95, 130, 137 } },
    { "ARCTIC",  { 231, 195, 159,  39 }, {  17,  18,  19,  24 } },
};

/* ── §2 performance — timing primitives (frame pacing lives in main, §8) ── */

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

/* ── §3 logic — pure decisions: hash, star test, perlin/fbm, name map ── */

/* These functions just compute answers — they never change any state or touch
 * the screen. The only data they read, perm[], is a fixed table that only the
 * reset code reshuffles, so the order cells are drawn in can't affect them. */

/* Names are padded to 9 chars so the HUD column never jumps width. */
static const char *pattern_name(Pattern p)
{
    static const char *names[N_PATTERNS] = {
        "STARFIELD", "TWINKLE  ", "NEBULA   ", "WARP     ", "TUNNEL   ",
        "STARFALL ", "WORMHOLE ", "REDSHIFT ", "PULSAR   ", "SUPERNOVA",
        "METEORS  ", "MILKYWAY ", "PLASMA   ", "AURORA   ", "QUASAR   ",
    };
    return ((int)p >= 0 && (int)p < N_PATTERNS) ? names[p] : "?        ";
}

/*
 * Turn three integers into one scrambled 32-bit number — same inputs always
 * give the same output. This is what lets stars exist "everywhere and nowhere":
 * we don't store them, we just ask this function. The three big primes and the
 * shift-and-multiply scramble (Teschner et al. 2003) make neighbouring cells
 * give very different results, so the stars don't fall into visible stripes.
 * Plain rand() can't do this — it carries state and wouldn't repeat the same
 * star at the same spot every frame.
 */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/*
 * Is there a star at this world cell on sheet L? We hash the cell and say yes
 * when the hash lands on a multiple of the sheet's density. When yes, we hand
 * back the full hash through out_h so the caller can read the star's character,
 * colour and twinkle from the same number — that keeps each star looking
 * identical every time the camera passes it.
 */
static inline bool star_at(int wx, int wy, int L, uint32_t *out_h)
{
    uint32_t h = hash3(wx, wy, L);
    *out_h = h;
    return (h % (uint32_t)LAYER_DENSITY[L]) == 0u;
}

/*
 * Perlin noise: smooth, natural-looking randomness used to paint the nebula
 * clouds. Copied inline from perin_noise_flow_showcase.c (this project keeps
 * each file self-contained); see that file for the full walk-through.
 *
 * perm[] is the shuffled lookup table the noise is built on — stored twice
 * back to back so the lookups never need a wraparound. perlin2d returns a
 * smooth value in roughly [-1, 1].
 */
static uint8_t perm[512];

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }

static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    float u = fade_q(x);
    float v = fade_q(y);

    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;

    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);

    return lerp_f(
        lerp_f(n00, n10, u),
        lerp_f(n01, n11, u),
        v);
}

/*
 * Layered Perlin noise (fBm). Adds several copies of the noise on top of each
 * other — each finer and fainter than the last — to get cloud detail at many
 * sizes at once. Scaled back to roughly [0, 1] so the cloud thresholds stay
 * meaningful.
 */
static float fbm2(float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* How bright a twinkling star is right now (0..1). Each star gets its own
 * starting offset from its hash, so neighbours pulse out of step instead of
 * blinking together. */
static inline float twinkle_brightness(uint32_t h, float time_secs)
{
    float phase = ((float)((h >> 24) & 0xFFu) / 255.0f) * 2.0f * (float)M_PI;
    return 0.5f + 0.5f * sinf(2.0f * (float)M_PI * TWINKLE_HZ * time_secs + phase);
}

/* WORMHOLE twist: spin a cell around the centre by an angle that grows the
 * closer it is to the middle (and slowly turns over time), so the whole field
 * looks like it's spiralling inward. The twisted spot to sample is written to
 * *esx, *esy. */
static inline void swirl_cell(int sx, int sy, float cx, float cy, float time_secs,
                              float *esx, float *esy)
{
    float dx = (float)sx - cx, dy = ((float)sy - cy) * SF_ASPECT;
    float rr = sqrtf(dx * dx + dy * dy);
    float ang = atan2f(dy, dx)
              + WORM_SWIRL / (rr * WORM_FALLOFF + 1.0f)
              + time_secs * WORM_SPIN;
    *esx = cx + rr * cosf(ang);
    *esy = cy + rr * sinf(ang) / SF_ASPECT;
}

/* ── §4 simulation — advances state (the only code that changes the world) ── */

/* Everything that moves the world forward lives here: scene_tick nudges the
 * camera and clock once per step, and scene_reset / scene_init set the starting
 * state. Nothing else in the file changes simulation state (key presses do, but
 * they run outside the tick — see §8). */

static void perm_shuffle(void)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

/*
 * The viewpoint flying through the field. Position drifts by velocity every
 * tick; the user scales the velocity with +/-. It heads mostly rightward with
 * a little downward drift so the depth effect shows on both axes.
 *   x, y   — where the camera is now, in world cells
 *   vx, vy — how fast it moves (cells/sec), before the speed knob and the
 *            per-sheet slowdown are applied
 */
typedef struct {
    float x, y;
    float vx, vy;
} Camera;

/*
 * Everything that has to persist between frames. There's no grid of cells —
 * the picture is recomputed from the camera, the chosen mode and the clock
 * every frame. Fields are grouped by what they mean, not by which key changes
 * them. scene_tick is the only thing that advances these; key presses set the
 * knobs and selections.
 */
typedef struct {
    Camera  cam;              /* the moving viewpoint                     */
    int     speed;            /* scroll-speed knob, 1..SCROLL_SPEED_MAX   */
    Pattern current_pattern;  /* which of the 15 modes is showing (n/p)   */
    int     current_theme;    /* which colour scheme (t/T)                */
    float   time_secs;        /* running clock — drives twinkle/effects   */
    bool    paused;           /* freeze the world; drawing keeps going    */
} Scene;

/*
 * Snap the camera back to the origin and reshuffle the noise. This is what 'r'
 * does, and it's also how the world gets set up the first time.
 */
static void scene_reset(Scene *s)
{
    s->cam.x = 0.0f;
    s->cam.y = 0.0f;
    /* Mostly rightward with a touch of downward, so the depth effect shows on
     * both axes. The speed knob just scales this whole direction later. */
    s->cam.vx = 1.00f;
    s->cam.vy = 0.18f;
    s->time_secs = 0.0f;
    perm_shuffle();
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SCROLL_SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_STARFIELD;
    scene_reset(s);
}

/* Move the camera and clock forward by one time step. No drawing here. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Speed knob, with an extra kick in WARP mode. */
    float speed_mul = (float)s->speed;
    if (s->current_pattern == PATTERN_WARP) speed_mul *= WARP_SPEED_MULT;

    s->cam.x += s->cam.vx * speed_mul * dt;
    s->cam.y += s->cam.vy * speed_mul * dt;

    s->time_secs += dt;
}

/* ── §5 effects — none stored ──
 * The trails, the twinkle, and the ring/meteor overlays are all worked out
 * fresh while drawing (§7) from the star's position and the clock. There's no
 * glow buffer to keep around. */

/* ── §6 delays — none ──
 * The only timing control is the pause toggle, which just stops scene_tick.
 * The field scrolls and the effects animate continuously otherwise. */

/* ── §7 render — turn the current state into a screenful of characters ── */

/* Drawing only: read the camera/clock/mode and paint. scene_draw picks the
 * right path — a full-screen effect, or the scrolling star field plus an
 * optional overlay — and screen_draw lays the HUD on top. The only things
 * written here are the ncurses screen and the colour table. */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_STAR_BASE   + i), t->star  [i], -1);
            init_pair((short)(PAIR_NEBULA_BASE + i), t->nebula[i], -1);
        }
    } else {
        /* Plain 8-colour terminals: pick distinct hues that work everywhere. */
        static const short fb_star[4]   = { COLOR_WHITE,   COLOR_YELLOW,
                                            COLOR_CYAN,    COLOR_BLUE };
        static const short fb_nebula[4] = { COLOR_BLUE,    COLOR_BLUE,
                                            COLOR_MAGENTA, COLOR_CYAN };
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_STAR_BASE   + i), fb_star  [i], -1);
            init_pair((short)(PAIR_NEBULA_BASE + i), fb_nebula[i], -1);
        }
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

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

/* WARP: a short fading streak trailing to the right behind a front star. */
static void draw_warp_trail(int sy, int sx, int pair, int cols)
{
    for (int d = 1; d <= WARP_STREAK_LEN; d++) {
        int tx = sx + d;
        if (tx >= cols) break;
        char tg = (d == 1) ? '-' : (d == 2) ? '~' : '.';
        int  ta = (d == 1) ? A_BOLD : (d == 2) ? A_NORMAL : A_DIM;
        attron(COLOR_PAIR(pair) | ta);
        mvaddch(sy, tx, (chtype)(unsigned char)tg);
        attroff(COLOR_PAIR(pair) | ta);
    }
}

/* TUNNEL: a streak pointing straight out from the centre, longer near the
 * edges — the look of flying forward toward a vanishing point. */
static void draw_tunnel_trail(int sy, int sx, int pair, float cx, float cy,
                              int rows, int cols)
{
    float dx = (float)sx - cx, dy = (float)sy - cy;
    float len = sqrtf(dx * dx + dy * dy);
    if (len <= 0.5f) return;
    float ux = dx / len, uy = dy / len;
    float maxlen = sqrtf(cx * cx + cy * cy) + 1.0f;
    int slen = 1 + (int)(len / maxlen * TUNNEL_STREAK_MAX);
    for (int d = 1; d <= slen; d++) {
        int tx = sx + (int)lroundf(ux * (float)d);
        int ty = sy + (int)lroundf(uy * (float)d);
        if (tx < 0 || tx >= cols || ty < 2 || ty >= rows - 1) break;
        char tg = (d <= 1) ? '-' : (d <= 3) ? ':' : '.';
        int  ta = (d <= 1) ? A_BOLD : (d <= 3) ? A_NORMAL : A_DIM;
        attron(COLOR_PAIR(pair) | ta);
        mvaddch(ty, tx, (chtype)(unsigned char)tg);
        attroff(COLOR_PAIR(pair) | ta);
    }
}

/* STARFALL: a short trail hanging below a star, like rain. */
static void draw_fall_trail(int sy, int sx, int pair, int rows)
{
    for (int d = 1; d <= FALL_STREAK_LEN; d++) {
        int ty = sy + d;
        if (ty >= rows - 1) break;
        char tg = (d == 1) ? '|' : (d == 2) ? ':' : '.';
        int  ta = (d == 1) ? A_NORMAL : A_DIM;
        attron(COLOR_PAIR(pair) | ta);
        mvaddch(ty, sx, (chtype)(unsigned char)tg);
        attroff(COLOR_PAIR(pair) | ta);
    }
}

/*
 * Paint one star. Its colour and character come straight from the hash, then
 * the current mode tweaks it (REDSHIFT tints by depth, WARP swaps the glyph,
 * TWINKLE dims/brightens it), and any trail is added. Returns false only when
 * TWINKLE has this star in its dark phase — that tells the caller to keep
 * looking at the sheets behind it.
 *   sy, sx — screen cell   layer — 0 (front) .. N_LAYERS-1 (back)   h — its hash
 */
static bool draw_star_at(int sy, int sx, int layer, uint32_t h,
                         Pattern pattern, float time_secs,
                         int rows, int cols, float cx, float cy)
{
    int color_idx = (int)((h >> 16) & 3u);
    int glyph_idx = (int)((h >>  8) & 3u);

    /* Front sheets are bold, back ones dim — that's the depth cue. In REDSHIFT
     * the colour is set by depth instead. */
    int attr = A_NORMAL;
    if      (layer == 0) attr = A_BOLD;
    else if (layer == 3) attr = A_DIM;
    if (pattern == PATTERN_REDSHIFT) color_idx = layer;

    /* In WARP the front stars become streak heads. */
    char glyph = (pattern == PATTERN_WARP && layer == 0)
               ? WARP_GLYPHS[glyph_idx] : LAYER_GLYPHS[layer][glyph_idx];

    /* In TWINKLE, a star in its dark phase bows out so a sheet behind shows. */
    if (pattern == PATTERN_TWINKLE) {
        float b = twinkle_brightness(h, time_secs);
        if (b < TWINKLE_OFF_THRESH)       return false;
        else if (b < TWINKLE_BOLD_THRESH) attr = A_DIM;
        else                              attr = A_BOLD;
    }

    int pair = PAIR_STAR_BASE + color_idx;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);

    /* Add a motion trail, with the direction set by the mode. */
    if      (pattern == PATTERN_WARP     && layer == 0) draw_warp_trail(sy, sx, pair, cols);
    else if (pattern == PATTERN_TUNNEL   && layer == 0) draw_tunnel_trail(sy, sx, pair, cx, cy, rows, cols);
    else if (pattern == PATTERN_STARFALL && layer <= 1) draw_fall_trail(sy, sx, pair, rows);

    return true;
}

/*
 * Paint a bit of nebula cloud in one empty cell. The noise value picks how
 * thick the cloud looks here: below the lowest cutoff the cell stays empty,
 * otherwise it steps up through '.', '*', '#' as the cloud gets denser.
 */
static void draw_nebula_cell(int sy, int sx, float wx, float wy)
{
    float n = fbm2(wx * NEBULA_SCALE, wy * NEBULA_SCALE);

    char glyph;
    int  color_idx;
    int  attr = A_NORMAL;

    if (n < NEBULA_DOT_THRESH) return;

    if      (n < NEBULA_MID_THRESH) { glyph = '.'; color_idx = 0; attr = A_DIM;  }
    else if (n < NEBULA_HI_THRESH ) { glyph = '*'; color_idx = 1;                }
    else                            { glyph = '#'; color_idx = 2; attr = A_BOLD; }

    /* The very densest spots get the fourth, brightest cloud tint. */
    if (n > 0.86f) { color_idx = 3; attr = A_BOLD; glyph = '#'; }

    int pair = PAIR_NEBULA_BASE + color_idx;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/* MILKYWAY: the same cloud dust, but only inside a tilted diagonal band that
 * fades out toward its edges. */
static void draw_band_cell(int sy, int sx, float wx, float wy, float cx, float cy)
{
    float dband = ((float)sy - cy) * SF_ASPECT - BAND_SLOPE * ((float)sx - cx);
    if (fabsf(dband) > BAND_HALF_W) return;
    float falloff = 1.0f - fabsf(dband) / BAND_HALF_W;           /* brightest on the band's centre-line */
    float n = fbm2(wx * NEBULA_SCALE * 1.6f, wy * NEBULA_SCALE * 1.6f) * falloff;
    if (n < 0.28f) return;

    char glyph; int color_idx, attr = A_NORMAL;
    if      (n < 0.42f) { glyph = '.'; color_idx = 0; attr = A_DIM;  }
    else if (n < 0.55f) { glyph = '*'; color_idx = 1;               }
    else                { glyph = '#'; color_idx = 3; attr = A_BOLD; }
    int pair = PAIR_NEBULA_BASE + color_idx;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * draw_star_field — the parallax scan shared by every star-based mode. For
 * each cell, scan layers front-to-back; the first hit wins (foreground stars
 * occlude deeper ones, and most cells terminate on layer 0/1). WORMHOLE twists
 * the sample position around the centre (swirl_cell); NEBULA / MILKYWAY fill
 * empty cells with dust. For the plain modes it is just the parallax scan.
 */
static void draw_star_field(const Camera *cam, Pattern pattern, float time_secs,
                            int top, int bottom, int width, int rows, int cols)
{
    Pattern p = pattern;
    float cx = (float)width * 0.5f;
    float cy = (float)(top + bottom) * 0.5f;

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {

            /* The cell we sample the field at — WORMHOLE swirls it inward. */
            float esx = (float)sx, esy = (float)sy;
            if (p == PATTERN_WORMHOLE)
                swirl_cell(sx, sy, cx, cy, time_secs, &esx, &esy);

            bool drew = false;
            for (int L = 0; L < N_LAYERS; L++) {
                /* Per-layer world coord — FLOOR (not cast) to avoid a jump at 0. */
                float wxf = esx + cam->x * LAYER_SPEEDS[L];
                float wyf = esy + cam->y * LAYER_SPEEDS[L];
                int   wx  = (int)floorf(wxf);
                int   wy  = (int)floorf(wyf);

                uint32_t h;
                if (!star_at(wx, wy, L, &h)) continue;

                if (draw_star_at(sy, sx, L, h, p, time_secs,
                                 rows, cols, cx, cy)) {
                    drew = true;
                    break;          /* foreground wins */
                }
                /* TWINKLE off-cycle: continue scanning behind. */
            }

            if (!drew && p == PATTERN_NEBULA) {
                float wxf = (float)sx + cam->x * NEBULA_SCROLL;
                float wyf = (float)sy + cam->y * NEBULA_SCROLL;
                draw_nebula_cell(sy, sx, wxf, wyf);
            } else if (!drew && p == PATTERN_MILKYWAY) {
                float wxf = (float)sx + cam->x * NEBULA_SCROLL;
                float wyf = (float)sy + cam->y * NEBULA_SCROLL;
                draw_band_cell(sy, sx, wxf, wyf, cx, cy);
            }
        }
    }
}

/* overlay_pulsar — concentric brightness rings marching outward from centre. */
static void overlay_pulsar(float time_secs, int top, int bottom, int width)
{
    float cx = (float)width * 0.5f, cy = (float)(top + bottom) * 0.5f;
    float march = fmodf(time_secs * PULSAR_SPEED, PULSAR_SPACING);
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float dx = (float)sx - cx, dy = ((float)sy - cy) * SF_ASPECT;
            float rr = sqrtf(dx * dx + dy * dy);
            float ph = fmodf(rr - march, PULSAR_SPACING);
            if (ph < 0.0f) ph += PULSAR_SPACING;
            if (ph < PULSAR_RING_W || ph > PULSAR_SPACING - PULSAR_RING_W) {
                int  pair  = PAIR_STAR_BASE + ((int)(rr / PULSAR_SPACING) & 3);
                char glyph = (ph < PULSAR_RING_W * 0.5f
                              || ph > PULSAR_SPACING - PULSAR_RING_W * 0.5f) ? 'O' : 'o';
                attron(COLOR_PAIR(pair) | A_BOLD);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(pair) | A_BOLD);
            }
        }
    }
}

/* overlay_supernova — a few hashed sites, each detonating into an expanding,
 * fading shock ring on its own clock. */
static void overlay_supernova(float time_secs, int top, int bottom, int width)
{
    int span_w = width > 1 ? width : 1;
    int span_h = (bottom - top) > 1 ? (bottom - top) : 1;
    for (int k = 0; k < NOVA_SITES; k++) {
        uint32_t hs = hash3(k * 97 + 13, k * 61 + 7, 1234);
        float sxc = (float)(hs % (uint32_t)span_w);
        float syc = (float)(top + (int)((hs >> 8) % (uint32_t)span_h));
        float age = fmodf(time_secs + (float)(hs & 0xFFu) / 255.0f * NOVA_PERIOD,
                          NOVA_PERIOD);
        float ring_r = age * NOVA_SPEED;
        float fade   = 1.0f - age / NOVA_PERIOD;       /* 1 (fresh) → 0 (faded) */
        if (fade < 0.10f) continue;

        for (int sy = top; sy < bottom; sy++) {
            for (int sx = 0; sx < width; sx++) {
                float dx = (float)sx - sxc, dy = ((float)sy - syc) * SF_ASPECT;
                float rr = sqrtf(dx * dx + dy * dy);
                if (fabsf(rr - ring_r) >= NOVA_RING_W) continue;
                int  attr  = fade > 0.6f ? A_BOLD : fade > 0.3f ? A_NORMAL : A_DIM;
                char glyph = fade > 0.6f ? '#'    : fade > 0.3f ? '*'      : '.';
                int  pair  = PAIR_NEBULA_BASE + (k & 3);
                attron(COLOR_PAIR(pair) | attr);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(pair) | attr);
            }
        }
    }
}

/* overlay_meteors — sparse fast diagonal streaks, each on its own phase clock. */
static void overlay_meteors(float time_secs, int top, int bottom, int width)
{
    int span_h = (bottom - top) > 1 ? (bottom - top) : 1;
    for (int m = 0; m < METEOR_COUNT; m++) {
        uint32_t hm = hash3(m * 131 + 5, m * 89 + 3, 77);
        float phase   = (float)(hm & 0xFFFFu) / 65535.0f;
        float t       = fmodf(time_secs / METEOR_PERIOD + phase, 1.0f);  /* 0..1 */
        float head_x  = t * (float)(width + METEOR_LEN) - (float)METEOR_LEN;
        float start_y = (float)(top + (int)((hm >> 16) % (uint32_t)span_h));
        float head_y  = start_y + t * 6.0f;                                  /* slight fall */
        for (int d = 0; d < METEOR_LEN; d++) {
            int tx = (int)(head_x - (float)d);
            int ty = (int)(head_y - (float)d * 0.4f);
            if (tx < 0 || tx >= width || ty < top || ty >= bottom) continue;
            char glyph = d == 0 ? '@' : d < 3 ? '*' : d < 6 ? '-' : '.';
            int  attr  = d == 0 ? A_BOLD : d < 6 ? A_NORMAL : A_DIM;
            int  pair  = PAIR_STAR_BASE + (m & 3);
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(ty, tx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* draw_plasma — full-screen animated interference field (sum of sines). Fills
 * every cell, so there is no star scan; colour + glyph track the field value. */
static void draw_plasma(float time_secs, int top, int bottom, int width)
{
    float t = time_secs * PLASMA_SPEED;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float fx = (float)sx * PLASMA_SCALE, fy = (float)sy * PLASMA_SCALE;
            float v = sinf(fx + t)
                    + sinf(fy * 1.3f + t * 0.9f)
                    + sinf((fx + fy) * 0.7f + t * 1.1f)
                    + sinf(sqrtf(fx * fx + fy * fy) * 0.9f + t);
            float u = (v + 4.0f) / 8.0f;                     /* → [0,1] */
            int  idx   = (int)(u * 3.999f);
            char glyph = u < 0.25f ? '.' : u < 0.50f ? '*' : u < 0.75f ? '#' : '@';
            int  attr  = u < 0.30f ? A_DIM : u > 0.70f ? A_BOLD : A_NORMAL;
            int  pair  = PAIR_NEBULA_BASE + (idx & 3);
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* draw_aurora — flowing vertical curtains: fBm stretched tall and drifting
 * upward, modulated by a horizontal shimmer. Dark sky shows between curtains. */
static void draw_aurora(float time_secs, int top, int bottom, int width)
{
    float t = time_secs * AURORA_SPEED;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float n = fbm2((float)sx * AURORA_SCALE,
                           (float)sy * AURORA_SCALE * 3.0f - t);
            float shimmer = 0.5f + 0.5f * sinf((float)sx * 0.08f + t * 2.0f + n * 4.0f);
            float v = n * shimmer;
            if (v < 0.18f) continue;
            int  idx   = v < 0.30f ? 0 : v < 0.45f ? 1 : v < 0.60f ? 2 : 3;
            char glyph = v < 0.30f ? '.' : v < 0.45f ? ':' : v < 0.60f ? '|' : '#';
            int  attr  = v < 0.30f ? A_DIM : v > 0.60f ? A_BOLD : A_NORMAL;
            int  pair  = PAIR_NEBULA_BASE + idx;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* draw_quasar — a brilliant nucleus with two vertical pulsing jets, a faint
 * accretion sprinkle filling the rest. */
static void draw_quasar(float time_secs, int top, int bottom, int width)
{
    float cx = (float)width * 0.5f, cy = (float)(top + bottom) * 0.5f;
    float span = (float)(bottom - top);
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float dx = (float)sx - cx, dy = ((float)sy - cy) * SF_ASPECT;
            float rr = sqrtf(dx * dx + dy * dy);

            if (rr < QUASAR_CORE_R) {                         /* nucleus */
                int  attr  = rr < QUASAR_CORE_R * 0.5f ? A_BOLD : A_NORMAL;
                char glyph = rr < QUASAR_CORE_R * 0.5f ? '@' : '#';
                attron(COLOR_PAIR(PAIR_STAR_BASE) | attr);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(PAIR_STAR_BASE) | attr);
            } else if (fabsf((float)sx - cx) < QUASAR_JET_HALF_W) {   /* jets */
                float jf    = 1.0f - rr / span;
                float pulse = 0.5f + 0.5f * sinf(rr * 0.3f - time_secs * 4.0f);
                float b     = jf * pulse;
                if (b > 0.08f) {
                    int  attr  = b > 0.4f ? A_BOLD : b > 0.2f ? A_NORMAL : A_DIM;
                    char glyph = b > 0.4f ? '|' : ':';
                    attron(COLOR_PAIR(PAIR_NEBULA_BASE + 3) | attr);
                    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                    attroff(COLOR_PAIR(PAIR_NEBULA_BASE + 3) | attr);
                }
            } else {                                          /* faint sprinkle */
                uint32_t h = hash3(sx, sy, 909);
                if ((h % 40u) == 0u) {
                    int pair = PAIR_STAR_BASE + (int)((h >> 8) & 3u);
                    attron(COLOR_PAIR(pair) | A_DIM);
                    mvaddch(sy, sx, '.');
                    attroff(COLOR_PAIR(pair) | A_DIM);
                }
            }
        }
    }
}

/*
 * scene_draw — dispatch the active pattern. Full-screen field modes (plasma /
 * aurora / quasar) replace the star scan entirely; everything else renders the
 * parallax star field, then layers a timed overlay (pulsar / supernova /
 * meteors) on top.
 */
static void scene_draw(const Camera *cam, Pattern pattern, float time_secs,
                       int cols, int rows)
{
    /* Reserve top 2 rows for HUD, bottom 1 for hint. */
    int top    = 2;
    int bottom = rows - 1;
    int width  = cols;
    if (width  > MAP_W_MAX) width  = MAP_W_MAX;
    if (bottom - top > MAP_H_MAX) bottom = top + MAP_H_MAX;

    switch (pattern) {
    case PATTERN_PLASMA: draw_plasma(time_secs, top, bottom, width); return;
    case PATTERN_AURORA: draw_aurora(time_secs, top, bottom, width); return;
    case PATTERN_QUASAR: draw_quasar(time_secs, top, bottom, width); return;
    default: break;
    }

    draw_star_field(cam, pattern, time_secs, top, bottom, width, rows, cols);

    switch (pattern) {
    case PATTERN_PULSAR:    overlay_pulsar   (time_secs, top, bottom, width); break;
    case PATTERN_SUPERNOVA: overlay_supernova(time_secs, top, bottom, width); break;
    case PATTERN_METEORS:   overlay_meteors  (time_secs, top, bottom, width); break;
    default: break;
    }
}

/* Draw a 4-tint palette swatch at row 1, col x; returns the next free column. */
static int draw_swatch(int x, int base_pair, char glyph, int attr)
{
    for (int i = 0; i < 4; i++) {
        attron(COLOR_PAIR(base_pair + i) | attr);
        mvaddch(1, x, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(base_pair + i) | attr);
        x++;
    }
    return x;
}

/* Row 0 right — primary status: fps, sim Hz, mode/pause, speed. Right-aligned. */
static void draw_status_line(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED   " : pattern_name(s->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern (n/N counter), theme, the star + nebula tint swatches, and
 * the camera position. Fixed left-aligned layout. */
static void draw_param_line(const Scene *s)
{
    int x = 1;
    char pbuf[40];
    snprintf(pbuf, sizeof pbuf, " pattern:%s %d/%d ",
             pattern_name(s->current_pattern),
             (int)s->current_pattern + 1, (int)N_PATTERNS);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, "%s", pbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += (int)strlen(pbuf);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " stars:");  attroff(COLOR_PAIR(PAIR_HUD));
    x = draw_swatch(x + 7, PAIR_STAR_BASE, '*', A_BOLD);
    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " neb:");    attroff(COLOR_PAIR(PAIR_HUD));
    x = draw_swatch(x + 5, PAIR_NEBULA_BASE, '#', A_NORMAL);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  cam:(%7.1f,%6.1f)  layers:%d ",
             (double)s->cam.x, (double)s->cam.y, N_LAYERS);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_draw — clear, draw the scene, then lay the HUD over it (status, title,
 * params, hint).
 *
 * The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, camera, run-state. A
 * const read can't re-couple the layers; scene_draw and the renderers stay narrow.
 */
static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(&s->cam, s->current_pattern, s->time_secs, sc->cols, sc->rows);

    draw_status_line(sc, s, fps, sim_fps);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " STAR FIELD / PARALLAX ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    draw_param_line(s);
    draw_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main
 * loop. main() is the ONE place that combines the layers per tick, in fixed
 * order:  scene_tick (SIM) -> screen_draw (RENDER) -> screen_present -> input.
 * app_handle_key() / app_do_resize() mutate state on USER EVENTS (a keypress or
 * SIGWINCH; 'r' also reseeds the noise) and are deliberately OUTSIDE the tick. */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset(s);                                break;

    case '=': case '+':
        if (s->speed < SCROLL_SPEED_MAX) s->speed *= 2;
        if (s->speed > SCROLL_SPEED_MAX) s->speed  = SCROLL_SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SCROLL_SPEED_MIN) s->speed  = SCROLL_SPEED_MIN;
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
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
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
    scene_init(&app->scene);

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

        /* Fixed-step accumulator — same idiom as every other demo. */
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
