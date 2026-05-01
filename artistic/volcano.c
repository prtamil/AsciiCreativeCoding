/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * volcano.c — volcanic eruption with lava bombs and ash plume
 *
 * DEMO: A truncated cone fills the lower half of the screen with a
 *       glowing orange crater at its apex. Lava bombs erupt from the
 *       crater on parabolic trajectories — they arc up, fly outward,
 *       fall under gravity, and die on impact with the cone slopes,
 *       cooling from white-hot to dim red as they age. Above the
 *       crater, a slow ash plume rises in a noisy column with
 *       horizontal random-walk diffusion. Periodic eruption bursts
 *       (every ~3 seconds) blast a dozen extra bombs out at once.
 *
 *       Two distinct particle systems share a single heat ramp: the
 *       hot bombs sit on the bright end, the cool ash on the dim end.
 *
 * Study alongside: artistic/fire_tornado.c (sister vertical-fire piece;
 *                  this one is ballistic + gravity, no rotation),
 *                  particle_systems/fireworks.c (similar bomb pool),
 *                  particle_systems/smoke.c (similar plume pattern).
 *
 * Section map:
 *   §1 config    — pool sizes, geometry, eruption rates, palette
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 5-stop heat ramp (lava) + 3-stop grey ramp (ash)
 *   §4 random    — frand
 *   §5 bomb      — Bomb struct + spawn / tick / draw
 *   §6 ash       — Ash struct + spawn / tick / draw
 *   §7 volcano   — Volcano state, mountain geometry, eruption lifecycle
 *   §8 scene     — draw_mountain + draw_crater + draw_bombs + draw_ash + HUD
 *   §9 screen    — ncurses init / cleanup
 *  §10 app       — signals, dt tracking, key handling, main loop
 *
 * Keys:  [/]   bomb count          (20..200)
 *        -/+   eruption rate       (slow .. violent)
 *        ,/.   crater width        (narrow .. wide)
 *        b     trigger a burst on demand
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/volcano.c \
 *       -o volcano -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two ballistic particle pools plus a static silhouette.
 *                 Bombs spawn at the crater with random launch angle in
 *                 a narrow upward cone (±0.45 rad). Each tick: gravity
 *                 adds to vy, position advances, temperature decays, and
 *                 the bomb dies if it leaves the screen, hits the cone
 *                 silhouette (`is_in_mountain`), or cools out. Ash
 *                 particles spawn at the crater with slight upward
 *                 velocity, drift via horizontal random walk, and fade
 *                 by life rather than by temperature. A periodic
 *                 "eruption burst" spawns BURST_COUNT bombs at once
 *                 every BURST_INTERVAL seconds — that's what makes the
 *                 volcano visibly *erupt* rather than just sputter.
 *
 * Data-structure: Volcano holds Bomb[N_BOMBS_MAX] and Ash[N_ASH_MAX]
 *                 inline. Mountain geometry is parameterised by axis_x,
 *                 top_row, base_row, crater_half, base_half — all
 *                 recomputed on resize so the cone re-flows.
 *
 * Rendering     : Per frame, in this draw order:
 *                   ash → mountain silhouette → crater glow → bombs.
 *                 Ash draws first so the cone overpaints any plume that
 *                 strays inside the silhouette. Bombs draw last to
 *                 appear in front of the crater.
 *
 * Performance   : O(N_BOMBS + N_ASH) per frame. At N=200 + N=120 it's
 *                 a few hundred mvaddch per frame — trivial.
 *
 * References    :
 *   Reeves, "Particle Systems — A Technique for Modelling a Class of
 *     Fuzzy Objects" SIGGRAPH (1983).
 *   Self & Whitehead, "Strombolian eruption mechanics" *Bulletin of
 *     Volcanology* 49 (1986) — the ballistic-bomb regime modelled here.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A static cone silhouette plus two ballistic particle systems
 * (lava bombs + ash) plus a periodic burst event. Bombs follow plain
 * Newtonian projectile motion `vy += g·dt`, no curve fitting. Ash is
 * a slow rising haze with horizontal random walk. The cone is just a
 * mathematical predicate `is_in_mountain(row, col)` — bombs that land
 * inside the cone die on impact. There is no "lava flow," no terrain
 * deformation, no fluid simulation — just three independent layers
 * sharing a coordinate system.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a child's drawing of a volcano. The triangle outline is
 * literally drawn with `/` and `\`. Above the apex, you scatter dots
 * at random and give them initial velocities pointing up and out.
 * Gravity pulls them down. Some come back over the slope; they hit
 * the rock and stop. Others escape sideways and fall off-screen.
 * Rinse, repeat. The volcano never changes shape; only the air above
 * it changes, frame by frame, as bombs and ash inhabit different
 * positions over time.
 *
 * DRAWING METHOD  (per frame, layered)
 * ──────────────
 *  1. erase()
 *  2. ash_draw — rising haze, drawn first so the cone overpaints any
 *     plume that strays into the silhouette
 *  3. draw_mountain — `/`/`\` slopes + sparse interior `.` rocks +
 *     ground-line `_____` underneath
 *  4. draw_crater — the hottest row at the apex: `\`/`_`/`_`/`/` in
 *     bright white, `A_BOLD`
 *  5. bomb_draw — every alive bomb in heat-ramp by temperature
 *  6. HUD + key hints
 *
 * KEY FORMULAS
 * ────────────
 *  Cone membership (linear taper from crater to base):
 *    frac    = (row − top_row) / (base_row − top_row)
 *    half_w  = crater_half + frac · (base_half − crater_half)
 *    in_cone = (row > top_row) AND (|col − axis_x| ≤ half_w)
 *
 *  Bomb launch (narrow upward cone):
 *    ang   ∈ [-CONE, +CONE] uniform (default cone = 0.7 rad ≈ 40°)
 *    speed ∈ [SPEED_MIN, SPEED_MAX]
 *    vx = sin(ang) · speed · ASPECT_X
 *    vy = -cos(ang) · speed                  (negative = up)
 *
 *  Bomb ballistics (per tick):
 *    vy   += GRAVITY · dt
 *    pos  += v · dt
 *    temp -= BOMB_COOL · dt
 *    die if off-screen, in_cone, or temp ≤ 0
 *
 *  Ash random walk:
 *    vx   += rand_signed() · ASH_DRIFT · dt
 *    vx   *= (1 − 0.6 · dt)                  (light damping)
 *    pos  += v · dt
 *    life -= ASH_FADE · dt
 *
 *  Apex height above launch  (ang=0, max info-density):
 *    apex = vy² / (2·GRAVITY) ≈ MAX_VEL² / (2·g)
 *    With MAX_VEL = 12, g = 14: apex ≈ 5 cells. Bombs stay on-screen.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Bombs flying off-screen — early tuning had MAX_VEL = 22, g = 18,
 *    which gave apex ≈ 13 cells. On a 24-row terminal with top_row=8,
 *    bombs flew off the top before falling back. Current values keep
 *    every arc visible.
 *
 *  • Pool full — `try_spawn_bomb` silently drops new spawns when no
 *    dead slot exists. The `b` key handler avoids this by replacing
 *    every active slot with a fresh `bomb_spawn_burst`.
 *
 *  • Cone overlap with bombs — bombs DO render briefly inside the
 *    cone region during their last tick before death. The mountain
 *    is drawn before bombs, so a hot glyph appears momentarily on
 *    rock; visually fine, even nice.
 *
 *  • Crater vs ash spawn point — both spawn at row=top_row, but the
 *    crater glyph is drawn AFTER ash, so the crater's `_____` in
 *    PAIR_HEAT_4 overpaints any ash on that row. Acceptable: ash is
 *    above the crater, not below.
 *
 *  • Resize — cone geometry recomputes (`volcano_position`), pools
 *    keep flying. Bombs in mid-air at old screen positions naturally
 *    crash off the new bounds.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Default config: bombs visibly arc above the cone, most landing
 *    on the slopes, a few escaping sideways.
 *
 *  • Press `b`: every active bomb slot fires at once with a wider
 *    cone and 1.5× speed. Dramatic eruption.
 *
 *  • Press `,` / `.`: crater shrinks/widens. Ash and bomb spawn area
 *    follow because both use `crater_half` for spawn jitter.
 *
 *  • Periodic burst: every 3 seconds 12 bombs spawn in a tight cluster
 *    (visible as a sudden surge of activity).
 *
 *  • Themes: red/blue/green/purple — colour change is unmistakable
 *    because every layer (mountain, crater, bombs, ash) updates.
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS         30          /* slower frame rate keeps key polling
                                        * snappy on terminals that lag at 60 */

/* Particle pools. */
#define N_BOMBS_MAX        200
#define N_BOMBS_DEFAULT     60
#define N_BOMBS_MIN         20
#define N_BOMBS_STEP        20

#define N_ASH_MAX          150

/* Bomb dynamics. Tuned so bombs stay on-screen for their full arc on
 * a typical 24-row terminal: launch speed × cos / (2·gravity) gives the
 * apex height in cells. With these values, apex ≈ 7 cells above the
 * crater — visible the whole flight. */
#define BOMB_SPEED_MIN      6.0f       /* cells/sec at launch              */
#define BOMB_SPEED_MAX     12.0f
#define BOMB_LAUNCH_CONE    0.7f       /* radians ± from vertical (wider!) */
#define GRAVITY            14.0f       /* cells/sec² downward              */
#define BOMB_COOL           0.20f      /* temp/sec — slow so bombs stay     *
                                        * bright across most of the arc     */

/* Ash dynamics. */
#define ASH_RISE            3.5f       /* cells/sec average upward          */
#define ASH_DRIFT           4.0f       /* horizontal random-walk amplitude */
#define ASH_FADE            0.18f      /* life/sec — slow fade              */

/* Eruption rhythm. */
#define ERUPT_RATE_DEFAULT  20.0f      /* bombs/sec                         */
#define ERUPT_RATE_MIN       5.0f
#define ERUPT_RATE_MAX      60.0f
#define ERUPT_RATE_STEP     10.0f      /* big enough to be visibly different*/
#define ASH_RATE            40.0f      /* ash spawns/sec                    */
#define BURST_INTERVAL       3.0f      /* seconds between bursts            */
#define BURST_COUNT         12         /* bombs per burst                   */

/* Geometry. Cone cells, crater half-width. */
#define CRATER_HALF_DEFAULT  4
#define CRATER_HALF_MIN      2
#define CRATER_HALF_MAX     12
#define BASE_HALF_FRAC       0.40f     /* base half-width as frac of cols  */
#define TOP_ROW_FRAC         0.35f     /* crater row as frac of rows       */

#define ASPECT_X             2.0f
#define DT_CAP_S             0.10f
#define N_THEMES             4

/* Colour pairs */
#define PAIR_HEAT_0   1
#define PAIR_HEAT_1   2
#define PAIR_HEAT_2   3
#define PAIR_HEAT_3   4
#define PAIR_HEAT_4   5
#define PAIR_ASH_0    6      /* darkest ash (greenish on hot terms) */
#define PAIR_ASH_1    7
#define PAIR_ASH_2    8
#define PAIR_ROCK     9
#define PAIR_HUD      10
#define PAIR_HINT     11

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Heat ramp 0..4 plus ash ramp 0..2 per theme. Each theme picks a
 * distinct dominant hue across the WHOLE ramp so that cycling 't' is
 * visibly different even though most bombs sit in the bright end. */
static const short HEAT_256[N_THEMES][5] = {
    /* 0 fire     — red → orange → yellow → white   */
    { 124, 196, 208, 226, 231 },
    /* 1 blue     — navy → blue → cyan → white      */
    {  25,  33,  39,  51, 231 },
    /* 2 toxic    — dark green → green → lime → white */
    {  22,  28,  82, 154, 231 },
    /* 3 purple   — magenta → pink → light pink → white */
    {  53,  91, 165, 213, 231 },
};
/* Ash palette — light grays so the plume actually shows up against a
 * dark terminal. 247/250/254 read as faint smoke without disappearing. */
static const short ASH_256[N_THEMES][3] = {
    { 244, 250, 255 },
    { 245, 251, 255 },
    { 244, 250, 255 },
    { 246, 252, 255 },
};
static const short HEAT_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};
static const short ASH_8[3] = { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE };

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    for (int i = 0; i < 3; i++) {
        short fg = x256 ? ASH_256[theme][i] : ASH_8[i];
        init_pair((short)(PAIR_ASH_0 + i), fg, -1);
    }
    /* Rock — medium-dim gray. Was 240 (near-black); 244 reads as a
     * proper rock colour without competing with the lava. */
    init_pair(PAIR_ROCK, x256 ? 244 : COLOR_WHITE, -1);
    init_pair(PAIR_HUD,  x256 ?   0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT, x256 ?  75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

static int heat_bucket(float t)
{
    int b = (int)(t * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

static const char HEAT_GLYPH[5] = { '`', '.', '*', 'o', '#' };
static const char ASH_GLYPH[3]  = { '.', ':', '%' };

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  bomb                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float x, y;
    float vx, vy;
    float temp;
    int   alive;
} Bomb;

/*
 * bomb_spawn — launch from the crater with a small upward cone of angles.
 * Speed is randomised in [SPEED_MIN, SPEED_MAX]. Horizontal velocity
 * is multiplied by ASPECT_X so the parabola looks balanced on screen.
 */
static void bomb_spawn(Bomb *b, int axis_x, int top_row, int crater_half)
{
    b->x = (float)axis_x + (frand() - 0.5f) * 2.0f * (float)crater_half;
    b->y = (float)top_row;
    float ang   = (frand() - 0.5f) * 2.0f * BOMB_LAUNCH_CONE;
    float speed = BOMB_SPEED_MIN + frand() * (BOMB_SPEED_MAX - BOMB_SPEED_MIN);
    b->vx = sinf(ang) * speed * ASPECT_X;
    b->vy = -cosf(ang) * speed;
    b->temp = 0.95f + 0.05f * frand();
    b->alive = 1;
}

/*
 * bomb_spawn_burst — like bomb_spawn but with a wider cone and stronger
 * launch, used by the 'b' burst key so the eruption is unmistakable
 * even when the pool is already full.
 *
 *   cone   ≈ 1.6× normal
 *   speed  ≈ 1.5× normal max, plus jitter
 *   temp   = 1.0 (white-hot)
 */
static void bomb_spawn_burst(Bomb *b, int axis_x, int top_row, int crater_half)
{
    b->x = (float)axis_x + (frand() - 0.5f) * 2.0f * (float)crater_half;
    b->y = (float)top_row;
    float ang   = (frand() - 0.5f) * 2.0f * BOMB_LAUNCH_CONE * 1.6f;
    float speed = BOMB_SPEED_MAX + 4.0f + frand() * 6.0f;   /* big punch  */
    b->vx = sinf(ang) * speed * ASPECT_X;
    b->vy = -cosf(ang) * speed;
    b->temp = 1.0f;
    b->alive = 1;
}

/* Forward declaration — needed before bomb_tick uses it. */
typedef struct Volcano Volcano;
static int  is_in_mountain(const Volcano *v, int row, int col);

/*
 * bomb_tick — gravity + advance + cool. Death conditions: cooled out,
 * left the screen, or impacted the cone silhouette.
 */
static void bomb_tick(Bomb *b, const Volcano *v, float dt, int rows, int cols);

static void bomb_draw(const Bomb *b, int rows, int cols)
{
    if (!b->alive) return;
    int sr = (int)b->y, sc = (int)b->x;
    if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) return;
    int bucket = heat_bucket(b->temp);
    attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
    attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  ash                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float x, y;
    float vx, vy;
    float life;       /* 1.0 → 0.0 */
    int   alive;
} Ash;

static void ash_spawn(Ash *a, int axis_x, int top_row, int crater_half)
{
    a->x = (float)axis_x + (frand() - 0.5f) * 2.0f * (float)crater_half;
    a->y = (float)top_row;
    a->vx = (frand() - 0.5f) * 2.0f * ASPECT_X;
    a->vy = -ASH_RISE * (0.7f + 0.6f * frand());
    a->life = 1.0f;
    a->alive = 1;
}

static void ash_tick(Ash *a, float dt, int rows)
{
    if (!a->alive) return;
    /* Random-walk horizontal drift; light vertical drag. */
    a->vx += (frand() - 0.5f) * ASH_DRIFT * dt;
    a->vx *= (1.0f - 0.6f * dt);
    a->x  += a->vx * dt;
    a->y  += a->vy * dt;
    a->life -= ASH_FADE * dt;
    if (a->life <= 0.0f || a->y < 0 || a->y >= rows - 1) a->alive = 0;
}

static void ash_draw(const Ash *a, int rows, int cols)
{
    if (!a->alive) return;
    int sr = (int)a->y, sc = (int)a->x;
    if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) return;
    int bucket = (int)(a->life * 2.99f);
    if (bucket < 0) bucket = 0;
    if (bucket > 2) bucket = 2;
    attron(COLOR_PAIR(PAIR_ASH_0 + bucket) | A_DIM);
    mvaddch(sr, sc, (chtype)(unsigned char)ASH_GLYPH[bucket]);
    attroff(COLOR_PAIR(PAIR_ASH_0 + bucket) | A_DIM);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  volcano                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

struct Volcano {
    Bomb bombs[N_BOMBS_MAX];
    Ash  ash[N_ASH_MAX];
    int  n_bombs;             /* active bomb pool size                    */

    int  axis_x;
    int  top_row;
    int  base_row;
    int  crater_half;
    int  base_half;

    float erupt_rate;          /* bombs/sec target                          */
    float bomb_accum;
    float ash_accum;
    float burst_timer;

    int   theme;
    int   paused;
};

/* Cone-membership test. Linear taper from crater (top) to base (bottom). */
static int is_in_mountain(const Volcano *v, int row, int col)
{
    if (row <= v->top_row || row > v->base_row) return 0;
    float frac = (float)(row - v->top_row) / (float)(v->base_row - v->top_row);
    int half_w = v->crater_half + (int)(frac * (float)(v->base_half - v->crater_half));
    int dx = col - v->axis_x;
    return (dx >= -half_w && dx <= half_w);
}

/* Now define bomb_tick (uses is_in_mountain). */
static void bomb_tick(Bomb *b, const Volcano *v, float dt, int rows, int cols)
{
    if (!b->alive) return;
    b->vy   += GRAVITY * dt;
    b->x    += b->vx * dt;
    b->y    += b->vy * dt;
    b->temp -= BOMB_COOL * dt;
    if (b->temp <= 0
        || b->x < 0 || b->x >= cols
        || b->y >= rows - 1
        || is_in_mountain(v, (int)b->y, (int)b->x))
        b->alive = 0;
}

static void volcano_position(Volcano *v, int rows, int cols)
{
    v->axis_x   = cols / 2;
    v->top_row  = (int)(rows * TOP_ROW_FRAC);
    v->base_row = rows - 2;
    v->base_half = (int)(cols * BASE_HALF_FRAC);
    if (v->base_half < v->crater_half + 4) v->base_half = v->crater_half + 4;
}

static void volcano_reseed(Volcano *v)
{
    for (int i = 0; i < N_BOMBS_MAX; i++) v->bombs[i].alive = 0;
    for (int i = 0; i < N_ASH_MAX;   i++) v->ash[i].alive   = 0;
    v->bomb_accum  = 0.0f;
    v->ash_accum   = 0.0f;
    v->burst_timer = 0.0f;
}

static void try_spawn_bomb(Volcano *v)
{
    /* Find a dead slot; if pool is full, the loop completes naturally
     * and the spawn is silently dropped — matches the `n_bombs` cap. */
    for (int i = 0; i < v->n_bombs; i++) {
        if (!v->bombs[i].alive) {
            bomb_spawn(&v->bombs[i], v->axis_x, v->top_row, v->crater_half);
            return;
        }
    }
}

static void try_spawn_ash(Volcano *v)
{
    for (int i = 0; i < N_ASH_MAX; i++) {
        if (!v->ash[i].alive) {
            ash_spawn(&v->ash[i], v->axis_x, v->top_row, v->crater_half);
            return;
        }
    }
}

/*
 * volcano_tick — advance every particle, accumulate spawn rates with
 * fractional accumulators (frame-rate independent), and fire periodic
 * bursts.
 */
static void volcano_tick(Volcano *v, float dt, int rows, int cols)
{
    /* Tick bombs + ash. */
    for (int i = 0; i < v->n_bombs; i++) bomb_tick(&v->bombs[i], v, dt, rows, cols);
    for (int i = 0; i < N_ASH_MAX;   i++) ash_tick(&v->ash[i], dt, rows);

    /* Spawn bombs at erupt_rate. */
    v->bomb_accum += v->erupt_rate * dt;
    while (v->bomb_accum >= 1.0f) {
        v->bomb_accum -= 1.0f;
        try_spawn_bomb(v);
    }

    /* Spawn ash. */
    v->ash_accum += ASH_RATE * dt;
    while (v->ash_accum >= 1.0f) {
        v->ash_accum -= 1.0f;
        try_spawn_ash(v);
    }

    /* Periodic burst — multiple bombs at once. */
    v->burst_timer += dt;
    if (v->burst_timer >= BURST_INTERVAL) {
        v->burst_timer -= BURST_INTERVAL;
        for (int i = 0; i < BURST_COUNT; i++) try_spawn_bomb(v);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_mountain(const Volcano *v, int rows, int cols)
{
    /* Slopes: '/' on the left, '\' on the right; sparse rock dots inside. */
    attron(COLOR_PAIR(PAIR_ROCK));
    for (int r = v->top_row + 1; r <= v->base_row; r++) {
        if (r < 0 || r >= rows - 1) continue;
        float frac = (float)(r - v->top_row) / (float)(v->base_row - v->top_row);
        int half_w = v->crater_half + (int)(frac * (float)(v->base_half - v->crater_half));
        int lc = v->axis_x - half_w;
        int rc = v->axis_x + half_w;
        if (lc >= 0 && lc < cols) mvaddch(r, lc, (chtype)'/');
        if (rc >= 0 && rc < cols) mvaddch(r, rc, (chtype)'\\');
        for (int c = lc + 1; c < rc; c++) {
            if (c < 0 || c >= cols) continue;
            if (((r * 7 + c * 13) % 31) == 0)
                mvaddch(r, c, (chtype)'.');
        }
    }
    /* Ground line just below the cone. */
    if (v->base_row + 1 < rows - 1) {
        for (int c = 0; c < cols; c++)
            mvaddch(v->base_row + 1, c, (chtype)'_');
    }
    attroff(COLOR_PAIR(PAIR_ROCK));
}

/*
 * draw_crater — bright glow at the apex. Uses '_' fill across the
 * crater opening and '\\' '/' marking the rim; pulses subtly via temp
 * bucket=4 (white-hot).
 */
static void draw_crater(const Volcano *v, int rows, int cols)
{
    int r = v->top_row;
    if (r < 0 || r >= rows - 1) return;
    int lc = v->axis_x - v->crater_half;
    int rc = v->axis_x + v->crater_half;
    attron(COLOR_PAIR(PAIR_HEAT_4) | A_BOLD);
    if (lc >= 0 && lc < cols) mvaddch(r, lc, (chtype)'\\');
    if (rc >= 0 && rc < cols) mvaddch(r, rc, (chtype)'/');
    for (int c = lc + 1; c < rc; c++) {
        if (c < 0 || c >= cols) continue;
        mvaddch(r, c, (chtype)'_');
    }
    attroff(COLOR_PAIR(PAIR_HEAT_4) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Volcano *v, double fps)
{
    erase();

    /* Ash first, behind the cone. */
    for (int i = 0; i < N_ASH_MAX; i++) ash_draw(&v->ash[i], rows, cols);

    /* Cone overpaints any plume that strayed into the silhouette. */
    draw_mountain(v, rows, cols);

    /* Crater glow on top of the cone. */
    draw_crater(v, rows, cols);

    /* Bombs in front of everything. */
    for (int i = 0; i < v->n_bombs; i++) bomb_draw(&v->bombs[i], rows, cols);

    /* HUD */
    char buf[160];
    snprintf(buf, sizeof buf,
             " bombs:%d  rate:%.0f  crater:%d  theme:%d  %5.1f fps  %s ",
             v->n_bombs, v->erupt_rate, v->crater_half, v->theme, fps,
             v->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:bombs  -/+:rate  ,/.:crater  b:burst  t:theme  "
             "r:reset  p:pause  q:quit  [volcano] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Volcano g_volcano;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_volcano.n_bombs     = N_BOMBS_DEFAULT;
    g_volcano.erupt_rate  = ERUPT_RATE_DEFAULT;
    g_volcano.crater_half = CRATER_HALF_DEFAULT;
    g_volcano.theme       = 0;

    screen_init(g_volcano.theme);
    int rows = LINES, cols = COLS;
    volcano_position(&g_volcano, rows, cols);
    volcano_reseed(&g_volcano);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            volcano_position(&g_volcano, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_volcano.paused ^= 1; break;
                case 'r':          volcano_reseed(&g_volcano); break;
                case 't':          g_volcano.theme = (g_volcano.theme + 1) % N_THEMES;
                                   color_init(g_volcano.theme); break;
                case 'b':
                    /* Maximum-drama burst: replace EVERY active slot in
                     * the pool with a fresh, high-speed, wide-cone bomb.
                     * All n_bombs particles fire from the crater in the
                     * same frame — unmistakable eruption. The steady-
                     * state eruption resumes naturally once these die. */
                    for (int i = 0; i < g_volcano.n_bombs; i++) {
                        bomb_spawn_burst(&g_volcano.bombs[i],
                                         g_volcano.axis_x,
                                         g_volcano.top_row,
                                         g_volcano.crater_half);
                    }
                    /* Also crank the ash plume momentarily by saturating
                     * the ash pool. */
                    for (int i = 0; i < N_ASH_MAX; i++) {
                        if (!g_volcano.ash[i].alive)
                            ash_spawn(&g_volcano.ash[i],
                                      g_volcano.axis_x,
                                      g_volcano.top_row,
                                      g_volcano.crater_half);
                    }
                    break;
                case '[':
                    if (g_volcano.n_bombs - N_BOMBS_STEP >= N_BOMBS_MIN)
                        g_volcano.n_bombs -= N_BOMBS_STEP;
                    break;
                case ']':
                    if (g_volcano.n_bombs + N_BOMBS_STEP <= N_BOMBS_MAX)
                        g_volcano.n_bombs += N_BOMBS_STEP;
                    break;
                case '-':
                    if (g_volcano.erupt_rate - ERUPT_RATE_STEP >= ERUPT_RATE_MIN)
                        g_volcano.erupt_rate -= ERUPT_RATE_STEP;
                    break;
                case '+': case '=':
                    if (g_volcano.erupt_rate + ERUPT_RATE_STEP <= ERUPT_RATE_MAX)
                        g_volcano.erupt_rate += ERUPT_RATE_STEP;
                    break;
                case ',':
                    if (g_volcano.crater_half > CRATER_HALF_MIN) {
                        g_volcano.crater_half--;
                        volcano_position(&g_volcano, rows, cols);
                    }
                    break;
                case '.':
                    if (g_volcano.crater_half < CRATER_HALF_MAX) {
                        g_volcano.crater_half++;
                        volcano_position(&g_volcano, rows, cols);
                    }
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_volcano.paused) volcano_tick(&g_volcano, dt, rows, cols);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_volcano, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
