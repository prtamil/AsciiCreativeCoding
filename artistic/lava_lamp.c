/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lava_lamp.c — magma chamber with metaball blobs
 *
 * DEMO: A few large viscous "lava" blobs drift slowly inside a confined
 *       chamber. Each blob is a smooth metaball — a Gaussian-like
 *       scalar field centred on its position. The total field is the
 *       sum of every blob's contribution; cells whose field exceeds a
 *       threshold are rendered as molten material, with the field
 *       gradient driving glyph and colour intensity. The blobs heat
 *       at the floor (buoyant lift), cool near the ceiling (gentle
 *       sinking), and merge or split as they pass through one another
 *       — the field sums freely, so two close blobs become one larger
 *       lobe and slowly separate again. Slow time scale: a single
 *       blob rises in ~6 seconds, the whole scene is meditative
 *       rather than frenetic.
 *
 * Study alongside: raymarcher/metaballs.c (3-D version),
 *                  fluid/marching_squares.c (iso-contour extraction —
 *                  this file uses the same scalar-field idea but
 *                  threshold-fills the interior rather than tracing
 *                  the contour).
 *
 * Section map:
 *   §1 config    — N_BLOBS, field constants, temperature dynamics
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 5-stop heat ramp per theme
 *   §4 random    — frand
 *   §5 blob      — Blob struct + spawn / tick + buoyancy
 *   §6 field     — scalar field evaluation: f(x, y) = Σ Aᵢ / (rᵢ² + ε²)
 *   §7 lamp      — Lamp state + lifecycle + chamber bounds
 *   §8 scene     — field raster + chamber walls + HUD
 *   §9 screen    — ncurses init / cleanup
 *  §10 app       — signals, dt tracking, key handling, main loop
 *
 * Keys:  [/]   blob count (3..10)
 *        -/+   field threshold (smaller = larger blobs)
 *        ,/.   buoyancy strength
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/lava_lamp.c \
 *       -o lava_lamp -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : N circular blobs, each with position (x, y), radius
 *                 r, and temperature T ∈ [0, 1]. A scalar field is
 *                 defined as
 *                     f(x, y) = Σ_i  rᵢ² / ((x - xᵢ)² + (y - yᵢ)²·k + ε²)
 *                 where `k` is the cell-aspect correction (terminal
 *                 cells are ~2× tall as wide, so the field is squashed
 *                 vertically by k = 4 = 2²). The renderer iterates
 *                 every screen cell, evaluates f, and emits a glyph
 *                 from the heat ramp when f > THRESHOLD.
 *
 *                 Buoyancy: a blob's vertical velocity is driven by
 *                 (T - T_ambient(y)), where T_ambient is cool at the
 *                 ceiling and hot at the floor. So hot blobs rise, cool
 *                 blobs sink — a slow oscillation. Damping prevents
 *                 unbounded acceleration. Blobs additionally take a
 *                 random horizontal walk to break perfect verticality.
 *
 * Data-structure: Lamp holds Blob[N_BLOBS_MAX] inline. Each Blob has
 *                 (x, y, vx, vy, r, temp). Field evaluation is on the
 *                 fly; no grid storage.
 *
 * Rendering     : Per frame: erase, raster every cell (sr, sc), compute
 *                 f, choose glyph + colour from f's magnitude. The
 *                 raster is O(rows · cols · N_BLOBS) — at 24×80×8 =
 *                 ~15k field evaluations per frame; each is 8 multiplies
 *                 + a divide. Easily 60 fps.
 *
 * References    :
 *   Blinn, "A Generalization of Algebraic Surface Drawing" *ACM Trans.
 *     Graph.* 1 (1982) — original metaball / "blobby" surface paper.
 *   Wyvill, McPheeters & Wyvill, "Data Structure for Soft Objects"
 *     *The Visual Computer* 2 (1986) — refined kernel choice.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The visible material is defined by a SCALAR FIELD, not by drawing
 * each blob individually. Each blob is just a centre + radius that
 * contributes an inverse-square term to a global field. The renderer
 * walks every screen cell, evaluates the field there, and emits a
 * glyph if `f(x,y) > threshold`. Two close blobs naturally form a
 * smooth dumbbell because their contributions ADD — there's no
 * "merge" operation; merging is a free consequence of summation.
 *
 * Blobs heat near the floor, cool near the ceiling. Buoyancy is
 * driven by the temperature difference from local ambient: hot
 * blob in cool zone rises; cool blob in hot zone sinks. The slow
 * oscillation period is what gives the lava-lamp feel.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a height map of an invisible terrain. Each blob is a
 * mountain peak at its centre. The terrain is the sum of all mountain
 * profiles. Now imagine flooding the terrain to a fixed water level
 * (the threshold). Wherever the terrain is above water, we paint a
 * glyph; wherever it's below, we paint nothing. As blobs move, the
 * peaks shift, the flooded silhouette shifts smoothly. Two peaks
 * close together create a connected island; far apart, two separate
 * islands. Buoyancy makes the peaks drift up when hot and down when
 * cool. The whole field re-evaluates fresh each frame; nothing is
 * stored between frames except the blob centres + temperatures.
 *
 * DRAWING METHOD  (per frame)
 * ──────────────
 *  1. erase()
 *  2. draw_walls — chamber border in dim grey.
 *  3. draw_field — for every screen cell (sr, sc):
 *       a. Evaluate `f = Σᵢ rᵢ² / ((Δx)² + (Δy)²·k² + ε²)` and the
 *          weighted-average blob temperature `t_avg`.
 *       b. If `f < threshold`: skip — empty space.
 *       c. Else: blend interior intensity (proximity to a blob centre)
 *          with `t_avg` to pick a heat-ramp bucket, emit glyph.
 *  4. HUD + key hints.
 *
 *  blob_tick (slow physics):
 *    1. Compute T_ambient(y) = 1 − y/(rows-2)  (hot floor, cool ceiling)
 *    2. vy += -BUOYANCY · (temp − T_ambient(y)) · dt
 *    3. vx += signed_random · HORIZ_NOISE · dt
 *    4. velocities damp by `(1 − DAMPING · dt)`
 *    5. position += velocity · dt
 *    6. heat or cool depending on which half we're in
 *    7. wall bounce — clamp to bounds, halve the impacting velocity
 *
 * KEY FORMULAS
 * ────────────
 *  Field at (x, y):
 *    f(x, y) = Σᵢ  rᵢ² / ((x − xᵢ)² + (y − yᵢ)²·k² + ε²)
 *    where k² = 4 (cell-aspect squared, ASPECT_X = 2)
 *
 *  Weighted-average temperature:
 *    t_avg = (Σᵢ wᵢ · tempᵢ) / (Σᵢ wᵢ)    where wᵢ is blob i's
 *                                        contribution at this cell.
 *
 *  Visible material:
 *    f > threshold  →  draw with bucket = blend(intensity, t_avg)
 *    intensity      = (f − threshold) / (threshold · 2)  (clamped 0..1)
 *    v              = 0.4 · intensity + 0.6 · t_avg
 *    bucket         = floor(v · 4.99)
 *
 *  Buoyancy force (drives the slow oscillation):
 *    F = -BUOYANCY · (temp − T_ambient(y))
 *    (negative = upward in screen coords; hot blob in cool zone rises)
 *
 *  Cell aspect-correction:
 *    Without `k² = 4` the field is symmetric in pixels. Each cell is
 *    twice as tall as wide, so we squash the y component by 2 in the
 *    distance calc — blobs render as round.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • A_DIM on dark cool blobs = invisible. Earlier code had
 *    `attr |= (bucket >= 3) ? A_BOLD : A_DIM` and dim cool blobs at
 *    bucket 0 with already-dark colours disappeared. Removed A_DIM;
 *    brightness gradient comes from colour values alone.
 *
 *  • Threshold tuning — `THRESHOLD_DEFAULT = 0.8` is set so blobs
 *    are clearly visible without merging into one big mass. Lower
 *    threshold (`+` key) → larger blobs that merge sooner. Higher
 *    threshold (`-` key) → smaller blobs, more separation.
 *
 *  • Performance — O(rows · cols · N_BLOBS) per frame for field
 *    evaluation. With 24×80×6 = 11k cells × 6 blobs = 66k ops; trivial
 *    at 30 fps. Going past N_BLOBS_MAX = 10 would still be fine
 *    arithmetically but visually crowds the chamber.
 *
 *  • Blob temperature drift — at the wall bounce, we keep `temp`
 *    unchanged. A blob that bounces off the floor several times
 *    accumulates `temp ≈ 1.0` and starts rising fast. That's the
 *    desired behaviour.
 *
 *  • Resize — chamber bounds update via wall bounce; blobs already
 *    at extreme y/x positions get clamped on next tick. Acceptable.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Default config: 6 slow blobs visibly oscillate up and down with
 *    a ~6-second period. Two close blobs form a dumbbell shape that
 *    smoothly separates as they drift apart.
 *
 *  • Press `+` to lower threshold: blobs grow visibly; once below
 *    `0.4` everything merges into one big chamber-filling blob.
 *
 *  • Press `-` to raise threshold: blobs shrink; at `2.0` only the
 *    very centres of blobs render — like a sparse star field.
 *
 *  • Press `,` to lower buoyancy: blobs barely move; nearly static.
 *  • Press `.` to raise buoyancy: blobs zip up and down rapidly.
 *
 *  • Press `t` to cycle theme: red → blue → green → purple. Whole
 *    chamber recolours.
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

#define TARGET_FPS         30        /* slow scene; 30 fps is plenty       */

#define N_BLOBS_MAX        10
#define N_BLOBS_DEFAULT     6
#define N_BLOBS_MIN         3

#define BLOB_RADIUS_MIN     2.5f
#define BLOB_RADIUS_MAX     5.0f

/* Cell-aspect correction. y in cell space is squashed by ~2× vs x. */
#define ASPECT_K2           4.0f     /* k² where k = ASPECT_X = 2.0        */

/* Field threshold — bigger threshold means smaller-looking blobs. */
#define THRESHOLD_DEFAULT   0.8f
#define THRESHOLD_MIN       0.3f
#define THRESHOLD_MAX       2.0f
#define THRESHOLD_STEP      0.1f

/* Buoyancy: vertical force = BUOYANCY · (T - T_ambient(y)). */
#define BUOYANCY_DEFAULT    8.0f
#define BUOYANCY_MIN        2.0f
#define BUOYANCY_MAX        20.0f
#define BUOYANCY_STEP       2.0f
#define DAMPING             1.0f     /* velocity damping per second        */
#define HORIZ_NOISE         3.0f     /* horizontal walk amplitude          */

/* Temperature dynamics — heated at floor, cooled at ceiling. */
#define HEAT_GAIN           0.30f    /* temp/sec when at floor             */
#define HEAT_LOSS           0.20f    /* temp/sec when at ceiling           */
#define BOUND_MARGIN        2        /* keep blobs this many cells inside  */

#define DT_CAP_S            0.10f
#define N_THEMES            4

/* Colour pair IDs */
#define PAIR_HEAT_0  1   /* coolest                                       */
#define PAIR_HEAT_1  2
#define PAIR_HEAT_2  3
#define PAIR_HEAT_3  4
#define PAIR_HEAT_4  5   /* hottest                                       */
#define PAIR_WALL    6
#define PAIR_HUD     7
#define PAIR_HINT    8

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

/* Four distinct dominant hues. Cold end (bucket 0) is mid-bright in
 * every theme so that cool/sinking blobs stay visible without A_DIM
 * — see draw_field, which no longer dims the lower buckets. */
static const short HEAT_256[N_THEMES][5] = {
    /* 0 lava   — red → orange → yellow → white   */
    { 160, 196, 208, 220, 231 },
    /* 1 ocean  — sky blue → cyan → white         */
    {  39,  45,  51,  87, 231 },
    /* 2 toxic  — medium green → lime → white     */
    {  34,  82, 118, 154, 231 },
    /* 3 royal  — purple → pink → white           */
    {  91, 127, 165, 213, 231 },
};
static const short HEAT_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    init_pair(PAIR_WALL, x256 ? 240 : COLOR_WHITE, -1);
    init_pair(PAIR_HUD,  x256 ?   0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT, x256 ?  75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void)        { return (float)rand() / (float)RAND_MAX; }
static float frand_signed(void) { return frand() * 2.0f - 1.0f; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  blob                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float x, y;
    float vx, vy;
    float r;
    float temp;
} Blob;

static void blob_spawn(Blob *b, int rows, int cols)
{
    b->x   = (float)BOUND_MARGIN + frand() * (float)(cols - 2 * BOUND_MARGIN);
    b->y   = (float)BOUND_MARGIN + frand() * (float)(rows - 2 - 2 * BOUND_MARGIN);
    b->vx  = 0;
    b->vy  = 0;
    b->r   = BLOB_RADIUS_MIN + frand() * (BLOB_RADIUS_MAX - BLOB_RADIUS_MIN);
    b->temp = frand();
}

/*
 * blob_tick — buoyancy-driven motion.
 *   T_ambient(y) = 1.0 at floor, 0.0 at ceiling (linear).
 *   buoyancy force = -BUOYANCY · (temp - T_ambient(y))   (negative = up)
 *
 * Blobs heat when near the floor and cool near the ceiling — that drives
 * the slow oscillation. A small horizontal random walk breaks symmetry.
 * Bounce off the chamber walls.
 */
static void blob_tick(Blob *b, float dt, float buoyancy, int rows, int cols)
{
    float t_ambient = 1.0f - (b->y / (float)(rows - 2));
    if (t_ambient < 0) t_ambient = 0;
    if (t_ambient > 1) t_ambient = 1;

    /* Negative dy is "up" in screen coords. */
    b->vy += -buoyancy * (b->temp - t_ambient) * dt;
    b->vx += frand_signed() * HORIZ_NOISE * dt;

    /* Damping. */
    float k = 1.0f - DAMPING * dt;
    if (k < 0) k = 0;
    b->vx *= k;
    b->vy *= k;

    b->x += b->vx * dt;
    b->y += b->vy * dt;

    /* Heat / cool based on which half we're in. */
    if (b->y > (float)rows * 0.5f) b->temp += HEAT_GAIN * dt;
    else                           b->temp -= HEAT_LOSS * dt;
    if (b->temp < 0) b->temp = 0;
    if (b->temp > 1) b->temp = 1;

    /* Wall bounce. */
    float xmin = (float)BOUND_MARGIN;
    float xmax = (float)(cols - BOUND_MARGIN);
    float ymin = (float)BOUND_MARGIN;
    float ymax = (float)(rows - 2 - BOUND_MARGIN);
    if (b->x < xmin) { b->x = xmin; b->vx = fabsf(b->vx) * 0.5f; }
    if (b->x > xmax) { b->x = xmax; b->vx = -fabsf(b->vx) * 0.5f; }
    if (b->y < ymin) { b->y = ymin; b->vy = fabsf(b->vy) * 0.5f; }
    if (b->y > ymax) { b->y = ymax; b->vy = -fabsf(b->vy) * 0.5f; }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  field                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * field_eval — sum of inverse-square contributions from each blob.
 *   contribution_i = rᵢ² / ((x - xᵢ)² + (y - yᵢ)²·k² + ε²)
 * where k² is the cell-aspect squared (so the field is squashed
 * vertically and the rendered blobs read as round on screen).
 *
 * Returns total field magnitude at (x, y) in cell coordinates.
 *
 * Also fills *out_t_avg with the radius-weighted average temperature
 * of contributing blobs (for the heat-ramp colour). The bigger the
 * blob's contribution at this cell, the more its temperature dominates.
 */
static float field_eval(const Blob *blobs, int n,
                        float x, float y, float *out_t_avg)
{
    float total = 0.0f;
    float wsum  = 0.0f;
    float tw    = 0.0f;
    for (int i = 0; i < n; i++) {
        float dx = x - blobs[i].x;
        float dy = (y - blobs[i].y);
        float d2 = dx * dx + dy * dy * ASPECT_K2 + 0.5f;
        float w  = blobs[i].r * blobs[i].r / d2;
        total += w;
        wsum  += w;
        tw    += w * blobs[i].temp;
    }
    if (out_t_avg) *out_t_avg = (wsum > 1e-6f) ? (tw / wsum) : 0.0f;
    return total;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  lamp                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    Blob blobs[N_BLOBS_MAX];
    int  n;
    float threshold;
    float buoyancy;
    int   theme;
    int   paused;
} Lamp;

static void lamp_reseed(Lamp *l, int rows, int cols)
{
    for (int i = 0; i < l->n; i++) blob_spawn(&l->blobs[i], rows, cols);
}

static void lamp_tick(Lamp *l, float dt, int rows, int cols)
{
    for (int i = 0; i < l->n; i++)
        blob_tick(&l->blobs[i], dt, l->buoyancy, rows, cols);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Heat ramp glyphs by intensity bucket (interior gradient). */
static const char HEAT_GLYPH[5] = { '`', '.', '*', 'o', '#' };

static int heat_bucket(float t)
{
    int b = (int)(t * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

static void draw_walls(int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_WALL) | A_DIM);
    /* Top + bottom */
    for (int c = 0; c < cols; c++) {
        mvaddch(0,        c, (chtype)'_');
        if (rows - 2 >= 0) mvaddch(rows - 2, c, (chtype)'_');
    }
    /* Left + right */
    for (int r = 1; r < rows - 1; r++) {
        if (0 < cols)        mvaddch(r, 0,        (chtype)'|');
        if (cols - 1 < cols) mvaddch(r, cols - 1, (chtype)'|');
    }
    attroff(COLOR_PAIR(PAIR_WALL) | A_DIM);
}

static void draw_field(const Lamp *l, int rows, int cols)
{
    /* Inset 1 cell inside the chamber walls. */
    for (int sr = 1; sr < rows - 2; sr++) {
        for (int sc = 1; sc < cols - 1; sc++) {
            float t_avg;
            float f = field_eval(l->blobs, l->n,
                                 (float)sc, (float)sr, &t_avg);
            if (f < l->threshold) continue;

            /* Glyph by interior gradient: closer to a blob centre →
             * higher f → brighter glyph. Map f over [thresh, 3·thresh]. */
            float intensity = (f - l->threshold) / (l->threshold * 2.0f);
            if (intensity < 0)  intensity = 0;
            if (intensity > 1)  intensity = 1;

            /* Blend interior intensity with average blob temperature
             * so cool blobs are dim and hot blobs are bright. */
            float v = 0.4f * intensity + 0.6f * t_avg;
            int bucket = heat_bucket(v);

            /* Brightness gradient comes from the colour values
             * themselves (cool end darker, hot end whiter). A_BOLD on
             * the top buckets makes hot blobs glow; A_DIM on the cold
             * buckets used to make them invisible on dark terminals,
             * so we drop it. */
            chtype attr = COLOR_PAIR(PAIR_HEAT_0 + bucket);
            if (bucket >= 3) attr |= A_BOLD;
            attron(attr);
            mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
            attroff(attr);
        }
    }
}

static void scene_draw(int rows, int cols, const Lamp *l, double fps)
{
    erase();
    draw_walls(rows, cols);
    draw_field(l, rows, cols);

    /* HUD */
    char buf[160];
    snprintf(buf, sizeof buf,
             " blobs:%d  thresh:%.2f  buoy:%.1f  theme:%d  "
             "%5.1f fps  %s ",
             l->n, l->threshold, l->buoyancy, l->theme, fps,
             l->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:blobs  -/+:thresh  ,/.:buoyancy  t:theme  "
             "r:reset  p:pause  q:quit  [lava_lamp] ");
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

static Lamp g_lamp;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_lamp.n         = N_BLOBS_DEFAULT;
    g_lamp.threshold = THRESHOLD_DEFAULT;
    g_lamp.buoyancy  = BUOYANCY_DEFAULT;
    g_lamp.theme     = 0;

    screen_init(g_lamp.theme);
    int rows = LINES, cols = COLS;
    lamp_reseed(&g_lamp, rows, cols);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            lamp_reseed(&g_lamp, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_lamp.paused ^= 1; break;
                case 'r':          lamp_reseed(&g_lamp, rows, cols); break;
                case 't':          g_lamp.theme = (g_lamp.theme + 1) % N_THEMES;
                                   color_init(g_lamp.theme); break;
                case '[':
                    if (g_lamp.n > N_BLOBS_MIN) g_lamp.n--;
                    break;
                case ']':
                    if (g_lamp.n < N_BLOBS_MAX) {
                        blob_spawn(&g_lamp.blobs[g_lamp.n], rows, cols);
                        g_lamp.n++;
                    }
                    break;
                case '-':
                    if (g_lamp.threshold + THRESHOLD_STEP <= THRESHOLD_MAX)
                        g_lamp.threshold += THRESHOLD_STEP;
                    break;
                case '+': case '=':
                    if (g_lamp.threshold - THRESHOLD_STEP >= THRESHOLD_MIN)
                        g_lamp.threshold -= THRESHOLD_STEP;
                    break;
                case ',':
                    if (g_lamp.buoyancy - BUOYANCY_STEP >= BUOYANCY_MIN)
                        g_lamp.buoyancy -= BUOYANCY_STEP;
                    break;
                case '.':
                    if (g_lamp.buoyancy + BUOYANCY_STEP <= BUOYANCY_MAX)
                        g_lamp.buoyancy += BUOYANCY_STEP;
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_lamp.paused) lamp_tick(&g_lamp, dt, rows, cols);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_lamp, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
