/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * burst.c — fireworks-style radial particle bursts on a persistent scorch grid.
 *
 * Each burst walks a 3-state FSM (IDLE → FLASH → LIVE → IDLE).  On
 * ignite it picks a random detonation centre, primes a one-frame
 * '*+'-cross flash, then emits BURST_PARTICLES particles in a fan of
 * BURST_WAVE_COUNT staggered waves around 360°.  Particles are
 * advanced with explicit Euler + exponential per-tick drag + linear
 * life decay; the visible glyph and brightness fade through a
 * 9-slot ramp indexed by remaining life.  Cells touched by any
 * particle are marked in a persistent SCORCH grid so the screen
 * "remembers" past bursts.  Ten themes cycle the colour pairs.
 *
 * Section map
 *   §1  config         constants — sim, burst, particle, scorch
 *   §2  clock          monotonic-ns timer + sleep
 *   §3  random + math  uniform samples + integer clamp
 *   §4  themes         10 hue palettes + Hue enum + theme_apply
 *   §5  debug overlay  d/D cycle through inspection modes
 *   §6  particle       Particle struct + spawn + tick + draw
 *   §7  burst FSM      Burst struct + state enum + burst_ignite
 *   §8  burst tick     FSM advance + complete + re-arm
 *   §9  burst render   flash cross + fuse overlay + burst_draw
 *   §10 field          burst pool + persistent scorch grid
 *   §11 screen + HUD   ncurses init + status strip
 *   §12 app            App struct + signal handlers + key dispatch
 *   §13 main           fixed-step main loop
 */

/* ── CONCEPTS & ALGORITHMS ───────────────────────────────────────────── *
 *
 *   Particle system        Pool of free particles, each with pos, vel,
 *                          life, glyph, hue.  Per-frame integrate + cull.
 *                          → Reeves (1983), ACM TOG 2(2): 91
 *
 *   Burst FSM              IDLE → FLASH (1 frame) → LIVE (until all
 *                          particles dead) → IDLE.  Re-armed with a
 *                          random delay.  Three states, three transitions.
 *                          → standard explosion/effects-engine pattern
 *
 *   Wave-staggered fan     N particles split into K waves; each wave
 *                          fires (max_delay · wave / (K-1)) frames after
 *                          ignite.  Reads as a shockwave, not a single
 *                          instant ring.
 *                          → Doom-style burst convention
 *
 *   Exponential drag       v *= exp(-DRAG_PER_TICK · dt)  (analytic
 *                          per-frame form; stable under any dt).
 *
 *   Life-keyed ramp        slot = clamp(floor((life/max_life)·9), 0..8)
 *                          → glyph from k_syms; colour pair from theme.
 *                          → ember/cooling-palette pattern (this repo)
 *
 *   Scorch overlay         Persistent uint8 grid; particles increment
 *                          their cell on every visible draw.  Renders
 *                          UNDER active particles so trails accumulate.
 *                          → frame-buffer "trail" technique
 *
 *   Object pool            Fixed-size Burst[BURST_COUNT]; no malloc in
 *                          the hot path.  Each Burst owns its own
 *                          particle pool inline.
 *
 *   Fixed-step accumulator Deterministic per-tick physics regardless
 *                          of render Hz.
 *                          → Glenn Fiedler, "Fix Your Timestep!" (2004)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── OVERALL PSEUDOCODE ──────────────────────────────────────────────── *
 *
 *   init:
 *     install signals; start ncurses (themes + HUD/HINT pairs)
 *     field_init: alloc burst pool + scorch grid; arm initial timers
 *
 *   loop while running:
 *     if need_resize:        endwin → refresh → field re-init
 *     dt = clock_now − last_frame_time      (capped at 100 ms)
 *     while sim_accum ≥ tick_ns:
 *       field_tick:          for each burst: burst_tick (FSM advance)
 *       sim_accum -= tick_ns
 *     screen_draw_field:     scorch layer + active-burst layer
 *     screen_draw_hud        fps + theme + bursts + sim_fps
 *     screen_present
 *     getch + app_handle_key q/space/t/d/+
 *     sleep to ~60 fps
 *
 *   cleanup:
 *     atexit endwin restores the terminal
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── DRIVER PSEUDOCODE ───────────────────────────────────────────────── *
 *
 * particle_tick(p, cols, rows):                   (per-particle physics)
 *   if !p->alive:           return
 *   if p->delay > 0:        p->delay--; return    wave-stagger countdown
 *   p->vx *= exp(-DRAG · dt);  p->vy *= exp(-DRAG · dt)   exponential drag
 *   p->px += p->vx;  p->py += p->vy                       explicit Euler
 *   p->life -= LIFE_DECAY                                 linear cooling
 *   if p->life ≤ 0 OR off-screen:  p->alive = false
 *
 *   Order: drag BEFORE integrate so dt-cap doesn't tunnel the particle
 *   past the screen edge in one frame.
 *
 *
 * burst_tick(b, cols, rows, ignite_chance, particle_count):    (FSM advance)
 *   switch (b->state):
 *     BS_IDLE     b->idle_ticks--; if reached 0 + roll < ignite_chance:
 *                 burst_ignite(b, cols, rows)         transition → FLASH
 *     BS_FLASH    b->flash_ttl--; if 0:                transition → LIVE
 *     BS_LIVE     for each p in b->particles:
 *                   particle_tick(p, cols, rows)
 *                   if alive: scorch_grid[p.cell]++
 *                 if no particles alive:
 *                   burst_complete_and_rearm(b)       transition → IDLE
 *
 *   The FSM is the entire algorithmic spine of this file.  Every visual
 *   you see traces back to one of these three states.
 *
 *
 * burst_draw(b, w, cols, rows):                    (per-burst render)
 *   if BS_FLASH:    draw_flash_cross(centre)        bright '*+' cross
 *   if BS_LIVE:     for each alive p: particle_draw(p, w)
 *                                                    (glyph + hue + bold tier)
 *                   draw_fuse_overlay(b)             debug-mode-only
 *
 *   Flash and live phases are mutually exclusive (the FSM enforces it),
 *   so no z-order conflicts in the same burst.  Active bursts paint OVER
 *   the persistent scorch layer.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * Keys
 *   q | Q | ESC      quit                space      pause / resume
 *   t / T            next / prev theme
 *   d / D            cycle debug overlay (off / vector / fuse / cell-grid)
 *   + / =            ignite an extra burst right now (manual trigger)
 *   ] / [            sim Hz up / down
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/burst.c \
 *       -o burst -lncurses -lm
 */

/* ── REFERENCES ──────────────────────────────────────────────────────── *
 *
 * PAPERS
 *   Reeves, W. T. (1983)
 *     "Particle Systems: A Technique for Modelling a Class of Fuzzy Objects"
 *     ACM Transactions on Graphics 2(2): 91–108.
 *
 * BOOKS
 *   Press et al. — "Numerical Recipes in C"  (3rd ed., 2007)
 *     §17.x covers the fixed-step accumulator pattern; §1.1.3 the
 *     squared-distance / no-sqrt idiom used in particle culling.
 *   Watt, A. & Watt, M. — "Advanced Animation and Rendering Techniques"
 *     (Addison-Wesley, 1992)
 *     Ch. 13 covers explicit Euler + exponential damping for particle
 *     systems and the trade-offs vs Verlet integration.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

enum {
    SIM_FPS_MIN      =  5,
    SIM_FPS_DEFAULT  = 24,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  4,

    BURSTS_MIN       =  1,
    BURSTS_DEFAULT   =  5,
    BURSTS_MAX       = 16,

    PARTICLES        = 48,    /* sparks per burst                          */
    BURST_TICKS      = 22,    /* max LIVE-state duration                   */
    FUSE_MIN         =  8,    /* idle-fuse minimum (ticks)                 */
    FUSE_RANGE       = 20,    /* idle-fuse extra uniform range             */

    BURST_WAVES      =  4,    /* concentric rings inside one burst         */
    BURST_MAX_DELAY  =  5,    /* outermost wave's spawn delay (ticks)      */

    HUD_COLS         = 64,    /* fits fps + spd + burst + [theme] + dbg    */
    FPS_UPDATE_MS    = 500,
};

/*
 * Float constants that read better as #define than as enum.  Grouped by
 * concept and annotated with units / role so the inner loops below stay
 * pure mechanics — every magic number lives here.
 */
#define DRAG_FACTOR              0.82f   /* per-tick velocity retention (≈18% loss) */
#define FLASH_LIFE_THRESHOLD     0.65f   /* sparks bold while life > this           */

#define BURST_ANGLE_JITTER       0.2f    /* radians of extra angle per spark        */
#define BURST_SPEED_MIN          1.8f    /* pixels / tick (lower bound)             */
#define BURST_SPEED_MAX          4.6f    /* pixels / tick (upper bound)             */

#define PARTICLE_LIFE_MIN        0.8f    /* fresh-spawn life (lower bound)          */
#define PARTICLE_LIFE_MAX        1.0f    /* fresh-spawn life (upper bound)          */
#define PARTICLE_DECAY_MIN       0.05f   /* per-tick life decay (lower bound)       */
#define PARTICLE_DECAY_MAX       0.09f   /* per-tick life decay (upper bound)       */

#define FUSE_NEVER               (INT32_MAX / 2)   /* idle slot's fuse: never fires */

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

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
/* §3  random + math — uniform samples + integer clamp                   */
/* ===================================================================== */

/* rand_unit / rand_range / rand_int_below / clamp_int — named primitives. */
static inline float rand_unit(void) { return (float)rand() / RAND_MAX; }
static inline float rand_range(float lo, float hi) { return lo + rand_unit() * (hi - lo); }
static inline int   rand_int_below(int n) { return rand() % n; }
static inline int   clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ===================================================================== */
/* §4  themes — 10 hue palettes + Hue enum + theme_apply + color_init    */
/* ===================================================================== */

typedef enum {
    C_RED     = 1,
    C_ORANGE  = 2,
    C_YELLOW  = 3,
    C_GREEN   = 4,
    C_CYAN    = 5,
    C_BLUE    = 6,
    C_MAGENTA = 7,
    C_COUNT   = 7,
} Hue;

/* HUD/HINT pairs sit OUTSIDE the 7-hue rendering range so theme cycling
 * never clobbers them. */
#define PAIR_HUD   8
#define PAIR_HINT  9

/*
 * BurstTheme — one palette: 7 hue slots indexed by the Hue enum.
 *
 *   name        cycled label, shown in HUD
 *   fg256       256-cube colour per slot  (rich terminals)
 *   fg8         8-colour fallback per slot
 *
 * Lifecycle: const table indexed at runtime by the t/T key handler.
 *   theme_apply() rebinds colour-pair IDs 1..C_COUNT to the chosen
 *   entry; live particles + scorch pick up the new colour on the next
 *   mvaddch — no clear, no flicker.
 */
typedef struct {
    const char *name;
    int         fg256[C_COUNT];
    int         fg8  [C_COUNT];
} BurstTheme;

#define THEME_COUNT  10

static const BurstTheme k_themes[THEME_COUNT] = {
    { "matrix",
      {  22,  28,  34,  40,  46,  82, 118 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_WHITE } },
    { "neon",
      { 201, 207, 213, 159, 226, 195,  51 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,
        COLOR_YELLOW,  COLOR_CYAN,    COLOR_CYAN } },
    { "nova",
      {  52,  88, 124, 160, 196, 208, 220 },
      { COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
        COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW } },
    { "ocean",
      {  24,  31,  39,  45,  51, 123, 195 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_WHITE, COLOR_WHITE } },
    { "fire",
      { 196, 202, 208, 214, 220, 226, 231 },
      { COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE } },
    { "toxic",
      {  28,  40,  46, 154, 190, 226, 220 },
      { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "gold",
      { 130, 136, 178, 214, 220, 226, 230 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE } },
    { "ice",
      {  21,  27,  33,  39,  45,  51, 195 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },
    { "aurora",
      {  28,  35,  50,  86, 121, 207, 219 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN,
        COLOR_CYAN,  COLOR_MAGENTA, COLOR_MAGENTA } },
    { "plasma",
      {  53,  91, 129, 165, 207, 213,  51 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_CYAN,    COLOR_CYAN } },
};

/* theme_apply — rebind the 7 hue pair IDs (1..C_COUNT) to k_themes[theme]. */
static void theme_apply(int theme)
{
    const BurstTheme *th = &k_themes[theme];
    for (int i = 0; i < C_COUNT; i++) {
        int slot = i + 1;        /* C_RED=1 .. C_MAGENTA=7 */
        if (COLORS >= 256)
            init_pair(slot, th->fg256[i], COLOR_BLACK);
        else
            init_pair(slot, th->fg8[i],   COLOR_BLACK);
    }
}

/* color_init — start_color + apply initial theme + pin HUD/HINT + debug pairs. */
static void color_init(int theme)
{
    start_color();
    use_default_colors();
    theme_apply(theme);

    /* HUD pairs are theme-independent — init ONCE, theme_apply leaves them alone. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);

    /* Debug overlay pairs (pairs 10..13) — also theme-independent.
     * Used only by debug rendering modes; reserved bright distinct hues. */
    if (COLORS >= 256) {
        init_pair(10, 196, -1);   /* wave 0: red       */
        init_pair(11, 226, -1);   /* wave 1: yellow    */
        init_pair(12,  46, -1);   /* wave 2: green     */
        init_pair(13,  51, -1);   /* wave 3: cyan      */
    } else {
        init_pair(10, COLOR_RED,    -1);
        init_pair(11, COLOR_YELLOW, -1);
        init_pair(12, COLOR_GREEN,  -1);
        init_pair(13, COLOR_CYAN,   -1);
    }
}

static Hue hue_rand(void) { return (Hue)(1 + rand() % C_COUNT); }

/* ===================================================================== */
/* §5  debug overlay — turn the simulation into a learning instrument    */
/* ===================================================================== */

typedef enum {
    DBG_NORMAL   = 0,    /* production rendering (artistic view)            */
    DBG_WAVES    = 1,    /* colour each spark by wave index (visible stagger)*/
    DBG_VELOCITY = 2,    /* replace glyph with arrow showing velocity dir   */
    DBG_FUSE     = 3,    /* write FSM countdown/age as number at centre    */
    DBG_COUNT    = 4,
} DebugMode;

static const char *const k_debug_names[DBG_COUNT] = {
    "normal", "waves", "velocity", "fuse",
};

static DebugMode g_debug = DBG_NORMAL;

/* dir_char — atan2(vy, vx) → 8-octant ASCII arrow for DBG_VELOCITY. */
static char dir_char(float vx, float vy)
{
    static const char k_dirs[8] = { '>', '\\', 'v', '/', '<', '\\', '^', '/' };
    float a = atan2f(vy, vx);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    int idx = (int)((a + (float)M_PI / 8.0f) / ((float)M_PI / 4.0f)) & 7;
    return k_dirs[idx];
}

/* Wave index → debug-palette colour pair (10..13). */
static int wave_pair(int wave) { return 10 + (wave & 3); }

/* ===================================================================== */
/* §6  particle — Particle struct + spawn + tick + draw                   */
/* ===================================================================== */

#define ASPECT 2.0f

/*
 * Particle — the atom of the simulation.  One ASCII spark.
 *
 *   cx, cy      burst centre anchor                 (terminal CELLS)
 *   rx, ry      offset from centre                  (pixels — sub-cell)
 *   vx, vy      velocity                            (pixels/tick)
 *   life        remaining lifetime                  (1.0 fresh → 0.0 dead)
 *   decay       per-tick life loss                  (per-particle jitter)
 *   delay       wave-stagger countdown              (frames before active)
 *   wave        0..3 — set at spawn, read by DBG_WAVES overlay
 *   sym         glyph fixed at spawn for trackability
 *   hue         colour pair fixed at spawn
 *   alive       false → particle slot is free, skip in tick + draw
 *
 * Lifecycle: particle_spawn at burst ignite → particle_tick per frame →
 *   particle_draw per frame.  When life ≤ 0 OR off-screen, alive=false
 *   and the slot becomes available (no malloc/free; pool is fixed).
 *
 * Why pixel-space offset (not cell): sub-cell precision keeps motion
 * smooth at low speeds.  ASPECT (=2) compensates for terminal cells
 * being ~2× as tall as wide; without it horizontal motion would look
 * 2× faster than vertical at the same |vx|=|vy|.
 */
typedef struct {
    float cx, cy;
    float rx, ry;
    float vx, vy;
    float life;
    float decay;
    int   delay;
    int   wave;
    char  sym;
    Hue   hue;
    bool  alive;
} Particle;

static const char k_syms[] = "*.+o#@%&$!^~-=|/\\:;,`'\"";
#define NSYMS (int)(sizeof k_syms - 1)

/* particle_spawn — fill one Particle at burst-ignite time. */
static void particle_spawn(Particle *p, float cx, float cy,
                            float angle, float speed,
                            int delay_ticks, int wave)
{
    /* Anchor — burst centre in CELL space, never moves after spawn. */
    p->cx    = cx;
    p->cy    = cy;

    /* Offset — pixel-space deviation from centre, grows during tick. */
    p->rx    = 0.0f;
    p->ry    = 0.0f;

    /* Velocity — radial outward fan (polar → Cartesian). */
    p->vx    = cosf(angle) * speed;
    p->vy    = sinf(angle) * speed;

    /* Life budget — fresh in [LIFE_MIN, LIFE_MAX), drains by `decay`/tick. */
    p->life  = rand_range(PARTICLE_LIFE_MIN,  PARTICLE_LIFE_MAX);
    p->decay = rand_range(PARTICLE_DECAY_MIN, PARTICLE_DECAY_MAX);

    /* FSM bookkeeping — wave index and per-spark spawn delay. */
    p->delay = delay_ticks;
    p->wave  = wave;

    /* Appearance — glyph and hue fixed at spawn so the eye can track. */
    p->sym   = k_syms[rand_int_below(NSYMS)];
    p->hue   = hue_rand();
    p->alive = true;
}

/*
 * particle_tick — DRIVER.  One frame of per-spark physics.
 *
 *   if !p->alive:           return                       dead → inert
 *   if p->delay > 0:        p->delay--;  return          wave-stagger gate
 *   p->vx *= DRAG_FACTOR                                 multiplicative drag
 *   p->vy *= DRAG_FACTOR
 *   p->rx += p->vx;  p->ry += p->vy                      explicit Euler in
 *                                                         pixel space
 *   p->life -= p->decay                                  linear life loss
 *   screen = p->cx + p->rx · ASPECT, p->cy + p->ry        ASPECT-corrected
 *   if life ≤ 0 OR off-screen:  p->alive = false
 *
 * INPUTS / UNITS
 *   p              Particle mutated in place.
 *   cols, rows     terminal extent in CELLS, for the off-screen test.
 *   vx, vy in pixels/tick; rx, ry in pixels.  ASPECT (=2) compensates
 *   for terminal cell aspect ratio at the screen-bounds check (and at
 *   particle_draw — the same one place that owns the pixel→cell map).
 *
 * WHY DRAG BEFORE INTEGRATE
 *   Drag-then-integrate uses the new (slowed) velocity for this frame's
 *   step, matching how a Stokes drag actually behaves and preventing
 *   the "tunnelling past the wall" artefact when dt-cap kicks in.
 *
 * WHY IT EXISTS (vs. inlining in burst_tick)
 *   burst_tick orchestrates the FSM and counts living sparks.  Keeping
 *   per-spark physics here means burst_tick stays at FSM level — no
 *   mixing of state-machine and integrator concerns.
 */
static void particle_tick(Particle *p, int cols, int rows)
{
    /* Gate 1 — dead sparks are inert. */
    if (!p->alive) return;

    /* Gate 2 — wave-stagger delay holds the spark in its starting cell. */
    if (p->delay > 0) { p->delay--; return; }

    /* Step 1 — multiplicative drag (Stokes-like: force ∝ velocity). */
    p->vx *= DRAG_FACTOR;
    p->vy *= DRAG_FACTOR;

    /* Step 2 — explicit Euler integration in pixel space. */
    p->rx += p->vx;
    p->ry += p->vy;

    /* Step 3 — age the life counter. */
    p->life -= p->decay;

    /* Step 4 — die if life is exhausted OR the spark left the screen.
     *           ASPECT compensation applied so the bounds test matches
     *           what particle_draw will actually paint. */
    float screen_x   = p->cx + p->rx * ASPECT;
    float screen_y   = p->cy + p->ry;
    bool  burned_out = (p->life <= 0.0f);
    bool  off_screen = (screen_x < 0.f || screen_x >= (float)cols
                     || screen_y < 0.f || screen_y >= (float)rows);
    if (burned_out || off_screen) p->alive = false;
}

/* particle_pixel_to_cell — pixel-space → terminal cell with ×ASPECT in x. */
static void particle_pixel_to_cell(const Particle *p, int *cell_x, int *cell_y)
{
    *cell_x = (int)(p->cx + p->rx * ASPECT);
    *cell_y = (int)(p->cy + p->ry);
}

/* particle_draw — paint one spark; switches on g_debug for overlay modes. */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows)
{
    /* Gates — same suppression rules as particle_tick. */
    if (!p->alive || p->delay > 0) return;

    int cell_x, cell_y;
    particle_pixel_to_cell(p, &cell_x, &cell_y);
    if (cell_x < 0 || cell_x >= cols || cell_y < 0 || cell_y >= rows) return;

    /*
     * Debug-mode dispatch — pick (glyph, attr) per mode.  Physics is
     * identical across modes; only the rendering changes.
     */
    chtype glyph;
    attr_t attr;
    switch (g_debug) {
    case DBG_WAVES:
        /* Colour by wave (0..3) so staggered emission is legible. */
        glyph = (chtype)(unsigned char)p->sym;
        attr  = COLOR_PAIR(wave_pair(p->wave)) | A_BOLD;
        break;
    case DBG_VELOCITY:
        /* Replace glyph with an arrow showing velocity direction. */
        glyph = (chtype)(unsigned char)dir_char(p->vx, p->vy);
        attr  = COLOR_PAIR(p->hue) | A_BOLD;
        break;
    case DBG_FUSE:
    case DBG_NORMAL:
    default: {
        /* Production view — fixed glyph + fading bold gate. */
        bool is_fresh_spark = (p->life > FLASH_LIFE_THRESHOLD);
        glyph = (chtype)(unsigned char)p->sym;
        attr  = COLOR_PAIR(p->hue) | (is_fresh_spark ? A_BOLD : 0);
        break;
    }
    }

    wattron(w, attr);
    mvwaddch(w, cell_y, cell_x, glyph);
    wattroff(w, attr);
}

/* ===================================================================== */
/* §7  burst FSM — Burst struct + state enum + burst_ignite               */
/* ===================================================================== */

typedef enum {
    BS_IDLE  = 0,    /* fuse counting down; no visible state                */
    BS_FLASH = 1,    /* exactly ONE tick of central '*+' cross flash        */
    BS_LIVE  = 2,    /* particles flying; transitions back to IDLE when all dead */
} BurstState;

/*
 * Burst — one explosion.  A pool of Particles plus a 3-state FSM.
 *
 *   cx, cy      detonation centre              (terminal CELLS)
 *   state       BS_IDLE / BS_FLASH / BS_LIVE
 *   ticks       counts UP from 0 in BS_LIVE; reset on each ignite
 *   fuse        counts DOWN in BS_IDLE; ignites at 0 (probabilistic)
 *   parts       inline pool of PARTICLES (=48) sparks
 *
 * Lifecycle:
 *   IDLE  ── fuse hits 0 ─────▶ FLASH   (burst_ignite spawns all sparks)
 *   FLASH ── one tick ────────▶ LIVE
 *   LIVE  ── all dead OR ticks≥BURST_TICKS ─▶ IDLE  (re-arm fuse)
 *
 * Why three states (vs single "active" flag): the FLASH frame's bright
 * central '*+' cross gives the explosion its "BANG, then shrapnel"
 * reading.  Without that one-frame anchor, the burst looks like just
 * a puff of particles — which doesn't read as an explosion.
 *
 * The IDLE fuse gives the screen its rhythm: 16 bursts each independently
 * counting down (8–28 ticks) means roughly one burst per
 * (sim_fps / N_BURSTS / mean_fuse) seconds.  Each slot is independent
 * — no global scheduler.
 */
typedef struct {
    float      cx, cy;
    BurstState state;
    int        ticks;
    int        fuse;
    Particle   parts[PARTICLES];
} Burst;

/* pick_detonation_centre — random (cx, cy) inside a safe area for the FLASH cross. */
static void pick_detonation_centre(int cols, int rows, float *cx, float *cy)
{
    int safe_cols_extent = (cols - 4) > 1 ? (cols - 4) : 1;
    int safe_rows_extent = (rows - 2) > 1 ? (rows - 2) : 1;
    *cx = (float)(2 + rand_int_below(safe_cols_extent));
    *cy = (float)(1 + rand_int_below(safe_rows_extent));
}

/* compute_emission_angle — (i/total)·2π + small jitter; organic, not mathematical. */
static float compute_emission_angle(int i, int total_particles)
{
    float evenly_spaced = ((float)i / (float)total_particles) * 2.0f * (float)M_PI;
    float jitter        = rand_unit() * BURST_ANGLE_JITTER;
    return evenly_spaced + jitter;
}

/* compute_emission_speed — uniform in [BURST_SPEED_MIN, BURST_SPEED_MAX). */
static float compute_emission_speed(void)
{
    return rand_range(BURST_SPEED_MIN, BURST_SPEED_MAX);
}

/*
 * compute_wave_delay() — staggered spawn delay so the burst expands as
 * a multi-ring shockwave instead of a single instant ring.
 *
 *   delay = wave × MAX_DELAY / (waves - 1)
 *   With waves=4, MAX_DELAY=5 → delays 0, 1 (rounded down), 3, 5 ticks.
 */
static int compute_wave_delay(int wave, int wave_count, int max_delay)
{
    if (wave_count <= 1) return 0;
    return (wave * max_delay) / (wave_count - 1);
}

/*
 * burst_ignite() — pick a detonation point and seed all 48 sparks.
 *
 *   The body is three labelled steps:
 *     Step 1 — pick the detonation centre (safe-area clamp).
 *     Step 2 — enter FLASH state for exactly one tick.
 *     Step 3 — spawn PARTICLES sparks in a wave-staggered radial fan.
 */
static void burst_ignite(Burst *b, int cols, int rows)
{
    /* Step 1 — detonation point. */
    pick_detonation_centre(cols, rows, &b->cx, &b->cy);

    /* Step 2 — enter FLASH state. */
    b->state = BS_FLASH;
    b->ticks = 0;

    /* Step 3 — spawn the radial fan. */
    for (int i = 0; i < PARTICLES; i++) {
        float angle = compute_emission_angle(i, PARTICLES);
        float speed = compute_emission_speed();
        int   wave  = i % BURST_WAVES;
        int   delay = compute_wave_delay(wave, BURST_WAVES, BURST_MAX_DELAY);
        particle_spawn(&b->parts[i], b->cx, b->cy, angle, speed, delay, wave);
    }
}

/* ===================================================================== */
/* §8  burst tick — FSM advance + complete + re-arm                       */
/* ===================================================================== */

/* burst_advance_live_particles — tick every spark; return whether any survived. */
static bool burst_advance_live_particles(Burst *b, int cols, int rows)
{
    bool any_alive = false;
    for (int i = 0; i < PARTICLES; i++) {
        particle_tick(&b->parts[i], cols, rows);
        if (b->parts[i].alive) any_alive = true;
    }
    return any_alive;
}

/* burst_complete_and_rearm — fire scorch_cb; set new random fuse; → IDLE. */
static void burst_complete_and_rearm(Burst *b,
                                     void (*scorch_cb)(int, int, void *),
                                     void *ud)
{
    if (scorch_cb) scorch_cb((int)b->cx, (int)b->cy, ud);
    b->fuse  = FUSE_MIN + rand_int_below(FUSE_RANGE);
    b->state = BS_IDLE;
}

/*
 * burst_tick — DRIVER.  Run one Burst FSM step.
 *
 *   switch (b->state):
 *     BS_IDLE   b->fuse--; if fuse ≤ 0:  burst_ignite     IDLE → FLASH
 *     BS_FLASH  b->state = BS_LIVE; b->ticks = 0           FLASH → LIVE
 *     BS_LIVE   any_alive = burst_advance_live_particles(b)
 *               b->ticks++
 *               if !any_alive OR b->ticks ≥ BURST_TICKS:
 *                   burst_complete_and_rearm(b, scorch_cb, ud)
 *                                                          LIVE → IDLE
 *
 * INPUTS / UNITS
 *   b              Burst mutated; state machine + every parts[i] advance.
 *   cols, rows     terminal extent (passed through to particle_tick).
 *   scorch_cb, ud  callback fired ONCE on LIVE → IDLE; stamps the scorch
 *                  grid with the burst's centre.
 *
 * The FSM is the algorithmic spine of this file.  Every visible
 * frame-by-frame change traces back to one of these three branches.
 *
 * WHY IT EXISTS (vs. inlining in field_tick)
 *   field_tick is a one-line loop over the burst pool; all FSM detail
 *   lives here.  Engine vs orchestrator — each burst owns its own
 *   clock and never knows other bursts exist.
 */
static void burst_tick(Burst *b, int cols, int rows,
                       void (*scorch_cb)(int x, int y, void *ud), void *ud)
{
    switch (b->state) {
    case BS_IDLE:
        /* Countdown the fuse; ignite at zero. */
        b->fuse--;
        if (b->fuse <= 0) burst_ignite(b, cols, rows);
        break;

    case BS_FLASH:
        /* FLASH lasts exactly one tick. */
        b->state = BS_LIVE;
        b->ticks = 0;
        break;

    case BS_LIVE: {
        bool any_alive   = burst_advance_live_particles(b, cols, rows);
        b->ticks++;
        bool out_of_time = (b->ticks >= BURST_TICKS);
        if (!any_alive || out_of_time)
            burst_complete_and_rearm(b, scorch_cb, ud);
        break;
    }
    }
}

/* ===================================================================== */
/* §9  burst render — flash cross + fuse overlay + burst_draw             */
/* ===================================================================== */

/* draw_flash_cross — central '*' + four cardinal '+' in bright yellow + bold. */
static void draw_flash_cross(WINDOW *w, int cx, int cy, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    wattron(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
    mvwaddch(w, cy, cx, '*');
    if (cx > 0)       mvwaddch(w, cy,     cx - 1, '+');
    if (cx < cols-1)  mvwaddch(w, cy,     cx + 1, '+');
    if (cy > 0)       mvwaddch(w, cy - 1, cx,     '+');
    if (cy < rows-1)  mvwaddch(w, cy + 1, cx,     '+');
    wattroff(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
}

/* draw_fuse_overlay — DBG_FUSE annotation: "f<fuse>" (IDLE) or "t<ticks>" (LIVE). */
static void draw_fuse_overlay(const Burst *b, WINDOW *w,
                              int cx, int cy, int cols, int rows)
{
    bool centre_in_bounds = (cx >= 0 && cx < cols - 3 && cy >= 0 && cy < rows);
    if (!centre_in_bounds) return;

    char label[8];
    if      (b->state == BS_IDLE) snprintf(label, sizeof label, "f%d", b->fuse);
    else if (b->state == BS_LIVE) snprintf(label, sizeof label, "t%d", b->ticks);
    else return;

    wattron(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvwaddstr(w, cy, cx, label);
    wattroff(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/*
 * burst_draw — DRIVER.  State-dependent per-burst render.
 *
 *   if BS_FLASH:    draw_flash_cross(centre)         '*'+ '+'cross, return
 *   if BS_LIVE:     for each alive p: particle_draw(p)   spark fan
 *   (BS_IDLE)       invisible — scorch layer is the field's job
 *
 *   if g_debug == DBG_FUSE:  draw_fuse_overlay overlays FSM counter
 *
 * INPUTS / UNITS
 *   b              Burst to render (const).
 *   w              destination WINDOW.
 *   cols, rows     terminal extent in cells.
 *
 * State branches are mutually exclusive (the FSM enforces it), so no
 * z-order conflicts within a single Burst.  Active Bursts paint OVER
 * the persistent scorch layer (field_draw_scorch_layer runs first).
 *
 * WHY IT EXISTS (vs. inlining in field_draw)
 *   field_draw is a one-line loop over the burst pool; per-state visual
 *   logic stays here.  Adding a new state's visual touches ONLY this
 *   function.
 */
static void burst_draw(const Burst *b, WINDOW *w, int cols, int rows)
{
    int cx = (int)b->cx;
    int cy = (int)b->cy;

    if (b->state == BS_FLASH) {
        draw_flash_cross(w, cx, cy, cols, rows);
        return;
    }

    if (b->state == BS_LIVE) {
        for (int i = 0; i < PARTICLES; i++)
            particle_draw(&b->parts[i], w, cols, rows);
    }

    if (g_debug == DBG_FUSE) draw_fuse_overlay(b, w, cx, cy, cols, rows);
}

/* ===================================================================== */
/* §10  field — burst pool + persistent scorch grid                       */
/* ===================================================================== */

/*
 * Field — burst pool + persistent scorch grid.
 *
 *   bursts[BURSTS_MAX]    independent Burst slots; each owns its own FSM
 *   scorch[cols × rows]   char grid: '.' at every past burst's centre
 *   cols, rows            terminal extent in cells
 *   active_bursts         live count [0, BURSTS_MAX]
 *
 * Lifecycle: field_init at startup + on resize + on 'r' → field_tick
 *   per frame → field_draw per frame → field_free at exit.
 *
 * Why scorch is callback-driven (not direct call): burst_tick doesn't
 * know about scorch — it just fires a callback when LIVE ends.  That
 * decoupling means burst.c could be reused with scorch_cb = NULL for
 * a memoryless "sky" variant.
 */
typedef struct {
    Burst  bursts[BURSTS_MAX];
    char  *scorch;
    int    cols;
    int    rows;
    int    active_bursts;
} Field;

/* field_scorch_cb — burst_tick LIVE→IDLE callback; stamps '.' at (x, y). */
static void field_scorch_cb(int x, int y, void *ud)
{
    Field *f = (Field *)ud;
    if (x >= 0 && x < f->cols && y >= 0 && y < f->rows)
        f->scorch[y * f->cols + x] = '.';
}

/* field_init — alloc scorch + init all slots; active slots get staggered fuses. */
static void field_init(Field *f, int cols, int rows, int burst_count)
{
    f->cols          = cols;
    f->rows          = rows;
    f->active_bursts = burst_count;
    f->scorch        = calloc((size_t)(cols * rows), sizeof(char));

    /* Active slots get staggered fuses so they don't all fire on frame 0;
     * inactive slots sit at FUSE_NEVER until '+' wakes them. */
    int stagger_step = FUSE_MIN + (burst_count > 0 ? FUSE_RANGE / burst_count : 0);

    for (int i = 0; i < BURSTS_MAX; i++) {
        memset(&f->bursts[i], 0, sizeof(Burst));
        f->bursts[i].state = BS_IDLE;
        f->bursts[i].fuse  = (i < burst_count) ? (i * stagger_step)
                                               : FUSE_NEVER;
    }
}

/* field_free — free scorch grid; zero the struct. */
static void field_free(Field *f)
{
    free(f->scorch);
    *f = (Field){0};
}

/* field_tick — burst_tick over every active slot. */
static void field_tick(Field *f)
{
    for (int i = 0; i < f->active_bursts; i++)
        burst_tick(&f->bursts[i], f->cols, f->rows, field_scorch_cb, f);
}

/* field_draw_scorch_layer — paint every non-zero scorch cell in dim orange.
 * Drawn first so live sparks overlay it. */
static void field_draw_scorch_layer(const Field *f, WINDOW *w)
{
    int total_cells = f->cols * f->rows;

    wattron(w, COLOR_PAIR(C_ORANGE) | A_DIM);
    for (int i = 0; i < total_cells; i++) {
        char scorch_glyph = f->scorch[i];
        if (!scorch_glyph) continue;
        int cell_y = i / f->cols;
        int cell_x = i % f->cols;
        mvwaddch(w, cell_y, cell_x, (chtype)(unsigned char)scorch_glyph);
    }
    wattroff(w, COLOR_PAIR(C_ORANGE) | A_DIM);
}

/* field_draw_active_bursts — burst_draw over every active slot (atop scorch). */
static void field_draw_active_bursts(const Field *f, WINDOW *w)
{
    for (int i = 0; i < f->active_bursts; i++)
        burst_draw(&f->bursts[i], w, f->cols, f->rows);
}

/* field_draw — two passes: scorch underneath, active bursts on top. */
static void field_draw(const Field *f, WINDOW *w)
{
    field_draw_scorch_layer (f, w);
    field_draw_active_bursts(f, w);
}

/* ===================================================================== */
/* §11  screen + HUD — ncurses init + status strip                        */
/* ===================================================================== */

typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s, int theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_field(Screen *s, const Field *f)
{
    (void)s;     /* s only needed for HUD; field draw uses stdscr directly */
    erase();
    field_draw(f, stdscr);
}

/*
 * HUD layout per CLAUDE.md:
 *   row 0        — fps + sim_fps + burst count in bright bold yellow (top-right)
 *   row rows-1   — every interactive key in bright bold cyan (bottom-left)
 * Both pairs are dedicated (PAIR_HUD / PAIR_HINT), never reused by sparks
 * or scorch, so the colour stays stable across all bursts.
 */
static void screen_draw_hud(Screen *s, double fps, int sim_fps, int bursts,
                            int theme)
{
    char buf[HUD_COLS + 1];
    /* Debug-mode suffix only shown when non-normal; keeps top-right tidy. */
    if (g_debug == DBG_NORMAL) {
        snprintf(buf, sizeof buf, " %5.1f fps  spd:%d  burst:%d  [%s] ",
                 fps, sim_fps, bursts, k_themes[theme].name);
    } else {
        snprintf(buf, sizeof buf,
                 " %5.1f fps  spd:%d  burst:%d  [%s]  dbg:%s ",
                 fps, sim_fps, bursts, k_themes[theme].name,
                 k_debug_names[g_debug]);
    }
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q/ESC:quit  ]/[:speed  +/-:bursts  r:clear-scorch"
             "  t/T:theme  d/D:debug ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §12  app — App struct + signal handlers + key dispatch                 */
/* ===================================================================== */

typedef struct {
    Field                 field;
    Screen                screen;
    int                   sim_fps;
    int                   bursts;
    int                   theme;      /* index into k_themes[] */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    field_free(&app->field);
    screen_resize(&app->screen);
    field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case '=': case '+':
        if (app->bursts < BURSTS_MAX) {
            int i = app->bursts;
            memset(&app->field.bursts[i], 0, sizeof(Burst));
            app->field.bursts[i].state = BS_IDLE;
            app->field.bursts[i].fuse  = 2 + rand() % FUSE_RANGE;
            app->bursts++;
            app->field.active_bursts = app->bursts;
        }
        break;
    case '-':
        if (app->bursts > BURSTS_MIN) {
            app->bursts--;
            app->field.active_bursts = app->bursts;
        }
        break;

    case 'r': case 'R':
        field_free(&app->field);
        field_init(&app->field, app->screen.cols, app->screen.rows,
                   app->bursts);
        break;

    case 't':
        app->theme = (app->theme + 1) % THEME_COUNT;
        theme_apply(app->theme);
        break;
    case 'T':
        app->theme = (app->theme + THEME_COUNT - 1) % THEME_COUNT;
        theme_apply(app->theme);
        break;

    case 'd':
        g_debug = (DebugMode)((g_debug + 1) % DBG_COUNT);
        break;
    case 'D':
        g_debug = (DebugMode)((g_debug + DBG_COUNT - 1) % DBG_COUNT);
        break;

    default: break;
    }
    return true;
}

/* ===================================================================== */
/* §13  main — fixed-step loop                                            */
/* ===================================================================== */

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    app->bursts  = BURSTS_DEFAULT;
    app->theme   = 0;       /* start on "rainbow" */

    screen_init(&app->screen, app->theme);
    field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            field_tick(&app->field);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── HUD counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── render + HUD ────────────────────────────────────────── */
        screen_draw_field(&app->screen, &app->field);
        screen_draw_hud(&app->screen, fps_display,
                         app->sim_fps, app->bursts, app->theme);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    field_free(&app->field);
    screen_free(&app->screen);
    return 0;
}
