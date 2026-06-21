/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * slime_mold.c — slime mold (Physarum) network simulation.
 *
 * Thousands of agents wander a grid, sniff the chemical trail just ahead,
 * steer toward the strongest scent, step forward, and leave more trail.
 * The trail spreads and fades each tick. Out of those simple local rules,
 * tube-like networks self-organize and connect the food sources — no
 * central control. Based on Jones (2010), "Characteristics of pattern
 * formation and evolution in approximations of Physarum transport
 * networks", Artificial Life 16(2).
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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — tunable constants ── */

#define ROWS_MAX        128
#define COLS_MAX        512

/* Agent population */
#define N_AGENTS_DEF   2000  /* default; fewer and the tubes never form, more and fps drops */
#define N_AGENTS_MIN    200  /* below this the network is too sparse to hold together */
#define N_AGENTS_MAX   6000  /* above this it gets too slow on typical hardware       */
#define N_AGENTS_STEP   200

/* How each agent looks ahead and turns (Jones 2010 values) */
#define SENSOR_ANGLE  ((float)M_PI / 4.0f)   /* sensors sit 45 deg either side of straight ahead */
#define SENSOR_DIST    4.0f                   /* how many cells ahead the sensors reach;
                                               * shorter = tighter curves, longer = straighter tubes */
#define ROTATE_ANGLE  ((float)M_PI / 4.0f)   /* agents snap 45 deg per turn — sharp turns make
                                               * crisp tubes; soft turns smear into blobs */
#define STEP_SIZE      1.0f                   /* cells moved per tick */

/* Trail (the chemical the agents leave and follow) */
#define DEPOSIT_DEF    5.0f    /* trail dropped per agent per tick (multiplied near food) */
#define MAX_TRAIL     100.0f   /* trail can't exceed this — keeps values bounded */
#define DECAY_DEF      0.08f   /* fraction of trail that fades away each tick (8%) */
#define DIFFUSE_DEF    0.35f   /* how much trail blurs into its neighbours each tick;
                                * higher = smoother but fuzzier network */

/* Food sources — the points the network learns to connect */
#define N_FOOD          3
#define FOOD_RADIUS     3.0f   /* how close an agent must be to count as "at" a food source */
#define FOOD_BONUS      6.0f   /* agents drop 6x more trail near food, so it pulls them in */
#define FOOD_MIN_TRAIL 30.0f   /* keep food cells at least this strong so they never fade out */

/* Simulation */
#define SIM_FPS_DEF    30
#define SIM_FPS_MIN     5
#define SIM_FPS_MAX    60
#define SIM_FPS_STEP    5

#define N_PRESETS       4
#define N_THEMES        5

#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §2 clock — monotonic timer and sleep ── */

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

/* ── §3 color / theme — palette and ncurses pairs ── */

/* Colour-pair slots. The five trail pairs and the food pair change with
 * the theme; the two HUD pairs are fixed so the status text stays
 * readable over any theme. */
enum {
    CP_T1 = 1,  /* dimmest trail  */
    CP_T2 = 2,
    CP_T3 = 3,
    CP_T4 = 4,
    CP_T5 = 5,  /* brightest trail */
    CP_FOOD = 6,
    CP_HUD  = 7,
    CP_HINT = 8,
};

/*
 * Theme — one named colour scheme: five trail colours (faint to bright)
 * plus a food-marker colour. Switching themes recolours the whole
 * picture (amber slime, cold cyan, fiery red, ...). Each theme carries
 * two palettes: a rich 256-colour one and a fallback for terminals that
 * only have the 8 basic colours.
 *
 * Members
 *   t[5]    256-colour trail colours, faint -> bright. Trail strength
 *           picks a bucket: weakest -> t[0], strongest -> t[4].
 *   t8[5]   same five, but for 8-colour terminals.
 *   food    food-marker colour (256-colour).
 *   food8   food-marker colour (8-colour).
 *   name    label shown in the HUD ("Physarum", "Cyan", ...); never NULL.
 *
 * All colour indices stay inside the terminal's valid range.
 */
typedef struct {
    short t[5];        /* trail colours, faint->bright (256-colour) */
    short t8[5];       /* same, 8-colour fallback                   */
    short food;        /* food-marker colour, 256-colour            */
    short food8;       /* food-marker colour, 8-colour              */
    const char *name;  /* HUD label                                 */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 Physarum — yellow/amber network on black, like real slime mold */
    { {22, 58, 100, 136, 220},
      {COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
      196, COLOR_RED, "Physarum" },
    /* 1 Cyan — cold network */
    { {17, 19, 27, 39, 87},
      {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
      226, COLOR_YELLOW, "Cyan" },
    /* 2 Neon — magenta/violet */
    { {53, 91, 129, 165, 207},
      {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE},
      226, COLOR_YELLOW, "Neon" },
    /* 3 Forest — organic green */
    { {22, 28, 34, 40, 118},
      {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE},
      196, COLOR_RED, "Forest" },
    /* 4 Lava — red/orange */
    { {52, 88, 124, 166, 226},
      {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
      231, COLOR_WHITE, "Lava" },
};

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    for (int i = 0; i < 5; i++) {
        if (COLORS >= 256)
            init_pair(i + 1, th->t[i],  -1);
        else
            init_pair(i + 1, th->t8[i], -1);
    }
    if (COLORS >= 256) init_pair(CP_FOOD, th->food,  -1);
    else               init_pair(CP_FOOD, th->food8, -1);

    /* The two HUD pairs are not touched here — see color_init. */
}

static void color_init(void)
{
    enum { HUD_YELLOW_256 = 226, HUD_CYAN_256 = 51 };

    start_color();
    use_default_colors();

    /* HUD colours: bright yellow for the status row, bright cyan for the
     * key hints. Set once here so cycling themes never disturbs them. */
    if (COLORS >= 256) {
        init_pair(CP_HUD,  HUD_YELLOW_256, -1);
        init_pair(CP_HINT, HUD_CYAN_256,   -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW,   -1);
        init_pair(CP_HINT, COLOR_CYAN,     -1);
    }

    theme_apply(0);
}

/* ── §4 trail grid — the chemical field agents read and write ── */

/*
 * TrailField — the grid of trail strengths the slime mold lives on.
 * Each cell holds how much chemical is there. Agents read it to decide
 * where to go and add to it where they step; every tick the field also
 * blurs a little into its neighbours and fades.
 *
 * It keeps TWO grids on purpose. The blur step reads each cell's
 * neighbours, so if we wrote results back into the same grid, cells
 * computed later would see already-changed neighbours and the result
 * would depend on scan order (lopsided artefacts). Instead we read the
 * live grid, write into a scratch grid, then copy scratch back. Both
 * grids are fixed-size (~512 KB total) and allocated once, so there is
 * no per-frame allocation and the hot loop hits a stable address.
 *
 * Members
 *   current[r][c]   live trail strength at cell (r, c). Read by agents
 *                   and the renderer; written by deposits and the
 *                   blur+fade step. Stays in [0, MAX_TRAIL].
 *   buffer [r][c]   scratch grid for the blur+fade step; never seen by
 *                   agents or the renderer. Filled by trail_update, then
 *                   copied back to current. Stays >= 0.
 *   active_rows     rows actually in use (terminal rows minus 1; the
 *                   bottom row is reserved for the HUD). 0..ROWS_MAX.
 *   active_cols     columns actually in use (= terminal columns).
 *                   0..COLS_MAX. Cells beyond the active area are zero.
 *
 * Based on Jones (2010) — same blur-and-fade trail rule.
 */
typedef struct {
    float current[ROWS_MAX][COLS_MAX];
    float buffer [ROWS_MAX][COLS_MAX];
    int   active_rows;
    int   active_cols;
} TrailField;

/* The one trail grid (~512 KB), allocated once. Scene.trail points at it. */
static TrailField g_trail_field;

/* Wrap a row/column back into range so the grid behaves like a loop (the
 * top edge touches the bottom, the left edge touches the right). */
static inline int wrap_r_on(const TrailField *t, int r) {
    return (r % t->active_rows + t->active_rows) % t->active_rows;
}
static inline int wrap_c_on(const TrailField *t, int c) {
    return (c % t->active_cols + t->active_cols) % t->active_cols;
}

/* Read trail strength at a (column, row) point, wrapping at the edges. */
static float trail_sample(const TrailField *t, float fc, float fr)
{
    int c = wrap_c_on(t, (int)(fc + 0.5f));
    int r = wrap_r_on(t, (int)(fr + 0.5f));
    return t->current[r][c];
}

/* Add `amount` of trail at a (column, row) point, never past MAX_TRAIL. */
static void trail_deposit(TrailField *t, float fc, float fr, float amount)
{
    int c = wrap_c_on(t, (int)(fc + 0.5f));
    int r = wrap_r_on(t, (int)(fr + 0.5f));
    t->current[r][c] += amount;
    if (t->current[r][c] > MAX_TRAIL) t->current[r][c] = MAX_TRAIL;
}

/* Average a cell with its eight neighbours (a 3x3 block, wrapping at the
 * edges). This is the blur: each cell drifts toward its surroundings. */
static float box_average_3x3_wrapped(const TrailField *t, int r, int c)
{
    enum { KERNEL_HALF = 1 };                /* 3x3 block reaches one cell out */
    const float KERNEL_CELL_COUNT = 9.0f;    /* 3x3 = 9 cells                  */

    float sum = 0.0f;
    for (int dr = -KERNEL_HALF; dr <= KERNEL_HALF; dr++)
        for (int dc = -KERNEL_HALF; dc <= KERNEL_HALF; dc++)
            sum += t->current[wrap_r_on(t, r+dr)][wrap_c_on(t, c+dc)];
    return sum / KERNEL_CELL_COUNT;
}

/* Advance the trail one tick: blur each cell toward its neighbours, then
 * fade. Writes into the scratch grid and copies it back so every cell is
 * computed from the same starting state (order doesn't matter). */
static void trail_update(TrailField *t, float diffuse_w, float decay)
{
    const float retain = 1.0f - decay;
    int   R = t->active_rows;
    int   C = t->active_cols;

    for (int r = 0; r < R; r++) {
        for (int c = 0; c < C; c++) {
            float neighbour_avg   = box_average_3x3_wrapped(t, r, c);
            float self            = t->current[r][c];

            /* blur: slide partway from this cell toward its neighbours */
            float after_diffusion = self * (1.0f - diffuse_w)
                                  + neighbour_avg * diffuse_w;

            /* fade: shave off the decay fraction */
            float after_decay     = after_diffusion * retain;

            /* guard against tiny negative rounding errors */
            t->buffer[r][c] = (after_decay > 0.0f) ? after_decay : 0.0f;
        }
    }

    memcpy(t->current, t->buffer, sizeof t->current);
}

/* Zero both grids. */
static void trail_clear(TrailField *t)
{
    memset(t->current, 0, sizeof t->current);
    memset(t->buffer,  0, sizeof t->buffer);
}

/* Set how much of the (fixed-size) grid is in use, capped at the max. */
static void trail_resize(TrailField *t, int rows, int cols)
{
    if (rows > ROWS_MAX) rows = ROWS_MAX;
    if (cols > COLS_MAX) cols = COLS_MAX;
    t->active_rows = rows;
    t->active_cols = cols;
}

/* Pick a character and colour for a trail strength. Stronger trail gets a
 * heavier glyph, so the tubes read even on 8-colour terminals:
 *   blank . + x # @   (weakest to strongest) */
static void trail_char_pair(float v, chtype *ch_out, int *cp_out)
{
    if      (v < 0.3f)  { *ch_out = ' '; *cp_out = CP_T1; }
    else if (v < 2.0f)  { *ch_out = '.'; *cp_out = CP_T1; }
    else if (v < 8.0f)  { *ch_out = '+'; *cp_out = CP_T2; }
    else if (v < 20.0f) { *ch_out = 'x'; *cp_out = CP_T3; }
    else if (v < 50.0f) { *ch_out = '#'; *cp_out = CP_T4; }
    else                { *ch_out = '@'; *cp_out = CP_T5; }
}

static void trail_draw(const TrailField *t, int rows, int cols)
{
    int max_r = (rows < t->active_rows) ? rows : t->active_rows;
    int max_c = (cols < t->active_cols) ? cols : t->active_cols;

    for (int r = 0; r < max_r; r++) {
        for (int c = 0; c < max_c; c++) {
            chtype ch; int cp;
            trail_char_pair(t->current[r][c], &ch, &cp);
            if (ch == ' ') continue;  /* skip background cells */
            attron(COLOR_PAIR(cp));
            mvaddch(r, c, ch);
            attroff(COLOR_PAIR(cp));
        }
    }
}

/* ── §5 agents — one particle and its sense-turn-move-deposit step ── */

/*
 * Agent — one slime-mold particle. Each tick it sniffs the trail at three
 * points ahead, turns toward the strongest, steps forward, and drops more
 * trail. Agents never look at each other directly; they only talk through
 * the trail they leave behind (this indirect coordination is called
 * stigmergy). They have no mass or momentum — speed is fixed and turns are
 * sharp 45-degree snaps, which is what makes the tubes come out crisp
 * rather than smeared.
 *
 * Members
 *   x, y    position in cell coordinates, kept as floats so a deposit can
 *           land between cells. Wrapped back in range each step.
 *   angle   heading in radians; sets which way the agent faces. No fixed
 *           range — sine and cosine handle any value.
 */
typedef struct {
    float x, y;     /* cell-space position, float for sub-cell deposits */
    float angle;    /* heading in radians                               */
} Agent;

/*
 * FoodSrc — one fixed food source: a point the network tries to reach.
 * Agents passing near it drop extra trail (so it pulls in more agents),
 * its cell is kept from fading out, and it's drawn as an '@' so you can
 * see what the tubes are connecting. The preset decides where they sit.
 *
 * Members
 *   x, y   position in cell coordinates, set by the active preset.
 */
typedef struct { float x, y; } FoodSrc;

/*
 * Crowd — the pool of agents: a heap array plus how many are active.
 * Kept together so helpers take one pointer instead of two values. The
 * array is sized to the actual count (not the 6000 max) and reallocated
 * only when the user changes the count or the scene resets.
 *
 * Members
 *   agents  malloc'd array of `count` agents; freed at exit by do_cleanup.
 *           Non-NULL after scene_init.
 *   count   number of active agents, kept in [N_AGENTS_MIN, N_AGENTS_MAX].
 */
typedef struct {
    Agent *agents;     /* malloc'd, sized to `count`              */
    int    count;      /* active count, tunable with the +/- keys */
} Crowd;

/*
 * SimControls — every knob the user can turn, in one place. The HUD reads
 * these to display, the key handler writes them, and the tick reads them.
 * The three trail knobs (deposit/decay/diffuse) are what reshape the
 * network from fragmented to branching to flooded.
 *
 * Members
 *   preset    starting layout: 0=Scatter, 1=Ring, 2=Clusters, 3=Mesh.
 *   theme     active colour scheme, 0..N_THEMES-1.
 *   sim_fps   physics ticks per second, in [SIM_FPS_MIN, SIM_FPS_MAX].
 *   paused    when true, the tick is skipped and the HUD shows PAUSED.
 *   food_on   when true, food gives the deposit bonus and is drawn.
 *   deposit   trail dropped per agent step (> 0).
 *   decay     fraction of trail lost per tick, in [0.01, 0.30].
 *   diffuse   how strongly trail blurs per tick, in [0.05, 0.90].
 *
 * The key handler clamps each value to the range above on every change.
 */
typedef struct {
    int   preset;
    int   theme;
    int   sim_fps;
    bool  paused;
    bool  food_on;
    float deposit;
    float decay;
    float diffuse;
} SimControls;

/*
 * Screen — the terminal's size in characters. The whole terminal is used
 * except the bottom row, which is reserved for the HUD; so the simulation
 * grid is `cols` wide and `rows - 1` tall. Re-read on every resize.
 *
 * Members
 *   cols   terminal width in characters; grid spans columns [0, cols).
 *   rows   terminal height; grid spans rows [0, rows-1), bottom row is HUD.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

/*
 * Scene — all the state for one run, reachable from a single pointer.
 * (Signal flags live on App instead, since signal handlers can't be
 * handed a pointer.)
 *
 * Members
 *   sim      the user-tunable knobs.
 *   crowd    the agent pool and its count.
 *   food[]   the N_FOOD food sources, placed by the active preset.
 *   trail    pointer to the trail grid (g_trail_field). It's a pointer
 *            because the grid is ~512 KB — too big to copy around by value.
 *   screen   the terminal size.
 */
typedef struct {
    SimControls  sim;
    Crowd        crowd;
    FoodSrc      food[N_FOOD];
    TrailField  *trail;
    Screen       screen;
} Scene;

/*
 * SensorReadings — the three trail strengths an agent reads ahead of it:
 * a little to the left, straight ahead, and a little to the right.
 * Returned as one bundle so sensing and turning pass it cleanly.
 */
typedef struct {
    float front_left;
    float front;
    float front_right;
} SensorReadings;

/* Read the trail at the three points the agent looks at: front-left,
 * straight ahead, and front-right, each SENSOR_DIST cells away. */
static SensorReadings agent_sense_three_sensors(const Agent *a,
                                                const TrailField *trail)
{
    float left_x  = a->x + cosf(a->angle - SENSOR_ANGLE) * SENSOR_DIST;
    float left_y  = a->y + sinf(a->angle - SENSOR_ANGLE) * SENSOR_DIST;
    float fwd_x   = a->x + cosf(a->angle               ) * SENSOR_DIST;
    float fwd_y   = a->y + sinf(a->angle               ) * SENSOR_DIST;
    float right_x = a->x + cosf(a->angle + SENSOR_ANGLE) * SENSOR_DIST;
    float right_y = a->y + sinf(a->angle + SENSOR_ANGLE) * SENSOR_DIST;

    return (SensorReadings){
        .front_left  = trail_sample(trail, left_x,  left_y ),
        .front       = trail_sample(trail, fwd_x,   fwd_y  ),
        .front_right = trail_sample(trail, right_x, right_y),
    };
}

/* Turn the agent toward whichever sensor read the strongest trail. If the
 * centre is as strong as both sides it keeps going straight (this bias
 * toward straight is why tubes form rather than wandering). A dead tie on
 * the two sides picks a side at random. */
static void agent_rotate_toward_brightest(Agent *a, SensorReadings r)
{
    bool centre_wins = (r.front >= r.front_left) && (r.front >= r.front_right);
    if (centre_wins) return;                                /* keep heading */

    if      (r.front_left  > r.front_right) a->angle -= ROTATE_ANGLE;
    else if (r.front_right > r.front_left ) a->angle += ROTATE_ANGLE;
    else {                                  /* both sides equal, both beat centre */
        bool flip_to_right = (rand() & 1) != 0;
        a->angle += flip_to_right ? ROTATE_ANGLE : -ROTATE_ANGLE;
    }
}

/* Step the agent forward one move in its heading, then wrap it back onto
 * the grid if it ran off an edge. Wrapping with a loop (not modulo) keeps
 * the position a float so the fractional part survives for the deposit. */
static void agent_move_and_wrap(Agent *a, const TrailField *trail)
{
    a->x += cosf(a->angle) * STEP_SIZE;
    a->y += sinf(a->angle) * STEP_SIZE;

    float grid_w = (float)trail->active_cols;
    float grid_h = (float)trail->active_rows;
    while (a->x <  0.0f  ) a->x += grid_w;
    while (a->x >= grid_w) a->x -= grid_w;
    while (a->y <  0.0f  ) a->y += grid_h;
    while (a->y >= grid_h) a->y -= grid_h;
}

/* Give a deposit multiplier: FOOD_BONUS if the agent is near any food
 * source, otherwise 1 (and always 1 when food is off). The extra trail
 * near food makes a stronger scent there, which pulls in more agents —
 * that feedback is how the network finds the food in the first place. */
static float agent_food_bonus_multiplier(const Agent *a,
                                          const FoodSrc *food,
                                          bool food_on)
{
    if (!food_on) return 1.0f;

    for (int f = 0; f < N_FOOD; f++) {
        float dx = a->x - food[f].x;
        float dy = a->y - food[f].y;
        float distance_to_food = sqrtf(dx*dx + dy*dy);
        if (distance_to_food < FOOD_RADIUS) return FOOD_BONUS;
    }
    return 1.0f;
}

/* One agent's full turn: sniff ahead, turn toward the strongest scent,
 * step forward, drop trail (more if it's near food). */
static void agent_step(Agent *a, Scene *s)
{
    TrailField *trail = s->trail;

    SensorReadings sensors = agent_sense_three_sensors(a, trail);
    agent_rotate_toward_brightest(a, sensors);
    agent_move_and_wrap(a, trail);

    float bonus  = agent_food_bonus_multiplier(a, s->food, s->sim.food_on);
    float amount = s->sim.deposit * bonus;
    trail_deposit(trail, a->x, a->y, amount);
}

static void agents_step_all(Scene *s)
{
    for (int i = 0; i < s->crowd.count; i++)
        agent_step(&s->crowd.agents[i], s);

    /* Top food cells back up so they never fade out — otherwise the scent
     * agents follow toward food would slowly disappear. */
    if (s->sim.food_on) {
        TrailField *trail = s->trail;
        for (int f = 0; f < N_FOOD; f++) {
            int c = wrap_c_on(trail, (int)(s->food[f].x + 0.5f));
            int r = wrap_r_on(trail, (int)(s->food[f].y + 0.5f));
            if (trail->current[r][c] < FOOD_MIN_TRAIL)
                trail->current[r][c] = FOOD_MIN_TRAIL;
        }
    }
}

static void food_draw(const Scene *s, int rows, int cols)
{
    if (!s->sim.food_on) return;
    for (int f = 0; f < N_FOOD; f++) {
        int c = (int)(s->food[f].x + 0.5f);
        int r = (int)(s->food[f].y + 0.5f);
        if (r >= 0 && r < rows-1 && c >= 0 && c < cols) {
            attron(COLOR_PAIR(CP_FOOD) | A_BOLD);
            mvaddch(r, c, '@');
            attroff(COLOR_PAIR(CP_FOOD) | A_BOLD);
        }
    }
}

/* ── §6 scene — starting layouts and scene setup ── */

static const char *k_preset_names[N_PRESETS] = {
    "Scatter", "Ring", "Clusters", "Mesh"
};

/* Random float helpers: randf gives 0..1, randf_range gives lo..hi. */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }
static float randf_range(float lo, float hi) { return lo + randf() * (hi - lo); }

/* (Re)size the agent array to `count`. Frees the old one first; safe on
 * the first call since free(NULL) does nothing. */
static void crowd_alloc(Crowd *crowd, int count)
{
    free(crowd->agents);
    crowd->agents = malloc((size_t)count * sizeof(Agent));
    crowd->count  = count;
}

static void agent_spawn_random(Agent *a, const TrailField *t)
{
    a->x     = randf() * (float)t->active_cols;
    a->y     = randf() * (float)t->active_rows;
    a->angle = randf() * 2.0f * (float)M_PI;
}

/* Scatter: agents dropped at random; three food sources in a triangle. */
static void preset_scatter(Scene *s)
{
    const TrailField *t = s->trail;
    float cx = (float)t->active_cols * 0.5f;
    float cy = (float)t->active_rows * 0.5f;
    float rx = (float)t->active_cols * 0.30f;
    float ry = (float)t->active_rows * 0.28f;

    /* triangle food sources */
    s->food[0] = (FoodSrc){ cx,            cy - ry       };
    s->food[1] = (FoodSrc){ cx - rx,       cy + ry * 0.6f};
    s->food[2] = (FoodSrc){ cx + rx,       cy + ry * 0.6f};

    for (int i = 0; i < s->crowd.count; i++)
        agent_spawn_random(&s->crowd.agents[i], t);
}

/* Ring: agents on a circle facing inward; food clustered near the centre. */
static void preset_ring(Scene *s)
{
    const TrailField *t = s->trail;
    float cx = (float)t->active_cols * 0.5f;
    float cy = (float)t->active_rows * 0.5f;
    float r  = fminf((float)t->active_cols, (float)t->active_rows * 2.0f) * 0.38f;

    s->food[0] = (FoodSrc){ cx, cy };
    s->food[1] = (FoodSrc){ cx - r * 0.5f, cy };
    s->food[2] = (FoodSrc){ cx + r * 0.5f, cy };

    for (int i = 0; i < s->crowd.count; i++) {
        float angle  = (float)i / (float)s->crowd.count * 2.0f * (float)M_PI;
        float jitter = randf_range(-0.03f, 0.03f) * 2.0f * (float)M_PI;
        s->crowd.agents[i].x = cx + cosf(angle) * r;
        s->crowd.agents[i].y = cy + sinf(angle) * r * 0.5f; /* aspect correction */
        s->crowd.agents[i].angle = angle + (float)M_PI + jitter;  /* inward + jitter */
    }
}

/* Clusters: two dense blobs on the left and right; food spread to the sides. */
static void preset_clusters(Scene *s)
{
    const TrailField *t = s->trail;
    float lx = (float)t->active_cols * 0.22f;
    float rx = (float)t->active_cols * 0.78f;
    float cy = (float)t->active_rows * 0.50f;
    float spread_c = (float)t->active_cols * 0.08f;
    float spread_r = (float)t->active_rows * 0.15f;

    s->food[0] = (FoodSrc){ (float)t->active_cols * 0.05f, cy };
    s->food[1] = (FoodSrc){ (float)t->active_cols * 0.95f, cy };
    s->food[2] = (FoodSrc){ (float)t->active_cols * 0.50f,
                            (float)t->active_rows * 0.20f };

    for (int i = 0; i < s->crowd.count; i++) {
        bool left = (i < s->crowd.count / 2);
        float cx  = left ? lx : rx;
        s->crowd.agents[i].x = cx + randf_range(-spread_c, spread_c);
        s->crowd.agents[i].y = cy + randf_range(-spread_r, spread_r);
        s->crowd.agents[i].angle = randf() * 2.0f * (float)M_PI;
    }
}

/* Mesh: agents laid out on an even grid; food at three corners (the array holds
   three sources, so the network spans a triangle rather than a full rectangle). */
static void preset_mesh(Scene *s)
{
    const TrailField *t = s->trail;
    float mx = (float)t->active_cols * 0.12f;
    float my = (float)t->active_rows * 0.12f;

    s->food[0] = (FoodSrc){ mx,                         my                         };
    s->food[1] = (FoodSrc){ (float)t->active_cols - mx, my                         };
    s->food[2] = (FoodSrc){ (float)t->active_cols - mx, (float)t->active_rows - my };

    int grid_side = (int)sqrtf((float)s->crowd.count);
    int placed = 0;
    for (int gi = 0; gi < grid_side && placed < s->crowd.count; gi++) {
        for (int gj = 0; gj < grid_side && placed < s->crowd.count; gj++, placed++) {
            s->crowd.agents[placed].x = ((float)gj + 0.5f) / (float)grid_side
                                         * (float)t->active_cols;
            s->crowd.agents[placed].y = ((float)gi + 0.5f) / (float)grid_side
                                         * (float)t->active_rows;
            s->crowd.agents[placed].angle = randf() * 2.0f * (float)M_PI;
        }
    }
    for (; placed < s->crowd.count; placed++)
        agent_spawn_random(&s->crowd.agents[placed], t);
}

/* Reset a scene to a fresh start: match the grid to the terminal, size
 * the agent array, wipe the trail and food, then lay out the chosen
 * preset. Leaves the user's knobs and the agent count alone. */
static void scene_init(Scene *s, int preset)
{
    s->sim.preset = preset;

    /* Grid = full terminal minus the bottom HUD row. */
    trail_resize(s->trail, s->screen.rows - 1, s->screen.cols);

    crowd_alloc(&s->crowd, s->crowd.count);
    trail_clear(s->trail);
    memset(s->food, 0, sizeof s->food);

    srand((unsigned)time(NULL));  /* reseed so each reset differs */

    switch (preset) {
        case 0: preset_scatter (s); break;
        case 1: preset_ring    (s); break;
        case 2: preset_clusters(s); break;
        case 3: preset_mesh    (s); break;
    }
}

/* ── §7 screen / HUD — ncurses setup and the status bars ── */

static void screen_init(void)
{
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

/* Draw the two status bars: live readouts along the top, the key list
 * along the bottom. Drawn last each frame so they sit over everything. */
static void hud_draw(const Scene *s, int fps)
{
    /* top row: readouts, right-aligned */
    char status[160];
    snprintf(status, sizeof status,
             " %3d fps  preset:%s  theme:%s  agents:%d  diffuse:%.2f"
             "  decay:%.2f  food:%s%s ",
             fps,
             k_preset_names[s->sim.preset],
             k_themes[s->sim.theme].name,
             s->crowd.count,
             s->sim.diffuse, s->sim.decay,
             s->sim.food_on ? "ON" : "OFF",
             s->sim.paused  ? "  PAUSED" : "");

    int right_col = s->screen.cols - (int)strlen(status);
    if (right_col < 0) right_col = 0;

    move(0, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, right_col, "%s", status);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row: the key list */
    move(s->screen.rows - 1, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->screen.rows - 1, 0,
             " q:quit  spc/p:pause  r:reset  n/N:preset  t/T:theme  "
             "+/-:agents  d/D:diffuse  e/E:decay  f:food  [/]:sim-fps ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §8 app — signals, input, and the main loop ── */

/*
 * FpsCounter — a steady frame-rate readout. A single frame's timing
 * jumps around too much to show, so this tallies frames and elapsed time
 * over a half-second window and only then updates `display`.
 *
 * Members
 *   frame_count   frames counted so far in the current window.
 *   window_ns     nanoseconds counted so far in the current window.
 *   display       the smoothed whole-number fps shown in the HUD.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    int     display;       /* whole fps, shown as " %3d fps " */
} FpsCounter;

static void fps_counter_init(FpsCounter *f)
{
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;     /* 500 ms */
    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;

    /* window full: convert frames-per-window into frames-per-second */
    f->display     = (int)(f->frame_count
                         * (double)NS_PER_SEC / (double)f->window_ns);
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — the whole program's state in one place. The single global g_app
 * exists so the signal handler can reach the quit/resize flags (handlers
 * can't be passed a pointer); everything else is passed around explicitly.
 *
 * The two flags are `volatile sig_atomic_t` because a signal handler may
 * set them at any moment: sig_atomic_t guarantees the write happens in one
 * step (no half-written value), and volatile forces the main loop to
 * re-read them each pass instead of caching a stale copy.
 *
 * Members
 *   scene    the simulation state.
 *   fps      the frame-rate readout.
 *   quit     set by Ctrl-C / kill or the 'q' key; ends the loop.
 *   resize   set by a terminal resize; handled at the top of the next loop.
 */
typedef struct {
    Scene                 scene;
    FpsCounter            fps;
    volatile sig_atomic_t quit;
    volatile sig_atomic_t resize;
} App;

static App g_app;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_app.quit   = 1;
    if (s == SIGWINCH)               g_app.resize = 1;
}

static void do_cleanup(void)
{
    endwin();
    free(g_app.scene.crowd.agents);
}

/* Pin a value into the range [lo, hi]. */
static void clampf(float *v, float lo, float hi)
{
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}
static void clampi(int *v, int lo, int hi)
{
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}

/* Change the agent count by `delta` (clamped), then rebuild the scene so
 * the array is resized and the preset re-runs at the new count. */
static void adjust_agent_count(Scene *s, int delta)
{
    int next = s->crowd.count + delta;
    clampi(&next, N_AGENTS_MIN, N_AGENTS_MAX);
    if (next == s->crowd.count) return;
    s->crowd.count = next;
    scene_init(s, s->sim.preset);
}

/* Step to the next/previous preset (wrapping) and rebuild the scene. */
static void cycle_preset(Scene *s, int dir)
{
    int next = (s->sim.preset + dir + N_PRESETS) % N_PRESETS;
    scene_init(s, next);
}

/* Step to the next/previous colour theme (wrapping) and apply it. */
static void cycle_theme(Scene *s, int dir)
{
    s->sim.theme = (s->sim.theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->sim.theme);
}

/* Limits for the live knobs. Too little blur or fade and the network
 * fragments; too much and it floods into a uniform haze. */
#define DIFFUSE_MIN  0.05f
#define DIFFUSE_MAX  0.90f
#define DIFFUSE_STEP 0.05f
#define DECAY_MIN    0.01f
#define DECAY_MAX    0.30f
#define DECAY_STEP   0.01f

/* Nudge the blur strength up or down one step, clamped. */
static void adjust_diffuse(Scene *s, int dir)
{
    s->sim.diffuse += (float)dir * DIFFUSE_STEP;
    clampf(&s->sim.diffuse, DIFFUSE_MIN, DIFFUSE_MAX);
}

/* Nudge the fade rate up or down one step, clamped. */
static void adjust_decay(Scene *s, int dir)
{
    s->sim.decay += (float)dir * DECAY_STEP;
    clampf(&s->sim.decay, DECAY_MIN, DECAY_MAX);
}

/* Nudge the simulation speed up or down one step, clamped. */
static void adjust_sim_fps(Scene *s, int dir)
{
    s->sim.sim_fps += dir * SIM_FPS_STEP;
    clampi(&s->sim.sim_fps, SIM_FPS_MIN, SIM_FPS_MAX);
}

/* Act on one keypress. Returns false when the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;       break;

    case 'r': case 'R': scene_init(s, s->sim.preset);                   break;

    case 'n':           cycle_preset(s, +1);                            break;
    case 'N':           cycle_preset(s, -1);                            break;

    case 't':           cycle_theme(s, +1);                             break;
    case 'T':           cycle_theme(s, -1);                             break;

    case '+': case '=': adjust_agent_count(s, +N_AGENTS_STEP);          break;
    case '-':           adjust_agent_count(s, -N_AGENTS_STEP);          break;

    case 'd':           adjust_diffuse(s, +1);                          break;
    case 'D':           adjust_diffuse(s, -1);                          break;

    case 'e':           adjust_decay(s, +1);                            break;
    case 'E':           adjust_decay(s, -1);                            break;

    case 'f': case 'F': s->sim.food_on = !s->sim.food_on;               break;

    case ']':           adjust_sim_fps(s, +1);                          break;
    case '[':           adjust_sim_fps(s, -1);                          break;

    default: break;
    }
    return true;
}

/* Handle a terminal resize: restart ncurses so it sees the new size, read
 * the new dimensions, and rebuild the scene to fit. */
static void app_do_resize(App *app)
{
    Scene *s = &app->scene;
    endwin();
    refresh();
    getmaxyx(stdscr, s->screen.rows, s->screen.cols);
    scene_init(s, s->sim.preset);
    app->resize = 0;
}

int main(void)
{
    /* cleanup at exit and catch quit/resize signals */
    atexit(do_cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    App   *app   = &g_app;
    Scene *scene = &app->scene;

    /* starting settings */
    scene->sim.preset    = 0;
    scene->sim.theme     = 0;
    scene->sim.sim_fps   = SIM_FPS_DEF;
    scene->sim.paused    = false;
    scene->sim.food_on   = true;
    scene->sim.deposit   = DEPOSIT_DEF;
    scene->sim.decay     = DECAY_DEF;
    scene->sim.diffuse   = DIFFUSE_DEF;
    scene->crowd.count   = N_AGENTS_DEF;
    scene->trail         = &g_trail_field;

    fps_counter_init(&app->fps);

    screen_init();
    getmaxyx(stdscr, scene->screen.rows, scene->screen.cols);
    scene_init(scene, scene->sim.preset);

    int64_t t_last = clock_ns();

    while (!app->quit) {

        /* a pending resize is handled before anything else */
        if (app->resize) {
            app_do_resize(app);
            t_last = clock_ns();
            continue;
        }

        /* read every key waiting in the buffer */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->quit = 1;
                break;
            }
        }

        /* advance the simulation one step (unless paused) */
        if (!scene->sim.paused) {
            agents_step_all(scene);
            trail_update(scene->trail, scene->sim.diffuse, scene->sim.decay);
        }

        /* draw the frame */
        erase();
        trail_draw(scene->trail, scene->screen.rows, scene->screen.cols);
        food_draw (scene, scene->screen.rows, scene->screen.cols);
        hud_draw  (scene, app->fps.display);
        wnoutrefresh(stdscr);
        doupdate();

        /* Cap the frame rate. The elapsed time is capped at 100 ms so the
         * fps reading doesn't blow up if the process was paused/suspended. */
        const int64_t DT_CAP_NS = 100000000LL;        /* 100 ms */

        int64_t t_now  = clock_ns();
        int64_t t_used = t_now - t_last;
        t_last         = t_now;
        if (t_used > DT_CAP_NS) t_used = DT_CAP_NS;

        int64_t t_sleep = TICK_NS(scene->sim.sim_fps) - (clock_ns() - t_now);
        clock_sleep_ns(t_sleep);

        fps_counter_tick(&app->fps, t_used);
    }
    return 0;
}
