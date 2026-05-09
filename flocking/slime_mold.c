/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * slime_mold.c — Physarum polycephalum (Slime Mold) Simulation
 *
 * Implements the Jeff Jones (2010) agent-based model of Physarum transport
 * networks.  Thousands of agents move on a 2D trail grid, sense the trail
 * ahead with three sensors, steer toward the highest concentration, move
 * one step, deposit trail, then the grid diffuses and decays.  The emergent
 * behaviour — self-organising tubular networks that approximate minimum
 * Steiner trees between food sources — requires no central control.
 *
 * Algorithm (per tick, per agent):
 *   1. SENSE   — sample trail at FL (front-left), F (front), FR (front-right)
 *                sensor positions, each SENSOR_DIST cells ahead at ±SENSOR_ANGLE
 *   2. ROTATE  — if F > FL and F > FR: keep heading
 *                if FL > FR: turn left by ROTATE_ANGLE
 *                if FR > FL: turn right by ROTATE_ANGLE
 *                if FL == FR (and > F): random ±ROTATE_ANGLE
 *   3. MOVE    — advance STEP_SIZE cells in heading direction (wrap screen)
 *   4. DEPOSIT — add DEPOSIT_AMT to trail at current cell
 *
 * Grid update (per tick, applied to all cells simultaneously):
 *   trail_new[r][c] = lerp(trail[r][c], avg_3x3(trail, r, c), DIFFUSE_W)
 *                     × (1 − DECAY_RATE)
 *
 * Food sources: 3 fixed positions (configurable per preset).
 *   Agents within FOOD_RADIUS cells deposit FOOD_BONUS × DEPOSIT_AMT.
 *   Food sources are displayed as bright '@' markers; the network of tubes
 *   connecting them is the "optimal Steiner tree" approximation.
 *
 * Presets:
 *   0  Scatter   — agents randomised; 3 food sources in a triangle
 *   1  Ring      — agents on a ring pointing inward; food at centre
 *   2  Clusters  — two dense clusters; food at opposite edges
 *   3  Mesh      — agents on a grid; 4 corner food sources
 *
 * Keys:
 *   q / ESC    quit
 *   p / space  pause / resume
 *   r          reset current preset
 *   n / N      next / previous preset
 *   t / T      next / previous theme
 *   + / -      more / fewer agents
 *   d / D      more / less diffusion
 *   e / E      faster / slower decay
 *   f          toggle food sources
 *   ] / [      sim FPS up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/slime_mold.c \
 *       -o slime_mold -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 trail grid  §5 agents
 *           §6 scene   §7 screen §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Agent-based simulation — emergent behaviour from local rules.
 *                  No global coordinator; network topology arises purely from
 *                  the sense→rotate→move→deposit loop repeated N_AGENTS times
 *                  per tick. The trail grid mediates indirect communication
 *                  (stigmergy): agents respond to paths left by earlier agents.
 *
 * Biology        : Models Physarum polycephalum (Jeff Jones, 2010).
 *                  Real slime mold spans food sources with tubular networks that
 *                  approximate minimum Steiner trees — surprisingly close to
 *                  optimal transport graphs.  The model captures this using only
 *                  three sensor readings and a random tie-break rule.
 *
 * Math           : Diffusion step is a lerp toward a 3×3 box average:
 *                    trail' = lerp(trail, avg_3×3(trail), DIFFUSE_W)
 *                  This is a discrete approximation of the heat equation
 *                  (∂u/∂t = D·∇²u).  DIFFUSE_W controls the effective
 *                  diffusion coefficient D.
 *                  Decay: trail' *= (1 − DECAY_RATE) per tick — exponential
 *                  fade without diffusion would give trail lifetime ≈ 1/DECAY.
 *
 * Performance    : O(N_AGENTS + W×H) per tick.  The trail grid update is the
 *                  bottleneck: 512×128 ≈ 65K cells × 9-neighbour sum per cell
 *                  ≈ 590K ops per tick.  Agents cost N_AGENTS × ~15 ops each.
 *
 * Data-structure : Two float arrays (g_trail / g_buf) for double-buffering.
 *                  Agents read and deposit into g_trail; the grid update reads
 *                  g_trail → writes g_buf → copies back.  This prevents mid-tick
 *                  positional feedback (a grain reading its own fresh deposit).
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define ROWS_MAX        128
#define COLS_MAX        512

/* Agent population */
#define N_AGENTS_DEF   2000  /* default agent count; 2000 fills ~200×60 grid visually */
#define N_AGENTS_MIN    200  /* below this, tubes fail to form (too sparse)            */
#define N_AGENTS_MAX   6000  /* above this, cost > 30fps on typical hardware           */
#define N_AGENTS_STEP   200

/* Physarum sensor parameters (Jones 2010) */
#define SENSOR_ANGLE  ((float)M_PI / 4.0f)   /* ±45° from heading; Jones found 45°
                                               * gives crisp tubes without over-steering */
#define SENSOR_DIST    4.0f                   /* cells ahead; smaller→tighter curves,
                                               * larger→straighter long-range tubes      */
#define ROTATE_ANGLE  ((float)M_PI / 4.0f)   /* 45° abrupt turn per step — discrete
                                               * steering; fractional angles give blurry
                                               * diffuse blobs instead of tubes          */
#define STEP_SIZE      1.0f                   /* cells moved per tick (1 = one cell)    */

/* Trail parameters */
#define DEPOSIT_DEF    5.0f    /* trail concentration added per agent tick; scales
                                * with FOOD_BONUS near food sources                    */
#define MAX_TRAIL     100.0f   /* saturation ceiling; prevents overflow and keeps
                                * concentration mapping in [0,100] for display         */
#define DECAY_DEF      0.08f   /* 8% removed per tick → trail lifetime ≈ 1/0.08 = 12.5
                                * ticks at 30fps ≈ 0.4s half-life without diffusion    */
#define DIFFUSE_DEF    0.35f   /* lerp weight toward 3×3 average; higher→smoother but
                                * more blurry network (tubes lose sharp boundaries)    */

/* Food sources */
#define N_FOOD          3
#define FOOD_RADIUS     3.0f   /* detection radius in cells (about 3 character widths) */
#define FOOD_BONUS      6.0f   /* ×6 deposit near food → strong attractor gradient     */
#define FOOD_MIN_TRAIL 30.0f   /* floor concentration at food cells; prevents food
                                * sites from fading after most agents move away        */

/* Simulation */
#define SIM_FPS_DEF    30
#define SIM_FPS_MIN     5
#define SIM_FPS_MAX    60
#define SIM_FPS_STEP    5

#define N_PRESETS       4
#define N_THEMES        5

#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * 6 trail intensity color pairs (1=dimmest .. 5=brightest) + 6=food + 7=HUD
 * The trail is rendered with 5 characters × 5 colors.
 */
enum {
    CP_T1 = 1,  /* dimmest trail  */
    CP_T2 = 2,
    CP_T3 = 3,
    CP_T4 = 4,
    CP_T5 = 5,  /* brightest trail */
    CP_FOOD = 6,
    CP_HUD  = 7,
};

typedef struct {
    short t[5];       /* trail pair fg colors, dim→bright (256-color)    */
    short t8[5];      /* 8-color fallbacks                                */
    short food, hud;
    short food8, hud8;
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 Physarum — yellow/amber network on black, like real slime mold */
    { {22, 58, 100, 136, 220},
      {COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
      196, 244,   COLOR_RED, COLOR_WHITE,   "Physarum" },
    /* 1 Cyan — cold network */
    { {17, 19, 27, 39, 87},
      {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
      226, 244,   COLOR_YELLOW, COLOR_WHITE,   "Cyan" },
    /* 2 Neon — magenta/violet */
    { {53, 91, 129, 165, 207},
      {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE},
      226, 244,   COLOR_YELLOW, COLOR_WHITE,   "Neon" },
    /* 3 Forest — organic green */
    { {22, 28, 34, 40, 118},
      {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE},
      196, 244,   COLOR_RED, COLOR_WHITE,   "Forest" },
    /* 4 Lava — red/orange */
    { {52, 88, 124, 166, 226},
      {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
      231, 244,   COLOR_WHITE, COLOR_WHITE,   "Lava" },
};

static int g_theme = 0;

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    for (int i = 0; i < 5; i++) {
        if (COLORS >= 256)
            init_pair(i + 1, th->t[i],  -1);
        else
            init_pair(i + 1, th->t8[i], -1);
    }
    if (COLORS >= 256) {
        init_pair(CP_FOOD, th->food,  -1);
        init_pair(CP_HUD,  th->hud,   -1);
    } else {
        init_pair(CP_FOOD, th->food8, -1);
        init_pair(CP_HUD,  th->hud8,  -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(0);
}

/* ===================================================================== */
/* §4  trail grid                                                         */
/* ===================================================================== */

/*
 * Two float arrays: g_trail (current) and g_buf (diffusion workspace).
 * After diffusion+decay writes into g_buf, we swap the pointers.
 *
 * Agents sample and deposit into g_trail.
 * The grid update (diffuse + decay) reads g_trail → writes g_buf, then swap.
 */
static float g_trail[ROWS_MAX][COLS_MAX];
static float g_buf  [ROWS_MAX][COLS_MAX];
static int   g_grid_rows, g_grid_cols;  /* active grid size = terminal size */

static inline int wrap_r(int r) {
    return (r % g_grid_rows + g_grid_rows) % g_grid_rows;
}
static inline int wrap_c(int c) {
    return (c % g_grid_cols + g_grid_cols) % g_grid_cols;
}

/* Sample trail at float position (wrapping) */
static float trail_sample(float fc, float fr)
{
    int c = wrap_c((int)(fc + 0.5f));
    int r = wrap_r((int)(fr + 0.5f));
    return g_trail[r][c];
}

/* Deposit to trail at float position */
static void trail_deposit(float fc, float fr, float amount)
{
    int c = wrap_c((int)(fc + 0.5f));
    int r = wrap_r((int)(fr + 0.5f));
    g_trail[r][c] += amount;
    if (g_trail[r][c] > MAX_TRAIL) g_trail[r][c] = MAX_TRAIL;
}

/* Diffuse + decay: reads g_trail, writes g_buf, then swap */
static void trail_update(float diffuse_w, float decay)
{
    float retain = 1.0f - decay;
    int R = g_grid_rows, C = g_grid_cols;

    for (int r = 0; r < R; r++) {
        for (int c = 0; c < C; c++) {
            /* 3×3 box average (wrap) */
            float sum = 0.0f;
            for (int dr = -1; dr <= 1; dr++)
                for (int dc = -1; dc <= 1; dc++)
                    sum += g_trail[wrap_r(r+dr)][wrap_c(c+dc)];
            float avg = sum / 9.0f;

            /* lerp toward neighbour average, then decay */
            float v = g_trail[r][c] * (1.0f - diffuse_w) + avg * diffuse_w;
            g_buf[r][c] = v * retain;
            if (g_buf[r][c] < 0.0f) g_buf[r][c] = 0.0f;
        }
    }

    /* swap: copy buf → trail */
    memcpy(g_trail, g_buf, sizeof g_trail);
}

static void trail_clear(void)
{
    memset(g_trail, 0, sizeof g_trail);
    memset(g_buf,   0, sizeof g_buf);
}

/*
 * trail_char_pair — map concentration to (character, color_pair).
 *
 * Five intensity levels with distinct characters so the network structure
 * is legible even on 8-color terminals:
 *   0.3–2   '.'  dim      (faint trace)
 *   2–8     '+'  medium   (moderate concentration)
 *   8–20    'x'  high
 *  20–50    '#'  dense    (major tube)
 *  50+      '@'  saturated (hub / intersection)
 */
static void trail_char_pair(float v, chtype *ch_out, int *cp_out)
{
    if      (v < 0.3f)  { *ch_out = ' '; *cp_out = CP_T1; }
    else if (v < 2.0f)  { *ch_out = '.'; *cp_out = CP_T1; }
    else if (v < 8.0f)  { *ch_out = '+'; *cp_out = CP_T2; }
    else if (v < 20.0f) { *ch_out = 'x'; *cp_out = CP_T3; }
    else if (v < 50.0f) { *ch_out = '#'; *cp_out = CP_T4; }
    else                { *ch_out = '@'; *cp_out = CP_T5; }
}

static void trail_draw(int rows, int cols)
{
    for (int r = 0; r < rows && r < g_grid_rows; r++) {
        for (int c = 0; c < cols && c < g_grid_cols; c++) {
            chtype ch; int cp;
            trail_char_pair(g_trail[r][c], &ch, &cp);
            if (ch == ' ') continue;  /* skip background cells */
            attron(COLOR_PAIR(cp));
            mvaddch(r, c, ch);
            attroff(COLOR_PAIR(cp));
        }
    }
}

/* ===================================================================== */
/* §5  agents                                                             */
/* ===================================================================== */

typedef struct {
    float x, y;     /* position in cell space (float for sub-cell precision) */
    float angle;    /* heading in radians */
} Agent;

/* Food source positions */
typedef struct { float x, y; } FoodSrc;

static Agent   *g_agents   = NULL;
static int      g_n_agents = N_AGENTS_DEF;
static FoodSrc  g_food[N_FOOD];
static bool     g_food_on  = true;

static float g_deposit  = DEPOSIT_DEF;
static float g_decay    = DECAY_DEF;
static float g_diffuse  = DIFFUSE_DEF;

/*
 * agent_step — one Physarum agent tick (sense → rotate → move → deposit).
 *
 * Sensor positions are sampled in float space (wrap at grid boundary).
 * Rotation amounts to ±ROTATE_ANGLE per tick — turning is abrupt, not gradual.
 * This produces the crisp tube morphology characteristic of Physarum models.
 */
static void agent_step(Agent *a)
{
    float ca = cosf(a->angle), sa = sinf(a->angle);

    /* sensor positions */
    float fl_x = a->x + cosf(a->angle - SENSOR_ANGLE) * SENSOR_DIST;
    float fl_y = a->y + sinf(a->angle - SENSOR_ANGLE) * SENSOR_DIST;
    float  f_x = a->x + ca * SENSOR_DIST;
    float  f_y = a->y + sa * SENSOR_DIST;
    float fr_x = a->x + cosf(a->angle + SENSOR_ANGLE) * SENSOR_DIST;
    float fr_y = a->y + sinf(a->angle + SENSOR_ANGLE) * SENSOR_DIST;

    float vFL = trail_sample(fl_x, fl_y);
    float vF  = trail_sample( f_x,  f_y);
    float vFR = trail_sample(fr_x, fr_y);

    /* rotate */
    if (vF >= vFL && vF >= vFR) {
        /* keep heading */
    } else if (vFL > vFR) {
        a->angle -= ROTATE_ANGLE;
    } else if (vFR > vFL) {
        a->angle += ROTATE_ANGLE;
    } else {
        /* equal sides, both > front: random jitter */
        a->angle += (rand() & 1) ? ROTATE_ANGLE : -ROTATE_ANGLE;
    }

    /* move */
    a->x += cosf(a->angle) * STEP_SIZE;
    a->y += sinf(a->angle) * STEP_SIZE;

    /* wrap */
    float R = (float)g_grid_rows, C = (float)g_grid_cols;
    while (a->x < 0.0f) a->x += C;
    while (a->x >= C)   a->x -= C;
    while (a->y < 0.0f) a->y += R;
    while (a->y >= R)   a->y -= R;

    /* deposit — more if near a food source */
    float dep = g_deposit;
    if (g_food_on) {
        for (int f = 0; f < N_FOOD; f++) {
            float dx = a->x - g_food[f].x;
            float dy = a->y - g_food[f].y;
            if (sqrtf(dx*dx + dy*dy) < FOOD_RADIUS) {
                dep *= FOOD_BONUS;
                break;
            }
        }
    }
    trail_deposit(a->x, a->y, dep);
}

static void agents_step_all(void)
{
    for (int i = 0; i < g_n_agents; i++)
        agent_step(&g_agents[i]);

    /* keep food source cells above their floor */
    if (g_food_on) {
        for (int f = 0; f < N_FOOD; f++) {
            int c = wrap_c((int)(g_food[f].x + 0.5f));
            int r = wrap_r((int)(g_food[f].y + 0.5f));
            if (g_trail[r][c] < FOOD_MIN_TRAIL)
                g_trail[r][c] = FOOD_MIN_TRAIL;
        }
    }
}

static void food_draw(int rows, int cols)
{
    if (!g_food_on) return;
    for (int f = 0; f < N_FOOD; f++) {
        int c = (int)(g_food[f].x + 0.5f);
        int r = (int)(g_food[f].y + 0.5f);
        if (r >= 0 && r < rows-1 && c >= 0 && c < cols) {
            attron(COLOR_PAIR(CP_FOOD) | A_BOLD);
            mvaddch(r, c, '@');
            attroff(COLOR_PAIR(CP_FOOD) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §6  scene — presets & lifecycle                                        */
/* ===================================================================== */

static int g_preset  = 0;
static int g_sim_fps = SIM_FPS_DEF;
static bool g_paused = false;
static int g_rows, g_cols;

static const char *k_preset_names[N_PRESETS] = {
    "Scatter", "Ring", "Clusters", "Mesh"
};

/* Spawn helpers */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }
static float randf_range(float lo, float hi) { return lo + randf() * (hi - lo); }

static void agents_alloc(int n)
{
    free(g_agents);
    g_agents   = malloc((size_t)n * sizeof(Agent));
    g_n_agents = n;
}

static void agent_spawn_random(Agent *a)
{
    a->x     = randf() * (float)g_grid_cols;
    a->y     = randf() * (float)g_grid_rows;
    a->angle = randf() * 2.0f * (float)M_PI;
}

/* ── Preset 0: Scatter ────────────────────────────────────────────── */
/* Random positions + 3 food sources in a triangle */
static void preset_scatter(void)
{
    float cx = (float)g_grid_cols * 0.5f;
    float cy = (float)g_grid_rows * 0.5f;
    float rx = (float)g_grid_cols * 0.30f;
    float ry = (float)g_grid_rows * 0.28f;

    /* triangle food sources */
    g_food[0] = (FoodSrc){ cx,            cy - ry       };
    g_food[1] = (FoodSrc){ cx - rx,       cy + ry * 0.6f};
    g_food[2] = (FoodSrc){ cx + rx,       cy + ry * 0.6f};

    for (int i = 0; i < g_n_agents; i++)
        agent_spawn_random(&g_agents[i]);
}

/* ── Preset 1: Ring ───────────────────────────────────────────────── */
/* Agents on a circle pointing inward; single food source at centre */
static void preset_ring(void)
{
    float cx = (float)g_grid_cols * 0.5f;
    float cy = (float)g_grid_rows * 0.5f;
    float r  = fminf((float)g_grid_cols, (float)g_grid_rows * 2.0f) * 0.38f;

    g_food[0] = (FoodSrc){ cx, cy };
    g_food[1] = (FoodSrc){ cx - r * 0.5f, cy };
    g_food[2] = (FoodSrc){ cx + r * 0.5f, cy };

    for (int i = 0; i < g_n_agents; i++) {
        float angle = (float)i / (float)g_n_agents * 2.0f * (float)M_PI;
        float jitter = randf_range(-0.03f, 0.03f) * 2.0f * (float)M_PI;
        g_agents[i].x = cx + cosf(angle) * r;
        g_agents[i].y = cy + sinf(angle) * r * 0.5f; /* aspect correction */
        /* point inward + jitter */
        g_agents[i].angle = angle + (float)M_PI + jitter;
    }
}

/* ── Preset 2: Clusters ───────────────────────────────────────────── */
/* Two dense clusters on left and right; food at screen corners */
static void preset_clusters(void)
{
    float lx = (float)g_grid_cols * 0.22f;
    float rx = (float)g_grid_cols * 0.78f;
    float cy = (float)g_grid_rows * 0.50f;
    float spread_c = (float)g_grid_cols * 0.08f;
    float spread_r = (float)g_grid_rows * 0.15f;

    g_food[0] = (FoodSrc){ (float)g_grid_cols * 0.05f, cy };
    g_food[1] = (FoodSrc){ (float)g_grid_cols * 0.95f, cy };
    g_food[2] = (FoodSrc){ (float)g_grid_cols * 0.50f,
                           (float)g_grid_rows * 0.20f };

    for (int i = 0; i < g_n_agents; i++) {
        bool left = (i < g_n_agents / 2);
        float cx  = left ? lx : rx;
        g_agents[i].x = cx + randf_range(-spread_c, spread_c);
        g_agents[i].y = cy + randf_range(-spread_r, spread_r);
        g_agents[i].angle = randf() * 2.0f * (float)M_PI;
    }
}

/* ── Preset 3: Mesh ───────────────────────────────────────────────── */
/* Agents on a regular grid; food at 4 corners */
static void preset_mesh(void)
{
    float mx = (float)g_grid_cols * 0.12f;
    float my = (float)g_grid_rows * 0.12f;

    g_food[0] = (FoodSrc){ mx,                              my                             };
    g_food[1] = (FoodSrc){ (float)g_grid_cols - mx,         my                             };
    g_food[2] = (FoodSrc){ mx,                              (float)g_grid_rows - my        };
    g_food[3 % N_FOOD] = (FoodSrc){ (float)g_grid_cols - mx, (float)g_grid_rows - my };

    int grid_side = (int)sqrtf((float)g_n_agents);
    int placed = 0;
    for (int gi = 0; gi < grid_side && placed < g_n_agents; gi++) {
        for (int gj = 0; gj < grid_side && placed < g_n_agents; gj++, placed++) {
            g_agents[placed].x = ((float)gj + 0.5f) / (float)grid_side
                                  * (float)g_grid_cols;
            g_agents[placed].y = ((float)gi + 0.5f) / (float)grid_side
                                  * (float)g_grid_rows;
            g_agents[placed].angle = randf() * 2.0f * (float)M_PI;
        }
    }
    for (; placed < g_n_agents; placed++)
        agent_spawn_random(&g_agents[placed]);
}

static void scene_init(int preset)
{
    g_preset = preset;
    g_grid_rows = g_rows - 1;   /* bottom row reserved for HUD */
    g_grid_cols = g_cols;

    agents_alloc(g_n_agents);
    trail_clear();

    /* seed RNG with time */
    srand((unsigned)time(NULL));

    /* zero food sources before preset fills them */
    memset(g_food, 0, sizeof g_food);

    switch (preset) {
    case 0: preset_scatter (); break;
    case 1: preset_ring    (); break;
    case 2: preset_clusters(); break;
    case 3: preset_mesh    (); break;
    }
}

/* ===================================================================== */
/* §7  screen / HUD                                                       */
/* ===================================================================== */

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

static void hud_draw(int fps)
{
    move(g_rows - 1, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HUD));
    printw(" SlimeMold  q:quit  n:%s  t:%s  +/-:agents(%d)"
           "  d/D:diffuse(%.2f)  e/E:decay(%.2f)  f:food(%s)"
           "  p:pause  %dfps",
           k_preset_names[g_preset],
           k_themes[g_theme].name,
           g_n_agents,
           g_diffuse, g_decay,
           g_food_on ? "ON" : "OFF",
           fps);
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit_flag   = 0;
static volatile sig_atomic_t g_resize_flag = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit_flag   = 1;
    if (s == SIGWINCH)               g_resize_flag = 1;
}
static void do_cleanup(void) { endwin(); free(g_agents); }

int main(void)
{
    atexit(do_cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    screen_init();
    getmaxyx(stdscr, g_rows, g_cols);
    scene_init(0);

    int64_t t_last = clock_ns();

    int64_t fps_acc  = 0;
    int     fps_cnt  = 0;
    int     fps_disp = 0;

    while (!g_quit_flag) {

        /* ── resize ── */
        if (g_resize_flag) {
            g_resize_flag = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            scene_init(g_preset);
            t_last = clock_ns();
            continue;
        }

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_quit_flag = 1; break;

            case ' ': case 'p': case 'P':
                g_paused = !g_paused; break;

            case 'r': case 'R':
                scene_init(g_preset); break;

            case 'n':
                scene_init((g_preset + 1) % N_PRESETS); break;
            case 'N':
                scene_init((g_preset + N_PRESETS - 1) % N_PRESETS); break;

            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme); break;
            case 'T':
                g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_theme); break;

            case '+': case '=':
                if (g_n_agents < N_AGENTS_MAX) {
                    g_n_agents += N_AGENTS_STEP;
                    scene_init(g_preset);
                }
                break;
            case '-':
                if (g_n_agents > N_AGENTS_MIN) {
                    g_n_agents -= N_AGENTS_STEP;
                    scene_init(g_preset);
                }
                break;

            case 'd':
                if (g_diffuse < 0.90f) g_diffuse += 0.05f;
                break;
            case 'D':
                if (g_diffuse > 0.05f) g_diffuse -= 0.05f;
                break;

            case 'e':
                if (g_decay < 0.30f) g_decay += 0.01f;
                break;
            case 'E':
                if (g_decay > 0.01f) g_decay -= 0.01f;
                break;

            case 'f': case 'F':
                g_food_on = !g_food_on;
                break;

            case ']':
                if (g_sim_fps < SIM_FPS_MAX) g_sim_fps += SIM_FPS_STEP;
                break;
            case '[':
                if (g_sim_fps > SIM_FPS_MIN) g_sim_fps -= SIM_FPS_STEP;
                break;
            }
        }

        /* ── tick ── */
        if (!g_paused) {
            agents_step_all();
            trail_update(g_diffuse, g_decay);
        }

        /* ── draw ── */
        erase();
        trail_draw(g_rows, g_cols);
        food_draw (g_rows, g_cols);
        hud_draw(fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── FPS cap ── */
        int64_t t_now  = clock_ns();
        int64_t t_used = t_now - t_last;
        t_last = t_now;
        if (t_used > 100000000LL) t_used = 100000000LL;

        int64_t t_sleep = TICK_NS(g_sim_fps) - (clock_ns() - t_now);
        clock_sleep_ns(t_sleep);

        /* ── FPS counter ── */
        fps_acc += t_used;
        fps_cnt++;
        if (fps_acc >= NS_PER_SEC / 2) {
            fps_disp = fps_cnt * 2;
            fps_acc  = 0;
            fps_cnt  = 0;
        }
    }
    return 0;
}
