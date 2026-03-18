/*
 * flowfield.c  —  ncurses ASCII flow field
 *
 * A 2-D vector field driven by layered Perlin noise evolves over time.
 * Hundreds of particles follow the field, leaving short fading trails.
 * The result looks like wind, smoke, or ocean currents rendered in ASCII.
 *
 * Visual design:
 *   Direction arrows → the flow vector at each cell
 *   Particle trails  → streaks of '.' ',' '+' '*' that fade as they age
 *   Color            → hue rotates with the vector angle (rainbow wind)
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         respawn all particles
 *   t         cycle color theme (rainbow / mono-cyan / mono-green / mono-white)
 *   ]  [      simulation faster / slower
 *   +  -      more / fewer particles
 *   f  F      field evolve speed faster / slower
 *   s  S      increase / decrease trail length
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flowfield.c -o flowfield -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config    — every tunable constant
 *   §2  clock     — monotonic ns clock + sleep
 *   §3  color     — direction-to-color mapping; themes
 *   §4  noise     — layered Perlin noise (2-D + time)
 *   §5  field     — vector field grid; evolve; sample
 *   §6  particle  — one tracer particle; spawn; tick; trail
 *   §7  scene     — particle pool + field + tick + draw
 *   §8  screen    — double WINDOW buffer + HUD overlay
 *   §9  app       — dt loop, input, resize, cleanup
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  5,
    SIM_FPS_DEFAULT  = 30,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    HUD_COLS         = 52,
    FPS_UPDATE_MS    = 500,

    /* Particle pool */
    PARTICLES_MIN    =  50,
    PARTICLES_DEFAULT= 300,
    PARTICLES_MAX    = 800,
    PARTICLES_STEP   =  50,

    /* Trail */
    TRAIL_LEN_MIN    =  3,
    TRAIL_LEN_DEFAULT= 14,
    TRAIL_LEN_MAX    = 20,

    /* Theme count */
    THEME_COUNT      =  4,
};

/* Field evolution speed (noise time axis advances by this per tick) */
#define FIELD_SPD_DEFAULT  0.008f
#define FIELD_SPD_MIN      0.001f
#define FIELD_SPD_MAX      0.08f
#define FIELD_SPD_STEP     1.4f

/* Particle speed range — base ± jitter gives each particle a unique pace */
#define PARTICLE_SPEED     0.9f
#define PARTICLE_SPD_JITTER 0.4f  /* each particle gets speed in [0.5, 1.3]  */

/* Noise scale — larger = slower spatial variation in the field */
#define NOISE_SCALE_X      0.04f
#define NOISE_SCALE_Y      0.07f

/* Particle lifetime jitter: lives LIFE_BASE ± LIFE_JITTER ticks */
#define LIFE_BASE          100
#define LIFE_JITTER         60

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Four themes controlled by `scene.theme`:
 *   0  RAINBOW   — hue rotates 0→360° with flow angle
 *   1  CYAN      — all particles in bright/mid/dim cyan
 *   2  GREEN     — all particles in bright/mid/dim green
 *   3  WHITE     — grey ramp (white → dark grey)
 *
 * We use 8 color pairs (1–8).
 * In RAINBOW mode each pair corresponds to a hue octant (45° slice).
 * In mono modes all 8 pairs are shades of one hue for the trail fade.
 *
 * Pair index 1 = brightest (fresh particle head)
 * Pair index 8 = dimmest   (oldest trail cell)
 */
enum { N_PAIRS = 8 };

static const char * const k_theme_names[THEME_COUNT] = {
    "rainbow", "cyan", "green", "white"
};

static void color_apply_theme(int theme)
{
    if (COLORS >= 256) {
        if (theme == 0) {   /* RAINBOW — 8 hue steps around the wheel */
            static const int hues[N_PAIRS] = {196, 208, 226, 46, 51, 21, 129, 201};
            for (int i = 0; i < N_PAIRS; i++)
                init_pair(i + 1, hues[i], COLOR_BLACK);
        } else if (theme == 1) {   /* CYAN fade */
            static const int c[N_PAIRS] = {51, 45, 39, 33, 27, 21, 19, 17};
            for (int i = 0; i < N_PAIRS; i++)
                init_pair(i + 1, c[i], COLOR_BLACK);
        } else if (theme == 2) {   /* GREEN fade */
            static const int g[N_PAIRS] = {82, 46, 40, 34, 28, 22, 22, 22};
            for (int i = 0; i < N_PAIRS; i++)
                init_pair(i + 1, g[i], COLOR_BLACK);
        } else {                    /* WHITE/GREY fade */
            static const int w[N_PAIRS] = {255, 250, 244, 238, 235, 234, 233, 232};
            for (int i = 0; i < N_PAIRS; i++)
                init_pair(i + 1, w[i], COLOR_BLACK);
        }
    } else {
        /* 8-color fallback — cycle through available colors */
        static const int fb[THEME_COUNT][N_PAIRS] = {
            {COLOR_RED,COLOR_YELLOW,COLOR_GREEN,COLOR_CYAN,
             COLOR_BLUE,COLOR_MAGENTA,COLOR_WHITE,COLOR_WHITE},
            {COLOR_CYAN,COLOR_CYAN,COLOR_CYAN,COLOR_BLUE,
             COLOR_BLUE,COLOR_BLUE,COLOR_BLUE,COLOR_BLUE},
            {COLOR_GREEN,COLOR_GREEN,COLOR_GREEN,COLOR_GREEN,
             COLOR_GREEN,COLOR_GREEN,COLOR_GREEN,COLOR_GREEN},
            {COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,
             COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,COLOR_WHITE},
        };
        for (int i = 0; i < N_PAIRS; i++)
            init_pair(i + 1, fb[theme][i], COLOR_BLACK);
    }
}

static void color_init(int theme)
{
    start_color();
    color_apply_theme(theme);
}

/*
 * angle_to_pair() — map a flow angle [0, 2π) to a color pair (1–8).
 * Used only in RAINBOW theme; in mono themes a fixed pair per trail age
 * is used instead.
 */
static int angle_to_pair(float angle)
{
    float a = angle;
    if (a < 0) a += 2.0f * (float)M_PI;
    int p = (int)(a / (2.0f * (float)M_PI) * N_PAIRS) % N_PAIRS;
    return p + 1;
}

/* ===================================================================== */
/* §4  noise                                                              */
/* ===================================================================== */

/*
 * Layered value noise — simple, fast, and sufficient for a flow field.
 *
 * We use a 256-element permutation table (Ken Perlin's classic approach)
 * to hash integer grid coordinates to random gradient values, then
 * interpolate with smoothstep.
 *
 * Three octaves are summed with decreasing amplitude (1, 0.5, 0.25)
 * and increasing frequency (1×, 2×, 4×).  This gives the organic
 * swirling appearance of real vector fields.
 */
static uint8_t perm[512];

static void noise_init(void)
{
    uint8_t p[256];
    for (int i = 0; i < 256; i++) p[i] = (uint8_t)i;
    /* Fisher-Yates shuffle */
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = p[i]; p[i] = p[j]; p[j] = t;
    }
    for (int i = 0; i < 512; i++) perm[i] = p[i & 255];
}

/* Smoothstep: 3t² - 2t³ */
static inline float fade(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

static inline float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

/* Hash gradient: maps integer grid point to a pseudo-random float [-1,1] */
static inline float grad(int h, float x, float y)
{
    int H = h & 3;
    float u = (H < 2) ? x : y;
    float v = (H < 2) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}

/* Single-octave 2-D Perlin noise in [-1, 1] */
static float noise2(float x, float y)
{
    int   xi = (int)floorf(x) & 255;
    int   yi = (int)floorf(y) & 255;
    float xf = x - floorf(x);
    float yf = y - floorf(y);
    float u  = fade(xf);
    float v  = fade(yf);

    int aa = perm[perm[xi  ] + yi  ];
    int ab = perm[perm[xi  ] + yi+1];
    int ba = perm[perm[xi+1] + yi  ];
    int bb = perm[perm[xi+1] + yi+1];

    return lerp(
        lerp(grad(aa, xf,   yf  ), grad(ba, xf-1, yf  ), u),
        lerp(grad(ab, xf,   yf-1), grad(bb, xf-1, yf-1), u),
        v
    );
}

/*
 * noise_field_angle() — return the flow angle at (x, y, t) in [0, 2π).
 *
 * Two independent noise samples (offset spatially) give the X and Y
 * components of the flow vector.  atan2 converts to an angle.
 * Three octaves give the organic multi-scale swirling look.
 */
static float noise_field_angle(float x, float y, float t)
{
    float ox = 100.3f;   /* offset so the two samples are independent   */
    float oy = 200.7f;

    float nx = 0.0f, ny = 0.0f;
    float amp = 1.0f, freq = 1.0f;
    for (int oct = 0; oct < 3; oct++) {
        nx += noise2(x * NOISE_SCALE_X * freq + t,
                     y * NOISE_SCALE_Y * freq + t * 0.7f) * amp;
        ny += noise2(x * NOISE_SCALE_X * freq + ox + t * 1.1f,
                     y * NOISE_SCALE_Y * freq + oy + t * 0.5f) * amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return atan2f(ny, nx);   /* result in (-π, π) — used mod 2π elsewhere */
}

/* ===================================================================== */
/* §5  field                                                              */
/* ===================================================================== */

/*
 * Field — the 2-D vector field.
 *
 * Stores one angle per cell (the flow direction at that cell).
 * Recomputed from noise each tick as `time` advances.
 *
 * Keeping the field as a flat angle array (not vector pairs) saves
 * memory and makes sampling O(1): look up `angles[y*cols+x]`.
 *
 * The field also provides the ASCII arrow character that best represents
 * each angle — used to draw the background field visualization.
 */
typedef struct {
    float *angles;   /* [rows * cols] flow angle in radians              */
    int    cols;
    int    rows;
    float  time;     /* current noise time axis value                    */
    float  speed;    /* time advance per tick                            */
} Field;

static void field_alloc(Field *f, int cols, int rows)
{
    f->cols   = cols;
    f->rows   = rows;
    f->angles = calloc((size_t)(cols * rows), sizeof(float));
    f->time   = 0.0f;
    f->speed  = FIELD_SPD_DEFAULT;
}

static void field_free(Field *f)
{
    free(f->angles);
    *f = (Field){0};
}

static void field_resize(Field *f, int cols, int rows)
{
    free(f->angles);
    f->cols   = cols;
    f->rows   = rows;
    f->angles = calloc((size_t)(cols * rows), sizeof(float));
}

/*
 * field_tick() — advance time and recompute all angles from noise.
 * Called once per simulation tick.
 */
static void field_tick(Field *f)
{
    f->time += f->speed;
    for (int y = 0; y < f->rows; y++)
        for (int x = 0; x < f->cols; x++)
            f->angles[y * f->cols + x] =
                noise_field_angle((float)x, (float)y, f->time);
}

/*
 * field_sample() — return the flow angle at continuous position (px, py).
 * Bilinear interpolation between the four surrounding grid cells.
 */
static float field_sample(const Field *f, float px, float py)
{
    int x0 = (int)floorf(px);
    int y0 = (int)floorf(py);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    /* Clamp to grid */
    if (x0 < 0) x0 = 0;  if (x0 >= f->cols) x0 = f->cols - 1;
    if (y0 < 0) y0 = 0;  if (y0 >= f->rows) y0 = f->rows - 1;
    if (x1 < 0) x1 = 0;  if (x1 >= f->cols) x1 = f->cols - 1;
    if (y1 < 0) y1 = 0;  if (y1 >= f->rows) y1 = f->rows - 1;

    float tx = px - floorf(px);
    float ty = py - floorf(py);

    float a00 = f->angles[y0 * f->cols + x0];
    float a10 = f->angles[y0 * f->cols + x1];
    float a01 = f->angles[y1 * f->cols + x0];
    float a11 = f->angles[y1 * f->cols + x1];

    return lerp(lerp(a00, a10, tx), lerp(a01, a11, tx), ty);
}

/*
 * angle_to_char() — map a flow angle to the best ASCII arrow character.
 *
 * 8 directions → 8 characters:
 *   E   →   '>'
 *   NE  →   '/'  (using ASCII slash for diagonal up-right)
 *   N   →   '^'
 *   NW  →   '\'
 *   W   →   '<'
 *   SW  →   '/'
 *   S   →   'v'
 *   SE  →   '\'
 */
static char angle_to_char(float angle)
{
    static const char arrows[8] = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};
    float a = angle;
    if (a < 0) a += 2.0f * (float)M_PI;
    int idx = (int)(a / (2.0f * (float)M_PI) * 8.0f + 0.5f) % 8;
    return arrows[idx];
}

/* ===================================================================== */
/* §6  particle                                                           */
/* ===================================================================== */

/*
 * Particle — one tracer that follows the flow field.
 *
 * Each particle has a fixed-length trail stored as a circular buffer
 * of (x, y) positions.  The trail is drawn from oldest (dim) to newest
 * (bright) so it fades visually as it ages.
 *
 * alive:  false = slot is free
 * life:   countdown ticks until automatic respawn
 * trail:  ring buffer of past positions, length = trail_len
 * head:   index of the most recent trail position
 * color:  color pair index (1–8); for rainbow theme, updated each tick
 *         from the current angle
 */
#define MAX_TRAIL 20

typedef struct {
    float x, y;
    float speed;          /* per-particle speed in [0.5, 1.3] cells/tick  */
    float angle;          /* current movement angle — drives head char     */
    int   trail_x[MAX_TRAIL];
    int   trail_y[MAX_TRAIL];
    int   head;
    int   trail_fill;
    int   trail_len;
    int   life;
    int   color_pair;
    bool  alive;
} Particle;

static void particle_spawn(Particle *p, int cols, int rows, int trail_len)
{
    p->x         = (float)(rand() % cols);
    p->y         = (float)(rand() % rows);
    /* Each particle gets a unique speed — prevents synchronized lockstep */
    p->speed     = PARTICLE_SPEED - PARTICLE_SPD_JITTER * 0.5f
                 + PARTICLE_SPD_JITTER * ((float)rand() / RAND_MAX);
    p->angle     = 0.0f;
    p->head      = 0;
    p->trail_fill= 0;
    p->trail_len = trail_len;
    p->life      = LIFE_BASE + rand() % LIFE_JITTER;
    p->color_pair= 1 + rand() % N_PAIRS;
    p->alive     = true;
    for (int i = 0; i < MAX_TRAIL; i++) {
        p->trail_x[i] = (int)p->x;
        p->trail_y[i] = (int)p->y;
    }
}

/*
 * particle_tick() — advance one particle one step.
 *
 * 1. Push current integer position into the trail ring buffer.
 * 2. Sample the field at the particle's continuous position.
 * 3. Move by PARTICLE_SPEED in the field direction.
 * 4. Wrap around screen edges (flow field wraps, no hard boundaries).
 * 5. Update color from angle (rainbow) or leave fixed (mono).
 * 6. Decrement life; mark dead when expired.
 */
static void particle_tick(Particle *p, const Field *f,
                            int cols, int rows, int theme)
{
    if (!p->alive) return;

    /* Push current position to trail */
    p->trail_x[p->head] = (int)p->x;
    p->trail_y[p->head] = (int)p->y;
    p->head = (p->head + 1) % p->trail_len;
    if (p->trail_fill < p->trail_len) p->trail_fill++;

    /* Sample field and move using per-particle speed */
    float angle = field_sample(f, p->x, p->y);
    p->angle = angle;
    p->x += cosf(angle) * p->speed;
    p->y += sinf(angle) * p->speed;

    /* Wrap — flow field is toroidal */
    if (p->x < 0)          p->x += (float)cols;
    if (p->x >= (float)cols) p->x -= (float)cols;
    if (p->y < 0)          p->y += (float)rows;
    if (p->y >= (float)rows) p->y -= (float)rows;

    /* Update color */
    if (theme == 0)
        p->color_pair = angle_to_pair(angle);

    /* Age out */
    p->life--;
    if (p->life <= 0) p->alive = false;
}

/*
 * particle_draw() — draw the trail of one particle.
 *
 * Visual improvements over the original:
 *
 * 1. Direction-aware head character.
 *    The head shows an arrow in the particle's current movement direction:
 *      E/W  →  '-'   NE/SW  →  '/'   N/S  →  '|'   NW/SE  →  '\'
 *    This makes each particle look like it is *going somewhere* rather
 *    than being a generic dot. The head is always bold.
 *
 * 2. Proportional trail ramp.
 *    The 4-char ramp '.,+*' is mapped proportionally across the whole
 *    trail regardless of trail length:
 *      position 0/trail_fill   = '.'  (dimmest, oldest)
 *      position trail_fill-1   = '*'  (brightest, newest)
 *    So a trail of 3 and a trail of 20 both show the full gradient.
 *    The old code used raw age index which clipped to '.' for most of
 *    a long trail, making it look like a uniform caterpillar.
 *
 * 3. Proportional color fade.
 *    Color pair fades from base_pair (freshest) to 1 (oldest) across
 *    the trail proportionally:
 *      cp = max(1, base_pair - i * (N_PAIRS-1) / trail_fill)
 *    This uses the full 8-step color range regardless of trail length,
 *    giving a smooth visible hue/brightness gradient along each trail.
 */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows)
{
    if (!p->alive) return;

    static const char k_ramp[] = ".,+~*";   /* 5 levels: dim → bright    */
    const int         RAMP_N   = 5;
    /* Direction chars for 8 octants — matches angle_to_char ordering    */
    static const char k_dir[]  = {'-','/',  '|', '\\', '-', '/', '|', '\\'};

    int fill = p->trail_fill;
    if (fill == 0) return;

    for (int i = 0; i < fill; i++) {
        /* i=0 is oldest, i=fill-1 is newest (head) */
        int ti  = (p->head - fill + i + p->trail_len * 2) % p->trail_len;
        int tx  = p->trail_x[ti];
        int ty  = p->trail_y[ti];

        if (tx < 0 || tx >= cols || ty < 0 || ty >= rows) continue;

        bool is_head = (i == fill - 1);

        char ch;
        if (is_head) {
            /* Direction arrow based on current movement angle */
            float a = p->angle;
            if (a < 0) a += 2.0f * (float)M_PI;
            int di = (int)(a / (2.0f * (float)M_PI) * 8.0f + 0.5f) % 8;
            ch = k_dir[di];
        } else {
            /* Proportional ramp: oldest=0 maps to k_ramp[0], newest-1 maps to k_ramp[RAMP_N-2] */
            int ri = (i * (RAMP_N - 1)) / (fill > 1 ? fill - 1 : 1);
            if (ri >= RAMP_N - 1) ri = RAMP_N - 2;  /* head slot reserved */
            ch = k_ramp[ri];
        }

        /* Proportional color fade: newest = base_pair, oldest = 1 */
        int cp = p->color_pair - (i == fill-1 ? 0
                   : ((fill - 1 - i) * (p->color_pair - 1)) / (fill > 1 ? fill - 1 : 1));
        if (cp < 1) cp = 1;
        if (cp > N_PAIRS) cp = N_PAIRS;

        attr_t attr = COLOR_PAIR(cp);
        if (is_head) attr |= A_BOLD;

        wattron(w, attr);
        mvwaddch(w, ty, tx, (chtype)(unsigned char)ch);
        wattroff(w, attr);
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Scene — field + particle pool + display options.
 *
 * show_field: when true, draw the background arrow field before particles.
 *             Gives a wind-map aesthetic.
 * theme:      0=rainbow 1=cyan 2=green 3=white
 * n_particles: how many slots in pool are active
 */
typedef struct {
    Field    field;
    Particle pool[PARTICLES_MAX];
    int      n_particles;
    int      trail_len;
    int      theme;
    bool     show_field;
    bool     paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->n_particles = PARTICLES_DEFAULT;
    s->trail_len   = TRAIL_LEN_DEFAULT;
    s->theme       = 0;
    s->show_field  = false;
    s->paused      = false;
    field_alloc(&s->field, cols, rows);
    /* Initial field compute */
    field_tick(&s->field);
    /* Spawn all initial particles */
    for (int i = 0; i < s->n_particles; i++)
        particle_spawn(&s->pool[i], cols, rows, s->trail_len);
}

static void scene_free(Scene *s)
{
    field_free(&s->field);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    field_resize(&s->field, cols, rows);
    field_tick(&s->field);
    /* Respawn all particles within new bounds */
    for (int i = 0; i < PARTICLES_MAX; i++) {
        if (s->pool[i].alive)
            particle_spawn(&s->pool[i], cols, rows, s->trail_len);
    }
}

static void scene_respawn_all(Scene *s, int cols, int rows)
{
    for (int i = 0; i < s->n_particles; i++)
        particle_spawn(&s->pool[i], cols, rows, s->trail_len);
    for (int i = s->n_particles; i < PARTICLES_MAX; i++)
        s->pool[i].alive = false;
}

static void scene_tick(Scene *s, int cols, int rows)
{
    if (s->paused) return;

    field_tick(&s->field);

    for (int i = 0; i < s->n_particles; i++) {
        if (!s->pool[i].alive)
            particle_spawn(&s->pool[i], cols, rows, s->trail_len);
        particle_tick(&s->pool[i], &s->field, cols, rows, s->theme);
    }
}

/*
 * scene_draw() — paint field arrows (optional) then particle trails.
 *
 * Field arrows are drawn at low brightness (A_DIM) so particles
 * stand out clearly on top.  The arrow character shows the wind
 * direction at each cell.
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    /* ── optional field background ───────────────────────────────── */
    if (s->show_field) {
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                float angle = s->field.angles[y * cols + x];
                char  ch    = angle_to_char(angle);
                int   cp    = (s->theme == 0) ? angle_to_pair(angle) : 1;
                wattron(w, COLOR_PAIR(cp) | A_DIM);
                mvwaddch(w, y, x, (chtype)(unsigned char)ch);
                wattroff(w, COLOR_PAIR(cp) | A_DIM);
            }
        }
    }

    /* ── particle trails ─────────────────────────────────────────── */
    for (int i = 0; i < s->n_particles; i++)
        particle_draw(&s->pool[i], w, cols, rows);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols;
    int rows;
} Screen;

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

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  n:%d  trail:%d  theme:%d  fps:%d",
             fps, sc->n_particles, sc->trail_len, sc->theme, sim_fps);
    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    int cp = (sc->theme == 0) ? angle_to_pair(0.f) : 1;
    attron(COLOR_PAIR(cp) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}
/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    int cols = app->screen.cols, rows = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        scene_respawn_all(s, cols, rows);
        break;

    case 't': case 'T':
        s->theme = (s->theme + 1) % THEME_COUNT;
        color_apply_theme(s->theme);
        break;

    case 'a': case 'A':   /* toggle field arrows */
        s->show_field = !s->show_field;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case '=': case '+':
        s->n_particles += PARTICLES_STEP;
        if (s->n_particles > PARTICLES_MAX) s->n_particles = PARTICLES_MAX;
        for (int i = s->n_particles - PARTICLES_STEP; i < s->n_particles; i++)
            particle_spawn(&s->pool[i], cols, rows, s->trail_len);
        break;
    case '-':
        s->n_particles -= PARTICLES_STEP;
        if (s->n_particles < PARTICLES_MIN) s->n_particles = PARTICLES_MIN;
        for (int i = s->n_particles; i < s->n_particles + PARTICLES_STEP; i++)
            s->pool[i].alive = false;
        break;

    case 'f':
        s->field.speed *= FIELD_SPD_STEP;
        if (s->field.speed > FIELD_SPD_MAX) s->field.speed = FIELD_SPD_MAX;
        break;
    case 'F':
        s->field.speed /= FIELD_SPD_STEP;
        if (s->field.speed < FIELD_SPD_MIN) s->field.speed = FIELD_SPD_MIN;
        break;

    case 's':
        s->trail_len++;
        if (s->trail_len > TRAIL_LEN_MAX) s->trail_len = TRAIL_LEN_MAX;
        for (int i = 0; i < PARTICLES_MAX; i++)
            s->pool[i].trail_len = s->trail_len;
        break;
    case 'S':
        s->trail_len--;
        if (s->trail_len < TRAIL_LEN_MIN) s->trail_len = TRAIL_LEN_MIN;
        for (int i = 0; i < PARTICLES_MAX; i++)
            s->pool[i].trail_len = s->trail_len;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());
    noise_init();

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    color_init(0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

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
            scene_tick(&app->scene, app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /*
         * ── render interpolation alpha ────────────────────────────
         *
         * sim_accum is the leftover nanoseconds after draining all
         * complete ticks — how far we are into the next tick that
         * has not fired yet.
         *
         * alpha = sim_accum / tick_ns  ∈ [0.0, 1.0)
         *
         * Particles have continuous float positions so alpha could
         * project each particle forward by (speed * alpha) for
         * sub-cell smoothness.  The draw call does not yet accept
         * alpha; it is computed here for structural consistency.
         */
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── FPS counter ─────────────────────────────────────────── */
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

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
