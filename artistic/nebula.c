/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nebula.c — drifting nebula with star births and shock illumination
 *
 * DEMO: A multi-octave fBm scalar field defines the density and tint
 *       of a glowing gas cloud that fills the screen. The field
 *       slowly scrolls across two parallax layers — a near layer
 *       (high frequency, dim) and a far layer (low frequency, brighter)
 *       — giving an illusion of cosmic depth. A static catalogue of
 *       background stars sparkles. Periodically (every ~5–8 seconds)
 *       a NEW STAR is born somewhere inside the densest gas: a sharp
 *       flash, then a slow expanding spherical shock-wave illuminates
 *       the surrounding gas for several seconds before fading.
 *
 *       The result is a still, vast, slowly evolving cosmic scene with
 *       occasional dramatic moments — a star nursery seen from a
 *       distant telescope.
 *
 * Study alongside: artistic/galaxy.c (rotating disc — opposite scale),
 *                  fluid/wave_2d.c (the radial wave-front under the
 *                  shock illumination uses similar 1/r decay),
 *                  fractal_random/perlin_landscape.c (multi-octave fBm).
 *
 * Section map:
 *   §1 config    — fBm octaves, parallax, star catalogue, shock cadence
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — multi-stop nebula palette per theme + star colour
 *   §4 random    — frand
 *   §5 noise     — value-noise + fBm helper
 *   §6 star      — Star catalogue + Shock event pool
 *   §7 nebula    — Nebula state, scroll, lifecycle
 *   §8 scene     — fBm raster + stars + shock illumination + HUD
 *   §9 screen    — ncurses init / cleanup
 *  §10 app       — signals, dt + world_time tracking, main loop
 *
 * Keys:  [/]   star count (50..400)
 *        -/+   nebula brightness threshold
 *        ,/.   parallax scroll speed
 *        b     trigger a star birth on demand
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/nebula.c \
 *       -o nebula -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : The gas is a 2-D fBm field — sum of OCTAVES value-noise
 *                 layers at doubling frequency and halving amplitude.
 *                 Two layers are sampled and added: a "near" layer at
 *                 fine spatial scale, and a "far" layer at coarse scale,
 *                 each scrolled at a different speed (`scroll_near` >
 *                 `scroll_far`). The combined field controls colour
 *                 (deep gas → cool, sparse → dark) and glyph density
 *                 (Bourke ramp, indexed by field magnitude).
 *
 *                 Stars are a fixed catalogue: random (x, y, brightness,
 *                 phase) generated at startup; rendered each frame with
 *                 a slow sinusoidal twinkle.
 *
 *                 Star births are events with a small expanding shock-
 *                 wave: each Shock has (x, y, age). A radius `r =
 *                 SHOCK_SPEED · age` advances; cells whose distance to
 *                 (x, y) is near `r` get extra brightness. Older shocks
 *                 fade. New shocks spawn periodically + on user demand.
 *
 * Data-structure: Nebula holds Star[N_STARS_MAX] and Shock[N_SHOCKS_MAX]
 *                 inline. fBm is evaluated on demand per cell each frame
 *                 — O(rows · cols · OCTAVES) per frame. At 80×24×4 ≈ 7k
 *                 evaluations, each ~10 multiplies — easily 30 fps.
 *
 * Rendering     : Per frame: fBm raster (every cell), then star catalogue,
 *                 then shocks (additive brightness over the gas). HUD
 *                 last.
 *
 * References    :
 *   Perlin, "An Image Synthesizer" SIGGRAPH (1985) — original noise.
 *   Mandelbrot, "The Fractal Geometry of Nature" (1982) — fBm.
 *   Musgrave, Kolb & Mace, "The synthesis and rendering of eroded
 *     fractal terrains" SIGGRAPH (1989) — fBm parameter choices.
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

/* fBm */
#define FBM_OCTAVES         4
#define FBM_FREQ_NEAR       0.10f
#define FBM_FREQ_FAR        0.04f
#define FBM_LACUNARITY      2.0f
#define FBM_GAIN            0.55f

/* Parallax scroll speeds — cell-units per second. */
#define SCROLL_NEAR_DEFAULT 0.6f
#define SCROLL_FAR_DEFAULT  0.18f
#define SCROLL_MIN          0.0f
#define SCROLL_MAX          3.0f
#define SCROLL_STEP         0.1f

/* Stars */
#define N_STARS_MAX         400
#define N_STARS_DEFAULT     180
#define N_STARS_MIN          50
#define N_STARS_STEP         50

/* Shocks (star births) */
#define N_SHOCKS_MAX        12
#define SHOCK_INTERVAL_MIN   4.0f
#define SHOCK_INTERVAL_MAX   9.0f
#define SHOCK_LIFE           5.0f
#define SHOCK_SPEED          5.0f      /* cells/sec radius growth          */
#define SHOCK_THICKNESS      1.6f

/* Nebula brightness threshold — below this the gas is invisible. */
#define THRESH_DEFAULT       0.45f
#define THRESH_MIN           0.20f
#define THRESH_MAX           0.85f
#define THRESH_STEP          0.05f

#define ASPECT_X             2.0f
#define DT_CAP_S             0.10f
#define N_THEMES             4

/* Colour pairs */
#define PAIR_GAS_0    1   /* coolest gas (e.g. deep blue)                 */
#define PAIR_GAS_1    2
#define PAIR_GAS_2    3
#define PAIR_GAS_3    4
#define PAIR_GAS_4    5   /* hottest gas (white-pink)                     */
#define PAIR_STAR     6
#define PAIR_SHOCK    7
#define PAIR_HUD      8
#define PAIR_HINT     9

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

/* 5-stop nebula palette per theme + a star colour and shock colour.
 * Each theme has a distinct dominant hue so cycling 't' is visibly
 * different across the sky; the dim end is brightened from near-black
 * (17/22/52) to visible mid-tones. */
static const short GAS_256[N_THEMES][5] = {
    /* 0 emission red   — pink/red → orange → white                   */
    {  88, 131, 196, 215, 231 },
    /* 1 reflection blue — deep blue → cyan → white                   */
    {  25,  33,  39,  51, 231 },
    /* 2 emerald nebula — dark green → lime → yellow-green → white     */
    {  22,  28,  82, 154, 231 },
    /* 3 horsehead pink — purple → magenta → light pink → white       */
    {  53,  91, 165, 213, 231 },
};
static const short STAR_256[N_THEMES] = { 231, 231, 231, 231 };
static const short SHOCK_256[N_THEMES] = { 226, 226, 226, 226 };

static const short GAS_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_YELLOW, COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,   COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW, COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? GAS_256[theme][i] : GAS_8[theme][i];
        init_pair((short)(PAIR_GAS_0 + i), fg, -1);
    }
    init_pair(PAIR_STAR,  x256 ? STAR_256[theme]  : COLOR_WHITE,  -1);
    init_pair(PAIR_SHOCK, x256 ? SHOCK_256[theme] : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,   x256 ? 0  : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,  x256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  noise — value noise + fBm                                           */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Fast integer hash → [0, 1] for value-noise lattice samples. */
static float hash01(int x, int y)
{
    uint32_t h = (uint32_t)(x * 374761393) ^ (uint32_t)(y * 668265263);
    h = (h ^ (h >> 13)) * 1274126177u;
    h = h ^ (h >> 16);
    return (float)(h & 0xFFFFFF) / (float)0x1000000;
}

static float smoothstep(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

/* Value noise at fractional (x, y). Bilinear interp + smoothstep. */
static float value_noise(float x, float y)
{
    int   xi = (int)floorf(x), yi = (int)floorf(y);
    float fx = x - (float)xi,  fy = y - (float)yi;
    float v00 = hash01(xi,   yi);
    float v10 = hash01(xi+1, yi);
    float v01 = hash01(xi,   yi+1);
    float v11 = hash01(xi+1, yi+1);
    float ux = smoothstep(fx);
    float uy = smoothstep(fy);
    float a  = v00 + (v10 - v00) * ux;
    float b  = v01 + (v11 - v01) * ux;
    return a + (b - a) * uy;
}

/* fBm: sum of OCTAVES value-noise samples at increasing freq. */
static float fbm(float x, float y, float base_freq)
{
    float sum  = 0.0f;
    float amp  = 1.0f;
    float freq = base_freq;
    float norm = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        sum += amp * value_noise(x * freq, y * freq);
        norm += amp;
        amp  *= FBM_GAIN;
        freq *= FBM_LACUNARITY;
    }
    return sum / norm;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  star + shock                                                        */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int   x, y;            /* fixed integer cell                          */
    float brightness;      /* 0.3..1.0                                     */
    float phase;            /* 0..2π for twinkle                            */
    int   alive;
} Star;

typedef struct {
    float x, y;            /* world-space (cell) — does NOT scroll        */
    float age;              /* seconds since birth                          */
    int   alive;
} Shock;

static const char STAR_GLYPH[3] = { '.', '+', '*' };

/*
 * star_draw — bright dot with sinusoidal twinkle. brightness × twinkle
 * picks one of three glyphs and bold/dim attribute.
 */
static void star_draw(const Star *s, float world_time, int rows, int cols)
{
    if (!s->alive) return;
    if (s->x < 0 || s->x >= cols || s->y < 0 || s->y >= rows - 1) return;
    float twinkle = 0.7f + 0.3f * sinf(world_time * 1.5f + s->phase);
    float v       = s->brightness * twinkle;
    int   bucket  = (v < 0.4f) ? 0 : (v < 0.75f) ? 1 : 2;
    chtype attr   = COLOR_PAIR(PAIR_STAR);
    if (bucket == 2) attr |= A_BOLD;
    if (bucket == 0) attr |= A_DIM;
    attron(attr);
    mvaddch(s->y, s->x, (chtype)(unsigned char)STAR_GLYPH[bucket]);
    attroff(attr);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  nebula                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    Star  stars[N_STARS_MAX];
    Shock shocks[N_SHOCKS_MAX];
    int   n_stars;

    float world_time;
    float scroll_x_near, scroll_y_near;
    float scroll_x_far,  scroll_y_far;

    float scroll_near, scroll_far;     /* scalars: speeds */
    float threshold;
    int   theme;
    int   paused;

    float next_shock_in;               /* seconds until next auto birth */
} Nebula;

static float random_shock_interval(void)
{
    return SHOCK_INTERVAL_MIN
         + frand() * (SHOCK_INTERVAL_MAX - SHOCK_INTERVAL_MIN);
}

static void nebula_seed_stars(Nebula *n, int rows, int cols)
{
    for (int i = 0; i < n->n_stars; i++) {
        n->stars[i].x          = rand() % (cols > 0 ? cols : 1);
        n->stars[i].y          = rand() % ((rows > 1) ? (rows - 1) : 1);
        n->stars[i].brightness = 0.3f + 0.7f * frand();
        n->stars[i].phase      = frand() * 2.0f * (float)M_PI;
        n->stars[i].alive      = 1;
    }
}

static void nebula_reseed(Nebula *n, int rows, int cols)
{
    for (int i = 0; i < N_STARS_MAX; i++) n->stars[i].alive  = 0;
    for (int i = 0; i < N_SHOCKS_MAX; i++) n->shocks[i].alive = 0;
    nebula_seed_stars(n, rows, cols);
    n->scroll_x_near = n->scroll_y_near = 0.0f;
    n->scroll_x_far  = n->scroll_y_far  = 0.0f;
    n->world_time    = 0.0f;
    n->next_shock_in = random_shock_interval();
}

/*
 * shock_spawn — find a dead slot, set position to a high-density spot
 * (sample fBm at random points, keep the brightest of K tries).
 */
static void shock_spawn(Nebula *n, int rows, int cols)
{
    int slot = -1;
    for (int i = 0; i < N_SHOCKS_MAX; i++)
        if (!n->shocks[i].alive) { slot = i; break; }
    if (slot < 0) return;

    /* Sample K candidate positions; pick the brightest gas cell. */
    int best_x = cols / 2, best_y = (rows - 1) / 2;
    float best_v = -1.0f;
    for (int k = 0; k < 12; k++) {
        int cx = rand() % cols;
        int cy = rand() % ((rows - 1) > 0 ? (rows - 1) : 1);
        float fn = fbm((float)cx + n->scroll_x_near,
                       (float)cy + n->scroll_y_near, FBM_FREQ_NEAR);
        float ff = fbm((float)cx + n->scroll_x_far,
                       (float)cy + n->scroll_y_far, FBM_FREQ_FAR);
        float v  = 0.55f * fn + 0.45f * ff;
        if (v > best_v) { best_v = v; best_x = cx; best_y = cy; }
    }
    n->shocks[slot].x     = (float)best_x;
    n->shocks[slot].y     = (float)best_y;
    n->shocks[slot].age   = 0.0f;
    n->shocks[slot].alive = 1;
}

static void nebula_tick(Nebula *n, float dt, int rows, int cols)
{
    n->world_time     += dt;
    n->scroll_x_near  += n->scroll_near * ASPECT_X * dt;
    n->scroll_x_far   += n->scroll_far  * ASPECT_X * dt;
    /* Slight vertical drift so the parallax isn't purely horizontal. */
    n->scroll_y_near  += n->scroll_near * 0.25f * dt;
    n->scroll_y_far   += n->scroll_far  * 0.25f * dt;

    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        if (!n->shocks[i].alive) continue;
        n->shocks[i].age += dt;
        if (n->shocks[i].age > SHOCK_LIFE) n->shocks[i].alive = 0;
    }

    n->next_shock_in -= dt;
    if (n->next_shock_in <= 0.0f) {
        shock_spawn(n, rows, cols);
        n->next_shock_in = random_shock_interval();
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Glyph ramp by gas brightness bucket. */
static const char GAS_GLYPH[5] = { '.', ':', '~', 'o', '#' };

static int gas_bucket(float v)
{
    int b = (int)(v * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

/*
 * shock_brightness_at — extra brightness contribution from active shocks
 * at cell (sx, sy). Each shock adds a Gaussian-ish ring of brightness
 * centred on its expanding radius.
 */
static float shock_brightness_at(const Nebula *n, float sx, float sy)
{
    float total = 0.0f;
    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        const Shock *s = &n->shocks[i];
        if (!s->alive) continue;
        float r_now = SHOCK_SPEED * s->age;
        float dx    = (sx - s->x) / ASPECT_X;
        float dy    = (sy - s->y);
        float d     = sqrtf(dx * dx + dy * dy);
        float ring  = d - r_now;
        float fade  = 1.0f - (s->age / SHOCK_LIFE);
        if (fade < 0) fade = 0;
        /* Gaussian-ish ring, peak at d == r_now. */
        float gauss = expf(-ring * ring / (SHOCK_THICKNESS * SHOCK_THICKNESS));
        total += gauss * fade;
    }
    return total;
}

static void scene_draw(int rows, int cols, const Nebula *n, double fps)
{
    erase();

    /* fBm raster across every cell. */
    for (int sr = 0; sr < rows - 1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            float fn = fbm((float)sc + n->scroll_x_near,
                           (float)sr + n->scroll_y_near, FBM_FREQ_NEAR);
            float ff = fbm((float)sc + n->scroll_x_far,
                           (float)sr + n->scroll_y_far, FBM_FREQ_FAR);
            float v = 0.55f * fn + 0.45f * ff;
            float boost = shock_brightness_at(n, (float)sc, (float)sr);
            v += boost * 0.5f;
            if (v < n->threshold) continue;

            float intensity = (v - n->threshold) / (1.0f - n->threshold + 0.001f);
            if (intensity < 0) intensity = 0;
            if (intensity > 1) intensity = 1;
            int bucket = gas_bucket(intensity);
            chtype attr = COLOR_PAIR(PAIR_GAS_0 + bucket);
            attr |= (bucket >= 3) ? A_BOLD : A_DIM;
            attron(attr);
            mvaddch(sr, sc, (chtype)(unsigned char)GAS_GLYPH[bucket]);
            attroff(attr);
        }
    }

    /* Stars on top of gas. */
    for (int i = 0; i < n->n_stars; i++)
        star_draw(&n->stars[i], n->world_time, rows, cols);

    /* Shock centres marked as a bright pulsing dot. */
    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        const Shock *s = &n->shocks[i];
        if (!s->alive) continue;
        int sr = (int)s->y, sc = (int)s->x;
        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
        float fade = 1.0f - (s->age / SHOCK_LIFE);
        if (fade < 0) fade = 0;
        chtype attr = COLOR_PAIR(PAIR_SHOCK) | (fade > 0.5f ? A_BOLD : 0);
        attron(attr);
        mvaddch(sr, sc, (chtype)(unsigned char)(fade > 0.7f ? '*' : '+'));
        attroff(attr);
    }

    /* HUD */
    char buf[160];
    int  n_shocks_active = 0;
    for (int i = 0; i < N_SHOCKS_MAX; i++)
        if (n->shocks[i].alive) n_shocks_active++;
    snprintf(buf, sizeof buf,
             " stars:%d  shocks:%d  thresh:%.2f  scroll:%.2f  theme:%d  "
             "%5.1f fps  %s ",
             n->n_stars, n_shocks_active, n->threshold,
             n->scroll_near, n->theme, fps,
             n->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:stars  -/+:thresh  ,/.:scroll  b:birth  t:theme  "
             "r:reset  p:pause  q:quit  [nebula] ");
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

static Nebula g_neb;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_neb.n_stars     = N_STARS_DEFAULT;
    g_neb.scroll_near = SCROLL_NEAR_DEFAULT;
    g_neb.scroll_far  = SCROLL_FAR_DEFAULT;
    g_neb.threshold   = THRESH_DEFAULT;
    g_neb.theme       = 0;

    screen_init(g_neb.theme);
    int rows = LINES, cols = COLS;
    nebula_reseed(&g_neb, rows, cols);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            nebula_reseed(&g_neb, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_neb.paused ^= 1; break;
                case 'r':          nebula_reseed(&g_neb, rows, cols); break;
                case 't':          g_neb.theme = (g_neb.theme + 1) % N_THEMES;
                                   color_init(g_neb.theme); break;
                case 'b':          shock_spawn(&g_neb, rows, cols); break;
                case '[':
                    if (g_neb.n_stars - N_STARS_STEP >= N_STARS_MIN) {
                        g_neb.n_stars -= N_STARS_STEP;
                    }
                    break;
                case ']':
                    if (g_neb.n_stars + N_STARS_STEP <= N_STARS_MAX) {
                        int old = g_neb.n_stars;
                        g_neb.n_stars += N_STARS_STEP;
                        for (int i = old; i < g_neb.n_stars; i++) {
                            g_neb.stars[i].x = rand() % cols;
                            g_neb.stars[i].y = rand() % (rows - 1);
                            g_neb.stars[i].brightness = 0.3f + 0.7f * frand();
                            g_neb.stars[i].phase = frand() * 2.0f * (float)M_PI;
                            g_neb.stars[i].alive = 1;
                        }
                    }
                    break;
                case '-':
                    if (g_neb.threshold + THRESH_STEP <= THRESH_MAX)
                        g_neb.threshold += THRESH_STEP;
                    break;
                case '+': case '=':
                    if (g_neb.threshold - THRESH_STEP >= THRESH_MIN)
                        g_neb.threshold -= THRESH_STEP;
                    break;
                case ',':
                    if (g_neb.scroll_near - SCROLL_STEP >= SCROLL_MIN) {
                        g_neb.scroll_near -= SCROLL_STEP;
                        g_neb.scroll_far  -= SCROLL_STEP * 0.3f;
                        if (g_neb.scroll_far < SCROLL_MIN) g_neb.scroll_far = SCROLL_MIN;
                    }
                    break;
                case '.':
                    if (g_neb.scroll_near + SCROLL_STEP <= SCROLL_MAX) {
                        g_neb.scroll_near += SCROLL_STEP;
                        g_neb.scroll_far  += SCROLL_STEP * 0.3f;
                    }
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_neb.paused) nebula_tick(&g_neb, dt, rows, cols);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_neb, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
