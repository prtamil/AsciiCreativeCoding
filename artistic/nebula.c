/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nebula.c — a drifting glowing gas cloud with twinkling stars and
 * occasional star births that flash and light up the surrounding gas.
 * Gas field uses fBm value-noise (Mandelbrot 1982; Ebert et al., "Texturing
 * & Modeling"); newborn diffraction spikes after Spencer et al. (SIGGRAPH 1995).
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config — fBm, parallax, star, shock constants and colour-pair IDs ── */

#define TARGET_FPS         30        /* slow scene, so 30 fps is plenty    */

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
#define SHOCK_SPEED          5.0f      /* how fast the ring grows, cells/sec */
#define SHOCK_THICKNESS      1.6f
#define SHOCK_FLASH_DUR      0.3f      /* younger than this: 3x3 bright flash */
#define SHOCK_SPIKE_DUR      1.5f      /* younger than this: 4-point spikes;
                                        * also when the lasting star appears  */
#define SHOCK_SAMPLES        12        /* random cells tried to find dense gas */

/* Nebula brightness threshold — below this the gas is invisible. */
#define THRESH_DEFAULT       0.45f
#define THRESH_MIN           0.20f
#define THRESH_MAX           0.85f
#define THRESH_STEP          0.05f

#define ASPECT_X             2.0f      /* cells are ~2x taller than wide       */
#define SCROLL_VDRIFT        0.25f     /* vertical drift = this x layer speed  */

/* How the two scrolling gas layers are mixed, plus how much a shock adds. */
#define GAS_NEAR_W           0.55f     /* weight of the near (fine) layer      */
#define GAS_FAR_W            0.45f     /* weight of the far (coarse) layer     */
#define SHOCK_BOOST          0.5f      /* how much shock light brightens gas   */

#define DT_CAP_S             0.10f
#define N_THEMES             4

/* Colour pairs */
#define PAIR_GAS_0    1   /* dimmest gas                                  */
#define PAIR_GAS_1    2
#define PAIR_GAS_2    3
#define PAIR_GAS_3    4
#define PAIR_GAS_4    5   /* brightest gas (near white)                   */
#define PAIR_STAR     6
#define PAIR_SHOCK    7
#define PAIR_HUD      8
#define PAIR_HINT     9

/* ── §2 performance — monotonic clock and sleep (frame cap lives in §7) ── */

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

/* ── §3 simulation state — Star, Shock, ParallaxLayer, and the Nebula scene ── */

/* One background star: a fixed cell that gently brightens and dims (twinkles).
 * The starfield never moves or needs simulating, so we keep a big fixed pool
 * and just draw it. Stars born from a settling shock are added to this same
 * pool, so a birth leaves a permanent dot in the sky. */
typedef struct {
    int   x, y;            /* fixed cell on the grid                          */
    float brightness;      /* base brightness 0.3..1.0 (newborns near 1.0)    */
    float phase;           /* 0..2pi twinkle offset, random per star, so the
                            * sky shimmers instead of all pulsing together    */
    int   alive;           /* 0 means this pool slot is unused                */
} Star;

/* A star-birth event: an expanding ring of light that brightens the gas and,
 * once it settles, leaves a permanent star behind. Lives SHOCK_LIFE seconds.
 *
 * Everything about the effect is figured out from one number, the age (seconds
 * since birth): the ring's size, the three light contributions (§4
 * shock_brightness_at), and which of three drawing phases it's in (§6). The
 * 4-point spikes and halo mimic how a bright point of light looks through a
 * lens (Spencer et al., SIGGRAPH 1995). The position stays put while the gas
 * drifts past it. */
typedef struct {
    float x, y;            /* birthplace cell; stays fixed, does not scroll    */
    float age;             /* seconds since birth; drives the whole effect     */
    int   alive;           /* 0 means this pool slot is unused                 */
    int   star_added;      /* flips to 1 once the lasting star has been placed */
} Shock;

/* One scrolling layer of gas. We draw two of them: a near layer (fine and
 * fast) and a far layer (coarse and slow). Drawing the same noise at two
 * scales and sliding them at different speeds, then adding them, fakes a sense
 * of depth on a flat screen, the way roadside trees rush past while distant
 * hills barely move. The offset just gets added to each cell's coordinates
 * before sampling, so the cloud slides under a fixed screen, nothing actually
 * moves in memory. */
typedef struct {
    float ox, oy;          /* how far this layer has scrolled, in cells        */
    float speed;           /* drift speed, cells/sec (user can adjust)         */
} ParallaxLayer;

/* The whole scene in one struct. The star and shock pools are sized to their
 * limits up front and never grown or freed (the `alive` flags say which slots
 * are in use), and the gas is recomputed for every cell each frame with no
 * stored grid. So the scene is just this struct plus the terminal, with no
 * memory allocation in the per-frame loop. */
typedef struct {
    Star  stars[N_STARS_MAX];      /* star pool; first n_stars are in use      */
    Shock shocks[N_SHOCKS_MAX];    /* active star-birth events                 */
    int   n_stars;

    float         world_time;      /* seconds elapsed; drives star twinkle     */
    ParallaxLayer near, far;       /* the two scrolling gas layers             */

    float threshold;               /* gas below this brightness is hidden;
                                    * higher means a thinner, sparser cloud    */
    int   theme;                   /* which colour palette is shown            */

    int   paused;
    float next_shock_in;           /* seconds until the next automatic birth   */
} Nebula;

/* ── §4 logic — noise, fBm gas field, and shock illumination (no I/O) ── */

/* Scramble two integers into a repeatable random-looking number in 0..1.
 * Same inputs always give the same output, so the cloud's shape is stable. */
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

/* A smooth random field: pick random values at whole-number grid points and
 * blend between them, giving soft lumps instead of a jagged checkerboard. */
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

/* Stack several copies of the noise at finer and finer scales, each fainter
 * than the last, and add them up. Big soft blobs gain wispy detail, the way
 * real clouds have both broad shapes and fine tendrils (fBm). */
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

/* Turn a 0..1 brightness into one of five steps, picking a glyph and colour. */
static int gas_bucket(float v)
{
    int b = (int)(v * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

/* Extra light that all the active shocks add to one cell. Three things stack:
 * a bright expanding ring, the four spikes radiating from a new star, and a
 * soft round glow around it. Each fades as the shock ages. */
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
        float fade  = 1.0f - (s->age / SHOCK_LIFE);
        if (fade < 0) fade = 0;

        /* The bright ring: brightest exactly at the ring's current radius,
         * dropping off on either side. */
        float ring  = d - r_now;
        float gauss = expf(-ring * ring / (SHOCK_THICKNESS * SHOCK_THICKNESS));
        total += gauss * fade;

        /* The four spikes: thin bright lines straight up/down and left/right
         * through the star, also dimming with distance so they don't reach
         * too far. Strongest right after birth. */
        float spike_t = 1.0f - (s->age / 1.2f);
        if (spike_t > 0.0f) {
            float h_spike = expf(-dy * dy / 0.6f) * expf(-d * d / 80.0f);
            float v_spike = expf(-dx * dx / 0.3f) * expf(-d * d / 80.0f);
            total += (h_spike + v_spike) * spike_t * 1.5f;
        }

        /* The soft glow around the star, fading over a couple of seconds. */
        float halo_t = 1.0f - (s->age / 2.5f);
        if (halo_t < 0.0f) halo_t = 0.0f;
        float halo = expf(-d * d / 12.0f);
        total += halo * halo_t * 0.6f;
    }
    return total;
}

/* ── §5 simulation — seed, spawn shocks, and advance the scene each tick ── */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

static float random_shock_interval(void)
{
    return SHOCK_INTERVAL_MIN
         + frand() * (SHOCK_INTERVAL_MAX - SHOCK_INTERVAL_MIN);
}

/* Place a live star, giving it a random twinkle offset. Used when seeding the
 * field, when the user adds stars, and when a shock plants a newborn. */
static void star_set(Star *st, int x, int y, float brightness)
{
    st->x          = x;
    st->y          = y;
    st->brightness = brightness;
    st->phase      = frand() * 2.0f * (float)M_PI;
    st->alive      = 1;
}

static void nebula_seed_stars(Nebula *n, int rows, int cols)
{
    for (int i = 0; i < n->n_stars; i++) {
        int   sx = rand() % (cols > 0 ? cols : 1);
        int   sy = rand() % ((rows > 1) ? (rows - 1) : 1);
        float b  = 0.3f + 0.7f * frand();
        star_set(&n->stars[i], sx, sy, b);
    }
}

static void nebula_reseed(Nebula *n, int rows, int cols)
{
    for (int i = 0; i < N_STARS_MAX; i++) n->stars[i].alive  = 0;
    for (int i = 0; i < N_SHOCKS_MAX; i++) n->shocks[i].alive = 0;
    nebula_seed_stars(n, rows, cols);
    n->near.ox = n->near.oy = 0.0f;
    n->far.ox  = n->far.oy  = 0.0f;
    n->world_time    = 0.0f;
    n->next_shock_in = random_shock_interval();
}

/* Index of the first free shock slot, or -1 if all are in use. */
static int find_dead_shock(const Nebula *n)
{
    for (int i = 0; i < N_SHOCKS_MAX; i++)
        if (!n->shocks[i].alive) return i;
    return -1;
}

/* Try a handful of random cells and return the one with the thickest gas, so
 * stars tend to be born in the bright parts of the cloud. */
static void densest_cell(const Nebula *n, int rows, int cols, int *bx, int *by)
{
    *bx = cols / 2; *by = (rows - 1) / 2;
    float best_v = -1.0f;
    for (int k = 0; k < SHOCK_SAMPLES; k++) {
        int cx = rand() % cols;
        int cy = rand() % ((rows - 1) > 0 ? (rows - 1) : 1);
        float fn = fbm((float)cx + n->near.ox,
                       (float)cy + n->near.oy, FBM_FREQ_NEAR);
        float ff = fbm((float)cx + n->far.ox,
                       (float)cy + n->far.oy, FBM_FREQ_FAR);
        float v  = GAS_NEAR_W * fn + GAS_FAR_W * ff;
        if (v > best_v) { best_v = v; *bx = cx; *by = cy; }
    }
}

/* Start a new star birth at the densest gas cell we can find. */
static void shock_spawn(Nebula *n, int rows, int cols)
{
    int slot = find_dead_shock(n);
    if (slot < 0) return;

    int bx, by;
    densest_cell(n, rows, cols, &bx, &by);
    n->shocks[slot].x          = (float)bx;
    n->shocks[slot].y          = (float)by;
    n->shocks[slot].age        = 0.0f;
    n->shocks[slot].alive      = 1;
    n->shocks[slot].star_added = 0;
}

/* Tick the clock forward and slide both gas layers. Horizontal drift is scaled
 * up because cells are taller than wide, so motion looks even; vertical drift
 * is kept gentle. */
static void advance_parallax(Nebula *n, float dt)
{
    n->world_time += dt;
    n->near.ox += n->near.speed * ASPECT_X      * dt;
    n->far.ox  += n->far.speed  * ASPECT_X      * dt;
    n->near.oy += n->near.speed * SCROLL_VDRIFT * dt;
    n->far.oy  += n->far.speed  * SCROLL_VDRIFT * dt;
}

/* Age each active shock. When its spike phase ends, leave a permanent star
 * behind; once it has lived past SHOCK_LIFE, free the slot. */
static void age_shocks(Nebula *n, float dt)
{
    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        Shock *s = &n->shocks[i];
        if (!s->alive) continue;
        s->age += dt;

        if (!s->star_added && s->age >= SHOCK_SPIKE_DUR) {
            if (n->n_stars < N_STARS_MAX) {
                float b = 0.85f + 0.15f * frand();          /* newborns are bright */
                star_set(&n->stars[n->n_stars++], (int)s->x, (int)s->y, b);
            }
            s->star_added = 1;
        }

        if (s->age > SHOCK_LIFE) s->alive = 0;
    }
}

static void nebula_tick(Nebula *n, float dt, int rows, int cols)
{
    advance_parallax(n, dt);
    age_shocks(n, dt);

    /* Count down to the next automatic star birth. */
    n->next_shock_in -= dt;
    if (n->next_shock_in <= 0.0f) {
        shock_spawn(n, rows, cols);
        n->next_shock_in = random_shock_interval();
    }
}

/* ── §6 render — draw gas, stars, shocks and the HUD; never changes state ── */

/* Five gas colours per theme, plus a star and shock colour. Each theme has a
 * different main hue so pressing 't' visibly changes the sky. The darkest end
 * is kept off pure black so it stays visible. */
static const short GAS_256[N_THEMES][5] = {
    /* 0 emission red:    pink/red to orange to white                 */
    {  88, 131, 196, 215, 231 },
    /* 1 reflection blue: deep blue to cyan to white                  */
    {  25,  33,  39,  51, 231 },
    /* 2 emerald nebula:  dark green to lime to yellow-green to white */
    {  22,  28,  82, 154, 231 },
    /* 3 horsehead pink:  purple to magenta to light pink to white    */
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
    init_pair(PAIR_HUD,   x256 ? 226 : COLOR_YELLOW, -1);  /* top: bright yellow */
    init_pair(PAIR_HINT,  x256 ? 51  : COLOR_CYAN,   -1);  /* bottom: bright cyan */
}

static const char STAR_GLYPH[3] = { '.', '+', '*' };

/* Draw one star as a dot that gently brightens and dims over time. Its current
 * brightness picks one of three glyphs and whether it's bold or dim. */
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

/* Glyphs from faint to thick gas. */
static const char GAS_GLYPH[5] = { '.', ':', '~', 'o', '#' };

/* Draw the gas: for every cell, add up the two scrolling layers plus any shock
 * light, and print a glyph wherever the result is bright enough to show. */
static void draw_gas(const Nebula *n, int rows, int cols)
{
    for (int sr = 0; sr < rows - 1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            float fn = fbm((float)sc + n->near.ox,
                           (float)sr + n->near.oy, FBM_FREQ_NEAR);
            float ff = fbm((float)sc + n->far.ox,
                           (float)sr + n->far.oy, FBM_FREQ_FAR);
            float v = GAS_NEAR_W * fn + GAS_FAR_W * ff;
            v += shock_brightness_at(n, (float)sc, (float)sr) * SHOCK_BOOST;
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
}

/* The whole twinkling star catalogue, drawn over the gas. */
static void draw_stars(const Nebula *n, int rows, int cols)
{
    for (int i = 0; i < n->n_stars; i++)
        star_draw(&n->stars[i], n->world_time, rows, cols);
}

/* Newest shock: a bright 3x3 flash at the centre. */
static void draw_shock_flash(int sr, int sc, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            int rr = sr + dr, cc = sc + dc;
            if (rr < 0 || rr >= rows - 1) continue;
            if (cc < 0 || cc >= cols)     continue;
            char ch = (dr == 0 && dc == 0) ? '*' : '+';
            mvaddch(rr, cc, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
}

/* A young star: a 4-pointed cross with spikes reaching two cells each way. */
static void draw_shock_spikes(int sr, int sc, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
    mvaddch(sr, sc, (chtype)'*');
    if (sc - 1 >= 0)        mvaddch(sr, sc - 1, (chtype)'-');
    if (sc - 2 >= 0)        mvaddch(sr, sc - 2, (chtype)'<');
    if (sc + 1 < cols)      mvaddch(sr, sc + 1, (chtype)'-');
    if (sc + 2 < cols)      mvaddch(sr, sc + 2, (chtype)'>');
    if (sr - 1 >= 0)        mvaddch(sr - 1, sc, (chtype)'|');
    if (sr + 1 < rows - 1)  mvaddch(sr + 1, sc, (chtype)'|');
    attroff(COLOR_PAIR(PAIR_SHOCK) | A_BOLD);
}

/* A settling shock: a single dot fading out as the event ends. */
static void draw_shock_settle(int sr, int sc, float age)
{
    float fade = 1.0f - (age / SHOCK_LIFE);
    if (fade <= 0.2f) return;
    chtype attr = COLOR_PAIR(PAIR_SHOCK) | (fade > 0.5f ? A_BOLD : 0);
    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)(fade > 0.6f ? '+' : '.'));
    attroff(attr);
}

/* Draw each shock's centre in the right stage for its age:
 * flash, then 4-point spikes, then a fading dot. */
static void draw_shocks(const Nebula *n, int rows, int cols)
{
    for (int i = 0; i < N_SHOCKS_MAX; i++) {
        const Shock *s = &n->shocks[i];
        if (!s->alive) continue;
        int sr = (int)s->y, sc = (int)s->x;
        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;

        if      (s->age < SHOCK_FLASH_DUR) draw_shock_flash(sr, sc, rows, cols);
        else if (s->age < SHOCK_SPIKE_DUR) draw_shock_spikes(sr, sc, rows, cols);
        else                               draw_shock_settle(sr, sc, s->age);
    }
}

/* HUD — data readout top-right, action keys bottom-left. */
static void draw_hud(const Nebula *n, int rows, int cols, double fps)
{
    int n_shocks_active = 0;
    for (int i = 0; i < N_SHOCKS_MAX; i++)
        if (n->shocks[i].alive) n_shocks_active++;

    char buf[160];
    snprintf(buf, sizeof buf,
             " stars:%d  shocks:%d  thresh:%.2f  scroll:%.2f  theme:%d  "
             "%5.1f fps  %s ",
             n->n_stars, n_shocks_active, n->threshold,
             n->near.speed, n->theme, fps,
             n->paused ? "PAUSED " : "running");
    int x = cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " [/]:stars  -/+:thresh  ,/.:scroll  b:birth  t:theme  "
             "r:reset  p:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Nebula *n, double fps)
{
    erase();
    draw_gas(n, rows, cols);
    draw_stars(n, rows, cols);
    draw_shocks(n, rows, cols);
    draw_hud(n, rows, cols, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

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

/* ── §7 app — signals, the main loop, key handling, and the frame cap ── */

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
    g_neb.near.speed = SCROLL_NEAR_DEFAULT;
    g_neb.far.speed  = SCROLL_FAR_DEFAULT;
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
        /* Window was resized: rebuild for the new size and reseed the scene. */
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            nebula_reseed(&g_neb, rows, cols);
        }

        /* Read all pending keys and adjust the knobs or run-state. */
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
                            int   sx = rand() % cols;
                            int   sy = rand() % (rows - 1);
                            float b  = 0.3f + 0.7f * frand();
                            star_set(&g_neb.stars[i], sx, sy, b);
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
                    if (g_neb.near.speed - SCROLL_STEP >= SCROLL_MIN) {
                        g_neb.near.speed -= SCROLL_STEP;
                        g_neb.far.speed  -= SCROLL_STEP * 0.3f;
                        if (g_neb.far.speed < SCROLL_MIN) g_neb.far.speed = SCROLL_MIN;
                    }
                    break;
                case '.':
                    if (g_neb.near.speed + SCROLL_STEP <= SCROLL_MAX) {
                        g_neb.near.speed += SCROLL_STEP;
                        g_neb.far.speed  += SCROLL_STEP * 0.3f;
                    }
                    break;
            }
        }

        /* Seconds since last frame, capped so a long stall can't make the sim
         * lurch forward in one giant step. */
        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;

        /* Advance the scene, unless paused. */
        if (!g_neb.paused) nebula_tick(&g_neb, dt, rows, cols);

        /* Smoothed fps for the HUD. */
        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_neb, fps);

        /* Sleep off the rest of the frame so we hold the target rate. */
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
