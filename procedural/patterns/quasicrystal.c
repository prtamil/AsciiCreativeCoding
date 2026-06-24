/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * quasicrystal.c — animated quasicrystal interference patterns in the terminal.
 *
 * We add up a handful of cosine waves coming from evenly-spaced directions
 * around a circle, then slowly drift each wave's phase so the picture morphs.
 * When the number of waves is one the usual crystal symmetries (3/4/6) can't
 * accommodate — 5, 7, 11 — the result has perfect rotational symmetry but
 * never repeats. That non-repeating-but-ordered look is a quasicrystal, the
 * same idea behind Penrose tilings.
 *
 * The interesting choices live in two tables in §1: presets[] (which pattern)
 * and themes[] (which colours). See the key legend on the bottom HUD row.
 *
 * Why odd wave counts never repeat: a 2-D lattice can only have 2/3/4/6-fold
 * rotational symmetry (the crystallographic restriction theorem), so a pattern
 * with 10- or 14-fold symmetry simply can't be a repeating tile.
 *
 * References (the code can't tell you these):
 *   Levine & Steinhardt (1984), Phys. Rev. Lett. 53:2477 — coined
 *     "quasicrystal" and modelled it as exactly this sum of plane waves.
 *   Shechtman et al. (1984), Phys. Rev. Lett. 53:1951 — the experimental
 *     discovery (Shechtman's 2011 Nobel Prize).
 *   K. McAllister (2011), "Quasicrystals as sums of waves in the plane" —
 *     a step-by-step build of this same animation; closest match to the code.
 *   P. Bourke, "Character representation of grey scale images" — the
 *     intensity-to-ASCII ramp behind RAMP_GLYPHS.
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

/* §1  CONFIG — constants, presets, glyph/theme tables (data only) */

enum {
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    RENDER_FPS_CAP      =  60,    /* cap how often we redraw, so a fast box doesn't spin */
    MAX_FRAME_DT_MS     = 100,    /* if a frame stalls, pretend no more than this passed,
                                   * otherwise the sim tries to catch up forever */

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    N_WAVES_MAX         =  13,    /* most waves any preset uses (STAR-13) */

    /* Colour pair slots. PAIR_HUD / PAIR_HINT are reserved by the house style. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 hold the 8 gradient tiers */
    PAIR_FLASH          =  13,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y_F           2.0f

/*
 * Reseed flash: pressing 'r' sets flash_t to 1.0, and we fade it out over the
 * next fraction of a second. While it is still bright we sprinkle a few twinkle
 * stars on top so the snap to a new pattern is visible.
 *   FLASH_DECAY_RATE   — how fast the flash fades (higher = quicker)
 *   FLASH_VISIBLE_MIN  — stop drawing the twinkle once it dims below this
 */
#define FLASH_DECAY_RATE     4.0f
#define FLASH_VISIBLE_MIN    0.05f

/*
 * Preset — one entry in the pattern gallery. A preset is just four numbers,
 * which is why the gallery is a flat table: adding or tweaking a look is a
 * one-line change. (These are exactly the knobs Levine & Steinhardt vary to
 * get different quasicrystalline orders.)
 *
 *   name       — the label shown in the HUD.
 *   n_waves    — how many waves. This sets the symmetry: more waves, more
 *                points on each star. Counts of 3/4/6/12 give plain repeating
 *                crystals; 5/7/8/9/10/11/13 give the non-repeating
 *                quasicrystals. Must stay within N_WAVES_MAX.
 *   wavelength — feature size, in cells. Smaller packs the pattern tighter;
 *                larger makes big sweeping shapes. Same wave count at a
 *                different wavelength looks completely different.
 *   rate_base  — how fast the whole pattern drifts (radians per second).
 *   rate_delta — a small extra drift added per wave. Without it every wave
 *                moves in lockstep and the pattern just slides; with it the
 *                waves slowly slip out of step so the pattern morphs in place.
 */
typedef struct {
    const char *name;       /* label shown in the HUD                          */
    int         n_waves;    /* number of waves — drives the symmetry           */
    float       wavelength; /* feature size in cells — drives the density      */
    float       rate_base;  /* drift speed of the whole pattern (rad/s)        */
    float       rate_delta; /* extra per-wave drift — makes it morph, not slide */
} Preset;

#define N_PRESETS 15

/*
 * The gallery: a sweep of wave counts 3..13 (plain crystals mixed in with
 * quasicrystals so the contrast is easy to see), plus a few fine/giant
 * variants of the best-looking orders. Boots into PENROSE-5, the classic
 * 10-pointed look.
 */
static const Preset presets[N_PRESETS] = {
    /* name         N   λ     base  delta */
    { "HEX-3",      3, 16.0f, 0.50f, 0.07f },  /* periodic hexagon — the reference */
    { "SQUARE-4",   4, 16.0f, 0.45f, 0.06f },  /* 8-fold, square-ish weave         */
    { "PENROSE-5",  5, 14.0f, 0.50f, 0.07f },  /* 10-fold, Penrose-flavoured       */
    { "FLOWER-6",   6, 18.0f, 0.40f, 0.05f },  /* periodic hexagonal rosettes      */
    { "SEPTA-7",    7, 13.0f, 0.55f, 0.08f },  /* 14-fold, denser stars            */
    { "OCTA-8",     8, 14.0f, 0.50f, 0.06f },  /* 16-fold octagonal                */
    { "ENNEA-9",    9, 12.0f, 0.55f, 0.07f },  /* 18-fold                          */
    { "DECA-10",   10, 13.0f, 0.50f, 0.06f },  /* 20-fold                          */
    { "UNDECA-11", 11, 12.0f, 0.60f, 0.08f },  /* 22-fold, near-cloud detail       */
    { "DODECA-12", 12, 13.0f, 0.45f, 0.05f },  /* 24-fold, near-periodic           */
    { "STAR-13",   13, 11.0f, 0.60f, 0.09f },  /* 26-fold, extremely intricate     */
    { "FINE-5",     5,  8.0f, 0.70f, 0.10f },  /* tight decagonal lattice          */
    { "GIANT-5",    5, 22.0f, 0.30f, 0.04f },  /* huge slow 10-pointed stars       */
    { "WEAVE-7",    7,  9.0f, 0.65f, 0.10f },  /* dense septagonal weave           */
    { "NOVA-11",   11, 20.0f, 0.35f, 0.05f },  /* big intricate 11-fold rosettes   */
};

/*
 * GlyphSet — how we turn each cell's wave value into a character (cycled by
 * g/G). This only changes what you see, never the underlying field. Each mode
 * highlights a different feature:
 *   RAMP    — the whole field as a light-to-dark gradient (the default).
 *   PEAKS   — only the crests; the bright star shapes, troughs left blank.
 *   CONTOUR — only the lines where the field crosses zero; the wave fronts.
 *   WAVES   — crests bright, troughs dim, so up and down both show.
 * N_GLYPH_SETS is just the count, used to wrap the g/G cycle.
 */
typedef enum {
    GLYPH_RAMP    = 0,
    GLYPH_PEAKS   = 1,
    GLYPH_CONTOUR = 2,
    GLYPH_WAVES   = 3,
    N_GLYPH_SETS  = 4,
} GlyphSet;

/* Characters from faint to solid, picked to read as a brightness gradient. */
static const char RAMP_GLYPHS[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };

/* How close to zero a cell must be to count as "on the wave front" for CONTOUR. */
#define CONTOUR_BAND_HALF    0.20f

/*
 * Theme — one named colour palette, switched live with t/T. Keeping it as a
 * struct means a whole palette is one table row, so adding one is a one-liner.
 *
 *   name — the label shown in the HUD.
 *   ramp — eight colours running dark to bright. These get loaded into colour
 *          pairs PAIR_RAMP_BASE+0..+7 and picked by the brightness level (0..7)
 *          of each cell, so the look comes from the gradient as a whole, not
 *          any single colour. Values are 256-colour indices; they all sit in
 *          the bright half of the cube so even dimmed cells stay readable on a
 *          black terminal.
 */
typedef struct {
    const char *name;      /* label shown in the HUD                         */
    short       ramp[8];   /* eight colours, dark to bright (256-colour indices) */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7  */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 } },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 } },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 } },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 } },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 } },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 } },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 } },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 } },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 } },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 } },
};

/* §2  PERFORMANCE — timing primitives */
/* A steady clock and a sleep. The frame-pacing that uses them lives in main. */

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

/* §3  LOGIC — pure helpers (answer depends only on the arguments, no drawing) */

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_RAMP:    return "ramp ";
    case GLYPH_PEAKS:   return "peaks";
    case GLYPH_CONTOUR: return "cntr ";
    case GLYPH_WAVES:   return "waves";
    default:            return "?    ";
    }
}

/* Step an index to the next / previous item, wrapping around the ends. Used to
 * cycle presets, themes and glyph sets. (wrap_dec adds n first so it never goes
 * negative.) */
static inline int wrap_inc(int v, int n) { return (v + 1) % n; }
static inline int wrap_dec(int v, int n) { return (v + n - 1) % n; }

/* Keep a brightness level inside the eight ramp tiers. */
static inline int clamp_level(int level)
{
    if (level < 0) return 0;
    if (level > 7) return 7;
    return level;
}

/* Bold the brightest tiers and dim the darkest so the high and low ends of the
 * pattern don't all blur into the same murky shade. */
static inline int ramp_attr(int level)
{
    if (level >= 6) return A_BOLD;
    if (level <= 1) return A_DIM;
    return A_NORMAL;
}

/* A scrambler that turns a few ints into one well-mixed number. Only used to
 * place the random twinkle stars during the reseed flash. */
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
 * Pick the character, colour level and text attribute for one cell from its
 * wave value and the chosen glyph set. Returns false when the cell should stay
 * blank (a trough in PEAKS, or anything off the wave front in CONTOUR), so the
 * caller can just skip it.
 */
static bool intensity_to_glyph(float intensity, GlyphSet g,
                               char *glyph, int *ramp_idx, int *attr)
{
    *attr = A_NORMAL;

    switch (g) {
    case GLYPH_RAMP: {
        /* whole range, low to high, mapped onto the eight ramp tiers */
        int level = clamp_level((int)((intensity + 1.0f) * 4.0f));
        *glyph = RAMP_GLYPHS[level];
        *ramp_idx = level;
        *attr = ramp_attr(level);
        return true;
    }

    case GLYPH_PEAKS: {
        /* crests only; troughs left blank */
        if (intensity < 0.0f) return false;
        int level = clamp_level((int)(intensity * 8.0f));
        *glyph = RAMP_GLYPHS[level];
        *ramp_idx = level;
        *attr = ramp_attr(level);
        return true;
    }

    case GLYPH_CONTOUR: {
        /* only cells near zero; the closer to zero, the brighter */
        float dist = fabsf(intensity);
        if (dist > CONTOUR_BAND_HALF) return false;
        float strength = 1.0f - dist / CONTOUR_BAND_HALF;     /* 1 right on the front */
        int level = clamp_level((int)(strength * 8.0f));
        *glyph = (strength > 0.65f) ? '*' : (strength > 0.30f) ? '.' : '`';
        *ramp_idx = level;
        if (level >= 6) *attr = A_BOLD;   /* don't dim the faint ones, just leave them plain */
        return true;
    }

    case GLYPH_WAVES: {
        /* crests bright, troughs dim */
        if (intensity > 0.55f) {
            *glyph = '#'; *ramp_idx = 7; *attr = A_BOLD;
        } else if (intensity > 0.15f) {
            *glyph = 'o'; *ramp_idx = 5;
        } else if (intensity > -0.15f) {
            *glyph = '.'; *ramp_idx = 3; *attr = A_DIM;
        } else if (intensity > -0.55f) {
            *glyph = ','; *ramp_idx = 2; *attr = A_DIM;
        } else {
            *glyph = '`'; *ramp_idx = 1; *attr = A_DIM;
        }
        return true;
    }

    case N_GLYPH_SETS:
        return false;
    }
    return false;
}

/* §4  SIMULATION — the wave directions, the field sampler, and the Scene */
/* This is the only part that changes over time: the drift clock ticks forward */
/* and the reseed flash fades. waves_init rebuilds the directions when you pick */
/* a new preset.                                                               */

/*
 * WaveVectors — the directions the waves travel in. There are `count` of them,
 * spread evenly over a half-circle, and each is stored as the (cos, sin) of its
 * angle. This is the heart of the whole effect: summing waves from these
 * directions is what produces the quasicrystal, and choosing a count the normal
 * crystal symmetries can't match is what makes the pattern never repeat.
 *
 * We work them out once when the preset changes so the per-cell sampler never
 * has to call cos/sin on the angles again. They're kept as two parallel arrays
 * (rather than an array of pairs) so the inner loop reads them straight through
 * memory — this gets touched once per wave for every cell on screen.
 *
 *   count                  — how many directions are live (at most N_WAVES_MAX).
 *   cos_theta / sin_theta  — the x and y parts of each direction. Only entries
 *                            0..count-1 are valid; the rest are leftover junk.
 */
typedef struct {
    int   count;                    /* how many directions are in use */
    float cos_theta[N_WAVES_MAX];   /* x part of each direction (0..count-1) */
    float sin_theta[N_WAVES_MAX];   /* y part of each direction (0..count-1) */
} WaveVectors;

/* Lay out `n` directions evenly across a half-circle. */
static void waves_init(WaveVectors *w, int n)
{
    if (n < 1) n = 1;
    if (n > N_WAVES_MAX) n = N_WAVES_MAX;
    w->count = n;
    for (int k = 0; k < n; k++) {
        float angle = (float)k * (float)M_PI / (float)n;
        w->cos_theta[k] = cosf(angle);
        w->sin_theta[k] = sinf(angle);
    }
}

/*
 * Add up all the waves at one cell and return the combined height, which lands
 * roughly between -1 (everything cancels) and +1 (everything lines up). For
 * each wave we measure how far the cell sits along that wave's direction, turn
 * that distance plus the wave's drifting phase into a cosine, and average.
 *
 * The y coordinate is stretched by ASPECT_Y_F first because terminal cells are
 * about twice as tall as they are wide; without it the round stars come out as
 * ovals. freq, rate_base and rate_delta all come from the current preset.
 */
static float compute_intensity(const WaveVectors *w, int sx, int sy, float t,
                               float freq, float rate_base, float rate_delta)
{
    float fx = (float)sx;
    float fy = (float)sy * ASPECT_Y_F;
    float sum = 0.0f;
    for (int k = 0; k < w->count; k++) {
        float wx    = fx * w->cos_theta[k] + fy * w->sin_theta[k];
        float phase = t * (rate_base + (float)k * rate_delta);
        sum += cosf(freq * wx + phase);
    }
    return sum / (float)w->count;
}

/*
 * Scene — everything about the running animation in one place: which pattern,
 * how it's drawn, and where we are in the drift. The drawing and HUD code only
 * read it; the handful of update functions below are the only ones that change
 * it.
 */
typedef struct {
    /* the wave directions we're summing */
    WaveVectors waves;

    /* what the user has chosen */
    int      current_preset;     /* which row of presets[] is active */
    int      speed;              /* drift speed (+/- keys)           */

    /* where we are in the animation */
    float    time_secs;          /* the drift clock                  */
    float    phase_offset;       /* random nudge added by 'r'        */
    bool     paused;             /* true = drift is frozen           */
    float    flash_t;            /* reseed twinkle, fades to 0       */

    /* look-only choices, toggled by keys */
    GlyphSet current_glyph;      /* which glyph set is active        */
    int      current_theme;      /* which colour theme is active     */
} Scene;

static void scene_pattern_changed(Scene *s)
{
    waves_init(&s->waves, presets[s->current_preset].n_waves);
}

static void scene_reseed(Scene *s)
{
    /* Jump the phase to a random spot so the pattern snaps to a fresh look. */
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->phase_offset * 100.0f), 0xDECAF);
    s->phase_offset = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->flash_t = 1.0f;
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_preset  = 2;          /* PENROSE-5 — the classic 10-pointed look */
    s->current_glyph   = GLYPH_RAMP;
    s->phase_offset    = 0.0f;
    scene_pattern_changed(s);
}

/*
 * One step of the animation: fade the flash a little, and (unless paused) push
 * the drift clock forward. The speed setting just scales how much the clock
 * moves, so faster/slower is the same maths with a bigger/smaller step.
 */
static void scene_tick(Scene *s, float dt)
{
    s->flash_t *= expf(-FLASH_DECAY_RATE * dt);
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* §5  RENDER — turn the Scene into characters on screen (read-only) */
/* Colours, terminal size, then drawing the field, the flash and the HUD.    */

/* Load a theme's eight gradient colours into the ramp colour pairs. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/*
 * Screen — just the terminal's current size in character cells. The drawing
 * code takes this instead of the whole program state, so it only ever sees the
 * geometry it needs. We refill it at startup and again on every window resize.
 */
typedef struct {
    int cols, rows;   /* terminal width / height in cells */
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

/*
 * Draw the whole pattern: walk every cell, work out its wave value, and ink it
 * with the active glyph set. We measure cells from the middle of the screen so
 * the centre of symmetry sits dead centre.
 */
static void draw_field(const Screen *sc, const Scene *s)
{
    int top = 2, bottom = sc->rows - 1;
    const Preset *ps = &presets[s->current_preset];
    float freq = 2.0f * (float)M_PI / ps->wavelength;   /* wavelength -> wave frequency */
    float t    = s->time_secs + s->phase_offset;
    int   cx   = sc->cols / 2;
    int   cy   = (top + bottom) / 2;

    for (int sy = top; sy < bottom; sy++) {
        int rel_y = sy - cy;
        for (int sx = 0; sx < sc->cols; sx++) {
            int rel_x = sx - cx;
            float intensity = compute_intensity(&s->waves, rel_x, rel_y, t,
                                                freq, ps->rate_base,
                                                ps->rate_delta);

            char glyph;
            int  ramp_idx, attr;
            if (!intensity_to_glyph(intensity, s->current_glyph,
                                    &glyph, &ramp_idx, &attr)) continue;

            int pair = PAIR_RAMP_BASE + ramp_idx;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/*
 * Sprinkle a few twinkle stars on top right after 'r' is pressed. We test a
 * sparse scatter of cells and mix the clock into the test so the stars shift
 * frame to frame, giving the snap a brief sparkle.
 */
static void draw_reseed_flash(const Screen *sc, const Scene *s)
{
    int seed = (int)(s->time_secs * 1000.0f);
    attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    for (int sy = 2; sy < sc->rows - 1; sy += 2) {
        for (int sx = 0; sx < sc->cols; sx += 2) {
            if (((sx ^ sy ^ seed) & 7) == 0)
                mvaddch(sy, sx, '*');
        }
    }
    attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
}

/* Draw the pattern, then lay the twinkle on top while the flash is still bright. */
static void scene_draw(const Screen *sc, const Scene *s)
{
    draw_field(sc, s);
    if (s->flash_t > FLASH_VISIBLE_MIN)
        draw_reseed_flash(sc, s);
}

/* Top row: title and current preset on the left, fps / state / speed on the right. */
static void hud_draw_status_line(const Screen *sc, const Scene *s,
                                 double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED" : "DRIFT ";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    mvprintw(0, 1, " QUASICRYSTAL  %2d/%d %-9s ",
             s->current_preset + 1, N_PRESETS,
             presets[s->current_preset].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Second row: the key-controlled settings — glyph mode, theme, a live colour
 * swatch, the wave count, and the current drift phase. Each field advances `x`
 * past its own width so the next one lands cleanly after it. */
static void hud_draw_param_line(const Screen *sc, const Scene *s)
{
    (void)sc;
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", glyph_set_name(s->current_glyph));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    /* Colour swatch — show each of the eight gradient tiers in its own colour. */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    for (int i = 0; i < 8; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)RAMP_GLYPHS[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  N=%d  phase:%5.2f ",
             presets[s->current_preset].n_waves,
             (double)(s->time_secs + s->phase_offset));
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row: the full list of keys you can press. */
static void hud_draw_key_hints(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  +/-:drift  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);
    hud_draw_status_line(sc, s, fps, sim_fps);
    hud_draw_param_line(sc, s);
    hud_draw_key_hints(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* §6  APP — signals, resize, keyboard, and the main loop */

/*
 * App — the whole running program in one struct: the animation, the screen,
 * and the loop's bookkeeping.
 *
 *   scene / screen — the animation and where it draws.
 *   sim_fps        — how many steps per second the drift takes ([ and ] keys).
 *   running        — set to 0 to quit; the signal handlers clear it.
 *   need_resize    — set to 1 when the window changed; handled before the next
 *                    frame.
 * running and need_resize are written from signal handlers, so they're
 * volatile sig_atomic_t — the one type a handler is allowed to touch, and the
 * "volatile" stops the compiler from caching the flag and never noticing it.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;      /* drift steps per second ([ / ] keys) */
    volatile sig_atomic_t running;      /* 0 = quit (set by SIGINT/SIGTERM)    */
    volatile sig_atomic_t need_resize;  /* 1 = window changed (set by SIGWINCH) */
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

/* Act on one keypress. Returns false when the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                               break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
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
        s->current_theme = wrap_inc(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = wrap_dec(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_preset = wrap_inc(s->current_preset, N_PRESETS);
        scene_pattern_changed(s);
        break;
    case 'p': case 'P':
        s->current_preset = wrap_dec(s->current_preset, N_PRESETS);
        scene_pattern_changed(s);
        break;

    case 'g':
        s->current_glyph = (GlyphSet)wrap_inc((int)s->current_glyph, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)wrap_dec((int)s->current_glyph, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/* Everything that happens once at startup: seed randomness, hook up the signal
 * handlers and the cleanup-on-exit, set the starting state, then open the
 * screen and the scene. */
static void app_init(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

int main(void)
{
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;   /* real time owed to the sim but not yet stepped */
    int64_t fps_accum   = 0;   /* time gathered for the current fps reading      */
    int     frame_count = 0;
    double  fps_display = 0.0;

    const int64_t max_dt_ns    = (int64_t)MAX_FRAME_DT_MS * NS_PER_MS;
    const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

    while (app->running) {

        /* Handle a window resize before doing anything timed. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* How much real time passed since last frame, capped so a long stall
         * doesn't make the sim try to replay seconds of catch-up at once. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > max_dt_ns) dt = max_dt_ns;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* Step the drift in fixed-size chunks, as many as the time we banked
         * allows. Fixed steps keep the animation speed the same on any machine. */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* Update the fps reading, then sleep so we don't redraw too fast. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(frame_cap_ns - elapsed);

        /* Draw the frame. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* Read one keypress, if any. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
