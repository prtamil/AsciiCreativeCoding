/*
 * fire.c  —  ncurses ASCII fire, Doom-style cellular automaton
 *
 * Algorithm:
 *   The screen is a 2-D heat grid [rows × cols].
 *   Bottom row is held at MAX_HEAT (the fuel line).
 *   Each tick, every cell is updated:
 *     heat[y][x] = (heat[y+1][x-1..x+1] summed + random neighbour) / 4
 *                  - decay
 *   Heat diffuses upward, spreads sideways, cools as it rises.
 *   This produces natural flame shapes — tall spires, short embers,
 *   flickering tongues — all emerging from one rule.
 *
 * Rendering pipeline (physics → display):
 *   heat[]  — raw float [0, MAX_HEAT]        (physics, never touched by display)
 *   Floyd-Steinberg dithering on the heat grid
 *   Perceptual LUT  — map heat float to char index
 *   Color LUT       — map char index to color pair (theme-driven)
 *   terminal        — mvaddch + attron
 *
 * Themes cycle automatically every CYCLE_FRAMES frames:
 *   fire    — classic red/orange/yellow
 *   ice     — deep blue / cyan / white
 *   plasma  — purple / magenta / white
 *   nova    — dark green / yellow-green / white
 *   poison  — dark green / bright green / white
 *   gold    — dark amber / bright yellow / white
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   ]  [      simulation faster / slower
 *   t         skip to next theme immediately
 *   g  G      gravity (fuel intensity) up / down
 *   w  W      wind right / left (shifts fuel row each tick)
 *   0         calm — no wind
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fire.c -o fire -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  theme  — 6 color palettes, Floyd-Steinberg + LUT pipeline
 *   §4  grid   — heat CA + dithered render
 *   §5  scene  — owns grid + theme state + wind
 *   §6  screen — single stdscr, ncurses internal double buffer
 *   §7  app    — dt loop, input, resize, cleanup
 */

#define _POSIX_C_SOURCE 200809L

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

    /* How many sim ticks before auto-cycling to next theme */
    CYCLE_TICKS      = 300,
};

/* Heat range — physics lives in [0, MAX_HEAT] */
#define MAX_HEAT      1.0f

/* Decay per tick — strong variance creates ragged spire tops */
#define DECAY_BASE    0.040f
#define DECAY_RAND    0.060f   /* wide random range = irregular flame height */

/* Wind max offset per tick (cells/tick) */
#define WIND_MAX      3

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(f)    (NS_PER_SEC/(f))

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
    struct timespec r = { (time_t)(ns/NS_PER_SEC),(long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  theme + rendering pipeline                                         */
/* ===================================================================== */

/*
 * Ramp — ASCII characters ordered by visual ink density (dark → bright).
 * These are the display levels the dithering quantises to.
 *
 * Chosen for fire:
 *   ' '  0%  — cold / empty
 *   '.'  5%  — faint ember glow
 *   ':'  14% — low flame
 *   '+'  28% — mid flame
 *   'x'  45% — hot mid
 *   '*'  52% — bright flame body
 *   'X'  60% — intense
 *   '#'  68% — near-peak heat
 *   '@'  80% — peak heat / core
 *
 * 9 levels. The dithering + LUT will use these.
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1)   /* 9 */

/*
 * LUT breaks — gamma-corrected intensity thresholds for each ramp char.
 * Gamma correction: raw heat [0,1] → pow(heat, 1/2.2) before LUT lookup.
 * The breaks are clustered in the 0.3–0.75 range where flame curvature
 * is most visible, giving smooth gradients in the main body of the fire.
 */
static const float k_lut_breaks[RAMP_N] = {
    0.000f,  /* ' '  cold           */
    0.080f,  /* '.'  ember          */
    0.180f,  /* ':'  low flame      */
    0.290f,  /* '+'  mid-low        */
    0.390f,  /* 'x'  mid            */
    0.500f,  /* '*'  mid-high       */
    0.620f,  /* 'X'  hot            */
    0.750f,  /* '#'  very hot       */
    0.900f,  /* '@'  core           */
};

static int lut_index(float v)
{
    /* Highest level whose threshold v meets or exceeds */
    int idx = 0;
    for (int i = RAMP_N - 1; i >= 0; i--) {
        if (v >= k_lut_breaks[i]) { idx = i; break; }
    }
    return idx;
}

static float lut_midpoint(int idx)
{
    if (idx <= 0)         return 0.f;
    if (idx >= RAMP_N-1)  return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx+1]) * 0.5f;
}

/*
 * Theme — 9 color pairs, one per ramp level (cold → hot).
 *
 * Design principle: the coldest levels (0-2) are near-black / very dim
 * so the fire appears to emerge from darkness. The hot levels (7-8) are
 * always bright white-ish so the core burns visually.
 *
 * 256-color: precise gradient using xterm color indices.
 * 8-color:   approximate using standard COLOR_* + A_DIM/A_BOLD.
 */
typedef struct {
    const char *name;
    int         fg256[RAMP_N];  /* xterm-256 index per ramp level       */
    int         fg8[RAMP_N];    /* standard COLOR_* fallback             */
    attr_t      attr8[RAMP_N];  /* A_DIM / A_BOLD for 8-color           */
} FireTheme;

/* Color pair IDs: 1..RAMP_N (pair 0 is reserved by ncurses) */
#define CP_BASE 1

static const FireTheme k_themes[] = {
    {   /* 0  fire — classic Doom fire */
        "fire",
        {  232, 52, 88, 124, 160, 196, 202, 214, 231 },
        {  COLOR_BLACK, COLOR_RED,    COLOR_RED,    COLOR_RED,
           COLOR_RED,   COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
           COLOR_WHITE  },
        {  A_NORMAL, A_DIM, A_NORMAL, A_NORMAL,
           A_BOLD,   A_DIM, A_NORMAL, A_BOLD, A_BOLD }
    },
    {   /* 1  ice — cold blue/cyan fire */
        "ice",
        {  232, 17, 19, 21, 27, 33, 51, 123, 231 },
        {  COLOR_BLACK, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
           COLOR_CYAN,  COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
           COLOR_WHITE  },
        {  A_NORMAL, A_DIM, A_NORMAL, A_BOLD,
           A_DIM,    A_NORMAL, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 2  plasma — purple/magenta */
        "plasma",
        {  232, 53, 54, 91, 93, 129, 165, 201, 231 },
        {  COLOR_BLACK,   COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
           COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE,
           COLOR_WHITE    },
        {  A_NORMAL, A_DIM, A_NORMAL, A_NORMAL,
           A_BOLD,   A_BOLD, A_DIM,   A_NORMAL, A_BOLD }
    },
    {   /* 3  nova — dark → lime green */
        "nova",
        {  232, 22, 28, 34, 40, 46, 82, 118, 231 },
        {  COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
           COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
           COLOR_WHITE  },
        {  A_NORMAL, A_DIM, A_DIM, A_NORMAL,
           A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 4  poison — sickly green/yellow */
        "poison",
        {  232, 22, 58, 64, 70, 76, 154, 190, 231 },
        {  COLOR_BLACK,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
           COLOR_WHITE   },
        {  A_NORMAL, A_DIM, A_NORMAL, A_BOLD,
           A_DIM,    A_NORMAL, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 5  gold — amber to bright yellow */
        "gold",
        {  232, 94, 130, 136, 172, 178, 214, 220, 231 },
        {  COLOR_BLACK,  COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
           COLOR_WHITE   },
        {  A_NORMAL, A_DIM, A_NORMAL, A_NORMAL,
           A_BOLD,   A_BOLD, A_BOLD,  A_BOLD, A_BOLD }
    },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/*
 * theme_apply() — register color pairs for the active theme.
 * Pairs CP_BASE .. CP_BASE+RAMP_N-1 map to ramp levels 0..RAMP_N-1.
 * Safe to call at any time — takes effect next rendered frame.
 */
static void theme_apply(int t)
{
    const FireTheme *th = &k_themes[t];
    for (int i = 0; i < RAMP_N; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
        else
            init_pair(CP_BASE + i, th->fg8[i],   COLOR_BLACK);
    }
}

static void color_init(int theme)
{
    start_color();
    theme_apply(theme);
}

/*
 * ramp_attr() — ncurses attribute for ramp level i.
 * In 256-color mode just COLOR_PAIR + A_BOLD for the hot end.
 * In 8-color mode the theme supplies per-level A_DIM/A_BOLD.
 */
static attr_t ramp_attr(int i, int theme)
{
    attr_t a = COLOR_PAIR(CP_BASE + i);
    if (COLORS >= 256) {
        if (i >= RAMP_N - 2) a |= A_BOLD;
    } else {
        a |= k_themes[theme].attr8[i];
    }
    return a;
}

/* ===================================================================== */
/* §4  grid  — heat CA + dithered render                                  */
/* ===================================================================== */

/*
 * Grid — the heat simulation.
 *
 * heat[y * cols + x]  raw float [0, MAX_HEAT]
 * fire intensity decreases going up (y=0 is top, y=rows-1 is bottom/fuel)
 *
 * Doom fire update rule (applied bottom-to-top, skipping the fuel row):
 *
 *   For each cell (x, y) from y=rows-2 down to y=0:
 *     rand_x = x + random(-1, 0, +1)   (spread left/right randomly)
 *     src    = heat[(y+1) * cols + clamp(rand_x)]
 *     decay  = DECAY_BASE + rand() * DECAY_RAND
 *     heat[y * cols + x] = max(0, src - decay)
 *
 * The bottom row (y = rows-1) is the fuel row — held at MAX_HEAT
 * weighted by a fuel_intensity that can be varied (the 'gravity' control).
 *
 * Wind: the fuel row is cyclically shifted by wind_offset each tick,
 * making the flame lean left or right.
 */

typedef struct {
    float *heat;      /* [rows * cols] current heat                    */
    float *prev_heat; /* [rows * cols] heat last drawn frame            */
    float *dither;    /* [rows * cols] working dither buffer            */
    int    cols;
    int    rows;
    float  fuel;      /* fuel intensity [0, 1] — controls flame height  */
    int    wind;      /* wind offset in cells/tick (-WIND_MAX..+WIND_MAX)*/
    int    wind_acc;  /* accumulated wind offset for fuel row shift      */
    int    theme;
    int    cycle_tick;/* ticks since last theme change                   */
    int    warmup;    /* counts up from 0; flame builds gradually        */
} Grid;

static void grid_alloc(Grid *g, int cols, int rows)
{
    g->cols      = cols;
    g->rows      = rows;
    g->heat      = calloc((size_t)(cols * rows), sizeof(float));
    g->prev_heat = calloc((size_t)(cols * rows), sizeof(float));
    g->dither    = calloc((size_t)(cols * rows), sizeof(float));
}

static void grid_free(Grid *g)
{
    free(g->heat);
    free(g->prev_heat);
    free(g->dither);
    memset(g, 0, sizeof *g);
}

static void grid_resize(Grid *g, int cols, int rows)
{
    grid_free(g);
    grid_alloc(g, cols, rows);
}

static void grid_init(Grid *g, int cols, int rows, int theme);

static void grid_init(Grid *g, int cols, int rows, int theme)
{
    grid_alloc(g, cols, rows);
    g->fuel       = 1.0f;
    g->wind       = 0;
    g->wind_acc   = 0;
    g->theme      = theme;
    g->cycle_tick = 0;
    g->warmup     = 0;
}

/*
 * grid_tick() — one CA step.
 * Returns true if the theme just cycled.
 *
 * Fuel row seeding — arch-shaped:
 *
 *   The heat at column x is scaled by an arch envelope that is zero
 *   at the left and right edges and 1.0 at the centre.  The arch is
 *   squared to make the edges steeper — it rises quickly from zero
 *   then plateaus across the middle, giving the bonfire dome silhouette.
 *
 *   A 10% margin on each side keeps the flame away from the terminal
 *   edges so it reads as free-floating rather than edge-constrained.
 *
 *   warmup multiplier: clamps the arch to 0→1 over the first 80 ticks
 *   so the fire builds from cold on startup or after a theme change.
 *
 *   Wind shifts the arch horizontally by wind_acc cells, making the
 *   whole dome lean left or right as a unit.
 */
static bool grid_tick(Grid *g)
{
    int   cols = g->cols;
    int   rows = g->rows;
    float *h   = g->heat;

    /* Warm-up scale: 0 → 1 over 80 ticks */
    float warmup_scale = (g->warmup < 80)
                       ? (float)g->warmup / 80.f
                       : 1.f;
    g->warmup++;

    /* Wind accumulator */
    g->wind_acc += g->wind;
    if (g->wind_acc >= cols || g->wind_acc <= -cols)
        g->wind_acc = 0;

    /* Arch fuel seeding */
    int fy = rows - 1;
    /* 10% margin each side keeps flame off terminal edges */
    float margin = cols * 0.10f;
    float span   = (float)cols - 2.f * margin;

    for (int x = 0; x < cols; x++) {
        /* Apply wind shift to the arch sampling position */
        float sx = (float)x - margin - (float)g->wind_acc;

        /* Normalised distance along arch span [0, 1] */
        float t = (span > 0.f) ? sx / span : 0.f;
        /* t < 0 or t > 1 means we are in the margin — no fuel */
        if (t < 0.f || t > 1.f) {
            h[fy * cols + x] = 0.f;
            continue;
        }

        /* Arch: map t ∈ [0,1] → distance from nearest edge ∈ [0, 0.5] */
        float edge_dist = (t < 0.5f) ? t : 1.f - t;   /* 0 at edges, 0.5 at centre */
        float arch      = edge_dist * 2.f;              /* rescale to [0, 1]         */
        arch            = arch * arch;                  /* square: steeper edges     */

        /* Small per-cell jitter keeps the fuel line organic */
        float jitter = 0.82f + 0.18f * ((float)rand() / RAND_MAX);

        h[fy * cols + x] = MAX_HEAT * g->fuel * arch * jitter * warmup_scale;
    }

    /* Propagate heat upward — Doom 3-neighbour rule, unchanged */
    for (int y = 0; y < rows - 1; y++) {
        for (int x = 0; x < cols; x++) {
            int rx = x + (rand() % 3) - 1;
            if (rx < 0)     rx = 0;
            if (rx >= cols) rx = cols - 1;

            float src   = h[(y + 1) * cols + rx];
            float decay = DECAY_BASE + ((float)rand() / RAND_MAX) * DECAY_RAND;
            float v     = src - decay;
            h[y * cols + x] = v < 0.f ? 0.f : v;
        }
    }

    /* Auto-cycle theme */
    g->cycle_tick++;
    if (g->cycle_tick >= CYCLE_TICKS) {
        g->cycle_tick = 0;
        g->warmup     = 0;   /* restart warm-up for new theme */
        g->theme      = (g->theme + 1) % THEME_COUNT;
        theme_apply(g->theme);
        return true;
    }
    return false;
}

/*
 * grid_draw() — borderless dithered fire render.
 *
 * Key design: we never call erase() and we never draw ' ' for every
 * cold cell.  That would fill the entire grid rectangle with explicit
 * spaces, making the empty area above the flames feel like a solid box.
 *
 * Instead:
 *   - Only cells that WERE hot last frame and ARE cold this frame get
 *     an explicit ' ' to erase the stale character.  Every other cold
 *     cell is silently skipped — ncurses' diff engine (doupdate) leaves
 *     those cells unchanged, which is exactly what we want.
 *   - Hot cells are drawn normally via the dither+LUT pipeline.
 *   - After drawing we swap heat ↔ prev_heat so next frame knows what
 *     was drawn.
 *
 * Result: fire rises freely from the bottom edge, spires dissolve into
 * blank terminal background at irregular heights, no visible boundary.
 */
static void grid_draw(Grid *g, int tcols, int trows)
{
    int   cols = g->cols;
    int   rows = g->rows;
    float *d   = g->dither;
    float *h   = g->heat;
    float *ph  = g->prev_heat;

    /* Gamma-correct into dither buffer; mark cold cells -1 */
    for (int i = 0; i < cols * rows; i++) {
        float v = h[i];
        if (v <= 0.f) { d[i] = -1.f; continue; }
        v = fminf(1.f, v / MAX_HEAT);
        d[i] = powf(v, 1.f / 2.2f);
    }

    /* Floyd-Steinberg dither + draw */
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int   i  = y * cols + x;
            float v  = d[i];
            int   tx = x, ty = y;
            if (tx >= tcols || ty >= trows) continue;

            if (v < 0.f) {
                /*
                 * Cold cell.  Only clear it if it was hot last frame —
                 * otherwise leave it untouched (no erase, no flicker).
                 */
                if (ph[i] > 0.f)
                    mvaddch(ty, tx, ' ');
                continue;
            }

            /* Quantise via LUT */
            int   idx = lut_index(v);
            float qv  = lut_midpoint(idx);
            float err = v - qv;

            /* Distribute error only to warm neighbours */
            if (x+1 < cols && d[i+1] >= 0.f)
                d[i+1]        += err * (7.f/16.f);
            if (y+1 < rows) {
                if (x-1 >= 0 && d[i+cols-1] >= 0.f)
                    d[i+cols-1]   += err * (3.f/16.f);
                if (d[i+cols] >= 0.f)
                    d[i+cols]     += err * (5.f/16.f);
                if (x+1 < cols && d[i+cols+1] >= 0.f)
                    d[i+cols+1]   += err * (1.f/16.f);
            }

            /* Draw character */
            attr_t attr = ramp_attr(idx, g->theme);
            attron(attr);
            mvaddch(ty, tx, (chtype)(unsigned char)k_ramp[idx]);
            attroff(attr);
        }
    }

    /* Swap heat ↔ prev_heat for next frame's diff */
    float *tmp   = g->prev_heat;
    g->prev_heat = g->heat;
    g->heat      = tmp;
    /* prev_heat now holds what was just drawn; heat is the next write target */
    memcpy(g->heat, g->prev_heat, (size_t)(cols * rows) * sizeof(float));
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid  grid;
    bool  paused;
    bool  needs_clear;   /* true after resize/theme change — do one erase */
} Scene;

static void scene_init(Scene *s, int cols, int rows, int theme)
{
    memset(s, 0, sizeof *s);
    grid_init(&s->grid, cols, rows, theme);
}

static void scene_free(Scene *s)   { grid_free(&s->grid); }
static void scene_resize(Scene *s, int cols, int rows)
{
    int   t    = s->grid.theme;
    float fuel = s->grid.fuel;
    int   wind = s->grid.wind;
    grid_resize(&s->grid, cols, rows);
    s->grid.fuel   = fuel;
    s->grid.wind   = wind;
    s->grid.theme  = t;
    s->grid.warmup = 0;   /* restart warm-up at new size */
    s->needs_clear = true;
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    grid_tick(&s->grid);
}

static void scene_draw(Scene *s, int cols, int rows)
{
    grid_draw(&s->grid, cols, rows);
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)  { (void)s; endwin(); }
static void screen_resize(Screen *s){ endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols); }

static void screen_draw(Screen *s, Scene *sc, double fps, int sfps)
{
    /* One-time full clear after resize or theme change to wipe stale
       chars. Normal frames never call erase() — fire manages its own
       cell clearing via the prev_heat diff in grid_draw. */
    if (sc->needs_clear) {
        erase();
        sc->needs_clear = false;
    }
    scene_draw(sc, s->cols, s->rows);

    /* HUD */
    const Grid *g = &sc->grid;
    char buf[HUD_COLS + 1];
    const char *wind_str = g->wind > 0 ? ">>>" : g->wind < 0 ? "<<<" : "---";
    snprintf(buf, sizeof buf,
             "%4.1f fps  [%s]  fuel:%.2f  wind:%s  sim:%d",
             fps, k_themes[g->theme].name, g->fuel, wind_str, sfps);
    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);

    attron(COLOR_PAIR(CP_BASE + 2));
    mvprintw(1, hx, "space=pause t=theme g/G=fuel w/W=wind 0=calm");
    attroff(COLOR_PAIR(CP_BASE + 2));
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int s){ (void)s; g_app.running=0; }
static void on_resize(int s){ (void)s; g_app.need_resize=1; }
static void cleanup  (void) { endwin(); }

static bool app_handle_key(App *a, int ch)
{
    Grid *g = &a->scene.grid;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': a->scene.paused = !a->scene.paused; break;

    case 't': case 'T':
        g->theme = (g->theme + 1) % THEME_COUNT;
        g->cycle_tick = 0;
        g->warmup     = 0;
        theme_apply(g->theme);
        a->scene.needs_clear = true;
        break;

    case 'g':
        g->fuel += 0.05f;
        if (g->fuel > 1.0f) g->fuel = 1.0f;
        break;
    case 'G':
        g->fuel -= 0.05f;
        if (g->fuel < 0.1f) g->fuel = 0.1f;
        break;

    case 'w':
        g->wind++;
        if (g->wind >  WIND_MAX) g->wind =  WIND_MAX;
        break;
    case 'W':
        g->wind--;
        if (g->wind < -WIND_MAX) g->wind = -WIND_MAX;
        break;
    case '0':
        g->wind = 0;
        break;

    case ']':
        a->sim_fps += SIM_FPS_STEP;
        if (a->sim_fps > SIM_FPS_MAX) a->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        a->sim_fps -= SIM_FPS_STEP;
        if (a->sim_fps < SIM_FPS_MIN) a->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT, on_exit); signal(SIGTERM, on_exit); signal(SIGWINCH, on_resize);

    App *app   = &g_app;
    app->running  = 1;
    app->sim_fps  = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0);

    int64_t ft=clock_ns(), sa=0, fa=0; int fc=0; double fpsd=0.;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            ft = clock_ns(); sa = 0;
        }

        int64_t now=clock_ns(), dt=now-ft; ft=now;
        if (dt > 100*NS_PER_MS) dt = 100*NS_PER_MS;

        int64_t tick = TICK_NS(app->sim_fps);
        sa += dt;
        while (sa >= tick) {
            scene_tick(&app->scene);
            sa -= tick;
        }
        float alpha = (float)sa / (float)tick;
        (void)alpha;

        fc++; fa += dt;
        if (fa >= FPS_UPDATE_MS*NS_PER_MS) {
            fpsd = (double)fc / ((double)fa/(double)NS_PER_SEC);
            fc=0; fa=0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t el = clock_ns()-ft+dt;
        clock_sleep_ns(NS_PER_SEC/60 - el);

        screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
