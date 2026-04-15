/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * newfire.c  —  aalib aafire algorithm, ncurses framework
 *
 * This is a faithful port of the aafire algorithm from aalib, rewritten
 * without the aalib dependency and rendered through the same dithering +
 * LUT + theme pipeline as fire.c.
 *
 * Algorithm (from aafire.c, aalib 1.4):
 *
 *   FUEL ROW SEEDING (drawfire):
 *     Two rows at the bottom are seeded each tick.
 *     The seed pattern uses a sweeping arch — i1 counts up from 1 and
 *     i2 counts down from 4*cols+1 — making the flame taller in the
 *     centre and shorter at the edges.
 *     Within each "burst" of 6 cells, the heat value walks randomly
 *     ±2 from a base that grows with `height` (frame counter), so the
 *     fire "warms up" from a cold start over many frames.
 *     This produces the distinctive aafire silhouette: tall rounded arch,
 *     not a flat bottom-edge like Doom fire.
 *
 *   PROPAGATION (firemain):
 *     Each cell is updated from 5 neighbours:
 *       3 from one row below:  (x-1, y+1)  (x, y+1)  (x+1, y+1)
 *       2 from two rows below: (x-1, y+2)  (x+1, y+2)
 *     Sum those 5 values, run through the decay table.
 *     Decay table: table[i] = max(0, (i - minus) / 5)
 *     where minus = 800 / rows (scales with screen height).
 *     Dividing by 5 = averaging the 5 neighbours.
 *     Subtracting minus = cooling per row.
 *
 *   Why it looks different from Doom fire:
 *     Doom uses 3 neighbours (one row below, random horizontal spread).
 *     aafire uses 5 neighbours including two rows below — this gives more
 *     vertical inertia, rounder blob-like flames instead of sharp spires.
 *     The arch-shaped fuel row makes a dome profile instead of a flat base.
 *
 * Rendering pipeline (identical to fire.c):
 *   bitmap[y][x]  uint8 [0..255]       (physics, 256 heat levels)
 *   gamma correction: v = pow(v/255, 1/2.2)
 *   Floyd-Steinberg dithering
 *   Perceptual LUT: heat float → ramp index
 *   Color pairs: ramp index → theme color
 *   terminal: mvaddch + attron
 *
 * Themes cycle every CYCLE_TICKS ticks (same 6 as fire.c):
 *   fire / ice / plasma / nova / poison / gold
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   ]  [      faster / slower
 *   t         next theme
 *   g  G      fuel intensity up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra newfire.c -o newfire -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  theme + LUT + dithering pipeline  (shared with fire.c)
 *   §4  bitmap  — uint8 heat buffer + aalib CA
 *   §5  scene
 *   §6  screen  — single stdscr
 *   §7  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : aafire cellular automaton (Olszak 1999, aalib 1.4).
 *                  Each cell updated from 5 neighbours (3 from one row below,
 *                  2 from two rows below) vs Doom's 3-neighbour approach.
 *                  Decay table: table[i] = max(0, (i − 800/rows) / 5).
 *                  5-neighbour average + extra vertical reach produces rounder
 *                  blob-like flames (vertical inertia) vs Doom's sharp spires.
 *
 * Physics        : Heat diffuses upward from the fuel row.  The arch-shaped
 *                  fuel seeding (sweeping i1/i2 counters) creates a tall
 *                  rounded flame dome rather than a flat bottom edge.
 *                  "Warm-up": base heat grows with frame counter `height`,
 *                  so flames start small and grow to full intensity.
 *
 * Rendering      : Floyd-Steinberg dithering on the float heat grid reduces
 *                  visible banding when mapping to the ASCII char palette.
 *                  Perceptual LUT: heat → ramp index chosen to match human
 *                  perceived brightness rather than linear heat value.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,
    HUD_COLS        = 52,
    FPS_UPDATE_MS   = 500,
    CYCLE_TICKS     = 300,   /* ticks before auto-cycling theme          */
};

/*
 * aalib uses a 256-entry uint8 bitmap.
 * MAXTABLE must cover the maximum possible sum of 5 neighbours × 255 = 1275.
 * Original: MAXTABLE = 256*5 = 1280.  We keep the same.
 */
#define MAXTABLE   1280

/* Fuel intensity scale [0.1, 1.0] — multiplied against the base heat */
#define FUEL_DEFAULT  1.0f
#define FUEL_STEP     0.05f
#define FUEL_MIN      0.1f
#define FUEL_MAX      1.0f

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC/(f))

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
/* §3  theme + LUT + dithering pipeline                                  */
/* ===================================================================== */

/*
 * Ramp — same 9-level set as fire.c.
 * ' ' = cold/background, '@' = hottest core.
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1)

static const float k_lut_breaks[RAMP_N] = {
    0.000f, 0.080f, 0.180f, 0.290f, 0.390f,
    0.500f, 0.620f, 0.750f, 0.900f,
};

static int lut_index(float v)
{
    int idx = 0;
    for (int i = RAMP_N-1; i >= 0; i--)
        if (v >= k_lut_breaks[i]) { idx = i; break; }
    return idx;
}
static float lut_midpoint(int idx)
{
    if (idx <= 0)        return 0.f;
    if (idx >= RAMP_N-1) return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx+1]) * 0.5f;
}

typedef struct {
    const char *name;
    int         fg256[RAMP_N];
    int         fg8[RAMP_N];
    attr_t      attr8[RAMP_N];
} FireTheme;

#define CP_BASE 1

static const FireTheme k_themes[] = {
    { "fire",
      { 232, 52, 88, 124, 160, 196, 202, 214, 231 },
      { COLOR_BLACK, COLOR_RED,    COLOR_RED,    COLOR_RED,
        COLOR_RED,   COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
      { A_NORMAL, A_DIM, A_NORMAL, A_NORMAL,
        A_BOLD, A_DIM, A_NORMAL, A_BOLD, A_BOLD }
    },
    { "ice",
      { 232, 17, 19, 21, 27, 33, 51, 123, 231 },
      { COLOR_BLACK, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
        COLOR_CYAN,  COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM, A_NORMAL, A_BOLD,
        A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_BOLD }
    },
    { "plasma",
      { 232, 53, 54, 91, 93, 129, 165, 201, 231 },
      { COLOR_BLACK, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM, A_NORMAL, A_NORMAL,
        A_BOLD, A_BOLD, A_DIM, A_NORMAL, A_BOLD }
    },
    { "nova",
      { 232, 22, 28, 34, 40, 46, 82, 118, 231 },
      { COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM, A_DIM, A_NORMAL,
        A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    { "poison",
      { 232, 22, 58, 64, 70, 76, 154, 190, 231 },
      { COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM, A_NORMAL, A_BOLD,
        A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_BOLD }
    },
    { "gold",
      { 232, 94, 130, 136, 172, 178, 214, 220, 231 },
      { COLOR_BLACK, COLOR_RED, COLOR_RED, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
      { A_NORMAL, A_DIM, A_NORMAL, A_NORMAL,
        A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
};
#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static void theme_apply(int t)
{
    const FireTheme *th = &k_themes[t];
    for (int i = 0; i < RAMP_N; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE+i, th->fg256[i], COLOR_BLACK);
        else
            init_pair(CP_BASE+i, th->fg8[i],   COLOR_BLACK);
    }
}
static void color_init(int theme) { start_color(); theme_apply(theme); }

static attr_t ramp_attr(int i, int theme)
{
    attr_t a = COLOR_PAIR(CP_BASE+i);
    if (COLORS >= 256) {
        if (i >= RAMP_N-2) a |= A_BOLD;
    } else {
        a |= k_themes[theme].attr8[i];
    }
    return a;
}

/* ===================================================================== */
/* §4  bitmap  — aalib heat buffer + CA                                  */
/* ===================================================================== */

/*
 * Bitmap — aalib uses a flat uint8 array of (cols × (rows+2)).
 * The extra 2 rows at the bottom are the fuel rows that get seeded.
 * `rows` here is the visible rows; `rows+2` is the full height.
 *
 * table[]  — the decay lookup table built by gentable().
 *            table[sum_of_5_neighbours] = decayed heat value.
 *
 * prev[]   — previous frame's bitmap for diff-based clearing.
 *            Only cells that went from hot to cold get erased.
 *
 * dither[] — float working buffer for Floyd-Steinberg.
 *
 * height   — frame counter; grows until fuel reaches full brightness.
 * loop     — countdown for fuel seeding bursts (original logic).
 * sloop    — counts seeding cycles.
 * fuel     — user-adjustable intensity [0.1, 1.0].
 * theme, cycle_tick — theme state.
 */
typedef struct {
    unsigned char *bmap;      /* [cols * (rows+2)] heat uint8             */
    unsigned char *prev;      /* [cols * rows]     last drawn frame        */
    float         *dither;    /* [cols * rows]     dither working buffer   */
    unsigned int   table[MAXTABLE];
    int            cols;
    int            rows;
    int            height;    /* frame counter for warm-up                 */
    int            loop;      /* fuel burst countdown                      */
    int            sloop;     /* fuel sweep counter                        */
    float          fuel;      /* intensity scale                           */
    int            theme;
    int            cycle_tick;
} Bitmap;

/*
 * gentable() — build the decay lookup table.
 *
 * aalib exact port:
 *   minus = 800 / rows  (at least 1)
 *   table[i] = (i > minus) ? (i - minus) / 5 : 0
 *
 * Interpretation:
 *   The sum of 5 neighbours ranges from 0 to 5×255=1275.
 *   Subtracting `minus` (which grows smaller as rows increases, so
 *   taller screens have less decay per row = fire burns higher).
 *   Dividing by 5 = averaging the 5 neighbours back to [0,255].
 *   The subtraction before the division is what provides cooling.
 */
static void gentable(Bitmap *b)
{
    int minus = 800 / b->rows;
    if (minus == 0) minus = 1;
    for (int i = 0; i < MAXTABLE; i++) {
        if (i > minus) {
            unsigned int p2 = (unsigned int)(i - minus) / 5;
            b->table[i] = p2;
        } else {
            b->table[i] = 0;
        }
    }
}

/*
 * firemain() — propagate heat upward through 5-neighbour averaging.
 *
 * aalib exact port of:
 *   for each cell p in [0 .. cols*rows):
 *     *p = table[ p[cols-1] + p[cols+1] + p[cols]
 *                + p[2*cols-1] + p[2*cols+1] ]
 *
 * The five source cells are all BELOW p (positive y = down):
 *   (x-1, y+1)  (x, y+1)  (x+1, y+1)   — one row below
 *   (x-1, y+2)  (x+1, y+2)              — two rows below (no centre)
 *
 * Edge cells clamp to the nearest valid column.
 * The two extra rows at the bottom ensure p+2*cols never reads garbage.
 */
static void firemain(Bitmap *b)
{
    int cols = b->cols;
    int rows = b->rows;
    unsigned char *bmap = b->bmap;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int i  = y * cols + x;
            /* clamp x neighbours */
            int xl = (x > 0)        ? x-1 : 0;
            int xr = (x < cols-1)   ? x+1 : cols-1;

            unsigned int sum =
                bmap[(y+1)*cols + xl] +
                bmap[(y+1)*cols + x ] +
                bmap[(y+1)*cols + xr] +
                bmap[(y+2)*cols + xl] +
                bmap[(y+2)*cols + xr];

            bmap[i] = (unsigned char)(b->table[sum < MAXTABLE ? sum : MAXTABLE-1]);
        }
    }
}

/*
 * drawfire() — seed the two fuel rows at the bottom.
 *
 * aalib exact port, adapted for variable cols/rows:
 *
 *   height++ — frame counter; `last1` value is clamped by min(i1,i2,height)
 *              so the fire "warms up" from a cold start over ~cols frames.
 *
 *   loop--   — countdown; when it hits 0 it resets to rand()%3 and sloop++.
 *              This introduces occasional short gaps in the fuel row.
 *
 *   i1 counts up from 1 (4 per cell), i2 counts down from 4*cols+1.
 *   min(i1, i2) produces an arch: large in the middle, small at the edges.
 *   min(..., height) clamps during warm-up so edges start cool.
 *
 *   Within each burst of up to 6 cells:
 *     last1 is set to rand() % min(i1, i2, height)
 *     then walks ±2 randomly for each cell, writing to both
 *     the fuel row and the row one below it.
 *
 *   fuel scale: we multiply last1 by b->fuel so the user can dim the fire.
 */
static void drawfire(Bitmap *b)
{
    int   cols = b->cols;
    int   rows = b->rows;
    unsigned char *bmap = b->bmap;
    unsigned char *fuel_row  = bmap + rows * cols;        /* row `rows`   */
    unsigned char *fuel_row2 = bmap + (rows+1) * cols;    /* row `rows+1` */

    b->height++;
    b->loop--;
    if (b->loop < 0) {
        b->loop  = rand() % 3;
        b->sloop++;
    }

    int i1 = 1;
    int i2 = 4 * cols + 1;

    int x = 0;
    while (x < cols) {
        /* min(i1, i2, height) — arch shape + warm-up clamp */
        int cap = i1 < i2 ? i1 : i2;
        if (b->height < cap) cap = b->height;
        if (cap < 1) cap = 1;

        int last1 = (int)((float)(rand() % cap) * b->fuel);

        /* burst of up to 6 cells (original `i` = rand()%6, counted down) */
        int burst = rand() % 6;
        for (int j = 0; j <= burst && x < cols; j++, x++, i1+=4, i2-=4) {
            /* clamp last1 to [0, 255] */
            if (last1 < 0)   last1 = 0;
            if (last1 > 255) last1 = 255;

            fuel_row[x]  = (unsigned char)last1;
            last1 += rand() % 6 - 2;
            if (last1 < 0)   last1 = 0;
            if (last1 > 255) last1 = 255;

            fuel_row2[x] = (unsigned char)last1;
            last1 += rand() % 6 - 2;
        }

        /* skip gap: advance x, i1, i2 for the remaining cells in the loop */
        x++;
        i1 += 4;
        i2 -= 4;
    }
}

/* ---- allocation ---- */

static void bitmap_alloc(Bitmap *b, int cols, int rows)
{
    b->cols   = cols;
    b->rows   = rows;
    /* +2 extra rows for the fuel rows that firemain reads from */
    b->bmap   = calloc((size_t)(cols * (rows + 2)), sizeof(unsigned char));
    b->prev   = calloc((size_t)(cols * rows),       sizeof(unsigned char));
    b->dither = calloc((size_t)(cols * rows),       sizeof(float));
}

static void bitmap_free(Bitmap *b)
{
    free(b->bmap); free(b->prev); free(b->dither);
    memset(b, 0, sizeof *b);
}

static void bitmap_init(Bitmap *b, int cols, int rows, int theme)
{
    bitmap_alloc(b, cols, rows);
    b->height     = 0;
    b->loop       = 0;
    b->sloop      = 0;
    b->fuel       = FUEL_DEFAULT;
    b->theme      = theme;
    b->cycle_tick = 0;
    gentable(b);
}

/*
 * bitmap_tick() — one simulation step: seed fuel rows then propagate.
 * Returns true if theme just cycled.
 */
static bool bitmap_tick(Bitmap *b)
{
    drawfire(b);
    firemain(b);

    b->cycle_tick++;
    if (b->cycle_tick >= CYCLE_TICKS) {
        b->cycle_tick = 0;
        b->theme = (b->theme + 1) % THEME_COUNT;
        theme_apply(b->theme);
        return true;
    }
    return false;
}

/*
 * bitmap_draw() — dithered render of the heat bitmap into stdscr.
 *
 * Same pipeline as fire.c:
 *   1. Gamma-correct uint8 heat → float [0,1] via pow(v/255, 1/2.2)
 *   2. Floyd-Steinberg dithering
 *   3. LUT → ramp char + color pair
 *
 * Cold cells only get an explicit space if they were hot last frame
 * (diff-based clearing — no erase(), no visible box boundary).
 */
static void bitmap_draw(Bitmap *b, int tcols, int trows)
{
    int    cols = b->cols;
    int    rows = b->rows;
    float *d    = b->dither;
    unsigned char *bmap = b->bmap;
    unsigned char *prev = b->prev;

    /* Gamma-correct into dither buffer */
    for (int i = 0; i < cols * rows; i++) {
        float v = (float)bmap[i] / 255.f;
        if (v <= 0.f) { d[i] = -1.f; continue; }
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
                /* Cold — only clear if was hot last frame */
                if (prev[i] > 0) mvaddch(ty, tx, ' ');
                continue;
            }

            int   idx = lut_index(v);
            float qv  = lut_midpoint(idx);
            float err = v - qv;

            if (x+1 < cols && d[i+1] >= 0.f)
                d[i+1]        += err * (7.f/16.f);
            if (y+1 < rows) {
                if (x-1 >= 0 && d[i+cols-1] >= 0.f)
                    d[i+cols-1] += err * (3.f/16.f);
                if (d[i+cols] >= 0.f)
                    d[i+cols]   += err * (5.f/16.f);
                if (x+1 < cols && d[i+cols+1] >= 0.f)
                    d[i+cols+1] += err * (1.f/16.f);
            }

            attr_t attr = ramp_attr(idx, b->theme);
            attron(attr);
            mvaddch(ty, tx, (chtype)(unsigned char)k_ramp[idx]);
            attroff(attr);
        }
    }

    /* Save this frame as prev for next frame's diff */
    memcpy(prev, bmap, (size_t)(cols * rows) * sizeof(unsigned char));
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

typedef struct {
    Bitmap bmap;
    bool   paused;
    bool   needs_clear;
} Scene;

static void scene_init(Scene *s, int cols, int rows, int theme)
{
    memset(s, 0, sizeof *s);
    bitmap_init(&s->bmap, cols, rows, theme);
}
static void scene_free(Scene *s) { bitmap_free(&s->bmap); }

static void scene_resize(Scene *s, int cols, int rows)
{
    int   t    = s->bmap.theme;
    float fuel = s->bmap.fuel;
    bitmap_free(&s->bmap);
    bitmap_init(&s->bmap, cols, rows, t);
    s->bmap.fuel   = fuel;
    s->needs_clear = true;
    gentable(&s->bmap);   /* rebuild decay table for new rows */
}

static void scene_tick(Scene *s)
{
    if (!s->paused) {
        if (bitmap_tick(&s->bmap))
            s->needs_clear = true;
    }
}

static void scene_draw(Scene *s, int cols, int rows)
{
    bitmap_draw(&s->bmap, cols, rows);
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
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols); }

static void screen_draw(Screen *s, Scene *sc, double fps, int sfps)
{
    if (sc->needs_clear) {
        erase();
        sc->needs_clear = false;
    }
    scene_draw(sc, s->cols, s->rows);

    /* HUD */
    char buf[HUD_COLS+1];
    snprintf(buf, sizeof buf,
             "%4.1f fps  [%s]  fuel:%.2f  sim:%d",
             fps, k_themes[sc->bmap.theme].name, sc->bmap.fuel, sfps);
    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD);
    attron(COLOR_PAIR(CP_BASE + 2));
    mvprintw(1, hx, "space=pause  t=theme  g/G=fuel  ]/[=speed");
    attroff(COLOR_PAIR(CP_BASE + 2));
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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
    Bitmap *b = &a->scene.bmap;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': a->scene.paused = !a->scene.paused; break;

    case 't': case 'T':
        b->theme = (b->theme + 1) % THEME_COUNT;
        b->cycle_tick = 0;
        theme_apply(b->theme);
        a->scene.needs_clear = true;
        break;

    case 'g':
        b->fuel += FUEL_STEP;
        if (b->fuel > FUEL_MAX) b->fuel = FUEL_MAX;
        break;
    case 'G':
        b->fuel -= FUEL_STEP;
        if (b->fuel < FUEL_MIN) b->fuel = FUEL_MIN;
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
    signal(SIGINT,on_exit); signal(SIGTERM,on_exit); signal(SIGWINCH,on_resize);

    App *app  = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

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
        while (sa >= tick) { scene_tick(&app->scene); sa -= tick; }
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
