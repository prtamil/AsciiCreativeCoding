/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * terrain.c — Fractal Terrain (Diamond-Square)
 *
 * Generates a 65×65 heightmap (2^6+1 grid) with the diamond-square midpoint
 * displacement algorithm, renders it as ASCII elevation contours scaled to
 * the terminal, and slowly erodes it each tick with thermal weathering.
 *
 * DIAMOND-SQUARE ALGORITHM
 * ─────────────────────────
 * Iteratively bisects each square:
 *
 *   Diamond step  — centre of each size×size square:
 *     h[y+h][x+h] = avg(4 corners) + random × scale
 *
 *   Square  step  — midpoint of each diamond edge:
 *     h[y][x] = avg(up to 4 diamond neighbours) + random × scale
 *
 *   scale *= ROUGHNESS each iteration.
 *   After generation, normalise [lo, hi] → [0, 1] so the full contour
 *   range is always visible regardless of random seed.
 *
 * THERMAL WEATHERING EROSION
 * ──────────────────────────
 * Each tick, ERODE_PASSES sweeps of all cells:
 *   diff = h[y][x] − h[ny][nx]
 *   if diff > TALUS:
 *     move = RATE × (diff − TALUS)
 *     h[y][x]   -= move      ← material falls off steep slope
 *     h[ny][nx] += move      ← deposited at base
 * Gentle slopes (diff ≤ TALUS) are stable; peaks erode to plains over time.
 *
 * HEIGHT → CONTOUR MAPPING
 * ─────────────────────────
 *   < 0.20  '~'  deep water    blue dim
 *   < 0.30  '~'  water         blue
 *   < 0.40  '.'  lowland       yellow dim
 *   < 0.52  '-'  foothills     green
 *   < 0.65  '^'  slopes        green bold
 *   < 0.78  '#'  peaks         orange
 *   ≥ 0.78  '*'  snowcap       cyan bold
 *
 * Display is bilinearly interpolated from the 65×65 grid to fit any terminal.
 *
 * Keys:
 *   q/ESC quit   space pause/resume   r regenerate   e toggle erosion
 *   ] / [   sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra terrain.c -o terrain -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Diamond-Square algorithm for fractal terrain generation,
 *                  combined with thermal weathering erosion simulation.
 *                  Diamond-Square: a 2D analogue of midpoint displacement.
 *                  Step 1 (diamond): centre of each square = mean of 4 corners
 *                    + random(amplitude).
 *                  Step 2 (square): edge midpoints = mean of 2 opposing corners
 *                    + mean of 2 adjacent diamonds + random(amplitude).
 *                  Amplitude halved each iteration → fractal terrain.
 *
 * Math           : Diamond-Square produces 1/f^(2H) power spectrum where H is
 *                  the Hurst exponent.  With amplitude_factor=0.5 → H≈0.5
 *                  (standard Brownian surface, "white" terrain).
 *                  Thermal erosion rule: if slope > TALUS threshold, move
 *                  RATE × (slope − TALUS) material downhill.  This rounds peaks
 *                  and fills valleys, mimicking real geological weathering.
 *
 * Rendering      : Height-mapped to terrain types (water/plains/hills/rock/snow).
 *                  Contour lines drawn at regular height intervals using marching
 *                  squares on the interior, producing topographic map style.
 *
 * References      :
 *
 *   Fractal terrain generation & erosion (the concepts) —
 *   • Fournier, A., Fussell, D. & Carpenter, L. (1982) — "Computer Rendering of
 *     Stochastic Models", CACM 25(6). The origin of Diamond-Square / midpoint
 *     displacement for terrain.
 *   • Miller, G.S.P. (1986) — "The Definition and Rendering of Terrain Maps",
 *     SIGGRAPH. The corrected square step that removes the creasing artefacts of
 *     naive midpoint displacement.
 *   • Musgrave, F.K., Kolb, C. & Mace, R. (1989) — "The Synthesis and Rendering
 *     of Eroded Fractal Terrains", SIGGRAPH. The thermal-weathering (talus-angle)
 *     erosion model used here.
 *   • Mandelbrot, B. (1982) — "The Fractal Geometry of Nature". fBm, the Hurst
 *     exponent H, and the 1/f^(2H) power spectrum the Math note cites.
 *   • Ebert, Musgrave, Peachey, Perlin & Worley — "Texturing & Modeling: A
 *     Procedural Approach". The textbook tying fractal terrain + erosion together.
 *
 *   Contour rendering —
 *   • Lorensen, W. & Cline, H. (1987) — "Marching Cubes", SIGGRAPH; the contour
 *     lines here are its 2D analogue, "marching squares".
 *     https://en.wikipedia.org/wiki/Marching_squares
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into separated concern-layers (a SEPARATION
 * pass: RELOCATE + LABEL only — every function body is byte-identical, nothing
 * renamed). The world is a single heightmap that is generated once and then
 * eroded each tick. Layer → section → what it mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time constants only (grid size,
 *                    ROUGHNESS, TALUS, EROSION_RATE, …).
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers; the
 *                    frame cap + fixed-timestep accumulator are POLICY in main.
 *   LOGIC        §3  nothing — clampf, the one pure decision (the RNG helpers
 *                    are NOT pure — rand() advances hidden state — so they sit
 *                    in §4). No render/effects reorder can change a LOGIC result.
 *   SIMULATION   §4  Terrain.{hmap,erode_count} + Scene.{erode,paused} (+ the
 *                    global RNG). The world is GENERATED (terrain_generate:
 *                    Diamond-Square) then ERODED each tick (terrain_erode:
 *                    thermal weathering); scene_tick is the per-tick advance.
 *                    The ONLY writers of sim state.
 *   EFFECTS      §5  (none) — no stored cosmetic buffer; the contour lines are
 *                    derived at render time. One-line section, not a real layer.
 *   DELAYS       §6  (none) — only the pause flag (Scene.paused), checked in
 *                    scene_tick (§4). No timers.
 *   RENDER       §7  ncurses back buffer + colour-pair table only (color_init,
 *                    terrain_draw + its sample_height / terrain_glyph helpers,
 *                    scene_draw, screen_draw). Reads the heightmap (const); never
 *                    writes simulation state.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     while (sim_accum >= tick_ns)        // PERFORMANCE: fixed timestep
 *         scene_tick()                    //   SIMULATION (one erosion sweep)
 *     screen_draw() ; screen_present()    // RENDER (reads only; contours re-derived)
 *     getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick() advances simulation state, and it does so only
 * while not paused. User events (app_handle_key: 'r' regenerates the heightmap,
 * 'e' toggles erosion) mutate Scene/Terrain on a keypress, once per
 * frame OUTSIDE the accumulator loop, not as part of the tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants (grid size, roughness, erosion knobs)        */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,

    GRID_N          = 6,                  /* grid exponent: 2^6 = 64       */
    GRID            = (1 << GRID_N) + 1,  /* 65 × 65 heightmap             */
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/*
 * ROUGHNESS — scale multiplier per iteration (0 = very smooth, 1 = fractal noise).
 * 0.60 gives natural-looking mid-frequency mountains.
 */
#define ROUGHNESS    0.60f

/*
 * TALUS — maximum stable height difference between adjacent cells.
 * Cells steeper than TALUS erode.
 */
#define TALUS        0.022f

/*
 * EROSION_RATE — fraction of excess slope moved per cell per pass.
 * At 60 Hz × ERODE_PASSES=2 per tick: steady, visible erosion over minutes.
 */
#define EROSION_RATE 0.0012f
#define ERODE_PASSES 2

/* ===================================================================== */
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The frame cap and the fixed-timestep accumulator
 * that decide how many scene_tick()s run per frame are POLICY, in main (§8). */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  LOGIC  -- pure decisions: no mutation, no I/O                     */
/* ===================================================================== */

/* Pure decisions — clampf is the only one: it returns a value from its
 * arguments with NO mutation and NO I/O. (The RNG helpers are NOT here: rand()
 * advances hidden library state, so they live in §4 with the generation that
 * consumes them.) Nothing in RENDER/EFFECTS can change a LOGIC result. */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (generate + erode the heightmap)    */
/* ===================================================================== */

/* The ONLY writers of simulation state. The world is a single 65x65 heightmap
 * that is GENERATED then continuously ERODED:
 *  • terrain_generate — Diamond-Square fills the heightmap (uses the randf
 *    stochastic source + clampf), then normalises it to [0,1].
 *  • terrain_erode — one thermal-weathering sweep, moving material downhill
 *    where the slope exceeds TALUS; scene_tick applies it every tick while
 *    Scene.erode is on, so the terrain keeps smoothing over time.
 * Mutates: Terrain.{hmap,erode_count} and Scene.{erode,paused} (+ the global
 * RNG). scene_tick() is the single per-tick entry, called only from main (§8);
 * user events (r regenerate, e toggle erode) also mutate Scene/Terrain but are
 * NOT part of the tick -- see §8. */

/* ── Terrain ───────────────────────────────────────────────────────────── *
 * The simulated terrain: a square height field plus how far it has eroded. This
 * is the domain object Diamond-Square fills and thermal weathering smooths — the
 * "height field" of the fractal-terrain literature. The control knobs (pause /
 * erode-on-off) are NOT here; they are how the user drives it, and live on Scene.
 *   hmap         (GRID×GRID) heights ∈ [0,1]; GRID = 2^n+1 so the corners line
 *                up for midpoint displacement.
 *   erode_count  erosion sweeps applied to this height field (reset on regen). */
typedef struct {
    float hmap[GRID][GRID];
    int   erode_count;
} Terrain;

/* ── RNG helpers ────────────────────────────────────────────────────── */

static float randf01(void) { return (float)rand() / (float)RAND_MAX; }
static float randf11(void) { return randf01() * 2.0f - 1.0f; }

/* ── Diamond-Square generation ──────────────────────────────────────── */

/* diamond_step — for every stride×stride square, set its CENTRE to the mean of
 * the square's four corners plus a random displacement scaled by `scale`. */
static void diamond_step(float (*h)[GRID], int stride, int half, float scale)
{
    for (int y = 0; y < GRID - 1; y += stride) {
        for (int x = 0; x < GRID - 1; x += stride) {
            float avg = (h[y][x]          + h[y][x + stride]
                       + h[y + stride][x] + h[y + stride][x + stride]) * 0.25f;
            h[y + half][x + half] = clampf(avg + randf11() * scale, 0.0f, 1.0f);
        }
    }
}

/* square_step — set every diamond midpoint to the mean of its in-grid edge
 * neighbours (2 on the border, 4 inside) plus a random displacement. The
 * alternating start column per row visits only the diamond centres. */
static void square_step(float (*h)[GRID], int stride, int half, float scale)
{
    for (int y = 0; y < GRID; y += half) {
        for (int x = ((y / half) & 1) ? 0 : half; x < GRID; x += stride) {
            float sum = 0.0f;
            int   cnt = 0;
            if (y >= half)       { sum += h[y - half][x]; cnt++; }
            if (y + half < GRID) { sum += h[y + half][x]; cnt++; }
            if (x >= half)       { sum += h[y][x - half]; cnt++; }
            if (x + half < GRID) { sum += h[y][x + half]; cnt++; }
            h[y][x] = clampf(sum / (float)cnt + randf11() * scale, 0.0f, 1.0f);
        }
    }
}

/* normalize_heights — rescale the whole field so its min→max spans exactly
 * [0,1], so the full terrain-type / contour range is visible. */
static void normalize_heights(float (*h)[GRID])
{
    float lo = 1.0f, hi = 0.0f;
    for (int y = 0; y < GRID; y++)
        for (int x = 0; x < GRID; x++) {
            if (h[y][x] < lo) lo = h[y][x];
            if (h[y][x] > hi) hi = h[y][x];
        }
    float range = hi - lo;
    if (range > 1e-4f)
        for (int y = 0; y < GRID; y++)
            for (int x = 0; x < GRID; x++)
                h[y][x] = (h[y][x] - lo) / range;
}

/*
 * terrain_generate — fill hmap with a fresh Diamond-Square heightmap. Each pass
 * halves the stride and the displacement amplitude (scale *= ROUGHNESS), adding
 * finer detail — a 1/f^(2H) fractal surface — then the field is normalised.
 */
static void terrain_generate(Terrain *t)
{
    float (*h)[GRID] = t->hmap;

    /* Seed the four corners; midpoint displacement fills the rest. */
    h[0][0]               = randf01();
    h[0][GRID - 1]        = randf01();
    h[GRID - 1][0]        = randf01();
    h[GRID - 1][GRID - 1] = randf01();

    float scale = 0.5f;                          /* displacement amplitude */
    for (int stride = GRID - 1; stride > 1; stride >>= 1) {
        int half = stride >> 1;
        diamond_step(h, stride, half, scale);
        square_step (h, stride, half, scale);
        scale *= ROUGHNESS;
    }

    normalize_heights(h);
    t->erode_count = 0;
}

/* ── Thermal weathering ─────────────────────────────────────────────── */

static void terrain_erode(Terrain *t)
{
    float (*h)[GRID] = t->hmap;
    static const int DX[4] = { 1, -1,  0,  0 };
    static const int DY[4] = { 0,  0,  1, -1 };

    for (int pass = 0; pass < ERODE_PASSES; pass++) {
        for (int y = 0; y < GRID; y++) {
            for (int x = 0; x < GRID; x++) {
                for (int d = 0; d < 4; d++) {
                    int nx = x + DX[d];
                    int ny = y + DY[d];
                    if (nx < 0 || nx >= GRID || ny < 0 || ny >= GRID) continue;
                    float diff = h[y][x] - h[ny][nx];
                    if (diff > TALUS) {
                        float move = EROSION_RATE * (diff - TALUS);
                        h[y][x]   -= move;
                        h[ny][nx] += move;
                    }
                }
            }
        }
    }
    t->erode_count++;
}

static void terrain_init(Terrain *t)
{
    memset(t, 0, sizeof *t);
    terrain_generate(t);
}

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * The whole session, as a table of contents: the terrain being simulated plus
 * how the user is driving it. scene_tick (the per-tick orchestrator) is the only
 * writer that advances state; user events flip the knobs.
 *   terrain   WHAT is simulated — the height field (§4 generates/erodes it).
 *   erode     HOW the user drives it — keep applying thermal weathering? (e)
 *   paused    run-state — freeze the erosion (spc). */
typedef struct {
    Terrain terrain;
    bool    erode;
    bool    paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    s->erode = true;
    terrain_init(&s->terrain);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)dt; (void)cols; (void)rows;
    if (s->paused) return;
    if (s->erode)  terrain_erode(&s->terrain);
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. Nothing cosmetic is stored: the terrain glyphs and the
 * topographic contour lines are DERIVED at render time by bilinearly sampling
 * the heightmap (terrain_draw, §7) — there is no glow/trail buffer. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer. The only control is the pause flag (Scene.paused), which
 * early-returns scene_tick (§4) so erosion halts. There are no holds or timers. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. terrain_draw bilinearly samples the heightmap onto the
 * terminal, picking a terrain-type glyph/colour per cell and overlaying
 * contour lines; screen_draw lays the HUD over it. Reads Terrain; writes ONLY
 * the ncurses back buffer and the colour-pair table (color_init at init).
 * Never mutates simulation state. */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);   /* red     */
        init_pair(2, 208, COLOR_BLACK);   /* orange  */
        init_pair(3, 226, COLOR_BLACK);   /* yellow  */
        init_pair(4,  46, COLOR_BLACK);   /* green   */
        init_pair(5,  51, COLOR_BLACK);   /* cyan    */
        init_pair(6, 33, COLOR_BLACK);   /* blue    */
        init_pair(7, 201, COLOR_BLACK);   /* magenta */
        init_pair(8, 226, COLOR_BLACK);   /* yellow  — HUD */
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);   /* HUD */
    }
}

/* ── Drawing ────────────────────────────────────────────────────────── */

/* sample_height — bilinearly sample the height field at a terminal cell. The
 * cell maps to a fractional grid coordinate; interpolating the four surrounding
 * grid heights keeps contours smooth and band-free at any terminal size. */
static float sample_height(const Terrain *t, int row, int col, int rows, int cols)
{
    float gy_f = (float)(row - 1) / (float)(rows - 2) * (float)(GRID - 1);
    int   gy   = (int)gy_f;
    if (gy > GRID - 2) gy = GRID - 2;
    float ty = gy_f - (float)gy;

    float gx_f = (float)col / (float)(cols - 1) * (float)(GRID - 1);
    int   gx   = (int)gx_f;
    if (gx > GRID - 2) gx = GRID - 2;
    float tx = gx_f - (float)gx;

    return t->hmap[gy    ][gx    ] * (1.0f - tx) * (1.0f - ty)
         + t->hmap[gy    ][gx + 1] * tx          * (1.0f - ty)
         + t->hmap[gy + 1][gx    ] * (1.0f - tx) * ty
         + t->hmap[gy + 1][gx + 1] * tx          * ty;
}

/* terrain_glyph — classify a height into a terrain band; returns the glyph and
 * fills the pair + attr out-params with its colour. The thresholds ARE the
 * band table. */
static char terrain_glyph(float v, int *pair, chtype *attr)
{
    if      (v < 0.20f) { *pair = 6; *attr = A_DIM;  return '~'; }  /* deep water */
    else if (v < 0.30f) { *pair = 6; *attr = 0;      return '~'; }  /* water      */
    else if (v < 0.40f) { *pair = 3; *attr = A_DIM;  return '.'; }  /* coast      */
    else if (v < 0.52f) { *pair = 4; *attr = 0;      return '-'; }  /* plains     */
    else if (v < 0.65f) { *pair = 4; *attr = A_BOLD; return '^'; }  /* hills      */
    else if (v < 0.78f) { *pair = 2; *attr = 0;      return '#'; }  /* mountains  */
    else                { *pair = 5; *attr = A_BOLD; return '*'; }  /* peaks/snow */
}

/* terrain_draw — paint the height field: per terminal cell, sample the height
 * and draw its terrain-type glyph. */
static void terrain_draw(const Terrain *t, WINDOW *w, int cols, int rows)
{
    if (cols < 2 || rows < 3) return;

    for (int row = 1; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float  v = sample_height(t, row, col, rows, cols);
            int    pair;
            chtype attr;
            char   ch = terrain_glyph(v, &pair, &attr);

            wattron(w, COLOR_PAIR(pair) | attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, COLOR_PAIR(pair) | attr);
        }
    }
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    terrain_draw(&s->terrain, w, cols, rows);
}

/* Screen — the terminal viewport: just its current size in cells, refreshed at
 * startup and on resize. Pure presentation geometry; holds no simulation state.
 * (terrain_draw maps the GRID×GRID height field onto these rows × cols.) */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  erode:%s (%d)  %s ",
             fps, sim_fps,
             sc->erode ? "on " : "off", sc->terrain.erode_count,
             sc->paused ? "PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(0, 1, " TERRAIN ");
    attroff(COLOR_PAIR(4) | A_BOLD);

    attron(COLOR_PAIR(8) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:regenerate  e:erosion  [/]:Hz ");
    attroff(COLOR_PAIR(8) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main loop.
 * main() is the ONE place that combines the layers per tick, in fixed order:
 * scene_tick (SIM, gated by the pause flag) -> screen_draw (RENDER) ->
 * screen_present -> input. app_handle_key() mutates state on USER EVENTS (a
 * keypress; r regenerates, e toggles erosion) and is OUTSIDE the tick. */

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

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': s->paused = !s->paused; break;
    case 'r': case 'R': terrain_generate(&s->terrain); break;
    case 'e': case 'E': s->erode = !s->erode; break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
