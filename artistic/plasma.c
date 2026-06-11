/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * plasma.c — Demoscene Plasma / Colour Wave
 *
 * Classic demoscene plasma: sums of sinusoids at each terminal cell
 * mapped through a cycling colour palette.
 *
 *   v(col, row, t) = sin(col·f1 + t·s1)
 *                  + sin(row·f2 + t·s2)
 *                  + sin((col+row)·f3 + t·s3)
 *                  + sin(√(dx²+(2·dy)²)·f4 + t·s4)
 * Normalised to [0,1], then shifted by a phase that rotates at CYCLE_HZ
 * cycles/second — colours flow across the screen without physics.
 *
 * Four frequency presets ('f') give distinct wave structures.
 * Four colour themes ('t') select different palette mappings.
 *
 * Keys:
 *   q/ESC quit   space/p pause   t next theme   f next frequencies
 *   ] / [   sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra plasma.c -o plasma -lncurses -lm
 *
 * §1 config       §2 performance   §3 types
 * §4 logic        §5 simulation     §6 render        §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic plasma — no state grid, no simulation.
 *                  Each frame: v(col,row,t) computed as a sum of 4 sine
 *                  waves at varying spatial frequencies and time speeds.
 *                  The radial term sin(√(dx²+(2·dy)²)·f4) creates circular
 *                  ripples centred on the screen; the factor 2 on dy corrects
 *                  for terminal cell aspect ratio (cells taller than wide).
 *
 * Math           : Classic demoscene technique (Commodore 64 era, ~1990s).
 *                  Superposition of travelling sinusoidal waves produces
 *                  interference patterns.  Phase offset rotates at CYCLE_HZ
 *                  Hz, causing the colour palette to cycle continuously.
 *                  Normalise v to [0,1]: v_norm = (v + 4) / 8 (4 waves
 *                  with amplitude ≤1 each → sum ∈ [-4, 4]).
 *
 * Rendering      : v_norm mapped through a 256-entry colour palette built
 *                  from sinusoidal RGB components (each colour component a
 *                  different phase of the same frequency → smooth hue cycle).
 *                  256-colour terminal required for smooth gradients; 8-colour
 *                  fallback uses colour pair cycling.
 *
 * References     :
 *   Plasma effect & wave superposition (§4 plasma_value)
 *     [1] Vandevenne, "Lode's Computer Graphics Tutorial — Plasma"
 *         (lodev.org) — the canonical sum-of-sines plasma this file follows.
 *     [2] Hecht, "Optics" — superposition / interference of travelling sine
 *         waves, the physics the four added terms imitate.
 *   Palette cycling animation (§6 colour pairs, phase rotate)
 *     [3] Ferrari, "8-bit colour cycling" (Effect Games, 2010; technique from
 *         1990s games) — animating by rotating the palette rather than the
 *         pixels, which is exactly the CYCLE_HZ phase shift here.
 *   ASCII rendering substrate (§6 PalEntry glyphs)
 *     [4] Bourke, "Character representation of grey scale images" (1997) — the
 *         value → glyph ramp ('.' ':' '+' '*' '#' '@').
 *     [5] Padala, "NCURSES Programming HOWTO" (TLDP) — colour pairs and the
 *         erase → draw → refresh frame model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Plasma is a stateless function: give it a cell coordinate (col, row)
 * and a time t, and it returns a number v ∈ [0,1].  That number indexes
 * a palette of glyph+colour pairs.  Render one glyph per cell, every
 * frame.  No grid, no particles, no physics — every pixel is computed
 * fresh, independent of every other pixel.  The "movement" you see is
 * pure phase: the same sine waves shifted by t·speed over time, so what
 * was at value 0.3 last frame becomes 0.31 this frame, and the colour
 * mapped to 0.3 quietly slides one cell over.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture four flashlights with rippled lenses pointed at the screen.
 * One ripples horizontally, one vertically, one diagonally, one in
 * concentric circles from the centre.  Their brightness adds at each
 * cell.  Now slowly rotate each lens — same ripple pattern, different
 * orientation — and the interference pattern shimmers and shifts.
 * That's it.  No simulation; the math itself IS the animation.  This
 * is why old demoscene coders loved plasma: it ran in fixed time per
 * pixel on a Commodore 64, no buffers, no per-pixel state.
 *
 * ALGORITHM IN STEPS  (per frame, per cell)
 * ──────────────────
 *  1. Advance global time t by dt.
 *  2. For each (col, row) in the visible region:
 *       a. Compute four sinusoid terms:
 *            s1 = sin(col·f1 + t·speed1)        horizontal wave
 *            s2 = sin(row·f2 + t·speed2)        vertical wave
 *            s3 = sin((col+row)·f3 + t·speed3)  diagonal wave
 *            s4 = sin(dist·f4 + t·speed4)       radial wave
 *          where dist = sqrt(dx² + (2·dy)²) from screen centre, with
 *          the dy×2 to correct cell-aspect-ratio (terminal cells are
 *          ~2× taller than wide → circles look round).
 *       b. v = (s1+s2+s3+s4 + 4) / 8     map sum∈[-4,4] to [0,1].
 *       c. vs = (v + phase) mod 1        phase = t·CYCLE_HZ ∈ [0,1)
 *                                        cycles palette globally.
 *       d. idx = floor(vs · N_PAL)       pick palette entry.
 *       e. mvaddch(row, col, glyph) with that pair+attr.
 *  3. Draw HUD (top-right status, bottom-left hint).  Present.
 *
 * KEY FORMULAS
 * ────────────
 *  v(c,r,t) = sin(c·f1 + t·s1) + sin(r·f2 + t·s2)
 *           + sin((c+r)·f3 + t·s3) + sin(d·f4 + t·s4)
 *  d        = sqrt((c-cx)² + ((r-cy)·2)²)        aspect-corrected
 *  v_norm   = (v + 4) / 8                        sum∈[-4,4] → [0,1]
 *  phase(t) = (t · CYCLE_HZ) mod 1               palette rotates
 *  idx      = floor(((v_norm + phase) mod 1) · 14)
 *  fp.f*    spatial frequency rad/cell           gentle 0.10-0.20
 *  fp.s*    angular speed     rad/sec            grand 0.3-0.5
 *  CYCLE_HZ = 0.20            full cycle every 5s
 *
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
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

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * Three real layers; three are intentionally absent/trivial.
 *
 *   LAYER        SECTION  MUTATES
 *   ────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — clock primitives (the fixed-timestep
 *                           accumulator, frame cap and fps live in §7 main)
 *   LOGIC        §4       nothing — pure plasma_value(col,row,t,preset) → the
 *                           cell's [0,1] field value.  No mutation, no I/O.
 *   SIMULATION   §5       Plasma.time only — plasma_tick does `time += dt`.
 *                           Plasma is otherwise stateless (analytic).
 *   RENDER       §6       the terminal + colour pairs only; READS the Plasma
 *                           (time/theme/preset), never writes it.
 *
 *   EFFECTS  : ABSENT — plasma is stateless; every cell's glyph/colour is
 *              recomputed fresh from plasma_value each frame, nothing stored.
 *   DELAYS   : the only pause is Plasma.paused (plasma_tick early-returns);
 *              there are no timers or holds.
 *   No coordinate layer: plasma works directly in terminal cell space (the
 *              only correction is the ×2 on dy, inline in plasma_value).
 *
 * LOGIC (§4) is provably uncorruptable from RENDER: it does no mutation and no
 * I/O, so deleting or reordering any draw cannot change plasma_value.
 *
 * Per-tick combine order — main() (§7) is the only place that advances state:
 *     1. PERFORMANCE  measure dt (capped); fixed-timestep accumulator    §7
 *     2. SIMULATION   scene_tick(dt) per fixed step  [skipped if paused]  §5
 *     3. PERFORMANCE  fps tally + sleep to the 60 fps cap                §7
 *     4. RENDER       screen_draw() + present  (read-only)               §6
 *
 * User events (keys: pause/theme/freq/Hz; SIGWINCH resize) mutate Plasma /
 * sim_fps but are NOT part of the tick — handled after render, in the getch
 * branch and the resize block (§7).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,   /* plasma recomputes every cell; 30 is plenty */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FRAME_CAP_FPS   = 60,   /* hard render frame cap, independent of sim Hz */
    FPS_UPDATE_MS   = 500,
    N_FREQ_PRESETS  = 4,
    N_THEMES        = 4,
    N_PAL           = 14,   /* palette entries per theme                  */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CYCLE_HZ    0.20f   /* palette phase cycles per second            */

/* ===================================================================== */
/* §2  performance — timing primitives (frame cap / fixed step in §7)      */
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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  types                                                              */
/* ===================================================================== */

/* PalEntry — one stop in a plasma colour palette: an ncurses colour pair, a
 * brightness attribute, and the glyph to draw.
 *
 * WHY a baked table (not computed colour): mapping a value to a colour for every
 * cell every frame must be cheap, so each theme is a precomputed array of N_PAL
 * stops and the value just indexes it.  Rotating that index by a phase each frame
 * is the classic palette-cycling animation (Ferrari, ref [3]) — the screen
 * appears to flow though no pixel moves.  Glyph density rising along the ramp
 * ('.' → '@') is the ASCII-luminance trick (Bourke, ref [4]).
 *
 * VALUE LOGIC: pair → a §6 color_init hue; attr ∈ {A_DIM, 0, A_BOLD} shades that
 * hue darker → brighter; ch is the stop's glyph. */
typedef struct {
    int    pair;   /* ncurses colour-pair id (see §6 color_init)   */
    chtype attr;   /* A_DIM / 0 / A_BOLD — brightness within a hue  */
    char   ch;     /* glyph (density rises along the ramp)          */
} PalEntry;

/* FreqPreset — the parameters of the four superposed plasma waves (wave
 * superposition, Hecht ref [2]; the plasma technique, ref [1]): a spatial
 * frequency and a time speed for the horizontal, vertical, diagonal and radial
 * terms of plasma_value.
 *
 * WHY presets: the same four-sine formula yields wildly different patterns from
 * the frequency/speed mix — low+slow reads as a "grand" swell, high+fast as
 * "turbulent".  Bundling a curated mix as a named preset lets 'f' swap the whole
 * look at once.
 *
 * VALUE LOGIC: f* are spatial frequencies (rad/cell, ~0.08–0.45) — higher = more
 * ripples per screen; s* are time speeds (rad/s, ~0.3–2.2) — higher = faster
 * shimmer.  name labels the preset in the HUD. */
typedef struct {
    float f1, f2, f3, f4;   /* spatial frequencies, rad/cell */
    float s1, s2, s3, s4;   /* time speeds, rad/s            */
    const char *name;
} FreqPreset;

/* Plasma — the entire effect's state.  It is STATELESS in the demoscene sense
 * (Vandevenne, ref [1]): there is no pixel grid or per-cell history — every cell
 * is recomputed from (col, row, time) each frame, so the whole "object" is a
 * clock plus three user choices.
 *
 * WHY so little: that's the point of plasma — it ran in fixed time per pixel on a
 * C64 with zero buffers.  Only `time` advances; the rest are knobs the user sets.
 *
 * VALUE LOGIC: time (s) feeds the wave phases AND the palette cycle; freq_preset
 * indexes FREQ_PRESETS (the wave maths); theme indexes THEMES (the palette — a
 * render choice, kept here only because it's a one-key toggle); paused freezes
 * the clock so the pattern holds still. */
typedef struct {
    float time;          /* seconds elapsed — the sole advancing state   */
    int   freq_preset;   /* index into FREQ_PRESETS (wave parameters)     */
    int   theme;         /* index into THEMES (palette — a render choice) */
    bool  paused;        /* freezes the time advance                      */
} Plasma;

/* Scene — the framework's "what is simulated" slot.  For plasma that is just the
 * one Plasma effect (no other entities); kept as a named aggregate for symmetry
 * with the other demos. */
typedef struct { Plasma plasma; } Scene;

/* Screen — the ncurses display surface: the cached terminal size in cells.
 * ncurses owns the real frame buffer (stdscr); this just keeps cols×rows so the
 * hot path and HUD don't call getmaxyx() — refreshed at init and on SIGWINCH. */
typedef struct { int cols, rows; } Screen;

/* App — process-level glue binding the scene to its terminal.
 *
 * WHY global (g_app): POSIX signal handlers take no user-data argument, so the
 * SIGINT/SIGTERM/SIGWINCH handler reaches the run/resize flags through a
 * file-scope object; volatile sig_atomic_t is the one type the C standard
 * guarantees is safe to write in a handler and read in the loop. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;       /* fixed-timestep rate (] / [ keys)  */
    volatile sig_atomic_t running;       /* cleared by SIGINT/SIGTERM → exit  */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH → loop re-reads   */
} App;

/* ===================================================================== */
/* §4  logic — pure plasma field value: no mutation, no I/O               */
/* ===================================================================== */

/*
 * plasma_value — evaluate the plasma formula at one cell.
 *
 * Four sinusoid terms:
 *   • horizontal wave
 *   • vertical wave
 *   • diagonal wave
 *   • radial wave (aspect-corrected: dy × 2 so rings look circular)
 * Sum ∈ [−4, +4]; normalise to [0, 1].
 */
static float plasma_value(int col, int row, int cols, int rows,
                          float t, const FreqPreset *fp)
{
    float cx   = (float)cols * 0.5f;
    float cy   = (float)rows * 0.5f;
    float dx   = (float)col - cx;
    float dy   = ((float)row - cy) * 2.0f;  /* aspect correction */
    float dist = sqrtf(dx * dx + dy * dy);

    float v = sinf((float)col * fp->f1 + t * fp->s1)
            + sinf((float)row * fp->f2 + t * fp->s2)
            + sinf(((float)(col + row)) * fp->f3 + t * fp->s3)
            + sinf(dist * fp->f4 + t * fp->s4);

    return (v + 4.0f) * 0.125f;   /* [0, 1] */
}

/* palette_index — map a plasma value (plus the global cycle phase) to a palette
 * stop 0..N_PAL-1.  Adding `phase` before the wrap is what scrolls the colours
 * across the screen each frame (palette cycling, ref [3]). */
static int palette_index(float v, float phase)
{
    float vs = fmodf(v + phase, 1.0f);
    int   idx = (int)(vs * (float)N_PAL);
    if (idx < 0)      idx = 0;
    if (idx >= N_PAL) idx = N_PAL - 1;
    return idx;
}

/* ===================================================================== */
/* §5  simulation — advances state (Plasma.time only; otherwise stateless) */
/* ===================================================================== */

static void plasma_init(Plasma *p)
{
    p->time        = 0.0f;
    p->freq_preset = 0;
    p->theme       = 0;
    p->paused      = false;
}

static void plasma_tick(Plasma *p, float dt)
{
    if (p->paused) return;
    p->time += dt;
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    plasma_init(&s->plasma);
}

static void scene_tick(Scene *s, float dt)
{
    plasma_tick(&s->plasma, dt);
}

/* ===================================================================== */
/* §6  render — state → screen; reads only, never mutates sim state        */
/* ===================================================================== */

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
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

static const FreqPreset FREQ_PRESETS[N_FREQ_PRESETS] = {
    { 0.20f, 0.25f, 0.15f, 0.18f,  1.0f, 0.80f, 1.2f, 0.90f, "gentle"    },
    { 0.40f, 0.35f, 0.30f, 0.22f,  1.5f, 1.30f, 1.8f, 1.20f, "energetic" },
    { 0.10f, 0.12f, 0.08f, 0.09f,  0.4f, 0.30f, 0.5f, 0.35f, "grand"     },
    { 0.35f, 0.28f, 0.45f, 0.20f,  2.0f, 1.80f, 2.2f, 1.60f, "turbulent" },
};

static const char *THEME_NAMES[N_THEMES] = {
    "rainbow", "fire", "ocean", "matrix"
};

/*
 * Palette entries per theme.  N_PAL=14 entries arranged so that cycling
 * the plasma phase at CYCLE_HZ makes colours flow smoothly across the
 * screen.  Pair numbers map to §6 color_init() definitions.
 */
static const PalEntry THEMES[N_THEMES][N_PAL] = {
    /* 0: rainbow — blue → cyan → green → yellow → orange → red → magenta */
    {
        {6, A_DIM,  '.'}, {6, 0,      ':'},
        {5, 0,      '+'}, {5, A_BOLD, '+'},
        {4, 0,      '+'}, {4, A_BOLD, '*'},
        {3, 0,      '*'}, {3, A_BOLD, '*'},
        {2, A_BOLD, '#'}, {2, 0,      '#'},
        {1, A_BOLD, '#'}, {1, 0,      ':'},
        {7, 0,      '.'}, {7, A_DIM,  '.'},
    },
    /* 1: fire — dark red → red → orange → yellow */
    {
        {1, A_DIM,  '.'}, {1, A_DIM,  '.'},
        {1, 0,      ':'}, {1, A_BOLD, ':'},
        {2, A_DIM,  '+'}, {2, 0,      '+'},
        {2, A_BOLD, '*'}, {2, A_BOLD, '*'},
        {3, A_DIM,  '#'}, {3, 0,      '#'},
        {3, A_BOLD, '#'}, {3, A_BOLD, '@'},
        {3, A_BOLD, '@'}, {3, A_BOLD, '@'},
    },
    /* 2: ocean — deep blue → cyan → teal */
    {
        {6, A_DIM,  '.'}, {6, A_DIM,  '.'},
        {6, 0,      ':'}, {6, 0,      ':'},
        {6, A_BOLD, ':'}, {5, A_DIM,  '+'},
        {5, 0,      '+'}, {5, A_BOLD, '*'},
        {5, A_BOLD, '#'}, {4, A_DIM,  '#'},
        {4, 0,      '#'}, {4, A_BOLD, '@'},
        {4, A_BOLD, '@'}, {4, A_BOLD, '@'},
    },
    /* 3: matrix — shades of green only */
    {
        {4, A_DIM,  '.'}, {4, A_DIM,  '.'},
        {4, A_DIM,  ':'}, {4, 0,      ':'},
        {4, 0,      '+'}, {4, 0,      '+'},
        {4, 0,      '*'}, {4, A_BOLD, '*'},
        {4, A_BOLD, '#'}, {4, A_BOLD, '#'},
        {4, A_BOLD, '@'}, {4, A_BOLD, '@'},
        {4, A_BOLD, '@'}, {4, A_BOLD, '@'},
    },
};

static void plasma_draw(const Plasma *p, WINDOW *w, int cols, int rows)
{
    const FreqPreset *fp  = &FREQ_PRESETS[p->freq_preset];
    const PalEntry   *pal = THEMES[p->theme];
    float phase = fmodf(p->time * CYCLE_HZ, 1.0f);

    for (int row = 1; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float v = plasma_value(col, row, cols, rows, p->time, fp);
            const PalEntry *e = &pal[palette_index(v, phase)];
            wattron(w, COLOR_PAIR(e->pair) | e->attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)e->ch);
            wattroff(w, COLOR_PAIR(e->pair) | e->attr);
        }
    }
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    plasma_draw(&s->plasma, w, cols, rows);
}

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* HUD — data readout top-right (yellow), action keys bottom-left (cyan). */
static void draw_hud(const Plasma *p, int cols, int rows, double fps, int sim_fps)
{
    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  theme:%s  freq:%s  %s ",
             fps, sim_fps, THEME_NAMES[p->theme],
             FREQ_PRESETS[p->freq_preset].name,
             p->paused ? "PAUSED " : "running");
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc/p:pause  t:theme  f:frequencies  [/]:Hz ");
    attroff(COLOR_PAIR(5) | A_BOLD);
}

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);
    draw_hud(&sc->plasma, s->cols, s->rows, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §7  app — combine point (per-tick order) + user events + frame cap      */
/* ===================================================================== */

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* User event — mutates the Plasma / sim_fps, but NOT part of the tick. */
static bool app_handle_key(App *app, int ch)
{
    Plasma *p = &app->scene.plasma;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': case 'p': case 'P': p->paused = !p->paused; break;
    case 't': case 'T':
        p->theme = (p->theme + 1) % N_THEMES;
        break;
    case 'f': case 'F':
        p->freq_preset = (p->freq_preset + 1) % N_FREQ_PRESETS;
        break;
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
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        /* USER EVENT (out of tick): resize */
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 1. PERFORMANCE — measure dt (capped), then fixed-timestep accumulate */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* 2. SIMULATION — advance one fixed step per accumulated tick */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* 3. PERFORMANCE — fps tally + sleep to the 60 fps frame cap */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);

        /* 4. RENDER — read-only */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps);
        screen_present();

        /* USER EVENT (out of tick): key handling */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
