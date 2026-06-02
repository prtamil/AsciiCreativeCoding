/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lenia.c — Lenia (Continuous Life)
 *
 * Continuous-state, continuous-time generalisation of Game of Life.
 * Each cell u ∈ [0,1] evolves via:
 *
 *   u(t+dt) = clip(u(t) + dt · (2·G(K⊛u(t)) - 1), 0, 1)
 *
 * where K is a ring-shaped convolution kernel (all weight on a hollow shell
 * at radius R) and G is a Gaussian-shaped growth function:
 *
 *   G(x) = exp(-((x-μ)/σ)² / 2)
 *
 * With the right parameters (μ=0.15, σ=0.015) self-organising "creatures"
 * emerge and glide across the screen.
 *
 * Presets (1-3):
 *   1  Orbium  — μ=0.15 σ=0.015 R=13 dt=0.10  (the classic glider)
 *   2  Aquarium — μ=0.26 σ=0.036 R=10 dt=0.10  (jellyfish-like)
 *   3  Scutium  — μ=0.17 σ=0.015 R=8  dt=0.15  (shield shape)
 *
 * Keys: q quit  1/2/3 preset  p pause  r random  SPACE seed  +/- speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/lenia.c \
 *       -o lenia -lncurses -lm
 *
 * §1 config+types  §2 performance  §3 logic  §4 simulation  §5 render  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Continuous cellular automaton with convolution kernel.
 *                  Each step: convolve the state grid with kernel K (weighted
 *                  ring), evaluate growth function G, apply update rule.
 *                  Naïve O(W·H·πR²) convolution — for R=13 and a 200×60 grid
 *                  that's ~6M ops per step.
 *
 * Physics/Biology: Lenia (Bert Wang-Chak Chan, 2019).
 *                  A continuous generalisation of Conway's Game of Life.
 *                  The ring kernel captures "neighbourhood density at radius R"
 *                  — analogous to how a biological cell senses chemical
 *                  gradients from nearby cells.  Self-organisation emerges
 *                  from the tension between growth (G>0.5) and decay (G<0.5).
 *
 * Math           : Growth function G(x) = exp(−(x−μ)²/(2σ²)).
 *                  This is a Gaussian centred at μ.  When the kernel
 *                  convolution result matches μ exactly, G=1 and the cell
 *                  grows toward 1.  Far from μ, G→0 and the cell decays.
 *                  The parameter pair (μ, σ) defines a "species" of creature.
 *
 * Performance    : The convolution sums weights from all cells within
 *                  radius R.  Normalised so total kernel weight = 1.
 *                  Pre-building the kernel O(R²) once amortises the cost
 *                  across all (W×H) cells per step.
 *
 * References     : Concept —
 *                  [1] Chan, "Lenia: Biology of Artificial Life",
 *                      Complex Systems 28(3), 2019. arXiv:1812.05433.
 *                      The defining paper: ring kernel + growth formalism.
 *                  [2] Chan, "Lenia and Expanded Universe", ALIFE 2020.
 *                      arXiv:2005.03742. Multi-kernel/channel extensions
 *                      and a catalogue of self-organising creatures.
 *                  [3] Rafler, "SmoothLife: Generalization of Conway's
 *                      Game of Life to a continuous domain", 2011.
 *                      arXiv:1111.1567. The continuous-convolution
 *                      precursor this update rule is built on.
 *                  [4] Gardner, "Mathematical Games", Scientific American
 *                      223, Oct 1970. Conway's original Game of Life.
 *                  Rendering —
 *                  [5] Bourke, "Character Representation of Grey Scale
 *                      Images", 1997. paulbourke.net/dataformats/asciiart.
 *                      Luminance→glyph ramp behind the . : + * # mapping.
 *                  [6] Padala, "NCURSES Programming HOWTO", TLDP.
 *                      Color pairs, erase/refresh, non-blocking input.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * All data hangs off one Scene (§1), but functions take the NARROWEST type
 * they need, so aggregating on Scene never re-couples the layers:
 *
 *   Layer        Section  Takes                          Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       (scalars)                       nothing — time / sleep
 *   LOGIC        §3       const Field / Kernel / Species  NOTHING — pure, no I/O
 *   SIMULATION   §4       Field*, const Kernel*, …        Field (cur/next), Kernel
 *   RENDER       §5       const Scene*                    the terminal only
 *   APP          §6       Scene*                          Scene; drives the above
 *
 *   EFFECTS — none.  Glyph + colour are derived at render time from the cell
 *             value u (§5 scene_draw); no trail/glow/flash state is stored.
 *   DELAYS  — trivial.  The only wait is the frame-cap sleep (§2 PERFORMANCE);
 *             'p' toggles Scene.paused, a flag read by the tick, not a timer.
 *
 * PER-TICK COMBINE — exactly one function advances state, in this order
 * (§6 scene_tick):
 *     1. SIMULATION : field_step() × speed       (advance the field)
 *     2. LOGIC      : field_activity()           (is the field dead?)
 *     3. SIMULATION : field_randomize() if dead  (auto-reseed)
 * RENDER (erase → scene_draw → doupdate) then PERFORMANCE (sleep) run every
 * frame, OUTSIDE the tick — they read state but never advance it.
 *
 * USER EVENTS are NOT part of the tick: key presses, resize, and
 * scene_load_preset() mutate Scene directly in §6 and may reseed or rebuild
 * the kernel, but none of them call scene_tick()/field_step().
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config + types   (the ONLY place data is declared)                 */
/* ===================================================================== */

#define GRID_W_MAX  200
#define GRID_H_MAX   60
#define HUD_TOP       1   /* data row at top      */
#define HUD_BOT       1   /* action row at bottom */
#define KERNEL_R_MAX 20   /* max kernel radius in cells */
#define RENDER_NS   (1000000000LL / 30)
#define ACTIVITY_MIN 0.003f   /* mean cell value below which the field is "dead" → auto-reseed */

/* simulation tunables */
#define SPEED_MAX         4       /* cap on simulation steps per rendered frame    */

/* kernel shape (ring profile) */
#define RING_THICKNESS    0.2f    /* Gaussian shell width as a fraction of R        */
#define KERNEL_WEIGHT_MIN 0.001f  /* drop kernel entries lighter than this (sparse) */

/* seeding & rendering */
#define SEED_DISC_RATIO   0.8f    /* centre seed fills a disc of this × R           */
#define CELL_DRAW_MIN     0.05f   /* cells fainter than this are left blank         */

/* Species — the four numbers that define one Lenia creature's behaviour.
 *
 * WHY a struct: in Lenia a "species" IS its parameter tuple — Orbium and
 * Aquarium differ ONLY in these values; the code path is identical.  So
 * loading a preset is just copying one Species.  (Chan 2019 [1], §2: a
 * lifeform is the tuple (kernel K, growth G, timestep dt).)
 *
 * The update rule every member feeds, per cell:
 *     u' = clip( u + dt · (2·G(K⊛u) − 1) ),   G(x) = exp(−((x−mu)/sigma)²/2) */
typedef struct {
    /* growth centre μ: the neighbourhood density (kernel result U) at which
     * growth peaks (G=1).  Larger μ → the creature favours denser
     * surroundings before it will grow.  Typical 0.10–0.30. */
    float mu;
    /* growth width σ: the tolerance band around μ.  Small σ = a sharp,
     * fragile rule that dies on the slightest mismatch; larger σ = more
     * forgiving and blobby.  Typical 0.01–0.04. */
    float sigma;
    /* kernel radius R, in cells: how far each cell "senses" its neighbours.
     * Sets the creature's spatial scale and the O(R²) per-cell step cost;
     * clamped to KERNEL_R_MAX when the kernel is built. */
    float R;
    /* integration timestep dt (explicit Euler): the fraction of the growth
     * rule applied per step.  Smaller = smoother and slower evolution; too
     * large lets the field overshoot and diverge.  Typical 0.10–0.15. */
    float dt;
} Species;

/* Preset — a named entry in the species catalogue, bound to keys 1-3.
 *
 * WHY separate from Species: the simulation only ever needs the numbers; the
 * name exists purely so the HUD can label what is running and so the table
 * below reads like a field guide.  The three presets are classic Lenia
 * lifeforms reported in Chan 2019 [1]. */
typedef struct {
    const char *name;    /* display label, e.g. "Orbium" — HUD use only   */
    Species     species; /* the rule this catalogue entry loads           */
} Preset;

static const Preset PRESETS[] = {
    { "Orbium",   { 0.150f, 0.015f, 13.f, 0.10f } },
    { "Aquarium", { 0.260f, 0.036f, 10.f, 0.10f } },
    { "Scutium",  { 0.170f, 0.015f,  8.f, 0.15f } },
};

/* ncurses colour-pair IDs.  CP_U0..CP_U4 form a 5-step ramp over the cell
 * value u (low→high = navy→white, assigned in color_init / cell_glyph);
 * CP_HUD and CP_HINT colour the two HUD rows.  Numbered from 1 because
 * ncurses reserves pair 0 for the terminal's default colours. */
enum { CP_U0=1, CP_U1, CP_U2, CP_U3, CP_U4, CP_HUD, CP_HINT };

/* Field — the world: a continuous scalar field u(r,c) ∈ [0,1] sampled on a
 * grid.  This is the cellular-automaton state ("A" in Chan 2019 [1]); the
 * continuous range — versus Life's {0,1} — is exactly what lets smooth
 * gliders exist instead of blocky cells.
 *
 * WHY double-buffered: one step must read every neighbour from the SAME
 * snapshot, so field_step writes into `next` and only then overwrites `cur`.
 * An in-place update would let already-updated cells corrupt the neighbours
 * still being computed, breaking the simultaneity the rule assumes.
 *
 * WHY toroidal: neighbour indexing wraps modulo h/w (see field_potential),
 * so a creature gliding off one edge reappears on the opposite side — there
 * are no boundary walls to disturb it. */
typedef struct {
    /* live state: each cell's value in [0,1]; 0 = empty, 1 = saturated.
     * This is the buffer every layer reads and the renderer draws. */
    float cur [GRID_H_MAX][GRID_W_MAX];
    /* write target for the step in progress; holds the next generation until
     * the memcpy swap.  Read by nothing except field_step — not real state. */
    float next[GRID_H_MAX][GRID_W_MAX];
    /* the region actually simulated and drawn, ≤ the fixed MAX backing store.
     * Shrinks to fit the terminal so a small window doesn't iterate dead
     * cells; the backing arrays stay MAX-sized to avoid reallocation. */
    int   h, w;
} Field;

/* Kernel — the convolution kernel K: a Gaussian "ring" whose weight sits on
 * a hollow shell at radius R (peak at distance r=R, shell thickness ~0.2R).
 * Convolving the field with K measures local neighbourhood density at that
 * radius — the "sensing" half of the rule.  The ring shape is Lenia's
 * signature (Chan 2019 [1]); it generalises SmoothLife's annulus (Rafler
 * 2011 [3]).
 *
 * WHY sparse — parallel arrays + a count, not a dense (2R+1)² mask: most
 * cells in that bounding box carry ~0 weight (only the shell matters), so we
 * keep only the entries above a threshold and the hot loop in field_step
 * iterates `n` of them instead of the full box. */
typedef struct {
    /* weight of entry i, normalised so Σ w = 1.  Normalisation makes the
     * convolution a weighted AVERAGE, keeping the result U in the same [0,1]
     * range as u so the growth function sees a comparable scale. */
    float w [(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
    /* neighbour offset of entry i relative to the cell being updated:
     * dr = row delta, dc = column delta (either may be negative). */
    int   dr[(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
    int   dc[(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
    /* count of entries actually populated in w/dr/dc (≤ array capacity);
     * depends on R, recomputed every kernel_build. */
    int   n;
} Kernel;

/* Scene — the entire simulation in one aggregate, handed to the §6
 * orchestrators (init / reset / tick).  Laid out as a table of contents:
 *   WHAT is simulated — field + kernel
 *   HOW it is driven  — species (the rule), which preset, how fast
 *   WHERE / when      — terminal geometry + paused flag
 * Data aggregates here, but every non-orchestrator takes a narrow sub-type
 * (Field*, const Species*, …), so gathering it on Scene never re-couples the
 * layers separated in §2–§5. */
typedef struct {
    /* WHAT — the world and the kernel that evolves it. */
    Field   field;
    Kernel  kernel;
    /* HOW — the active rule plus the UI bookkeeping around it. */
    Species species;     /* live mu/sigma/R/dt — a copy of a preset's rule  */
    int     preset;      /* index into PRESETS: which species, for the HUD  */
    int     speed;       /* simulation steps run per rendered frame (1–4)   */
    /* WHERE/when — presentation state, too thin to deserve their own types. */
    int     rows, cols;  /* current terminal size, in characters            */
    bool    paused;      /* true = freeze the tick, but keep rendering       */
} Scene;

static Scene g_scene = { .speed = 1 };

/* Signal-handler ↔ loop comms — not Scene data, so kept file-scope. */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ===================================================================== */
/* §2  performance   (frame clock + frame-cap sleep; the only DELAY)      */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  logic   (pure decisions: read-only, no I/O — cannot be corrupted)  */
/* ===================================================================== */

/* Clip a cell value back into the valid range [0,1]. */
static float clamp01(float v)
{
    return v < 0.f ? 0.f : v > 1.f ? 1.f : v;
}

/* True if the offset (dr,dc) lies within `radius` cells of the origin. */
static bool within_radius(int dr, int dc, float radius)
{
    return sqrtf((float)(dr*dr + dc*dc)) <= radius;
}

/* Smaller of two ints — used to clamp a span to its backing-store capacity. */
static int imin(int a, int b) { return a < b ? a : b; }

/* The ring kernel's profile: a Gaussian shell peaking at distance R with
 * width RING_THICKNESS·R.  Returns the (un-normalised) weight at `dist`. */
static float ring_weight(float dist, float R)
{
    float x = (dist - R) / (RING_THICKNESS * R + 0.001f);  /* +eps guards R=0 */
    return expf(-x*x * 0.5f);
}

/* Growth function G(U) for a cell whose neighbourhood density is U. */
static float growth(const Species *sp, float U)
{
    float x = (U - sp->mu) / sp->sigma;
    return expf(-x*x * 0.5f);
}

/* Potential U = (K ⊛ u) at cell (r,c): the kernel-weighted sum over the
 * neighbourhood, wrapping toroidally at the grid edges.  This is the
 * "neighbourhood density" that the growth function then judges. */
static float field_potential(const Field *f, const Kernel *k, int r, int c)
{
    float U = 0.f;
    for (int i = 0; i < k->n; i++) {
        int nr = (r + k->dr[i] + f->h) % f->h;   /* wrap row  */
        int nc = (c + k->dc[i] + f->w) % f->w;   /* wrap col  */
        U += k->w[i] * f->cur[nr][nc];
    }
    return U;
}

/* Mean cell value — a near-zero result means the field has died out. */
static float field_activity(const Field *f)
{
    float sum = 0.f;
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            sum += f->cur[r][c];
    return sum / (float)(f->h * f->w);
}

/* ===================================================================== */
/* §4  simulation   (advances state: mutates Field cur/next + Kernel)     */
/* ===================================================================== */

/* Append one (offset, weight) entry to the sparse kernel. */
static void kernel_add_entry(Kernel *k, int dr, int dc, float w)
{
    int e = k->n++;
    k->dr[e] = dr; k->dc[e] = dc; k->w[e] = w;
}

/* Scale every weight so they sum to 1 → the convolution becomes a weighted
 * average, keeping the potential U in the same [0,1] range as the field. */
static void kernel_normalize(Kernel *k)
{
    float total = 0.f;
    for (int e = 0; e < k->n; e++) total += k->w[e];
    if (total > 0.f)
        for (int e = 0; e < k->n; e++) k->w[e] /= total;
}

/* Build the ring kernel for radius R: over the (2·iR+1)² bounding box, keep
 * every neighbour offset whose shell weight is non-negligible, then normalise. */
static void kernel_build(Kernel *k, float R)
{
    int iR = imin((int)ceilf(R), KERNEL_R_MAX);   /* scan box half-size */
    k->n = 0;
    for (int dr = -iR; dr <= iR; dr++)
        for (int dc = -iR; dc <= iR; dc++) {
            float dist = sqrtf((float)(dr*dr + dc*dc));
            float w    = ring_weight(dist, R);
            if (w >= KERNEL_WEIGHT_MIN)
                kernel_add_entry(k, dr, dc, w);
        }
    kernel_normalize(k);
}

/* State setup / reseed — mutate the field.  Invoked by user events and the
 * auto-reseed check, NOT by the per-tick combine. */
static void field_randomize(Field *f)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            f->cur[r][c] = (float)rand()/RAND_MAX;
}

static void field_clear(Field *f)
{
    memset(f->cur, 0, sizeof f->cur);
}

/* Place an "orbium seed" at center: random values filling a disc of radius
 * 0.8·R around the middle cell; everything outside stays cleared. */
static void field_seed_center(Field *f, const Species *sp)
{
    field_clear(f);
    int cr = f->h/2, cc = f->w/2;     /* centre cell        */
    int R  = (int)sp->R;              /* scan box half-size */
    for (int dr = -R; dr <= R; dr++)
        for (int dc = -R; dc <= R; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r < 0 || r >= f->h || c < 0 || c >= f->w) continue;  /* off-grid */
            if (within_radius(dr, dc, R * SEED_DISC_RATIO))
                f->cur[r][c] = (float)rand()/RAND_MAX;
        }
}

/* One simulation step.  For every cell: sense its neighbourhood, nudge the
 * value up (G>0.5) or down (G<0.5) by the growth response, then clip.
 * Writes into `next` so all cells update from the same snapshot of `cur`;
 * the buffers are swapped at the end.  THE state advance. */
static void field_step(Field *f, const Kernel *k, const Species *sp)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++) {
            float U  = field_potential(f, k, r, c);           /* sense neighbourhood */
            float du = sp->dt * (2.f * growth(sp, U) - 1.f);  /* growth-driven delta */
            f->next[r][c] = clamp01(f->cur[r][c] + du);        /* integrate + clip    */
        }
    memcpy(f->cur, f->next, sizeof f->cur);
}

/* ===================================================================== */
/* §5  render   (state → screen: reads Scene, mutates only the terminal)  */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_U0, 25, -1);   /* dark navy    — 0.0-0.2  */
        init_pair(CP_U1,  27, -1);   /* blue         — 0.2-0.4  */
        init_pair(CP_U2,  51, -1);   /* cyan         — 0.4-0.6  */
        init_pair(CP_U3, 195, -1);   /* pale cyan    — 0.6-0.8  */
        init_pair(CP_U4, 231, -1);   /* white        — 0.8-1.0  */
        init_pair(CP_HUD, 226, -1);  /* bright yellow — top data row    */
        init_pair(CP_HINT, 51, -1);  /* bright cyan   — bottom key hints */
    } else {
        init_pair(CP_U0, COLOR_BLUE,   -1);
        init_pair(CP_U1, COLOR_BLUE,   -1);
        init_pair(CP_U2, COLOR_CYAN,   -1);
        init_pair(CP_U3, COLOR_CYAN,   -1);
        init_pair(CP_U4, COLOR_WHITE,  -1);
        init_pair(CP_HUD, COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,  -1);
    }
}

/* Map a cell value u∈[0,1] to its glyph + colour pair along a 5-band
 * navy→white luminance ramp (Bourke [5]).  Returns false when the cell is
 * fainter than CELL_DRAW_MIN, meaning "leave it blank". */
static bool cell_glyph(float u, chtype *glyph, int *pair)
{
    if      (u < CELL_DRAW_MIN) return false;
    else if (u < 0.20f) { *pair = CP_U0; *glyph = '.'; }   /* uniform 0.20-wide */
    else if (u < 0.40f) { *pair = CP_U1; *glyph = ':'; }   /* ramp bands        */
    else if (u < 0.60f) { *pair = CP_U2; *glyph = '+'; }
    else if (u < 0.80f) { *pair = CP_U3; *glyph = '*'; }
    else                { *pair = CP_U4; *glyph = '#'; }
    return true;
}

/* Draw the field cells into the region between the two HUD rows. */
static void draw_field(const Field *f, int rows, int cols)
{
    for (int r = 0; r < f->h && r + HUD_TOP < rows - HUD_BOT; r++)
        for (int c = 0; c < f->w && c < cols; c++) {
            chtype glyph; int pair;
            if (!cell_glyph(f->cur[r][c], &glyph, &pair)) continue;
            attron(COLOR_PAIR(pair));
            mvaddch(r + HUD_TOP, c, glyph);
            attroff(COLOR_PAIR(pair));
        }
}

/* Draw one HUD row left-aligned at `row`, clipped to the terminal width so
 * it can never overflow onto the next line. */
static void draw_hud_row(int row, int pair, int cols, const char *text)
{
    char buf[160];
    snprintf(buf, sizeof buf, "%s", text);
    if ((int)strlen(buf) > cols) buf[cols] = '\0';
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Top row = live parameters (data); bottom row = key hints (actions).
 * ASCII only — "mu"/"sig" stand in for the μ/σ growth parameters. */
static void draw_hud(const Scene *sc)
{
    const Species *sp = &sc->species;
    char data[160];
    snprintf(data, sizeof data,
        " Lenia [%d] %-8s mu=%.3f sig=%.3f R=%.0f dt=%.2f  speed:%dx  %s ",
        sc->preset+1, PRESETS[sc->preset].name,
        sp->mu, sp->sigma, sp->R, sp->dt,
        sc->speed, sc->paused ? "PAUSED " : "running");
    draw_hud_row(0, CP_HUD, sc->cols, data);
    draw_hud_row(sc->rows - 1, CP_HINT, sc->cols,
        " q:quit  1/2/3:preset  p:pause  r:random  +/-:speed  spc:seed ");
}

/* Render the whole scene: field cells, then the two HUD rows.
 * const → reads state only, so rendering can never corrupt the simulation. */
static void scene_draw(const Scene *sc)
{
    draw_field(&sc->field, sc->rows, sc->cols);
    draw_hud(sc);
}

/* ===================================================================== */
/* §6  app   (events + orchestrators; the only functions that take Scene*) */
/* ===================================================================== */

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void install_signals(void)
{
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);
}

/* Put the terminal into raw, non-blocking, no-cursor mode for animation. */
static void terminal_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
}

/* Orchestrator: fit the active grid to the current terminal size, clamping
 * each span to the fixed backing-store capacity. */
static void scene_fit_terminal(Scene *sc)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    sc->rows = rows; sc->cols = cols;
    sc->field.h = imin(rows - HUD_TOP - HUD_BOT, GRID_H_MAX);
    sc->field.w = imin(cols, GRID_W_MAX);
}

/* Orchestrator (reset): adopt a catalogued species, rebuild kernel, reseed. */
static void scene_load_preset(Scene *sc, int p)
{
    sc->preset  = p;
    sc->species = PRESETS[p].species;
    kernel_build(&sc->kernel, sc->species.R);
    field_randomize(&sc->field);   /* presets start full-screen, like 'r' */
}

/* Orchestrator (tick): the ONE place simulation state advances. */
static void scene_tick(Scene *sc)
{
    if (sc->paused) return;
    for (int s = 0; s < sc->speed; s++)
        field_step(&sc->field, &sc->kernel, &sc->species);   /* SIMULATION */
    if (field_activity(&sc->field) < ACTIVITY_MIN)           /* LOGIC      */
        field_randomize(&sc->field);                         /* SIMULATION: reseed */
}

/* USER EVENT: terminal resized — refit the grid and reseed (not a tick). */
static void handle_resize(Scene *sc)
{
    g_resize = 0;
    endwin(); refresh();
    scene_fit_terminal(sc);
    scene_load_preset(sc, sc->preset);
}

/* USER EVENT: dispatch one key press — mutates flags/params/state, not a tick. */
static void handle_key(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1; break;
    case 'p': case 'P': sc->paused = !sc->paused; break;
    case 'r': case 'R': field_randomize(&sc->field); break;
    case ' ': field_seed_center(&sc->field, &sc->species); break;
    case '1': scene_load_preset(sc, 0); break;
    case '2': scene_load_preset(sc, 1); break;
    case '3': scene_load_preset(sc, 2); break;
    case '+': case '=': sc->speed++; if (sc->speed > SPEED_MAX) sc->speed = SPEED_MAX; break;
    case '-': sc->speed--; if (sc->speed < 1) sc->speed = 1; break;
    default: break;
    }
}

/* RENDER: clear, draw the scene, flush in one diff write. */
static void frame_render(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    install_signals();
    terminal_init();

    scene_fit_terminal(&g_scene);
    scene_load_preset(&g_scene, 0);

    while (!g_quit) {
        if (g_resize) handle_resize(&g_scene);
        handle_key(&g_scene, getch());

        long long frame_start = clock_ns();
        scene_tick(&g_scene);                  /* advance simulation */
        frame_render(&g_scene);                /* state → screen     */

        long long elapsed = clock_ns() - frame_start;
        clock_sleep_ns(RENDER_NS - elapsed);   /* PERFORMANCE: hold the frame rate */
    }
    return 0;
}
