/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * excitable.c — Greenberg-Hastings Excitable Medium
 *
 * N-state cellular automaton on the terminal grid:
 *
 *   State 0          Resting    — quiescent, susceptible to excitation
 *   State 1          Excited    — firing; can excite any 4-neighbour
 *   States 2..N-1    Refractory — recovering; immune to excitation
 *
 * Rule (von Neumann neighbourhood, periodic boundary):
 *   cell == 0 AND any neighbour == 1  →  cell = 1   (excite)
 *   cell >  0                         →  cell = (cell+1) % N  (advance)
 *
 * N (total states, min 5, max 20) sets the refractory depth:
 *   small N → short recovery, dense fast waves
 *   large N → long recovery, sparse slow waves
 *
 * 4 presets:
 *   1 SPIRAL  — broken wave front; open end curls into a rotating spiral
 *   2 DOUBLE  — two counter-rotating spirals from offset broken fronts
 *   3 RINGS   — concentric target waves (radially periodic IC)
 *   4 CHAOS   — random 5% seeding → turbulent multi-spiral field
 *
 * Keys: q quit  p pause  r reset  1-4 preset  +/- N  t/T theme  spc pulse
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/excitable.c \
 *       -o excitable -lncurses -lm
 *
 * Sections (cut by concern — see ARCHITECTURE block):
 *   §1 config  §2 perf/delays  §3 state  §4 logic  §5 simulation
 *   §6 effects §7 render        §8 platform/app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Greenberg-Hastings automaton (1978).
 *                  A discrete 3-state CA extended to N states:
 *                  - State 0 (resting): can be excited by an excited neighbour
 *                  - State 1 (excited): fires; excites 4-von Neumann neighbours
 *                  - States 2..N-1 (refractory): cycling back to resting
 *                  Rule is purely local — each cell looks only at its 4 neighbours.
 *
 * Biology        : Models excitable media like cardiac muscle and neural tissue.
 *                  The refractory period (states 2..N-1) prevents a wave from
 *                  re-exciting cells it just passed through — this is why
 *                  electrical waves in the heart travel as ordered rings rather
 *                  than spreading chaotically in all directions.
 *                  Spirals emerge from broken wave fronts: the open end of the
 *                  front finds susceptible (resting) cells to its side and curls.
 *
 * Math           : N (total states) tunes two competing timescales:
 *                    - Excitation time: fixed at 1 step
 *                    - Refractory period: N−2 steps
 *                  Small N (N=5): refractory period=3 steps → dense fast waves.
 *                  Large N (N=20): refractory period=18 steps → sparse slow waves.
 *
 * Performance    : O(W×H) per step.  Double-buffering (cur/nxt arrays) ensures
 *                  each cell's next state depends on the current-tick neighbours
 *                  only, not mid-tick updates.  STEP_NS = 1/15s independent of
 *                  RENDER_NS = 1/30s so the CA runs at half display rate.
 *
 * References     : Concepts —
 *                   [1] Greenberg & Hastings, "Spatial Patterns for Discrete
 *                       Models of Diffusion in Excitable Media", SIAM J. Appl.
 *                       Math. 34(3), 515 (1978). The founding paper for this
 *                       exact automaton.
 *                   [2] Gerhardt, Schuster & Tyson, "A cellular automaton model
 *                       of excitable media", Physica D 46, 392 (1990). The
 *                       N-state CA refinement closest to the rule used here.
 *                   [3] Wiener & Rosenblueth, "The mathematical formulation of
 *                       the problem of conduction of impulses … in cardiac
 *                       muscle", Arch. Inst. Cardiol. Mex. 16, 205 (1946). The
 *                       original excitable-element model of the heart.
 *                   [4] Winfree, "The Geometry of Biological Time" (Springer,
 *                       2nd ed. 2001). The definitive treatment of spiral waves
 *                       and rotors in excitable media — why broken fronts curl.
 *                   [5] FitzHugh (Biophys. J. 1, 445, 1961) & Nagumo et al.
 *                       (Proc. IRE 50, 2061, 1962). FitzHugh–Nagumo, the
 *                       continuous-PDE cousin of this discrete model.
 *                   [6] Murray, "Mathematical Biology II: Spatial Models and
 *                       Biomedical Applications" (Springer, 3rd ed. 2003).
 *                       Textbook chapters on travelling & spiral waves.
 *                   [7] Davidenko et al., "Stationary and drifting spiral waves
 *                       of excitation in isolated cardiac muscle", Nature 355,
 *                       349 (1992). Spiral waves observed in real heart tissue
 *                       (the fibrillation link the Biology block describes).
 *                  Rendering —
 *                   [8] Ben-Halim, Raymond et al., "Writing Programs with
 *                       NCURSES" (NCURSES HOWTO). Colour pairs, erase/refresh,
 *                       the terminal rendering model used in §7.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layers) ───────────────────────────────────────────────
 *
 * Re-cut into labelled layers. Data is aggregated on one Scene (the medium +
 * its knobs); the table is the contract for who writes what. The per-tick
 * advance is a single call — medium_step — wrapped by RENDER and PERFORMANCE in
 * main(). Workers take the narrowest type (Medium* / const Scene*); only the
 * orchestrators (resize_and_reset, main) take the whole Scene.
 *
 *   LAYER          §   MUTATES                              KEY FUNCTIONS
 *   ------------  --   ----------------------------------   ----------------------
 *   CONFIG         1   (constants + pair layout)           —
 *   PERF/DELAYS    2   (local timing only)                 clock_ns, clock_sleep_ns
 *   STATE          3   defines Medium + Scene (g_scene)     —
 *   LOGIC          4   nothing (pure reads)                wrap_index, any_neighbour_excited
 *   SIMULATION     5   Medium (cur/nxt grid, gh/gw, n)     medium_step <-- the tick,
 *                                                          medium_clear, medium_seed,
 *                                                          medium_pulse
 *   EFFECTS        6   (none — see note)                   —
 *   RENDER         7   terminal + ncurses palette only     scene_draw, hud_line,
 *                                                          theme_apply, color_init
 *   PLATFORM/APP   8   g_quit/g_resize, g_rows/g_cols, and  main, resize_and_reset,
 *                      Scene knobs via key/resize events    sig_h, cleanup
 *
 * Per-tick combine (main, fixed-timestep loop), only when not paused:
 *   1. SIMULATION — medium_step(): one Greenberg-Hastings update of the whole
 *      medium (double-buffered: read cur, write nxt, then swap).
 *   No EFFECTS step and no separate LOGIC step — the tick IS medium_step.
 *   RENDER (erase → scene_draw → doupdate) and the PERFORMANCE shell (STEP_NS
 *   accumulator + RENDER_NS frame cap) wrap it in main and advance no sim state.
 *
 * Notes / contracts:
 *   (a) Signature convention IN FORCE: a pure read takes const (const Scene*); a
 *       mutator takes Medium* / Scene*. Only resize_and_reset / main take Scene*.
 *   (b) LOGIC (§4) is pure (const Medium*, no mutation / no I/O): wrap_index
 *       (toroidal index) and any_neighbour_excited (the GH excitation test).
 *       medium_step calls them; render/effects order cannot change their result.
 *   (c) medium_step() is the sole per-tick state advance. medium_seed /
 *       medium_pulse / medium_clear (§5), resize_and_reset (§8) and the key
 *       handlers in main also mutate state, but are INIT or USER EVENTS, not the tick.
 *   EFFECTS: none. The visual "fade" from bright excited cell to dim trailing
 *       cell IS the refractory state itself (simulation state 2..N-1), rendered
 *       through the k_rchar ramp — there is no cosmetic-only buffer.
 *   DELAYS: the paused gate (Scene.paused, checked in main before the step loop)
 *       and the inter-frame sleep (clock_sleep_ns, §2) — interwoven, not a layer.
 * ──────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GRID_H_MAX  120
#define GRID_W_MAX  360

#define N_MIN         5    /* N=5: refractory period = 3 steps (dense fast spirals)  */
#define N_MAX        20    /* N=20: refractory period = 18 steps (sparse slow waves) */
#define N_DEF        12    /* N=12: refractory = 10 steps; good balance of density   */

#define RENDER_NS  (1000000000LL / 30)   /* display: 30 fps                        */
#define STEP_NS    (1000000000LL / 15)   /* CA step: 15 steps/sec = 1 step per 2 frames;
                                          * slower than render so motion is smooth   */
#define HUD_ROWS    2    /* total reserved: 1 data row (top) + 1 actions row (bottom) */
#define HUD_TOP     1    /* data occupies row 0; the grid starts at row HUD_TOP       */
#define N_THEMES    5

#define PULSE_HALF_H  1            /* manual pulse (spc): half-height in rows    */
#define PULSE_HALF_W  3            /* ...and half-width in cols (a 3×7 block)    */
#define DT_MAX_NS     100000000LL  /* clamp a frame's dt to 100 ms — keeps a stall
                                    * from triggering a spiral-of-death of steps  */

/* Color pair layout: 1=HUD, 2..N_MAX+1=state 0..N_MAX-1, then HINT */
enum { CP_HUD = 1, CP_S0 = 2, CP_HINT = CP_S0 + N_MAX };

/* ===================================================================== */
/* §2  perf / delays  —  monotonic clock + sleep primitives               */
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
/* §3  state types  —  Medium + Scene (the one g_scene) + platform globals  */
/* ===================================================================== */

/*
 * Medium — the excitable medium: a toroidal lattice of cells, each holding an
 * integer phase in 0..n-1 that cycles 0(resting) → 1(excited) → 2..n-1
 * (refractory) → 0. This is the Greenberg-Hastings model (SIAM J. Appl. Math.
 * 34, 515, 1978), the discrete cousin of the FitzHugh-Nagumo excitable PDE;
 * it abstracts a patch of cardiac/neural tissue (Wiener & Rosenblueth 1946).
 *
 * WHY double-buffered: a synchronous CA must compute the ENTIRE next generation
 * from the current one — if a cell wrote into the grid mid-sweep, its neighbours
 * would see a half-updated field and the dynamics would corrupt. So we keep two
 * full generations and swap pointers each step (read `cur`, write `nxt`, swap).
 */
typedef struct {
    /* the two generations — only the pointers below say which is "current" */
    uint8_t bufA[GRID_H_MAX][GRID_W_MAX];
    uint8_t bufB[GRID_H_MAX][GRID_W_MAX];
    uint8_t (*cur)[GRID_W_MAX];   /* current generation — READ; one of bufA/bufB */
    uint8_t (*nxt)[GRID_W_MAX];   /* next generation — WRITTEN, then swapped in   */

    int gh, gw;   /* active extent this run (≤ GRID_H/W_MAX); sized to the
                     terminal so the lattice fills the screen below the HUD.     */
    int n;        /* states per cell. n lives here because the cell values ARE
                     taken mod n (medium_step does (s+1)%n). It also sets the
                     two competing timescales: excitation lasts 1 step, the
                     refractory tail lasts n-2 steps — so small n → dense fast
                     waves, large n → sparse slow ones. User-tunable (+/-).      */
} Medium;

/*
 * Scene — the whole simulation in one place, read like a table of contents:
 * the medium (WHAT is simulated) plus the handful of loose knobs the user turns.
 * None of preset/paused/theme is a concept worth its own type, so they sit flat
 * here, grouped by what they BELONG to (simulation vs. a render choice).
 */
typedef struct {
    Medium medium;     /* WHAT is simulated                                     */

    int    preset;     /* which initial condition is (re)seeded — 1 SPIRAL,
                          2 DOUBLE, 3 RINGS, 4 CHAOS. Stored (not just applied)
                          so resize re-seeds the same pattern and the HUD can
                          name it. SPIRAL/DOUBLE work because a BROKEN wave front
                          (an open end) curls into a rotor (Winfree, "The
                          Geometry of Biological Time", 2001).                   */
    bool   paused;     /* run-state: when set, medium_step is skipped (render
                          keeps running, so the frozen field stays on screen).   */
    int    theme;      /* RENDER choice only — palette index 0..N_THEMES-1. Kept
                          off the simulation knobs on purpose: it changes how
                          cells are coloured, nothing about how they evolve.     */
} Scene;

static Scene g_scene = {
    .medium = { .n = N_DEF },
    .preset = 1,
    .theme  = 0,
};

/* terminal geometry (PLATFORM §8 writes; RENDER §7 reads) */
static int g_rows, g_cols;

/* platform signal flags */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ===================================================================== */
/* §4  logic  —  pure decisions                                           */
/* ===================================================================== */

/* Toroidal (periodic-boundary) index: wraps i back into [0, len). */
static int wrap_index(int i, int len)
{
    return (i + len) % len;
}

/*
 * any_neighbour_excited — the Greenberg-Hastings excitation test: true iff at
 * least one of the four von-Neumann neighbours of (r,c) is excited (state 1),
 * with the lattice treated as a torus (edges wrap via wrap_index).
 */
static bool any_neighbour_excited(const Medium *m, int r, int c)
{
    int ru = wrap_index(r - 1, m->gh), rd = wrap_index(r + 1, m->gh);
    int cl = wrap_index(c - 1, m->gw), cr = wrap_index(c + 1, m->gw);
    return m->cur[ru][c] == 1 || m->cur[rd][c] == 1
        || m->cur[r][cl] == 1 || m->cur[r][cr] == 1;
}

/* ===================================================================== */
/* §5  simulation  —  the only layer that advances state                  */
/* ===================================================================== */

static void medium_clear(Medium *m)
{
    memset(m->bufA, 0, sizeof m->bufA);
    memset(m->bufB, 0, sizeof m->bufB);
    m->cur = m->bufA;
    m->nxt = m->bufB;
}

/*
 * medium_step — one synchronous Greenberg-Hastings update of the whole lattice:
 * a resting cell excites iff a neighbour is excited; every other cell advances
 * one phase toward resting. The new generation is written to `nxt`, then the
 * two buffers swap so `nxt` becomes the next read.
 */
static void medium_step(Medium *m)
{
    int n = m->n;
    for (int r = 0; r < m->gh; r++) {
        for (int c = 0; c < m->gw; c++) {
            uint8_t s = m->cur[r][c];
            if (s == 0)
                m->nxt[r][c] = any_neighbour_excited(m, r, c) ? 1 : 0;
            else
                m->nxt[r][c] = (uint8_t)((s + 1) % n);   /* advance toward resting */
        }
    }
    /* swap generations: nxt becomes the new current */
    uint8_t (*tmp)[GRID_W_MAX] = m->cur;
    m->cur = m->nxt;
    m->nxt = tmp;
}

/*
 * lay_wavefront — paint a BROKEN wave front: a run of excited cells (state 1)
 * along `row`, columns [c0,c1), trailing a refractory wake (states 2,3,…)
 * `wake_dir` rows away (-1 = wake above, +1 = wake below). The wake gives the
 * front a travel direction; its open end is what curls into a rotor.
 */
static void lay_wavefront(Medium *m, int row, int c0, int c1, int wake_dir)
{
    for (int c = c0; c < c1; c++) {
        m->cur[row][c] = 1;
        for (int dr = 1; dr < m->n - 1; dr++) {
            int rr = row + wake_dir * dr;
            if (rr < 0 || rr >= m->gh) break;
            m->cur[rr][c] = (uint8_t)(dr + 1);
        }
    }
}

/* SPIRAL — one broken front on the left half of the middle row; its open right
 * end has no downstream support and curls into a single rotating spiral. */
static void seed_spiral(Medium *m)
{
    int mr = m->gh / 2, mc = m->gw / 2;
    lay_wavefront(m, mr, 0, mc, -1);
}

/* DOUBLE — two broken fronts in opposite quadrants, wakes pointing opposite
 * ways, so they curl into a counter-rotating pair that never annihilates. */
static void seed_double(Medium *m)
{
    int mr1 = m->gh / 3, mr2 = m->gh * 2 / 3, mc = m->gw / 2;
    lay_wavefront(m, mr1, 0,  mc,     -1);
    lay_wavefront(m, mr2, mc, m->gw,  +1);
}

/* RINGS — radially periodic IC: phase = (n-1) - (dist mod n), so each ring of
 * a given radius sits one phase ahead of the next → concentric target waves.
 * The ×2 on the row term corrects the terminal's ~2:1 char aspect so rings
 * look circular. */
static void seed_rings(Medium *m)
{
    int gh = m->gh, gw = m->gw, n = m->n;
    int cr = gh / 2, cc = gw / 2;
    for (int r = 0; r < gh; r++)
        for (int c = 0; c < gw; c++) {
            float dr   = (float)(r - cr) * 2.0f;  /* aspect correction */
            float dc   = (float)(c - cc);
            int   dist = (int)sqrtf(dr * dr + dc * dc);
            int   s    = (n - 1) - (dist % n);
            m->cur[r][c] = (uint8_t)(s >= 0 ? s : 0);
        }
}

/* CHAOS — every cell a uniformly-random phase in 0..n-1. The random phase
 * boundaries act as broken fronts, so spirals nucleate everywhere and the field
 * stays in sustained multi-spiral turbulence. (Seeding isolated excited POINTS
 * instead makes closed rings that collide, annihilate, and die to blank within
 * ~2 s — no broken fronts to curl, so nothing self-sustains.) */
static void seed_chaos(Medium *m)
{
    int n = m->n;
    for (int r = 0; r < m->gh; r++)
        for (int c = 0; c < m->gw; c++)
            m->cur[r][c] = (uint8_t)(rand() % n);
}

/* Clear the lattice, then seed the chosen initial condition. */
static void medium_seed(Medium *m, int preset)
{
    medium_clear(m);
    if      (preset == 1) seed_spiral(m);
    else if (preset == 2) seed_double(m);
    else if (preset == 3) seed_rings(m);
    else                  seed_chaos(m);
}

/* Trigger a small rectangular pulse of excitation at the centre. */
static void medium_pulse(Medium *m)
{
    int cr = m->gh / 2, cc = m->gw / 2;
    for (int dr = -PULSE_HALF_H; dr <= PULSE_HALF_H; dr++)
        for (int dc = -PULSE_HALF_W; dc <= PULSE_HALF_W; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r >= 0 && r < m->gh && c >= 0 && c < m->gw)
                m->cur[r][c] = 1;
        }
}

/* ===================================================================== */
/* §6  effects                                                            */
/* ===================================================================== */

/*
 * No EFFECTS layer. There is no cosmetic-only state. The visual fade from a
 * bright just-excited cell to a dim trailing one IS the refractory state
 * itself (simulation state 2..N-1), rendered through the k_rchar ramp below —
 * nothing is stored that the simulation does not already own.
 */

/* ===================================================================== */
/* §7  render  —  state → screen, reads only; palette setup               */
/* ===================================================================== */

/*
 * Each theme defines a 12-entry 256-colour ramp: index 0 = peak (state 1),
 * index 11 = darkest (state 0).  States 2..N-1 interpolate through 1..9.
 */
static const char *k_theme_names[N_THEMES] = {
    "Fire", "Ice", "Matrix", "Plasma", "Mono"
};

static const short k_ramps256[N_THEMES][12] = {
    /* Fire:   white → yellow → orange → red → dark */
    { 231, 230, 226, 220, 214, 208, 202, 196, 160,  88,  52,  16 },
    /* Ice:    white → cyan → blue → dark navy */
    { 231, 195, 159, 123,  87,  51,  45,  39,  33,  27,  21,  17 },
    /* Matrix: white → bright-green → green → black */
    { 231,  82,  46,  40,  34,  28,  22,  22,  16,  16,  16,  16 },
    /* Plasma: white → magenta → purple → dark */
    { 231, 213, 207, 201, 165, 129,  93,  57,  54,  53,  17,  16 },
    /* Mono:   white → grey gradient → black */
    { 231, 252, 248, 244, 240, 236, 234, 233, 232, 232,  16,  16 },
};

static const short k_ramps8[N_THEMES][3] = {  /* peak / mid / dim */
    { COLOR_WHITE, COLOR_YELLOW,  COLOR_RED     },
    { COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE    },
    { COLOR_WHITE, COLOR_GREEN,   COLOR_GREEN   },
    { COLOR_WHITE, COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE, COLOR_WHITE,   COLOR_BLACK   },
};

/* Recompute all state color pairs.  Call when theme OR n changes. */
static void theme_apply(int theme, int n)
{
    init_pair(CP_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);

    for (int s = 0; s < N_MAX; s++) {
        int ri;
        if      (s == 0) ri = 11;   /* resting: darkest */
        else if (s == 1) ri = 0;    /* excited: brightest */
        else {
            ri = 1 + (s - 2) * 9 / (n > 2 ? n - 2 : 1);
            if (ri > 10) ri = 10;
        }
        short fg = (COLORS >= 256) ? k_ramps256[theme][ri]
                 : (s == 0) ? k_ramps8[theme][2]
                 : (s == 1) ? k_ramps8[theme][0]
                 :             k_ramps8[theme][1];
        init_pair((short)(CP_S0 + s), fg, -1);
    }
}

static void color_init(int theme, int n)
{
    start_color();
    use_default_colors();
    theme_apply(theme, n);
}

/* Refractory character ramp (7 entries), just-excited → about-to-rest;
 * draw_medium_cell maps a cell's refractory phase onto these glyphs. */
static const char k_rchar[] = "@*+=-.'";

/* Draw a HUD line in `pair` (bold), clipped to the terminal width so it never
 * overflows or wraps. */
static void hud_line(int row, int pair, const char *s)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddnstr(row, 0, s, g_cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Human-readable name of the active initial-condition preset (for the HUD). */
static const char *preset_name(int preset)
{
    return preset == 1 ? "Spiral" : preset == 2 ? "Double"
         : preset == 3 ? "Rings"  : "Chaos";
}

/*
 * draw_medium_cell — render one non-resting cell at screen (sy,sx). State 1 is
 * the wave front ('#', bold); refractory states 2..n-1 map onto the k_rchar
 * ramp by phase, freshest (state 2) still bold. Colour pair tracks the state.
 */
static void draw_medium_cell(int s, int n, int sy, int sx)
{
    chtype ch;
    attr_t at;
    if (s == 1) {
        ch = '#'; at = A_BOLD;
    } else {
        int ramp = (s - 2) * 7 / (n > 2 ? n - 2 : 1);   /* refractory phase → 0..6 (k_rchar len) */
        if (ramp >= 7) ramp = 6;
        ch = (chtype)(unsigned char)k_rchar[ramp];
        at = (s == 2) ? A_BOLD : A_NORMAL;
    }
    attron(COLOR_PAIR(CP_S0 + s) | at);
    mvaddch(sy, sx, ch);
    attroff(COLOR_PAIR(CP_S0 + s) | at);
}

/* Draw the lattice into the screen region between the HUD rows. */
static void draw_medium(const Medium *m)
{
    for (int r = 0; r < m->gh && r + HUD_TOP < g_rows - 1; r++) {
        for (int c = 0; c < m->gw && c < g_cols; c++) {
            int s = m->cur[r][c];
            if (s == 0) continue;   /* resting: leave blank */
            draw_medium_cell(s, m->n, r + HUD_TOP, c);
        }
    }
}

/* Top row: live status. Bottom row: the key actions. */
static void draw_hud(const Scene *sc)
{
    char buf[160];
    snprintf(buf, sizeof buf, " N=%d  preset=%s  theme=%s  %s ",
        sc->medium.n, preset_name(sc->preset),
        k_theme_names[sc->theme], sc->paused ? "PAUSED" : "running");
    hud_line(0, CP_HUD, buf);

    hud_line(g_rows - 1, CP_HINT,
        " q:quit  p:pause  r:reset  1-4:preset  +/-:N  t/T:theme  spc:pulse ");
}

static void scene_draw(const Scene *sc)
{
    draw_medium(&sc->medium);
    draw_hud(sc);
}

/* ===================================================================== */
/* §8  platform / app  —  ncurses, signals, resize, main loop             */
/* ===================================================================== */

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

/* Restore the terminal on exit and route quit/resize signals to the flags. */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);
}

/* Put the terminal into raw, non-blocking, cursor-off mode for the animation. */
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
}

static void resize_and_reset(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    s->medium.gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
    s->medium.gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
    medium_seed(&s->medium, s->preset);
}

/*
 * handle_key — apply one keypress to the scene. Returns true if it RESEEDED the
 * medium (reset or preset switch) so the caller can restart the step clock.
 */
static bool handle_key(Scene *s, int ch)
{
    Medium *m = &s->medium;
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1;            break;
    case 'p': case 'P': s->paused = !s->paused;         break;

    case 'r': case 'R':
        medium_seed(m, s->preset);
        return true;
    case '1': case '2': case '3': case '4':
        s->preset = ch - '0';
        medium_seed(m, s->preset);
        return true;

    case ' ': medium_pulse(m);                          break;

    case '+': case '=':
        if (m->n < N_MAX) {                  /* raising N can't invalidate a cell */
            m->n++;
            theme_apply(s->theme, m->n);
        }
        break;
    case '-':
        if (m->n > N_MIN) {
            m->n--;
            theme_apply(s->theme, m->n);
            for (int r = 0; r < m->gh; r++)  /* clamp states that now exceed N */
                for (int c = 0; c < m->gw; c++)
                    if (m->cur[r][c] >= (uint8_t)m->n)
                        m->cur[r][c] = 0;
        }
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme, m->n);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme, m->n);
        break;

    default: break;
    }
    return false;
}

int main(void)
{
    srand((unsigned)time(NULL));
    install_signals();
    screen_init();
    color_init(g_scene.theme, g_scene.medium.n);
    resize_and_reset(&g_scene);

    Medium   *m        = &g_scene.medium;
    long long step_acc = 0;
    long long last_ns  = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            resize_and_reset(&g_scene);
            last_ns  = clock_ns();
            step_acc = 0;
        }

        if (handle_key(&g_scene, getch()))   /* reseeded → restart the step clock */
            step_acc = 0;

        /* advance the wall clock, capped so a stall can't avalanche steps */
        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        if (dt > DT_MAX_NS) dt = DT_MAX_NS;
        last_ns = now_ns;

        /* run the fixed-timestep CA ticks that have come due (unless paused) */
        if (!g_scene.paused) {
            step_acc += dt;
            while (step_acc >= STEP_NS) {
                medium_step(m);
                step_acc -= STEP_NS;
            }
        }

        /* render one frame, then cap the display rate */
        erase();
        scene_draw(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }
    return 0;
}
