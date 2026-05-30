/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cellular_automata_1d.c — Wolfram 1-D Elementary Cellular Automaton
 *
 * The pattern builds row-by-row from the top of the screen downward.
 * When the screen fills it holds for 3 seconds then loads the next preset.
 *
 * Layout:
 *   Row 0          — data bar: rule, name, class, preset, speed, state
 *                    (coloured by the rule's Wolfram class)
 *   Rows 1 … n-2   — CA area, filling top-down one row at a time
 *   Row n-1        — action bar: the interactive keys
 *
 * Each rule has a Wolfram class, colour-coded:
 *   Class 1 (Fixed)    — grey    — converges to uniform state
 *   Class 2 (Periodic) — cyan    — stable repeating patterns
 *   Class 3 (Chaotic)  — orange  — random-looking, sensitive to seed
 *   Class 4 (Complex)  — green   — localised glider-like structures
 *   Class 5 (Fractal)  — yellow  — Sierpinski / self-similar triangles
 *
 * Keys:
 *   n / p         next / previous preset
 *   t / T         next / previous colour theme
 *   a             toggle auto-advance (on by default)
 *   r             reseed — single cell at centre
 *   R             reseed — random initial row
 *   + / =         faster (fewer ticks between rows)
 *   - / _         slower (more ticks between rows)
 *   space         pause / resume
 *   q / Q         quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/cellular_automata_1d.c \
 *       -o cellular_automata_1d -lncurses
 *
 * Sections: §1 config  §2 clock  §3 color  §4 model  §5 simulation
 *           §6 render   §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Wolfram Elementary Cellular Automaton (ECA).
 *                  A 1-D binary row evolves by applying an 8-bit lookup
 *                  table (the rule): each cell's new state depends on
 *                  itself and its two neighbours → 2³=8 neighbourhood
 *                  configurations → 256 possible rules (Wolfram 1983).
 *
 * Math           : Rule encoding: the 3-bit neighbourhood (left, center,
 *                  right) is treated as a binary number 0–7; the rule's
 *                  n-th bit gives the new center state for neighbourhood n.
 *                  Example: Rule 110 = 0b01101110 → bit[n] = (110>>n)&1.
 *
 * Physics        : Wolfram classes:
 *                  Class 1 (Fixed)    — all initial conditions → same state
 *                  Class 2 (Periodic) — stable or periodic patterns
 *                  Class 3 (Chaotic)  — pseudo-random, sensitive to init
 *                  Class 4 (Complex)  — localised propagating structures
 *                  Class 5 (Fractal)  — Sierpinski-like self-similarity
 *                  Rule 110 is Class 4 and is Turing-complete (Cook 2004).
 *
 * Performance    : Each row update is O(cols) — single pass with bitwise
 *                  neighbourhood extraction.  No double buffer needed since
 *                  rows are computed top-to-bottom from the previous row only.
 *
 * References     :
 *
 *   CONCEPTS — elementary cellular automata
 *   • Wolfram, S. (1983) — "Statistical mechanics of cellular automata",
 *     *Reviews of Modern Physics* 55:601-644.  The foundational paper: the
 *     2³→256 rule numbering and the rule-as-lookup-table scheme used here.
 *   • Wolfram, S. (1984) — "Universality and complexity in cellular
 *     automata", *Physica D* 10:1-35.  The four behaviour classes
 *     (fixed / periodic / chaotic / complex) this demo colour-codes
 *     (we add a 5th "fractal" bucket for Sierpinski-type rules).
 *   • Cook, M. (2004) — "Universality in Elementary Cellular Automata",
 *     *Complex Systems* 15(1):1-40.  Proof that Rule 110 is Turing-complete.
 *   • Wolfram, S. (2002) — *A New Kind of Science*, Wolfram Media.  The
 *     comprehensive reference; Rule 30 as an RNG, Rule 90 → Sierpinski,
 *     Rule 110 gliders.  Browseable free at wolframscience.com.
 *
 *   RENDERING — terminal output
 *   • Padala, P. (2005) — "NCURSES Programming HOWTO", TLDP
 *     (tldp.org/HOWTO/NCURSES-Programming-HOWTO/).  The colour-pair,
 *     attribute, and double-buffered (wnoutrefresh/doupdate) model used
 *     in §3/§6.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A whole universe of one-dimensional patterns lives inside a single byte.
 * Take any 8-bit number 0-255, treat its bits as a tiny truth-table indexed
 * by the three cells above (left, centre, right), and you have a complete
 * deterministic rule for evolving an infinite row of cells. The screen is
 * just that row stamped over and over, one generation per terminal line.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a row of dominoes, each either standing or fallen. To produce the
 * next row, every domino looks at itself and its two neighbours, finds that
 * 3-bit pattern (one of 8 possibilities) on a small lookup card, and sets
 * its successor accordingly. The "card" is the rule number printed in
 * binary. Stack 100 such rows and you get either Sierpinski triangles
 * (rule 90), pseudo-random noise (rule 30), or gliders (rule 110).
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Seed generation 0: usually a single live cell at the centre column.
 *  2. To compute generation g+1 from g, for each column c:
 *       a. Read l = row[c-1], m = row[c], rv = row[c+1] (wrap toroidally).
 *       b. Form the 3-bit index n = (l<<2)|(m<<1)|rv  ∈ [0,7].
 *       c. New cell = (rule >> n) & 1.              ← cell_next()
 *  3. Each tick adds one generation (every ctl.delay ticks).
 *  4. When the last generation is reached, freeze, hold for PAUSE_TICKS,
 *     then auto-advance to the next preset (if auto is on).
 *  5. Render: generation g maps to terminal row 1+g; the class colour comes
 *     from ca_classify().
 *
 * KEY FORMULAS
 * ────────────
 *  n = (l<<2)|(m<<1)|r          neighbourhood → 3-bit index 0..7
 *  next = (rule >> n) & 1       Wolfram lookup: rule's n-th bit
 *  rule 110 = 0b01101110        bit pattern that yields Turing-completeness
 *  wrap: src[(c-1+cols) % cols] toroidal boundary, no edge artefacts
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Rule 0 and 255 are degenerate (all-dead / all-alive after one step) —
 *    kept in the preset list as visual sanity checks.
 *  • Toroidal wrap means a "centre" seed is only centred for ONE row;
 *    after a few generations the pattern can re-enter from the opposite edge.
 *  • Each generation is computed top-down once; resizing the terminal re-fits
 *    the grid and re-centres the seed (app_fit → scene_reseed).
 *  • Random seed (R) populates row 0 with rand()&1 — chaotic rules look
 *    drastically different from centre seed, fixed/fractal ones may not.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Rule 90 must show a Sierpinski triangle from a centre seed —
 *    canonical fractal sanity check.
 *  • Rule 30's centre column should look statistically random (Wolfram
 *    used it as the Mathematica RNG until 2002).
 *  • Rule 0 collapses to all-zero on row 1; rule 255 to all-ones.
 *  • The classification colour at the top must match the visual character
 *    of the pattern (chaotic-orange for rule 30, fractal-yellow for 90).
 *  • Pressing + halves the rows-per-row delay; the bottom of the screen
 *    should fill in roughly half the time.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — SIMULATION / LOGIC / EFFECTS / DELAYS / RENDER ─────── *
 *
 * One rule keeps the layers clear: a function's SIGNATURE says whether it
 * can change the world.
 *   • non-const pointer (Automaton*, Scene*, App*) → an EFFECT; may mutate.
 *   • const pointer / by-value                     → a pure read / render.
 *
 * SIMULATION — the automaton itself (§5):
 *   ca_step         compute one generation from the row above (the effect)
 *   ca_seed_center / ca_seed_random   lay down generation 0
 *
 * LOGIC — stateless rules (§4), no mutation:
 *   cell_next       the Wolfram lookup: 3-cell neighbourhood → next state
 *   ca_classify     rule number → Wolfram behaviour class (1..5)
 *
 * EFFECTS — everything that mutates state, behind non-const pointers:
 *   the cell grid + generation counter (ca_step / seeds), and the control
 *   knobs (scene_set_preset / scene_set_theme / scene_reseed / scene_tick).
 *
 * DELAYS — the only time-bending, in the control + main loop:
 *   row pacing   one new generation every ctl.delay ticks
 *   hold         freeze a finished pattern for PAUSE_TICKS, then advance
 *   frame cap    main() sleeps to a fixed TICK_NS (~30 fps)
 *
 * RENDERING (§6) is PURE: render_* read a const Scene and write only the
 * terminal — the data bar (row 0), the CA grid (rows 1..n-2), and the
 * action bar (row n-1) are three independent draws that mutate nothing.
 *
 * PERFORMANCE — modest by design: each generation is O(cols), a single
 * bitwise pass; no double buffer is needed because every generation is kept
 * (the grid is the history) and each row is computed once from the one above.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

/* ===================================================================== */
/* §1  config — tunables, presets, colour-pair IDs                        */
/* ===================================================================== */

#define TICKS_PER_SEC 30                             /* sim/render tick rate */
#define TICK_NS      (1000000000LL / TICKS_PER_SEC)  /* ~30 fps frame cap    */
#define MAX_ROWS     128         /* grid depth; ≥ any terminal height     */
#define MAX_COLS     320         /* grid width; ≥ any terminal width      */
#define DELAY_DEF    3           /* ticks between rows (~10 rows/sec)     */
#define DELAY_MIN    1           /* fastest: 1 tick / row                 */
#define DELAY_MAX    30          /* slowest: 30 ticks / row               */
#define PAUSE_TICKS  (3 * TICKS_PER_SEC)  /* hold a finished pattern ~3 s */
#define LIVE_CHAR    '#'

/* HUD reserves the top row (data bar) and the bottom row (action bar);
 * the CA fills the rows between. */
#define HUD_TOP_ROWS 1
#define HUD_BOT_ROWS 1
#define HUD_ROWS     (HUD_TOP_ROWS + HUD_BOT_ROWS)

/* element count of a fixed array. */
#define COUNT_OF(a)  ((int)(sizeof (a) / sizeof (a)[0]))

/* color-pair IDs: CP_CL1..5 = the five Wolfram classes (data bar + grid);
 * CP_HINT = the action bar. */
enum { CP_CL1 = 1, CP_CL2, CP_CL3, CP_CL4, CP_CL5, CP_HINT };

/*
 * Preset bank — a curated tour of 17 of the 256 rules.  Most rules are
 * visually dull; these are the famous / striking ones.  n/p step through
 * this list and auto-advance cycles it; the order is a deliberate tour, not
 * numeric.  A "preset" is just a rule with a name — see scene_set_preset.
 */
#define N_PRESETS 17
static const struct {
    int         rule;   /* the Wolfram rule number 0..255 */
    const char *desc;   /* short HUD caption              */
} PRESETS[N_PRESETS] = {
    {  30, "Chaos / RNG"       },
    {  90, "Sierpinski"        },
    { 110, "Turing-complete"   },
    {  18, "Sierpinski-like"   },
    { 150, "Pascal mod 2"      },
    {  60, "XOR fractal"       },
    {  54, "Complex"           },
    { 105, "Complex / fractal" },
    { 106, "Complex"           },
    {  45, "Chaotic"           },
    {  22, "Chaotic"           },
    { 126, "Chaotic"           },
    {  57, "Complex"           },
    {  73, "Complex"           },
    {  99, "Complex"           },
    {   0, "All zeros"         },
    { 255, "All ones"          },
};

/* ===================================================================== */
/* §2  clock — the program's frame DELAY lives behind these               */
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
/* §3  color — theme → ncurses pairs (a terminal effect)                  */
/* ===================================================================== */

/*
 * Theme — a palette for the five Wolfram classes plus the action bar.  The
 * colour ENCODES the class (fixed/periodic/chaotic/complex/fractal), so every
 * theme keeps five distinct entries; cycling t/T changes the mood, not the
 * meaning.  All entries sit in the bright half of the 256-cube.
 */
#define N_THEMES 5
typedef struct {
    const char *name;
    short cls[5];     /* classes 1..5: fixed, periodic, chaotic, complex, fractal */
    short hint;       /* action-bar colour                                        */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "CLASSIC", { 244,  51, 202,  82, 226 },  51 },  /* grey/cyan/orange/green/yellow */
    { "NEON   ", { 201,  51, 226,  46, 208 }, 213 },  /* vivid magenta…orange          */
    { "EMBER  ", { 223, 215, 208, 202, 196 }, 220 },  /* warm light→red ramp           */
    { "ICE    ", { 195, 159, 123,  87,  51 }, 159 },  /* cool light→cyan ramp          */
    { "MONO   ", { 242, 246, 250, 253, 255 }, 248 },  /* greyscale tiers               */
};

/* bind the five class pairs + the hint pair from a 256-colour theme. */
static void theme_bind_256(const Theme *t)
{
    init_pair(CP_CL1, t->cls[0], -1);
    init_pair(CP_CL2, t->cls[1], -1);
    init_pair(CP_CL3, t->cls[2], -1);
    init_pair(CP_CL4, t->cls[3], -1);
    init_pair(CP_CL5, t->cls[4], -1);
    init_pair(CP_HINT, t->hint, -1);
}

/* 8-colour fallback: fixed class hues, theme-independent. */
static void theme_bind_8(void)
{
    init_pair(CP_CL1, COLOR_WHITE,  -1);
    init_pair(CP_CL2, COLOR_CYAN,   -1);
    init_pair(CP_CL3, COLOR_RED,    -1);
    init_pair(CP_CL4, COLOR_GREEN,  -1);
    init_pair(CP_CL5, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN,  -1);
}

/* apply theme `idx` to the colour pairs (a terminal effect). */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_bind_256(&THEMES[idx]);
    else               theme_bind_8();
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(0);
}

/* ===================================================================== */
/* §4  model — STATE (Automaton + Control + Scene) and pure reads         */
/* ===================================================================== */

/* Wolfram's behaviour classes (Wolfram 1984) — what a rule does over time. */
typedef enum {
    CLASS_FIXED    = 1,   /* converges to a uniform state         */
    CLASS_PERIODIC = 2,   /* stable or repeating patterns         */
    CLASS_CHAOTIC  = 3,   /* pseudo-random, sensitive to the seed */
    CLASS_COMPLEX  = 4,   /* localised propagating structures     */
    CLASS_FRACTAL  = 5,   /* Sierpinski-like self-similarity      */
} WolframClass;

/* how generation 0 is laid down — the initial condition the rule evolves. */
typedef enum {
    SEED_CENTER,   /* one live cell at the centre column (the canonical seed) */
    SEED_RANDOM,   /* every cell a coin-flip (chaotic rules diverge wildly)   */
} SeedKind;

/*
 * Automaton — the elementary cellular automaton's state (Wolfram 1983).
 * `cells` stores the WHOLE run, not just the live row: row g is generation
 * g.  That doubles as the on-screen image AND removes the need for a double
 * buffer — each generation is read from the row above and written exactly
 * once, top to bottom (see ca_step).  Static, sized for the largest terminal.
 */
typedef struct {
    uint8_t cells[MAX_ROWS][MAX_COLS]; /* cells[gen][col] ∈ {0,1}; gen 0 = seed */
    int     w;     /* live row width = terminal columns (≤ MAX_COLS)         */
    int     gens;  /* generations that fit = CA area height (≤ MAX_ROWS)     */
    int     gen;   /* highest generation computed so far (0 right after seed)*/
    int          rule; /* the active Wolfram rule, 0..255 (the lookup table) */
    WolframClass cls;  /* its behaviour class, cached so colour is an O(1) lookup */
} Automaton;

/*
 * Phase — the two-state run cycle.  BUILD adds one generation per delay
 * window until the screen fills; HOLD then freezes the finished pattern for
 * PAUSE_TICKS before the next preset (or a redraw).  scene_tick switches them.
 */
typedef enum { PHASE_BUILD, PHASE_HOLD } Phase;

/*
 * Control — the UI knobs and the phase state machine.  Kept apart from the
 * Automaton so input mutates THESE while the simulation mutates the grid.
 */
typedef struct {
    int   preset;       /* index into PRESETS                              */
    int   theme;        /* index into THEMES                               */
    int   delay;        /* ticks between rows (speed; DELAY_MIN..MAX)       */
    bool  paused;       /* freeze stepping; render keeps running           */
    bool  auto_advance; /* on completion: next preset (vs. redraw same)    */
    Phase phase;        /* BUILD = filling; HOLD = done, counting down     */
    int   delay_ctr;    /* ticks since the last row was added              */
    int   hold_ctr;     /* ticks held since the pattern completed          */
} Control;

/*
 * Scene — one running instance: the simulated automaton plus the control
 * that steers it.  This is the unit the effects mutate (Scene*) and the
 * renderer reads (const Scene*); App wraps it with the terminal/loop state.
 */
typedef struct {
    Automaton ca;    /* the simulation (§5 advances it)  */
    Control   ctl;   /* the UI knobs + phase machine     */
} Scene;

/* ── pure reads / logic ──────────────────────────────────────────────── */

/* is rule `r` listed in the class-membership table `set`? */
static bool rule_in_set(int r, const int *set, int n)
{
    for (int i = 0; i < n; i++) if (set[i] == r) return true;
    return false;
}

/* Wolfram class of a rule — first matching membership table wins, else periodic. */
static WolframClass ca_classify(int r)
{
    static const int cl5[] = { 18, 60, 90, 105, 150 };
    static const int cl4[] = { 54, 57, 62, 73, 99, 106, 110 };
    static const int cl3[] = { 22, 30, 45, 75, 89, 109, 126, 135, 149, 153, 154 };
    static const int cl1[] = { 0, 8, 32, 40, 128, 136, 160, 168, 255 };
    if (rule_in_set(r, cl5, COUNT_OF(cl5))) return CLASS_FRACTAL;
    if (rule_in_set(r, cl4, COUNT_OF(cl4))) return CLASS_COMPLEX;
    if (rule_in_set(r, cl3, COUNT_OF(cl3))) return CLASS_CHAOTIC;
    if (rule_in_set(r, cl1, COUNT_OF(cl1))) return CLASS_FIXED;
    return CLASS_PERIODIC;
}

static const char *class_name(WolframClass cls)
{
    switch (cls) {
        case CLASS_FIXED:    return "Fixed";
        case CLASS_PERIODIC: return "Periodic";
        case CLASS_CHAOTIC:  return "Chaotic";
        case CLASS_COMPLEX:  return "Complex";
        case CLASS_FRACTAL:  return "Fractal";
    }
    return "Unknown";
}

/* the colour pair that encodes a class on the data bar + grid. */
static int class_cp(WolframClass cls)
{
    switch (cls) {
        case CLASS_FIXED:    return CP_CL1;
        case CLASS_PERIODIC: return CP_CL2;
        case CLASS_CHAOTIC:  return CP_CL3;
        case CLASS_COMPLEX:  return CP_CL4;
        case CLASS_FRACTAL:  return CP_CL5;
    }
    return CP_CL2;
}

/* pack a left/centre/right neighbourhood into its 3-bit code 0..7. */
static int neighbourhood_code(uint8_t l, uint8_t m, uint8_t r)
{
    return (l << 2) | (m << 1) | r;
}

/* read bit `code` of the rule's 8-bit lookup table → the new centre state. */
static uint8_t rule_bit(int rule, int code)
{
    return (uint8_t)((rule >> code) & 1);
}

/* the Wolfram update for one cell: index the rule by the neighbourhood code. */
static uint8_t cell_next(uint8_t l, uint8_t m, uint8_t r, int rule)
{
    return rule_bit(rule, neighbourhood_code(l, m, r));
}

/* fold a column index onto the toroidal row — the wrap-around boundary. */
static int wrap_col(int c, int w) { return (c % w + w) % w; }

/* ===================================================================== */
/* §5  simulation — EFFECTS that advance the automaton + control          */
/* ===================================================================== */

static void ca_clear(Automaton *a)
{
    memset(a->cells, 0, sizeof a->cells);
    a->gen = 0;
}

/* generation 0 = a single live cell at the centre column. */
static void ca_seed_center(Automaton *a)
{
    ca_clear(a);
    if (a->w > 0) a->cells[0][a->w / 2] = 1;
}

/* generation 0 = a random row (chaotic rules diverge wildly from centre). */
static void ca_seed_random(Automaton *a)
{
    ca_clear(a);
    for (int c = 0; c < a->w; c++) a->cells[0][c] = (uint8_t)(rand() & 1);
}

/* ca_step — compute the next generation from the current top row (toroidal). */
static void ca_step(Automaton *a)
{
    if (a->gen >= a->gens - 1) return;             /* screen full */
    const uint8_t *src = a->cells[a->gen];
    uint8_t       *dst = a->cells[a->gen + 1];
    int w = a->w;
    for (int c = 0; c < w; c++) {
        uint8_t l = src[wrap_col(c - 1, w)];
        uint8_t m = src[c];
        uint8_t r = src[wrap_col(c + 1, w)];
        dst[c] = cell_next(l, m, r, a->rule);
    }
    a->gen++;
}

/* (re)seed generation 0 and restart the build phase. */
static void scene_reseed(Scene *s, SeedKind kind)
{
    if (kind == SEED_RANDOM) ca_seed_random(&s->ca);
    else                     ca_seed_center(&s->ca);
    s->ctl.phase     = PHASE_BUILD;
    s->ctl.delay_ctr = 0;
    s->ctl.hold_ctr  = 0;
}

/* select a preset: set its rule + class, then reseed from the centre. */
static void scene_set_preset(Scene *s, int idx)
{
    s->ctl.preset = ((idx % N_PRESETS) + N_PRESETS) % N_PRESETS;
    s->ca.rule    = PRESETS[s->ctl.preset].rule;
    s->ca.cls     = ca_classify(s->ca.rule);
    scene_reseed(s, SEED_CENTER);
}

/* recolour to theme `idx` (does not touch the pattern). */
static void scene_set_theme(Scene *s, int idx)
{
    s->ctl.theme = ((idx % N_THEMES) + N_THEMES) % N_THEMES;
    theme_apply(s->ctl.theme);
}

/* one simulation tick: pace the build, or count down the hold. */
static void scene_tick(Scene *s)
{
    Control *c = &s->ctl;
    if (c->paused) return;

    if (c->phase == PHASE_HOLD) {
        if (++c->hold_ctr >= PAUSE_TICKS) {
            /* Auto on → next preset; off → redraw this same rule. */
            if (c->auto_advance) scene_set_preset(s, c->preset + 1);
            else                 scene_reseed(s, SEED_CENTER);
        }
        return;
    }

    /* BUILD: add one generation every `delay` ticks. */
    if (++c->delay_ctr >= c->delay) {
        c->delay_ctr = 0;
        ca_step(&s->ca);
        if (s->ca.gen >= s->ca.gens - 1) {
            c->phase    = PHASE_HOLD;
            c->hold_ctr = 0;
        }
    }
}

static void scene_init(Scene *s)
{
    s->ctl.preset       = 0;
    s->ctl.theme        = 0;
    s->ctl.delay        = DELAY_DEF;
    s->ctl.paused       = false;
    s->ctl.auto_advance = true;
    scene_set_preset(s, 0);      /* sets rule/class + seeds + resets phase */
}

/* speed = rows per second: fewer ticks/row is faster, more is slower. */
static void ctl_speed_up  (Control *c) { if (c->delay > DELAY_MIN) c->delay--; }
static void ctl_speed_down(Control *c) { if (c->delay < DELAY_MAX) c->delay++; }

/* ===================================================================== */
/* §6  render — PURE: const Scene → screen                                */
/* ===================================================================== */

/*
 * hud_bar — draw one full-width bar on `row`, filled in `attr`, with the
 * text clipped so it can never wrap onto the CA area.  Leaves the last
 * column untouched (avoids the bottom-right corner scroll quirk).
 */
static void hud_bar(int row, int cols, chtype attr, const char *buf)
{
    if (row < 0 || cols < 2) return;
    int w = cols - 1;
    attron(attr);
    for (int x = 0; x < w; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, w);
    attroff(attr);
}

/* the status tail on the data bar: paused, or the hold countdown in seconds
 * (empty while building).  hold_ctr counts up in ticks; convert with a
 * ceil-division to whole seconds. */
static void phase_status(char *buf, size_t n, const Control *c)
{
    if (c->paused) {
        snprintf(buf, n, "  [PAUSED]");
    } else if (c->phase == PHASE_HOLD) {
        int secs = (PAUSE_TICKS - c->hold_ctr + TICKS_PER_SEC - 1) / TICKS_PER_SEC;
        snprintf(buf, n, "  [%s in %ds]", c->auto_advance ? "next" : "redraw", secs);
    } else {
        buf[0] = '\0';
    }
}

/* Top DATA bar (row 0), coloured by the rule's Wolfram class. */
static void render_data_bar(const Scene *s, int cols)
{
    const Automaton *a = &s->ca;
    const Control   *c = &s->ctl;

    char status[48];
    phase_status(status, sizeof status, c);

    char buf[220];
    snprintf(buf, sizeof buf,
             " 1D_CA  rule:%-3d %-18s class:%-8s  preset:%2d/%d  theme:%s  "
             "gen:%d/%d  spd:%d  auto:%-3s%s ",
             a->rule, PRESETS[c->preset].desc, class_name(a->cls),
             c->preset + 1, N_PRESETS, THEMES[c->theme].name,
             a->gen, a->gens, c->delay, c->auto_advance ? "on" : "off", status);

    hud_bar(0, cols, COLOR_PAIR(class_cp(a->cls)) | A_BOLD | A_REVERSE, buf);
}

/* The CA grid: generation g → terminal row 1+g, drawn in the class colour. */
static void render_grid(const Scene *s, int cols)
{
    const Automaton *a = &s->ca;
    chtype attr = COLOR_PAIR(class_cp(a->cls));
    int w = a->w < cols ? a->w : cols;

    attron(attr);
    for (int g = 0; g <= a->gen && g < a->gens; g++) {
        const uint8_t *row = a->cells[g];
        for (int c = 0; c < w - 1; c++)
            mvaddch(HUD_TOP_ROWS + g, c, row[c] ? (chtype)(unsigned char)LIVE_CHAR : ' ');
    }
    attroff(attr);
}

/* Bottom ACTION bar (row n-1): every interactive key. */
static void render_action_bar(int cols, int rows)
{
    static const char *keys =
        " n/p:preset  t/T:theme  a:auto  r:seed  R:rand  +/-:speed  spc:pause  q:quit ";
    hud_bar(rows - 1, cols, COLOR_PAIR(CP_HINT) | A_BOLD, keys);
}

/* one frame: erase → data bar → grid → action bar → flush (one diff write). */
static void render_frame(const Scene *s, int cols, int rows)
{
    erase();
    render_data_bar(s, cols);
    render_grid(s, cols);
    render_action_bar(cols, rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  app — orchestration: input → effects → delay → render              */
/* ===================================================================== */

/*
 * App — the running PROCESS around the Scene: the terminal size and the
 * loop's flags.  Split from Scene because these describe the program, not
 * the automaton.  One file-scope instance (g_app) so the signal handler can
 * reach the flags.
 */
typedef struct {
    Scene scene;               /* the running automaton (§4-§6)              */
    int   rows, cols;          /* terminal size in character cells           */
    volatile sig_atomic_t running;     /* main-loop flag, 0 = quit; written by
                                        * SIGINT/SIGTERM (async-signal-safe)   */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH, drained atop loop   */
} App;

static App g_app;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

/* read the terminal size and size the CA grid to fit: row width = cols,
 * generations = the rows between the two HUD bars. */
static void app_fit(App *app)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    app->rows = rows < MAX_ROWS ? rows : MAX_ROWS;
    app->cols = cols < MAX_COLS ? cols : MAX_COLS;
    app->scene.ca.w    = app->cols;
    app->scene.ca.gens = (app->rows - HUD_ROWS < 1) ? 1 : app->rows - HUD_ROWS;
}

/* map a keypress to an intent; some fire §5 effects. */
static void app_handle_key(App *app, int ch)
{
    Scene   *s = &app->scene;
    Control *c = &s->ctl;
    switch (ch) {
    case 'q': case 'Q': app->running = 0;                    break;
    case ' ':           c->paused = !c->paused;              break;
    case 'a': case 'A': c->auto_advance = !c->auto_advance;  break;
    case 'n':           scene_set_preset(s, c->preset + 1);  break;
    case 'p':           scene_set_preset(s, c->preset - 1);  break;
    case 't':           scene_set_theme (s, c->theme + 1);   break;
    case 'T':           scene_set_theme (s, c->theme - 1);   break;
    case 'r':           scene_reseed(s, SEED_CENTER);              break;
    case 'R':           scene_reseed(s, SEED_RANDOM);               break;
    case '+': case '=': ctl_speed_up(c);                     break;
    case '-': case '_': ctl_speed_down(c);                   break;
    default: break;
    }
}

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));

    App *app     = &g_app;
    app->running = 1;

    screen_init();
    app_fit(app);                /* terminal size → CA dims (before seeding) */
    scene_init(&app->scene);     /* control defaults + preset 0 + centre seed */

    long long next = clock_ns();

    /*
     * Fixed-step main loop — each iteration reads as four named steps:
     *   INPUT   drain getch() → app_handle_key (may fire §5 effects)
     *   EFFECTS scene_tick — pace one generation / count the hold
     *   RENDER  render_frame — pure paint
     *   DELAY   sleep to the next TICK_NS (~30 fps)
     * On SIGWINCH the grid is re-fitted and re-centred.
     */
    while (app->running) {
        if (app->need_resize) {
            app->need_resize = 0;
            endwin();
            refresh();
            app_fit(app);
            scene_reseed(&app->scene, SEED_CENTER);
        }

        int ch;                                              /* INPUT   */
        while ((ch = getch()) != ERR) app_handle_key(app, ch);

        scene_tick(&app->scene);                             /* EFFECTS */
        render_frame(&app->scene, app->cols, app->rows);     /* RENDER  */

        next += TICK_NS;                                     /* DELAY   */
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
