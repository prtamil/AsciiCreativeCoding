/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * l_system.c — L-System fractal generator (string rewriting + turtle graphics)
 *
 * FOUR LAYERS, deliberately separated so the fractal logic is trustworthy and
 * the cosmetic timing is accountable:
 *
 *   §4 GRAMMAR   — the CORE: pure L-system string rewriting.  Given a preset's
 *                  axiom + rules it produces generation N's string.  It knows
 *                  nothing about time, drawing, scale, or colour — so the math
 *                  that defines the fractal can never be disturbed by a visual
 *                  tweak.  This is the safe core.
 *   §5 ANIMATION — the EFFECTS & DELAYS: a dumb timer that tracks how much of the
 *                  path to reveal (the draw-in) and how long to hold a finished
 *                  generation.  Pure presentation; it never touches the grammar.
 *   §6 RENDER    — turtle-walks the string into a 2-D path, fits it to the
 *                  screen, and draws the revealed portion with the theme colours.
 *                  Reads the grammar / fit / animation; mutates none of them.
 *   §7 SCENE     — orchestration: ties grammar + animation + render together and
 *                  runs the generation state machine (draw -> hold -> advance).
 *
 * Five presets, each built generation by generation; every generation draws
 * itself in progressively, holds briefly, then advances — transitions animate
 * instead of popping between scenes.
 *
 *   0  Dragon Curve      — self-similar fold;        90 turns
 *   1  Hilbert Curve     — space-filling;            90 turns
 *   2  Sierpinski Arrow  — triangle subdivision;     60 turns
 *   3  Branching Plant   — organic fractal tree;     25 turns, branch stack
 *   4  Koch Snowflake    — edge-replacement;         60 turns
 *
 * L-SYSTEM NOTATION
 *   Variables : letters rewritten each generation (X, Y, A, B …)
 *   Constants : symbols passed through unchanged (+, -, [, ], F …)
 *   Axiom     : the starting string (generation 0)
 *   Rules     : variable -> expansion string, applied to every match each gen.
 *
 * TURTLE SYMBOLS
 *   F / A / B   move forward one step, drawing a segment
 *   +           turn left  by angle delta        -   turn right by angle delta
 *   [           push position + heading (branch)  ]  pop  position + heading
 *   X / Y       rewrite-only; no turtle action
 *
 * AUTO-SCALING (§6 fit)
 *   Each generation grows exponentially, so the path is re-fitted: dry-run the
 *   turtle to find its bounding box, then scale = min(usable_w/bbox_w,
 *   usable_h/bbox_h) and centre (or bottom-pin the Branching Plant).
 *
 * ASPECT CORRECTION (§6)
 *   Terminal cells are ~2x taller than wide, so turtle Y-motion is multiplied by
 *   ASPECT = CELL_W/CELL_H (~0.5) to keep diagonals isotropic.
 *
 * Keys:
 *   q / ESC    quit                     space  pause / resume the draw-in
 *   Enter      step to next generation   n / p  next / previous preset
 *   t / T      cycle colour theme        r      restart at generation 0
 *   + / =      faster draw-in            -      slower draw-in
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra l_system.c -o l_system -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config    — presets, constants, animation tuning
 *   §2  clock     — monotonic ns clock + FrameClock (fixed-timestep pacing)
 *   §3  color     — themeable 7-stop branch-depth ramp (t/T) + HUD pairs
 *   §4  grammar   — L-system string rewriting (PURE CORE LOGIC)
 *   §5  animation — reveal draw-in + inter-generation hold (EFFECTS & DELAYS)
 *   §6  render    — turtle interpreter + screen fit + glyph drawing
 *   §7  scene     — orchestration: generation state machine
 *   §8  screen    — ncurses lifecycle + HUD
 *   §9  app       — main loop
 */

/* ── REFERENCES — for the concepts and the rendering ──────────────────── *
 *
 *   L-systems & turtle interpretation  (§4 grammar, §6 turtle)
 *   ── Lindenmayer, A. (1968). "Mathematical Models for Cellular Interaction
 *      in Development, I & II." J. Theoretical Biology 18, 280-315.  The origin
 *      of L-systems: parallel string rewriting (grammar_expand in §4).
 *   ── Prusinkiewicz, P. & Lindenmayer, A. (1990). "The Algorithmic Beauty of
 *      Plants." Springer.  The canonical treatment of L-systems + turtle
 *      graphics — every preset, the [ ] branch stack, and the depth model here.
 *   ── Prusinkiewicz, P. (1986). "Graphical Applications of L-systems." Proc.
 *      Graphics Interface '86, 247-253.  Introduced the turtle interpretation
 *      of L-system strings — exactly turtle_apply() in §6.
 *   ── Abelson, H. & diSessa, A. (1981). "Turtle Geometry." MIT Press.  The
 *      foundations of turtle graphics (pose, heading, forward/turn) behind the
 *      Turtle struct in §6.
 *
 *   Fractal curves  (the presets)
 *   ── Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *      Self-similarity and fractal dimension — why these curves fill space.
 *   ── von Koch, H. (1904). "Sur une courbe continue sans tangente …" Arkiv
 *      foer Matematik.  The Koch curve — the Koch Snowflake preset.
 *   ── Hilbert, D. (1891). "Ueber die stetige Abbildung einer Linie auf ein
 *      Flaechenstueck." Math. Annalen 38, 459-460.  The space-filling curve —
 *      the Hilbert Curve preset (Hausdorff dimension 2).
 *
 *   Rendering  (§6 render, §8 screen)
 *   ── Bresenham, J. E. (1965). "Algorithm for Computer Control of a Digital
 *      Plotter." IBM Systems Journal 4(1), 25-30.  The DDA line stepping that
 *      put_segment() uses to fill every cell along a segment.
 *   ── Foley, van Dam, Feiner & Hughes. "Computer Graphics: Principles and
 *      Practice."  Window-to-viewport mapping — the basis of fit_compute()'s
 *      bounding-box-to-screen auto-scale.
 *   ── Gookin, D. (2007). "Programmer's Guide to NCURSES." Wiley.  The cell-
 *      drawing API behind §8: init_pair, mvaddch, refresh, resize.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS         = 60,    /* fixed sim tick rate (no continuous physics) */

    N_COLORS        =  7,    /* branch-depth colour stops per theme         */
    N_PRESETS       =  5,
    N_THEMES        =  5,    /* colour themes, cycled with t / T            */
    MAX_RULES       =  4,    /* max rewrite rules per preset                */
    STACK_MAX       = 128,   /* max branch nesting depth                    */
    CHAR_RANGE      = 256,   /* byte values — size of the rule lookup table */

    HUD_TOP_ROWS    =  2,    /* rows 0..1 — data HUD (status + parameters)  */
    HUD_BOT_ROWS    =  1,    /* last row  — action / key-hint bar           */
    FIT_MARGIN_COLS =  2,    /* columns kept clear at the sides when scaling */

    FPS_UPDATE_MS   = 500,
    FRAME_DT_CAP_MS = 100,   /* clamp one frame's dt — stops spiral-of-death */
    RENDER_FPS_CAP  =  60,   /* render-rate ceiling the loop sleeps down to  */
};

/* Maximum L-system string length.  The string grows exponentially; 1 MB covers
 * Koch gen 6, Hilbert gen 6, Branching Plant gen 7.  Stop advancing past this. */
#define MAX_STR  (1 * 1024 * 1024)

/* Animation tuning (the EFFECTS & DELAYS — see §5).  draw_sec is per-preset and
 * runtime-adjustable; the rest bound how far + / - can push it. */
#define GEN_HOLD_SEC  0.6f    /* hold each completed generation this long      */
#define DRAW_SEC_MIN  0.2f    /* fastest draw-in (smallest seconds-per-gen)    */
#define DRAW_SEC_MAX 10.0f    /* slowest draw-in                               */
#define DRAW_FASTER   0.75f   /* '+' multiplies draw_sec by this (quicker)     */
#define DRAW_SLOWER   1.33f   /* '-' multiplies draw_sec by this (slower)      */

/* Render tuning (see §6 fit). */
#define MIN_SCALE     0.5f    /* never shrink turtle steps below half a cell   */

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))
#define TICK_SEC     (1.0f / (float)SIM_FPS)   /* seconds per fixed sim tick */

/* ── Preset table ──────────────────────────────────────────────────── */

/*
 * LRule — one production of the grammar: rewrite every `from` into `to` each
 * generation (Lindenmayer 1968).  `from` is a single variable symbol (e.g. 'X');
 * `to` is the string it expands to.  A symbol with no matching rule is a constant
 * and passes through unchanged.
 */
typedef struct {
    char        from;   /* the variable symbol this rule rewrites   */
    const char *to;     /* its expansion (may contain F + - [ ] …)  */
} LRule;

/*
 * Preset — a complete L-system + how to draw it (Prusinkiewicz & Lindenmayer
 * 1990).  The grammar fields (axiom, rules) define the string; the turtle fields
 * (angle, heading, draw_chars) define how that string becomes a path; the rest is
 * presentation/pacing.  The whole demo is data-driven from this table — adding a
 * fractal is one new row, no code.
 */
typedef struct {
    const char *name;         /* shown in the HUD                                 */
    const char *axiom;        /* the generation-0 string                          */
    LRule       rules[MAX_RULES];
    int         n_rules;      /* rules in use (0..MAX_RULES)                       */
    float       angle_deg;    /* turn delta for + / - (degrees)                   */
    float       start_deg;    /* initial heading; -90 faces up (screen y grows    *
                               * downward), used by the Branching Plant           */
    int         max_gen;      /* last generation to draw (string-size ceiling)    */
    float       advance_sec;  /* default seconds to draw one generation in        */
    const char *draw_chars;   /* symbols that move the turtle forward (draw)      */
    bool        bottom;       /* true -> pin the base to the bottom (plant stem)  */
} Preset;

/*
 * Branching Plant uses start_deg = -90 (facing up; screen y grows downward) and
 * bottom = true so the stem sits at the bottom.  The non-branching curves draw
 * entirely at depth 0, so they show only the theme's first colour stop.
 */
static const Preset PRESETS[N_PRESETS] = {
    { "Dragon Curve",     "FX",
      { {'X',"X+YF+"}, {'Y',"-FX-Y"} }, 2,            90.0f,   0.0f, 12, 1.5f, "F",  false },
    { "Hilbert Curve",    "A",
      { {'A',"+BF-AFA-FB+"}, {'B',"-AF+BFB+FA-"} }, 2, 90.0f,   0.0f,  6, 2.0f, "F",  false },
    { "Sierpinski Arrow", "A",
      { {'A',"B-A-B"}, {'B',"A+B+A"} }, 2,            60.0f,   0.0f,  8, 1.5f, "AB", false },
    { "Branching Plant",  "X",
      { {'X',"F+[[X]-X]-F[-FX]+X"}, {'F',"FF"} }, 2,  25.0f, -90.0f,  6, 2.0f, "F",  true  },
    { "Koch Snowflake",   "F++F++F",
      { {'F',"F-F++F-F"} }, 1,                        60.0f,   0.0f,  5, 2.0f, "F",  false },
};

/* ===================================================================== */
/* §2  clock — monotonic ns clock + fixed-timestep frame pacing           */
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
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/*
 * FrameClock — the fixed-timestep heartbeat.  Real frames arrive irregularly,
 * but the simulation must advance in equal slices: each frame we bank the
 * elapsed real time as `sim_debt`, then repay it one TICK_NS step at a time.
 * The fps_* fields measure a smoothed rate for the HUD.
 */
typedef struct {
    int64_t prev_ns;     /* CLOCK_MONOTONIC stamp at this frame's start          */
    int64_t sim_debt;    /* unspent ns owed to the fixed step; left in [0,TICK)  */
    int64_t fps_window;  /* ns elapsed since the last fps sample was emitted     */
    int     fps_frames;  /* frames counted since the last fps sample             */
    double  fps;         /* smoothed frames/sec, refreshed every FPS_UPDATE_MS   */
} FrameClock;

static void frameclock_init(FrameClock *fc)
{
    fc->prev_ns    = clock_ns();
    fc->sim_debt   = 0;
    fc->fps_window = 0;
    fc->fps_frames = 0;
    fc->fps        = 0.0;
}

static void frameclock_sample_fps(FrameClock *fc, int64_t dt)
{
    fc->fps_frames++;
    fc->fps_window += dt;
    if (fc->fps_window >= FPS_UPDATE_MS * NS_PER_MS) {
        fc->fps        = (double)fc->fps_frames
                       / ((double)fc->fps_window / (double)NS_PER_SEC);
        fc->fps_frames = 0;
        fc->fps_window = 0;
    }
}

/* Open a frame: measure dt since the last one, clamp a long stall, bank it. */
static void frameclock_begin_frame(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->prev_ns;
    fc->prev_ns = now;

    int64_t stall_cap = FRAME_DT_CAP_MS * NS_PER_MS;
    if (dt > stall_cap) dt = stall_cap;

    fc->sim_debt += dt;
    frameclock_sample_fps(fc, dt);
}

/* True (and pays down the debt) while a fixed sim step is owed. */
static bool frameclock_step_due(FrameClock *fc, int64_t tick_ns)
{
    if (fc->sim_debt < tick_ns) return false;
    fc->sim_debt -= tick_ns;
    return true;
}

/* Sleep off the remainder of this frame's budget to cap the render rate. */
static void frameclock_throttle(const FrameClock *fc)
{
    int64_t budget  = NS_PER_SEC / RENDER_FPS_CAP;
    int64_t elapsed = clock_ns() - fc->prev_ns;
    clock_sleep_ns(budget - elapsed);
}

/* ===================================================================== */
/* §3  color — themeable branch-depth ramp + HUD pairs                    */
/* ===================================================================== */

/*
 * Pairs 1..N_COLORS are the branch-depth colours, rebound by theme_apply().
 * depth_attr() picks pair (depth % N_COLORS)+1, with depth 0 (the trunk, and the
 * single colour of non-branching presets) drawn bold.  Two extra slots hold the
 * HUD colours, theme-independent so the text stays legible over any fractal.
 */
enum { PAIR_HUD = 8, PAIR_HINT = 9 };

/*
 * Theme — a named 7-stop palette for the branch-depth ramp, cycled with t / T.
 * fg256[0] (depth 0) is the dominant colour for the non-branching presets; the
 * higher stops tint the branch levels of the Branching Plant.  Every entry sits
 * in the bright half of the 256-cube so the strokes stay legible on black.
 */
typedef struct {
    const char *name;
    int fg256[N_COLORS];   /* xterm-256 codes, depth 0..6 (preferred) */
    int fg8  [N_COLORS];   /* 8-colour ANSI fallback, same order      */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "Spectrum", {  46,  82, 226, 208, 196, 201,  51 },
       { COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_MAGENTA, COLOR_CYAN } },
    { "Fire",     { 226, 220, 214, 208, 202, 196, 160 },
       { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED } },
    { "Ocean",    {  51,  45,  39,  33,  27,  75, 117 },
       { COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN } },
    { "Neon",     { 201, 207, 171, 141,  99,  51,  87 },
       { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN } },
    { "Mono",     { 231, 252, 250, 248, 246, 245, 244 },
       { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE } },
};

/* theme_apply() — bind the branch-depth pairs (1..N_COLORS) to a theme. */
static void theme_apply(int theme)
{
    const Theme *t = &THEMES[theme];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < N_COLORS; i++)
        init_pair((short)(i + 1),
                  (short)(truecolor ? t->fg256[i] : t->fg8[i]), COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* HUD pairs are theme-independent — yellow data, cyan actions */
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, COLOR_BLACK);
        init_pair(PAIR_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(PAIR_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* Spectrum; Scene.theme tracks the active one thereafter */
}

/* Map branch depth to an ncurses attribute (depth 0 = trunk, drawn bold). */
static chtype depth_attr(int depth)
{
    int pair = (depth % N_COLORS) + 1;
    return (chtype)(COLOR_PAIR(pair) | (depth == 0 ? A_BOLD : 0));
}

/* ===================================================================== */
/* §4  grammar — L-system string rewriting (PURE CORE LOGIC)              */
/* ===================================================================== */

/*
 * Grammar — the one piece of state that IS the fractal: the current generation
 * string (Lindenmayer 1968).  A generation is produced by rewriting every
 * variable in parallel, so the string grows exponentially —
 * |L_n| ~ |L_{n-1}| * (average expansion length).  Everything visual is just a
 * reading of this string; nothing here knows about time, colour, scale, or the
 * screen, so the rewriting math is safe from any cosmetic change.
 *
 * cur / nxt are a double buffer: a rewrite fills nxt, then swaps the pointers
 * (no copy).  Both are MAX_STR bytes, allocated once at startup — a documented
 * heap exception, since the strings are far too large and variable for the stack.
 */
typedef struct {
    char  *cur;     /* current generation string (NUL-terminated)  */
    char  *nxt;     /* scratch buffer the next rewrite is built in */
    size_t len;     /* strlen(cur) — grows exponentially with gen  */
    int    gen;     /* generation number (0 = the axiom)           */
} Grammar;

static void grammar_alloc(Grammar *g)
{
    g->cur = malloc(MAX_STR);
    g->nxt = malloc(MAX_STR);
    if (!g->cur || !g->nxt) { endwin(); exit(1); }
}

static void grammar_free(Grammar *g)
{
    free(g->cur); free(g->nxt);
    g->cur = g->nxt = NULL;
}

/* grammar_load() — install the preset's axiom as generation 0. */
static void grammar_load(Grammar *g, const Preset *p)
{
    g->len = strlen(p->axiom);
    memcpy(g->cur, p->axiom, g->len + 1);
    g->gen = 0;
}

/*
 * build_rule_table() — index the preset's rules by symbol so each character is
 * looked up in O(1) (a 256-slot table; NULL = no rule, i.e. a constant symbol).
 */
static void build_rule_table(const Preset *p, const char *table[])
{
    for (int i = 0; i < CHAR_RANGE; i++) table[i] = NULL;
    for (int i = 0; i < p->n_rules; i++)
        table[(unsigned char)p->rules[i].from] = p->rules[i].to;
}

/*
 * emit() — append `n` bytes to the output buffer at *out, or return false if that
 * would overflow MAX_STR (the string-size ceiling that caps how deep we expand).
 */
static bool emit(char *buf, size_t *out, const char *src, size_t n)
{
    if (*out + n >= (size_t)MAX_STR) return false;
    memcpy(buf + *out, src, n);
    *out += n;
    return true;
}

/*
 * grammar_expand() — apply the preset's rules once, producing generation N+1.
 * Reads like the rewrite rule: for each symbol, emit its expansion (or the symbol
 * itself if it is a constant), swap the double buffer, and bump the generation.
 * Returns false (leaving the grammar unchanged) if the result would exceed MAX_STR.
 */
static bool grammar_expand(Grammar *g, const Preset *p)
{
    const char *rule[CHAR_RANGE];
    build_rule_table(p, rule);

    size_t out = 0;
    for (size_t i = 0; i < g->len; i++) {
        unsigned char c   = (unsigned char)g->cur[i];
        const char   *exp = rule[c];
        bool ok = exp ? emit(g->nxt, &out, exp, strlen(exp))
                      : emit(g->nxt, &out, (const char *)&c, 1);
        if (!ok) return false;                 /* hit the MAX_STR ceiling */
    }
    g->nxt[out] = '\0';

    char *tmp = g->cur; g->cur = g->nxt; g->nxt = tmp;   /* swap the double buffer */
    g->len = out;
    g->gen++;
    return true;
}

/* ===================================================================== */
/* §5  animation — reveal draw-in + hold (EFFECTS & DELAYS)               */
/* ===================================================================== */

/*
 * Animation — the entire cosmetic timing layer, isolated here so it is obvious
 * what is "the fractal" (§4) and what is "how it appears over time" (this).  It
 * is a dumb timer: it never reads or writes the grammar.
 *
 *   phase      ANIM_DRAW while the path is drawing in, ANIM_HOLD once complete.
 *   reveal     0..1 fraction of the path the renderer should show (the draw-in).
 *   hold_timer seconds the completed generation has been held (the delay).
 *   draw_sec   seconds to draw one whole generation (the speed; + / - tune it).
 */
typedef enum { ANIM_DRAW, ANIM_HOLD } AnimPhase;

typedef struct {
    AnimPhase phase;
    float     reveal;
    float     hold_timer;
    float     draw_sec;
} Animation;

/* Start drawing a fresh generation from nothing. */
static void animation_begin(Animation *a)
{
    a->phase      = ANIM_DRAW;
    a->reveal     = 0.0f;
    a->hold_timer = 0.0f;
}

/* Jump straight to fully drawn + holding (used for a manual step). */
static void animation_show_full(Animation *a)
{
    a->phase      = ANIM_HOLD;
    a->reveal     = 1.0f;
    a->hold_timer = 0.0f;
}

/*
 * animation_step() — advance the timing by dt seconds.  Returns true when the
 * post-draw hold has elapsed (i.e. the caller should advance a generation).
 *   phase DRAW : grow reveal 0 -> 1 over draw_sec, then switch to HOLD.
 *   phase HOLD : count hold_timer up to GEN_HOLD_SEC.
 */
static bool animation_step(Animation *a, float dt)
{
    if (a->phase == ANIM_DRAW) {
        a->reveal += dt / a->draw_sec;
        if (a->reveal >= 1.0f) {
            a->reveal     = 1.0f;
            a->phase      = ANIM_HOLD;
            a->hold_timer = 0.0f;
        }
        return false;
    }
    a->hold_timer += dt;
    return a->hold_timer >= GEN_HOLD_SEC;
}

/* anim_speed() — the draw-in RATE shown in the HUD: the inverse of draw_sec (an
 * interval), so a bigger number means faster, matching the '+' key. */
static float anim_speed(const Animation *a) { return 1.0f / a->draw_sec; }

/* ===================================================================== */
/* §6  render — turtle interpreter + screen fit + glyph drawing           */
/* ===================================================================== */

/*
 * Terminal cells are ~2x taller than wide, so a turtle moving "down" covers half
 * the rows that moving "right" covers columns.  Multiplying Y motion by ASPECT
 * (= CELL_W/CELL_H) keeps diagonals and angles isotropic.  Applied identically
 * in the measure and draw passes so the computed scale is correct on both axes.
 */
#define CELL_W   8
#define CELL_H  16
#define ASPECT   ((float)CELL_W / (float)CELL_H)   /* ~0.5 */

/* radians() — degrees -> radians, the unit the turtle turns and heads in. */
static float radians(float deg) { return deg * (float)M_PI / 180.0f; }

/* ── geometry types ──────────────────────────────────────────────────── */

/*
 * BBox — an axis-aligned bounding box measured in UNIT-step turtle space (step =
 * 1), so it is resolution-independent; fit_compute() scales it to the screen
 * afterwards.  bbox_reset() seeds it inverted (min = +inf, max = -inf) so the
 * first bbox_extend() always overwrites it.
 */
typedef struct {
    float minx, miny;   /* lowest  x / y any part of the path reaches */
    float maxx, maxy;   /* highest x / y any part of the path reaches */
} BBox;

static void bbox_reset(BBox *b)
{
    b->minx = b->miny =  1e30f;
    b->maxx = b->maxy = -1e30f;
}

static void bbox_extend(BBox *b, float x, float y)
{
    if (x < b->minx) b->minx = x;
    if (x > b->maxx) b->maxx = x;
    if (y < b->miny) b->miny = y;
    if (y > b->maxy) b->maxy = y;
}

/*
 * Segment — the value a single 'F' produces: a straight move.  Carrying it as a
 * value is what keeps the turtle (which knows geometry) and the renderer (which
 * knows the screen) decoupled: turtle_forward() returns one, and the caller
 * either measures it (bbox) or draws it (put_segment).
 */
typedef struct {
    float x0, y0;    /* start cell — the turtle's pose before the step */
    float x1, y1;    /* end cell   — its pose after the step           */
    float angle;     /* heading (radians) — chooses the line glyph     */
    int   depth;     /* branch depth when drawn — chooses the colour   */
} Segment;

/*
 * Fit — the window-to-viewport transform (Foley et al.): maps the path's unit-step
 * bounding box onto the usable screen band (the screen minus the HUD rows) as a
 * single uniform scale + offset.  Recomputed per generation, since each one has a
 * different extent.  n_segments lets the renderer turn the animation's reveal
 * fraction into a concrete segment count.
 */
typedef struct {
    float scale;       /* screen cells per unit turtle step                */
    float ox, oy;      /* offset (cells) that centres / pins the path      */
    int   n_segments;  /* total forward segments in the current generation */
} Fit;

/* ── the turtle ──────────────────────────────────────────────────────── */

/*
 * TurtleState — one turtle pose.  This is exactly what '[' saves and ']' restores,
 * so a branch resumes at the precise position, heading, AND colour depth it forked
 * from.
 */
typedef struct {
    float x, y;     /* position in turtle space (cells once scaled)     */
    float angle;    /* heading in radians (0 = +x, increases anticlockwise) */
    int   depth;    /* branch nesting: 0 = trunk, +1 per enclosing '['  */
} TurtleState;

/*
 * Turtle — the drawing agent of turtle graphics (Abelson & diSessa 1981): a
 * current pose plus a stack of saved poses for branches.  This little machine IS
 * the L-system interpreter's mechanics (Prusinkiewicz 1986): 'F' steps forward
 * leaving a Segment, '+'/'-' rotate, '[' saves a pose (start a branch), ']'
 * restores it (end one).  The push/pop stack makes drawing a depth-first walk of
 * the fractal's branch tree.
 */
typedef struct {
    TurtleState cur;                /* the turtle's current pose            */
    TurtleState stack[STACK_MAX];   /* branch stack: poses saved by '['     */
    int         sp;                 /* stack depth (0..STACK_MAX)           */
} Turtle;

static void turtle_init(Turtle *t, float heading_deg)
{
    t->cur = (TurtleState){ 0.0f, 0.0f, radians(heading_deg), 0 };
    t->sp  = 0;
}

static void turtle_turn(Turtle *t, float delta) { t->cur.angle += delta; }

/* '[' — start a branch: save the pose, then descend one depth (for colouring). */
static void turtle_push(Turtle *t)
{
    if (t->sp < STACK_MAX) t->stack[t->sp++] = t->cur;
    t->cur.depth++;
}

/* ']' — end a branch: pop back to the saved pose (restoring its depth too). */
static void turtle_pop(Turtle *t)
{
    if (t->sp > 0) t->cur = t->stack[--t->sp];
}

/* 'F' — step forward by `step`, returning the segment drawn (aspect-corrected). */
static Segment turtle_forward(Turtle *t, float step)
{
    float nx = t->cur.x + cosf(t->cur.angle) * step;
    float ny = t->cur.y + sinf(t->cur.angle) * step * ASPECT;
    Segment s = { t->cur.x, t->cur.y, nx, ny, t->cur.angle, t->cur.depth };
    t->cur.x = nx;
    t->cur.y = ny;
    return s;
}

/*
 * turtle_apply() — the L-system symbol -> turtle action rule, the heart of the
 * interpreter.  Returns true and fills *seg when the symbol drew a forward
 * segment; otherwise it only turned or branched the turtle.  Shared by both
 * passes so the measure and the draw can never diverge.
 */
static bool turtle_apply(Turtle *t, char c, const Preset *p,
                         float step, float delta, Segment *seg)
{
    if (strchr(p->draw_chars, c)) { *seg = turtle_forward(t, step); return true; }
    if      (c == '+') turtle_turn(t, +delta);
    else if (c == '-') turtle_turn(t, -delta);
    else if (c == '[') turtle_push(t);
    else if (c == ']') turtle_pop(t);
    /* X, Y, A, B (rewrite-only variables) and unknowns: no turtle action */
    return false;
}

/* ── render primitives ───────────────────────────────────────────────── */

/*
 * angle_char() — the ASCII glyph that best shows a segment's direction.
 * Modulus pi first, since '-' and '|' are direction-symmetric.
 */
static char angle_char(float angle)
{
    float a = fmodf(angle, (float)M_PI);
    if (a < 0.0f) a += (float)M_PI;
    if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f) return '-';
    if (a < 3.0f * (float)M_PI / 8.0f)                            return '/';
    if (a < 5.0f * (float)M_PI / 8.0f)                            return '|';
    return '\\';
}

/*
 * dda_steps() — how many cells a segment spans: the longer of |dx|,|dy| (the DDA
 * / Bresenham step count).  At least 1, so a zero-length segment still plots one
 * point.
 */
static int dda_steps(float dx, float dy)
{
    int n = (int)(fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy));
    return n < 1 ? 1 : n;
}

/* in_canvas() — is (cx,cy) inside the drawable band: on screen and clear of the
 * HUD rows top and bottom? */
static bool in_canvas(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols && cy >= HUD_TOP_ROWS && cy < rows - HUD_BOT_ROWS;
}

/*
 * put_segment() — stamp one screen segment with a DDA so every cell along it is
 * filled (no gaps when the step is large in early generations).  Walks `steps`+1
 * evenly-spaced points and plots those that land in the canvas.
 */
static void put_segment(WINDOW *w, float x0, float y0, float x1, float y1,
                        chtype attr, float angle, int cols, int rows)
{
    float dx = x1 - x0, dy = y1 - y0;
    int   steps = dda_steps(dx, dy);
    char  ch = angle_char(angle);

    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)steps;          /* 0..1 along the segment */
        int   cx = (int)roundf(x0 + dx * t);
        int   cy = (int)roundf(y0 + dy * t);
        if (in_canvas(cx, cy, cols, rows)) {
            wattron(w, attr);
            mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
            wattroff(w, attr);
        }
    }
}

/* ── passes: measure (geometry) and draw (render) ────────────────────── */

/*
 * turtle_measure() — walk the WHOLE string at unit step, reporting the path's
 * bounding box and forward-segment count.  Pure geometry: no screen, no colour.
 * Reads like the algorithm — drive the turtle over every symbol, growing the box
 * around each segment it draws.
 */
static void turtle_measure(const Grammar *g, const Preset *p,
                           BBox *bbox, int *n_segments)
{
    Turtle t; turtle_init(&t, p->start_deg);
    float  delta = radians(p->angle_deg);
    int    segs  = 0;

    bbox_reset(bbox);
    for (size_t i = 0; i < g->len; i++) {
        Segment s = {0};
        if (turtle_apply(&t, g->cur[i], p, 1.0f, delta, &s)) {
            bbox_extend(bbox, s.x0, s.y0);
            bbox_extend(bbox, s.x1, s.y1);
            segs++;
        }
    }
    *n_segments = segs;
}

/*
 * scale_to_fit() — the largest uniform scale that fits a bw x bh box into a
 * uw x uh area (the smaller of the two axis ratios), floored at MIN_SCALE so the
 * deepest generations stay visible rather than collapsing to a dot.
 */
static float scale_to_fit(float bw, float bh, int uw, int uh)
{
    float sx = (float)uw / bw, sy = (float)uh / bh;
    float s  = (sx < sy) ? sx : sy;
    return s < MIN_SCALE ? MIN_SCALE : s;
}

/*
 * axis_center() — the offset that centres a scaled box within `outer` cells along
 * one axis.  `extent` is the box size and `origin` its minimum, both in unit
 * space; the result already cancels the box origin so a cell at `origin` lands
 * correctly.
 */
static float axis_center(int outer, float extent, float origin, float scale)
{
    return ((float)outer - extent * scale) / 2.0f - origin * scale;
}

/*
 * fit_compute() — dry-run the turtle to size the fractal, then choose the scale
 * and offset that drop it into the usable band.  Reads as named steps: measure,
 * box size, usable area, scale-to-fit, then centre (or bottom-pin the plant).
 * Recomputed once per generation change and per resize.
 */
static void fit_compute(Fit *fit, const Grammar *g, const Preset *p,
                        int cols, int rows)
{
    BBox b; int segs = 0;
    turtle_measure(g, p, &b, &segs);
    fit->n_segments = segs;

    float bw = b.maxx - b.minx, bh = b.maxy - b.miny;
    if (bw < 1.0f) bw = 1.0f;                        /* avoid a zero-size box */
    if (bh < 1.0f) bh = 1.0f;

    int uw = cols - FIT_MARGIN_COLS;                 /* usable width  (side margins) */
    int uh = rows - HUD_TOP_ROWS - HUD_BOT_ROWS;     /* usable band between the HUDs */
    if (uw < 1) uw = 1;
    if (uh < 1) uh = 1;

    fit->scale = scale_to_fit(bw, bh, uw, uh);
    fit->ox    = axis_center(cols, bw, b.minx, fit->scale);   /* always centred */

    if (p->bottom)
        fit->oy = (float)(rows - HUD_BOT_ROWS - 1) - b.maxy * fit->scale;  /* pin base */
    else
        fit->oy = (float)HUD_TOP_ROWS + axis_center(uh, bh, b.miny, fit->scale);
}

/*
 * turtle_draw() — walk the string at the fit's scale and render the first `limit`
 * forward segments through the fit; the progressive reveal stops once `limit`
 * segments are drawn.  Same drive-the-turtle loop as turtle_measure, but each
 * segment is painted instead of measured.
 */
static void turtle_draw(const Grammar *g, const Preset *p, const Fit *fit,
                        int limit, WINDOW *w, int cols, int rows)
{
    Turtle t; turtle_init(&t, p->start_deg);
    float  delta = radians(p->angle_deg);
    int    seg   = 0;

    for (size_t i = 0; i < g->len && seg < limit; i++) {
        Segment s = {0};
        if (turtle_apply(&t, g->cur[i], p, fit->scale, delta, &s)) {
            put_segment(w, fit->ox + s.x0, fit->oy + s.y0,
                           fit->ox + s.x1, fit->oy + s.y1,
                        depth_attr(s.depth), s.angle, cols, rows);
            seg++;
        }
    }
}

/*
 * render_fractal() — draw the revealed portion of the current generation.  Reads
 * the grammar (what), the fit (where on screen), and the animation (how much);
 * mutates none of them.  The reveal fraction becomes a segment count, clamped to
 * at least 1 so a fresh generation never flashes blank.
 */
static void render_fractal(const Grammar *g, const Preset *p, const Fit *fit,
                           const Animation *a, WINDOW *w, int cols, int rows)
{
    int limit = (int)(a->reveal * (float)fit->n_segments + 0.5f);
    if (limit < 1)               limit = 1;
    if (limit > fit->n_segments) limit = fit->n_segments;
    turtle_draw(g, p, fit, limit, w, cols, rows);
}

/* ===================================================================== */
/* §7  scene — orchestration: the generation state machine                */
/* ===================================================================== */

/*
 * Scene — the whole picture in one object, composed from the layers above:
 *   grammar  the CORE string,            anim  the EFFECTS/DELAYS timer,
 *   fit      the RENDER mapping,          plus the user's selections.
 *
 * The state machine is: draw the generation in (anim), hold it (anim), then
 * advance the grammar and re-fit — or stop at the preset's last generation.
 */
typedef struct {
    Grammar   grammar;     /* §4 core: the string                           */
    Animation anim;        /* §5 effects: reveal + hold timing              */
    Fit       fit;         /* §6 render: screen mapping + segment count     */

    int       preset;      /* active preset, 0..N_PRESETS-1 (n / p keys)    */
    int       theme;       /* active colour theme, 0..N_THEMES-1; persists  *
                            * across preset changes and restarts (t / T)    */
    bool      paused;      /* space: freezes the draw-in AND the hold       */
    bool      at_max;      /* reached the preset's last generation — hold    */
} Scene;

static void scene_load_preset(Scene *s, int idx, int cols, int rows)
{
    s->preset = ((idx % N_PRESETS) + N_PRESETS) % N_PRESETS;
    const Preset *p = &PRESETS[s->preset];

    s->paused = false;
    s->at_max = false;
    grammar_load(&s->grammar, p);
    fit_compute(&s->fit, &s->grammar, p, cols, rows);
    s->anim.draw_sec = p->advance_sec;
    animation_begin(&s->anim);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    grammar_alloc(&s->grammar);
    s->theme = 0;                       /* color_init() already applied theme 0 */
    scene_load_preset(s, 0, cols, rows);
}

static void scene_free(Scene *s) { grammar_free(&s->grammar); }

/*
 * scene_advance() — move to the next generation: expand the grammar, re-fit, and
 * restart the draw-in.  If there is no next generation (max reached or the string
 * would overflow), latch at_max so the final form is held.
 */
static void scene_advance(Scene *s, int cols, int rows)
{
    const Preset *p = &PRESETS[s->preset];
    if (s->grammar.gen < p->max_gen && grammar_expand(&s->grammar, p)) {
        fit_compute(&s->fit, &s->grammar, p, cols, rows);
        animation_begin(&s->anim);
    } else {
        s->at_max = true;
        animation_show_full(&s->anim);
    }
}

/* scene_tick() — one fixed sim step: drive the animation; advance when its hold
 * elapses.  Paused or finished scenes do nothing (the image just holds). */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    if (s->paused || s->at_max) return;
    if (animation_step(&s->anim, dt))
        scene_advance(s, cols, rows);
}

/* scene_step() — manual single-step (Enter): jump to the next generation, shown
 * fully drawn.  Most useful while paused. */
static void scene_step(Scene *s, int cols, int rows)
{
    const Preset *p = &PRESETS[s->preset];
    if (s->at_max || s->grammar.gen >= p->max_gen) return;
    if (grammar_expand(&s->grammar, p))
        fit_compute(&s->fit, &s->grammar, p, cols, rows);
    animation_show_full(&s->anim);
}

static void scene_cycle_preset(Scene *s, int dir, int cols, int rows)
{
    scene_load_preset(s, s->preset + dir, cols, rows);
}

static void scene_restart(Scene *s, int cols, int rows)
{
    scene_load_preset(s, s->preset, cols, rows);
}

/* Theme lives on the Scene (not the per-fractal grammar), so it persists across
 * preset changes and restarts. */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

static void scene_faster(Scene *s)
{
    s->anim.draw_sec *= DRAW_FASTER;
    if (s->anim.draw_sec < DRAW_SEC_MIN) s->anim.draw_sec = DRAW_SEC_MIN;
}

static void scene_slower(Scene *s)
{
    s->anim.draw_sec *= DRAW_SLOWER;
    if (s->anim.draw_sec > DRAW_SEC_MAX) s->anim.draw_sec = DRAW_SEC_MAX;
}

static void scene_refit(Scene *s, int cols, int rows)
{
    fit_compute(&s->fit, &s->grammar, &PRESETS[s->preset], cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    render_fractal(&s->grammar, &PRESETS[s->preset], &s->fit, &s->anim,
                   w, cols, rows);
}

/* ===================================================================== */
/* §8  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the terminal as a drawing surface: just its current size, cached from
 * ncurses (getmaxyx) at startup and after each resize so the hot draw path never
 * re-queries it.  The ncurses lifecycle this wraps — initscr, colour, resize,
 * teardown — follows Gookin (2007).
 */
typedef struct {
    int cols;   /* terminal width  in character columns; valid x are [0,cols) */
    int rows;   /* terminal height in character rows;    valid y are [0,rows) */
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

static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

/* hud_line() — one HUD line, clamped to terminal width so it never wraps. */
static void hud_line(int row, int x, int pair, attr_t attr, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/* hud_status() — row 0, right-aligned + bold: title, fps, speed, run/phase word.
 * Speed is the draw-in RATE (anim_speed) so a bigger number means faster. */
static void hud_status(const Screen *s, const Scene *sc, double fps)
{
    const char *state = sc->paused ? "PAUSED "
                      : sc->at_max ? "max gen"
                      :              "running";
    char buf[128];
    snprintf(buf, sizeof buf, " L-System  %5.1f fps  speed:%.2f  %s ",
             fps, anim_speed(&sc->anim), state);
    hud_line(0, s->cols - (int)strlen(buf), PAIR_HUD, A_BOLD, s->cols, buf);
}

/* hud_params() — row 1, left: the live parameter readouts. */
static void hud_params(const Screen *s, const Scene *sc)
{
    const Preset *p = &PRESETS[sc->preset];
    char buf[128];
    snprintf(buf, sizeof buf, " %s  gen %d/%d  len:%zu  theme:%s ",
             p->name, sc->grammar.gen, p->max_gen, sc->grammar.len,
             THEMES[sc->theme].name);
    hud_line(1, 0, PAIR_HUD, A_NORMAL, s->cols, buf);
}

/* hud_keys() — bottom row: every interactive key. */
static void hud_keys(const Screen *s)
{
    hud_line(s->rows - 1, 0, PAIR_HINT, A_BOLD, s->cols,
             " q:quit  spc:pause  Enter:step  n/p:preset  t:theme  r:restart  +/-:speed ");
}

/*
 * HUD layout — data on top, actions on the bottom:
 *   row 0      (yellow bold,  right)  title, fps, speed, run/phase state
 *   row 1      (yellow plain, left)   preset, generation, length, theme
 *   row rows-1 (cyan bold,    left)   every interactive key
 */
static void screen_draw(Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);
    hud_status(s, sc, fps);
    hud_params(s, sc);
    hud_keys(s);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app — main loop                                                    */
/* ===================================================================== */

/*
 * App — the top-level program state: the picture (scene), the surface it draws to
 * (screen), and the two signal flags.  Everything user-facing is reached through
 * this one object.
 */
typedef struct {
    Scene                 scene;         /* the whole simulation + render state    */
    Screen                screen;        /* cached terminal size                   */
    volatile sig_atomic_t running;       /* main-loop flag; cleared by SIGINT/TERM */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH, serviced next frame   */
} App;

/*
 * The one global.  Signal handlers receive only an int, so the flags they must
 * touch (running / need_resize) have to be reachable without a parameter.  Those
 * two are volatile sig_atomic_t — the only kind of object a handler may portably
 * write and the loop may portably read.  Everything else is passed by pointer.
 */
static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_refit(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    int cols = app->screen.cols, rows = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':                sc->paused = !sc->paused;           break;
    case '\n': case '\r': case KEY_ENTER: scene_step(sc, cols, rows); break;

    case 'n': scene_cycle_preset(sc, +1, cols, rows); break;
    case 'p': scene_cycle_preset(sc, -1, cols, rows); break;

    case 't': scene_cycle_theme(sc, +1); break;
    case 'T': scene_cycle_theme(sc, -1); break;

    case 'r': case 'R': scene_restart(sc, cols, rows); break;

    case '=': case '+': scene_faster(sc); break;
    case '-':           scene_slower(sc); break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    FrameClock clock;
    frameclock_init(&clock);

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frameclock_init(&clock);     /* fresh timing after the resize */
        }

        frameclock_begin_frame(&clock);

        /* fixed-timestep: advance the sim in equal slices, however fast we draw */
        int64_t tick_ns = TICK_NS(SIM_FPS);
        while (frameclock_step_due(&clock, tick_ns))
            scene_tick(&app->scene, TICK_SEC, app->screen.cols, app->screen.rows);

        frameclock_throttle(&clock);

        screen_draw(&app->screen, &app->scene, clock.fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
