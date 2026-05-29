/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lyapunov.c — Markus-Lyapunov fractal
 *
 * For each terminal cell the logistic map x' = r·x·(1−x) is iterated with the
 * parameter r alternating between two values a and b according to a symbol
 * sequence (e.g. "AB", "AAB", …).  The Lyapunov exponent
 *     λ = (1/N) Σ ln|r·(1−2x)|
 * measures how fast nearby orbits diverge, and classifies the cell:
 *     λ < 0  → stable  (cool colours)
 *     λ > 0  → chaotic (warm colours)
 *     λ outside [LEV_MIN, LEV_MAX] → blank
 *
 * FOUR LAYERS, deliberately separated so the maths is trustworthy and the one
 * thing that happens "over time" is obvious:
 *
 *   §4 CORE   — the pure maths: logistic map → Lyapunov exponent → bucket.  Given
 *               (a, b, sequence) it returns a level.  It touches no grid, no
 *               clock, no screen, so the fractal definition is safe from any
 *               cosmetic change.
 *   §5 FIELD  — the sampled grid (a level per cell) plus the cell → (a, b) domain
 *               mapping.  Pure data.
 *   §6 SCAN   — the ONLY effect/delay: the progressive build-up.  It samples a few
 *               field rows per frame so the image grows top-to-bottom.  Remove it
 *               and the picture would simply appear all at once; nothing else is
 *               applied over time apart from drawing.
 *   §8 RENDER — reads the field and paints coloured glyphs.  Mutates nothing.
 *   §7 SCENE  — orchestration: field + scan + sequence + theme + pause.
 *
 * Keys:
 *   q / ESC        quit
 *   space          pause / resume
 *   n              next sequence
 *   p              prev sequence
 *   r              reset / restart current sequence
 *   t / T          next / prev theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lyapunov.c -o lyapunov -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config   — constants, sequences, theme table
 *   §2  clock    — monotonic ns clock + sleep
 *   §3  color    — themeable palette (t/T) + HUD pairs
 *   §4  core     — logistic map + Lyapunov exponent + classification (PURE MATH)
 *   §5  field    — sampled level grid + parameter-domain mapping (DATA)
 *   §6  scan     — progressive build-up (THE EFFECT / DELAY)
 *   §7  scene    — orchestration: field + scan + sequence + theme
 *   §8  render   — field → coloured glyphs (READ-ONLY)
 *   §9  screen   — ncurses lifecycle + HUD
 *   §10 app      — main loop
 */

/* ── REFERENCES — for the concepts and the rendering ──────────────────── *
 *
 *   The logistic map & chaos  (§4 core)
 *   ── May, R. M. (1976). "Simple Mathematical Models with Very Complicated
 *      Dynamics." Nature 261, 459-467.  The logistic map x' = r·x·(1−x) and its
 *      route to chaos — the engine iterated in logistic_step().
 *   ── Feigenbaum, M. J. (1978). "Quantitative Universality for a Class of
 *      Nonlinear Transformations." J. Stat. Phys. 19(1), 25-52.  Period-doubling
 *      universality — the bifurcation structure these images expose.
 *   ── Strogatz, S. H. (1994). "Nonlinear Dynamics and Chaos." Addison-Wesley.
 *      The standard textbook for the logistic map, attractors, and Lyapunov
 *      exponents — the gentlest way into everything in §4.
 *
 *   Lyapunov exponents & this fractal  (§4 core, the whole program)
 *   ── Wolf, A., Swift, J. B., Swinney, H. L. & Vastano, J. A. (1985).
 *      "Determining Lyapunov Exponents from a Time Series." Physica D 16(3),
 *      285-317.  Computing λ by averaging ln|df/dx| — exactly lyapunov_exponent().
 *   ── Markus, M. & Hess, B. (1989). "Lyapunov Exponents of the Logistic Map with
 *      Periodic Forcing." Computers & Graphics 13(4), 553-558.  Introduced the
 *      Markus-Lyapunov fractal: λ over the (a,b) plane with the map's parameter
 *      driven by a symbol sequence — precisely what this program draws.
 *   ── Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *      Self-similarity and fractal dimension — why the stable/chaotic boundary
 *      is a fractal.
 *
 *   Rendering  (§9 screen)
 *   ── Gookin, D. (2007). "Programmer's Guide to NCURSES." Wiley.  The cell-
 *      drawing API behind §9: init_pair, mvaddch, refresh, resize.
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GRID_ROWS_MAX  80
#define GRID_COLS_MAX  300

#define HUD_TOP_ROWS    2    /* rows 0..1 — data HUD (status + parameters) */
#define HUD_BOT_ROWS    1    /* last row  — action / key-hint bar          */

/* Parameter domain sampled across the screen: a along columns, b down rows. */
#define A_MIN  2.5
#define A_MAX  4.0
#define B_MIN  2.5
#define B_MAX  4.0

#define WARMUP_ITERS    100   /* iterations discarded before measuring λ   */
#define LYAP_ITERS      200   /* iterations averaged into λ                */
#define ROWS_PER_FRAME    2   /* field rows sampled per frame (the build-up) */
#define INITIAL_X       0.5   /* logistic-map seed x0 (the midpoint of (0,1)) */
#define DERIV_FLOOR     1e-15 /* floor on |f'(x)| so its log stays finite   */

/* λ outside this range is treated as blank (clipped / divergent). */
#define LEV_MIN  (-4.0)
#define LEV_MAX    2.5
#define DIVERGED_LAMBDA  10.0   /* λ returned for orbits that leave (0,1) */

/* Encoded cell levels (int8): negative = stable, positive = chaotic. */
#define LEVEL_UNSAMPLED  (-128)  /* not computed yet */
#define LEVEL_BLANK         0    /* computed but clipped — draw nothing */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define RENDER_FPS     30               /* render-rate cap */
#define FRAME_NS    (NS_PER_SEC / RENDER_FPS)
#define FPS_UPDATE_MS  500              /* refresh the fps readout this often */

#define N_SEQUENCES  6

static const char *k_sequences[N_SEQUENCES] = {
    "AB", "AAB", "ABB", "AABB", "AABAB", "BBBAAAB",
};

#define N_THEMES  10

/*
 * Theme — the colours of the fractal's two regions, so the stable (λ<0) and
 * chaotic (λ>0) halves read as cool vs warm.  A theme is pure data: one table row
 * per palette, cycled with t / T; the engine never changes.
 *
 * Every 256-colour stop is kept in the bright half of the cube so even the
 * faintest '.' tier stays visible on black (dim cube indices vanish there).
 */
typedef struct {
    const char *name;       /* shown in the HUD                                 */
    int fg256[8];           /* [0..3] stable S1..S4 (deep→shallow), [4..7] chaos */
    int s_fg8[4];           /* 8-colour stable fallback (when COLORS < 256)      */
    int c_fg8[4];           /* 8-colour chaotic fallback                         */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* name        stable S1..S4 / chaos C1..C4         8-colour stable                 8-colour chaos */
    { "Classic", { 51, 45, 39, 33,   226, 214, 202, 196 },
        { COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED } },
    { "Neon",    { 46, 40, 34, 28,   201, 165, 129, 93 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "Ocean",   { 51, 50, 44, 38,   223, 216, 209, 202 },
        { COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE },
        { COLOR_YELLOW, COLOR_RED, COLOR_RED, COLOR_RED } },
    { "Azure",   { 75, 69, 63, 39,   214, 208, 172, 166 },
        { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED } },
    { "Violet",  { 141, 135, 99, 93, 220, 214, 208, 202 },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLUE, COLOR_BLUE },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED } },
    { "Mint",    { 86, 79, 48, 42,   213, 207, 201, 200 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN },
        { COLOR_MAGENTA, COLOR_RED, COLOR_RED, COLOR_RED } },
    { "Sky",     { 117, 111, 75, 69, 218, 212, 206, 200 },
        { COLOR_CYAN, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "Spring",  { 48, 42, 36, 30,   222, 216, 178, 172 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_CYAN },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED } },
    { "Forest",  { 82, 76, 70, 64,   209, 203, 197, 196 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
        { COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED } },
    { "Rose",    { 45, 39, 38, 37,   211, 205, 199, 198 },
        { COLOR_CYAN, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_RED, COLOR_RED } },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/* clock_ns() — a monotonic timestamp in ns; never jumps backward, so it is safe
 * for measuring frame durations. */
static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

/* clock_sleep_ns() — sleep for ns; a non-positive request is a no-op (the frame
 * already overran its budget, so don't sleep). */
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/*
 * FpsMeter — a smoothed frames-per-second readout for the HUD: bank each frame's
 * duration, then publish frames / elapsed-seconds once per FPS_UPDATE_MS.
 */
typedef struct {
    long long accum_ns;   /* frame time banked since the last publish */
    int       frames;     /* frames counted since the last publish    */
    double    value;      /* most recent smoothed fps                 */
} FpsMeter;

static void fps_tick(FpsMeter *m, long long frame_ns)
{
    m->accum_ns += frame_ns;
    m->frames++;
    if (m->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        m->value    = (double)m->frames / ((double)m->accum_ns / (double)NS_PER_SEC);
        m->accum_ns = 0;
        m->frames   = 0;
    }
}

/* ===================================================================== */
/* §3  color — themeable palette + HUD pairs                              */
/* ===================================================================== */

/*
 * Colour-pair slots.  CP_S1..S4 / CP_C1..C4 are the stable / chaotic ramps,
 * rebound by theme_apply().  CP_HUD (data) and CP_HINT (actions) are theme-
 * independent so the HUD stays legible over any palette.
 */
enum {
    CP_HUD = 1,
    CP_S1, CP_S2, CP_S3, CP_S4,   /* stable buckets  */
    CP_C1, CP_C2, CP_C3, CP_C4,   /* chaotic buckets */
    CP_HINT,
};

/* theme_apply() — bind the stable/chaotic ramps to a theme (HUD pairs are fixed). */
static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    bool truecolor = COLORS >= 256;

    if (truecolor) {
        init_pair(CP_HUD,  226, COLOR_BLACK);
        init_pair(CP_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_HINT, COLOR_CYAN,   COLOR_BLACK);
    }

    for (int i = 0; i < 4; i++) {
        init_pair((short)(CP_S1 + i),
                  (short)(truecolor ? th->fg256[i]     : th->s_fg8[i]), COLOR_BLACK);
        init_pair((short)(CP_C1 + i),
                  (short)(truecolor ? th->fg256[4 + i] : th->c_fg8[i]), COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    theme_apply(0);   /* Classic; the Scene tracks the active theme thereafter */
}

/* ===================================================================== */
/* §4  core — logistic map + Lyapunov exponent (PURE MATH)                */
/* ===================================================================== */

/*
 * Param — a point in the control-parameter plane: the two r-values the logistic
 * map alternates between (a on an 'A' symbol, b on a 'B').  The Markus-Lyapunov
 * fractal IS the Lyapunov exponent as a function over this (a,b) plane, so every
 * pixel of the image is exactly one Param.
 */
typedef struct { double a, b; } Param;

/* logistic_step() — one iteration of the logistic map x' = r·x·(1−x). */
static double logistic_step(double r, double x) { return r * x * (1.0 - x); }

/* logistic_r() — the parameter at step i: a on an 'A' symbol, b on a 'B'.  The
 * sequence string drives the alternation that shapes the fractal. */
static double logistic_r(Param p, const char *seq, int i, int slen)
{
    return (seq[i % slen] == 'A') ? p.a : p.b;
}

/* orbit_escaped() — the orbit left the unit interval (0,1), i.e. it diverged. */
static bool orbit_escaped(double x) { return x <= 0.0 || x >= 1.0; }

/*
 * logistic_deriv_mag() — |f'(x)| for f(x) = r·x·(1−x), i.e. |r·(1−2x)|, floored
 * away from zero so its log stays finite.  This local stretching factor is the
 * heart of the measurement: averaging ln of it IS the Lyapunov exponent
 * (Wolf et al. 1985).
 */
static double logistic_deriv_mag(double r, double x)
{
    double d = fabs(r * (1.0 - 2.0 * x));
    return d < DERIV_FLOOR ? DERIV_FLOOR : d;
}

/* orbit_warmup() — iterate WARMUP_ITERS steps to settle onto the attractor,
 * discarding the transient.  False if the orbit diverged. */
static bool orbit_warmup(Param p, const char *seq, int slen, double *x)
{
    for (int i = 0; i < WARMUP_ITERS; i++) {
        *x = logistic_step(logistic_r(p, seq, i, slen), *x);
        if (orbit_escaped(*x)) return false;
    }
    return true;
}

/* orbit_lyapunov() — average ln|f'(x)| over LYAP_ITERS settled steps; that mean
 * IS λ.  Returns DIVERGED_LAMBDA if the orbit escapes mid-measurement. */
static double orbit_lyapunov(Param p, const char *seq, int slen, double *x)
{
    double sum = 0.0;
    for (int i = 0; i < LYAP_ITERS; i++) {
        double r = logistic_r(p, seq, i, slen);
        *x = logistic_step(r, *x);
        if (orbit_escaped(*x)) return DIVERGED_LAMBDA;
        sum += log(logistic_deriv_mag(r, *x));
    }
    return sum / LYAP_ITERS;
}

/*
 * lyapunov_exponent() — λ at parameter point p under symbol sequence `seq`:
 * seed the orbit, warm it onto the attractor, then measure.  Pure: same inputs,
 * same λ.
 */
static double lyapunov_exponent(Param p, const char *seq)
{
    int    slen = (int)strlen(seq);
    double x    = INITIAL_X;

    if (!orbit_warmup(p, seq, slen, &x)) return DIVERGED_LAMBDA;
    return orbit_lyapunov(p, seq, slen, &x);
}

/* |λ| band edges splitting each region into four visual tiers (barely → very):
 * a value below band[i] lands in tier i, above all three → tier 3. */
static const double STABLE_BANDS[3] = { 0.5, 1.5, 3.0 };   /* keyed on |λ| */
static const double CHAOS_BANDS [3] = { 0.4, 1.0, 2.0 };   /* keyed on  λ  */

/* bucket_of() — tier 0..3 of the first band `v` falls under (3 if above all). */
static int bucket_of(double v, const double bands[3])
{
    for (int i = 0; i < 3; i++)
        if (v < bands[i]) return i;
    return 3;
}

/*
 * level_encode() — classify λ into a signed bucket the renderer understands:
 *   stable  (λ<0): -1 barely .. -4 very stable
 *   chaotic (λ>0):  1 barely ..  4 very chaotic
 *   clipped / divergent (outside [LEV_MIN, LEV_MAX]): LEVEL_BLANK
 */
static int8_t level_encode(double lam)
{
    if (lam >= LEV_MAX || lam <= LEV_MIN) return LEVEL_BLANK;
    if (lam < 0.0) return (int8_t)(-(bucket_of(-lam, STABLE_BANDS) + 1));   /* stable */
    return (int8_t)(bucket_of(lam, CHAOS_BANDS) + 1);                       /* chaotic */
}

/* ===================================================================== */
/* §5  field — the sampled grid + parameter-domain mapping (DATA)         */
/* ===================================================================== */

/*
 * Field — the sampled Markus-Lyapunov fractal (Markus & Hess 1989): one encoded
 * λ bucket per cell over an active rows x cols area.  It also fixes the parameter
 * domain it samples — a runs A_MIN..A_MAX across the columns, b runs B_MAX..B_MIN
 * down the rows (top = high b) — via field_a/field_b/field_param below.  Pure
 * data: no timing, no colour, no screen.
 *
 * The grid is a fixed worst-case array (no per-frame malloc); only the active
 * rows/cols are sampled and drawn.
 */
typedef struct {
    int8_t level[GRID_ROWS_MAX][GRID_COLS_MAX];  /* per cell: level_encode() value,  *
                                                  * or LEVEL_UNSAMPLED until computed */
    int    rows;     /* active height = the on-screen fractal band */
    int    cols;     /* active width  = terminal columns           */
} Field;

/* field_clear() — mark every cell UNSAMPLED so the renderer skips it until the
 * scan fills it; this is what lets the image build up rather than flash in. */
static void field_clear(Field *f)
{
    memset(f->level, (unsigned char)LEVEL_UNSAMPLED, sizeof f->level);
}

/* field_init() — size the active area to a new screen, clamped to the fixed
 * buffer so sampling can never run past it, then clear it. */
static void field_init(Field *f, int rows, int cols)
{
    f->rows = rows < 1 ? 1 : (rows > GRID_ROWS_MAX ? GRID_ROWS_MAX : rows);
    f->cols = cols < 1 ? 1 : (cols > GRID_COLS_MAX ? GRID_COLS_MAX : cols);
    field_clear(f);
}

/* field_a / field_b — the cell → parameter mapping (the sampled domain). */
static double field_a(const Field *f, int col)
{
    double span = (f->cols > 1) ? (double)(f->cols - 1) : 1.0;
    return A_MIN + (double)col / span * (A_MAX - A_MIN);
}

static double field_b(const Field *f, int row)
{
    double span = (f->rows > 1) ? (double)(f->rows - 1) : 1.0;
    return B_MAX - (double)row / span * (B_MAX - B_MIN);
}

/* field_param() — the parameter point sampled at cell (row, col): a along the
 * columns, b down the rows. */
static Param field_param(const Field *f, int row, int col)
{
    return (Param){ field_a(f, col), field_b(f, row) };
}

/* ===================================================================== */
/* §6  scan — progressive build-up (THE EFFECT / DELAY)                   */
/* ===================================================================== */

/*
 * Scan — the only thing applied over time apart from drawing.  Computing every
 * cell at once would hitch for a second, so instead `next_row` advances a few
 * rows per frame, sampling the field top-to-bottom — which also reads as a
 * pleasing build-up.  The scan calls the pure core (§4); it only chooses WHICH
 * cells get computed WHEN.  Nothing here decides colour or position.
 */
typedef struct {
    int next_row;   /* next field row to sample (the build-up cursor) */
} Scan;

/* scan_reset() — rewind the build-up to the top to start a fresh sweep. */
static void scan_reset(Scan *s) { s->next_row = 0; }

/* scan_complete() — has the sweep reached the bottom (the whole image drawn)? */
static bool scan_complete(const Scan *s, const Field *f) { return s->next_row >= f->rows; }

/* scan_progress_pct() — how far the build-up has got, 0..100, for the HUD. */
static int scan_progress_pct(const Scan *s, const Field *f)
{
    if (f->rows < 1) return 100;
    int pct = s->next_row * 100 / f->rows;
    return pct > 100 ? 100 : pct;
}

/* scan_step() — sample the next ROWS_PER_FRAME rows into the field. */
static void scan_step(Scan *s, Field *f, const char *seq)
{
    for (int k = 0; k < ROWS_PER_FRAME && s->next_row < f->rows; k++) {
        int row = s->next_row++;
        for (int col = 0; col < f->cols; col++) {
            Param p = field_param(f, row, col);
            f->level[row][col] = level_encode(lyapunov_exponent(p, seq));
        }
    }
}

/* ===================================================================== */
/* §7  scene — orchestration: field + scan + sequence + theme            */
/* ===================================================================== */

/*
 * Scene — the whole picture in one object, composed from the layers above so a
 * reader can see at a glance what is data, what is the effect, and what the user
 * controls.  scene_tick() drives the build-up; the rest is user state that
 * persists across restarts (theme outlives a sequence change; the field/scan are
 * rebuilt each time).
 */
typedef struct {
    Field field;       /* §5 data:   the sampled levels                 */
    Scan  scan;        /* §6 effect: the progressive build-up           */
    int   seq_idx;     /* which driving sequence, 0..N_SEQUENCES-1 (n/p) */
    int   theme;       /* active colour theme, 0..N_THEMES-1 (t/T)       */
    bool  paused;      /* space: freezes the build-up (the field holds)  */
} Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);   /* seq_idx = theme = 0, paused = false */
}

/* scene_resize() — fit the field to a new screen and restart the scan. */
static void scene_resize(Scene *s, int term_rows, int term_cols)
{
    field_init(&s->field, term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS, term_cols);
    scan_reset(&s->scan);
}

/* scene_restart() — recompute the current sequence from scratch (keep the size). */
static void scene_restart(Scene *s)
{
    field_clear(&s->field);
    scan_reset(&s->scan);
}

/* scene_set_sequence() — step to the next/prev driving sequence; a different
 * sequence is a different fractal, so recompute from scratch. */
static void scene_set_sequence(Scene *s, int dir)
{
    s->seq_idx = (s->seq_idx + dir + N_SEQUENCES) % N_SEQUENCES;
    scene_restart(s);
}

/* Theme persists across sequence changes (it lives on the Scene, not the field). */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* scene_sequence() — the active driving sequence string. */
static const char *scene_sequence(const Scene *s) { return k_sequences[s->seq_idx]; }

/* scene_tick() — advance the build-up effect one frame (paused/complete: hold). */
static void scene_tick(Scene *s)
{
    if (!s->paused && !scan_complete(&s->scan, &s->field))
        scan_step(&s->scan, &s->field, scene_sequence(s));
}

/* ===================================================================== */
/* §8  render — field → coloured glyphs (READ-ONLY)                       */
/* ===================================================================== */

/*
 * level_appearance() — map an encoded level to its colour pair + glyph.
 *   stable  -1..-4 (barely→very): '.' ':' '+' '#', brightening toward S1
 *   chaotic  1.. 4 (barely→very): '@' '#' '+' ':'
 */
static void level_appearance(int8_t lv, int *pair, char *glyph)
{
    static const int  stable_cp[4] = { CP_S4, CP_S3, CP_S2, CP_S1 };
    static const char stable_ch[4] = { '.',   ':',   '+',   '#'  };
    static const int  chaos_cp [4] = { CP_C1, CP_C2, CP_C3, CP_C4 };
    static const char chaos_ch [4] = { '@',   '#',   '+',   ':'  };

    if (lv < 0) {
        int i = (-lv) - 1; if (i > 3) i = 3;
        *pair = stable_cp[i]; *glyph = stable_ch[i];
    } else {
        int i = lv - 1;     if (i > 3) i = 3;
        *pair = chaos_cp[i]; *glyph = chaos_ch[i];
    }
}

/* render_field() — paint each sampled cell below the top HUD rows.  Read-only. */
static void render_field(const Field *f)
{
    for (int row = 0; row < f->rows && row < GRID_ROWS_MAX; row++) {
        for (int col = 0; col < f->cols && col < GRID_COLS_MAX; col++) {
            int8_t lv = f->level[row][col];
            if (lv == LEVEL_UNSAMPLED || lv == LEVEL_BLANK) continue;

            int  pair; char ch;
            level_appearance(lv, &pair, &ch);
            attron(COLOR_PAIR(pair));
            mvaddch(HUD_TOP_ROWS + row, col, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(pair));
        }
    }
}

/* ===================================================================== */
/* §9  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the terminal as a drawing surface: just its current size, cached from
 * ncurses (getmaxyx) at startup and after each resize, and clamped to the grid
 * maxima so it always indexes the field safely.  The ncurses lifecycle this wraps
 * — initscr, colour, resize, teardown — follows Gookin (2007).
 */
typedef struct {
    int rows;   /* terminal height in cells, clamped to GRID_ROWS_MAX */
    int cols;   /* terminal width  in cells, clamped to GRID_COLS_MAX */
} Screen;

/* screen_measure() — read the terminal size, clamped to the fixed grid maxima so
 * it always stays a valid index into the field's arrays. */
static void screen_measure(Screen *s)
{
    getmaxyx(stdscr, s->rows, s->cols);
    if (s->rows > GRID_ROWS_MAX) s->rows = GRID_ROWS_MAX;
    if (s->cols > GRID_COLS_MAX) s->cols = GRID_COLS_MAX;
}

/* screen_init() — enter ncurses: raw keys, no echo, hidden cursor, non-blocking
 * input, and no tearing while we write the frame. */
static void screen_init(Screen *s)
{
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    screen_measure(s);
}

/* screen_resize() — re-sync ncurses and the cached size to the new terminal
 * after a SIGWINCH. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    screen_measure(s);
}

/* hud_line() — one HUD line at (row, x) in `pair`, clamped to the terminal width
 * so a long line is truncated rather than wrapping over the fractal. */
static void hud_line(int row, int x, int pair, attr_t attr, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/* hud_status() — row 0, right-aligned + bold: title, fps, render state. */
static void hud_status(const Screen *sc, const Scene *s, double fps)
{
    const char *state = s->paused                         ? "PAUSED "
                      : scan_complete(&s->scan, &s->field) ? "done"
                      :                                      "rendering";
    char buf[GRID_COLS_MAX + 1];
    snprintf(buf, sizeof buf, " Lyapunov  %5.1f fps  %s ", fps, state);
    hud_line(0, sc->cols - (int)strlen(buf), CP_HUD, A_BOLD, sc->cols, buf);
}

/* hud_params() — row 1, left: sequence, theme, build-up progress. */
static void hud_params(const Screen *sc, const Scene *s)
{
    char buf[GRID_COLS_MAX + 1];
    snprintf(buf, sizeof buf, " seq:%s  theme:%s  %d%% ",
             scene_sequence(s), k_themes[s->theme].name,
             scan_progress_pct(&s->scan, &s->field));
    hud_line(1, 0, CP_HUD, A_NORMAL, sc->cols, buf);
}

/* hud_keys() — bottom row: every interactive key. */
static void hud_keys(const Screen *sc)
{
    hud_line(sc->rows - 1, 0, CP_HINT, A_BOLD, sc->cols,
             " q:quit  spc:pause  n/p:seq  t/T:theme  r:reset ");
}

/*
 * HUD — data on top, actions on the bottom:
 *   row 0      (yellow bold,  right)  title, fps, render state
 *   row 1      (yellow plain, left)   sequence, theme, progress %
 *   row rows-1 (cyan bold,    left)   every interactive key
 */
static void draw_hud(const Screen *sc, const Scene *s, double fps)
{
    hud_status(sc, s, fps);
    hud_params(sc, s);
    hud_keys(sc);
}

/* ===================================================================== */
/* §10  app — main loop                                                   */
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
 * The one global: signal handlers receive only an int, so the flags they touch
 * must be reachable without a parameter (volatile sig_atomic_t — the only kind a
 * handler may portably write and the loop read).  Pause is a key, not a signal,
 * so it lives on the Scene.
 */
static App g_app;

/* on_signal() — record the event in a flag and return; the loop services it next
 * frame.  Handlers must do almost nothing (only async-signal-safe flag writes). */
static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

/* cleanup() — restore the terminal on exit (registered with atexit). */
static void cleanup(void) { endwin(); }

/* app_handle_key() — dispatch one keypress to the scene; false means quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':            s->paused = !s->paused;     break;
    case 'n':            scene_set_sequence(s, +1);  break;
    case 'p':            scene_set_sequence(s, -1);  break;
    case 'r': case 'R':  scene_restart(s);           break;
    case 't':            scene_cycle_theme(s, +1);   break;
    case 'T':            scene_cycle_theme(s, -1);   break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    App *app     = &g_app;
    app->running = 1;

    scene_init(&app->scene);
    screen_init(&app->screen);
    scene_resize(&app->scene, app->screen.rows, app->screen.cols);

    FpsMeter fps = {0};

    while (app->running) {

        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
        }

        long long frame_start = clock_ns();

        int ch;
        while ((ch = getch()) != ERR)
            if (!app_handle_key(app, ch)) { app->running = 0; break; }

        scene_tick(&app->scene);              /* §6 effect: advance the build-up */

        erase();
        render_field(&app->scene.field);
        draw_hud(&app->screen, &app->scene, fps.value);
        wnoutrefresh(stdscr);
        doupdate();

        long long frame_dur = clock_ns() - frame_start;
        fps_tick(&fps, frame_dur);
        clock_sleep_ns(FRAME_NS - frame_dur);
    }

    return 0;
}
