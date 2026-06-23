/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lyapunov.c — the Markus-Lyapunov fractal, drawn in the terminal.
 *
 * The idea: take a simple population-growth rule and feed it two different
 * growth rates in turn, following a pattern like "AB" or "AABB".  For each
 * point we ask: do nearby starting values settle down together (stable) or
 * fly apart (chaotic)?  Stable points get cool colours, chaotic ones warm,
 * and the boundary between them turns out to be a beautiful fractal.
 *
 * Reference: Markus & Hess (1989), "Lyapunov Exponents of the Logistic Map
 * with Periodic Forcing," Computers & Graphics 13(4), 553-558 — the paper
 * that introduced this picture.  The growth rule itself is the logistic map
 * from May (1976), Nature 261, 459-467.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GRID_ROWS_MAX  80
#define GRID_COLS_MAX  300

#define HUD_TOP_ROWS    2    /* rows 0..1 — data HUD (status + parameters) */
#define HUD_BOT_ROWS    1    /* last row  — action / key-hint bar          */

/* The two growth rates we test, spread across the screen: rate a runs left to
 * right along the columns, rate b runs top to bottom down the rows. */
#define A_MIN  2.5
#define A_MAX  4.0
#define B_MIN  2.5
#define B_MAX  4.0

#define WARMUP_ITERS    100   /* run the rule this many times first, then start measuring */
#define LYAP_ITERS      200   /* this many measured steps, averaged into the verdict */
#define ROWS_PER_FRAME    2   /* how many screen rows to compute each frame (the slow reveal) */
#define INITIAL_X       0.5   /* starting population, halfway between empty and full */
#define DERIV_FLOOR     1e-15 /* tiny floor so we never take the log of zero */

/* Scores outside this range are off-the-chart and drawn as empty space. */
#define LEV_MIN  (-4.0)
#define LEV_MAX    2.5
#define DIVERGED_LAMBDA  10.0   /* the "blew up" score, for points that run off to infinity */

/* What a cell holds (a signed byte): below zero = stable, above zero = chaotic. */
#define LEVEL_UNSAMPLED  (-128)  /* not worked out yet */
#define LEVEL_BLANK         0    /* worked out, but off the chart — leave it blank */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define RENDER_FPS     30               /* cap the frame rate here */
#define FRAME_NS    (NS_PER_SEC / RENDER_FPS)
#define FPS_UPDATE_MS  500              /* how often to refresh the fps number on screen */

#define N_SEQUENCES  6

static const char *k_sequences[N_SEQUENCES] = {
    "AB", "AAB", "ABB", "AABB", "AABAB", "BBBAAAB",
};

#define N_THEMES  10

/*
 * Theme — one colour scheme for the picture.  Stable areas get one set of
 * colours, chaotic areas another, so the two read clearly apart (typically
 * cool vs warm).  It is just data; press t / T to cycle through the schemes.
 *
 * The 256-colour values are all picked from the bright half of the palette so
 * that even the faintest dot stays visible on a black background — the dark
 * end of the palette tends to disappear there.
 */
typedef struct {
    const char *name;       /* the scheme's name, shown in the status bar */
    int fg256[8];           /* full-colour: first 4 are the stable shades, last 4 the chaotic shades */
    int s_fg8[4];           /* stable shades for old 8-colour terminals */
    int c_fg8[4];           /* chaotic shades for old 8-colour terminals */
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

/* A steadily-rising timer in nanoseconds.  It never jumps backward, which makes
 * it safe for timing how long a frame took. */
static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

/* Sleep for the given number of nanoseconds.  If asked for zero or less, do
 * nothing — that means the frame already ran long, so there's no time to spare. */
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/*
 * FpsMeter — works out the frames-per-second number shown in the corner.
 * Rather than jitter every frame, it tallies frames and time for half a second,
 * then publishes a steady average.
 */
typedef struct {
    long long accum_ns;   /* time added up since we last published a number */
    int       frames;     /* frames counted since we last published */
    double    value;      /* the smoothed fps figure we're currently showing */
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
 * Names for our colour slots.  S1..S4 are four shades for stable areas, C1..C4
 * four shades for chaotic areas; these change when you switch themes.  HUD and
 * HINT are the on-screen text colours, kept fixed so they read over any scheme.
 */
enum {
    CP_HUD = 1,
    CP_S1, CP_S2, CP_S3, CP_S4,   /* stable shades */
    CP_C1, CP_C2, CP_C3, CP_C4,   /* chaotic shades */
    CP_HINT,
};

/* Load a theme's colours into the stable and chaotic slots (text colours stay put). */
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
    theme_apply(0);   /* start on Classic; the Scene remembers the choice after that */
}

/* ===================================================================== */
/* §4  core — logistic map + Lyapunov exponent (PURE MATH)                */
/* ===================================================================== */

/*
 * Param — one point we want to colour, described by the two growth rates it
 * uses.  Rate `a` is applied on every 'A' in the pattern, rate `b` on every 'B'.
 * Every cell on screen is one of these (a, b) pairs.
 */
typedef struct { double a, b; } Param;

/* Apply the growth rule once: from population x, get next year's population.
 * Growth is fast when the population is small, but crowding pulls it back down
 * as it nears full (that's the (1 - x) part). */
static double logistic_step(double r, double x) { return r * x * (1.0 - x); }

/* Which growth rate to use at step i: rate a on an 'A' in the pattern, b on a 'B'.
 * Repeating this pattern is what shapes the whole picture. */
static double logistic_r(Param p, const char *seq, int i, int slen)
{
    return (seq[i % slen] == 'A') ? p.a : p.b;
}

/* True if the population has run off the edge (gone to zero or past full) and
 * the numbers have blown up. */
static bool orbit_escaped(double x) { return x <= 0.0 || x >= 1.0; }

/*
 * How sensitive the rule is right here — how much a tiny nudge to the population
 * gets stretched or shrunk in one step.  Averaging this (in log form) over many
 * steps is exactly the stable-vs-chaotic score we're after (Wolf et al. 1985).
 * Floored just above zero so we never try to take the log of nothing.
 */
static double logistic_deriv_mag(double r, double x)
{
    double d = fabs(r * (1.0 - 2.0 * x));
    return d < DERIV_FLOOR ? DERIV_FLOOR : d;
}

/* Run the rule a bunch of times first, before measuring anything, so the
 * population settles into its long-term behaviour and we skip the messy start.
 * Returns false if it blew up along the way. */
static bool orbit_warmup(Param p, const char *seq, int slen, double *x)
{
    for (int i = 0; i < WARMUP_ITERS; i++) {
        *x = logistic_step(logistic_r(p, seq, i, slen), *x);
        if (orbit_escaped(*x)) return false;
    }
    return true;
}

/* The measurement: keep running the rule and average up how sensitive it is at
 * each step.  A negative average means nearby populations pull together (stable);
 * positive means they fly apart (chaos).  Returns the "blew up" score if the
 * population escapes partway through. */
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
 * The whole verdict for one point: start the population, let it settle, then
 * measure whether it's stable or chaotic.  Same inputs always give the same
 * answer.
 */
static double lyapunov_exponent(Param p, const char *seq)
{
    int    slen = (int)strlen(seq);
    double x    = INITIAL_X;

    if (!orbit_warmup(p, seq, slen, &x)) return DIVERGED_LAMBDA;
    return orbit_lyapunov(p, seq, slen, &x);
}

/* Cut-offs that split each region into four strengths (barely → very) so we can
 * pick four shades of colour.  A score below the first cut-off is the weakest
 * tier; past all three, the strongest. */
static const double STABLE_BANDS[3] = { 0.5, 1.5, 3.0 };
static const double CHAOS_BANDS [3] = { 0.4, 1.0, 2.0 };

/* Which strength tier (0..3) a score falls into. */
static int bucket_of(double v, const double bands[3])
{
    for (int i = 0; i < 3; i++)
        if (v < bands[i]) return i;
    return 3;
}

/*
 * Turn a raw score into the single number a cell stores:
 *   stable  -> -1 (barely) .. -4 (very stable)
 *   chaotic ->  1 (barely) ..  4 (very chaotic)
 *   off the chart -> blank, drawn as nothing
 */
static int8_t level_encode(double lam)
{
    if (lam >= LEV_MAX || lam <= LEV_MIN) return LEVEL_BLANK;
    if (lam < 0.0) return (int8_t)(-(bucket_of(-lam, STABLE_BANDS) + 1));   /* stable: negative */
    return (int8_t)(bucket_of(lam, CHAOS_BANDS) + 1);                       /* chaotic: positive */
}

/* ===================================================================== */
/* §5  field — the sampled grid + parameter-domain mapping (DATA)         */
/* ===================================================================== */

/*
 * Field — the picture as a grid of computed scores, one per cell.  It also
 * decides which growth rates each cell stands for: rate a runs left to right
 * across the columns, rate b runs top to bottom down the rows (top row = highest
 * b).  Just data — it knows nothing about timing, colour, or the screen.
 *
 * The grid is one big fixed array sized for the largest terminal we'll allow, so
 * we never allocate memory mid-run; only the active part is used and drawn.
 */
typedef struct {
    int8_t level[GRID_ROWS_MAX][GRID_COLS_MAX];  /* each cell's score, or "not done yet" */
    int    rows;     /* how many rows are actually in use (the picture's height) */
    int    cols;     /* how many columns are in use (the picture's width) */
} Field;

/* Mark every cell as "not done yet" so the drawing step skips it until the slow
 * reveal computes it — this is what makes the image build up instead of popping in. */
static void field_clear(Field *f)
{
    memset(f->level, (unsigned char)LEVEL_UNSAMPLED, sizeof f->level);
}

/* Resize the picture to fit a new terminal, capped to the fixed array so we can
 * never write past it, then wipe it clean. */
static void field_init(Field *f, int rows, int cols)
{
    f->rows = rows < 1 ? 1 : (rows > GRID_ROWS_MAX ? GRID_ROWS_MAX : rows);
    f->cols = cols < 1 ? 1 : (cols > GRID_COLS_MAX ? GRID_COLS_MAX : cols);
    field_clear(f);
}

/* Work out which growth rate a given column / row stands for. */
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

/* The (a, b) pair that cell (row, col) stands for. */
static Param field_param(const Field *f, int row, int col)
{
    return (Param){ field_a(f, col), field_b(f, row) };
}

/* ===================================================================== */
/* §6  scan — progressive build-up (THE EFFECT / DELAY)                   */
/* ===================================================================== */

/*
 * Scan — the slow reveal.  Computing every cell in one go would freeze the
 * program for a second, so we only do a few rows each frame and work down the
 * grid, which also looks nice.  It just decides which cells to compute and when;
 * the actual maths lives in §4, and it never touches colour or placement.
 */
typedef struct {
    int next_row;   /* the next row to compute — how far down we've got */
} Scan;

/* Start the reveal over from the top. */
static void scan_reset(Scan *s) { s->next_row = 0; }

/* Has the reveal reached the bottom (whole picture done)? */
static bool scan_complete(const Scan *s, const Field *f) { return s->next_row >= f->rows; }

/* How far along the reveal is, 0..100, for the status bar. */
static int scan_progress_pct(const Scan *s, const Field *f)
{
    if (f->rows < 1) return 100;
    int pct = s->next_row * 100 / f->rows;
    return pct > 100 ? 100 : pct;
}

/* Compute the next few rows of the picture. */
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
 * Scene — everything about the current picture in one place: the computed grid,
 * the reveal, and the user's current choices.  Keeping it together makes it easy
 * to see what's data, what's the animation, and what the keys control.  The
 * chosen theme survives when you switch patterns; the grid and reveal are rebuilt.
 */
typedef struct {
    Field field;       /* the computed grid of scores */
    Scan  scan;        /* the slow reveal */
    int   seq_idx;     /* which A/B pattern is selected (n/p keys) */
    int   theme;       /* which colour scheme is selected (t/T keys) */
    bool  paused;      /* space bar: freeze the reveal where it is */
} Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);   /* first pattern, first theme, not paused */
}

/* Refit the picture to a new terminal size and start the reveal again. */
static void scene_resize(Scene *s, int term_rows, int term_cols)
{
    field_init(&s->field, term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS, term_cols);
    scan_reset(&s->scan);
}

/* Wipe and redraw the current picture from scratch (same size). */
static void scene_restart(Scene *s)
{
    field_clear(&s->field);
    scan_reset(&s->scan);
}

/* Switch to the next/previous A/B pattern.  A different pattern is a different
 * picture, so we start over. */
static void scene_set_sequence(Scene *s, int dir)
{
    s->seq_idx = (s->seq_idx + dir + N_SEQUENCES) % N_SEQUENCES;
    scene_restart(s);
}

/* Switch colour scheme.  Only recolours — the computed grid stays, so this is
 * instant and survives a pattern change. */
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
