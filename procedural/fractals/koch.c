/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * koch.c  —  Koch snowflake fractal, animated level-by-level
 *
 * The Koch snowflake starts as an equilateral triangle.  Each straight
 * edge is repeatedly replaced by four shorter edges: the middle third is
 * removed and replaced with an outward equilateral bump.  Repeating this
 * rule produces the classic snowflake fractal boundary.
 *
 * Animation — the outline is drawn stroke-by-stroke at one detail level.
 * Five levels are available; 'n' steps to the next (no auto-advance — the
 * view holds once a level finishes drawing).  Level 3 is shown at start.
 *
 *   Level 1 :   12 segments  — triangle with bumps
 *   Level 2 :   48 segments  — bumps on bumps
 *   Level 3 :  192 segments  — fine detail emerging   ← default
 *   Level 4 :  768 segments  — very fine detail
 *   Level 5 : 3072 segments  — near-fractal resolution
 *
 * Colour runs along the curve (first segment → last) and is themeable
 * with t / T.
 *
 * Koch subdivision rule — for segment A → B:
 *   P = A + (B-A)/3
 *   Q = A + (B-A)*2/3
 *   M = P + R(+60°)(Q-P)       ← outward equilateral bump peak
 *   Replace with: A→P, P→M, M→Q, Q→B
 *
 * All segment geometry is stored in Euclidean coordinates (circumradius=1)
 * and converted to terminal (col, row) at draw time, applying ASPECT_R to
 * keep the snowflake circular rather than squashed.
 *
 * Keys:
 *   q / ESC   quit
 *   n         next level
 *   t / T     cycle color theme (Aurora/Fire/Ice/Toxic/Mono)
 *   r         reset to the default level (3)
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra koch.c -o koch -lncurses -lm
 *
 * Reading order — built bottom-up; each section uses only earlier ones.
 *   §1  config   — constants, palette slots, themes
 *   §2  clock    — monotonic time
 *   §3  geometry — Vec2 (the plane the snowflake lives on) + a subtract helper
 *   §4  curve    — Segment, KochCurve, the subdivision rule + build
 *   §5  color    — gradient-along-curve + theme apply
 *   §6  canvas   — cell buffer, Euclid→cell projection, Bresenham stroke
 *   §7  reveal   — stroke-by-stroke draw progress
 *   §8  scene    — Scene: curve + canvas + reveal + theme
 *   §9  screen   — ncurses lifecycle + HUD
 *   §10 app      — signals, input, main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Iterative edge-replacement (segment subdivision).  At each
 *                  level every segment A→B becomes four: A→P, P→M, M→Q, Q→B,
 *                  where M is the peak of an outward equilateral bump on the
 *                  middle third.  No recursion stack of state — segments are
 *                  written straight into a flat array.
 *
 * Math           : After n levels — segment count 3·4ⁿ, segment length (1/3)ⁿ,
 *                  perimeter (4/3)ⁿ → ∞, enclosed area → (2/5)·triangle.  Fractal
 *                  dimension D = log4/log3 ≈ 1.26 (between a curve and an area).
 *                  The bump peak comes from rotating the edge's middle third by
 *                  +60° (equilateral-triangle geometry).
 *
 * Data model     : Small structs, one idea each —
 *                    Vec2        a point in the snowflake's plane
 *                    Segment     one edge A→B; the curve is an array of these
 *                    KochCurve   the segments + level (the geometry)
 *                    Canvas      the cell buffer + Euclid→cell projection
 *                    Reveal      the stroke-by-stroke draw progress
 *                    Scene       all of the above = the live picture
 *
 * Rendering      : Each Euclidean segment is projected to terminal cells (centre
 *                  + scale + ASPECT_R) and rasterised with Bresenham; colour
 *                  runs along the curve so you watch it draw.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 * Concepts & maths
 *   [1] von Koch, H. (1904). "Sur une courbe continue sans tangente, obtenue
 *       par une construction géométrique élémentaire." Arkiv för Matematik 1:
 *       681-704.  — the original paper that introduced the curve as an example
 *       of a continuous, nowhere-differentiable function.  The source.
 *   [2] Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *       — the Koch curve as the canonical fractal: infinite perimeter bounding
 *       finite area, and self-similarity dimension D = log4/log3 ≈ 1.26.  Start
 *       here for the why.
 *   [3] Peitgen, Jürgens & Saupe (1992). "Chaos and Fractals: New Frontiers of
 *       Science." Springer.  — the most readable construction chapter: the
 *       edge-replacement rule (§4), the length (4/3)ⁿ→∞ and area→(2/5)·triangle
 *       limits, and the dimension worked end to end.
 *   [4] Falconer, K. (2003). "Fractal Geometry: Mathematical Foundations and
 *       Applications" (2nd ed.). Wiley.  — rigorous Hausdorff dimension; derives
 *       D = log4/log3 for the Koch curve as a self-similar set.
 *
 * Rendering, L-systems & line drawing
 *   [5] Prusinkiewicz, P. & Lindenmayer, A. (1990). "The Algorithmic Beauty of
 *       Plants." Springer (free at algorithmicbotany.org).  — the Koch curve as
 *       the L-system F→F+F--F+F; the edge-rewriting view that §4's subdivision
 *       implements directly, without a symbol string.
 *   [6] Bresenham, J. E. (1965). "Algorithm for computer control of a digital
 *       plotter." IBM Systems Journal 4(1):25-30.  — the integer line-stepping
 *       algorithm canvas_stroke uses in §6 to rasterise each segment.
 *
 * In-repo study companions
 *   [7] dragon_curve.c — another fractal curve traced as segments; same
 *       Reveal-style stroke animation, a different replacement rule.
 *   [8] l_system.c — draws the Koch curve from a rewrite grammar; contrast its
 *       generic symbol rewriting with the explicit subdivision in §4.
 *   [9] buddhabrot.c — the palette-table pattern §1/§5's themes borrow.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  30,
    SIM_FPS_MAX     =  60,
    SIM_FPS_STEP    =   5,

    MAX_LEVEL       =   5,    /* highest detail level                   */
    DEFAULT_LEVEL   =   3,    /* level shown at start and on reset       */
    N_KOCH_COLORS   =   5,    /* color bands along the drawn curve       */
    N_THEMES        =   5,    /* colour themes to cycle with t/T         */

    LEVEL_DRAW_TICKS = 60,    /* ticks to draw a whole level (~2 s @30fps) */

    /*
     * MAX_SEGS must hold the largest level: 3 × 4^MAX_LEVEL.
     * Level 5 → 3 × 1024 = 3072.  4096 gives a safe margin.
     */
    MAX_SEGS        = 4096,

    CANVAS_ROWS_MAX =  80,
    CANVAS_COLS_MAX = 300,

    HUD_COLS        =  80,
    FPS_UPDATE_MS   = 500,
};

/*
 * ASPECT_R — terminal cell height / width ≈ 2.  Corrects for non-square cells
 * so the snowflake looks circular rather than squashed.
 *
 * SIN60 / COS60 — the 60° rotation used to find the bump peak.
 */
#define ASPECT_R  2.0f
#define SIN60     0.8660254f
#define COS60     0.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the colour-pair slots.  A drawn cell holds COL_K1..K5, the gradient
 * ALONG the curve (first segment drawn → last); the HUD pairs are theme-
 * independent so the text stays legible on any palette.
 */
typedef enum {
    COL_K1  = 1,   /* first segments drawn */
    COL_K2  = 2,
    COL_K3  = 3,
    COL_K4  = 4,
    COL_K5  = 5,   /* last segments drawn  */
    COL_HUD = 6,   /* top data lines    — yellow */
    COL_HINT = 7,  /* bottom action bar — cyan   */
} ColorID;

/*
 * Theme — a named 5-colour gradient painted ALONG the curve (K1 first segment
 * drawn → K5 last), cycled with t / T.
 *
 * WHY a table of colours, not one: the snowflake is one continuous stroke, so a
 * gradient along draw-order turns "when was this drawn" into a visible hue — you
 * watch the colour sweep around the outline as it fills in.
 * CONTEXT: every value is kept in the bright half of the 256-cube so the whole
 * stroke stays legible on black (dim cube/gray indices vanish under A_DIM).
 * Adding a theme is a single table row — the palette-table pattern from
 * buddhabrot.c (ref [9]); the engine never changes.
 */
typedef struct {
    const char *name;            /* shown in the HUD, e.g. "Aurora"          */
    int fg256[N_KOCH_COLORS];    /* xterm-256 codes, K1..K5 (preferred)       */
    int fg8[N_KOCH_COLORS];      /* 8-colour ANSI fallback, same K1..K5 order */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*            name      K1   K2   K3   K4   K5      8-colour fallback (same order) */
    { "Aurora", {  51,  86, 118, 226, 231 }, { COLOR_CYAN,  COLOR_CYAN,  COLOR_GREEN,  COLOR_YELLOW, COLOR_WHITE  } },
    { "Fire",   { 196, 202, 208, 220, 231 }, { COLOR_RED,   COLOR_RED,   COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE  } },
    { "Ice",    {  39,  45,  51, 117, 231 }, { COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
    { "Toxic",  {  46,  82, 118, 154, 226 }, { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW } },
    { "Mono",   { 245, 248, 250, 252, 231 }, { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  } },
};

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
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  geometry — points in the snowflake's plane                         */
/* ===================================================================== */

/*
 * Vec2 — a point (or direction) in the snowflake's continuous Euclidean plane.
 *
 * WHY name the type: the whole curve is built from these, and the Koch rule (§4)
 * is pure 2-D vector geometry — thirds along an edge, a +60° rotation for the
 * bump.  Giving the pair a name lets §4 read like the formula (ref [1]) instead
 * of a tangle of x1/y1/x2/y2 floats.
 * CONTEXT: coordinates are resolution-independent — the snowflake is defined at
 * circumradius 1, centred on the origin, and only mapped to cells at draw time
 * (canvas_project, §6).  So the geometry never depends on terminal size.
 */
typedef struct {
    float x;   /* horizontal, +x right; the triangle spans x ∈ [−√3/2, √3/2] */
    float y;   /* vertical,   +y up;    apex at y = 1, base at y = −1/2       */
} Vec2;

/* b − a, used as a direction vector */
static inline Vec2 vec2_sub(Vec2 a, Vec2 b) { return (Vec2){ a.x - b.x, a.y - b.y }; }

/* ===================================================================== */
/* §4  curve — the Koch geometry                                          */
/* ===================================================================== */

/*
 * Segment — one straight edge A→B of the curve, in Euclidean coordinates.
 * The atom of the whole construction: subdivision turns one Segment into four
 * (ref [1]), and the final curve is just an ordered array of them.  Order is the
 * walk around the outline, which is also the draw order the gradient follows.
 */
typedef struct {
    Vec2 a;   /* start point of the edge */
    Vec2 b;   /* end point   of the edge */
} Segment;

/*
 * KochCurve — the fractal flattened to an ordered list of segments at one fold
 * level.  koch_build fills it; the renderer (§6) just walks seg[0..count).
 *
 * WHY flatten instead of recurse at draw time: building once into an array means
 * the hot loop is a linear scan (no per-frame recursion, no malloc), and the
 * stroke order gives the colour gradient and the reveal animation for free.
 * SIZING: at level n the 3 triangle edges become 3·4ⁿ segments (ref [3]); seg[]
 * is sized for the largest level (see MAX_SEGS in §1).
 */
typedef struct {
    int     level;            /* fold depth this was built at, 1..MAX_LEVEL    */
    int     count;            /* live segments in seg[]; equals 3·4^level      */
    Segment seg[MAX_SEGS];    /* the outline, in walk order (= draw/colour order) */
} KochCurve;

/*
 * koch_subdivide — THE rule.  Replace edge A→B with four edges A→P→M→Q→B:
 *   P, Q  cut the edge into thirds
 *   M     pushes the middle third out into an equilateral bump (rotate +60°)
 * Recurse on each of the four until depth 0, where the segment is emitted.
 */
static void koch_subdivide(KochCurve *k, Vec2 a, Vec2 b, int depth)
{
    if (depth == 0) {
        if (k->count < MAX_SEGS) k->seg[k->count++] = (Segment){ a, b };
        return;
    }
    /* P, Q cut the edge into thirds; M is the middle third rotated +60° outward */
    Vec2 p = { a.x + (b.x - a.x) / 3.0f,        a.y + (b.y - a.y) / 3.0f };
    Vec2 q = { a.x + (b.x - a.x) * 2.0f / 3.0f, a.y + (b.y - a.y) * 2.0f / 3.0f };
    Vec2 d = vec2_sub(q, p);                     /* the middle third, as a vector */
    Vec2 m = { p.x + COS60 * d.x - SIN60 * d.y,  /* d rotated +60° (2-D rotation), */
               p.y + SIN60 * d.x + COS60 * d.y };/* offset from P → the bump peak  */

    koch_subdivide(k, a, p, depth - 1);
    koch_subdivide(k, p, m, depth - 1);
    koch_subdivide(k, m, q, depth - 1);
    koch_subdivide(k, q, b, depth - 1);
}

/*
 * koch_build — start from a CW equilateral triangle (circumradius 1) and
 * subdivide each edge to `level`.
 *   V1 = (0, 1)        top
 *   V2 = (√3/2, −1/2)  bottom-right
 *   V3 = (−√3/2, −1/2) bottom-left
 */
static void koch_build(KochCurve *k, int level)
{
    if (level < 1)         level = 1;
    if (level > MAX_LEVEL) level = MAX_LEVEL;
    k->level = level;
    k->count = 0;

    Vec2 v1 = {  0.0f,   1.0f };
    Vec2 v2 = {  SIN60, -0.5f };
    Vec2 v3 = { -SIN60, -0.5f };
    koch_subdivide(k, v1, v2, level);
    koch_subdivide(k, v2, v3, level);
    koch_subdivide(k, v3, v1, level);
}

/* ===================================================================== */
/* §5  color — gradient along the curve, bound to the active theme        */
/* ===================================================================== */

/*
 * curve_band — colour band 1..N_KOCH_COLORS for the segment at draw index `idx`,
 * so the colour sweeps along the curve from K1 (first drawn) to K5 (last).
 */
static uint8_t curve_band(int idx, int count)
{
    int band = 1 + (int)((float)idx / (float)count * (float)N_KOCH_COLORS);
    return (uint8_t)(band > N_KOCH_COLORS ? N_KOCH_COLORS : band);
}

/* Bind the five curve bands (COL_K1..K5) to the active theme. */
static void theme_apply(int theme)
{
    const Theme *t = &k_themes[theme];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < N_KOCH_COLORS; i++)
        init_pair(COL_K1 + i, truecolor ? t->fg256[i] : t->fg8[i], COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* HUD/HINT are theme-independent — yellow data, cyan actions on any palette */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* Aurora; Scene.theme tracks the active one thereafter */
}

/* ===================================================================== */
/* §6  canvas — the cell buffer and the Euclid → cell projection          */
/* ===================================================================== */

/*
 * Cell — an integer terminal cell (column, row): where a continuous Euclidean
 * point lands after projection.  The bridge between the float world (Vec2) and
 * the character grid — canvas_project makes one, canvas_stroke walks between two.
 */
typedef struct {
    int col;   /* terminal column (x), 0 = left */
    int row;   /* terminal row    (y), 0 = top  */
} Cell;

/*
 * Canvas — the picture in memory: one ColorID per terminal cell, plus the lens
 * that maps the unit snowflake onto those cells.
 *
 * WHY a buffer instead of drawing straight to ncurses: it splits "where does the
 * geometry land + what colour" (the fractal) from "push bytes to the terminal"
 * (canvas_paint).  The reveal animation also just re-paints the same buffer.
 * PROJECTION (ref [6] for the stroke that fills it): a point is centred, scaled
 * by `scale` columns per Euclidean unit, and its row divided by ASPECT_R so the
 * shape reads as a circle on tall terminal cells rather than a squashed oval.
 */
typedef struct {
    int     rows, cols;                              /* live drawing area in cells   */
    float   scale;                                   /* columns per Euclidean unit   */
    uint8_t cell[CANVAS_ROWS_MAX][CANVAS_COLS_MAX];  /* 0 = empty, else COL_K1..K5   */
} Canvas;

static void canvas_reset(Canvas *cv, int cols, int rows)
{
    if (cols > CANVAS_COLS_MAX) cols = CANVAS_COLS_MAX;
    if (rows > CANVAS_ROWS_MAX) rows = CANVAS_ROWS_MAX;
    cv->cols = cols;
    cv->rows = rows;
    memset(cv->cell, 0, sizeof cv->cell);

    /* fit the radius-1 shape, constrained by both width and height */
    float by_cols = (float)cols * 0.45f;
    float by_rows = (float)rows * 0.45f * ASPECT_R;
    cv->scale = (by_cols < by_rows) ? by_cols : by_rows;
}

/* project a Euclidean point to a terminal cell (centred, aspect-corrected) */
static Cell canvas_project(const Canvas *cv, Vec2 p)
{
    return (Cell){
        .col = cv->cols / 2 + (int)roundf(p.x * cv->scale),
        .row = cv->rows / 2 - (int)roundf(p.y * cv->scale / ASPECT_R),
    };
}

/*
 * canvas_stroke — rasterise one segment into the cell buffer with Bresenham's
 * line algorithm (the standard integer DDA — no floating point per pixel).
 */
static void canvas_stroke(Canvas *cv, Segment s, uint8_t color)
{
    Cell a = canvas_project(cv, s.a);
    Cell b = canvas_project(cv, s.b);

    int x0 = a.col, y0 = a.row, x1 = b.col, y1 = b.row;
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if (x0 >= 0 && x0 < cv->cols && y0 >= 0 && y0 < cv->rows)
            cv->cell[y0][x0] = color;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void canvas_paint(const Canvas *cv, WINDOW *w)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            uint8_t c = cv->cell[row][col];
            if (c == 0) continue;
            attr_t attr = COLOR_PAIR((int)c);
            if (c >= COL_K4) attr |= A_BOLD;          /* brighten the late strokes */
            wattron(w, attr);
            mvwaddch(w, row, col, (chtype)'*');
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §7  reveal — the stroke-by-stroke draw progress                        */
/* ===================================================================== */

/*
 * Reveal — the stroke-by-stroke draw progress: how much of the curve is on
 * screen and how fast more appears.
 *
 * WHY animate at all: drawing the outline in walk order makes the construction
 * legible — you see the gradient sweep around as edges fill in, instead of the
 * finished shape popping in at once.  `drawn` climbs by `per_tick` each tick
 * until it reaches curve.count, then the picture simply holds (no auto-advance).
 * TUNING: per_tick is derived from the segment count so every level finishes in
 * roughly the same wall-clock time (~LEVEL_DRAW_TICKS), whether it's 12 segments
 * or 3072 — see reveal_begin.
 */
typedef struct {
    int  drawn;       /* segments painted so far, 0..curve.count           */
    int  per_tick;    /* segments added each tick (∝ count, set by reveal_begin) */
    bool paused;      /* spc/p freezes the reveal; survives level change & resize */
} Reveal;

static void reveal_begin(Reveal *r, int count)
{
    r->drawn    = 0;
    r->per_tick = count / LEVEL_DRAW_TICKS + 1;
    /* paused is left as-is, so it survives a level change / resize */
}

static bool reveal_complete(const Reveal *r, int count) { return r->drawn >= count; }

/* ===================================================================== */
/* §8  scene — curve + canvas + reveal = the live picture                 */
/* ===================================================================== */

/*
 * Scene — the whole live picture in one place: the geometry, the buffer it's
 * painted into, the animation that fills it, and the active palette.
 *
 * WHY group them: they share a lifecycle.  A level change, a reset, or a resize
 * rebuilds the curve, re-fits the canvas, and restarts the reveal together; one
 * struct keeps those moves atomic and gives the §10 input layer a single handle
 * to act on.  The split mirrors the pipeline: geometry → buffer → reveal → colour.
 */
typedef struct {
    KochCurve curve;     /* the fractal geometry at the current level   */
    Canvas    canvas;    /* the cell buffer it's projected & drawn into  */
    Reveal    reveal;    /* how far the stroke-by-stroke draw has got    */
    int       theme;     /* index into k_themes, cycled with t / T       */
} Scene;

/*
 * scene_setup — (re)build the snowflake at `level` for the current screen size:
 * size the canvas, build the curve, restart the reveal.  Leaves theme and the
 * paused flag alone, so resize / next-level keep them.
 */
static void scene_setup(Scene *s, int cols, int rows, int level)
{
    canvas_reset(&s->canvas, cols, rows);
    koch_build(&s->curve, level);
    reveal_begin(&s->reveal, s->curve.count);
}

/* First start / reset: the default level (theme & paused stay at BSS defaults). */
static void scene_init(Scene *s, int cols, int rows)
{
    scene_setup(s, cols, rows, DEFAULT_LEVEL);
}

static void scene_next_level(Scene *s, int cols, int rows)
{
    scene_setup(s, cols, rows, s->curve.level % MAX_LEVEL + 1);   /* wraps 5 → 1 */
}

static void scene_cycle_theme(Scene *s)
{
    s->theme = (s->theme + 1) % N_THEMES;
    theme_apply(s->theme);
}

/*
 * scene_tick — draw the next batch of strokes.  Holds once the level is fully
 * drawn (no auto-advance — press 'n' for the next level).
 */
static void scene_tick(Scene *s)
{
    if (s->reveal.paused) return;
    if (reveal_complete(&s->reveal, s->curve.count)) return;

    int end = s->reveal.drawn + s->reveal.per_tick;
    if (end > s->curve.count) end = s->curve.count;

    for (int i = s->reveal.drawn; i < end; i++)
        canvas_stroke(&s->canvas, s->curve.seg[i], curve_band(i, s->curve.count));

    s->reveal.drawn = end;
}

static void scene_draw(const Scene *s, WINDOW *w) { canvas_paint(&s->canvas, w); }

/* ===================================================================== */
/* §9  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the terminal's current size, cached so layout (HUD position, canvas
 * fit) reads it without calling getmaxyx every frame.  Refreshed on init and on
 * every SIGWINCH (screen_resize); the canvas re-fits to whatever it holds.
 */
typedef struct {
    int cols;   /* terminal width  in cells (getmaxyx x) */
    int rows;   /* terminal height in cells (getmaxyx y) */
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

static void screen_free(Screen *s)  { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * hud_line — draw one HUD line at (row, x) in colour `pair`, clamped to the
 * terminal width so a long line never wraps onto the snowflake.
 */
static void hud_line(int row, int x, int pair, attr_t bold, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | bold);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | bold);
}

/*
 * HUD layout — top is data, bottom is actions:
 *   row 0      (bold,  right)  title, fps, run state
 *   row 1      (plain, left)   level N/total, theme, % complete, speed
 *   row rows-1 (cyan,  left)   every interactive key
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    int count = sc->curve.count;
    int pct = (count > 0) ? sc->reveal.drawn * 100 / count : 100;
    if (pct > 100) pct = 100;
    const char *state = sc->reveal.paused ? "PAUSED "
                      : (reveal_complete(&sc->reveal, count) ? "complete" : "drawing ");

    char buf[HUD_COLS + 1];

    /* row 0 — primary data, right-aligned + bold */
    snprintf(buf, sizeof buf, " KochSnowflake  %5.1f fps  %s ", fps, state);
    hud_line(0, s->cols - (int)strlen(buf), COL_HUD, A_BOLD, s->cols, buf);

    /* row 1 — parameter readouts, not bold so row 0 stays dominant */
    snprintf(buf, sizeof buf, " level %d/%d  theme:%s  %d%%  spd:%d Hz ",
             sc->curve.level, MAX_LEVEL, k_themes[sc->theme].name, pct, sim_fps);
    hud_line(1, 0, COL_HUD, A_NORMAL, s->cols, buf);

    /* last row — actions, bright cyan + bold */
    hud_line(s->rows - 1, 0, COL_HINT, A_BOLD, s->cols,
             " q:quit  n:next level  t:theme  r:reset  spc:pause  [ / ]:speed ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10  app — signals, input, main loop                                   */
/* ===================================================================== */

/*
 * App — the whole running program: what to draw (scene), where (screen), and how
 * fast the reveal advances (sim_fps).  One value lives in main as static (BSS)
 * because Scene embeds the large seg[]/cell[] arrays — too big for the stack.
 * It's the single object every input handler and the main loop mutate.
 */
typedef struct {
    Scene  scene;       /* the live picture (geometry + canvas + reveal + theme) */
    Screen screen;      /* cached terminal size for layout                       */
    int    sim_fps;     /* reveal ticks/sec, SIM_FPS_MIN..MAX, stepped by [ / ]  */
} App;

/* The only globals: POSIX signal handlers take no arguments, so they must
 * write a flag the main loop polls. */
static volatile sig_atomic_t g_should_quit   = 0;
static volatile sig_atomic_t g_should_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_should_quit   = 1;
    if (s == SIGWINCH)               g_should_resize = 1;
}

static void cleanup(void) { endwin(); }

static void app_resize(App *a)
{
    screen_resize(&a->screen);
    /* preserve the current level across a resize (only the fit scale changes) */
    scene_setup(&a->scene, a->screen.cols, a->screen.rows, a->scene.curve.level);
    g_should_resize = 0;
}

static void app_change_speed(App *a, int delta)
{
    int fps = a->sim_fps + delta;
    if (fps < SIM_FPS_MIN) fps = SIM_FPS_MIN;
    if (fps > SIM_FPS_MAX) fps = SIM_FPS_MAX;
    a->sim_fps = fps;
}

static bool app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'n': case 'N':
        scene_next_level(&a->scene, a->screen.cols, a->screen.rows);
        break;

    case 't': case 'T':
        scene_cycle_theme(&a->scene);
        break;

    case 'r': case 'R':
        scene_init(&a->scene, a->screen.cols, a->screen.rows);   /* default level */
        break;

    case 'p': case 'P': case ' ':
        a->scene.reveal.paused = !a->scene.reveal.paused;
        break;

    case ']': app_change_speed(a, +SIM_FPS_STEP); break;
    case '[': app_change_speed(a, -SIM_FPS_STEP); break;

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

    /* static (BSS) — Scene holds large arrays; keep them off the stack */
    static App app = { .sim_fps = SIM_FPS_DEFAULT };

    screen_init(&app.screen);
    scene_init(&app.scene, app.screen.cols, app.screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (!g_should_quit) {

        if (g_should_resize) {
            app_resize(&app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app.sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app.scene);
            sim_accum -= tick_ns;
        }

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

        screen_draw(&app.screen, &app.scene, fps_display, app.sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(&app, ch))
            g_should_quit = 1;
    }

    screen_free(&app.screen);
    return 0;
}
