/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snowflake.c  —  Dendritic snowflake fractal (recursive 6-fold branching)
 *
 * A real snow crystal is a DENDRITE: six arms radiate from a centre, and each
 * arm sprouts smaller side-branches, which sprout smaller ones again — the same
 * shape at every scale.  This demo draws that directly as a deterministic fractal:
 *
 *   - six arms at 60° apart (the hexagonal symmetry of ice, H2O ice Ih);
 *   - along each arm, at fixed fractions, a mirror-paired branch sprouts left and
 *     right at ±60°, each a half-size copy of the whole rule;
 *   - recurse to DEPTH levels.
 *
 * Because the rule is deterministic and left/right symmetric, all six arms come
 * out identical and mirror-symmetric — so the full D6 snowflake symmetry (6
 * rotations × 2 reflections) falls out for free, with no symmetry bookkeeping.
 *
 * This is the snowflake's ARMS as a fractal.  Its sibling koch.c draws the
 * snowflake's BOUNDARY as a fractal (the Koch curve); the two are complementary.
 *
 * DEPTH is the knob: depth 1 = bare arms + first branches; higher depth = a finer,
 * lacier crystal.  Colour runs by recursion level (deep core → bright tips).
 *
 * FOUR LAYERS, separated so the geometry is trustworthy and it is clear what runs
 * apart from drawing:
 *
 *   §4 GEOMETRY  — pure vector math: the plane the crystal lives in.
 *   §5 CANVAS    — the cached image: a cell buffer, the math→cell projection, and
 *                  a line-stroke primitive (the WRITE side).
 *   §6 SNOWFLAKE — the recursive branching build that strokes the crystal into the
 *                  canvas.  This is the ONLY heavy work and a PURE FUNCTION of
 *                  (depth, size) — gated by Scene.dirty, run only on a depth or
 *                  resize change, never per frame.
 *   §8 RENDER    — reads the cell buffer and blits glyphs (the READ side).
 *   §7 SCENE     — orchestration: depth + theme + canvas + the dirty flag.
 *
 * EFFECTS / DELAYS: there are none.  This is a static explorer — no animation, no
 * reveal, no timer.  Changing depth rebuilds the canvas instantly; changing the
 * theme is a pure recolor (it re-binds palette pairs; the canvas is NOT rebuilt).
 *
 * Keys:
 *   q / ESC   quit
 *   + / -     increase / decrease depth (1..6)
 *   r         reset to default depth
 *   t / T     cycle color theme forward / back
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra snowflake.c -o snowflake -lncurses -lm
 *
 * Sections
 * --------
 *   §1 config    — constants, branch rule, theme table
 *   §2 clock     — monotonic ns clock + sleep
 *   §3 color     — themeable core→tip ramp (t/T) + HUD pairs
 *   §4 geometry  — Vec2 + add / scale / rotate / from-angle (PURE)
 *   §5 canvas    — cell buffer + math→cell projection + line stroke
 *   §6 snowflake — the recursive 6-fold branching build
 *   §7 scene     — orchestration: depth + theme + canvas + dirty flag
 *   §8 render    — canvas → glyphs + HUD (READ-ONLY)
 *   §9 app       — main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm : A symmetric fractal tree.  snowflake_branch() draws one segment,
 *             then sprouts a half-size copy of itself left and right (±60°) at
 *             each of N_BRANCH_PTS fixed points; six arms tile that around a hub.
 *
 * Math      : Self-similar.  Each level multiplies the branch count by
 *             B = N_BRANCH_PTS × 2 and shrinks length by BRANCH_SCALE, so the
 *             branch set has similarity dimension D = log B / log(1/BRANCH_SCALE).
 *             The ±60° branch angle and 6 arms come from ice's hexagonal lattice.
 *
 * Physics   : Real snow crystals grow as dendrites branching at 60°; this is the
 *             deterministic, idealized version of that morphology (contrast the
 *             stochastic DLA growth model).
 *
 * Rendering : Every branch is a Bresenham line; its glyph follows its direction
 *             (- | / \\), and its colour follows its recursion level.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES — to understand the concepts and the rendering ───────────── *
 *
 *   Self-similar trees, branching & dimension  (§6, CONCEPTS)
 *   ── Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *      Self-similar trees and natural branching; the language of fractal dimension.
 *   ── Barnsley, M. F. (1993). "Fractals Everywhere" (2nd ed.). Academic Press.
 *      Deterministic fractals as collections of contraction maps — exactly the
 *      "half-size copy of myself at ±60°" rule used here.
 *   ── Falconer, K. (2003). "Fractal Geometry: Mathematical Foundations and
 *      Applications" (2nd ed.). Wiley.  The similarity dimension D = log B /
 *      log(1/scale) the CONCEPTS block quotes, done rigorously.
 *   ── Prusinkiewicz, P. & Lindenmayer, A. (1990). "The Algorithmic Beauty of
 *      Plants." Springer.  Recursive branching geometry (turtle / L-systems).
 *
 *   The physics of the shape  (CONCEPTS)
 *   ── Nakaya, U. (1954). "Snow Crystals: Natural and Artificial." Harvard Univ.
 *      Press.  The founding study of snow-crystal morphology — why the six arms
 *      and their dendritic branches form (the Nakaya diagram).
 *   ── Libbrecht, K. G. (2005). "The physics of snow crystals." Rep. Prog. Phys.
 *      68(4), 855–895.  The modern review of hexagonal dendrite growth.
 *
 *   Rendering  (§5, §8)
 *   ── Bresenham, J. E. (1965). "Algorithm for computer control of a digital
 *      plotter." IBM Systems Journal 4(1), 25–30.  The integer line stroke.
 *   ── Gookin, D. (2007). "Programmer's Guide to NCURSES." Wiley.  The cell-
 *      drawing / colour-pair API behind §8.
 * ─────────────────────────────────────────────────────────────────────────── */

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

#define PI  3.14159265358979f

enum {
    DEPTH_MIN     =   1,
    DEPTH_MAX     =   6,
    DEPTH_DEFAULT =   4,

    GRID_ROWS_MAX =  80,
    GRID_COLS_MAX = 300,

    RENDER_FPS    =  30,     /* idle redraw / input-poll rate */
    RAMP_LEN      =   6,     /* colour stops, core → tip      */
    N_THEMES      =   6,
};

/* The crystal's shape, all in math space (the hub is the origin, arm length 1). */
#define N_ARMS         6                  /* 6-fold hexagonal symmetry        */
#define ARM_LENGTH     1.0f               /* main arm length (math units)     */
#define ARM_STEP       (2.0f * PI / N_ARMS)   /* angle between arms (= 60°)   */
#define BRANCH_ANGLE   (PI / 3.0f)        /* side branches sprout at ±60°     */
#define BRANCH_SCALE   0.40f              /* each level is this fraction long */
#define N_BRANCH_PTS   2                  /* sprout points per segment        */
static const float BRANCH_FRAC[N_BRANCH_PTS] = { 0.40f, 0.72f };   /* along the segment */

/* EXTENT — how far (in math units) the crystal reaches from the hub; the canvas
 * scales so a disc of this radius fits the terminal.  A little generous; the
 * rasterizer clips anything that still spills over. */
#define EXTENT       1.3f
#define MARGIN_CELLS 2

#define ASPECT_R    2.0f       /* cell height / width ≈ 2; stretches x so the crystal is round */

/* GLYPH_AXIS_RATIO — when a stroke's run exceeds this multiple of its rise (or vice
 * versa) it is drawn as an axis glyph ('-' or '|') rather than a diagonal. */
#define GLYPH_AXIS_RATIO  2.0f

#define NS_PER_SEC  1000000000LL

/*
 * Theme — a colour ramp from the crystal's core (c[0]) to its tips (c[RAMP_LEN-1]).
 * The renderer colours every branch by its recursion LEVEL (see level_color), so
 * this ramp turns "how deep in the fractal a branch is" into "how bright it is":
 * the long main arms read deep, the fine outer tracery reads bright — which is how
 * the eye separates the fractal's levels at a glance.  Every stop is kept in the
 * bright half of the 256-cube (>= ~33) so even the core stays legible on black
 * (the project's Theme Palette Brightness rule).
 */
typedef struct {
    const char *name;   /* HUD label; how the user names the theme when cycling (t/T)        */
    int c [RAMP_LEN];   /* 256-color ramp, ordered core → tip (deep → bright)               */
    int c8[RAMP_LEN];   /* 8-color fallback, same ordering; the ramp collapses to a few hues */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Ice",    {  39,  45,  51, 117, 159, 195 },
        { COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE } },
    { "Frost",  {  33,  39,  45,  51,  87, 159 },
        { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },
    { "Aurora", {  41,  48,  49,  50,  86, 159 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },
    { "Fire",   { 130, 166, 202, 208, 214, 220 },
        { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "Gold",   { 136, 172, 178, 214, 220, 228 },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE } },
    { "Mono",   { 245, 248, 250, 252, 254, 231 },
        { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE } },
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
/* §3  color                                                              */
/* ===================================================================== */

/* Palette pairs: 1..RAMP_LEN are the core→tip ramp (re-bound per theme);
 * COL_HUD / COL_HINT are theme-independent. */
#define COL_RAMP 1                       /* ramp occupies pairs 1 .. RAMP_LEN */
enum {
    COL_HUD  = 1 + RAMP_LEN,   /* HUD data    — bright yellow */
    COL_HINT,                  /* HUD actions — bright cyan   */
};

/* level_color — the colour pair for a branch at recursion `level` (0 = arm/core),
 * clamped to the ramp so very deep branches keep the brightest tip colour. */
static int level_color(int level)
{
    int idx = level < RAMP_LEN ? level : RAMP_LEN - 1;
    return COL_RAMP + idx;
}

/* theme_apply — bind the core→tip ramp to theme t. */
static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < RAMP_LEN; i++)
        init_pair((short)(COL_RAMP + i),
                  (short)(truecolor ? th->c[i] : th->c8[i]), COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* HUD pairs stay constant across themes: bright yellow data, bright cyan actions */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* Ice at boot; Scene.theme tracks the active one after */
}

/* ===================================================================== */
/* §4  geometry — the plane the crystal lives in (PURE)                   */
/* ===================================================================== */

/*
 * Vec2 — a point or vector in MATH space (x right, y up; the hub is the origin).
 * The whole crystal is built in this clean continuous space — arms, ±60° rotations,
 * branch nodes — and only projected to integer cells at stroke time (§5).  Keeping
 * the geometry in floats is what lets midpoints and rotations stay exact all the
 * way down the recursion.  (Vectors & 2-D rotation: any linear-algebra text.)
 */
typedef struct {
    float x;   /* horizontal component (math units; the main arm length is 1.0) */
    float y;   /* vertical   component (math units; y increases upward)         */
} Vec2;

static Vec2 vec_add(Vec2 a, Vec2 b)        { return (Vec2){ a.x + b.x, a.y + b.y }; }
static Vec2 vec_scale(Vec2 v, float s)     { return (Vec2){ v.x * s, v.y * s }; }
static Vec2 vec_from_angle(float radians)  { return (Vec2){ cosf(radians), sinf(radians) }; }

/* vec_rotate — rotate v by `radians` counter-clockwise (standard 2-D rotation). */
static Vec2 vec_rotate(Vec2 v, float radians)
{
    float c = cosf(radians), s = sinf(radians);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/* ===================================================================== */
/* §5  canvas — cell buffer + projection + line stroke                    */
/* ===================================================================== */

/*
 * Pt — a point in SCREEN space (col, row).  Deliberately a different type from
 * Vec2 so the two coordinate systems can never be mixed up: Vec2 is the ideal
 * plane, Pt is where a Vec2 lands on the terminal after canvas_project() (which
 * flips y and stretches x by ASPECT_R).  Kept sub-cell precise; rounded to ints
 * only inside the Bresenham stroke.
 */
typedef struct {
    float x;   /* column */
    float y;   /* row    */
} Pt;

/*
 * Cell — one drawn cell of the cached image: everything the renderer needs and
 * nothing about the geometry that produced it.  This split is exactly why the heavy
 * build (§6) and the cheap per-frame blit (§8) stay separate — the Canvas caches a
 * grid of these, and the renderer only reads them.
 */
typedef struct {
    uint8_t color;   /* colour-pair slot: a ramp pair (1..RAMP_LEN), or 0 = empty/background */
    char    glyph;   /* line glyph ('-' '|' '/' '\\') chosen from the branch's screen direction */
} Cell;

/*
 * Canvas — the cached image plus the math→screen projection that produced it.
 * gasket-style cache: snowflake_build() writes it, the renderer (§8) only reads
 * it.  scale_x/scale_y and (cx, cy) place a disc of radius EXTENT, centred, with
 * x stretched by ASPECT_R so the hexagon is not squashed by tall cells.
 */
typedef struct {
    /* Row-major cached image, [row][col].  Statically sized to the largest terminal
       we support so the hot path never allocates (project rule); only the top-left
       rows×cols sub-rectangle is live in any given frame. */
    Cell  cell[GRID_ROWS_MAX][GRID_COLS_MAX];
    int   rows, cols;         /* live size, clamped to the GRID_*_MAX bounds            */
    float scale_x, scale_y;   /* math unit → cells per axis; scale_x = scale_y·ASPECT_R */
    int   cx, cy;             /* screen cell of the hub (math origin) — the centre      */
} Canvas;

static void canvas_resize(Canvas *cv, int cols, int rows)
{
    cv->cols = cols < 1 ? 1 : (cols > GRID_COLS_MAX ? GRID_COLS_MAX : cols);
    cv->rows = rows < 1 ? 1 : (rows > GRID_ROWS_MAX ? GRID_ROWS_MAX : rows);

    /* Largest scale fitting a radius-EXTENT disc in both axes (x is ASPECT_R wider). */
    float fit_y = ((float)cv->rows * 0.5f - MARGIN_CELLS) / EXTENT;
    float fit_x = ((float)cv->cols * 0.5f - MARGIN_CELLS) / (EXTENT * ASPECT_R);
    cv->scale_y = fminf(fit_y, fit_x);
    cv->scale_x = cv->scale_y * ASPECT_R;

    cv->cx = cv->cols / 2;
    cv->cy = cv->rows / 2;
}

static void canvas_clear(Canvas *cv)
{
    memset(cv->cell, 0, sizeof cv->cell);
}

/* canvas_project — math-space point → screen point.  y flips (math up = row up). */
static Pt canvas_project(const Canvas *cv, Vec2 v)
{
    return (Pt){ (float)cv->cx + v.x * cv->scale_x,
                 (float)cv->cy - v.y * cv->scale_y };
}

/* canvas_mark — write one cell if it is in bounds (the only guarded write). */
static void canvas_mark(Canvas *cv, int row, int col, uint8_t color, char glyph)
{
    if (row >= 0 && row < cv->rows && col >= 0 && col < cv->cols) {
        cv->cell[row][col].color = color;
        cv->cell[row][col].glyph = glyph;
    }
}

/* stroke_glyph — pick a line glyph from a screen-space direction (rows grow down):
 * mostly-horizontal → '-', mostly-vertical → '|', else the matching diagonal. */
static char stroke_glyph(float dx, float dy)
{
    float run = fabsf(dx), rise = fabsf(dy);
    if (run  > GLYPH_AXIS_RATIO * rise) return '-';
    if (rise > GLYPH_AXIS_RATIO * run)  return '|';
    return ((dx > 0.0f) == (dy > 0.0f)) ? '\\' : '/';
}

/*
 * canvas_draw_line — Bresenham integer line from (x0,y0) to (x1,y1), writing
 * glyph/color into every cell it crosses.  `err` is the running decision
 * accumulator: doubling it each step (the 2·err) lets the choice of which axis to
 * advance be made with integers only — the whole point of Bresenham.
 */
static void canvas_draw_line(Canvas *cv, int x0, int y0, int x1, int y1,
                             uint8_t color, char glyph)
{
    int span_x =  abs(x1 - x0), step_x = x0 < x1 ? 1 : -1;
    int span_y = -abs(y1 - y0), step_y = y0 < y1 ? 1 : -1;
    int err = span_x + span_y;

    for (;;) {
        canvas_mark(cv, y0, x0, color, glyph);
        if (x0 == x1 && y0 == y1) break;
        int err2 = 2 * err;
        if (err2 >= span_y) { err += span_y; x0 += step_x; }   /* advance the column */
        if (err2 <= span_x) { err += span_x; y0 += step_y; }   /* advance the row    */
    }
}

/*
 * canvas_stroke — draw the math-space segment a→b in `color`, in three steps:
 *   1. project both endpoints to screen cells;
 *   2. choose the line glyph from the screen direction;
 *   3. rasterize the line between them.
 */
static void canvas_stroke(Canvas *cv, Vec2 a, Vec2 b, uint8_t color)
{
    Pt   pa    = canvas_project(cv, a);
    Pt   pb    = canvas_project(cv, b);
    char glyph = stroke_glyph(pb.x - pa.x, pb.y - pa.y);
    canvas_draw_line(cv, (int)lroundf(pa.x), (int)lroundf(pa.y),
                         (int)lroundf(pb.x), (int)lroundf(pb.y), color, glyph);
}

/* ===================================================================== */
/* §6  snowflake — the recursive 6-fold branching build                   */
/* ===================================================================== */

/*
 * snowflake_branch — THE fractal rule.  Draw one segment from `base` along `dir`
 * for `length`; then, if depth remains, sprout a half-size copy of this very rule
 * to the left and right (±BRANCH_ANGLE) at each fixed fraction along the segment.
 * `level` only colours the result (0 = arm/core, deeper = brighter tips).
 */
static void snowflake_branch(Canvas *cv, Vec2 base, Vec2 dir, float length,
                             int level, int max_level)
{
    Vec2 tip = vec_add(base, vec_scale(dir, length));
    canvas_stroke(cv, base, tip, (uint8_t)level_color(level));

    if (level >= max_level) return;

    Vec2 left  = vec_rotate(dir,  BRANCH_ANGLE);
    Vec2 right = vec_rotate(dir, -BRANCH_ANGLE);
    for (int i = 0; i < N_BRANCH_PTS; i++) {
        Vec2 node = vec_add(base, vec_scale(dir, length * BRANCH_FRAC[i]));
        snowflake_branch(cv, node, left,  length * BRANCH_SCALE, level + 1, max_level);
        snowflake_branch(cv, node, right, length * BRANCH_SCALE, level + 1, max_level);
    }
}

/*
 * snowflake_build — THE processing: clear the canvas and grow the whole crystal at
 * the given depth.  Six identical arms, rotated 60° apart around the hub; because
 * each arm uses the same mirror-balanced rule, the result carries full D6 symmetry
 * automatically.  A pure function of (depth, canvas size).
 */
static void snowflake_build(Canvas *cv, int depth)
{
    canvas_clear(cv);
    Vec2 hub = { 0.0f, 0.0f };
    for (int arm = 0; arm < N_ARMS; arm++) {
        Vec2 dir = vec_from_angle((float)arm * ARM_STEP);
        snowflake_branch(cv, hub, dir, ARM_LENGTH, 0, depth);
    }
}

/* ===================================================================== */
/* §7  scene — orchestration: depth + theme + canvas + dirty              */
/* ===================================================================== */

/*
 * Scene — the complete logical state of the explorer, independent of the terminal.
 * The orchestration layer: it owns the detail level (depth), the palette (theme),
 * the cached crystal for that depth (canvas), and one flag (dirty) that binds them.
 * Keeping it terminal-agnostic is what lets the build (§6) and the draw (§8) stay
 * separate: anything that changes the geometry just sets dirty, and scene_update()
 * is then the single place that ever pays for a rebuild — and only when one is owed.
 */
typedef struct {
    int    depth;    /* recursion depth — the detail knob (more depth → lacier crystal)  */
    int    theme;    /* index into k_themes[]; changing it is a pure recolor, no rebuild */
    Canvas canvas;   /* §5/§6 cached crystal rendered at `depth`                         */
    bool   dirty;    /* true ⇒ depth or terminal size changed and `canvas` is stale.
                        Set by scene_set_depth / scene_resize; cleared by scene_update()
                        once it rebuilds.  This flag is the entire reason idle frames
                        are cheap. */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->depth = DEPTH_DEFAULT;
    s->theme = 0;
    canvas_resize(&s->canvas, cols, rows);
    s->dirty = true;
}

/* scene_update — the only non-render processing: rebuild the crystal if (and only
 * if) depth or size changed.  Idle frames skip it. */
static void scene_update(Scene *s)
{
    if (!s->dirty) return;
    snowflake_build(&s->canvas, s->depth);
    s->dirty = false;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    canvas_resize(&s->canvas, cols, rows);
    s->dirty = true;
}

/* scene_set_depth — clamp to [DEPTH_MIN, DEPTH_MAX]; mark dirty on a real change. */
static void scene_set_depth(Scene *s, int depth)
{
    if (depth < DEPTH_MIN) depth = DEPTH_MIN;
    if (depth > DEPTH_MAX) depth = DEPTH_MAX;
    if (depth != s->depth) {
        s->depth = depth;
        s->dirty = true;
    }
}

/* scene_cycle_theme — a pure recolor: re-bind the ramp, leave the canvas. */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* ===================================================================== */
/* §8  render — canvas → glyphs + HUD (READ-ONLY)                         */
/* ===================================================================== */

/*
 * Screen — the live terminal size, refreshed from getmaxyx() at start-up and after
 * every SIGWINCH.  Held apart from Scene on purpose: it is a property of the
 * display, not of the crystal.  The HUD clamps its text to these bounds and the
 * canvas carves its drawing area out of them.
 */
typedef struct {
    int rows;   /* terminal height, in character cells */
    int cols;   /* terminal width,  in character cells */
} Screen;

/* render_canvas — blit the cached cells.  Read-only: never writes the Canvas. */
static void render_canvas(const Canvas *cv)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            Cell c = cv->cell[row][col];
            if (c.color == 0) continue;
            attron(COLOR_PAIR((int)c.color) | A_BOLD);
            mvaddch(row, col, (chtype)(unsigned char)c.glyph);
            attroff(COLOR_PAIR((int)c.color) | A_BOLD);
        }
    }
}

/* hud_line — one HUD line at (row, x), truncated to the screen width so it can
 * never wrap or overrun, however long the string or narrow the terminal. */
static void hud_line(int row, int x, int cols, int pair, attr_t attr, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/* branch_count — total segments drawn at this depth: 6 arms × Σ Bˡ, B = children. */
static int branch_count(int depth)
{
    int children = N_BRANCH_PTS * 2;
    int per_arm = 0, term = 1;
    for (int l = 0; l <= depth; l++) { per_arm += term; term *= children; }
    return N_ARMS * per_arm;
}

/*
 * render_hud — data on top, actions on the bottom:
 *   row 0      (yellow bold)  depth, segment count, theme
 *   row rows-1 (cyan bold)    every interactive key
 */
static void render_hud(const Screen *s, const Scene *sc)
{
    char buf[96];
    snprintf(buf, sizeof buf, " Snowflake  depth:%d  segments:%d  theme:%s ",
             sc->depth, branch_count(sc->depth), k_themes[sc->theme].name);
    hud_line(0, 0, s->cols, COL_HUD, A_BOLD, buf);

    hud_line(s->rows - 1, 0, s->cols, COL_HINT, A_BOLD,
             " q:quit  +/-:depth  r:reset  t:theme ");
}

/* ===================================================================== */
/* §9  app — main loop                                                    */
/* ===================================================================== */

/*
 * App — the whole running program in one object: the Scene being explored, the
 * Screen it is drawn on, and the two flags the OS pokes asynchronously.  Bundling
 * them is what lets a single file-scope instance (g_app) be reached from the signal
 * handler while everything else is passed by pointer.
 */
typedef struct {
    Scene                 scene;         /* §7 depth + theme + canvas + dirty */
    Screen                screen;        /* §8 terminal size                  */
    volatile sig_atomic_t running;       /* cleared by SIGINT/SIGTERM or 'q'  */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH                   */
} App;

/* The one global: signal handlers receive only an int, so the flags they set must
 * be reachable without a parameter (volatile sig_atomic_t).  Everything else is
 * reached through this object. */
static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static void terminal_init(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);     /* getch() returns ERR instead of blocking */
    keypad(stdscr, TRUE);
    typeahead(-1);             /* don't let pending input interrupt the diff write */
    color_init();
}

/* app_apply_resize — after SIGWINCH: rebuild ncurses' screen model, re-read the
 * size, and mark the canvas stale so the next frame rebuilds at the new size. */
static void app_apply_resize(App *app)
{
    endwin();
    refresh();
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/* app_handle_key — translate one keypress into a scene action; false = quit. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case '+': case '=': scene_set_depth(&app->scene, app->scene.depth + 1); break;
    case '-': case '_': scene_set_depth(&app->scene, app->scene.depth - 1); break;

    case 'r': case 'R': scene_set_depth(&app->scene, DEPTH_DEFAULT); break;

    case 't': scene_cycle_theme(&app->scene, +1); break;
    case 'T': scene_cycle_theme(&app->scene, -1); break;

    default: break;
    }
    return true;
}

static void app_drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR)
        if (!app_handle_key(app, ch)) { app->running = 0; break; }
}

/* app_present — compose one frame: clear, blit the cached canvas, overlay the HUD,
 * and flush as a single terminal diff. */
static void app_present(App *app)
{
    erase();
    render_canvas(&app->scene.canvas);
    render_hud(&app->screen, &app->scene);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    terminal_init();

    App *app     = &g_app;
    app->running = 1;
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    while (app->running) {
        if (app->need_resize) app_apply_resize(app);

        int64_t frame_start = clock_ns();

        app_drain_input(app);          /* 1. input   → depth/theme edits (sets dirty) */
        scene_update(&app->scene);     /* 2. process → rebuild the crystal if dirty   */
        app_present(app);              /* 3. render  → blit cached canvas + HUD       */

        clock_sleep_ns(NS_PER_SEC / RENDER_FPS - (clock_ns() - frame_start));
    }
    return 0;
}
