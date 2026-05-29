/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dragon_curve.c — Paper-Folding Dragon Curve
 *
 * The dragon curve is built by iterative right-folding of a paper strip:
 *
 *   Gen 1: [R]
 *   Gen 2: [R  R  L]
 *   Gen 3: [R  R  L  R  R  L  L]
 *
 * Each generation appends a right-turn midpoint plus the reverse-complement
 * of the existing sequence.  The result is a space-filling path that never
 * self-intersects.
 *
 * Animation draws the path segment by segment.  Color encodes generation
 * depth: dark blue (gen 1-2) → cyan → green → yellow → white (gen 13).
 *
 * Keys: q quit  SPACE pause/resume  r restart  +/- speed  n/p generation
 *
 * Build (no -lm — this file uses no math-library functions):
 *   gcc -std=c11 -O2 -Wall -Wextra dragon_curve.c -o dragon_curve -lncurses
 *
 * Reading order — built bottom-up; each section uses only earlier ones.
 *   §1  config   — constants, palette slots
 *   §2  clock    — monotonic time
 *   §3  geometry — Point, Heading, BBox (the integer grid the curve lives on)
 *   §4  turtle   — Turn alphabet + Turtle that walks turns into a path
 *   §5  curve    — DragonCurve: fold the turn sequence, trace it, measure bounds
 *   §6  color    — generation depth → colour band
 *   §7  fit      — Fit: map grid points onto terminal cells (aspect-corrected)
 *   §8  reveal   — Reveal: segment-by-segment animation state
 *   §9  scene    — Scene: composes curve + reveal into the running picture
 *   §10 screen   — ncurses lifecycle + HUD
 *   §11 app      — signals, input, main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Paper-folding sequence construction.
 *                  The turn sequence for generation n is:
 *                    T_n = T_{n-1}  R  reverse-complement(T_{n-1})
 *                  where R = right-turn and reverse-complement flips L↔R and
 *                  reverses order.
 *
 * Math           : After n folds the sequence has 2ⁿ−1 turns and 2ⁿ points
 *                  (so 2ⁿ−1 segments).  Gen 13 → 8191 turns, 8192 points.
 *                  The path never self-intersects, and 4 copies at 0/90/180/270°
 *                  tile the plane — the dragon is a rep-tile of order 4.
 *
 * Data model     : Small structs, one idea each —
 *                    Turn / Turtle  the L/R alphabet, and the pen that walks it
 *                    DragonCurve    turn sequence + traced path + bounding box
 *                    Fit            maps grid points onto terminal cells
 *                    Reveal         the segment-by-segment animation
 *                    Scene          the curve + its animation = the live picture
 *
 * Rendering      : Segments drawn one per frame via turtle graphics.  Colour
 *                  encodes a segment's age (the generation it was born in),
 *                  exposing the folding hierarchy.  Terminal cells are ~2× taller
 *                  than wide, so the x scale is doubled (ASPECT_R) to stay square.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 * Concepts & maths
 *   [1] Davis, C. & Knuth, D. E. (1970). "Number Representations and Dragon
 *       Curves, I & II." J. Recreational Mathematics 3:66-81, 133-149.  — the
 *       canonical mathematical analysis of this curve.  Start here.
 *   [2] Gardner, M. (1967). "Mathematical Games." Scientific American.  —
 *       popularised the Heighway dragon (found by NASA physicists Heighway,
 *       Banks & Harter); the readable origin story.
 *   [3] Allouche, J.-P. & Shallit, J. (2003). "Automatic Sequences." Cambridge.
 *       — the turn string in §5 is the *regular paperfolding sequence*; this
 *       treats it rigorously and gives the fold recursion's properties.
 *   [4] Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *       — the dragon among self-similar, plane-filling, rep-tile curves.
 *
 * Rendering & turtle graphics
 *   [5] Prusinkiewicz, P. & Lindenmayer, A. (1990). "The Algorithmic Beauty of
 *       Plants." Springer (free at algorithmicbotany.org).  — L-systems and the
 *       turtle interpretation of a symbol string: exactly how §4 walks turns
 *       into a path.  The dragon has a short L-system; compare it to §5.
 *   [6] Abelson, H. & diSessa, A. (1980). "Turtle Geometry." MIT Press.  — the
 *       turtle model (position + heading, step + turn) the §4 Turtle embodies.
 *
 * In-repo study companions
 *   [7] l_system.c — draws curves (including the dragon) from a rewrite grammar;
 *       contrast its generic rules with the explicit fold recursion in §5.
 *   [8] koch.c — another turtle-traced fractal curve; same Fit/Reveal rendering
 *       shape, a different turn rule.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_GEN_MIN    4
#define N_GEN_MAX   13
#define N_GEN_INIT  12

/* A generation-n curve has 2ⁿ−1 turns and 2ⁿ points; size arrays for the max. */
#define MAX_TURNS   8191       /* 2^N_GEN_MAX − 1 */
#define MAX_POINTS  8192       /* 2^N_GEN_MAX     */

#define SPEED_MIN      1
#define SPEED_MAX   1024
#define SPEED_DEFAULT  8

#define NS_PER_SEC  1000000000LL
#define RENDER_FPS  30
#define RENDER_NS   (NS_PER_SEC / RENDER_FPS)

/* Terminal cell height / width ≈ 2 — the x scale is doubled by this so a
 * square block of grid reads square on screen. */
#define ASPECT_R   2.0f

/* Rows/cols kept clear of the curve so it never overwrites the HUD. */
enum {
    MARGIN_TOP    = 2,    /* top rows: data lines     */
    MARGIN_BOTTOM = 2,    /* bottom rows: action bar  */
    MARGIN_SIDE   = 1,    /* left/right padding       */
};

/*
 * Palette slots — the colour pairs a segment can be drawn in.  ncurses draws
 * in numbered "pairs" (a foreground/background combo); these names ARE those
 * numbers, so painting a cell is just COLOR_PAIR(CP_Gk).
 *
 * WHY seven colours for the curve: every segment is born in some fold
 * generation (ref [1]), and we tint by that age — oldest folds cool (blue),
 * newest warm (white).  Seven bands give a visible gradient across the folding
 * hierarchy without needing one colour per generation.  seg_color (§6) maps a
 * segment's depth onto CP_G1..CP_G7.
 */
enum {
    CP_G1 = 1,   /* oldest folds — dark blue */
    CP_G2,       /*              — blue       */
    CP_G3,       /*              — cyan       */
    CP_G4,       /*              — green      */
    CP_G5,       /*              — yellow     */
    CP_G6,       /*              — orange     */
    CP_G7,       /* newest folds — white      */
    CP_HUD,      /* HUD top data lines    — yellow */
    CP_HINT,     /* HUD bottom action bar — cyan   */
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/*
 * frame_pace — sleep whatever is left of this frame's RENDER_NS budget, so the
 * loop holds a steady ~RENDER_FPS no matter how long the work took.
 */
static void frame_pace(long long frame_start)
{
    clock_sleep_ns(RENDER_NS - (clock_ns() - frame_start));
}

/* ===================================================================== */
/* §3  geometry — the integer grid the curve is drawn on                  */
/* ===================================================================== */

/*
 * Point — one integer cell, the atom of everything spatial here.
 *
 * WHY integer (not float): the dragon curve lives on a square lattice — every
 * vertex sits exactly on a grid intersection and every move is exactly one
 * cell.  Integers model that with no rounding drift.
 *
 * WHY one type for two jobs: a Point is a vertex of the curve (abstract grid
 * units) AND, once Fit (§7) is applied, a cell on the terminal.  Sharing the
 * type keeps the grid→screen mapping a one-liner.
 */
typedef struct {
    int x;   /* column — grows rightward                      */
    int y;   /* row    — grows DOWNWARD (screen convention)   */
} Point;

/*
 * Heading — the four directions the turtle (§4) can face.
 *
 * WHY this exact order (E,S,W,N): it makes turning pure arithmetic.  Clockwise
 * E→S→W→N→E is +1 each step, so a RIGHT turn is heading+1 and a LEFT turn is
 * heading−1, both mod 4 — no lookup tables, no special cases.  (The
 * turtle-graphics model: refs [5][6].)
 */
typedef enum { EAST, SOUTH, WEST, NORTH } Heading;

/*
 * HEADING_STEP[h] — the (dx, dy) added to a Point to move one cell while facing
 * h.  Indexed by Heading so turtle_step is just "pos += HEADING_STEP[heading]".
 */
static const Point HEADING_STEP[4] = {
    [EAST]  = {  1,  0 },   /* →                     */
    [SOUTH] = {  0,  1 },   /* ↓  (y grows downward) */
    [WEST]  = { -1,  0 },   /* ←                     */
    [NORTH] = {  0, -1 },   /* ↑                     */
};

/*
 * BBox — the smallest axis-aligned rectangle containing every path point, in
 * grid units.
 *
 * WHY we need it: the curve's raw grid size grows with the generation, but the
 * terminal stays fixed.  Knowing the exact extent lets Fit (§7) scale the whole
 * shape to fill the screen and stay centred, instead of guessing.
 *
 * Built in one pass while tracing: bbox_reset to the first point, then
 * bbox_include for each new one — the standard running min/max.
 */
typedef struct {
    int x_min, x_max;   /* leftmost / rightmost column the path reaches */
    int y_min, y_max;   /* topmost  / bottommost row the path reaches   */
} BBox;

static void bbox_reset(BBox *b, Point p)
{
    b->x_min = b->x_max = p.x;
    b->y_min = b->y_max = p.y;
}

static void bbox_include(BBox *b, Point p)
{
    if (p.x < b->x_min) b->x_min = p.x;
    if (p.x > b->x_max) b->x_max = p.x;
    if (p.y < b->y_min) b->y_min = p.y;
    if (p.y > b->y_max) b->y_max = p.y;
}

static int bbox_width (const BBox *b) { return b->x_max - b->x_min + 1; }
static int bbox_height(const BBox *b) { return b->y_max - b->y_min + 1; }

/* ===================================================================== */
/* §4  turtle — the alphabet, and the pen that walks it                   */
/* ===================================================================== */

/*
 * Turn — the two-letter alphabet a dragon curve is written in: at each step the
 * turtle goes LEFT or RIGHT.  The entire fractal is one long string of these.
 *
 * WHY the values are 0 and 1 specifically: the fold recursion (§5) needs the
 * "complement" of a turn (L↔R).  With TURN_LEFT = 0 and TURN_RIGHT = 1 the
 * complement is a simple flip, which turn_flip names.  (The string is the
 * regular paperfolding sequence — ref [3].)
 */
typedef enum { TURN_LEFT, TURN_RIGHT } Turn;

static Turn turn_flip(Turn t) { return t == TURN_RIGHT ? TURN_LEFT : TURN_RIGHT; }

/*
 * Turtle — an imaginary pen sitting at a grid Point and facing a Heading.
 *
 * WHY it exists: the turn sequence (§5) is abstract — just L/R letters.  A
 * turtle is what turns that string into an actual shape: read a letter, turn,
 * step, leaving a trail of Points behind.  This "turtle graphics" idea
 * (refs [5][6]) is the bridge from the symbolic recipe to drawable geometry.
 */
typedef struct {
    Point   pos;       /* where the pen currently sits, in grid cells */
    Heading heading;   /* the direction it will step next             */
} Turtle;

/* Advance one cell in the current heading; return the new position. */
static Point turtle_step(Turtle *t)
{
    t->pos.x += HEADING_STEP[t->heading].x;
    t->pos.y += HEADING_STEP[t->heading].y;
    return t->pos;
}

/* Rotate in place: right = +1 quarter-turn, left = −1 (i.e. +3), mod 4. */
static void turtle_turn(Turtle *t, Turn turn)
{
    int delta = (turn == TURN_RIGHT) ? 1 : 3;
    t->heading = (Heading)((t->heading + delta) % 4);
}

/* ===================================================================== */
/* §5  curve — the DragonCurve itself                                     */
/* ===================================================================== */

/*
 * DragonCurve — the fractal itself, the central data structure of the program.
 * dragon_build fills it in two phases (refs [1][3]):
 *   phase 1  fold the L/R turn sequence — the *recipe*
 *   phase 2  walk a turtle along it to trace the path — the *shape*
 *
 * WHY bundle recipe + shape + bounds together: drawing needs the path, colour
 * needs the turn count, scaling needs the bounds — all derived from one
 * generation number.  Keeping them in one struct keeps them consistent and
 * lets a single pointer carry the whole fractal.
 *
 * Sizes at fold depth `gen`:  n_turns = 2^gen − 1,  n_points = 2^gen,  and the
 * number of drawable segments equals n_turns (one segment follows each turn).
 */
typedef struct {
    int   gen;                   /* fold depth, N_GEN_MIN..N_GEN_MAX          */
    int   n_turns;               /* length of turns[]  = segment count        */
    Turn  turns[MAX_TURNS];      /* the L/R recipe (phase 1)                  */
    int   n_points;              /* length of path[]   = n_turns + 1          */
    Point path[MAX_POINTS];      /* traced vertices, grid units (phase 2)     */
    BBox  bounds;                /* extent of path[], for screen scaling      */
} DragonCurve;

/*
 * Phase 1 — fold the turn sequence.
 *   T_n = T_{n-1}  ·  R  ·  reverse-complement(T_{n-1})
 * Start from a single right turn; each generation copies the first half, drops
 * a right-turn crease in the middle, then mirrors-and-flips the first half.
 */
static void dragon_build_turns(DragonCurve *d, int gen)
{
    d->turns[0] = TURN_RIGHT;
    d->n_turns  = 1;

    for (int g = 2; g <= gen; g++) {
        int half = d->n_turns;
        d->turns[half] = TURN_RIGHT;                          /* the fold crease */
        for (int i = 0; i < half; i++)                        /* reverse-complement */
            d->turns[half + 1 + i] = turn_flip(d->turns[half - 1 - i]);
        d->n_turns = half * 2 + 1;
    }
}

/*
 * Phase 2 — trace the path.
 * Drop a turtle at the origin facing EAST, then for each turn: step forward
 * (recording the new point and growing the bounds), then rotate for the next
 * segment.  n_points ends at n_turns + 1.
 */
static void dragon_trace_path(DragonCurve *d)
{
    Turtle t = { .pos = { 0, 0 }, .heading = EAST };

    d->path[0] = t.pos;
    bbox_reset(&d->bounds, t.pos);

    for (int i = 0; i < d->n_turns; i++) {
        d->path[i + 1] = turtle_step(&t);
        bbox_include(&d->bounds, d->path[i + 1]);
        turtle_turn(&t, d->turns[i]);
    }
    d->n_points = d->n_turns + 1;
}

/* Build the whole curve at a fold depth, clamped to the supported range. */
static void dragon_build(DragonCurve *d, int gen)
{
    if (gen < N_GEN_MIN) gen = N_GEN_MIN;
    if (gen > N_GEN_MAX) gen = N_GEN_MAX;
    d->gen = gen;
    dragon_build_turns(d, gen);
    dragon_trace_path(d);
}

/* ===================================================================== */
/* §6  color — generation depth → colour band                            */
/* ===================================================================== */

/*
 * seg_generation — which fold generation first created segment `seg`.
 * Segment 0 is gen 1; segs 1-2 are gen 2; segs 3-6 are gen 3; …  The boundary
 * doubles each time, so the generation is floor(log2(seg+1)) + 1.
 */
static int seg_generation(int seg)
{
    int g = 1, n = seg + 1;
    while (n > 1) { n >>= 1; g++; }
    return g;
}

/*
 * seg_color — spread a segment's generation depth across the 7 colour pairs,
 * so the oldest folds read cool (blue) and the newest read warm (white).
 */
static int seg_color(int seg, int gen)
{
    int depth = seg_generation(seg);              /* 1..gen — how deep this fold is */
    int steps = CP_G7 - CP_G1;                    /* colour-gradient resolution (6) */
    int span  = gen > 1 ? gen - 1 : 1;            /* fold depths to spread across   */
    int pair  = CP_G1 + (depth - 1) * steps / span;
    if (pair < CP_G1) pair = CP_G1;
    if (pair > CP_G7) pair = CP_G7;
    return pair;
}

/* ===================================================================== */
/* §7  fit — map grid points onto terminal cells                          */
/* ===================================================================== */

/*
 * Fit — the lens between grid units and screen cells: a per-axis scale plus an
 * offset.  fit_compute rebuilds it each frame from the curve's bounds and the
 * drawable area, so the picture stays centred and as large as fits at any
 * generation or window size; fit_apply maps a single grid Point through it.
 *
 * WHY two different scales: terminal character cells are about twice as tall as
 * wide, so a 1:1 mapping would squash the curve vertically.  scale_x is
 * ASPECT_R × scale_y to cancel that out and keep the shape true.
 */
typedef struct {
    float scale_x, scale_y;   /* grid units → cells, per axis (x is stretched)     */
    int   off_x, off_y;       /* top-left cell of the curve (centres it on screen) */
} Fit;

static Fit fit_compute(const BBox *bounds, int cols, int rows)
{
    /* 1. drawable area = the screen minus the HUD margins */
    int draw_cols = cols - 2 * MARGIN_SIDE;
    int draw_rows = rows - (MARGIN_TOP + MARGIN_BOTTOM);
    if (draw_cols < 1) draw_cols = 1;
    if (draw_rows < 1) draw_rows = 1;

    int grid_w = bbox_width(bounds);
    int grid_h = bbox_height(bounds);

    /* 2. largest vertical scale that still fits both axes.  Each grid unit is
     *    drawn ASPECT_R cells wide, so the width limit is tested against the
     *    stretched scale; keep whichever axis runs out of room first. */
    float fit_by_rows = (float)draw_rows / (float)grid_h;
    float fit_by_cols = (float)draw_cols / (float)grid_w;
    float unit = fit_by_rows;
    if (unit * ASPECT_R > fit_by_cols)            /* too wide → clamp to columns */
        unit = fit_by_cols / ASPECT_R;

    /* 3. centre the scaled curve inside the drawable area */
    Fit f;
    f.scale_y = unit;
    f.scale_x = unit * ASPECT_R;
    f.off_x = MARGIN_SIDE + (draw_cols - (int)(grid_w * f.scale_x)) / 2;
    f.off_y = MARGIN_TOP  + (draw_rows - (int)(grid_h * f.scale_y)) / 2;
    return f;
}

/* Map one grid point to its screen cell under this fit. */
static Point fit_apply(const Fit *f, const BBox *bounds, Point grid)
{
    return (Point){
        .x = (int)((grid.x - bounds->x_min) * f->scale_x) + f->off_x,
        .y = (int)((grid.y - bounds->y_min) * f->scale_y) + f->off_y,
    };
}

/* ===================================================================== */
/* §8  reveal — the segment-by-segment animation                          */
/* ===================================================================== */

/*
 * Reveal — the segment-by-segment animation state, kept SEPARATE from the curve
 * so restarting or changing speed never touches the (expensive) geometry.
 *
 * `drawn` climbs by `speed` each tick until it reaches the segment count, then
 * the picture holds (the curve does not loop).  Restart ('r') just sets drawn
 * back to 0 and reuses the same curve.
 */
typedef struct {
    int  drawn;    /* segments shown so far (0 .. curve.n_turns)      */
    int  speed;    /* segments added per tick (SPEED_MIN .. SPEED_MAX)*/
    bool paused;   /* true = freeze the reveal                        */
} Reveal;

static void reveal_reset (Reveal *r)            { r->drawn = 0; }
static void reveal_faster(Reveal *r)            { r->speed *= 2; if (r->speed > SPEED_MAX) r->speed = SPEED_MAX; }
static void reveal_slower(Reveal *r)            { r->speed /= 2; if (r->speed < SPEED_MIN) r->speed = SPEED_MIN; }

static void reveal_tick(Reveal *r, int total)
{
    if (r->paused) return;
    r->drawn += r->speed;
    if (r->drawn > total) r->drawn = total;
}

/* ===================================================================== */
/* §9  scene — the curve plus its animation = the running picture         */
/* ===================================================================== */

/*
 * Scene — the whole "what is on screen": the fractal plus how much of it is
 * currently revealed.  Everything the program does reduces to updating a Scene
 * (scene_tick) and drawing it (scene_draw).
 *
 * WHY just these two: the curve is the *what* and the reveal is the *how much* —
 * together they fully describe a frame.  Terminal size lives in Screen instead,
 * because it describes the device, not the picture.
 */
typedef struct {
    DragonCurve curve;    /* the fractal being shown          */
    Reveal      reveal;   /* how far the animation has gotten */
} Scene;

/* Rebuild the curve at a generation and restart the reveal from empty. */
static void scene_load(Scene *s, int gen)
{
    dragon_build(&s->curve, gen);
    reveal_reset(&s->reveal);
}

static void scene_init(Scene *s, int gen, int speed)
{
    s->reveal.speed  = speed;
    s->reveal.paused = false;
    scene_load(s, gen);
}

static void scene_tick(Scene *s)
{
    reveal_tick(&s->reveal, s->curve.n_turns);
}

/* scene_shown — how many segments are visible right now: the reveal's progress
 * clamped to the curve's segment count.  Used by both the renderer and the HUD. */
static int scene_shown(const Scene *s)
{
    int total = s->curve.n_turns;
    return s->reveal.drawn < total ? s->reveal.drawn : total;
}

/* plot_point — draw one grid point as a glyph, clipped to the drawable area
 * (keeps the curve out of the HUD rows). */
static void plot_point(Point cell, int color, int cols, int rows)
{
    if (cell.y >= MARGIN_TOP && cell.y < rows - 1 && cell.x >= 0 && cell.x < cols) {
        attron(COLOR_PAIR(color) | A_BOLD);
        mvaddch(cell.y, cell.x, (chtype)'#');
        attroff(COLOR_PAIR(color) | A_BOLD);
    }
}

/*
 * scene_draw — paint the revealed portion of the curve.
 * Build the Fit for the current bounds + window, then for each shown segment
 * map and plot both of its endpoints (consecutive segments share endpoints,
 * so plotting endpoints fills the whole path).
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const DragonCurve *d = &s->curve;
    int shown = scene_shown(s);

    Fit fit = fit_compute(&d->bounds, cols, rows);

    for (int i = 0; i < shown; i++) {
        int color = seg_color(i, d->gen);
        plot_point(fit_apply(&fit, &d->bounds, d->path[i]),     color, cols, rows);
        plot_point(fit_apply(&fit, &d->bounds, d->path[i + 1]), color, cols, rows);
    }
}

/* ===================================================================== */
/* §10  screen — ncurses lifecycle + HUD                                  */
/* ===================================================================== */

/*
 * Screen — the live terminal size in character cells.  Measured at startup and
 * after every resize (SIGWINCH); Fit (§7) reads it to scale the curve and the
 * HUD reads it to place its lines.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
} Screen;

/*
 * bind_palette — wire the 7 generation colours + the 2 HUD colours to ncurses
 * pairs (foreground on the terminal's default background).  Called with either
 * the 256-colour or the 8-colour list, so the repetitive init_pair wiring lives
 * in exactly one place.
 */
static void bind_palette(const int gen_fg[7], int hud_fg, int hint_fg)
{
    for (int i = 0; i < 7; i++)
        init_pair(CP_G1 + i, gen_fg[i], -1);     /* CP_G1..CP_G7, oldest → newest */
    init_pair(CP_HUD,  hud_fg,  -1);
    init_pair(CP_HINT, hint_fg, -1);
}

static void color_init(void)
{
    static const int gen_256[7] = {  26,  27,  51,  46, 226, 208, 231 };
    static const int gen_8[7]   = { COLOR_BLUE,   COLOR_BLUE,  COLOR_CYAN, COLOR_GREEN,
                                    COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE };
    start_color();
    use_default_colors();

    bool truecolor = COLORS >= 256;
    bind_palette(truecolor ? gen_256 : gen_8,
                 truecolor ? 226 : COLOR_YELLOW,
                 truecolor ?  51 : COLOR_CYAN);
}

static void screen_init(Screen *sc)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/*
 * gen_name — a dragon-growth label for each fold depth, shown in place of a
 * bare generation number.  The curve grows from a handful of segments
 * ("hatchling", gen 4) to the full 8192-point "dragon" (gen 13), so the names
 * double as a sense of how grown the creature is.
 */
static const char *gen_name(int gen)
{
    static const char *k_names[N_GEN_MAX - N_GEN_MIN + 1] = {
        "hatchling",  /* gen 4  */
        "wyrmling",   /* gen 5  */
        "fledgling",  /* gen 6  */
        "juvenile",   /* gen 7  */
        "serpent",    /* gen 8  */
        "wyrm",       /* gen 9  */
        "drake",      /* gen 10 */
        "wyvern",     /* gen 11 */
        "elder",      /* gen 12 */
        "dragon",     /* gen 13 */
    };
    int idx = gen - N_GEN_MIN;
    if (idx < 0) idx = 0;
    if (idx > N_GEN_MAX - N_GEN_MIN) idx = N_GEN_MAX - N_GEN_MIN;
    return k_names[idx];
}

/*
 * hud_draw — top is data, bottom is actions:
 *   row 0      (bold, right)  name + stage position + run state
 *   row 1      (plain, left)  segments revealed / total, speed
 *   row rows-1 (cyan, left)   every interactive key
 */
/* hud_data_line — row 0, right-aligned bold: name + stage position + run state. */
static void hud_data_line(const Screen *sc, const Scene *s)
{
    const DragonCurve *d = &s->curve;
    int shown = scene_shown(s);
    const char *state = s->reveal.paused ? "PAUSED "
                      : (shown >= d->n_turns ? "complete" : "drawing ");
    int stage  = d->gen - N_GEN_MIN + 1;          /* 1-based position in the range */
    int stages = N_GEN_MAX - N_GEN_MIN + 1;       /* how many named stages exist   */

    char buf[80];
    snprintf(buf, sizeof buf, " DragonCurve  %s %d/%d  %s ",
             gen_name(d->gen), stage, stages, state);
    int right_x = sc->cols - (int)strlen(buf);
    if (right_x < 0) right_x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, right_x, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* hud_params_line — row 1, plain: segments revealed / total, and speed. */
static void hud_params_line(const Scene *s)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(1, 0, " segs:%d/%d  speed:%d ",
             scene_shown(s), s->curve.n_turns, s->reveal.speed);
    attroff(COLOR_PAIR(CP_HUD));
}

/* hud_action_line — last row, bright cyan: every interactive key. */
static void hud_action_line(const Screen *sc)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
        " q:quit  spc:pause  r:restart  +/-:speed  n/p:age ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* hud_draw — top is data, bottom is actions (see the three helpers). */
static void hud_draw(const Screen *sc, const Scene *s)
{
    hud_data_line(sc, s);     /* row 0    — name + stage + state */
    hud_params_line(s);       /* row 1    — segments + speed     */
    hud_action_line(sc);      /* last row — key actions          */
}

static void screen_draw(const Screen *sc, const Scene *s)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);
    hud_draw(sc, s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §11  app — signals, input, main loop                                   */
/* ===================================================================== */

/*
 * App — the top-level container main() owns: the picture (scene) and the
 * terminal it lives in (screen).  Kept as one struct so the loop passes
 * "everything" with a single &app, and so it can sit in BSS — it holds the
 * large curve arrays, which don't belong on the stack.
 */
typedef struct {
    Scene  scene;    /* the fractal + its animation */
    Screen screen;   /* current terminal dimensions */
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
    g_should_resize = 0;
    screen_resize(&a->screen);
}

/* Map one keypress to an action. */
static void app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:  g_should_quit = 1;                       break;
    case ' ':            a->scene.reveal.paused = !a->scene.reveal.paused; break;
    case 'r': case 'R':  reveal_reset(&a->scene.reveal);                   break;
    case '+': case '=':  reveal_faster(&a->scene.reveal);                  break;
    case '-':            reveal_slower(&a->scene.reveal);                  break;
    case 'n': case 'N':  scene_load(&a->scene, a->scene.curve.gen + 1);    break;  /* grow older */
    case 'p': case 'P':  scene_load(&a->scene, a->scene.curve.gen - 1);    break;  /* grow younger */
    default: break;
    }
}

static void app_poll_input(App *a)
{
    int ch = getch();
    if (ch != ERR) app_handle_key(a, ch);
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    /* static (BSS) — Scene holds large arrays; keep them off the stack */
    static App app;

    screen_init(&app.screen);
    scene_init(&app.scene, N_GEN_INIT, SPEED_DEFAULT);

    /* Main loop — one pass per frame:
     *   1. apply a pending resize
     *   2. handle one keypress
     *   3. grow the reveal by one tick
     *   4. draw the fractal + HUD
     *   5. pace the frame to ~RENDER_FPS
     */
    while (!g_should_quit) {

        if (g_should_resize)
            app_resize(&app);

        long long frame_start = clock_ns();

        app_poll_input(&app);                   /* one key → action */
        scene_tick(&app.scene);                 /* grow the reveal  */
        screen_draw(&app.screen, &app.scene);   /* fractal + HUD    */
        screen_present();

        frame_pace(frame_start);                /* hold ~RENDER_FPS */
    }
    return 0;
}
