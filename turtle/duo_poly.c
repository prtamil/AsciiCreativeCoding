/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * duo_poly.c — Dual Turtle Polygon Animator
 *
 * Two turtles draw regular polygons step-by-step in the terminal:
 *   Turtle A (cyan)    — left half,  starts as a triangle (3 sides)
 *   Turtle B (magenta) — right half, starts as a pentagon (5 sides)
 *
 * Each sim tick advances one polygon edge per turtle — you watch the pen
 * move vertex to vertex.  After both polygons complete, a 2-second pause
 * fires and both auto-reset, each gaining one side (3 → 4 → … → 12 → 3).
 *
 * The turtle head (@) shows the current pen position.  Line characters
 * (-  |  /  \) reflect each edge's true heading for a natural look.
 *
 * Framework: follows framework.c §1–§8 structure exactly.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  coords    — aspect-ratio correction (CELL_W/CELL_H ≈ 0.5)
 *   §5  entity    — Polygon + Pen composed into Turtle; tick + draw
 *   §6  scene     — turtle pair tick / draw helpers
 *   §7  screen    — Screen + FrameTimer + Scene container; HUD bars
 *   §8  app       — signals, key handling, main loop
 * ─────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q / ESC     quit
 *   space       pause / resume
 *   r           reset both turtles (keep current sides)
 *   a / z       turtle A: +1 / -1 sides  (3–12)
 *   s / x       turtle B: +1 / -1 sides  (3–12)
 *   + / =       draw faster
 *   -           draw slower
 *   ] / [       sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra duo_poly.c -o duo_poly -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Regular polygon via turtle graphics.  A regular n-gon
 *                  inscribed in a circle of radius R has vertices at angles
 *                  θ_k = θ_0 + k·(2π/n).  The turtle starts at vertex 0
 *                  and walks to vertex 1, 2, … n (= vertex 0) in sequence.
 *                  Each edge is rasterised with a DDA line fill so no cell
 *                  gaps appear at any step size.
 *
 * Aspect fix     : Terminal cells are ~2× taller than wide (CELL_H/CELL_W).
 *                  The polygon Y coordinates are scaled by ASPECT=0.5 so the
 *                  shape is visually circular/square rather than vertically
 *                  compressed.  Both turtles use the same correction.
 *
 * Timing         : Fixed-step accumulator (framework.c §8).  Each tick
 *                  decrements a per-turtle edge timer; when it expires the
 *                  edge index advances and the timer resets.  Speed is
 *                  controlled by edges_per_second (eps), adjustable via +/-.
 *
 * References     : grouped by the concepts this file teaches. Seven
 *                  entries — one or two strong picks per topic.
 *
 *   ── Turtle graphics (the abstraction this file animates) ─────────
 *   Papert, "Mindstorms: Children, Computers, and Powerful Ideas"
 *     (Basic Books, 1980). The original LOGO turtle and the
 *     foundational text on the pen-with-heading abstraction this
 *     file builds on.
 *   Abelson & diSessa, "Turtle Geometry: The Computer as a Medium for
 *     Exploring Mathematics" (MIT Press, 1981). The canonical book on
 *     turtle-graphics MATH — covers regular polygons, curves,
 *     differential geometry by turtle. Chapter 1's "POLY" procedure
 *     is essentially what duo_poly.c animates.
 *
 *   ── Regular polygons (the shapes the turtles draw) ───────────────
 *   Coxeter, "Regular Polytopes" (Dover, 3rd ed., 1973). The
 *     canonical reference on regular polygons and their
 *     higher-dimensional cousins. Chapter 1 covers planar regular
 *     polygons — the 2π/n exterior-angle derivation that drives the
 *     turtle's "walk, turn 2π/n, repeat" recipe.
 *
 *   ── Line rasterisation (§5 put_seg) ──────────────────────────────
 *   Newman & Sproull, "Principles of Interactive Computer Graphics"
 *     (McGraw-Hill, 2nd ed., 1979) ch. 2. The textbook treatment of
 *     DDA (Digital Differential Analyzer) line drawing — exactly
 *     what put_seg() implements (step along major axis, interpolate
 *     the minor).
 *   Bresenham, "Algorithm for computer control of a digital plotter"
 *     (IBM Systems Journal 4(1):25–30, 1965). The integer-only
 *     alternative to DDA. This file uses DDA for simplicity;
 *     Bresenham is the standard comparison.
 *
 *   ── Rendering (§3, §6, §7) ───────────────────────────────────────
 *   Padala, "NCURSES Programming HOWTO" (The Linux Documentation
 *     Project). Practical reference for the ncurses API used in
 *     §3 (color_init) and §7 (mvwaddch, attron, A_BOLD).
 *     https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *   ── Game loop & numerical timing (§8 main loop) ──────────────────
 *   Fiedler ("Gaffer on Games"), "Fix Your Timestep!" (2004). The
 *     fixed-timestep accumulator pattern in main() — explains why a
 *     separate sim_fps from render fps keeps the turtle's drawing
 *     pace consistent regardless of frame rate.
 *     https://gafferongames.com/post/fix_your_timestep/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A turtle is a PEN with a POSITION and a HEADING.  It can do two
 * things: walk forward by some distance (drawing as it goes), or
 * turn left/right by some angle.  Regular polygons emerge from a
 * single repeating recipe: "walk one edge, turn by 2π/n, repeat n
 * times."  The turtle never needs to know it's drawing a polygon
 * — the geometry is in the angle.
 *
 *
 * ALGORITHM IN STEPS  (per tick, per turtle)
 * ─────────────────────────────────────────
 *   1. If turtle is DONE, increment "done timer".  After RESET_DELAY
 *      seconds, reset (clear edges drawn, advance side count by ±1).
 *   2. Else: decrement edge_timer by dt.
 *   3. If edge_timer ≤ 0:
 *        a. Drawing the edge = DDA line walk from vertex[k] to
 *           vertex[k+1], stamping a line glyph (- | / \) at each cell.
 *        b. edge_index k += 1.
 *        c. If k == n: turtle DONE.
 *        d. edge_timer reset to (1 / eps).
 *
 * KEY FORMULAS
 * ────────────
 *   Vertex k of regular n-gon inscribed in radius R, centred at (cx, cy):
 *     θ_k = θ_0 + k · (2π / n)
 *     v.x = cx + R · cos(θ_k)
 *     v.y = cy + R · sin(θ_k) · ASPECT          (terminal cells 2:1 tall)
 *
 *   Exterior angle (the turn the turtle makes between edges):
 *     α = 2π / n
 *     n = 3  → α = 120°
 *     n = 4  → α = 90°
 *     n = 6  → α = 60°
 *     n = 12 → α = 30°
 *
 *   DDA line walk from (x1, y1) to (x2, y2):
 *     n_steps = max(|dx|, |dy|)
 *     for i in 0..n_steps:
 *       col = x1 + i·dx/n_steps
 *       row = y1 + i·dy/n_steps
 *
 *
 * Background you need
 * ───────────────────
 *   - Trig: (cos θ, sin θ) is a unit vector at angle θ from +X.
 *   - Fixed-step accumulator (ticks at SIM_FPS regardless of
 *     render rate).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - LOGO / Scratch turtle commands.  We use the IDEA of a turtle
 *     (pen + heading) but skip the language.
 *   - Bezier / spline curves.  Regular polygons only.
 *   - Subpixel anti-aliasing.  Cell-grid output, no smoothing.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =   5,
    SIM_FPS_DEFAULT =  30,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =   5,

    N_COLORS        =   7,
    SIDES_MIN       =   3,
    SIDES_MAX       =  12,

    HUD_COLS        =  64,
    FPS_UPDATE_MS   = 500,
};

#define EPS_DEFAULT   1.5f   /* edges per second (default drawing speed) */
#define EPS_MIN       0.3f
#define EPS_MAX      12.0f
#define EPS_SCALE_FACTOR 1.5f /* + / - keys scale eps geometrically by this */
#define RESET_DELAY   2.0f   /* seconds to wait after both polygons done */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §1.1 layout fractions (turtle placement on screen) ────────── */

/* The two halves sit at cols * X_FRAC. 0.25 puts the left turtle in
 * the centre of the left half; 0.75 the right half. */
#define HALF_LEFT_X_FRAC   0.25f
#define HALF_RIGHT_X_FRAC  0.75f

/* Polygon radius is the smaller of two bounds (per half-screen):
 *      x-bound: cols  * POLY_MAX_R_X_FRAC      → margin from divider
 *      y-bound: rows  * POLY_MAX_R_Y_FRAC / ASPECT
 *                                              → cell-aspect corrected
 * Then scaled by POLY_R_FIT_FRAC for breathing room around the label
 * row and bottom hint strip. */
#define POLY_MAX_R_X_FRAC  0.21f
#define POLY_MAX_R_Y_FRAC  0.40f
#define POLY_R_FIT_FRAC    0.85f

/* ── §1.2 main-loop pacing ─────────────────────────────────────── */

#define RENDER_FPS         60
#define RENDER_FRAME_NS    (NS_PER_SEC / RENDER_FPS)

/* Cap dt at 100 ms so a process suspension can't dump a huge catch-
 * up burst of sim ticks (spiral-of-death prevention). */
#define DT_CAP_NS          (100 * NS_PER_MS)

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

/*
 * Color pairs:
 *   1  → cyan      (turtle A edges + label)
 *   2  → magenta   (turtle B edges + label)
 *   3  → yellow    (DONE / completion banner)
 *   4  → green     (polygon names)
 *   5  → red       (reserved, currently unused)
 *   6  → blue      (vertical divider between halves)
 *   7  → white     (turtle @ head)
 *
 * HUD chrome (canonical CLAUDE.md two-bar spec — on DEFAULT background
 * so the bars stay legible against any animation, with A_BOLD never A_DIM):
 *   8  → bright yellow on default bg  (top status row 0)
 *   9  → bright cyan   on default bg  (bottom hint row last)
 */
#define PAIR_TURTLE_A      1
#define PAIR_TURTLE_B      2
#define PAIR_DONE          3
#define PAIR_POLY_NAME     4
#define PAIR_FLASH         5
#define PAIR_DIVIDER       6
#define PAIR_HEAD          7
#define PAIR_HUD           8
#define PAIR_HINT          9

/*
 * Palette table — each row is one role mapped to its 256-mode and
 * 8-mode foreground codes plus its background. HUD chrome (PAIR_HUD /
 * PAIR_HINT) explicitly uses default background per CLAUDE.md HUD
 * spec; everything else uses COLOR_BLACK for the on-canvas roles.
 *
 * Reading this table top-to-bottom gives the full palette in one
 * glance — no parallel 256/8-mode branches to keep mentally in sync.
 */
typedef struct {
    short pair;     /* PAIR_* id from §3                              */
    short fg256;    /* foreground when COLORS >= 256                  */
    short fg8;      /* foreground in 8-colour fallback                */
    short bg;       /* background (COLOR_BLACK for canvas, -1 for HUD)*/
} PaletteEntry;

static const PaletteEntry PALETTE[] = {
    /* pair             256   8-fallback        bg          role     */
    { PAIR_TURTLE_A,     51,  COLOR_CYAN,    COLOR_BLACK }, /* cyan        */
    { PAIR_TURTLE_B,    201,  COLOR_MAGENTA, COLOR_BLACK }, /* magenta     */
    { PAIR_DONE,        226,  COLOR_YELLOW,  COLOR_BLACK }, /* yellow      */
    { PAIR_POLY_NAME,    46,  COLOR_GREEN,   COLOR_BLACK }, /* green       */
    { PAIR_FLASH,       196,  COLOR_RED,     COLOR_BLACK }, /* red         */
    { PAIR_DIVIDER,      33,  COLOR_BLUE,    COLOR_BLACK }, /* blue        */
    { PAIR_HEAD,        255,  COLOR_WHITE,   COLOR_BLACK }, /* white       */
    { PAIR_HUD,         226,  COLOR_YELLOW,  -1          }, /* HUD chrome  */
    { PAIR_HINT,         51,  COLOR_CYAN,    -1          }, /* HINT chrome */
};
#define PALETTE_LEN  (int)(sizeof PALETTE / sizeof PALETTE[0])

static void color_init(void)
{
    start_color();
    use_default_colors();
    bool truecolor = (COLORS >= 256);
    for (int i = 0; i < PALETTE_LEN; i++) {
        short fg = truecolor ? PALETTE[i].fg256 : PALETTE[i].fg8;
        init_pair(PALETTE[i].pair, fg, PALETTE[i].bg);
    }
}

/* ===================================================================== */
/* §4  coords — aspect-ratio correction for isotropic turtle motion       */
/* ===================================================================== */

/*
 * Terminal cells are ~2× taller than wide.
 * We multiply turtle Y positions by ASPECT = CELL_W/CELL_H ≈ 0.5 so that
 * equal column and row distances represent equal physical pixel distances.
 * A polygon drawn this way looks round/square, not vertically stretched.
 */
#define CELL_W   8
#define CELL_H  16
#define ASPECT   ((float)CELL_W / (float)CELL_H)   /* 0.5 */

/*
 * angle_char() — ASCII line character that best represents edge direction.
 *   '-'  for near-horizontal
 *   '|'  for near-vertical
 *   '/'  for up-right / down-left diagonal
 *   '\'  for down-right / up-left diagonal
 */
static char angle_char(float angle)
{
    float a = fmodf(angle, (float)M_PI);
    if (a < 0.0f) a += (float)M_PI;
    if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f) return '-';
    if (a < 3.0f * (float)M_PI / 8.0f)                             return '/';
    if (a < 5.0f * (float)M_PI / 8.0f)                             return '|';
    return '\\';
}

/* ===================================================================== */
/* §5  entity — Turtle                                                    */
/* ===================================================================== */

/*
 * §5.1 Polygon — the shape this turtle is drawing.
 *
 * INTENT
 *   The "what" of the turtle — its target shape. Vertex k is at:
 *
 *      θ_k  = start_angle + k · (2π/sides)
 *      v.x  = cx + radius · cos(θ_k)
 *      v.y  = cy + radius · sin(θ_k) · ASPECT     (ASPECT = 0.5)
 *
 *   Set once at turtle_init from terminal dimensions; immutable for
 *   the polygon's lifetime. The animation reads from here; nothing
 *   writes to it during a polygon's run.
 *
 * INVARIANT
 *   The struct is READ-ONLY between resets. turtle_init re-seeds it
 *   on resize, full reset ('r'), per-turtle side change (a/z, s/x),
 *   and at end-of-polygon auto-cycle (sides += 1, wrap at SIDES_MAX).
 *
 * WHY SEPARATE FROM Pen
 *   "What is being drawn" (Polygon, immutable) vs "how far we've
 *   drawn" (Pen, written each tick) are different concerns. Keeping
 *   them in separate sub-structs documents at the TYPE LEVEL which
 *   fields are the SHAPE definition and which are the PROGRESS
 *   tracker — a learner studying turtle_init / turtle_tick can
 *   trust the field names to tell them which group a field belongs
 *   to.
 *
 * WHY THE 2:1 ASPECT CORRECTION LIVES IN poly_vertex (not here)
 *   `radius` is stored in COLUMN units. Y at the use site multiplies
 *   sin(θ) by ASPECT = 0.5 so the visual circle stays round even
 *   though terminal cells are ~2× taller than wide. Storing the
 *   uncorrected radius keeps the trig math clean and lets the
 *   correction happen once, at draw time.
 *
 * ALGORITHM REFERENCES
 *   Abelson & diSessa (1981), "Turtle Geometry" §1.3 — the POLY
 *     procedure: "REPEAT n [FORWARD R; LEFT 2π/n]". This struct is
 *     the data form of that recipe.
 *   Coxeter (1973), "Regular Polytopes" §1.1 — circumradius +
 *     equal-angle definition of regular polygons.
 */
typedef struct {
    float cx;            /* polygon centre column (cell-x).            *
                          * Set by turtle_init from cols * 0.25 or    *
                          * cols * 0.75 (left/right half).             */
    float cy;            /* polygon centre row (cell-y).               *
                          * Approximately rows/2.                      */
    float radius;        /* circumradius in COLUMN units. Y axis      *
                          * multiplies sin(θ) by ASPECT at use site.  *
                          * Chosen by turtle_init to fit the half-    *
                          * screen with margin (85% of available).    */
    int   sides;         /* number of polygon sides, [SIDES_MIN, MAX]  *
                          * = [3, 12]. Cycles 3→4→…→12→3 on auto-     *
                          * reset; a/z and s/x keys nudge per turtle. */
    float start_angle;   /* first vertex angle (radians, math frame). *
                          * Always -π/2 → the first vertex is at the  *
                          * TOP of the polygon, which reads as the    *
                          * natural "starting point" to the eye.      */
} Polygon;

/*
 * §5.2 Pen — the "what's the pen doing right now?" state.
 *
 * INTENT
 *   Animation progress for one turtle. `edge` advances as each edge
 *   completes; `edge_timer` counts down the seconds until the next
 *   advance fires; `done` flips when the polygon is closed. Reset
 *   to (edge=0, timer=1/eps, done=false) at every launch.
 *
 * WHY SEPARATE FROM Polygon
 *   "What is being drawn" (Polygon, immutable) vs "how far we've
 *   drawn" (Pen, written each tick) split into two sub-structs so
 *   reading `t->poly.X` vs `t->pen.X` reveals at a glance which
 *   group a field belongs to. The animation loop touches Pen
 *   exclusively; turtle_init touches both.
 *
 * STATE MACHINE (per turtle, per generation)
 *
 *      edge = 0, timer = 1/eps, done = false        ← launch
 *      ──tick──►  timer -= dt
 *      ──timer ≤ 0──►  edge += 1,  timer += 1/eps
 *      ──edge == sides──►  done = true, edge clamped to sides
 *
 * WHY THE while LOOP (not single decrement)
 *   turtle_tick uses `while (timer ≤ 0) { ... timer += 1/eps; }` so
 *   a large dt that should cover MULTIPLE edges advances correctly.
 *   Without the while, a 100 ms dt with eps = 12 (≈83 ms/edge)
 *   would lose an edge per tick.
 *
 * WHY edge IS CAPPED AT sides (not sides+1 or unbounded)
 *   The closing edge from vertex (sides-1) → vertex 0 is the LAST
 *   thing drawn. Once edge = sides, the polygon is closed and the
 *   pen is back at vertex 0 — no more edges to draw. Capping at
 *   sides means turtle_draw can render edges [0, edge) cleanly.
 */
typedef struct {
    int   edge;          /* edges drawn so far:                       *
                          *   0       = pen at vertex 0, not started *
                          *   k       = k edges drawn, pen at v_k    *
                          *   sides   = polygon complete (capped)    */
    float edge_timer;    /* seconds remaining until next edge fires. *
                          * Reset to 1/eps each time it crosses 0.   */
    float eps;           /* per-turtle drawing rate, edges/second.   *
                          * Mirrors Scene.eps (the shared global);   *
                          * could differ in future for per-turtle    *
                          * speed but the current demo keeps them    *
                          * equal.                                    */
    bool  done;          /* true once edge == sides. Skips further  *
                          * tick work; cleared on launch.            */
} Pen;

/*
 * §5.3 Turtle — Polygon + Pen + colour.
 *
 * INTENT
 *   One turtle = one pen-with-heading drawing one polygon. Composes
 *   the three concept-sized pieces so each access self-documents
 *   which layer it touches:
 *
 *      t->poly.cx          shape definition (immutable this run)
 *      t->pen.edge         animation progress (changes each tick)
 *      t->cpair            ncurses colour pair (one per turtle)
 *
 * LAYERING (read in this order to understand a turtle)
 *
 *      Polygon  — what shape is the turtle going to draw?
 *      Pen      — where is the pen right now within that polygon?
 *      cpair    — what colour are we drawing the result?
 *
 * WHY cpair IS A TOP-LEVEL FIELD (not in Polygon or Pen)
 *   cpair belongs to neither the shape ("what is drawn") nor the
 *   animation ("how far"). It's purely a RENDERING choice — which
 *   ncurses palette entry to use — so it sits at the Turtle level
 *   where the renderer reads it.
 *
 * SIZE
 *   Polygon: 5 floats + 1 int                ≈ 24 B
 *   Pen:     3 floats + 1 int + 1 bool      ≈ 20 B
 *   cpair:   1 int                          =   4 B
 *   Turtle:                                 ≈ 48 B per turtle
 *   Two turtles total ≈ 100 B — fits in a cache line easily.
 *
 * ALGORITHM REFERENCE
 *   Papert (1980), "Mindstorms" — the pen-with-heading turtle
 *   abstraction. This file's Turtle is a position-and-heading-free
 *   "geometric turtle" that walks pre-computed vertices, but the
 *   conceptual lineage is direct.
 */
typedef struct {
    Polygon poly;        /* the shape — set at init, immutable.       *
                          * Read by poly_vertex, turtle_draw.         */
    Pen     pen;         /* animation state — written each tick by   *
                          * turtle_tick; read by turtle_draw.         */
    int     cpair;       /* ncurses color pair index (PAIR_TURTLE_A  *
                          * or PAIR_TURTLE_B).                       */
} Turtle;

/*
 * turtle_init() — position and reset one turtle from terminal dimensions.
 *
 * half = 0 → left half (center at cols/4)
 * half = 1 → right half (center at 3*cols/4)
 *
 * Radius is chosen so the polygon fits within its half-screen:
 *   horizontal: radius ≤ cols * 0.21  (leave margin from divider and edge)
 *   vertical:   radius * ASPECT ≤ (rows-4) * 0.40  → radius ≤ that / ASPECT
 */
/* Half-screen x centre for left (half == 0) / right (half == 1). */
static inline float half_screen_centre_x(int cols, int half)
{
    return (half == 0) ? (float)cols * HALF_LEFT_X_FRAC
                       : (float)cols * HALF_RIGHT_X_FRAC;
}

/* Polygon radius chosen so the shape fits in its half-screen with
 * margin for the label row and bottom hint strip. The min() picks
 * the tighter of the two bounds; the *0.85 multiplier leaves breathing
 * room around the bounds. ASPECT compensates for 2:1 cell shape. */
static inline float fit_polygon_radius(int cols, int rows)
{
    float max_r_x = (float)cols       * POLY_MAX_R_X_FRAC;
    float max_r_y = (float)(rows - 4) * POLY_MAX_R_Y_FRAC / ASPECT;
    return fminf(max_r_x, max_r_y) * POLY_R_FIT_FRAC;
}

static void turtle_init(Turtle *t, int cols, int rows,
                        int half, int sides, int cpair, float eps)
{
    /* (1) polygon — the shape (where + how many sides) */
    t->poly.cx          = half_screen_centre_x(cols, half);
    t->poly.cy          = (float)(rows - 2) * 0.5f + 1.5f;
    t->poly.radius      = fit_polygon_radius(cols, rows);
    t->poly.sides       = sides;
    t->poly.start_angle = -(float)M_PI / 2.0f;   /* first vertex at top */

    /* (2) pen — animation reset to start of edge 0 */
    t->pen.edge       = 0;
    t->pen.eps        = eps;
    t->pen.edge_timer = 1.0f / eps;
    t->pen.done       = false;

    /* (3) style */
    t->cpair          = cpair;
}

/*
 * turtle_tick() — advance edge timer by dt seconds.
 * When timer expires, move pen to next vertex (edge++).
 */
static void turtle_tick(Turtle *t, float dt)
{
    Pen *p = &t->pen;
    if (p->done) return;
    p->edge_timer -= dt;
    while (p->edge_timer <= 0.0f) {
        p->edge++;
        if (p->edge >= t->poly.sides) {
            p->done = true;
            p->edge = t->poly.sides;   /* cap — closing edge is drawn */
            break;
        }
        p->edge_timer += 1.0f / p->eps;
    }
}

/*
 * poly_vertex() — compute cell position of vertex i (0-based, wraps at n).
 *
 * Vertex i is at angle:  start_angle + i * (2π/sides)
 * Y is scaled by ASPECT to correct for non-square cells.
 */
static void poly_vertex(const Turtle *t, int i, float *vx, float *vy)
{
    const Polygon *g = &t->poly;
    float a = g->start_angle + (float)i * 2.0f * (float)M_PI / (float)g->sides;
    *vx = g->cx + g->radius * cosf(a);
    *vy = g->cy + g->radius * sinf(a) * ASPECT;
}

/* DDA step count — one stamp per cell along the MAJOR axis (the one
 * with the larger absolute delta). Guarantees no gaps in the rendered
 * line at any angle. Minimum 1 so a zero-length segment still tries
 * once at the start point. */
static inline int dda_step_count(float dx, float dy)
{
    int steps = (int)(fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy));
    return steps < 1 ? 1 : steps;
}

/* Stamp one DDA-interpolated cell if it's inside the drawable area
 * (cols × [1, rows-1) — leaves HUD row 0 and hint row rows-1 alone). */
static inline void dda_stamp_cell(WINDOW *w, float x, float y, char ch,
                                  chtype attr, int cols, int rows)
{
    int col = (int)roundf(x);
    int row = (int)roundf(y);
    if (col < 0 || col >= cols || row < 1 || row >= rows - 1) return;
    wattron (w, attr);
    mvwaddch(w, row, col, (chtype)(unsigned char)ch);
    wattroff(w, attr);
}

/*
 * put_seg — rasterise a line segment with DDA (Digital Differential
 * Analyzer). For each of `steps` intermediate t ∈ [0, 1] points,
 * compute (col, row) by linear interpolation and stamp one glyph.
 *
 *   steps  = max(|Δx|, |Δy|)
 *   t_i    = i / steps
 *   col_i  = x0 + dx · t_i
 *   row_i  = y0 + dy · t_i
 *
 * Glyph is chosen ONCE from the segment's overall angle (- | / \) so
 * the whole edge reads as one straight line, not staircased.
 *
 * Algorithm reference: Newman & Sproull (1979) §2, DDA line algorithm.
 */
static void put_seg(WINDOW *w, float x0, float y0, float x1, float y1,
                    chtype attr, int cols, int rows)
{
    float dx    = x1 - x0;
    float dy    = y1 - y0;
    int   steps = dda_step_count(dx, dy);
    char  ch    = angle_char(atan2f(dy, dx));

    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        dda_stamp_cell(w, x0 + dx * t, y0 + dy * t,
                       ch, attr, cols, rows);
    }
}

/* Paint edges [0, pen.edge) — every edge the pen has already finished.
 * Each edge goes from vertex e to vertex (e+1), rasterised by put_seg
 * in the turtle's colour pair. */
static void paint_completed_edges(const Turtle *t, WINDOW *w,
                                  int cols, int rows)
{
    chtype attr = (chtype)(COLOR_PAIR(t->cpair) | A_BOLD);
    for (int e = 0; e < t->pen.edge; e++) {
        float x0, y0, x1, y1;
        poly_vertex(t, e,     &x0, &y0);
        poly_vertex(t, e + 1, &x1, &y1);
        put_seg(w, x0, y0, x1, y1, attr, cols, rows);
    }
}

/* Paint the pen-head glyph '@' at the current vertex (the start of
 * the in-progress edge). Skipped once the polygon is complete so the
 * closed shape looks settled. */
static void paint_pen_head(const Turtle *t, WINDOW *w, int cols, int rows)
{
    if (t->pen.done) return;

    float hx, hy;
    poly_vertex(t, t->pen.edge, &hx, &hy);
    int ix = (int)roundf(hx);
    int iy = (int)roundf(hy);
    if (ix < 0 || ix >= cols || iy < 1 || iy >= rows - 1) return;

    wattron (w, COLOR_PAIR(PAIR_HEAD) | A_BOLD);
    mvwaddch(w, iy, ix, '@');
    wattroff(w, COLOR_PAIR(PAIR_HEAD) | A_BOLD);
}

/*
 * turtle_draw — render the turtle's completed edges and its pen head.
 *
 *   1. paint every edge [0, pen.edge) in the turtle's colour
 *   2. stamp the pen-head '@' at vertex pen.edge (unless done)
 *
 * Painter's order: head after edges so it overdraws the edge endpoint
 * cleanly.
 */
static void turtle_draw(const Turtle *t, WINDOW *w, int cols, int rows)
{
    paint_completed_edges(t, w, cols, rows);
    paint_pen_head       (t, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene — Screen + FrameTimer + Scene container + helpers            */
/* ===================================================================== */

/*
 * §6.1 Screen — current ncurses dimensions.
 *
 * INTENT
 *   Single point of truth for "how big is the terminal RIGHT NOW".
 *   turtle_init derives the polygon centre and radius from these;
 *   every renderer clamps draws to (cols, rows-1) so the bottom
 *   hint strip stays unobstructed; the HUD reads cols to right-
 *   align the status string.
 *
 * MUTATION RULE
 *   Modified only inside screen_init() and screen_resize(). Never
 *   changes mid-frame — physics and renderer treat them as
 *   immutable for the duration of one tick.
 *
 * RESIZE BEHAVIOUR
 *   On SIGWINCH the main loop calls scene_handle_resize() which
 *   updates these dims AND re-seeds both turtles so the polygon
 *   centre + radius re-fit the new geometry. Side counts and pen
 *   speed are preserved across the resize.
 */
typedef struct {
    int cols;            /* terminal width in character cells.       */
    int rows;            /* terminal height in character cells.      */
} Screen;

/*
 * §6.2 FrameTimer — main-loop pacing + rolling fps.
 *
 * INTENT
 *   Pulls the timing locals that used to live in main() (frame_time,
 *   sim_accum, fps_accum, frame_count, fps_display) into one named
 *   struct. Reading the main loop now means reading "the FrameTimer
 *   gets ticked" rather than "five disconnected locals get updated
 *   in scattered places".
 *
 * TWO TIMING SCALES
 *   (1) RENDER scale — every loop iteration paints one frame and
 *       sleeps the remainder of (NS_PER_SEC / 60). Best-effort.
 *   (2) SIM scale — tick_ns = 1e9 / sim_fps, FIXED. Each frame's
 *       dt accumulates into sim_accum; the loop consumes whole
 *       sim ticks via scene_tick. Disk drawing pace stays constant
 *       regardless of render rate.
 *
 * WHY DECOUPLED (renderer != sim)
 *   Turtle drawing speed is a USER-CONTROLLED parameter (eps + sim
 *   Hz). Coupling it to render rate would make the polygon draw
 *   slower on slow machines, breaking the visual cadence. Fiedler's
 *   "Fix Your Timestep!" is the canonical write-up of this pattern.
 *
 * WHY 500 ms FPS WINDOW
 *   Per-frame fps fluctuates wildly. A 500 ms accumulation window
 *   gives a stable, human-readable HUD value. Updated ≈2× per
 *   second.
 *
 * ALGORITHM REFERENCE
 *   Fiedler, "Fix Your Timestep!" — accumulator pattern.
 *   https://gafferongames.com/post/fix_your_timestep/
 */
typedef struct {
    int64_t frame_time;   /* clock_ns() at the start of the previous *
                           * frame. dt = now - frame_time each loop. *
                           * Reset on resize so the post-resize frame*
                           * doesn't see a giant dt spike.            */
    int64_t sim_accum;    /* dt accumulator for fixed-timestep sim.  *
                           * Each frame: sim_accum += dt; while it   *
                           * crosses tick_ns: scene_tick once and    *
                           * sim_accum -= tick_ns. Reset on resize.  */
    int64_t fps_accum;    /* ns elapsed since the last fps update.   *
                           * Resets when it crosses 500 ms.           */
    int     frame_count;  /* frames included in fps_accum.           *
                           *   fps = frame_count · 1e9 / fps_accum   */
    double  fps_display;  /* most recent rolling-window fps reading. *
                           * Updated ≈ 2× per second; shown in HUD.   */
} FrameTimer;

/*
 * §6.3 Scene — the top-level container.
 *
 * INTENT
 *   Owns every long-lived piece of program state in one struct: the
 *   two turtles, their shared drawing speed, the reset countdown,
 *   the pause flag, screen dimensions, loop timing, sim-tick rate,
 *   and signal flags. Replaces what used to be a separate Scene
 *   (turtle pair) wrapped inside an App (Scene + Screen + sim_fps +
 *   signal flags) plus loose timing locals in main().
 *
 * SUB-STRUCT LAYERING (read in this order)
 *
 *   1. tA, tB       the duo (Turtle = Polygon + Pen + cpair)
 *   2. eps          shared drawing speed, propagated to both pens
 *   3. reset_timer  countdown to "auto-reseed with sides += 1"
 *   4. paused       UI flag
 *   5. screen       ncurses dimensions
 *   6. timer        loop pacing (sim + render scales)
 *   7. sim_fps      fixed-timestep sim rate
 *   + running, need_resize    signal-handler flags
 *
 * MUTATION CONTRACT (which keys / events touch what)
 *
 *   tA / tB        a/z, s/x       reseed via turtle_init
 *                  r              both re-seeded at current sides
 *                  (per tick)     pen.edge / pen.edge_timer advance
 *                  (cycle)        sides += 1 on auto-reset
 *   eps            + / -          1.5× scale, clamped [MIN, MAX]
 *   reset_timer    (per tick)     counts up after both pens done;
 *                                 cleared after scene_cycle_polygons
 *   paused         space          freeze sim_tick
 *   sim_fps        [ / ]          fixed-step tick rate, ±SIM_FPS_STEP
 *   running        q/ESC/SIGINT   clears, main loop exits
 *   need_resize    SIGWINCH       set; main loop calls resize handler
 *
 *   Turtle / eps writes affect what's DRAWN.
 *   paused / sim_fps writes affect HOW FAST it draws.
 *   running / need_resize are signal control flags.
 *
 * INITIALIZATION ORDER (see main())
 *   1. install signal handlers + atexit cleanup
 *   2. running = 1, sim_fps = SIM_FPS_DEFAULT
 *   3. screen_init       — bring ncurses up, learn rows/cols
 *   4. scene_init        — seed defaults + both turtles
 *   5. timer.frame_time  — seed FrameTimer
 *
 * WHY A FILE-STATIC GLOBAL (g_scene)
 *   Signal handlers can't take parameters and must be async-signal-
 *   safe. They need a path to running / need_resize. A single file-
 *   scope Scene gives them that without ceremony; everything else
 *   reaches state through  scene->  pointer parameters.
 */
typedef struct Scene_ {
    /* — turtle pair + their shared scene state — */
    Turtle tA;            /* left  turtle (PAIR_TURTLE_A, cyan).      *
                           * Drawn in the left  half: cols * 0.25.   */
    Turtle tB;            /* right turtle (PAIR_TURTLE_B, magenta).   *
                           * Drawn in the right half: cols * 0.75.   */
    float  eps;           /* shared drawing speed (edges/second).     *
                           * Range [EPS_MIN, EPS_MAX] = [0.3, 12.0]. *
                           * + / - scales by 1.5×.                    *
                           * Propagated to each t->pen.eps inside    *
                           * turtle_init.                             */
    float  reset_timer;   /* seconds since both polygons completed.   *
                           * Counts up each tick once both pen.done; *
                           * at RESET_DELAY (= 2 sec) both turtles   *
                           * auto-reseed with sides += 1 (wrap at    *
                           * SIDES_MAX → SIDES_MIN).                  */
    bool   paused;        /* spacebar — freeze the simulation.       *
                           * Rendering keeps running so the user can *
                           * study a frame.                           */

    /* — frame pacing — */
    Screen     screen;    /* ncurses dimensions; refreshed on resize.*/
    FrameTimer timer;     /* sim accumulator + rolling-fps state.    */
    int        sim_fps;   /* fixed-step sim rate (Hz). Clamped to    *
                           * [SIM_FPS_MIN, SIM_FPS_MAX] = [5, 120].  *
                           * Default SIM_FPS_DEFAULT = 30.            *
                           * [ / ] keys nudge by SIM_FPS_STEP = 5.   */

    /* — signal flags — */
    volatile sig_atomic_t running;     /* cleared by 'q'/ESC/SIGINT */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH           */
} Scene;

static void scene_init(Scene *s)
{
    s->eps         = EPS_DEFAULT;
    s->reset_timer = 0.0f;
    s->paused      = false;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, 3, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, 5, PAIR_TURTLE_B, s->eps);
}

/* Both turtles complete → wait RESET_DELAY, then re-seed each with
 * sides += 1 (wrapping SIDES_MAX → SIDES_MIN). Implements the
 * "3 → 4 → … → 12 → 3" cycle the demo headlines. */
static void scene_cycle_polygons(Scene *s)
{
    int na = (s->tA.poly.sides < SIDES_MAX) ? s->tA.poly.sides + 1 : SIDES_MIN;
    int nb = (s->tB.poly.sides < SIDES_MAX) ? s->tB.poly.sides + 1 : SIDES_MIN;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, na, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, nb, PAIR_TURTLE_B, s->eps);
    s->reset_timer = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    turtle_tick(&s->tA, dt);
    turtle_tick(&s->tB, dt);

    if (s->tA.pen.done && s->tB.pen.done) {
        s->reset_timer += dt;
        if (s->reset_timer >= RESET_DELAY)
            scene_cycle_polygons(s);
    }
}

/*
 * Polygon name table (index = sides).
 */
static const char *const POLY_NAMES[] = {
    "", "", "",
    "Triangle", "Square",      "Pentagon",   "Hexagon",
    "Heptagon", "Octagon",     "Nonagon",    "Decagon",
    "Undecagon","Dodecagon"
};

static const char *poly_name(int sides)
{
    if (sides >= SIDES_MIN && sides <= SIDES_MAX) return POLY_NAMES[sides];
    return "Polygon";
}

/* Vertical '|' line at the screen midline separating the two halves. */
static void paint_half_divider(WINDOW *w, int cols, int rows)
{
    wattron(w, COLOR_PAIR(PAIR_DIVIDER) | A_DIM);
    for (int r = 1; r < rows - 1; r++)
        mvwaddch(w, r, cols / 2, '|');
    wattroff(w, COLOR_PAIR(PAIR_DIVIDER) | A_DIM);
}

/* Polygon name label for one turtle, centred at column cx on row 1.
 * Format: "Triangle (3)" — name from the static poly_name table. */
static void paint_poly_label(WINDOW *w, const Turtle *t, int cx, int cpair)
{
    char lab[32];
    snprintf(lab, sizeof lab, "%s (%d)",
             poly_name(t->poly.sides), t->poly.sides);
    wattron (w, COLOR_PAIR(cpair) | A_BOLD);
    mvwprintw(w, 1, cx - (int)strlen(lab) / 2, "%s", lab);
    wattroff(w, COLOR_PAIR(cpair) | A_BOLD);
}

/* "DONE — next in Ns" banner centred horizontally at mid-row. Shown
 * only while both pens are done and the reset_timer is counting up. */
static void paint_done_banner(WINDOW *w, const Scene *s, int cols, int rows)
{
    if (!(s->tA.pen.done && s->tB.pen.done)) return;

    int  sec_left = (int)(RESET_DELAY - s->reset_timer) + 1;
    char msg[48];
    snprintf(msg, sizeof msg, "DONE — next in %ds", sec_left);

    int mx = (cols - (int)strlen(msg)) / 2;
    if (mx < 0) mx = 0;

    wattron (w, COLOR_PAIR(PAIR_DONE) | A_BOLD);
    mvwprintw(w, rows / 2, mx, "%s", msg);
    wattroff(w, COLOR_PAIR(PAIR_DONE) | A_BOLD);
}

/*
 * scene_draw — paint the canvas content (everything except HUD chrome).
 *
 *   1. vertical divider (dim, behind both halves)
 *   2. both turtles (edges + pen heads)
 *   3. polygon name labels above each half
 *   4. "DONE — next in Ns" banner when both polygons finished
 *
 * Painter's order: divider first (behind), labels and banner last
 * (overlay so text is always legible).
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;

    paint_half_divider(w, cols, rows);

    turtle_draw(&s->tA, w, cols, rows);
    turtle_draw(&s->tB, w, cols, rows);

    paint_poly_label(w, &s->tA, cols / 4,     PAIR_TURTLE_A);
    paint_poly_label(w, &s->tB, 3 * cols / 4, PAIR_TURTLE_B);

    paint_done_banner(w, s, cols, rows);
}

/* ===================================================================== */
/* §7  screen — ncurses init / present + HUD bars                         */
/* ===================================================================== */

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

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* HUD bars per CLAUDE.md two-bar spec:
 *   row 0  right    bright yellow + A_BOLD on default bg   live data
 *   row -1 left     bright cyan   + A_BOLD on default bg   action keys
 * Out-of-theme colours so the HUD chrome stays legible no matter what
 * the turtles are drawing behind it; A_BOLD never A_DIM.
 */

/* Tiny per-turtle progress label: "A:3-gon > 2/3" reads as
 * "turtle A, 3-sided polygon, 2 edges drawn out of 3" — the '>'
 * becomes '*' when that polygon completes. */
static int hud_write_turtle_status(char *dst, size_t cap,
                                   char letter, const Turtle *t)
{
    return snprintf(dst, cap, " %c:%d-gon %s %d/%d ",
                    letter,
                    t->poly.sides,
                    t->pen.done ? "*"  : ">",
                    t->pen.edge,
                    t->poly.sides);
}

/* Build the top-row status string. Order: per-turtle progress first
 * (the user's eye lands here when watching convergence), then speed /
 * timing parameters. */
static void hud_format_status(const Scene *sc, double fps, int sim_fps,
                              char *buf, size_t n)
{
    char a_buf[32], b_buf[32];
    hud_write_turtle_status(a_buf, sizeof a_buf, 'A', &sc->tA);
    hud_write_turtle_status(b_buf, sizeof b_buf, 'B', &sc->tB);

    snprintf(buf, n,
             "%s%s %.1f fps  sim:%d Hz  %.2f eps  %s ",
             a_buf, b_buf,
             fps, sim_fps, (double)sc->eps,
             sc->paused ? "PAUSED " : "drawing");
}

/* Paint the HUD status string right-justified on row 0. */
static void hud_paint_status(const char *buf, int cols)
{
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Bottom-row hint strip — lists every interactive key. */
static void hud_paint_hint(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  "
             "a/z:A±sides  s/x:B±sides  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[160];
    hud_format_status(sc, fps, sim_fps, buf, sizeof buf);
    hud_paint_status (buf, s->cols);
    hud_paint_hint   (s->rows);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app — signals, key dispatch, main loop                             */
/* ===================================================================== */

/* The single Scene instance. File-scope so signal handlers can flip
 * the volatile sig_atomic_t flags inside it without ceremony.        */
static Scene g_scene;

static void on_exit_signal(int sig)   { (void)sig; g_scene.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_scene.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* SIGWINCH handler — re-sync ncurses + reseed turtles to the new
 * terminal size (radius depends on cols/rows). Side counts and
 * styles are preserved; only geometry refreshes. */
static void scene_handle_resize(Scene *s)
{
    screen_resize(&s->screen);
    int sa = s->tA.poly.sides;
    int sb = s->tB.poly.sides;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, sa, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, sb, PAIR_TURTLE_B, s->eps);
    s->reset_timer    = 0.0f;
    s->need_resize    = 0;
    s->timer.frame_time = clock_ns();
    s->timer.sim_accum  = 0;
}

/* spacebar — freeze the simulation. */
static void key_pause_toggle(Scene *s) { s->paused = !s->paused; }

/* 'r' — re-init both turtles at current sides. */
static void key_reset_both(Scene *s)
{
    int sa = s->tA.poly.sides, sb = s->tB.poly.sides;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, sa, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, sb, PAIR_TURTLE_B, s->eps);
    s->reset_timer = 0.0f;
}

/* Cycle one turtle's side count by ±1 (wraps SIDES_MAX → SIDES_MIN). */
static void key_change_sides(Scene *s, Turtle *t, int delta,
                             int half, int cpair)
{
    int n = t->poly.sides + delta;
    if      (n > SIDES_MAX) n = SIDES_MIN;
    else if (n < SIDES_MIN) n = SIDES_MAX;
    turtle_init(t, s->screen.cols, s->screen.rows,
                half, n, cpair, s->eps);
    s->reset_timer = 0.0f;
}

/* '+' / '-' — scale drawing speed by 1.5×; propagate to both pens. */
static void key_speed_scale(Scene *s, float factor)
{
    s->eps *= factor;
    if (s->eps > EPS_MAX) s->eps = EPS_MAX;
    if (s->eps < EPS_MIN) s->eps = EPS_MIN;
    s->tA.pen.eps = s->tB.pen.eps = s->eps;
}

/* '[' / ']' — nudge sim-tick rate. */
static void key_sim_fps_nudge(Scene *s, int delta)
{
    s->sim_fps += delta;
    if (s->sim_fps > SIM_FPS_MAX) s->sim_fps = SIM_FPS_MAX;
    if (s->sim_fps < SIM_FPS_MIN) s->sim_fps = SIM_FPS_MIN;
}

/* Dispatch one keystroke; returns false on quit. */
static bool scene_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:  return false;
    case ' ':            key_pause_toggle(s);                              break;
    case 'r': case 'R':  key_reset_both(s);                                break;
    case 'a': case 'A':  key_change_sides(s, &s->tA, +1, 0, PAIR_TURTLE_A); break;
    case 'z': case 'Z':  key_change_sides(s, &s->tA, -1, 0, PAIR_TURTLE_A); break;
    case 's': case 'S':  key_change_sides(s, &s->tB, +1, 1, PAIR_TURTLE_B); break;
    case 'x': case 'X':  key_change_sides(s, &s->tB, -1, 1, PAIR_TURTLE_B); break;
    case '=': case '+':  key_speed_scale(s, EPS_SCALE_FACTOR);             break;
    case '-':            key_speed_scale(s, 1.0f / EPS_SCALE_FACTOR);      break;
    case ']':            key_sim_fps_nudge(s, +SIM_FPS_STEP);              break;
    case '[':            key_sim_fps_nudge(s, -SIM_FPS_STEP);              break;
    default: break;
    }
    return true;
}

/* ───────────────────────────────────────────────────────────────── *
 *  §8.1 setup + per-step main-loop helpers
 * ───────────────────────────────────────────────────────────────── */

/* One-shot setup before the loop: signals, scene defaults, ncurses,
 * turtle pair, FrameTimer seed. */
static void scene_setup(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* (1) seed defaults */
    g_scene.running = 1;
    g_scene.sim_fps = SIM_FPS_DEFAULT;

    /* (2) bring subsystems up in dependency order */
    screen_init(&g_scene.screen);
    scene_init (&g_scene);

    /* (3) seed FrameTimer */
    g_scene.timer.frame_time  = clock_ns();
    g_scene.timer.fps_display = 0.0;
}

/* Measure wall-clock dt since previous frame, capped at DT_CAP_NS so
 * a stalled process can't dump a giant catch-up burst into the sim. */
static int64_t frame_measure_dt(FrameTimer *tm)
{
    int64_t now = clock_ns();
    int64_t dt  = now - tm->frame_time;
    tm->frame_time = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;
    return dt;
}

/* Fixed-timestep sim advance — accumulate dt and consume one full
 * sim tick whenever the accumulator crosses tick_ns. Decouples disk
 * pace from render fps. Returns the dt_sec used for one tick (needed
 * by screen_draw for interpolation alpha). */
static float scene_advance_sim_burst(Scene *s, int64_t dt)
{
    int64_t tick_ns = TICK_NS(s->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

    s->timer.sim_accum += dt;
    while (s->timer.sim_accum >= tick_ns) {
        scene_tick(s, dt_sec);
        s->timer.sim_accum -= tick_ns;
    }
    return dt_sec;
}

/* Rolling-fps update — recomputes fps_display once the accumulator
 * crosses FPS_UPDATE_MS (500 ms). */
static void frame_tick_fps_window(FrameTimer *tm, int64_t dt)
{
    tm->frame_count++;
    tm->fps_accum += dt;
    if (tm->fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
        tm->fps_display = (double)tm->frame_count
                        / ((double)tm->fps_accum / (double)NS_PER_SEC);
        tm->frame_count = 0;
        tm->fps_accum   = 0;
    }
}

/* Sleep the remainder of one render frame so we hit RENDER_FPS = 60. */
static void frame_cap_to_render_fps(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(RENDER_FRAME_NS - elapsed);
}

/* Drain one keystroke through scene_handle_key; return false on quit. */
static bool scene_drain_one_key(Scene *s)
{
    int ch = getch();
    if (ch == ERR) return true;
    return scene_handle_key(s, ch);
}

/* ───────────────────────────────────────────────────────────────── *
 *  §8.2 main
 *
 *  Reads as the program's lifecycle:
 *
 *    SETUP:
 *      scene_setup() — signals, defaults, ncurses, turtles, timer
 *
 *    LOOP (each frame):
 *      1. handle pending resize
 *      2. measure dt (capped)
 *      3. advance sim by whole ticks (fixed-timestep accumulator)
 *      4. tick rolling-fps window
 *      5. sleep to hit RENDER_FPS budget
 *      6. draw + present
 *      7. dispatch one keystroke (quit = ESC/q/Q)
 * ───────────────────────────────────────────────────────────────── */
int main(void)
{
    scene_setup();

    while (g_scene.running) {
        /* (1) deferred resize */
        if (g_scene.need_resize)
            scene_handle_resize(&g_scene);

        /* (2) frame timing */
        int64_t frame_start = g_scene.timer.frame_time;
        int64_t dt          = frame_measure_dt(&g_scene.timer);

        /* (3) fixed-step sim burst */
        float   dt_sec = scene_advance_sim_burst(&g_scene, dt);
        float   alpha  = (float)g_scene.timer.sim_accum
                       / (float)TICK_NS(g_scene.sim_fps);

        /* (4) rolling fps for HUD */
        frame_tick_fps_window(&g_scene.timer, dt);

        /* (5) sleep before render so frame cap is consistent */
        frame_cap_to_render_fps(frame_start, dt);

        /* (6) draw + present */
        screen_draw(&g_scene.screen, &g_scene,
                    g_scene.timer.fps_display, g_scene.sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* (7) dispatch one keystroke */
        if (!scene_drain_one_key(&g_scene))
            g_scene.running = 0;
    }

    screen_free(&g_scene.screen);
    return 0;
}
