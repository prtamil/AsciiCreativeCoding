/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * l_system.c — L-System Fractal Generator
 *
 * String-rewriting L-systems with turtle-graphics rendering.
 * Five presets, each building generation by generation:
 *
 *   0  Dragon Curve      — self-similar fold; 90° turns
 *   1  Hilbert Curve     — space-filling; 90° turns
 *   2  Sierpinski Arrow  — triangle subdivision; 60° turns
 *   3  Branching Plant   — organic fractal tree; 25° turns, branch stack
 *   4  Koch Snowflake    — edge-replacement; 60° turns
 *
 * Framework: follows framework.c §1–§8 skeleton exactly.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config   — presets, constants
 *   §2  clock    — monotonic ns clock + sleep
 *   §3  color    — 7 pairs, one per branch depth
 *   §4  coords   — CELL_W/H aspect correction for isotropic turtle motion
 *   §5  entity   — LSystem: string rewriting + turtle-graphics draw
 *   §6  scene
 *   §7  screen
 *   §8  app
 * ─────────────────────────────────────────────────────────────────────
 *
 * L-SYSTEM NOTATION
 * ─────────────────────────────────────────────────────────────────────
 *   Variables: letters rewritten each generation (A–Z, X, Y …)
 *   Constants: symbols that pass through unchanged (+, -, [, ], F …)
 *   Axiom:    starting string (generation 0)
 *   Rules:    variable → expansion string
 *   After n generations: rules applied n times to the axiom.
 *
 * TURTLE SYMBOLS
 *   F / A / B    move forward one step, draw a segment
 *   f            move forward one step, no draw (lift pen)
 *   +            turn left  by angle δ
 *   -            turn right by angle δ
 *   [            push position + heading onto stack (branch start)
 *   ]            pop  position + heading from stack  (branch end)
 *   X / Y        rewrite-only; no turtle action
 *
 * AUTO-SCALING (the key challenge)
 * ─────────────────────────────────────────────────────────────────────
 *   Each generation the string grows exponentially, so the path grows.
 *   Rather than choosing a fixed step size, we:
 *     1. Dry-run the turtle with step=1 to find the bounding box.
 *     2. Compute scale = min(screen_w / bbox_w, screen_h / bbox_h).
 *     3. Compute centering offsets (cx_off, cy_off).
 *     4. Render with step = scale, offset applied.
 *   Recalculated once per generation change; cached for all frames until
 *   the next advance.
 *
 * ASPECT CORRECTION (§4)
 * ─────────────────────────────────────────────────────────────────────
 *   Terminal cells are ~2× taller than wide (CELL_H/CELL_W ≈ 2).
 *   Turtle Y-motion is multiplied by ASPECT = CELL_W/CELL_H ≈ 0.5 so
 *   a right-facing turtle and a down-facing turtle cover equal physical
 *   distances.  Applied consistently in both dry-run and draw passes so
 *   the scale factor is correct.
 *
 * BRANCH DEPTH COLORING
 * ─────────────────────────────────────────────────────────────────────
 *   '[' increments depth; ']' restores previous depth.
 *   depth % N_COLORS selects the ncurses color pair.
 *   Non-branching presets always draw at depth 0 (single color).
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume auto-advance
 *   Enter      advance one generation manually
 *   n / p      next / previous preset
 *   r          restart at generation 0
 *   + / =      faster auto-advance
 *   -          slower auto-advance
 *   ] / [      raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra l_system.c -o l_system -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : String-rewriting L-system with turtle-graphics rendering.
 *                  Each generation replaces every variable in the string using
 *                  a production rule table.  The resulting string is then
 *                  "executed" by a turtle: F=forward, +=turn_left, -=turn_right,
 *                  [=push_state, ]=pop_state.
 *
 * Math           : L-systems (Lindenmayer, 1968) were invented to model plant
 *                  cell division.  The string length grows exponentially with
 *                  generation: |L_n| = |L_{n-1}| × avg_expansion_factor.
 *                  For the Dragon Curve rule F→F+G, G→F-G: |L_n| = 2ⁿ.
 *                  The limit object (infinite generation) of space-filling L-systems
 *                  (Hilbert, Peano) has Hausdorff dimension = 2 (fills area).
 *
 * Performance    : String length limits: most presets cap at generation 8-12.
 *                  The string is stored in a dynamic buffer; each rewrite step
 *                  O(|string| × max_rule_length) copies characters.
 *                  Aspect correction: forward step is (STEP_PX_COL, STEP_PX_ROW)
 *                  accounting for CELL_W/CELL_H ratio to keep branches isotropic.
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
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 20,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    N_COLORS        =  7,
    N_PRESETS       =  5,
    MAX_RULES       =  4,    /* max rewrite rules per preset             */
    STACK_MAX       = 128,   /* max branch nesting depth                 */

    HUD_ROWS        =  2,    /* rows reserved top (HUD) + bottom (hint)  */
    FPS_UPDATE_MS   = 500,
};

/* Maximum L-system string length.  Stop auto-advancing when the NEXT
 * generation would exceed this. String grows exponentially; 1 MB covers
 * Koch gen 6 (~490 K), Hilbert gen 6 (~490 K), Branching Plant gen 7.  */
#define MAX_STR  (1 * 1024 * 1024)

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/* ── Preset table ──────────────────────────────────────────────────── */

typedef struct { char from; const char *to; } LRule;

typedef struct {
    const char *name;
    const char *axiom;
    LRule       rules[MAX_RULES];
    int         n_rules;
    float       angle_deg;   /* turn angle δ (degrees)                  */
    float       start_deg;   /* initial turtle heading (degrees)         */
    int         max_gen;     /* stop auto-advancing after this           */
    float       advance_sec; /* seconds between auto-advances            */
    const char *draw_chars;  /* chars that move the turtle forward       */
    bool        bottom;      /* true → pin base of fractal to bottom row */
} Preset;

/*
 * Preset definitions.  Max neighbours = (2R+1)²-1.
 *
 * Dragon Curve (gen 1–12)
 *   Axiom FX; rules X→X+YF+, Y→-FX-Y; only F draws.
 *   Self-similar folded path; at high gen fills a triangular region.
 *
 * Hilbert Curve (gen 1–6)
 *   Axiom A; rules A→+BF-AFA-FB+, B→-AF+BFB+FA-.  Only F draws.
 *   Space-filling — at gen 6 it visits every cell in a dense grid.
 *
 * Sierpinski Arrow (gen 1–8)
 *   Axiom A; rules A→B-A-B, B→A+B+A.  Both A and B draw forward.
 *   60° turns produce the classic Sierpinski triangle / arrow.
 *
 * Branching Plant (gen 1–6)
 *   Axiom X; X→F+[[X]-X]-F[-FX]+X, F→FF.  F draws; X is structural.
 *   25° branching angle. Stack produces forking branches.
 *   start_deg = -90° (facing up, since screen y increases downward).
 *   bottom = true so the stem sits at the bottom of the screen.
 *
 * Koch Snowflake (gen 1–5)
 *   Axiom F++F++F (equilateral triangle outline).
 *   Rule F→F-F++F-F; 60° turns.  Three Koch curves form the snowflake.
 */
static const Preset PRESETS[N_PRESETS] = {
    {
        "Dragon Curve",
        "FX",
        { {'X',"X+YF+"}, {'Y',"-FX-Y"} }, 2,
        90.0f, 0.0f, 12, 1.5f, "F", false
    },
    {
        "Hilbert Curve",
        "A",
        { {'A',"+BF-AFA-FB+"}, {'B',"-AF+BFB+FA-"} }, 2,
        90.0f, 0.0f, 6, 2.0f, "F", false
    },
    {
        "Sierpinski Arrow",
        "A",
        { {'A',"B-A-B"}, {'B',"A+B+A"} }, 2,
        60.0f, 0.0f, 8, 1.5f, "AB", false
    },
    {
        "Branching Plant",
        "X",
        { {'X',"F+[[X]-X]-F[-FX]+X"}, {'F',"FF"} }, 2,
        25.0f, -90.0f, 6, 2.0f, "F", true
    },
    {
        "Koch Snowflake",
        "F++F++F",
        { {'F',"F-F++F-F"} }, 1,
        60.0f, 0.0f, 5, 2.0f, "F", false
    },
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

/*
 * Seven color pairs, indexed by branch depth % N_COLORS.
 * Depth 0 (pair 1): brightest — used for the trunk and non-branching
 *                   presets (Dragon, Hilbert, Sierpinski, Koch).
 * Depth 1–6: progressively different hues for branch levels.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1,  46, COLOR_BLACK);   /* bright green  — depth 0 */
        init_pair(2,  82, COLOR_BLACK);   /* lime          — depth 1 */
        init_pair(3, 226, COLOR_BLACK);   /* yellow        — depth 2 */
        init_pair(4, 208, COLOR_BLACK);   /* orange        — depth 3 */
        init_pair(5, 196, COLOR_BLACK);   /* red           — depth 4 */
        init_pair(6, 201, COLOR_BLACK);   /* magenta       — depth 5 */
        init_pair(7,  51, COLOR_BLACK);   /* cyan          — depth 6+ */
        init_pair(8, 226, COLOR_BLACK);   /* yellow HUD                */
    } else {
        init_pair(1, COLOR_GREEN,   COLOR_BLACK);
        init_pair(2, COLOR_GREEN,   COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_RED,     COLOR_BLACK);
        init_pair(5, COLOR_RED,     COLOR_BLACK);
        init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(7, COLOR_CYAN,    COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);
    }
}

/* Map branch depth to ncurses attribute */
static chtype depth_attr(int depth)
{
    int pair = (depth % N_COLORS) + 1;
    return (chtype)(COLOR_PAIR(pair) | (depth == 0 ? A_BOLD : 0));
}

/* ===================================================================== */
/* §4  coords — aspect-ratio correction for isotropic turtle motion       */
/* ===================================================================== */

/*
 * Terminal cells are ~2× taller than wide (CELL_H/CELL_W ≈ 2).
 * Without correction, a turtle moving "right" crosses 1 column but
 * a turtle moving "down" only crosses 0.5 rows for the same physical
 * distance — diagonals look skewed and circles become ellipses.
 *
 * Fix: multiply Y turtle motion by ASPECT = CELL_W/CELL_H ≈ 0.5.
 * Applied consistently in both the dry-run (bounds computation) and
 * the draw pass so the computed scale is correct for both axes.
 *
 * The turtle works in cell coordinates (columns, rows).  It never
 * needs the full pixel↔cell conversion from bounce_ball.c because
 * L-systems have no free-moving physics — the turtle always snaps
 * to the computed step from the auto-scale pass.
 */
#define CELL_W   8
#define CELL_H  16
#define ASPECT   ((float)CELL_W / (float)CELL_H)   /* ≈ 0.5 */

/*
 * angle_char() — pick an ASCII character that best represents the
 * direction of a turtle segment.
 *
 *   0°  (right)      → '-'
 *   45° (down-right) → '\'
 *   90° (down)       → '|'
 *  135° (down-left)  → '/'
 *  180° (left)       → '-'
 *  225° (up-left)    → '\'
 *  270° (up)         → '|'
 *  315° (up-right)   → '/'
 *
 * Modulus π first since '-' and '|' are direction-symmetric.
 */
static char angle_char(float angle)
{
    float a = fmodf(angle, (float)M_PI);
    if (a < 0.0f) a += (float)M_PI;
    if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f)
        return '-';
    if (a < 3.0f * (float)M_PI / 8.0f)
        return '/';
    if (a < 5.0f * (float)M_PI / 8.0f)
        return '|';
    return '\\';
}

/* ===================================================================== */
/* §5  entity — LSystem                                                   */
/* ===================================================================== */

/* ── Turtle stack ────────────────────────────────────────────────────── */

typedef struct { float x, y, angle; int depth; } TurtleState;

/* ── LSystem struct ──────────────────────────────────────────────────── */

/*
 * LSystem — all state for the L-system animation.
 *
 * cur / nxt: two heap-allocated buffers of MAX_STR bytes each.
 *   Rule application writes the next generation into nxt, then
 *   swaps the pointers.  No memcpy — just a pointer exchange.
 *
 * bounds_valid: false after each lsys_advance(); true after the first
 *   turtle_compute_scale() call for the new generation.  The draw pass
 *   checks this flag and recomputes if needed.
 *
 * cx_off, cy_off, scale: cached auto-scale result from the dry-run.
 *   Used by turtle_draw() every frame until the next generation.
 *
 * advance_timer: counts down in seconds; reset to preset.advance_sec
 *   after each auto-advance.  scene_tick() decrements it by dt each
 *   fixed tick.
 */
typedef struct {
    char  *cur;            /* current generation string (null-terminated) */
    char  *nxt;            /* rewrite scratch buffer                      */
    size_t cur_len;        /* strlen(cur)                                 */

    int    gen;            /* current generation number                   */
    int    preset;         /* active preset index                         */
    bool   at_max;         /* true when gen == preset.max_gen             */
    bool   paused;
    float  advance_timer;  /* seconds until next auto-advance             */
    float  advance_sec;    /* current advance interval (user-adjustable)  */

    /* auto-scale cache */
    bool   bounds_valid;
    float  cx_off;         /* x offset (columns) for centering            */
    float  cy_off;         /* y offset (rows)    for centering/alignment  */
    float  step;           /* scaled step size (columns per F)            */
} LSystem;

/* ── allocation ──────────────────────────────────────────────────────── */

static void lsys_alloc(LSystem *ls)
{
    ls->cur = malloc(MAX_STR);
    ls->nxt = malloc(MAX_STR);
    if (!ls->cur || !ls->nxt) { endwin(); exit(1); }
}

static void lsys_free(LSystem *ls)
{
    free(ls->cur); free(ls->nxt);
    ls->cur = ls->nxt = NULL;
}

/* ── rule application ────────────────────────────────────────────────── */

/*
 * lsys_advance() — apply preset rules once, producing gen+1.
 *
 * Uses a 256-entry lookup table (rule_map) so each character is
 * expanded in O(1) rather than searching the rules[] array.
 *
 * Returns false if the result would exceed MAX_STR.
 * On false return, ls is unchanged and at_max is set.
 *
 * String budget:
 *   We check (new_len + expansion_len < MAX_STR) before each append.
 *   If the budget would be exceeded we stop, mark at_max, and leave
 *   the current generation displayed indefinitely.
 */
static bool lsys_advance(LSystem *ls, const Preset *p)
{
    /* Build O(1) rule lookup: rule_map[c] = expansion string or NULL */
    const char *rule_map[256];
    memset(rule_map, 0, sizeof rule_map);
    for (int i = 0; i < p->n_rules; i++)
        rule_map[(unsigned char)p->rules[i].from] = p->rules[i].to;

    size_t new_len = 0;
    for (size_t i = 0; i < ls->cur_len; i++) {
        unsigned char c = (unsigned char)ls->cur[i];
        const char *exp = rule_map[c];
        if (exp) {
            size_t elen = strlen(exp);
            if (new_len + elen >= (size_t)MAX_STR) {
                ls->at_max = true;
                return false;
            }
            memcpy(ls->nxt + new_len, exp, elen);
            new_len += elen;
        } else {
            if (new_len + 1 >= (size_t)MAX_STR) {
                ls->at_max = true;
                return false;
            }
            ls->nxt[new_len++] = (char)c;
        }
    }
    ls->nxt[new_len] = '\0';

    /* Swap pointers — no memcpy */
    char *tmp  = ls->cur;
    ls->cur    = ls->nxt;
    ls->nxt    = tmp;
    ls->cur_len = new_len;
    ls->gen++;
    ls->bounds_valid = false;
    return true;
}

/* ── turtle core — shared by dry-run and draw pass ─────────────────── */

/*
 * turtle_step_bounds() and turtle_draw() share the same traversal loop.
 * Rather than duplicating the loop, we write one function parametrised
 * by draw_mode.  In non-draw mode it tracks min/max; in draw mode it
 * calls mvwaddch.  The compiler inlines and optimises each call site.
 *
 * step  : forward movement per F in cell columns (=1 for dry-run,
 *         =ls->step for draw).
 * ox,oy : turtle origin in cell coordinates.
 *
 * Segment drawing uses a DDA (digital differential analyser) to fill
 * all cells along a segment, not just the endpoint.  This ensures no
 * gaps when step > 1 (early generations have large steps).
 */

static void put_segment(WINDOW *w, float x0, float y0, float x1, float y1,
                        chtype attr, float angle, int cols, int rows)
{
    float dx   = x1 - x0;
    float dy   = y1 - y0;
    int   steps = (int)(fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy));
    if (steps < 1) steps = 1;
    char ch = angle_char(angle);

    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)steps;
        int   cx = (int)roundf(x0 + dx * t);
        int   cy = (int)roundf(y0 + dy * t);
        if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1) {
            wattron(w, attr);
            mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
            wattroff(w, attr);
        }
    }
}

/*
 * turtle_run() — core turtle interpreter.
 *
 * draw_mode = false: accumulate bounding box into minx/miny/maxx/maxy.
 * draw_mode = true : render segments into WINDOW *w.
 *
 * Always uses step and ox/oy as passed; caller sets these appropriately
 * for the dry-run (step=1, ox=oy=0) and the draw pass.
 */
static void turtle_run(const LSystem *ls, const Preset *p,
                       bool draw_mode,
                       WINDOW *w, int cols, int rows,
                       float step, float ox, float oy,
                       float *minx, float *miny,
                       float *maxx, float *maxy)
{
    TurtleState stack[STACK_MAX];
    int    sp    = 0;
    int    depth = 0;
    float  x     = 0.0f;
    float  y     = 0.0f;
    float  angle = p->start_deg * (float)M_PI / 180.0f;
    float  delta = p->angle_deg * (float)M_PI / 180.0f;

    if (!draw_mode) {
        *minx = *miny =  1e30f;
        *maxx = *maxy = -1e30f;
    }

    for (size_t i = 0; i < ls->cur_len; i++) {
        char c = ls->cur[i];

        if (strchr(p->draw_chars, c)) {
            float nx = x + cosf(angle) * step;
            float ny = y + sinf(angle) * step * ASPECT;
            if (draw_mode) {
                chtype attr = depth_attr(depth);
                put_segment(w, ox + x, oy + y, ox + nx, oy + ny,
                            attr, angle, cols, rows);
            } else {
                if (nx < *minx) *minx = nx;
                if (nx > *maxx) *maxx = nx;
                if (ny < *miny) *miny = ny;
                if (ny > *maxy) *maxy = ny;
                /* also track current pos for the very first segment */
                if (x < *minx) *minx = x;
                if (x > *maxx) *maxx = x;
                if (y < *miny) *miny = y;
                if (y > *maxy) *maxy = y;
            }
            x = nx; y = ny;

        } else if (c == '+') {
            angle += delta;
        } else if (c == '-') {
            angle -= delta;
        } else if (c == '[') {
            if (sp < STACK_MAX)
                stack[sp++] = (TurtleState){ x, y, angle, depth };
            depth++;
        } else if (c == ']') {
            if (sp > 0) {
                TurtleState s = stack[--sp];
                x = s.x; y = s.y; angle = s.angle; depth = s.depth;
            }
        }
        /* X, Y, A, B (non-draw variables) and unknowns: no action */
    }
}

/* ── bounds + scale ──────────────────────────────────────────────────── */

/*
 * lsys_compute_scale() — dry-run the turtle, compute step and offsets
 * that fit the fractal into the usable screen area.
 *
 * Usable area: cols × (rows - HUD_ROWS) after reserving top row (HUD)
 * and bottom row (key hints).
 *
 * scale = min(usable_w / bbox_w, usable_h / bbox_h)  with 2-cell margin.
 *
 * For preset.bottom == true (Branching Plant): pin the stem to the
 * bottom of the usable area instead of centering vertically.
 */
static void lsys_compute_scale(LSystem *ls, const Preset *p,
                                int cols, int rows)
{
    float minx, miny, maxx, maxy;
    turtle_run(ls, p, false, NULL, cols, rows,
               1.0f, 0.0f, 0.0f,
               &minx, &miny, &maxx, &maxy);

    float bw = maxx - minx;
    float bh = maxy - miny;
    if (bw < 1.0f) bw = 1.0f;
    if (bh < 1.0f) bh = 1.0f;

    int usable_cols = cols - 2;
    int usable_rows = rows - HUD_ROWS - 2;   /* -2 for top/bottom margin */
    if (usable_cols < 1) usable_cols = 1;
    if (usable_rows < 1) usable_rows = 1;

    float sx = (float)usable_cols / bw;
    float sy = (float)usable_rows / bh;
    float s  = (sx < sy) ? sx : sy;
    if (s < 0.5f) s = 0.5f;   /* never shrink below half-cell steps */

    ls->step = s;

    /* Center horizontally always */
    ls->cx_off = ((float)cols - bw * s) / 2.0f - minx * s;

    /* Vertical: pin to bottom for plant, center for all others */
    if (p->bottom) {
        /* maxy * s is the bottom of the bounding box in scaled space.
         * We want that to sit at (rows - 2) (last usable row).         */
        ls->cy_off = (float)(rows - 2) - maxy * s;
    } else {
        ls->cy_off = ((float)(rows - HUD_ROWS) - bh * s) / 2.0f
                     - miny * s + 1.0f;  /* +1: skip HUD row            */
    }

    ls->bounds_valid = true;
}

/* ── init / set preset ───────────────────────────────────────────────── */

static void lsys_set_preset(LSystem *ls, int p, int cols, int rows);

static void lsys_init(LSystem *ls, int cols, int rows)
{
    memset(ls, 0, sizeof *ls);
    lsys_alloc(ls);
    lsys_set_preset(ls, 0, cols, rows);
}

static void lsys_set_preset(LSystem *ls, int p, int cols, int rows)
{
    ls->preset       = (p + N_PRESETS) % N_PRESETS;
    ls->gen          = 0;
    ls->at_max       = false;
    ls->paused       = false;
    ls->bounds_valid = false;
    ls->advance_sec  = PRESETS[ls->preset].advance_sec;
    ls->advance_timer = ls->advance_sec;

    /* Load axiom as generation 0 */
    const char *axiom = PRESETS[ls->preset].axiom;
    ls->cur_len = strlen(axiom);
    memcpy(ls->cur, axiom, ls->cur_len + 1);

    lsys_compute_scale(ls, &PRESETS[ls->preset], cols, rows);
}

static void lsys_restart(LSystem *ls, int cols, int rows)
{
    lsys_set_preset(ls, ls->preset, cols, rows);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { LSystem ls; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    lsys_init(&s->ls, cols, rows);
}

static void scene_free(Scene *s) { lsys_free(&s->ls); }

/*
 * scene_tick() — decrement advance timer; fire lsys_advance when due.
 *
 * dt is the fixed tick duration in seconds (from the accumulator loop).
 * The timer approach decouples the visual advance cadence from sim_fps —
 * the user can adjust advance_sec independently with + / -.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    LSystem *ls = &s->ls;
    const Preset *p = &PRESETS[ls->preset];

    if (ls->paused || ls->at_max) return;

    ls->advance_timer -= dt;
    if (ls->advance_timer <= 0.0f) {
        ls->advance_timer = ls->advance_sec;
        if (ls->gen < p->max_gen) {
            if (lsys_advance(ls, p))
                lsys_compute_scale(ls, p, cols, rows);
        } else {
            ls->at_max = true;
        }
    }
}

/*
 * scene_draw() — render the current generation's turtle path.
 *
 * alpha is accepted for framework signature compatibility but unused —
 * the turtle path is a static image for each generation.
 *
 * Recomputes scale if bounds_valid is false (happens after resize or
 * when called before scene_tick has fired the first time).
 */
static void scene_draw(Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;

    LSystem *ls      = &s->ls;
    const Preset *p  = &PRESETS[ls->preset];

    if (!ls->bounds_valid)
        lsys_compute_scale(ls, p, cols, rows);

    turtle_run(ls, p, true, w, cols, rows,
               ls->step, ls->cx_off, ls->cy_off,
               NULL, NULL, NULL, NULL);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Standard framework screen layer.
 * ONE window (stdscr), ONE flush per frame (doupdate).
 * See framework.c §7 for full double-buffer explanation.
 */
typedef struct { int cols, rows; } Screen;

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

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

static void screen_draw(Screen *s, Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const LSystem *ls = &sc->ls;
    const Preset  *p  = &PRESETS[ls->preset];

    /* HUD — row 0 */
    char buf[128];
    snprintf(buf, sizeof buf,
             " %-20s  gen %d/%d  len:%-7zu  %.1f s/gen  %dHz  %.1ffps  %s ",
             p->name, ls->gen, p->max_gen, ls->cur_len,
             ls->advance_sec, sim_fps, fps,
             ls->paused ? "PAUSED" :
             ls->at_max ? "MAX GEN" : "running");
    attron(COLOR_PAIR(8) | A_BOLD);
    mvprintw(0, 0, "%.*s", s->cols, buf);
    attroff(COLOR_PAIR(8) | A_BOLD);

    /* hint bar — last row */
    attron(A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  Enter:step  n/p:preset  r:restart  +/-:speed ");
    attroff(A_DIM);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    /* Recompute scale for new dimensions */
    LSystem *ls = &app->scene.ls;
    ls->bounds_valid = false;
    lsys_compute_scale(ls, &PRESETS[ls->preset],
                       app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    LSystem *ls = &app->scene.ls;
    int cols = app->screen.cols, rows = app->screen.rows;
    const Preset *p = &PRESETS[ls->preset];

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        ls->paused = !ls->paused;
        break;

    case '\n': case KEY_ENTER: case '\r':
        /* Manual advance one generation */
        if (!ls->at_max && ls->gen < p->max_gen) {
            if (lsys_advance(ls, p))
                lsys_compute_scale(ls, p, cols, rows);
            ls->advance_timer = ls->advance_sec;
        }
        break;

    case 'n':
        lsys_set_preset(ls, ls->preset + 1, cols, rows);
        break;

    case 'p':
        lsys_set_preset(ls, ls->preset - 1, cols, rows);
        break;

    case 'r': case 'R':
        lsys_restart(ls, cols, rows);
        break;

    case '=': case '+':
        ls->advance_sec *= 0.75f;
        if (ls->advance_sec < 0.2f) ls->advance_sec = 0.2f;
        break;

    case '-':
        ls->advance_sec *= 1.33f;
        if (ls->advance_sec > 10.0f) ls->advance_sec = 10.0f;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;

    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── alpha — unused; L-system frames are static images ───── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap — sleep BEFORE render ─────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
