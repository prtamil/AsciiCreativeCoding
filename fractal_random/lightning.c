/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lightning.c  —  fractal branching lightning in a dark terminal sky
 *
 * Growth algorithm — recursive tip branching (not DLA walkers):
 *
 *   One tip starts at the top-centre.  Each tick every active tip
 *   advances exactly one cell downward, leaning left or right according
 *   to its persistent lean bias.  After at least MIN_FORK_STEPS steps,
 *   a tip may fork into two child tips with lean bias ±1 from the parent.
 *   Both children then grow independently from the fork point.
 *
 *   This produces a fractal binary-tree structure:
 *     — single-cell-wide paths (no DLA blobs)
 *     — branches spread apart as they descend (each lean is ±1 wider)
 *     — characters show direction: '|' straight, '/' left-lean, '\' right-lean
 *
 * Color by row position (depth in the bolt):
 *     top third    → light blue   (45)
 *     middle third → teal         (51)
 *     bottom third → bright white (231)
 *
 * Glow: Manhattan-radius 2 halo around every frozen cell.
 *     distance 1  → '|'  teal dim     (inner corona)
 *     distance 2  → '.'  deep blue dim (outer halo)
 *
 * Active tips are drawn as '!' bright white.
 *
 * Life cycle (state machine):
 *   ST_GROWING  → bolt grows from seed downward, branching as it goes
 *   ST_STRIKING → all tips have reached/passed the ground; the full bolt
 *                 blazes bright white while a shockwave ring expands from
 *                 the deepest strike point
 *   ST_DARK     → screen goes black for ~1 second; then a new bolt starts
 *                 from a fresh random x position
 *
 * Keys:
 *   q / ESC   quit
 *   r         force-start a new bolt immediately
 *   + =       more fork probability (more branches)
 *   -         less fork probability (fewer branches)
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lightning.c -o lightning -lncurses
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid        — frozen cells (encodes color+direction), glow array
 *   §5  tip         — single growing branch tip, fork logic
 *   §6  scene       — tip pool, state machine, shockwave draw
 *   §7  screen
 *   §8  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Recursive binary-tree branching with persistent lean bias.
 *                  Each active tip moves one cell downward each tick, leaning
 *                  ±1 column per MIN_FORK_STEPS steps.  After MIN_FORK_STEPS,
 *                  a tip may fork — producing two child tips with bias ±1 from
 *                  parent's bias.  This is NOT a DLA simulation; there are no
 *                  random walkers — only the branching probability is stochastic.
 *
 * Math           : A tip with lean bias b traces a path offset by ±b cells
 *                  per row descended.  After d rows, the tip is at column
 *                  c₀ ± b·d.  With uniform branching, the horizontal spread of
 *                  tips grows as O(d) — creating a roughly triangular shape.
 *                  The path width (cells visited) is a fractal between 1 and 2D:
 *                  single-cell-wide paths with unlimited branching have D ≈ 1.5.
 *
 * Visual         : The shockwave (striking phase) expands as a Manhattan-radius
 *                  ring from the lowest strike point, fading over time.
 *                  Characters encode direction: '|' straight, '/' '\\' leaning.
 *                  Color depth (row / total_rows) maps top-to-bottom: blue→teal→white.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    MAX_TIPS        =  64,   /* max simultaneous growing branch tips    */

    /* Fork timing */
    MIN_FORK_STEPS  =   2,   /* a tip must grow this many cells first   */

    /* Glow */
    GLOW_RADIUS     =   2,   /* Manhattan radius of ambient halo        */

    /* Ground strike + shockwave */
    GROUND_MARGIN   =   4,   /* rows from bottom that trigger strike    */
    FLASH_FRAMES    =  16,   /* shockwave expansion frames              */
    SHOCK_SPEED     =   7,   /* Manhattan radius added per frame        */
    DARK_FRAMES     =  32,   /* black sky frames between bolts          */

    HUD_COLS        =  38,
    FPS_UPDATE_MS   = 500,
};

/*
 * LEAN_PCT  — percent chance a tip follows its lean direction each step.
 *             Higher = more diagonal, more spread between branches.
 * FORK_PCT  — percent chance a tip forks each step (after MIN_FORK_STEPS).
 *             Higher = more branches, shorter segments between forks.
 *
 * These are tunable at runtime with + / - keys (FORK_PCT only).
 */
#define LEAN_PCT_DEFAULT    60
#define FORK_PCT_DEFAULT    30
#define FORK_PCT_MIN         8
#define FORK_PCT_MAX        50

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

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
 * Bolt color progression by depth:
 *   COL_LB     light blue — top third of bolt (near the cloud)
 *   COL_TEAL   teal       — middle third
 *   COL_WHITE  white      — bottom third / tips (hottest, nearest ground)
 *
 * Glow pairs:
 *   COL_GLOW_I  teal dim  — inner halo (1 cell from bolt)
 *   COL_GLOW_O  deep blue — outer halo (2 cells from bolt)
 *
 * COL_SHOCK  yellow — shockwave expansion ring
 */
typedef enum {
    COL_LB     = 1,
    COL_TEAL   = 2,
    COL_WHITE  = 3,
    COL_GLOW_I = 4,
    COL_GLOW_O = 5,
    COL_SHOCK  = 6,
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_LB,     45, COLOR_BLACK);   /* light blue      */
        init_pair(COL_TEAL,   51, COLOR_BLACK);   /* bright cyan/teal */
        init_pair(COL_WHITE, 231, COLOR_BLACK);   /* white           */
        init_pair(COL_GLOW_I, 30, COLOR_BLACK);   /* dark teal glow  */
        init_pair(COL_GLOW_O, 18, COLOR_BLACK);   /* dark navy glow  */
        init_pair(COL_SHOCK, 226, COLOR_BLACK);   /* bright yellow   */
    } else {
        init_pair(COL_LB,    COLOR_CYAN,   COLOR_BLACK);
        init_pair(COL_TEAL,  COLOR_CYAN,   COLOR_BLACK);
        init_pair(COL_WHITE, COLOR_WHITE,  COLOR_BLACK);
        init_pair(COL_GLOW_I,COLOR_CYAN,   COLOR_BLACK);
        init_pair(COL_GLOW_O,COLOR_BLUE,   COLOR_BLACK);
        init_pair(COL_SHOCK, COLOR_YELLOW, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Cell encoding — cells[y][x] stores both color and direction in one byte:
 *
 *   0           → empty
 *   1–9         → frozen bolt cell
 *
 *   encoding = (color_idx − 1) × 3 + dir_idx + 1
 *
 *   color_idx:  1 = light blue, 2 = teal, 3 = white
 *   dir_idx:    0 = '|' (straight down)
 *               1 = '/' (tip moved left while descending)
 *               2 = '\' (tip moved right while descending)
 *
 * Decoding at draw time:
 *   enc        = cells[y][x] − 1          (0–8)
 *   color_idx  = enc / 3                  (0, 1, 2)
 *   dir_idx    = enc % 3                  (0, 1, 2)
 *
 * glow[y][x] — ambient halo level:
 *   0 = none, 1 = outer (COL_GLOW_O), 2 = inner (COL_GLOW_I)
 *
 * lowest_cy / lowest_cx — deepest frozen cell seen (strike fallback).
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    uint8_t glow [GRID_ROWS_MAX][GRID_COLS_MAX];
    int     frozen_count;
    int     lowest_cy, lowest_cx;
    bool    ground_struck;
    int     strike_x, strike_y;
    int     rows, cols;
} Grid;

/*
 * grid_color_for_row() — map vertical position to bolt color index (1–3).
 *
 * Divides the screen into thirds:
 *   top    → 1 (light blue)
 *   middle → 2 (teal)
 *   bottom → 3 (white / brightest)
 */
static int grid_color_for_row(int cy, int rows)
{
    if (cy < rows / 3)     return 1;
    if (cy < 2 * rows / 3) return 2;
    return 3;
}

static void grid_init(Grid *g, int cols, int rows, int seed_x)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    memset(g->glow,  0, sizeof g->glow);
    g->frozen_count  = 0;
    g->lowest_cy     = 0;
    g->lowest_cx     = seed_x;
    g->ground_struck = false;
    g->strike_x      = seed_x;
    g->strike_y      = 0;
    g->rows          = rows;
    g->cols          = cols;

    /* Freeze the seed cell (straight, light-blue) */
    if (seed_x < 0)     seed_x = 0;
    if (seed_x >= cols) seed_x = cols - 1;
    g->cells[0][seed_x] = (uint8_t)((1 - 1) * 3 + 0 + 1);   /* enc for lb|straight */
    g->frozen_count     = 1;
}

/*
 * grid_update_glow() — stamp Manhattan-radius glow onto neighbours.
 * dist 1 → level 2 (inner, COL_GLOW_I)
 * dist 2 → level 1 (outer, COL_GLOW_O)
 * Takes the max so adjacent bolt cells accumulate correctly.
 * Skips cells that are already frozen (bolt channel wins).
 */
static void grid_update_glow(Grid *g, int cx, int cy)
{
    for (int dy = -GLOW_RADIUS; dy <= GLOW_RADIUS; dy++) {
        for (int dx = -GLOW_RADIUS; dx <= GLOW_RADIUS; dx++) {
            int dist = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
            if (dist == 0 || dist > GLOW_RADIUS) continue;
            int gx = cx + dx, gy = cy + dy;
            if (gx < 0 || gx >= g->cols || gy < 0 || gy >= g->rows) continue;
            if (g->cells[gy][gx] != 0) continue;
            uint8_t level = (uint8_t)(GLOW_RADIUS + 1 - dist);   /* 2 or 1 */
            if (level > g->glow[gy][gx]) g->glow[gy][gx] = level;
        }
    }
}

/*
 * grid_freeze_dir() — freeze cell (cx, cy) with direction dx.
 *   dx = 0  → '|'   dx < 0  → '/'   dx > 0  → '\'
 */
static void grid_freeze_dir(Grid *g, int cx, int cy, int dx)
{
    int color_idx = grid_color_for_row(cy, g->rows);   /* 1, 2, 3  */
    int dir_idx   = (dx == 0) ? 0 : (dx < 0 ? 1 : 2); /* 0, 1, 2  */
    g->cells[cy][cx] = (uint8_t)((color_idx - 1) * 3 + dir_idx + 1);
    g->glow[cy][cx]  = 0;
    g->frozen_count++;
    if (cy > g->lowest_cy) { g->lowest_cy = cy; g->lowest_cx = cx; }
    grid_update_glow(g, cx, cy);
}

static bool grid_frozen(const Grid *g, int cx, int cy)
{
    if (cx < 0 || cx >= g->cols || cy < 0 || cy >= g->rows) return false;
    return g->cells[cy][cx] != 0;
}

/*
 * grid_draw() — two-pass draw: glow halo first, bolt on top.
 *
 * Pass 1 (glow):
 *   level 2 → '|' COL_GLOW_I        (inner corona, same shape as bolt)
 *   level 1 → '.' COL_GLOW_O A_DIM  (outer deep-blue halo)
 *
 * Pass 2 (bolt): decode cell encoding → character + color pair, A_BOLD.
 *   The cell value 1–9 encodes both direction char and depth color.
 */
static void grid_draw(const Grid *g, WINDOW *w)
{
    /* Pass 1: glow halo */
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            if (g->cells[cy][cx] != 0) continue;
            uint8_t lv = g->glow[cy][cx];
            if (lv == 0) continue;
            if (lv == 2) {
                wattron(w, COLOR_PAIR(COL_GLOW_I) | A_DIM);
                mvwaddch(w, cy, cx, (chtype)'|');
                wattroff(w, COLOR_PAIR(COL_GLOW_I) | A_DIM);
            } else {
                wattron(w, COLOR_PAIR(COL_GLOW_O) | A_DIM);
                mvwaddch(w, cy, cx, (chtype)'.');
                wattroff(w, COLOR_PAIR(COL_GLOW_O) | A_DIM);
            }
        }
    }

    /* Pass 2: bolt channel */
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            uint8_t v = g->cells[cy][cx];
            if (v == 0) continue;

            int enc       = v - 1;
            int color_idx = enc / 3;   /* 0=lb, 1=teal, 2=white */
            int dir_idx   = enc % 3;   /* 0=|,  1=/,    2=\     */

            chtype ch;
            switch (dir_idx) {
            case 1:  ch = (chtype)'/';  break;
            case 2:  ch = (chtype)'\\'; break;
            default: ch = (chtype)'|';  break;
            }

            ColorID col;
            switch (color_idx) {
            case 0:  col = COL_LB;   break;
            case 1:  col = COL_TEAL; break;
            default: col = COL_WHITE; break;
            }

            wattron(w, COLOR_PAIR(col) | A_BOLD);
            mvwaddch(w, cy, cx, ch);
            wattroff(w, COLOR_PAIR(col) | A_BOLD);
        }
    }
}

/* grid_draw_flash() — entire bolt blazes as '!' bright white */
static void grid_draw_flash(const Grid *g, WINDOW *w)
{
    wattron(w, COLOR_PAIR(COL_WHITE) | A_BOLD);
    for (int cy = 0; cy < g->rows; cy++)
        for (int cx = 0; cx < g->cols; cx++)
            if (g->cells[cy][cx] != 0)
                mvwaddch(w, cy, cx, (chtype)'!');
    wattroff(w, COLOR_PAIR(COL_WHITE) | A_BOLD);
}

/* ===================================================================== */
/* §5  tip                                                                */
/* ===================================================================== */

/*
 * Tip — one actively growing branch of the bolt.
 *
 *   cx, cy  current cell position
 *   lean    persistent horizontal bias: -3 (hard left) to +3 (hard right)
 *           lean = 0 → mostly straight down
 *           lean = ±1 → slight diagonal
 *           lean = ±3 → strong diagonal branch
 *   steps   cells grown since this tip was created (or last forked)
 *   active  false once the tip reaches the ground, goes out of bounds,
 *           or is blocked by an already-frozen cell
 *
 * Each tick a tip:
 *   1. Determines dx: follows lean with LEAN_PCT probability, else 0.
 *   2. Moves to (cx + dx, cy + 1).
 *   3. If the destination is frozen or out of bounds, deactivates.
 *   4. Otherwise freezes the destination and increments steps.
 *   5. After MIN_FORK_STEPS, rolls FORK_PCT: if fork, spawns two child
 *      tips (lean ± 1) at the current position and deactivates itself.
 */
typedef struct {
    int  cx, cy;
    int  lean;
    int  steps;
    bool active;
} Tip;

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef enum { ST_GROWING, ST_STRIKING, ST_DARK } SceneState;

typedef struct {
    Grid        grid;
    Tip         tips[MAX_TIPS];
    int         n_tips;        /* used slots (active + inactive)         */
    bool        paused;
    SceneState  state;
    int         flash_timer;
    int         dark_timer;
    int         shock_radius;
    int         strike_x;
    int         strike_y;
    int         fork_pct;      /* runtime-tunable fork probability       */
} Scene;

/* scene_add_tip() — append a new tip if the pool has room */
static void scene_add_tip(Scene *s, int cx, int cy, int lean)
{
    if (s->n_tips >= MAX_TIPS) return;
    s->tips[s->n_tips].cx     = cx;
    s->tips[s->n_tips].cy     = cy;
    s->tips[s->n_tips].lean   = lean;
    s->tips[s->n_tips].steps  = 0;
    s->tips[s->n_tips].active = true;
    s->n_tips++;
}

static void scene_start_bolt(Scene *s, int cols, int rows)
{
    int seed_x = cols / 3 + rand() % (cols / 3 + 1);
    grid_init(&s->grid, cols, rows, seed_x);

    /* Zero tips pool */
    memset(s->tips, 0, sizeof s->tips);
    s->n_tips = 0;

    /* Three initial tips spread left/center/right — no single root stem */
    scene_add_tip(s, seed_x, 0, -1);
    scene_add_tip(s, seed_x, 0,  0);
    scene_add_tip(s, seed_x, 0, +1);

    s->state        = ST_GROWING;
    s->flash_timer  = 0;
    s->dark_timer   = 0;
    s->shock_radius = 0;
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->paused   = false;
    s->fork_pct = FORK_PCT_DEFAULT;
    scene_start_bolt(s, cols, rows);
}

/*
 * scene_tick_growing() — advance every active tip by one step.
 *
 * Each tip:
 *   dx = lean > 0 ? +1 : lean < 0 ? -1 : 0,  applied with LEAN_PCT chance
 *   dy = +1 always (bolt only grows downward)
 *
 * On reaching ground margin: records strike point, deactivates tip.
 * On hitting a frozen cell or column edge: deactivates tip (blocked).
 *
 * Fork: after MIN_FORK_STEPS steps, fork_pct chance per step.
 *   Left child:  lean − 1  (clamped to −3)
 *   Right child: lean + 1  (clamped to +3)
 *   Parent deactivates so only children continue.
 *
 * When no active tips remain, transitions to ST_STRIKING.
 */
static void scene_tick_growing(Scene *s)
{
    for (int i = 0; i < s->n_tips; i++) {
        Tip *t = &s->tips[i];
        if (!t->active) continue;

        /* Horizontal lean step */
        int dx = 0;
        if (t->lean != 0) {
            int lean_sign = (t->lean > 0) ? 1 : -1;
            if (rand() % 100 < LEAN_PCT_DEFAULT) dx = lean_sign;
        }

        int nx = t->cx + dx;
        int ny = t->cy + 1;

        /* Clamp horizontally so bolt doesn't leave the screen */
        if (nx < 0)             nx = 0;
        if (nx >= s->grid.cols) nx = s->grid.cols - 1;

        /* Off the bottom: tip has reached the ground */
        if (ny >= s->grid.rows) {
            if (!s->grid.ground_struck) {
                s->grid.ground_struck = true;
                s->grid.strike_x      = t->cx;
                s->grid.strike_y      = t->cy;
            }
            t->active = false;
            continue;
        }

        /* Destination already frozen: tip is blocked */
        if (grid_frozen(&s->grid, nx, ny)) {
            t->active = false;
            continue;
        }

        /* Advance: freeze new cell, update tip */
        grid_freeze_dir(&s->grid, nx, ny, dx);
        t->cx = nx;
        t->cy = ny;
        t->steps++;

        /* Check ground margin */
        if (!s->grid.ground_struck && ny >= s->grid.rows - GROUND_MARGIN) {
            s->grid.ground_struck = true;
            s->grid.strike_x      = nx;
            s->grid.strike_y      = ny;
        }

        /* Fork check (only after minimum steps, only if slots remain) */
        if (t->steps >= MIN_FORK_STEPS
                && rand() % 100 < s->fork_pct
                && s->n_tips <= MAX_TIPS - 2) {
            int ll = t->lean - 1; if (ll < -3) ll = -3;
            int rl = t->lean + 1; if (rl >  3) rl =  3;
            scene_add_tip(s, nx, ny, ll);
            scene_add_tip(s, nx, ny, rl);
            t->active = false;   /* parent deactivates; children continue */
        }
    }

    /* Check if all tips have died */
    bool any_alive = false;
    for (int i = 0; i < s->n_tips; i++)
        if (s->tips[i].active) { any_alive = true; break; }

    if (!any_alive) {
        /* Ensure ground_struck is set (fallback to deepest point reached) */
        if (!s->grid.ground_struck) {
            s->grid.ground_struck = true;
            s->grid.strike_x      = s->grid.lowest_cx;
            s->grid.strike_y      = s->grid.lowest_cy;
        }
    }

    if (s->grid.ground_struck) {
        s->state        = ST_STRIKING;
        s->flash_timer  = FLASH_FRAMES;
        s->shock_radius = 0;
        s->strike_x     = s->grid.strike_x;
        s->strike_y     = s->grid.strike_y;
    }
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;

    switch (s->state) {
    case ST_GROWING:
        scene_tick_growing(s);
        break;

    case ST_STRIKING:
        s->shock_radius += SHOCK_SPEED;
        s->flash_timer--;
        if (s->flash_timer <= 0) {
            s->state      = ST_DARK;
            s->dark_timer = DARK_FRAMES;
        }
        break;

    case ST_DARK:
        s->dark_timer--;
        if (s->dark_timer <= 0)
            scene_start_bolt(s, s->grid.cols, s->grid.rows);
        break;
    }
}

/*
 * draw_shockwave() — expanding diamond ring from the ground-strike point.
 *
 * Uses aspect-ratio-adjusted Manhattan distance (y counts ×2) to produce
 * an ellipse that looks circular on a typical terminal cell shape.
 *
 * Ring front  (dist ∈ [radius−2, radius]):  '*'  yellow A_BOLD
 * Wake trail  (dist ∈ [radius−9, radius−3]): '.'  light-blue A_DIM
 */
static void draw_shockwave(WINDOW *w,
                           int sx, int sy, int radius,
                           int cols, int rows)
{
    for (int cy = 0; cy < rows; cy++) {
        int dy = cy - sy; if (dy < 0) dy = -dy;
        for (int cx = 0; cx < cols; cx++) {
            int dx   = cx - sx; if (dx < 0) dx = -dx;
            int dist = dx + dy * 2;
            if (dist >= radius - 2 && dist <= radius) {
                wattron(w, COLOR_PAIR(COL_SHOCK) | A_BOLD);
                mvwaddch(w, cy, cx, (chtype)'*');
                wattroff(w, COLOR_PAIR(COL_SHOCK) | A_BOLD);
            } else if (dist >= radius - 9 && dist < radius - 2) {
                wattron(w, COLOR_PAIR(COL_GLOW_I) | A_DIM);
                mvwaddch(w, cy, cx, (chtype)'.');
                wattroff(w, COLOR_PAIR(COL_GLOW_I) | A_DIM);
            }
        }
    }
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    switch (s->state) {

    case ST_GROWING:
        /* Glow halo + bolt channel */
        grid_draw(&s->grid, w);
        /* Active tips drawn as '!' bright white on top */
        wattron(w, COLOR_PAIR(COL_WHITE) | A_BOLD);
        for (int i = 0; i < s->n_tips; i++) {
            const Tip *t = &s->tips[i];
            if (!t->active) continue;
            if (t->cy >= 0 && t->cy < s->grid.rows &&
                t->cx >= 0 && t->cx < s->grid.cols)
                mvwaddch(w, t->cy, t->cx, (chtype)'!');
        }
        wattroff(w, COLOR_PAIR(COL_WHITE) | A_BOLD);
        break;

    case ST_STRIKING:
        /* Full bolt flash first, then shockwave sweeps over it */
        grid_draw_flash(&s->grid, w);
        draw_shockwave(w, s->strike_x, s->strike_y, s->shock_radius,
                       s->grid.cols, s->grid.rows);
        break;

    case ST_DARK:
        /* Dark sky — nothing drawn */
        break;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols;
    int rows;
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

/* HUD only during GROWING so strike and dark remain cinematic */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    if (sc->state == ST_GROWING) {
        char buf[HUD_COLS + 1];
        snprintf(buf, sizeof buf,
                 "%5.1f fps  cells:%-4d  fork:%d%%  spd:%d",
                 fps, sc->grid.frozen_count, sc->fork_pct, sim_fps);
        int hx = s->cols - HUD_COLS;
        if (hx < 0) hx = 0;
        attron(COLOR_PAIR(COL_GLOW_I) | A_DIM);
        mvprintw(0, hx, "%s", buf);
        attroff(COLOR_PAIR(COL_GLOW_I) | A_DIM);
    }
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case 'p': case 'P': case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    /* + / - tune fork probability: more/fewer branches */
    case '=': case '+':
        app->scene.fork_pct += 2;
        if (app->scene.fork_pct > FORK_PCT_MAX)
            app->scene.fork_pct = FORK_PCT_MAX;
        break;

    case '-':
        app->scene.fork_pct -= 2;
        if (app->scene.fork_pct < FORK_PCT_MIN)
            app->scene.fork_pct = FORK_PCT_MIN;
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
    srand((unsigned int)clock_ns());

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

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
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

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
