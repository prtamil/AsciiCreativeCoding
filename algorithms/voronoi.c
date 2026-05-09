/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * voronoi.c — Animated Voronoi Diagram
 *
 * N_SEEDS seed points drift with Langevin Brownian motion and bounce off
 * the screen boundary.  Each frame every terminal cell is coloured by its
 * nearest seed (brute-force O(cells × seeds) nearest-neighbour search).
 *
 * PHYSICS
 * ───────
 * Langevin equation for each seed:
 *   dv/dt = −γ·v + σ·ξ      (ξ = uniform random in [−1,1])
 * Discrete update per tick:
 *   v += (−DAMP·v + NOISE·ξ) · dt
 *   p += v · dt
 * This gives self-limiting Brownian motion (terminal speed ≈ NOISE/DAMP).
 *
 * DRAWING
 * ───────
 * For each cell centre (px, py) in pixel space:
 *   • find d1 (nearest seed distance) and d2 (second nearest)
 *   • if d2 − d1 < BORDER_PX → border cell → draw '+' at normal brightness
 *   • if d1 < SEED_PX         → seed centre  → draw 'O' bold
 *   • otherwise                → interior    → draw '.' dim
 * All cells coloured with their nearest seed's colour pair.
 *
 * Keys:
 *   q/ESC quit   space pause   r reset seeds   ] / [  sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra voronoi.c -o voronoi -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Brute-force Voronoi diagram — O(cells × seeds) per frame.
 *                  For each cell, scan all seeds and find the nearest.
 *                  Efficient for small N (N_SEEDS ≤ 30); for larger N, a
 *                  Fortune's sweep-line algorithm gives O(N log N).
 *
 * Math           : The Voronoi diagram partitions the plane into regions where
 *                  each region is the set of points closer to one seed than all
 *                  others.  The dual graph of the Voronoi diagram is the Delaunay
 *                  triangulation (every Voronoi edge connects the circumcentres
 *                  of two Delaunay triangles).
 *                  Border detection: cell is a border cell when the distance to
 *                  the nearest seed and second-nearest differ by less than BORDER_PX.
 *                  This approximates the Voronoi edge without exact line computation.
 *
 * Physics        : Seeds move under the Langevin equation:
 *                    dv/dt = −γ·v + σ·ξ  (ξ = white noise)
 *                  This is Ornstein-Uhlenbeck process — Brownian motion with
 *                  mean-reverting velocity (terminal speed = NOISE/DAMP).
 *
 * References     :
 *   Aurenhammer, "Voronoi diagrams" (ACM Comp. Surv. 23, 1991) —
 *     comprehensive survey of algorithms + applications.
 *   Fortune, "A sweepline algorithm for Voronoi diagrams"
 *     (Algorithmica 2, 1987) — the O(N log N) classical algorithm
 *     not used here.
 *   de Berg et al., "Computational Geometry" (3rd ed., 2008) ch. 7.
 *   See also: procedural/generational/voronoi_region_map.c (static
 *     Voronoi for region mapping) and the Voronoi-Delaunay duality
 *     in procedural/generational/delaunay_triangulation.c.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Pin N seeds in the plane.  For every other point in the
 * plane, ask "which seed am I closest to?"  The points that
 * share an answer form a CELL — one Voronoi region per seed.
 * The cell boundaries are the locus of points equidistant from
 * two seeds (perpendicular bisectors).  Brute-force compute by
 * asking the question per terminal cell.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture N post offices.  For every house in town, the closest
 * post office is the one that delivers your mail.  The map of
 * "which post office serves where" partitions the town into
 * convex polygonal districts — Voronoi cells.  Move a post
 * office and the boundaries shift.  Add a new one and a fresh
 * district carves out of the existing ones.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   ●·························                    │
 *      │   ····●··················                       │
 *      │   ··········.....●·····                          │
 *      │   ···········.....·····●·                        │
 *      │   ···············......                          │
 *      │   ●··············●····                           │
 *      │                                                  │
 *      │   '●' = seeds (post offices)                     │
 *      │   '·' = filled-in district                       │
 *      │   borders detected by "second-nearest is close"  │
 *      └──────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (per frame)
 * ───────────────────────────────
 *  1. Move seeds (Langevin: damped Brownian, mean-reverting).
 *  2. For each terminal CELL (col, row):
 *     a. Compute d1 = nearest seed distance, d2 = second
 *        nearest, by scanning all N seeds.
 *     b. If d2 - d1 < BORDER_PX: cell is on a Voronoi edge →
 *        paint '+' in the nearest seed's colour.
 *        (The border is "where two seeds are nearly tied.")
 *     c. Else if d1 < SEED_PX: cell is a seed centre → paint 'O'.
 *     d. Else: cell is interior → paint '.' dim.
 *  3. HUD + present.
 *
 * KEY FORMULAS
 * ────────────
 *   Voronoi cell of seed i:
 *     V(i) = { p : ∀j ≠ i, dist(p, seed_i) ≤ dist(p, seed_j) }
 *
 *   Border test (approximate):
 *     point p is "on the boundary" if
 *       d_2nd_nearest(p) - d_1st_nearest(p) < BORDER_PX
 *
 *   Langevin (Ornstein-Uhlenbeck velocity):
 *     v += (-DAMP · v + NOISE · ξ) · dt    where ξ ∈ [-1, 1] random
 *     p += v · dt
 *     terminal speed ≈ NOISE / DAMP
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Brute-force O(W · H · N) per frame.  At 80 × 24 × 24 =
 *     46k distance comparisons per frame.  Fine.  Fortune's
 *     sweep-line gives O(N log N) but is much harder to write
 *     and unnecessary at N ≤ 30.
 *   • Border detection is APPROXIMATE — wider near vertices
 *     where 3 seeds are nearly equidistant.  For exact lines
 *     you'd compute perpendicular bisectors analytically.
 *   • Bouncing seeds: when a seed hits the screen edge, both
 *     position is clamped AND velocity flipped.  Without the
 *     clamp, seeds drift slowly through the wall.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Default 24 seeds: 24 distinct coloured regions visible.
 *     Each seed has a single 'O' at its centre + a halo of '.'
 *     in its colour.
 *   • Watch borders: as two seeds approach, the border between
 *     them flexes; if seeds collide, one cell may briefly
 *     shrink to nothing and re-emerge as the seeds separate.
 *   • Pause + step through frames: seed positions update
 *     smoothly under Langevin; never teleport.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *   2. §4 voronoi — render loop with brute-force nearest-seed
 *      lookup.  Read AFTER tutorials T1-T4.
 *   3. §5 langevin — seed motion (Ornstein-Uhlenbeck).
 *      Independent of voronoi; read as a self-contained
 *      sub-lesson on noise-driven motion.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   seeds[N_SEEDS]             seed positions + velocities.
 *   d1, d2                     nearest + second-nearest seed
 *                              distances at a point.
 *   BORDER_PX = 15             border detection threshold.
 *   SEED_PX = 12               seed-centre detection radius.
 *   DAMP, NOISE                Langevin coefficients.
 *
 * Background you need
 * ───────────────────
 *   - Distance: |p - q| = sqrt((px - qx)² + (py - qy)²).
 *   - Brute-force search: scan all N candidates for min/second-min.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Fortune's sweep-line algorithm (O(N log N) Voronoi).
 *   - Spatial indexing (kd-tree, etc.) for accelerated nearest-
 *     neighbour search.  Algorithms/kd_tree.c covers that
 *     separately.
 *   - Voronoi-Delaunay duality.  Mentioned in CONCEPTS; see
 *     procedural/generational/delaunay_triangulation.c for the
 *     dual structure.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Four tutorials that build a brute-force animated Voronoi
 * diagram.
 *
 *   T1  What IS a Voronoi diagram?
 *   T2  Brute-force vs. Fortune's algorithm — when each wins
 *   T3  The "second-nearest distance" border-detection trick
 *   T4  Langevin motion — drifting seeds without escape velocity
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS A VORONOI DIAGRAM?
 * ──────────────────────────────
 * Given N "seed" points in a plane, the VORONOI DIAGRAM
 * partitions the rest of the plane into N regions, one per
 * seed, where each region V(i) is the locus of points closer
 * to seed i than to any other seed:
 *
 *     V(i) = { p : dist(p, seed_i) ≤ dist(p, seed_j)  ∀j ≠ i }
 *
 * Properties:
 *   - Every Voronoi region is CONVEX (intersection of half-
 *     planes from perpendicular bisectors).
 *   - Region boundaries are LINE SEGMENTS — the perpendicular
 *     bisectors of segments joining adjacent seeds.
 *   - Vertex points (where 3+ regions meet) are equidistant
 *     from 3+ seeds — the circumcentre of those seeds'
 *     Delaunay triangle.
 *
 * Use cases (this is one of the most-used structures in CS):
 *   - Nearest-neighbour search: which seed is closest to query
 *     point p?  Lookup the cell.
 *   - Region growing: bacterial colony spread, crystal grain
 *     boundaries, cell biology.
 *   - Resource partitioning: schools, fire stations, cell
 *     towers.
 *   - Mesh generation, computational fluid dynamics, image
 *     stippling.
 *   - Procedural generation: organic-looking region maps in
 *     games (see procedural/generational/voronoi_region_map.c).
 *
 * T2  BRUTE-FORCE VS. FORTUNE'S ALGORITHM
 * ───────────────────────────────────────
 * Two ways to compute Voronoi:
 *
 *   BRUTE-FORCE (this file)
 *     For each query point, scan all N seeds.
 *     Cost: O(N) per query.
 *     Animation: O(N · W · H) per frame for full diagram.
 *     At W=80, H=24, N=24: 46K compares per frame.  Trivial.
 *
 *   FORTUNE'S SWEEP-LINE (Fortune 1987)
 *     Sweep a line top-to-bottom; maintain a "beach line" of
 *     parabolas (each seed's region of influence).  Process
 *     seed events (new parabola appears) and circle events
 *     (parabola disappears, vertex created).
 *     Cost: O(N log N) total — independent of grid resolution.
 *     Output: explicit list of edges + vertices.
 *
 * When to use which:
 *   - VISUALISATION (every pixel is a query, like this demo):
 *     brute-force is simpler and competitive when the grid is
 *     small.  GPU-accelerated brute-force is the standard
 *     approach for animated diagrams.
 *
 *   - TOPOLOGY (you need vertex coordinates and edge lists):
 *     Fortune's gives you the structure directly without
 *     pixel-resolution loss.  Required for mesh generation,
 *     CFD, etc.
 *
 *   - SCIENTIFIC SCALE (millions of points): Fortune or even
 *     more advanced (incremental, divide-and-conquer).
 *
 * For terminal-resolution animation with N ≤ 30, brute-force
 * is the right call.
 *
 * T3  "SECOND-NEAREST DISTANCE" BORDER-DETECTION TRICK
 * ────────────────────────────────────────────────────
 * The Voronoi BORDERS are the perpendicular bisectors between
 * seeds.  Computing them analytically (intersecting half-
 * planes) is involved.
 *
 * Cheap trick: a query point sits ON a border if its FIRST and
 * SECOND nearest seeds are at NEARLY EQUAL distance:
 *
 *     border_test(p):
 *       d1 = min over seeds of |p - seed|
 *       d2 = second-min
 *       return (d2 - d1) < BORDER_PX
 *
 * Reasoning: at a border between seed i and seed j, by
 * definition |p - seed_i| = |p - seed_j|, so d1 = d2.  Just
 * off the border, d2 - d1 grows linearly with distance from
 * the border.  Threshold this difference and you get a thick
 * "approximate border" band.
 *
 * Cost: ONE extra distance comparison per query (track top-2
 * instead of just top-1).
 *
 * Approximation quality:
 *   - Borders away from vertices: clean ~BORDER_PX-wide bands.
 *   - Near vertices (3+ seeds nearly equidistant): the band
 *     widens, sometimes filling small triangles.  Visually
 *     fine for a animated demo; not ok for surveying.
 *
 * Same trick generalises to "find the K-nearest-neighbour
 * boundary" by tracking K candidates instead of 2.
 *
 * T4  LANGEVIN MOTION — DRIFTING SEEDS WITHOUT ESCAPE VELOCITY
 * ────────────────────────────────────────────────────────────
 * To animate the diagram, seeds need to MOVE.  Naïve random
 * walks have a problem: variance grows linearly with time, so
 * after N steps the seed drifts ~√N pixels away.  In the
 * limit, all seeds escape any bounded box.
 *
 * The LANGEVIN EQUATION (Ornstein-Uhlenbeck process) is a
 * BOUNDED random walk:
 *
 *     dv/dt = -γ · v + σ · ξ(t)
 *
 *     -γ · v       drag — pulls velocity back toward zero
 *     σ · ξ(t)     random kick (Gaussian or uniform white noise)
 *
 * Discretised:
 *
 *     v += (-DAMP · v + NOISE · ξ) · dt    where ξ ∈ [-1, 1]
 *     p += v · dt
 *
 * Properties:
 *   - Velocity has a STATIONARY DISTRIBUTION centred at 0.
 *     Terminal RMS speed ≈ NOISE / DAMP.
 *   - Position drift IS still unbounded — Langevin is just
 *     bounded VELOCITY, not bounded POSITION.  We add walls
 *     that bounce seeds back at the screen edges.
 *   - At small noise: seeds barely move (low-temperature limit).
 *   - At high noise: seeds fly fast but slow down quickly.
 *
 * Compared with simpler "constant random velocity" motion:
 *   - Pure random velocity: seeds wander chaotically, no
 *     correlation in time.
 *   - Langevin: seeds have MOMENTUM — they keep moving in
 *     the same direction for a few ticks, giving smooth
 *     curves rather than zigzags.
 *
 * Same equation underlies:
 *   - Brownian motion of dust in a fluid (Einstein 1905)
 *   - Stock price models (geometric Brownian motion)
 *   - Particle filtering (in robotics, sensor fusion)
 *   - Reinforcement learning exploration noise
 *
 * It's a versatile "smooth random motion" primitive.
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
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 7,
    N_SEEDS         = 24,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Pixel cell dimensions (logical sub-pixel spacing) */
#define CELL_W  8
#define CELL_H  16

/* Langevin motion parameters */
#define DAMP     2.0f    /* velocity damping coefficient (s⁻¹)             */
#define NOISE   60.0f    /* random force amplitude (px/s per √s)           */

/* Drawing thresholds in pixels */
#define BORDER_PX  15.0f  /* d2−d1 threshold for border cell              */
#define SEED_PX    12.0f  /* d1 threshold for seed-centre cell             */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);
        init_pair(2, 208, COLOR_BLACK);
        init_pair(3, 226, COLOR_BLACK);
        init_pair(4,  46, COLOR_BLACK);
        init_pair(5,  51, COLOR_BLACK);
        init_pair(6,  75, COLOR_BLACK);
        init_pair(7, 201, COLOR_BLACK);
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline float pw(int cols) { return (float)cols * CELL_W; }
static inline float ph(int rows) { return (float)rows * CELL_H; }

/* ===================================================================== */
/* §5  entity — Seed, Voronoi                                             */
/* ===================================================================== */

typedef struct {
    float px, py;   /* position in pixel space                            */
    float vx, vy;   /* velocity (px/s)                                    */
    int   pair;     /* colour pair 1–7                                    */
} Seed;

typedef struct {
    Seed  seeds[N_SEEDS];
    bool  paused;
} Voronoi;

/* randf — uniform float in [−1, 1] */
static float randf(void) { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

static void voronoi_reset(Voronoi *v, int cols, int rows)
{
    float W = pw(cols);
    float H = ph(rows);
    float mx = (float)CELL_W * 3;
    float my = (float)CELL_H * 2;

    for (int i = 0; i < N_SEEDS; i++) {
        Seed *s  = &v->seeds[i];
        s->px    = mx + (float)rand() / RAND_MAX * (W - 2*mx);
        s->py    = my + (float)rand() / RAND_MAX * (H - 2*my);
        s->vx    = randf() * 20.0f;
        s->vy    = randf() * 20.0f;
        s->pair  = (i % N_COLORS) + 1;
    }
    /* Shuffle colour assignments */
    for (int i = N_SEEDS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = v->seeds[i].pair;
        v->seeds[i].pair = v->seeds[j].pair;
        v->seeds[j].pair = t;
    }
}

static void voronoi_init(Voronoi *v, int cols, int rows)
{
    memset(v, 0, sizeof *v);
    voronoi_reset(v, cols, rows);
}

/*
 * voronoi_tick — Langevin motion + wall bounce.
 *
 * Each seed undergoes over-damped Brownian motion:
 *   v += (−DAMP·v + NOISE·ξ) · dt
 * A soft inward force near walls prevents escape:
 *   if px < margin: vx += WALL_PUSH · dt
 */
static void voronoi_tick(Voronoi *v, float dt, int cols, int rows)
{
    if (v->paused) return;

    float W   = pw(cols);
    float H   = ph(rows);
    float mxL = (float)CELL_W * 2;
    float myT = (float)CELL_H * 2;
    float mxR = W - mxL;
    float myB = H - myT;

    for (int i = 0; i < N_SEEDS; i++) {
        Seed *s = &v->seeds[i];

        /* Langevin: damp + random kick */
        s->vx += (-DAMP * s->vx + NOISE * randf()) * dt;
        s->vy += (-DAMP * s->vy + NOISE * randf()) * dt;

        /* Integrate */
        s->px += s->vx * dt;
        s->py += s->vy * dt;

        /* Bounce off walls */
        if (s->px < mxL) { s->px = mxL; s->vx =  fabsf(s->vx); }
        if (s->px > mxR) { s->px = mxR; s->vx = -fabsf(s->vx); }
        if (s->py < myT) { s->py = myT; s->vy =  fabsf(s->vy); }
        if (s->py > myB) { s->py = myB; s->vy = -fabsf(s->vy); }
    }
}

/*
 * voronoi_draw — per-cell nearest-seed search and rendering.
 *
 * Cell centre pixel: (col·CELL_W + CELL_W/2, row·CELL_H + CELL_H/2).
 * Distance uses pixel-space Euclidean metric so Voronoi regions have
 * correct proportions (not distorted by terminal aspect ratio).
 */
static void voronoi_draw(const Voronoi *v, WINDOW *w, int cols, int rows)
{
    float half_cw = (float)CELL_W * 0.5f;
    float half_ch = (float)CELL_H * 0.5f;

    for (int row = 1; row < rows - 1; row++) {
        float cy = (float)row * CELL_H + half_ch;

        for (int col = 0; col < cols; col++) {
            float cx = (float)col * CELL_W + half_cw;

            float d1 = 1e18f, d2 = 1e18f;
            int   best = 0;

            for (int k = 0; k < N_SEEDS; k++) {
                float dx = cx - v->seeds[k].px;
                float dy = cy - v->seeds[k].py;
                float d  = dx*dx + dy*dy;   /* compare squared distances */
                if (d < d1) { d2 = d1; d1 = d; best = k; }
                else if (d < d2) { d2 = d; }
            }

            d1 = sqrtf(d1);
            d2 = sqrtf(d2);

            int   pair = v->seeds[best].pair;
            chtype attr;
            char   ch;

            if (d1 < SEED_PX) {
                ch   = 'O';
                attr = A_BOLD;
            } else if (d2 - d1 < BORDER_PX) {
                ch   = '+';
                attr = 0;
            } else {
                ch   = '.';
                attr = A_DIM;
            }

            wattron(w, COLOR_PAIR(pair) | attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, COLOR_PAIR(pair) | attr);
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Voronoi voronoi; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    voronoi_init(&s->voronoi, cols, rows);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    voronoi_tick(&s->voronoi, dt, cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    voronoi_draw(&s->voronoi, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  seeds:%d  %s ",
             fps, sim_fps, N_SEEDS,
             sc->voronoi.paused ? "PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, " VORONOI ");
    attroff(COLOR_PAIR(5) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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

static bool app_handle_key(App *app, int ch)
{
    Voronoi *v = &app->scene.voronoi;
    Screen  *s = &app->screen;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': v->paused = !v->paused; break;
    case 'r': case 'R':
        voronoi_reset(v, s->cols, s->rows);
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
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            voronoi_reset(&app->scene.voronoi,
                          app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

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

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
