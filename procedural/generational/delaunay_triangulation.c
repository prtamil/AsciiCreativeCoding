/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * delaunay_triangulation.c — watch a triangle mesh build itself, point
 * by point, the Bowyer-Watson way.
 *
 * We drop seed points onto the map one at a time and connect them into
 * triangles following one rule: no triangle's circumcircle (the circle
 * through its three corners) may contain any other point. That rule gives
 * the "fattest" triangles — no skinny slivers — which is exactly what you
 * want for terrain meshes, finite-element grids, anything triangle-based.
 *
 * Worth reading next to:
 *   ./voronoi_region_map.c — the mirror image of this. Voronoi colours
 *     each spot by its nearest seed; Delaunay connects seeds whose Voronoi
 *     regions touch. Same points, two pictures.
 *   geometry/delaunay_triangulation.c — same algorithm, but a proof-style
 *     showcase that draws each circumcircle and checks the rule. This file
 *     tells the story; that one verifies it.
 *
 * References the code can't give you:
 *   Bowyer (1981) & Watson (1981), The Computer Journal 24(2) — the two
 *     namesake papers for this incremental method.
 *   de Berg, Cheong, van Kreveld & Overmars, "Computational Geometry"
 *     (3rd ed., Springer 2008), ch. 9 — the textbook treatment.
 *   Guibas & Stolfi (1985), ACM TOG 4(2) — origin of the in-circle
 *     determinant test that in_circumcircle() uses.
 *   Shewchuk (1997), "Robust Geometric Predicates" — why that determinant
 *     is fragile in floating point (we dodge it with integer coordinates).
 *   Bresenham (1965), IBM Systems Journal 4(1) — the integer line-walk
 *     each edge is drawn with.
 *   https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
 *
 * Section map:
 *   §1 config   — map size, storage caps, the 15 presets
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD pairs + preset palette
 *   §5 dt       — the mesh data, the geometry math, the glow, the algorithm
 *   §6 scene    — the one place all of that combines each tick
 *   §7 render   — turn the mesh into ASCII edges/points + the HUD
 *   §8 app      — main loop, signals, resize, keys
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

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,

    /* Most real points any preset asks for, plus the 3 "super-triangle"
     * corners (explained in §5). Just sizes the storage arrays; how many
     * points a given style actually drops lives in presets[]. */
    N_REAL_MAX        =  32,
    SUPER_TRI_VERTS   =   3,
    MAX_POINTS        = N_REAL_MAX + SUPER_TRI_VERTS,

    /* Room for triangles. We never reclaim a removed triangle's slot (it's
     * just flagged dead, see §5), so this must cover every triangle ever
     * made during a build — roughly 3–6 per point, so 512 is plenty for 32. */
    MAX_TRIS          = 512,

    /* Two scratch buffers used while inserting one point. Sized large enough
     * that they never fill up at N_REAL_MAX — overflowing one would silently
     * drop edges and wreck the mesh. */
    MAX_BAD_TRIS      = 128,
    MAX_EDGE_BUF      = 256,

    /* Keep seed points at least this far apart so the mesh looks balanced.
     * POINT_PLACE_TRIES = how many random spots we'll try before giving up. */
    MIN_POINT_DIST    =  10,
    POINT_PLACE_TRIES = 100,

    /* How many ticks to wait between dropping points while building. */
    INSERT_HOLD_TICKS_DEF = 18,     /* about 0.3 s at 60 Hz */
    INSERT_HOLD_TICKS_MIN =   1,
    INSERT_HOLD_TICKS_MAX = 240,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    FPS_UPDATE_MS     = 500,

    /* Colour-pair slots. HUD/HINT are reserved project-wide; 3 is skipped
     * on purpose so the three drawn layers land on 4..6. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_EDGE         =   4,        /* triangle edges                  */
    PAIR_POINT        =   5,        /* the seed-point glyph            */
    PAIR_FLASH        =   6,        /* brand-new edges, mid-flash      */
};

/* How fast a fresh edge's flash fades, and the brightness it's "gone" at. */
#define EDGE_GLOW_DECAY     2.5f    /* fully faded in ~0.7 s */
#define GLOW_THRESHOLD      0.05f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Preset — one ready-made look. Switching with n/p changes the colours, the
 * point character, and how many points get dropped (so a denser or sparser
 * mesh) — but never the algorithm itself. Each one builds once and holds.
 *
 *   name              shown in the HUD
 *   edge/point/flash  256-colour codes for the three drawn layers, all kept
 *                     bright enough to read against a default background
 *   n_points          how many seed points this style drops (its density),
 *                     never more than N_REAL_MAX
 *   point_glyph       the character drawn at each seed point
 */
typedef struct {
    const char *name;
    short       edge, point, flash;
    int         n_points;
    char        point_glyph;
} Preset;

#define N_PRESETS 15

static const Preset presets[N_PRESETS] = {
  /*  name        edge point flash  pts glyph */
  { "CLASSIC",   51, 231, 220,  12, '@' },   /* cyan / white / gold        */
  { "MATRIX",    46, 118, 226,  16, '#' },   /* greens                     */
  { "NOVA",     201, 219, 226,  10, '*' },   /* magenta / pink / yellow    */
  { "MONO",     250, 255, 244,  14, 'o' },   /* greyscale                  */
  { "OCEAN",     39, 159,  51,  18, '@' },   /* navy / ice / cyan          */
  { "FIRE",     208, 226, 196,  12, '*' },   /* orange / yellow / red      */
  { "FOREST",    82, 154, 226,  20, '+' },   /* greens                     */
  { "DESERT",   178, 230, 220,  14, 'o' },   /* sand / cream / gold        */
  { "ARCTIC",   159, 231,  87,  16, '@' },   /* ice / white / cyan         */
  { "AMETHYST", 141, 219, 201,  22, '*' },   /* violet / pink / magenta    */
  { "EMBER",    166, 214, 202,  10, '#' },   /* deep orange embers         */
  { "NEON",      51, 201, 226,  24, '@' },   /* cyan / magenta / yellow    */
  { "SPARSE",   244, 252, 220,   6, 'O' },   /* few points, big triangles  */
  { "DENSE",     45, 195,  51,  30, '.' },   /* many points, fine mesh     */
  { "REEF",     211, 230, 207,  18, '@' },   /* coral pink / cream         */
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

/* preset_apply — load one preset's three colours into the drawing pairs. */
static void preset_apply(int idx)
{
    if (idx < 0 || idx >= N_PRESETS) idx = 0;
    if (COLORS >= 256) {
        const Preset *p = &presets[idx];
        init_pair(PAIR_EDGE,  p->edge,  -1);
        init_pair(PAIR_POINT, p->point, -1);
        init_pair(PAIR_FLASH, p->flash, -1);
    } else {
        init_pair(PAIR_EDGE,  COLOR_CYAN,    -1);
        init_pair(PAIR_POINT, COLOR_WHITE,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW,  -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    preset_apply(0);
}

/* ===================================================================== */
/* §5  dt — STATE · LOGIC · EFFECTS · SIMULATION                          */
/* ===================================================================== */
/*
 * The core of the program, in four layers you can read top-down — each only
 * uses the ones above it:
 *   5.1 STATE       the data the mesh lives in
 *   5.2 LOGIC       the pure geometry math — no state, no side effects
 *   5.3 EFFECTS     the flash fade — purely cosmetic, the math never reads it
 *   5.4 SIMULATION  Bowyer-Watson — grows the STATE using LOGIC, sets the glow
 */

/* ── 5.1 STATE ───────────────────────────────────────────────────────── */

/*
 * Point — one corner, kept as whole-number grid coordinates. Why integers
 * and not floats: the "is this point inside that circle?" test (§5.2) comes
 * down to the sign of a big sum, and with integers plus 64-bit math that
 * sign is always exact — no float rounding can flip the answer when a point
 * sits right on a circle. Index rule: 0..2 are the three off-screen scaffold
 * corners, 3 and up are the real points, so "index >= 3" means "real".
 */
typedef struct {
    int x, y;          /* column, row on the grid */
} Point;

/*
 * Triangle — one face of the mesh, stored as three point INDICES (not copies
 * of the coordinates). Storing indices means an edge shared by two triangles
 * is literally the same pair of numbers in both, so we can spot shared edges
 * just by comparing them — that's the trick the cavity walk relies on.
 *
 * We never actually delete a triangle. Removing one just flips valid to
 * false and leaves the slot where it is, so every other triangle keeps its
 * index across an insertion. As a result n_tris counts slots ever filled,
 * not how many are currently alive (the HUD counts the live ones separately).
 */
typedef struct {
    int   p[3];        /* the three corners, as indices into points[]       */
    bool  valid;       /* false = removed, but the slot stays put           */
    float new_glow;    /* 1.0 the instant it's born, fades out in ~0.7 s     */
} Triangle;

/* triangle_is_real — true only if all three corners are real points, false
 * if it still touches a scaffold corner. This is the test that keeps the
 * scaffolding off-screen; named once here and reused everywhere. */
static inline bool triangle_is_real(const Triangle *t)
{
    return t->p[0] >= SUPER_TRI_VERTS
        && t->p[1] >= SUPER_TRI_VERTS
        && t->p[2] >= SUPER_TRI_VERTS;
}

/* triangle_is_flashing — true while a fresh triangle's glow is still bright
 * enough to show; flashing ones get drawn in the accent colour. */
static inline bool triangle_is_flashing(const Triangle *t)
{
    return t->new_glow > GLOW_THRESHOLD;
}

/*
 * Delaunay — the whole growing mesh.
 *
 * The "super-triangle" (points 0..2) is a starting scaffold: the algorithm
 * needs a triangulation to insert into, so it begins with one giant triangle
 * stretched far past the map edges, big enough that every real point lands
 * inside it. At the end we just don't draw any triangle that still touches a
 * scaffold corner, and what's left is the real mesh.
 *
 * Both arrays only ever grow during a run — we add to the end and mark things
 * dead in place rather than shuffling, because triangles and edges refer to
 * points by index and indices must not move out from under them.
 */
typedef struct {
    int       w, h;                /* map size in cells                      */
    Point     points[MAX_POINTS];  /* 0..2 scaffold corners, then real ones  */
    int       n_points;            /* points placed so far (3 + n_real)      */
    int       n_real;              /* real points dropped (0..target)        */
    int       target;              /* real points this run will drop (preset)*/
    Triangle  triangles[MAX_TRIS]; /* every triangle ever made; some dead    */
    int       n_tris;              /* slots used (alive + dead)              */

    int       insert_cooldown;     /* ticks left before the next point drops */
} Delaunay;

/* ── 5.2 LOGIC (pure geometry — takes points by value, changes nothing) ── *
 * Whole-number geometry. Everything uses 64-bit math so the multiplies can't
 * overflow on a 200×200 grid. These read no shared state and you can test
 * them on their own — no glow or drawing bug can ever reach in here.
 * ── ─────────────────────────────────────────────────────────────────── */

/* clamp_int — keep v inside [lo, hi]. The key handler runs every knob it
 * changes through this so nothing can be pushed out of range. */
static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/*
 * tri_signed_area_2x — twice the area of triangle (a,b,c), with a sign that
 * tells you which way the corners wind. We only care about the sign:
 * positive, negative, or zero (the three corners are in a straight line). The
 * in-circle test below uses that sign to know the triangle's orientation.
 */
static long long tri_signed_area_2x(Point a, Point b, Point c)
{
    return (long long)(b.x - a.x) * (c.y - a.y)
         - (long long)(c.x - a.x) * (b.y - a.y);
}

/*
 * in_circumcircle — is point p strictly inside the circle through the three
 * corners a, b, c? This is the one rule the whole algorithm turns on.
 *
 * The answer is the sign of a number built from the corners (a determinant).
 * Which sign means "inside" depends on the triangle's winding, so we check
 * the winding first and read the sign accordingly. Exact for integer inputs
 * up to about 100,000 — well past anything on screen.
 */
static bool in_circumcircle(Point p, Point a, Point b, Point c)
{
    long long sign_area = tri_signed_area_2x(a, b, c);
    if (sign_area == 0) return false;       /* corners in a line — no circle */

    long long ax = a.x - p.x, ay = a.y - p.y;
    long long bx = b.x - p.x, by = b.y - p.y;
    long long cx = c.x - p.x, cy = c.y - p.y;
    long long as_ = ax * ax + ay * ay;
    long long bs_ = bx * bx + by * by;
    long long cs_ = cx * cx + cy * cy;

    long long det = ax * (by * cs_ - bs_ * cy)
                  - ay * (bx * cs_ - bs_ * cx)
                  + as_ * (bx * cy - by * cx);

    return (sign_area > 0) ? (det > 0) : (det < 0);
}

/* ── 5.3 EFFECTS (the flash — touches ONLY the new_glow field) ────────── *
 * A new triangle is born glowing (new_glow = 1, set where it's created); this
 * is the one place that glow dims back down. The geometry never reads it, so
 * a bug here can only mess up brightness, never the mesh.
 * ── ─────────────────────────────────────────────────────────────────── */

/* fx_decay — dim every triangle's flash a little, once per tick. Purely
 * cosmetic; dt is the fixed timestep. */
static void fx_decay(Delaunay *d, float dt)
{
    float edge_k = expf(-EDGE_GLOW_DECAY * dt);
    for (int i = 0; i < d->n_tris; i++)
        d->triangles[i].new_glow *= edge_k;
}

/* ── 5.4 SIMULATION (Bowyer-Watson — mutates points/triangles) ───────── */

/*
 * CavityEdge — one edge of the hole the new point carves out. a and b are
 * the two corner indices (always stored a <= b so the same edge always looks
 * the same), and count is how many of the doomed triangles share it. Once
 * we've looked at them all: count 1 means the edge is on the hole's outline
 * (keep it — we'll build new triangles off it), count 2 means it's inside the
 * hole and about to vanish (throw it away).
 */
typedef struct { int a, b, count; } CavityEdge;

/* place_super_triangle — drop the three scaffold corners far outside the map,
 * far enough to swallow every real point yet near enough that the in-circle
 * math stays well inside 64 bits. */
static void place_super_triangle(Delaunay *d)
{
    d->points[0] = (Point){ -5 * d->w,     -5 * d->h };
    d->points[1] = (Point){  5 * d->w,     -5 * d->h };
    d->points[2] = (Point){      d->w / 2,  5 * d->h };
    d->n_points = SUPER_TRI_VERTS;
}

/* seed_initial_triangle — the one starting triangle (the three scaffold
 * corners) that every real point gets inserted into. */
static void seed_initial_triangle(Delaunay *d)
{
    d->triangles[0] = (Triangle){
        .p = {0, 1, 2},
        .valid = true,
        .new_glow = 0.0f,
    };
    d->n_tris = 1;
}

/* dt_reset — wipe the mesh back to just the scaffold and one starting
 * triangle, ready to build `target` real points from scratch. */
static void dt_reset(Delaunay *d, int w, int h, int target)
{
    d->w = w;
    d->h = h;
    d->n_points = 0;
    d->n_real   = 0;
    d->target   = target;
    d->n_tris   = 0;
    d->insert_cooldown = 0;

    place_super_triangle(d);
    seed_initial_triangle(d);
}

/* too_close_to_existing — true if (x, y) is closer than MIN_POINT_DIST to any
 * point already placed. Compares squared distances to skip the square root. */
static bool too_close_to_existing(const Delaunay *d, int x, int y)
{
    for (int i = SUPER_TRI_VERTS; i < d->n_points; i++) {
        int dx = x - d->points[i].x;
        int dy = y - d->points[i].y;
        if (dx * dx + dy * dy < MIN_POINT_DIST * MIN_POINT_DIST)
            return true;
    }
    return false;
}

/*
 * dt_pick_real_point — find a random spot for the next point that isn't too
 * close to the others. If every try is crowded (only happens on absurd
 * inputs) we just use the last one so the build never stalls.
 */
static void dt_pick_real_point(const Delaunay *d, int *out_x, int *out_y)
{
    int best_x = 0, best_y = 0;
    int margin = 2;     /* keep points off the very edge */

    for (int attempt = 0; attempt < POINT_PLACE_TRIES; attempt++) {
        int x = margin + (rand() % (d->w - 2 * margin));
        int y = margin + (rand() % (d->h - 2 * margin));
        best_x = x; best_y = y;
        if (!too_close_to_existing(d, x, y)) break;
    }
    *out_x = best_x;
    *out_y = best_y;
}

/*
 * find_bad_triangles — step 1: mark every live triangle whose circumcircle
 * swallows the new point P. Those are the ones P breaks; together they're the
 * hole P will sit in. is_bad[] is caller-cleared scratch, one slot per
 * triangle. The MAX_BAD_TRIS cap is just a safety net we never actually hit.
 */
static void find_bad_triangles(const Delaunay *d, Point p, bool is_bad[])
{
    int n_bad = 0;
    for (int i = 0; i < d->n_tris; i++) {
        if (!d->triangles[i].valid) continue;
        const Triangle *t = &d->triangles[i];
        if (in_circumcircle(p,
                            d->points[t->p[0]],
                            d->points[t->p[1]],
                            d->points[t->p[2]])) {
            is_bad[i] = true;
            if (++n_bad >= MAX_BAD_TRIS) break;
        }
    }
}

/*
 * cavity_edge_add — note one edge (a,b). Store it as a <= b so the two
 * triangles sharing an edge produce the same key, then either bump its count
 * (seen before) or add it. That count is what later tells a hole-outline edge
 * (seen once) from an inside-the-hole edge (seen twice).
 */
static void cavity_edge_add(CavityEdge edges[], int *n_edges, int a, int b)
{
    if (a > b) { int tmp = a; a = b; b = tmp; }   /* same edge, same key */

    for (int j = 0; j < *n_edges; j++)
        if (edges[j].a == a && edges[j].b == b) { edges[j].count++; return; }

    if (*n_edges < MAX_EDGE_BUF) {
        edges[*n_edges].a     = a;
        edges[*n_edges].b     = b;
        edges[*n_edges].count = 1;
        (*n_edges)++;
    }
}

/*
 * collect_cavity_edges — step 2: feed all three edges of every doomed
 * triangle through cavity_edge_add. Returns how many distinct edges there
 * were; each one's count then says whether it's on the hole's outline.
 */
static int collect_cavity_edges(const Delaunay *d, const bool is_bad[],
                                CavityEdge edges[])
{
    int n_edges = 0;
    for (int i = 0; i < d->n_tris; i++) {
        if (!is_bad[i]) continue;
        for (int e = 0; e < 3; e++) {
            int a = d->triangles[i].p[e];
            int b = d->triangles[i].p[(e + 1) % 3];
            cavity_edge_add(edges, &n_edges, a, b);
        }
    }
    return n_edges;
}

/* invalidate_triangles — step 3: kill the doomed triangles. We just flip
 * their valid flag off and leave the slots alone, so no indices shift. */
static void invalidate_triangles(Delaunay *d, const bool is_bad[])
{
    for (int i = 0; i < d->n_tris; i++)
        if (is_bad[i]) d->triangles[i].valid = false;
}

/*
 * retriangulate_cavity — step 4: close the hole. For each outline edge (the
 * ones seen exactly once), add a new triangle joining that edge to the new
 * point, born glowing. Returns false only if we'd run out of slots (a safety
 * check that can't trip with our sizing).
 */
static bool retriangulate_cavity(Delaunay *d, const CavityEdge edges[],
                                 int n_edges, int p_idx)
{
    for (int j = 0; j < n_edges; j++) {
        if (edges[j].count != 1) continue;          /* inside the hole — skip */
        if (d->n_tris >= MAX_TRIS) return false;
        d->triangles[d->n_tris++] = (Triangle){
            .p = { edges[j].a, edges[j].b, p_idx },
            .valid = true,
            .new_glow = 1.0f,          /* born flashing */
        };
    }
    return true;
}

/*
 * dt_insert_point — drop one new point into the mesh, as the four steps:
 * find the triangles it breaks, find the hole's outline, remove the broken
 * ones, fill the hole. Returns false only if storage would overflow.
 */
static bool dt_insert_point(Delaunay *d, int x, int y)
{
    if (d->n_points >= MAX_POINTS) return false;

    int p_idx = d->n_points;                     /* add the new point */
    d->points[p_idx] = (Point){ x, y };
    d->n_points++;
    Point P = d->points[p_idx];

    bool is_bad[MAX_TRIS] = { false };
    find_bad_triangles(d, P, is_bad);            /* 1 */

    CavityEdge edges[MAX_EDGE_BUF];
    int n_edges = collect_cavity_edges(d, is_bad, edges);  /* 2 */

    invalidate_triangles(d, is_bad);             /* 3 */

    if (!retriangulate_cavity(d, edges, n_edges, p_idx))   /* 4 */
        return false;

    d->n_real++;
    return true;
}

/*
 * dt_step — drop the next point if the cooldown has run out. Returns true if
 * a point actually went in, false if we're still waiting or already done.
 */
static bool dt_step(Delaunay *d)
{
    if (d->n_real >= d->target) return false;
    if (d->insert_cooldown > 0) {
        d->insert_cooldown--;
        return false;
    }
    int x, y;
    dt_pick_real_point(d, &x, &y);
    bool ok = dt_insert_point(d, x, y);
    d->insert_cooldown = INSERT_HOLD_TICKS_DEF;
    return ok;
}

/* ===================================================================== */
/* §6  scene — orchestration: the one place the layers combine            */
/* ===================================================================== */

/*
 * Where the build is in its life:
 *   BUILDING — still dropping points, one per cooldown, until it hits target.
 *   DONE     — finished; just hold the picture. Nothing restarts on its own;
 *              the user kicks off a new build with r, or n/p to switch style.
 */
typedef enum {
    SCENE_BUILDING = 0,
    SCENE_DONE     = 1,
} SceneState;

/*
 * Control — every knob the user can turn, in one place. Each field is set by
 * a key, never by the algorithm. The min/max for each lives in §1, and
 * app_handle_key clamps to them.
 */
typedef struct {
    int  preset_idx;          /* n/p  which style (index into presets[])      */
    int  insert_hold_ticks;   /* +/-  build speed (ticks between points)      */
    int  sim_fps;             /* [ ]  tick rate (Hz)                          */
    bool paused;              /* space  freeze the build                      */
} Control;

/*
 * Scene — the whole thing, with the three concerns kept separate:
 *   d      what's being built (the mesh and its progress)
 *   ctrl   how the user is driving it (the knobs)
 *   state  whether it's still building or done
 */
typedef struct {
    Delaunay    d;
    Control     ctrl;
    SceneState  state;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    const Preset *p = &presets[s->ctrl.preset_idx];
    preset_apply(s->ctrl.preset_idx);             /* load this style's colours */
    dt_reset(&s->d, mw, mh, p->n_points);         /* empty mesh, this density  */
    s->d.insert_cooldown = s->ctrl.insert_hold_ticks;
    s->state = SCENE_BUILDING;
}

/* scene_init — first-time setup. Picks the knob defaults once; every later
 * rebuild or resize goes through scene_reset, which leaves the knobs alone. */
static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->ctrl.preset_idx        = 0;
    s->ctrl.insert_hold_ticks = INSERT_HOLD_TICKS_DEF;
    s->ctrl.sim_fps           = SIM_FPS_DEFAULT;
    s->ctrl.paused            = false;
    scene_reset(s, mw, mh);
}

/* scene_regrow — start a fresh build at the current size, keeping the chosen
 * style and speed. Only ever called from a keypress (r, or n/p). */
static void scene_regrow(Scene *s)
{
    scene_reset(s, s->d.w, s->d.h);
}

/*
 * scene_tick — the single spot where everything moves forward each tick, in
 * this fixed order: bail if paused, fade the flashes, drop a point if it's
 * time, flip to DONE once we've hit target. Nothing else changes state.
 * dt is the fixed timestep.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    fx_decay(&s->d, dt);                   /* keep fading even when DONE so the
                                            * very last flash gets to settle    */

    if (s->state == SCENE_DONE) return;

    /* Pull the cooldown down if the user just sped things up, so a +/- press
     * takes hold on the very next point instead of after the old wait. */
    if (s->d.insert_cooldown > s->ctrl.insert_hold_ticks)
        s->d.insert_cooldown = s->ctrl.insert_hold_ticks;
    dt_step(&s->d);

    if (s->d.n_real >= s->d.target)
        s->state = SCENE_DONE;
}

/* ===================================================================== */
/* §7  render — state → ASCII (pure reads of the simulation)              */
/* ===================================================================== */

/*
 * Screen — the terminal's size, remembered so we don't re-ask every frame.
 * It only changes when the window is resized (SIGWINCH → screen_resize
 * re-reads it), but every frame needs it to centre the mesh and place the HUD
 * bars. Measured in character cells.
 */
typedef struct {
    int cols;   /* width  in character columns */
    int rows;   /* height in character rows    */
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
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * edge_glyph — pick the character that best matches an edge's slant: '-' for
 * mostly-flat, '|' for mostly-upright, '\\' or '/' for the two diagonals.
 * Based on the edge's overall direction so the whole line uses one character.
 */
static char edge_glyph(int dx, int dy)
{
    int adx = dx < 0 ? -dx : dx;
    int ady = dy < 0 ? -dy : dy;
    if (adx > 2 * ady) return '-';
    if (ady > 2 * adx) return '|';
    return ((long)dx * (long)dy > 0) ? '\\' : '/';
}

/*
 * draw_edge_segment — walk a straight line from (x0,y0) to (x1,y1) cell by
 * cell, stamping the glyph on each one that's on screen. The caller sets the
 * colour first, so we set it once per edge instead of once per cell.
 */
static void draw_edge_segment(int x0, int y0, int x1, int y1,
                              int gx0, int gy0, int cols, int rows,
                              char glyph)
{
    int dx =  abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (1) {
        int sx_screen = gx0 + x;
        int sy_screen = gy0 + y;
        if (sx_screen >= 0 && sx_screen < cols
            && sy_screen >= 0 && sy_screen < rows) {
            mvaddch(sy_screen, sx_screen, (chtype)(unsigned char)glyph);
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

/*
 * mesh_origin — where on screen the mesh's (0,0) corner goes, picked so the
 * w×h mesh sits centred in the space between the top status bar and the
 * bottom hint bar. Clamped so it never paints over either bar.
 */
static void mesh_origin(const Delaunay *d, int cols, int rows,
                        int *gx0, int *gy0)
{
    *gx0 = (cols - d->w) / 2;
    *gy0 = ((rows - 2) - d->h) / 2 + 1;   /* leave row 0 and the last row free */
    if (*gx0 < 0) *gx0 = 0;
    if (*gy0 < 1) *gy0 = 1;
}

/*
 * draw_mesh_edges — first pass: draw the three sides of every real triangle.
 * Scaffold triangles are skipped; flashing ones get the bright accent colour.
 * Shared edges get drawn twice, which is harmless.
 */
static void draw_mesh_edges(const Scene *s, int gx0, int gy0,
                            int cols, int rows)
{
    const Delaunay *d = &s->d;
    for (int i = 0; i < d->n_tris; i++) {
        const Triangle *t = &d->triangles[i];
        if (!t->valid || !triangle_is_real(t)) continue;

        int pair = triangle_is_flashing(t) ? PAIR_FLASH : PAIR_EDGE;
        int attr = triangle_is_flashing(t) ? A_BOLD     : A_NORMAL;

        attron(COLOR_PAIR(pair) | attr);
        for (int e = 0; e < 3; e++) {
            const Point *a = &d->points[t->p[e]];
            const Point *b = &d->points[t->p[(e + 1) % 3]];
            char g = edge_glyph(b->x - a->x, b->y - a->y);
            draw_edge_segment(a->x, a->y, b->x, b->y,
                              gx0, gy0, cols, rows, g);
        }
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* draw_seed_points — second pass: stamp each seed point's character on top of
 * the edges, in the style's point colour. */
static void draw_seed_points(const Scene *s, int gx0, int gy0,
                             int cols, int rows)
{
    const Delaunay *d = &s->d;
    char point_glyph = presets[s->ctrl.preset_idx].point_glyph;

    attron(COLOR_PAIR(PAIR_POINT) | A_BOLD);
    for (int i = SUPER_TRI_VERTS; i < d->n_points; i++) {
        int sx = gx0 + d->points[i].x;
        int sy = gy0 + d->points[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)point_glyph);
    }
    attroff(COLOR_PAIR(PAIR_POINT) | A_BOLD);
}

/* scene_draw — draw the mesh in two passes: edges underneath, points on top. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    mesh_origin(&s->d, cols, rows, &gx0, &gy0);

    draw_mesh_edges(s, gx0, gy0, cols, rows);
    draw_seed_points(s, gx0, gy0, cols, rows);
}

/*
 * hud_bar — paint one full-width coloured bar on `row` and write `buf` on it,
 * cut off at the screen width so a long line can't wrap down onto the mesh.
 * Used for both the top status bar and the bottom key hint.
 */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    if (row < 0 || cols < 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* count_real_triangles — how many live triangles are part of the real mesh
 * (scaffold ones not counted). This is the tris:N number in the HUD. */
static int count_real_triangles(const Delaunay *d)
{
    int n = 0;
    for (int i = 0; i < d->n_tris; i++)
        if (d->triangles[i].valid && triangle_is_real(&d->triangles[i]))
            n++;
    return n;
}

static void screen_draw(const Screen *sc, const Scene *s, double fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Delaunay *d = &s->d;
    const Control  *c = &s->ctrl;
    const char *state_str =
        c->paused                    ? "PAUSED"   :
        (s->state == SCENE_BUILDING) ? "BUILDING" :
                                       "DONE";

    int n_visible = count_real_triangles(d);

    /* Top bar: title, current style, progress, and timing — one line. */
    char data[200];
    snprintf(data, sizeof data,
             " DELAUNAY B-W  %-8s  %s (%d/%d)  pts:%d/%d  tris:%d  "
             "hold:%-3d  %5.1f fps  %3d Hz ",
             state_str, presets[c->preset_idx].name,
             c->preset_idx + 1, N_PRESETS,
             d->n_real, d->target, n_visible,
             c->insert_hold_ticks, fps, c->sim_fps);

    /* Bottom bar: every key you can press. */
    static const char *keys =
        " q:quit  spc:pause  n/p:preset  r:reset  +/-:speed  [/]:Hz ";

    hud_bar(0,            sc->cols, PAIR_HUD,  data);
    hud_bar(sc->rows - 1, sc->cols, PAIR_HINT, keys);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app — PERFORMANCE loop (fixed timestep) + signals + input          */
/* ===================================================================== */

/*
 * App — the whole program in one struct: the scene, the screen, the mesh size
 * we picked, and two flags the signal handlers set. It's a single global
 * because signal handlers can't be handed a pointer — they reach the program
 * through this one well-known spot. It's the only global; everything else is
 * passed around by pointer.
 */
typedef struct {
    Scene                 scene;          /* the mesh + its controls          */
    Screen                screen;         /* cached terminal size             */
    int                   map_w, map_h;   /* mesh size, fit to the screen     */
    volatile sig_atomic_t running;        /* set to 0 to quit (SIGINT/TERM).
                                           * this type is the only one safe to
                                           * write inside a signal handler     */
    volatile sig_atomic_t need_resize;    /* set to 1 on a resize (SIGWINCH)   */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 2;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene   *s = &app->scene;
    Control *c = &s->ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     c->paused = !c->paused; break;

    case 'n': case 'N':   /* next preset */
        c->preset_idx = (c->preset_idx + 1) % N_PRESETS;
        scene_regrow(s);
        break;
    case 'p': case 'P':   /* previous preset */
        c->preset_idx = (c->preset_idx + N_PRESETS - 1) % N_PRESETS;
        scene_regrow(s);
        break;

    case 'r': case 'R':   /* rebuild, same preset */
        scene_regrow(s);
        break;
    case '=': case '+':   /* faster: shorter wait between points */
        c->insert_hold_ticks = clamp_int(c->insert_hold_ticks / 2,
                                         INSERT_HOLD_TICKS_MIN,
                                         INSERT_HOLD_TICKS_MAX);
        break;
    case '-':             /* slower: longer wait between points */
        c->insert_hold_ticks = clamp_int(c->insert_hold_ticks * 2,
                                         INSERT_HOLD_TICKS_MIN,
                                         INSERT_HOLD_TICKS_MAX);
        break;
    case ']':
        c->sim_fps = clamp_int(c->sim_fps + SIM_FPS_STEP,
                               SIM_FPS_MIN, SIM_FPS_MAX);
        break;
    case '[':
        c->sim_fps = clamp_int(c->sim_fps - SIM_FPS_STEP,
                               SIM_FPS_MIN, SIM_FPS_MAX);
        break;

    default: break;
    }
    return true;
}

/* install_signals — wire up quitting (SIGINT/SIGTERM), resizing (SIGWINCH),
 * and an atexit cleanup so even a crash hands the terminal back working. */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    install_signals();

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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

        int64_t tick_ns = TICK_NS(app->scene.ctrl.sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
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

        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
