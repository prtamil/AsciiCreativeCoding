/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * soft_body.c — Jelly Bodies  (Position-Based Dynamics)
 *
 * Multiple soft cubes and spheres with full pairwise collision.
 * All collision types handled by the same generic boundary-polygon test:
 *   cube-cube, cube-sphere, sphere-cube, sphere-sphere,
 *   cube-floor, sphere-floor (floor/wall: per node clamp in blob_step).
 *
 * Physics: Position-Based Dynamics
 *   Verlet predict → project distance constraints × N → clamp walls
 *   Unconditionally stable. Stiffness = iteration count.
 *
 * Collision response (PBD, per penetrating node):
 *   node of A pushed outward by depth/2 along nearest-edge normal
 *   two boundary nodes of B pushed inward by depth/4 each (Newton's 3rd)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/soft_body.c -o soft_body -lncurses -lm
 *
 * Keys:
 *   c   add cube     s   add sphere    x   remove last body
 *   q/ESC quit       p   pause         r   reset (1 cube + 1 sphere)
 *   g   gravity      i/I iterations±   t/T theme
 *
 * Sections: §1 config  §2 clock  §3 color  §4 node/blob
 *           §5 physics  §6 collision  §7 scene  §8 draw  §9 screen  §10 app
 */

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

#define ROWS_MAX      128
#define COLS_MAX      512

/* Cube */
#define CUBE_W        6
#define CUBE_H        6
#define CUBE_SP       3.0f
#define CUBE_NODES   (CUBE_W * CUBE_H)   /* 36 */

/* Sphere */
#define SPH_RING      12          /* ring nodes; node 0 = centre */
#define SPH_NODES    (SPH_RING + 1)
#define SPH_R         5.0f        /* visual col-radius; y-extent = 2×SPH_R */

/* Generic blob limits */
#define BLOB_NODES_MAX   50
#define BLOB_CONS_MAX   250
#define BLOB_BND_MAX     50

/* Scene */
#define MAX_BLOBS    16

/* Color slots: 0-2 = cube family (cool), 3-5 = sphere family (warm) */
#define N_BSLOTS      6
#define CP_BSURF(i)  (1 + (i))             /* pairs 1-6  */
#define CP_BFILL(i)  (1 + N_BSLOTS + (i)) /* pairs 7-12 */
#define CP_FLOOR     (1 + 2*N_BSLOTS)      /* 13 */
#define CP_HUD       (2 + 2*N_BSLOTS)      /* 14 */

/* PBD */
#define PBD_ITERS_DEF    6
#define PBD_ITERS_MIN    1
#define PBD_ITERS_MAX   20
#define STRUCT_K         1.0f
#define SHEAR_K          0.8f
#define COLL_ITERS       2

/* Physics (per substep) */
#define GRAVITY_DEF      0.06f
#define DAMPING_DEF      0.985f
#define FLOOR_REST       0.12f
#define WALL_REST        0.10f
#define FLOOR_FRIC       0.82f

#define PHY_TO_ROW(y)   ((int)((y) * 0.5f + 0.5f))
#define PHY_TO_COL(x)   ((int)((x)       + 0.5f))

#define STEPS_DEF        3
#define SIM_FPS         20
#define NS_PER_SEC      1000000000LL
#define TICK_NS(f)     (NS_PER_SEC / (f))

#define N_THEMES         5

/* Simple LCG for random placement */
static uint32_t g_rng = 0xdeadbeef;
static float rng_f(void) {
    g_rng = g_rng * 1664525u + 1013904223u;
    return (g_rng >> 8) * (1.f / 16777216.f);
}

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

typedef struct {
    short surf[N_BSLOTS];   /* surface fg per slot (0-2 cube, 3-5 sphere) */
    short fill[N_BSLOTS];   /* fill fg per slot */
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Ocean */
    { { 51,  45,  87,  214, 196, 206 },
      { 17,  17,  55,  130,  88, 125 }, "Ocean"  },
    /* Matrix */
    { { 82,  46, 118,  196, 160, 124 },
      { 22,  22,  28,   52,  52,  52 }, "Matrix" },
    /* Nebula */
    { {213, 177, 147,   87, 123, 129 },
      { 93,  61,  55,   55,  89,  93 }, "Nebula" },
    /* Fire */
    { {226, 220, 214,  171, 135,  99 },
      {100,  94,  88,   93,  57,  57 }, "Fire"   },
    /* Mono */
    { {255, 245, 235,  230, 220, 210 },
      {240, 230, 220,  215, 205, 195 }, "Mono"   },
};

static bool g_has_256;
static int  g_theme = 0;

static void theme_apply(int ti)
{
    const Theme *t = &k_themes[ti];
    for (int i = 0; i < N_BSLOTS; i++) {
        if (g_has_256) {
            init_pair(CP_BSURF(i), t->surf[i], COLOR_BLACK);
            init_pair(CP_BFILL(i), t->fill[i], COLOR_BLACK);
        } else {
            short cs = (i < 3) ? COLOR_CYAN   : COLOR_RED;
            short cf = (i < 3) ? COLOR_CYAN   : COLOR_RED;
            if (i == 1) cs = COLOR_BLUE;
            if (i == 4) cs = COLOR_YELLOW;
            init_pair(CP_BSURF(i), cs, COLOR_BLACK);
            init_pair(CP_BFILL(i), cf, COLOR_BLACK);
        }
    }
    init_pair(CP_FLOOR, g_has_256 ? 244 : COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_HUD,   g_has_256 ? 255 : COLOR_WHITE,
                        g_has_256 ? 236 : COLOR_BLACK);
}

/* ===================================================================== */
/* §4  node / constraint / blob                                           */
/* ===================================================================== */

typedef struct {
    float x, y;    /* current position  */
    float px, py;  /* previous position (Verlet: vel ≈ pos − prev) */
} Node;

typedef struct {
    int   a, b;
    float rest;
    float k;
} Con;

typedef enum { BKIND_CUBE, BKIND_SPHERE } BKind;

typedef struct {
    Node   nodes[BLOB_NODES_MAX];
    int    n_nodes;
    Con    cons[BLOB_CONS_MAX];
    int    n_cons;
    int    bnd[BLOB_BND_MAX];  /* boundary indices, CCW order */
    int    n_bnd;
    int    surf_cp, fill_cp;
    BKind  kind;
} Blob;

static void blob_add_con(Blob *bl, int a, int b, float k)
{
    if (bl->n_cons >= BLOB_CONS_MAX) return;
    float dx = bl->nodes[b].x - bl->nodes[a].x;
    float dy = bl->nodes[b].y - bl->nodes[a].y;
    bl->cons[bl->n_cons++] = (Con){ a, b, sqrtf(dx*dx+dy*dy), k };
}

/* ── cube ──────────────────────────────────────────────────────────── */
static void blob_build_cube(Blob *bl, float ox, float oy, int scp, int fcp)
{
    memset(bl, 0, sizeof *bl);
    bl->surf_cp = scp;  bl->fill_cp = fcp;  bl->kind = BKIND_CUBE;
    bl->n_nodes = CUBE_NODES;

    for (int r = 0; r < CUBE_H; r++)
        for (int c = 0; c < CUBE_W; c++) {
            int i = r*CUBE_W + c;
            bl->nodes[i].x = bl->nodes[i].px = ox + c*CUBE_SP;
            bl->nodes[i].y = bl->nodes[i].py = oy + r*CUBE_SP;
        }

    /* Structural */
    for (int r = 0; r < CUBE_H; r++)
        for (int c = 0; c < CUBE_W; c++) {
            int i = r*CUBE_W + c;
            if (c+1 < CUBE_W) blob_add_con(bl, i, i+1,       STRUCT_K);
            if (r+1 < CUBE_H) blob_add_con(bl, i, i+CUBE_W,  STRUCT_K);
        }
    /* Shear */
    for (int r = 0; r < CUBE_H-1; r++)
        for (int c = 0; c < CUBE_W-1; c++) {
            int i = r*CUBE_W + c;
            blob_add_con(bl, i,   i+CUBE_W+1, SHEAR_K);
            blob_add_con(bl, i+1, i+CUBE_W,   SHEAR_K);
        }

    /* Boundary ring CCW */
    int *bnd = bl->bnd;  bl->n_bnd = 0;
    for (int c = 0; c < CUBE_W;    c++) bnd[bl->n_bnd++] = c;
    for (int r = 1; r < CUBE_H;    r++) bnd[bl->n_bnd++] = r*CUBE_W + (CUBE_W-1);
    for (int c = CUBE_W-2; c >= 0; c--) bnd[bl->n_bnd++] = (CUBE_H-1)*CUBE_W + c;
    for (int r = CUBE_H-2; r > 0;  r--) bnd[bl->n_bnd++] = r*CUBE_W;
}

/* ── sphere ─────────────────────────────────────────────────────────── */
static void blob_build_sphere(Blob *bl, float cx, float cy, int scp, int fcp)
{
    memset(bl, 0, sizeof *bl);
    bl->surf_cp = scp;  bl->fill_cp = fcp;  bl->kind = BKIND_SPHERE;
    bl->n_nodes = SPH_NODES;

    /* node 0 = centre */
    bl->nodes[0].x = bl->nodes[0].px = cx;
    bl->nodes[0].y = bl->nodes[0].py = cy;

    /* nodes 1..SPH_RING = ring (physics ellipse → circular display) */
    for (int i = 0; i < SPH_RING; i++) {
        float a = 6.2831853f * i / SPH_RING;
        int ni = i + 1;
        bl->nodes[ni].x = bl->nodes[ni].px = cx + SPH_R * cosf(a);
        bl->nodes[ni].y = bl->nodes[ni].py = cy + SPH_R * 2.f * sinf(a);
    }

    for (int i = 0; i < SPH_RING; i++)
        blob_add_con(bl, 1+i, 1+(i+1)%SPH_RING, STRUCT_K); /* hoop  */
    for (int i = 0; i < SPH_RING; i++)
        blob_add_con(bl, 0, 1+i, STRUCT_K);                  /* spoke */
    for (int i = 0; i < SPH_RING/2; i++)
        blob_add_con(bl, 1+i, 1+i+SPH_RING/2, SHEAR_K);     /* diameter */

    for (int i = 0; i < SPH_RING; i++) bl->bnd[i] = 1+i;
    bl->n_bnd = SPH_RING;
}

/* ===================================================================== */
/* §5  physics                                                            */
/* ===================================================================== */

static int   g_rows, g_cols;
static bool  g_gravity_on = true;
static float g_gravity    = GRAVITY_DEF;
static float g_damping    = DAMPING_DEF;
static int   g_pbd_iters  = PBD_ITERS_DEF;

static void blob_step(Blob *bl)
{
    float ww = (float)g_cols - 1.f;
    float wh = (float)(g_rows - 2) * 2.f;

    /* 1. Verlet predict */
    for (int i = 0; i < bl->n_nodes; i++) {
        Node *nd = &bl->nodes[i];
        float vx = (nd->x - nd->px) * g_damping;
        float vy = (nd->y - nd->py) * g_damping;
        nd->px = nd->x;  nd->py = nd->y;
        nd->x += vx;
        nd->y += vy + (g_gravity_on ? g_gravity : 0.f);
    }

    /* 2. Project constraints + clamp to world */
    for (int iter = 0; iter < g_pbd_iters; iter++) {
        for (int c = 0; c < bl->n_cons; c++) {
            Node *a = &bl->nodes[bl->cons[c].a];
            Node *b = &bl->nodes[bl->cons[c].b];
            float dx = b->x - a->x, dy = b->y - a->y;
            float d  = sqrtf(dx*dx + dy*dy);
            if (d < 1e-6f) continue;
            float corr = (d - bl->cons[c].rest) / d * bl->cons[c].k * 0.5f;
            a->x += dx*corr;  a->y += dy*corr;
            b->x -= dx*corr;  b->y -= dy*corr;
        }
        for (int i = 0; i < bl->n_nodes; i++) {
            Node *nd = &bl->nodes[i];
            if (nd->x < 0.f)  nd->x = 0.f;
            if (nd->x > ww)   nd->x = ww;
            if (nd->y < 0.f)  nd->y = 0.f;
            if (nd->y > wh)   nd->y = wh;
        }
    }

    /* 3. Velocity correction at boundaries */
    for (int i = 0; i < bl->n_nodes; i++) {
        Node *nd = &bl->nodes[i];
        float vx = nd->x - nd->px, vy = nd->y - nd->py;
        if (nd->y >= wh-0.01f && vy > 0.f) {
            nd->py = nd->y + vy * FLOOR_REST;
            nd->px = nd->x - vx * FLOOR_FRIC;
        }
        if (nd->y <= 0.01f      && vy < 0.f) nd->py = nd->y + vy * WALL_REST;
        if (nd->x <= 0.01f      && vx < 0.f) nd->px = nd->x + vx * WALL_REST;
        if (nd->x >= ww - 0.01f && vx > 0.f) nd->px = nd->x - vx * WALL_REST;
    }
}

/* ===================================================================== */
/* §6  collision  (works for any pair: cube-cube, sphere-sphere, mixed)   */
/* ===================================================================== */

static bool point_in_blob(const Blob *bl, float px, float py)
{
    int crossings = 0, nb = bl->n_bnd;
    for (int i = 0; i < nb; i++) {
        int j = (i+1) % nb;
        float ax = bl->nodes[bl->bnd[i]].x, ay = bl->nodes[bl->bnd[i]].y;
        float bx = bl->nodes[bl->bnd[j]].x, by = bl->nodes[bl->bnd[j]].y;
        if ((ay <= py && by > py) || (by <= py && ay > py)) {
            float t = (py - ay) / (by - ay);
            if (px < ax + t*(bx - ax)) crossings++;
        }
    }
    return (crossings & 1) != 0;
}

static void nearest_edge(const Blob *bl, float px, float py,
                          float *nx, float *ny, float *depth,
                          int *ea, int *eb)
{
    float best = 1e9f;
    *ea = bl->bnd[0]; *eb = bl->bnd[1];
    *nx = 1.f; *ny = 0.f; *depth = 0.f;
    int nb = bl->n_bnd;
    for (int i = 0; i < nb; i++) {
        int j = (i+1) % nb;
        float ax = bl->nodes[bl->bnd[i]].x, ay = bl->nodes[bl->bnd[i]].y;
        float bx = bl->nodes[bl->bnd[j]].x, by = bl->nodes[bl->bnd[j]].y;
        float edx = bx-ax, edy = by-ay;
        float len2 = edx*edx + edy*edy;
        if (len2 < 1e-8f) continue;
        float t = ((px-ax)*edx + (py-ay)*edy) / len2;
        t = t<0.f?0.f:t>1.f?1.f:t;
        float cx = ax+t*edx, cy = ay+t*edy;
        float dx = px-cx, dy = py-cy;
        float d  = sqrtf(dx*dx + dy*dy);
        if (d < best) {
            best = d; *depth = d;
            *nx = d > 1e-6f ? dx/d : 0.f;
            *ny = d > 1e-6f ? dy/d : 1.f;
            *ea = bl->bnd[i]; *eb = bl->bnd[j];
        }
    }
}

/* Push nodes of A that are inside B back out; deform B's boundary inward */
static void collide_one_way(Blob *a, Blob *b)
{
    for (int i = 0; i < a->n_nodes; i++) {
        float px = a->nodes[i].x, py = a->nodes[i].y;
        if (!point_in_blob(b, px, py)) continue;
        float nx, ny, depth;  int ea, eb;
        nearest_edge(b, px, py, &nx, &ny, &depth, &ea, &eb);
        float push_a = (depth + 0.5f) * 0.5f;
        float push_b =  depth         * 0.25f;
        a->nodes[i].x   -= nx * push_a;
        a->nodes[i].y   -= ny * push_a;
        b->nodes[ea].x  += nx * push_b;
        b->nodes[ea].y  += ny * push_b;
        b->nodes[eb].x  += nx * push_b;
        b->nodes[eb].y  += ny * push_b;
    }
}

static void blob_collide(Blob *a, Blob *b)
{
    for (int iter = 0; iter < COLL_ITERS; iter++) {
        collide_one_way(a, b);
        collide_one_way(b, a);
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

static Blob g_blobs[MAX_BLOBS];
static int  g_nblobs  = 0;
static int  g_ncubes  = 0;   /* total ever added (for color cycling) */
static int  g_nsphs   = 0;
static bool g_paused  = false;
static int  g_steps   = STEPS_DEF;
static long g_tick    = 0;

/* Assign a color slot: cubes cycle through 0-2, spheres through 3-5 */
static void blob_color(BKind kind, int count, int *scp, int *fcp)
{
    int slot = (kind == BKIND_CUBE) ? (count % 3) : (3 + count % 3);
    *scp = CP_BSURF(slot);
    *fcp = CP_BFILL(slot);
}

static bool scene_add_cube(void)
{
    if (g_nblobs >= MAX_BLOBS) return false;
    float ww = (float)g_cols;
    float wh = (float)(g_rows - 2) * 2.f;
    float half_w = (CUBE_W-1) * CUBE_SP * 0.5f;
    float ox = ww * (0.15f + rng_f() * 0.70f) - half_w;
    /* clamp so cube stays inside screen */
    if (ox < 1.f) ox = 1.f;
    if (ox + (CUBE_W-1)*CUBE_SP > ww-2.f) ox = ww - 2.f - (CUBE_W-1)*CUBE_SP;
    float oy = 1.f;   /* near top */
    (void)wh;
    int scp, fcp;
    blob_color(BKIND_CUBE, g_ncubes, &scp, &fcp);
    blob_build_cube(&g_blobs[g_nblobs], ox, oy, scp, fcp);
    g_nblobs++;  g_ncubes++;
    return true;
}

static bool scene_add_sphere(void)
{
    if (g_nblobs >= MAX_BLOBS) return false;
    float ww = (float)g_cols;
    float cx = ww * (0.15f + rng_f() * 0.70f);
    if (cx < SPH_R + 1.f)       cx = SPH_R + 1.f;
    if (cx > ww - SPH_R - 1.f)  cx = ww - SPH_R - 1.f;
    float cy = SPH_R * 2.f + 1.f;   /* near top */
    int scp, fcp;
    blob_color(BKIND_SPHERE, g_nsphs, &scp, &fcp);
    blob_build_sphere(&g_blobs[g_nblobs], cx, cy, scp, fcp);
    g_nblobs++;  g_nsphs++;
    return true;
}

static void scene_remove_last(void)
{
    if (g_nblobs > 0) g_nblobs--;
}

static void scene_init(void)
{
    g_tick = 0;  g_nblobs = 0;  g_ncubes = 0;  g_nsphs = 0;
    float ww = (float)g_cols;
    float wh = (float)(g_rows - 2) * 2.f;
    float cx = ww * 0.5f;

    /* Cube sitting on floor, centred */
    float cube_w = (CUBE_W-1) * CUBE_SP;
    float cube_h = (CUBE_H-1) * CUBE_SP;
    int scp, fcp;
    blob_color(BKIND_CUBE, g_ncubes, &scp, &fcp);
    blob_build_cube(&g_blobs[g_nblobs],
                    cx - cube_w*0.5f, wh - cube_h - 0.5f, scp, fcp);
    g_nblobs++;  g_ncubes++;

    /* Sphere falling from top-centre */
    blob_color(BKIND_SPHERE, g_nsphs, &scp, &fcp);
    blob_build_sphere(&g_blobs[g_nblobs], cx, SPH_R*2.f+1.f, scp, fcp);
    g_nblobs++;  g_nsphs++;
}

static void scene_step(void)
{
    for (int s = 0; s < g_steps; s++) {
        /* Integrate each blob (includes floor/wall constraints) */
        for (int i = 0; i < g_nblobs; i++)
            blob_step(&g_blobs[i]);
        /* All pairwise collisions */
        for (int i = 0; i < g_nblobs; i++)
            for (int j = i+1; j < g_nblobs; j++)
                blob_collide(&g_blobs[i], &g_blobs[j]);
    }
    g_tick++;
}

/* ===================================================================== */
/* §8  draw                                                               */
/* ===================================================================== */

static int g_mincol[ROWS_MAX], g_maxcol[ROWS_MAX];

static void scanfill_edge(int x0,int y0,int x1,int y1, int fr, int cols)
{
    int dx=abs(x1-x0), sx=x0<x1?1:-1;
    int dy=-abs(y1-y0), sy=y0<y1?1:-1;
    int err=dx+dy, cx=x0, cy=y0;
    for (;;) {
        if (cy>=0&&cy<fr&&cx>=0&&cx<cols){
            if(cx<g_mincol[cy])g_mincol[cy]=cx;
            if(cx>g_maxcol[cy])g_maxcol[cy]=cx;
        }
        if(cx==x1&&cy==y1) break;
        int e2=2*err;
        if(e2>=dy){if(cx==x1)break;err+=dy;cx+=sx;}
        if(e2<=dx){if(cy==y1)break;err+=dx;cy+=sy;}
    }
}

static void bresenham(int x0,int y0,int x1,int y1, int fr,int cols, chtype ch)
{
    int dx=abs(x1-x0), sx=x0<x1?1:-1;
    int dy=-abs(y1-y0), sy=y0<y1?1:-1;
    int err=dx+dy, cx=x0, cy=y0;
    for (;;) {
        if(cy>=0&&cy<fr&&cx>=0&&cx<cols) mvaddch(cy,cx,ch);
        if(cx==x1&&cy==y1) break;
        int e2=2*err;
        if(e2>=dy){if(cx==x1)break;err+=dy;cx+=sx;}
        if(e2<=dx){if(cy==y1)break;err+=dx;cy+=sy;}
    }
}

static void draw_blob(const Blob *bl, int floor_row, int cols)
{
    /* Scan-fill interior */
    for (int r=0; r<floor_row; r++){g_mincol[r]=cols; g_maxcol[r]=-1;}
    for (int i=0; i<bl->n_bnd; i++) {
        int j=(i+1)%bl->n_bnd;
        const Node *a=&bl->nodes[bl->bnd[i]], *b=&bl->nodes[bl->bnd[j]];
        scanfill_edge(PHY_TO_COL(a->x),PHY_TO_ROW(a->y),
                      PHY_TO_COL(b->x),PHY_TO_ROW(b->y), floor_row, cols);
    }
    attron(COLOR_PAIR(bl->fill_cp));
    for (int r=0; r<floor_row; r++) {
        if (g_maxcol[r] <= g_mincol[r]) continue;
        for (int c=g_mincol[r]+1; c<g_maxcol[r]; c++)
            mvaddch(r, c, ':');
    }
    attroff(COLOR_PAIR(bl->fill_cp));

    /* Constraint wireframe */
    attron(COLOR_PAIR(bl->surf_cp));
    for (int ci=0; ci<bl->n_cons; ci++) {
        const Node *a=&bl->nodes[bl->cons[ci].a];
        const Node *b=&bl->nodes[bl->cons[ci].b];
        int x0=PHY_TO_COL(a->x),y0=PHY_TO_ROW(a->y);
        int x1=PHY_TO_COL(b->x),y1=PHY_TO_ROW(b->y);
        int adx=abs(x1-x0), ady=abs(y1-y0);
        chtype ch;
        if      (adx==0)              ch='|';
        else if (ady==0)              ch='-';
        else if ((x1-x0)*(y1-y0)>0)  ch='\\';
        else                          ch='/';
        bresenham(x0,y0,x1,y1, floor_row, cols, ch);
    }
    attroff(COLOR_PAIR(bl->surf_cp));

    /* Boundary nodes */
    attron(COLOR_PAIR(bl->surf_cp)|A_BOLD);
    for (int i=0; i<bl->n_bnd; i++) {
        const Node *nd=&bl->nodes[bl->bnd[i]];
        int cr=PHY_TO_ROW(nd->y), cc=PHY_TO_COL(nd->x);
        if (cr>=0&&cr<floor_row&&cc>=0&&cc<cols)
            mvaddch(cr,cc,'O');
    }
    attroff(COLOR_PAIR(bl->surf_cp)|A_BOLD);
}

static void scene_draw(void)
{
    erase();
    int rows=g_rows, cols=g_cols, floor_row=rows-2;

    attron(COLOR_PAIR(CP_FLOOR));
    for (int c=0; c<cols; c++) mvaddch(floor_row,c,'=');
    attroff(COLOR_PAIR(CP_FLOOR));

    /* Draw all blobs back to front */
    for (int i=0; i<g_nblobs; i++)
        draw_blob(&g_blobs[i], floor_row, cols);

    /* HUD */
    int ncubes=0, nsphs=0;
    for (int i=0; i<g_nblobs; i++) {
        if (g_blobs[i].kind==BKIND_CUBE) ncubes++; else nsphs++;
    }

    attron(COLOR_PAIR(CP_HUD)|A_BOLD);
    mvprintw(floor_row, 0,
        " [c]cube [s]sphere [x]del  cubes:%-2d sphs:%-2d / %-2d"
        "  iters:%-2d  grav:%s  tick:%-5ld  theme:%s  %s",
        ncubes, nsphs, MAX_BLOBS,
        g_pbd_iters, g_gravity_on?"on":"off",
        g_tick, k_themes[g_theme].name, g_paused?"[PAUSED]":"");
    attroff(COLOR_PAIR(CP_HUD)|A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    mvaddstr(rows-1, 0,
        "  [i/I]iters  [g]gravity  [t/T]theme  [r]reset  [p]pause  [q]quit");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_resize=0, g_quit=0;
static void on_sigwinch(int s){(void)s; g_resize=1;}
static void on_sigterm (int s){(void)s; g_quit=1;}

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr,TRUE); nodelay(stdscr,TRUE);
    curs_set(0); typeahead(-1);
    start_color(); g_has_256=(COLORS>=256);
    theme_apply(g_theme);
}
static void screen_resize(void)
{
    endwin(); refresh();
    getmaxyx(stdscr, g_rows, g_cols);
    if(g_rows>ROWS_MAX) g_rows=ROWS_MAX;
    if(g_cols>COLS_MAX) g_cols=COLS_MAX;
    g_resize=0;
}

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, on_sigwinch);
    signal(SIGTERM,  on_sigterm);
    signal(SIGINT,   on_sigterm);

    screen_init();
    getmaxyx(stdscr, g_rows, g_cols);
    if(g_rows>ROWS_MAX) g_rows=ROWS_MAX;
    if(g_cols>COLS_MAX) g_cols=COLS_MAX;

    scene_init();
    int64_t next_tick = clock_ns();

    while (!g_quit) {
        int ch;
        while ((ch=getch()) != ERR) {
            switch (ch) {
            case 'q': case 27:  g_quit=1; break;
            case 'p': case ' ': g_paused=!g_paused; break;
            case 'r': scene_init(); break;
            case 'c': scene_add_cube();   break;
            case 's': scene_add_sphere(); break;
            case 'x': scene_remove_last(); break;
            case 'g': g_gravity_on=!g_gravity_on; break;
            case 'i': if(g_pbd_iters<PBD_ITERS_MAX) g_pbd_iters++; break;
            case 'I': if(g_pbd_iters>PBD_ITERS_MIN) g_pbd_iters--; break;
            case 't': g_theme=(g_theme+1)%N_THEMES; theme_apply(g_theme); break;
            case 'T': g_theme=(g_theme+N_THEMES-1)%N_THEMES; theme_apply(g_theme); break;
            }
        }
        if (g_resize) { screen_resize(); scene_init(); }

        int64_t now=clock_ns();
        if (!g_paused && now>=next_tick) {
            scene_step();
            next_tick = now + TICK_NS(SIM_FPS);
        }
        scene_draw();
        wnoutrefresh(stdscr); doupdate();
        clock_sleep_ns(next_tick - clock_ns() - 1000000LL);
    }
    endwin();
    return 0;
}
