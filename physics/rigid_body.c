/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rigid_body.c — 2D Rigid Body Simulation (no rotation)
 *
 * Bodies: cube (AABB rectangle) · sphere (circle drawn, AABB physics)
 * Implicit floor at bottom of screen.  Nothing penetrates anything.
 *
 * Why AABB for spheres?
 *   Terminal chars are ~2× taller than wide.  The sphere is drawn as a
 *   circle with visual radius r (cols) × r (rows).  Its physics AABB is
 *   hw=r, hh=2r so that the bounding box aligns with the drawing exactly.
 *   Old circle-distance check used radius r for both axes → detected
 *   collision only when bodies were already r rows deep into each other.
 *
 * Collision detection: AABB overlap (minimum-penetration axis)
 * — same function for cube-cube, sphere-sphere, cube-sphere —
 *
 * Resolution — two separate passes per iteration:
 *   Pass A  positional correction  (ALWAYS, even when separating)
 *     corr = max(depth-SLOP, 0) * BAUMGARTE / (imA+imB)
 *     This is the critical fix: the old code only corrected positions when
 *     bodies were approaching (vn>0).  Bodies that spawned overlapping or
 *     that had zero relative velocity on the contact axis were never pushed
 *     apart and fell through each other.
 *   Pass B  velocity impulse  (only when approaching, vn>0)
 *     j = (1+e_eff)*vn / (imA+imB)
 *     e_eff = (vn > REST_THRESH) ? e : 0   ← no micro-bounce at rest
 *     Coulomb friction |jt| ≤ μ·j
 *
 * Floor / wall: full snap + impulse (no fraction — hard boundary)
 *   Runs for ALL bodies (sleeping included) to counter Baumgarte drift.
 *
 * Spawn: 8-attempt overlap check, reject if no clear spot found.
 *
 * Sleep  SLEEP_FRAMES quiet frames → frozen.  Woken by impulse > WAKE_IMP
 *        or by positional correction > WAKE_IMP.
 *
 * Keys  c add cube  s add sphere  x remove last  r reset
 *       p pause     g gravity     e/E restitution
 *       t/T theme   q quit
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra physics/rigid_body.c -o rigid_body -lncurses -lm
 *
 * Sections  §1 config  §2 clock  §3 color  §4 body  §5 framebuf
 *           §6 physics §7 scene  §8 draw   §9 screen §10 app
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

#define ROWS_MAX     128
#define COLS_MAX     512
#define MAX_BODIES    32
#define N_THEMES       5

/* physics */
#define GRAVITY      0.05f   /* downward accel per step                     */
#define REST_DEF     0.35f   /* default coefficient of restitution           */
#define REST_STEP    0.05f
#define FRICTION     0.35f   /* Coulomb μ                                    */
#define DAMPING      0.991f  /* per-step velocity multiplier                 */
#define MAX_SPEED    22.0f   /* velocity cap (tunneling prevention)          */

/* solver */
#define SOLVER_ITERS  10     /* iterations per step — higher = stiffer stack */
#define BAUMGARTE    0.50f   /* positional correction fraction (per iter)    */
#define SLOP         0.05f   /* penetration allowed before correction fires  */
#define REST_THRESH  0.20f   /* approach speed below this → e_eff = 0       */
/*                             kills micro-bounce when bodies settle          */

/* sleep */
#define SLEEP_VEL    0.07f   /* speed below which sleep counter ticks        */
#define SLEEP_FRAMES   30    /* consecutive quiet frames before sleep        */
#define WAKE_IMP     0.05f   /* min impulse / corr that wakes sleeping body  */

/* body sizes (physics units; terminal cell ratio ≈ 1 col : 2 rows)        */
#define CUBE_HW      7.0f    /* cube half-width                              */
#define CUBE_HH      5.0f    /* cube half-height                             */
#define SPH_R        4.0f    /* sphere visual radius in cols AND rows        */
/*                             sphere AABB: hw=SPH_R, hh=2*SPH_R            */

#define DENSITY      0.008f  /* mass = DENSITY × AABB area (4·hw·hh)        */

/* timing */
#define SIM_FPS       20
#define NS_PER_S  1000000000LL

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_S + t.tv_nsec;
}
static void sleep_ns(int64_t d)
{
    if (d <= 0) return;
    struct timespec t = { (time_t)(d / NS_PER_S), (long)(d % NS_PER_S) };
    nanosleep(&t, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

#define CP_FLOOR  9
#define CP_HUD   10

static const struct { short c[6]; const char *name; } k_themes[N_THEMES] = {
    {{ 196, 214, 226,  46,  51, 129 }, "Vivid"  },
    {{ 160, 172, 178,  34,  43,  91 }, "Muted"  },
    {{ 231, 220, 214, 208, 202, 196 }, "Fire"   },
    {{  46,  82, 118, 154, 190, 226 }, "Matrix" },
    {{  51,  45,  39,  33,  27,  21 }, "Ocean"  },
};
static bool g_256;
static int  g_theme = 0;

static void theme_apply(int ti)
{
    for (int i = 0; i < 6; i++) {
        short fg = g_256 ? k_themes[ti].c[i] : (short)(COLOR_RED + i % 6);
        init_pair(1 + i, fg, COLOR_BLACK);
    }
    init_pair(CP_FLOOR, g_256 ? 240 : COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_HUD,   g_256 ? 255 : COLOR_WHITE, g_256 ? 236 : COLOR_BLACK);
}

/* ===================================================================== */
/* §4  body                                                               */
/* ===================================================================== */

typedef enum { KIND_CUBE = 0, KIND_SPHERE } Kind;

/*
 * hw / hh are the AABB half-extents used for ALL physics.
 *   cube  : hw=CUBE_HW, hh=CUBE_HH
 *   sphere: hw=SPH_R,   hh=2*SPH_R   (aspect-corrected for terminal)
 */
typedef struct {
    Kind  kind;
    float x, y;        /* center; physics y increases downward          */
    float vx, vy;
    float hw, hh;      /* AABB half-extents                             */
    float mass, imass;
    int   cp;          /* ncurses color pair                            */
    int   sleep_cnt;
    bool  sleeping;
} Body;

static Body  g_b[MAX_BODIES];
static int   g_nb     = 0;
static int   g_ncubes = 0;
static int   g_nsphs  = 0;
static float g_rest   = REST_DEF;
static bool  g_grav   = true;
static bool  g_paused = false;
static long  g_tick   = 0;
static int   g_rows, g_cols;

static inline float WW(void) { return (float)g_cols; }
static inline float WH(void) { return (float)((g_rows - 3) * 2); }

static inline int pcol(float x) { return (int)(x + 0.5f); }
static inline int prow(float y) { return (int)(y * 0.5f + 0.5f); }

static uint32_t g_rng = 0xDEAD1234u;
static float rng_f(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return (float)(g_rng >> 8) / (float)(1u << 24);
}

static void body_init_mass(Body *b)
{
    float area = 4.f * b->hw * b->hh;   /* AABB area */
    b->mass  = area * DENSITY;
    b->imass = 1.f / b->mass;
}

static void body_wake(Body *b) { b->sleeping = false; b->sleep_cnt = 0; }

/* ===================================================================== */
/* §5  framebuffer                                                        */
/* ===================================================================== */

static char g_fb [ROWS_MAX][COLS_MAX];
static int  g_fcp[ROWS_MAX][COLS_MAX];

static void fb_clear(void)
{
    memset(g_fb,  0, sizeof g_fb);
    memset(g_fcp, 0, sizeof g_fcp);
}
static void fb_put(int r, int c, char ch, int cp)
{
    if (r < 0 || r >= g_rows - 3 || c < 0 || c >= g_cols) return;
    g_fb[r][c] = ch;  g_fcp[r][c] = cp;
}
static void fb_hline(int r, int x0, int x1, char ch, int cp)
{ for (int x = x0; x <= x1; x++) fb_put(r, x, ch, cp); }
static void fb_vline(int c, int y0, int y1, char ch, int cp)
{ for (int y = y0; y <= y1; y++) fb_put(y, c, ch, cp); }

static void fb_flush(void)
{
    for (int r = 0; r < g_rows - 3; r++)
        for (int c = 0; c < g_cols; c++) {
            if (!g_fb[r][c]) continue;
            attron (COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
            mvaddch(r, c, (chtype)g_fb[r][c]);
            attroff(COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
        }
}

/* ===================================================================== */
/* §6  physics                                                            */
/* ===================================================================== */

/*
 * col_bodies() — AABB overlap test + two-pass resolution.
 *
 * Pass A — positional correction (ALWAYS, even when bodies are separating)
 *   This is the critical fix over the previous version.  The old code had
 *   Baumgarte inside the velocity function, guarded by (vn > 0).  Bodies
 *   that overlapped but had the same velocity (vn=0) were never pushed
 *   apart; they fell together as a merged mass.
 *   Now positional correction runs unconditionally whenever depth > SLOP.
 *
 * Pass B — velocity impulse (only when approaching: vn > 0)
 *   j = (1+e_eff)*vn / (imA+imB)
 *   Adaptive restitution: e_eff = (vn > REST_THRESH) ? e : 0
 *   → at gravity-scale speeds, e_eff=0, impulse cancels vy exactly,
 *     no micro-bounce.
 *   Coulomb friction on tangent: |jt| ≤ μ·j
 *
 * n = (nx,ny) points from a toward b.
 */
static void col_bodies(Body *a, Body *b)
{
    float ox = (a->hw + b->hw) - fabsf(b->x - a->x);
    float oy = (a->hh + b->hh) - fabsf(b->y - a->y);
    if (ox <= 0.f || oy <= 0.f) return;      /* no overlap */

    float nx, ny, depth;
    if (ox < oy) {
        nx = (b->x > a->x) ? 1.f : -1.f;  ny = 0.f;  depth = ox;
    } else {
        nx = 0.f;  ny = (b->y > a->y) ? 1.f : -1.f;  depth = oy;
    }

    float imb   = b->imass;
    float denom = a->imass + imb;
    if (denom < 1e-12f) return;

    /* ── Pass A: positional correction (always) ─────────────────── */
    float corr = fmaxf(depth - SLOP, 0.f) * BAUMGARTE / denom;
    if (corr > 0.f) {
        float ca = corr * a->imass;
        float cb = corr * imb;
        a->x -= nx * ca;  a->y -= ny * ca;
        b->x += nx * cb;  b->y += ny * cb;
        if (ca > WAKE_IMP) body_wake(a);
        if (cb > WAKE_IMP) body_wake(b);
    }

    /* ── Pass B: velocity impulse (approaching only) ─────────────── */
    float dvx = a->vx - b->vx,  dvy = a->vy - b->vy;
    float vn  = dvx * nx + dvy * ny;
    if (vn <= 0.f) return;                    /* separating — skip */

    float e_eff = (vn > REST_THRESH) ? g_rest : 0.f;
    float jn    = (1.f + e_eff) * vn / denom;
    if (jn > WAKE_IMP) { body_wake(a); body_wake(b); }

    a->vx -= nx * jn * a->imass;  a->vy -= ny * jn * a->imass;
    b->vx += nx * jn * imb;       b->vy += ny * jn * imb;

    /* Coulomb friction */
    float tx = -ny, ty = nx;
    float vt = dvx * tx + dvy * ty;
    float jt = -vt / denom;
    float mx = FRICTION * jn;
    if (jt >  mx) jt =  mx;
    if (jt < -mx) jt = -mx;
    a->vx += tx * jt * a->imass;  a->vy += ty * jt * a->imass;
    b->vx -= tx * jt * imb;       b->vy -= ty * jt * imb;
}

/*
 * col_floor() — hard floor at y = WH().
 * Snaps position fully (no fraction) then resolves velocity.
 * Runs for ALL bodies so Baumgarte drift can't push sleeping bodies below.
 */
static void col_floor(Body *b)
{
    float wh  = WH();
    float bot = b->y + b->hh;
    if (bot <= wh) return;
    b->y = wh - b->hh;                      /* full snap — no fraction */

    /* velocity: cancel downward component only */
    if (b->vy <= 0.f) return;
    float e_eff = (b->vy > REST_THRESH) ? g_rest : 0.f;
    b->vy = -b->vy * e_eff;
    b->vx *= (1.f - FRICTION * (1.f + e_eff));   /* floor friction */
    if (fabsf(b->vx) < SLEEP_VEL) b->vx = 0.f;
}

/*
 * col_walls() — left wall, right wall, ceiling.
 * Same snap pattern as col_floor.
 */
static void col_walls(Body *b)
{
    float ww = WW();

    /* left */
    if (b->x - b->hw < 0.f) {
        b->x = b->hw;
        if (b->vx < 0.f) {
            float e_eff = (fabsf(b->vx) > REST_THRESH) ? g_rest : 0.f;
            b->vx = -b->vx * e_eff;
        }
    }
    /* right */
    if (b->x + b->hw > ww) {
        b->x = ww - b->hw;
        if (b->vx > 0.f) {
            float e_eff = (fabsf(b->vx) > REST_THRESH) ? g_rest : 0.f;
            b->vx = -b->vx * e_eff;
        }
    }
    /* ceiling */
    if (b->y - b->hh < 0.f) {
        b->y = b->hh;
        if (b->vy < 0.f) {
            float e_eff = (fabsf(b->vy) > REST_THRESH) ? g_rest : 0.f;
            b->vy = -b->vy * e_eff;
        }
    }
}

/*
 * scene_step() — one simulation tick.
 *
 *  1  Gravity (awake bodies only).
 *  2  Integrate + damping + speed cap.
 *  3  Body-body solver: SOLVER_ITERS × (col_bodies for all pairs).
 *  4  Floor + wall snap for ALL bodies (sleeping included).
 *  5  Sleep counter update.
 */
static void scene_step(void)
{
    /* 1. gravity */
    if (g_grav)
        for (int i = 0; i < g_nb; i++)
            if (!g_b[i].sleeping)
                g_b[i].vy += GRAVITY;

    /* 2. integrate + damping + cap */
    for (int i = 0; i < g_nb; i++) {
        Body *b = &g_b[i];
        if (b->sleeping) continue;
        b->x += b->vx;
        b->y += b->vy;
        b->vx *= DAMPING;
        b->vy *= DAMPING;
        float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
        if (spd > MAX_SPEED) {
            float s = MAX_SPEED / spd;
            b->vx *= s;  b->vy *= s;
        }
    }

    /* 3. body-body — multiple iterations to resolve stacks */
    for (int iter = 0; iter < SOLVER_ITERS; iter++) {
        for (int i = 0; i < g_nb; i++) {
            for (int j = i + 1; j < g_nb; j++) {
                Body *a = &g_b[i], *bj = &g_b[j];
                if (a->sleeping && bj->sleeping) continue;
                col_bodies(a, bj);
            }
        }
    }

    /* 4. boundaries — ALL bodies (sleeping too, to counter Baumgarte drift) */
    for (int i = 0; i < g_nb; i++) {
        col_floor(&g_b[i]);
        col_walls(&g_b[i]);
    }

    /* 5. sleep counter */
    for (int i = 0; i < g_nb; i++) {
        Body *b = &g_b[i];
        if (b->sleeping) continue;
        float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
        if (spd < SLEEP_VEL) {
            if (++b->sleep_cnt >= SLEEP_FRAMES) {
                b->vx = b->vy = 0.f;
                b->sleeping   = true;
            }
        } else {
            b->sleep_cnt = 0;
        }
    }

    g_tick++;
}

/* ===================================================================== */
/* §7  scene management                                                   */
/* ===================================================================== */

/*
 * aabb_overlaps_any() — true if candidate overlaps any existing body.
 * Used at spawn time to reject or reposition new bodies.
 */
static bool aabb_overlaps_any(const Body *c)
{
    for (int i = 0; i < g_nb; i++) {
        const Body *b = &g_b[i];
        if ((c->hw + b->hw) > fabsf(c->x - b->x) &&
            (c->hh + b->hh) > fabsf(c->y - b->y))
            return true;
    }
    return false;
}

static bool scene_add_cube(void)
{
    if (g_nb >= MAX_BODIES) return false;
    float ww = WW(), hw = CUBE_HW, hh = CUBE_HH;

    Body b = {0};
    b.kind = KIND_CUBE;
    b.hw   = hw;  b.hh = hh;
    b.cp   = 1 + (g_ncubes % 6);
    body_init_mass(&b);

    /* try up to 8 random x positions; skip if overlaps existing body */
    bool placed = false;
    for (int attempt = 0; attempt < 8; attempt++) {
        float x = ww * (0.10f + rng_f() * 0.80f);
        if (x < hw + 1.f)      x = hw + 1.f;
        if (x > ww - hw - 1.f) x = ww - hw - 1.f;
        b.x = x;
        b.y = hh + 1.f;
        if (!aabb_overlaps_any(&b)) { placed = true; break; }
    }
    if (!placed) return false;

    b.vx = (rng_f() - 0.5f) * 2.f;
    g_b[g_nb++] = b;
    g_ncubes++;
    return true;
}

static bool scene_add_sphere(void)
{
    if (g_nb >= MAX_BODIES) return false;
    float ww = WW();
    float hw  = SPH_R;
    float hh  = 2.f * SPH_R;   /* aspect-corrected: visual y-radius = 2r */

    Body b = {0};
    b.kind = KIND_SPHERE;
    b.hw   = hw;  b.hh = hh;
    b.cp   = 1 + (g_nsphs % 6);
    body_init_mass(&b);

    bool placed = false;
    for (int attempt = 0; attempt < 8; attempt++) {
        float x = ww * (0.10f + rng_f() * 0.80f);
        if (x < hw + 1.f)      x = hw + 1.f;
        if (x > ww - hw - 1.f) x = ww - hw - 1.f;
        b.x = x;
        b.y = hh + 1.f;
        if (!aabb_overlaps_any(&b)) { placed = true; break; }
    }
    if (!placed) return false;

    b.vx = (rng_f() - 0.5f) * 2.f;
    g_b[g_nb++] = b;
    g_nsphs++;
    return true;
}

static void scene_remove_last(void)
{ if (g_nb > 0) g_nb--; }

static void scene_init(void)
{
    g_nb = g_ncubes = g_nsphs = 0;
    g_tick = 0;
    g_rng  = (uint32_t)time(NULL) ^ 0xDEAD1234u;

    float ww = WW(), wh = WH();

    /* cube pre-placed at rest on floor */
    Body c = {0};
    c.kind     = KIND_CUBE;
    c.x        = ww * 0.50f;
    c.y        = wh - CUBE_HH;
    c.hw       = CUBE_HW;  c.hh = CUBE_HH;
    c.cp       = 1;
    c.sleeping = true;
    body_init_mass(&c);
    g_b[g_nb++] = c;
    g_ncubes++;

    /* sphere dropping from top */
    Body s = {0};
    s.kind = KIND_SPHERE;
    s.x    = ww * 0.50f;
    s.hw   = SPH_R;
    s.hh   = 2.f * SPH_R;
    s.y    = s.hh + 1.f;
    s.cp   = 4;
    body_init_mass(&s);
    g_b[g_nb++] = s;
    g_nsphs++;
}

/* ===================================================================== */
/* §8  draw                                                               */
/* ===================================================================== */

/*
 * Sphere is drawn using hw (x) and hh (y) so the ellipse matches the
 * AABB exactly.  prow(y + hh*sin) = (y + 2r*sin) / 2 → r rows from
 * center on screen.  pcol(x + hw*cos) = x + r cols from center.
 * The result is a circle of radius r on screen, matching the physics box.
 */
static void draw_body(const Body *b)
{
    int cp = b->cp;
    if (b->kind == KIND_SPHERE) {
        float rx = b->hw, ry = b->hh;
        int steps = (int)(2.f * 3.14159265f * rx) + 8;
        for (int i = 0; i < steps; i++) {
            float a = i * 2.f * 3.14159265f / steps;
            fb_put(prow(b->y + ry * sinf(a)),
                   pcol(b->x + rx * cosf(a)), 'O', cp);
        }
        fb_put(prow(b->y), pcol(b->x), '+', cp);
    } else {
        int x0 = pcol(b->x - b->hw), x1 = pcol(b->x + b->hw);
        int y0 = prow(b->y - b->hh), y1 = prow(b->y + b->hh);
        fb_hline(y0, x0, x1, '#', cp);
        fb_hline(y1, x0, x1, '#', cp);
        fb_vline(x0, y0, y1, '#', cp);
        fb_vline(x1, y0, y1, '#', cp);
        fb_put(prow(b->y), pcol(b->x), '+', cp);
    }
}

static void scene_draw(void)
{
    erase();
    fb_clear();

    /* floor line */
    int floor_row = prow(WH());
    attron(COLOR_PAIR(CP_FLOOR));
    for (int c = 0; c < g_cols; c++) mvaddch(floor_row, c, '=');
    attroff(COLOR_PAIR(CP_FLOOR));

    for (int i = 0; i < g_nb; i++) draw_body(&g_b[i]);
    fb_flush();

    int nc = 0, ns = 0;
    for (int i = 0; i < g_nb; i++) {
        if (g_b[i].kind == KIND_CUBE) nc++; else ns++;
    }
    int rows; { int cc; getmaxyx(stdscr, rows, cc); (void)cc; }

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(rows - 2, 0,
        " [c]cube [s]sphere [x]del  cubes:%-2d spheres:%-2d /%-2d"
        "  rest:%.2f  grav:%s  tick:%-5ld  theme:%s  %s",
        nc, ns, MAX_BODIES, g_rest,
        g_grav ? "on " : "off", g_tick,
        k_themes[g_theme].name, g_paused ? "[PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    mvaddstr(rows - 1, 0,
        "  [e/E]restitution  [g]gravity  [t/T]theme  [r]reset  [p]pause  [q]quit");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0, g_quit = 0;
static void on_sigwinch(int s) { (void)s; g_resize = 1; }
static void on_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    start_color(); g_256 = (COLORS >= 256);
    theme_apply(g_theme);
}
static void screen_resize(void)
{
    endwin(); refresh();
    int r, c; getmaxyx(stdscr, r, c);
    g_rows = (r < ROWS_MAX) ? r : ROWS_MAX;
    g_cols = (c < COLS_MAX) ? c : COLS_MAX;
    g_resize = 0;
}

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, on_sigwinch);
    signal(SIGTERM,  on_sigterm);
    signal(SIGINT,   on_sigterm);

    g_rng = (uint32_t)time(NULL) ^ 0xDEAD1234u;
    screen_init();
    { int r, c; getmaxyx(stdscr, r, c);
      g_rows = (r < ROWS_MAX) ? r : ROWS_MAX;
      g_cols = (c < COLS_MAX) ? c : COLS_MAX; }
    scene_init();

    int64_t next = clock_ns();
    while (!g_quit) {
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit   = 1;                              break;
            case 'p': case ' ': g_paused = !g_paused;                     break;
            case 'r': scene_init();                                        break;
            case 'c': scene_add_cube();                                    break;
            case 's': scene_add_sphere();                                  break;
            case 'x': scene_remove_last();                                 break;
            case 'g': g_grav = !g_grav;                                    break;
            case 'e': if (g_rest < 0.95f) g_rest += REST_STEP;            break;
            case 'E': if (g_rest > 0.05f) g_rest -= REST_STEP;            break;
            case 't': g_theme=(g_theme+1)%N_THEMES; theme_apply(g_theme); break;
            case 'T': g_theme=(g_theme+N_THEMES-1)%N_THEMES; theme_apply(g_theme); break;
            }
        }

        if (g_resize) { screen_resize(); scene_init(); }

        int64_t now = clock_ns();
        if (!g_paused && now >= next) {
            scene_step();
            next = now + NS_PER_S / SIM_FPS;
        }
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        sleep_ns(next - clock_ns() - 1000000LL);
    }

    endwin();
    return 0;
}
