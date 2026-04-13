/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rigid_body.c — 2D Rigid Body Simulation (no rotation)
 *
 * Three body types: floor (fixed boundary), cube (AABB rect), sphere (circle).
 * Nothing penetrates anything. Gravity pulls all dynamic bodies down.
 * Bodies collide, exchange momentum with friction, and eventually sleep.
 *
 * Collision detection:
 *   cube-cube    — AABB overlap, minimum-penetration axis
 *   sphere-sphere— circle distance vs sum of radii
 *   cube-sphere  — closest point on AABB to sphere center
 *   body-floor   — boundary clamp + velocity reflection
 *   body-walls   — same for left / right / ceiling
 *
 * Collision resolution:
 *   Normal impulse  j  = (1+e)·vn / (1/mA + 1/mB)
 *   Coulomb friction   |jt| ≤ μ·j  on the contact tangent
 *   Baumgarte positional correction for body-body pairs
 *   Sleep: SLEEP_FRAMES consecutive low-speed frames → frozen
 *
 * Keys:
 *   c  add cube      s  add sphere    x  remove last body
 *   r  reset         p  pause/resume  g  toggle gravity
 *   e/E restitution  t/T theme        q  quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/rigid_body.c -o rigid_body -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 body  §5 framebuf
 *           §6 physics  §7 scene  §8 draw  §9 screen  §10 app
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
#define MAX_BODIES     30
#define N_THEMES        5

#define GRAVITY       0.04f   /* physics units / step²  (downward) */
#define REST_DEF      0.35f   /* coefficient of restitution */
#define REST_STEP     0.05f
#define FRICTION      0.30f   /* Coulomb μ */
#define DAMPING       0.992f  /* per-step linear damping */
#define MAX_SPEED     20.0f   /* velocity cap — prevents tunneling */
#define BAUMGARTE     0.40f   /* positional correction fraction */
#define SLOP          0.30f   /* penetration allowed before Baumgarte fires */
#define SLEEP_VEL     0.06f   /* speed below which sleep counter increments */
#define SLEEP_FRAMES   18     /* frames of quiet before body sleeps */
#define WAKE_IMP      0.04f   /* minimum impulse that wakes a sleeping body */

#define CUBE_HW       7.0f    /* cube half-width  (physics units) */
#define CUBE_HH       5.0f    /* cube half-height */
#define SPH_R         5.0f    /* sphere radius */
#define DENSITY       0.008f  /* mass = DENSITY × area */

#define SIM_FPS        20
#define NS_PER_S  1000000000LL

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
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

#define CP_FLOOR   9
#define CP_HUD    10

static const struct { short c[6]; const char *name; } k_themes[N_THEMES] = {
    {{ 196, 214, 226,  46,  51, 129 }, "Vivid"  },
    {{ 160, 172, 178,  34,  43,  91 }, "Muted"  },
    {{ 231, 220, 214, 208, 202, 196 }, "Fire"   },
    {{  46,  82, 118, 154, 190, 226 }, "Matrix" },
    {{  51,  45,  39,  33,  27,  21 }, "Ocean"  },
};
static bool g_256; static int g_theme = 0;

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

typedef struct {
    Kind  kind;
    float x, y;        /* center, physics coords (y increases downward) */
    float vx, vy;
    float hw, hh;      /* cube: half-extents;  sphere: hw == hh == radius */
    float mass, imass; /* imass = 1/mass */
    int   cp;          /* ncurses color pair */
    int   sleep_cnt;
    bool  sleeping;
} Body;

static Body  g_b[MAX_BODIES];
static int   g_nb     = 0;
static int   g_ncubes = 0;   /* cumulative, for color cycling */
static int   g_nsphs  = 0;
static float g_rest   = REST_DEF;
static bool  g_grav   = true;
static bool  g_paused = false;
static long  g_tick   = 0;

static int g_rows, g_cols;

/* World dimensions in physics units */
static inline float WW(void) { return (float)g_cols; }
static inline float WH(void) { return (float)((g_rows - 3) * 2); }

/* Physics → screen */
static inline int pcol(float x) { return (int)(x + 0.5f); }
static inline int prow(float y) { return (int)(y * 0.5f + 0.5f); }

/* XorShift RNG */
static uint32_t g_rng = 0xDEAD1234u;
static float rng_f(void)
{
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return (float)(g_rng >> 8) / (float)(1u << 24);
}

static void body_init_mass(Body *b)
{
    float area = (b->kind == KIND_CUBE)
               ? 4.f * b->hw * b->hh
               : 3.14159265f * b->hw * b->hw;
    b->mass  = area * DENSITY;
    b->imass = 1.f / b->mass;
}

/* ===================================================================== */
/* §5  framebuffer                                                        */
/* ===================================================================== */

static char g_fb [ROWS_MAX][COLS_MAX];
static int  g_fcp[ROWS_MAX][COLS_MAX];

static void fb_clear(void)
{ memset(g_fb, 0, sizeof g_fb); memset(g_fcp, 0, sizeof g_fcp); }

static void fb_put(int r, int c, char ch, int cp)
{
    if (r < 0 || r >= g_rows - 3 || c < 0 || c >= g_cols) return;
    g_fb[r][c] = ch; g_fcp[r][c] = cp;
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
            attron(COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
            mvaddch(r, c, (chtype)g_fb[r][c]);
            attroff(COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
        }
}

static void draw_body(const Body *b)
{
    int cp = b->cp;
    if (b->kind == KIND_SPHERE) {
        float r = b->hw;
        int steps = (int)(2.f * 3.14159265f * r) + 8;
        for (int i = 0; i < steps; i++) {
            float a = i * 2.f * 3.14159265f / steps;
            /* multiply sin by 2 so the drawn ellipse looks circular on screen
               because prow maps y/2 → each physics y-unit is half a row     */
            fb_put(prow(b->y + r * 2.f * sinf(a)), pcol(b->x + r * cosf(a)), 'O', cp);
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

/* ===================================================================== */
/* §6  physics                                                            */
/* ===================================================================== */

/* Wake a sleeping body. */
static void body_wake(Body *b) { b->sleeping = false; b->sleep_cnt = 0; }

/*
 * impulse() — core collision resolver used by every collision pair.
 *
 * a, b   : colliding bodies (b == NULL means fixed surface).
 * nx, ny : unit contact normal pointing FROM a TOWARD b (or surface).
 * depth  : penetration depth (used for Baumgarte; pass 0 if already corrected).
 * e      : coefficient of restitution.
 *
 * Applies:
 *   1. Normal impulse  j = (1+e)·vn / (ima + imb)
 *   2. Coulomb friction tangent impulse
 *   3. Baumgarte positional correction (skipped when depth ≤ SLOP)
 */
static void impulse(Body *a, Body *b,
                    float nx, float ny, float depth, float e)
{
    float vax = a->vx, vay = a->vy;
    float vbx = b ? b->vx : 0.f, vby = b ? b->vy : 0.f;
    float dvx = vax - vbx, dvy = vay - vby;
    float vn  = dvx * nx + dvy * ny;
    if (vn <= 0.f) return;  /* bodies already separating */

    float imb   = b ? b->imass : 0.f;
    float denom = a->imass + imb;
    if (denom < 1e-12f) return;

    /* ── normal impulse ── */
    float jn = (1.f + e) * vn / denom;
    if (jn > WAKE_IMP) { body_wake(a); if (b) body_wake(b); }
    a->vx -= nx * jn * a->imass;  a->vy -= ny * jn * a->imass;
    if (b) { b->vx += nx * jn * b->imass; b->vy += ny * jn * b->imass; }

    /* ── Coulomb friction on tangent (uses pre-impulse relative vel) ── */
    float tx = -ny, ty = nx;
    float vt = dvx * tx + dvy * ty;
    float jt = -vt / denom;
    float mx = FRICTION * jn;
    if (jt >  mx) jt =  mx;
    if (jt < -mx) jt = -mx;
    a->vx += tx * jt * a->imass;  a->vy += ty * jt * a->imass;
    if (b) { b->vx -= tx * jt * b->imass; b->vy -= ty * jt * b->imass; }

    /* ── Baumgarte positional correction ── */
    float corr = fmaxf(depth - SLOP, 0.f) * BAUMGARTE / denom;
    a->x -= nx * corr * a->imass;  a->y -= ny * corr * a->imass;
    if (b) { b->x += nx * corr * b->imass; b->y += ny * corr * b->imass; }
}

/* ── cube-cube (AABB overlap) ──────────────────────────────────────── */
static void col_cc(Body *a, Body *b)
{
    float ox = (a->hw + b->hw) - fabsf(b->x - a->x);
    float oy = (a->hh + b->hh) - fabsf(b->y - a->y);
    if (ox <= 0.f || oy <= 0.f) return;
    float nx, ny, depth;
    if (ox < oy) { nx = (b->x > a->x) ? 1.f : -1.f; ny = 0.f;  depth = ox; }
    else         { nx = 0.f; ny = (b->y > a->y) ? 1.f : -1.f;   depth = oy; }
    impulse(a, b, nx, ny, depth, g_rest);
}

/* ── sphere-sphere (circle distance) ───────────────────────────────── */
static void col_ss(Body *a, Body *b)
{
    float dx = b->x - a->x, dy = b->y - a->y;
    float dist2 = dx * dx + dy * dy;
    float sum   = a->hw + b->hw;   /* sum of radii */
    if (dist2 >= sum * sum) return;
    float dist = sqrtf(dist2);
    float nx, ny;
    if (dist < 1e-6f) { nx = 1.f; ny = 0.f; }
    else { nx = dx / dist; ny = dy / dist; }
    impulse(a, b, nx, ny, sum - dist, g_rest);
}

/* ── cube-sphere (closest point on AABB to sphere center) ──────────── */
static void col_cs(Body *cube, Body *sph)
{
    /* closest point on cube AABB */
    float cx = fmaxf(cube->x - cube->hw, fminf(cube->x + cube->hw, sph->x));
    float cy = fmaxf(cube->y - cube->hh, fminf(cube->y + cube->hh, sph->y));
    float dx = sph->x - cx, dy = sph->y - cy;
    float dist2 = dx * dx + dy * dy;
    float r = sph->hw;
    if (dist2 >= r * r) return;

    float dist = sqrtf(dist2);
    float nx, ny, depth;
    if (dist < 1e-6f) {
        /* sphere center inside cube — push out along minimum axis */
        float px = cube->hw - fabsf(sph->x - cube->x);
        float py = cube->hh - fabsf(sph->y - cube->y);
        if (px < py) { nx = (sph->x > cube->x) ? 1.f : -1.f; ny = 0.f; depth = px + r; }
        else         { nx = 0.f; ny = (sph->y > cube->y) ? 1.f : -1.f; depth = py + r; }
    } else {
        nx = dx / dist; ny = dy / dist; depth = r - dist;
    }
    /* n points from cube toward sphere → impulse(cube, sph, n) */
    impulse(cube, sph, nx, ny, depth, g_rest);
}

/* ── body vs floor (bottom wall) ───────────────────────────────────── */
static void col_floor(Body *b)
{
    float wh   = WH();
    float bot  = b->y + (b->kind == KIND_SPHERE ? b->hw : b->hh);
    float depth = bot - wh;
    if (depth <= 0.f) return;
    /* correct position, then resolve with n=(0,1) toward floor */
    if (b->kind == KIND_SPHERE) b->y = wh - b->hw;
    else                        b->y = wh - b->hh;
    impulse(b, NULL, 0.f, 1.f, 0.f, g_rest);  /* depth=0: position already fixed */
}

/* ── body vs side walls and ceiling ─────────────────────────────────── */
static void col_walls(Body *b)
{
    float ww  = WW();
    float ext = (b->kind == KIND_SPHERE) ? b->hw : b->hw; /* same formula */

    /* left wall: n=(-1,0) from body toward left */
    if (b->x - ext < 0.f) {
        b->x = ext;
        impulse(b, NULL, -1.f, 0.f, 0.f, g_rest);
    }
    /* right wall: n=(1,0) */
    if (b->x + ext > ww) {
        b->x = ww - ext;
        impulse(b, NULL, 1.f, 0.f, 0.f, g_rest);
    }
    /* ceiling: n=(0,-1) from body toward ceiling */
    float top = b->y - (b->kind == KIND_SPHERE ? b->hw : b->hh);
    if (top < 0.f) {
        if (b->kind == KIND_SPHERE) b->y = b->hw;
        else                        b->y = b->hh;
        impulse(b, NULL, 0.f, -1.f, 0.f, g_rest);
    }
}

/* ── main simulation step ───────────────────────────────────────────── */
static void scene_step(void)
{
    /* 1. Gravity */
    if (g_grav)
        for (int i = 0; i < g_nb; i++)
            if (!g_b[i].sleeping)
                g_b[i].vy += GRAVITY;

    /* 2. Integrate + damping + speed cap */
    for (int i = 0; i < g_nb; i++) {
        Body *b = &g_b[i];
        if (b->sleeping) continue;
        b->x += b->vx;  b->y += b->vy;
        b->vx *= DAMPING; b->vy *= DAMPING;
        float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
        if (spd > MAX_SPEED) { float s = MAX_SPEED / spd; b->vx *= s; b->vy *= s; }
    }

    /* 3. Body-body collisions (3 iterations for stacked bodies) */
    for (int iter = 0; iter < 3; iter++)
        for (int i = 0; i < g_nb; i++)
            for (int j = i + 1; j < g_nb; j++) {
                Body *a = &g_b[i], *b = &g_b[j];
                if (a->sleeping && b->sleeping) continue;
                if      (a->kind==KIND_CUBE   && b->kind==KIND_CUBE)   col_cc(a, b);
                else if (a->kind==KIND_SPHERE && b->kind==KIND_SPHERE) col_ss(a, b);
                else if (a->kind==KIND_CUBE   && b->kind==KIND_SPHERE) col_cs(a, b);
                else                                                    col_cs(b, a);
            }

    /* 4. Floor and wall boundaries (skip sleeping — no gravity means no new penetration) */
    for (int i = 0; i < g_nb; i++) {
        if (g_b[i].sleeping) continue;
        col_floor(&g_b[i]);
        col_walls(&g_b[i]);
    }

    /* 5. Sleep counter:
     *    Increment while speed < SLEEP_VEL; freeze after SLEEP_FRAMES frames.
     *    Bodies are woken inside impulse() when j > WAKE_IMP. */
    for (int i = 0; i < g_nb; i++) {
        Body *b = &g_b[i];
        if (b->sleeping) continue;
        float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
        if (spd < SLEEP_VEL) {
            if (++b->sleep_cnt >= SLEEP_FRAMES) {
                b->vx = b->vy = 0.f;
                b->sleeping = true;
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

static bool scene_add_cube(void)
{
    if (g_nb >= MAX_BODIES) return false;
    float ww = WW();
    float hw = CUBE_HW, hh = CUBE_HH;
    float x  = ww * (0.15f + rng_f() * 0.70f);
    if (x < hw + 1.f) x = hw + 1.f;
    if (x > ww - hw - 1.f) x = ww - hw - 1.f;
    Body b = {0};
    b.kind = KIND_CUBE;
    b.x = x;  b.y = hh + 1.f;   /* near top */
    b.vx = (rng_f() - 0.5f) * 2.f;
    b.hw = hw; b.hh = hh;
    b.cp = 1 + (g_ncubes % 6);
    body_init_mass(&b);
    g_b[g_nb++] = b;
    g_ncubes++;
    return true;
}

static bool scene_add_sphere(void)
{
    if (g_nb >= MAX_BODIES) return false;
    float ww = WW();
    float r  = SPH_R;
    float x  = ww * (0.15f + rng_f() * 0.70f);
    if (x < r + 1.f) x = r + 1.f;
    if (x > ww - r - 1.f) x = ww - r - 1.f;
    Body b = {0};
    b.kind = KIND_SPHERE;
    b.x = x;  b.y = r + 1.f;   /* near top */
    b.vx = (rng_f() - 0.5f) * 2.f;
    b.hw = b.hh = r;
    b.cp = 1 + (g_nsphs % 6);
    body_init_mass(&b);
    g_b[g_nb++] = b;
    g_nsphs++;
    return true;
}

static void scene_remove_last(void)
{ if (g_nb > 0) g_nb--; }

static void scene_init(void)
{
    g_nb = g_ncubes = g_nsphs = 0; g_tick = 0;
    g_rng = (uint32_t)time(NULL) ^ 0xDEAD1234u;
    float ww = WW(), wh = WH();

    /* one cube resting on the floor */
    Body c = {0};
    c.kind = KIND_CUBE; c.x = ww * 0.5f; c.y = wh - CUBE_HH;
    c.hw = CUBE_HW; c.hh = CUBE_HH; c.cp = 1;
    body_init_mass(&c);
    g_b[g_nb++] = c; g_ncubes++;

    /* one sphere at the top, falling toward the cube */
    Body s = {0};
    s.kind = KIND_SPHERE; s.x = ww * 0.5f; s.y = SPH_R + 1.f;
    s.hw = s.hh = SPH_R; s.cp = 4;
    body_init_mass(&s);
    g_b[g_nb++] = s; g_nsphs++;
}

/* ===================================================================== */
/* §8  draw                                                               */
/* ===================================================================== */

static void scene_draw(void)
{
    erase(); fb_clear();

    /* floor line */
    int floor_row = prow(WH());
    attron(COLOR_PAIR(CP_FLOOR));
    for (int c = 0; c < g_cols; c++) mvaddch(floor_row, c, '=');
    attroff(COLOR_PAIR(CP_FLOOR));

    /* bodies */
    for (int i = 0; i < g_nb; i++) draw_body(&g_b[i]);
    fb_flush();

    /* HUD — count live bodies */
    int nc = 0, ns = 0;
    for (int i = 0; i < g_nb; i++) {
        if (g_b[i].kind == KIND_CUBE)   nc++;
        else                             ns++;
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
            case 'q': case 27: g_quit   = 1;                     break;
            case 'p': case ' ': g_paused = !g_paused;             break;
            case 'r': scene_init();                               break;
            case 'c': scene_add_cube();                           break;
            case 's': scene_add_sphere();                         break;
            case 'x': scene_remove_last();                        break;
            case 'g': g_grav = !g_grav;                           break;
            case 'e': if (g_rest < 0.95f) g_rest += REST_STEP;   break;
            case 'E': if (g_rest > 0.05f) g_rest -= REST_STEP;   break;
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
        wnoutrefresh(stdscr); doupdate();
        sleep_ns(next - clock_ns() - 1000000LL);
    }
    endwin(); return 0;
}
