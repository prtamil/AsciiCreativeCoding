/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/nuke.c — 2D Shockwave Propagation Demo
 *
 * Simulates an expanding circular pressure wave from a point detonation:
 *   • Scalar wave equation — radial pressure ring, reflects off walls
 *   • Cylindrical energy decay — ring dims as 1/r with distance
 *   • Terrain height-map — undulating ground, ripples on shockwave impact
 *   • Debris particle burst — radial ejecta with gravity
 *   • Ground dust cloud — spawned when ring sweeps the terrain
 *   • Screen shake — decaying sinusoidal jitter on detonation
 *   • Initial flash — full-screen white, fades in ~0.2 s
 *
 * Visual flow:  flash → bright ring → ring fades + terrain ripples →
 *               debris arc → dust drifts → quiet
 *
 * Keys: q/ESC quit  |  space/b  detonate  |  p pause  |  r reset
 * Build: gcc -std=c11 -O2 -Wall -Wextra physics/nuke.c \
 *             -o nuke -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — all tunable constants
 *   §2  clock       — monotonic timer + sleep
 *   §3  color       — wave/terrain/particle/flash palettes
 *   §4  wave        — 2D scalar wave field (finite-difference step)
 *   §5  terrain     — height-map generation + ripple interaction
 *   §6  particles   — debris burst + ground dust pool
 *   §7  simulation  — SimState, blast trigger, full tick
 *   §8  render      — wave + terrain + particles + effects → ncurses
 *   §9  screen      — ncurses setup / teardown / resize
 *  §10  app         — main loop
 */

/* ── HOW THIS WORKS ──────────────────────────────────────────────────────── *
 *
 *  WAVE EQUATION (§4):
 *
 *    ∂²u/∂t² = c² ∇²u  −  γ ∂u/∂t
 *
 *    u(x,y,t) — pressure at terminal cell (x,y) at time t
 *    c         — wave speed (cells/second); sets ring expansion rate
 *    ∇²u       — 5-point finite-difference Laplacian
 *    γ         — damping; converts wave energy to heat (ring fades)
 *
 *  The initial condition is a tight Gaussian pulse centred just above the
 *  terrain.  As it evolves, the positive pressure peak travels outward as
 *  a ring; a rarefaction trough (negative pressure) follows immediately
 *  behind.  This is the classic N-wave shape of a real shock.
 *
 *  CFL STABILITY:
 *    c · dt · √2 ≤ 1  →  c ≤ 42 cells/s at 60 Hz.
 *    We use 2 substeps per frame (effective dt = 1/120 s) so c can reach
 *    84 cells/s.  Default c = 28 → CFL = 28/120 · √2 ≈ 0.33  ✓
 *
 *  ENERGY DECAY:
 *    Two independent mechanisms:
 *      1. Geometric (cylindrical spreading): amplitude ∝ 1/√r.
 *         The total energy in the ring is conserved; as circumference
 *         grows, energy per unit length drops — ring dims naturally.
 *      2. Physical (damping γ): energy lost per second = γ × energy.
 *         Exponential envelope exp(−γ t) wraps the oscillation.
 *    Combined, a ring 30 cells out has ~40% of its initial peak.
 *
 *  TERRAIN INTERACTION (§5):
 *    The wave propagates over the terrain rows.  We read u at each
 *    column's terrain-surface row and push it into a per-column ripple
 *    value (terrain_d[]).  terrain_d decays with a spring restoring force
 *    (exponential decay per frame) — giving a heave-and-settle motion.
 *
 *  PARTICLES (§6):
 *    Two types spawned on each blast:
 *      DEBRIS — fast radial burst from blast centre, gravity pulls down.
 *      DUST   — slow particles spawned at terrain level as the ring
 *               sweeps past; drift laterally and upward, fade slowly.
 *
 *  SCREEN SHAKE (§7):
 *    Integer cell offset (shake_r, shake_c) applied to every mvaddch call.
 *    Magnitude = SHAKE_AMP × exp(−SHAKE_DECAY × t_since_blast).
 *    Oscillates at SHAKE_FREQ Hz so it alternates direction each half-cycle.
 *
 * ─────────────────────────────────────────────────────────────────────────*/

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

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

enum {
    SIM_FPS       = 60,
    N_SUBSTEPS    =  2,    /* wave sub-steps per frame (improves CFL margin) */
    HUD_COLS      = 80,
    FPS_MS        = 500,
    MAX_PARTS     = 240,   /* total particle pool                            */
    DEBRIS_BURST  =  80,   /* debris particles per blast                     */
    DUST_PER_COL  =   2,   /* max dust particles spawned per terrain column  */
    TERRAIN_ROWS  =   6,   /* rows reserved for terrain at the screen bottom */
};

/* ── Wave physics ────────────────────────────────────────────────────── */
/*
 * WAVE_C = 28 cells/s.  With N_SUBSTEPS=2 (dt_sub = 1/120 s):
 *   CFL = 28 / 120 × √2 ≈ 0.33  — well within stability limit of 1.
 * WAVE_DAMP = 1.2 / s gives a half-life of ln(2)/1.2 ≈ 0.58 s so the
 * ring is clearly visible for ~1.5 s then fades before reflecting back
 * from walls for the second time.
 */
#define WAVE_C          28.0f   /* propagation speed (cells/sec)              */
#define WAVE_DAMP        1.2f   /* energy damping coefficient (1/sec)         */
#define BLAST_AMP        9.0f   /* peak initial pressure (wave units)         */
#define BLAST_SIGMA      3.2f   /* Gaussian half-width of the initial pulse   */
#define NORM_SCALE      (BLAST_AMP * 0.42f) /* pressure → [0,1] denominator  */

/* ── Terrain ─────────────────────────────────────────────────────────── */
#define TERRAIN_H_MIN    1.5f   /* min terrain height (rows above floor)      */
#define TERRAIN_H_MAX    4.5f   /* max terrain height                         */
#define RIPPLE_SCALE     0.80f  /* wave-pressure → terrain displacement ratio */
#define RIPPLE_DECAY     0.88f  /* per-frame decay factor for terrain ripple  */
#define DUST_THRESH      0.60f  /* |pressure| at terrain → spawn dust         */

/* ── Particles ───────────────────────────────────────────────────────── */
#define CELL_ASPECT      2.0f   /* terminal cell h:w pixel ratio              */
#define GRAV            18.0f   /* gravity (cells/sec²)                       */
#define DEBRIS_SPD_MIN  10.0f   /* debris min initial speed (cells/sec)       */
#define DEBRIS_SPD_MAX  30.0f   /* debris max initial speed                   */
#define DUST_SPD_X       3.5f   /* dust lateral drift speed                   */
#define DUST_SPD_Y       2.0f   /* dust upward drift speed                    */

/* ── Screen shake ────────────────────────────────────────────────────── */
#define SHAKE_AMP        2.2f   /* initial shake magnitude (cells)            */
#define SHAKE_DECAY      3.8f   /* exponential decay rate (1/sec)             */
#define SHAKE_FREQ      13.0f   /* oscillation frequency (rad/sec)            */

/* ── Flash ───────────────────────────────────────────────────────────── */
#define FLASH_DECAY      8.0f   /* flash fade rate (1/sec)                    */

/* ── Timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define FRAME_NS    (NS_PER_SEC / SIM_FPS)

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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/*                                                                        */
/*   1–8   pressure ramp: dark-red → orange → yellow → white            */
/*   9     rarefaction trough: cold blue                                 */
/*   10    terrain body (dark green/brown)                               */
/*   11    terrain surface (olive)                                       */
/*   12    terrain displaced (bright orange — shows energy transfer)     */
/*   13    debris spark (yellow)                                         */
/*   14    dust cloud (pale tan)                                         */
/*   15    flash (white)                                                 */
/*   16    HUD                                                           */
/* ===================================================================== */

#define N_PRESS   8
#define CP_RAREFY 9
#define CP_TBASE  10
#define CP_TSURF  11
#define CP_TDISP  12
#define CP_DEBRIS 13
#define CP_DUST   14
#define CP_FLASH  15
#define CP_HUD    16

static const short k_press_fg[N_PRESS] = { 88, 124, 160, 196, 202, 208, 220, 231 };

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int i = 0; i < N_PRESS; i++)
            init_pair((short)(i + 1), k_press_fg[i], -1);
        init_pair(CP_RAREFY,  18, -1);   /* dark navy             */
        init_pair(CP_TBASE,   22, -1);   /* dark green            */
        init_pair(CP_TSURF,  100, -1);   /* olive                 */
        init_pair(CP_TDISP,  214, -1);   /* orange (wave impact)  */
        init_pair(CP_DEBRIS, 226, -1);   /* bright yellow         */
        init_pair(CP_DUST,   180, -1);   /* pale tan              */
        init_pair(CP_FLASH,  231, -1);   /* white                 */
        init_pair(CP_HUD,    231, -1);
    } else {
        for (int i = 0; i < 6; i++) init_pair((short)(i+1), COLOR_RED,    -1);
        for (int i = 6; i < 8; i++) init_pair((short)(i+1), COLOR_YELLOW, -1);
        init_pair(CP_RAREFY,  COLOR_BLUE,  -1);
        init_pair(CP_TBASE,   COLOR_GREEN, -1);
        init_pair(CP_TSURF,   COLOR_GREEN, -1);
        init_pair(CP_TDISP,   COLOR_YELLOW,-1);
        init_pair(CP_DEBRIS,  COLOR_YELLOW,-1);
        init_pair(CP_DUST,    COLOR_WHITE, -1);
        init_pair(CP_FLASH,   COLOR_WHITE, -1);
        init_pair(CP_HUD,     COLOR_WHITE, -1);
    }
}

/* Map normalised pressure [0,1] → color pair + ASCII char */
static void press_glyph(float norm, attr_t *attr_out, char *ch_out)
{
    static const char ramp[] = ".,;:+*#@";   /* 8 levels, index 0=dim */
    int idx = (int)(norm * (N_PRESS - 1));
    if (idx < 0) idx = 0;
    if (idx >= N_PRESS) idx = N_PRESS - 1;
    *ch_out   = ramp[idx];
    *attr_out = COLOR_PAIR(idx + 1);
    if (idx >= 5) *attr_out |= A_BOLD;
}

/* ===================================================================== */
/* §4  wave — 2D scalar wave field                                        */
/*                                                                        */
/* Uses the velocity-field (symplectic Euler) integrator:                */
/*   v += ( c² · ∇²u  −  γ · v ) · dt                                  */
/*   u += v · dt                                                          */
/*                                                                        */
/* Interior cells only.  Border cells stay at u=0 (Dirichlet BC):       */
/* waves reflect with phase inversion — you see the ring bounce back.    */
/* ===================================================================== */

static void wave_step(float *u, float *v, int rows, int cols, float dt)
{
    float c2   = WAVE_C * WAVE_C;
    float damp = WAVE_DAMP;

    /* Update velocity from Laplacian */
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            int   i   = r * cols + c;
            float lap = u[(r-1)*cols + c] + u[(r+1)*cols + c]
                      + u[r*cols + (c-1)] + u[r*cols + (c+1)]
                      - 4.f * u[i];
            v[i] += (c2 * lap - damp * v[i]) * dt;
        }
    }
    /* Apply displacement */
    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            u[r*cols + c] += v[r*cols + c] * dt;
}

/* Inject a Gaussian pressure pulse centred at (cx, cy) */
static void wave_blast(float *u, int rows, int cols, int cx, int cy)
{
    int radius = (int)(BLAST_SIGMA * 4.f);
    float sig2 = BLAST_SIGMA * BLAST_SIGMA;

    for (int r = cy - radius; r <= cy + radius; r++) {
        if (r < 0 || r >= rows) continue;
        for (int c = cx - radius; c <= cx + radius; c++) {
            if (c < 0 || c >= cols) continue;
            /* Correct for cell aspect ratio so the ring is circular  */
            float dy = (r - cy) / CELL_ASPECT;
            float dx = (float)(c - cx);
            float d2 = dx*dx + dy*dy;
            u[r*cols + c] += BLAST_AMP * expf(-d2 / (2.f * sig2));
        }
    }
}

/* ===================================================================== */
/* §5  terrain — undulating ground + ripple on wave impact               */
/*                                                                        */
/* terrain_h[c]  — static height (rows tall) of ground at column c      */
/* terrain_d[c]  — dynamic ripple displacement (positive = heave up)    */
/*                                                                        */
/* Each frame: read wave field at the terrain surface row; scale and     */
/* add to terrain_d[].  terrain_d decays toward 0 (restoring force).    */
/* ===================================================================== */

static void terrain_init(float *h, int cols)
{
    for (int c = 0; c < cols; c++) {
        h[c] = 2.8f
             + 1.2f * sinf(c * 0.09f)
             + 0.7f * sinf(c * 0.19f + 1.4f)
             + 0.3f * sinf(c * 0.41f + 0.7f);
        if (h[c] < TERRAIN_H_MIN) h[c] = TERRAIN_H_MIN;
        if (h[c] > TERRAIN_H_MAX) h[c] = TERRAIN_H_MAX;
    }
}

/*
 * Returns the screen row of the terrain surface at column c.
 * rows-1 is the bottom row; terrain rises from there.
 */
static inline int terrain_surf_row(int rows, float h, float d)
{
    int top = rows - 1 - (int)(h + d + 0.5f);
    if (top < 0) top = 0;
    return top;
}

/* ===================================================================== */
/* §6  particles — radial debris burst + terrain dust cloud              */
/* ===================================================================== */

typedef enum { PT_DEBRIS, PT_DUST } PartType;

typedef struct {
    float    x, y;
    float    vx, vy;
    float    life, max_life;
    PartType type;
    bool     alive;
} Particle;

static unsigned g_pseed = 0x3A7C1F9Du;

static float prand(void)
{
    g_pseed = g_pseed * 1664525u + 1013904223u;
    return (float)((g_pseed >> 8) & 0xFFFFFF) / (float)0x1000000;
}

static Particle *part_alloc(Particle *pool, int n)
{
    for (int i = 0; i < n; i++)
        if (!pool[i].alive) return &pool[i];
    return NULL;
}

static void spawn_debris(Particle *pool, int n, float cx, float cy)
{
    for (int i = 0; i < DEBRIS_BURST; i++) {
        Particle *p = part_alloc(pool, n);
        if (!p) break;
        float angle = prand() * 2.f * (float)M_PI;
        float spd   = DEBRIS_SPD_MIN + prand() * (DEBRIS_SPD_MAX - DEBRIS_SPD_MIN);
        p->x        = cx;
        p->y        = cy;
        p->vx       = cosf(angle) * spd;
        p->vy       = sinf(angle) * spd * (1.f / CELL_ASPECT);  /* aspect-correct */
        p->vy      -= spd * 0.3f;          /* bias upward                */
        p->max_life = p->life = 1.2f + prand() * 1.5f;
        p->type     = PT_DEBRIS;
        p->alive    = true;
    }
}

static void spawn_dust(Particle *pool, int n, float cx, float cy)
{
    Particle *p = part_alloc(pool, n);
    if (!p) return;
    float dir   = (prand() > 0.5f) ? 1.f : -1.f;
    p->x        = cx + (prand() - 0.5f) * 3.f;
    p->y        = cy;
    p->vx       = dir * DUST_SPD_X * (0.4f + prand() * 0.8f);
    p->vy       = -DUST_SPD_Y * (0.5f + prand() * 0.8f);   /* upward */
    p->max_life = p->life = 1.5f + prand() * 2.0f;
    p->type     = PT_DUST;
    p->alive    = true;
}

static void particles_tick(Particle *pool, int n, float dt)
{
    for (int i = 0; i < n; i++) {
        Particle *p = &pool[i];
        if (!p->alive) continue;
        if (p->type == PT_DEBRIS)
            p->vy += GRAV * dt / CELL_ASPECT;   /* gravity, aspect-corrected */
        else
            p->vy -= 0.5f * dt;                 /* dust: slow upward float   */
        p->x    += p->vx * dt;
        p->y    += p->vy * dt;
        p->life -= dt;
        if (p->life <= 0.f) p->alive = false;
    }
}

/* ===================================================================== */
/* §7  simulation — full state, blast trigger, per-frame tick            */
/* ===================================================================== */

typedef struct {
    float    *u;            /* wave displacement field [rows × cols]     */
    float    *v;            /* wave velocity field     [rows × cols]     */
    float    *terrain_h;    /* static terrain height per column          */
    float    *terrain_d;    /* dynamic ripple displacement per column    */
    Particle  parts[MAX_PARTS];
    int       rows, cols;
    float     time;
    float     shake_t;      /* time elapsed since last blast             */
    float     flash;        /* flash intensity [0,1]                     */
    int       blast_cx, blast_cy;
    bool      paused;
} SimState;

static SimState *sim_alloc(int rows, int cols)
{
    SimState *s = calloc(1, sizeof *s);
    s->rows      = rows;
    s->cols      = cols;
    s->u         = calloc((size_t)(rows * cols), sizeof(float));
    s->v         = calloc((size_t)(rows * cols), sizeof(float));
    s->terrain_h = calloc((size_t)cols, sizeof(float));
    s->terrain_d = calloc((size_t)cols, sizeof(float));
    s->shake_t   = 1e9f;   /* no shake initially */
    terrain_init(s->terrain_h, cols);
    return s;
}

static void sim_free(SimState *s)
{
    if (!s) return;
    free(s->u); free(s->v);
    free(s->terrain_h); free(s->terrain_d);
    free(s);
}

static SimState *sim_resize(SimState *s, int rows, int cols)
{
    sim_free(s);
    return sim_alloc(rows, cols);
}

static void sim_blast(SimState *s)
{
    int cx = s->cols / 2;
    int cy = s->rows - TERRAIN_ROWS - 3;  /* just above terrain */
    if (cy < 2) cy = 2;
    wave_blast(s->u, s->rows, s->cols, cx, cy);
    spawn_debris(s->parts, MAX_PARTS, (float)cx, (float)cy);
    s->flash    = 1.0f;
    s->shake_t  = 0.f;
    s->blast_cx = cx;
    s->blast_cy = cy;
}

static void sim_reset(SimState *s)
{
    memset(s->u, 0, (size_t)(s->rows * s->cols) * sizeof(float));
    memset(s->v, 0, (size_t)(s->rows * s->cols) * sizeof(float));
    memset(s->terrain_d, 0, (size_t)s->cols * sizeof(float));
    memset(s->parts, 0, sizeof s->parts);
    s->time    = 0.f;
    s->flash   = 0.f;
    s->shake_t = 1e9f;
    sim_blast(s);
}

static void sim_tick(SimState *s, float dt)
{
    if (s->paused) return;

    /* ── Wave substeps ────────────────────────────────────────────────── */
    float sub_dt = dt / (float)N_SUBSTEPS;
    for (int n = 0; n < N_SUBSTEPS; n++)
        wave_step(s->u, s->v, s->rows, s->cols, sub_dt);

    /* ── Terrain ripple from wave ─────────────────────────────────────── */
    for (int c = 1; c < s->cols - 1; c++) {
        int sr = terrain_surf_row(s->rows, s->terrain_h[c], 0.f);
        if (sr >= s->rows) sr = s->rows - 1;
        float wave_at_surf = s->u[sr * s->cols + c];

        /* Integrate wave pressure into ripple displacement */
        s->terrain_d[c] += wave_at_surf * RIPPLE_SCALE * dt;
        /* Spring decay toward zero */
        s->terrain_d[c] *= powf(RIPPLE_DECAY, dt * SIM_FPS);
        /* Clamp to keep terrain visible */
        if (s->terrain_d[c] >  2.5f) s->terrain_d[c] =  2.5f;
        if (s->terrain_d[c] < -1.5f) s->terrain_d[c] = -1.5f;

        /* Spawn dust where the wavefront sweeps the surface */
        if (fabsf(wave_at_surf) > DUST_THRESH && (g_pseed & 0x1F) == 0)
            spawn_dust(s->parts, MAX_PARTS, (float)c,
                       (float)terrain_surf_row(s->rows, s->terrain_h[c],
                                               s->terrain_d[c]));
    }

    particles_tick(s->parts, MAX_PARTS, dt);

    s->shake_t += dt;
    s->flash    = fmaxf(0.f, s->flash - FLASH_DECAY * dt);
    s->time    += dt;
}

/* ===================================================================== */
/* §8  render                                                             */
/* ===================================================================== */

/*
 * Compute integer screen shake offset.
 * Magnitude decays exponentially; direction oscillates at SHAKE_FREQ.
 */
static void get_shake(const SimState *s, int *sr, int *sc)
{
    float mag = SHAKE_AMP * expf(-SHAKE_DECAY * s->shake_t);
    *sr = (int)(mag * sinf(s->shake_t * SHAKE_FREQ) * 0.5f);
    *sc = (int)(mag * cosf(s->shake_t * SHAKE_FREQ));
}

static void render_frame(const SimState *s, int trows, int tcols,
                          float fps, bool paused)
{
    int shake_r, shake_c;
    get_shake(s, &shake_r, &shake_c);

    int draw_rows = s->rows;   /* rows available for simulation          */

    /* ── 1. Wave field ────────────────────────────────────────────────── */
    for (int r = 0; r < draw_rows - TERRAIN_ROWS; r++) {
        for (int c = 0; c < s->cols; c++) {
            float pres = s->u[r * s->cols + c];
            float norm = pres / NORM_SCALE;

            char  ch   = ' ';
            attr_t at  = 0;

            if (norm > 0.06f) {
                float t = fminf(norm, 1.f);
                press_glyph(t, &at, &ch);
            } else if (norm < -0.06f) {
                /* Rarefaction trough: cold, dim dot */
                ch = '.';
                at = COLOR_PAIR(CP_RAREFY);
            } else {
                continue;   /* near-zero — leave as background space */
            }

            int tr = r + shake_r, tc = c + shake_c;
            if (tr < 0 || tr >= trows - 1 || tc < 0 || tc >= tcols) continue;
            attron(at);
            mvaddch(tr, tc, (chtype)(unsigned char)ch);
            attroff(at);
        }
    }

    /* ── 2. Terrain ───────────────────────────────────────────────────── */
    for (int c = 0; c < s->cols; c++) {
        float d       = s->terrain_d[c];
        float h       = s->terrain_h[c];
        int   surf    = terrain_surf_row(s->rows, h, d);
        int   surf_nd = terrain_surf_row(s->rows, h, 0.f);
        bool  ripple  = (fabsf(d) > 0.3f);

        for (int r = surf; r < s->rows; r++) {
            int tr = r + shake_r, tc = c + shake_c;
            if (tr < 0 || tr >= trows - 1 || tc < 0 || tc >= tcols) continue;

            attr_t at;
            char   ch;
            if (r == surf) {
                /* Surface row: use displaced color if rippling */
                at = COLOR_PAIR(ripple ? CP_TDISP : CP_TSURF);
                ch = ripple ? '~' : '^';
                if (ripple) at |= A_BOLD;
            } else if (r == surf_nd && r != surf) {
                /* Original surface, now buried under ripple heave */
                at = COLOR_PAIR(CP_TBASE);
                ch = '#';
            } else {
                at = COLOR_PAIR(CP_TBASE);
                ch = (r % 2 == 0) ? '#' : '=';
            }
            attron(at);
            mvaddch(tr, tc, (chtype)(unsigned char)ch);
            attroff(at);
        }
    }

    /* ── 3. Particles ─────────────────────────────────────────────────── */
    for (int i = 0; i < MAX_PARTS; i++) {
        const Particle *p = &s->parts[i];
        if (!p->alive) continue;

        int pr = (int)(p->y + 0.5f) + shake_r;
        int pc = (int)(p->x + 0.5f) + shake_c;
        if (pr < 0 || pr >= trows - 1 || pc < 0 || pc >= tcols) continue;

        float age    = 1.f - p->life / p->max_life;
        float bright = 1.f - age;

        char   ch;
        attr_t at;
        if (p->type == PT_DEBRIS) {
            ch = (bright > 0.6f) ? '*' : (bright > 0.3f) ? '+' : '.';
            at = COLOR_PAIR(CP_DEBRIS);
            if (bright > 0.5f) at |= A_BOLD;
        } else {
            ch = (bright > 0.5f) ? ':' : '.';
            at = COLOR_PAIR(CP_DUST);
        }
        attron(at);
        mvaddch(pr, pc, (chtype)(unsigned char)ch);
        attroff(at);
    }

    /* ── 4. Flash overlay ─────────────────────────────────────────────── */
    if (s->flash > 0.05f) {
        float f  = s->flash;
        char  ch = (f > 0.7f) ? '@' : (f > 0.4f) ? '#' : '+';
        attr_t at = COLOR_PAIR(CP_FLASH) | A_BOLD;
        /* Flood the inner region (near blast centre) with the flash */
        int radius = (int)(f * 14.f);
        for (int dr = -radius; dr <= radius; dr++) {
            for (int dc = -radius * 2; dc <= radius * 2; dc++) {
                float dist = sqrtf((float)(dr*dr) + (float)(dc*dc / 4));
                if (dist > radius) continue;
                int tr = s->blast_cy + dr + shake_r;
                int tc = s->blast_cx + dc + shake_c;
                if (tr < 0 || tr >= trows - 1 || tc < 0 || tc >= tcols) continue;
                attron(at);
                mvaddch(tr, tc, (chtype)(unsigned char)ch);
                attroff(at);
            }
        }
    }

    /* ── 5. HUD ───────────────────────────────────────────────────────── */
    {
        float cfl = WAVE_C / ((float)SIM_FPS * (float)N_SUBSTEPS)
                    * 1.41421356f;
        char buf[HUD_COLS];
        snprintf(buf, sizeof buf,
                 " nuke | t=%.1fs | %.0ffps | CFL=%.2f | "
                 "space:blast  r:reset  p:pause%s  q:quit ",
                 (double)s->time, (double)fps, (double)cfl,
                 paused ? "(ON)" : "");
        attron(COLOR_PAIR(CP_HUD) | A_BOLD);
        mvaddnstr(trows - 1, 0, buf, tcols - 1);
        attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
    }
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGWINCH) g_resize = 1;
    else               g_quit   = 1;
}

static void cleanup(void) { endwin(); }

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();

    int trows, tcols;
    getmaxyx(stdscr, trows, tcols);

    /* Simulation rows = terminal rows minus HUD row */
    SimState *s = sim_alloc(trows - 1, tcols);
    sim_blast(s);

    /* FPS counter */
    int64_t fps_timer = clock_ns();
    int     fps_frames = 0;
    float   fps_val    = 0.f;

    int64_t last = clock_ns();

    while (!g_quit) {
        /* ── Resize ──────────────────────────────────────────────────── */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, trows, tcols);
            s = sim_resize(s, trows - 1, tcols);
            sim_blast(s);
        }

        /* ── Input ───────────────────────────────────────────────────── */
        int key = getch();
        switch (key) {
            case 'q': case 27: g_quit = 1;           break;
            case ' ': case 'b': sim_blast(s);         break;
            case 'r':           sim_reset(s);         break;
            case 'p':           s->paused = !s->paused; break;
        }

        /* ── Tick ────────────────────────────────────────────────────── */
        int64_t now   = clock_ns();
        float   dt    = (float)(now - last) * 1e-9f;
        last = now;
        if (dt > 0.1f) dt = 0.1f;   /* cap at 100 ms (resize/lag guard) */

        sim_tick(s, dt);

        /* ── FPS counter ─────────────────────────────────────────────── */
        fps_frames++;
        if (now - fps_timer >= FPS_MS * NS_PER_MS) {
            float elapsed = (float)(now - fps_timer) * 1e-9f;
            fps_val    = (float)fps_frames / elapsed;
            fps_frames = 0;
            fps_timer  = now;
        }

        /* ── Draw ────────────────────────────────────────────────────── */
        erase();
        render_frame(s, trows, tcols, fps_val, s->paused);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }

    sim_free(s);
    return 0;
}
