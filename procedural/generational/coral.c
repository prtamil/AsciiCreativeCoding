/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * coral.c  —  Radial DLA coral / fungus, grown outward from a centre seed
 *
 * Diffusion-Limited Aggregation (DLA) grown from the middle of the screen:
 *   — A single seed is frozen at the centre.
 *   — "Spore" walkers are launched from a circle just outside the current
 *     cluster and random-walk inward (a gravity-free drift toward the
 *     centre) until they touch the aggregate and freeze where they stand.
 *   — Because the growing tips screen the interior from incoming walkers,
 *     the cluster cannot fill in solid: it ramifies into branches — the
 *     self-similar dendrite that makes DLA look like coral, lichen, or
 *     fungus spreading from a spore.
 *
 * Colour encoding (radial rainbow):
 *   Each cell is coloured at freeze time by its distance from the centre:
 *     centre → coral red ; middle → violet / yellow / lime ;
 *     tips   → teal / lemon-green
 *   so the reef reads as concentric colour bands radiating outward,
 *   independent of the order cells happened to freeze.
 *
 * Presets: 15 named visual styles (palette + branchiness + thin/fat
 *   contact + glyphs). The reef grows ONCE and then HOLDS on the finished
 *   piece — no auto-restart, no auto style switch. The user starts a fresh
 *   grow with n / p (switch preset) or r (regrow the same preset).
 *
 * Keys:
 *   q / ESC   quit
 *   spc       pause / resume
 *   n / p     next / previous preset
 *   r         restart growth (same preset)
 *   + =       more walkers
 *   -         fewer walkers
 *   ] [       faster / slower
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra coral.c -o coral -lncurses -lm
 *
 * Sections
 * --------
 *   §1   config
 *   §2   clock
 *   §3   color
 *   §3.5 presets — 15 visual styles
 *   §4   grid
 *   §5   walker
 *   §6   scene
 *   §7   screen
 *   §8   app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Diffusion-Limited Aggregation (DLA), radial form. A seed
 *                  sits at the centre; random walkers launched from a
 *                  surrounding circle stick on first contact (probability
 *                  stick_prob, set per preset). Walkers drift inward so they
 *                  reliably reach the cluster instead of wandering off.
 *
 * Physics        : The branching IS the screening effect — a protruding tip
 *                  is far more likely to catch a wandering walker than a
 *                  sheltered interior gap, so growth concentrates at the
 *                  extremities and runs away into dendrites. The same
 *                  instability shapes mineral dendrites, electrodeposits,
 *                  lichen, and coral.
 *
 * Math           : DLA clusters are fractal: in 2D the mass within radius r
 *                  scales as r^D with D ≈ 1.71 — scale-free, branchy at every
 *                  magnification. Higher sticking probability → stronger
 *                  screening → more open/ramified; lower → denser, blob-like.
 *
 * References     : ALGORITHM — DLA, fractal growth, the screening instability
 *                  • Witten & Sander (1981), "Diffusion-Limited Aggregation,
 *                    a Kinetic Critical Phenomenon", Phys. Rev. Lett. 47,
 *                    1400. THE founding paper: the model, the screening that
 *                    forces branching, and the fractal dimension.
 *                  • Witten & Sander (1983), "Diffusion-limited aggregation",
 *                    Phys. Rev. B 27, 5686. Fuller treatment of D ≈ 1.71 and
 *                    the cluster's scaling.
 *                  • Vicsek, Tamás (1992), "Fractal Growth Phenomena", 2nd
 *                    ed., World Scientific. Textbook on DLA and its relatives
 *                    (dielectric breakdown, electrodeposition, viscous
 *                    fingering) — why so many systems share this morphology.
 *                  • Mandelbrot (1982), "The Fractal Geometry of Nature",
 *                    W. H. Freeman. The wider language of self-similarity and
 *                    fractal dimension this growth lives in.
 *                  • Bourke, Paul — "DLA: Diffusion Limited Aggregation",
 *                    paulbourke.net/fractals/dla/. Practical, code-oriented
 *                    notes: launch/kill circles, the inward-walk speed-up.
 *                  • Procedural Content Generation Wiki / RogueBasin — DLA for
 *                    organic level features (caves, ore veins, lightning).
 *
 *                  RENDERING — terminal / ASCII output
 *                  • Padala, Pradeep — "NCURSES Programming HOWTO" (TLDP).
 *                    Reference for the erase→draw→doupdate frame model,
 *                    colour pairs, and non-blocking input used here.
 *                  • documentation/COLOR.md (this project) — 256-colour cube
 *                    layout and the palette-brightness rule the presets obey.
 * ─────────────────────────────────────────────────────────────────────── */

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

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    WALKER_MIN      =  10,
    WALKER_DEFAULT  = 150,
    WALKER_MAX      = 400,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    N_CORAL_COLORS  =   6,

    /* Radial-growth tuning shared by all presets (DLA from a centre seed).
     * Branchiness, contact rule, drift and colours vary per preset below. */
    SPAWN_MARGIN    =   5,   /* launch walkers this many cells beyond tips */
    KILL_MARGIN     =  20,   /* relaunch a walker if it strays this far    */

    FPS_UPDATE_MS   = 500,

    /* Frame pacing. RENDER_FPS_CAP throttles screen redraws (independent of
     * the simulation tick rate Control.sim_fps). DT_CAP_MS clamps a single
     * frame's measured dt so a long stall can't trigger a spiral of death. */
    RENDER_FPS_CAP  =  60,
    DT_CAP_MS       = 100,

    /* Rows reserved for the HUD: one data bar (top) + one action bar
     * (bottom). The reef is drawn in the band between them, so neither
     * bar ever overlaps the coral. */
    HUD_ROW_TOP     =   1,
    HUD_ROW_BOTTOM  =   1,
};

/* 2*pi — launch walkers at a uniform random angle on the spawn circle. */
#define TAU          6.2831853f

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
 * Colour-pair slots. The six CORAL bands are the radial gradient (centre
 * → tip); their actual colours are NOT fixed here — they are supplied by
 * the active preset (see §3.5) and re-bound with init_pair on every switch.
 * The HUD/HINT pairs are fixed (yellow / cyan, per the project standard).
 */
typedef enum {
    COL_CORAL_1 = 1,   /* radial band 1 (centre)                      */
    COL_CORAL_2 = 2,
    COL_CORAL_3 = 3,
    COL_CORAL_4 = 4,
    COL_CORAL_5 = 5,
    COL_CORAL_6 = 6,   /* radial band 6 (outer tips)                  */
    COL_WALKER  = 7,   /* drifting spore particles                    */
    COL_HUD     = 8,   /* HUD data bar    — bright yellow             */
    COL_HINT    = 9,   /* HUD action bar  — bright cyan               */
} ColorID;

/* ===================================================================== */
/* §3.5  presets — 15 unique visual styles                                */
/* ===================================================================== */

/*
 * Preset — a complete visual style. Switching preset (n/p) changes how the
 * reef LOOKS and GROWS but not the algorithm. The reef grows once to the
 * edge and then holds; n/p or r begins a fresh grow.
 *
 *   palette[6]      radial colour bands, centre → tip (xterm-256 indices,
 *                   all in the bright half so they read on black)
 *   stick_prob      freeze chance on first contact. High → strong
 *                   screening → wispy open branches; low → walkers probe
 *                   deeper → dense, blobby growth.
 *   eight_neighbour true counts diagonal contact too → fatter clumps;
 *                   false (von Neumann) → thin dendrites.
 *   inward_bias     % of steps biased toward the centre (drift speed).
 *   glyphs[6]       one ASCII char per band, dense → light.
 *   bold_tips       A_BOLD the two outermost bands for extra glow.
 */
typedef struct {
    const char *name;
    short       palette[N_CORAL_COLORS];
    short       walker_col;
    float       stick_prob;
    bool        eight_neighbour;
    int         inward_bias;
    const char *glyphs;
    bool        bold_tips;
} Preset;

#define N_PRESETS 15

static const Preset presets[N_PRESETS] = {
  /*  name        palette (centre→tip)               walk  stick  8nb  bias glyphs    bold */
  { "RAINBOW",  {203,207,226,118, 86,154}, 251, 0.90f, false, 35, "#++**^", true  },
  { "EMBER",    {130,166,202,208,214,220}, 244, 0.92f, false, 30, "##**^^", true  },
  { "OCEAN",    { 26, 32, 39, 45, 51,123}, 245, 0.88f, false, 35, "#++**:", true  },
  { "NEON",     {201,165,129, 93, 57, 51}, 252, 0.96f, false, 40, "**####", true  },
  { "FOREST",   { 28, 34, 40, 70,106,154}, 244, 0.85f, false, 30, "##++^^", false },
  { "ICE",      { 24, 31, 74,117,159,195}, 250, 0.90f, false, 35, "::****", true  },
  { "GOLD",     { 94,136,178,214,220,229}, 244, 0.90f, false, 30, "%%##++", true  },
  { "AMETHYST", { 55, 91,127,163,170,219}, 252, 0.88f, false, 35, "#++**^", true  },
  { "TOXIC",    { 34, 40, 46, 82,118,154}, 250, 0.70f, true,  30, "######", false },
  { "MONO",     {241,245,248,251,253,255}, 246, 0.90f, false, 35, "##**::", false },
  { "SUNSET",   { 54, 90,126,168,204,220}, 245, 0.88f, false, 35, "#++**^", true  },
  { "ROSE",     {161,168,205,211,217,224}, 252, 0.90f, false, 35, "..oo**", true  },
  { "ELECTRIC", { 27, 33, 39, 45, 51, 87}, 252, 0.97f, false, 25, "::****", true  },
  { "LAVA",     { 52, 88,124,160,196,202}, 244, 0.65f, true,  35, "######", true  },
  { "SPRING",   {120,156,192,228,222,159}, 254, 0.88f, false, 30, "**++::", false },
};

/* preset_apply — bind the preset's palette into the coral colour pairs. */
static void preset_apply(const Preset *p)
{
    if (COLORS >= 256) {
        for (int i = 0; i < N_CORAL_COLORS; i++)
            init_pair(COL_CORAL_1 + i, p->palette[i], COLOR_BLACK);
        init_pair(COL_WALKER, p->walker_col, COLOR_BLACK);
    } else {
        init_pair(COL_CORAL_1, COLOR_RED,     COLOR_BLACK);
        init_pair(COL_CORAL_2, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_CORAL_3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(COL_CORAL_4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(COL_CORAL_5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_CORAL_6, COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_WALKER,  COLOR_WHITE,   COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(COL_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    preset_apply(&presets[0]);
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Grid — the frozen coral aggregate, grown radially from a centre seed.
 *
 *   cells[y][x] == 0  →  empty
 *   cells[y][x] == n  →  frozen, draw with COLOR_PAIR(n)
 *
 * Colour is assigned at freeze time by distance from the centre (see
 * grid_color_for_radius): centre = COL_CORAL_1, edge = COL_CORAL_6 — a
 * radial rainbow baked in once and never changed after freezing.
 *
 * max_radius is the distance of the farthest frozen cell from the centre;
 * walkers launch from just beyond it. max_radius reaching reset_radius is
 * what tells the scene the growth is complete.
 *
 * WHY a single buffer (no double buffer): unlike a synchronous CA, DLA is
 * SEQUENTIAL — a cell freezes permanently the instant a walker sticks, and
 * later walkers must see it immediately to grow off it. So a frozen cell is
 * never re-evaluated; one array suffices and in-place writes are correct.
 *
 * Ref: Witten & Sander (1981) — the DLA model; see the References block.
 */
typedef struct {
    /* the aggregate */
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];  /* 0 = empty; n = frozen,
                                                   * drawn with COLOR_PAIR(n) */
    int     frozen_count;   /* live tally of frozen cells (HUD readout)     */

    /* radial geometry — the basis of growth, colour, and the done test */
    int     ccx, ccy;       /* centre cell — the seed / growth origin       */
    int     max_radius;     /* distance of the farthest frozen cell         */
    int     reset_radius;   /* growth is "done" once max_radius reaches this
                             * (≈ the nearer screen half); also the divisor
                             * that maps radius → colour band               */
    int     rows, cols;     /* grid extent in cells                         */

    /* Active-preset growth/draw style, copied in by grid_init so the walker
     * and renderer read it here instead of reaching back to presets[]. */
    float       stick_prob;       /* freeze chance on contact (branchiness) */
    bool        eight_neighbour;  /* diagonal contact too → fatter clumps   */
    int         inward_bias;      /* % of walker steps drifting to centre   */
    const char *glyphs;           /* 6 band chars; points into presets[]    */
    bool        bold_tips;        /* A_BOLD the two outer bands             */
} Grid;

/* grid_radius() — integer distance of (cx,cy) from the centre seed. */
static int grid_radius(const Grid *g, int cx, int cy)
{
    int dx = cx - g->ccx, dy = cy - g->ccy;
    return (int)(sqrtf((float)(dx * dx + dy * dy)) + 0.5f);
}

/*
 * grid_color_for_radius() — map distance-from-centre to a colour band.
 *   radius 0            → COL_CORAL_1 (centre)
 *   radius reset_radius → COL_CORAL_6 (outer tips)
 */
static uint8_t grid_color_for_radius(const Grid *g, int radius)
{
    int idx = (int)((float)radius / (float)g->reset_radius * N_CORAL_COLORS);
    if (idx < 0)               idx = 0;
    if (idx >= N_CORAL_COLORS) idx = N_CORAL_COLORS - 1;
    return (uint8_t)(idx + 1);
}

static void grid_place_seed(Grid *g)
{
    g->cells[g->ccy][g->ccx] = COL_CORAL_1;
    g->frozen_count++;
}

static void grid_init(Grid *g, int cols, int rows, const Preset *p)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols < 1) cols = 1;            /* never let the HUD reservation   */
    if (rows < 1) rows = 1;            /* shrink the grid below one cell  */
    memset(g->cells, 0, sizeof g->cells);
    g->frozen_count = 0;
    g->cols         = cols;
    g->rows         = rows;
    g->ccx          = cols / 2;
    g->ccy          = rows / 2;
    g->max_radius   = 0;

    /* Adopt the active preset's growth + draw style. */
    g->stick_prob      = p->stick_prob;
    g->eight_neighbour = p->eight_neighbour;
    g->inward_bias     = p->inward_bias;
    g->glyphs          = p->glyphs;
    g->bold_tips       = p->bold_tips;

    /* Grow until the cluster reaches the nearer screen edge. */
    int half = (cols < rows ? cols : rows) / 2;
    g->reset_radius = half > 1 ? half - 1 : 1;

    grid_place_seed(g);
}

static void grid_freeze(Grid *g, int cx, int cy)
{
    int r = grid_radius(g, cx, cy);
    g->cells[cy][cx] = grid_color_for_radius(g, r);
    g->frozen_count++;
    if (r > g->max_radius) g->max_radius = r;
}

static bool grid_frozen(const Grid *g, int cx, int cy)
{
    if (cx < 0 || cx >= g->cols || cy < 0 || cy >= g->rows) return false;
    return g->cells[cy][cx] != 0;
}

/*
 * grid_touching() — is (cx,cy) adjacent to the cluster? A walker that lands
 * on such a cell is a candidate to freeze. The preset chooses the contact
 * rule: 4-neighbour (von Neumann) keeps branches thin and dendritic;
 * 8-neighbour also counts diagonals → fuller, blobbier growth.
 */
static bool grid_touching(const Grid *g, int cx, int cy)
{
    if (grid_frozen(g, cx + 1, cy) || grid_frozen(g, cx - 1, cy)
     || grid_frozen(g, cx, cy + 1) || grid_frozen(g, cx, cy - 1))
        return true;
    if (g->eight_neighbour)
        return grid_frozen(g, cx + 1, cy + 1) || grid_frozen(g, cx - 1, cy + 1)
            || grid_frozen(g, cx + 1, cy - 1) || grid_frozen(g, cx - 1, cy - 1);
    return false;
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            uint8_t col = g->cells[cy][cx];
            if (col == 0) continue;

            attr_t attr = COLOR_PAIR((int)col);
            if (g->bold_tips && col >= 5) attr |= A_BOLD;   /* glow the tips */

            /* glyph for this radial band (preset-defined, dense → light) */
            char ch = (col >= 1 && col <= N_CORAL_COLORS) ? g->glyphs[col - 1] : '*';

            wattron(w, attr);
            mvwaddch(w, cy + HUD_ROW_TOP, cx, (chtype)(unsigned char)ch);
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  walker                                                             */
/* ===================================================================== */

/*
 * Walker — one drifting "spore" particle searching for the cluster.
 *
 * Launched on a circle just outside the current cluster, it random-walks
 * with a mild bias toward the centre (Grid.inward_bias) so it reliably
 * reaches the aggregate instead of diffusing away. On first contact it
 * freezes with probability Grid.stick_prob; the screening of the outer
 * tips is what turns the frozen set into branches rather than a disc.
 * (Both come from the active preset — see §3.5.)
 */
typedef struct {
    int  cx, cy;     /* current cell in grid space (the random-walk position) */
    bool active;     /* false = slot unused (walkers[] is a fixed pool of size
                      * WALKER_MAX; only the first Control.n_walkers are live) */
} Walker;

/*
 * walker_spawn() — launch on the spawn circle: a uniform random angle at
 * radius (max_radius + SPAWN_MARGIN), clamped into the grid. Spawning just
 * beyond the tips (rather than at the screen edge) means the walker finds
 * the cluster in a few steps, keeping the animation lively.
 */
static void walker_spawn(Walker *w, const Grid *g)
{
    float radius = (float)g->max_radius + (float)SPAWN_MARGIN;
    float theta  = ((float)(rand() % 1000) / 1000.0f) * TAU;

    int cx = g->ccx + (int)(radius * cosf(theta));
    int cy = g->ccy + (int)(radius * sinf(theta));
    if (cx < 0) cx = 0; else if (cx >= g->cols) cx = g->cols - 1;
    if (cy < 0) cy = 0; else if (cy >= g->rows) cy = g->rows - 1;

    w->cx = cx;
    w->cy = cy;
    w->active = true;
}

/* try_stick() — freeze the walker's current cell if it touches the cluster
 * and wins the preset's stick_prob roll. Returns true on freeze. */
static bool try_stick(Walker *w, Grid *g)
{
    if (grid_frozen(g, w->cx, w->cy) || !grid_touching(g, w->cx, w->cy))
        return false;
    if ((float)rand() / (float)RAND_MAX >= g->stick_prob)
        return false;
    grid_freeze(g, w->cx, w->cy);
    return true;
}

/* step_toward — the unit step (-1, 0, +1) that moves `from` one cell toward
 * `to`. The atom of the walker's inward drift. */
static inline int step_toward(int from, int to)
{
    return (from < to) - (from > to);
}

/*
 * out_of_play — has (x,y) left the active region? True if off the grid, or
 * strayed more than KILL_MARGIN beyond the cluster. Either way the walker
 * is abandoned and relaunched rather than wandering uselessly far away.
 */
static bool out_of_play(const Grid *g, int x, int y)
{
    if (x < 0 || x >= g->cols || y < 0 || y >= g->rows) return true;
    return grid_radius(g, x, y) > g->max_radius + KILL_MARGIN;
}

/*
 * walker_pick_step — choose this tick's move: inward_bias% of the time a
 * step toward the centre (so the walker reliably reaches the cluster),
 * else a uniform random 4-direction step. Pure read; returns via dx,dy.
 */
static void walker_pick_step(const Walker *w, const Grid *g, int *dx, int *dy)
{
    *dx = 0;
    *dy = 0;
    if (rand() % 100 < g->inward_bias) {
        int tox = step_toward(w->cx, g->ccx);
        int toy = step_toward(w->cy, g->ccy);
        if ((rand() & 1) && tox) *dx = tox;
        else if (toy)            *dy = toy;
        else                     *dx = tox;
    } else {
        switch (rand() % 4) {
            case 0:  *dx =  1; break;
            case 1:  *dx = -1; break;
            case 2:  *dy =  1; break;
            default: *dy = -1; break;
        }
    }
}

/*
 * walker_tick() — advance one step toward the cluster and try to stick.
 * Returns true when the walker froze a cell (caller should relaunch it).
 */
static bool walker_tick(Walker *w, Grid *g)
{
    if (!w->active) return false;

    int dx, dy;
    walker_pick_step(w, g, &dx, &dy);          /* drift toward the cluster */
    int nx = w->cx + dx;
    int ny = w->cy + dy;

    if (out_of_play(g, nx, ny)) {              /* wandered off → relaunch  */
        walker_spawn(w, g);
        return false;
    }

    /* About to step onto the cluster → try to stick where we stand. */
    if (grid_frozen(g, nx, ny))
        return try_stick(w, g);

    /* Move, then try to stick at the new cell. */
    w->cx = nx;
    w->cy = ny;
    return try_stick(w, g);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Control — the user-tunable knobs, gathered in one place: every field is
 * something a key changes, nothing the algorithm decides on its own. Bounds
 * for each live in §1 config; app_handle_key clamps to them.
 */
typedef struct {
    int  preset_idx;      /* n/p   — visual style, index into presets[]   */
    int  n_walkers;       /* +/-   — active drifting spores                */
    int  sim_fps;         /* [ ]   — simulation tick rate (Hz)            */
    bool paused;          /* space — freeze the simulation                */
} Control;

/*
 * Scene — the whole showcase in one structure, three concerns kept apart:
 *   grid + walkers   WHAT is simulated (the reef and the spores building it)
 *   ctrl             HOW the user drives it (the knobs)
 *   done             WHERE in the lifecycle (growth finished → hold)
 */
typedef struct {
    Grid    grid;
    Walker  walkers[WALKER_MAX];
    Control ctrl;
    bool    done;
} Scene;

/*
 * scene_build — (re)grow the reef at (cols,rows) in the CURRENT preset's
 * style: bind its palette, seed the grid with its growth params, relaunch
 * the walker swarm. The shared core of init / regrow / preset-switch.
 */
static void scene_build(Scene *s, int cols, int rows)
{
    const Preset *p = &presets[s->ctrl.preset_idx];
    preset_apply(p);                          /* palette → colour pairs   */
    grid_init(&s->grid, cols, rows, p);       /* fresh seed + growth style */
    s->done = false;

    for (int i = 0; i < WALKER_MAX; i++)
        s->walkers[i].active = false;
    for (int i = 0; i < s->ctrl.n_walkers; i++)
        walker_spawn(&s->walkers[i], &s->grid);
}

/* scene_init — full setup for startup / resize. Resets walker count and
 * pause; preserves the chosen preset and speed (set once at startup). */
static void scene_init(Scene *s, int cols, int rows)
{
    s->ctrl.n_walkers = WALKER_DEFAULT;
    s->ctrl.paused    = false;
    scene_build(s, cols, rows);
}

/* scene_regrow — restart growth at the current size, keeping preset, walker
 * count and pause state. Triggered only by the user: r (same preset) or
 * n/p (switch preset) — there is no automatic restart. */
static void scene_regrow(Scene *s)
{
    scene_build(s, s->grid.cols, s->grid.rows);
}

static void scene_tick(Scene *s)
{
    if (s->ctrl.paused || s->done) return;

    /*
     * Growth complete: the cluster radius has reached the screen edge.
     * Freeze on the finished reef — NO auto-restart. The user starts a
     * fresh grow with n/p (switch preset) or r (same preset).
     */
    if (s->grid.max_radius >= s->grid.reset_radius) {
        s->done = true;
        return;
    }

    for (int i = 0; i < s->ctrl.n_walkers; i++) {
        bool froze = walker_tick(&s->walkers[i], &s->grid);
        if (froze)
            walker_spawn(&s->walkers[i], &s->grid);
    }
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    grid_draw(&s->grid, w);

    if (s->done) return;          /* finished reef — no drifting spores */

    wattron(w, COLOR_PAIR(COL_WALKER) | A_DIM);
    for (int i = 0; i < s->ctrl.n_walkers; i++) {
        const Walker *wk = &s->walkers[i];
        if (!wk->active) continue;
        if (wk->cy < 0 || wk->cy >= s->grid.rows) continue;
        if (wk->cx < 0 || wk->cx >= s->grid.cols) continue;
        if (s->grid.cells[wk->cy][wk->cx] == 0)
            mvwaddch(w, wk->cy + HUD_ROW_TOP, wk->cx, (chtype)'.');
    }
    wattroff(w, COLOR_PAIR(COL_WALKER) | A_DIM);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the terminal's current size, cached from getmaxyx(). WHY cache
 * it: the dimensions change only on a resize (SIGWINCH → screen_resize
 * re-reads them), yet every frame needs them to size the grid and pin the
 * HUD bars. Units are character cells — this is cell-space rendering, no
 * sub-pixel coordinates.
 */
typedef struct {
    int cols;   /* terminal width  in character columns */
    int rows;   /* terminal height in character rows    */
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

/*
 * hud_bar — paint one full-width status bar on `row`: fill the row with
 * spaces in `pair`, then write `buf` clipped to the terminal width with
 * mvaddnstr. Clipping (not mvprintw) is what stops an over-long string
 * from wrapping down onto the reef. Drives both the top data bar and the
 * bottom action bar.
 */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    if (row < 0 || cols < 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, stdscr);

    const Control *c = &sc->ctrl;

    /* Row 0 — DATA: identity, active preset, live state, one clipped line. */
    char data[160];
    snprintf(data, sizeof data,
             " CORAL DLA  %s (%d/%d)  %s  frozen:%-5d  walkers:%-3d  %5.1f fps  %d Hz ",
             presets[c->preset_idx].name, c->preset_idx + 1, N_PRESETS,
             c->paused ? "PAUSED " : sc->done ? "DONE   " : "growing",
             sc->grid.frozen_count, c->n_walkers, fps, c->sim_fps);

    /* Last row — ACTIONS only: every interactive key, nothing else. */
    static const char *keys =
        " q:quit  spc:pause  n/p:preset  r:reset  +/-:walkers  [/]:speed ";

    hud_bar(0,           s->cols, COL_HUD,  data);
    hud_bar(s->rows - 1, s->cols, COL_HINT, keys);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — the top-level program object: the showcase, the screen, and the two
 * signal flags. WHY a single g_app global: POSIX signal handlers take no
 * user pointer, so the handlers below reach the program through this one
 * well-known instance — the only global, by design; everything else is
 * passed by pointer.
 */
typedef struct {
    Scene                 scene;       /* the showcase (reef + control)      */
    Screen                screen;      /* cached terminal size               */
    volatile sig_atomic_t running;     /* 0 ⇒ exit main loop (SIGINT/TERM).
                                        * volatile sig_atomic_t: the only
                                        * type safe to touch in a handler    */
    volatile sig_atomic_t need_resize; /* 1 ⇒ re-read size (SIGWINCH)         */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* (Re)build the scene to fit the screen, reserving the HUD rows so the
 * reef lives in the band between the two bars. The single place that knows
 * the screen→grid size mapping. */
static void app_build_scene(App *app)
{
    scene_init(&app->scene, app->screen.cols,
               app->screen.rows - HUD_ROW_TOP - HUD_ROW_BOTTOM);
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_build_scene(app);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Control *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        c->paused = !c->paused;
        break;

    case 'n': case 'N':   /* next preset */
        c->preset_idx = (c->preset_idx + 1) % N_PRESETS;
        scene_regrow(&app->scene);
        break;

    case 'p': case 'P':   /* previous preset */
        c->preset_idx = (c->preset_idx + N_PRESETS - 1) % N_PRESETS;
        scene_regrow(&app->scene);
        break;

    case 'r': case 'R':   /* restart growth, same preset */
        scene_regrow(&app->scene);
        break;

    case '=': case '+':
        if (c->n_walkers < WALKER_MAX) {
            int i = c->n_walkers++;
            walker_spawn(&app->scene.walkers[i], &app->scene.grid);
        }
        break;

    case '-':
        if (c->n_walkers > WALKER_MIN)
            c->n_walkers--;
        break;

    case ']':
        c->sim_fps += SIM_FPS_STEP;
        if (c->sim_fps > SIM_FPS_MAX) c->sim_fps = SIM_FPS_MAX;
        break;

    case '[':
        c->sim_fps -= SIM_FPS_STEP;
        if (c->sim_fps < SIM_FPS_MIN) c->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/*
 * app_step_simulation — FIXED-TIMESTEP update: bank the frame's real
 * elapsed time, then spend it one whole TICK at a time (rate = sim_fps),
 * leaving the sub-tick remainder in *sim_accum for next frame. Decouples
 * simulation speed from frame rate.
 */
static void app_step_simulation(App *app, int64_t dt, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->scene.ctrl.sim_fps);

    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene);
        *sim_accum -= tick_ns;
    }
}

/*
 * app_pace_frame — sleep so each rendered frame lasts about one
 * RENDER_FPS_CAP period, regardless of how long this frame's work took.
 * frame_start = when this iteration began; frame_dt = the previous frame's
 * measured length. Holds a steady cap and keeps the process off a busy-spin.
 */
static void app_pace_frame(int64_t frame_start, int64_t frame_dt)
{
    int64_t budget_ns  = NS_PER_SEC / RENDER_FPS_CAP;
    int64_t elapsed_ns = clock_ns() - frame_start + frame_dt;
    clock_sleep_ns(budget_ns - elapsed_ns);
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
    app->scene.ctrl.sim_fps = SIM_FPS_DEFAULT;   /* set once; survives resize */

    screen_init(&app->screen);
    app_build_scene(app);

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

        /* Measure this frame's real elapsed time, capped so a long stall
         * can't trigger a spiral of death; advance the clock. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_MS * NS_PER_MS) dt = DT_CAP_MS * NS_PER_MS;

        /* Advance the simulation by that much real time. */
        app_step_simulation(app, dt, &sim_accum);

        /* Update the FPS readout once per FPS_UPDATE_MS window. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Sleep to the frame cap, THEN draw (steady pacing). */
        app_pace_frame(frame_time, dt);
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        /* Drain one input event. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
