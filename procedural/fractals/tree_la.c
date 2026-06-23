/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * tree_la.c  —  growing electric fractals (Dielectric Breakdown Model)
 *
 * We spread a fake "voltage" across the grid, then grow a branching shape one cell
 * at a time, always favouring cells where the voltage is highest. That simple rule
 * grows lightning bolts, coral, frost ferns and trees — the shape depends only on
 * where we put the high-voltage edge and the low-voltage seed. The `eta` knob
 * controls how greedy growth is about chasing the very hottest spots: high = jagged
 * and spindly, low = round and bushy.
 *
 * References (the ideas the code can't explain on its own):
 *   - Niemeyer, Pietronero & Wiesmann (1984), "Fractal Dimension of Dielectric
 *     Breakdown," Phys. Rev. Lett. 52(12) — this is exactly the model implemented.
 *   - Witten & Sander (1981), "Diffusion-Limited Aggregation," Phys. Rev. Lett.
 *     47(19) — the eta=1 special case this generalises.
 *   - Numerical Recipes (3rd ed.), Ch. 20 — the relaxation method used to solve
 *     the voltage field in §5.
 *   Sister file: l_system.c grows trees a different way (grammar/turtle rules).
 *
 * Keys:
 *   q / ESC   quit            r       reset
 *   p / spc   pause           1..5    pick a preset (Tree/Lightning/Coral/Frost/Discharge)
 *   t / T     cycle theme     e / E   raise / lower eta
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra tree_la.c -o tree_la -lncurses -lm
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

enum {
    ROWS_MAX        =  80,
    COLS_MAX        = 300,

    RENDER_FPS      =  30,

    N_RELAX         =   8,   /* how many times we smooth the voltage per frame */
    N_GROW          =   1,   /* how many cells we add per frame                */

    N_THEMES        =   6,
    N_AGE_LEVELS    =   5,   /* five shades from newest to oldest growth        */

    CP_HUD          =   1,   /* yellow, used for the readout (same in every theme) */
    CP_A0           =   2,   /* the five age colours live at CP_A0 .. CP_A0+4      */
    CP_HINT         =   7,   /* cyan, used for the key hints (same in every theme) */

    HUD_TOP_ROWS    =   2,   /* top two rows belong to the readout, not the grid */
    HUD_BOT_ROWS    =   1,   /* bottom row belongs to the key hints              */

    FPS_UPDATE_MS   = 500,
    FROST_SPACING   =   4,   /* Frost plants a seed every 4th column along the floor */
};

/* How we sort growth by age into the five shades: a cell counts as tier i while it
 * is no older than AGE_THRESH[i] steps. Last entry is "everything older". */
static const int AGE_THRESH[N_AGE_LEVELS] = { 5, 20, 80, 300, INT32_MAX };

#define ETA_MIN   1.0f
#define ETA_MAX   4.0f
#define ETA_STEP  0.25f
#define ETA_DEFAULT_TREE      1.5f
#define ETA_DEFAULT_LIGHTNING 2.5f
#define ETA_DEFAULT_CORAL     2.0f
#define ETA_DEFAULT_FROST     2.0f
#define ETA_DEFAULT_DISCHARGE 2.5f

#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS  1000000LL

#define STENCIL_AVG_W   0.25f   /* smoothing a cell means averaging its 4 neighbours (×1/4) */
#define FRAME_DT_CAP_MS 200     /* ignore frame gaps longer than this so a pause won't wreck the fps number */

/* A preset is one ready-made setup: where the starting seed sits and which screen
 * edge holds the high voltage. Those two choices alone decide which way the shape
 * grows and what it ends up looking like (see §5 and §6). */
typedef enum {
    PRESET_TREE = 0, PRESET_LIGHTNING, PRESET_CORAL,
    PRESET_FROST, PRESET_DISCHARGE, N_PRESETS
} Preset;

static const char *PRESET_NAME[N_PRESETS] = {
    "Tree", "Lightning", "Coral", "Frost", "Discharge"
};

/* ── the data: the voltage field, the growing shape, and the scene that holds them ── */

/*
 * Field — the "voltage" at every cell on the grid. This is the landscape that steers
 * growth: cells with the most voltage are the most likely to grow next. We hold it at
 * 1 along the chosen source edge and at 0 everywhere the shape already exists, then
 * smooth the empty middle so voltage flows gently between them (§5 does the smoothing).
 * The smoothing naturally piles up voltage at exposed tips, which is why the shapes
 * come out spiky and branching. One number per cell.
 *   Refs: Niemeyer et al. 1984; Numerical Recipes Ch. 20 for the smoothing method.
 */
typedef struct {
    float phi[ROWS_MAX][COLS_MAX];   /* voltage, 0..1: 1 on the source edge, 0 on the shape */
} Field;

/*
 * Aggregate — the growing shape itself: the set of cells that have joined so far.
 * Cells join one at a time, each new cell snapping on next to an existing one (§6).
 * We remember when each cell joined so the renderer can colour fresh growth bright
 * and let old growth fade. Ref: Witten & Sander 1981 (the growing-cluster idea).
 */
typedef struct {
    uint8_t  occupied[ROWS_MAX][COLS_MAX];  /* 1 if this cell is part of the shape         */
    uint16_t age     [ROWS_MAX][COLS_MAX];  /* the step number when this cell joined.       *
                                             * 16 bits is plenty: the grid holds at most    *
                                             * ROWS_MAX*COLS_MAX = 24000 cells, well under  *
                                             * 65536, so the count never overflows.         */
    int      step;                          /* counts up by one per cell added — this is    *
                                             * "now", which we subtract from age to get how *
                                             * old a cell is                                 */
    int      size;                          /* how many cells the shape has                  */
} Aggregate;

#define CELLS_MAX (ROWS_MAX * COLS_MAX)

/*
 * Frontier — the list of empty cells touching the shape, i.e. the cells that could
 * grow next. It's purely a speed trick, not part of the model: keeping this short
 * list up to date lets each growth step look at only the candidates instead of
 * re-scanning the entire grid. We keep it current as we go — a cell leaves the list
 * the moment it joins the shape, and its empty neighbours get added.
 *   `slot` is a lookup the other way: given a cell, where does it sit in cr/cc (or
 *   -1 if it's not in the list). That lets us add and remove cells instantly by
 *   swapping the last entry into the freed spot. Only the simulation touches this;
 *   the renderer never does.
 */
typedef struct {
    uint16_t cr[CELLS_MAX];              /* row of each candidate cell    */
    uint16_t cc[CELLS_MAX];              /* column of each candidate cell */
    int      slot[ROWS_MAX][COLS_MAX];   /* cell -> its spot in cr/cc, or -1 if not listed */
    int      count;                      /* how many candidates there are */
} Frontier;

/*
 * Scene — everything the simulation needs in one bundle (no scattered globals): the
 * growing shape, the voltage field that steers it, the candidate list that speeds it
 * up, plus the dials (preset, eta, theme) and the random-number state. It knows
 * nothing about the terminal or screen size. Only scene_tick() (§7) ever changes it.
 */
typedef struct {
    int        rows, cols;   /* the grid size in use (never above ROWS/COLS_MAX)  */

    Aggregate  aggregate;    /* §6 the growing shape                              */
    Field      field;        /* §5 the voltage that steers the growth             */
    Frontier   frontier;     /* §6 the cells that could grow next                 */

    Preset     preset;       /* which ready-made setup we're running              */
    float      eta;          /* greed dial: high = spiky, low = round             */
    int        theme;        /* colour scheme; affects looks only                 */
    bool       paused;       /* when true, scene_tick does nothing                */

    uint32_t   rng;          /* random-number state; the only randomness in growth */
} Scene;

/* ===================================================================== */
/* §2  clock                                                             */
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
/* §3  rng — per-scene linear congruential generator                     */
/* ===================================================================== */

/* Next random number between 0 and 1. We keep the state on the Scene rather than in a
 * hidden global so growth is the only thing pulling random numbers. */
static float lcg_f(Scene *s)
{
    s->rng = s->rng * 1664525u + 1013904223u;
    return (float)(s->rng >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §4  color — themeable age ramp + HUD pairs                            */
/* ===================================================================== */

/*
 * Six colour schemes, each five shades from newest growth to oldest. Even the oldest
 * shade stays fairly bright on purpose, so finished growth doesn't vanish into the
 * black background (a project-wide rule). Each row reads roughly:
 *   Plasma  white -> pink -> purple
 *   Fire    white -> yellow -> orange
 *   Ice     white -> pale -> light blue -> cyan -> blue
 *   Ghost   bright -> mid grays
 *   Neon    white -> cyan -> green -> azure -> blue
 *   Matrix  white head -> green falling-code trail
 */
static const short THEME_FG[N_THEMES][N_AGE_LEVELS] = {
    { 231, 213, 177, 135,  99 },  /* Plasma */
    { 231, 226, 214, 208, 166 },  /* Fire   */
    { 231, 159, 117,  45,  39 },  /* Ice    */
    { 255, 252, 249, 246, 244 },  /* Ghost  */
    { 231,  51,  46,  45,  39 },  /* Neon   */
    { 231,  47,  46,  40,  34 },  /* Matrix */
};

static const char *THEME_NAME[N_THEMES] = {
    "Plasma", "Fire", "Ice", "Ghost", "Neon", "Matrix"
};

/* Fallback shades for old terminals that only have 8 colours (same for every theme). */
static const short THEME_FG8[N_AGE_LEVELS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE
};

/* Set up the colour slots: the fixed HUD colours plus the five age shades for the
 * chosen theme. Falls back to the 8-colour shades when the terminal is limited. */
static void color_apply_theme(int theme)
{
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(CP_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    for (int i = 0; i < N_AGE_LEVELS; i++)
        init_pair((short)(CP_A0 + i),
                  (COLORS >= 256) ? THEME_FG[theme][i] : THEME_FG8[i],
                  COLOR_BLACK);
}

/* age_pair / age_char — pure maps from how-old a cell is to its colour and glyph. */
static int age_pair(int age_delta)
{
    for (int i = 0; i < N_AGE_LEVELS; i++)
        if (age_delta <= AGE_THRESH[i])
            return CP_A0 + i;
    return CP_A0 + N_AGE_LEVELS - 1;
}

static chtype age_char(int age_delta)
{
    if (age_delta <= AGE_THRESH[0]) return (chtype)'@';
    if (age_delta <= AGE_THRESH[1]) return (chtype)'#';
    if (age_delta <= AGE_THRESH[2]) return (chtype)'+';
    if (age_delta <= AGE_THRESH[3]) return (chtype)':';
    return (chtype)'.';
}

/* ===================================================================== */
/* §5  field — the Laplace potential φ  (CORE physics)                    */
/* ===================================================================== */

/* ── boundary-condition primitives (each names one kind of edge constraint) ── */

/* bc_row — pin an entire row to potential v (a Dirichlet source or ground edge). */
static void bc_row(Field *f, int row, float v, int cols)
{
    for (int c = 0; c < cols; c++) f->phi[row][c] = v;
}

/* bc_col — pin an entire column to potential v. */
static void bc_col(Field *f, int col, float v, int rows)
{
    for (int r = 0; r < rows; r++) f->phi[r][col] = v;
}

/* bc_neumann_lr — free (zero-flux) left & right edges: copy the inward neighbour. */
static void bc_neumann_lr(Field *f, int rows, int cols)
{
    for (int r = 1; r < rows - 1; r++) {
        f->phi[r][0]        = f->phi[r][1];
        f->phi[r][cols - 1] = f->phi[r][cols - 2];
    }
}

/* bc_neumann_tb — free (zero-flux) top & bottom edges: copy the inward neighbour. */
static void bc_neumann_tb(Field *f, int rows, int cols)
{
    for (int c = 1; c < cols - 1; c++) {
        f->phi[0][c]        = f->phi[1][c];
        f->phi[rows - 1][c] = f->phi[rows - 2][c];
    }
}

/* bc_ground_aggregate — the conductor is grounded: force φ=0 on every occupied cell. */
static void bc_ground_aggregate(Scene *s)
{
    for (int r = 0; r < s->rows; r++)
        for (int c = 0; c < s->cols; c++)
            if (s->aggregate.occupied[r][c])
                s->field.phi[r][c] = 0.0f;
}

/*
 * field_set_bc — re-impose the preset's boundary conditions, then ground the
 * aggregate.  Now reads as the physics: which edges are the φ=1 SOURCE, which are
 * GROUND (φ=0), and which are FREE (Neumann).  Re-run after every relaxation sweep.
 */
static void field_set_bc(Scene *s)
{
    Field *f = &s->field;
    int R = s->rows, C = s->cols;

    switch (s->preset) {
    case PRESET_TREE:
    case PRESET_FROST:
        bc_row(f, 0,     1.0f, C);   /* source: top    */
        bc_row(f, R - 1, 0.0f, C);   /* ground: bottom */
        bc_neumann_lr(f, R, C);
        break;
    case PRESET_LIGHTNING:
        bc_row(f, 0,     0.0f, C);   /* ground: top    */
        bc_row(f, R - 1, 1.0f, C);   /* source: bottom */
        bc_neumann_lr(f, R, C);
        break;
    case PRESET_CORAL:
        bc_row(f, 0,     1.0f, C);   /* source: all 4 edges */
        bc_row(f, R - 1, 1.0f, C);
        bc_col(f, 0,     1.0f, R);
        bc_col(f, C - 1, 1.0f, R);
        break;
    case PRESET_DISCHARGE:
        bc_col(f, 0,     1.0f, R);   /* source: left & right */
        bc_col(f, C - 1, 1.0f, R);
        bc_neumann_tb(f, R, C);
        break;
    default: break;
    }

    bc_ground_aggregate(s);
}

/* ── initial-field fills (a cheap φ guess so Gauss-Seidel converges fast) ── */

/* field_fill_vramp — vertical linear ramp between the source and ground rows.
 * top_is_source: φ goes 1 (top) → 0 (bottom); otherwise 0 (top) → 1 (bottom). */
static void field_fill_vramp(Scene *s, bool top_is_source)
{
    for (int r = 0; r < s->rows; r++) {
        float depth = (s->rows > 1) ? (float)r / (float)(s->rows - 1) : 0.0f;  /* 0 top → 1 bottom */
        float v = top_is_source ? (1.0f - depth) : depth;
        for (int c = 0; c < s->cols; c++)
            s->field.phi[r][c] = v;
    }
}

/* field_fill_radial — φ ≈ normalised distance from the centre (0 at the centre seed,
 * → 1 toward the edge sources): the Coral guess. */
static void field_fill_radial(Scene *s)
{
    float cx = (float)(s->cols - 1) * 0.5f;
    float cy = (float)(s->rows - 1) * 0.5f;
    float max_d = (cx > cy) ? cx : cy;
    for (int r = 0; r < s->rows; r++)
        for (int c = 0; c < s->cols; c++) {
            float dx = (float)c - cx, dy = (float)r - cy;
            float d  = sqrtf(dx*dx + dy*dy);
            float v  = (max_d > 0.0f) ? d / max_d : 0.0f;
            s->field.phi[r][c] = (v > 1.0f) ? 1.0f : v;
        }
}

/* field_fill_hramp_center — horizontal ramp: 0 at the centre column → 1 at the left
 * and right edge sources: the Discharge guess. */
static void field_fill_hramp_center(Scene *s)
{
    float cx = (float)(s->cols - 1) * 0.5f;
    for (int r = 0; r < s->rows; r++)
        for (int c = 0; c < s->cols; c++)
            s->field.phi[r][c] = (cx > 0.0f) ? fabsf((float)c - cx) / cx : 0.0f;
}

/* field_seed_guess — pick the initial-φ fill that matches the preset's sources. */
static void field_seed_guess(Scene *s)
{
    switch (s->preset) {
    case PRESET_TREE:
    case PRESET_FROST:     field_fill_vramp(s, true);   break;   /* source at top    */
    case PRESET_LIGHTNING: field_fill_vramp(s, false);  break;   /* source at bottom */
    case PRESET_CORAL:     field_fill_radial(s);        break;
    case PRESET_DISCHARGE: field_fill_hramp_center(s);  break;
    default: break;
    }
}

/* laplace_avg — the 5-point Laplacian update: a free cell relaxes to the average of
 * its 4 orthogonal neighbours, i.e. the discrete solution of ∇²φ = 0 at that cell. */
static float laplace_avg(const Field *f, int r, int c)
{
    return STENCIL_AVG_W * (f->phi[r-1][c] + f->phi[r+1][c]
                          + f->phi[r][c-1] + f->phi[r][c+1]);
}

/* field_apply_free_edges — re-impose ONLY the preset's free (Neumann) edges, which
 * track the moving interior and so must be refreshed every sweep.  The Dirichlet
 * source/ground edges and the grounded aggregate are constant, so they are NOT
 * touched here (see field_relax for why that is correct and fast). */
static void field_apply_free_edges(Scene *s)
{
    Field *f = &s->field;
    switch (s->preset) {
    case PRESET_TREE:
    case PRESET_FROST:
    case PRESET_LIGHTNING: bc_neumann_lr(f, s->rows, s->cols); break;
    case PRESET_DISCHARGE: bc_neumann_tb(f, s->rows, s->cols); break;
    case PRESET_CORAL:
    default:               break;   /* all-Dirichlet: nothing to refresh */
    }
}

/*
 * field_relax — N_RELAX Gauss-Seidel sweeps toward ∇²φ = 0: relax every FREE cell to
 * laplace_avg, then refresh the moving Neumann edges.  This is the heavier half of
 * the physics step (see scene_tick).
 *
 * PERFORMANCE: it does NO full-grid boundary work per tick.  That is correct by an
 * invariant the rest of the code maintains: occupied cells are grounded once at join
 * (aggregate_add) and the sweep skips them, so they stay 0; the Dirichlet edges are
 * pinned once at reset (field_set_bc) and the interior sweep never touches an edge.
 * Only the Neumann edges move, and only those are refreshed — turning what used to be
 * 8 whole-grid grounding scans per tick into a handful of edge updates.
 */
static void field_relax(Scene *s)
{
    for (int pass = 0; pass < N_RELAX; pass++) {
        for (int r = 1; r < s->rows - 1; r++)
            for (int c = 1; c < s->cols - 1; c++)
                if (!s->aggregate.occupied[r][c])
                    s->field.phi[r][c] = laplace_avg(&s->field, r, c);
        field_apply_free_edges(s);
    }
}

/* ===================================================================== */
/* §6  aggregate — the growing conductor  (CORE algorithm)               */
/* ===================================================================== */

static const int DR4[4] = { -1, 1,  0, 0 };
static const int DC4[4] = {  0, 0, -1, 1 };

/* aggregate_clear — empty the cluster back to "no cells", step counter at 1. */
static void aggregate_clear(Aggregate *a)
{
    memset(a->occupied, 0, sizeof a->occupied);
    memset(a->age,      0, sizeof a->age);
    a->step = 1;
    a->size = 0;
}

/* ── Frontier maintenance — O(1) add/remove for the growth candidate set ── */

/* frontier_clear — empty the candidate set (every slot marked "absent" = −1). */
static void frontier_clear(Frontier *f)
{
    f->count = 0;
    memset(f->slot, 0xFF, sizeof f->slot);   /* fills each int slot with −1 */
}

/* frontier_add — insert cell (r,c) if it is not already a candidate. */
static void frontier_add(Frontier *f, int r, int c)
{
    if (f->slot[r][c] >= 0) return;
    int i = f->count++;
    f->cr[i] = (uint16_t)r;
    f->cc[i] = (uint16_t)c;
    f->slot[r][c] = i;
}

/* frontier_remove — drop cell (r,c) by swapping the last entry into its slot. */
static void frontier_remove(Frontier *f, int r, int c)
{
    int i = f->slot[r][c];
    if (i < 0) return;
    int last = --f->count;
    int lr = f->cr[last], lc = f->cc[last];
    f->cr[i] = (uint16_t)lr;
    f->cc[i] = (uint16_t)lc;
    f->slot[lr][lc] = i;
    f->slot[r][c] = -1;
}

/* aggregate_add — join cell (r,c): mark it occupied, stamp its age, ground its
 * potential, count it, and keep the frontier current (this cell leaves the
 * frontier; each empty 4-neighbour enters it). */
static void aggregate_add(Scene *s, int r, int c)
{
    s->aggregate.occupied[r][c] = 1;
    s->aggregate.age     [r][c] = (uint16_t)s->aggregate.step;
    s->field.phi         [r][c] = 0.0f;
    s->aggregate.size++;

    frontier_remove(&s->frontier, r, c);
    for (int d = 0; d < 4; d++) {
        int nr = r + DR4[d], nc = c + DC4[d];
        if (nr < 0 || nr >= s->rows || nc < 0 || nc >= s->cols) continue;
        if (!s->aggregate.occupied[nr][nc])
            frontier_add(&s->frontier, nr, nc);
    }
}

/* aggregate_seed — plant the initial cell(s) for the preset.  Frost is the one
 * multi-seed case: a line along the bottom whose ferns compete as they climb. */
static void aggregate_seed(Scene *s)
{
    if (s->preset == PRESET_FROST) {
        for (int c = 0; c < s->cols; c += FROST_SPACING)
            aggregate_add(s, s->rows - 1, c);
        return;
    }

    int sr, sc;
    switch (s->preset) {
    case PRESET_LIGHTNING:                 sr = 0;            sc = s->cols / 2; break;
    case PRESET_CORAL:
    case PRESET_DISCHARGE:                 sr = s->rows / 2;  sc = s->cols / 2; break;
    case PRESET_TREE:
    default:                               sr = s->rows - 1;  sc = s->cols / 2; break;
    }
    aggregate_add(s, sr, sc);
}

/* cell_weight — a frontier cell's growth weight, φ^η (the DBM rule). */
static double cell_weight(const Scene *s, int r, int c)
{
    float p = s->field.phi[r][c];
    if (p < 0.0f) p = 0.0f;
    return (double)powf(p, s->eta);
}

/* frontier_total_weight — Σ φ^η over the candidate list: the roulette wheel's total
 * circumference, and the normaliser for weighted selection.  O(frontier), not O(grid). */
static double frontier_total_weight(const Scene *s)
{
    const Frontier *f = &s->frontier;
    double total = 0.0;
    for (int i = 0; i < f->count; i++)
        total += cell_weight(s, f->cr[i], f->cc[i]);
    return total;
}

/* frontier_pick — walk the candidate list accumulating φ^η until the running sum
 * passes `target`; that cell is the roulette-wheel winner.  False only on a rounding
 * miss.  O(frontier), not O(grid). */
static bool frontier_pick(const Scene *s, double target, int *pr, int *pc)
{
    const Frontier *f = &s->frontier;
    double cum = 0.0;
    for (int i = 0; i < f->count; i++) {
        cum += cell_weight(s, f->cr[i], f->cc[i]);
        if (cum >= target) { *pr = f->cr[i]; *pc = f->cc[i]; return true; }
    }
    return false;
}

/*
 * aggregate_grow — add N_GROW cells by roulette-wheel selection ∝ φ^η: total the
 * frontier weight, spin the wheel, pick the cell the spin lands on, and join it.
 */
static void aggregate_grow(Scene *s)
{
    for (int g = 0; g < N_GROW; g++) {
        double total = frontier_total_weight(s);
        if (total <= 0.0) return;                       /* nothing can grow */

        double target = (double)lcg_f(s) * total;       /* spin the wheel */
        int r, c;
        if (!frontier_pick(s, target, &r, &c)) return;  /* rounding edge case */

        aggregate_add(s, r, c);
        s->aggregate.step++;
    }
}

/* ===================================================================== */
/* §7  scene — state + reset; scene_tick is THE simulation step           */
/* ===================================================================== */

static void scene_fit(Scene *s, int cols, int rows)
{
    s->cols = cols < 1 ? 1 : (cols > COLS_MAX ? COLS_MAX : cols);
    s->rows = rows < 1 ? 1 : (rows > ROWS_MAX ? ROWS_MAX : rows);
}

/* scene_reset — clear the aggregate & frontier, seed it, and lay down the field. */
static void scene_reset(Scene *s)
{
    aggregate_clear(&s->aggregate);   /* §6 empty the cluster                 */
    frontier_clear(&s->frontier);     /* §6 empty the candidate set           */
    aggregate_seed(s);                /* §6 plant seed(s) → repopulates frontier */
    field_seed_guess(s);              /* §5 initial φ                         */
    field_set_bc(s);                  /* §5 pin BC + ground the seed          */
}

/* scene_set_preset — pick a preset, its default η, and start fresh. */
static void scene_set_preset(Scene *s, Preset p)
{
    s->preset = p;
    switch (p) {
    case PRESET_TREE:      s->eta = ETA_DEFAULT_TREE;      break;
    case PRESET_LIGHTNING: s->eta = ETA_DEFAULT_LIGHTNING; break;
    case PRESET_FROST:     s->eta = ETA_DEFAULT_FROST;     break;
    case PRESET_DISCHARGE: s->eta = ETA_DEFAULT_DISCHARGE; break;
    case PRESET_CORAL:
    default:               s->eta = ETA_DEFAULT_CORAL;     break;
    }
    scene_reset(s);
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->rng    = (uint32_t)(clock_ns() ^ (clock_ns() << 13)) | 1u;
    s->theme  = 0;
    s->paused = false;
    scene_fit(s, cols, rows);
    scene_set_preset(s, PRESET_TREE);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    scene_fit(s, cols, rows);
    scene_reset(s);
}

static void scene_set_eta(Scene *s, float eta)
{
    if (eta < ETA_MIN) eta = ETA_MIN;
    if (eta > ETA_MAX) eta = ETA_MAX;
    s->eta = eta;
}

/*
 * scene_tick — THE simulation step, and the ONLY place the world changes: relax the
 * field, then grow the aggregate.  `paused` gates it; nothing else (no renderer,
 * no input handler) mutates the physics.
 */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    field_relax(s);          /* §5 — push φ toward ∇²φ = 0 */
    aggregate_grow(s);       /* §6 — add cells ∝ φ^η        */
}

/* ===================================================================== */
/* §8  render — aggregate → glyphs + HUD (READ-ONLY)                      */
/* ===================================================================== */

/*
 * Screen — the live terminal size, refreshed from getmaxyx() at start-up and on
 * SIGWINCH.  Kept apart from Scene on purpose: it is a property of the display, not
 * of the simulation.  The HUD clamps its text to these bounds, and the grid carves
 * its area out of them (rows minus the reserved HUD rows).
 */
typedef struct {
    int rows;   /* terminal height, in character cells */
    int cols;   /* terminal width,  in character cells */
} Screen;

/*
 * render_aggregate — paint the aggregate below the top HUD rows.  Strictly
 * read-only (const Scene *): a cell's colour/glyph come from its age relative to
 * the current step, computed on the fly — nothing is written back.
 */
static void render_aggregate(const Scene *s)
{
    for (int r = 0; r < s->rows; r++) {
        for (int c = 0; c < s->cols; c++) {
            if (!s->aggregate.occupied[r][c]) continue;

            int age_delta = s->aggregate.step - (int)s->aggregate.age[r][c];
            if (age_delta < 0) age_delta = 0;

            attr_t attr = (attr_t)COLOR_PAIR(age_pair(age_delta));
            if (age_delta <= AGE_THRESH[0]) attr |= A_BOLD;   /* newest growth pops */

            attron(attr);
            mvaddch(r + HUD_TOP_ROWS, c, age_char(age_delta));
            attroff(attr);
        }
    }
}

/* hud_line — one HUD line at (row, x), truncated to the screen width so it can
 * never wrap or overrun, however long the string or narrow the terminal. */
static void hud_line(const Screen *scr, int row, int x, int pair, attr_t attr, const char *str)
{
    if (x < 0) x = 0;
    if (x >= scr->cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", scr->cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * render_hud — data on top, actions on the bottom:
 *   row 0      (yellow bold,  right)  live status: aggregate size, run state, fps
 *   row 1      (yellow plain, left)   parameters: preset, eta, theme
 *   row rows-1 (cyan bold,    left)   every interactive key
 */
static void render_hud(const Screen *scr, const Scene *s, double fps)
{
    char buf[160];

    snprintf(buf, sizeof buf, " size:%-5d  %s%4.1f fps ",
             s->aggregate.size, s->paused ? "PAUSED  " : "", fps);
    hud_line(scr, 0, scr->cols - (int)strlen(buf), CP_HUD, A_BOLD, buf);

    snprintf(buf, sizeof buf, " DBM Laplace  preset:%s  eta:%.2f  theme:%s ",
             PRESET_NAME[s->preset], (double)s->eta, THEME_NAME[s->theme]);
    hud_line(scr, 1, 0, CP_HUD, A_NORMAL, buf);

    hud_line(scr, scr->rows - 1, 0, CP_HINT, A_BOLD,
             " q:quit  spc:pause  r:reset  1-5:preset  t:theme  e/E:eta ");
}

/* ===================================================================== */
/* §9  app — main loop                                                    */
/* ===================================================================== */

/*
 * FpsMeter — a smoothed frames-per-second readout for the HUD.  Sums real frame Δt
 * and recomputes the average once per FPS_UPDATE_MS window, so the number is steady
 * rather than jittering every frame.
 */
typedef struct {
    int64_t accum_ns;   /* time summed in the current window     */
    int     frames;     /* frames counted in the current window  */
    double  value;      /* last computed fps — what the HUD shows */
} FpsMeter;

static void fps_meter_reset(FpsMeter *m) { m->accum_ns = 0; m->frames = 0; }

/* fps_meter_add — fold one frame's Δt in; refresh `value` at each window boundary.
 * A huge Δt (a pause or resize stall) is clamped so it can't skew the average. */
static void fps_meter_add(FpsMeter *m, int64_t dt_ns)
{
    if (dt_ns > FRAME_DT_CAP_MS * NS_PER_MS) dt_ns = FRAME_DT_CAP_MS * NS_PER_MS;
    m->accum_ns += dt_ns;
    m->frames++;
    if (m->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        m->value = (double)m->frames / ((double)m->accum_ns / (double)NS_PER_SEC);
        fps_meter_reset(m);
    }
}

/*
 * App — the whole running program in one object: the Scene being simulated, the
 * Screen it is drawn on, the fps meter, and the two flags the OS pokes
 * asynchronously.  Bundling them lets a single file-scope instance be reached from
 * the signal handler while everything else is passed by pointer.
 */
typedef struct {
    Scene                 scene;         /* §7 the simulation                 */
    Screen                screen;        /* §8 terminal size                  */
    FpsMeter              fps;           /* smoothed frame rate, for the HUD  */
    volatile sig_atomic_t running;       /* cleared by SIGINT/SIGTERM or 'q'  */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH                   */
} App;

/* The one global: signal handlers receive only an int, so the flags they set must
 * be reachable without a parameter.  Everything else is reached through this. */
static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static void terminal_init(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    start_color();
}

/* grid_rows / grid_cols — the simulation size carved out of the terminal (the HUD
 * rows are reserved at top and bottom). */
static int grid_rows(const Screen *scr) { return scr->rows - HUD_TOP_ROWS - HUD_BOT_ROWS; }
static int grid_cols(const Screen *scr) { return scr->cols; }

static void app_apply_resize(App *app)
{
    endwin();
    refresh();
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_resize(&app->scene, grid_cols(&app->screen), grid_rows(&app->screen));
    app->need_resize = 0;
}

/* app_handle_key — translate one keypress into a scene action; false = quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'p': case 'P': case ' ': s->paused = !s->paused; break;
    case 'r': case 'R':           scene_reset(s);         break;

    case '1': scene_set_preset(s, PRESET_TREE);      break;
    case '2': scene_set_preset(s, PRESET_LIGHTNING); break;
    case '3': scene_set_preset(s, PRESET_CORAL);     break;
    case '4': scene_set_preset(s, PRESET_FROST);     break;
    case '5': scene_set_preset(s, PRESET_DISCHARGE); break;

    case 't': s->theme = (s->theme + 1) % N_THEMES;            color_apply_theme(s->theme); break;
    case 'T': s->theme = (s->theme + N_THEMES - 1) % N_THEMES; color_apply_theme(s->theme); break;

    case 'e': scene_set_eta(s, s->eta + ETA_STEP); break;
    case 'E': scene_set_eta(s, s->eta - ETA_STEP); break;

    default: break;
    }
    return true;
}

static void app_drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR)
        if (!app_handle_key(app, ch)) { app->running = 0; break; }
}

/* app_present — READ-ONLY frame: clear, draw the aggregate, overlay the HUD, flush. */
static void app_present(App *app)
{
    erase();
    render_aggregate(&app->scene);
    render_hud(&app->screen, &app->scene, app->fps.value);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    terminal_init();

    App *app     = &g_app;
    app->running = 1;
    app->fps     = (FpsMeter){ 0 };
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_init(&app->scene, grid_cols(&app->screen), grid_rows(&app->screen));
    color_apply_theme(app->scene.theme);

    int64_t last_frame = clock_ns();

    while (app->running) {
        if (app->need_resize) {
            app_apply_resize(app);
            fps_meter_reset(&app->fps);
            last_frame = clock_ns();
        }

        int64_t frame_start = clock_ns();

        scene_tick(&app->scene);     /* 1. THE effect: relax field + grow aggregate */
        app_drain_input(app);        /* 2. input  → scene edits                     */
        app_present(app);            /* 3. render → read-only blit + flush          */

        fps_meter_add(&app->fps, frame_start - last_frame);   /* 4. measure cadence */
        last_frame = frame_start;

        clock_sleep_ns(RENDER_NS - (clock_ns() - frame_start));   /* 5. cap to RENDER_FPS */
    }
    return 0;
}
