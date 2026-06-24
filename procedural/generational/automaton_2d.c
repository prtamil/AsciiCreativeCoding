/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * automaton_2d.c — a cellular automaton like Conway's Game of Life, but
 * each cell looks at a bigger square of neighbours (radius 1 to 10) and can
 * have a fading "dying" trail. Big-radius rules grow crystals, spirals, and
 * drifting blobs you can't get with plain Life.
 *
 * The one trick worth knowing: counting alive cells in a big square per cell
 * would be slow, so we precompute a running-total table (a "summed-area
 * table") once per generation and read any square's count in four lookups.
 *
 * Rules and references: Larger-than-Life (Evans 2001); the multi-state
 * "Generations" trail rules (MCell, Wojtowicz 2001); the summed-area table
 * (Crow 1984, also called the "integral image", Viola & Jones 2001).
 * Sister file: life.c is the plain radius-1 Game of Life this generalises.
 *
 *   §1 config  §2 clock  §3 color  §4 model  §5 areatable
 *   §6 sim     §7 render  §8 app
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra automaton_2d.c -o automaton_2d -lncurses
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 20,    /* the world steps slower than it draws     */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,
    RENDER_FPS      = 60,    /* cap on how often we repaint the screen   */

    FPS_UPDATE_MS   = 500,
    DT_CAP_MS       = 100,   /* ignore the extra if one frame stalls     */

    HUD_TOP_ROWS    =  1,    /* top row is the live data bar             */
    HUD_BOT_ROWS    =  1,    /* bottom row is the key-hints bar          */
    HUD_ROWS        = HUD_TOP_ROWS + HUD_BOT_ROWS,

    PAIR_HUD        =  8,    /* colour of the top data bar               */
    PAIR_HINT       =  9,    /* colour of the bottom hints bar           */

    N_PRESETS       = 15,
    N_THEMES        =  6,
    R_MAX           = 10,    /* biggest radius — sets how much we allocate */
    N_MAX           =  8,    /* most states a rule can have              */

    GENS_DEF        =  1,    /* how many generations one tick runs       */
    GENS_MAX        = 16,

    SEED_BLOCK_NUM  =  3,    /* the central seed block fills 3/5 of each axis */
    SEED_BLOCK_DEN  =  5,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CELL_GLYPH  '#'

/*
 * Interval — an inclusive range of counts, [lo, hi]. The whole rule comes
 * down to two yes/no questions: is the alive-neighbour count inside the
 * "birth" range, and inside the "survive" range? A backwards range
 * (lo > hi) is empty — it means "never", which is how BRAIN makes every
 * cell die after one step.
 */
typedef struct {
    int lo;   /* lowest count that counts as inside (inclusive)        */
    int hi;   /* highest count that's inside; hi < lo means "empty"    */
} Interval;
static bool interval_has(Interval iv, int n) { return n >= iv.lo && n <= iv.hi; }

/*
 * Rule — one complete rule: how far each cell looks, how many states it has,
 * and the two count-ranges that decide births and survivals. It's just data;
 * the actual behaviour lives in cell_next (§6). Keeping it as plain data lets
 * the whole preset list be a static table, so switching rules is one
 * assignment with no memory allocation. The numbers match Evans's
 * Larger-than-Life notation (Evans 2001); rules with more than 2 states add
 * a fading trail in the style of the "Generations" family (MCell).
 */
typedef struct {
    int         radius;     /* R: how far a cell looks. The neighbourhood is
                             * the (2R+1)-square around it, so the most alive
                             * neighbours it can see is (2R+1)²−1.           */
    int         states;     /* N: 2 means plain alive/dead; more than 2 adds
                             * a fading trail (the comet-tail look).         */
    Interval    birth;      /* a DEAD cell turns on when its alive-neighbour
                             * count lands in this range.                    */
    Interval    survive;    /* an ALIVE cell stays alive when its count lands
                             * in this range; empty range = never survives.  */
    int         seed_pct;   /* 0..100 chance a cell starts alive, picked per
                             * rule so it settles instead of dying out.      */
    bool        seed_block; /* true = seed a dense central block; false = fill
                             * the whole grid evenly. See grid_seed_block.    */
    const char *name;       /* fixed-width label for the HUD.                */
} Rule;

/*
 * The 15 built-in rules. Each carries its own seeding recipe on purpose:
 * the picky narrow-range rules die instantly from an even random fill, so
 * they get a dense central block (locally crowded, globally sparse — its
 * edge sits right where these rules like it). The chaotic ones do better
 * with an even fill. The big-radius thresholds were hand-tuned so the
 * patterns keep moving instead of freezing or fizzling out.
 */
static const Rule PRESETS[N_PRESETS] = {
    /*   R   N    birth       survive    seed% block  name        */
    {    5,  2, { 34,  58}, { 34,  58},   30, false, "BOSCO    " }, /* classic drifting blobs   */
    {    5,  6, { 34,  45}, { 34,  58},   40, true,  "BUGS     " }, /* colourful chasing bugs   */
    {    1,  4, {  1,   1}, {  1,   1},    5, false, "GNARL    " }, /* fractal growth, 3-step fade */
    {    2,  5, {  9,  16}, {  5,  14},   25, false, "AMOEBA   " }, /* soft organic teardrops   */
    {    7,  2, { 75, 170}, {100, 200},   45, true,  "WAFFLE   " }, /* rigid waffle lattice     */
    {    8,  4, { 68, 116}, { 68, 139},   40, true,  "GLOBE    " }, /* huge smooth drifting orbs */
    {   10,  2, {104, 177}, {104, 213},   40, true,  "BUGSMOVIE" }, /* giant drifting organism  */
    {    1,  3, {  2,   2}, {  1,   0},   25, false, "BRAIN    " }, /* survive {1,0}=never → Brian's Brain */
    {    1,  4, {  2,   2}, {  3,   5},   28, false, "STARWARS " }, /* spaceships & oscillators */
    {    1,  3, {  3,   4}, {  1,   2},   30, false, "FROGS    " }, /* hopping speckle texture  */
    {    1,  6, {  2,   2}, {  3,   6},   22, false, "STICKS   " }, /* growing trailing sticks  */
    {    3,  2, { 13,  23}, { 13,  23},   28, false, "CORAL    " }, /* static coral / reef      */
    {    4,  4, { 22,  38}, { 22,  40},   30, true,  "CRYSTAL  " }, /* faceted crystal growth   */
    {    4,  2, { 41,  81}, { 40,  80},   50, false, "MAJORITY " }, /* coarsening vote domains  */
    {    6,  7, { 47,  90}, { 47, 110},   35, true,  "NEBULA   " }, /* huge multi-hue nebula    */
};

/*
 * Theme — just the colours. A theme and a rule are separate things you cycle
 * on their own: n/p change the rule, t/T recolour whatever's running. The
 * colours run bright-to-dim so a fully-alive cell pops and the fading trail
 * dims out behind it. Every colour is kept in the bright half of the palette
 * on purpose, so even the dimmed trail stays visible. Old 8-colour terminals
 * just use one flat colour.
 */
typedef struct {
    const char *name;        /* label shown in the HUD                     */
    short       grad[N_MAX]; /* bright-to-dim colours for the live + dying
                              * states; the last entry tints the HUD.      */
    short       fb8;         /* the one colour to use on 8-colour terminals */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "OCEAN  ", {  51,  45,  39,  44,  74, 117, 159, 226 }, COLOR_CYAN    },
    { "FIRE   ", { 226, 220, 208, 202, 196, 166, 130, 226 }, COLOR_YELLOW  },
    { "MATRIX ", {  46,  47,  41,  40,  35,  34,  71, 226 }, COLOR_GREEN   },
    { "VIOLET ", { 207, 201, 171, 141, 135,  99, 105, 226 }, COLOR_MAGENTA },
    { "ICE    ", { 231, 195, 159, 123,  87,  81,  75,  51 }, COLOR_WHITE   },
    { "AMBER  ", { 229, 223, 221, 215, 214, 208, 202, 226 }, COLOR_YELLOW  },
};

/* wrap a list index back into 0..n-1, working for negative steps too. */
static int wrap_idx(int i, int n) { i %= n; if (i < 0) i += n; return i; }

/* wrap a coordinate around the grid edges — this is what makes the world
 * loop around like a doughnut, so the top edge meets the bottom. */
static int wrap_torus(int i, int n) { return ((i % n) + n) % n; }

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int imin(int a, int b) { return a < b ? a : b; }

/* ── §2 clock ── */

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

/* ── §3 color ── */

static int g_n256 = 0;   /* true if the terminal has 256 colours */

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_n256 = (COLORS >= 256);
    /* the real state/HUD colours come later from palette_apply; start white */
    for (int i = 1; i <= PAIR_HINT; i++)
        init_pair((short)i, COLOR_WHITE, COLOR_BLACK);
    /* the hint bar never changes with the theme — always bright cyan */
    init_pair(PAIR_HINT, g_n256 ? 51 : COLOR_CYAN, -1);
}

/* point the colour pairs at the current theme. Called on every rule or theme
 * change; redoing colours mid-run doesn't flicker on modern terminals. */
static void palette_bind_256(const Theme *t, int N)
{
    for (int i = 0; i < N - 1 && i < N_MAX - 1; i++)
        init_pair((short)(i + 1), t->grad[i], COLOR_BLACK);
    init_pair(PAIR_HUD, t->grad[N_MAX - 1], -1);
}

/* on an 8-colour terminal, paint every state the theme's one flat colour. */
static void palette_bind_8(const Theme *t, int N)
{
    for (int i = 1; i <= N - 1; i++)
        init_pair((short)i, t->fb8, COLOR_BLACK);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
}

static void palette_apply(int theme, int N)
{
    const Theme *t = &THEMES[theme];
    if (g_n256) palette_bind_256(t, N);
    else        palette_bind_8(t, N);
}

/* ── §4 model ── */

/*
 * What the numbers in a cell mean. Each cell is one byte:
 *   CELL_DEAD (0)        empty
 *   cell_alive(N) = N-1  fully alive — the only value that counts as a neighbour
 *   1 .. N-2             the fading trail: ghosts that step down one level each
 *                        generation and don't count as neighbours
 * Giving these names keeps the rule (§6) and the drawing (§7) readable
 * instead of bare "N-1" arithmetic.
 */
enum { CELL_DEAD = 0 };
static uint8_t cell_alive(int states) { return (uint8_t)(states - 1); }

/*
 * Grid — the actual cells, kept as two copies. A cellular automaton updates
 * every cell at the same instant: we read the current copy (cur), compute the
 * next values into the spare copy (nxt), then swap them. With only one copy,
 * a cell we just changed would wrongly affect its neighbours' counts — the
 * classic mistake (Toffoli & Margolus, 1987).
 *
 * These live on the heap (not as fixed globals) because the grid follows the
 * terminal size — a noted exception to the no-malloc rule. It only allocates
 * at startup and on resize, never inside a frame.
 */
typedef struct {
    uint8_t *cur;        /* the current cells, one byte each, stored row by row */
    uint8_t *nxt;        /* spare buffer we write the next generation into;
                          * its contents between steps don't matter.       */
    int      w, h;       /* size in cells = screen cols × (rows minus the HUD) */
    long     generation; /* generations since the last seed, shown as "gen:".
                          * long so it doesn't overflow after a long run.  */
} Grid;

/* swap the two buffers so the freshly-written one becomes current — this is
 * what makes every cell update at the same moment. */
static void grid_swap(Grid *g) { uint8_t *t = g->cur; g->cur = g->nxt; g->nxt = t; }

/*
 * Settings — the knobs the user controls, kept separate from the Grid so
 * keypresses change these while the simulation changes the cells. The current
 * rule and theme are stored as positions in their lists (not the values
 * themselves) so n/p and t/T can wrap around and the HUD can show "3 of 15".
 * The chosen Rule itself gets copied into Scene.rule when the preset changes.
 */
typedef struct {
    int  preset;     /* which rule, 0..N_PRESETS-1                          */
    int  theme;      /* which colour theme, 0..N_THEMES-1                   */
    int  gens;       /* generations run per tick, 1..GENS_MAX — a fast-forward
                      * dial that's separate from the tick rate.           */
    bool paused;     /* freeze the simulation; drawing and input go on.    */
} Settings;

/* ── §5 areatable ── */

/*
 * This whole section exists for one reason: to count the alive cells inside a
 * big square fast. Counting them one by one would be slow for large radii. So
 * once per generation we build a running-total table; after that, counting any
 * square is just four lookups. It holds no game state — it's scratch rebuilt
 * from the Grid every generation. (The technique is the summed-area table, also
 * called the integral image — Crow 1984.)
 */

/*
 * AreaTable — the scratch used for fast square-counts.
 *
 * `pad` is a 0/1 copy of the grid (1 = fully alive) but with extra borders.
 * We copy the opposite edges into the borders so the world wraps like a
 * doughnut; that way every square we'll ever ask about fits inside `pad`
 * without running off an edge.
 *
 * `sat` is the running-total table: each entry holds the sum of all `pad`
 * values above-and-to-the-left of it. It has an all-zero first row and column
 * so the four-corner lookup never has to special-case the edge.
 *
 * Both are sized for the largest radius so changing radius never reallocates;
 * `pw`/`psw` are the row widths for the radius actually in use.
 */
typedef struct {
    int32_t *pad;   /* 0/1 alive map with wrap-around borders, so a square
                     * count never runs off an edge.                        */
    int32_t *sat;   /* running totals of pad; entry [i][j] = sum of every
                     * pad cell above and left of it. Zero first row/col.    */
    int      pw;    /* width of one pad row for the current radius           */
    int      psw;   /* width of one sat row (one wider than pad)             */
} AreaTable;

static void area_alloc(AreaTable *a, int w, int h)
{
    free(a->pad);  free(a->sat);
    int pw = w + 2 * R_MAX, ph = h + 2 * R_MAX;     /* room for the biggest radius */
    a->pad = calloc((size_t)pw * ph, sizeof(int32_t));
    a->sat = calloc((size_t)(pw + 1) * (ph + 1), sizeof(int32_t));
}

static void area_free(AreaTable *a)
{
    free(a->pad);  free(a->sat);
    a->pad = NULL; a->sat = NULL;
}

/* step 1: fill the bordered map — 1 where a cell is fully alive, 0 otherwise.
 * The borders are wrap-around copies of the far edges, which is what lets the
 * later counting skip all the edge-checking. */
static void area_stamp_indicator(AreaTable *a, const Grid *g, int R,
                                 uint8_t alive, int pw, int ph)
{
    int w = g->w, h = g->h;
    for (int pr = 0; pr < ph; pr++) {
        int sr = wrap_torus(pr - R, h);               /* which real row this border row copies */
        for (int pc = 0; pc < pw; pc++) {
            int sc = wrap_torus(pc - R, w);
            a->pad[pr * pw + pc] = (g->cur[sr * w + sc] == alive) ? 1 : 0;
        }
    }
}

/* step 2: fill the running-total table. Each entry is its own cell plus the
 * totals above it and to its left, minus the corner that those two share (so
 * it isn't counted twice). The zero first row and column let the counting
 * step below stay simple. */
static void area_prefix_sum(AreaTable *a, int pw, int ph)
{
    int psw = a->psw;
    for (int i = 0; i <= ph; i++) a->sat[i * psw] = 0;  /* zero the left column */
    for (int j = 0; j <= pw; j++) a->sat[j]       = 0;  /* zero the top row    */

    for (int i = 1; i <= ph; i++) {
        for (int j = 1; j <= pw; j++) {
            int here   = a->pad[(i - 1) * pw  + (j - 1)];
            int above  = a->sat[(i - 1) * psw +  j     ];
            int left   = a->sat[ i      * psw + (j - 1)];
            int corner = a->sat[(i - 1) * psw + (j - 1)];
            a->sat[i * psw + j] = here + above + left - corner;
        }
    }
}

/* rebuild the whole table for one generation: make the bordered 0/1 map, then
 * its running totals. Reads the grid, writes only the scratch. */
static void area_build(AreaTable *a, const Grid *g, int R, uint8_t alive)
{
    int pw = g->w + 2 * R, ph = g->h + 2 * R;
    a->pw = pw; a->psw = pw + 1;
    area_stamp_indicator(a, g, R, alive, pw, ph);
    area_prefix_sum(a, pw, ph);
}

/*
 * Count the alive cells in the square around grid cell (r,c) using four corner
 * lookups in the running-total table. The total inside a rectangle is the
 * bottom-right corner minus the strips above and to the left, plus the corner
 * those two strips overlap (added back once). The cell sits offset by R inside
 * the padded map, so its square runs from (r,c) to (r+2R,c+2R) there.
 */
static int area_count(const AreaTable *a, int r, int c, int R)
{
    const int32_t *P = a->sat;
    int psw = a->psw;
    int r1 = r + 2 * R, c1 = c + 2 * R;       /* the square's far corner */
    int br = P[(r1 + 1) * psw + (c1 + 1)];    /* bottom-right */
    int tr = P[ r       * psw + (c1 + 1)];    /* top-right    */
    int bl = P[(r1 + 1) * psw +  c      ];    /* bottom-left  */
    int tl = P[ r       * psw +  c      ];    /* top-left     */
    return br - tr - bl + tl;
}

/* ── §6 sim ── */

/*
 * Scene — the entire automaton in one bundle: the cells, the fast-count
 * scratch, the rule in play, and the user's knobs. Everything that changes the
 * world takes a Scene*; everything that only draws takes a const Scene*. The
 * fields are in tick order: read `grid` using `rule`, rebuild `area`, write the
 * next `grid`; `cfg` steers all of it.
 */
typedef struct {
    Grid      grid;     /* the cells (§4)                                 */
    AreaTable area;     /* fast-count scratch, rebuilt each generation (§5) */
    Rule      rule;     /* the rule currently running, copied from PRESETS */
    Settings  cfg;      /* the user knobs (§4)                            */
} Scene;

/*
 * The whole rule for one cell, given its current state and how many alive
 * neighbours it has:
 *   dead    → comes alive if the count is in the birth range, else stays dead
 *   alive   → stays alive if the count is in the survive range; otherwise it
 *             dies (2-state rules go straight to dead; multi-state rules step
 *             into the fading trail)
 *   fading  → always steps down one level toward dead
 * Fading cells aren't counted as neighbours, which is what makes the trail a
 * harmless ghost instead of feeding new births.
 */
static uint8_t cell_next(uint8_t state, int neighbours, const Rule *r)
{
    uint8_t alive = cell_alive(r->states);

    if (state == CELL_DEAD)
        return interval_has(r->birth, neighbours) ? alive : CELL_DEAD;

    if (state == alive)
        return interval_has(r->survive, neighbours) ? alive
             : (r->states > 2 ? (uint8_t)(alive - 1) : CELL_DEAD);

    return (uint8_t)(state - 1);
}

/*
 * How many fully-alive neighbours surround cell (r,c), not counting the cell
 * itself. This is the number the birth/survive ranges get tested against. We
 * grab the whole square's count in one fast lookup, then subtract the centre
 * if it's alive (fading-trail centres were never in the count anyway).
 */
static int live_neighbours(const Scene *s, int r, int c)
{
    int     R     = s->rule.radius;
    uint8_t alive = cell_alive(s->rule.states);
    int     count = area_count(&s->area, r, c, R);
    if (s->grid.cur[r * s->grid.w + c] == alive) count--;   /* don't count the cell itself */
    return count;
}

/* advance the whole grid by one generation. */
static void sim_step(Scene *s)
{
    Grid *g = &s->grid;

    /* rebuild the fast-count table so every count below is just four lookups */
    area_build(&s->area, g, s->rule.radius, cell_alive(s->rule.states));

    for (int r = 0; r < g->h; r++)
        for (int c = 0; c < g->w; c++)
            g->nxt[r * g->w + c] = cell_next(g->cur[r * g->w + c],
                                             live_neighbours(s, r, c), &s->rule);

    /* make the new generation the live one */
    grid_swap(g);
    g->generation++;
}

/* one tick runs `gens` generations; does nothing while paused. */
static void scene_tick(Scene *s)
{
    if (s->cfg.paused) return;
    for (int i = 0; i < s->cfg.gens; i++) sim_step(s);
}

/* true pct% of the time — the coin flip used when seeding. */
static bool chance_pct(int pct) { return rand() % 100 < pct; }

/* sprinkle random alive cells evenly across the whole grid. */
static void grid_seed_fill(Grid *g, uint8_t alive, int pct)
{
    int total = g->w * g->h;
    for (int i = 0; i < total; i++)
        g->cur[i] = chance_pct(pct) ? alive : CELL_DEAD;
    g->generation = 0;
}

/*
 * Seed a dense patch in the middle and leave the rest empty. The picky
 * narrow-range rules die out almost instantly from an even fill, but a central
 * block is crowded inside and empty outside, so its edge sits right where these
 * rules come alive — that's where the moving shapes form.
 */
static void grid_seed_block(Grid *g, uint8_t alive, int pct)
{
    int bw = g->w * SEED_BLOCK_NUM / SEED_BLOCK_DEN;   /* block size */
    int bh = g->h * SEED_BLOCK_NUM / SEED_BLOCK_DEN;
    int ox = (g->w - bw) / 2, oy = (g->h - bh) / 2;    /* centred */
    memset(g->cur, CELL_DEAD, (size_t)g->w * g->h);
    for (int r = oy; r < oy + bh; r++)
        for (int c = ox; c < ox + bw; c++)
            if (chance_pct(pct)) g->cur[r * g->w + c] = alive;
    g->generation = 0;
}

/* (re)seed the grid using whatever recipe the current rule prefers; also the
 * 'r' key. */
static void sim_seed(Scene *s)
{
    uint8_t alive = cell_alive(s->rule.states);
    if (s->rule.seed_block) grid_seed_block(&s->grid, alive, s->rule.seed_pct);
    else                    grid_seed_fill (&s->grid, alive, s->rule.seed_pct);
}

/* switch to a different rule: load it, recolour, and reseed the grid. */
static void sim_set_preset(Scene *s, int p)
{
    s->cfg.preset = wrap_idx(p, N_PRESETS);
    s->rule       = PRESETS[s->cfg.preset];
    palette_apply(s->cfg.theme, s->rule.states);
    sim_seed(s);
}

/* switch colour theme only — the rule and the cells stay put. */
static void sim_set_theme(Scene *s, int t)
{
    s->cfg.theme = wrap_idx(t, N_THEMES);
    palette_apply(s->cfg.theme, s->rule.states);
}

static void scene_alloc(Scene *s, int w, int h)
{
    free(s->grid.cur);  free(s->grid.nxt);
    s->grid.w = w;  s->grid.h = h;  s->grid.generation = 0;
    s->grid.cur = calloc((size_t)w * h, 1);
    s->grid.nxt = calloc((size_t)w * h, 1);
    area_alloc(&s->area, w, h);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->cfg.gens   = GENS_DEF;
    s->cfg.paused = false;
    s->cfg.preset = 0;
    s->cfg.theme  = 0;
    scene_alloc(s, w, h);
    sim_set_preset(s, 0);
}

static void scene_free(Scene *s)
{
    free(s->grid.cur);  free(s->grid.nxt);
    s->grid.cur = s->grid.nxt = NULL;
    area_free(&s->area);
}

/* the terminal changed size: rebuild at the new size and reseed. */
static void scene_resize(Scene *s, int w, int h)
{
    scene_alloc(s, w, h);
    sim_set_preset(s, s->cfg.preset);
}

/* ── §7 render ── */
/* Everything here only reads the scene and paints the terminal. Grid row r is
 * drawn one row down (row 0 is the data bar, the last row is the hints bar). */

/* pick the colour and brightness for a cell: fully-alive cells are bright and
 * bold; each step into the fading trail uses a dimmer colour, so the trail
 * fades away behind the live cells. */
static chtype cell_attr(int state, int N)
{
    if (state == cell_alive(N))
        return (chtype)(COLOR_PAIR(1) | A_BOLD);     /* fully alive */
    int idx = (N - 1 - state);                       /* how far into the trail */
    if (idx >= N_MAX - 1) idx = N_MAX - 2;
    return (chtype)(COLOR_PAIR(idx + 1) | A_DIM);    /* fading */
}

static void render_grid(const Scene *s, int rows)
{
    const Grid *g   = &s->grid;
    int         N   = s->rule.states;
    int         top = HUD_TOP_ROWS;                  /* one row down for the HUD */
    int         gh  = imin(g->h, rows - HUD_ROWS);   /* don't draw over the HUD */

    for (int r = 0; r < gh; r++) {
        for (int c = 0; c < g->w; c++) {
            uint8_t state = g->cur[r * g->w + c];
            if (state == CELL_DEAD) continue;
            chtype attr = cell_attr(state, N);
            attron(attr);
            mvaddch(r + top, c, (chtype)(unsigned char)CELL_GLYPH);
            attroff(attr);
        }
    }
}

/* paint one full-width bar: colour the whole row, then write the text cut off
 * at the screen width so a long line never spills onto the grid. */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* build the top-bar text: the rule, the theme, radius/states, the generation
 * count, the fast-forward dial, and the timing numbers. */
static void hud_status_line(char *buf, size_t n, const Scene *s,
                            int sim_fps, double fps)
{
    const Rule     *rule = &s->rule;
    const Settings *cfg  = &s->cfg;
    snprintf(buf, n,
             " AUTOMATON_2D  preset:%s (%d/%d)  R%d N%d  theme:%s (%d/%d)  "
             "gen:%-6ld  %dx/tick  %dHz  %4.1ffps  %s ",
             rule->name, cfg->preset + 1, N_PRESETS,
             rule->radius, rule->states,
             THEMES[cfg->theme].name, cfg->theme + 1, N_THEMES,
             s->grid.generation, cfg->gens, sim_fps, fps,
             cfg->paused ? "PAUSED" : "running");
}

static void render_hud(const Scene *s, int cols, int rows,
                       int sim_fps, double fps)
{
    char status[160];
    hud_status_line(status, sizeof status, s, sim_fps, fps);

    const char *keys =
        " q:quit  spc:pause  r:reset  n/p:preset  t/T:theme  "
        "+/-:speed  [/]:Hz ";

    hud_bar(0,        cols, PAIR_HUD,  status);  /* top: live readout */
    hud_bar(rows - 1, cols, PAIR_HINT, keys);    /* bottom: key hints */
}

/* ── §8 app ── */

/*
 * FpsMeter — averages the frame rate over a short window so the number in the
 * HUD holds still instead of jumping every frame. It tallies elapsed time and
 * frames, works out the rate once per window, then starts over.
 */
typedef struct {
    int64_t accum_ns;   /* time tallied since the last readout          */
    int     frames;     /* frames tallied since the last readout        */
    double  value;      /* the last rate worked out, shown in the HUD   */
} FpsMeter;

static void fps_meter_tick(FpsMeter *m, int64_t dt)
{
    m->frames++;
    m->accum_ns += dt;
    if (m->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        m->value    = (double)m->frames / ((double)m->accum_ns / (double)NS_PER_SEC);
        m->frames   = 0;
        m->accum_ns = 0;
    }
}

/*
 * App — the running program around the Scene: the world plus the loop's own
 * bookkeeping — terminal size, the tick rate, the time bank that paces ticks,
 * the fps meter, and the flags the signal handlers set. Kept apart from Scene
 * because these describe the program, not the simulation. There's one global
 * (g_app) so the signal handlers can reach the flags.
 */
typedef struct {
    Scene                 scene;       /* the world (§4-§7)                   */
    int                   cols, rows;  /* terminal size in cells              */
    int                   sim_fps;     /* how many times a second the world
                                        * steps; drawing is capped separately. */
    int64_t               sim_accum;   /* banks up real time, spent in whole
                                        * sim ticks.                          */
    FpsMeter              fps;          /* the smoothed frame-rate readout.    */
    volatile sig_atomic_t running;     /* 0 means quit; set by Ctrl-C/kill.   */
    volatile sig_atomic_t need_resize; /* set when the window resizes.        */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void screen_init(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
}

/* window resized: re-read the size and rebuild the scene to fit. */
static void app_resize(App *app)
{
    endwin();
    refresh();
    getmaxyx(stdscr, app->rows, app->cols);
    int gh = app->rows - HUD_ROWS;
    if (gh < 1) gh = 1;
    scene_resize(&app->scene, app->cols, gh);
    app->sim_accum   = 0;
    app->need_resize = 0;
}

/* nudge the fast-forward dial and the tick rate up or down a notch, each kept
 * inside its limits. */
static void settings_nudge_gens(Settings *c, int d)
{
    c->gens = clampi(c->gens + d, 1, GENS_MAX);
}
static void app_nudge_fps(App *app, int d)
{
    app->sim_fps = clampi(app->sim_fps + d, SIM_FPS_MIN, SIM_FPS_MAX);
}

/* act on a keypress; returns false only when the user wants to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene    *s   = &app->scene;
    Settings *cfg = &s->cfg;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC key */: return false;

    case ' ':           cfg->paused = !cfg->paused;          break;
    case 'r': case 'R': sim_seed(s);                         break;

    case 'n':           sim_set_preset(s, cfg->preset + 1);  break;
    case 'p':           sim_set_preset(s, cfg->preset - 1);  break;

    case 't':           sim_set_theme(s, cfg->theme + 1);    break;
    case 'T':           sim_set_theme(s, cfg->theme - 1);    break;

    case '=': case '+': settings_nudge_gens(cfg, +1);        break;
    case '-':           settings_nudge_gens(cfg, -1);        break;

    case ']':           app_nudge_fps(app, +SIM_FPS_STEP);   break;
    case '[':           app_nudge_fps(app, -SIM_FPS_STEP);   break;

    default: break;
    }
    return true;
}

/*
 * Spend banked time on simulation ticks: each tick (which runs `gens`
 * generations) happens once every 1/sim_fps seconds of real time. Keeping it
 * time-based means the world runs at the same speed no matter how fast we draw.
 */
static void app_advance(App *app, int64_t dt)
{
    int64_t tick = TICK_NS(app->sim_fps);
    app->sim_accum += dt;
    while (app->sim_accum >= tick) {
        scene_tick(&app->scene);
        app->sim_accum -= tick;
    }
}

/* draw the current state to the screen. */
static void app_draw(const App *app)
{
    erase();
    render_grid(&app->scene, app->rows);
    render_hud(&app->scene, app->cols, app->rows, app->sim_fps, app->fps.value);
    wnoutrefresh(stdscr);
    doupdate();
}

/* time since the last frame, capped so one big stall can't make the world
 * lurch ahead all at once. */
static int64_t frame_delta(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    return dt > DT_CAP_MS * NS_PER_MS ? DT_CAP_MS * NS_PER_MS : dt;
}

/* sleep just enough that the screen doesn't redraw faster than RENDER_FPS. */
static void app_pace_frame(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(TICK_NS(RENDER_FPS) - elapsed);
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

    screen_init();
    getmaxyx(stdscr, app->rows, app->cols);
    int gh = app->rows - HUD_ROWS;
    if (gh < 1) gh = 1;
    scene_init(&app->scene, app->cols, gh);

    int64_t frame_time = clock_ns();

    /*
     * The main loop, five steps each pass round:
     *   read a key, measure how much time passed, run any due sim ticks,
     *   sleep to cap the frame rate, then draw.
     */
    while (app->running) {
        if (app->need_resize) {
            app_resize(app);
            frame_time = clock_ns();
        }

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;

        int64_t dt = frame_delta(&frame_time);

        app_advance(app, dt);
        fps_meter_tick(&app->fps, dt);
        app_pace_frame(frame_time, dt);
        app_draw(app);
    }

    scene_free(&app->scene);
    endwin();
    return 0;
}
