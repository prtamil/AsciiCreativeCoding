/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lattice_gas.c — FHP-I Lattice Gas Automaton (Frisch-Hasslacher-Pomeau 1986)
 *
 * Watch real fluid physics emerge from simple particle rules.
 *
 * Particles enter from the LEFT, bounce off each other and obstacles on a
 * hexagonal grid, and collectively behave like a real fluid.  The Navier-
 * Stokes equations emerge automatically from two local rules:
 *   1. COLLISION — particles meeting in the same cell swap directions
 *                  in a way that conserves mass and momentum
 *   2. STREAMING — each particle moves one step in its direction
 *
 * COLOUR KEY (what you are watching):
 *   BLUE   — fluid moving LEFT  (slow zones, wake behind an obstacle)
 *   GREY   — fluid approximately still
 *   RED    — fluid moving RIGHT (the main current)
 *   ######   solid obstacle — particles bounce back off the walls
 *
 * Character density key:
 *   . , o O 0 @  — sparse to dense fluid
 *   (space)      — empty / no particles
 *
 * Display uses a 3×3 spatial average so you see the smooth macroscopic
 * flow field, not the noisy individual-particle level.
 *
 * Presets:
 *   1  Cylinder  — flow past a circular obstacle; BLUE wake forms behind it
 *   2  2-Slit    — two gaps in a wall; particles fan out after each slit
 *   3  3-Slit    — three gaps; interference-like fringes in the jets
 *   4  4-Slit    — four gaps; tighter jets, richer wake pattern
 *   5  Channel   — parallel walls; flow fastest at centre (Poiseuille)
 *   6  Free      — no inlet, no walls; random gas reaches equilibrium
 *
 * Keys:
 *   q/ESC   quit         p/Space  pause/resume     r  reset
 *   n/N     next/prev preset       t/T  next/prev theme
 *   i/I     inlet stronger/weaker  +/-  steps/frame faster/slower
 *   ]/[     FPS up/down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/lattice_gas.c -o lattice_gas -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 hex  §5 grid  §6 draw  §7 screen  §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : FHP-I Lattice Gas Automaton (Frisch, Hasslacher, Pomeau 1986).
 *                  A cellular automaton where each cell holds 6 bits —
 *                  one per hexagonal lattice direction (0–5).  Each tick:
 *                    1. COLLISION: particles in same cell swap according to
 *                       lookup table that conserves mass and momentum.
 *                    2. STREAMING: each particle hops one cell in its direction.
 *
 * Physics        : Emergent hydrodynamics.
 *                  The H-theorem guarantees that many FHP particles, averaged
 *                  over many cells, obey the Navier-Stokes equations.
 *                  This is a proof that macroscopic fluid behaviour arises
 *                  purely from microscopic conservation laws — no explicit PDEs.
 *                  The hexagonal grid removes velocity-space anisotropy that
 *                  afflicted earlier square-lattice gas models.
 *
 * Rendering      : 3×3 spatial averaging before display.
 *                  Individual cells are binary (particle or no particle), too
 *                  noisy to show macroscopic flow.  Averaging over 9 cells
 *                  gives the local particle density ρ and mean momentum ⟨p⟩.
 *                  These map to color (velocity direction) and character
 *                  (density level) for smooth visualisation.
 *
 * Performance    : O(W×H) per step — one lookup per cell.  The collision
 *                  table is precomputed at init.  Very cache-friendly since
 *                  each cell is a single byte.
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

#define ROWS_MAX   128
#define COLS_MAX   512

/* Physics steps computed per rendered frame — more = faster time-lapse */
#define STEPS_DEF   8
#define STEPS_MIN   1
#define STEPS_MAX  32

/* Inlet: probability that a left-column cell gets an eastward particle/step */
#define INLET_PROB_DEF  0.55f
#define INLET_PROB_MIN  0.10f
#define INLET_PROB_MAX  0.95f
#define INLET_PROB_STEP 0.05f

#define SIM_FPS_DEF  20
#define SIM_FPS_MIN   5
#define SIM_FPS_MAX  60
#define SIM_FPS_STEP  5

/* Correct circular obstacle for terminal cell aspect ratio (height/width ≈ 2) */
#define ASPECT_R  2.0f

#define N_PRESETS  6
#define N_THEMES   5
#define NS_PER_SEC 1000000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                            .tv_nsec = (long)(ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * 9 momentum colour pairs (CP_M1..CP_M9): strongest-west → strongest-east.
 *
 * Colour encodes flow direction; the ASCII character encodes density.
 * Characters ". , o O 0 @" go from sparse to dense fluid.
 * This works in both 256-color and 8-color terminals.
 *
 * CP_WALL  = solid obstacle cells  (white '#')
 * CP_HUD   = status bar
 */
enum {
    CP_M1=1, CP_M2, CP_M3, CP_M4, CP_M5,
    CP_M6,   CP_M7, CP_M8, CP_M9,
    CP_WALL=10, CP_HUD=11,
};

typedef struct {
    short col[9];    /* 256-color fg: west(0) → east(8), drawn on black bg */
    short col8[9];   /* 8-color fg fallback */
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 Classic — deep blue → cyan → grey → orange → red */
    { { 21, 33, 51, 87, 244, 208, 202, 196, 160 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED },
      "Classic" },
    /* 1 Ocean   — midnight blue → teal → cyan → white */
    { { 19, 27, 39, 51, 87, 123, 159, 195, 231 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN,
        COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
      "Ocean" },
    /* 2 Plasma  — deep purple → magenta → gold */
    { { 91, 129, 165, 201, 207, 213, 214, 220, 226 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_RED, COLOR_RED,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
      "Plasma" },
    /* 3 Matrix  — dark green → bright lime */
    { { 22, 28, 34, 40, 46, 82, 118, 154, 190 },
      { COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE },
      "Matrix" },
    /* 4 Heat    — dark red → orange → bright yellow */
    { { 52, 88, 124, 166, 172, 178, 214, 220, 226 },
      { COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
      "Heat" },
};

static bool g_has_256;

static void theme_apply(int ti)
{
    const Theme *th = &k_themes[ti];
    for (int i = 0; i < 9; i++) {
        short fg = g_has_256 ? th->col[i] : th->col8[i];
        init_pair(CP_M1 + i, fg, COLOR_BLACK);
    }
    init_pair(CP_WALL, COLOR_WHITE,  COLOR_BLACK);
    init_pair(CP_HUD,  g_has_256 ? 255 : COLOR_WHITE,
                       g_has_256 ? 236 : COLOR_BLACK);
}

/* ===================================================================== */
/* §4  hex grid — directions and collision tables                         */
/* ===================================================================== */

/*
 * Hex offset layout, pointy-top hexagons, offset rows.
 * Direction bits:  0=E  1=NE  2=NW  3=W  4=SW  5=SE
 * Opposite of d:   (d + 3) % 6
 *
 * Even row neighbours:  E=(0,+1) NE=(-1,0)  NW=(-1,-1) W=(0,-1) SW=(+1,-1) SE=(+1,0)
 * Odd  row neighbours:  E=(0,+1) NE=(-1,+1) NW=(-1,0)  W=(0,-1) SW=(+1,0)  SE=(+1,+1)
 */
static const int DIR_DR[2][6] = {
    {  0, -1, -1,  0,  1,  1 },
    {  0, -1, -1,  0,  1,  1 },
};
static const int DIR_DC[2][6] = {
    { +1,  0, -1, -1, -1,  0 },
    { +1, +1,  0, -1,  0, +1 },
};

/*
 * Two 64-entry lookup tables resolve the 5 non-trivial FHP-I collisions.
 * Parity = (row+col)&1; alternating CW/CCW removes spurious global chirality.
 *
 * Head-on pairs (ambiguous, two valid post-states):
 *   parity 0 (CCW):  0x09→0x12→0x24→0x09 ...
 *   parity 1 (CW):   0x09→0x24→0x12→0x09 ...
 * 3-particle symmetric (unique rotation, parity-independent):
 *   0x15 ↔ 0x2A
 * All 59 other patterns: identity.
 */
static uint8_t g_col[2][64];

static void build_collision_tables(void)
{
    for (int s = 0; s < 64; s++) g_col[0][s] = g_col[1][s] = (uint8_t)s;
    g_col[0][0x09]=0x12; g_col[1][0x09]=0x24;
    g_col[0][0x12]=0x24; g_col[1][0x12]=0x09;
    g_col[0][0x24]=0x09; g_col[1][0x24]=0x12;
    g_col[0][0x15]=0x2A; g_col[1][0x15]=0x2A;
    g_col[0][0x2A]=0x15; g_col[1][0x2A]=0x15;
}

/*
 * Horizontal momentum ×2 (integer to avoid floats in the hot draw loop).
 * Hex direction unit-vector x-components: E=+1 NE=+½ NW=-½ W=-1 SW=-½ SE=+½
 * Scaled by 2: E=+2 NE=+1 NW=-1 W=-2 SW=-1 SE=+1
 */
static inline int cell_mx2(uint8_t s)
{
    return  2*((s>>0)&1) + ((s>>1)&1) - ((s>>2)&1)
           -2*((s>>3)&1) - ((s>>4)&1) + ((s>>5)&1);
}

/* ===================================================================== */
/* §5  grid state and simulation                                          */
/* ===================================================================== */

static uint8_t g_grid[ROWS_MAX][COLS_MAX];   /* particle bits per cell   */
static uint8_t g_buf [ROWS_MAX][COLS_MAX];   /* streaming double-buffer  */
static bool    g_wall[ROWS_MAX][COLS_MAX];   /* solid obstacle mask      */

static int   g_rows, g_cols;
static int   g_preset     = 0;
static int   g_theme      = 0;
static int   g_steps      = STEPS_DEF;
static int   g_sim_fps    = SIM_FPS_DEF;
static bool  g_paused     = false;
static bool  g_inlet_on   = true;
static float g_inlet_prob = INLET_PROB_DEF;
static long  g_tick       = 0;

/* xorshift32 — fast, 32-bit, no global locking */
static uint32_t g_rng = 12345u;
static inline uint32_t rng_next(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}
static inline float rng_float(void)
{
    return (float)(rng_next() >> 8) / (float)(1u << 24);
}

/* ------------------------------------------------------------------ */

typedef struct {
    bool  inlet;
    float init_density;   /* 0 = start empty; flow develops from inlet */
    const char *name;
    const char *desc;     /* plain-English description shown in HUD */
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { true,  0.0f, "Cylinder",
      "Particles hit cylinder -- watch BLUE wake form and grow behind it" },
    { true,  0.0f, "2-Slit",
      "Two gaps in a wall -- jets spread out and meet in the middle" },
    { true,  0.0f, "3-Slit",
      "Three gaps -- three jets spread and interfere beyond the wall" },
    { true,  0.0f, "4-Slit",
      "Four gaps -- tight jets, rich interference pattern downstream" },
    { true,  0.0f, "Channel",
      "Parallel walls top & bottom -- flow is fastest in the centre (Poiseuille)" },
    { false, 0.40f, "Free",
      "No inlet, no walls -- random gas collides and spreads toward equilibrium" },
};

/* ------------------------------------------------------------------ */

/*
 * Build a vertical slit wall at column wc with n_slits evenly-spaced gaps.
 * Each gap is ~11% of the screen height; centres are equally distributed.
 */
static void add_slit_wall(int n_slits)
{
    int wc    = g_cols / 3;
    int gap_h = g_rows / (n_slits * 3);   /* each gap height */
    if (gap_h < 2) gap_h = 2;

    for (int r = 0; r < g_rows; r++) {
        bool in_slit = false;
        for (int s = 0; s < n_slits; s++) {
            int center = g_rows * (s + 1) / (n_slits + 1);
            if (r >= center - gap_h/2 && r < center + gap_h/2) {
                in_slit = true;
                break;
            }
        }
        if (!in_slit)
            for (int cc = wc; cc < wc + 2 && cc < g_cols; cc++)
                { g_wall[r][cc] = true; g_grid[r][cc] = 0; }
    }
}

static void scene_init(int preset)
{
    g_preset   = preset;
    g_inlet_on = k_presets[preset].inlet;
    g_tick     = 0;
    memset(g_wall, 0, sizeof g_wall);
    memset(g_grid, 0, sizeof g_grid);

    /* Random seed only for Free preset; others start empty so flow develops */
    float d = k_presets[preset].init_density;
    if (d > 0.0f)
        for (int r = 0; r < g_rows; r++)
            for (int c = 0; c < g_cols; c++)
                for (int b = 0; b < 6; b++)
                    if (rng_float() < d)
                        g_grid[r][c] |= (uint8_t)(1u << b);

    switch (preset) {
    case 0: {   /* Cylinder: circular obstacle at 2/5 from left */
        int cx = g_cols * 2 / 5, cy = g_rows / 2;
        int R  = g_cols / 12; if (R < 3) R = 3;
        for (int r = 0; r < g_rows; r++)
            for (int c = 0; c < g_cols; c++) {
                float dx = (float)(c - cx);
                float dy = (float)(r - cy) * ASPECT_R;
                if (dx*dx + dy*dy < (float)(R*R))
                    { g_wall[r][c] = true; g_grid[r][c] = 0; }
            }
        break;
    }
    case 1: add_slit_wall(2); break;
    case 2: add_slit_wall(3); break;
    case 3: add_slit_wall(4); break;
    case 4: {   /* Channel: solid top and bottom walls */
        int wh = (g_rows > 8) ? 2 : 1;
        for (int c = 0; c < g_cols; c++)
            for (int i = 0; i < wh; i++) {
                g_wall[i][c] = g_wall[g_rows-1-i][c] = true;
                g_grid[i][c] = g_grid[g_rows-1-i][c] = 0;
            }
        break;
    }
    case 5: break;  /* Free: no obstacles */
    }
}

/* ------------------------------------------------------------------ */

static void inject_inlet(void)
{
    /* Force eastward particles at left column — this drives the flow */
    for (int r = 0; r < g_rows; r++) {
        if (g_wall[r][0]) continue;
        if (rng_float() < g_inlet_prob)
            g_grid[r][0] |= 0x01u;   /* bit 0 = East direction */
    }
}

static void grid_collide(void)
{
    for (int r = 0; r < g_rows; r++)
        for (int c = 0; c < g_cols; c++) {
            if (g_wall[r][c]) continue;
            g_grid[r][c] = g_col[(r+c)&1][g_grid[r][c] & 0x3Fu];
        }
}

static void grid_stream(void)
{
    memset(g_buf, 0, sizeof g_buf);
    for (int r = 0; r < g_rows; r++) {
        int par = r & 1;
        for (int c = 0; c < g_cols; c++) {
            if (g_wall[r][c]) continue;
            uint8_t s = g_grid[r][c];
            if (!s) continue;
            for (int d = 0; d < 6; d++) {
                if (!(s & (uint8_t)(1u << d))) continue;
                int nr = (r + DIR_DR[par][d] + g_rows) % g_rows;
                int nc = (c + DIR_DC[par][d] + g_cols) % g_cols;
                if (g_wall[nr][nc])
                    g_buf[r][c]   |= (uint8_t)(1u << ((d+3)%6));
                else
                    g_buf[nr][nc] |= (uint8_t)(1u << d);
            }
        }
    }
    memcpy(g_grid, g_buf, sizeof g_grid);
}

static void grid_step(void)
{
    if (g_inlet_on) inject_inlet();
    grid_collide();
    grid_stream();
    g_tick++;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

/*
 * cell_avg: average particle density and horizontal momentum over a 3×3
 * neighbourhood.  This smooths out discrete particle noise and reveals
 * the macroscopic flow patterns (wake, boundary layer, jets).
 */
static void cell_avg(int r, int c, float *den_out, float *mx_out)
{
    float den = 0.0f, mx = 0.0f;
    int   cnt = 0;
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            int nr = r + dr, nc = c + dc;
            if (nr < 0 || nr >= g_rows || nc < 0 || nc >= g_cols) continue;
            if (g_wall[nr][nc]) continue;
            uint8_t s = g_grid[nr][nc];
            den += (float)__builtin_popcount((unsigned)s);
            mx  += (float)cell_mx2(s);
            cnt++;
        }
    }
    if (cnt > 0) { den /= (float)cnt; mx /= (float)cnt; }
    *den_out = den;
    *mx_out  = mx;
}

/*
 * Map averaged horizontal momentum to colour-pair index 0..8.
 * After 3×3 averaging, mx lives roughly in [-2, +2].
 */
static inline int mx_to_idx(float mx)
{
    float t = (mx + 2.0f) / 4.0f;   /* [-2,+2] → [0,1] */
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    int idx = (int)(t * 8.49f);
    return (idx < 0) ? 0 : (idx > 8) ? 8 : idx;
}

/*
 * Fluid character set — density drives character choice.
 * Sparse fluid: '.' or ','  Dense fluid: 'O' '0' '@'
 * Index 0 = empty (never reached via den threshold), 1..6 = particle counts.
 */
static const char k_fluid[7] = { ' ', '.', ',', 'o', 'O', '0', '@' };

static void scene_draw(void)
{
    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int draw_rows = (g_rows < rows - 2) ? g_rows : rows - 2;
    int draw_cols = (g_cols < cols)     ? g_cols : cols;

    /* ── main fluid field ── */
    for (int r = 0; r < draw_rows; r++) {
        for (int c = 0; c < draw_cols; c++) {

            if (g_wall[r][c]) {
                attron(COLOR_PAIR(CP_WALL) | A_BOLD);
                mvaddch(r, c, '#');
                attroff(COLOR_PAIR(CP_WALL) | A_BOLD);
                continue;
            }

            float den, mx;
            cell_avg(r, c, &den, &mx);

            if (den < 0.25f) {
                /* Nearly empty — leave as terminal background (black) */
                mvaddch(r, c, ' ');
                continue;
            }

            /* Density → character, momentum → colour */
            int n = (int)(den + 0.5f);
            if (n < 1) n = 1;
            if (n > 6) n = 6;

            int cp    = CP_M1 + mx_to_idx(mx);
            int bold  = (den >= 3.5f) ? A_BOLD : 0;

            attron(COLOR_PAIR(cp) | bold);
            mvaddch(r, c, (chtype)k_fluid[n]);
            attroff(COLOR_PAIR(cp) | bold);
        }
    }

    /* ── HUD row 1: preset description ── */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(rows-2, 0,
        " [%d] %-8s  %s  %s  tick:%-6ld  inlet:%.0f%%  steps:%d  fps:%d ",
        g_preset + 1,
        k_presets[g_preset].name,
        k_presets[g_preset].desc,
        g_paused ? " ** PAUSED **" : "",
        g_tick, g_inlet_prob * 100.0f, g_steps, g_sim_fps);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* ── HUD row 2: colour legend + controls ── */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows-1, 0,
        "  Colour: BLUE<-left  GREY=still  RED->right  "
        "Chars: .=sparse  @=dense  "
        "[n/N]preset  [t/T]theme  [i/I]inlet  [+/-]speed  [r]reset  [p]pause  [q]quit");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    start_color();
    g_has_256 = (COLORS >= 256);
    theme_apply(g_theme);
}

static void screen_resize(void)
{
    endwin(); refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = (rows-2 < ROWS_MAX) ? rows-2 : ROWS_MAX;
    g_cols = (cols   < COLS_MAX) ? cols    : COLS_MAX;
    g_resize = 0;
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);

    g_rng = (uint32_t)time(NULL) ^ 0xFACEB00Cu;
    build_collision_tables();
    screen_init();

    { int rows, cols; getmaxyx(stdscr, rows, cols);
      g_rows = (rows-2 < ROWS_MAX) ? rows-2 : ROWS_MAX;
      g_cols = (cols   < COLS_MAX) ? cols    : COLS_MAX; }

    scene_init(g_preset);
    int64_t next_tick = clock_ns();

    while (!g_quit) {

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case 'p': case ' ': g_paused = !g_paused; break;
            case 'r': scene_init(g_preset); break;
            case 'n': scene_init((g_preset + 1) % N_PRESETS); break;
            case 'N': scene_init((g_preset + N_PRESETS - 1) % N_PRESETS); break;
            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'T':
                g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'i':
                if (g_inlet_prob < INLET_PROB_MAX) g_inlet_prob += INLET_PROB_STEP;
                break;
            case 'I':
                if (g_inlet_prob > INLET_PROB_MIN) g_inlet_prob -= INLET_PROB_STEP;
                break;
            case '+': case '=':
                if (g_steps < STEPS_MAX) g_steps++;
                break;
            case '-':
                if (g_steps > STEPS_MIN) g_steps--;
                break;
            case ']':
                if (g_sim_fps < SIM_FPS_MAX) g_sim_fps += SIM_FPS_STEP;
                break;
            case '[':
                if (g_sim_fps > SIM_FPS_MIN) g_sim_fps -= SIM_FPS_STEP;
                break;
            }
        }

        /* ── resize ── */
        if (g_resize) { screen_resize(); scene_init(g_preset); }

        /* ── simulate ── */
        int64_t now = clock_ns();
        if (!g_paused && now >= next_tick) {
            for (int s = 0; s < g_steps; s++) grid_step();
            next_tick = now + TICK_NS(g_sim_fps);
        }

        /* ── render ── */
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();

        /* ── sleep until next frame ── */
        clock_sleep_ns(next_tick - clock_ns() - 1000000LL);
    }

    endwin();
    return 0;
}
