/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * tree_la.c  —  Dielectric Breakdown Model (DBM) Laplace fractal
 *
 * A Laplace potential field phi is solved iteratively (Gauss-Seidel).
 * Boundary conditions impose a high-potential source and a grounded
 * sink; the growing tree structure is a grounded conductor.
 *
 * Growth probability at each frontier cell is proportional to phi^eta:
 *   high eta  →  growth concentrates at high-phi tips  →  jagged / fractal
 *   low  eta  →  growth more uniform across frontier  →  rounder / Eden-like
 *
 * Three presets control seed placement and boundary conditions:
 *   Tree      — seed at bottom-center, phi=1 at top (grows upward)
 *   Lightning — seed at top-center, phi=1 at bottom (grows downward)
 *   Coral     — seed at center, phi=1 on all 4 boundary edges (grows outward)
 *
 * Color is by age_delta = g_step - cell_join_step (same 5-level scheme as
 * diffusion_map.c): newest=bright/bold '@', oldest=dim '.'.
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   r         reset (re-seed, clear phi)
 *   1 / 2 / 3 switch preset (resets automatically)
 *   t / T     next / prev theme
 *   e         increase eta by 0.25 (max 4.0)
 *   E         decrease eta by 0.25 (min 1.0)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra tree_la.c -o tree_la -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  lcg rng
 *   §4  color / themes
 *   §5  grid & phi
 *   §6  Gauss-Seidel relaxation
 *   §7  frontier growth
 *   §8  scene
 *   §9  screen
 *   §10 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Dielectric Breakdown Model (DBM, Niemeyer et al. 1984).
 *                  A Laplace equation (∇²φ = 0) is solved over the grid;
 *                  the growing tree structure is a grounded conductor (φ=0).
 *                  At each step, one frontier cell is chosen to join the tree
 *                  with probability ∝ φ(x,y)^η.
 *
 * Math           : The Laplace equation ∇²φ = 0 is solved iteratively via
 *                  Gauss-Seidel relaxation:
 *                    φ_{k+1}(i,j) = (φ(i±1,j) + φ(i,j±1)) / 4
 *                  (5-point stencil, repeated until convergence).
 *                  At η=1: reduces to DLA (diffusion = Laplace potential).
 *                  At η→∞: growth concentrates only at the highest-φ tip →
 *                  a single straight needle (no branching).
 *                  At η=0: growth is uniform → Eden-model-like compact blob.
 *
 * Physics        : Models dielectric breakdown (lightning channels through
 *                  insulating material), electrodeposition, and solidification
 *                  from a supercooled melt — all governed by Laplace/diffusion
 *                  equations near a growing, absorbing boundary.
 *
 * Performance    : Gauss-Seidel iterations per growth step is the bottleneck.
 *                  Fewer iterations → faster but less accurate φ → growth bias.
 *                  The frontier set is stored explicitly for O(1) sampling.
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
    ROWS_MAX        =  80,
    COLS_MAX        = 300,

    RENDER_FPS      =  30,

    N_RELAX         =   8,   /* Gauss-Seidel passes per frame              */
    N_GROW          =   1,   /* new cells added per frame                  */

    N_THEMES        =   5,
    N_AGE_LEVELS    =   5,   /* color pairs CP_A0 … CP_A4                  */
    CP_HUD          =   1,   /* HUD color pair                             */
    CP_A0           =   2,   /* newest (offset 0)                          */

    FPS_UPDATE_MS   = 500,
};

/* age_delta thresholds */
static const int AGE_THRESH[N_AGE_LEVELS] = { 5, 20, 80, 300, INT32_MAX };

#define ETA_MIN   1.0f
#define ETA_MAX   4.0f
#define ETA_STEP  0.25f
#define ETA_DEFAULT_TREE      1.5f
#define ETA_DEFAULT_LIGHTNING 2.5f
#define ETA_DEFAULT_CORAL     2.0f

#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS  1000000LL

typedef enum { PRESET_TREE = 0, PRESET_LIGHTNING, PRESET_CORAL, N_PRESETS } Preset;

static const char *PRESET_NAME[N_PRESETS] = { "Tree", "Lightning", "Coral" };

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
/* §3  lcg rng                                                           */
/* ===================================================================== */

static uint32_t g_lcg;

static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §4  color / themes                                                    */
/* ===================================================================== */

/*
 * 5 themes × 5 age levels (newest → oldest):
 *
 *   Plasma: 231 213 165  93  54  white→pink→purple
 *   Fire:   231 226 208 196 124  white→yellow→orange→red→dark
 *   Ice:    231 123  51  33  17  white→cyan→blue→dark
 *   Ghost:  255 249 244 240 235  bright→dark grays
 *   Neon:   231  51  46  21  17  white/cyan/green/blue
 */
static const short THEME_FG[N_THEMES][N_AGE_LEVELS] = {
    { 231, 213, 165,  93,  54 },  /* Plasma */
    { 231, 226, 208, 196, 124 },  /* Fire   */
    { 231, 123,  51,  33,  17 },  /* Ice    */
    { 255, 249, 244, 240, 235 },  /* Ghost  */
    { 231,  51,  46,  21,  17 },  /* Neon   */
};

static const char *THEME_NAME[N_THEMES] = {
    "Plasma", "Fire", "Ice", "Ghost", "Neon"
};

static const short THEME_FG8[N_AGE_LEVELS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE
};

static int g_theme;

static void color_init_theme(void)
{
    if (COLORS >= 256)
        init_pair(CP_HUD, 250, COLOR_BLACK);
    else
        init_pair(CP_HUD, COLOR_WHITE, COLOR_BLACK);

    for (int i = 0; i < N_AGE_LEVELS; i++) {
        if (COLORS >= 256)
            init_pair((short)(CP_A0 + i),
                      THEME_FG[g_theme][i], COLOR_BLACK);
        else
            init_pair((short)(CP_A0 + i),
                      THEME_FG8[i], COLOR_BLACK);
    }
}

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
/* §5  grid & phi                                                        */
/* ===================================================================== */

static uint8_t  g_tree[ROWS_MAX][COLS_MAX]; /* 1 if in tree, 0 otherwise */
static float    g_phi [ROWS_MAX][COLS_MAX]; /* electric potential [0,1]   */
static uint16_t g_age [ROWS_MAX][COLS_MAX]; /* growth step when joined    */

static int g_rows, g_cols;
static int g_step;          /* growth step counter */
static int g_tree_size;     /* total cells in tree */

static Preset g_preset;
static float  g_eta;

static void grid_set_size(int cols, int rows)
{
    g_cols = (cols < COLS_MAX) ? cols : COLS_MAX;
    g_rows = (rows < ROWS_MAX) ? rows : ROWS_MAX;
}

/*
 * Enforce boundary conditions for the current preset.
 * Called after every Gauss-Seidel sweep.
 */
static void phi_enforce_bc(void)
{
    switch (g_preset) {

    case PRESET_TREE:
        /* Top row = 1.0 (sky/source), bottom row = 0.0 (ground) */
        for (int c = 0; c < g_cols; c++) {
            g_phi[0][c]           = 1.0f;
            g_phi[g_rows-1][c]    = 0.0f;
        }
        /* Neumann left/right */
        for (int r = 1; r < g_rows - 1; r++) {
            g_phi[r][0]          = g_phi[r][1];
            g_phi[r][g_cols - 1] = g_phi[r][g_cols - 2];
        }
        break;

    case PRESET_LIGHTNING:
        /* Bottom row = 1.0 (source), top row = 0.0 */
        for (int c = 0; c < g_cols; c++) {
            g_phi[0][c]           = 0.0f;
            g_phi[g_rows-1][c]    = 1.0f;
        }
        for (int r = 1; r < g_rows - 1; r++) {
            g_phi[r][0]          = g_phi[r][1];
            g_phi[r][g_cols - 1] = g_phi[r][g_cols - 2];
        }
        break;

    case PRESET_CORAL:
        /* All 4 boundary edges = 1.0 */
        for (int c = 0; c < g_cols; c++) {
            g_phi[0][c]           = 1.0f;
            g_phi[g_rows-1][c]    = 1.0f;
        }
        for (int r = 0; r < g_rows; r++) {
            g_phi[r][0]          = 1.0f;
            g_phi[r][g_cols - 1] = 1.0f;
        }
        break;

    default: break;
    }

    /* Tree cells are grounded conductors (phi = 0) */
    for (int r = 0; r < g_rows; r++)
        for (int c = 0; c < g_cols; c++)
            if (g_tree[r][c])
                g_phi[r][c] = 0.0f;
}

/*
 * Initialize phi to a reasonable starting gradient (speeds GS convergence).
 * For Tree/Lightning a linear ramp; for Coral a radial distance-to-boundary.
 */
static void phi_init(void)
{
    switch (g_preset) {

    case PRESET_TREE:
        /* Linear gradient: 1.0 at top, 0.0 at bottom */
        for (int r = 0; r < g_rows; r++)
            for (int c = 0; c < g_cols; c++)
                g_phi[r][c] = (g_rows > 1)
                    ? (float)(g_rows - 1 - r) / (float)(g_rows - 1)
                    : 0.0f;
        break;

    case PRESET_LIGHTNING:
        /* Linear gradient: 0.0 at top, 1.0 at bottom */
        for (int r = 0; r < g_rows; r++)
            for (int c = 0; c < g_cols; c++)
                g_phi[r][c] = (g_rows > 1)
                    ? (float)r / (float)(g_rows - 1)
                    : 0.0f;
        break;

    case PRESET_CORAL: {
        /* phi = normalized distance from center toward boundary */
        float cx = (float)(g_cols - 1) * 0.5f;
        float cy = (float)(g_rows - 1) * 0.5f;
        float max_d = (cx > cy) ? cx : cy;
        for (int r = 0; r < g_rows; r++) {
            for (int c = 0; c < g_cols; c++) {
                float dx = (float)c - cx;
                float dy = (float)r - cy;
                float d  = sqrtf(dx*dx + dy*dy);
                g_phi[r][c] = (max_d > 0.0f) ? (d / max_d) : 0.0f;
                if (g_phi[r][c] > 1.0f) g_phi[r][c] = 1.0f;
            }
        }
        break;
    }
    default: break;
    }
}

/*
 * Place the seed cell and initialize g_tree, g_age, g_step, g_tree_size.
 * phi is initialized separately.
 */
static void grid_reset(void)
{
    memset(g_tree, 0, sizeof g_tree);
    memset(g_age,  0, sizeof g_age);
    g_step      = 1;
    g_tree_size = 0;

    int seed_r, seed_c;

    switch (g_preset) {
    case PRESET_TREE:
        /* Seed at bottom-center */
        seed_r = g_rows - 1;
        seed_c = g_cols / 2;
        break;
    case PRESET_LIGHTNING:
        /* Seed at top-center */
        seed_r = 0;
        seed_c = g_cols / 2;
        break;
    case PRESET_CORAL:
    default:
        /* Seed at center */
        seed_r = g_rows / 2;
        seed_c = g_cols / 2;
        break;
    }

    g_tree[seed_r][seed_c] = 1;
    g_age [seed_r][seed_c] = 1;
    g_tree_size = 1;

    phi_init();
    phi_enforce_bc();
}

static void grid_draw(void)
{
    for (int r = 0; r < g_rows; r++) {
        for (int c = 0; c < g_cols; c++) {
            if (!g_tree[r][c]) continue;

            int age_delta = g_step - (int)(g_age[r][c]);
            if (age_delta < 0) age_delta = 0;

            int pair  = age_pair(age_delta);
            chtype ch = age_char(age_delta);
            attr_t attr = (attr_t)COLOR_PAIR(pair);
            if (age_delta <= AGE_THRESH[0]) attr |= A_BOLD;

            attron(attr);
            mvaddch(r, c, ch);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §6  Gauss-Seidel relaxation                                          */
/* ===================================================================== */

static void phi_relax(void)
{
    for (int pass = 0; pass < N_RELAX; pass++) {
        for (int r = 1; r < g_rows - 1; r++) {
            for (int c = 1; c < g_cols - 1; c++) {
                if (g_tree[r][c]) continue;
                g_phi[r][c] = 0.25f * (g_phi[r-1][c] + g_phi[r+1][c]
                                      + g_phi[r][c-1] + g_phi[r][c+1]);
            }
        }
        phi_enforce_bc();
    }
}

/* ===================================================================== */
/* §7  frontier growth                                                   */
/* ===================================================================== */

static const int DR4[4] = { -1, 1,  0, 0 };
static const int DC4[4] = {  0, 0, -1, 1 };

static bool cell_is_frontier(int r, int c)
{
    if (g_tree[r][c]) return false;
    for (int d = 0; d < 4; d++) {
        int nr = r + DR4[d];
        int nc = c + DC4[d];
        if (nr < 0 || nr >= g_rows || nc < 0 || nc >= g_cols) continue;
        if (g_tree[nr][nc]) return true;
    }
    return false;
}

/*
 * Grow N_GROW new cells using weighted random selection.
 * Weight of frontier cell = phi[r][c]^eta.
 * We use a linear scan with cumulative sum, picking proportional to weight.
 */
static void tree_grow(void)
{
    for (int grow = 0; grow < N_GROW; grow++) {

        /* Compute total weight across all frontier cells */
        double total = 0.0;
        for (int r = 0; r < g_rows; r++) {
            for (int c = 0; c < g_cols; c++) {
                if (!cell_is_frontier(r, c)) continue;
                float p = g_phi[r][c];
                if (p < 0.0f) p = 0.0f;
                total += (double)powf(p, g_eta);
            }
        }

        if (total <= 0.0) return;  /* nothing can grow */

        /* Select one frontier cell by weighted random */
        double pick = (double)lcg_f() * total;
        double cum  = 0.0;
        int    sel_r = -1, sel_c = -1;

        for (int r = 0; r < g_rows && sel_r < 0; r++) {
            for (int c = 0; c < g_cols && sel_r < 0; c++) {
                if (!cell_is_frontier(r, c)) continue;
                float p = g_phi[r][c];
                if (p < 0.0f) p = 0.0f;
                cum += (double)powf(p, g_eta);
                if (cum >= pick) {
                    sel_r = r;
                    sel_c = c;
                }
            }
        }

        if (sel_r < 0) return;   /* rounding edge case */

        /* Add selected cell to tree */
        g_tree[sel_r][sel_c] = 1;
        g_age [sel_r][sel_c] = (uint16_t)(g_step & 0xFFFF);
        g_phi [sel_r][sel_c] = 0.0f;
        g_tree_size++;
        g_step++;
    }
}

/* ===================================================================== */
/* §8  scene                                                             */
/* ===================================================================== */

static bool g_paused;

static void scene_set_preset(Preset p)
{
    g_preset = p;
    switch (p) {
    case PRESET_TREE:      g_eta = ETA_DEFAULT_TREE;      break;
    case PRESET_LIGHTNING: g_eta = ETA_DEFAULT_LIGHTNING; break;
    case PRESET_CORAL:
    default:               g_eta = ETA_DEFAULT_CORAL;     break;
    }
    grid_reset();
}

static void scene_init(void)
{
    g_paused = false;
    scene_set_preset(PRESET_TREE);
}

static void scene_tick(void)
{
    if (g_paused) return;
    phi_relax();
    tree_grow();
}

/* ===================================================================== */
/* §9  screen                                                            */
/* ===================================================================== */

static int g_scr_rows, g_scr_cols;

static void screen_init_ncurses(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    start_color();
    color_init_theme();
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
}

static void screen_draw_hud(double fps)
{
    char buf[160];
    snprintf(buf, sizeof buf,
             " %s | eta:%.2f | size:%-5d | %s | %4.1f fps ",
             PRESET_NAME[g_preset],
             (double)g_eta,
             g_tree_size,
             THEME_NAME[g_theme],
             fps);

    int hud_row = g_rows < g_scr_rows ? g_rows : g_scr_rows - 1;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(hud_row, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §10 app                                                               */
/* ===================================================================== */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_resize(void)
{
    endwin();
    refresh();
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
    grid_set_size(g_scr_cols, g_scr_rows - 1);
    grid_reset();
    g_need_resize = 0;
}

static bool app_handle_key(int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:
        return false;

    case 'p': case 'P': case ' ':
        g_paused = !g_paused;
        break;

    case 'r': case 'R':
        grid_reset();
        break;

    case '1':
        scene_set_preset(PRESET_TREE);
        break;

    case '2':
        scene_set_preset(PRESET_LIGHTNING);
        break;

    case '3':
        scene_set_preset(PRESET_CORAL);
        break;

    case 't':
        g_theme = (g_theme + 1) % N_THEMES;
        color_init_theme();
        break;

    case 'T':
        g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
        color_init_theme();
        break;

    case 'e':
        g_eta += ETA_STEP;
        if (g_eta > ETA_MAX) g_eta = ETA_MAX;
        break;

    case 'E':
        g_eta -= ETA_STEP;
        if (g_eta < ETA_MIN) g_eta = ETA_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_lcg = (uint32_t)(ts.tv_nsec ^ ts.tv_sec);

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    g_theme = 0;

    screen_init_ncurses();
    grid_set_size(g_scr_cols, g_scr_rows - 1);
    scene_init();

    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;
    int64_t last_frame  = clock_ns();

    while (g_running) {

        if (g_need_resize) {
            app_resize();
            last_frame  = clock_ns();
            fps_accum   = 0;
            frame_count = 0;
        }

        scene_tick();

        /* FPS measurement */
        int64_t now = clock_ns();
        int64_t dt  = now - last_frame;
        last_frame  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;
        fps_accum += dt;
        frame_count++;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_accum   = 0;
            frame_count = 0;
        }

        /* Draw */
        erase();
        grid_draw();
        screen_draw_hud(fps_display);
        screen_present();

        /* Key input */
        int ch = getch();
        if (ch != ERR && !app_handle_key(ch))
            g_running = 0;

        /* Frame cap */
        int64_t elapsed = clock_ns() - last_frame + dt;
        clock_sleep_ns(RENDER_NS - elapsed);
    }

    endwin();
    return 0;
}
