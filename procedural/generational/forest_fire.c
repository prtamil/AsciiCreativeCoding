/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * forest_fire.c — the classic forest-fire cellular automaton, live in the terminal.
 *
 * Every grid cell is empty, a tree, or on fire. Each tick all cells update at once
 * by four simple rules: fire burns out, fire spreads to neighbouring trees,
 * lightning randomly lights a tree, and empty ground randomly grows a tree. Two
 * dials — how fast trees grow (p) and how often lightning strikes (f) — and the
 * forest settles by itself onto a state where fire sizes follow a power law: the
 * famous self-organised criticality, with no tuning needed.
 *
 * Reference: Drossel & Schwabl, "Self-organized critical forest-fire model,"
 *            Phys. Rev. Lett. 69, 1629 (1992) — this file's exact rules.
 *            Background: Bak, "How Nature Works" (Copernicus, 1996).
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define ROWS_MAX   128
#define COLS_MAX   512

/* The three things a cell can be */
#define EMPTY  0
#define TREE   1
#define FIRE   2

/* How much one keypress nudges p and f, and how far they're allowed to go.
 * The starting values live in the presets below, not here. */
#define P_GROW_STEP  0.005f
#define P_FIRE_STEP  0.0001f
#define P_GROW_MIN   0.001f
#define P_GROW_MAX   0.200f
#define P_FIRE_MIN   0.00001f
#define P_FIRE_MAX   0.010f

/* How many simulation ticks per second, and the range the +/- keys allow */
#define SIM_FPS_DEF   20
#define SIM_FPS_MIN    2
#define SIM_FPS_MAX   60
#define SIM_FPS_STEP   2

#define N_PRESETS  4
#define N_THEMES   5

#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define FLASH_NS    (NS_PER_SEC * 9 / 10)   /* HUD action-flash lifetime (~0.9 s)        */
#define SLEEP_MARGIN_NS  1000000LL           /* wake ~1 ms early so we never oversleep a tick */

/*
 * Names for ncurses colour pairs. The first four take their colour from the
 * current theme; the two HUD pairs are always the same bright yellow / cyan.
 * Empty cells get no pair at all — we draw a blank space so the terminal's own
 * background shows through, which looks cleaner than tinting the bare ground.
 */
enum {
    CP_TREE   = 1,
    CP_FIRE1  = 2,   /* dim fire   */
    CP_FIRE2  = 3,   /* bright fire */
    CP_ASH    = 4,   /* the '.' left for one tick where fire just died */
    PAIR_HUD  = 8,   /* bright yellow — top status bar */
    PAIR_HINT = 9,   /* bright cyan   — bottom key hints */
};

/*
 * Theme — one named colour palette for the forest. There are two fire colours so
 * neighbouring burning cells can flip between bright and dim, making a fire patch
 * look like it's crackling. Empty cells aren't listed because they're just blank.
 *
 * Every colour is kept in the brighter half of the 256-colour range — the very
 * dark indices come out near-black in the terminal, so we avoid them. Each theme
 * also keeps a plain 8-colour version for older terminals (theme_apply picks).
 */
typedef struct {
    short tree;          /* the '^' tree colour */
    short fire1, fire2;  /* the dim ',' and bright '*' fire colours */
    short ash;           /* the '.' just-burned-ground colour */
    short tree8, fire18, fire28, ash8;   /* same four, for 8-colour terminals */
    const char *name;    /* what the HUD calls this theme */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0  Classic — green forest, orange-red fire */
    {  34, 202, 196, 244,
       COLOR_GREEN, COLOR_YELLOW, COLOR_RED, COLOR_WHITE,
       "Classic" },
    /* 1  Night   — emerald forest, yellow-white fire */
    {  35, 214, 226, 247,
       COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE,
       "Night" },
    /* 2  Autumn  — amber trees, deep red fire */
    { 130, 196, 160, 245,
       COLOR_YELLOW, COLOR_RED, COLOR_RED, COLOR_WHITE,
       "Autumn" },
    /* 3  Boreal  — teal trees, yellow-white fire */
    {  37, 220, 231, 250,
       COLOR_CYAN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE,
       "Boreal" },
    /* 4  Lava    — green trees, magenta-white fire */
    {  29, 201, 207, 248,
       COLOR_GREEN, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE,
       "Lava" },
};

/*
 * Preset — one ready-made set of dials, so a single keypress jumps the forest to
 * a different character. The two dials are how fast trees grow (p) and how often
 * lightning strikes (f); it's mostly their ratio that decides how big fires get.
 * Each preset is just a different (p, f) plus a couple of extra choices.
 */
typedef struct {
    float p_grow;          /* p: chance an empty cell grows a tree each tick (0..1) */
    float p_fire;          /* f: chance a tree is struck by lightning each tick (0..1) */
    bool  eight_neighbor;  /* true = fire can also jump diagonally, not just up/down/left/right */
    float density;         /* fraction of the grid seeded with trees at the start (0..1) */
    const char *name;      /* what the HUD calls this preset */
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { 0.030f, 0.0002f, false, 0.60f, "Classic" },
    { 0.060f, 0.0001f, false, 0.70f, "Dense"   },
    { 0.010f, 0.0010f, false, 0.40f, "Sparse"  },
    { 0.020f, 0.0003f, true,  0.55f, "Smoulder"},
};

/* ── §2 perf / delays — reading the clock and sleeping ── */

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

/* ── §3 state — the data the whole program shares ── */

/*
 * Forest — the grid the simulation runs on, plus a few tallies about it.
 *
 * Why two grids: every cell has to update based on what its neighbours look like
 * RIGHT NOW, all at the same instant. If we overwrote one grid as we walked it,
 * a fire near the top-left would already be gone by the time we checked the cell
 * next to it, and fires would smear down and to the right. So we read from `grid`,
 * write the new state into `next`, then copy `next` back over `grid` at the end.
 */
typedef struct {
    /* the board itself */
    uint8_t grid[ROWS_MAX][COLS_MAX];   /* what every cell is right now */
    uint8_t next[ROWS_MAX][COLS_MAX];   /* what every cell becomes; copied into grid */
    int     rows, cols;                 /* the part actually in use, sized to fit
                                           the terminal under the HUD bars */

    /* a marker for cells that burned this very tick, drawn as '.' for one frame.
     * Set when a fire dies, wiped at the start of the next tick. Purely for looks
     * — the simulation never reads it back. */
    uint8_t ash[ROWS_MAX][COLS_MAX];

    /* recounted every tick so the HUD can show them */
    int     n_tree, n_fire, n_empty;    /* how many cells are each kind */
    long    tick;                       /* ticks so far; also flips the fire flicker */
} Forest;

/*
 * Scene — everything the program is doing, gathered in one struct: the forest, the
 * dials the user is turning, whether we're paused, and which theme is showing. The
 * p_grow/p_fire/eight_neighbor here are a live copy of the chosen preset's values,
 * so the user can tweak them with keys without editing the preset table — and the
 * simulation always reads these copies, not the table.
 */
typedef struct {
    Forest forest;          /* the grid being simulated */

    /* the live dials the user turns; start as a copy of the current preset */
    float  p_grow;          /* tree growth chance        (g / G keys) */
    float  p_fire;          /* lightning chance          (l / L keys) */
    bool   eight_neighbor;  /* can fire jump diagonally? */
    int    preset;          /* which preset is loaded    (n / N keys) */
    int    sim_fps;         /* ticks per second          (+ / - keys) */

    bool   paused;          /* when true, the forest stops updating */
    int    theme;           /* which colour theme        (t / T keys) */
} Scene;

static Scene g_scene = {
    .preset  = 0,
    .sim_fps = SIM_FPS_DEF,
    .theme   = 0,
};

/* the random-number generator's running state (the simulation owns it) */
static uint32_t g_rng = 12345;

/* does this terminal support 256 colours? plus the brief HUD message and when it expires */
static bool    g_has_256;
static char    g_flash[48];
static int64_t g_flash_until = 0;

/* set by the signal handlers, read by the main loop */
static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

/* ── §4 logic — questions we ask about the grid, no changes ── */

/* True if any of the four neighbours listed in dr/dc is on fire (and on the grid). */
static bool any_neighbour_on_fire(const Forest *f, int r, int c,
                                  const int *dr, const int *dc)
{
    for (int d = 0; d < 4; d++) {
        int nr = r + dr[d], nc = c + dc[d];
        if (nr >= 0 && nr < f->rows && nc >= 0 && nc < f->cols
            && f->grid[nr][nc] == FIRE) return true;
    }
    return false;
}

/*
 * has_fire_neighbor — is there a fire touching this cell? The four straight
 * neighbours (up, down, left, right) always count; the four diagonals count only
 * when the preset allows diagonal spread. The grid doesn't wrap around, so cells
 * on the edge just have fewer neighbours to check.
 */
static bool has_fire_neighbor(const Forest *f, int r, int c, bool eight)
{
    static const int orth_dr[4] = {-1, 1, 0, 0}, orth_dc[4] = { 0, 0,-1, 1};
    static const int diag_dr[4] = {-1,-1, 1, 1}, diag_dc[4] = {-1, 1,-1, 1};

    if (any_neighbour_on_fire(f, r, c, orth_dr, orth_dc)) return true;
    if (!eight) return false;
    return any_neighbour_on_fire(f, r, c, diag_dr, diag_dc);
}

/* ── §5 simulation — the only place the forest actually changes ── */

/* A tiny, fast random-number generator (xorshift). The inner loop calls it once
 * per cell, so we want something quicker and simpler than the standard rand(). */
static inline uint32_t rng_next(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}
/* A random number from 0 up to (but not including) 1 — handy for "x% chance" tests. */
static inline float rng_float(void)
{
    return (float)(rng_next() >> 8) / (float)(1 << 24);
}

static void forest_seed(Forest *f, float density)
{
    for (int r = 0; r < f->rows; r++)
        for (int c = 0; c < f->cols; c++) {
            f->grid[r][c] = (rng_float() < density) ? TREE : EMPTY;
            f->ash[r][c]  = 0;
        }
}

/* Switch to a preset: copy its dials into the live scene and scatter a fresh forest. */
static void scene_init(Scene *s, int preset)
{
    s->preset         = preset;
    s->p_grow         = k_presets[preset].p_grow;
    s->p_fire         = k_presets[preset].p_fire;
    s->eight_neighbor = k_presets[preset].eight_neighbor;
    s->forest.tick    = 0;
    forest_seed(&s->forest, k_presets[preset].density);
}

/*
 * forest_step — advance the whole forest by one tick. This is the heart of the
 * simulation: it works out every cell's new state from the current board, then
 * swaps the new board in all at once so the spread looks even in all directions.
 */
static void forest_step(Forest *f, float p_grow, float p_fire, bool eight)
{
    int nt = 0, nf = 0, ne = 0;
    memset(f->ash, 0, sizeof f->ash);   /* forget last tick's burn marks */

    /* the four rules, applied to every cell from the same starting board */
    for (int r = 0; r < f->rows; r++) {
        for (int c = 0; c < f->cols; c++) {
            uint8_t state = f->grid[r][c];
            uint8_t next  = state;

            if (state == FIRE) {                 /* fire lasts one tick, then it's gone */
                next = EMPTY; f->ash[r][c] = 1; ne++;
            } else if (state == TREE) {          /* a tree catches from a neighbour or from lightning */
                bool ignite = has_fire_neighbor(f, r, c, eight)
                           || rng_float() < p_fire;
                next = ignite ? FIRE : TREE;
                if (ignite) nf++; else nt++;
            } else {                             /* empty ground may sprout a tree */
                bool grow = rng_float() < p_grow;
                next = grow ? TREE : EMPTY;
                if (grow) nt++; else ne++;
            }
            f->next[r][c] = next;
        }
    }

    memcpy(f->grid, f->next, (size_t)f->rows * COLS_MAX);   /* make the new board the current one */
    f->n_tree  = nt;
    f->n_fire  = nf;
    f->n_empty = ne;
    f->tick++;
}

/* ── §6 effects — purely visual extras, never fed back into the simulation ── */

/*
 * hud_flash — flash a short message in the top-left corner to confirm a keypress.
 * Turning the grow/lightning dials only changes the odds, so the forest reacts
 * slowly; this gives the key an instant, visible "got it" the user can't miss.
 */
static void hud_flash(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_flash, sizeof g_flash, fmt, ap);
    va_end(ap);
    g_flash_until = clock_ns() + FLASH_NS;
}

/* ── §7 render — turning the state into pixels; never changes the state ── */

/*
 * theme_apply — hand the chosen theme's colours to ncurses. Every pair uses -1 for
 * its background so the terminal's own background shows behind the forest. It only
 * touches the four forest pairs, not the HUD pairs, so flipping themes leaves the
 * status and hint bars looking the same.
 */
static void theme_apply(int ti)
{
    const Theme *th = &k_themes[ti];
    if (g_has_256) {
        init_pair(CP_TREE,   th->tree,   -1);
        init_pair(CP_FIRE1,  th->fire1,  -1);
        init_pair(CP_FIRE2,  th->fire2,  -1);
        init_pair(CP_ASH,    th->ash,    -1);
    } else {
        init_pair(CP_TREE,   th->tree8,   -1);
        init_pair(CP_FIRE1,  th->fire18,  -1);
        init_pair(CP_FIRE2,  th->fire28,  -1);
        init_pair(CP_ASH,    th->ash8,    -1);
    }
}

/*
 * mark_cell — draw one character at a screen spot in a given colour. It's the one
 * place we do the slightly fiddly cast ncurses needs (without it, characters above
 * 127 can come out wrong) and the off-screen check, so the callers stay clean.
 * Anything outside the screen is quietly skipped.
 */
static void mark_cell(int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(cy, cx, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * draw_forest_cell — draw one grid cell. Everything is shifted down one row so the
 * top row is free for the HUD. Fire flickers: whether it's drawn bright or dim
 * depends on its position and the tick count, so a burning patch shimmers.
 */
static void draw_forest_cell(const Forest *f, int r, int c, int cols, int rows)
{
    uint8_t state = f->grid[r][c];

    if (state == EMPTY) {
        if (f->ash[r][c])
            mark_cell(c, r + 1, '.', CP_ASH, A_DIM, cols, rows);   /* just-burned ground */
        /* otherwise draw nothing — the terminal background shows through */
    } else if (state == TREE) {
        mark_cell(c, r + 1, '^', CP_TREE, A_BOLD, cols, rows);
    } else { /* FIRE */
        bool bright = ((r + c + (int)f->tick) & 1);   /* alternate bright/dim like a checkerboard */
        if (bright)
            mark_cell(c, r + 1, '*', CP_FIRE2, A_BOLD, cols, rows);
        else
            mark_cell(c, r + 1, ',', CP_FIRE1, A_NORMAL, cols, rows);
    }
}

static void draw_forest(const Forest *f, int cols, int rows)
{
    /* leave the top and bottom rows for the HUD bars */
    int draw_rows = (f->rows < rows - 2) ? f->rows : rows - 2;
    int draw_cols = (f->cols < cols)     ? f->cols : cols;

    for (int r = 0; r < draw_rows; r++)
        for (int c = 0; c < draw_cols; c++)
            draw_forest_cell(f, r, c, cols, rows);
}

/*
 * draw_status — the status bar along the top-right: preset, theme, current tree
 * and fire percentages, the two dials, speed, tick count, and whether we're paused.
 */
static void draw_status(const Scene *s, int cols)
{
    const Forest *f = &s->forest;
    int total = f->n_tree + f->n_fire + f->n_empty;
    float tree_pct = total > 0 ? 100.0f * (float)f->n_tree / (float)total : 0;
    float fire_pct = total > 0 ? 100.0f * (float)f->n_fire / (float)total : 0;

    char buf[160];
    snprintf(buf, sizeof buf,
        " [%d/%d %s] %s  trees:%5.1f%% fire:%4.1f%%  p=%.4f f=%.5f"
        "  sim:%dHz tick:%ld  %s ",
        s->preset + 1, N_PRESETS, k_presets[s->preset].name, k_themes[s->theme].name,
        tree_pct, fire_pct, s->p_grow, s->p_fire,
        s->sim_fps, f->tick,
        s->paused ? "PAUSED " : "running");
    int hx = cols - (int)strlen(buf);   /* push it to the right edge */
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* draw_action_flash — show the brief keypress message in the top-left, in reverse
 * video so it stands out against the normal status bar. */
static void draw_action_flash(void)
{
    if (clock_ns() < g_flash_until) {
        attron(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
        mvprintw(0, 1, " %s ", g_flash);
        attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
    }
}

/* draw_keyhint — the list of keys along the bottom row. Drawn bold and bright so
 * it stays readable even with fire flickering behind it. */
static void draw_keyhint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
        " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme"
        "  g/G:grow  l/L:lightning  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void draw_hud(const Scene *s, int cols, int rows)
{
    draw_status(s, cols);
    draw_action_flash();
    draw_keyhint(rows);
}

/* scene_draw — paint one whole frame: clear the screen, draw the forest, then put
 * the HUD bars on top. */
static void scene_draw(const Scene *s)
{
    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    draw_forest(&s->forest, cols, rows);
    draw_hud   (s, cols, rows);
}

/* ── §8 platform / app — ncurses setup, signals, resize, input, main loop ── */

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);

    start_color();
    use_default_colors();         /* lets us use -1 for "the terminal's own background" */
    g_has_256 = (COLORS >= 256);

    /* The HUD colours are set once here and never touched by theme_apply, so they
     * stay the same bright yellow/cyan no matter which theme is showing. */
    if (g_has_256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }

    theme_apply(g_scene.theme);
}

/* Make the forest match the terminal size, leaving 2 rows for the HUD bars. */
static void forest_fit_terminal(Forest *f)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    f->rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
    f->cols = (cols     < COLS_MAX) ? cols      : COLS_MAX;
}

/* After the window is resized, find out the new size and refit the forest. */
static void screen_resize(Forest *f)
{
    endwin();
    refresh();
    forest_fit_terminal(f);
    g_resize = 0;
}

/* Hook up the quit and resize signals so they just set a flag for the main loop. */
static void install_signals(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);
}

/* Seed the generator from the current time (mixed with a constant so it's never 0). */
static void seed_rng(void)
{
    g_rng = (uint32_t)time(NULL) ^ 0xDEADBEEFu;
}

/*
 * handle_key — do whatever one keypress asks: quit, pause, reset, switch preset or
 * theme, nudge a dial (with a flash to confirm), or change the speed.
 */
static void handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 27: g_quit = 1; break;

    case 'p': case ' ':
        s->paused = !s->paused;
        break;

    case 'r':
        scene_init(s, s->preset);
        break;

    case 'n':
        scene_init(s, (s->preset + 1) % N_PRESETS);
        break;
    case 'N':
        scene_init(s, (s->preset + N_PRESETS - 1) % N_PRESETS);
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme);
        break;

    case 'g':
        if (s->p_grow < P_GROW_MAX) s->p_grow += P_GROW_STEP;
        hud_flash("grow  p=%.4f  +", s->p_grow);
        break;
    case 'G':
        if (s->p_grow > P_GROW_MIN) s->p_grow -= P_GROW_STEP;
        hud_flash("grow  p=%.4f  -", s->p_grow);
        break;

    case 'l':
        if (s->p_fire < P_FIRE_MAX) s->p_fire += P_FIRE_STEP;
        hud_flash("lightning  f=%.5f  +", s->p_fire);
        break;
    case 'L':
        if (s->p_fire > P_FIRE_MIN) s->p_fire -= P_FIRE_STEP;
        hud_flash("lightning  f=%.5f  -", s->p_fire);
        break;

    case '+': case '=':
        if (s->sim_fps < SIM_FPS_MAX) s->sim_fps += SIM_FPS_STEP;
        break;
    case '-':
        if (s->sim_fps > SIM_FPS_MIN) s->sim_fps -= SIM_FPS_STEP;
        break;
    }
}

int main(void)
{
    install_signals();
    seed_rng();
    screen_init();
    forest_fit_terminal(&g_scene.forest);
    scene_init(&g_scene, g_scene.preset);

    int64_t next_tick = clock_ns();

    while (!g_quit) {
        Scene *s = &g_scene;

        /* handle every key waiting in the buffer */
        int ch;
        while ((ch = getch()) != ERR)
            handle_key(s, ch);

        if (g_resize) {
            screen_resize(&s->forest);
            scene_init(s, s->preset);
        }

        /* step the forest once each time its tick is due, unless paused */
        int64_t now = clock_ns();
        if (!s->paused && now >= next_tick) {
            forest_step(&s->forest, s->p_grow, s->p_fire, s->eight_neighbor);
            next_tick = now + TICK_NS(s->sim_fps);
        }

        /* draw the frame */
        scene_draw(s);
        wnoutrefresh(stdscr);
        doupdate();

        /* wait until it's almost time for the next tick */
        clock_sleep_ns(next_tick - clock_ns() - SLEEP_MARGIN_NS);
    }

    endwin();
    return 0;
}
