/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * excitable.c — Greenberg-Hastings excitable medium.
 *
 * A grid of cells where "excitement" spreads like a wave and then has to
 * cool down before it can fire again — the same idea behind ripples in
 * heart and nerve tissue. Broken wave fronts curl up into rotating spirals.
 * Each cell cycles 0 (resting) -> 1 (excited) -> 2..N-1 (cooling down) -> 0.
 *
 * Founding paper: Greenberg & Hastings, SIAM J. Appl. Math. 34(3), 515 (1978).
 * Spiral-wave background: Winfree, "The Geometry of Biological Time" (2001).
 *
 * Keys: q quit  p pause  r reset  1-4 preset  +/- N  t/T theme  spc pulse
 */

/* ── §0  section map ──────────────────────────────────────────────────────
 *   §1 config  §2 perf/delays  §3 state  §4 logic  §5 simulation
 *   §6 effects §7 render        §8 platform/app
 *
 * How the cells live:
 *   - Resting cells wake up the moment a neighbour is excited.
 *   - An excited cell wakes its four up/down/left/right neighbours, then starts
 *     cooling down. While cooling it ignores everything around it — that pause
 *     is what stops a wave from immediately re-lighting the cells behind it, so
 *     waves travel as clean fronts instead of a chaotic spread.
 *   - N is how many states there are. More states = longer cooldown = slower,
 *     more spaced-out waves. Fewer states = faster, denser waves.
 *
 * One key trick: every cell updates from the SAME snapshot, so we keep two full
 * grids and swap them each step (read one, write the other). The simulation
 * advances at half the display rate so the motion looks smooth, not jumpy.
 * ──────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config ─────────────────────────────────────────────────────────── */

#define GRID_H_MAX  120
#define GRID_W_MAX  360

/* N = number of states a cell cycles through. Fewer -> faster denser waves;
 * more -> slower, more spaced-out waves. Cooldown length is N-2 steps. */
#define N_MIN         5
#define N_MAX        20
#define N_DEF        12

#define RENDER_NS  (1000000000LL / 30)   /* draw 30 frames a second              */
#define STEP_NS    (1000000000LL / 15)   /* advance the sim 15 times a second —
                                          * half the frame rate, so motion reads
                                          * smoothly instead of jumping            */
#define HUD_ROWS    2    /* rows the grid gives up: status on top, keys on bottom  */
#define HUD_TOP     1    /* status sits on row 0; the grid starts at row 1         */
#define N_THEMES    5

#define PULSE_HALF_H  1            /* spc pulse: half-height, so a 3-row block      */
#define PULSE_HALF_W  3            /* spc pulse: half-width, so a 7-col block        */
#define DT_MAX_NS     100000000LL  /* never let one frame count more than 100 ms of
                                    * time — otherwise a hiccup makes the sim try to
                                    * catch up with a flood of steps and freezes     */

/* One ncurses colour slot per thing we draw: HUD, then one per cell state,
 * then the key-hint line. */
enum { CP_HUD = 1, CP_S0 = 2, CP_HINT = CP_S0 + N_MAX };

/* ── §2  perf / delays  —  read the clock, sleep between frames ─────────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3  state types  —  Medium + Scene (the one g_scene) + platform globals ── */

/*
 * Medium — the sheet of cells we're simulating. Each cell holds a number 0..n-1:
 * 0 is resting, 1 is excited (firing), and 2..n-1 are the cooldown phases
 * counting back down to rest. The edges wrap around, so the sheet behaves like
 * the surface of a doughnut and waves never hit a wall. This is the classic
 * Greenberg-Hastings model (SIAM J. Appl. Math. 34, 515, 1978), a simplified
 * stand-in for a patch of heart or nerve tissue (Wiener & Rosenblueth 1946).
 *
 * Why two grids: every cell has to update from the same shared snapshot. If a
 * cell wrote its new value into the live grid mid-sweep, its neighbours would
 * read a half-updated picture and the whole thing falls apart. So we keep two
 * full copies and just swap which one is "current" after each step.
 */
typedef struct {
    /* the two copies; the pointers below decide which is current right now */
    uint8_t bufA[GRID_H_MAX][GRID_W_MAX];
    uint8_t bufB[GRID_H_MAX][GRID_W_MAX];
    uint8_t (*cur)[GRID_W_MAX];   /* the copy we read this step                  */
    uint8_t (*nxt)[GRID_W_MAX];   /* the copy we write, then swap in as current  */

    int gh, gw;   /* how many rows/cols are actually in use — sized to the
                     terminal so the grid fills the screen below the HUD         */
    int n;        /* how many states a cell cycles through (N_MIN..N_MAX). Lives
                     here because cell values are taken mod n. Also sets the
                     pace: excitement lasts 1 step, cooldown lasts n-2 steps, so
                     small n means fast dense waves, large n means slow sparse
                     ones. The user nudges it with +/-.                          */
} Medium;

/*
 * Scene — everything the program is doing, in one place: the cells themselves
 * plus the few settings the user can change. preset/paused/theme aren't big
 * enough ideas to deserve their own structs, so they just sit here, grouped by
 * whether they affect the simulation or only how it's drawn.
 */
typedef struct {
    Medium medium;     /* the cells being simulated                             */

    int    preset;     /* which starting pattern to seed — 1 Spiral, 2 Double,
                          3 Rings, 4 Chaos. Remembered (not just applied once)
                          so a resize can re-seed the same pattern and the HUD
                          can name it. Spiral/Double work because a wave front
                          with an open end curls up into a rotating spiral.      */
    bool   paused;     /* if true, stop advancing the sim but keep drawing, so
                          the frozen picture stays on screen                     */
    int    theme;      /* colour palette 0..N_THEMES-1. Purely a look — kept
                          apart from the sim settings because it changes how
                          cells are coloured, never how they behave.             */
} Scene;

static Scene g_scene = {
    .medium = { .n = N_DEF },
    .preset = 1,
    .theme  = 0,
};

/* terminal geometry (PLATFORM §8 writes; RENDER §7 reads) */
static int g_rows, g_cols;

/* platform signal flags */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ── §4  logic  —  pure decisions, no writes ───────────────────────────── */

/* Wrap an index back inside the grid so stepping off one edge lands on the
 * opposite edge (the doughnut-surface trick). */
static int wrap_index(int i, int len)
{
    return (i + len) % len;
}

/*
 * any_neighbour_excited — is one of this cell's four neighbours (up, down,
 * left, right) currently firing? That's the only thing a resting cell needs
 * to know to decide whether to wake up. Edges wrap around.
 */
static bool any_neighbour_excited(const Medium *m, int r, int c)
{
    int ru = wrap_index(r - 1, m->gh), rd = wrap_index(r + 1, m->gh);
    int cl = wrap_index(c - 1, m->gw), cr = wrap_index(c + 1, m->gw);
    return m->cur[ru][c] == 1 || m->cur[rd][c] == 1
        || m->cur[r][cl] == 1 || m->cur[r][cr] == 1;
}

/* ── §5  simulation  —  the only place cells actually change ────────────── */

/* Wipe both grids to all-resting and pick a starting current/next. */
static void medium_clear(Medium *m)
{
    memset(m->bufA, 0, sizeof m->bufA);
    memset(m->bufB, 0, sizeof m->bufB);
    m->cur = m->bufA;
    m->nxt = m->bufB;
}

/*
 * medium_step — advance every cell once. A resting cell fires if a neighbour is
 * firing; any non-resting cell just ticks one step closer to rest. We write the
 * fresh values into the spare grid, then swap so it becomes current.
 */
static void medium_step(Medium *m)
{
    int n = m->n;
    for (int r = 0; r < m->gh; r++) {
        for (int c = 0; c < m->gw; c++) {
            uint8_t s = m->cur[r][c];
            if (s == 0)
                m->nxt[r][c] = any_neighbour_excited(m, r, c) ? 1 : 0;
            else
                m->nxt[r][c] = (uint8_t)((s + 1) % n);   /* one step toward rest */
        }
    }
    /* the grid we just filled in becomes the one we read next time */
    uint8_t (*tmp)[GRID_W_MAX] = m->cur;
    m->cur = m->nxt;
    m->nxt = tmp;
}

/*
 * lay_wavefront — draw a wave front with one open end. Along `row`, columns
 * [c0,c1) are set firing, with a trail of cooling cells stacked off to one side
 * (wake_dir = -1 above, +1 below). The trail gives the wave a direction to move;
 * the open end with nothing behind it is what curls into a spiral.
 */
static void lay_wavefront(Medium *m, int row, int c0, int c1, int wake_dir)
{
    for (int c = c0; c < c1; c++) {
        m->cur[row][c] = 1;
        for (int dr = 1; dr < m->n - 1; dr++) {
            int rr = row + wake_dir * dr;
            if (rr < 0 || rr >= m->gh) break;
            m->cur[rr][c] = (uint8_t)(dr + 1);
        }
    }
}

/* Spiral — one front across the left half of the middle row. Its open right end
 * has nothing trailing it, so it curls into a single rotating spiral. */
static void seed_spiral(Medium *m)
{
    int mr = m->gh / 2, mc = m->gw / 2;
    lay_wavefront(m, mr, 0, mc, -1);
}

/* Double — two fronts offset top and bottom with trails facing opposite ways,
 * so they curl into a pair of spirals spinning opposite directions that keep
 * going forever instead of cancelling each other out. */
static void seed_double(Medium *m)
{
    int mr1 = m->gh / 3, mr2 = m->gh * 2 / 3, mc = m->gw / 2;
    lay_wavefront(m, mr1, 0,  mc,     -1);
    lay_wavefront(m, mr2, mc, m->gw,  +1);
}

/* Rings — set each cell's phase from its distance to the centre, so cells at the
 * same radius share a phase and form concentric rings, like a target. Rows are
 * doubled before measuring distance because terminal characters are about twice
 * as tall as wide, which keeps the rings looking round instead of squashed. */
static void seed_rings(Medium *m)
{
    int gh = m->gh, gw = m->gw, n = m->n;
    int cr = gh / 2, cc = gw / 2;
    for (int r = 0; r < gh; r++)
        for (int c = 0; c < gw; c++) {
            float dr   = (float)(r - cr) * 2.0f;  /* squish rows to match aspect */
            float dc   = (float)(c - cc);
            int   dist = (int)sqrtf(dr * dr + dc * dc);
            int   s    = (n - 1) - (dist % n);
            m->cur[r][c] = (uint8_t)(s >= 0 ? s : 0);
        }
}

/* Chaos — give every cell a random phase. The random jumble is full of open
 * front ends, so spirals keep popping up all over and the field stays in
 * permanent churn. (Seeding a few lone firing dots instead just makes closed
 * rings that collide and vanish within a couple of seconds — no open ends to
 * curl, so nothing keeps itself alive.) */
static void seed_chaos(Medium *m)
{
    int n = m->n;
    for (int r = 0; r < m->gh; r++)
        for (int c = 0; c < m->gw; c++)
            m->cur[r][c] = (uint8_t)(rand() % n);
}

/* Wipe the grid and lay down whichever starting pattern was chosen. */
static void medium_seed(Medium *m, int preset)
{
    medium_clear(m);
    if      (preset == 1) seed_spiral(m);
    else if (preset == 2) seed_double(m);
    else if (preset == 3) seed_rings(m);
    else                  seed_chaos(m);
}

/* Light up a small block of cells in the centre — the spacebar "poke". */
static void medium_pulse(Medium *m)
{
    int cr = m->gh / 2, cc = m->gw / 2;
    for (int dr = -PULSE_HALF_H; dr <= PULSE_HALF_H; dr++)
        for (int dc = -PULSE_HALF_W; dc <= PULSE_HALF_W; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r >= 0 && r < m->gh && c >= 0 && c < m->gw)
                m->cur[r][c] = 1;
        }
}

/* ── §6  effects ──────────────────────────────────────────────────────────
 * Nothing here on purpose. The fade you see from a bright just-fired cell to a
 * dim trailing one isn't a separate visual effect — it's the cooldown states
 * themselves, drawn through the character ramp below. No extra data to store.
 * ──────────────────────────────────────────────────────────────────────── */

/* ── §7  render  —  cells to screen (reads only) + palette setup ────────── */

/* Each theme is a 12-colour gradient, brightest first (index 0 = firing) to
 * darkest last (index 11 = resting). The cooldown states fade through the
 * middle. */
static const char *k_theme_names[N_THEMES] = {
    "Fire", "Ice", "Matrix", "Plasma", "Mono"
};

static const short k_ramps256[N_THEMES][12] = {
    /* Fire:   white → yellow → orange → red → dark */
    { 231, 230, 226, 220, 214, 208, 202, 196, 160,  88,  52,  16 },
    /* Ice:    white → cyan → blue → dark navy */
    { 231, 195, 159, 123,  87,  51,  45,  39,  33,  27,  21,  17 },
    /* Matrix: white → bright-green → green → black */
    { 231,  82,  46,  40,  34,  28,  22,  22,  16,  16,  16,  16 },
    /* Plasma: white → magenta → purple → dark */
    { 231, 213, 207, 201, 165, 129,  93,  57,  54,  53,  17,  16 },
    /* Mono:   white → grey gradient → black */
    { 231, 252, 248, 244, 240, 236, 234, 233, 232, 232,  16,  16 },
};

/* Fallback for terminals with only 8 colours: bright / mid / dim per theme. */
static const short k_ramps8[N_THEMES][3] = {
    { COLOR_WHITE, COLOR_YELLOW,  COLOR_RED     },
    { COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE    },
    { COLOR_WHITE, COLOR_GREEN,   COLOR_GREEN   },
    { COLOR_WHITE, COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE, COLOR_WHITE,   COLOR_BLACK   },
};

/* Rebuild the colour for every cell state. Call whenever the theme or N changes. */
static void theme_apply(int theme, int n)
{
    init_pair(CP_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);

    for (int s = 0; s < N_MAX; s++) {
        int ri;
        if      (s == 0) ri = 11;   /* resting: darkest end of the gradient  */
        else if (s == 1) ri = 0;    /* firing: brightest end                 */
        else {
            ri = 1 + (s - 2) * 9 / (n > 2 ? n - 2 : 1);   /* cooldown: fade between */
            if (ri > 10) ri = 10;
        }
        short fg = (COLORS >= 256) ? k_ramps256[theme][ri]
                 : (s == 0) ? k_ramps8[theme][2]
                 : (s == 1) ? k_ramps8[theme][0]
                 :             k_ramps8[theme][1];
        init_pair((short)(CP_S0 + s), fg, -1);
    }
}

static void color_init(int theme, int n)
{
    start_color();
    use_default_colors();
    theme_apply(theme, n);
}

/* Glyphs for the cooling-down trail, densest first (just fired) to faintest
 * (about to rest); draw_medium_cell picks one by how far along cooldown a cell is. */
static const char k_rchar[] = "@*+=-.'";

/* Print one HUD line, bold, cut off at the screen edge so it never wraps. */
static void hud_line(int row, int pair, const char *s)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddnstr(row, 0, s, g_cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* The preset's name for the status line. */
static const char *preset_name(int preset)
{
    return preset == 1 ? "Spiral" : preset == 2 ? "Double"
         : preset == 3 ? "Rings"  : "Chaos";
}

/*
 * draw_medium_cell — draw one cell that isn't resting. A firing cell is a bold
 * '#'; a cooling cell picks a fainter glyph the further along cooldown it is.
 * Its colour comes straight from the cell's state.
 */
static void draw_medium_cell(int s, int n, int sy, int sx)
{
    chtype ch;
    attr_t at;
    if (s == 1) {
        ch = '#'; at = A_BOLD;
    } else {
        int ramp = (s - 2) * 7 / (n > 2 ? n - 2 : 1);   /* how far into cooldown -> glyph */
        if (ramp >= 7) ramp = 6;
        ch = (chtype)(unsigned char)k_rchar[ramp];
        at = (s == 2) ? A_BOLD : A_NORMAL;
    }
    attron(COLOR_PAIR(CP_S0 + s) | at);
    mvaddch(sy, sx, ch);
    attroff(COLOR_PAIR(CP_S0 + s) | at);
}

/* Draw the whole grid into the screen area between the two HUD rows. */
static void draw_medium(const Medium *m)
{
    for (int r = 0; r < m->gh && r + HUD_TOP < g_rows - 1; r++) {
        for (int c = 0; c < m->gw && c < g_cols; c++) {
            int s = m->cur[r][c];
            if (s == 0) continue;   /* resting cells stay blank */
            draw_medium_cell(s, m->n, r + HUD_TOP, c);
        }
    }
}

/* Top row shows the live settings; bottom row lists the keys. */
static void draw_hud(const Scene *sc)
{
    char buf[160];
    snprintf(buf, sizeof buf, " N=%d  preset=%s  theme=%s  %s ",
        sc->medium.n, preset_name(sc->preset),
        k_theme_names[sc->theme], sc->paused ? "PAUSED" : "running");
    hud_line(0, CP_HUD, buf);

    hud_line(g_rows - 1, CP_HINT,
        " q:quit  p:pause  r:reset  1-4:preset  +/-:N  t/T:theme  spc:pulse ");
}

static void scene_draw(const Scene *sc)
{
    draw_medium(&sc->medium);
    draw_hud(sc);
}

/* ── §8  platform / app  —  ncurses, signals, resize, main loop ────────── */

/* Signals can fire at any moment, so they just set a flag the loop checks. */
static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

/* Make sure the terminal gets restored on exit, and catch quit/resize signals. */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);
}

/* Put the terminal into raw, no-echo, hidden-cursor, non-blocking mode. */
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
}

static void resize_and_reset(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    s->medium.gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
    s->medium.gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
    medium_seed(&s->medium, s->preset);
}

/*
 * handle_key — act on one keypress. Returns true if it laid down a fresh grid
 * (reset or preset change) so the caller knows to restart the step clock.
 */
static bool handle_key(Scene *s, int ch)
{
    Medium *m = &s->medium;
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1;            break;
    case 'p': case 'P': s->paused = !s->paused;         break;

    case 'r': case 'R':
        medium_seed(m, s->preset);
        return true;
    case '1': case '2': case '3': case '4':
        s->preset = ch - '0';
        medium_seed(m, s->preset);
        return true;

    case ' ': medium_pulse(m);                          break;

    case '+': case '=':
        if (m->n < N_MAX) {                  /* adding states leaves existing cells valid */
            m->n++;
            theme_apply(s->theme, m->n);
        }
        break;
    case '-':
        if (m->n > N_MIN) {
            m->n--;
            theme_apply(s->theme, m->n);
            for (int r = 0; r < m->gh; r++)  /* any cell now past the new max -> send to rest */
                for (int c = 0; c < m->gw; c++)
                    if (m->cur[r][c] >= (uint8_t)m->n)
                        m->cur[r][c] = 0;
        }
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme, m->n);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme, m->n);
        break;

    default: break;
    }
    return false;
}

int main(void)
{
    srand((unsigned)time(NULL));
    install_signals();
    screen_init();
    color_init(g_scene.theme, g_scene.medium.n);
    resize_and_reset(&g_scene);

    Medium   *m        = &g_scene.medium;
    long long step_acc = 0;
    long long last_ns  = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();   /* makes ncurses notice the new window size */
            resize_and_reset(&g_scene);
            last_ns  = clock_ns();
            step_acc = 0;
        }

        if (handle_key(&g_scene, getch()))   /* fresh grid -> restart the step clock */
            step_acc = 0;

        /* measure how much time passed, capped so a stall can't snowball */
        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        if (dt > DT_MAX_NS) dt = DT_MAX_NS;
        last_ns = now_ns;

        /* run however many sim steps that time bought us (unless paused) */
        if (!g_scene.paused) {
            step_acc += dt;
            while (step_acc >= STEP_NS) {
                medium_step(m);
                step_acc -= STEP_NS;
            }
        }

        /* draw a frame, then sleep just long enough to hold the frame rate */
        erase();
        scene_draw(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }
    return 0;
}
