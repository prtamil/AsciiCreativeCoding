/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cymatics.c — Chladni Figures (2-D Standing-Wave Nodal Patterns)
 *
 * A Chladni figure is the pattern of sand on a plate vibrating at a
 * resonant frequency.  Sand collects on the nodal lines where the
 * displacement is zero.
 *
 * Formula for mode (m, n):
 *   Z(x,y) = cos(m·π·x)·cos(n·π·y)  −  cos(n·π·x)·cos(m·π·y)
 *   x, y ∈ [0,1]
 *
 * Rendering:
 *   |Z| ≈ 0  → nodal line  (bright chars: @ # * + . depending on closeness)
 *   Z  > 0   → positive antinode  (dim, theme colour A)
 *   Z  < 0   → negative antinode  (dim, theme colour B)
 *
 * Animation: the figure slowly morphs to the next (m,n) mode by blending
 * Z values, then holds the complete figure before advancing.
 *
 * Themes (t key):
 *   Classic   white nodal / cyan+ / red−
 *   Ocean     white nodal / blue+ / teal−
 *   Ember     white nodal / orange+ / red−
 *   Neon      white nodal / green+ / magenta−
 *
 * Keys:
 *   n / p     next / previous mode
 *   t         next colour theme
 *   a         toggle auto-advance
 *   space     pause / resume
 *   q / Q     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/cymatics.c -o cymatics \
 *       -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 field  §5 scene
 *           §6 screen  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic Chladni figure computation — no simulation, no PDE.
 *                  The nodal pattern is computed by evaluating the 2D mode shape
 *                  function at every cell and testing if |Z| < threshold.
 *
 * Physics        : Chladni figures (Ernst Chladni, 1787): when a plate vibrates
 *                  at a resonant frequency, sand on the plate migrates to nodal
 *                  lines (where displacement = 0).  The mode (m,n) describes
 *                  how many half-wavelengths fit in each direction.
 *
 * Math           : Square plate mode shape function:
 *                    Z(x,y) = cos(m·π·x)·cos(n·π·y) − cos(n·π·x)·cos(m·π·y)
 *                  Nodal lines are where Z(x,y) = 0.  The cos−cos structure
 *                  comes from satisfying Neumann boundary conditions (free edge).
 *                  Resonant frequency: f_mn ∝ √(m² + n²) — Pythagorean relationship.
 *                  Modes with m=n are degenerate: Z=0 everywhere (trivial solution);
 *                  interesting patterns require m ≠ n.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A Chladni figure is just a contour plot of one analytic function:
 * Z(x, y) = cos(mπx)·cos(nπy) - cos(nπx)·cos(mπy).  Wherever Z ≈ 0 the
 * sand piles up; wherever Z is positive or negative the plate is
 * vibrating up or down (an antinode).  No simulation, no PDE solver,
 * no time-stepping for the physics — we just evaluate Z at every cell
 * each frame.  The animation is a simple linear blend between the
 * current mode's Z and the next mode's Z, with t walking from 0 to 1.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine pouring sand on a vibrating glass plate.  The plate has
 * standing-wave humps in two directions; integers m and n count the
 * humps along x and y.  The MINUS in cos·cos − cos·cos enforces the
 * symmetry: swap (x↔y) and Z flips sign — so the nodal lines lie along
 * the diagonals of symmetry.  Sand falls off the moving regions and
 * collects on the still ones.  We simulate "still" with the test
 * |Z| < threshold and paint a glyph there; "moving" we paint blank or
 * a faint coloured antinode tint.  Picking different (m, n) pairs gives
 * a museum of these classic patterns — one per pair.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Each tick:
 *       if paused → return.
 *       if state == ST_HOLD and auto: hold_ctr++; if ≥ HOLD_TICKS,
 *         enter ST_MORPH with t=0.
 *       if state == ST_MORPH: t += MORPH_SPEED.  When t ≥ 1, advance
 *         g_mode_idx by 1 (mod N_MODES), reset t = 0, return to HOLD.
 *  2. Each frame, for every cell (r, c):
 *       Map (r, c) → (x, y) ∈ [0, 1]².
 *       z1 = chladni(x, y, m_cur, n_cur)
 *       z  = z1   if t == 0
 *          = (1-t)·z1 + t·z2  otherwise (z2 from next mode)
 *  3. Compute az = |z|.  Five threshold bands:
 *       az < 0.04 → '@' (nodal core, white, A_BOLD)
 *       az < 0.10 → '#' (close to nodal, A_BOLD)
 *       az < 0.18 → '*' (penumbra, A_DIM, theme colour)
 *       az < 0.28 → '+' (faint antinode glow, A_DIM)
 *       az < 0.40 → '.' (very faint, A_DIM)
 *       else      → ' ' (blank — pure antinode)
 *  4. Glyphs in the inner 2 bands paint with CP_NODE (white).  Outer
 *     bands paint with CP_POS or CP_NEG depending on sign(z).
 *  5. HUD shows current and next (m, n), state (hold / morph), morph
 *     parameter t, theme name, and key map.
 *
 * KEY FORMULAS
 * ────────────
 *  Mode shape       Z(x,y) = cos(mπx)·cos(nπy) - cos(nπx)·cos(mπy)
 *  Symmetry         Z(y,x) = -Z(x,y)         (anti-symmetric in x↔y)
 *  Resonance        f_{m,n} ∝ √(m² + n²)
 *  Cell→plate       x = c / (cols - 1),  y = r / (rows - 1)
 *  Morph blend      Z(x,y,t) = (1-t)·Z_cur + t·Z_next     t ∈ [0,1]
 *  Nodal test       |Z| < NODAL_THRESH  → near-zero displacement
 *  Band thresholds  0.04, 0.10, 0.18, 0.28, 0.40 → glyphs @ # * + .
 *  Hold then morph  state oscillates HOLD (HOLD_TICKS=120 ≈ 4s) →
 *                   MORPH (1/MORPH_SPEED = 40 ticks ≈ 1.3s) → HOLD …
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The mode list intentionally OMITS m == n: chladni(x, y, k, k)
 *    returns 0 everywhere because the two cos·cos terms are equal —
 *    the visual would be a blank plate.  Adding {1,1} or {2,2} to
 *    MODES[] creates that degenerate frame.
 *  • Cell (0,0), (cols-1,0), (0,rows-1), and (cols-1,rows-1) ALWAYS
 *    sit on a nodal line because cos(0) = cos(π) = ±1 — the corners
 *    are perpetually still.  This is the free-edge boundary.
 *  • Resize is implicit: g_rows / g_cols are read every frame.  After
 *    SIGWINCH endwin/refresh re-syncs but the figure briefly stutters
 *    while next is recomputed.
 *  • MORPH_SPEED = 0.025 means a full transition in 40 ticks; halving
 *    it doubles morph time.  Dropping below 0.005 produces visible
 *    flicker because the float t becomes hard to distinguish frame
 *    to frame.
 *  • Pressing 'n' or 'p' during a morph aborts it (state → HOLD,
 *    t = 0); the next frame renders the new mode cleanly.
 *  • Auto-advance flag controls ONLY the HOLD → MORPH transition;
 *    once morphing, it always completes.  Toggle 'a' to freeze on
 *    the current mode.
 *  • Theme change re-runs init_pair on CP_POS and CP_NEG only —
 *    CP_NODE stays white.  The nodal lines never change colour.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Mode (1,2): expect ONE horizontal and ONE vertical nodal line
 *    crossing at the centre, with four antinode quadrants alternating
 *    sign in a + pattern.
 *  • Mode (2,3): three diagonal nodal arcs visible; count them.
 *  • Mode (3,4): the figure has 3·4·2 = 24 antinodal cells (rough
 *    rule of thumb — m·n·2 visual cells for low integer modes).
 *  • Press 'n' rapidly: HUD mode counter cycles 1/20 → 2/20 → … →
 *    20/20 → 1/20.  Confirm the figure changes for each.
 *  • Press 't' four times to return to the original theme — the
 *    pattern should be identical, only colour swapped.
 *  • Pause mid-morph (space) and confirm t freezes at its current
 *    value (HUD shows it).  Resume — morph continues.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS       33333333LL
#define HOLD_TICKS    120          /* ~4 s hold before morphing starts    */
#define MORPH_SPEED   0.025f       /* t increment per tick (~1.3 s morph) */
#define NODAL_THRESH  0.04f        /* |Z| < this → centre nodal char      */

enum { ST_HOLD = 0, ST_MORPH };
enum { CP_POS=1, CP_NEG, CP_NODE, CP_HUD };

/* All (m,n) mode pairs with 1 ≤ m < n ≤ 7 */
static const int MODES[][2] = {
    {1,2},{1,3},{2,3},{1,4},{2,4},{3,4},
    {1,5},{2,5},{3,5},{4,5},{1,6},{2,6},
    {3,6},{4,6},{5,6},{2,7},{3,7},{4,7},
    {5,7},{6,7}
};
#define N_MODES  (int)(sizeof(MODES)/sizeof(MODES[0]))

#define N_THEMES 4
/* theme[theme][0]=pos fg, [1]=neg fg */
static const short THEMES_256[N_THEMES][2] = {
    {  51, 196 },   /* Classic:  cyan / red      */
    {  45,  30 },   /* Ocean:    blue / teal      */
    { 202, 160 },   /* Ember:  orange / dark-red  */
    {  82, 201 },   /* Neon:    green / magenta   */
};
static const short THEMES_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_RED     },
    { COLOR_BLUE,    COLOR_CYAN    },
    { COLOR_YELLOW,  COLOR_RED     },
    { COLOR_GREEN,   COLOR_MAGENTA },
};
static const char *THEME_NAMES[N_THEMES] = {
    "Classic", "Ocean", "Ember", "Neon"
};

/* density chars: distance from nodal line → char brightness */
static const char NODAL_CHARS[] = "@#*+.";   /* closest → farthest  */
#define N_NCHARS (int)(sizeof(NODAL_CHARS) - 1)

/* ── §2 clock ────────────────────────────────────────────────────────── */

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

/* ── §3 color ────────────────────────────────────────────────────────── */

static int g_theme = 0;

static void color_init(void)
{
    start_color();
    use_default_colors();
    short pos, neg;
    if (COLORS >= 256) {
        pos = THEMES_256[g_theme][0];
        neg = THEMES_256[g_theme][1];
    } else {
        pos = THEMES_8[g_theme][0];
        neg = THEMES_8[g_theme][1];
    }
    init_pair(CP_POS,  pos,                                  -1);
    init_pair(CP_NEG,  neg,                                  -1);
    init_pair(CP_NODE, (COLORS >= 256) ? 231 : COLOR_WHITE,  -1);
    init_pair(CP_HUD,  (COLORS >= 256) ?  82 : COLOR_GREEN,  -1);
}

/* ── §4 field ────────────────────────────────────────────────────────── */

static int g_rows, g_cols;
static int g_mode_idx;
static int g_state;
static int g_hold_ctr;
static float g_t;           /* morph interpolation 0→1                  */
static int g_auto;
static int g_paused;

/* Chladni formula for mode (m,n) at normalised (x,y) ∈ [0,1] */
static float chladni(float x, float y, int m, int n)
{
    return cosf((float)m * (float)M_PI * x) * cosf((float)n * (float)M_PI * y)
         - cosf((float)n * (float)M_PI * x) * cosf((float)m * (float)M_PI * y);
}

/* Blended Z for current animation state */
static float z_at(int r, int c)
{
    float x = (g_cols > 1) ? (float)c / (float)(g_cols - 1) : 0.5f;
    float y = (g_rows > 1) ? (float)r / (float)(g_rows - 1) : 0.5f;

    int cm = MODES[g_mode_idx][0], cn = MODES[g_mode_idx][1];
    float z1 = chladni(x, y, cm, cn);
    if (g_t <= 0.0f) return z1;

    int nm = MODES[(g_mode_idx + 1) % N_MODES][0];
    int nn = MODES[(g_mode_idx + 1) % N_MODES][1];
    float z2 = chladni(x, y, nm, nn);
    return (1.0f - g_t) * z1 + g_t * z2;
}

static void sim_tick(void)
{
    if (g_paused) return;

    if (g_state == ST_HOLD) {
        if (g_auto) {
            if (++g_hold_ctr >= HOLD_TICKS) {
                g_state = ST_MORPH;
                g_t     = 0.0f;
            }
        }
        return;
    }

    /* ST_MORPH */
    g_t += MORPH_SPEED;
    if (g_t >= 1.0f) {
        g_t        = 0.0f;
        g_mode_idx = (g_mode_idx + 1) % N_MODES;
        g_state    = ST_HOLD;
        g_hold_ctr = 0;
    }
}

/* ── §5 scene ────────────────────────────────────────────────────────── */

static void scene_draw(void)
{
    for (int r = 0; r < g_rows - 1; r++) {
        for (int c = 0; c < g_cols - 1; c++) {
            float z  = z_at(r, c);
            float az = fabsf(z);

            if (az < 0.40f) {
                /* nodal / near-nodal band — map distance to char */
                /* threshold bands: 0.04, 0.10, 0.18, 0.28, 0.40 */
                static const float BANDS[] = {0.04f, 0.10f, 0.18f, 0.28f, 0.40f};
                int idx = N_NCHARS - 1;
                for (int b = 0; b < N_NCHARS; b++) {
                    if (az < BANDS[b]) { idx = b; break; }
                }
                int cp = (idx == 0) ? CP_NODE :
                         (z > 0.0f) ? CP_POS : CP_NEG;
                attr_t attr = (idx <= 1) ? (COLOR_PAIR(cp) | A_BOLD)
                                         : (COLOR_PAIR(cp) | A_DIM);
                attron(attr);
                mvaddch(r, c, (chtype)(unsigned char)NODAL_CHARS[idx]);
                attroff(attr);
            } else {
                mvaddch(r, c, ' ');
            }
        }
    }
}

static void scene_hud(void)
{
    int cm = MODES[g_mode_idx][0], cn = MODES[g_mode_idx][1];
    int nm = MODES[(g_mode_idx + 1) % N_MODES][0];
    int nn = MODES[(g_mode_idx + 1) % N_MODES][1];
    const char *state_str = (g_state == ST_MORPH) ? "morphing" : "hold    ";

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " mode(%d,%d)→(%d,%d) [%d/%d] %s t=%.2f  theme=%-7s  %s"
             "  n/p:mode  t:theme  a:%s  spc:pause  q:quit",
             cm, cn, nm, nn,
             g_mode_idx + 1, N_MODES, state_str, (double)g_t,
             THEME_NAMES[g_theme],
             g_paused ? "PAUSED" : "      ",
             g_auto ? "auto" : "man.");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §6 screen ───────────────────────────────────────────────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows;
    g_cols = cols;
    erase();
}

/* ── §7 app ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    screen_init();
    g_mode_idx = 0;
    g_state    = ST_HOLD;
    g_hold_ctr = 0;
    g_t        = 0.0f;
    g_auto     = 1;
    g_paused   = 0;
    screen_resize();

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize();
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0; break;
            case ' ':           g_paused ^= 1; break;
            case 'a': case 'A': g_auto   ^= 1; break;
            case 'n':
                g_mode_idx = (g_mode_idx + 1) % N_MODES;
                g_state = ST_HOLD; g_hold_ctr = 0; g_t = 0.0f;
                break;
            case 'p':
                g_mode_idx = (g_mode_idx - 1 + N_MODES) % N_MODES;
                g_state = ST_HOLD; g_hold_ctr = 0; g_t = 0.0f;
                break;
            case 't': case 'T':
                g_theme = (g_theme + 1) % N_THEMES;
                color_init();
                break;
            }
        }

        sim_tick();
        erase();
        scene_draw();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
