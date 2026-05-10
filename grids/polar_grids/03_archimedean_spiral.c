/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_archimedean_spiral.c — Archimedean spiral grid (constant pitch)
 *
 * DEMO: One or more spiral arms wind outward at constant pitch — the gap
 *       between successive passes at any radius is always the same.  The
 *       N_ARMS trick renders all arms simultaneously with a single modular
 *       test per cell.  An '@' cursor sits at the (turn, spoke) cell — arrows
 *       step it across the grid.  +/- adjusts the pitch (tighter vs looser
 *       coil); [/] changes the number of arms.
 *
 * Study alongside: 04_log_spiral.c (variable pitch),
 *                  01_rings_spokes.c (constant ring spacing),
 *                  ../rect_grids/01_uniform_rect.c (the GridCtx template)
 *
 * Section map:
 *   §1 config   — pitch, arm count, themes, EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID + HUD/HINT/CURSOR
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg + angle_char
 *   §5 cursor   — Cursor (turn, spoke) + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme   r reset
 *        arrows move @   +/- pitch (pixels/turn)   [/] arm count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/03_archimedean_spiral.c \
 *       -o 03_archimedean_spiral -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Archimedean spiral arm detection via the N-arm phase test.
 *
 *                  An Archimedean spiral satisfies r = a × θ (one arm).
 *                  Rearranging: θ = r / a.  For N arms equally spaced by
 *                  2π/N, arm k satisfies θ = r/a + k × 2π/N.
 *
 *                  The N-arm phase trick: multiply the "offset" (θ − r/a) by
 *                  N and test whether the result is near a multiple of 2π:
 *
 *                    phase = fmod(N × (θ − r/a) + N×2π, 2π)
 *                    on_spiral : phase < W || phase > 2π − W
 *
 *                  Why it works: for point on arm k, θ = r/a + k×2π/N, so
 *                  N×(θ−r/a) = N×k×2π/N = k×2π.  fmod(k×2π, 2π) = 0. ✓
 *                  Any k tests as on-spiral, so ALL arms are detected at once.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, pitch, n_arms,
 *                  ox/oy) and Cursor (turn index, spoke index in the angular
 *                  fineness chosen for placement).  ctx_to_screen samples the
 *                  spiral at angle (turn × 2π + spoke × 2π/CURSOR_SPOKES)
 *                  and lays the cursor on the resulting (r, θ) point.
 *
 * Math           : The pitch (gap between successive turns at fixed θ) is
 *                  PITCH = a × 2π for a single arm, or PITCH/N for N arms.
 *                  The parameter a = PITCH / (2π).  Pitch is set in pixels
 *                  and relates to RING_SPACING in 01_rings_spokes: equal
 *                  pitch gives equal turn-to-turn gap for any radius.
 *
 * Rendering      : Because the spiral is neither purely horizontal nor
 *                  vertical, angle_char(theta) naturally produces the
 *                  correct slanted line character at each point.
 *
 * Performance    : Same O(rows × cols) sweep as 01.
 *
 * References     :
 *   Archimedean spiral — en.wikipedia.org/wiki/Archimedean_spiral
 *   Spirals in nature — Livio 2002, "The Golden Ratio", chapter 5
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   An Archimedean spiral winds outward at constant pitch: at any radius, the
 *   gap between successive turns is always PITCH pixels.  The N-arm phase test
 *   detects ALL arms simultaneously with one modular expression — no loop.
 *   The Cursor address (turn t, spoke s) names a sample taken on arm 0 at
 *   angle (t × 2π + (s + 0.5) × 2π / CURSOR_SPOKES), giving exactly one
 *   well-defined screen position per (t, s).
 *
 * HOW TO THINK ABOUT IT
 *   For arm 0: r = a × θ.  Every point on the spiral satisfies θ = r/a.
 *   The "phase" of a cell (r, θ) is how far θ deviates from the spiral's
 *   prediction r/a.  Phase ≈ 0 → on the spiral.  Phase ≈ π → opposite side.
 *
 *   For N arms equally spaced by 2π/N, arm k satisfies θ = r/a + k×2π/N.
 *   Multiplying the phase by N maps all N arms to the SAME modular value 0.
 *   One test catches every arm.
 *
 * DRAWING METHOD
 *   1. dx = (col−ox)×CELL_W,  dy = (row−oy)×CELL_H
 *   2. r = √(dx²+dy²),  θ = atan2(dy,dx)
 *   3. If r < MIN_R: skip
 *   4. θ_norm = fmod(θ + 2π, 2π)            ← [0, 2π)
 *   5. a = PITCH / (2π)                      ← radial advance per radian
 *   6. raw = N × (θ_norm − r/a)             ← N-arm phase, unbounded
 *   7. phase = fmod(raw + N×2π, 2π)          ← normalise to [0, 2π)
 *   8. on_spiral = phase < SPIRAL_W  ||  phase > 2π − SPIRAL_W
 *   9. Draw angle_char(θ) if on_spiral.
 *
 * KEY FORMULAS
 *   Single arm: r = a × θ,  so  a = PITCH / (2π)
 *     After one turn Δθ = 2π the radius increases by a×2π = PITCH.  ✓
 *
 *   N-arm phase derivation:
 *     On arm k: θ = r/a + k×2π/N
 *     phase = N × (θ_norm − r/a)
 *           = N × (r/a + k×2π/N − r/a)     [substitute arm k equation]
 *           = k × 2π
 *     fmod(k×2π, 2π) = 0  →  every arm k tests as "on_spiral".  ✓
 *
 *   Cursor → screen (turn t, spoke s):
 *     theta_sample = (t + (s + 0.5) / CURSOR_SPOKES) × 2π
 *     r_sample     = a × theta_sample
 *     cx = r_sample × cos theta_sample;  cy = r_sample × sin theta_sample
 *     sc = ox + (int)round(cx / CELL_W);  sr = oy + (int)round(cy / CELL_H)
 *
 * EDGE CASES TO WATCH
 *   • PITCH=0: a=0 → r/a = ∞.  Constrained to [PITCH_MIN, PITCH_MAX].
 *   • Centre smear: near origin θ_norm − r/a changes rapidly.
 *     Hard guard: skip cells with r < MIN_R.
 *   • Large N: each arm's angular width is SPIRAL_W/N in θ space.  At N=8
 *     arms may look very thin.  Increase SPIRAL_W if arms disappear.
 *   • Cursor max_turn comes from the visible radius vs pitch; recomputed
 *     on resize and on +/- pitch changes.
 *
 * HOW TO VERIFY
 *   PITCH=32, N=1, ox=40, oy=12.  a = 32/(2π) ≈ 5.093.
 *
 *   Point on arm 0 at θ=0 (rightward), k-th pass (k≥1):
 *     r = a × 2πk = PITCH × k  →  first pass at r=32px
 *     col = 40 + round(32/2) = 56.
 *   Check cell (col=56, row=12):
 *     dx=(56−40)×2=32, dy=0  →  r=32, θ=0  →  θ_norm=0
 *     raw = 1×(0 − 32/5.093) = −6.284 ≈ −2π
 *     phase = fmod(−6.284 + 2π, 2π) = fmod(0, 2π) = 0
 *     0 < SPIRAL_W(0.20)  →  on_spiral = true  ✓
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS      30
#define CELL_W          2
#define CELL_H          4

/* Spiral pitch: pixel distance between successive turns (= a × 2π for 1 arm) */
#define PITCH_DEFAULT   32.0
#define PITCH_MIN        8.0
#define PITCH_MAX       80.0
#define PITCH_STEP       4.0

/* Angular half-width of the spiral line (radians in N×phase space) */
#define SPIRAL_W        0.20

/* Number of spiral arms */
#define N_ARMS_DEFAULT   1
#define N_ARMS_MIN       1
#define N_ARMS_MAX       8

/* Minimum pixel radius — avoids a dense smear at the very centre */
#define MIN_R            3.0

/* Cursor angular fineness within one turn (samples per 2π) */
#define CURSOR_SPOKES    12

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA   0.05

#define PAIR_GRID    1
#define PAIR_CURSOR  2
#define PAIR_HUD     3
#define PAIR_HINT    4

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES 5

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec=(time_t)(ns/1000000000LL),
                          .tv_nsec=(long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg = COLORS >= 256 ? THEME_FG[theme][0] : THEME_FG[theme][1];
    init_pair(PAIR_GRID,   fg,                              -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the spiral ↔ screen mapping                  */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the Archimedean spiral plus cursor bounds.
 *
 * a = pitch / (2π) is the per-radian radial advance for arm 0.
 * max_turn is the largest cursor turn that still fits visibly on screen.
 */
typedef struct {
    int rows, cols;

    double pitch;          /* pixels per full turn                          */
    double a;              /* = pitch / (2π); cached in ctx_init             */
    int    n_arms;
    int    cell_w, cell_h;

    int    ox, oy;

    int    max_turn, max_spoke;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * For a sample at (turn t, spoke s) on arm 0:
 *   theta = (t + 0.5) × 2π   (worst case: the largest sample for that turn)
 *   r     = a × theta = pitch × (t + 0.5)
 * max_turn = floor(r_visible / pitch − 0.5).
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->pitch  <= 0.0) g->pitch  = PITCH_DEFAULT;
    if (g->n_arms <= 0)   g->n_arms = N_ARMS_DEFAULT;
    g->a = g->pitch / (2.0 * M_PI);

    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mt = (int)(r_visible / g->pitch - 0.5);
    g->max_turn  = mt < 0 ? 0 : mt;
    g->max_spoke = CURSOR_SPOKES - 1;
}

/*
 * ctx_to_screen — sample the spiral on arm 0 at (turn t, spoke s).
 *
 * THE FORMULA:
 *   theta_sample = (t + (s + 0.5) / CURSOR_SPOKES) × 2π
 *   r_sample     = a × theta_sample           (the Archimedean law)
 *   cx = r_sample × cos theta_sample
 *   cy = r_sample × sin theta_sample
 *   sc = ox + (int)round(cx / CELL_W)
 *   sr = oy + (int)round(cy / CELL_H)
 *
 * The (s + 0.5) shift centres the cursor inside its angular wedge so it
 * does not overlap the previous-spoke sample at integer s.
 */
static void ctx_to_screen(const GridCtx *g, int turn, int spoke,
                          int *sr, int *sc)
{
    double theta_sample = ((double)turn +
                           ((double)spoke + 0.5) / (double)CURSOR_SPOKES)
                          * 2.0 * M_PI;
    double r_sample = g->a * theta_sample;
    double cx = r_sample * cos(theta_sample);
    double cy = r_sample * sin(theta_sample);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/*
 * angle_char — pick the ASCII line character that best matches orientation theta.
 *
 * THE FORMULA:
 *   a = fmod(theta + 2π, π)  ← fold into [0, π) (orientation, not direction)
 *   a ∈ [0, π/8) or [7π/8, π) → '-';  a ∈ [π/8, 3π/8) → '\'
 *   a ∈ [3π/8, 5π/8) → '|';          a ∈ [5π/8, 7π/8) → '/'
 */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/*
 * ctx_draw_bg — sweep every cell, apply N-arm Archimedean phase test, draw.
 *
 * THE PIPELINE:
 *   a = pitch / (2π)                      per-radian radial advance
 *   for each cell:
 *     dx = (col−ox)×CELL_W,  dy = (row−oy)×CELL_H
 *     r  = √(dx²+dy²),  θ = atan2(dy,dx); if r < MIN_R: skip
 *     θ_norm = fmod(θ + 2π, 2π)
 *     raw    = N × (θ_norm − r/a)         N-arm phase (unbounded)
 *     phase  = fmod(raw + N×2π, 2π)       normalised to [0, 2π)
 *     on_spiral = phase < SPIRAL_W  ||  phase > 2π − SPIRAL_W
 */
static void ctx_draw_bg(const GridCtx *g)
{
    double two_pi = 2.0 * M_PI;
    double a = g->a;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            double r_px = sqrt(dx*dx + dy*dy);
            if (r_px < MIN_R) continue;

            double theta = atan2(dy, dx);
            double theta_norm = fmod(theta + two_pi, two_pi);

            /*
             * N-arm phase test:
             *   phase = N × (θ − r/a)  mod 2π
             * On arm k: phase = N×k×2π/N = k×2π ≡ 0.  ✓
             */
            double raw = (double)g->n_arms * (theta_norm - r_px / a);
            double phase = fmod(raw + (double)g->n_arms * two_pi, two_pi);

            if (phase < SPIRAL_W || phase > two_pi - SPIRAL_W) {
                char c = angle_char(theta);
                mvaddch(row, col, (chtype)(unsigned char)c);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — (turn, spoke) on arm 0.
 *
 * turn  : which winding pass (0 = first pass through (s+0.5)/N angle)
 * spoke : which angular sample within the turn (0..CURSOR_SPOKES−1)
 *
 * Bounds and geometry live in GridCtx.  The two structs compose:
 * Cursor + GridCtx → screen position via ctx_to_screen().
 */
typedef struct { int turn, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->turn  = g->max_turn / 2;
    cur->spoke = 0;
}

/*
 * cursor_move — apply (d_turn, d_spoke); turn clamped, spoke wraps mod
 * CURSOR_SPOKES.
 *
 * Arrow-key dispatch:
 *   UP    → turn  −1   (inward / earlier pass)
 *   DOWN  → turn  +1   (outward / later pass)
 *   LEFT  → spoke −1   (counter-clockwise within turn)
 *   RIGHT → spoke +1   (clockwise within turn — y-down convention)
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int d_turn, int d_spoke)
{
    int nt = cur->turn + d_turn;
    if (nt < 0)            nt = 0;
    if (nt > g->max_turn)  nt = g->max_turn;
    cur->turn = nt;

    int n = CURSOR_SPOKES;
    int ns = (cur->spoke + d_spoke) % n;
    if (ns < 0) ns += n;
    cur->spoke = ns;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->turn, cur->spoke, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     bool paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " turn:%d spoke:%d  pitch:%.0fpx  arms:%d  th:%d  %5.1f fps  %s ",
             cur->turn, cur->spoke, g->pitch, g->n_arms,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:pitch  [/]:arms ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       bool paused, double fps)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    int theme = 0;
    screen_init();
    color_init(theme);

    GridCtx g = {0};
    g.pitch  = PITCH_DEFAULT;
    g.n_arms = N_ARMS_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    bool   paused = false;
    double fps    = TARGET_FPS;
    int64_t t0    = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS);
            cursor_reset(&cur, &g);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 'r': cursor_reset(&cur, &g); break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        case '+': case '=':
            if (g.pitch < PITCH_MAX) {
                g.pitch += PITCH_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.turn > g.max_turn) cur.turn = g.max_turn;
            }
            break;
        case '-':
            if (g.pitch > PITCH_MIN) {
                g.pitch -= PITCH_STEP;
                ctx_init(&g, LINES, COLS);
            }
            break;
        case '[':
            if (g.n_arms > N_ARMS_MIN) g.n_arms--;
            break;
        case ']':
            if (g.n_arms < N_ARMS_MAX) g.n_arms++;
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
