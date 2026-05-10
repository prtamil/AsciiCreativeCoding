/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_rings_spokes.c — standard polar grid: concentric rings + radial spokes
 *
 * DEMO: Every screen cell is tested for proximity to a ring or spoke using
 *       polar coordinates.  Rings are detected when the pixel radius is a
 *       near-integer multiple of RING_SPACING.  Spokes are detected when
 *       the polar angle is a near-multiple of 2π/N_SPOKES.  An '@' cursor
 *       sits at one (ring, spoke) cell — arrows step it across the grid.
 *
 * Study alongside: 02_log_polar.c (logarithmic ring spacing),
 *                  ../rect_grids/01_uniform_rect.c (the GridCtx template),
 *                  ../hex_grids/01_flat_top.c (non-trivial GridCtx adaptation)
 *
 * Section map:
 *   §1 config   — RING_SPACING, N_SPOKES, thresholds, themes, EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID + HUD/HINT/CURSOR
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg + angle_char
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme   r reset
 *        arrows move @   +/- ring spacing   [/] spoke count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/01_rings_spokes.c \
 *       -o 01_rings_spokes -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Screen-sweep polar detection.  For every terminal cell
 *                  (col, row) the pixel-space distance r and angle θ from
 *                  the screen centre are computed.  Two Boolean tests decide
 *                  what to draw:
 *
 *                    on_ring : fmod(r, RING_SPACING) < RING_W
 *                              || fmod(r, RING_SPACING) > RING_SPACING−RING_W
 *                    on_spoke: fmod(θ_norm, spoke_angle) < SPOKE_W
 *                              || fmod(θ_norm, spoke_angle) > spoke_angle−SPOKE_W
 *
 *                  Both tests use modular arithmetic so they detect ALL rings
 *                  and ALL spokes simultaneously with a single expression each.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, ring_spacing,
 *                  n_spokes, CELL_W/CELL_H, screen origin ox/oy) and Cursor
 *                  (linear ring index, spoke index).  No grid array; the
 *                  grid is computed per pixel via modular arithmetic, and
 *                  ctx_to_screen places (ring, spoke) at the centre of its
 *                  annular sector.
 *
 * Rendering      : Character selection by angle makes spokes look like
 *                  actual lines: horizontal near ±0°, vertical near ±90°,
 *                  diagonals in between.  The ring character uses the same
 *                  angle_char() function so ring and spoke intersections join
 *                  smoothly.  Cursor cell highlighted with '@' over the
 *                  (ring + 0.5) × spacing radius at the cursor's spoke angle.
 *
 * Performance    : O(rows × cols) per frame with one sqrt + one atan2 per
 *                  cell.  At 80×24 that is 1 920 cells — imperceptibly fast.
 *
 * References     :
 *   Polar coordinate system — en.wikipedia.org/wiki/Polar_coordinate_system
 *   Terminal aspect ratio   — CLAUDE.md §Coordinate/Physics Model
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   A polar grid divides the plane into concentric rings (equal distance
 *   bands) and radial spokes (equal angle wedges).  Every screen cell is
 *   either on a ring boundary, on a spoke boundary, both, or neither.
 *   The Cursor address is (ring, spoke) — a linear ring index and a spoke
 *   wedge index — and ctx_to_screen places the centre of that annular
 *   sector at (mid_radius × cos θ_centre, mid_radius × sin θ_centre).
 *
 * HOW TO THINK ABOUT IT
 *   Picture the screen as a flat disc viewed from directly above.  The origin
 *   is the screen centre.  Moving right is angle 0°; moving down is 90°
 *   (terminal y grows downward, so atan2 follows screen-y = positive).
 *
 *   Two independent Boolean tests cover the entire grid:
 *     • Ring test: is the cell's radius a near-integer multiple of RING_SPACING?
 *     • Spoke test: is the cell's angle a near-integer multiple of 2π/N_SPOKES?
 *
 * DRAWING METHOD
 *   1. Compute pixel-space offset: dx = (col−ox)×CELL_W, dy = (row−oy)×CELL_H
 *   2. r = √(dx²+dy²),  θ = atan2(dy,dx)  ∈ (−π, π]
 *   3. Normalise θ to [0, 2π):  θ_norm = fmod(θ + 2π, 2π)
 *   4. Ring test:  ring_phase = fmod(r, RING_SPACING)
 *                  on_ring = ring_phase < RING_W  ||  ring_phase > RING_SPACING−RING_W
 *   5. Spoke test: spoke_phase = fmod(θ_norm, spoke_angle)
 *                  on_spoke = r > SPOKE_MIN_R  &&
 *                             (spoke_phase < SPOKE_W || spoke_phase > spoke_angle−SPOKE_W)
 *   6. Draw '+' at intersection, angle_char(θ) otherwise.
 *
 * KEY FORMULAS
 *   Aspect correction (why CELL_H=4, CELL_W=2):
 *     Terminal characters are ~2× taller than wide.  Without dy×CELL_H
 *     scaling, "equal radii" form ellipses on screen.  The ratio
 *     CELL_H/CELL_W = 2 matches the typical 2:1 terminal cell aspect,
 *     making the Euclidean distance circular rather than elliptical.
 *
 *   Cursor → screen (ring k, spoke s):
 *     mid_radius = (k + 0.5) × RING_SPACING
 *     theta_mid  = (s + 0.5) × (2π / N_SPOKES)
 *     cx_pix = mid_radius × cos theta_mid
 *     cy_pix = mid_radius × sin theta_mid
 *     sc = ox + (int)round(cx_pix / CELL_W)
 *     sr = oy + (int)round(cy_pix / CELL_H)
 *
 * EDGE CASES TO WATCH
 *   • Centre smear: at r < SPOKE_MIN_R the spoke test would fill a solid disc.
 *     Guarded with (r_px > SPOKE_MIN_R).
 *   • θ range: atan2 returns (−π, π].  Adding 2π before fmod normalises to
 *     [0, 2π) — without this, fmod(−0.1, spoke_angle) ≈ −0.1 (negative),
 *     which is neither < SPOKE_W nor > spoke_angle−SPOKE_W → spoke goes missing.
 *   • Ring at r=0: ring_phase=0 → always on_ring at origin.  The SPOKE_MIN_R
 *     guard does NOT suppress this centre dot.
 *   • Cursor bounds: max_ring/max_spoke are recomputed in ctx_init from the
 *     terminal extent; resize re-clamps the cursor.
 *
 * HOW TO VERIFY
 *   Terminal 80×24, RING_SPACING=20, N_SPOKES=12, ox=40, oy=12.
 *
 *   Cell (col=60, row=12) — right of centre on horizontal axis:
 *     dx = (60−40)×2 = 40,  dy = (12−12)×4 = 0
 *     r = √(1600) = 40.0 px  ← exactly 2×RING_SPACING → ring_phase = 0
 *     on_ring = (0 < 1.6) = true
 *     θ = atan2(0,40) = 0  →  angle_char(0) = '-'  ✓
 *
 *   Cursor (ring=1, spoke=0): mid_radius=30, theta_mid=2π/24≈0.262
 *     cx=30·cos(0.262)≈28.97, cy=30·sin(0.262)≈7.76
 *     sc=40+round(28.97/2)=40+14=54, sr=12+round(7.76/4)=12+2=14
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

#define TARGET_FPS    30

/* Terminal cell size in "pixels" — ratio CELL_H/CELL_W ≈ 2 makes circles
 * appear circular despite rows being taller than columns.              */
#define CELL_W        2
#define CELL_H        4

/* Ring geometry */
#define RING_SPACING_DEFAULT  20.0f  /* pixels between rings  */
#define RING_SPACING_MIN       8.0f
#define RING_SPACING_MAX      48.0f
#define RING_SPACING_STEP      4.0f
#define RING_W                 1.6f  /* pixel half-width of a ring line */

/* Spoke geometry */
#define N_SPOKES_DEFAULT  12
#define N_SPOKES_MIN       4
#define N_SPOKES_MAX      36
#define SPOKE_W            0.10      /* radian half-width of a spoke line */
#define SPOKE_MIN_R        3.0f      /* ignore centre blob */

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA     0.05

/* Colour pairs */
#define PAIR_GRID    1
#define PAIR_CURSOR  2   /* bright '@' cursor */
#define PAIR_HUD     3   /* yellow status bar */
#define PAIR_HINT    4   /* cyan key-hint footer */

/* Theme palette: 256-colour fg, 8-colour fallback */
static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES  5

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
/* §4  formula — GridCtx and the ring/spoke ↔ screen mapping              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active polar grid plus cursor bounds.
 *
 * The grid is centred on screen.  For terminal cell (col, row):
 *   dx_px = (col − ox) × CELL_W      ← pixel offset from screen centre
 *   dy_px = (row − oy) × CELL_H
 * with  ox = cols/2,  oy = rows/2.
 *
 * ring_spacing/n_spokes are tunable per frame (+/-, [/]).
 * max_ring is the largest cursor ring that still places its centre inside
 * the visible area; max_spoke = n_spokes − 1.
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* polar geometry */
    float  ring_spacing;   /* pixels between rings                          */
    int    n_spokes;       /* number of radial wedges                       */
    int    cell_w, cell_h; /* sub-pixel scaling — CELL_W, CELL_H            */

    /* screen origin = pixel (0,0) */
    int    ox, oy;

    /* cursor bounds */
    int    max_ring, max_spoke;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * ox/oy are integer cell coordinates of the screen centre.
 * max_ring is derived from the half-extent that stays on screen given
 * ring_spacing: pixel half-radius / ring_spacing − 1, clamped non-negative.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->ring_spacing <= 0.0f) g->ring_spacing = RING_SPACING_DEFAULT;
    if (g->n_spokes     <= 0)    g->n_spokes     = N_SPOKES_DEFAULT;

    /* Largest ring that still fits inside the smaller pixel half-extent. */
    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mr = (int)(r_visible / g->ring_spacing) - 1;
    g->max_ring  = mr < 0 ? 0 : mr;
    g->max_spoke = g->n_spokes - 1;
}

/*
 * ctx_to_screen — centre cell of (ring k, spoke s) in screen coordinates.
 *
 * THE FORMULA:
 *   mid_radius = (k + 0.5) × RING_SPACING
 *   theta_mid  = (s + 0.5) × (2π / N_SPOKES)
 *   cx_pix = mid_radius × cos(theta_mid)
 *   cy_pix = mid_radius × sin(theta_mid)
 *   sc = ox + (int)round(cx_pix / CELL_W)
 *   sr = oy + (int)round(cy_pix / CELL_H)
 */
static void ctx_to_screen(const GridCtx *g, int ring, int spoke,
                          int *sr, int *sc)
{
    double mid_radius = ((double)ring + 0.5) * (double)g->ring_spacing;
    double theta_mid  = ((double)spoke + 0.5) * (2.0 * M_PI / (double)g->n_spokes);
    double cx = mid_radius * cos(theta_mid);
    double cy = mid_radius * sin(theta_mid);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/*
 * angle_char — pick the ASCII line character that best matches orientation theta.
 *
 * THE FORMULA:
 *   a = fmod(theta + 2π, π)   ← fold (−π, π] into [0, π) (lines have no direction)
 *   a ∈ [0, π/8) or [7π/8, π) → '-'   (near-horizontal)
 *   a ∈ [π/8,  3π/8)          → '\'  (diagonal down-right)
 *   a ∈ [3π/8, 5π/8)          → '|'  (near-vertical)
 *   a ∈ [5π/8, 7π/8)          → '/'  (diagonal up-right)
 *
 * Why fold by π not 2π?  A line at angle α and α+π looks identical in ASCII
 * (no directed arrow, only orientation), so folding into [0,π) halves the cases.
 */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI); /* fold to [0, π) */
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/*
 * ctx_draw_bg — sweep every cell, apply ring and spoke tests, draw.
 *
 * THE PIPELINE:
 *   for each cell (col, row):
 *     dx = (col−ox)×CELL_W, dy = (row−oy)×CELL_H
 *     r  = √(dx²+dy²),  θ = atan2(dy,dx)
 *     ring_phase  = fmod(r, ring_spacing)        scalar distance to nearest ring
 *     spoke_phase = fmod(θ_norm, spoke_angle)    scalar angle to nearest spoke
 *     on_ring  = ring_phase  < RING_W  ||  ring_phase  > spacing − RING_W
 *     on_spoke = r > MIN_R   &&  (spoke_phase < SPOKE_W || > angle − SPOKE_W)
 *     draw '+' if both; angle_char(θ) if either; skip if neither
 *
 * The dual-boundary test (< W || > step−W) fires at BOTH edges of each band,
 * so the drawn line has width 2×RING_W px centred on the ring radius.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    double spoke_angle = 2.0 * M_PI / (double)g->n_spokes;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            float  r_px = (float)sqrt(dx*dx + dy*dy);
            double theta = atan2(dy, dx);

            /* ring test: r near any integer multiple of ring_spacing */
            float ring_phase = fmodf(r_px, g->ring_spacing);
            bool on_ring = (ring_phase < RING_W ||
                            ring_phase > g->ring_spacing - RING_W);

            /* spoke test: theta near any multiple of spoke_angle */
            double theta_norm  = fmod(theta + 2.0*M_PI, 2.0*M_PI);
            double spoke_phase = fmod(theta_norm, spoke_angle);
            bool on_spoke = (r_px > SPOKE_MIN_R) &&
                            (spoke_phase < SPOKE_W ||
                             spoke_phase > spoke_angle - SPOKE_W);

            if (!on_ring && !on_spoke) continue;

            char c = (on_ring && on_spoke) ? '+' : angle_char(theta);
            mvaddch(row, col, (chtype)(unsigned char)c);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — (ring index, spoke index).
 *
 * Bounds and geometry live in GridCtx; the cursor is just where in
 * (ring, spoke) address space the user is pointing.  The two structs
 * compose: Cursor + GridCtx → screen position via ctx_to_screen().
 */
typedef struct { int ring, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->ring  = g->max_ring / 2;
    cur->spoke = 0;
}

/*
 * cursor_move — apply (d_ring, d_spoke) and clamp / wrap.
 *
 * Ring is clamped to [0, max_ring].  Spoke wraps modulo n_spokes
 * (the angular dimension is naturally periodic).
 *
 * Arrow-key dispatch:
 *   UP     → ring −1
 *   DOWN   → ring +1
 *   LEFT   → spoke −1 (counter-clockwise)
 *   RIGHT  → spoke +1 (clockwise in screen y-down convention)
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int d_ring, int d_spoke)
{
    int nr = cur->ring + d_ring;
    if (nr < 0)              nr = 0;
    if (nr > g->max_ring)    nr = g->max_ring;
    cur->ring = nr;

    int n = g->n_spokes > 0 ? g->n_spokes : 1;
    int ns = (cur->spoke + d_spoke) % n;
    if (ns < 0) ns += n;
    cur->spoke = ns;
}

/*
 * cursor_draw — place '@' at the centre cell of the cursor (ring, spoke).
 *
 * Drawn after ctx_draw_bg so '@' sits on top of any ring/spoke characters.
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->ring, cur->spoke, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow status (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     bool paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " ring:%d spoke:%d  rings:%.0fpx  spokes:%d  th:%d  %5.1f fps  %s ",
             cur->ring, cur->spoke, (double)g->ring_spacing, g->n_spokes,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:rings  [/]:spokes ");
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
    g.ring_spacing = RING_SPACING_DEFAULT;
    g.n_spokes     = N_SPOKES_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    bool   paused   = false;
    double fps      = TARGET_FPS;
    int64_t t0      = clock_ns();
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
        case 't':
            theme = (theme + 1) % N_THEMES;
            color_init(theme);
            break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        case '+': case '=':
            if (g.ring_spacing < RING_SPACING_MAX) {
                g.ring_spacing += RING_SPACING_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case '-':
            if (g.ring_spacing > RING_SPACING_MIN) {
                g.ring_spacing -= RING_SPACING_STEP;
                ctx_init(&g, LINES, COLS);
            }
            break;
        case '[':
            if (g.n_spokes > N_SPOKES_MIN) {
                g.n_spokes -= (g.n_spokes > 8 ? 4 : 2);
                ctx_init(&g, LINES, COLS);
                if (cur.spoke > g.max_spoke) cur.spoke = g.max_spoke;
            }
            break;
        case ']':
            if (g.n_spokes < N_SPOKES_MAX) {
                g.n_spokes += (g.n_spokes >= 8 ? 4 : 2);
                ctx_init(&g, LINES, COLS);
            }
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
