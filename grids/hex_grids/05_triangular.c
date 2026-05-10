/*
 * 05_triangular.c — triangular grid with stripe-index cursor movement
 *
 * DEMO: Equilateral triangular tiling. A '@' cursor moves between adjacent
 *       triangles using arrow keys. The cursor triangle is highlighted.
 *       The cursor position is stored as two stripe indices (si, sj) —
 *       a different, non-cube coordinate system specific to triangular grids.
 *
 * Study alongside: ../README.md (the GridCtx + Cursor abstraction),
 *                  hex_grids/01_flat_top.c (hex = dual of this tiling)
 *
 * Section map:
 *   §1 config   — CELL_W/CELL_H, tri_size + border_w bounds, themes,
 *                 FPS_EWMA_ALPHA, color pair IDs
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — border / cursor / HUD / HINT pairs (4 themes)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *                 (three-family stripe classifier per pixel)
 *   §5 cursor   — Cursor (si, sj) + cursor_reset / cursor_move / cursor_draw
 *                 + TRI_DIR direction table
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids/05_triangular.c \
 *       -o 05_triangular -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three-family stripe classifier. For triangle side s,
 *                  h = s·√3/2:
 *                    n1 = py / h             (horizontal family)
 *                    n2 = (√3·px + py) / h   (−60° family)
 *                    n3 = (−√3·px + py) / h  (+60° family)
 *                  edge_frac(nk) < border_w → draw '-', '/', or '\'.
 *
 * Data-structure : Two structs — GridCtx (cell size, terminal size, cursor
 *                  bounds in stripe-index space) and Cursor (just (si, sj)).
 *                  No grid array; the three-family stripe values are
 *                  recomputed per pixel each frame. See ../README.md "The
 *                  two primitives" for why the GridCtx + Cursor split is
 *                  reused across rect / hex / tri / polar.
 *
 * Cursor storage : (si, sj) = (floor(n1), floor(n2)) — two stripe indices
 *                  uniquely identify the triangle. This works because each
 *                  triangle occupies exactly one integer cell in the (n1,n2)
 *                  stripe-pair space.
 *
 *                  Triangle center in pixel space (for centered grid):
 *                    py_c = (si + 0.5) × h
 *                    px_c = (sj − si) × s/2
 *                  Derived by solving n1_center = si+0.5 and n2_center = sj+0.5.
 *
 * Cursor movement : TRI_DIR[4] moves by ±1 in si or sj:
 *                    UP/DOWN change si → move vertically by one stripe row
 *                    LEFT/RIGHT change sj → move horizontally by half a side
 *                  Adjacent triangles share an edge, so each step moves to
 *                  exactly one neighboring triangle.
 *
 * References     :
 *   Triangular tiling        — https://mathworld.wolfram.com/TriangularGrid.html
 *   Hex-triangle duality     — https://www.redblobgames.com/grids/hexagons/#map-storage
 *   Archimedean tilings list — https://en.wikipedia.org/wiki/Archimedean_tiling
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The equilateral triangular tiling is the dual of the hexagonal tiling:
 * place a point at every hex center and connect adjacent centers → you get
 * a triangular grid. The triangular grid has THREE families of parallel
 * lines (not two, like a rectangular grid). These families run at 0°, 60°,
 * and 120° to the horizontal. A pixel is "on a border" if it falls close to
 * any of the three families.
 *
 * The three-family classifier is fundamentally different from the cube-
 * coordinate hex classifier. There is no "cube rounding" — instead you
 * measure fractional distance to the nearest line in each family, and use
 * the MINIMUM as the border-distance metric.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each line family divides the plane into parallel stripes. Family 1 is
 * horizontal: stripes are rows of height h = s·√3/2. Family 2 tilts at
 * 60°. Family 3 tilts at 120°. Together, these three families slice the
 * plane into small equilateral triangles.
 *
 * For each pixel, compute its fractional position WITHIN each stripe
 * (where 0 = on a line, 0.5 = maximum distance from any line). The
 * minimum over all three families tells you how close the pixel is to the
 * nearest triangular edge. If that minimum is below border_w → border.
 *
 * The cursor uses a STRIPE INDEX coordinate (si, sj), not cube coordinates.
 * si = floor(n1) identifies which row, sj = floor(n2) identifies which
 * diagonal stripe. Together (si, sj) uniquely labels a triangle.
 *
 * DRAWING METHOD  (three-family stripe rasterizer)
 * ─────────────────────────────────────────────────
 *  1. Center the grid: px = (col−ox)×CELL_W, py = (row−oy)×CELL_H.
 *
 *  2. Compute three stripe coordinates:
 *       n1 = py / h                    (family 1: horizontal lines at y = k·h)
 *       n2 = (√3·px + py) / h         (family 2: lines tilted −60°)
 *       n3 = (−√3·px + py) / h        (family 3: lines tilted +60°)
 *
 *  3. Compute edge fraction for each:
 *       d1 = edge_frac(n1)   d2 = edge_frac(n2)   d3 = edge_frac(n3)
 *       edge_frac(v) = min(frac(v), 1−frac(v))  ∈ [0, 0.5]
 *       0 = on a line, 0.5 = center of stripe.
 *
 *  4. dmin = min(d1, d2, d3). The character matches whichever family wins:
 *       d1 wins → '-'   (horizontal line)
 *       d2 wins → '/'   (−60° line)
 *       d3 wins → '\'   (+60° line)
 *
 *  5. Stripe indices for cursor detection:
 *       cell_si = floor(n1),  cell_sj = floor(n2)
 *       on_cur = (cell_si == cur.si && cell_sj == cur.sj)
 *
 *  6. Draw:
 *       dmin < border_w  → border char (cursor or border color)
 *       dmin ≥ border_w && on_cur → fill interior with PAIR_CURSOR background
 *
 * KEY FORMULAS
 * ────────────
 *  Stripe period: h = tri_size × √3 / 2
 *  (The perpendicular distance between parallel lines in each family)
 *
 *  Three stripe coordinates:
 *    n1 = py / h
 *    n2 = (√3·px + py) / h
 *    n3 = (−√3·px + py) / h
 *
 *  edge_frac(v) = frac(v) if frac(v) < 0.5 else 1 − frac(v)
 *    where frac(v) = v − floor(v) ∈ [0,1)
 *
 *  Triangle center from stripe indices:
 *    py_c = (si + 0.5) × h
 *    px_c = (sj − si) × s/2
 *    Derivation: set n1 = si+0.5 → py_c = (si+0.5)·h
 *                set n2 = sj+0.5 → √3·px_c + py_c = (sj+0.5)·h
 *                → px_c = [(sj+0.5)·h − (si+0.5)·h] / √3
 *                       = (sj−si)·h/√3 = (sj−si)·s/2  (since h = s·√3/2)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • edge_frac must handle NEGATIVE n values. fmod can return negative
 *    results in C for negative inputs. Always add 1.0 if fmod result < 0:
 *    t = fmod(v, 1.0); if (t < 0) t += 1.0;
 *
 *  • "Which character wins" is determined by argmin(d1,d2,d3), not by a
 *    fixed threshold. If border_w is large, many pixels show border chars;
 *    if small, only the line-nearest pixels do.
 *
 *  • Cursor detection uses floor(n1) and floor(n2) only — NOT n3. Two
 *    families suffice to uniquely identify a triangle (the third is
 *    redundant). Using all three would over-constrain and miss some triangles.
 *
 *  • The triangular grid has NO concept of cube coordinates. Do not try to
 *    apply HEX_DIR here — the movement table TRI_DIR works in (si, sj) space.
 *
 * HOW TO VERIFY  (80×24 terminal, TRI_SIZE=20, cursor at si=0, sj=0)
 * ─────────────
 *  h = 20 × √3/2 ≈ 17.32 pixels.
 *
 *  Cursor center: py_c = 0.5 × 17.32 ≈ 8.66 px → row = oy + 2 = 13.
 *                 px_c = (0−0) × 10 = 0 → col = ox = 40.
 *  '@' at (13, 40). ✓
 *
 *  Pixel at center (col=40, row=13): px=0, py=8.66.
 *    n1 = 8.66/17.32 = 0.5 → edge_frac = 0.5 (far from line). Interior ✓
 *    n2 = (0 + 8.66)/17.32 = 0.5 → edge_frac = 0.5. Interior ✓
 *    Both far from borders → dmin = 0.5 > border_w → interior fill drawn. ✓
 *
 *  Pixel at the n1=0 line (py=0, any px):
 *    n1 = 0 → edge_frac(0) = 0 → d1=0 → dmin=0 < border_w → border '-'. ✓
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Sub-pixel resolution per terminal cell (hex/tri stripes work in pixels). */
#define CELL_W              2
#define CELL_H              4

/* Triangle side length in pixels — the lone tunable that sets cell scale. */
#define TRI_SIZE_DEFAULT   20.0
#define TRI_SIZE_MIN        8.0
#define TRI_SIZE_MAX       60.0
#define TRI_SIZE_STEP       4.0

/* Stripe-distance threshold for "on a border line" (normalized [0,0.5]). */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.02
#define BORDER_W_MAX        0.45
#define BORDER_W_STEP       0.02

#define N_THEMES            4
#define TICK_NS            16666667LL    /* ~60 Hz frame budget */

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA      0.05

/* Color pair IDs */
#define PAIR_BORDER   1   /* triangle borders (theme-coloured) */
#define PAIR_CURSOR   2   /* cursor triangle highlight (white-on-blue) */
#define PAIR_HUD      3   /* status bar (yellow)               */
#define PAIR_HINT     4   /* key-hint footer (cyan)            */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { .tv_sec  = (time_t)(ns / 1000000000LL),
                           .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&ts, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEMES[N_THEMES][2] = {
    { COLOR_CYAN,   COLOR_BLACK },
    { COLOR_GREEN,  COLOR_BLACK },
    { COLOR_YELLOW, COLOR_BLACK },
    { COLOR_WHITE,  COLOR_BLACK },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_BORDER, THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_CURSOR, COLOR_WHITE,      COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the three-family stripe classifier            */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the triangular tiling plus cursor bounds.
 *
 * cw/ch are carried as fields (not just #defines) so the rest of the file
 * does not depend on global constants — ctx_to_screen() works for any
 * GridCtx value, the abstraction shared with placement files (see
 * ../README.md). The cursor lives in stripe-index space (si, sj), so the
 * bounds carried here are max_si / max_sj — a different shape from rect's
 * (max_r, max_c) but the same role.
 */
typedef struct {
    /* terminal extent (in cells) */
    int rows, cols;

    /* triangle side length in pixels + sub-pixel cell size */
    double tri_size;
    int    cell_w, cell_h;

    /* centring origin in cell coordinates */
    int ox, oy;

    /* cursor bounds in stripe-index space (loose — large enough that the
     * cursor can roam over any triangle visible in the current terminal) */
    int max_si, max_sj;

    /* render knob — stripe-distance threshold for borders */
    double border_w;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size + tri_size.
 *
 * The grid is centered on screen: ox = cols/2, oy = (rows-1)/2.
 * Stripe bounds are conservative — cover the whole visible terminal in
 * both stripe directions. Out-of-bounds cursor values just sit off-screen,
 * cursor_draw checks before mvaddch.
 */
static void ctx_init(GridCtx *g, int rows, int cols,
                     double tri_size, double border_w)
{
    g->rows = rows; g->cols = cols;
    g->tri_size = tri_size;
    g->cell_w = CELL_W; g->cell_h = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->border_w = border_w;

    /* Loose stripe bounds — the screen is at most cols/CELL_W ≈ 40 pixels
     * wide and rows*CELL_H ≈ 100 tall. Stripe period ≈ tri_size·√3/2.
     * Allow 2× the visible range so cursor can move freely. */
    int max_pix_w = cols * CELL_W;
    int max_pix_h = rows * CELL_H;
    double h = tri_size * sqrt(3.0) * 0.5;
    g->max_si = (int)(max_pix_h / h) + 4;
    g->max_sj = (int)((max_pix_w + max_pix_h) / h) + 4;
}

/*
 * ctx_to_screen — center cell of triangle (si, sj) in screen coordinates.
 *
 * THE FORMULA (triangle center from stripe indices — see KEY FORMULAS):
 *   h    = tri_size × √3 / 2
 *   py_c = (si + 0.5) × h
 *   px_c = (sj − si) × tri_size / 2
 *
 *   sr = oy + (int)(py_c / CELL_H)
 *   sc = ox + (int)(px_c / CELL_W)
 *
 * Reading it: si selects a horizontal stripe row; sj selects how far along
 * that row the triangle's diagonal stripe lands. The (sj − si) term comes
 * from solving the two stripe equations simultaneously.
 *
 * This is the §4 formula for THIS grid family — every triangular file in
 * the series shares this shape, only the inverse classifier in
 * ctx_draw_bg() changes.
 */
static void ctx_to_screen(const GridCtx *g, int si, int sj, int *sr, int *sc)
{
    double h    = g->tri_size * sqrt(3.0) * 0.5;
    double py_c = ((double)si + 0.5) * h;
    double px_c = (double)(sj - si) * g->tri_size * 0.5;
    *sr = g->oy + (int)(py_c / g->cell_h);
    *sc = g->ox + (int)(px_c / g->cell_w);
}

/*
 * edge_frac — distance [0, 0.5] to the nearest integer in v.
 *
 * THE FORMULA:
 *   t = fmod(v, 1.0)       → fractional part in [0,1)
 *   if t < 0: t += 1.0     → handle negative v (C fmod can return negative)
 *   return t < 0.5 ? t : 1.0-t
 *
 *   Result: 0 when v is an integer (on a grid line),
 *           0.5 when v = k+0.5 (farthest from any grid line).
 *
 * This measures the "closeness to a stripe boundary" in normalized units.
 * Comparing this to border_w gives the border/interior classification.
 */
static double edge_frac(double v)
{
    double t = fmod(v, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/*
 * ctx_draw_bg — paint the triangular grid background and cursor highlight.
 *
 * THE PIPELINE (per screen cell):
 *
 *   (col, row)
 *      │
 *      ▼  Center the grid
 *   px = (col−ox)×CELL_W,  py = (row−oy)×CELL_H
 *      │
 *      ▼  Compute three stripe coordinates
 *   n1 = py / h
 *   n2 = (√3·px + py) / h
 *   n3 = (−√3·px + py) / h
 *      │
 *      ▼  Stripe indices for cursor detection
 *   cell_si = floor(n1),  cell_sj = floor(n2)
 *   on_cur  = (cell_si == cur.si && cell_sj == cur.sj)
 *      │
 *      ▼  Edge fractions — distance to nearest line in each family
 *   d1 = edge_frac(n1)   character '-'
 *   d2 = edge_frac(n2)   character '/'
 *   d3 = edge_frac(n3)   character '\'
 *   dmin = min(d1, d2, d3);  ch = character of the winning family
 *      │
 *      ├── dmin < border_w:  draw ch in cursor or border color
 *      │
 *      └── dmin ≥ border_w && on_cur: fill with PAIR_CURSOR background
 *
 * CHARACTER-TO-FAMILY MAPPING:
 *   Family 1 (n1, horizontal lines) → '-'
 *   Family 2 (n2, −60° lines)       → '/'
 *   Family 3 (n3, +60° lines)       → '\'
 * The character direction matches the actual line slope in screen space.
 *
 * Cursor (si, sj) is read from the Cursor passed in — the GridCtx itself
 * does not store cursor state.
 */
static void ctx_draw_bg(const GridCtx *g, int cur_si, int cur_sj)
{
    double h   = g->tri_size * sqrt(3.0) * 0.5;
    double sq3 = sqrt(3.0);

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double n1 = py / h;
            double n2 = (sq3 * px + py) / h;
            double n3 = (-sq3 * px + py) / h;

            /* Stripe indices identify which triangle this cell is in */
            int cell_si = (int)floor(n1);
            int cell_sj = (int)floor(n2);
            int on_cur  = (cell_si == cur_si && cell_sj == cur_sj);

            double d1 = edge_frac(n1);
            double d2 = edge_frac(n2);
            double d3 = edge_frac(n3);

            double dmin = d1;
            char   ch   = '-';
            if (d2 < dmin) { dmin = d2; ch = '/';  }
            if (d3 < dmin) { dmin = d3; ch = '\\'; }

            if (dmin < g->border_w) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
            } else if (on_cur) {
                /* Fill interior of cursor triangle with dim highlight */
                attron(COLOR_PAIR(PAIR_CURSOR));
                mvaddch(row, col, ' ');
                attroff(COLOR_PAIR(PAIR_CURSOR));
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — just (si, sj) in stripe-index space.
 *
 * Bounds live in GridCtx (max_si, max_sj), not here — the cursor doesn't
 * know how big the grid is, just where in it the user is pointing. The two
 * structs compose: Cursor + GridCtx → screen position via ctx_to_screen().
 */
typedef struct { int si, sj; } Cursor;

/*
 * TRI_DIR — 4-direction movement in (si, sj) stripe-index space.
 *
 *                UP: si-1  (one stripe row up — moves py_c by -h)
 *                   ↑
 *   LEFT: sj-1 ← ● → RIGHT: sj+1  (moves px_c by ±s/2)
 *                   ↓
 *               DOWN: si+1
 *
 * WHY THIS WORKS:
 *   si indexes horizontal stripes. si±1 moves to the adjacent stripe row.
 *   This crosses a family-1 line (horizontal edge), moving to the triangle
 *   that shares the horizontal edge.
 *
 *   sj indexes diagonal stripes. sj±1 moves along the n2-family diagonal.
 *   This crosses a family-2 line (60° edge), moving to the triangle that
 *   shares that diagonal edge.
 *
 *   Both directions produce strictly edge-adjacent triangles.
 *
 * WHY NOT USE HEX_DIR:
 *   HEX_DIR applies to axial (Q,R) cube space. Triangular stripe space
 *   (si, sj) is a completely different coordinate system — using HEX_DIR
 *   here would move to the wrong triangles.
 */
static const int TRI_DIR[4][2] = {
    {-1,  0 },   /* UP    — si decreases → row moves up */
    {+1,  0 },   /* DOWN  — si increases */
    { 0, -1 },   /* LEFT  — sj decreases → px_c decreases by s/2 */
    { 0, +1 },   /* RIGHT — sj increases */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->si = 0;
    cur->sj = 0;
}

/*
 * cursor_move — apply (dsi, dsj) and clamp to GridCtx bounds.
 *
 * Arrow-key dispatch reads TRI_DIR[k] and forwards (dsi, dsj) here.
 * Bounds are loose (±max_si/max_sj around 0) — the cursor can move
 * anywhere reachable on screen.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dsi, int dsj)
{
    int nsi = cur->si + dsi;
    int nsj = cur->sj + dsj;
    if (nsi >= -g->max_si && nsi <= g->max_si) cur->si = nsi;
    if (nsj >= -g->max_sj && nsj <= g->max_sj) cur->sj = nsj;
}

/*
 * cursor_draw — place '@' at the centre cell of triangle (si, sj).
 *
 * Delegates the screen-position math to ctx_to_screen(); only checks
 * bounds and emits the bold '@'. Same shape as cursor_draw across every
 * grid file in the series.
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->si, cur->sj, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow status (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Cursor *cur,
                     int theme, int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " stripe si:%+d sj:%+d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->si, cur->sj, g->tri_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur,
                       int theme, int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->si, cur->sj);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();

    double tri_size = TRI_SIZE_DEFAULT;
    double border_w = BORDER_W_DEFAULT;
    int    theme    = 0;
    int    paused   = 0;

    GridCtx g;     ctx_init(&g, LINES, COLS, tri_size, border_w);
    Cursor  cur;   cursor_reset(&cur, &g);
    color_init(theme);

    double  fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, tri_size, border_w);
            cursor_reset(&cur, &g);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'r': cursor_reset(&cur, &g); break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                color_init(theme);
                break;
            /* Arrow keys apply TRI_DIR deltas to stripe cursor */
            case KEY_UP:    cursor_move(&cur, &g, TRI_DIR[0][0], TRI_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, TRI_DIR[1][0], TRI_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, TRI_DIR[2][0], TRI_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, TRI_DIR[3][0], TRI_DIR[3][1]); break;
            case '+': case '=':
                if (tri_size < TRI_SIZE_MAX) {
                    tri_size += TRI_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, tri_size, border_w);
                }
                break;
            case '-':
                if (tri_size > TRI_SIZE_MIN) {
                    tri_size -= TRI_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, tri_size, border_w);
                }
                break;
            case '[':
                if (border_w > BORDER_W_MIN) {
                    border_w -= BORDER_W_STEP;
                    g.border_w = border_w;
                }
                break;
            case ']':
                if (border_w < BORDER_W_MAX) {
                    border_w += BORDER_W_STEP;
                    g.border_w = border_w;
                }
                break;
            }
        }

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0) {
            fps = fps * (1.0 - FPS_EWMA_ALPHA)
                + (1e9 / (double)dt) * FPS_EWMA_ALPHA;
        }

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
