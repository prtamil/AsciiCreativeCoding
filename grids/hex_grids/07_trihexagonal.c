/*
 * 07_trihexagonal.c — trihexagonal tiling: hex grid + triangular grid overlaid
 *
 * DEMO: Two grids drawn simultaneously — the flat-top hexagonal grid (cyan)
 *       and the equilateral triangular grid whose vertices coincide with the
 *       hex vertices (green). Together they approximate the trihexagonal
 *       Archimedean tiling (vertex config 3.6.3.6). Toggle each grid with h/t.
 *       An '@' cursor moves between hex cells with arrow keys; the cursor hex
 *       is highlighted across both visible layers.
 *
 * Study alongside: ../README.md (the GridCtx + Cursor abstraction),
 *                  hex_grids/01_flat_top.c (hex algorithm)
 *                  hex_grids/05_triangular.c (triangle algorithm)
 *
 * Section map:
 *   §1 config   — CELL_W/CELL_H, hex_size + border_w bounds,
 *                 FPS_EWMA_ALPHA, color pair IDs
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — hex / tri / cursor / HUD / HINT pairs
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *                 (calls hex_layer + tri_layer per visibility flags)
 *   §5 cursor   — Cursor (q, r) + cursor_reset / cursor_move / cursor_draw
 *                 + HEX_DIR axial direction table
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  h:hex  t:tri  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids/07_trihexagonal.c \
 *       -o 07_trihexagonal -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : The trihexagonal tiling is Archimedean tiling #3.6.3.6:
 *                  at every vertex, the faces alternate triangle-hex-triangle-hex.
 *                  It can be constructed by superimposing:
 *                    (A) a flat-top hex grid of side s
 *                    (B) a triangular grid whose vertices coincide with the
 *                        hex vertices (triangle side = s, scaled by √3)
 *                  The combination creates the star-of-David / snowflake
 *                  pattern characteristic of the trihexagonal tiling.
 *
 * Data-structure : Two structs — GridCtx (hex_size, border_w, layer toggles,
 *                  cursor bounds in axial space) and Cursor (just (q, r)).
 *                  No grid array; ctx_draw_bg() runs two independent
 *                  rasterizers — hex_layer() and tri_layer() — each
 *                  classifying every screen cell. The toggles show_hex /
 *                  show_tri gate which passes run. See ../README.md "The
 *                  two primitives" for why the GridCtx + Cursor split is
 *                  reused across rect / hex / tri / polar.
 *
 *                  Hex layer: flat-top cube coordinates (01_flat_top algorithm).
 *                  Tri layer: three-family stripe classifier (05_triangular),
 *                             but with tri_size scaled so triangle vertices
 *                             align with hex vertices.
 *
 * Rendering      : Hex borders drawn in PAIR_HEX (cyan). Triangle borders
 *                  drawn in PAIR_TRI (green). Cursor hex drawn in PAIR_CURSOR
 *                  (white-on-blue). Where they share a border cell (at hex-edge
 *                  midpoints), the triangle color wins (drawn last). Toggle
 *                  visibility with 'h' and 't' keys to see each grid
 *                  independently, then combine to observe the interaction.
 *
 * Performance    : Two O(rows×cols) passes per frame. Total ~2× 01_flat_top.
 *                  Still well within 60 fps budget on any modern terminal.
 *
 * References     :
 *   Trihexagonal tiling (Wikipedia)
 *     https://en.wikipedia.org/wiki/Trihexagonal_tiling
 *   Archimedean tilings enumeration
 *     https://en.wikipedia.org/wiki/Archimedean_tiling
 *   Hex–triangle duality
 *     https://www.redblobgames.com/grids/hexagons/#map-storage
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The trihexagonal tiling is the result of overlaying the hexagonal and
 * triangular tilings so their vertices coincide. At every vertex in the
 * combined tiling, exactly 2 triangles and 2 hexagons meet in alternating
 * order (triangle-hex-triangle-hex) — the vertex configuration 3.6.3.6.
 *
 * This file implements the tiling as TWO INDEPENDENT RASTERIZERS that both
 * scan every screen pixel. The hex rasterizer runs first (PAIR_HEX, cyan),
 * then the triangle rasterizer overwrites some pixels (PAIR_TRI, green).
 * The cursor hex is highlighted during the hex pass (PAIR_CURSOR), then the
 * '@' is drawn last on top of everything.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The hex and triangular grids are DUALS of each other in a specific sense:
 *   - Each hexagon center is surrounded by 6 triangle vertices
 *   - Each triangle vertex is shared by 6 triangles AND 6 hexagons (in full)
 *   - The "dual" (Voronoi/Delaunay) relationship: if you place a point at
 *     every hex center and connect adjacent ones, you get a triangular lattice.
 *     Vice versa: connecting triangle centers gives a hex lattice.
 *
 * When both grids are drawn together at the SAME SIZE (tri_size = hex_size),
 * their vertices coincide. The shared vertices are where the edges of both
 * grids meet, creating the star-of-David "snowflake" pattern.
 *
 * Use the 'h' and 't' keys to toggle layers:
 *   h only: pure flat-top hex grid   (see 01_flat_top)
 *   t only: pure triangular grid     (see 05_triangular, but same hex size)
 *   both:   the combined trihexagonal tiling
 *
 * DRAWING METHOD  (two-pass rendering)
 * ─────────────────────────────────────
 *  PASS 1 — hex_layer (if show_hex):
 *   Same pipeline as 01_flat_top ctx_draw_bg.
 *   Cursor hex border drawn in PAIR_CURSOR.
 *   Non-cursor borders drawn in PAIR_HEX (cyan).
 *
 *  PASS 2 — tri_layer (if show_tri):
 *   Same pipeline as 05_triangular ctx_draw_bg, but centered with ox/oy.
 *   tri_size = hex_size so triangle vertices align with hex vertices.
 *   Drawn AFTER hex_layer → triangle edges overwrite hex edges at crossings.
 *   Drawn without cursor awareness (cursor is hex-cell-based, not tri-based).
 *
 *  PASS 3 — cursor_draw:
 *   Draw '@' at the flat-top forward matrix position of (cur.q, cur.r).
 *   Always drawn last so '@' is on top of both grid layers.
 *
 * KEY FORMULAS
 * ────────────
 *  WHY tri_size = hex_size for vertex alignment:
 *    A flat-top hex of side s has vertices at angles 0°,60°,...,300°,
 *    each at distance s from the center.
 *    An equilateral triangle of side s has edge length s.
 *    The triangular grid's stripe period h = s × √3/2 = the hex apothem.
 *    When both grids are drawn with the same s, their vertices coincide
 *    because the triangle vertices lie exactly on the hex vertex lattice.
 *
 *  Draw-order conflict at shared edge midpoints:
 *    Hex edges and triangle edges both pass through the midpoints of
 *    hex sides. These pixels are drawn by BOTH passes. The tri_layer
 *    (drawn second) wins → these midpoints show green not cyan.
 *    This is intentional — the combined pattern shows the triangle edges
 *    "cutting through" the hex edges.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The cursor uses hex coordinates (q, r). If show_hex is OFF, the cursor
 *    still moves through hex space (invisible hex lattice). The '@' is
 *    always drawn; only hex borders are gated by show_hex.
 *
 *  • When both layers are off (h=OFF, t=OFF), only '@' is visible at the
 *    cursor hex center — the grid is blank.
 *
 *  • Triangular stripe alignment: the tri_layer uses px/py centered the same
 *    way as hex_layer (both subtract ox/oy). This keeps both grids centered on
 *    the same origin hex, ensuring vertex coincidence at origin.
 *
 * HOW TO VERIFY  (HEX_SIZE=14, 80×24 terminal)
 * ─────────────
 *  Origin hex (q=0, r=0): hex center at (ox=40, oy=11). '@' at (11,40). ✓
 *
 *  A hex vertex at math 0° from center is at pixel (14, 0):
 *    col = 40 + 14/2 = 47.  That column should show a triangle-grid line.
 *    Triangular n2 = (√3×14 + 0) / (14×√3/2) = (14√3)/(7√3) = 2.0 → integer!
 *    edge_frac(2.0) = 0 → on a triangle border. ✓ The hex vertex and the
 *    triangle grid line coincide at column 47. Vertex alignment confirmed.
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Sub-pixel resolution per terminal cell — math runs in pixel units. */
#define CELL_W              2
#define CELL_H              4

/* Hex side length in pixels; triangle side = hex_size (they share vertices) */
#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

/* Border width for both layers */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35
#define BORDER_W_STEP       0.02

#define TICK_NS            16666667LL    /* ~60 Hz frame budget */

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA      0.05

/* Color pair IDs */
#define PAIR_HEX     1   /* cyan  — hex grid borders */
#define PAIR_TRI     2   /* green — triangle grid borders */
#define PAIR_CURSOR  3   /* cursor hex border + '@' character */
#define PAIR_HUD     4
#define PAIR_HINT    5

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

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_HEX,    COLOR_CYAN,  COLOR_BLACK);
    init_pair(PAIR_TRI,    COLOR_GREEN, COLOR_BLACK);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the dual-layer (hex + tri) classifier         */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the trihexagonal tiling plus cursor bounds.
 *
 * Cursor is axial (q, r) — the same coordinate system as 01_flat_top, so
 * max_q / max_r play the same role as max_r / max_c do in rect's GridCtx.
 *
 * The two visibility toggles (show_hex, show_tri) and border_w live in
 * GridCtx because ctx_draw_bg() reads them directly when deciding which
 * passes to run. Same shape as 06_rhombille's spoke knobs.
 */
typedef struct {
    /* terminal extent (in cells) */
    int rows, cols;

    /* hex side length in pixels + sub-pixel cell size */
    double hex_size;
    int    cell_w, cell_h;

    /* centring origin in cell coordinates */
    int ox, oy;

    /* cursor bounds in axial space (loose — cursor can roam freely) */
    int max_q, max_r;

    /* render knobs */
    double border_w;       /* border threshold for BOTH layers */
    bool   show_hex;
    bool   show_tri;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size + hex_size.
 *
 * The grid is centered on screen: ox = cols/2, oy = (rows-1)/2.
 * Axial bounds are loose — cursor can roam over any hex visible on screen.
 */
static void ctx_init(GridCtx *g, int rows, int cols,
                     double hex_size, double border_w,
                     bool show_hex, bool show_tri)
{
    g->rows = rows; g->cols = cols;
    g->hex_size = hex_size;
    g->cell_w = CELL_W; g->cell_h = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->border_w = border_w;
    g->show_hex = show_hex;
    g->show_tri = show_tri;

    /* Loose axial bounds — cover screen extent in both axes. */
    g->max_q = (int)((cols * CELL_W) / (1.5 * hex_size)) + 2;
    g->max_r = (int)((rows * CELL_H) / (sqrt(3.0) * hex_size)) + 2;
}

/*
 * ctx_to_screen — center cell of flat-top hex (q, r) in screen coordinates.
 *
 * THE FORMULA (flat-top forward matrix, same as 01_flat_top):
 *   cx_pix = size × 3/2 × q
 *   cy_pix = size × (√3/2 × q  +  √3 × r)
 *   sc = ox + (int)(cx_pix / CELL_W)
 *   sr = oy + (int)(cy_pix / CELL_H)
 *
 * Reading it: q increases east (1.5·s per step), r increases south-east
 * (√3·s per step plus a √3/2·s contribution from q). Integer truncation
 * keeps '@' inside the hex interior rather than on a border character.
 */
static void ctx_to_screen(const GridCtx *g, int q, int r, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5      * (double)q;
    double cy_pix = g->hex_size * (sq3_2   * (double)q + sq3 * (double)r);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/*
 * angle_char — same as 01_flat_top. See that file for documentation.
 */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/*
 * edge_frac — same as 05_triangular. See that file for documentation.
 *   0 = on a grid line, 0.5 = farthest from any line.
 */
static double edge_frac(double v)
{
    double t = fmod(v, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/*
 * hex_layer — draw flat-top hex grid borders (PASS 1 of ctx_draw_bg).
 *
 * THE FORMULA (per screen cell):
 *   Identical to 01_flat_top ctx_draw_bg.
 *   Cursor hex (cur_q, cur_r) drawn in PAIR_CURSOR instead of PAIR_HEX.
 *
 * DRAW ORDER NOTE:
 *   hex_layer is drawn FIRST. tri_layer (drawn after) will overwrite some
 *   hex border cells where the two grids share edge-midpoint pixels.
 *   The cursor hex borders drawn in PAIR_CURSOR survive only in regions
 *   not overwritten by the triangle grid.
 */
static void hex_layer(const GridCtx *g, int cur_q, int cur_r)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;
    double size  = g->hex_size;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;

            int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
            double dq = fabs((double)rq - fq);
            double dr = fabs((double)rr - fr);
            double ds = fabs((double)rs - fs);
            if      (dq > dr && dq > ds) rq = -rr - rs;
            else if (dr > ds)             rr = -rq - rs;
            int Q = rq, R = rr;

            double fQ = (double)Q, fR = (double)R, fS = (double)(-Q - R);
            double dist = fabs(fq - fQ);
            double d2   = fabs(fr - fR);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;

            double cx = size * 1.5 * fQ;
            double cy = size * (sq3_2 * fQ + sq3 * fR);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int on_cur = (Q == cur_q && R == cur_r);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_HEX)    | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/*
 * tri_layer — draw the triangular grid borders in PAIR_TRI (PASS 2).
 *
 * THE FORMULA (per screen cell):
 *   Same three-family stripe classifier as 05_triangular ctx_draw_bg.
 *   tri_size = hex_size so triangle vertices coincide with hex vertices.
 *
 *   h = hex_size × √3/2
 *   n1 = py / h              → family 1: horizontal lines at py = k·h
 *   n2 = (√3·px + py) / h   → family 2: −60° lines
 *   n3 = (−√3·px + py) / h  → family 3: +60° lines
 *   dmin = min(edge_frac(n1), edge_frac(n2), edge_frac(n3))
 *   if dmin < border_w: draw '-', '/', or '\' in PAIR_TRI
 *
 * DRAWN AFTER hex_layer — triangle edges overwrite hex edges at crossings,
 * making the triangle edges appear "dominant" at shared boundary points.
 */
static void tri_layer(const GridCtx *g)
{
    /* Triangle side = hex_size; stripe period h = hex_size × √3/2 */
    double h   = g->hex_size * sqrt(3.0) * 0.5;
    double sq3 = sqrt(3.0);

    attron(COLOR_PAIR(PAIR_TRI) | A_BOLD);
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            double n1 = py / h;
            double n2 = (sq3 * px + py) / h;
            double n3 = (-sq3 * px + py) / h;

            double d1 = edge_frac(n1);
            double d2 = edge_frac(n2);
            double d3 = edge_frac(n3);

            double dmin = d1;
            char   ch   = '-';
            if (d2 < dmin) { dmin = d2; ch = '/';  }
            if (d3 < dmin) { dmin = d3; ch = '\\'; }

            if (dmin < g->border_w) {
                mvaddch(row, col, (chtype)(unsigned char)ch);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_TRI) | A_BOLD);
}

/*
 * ctx_draw_bg — paint the trihexagonal tiling background.
 *
 * Runs hex_layer() then tri_layer() per the visibility flags carried in
 * GridCtx. The cursor hex (cur_q, cur_r) is highlighted only inside
 * hex_layer; tri_layer doesn't know about the cursor (the triangles are a
 * visual overlay, not a navigable lattice).
 *
 * Per the project's grid-folder convention, ctx_draw_bg is the single
 * background-painting entry point — its body just dispatches to the
 * per-pass helpers above.
 */
static void ctx_draw_bg(const GridCtx *g, int cur_q, int cur_r)
{
    if (g->show_hex) hex_layer(g, cur_q, cur_r);
    if (g->show_tri) tri_layer(g);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — just (q, r) in axial flat-top hex coordinates.
 *
 * The cursor navigates the hex grid (not the triangle grid — the triangles
 * are just a visual overlay). Bounds live in GridCtx (max_q, max_r).
 */
typedef struct { int q, r; } Cursor;

/*
 * HEX_DIR — 4-direction movement in axial (q, r) space.
 *
 * Same direction table as 01_flat_top:
 *
 *                 UP: (q=0, r=-1)
 *                       ↑
 *   LEFT: (q=-1, r=0) ← ● → RIGHT: (q=+1, r=0)
 *                       ↓
 *                DOWN: (q=0, r=+1)
 *
 * Effect: the cursor hex is highlighted in PAIR_CURSOR on the hex layer.
 * The triangle layer overwrites most border cells of the cursor hex, but
 * the cursor '@' and some hex border pixels remain visible in blue.
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/*
 * cursor_move — apply (dq, dr) and clamp to GridCtx bounds.
 *
 * Arrow-key dispatch reads HEX_DIR[k] and forwards (dq, dr) here.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    int nq = cur->q + dq;
    int nr = cur->r + dr;
    if (nq >= -g->max_q && nq <= g->max_q) cur->q = nq;
    if (nr >= -g->max_r && nr <= g->max_r) cur->r = nr;
}

/*
 * cursor_draw — place '@' at the centre cell of hex (q, r).
 *
 * Called after BOTH layers so '@' is always on top and fully visible,
 * regardless of which layers are toggled. The hex center pixel is interior
 * to both grids, so both hex_layer and tri_layer skip it (they only draw
 * borders) — '@' lands on an empty cell.
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->q, cur->r, &sr, &sc);
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
                     int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  hex:%s  tri:%s  %5.1f fps  %s ",
             cur->q, cur->r, g->hex_size,
             g->show_hex ? "ON " : "OFF",
             g->show_tri ? "ON " : "OFF",
             fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  h:hex  t:tri  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, cur, paused, fps);
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

    double hex_size = HEX_SIZE_DEFAULT;
    double border_w = BORDER_W_DEFAULT;
    bool   show_hex = true;
    bool   show_tri = true;
    int    paused   = 0;

    GridCtx g;     ctx_init(&g, LINES, COLS, hex_size, border_w,
                            show_hex, show_tri);
    Cursor  cur;   cursor_reset(&cur, &g);
    color_init();

    double  fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, hex_size, border_w,
                     show_hex, show_tri);
            cursor_reset(&cur, &g);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'h': show_hex = !show_hex; g.show_hex = show_hex; break;
            case 't': show_tri = !show_tri; g.show_tri = show_tri; break;
            case 'r': cursor_reset(&cur, &g); break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (hex_size < HEX_SIZE_MAX) {
                    hex_size += HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w,
                             show_hex, show_tri);
                }
                break;
            case '-':
                if (hex_size > HEX_SIZE_MIN) {
                    hex_size -= HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w,
                             show_hex, show_tri);
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

        scene_draw(&g, &cur, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
