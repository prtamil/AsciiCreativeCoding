/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision.c — flat-top hexagons split into 6 equilateral tris
 *
 * DEMO: A flat-top hex grid, with each hexagon further divided into 6
 *       equilateral triangles by drawing radii from the hex centre to
 *       each vertex (three diagonals through the centre). Arrow keys
 *       move the cursor between hexes; ',' and '.' rotate the cursor
 *       sub-triangle within the hex (sector 0..5, CCW / CW).
 *
 * Study alongside: grids/hex_grids/01_flat_top.c — base hex rasterizer.
 *                  04_30_60_90.c — kis-operation on equilateral triangles
 *                  (this file is the analogous kis-operation on hexagons).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, HEX_SIZE, BORDER_W, RADIUS_T_FRAC,
 *                 FPS_EWMA_ALPHA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / radius / cursor / HUD / HINT
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_pixel_to_hex /
 *                 ctx_draw_bg + hex_centre_pixel + sector_of + angle_char
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_rotate /
 *                 cursor_draw, HEX_DIR
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move hex   ,/. rotate sector   r reset   t theme   p pause
 *        +/- size          [/] border thickness   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/06_hex_subdivision.c \
 *       -o 06_hex_subdivision -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : A regular hexagon contains 6 equilateral triangles —
 *                  three "long diagonals" through its centre pass through
 *                  opposite vertices, dividing the hex into 6 wedges.
 *                  Each wedge is equilateral because hex circumradius
 *                  equals hex edge length. Pixel→hex via the cube-rounding
 *                  trick from hex_grids/01; sector ID via atan2 of
 *                  (Δx, Δy) bucketed into 60° bins.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, hex_size,
 *                  CELL_W/CELL_H, screen origin ox/oy, border_w) and
 *                  Cursor (Q, R, sector). No grid array.
 *
 * Formula        : Hex identification (flat-top inverse, see hex_grids/01):
 *                    fq =  (2/3 · px) / size
 *                    fr = (-px/3 + √3·py/3) / size
 *                    cube_round → integer (Q, R)
 *                  Sector classification within hex (Δ from centre):
 *                    angle = atan2(Δy, Δx)
 *                    sector = ⌊(angle + π/6) / (π/3)⌋ mod 6
 *
 * Edge chars     : Hex edges drawn by angle_char() of the tangent angle
 *                  (same as hex_grids/01). Radii drawn by perpendicular-
 *                  distance test against the three diagonal lines through
 *                  the hex centre, each at 0°, 60°, 120° from horizontal:
 *                    line at  0° → '-'    line at 60° → '/'
 *                    line at 120° → '\\'
 *
 * Movement       : Arrows walk hex by HEX_DIR[4][2] axial deltas (4 of
 *                  the 6 hex faces). Comma/period rotate the sub-sector
 *                  within the current hex (CCW / CW).
 *
 * References     :
 *   Triangular tiling decomposition of hexagon — https://en.wikipedia.org/wiki/Triangular_tiling
 *   Red Blob Games hex guide  — https://www.redblobgames.com/grids/hexagons/
 *   Coxeter, "Regular Polytopes" §4.6
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Combine two analytical formulas. The first identifies WHICH HEX you are
 * in (cube rounding). The second identifies WHICH WEDGE of that hex you
 * are in (atan2 angular bin). Two formulas, no data structure, the entire
 * hex+sub-triangle topology drops out per pixel.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each hexagon has 6 corners. The three "long diagonals" connect opposite
 * corners and all pass through the hex centre. They split the hexagon into
 * 6 equilateral triangles like slices of a pie. To know which slice owns
 * a pixel, just take the pixel's offset from the hex centre and ask "what
 * angle does that point at, measured CCW from due east?". Bucket the
 * angle into 60° bins and you have the sector.
 *
 * For rendering, we paint (a) the hex border using the hex_grids/01 trick
 * (pixels far enough from hex centre are on a border), and (b) the 3
 * diagonals as lines through centre — perpendicular distance to each line
 * gives us the radius character.
 *
 * DRAWING METHOD  (raster scan, the approach used here)
 * ──────────────
 *  1. Pick HEX_SIZE = circumradius of one hex in pixels.
 *  2. Loop every screen cell; convert to centred pixel.
 *  3. Pixel→fractional cube; cube_round → integer (Q, R); compute distance
 *     to nearest hex centre.
 *  4. If distance > 0.5 − BORDER_W: BORDER. Pick angle_char of tangent.
 *     Skip the radius test (we're outside the interior anyway).
 *  5. Otherwise INTERIOR — test 3 radii (perpendicular distance to each
 *     line through hex centre). Smallest distance < RADIUS_T → draw the
 *     radius character.
 *  6. Cursor highlight if (Q, R) matches AND sector matches.
 *
 * KEY FORMULAS
 * ────────────
 *  Flat-top hex inverse (px → fractional cube):
 *    fq =  2/3 · px / size
 *    fr = (-1/3 · px + √3/3 · py) / size
 *    fs = -fq - fr
 *
 *  Cube round (fix the largest-error component):
 *    rq = round(fq), rr = round(fr), rs = round(fs)
 *    if dq largest: rq = -rr - rs
 *    elif dr largest: rr = -rq - rs
 *    else: rs = -rq - rr
 *
 *  Sector of (dx, dy) from hex centre:
 *    angle = atan2(dy, dx)              ∈ [-π, π]
 *    sector = ⌊(angle + π/6) / (π/3)⌋ mod 6
 *
 *  Radii (3 lines through centre, perpendicular distance):
 *    line  0°: |dy|
 *    line 60°: |dy/2 − dx·√3/2|
 *    line 120°: |dy/2 + dx·√3/2|
 *
 *  Sub-triangle centroid pixel (for placing '@'):
 *    angle = sector · 60°
 *    centroid = hex_centre + (size·√3/3)·(cos angle, sin angle)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Sector boundary at angle = ±π is the cusp where atan2 wraps. The
 *    "+ π/6" offset in the bucket formula avoids landing the boundary
 *    exactly on a vertex direction.
 *  • The radius test only fires for interior pixels (after the border
 *    check). Otherwise the diagonals would extend past the hex outline
 *    and bleed into neighbour hexes' borders.
 *  • If the cursor sub-triangle has no border / radius pixel inside its
 *    interior (small HEX_SIZE), we draw a dim '.' marker so the user can
 *    still tell which sector is selected.
 *  • Resize is free — the per-pixel formula doesn't care about screen size.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At hex (Q=0, R=0) the centre is at screen pixel (0, 0) (after centring).
 *  A pixel at (size·√3/3, 0) ≈ (0.577·size, 0) sits on the radius going east
 *  (sector 0). atan2(0, +) = 0 → sector ⌊(0 + π/6)/(π/3)⌋ = 0 ✓.
 *  At (0, -size·√3/3) atan2(-, 0) = -π/2 → sector ⌊(-π/2 + π/6)/(π/3)⌋
 *  = ⌊-1⌋ = -1 → mod 6 = 5. Sector 5 = lower-right wedge. ✓
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

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define HEX_SIZE_DEFAULT 16.0
#define HEX_SIZE_MIN      8.0
#define HEX_SIZE_MAX     40.0
#define HEX_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.30
#define BORDER_W_STEP    0.02

/* Pixel-distance threshold for the three centre-to-vertex diagonals,
 * in fraction of HEX_SIZE — tuned to look about as thick as the border. */
#define RADIUS_T_FRAC 0.12

#define N_THEMES 4

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_RADIUS 2
#define PAIR_CURSOR 3
#define PAIR_HUD    4
#define PAIR_HINT   5

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_FG[N_THEMES][2] = {
    /* edge,  radius */
    {  75,  39 },
    {  82, 226 },
    { 207, 196 },
    {  15,  87 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_BLUE   },
    { COLOR_GREEN,   COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_RED    },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_r;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0];   fg_r = THEME_FG[theme][1];   }
    else               { fg_e = THEME_FG_8[theme][0]; fg_r = THEME_FG_8[theme][1]; }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_RADIUS, fg_r, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx, pixel ↔ cube + sector                            */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active hex-subdivision grid.
 *
 * Same centring scheme as the hex template: ox = cols/2, oy = (rows-1)/2.
 * hex_size and border_w are tunable per frame from the main loop.
 *
 * max_q / max_r are advisory cursor bounds — the hex plane is infinite,
 * so "bounds" here means the largest Q/R that still places its centre on
 * screen given the current hex_size.
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* hex geometry */
    double hex_size;       /* circumradius in pixels                         */
    double border_w;       /* fraction of cube distance that counts as border */
    int    cw, ch;         /* sub-pixel scaling — CELL_W, CELL_H              */

    /* screen origin = pixel (0,0) */
    int    ox, oy;

    /* advisory cursor bounds in axial space */
    int    max_q, max_r;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->cw   = CELL_W;
    g->ch   = CELL_H;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
    if (g->hex_size <= 0.0) g->hex_size = HEX_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_q = (int)((double)cols * CELL_W / (3.0 * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (sqrt(3.0) * g->hex_size)) + 1;
}

/*
 * angle_char — copy from hex_grids/01_flat_top.c.
 *
 * Maps a tangent angle to the best-fit ASCII line character.
 * Folded into [0, π) because line characters are symmetric under 180° flip.
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
 * sector_of — angular bin around the hex centre (returns 0..5).
 *
 * Bins are 60° wide, starting at sector 0 = "right" (toward vertex 0).
 *   sector 0:  −30° ≤ θ <  30°
 *   sector 1:   30° ≤ θ <  90°
 *   sector 2:   90° ≤ θ < 150°
 *   sector 3:  150° ≤ θ ... wraps  (around left vertex)
 *   sector 4:  −150° ≤ θ < −90°
 *   sector 5:  −90° ≤ θ < −30°
 */
static int sector_of(double dx, double dy)
{
    double ang = atan2(dy, dx);
    int s = (int)floor((ang + M_PI / 6.0) / (M_PI / 3.0));
    s %= 6; if (s < 0) s += 6;
    return s;
}

/*
 * ctx_pixel_to_hex — cube round; returns (Q, R) and dist (cube distance).
 *
 * THE FORMULA (flat-top inverse + cube round, see hex_grids/01):
 *
 *   fq = (2/3 · px) / size
 *   fr = (-px/3 + √3·py/3) / size
 *   fs = -fq - fr
 *   round each → restore Q+R+S=0 by fixing the largest-error axis
 */
static void ctx_pixel_to_hex(const GridCtx *g, double px, double py,
                             int *Q, int *R, double *dist)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double fq = (2.0 / 3.0 * px) / g->hex_size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / g->hex_size;
    double fs = -fq - fr;

    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;

    double fQ = (double)*Q, fR = (double)*R, fS = (double)(-*Q - *R);
    double d = fabs(fq - fQ);
    double d2 = fabs(fr - fR);
    double d3 = fabs(fs - fS);
    if (d2 > d) d = d2;
    if (d3 > d) d = d3;
    *dist = d;
}

/*
 * hex_centre_pixel — pure forward map (Q, R) → pixel of hex centre.
 *
 *   cx = size · 3/2 · Q
 *   cy = size · (√3/2 · Q + √3 · R)
 *
 * Pure math helper — keeps its domain name. Used by ctx_to_screen and
 * ctx_draw_bg.
 */
static void hex_centre_pixel(int Q, int R, double size,
                              double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5      * (double)Q;
    *cy = size * (sq3*0.5 * (double)Q + sq3 * (double)R);
}

/*
 * ctx_to_screen — terminal cell of the centroid of sub-triangle (Q, R, sector).
 *
 * Sub-triangle centroid: 2/3 of the way from hex centre to outer-edge
 * midpoint. Outer-edge midpoint at angle (sector·60°), distance size·√3/2.
 * Centroid distance from hex centre = (2/3)·size·√3/2 = size·√3/3.
 */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int sector,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    hex_centre_pixel(Q, R, g->hex_size, &cx_pix, &cy_pix);
    double ang = (double)sector * M_PI / 3.0;
    double r   = g->hex_size * sqrt(3.0) / 3.0;
    double mx  = cx_pix + r * cos(ang);
    double my  = cy_pix + r * sin(ang);
    *sc = g->ox + (int)(mx / g->cw);
    *sr = g->oy + (int)(my / g->ch);
}

/*
 * ctx_draw_bg — raster scan: hex border + 3 radii at every cell.
 *
 * Per cell: cube_round picks the hex; cube distance picks border vs interior.
 * Interior cells run the 3-radius perpendicular-distance test for the
 * sub-triangle separators. Cursor sub-triangle highlighted with PAIR_CURSOR.
 */
static void ctx_draw_bg(const GridCtx *g, int cQ, int cR, int cSector)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double limit_inner = 0.5 - g->border_w;
    double radius_t    = g->hex_size * RADIUS_T_FRAC * 0.5;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int Q, R;
            double dist;
            ctx_pixel_to_hex(g, px, py, &Q, &R, &dist);

            double cx, cy;
            hex_centre_pixel(Q, R, g->hex_size, &cx, &cy);
            double dxp = px - cx, dyp = py - cy;

            int on_cur_hex = (Q == cQ && R == cR);
            int sector     = sector_of(dxp, dyp);
            int on_cur_sec = (on_cur_hex && sector == cSector);

            /* Border */
            if (dist >= limit_inner) {
                double theta = atan2(dyp, dxp);
                char ch = angle_char(theta + M_PI / 2.0);
                int attr = on_cur_sec ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                       : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
                continue;
            }

            /* Interior — test 3 radii */
            double r0 = fabs(dyp);
            double r1 = fabs(0.5 * dyp - sq3_2 * dxp);
            double r2 = fabs(0.5 * dyp + sq3_2 * dxp);
            char rch = '-'; double rmin = r0;
            if (r1 < rmin) { rmin = r1; rch = '/'; }
            if (r2 < rmin) { rmin = r2; rch = '\\'; }
            if (rmin < radius_t) {
                int attr = on_cur_sec ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                       : COLOR_PAIR(PAIR_RADIUS);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)rch);
                attroff(attr);
                continue;
            }

            /* Empty interior — drop a faint marker if cursor sub-triangle */
            if (on_cur_sec) {
                attron(COLOR_PAIR(PAIR_CURSOR));
                mvaddch(row, col, '.');
                attroff(COLOR_PAIR(PAIR_CURSOR));
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — (q, r, sector) where (q, r) is axial hex space and
 * sector ∈ 0..5 picks the CCW wedge inside that hex.
 *
 * Bounds and geometry live in GridCtx, not here — the cursor doesn't know
 * how big the grid is. The two structs compose: Cursor + GridCtx → screen
 * position via ctx_to_screen().
 */
typedef struct { int q, r, sector; } Cursor;

/* HEX_DIR — same as hex_grids/01 (4 of the 6 hex faces). */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q      = 0;
    cur->r      = 0;
    cur->sector = 0;
}

/*
 * cursor_move — apply (dq, dr) in axial space.
 *
 * The hex plane is infinite; we do not clamp here. (max_q/max_r in GridCtx
 * are advisory — visible-extent only.)
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int idx)
{
    (void)g;
    cur->q += HEX_DIR[idx][0];
    cur->r += HEX_DIR[idx][1];
}

static void cursor_rotate(Cursor *cur, const GridCtx *g, int delta)
{
    (void)g;
    cur->sector = (cur->sector + delta + 6) % 6;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->q, cur->r, cur->sector, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d sec:%d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, cur->sector, g->hex_size, g->border_w,
             theme, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:hex  ,/.: sector  +/-:size  [/]:border  t:theme  r:reset  q:quit  [06 hex subdivision] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r, cur->sector);
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
    int theme = 0, paused = 0;
    color_init(theme);

    GridCtx g = {0};
    g.hex_size = HEX_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           paused ^= 1; break;
                case 'r':           cursor_reset(&cur, &g); break;
                case 't':
                    theme = (theme + 1) % N_THEMES;
                    color_init(theme);
                    break;
                case KEY_UP:    cursor_move(&cur, &g, 0); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 1); break;
                case KEY_LEFT:  cursor_move(&cur, &g, 2); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 3); break;
                case ',': case '<': cursor_rotate(&cur, &g, -1); break;
                case '.': case '>': cursor_rotate(&cur, &g, +1); break;
                case '+': case '=':
                    if (g.hex_size < HEX_SIZE_MAX) { g.hex_size += HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '-':
                    if (g.hex_size > HEX_SIZE_MIN) { g.hex_size -= HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '[':
                    if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - t0; t0 = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
