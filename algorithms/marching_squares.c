/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * marching_squares.c — draw the contour line where a metaball field crosses a
 * threshold, one grid cell at a time. For each 2x2 cell we look at which corners
 * are "inside" and pick a glyph showing how the line cuts through.
 *
 * Marching Squares is the 2-D cousin of Marching Cubes (Lorensen & Cline,
 * SIGGRAPH 1987). The 3-D version lives in raster/marching_cubes.c.
 * Keys: q/ESC quit  space pause  +/- threshold  m multi-level  t theme  r randomise
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/marching_squares.c \
 *        -o marching_squares -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── §1 config — tunable constants ── */

#define N_BLOBS       4        /* how many moving blobs feed the field           */
#define BLOB_R        0.28f    /* each blob's reach in [0,1] coords; at 0.28 they
                                * merge and split as they drift around           */
#define THRESH_DEF    0.20f    /* starting threshold; lower encloses more area    */
#define THRESH_STEP   0.02f
#define THRESH_MIN    0.02f
#define THRESH_MAX    0.95f
#define RENDER_FPS    20       /* slower than a plain cellular automaton: each
                                * frame re-evaluates the field at every corner   */
#define RENDER_NS    (1000000000LL / RENDER_FPS)
#define N_THEMES      4
#define N_LEVELS      5        /* contour lines drawn at once in multi mode       */

/* Terminal cells are about twice as tall as wide. Squashing x by half before we
 * sample the field makes round blobs actually look round on screen. */
#define ASPECT        0.5f

/* Upper bound on the per-frame sample buffer. Holds any terminal up to 255x127
 * cells; storage is (FIELD_W_MAX+1)*(FIELD_H_MAX+1) floats, roughly 128 KB. */
#define FIELD_W_MAX   255
#define FIELD_H_MAX   127

/* One row reserved top and bottom for the two status bars. The field is drawn
 * in between; everything offsets by HUD_TOP_ROWS to clear the top bar. */
#define HUD_TOP_ROWS  1
#define HUD_BOT_ROWS  1

/* ── §2 clock — monotonic time + sleep ── */

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

/* ── §3 data types — Blob, FieldGrid, Scene ── */

/*
 * Blob — one moving lump that pushes up the field around it. The blobs are the
 * input: at each grid corner the field is the sum of every blob's bump, and the
 * contour is traced from that field. Positions are in [0,1] so the picture looks
 * the same at any terminal size — resizing just re-samples the same field.
 *
 * Members:
 *   x, y       position, normalised to [0, 1]
 *   vx, vy     velocity per frame (also normalised, around 0.005)
 *   strength   unused; the bump is hard-coded to peak at 1. A hook left in case
 *              you ever want to weight blobs differently.
 */
typedef struct {
    float x, y;        /* position, [0, 1] coords        */
    float vx, vy;      /* velocity per frame             */
    float strength;    /* per-blob weight (unused)       */
} Blob;

/*
 * FieldGrid — the field's value at every grid corner, filled once per frame.
 * We sample the field once and reuse it: the marching loop reads sample[][]
 * over and over, and in multi-level mode all N_LEVELS contours share the same
 * samples instead of re-evaluating the (expensive) field. It is throwaway
 * scratch — nothing here carries over between frames, which is why it lives
 * outside Scene. Kept as a 128 KB static (not malloc'd) since the max size is
 * known up front; too big for the stack, trivial in BSS.
 *
 * Members:
 *   cols     cells wide this frame (corners run 0..cols, so cols+1 of them)
 *   rows     cells tall this frame (corners run 0..rows)
 *   sample   field value per corner, row-major; sample[0..rows][0..cols] valid
 *
 * Invariants: 0 <= cols <= FIELD_W_MAX, 0 <= rows <= FIELD_H_MAX. After
 * field_sample_grid returns, only the [0..rows][0..cols] region is meaningful;
 * anything outside it is stale.
 */
typedef struct {
    int   cols;
    int   rows;
    float sample[FIELD_H_MAX + 1][FIELD_W_MAX + 1];
} FieldGrid;

/*
 * Scene — everything that survives from one frame to the next. One instance
 * lives in main() and is passed by pointer everywhere, so there are no global
 * mutables (the signal flags in §7 are the exception — handlers can't take a
 * pointer). Per-frame scratch lives in FieldGrid, not here.
 *
 * Members:
 *   blobs[N_BLOBS]   the moving blobs (the simulation itself)
 *   rows, cols       terminal size in cells, refreshed each frame so resize
 *                    is picked up
 *   threshold        the field value we trace the contour at
 *   paused           freeze blob motion (toggle: SPACE)
 *   multi            draw N_LEVELS nested contours at once (toggle: 'm')
 *   theme            colour theme, 0..N_THEMES (cycle: 't')
 */
typedef struct {
    Blob   blobs[N_BLOBS];
    int    rows, cols;
    float  threshold;
    bool   paused;
    bool   multi;
    int    theme;
} Scene;

/* ── §4 scalar field — blob motion + field sampling ── */

/*
 * Scatter the blobs to random spots and give them random velocities. Called at
 * startup and on 'r'. Positions stay in [0.2, 0.8] so blobs start visible and
 * away from the walls; speeds are 0.003..0.007 with a random direction.
 */
static void blobs_init(Scene *sc)
{
    for (int i = 0; i < N_BLOBS; i++) {
        sc->blobs[i].x        = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        sc->blobs[i].y        = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        sc->blobs[i].vx       = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        sc->blobs[i].vy       = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        sc->blobs[i].strength = 1.0f;
        if (rand() & 1) sc->blobs[i].vx = -sc->blobs[i].vx;
        if (rand() & 1) sc->blobs[i].vy = -sc->blobs[i].vy;
    }
}

/* Move one blob forward by its velocity. No acceleration — constant speed
 * between wall bounces. */
static inline void integrate_blob_euler(Blob *b)
{
    b->x += b->vx;
    b->y += b->vy;
}

/* Bounce a blob off the [0.05, 0.95] box: cross a wall, flip that axis'
 * velocity. No friction or overshoot correction — the step (~0.005) is tiny
 * next to the box, so it never sticks past a wall noticeably. */
static inline void bounce_blob_off_walls(Blob *b)
{
    if (b->x < 0.05f || b->x > 0.95f) b->vx = -b->vx;
    if (b->y < 0.05f || b->y > 0.95f) b->vy = -b->vy;
}

/* Move every blob one frame: step it, then bounce it off the walls. */
static void blobs_step(Scene *sc)
{
    for (int i = 0; i < N_BLOBS; i++) {
        integrate_blob_euler(&sc->blobs[i]);
        bounce_blob_off_walls(&sc->blobs[i]);
    }
}

/*
 * How much one blob is "felt" at squared distance r2 — a bump that peaks at 1
 * in the middle and falls smoothly to exactly 0 at radius R, nothing beyond.
 * The hard zero outside R means no faint tails to clean up, and the peak of 1
 * keeps the summed field in a tidy [0, N_BLOBS] range. Takes r2 (not r) so the
 * caller skips a sqrt. (Wyvill, McPheeters & Wyvill 1986, "Data structure for
 * soft objects", The Visual Computer 2(4).)
 */
static inline float wyvill_kernel(float r2, float R2)
{
    if (r2 >= R2) return 0.0f;
    float t = 1.0f - r2 / R2;
    return t * t * t;
}

/*
 * One blob's value at point (nx, ny). The only place screen aspect touches the
 * field math: we shrink the vertical gap by ASPECT before measuring distance,
 * so a round blob reads round on tall terminal cells instead of stretched.
 */
static inline float blob_contribution_at(const Blob *b, float nx, float ny)
{
    float dx = nx - b->x;
    float dy = (ny - b->y) * ASPECT;
    float r2 = dx*dx + dy*dy;
    return wyvill_kernel(r2, BLOB_R * BLOB_R);
}

/*
 * The field at a point is just the sum of every blob's value there. Called once
 * per grid corner per frame, so the helpers are inline to keep it tight.
 */
static float field_eval_at(const Blob *blobs, int n, float nx, float ny)
{
    float v = 0.0f;
    for (int i = 0; i < n; i++)
        v += blob_contribution_at(&blobs[i], nx, ny);
    return v;
}

/*
 * Work out how big the field is this frame: full terminal width, height minus
 * the two HUD rows, then clamp both to the buffer's fixed max. If the window is
 * too small to march, set the extent to zero so march_cells just skips.
 */
static void init_field_extent(const Scene *sc, FieldGrid *field)
{
    int cols = sc->cols;
    int rows = sc->rows - HUD_TOP_ROWS - HUD_BOT_ROWS;
    if (cols > FIELD_W_MAX) cols = FIELD_W_MAX;
    if (rows > FIELD_H_MAX) rows = FIELD_H_MAX;
    if (cols < 2 || rows < 2) { field->cols = 0; field->rows = 0; return; }
    field->cols = cols;
    field->rows = rows;
}

/*
 * Fill in the field value at every grid corner. This is the expensive part of
 * the frame (one field evaluation per corner); everything after just reads the
 * stored numbers. Corners map to [0,1] so the grid covers the whole field no
 * matter the window size.
 */
static void sample_all_corners(const Scene *sc, FieldGrid *field)
{
    for (int gy = 0; gy <= field->rows; gy++) {
        float ny = (float)gy / (float)field->rows;
        for (int gx = 0; gx <= field->cols; gx++) {
            float nx = (float)gx / (float)field->cols;
            field->sample[gy][gx] = field_eval_at(sc->blobs, N_BLOBS, nx, ny);
        }
    }
}

/* Decide the grid size, then sample the field across it (skip if too small). */
static void field_sample_grid(const Scene *sc, FieldGrid *field)
{
    init_field_extent(sc, field);
    if (field->cols == 0 || field->rows == 0) return;
    sample_all_corners(sc, field);
}

/* ── §5 marching squares — classify each cell, look up a glyph ── */

/*
 * How a cell is named. Each 2x2 cell has four corners; a corner is "inside" if
 * its field value is above the threshold. Pack the four inside/outside bits into
 * one number 0..15 — that names one of the 16 possible cases:
 *
 *   TL ─────── TR        bit 3 = TL inside
 *    |          |        bit 2 = TR inside
 *    |          |        bit 1 = BR inside
 *   BL ─────── BR        bit 0 = BL inside
 *
 * Cases 0 (all out) and 15 (all in) have no line through them. Cases 5 and 10
 * are "saddles" — opposite corners in, the line could go two ways; we pick one
 * and stick with it. (This is the flat version of Lorensen & Cline's 8-corner
 * cube cases, Marching Cubes, SIGGRAPH 1987.)
 */

/*
 * For each of the 16 cases, the one ASCII glyph that best shows how the contour
 * cuts through the cell. We draw a glyph per cell rather than interpolating real
 * line endpoints, so the glyph's shape stands in for the line direction:
 *   '-' cuts left-right,  '|' cuts top-bottom,  '/' and '\' the diagonals,
 *   'X' a saddle (two crossings),  ' ' no line.
 * (raster/marching_cubes.c keeps an edge list instead and draws real segments.)
 */
static const char case_glyph[16] = {
    ' ',    /* 0000 — all outside              — no contour              */
    '/',    /* 0001  BL inside                  — left + bottom edges    */
    '\\',   /* 0010  BR inside                  — bottom + right edges   */
    '-',    /* 0011  BL + BR inside             — left + right edges     */
    '\\',   /* 0100  TR inside                  — right + top edges      */
    'X',    /* 0101  BL + TR (saddle)           — two crossings          */
    '|',    /* 0110  TR + BR inside             — top + bottom edges     */
    '/',    /* 0111  all but TL                 — top + left edges       */
    '/',    /* 1000  TL inside                  — top + left edges       */
    '|',    /* 1001  TL + BL inside             — top + bottom edges     */
    'X',    /* 1010  TL + BR (saddle)           — two crossings          */
    '\\',   /* 1011  all but TR                 — right + top edges      */
    '-',    /* 1100  TL + TR inside             — left + right edges     */
    '\\',   /* 1101  all but BR                 — bottom + right edges   */
    '/',    /* 1110  all but BL                 — left + bottom edges    */
    ' ',    /* 1111 — all inside                — no contour             */
};

/*
 * CellCorners — the four field values at one cell's corners. This is the input
 * to the case lookup. The naming order (top-left, top-right, bottom-right,
 * bottom-left) is fixed because cell_case_index reads it to build the bits.
 * Members tl, tr, br, bl: field value at that corner.
 */
typedef struct { float tl, tr, br, bl; } CellCorners;

/* Pull the four corner values for cell (gx, gy) out of the sampled grid and
 * label them tl/tr/br/bl so the next step doesn't repeat the index juggling. */
static inline CellCorners read_cell_corners(const FieldGrid *field,
                                            int gx, int gy)
{
    return (CellCorners){
        .tl = field->sample[gy  ][gx  ],
        .tr = field->sample[gy  ][gx+1],
        .br = field->sample[gy+1][gx+1],
        .bl = field->sample[gy+1][gx  ],
    };
}

/* Turn the four corners into a case number 0..15: one bit per corner, set if
 * that corner is above the threshold. This single number drives the whole
 * algorithm — it indexes case_glyph[]. */
static inline int cell_case_index(CellCorners c, float thresh)
{
    return ((c.tl > thresh) ? 8 : 0)
         | ((c.tr > thresh) ? 4 : 0)
         | ((c.br > thresh) ? 2 : 0)
         | ((c.bl > thresh) ? 1 : 0);
}

/* ── §6 drawing — colours + per-frame render ── */

/*
 * Colour-pair slots:
 *   CP_HUD / CP_HINT      the two status bars (bright yellow / cyan, bold so
 *                         they stay readable over any field cells behind them)
 *   CP_CONTOUR / _INSIDE  the contour line and the interior fill, single mode
 *   CP_OUTSIDE            unused grey, kept for symmetry
 *   CP_LEVEL_BASE + i     one colour per contour level in multi mode
 * All set once in init_theme_colors, never inside the draw loop.
 */
enum {
    CP_HUD        = 1,     /* top data bar       — bright yellow + A_BOLD */
    CP_HINT       = 2,     /* bottom action bar  — bright cyan   + A_BOLD */
    CP_CONTOUR    = 3,
    CP_INSIDE     = 4,
    CP_OUTSIDE    = 5,
    CP_LEVEL_BASE = 10,    /* CP_LEVEL_BASE + i  for i ∈ [0, N_LEVELS)    */
};

/* Per-theme colours: the contour line colour and the inside-fill colour.
 * In multi mode each level shifts the contour colour by 6 steps. */
static const short theme_contour[N_THEMES] = { 51, 196, 46, 201 };
static const short theme_inside [N_THEMES] = { 87, 202, 82, 171 };

/* Set every colour pair for the chosen theme at once. Called at startup and on
 * 't'. Doing it here keeps init_pair out of the per-cell draw loop. */
static void init_theme_colors(int theme)
{
    init_pair(CP_HUD,     226,                  16);  /* bright yellow */
    init_pair(CP_HINT,     51,                  16);  /* bright cyan   */
    init_pair(CP_CONTOUR, theme_contour[theme], 16);
    init_pair(CP_INSIDE,  theme_inside [theme], 16);
    init_pair(CP_OUTSIDE, 244,                  16);
    for (int lv = 0; lv < N_LEVELS; lv++) {
        short col = (short)(theme_contour[theme] + lv * 6);
        init_pair((short)(CP_LEVEL_BASE + lv), col, 16);
    }
}

/*
 * Draw one cell for a single threshold: blank if fully outside, a '.' fill if
 * fully inside, otherwise the contour glyph for whichever way the line crosses.
 */
static void draw_cell_single(int gx, int gy, CellCorners c, float threshold)
{
    int idx = cell_case_index(c, threshold);
    if (idx == 0)  return;                       /* fully outside — blank */
    if (idx == 15) {                             /* fully inside — fill   */
        attron(COLOR_PAIR(CP_INSIDE));
        mvaddch(gy, gx, '.');
        attroff(COLOR_PAIR(CP_INSIDE));
    } else {                                     /* contour crossing      */
        attron(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
        mvaddch(gy, gx, case_glyph[idx]);
        attroff(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
    }
}

/*
 * Draw one cell for several thresholds at once, like nested contour lines on a
 * map. Walk the levels from outer to inner; the first level whose line passes
 * through this cell wins and gets drawn in its own colour. If no line passes
 * here but the cell is inside the outermost contour, drop a faint fill mark.
 */
static void draw_cell_multi(int gx, int gy, CellCorners c, float base_threshold)
{
    for (int lv = 0; lv < N_LEVELS; lv++) {
        float t = base_threshold * (0.3f + lv * 0.15f);
        int idx = cell_case_index(c, t);
        if (idx != 0 && idx != 15) {
            short cp = (short)(CP_LEVEL_BASE + lv);
            attron(COLOR_PAIR(cp));
            mvaddch(gy, gx, case_glyph[idx]);
            attroff(COLOR_PAIR(cp));
            return;
        }
    }
    /* No line crossed here. If the cell is inside the outermost contour, fill
       it faintly. The TL corner stands in for the cell: with no crossing, all
       four corners are on the same side of the threshold. */
    if (c.tl > base_threshold) {
        attron(COLOR_PAIR(CP_INSIDE));
        mvaddch(gy, gx, '`');
        attroff(COLOR_PAIR(CP_INSIDE));
    }
}

/* The "marching": loop over every cell and hand it to the single- or
 * multi-level drawer. The grid is offset down by HUD_TOP_ROWS to clear the top
 * bar. */
static void march_cells(const Scene *sc, const FieldGrid *field)
{
    for (int gy = 0; gy < field->rows; gy++) {
        for (int gx = 0; gx < field->cols; gx++) {
            CellCorners c   = read_cell_corners(field, gx, gy);
            int  screen_row = gy + HUD_TOP_ROWS;       /* slip below top HUD */
            if (sc->multi)
                draw_cell_multi (gx, screen_row, c, sc->threshold);
            else
                draw_cell_single(gx, screen_row, c, sc->threshold);
        }
    }
}

/* Top bar: shows the current threshold, mode, theme and pause state, so a key
 * press visibly does something. */
static void draw_hud_top(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
             " Marching Squares  thresh=%.2f  mode:%-6s  theme:%d/%d  %s ",
             sc->threshold,
             sc->multi ? "multi" : "single",
             sc->theme, N_THEMES,
             sc->paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom bar: the key hints. Lists every key handle_input responds to. */
static void draw_hud_bottom(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  +/-:thresh  m:multi  t:theme  r:random ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* Render one frame: sample the field, march the cells, draw the two HUD bars,
 * then flush to the terminal. */
static void scene_draw(const Scene *sc, FieldGrid *field)
{
    field_sample_grid(sc, field);
    march_cells(sc, field);
    draw_hud_top(sc);
    draw_hud_bottom(sc);
    refresh();
}

/* ── §7 app — signals, input, main loop ── */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;
static void on_signal(int s) { (void)s; g_quit   = 1; }
static void on_resize(int s) { (void)s; g_resize = 1; }

/* React to one key. 'q'/ESC route through g_quit so they exit the same way as a
 * SIGINT/SIGTERM. 't' also re-sets the colour pairs for the new theme. */
static void handle_input(Scene *sc, int ch)
{
    switch (ch) {
        case 'q': case 27: g_quit = 1; break;
        case ' ': sc->paused = !sc->paused; break;
        case '+': case '=':
            sc->threshold += THRESH_STEP;
            if (sc->threshold > THRESH_MAX) sc->threshold = THRESH_MAX;
            break;
        case '-':
            sc->threshold -= THRESH_STEP;
            if (sc->threshold < THRESH_MIN) sc->threshold = THRESH_MIN;
            break;
        case 'm': sc->multi = !sc->multi; break;
        case 't':
            sc->theme = (sc->theme + 1) % N_THEMES;
            init_theme_colors(sc->theme);
            break;
        case 'r': blobs_init(sc); break;
    }
}

/* Own the Scene and the FieldGrid scratch, then run the loop at RENDER_FPS.
 * When there's time before the next frame we sleep instead of spinning.
 * The FieldGrid is a static (BSS) here, not on the stack: it's ~128 KB of
 * per-frame scratch, too big for the stack and not worth a malloc. */
int main(void)
{
    srand((unsigned)time(NULL));
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_resize);

    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    start_color();
    use_default_colors();

    Scene scene = {
        .threshold = THRESH_DEF,
        .paused    = false,
        .multi     = false,
        .theme     = 0,
        .rows      = LINES,
        .cols      = COLS,
    };
    static FieldGrid field;          /* BSS — ~128 KB per-frame scratch */

    init_theme_colors(scene.theme);
    blobs_init(&scene);

    long long next = clock_ns();

    while (!g_quit) {
        if (g_resize) { g_resize = 0; endwin(); refresh(); }

        scene.rows = LINES;
        scene.cols = COLS;

        int ch = getch();
        handle_input(&scene, ch);

        long long now = clock_ns();
        if (now >= next) {
            if (!scene.paused) blobs_step(&scene);
            erase();
            scene_draw(&scene, &field);
            next += RENDER_NS;
        } else {
            clock_sleep_ns(next - now);
        }
    }

    endwin();
    return 0;
}
