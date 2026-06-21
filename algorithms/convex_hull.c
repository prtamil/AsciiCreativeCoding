/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * convex_hull.c — two classic convex-hull algorithms racing side by side.
 *
 * Left panel runs Graham scan (sort by angle, then sweep with a stack);
 * right panel runs Jarvis march / gift-wrapping (wrap a string around the
 * points). Both chew on the same 40 random points, one visible step per tick.
 *
 * Graham, R. L. (1972), Info. Processing Letters 1(4) 132-133.
 * Jarvis, R. A. (1973), Info. Processing Letters 2(1) 18-21.
 * O'Rourke, "Computational Geometry in C" (1998) §3.5 — reference C code.
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/convex_hull.c \
 *            -o convex_hull -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — point count, frame/step timing, color-pair ids ── */

#define N_POINTS     40
#define HUD_ROWS      2                    /* top rows reserved for the HUD */
#define RENDER_NS    (1000000000LL / 30)   /* draw the screen 30 times/sec */
#define STEP_NS      (1000000000LL / 6)    /* advance algorithms 6 steps/sec */

/* Color-pair slots: dots, Graham hull, Jarvis hull, the moving ray, HUD text. */
enum { CP_PTS=1, CP_GR, CP_JA, CP_CUR, CP_HUD };

/* ── §2 clock — monotonic nanosecond timer + sleep ── */

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

/* ── §3 color — one color pair per visual role ── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_PTS, 244, -1);   /* grey  — points */
        init_pair(CP_GR,   51, -1);   /* cyan  — Graham */
        init_pair(CP_JA,   46, -1);   /* green — Jarvis */
        init_pair(CP_CUR, 226, -1);   /* yellow — current */
        init_pair(CP_HUD, 244, -1);
    } else {
        init_pair(CP_PTS, COLOR_WHITE,  -1);
        init_pair(CP_GR,  COLOR_CYAN,   -1);
        init_pair(CP_JA,  COLOR_GREEN,  -1);
        init_pair(CP_CUR, COLOR_YELLOW, -1);
        init_pair(CP_HUD, COLOR_WHITE,  -1);
    }
}

/* ── §4 algorithms — Graham scan + Jarvis march, one step at a time ── */

/*
 * Point — one input dot, in screen-pixel coordinates.
 *   x: column, grows rightward.
 *   y: row, grows DOWNWARD (screen convention, not math).
 *
 * The algorithms keep arrays of INDICES into the point list, not copies of
 * the points, so both can share one canonical list and stay in sync.
 *
 * Heads-up on the flipped y: because rows count downward, a turn that the
 * math calls "counter-clockwise" looks clockwise on screen. The hull still
 * comes out right, because every test only cares that the SIGN is used
 * consistently — not which way is "really" CCW.
 *
 * Floats (not ints) because new_points() scatters dots with fractional
 * positions; keeping them float lets the turn test stay as precise as the
 * generator made them, and matches O'Rourke's reference code.
 * Caveat: with plain floats the turn test can misjudge points that are
 * almost in a straight line (rounding flips a near-zero result). Real
 * production code uses exact/robust arithmetic; this demo accepts the risk
 * (see Shewchuk 1997).
 */
typedef struct {
    float x;   /* column (rightward) */
    float y;   /* row (downward) */
} Point;

/*
 * Scene — every piece of state for one run, passed by pointer everywhere.
 * It's one struct (not loose globals) so each function's signature shows
 * exactly what it reads and writes. Fields are grouped by who uses them.
 * (The two signal flags in §6 must stay global — a signal handler can't be
 * handed a pointer.)
 */
typedef struct {
    /* Shared input — both algorithms read these */
    Point     pts[N_POINTS];      /* the random dots being wrapped */
    int       rows, cols;         /* terminal size in character cells */

    /* Graham scan working state */
    int       gs_sorted[N_POINTS];/* dot indices in angle order around pivot */
    int       gs_stack [N_POINTS];/* current hull candidates; hull = [0..sp-1] */
    int       gs_sp;              /* how many entries are on the stack */
    int       gs_idx;             /* next sorted dot to feed in */
    bool      gs_done;            /* true once every dot has been processed */
    long long gs_steps;           /* dots pushed so far (for the HUD counter) */
    int       pivot;              /* lowest-then-leftmost dot — Graham's anchor */

    /* Jarvis march working state */
    int       jv_hull[N_POINTS + 1]; /* hull vertices in order; +1 to close loop */
    int       jv_n;               /* hull vertices found so far */
    int       jv_cur;             /* the vertex we're currently wrapping from */
    int       jv_cand;            /* dot being weighed in this comparison */
    int       jv_best;            /* best (most-CCW) candidate seen this pass */
    bool      jv_done;            /* true once the wrap returns to the start */
    long long jv_steps;           /* comparisons so far (for the HUD counter) */
    int       jv_start;           /* leftmost dot — where the wrap begins/ends */

    /* Playback state */
    bool      paused;             /* 'p' toggles this */
    long long step_accum;         /* leftover time, ns, in the fixed-step loop */
    long long step_ns;            /* ns between algorithm steps; +/- changes it */
} Scene;

/*
 * cross2 — stand at O, look toward A, then ask: which side is B on?
 *   This one number is the whole engine of the file. Its sign tells you the
 *   turn direction of the path O -> A -> B:
 *       positive -> turns left  (counter-clockwise)
 *       negative -> turns right (clockwise)
 *       zero     -> dead straight (O, A, B on one line)
 *   Both algorithms boil down to asking this over and over.
 *   (The value also happens to be twice the signed area of triangle OAB;
 *   O'Rourke §1.5 derives it.)
 */
static float cross2(const Point *O, const Point *A, const Point *B)
{
    return (A->x - O->x)*(B->y - O->y) - (A->y - O->y)*(B->x - O->x);
}

/*
 * is_strict_left_turn — does O -> A -> B turn clearly to the left?
 *   Dead-straight (collinear) counts as NO: a hull edge shouldn't bend
 *   through a point that sits exactly on it.
 */
static inline bool is_strict_left_turn(const Point *O, const Point *A,
                                       const Point *B)
{
    return cross2(O, A, B) > 0.0f;
}

/*
 * is_not_strict_left_turn — the opposite question: did O -> A -> B fail to
 *   turn left? (right turn OR straight). Graham's inner loop pops the stack
 *   while this is true. Straight counts as a fail on purpose: a point sitting
 *   on a hull edge adds nothing, so we drop it.
 */
static inline bool is_not_strict_left_turn(const Point *O, const Point *A,
                                           const Point *B)
{
    return cross2(O, A, B) <= 0.0f;
}

/*
 * polar_angle — the compass bearing from ref to p, in radians.
 *   Graham sorts the dots by this so the scan walks them in a clean
 *   sweep around the pivot. atan2 handles all directions without the
 *   usual divide-by-zero corner cases.
 */
static float polar_angle(const Point *ref, const Point *p)
{
    return atan2f(p->y - ref->y, p->x - ref->x);
}

/*
 * squared_distance — how far apart two dots are, but skipping the sqrt.
 *   We only ever compare distances, and sqrt keeps the same order, so we
 *   leave it out and save the work.
 */
static inline float squared_distance(const Point *a, const Point *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    return dx*dx + dy*dy;
}

/* C's qsort can't pass extra data to the comparator, so cmp_angle reaches the
 * Scene through this pointer. Set it right before qsort, clear it right after.
 * Single-threaded, so no locking needed. */
static const Scene *cmp_angle_ctx;

/*
 * cmp_angle — sort order for Graham: by bearing around the pivot, and when
 *   two dots share a bearing, the nearer one first. (The pop loop would throw
 *   out the inner ones anyway; near-first just keeps the walk tidy.)
 */
static int cmp_angle(const void *a, const void *b)
{
    const Scene *sc = cmp_angle_ctx;
    int ia = *(const int*)a, ib = *(const int*)b;
    const Point *pivot = &sc->pts[sc->pivot];

    float da = polar_angle(pivot, &sc->pts[ia]);
    float db = polar_angle(pivot, &sc->pts[ib]);
    if (da < db) return -1;
    if (da > db) return  1;

    /* same bearing — break the tie by distance from the pivot */
    float la = squared_distance(pivot, &sc->pts[ia]);
    float lb = squared_distance(pivot, &sc->pts[ib]);
    return (la < lb) ? -1 : (la > lb) ? 1 : 0;
}

/*
 * find_lowest_leftmost_point — Graham's anchor: lowest dot, ties broken by
 *   leftmost. A bottom-edge dot is always on the hull, so it's a safe,
 *   known starting vertex to sort everything else around.
 */
static int find_lowest_leftmost_point(const Point *pts, int n)
{
    int idx = 0;
    for (int i = 1; i < n; i++) {
        bool lower      = pts[i].y < pts[idx].y;
        bool same_y_lft = pts[i].y == pts[idx].y && pts[i].x < pts[idx].x;
        if (lower || same_y_lft) idx = i;
    }
    return idx;
}

/*
 * sort_indices_by_polar_angle_around_pivot — list every dot's index, then
 *   sort the list so walking it sweeps around the pivot in order. This sort
 *   is the slow part of Graham (the rest is a quick single pass).
 */
static void sort_indices_by_polar_angle_around_pivot(const Scene *sc,
                                                     int *idx_out, int n)
{
    for (int i = 0; i < n; i++) idx_out[i] = i;
    cmp_angle_ctx = sc;
    qsort(idx_out, (size_t)n, sizeof(int), cmp_angle);
    cmp_angle_ctx = NULL;
}

/*
 * graham_init — get Graham ready to run one step at a time.
 *   Pick the anchor, sort the dots around it, seed the stack with the first
 *   two (the anchor and its neighbour always form a valid first edge), and
 *   point the scan at the third. graham_step takes it from there.
 */
static void graham_init(Scene *sc)
{
    sc->pivot = find_lowest_leftmost_point(sc->pts, N_POINTS);

    sort_indices_by_polar_angle_around_pivot(sc, sc->gs_sorted, N_POINTS);

    /* first two sorted dots are trivially a good edge — seed the stack */
    sc->gs_stack[0] = sc->gs_sorted[0];
    sc->gs_stack[1] = sc->gs_sorted[1];
    sc->gs_sp       = 2;

    sc->gs_idx = 2;          /* scan begins at the third dot */

    sc->gs_done  = false;
    sc->gs_steps = 0;
}

/*
 * pop_while_not_strict_left_turn — Graham's clean-up step. Before adding the
 *   new dot, throw away any recent stack dots that would put a dent (a right
 *   turn) in the outline. We keep dropping the top until the last two dots
 *   plus the newcomer bend the correct way.
 *   Each dot is added once and dropped at most once, so this stays cheap
 *   overall even when one call drops several.
 */
static void pop_while_not_strict_left_turn(const Point *pts,
                                           int *stack, int *sp,
                                           int candidate_idx)
{
    while (*sp >= 2 &&
           is_not_strict_left_turn(&pts[stack[*sp - 2]],
                                   &pts[stack[*sp - 1]],
                                   &pts[candidate_idx]))
        (*sp)--;
}

/*
 * graham_step — fold in ONE more dot, so the viewer sees the scan advance.
 *   Drop any dots that would dent the outline, push the new one, move on.
 *   When the last dot is in, the stack is the finished hull.
 */
static void graham_step(Scene *sc)
{
    if (sc->gs_done) return;
    if (sc->gs_idx >= N_POINTS) { sc->gs_done = true; return; }

    int candidate = sc->gs_sorted[sc->gs_idx];
    pop_while_not_strict_left_turn(sc->pts, sc->gs_stack, &sc->gs_sp,
                                   candidate);
    sc->gs_stack[sc->gs_sp++] = candidate;
    sc->gs_idx++;
    sc->gs_steps++;
}

/*
 * find_leftmost_point — Jarvis's start: the leftmost dot. It's always on the
 *   hull, so the wrap has a guaranteed-correct place to begin and end.
 */
static int find_leftmost_point(const Point *pts, int n)
{
    int idx = 0;
    for (int i = 1; i < n; i++)
        if (pts[i].x < pts[idx].x) idx = i;
    return idx;
}

/*
 * first_index_not_equal_to — a throwaway starting guess for "best" that isn't
 *   the current vertex, so the comparison loop has something to beat.
 */
static inline int first_index_not_equal_to(int cur)
{
    return (cur == 0) ? 1 : 0;
}

/*
 * is_more_counterclockwise — should the wrapping string swing over to cand?
 *   It does if cand sits further around the bend than the current best pick.
 *   (cross2 < 0 is the side that keeps the string sweeping forward; it reads
 *   as "clockwise" in math but matches the on-screen wrap because y is
 *   flipped — see the note on Point.)
 */
static inline bool is_more_counterclockwise(const Point *pts,
                                            int cur, int best, int cand)
{
    return cross2(&pts[cur], &pts[best], &pts[cand]) < 0.0f;
}

/*
 * jarvis_init — get the gift-wrap ready. Plant the first vertex at the
 *   leftmost dot, point the tip at it, and arm the candidate scanner.
 *   jarvis_step then sweeps one candidate per call until it wraps home.
 */
static void jarvis_init(Scene *sc)
{
    sc->jv_start = find_leftmost_point(sc->pts, N_POINTS);

    sc->jv_hull[0] = sc->jv_start;
    sc->jv_n       = 1;
    sc->jv_cur     = sc->jv_start;
    sc->jv_cand    = 0;
    sc->jv_best    = first_index_not_equal_to(sc->jv_cur);

    sc->jv_done  = false;
    sc->jv_steps = 0;
}

/*
 * commit_best_as_next_hull_vertex — the winning candidate becomes the next
 *   hull corner. Record it, move the tip onto it, and rearm the scanner for
 *   the next sweep.
 */
static inline void commit_best_as_next_hull_vertex(Scene *sc)
{
    sc->jv_hull[sc->jv_n++] = sc->jv_best;
    sc->jv_cur              = sc->jv_best;
    sc->jv_cand             = 0;
    sc->jv_best             = first_index_not_equal_to(sc->jv_cur);
}

/*
 * wraps_back_to_start — are we done? Yes when the next corner would be the dot
 *   we started from. The "n > 1" guard stops us calling it quits on the very
 *   first vertex, before any wrapping has actually happened.
 */
static inline bool wraps_back_to_start(const Scene *sc)
{
    return sc->jv_best == sc->jv_start && sc->jv_n > 1;
}

/*
 * jarvis_step — one tiny move of the gift-wrap per call, so it animates.
 *   Each call either weighs one more candidate against the current best, or,
 *   once every candidate has been weighed, locks in that corner (or stops if
 *   the wrap has come full circle).
 */
static void jarvis_step(Scene *sc)
{
    if (sc->jv_done) return;

    /* don't compare the current vertex against itself */
    if (sc->jv_cand == sc->jv_cur) { sc->jv_cand++; return; }

    /* every candidate weighed: stop if we've wrapped home, else lock the corner */
    if (sc->jv_cand >= N_POINTS) {
        if (wraps_back_to_start(sc)) {
            sc->jv_done = true;
            return;
        }
        commit_best_as_next_hull_vertex(sc);
        return;
    }

    /* does this candidate beat the current best? if so, it becomes best */
    if (is_more_counterclockwise(sc->pts, sc->jv_cur, sc->jv_best, sc->jv_cand))
        sc->jv_best = sc->jv_cand;

    sc->jv_cand++;
    sc->jv_steps++;
}

/*
 * new_points — scatter a fresh set of random dots and restart both algorithms.
 *   Dots are placed in the LEFT panel's area only; the renderer mirrors them
 *   into the right panel, so both algorithms get the exact same input. The
 *   small margins keep dots off the centre divider and clear of the HUD rows.
 *   Bound to SPACE, and also run on resize so the dots fit the new size.
 */
static void new_points(Scene *sc)
{
    int pw = sc->cols / 2 - 4;
    int ph = sc->rows - HUD_ROWS - 2;
    for (int i = 0; i < N_POINTS; i++) {
        sc->pts[i].x = 2 + (float)rand() / RAND_MAX * pw;
        sc->pts[i].y = 1 + (float)rand() / RAND_MAX * ph;
    }
    graham_init(sc);
    jarvis_init(sc);
}

/* ── §5 draw — paint the two panels each frame ── */

/*
 * draw_hull_edge — draw one straight line of '-' between two dots. It walks
 *   the line in even steps along its longer axis so no cell is skipped.
 *   col_offset slides the line into the right panel (it's 0 for the left).
 *   The bounds check stops a stray off-screen dot from scribbling outside.
 */
static void draw_hull_edge(const Scene *sc,
                           float x0, float y0, float x1, float y1,
                           int cp, int col_offset)
{
    int dx = (int)fabsf(x1 - x0), dy = (int)fabsf(y1 - y0);
    int steps = dx > dy ? dx : dy;
    if (steps == 0) steps = 1;
    attron(COLOR_PAIR(cp) | A_BOLD);
    for (int s = 0; s <= steps; s++) {
        float t = (float)s / (float)steps;
        int col = (int)(x0 + t*(x1-x0)) + col_offset;
        int row = (int)(y0 + t*(y1-y0)) + HUD_ROWS;
        if (row >= HUD_ROWS && row < sc->rows && col >= 0 && col < sc->cols)
            mvaddch(row, col, '-');
    }
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/*
 * draw_hull_polyline — connect a list of hull corners with edges. While an
 *   algorithm is still running we draw an open chain; once "closed" is true
 *   we also draw the final edge back to the start, sealing it into the
 *   finished polygon. Both algorithms feed their corner list in here.
 */
static void draw_hull_polyline(const Scene *sc, const int *indices, int n,
                               bool closed, int cp, int col_offset)
{
    for (int i = 0; i < n - 1; i++) {
        draw_hull_edge(sc,
                       sc->pts[indices[i]].x,   sc->pts[indices[i]].y,
                       sc->pts[indices[i+1]].x, sc->pts[indices[i+1]].y,
                       cp, col_offset);
    }
    if (closed && n >= 3) {
        draw_hull_edge(sc,
                       sc->pts[indices[n-1]].x, sc->pts[indices[n-1]].y,
                       sc->pts[indices[0]].x,   sc->pts[indices[0]].y,
                       cp, col_offset);
    }
}

/*
 * draw_active_wrapping_ray — the live "string" Jarvis is pulling taut. It runs
 *   from the current vertex to whichever candidate is winning right now; when
 *   a better candidate turns up mid-sweep, the line snaps over to it. Only
 *   drawn while Jarvis is still running.
 */
static void draw_active_wrapping_ray(const Scene *sc, int half)
{
    draw_hull_edge(sc,
                   sc->pts[sc->jv_cur ].x, sc->pts[sc->jv_cur ].y,
                   sc->pts[sc->jv_best].x, sc->pts[sc->jv_best].y,
                   CP_CUR, half);
}

/*
 * draw_panel_divider — the '|' line down the middle splitting the two panels.
 *   Drawn dim so it never fights with the hull edges for attention.
 */
static void draw_panel_divider(const Scene *sc, int half)
{
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    for (int r = HUD_ROWS; r < sc->rows; r++) mvaddch(r, half, '|');
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);
}

/*
 * draw_point_cloud — plot every input dot as '*' in both panels (once on the
 *   left, once mirrored on the right) so you can see both algorithms chewing
 *   on the same input. Bounds checks keep an off-screen dot from scribbling
 *   outside the terminal.
 */
static void draw_point_cloud(const Scene *sc, int half)
{
    attron(COLOR_PAIR(CP_PTS));
    for (int i = 0; i < N_POINTS; i++) {
        int col = (int)sc->pts[i].x;
        int row = (int)sc->pts[i].y + HUD_ROWS;
        if (row < HUD_ROWS || row >= sc->rows) continue;
        if (col >= 0      && col      < sc->cols) mvaddch(row, col,      '*');
        if (col+half >= 0 && col+half < sc->cols) mvaddch(row, col+half, '*');
    }
    attroff(COLOR_PAIR(CP_PTS));
}

/*
 * draw_panel_labels — name each panel's algorithm, in the same colour as that
 *   panel's hull so the eye can pair label to drawing at a glance.
 */
static void draw_panel_labels(int half)
{
    attron(COLOR_PAIR(CP_GR) | A_BOLD);
    mvprintw(HUD_ROWS, 1, "Graham scan O(n log n)");
    attroff(COLOR_PAIR(CP_GR) | A_BOLD);
    attron(COLOR_PAIR(CP_JA) | A_BOLD);
    mvprintw(HUD_ROWS, half + 1, "Jarvis march O(n*h)");
    attroff(COLOR_PAIR(CP_JA) | A_BOLD);
}

/*
 * draw_hud — the two top status lines: key hints on row 0, and live step and
 *   hull-size counts for each algorithm on row 1 (with a [DONE] tag when one
 *   finishes). Watching the two step counts race is the point of the demo.
 */
static void draw_hud(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " ConvexHull  q:quit  spc:new points  p:pause  +/-:speed  N=%d",
        N_POINTS);
    mvprintw(1, 0,
        " Graham: steps=%lld hull=%d %s   Jarvis: steps=%lld hull=%d %s",
        sc->gs_steps, sc->gs_sp, sc->gs_done ? "[DONE]" : "",
        sc->jv_steps, sc->jv_n,  sc->jv_done ? "[DONE]" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/*
 * scene_draw — paint one whole frame, back to front (later layers sit on top).
 *   Divider, dots, the two hulls, Jarvis's live ray, labels, then the HUD.
 *   The HUD is painted last so it always shows over anything that crept into
 *   the top rows.
 */
static void scene_draw(const Scene *sc)
{
    int half = sc->cols / 2;

    draw_panel_divider(sc, half);
    draw_point_cloud  (sc, half);

    /* Graham hull on the left (offset 0), Jarvis hull on the right (offset half) */
    draw_hull_polyline(sc, sc->gs_stack, sc->gs_sp, sc->gs_done, CP_GR, 0);
    draw_hull_polyline(sc, sc->jv_hull,  sc->jv_n,  sc->jv_done, CP_JA, half);

    if (!sc->jv_done) draw_active_wrapping_ray(sc, half);

    draw_panel_labels(half);
    draw_hud         (sc);
}

/* ── §6 app — ncurses setup, signals, and the main loop ── */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();

    Scene scene = {0};
    scene.step_ns = STEP_NS;
    getmaxyx(stdscr, scene.rows, scene.cols);
    new_points(&scene);

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, scene.rows, scene.cols);
            new_points(&scene);
            last = clock_ns();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case ' ': new_points(&scene); break;
        case 'p': case 'P': scene.paused = !scene.paused; break;
        case '+': case '=': scene.step_ns /= 2; if (scene.step_ns < 10000000LL) scene.step_ns = 10000000LL; break;
        case '-': scene.step_ns *= 2; if (scene.step_ns > 2000000000LL) scene.step_ns = 2000000000LL; break;
        default: break;
        }

        long long now = clock_ns();
        long long dt  = now - last;
        last = now;

        if (!scene.paused) {
            scene.step_accum += dt;
            while (scene.step_accum >= scene.step_ns) {
                scene.step_accum -= scene.step_ns;
                if (!scene.gs_done) graham_step(&scene);
                if (!scene.jv_done) jarvis_step(&scene);
            }
        }

        erase();
        scene_draw(&scene);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
