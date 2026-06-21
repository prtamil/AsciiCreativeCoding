/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * railwaymap.c — a procedurally generated subway-style transit map.
 * Stations, lines and moving trains drawn with ncurses box-line characters.
 * Look modelled on Beck's London Underground; see Ovenden, "Transit Maps
 * of the World" (2007).
 */

#define _POSIX_C_SOURCE 200809L
#include <curses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — grid size, limits, color-pair IDs ── */

#define GNODES_C     8      /* logical grid columns                        */
#define GNODES_R     6      /* logical grid rows                           */
#define MAX_STATIONS 48     /* GNODES_C x GNODES_R                        */
#define MAX_LINES    15
#define MAX_PATH     70     /* max grid nodes per line path                */
#define NAME_LEN     14
#define LNAME_LEN    10
#define N_THEMES     10
#define MAX_TRAINS   10     /* animated trains; one per line up to this cap  */
#define LINES_MIN    12     /* fewest lines per generated map              */
#define LINES_VAR     4     /* line count = LINES_MIN + rand()%LINES_VAR   */

#define CANVAS_COLS 320
#define CANVAS_ROWS  90

/*
 * Cell — one character of the off-screen "ink layer" where tracks get painted
 * before they hit the screen.  We keep the horizontal and vertical track
 * separate so we can tell when two lines cross: a cell with both a horizontal
 * AND a vertical track is a junction and gets drawn as a '+'.  Painting into
 * this layer first also lets later lines sit on top of earlier ones cleanly.
 *   h_cp, v_cp — color-pair index of the line on that axis (CP_LINE0+slot),
 *                or 0 meaning "no track here". unsigned char keeps the big
 *                320x90 canvas small.
 */
typedef struct {
    unsigned char h_cp;
    unsigned char v_cp;
} Cell;

static Cell g_canvas[CANVAS_ROWS][CANVAS_COLS];

#define NSPS 1000000000LL
enum { TARGET_FPS = 20 };

/* Color-pair IDs. The 15 line colors are a contiguous block so a line's
 * slot maps straight to CP_LINE0+slot. */
enum {
    CP_LINE0 = 1,   /* line colours occupy 1..15 (CP_LINE0+0 .. +14) */
    CP_STN   = 16,
    CP_XCHG  = 17,
    CP_NAME  = 18,
    CP_HUD   = 19,
    CP_HINT  = 20,
};

/* ── §2 performance — monotonic clock and sleep helpers ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NSPS + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NSPS),
        .tv_nsec = (long)  (ns % NSPS),
    };
    nanosleep(&req, NULL);
}

/* ── §3 types — grid nodes, stations, lines, trains, the whole map ── */

/*
 * GNode — one node of the coarse 8x6 logical grid (gc = column, gr = row).
 * Real subway maps aren't drawn to geographic scale; they're tidy diagrams
 * snapped to a grid with only right-angle turns. We do all the layout work
 * in this grid (where a track segment is just two grid indices) and only
 * convert to actual screen cells at draw time, so one map fits any window.
 */
typedef struct { int gc, gr; } GNode;

/*
 * Train — a little marker that glides along one line's path of grid nodes.
 * The path is a list of nodes, but we want smooth motion between them, so the
 * position is a fractional number: a t of 2.5 means "halfway between node 2
 * and node 3". A train never leaves its own line; it just slides back and
 * forth, flipping direction at each end.
 *   line_idx — which Line this train rides
 *   t        — position along that line's path; 0 .. (n_path-1)
 *   spd      — how fast it moves, in path-nodes per second
 *   dir      — +1 going forward, -1 going back; flips at the ends
 */
typedef struct {
    int   line_idx;
    float t;
    float spd;
    int   dir;
} Train;

/*
 * Station — a stop sitting on one grid node.
 * The key field is n_lines: a stop where two or more lines meet is an
 * interchange (a transfer point), the most important feature on a transit
 * map, so we draw it bigger ('O' instead of 'o'). The name has to be placed
 * off the track so it stays readable, so we record which way the track runs
 * through here (dir_h) and which side to push the label (name_side).
 *   gc, gr     — grid node this stop lives on
 *   col, row   — its actual screen cell, cached from the grid->screen map
 *   name       — display name (from a shuffled pool)
 *   n_lines    — how many lines pass through; 2 or more means interchange
 *   dir_h      — true if the track runs horizontally through this stop
 *   name_side  — +1 or -1: which side of the track to write the label on
 */
typedef struct {
    int  gc, gr;
    int  col, row;
    char name[NAME_LEN];
    int  n_lines;
    bool dir_h;
    int  name_side;
} Station;

/*
 * Line — one transit line: the ordered list of grid nodes it runs through,
 * plus its identity. Each line is stamped out from one of five shape templates
 * (straight across, straight down, or various zig-zags) as a run of right-angle
 * segments. Storing the final node list lets the rest of the code (stations,
 * track painting, train motion) just walk path[] without re-deriving the shape.
 *   path, n_path — the grid nodes in order
 *   label        — legend letter 'A'..'O'
 *   lname        — legend name, e.g. "CENTRAL"
 *   cp           — color pair shared by this line's tracks AND its train, so
 *                  the whole line reads as one color
 */
typedef struct {
    GNode path[MAX_PATH];
    int   n_path;
    char  label;
    char  lname[LNAME_LEN];
    int   cp;
} Line;

/*
 * RailMap — the whole generated network in one place. The pieces refer to each
 * other by index (a Train points at lines[], stations share grid nodes), and
 * the whole thing is built or rebuilt as a unit, so they live together.
 *   stations/lines/trains + counts — the network contents
 *   n_xchg     — number of interchange stops, cached for the HUD readout
 *   theme      — active color palette index
 *   term_col[gc], term_row[gr] — screen column/row for each grid index;
 *                recomputed on every (re)generation so the map fills the window
 */
typedef struct {
    Station stations[MAX_STATIONS];
    int     n_stations;
    Line    lines[MAX_LINES];
    int     n_lines;
    Train   trains[MAX_TRAINS];
    int     n_trains;
    int     theme;
    int     n_xchg;
    int     term_col[GNODES_C];
    int     term_row[GNODES_R];
} RailMap;

/* Scene — what's shown (the map) plus the window size it was laid out for.
 * cols/rows are kept so a resize can re-lay the map to fill the new window. */
typedef struct {
    RailMap map;
    int     cols, rows;
} Scene;

/* Screen — the terminal size in cells, re-read at startup and on resize. */
typedef struct { int cols, rows; } Screen;

/*
 * App — everything the program holds at once: the scene, the screen, and two
 * flags set by signal handlers. It's a single global because POSIX signal
 * handlers get no user pointer, so the handler reaches these flags through a
 * file-scope object. volatile sig_atomic_t is the only type C promises is safe
 * to set in a handler and read in the main loop.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

/* ── §4 simulation — generate the map and move the trains ── */

/* ── name / label pools ── */

static const char *STATION_POOL[] = {
    /* city landmarks */
    "VICTORIA", "CENTRAL",  "WESTGATE", "EASTPORT", "NORTHEND",
    "SOUTHWAY",  "MIDTOWN",  "HARBOR",   "AIRPORT",  "MARKET",
    "BANK",      "BRIDGE",   "PALACE",   "GARDENS",  "RIVERSIDE",
    "HIGHBURY",  "LOWFIELD", "OLDTOWN",  "NEWGATE",  "PARKSIDE",
    "MOORGATE",  "JUNCTION", "UPTOWN",   "DOWNTOWN", "SUMMIT",
    "LAKESIDE",  "BAYSIDE",  "FAIRVIEW", "EDGEHILL", "GROVE",
    /* extended */
    "CHAPEL",   "MEADOW",   "VALLEY",   "RIDGE",    "HEATH",
    "FIELDS",   "CROSS",    "GREEN",    "SQUARE",   "LANE",
    "STATION",  "GATE",     "HILL",     "WOOD",     "FORD",
    "BROOK",    "MILL",     "PARK",     "CLOSE",    "ROAD",
    /* waterfront */
    "WHARF",    "QUAY",     "DOCK",     "PIER",     "COVE",
    /* rural */
    "MOOR",     "FEN",      "MARSH",    "DOWNS",    "CLIFFS",
};
#define N_POOL  (int)(sizeof STATION_POOL / sizeof STATION_POOL[0])

/* One name per line slot. */
static const char *LINE_NAMES[15] = {
    "EXPRESS",  "CENTRAL",  "CIRCLE",  "DISTRICT", "JUBILEE",
    "ORBITAL",  "RAPID",    "METRO",   "PIONEER",  "ECLIPSE",
    "HORIZON",  "AURORA",   "COASTAL", "OVERLAND", "TRANSIT",
};

/* ── helpers ── */

static void shuffle_ints(int *a, int n)
{
    for (int i = n-1; i > 0; i--) {
        int j = rand() % (i+1);
        int t = a[i]; a[i] = a[j]; a[j] = t;
    }
}

/*
 * append_h / append_v — add a straight horizontal / vertical run of grid nodes
 * to a path. If the first new node is the same as the path's current end (which
 * happens where two segments meet at a corner), it's skipped so the join isn't
 * duplicated.
 */
static int append_h(GNode *p, int n, int gc0, int gc1, int gr)
{
    int step = (gc1 >= gc0) ? 1 : -1;
    for (int gc = gc0; gc != gc1 + step; gc += step) {
        if (n > 0 && p[n-1].gc == gc && p[n-1].gr == gr) continue;
        if (n >= MAX_PATH) break;
        p[n].gc = gc; p[n].gr = gr; n++;
    }
    return n;
}

static int append_v(GNode *p, int n, int gc, int gr0, int gr1)
{
    int step = (gr1 >= gr0) ? 1 : -1;
    for (int gr = gr0; gr != gr1 + step; gr += step) {
        if (n > 0 && p[n-1].gc == gc && p[n-1].gr == gr) continue;
        if (n >= MAX_PATH) break;
        p[n].gc = gc; p[n].gr = gr; n++;
    }
    return n;
}

/* Find or create a station at (gc,gr); returns its index. */
static int stn_get(RailMap *m, int gc, int gr)
{
    for (int i = 0; i < m->n_stations; i++)
        if (m->stations[i].gc == gc && m->stations[i].gr == gr)
            return i;
    if (m->n_stations >= MAX_STATIONS) return 0;
    int i = m->n_stations++;
    m->stations[i] = (Station){ .gc=gc, .gr=gr, .n_lines=0 };
    return i;
}

/* Clamp gc to [0, GNODES_C-1] */
static int clamp_gc(int v) { return v < 0 ? 0 : v >= GNODES_C ? GNODES_C-1 : v; }

/* ── map generation (RESET: rebuilds the whole map + canvas) ── */

/* Set a line's legend identity (label letter, colour pair, name). */
static void line_set_identity(Line *l, int li)
{
    l->label = (char)('A' + li);
    l->cp    = CP_LINE0 + li;
    strncpy(l->lname, LINE_NAMES[li], LNAME_LEN-1);
}

/* Work out the screen column/row for each grid index, spreading the grid evenly
 * across the window with a margin around it (the bottom 4 rows are saved for the
 * legend and key hint). This is the one place the logical grid becomes pixels. */
static void compute_term_grid(RailMap *m, int cols, int rows)
{
    int mg_c = cols / 10;
    if (mg_c < 4) mg_c = 4;
    int mg_r = 3;

    /* Bottom margin: 3 legend rows + 1 hint row = 4 rows reserved */
    int step_c = (cols - 2*mg_c) / (GNODES_C - 1);
    int step_r = (rows - mg_r - 4) / (GNODES_R - 1);
    if (step_c < 8)  step_c = 8;
    if (step_r < 3)  step_r = 3;

    for (int i = 0; i < GNODES_C; i++) {
        m->term_col[i] = mg_c + i * step_c;
        if (m->term_col[i] >= cols - 2) m->term_col[i] = cols - 3;
    }
    for (int i = 0; i < GNODES_R; i++) {
        m->term_row[i] = mg_r + i * step_r;
        if (m->term_row[i] >= rows - 4) m->term_row[i] = rows - 5;
    }
}

/* Lay down 12-15 lines on the grid using the five shape templates (a few of
 * each, then alternating zig-zags to fill out the count). The row/column pools
 * are shuffled first so the straight lines land in different places each run. */
static void build_lines(RailMap *m)
{
    /* ── shuffled pools for rows, cols ── */
    int h_rows[GNODES_R], v_cols[GNODES_C];
    for (int i = 0; i < GNODES_R; i++) h_rows[i] = i;
    for (int i = 0; i < GNODES_C; i++) v_cols[i] = i;
    shuffle_ints(h_rows, GNODES_R);
    shuffle_ints(v_cols, GNODES_C);

    /* ── line count and order ── */
    int n_lines = LINES_MIN + rand() % LINES_VAR;   /* 12-15 */
    if (n_lines > MAX_LINES) n_lines = MAX_LINES;

    int li = 0;

    /* ── Template A: 3 H_FULL lines at spread rows ── */
    for (int k = 0; k < 3 && li < n_lines; k++, li++) {
        Line *l = &m->lines[li];
        l->n_path = 0;
        int gr  = h_rows[k % GNODES_R];
        int gc0 = (rand() % 5 == 0) ? 1 : 0;
        int gc1 = (rand() % 5 == 0) ? GNODES_C-2 : GNODES_C-1;
        l->n_path = append_h(l->path, 0, gc0, gc1, gr);
        line_set_identity(l, li);
    }

    /* ── Template B: 3 V_FULL lines at spread cols ── */
    for (int k = 0; k < 3 && li < n_lines; k++, li++) {
        Line *l = &m->lines[li];
        l->n_path = 0;
        int gc  = v_cols[k % GNODES_C];
        int gr0 = (rand() % 5 == 0) ? 1 : 0;
        int gr1 = (rand() % 5 == 0) ? GNODES_R-2 : GNODES_R-1;
        l->n_path = append_v(l->path, 0, gc, gr0, gr1);
        line_set_identity(l, li);
    }

    /* ── Template C: 3 Z_SHAPE lines (H → V → H) ── */
    for (int k = 0; k < 3 && li < n_lines; k++, li++) {
        Line *l = &m->lines[li];
        l->n_path = 0;
        int gr1    = rand() % GNODES_R;
        int gr2; do { gr2 = rand() % GNODES_R; } while (gr2 == gr1);
        int gc_mid = 1 + rand() % (GNODES_C - 2);
        int gc0    = (rand() % 3 == 0) ? 1 : 0;
        int gc1    = (rand() % 3 == 0) ? GNODES_C-2 : GNODES_C-1;
        l->n_path = append_h(l->path, 0,          gc0,    gc_mid, gr1);
        l->n_path = append_v(l->path, l->n_path,  gc_mid, gr1,    gr2);
        l->n_path = append_h(l->path, l->n_path,  gc_mid, gc1,    gr2);
        line_set_identity(l, li);
    }

    /* ── Template D: 2 REV_Z lines (V → H bridge → V) ── */
    for (int k = 0; k < 2 && li < n_lines; k++, li++) {
        Line *l = &m->lines[li];
        l->n_path = 0;
        int gc1    = v_cols[(k + 3) % GNODES_C];
        int gc2; do { gc2 = rand() % GNODES_C; } while (gc2 == gc1);
        int gr_top = (rand() % 3 == 0) ? 1 : 0;
        int gr_bot = (rand() % 3 == 0) ? GNODES_R-2 : GNODES_R-1;
        int gr_mid = 1 + rand() % (GNODES_R - 2);
        l->n_path = append_v(l->path, 0,          gc1, gr_top, gr_mid);
        l->n_path = append_h(l->path, l->n_path,  gc1, gc2,    gr_mid);
        l->n_path = append_v(l->path, l->n_path,  gc2, gr_mid, gr_bot);
        line_set_identity(l, li);
    }

    /* ── Template E: 1 DOUBLE_Z line (H → V → H → V → H, two bends) ── */
    if (li < n_lines) {
        Line *l = &m->lines[li];
        l->n_path = 0;
        int gr_a = rand() % GNODES_R;
        int gr_b; do { gr_b = rand() % GNODES_R; } while (gr_b == gr_a);
        int gr_c; do { gr_c = rand() % GNODES_R; } while (gr_c == gr_b);
        int gc_b1 = clamp_gc(1 + rand() % (GNODES_C - 3));
        int gc_b2 = clamp_gc(gc_b1 + 1 + rand() % (GNODES_C - gc_b1 - 1));
        if (gc_b2 <= gc_b1) gc_b2 = clamp_gc(gc_b1 + 1);
        l->n_path = append_h(l->path, 0,          0,     gc_b1, gr_a);
        l->n_path = append_v(l->path, l->n_path,  gc_b1, gr_a,  gr_b);
        l->n_path = append_h(l->path, l->n_path,  gc_b1, gc_b2, gr_b);
        l->n_path = append_v(l->path, l->n_path,  gc_b2, gr_b,  gr_c);
        l->n_path = append_h(l->path, l->n_path,  gc_b2, GNODES_C-1, gr_c);
        line_set_identity(l, li);
        li++;
    }

    /* ── Remaining slots: alternate Z_SHAPE and REV_Z ── */
    while (li < n_lines) {
        Line *l = &m->lines[li];
        l->n_path = 0;
        if (li % 2 == 0) {
            /* Z_SHAPE */
            int gr1    = rand() % GNODES_R;
            int gr2; do { gr2 = rand() % GNODES_R; } while (gr2 == gr1);
            int gc_mid = 1 + rand() % (GNODES_C - 2);
            l->n_path = append_h(l->path, 0,         0,      gc_mid, gr1);
            l->n_path = append_v(l->path, l->n_path, gc_mid, gr1,    gr2);
            l->n_path = append_h(l->path, l->n_path, gc_mid, GNODES_C-1, gr2);
        } else {
            /* REV_Z */
            int gc1    = rand() % GNODES_C;
            int gc2; do { gc2 = rand() % GNODES_C; } while (gc2 == gc1);
            int gr_mid = 1 + rand() % (GNODES_R - 2);
            l->n_path = append_v(l->path, 0,         gc1, 0,      gr_mid);
            l->n_path = append_h(l->path, l->n_path, gc1, gc2,    gr_mid);
            l->n_path = append_v(l->path, l->n_path, gc2, gr_mid, GNODES_R-1);
        }
        line_set_identity(l, li);
        li++;
    }
    m->n_lines = n_lines;
}

/* Turn every node on a line's path into a station (creating it, or bumping its
 * line count if another line already stops there). The first time a station is
 * created we decide which side its label goes, off the track. */
static void register_line_stations(RailMap *m, const Line *l, int cols)
{
    for (int pi = 0; pi < l->n_path; pi++) {
        int gc = l->path[pi].gc, gr = l->path[pi].gr;
        int si = stn_get(m, gc, gr);
        Station *s = &m->stations[si];
        bool first = (s->n_lines == 0);
        s->n_lines++;
        s->col = m->term_col[gc];
        s->row = m->term_row[gr];

        if (first) {
            /* does the track run horizontally through this stop? */
            bool h_nbr =
                (pi > 0           && l->path[pi-1].gr == gr) ||
                (pi < l->n_path-1 && l->path[pi+1].gr == gr);
            s->dir_h = h_nbr;

            if (h_nbr) {
                /* horizontal stop: put the name above or below, alternating by row */
                s->name_side = (gr % 2 == 0) ? 1 : -1;
            } else {
                /* vertical stop: name on the right if near the left edge, else left */
                s->name_side = (m->term_col[gc] < cols / 2) ? 1 : -1;
            }
        }
    }
}

/* Paint a line's color onto the canvas cell by cell: along horizontal runs the
 * color goes in h_cp, along vertical runs in v_cp. A cell that ends up with both
 * is where two lines cross, and it'll be drawn as a junction. */
static void paint_line_to_canvas(const RailMap *m, const Line *l)
{
    for (int pi = 0; pi < l->n_path - 1; pi++) {
        int c1 = m->term_col[l->path[pi].gc],    r1 = m->term_row[l->path[pi].gr];
        int c2 = m->term_col[l->path[pi+1].gc],  r2 = m->term_row[l->path[pi+1].gr];

        if (r1 == r2) {
            /* horizontal segment */
            int clo = c1 < c2 ? c1 : c2;
            int chi = c1 < c2 ? c2 : c1;
            for (int c = clo; c <= chi; c++)
                if (r1 >= 0 && r1 < CANVAS_ROWS && c >= 0 && c < CANVAS_COLS)
                    g_canvas[r1][c].h_cp = (unsigned char)l->cp;
        } else {
            /* vertical segment */
            int rlo = r1 < r2 ? r1 : r2;
            int rhi = r1 < r2 ? r2 : r1;
            for (int r = rlo; r <= rhi; r++)
                if (r >= 0 && r < CANVAS_ROWS && c1 >= 0 && c1 < CANVAS_COLS)
                    g_canvas[r][c1].v_cp = (unsigned char)l->cp;
        }
    }
}

/* assign_station_names — give every station a distinct name from the shuffled
 * pool (wrapping if there are more stations than names). */
static void assign_station_names(RailMap *m)
{
    int name_ord[N_POOL];
    for (int i = 0; i < N_POOL; i++) name_ord[i] = i;
    shuffle_ints(name_ord, N_POOL);
    for (int i = 0; i < m->n_stations; i++) {
        strncpy(m->stations[i].name,
                STATION_POOL[name_ord[i % N_POOL]], NAME_LEN-1);
        m->stations[i].name[NAME_LEN-1] = '\0';
    }
}

/* count_interchanges — cache the count of ≥2-line stations for the HUD. */
static void count_interchanges(RailMap *m)
{
    m->n_xchg = 0;
    for (int i = 0; i < m->n_stations; i++)
        if (m->stations[i].n_lines >= 2) m->n_xchg++;
}

/* Put one train on each line (up to the train cap), starting at a random spot
 * along the line so they don't all bunch together, at a random speed and
 * direction. */
static void spawn_trains(RailMap *m)
{
    m->n_trains = m->n_lines < MAX_TRAINS ? m->n_lines : MAX_TRAINS;
    for (int i = 0; i < m->n_trains; i++) {
        Train *tr   = &m->trains[i];
        Line  *l    = &m->lines[i];
        tr->line_idx = i;
        /* stagger start positions so trains don't bunch at t=0 */
        tr->t   = (float)(rand() % (l->n_path > 1 ? l->n_path - 1 : 1));
        tr->spd = 1.2f + (float)(rand() % 28) * 0.1f;   /* 1.2-4.0 nodes/s */
        tr->dir = (rand() % 2) ? +1 : -1;
    }
}

/*
 * Build a whole fresh map from scratch: lay out the grid, draw the lines, turn
 * their nodes into stations and paint the tracks, name the stations, count the
 * interchanges, then spawn the trains. The color theme is kept across the wipe.
 */
static void railmap_gen(RailMap *m, int cols, int rows)
{
    int theme = m->theme;
    memset(m, 0, sizeof *m);
    m->theme = theme;

    compute_term_grid(m, cols, rows);
    build_lines(m);

    memset(g_canvas, 0, sizeof g_canvas);
    for (int i = 0; i < m->n_lines; i++) {
        register_line_stations(m, &m->lines[i], cols);
        paint_line_to_canvas(m, &m->lines[i]);
    }

    assign_station_names(m);
    count_interchanges(m);
    spawn_trains(m);
}

/* Move every train forward by dt seconds, reversing direction when one hits
 * either end of its line so it bounces back. */
static void trains_tick(RailMap *m, float dt)
{
    for (int i = 0; i < m->n_trains; i++) {
        Train *tr = &m->trains[i];
        Line  *l  = &m->lines[tr->line_idx];
        float  end = (float)(l->n_path - 1);

        tr->t += (float)tr->dir * tr->spd * dt;

        if (tr->t >= end) { tr->t = end; tr->dir = -1; }
        if (tr->t <= 0.0f){ tr->t = 0.0f; tr->dir = +1; }
    }
}

static void scene_init(Scene *sc, int cols, int rows)
{
    int theme  = sc->map.theme;
    sc->cols   = cols;
    sc->rows   = rows;
    sc->map.theme = theme;
    railmap_gen(&sc->map, cols, rows);
}

/* ── §5 render — draw the map to the screen; never changes the map ── */

/*
 * Theme — one color palette for the map. Telling lines apart by color is the
 * whole point of a subway map, so a theme is really 15 distinct line colors plus
 * a few accent colors. Cycle through them with t/T.
 *   name    — palette name shown in the HUD
 *   line_fg — one color per line slot (also used for that line's train)
 *   stn_fg  — regular station dot
 *   xchg_fg — interchange station dot
 *   name_fg — station name text
 * All colors are 256-color indices, kept in the bright half so they stay
 * readable on the default background.
 */
typedef struct {
    const char *name;
    int line_fg[15];
    int stn_fg;
    int xchg_fg;
    int name_fg;
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* METRO — diverse transit-map palette, one hue per line */
    { "METRO",
      { 196, 208, 220, 154,  46,  43,  51, 117,  27,  99,
        201, 213, 130, 231, 244 },
      231, 226, 255 },

    /* FIRE — warm spectrum from ember to white */
    { "FIRE",
      { 196, 202, 208, 214, 220, 226, 228, 203, 197, 160,
        124,  88,  52,  58, 231 },
      231, 226, 220 },

    /* MATRIX — full green range */
    { "MATRIX",
      {  46,  40,  34,  82, 118, 154, 190, 148, 106,  64,
         22,  28, 155, 120, 231 },
      231, 118, 118 },

    /* PLASMA — violet to pink */
    { "PLASMA",
      { 201, 207, 213, 219, 165, 171, 177, 183, 129, 135,
        141, 147,  93,  57, 231 },
      231, 207, 207 },

    /* NOVA — deep blue to white */
    { "NOVA",
      {  21,  27,  33,  39,  45,  51,  63,  69,  81,  87,
        117, 123, 159, 195, 231 },
      231, 123, 123 },

    /* OCEAN — navy to ice */
    { "OCEAN",
      {  17,  24,  31,  38,  45,  51,  61,  67,  73,  87,
        123, 159, 195,  39, 231 },
      159,  87,  87 },

    /* GOLD — copper to pale */
    { "GOLD",
      { 124, 130, 136, 172, 178, 208, 214, 220, 228, 222,
        186, 180, 174, 168, 231 },
      231, 228, 228 },

    /* NEON — maximum contrast mixed hues */
    { "NEON",
      { 201, 226,  46,  51,  21, 165, 208, 154,  87, 213,
        220, 118,  45, 177, 231 },
      231, 231, 213 },

    /* ARCTIC — cool ice palette */
    { "ARCTIC",
      { 231, 195, 153, 117,  81,  45, 159, 123,  87,  51,
         39,  33,  27,  21, 244 },
      231, 231, 195 },

    /* LAVA — molten deep spectrum */
    { "LAVA",
      { 196, 202, 208, 124, 160,  88, 214, 220,  52, 160,
        130,  94, 166, 172, 231 },
      228, 228, 208 },
};

static void color_apply_theme(int idx)
{
    const Theme *t = &THEMES[idx];
    if (COLORS >= 256) {
        for (int i = 0; i < 15; i++)
            init_pair(CP_LINE0 + i, t->line_fg[i], -1);
        init_pair(CP_STN,  t->stn_fg,  -1);
        init_pair(CP_XCHG, t->xchg_fg, -1);
        init_pair(CP_NAME, t->name_fg, -1);
        init_pair(CP_HUD,  226,        -1);
        init_pair(CP_HINT,  51,        -1);
    } else {
        /* 8-colour fallback: cycle the 6 basic colours, reuse for excess */
        static const int fb[6] = {
            COLOR_RED, COLOR_GREEN, COLOR_YELLOW,
            COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN
        };
        for (int i = 0; i < 15; i++)
            init_pair(CP_LINE0 + i, fb[i % 6], -1);
        init_pair(CP_STN,  COLOR_WHITE,  -1);
        init_pair(CP_XCHG, COLOR_WHITE,  -1);
        init_pair(CP_NAME, COLOR_WHITE,  -1);
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    color_apply_theme(theme);
}

/*
 * Draw each train on top of the tracks. A train is a short bar: a '0' head with
 * a '##' body trailing behind it, oriented along its current segment and always
 * with the head facing the way it's going. A_REVERSE swaps foreground and
 * background so the train shows up as a solid colored block, easy to tell apart
 * from the thin track lines.
 */
static void draw_trains(WINDOW *win, const RailMap *m, int cols, int rows)
{
    int map_bot = rows - 4;    /* last usable row for map content */

    for (int i = 0; i < m->n_trains; i++) {
        const Train *tr = &m->trains[i];
        const Line  *l  = &m->lines[tr->line_idx];

        /* which segment is the train on, and how far along it (f, 0..1)? */
        int n = (int)tr->t;
        if (n >= l->n_path - 1) n = l->n_path - 2;
        if (n < 0) n = 0;
        float f = tr->t - (float)n;

        int gc0 = l->path[n].gc,   gr0 = l->path[n].gr;
        int gc1 = l->path[n+1].gc, gr1 = l->path[n+1].gr;
        int c0  = m->term_col[gc0], r0  = m->term_row[gr0];
        int c1  = m->term_col[gc1], r1  = m->term_row[gr1];

        /* screen cell of the head, blended between the two segment ends */
        int c = (int)(c0 + f * (float)(c1 - c0) + 0.5f);
        int r = (int)(r0 + f * (float)(r1 - r0) + 0.5f);

        if (r < 1 || r >= map_bot || c < 0 || c >= cols) continue;

        bool horiz   = (gr0 == gr1);
        int  cp      = l->cp;
        attr_t attr  = COLOR_PAIR(cp) | A_BOLD | A_REVERSE;

        wattron(win, attr);

        if (horiz) {
            bool go_right = ((c1 >= c0) == (tr->dir > 0));
            if (go_right) {
                /* body trails to the left of the head */
                if (c - 2 >= 0)     mvwaddch(win, r, c - 2, '#');
                if (c - 1 >= 0)     mvwaddch(win, r, c - 1, '#');
                mvwaddch(win, r, c, '0');
            } else {
                /* body trails to the right of the head */
                mvwaddch(win, r, c, '0');
                if (c + 1 < cols)   mvwaddch(win, r, c + 1, '#');
                if (c + 2 < cols)   mvwaddch(win, r, c + 2, '#');
            }
        } else {
            /* vertical: head 0 on front row, ## on two trailing rows */
            bool go_down = ((r1 >= r0) == (tr->dir > 0));
            if (go_down) {
                /* head at bottom */
                if (r - 2 >= 1 && r - 2 < map_bot) mvwaddch(win, r - 2, c, '#');
                if (r - 1 >= 1 && r - 1 < map_bot) mvwaddch(win, r - 1, c, '#');
                mvwaddch(win, r, c, '0');
            } else {
                /* head at top */
                mvwaddch(win, r, c, '0');
                if (r + 1 >= 1 && r + 1 < map_bot) mvwaddch(win, r + 1, c, '#');
                if (r + 2 >= 1 && r + 2 < map_bot) mvwaddch(win, r + 2, c, '#');
            }
        }

        wattroff(win, attr);
    }
}

/* Draw the painted track layer onto the screen: a horizontal line, a vertical
 * line, or a '+' junction where a cell has both a horizontal and vertical track. */
static void draw_canvas(WINDOW *win, int cols, int rows)
{
    int r_lo = 1;
    int r_hi = (rows - 4 < CANVAS_ROWS) ? rows - 4 : CANVAS_ROWS;

    for (int r = r_lo; r < r_hi; r++) {
        for (int c = 0; c < cols && c < CANVAS_COLS; c++) {
            const Cell *cl = &g_canvas[r][c];
            if (!cl->h_cp && !cl->v_cp) continue;

            chtype ch; int cp;
            if (cl->h_cp && cl->v_cp) {
                ch = ACS_PLUS;  cp = cl->h_cp;   /* at a crossing the horizontal line's colour wins */
            } else if (cl->h_cp) {
                ch = ACS_HLINE; cp = cl->h_cp;
            } else {
                ch = ACS_VLINE; cp = cl->v_cp;
            }
            wattron(win, COLOR_PAIR(cp) | A_BOLD);
            mvwaddch(win, r, c, ch);
            wattroff(win, COLOR_PAIR(cp) | A_BOLD);
        }
    }
}

/* Put a dot at each station: 'O' for an interchange, 'o' for a regular stop. */
static void draw_stations(WINDOW *win, const RailMap *m, int cols, int rows)
{
    for (int i = 0; i < m->n_stations; i++) {
        const Station *s = &m->stations[i];
        if (s->col < 0 || s->col >= cols) continue;
        if (s->row < 1 || s->row >= rows - 4) continue;

        bool xchg = (s->n_lines >= 2);
        int  cp   = xchg ? CP_XCHG : CP_STN;
        wattron(win, COLOR_PAIR(cp) | A_BOLD);
        mvwaddch(win, s->row, s->col, xchg ? 'O' : 'o');
        wattroff(win, COLOR_PAIR(cp) | A_BOLD);
    }
}

/* Write each station's name beside it, off to the side of its track, nudged back
 * inside the visible area if it would run off an edge. */
static void draw_station_names(WINDOW *win, const RailMap *m, int cols, int rows)
{
    for (int i = 0; i < m->n_stations; i++) {
        const Station *s = &m->stations[i];
        int nlen = (int)strlen(s->name);
        int nc, nr;

        if (s->dir_h) {
            /* horizontal stop: name centered above or below the dot */
            nc = s->col - nlen / 2;
            nr = s->row + s->name_side;
        } else {
            /* vertical stop: name to the right or left of the dot */
            nr = s->row;
            nc = (s->name_side > 0) ? s->col + 2 : s->col - nlen - 1;
        }

        if (nc < 1)            nc = 1;
        if (nc + nlen >= cols) nc = cols - nlen - 1;
        if (nr < 1)            nr = 1;
        if (nr >= rows - 4)    nr = rows - 5;

        wattron(win, COLOR_PAIR(CP_NAME) | A_BOLD);
        mvwprintw(win, nr, nc, "%s", s->name);
        wattroff(win, COLOR_PAIR(CP_NAME) | A_BOLD);
    }
}

/* Draw the line key along the bottom: up to 5 "[letter]name" entries per row,
 * 3 rows, each in its line's color. */
static void draw_legend(WINDOW *win, const RailMap *m, int cols, int rows)
{
    int entry_w = (cols < 100) ? 14 : 16;   /* width of one legend entry */
    int per_row = 5;
    int li      = 0;
    for (int row_off = 0; row_off < 3; row_off++) {
        int lr = rows - 4 + row_off;
        int lc = 2;
        for (int k = 0; k < per_row && li < m->n_lines; k++, li++) {
            const Line *l = &m->lines[li];
            wattron(win, COLOR_PAIR(l->cp) | A_BOLD);
            mvwprintw(win, lr, lc, "[%c]%-*s", l->label, entry_w - 4, l->lname);
            wattroff(win, COLOR_PAIR(l->cp) | A_BOLD);
            lc += entry_w;
            if (lc >= cols - entry_w) break;
        }
    }
}

/* Draw the status line along the top and the key hint along the bottom. */
static void draw_hud(WINDOW *win, const RailMap *m, int cols, int rows)
{
    char buf[160];
    snprintf(buf, sizeof buf,
             " TRANSIT MAP   theme:[%d]%-6s  "
             "%2d lines  %2d stations  %2d interchange ",
             m->theme, THEMES[m->theme].name,
             m->n_lines, m->n_stations, m->n_xchg);
    wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvwprintw(win, 0, 0, "%.*s", cols, buf);   /* clip to window width so it can't wrap */
    wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);

    wattron(win, COLOR_PAIR(CP_HINT) | A_BOLD);
    mvwprintw(win, rows - 1, 0, "%.*s", cols, " r:new   t/T:theme   q:quit ");
    wattroff(win, COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* Draw the whole frame bottom layer first so each layer covers the one below:
 * tracks, then station dots, then trains, then names, then legend, then HUD. */
static void draw_railmap(WINDOW *win, const RailMap *m, int cols, int rows)
{
    draw_canvas(win, cols, rows);
    draw_stations(win, m, cols, rows);
    draw_trains(win, m, cols, rows);
    draw_station_names(win, m, cols, rows);
    draw_legend(win, m, cols, rows);
    draw_hud(win, m, cols, rows);
}

static void screen_init(Screen *sc, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize(Screen *sc)
{
    endwin(); refresh(); getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_render(const Screen *sc, const Scene *s)
{
    erase();
    draw_railmap(stdscr, &s->map, sc->cols, sc->rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §6 app — main loop, keys, signals, frame timing ── */

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

/* Handle a key: q/ESC quits, r builds a new map, t/T cycle the color theme. */
static bool app_key(App *app, int ch)
{
    RailMap *m = &app->scene.map;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case 'r': case 'R':
        railmap_gen(m, app->screen.cols, app->screen.rows);
        break;
    case 't':
        m->theme = (m->theme + 1) % N_THEMES;
        color_apply_theme(m->theme);
        break;
    case 'T':
        m->theme = (m->theme + N_THEMES - 1) % N_THEMES;
        color_apply_theme(m->theme);
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    App *app     = &g_app;
    app->running = 1;
    app->scene.map.theme = 0;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_ns = NSPS / TARGET_FPS;
    int64_t prev     = clock_ns();

    while (app->running) {
        /* seconds since last frame; cap it so a long stall doesn't jump the world */
        int64_t now = clock_ns();
        float   dt  = (float)(now - prev) * 1e-9f;
        if (dt > 0.15f) dt = 0.15f;
        prev = now;

        /* window resized: rebuild the whole map to fit the new size */
        if (app->need_resize) {
            int saved = app->scene.map.theme;
            screen_resize(&app->screen);
            app->scene.map.theme = saved;
            scene_init(&app->scene, app->screen.cols, app->screen.rows);
            color_apply_theme(saved);
            app->need_resize = 0;
            prev = clock_ns();   /* restart the clock so the resize gap doesn't teleport trains */
            continue;
        }

        trains_tick(&app->scene.map, dt);
        screen_render(&app->screen, &app->scene);

        int key = getch();
        if (key != ERR && !app_key(app, key))
            app->running = 0;

        /* sleep off the rest of the frame to hold a steady frame rate */
        clock_sleep_ns(frame_ns - (clock_ns() - now));
    }

    screen_free(&app->screen);
    return 0;
}
