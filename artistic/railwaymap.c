/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * railwaymap.c — complex transit-map network  (10–15 interconnected lines)
 *
 * An 8×6 logical grid hosts up to 15 lines built from five path templates:
 *   H_FULL   — full-width horizontal at one grid row
 *   V_FULL   — full-height vertical at one grid column
 *   Z_SHAPE  — H → V bend → H  (classic S/Z shape)
 *   REV_Z    — V → H bridge → V  (upright Z)
 *   DOUBLE_Z — H → V → H → V → H  (two-bend zigzag)
 *
 * Stations emerge at every grid node on a line's path.
 * Nodes shared by ≥2 lines become interchange stations (shown as 'O').
 * A canvas is filled before drawing: each terminal cell is tagged with the
 * color-pair of its H-track and/or V-track → ACS_HLINE / ACS_VLINE / ACS_PLUS.
 * Station names are placed perpendicular to their line's local direction.
 *
 * Keys:  r new map   t/T theme   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/railwaymap.c \
 *       -o railwaymap -lncurses -lm
 *
 * §1 config  §2 clock  §3 themes  §4 color  §5 entity
 * §6 draw    §7 screen §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Procedural transit map generation using path templates.
 *                  Lines are built from 5 path templates (H_FULL, V_FULL,
 *                  Z_SHAPE, REV_Z, DOUBLE_Z) routed through an 8×6 logical
 *                  grid.  Interchange stations detected by counting how many
 *                  lines share a grid node (O(lines × path_length) scan).
 *
 * Data-structure : Canvas — flat 2D array of cell descriptors storing which
 *                  H-line and V-line (if any) pass through each terminal cell.
 *                  At render time: 0 lines → space, 1 H-line → ACS_HLINE,
 *                  1 V-line → ACS_VLINE, both → ACS_PLUS (cross symbol).
 *                  Animated trains stored as a pool of (line, progress)
 *                  structs; progress advances along the line's path.
 *
 * Math           : Grid node → terminal cell mapping: linear interpolation
 *                  between left/right margins and top/bottom margins so the
 *                  map fills the terminal regardless of size.  Station name
 *                  placement rotated 90° for vertical lines.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <curses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GNODES_C     8      /* logical grid columns                        */
#define GNODES_R     6      /* logical grid rows                           */
#define MAX_STATIONS 48     /* GNODES_C × GNODES_R                        */
#define MAX_LINES    15
#define MAX_PATH     70     /* max grid nodes per line path                */
#define NAME_LEN     14
#define LNAME_LEN    10
#define N_THEMES     10
#define MAX_TRAINS   10     /* animated trains; one per line up to this cap  */

/*
 * Canvas: one Cell per terminal character cell.
 * h_cp / v_cp = color-pair index of the track on that axis; 0 = no track.
 */
#define CANVAS_COLS 320
#define CANVAS_ROWS  90

typedef struct {
    unsigned char h_cp;
    unsigned char v_cp;
} Cell;

static Cell g_canvas[CANVAS_ROWS][CANVAS_COLS];

#define NSPS 1000000000LL
#define NSPM    1000000LL
enum { TARGET_FPS = 20 };

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  themes                                                             */
/* ===================================================================== */

typedef struct {
    const char *name;
    int line_fg[15];    /* one colour per line slot                        */
    int stn_fg;         /* regular station dot                             */
    int xchg_fg;        /* interchange station dot                         */
    int name_fg;        /* station name text                               */
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

/* ===================================================================== */
/* §4  color                                                              */
/* ===================================================================== */

/*
 * Color pair layout:
 *   1–15  : line colours (CP_LINE0 … CP_LINE0+14)
 *   16    : regular station dot
 *   17    : interchange station dot
 *   18    : station name text
 *   19    : HUD bar
 */
enum {
    CP_LINE0 = 1,   /* …through CP_LINE0+14 = 15 */
    CP_STN   = 16,
    CP_XCHG  = 17,
    CP_NAME  = 18,
    CP_HUD   = 19,
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
    }
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    color_apply_theme(theme);
}

/* ===================================================================== */
/* §5  entity                                                             */
/* ===================================================================== */

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

/* 15 line names — one per slot */
static const char *LINE_NAMES[15] = {
    "EXPRESS",  "CENTRAL",  "CIRCLE",  "DISTRICT", "JUBILEE",
    "ORBITAL",  "RAPID",    "METRO",   "PIONEER",  "ECLIPSE",
    "HORIZON",  "AURORA",   "COASTAL", "OVERLAND", "TRANSIT",
};

/* ── data structures ── */

typedef struct { int gc, gr; } GNode;

/*
 * Train — rides along one line's GNode path.
 * t is a float in [0, n_path-1].  dir = +1 or -1 (bounces at endpoints).
 * spd is in path-units per second.
 */
typedef struct {
    int   line_idx;
    float t;          /* current position along path                        */
    float spd;        /* path-units / second                                */
    int   dir;        /* +1 forward, -1 reverse                             */
} Train;

typedef struct {
    int  gc, gr;
    int  col, row;          /* terminal cell position                      */
    char name[NAME_LEN];
    int  n_lines;           /* lines passing through; ≥2 = interchange     */
    bool dir_h;             /* primary direction at this station           */
    int  name_side;         /* +1 or −1 depending on dir_h                 */
} Station;

typedef struct {
    GNode path[MAX_PATH];
    int   n_path;
    char  label;            /* 'A'–'O'                                     */
    char  lname[LNAME_LEN];
    int   cp;               /* color pair                                  */
} Line;

typedef struct {
    Station stations[MAX_STATIONS];
    int     n_stations;
    Line    lines[MAX_LINES];
    int     n_lines;
    Train   trains[MAX_TRAINS];
    int     n_trains;
    int     theme;
    int     n_xchg;         /* interchange count (for HUD)                 */
    int     term_col[GNODES_C];
    int     term_row[GNODES_R];
} RailMap;

/* ── helpers ── */

static void shuffle_ints(int *a, int n)
{
    for (int i = n-1; i > 0; i--) {
        int j = rand() % (i+1);
        int t = a[i]; a[i] = a[j]; a[j] = t;
    }
}

/*
 * append_h / append_v — grow a GNode path with a horizontal / vertical
 * segment, skipping the first node when it duplicates the current tail.
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

/* ── map generation ── */

static void railmap_gen(RailMap *m, int cols, int rows)
{
    int theme = m->theme;
    memset(m, 0, sizeof *m);
    m->theme = theme;

    /* ── terminal grid coordinates ── */
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

    /* ── shuffled pools for rows, cols, names ── */
    int h_rows[GNODES_R], v_cols[GNODES_C];
    for (int i = 0; i < GNODES_R; i++) h_rows[i] = i;
    for (int i = 0; i < GNODES_C; i++) v_cols[i] = i;
    shuffle_ints(h_rows, GNODES_R);
    shuffle_ints(v_cols, GNODES_C);

    /* ── line count and order ── */
    int n_lines = 12 + rand() % 4;   /* 12–15 */
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
        l->label  = (char)('A' + li);
        l->cp     = CP_LINE0 + li;
        strncpy(l->lname, LINE_NAMES[li], LNAME_LEN-1);
    }

    /* ── Template B: 3 V_FULL lines at spread cols ── */
    for (int k = 0; k < 3 && li < n_lines; k++, li++) {
        Line *l = &m->lines[li];
        l->n_path = 0;
        int gc  = v_cols[k % GNODES_C];
        int gr0 = (rand() % 5 == 0) ? 1 : 0;
        int gr1 = (rand() % 5 == 0) ? GNODES_R-2 : GNODES_R-1;
        l->n_path = append_v(l->path, 0, gc, gr0, gr1);
        l->label  = (char)('A' + li);
        l->cp     = CP_LINE0 + li;
        strncpy(l->lname, LINE_NAMES[li], LNAME_LEN-1);
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
        l->label  = (char)('A' + li);
        l->cp     = CP_LINE0 + li;
        strncpy(l->lname, LINE_NAMES[li], LNAME_LEN-1);
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
        l->label  = (char)('A' + li);
        l->cp     = CP_LINE0 + li;
        strncpy(l->lname, LINE_NAMES[li], LNAME_LEN-1);
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
        l->label  = (char)('A' + li);
        l->cp     = CP_LINE0 + li;
        strncpy(l->lname, LINE_NAMES[li], LNAME_LEN-1);
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
        l->label = (char)('A' + li);
        l->cp    = CP_LINE0 + li;
        strncpy(l->lname, LINE_NAMES[li], LNAME_LEN-1);
        li++;
    }
    m->n_lines = n_lines;

    /* ── register stations and fill canvas ── */
    memset(g_canvas, 0, sizeof g_canvas);

    for (int lii = 0; lii < m->n_lines; lii++) {
        Line *l = &m->lines[lii];

        /* register every path node as a station */
        for (int pi = 0; pi < l->n_path; pi++) {
            int gc = l->path[pi].gc, gr = l->path[pi].gr;
            int si = stn_get(m, gc, gr);
            Station *s = &m->stations[si];
            bool first = (s->n_lines == 0);
            s->n_lines++;
            s->col = m->term_col[gc];
            s->row = m->term_row[gr];

            if (first) {
                /* determine if this node sits on an H or V segment */
                bool h_nbr =
                    (pi > 0           && l->path[pi-1].gr == gr) ||
                    (pi < l->n_path-1 && l->path[pi+1].gr == gr);
                s->dir_h = h_nbr;

                if (h_nbr) {
                    /* H station: alternate names above/below by grid row */
                    s->name_side = (gr % 2 == 0) ? 1 : -1;
                } else {
                    /* V station: right if in left half, left if right half */
                    s->name_side = (m->term_col[gc] < cols / 2) ? 1 : -1;
                }
            }
        }

        /* fill canvas from consecutive path-node pairs */
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

    /* ── assign station names (shuffled pool) ── */
    int name_ord[N_POOL];
    for (int i = 0; i < N_POOL; i++) name_ord[i] = i;
    shuffle_ints(name_ord, N_POOL);
    for (int i = 0; i < m->n_stations; i++) {
        strncpy(m->stations[i].name,
                STATION_POOL[name_ord[i % N_POOL]], NAME_LEN-1);
        m->stations[i].name[NAME_LEN-1] = '\0';
    }

    /* ── count interchanges ── */
    m->n_xchg = 0;
    for (int i = 0; i < m->n_stations; i++)
        if (m->stations[i].n_lines >= 2) m->n_xchg++;

    /* ── spawn one train per line (capped at MAX_TRAINS) ── */
    m->n_trains = m->n_lines < MAX_TRAINS ? m->n_lines : MAX_TRAINS;
    for (int i = 0; i < m->n_trains; i++) {
        Train *tr   = &m->trains[i];
        Line  *l    = &m->lines[i];
        tr->line_idx = i;
        /* stagger start positions so trains don't bunch at t=0 */
        tr->t   = (float)(rand() % (l->n_path > 1 ? l->n_path - 1 : 1));
        tr->spd = 1.2f + (float)(rand() % 28) * 0.1f;   /* 1.2–4.0 nodes/s */
        tr->dir = (rand() % 2) ? +1 : -1;
    }
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

/*
 * trains_tick — advance all train positions by dt seconds.
 * Trains bounce at the ends of their line paths.
 */
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

/*
 * draw_trains — render each train on top of the canvas.
 *
 * Horizontal train (moving right): ## 0   (body ## + head 0 at the front)
 * Horizontal train (moving left):   0 ##
 * Vertical train   (moving down):   head 0 at bottom, ## above it
 * Vertical train   (moving up):     head 0 at top, ## below it
 *
 * A_REVERSE swaps fg/bg so the train appears as a solid coloured block —
 * clearly distinct from the thin ACS line characters of the track.
 */
static void draw_trains(WINDOW *win, const RailMap *m, int cols, int rows)
{
    int map_bot = rows - 4;    /* last usable row for map content */

    for (int i = 0; i < m->n_trains; i++) {
        const Train *tr = &m->trains[i];
        const Line  *l  = &m->lines[tr->line_idx];

        /* ── find current segment ── */
        int n = (int)tr->t;
        if (n >= l->n_path - 1) n = l->n_path - 2;
        if (n < 0) n = 0;
        float f = tr->t - (float)n;

        int gc0 = l->path[n].gc,   gr0 = l->path[n].gr;
        int gc1 = l->path[n+1].gc, gr1 = l->path[n+1].gr;
        int c0  = m->term_col[gc0], r0  = m->term_row[gr0];
        int c1  = m->term_col[gc1], r1  = m->term_row[gr1];

        /* interpolated terminal position of the head */
        int c = (int)(c0 + f * (float)(c1 - c0) + 0.5f);
        int r = (int)(r0 + f * (float)(r1 - r0) + 0.5f);

        if (r < 1 || r >= map_bot || c < 0 || c >= cols) continue;

        bool horiz   = (gr0 == gr1);
        int  cp      = l->cp;
        attr_t attr  = COLOR_PAIR(cp) | A_BOLD | A_REVERSE;

        wattron(win, attr);

        if (horiz) {
            /* heading right when segment goes right AND dir=+1, or left AND dir=-1 */
            bool go_right = ((c1 >= c0) == (tr->dir > 0));
            if (go_right) {
                /* body ## trails left of head 0 */
                if (c - 2 >= 0)     mvwaddch(win, r, c - 2, '#');
                if (c - 1 >= 0)     mvwaddch(win, r, c - 1, '#');
                mvwaddch(win, r, c, '0');
            } else {
                /* head 0 leads, body ## trails right */
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

static void draw_railmap(WINDOW *win, const RailMap *m, int cols, int rows)
{
    /* ── 1. canvas: H-track, V-track, or junction ── */
    int r_lo = 1;
    int r_hi = (rows - 4 < CANVAS_ROWS) ? rows - 4 : CANVAS_ROWS;

    for (int r = r_lo; r < r_hi; r++) {
        for (int c = 0; c < cols && c < CANVAS_COLS; c++) {
            const Cell *cl = &g_canvas[r][c];
            if (!cl->h_cp && !cl->v_cp) continue;

            chtype ch; int cp;
            if (cl->h_cp && cl->v_cp) {
                ch = ACS_PLUS;  cp = cl->h_cp;   /* H-line colour wins at junction */
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

    /* ── 2. station dots (overwrite canvas) ── */
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

    /* ── 3. trains (overwrite station dots they pass through) ── */
    draw_trains(win, m, cols, rows);

    /* ── 4. station names, perpendicular to track direction ── */
    for (int i = 0; i < m->n_stations; i++) {
        const Station *s = &m->stations[i];
        int nlen = (int)strlen(s->name);
        int nc, nr;

        if (s->dir_h) {
            /* H station: name above or below */
            nc = s->col - nlen / 2;
            nr = s->row + s->name_side;
        } else {
            /* V station: name right or left */
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

    /* ── 5. legend — 3 rows of 5 lines each ── */
    {
        int entry_w = (cols < 100) ? 14 : 16;   /* chars per legend slot  */
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

    /* ── 6. key hint (bottom row) ── */
    wattron(win, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(win, rows - 1, 0, " r:new   t/T:theme   q:quit ");
    wattroff(win, COLOR_PAIR(CP_HUD) | A_DIM);

    /* ── 7. HUD bar (top row) ── */
    wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvwprintw(win, 0, 0,
              " TRANSIT MAP   theme:[%d]%-6s  "
              "%2d lines  %2d stations  %2d interchange ",
              m->theme, THEMES[m->theme].name,
              m->n_lines, m->n_stations, m->n_xchg);
    wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §7  screen / scene                                                     */
/* ===================================================================== */

typedef struct {
    RailMap map;
    int     cols, rows;
} Scene;

static void scene_init(Scene *sc, int cols, int rows)
{
    int theme  = sc->map.theme;
    sc->cols   = cols;
    sc->rows   = rows;
    sc->map.theme = theme;
    railmap_gen(&sc->map, cols, rows);
}

typedef struct { int cols, rows; } Screen;

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
static void screen_render(Screen *sc, const Scene *s)
{
    erase();
    draw_railmap(stdscr, &s->map, sc->cols, sc->rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

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
        int64_t now = clock_ns();
        float   dt  = (float)(now - prev) * 1e-9f;
        if (dt > 0.15f) dt = 0.15f;   /* cap after pauses / resize */
        prev = now;

        if (app->need_resize) {
            int saved = app->scene.map.theme;
            screen_resize(&app->screen);
            app->scene.map.theme = saved;
            scene_init(&app->scene, app->screen.cols, app->screen.rows);
            color_apply_theme(saved);
            app->need_resize = 0;
            prev = clock_ns();   /* reset timer so resize spike doesn't teleport trains */
            continue;
        }

        trains_tick(&app->scene.map, dt);

        screen_render(&app->screen, &app->scene);

        int key = getch();
        if (key != ERR && !app_key(app, key))
            app->running = 0;

        clock_sleep_ns(frame_ns - (clock_ns() - now));
    }

    screen_free(&app->screen);
    return 0;
}
