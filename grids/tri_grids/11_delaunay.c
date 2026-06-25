/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 11_delaunay.c — connect scattered dots into nice, well-shaped triangles.
 *
 * Drop N random points on the screen and join them into triangles with the
 * Bowyer-Watson method, which avoids skinny slivers and favours fat, even
 * triangles. Move the cursor with ',' / '.' to pick a point and light up the
 * triangles around it; 'r' scatters fresh points.
 *
 * Sister files: 01_equilateral.c is the tidy regular-grid cousin; this is the
 * messy free-form version. geometry/delaunay_triangulation.c shows the same
 * idea in more detail, including the circles it draws through each triangle.
 *
 * References: Bowyer & Watson, both "Computing... tessellation" (1981);
 * de Berg et al., "Computational Geometry" (3e), §9; and Shewchuk's "Robust
 * Adaptive Floating-Point Geometric Predicates" (1996) for the inside-circle
 * test done with bullet-proof precision.
 */

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

/* ── §1 config ── */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define N_POINTS_DEFAULT 30
#define N_POINTS_MIN      6
#define N_POINTS_MAX     80

/* We kick things off with one enormous triangle that swallows the whole
 * screen, so every real point lands inside something. Its 3 corners take the
 * first 3 slots in the points array. */
#define N_SUPER          3
#define MAX_POINTS      (N_POINTS_MAX + N_SUPER)
#define MAX_TRIS        (MAX_POINTS * 4)
#define MAX_BOUNDARY    (MAX_TRIS * 3)

#define MARGIN_FRAC      0.08
#define N_THEMES         3

/* How quickly the on-screen fps number settles — small value = smooth. */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_EDGE   1
#define PAIR_POINT  2
#define PAIR_CURSOR 3
#define PAIR_HUD    4
#define PAIR_HINT   5

/* ── §2 clock ── */

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

/* ── §3 color ── */

static const short THEME_FG[N_THEMES][2] = {
    /* edge,  point */
    {  75, 226 },
    {  82, 196 },
    {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,  COLOR_YELLOW },
    { COLOR_GREEN, COLOR_RED    },
    { COLOR_WHITE, COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_p;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0];   fg_p = THEME_FG[theme][1];   }
    else               { fg_e = THEME_FG_8[theme][0]; fg_p = THEME_FG_8[theme][1]; }
    init_pair(PAIR_EDGE,   fg_e, -1);
    init_pair(PAIR_POINT,  fg_p, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — GridCtx + orient2d + in_circumcircle ── */

/* A single 2-D point, in sub-cell pixel units. */
typedef struct { double x, y; } Pt;

/* One triangle: its three corners as indices into the points array, plus a
 * flag we flip off when the triangle gets deleted (cheaper than shuffling
 * the array). valid == 0 means "ignore me". */
typedef struct { int v[3]; int valid; } Tri;

/*
 * GridCtx — the few screen numbers the drawing helpers need to share.
 *
 * It only describes the canvas (size, cell dimensions, where the centre is)
 * plus how many real points we asked for. The actual mesh of points and
 * triangles lives in the §5 globals, deliberately kept out of here so this
 * stays a small, plain description of the screen.
 */
typedef struct {
    int rows, cols;      /* terminal size, in characters */
    int cw, ch;          /* sub-cell width / height (pixels per character) */
    int ox, oy;          /* screen centre in pixels — handy for some helpers */
    int n_pts;           /* how many real points (not counting the 3 big-triangle corners) */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, int n_pts)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = (cols * CELL_W) / 2;
    g->oy = ((rows - 1) * CELL_H) / 2;
    g->n_pts = n_pts;
}

/*
 * Tells us which way the corners A, B, C wind: a positive result means
 * counter-clockwise, negative means clockwise, zero means they sit in a line.
 * The inside-circle test below only works on counter-clockwise triangles, so
 * we lean on this to keep them all turning the same way.
 */
static double orient2d(Pt A, Pt B, Pt C)
{
    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

/*
 * Imagine the one circle that passes through all three corners of triangle ABC.
 * Does point P fall inside that circle? Returns non-zero if it does. This is the
 * whole idea behind "Delaunay": a triangle only gets to survive if no other
 * point sits inside its circle. ABC must wind counter-clockwise for the test
 * below to give the right answer.
 */
static int in_circumcircle(Pt A, Pt B, Pt C, Pt P)
{
    double ax = A.x - P.x, ay = A.y - P.y;
    double bx = B.x - P.x, by = B.y - P.y;
    double cx = C.x - P.x, cy = C.y - P.y;
    double a2 = ax * ax + ay * ay;
    double b2 = bx * bx + by * by;
    double c2 = cx * cx + cy * cy;
    double det = a2 * (bx * cy - cx * by)
               - b2 * (ax * cy - cx * ay)
               + c2 * (ax * by - bx * ay);
    return det > 1e-9;
}

/* ── §5 mesh — Bowyer-Watson ── */

/* The one shared mesh: all the points, and all the triangles built from them.
 * It's global because the inside-circle test has to consult every triangle,
 * and that's the project's documented exception to "no big shared state". */
static Pt   g_pts[MAX_POINTS];
static int  g_n_pts;
static Tri  g_tris[MAX_TRIS];
static int  g_n_tris;

static void mesh_clear(void) { g_n_pts = 0; g_n_tris = 0; }

/*
 * Add a triangle, flipping two corners if needed so it always winds
 * counter-clockwise — the inside-circle test refuses to work otherwise.
 */
static void tri_add(int v0, int v1, int v2)
{
    if (g_n_tris >= MAX_TRIS) return;
    if (orient2d(g_pts[v0], g_pts[v1], g_pts[v2]) < 0.0) {
        int tmp = v1; v1 = v2; v2 = tmp;
    }
    g_tris[g_n_tris++] = (Tri){ {v0, v1, v2}, 1 };
}

/*
 * Lay down the starting scaffold: one huge triangle (corners 0..2) big enough
 * to cover the whole screen, so every real point we add later lands inside it.
 */
static void mesh_seed_super(double pw, double ph)
{
    double D = (pw > ph ? pw : ph) * 50.0;
    g_pts[0] = (Pt){ pw * 0.5,         ph * 0.5 - D };
    g_pts[1] = (Pt){ pw * 0.5 - D,     ph * 0.5 + D };
    g_pts[2] = (Pt){ pw * 0.5 + D,     ph * 0.5 + D };
    g_n_pts  = N_SUPER;
    g_n_tris = 0;
    tri_add(0, 1, 2);
}

/*
 * Drop one new point P into the mesh and re-stitch the triangles around it.
 * The trick (Bowyer-Watson): any triangle whose circle swallows P is now
 * wrong, so we delete all of them. That leaves a hole; the edges around the
 * rim of that hole are the ones that belonged to exactly one deleted triangle.
 * Fan those rim edges out to P to fill the hole back in with fresh triangles.
 */
static void mesh_insert(Pt P)
{
    if (g_n_pts >= MAX_POINTS) return;
    g_pts[g_n_pts] = P;
    int pid = g_n_pts;
    g_n_pts++;

    int bad[MAX_TRIS];
    int n_bad = 0;
    for (int i = 0; i < g_n_tris; i++) {
        if (!g_tris[i].valid) continue;
        Pt A = g_pts[g_tris[i].v[0]];
        Pt B = g_pts[g_tris[i].v[1]];
        Pt C = g_pts[g_tris[i].v[2]];
        if (in_circumcircle(A, B, C, P)) {
            g_tris[i].valid = 0;
            bad[n_bad++] = i;
        }
    }

    int boundary[MAX_BOUNDARY][2];
    int n_b = 0;
    for (int bi = 0; bi < n_bad; bi++) {
        int ti = bad[bi];
        for (int e = 0; e < 3; e++) {
            int a = g_tris[ti].v[e];
            int b = g_tris[ti].v[(e + 1) % 3];
            int shared = 0;
            for (int bj = 0; bj < n_bad; bj++) {
                if (bj == bi) continue;
                int tj = bad[bj];
                for (int f = 0; f < 3; f++) {
                    int aa = g_tris[tj].v[f];
                    int bb = g_tris[tj].v[(f + 1) % 3];
                    if ((aa == a && bb == b) || (aa == b && bb == a)) {
                        shared = 1; break;
                    }
                }
                if (shared) break;
            }
            if (!shared && n_b < MAX_BOUNDARY) {
                boundary[n_b][0] = a;
                boundary[n_b][1] = b;
                n_b++;
            }
        }
    }

    for (int i = 0; i < n_b; i++)
        tri_add(boundary[i][0], boundary[i][1], pid);
}

/*
 * Throw away the scaffolding: any triangle still clinging to a corner of the
 * giant starter triangle (the first N_SUPER points) is just leftover frame,
 * not a real result, so drop it.
 */
static void mesh_strip_super(void)
{
    for (int i = 0; i < g_n_tris; i++) {
        if (!g_tris[i].valid) continue;
        for (int j = 0; j < 3; j++) {
            if (g_tris[i].v[j] < N_SUPER) { g_tris[i].valid = 0; break; }
        }
    }
}

static int count_valid_tris(void)
{
    int n = 0;
    for (int i = 0; i < g_n_tris; i++) if (g_tris[i].valid) n++;
    return n;
}

/* ── §5b cursor — which input point is currently picked ── */

/*
 * Cursor — remembers which scattered point is currently picked.
 *
 * The points land at random, so there's no tidy left/right/up/down grid to
 * step through. Instead we just walk down the list with ',' / '.'. The picked
 * point is drawn as '@', and every triangle touching it lights up bold.
 *
 * point_idx is a slot in the points array, skipping the first N_SUPER slots
 * (those belong to the giant starter triangle, not to real points).
 */
typedef struct { int point_idx; } Cursor;

static void cursor_reset(Cursor *cur)
{
    cur->point_idx = (g_n_pts > N_SUPER) ? N_SUPER : -1;
}

static void cursor_advance(Cursor *cur, int dir)
{
    int n_input = g_n_pts - N_SUPER;
    if (n_input <= 0) { cur->point_idx = -1; return; }
    int rel = cur->point_idx - N_SUPER;
    rel = (rel + dir + n_input) % n_input;
    cur->point_idx = N_SUPER + rel;
}

/* True if the picked point is one of triangle ti's three corners. */
static int tri_has_point(int ti, int point_idx)
{
    if (point_idx < 0) return 0;
    return g_tris[ti].v[0] == point_idx
        || g_tris[ti].v[1] == point_idx
        || g_tris[ti].v[2] == point_idx;
}

/* ── §5c draw — pick a line glyph, then draw the line ── */

static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

static void line_draw(const GridCtx *g, double px0, double py0,
                      double px1, double py1, int attr)
{
    char ch = slope_char(px1 - px0, py1 - py0);
    int x0 = (int)(px0 / CELL_W), y0 = (int)(py0 / CELL_H);
    int x1 = (int)(px1 / CELL_W), y1 = (int)(py1 / CELL_H);
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    attron(attr);
    for (;;) {
        if (x0 >= 0 && x0 < g->cols && y0 >= 0 && y0 < g->rows - 1)
            mvaddch(y0, x0, (chtype)(unsigned char)ch);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    attroff(attr);
}

/*
 * Draw the whole web of triangle edges. Triangles touching the picked point
 * are highlighted; everything else uses the plain edge colour.
 */
static void ctx_draw_bg(const GridCtx *g, const Cursor *cur)
{
    for (int i = 0; i < g_n_tris; i++) {
        if (!g_tris[i].valid) continue;
        int hot  = tri_has_point(i, cur->point_idx);
        int attr = hot ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                       : COLOR_PAIR(PAIR_EDGE);
        Pt A = g_pts[g_tris[i].v[0]];
        Pt B = g_pts[g_tris[i].v[1]];
        Pt C = g_pts[g_tris[i].v[2]];
        line_draw(g, A.x, A.y, B.x, B.y, attr);
        line_draw(g, B.x, B.y, C.x, C.y, attr);
        line_draw(g, C.x, C.y, A.x, A.y, attr);
    }
}

static void cursor_draw(const GridCtx *g, const Cursor *cur)
{
    /* All input points as '*' */
    attron(COLOR_PAIR(PAIR_POINT) | A_BOLD);
    for (int i = N_SUPER; i < g_n_pts; i++) {
        int col = (int)(g_pts[i].x / CELL_W);
        int row = (int)(g_pts[i].y / CELL_H);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, '*');
    }
    attroff(COLOR_PAIR(PAIR_POINT) | A_BOLD);

    /* Selected point as '@' on top */
    if (cur->point_idx >= N_SUPER && cur->point_idx < g_n_pts) {
        int col = (int)(g_pts[cur->point_idx].x / CELL_W);
        int row = (int)(g_pts[cur->point_idx].y / CELL_H);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
            attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
            mvaddch(row, col, '@');
            attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        }
    }
}

/* ── §6 scene ── */

typedef struct {
    int          n_pts;
    int          theme;
    int          paused;
    unsigned int seed;
} Scene;

static double frand(unsigned int *s)
{
    *s = *s * 1103515245u + 12345u;
    return ((double)((*s >> 16) & 0x7FFF)) / 32767.0;
}

static void scene_seed_random(Scene *s, Cursor *cur, int rows, int cols)
{
    double pw = (double)cols * CELL_W;
    double ph = (double)(rows - 1) * CELL_H;
    double mx = pw * MARGIN_FRAC;
    double my = ph * MARGIN_FRAC;

    mesh_clear();
    mesh_seed_super(pw, ph);
    for (int i = 0; i < s->n_pts; i++) {
        Pt p = { mx + frand(&s->seed) * (pw - 2 * mx),
                 my + frand(&s->seed) * (ph - 2 * my) };
        mesh_insert(p);
    }
    mesh_strip_super();
    cursor_reset(cur);
}

static void scene_reset(Scene *s, Cursor *cur, int rows, int cols)
{
    s->n_pts  = N_POINTS_DEFAULT;
    s->theme  = 0;
    s->paused = 0;
    s->seed   = (unsigned int)time(NULL);
    scene_seed_random(s, cur, rows, cols);
}

static void hud_draw(const GridCtx *g, const Scene *s, const Cursor *cur, double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " pts:%d  tris:%d  cursor:%d  theme:%d  %5.1f fps  %s ",
             s->n_pts, count_valid_tris(),
             cur->point_idx >= 0 ? cur->point_idx - N_SUPER : -1,
             s->theme, fps, s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " ,/.:cursor  +/-:N  r:reseed  t:theme  p:pause  q:quit  [11 delaunay] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Scene *s, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g, cur);
    cursor_draw(g, cur);
    hud_draw(g, s, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §8 app ── */

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

    Scene  sc  = { 0 };
    Cursor cur = { -1 };
    sc.theme = 0;
    screen_init(sc.theme);

    GridCtx g;
    ctx_init(&g, LINES, COLS, N_POINTS_DEFAULT);
    scene_reset(&sc, &cur, LINES, COLS);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            scene_seed_random(&sc, &cur, LINES, COLS);
        }
        ctx_init(&g, LINES, COLS, sc.n_pts);

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           sc.paused ^= 1; break;
                case 'r':
                    sc.seed = (unsigned int)clock_ns();
                    scene_seed_random(&sc, &cur, g.rows, g.cols);
                    break;
                case 't':
                    sc.theme = (sc.theme + 1) % N_THEMES;
                    color_init(sc.theme);
                    break;
                case ',': case '<': cursor_advance(&cur, -1); break;
                case '.': case '>': cursor_advance(&cur, +1); break;
                case '+': case '=':
                    if (sc.n_pts < N_POINTS_MAX) {
                        sc.n_pts += 2; scene_seed_random(&sc, &cur, g.rows, g.cols);
                    } break;
                case '-':
                    if (sc.n_pts > N_POINTS_MIN) {
                        sc.n_pts -= 2; scene_seed_random(&sc, &cur, g.rows, g.cols);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &sc, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
