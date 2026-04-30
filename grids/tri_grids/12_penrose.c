/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 12_penrose.c — Penrose-style aperiodic substitution with Robinson triangles
 *
 * DEMO: Two golden-ratio isoceles triangles — "acute" (apex 36°) and
 *       "obtuse" (apex 108°) — recursively substitute into smaller
 *       copies of themselves with the golden ratio φ. Use +/- to change
 *       depth (0..10). Triangle count grows roughly as 2^N. The result
 *       is a fragment of a Penrose P3 tiling: aperiodic, with 5-fold
 *       rotational symmetry on average and self-similar at scale 1/φ.
 *
 * Study alongside: 10_pinwheel.c — another aperiodic substitution
 *                  (5-way split with √5 scaling).
 *                  09_sierpinski.c — periodic 3-way self-similar split.
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, DEPTH, SIZE_FRAC, PHI
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — type-keyed palette (acute vs obtuse) + HUD / hint
 *   §4 formula  — slope_char + Bresenham line_draw
 *   §5 substitute — golden-ratio split for each Robinson triangle
 *   §6 scene    — seed_triangle (acute or obtuse) + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  +/- depth   [/] size   s swap-seed-type   r reset
 *        t theme   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/12_penrose.c \
 *       -o 12_penrose -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Robinson-triangle substitution. Two prototiles:
 *                    Acute (A): isoceles, apex 36°, base angles 72° each,
 *                              leg = φ, base = 1.
 *                    Obtuse (B): isoceles, apex 108°, base angles 36° each,
 *                              leg = 1, base = φ.
 *                  Substitution rule (the "half-rhomb" 2/2 version):
 *                    A → B + A    (B at full scale, A at scale 1/φ)
 *                    B → A + B    (both at scale 1/φ)
 *
 *                  NOT a strict Penrose P3 inflation rule — strict P3 uses
 *                  A→A+B and B→A+2B with all children at scale 1/φ. The
 *                  2/2 simplification produces an aperiodic-looking
 *                  golden-ratio fractal that demonstrates the same key
 *                  ideas (two prototiles, golden split, self-similarity).
 *
 * Formula        : Place a split point P on a chosen edge at the golden-
 *                  ratio fraction (1/φ along the leg or base). The two
 *                  children share the apex of one of the parent's triangles
 *                  with this new vertex; see §5 substitute for explicit
 *                  vertex lists.
 *
 * Edge chars     : Same Bresenham + slope_char as 07-11. Each leaf draws
 *                  3 edges in its prototype color (acute or obtuse).
 *
 * Movement       : None. Depth controlled by +/-, seed type by 's'.
 *
 * References     :
 *   Penrose, "Pentaplexity" (1979)
 *   Robinson, "Undecidability and Nonperiodicity for Tilings" (1971)
 *   Senechal, "Quasicrystals and Geometry" (1995) §7
 *   Penrose tiling — https://en.wikipedia.org/wiki/Penrose_tiling
 *   Golden ratio   — https://en.wikipedia.org/wiki/Golden_ratio
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Penrose tilings are aperiodic — they tile the plane without ever
 * repeating. The Robinson-triangle decomposition expresses Penrose P3
 * (rhomb tilings) as just TWO triangle types (acute and obtuse), both
 * isoceles with golden-ratio side lengths. A substitution rule maps each
 * triangle to a SMALLER COMBINATION of the same two types. Recursing
 * the rule produces the aperiodic pattern at any chosen "magnification".
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Acute (A): apex angle 36°, isoceles, legs = φ × base. Picture a tall,
 * thin isoceles triangle.
 *
 * Obtuse (B): apex angle 108°, isoceles, base = φ × legs. Picture a wide,
 * shallow isoceles triangle.
 *
 * Both are "Robinson triangles" — specifically the golden gnomon and
 * golden triangle. Their shared link is the golden ratio φ ≈ 1.618.
 *
 * The substitution rule places a new vertex P at golden-ratio position
 * along one edge. P, together with the existing vertices, defines two
 * smaller triangles: one of type A and one of type B. Their sides also
 * form 1:φ ratios because P split a side that was already a Fibonacci-
 * related length.
 *
 * Why aperiodic? Because the substitution scales by 1/φ — an irrational
 * factor. Any periodic pattern would need rational scaling. So no
 * translation symmetry can exist in the limit; the tiling is forced to
 * be aperiodic.
 *
 * Why "half-rhomb"? Two acutes joined at their bases form a thin Penrose
 * rhomb (36°-144°-36°-144°). Two obtuses joined at their bases form a
 * fat Penrose rhomb (72°-108°-72°-108°). The rhomb-level Penrose tiling
 * IS the union of these triangles.
 *
 * DRAWING METHOD  (recursive emit)
 * ──────────────
 *  1. Pick DEPTH and SIZE_FRAC.
 *  2. Build the seed triangle (A or B based on user choice).
 *  3. substitute(t, depth):
 *       if depth == max_depth: draw 3 edges in t's prototype color
 *       else:
 *         compute P at golden-ratio position
 *         build 2 children (A + B for A-parent; A + B for B-parent)
 *         recurse on each
 *  4. Bresenham line_draw per leaf edge.
 *
 * KEY FORMULAS
 * ────────────
 *  Golden ratio:  φ = (1 + √5) / 2 ≈ 1.6180339887
 *  Inverse:       1/φ = φ − 1 ≈ 0.6180339887
 *
 *  ACUTE (legs φ, base 1; apex 36° at index 0):
 *    Place P on segment apex→base[1] at fraction 1/φ from apex.
 *    (V0 to V1 has length φ; V0-P = 1, P-V1 = 1/φ.)
 *
 *    Child #1 (OBTUSE): apex at P (108°), base = V0-V2.
 *      sides |V0-V2| = φ, |P-V0| = 1, |P-V2| = 1.
 *
 *    Child #2 (ACUTE):  apex at V2 (36°), base = P-V1.
 *      sides |V2-V1| = 1, |V2-P| = 1, |P-V1| = 1/φ.
 *
 *  OBTUSE (legs 1, base φ; apex 108° at index 0):
 *    Place P on base[0]→base[1] at fraction 1/φ from base[0].
 *    (V1 to V2 has length φ; V1-P = 1, P-V2 = 1/φ.)
 *
 *    Child #1 (ACUTE):  apex at V1 (36°), base = V0-P.
 *      sides |V1-V0| = 1, |V1-P| = 1, |V0-P| = 1/φ.
 *
 *    Child #2 (OBTUSE): apex at P (108°), base = V0-V2.
 *      sides |V0-V2| = 1, |P-V0| = 1/φ, |P-V2| = 1/φ.
 *
 *  Vertex convention :
 *    t.v[0] is always the APEX (36° for acute, 108° for obtuse).
 *    t.v[1], t.v[2] are the base corners.
 *    Substitutions preserve this so children inherit it cleanly.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Depth grows leaf count as 2^N. At N=10, 1024 leaves; still fast.
 *    Strict Penrose P3 with 2/3 child counts grows faster (3^N for B).
 *  • Different seed types (A vs B) produce different patterns; toggle
 *    with 's' to compare.
 *  • The aperiodicity is approximate at finite depth — a periodic eye
 *    might still find motifs. The true aperiodicity is the LIMIT as N→∞.
 *  • Leaf colors are by TYPE (acute vs obtuse), not by depth — to make
 *    the prototile structure visually obvious.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At depth 0: 1 leaf — the seed.
 *  At depth 1: 2 leaves — one acute and one obtuse, geometrically
 *    arranged inside the parent.
 *  At depth N: 2^N leaves total.
 *
 *  Visual check at depth ≥ 4: you should see the characteristic Penrose
 *  motifs (sun, star, kite, dart) once enough leaves have accumulated
 *  to spell them out. The 5-fold symmetry on average is approximate at
 *  finite depth.
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

#define PHI     1.6180339887498949
#define INV_PHI 0.6180339887498949

#define DEPTH_DEFAULT 4
#define DEPTH_MIN     0
#define DEPTH_MAX    10

#define SIZE_FRAC_DEFAULT 0.85
#define SIZE_FRAC_MIN     0.30
#define SIZE_FRAC_MAX     0.95
#define SIZE_FRAC_STEP    0.05

#define N_THEMES 3

#define PAIR_ACUTE  1
#define PAIR_OBTUSE 2
#define PAIR_HUD    3
#define PAIR_HINT   4

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
    /* acute, obtuse */
    {  39, 207 },
    { 226,  21 },
    {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_MAGENTA },
    { COLOR_YELLOW,  COLOR_BLUE    },
    { COLOR_WHITE,   COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_a, fg_b;
    if (COLORS >= 256) { fg_a = THEME_FG[theme][0];   fg_b = THEME_FG[theme][1];   }
    else               { fg_a = THEME_FG_8[theme][0]; fg_b = THEME_FG_8[theme][1]; }
    init_pair(PAIR_ACUTE,  fg_a, -1);
    init_pair(PAIR_OBTUSE, fg_b, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — slope_char + Bresenham line                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

static void line_draw(int rows, int cols, double px0, double py0,
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
        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows - 1)
            mvaddch(y0, x0, (chtype)(unsigned char)ch);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    attroff(attr);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  substitute — golden-ratio split for Robinson triangles              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TYPE_ACUTE   0
#define TYPE_OBTUSE  1

typedef struct {
    double x[3], y[3];      /* x[0]/y[0] = apex, x[1..2]/y[1..2] = base */
    int    type;            /* TYPE_ACUTE or TYPE_OBTUSE */
} PTri;

static void tri_draw_edges(int rows, int cols, PTri t)
{
    int pair = (t.type == TYPE_ACUTE) ? PAIR_ACUTE : PAIR_OBTUSE;
    int attr = COLOR_PAIR(pair);
    line_draw(rows, cols, t.x[0], t.y[0], t.x[1], t.y[1], attr);
    line_draw(rows, cols, t.x[1], t.y[1], t.x[2], t.y[2], attr);
    line_draw(rows, cols, t.x[2], t.y[2], t.x[0], t.y[0], attr);
}

/*
 * substitute — recursive Robinson-triangle inflation.
 *
 *   ACUTE (legs φ, base 1; apex 36° at index 0):
 *     P on apex→base[1] at fraction 1/φ from apex.
 *
 *     Child OBTUSE: apex at P, base V0-V2.
 *     Child ACUTE:  apex at V2, base P-V1.
 *
 *   OBTUSE (legs 1, base φ; apex 108° at index 0):
 *     P on base[0]→base[1] at fraction 1/φ from base[0].
 *
 *     Child ACUTE:  apex at V1, base V0-P.
 *     Child OBTUSE: apex at P, base V0-V2.
 */
static void substitute(int rows, int cols, PTri t, int depth, int max_depth)
{
    if (depth == max_depth) { tri_draw_edges(rows, cols, t); return; }

    if (t.type == TYPE_ACUTE) {
        double Px = t.x[0] + (t.x[1] - t.x[0]) * INV_PHI;
        double Py = t.y[0] + (t.y[1] - t.y[0]) * INV_PHI;
        PTri c0 = { { Px, t.x[0], t.x[2] }, { Py, t.y[0], t.y[2] }, TYPE_OBTUSE };
        PTri c1 = { { t.x[2], Px, t.x[1] }, { t.y[2], Py, t.y[1] }, TYPE_ACUTE  };
        substitute(rows, cols, c0, depth + 1, max_depth);
        substitute(rows, cols, c1, depth + 1, max_depth);
    } else {
        double Px = t.x[1] + (t.x[2] - t.x[1]) * INV_PHI;
        double Py = t.y[1] + (t.y[2] - t.y[1]) * INV_PHI;
        PTri c0 = { { t.x[1], t.x[0], Px }, { t.y[1], t.y[0], Py }, TYPE_ACUTE  };
        PTri c1 = { { Px, t.x[0], t.x[2] }, { Py, t.y[0], t.y[2] }, TYPE_OBTUSE };
        substitute(rows, cols, c0, depth + 1, max_depth);
        substitute(rows, cols, c1, depth + 1, max_depth);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    depth;
    double size_frac;
    int    seed_type;       /* TYPE_ACUTE or TYPE_OBTUSE */
    int    theme;
    int    paused;
} Scene;

static void scene_reset(Scene *s)
{
    s->depth     = DEPTH_DEFAULT;
    s->size_frac = SIZE_FRAC_DEFAULT;
    s->seed_type = TYPE_ACUTE;
    s->theme     = 0;
    s->paused    = 0;
}

/*
 * Build the seed triangle. Two flavors: an acute or obtuse Robinson tile
 * centered on screen, sized to fit. Apex always at the top.
 *
 *   ACUTE: leg = base · φ, apex angle 36°. Height = leg · sin(72°).
 *   OBTUSE: base = leg · φ, apex angle 108°. Height = leg · sin(36°).
 */
static PTri scene_seed(const Scene *s, int rows, int cols)
{
    double pw = (double)cols * CELL_W;
    double ph = (double)(rows - 1) * CELL_H;
    double cxp = pw * 0.5;
    double cyp = ph * 0.5;

    if (s->seed_type == TYPE_ACUTE) {
        double leg    = (pw < ph ? pw : ph) * s->size_frac * 0.5;
        double base   = leg / PHI;
        double height = leg * sin(72.0 * M_PI / 180.0);
        PTri t = {
            { cxp,                cxp - base * 0.5,    cxp + base * 0.5 },
            { cyp - height * 0.5, cyp + height * 0.5,  cyp + height * 0.5 },
            TYPE_ACUTE
        };
        return t;
    } else {
        double leg    = (pw < ph ? pw : ph) * s->size_frac * 0.4;
        double base   = leg * PHI;
        double height = leg * sin(36.0 * M_PI / 180.0);
        PTri t = {
            { cxp,                cxp - base * 0.5,    cxp + base * 0.5 },
            { cyp - height * 0.5, cyp + height * 0.5,  cyp + height * 0.5 },
            TYPE_OBTUSE
        };
        return t;
    }
}

static void scene_draw(int rows, int cols, const Scene *s, double fps)
{
    erase();
    PTri seed = scene_seed(s, rows, cols);
    substitute(rows, cols, seed, 0, s->depth);

    long leaves = 1; for (int i = 0; i < s->depth; i++) leaves *= 2;
    char buf[128];
    snprintf(buf, sizeof buf,
             " seed:%s  depth:%d  leaves~%ld  size:%.2f  theme:%d  %5.1f fps  %s ",
             s->seed_type == TYPE_ACUTE ? "acute(36°)" : "obtuse(108°)",
             s->depth, leaves, s->size_frac, s->theme, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " +/-:depth  [/]:size  s:swap-seed  t:theme  r:reset  p:pause  q:quit  [12 penrose] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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

    Scene sc;
    scene_reset(&sc);
    screen_init(sc.theme);

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           sc.paused ^= 1; break;
                case 'r':           scene_reset(&sc); color_init(sc.theme); break;
                case 's':
                    sc.seed_type = (sc.seed_type == TYPE_ACUTE)
                                    ? TYPE_OBTUSE : TYPE_ACUTE;
                    break;
                case 't':
                    sc.theme = (sc.theme + 1) % N_THEMES;
                    color_init(sc.theme);
                    break;
                case '+': case '=':
                    if (sc.depth < DEPTH_MAX) { sc.depth++; } break;
                case '-':
                    if (sc.depth > DEPTH_MIN) { sc.depth--; } break;
                case '[':
                    if (sc.size_frac > SIZE_FRAC_MIN) { sc.size_frac -= SIZE_FRAC_STEP; } break;
                case ']':
                    if (sc.size_frac < SIZE_FRAC_MAX) { sc.size_frac += SIZE_FRAC_STEP; } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &sc, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
