/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * neural_net_vis.c — neural network architecture visualiser (Stage 2)
 *
 * DEMO: Draws a feed-forward neural network as columns of `(O)` neurons
 *       fully connected by slope-character lines. One particle per input
 *       neuron drifts forward through the network — at each layer it
 *       picks a random target in the next layer and travels along that
 *       edge — and on reaching the output layer loops back to a random
 *       input neuron. Resize the terminal, change layer count with
 *       `[`/`]`, neurons-per-layer with `-`/`+`, line thickness with
 *       `<`/`>`; the particle pool reseeds whenever the network shape
 *       changes.
 *
 *       Stage 3 will add trails, fan-in pulse on neuron arrival, and
 *       optional weighted-random target selection.
 *
 * Study alongside: artistic/galaxy.c (similar dot-cluster + edge pattern),
 *                  artistic/graph_search.c (node + edge animation).
 *
 * Section map:
 *   §1 config    — MIN/MAX/DEFAULT layers, neurons, particle speed
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — bright + connection + particle foreground per theme
 *   §4 layout    — neuron_cell() coord seam + LINE_THICKNESS glyph table
 *   §5 net       — Net struct (sizes, thickness, theme)
 *   §6 particle  — Particle pool: reset / tick / draw + edge-by-edge hop
 *   §7 scene     — draw_connections + particle_draw + draw_neurons + HUD
 *   §8 screen    — ncurses init / cleanup
 *   §9 app       — signals, dt tracking, key handling, main loop
 *
 * Keys:  [/]  decrease/increase layer count (2..12)
 *        -/+  decrease/increase neurons per layer (2..16)
 *        </>  thinner/thicker connections (dot / thin / bold / heavy)
 *        t    cycle theme   r reset   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/neural_net_vis.c \
 *       -o neural_net_vis -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Pure layout — every layer is a vertical column at a
 *                 fixed x-fraction of the screen; every neuron is a cell
 *                 at a fixed y-fraction within the layer; every neuron in
 *                 layer i connects to every neuron in layer i+1. No data
 *                 flow yet — this stage only fixes the geometry.
 *
 * Data-structure: int n_layers, int n_per_layer in a Net struct. Neuron
 *                 positions are NOT stored — they're computed on demand
 *                 from (layer, idx, rows, cols). Resize is therefore
 *                 free: the next frame reads the new dimensions and
 *                 places neurons in the right place automatically.
 *
 * Rendering     : Per frame: erase, draw all O(L · N²) connections in a
 *                 dim colour pair, draw all O(L · N) neurons in a bright
 *                 pair over the connections, then HUD. Connections are
 *                 drawn by linear-step interpolation (cheaper than full
 *                 Bresenham, accurate enough at terminal resolution),
 *                 with the two endpoints skipped so the neuron glyph
 *                 lands cleanly on top.
 *
 * Performance   : O(layers · neurons²) line draws per frame — at the
 *                 12-layer × 16-neuron cap that's ~3000 short lines, well
 *                 under one millisecond on any modern CPU.
 *
 * References    :
 *   Goodfellow, Bengio & Courville, "Deep Learning" (2016) ch. 6 —
 *     standard feed-forward network diagram notation.
 *   Network architecture diagrams as visual notation —
 *     Schmidhuber, "Deep Learning in Neural Networks: An Overview"
 *     *Neural Networks* 61 (2015).
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <locale.h>
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

#define TARGET_FPS         30

#define MIN_LAYERS          2
#define MAX_LAYERS         12
#define DEFAULT_LAYERS      3

#define MIN_NEURONS         2
#define MAX_NEURONS        16
#define DEFAULT_NEURONS     5

#define MIN_THICKNESS       0
#define MAX_THICKNESS       3
#define DEFAULT_THICKNESS   1

/* One particle per input neuron. Speed is in edges-per-second; jitter
 * gives each particle a random factor in [1−J, 1+J] times the base. */
#define MAX_PARTICLES       MAX_NEURONS
#define PARTICLE_SPEED      0.6f
#define PARTICLE_JITTER     0.4f
#define PARTICLE_GLYPH      '*'
#define DT_CAP_S            0.10f

#define N_THEMES            4

/* Colour pair IDs */
#define PAIR_NEURON         1   /* bright (O) glyph                  */
#define PAIR_CONN           2   /* connection-line characters        */
#define PAIR_PARTICLE       3   /* travelling '*' dots               */
#define PAIR_HUD            4   /* status bar (top right)            */
#define PAIR_HINT           5   /* key hints (bottom left)           */

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

/* Per-theme [neuron_fg, connection_fg, particle_fg] for 256-color and
 * 8-color fallback. The particle accent is chosen to contrast with both
 * the neuron and connection tones so travelling dots stay visible
 * regardless of which line they're crossing. */
static const short THEME_FG_256[N_THEMES][3] = {
    {  51,  45, 226 },   /* cyan family    — aqua / teal   / gold      */
    {  82,  34, 220 },   /* green family   — lime / forest / amber     */
    { 220, 178, 207 },   /* amber family   — gold / honey  / pink      */
    { 207, 134,  51 },   /* magenta family — pink / orchid / aqua      */
};
static const short THEME_FG_8[N_THEMES][3] = {
    { COLOR_CYAN,    COLOR_CYAN,    COLOR_YELLOW  },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW  },
    { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int   x256   = (COLORS >= 256);
    short neuron = x256 ? THEME_FG_256[theme][0] : THEME_FG_8[theme][0];
    short conn   = x256 ? THEME_FG_256[theme][1] : THEME_FG_8[theme][1];
    short part   = x256 ? THEME_FG_256[theme][2] : THEME_FG_8[theme][2];
    init_pair(PAIR_NEURON,   neuron, -1);
    init_pair(PAIR_CONN,     conn,   -1);
    init_pair(PAIR_PARTICLE, part,   -1);
    init_pair(PAIR_HUD,      x256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,     x256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  layout — single coordinate seam                                     */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * neuron_cell — turn an abstract (layer, idx) address into a concrete
 * terminal cell. The whole network depends on this one function — change
 * it and the entire layout follows.
 *
 *   col = (layer + 1) · cols       / (n_layers + 1)
 *   row = (idx   + 1) · (rows - 1) / (n_in_layer + 1)
 *
 * The +1 in the denominator leaves equal margin at the top, bottom and
 * sides of the network. The (rows - 1) reserves the bottom row for the
 * key-hint strip.
 */
static void neuron_cell(int layer, int idx,
                        int n_layers, int n_in_layer,
                        int rows, int cols,
                        int *out_row, int *out_col)
{
    *out_col = (layer + 1) * cols       / (n_layers + 1);
    *out_row = (idx   + 1) * (rows - 1) / (n_in_layer + 1);
}

/*
 * Connection line style — glyph set plus an ncurses attribute, indexed
 * by thickness 0..3. Each entry maps the four step types (horizontal,
 * vertical, down-diagonal, up-diagonal) to glyph strings; multi-byte
 * UTF-8 is fine because draw_line uses mvaddstr (and screen_init calls
 * setlocale).
 *
 *   0 dot   — `.` everywhere, A_DIM            (faintest, exploratory)
 *   1 thin  — ASCII `- | \ /`                  (default)
 *   2 bold  — ASCII `- | \ /` + A_BOLD         (visible but slope-aware)
 *   3 heavy — Unicode `═ ║ ╲ ╱` + A_BOLD       (double-line horizontal/
 *                                               vertical + light Unicode
 *                                               diagonals — the visual
 *                                               jumps but the slope of
 *                                               every line stays legible)
 */
typedef struct {
    const char *hor, *ver, *dn, *up;
    attr_t      attr;
    const char *name;
} LineStyle;

static const LineStyle LINE_THICKNESS[] = {
    /* 0 */ { ".", ".", ".",  ".",  A_DIM,   "dot"   },
    /* 1 */ { "-", "|", "\\", "/",  0,       "thin"  },
    /* 2 */ { "-", "|", "\\", "/",  A_BOLD,  "bold"  },
    /* 3 */ { "═", "║", "╲",  "╱",  A_BOLD,  "heavy" },
};

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  net                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int n_layers;       /* MIN_LAYERS .. MAX_LAYERS                */
    int n_per_layer;    /* uniform width for now (extends easily)  */
    int thickness;      /* MIN_THICKNESS .. MAX_THICKNESS          */
    int theme;
    int paused;
} Net;

static void net_reset(Net *n)
{
    n->n_layers    = DEFAULT_LAYERS;
    n->n_per_layer = DEFAULT_NEURONS;
    n->thickness   = DEFAULT_THICKNESS;
    n->theme       = 0;
    n->paused      = 0;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  particle                                                            */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * A particle is a single bright dot travelling along ONE edge between
 * two adjacent layers. When it reaches the target neuron (t ≥ 1) it
 * picks a random target in the next layer and continues. When the next
 * layer would be past the output layer, it loops back to a random input
 * neuron — every particle therefore traces an endless input → output →
 * input → output trajectory.
 *
 * Pool size is fixed at the input-layer width: one particle per input
 * neuron. `particle_reset()` rebuilds the pool whenever the network
 * shape changes (layer count or neurons-per-layer).
 *
 * Each particle has a per-particle speed jitter and an initial random
 * `t` so the dots never bunch up into a single moving wavefront.
 */
typedef struct {
    int   from_layer;     /* 0 .. n_layers - 2                    */
    int   from_idx;       /* 0 .. n_per_layer - 1                  */
    int   to_idx;         /* 0 .. n_per_layer - 1 (next layer)     */
    float t;              /* progress along current edge, 0..1     */
    float speed;          /* edges per second                       */
} Particle;

typedef struct {
    Particle items[MAX_PARTICLES];
    int      count;
} ParticlePool;

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * particle_reset — one particle per input neuron, all starting at t=0
 * (right at the input neuron). Per-particle speed jitter desyncs them
 * within 2–3 edges so they don't stay in lock-step, while the t=0 start
 * makes the moment of reset visibly obvious — every particle snaps back
 * onto its input neuron and the fan-forward begins fresh.
 *
 * Called once at startup and again whenever the network shape changes
 * (layer count, neurons-per-layer) so the pool size and indices match
 * the new geometry.
 */
static void particle_reset(ParticlePool *p, const Net *n)
{
    p->count = n->n_per_layer;
    for (int i = 0; i < p->count; i++) {
        Particle *q  = &p->items[i];
        q->from_layer = 0;
        q->from_idx   = i;
        q->to_idx     = (int)(frand() * n->n_per_layer);
        q->t          = 0.0f;
        q->speed      = PARTICLE_SPEED *
                        (1.0f - PARTICLE_JITTER + 2.0f * PARTICLE_JITTER * frand());
    }
}

/*
 * particle_tick — advance every particle by speed·dt along its current
 * edge. On reaching t ≥ 1: hop to the next layer (carrying to_idx as
 * the new from_idx) or loop back to the input layer when at the output.
 *
 * The `while` loop catches the rare case where dt is large enough that
 * a particle traverses more than one edge in a single tick.
 */
static void particle_tick(ParticlePool *p, const Net *n, float dt)
{
    for (int i = 0; i < p->count; i++) {
        Particle *q = &p->items[i];
        q->t += q->speed * dt;
        while (q->t >= 1.0f) {
            q->t -= 1.0f;
            if (q->from_layer == n->n_layers - 2) {
                /* arrived at output — loop back to a random input neuron */
                q->from_layer = 0;
                q->from_idx   = (int)(frand() * n->n_per_layer);
                q->to_idx     = (int)(frand() * n->n_per_layer);
            } else {
                q->from_layer += 1;
                q->from_idx    = q->to_idx;
                q->to_idx      = (int)(frand() * n->n_per_layer);
            }
        }
    }
}

/*
 * particle_draw — interpolate between source and target neuron centres
 * and paint a bright glyph at the particle's current cell.
 */
static void particle_draw(const ParticlePool *p, const Net *n, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        const Particle *q = &p->items[i];
        int ar, ac, br, bc;
        neuron_cell(q->from_layer,     q->from_idx,
                    n->n_layers, n->n_per_layer, rows, cols, &ar, &ac);
        neuron_cell(q->from_layer + 1, q->to_idx,
                    n->n_layers, n->n_per_layer, rows, cols, &br, &bc);
        int sr = ar + (int)((br - ar) * q->t);
        int sc = ac + (int)((bc - ac) * q->t);
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddch(sr, sc, (chtype)PARTICLE_GLYPH);
    }
    attroff(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * draw_line — step from (r0,c0) to (r1,c1), drawing one character per
 * interior cell. The character comes from the LineStyle indexed by the
 * net's current thickness, with the slope (hor/ver/dn/up) chosen PER
 * CELL from the local step (dsr, dsc) — not from the global (dr, dc) —
 * so shallow lines render as rows of `-` linked by occasional `\` or
 * `/` at row changes, instead of `\\` pairs that read as parallel lines.
 *
 * Endpoints (i=0 and i=steps) are skipped so the neuron glyph (drawn
 * afterwards) lands on a clean background.
 */
static void draw_line(int r0, int c0, int r1, int c1,
                      int rows, int cols, const LineStyle *s)
{
    int dr = r1 - r0, dc = c1 - c0;
    int steps = (abs(dr) > abs(dc)) ? abs(dr) : abs(dc);
    if (steps <= 1) return;
    int prev_sr = r0, prev_sc = c0;
    for (int i = 1; i < steps; i++) {
        int sr = r0 + dr * i / steps;
        int sc = c0 + dc * i / steps;
        int dsr = sr - prev_sr;
        int dsc = sc - prev_sc;
        const char *ch;
        if      (dsr == 0)                ch = s->hor;
        else if (dsc == 0)                ch = s->ver;
        else if ((dsr > 0) == (dsc > 0))  ch = s->dn;
        else                              ch = s->up;
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddstr(sr, sc, ch);
        prev_sr = sr;
        prev_sc = sc;
    }
}

/*
 * draw_connections — every neuron in layer i to every neuron in layer i+1.
 * O(layers · neurons²) — cheap at our caps.
 */
static void draw_connections(const Net *n, int rows, int cols)
{
    const LineStyle *s = &LINE_THICKNESS[n->thickness];
    attron(COLOR_PAIR(PAIR_CONN) | s->attr);
    for (int i = 0; i < n->n_layers - 1; i++) {
        for (int a = 0; a < n->n_per_layer; a++) {
            int ar, ac;
            neuron_cell(i, a, n->n_layers, n->n_per_layer, rows, cols, &ar, &ac);
            for (int b = 0; b < n->n_per_layer; b++) {
                int br, bc;
                neuron_cell(i + 1, b, n->n_layers, n->n_per_layer, rows, cols, &br, &bc);
                draw_line(ar, ac, br, bc, rows, cols, s);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_CONN) | s->attr);
}

/*
 * draw_neurons — bright `(O)` 3-cell sigil at every neuron centre. The
 * parens give the neuron a clear visual border that reads as a circle
 * even in fonts where 'O' alone looks like a regular letter.
 */
static void draw_neurons(const Net *n, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
    for (int i = 0; i < n->n_layers; i++) {
        for (int j = 0; j < n->n_per_layer; j++) {
            int sr, sc;
            neuron_cell(i, j, n->n_layers, n->n_per_layer, rows, cols, &sr, &sc);
            if (sr < 0 || sr >= rows - 1) continue;
            if (sc - 1 >= 0  )  mvaddch(sr, sc - 1, '(');
            if (sc     <  cols)  mvaddch(sr, sc,     'O');
            if (sc + 1 <  cols)  mvaddch(sr, sc + 1, ')');
        }
    }
    attroff(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Net *n,
                       const ParticlePool *p, double fps)
{
    erase();
    draw_connections(n, rows, cols);
    particle_draw(p, n, rows, cols);   /* over connections, under neurons */
    draw_neurons(n, rows, cols);

    /* HUD — top right */
    char buf[112];
    snprintf(buf, sizeof buf,
             " layers:%d  neurons:%d  thick:%s  theme:%d  %5.1f fps  %s ",
             n->n_layers, n->n_per_layer,
             LINE_THICKNESS[n->thickness].name, n->theme, fps,
             n->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Key hints — bottom left */
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:layers  -/+:neurons  </>:thick  t:theme  r:reset  p:pause  q:quit  [neural_net_vis stage 1] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    /* Activate the user's locale so mvaddstr renders multi-byte UTF-8
     * line characters correctly (used by the heavy thickness level). */
    setlocale(LC_ALL, "");
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  app                                                                 */
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

    Net          net;  net_reset(&net);
    ParticlePool pool; particle_reset(&pool, &net);
    screen_init(net.theme);

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

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
                case 'p':           net.paused ^= 1; break;
                case 'r':           net_reset(&net); particle_reset(&pool, &net);
                                    color_init(net.theme); break;
                case 't':           net.theme = (net.theme + 1) % N_THEMES;
                                    color_init(net.theme); break;
                case '[':           if (net.n_layers > MIN_LAYERS) {
                                        net.n_layers--; particle_reset(&pool, &net);
                                    } break;
                case ']':           if (net.n_layers < MAX_LAYERS) {
                                        net.n_layers++; particle_reset(&pool, &net);
                                    } break;
                case '-':           if (net.n_per_layer > MIN_NEURONS) {
                                        net.n_per_layer--; particle_reset(&pool, &net);
                                    } break;
                case '+': case '=': if (net.n_per_layer < MAX_NEURONS) {
                                        net.n_per_layer++; particle_reset(&pool, &net);
                                    } break;
                case ',': case '<': if (net.thickness > MIN_THICKNESS) net.thickness--; break;
                case '.': case '>': if (net.thickness < MAX_THICKNESS) net.thickness++; break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;   /* spiral-of-death cap */
        t_tick_prev = now;
        if (!net.paused) particle_tick(&pool, &net, dt);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &net, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
