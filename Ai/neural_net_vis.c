/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * neural_net_vis.c — draws a feed-forward neural network in the terminal: rows
 * of (O) neurons in columns, every neuron wired to every neuron in the next
 * column, with little dots drifting forward along the wires.  It's just the
 * picture — no real learning.  Sister demos: artistic/galaxy.c and
 * artistic/graph_search.c (dots-and-edges animations).
 */

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

/* ── §1  config — all the tunable numbers ────────────────────────────── */

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

/* the drifting dots: one per input neuron.  Speed is in wires-per-second, and
 * jitter makes each dot a little faster or slower than the rest. */
#define MAX_PARTICLES       MAX_NEURONS
#define PARTICLE_SPEED      0.6f
#define PARTICLE_JITTER     0.4f
#define PARTICLE_GLYPH      '*'
#define DT_CAP_S            0.10f

#define N_THEMES            4

/* ── §1.1 timing + smoothed fps ──────────────────────────────────────── */

#define NS_PER_SEC          1000000000LL

/* The shown fps is smoothed so the digits don't flicker: keep mostly the last
 * reading and mix in a little of the newest (averages over ~20 frames). */
#define EWMA_RETAIN         0.95
#define EWMA_NEW            0.05

/* ── §1.2 how a neuron and the dots are drawn ────────────────────────── */

/* a neuron is drawn as three characters, ( O ).  NEURON_SIGIL_HALF is how many
 * cells the parens sit out from the centre 'O'. */
#define NEURON_SIGIL_HALF   1
#define NEURON_GLYPH_LEFT   '('
#define NEURON_GLYPH_CENTRE 'O'
#define NEURON_GLYPH_RIGHT  ')'

/* a dot only travels along wires that LEAVE a layer, so the last layer (no
 * outgoing wire) can't be a starting layer — that's the gap to subtract. */
#define LAYER_EDGE_TAIL     2

/* Colour pair IDs */
#define PAIR_NEURON         1   /* bright (O) glyph                  */
#define PAIR_CONN           2   /* connection-line characters        */
#define PAIR_PARTICLE       3   /* travelling '*' dots               */
#define PAIR_HUD            4   /* status bar (top right)            */
#define PAIR_HINT           5   /* key hints (bottom left)           */

/* ── §2  clock — read the time and sleep ─────────────────────────────── *
 * The monotonic clock only ever counts forward, so it won't jump if the system
 * time changes. */

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

/* ── §3  color — the per-theme colours ───────────────────────────────── */

/* Each theme gives three colours — for the neurons, the wires, and the dots —
 * with a 256-colour set and an 8-colour fallback.  The dot colour is picked to
 * stand out against both the others so the dots stay visible on any wire. */
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

/* HUD colours: bright yellow for the status line, bright cyan for the key
 * hints — kept out of the themes (and always bold, never dim) so they stay
 * readable over the animation no matter which theme is on. */
#define HUD_FG_256          226    /* bright yellow */
#define HINT_FG_256          51    /* bright cyan   */

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
    init_pair(PAIR_HUD,      x256 ? HUD_FG_256  : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,     x256 ? HINT_FG_256 : COLOR_CYAN,   -1);
}

/* ── §4  layout — where each neuron sits on screen ───────────────────── */

/* where a neuron lands on screen, from its (layer, index).  Layers spread
 * evenly across the width and neurons evenly down the height, with a matching
 * margin all round; the bottom row is left for the key hints.  This is the only
 * place positions are decided — so the whole layout follows from it, and resize
 * is free since no positions are stored. */
static void neuron_cell(int layer, int idx,
                        int n_layers, int n_in_layer,
                        int rows, int cols,
                        int *out_row, int *out_col)
{
    *out_col = (layer + 1) * cols       / (n_layers + 1);
    *out_row = (idx   + 1) * (rows - 1) / (n_in_layer + 1);
}

/* LineStyle — one look for the connection wires.  A wire is drawn cell by cell;
 * at each cell we pick a character for the local direction (flat, vertical, or a
 * down/up diagonal) so a sloping wire reads as a real slope instead of a
 * staircase of dashes.  The '<' / '>' keys step through the four looks below.
 * The "heavy" look uses UTF-8 box characters, which print fine as long as
 * setlocale() ran first (it does, in screen_init).
 *
 *   0 dot   — faint dots          1 thin  — plain ASCII lines (default)
 *   2 bold  — the same, bolder    3 heavy — UTF-8 box lines, bold */
typedef struct {
    const char *hor;    /* character for a flat (horizontal) step  */
    const char *ver;    /* character for a vertical step           */
    const char *dn;     /* character for a down-sloping diagonal    */
    const char *up;     /* character for an up-sloping diagonal     */
    attr_t      attr;   /* bold / dim / normal, mixed in when drawing */
    const char *name;   /* the label shown in the HUD ("thin"…)    */
} LineStyle;

static const LineStyle LINE_THICKNESS[] = {
    /* 0 */ { ".", ".", ".",  ".",  A_DIM,   "dot"   },
    /* 1 */ { "-", "|", "\\", "/",  0,       "thin"  },
    /* 2 */ { "-", "|", "\\", "/",  A_BOLD,  "bold"  },
    /* 3 */ { "═", "║", "╲",  "╱",  A_BOLD,  "heavy" },
};

/* ── §5  net — the program's state, split into small pieces ──────────── */

/* NetArch — the network's shape: how many layers, and how many neurons in each.
 * Everything else (positions, wires, how many dots) is worked out from these two
 * numbers.  IMPORTANT: after changing either field you must call
 * particle_reset(), since the number of dots depends on the layer width — the
 * '[' ']' '-' '+' keys all do this.  (Every layer is the same width here, just
 * to keep it simple; real networks vary.) */
typedef struct {
    int n_layers;    /* how many layers, 2..12 (default 3); '[' / ']' change it  */
    int n_per_layer; /* neurons per layer, 2..16 (default 5); '-' / '+' change it */
} NetArch;

/* RenderConfig — looks only.  Changing these repaints the same network in a
 * different style; they never change its shape, so they can't need a reset.
 * (Rule of thumb when adding a setting: if it changes WHAT is drawn it belongs
 * in NetArch; if it only changes how it looks, it belongs here.) */
typedef struct {
    int thickness; /* which wire look (§4), 0..3 (default 1 "thin"); '<' / '>' */
    int theme;     /* which colour theme, 0..3 (default 0); 't' cycles it      */
} RenderConfig;

/* NetUI — the human's own toggles, kept apart from the network's state. */
typedef struct {
    int paused; /* 'p' freezes the dots; drawing keeps going so you can study a frame */
} NetUI;

static void arch_reset(NetArch *a)
{
    a->n_layers    = DEFAULT_LAYERS;
    a->n_per_layer = DEFAULT_NEURONS;
}

static void render_reset(RenderConfig *r)
{
    r->thickness = DEFAULT_THICKNESS;
    r->theme     = 0;
}

static void ui_reset(NetUI *u)
{
    u->paused = 0;
}

/* totals worked out from the shape (not stored), for the HUD: how many neurons,
 * and how many wires — every neuron joins every neuron in the next layer. */
static inline int net_total_neurons(const NetArch *a)
{
    return a->n_layers * a->n_per_layer;
}

static inline int net_total_edges(const NetArch *a)
{
    return (a->n_layers - 1) * a->n_per_layer * a->n_per_layer;
}

/* ── §6  particle — the dots that drift along the wires ──────────────── */

/* Particle — one bright dot in flight along a single wire between two
 * neighbouring layers.  It just shows "something flowing forward" — there's no
 * real maths.  Its trip: start at an input neuron, hop one wire at a time toward
 * the output, then loop back to a random input and go again. */
typedef struct {
    int   from_layer; /* the layer it set off from this hop (never the last)     */
    int   from_idx;   /* which neuron in that layer it left                      */
    int   to_idx;     /* which neuron in the next layer it's heading to          */
    float t;          /* how far along the wire: 0 at the start, 1 on arrival    */
    float speed;      /* its own pace in wires/sec — jittered so the dots desync */
} Particle;

/* ParticlePool — a fixed array of dots (one per input neuron) and how many are
 * live.  Sized to the biggest possible input layer (MAX_NEURONS) so it never has
 * to allocate while running.  particle_reset rebuilds it on any shape change. */
typedef struct {
    Particle items[MAX_PARTICLES]; /* room for the most we'd ever need; only [0..count) are live */
    int      count;                /* how many dots are live now (= neurons per layer) */
} ParticlePool;

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

/* (re)seed the dots: one per input neuron, all snapped to the start right on
 * their input neuron — so a reset is plainly visible.  Each gets a slightly
 * different speed so they don't move in lock-step.  Call at startup and after
 * any shape change, so the dot count and indices match the new network. */
static void particle_reset(ParticlePool *p, const NetArch *a)
{
    p->count = a->n_per_layer;
    for (int i = 0; i < p->count; i++) {
        Particle *q  = &p->items[i];
        q->from_layer = 0;
        q->from_idx   = i;
        q->to_idx     = (int)(frand() * a->n_per_layer);
        q->t          = 0.0f;
        q->speed      = PARTICLE_SPEED *
                        (1.0f - PARTICLE_JITTER + 2.0f * PARTICLE_JITTER * frand());
    }
}

/* pick a random neuron in a layer (a dot's next target) */
static inline int random_target_in_layer(const NetArch *a)
{
    return (int)(frand() * a->n_per_layer);
}

/* has this dot just reached the last layer? (its current wire is the final one) */
static inline bool particle_at_output_layer(const Particle *q, const NetArch *a)
{
    return q->from_layer == a->n_layers - LAYER_EDGE_TAIL;
}

/* move one dot forward by a single wire: step into the next layer, or — if it's
 * already at the output — jump back to a random input neuron to start over. */
static void particle_hop_or_loop(Particle *q, const NetArch *a)
{
    if (particle_at_output_layer(q, a)) {
        /* output reached — re-seed at a random input neuron */
        q->from_layer = 0;
        q->from_idx   = random_target_in_layer(a);
    } else {
        /* forward hop — current target becomes the next source */
        q->from_layer += 1;
        q->from_idx    = q->to_idx;
    }
    q->to_idx = random_target_in_layer(a);
}

/* move every dot forward by its speed.  When one reaches the end of a wire it
 * hops to the next; the while loop covers the rare case where a big time step
 * carries it across two wires in one go. */
static void particle_tick(ParticlePool *p, const NetArch *a, float dt)
{
    for (int i = 0; i < p->count; i++) {
        Particle *q = &p->items[i];
        q->t += q->speed * dt;
        while (q->t >= 1.0f) {
            q->t -= 1.0f;
            particle_hop_or_loop(q, a);
        }
    }
}

/* where a dot is right now: slide from its start neuron toward its target by the
 * fraction t, and round to a screen cell. */
static void particle_interp_cell(const Particle *q, const NetArch *a,
                                 int rows, int cols,
                                 int *out_sr, int *out_sc)
{
    int ar, ac, br, bc;
    neuron_cell(q->from_layer,     q->from_idx,
                a->n_layers, a->n_per_layer, rows, cols, &ar, &ac);
    neuron_cell(q->from_layer + 1, q->to_idx,
                a->n_layers, a->n_per_layer, rows, cols, &br, &bc);
    *out_sr = ar + (int)((br - ar) * q->t);
    *out_sc = ac + (int)((bc - ac) * q->t);
}

/* paint each dot as a bright '*'.  Drawn after the wires but before the neurons,
 * so a neuron cleanly covers a dot sitting right on it. */
static void particle_draw(const ParticlePool *p, const NetArch *a,
                          int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sr, sc;
        particle_interp_cell(&p->items[i], a, rows, cols, &sr, &sc);
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddch(sr, sc, (chtype)PARTICLE_GLYPH);
    }
    attroff(COLOR_PAIR(PAIR_PARTICLE) | A_BOLD);
}

/* ── §7  scene — draw the whole picture, plus the HUD ────────────────── */

/* choose the wire character for this cell from the step we just took (flat,
 * vertical, or a down/up diagonal).  Picking it per cell, instead of from the
 * wire's overall slope, makes a gentle slope read as a long flat run with the
 * odd diagonal where it steps — which looks right. */
static const char *pick_line_glyph(int dsr, int dsc, const LineStyle *s)
{
    if (dsr == 0)                    return s->hor;
    if (dsc == 0)                    return s->ver;
    if ((dsr > 0) == (dsc > 0))      return s->dn;
    return s->up;
}

/* draw a straight wire from (r0,c0) to (r1,c1), one character per cell.  The two
 * end cells are skipped so the neuron drawn there later sits on clean
 * background. */
static void draw_line(int r0, int c0, int r1, int c1,
                      int rows, int cols, const LineStyle *s)
{
    int dr    = r1 - r0, dc = c1 - c0;
    int steps = (abs(dr) > abs(dc)) ? abs(dr) : abs(dc);
    if (steps <= 1) return;

    int prev_sr = r0, prev_sc = c0;
    for (int i = 1; i < steps; i++) {
        int sr  = r0 + dr * i / steps;
        int sc  = c0 + dc * i / steps;
        int dsr = sr - prev_sr;
        int dsc = sc - prev_sc;

        const char *glyph = pick_line_glyph(dsr, dsc, s);
        if (sr >= 0 && sr < rows - 1 && sc >= 0 && sc < cols)
            mvaddstr(sr, sc, glyph);

        prev_sr = sr;
        prev_sc = sc;
    }
}

/* wire every neuron to every neuron in the next layer. */
static void draw_connections(const NetArch *arch, const RenderConfig *rc,
                             int rows, int cols)
{
    const LineStyle *s = &LINE_THICKNESS[rc->thickness];
    attron(COLOR_PAIR(PAIR_CONN) | s->attr);
    for (int i = 0; i < arch->n_layers - 1; i++) {
        for (int a = 0; a < arch->n_per_layer; a++) {
            int ar, ac;
            neuron_cell(i, a, arch->n_layers, arch->n_per_layer,
                        rows, cols, &ar, &ac);
            for (int b = 0; b < arch->n_per_layer; b++) {
                int br, bc;
                neuron_cell(i + 1, b, arch->n_layers, arch->n_per_layer,
                            rows, cols, &br, &bc);
                draw_line(ar, ac, br, bc, rows, cols, s);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_CONN) | s->attr);
}

/* draw one neuron as ( O ) centred at (sr, sc).  The parens make it read as a
 * round node even in fonts where a bare 'O' looks like a letter.  Each of the
 * three cells is checked on its own, so a neuron at the screen edge draws the
 * cells that fit and quietly skips the rest. */
static void paint_neuron_sigil(int sr, int sc, int cols)
{
    if (sc - NEURON_SIGIL_HALF >= 0    ) mvaddch(sr, sc - NEURON_SIGIL_HALF, NEURON_GLYPH_LEFT);
    if (sc                     <  cols ) mvaddch(sr, sc,                     NEURON_GLYPH_CENTRE);
    if (sc + NEURON_SIGIL_HALF <  cols ) mvaddch(sr, sc + NEURON_SIGIL_HALF, NEURON_GLYPH_RIGHT);
}

/* paint every neuron as a bright ( O ), keeping clear of the bottom hint row. */
static void draw_neurons(const NetArch *arch, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
    for (int i = 0; i < arch->n_layers; i++) {
        for (int j = 0; j < arch->n_per_layer; j++) {
            int sr, sc;
            neuron_cell(i, j, arch->n_layers, arch->n_per_layer,
                        rows, cols, &sr, &sc);
            if (sr < 0 || sr >= rows - 1) continue;
            paint_neuron_sigil(sr, sc, cols);
        }
    }
    attroff(COLOR_PAIR(PAIR_NEURON) | A_BOLD);
}

/* Screen — the terminal's size right now, the single source of truth everything
 * lays out against.  Since no neuron positions are stored, a resize just updates
 * these two numbers and the next frame redraws at the new size — nothing to
 * rebuild. */
typedef struct {
    int rows;   /* terminal height in character cells */
    int cols;   /* terminal width  in character cells */
} Screen;

/* FrameTimer — the loop's timing: how long a frame should take, the timestamps
 * it measures gaps from, and a smoothed fps for the HUD.  The dt cap matters: if
 * the program is suspended (Ctrl-Z, laptop sleep) the gap can jump by seconds,
 * so we cap it — the dots skip the lost time instead of lurching across the
 * whole network in one step. */
typedef struct {
    int64_t frame_ns;    /* how long one frame should last, ns (~33 ms at 30 fps) */
    int64_t t_tick_prev; /* time of the last move, to measure the gap since        */
    int64_t t_fps_prev;  /* time of the last fps update, likewise                  */
    double  fps;         /* smoothed frames per second, shown in the HUD           */
} FrameTimer;

/* Scene — one struct that owns everything long-lived, so the signal handlers and
 * the main loop share a single home for the state.  The pieces split by job: the
 * network's shape (changing it needs particle_reset), its looks, the human's
 * toggles, the flowing dots, the terminal size, the loop timing, and two flags
 * the signal handlers set. */
typedef struct {
    NetArch               arch;        /* the network's shape — changing it needs particle_reset */
    RenderConfig          render;      /* looks only (colours, wire weight)           */
    NetUI                 ui;          /* the human's toggles (pause)                 */
    ParticlePool          particles;   /* the dots flowing along the wires            */
    Screen                screen;      /* current terminal size                       */
    FrameTimer            timer;       /* loop pacing + fps                           */
    volatile sig_atomic_t running;     /* cleared by q/ESC or a quit signal           */
    volatile sig_atomic_t need_resize; /* set by a terminal-resize signal             */
} Scene;

/* §7.4 the two HUD strips: network settings + fps on the top row, the key list
 * on the bottom — both bold so they stay readable over the animation. */

/* build the top status line: the settings the user changes most first, then the
 * style, then fps and whether it's paused. */
static void hud_format_status(const Scene *scene, char *buf, size_t n)
{
    const NetArch      *arch = &scene->arch;
    const RenderConfig *rc   = &scene->render;

    snprintf(buf, n,
             " layers:%d  neurons:%d  total:%d  edges:%d  thick:%s  "
             "theme:%d  %5.1f fps  %s ",
             arch->n_layers, arch->n_per_layer,
             net_total_neurons(arch), net_total_edges(arch),
             LINE_THICKNESS[rc->thickness].name, rc->theme,
             scene->timer.fps,
             scene->ui.paused ? "PAUSED " : "running");
}

/* paint the status line, right-aligned on the top row */
static void hud_paint_status(const char *buf, int cols)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* the bottom strip, listing every key */
static void hud_paint_hint(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  r:reset  t:theme  "
             "[/]:layers  -/+:neurons  </>:thick ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* draw one frame, back to front: wipe, wires, dots, neurons on top, then the
 * HUD.  (Dots go under the neurons so a neuron covers a dot landing on it.) */
static void scene_draw(const Scene *scene)
{
    const NetArch      *arch = &scene->arch;
    const RenderConfig *rc   = &scene->render;
    int rows = scene->screen.rows, cols = scene->screen.cols;

    erase();
    draw_connections(arch, rc, rows, cols);
    particle_draw(&scene->particles, arch, rows, cols);
    draw_neurons(arch, rows, cols);

    char buf[160];
    hud_format_status(scene, buf, sizeof buf);
    hud_paint_status (buf, cols);
    hud_paint_hint   (rows);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8  screen — bring ncurses up and tear it down ──────────────────── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    /* turn on the user's locale so the UTF-8 box characters (the "heavy" wire
     * look) print correctly instead of as garbled bytes. */
    setlocale(LC_ALL, "");
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §9  app — signals, input, and the main loop ─────────────────────── */

/* The one Scene, at file scope so the signal handlers can reach its flags. */
static Scene g_scene;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_scene.running     = 0;
    if (s == SIGWINCH)               g_scene.need_resize = 1;
}

/* put everything back to startup defaults — the 'r' key.  One function so a
 * reset can't forget a piece: the shape, the looks, the toggles, the dot pool,
 * and the colours all go back together. */
static void scene_reset(Scene *s)
{
    arch_reset    (&s->arch);
    render_reset  (&s->render);
    ui_reset      (&s->ui);
    particle_reset(&s->particles, &s->arch);
    color_init    (s->render.theme);
}

/* ── §9.1 setup + per-step main-loop helpers ───────────────────────── */

/* One-shot scene setup before the loop: signals, defaults, ncurses,
 * particle pool, FrameTimer seed. */
static void scene_setup(void)
{
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    /* (1) seed defaults */
    g_scene.running = 1;
    arch_reset    (&g_scene.arch);
    render_reset  (&g_scene.render);
    ui_reset      (&g_scene.ui);
    particle_reset(&g_scene.particles, &g_scene.arch);

    /* (2) ncurses up; learn rows/cols */
    screen_init(g_scene.render.theme);
    g_scene.screen.rows = LINES;
    g_scene.screen.cols = COLS;

    /* (3) seed FrameTimer */
    g_scene.timer.frame_ns    = NS_PER_SEC / TARGET_FPS;
    g_scene.timer.fps         = TARGET_FPS;
    g_scene.timer.t_fps_prev  = clock_ns();
    g_scene.timer.t_tick_prev = g_scene.timer.t_fps_prev;
}

/* after a terminal resize: re-sync ncurses and read the new size.  The dots are
 * untouched — positions are worked out fresh from the new size next frame. */
static void scene_handle_resize(void)
{
    g_scene.need_resize = 0;
    endwin();
    refresh();
    g_scene.screen.rows = LINES;
    g_scene.screen.cols = COLS;
}

/* seconds since the last move, capped so a stalled program can't dump a huge
 * catch-up step onto the dots. */
static float frame_measure_dt(int64_t now)
{
    float dt = (float)(now - g_scene.timer.t_tick_prev) / (float)NS_PER_SEC;
    if (dt > DT_CAP_S) dt = DT_CAP_S;
    g_scene.timer.t_tick_prev = now;
    return dt;
}

/* fold this frame's rate into the smoothed fps (the +1 avoids a divide-by-zero
 * on a zero-length frame). */
static void frame_update_ewma_fps(int64_t now)
{
    int64_t dt_ns   = now - g_scene.timer.t_fps_prev + 1;
    double  instant = (double)NS_PER_SEC / (double)dt_ns;
    g_scene.timer.fps = g_scene.timer.fps * EWMA_RETAIN
                      + instant            * EWMA_NEW;
    g_scene.timer.t_fps_prev = now;
}

/* Sleep the remainder of frame_ns so we hit TARGET_FPS. */
static void frame_cap_to_target_fps(int64_t frame_start)
{
    clock_sleep_ns(g_scene.timer.frame_ns - (clock_ns() - frame_start));
}

/* ── §9.2 keyboard action handlers ─────────────────────────────────── */

/* 'p' — pause / resume particle ticks. Rendering keeps running. */
static void key_pause_toggle(void) { g_scene.ui.paused ^= 1; }

/* 't' — move to the next colour theme and re-bind the colours. */
static void key_cycle_theme(void)
{
    g_scene.render.theme = (g_scene.render.theme + 1) % N_THEMES;
    color_init(g_scene.render.theme);
}

/* '[' / ']' — change how many layers (kept within range).  A shape change
 * rebuilds the dots, since which wires exist depends on the layer count. */
static void key_change_layers(int delta)
{
    int n = g_scene.arch.n_layers + delta;
    if (n < MIN_LAYERS || n > MAX_LAYERS) return;
    g_scene.arch.n_layers = n;
    particle_reset(&g_scene.particles, &g_scene.arch);
}

/* '-' / '+' — change neurons per layer (kept within range).  Rebuilds the dots,
 * since there's one per input neuron. */
static void key_change_neurons(int delta)
{
    int n = g_scene.arch.n_per_layer + delta;
    if (n < MIN_NEURONS || n > MAX_NEURONS) return;
    g_scene.arch.n_per_layer = n;
    particle_reset(&g_scene.particles, &g_scene.arch);
}

/* '<' / '>' — change the wire weight (kept within range).  No rebuild — it only
 * changes how things look. */
static void key_change_thickness(int delta)
{
    int t = g_scene.render.thickness + delta;
    if (t < MIN_THICKNESS || t > MAX_THICKNESS) return;
    g_scene.render.thickness = t;
}

/* send one key to its action — the switch is the keymap. */
static void scene_handle_one_keystroke(int ch)
{
    switch (ch) {
    case 'q': case 27 /* ESC */:  g_scene.running = 0;           break;
    case 'p':                     key_pause_toggle();            break;
    case 'r':                     scene_reset(&g_scene);         break;
    case 't':                     key_cycle_theme();             break;
    case '[':                     key_change_layers(-1);         break;
    case ']':                     key_change_layers(+1);         break;
    case '-':                     key_change_neurons(-1);        break;
    case '+': case '=':           key_change_neurons(+1);        break;
    case ',': case '<':           key_change_thickness(-1);      break;
    case '.': case '>':           key_change_thickness(+1);      break;
    default:                                                     break;
    }
}

/* Drain all queued keystrokes through the action dispatcher. */
static void scene_drain_input(void)
{
    int ch;
    while ((ch = getch()) != ERR) scene_handle_one_keystroke(ch);
}

/* move the dots forward one time step (does nothing while paused). */
static void scene_advance_particles(float dt)
{
    if (g_scene.ui.paused) return;
    particle_tick(&g_scene.particles, &g_scene.arch, dt);
}

/* ── §9.3 main — set up once, then each frame: handle a resize, read keys, move
 * the dots, update the fps reading, draw, and sleep to hold the frame rate ─── */
int main(void)
{
    scene_setup();

    while (g_scene.running) {
        /* (1) deferred resize */
        if (g_scene.need_resize) scene_handle_resize();

        /* (2) drain queued keystrokes */
        scene_drain_input();

        /* (3) move the dots by the real time elapsed */
        int64_t now = clock_ns();
        float   dt  = frame_measure_dt(now);
        scene_advance_particles(dt);

        /* (4) update the smoothed fps for the HUD */
        frame_update_ewma_fps(now);

        /* (5) draw the frame */
        scene_draw(&g_scene);

        /* (6) sleep to target frame rate */
        frame_cap_to_target_fps(now);
    }
    return 0;
}
