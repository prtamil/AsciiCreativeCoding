/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cellular_automata_1d.c — Wolfram 1-D elementary cellular automata.
 *
 * Start with one row of cells, each on or off. A "rule" (a number 0-255)
 * decides each cell's next state from itself and its two neighbours. Stack
 * the rows down the screen and famous patterns appear: Sierpinski triangles
 * (rule 90), noise (rule 30), gliders (rule 110). Each rule is colour-coded
 * by its Wolfram behaviour class (fixed / periodic / chaotic / complex /
 * fractal). The action bar at the bottom lists every key.
 *
 * Background reading the code can't give you:
 *   Wolfram (1983) Rev. Mod. Phys. 55:601 — the 0-255 rule numbering.
 *   Wolfram (1984) Physica D 10:1         — the four behaviour classes.
 *   Cook   (2004) Complex Systems 15:1    — proof rule 110 is Turing-complete.
 *   Wolfram (2002) A New Kind of Science  — the big reference, free online.
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

/* ── §1  config — tunables, presets, colour-pair IDs ──────────────────── */

#define TICKS_PER_SEC 30                             /* sim + render tick rate */
#define TICK_NS      (1000000000LL / TICKS_PER_SEC)  /* one frame in nanosecs  */
#define MAX_ROWS     128         /* grid depth; bigger than any terminal     */
#define MAX_COLS     320         /* grid width; bigger than any terminal     */
#define DELAY_DEF    3           /* ticks between rows at startup (~10 rows/s)*/
#define DELAY_MIN    1           /* fastest: a new row every tick            */
#define DELAY_MAX    30          /* slowest: a new row every 30 ticks        */
#define PAUSE_TICKS  (3 * TICKS_PER_SEC)  /* how long a finished pattern holds */
#define LIVE_CHAR    '#'

/* The top row is the data bar and the bottom row is the action bar; the CA
 * gets everything in between. */
#define HUD_TOP_ROWS 1
#define HUD_BOT_ROWS 1
#define HUD_ROWS     (HUD_TOP_ROWS + HUD_BOT_ROWS)

#define COUNT_OF(a)  ((int)(sizeof (a) / sizeof (a)[0]))

/* Colour-pair IDs: one per Wolfram class (used on the data bar and grid),
 * plus one for the action bar. */
enum { CP_CL1 = 1, CP_CL2, CP_CL3, CP_CL4, CP_CL5, CP_HINT };

/*
 * Preset bank — a hand-picked tour of 17 of the 256 rules. Most rules look
 * dull, so this is just the famous, good-looking ones. The order is a
 * deliberate tour, not numeric; n/p step through it and auto-advance cycles
 * it. A "preset" is simply a rule plus a short caption.
 */
#define N_PRESETS 17
static const struct {
    int         rule;   /* the Wolfram rule number, 0..255 */
    const char *desc;   /* short caption shown on the data bar */
} PRESETS[N_PRESETS] = {
    {  30, "Chaos / RNG"       },
    {  90, "Sierpinski"        },
    { 110, "Turing-complete"   },
    {  18, "Sierpinski-like"   },
    { 150, "Pascal mod 2"      },
    {  60, "XOR fractal"       },
    {  54, "Complex"           },
    { 105, "Complex / fractal" },
    { 106, "Complex"           },
    {  45, "Chaotic"           },
    {  22, "Chaotic"           },
    { 126, "Chaotic"           },
    {  57, "Complex"           },
    {  73, "Complex"           },
    {  99, "Complex"           },
    {   0, "All zeros"         },
    { 255, "All ones"          },
};

/* ── §2  clock — read the time, sleep for a while ─────────────────────── */

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

/* ── §3  color — pick a palette, hand it to ncurses ───────────────────── */

/*
 * Theme — one palette: a colour for each of the five Wolfram classes plus a
 * colour for the action bar. The colour itself tells you the class, so every
 * theme keeps the five distinct — cycling t/T changes the mood, not the
 * meaning. All colours sit in the bright half of the palette so they stay
 * legible against a dark terminal.
 */
#define N_THEMES 5
typedef struct {
    const char *name;
    short cls[5];     /* one colour per class: fixed, periodic, chaotic, complex, fractal */
    short hint;       /* the action-bar colour */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "CLASSIC", { 244,  51, 202,  82, 226 },  51 },  /* grey/cyan/orange/green/yellow */
    { "NEON   ", { 201,  51, 226,  46, 208 }, 213 },  /* vivid magenta…orange          */
    { "EMBER  ", { 223, 215, 208, 202, 196 }, 220 },  /* warm light→red ramp           */
    { "ICE    ", { 195, 159, 123,  87,  51 }, 159 },  /* cool light→cyan ramp          */
    { "MONO   ", { 242, 246, 250, 253, 255 }, 248 },  /* greyscale tiers               */
};

static void theme_bind_256(const Theme *t)
{
    init_pair(CP_CL1, t->cls[0], -1);
    init_pair(CP_CL2, t->cls[1], -1);
    init_pair(CP_CL3, t->cls[2], -1);
    init_pair(CP_CL4, t->cls[3], -1);
    init_pair(CP_CL5, t->cls[4], -1);
    init_pair(CP_HINT, t->hint, -1);
}

/* Fallback for old 8-colour terminals: fixed hues, ignores the theme. */
static void theme_bind_8(void)
{
    init_pair(CP_CL1, COLOR_WHITE,  -1);
    init_pair(CP_CL2, COLOR_CYAN,   -1);
    init_pair(CP_CL3, COLOR_RED,    -1);
    init_pair(CP_CL4, COLOR_GREEN,  -1);
    init_pair(CP_CL5, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN,  -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_bind_256(&THEMES[idx]);
    else               theme_bind_8();
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(0);
}

/* ── §4  model — the data (Automaton + Control + Scene) and pure reads ── */

/*
 * WolframClass — the five buckets a rule's long-term behaviour falls into
 * (Wolfram 1984). We colour the pattern by its class so you can spot the
 * family at a glance.
 */
typedef enum {
    CLASS_FIXED    = 1,   /* settles to one solid state           */
    CLASS_PERIODIC = 2,   /* stable or repeating patterns         */
    CLASS_CHAOTIC  = 3,   /* looks random, very sensitive to seed */
    CLASS_COMPLEX  = 4,   /* little structures that move around    */
    CLASS_FRACTAL  = 5,   /* self-similar triangles (Sierpinski)  */
} WolframClass;

/*
 * SeedKind — how we fill the very first row, which is all that decides what
 * grows below it.
 */
typedef enum {
    SEED_CENTER,   /* one live cell dead centre (the classic seed) */
    SEED_RANDOM,   /* every cell a coin flip (chaotic rules look wildly different) */
} SeedKind;

/*
 * Automaton — the cellular automaton itself (Wolfram 1983). `cells` keeps the
 * ENTIRE run, not just the latest row: row g is generation g. That serves two
 * jobs at once — it's the picture on screen AND it means we never need a
 * second buffer, since each row is read from the one above and written once,
 * top to bottom (see ca_step). Sized for the biggest possible terminal.
 */
typedef struct {
    uint8_t cells[MAX_ROWS][MAX_COLS]; /* cells[gen][col] is 0 or 1; gen 0 is the seed */
    int     w;     /* live row width = terminal columns (at most MAX_COLS)  */
    int     gens;  /* rows that fit between the two bars (at most MAX_ROWS)  */
    int     gen;   /* the latest generation computed (0 right after seeding) */
    int          rule; /* the rule in play, 0..255 — the whole behaviour      */
    WolframClass cls;  /* its class, worked out once so colouring is instant  */
} Automaton;

/*
 * Phase — the run has two stages. BUILD keeps adding rows until the screen is
 * full; HOLD freezes the finished picture for a few seconds before moving on.
 * scene_tick flips between them.
 */
typedef enum { PHASE_BUILD, PHASE_HOLD } Phase;

/*
 * Control — everything the user can tweak, plus where we are in the build/hold
 * cycle. Kept separate from the Automaton so keypresses touch these knobs
 * while the simulation touches the grid.
 */
typedef struct {
    int   preset;       /* which entry of PRESETS is showing               */
    int   theme;        /* which entry of THEMES is active                 */
    int   delay;        /* ticks between rows — bigger is slower (MIN..MAX) */
    bool  paused;       /* stop stepping, but keep drawing                 */
    bool  auto_advance; /* when a pattern finishes: next preset, or redraw  */
    Phase phase;        /* BUILD = still filling; HOLD = done, waiting      */
    int   delay_ctr;    /* ticks since we last added a row                 */
    int   hold_ctr;     /* ticks elapsed since the pattern finished         */
} Control;

/*
 * Scene — one running show: the automaton plus the knobs that steer it. This
 * is the bundle the effects change (Scene*) and the renderer reads
 * (const Scene*); App adds the terminal and loop state around it.
 */
typedef struct {
    Automaton ca;    /* the simulation (§5 advances it) */
    Control   ctl;   /* the user knobs + build/hold stage */
} Scene;

/* ── pure reads / logic ──────────────────────────────────────────────── */

static bool rule_in_set(int r, const int *set, int n)
{
    for (int i = 0; i < n; i++) if (set[i] == r) return true;
    return false;
}

/* Look up a rule's behaviour class; rules not in any table count as periodic. */
static WolframClass ca_classify(int r)
{
    static const int cl5[] = { 18, 60, 90, 105, 150 };
    static const int cl4[] = { 54, 57, 62, 73, 99, 106, 110 };
    static const int cl3[] = { 22, 30, 45, 75, 89, 109, 126, 135, 149, 153, 154 };
    static const int cl1[] = { 0, 8, 32, 40, 128, 136, 160, 168, 255 };
    if (rule_in_set(r, cl5, COUNT_OF(cl5))) return CLASS_FRACTAL;
    if (rule_in_set(r, cl4, COUNT_OF(cl4))) return CLASS_COMPLEX;
    if (rule_in_set(r, cl3, COUNT_OF(cl3))) return CLASS_CHAOTIC;
    if (rule_in_set(r, cl1, COUNT_OF(cl1))) return CLASS_FIXED;
    return CLASS_PERIODIC;
}

static const char *class_name(WolframClass cls)
{
    switch (cls) {
        case CLASS_FIXED:    return "Fixed";
        case CLASS_PERIODIC: return "Periodic";
        case CLASS_CHAOTIC:  return "Chaotic";
        case CLASS_COMPLEX:  return "Complex";
        case CLASS_FRACTAL:  return "Fractal";
    }
    return "Unknown";
}

/* The colour pair that stands for a class on the data bar and the grid. */
static int class_cp(WolframClass cls)
{
    switch (cls) {
        case CLASS_FIXED:    return CP_CL1;
        case CLASS_PERIODIC: return CP_CL2;
        case CLASS_CHAOTIC:  return CP_CL3;
        case CLASS_COMPLEX:  return CP_CL4;
        case CLASS_FRACTAL:  return CP_CL5;
    }
    return CP_CL2;
}

/* Turn the three cells above (left, middle, right) into a number 0..7. */
static int neighbourhood_code(uint8_t l, uint8_t m, uint8_t r)
{
    return (l << 2) | (m << 1) | r;
}

/* The rule is 8 bits; that 0..7 number picks which bit, and that bit is the
 * cell's new value. This single step is the whole rule. */
static uint8_t rule_bit(int rule, int code)
{
    return (uint8_t)((rule >> code) & 1);
}

/* One cell's next value: look up the rule by the three cells above it. */
static uint8_t cell_next(uint8_t l, uint8_t m, uint8_t r, int rule)
{
    return rule_bit(rule, neighbourhood_code(l, m, r));
}

/* Wrap a column back onto the row, so the left edge meets the right (a loop,
 * no special handling at the ends). */
static int wrap_col(int c, int w) { return (c % w + w) % w; }

/* ── §5  simulation — the steps that change the automaton + knobs ─────── */

static void ca_clear(Automaton *a)
{
    memset(a->cells, 0, sizeof a->cells);
    a->gen = 0;
}

/* First row: one live cell in the middle. */
static void ca_seed_center(Automaton *a)
{
    ca_clear(a);
    if (a->w > 0) a->cells[0][a->w / 2] = 1;
}

/* First row: random on/off (chaotic rules look very different from this). */
static void ca_seed_random(Automaton *a)
{
    ca_clear(a);
    for (int c = 0; c < a->w; c++) a->cells[0][c] = (uint8_t)(rand() & 1);
}

/* Grow one new row from the row above; the edges wrap around. */
static void ca_step(Automaton *a)
{
    if (a->gen >= a->gens - 1) return;             /* screen is full */
    const uint8_t *src = a->cells[a->gen];
    uint8_t       *dst = a->cells[a->gen + 1];
    int w = a->w;
    for (int c = 0; c < w; c++) {
        uint8_t l = src[wrap_col(c - 1, w)];
        uint8_t m = src[c];
        uint8_t r = src[wrap_col(c + 1, w)];
        dst[c] = cell_next(l, m, r, a->rule);
    }
    a->gen++;
}

/* Lay down a fresh first row and start building from the top again. */
static void scene_reseed(Scene *s, SeedKind kind)
{
    if (kind == SEED_RANDOM) ca_seed_random(&s->ca);
    else                     ca_seed_center(&s->ca);
    s->ctl.phase     = PHASE_BUILD;
    s->ctl.delay_ctr = 0;
    s->ctl.hold_ctr  = 0;
}

/* Switch to a preset: take its rule, work out its class, start fresh. */
static void scene_set_preset(Scene *s, int idx)
{
    s->ctl.preset = ((idx % N_PRESETS) + N_PRESETS) % N_PRESETS;
    s->ca.rule    = PRESETS[s->ctl.preset].rule;
    s->ca.cls     = ca_classify(s->ca.rule);
    scene_reseed(s, SEED_CENTER);
}

/* Switch colour theme; the pattern itself is left alone. */
static void scene_set_theme(Scene *s, int idx)
{
    s->ctl.theme = ((idx % N_THEMES) + N_THEMES) % N_THEMES;
    theme_apply(s->ctl.theme);
}

/* One tick: either add a row (build) or wait out the hold timer. */
static void scene_tick(Scene *s)
{
    Control *c = &s->ctl;
    if (c->paused) return;

    if (c->phase == PHASE_HOLD) {
        if (++c->hold_ctr >= PAUSE_TICKS) {
            /* Auto on: move to the next preset. Off: redraw this same one. */
            if (c->auto_advance) scene_set_preset(s, c->preset + 1);
            else                 scene_reseed(s, SEED_CENTER);
        }
        return;
    }

    /* Building: add a row once every `delay` ticks. */
    if (++c->delay_ctr >= c->delay) {
        c->delay_ctr = 0;
        ca_step(&s->ca);
        if (s->ca.gen >= s->ca.gens - 1) {
            c->phase    = PHASE_HOLD;
            c->hold_ctr = 0;
        }
    }
}

static void scene_init(Scene *s)
{
    s->ctl.preset       = 0;
    s->ctl.theme        = 0;
    s->ctl.delay        = DELAY_DEF;
    s->ctl.paused       = false;
    s->ctl.auto_advance = true;
    scene_set_preset(s, 0);      /* picks rule + class, seeds, resets the stage */
}

/* Fewer ticks per row means a new row sooner, so it builds faster. */
static void ctl_speed_up  (Control *c) { if (c->delay > DELAY_MIN) c->delay--; }
static void ctl_speed_down(Control *c) { if (c->delay < DELAY_MAX) c->delay++; }

/* ── §6  render — read the scene, paint the screen, change nothing ────── */

/*
 * Draw a full-width coloured bar across one row, clipping the text so it can
 * never spill into the CA area. We skip the very last column on purpose:
 * writing the bottom-right corner makes some terminals scroll.
 */
static void hud_bar(int row, int cols, chtype attr, const char *buf)
{
    if (row < 0 || cols < 2) return;
    int w = cols - 1;
    attron(attr);
    for (int x = 0; x < w; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, w);
    attroff(attr);
}

/* The little status note at the end of the data bar: "paused", or the
 * countdown to the next preset, or nothing while it's still building. */
static void phase_status(char *buf, size_t n, const Control *c)
{
    if (c->paused) {
        snprintf(buf, n, "  [PAUSED]");
    } else if (c->phase == PHASE_HOLD) {
        int secs = (PAUSE_TICKS - c->hold_ctr + TICKS_PER_SEC - 1) / TICKS_PER_SEC;
        snprintf(buf, n, "  [%s in %ds]", c->auto_advance ? "next" : "redraw", secs);
    } else {
        buf[0] = '\0';
    }
}

/* Top data bar (row 0), coloured by the rule's class. */
static void render_data_bar(const Scene *s, int cols)
{
    const Automaton *a = &s->ca;
    const Control   *c = &s->ctl;

    char status[48];
    phase_status(status, sizeof status, c);

    char buf[220];
    snprintf(buf, sizeof buf,
             " 1D_CA  rule:%-3d %-18s class:%-8s  preset:%2d/%d  theme:%s  "
             "gen:%d/%d  spd:%d  auto:%-3s%s ",
             a->rule, PRESETS[c->preset].desc, class_name(a->cls),
             c->preset + 1, N_PRESETS, THEMES[c->theme].name,
             a->gen, a->gens, c->delay, c->auto_advance ? "on" : "off", status);

    hud_bar(0, cols, COLOR_PAIR(class_cp(a->cls)) | A_BOLD | A_REVERSE, buf);
}

/* The pattern: generation g goes on screen row 1+g, in the class colour. */
static void render_grid(const Scene *s, int cols)
{
    const Automaton *a = &s->ca;
    chtype attr = COLOR_PAIR(class_cp(a->cls));
    int w = a->w < cols ? a->w : cols;

    attron(attr);
    for (int g = 0; g <= a->gen && g < a->gens; g++) {
        const uint8_t *row = a->cells[g];
        for (int c = 0; c < w - 1; c++)
            mvaddch(HUD_TOP_ROWS + g, c, row[c] ? (chtype)(unsigned char)LIVE_CHAR : ' ');
    }
    attroff(attr);
}

/* Bottom action bar (last row): the list of keys you can press. */
static void render_action_bar(int cols, int rows)
{
    static const char *keys =
        " n/p:preset  t/T:theme  a:auto  r:seed  R:rand  +/-:speed  spc:pause  q:quit ";
    hud_bar(rows - 1, cols, COLOR_PAIR(CP_HINT) | A_BOLD, keys);
}

/* Paint one whole frame: clear, then the two bars and the grid, then flush. */
static void render_frame(const Scene *s, int cols, int rows)
{
    erase();
    render_data_bar(s, cols);
    render_grid(s, cols);
    render_action_bar(cols, rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7  app — tie it together: input, then tick, then draw, then wait ── */

/*
 * App — everything around the Scene that's about the program rather than the
 * automaton: how big the terminal is and the flags the main loop watches.
 * There's one global copy (g_app) so the signal handler can reach the flags;
 * the handler can only touch sig_atomic_t values safely.
 */
typedef struct {
    Scene scene;               /* the running automaton (§4-§6)              */
    int   rows, cols;          /* terminal size, in character cells          */
    volatile sig_atomic_t running;     /* 0 means quit; set by Ctrl-C / kill  */
    volatile sig_atomic_t need_resize; /* set when the window changed size    */
} App;

static App g_app;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

/* Re-measure the terminal and size the grid to it: width = columns,
 * number of rows = whatever sits between the two bars. */
static void app_fit(App *app)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    app->rows = rows < MAX_ROWS ? rows : MAX_ROWS;
    app->cols = cols < MAX_COLS ? cols : MAX_COLS;
    app->scene.ca.w    = app->cols;
    app->scene.ca.gens = (app->rows - HUD_ROWS < 1) ? 1 : app->rows - HUD_ROWS;
}

/* Turn a keypress into an action. */
static void app_handle_key(App *app, int ch)
{
    Scene   *s = &app->scene;
    Control *c = &s->ctl;
    switch (ch) {
    case 'q': case 'Q': app->running = 0;                    break;
    case ' ':           c->paused = !c->paused;              break;
    case 'a': case 'A': c->auto_advance = !c->auto_advance;  break;
    case 'n':           scene_set_preset(s, c->preset + 1);  break;
    case 'p':           scene_set_preset(s, c->preset - 1);  break;
    case 't':           scene_set_theme (s, c->theme + 1);   break;
    case 'T':           scene_set_theme (s, c->theme - 1);   break;
    case 'r':           scene_reseed(s, SEED_CENTER);              break;
    case 'R':           scene_reseed(s, SEED_RANDOM);               break;
    case '+': case '=': ctl_speed_up(c);                     break;
    case '-': case '_': ctl_speed_down(c);                   break;
    default: break;
    }
}

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));

    App *app     = &g_app;
    app->running = 1;

    screen_init();
    app_fit(app);                /* size the grid before we seed it */
    scene_init(&app->scene);     /* defaults, first preset, centre seed */

    long long next = clock_ns();

    /*
     * Main loop, one steady beat per frame:
     *   read any keys, advance the simulation by a tick, draw, then sleep
     *   until the next ~30th of a second. If the window was resized, re-fit
     *   the grid and start the pattern over.
     */
    while (app->running) {
        if (app->need_resize) {
            app->need_resize = 0;
            endwin();
            refresh();
            app_fit(app);
            scene_reseed(&app->scene, SEED_CENTER);
        }

        int ch;                                              /* INPUT   */
        while ((ch = getch()) != ERR) app_handle_key(app, ch);

        scene_tick(&app->scene);                             /* EFFECTS */
        render_frame(&app->scene, app->cols, app->rows);     /* RENDER  */

        next += TICK_NS;                                     /* DELAY   */
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
