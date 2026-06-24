/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * langton.c — Langton's Ant and multi-colour "turmites"
 *
 * An ant walks a wrap-around grid. A short rule string says what to do on
 * each cell colour ('R' = turn right, 'L' = turn left); the ant turns,
 * recolours the cell, and steps forward. From these trivial local rules
 * surprisingly rich pictures emerge — Langton's "RL" builds a repeating
 * diagonal "highway" out of pure chaos after about 10 000 steps.
 *
 * References (the idea, the proofs — things the code can't tell you):
 *   Langton, Physica D 22 (1986) — the original ant.
 *   Bunimovich & Troubetzkoy, J. Stat. Phys. 67 (1992) — the ant's path
 *     never stays confined; it always wanders off to infinity.
 *   Wolfram, "A New Kind of Science" (2002) — the multi-colour turmites.
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 CONFIG ─ constants, CP ids, dir vectors, presets, palettes ───── */

#define TICK_NS      33333333LL
#define MAX_ROWS     128
#define MAX_COLS     320
#define STEPS_DEF    12      /* how many ant-steps to run per frame, by default */
#define STEPS_MAX    2000
#define MAX_ANTS     3
#define MAX_COLORS   8       /* a rule can use at most this many cell colours   */
#define N_DIRS       4       /* the four headings: N, E, S, W                   */
#define NS_PER_SEC   1000000000LL

/* ncurses colour-pair slots. CP_C0..C7 colour the trail — a cell painted with
 * colour s shows up in pair CP_C0 + s, so the grid reads as bands of the ant's
 * history. CP_ANT is the bold '@' head; CP_HUD / CP_HINT are the two HUD rows. */
enum { CP_C0=1, CP_C1, CP_C2, CP_C3, CP_C4, CP_C5, CP_C6, CP_C7,
       CP_ANT, CP_HUD, CP_HINT };

/* How a step in each heading changes (row, col). 0=N 1=E 2=S 3=W. Two parallel
 * lookup tables, so taking a step is just `r += DR[dir]` with no if/else. */
static const int DR[4] = {-1,  0, 1,  0};
static const int DC[4] = { 0,  1, 0, -1};

/* The menu of rules you cycle with n/p. The whole point of Langton's ant is that
 * the same trivial rule gives wildly different pictures — highway, fractal,
 * spiral, chaos — just by changing this little turn string. */
#define N_PRESETS 8
static const struct {
    const char *rule;    /* one 'R'/'L' per cell colour */
    const char *name;    /* label shown in the HUD      */
} PRESETS[N_PRESETS] = {
    { "RL",    "Langton RL (highway)"  },
    { "LR",    "LR (fractal)"          },
    { "LLRR",  "LLRR (square spiral)"  },
    { "RLR",   "RLR (chaotic)"         },
    { "LRRL",  "LRRL (complex)"        },
    { "RRLL",  "RRLL (symmetric)"      },
    { "RLLR",  "RLLR (tiling)"         },
    { "LLRRR", "LLRRR (irregular)"     },
};

/* Cell colour -> screen colour. These are just distinct hues, not a light-to-dark
 * ramp: a cell's colour tells you which rule-step last painted it, not "how much"
 * of anything, so neighbours should look as different as possible. Slot 0 is the
 * empty background (-1 = let the terminal's own background show, drawn as a space).
 * PAL256 for 256-colour terminals, PAL8 the fallback — color_init picks one. */
static const short PAL256[MAX_COLORS] =
    { -1, 202, 226, 82, 51, 45, 201, 196 };  /* bg, orange, yellow, green, cyan, blue, magenta, red */
static const short PAL8[MAX_COLORS] =
    { -1, COLOR_RED, COLOR_YELLOW, COLOR_GREEN,
      COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA, COLOR_WHITE };

/* ── §2 STATE ─ the domain types and the one Scene that aggregates them ─ */

/* One ant: where it is (r, c) and which way it's facing (0=N 1=E 2=S 3=W, which
 * also indexes DR/DC). That's all — the ant remembers nothing; its whole memory
 * is the trail of colours it leaves on the grid. */
typedef struct { int r, c, dir; } Ant;

/* The grid the ants crawl over and paint.
 *   cells   each cell's current colour, 0..n_states-1. 0 is empty. One byte is
 *           plenty since a rule uses at most 8 colours. This IS the picture and
 *           the ants' shared memory.
 *   rows/cols  the part of the grid actually in use — sized to fit the terminal,
 *           never bigger than the MAX_* arrays. Ant moves wrap around the edges.
 *   steps   total ant-steps since the last reset — the run's clock, shown in the
 *           HUD. Lives here because it counts moves over this grid. */
typedef struct {
    uint8_t   cells[MAX_ROWS][MAX_COLS];
    int       rows, cols;
    long long steps;
} Board;

/* The rule the ant follows: for a cell of colour s, turns[s] is the turn ('R' or
 * 'L') to make; the cell then advances to colour (s+1) % n_states. "RL" is
 * Langton's original. n_states is just the length of the string, clamped to
 * [2, MAX_COLORS] — a 1-colour rule just spins the ant in place, and more than 8
 * colours would run past the palette. */
typedef struct {
    const char *turns;
    int         n_states;
} Rule;

/* Everything the program owns, gathered in one place. The big per-frame steps
 * (tick / reset / resize) take the whole Scene; the small helpers take only the
 * one piece they touch (an Ant, the Board, the Rule), which keeps the simulation,
 * the drawing, and the reset code from tangling together. */
typedef struct {
    Board board;           /* the grid the ants paint                       */
    Ant   ants[MAX_ANTS];  /* the ants; only 0..n_ants-1 are active         */
    int   n_ants;          /* how many ants share the grid (1..MAX_ANTS)    */
    Rule  rule;            /* the rule every ant currently follows          */
    int   preset;          /* which PRESETS entry is selected (n/p menu)    */
    int   speed;           /* ant-steps to run per frame (1..STEPS_MAX)     */
    int   paused;          /* nonzero = frozen                              */
} Scene;

/* The one and only instance. The grid lives inside it, so this is a single ~40 KB
 * static object — no heap, nothing allocated after startup. */
static Scene g_scene;

/* Set by the signal handlers, so they have to be these special flag types rather
 * than normal globals. Kept off the Scene since signals can fire any time. */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

/* ── §3 PERFORMANCE ─ monotonic clock; the frame cap lives in §9 ──────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ── §4 COLOR ─ palette setup (the only colour-pair I/O) ──────────────── */

static void color_init(const Rule *rule)
{
    start_color();
    use_default_colors();
    init_pair(CP_ANT,  (COLORS >= 256) ? 231 : COLOR_WHITE,  -1);
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* top HUD row    */
    init_pair(CP_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* bottom key row */
    /* colour 0 stays the blank background; give every other colour a pair */
    for (int i = 1; i < rule->n_states && i < MAX_COLORS; i++) {
        short fg = (COLORS >= 256) ? PAL256[i] : PAL8[i];
        init_pair(CP_C0 + i, fg, -1);
    }
}

/* ── §5 SIMULATION ─ the per-tick advance: ant_step, sim_tick ─────────── */

/* Plain modulo that's never negative: folds v into 0..n-1. Used both to wrap an
 * ant around the grid edges and to cycle the preset menu past either end. */
static int wrap(int v, int n) { return ((v % n) + n) % n; }

/* Turn a heading 90 degrees — right is +1, left is -1 (as +3) around the four
 * directions. */
static int turn(int dir, char lr)
{
    return (dir + (lr == 'R' ? 1 : N_DIRS - 1)) % N_DIRS;
}

static void ant_step(Ant *ant, Board *board, const Rule *rule)
{
    /* One Langton move: read the colour under the ant, turn the way the rule says,
     * bump the cell to its next colour, then walk forward (wrapping at the edges). */
    int  state = board->cells[ant->r][ant->c];
    char lr    = rule->turns[state % rule->n_states];

    ant->dir = turn(ant->dir, lr);
    board->cells[ant->r][ant->c] = (uint8_t)((state + 1) % rule->n_states);
    ant->r = wrap(ant->r + DR[ant->dir], board->rows);
    ant->c = wrap(ant->c + DC[ant->dir], board->cols);
}

static void sim_tick(Scene *s)
{
    if (s->paused) return;
    for (int step = 0; step < s->speed; step++) {
        for (int i = 0; i < s->n_ants; i++)
            ant_step(&s->ants[i], &s->board, &s->rule);
    }
    s->board.steps += s->speed;
}

/* ── §6 RENDER ─ state → screen; reads only, never mutates ────────────── */

/* Draw the trail the ants have left. Skips the top and bottom rows since those
 * belong to the HUD; empty cells (colour 0) are drawn as blank space. */
static void draw_trails(const Scene *s)
{
    const Board *board = &s->board;
    for (int r = 1; r < board->rows - 1; r++) {
        const uint8_t *row = board->cells[r];
        for (int c = 0; c < board->cols - 1; c++) {
            int st = row[c];
            if (st == 0) { mvaddch(r, c, ' '); continue; }
            int cp = CP_C0 + (st % MAX_COLORS);
            attron(COLOR_PAIR(cp));
            mvaddch(r, c, (chtype)(unsigned char)'#');
            attroff(COLOR_PAIR(cp));
        }
    }
}

/* Draw the ants themselves as bold '@', on top of the trail. */
static void draw_ants(const Scene *s)
{
    const Board *board = &s->board;
    attron(COLOR_PAIR(CP_ANT) | A_BOLD);
    for (int i = 0; i < s->n_ants; i++) {
        int r = s->ants[i].r, c = s->ants[i].c;
        if (r >= 1 && r < board->rows - 1 && c < board->cols - 1)
            mvaddch(r, c, '@');
    }
    attroff(COLOR_PAIR(CP_ANT) | A_BOLD);
}

static void scene_draw(const Scene *s)
{
    draw_trails(s);
    draw_ants(s);
}

static void scene_hud(const Scene *s)
{
    char buf[256];

    /* top row: rule name, ant count, step count, speed, paused/running */
    snprintf(buf, sizeof buf,
             " %s   ants:%d   steps:%lld   spd:%d   %s",
             PRESETS[s->preset].name, s->n_ants, s->board.steps, s->speed,
             s->paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, s->board.cols - 1);   /* cut off at the edge, never wraps */
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row: the keys you can press */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvaddnstr(s->board.rows - 1, 0,
              " q:quit  spc:pause  n/p:rule  1-3:ants  r:reset  +/-:speed",
              s->board.cols - 1);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §7 SEED/RESET ─ user-triggered re-init (NOT the tick) ────────────── */

/* Put the ants down at fixed spots (the centre plus two offsets) so several ants
 * don't all start stacked on the same square. */
static void spread_ants(Scene *s)
{
    const Board *board = &s->board;
    static const float ROFF[3] = {0.5f, 0.35f, 0.65f};   /* start row, as a fraction of the grid */
    static const float COFF[3] = {0.5f, 0.35f, 0.65f};   /* start col, as a fraction of the grid */
    static const int   DIRS[3] = {0, 1, 3};               /* starting heading for each ant       */
    for (int i = 0; i < s->n_ants; i++) {
        s->ants[i].r   = (int)(ROFF[i] * board->rows) % board->rows;
        s->ants[i].c   = (int)(COFF[i] * board->cols) % board->cols;
        s->ants[i].dir = DIRS[i % 3];
    }
}

static void sim_reset(Scene *s)
{
    Board *board = &s->board;
    if (board->rows < 1 || board->cols < 1) return;
    memset(board->cells, 0, sizeof(board->cells));   /* wipe the grid back to empty */
    board->steps = 0;
    spread_ants(s);
}

static void set_preset(Scene *s, int idx)
{
    s->preset = wrap(idx, N_PRESETS);   /* wraps around past either end of the menu */
    s->rule.turns    = PRESETS[s->preset].rule;
    s->rule.n_states = (int)strlen(s->rule.turns);
    if (s->rule.n_states < 2) s->rule.n_states = 2;                  /* 1-colour rules just spin */
    if (s->rule.n_states > MAX_COLORS) s->rule.n_states = MAX_COLORS; /* don't run past the palette */
    color_init(&s->rule);
    sim_reset(s);
}

/* ── §8 PLATFORM ─ terminal setup/resize + signal handlers ────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
}

static void screen_resize(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    s->board.rows = rows;
    s->board.cols = cols;
    sim_reset(s);
    erase();
}

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}

static void install_signals(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
}

static void cleanup(void) { endwin(); }

/* ── §9 DRIVER ─ setup, input, and the per-tick combine point ─────────── */

/* Act on one keypress: quit, pause, change rule / ant count, reset, or speed. */
static void handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': g_running = 0; break;
    case ' ':           s->paused ^= 1; break;
    case 'n':           set_preset(s, s->preset + 1); break;
    case 'p':           set_preset(s, s->preset - 1); break;
    case 'r':           sim_reset(s);                 break;
    case '1': s->n_ants = 1; sim_reset(s); break;
    case '2': s->n_ants = 2; sim_reset(s); break;
    case '3': s->n_ants = 3; sim_reset(s); break;
    case '+': case '=':
        s->speed = (s->speed < STEPS_MAX) ? s->speed * 2 : STEPS_MAX;   /* go faster */
        break;
    case '-': case '_':
        s->speed = (s->speed > 1) ? s->speed / 2 : 1;                   /* go slower */
        break;
    }
}

int main(void)
{
    Scene *s = &g_scene;

    install_signals();
    atexit(cleanup);
    screen_init();
    s->speed  = STEPS_DEF;
    s->paused = 0;
    s->n_ants = 1;
    screen_resize(s);
    set_preset(s, 0);

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {              /* terminal was resized: re-fit and restart */
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize(s);
        }

        int ch;
        while ((ch = getch()) != ERR)
            handle_key(s, ch);

        sim_tick(s);
        erase();
        scene_draw(s);
        scene_hud(s);
        wnoutrefresh(stdscr);
        doupdate();

        /* wait out the rest of the frame so we hold a steady 30 fps */
        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
