/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * life.c — Conway's Game of Life, plus five sibling rules.
 *
 * A grid of cells that each turn live or die based on how many live
 * neighbours they have.  The grid wraps around at the edges (a glider
 * leaving the right side comes back on the left).  Each rule is just a
 * pair of bit-flags saying which neighbour counts cause birth/survival,
 * so the same loop runs all six variants.
 *
 * Patterns and the maths behind them: LifeWiki (conwaylife.com/wiki) and
 * Gardner's original 1970 Scientific American column.
 *
 * Sections: §1 config+types  §2 performance  §3 logic
 *           §4 simulation  §5 render  §6 app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config + types   (the ONLY place data is declared) ────────────── */

#define TICK_NS     33333333LL
#define MAX_ROWS    128
#define MAX_COLS    320
#define HIST_LEN    512    /* how many past population counts we remember  */
#define HIST_ROWS   3      /* screen rows the population chart occupies     */
#define HUD_TOP     1      /* row 0 holds the top status line               */
#define HUD_BOT     1      /* bottom row holds the key hints                */
#define STEPS_DEF   3
#define STEPS_MAX   30
#define LIVE_CHAR   '#'
#define RAMP_LEN    9      /* one colour per neighbour count, 0..8          */

/* ncurses needs a numbered "pair" for every text colour we use, so we hand
 * out IDs here.  The first nine (CP_RAMP..CP_RAMP+8) are the live-cell
 * colours, one per neighbour count.  IDs start at 1 because ncurses keeps 0
 * for the terminal's own default colours. */
enum {
    CP_RAMP = 1,                   /* the nine live-cell colours start here */
    CP_HIST = CP_RAMP + RAMP_LEN,  /* then the chart and the two HUD rows   */
    CP_HUD,
    CP_HINT
};

/* Rule — one set of Life rules, written in the standard B/S shorthand.
 *
 * A cell's fate depends only on how many of its 8 neighbours are alive
 * (0..8).  So each rule is two little sets of numbers: which counts give
 * BIRTH to an empty cell, and which counts let a LIVE cell SURVIVE.  We
 * pack each set into the bits of one number, which lets the main loop test
 * a count with a single bit-check and run every variant unchanged.
 * Example — Conway's B3/S23: born on exactly 3 neighbours, survives on 2 or 3. */
typedef struct {
    uint16_t birth;     /* bit n on => an empty cell with n neighbours comes alive */
    uint16_t survive;   /* bit n on => a live cell with n neighbours stays alive   */
    const char *name;   /* label shown in the HUD, e.g. "Conway B3/S23"            */
} Rule;
#define N_RULES 6
static Rule RULES[N_RULES];   /* the six rules, filled in by rules_init (§4) */

/* Theme — the colours used to draw live cells, cycled with t/T.
 *
 * A cell is just alive or dead, so there's nothing to shade by — except its
 * own neighbour count.  We colour each live cell by how crowded it is, which
 * makes lone cells, gliders, and dense blobs read as different colours.
 *
 * Every colour here is from the bright half of the palette (project rule) so
 * even the lowest tier stays visible on a black background. */
typedef struct {
    const char *name;           /* label shown in the HUD, e.g. "OCEAN"            */
    short       ramp[RAMP_LEN];  /* ramp[n] = colour for a cell with n neighbours,
                                  * running dim (lonely) to bright (crowded)        */
} Theme;

static const Theme THEMES[] = {
    { "OCEAN",  {  25,  26,  27,  32,  38,  44,  45,  51, 159 } },
    { "FIRE",   {  88, 124, 160, 166, 196, 202, 208, 214, 226 } },
    { "MATRIX", {  28,  34,  40,  46,  82, 118, 154, 191, 194 } },
    { "AMBER",  {  94, 130, 136, 166, 172, 178, 214, 220, 229 } },
    { "MONO",   { 244, 246, 247, 249, 250, 251, 252, 253, 255 } },
};
#define N_THEMES 5

/* Board — the cell grid itself.  It keeps two copies of the board and wraps
 * around at the edges.
 *
 * Why two copies: every cell of a new generation must look at its neighbours'
 * OLD values, all at the same instant.  So a step reads from one copy and
 * writes the result into the other, then swaps.  Editing one board in place
 * would let cells we already changed mislead the cells we haven't — the classic
 * Life bug where gliders smear into mush.
 *
 * Why wrap around: neighbour lookups run modulo the height/width (see
 * board_neighbors), so the top edge touches the bottom and the left edge
 * touches the right.  Patterns drift off one side and reappear on the other,
 * with no walls to disturb them. */
typedef struct {
    /* the two boards; one is "now", the other is scratch.  0 = dead, 1 = alive */
    uint8_t cells[2][MAX_ROWS][MAX_COLS];
    /* which of the two boards is the current generation (0 or 1); flips each step */
    int     buf;
    /* turns elapsed since the last reset; shown in the HUD as gen= */
    long    gen;
    /* the part of the grid we actually use, trimmed to fit the terminal on each
     * resize so a small window doesn't waste time on off-screen cells */
    int     h, w;
} Board;

/* PopulationHistory — a rolling record of how many cells were alive each step,
 * used only to draw the little chart at the bottom.  The simulation writes to
 * it but never reads it back, so clearing or resizing it can't change how the
 * world evolves.
 *
 * It's a ring buffer: a fixed array we keep writing into, looping back to the
 * start when full.  `head` is the next slot to write, so the newest sample is
 * just before head and the oldest is at head.  No shuffling — the oldest entry
 * is simply overwritten. */
typedef struct {
    long samples[HIST_LEN];  /* live-cell count saved after each step */
    int  head;               /* next slot to write; wraps back to 0 at the end */
} PopulationHistory;

/* Scene — everything the program needs, in one bundle, handed to the §6
 * top-level routines (set up / reset / advance).  Reads like a contents page:
 * the board is the world, history is the chart's data, rule_idx/steps/paused
 * say how it runs, theme picks the colours, and rows is the terminal height.
 * There's no separate width because the board always spans the full terminal,
 * so board.w is the width.  The single-int knobs each sit here loose — too
 * small to deserve their own struct. */
typedef struct {
    Board             board;     /* the cell grid being evolved             */
    PopulationHistory history;   /* the data behind the population chart     */
    int rule_idx;                /* which of the six rules is active, 0..N-1 */
    int steps;                   /* generations to run per frame (fast-forward) */
    int paused;                  /* 1 = frozen; scene_tick does nothing      */
    int theme;                   /* which colour palette is active, 0..N-1   */
    int rows;                    /* terminal height; bottom HUD sits at rows-1 */
} Scene;

static Scene g_scene = { .steps = STEPS_DEF };

/* Set by the signal handler, read by the main loop — not part of the Scene. */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

/* ── §2 performance   (frame clock + fixed-timestep cap; the only DELAY) ─ */

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

/* ── §3 logic   (pure decisions: read-only, no I/O — cannot be corrupted) ─ */

/* Pull an index back onto the grid so the edges connect.  C's % can return a
 * negative for an index just past the left/top edge, so we add n first; one
 * add is enough because no caller ever goes more than one step off-grid. */
static int wrap(int i, int n) { return (i + n) % n; }

/* Turn a list of neighbour counts into the packed bit-set a Rule stores. */
static uint16_t rule_mask(const int *ns, int len)
{
    uint16_t m = 0;
    for (int i = 0; i < len; i++) m |= (uint16_t)(1u << ns[i]);
    return m;
}

/* Count how many of cell (r,c)'s 8 neighbours are alive (0..8), wrapping at
 * the edges.  The one place neighbours are counted; both the rule and the
 * colour-by-crowding renderer call it. */
static int board_neighbors(const Board *b, int r, int c)
{
    int rn = wrap(r + 1, b->h), rp = wrap(r - 1, b->h);
    int cn = wrap(c + 1, b->w), cp = wrap(c - 1, b->w);
    return b->cells[b->buf][rp][cp] + b->cells[b->buf][rp][c] +
           b->cells[b->buf][rp][cn] + b->cells[b->buf][r ][cp] +
           b->cells[b->buf][r ][cn] + b->cells[b->buf][rn][cp] +
           b->cells[b->buf][rn][c ] + b->cells[b->buf][rn][cn];
}

/* The whole rule, for one cell: given it's alive or dead now and has `n`
 * neighbours, is it alive next turn?  A live cell checks the survive set, a
 * dead cell checks the birth set.  Same code for every variant. */
static int cell_next_state(const Rule *ru, int alive, int n)
{
    uint16_t bit = (uint16_t)(1u << n);
    return alive ? ((ru->survive & bit) ? 1 : 0)
                 : ((ru->birth   & bit) ? 1 : 0);
}

/* Turn a population count into a bar height (0..HIST_ROWS) for the chart.
 * The ×2 makes a half-full board already fill the bar, so the interesting
 * low populations spread out; anything past 50% just maxes out. */
static int bar_level(long pop, long max_pop)
{
    int level = (int)((float)pop / (float)max_pop * HIST_ROWS * 2.0f);
    return level > HIST_ROWS ? HIST_ROWS : level;
}

/* ── §4 simulation   (advances state: mutates the Board, resets the Scene) ─ */

/* Shorthand so the rule table reads like B/S notation: B(3,6) packs the
 * counts 3 and 6 into a bit-set (via rule_mask, §3). */
#define B(...)  rule_mask((int[]){__VA_ARGS__}, \
                          sizeof((int[]){__VA_ARGS__})/sizeof(int))

static void rules_init(void)
{
    RULES[0] = (Rule){ B(3),         B(2,3),         "Conway B3/S23"          };
    RULES[1] = (Rule){ B(3,6),       B(2,3),         "HighLife B36/S23"       };
    RULES[2] = (Rule){ B(3,6,7,8),   B(3,4,6,7,8),   "Day&Night B3678/S34678" };
    RULES[3] = (Rule){ B(2),         0,              "Seeds B2/S"             };
    RULES[4] = (Rule){ B(3,6,8),     B(2,4,5),       "Morley B368/S245"       };
    RULES[5] = (Rule){ B(3,6),       B(1,2,5),       "2x2 B36/S125"           };
}

/* Wipe the board and the chart back to empty.  Each seed_* preset calls this
 * first, so a new pattern starts on a clean grid. */
static void scene_clear(Scene *sc)
{
    memset(sc->board.cells, 0, sizeof sc->board.cells);
    sc->board.gen = 0;
    memset(sc->history.samples, 0, sizeof sc->history.samples);
    sc->history.head = 0;
}

/* Drop a shape onto the board at (r0,c0): each entry is a row/col offset from
 * that anchor, wrapped to stay on the grid. */
static void place(Board *b, const int cells[][2], int n, int r0, int c0)
{
    for (int i = 0; i < n; i++) {
        int r = wrap(r0 + cells[i][0], b->h);
        int c = wrap(c0 + cells[i][1], b->w);
        b->cells[b->buf][r][c] = 1;
    }
}

static void seed_random(Scene *sc)
{
    scene_clear(sc);
    Board *b = &sc->board;
    for (int r = 0; r < b->h; r++)
        for (int c = 0; c < b->w; c++)
            b->cells[b->buf][r][c] = (rand() % 10 < 3) ? 1 : 0;
}

static void seed_glider(Scene *sc)
{
    scene_clear(sc);
    static const int G[][2] = {{0,1},{1,2},{2,0},{2,1},{2,2}};
    place(&sc->board, G, 5, sc->board.h/2 - 1, sc->board.w/2 - 1);
}

static void seed_rpentomino(Scene *sc)
{
    scene_clear(sc);
    static const int P[][2] = {{0,1},{0,2},{1,0},{1,1},{2,1}};
    place(&sc->board, P, 5, sc->board.h/2 - 1, sc->board.w/2 - 1);
}

static void seed_acorn(Scene *sc)
{
    scene_clear(sc);
    static const int A[][2] = {{0,1},{1,3},{2,0},{2,1},{2,4},{2,5},{2,6}};
    place(&sc->board, A, 7, sc->board.h/2 - 1, sc->board.w/2 - 3);
}

static void seed_gosper(Scene *sc)
{
    scene_clear(sc);
    static const int GG[][2] = {
        {0,24},
        {1,22},{1,24},
        {2,12},{2,13},{2,20},{2,21},{2,34},{2,35},
        {3,11},{3,15},{3,20},{3,21},{3,34},{3,35},
        {4, 0},{4, 1},{4,10},{4,16},{4,20},{4,21},
        {5, 0},{5, 1},{5,10},{5,14},{5,16},{5,17},{5,22},{5,24},
        {6,10},{6,16},{6,24},
        {7,11},{7,15},
        {8,12},{8,13},
    };
    int roff = sc->board.h / 4;
    int coff = (sc->board.w > 40) ? 5 : 0;
    place(&sc->board, GG, 36, roff, coff);
}

/* Advance the world one generation under rule `ru`, and return how many cells
 * ended up alive.  This is the heart of the sim: for every cell, count its
 * neighbours, apply the rule, write the result into the other board, then make
 * that board the current one. */
static long board_step(Board *b, const Rule *ru)
{
    int next = 1 - b->buf;
    long pop = 0;

    for (int r = 0; r < b->h; r++)
        for (int c = 0; c < b->w; c++) {
            int n = board_neighbors(b, r, c);
            uint8_t nxt = (uint8_t)cell_next_state(ru, b->cells[b->buf][r][c], n);
            b->cells[next][r][c] = nxt;
            pop += nxt;
        }

    b->buf = next;   /* the board we just filled in becomes "now" */
    b->gen++;
    return pop;
}

/* Save one population count into the rolling chart history. */
static void history_record(PopulationHistory *h, long pop)
{
    h->samples[h->head] = pop;
    h->head = (h->head + 1) % HIST_LEN;
}

/* One frame's worth of simulation: run `steps` generations, logging each
 * one's population for the chart.  Does nothing while paused. */
static void scene_tick(Scene *sc)
{
    if (sc->paused) return;
    const Rule *ru = &RULES[sc->rule_idx];
    for (int s = 0; s < sc->steps; s++) {
        long pop = board_step(&sc->board, ru);
        history_record(&sc->history, pop);
    }
}

/* ── §5 render   (state → screen: reads state, mutates only the terminal) ─ */

static void color_apply(const Theme *th)
{
    start_color();
    use_default_colors();
    /* the nine crowding colours from this theme (plain green on old 8-colour terminals) */
    for (int i = 0; i < RAMP_LEN; i++) {
        short col = (COLORS >= 256) ? th->ramp[i] : COLOR_GREEN;
        init_pair((short)(CP_RAMP + i), col, -1);
    }
    init_pair(CP_HIST, (COLORS >= 256) ? 240 : COLOR_WHITE,  -1);
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* top status line */
    init_pair(CP_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* bottom key hints */
}

static void draw_board(const Board *b)
{
    for (int r = 0; r < b->h; r++) {
        const uint8_t *row = b->cells[b->buf][r];
        for (int c = 0; c < b->w - 1; c++) {
            if (!row[c]) { mvaddch(r + HUD_TOP, c, ' '); continue; }
            int pair = CP_RAMP + board_neighbors(b, r, c);   /* colour by how crowded it is */
            attron(COLOR_PAIR(pair));
            mvaddch(r + HUD_TOP, c, (chtype)(unsigned char)LIVE_CHAR);
            attroff(COLOR_PAIR(pair));
        }
    }
}

static void draw_histogram(const PopulationHistory *h, const Board *b)
{
    long max_pop = (long)b->h * b->w;
    if (max_pop == 0) return;

    attron(COLOR_PAIR(CP_HIST));
    for (int c = 0; c < b->w - 1; c++) {
        /* newest sample at the right edge, older ones to the left */
        int idx = (h->head - (b->w - 1 - c) + HIST_LEN * 2) % HIST_LEN;
        int level = bar_level(h->samples[idx], max_pop);
        for (int hr = 0; hr < HIST_ROWS; hr++) {
            int sr = HUD_TOP + b->h + hr;
            char ch = (level >= HIST_ROWS - hr) ? '#' : '.';  /* fill the bar from the bottom up */
            mvaddch(sr, c, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(CP_HIST));
}

/* Print one HUD line, cut off at `width` so a long line can't spill onto the grid. */
static void draw_hud_row(int row, int pair, int width, const char *text)
{
    char buf[128];
    snprintf(buf, sizeof buf, "%s", text);
    if ((int)strlen(buf) > width) buf[width] = '\0';
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Top line shows the current settings; bottom line lists the keys. */
static void draw_hud(const Scene *sc)
{
    char data[128];
    snprintf(data, sizeof data,
             " Life  %-22s [%d/%d]  %s  gen=%ld  spd=%dx  %s",
             RULES[sc->rule_idx].name, sc->rule_idx + 1, N_RULES,
             THEMES[sc->theme].name, sc->board.gen, sc->steps,
             sc->paused ? "PAUSED" : "running");
    draw_hud_row(0, CP_HUD, sc->board.w, data);
    draw_hud_row(sc->rows - 1, CP_HINT, sc->board.w,
                 " q:quit  spc:pause  n/p:rule  t:theme  r:rand  g:glider  G:gun  e:pento  a:acorn  +/-:spd ");
}

/* Draw the whole frame: grid, then chart, then the HUD lines. */
static void scene_draw(const Scene *sc)
{
    draw_board(&sc->board);
    draw_histogram(&sc->history, &sc->board);
    draw_hud(sc);
}

/* ── §6 app   (terminal setup, user events, and the per-tick combine) ──── */

static void screen_init(const Scene *sc)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_apply(&THEMES[sc->theme]);
}

/* Terminal got resized: refit the board to the new size and start fresh. */
static void screen_resize(Scene *sc)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    sc->rows    = rows;
    sc->board.w = cols;
    sc->board.h = rows - HIST_ROWS - HUD_TOP - HUD_BOT;
    if (sc->board.h < 1) sc->board.h = 1;
    seed_random(sc);
    erase();
}

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

static void install_signals(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
}

/* Act on a resize the signal handler flagged: restart ncurses, then refit. */
static void handle_resize(Scene *sc)
{
    g_need_resize = 0;
    endwin(); refresh();
    screen_resize(sc);
}

/* Handle one key press: switch rules/themes, drop in a preset, change speed. */
static void handle_key(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': g_running = 0; break;
    case ' ':           sc->paused ^= 1; break;
    case 'n':
        sc->rule_idx = (sc->rule_idx + 1) % N_RULES;
        seed_random(sc); break;
    case 'p':
        sc->rule_idx = (sc->rule_idx - 1 + N_RULES) % N_RULES;
        seed_random(sc); break;
    case 't':
        sc->theme = (sc->theme + 1) % N_THEMES;
        color_apply(&THEMES[sc->theme]); break;
    case 'T':
        sc->theme = (sc->theme - 1 + N_THEMES) % N_THEMES;
        color_apply(&THEMES[sc->theme]); break;
    case 'r':  seed_random(sc);     break;
    case 'g':  seed_glider(sc);     break;
    case 'G':  seed_gosper(sc);     break;
    /* R-pentomino is on 'e' because 'p' is already taken by prev-rule */
    case 'e':  seed_rpentomino(sc); break;
    case 'a':  seed_acorn(sc);      break;
    case '+': case '=':
        if (sc->steps < STEPS_MAX) sc->steps++;
        break;
    case '-': case '_':
        if (sc->steps > 1) sc->steps--;
        break;
    }
}

/* Wipe the screen, draw the frame, and push it out in a single update. */
static void frame_render(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    install_signals();
    atexit(cleanup);
    rules_init();
    screen_init(&g_scene);
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    screen_resize(&g_scene);

    long long next = clock_ns();
    while (g_running) {
        if (g_need_resize) handle_resize(&g_scene);

        int ch;
        while ((ch = getch()) != ERR) handle_key(&g_scene, ch);

        scene_tick(&g_scene);                  /* run the simulation */
        frame_render(&g_scene);                /* draw it           */

        next += TICK_NS;                       /* sleep so frames come at a steady rate */
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
