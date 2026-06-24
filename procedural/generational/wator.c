/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wator.c — Wa-Tor, a little ocean where fish breed and sharks hunt them.
 *
 * Fish ('o') swim, age, and breed; sharks ('X') chase fish, breed, and
 * starve if they go too long without eating. Out of these tiny per-creature
 * rules the two populations rise and fall in waves (classic predator-prey).
 *
 * From A. K. Dewdney's "Wa-Tor", Scientific American 251(6), Dec 1984.
 * Below the ocean is a strip showing the recent fish/shark counts over time.
 *
 * Sections:
 *   §1 config  §2 performance  §3 simulation-data  §4 simulation-step
 *   §5 render  §6 app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ──────────────────────────────────────────────────────────── */

#define TICK_NS         33333333LL   /* ~30 fps                              */
#define MAX_ROWS        128
#define MAX_COLS        320
#define HIST_ROWS       4            /* rows reserved for population graph   */
#define HIST_LEN        512          /* ring-buffer length (columns of history)*/
#define HUD_TOP         1            /* row 0: data HUD                      */
#define HUD_BOT         1            /* bottom row: action HUD               */

/* A history bar reaches full height when that population hits 1/DIV of the
 * grid. Sharks use a smaller divisor (they fill at a lower count) because
 * there are far fewer of them, so their bar still reads clearly. */
#define HIST_FISH_FULL_DIV    2
#define HIST_SHARK_FULL_DIV   10

/*
 * Preset — one named "what kind of ocean" setting. The whole feel of Wa-Tor
 * comes from just three breeding/starving rates plus how crowded the ocean
 * starts. Different settings give very different stories: a calm one where
 * both keep cycling forever, a shark boom that eats everything then crashes,
 * or a fragile one where sharks starve out and fish take over. Each preset is
 * one such story; n/p cycle through them (and reseed the ocean).
 *
 *   fish_breed   : a fish breeds every N ticks. Smaller = fish multiply faster
 *                  = more food = more sharks.
 *   shark_breed  : a shark breeds every N ticks. Smaller = more sharks, more
 *                  hunting pressure on the fish.
 *   shark_starve : a shark dies if it goes this many ticks without eating.
 *                  Larger = sharks survive longer between meals = can overrun.
 *   fish_pct,
 *   shark_pct    : how crowded the ocean is at the start (% of cells filled).
 */
typedef struct {
    const char *name;
    int fish_breed;     /* fish breeds every N ticks                   */
    int shark_breed;    /* shark breeds every N ticks                  */
    int shark_starve;   /* shark dies after N ticks without food       */
    int fish_pct;       /* starting fish density  (% of cells)         */
    int shark_pct;      /* starting shark density (% of cells)         */
} Preset;

#define N_PRESETS 4
static const Preset PRESETS[N_PRESETS] = {
    /* name          Fbr  Sbr  Sstv   F%  S%    what you'll see              */
    { "STABLE",        3,  10,    4,  30,  5 },/* calm, steady up-and-down    */
    { "BLOOM",         2,  15,    3,  25,  3 },/* fish everywhere, few sharks */
    { "BOOM-BUST",     4,   6,    8,  35,  8 },/* sharks overrun, then crash  */
    { "FRAGILE",       5,  12,    2,  20,  6 },/* sharks starve, ocean empties*/
};

#define STEPS_DEF       1
#define STEPS_MAX       20

#define EMPTY  0
#define FISH   1
#define SHARK  2

/* the four steps to a neighbour: north, east, south, west */
static const int DR[4] = {-1,  0, 1, 0};
static const int DC[4] = { 0,  1, 0,-1};

/* colour-pair slots, used below in color_init */
enum {
    CP_FISH_Y = 1,   /* young fish    */
    CP_FISH_O,       /* old fish      */
    CP_SHARK_F,      /* fed shark     */
    CP_SHARK_H,      /* hungry shark  */
    CP_HIST_F,       /* fish history bar  */
    CP_HIST_S,       /* shark history bar */
    CP_HUD,          /* top data row     */
    CP_HINT          /* bottom key hints */
};

/* ── §2 performance — monotonic clock + sleep (the frame-cap helpers) ─────── */

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

/* ── §3 simulation-data — grid state, live params, reset/seed setup ───────── */

/*
 * Ocean — the wrap-around grid the creatures live on. Every cell is empty, a
 * fish, or a shark. "Wrap-around" means the edges connect: walk off the top
 * and you come out the bottom, off the right and you come out the left. So
 * there's no wall anywhere, and every cell has the same four neighbours.
 *
 * The cell's data is split into separate same-sized grids (one for type, one
 * for breed, etc.) instead of one grid of little cell-structs. It's just
 * faster: the busy code touches one of these at a time — it wipes the whole
 * `moved` grid in one go and scans `type` to find the living cells — so
 * keeping each kind of value packed together is friendlier to the CPU cache.
 * The four grids share the same [r][c]; together they ARE one cell.
 *
 *   rows, cols : grid size in cells. cols == terminal width; rows == terminal
 *                height minus the HUD + history rows, so the grid fills the
 *                band between the top and bottom bars.
 *   type       : EMPTY / FISH / SHARK — who's in the cell.
 *   breed      : a count-up timer that means two things. For a fish it's its
 *                age; for a shark it's "ticks since I last bred". When it
 *                reaches the preset's breed number, the creature leaves a baby
 *                and resets to 0. It's a uint8_t and stops at 255 rather than
 *                wrapping to 0 (wrapping would make a long-trapped creature
 *                look newborn again).
 *   hunger     : sharks only — ticks since the shark last ate. At shark_starve
 *                it dies. Also stops at 255.
 *   moved      : "already acted this tick" flag. Creatures take their turns in
 *                random order, so when one moves into a cell that hasn't had
 *                its turn yet, we mark the cell `moved` — when that cell's turn
 *                comes up we skip it, so nothing acts twice in one tick.
 *   order      : the to-do list — the positions of all living cells, shuffled
 *                so they take turns in a random order (a fixed left-to-right
 *                scan would nudge everything to drift the same way). Each entry
 *                packs (r,c) as r*MAX_COLS + c, using the fixed array width
 *                MAX_COLS (not the live `cols`) so the packing still unpacks
 *                correctly after a resize.
 *   fish_pop,
 *   shark_pop  : how many of each are alive right now, recounted every tick;
 *                these feed the HUD and the history graph.
 *   tick       : how many ticks since the last reset (the world's age).
 */
typedef struct {
    int       rows, cols;                  /* grid size in cells             */
    uint8_t   type  [MAX_ROWS][MAX_COLS];  /* EMPTY / FISH / SHARK           */
    uint8_t   breed [MAX_ROWS][MAX_COLS];  /* fish: age; shark: breed timer  */
    uint8_t   hunger[MAX_ROWS][MAX_COLS];  /* shark: ticks since last meal   */
    uint8_t   moved [MAX_ROWS][MAX_COLS];  /* "already acted this tick"      */
    int       order [MAX_ROWS * MAX_COLS]; /* shuffled to-do list of cells   */
    long      fish_pop, shark_pop;         /* current counts                 */
    long long tick;                        /* ticks since last reset         */
} Ocean;

/*
 * PopulationHistory — a rolling record of the last HIST_LEN ticks: how many
 * fish and how many sharks there were each tick. The fun of Wa-Tor is watching
 * the two counts rise and fall in waves (fish peak first, then sharks a few
 * ticks later) — a single live number can't show that, so we keep the recent
 * counts and draw them as the little graph under the ocean. This is real
 * recorded data, not just a decorative trail.
 *
 * It's a "ring": writes loop back to the start and overwrite the oldest entry,
 * so we never run off the end.
 *
 *   fish[], shark[] : the two count tracks; fish[i]/shark[i] is one tick's
 *                     pair. HIST_LEN (512) is wider than any real terminal, so
 *                     the on-screen graph always has enough history to fill it.
 *   head            : where the next write goes; advances (head+1) % HIST_LEN
 *                     each tick. The drawing code walks back from here so the
 *                     newest tick lands in the rightmost column.
 */
typedef struct {
    long fish [HIST_LEN];
    long shark[HIST_LEN];
    int  head;
} PopulationHistory;

/*
 * Scene — everything the program is currently holding, in one place. Bundling
 * it here keeps the parts loosely tied: the simulation functions take an
 * Ocean*, the drawing functions take const pieces, and only the few "do a
 * whole turn / reset" functions take the whole Scene*.
 *
 *   ocean      : the world being simulated.
 *   history    : its fish/shark counts over time (for the graph).
 *   steps      : how many simulation steps to run per drawn frame (the +/-
 *                speed knob, 1..STEPS_MAX).
 *   paused     : when set, the simulation holds still.
 *   preset     : which entry of PRESETS is active. We store just the index, not
 *                a copy of the numbers, so there's one source of truth — the
 *                live settings are always PRESETS[preset].
 *   term_rows,
 *   term_cols  : the terminal's size. Different from ocean.rows/cols (those
 *                leave room for the HUD + graph); used to place the bottom HUD
 *                and to trim HUD text to the screen width.
 */
typedef struct {
    Ocean             ocean;
    PopulationHistory history;
    int  steps;
    int  paused;
    int  preset;
    int  term_rows, term_cols;
} Scene;

static Scene g_scene;   /* the one and only world (~340 KB, lives in BSS) */

/* Fill the grid with a fresh random ocean at the current preset's densities. */
static void ocean_seed(Ocean *oc, const Preset *p)
{
    memset(oc->type,   0, sizeof oc->type);
    memset(oc->breed,  0, sizeof oc->breed);
    memset(oc->hunger, 0, sizeof oc->hunger);
    oc->tick = 0;

    for (int r = 0; r < oc->rows; r++) {
        for (int c = 0; c < oc->cols; c++) {
            int roll = rand() % 100;
            if (roll < p->fish_pct) {
                oc->type [r][c] = FISH;
                oc->breed[r][c] = (uint8_t)(rand() % p->fish_breed);
            } else if (roll < p->fish_pct + p->shark_pct) {
                oc->type  [r][c] = SHARK;
                oc->breed [r][c] = (uint8_t)(rand() % p->shark_breed);
                oc->hunger[r][c] = (uint8_t)(rand() % p->shark_starve);
            }
        }
    }
}

static void history_clear(PopulationHistory *h)
{
    memset(h->fish,  0, sizeof h->fish);
    memset(h->shark, 0, sizeof h->shark);
    h->head = 0;
}

/* Wipe the ocean and the graph and start over with the current preset. */
static void scene_reset(Scene *s)
{
    ocean_seed(&s->ocean, &PRESETS[s->preset]);
    history_clear(&s->history);
}

/* ── §4 simulation-step — the tick: shuffle, fish/shark step, census ──────── */

/* Shuffle an array into a random order, each ordering equally likely
 * (Fisher-Yates; Knuth TAOCP vol. 2, §3.4.2). */
static void ishuffle(int *arr, int n)
{
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = arr[i]; arr[i] = arr[j]; arr[j] = t;
    }
}

/* Step a coordinate by `delta`, wrapping around the edge (off one side comes
 * back the other). The +extent just keeps it positive before the remainder. */
static int wrap(int coord, int delta, int extent)
{
    return (coord + delta + extent) % extent;
}

/* Look at the four neighbours of (r,c) in the given order and report the first
 * one that holds `want` (its position goes in nr,nc). Returns 1 if found, 0 if
 * not. Reads only; the shuffled order makes the pick unbiased among matches. */
static int find_neighbour(const Ocean *oc, int r, int c, const int dirs[4],
                          uint8_t want, int *nr, int *nc)
{
    for (int i = 0; i < 4; i++) {
        int tr = wrap(r, DR[dirs[i]], oc->rows);
        int tc = wrap(c, DC[dirs[i]], oc->cols);
        if (oc->type[tr][tc] == want) { *nr = tr; *nc = tc; return 1; }
    }
    return 0;
}

/* Put a creature, with its counters, into a cell. */
static void cell_set(Ocean *oc, int r, int c, uint8_t type, uint8_t breed, uint8_t hunger)
{
    oc->type  [r][c] = type;
    oc->breed [r][c] = breed;
    oc->hunger[r][c] = hunger;
}

/* Empty a cell (its creature moved away or died). */
static void cell_clear(Ocean *oc, int r, int c)
{
    oc->type  [r][c] = EMPTY;
    oc->breed [r][c] = 0;
    oc->hunger[r][c] = 0;
}

static void fish_step(Ocean *oc, const Preset *p, int r, int c)
{
    if (oc->breed[r][c] < 255) oc->breed[r][c]++;          /* grow one tick older */

    int dirs[4] = {0, 1, 2, 3};
    ishuffle(dirs, 4);
    int nr, nc;
    if (!find_neighbour(oc, r, c, dirs, EMPTY, &nr, &nc))
        return;                                            /* no open water — stay put */

    int     breed_now   = (oc->breed[r][c] >= p->fish_breed);
    uint8_t carry_breed = breed_now ? 0 : oc->breed[r][c];

    cell_set(oc, nr, nc, FISH, carry_breed, 0);            /* swim into the open cell */
    oc->moved[nr][nc] = 1;

    if (breed_now) cell_set(oc, r, c, FISH, 0, 0);         /* old enough: leave a baby */
    else           cell_clear(oc, r, c);                   /* else the old cell empties */
}

static void shark_step(Ocean *oc, const Preset *p, int r, int c)
{
    if (oc->breed [r][c] < 255) oc->breed [r][c]++;        /* tick toward breeding */
    if (oc->hunger[r][c] < 255) oc->hunger[r][c]++;        /* tick toward starving */

    if (oc->hunger[r][c] >= p->shark_starve) {             /* hasn't eaten in too long */
        cell_clear(oc, r, c);
        return;
    }

    int dirs[4] = {0, 1, 2, 3};
    ishuffle(dirs, 4);
    int nr, nc, ate = 0;
    if      (find_neighbour(oc, r, c, dirs, FISH,  &nr, &nc)) ate = 1;  /* fish nearby: eat it */
    else if (!find_neighbour(oc, r, c, dirs, EMPTY, &nr, &nc)) return;  /* no fish, no gap: stay */

    uint8_t new_hunger  = ate ? 0 : oc->hunger[r][c];      /* eating resets hunger to 0 */
    int     breed_now   = (oc->breed[r][c] >= p->shark_breed);
    uint8_t carry_breed = breed_now ? 0 : oc->breed[r][c];

    cell_set(oc, nr, nc, SHARK, carry_breed, new_hunger);  /* move to the chosen cell */
    oc->moved[nr][nc] = 1;

    /* the baby starts with the parent's hunger AFTER the meal (the Wa-Tor rule) */
    if (breed_now) cell_set(oc, r, c, SHARK, 0, new_hunger);
    else           cell_clear(oc, r, c);
}

/* Fill oc->order with the positions of all living cells; return how many. */
static int collect_live_cells(Ocean *oc)
{
    int n = 0;
    for (int r = 0; r < oc->rows; r++)
        for (int c = 0; c < oc->cols; c++)
            if (oc->type[r][c] != EMPTY)
                oc->order[n++] = r * MAX_COLS + c;
    return n;
}

/* Recount how many fish and sharks are alive right now. */
static void census(Ocean *oc)
{
    oc->fish_pop = 0; oc->shark_pop = 0;
    for (int r = 0; r < oc->rows; r++)
        for (int c = 0; c < oc->cols; c++) {
            if      (oc->type[r][c] == FISH)  oc->fish_pop++;
            else if (oc->type[r][c] == SHARK) oc->shark_pop++;
        }
}

/* Record this tick's two counts into the rolling history. */
static void history_push(PopulationHistory *h, long fish, long shark)
{
    h->fish [h->head] = fish;
    h->shark[h->head] = shark;
    h->head = (h->head + 1) % HIST_LEN;
}

static void sim_step(Ocean *oc, PopulationHistory *h, const Preset *p)
{
    memset(oc->moved, 0, sizeof oc->moved);

    int n = collect_live_cells(oc);
    ishuffle(oc->order, n);            /* everyone takes their turn in random order */

    for (int i = 0; i < n; i++) {
        int r = oc->order[i] / MAX_COLS;   /* unpack (r,c) from the stored number */
        int c = oc->order[i] % MAX_COLS;
        if (oc->moved[r][c]) continue;     /* this cell already had its turn */
        if      (oc->type[r][c] == FISH)  fish_step(oc, p, r, c);
        else if (oc->type[r][c] == SHARK) shark_step(oc, p, r, c);
    }

    census(oc);
    history_push(h, oc->fish_pop, oc->shark_pop);
    oc->tick++;
}

static void sim_tick(Scene *s)
{
    if (s->paused) return;
    const Preset *p = &PRESETS[s->preset];
    for (int i = 0; i < s->steps; i++)
        sim_step(&s->ocean, &s->history, p);
}

/* ── §5 render — palette + draw + terminal (reads state; resize reseeds) ──── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_FISH_Y,  51,  -1);  /* bright cyan — young fish     */
        init_pair(CP_FISH_O,  37,  -1);  /* teal        — old fish       */
        init_pair(CP_SHARK_F, 196, -1);  /* bright red  — fed shark      */
        init_pair(CP_SHARK_H, 202, -1);  /* orange      — hungry shark   */
        init_pair(CP_HIST_F,  45,  -1);  /* cyan        — fish bar       */
        init_pair(CP_HIST_S,  160, -1);  /* dark red    — shark bar      */
        init_pair(CP_HUD,     226, -1);  /* bright yellow — top row      */
        init_pair(CP_HINT,    51,  -1);  /* bright cyan   — key hints    */
    } else {
        init_pair(CP_FISH_Y,  COLOR_CYAN,   -1);
        init_pair(CP_FISH_O,  COLOR_CYAN,   -1);
        init_pair(CP_SHARK_F, COLOR_RED,    -1);
        init_pair(CP_SHARK_H, COLOR_YELLOW, -1);
        init_pair(CP_HIST_F,  COLOR_CYAN,   -1);
        init_pair(CP_HIST_S,  COLOR_RED,    -1);
        init_pair(CP_HUD,     COLOR_YELLOW, -1);
        init_pair(CP_HINT,    COLOR_CYAN,   -1);
    }
}

static void scene_ocean(const Ocean *oc, const Preset *p)
{
    for (int r = 0; r < oc->rows; r++) {
        int sr = HUD_TOP + r;
        for (int c = 0; c < oc->cols - 1; c++) {
            uint8_t t = oc->type[r][c];
            if (t == EMPTY) { mvaddch(sr, c, ' '); continue; }
            if (t == FISH) {
                int old    = (oc->breed[r][c] >= p->fish_breed - 1);   /* about to breed: dim it */
                attr_t at  = COLOR_PAIR(old ? CP_FISH_O : CP_FISH_Y);
                if (!old) at |= A_BOLD;
                attron(at);
                mvaddch(sr, c, 'o');
                attroff(at);
            } else {
                int hungry = (oc->hunger[r][c] >= p->shark_starve - 1);/* about to starve: colour it */
                attr_t at  = COLOR_PAIR(hungry ? CP_SHARK_H : CP_SHARK_F) | A_BOLD;
                attron(at);
                mvaddch(sr, c, 'X');
                attroff(at);
            }
        }
    }
}

/* Which history entry belongs in chart column c: newest tick on the right,
 * older ticks to the left. (The +2*HIST_LEN just keeps the number positive.) */
static int history_slot(const PopulationHistory *h, int cols, int c)
{
    return (h->head - (cols - 1 - c) + HIST_LEN * 2) % HIST_LEN;
}

/* How many rows tall this population's bar should be, never taller than the band. */
static int bar_height(long pop, long scale, int rows)
{
    int h = (int)((float)pop / (float)scale * rows);
    return h > rows ? rows : h;
}

static void scene_histogram(const PopulationHistory *h, const Ocean *oc)
{
    long max_pop = (long)oc->rows * (oc->cols - 1);
    if (max_pop == 0) return;

    /* the count at which each bar hits full height (see HIST_*_FULL_DIV):
     * fish at about half the grid, sharks at a tenth — sharks are rarer, so a
     * lower count already fills their bar and it stays easy to see */
    long fish_scale  = max_pop / HIST_FISH_FULL_DIV;  if (fish_scale  < 1) fish_scale  = 1;
    long shark_scale = max_pop / HIST_SHARK_FULL_DIV; if (shark_scale < 1) shark_scale = 1;

    int fish_rows  = HIST_ROWS / 2;  /* top 2 rows: fish    */
    int shark_rows = HIST_ROWS / 2;  /* bottom 2 rows: sharks */

    for (int c = 0; c < oc->cols - 1; c++) {
        int idx = history_slot(h, oc->cols, c);

        /* fish bar — in the top rows, hanging down from the top edge */
        int fl = bar_height(h->fish[idx], fish_scale, fish_rows);
        attron(COLOR_PAIR(CP_HIST_F));
        for (int hr = 0; hr < fish_rows; hr++) {
            int sr = HUD_TOP + oc->rows + hr;
            mvaddch(sr, c, (fl > hr) ? '#' : '.');
        }
        attroff(COLOR_PAIR(CP_HIST_F));

        /* shark bar — in the bottom rows, rising up from the bottom edge */
        int sl = bar_height(h->shark[idx], shark_scale, shark_rows);
        attron(COLOR_PAIR(CP_HIST_S));
        for (int hr = 0; hr < shark_rows; hr++) {
            int sr = HUD_TOP + oc->rows + fish_rows + hr;
            mvaddch(sr, c, (sl >= shark_rows - hr) ? '#' : '.');
        }
        attroff(COLOR_PAIR(CP_HIST_S));
    }
}

static void scene_hud(const Scene *s)
{
    char buf[MAX_COLS + 1];

    /* top row: preset, tick count, populations, speed, paused/running */
    snprintf(buf, sizeof buf,
             " Wa-Tor  preset:%s %d/%d  tick:%lld  fish:%ld  sharks:%ld  spd:%d/f  %s",
             PRESETS[s->preset].name, s->preset + 1, N_PRESETS,
             (long long)s->ocean.tick, s->ocean.fish_pop, s->ocean.shark_pop,
             s->steps, s->paused ? "PAUSED" : "running");
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row: the keys you can press */
    snprintf(buf, sizeof buf,
             " q:quit  spc:pause  r:reseed  +/-:speed  n/p:preset ");
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->term_rows - 1, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* terminal setup and teardown; screen_resize re-fits to the new size and reseeds */

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

static void screen_resize(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    s->term_rows  = rows;
    s->term_cols  = cols;
    s->ocean.cols = cols;
    s->ocean.rows = rows - HUD_TOP - HIST_ROWS - HUD_BOT;
    if (s->ocean.rows < 1) s->ocean.rows = 1;
    scene_reset(s);
    erase();
}

/* ── §6 app — signals, key/resize events, fixed-timestep main loop ────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    screen_init();
    g_scene.steps  = STEPS_DEF;
    g_scene.paused = 0;
    g_scene.preset = 0;              /* start on the first preset */
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    screen_resize(&g_scene);

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize(&g_scene);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0;               break;
            case ' ':           g_scene.paused ^= 1;          break;
            case 'r':           scene_reset(&g_scene); erase();break;
            case '+': case '=':
                if (g_scene.steps < STEPS_MAX) g_scene.steps++;
                break;
            case '-': case '_':
                if (g_scene.steps > 1) g_scene.steps--;
                break;
            case 'n': case 'N':     /* next preset (starts a fresh ocean) */
                g_scene.preset = (g_scene.preset + 1) % N_PRESETS;
                scene_reset(&g_scene); erase();
                break;
            case 'p': case 'P':     /* previous preset (starts a fresh ocean) */
                g_scene.preset = (g_scene.preset + N_PRESETS - 1) % N_PRESETS;
                scene_reset(&g_scene); erase();
                break;
            }
        }

        sim_tick(&g_scene);
        erase();
        scene_ocean(&g_scene.ocean, &PRESETS[g_scene.preset]);
        scene_histogram(&g_scene.history, &g_scene.ocean);
        scene_hud(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
