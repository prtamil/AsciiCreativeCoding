/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sandpile.c — the Abelian sandpile, an automaton that grows a mandala.
 *
 * Drop sand grains on the centre cell. Whenever a cell holds 4 or more grains
 * it spills, handing one grain to each of its four neighbours; grains that
 * spill off the edge are gone. Spilling sets off more spilling, and out of
 * that one rule a perfectly 4-fold-symmetric pattern grows on its own — nobody
 * designed it. Press v to slow it down and watch one avalanche ripple outward.
 *
 * The model and the surprising self-organising behaviour: Bak, Tang &
 * Wiesenfeld, Phys. Rev. Lett. 59 (1987); the "order doesn't matter" proof:
 * Dhar, Phys. Rev. Lett. 64 (1990); the fractal mandala shape: Levine & Propp,
 * AMS Notices 57(8) (2010).
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ──────────────────────────────────────────────────────────── */

#define TICK_NS       33333333LL   /* ~30 fps                               */
#define MAX_ROWS      128
#define MAX_COLS      320
#define QMAX          (MAX_ROWS * MAX_COLS + 1)  /* room for every cell at once, +1 spare */

#define HUD_TOP        1           /* top row reserved for the status line   */
#define HUD_BOT        1           /* bottom row reserved for the key hints   */

#define DROPS_DEF     40           /* grains dropped per frame to start       */
#define DROPS_MAX     500          /* fastest the +/- keys can push it         */

/* A cell spills once it holds this many grains, handing one to each of its 4
 * neighbours. It's 4 because a cell has 4 neighbours — that the two numbers
 * match is exactly what makes the model behave so cleanly. */
#define TOPPLE_THRESHOLD 4

/* What each grain count looks like: blank, then sparser to denser. */
static const char GRAIN_CH[4] = { ' ', '.', 'o', '#' };

enum { CP_G1 = 1, CP_G2, CP_G3, CP_TOPPLE, CP_HUD, CP_HINT };

/* ── themes ──────────────────────────────────────────────────────────────── */

#define N_THEMES 10

/*
 * Theme — one colour scheme for the pile.
 *
 * A cell only ever holds a tiny number (0-3 when settled, briefly more while
 * spilling), so the only way the pattern becomes something you can see is by
 * painting each count a different colour. Keeping the four colours together
 * under one name lets t/T swap the whole look in one go, without the simulation
 * ever knowing colours exist.
 *
 *   name — what the theme is called (shown in the status line)
 *   c    — the four colours for terminals that support 256 colours, ordered
 *          darkest to brightest so brightness reads as "how full this cell is":
 *            c[0] one grain   — dimmest, the thin outer field
 *            c[1] two grains  — the bulk of the pile
 *            c[2] three grains — bright, the ridges about to spill
 *            c[3] spilling    — hottest, a cell mid-spill (only seen in v mode)
 *          Each colour stays in the brighter half of the palette so even c[0]
 *          shows up against a black terminal.
 *   c8   — the same idea for plain 8-colour terminals, kept dim-to-bright as
 *          best the coarse palette allows.
 *
 * The status-line colours aren't here — they're fixed in color_init() so the
 * HUD stays the same readable yellow/cyan no matter which theme is on.
 */
typedef struct {
    const char *name;
    short c [4];   /* 256-colour: grain 1, grain 2, grain 3, spilling */
    short c8[4];   /* 8-colour fallback                               */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        g1   g2   g3  spill   8-colour fallback                          */
    { "AURORA", {  27,  45, 226, 231 }, { COLOR_BLUE,    COLOR_CYAN,   COLOR_YELLOW, COLOR_WHITE  } },
    { "MATRIX", {  28,  46, 190, 231 }, { COLOR_GREEN,   COLOR_GREEN,  COLOR_GREEN,  COLOR_WHITE  } },
    { "EMBER",  {  52, 166, 220, 231 }, { COLOR_RED,     COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE  } },
    { "NOVA",   {  55, 129, 213, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA,COLOR_MAGENTA,COLOR_WHITE  } },
    { "OCEAN",  {  24,  39,  51, 159 }, { COLOR_BLUE,    COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
    { "GOLD",   {  94, 178, 226, 231 }, { COLOR_YELLOW,  COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE  } },
    { "ICE",    {  31, 117, 195, 231 }, { COLOR_CYAN,    COLOR_CYAN,   COLOR_WHITE,  COLOR_WHITE  } },
    { "POISON", {  58, 106, 154, 190 }, { COLOR_GREEN,   COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW } },
    { "NEBULA", {  61, 141, 219, 231 }, { COLOR_MAGENTA, COLOR_MAGENTA,COLOR_WHITE,  COLOR_WHITE  } },
    { "MONO",   { 240, 248, 255, 255 }, { COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  } },
};

/* ── §2 performance ──────────────────────────────────────────────────────── */

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

/* ── §3 simulation  (the only place grid state changes) ──────────────────── */

/*
 * Sandpile — the grid of grains plus the bookkeeping needed to spill it.
 * The whole demo is one rule: a cell with 4+ grains spills, giving one to each
 * of its 4 neighbours, and spilling sets off more spilling until everything
 * holds 0-3 again. This struct holds the state that rule reads and writes,
 * grouped into three parts.
 *
 * ── the grid itself ─────────────────────────────────────────────────────
 *   grain[r][c] — how many grains sit on each cell. A single byte is plenty:
 *     a settled cell holds 0-3, and even mid-spill it never exceeds 3+4 = 7.
 *     The small type also keeps the grid cache-friendly in the hot spill loop.
 *   rows, cols  — the size of the playing field in cells, set on resize to the
 *     terminal minus the two HUD rows. Neighbour checks use these so grains
 *     pushed off the edge simply vanish — that "open edge" is what lets the
 *     pile shed sand instead of filling up forever.
 *   drop_r, drop_c — the one cell sand falls on: the exact centre. Always
 *     dropping in the centre is what turns the result into a symmetric mandala
 *     instead of random texture.
 *
 * ── the to-do list of cells waiting to spill ────────────────────────────
 * We don't rescan the whole grid looking for full cells; we keep a queue of the
 * cells we already know are full and work through it. Because the order of
 * spilling can't change the final picture, a plain first-in-first-out queue is
 * fine — and working through it one batch at a time gives the expanding ring
 * you see in v mode.
 *   q_r[i], q_c[i] — the coordinates of waiting cells, stored in a circular
 *     buffer of QMAX slots. The +1 spare slot lets us tell "full" apart from
 *     "empty" (head == tail always means empty).
 *   queued[r][c]   — a yes/no flag per cell so a cell never lands in the queue
 *     twice in one avalanche. It shares the grid's coordinates, which is why the
 *     to-do list lives inside Sandpile rather than off on its own.
 *   q_head, q_tail — where we read from and write to in the circular buffer.
 *
 * ── numbers for the status line only (they never steer anything) ─────────
 *   total_drops    — grains dropped since the last reset; basically the age.
 *   last_avalanche — how many spills the most recent avalanche took. It climbs
 *     and jumps around wildly as the pile fills — that erratic spiking is the
 *     visible signature of self-organising criticality.
 */
typedef struct {
    uint8_t  grain[MAX_ROWS][MAX_COLS];   /* grains on each cell              */
    int      rows, cols;                  /* playing-field size, in cells     */
    int      drop_r, drop_c;              /* where sand falls — the centre    */

    int      q_r[QMAX], q_c[QMAX];        /* cells waiting to spill           */
    uint8_t  queued[MAX_ROWS][MAX_COLS];  /* "already in the queue?" flag     */
    int      q_head, q_tail;              /* read / write spots in the buffer */

    long long total_drops;                /* grains dropped since reset       */
    long      last_avalanche;             /* spills in the most recent avalanche */
} Sandpile;

/*
 * Scene — everything the program needs, in one place. It keeps the three
 * concerns the program juggles together while letting the layers stay separate:
 * functions that change the pile take just a Sandpile*, and only the whole-tick
 * drivers (sim_tick, screen_resize) take the whole Scene*, so bundling things
 * here never lets the simulation reach into the rendering.
 *
 * ── the thing being simulated ───────────────────────────────────────────
 *   pile — the Sandpile above; the only part the simulation changes.
 *
 * ── knobs the user turns ────────────────────────────────────────────────
 *   drops_per_frame — grains dropped each frame in normal mode (+/- keys,
 *     1..DROPS_MAX). Pure speed control: the final shape is identical at any
 *     rate, so turning it up just makes the mandala grow faster.
 *   paused — space toggles it; the simulation freezes but drawing keeps going.
 *   wave_view — v toggles how you watch it. 0 = normal (drop a burst and settle
 *     it all instantly). 1 = slow motion (drop one grain, then advance its
 *     avalanche one ring per frame) so you can actually watch one ripple out.
 *
 * ── drawing state ───────────────────────────────────────────────────────
 *   theme — which colour scheme is active (t/T cycle). Lives here rather than
 *     with the sim knobs because colour is a drawing thing, even if a key drives it.
 *   term_rows, term_cols — the terminal's current size, used only to place the
 *     status lines and trim them to width. Different from pile.rows/cols (the
 *     terminal minus the two HUD rows); keeping both makes the layout explicit.
 */
typedef struct {
    Sandpile pile;
    int      drops_per_frame;             /* grains dropped per frame          */
    int      paused;
    int      wave_view;                   /* slow-motion: watch one avalanche  */
    int      theme;                       /* which colour scheme is active     */
    int      term_rows, term_cols;        /* terminal size, for HUD placement  */
} Scene;

/* Steps to the four neighbours: up, right, down, left. */
static const int DR[4] = { -1,  0,  1,  0 };
static const int DC[4] = {  0,  1,  0, -1 };

/* True if (r,c) is on the grid; false means a grain just spilled off the edge. */
static int in_pile(const Sandpile *p, int r, int c)
{
    return r >= 0 && r < p->rows && c >= 0 && c < p->cols;
}

/* Wipe the grid and counters and put the drop point back in the centre. */
static void sandpile_reset(Sandpile *p)
{
    memset(p->grain, 0, sizeof p->grain);
    p->total_drops    = 0;
    p->last_avalanche = 0;
    p->drop_r = p->rows / 2;
    p->drop_c = p->cols / 2;
}

/* ── the to-do list: a circular queue of cells waiting to spill ──────────── */

static void enqueue(Sandpile *p, int r, int c)
{
    if (p->queued[r][c]) return;        /* already waiting — don't add it twice */
    p->queued[r][c] = 1;
    p->q_r[p->q_tail] = r;
    p->q_c[p->q_tail] = c;
    p->q_tail = (p->q_tail + 1) % QMAX;
}

/* Take the next cell off the front of the to-do list. */
static void dequeue(Sandpile *p, int *r, int *c)
{
    *r = p->q_r[p->q_head];
    *c = p->q_c[p->q_head];
    p->q_head = (p->q_head + 1) % QMAX;
    p->queued[*r][*c] = 0;
}

static int avalanche_pending(const Sandpile *p) { return p->q_head != p->q_tail; }

/* How many cells are waiting right now — that batch is exactly one ring of the
 * avalanche, which is what makes the slow-motion view show expanding rings. */
static int queued_count(const Sandpile *p)
{
    return (p->q_tail - p->q_head + QMAX) % QMAX;
}

/* ── spilling ───────────────────────────────────────────────────────────── */

/* Give one grain to each on-grid neighbour, and add any neighbour this just
 * pushed to 4+ to the to-do list. Grains pushed off the edge are simply lost. */
static void distribute_to_neighbours(Sandpile *p, int r, int c)
{
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (in_pile(p, nr, nc)) {
            p->grain[nr][nc]++;
            if (p->grain[nr][nc] >= TOPPLE_THRESHOLD) enqueue(p, nr, nc);
        }
    }
}

/* Spill one waiting cell: drop its count by 4, hand those grains out, and put
 * any cell still over the limit (maybe itself) back on the list. Returns 1 if it
 * actually spilled, 0 if it had already dropped below 4 before its turn came. */
static int topple_one(Sandpile *p)
{
    int r, c;
    dequeue(p, &r, &c);

    if (p->grain[r][c] < TOPPLE_THRESHOLD) return 0;   /* settled before its turn */

    p->grain[r][c] -= TOPPLE_THRESHOLD;                /* one grain per neighbour */
    distribute_to_neighbours(p, r, c);
    if (p->grain[r][c] >= TOPPLE_THRESHOLD) enqueue(p, r, c);  /* still too full */
    return 1;
}

/* Spill just the cells waiting right now — one ring of the avalanche — so the
 * slow-motion view advances by a single ring per call. Returns spills done. */
static long avalanche_wave(Sandpile *p)
{
    long topples = 0;
    int  ring = queued_count(p);        /* freeze the count first, ignore refills */
    for (int i = 0; i < ring; i++)
        topples += topple_one(p);
    return topples;
}

/* Spill until nothing is left waiting. Returns the total number of spills. */
static long avalanche_full(Sandpile *p)
{
    long topples = 0;
    while (avalanche_pending(p)) topples += topple_one(p);
    return topples;
}

/* Begin a fresh avalanche at (r,c): clear the to-do list, then queue that cell. */
static void begin_avalanche(Sandpile *p, int r, int c)
{
    memset(p->queued, 0, sizeof p->queued);
    p->q_head = p->q_tail = 0;
    enqueue(p, r, c);
}

static void drop_grain(Sandpile *p)
{
    p->grain[p->drop_r][p->drop_c]++;
    p->total_drops++;
    if (p->grain[p->drop_r][p->drop_c] >= TOPPLE_THRESHOLD)
        begin_avalanche(p, p->drop_r, p->drop_c);
}

/* The one place per frame where the pile actually advances. */
static void sim_tick(Scene *s)
{
    if (s->paused) return;
    Sandpile *p = &s->pile;

    if (s->wave_view) {
        /* slow motion: one grain, then advance its avalanche one ring per frame */
        if (avalanche_pending(p)) {
            p->last_avalanche += avalanche_wave(p);
        } else {
            drop_grain(p);
            p->last_avalanche = 0;
        }
    } else {
        /* normal: drop a burst, settling each grain fully and instantly */
        for (int i = 0; i < s->drops_per_frame; i++) {
            drop_grain(p);
            if (avalanche_pending(p))
                p->last_avalanche = avalanche_full(p);
        }
    }
}

/* ── §4 render  (reads the grid and paints the screen; never changes the grid) */

/* Load the active theme's four grain colours into ncurses' colour slots. */
static void theme_apply(int theme)
{
    const Theme  *th      = &k_themes[theme];
    const short  *palette = (COLORS >= 256) ? th->c : th->c8;  /* pick by terminal */
    init_pair(CP_G1,     palette[0], -1);   /* one grain    */
    init_pair(CP_G2,     palette[1], -1);   /* two grains   */
    init_pair(CP_G3,     palette[2], -1);   /* three grains */
    init_pair(CP_TOPPLE, palette[3], -1);   /* spilling     */
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* status line, fixed */
    init_pair(CP_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* key hints, fixed   */
    theme_apply(theme);
}

/*
 * Pick the colour, character, and boldness for a cell from its grain count.
 * 4+ is the bright spill flash, only seen in slow-motion view; counts 1/2/3 get
 * brighter and bolder so a fuller cell visibly stands out as closer to spilling.
 */
static void cell_appearance(int grains, int *pair, char *glyph, attr_t *bold)
{
    if (grains >= TOPPLE_THRESHOLD) {       /* spilling right now */
        *pair = CP_TOPPLE; *glyph = '*'; *bold = A_BOLD;
    } else {                                /* settled, 1/2/3 grains */
        *pair  = CP_G1 + (grains - 1);
        *glyph = GRAIN_CH[grains];
        *bold  = (grains >= 2) ? A_BOLD : 0;   /* make the fuller cells glow */
    }
}

/*
 * Draw the pile. The drop point is the grid centre and the grid fills the
 * screen, so the symmetric pattern always sits in the middle. Each cell is
 * coloured by its grain count; empties are left blank (erase() cleared them).
 */
static void scene_draw(const Sandpile *p)
{
    for (int r = 0; r < p->rows; r++) {
        for (int c = 0; c < p->cols - 1; c++) {   /* skip last column so the cursor can't wrap */
            int grains = p->grain[r][c];
            if (grains == 0) continue;             /* leave empty cells blank */

            int    pair;
            char   glyph;
            attr_t bold;
            cell_appearance(grains, &pair, &glyph, &bold);

            attron(COLOR_PAIR(pair) | bold);
            mvaddch(HUD_TOP + r, c, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | bold);
        }
    }
}

static void scene_hud(const Scene *s)
{
    char buf[256];

    /* top row: title, theme, grain count, last avalanche, rate, state */
    snprintf(buf, sizeof buf,
             " Sandpile  %s  grains:%lld  avalanche:%ld  rate:%d/f  %s",
             k_themes[s->theme].name, (long long)s->pile.total_drops,
             s->pile.last_avalanche, s->drops_per_frame,
             s->paused ? "PAUSED" : (s->wave_view ? "WAVE" : "running"));
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row: the keys you can press */
    snprintf(buf, sizeof buf,
             " q:quit  spc:pause  r:reset  t:theme  v:avalanche  +/-:rate ");
    if ((int)strlen(buf) > s->term_cols) buf[s->term_cols] = '\0';
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->term_rows - 1, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_init(const Scene *s)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(s->theme);
}

/* Terminal got resized: refit the grid to the new size and start over. */
static void screen_resize(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    s->term_rows  = rows;
    s->term_cols  = cols;
    s->pile.cols  = cols;
    s->pile.rows  = rows - HUD_TOP - HUD_BOT;   /* leave room for the two HUD rows */
    if (s->pile.rows < 1) s->pile.rows = 1;
    sandpile_reset(&s->pile);
    erase();
}

/* ── §5 app  (signals, key handling, main loop) ──────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static Scene g_scene;   /* the one Scene — too big for the stack, so it lives here */

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

    g_scene.theme           = 0;
    g_scene.drops_per_frame = DROPS_DEF;
    g_scene.paused          = 0;
    g_scene.wave_view       = 0;

    screen_init(&g_scene);
    screen_resize(&g_scene);

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize(&g_scene);
        }

        /* handle keys: they only flip settings, they don't advance the pile */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0;              break;
            case ' ':           g_scene.paused ^= 1;                 break;
            case 'r': case 'R': sandpile_reset(&g_scene.pile); erase(); break;
            case 't': case 'T':
                g_scene.theme = (g_scene.theme + 1) % N_THEMES;
                theme_apply(g_scene.theme);
                break;
            case 'v': case 'V': g_scene.wave_view ^= 1;              break;
            case '+': case '=':
                if (g_scene.drops_per_frame < DROPS_MAX) g_scene.drops_per_frame++;
                break;
            case '-': case '_':
                if (g_scene.drops_per_frame > 1) g_scene.drops_per_frame--;
                break;
            }
        }

        sim_tick(&g_scene);         /* advance the pile one step */

        /* draw, then sleep just enough to hold the frame rate */
        erase();
        scene_draw(&g_scene.pile);
        scene_hud(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
