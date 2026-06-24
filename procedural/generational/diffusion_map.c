/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diffusion_map.c — grows tree-like fractal blobs on the terminal.
 *
 * Specks wander in from the edge by random walk and freeze the moment they
 * bump into the growing cluster. Tips reach out and grab passing specks first,
 * so the shape branches into spidery arms (this is "diffusion-limited
 * aggregation"; Witten & Sander 1981). Press 'n' for a faster "Eden" mode that
 * just fills in random edge cells instead — it grows rounder, plainer blobs,
 * which makes the contrast easy to see.
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   r         reset (clear grid, re-seed)
 *   t / T     next / prev theme
 *   +         more walkers per frame (max 10)
 *   -         fewer walkers per frame (min 1)
 *   n         toggle DLA / Eden mode
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra diffusion_map.c -o diffusion_map -lncurses -lm
 *
 * References (the things the code can't tell you):
 *   Witten & Sander, Phys. Rev. Lett. 47, 1400 (1981) — the original DLA paper.
 *   Eden, Proc. 4th Berkeley Symp. (1961) — origin of the Eden growth model.
 *   Meakin, "Fractals, Scaling and Growth Far from Equilibrium" (1998) — the
 *     launch-ring / kill-radius walker recipe used here.
 *   Bourke, paulbourke.net/dataformats/asciiart — the @ # + : . brightness ramp.
 *
 * Sections:
 *   §1 config  §2 rng  §3 state  §4 logic  §5 simulation
 *   §6 effects (none)  §7 render  §8 timing  §9 ncurses/main loop
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    ROWS_MAX        =  80,
    COLS_MAX        = 300,

    RENDER_FPS      =  30,

    WALKER_MIN      =   1,
    WALKER_DEFAULT  =   3,
    WALKER_MAX      =  10,

    MAX_STEPS       = 500,   /* give up on a wandering speck after this many steps */
    LAUNCH_PAD      =   3,   /* specks start this far outside the cluster's edge   */

    N_THEMES        =   5,
    N_AGE_LEVELS    =   5,   /* 5 colour shades, newest cell to oldest             */
    CP_HUD          =   1,   /* colour slot for the top status line                */
    CP_A0           =   2,   /* colour slot for the newest cells; next 4 follow it */
    /* CP_A1 = 3, CP_A2 = 4, CP_A3 = 5, CP_A4 = 6                        */
    CP_HINT         =   7,   /* colour slot for the bottom key-hint line           */

    FPS_UPDATE_MS   = 500,
    MAX_FRAME_MS    = 200,   /* if a frame ran way long (slow terminal, suspend),
                                pretend it was only this long so the loop doesn't
                                try to "catch up" and stall forever                */
};

/* How old a cell can be and still count in each shade, newest to oldest. */
static const int AGE_THRESH[N_AGE_LEVELS] = { 5, 20, 80, 300, INT32_MAX };

#define RENDER_NS   (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* One full turn around a circle. Frozen at this exact rounded value (not the
   more precise 2·M_PI) on purpose: the random launch angles depend on it, so
   changing it would grow a different-looking cluster from the same seed. */
static const float TWO_PI = 6.28318f;

/* ===================================================================== */
/* §2  rng                                                               */
/* ===================================================================== */

static uint32_t g_lcg;

static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1u << 24);
}

/* A random whole number from 0 up to (but not including) n. */
static int lcg_i(int n)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (int)((g_lcg >> 8) % (uint32_t)n);
}

/* Mix the clock into the seed so each run grows a different cluster. */
static void seed_rng(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_lcg = (uint32_t)(ts.tv_nsec ^ ts.tv_sec);
}

/* ===================================================================== */
/* §3  state types                                                        */
/* ===================================================================== */

/*
 * AggregateGrid — the growing blob and everything we know about it.
 *
 * The blob lives on a grid, one cell per character on screen. A grid (rather
 * than a list of points) is the natural fit: in this model specks land on grid
 * squares and stick when they touch a neighbouring filled square, so "the grid"
 * really is the thing being grown, not just storage for it.
 *
 * The grid also carries its own little clock (`frame`). Every cell remembers
 * which frame it stuck on, and the colours fade with age (now minus that
 * stamp). The stamps mean nothing without the clock they were measured against,
 * so the clock rides along here instead of in a separate struct.
 *
 * Everything is fixed-size so the busy loop never has to ask for memory.
 */
typedef struct {
    /* the grid itself — only the simulation writes here; logic and drawing read */
    uint8_t  cell  [ROWS_MAX][COLS_MAX];  /* is this square filled? 0 = empty,
                                             1 = part of the blob. One byte each so
                                             a single memset() wipes it on reset.   */
    uint16_t joined[ROWS_MAX][COLS_MAX];  /* the frame this square got filled, kept
                                             as the low 16 bits to save memory. Past
                                             ~65k frames the number rolls over, so a
                                             very old cell's age can come out fuzzy —
                                             harmless, since it's already in the
                                             oldest, dimmest shade either way.       */

    /* size and shape — these grow on their own as the blob grows */
    int rows, cols;   /* how much of the grid we actually use, matched to the
                         terminal so the blob fills the visible screen.             */
    int cx, cy;       /* the centre cell we start from, so the blob can branch out
                         in every direction.                                         */
    int radius;       /* how far the blob reaches from the centre, measured as the
                         bigger of the row gap and column gap (chessboard distance).
                         Using that instead of true straight-line distance keeps the
                         launch ring and the "too far, give up" check to cheap box
                         comparisons. Grows as the arms reach outward.               */
    int size;         /* how many squares are filled — shown in the status line.    */

    /* the growth clock that joined[] is measured against */
    int frame;        /* ticks up every step, reset to 1 on clear. A cell's age is
                         this minus its joined-stamp (never negative).               */
} AggregateGrid;

/*
 * Scene — the whole program's state in one place, like a table of contents.
 *
 * Only the blob is complex enough to deserve its own type; the rest are just
 * user-facing knobs, grouped by what they affect (the colour theme sits apart
 * from the growth knobs since it only changes how cells look, not how they grow).
 */
typedef struct {
    AggregateGrid aggregate;  /* the blob being grown, plus its clock.            */

    /* knobs that change how the blob grows */
    bool eden_mode;  /* which growth style is running:
                        false = DLA — specks wander in and stick; arms grab passers-by
                                first, so it branches (the default, fractal look).
                        true  = Eden — just fill a random edge cell each step; no
                                wandering, so it grows into a plain rounded blob.
                        Both run on the same grid so you can flip between them and
                        compare — that side-by-side is the whole point of the demo. */
    int  n_walkers;  /* how many cells to add per frame (WALKER_MIN..WALKER_MAX).
                        Only a speed dial — it changes how fast the blob grows, not
                        its shape. Capped because each wandering speck is expensive,
                        so a high count would drag the frame rate down.             */
    bool paused;     /* true = stop growing but keep drawing, so you can study the
                        current shape on screen.                                    */

    /* knob that changes only how the blob looks */
    int  theme;      /* which colour palette, 0..N_THEMES-1.                       */
} Scene;

static Scene g_scene;

/* current terminal size; the §9 setup code sets it, the drawing code reads it */
static int g_scr_rows, g_scr_cols;

/* ===================================================================== */
/* §4  logic  —  questions about the grid; they answer, never change it    */
/* ===================================================================== */

/* True if any of the four squares touching (r,c) is part of the blob. */
static bool aggregate_has_neighbor(const AggregateGrid *agg, int r, int c)
{
    if (r > 0              && agg->cell[r-1][c]) return true;
    if (r < agg->rows - 1  && agg->cell[r+1][c]) return true;
    if (c > 0              && agg->cell[r][c-1]) return true;
    if (c < agg->cols - 1  && agg->cell[r][c+1]) return true;
    return false;
}

/* Pick the colour slot for a cell of this age (newer = brighter). */
static int age_to_pair(int age_delta)
{
    for (int i = 0; i < N_AGE_LEVELS; i++)
        if (age_delta <= AGE_THRESH[i])
            return CP_A0 + i;
    return CP_A0 + N_AGE_LEVELS - 1;
}

/* Pick the on-screen character for a cell of this age (newer = denser glyph). */
static chtype age_to_char(int age_delta)
{
    if (age_delta <= AGE_THRESH[0]) return (chtype)'@';
    if (age_delta <= AGE_THRESH[1]) return (chtype)'#';
    if (age_delta <= AGE_THRESH[2]) return (chtype)'+';
    if (age_delta <= AGE_THRESH[3]) return (chtype)':';
    return (chtype)'.';
}

/* How far (r,c) is from the centre, measured like a chess king: the bigger of
 * the row gap and the column gap. Cheaper than true distance, and that's all
 * the launch ring and the give-up check need. */
static int chebyshev_from_centre(const AggregateGrid *agg, int r, int c)
{
    int dr = abs(r - agg->cy);
    int dc = abs(c - agg->cx);
    return (dr > dc) ? dr : dc;
}

/* How many frames ago this cell stuck. Never negative — a stamp can only look
 * "in the future" after the 16-bit counter rolls over (see the joined[] note). */
static int cell_age(const AggregateGrid *agg, int r, int c)
{
    int age = agg->frame - (int)agg->joined[r][c];
    return (age < 0) ? 0 : age;
}

/* List every empty cell that touches the blob (the spots it could grow into)
 * into the caller's arrays, and return how many there are. */
static int collect_frontier(const AggregateGrid *agg, int *fr, int *fc)
{
    int n = 0;
    for (int r = 0; r < agg->rows; r++) {
        for (int c = 0; c < agg->cols; c++) {
            if (agg->cell[r][c]) continue;
            if (aggregate_has_neighbor(agg, r, c)) {
                fr[n] = r;
                fc[n] = c;
                n++;
            }
        }
    }
    return n;
}

/* ===================================================================== */
/* §5  simulation  —  the only place the grid actually changes            */
/* ===================================================================== */

/* The four steps a wandering speck can take: up, down, left, right. */
static const int DR4[4] = { -1, 1,  0, 0 };
static const int DC4[4] = {  0, 0, -1, 1 };

static void aggregate_set_size(AggregateGrid *agg, int cols, int rows)
{
    agg->cols = (cols < COLS_MAX) ? cols : COLS_MAX;
    agg->rows = (rows < ROWS_MAX) ? rows : ROWS_MAX;
}

/* Wipe the grid, restart the clock, and drop one seed cell in the middle. */
static void aggregate_reset(AggregateGrid *agg)
{
    memset(agg->cell,   0, sizeof agg->cell);
    memset(agg->joined, 0, sizeof agg->joined);
    agg->cx     = agg->cols / 2;
    agg->cy     = agg->rows / 2;
    agg->radius = 0;
    agg->frame  = 1;
    agg->size   = 0;

    /* The starting cell: one square in the middle, stamped with frame 1. */
    agg->cell  [agg->cy][agg->cx] = 1;
    agg->joined[agg->cy][agg->cx] = 1;
    agg->size  = 1;
}

/* Fill square (r,c), stamp it with the current frame, and grow the counts. */
static void aggregate_add_cell(AggregateGrid *agg, int r, int c)
{
    agg->cell  [r][c] = 1;
    agg->joined[r][c] = (uint16_t)(agg->frame & 0xFFFF);
    agg->size++;

    /* if this cell reaches further out than any before, the blob just grew. */
    int reach = chebyshev_from_centre(agg, r, c);
    if (reach > agg->radius) agg->radius = reach;
}

/* Drop a new speck at a random spot on a ring just outside the blob, kept
 * inside the grid. The row part is halved because terminal characters are about
 * twice as tall as they are wide — without that squash the "circle" would look
 * like a tall oval. */
static void launch_on_ring(const AggregateGrid *agg, int *r, int *c)
{
    float angle = lcg_f() * TWO_PI;
    float dist  = (float)(agg->radius + LAUNCH_PAD);

    *r = agg->cy + (int)roundf(dist * sinf(angle) * 0.5f);
    *c = agg->cx + (int)roundf(dist * cosf(angle));

    if (*r < 0) *r = 0;
    if (*r >= agg->rows) *r = agg->rows - 1;
    if (*c < 0) *c = 0;
    if (*c >= agg->cols) *c = agg->cols - 1;
}

/* Nudge the speck one square in a random direction — one step of its wander. */
static void random_walk_step(int *r, int *c)
{
    int dir = lcg_i(4);
    *r += DR4[dir];
    *c += DC4[dir];
}

/*
 * dla_step — grow the blob by one wandering speck.
 *   1. Drop it on the ring just outside the blob.
 *   2. Let it wander until it touches the blob, drifts too far away, or runs out
 *      of patience.
 *   3. Freeze it on the spot where it first touches.
 * Returns true if it stuck, false if it gave up.
 */
static bool dla_step(AggregateGrid *agg)
{
    int r, c;
    launch_on_ring(agg, &r, &c);

    int kill_radius = agg->radius * 2 + LAUNCH_PAD * 3;

    for (int step = 0; step < MAX_STEPS; step++) {
        if (r < 0 || r >= agg->rows || c < 0 || c >= agg->cols)
            return false;                                  /* wandered off the grid */

        if (chebyshev_from_centre(agg, r, c) > kill_radius)
            return false;                                  /* drifted too far — give up */

        if (aggregate_has_neighbor(agg, r, c)) {
            if (agg->cell[r][c] == 0) {
                aggregate_add_cell(agg, r, c);             /* touched the blob — freeze here */
                return true;
            }
            /* this square's already filled — keep wandering */
        }

        random_walk_step(&r, &c);
    }
    return false;                                          /* ran out of steps */
}

/*
 * eden_step — grow the blob the simple way: pick a random empty cell along its
 * edge and fill it. No wandering, so no arms reach out ahead of the rest —
 * that's why Eden grows a plain rounded blob.
 * Returns true if a cell was added.
 */
static bool eden_step(AggregateGrid *agg)
{
    /* room to list the edge cells; static so we don't grab memory every call. */
    static int fr[ROWS_MAX * COLS_MAX];
    static int fc[ROWS_MAX * COLS_MAX];

    int n = collect_frontier(agg, fr, fc);
    if (n == 0) return false;                  /* no edge left — blob fills the grid */

    int idx = lcg_i(n);                        /* every edge cell equally likely */
    aggregate_add_cell(agg, fr[idx], fc[idx]);
    return true;
}

/* Set the user knobs to their starting values. */
static void scene_init(Scene *s)
{
    s->eden_mode = false;
    s->n_walkers = WALKER_DEFAULT;
    s->paused    = false;
}

/*
 * scene_tick — one step of the whole simulation, and the only place the blob
 * grows: if not paused, bump the clock and add this frame's batch of cells
 * (wandering specks in DLA mode, random edge fills in Eden mode).
 */
static void scene_tick(Scene *s)
{
    if (s->paused) return;

    s->aggregate.frame++;

    if (s->eden_mode) {
        for (int i = 0; i < s->n_walkers; i++)
            eden_step(&s->aggregate);
    } else {
        for (int i = 0; i < s->n_walkers; i++)
            dla_step(&s->aggregate);
    }
}

/* ===================================================================== */
/* §6  effects                                                            */
/* ===================================================================== */

/*
 * Nothing lives here. The only visual flourish is the bright-to-dim fade as
 * cells age, and that's worked out fresh while drawing (§7) from each cell's
 * stick-time — it's never stored, so there's nothing for this section to keep.
 */

/* ===================================================================== */
/* §7  render  —  draws the grid to the screen; never changes it          */
/* ===================================================================== */

/*
 * The five colour palettes. Each row is one theme, listed newest cell to
 * oldest, so a cell starts bright and fades as it ages:
 *
 *  Coral:  white -> yellow -> orange -> red -> dark red
 *  Ice:    white -> cyan -> blue -> dark
 *  Lava:   white -> yellow -> orange -> red -> dark
 *  Plasma: white -> pink -> purple -> dark
 *  Mono:   bright -> dark grays
 */
static const short THEME_FG[N_THEMES][N_AGE_LEVELS] = {
    { 231, 226, 208, 196, 124 },  /* Coral  */
    { 231, 123,  51,  27,  17 },  /* Ice    */
    { 231, 220, 208, 196,  88 },  /* Lava   */
    { 231, 213, 165,  93,  54 },  /* Plasma */
    { 255, 251, 244, 238, 235 },  /* Mono   */
};

static const char *THEME_NAME[N_THEMES] = {
    "Coral", "Ice", "Lava", "Plasma", "Mono"
};

/* Stand-in colours for old terminals that only have 8 of them. */
static const short THEME_FG8[N_AGE_LEVELS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_RED, COLOR_RED
};

static void color_init_theme(int theme)
{
    /* The two status lines: yellow up top, cyan at the bottom. */
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, COLOR_BLACK);
        init_pair(CP_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_HINT, COLOR_CYAN,   COLOR_BLACK);
    }

    /* The five age shades for the blob itself. */
    for (int i = 0; i < N_AGE_LEVELS; i++) {
        if (COLORS >= 256)
            init_pair((short)(CP_A0 + i),
                      THEME_FG[theme][i], COLOR_BLACK);
        else
            init_pair((short)(CP_A0 + i),
                      THEME_FG8[i], COLOR_BLACK);
    }
}

static void aggregate_draw(const AggregateGrid *agg)
{
    for (int r = 0; r < agg->rows; r++) {
        for (int c = 0; c < agg->cols; c++) {
            if (!agg->cell[r][c]) continue;

            int age = cell_age(agg, r, c);
            int pair  = age_to_pair(age);
            chtype ch = age_to_char(age);
            attr_t attr = (attr_t)COLOR_PAIR(pair);
            if (age <= AGE_THRESH[0]) attr |= A_BOLD;     /* freshest cells glow bold */

            attron(attr);
            mvaddch(r, c, ch);
            attroff(attr);
        }
    }
}

/* Print one status line, trimmed so it never spills off the right edge. */
static void hud_line(int row, int pair, const char *s)
{
    char buf[128];
    int n = snprintf(buf, sizeof buf, "%s", s);
    if (n > g_scr_cols) buf[g_scr_cols] = '\0';   /* cut it to fit the screen width */

    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw both status lines: mode/theme/counts up top, key reminders at the bottom. */
static void screen_draw_hud(const Scene *s, double fps)
{
    /* Top line: what's happening right now. */
    char data[128];
    snprintf(data, sizeof data,
             " %s | %s | walkers:%d | size:%d | %.1f fps ",
             s->eden_mode ? "Eden" : "DLA",
             THEME_NAME[s->theme],
             s->n_walkers,
             s->aggregate.size,
             fps);
    hud_line(0, CP_HUD, data);

    /* Bottom line: the keys you can press. */
    hud_line(g_scr_rows - 1, CP_HINT,
             " q:quit  spc:pause  r:reset  t/T:theme  +/-:walkers  n:mode ");
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* Paint one full frame: wipe, draw the blob, lay the status lines on top, show it. */
static void render_frame(const Scene *s, double fps)
{
    erase();
    aggregate_draw(&s->aggregate);
    screen_draw_hud(s, fps);
    screen_present();
}

/* ===================================================================== */
/* §8  timing  —  read the clock and sleep, to hold a steady frame rate    */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §9  app  —  terminal setup, signals, keypresses, and the main loop      */
/* ===================================================================== */

static volatile sig_atomic_t g_running    = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Tidy the terminal on exit, and turn quit/resize signals into simple flags. */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void screen_init_ncurses(int theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    start_color();
    color_init_theme(theme);
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
}

static void app_resize(Scene *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
    aggregate_set_size(&s->aggregate, g_scr_cols, g_scr_rows - 1);
    aggregate_reset(&s->aggregate);
    g_need_resize = 0;
}

static bool app_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:
        return false;

    case 'p': case 'P': case ' ':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        aggregate_reset(&s->aggregate);
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        color_init_theme(s->theme);
        break;

    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        color_init_theme(s->theme);
        break;

    case '+': case '=':
        if (s->n_walkers < WALKER_MAX)
            s->n_walkers++;
        break;

    case '-':
        if (s->n_walkers > WALKER_MIN)
            s->n_walkers--;
        break;

    case 'n': case 'N':
        s->eden_mode = !s->eden_mode;
        break;

    default: break;
    }
    return true;
}

/* Start up the screen and reset everything to its opening state. */
static void app_init(Scene *s)
{
    s->theme = 0;
    screen_init_ncurses(s->theme);
    aggregate_set_size(&s->aggregate, g_scr_cols, g_scr_rows - 1);
    aggregate_reset(&s->aggregate);
    scene_init(s);
}

int main(void)
{
    seed_rng();
    install_signals();
    app_init(&g_scene);

    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;
    int64_t last_frame  = clock_ns();

    while (g_running) {

        if (g_need_resize) {
            app_resize(&g_scene);
            scene_init(&g_scene);
            last_frame = clock_ns();
            fps_accum  = 0;
            frame_count = 0;
        }

        scene_tick(&g_scene);

        /* time this frame, and recompute the fps number twice a second */
        int64_t now = clock_ns();
        int64_t dt  = now - last_frame;
        last_frame  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;
        fps_accum += dt;
        frame_count++;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_accum   = 0;
            frame_count = 0;
        }

        render_frame(&g_scene, fps_display);

        /* grab one keypress; if it was 'quit', stop the loop */
        int ch = getch();
        if (ch != ERR && !app_handle_key(&g_scene, ch))
            g_running = 0;

        /* sleep off the rest of this frame's time slice so the rate stays steady */
        int64_t elapsed = clock_ns() - last_frame + dt;
        clock_sleep_ns(RENDER_NS - elapsed);
    }

    endwin();
    return 0;
}
