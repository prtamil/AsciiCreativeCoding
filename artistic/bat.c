/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bat.c — three V-formations of ASCII bats burst from the screen centre and fly
 * off in scripted directions, flapping a 4-frame wing cycle as they go.
 *
 * Refs: Reynolds, "Steering Behaviors for Autonomous Characters" (GDC 1999) for
 * leader/follower formation motion; Williams, *The Animator's Survival Kit* for
 * the looping wing-flap cycle.
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

/* ── §1 config — constants, palette data, timing macros ── */

enum {
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MIN     =  10,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,

    /*
     * CELL_W x CELL_H — sub-cell "pixels" per terminal character cell.  Motion
     * is tracked in these pixels (not in whole cells) so that diagonal flight
     * looks evenly paced even though terminal cells are taller than they are
     * wide.
     */
    CELL_W = 8,
    CELL_H = 16,

    N_FORMATIONS = 3,

    /* How many follower rows fan out behind the leader.  A deeper wedge holds
     * more bats; the count grows as a triangular number (see bat_count). */
    ROWS_DEFAULT =  3,
    ROWS_MIN     =  1,
    ROWS_MAX     =  6,   /* a 6-row wedge is 7 bats wide; fits any terminal */

    STAGGER_TICKS = 30,  /* gap before each later formation launches, so the
                          * three don't all leave the centre at once */
    HOLD_TICKS    = 55,  /* how long a formation waits at centre before re-launch */

    /* One full wing flap takes WING_CYCLE ticks, split into WING_FRAMES poses.
     * WING_CYCLE must divide evenly by WING_FRAMES so each pose lasts the same
     * number of ticks (36 / 4 = 9). */
    WING_CYCLE    = 36,
    WING_FRAMES   =  4,

    FPS_UPDATE_MS = 500,

    /* Frame-pacing for the render loop.  Separate from SIM_FPS_*: those control
     * how often the world updates; these control how often it's drawn. */
    FRAME_CAP_FPS = 60,   /* draw at most this many frames per second */
    MAX_FRAME_MS  = 100,  /* cap a long pause so the sim doesn't try to catch up
                           * with a flood of ticks and lock up */

    HUD_TITLE_COLS = 18,  /* leftmost column the right-side stats may start at,
                           * so they never overwrite the title */
};

/* Biggest a formation can ever get: a full ROWS_MAX wedge holds 28 bats. */
#define MAX_BATS ((ROWS_MAX + 1) * (ROWS_MAX + 2) / 2)   /* = 28 */

/*
 * FORMATION_SPEED — flight speed in pixels per second.
 * LAG_PX          — how far behind the leader each successive row sits (pixels).
 * SPREAD_PX       — how far apart bats sit sideways within a row (pixels).
 */
#define FORMATION_SPEED  260.0f
#define LAG_PX            56.0f
#define SPREAD_PX         32.0f

/*
 * SECONDS_PER_TICK — how much flight time one tick stands for.  Fixed at 1/60 s
 * on purpose: the Hz key changes how often ticks happen, not how far each tick
 * moves, so raising the Hz simply reads as faster flight.
 * PARK_PX — a far-off-screen spot to hide a waiting formation's bats.
 */
#define SECONDS_PER_TICK (1.0f / (float)SIM_FPS_DEFAULT)
#define PARK_PX          (-99999.0f)

/*
 * The six headings a formation cycles through, in radians.  Y points down on a
 * terminal, so a positive sine means downward and a negative sine upward:
 *   330 deg -> upper-right   210 deg -> upper-left    90 deg -> straight down
 *    45 deg -> lower-right   135 deg -> lower-left   270 deg -> straight up
 * Scripted (not random) so the three formations stay visually apart and the
 * scene looks the same every run.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(d)  ((d) * (float)M_PI / 180.0f)

static const float k_angles[6] = {
    DEG2RAD(330.0f), DEG2RAD(210.0f), DEG2RAD( 90.0f),
    DEG2RAD( 45.0f), DEG2RAD(135.0f), DEG2RAD(270.0f),
};
#define N_ANGLES 6

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §2 clock — monotonic time and sleep ── */

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

/* ── §3 color — theme palettes and colour-pair binding ── */

/*
 * ColorID — friendly names for the numbered colour slots ("pairs") this program
 * uses.  ncurses identifies each foreground/background combination by a small
 * integer, so we name them instead of sprinkling bare 1..5 through the code.
 * The numbers start at 1 because slot 0 is reserved for the terminal's own
 * default colours and can't be changed.
 *   COL_G0..COL_G2  — one slot per formation; apply_theme() rebinds these when
 *                     the user cycles themes with t/T.
 *   COL_HUD/COL_HINT— the on-screen text overlay; fixed colours, never themed,
 *                     so the HUD stays readable over any palette.
 */
typedef enum {
    COL_G0   = 1,   /* formation 0 — themed (rebound by apply_theme)        */
    COL_G1   = 2,   /* formation 1 — themed                                 */
    COL_G2   = 3,   /* formation 2 — themed                                 */
    COL_HUD  = 4,   /* HUD data line (top): fixed bright white, never themed */
    COL_HINT = 5,   /* HUD action line (bottom): fixed bright cyan           */
} ColorID;

/*
 * The colour themes, one per row, picked by App.theme (cycled with t/T).  Each
 * row lists the three formation colours, lined up with COL_G0/G1/G2; the values
 * are xterm-256 colour codes, all from the bright half of the range so even the
 * darkest hue stays visible against black.  theme_names[] is the matching HUD
 * label.  On an 8-colour terminal apply_theme() ignores this and uses
 * magenta/cyan/white instead.
 */
#define N_THEMES 5
static const char *const theme_names[N_THEMES] = {
    "NEON  ", "SUNSET", "EMBER ", "FOREST", "CANDY ",
};
static const short bat_themes[N_THEMES][N_FORMATIONS] = {
    { 141,  87, 213 },   /* NEON   — purple / cyan / pink   */
    { 208, 197, 226 },   /* SUNSET — orange / rose / gold   */
    { 196, 208, 220 },   /* EMBER  — red / orange / amber   */
    {  40,  84, 159 },   /* FOREST — green / lime / aqua    */
    { 213, 226,  51 },   /* CANDY  — pink / yellow / cyan   */
};

/* Point the three formation colour slots at theme t.  Safe to call live (t/T);
 * the change shows on the next redraw.  Falls back to magenta/cyan/white on
 * terminals that lack 256 colours. */
static void apply_theme(int t)
{
    if (t < 0 || t >= N_THEMES) t = 0;
    if (COLORS >= 256) {
        init_pair(COL_G0, bat_themes[t][0], COLOR_BLACK);
        init_pair(COL_G1, bat_themes[t][1], COLOR_BLACK);
        init_pair(COL_G2, bat_themes[t][2], COLOR_BLACK);
    } else {
        init_pair(COL_G0, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_G1, COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_G2, COLOR_WHITE,   COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_HUD,  231, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_WHITE, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,  COLOR_BLACK);
    }
    apply_theme(0);   /* default theme */
}

/* ── §4 coords — convert between pixels and character cells ── */

static inline int   px_to_col(float px) { return (int)(px / CELL_W); }
static inline int   px_to_row(float py) { return (int)(py / CELL_H); }
static inline float col_to_px(int col)  { return (float)col * CELL_W + CELL_W * 0.5f; }
static inline float row_to_px(int row)  { return (float)row * CELL_H + CELL_H * 0.5f; }

/* ── §5 data — the types (Bat, Formation, Scene, Screen) ── */

/*
 * Bat — one drawn bat: where its body is, and how far through its wing flap.
 * Position is kept in pixels (see §1) rather than whole cells so diagonal
 * flight looks evenly paced; the conversion to a cell happens only at draw time.
 */
typedef struct {
    float px, py;   /* body centre in pixels (origin top-left, x right, y down) */
    float phase;    /* where in the flap, 0 .. WING_CYCLE; wing_frame() turns it
                     * into one of the four wing poses */
} Bat;

/*
 * Flight — which of two states a formation is in.  A formation flies until its
 * leader leaves the screen, then waits hidden at the centre, then launches
 * again in a new direction.  The names are what matter; the numbers don't.
 */
typedef enum {
    FLIGHT_HOLDING,   /* hidden at centre between flights, counting down to launch */
    FLIGHT_FLYING,    /* on screen, advancing every tick until the leader exits    */
} Flight;

/*
 * Formation — one V-shaped wedge of bats that all share a single heading and
 * speed.  Only the leader (bat 0, at the tip) really moves; each follower's
 * spot is the leader's spot plus a fixed offset turned to face the heading, so
 * the whole wedge holds its shape like a rigid cut-out.  That's why one struct
 * holds the motion once instead of giving every bat its own — there's no
 * per-bat physics.  The bats form a filled triangle, row r holding r+1 of them.
 */
typedef struct {
    Bat     bats[MAX_BATS];   /* slots for the bats; only [0, n_bats) are in use,
                               * bat 0 is the leader.  Fixed pool, never malloc'd */
    int     n_rows;           /* follower rows behind the leader, 1 .. ROWS_MAX */
    int     n_bats;           /* live bat count = (n_rows+1)(n_rows+2)/2, 3 .. 28 */
    float   vx, vy;           /* velocity in pixels/sec; the same for every bat,
                               * which is what keeps the wedge rigid */
    float   angle;            /* heading in radians; always one of k_angles */
    ColorID color;            /* this wedge's colour slot; set once, never changed */
    Flight  state;            /* flying, or holding at centre */
    int     timer;            /* ticks left while holding; at 0 or below it relaunches */
    int     angle_idx;        /* which of k_angles is current; steps +1 each relaunch
                               * so successive flights walk the six headings in turn */
} Formation;

/*
 * Scene — the whole simulated world in one value.  It holds only simulation
 * state; the colour theme lives on App instead, so resetting or resizing (which
 * rebuilds the Scene) never disturbs the chosen palette.
 */
typedef struct {
    Formation formations[N_FORMATIONS];  /* the three wedges */
    int       n_rows;                    /* shared wedge depth, set by +/- keys */
    int       cols, rows;                /* viewport size in cells; the sim's own
                                          * copy, so it never reads from Screen */
    bool      paused;                    /* when true, scene_tick does nothing */
} Scene;

/*
 * Screen — the terminal size in cells, as ncurses reports it (at start-up and
 * on every resize).  Kept apart from Scene's own cols/rows so the simulation
 * never reaches into the I/O side for its bounds; the two are synced when the
 * scene is built, then read independently.
 */
typedef struct { int cols, rows; } Screen;

/* ── §6 logic — pure geometry and tests, no state changes ── */

/*
 * Where bat k sits within the wedge, measured from the leader: how far back
 * (`along`, negative = behind) and how far to the side (`perp`).  The bats fill
 * a triangle — the first row has 1 bat, the next 2, the next 3, and so on — so
 * we first find which row r bat k is in by counting rows until adding one more
 * would pass k, then find its place within that row.  Each row sits one LAG_PX
 * further back, and the bats in a row spread out by SPREAD_PX, centred on the
 * leader's line.
 */
static void bat_form_offset(int k, float *along, float *perp)
{
    int r = 0;
    while ((r + 1) * (r + 2) / 2 <= k) r++;   /* row of bat k */
    int pos = k - r * (r + 1) / 2;            /* its place within that row, 0..r */

    *along = -(float)r * LAG_PX;
    *perp  = ((float)pos - (float)r * 0.5f) * SPREAD_PX;
}

static bool leader_exited(const Formation *f, int cols, int rows)
{
    int c = px_to_col(f->bats[0].px);
    int r = px_to_row(f->bats[0].py);
    return (c < -2 || c > cols + 1 || r < -2 || r > rows + 1);
}

/* How many bats a wedge of n_rows follower rows holds.  Adding up 1 + 2 + ... +
 * (n_rows+1) gives (n_rows+1)(n_rows+2)/2.  Both the sim and the HUD need it. */
static int bat_count(int n_rows)
{
    return (n_rows + 1) * (n_rows + 2) / 2;
}

/* ── §7 simulation — the only code that advances the world ── */

/*
 * Per-formation setup tables, one entry per formation:
 *   k_init_angle_idx — the heading each formation starts on (an index into
 *                      k_angles).  Three different values so the formations
 *                      head different ways from the start.
 *   k_colors         — each formation's colour slot, fixed once and never
 *                      changed (a theme change recolours the slot, not which
 *                      slot a formation owns).
 */
static const int     k_init_angle_idx[N_FORMATIONS] = { 0, 1, 2 };
static const ColorID k_colors[N_FORMATIONS]         = { COL_G0, COL_G1, COL_G2 };

/*
 * A starting flap position for bat k, spread evenly around the cycle plus a
 * little random jitter, so the bats don't all flap in lockstep and the wedge
 * looks alive rather than stamped.  Kept within [0, WING_CYCLE).
 */
static float staggered_wing_phase(int k, int n_bats)
{
    float phase = (float)k * (float)WING_CYCLE / (float)n_bats
                + (float)(rand() % (WING_CYCLE / WING_FRAMES));
    if (phase >= (float)WING_CYCLE) phase -= (float)WING_CYCLE;
    return phase;
}

/*
 * Place bat k on screen: take its offset within the wedge and turn it to point
 * along the formation's heading, then add it to the leader's spot (lx, ly).
 * Also give it a starting flap position.
 */
static void formation_place_bat(Formation *f, int k, float lx, float ly)
{
    float along, perp;
    bat_form_offset(k, &along, &perp);

    /* rotate the (along, perp) offset to face the heading */
    float cos_a = cosf(f->angle);
    float sin_a = sinf(f->angle);
    f->bats[k].px = lx + along * cos_a - perp * sin_a;
    f->bats[k].py = ly + along * sin_a + perp * cos_a;

    f->bats[k].phase = staggered_wing_phase(k, f->n_bats);
}

static void formation_launch(Formation *f, float cx, float cy)
{
    f->vx    = FORMATION_SPEED * cosf(f->angle);
    f->vy    = FORMATION_SPEED * sinf(f->angle);
    f->state = FLIGHT_FLYING;
    f->timer = 0;
    for (int k = 0; k < f->n_bats; k++)
        formation_place_bat(f, k, cx, cy);
}

static void formation_hold(Formation *f, int ticks)
{
    f->state = FLIGHT_HOLDING;
    f->timer = ticks;
    for (int k = 0; k < f->n_bats; k++) {
        f->bats[k].px = PARK_PX;
        f->bats[k].py = PARK_PX;
    }
}

/*
 * Resize a wedge to new_rows without interrupting its flight.  Growing it slots
 * the extra bats into place behind the current leader so they fall straight
 * into formation; shrinking it just lowers the count, leaving the now-unused
 * tail slots to be ignored.
 */
static void formation_set_rows(Formation *f, int new_rows)
{
    if (new_rows < ROWS_MIN) new_rows = ROWS_MIN;
    if (new_rows > ROWS_MAX) new_rows = ROWS_MAX;
    if (new_rows == f->n_rows) return;

    int old_n_bats = f->n_bats;
    f->n_rows = new_rows;
    f->n_bats = bat_count(new_rows);

    if (f->state == FLIGHT_FLYING && f->n_bats > old_n_bats) {
        float lx = f->bats[0].px;
        float ly = f->bats[0].py;
        for (int k = old_n_bats; k < f->n_bats; k++)
            formation_place_bat(f, k, lx, ly);
    }
}

/* Move to the next heading in the six-direction cycle. */
static void advance_to_next_heading(Formation *f)
{
    f->angle_idx = (f->angle_idx + 1) % N_ANGLES;
    f->angle     = k_angles[f->angle_idx];
}

/* Move every bat one tick forward and step its wing flap.  All bats share the
 * same velocity, so the wedge slides along without changing shape. */
static void formation_advance(Formation *f)
{
    for (int k = 0; k < f->n_bats; k++) {
        Bat *b = &f->bats[k];
        b->px    += f->vx * SECONDS_PER_TICK;
        b->py    += f->vy * SECONDS_PER_TICK;
        b->phase += 1.0f;
        if (b->phase >= (float)WING_CYCLE) b->phase -= (float)WING_CYCLE;
    }
}

static void formation_tick(Formation *f, float cx, float cy, int cols, int rows)
{
    if (f->state == FLIGHT_HOLDING) {
        if (--f->timer <= 0) {                /* wait's over: launch a new flight */
            advance_to_next_heading(f);
            formation_launch(f, cx, cy);
        }
        return;
    }

    formation_advance(f);
    if (leader_exited(f, cols, rows))         /* off screen: park and wait */
        formation_hold(f, HOLD_TICKS);
}

/*
 * Set formation `slot` to its starting state at the centre (cx, cy): its depth,
 * colour and heading.  Formation 0 launches at once; the others start holding
 * so they enter a little later, one after another.  A held formation rewinds
 * its heading index by one, because the relaunch will step it forward again and
 * we want it to land back on the intended starting heading.
 */
static void formation_init(Formation *f, int slot, int n_rows, float cx, float cy)
{
    f->color     = k_colors[slot];
    f->n_rows    = n_rows;
    f->n_bats    = bat_count(n_rows);
    f->angle_idx = k_init_angle_idx[slot];
    f->angle     = k_angles[f->angle_idx];

    if (slot == 0) {
        formation_launch(f, cx, cy);
    } else {
        formation_hold(f, slot * STAGGER_TICKS);
        f->angle_idx = (k_init_angle_idx[slot] - 1 + N_ANGLES) % N_ANGLES;
    }
}

/* Build (or rebuild) the whole scene at the given size.  Called at start-up, on
 * 'r' reset, and on resize. */
static void scene_init(Scene *s, int cols, int rows)
{
    s->cols   = cols;
    s->rows   = rows;
    s->paused = false;
    /* keep the current depth across a reset; only the very first call defaults it */
    if (s->n_rows < ROWS_MIN || s->n_rows > ROWS_MAX)
        s->n_rows = ROWS_DEFAULT;

    float cx = col_to_px(cols / 2);
    float cy = row_to_px(rows / 2);

    for (int i = 0; i < N_FORMATIONS; i++)
        formation_init(&s->formations[i], i, s->n_rows, cx, cy);
}

/* Grow or shrink every formation by one row (the +/- keys), clamped to the
 * allowed range.  Takes effect right away, even mid-flight. */
static void scene_set_rows(Scene *s, int delta)
{
    int new_rows = s->n_rows + delta;
    if (new_rows < ROWS_MIN || new_rows > ROWS_MAX) return;
    s->n_rows = new_rows;
    for (int i = 0; i < N_FORMATIONS; i++)
        formation_set_rows(&s->formations[i], new_rows);
}

/* Advance the world by one tick: the one place the simulation moves forward. */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    float cx = col_to_px(s->cols / 2);
    float cy = row_to_px(s->rows / 2);
    for (int i = 0; i < N_FORMATIONS; i++)
        formation_tick(&s->formations[i], cx, cy, s->cols, s->rows);
}

/* ── §8 render — draw the world to the screen, never changing it ── */

/*
 * The four wing poses of one flap.  Read side by side with the body, they give
 * the up/level/down/level look of a flapping bat:
 *   pose    0    1    2    3
 *   shape  /o\  -o-  \o/  -o-
 * Body 'o' looks the same in any direction.  All plain ASCII so they render the
 * same everywhere.
 */
static const char k_bat_lw[WING_FRAMES] = { '/', '-', '\\', '-' };  /* left wing per pose  */
static const char k_bat_rw[WING_FRAMES] = { '\\', '-', '/', '-' };  /* right wing per pose */
#define BAT_BODY 'o'

/* Which wing pose a flap position falls in.  Clamped so a stray value can never
 * read past the wing tables. */
static int wing_frame(float phase)
{
    int frame = (int)phase * WING_FRAMES / WING_CYCLE;
    if (frame < 0 || frame > WING_FRAMES - 1) frame = 0;
    return frame;
}

/* True when all three cells of a bat fit on screen, so none get clipped. */
static bool bat_glyph_on_screen(int col, int row, int cols, int rows)
{
    return row >= 0 && row < rows && col - 1 >= 0 && col + 1 < cols;
}

/* Draw one bat as left-wing / body / right-wing in its formation's colour; the
 * leader is bold so the tip of the V stands out. */
static void stamp_bat(WINDOW *w, int row, int col, int frame, ColorID color, bool leader)
{
    attr_t attr = COLOR_PAIR((int)color);
    if (leader) attr |= A_BOLD;
    wattron(w, attr);
    mvwaddch(w, row, col - 1, (chtype)(unsigned char)k_bat_lw[frame]);
    mvwaddch(w, row, col,     (chtype)BAT_BODY);
    mvwaddch(w, row, col + 1, (chtype)(unsigned char)k_bat_rw[frame]);
    wattroff(w, attr);
}

static void formation_draw(const Formation *f, WINDOW *w, int cols, int rows)
{
    if (f->state == FLIGHT_HOLDING) return;   /* hidden at centre: nothing to draw */

    for (int k = 0; k < f->n_bats; k++) {
        const Bat *b = &f->bats[k];
        int col = px_to_col(b->px);
        int row = px_to_row(b->py);
        if (!bat_glyph_on_screen(col, row, cols, rows)) continue;
        stamp_bat(w, row, col, wing_frame(b->phase), f->color, k == 0);
    }
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    for (int i = 0; i < N_FORMATIONS; i++)
        formation_draw(&s->formations[i], w, s->cols, s->rows);
}

/* Blank a HUD row with spaces so the text sits on a clean bar instead of over
 * the bats (which are drawn first). */
static void hud_bar(int row, int cols)
{
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
}

/* Top row: title on the left, live fps / Hz / run-state on the right.  The
 * right-side block is held back from overrunning the title on a narrow screen. */
static void hud_title_stats(int cols, double fps, int sim_fps, bool paused)
{
    hud_bar(0, cols);
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, 1, " BAT FORMATIONS ");
    char rbuf[48];
    snprintf(rbuf, sizeof rbuf, " %5.1f fps  %3d Hz  %s ",
             fps, sim_fps, paused ? "PAUSED " : "running");
    int hx = cols - (int)strlen(rbuf);
    if (hx < HUD_TITLE_COLS) hx = HUD_TITLE_COLS;
    mvprintw(0, hx, "%s", rbuf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

/* Second row: the adjustable settings (theme, wedge depth, total bat count).
 * Not bold, so the top row stays the dominant line. */
static void hud_params(int cols, int theme, int n_rows)
{
    hud_bar(1, cols);
    char pbuf[64];
    snprintf(pbuf, sizeof pbuf, " theme:%s %d/%d   rows:%d (%d bats) ",
             theme_names[theme], theme + 1, N_THEMES, n_rows,
             N_FORMATIONS * bat_count(n_rows));
    attron(COLOR_PAIR(COL_HUD));
    mvprintw(1, 1, "%s", pbuf);
    attroff(COLOR_PAIR(COL_HUD));
}

/* Bottom row: the list of keys the user can press. */
static void hud_keys(int cols, int rows)
{
    int brow = rows - 1;
    hud_bar(brow, cols);
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    mvprintw(brow, 1,
             "+/-:rows   p:pause   r:reset   t/T:theme   [/]:Hz   q:quit");
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

/* Draw one full frame: the bats first, then the three HUD rows on top. */
static void screen_draw(const Screen *s, const Scene *sc, double fps,
                        int sim_fps, int theme)
{
    erase();
    scene_draw(sc, stdscr);
    hud_title_stats(s->cols, fps, sim_fps, sc->paused);
    hud_params(s->cols, theme, sc->n_rows);
    hud_keys(s->cols, s->rows);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §9 screen — terminal setup, resize, and teardown ── */

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ── §10 app — key handling and the main loop ── */

/*
 * App — the whole running program: the two sub-systems plus the settings that
 * outlive any single scene.  sim_fps and theme are kept here, off the Scene, on
 * purpose: a reset or resize rebuilds the Scene, and these must survive that.
 * running and need_resize are set from signal handlers and read in the loop, so
 * they're volatile sig_atomic_t — the type a handler may safely touch, with the
 * qualifier that stops the compiler from caching a stale value.
 */
typedef struct {
    Scene                 scene;       /* the simulated world */
    Screen                screen;      /* cached terminal size */
    int                   sim_fps;     /* tick rate in Hz, changed by [ and ] */
    int                   theme;       /* current palette, cycled by t and T */
    volatile sig_atomic_t running;     /* set to 0 to leave the loop */
    volatile sig_atomic_t need_resize; /* set to 1 when the terminal was resized */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case 'p': case 'P': case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    /* grow / shrink the wedges */
    case '+': case '=':
        scene_set_rows(&app->scene, +1);
        break;
    case '-': case '_':
        scene_set_rows(&app->scene, -1);
        break;

    case ']':
        if (app->sim_fps < SIM_FPS_MAX) app->sim_fps += SIM_FPS_STEP;
        break;
    case '[':
        if (app->sim_fps > SIM_FPS_MIN) app->sim_fps -= SIM_FPS_STEP;
        break;

    case 't':
        app->theme = (app->theme + 1) % N_THEMES;
        apply_theme(app->theme);
        break;
    case 'T':
        app->theme = (app->theme + N_THEMES - 1) % N_THEMES;
        apply_theme(app->theme);
        break;

    default: break;
    }
    return true;
}

/* One-time start-up: seed the random numbers, install signal handlers, bring up
 * the terminal, then build the scene.  The scene is zeroed first so its depth
 * starts at 0 and picks up the default. */
static void app_init(App *app)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    memset(&app->scene, 0, sizeof app->scene);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
}

int main(void)
{
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        /* 1. apply any pending resize before this frame is measured */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 2. measure elapsed time, clamped so a stall can't spiral the sim */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        /* 3. advance the sim: one fixed tick per accumulated tick-interval */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

        /* 4. refresh the fps readout a couple of times a second */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. cap the frame rate, then draw the frame */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps,
                    app->theme);
        screen_present();

        /* 6. drain one input event (may flip running or mutate the scene) */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
