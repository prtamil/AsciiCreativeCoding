/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bat.c  —  groups of ASCII bats erupting from the screen centre into the dark
 *
 * Three groups of bats burst outward from the screen centre, each heading in a
 * different direction.  Every group flies in a V-formation — one leader at the
 * tip with follower rows fanning out behind.  Each bat flaps through a 4-frame
 * wing cycle.
 *
 * Formation shape — filled triangle (Pascal rows), n_rows = 3 shown:
 *
 *            /o\                 ← row 0: 1 bat  (leader, bold)
 *         /o\   /o\              ← row 1: 2 bats
 *      /o\   /o\   /o\           ← row 2: 3 bats
 *   /o\   /o\   /o\   /o\        ← row 3: 4 bats
 *
 * Row r has r+1 bats evenly spaced.  Total bats = (n+1)(n+2)/2.
 *   n_rows 1 → 3 bats    n_rows 4 → 15 bats
 *   n_rows 2 → 6 bats    n_rows 5 → 21 bats
 *   n_rows 3 → 10 bats   n_rows 6 → 28 bats
 *
 * Pressing + / - grows or shrinks the triangle live.  Flying groups gain or
 * lose bats immediately, placed at the correct position in the formation.
 *
 * Wing frames (triangle wave):
 *   0  /o\   wings up
 *   1  -o-   wings level
 *   2  \o/   wings down
 *   3  -o-   wings level
 *
 * When the leader of a group exits the screen the group pauses briefly then
 * re-launches from the centre in a new direction, cycling through 6 presets.
 *
 * Group colours:
 *   Group 0 — light purple  (xterm 141) / magenta fallback
 *   Group 1 — electric cyan (xterm  87) / cyan fallback
 *   Group 2 — pink-magenta  (xterm 213) / white fallback
 *
 * Keys:
 *   q / ESC   quit
 *   + / -     add / remove one row (1–6 rows, 3–28 bats per group)
 *   r         reset all groups to centre
 *   p / spc   pause / resume
 *   ] [       faster / slower simulation
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra Artistic/bat.c -o bat -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  coords
 *   §5  bat
 *   §6  scene
 *   §7  screen
 *   §8  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : V-formation particle system with triangular row layout.
 *                  Row r contains r+1 bats; total bats = (n+1)(n+2)/2
 *                  (triangular number).  Each bat's position is computed
 *                  analytically from group heading angle, row index, and
 *                  column index — no physics integration, pure kinematics.
 *
 * Data-structure : Fixed-size group array; each group stores heading (angle),
 *                  position, state (FLYING / RESPAWNING), and n_rows.  Bats
 *                  within a group are reconstructed each frame from group
 *                  state, avoiding per-bat storage for the formation shape.
 *
 * Math           : Row r, column c bat offset from leader:
 *                    dx = c·SPACING_X − r·SPACING_X/2  (fan-out)
 *                    dy = r·SPACING_Y                   (depth behind leader)
 *                  Rotation matrix applied to (dx,dy) using group heading θ.
 *
 * Rendering      : 4-frame wing cycle (0→up, 1→level, 2→down, 3→level)
 *                  implemented as a triangle wave index: frame%4 maps to
 *                  one of 3 ASCII bat shapes.  Leader drawn bold.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MIN     =  10,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,

    /*
     * CELL_W × CELL_H — physical pixels per terminal cell.
     * Physics lives in pixel space so that diagonal motion is isotropic
     * regardless of the non-square terminal cell aspect.
     */
    CELL_W = 8,
    CELL_H = 16,

    N_GROUPS = 3,

    /*
     * n_rows — number of follower rows behind the leader (1 = leader only pair,
     * 6 = six rows deep).  n_bats = (n_rows+1)*(n_rows+2)/2.
     * MAX_BATS covers the largest possible formation.
     */
    ROWS_DEFAULT =  3,
    ROWS_MIN     =  1,
    ROWS_MAX     =  6,   /* row 6 → 7 bats wide; comfortable on any terminal */

    /*
     * STAGGER_TICKS — how many ticks each successive group waits before
     * launching.  Prevents all three groups overlapping at t=0.
     */
    STAGGER_TICKS = 30,
    PAUSE_TICKS   = 55,  /* ticks to wait at centre before re-launch */

    /*
     * WING_CYCLE — ticks for one complete 4-frame wing flap.
     * 36 ticks @ 60 fps ≈ 0.6 s per flap ≈ 1.7 Hz.
     */
    WING_CYCLE    = 36,

    HUD_COLS      = 56,
    FPS_UPDATE_MS = 500,
};

/*
 * MAX_BATS — worst-case bat count per group (n_rows = ROWS_MAX).
 * Triangular number: (ROWS_MAX+1)*(ROWS_MAX+2)/2.
 * ROWS_MAX=6 → 7*8/2 = 28.
 */
#define MAX_BATS ((ROWS_MAX + 1) * (ROWS_MAX + 2) / 2)   /* = 28 */

/*
 * GROUP_SPEED — physical pixels per second.
 * LAG_PX      — along-flight gap between successive rows (pixels).
 * SPREAD_PX   — lateral gap per rank unit (pixels).
 *               Row k sits at ±k × SPREAD_PX from the centreline.
 */
#define GROUP_SPEED  260.0f
#define LAG_PX        56.0f
#define SPREAD_PX     32.0f

/*
 * 6 preset flight angles (radians, y-down screen coordinates).
 * Screen y-down: sin > 0 = downward, sin < 0 = upward.
 *
 *   330° → upper-right    210° → upper-left    90° → straight down
 *    45° → lower-right    135° → lower-left   270° → straight up
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

/* ===================================================================== */
/* §2  clock                                                              */
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
/* §3  color                                                              */
/* ===================================================================== */

typedef enum {
    COL_G0  = 1,   /* group 0 — light purple  */
    COL_G1  = 2,   /* group 1 — electric cyan */
    COL_G2  = 3,   /* group 2 — pink-magenta  */
    COL_HUD = 4,
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_G0,  141, COLOR_BLACK);
        init_pair(COL_G1,   87, COLOR_BLACK);
        init_pair(COL_G2,  213, COLOR_BLACK);
        init_pair(COL_HUD, 231, COLOR_BLACK);
    } else {
        init_pair(COL_G0,  COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_G1,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_G2,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_HUD, COLOR_WHITE,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

static inline int   px_to_col(float px) { return (int)(px / CELL_W); }
static inline int   px_to_row(float py) { return (int)(py / CELL_H); }
static inline float col_to_px(int col)  { return (float)col * CELL_W + CELL_W * 0.5f; }
static inline float row_to_px(int row)  { return (float)row * CELL_H + CELL_H * 0.5f; }

/* ===================================================================== */
/* §5  bat                                                                */
/* ===================================================================== */

/*
 * Wing animation — 4-frame triangle wave.
 * Body char 'o' is direction-neutral so it looks right on any heading.
 */
static const char k_bat_lw[4] = { '/', '-', '\\', '-' };
static const char k_bat_rw[4] = { '\\', '-', '/', '-' };
#define BAT_BODY 'o'

typedef struct {
    float px, py;   /* pixel-space position of body-cell centre */
    float phase;    /* wing phase in [0, WING_CYCLE)            */
} Bat;

typedef enum {
    GRP_PAUSE,    /* waiting at centre, invisible */
    GRP_FLYING,   /* bats in flight               */
} GroupState;

/*
 * bat_form_offset — formation position of bat k in flight-frame coordinates.
 *
 * Bats are laid out in a filled triangle (Pascal rows):
 *   Row r contains r+1 bats: indices r*(r+1)/2 … (r+1)*(r+2)/2 − 1.
 *
 * To find row r from k, we walk up until the next row would overshoot k.
 * Position within row:  pos = k − r*(r+1)/2,  ranging 0 … r.
 *
 * along = −r × LAG_PX          (behind the leader)
 * perp  = (pos − r/2) × SPREAD_PX  (centred on the leader's lateral axis)
 *
 *   r=0, pos=0:  perp = 0                           (leader)
 *   r=1, pos=0:  perp = −½S   pos=1: +½S            (2 bats)
 *   r=2, pos=0:  perp = −S    pos=1:  0   pos=2: +S (3 bats)
 *   r=3, pos=0:  perp = −3/2S … pos=3: +3/2S        (4 bats)
 */
static void bat_form_offset(int k, float *along, float *perp)
{
    /* Determine row r: largest r with r*(r+1)/2 <= k */
    int r = 0;
    while ((r + 1) * (r + 2) / 2 <= k) r++;
    int pos = k - r * (r + 1) / 2;

    *along = -(float)r * LAG_PX;
    *perp  = ((float)pos - (float)r * 0.5f) * SPREAD_PX;
}

typedef struct {
    Bat        bats[MAX_BATS];
    int        n_rows;   /* follower rows: 1..ROWS_MAX          */
    int        n_bats;   /* = (n_rows+1)*(n_rows+2)/2            */
    float      vx, vy;   /* px / second, identical for all bats */
    float      angle;    /* current flight direction (radians)  */
    ColorID    color;
    GroupState state;
    int        timer;    /* ticks remaining in pause            */
    int        angle_idx;
} Group;

/*
 * group_place_bat — compute world pixel position for bat k given the
 * leader's current position and the group's flight angle, and write it.
 * Also randomises the wing phase.
 */
static void group_place_bat(Group *g, int k, float lx, float ly)
{
    float cos_a = cosf(g->angle);
    float sin_a = sinf(g->angle);
    float along, perp;
    bat_form_offset(k, &along, &perp);
    /*
     * Rotate flight-frame offset into world pixel space.
     * Flight frame axes:
     *   along → (cos_a,  sin_a)   forward
     *   perp  → (−sin_a, cos_a)   left-normal
     */
    g->bats[k].px = lx + along * cos_a - perp * sin_a;
    g->bats[k].py = ly + along * sin_a + perp * cos_a;
    g->bats[k].phase = (float)k * (float)WING_CYCLE / (float)g->n_bats
                     + (float)(rand() % (WING_CYCLE / 4));
    if (g->bats[k].phase >= (float)WING_CYCLE)
        g->bats[k].phase -= (float)WING_CYCLE;
}

static void group_launch(Group *g, float cx, float cy)
{
    g->vx    = GROUP_SPEED * cosf(g->angle);
    g->vy    = GROUP_SPEED * sinf(g->angle);
    g->state = GRP_FLYING;
    g->timer = 0;
    for (int k = 0; k < g->n_bats; k++)
        group_place_bat(g, k, cx, cy);
}

static void group_start_pause(Group *g, int ticks)
{
    g->state = GRP_PAUSE;
    g->timer = ticks;
    for (int k = 0; k < g->n_bats; k++) {
        g->bats[k].px = -99999.0f;
        g->bats[k].py = -99999.0f;
    }
}

/*
 * group_set_rows — resize the formation to new_rows while preserving motion.
 *
 * Growing (new_rows > g->n_rows):
 *   New bats are placed at the correct formation positions relative to the
 *   leader's current pixel location and given random wing phases.  They
 *   immediately inherit the group velocity so the formation stays rigid.
 *
 * Shrinking (new_rows < g->n_rows):
 *   Simply decrement n_bats/n_rows — the tail bats are no longer iterated
 *   or drawn.  No memory needs clearing; the slots are just ignored.
 */
static void group_set_rows(Group *g, int new_rows)
{
    if (new_rows < ROWS_MIN) new_rows = ROWS_MIN;
    if (new_rows > ROWS_MAX) new_rows = ROWS_MAX;
    if (new_rows == g->n_rows) return;

    int old_n_bats = g->n_bats;
    g->n_rows = new_rows;
    g->n_bats = (new_rows + 1) * (new_rows + 2) / 2;

    if (g->state == GRP_FLYING && g->n_bats > old_n_bats) {
        float lx = g->bats[0].px;
        float ly = g->bats[0].py;
        for (int k = old_n_bats; k < g->n_bats; k++)
            group_place_bat(g, k, lx, ly);
    }
}

static bool leader_exited(const Group *g, int cols, int rows)
{
    int c = px_to_col(g->bats[0].px);
    int r = px_to_row(g->bats[0].py);
    return (c < -2 || c > cols + 1 || r < -2 || r > rows + 1);
}

static void group_tick(Group *g, float cx, float cy, int cols, int rows)
{
    if (g->state == GRP_PAUSE) {
        if (--g->timer <= 0) {
            g->angle_idx = (g->angle_idx + 1) % N_ANGLES;
            g->angle     = k_angles[g->angle_idx];
            group_launch(g, cx, cy);
        }
        return;
    }

    float dt_sec = 1.0f / (float)SIM_FPS_DEFAULT;
    for (int k = 0; k < g->n_bats; k++) {
        Bat *b = &g->bats[k];
        b->px    += g->vx * dt_sec;
        b->py    += g->vy * dt_sec;
        b->phase += 1.0f;
        if (b->phase >= (float)WING_CYCLE) b->phase -= (float)WING_CYCLE;
    }

    if (leader_exited(g, cols, rows))
        group_start_pause(g, PAUSE_TICKS);
}

static void group_draw(const Group *g, WINDOW *w, int cols, int rows)
{
    if (g->state == GRP_PAUSE) return;

    for (int k = 0; k < g->n_bats; k++) {
        const Bat *b = &g->bats[k];
        int col = px_to_col(b->px);
        int row = px_to_row(b->py);

        if (row < 0 || row >= rows)          continue;
        if (col - 1 < 0 || col + 1 >= cols) continue;

        int frame = (int)b->phase * 4 / WING_CYCLE;
        if (frame < 0 || frame > 3) frame = 0;

        /* Leader is bold — tip of the V stands out from its followers. */
        attr_t attr = COLOR_PAIR((int)g->color);
        if (k == 0) attr |= A_BOLD;

        wattron(w, attr);
        mvwaddch(w, row, col - 1, (chtype)(unsigned char)k_bat_lw[frame]);
        mvwaddch(w, row, col,     (chtype)BAT_BODY);
        mvwaddch(w, row, col + 1, (chtype)(unsigned char)k_bat_rw[frame]);
        wattroff(w, attr);
    }
    (void)cols;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Group  groups[N_GROUPS];
    int    n_rows;   /* shared formation depth, all groups track this */
    int    cols, rows;
    bool   paused;
} Scene;

static const int     k_init_angle_idx[N_GROUPS] = { 0, 1, 2 };
static const ColorID k_colors[N_GROUPS]         = { COL_G0, COL_G1, COL_G2 };

static void scene_init(Scene *s, int cols, int rows)
{
    s->cols   = cols;
    s->rows   = rows;
    s->paused = false;
    /* Preserve n_rows across resets; first call sets default. */
    if (s->n_rows < ROWS_MIN || s->n_rows > ROWS_MAX)
        s->n_rows = ROWS_DEFAULT;

    float cx = col_to_px(cols / 2);
    float cy = row_to_px(rows / 2);

    for (int i = 0; i < N_GROUPS; i++) {
        Group *g   = &s->groups[i];
        g->color   = k_colors[i];
        g->n_rows  = s->n_rows;
        g->n_bats  = (s->n_rows + 1) * (s->n_rows + 2) / 2;

        g->angle_idx = k_init_angle_idx[i];
        g->angle     = k_angles[g->angle_idx];

        if (i == 0) {
            group_launch(g, cx, cy);
        } else {
            group_start_pause(g, i * STAGGER_TICKS);
            /* Pre-offset angle_idx so the post-pause increment lands on
             * k_init_angle_idx[i] for the very first launch. */
            g->angle_idx = (k_init_angle_idx[i] - 1 + N_ANGLES) % N_ANGLES;
        }
    }
}

/*
 * scene_set_rows — resize all groups by delta (+1 or −1).
 * Clamps to [ROWS_MIN, ROWS_MAX].  Applied immediately: flying groups gain
 * or lose bats without waiting for the next reset.
 */
static void scene_set_rows(Scene *s, int delta)
{
    int new_rows = s->n_rows + delta;
    if (new_rows < ROWS_MIN || new_rows > ROWS_MAX) return;
    s->n_rows = new_rows;
    for (int i = 0; i < N_GROUPS; i++)
        group_set_rows(&s->groups[i], new_rows);
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    float cx = col_to_px(s->cols / 2);
    float cy = row_to_px(s->rows / 2);
    for (int i = 0; i < N_GROUPS; i++)
        group_tick(&s->groups[i], cx, cy, s->cols, s->rows);
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    for (int i = 0; i < N_GROUPS; i++)
        group_draw(&s->groups[i], w, s->cols, s->rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    /* HUD: fps + speed + formation size + key hints */
    int n_bats_total = N_GROUPS * ((sc->n_rows + 1) * (sc->n_rows + 2) / 2);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  spd:%d  rows:%d(%d bats)  +/-  [p] [r]",
             fps, sim_fps, sc->n_rows, n_bats_total);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
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

    /* Formation size — + / = to grow, - to shrink */
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

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    /* Zero-init ensures scene_init sees n_rows=0 and sets the default. */
    memset(&app->scene, 0, sizeof app->scene);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
