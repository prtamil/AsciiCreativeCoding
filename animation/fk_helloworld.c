/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_helloworld.c — a two-link arm you steer by joint angles (forward kinematics).
 *
 * You set the shoulder and elbow angles with the arrow keys; the elbow and
 * hand positions just follow from those angles. Forward kinematics means
 * angles in, positions out — the opposite of ik_helloworld.c, which takes a
 * hand position and solves back for the angles.
 *
 * Sisters: ik_helloworld.c (same arm, reversed), snake_forward_kinematics.c
 * and snake_inverse_kinematics.c (the same idea on a long chain).
 *
 * Keys: q/ESC quit  space pause  r reset  L/R shoulder angle  U/D elbow angle.
 * Build: gcc -std=c11 -O2 -Wall -Wextra animation/fk_helloworld.c \
 *            -o fk_helloworld -lncurses -lm   (trig, so -lm is required)
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — every tunable in one place ── */

enum {
    TARGET_FPS = 60,

    /* ncurses colour-pair slots. HUD and HINT use the shared
     * project look (bright yellow / bright cyan); the rest colour
     * the two arm links and the three joint markers. */
    PAIR_HUD          = 1,
    PAIR_HINT         = 2,
    PAIR_LINK_UPPER   = 3,
    PAIR_LINK_FORE    = 4,
    PAIR_JOINT        = 5,
    PAIR_SHOULDER     = 6,
    PAIR_HAND         = 7,
};

/* Both links the same length, so when the elbow folds all the way
 * back (elbow angle = 180°) the hand lands right on the shoulder. */
#define L1_PX  80.0f                  /* upper arm length, pixels */
#define L2_PX  80.0f                  /* forearm  length, pixels */

#define DEG_TO_RAD      ((float)M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / (float)M_PI)

/* How far each arrow keypress nudges a joint angle. 2° feels
 * smooth when you hold the key and still lets you stop on a target. */
#define ANGLE_STEP_RAD  (2.0f * DEG_TO_RAD)

/* A terminal cell is taller than it is wide, so we treat it as
 * 8 sub-pixels across and 16 tall. The arm math runs in these square
 * pixels and only converts to cells when drawing (see §4). */
#define CELL_W   8
#define CELL_H  16

#define NS_PER_SEC 1000000000LL

/* ── §2 clock — monotonic timer and sleep ── */

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

/* ── §3 color — one colour per visual role ── */

/*
 * Each part of the arm gets its own colour: cyan upper arm, orange
 * forearm, white elbow, lime shoulder (the fixed anchor), red hand.
 * Falls back to the 8 basic colours on older terminals.
 */
static void color_init(void)
{
    if (!has_colors()) return;
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,         226, -1);
        init_pair(PAIR_HINT,         51, -1);
        init_pair(PAIR_LINK_UPPER,   45, -1);
        init_pair(PAIR_LINK_FORE,   208, -1);
        init_pair(PAIR_JOINT,       255, -1);
        init_pair(PAIR_SHOULDER,    118, -1);
        init_pair(PAIR_HAND,        196, -1);
    } else {
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_UPPER,  COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_FORE,   COLOR_RED,     -1);
        init_pair(PAIR_JOINT,       COLOR_WHITE,   -1);
        init_pair(PAIR_SHOULDER,    COLOR_GREEN,   -1);
        init_pair(PAIR_HAND,        COLOR_RED,     -1);
    }
}

/* ── §4 coords — convert between pixels and cells ── */

/*
 * The arm math uses square pixels so angles and lengths behave
 * normally. These helpers turn a pixel position into the row/column
 * of a terminal cell, and vice versa. They are the only place the
 * pixel-to-cell conversion happens.
 */
static inline int   px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline float cells_to_px_w(int cols){ return (float)cols * CELL_W; }
static inline float cells_to_px_h(int rows){ return (float)rows * CELL_H; }

/* ── §5 fk — the forward-kinematics math ── */

/*
 * Vec2 — a point in pixel space. The whole arm lives in these
 * coordinates; positions get rounded to terminal cells only at
 * draw time. x grows rightward, y grows downward (screen-style, so
 * y is flipped from the usual math axis — only the HUD cares).
 */
typedef struct {
    float x;   /* rightward pixel coordinate */
    float y;   /* downward  pixel coordinate */
} Vec2;

static inline Vec2  vec2(float x, float y)    { return (Vec2){x, y}; }
static inline Vec2  vec2_sub(Vec2 a, Vec2 b)  { return (Vec2){a.x-b.x, a.y-b.y}; }
static inline float vec2_len(Vec2 v)          { return sqrtf(v.x*v.x + v.y*v.y); }
static inline float vec2_dist(Vec2 a, Vec2 b) { return vec2_len(vec2_sub(a, b)); }

/*
 * ArmPose — the two positions that compute_fk works out from the
 * angles. Bundling them lets the caller grab both with one
 * assignment. Only the computed positions live here; the angles
 * that produced them stay in Scene.
 */
typedef struct {
    Vec2 elbow;   /* where the upper arm ends */
    Vec2 hand;    /* where the forearm ends   */
} ArmPose;

/*
 * compute_fk — the heart of forward kinematics. You give it the
 * shoulder position and the two joint angles; it follows the chain
 * down and reports where the elbow and hand end up. Set the angles,
 * the positions follow.
 *
 * Walk out from the shoulder by L1 in the direction of theta1 to
 * reach the elbow, then out from the elbow by L2 to reach the hand.
 * theta2 is measured relative to the upper arm, so the forearm's
 * actual heading is theta1 + theta2.
 */
static ArmPose compute_fk(Vec2 S, float theta1, float theta2)
{
    Vec2 E = vec2(S.x + L1_PX * cosf(theta1),
                  S.y + L1_PX * sinf(theta1));
    Vec2 H = vec2(E.x + L2_PX * cosf(theta1 + theta2),
                  E.y + L2_PX * sinf(theta1 + theta2));
    return (ArmPose){E, H};
}

/* ── §6 scene — state, input, drawing ── */

/*
 * Scene — everything the demo tracks. The two angles are the only
 * real input (you change them with the arrow keys); the elbow and
 * hand are recomputed from them every frame and cached here so the
 * drawing code and HUD can read positions without redoing the math.
 * (ik_helloworld.c flips this: there the hand is the input and the
 * angles are computed.)
 */
typedef struct {
    Vec2  shoulder;     /* fixed anchor; re-centred on terminal resize */
    float theta1;       /* shoulder angle, radians, measured from +X   */
    float theta2;       /* elbow angle, radians, relative to upper arm */
    Vec2  elbow;        /* computed each frame by compute_fk           */
    Vec2  hand;         /* computed each frame by compute_fk           */

    bool  paused;       /* freezes angle input; drawing still runs     */
    int   rows, cols;   /* terminal size in cells; clips the drawing   */
} Scene;

/*
 * wrap_pi — keep an angle in the range -180°..180°. Purely cosmetic:
 * stops the HUD readout from climbing to huge numbers when you hold
 * a key; the trig would work fine without it.
 */
static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/*
 * scene_init — start fresh: shoulder at screen centre, both angles
 * zero (arm horizontal, pointing right), and one compute_fk so the
 * cached elbow and hand are valid before the first frame draws.
 */
static void scene_init(Scene *s, int rows, int cols)
{
    s->rows   = rows;
    s->cols   = cols;
    s->paused = false;
    s->theta1 = 0.0f;
    s->theta2 = 0.0f;

    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);

    ArmPose pose = compute_fk(s->shoulder, s->theta1, s->theta2);
    s->elbow = pose.elbow;
    s->hand  = pose.hand;
}

/* Re-centre the shoulder for a new terminal size; angles are kept,
 * so the arm just recomputes from the same pose. */
static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);

    ArmPose pose = compute_fk(s->shoulder, s->theta1, s->theta2);
    s->elbow = pose.elbow;
    s->hand  = pose.hand;
}

/*
 * scene_input — act on one keypress. Left/right turn the shoulder,
 * up/down bend the elbow, space pauses, r resets. There is no key
 * to move the elbow or hand directly: they always follow from the
 * angles.
 */
static void scene_input(Scene *s, int ch)
{
    const float k = ANGLE_STEP_RAD;

    if (!s->paused) {
        switch (ch) {
            case KEY_LEFT:  s->theta1 -= k; break;
            case KEY_RIGHT: s->theta1 += k; break;
            case KEY_UP:    s->theta2 -= k; break;
            case KEY_DOWN:  s->theta2 += k; break;
            default: break;
        }
        s->theta1 = wrap_pi(s->theta1);
        s->theta2 = wrap_pi(s->theta2);
    }

    switch (ch) {
        case ' ': s->paused = !s->paused; break;
        case 'r': scene_init(s, s->rows, s->cols); break;
        default: break;
    }
}

/* ── rendering ── */

static inline bool in_screen(int row, int col, int rows, int cols)
{
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

/*
 * draw_line_px — draw a straight line between two pixel points by
 * stepping along it in small increments (about half a cell each) so
 * no cell on the path gets skipped, whatever the angle.
 */
static void draw_line_px(Vec2 a, Vec2 b, chtype glyph, int rows, int cols)
{
    float dx = b.x - a.x, dy = b.y - a.y;
    float steps_f = fmaxf(fabsf(dx) / (CELL_W * 0.5f),
                          fabsf(dy) / (CELL_H * 0.5f));
    int   steps   = (int)steps_f + 1;
    for (int i = 0; i <= steps; i++) {
        float t   = (float)i / (float)steps;
        float px  = a.x + t * dx;
        float py  = a.y + t * dy;
        int   col = px_to_cell_x(px);
        int   row = px_to_cell_y(py);
        if (in_screen(row, col, rows, cols))
            mvaddch(row, col, glyph);
    }
}

static void draw_line(Vec2 a, Vec2 b, int color_pair, int rows, int cols)
{
    chtype glyph = (chtype)((unsigned char)'#') | COLOR_PAIR(color_pair);
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    draw_line_px(a, b, glyph, rows, cols);
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

static void draw_point(Vec2 p, char glyph, int color_pair, int rows, int cols)
{
    int row = px_to_cell_y(p.y);
    int col = px_to_cell_x(p.x);
    if (!in_screen(row, col, rows, cols)) return;
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    mvaddch(row, col, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

/*
 * scene_draw — draw the two arm links, then the three joint markers
 * on top. Order matters: the shoulder '@' is drawn last so it always
 * shows through when markers land in the same cell.
 */
static void scene_draw(const Scene *s)
{
    Vec2 S = s->shoulder, E = s->elbow, H = s->hand;

    draw_line (S, E, PAIR_LINK_UPPER, s->rows, s->cols);
    draw_line (E, H, PAIR_LINK_FORE,  s->rows, s->cols);

    draw_point(H, '*', PAIR_HAND,     s->rows, s->cols);
    draw_point(E, 'O', PAIR_JOINT,    s->rows, s->cols);
    draw_point(S, '@', PAIR_SHOULDER, s->rows, s->cols);
}

/* ── §7 screen — ncurses setup, present, HUD ── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    color_init();
}

static void screen_cleanup(void)
{
    if (!isendwin()) endwin();
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * screen_hud — the on-screen readouts: fps and pause state on the
 * top row, the live angles and the hand's distance from the shoulder
 * on the next, and the key hints along the bottom.
 */
static void screen_hud(const Scene *s, float fps)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    char  buf[112];
    float deg1  = s->theta1 * RAD_TO_DEG;
    float deg2  = s->theta2 * RAD_TO_DEG;
    float reach = vec2_dist(s->shoulder, s->hand);

    snprintf(buf, sizeof buf, " %5.1f fps  forward kinematics  %s ",
             fps, s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    snprintf(buf, sizeof buf,
             " L1:%.0f  L2:%.0f  theta1:%+7.1f°  theta2:%+7.1f°  |H-S|:%5.1fpx ",
             L1_PX, L2_PX, deg1, deg2, reach);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  L/R:rotate theta1  U/D:rotate theta2 ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §8 app — signals, resize, main loop ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running     = 0;
}

static void install_signals(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGINT,   &sa, NULL);
    sigaction(SIGTERM,  &sa, NULL);
    sigaction(SIGWINCH, &sa, NULL);
}

int main(void)
{
    install_signals();
    atexit(screen_cleanup);
    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene scene;
    scene_init(&scene, rows, cols);

    int64_t       t_prev       = clock_ns();
    int64_t       fps_accum_ns = 0;
    int           fps_frames   = 0;
    float         fps          = 0.0f;
    const int64_t TICK_NS      = NS_PER_SEC / TARGET_FPS;

    while (g_running) {
        int64_t t_now = clock_ns();
        fps_accum_ns += t_now - t_prev;
        t_prev = t_now;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC) {
            fps          = (float)fps_frames * 1e9f / (float)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        if (g_need_resize) {
            g_need_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&scene, rows, cols);
        }

        /* (1) read keys: the user steers angles, not a target */
        for (int ch; (ch = getch()) != ERR; ) {
            if (ch == 'q' || ch == 27 /*ESC*/) { g_running = 0; break; }
            scene_input(&scene, ch);
        }

        /* (2) angles changed, so recompute the elbow and hand */
        ArmPose pose = compute_fk(scene.shoulder, scene.theta1, scene.theta2);
        scene.elbow  = pose.elbow;
        scene.hand   = pose.hand;

        /* (3) redraw the frame */
        erase();
        scene_draw(&scene);
        screen_hud(&scene, fps);
        screen_present();

        int64_t elapsed = clock_ns() - t_now;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    return 0;
}
