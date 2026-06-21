/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_helloworld.c — a two-link arm reaches a target you steer with the
 * arrow keys. Inverse kinematics: you say where the tip should reach,
 * and it works out the joint angles to get there. With two links the
 * answer is one shot of trigonometry (law of cosines), no iteration.
 *
 * Sister files: ik_arm_reach.c / snake_inverse_kinematics.c (FABRIK on
 * longer chains), snake_forward_kinematics.c (the FK contrast).
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra animation/ik_helloworld.c \
 *            -o ik_helloworld -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — tunables and pair IDs ── */

enum {
    TARGET_FPS = 60,

    /* ncurses colour-pair IDs. HUD/HINT follow the project convention. */
    PAIR_HUD          = 1,
    PAIR_HINT         = 2,
    PAIR_LINK_UPPER   = 3,
    PAIR_LINK_FORE    = 4,
    PAIR_JOINT        = 5,
    PAIR_SHOULDER     = 6,
    PAIR_TARGET       = 7,
};

/* Equal link lengths: any target distance in [0, L1+L2] is reachable,
 * with no inner dead-zone. 80 px = 10 cells per link. */
#define L1_PX  80.0f                  /* upper arm length, pixels */
#define L2_PX  80.0f                  /* forearm  length, pixels */

#define KEY_STEP_PX                4.0f   /* target nudge per keypress */
#define INITIAL_TARGET_REACH_FRAC  0.7f   /* start at 70% of full reach */

/* Sub-pixels per cell. The 8:16 (2:1) ratio matches a typical terminal
 * font; geometry stays in square pixel space, converted only at draw. */
#define CELL_W   8
#define CELL_H  16

#define NS_PER_SEC 1000000000LL

/* ── §2 clock — monotonic timer + sleep ── */

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

/* upper arm cyan, forearm orange, elbow white, shoulder lime (anchor),
 * target red (the only thing the user moves). */
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
        init_pair(PAIR_TARGET,      196, -1);
    } else {
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_UPPER,  COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_FORE,   COLOR_RED,     -1);
        init_pair(PAIR_JOINT,       COLOR_WHITE,   -1);
        init_pair(PAIR_SHOULDER,    COLOR_GREEN,   -1);
        init_pair(PAIR_TARGET,      COLOR_RED,     -1);
    }
}

/* ── §4 coords — pixel <-> cell bridge ── */

/* Geometry lives in square pixel space; these convert to/from cells
 * only at draw time, undoing the 8:16 aspect ratio. */
static inline int   px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline float cells_to_px_w(int cols){ return (float)cols * CELL_W; }
static inline float cells_to_px_h(int rows){ return (float)rows * CELL_H; }

/* ── §5 ik — Vec2 and solve_ik(S, T) -> E ── */

/*
 * Vec2 — a 2-D point or vector in pixel space (sub-cell precision, so a
 * moving target glides instead of jumping cell-to-cell). Note y grows
 * DOWNWARD on screen, so "elbow up" is smaller y.
 *   x : pixels, positive points right.
 *   y : pixels, positive points down.
 */
typedef struct {
    float x;   /* pixels, positive -> right */
    float y;   /* pixels, positive -> down  */
} Vec2;

static inline Vec2  vec2(float x, float y)    { return (Vec2){x, y}; }
static inline Vec2  vec2_add(Vec2 a, Vec2 b)  { return (Vec2){a.x+b.x, a.y+b.y}; }
static inline Vec2  vec2_sub(Vec2 a, Vec2 b)  { return (Vec2){a.x-b.x, a.y-b.y}; }
static inline Vec2  vec2_scl(Vec2 a, float s) { return (Vec2){a.x*s, a.y*s}; }
static inline float vec2_len(Vec2 v)          { return sqrtf(v.x*v.x + v.y*v.y); }
static inline float vec2_dist(Vec2 a, Vec2 b) { return vec2_len(vec2_sub(a, b)); }

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * solve_ik — find the elbow E for a fixed shoulder S and target T.
 * S, E, T form a triangle whose three sides are known (L1, L2, and
 * d = |T-S|), so the law of cosines gives the shoulder angle directly.
 * Pure function; T is assumed reachable (caller clamps it). elbow_up
 * picks one of the two valid elbow positions.
 */
static Vec2 solve_ik(Vec2 S, Vec2 T, bool elbow_up)
{
    Vec2  d_vec = vec2_sub(T, S);
    float d     = vec2_len(d_vec);

    /* T on top of S: triangle collapses, cos_a would divide by zero.
     * Point the elbow along +x so the chain stays visible. */
    if (d < 1e-6f) return vec2(S.x + L1_PX, S.y);

    float cos_a       = (L1_PX*L1_PX + d*d - L2_PX*L2_PX) / (2.0f * L1_PX * d);
    float alpha       = acosf(clampf(cos_a, -1.0f, 1.0f));  /* clamp: fp drift can push just past +-1 -> NaN */
    float phi         = atan2f(d_vec.y, d_vec.x);   /* direction S -> T */
    float side        = elbow_up ? -1.0f : +1.0f;   /* which side of S->T the elbow sits */
    float elbow_angle = phi + side * alpha;

    return vec2(S.x + L1_PX * cosf(elbow_angle),
                S.y + L1_PX * sinf(elbow_angle));
}

/* ── §6 scene — state, input, draw ── */

/*
 * Scene — everything the demo tracks. The data flows one way each frame:
 * inputs (target, elbow_up) -> solve_ik -> output (elbow) -> render.
 * (fk_helloworld.c is the mirror image: the user sets the angles and the
 * tip falls out.)
 */
typedef struct {
    Vec2 shoulder;       /* S, the fixed anchor; re-centred on resize    */
    Vec2 target;         /* T, what you move; kept inside the reach disc */
    Vec2 elbow;          /* E, solver output; recomputed each frame      */
    bool elbow_up;       /* picks one of the two valid elbow positions   */
    bool paused;         /* freeze: input ignored, last frame held       */
    int  rows, cols;     /* terminal size in character cells             */
} Scene;

static Vec2 clamp_to_screen(Vec2 p, int rows, int cols)
{
    return vec2(clampf(p.x, 0.0f, cells_to_px_w(cols) - 1.0f),
                clampf(p.y, 0.0f, cells_to_px_h(rows) - 1.0f));
}

/*
 * clamp_to_reach — pull T radially back onto the reach disc (radius
 * L1+L2) if it has drifted outside, so the triangle is always solvable
 * and the forearm length never appears to stretch.
 */
static Vec2 clamp_to_reach(Vec2 t, Vec2 shoulder, float reach)
{
    Vec2  d   = vec2_sub(t, shoulder);
    float len = vec2_len(d);
    if (len <= reach) return t;
    return vec2_add(shoulder, vec2_scl(d, reach / len));
}

/*
 * scene_init — centre S, place T at 70% reach, default elbow-down, and
 * solve once so elbow is valid on the first frame.
 */
static void scene_init(Scene *s, int rows, int cols)
{
    s->rows     = rows;
    s->cols     = cols;
    s->paused   = false;
    s->elbow_up = false;

    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);

    float reach = L1_PX + L2_PX;
    s->target   = vec2(s->shoulder.x + reach * INITIAL_TARGET_REACH_FRAC,
                       s->shoulder.y);

    s->elbow = solve_ik(s->shoulder, s->target, s->elbow_up);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows     = rows;
    s->cols     = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);
    s->target   = clamp_to_screen(s->target, rows, cols);
    s->target   = clamp_to_reach (s->target, s->shoulder, L1_PX + L2_PX);
    s->elbow    = solve_ik(s->shoulder, s->target, s->elbow_up);
}

/*
 * scene_input — one keypress: arrows nudge T (ignored when paused),
 * 'f' flips elbow side, space pauses, 'r' resets. No key moves E; the
 * elbow is solver output, not input.
 */
static void scene_input(Scene *s, int ch)
{
    const float k = KEY_STEP_PX;

    if (!s->paused) {
        switch (ch) {
            case KEY_LEFT:  s->target.x -= k; break;
            case KEY_RIGHT: s->target.x += k; break;
            case KEY_UP:    s->target.y -= k; break;
            case KEY_DOWN:  s->target.y += k; break;
            default: break;
        }
        s->target = clamp_to_screen(s->target, s->rows, s->cols);
        s->target = clamp_to_reach (s->target, s->shoulder, L1_PX + L2_PX);
    }

    switch (ch) {
        case 'f': s->elbow_up = !s->elbow_up; break;
        case ' ': s->paused   = !s->paused;   break;
        case 'r': scene_init(s, s->rows, s->cols); break;
        default: break;
    }
}

/* rendering */

static inline bool in_screen(int row, int col, int rows, int cols)
{
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

/*
 * draw_line_px — walk a to b in pixel space at half-cell steps along the
 * longer axis, so no cell on the path is skipped at any angle.
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
 * scene_draw — two links, then three markers. '@' is drawn last so the
 * shoulder always wins its cell when markers overlap.
 */
static void scene_draw(const Scene *s)
{
    Vec2 S = s->shoulder, E = s->elbow, T = s->target;

    draw_line (S, E, PAIR_LINK_UPPER, s->rows, s->cols);
    draw_line (E, T, PAIR_LINK_FORE,  s->rows, s->cols);

    draw_point(T, '*', PAIR_TARGET,   s->rows, s->cols);
    draw_point(E, 'O', PAIR_JOINT,    s->rows, s->cols);
    draw_point(S, '@', PAIR_SHOULDER, s->rows, s->cols);
}

/* ── §7 screen — ncurses init / present / HUD ── */

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
 * screen_hud — status on row 0, params on row 1, key hints on the last
 * row. The |E-T| readout should stay pinned at L2; if it drifts, the
 * solver has a bug.
 */
static void screen_hud(const Scene *s, float fps)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    char  buf[96];
    float fore_len = vec2_dist(s->elbow, s->target);

    snprintf(buf, sizeof buf, " %5.1f fps  closed-form IK  %s ",
             fps, s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    snprintf(buf, sizeof buf,
             " L1:%.0f  L2:%.0f  reach:%.0fpx  |E-T|:%5.1fpx  elbow:%s ",
             L1_PX, L2_PX, L1_PX + L2_PX, fore_len,
             s->elbow_up ? "up  " : "down");
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  arrows:move target  f:flip elbow ");
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

        /* drain input -> moves the target */
        for (int ch; (ch = getch()) != ERR; ) {
            if (ch == 'q' || ch == 27 /*ESC*/) { g_running = 0; break; }
            scene_input(&scene, ch);
        }

        /* solve for the elbow, then draw */
        scene.elbow = solve_ik(scene.shoulder, scene.target, scene.elbow_up);

        erase();
        scene_draw(&scene);
        screen_hud(&scene, fps);
        screen_present();

        int64_t elapsed = clock_ns() - t_now;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    return 0;
}
