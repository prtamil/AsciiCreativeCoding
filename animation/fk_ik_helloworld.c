/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_ik_helloworld.c — a 3-link arm shown two ways, toggled with 'm'.
 *   FK: set the joint angles and the limb follows down the chain.
 *   IK: give a target and the solver finds angles that reach it (here a
 *       2-link law-of-cosines kernel + the hand link aimed at the target).
 * Sisters: ik_helloworld.c (2-link IK), fk_helloworld.c (2-link FK),
 *          ik_arm_reach.c (FABRIK on a 4-link arm).
 * Build: gcc -std=c11 -O2 -Wall -Wextra animation/fk_ik_helloworld.c \
 *            -o fk_ik_helloworld -lncurses -lm
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

/* ── §1 config — tunables, color pair IDs, cell geometry ── */

enum {
    TARGET_FPS = 60,

    /* ncurses color-pair IDs. HUD/HINT follow the project convention
     * (bright yellow / bright cyan, both A_BOLD). */
    PAIR_HUD          = 1,
    PAIR_HINT         = 2,
    PAIR_LINK_UPPER   = 3,
    PAIR_LINK_FORE    = 4,
    PAIR_LINK_HAND    = 5,
    PAIR_JOINT        = 6,
    PAIR_SHOULDER     = 7,
    PAIR_HAND         = 8,
    PAIR_TARGET       = 9,
};

/* Three equal-length links, so max reach = 3*L = 240 px = 30 cells.
 * Equal lengths keep the edge cases simple: any distance in [0, 3*L]
 * is reachable. */
#define L1_PX  80.0f                  /* upper arm length */
#define L2_PX  80.0f                  /* forearm  length */
#define L3_PX  80.0f                  /* hand link length */

#define DEG_TO_RAD      ((float)M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / (float)M_PI)

#define ANGLE_STEP_RAD  (2.0f * DEG_TO_RAD)   /* 2° per FK keypress */
#define KEY_STEP_PX     4.0f                  /* 4 px per IK arrow key (half a cell wide) */

/* Initial IK target sits at 70% of full reach, on the rest line. */
#define INITIAL_TARGET_REACH_FRAC  0.7f

/* Sub-pixels per character cell. Math runs in square pixel space;
 * conversion to cells happens only at draw time. */
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

/* ── §3 color — one fixed color per visual role ── */

/* upper arm cyan, forearm orange, hand link magenta, joints white,
 * shoulder lime, hand red, target gold (IK only). */
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
        init_pair(PAIR_LINK_HAND,   201, -1);
        init_pair(PAIR_JOINT,       255, -1);
        init_pair(PAIR_SHOULDER,    118, -1);
        init_pair(PAIR_HAND,        196, -1);
        init_pair(PAIR_TARGET,      220, -1);
    } else {
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_UPPER,  COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_FORE,   COLOR_RED,     -1);
        init_pair(PAIR_LINK_HAND,   COLOR_MAGENTA, -1);
        init_pair(PAIR_JOINT,       COLOR_WHITE,   -1);
        init_pair(PAIR_SHOULDER,    COLOR_GREEN,   -1);
        init_pair(PAIR_HAND,        COLOR_RED,     -1);
        init_pair(PAIR_TARGET,      COLOR_YELLOW,  -1);
    }
}

/* ── §4 coords — pixel<->cell aspect-ratio bridge ── */

/* Positions live in square pixel space; these convert to cells only at
 * draw time, undoing the 8:16 cell shape. Geometry done in pixel space
 * keeps right angles actually right (they would skew in cell space). */
static inline int   px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline float cells_to_px_w(int cols){ return (float)cols * CELL_W; }
static inline float cells_to_px_h(int rows){ return (float)rows * CELL_H; }

/* ── §5 ik_fk — Vec2, FK, and the 2+1 IK solver ── */

/*
 * Vec2 — a 2-D point in pixel space (sub-cell precision).
 *   x: rightward, positive -> right of screen.
 *   y: downward,  positive -> down the screen.
 * Angles use math convention (positive CCW about +X); since y points
 * down, a CCW rotation in the math shows as CW on screen. Only the HUD
 * readout has to care. Passed by value: 8 bytes, fits in registers.
 * Named fields turn an (x,y) vs (col,row) mixup into a compile error.
 */
typedef struct {
    float x;   /* rightward pixel coordinate */
    float y;   /* downward  pixel coordinate */
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

/* Unit vector, returning fallback when v is ~zero-length. Guards the IK
 * solvers against dividing by a length that can collapse to zero. */
static inline Vec2 vec2_normalize_or(Vec2 v, Vec2 fallback)
{
    float len = vec2_len(v);
    if (len < 1e-6f) return fallback;
    return vec2_scl(v, 1.0f / len);
}

/* Fold an angle into (-pi, pi]. Cosmetic: keeps the HUD readout from
 * drifting under a held key. */
static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/*
 * ArmPose — the three joint positions one arm evaluation produces.
 * Both solvers return this same shape, so the main loop picks a solver
 * by Mode and writes the result into one cache regardless of which ran.
 * That is the point of the file: FK and IK produce the same record of
 * joint positions from opposite inputs (angles vs target).
 */
typedef struct {
    Vec2 elbow;   /* E — end of upper arm, L1 from shoulder */
    Vec2 wrist;   /* W — end of forearm,   L2 from elbow    */
    Vec2 hand;    /* H — end-effector,     L3 from wrist    */
} ArmPose;

/*
 * Forward kinematics: set the angles, walk down the chain. Each link
 * inherits the running sum of the angles before it (upper arm th1,
 * forearm th1+th2, hand th1+th2+th3); step L along that direction from
 * the previous joint to reach the next. Pure, no branches.
 */
static ArmPose compute_fk(Vec2 S, float th1, float th2, float th3)
{
    float a1 = th1;
    float a2 = th1 + th2;
    float a3 = th1 + th2 + th3;

    Vec2 E = vec2(S.x + L1_PX * cosf(a1), S.y + L1_PX * sinf(a1));
    Vec2 W = vec2(E.x + L2_PX * cosf(a2), E.y + L2_PX * sinf(a2));
    Vec2 H = vec2(W.x + L3_PX * cosf(a3), W.y + L3_PX * sinf(a3));

    return (ArmPose){E, W, H};
}

/*
 * TwoLinkPose — what the 2-link IK kernel returns: the elbow and the
 * tip of the second link. Called "tip" not "wrist" because the 2-link
 * sub-problem has no third joint; solve_ik3 later treats this tip as
 * the wrist of the full arm. Separate from ArmPose so the kernel never
 * has to invent a fake third joint.
 */
typedef struct {
    Vec2 elbow;   /* E — law-of-cosines solution at the shoulder vertex */
    Vec2 tip;     /* W — end of link 2; equals target T when reachable  */
} TwoLinkPose;

/*
 * Inverse kinematics for two links: given a target T, solve the elbow
 * angle directly with the law of cosines (no iteration). The tip lands
 * on T whenever T is reachable. solve_ik3 uses this for the first two
 * links, then aims the third link at the real target.
 */
static TwoLinkPose solve_ik2(Vec2 S, Vec2 T, bool elbow_up,
                             float L1, float L2)
{
    Vec2  d_vec = vec2_sub(T, S);
    float d     = vec2_len(d_vec);

    /* Degenerate: target on top of shoulder. */
    if (d < 1e-6f)
        return (TwoLinkPose){ vec2(S.x + L1, S.y), S };

    /* Law of cosines at the shoulder vertex of triangle SET. */
    float cos_a       = (L1*L1 + d*d - L2*L2) / (2.0f * L1 * d);
    float alpha       = acosf(clampf(cos_a, -1.0f, 1.0f));
    float phi         = atan2f(d_vec.y, d_vec.x);
    float side        = elbow_up ? -1.0f : +1.0f;
    float elbow_angle = phi + side * alpha;

    Vec2 E = vec2(S.x + L1 * cosf(elbow_angle),
                  S.y + L1 * sinf(elbow_angle));

    /* Tip = E + L2 toward T. Lands on T exactly when reachable. */
    Vec2 dir = vec2_normalize_or(vec2_sub(T, E), vec2(1.0f, 0.0f));
    Vec2 W   = vec2_add(E, vec2_scl(dir, L2));

    return (TwoLinkPose){E, W};
}

/*
 * Inverse kinematics for three links via a 2+1 trick. A full 3-link
 * solver is underdetermined (many configs reach any target), so we
 * pin the last link's direction to collapse the ambiguity:
 *   1. pull T back along S->T by L3 to get a virtual 2-link target T2;
 *   2. solve 2-link IK from S to T2 to place elbow and wrist;
 *   3. aim the hand link from the wrist at the real target T.
 * Pure. Returns all three joint positions.
 */
static ArmPose solve_ik3(Vec2 S, Vec2 T, bool elbow_up)
{
    /* Step 1 — virtual 2-link target T2 = T − L3 · dir(S → T). */
    Vec2 dir_st = vec2_normalize_or(vec2_sub(T, S), vec2(1.0f, 0.0f));
    Vec2 T2     = vec2_sub(T, vec2_scl(dir_st, L3_PX));

    /* Step 2 — 2-link IK places E and the wrist W (= T2 if reachable). */
    TwoLinkPose tl = solve_ik2(S, T2, elbow_up, L1_PX, L2_PX);

    /* Step 3 — aim the hand link from W toward the real target. */
    Vec2 dir_wt = vec2_normalize_or(vec2_sub(T, tl.tip), vec2(1.0f, 0.0f));
    Vec2 H      = vec2_add(tl.tip, vec2_scl(dir_wt, L3_PX));

    return (ArmPose){tl.elbow, tl.tip, H};
}

/* ── §6 scene — state, input dispatch, drawing ── */

/*
 * Mode — which solver runs this frame, toggled by 'm'.
 *   MODE_FK: user edits angles, positions fall out (compute_fk).
 *   MODE_IK: user edits the target, angles fall out (solve_ik3).
 * The keymap and HUD also branch on it (scene_input, screen_hud).
 */
typedef enum { MODE_FK, MODE_IK } Mode;

/*
 * Scene — all state of the demo. It holds the inputs for BOTH solvers
 * at once plus one shared output cache (elbow/wrist/hand). Mode picks
 * which inputs the current frame uses; the other set is kept so
 * toggling back is instant. The renderer only ever reads the cache, so
 * it never has to know which solver ran:
 *   FK mode: theta1/2/3 -> compute_fk -> elbow,wrist,hand
 *   IK mode: target     -> solve_ik3  -> elbow,wrist,hand
 */
typedef struct {
    /* Anchor */
    Vec2  shoulder;     /* S — pinned origin; recentred on resize        */

    /* FK inputs (a/d, w/s, z/x edit these; active in MODE_FK) */
    float theta1;       /* shoulder angle (rad), absolute from +X        */
    float theta2;       /* elbow    angle (rad), relative to upper arm   */
    float theta3;       /* wrist    angle (rad), relative to forearm     */

    /* IK inputs (arrow keys / 'f' edit these; active in MODE_IK) */
    Vec2  target;       /* T — drawn as '+'; clamped to screen and reach */
    bool  elbow_up;     /* picks 1 of the 2 valid elbow solutions        */

    /* Cached solver outputs (written every frame, read by draw + HUD) */
    Vec2  elbow;        /* E */
    Vec2  wrist;        /* W */
    Vec2  hand;         /* H — also the IK end-effector */

    /* Render / control */
    Mode  mode;         /* MODE_FK or MODE_IK; also gates the '+' glyph   */
    bool  paused;       /* freezes the solver; render unchanged          */
    int   rows, cols;   /* terminal extent in character cells            */
} Scene;

static Vec2 clamp_to_screen(Vec2 p, int rows, int cols)
{
    return vec2(clampf(p.x, 0.0f, cells_to_px_w(cols) - 1.0f),
                clampf(p.y, 0.0f, cells_to_px_h(rows) - 1.0f));
}

static Vec2 clamp_to_reach(Vec2 t, Vec2 shoulder, float reach)
{
    Vec2  d   = vec2_sub(t, shoulder);
    float len = vec2_len(d);
    if (len <= reach) return t;
    return vec2_add(shoulder, vec2_scl(d, reach / len));
}

/* Run the mode's solver and store its joint positions in the cache. */
static void scene_recompute(Scene *s)
{
    ArmPose p = (s->mode == MODE_FK)
        ? compute_fk(s->shoulder, s->theta1, s->theta2, s->theta3)
        : solve_ik3 (s->shoulder, s->target, s->elbow_up);
    s->elbow = p.elbow;
    s->wrist = p.wrist;
    s->hand  = p.hand;
}

/* Restore default angles, target, and elbow side. Mode is left alone
 * on purpose: 'r' should not yank you out of the mode you're in. */
static void scene_reset(Scene *s)
{
    s->theta1   = 0.0f;
    s->theta2   = 0.0f;
    s->theta3   = 0.0f;
    s->elbow_up = false;
    s->paused   = false;

    float reach = L1_PX + L2_PX + L3_PX;
    s->target   = vec2(s->shoulder.x + reach * INITIAL_TARGET_REACH_FRAC,
                       s->shoulder.y);

    scene_recompute(s);
}

/* Anchor the shoulder at screen centre, start in FK mode, reset rest. */
static void scene_init(Scene *s, int rows, int cols)
{
    s->rows     = rows;
    s->cols     = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);
    s->mode     = MODE_FK;
    scene_reset(s);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);
    s->target   = clamp_to_screen(s->target, rows, cols);
    s->target   = clamp_to_reach (s->target, s->shoulder,
                                  L1_PX + L2_PX + L3_PX);
    scene_recompute(s);
}

/* Handle one keypress. space/r/m/f work even when paused; the
 * value-changing keys are ignored while paused. FK uses a/d, w/s, z/x
 * (a left-hand row stack); IK uses the arrow cluster. */
static void scene_input(Scene *s, int ch)
{
    switch (ch) {
        case ' ': s->paused   = !s->paused;        return;
        case 'r': scene_reset(s);                  return;
        case 'm': s->mode     = (s->mode == MODE_FK) ? MODE_IK : MODE_FK; return;
        case 'f': s->elbow_up = !s->elbow_up;      return;
        default: break;
    }

    if (s->paused) return;

    if (s->mode == MODE_FK) {
        const float a = ANGLE_STEP_RAD;
        switch (ch) {
            case 'a': s->theta1 -= a; break;
            case 'd': s->theta1 += a; break;
            case 'w': s->theta2 -= a; break;
            case 's': s->theta2 += a; break;
            case 'z': s->theta3 -= a; break;
            case 'x': s->theta3 += a; break;
            default: break;
        }
        s->theta1 = wrap_pi(s->theta1);
        s->theta2 = wrap_pi(s->theta2);
        s->theta3 = wrap_pi(s->theta3);
    } else {
        const float k = KEY_STEP_PX;
        switch (ch) {
            case KEY_LEFT:  s->target.x -= k; break;
            case KEY_RIGHT: s->target.x += k; break;
            case KEY_UP:    s->target.y -= k; break;
            case KEY_DOWN:  s->target.y += k; break;
            default: break;
        }
        s->target = clamp_to_screen(s->target, s->rows, s->cols);
        s->target = clamp_to_reach (s->target, s->shoulder,
                                    L1_PX + L2_PX + L3_PX);
    }
}

/* ── rendering ── */

static inline bool in_screen(int row, int col, int rows, int cols)
{
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

/* Draw a line a->b in pixel space, stepping at half-cell spacing along
 * the longer axis so no cell on the path is skipped at any angle. */
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

/* Three link lines, then markers on top. '@' shoulder is painted last
 * so the pinned anchor always wins when cells overlap. */
static void scene_draw(const Scene *s)
{
    Vec2 S = s->shoulder, E = s->elbow, W = s->wrist, H = s->hand;

    draw_line(S, E, PAIR_LINK_UPPER, s->rows, s->cols);
    draw_line(E, W, PAIR_LINK_FORE,  s->rows, s->cols);
    draw_line(W, H, PAIR_LINK_HAND,  s->rows, s->cols);

    /* Target before joints, so a joint marker hides any overlap (no
     * flicker when the hand sits on the target). */
    if (s->mode == MODE_IK)
        draw_point(s->target, '+', PAIR_TARGET, s->rows, s->cols);

    /* hand, wrist, elbow, shoulder — far joint first, anchor last. */
    draw_point(H, '*', PAIR_HAND,     s->rows, s->cols);
    draw_point(W, 'o', PAIR_JOINT,    s->rows, s->cols);
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
    typeahead(-1);
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

/* HUD: status on row 0, params on row 1, key hint on the last row. Row 1
 * and the hint are mode-aware (FK shows the three angles; IK shows
 * distances and elbow side). */
static void screen_hud(const Scene *s, float fps)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    char buf[128];

    /* Row 0 — mode + paused state + fps. */
    snprintf(buf, sizeof buf, " %5.1f fps  mode:%s  %s ",
             fps, s->mode == MODE_FK ? "FK" : "IK",
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — mode-specific parameters. */
    if (s->mode == MODE_FK) {
        snprintf(buf, sizeof buf,
                 " L:%.0f,%.0f,%.0f  th1:%+7.1f°  th2:%+7.1f°  th3:%+7.1f° ",
                 L1_PX, L2_PX, L3_PX,
                 s->theta1 * RAD_TO_DEG,
                 s->theta2 * RAD_TO_DEG,
                 s->theta3 * RAD_TO_DEG);
    } else {
        float reach = L1_PX + L2_PX + L3_PX;
        float h_err = vec2_dist(s->hand, s->target);
        snprintf(buf, sizeof buf,
                 " reach:%.0fpx  |T-S|:%5.1f  |H-T|:%5.1f  elbow:%s ",
                 reach,
                 vec2_dist(s->target, s->shoulder),
                 h_err,
                 s->elbow_up ? "up  " : "down");
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint — keys differ by mode. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    if (s->mode == MODE_FK)
        mvprintw(rows - 1, 0,
                 " q:quit  spc:pause  r:reset  m:mode  a/d:th1  w/s:th2  z/x:th3 ");
    else
        mvprintw(rows - 1, 0,
                 " q:quit  spc:pause  r:reset  m:mode  arrows:target  f:flip elbow ");
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

        /* (1) input — angles in FK mode, target in IK mode */
        for (int ch; (ch = getch()) != ERR; ) {
            if (ch == 'q' || ch == 27 /*ESC*/) { g_running = 0; break; }
            scene_input(&scene, ch);
        }

        /* (2) run the right solver and cache its output positions */
        scene_recompute(&scene);

        /* (3) clear / draw / present */
        erase();
        scene_draw(&scene);
        screen_hud(&scene, fps);
        screen_present();

        int64_t elapsed = clock_ns() - t_now;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    return 0;
}
