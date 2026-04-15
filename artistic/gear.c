/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * gear.c — wireframe rotating gear with themed spark effects
 *
 * 10 spark/gear themes cycled with 't'.
 * Every tooth tip emits sparks carrying the gear's tangential surface
 * velocity. Faster rotation = faster sparks + more of them.
 *
 * Keys:
 *   q / ESC   quit
 *   spc       pause / resume
 *   r         reset
 *   + / -     spin faster / slower
 *   ] / [     more / fewer sparks
 *   t / T     next / prev theme
 *   1–5       speed presets  (very slow → screaming fast)
 *
 * Themes:
 *   0 FIRE    1 MATRIX   2 PLASMA   3 NOVA     4 POISON
 *   5 OCEAN   6 GOLD     7 NEON     8 ARCTIC   9 LAVA
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/gear.c -o gear -lncurses -lm
 *
 * §1 config  §2 clock  §3 theme  §4 color  §5 coords
 * §6 entity  §7 draw   §8 screen §9 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic polar gear geometry + particle system sparks.
 *                  Gear outline computed each frame from N_TEETH tooth
 *                  definitions (outer radius, inner radius, hub) using polar
 *                  angle stepping.  No mesh storage — geometry is regenerated
 *                  each frame from the current rotation angle.
 *
 * Math           : Involute gear tooth approximated as trapezoid in polar
 *                  coordinates.  Tooth tip position at angle α:
 *                    (x,y) = GEAR_R_OUTER × (cos α, sin α)
 *                  Tangential surface velocity at tooth tip:
 *                    v_tip = GEAR_R_OUTER × ω  (ω = angular speed rad/s)
 *                  Spark initial velocity ≈ v_tip direction + random spread.
 *
 * Rendering      : Cell-aspect correction: CELL_W=8, CELL_H=16 → pixels are
 *                  taller than wide, so x-coordinates are divided by
 *                  (CELL_W/CELL_H)=0.5 to make the gear appear circular.
 *                  Gear outline drawn with Bresenham-style cell mapping.
 *
 * Data-structure : Spark pool — fixed array of N_SPARKS_MAX structs with
 *                  active flag.  O(N) per frame; emission rate scales with ω.
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

#define CELL_W   8
#define CELL_H  16

/* Gear geometry (pixels) */
#define GEAR_R_OUTER   88.0f
#define GEAR_R_INNER   64.0f
#define GEAR_R_HUB     22.0f
#define N_TEETH        10
#define TOOTH_DUTY     0.42f

/* Wireframe thresholds (pixels) */
#define THRESH_CIRC    7.5f
#define THRESH_SIDE    4.0f
#define THRESH_SPOKE   3.8f

/* Gear rotation */
#define GEAR_ROT_BASE   1.3f
#define GEAR_ROT_STEP   0.6f
#define GEAR_ROT_MAX   20.0f

/* Spark physics */
#define MAX_SPARKS      1500
#define SPARK_BASE_RATE  80.0f
#define TANG_SCALE        0.5f
#define SPARK_KICK_MIN   35.0f
#define SPARK_KICK_MAX  120.0f
#define SPARK_SCATTER    55.0f
#define SPARK_TURB       40.0f
#define SPARK_GRAVITY    28.0f
#define SPARK_DRAG        0.4f
#define SPARK_LIFE        1.9f

#define DENSITY_DEFAULT   1.0f
#define DENSITY_STEP      0.3f
#define DENSITY_MAX       6.0f

#define TAU  6.28318530f
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

enum { TARGET_FPS = 60, FPS_UPDATE_MS = 500 };

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
/* §3  themes                                                             */
/* ===================================================================== */

/*
 * Each theme describes:
 *   gear_fg      — 256-color index for bright gear wireframe
 *   gear_dim_fg  — 256-color index for dim gear lines
 *   spark_fg[7]  — color per cooling stage (freshest → dead)
 *   spark_ch[7]  — character per stage
 *   spark_at[7]  — attribute per stage: 1=A_BOLD 0=A_NORMAL 2=A_DIM
 */
typedef struct {
    const char *name;
    int  gear_fg;
    int  gear_dim_fg;
    int  spark_fg[7];
    char spark_ch[7];
    int  spark_at[7];
} Theme;

#define N_THEMES 10

/* spark_at encoding: 0=A_NORMAL  1=A_BOLD  2=A_DIM */

static const Theme THEMES[N_THEMES] = {

    /* ── 0  FIRE ─────────────────────────────────────────────────────── */
    /* white-hot → yellow → amber → orange → red-orange → red → ember    */
    { "FIRE",
      153,  67,
      { 231, 226, 220, 214, 202, 196, 160 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 1  MATRIX ───────────────────────────────────────────────────── */
    /* white flash → lime → green → mid-green → dark; digital rain vibe  */
    { "MATRIX",
       34,  22,
      { 231, 118,  82,  46,  40,  34,  22 },
      { '@', '#', '*', '+', ';', ':', '.' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 2  PLASMA ───────────────────────────────────────────────────── */
    /* white → pink → hot-magenta → purple → deep violet; electric arc   */
    { "PLASMA",
       99,  57,
      { 231, 207, 201, 165, 129,  93,  57 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 3  NOVA ─────────────────────────────────────────────────────── */
    /* white → bright-cyan → cyan → sky-blue → blue → deep-blue; stellar */
    { "NOVA",
       69,  27,
      { 231, 159, 123,  87,  51,  39,  27 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 4  POISON ───────────────────────────────────────────────────── */
    /* white → bright-yellow → yellow-green → lime → green; toxic waste  */
    { "POISON",
       64,  22,
      { 231, 226, 190, 154, 118,  82,  64 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 5  OCEAN ────────────────────────────────────────────────────── */
    /* white → ice → cyan → ocean → deep-blue; bioluminescent            */
    { "OCEAN",
       31,  23,
      { 231, 159, 123,  81,  45,  33,  24 },
      { '~', 'o', '~', '+', '.', ',', '.' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 6  GOLD ─────────────────────────────────────────────────────── */
    /* white → pale-gold → gold → ochre → dark-gold → bronze → copper    */
    { "GOLD",
      136,  94,
      { 231, 228, 220, 178, 136, 130,  94 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 7  NEON ─────────────────────────────────────────────────────── */
    /* white → light-pink → hot-pink → magenta → deep-pink; synthwave    */
    { "NEON",
      201, 164,
      { 231, 219, 213, 207, 201, 200, 164 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 8  ARCTIC ───────────────────────────────────────────────────── */
    /* white → pale-blue → ice → steel → periwinkle → cornflower → grey  */
    { "ARCTIC",
      153,  67,
      { 231, 195, 159, 153, 117,  75,  67 },
      { '*', '*', '+', '.', '.', ',', '`' },
      {   1,   1,   1,   0,   2,   2,   2 } },

    /* ── 9  LAVA ─────────────────────────────────────────────────────── */
    /* white → amber → deep-orange → red → dark-red → crimson → black-red*/
    { "LAVA",
       88,  52,
      { 231, 220, 208, 202, 196, 124,  52 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },
};

/* life thresholds for the 7 spark stages (inclusive lower bound)        */
static const float STAGE_THRESH[7] = {
    0.85f, 0.70f, 0.55f, 0.38f, 0.22f, 0.10f, 0.00f
};

static inline int spark_stage(float life)
{
    for (int i = 0; i < 6; i++)
        if (life > STAGE_THRESH[i]) return i;
    return 6;
}

/* ===================================================================== */
/* §4  color                                                              */
/* ===================================================================== */

enum {
    CP_GEAR     = 1,   /* bright gear wireframe                           */
    CP_GEAR_DIM = 2,   /* dim gear lines                                  */
    CP_S0       = 3,   /* spark stage 0 (freshest)                        */
    /* CP_S1 = 4 … CP_S6 = 9 implicit */
    CP_HUD      = 10,
};

/* Decode attr int (0/1/2) → ncurses attr_t */
static const attr_t ATTR_DEC[3] = { A_NORMAL, A_BOLD, A_DIM };

static void color_apply_theme(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        init_pair(CP_GEAR,     th->gear_fg,     -1);
        init_pair(CP_GEAR_DIM, th->gear_dim_fg, -1);
        for (int i = 0; i < 7; i++)
            init_pair(CP_S0 + i, th->spark_fg[i], -1);
        init_pair(CP_HUD, 226, -1);
    } else {
        /* 8-color fallback: white gear, yellow/red sparks                */
        init_pair(CP_GEAR,     COLOR_WHITE,  -1);
        init_pair(CP_GEAR_DIM, COLOR_WHITE,  -1);
        int fb[7] = {
            COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
            COLOR_RED,   COLOR_RED,    COLOR_RED
        };
        for (int i = 0; i < 7; i++)
            init_pair(CP_S0 + i, fb[i], -1);
        init_pair(CP_HUD, COLOR_WHITE, -1);
    }
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    color_apply_theme(theme);
}

/* ===================================================================== */
/* §5  coords                                                             */
/* ===================================================================== */

static inline int px_col(float px) { return (int)floorf(px / CELL_W); }
static inline int px_row(float py) { return (int)floorf(py / CELL_H); }

/* ===================================================================== */
/* §6  entity                                                             */
/* ===================================================================== */

typedef struct {
    float px, py, vx, vy;
    float life;   /* 1.0=just born  0.0=dead */
} Spark;

typedef struct {
    float cx, cy;
    float angle;
    float rot_speed;
    float spark_density;
    Spark sparks[MAX_SPARKS];
    float emit_acc;
} Gear;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static void gear_init(Gear *g, float max_px, float max_py)
{
    g->cx            = max_px * 0.5f;
    g->cy            = max_py * 0.5f;
    g->angle         = 0.0f;
    g->rot_speed     = GEAR_ROT_BASE;
    g->spark_density = DENSITY_DEFAULT;
    g->emit_acc      = 0.0f;
    for (int i = 0; i < MAX_SPARKS; i++) g->sparks[i].life = 0.0f;
}

static void spark_emit(Gear *g, float tip_ang)
{
    for (int i = 0; i < MAX_SPARKS; i++) {
        if (g->sparks[i].life > 0.0f) continue;
        Spark *s = &g->sparks[i];

        s->px = g->cx + GEAR_R_OUTER * cosf(tip_ang);
        s->py = g->cy + GEAR_R_OUTER * sinf(tip_ang);

        /* Tangential surface velocity scaled by TANG_SCALE              */
        float tv      = g->rot_speed * GEAR_R_OUTER * TANG_SCALE;
        float tang_vx = -sinf(tip_ang) * tv;
        float tang_vy =  cosf(tip_ang) * tv;

        /* Radial outward kick */
        float kick    = SPARK_KICK_MIN + randf() * (SPARK_KICK_MAX - SPARK_KICK_MIN);
        float kick_vx = cosf(tip_ang) * kick;
        float kick_vy = sinf(tip_ang) * kick;

        /* Random scatter */
        float sc_vx = (randf() - 0.5f) * SPARK_SCATTER;
        float sc_vy = (randf() - 0.5f) * SPARK_SCATTER;

        s->vx   = tang_vx + kick_vx + sc_vx;
        s->vy   = tang_vy + kick_vy + sc_vy;
        s->life = 0.85f + randf() * 0.15f;
        return;
    }
}

static void gear_tick(Gear *g, float dt, float max_px, float max_py)
{
    g->angle += g->rot_speed * dt;
    if (g->angle > TAU) g->angle -= TAU;

    /* Emission rate proportional to rotation speed */
    float speed_norm = g->rot_speed / GEAR_ROT_BASE;
    float total_rate = SPARK_BASE_RATE * speed_norm * g->spark_density;
    if (total_rate > 1200.0f) total_rate = 1200.0f;

    g->emit_acc += total_rate * dt;
    while (g->emit_acc >= 1.0f) {
        g->emit_acc -= 1.0f;
        int   t       = rand() % N_TEETH;
        float tip_ang = g->angle
                      + (t + TOOTH_DUTY * 0.5f) * TAU / N_TEETH;
        spark_emit(g, tip_ang);
    }

    float decay = dt / SPARK_LIFE;
    for (int i = 0; i < MAX_SPARKS; i++) {
        Spark *s = &g->sparks[i];
        if (s->life <= 0.0f) continue;

        s->life -= decay;
        if (s->life <= 0.0f) { s->life = 0.0f; continue; }

        s->vy += SPARK_GRAVITY * dt;
        s->vx += (randf() - 0.5f) * SPARK_TURB * dt;
        s->vy += (randf() - 0.5f) * SPARK_TURB * 0.4f * dt;
        s->vx *= expf(-SPARK_DRAG * dt);
        s->vy *= expf(-SPARK_DRAG * dt);

        s->px += s->vx * dt;
        s->py += s->vy * dt;

        if (s->px < 0 || s->px > max_px || s->py < 0 || s->py > max_py)
            s->life = 0.0f;
    }
}

/* ===================================================================== */
/* §7  draw                                                               */
/* ===================================================================== */

/*
 * Character for a wireframe edge running in direction ang (y-down).
 *   ang=0   → '-'   ang=π/4 → '\'   ang=π/2 → '|'   ang=3π/4 → '/'
 */
static chtype line_char(float ang)
{
    float a = fmodf(ang + TAU, TAU);
    if (a < TAU/ 16 || a >= TAU*15/16) return '-';
    if (a < TAU* 3/16) return '\\';
    if (a < TAU* 5/16) return '|';
    if (a < TAU* 7/16) return '/';
    if (a < TAU* 9/16) return '-';
    if (a < TAU*11/16) return '\\';
    if (a < TAU*13/16) return '|';
    return '/';
}

/*
 * Wireframe gear:
 *   1. Hub ring        — tangential char at R_HUB
 *   2. Spokes          — radial char, hub → inner
 *   3. Tooth sides     — radial char at tooth-gap boundaries
 *   4. Inner body arc  — tangential char at R_INNER (gap zones)
 *   5. Outer tooth arc — tangential char at R_OUTER (tooth zones)
 */
static void draw_gear(WINDOW *win, const Gear *g, int cols, int rows)
{
    float Ro = GEAR_R_OUTER, Ri = GEAR_R_INNER, Rh = GEAR_R_HUB;

    int row_lo = px_row(g->cy - Ro) - 1;  if (row_lo < 0)      row_lo = 0;
    int row_hi = px_row(g->cy + Ro) + 1;  if (row_hi >= rows)  row_hi = rows - 1;
    int col_lo = px_col(g->cx - Ro) - 1;  if (col_lo < 0)      col_lo = 0;
    int col_hi = px_col(g->cx + Ro) + 1;  if (col_hi >= cols)  col_hi = cols - 1;

    for (int r = row_lo; r <= row_hi; r++) {
        for (int c = col_lo; c <= col_hi; c++) {
            float dx  = c * CELL_W + CELL_W * 0.5f - g->cx;
            float dy  = r * CELL_H + CELL_H * 0.5f - g->cy;
            float rad = sqrtf(dx * dx + dy * dy);
            if (rad > Ro + THRESH_CIRC) continue;

            float ang_g = atan2f(dy, dx);
            float ang_l = ang_g - g->angle;

            float phase = fmodf(ang_l * N_TEETH / TAU, 1.0f);
            if (phase < 0.0f) phase += 1.0f;

            bool  in_tooth = (phase < TOOTH_DUTY);

            float dp0     = phase;
            float dpT     = fabsf(phase - TOOTH_DUTY);
            if (dp0 > 0.5f) dp0 = 1.0f - dp0;
            float arc_side = ((dp0 < dpT) ? dp0 : dpT) * (TAU / N_TEETH) * rad;

            float spoke_period = TAU / N_TEETH;
            float ang_mod      = fmodf(ang_l, spoke_period);
            if (ang_mod < 0.0f) ang_mod += spoke_period;
            float spoke_ang_d  = (ang_mod < spoke_period * 0.5f)
                               ? ang_mod : spoke_period - ang_mod;
            float arc_spoke    = spoke_ang_d * rad;

            chtype ch = 0;
            attr_t at = A_NORMAL;
            int    cp = CP_GEAR;

            if (fabsf(rad - Rh) < THRESH_CIRC) {
                ch = line_char(ang_g + TAU * 0.25f);
                at = A_BOLD; cp = CP_GEAR;
            } else if (rad > Rh && rad < Ri && arc_spoke < THRESH_SPOKE) {
                ch = line_char(ang_g);
                at = A_BOLD; cp = CP_GEAR;
            } else if (rad > Ri - THRESH_SIDE * 0.5f && rad < Ro
                       && arc_side < THRESH_SIDE) {
                ch = line_char(ang_g);
                at = A_BOLD; cp = CP_GEAR;
            } else if (!in_tooth && fabsf(rad - Ri) < THRESH_CIRC) {
                ch = line_char(ang_g + TAU * 0.25f);
                at = A_NORMAL; cp = CP_GEAR_DIM;
            } else if (in_tooth && fabsf(rad - Ro) < THRESH_CIRC) {
                ch = line_char(ang_g + TAU * 0.25f);
                at = A_BOLD; cp = CP_GEAR;
            }

            if (!ch) continue;
            wattron(win, COLOR_PAIR(cp) | at);
            mvwaddch(win, r, c, ch);
            wattroff(win, COLOR_PAIR(cp) | at);
        }
    }
}

/*
 * Draw sparks using the active theme's per-stage color/char/attr.
 * Stages run freshest (0) → deadest (6), driven by STAGE_THRESH[].
 */
static void draw_sparks(WINDOW *win, const Gear *g,
                        int cols, int rows, int theme_idx)
{
    const Theme *th = &THEMES[theme_idx];

    for (int i = 0; i < MAX_SPARKS; i++) {
        const Spark *s = &g->sparks[i];
        if (s->life <= 0.0f) continue;

        int c = px_col(s->px);
        int r = px_row(s->py);
        if (c < 0 || c >= cols || r < 0 || r >= rows) continue;

        int    st = spark_stage(s->life);
        chtype ch = (chtype)(unsigned char)th->spark_ch[st];
        attr_t at = ATTR_DEC[th->spark_at[st]];
        int    cp = CP_S0 + st;

        wattron(win, COLOR_PAIR(cp) | at);
        mvwaddch(win, r, c, ch);
        wattroff(win, COLOR_PAIR(cp) | at);
    }
}

/* ===================================================================== */
/* §8  screen / scene                                                     */
/* ===================================================================== */

typedef struct {
    Gear  gear;
    bool  paused;
    float max_px, max_py;
    int   theme;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->max_px = (float)(cols * CELL_W);
    s->max_py = (float)(rows * CELL_H);
    s->paused = false;
    /* keep current theme on reset */
    gear_init(&s->gear, s->max_px, s->max_py);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    gear_tick(&s->gear, dt, s->max_px, s->max_py);
}

static void scene_draw(const Scene *s, WINDOW *win,
                       int cols, int rows, double fps)
{
    draw_sparks(win, &s->gear, cols, rows, s->theme);
    draw_gear(win, &s->gear, cols, rows);

    int live = 0;
    for (int i = 0; i < MAX_SPARKS; i++)
        if (s->gear.sparks[i].life > 0.0f) live++;

    wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvwprintw(win, 0, 0,
        " %.0f fps  speed:%.1f rad/s  density:%.1fx  sparks:%d"
        "  theme:[%d] %s ",
        fps, s->gear.rot_speed, s->gear.spark_density,
        live, s->theme, THEMES[s->theme].name);
    mvwprintw(win, rows - 1, 0,
        " q:quit  spc:pause  r:reset  +/-:speed  ]/[:density"
        "  t/T:theme  1-5:speed presets ");
    wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);
}

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }

static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_render(Screen *sc, const Scene *s, double fps)
{
    erase();
    scene_draw(s, stdscr, sc->cols, sc->rows, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

static bool app_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': sc->paused = !sc->paused; break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;
    case '+': case '=':
        sc->gear.rot_speed += GEAR_ROT_STEP;
        if (sc->gear.rot_speed > GEAR_ROT_MAX) sc->gear.rot_speed = GEAR_ROT_MAX;
        break;
    case '-':
        sc->gear.rot_speed -= GEAR_ROT_STEP;
        if (sc->gear.rot_speed < 0.2f) sc->gear.rot_speed = 0.2f;
        break;
    case ']':
        sc->gear.spark_density += DENSITY_STEP;
        if (sc->gear.spark_density > DENSITY_MAX) sc->gear.spark_density = DENSITY_MAX;
        break;
    case '[':
        sc->gear.spark_density -= DENSITY_STEP;
        if (sc->gear.spark_density < 0.2f) sc->gear.spark_density = 0.2f;
        break;
    case 't': case 'T': {
        int dir = (ch == 't') ? 1 : -1;
        sc->theme = (sc->theme + dir + N_THEMES) % N_THEMES;
        color_apply_theme(sc->theme);
        break;
    }
    case '1': sc->gear.rot_speed =  0.4f; break;
    case '2': sc->gear.rot_speed =  2.0f; break;
    case '3': sc->gear.rot_speed =  6.0f; break;
    case '4': sc->gear.rot_speed = 12.0f; break;
    case '5': sc->gear.rot_speed = 20.0f; break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    App *app     = &g_app;
    app->running = 1;
    app->scene.theme = 0;   /* start with FIRE */

    screen_init(&app->screen, app->scene.theme);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t prev = clock_ns(), fps_acc = 0;
    int     fps_count = 0;
    double  fps_disp  = 0.0;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->scene.max_px    = (float)(app->screen.cols * CELL_W);
            app->scene.max_py    = (float)(app->screen.rows * CELL_H);
            app->scene.gear.cx   = app->scene.max_px * 0.5f;
            app->scene.gear.cy   = app->scene.max_py * 0.5f;
            app->need_resize     = 0;
            prev = clock_ns();
        }

        int64_t now   = clock_ns();
        int64_t dt_ns = now - prev;
        prev = now;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        scene_tick(&app->scene, dt);

        fps_count++;
        fps_acc += dt_ns;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp  = (double)fps_count / ((double)fps_acc / NS_PER_SEC);
            fps_count = 0; fps_acc = 0;
        }

        screen_render(&app->screen, &app->scene, fps_disp);

        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        int key = getch();
        if (key != ERR && !app_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
