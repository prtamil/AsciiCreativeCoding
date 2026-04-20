/* perlin_terrain_bot.c  — Learning Edition
 * Self-balancing wheel-bot on infinite Perlin terrain
 * Three interactive view modes teach inverted-pendulum + PID control
 *
 * Build:  gcc -std=c11 -O2 -Wall -Wextra robots/perlin_terrain_bot.c \
 *             -o perlin_terrain_bot -lncurses -lm
 *
 * Keys:
 *   SPACE  pause          q / ESC  quit        r  reset
 *   ↑ / ↓  drive speed    p        toggle PID
 *   m      cycle view     g        cycle gain preset
 *   + / -  tune Kp        [ / ]    sim Hz
 */

/* ── §1  CONFIG ─────────────────────────────────────────────────── */
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <stdio.h>

/*── pixel / cell ──*/
#define CELL_W        8
#define CELL_H       16

/*── physics (SI) ──*/
#define GRAVITY      9.81f   /* m/s² */
#define PEND_LEN     1.0f    /* metres  — axle to body COM */
#define MASS_CART    4.0f    /* kg  — chassis + wheel mass */
#define MASS_POLE    2.0f    /* kg  — upper body mass */
#define MAX_FORCE  200.0f    /* N   — motor force clamp */
#define PIX_PER_M  100.0f    /* px per metre (display scale) */
#define FALL_ANGLE   1.05f   /* rad (~60°) — fallen threshold */

/*── PID defaults ──*/
#define KP_DEF     120.0f
#define KI_DEF       0.2f
#define KD_DEF      18.0f
#define WINDUP_MAX   5.0f    /* integral anti-windup clamp */

/*── slope compensation ──*/
#define SLOPE_FEED   0.65f   /* fraction of slope fed to θ_ref */

/*── bot geometry (pixels) ──*/
#define WHEEL_R     18.0f
#define AXLE_HW     24.0f
#define BODY_H      96.0f
#define BODY_HW      8.0f

/*── drive ──*/
#define DRIVE_DEF   55.0f    /* px/s */
#define DRIVE_STEP  15.0f
#define DRIVE_MAX  160.0f

/*── terrain ring-buffer ──*/
#define TBUF       1024      /* power of 2 */
#define TMASK      (TBUF-1)
#define T_FREQ     0.022f    /* Perlin frequency per world-column */
#define T_AMP_F    0.20f     /* amplitude fraction of screen height */
#define T_MID_F    0.62f     /* baseline fraction of screen height */

/*── timing ──*/
#define FPS_TARGET   60
#define TICK_HZ_DEF 120

/*── history (phase portrait + error plot) ──*/
#define HIST_LEN   240       /* samples kept for phase portrait */

/*── view modes ──*/
typedef enum { VIEW_TELE = 0, VIEW_EQ, VIEW_PHASE, VIEW_COUNT } ViewMode;
static const char *VIEW_NAMES[] = { "TELEMETRY", "EQUATIONS", "PHASE SPACE" };

/*── color pairs ──*/
#define CP_SKY      1
#define CP_STAR     2
#define CP_SURF     3
#define CP_ROCK     4
#define CP_CHASSIS  5
#define CP_WHEEL    6
#define CP_SPOKE    7
#define CP_AXLE     8
#define CP_HUD      9
#define CP_WARN    10
#define CP_DIM     11
#define CP_TITLE   12
#define CP_BARP    13
#define CP_BARN    14
#define CP_SPD     15
#define CP_EQ      16   /* equation text */
#define CP_VAL     17   /* live numeric values */
#define CP_GOOD    18   /* stable / positive indicator */

/* ── §2  CLOCK ──────────────────────────────────────────────────── */
static long long now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* ── §3  COLOR ──────────────────────────────────────────────────── */
static void color_init(void) {
    start_color();
    use_default_colors();
    init_pair(CP_SKY,      17,  -1);
    init_pair(CP_STAR,    226,  -1);
    init_pair(CP_SURF,     34,  -1);
    init_pair(CP_ROCK,    240,  -1);
    init_pair(CP_CHASSIS, 253,  -1);
    init_pair(CP_WHEEL,    51,  -1);
    init_pair(CP_SPOKE,   226,  -1);
    init_pair(CP_AXLE,    201,  -1);
    init_pair(CP_HUD,      82,  -1);
    init_pair(CP_WARN,    196,  -1);
    init_pair(CP_DIM,     238,  -1);
    init_pair(CP_TITLE,   255,  -1);
    init_pair(CP_BARP,     51,  -1);
    init_pair(CP_BARN,    213,  -1);
    init_pair(CP_SPD,     214,  -1);
    init_pair(CP_EQ,      159,  -1);   /* light cyan — equation text */
    init_pair(CP_VAL,     229,  -1);   /* pale yellow — live values */
    init_pair(CP_GOOD,     46,  -1);   /* bright green — stable */
}

/* ── §4  PERLIN NOISE ───────────────────────────────────────────── */
static unsigned char perm[512];

static void perlin_init(unsigned int seed) {
    unsigned char p[256];
    srand(seed);
    for (int i = 0; i < 256; i++) p[i] = (unsigned char)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        unsigned char t = p[i]; p[i] = p[j]; p[j] = t;
    }
    for (int i = 0; i < 512; i++) perm[i] = p[i & 255];
}

/* Perlin smoothstep and gradient */
static float p_fade(float t)           { return t*t*t*(t*(t*6.f-15.f)+10.f); }
static float p_lerp(float a, float b, float t) { return a + t*(b-a); }
static float p_grad(int h, float x)   { return (h & 1) ? x : -x; }

static float perlin1(float x) {
    int xi = (int)floorf(x) & 255;
    float xf = x - floorf(x);
    return p_lerp(p_grad(perm[xi], xf),
                  p_grad(perm[xi+1], xf-1.f), p_fade(xf));
}

/* Fractional Brownian Motion — 5 octaves */
static float fbm(float x) {
    float v = 0.f, a = 0.5f, f = 1.f;
    for (int i = 0; i < 5; i++) { v += a*perlin1(x*f); a*=.5f; f*=2.f; }
    return v;   /* ≈ [-1, +1] */
}

/* ── §5  COORDS + DRAW PRIMITIVES ───────────────────────────────── */
static inline int px_to_cx(float px) { return (int)(px / CELL_W); }
static inline int px_to_cy(float py) { return (int)(py / CELL_H); }

static void put_c(int r, int c, chtype ch, attr_t a, int pair,
                  int rows, int cols) {
    if (r>=0 && r<rows && c>=0 && c<cols) {
        attron(a | COLOR_PAIR(pair));
        mvaddch(r, c, ch);
        attroff(a | COLOR_PAIR(pair));
    }
}

static void put_s(int r, int c, const char *s, attr_t a, int pair,
                  int rows, int cols) {
    if (r < 0 || r >= rows) return;
    attron(a | COLOR_PAIR(pair));
    for (int i = 0; s[i] && c+i < cols; i++)
        mvaddch(r, c+i, (unsigned char)s[i]);
    attroff(a | COLOR_PAIR(pair));
}

/* Ascii bar centred at zero; W cols wide, v in [-1,+1] */
static void draw_bar(int r, int c, int W, float v,
                     int pp, int pn, int rows, int cols) {
    int half = W/2, mid = c+half;
    int fill = (int)(fabsf(v)*half); if (fill>half) fill=half;
    attron(COLOR_PAIR(CP_DIM));
    for (int i=0;i<W;i++) put_c(r,c+i,' ',0,CP_DIM,rows,cols);
    put_c(r,mid,'|',0,CP_DIM,rows,cols);
    attroff(COLOR_PAIR(CP_DIM));
    int cp = (v>=0.f)?pp:pn;
    attron(COLOR_PAIR(cp)|A_BOLD);
    if (v>=0.f) for(int i=0;i<fill;i++) put_c(r,mid+1+i,'=',A_BOLD,cp,rows,cols);
    else        for(int i=0;i<fill;i++) put_c(r,mid-1-i,'=',A_BOLD,cp,rows,cols);
    attroff(COLOR_PAIR(cp)|A_BOLD);
}

/* ── §6  TERRAIN ────────────────────────────────────────────────── */
typedef struct {
    float h[TBUF];   /* surface height in px from screen top */
    int   gen_col;
    int   rows, cols;
} Terrain;

/* Height at a world column (px from top) */
static float terrain_h_at(int wc, int rows) {
    float mid = rows * T_MID_F * CELL_H;
    float amp = rows * T_AMP_F * CELL_H;
    return mid + fbm(wc * T_FREQ) * amp;
}

/* Slope angle (rad) at world column; positive = uphill to the right */
static float terrain_slope_at(const Terrain *t, int wc) {
    float dh = t->h[(wc+1)&TMASK] - t->h[wc&TMASK];
    return -atanf(dh / CELL_W);   /* negative: screen +y is down */
}

static void terrain_ensure(Terrain *t, int upto) {
    for (int c = t->gen_col+1; c <= upto; c++)
        t->h[c&TMASK] = terrain_h_at(c, t->rows);
    if (upto > t->gen_col) t->gen_col = upto;
}

static void terrain_init(Terrain *t, int rows, int cols) {
    t->rows = rows; t->cols = cols; t->gen_col = -1;
    terrain_ensure(t, cols + 64);
}

/* ── §7  GAIN PRESETS (for learning PID tuning) ─────────────────── */
typedef struct {
    const char *name;      /* short label */
    const char *lesson[3]; /* three lines explaining what to observe */
    float kp, ki, kd;
} GainPreset;

static const GainPreset PRESETS[] = {
    {
        "BALANCED",
        { "Kp=120 Ki=0.2 Kd=18: well-tuned baseline.",
          "Responds quickly, damps oscillation,",
          "corrects slope-induced drift via Ki." },
        120.f, 0.20f, 18.f
    },
    {
        "HIGH Kp ",
        { "Kp=240: stiffer proportional response.",
          "Watch: faster correction but more",
          "overshoot and oscillation on slopes." },
        240.f, 0.20f, 18.f
    },
    {
        "LOW  Kp ",
        { "Kp=40: sluggish proportional gain.",
          "Watch: bot leans further before",
          "correcting. Falls on steep terrain." },
        40.f, 0.10f, 10.f
    },
    {
        "NO   Kd ",
        { "Kd=0: no derivative (damping) term.",
          "Watch: bot oscillates continuously,",
          "never settles. Classic underdamping." },
        120.f, 0.20f,  0.f
    },
    {
        "HIGH Kd ",
        { "Kd=70: heavily overdamped response.",
          "Watch: very slow, sluggish correction.",
          "May struggle on sudden slope changes." },
        120.f, 0.20f, 70.f
    },
    {
        "NO   Ki ",
        { "Ki=0: no integral term.",
          "Watch: bot leans permanently on slope",
          "— steady-state error, never corrects." },
        120.f, 0.00f, 18.f
    },
};
#define N_PRESETS (int)(sizeof PRESETS / sizeof PRESETS[0])

/* ── §8  BOT ────────────────────────────────────────────────────── */
typedef struct {
    /* pendulum state */
    float theta;        /* body lean from terrain normal (rad) */
    float theta_dot;    /* angular velocity (rad/s) */

    /* cart-pole derived (computed each tick, exposed for display) */
    float theta_eff;    /* theta + alpha (effective lean from vertical) */
    float theta_ref;    /* PID setpoint */
    float theta_ddot;   /* angular acceleration this tick */
    float x_ddot;       /* horizontal acceleration this tick */
    float M_eff;        /* effective inertia this tick */
    float F;            /* motor force applied this tick */

    /* PID */
    float kp, ki, kd;
    float pid_int;
    float pid_prev_err;
    float pid_p, pid_i, pid_d, pid_out;
    bool  pid_on;

    /* motion / terrain */
    float world_x;
    float drive_spd;
    float spin_angle;
    float alpha;        /* terrain slope (rad) */

    /* phase portrait history */
    float ph_theta[HIST_LEN];
    float ph_omega[HIST_LEN];
    int   ph_head, ph_fill;

    /* sim control */
    bool  paused;
    int   tick_hz;
    bool  fallen;
    float dist_m;

    /* learning */
    ViewMode    view;
    int         preset_idx;
} Bot;

static void bot_reset(Bot *b) {
    b->theta = 0.04f; b->theta_dot = 0.f;
    b->pid_int = 0.f; b->pid_prev_err = 0.f;
    b->pid_p = b->pid_i = b->pid_d = b->pid_out = 0.f;
    b->world_x = 0.f; b->spin_angle = 0.f;
    b->alpha = 0.f; b->fallen = false; b->dist_m = 0.f;
    b->ph_head = 0; b->ph_fill = 0;
}

static void bot_init(Bot *b) {
    memset(b, 0, sizeof *b);
    b->kp = KP_DEF; b->ki = KI_DEF; b->kd = KD_DEF;
    b->pid_on = true;
    b->drive_spd = DRIVE_DEF;
    b->tick_hz = TICK_HZ_DEF;
    b->view = VIEW_TELE;
    b->preset_idx = 0;
    bot_reset(b);
}

static void bot_apply_preset(Bot *b) {
    const GainPreset *p = &PRESETS[b->preset_idx];
    b->kp = p->kp; b->ki = p->ki; b->kd = p->kd;
    b->pid_int = 0.f;   /* clear integral on gain change */
}

/* ── §9  PHYSICS ────────────────────────────────────────────────── */
static void bot_tick(Bot *b, const Terrain *t, float dt) {
    if (b->fallen) return;

    /* advance along terrain */
    b->world_x += b->drive_spd * dt;
    b->dist_m  += b->drive_spd * dt / PIX_PER_M;

    int wc   = (int)(b->world_x / CELL_W);
    b->alpha = terrain_slope_at(t, wc);

    b->spin_angle += (b->drive_spd / WHEEL_R) * dt;

    /* effective lean from vertical = body angle + terrain slope */
    b->theta_eff = b->theta + b->alpha;

    /* PID setpoint: lean into slope so bot stays upright on terrain */
    b->theta_ref = -b->alpha * SLOPE_FEED;
    float err = b->theta - b->theta_ref;

    b->F = 0.f;
    if (b->pid_on) {
        b->pid_int += err * dt;
        if (b->pid_int >  WINDUP_MAX) b->pid_int =  WINDUP_MAX;
        if (b->pid_int < -WINDUP_MAX) b->pid_int = -WINDUP_MAX;

        float deriv = (dt > 1e-9f) ? (err - b->pid_prev_err) / dt : 0.f;
        b->pid_prev_err = err;

        b->pid_p = b->kp * err;
        b->pid_i = b->ki * b->pid_int;
        b->pid_d = b->kd * deriv;
        b->F = b->pid_p + b->pid_i + b->pid_d;
    }
    if (b->F >  MAX_FORCE) b->F =  MAX_FORCE;
    if (b->F < -MAX_FORCE) b->F = -MAX_FORCE;
    b->pid_out = b->F;

    /* Exact Lagrangian cart-pole on slope */
    float st = sinf(b->theta_eff);
    float ct = cosf(b->theta_eff);
    b->M_eff   = MASS_CART + MASS_POLE * st * st;
    b->x_ddot  = (b->F + MASS_POLE*st*(PEND_LEN*b->theta_dot*b->theta_dot
                  - GRAVITY*ct)) / b->M_eff;
    b->theta_ddot = (GRAVITY*st - b->x_ddot*ct) / PEND_LEN;

    b->theta_dot += b->theta_ddot * dt;
    b->theta     += b->theta_dot  * dt;

    /* record phase history */
    b->ph_theta[b->ph_head] = b->theta;
    b->ph_omega[b->ph_head] = b->theta_dot;
    b->ph_head = (b->ph_head + 1) % HIST_LEN;
    if (b->ph_fill < HIST_LEN) b->ph_fill++;

    if (fabsf(b->theta_eff) > FALL_ANGLE) b->fallen = true;
}

/* ── §10  SIGNAL ────────────────────────────────────────────────── */
static volatile sig_atomic_t running     = 1;
static volatile sig_atomic_t need_resize = 0;

static void sig_handler(int s) {
    if (s == SIGINT || s == SIGTERM) running = 0;
    if (s == SIGWINCH) need_resize = 1;
}

/* ── §11  DRAW TERRAIN ──────────────────────────────────────────── */
static bool is_star(int r, int c) {
    unsigned h = (unsigned)(r*7919 + c*6271);
    h ^= h>>13; h *= 0x45d9f3b; h ^= h>>17;
    return (h % 60) == 0;
}

static chtype surface_glyph(float dh) {
    if (dh > CELL_W * 0.22f) return '/';
    if (dh < -CELL_W * 0.22f) return '\\';
    return '_';
}

static void draw_terrain(const Terrain *t, int bot_wc,
                         float bot_sub, int bot_sc,
                         int rows, int cols) {
    for (int sc = 0; sc < cols; sc++) {
        int wc = bot_wc - bot_sc + sc;
        if (wc < 0) wc = 0;
        float h   = t->h[wc & TMASK];
        float dh  = t->h[(wc+1)&TMASK] - h;
        int surf  = px_to_cy(h);
        chtype sg = surface_glyph(dh);
        (void)bot_sub;

        for (int r = 0; r < rows; r++) {
            if (r < surf) {
                if (is_star(r, sc))
                    put_c(r, sc, '.', A_BOLD, CP_STAR, rows, cols);
            } else if (r == surf) {
                int cp = (fabsf(dh) > CELL_W*0.22f) ? CP_ROCK : CP_SURF;
                put_c(r, sc, sg, A_BOLD, cp, rows, cols);
            } else if (r == surf+1) {
                put_c(r, sc, (sc%3==0)?':':'.', A_DIM, CP_SURF, rows, cols);
            } else {
                put_c(r, sc, (sc%2==0)?'#':' ', A_DIM, CP_ROCK, rows, cols);
            }
        }
    }
}

/* ── §12  DRAW BOT ──────────────────────────────────────────────── */
static void bot_body_pt(float ax, float ay, float theta, float alpha,
                        float along, float side,
                        float *ox, float *oy) {
    float ang = theta + alpha;
    *ox = ax + along*sinf(ang) + side*cosf(ang);
    *oy = ay - along*cosf(ang) + side*sinf(ang);
}

static chtype seg_glyph(int dr, int dc) {
    if (!dr)       return '-';
    if (!dc)       return '|';
    return (dr*dc < 0) ? '/' : '\\';
}

static void draw_seg(float ax, float ay, float bx, float by,
                     attr_t a, int cp, int rows, int cols) {
    int r0=px_to_cy(ay), c0=px_to_cx(ax);
    int r1=px_to_cy(by), c1=px_to_cx(bx);
    int dr=r1-r0, dc=c1-c0;
    chtype ch = seg_glyph(dr, dc);
    int sr=(r0<r1)?1:-1, sc2=(c0<c1)?1:-1;
    int err=abs(dr)-abs(dc), r=r0, c=c0;
    for (;;) {
        put_c(r, c, ch, a, cp, rows, cols);
        if (r==r1 && c==c1) break;
        int e2=2*err;
        if (e2 > -abs(dc)) { err-=abs(dc); r+=sr; }
        if (e2 <  abs(dr)) { err+=abs(dr); c+=sc2; }
    }
}

static void draw_wheel(float wcx, float wcy, float spin,
                       int rows, int cols) {
    int cx=px_to_cx(wcx), cy=px_to_cy(wcy);
    static const char spk[4]={'|','/','-','\\'};
    int si=((int)(spin/(M_PI*0.5f)))&3;
    put_c(cy-1,cx-2,'/',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy-1,cx-1,'-',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy-1,cx+1,'-',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy-1,cx+2,'\\', A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy,  cx-2,'|',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy,  cx+2,'|',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy+1,cx-2,'\\', A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy+1,cx-1,'_',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy+1,cx+1,'_',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy+1,cx+2,'/',  A_BOLD,CP_WHEEL,rows,cols);
    put_c(cy-1,cx,  spk[si],       A_BOLD,CP_SPOKE,rows,cols);
    put_c(cy,  cx-1,spk[(si+1)&3], A_BOLD,CP_SPOKE,rows,cols);
    put_c(cy,  cx+1,spk[(si+1)&3], A_BOLD,CP_SPOKE,rows,cols);
    put_c(cy+1,cx,  spk[si],       A_BOLD,CP_SPOKE,rows,cols);
    put_c(cy,  cx,  'O',            A_BOLD,CP_AXLE, rows,cols);
}

static void draw_bot(const Bot *b, const Terrain *t,
                     int bot_sc, int rows, int cols) {
    int   wc    = (int)(b->world_x / CELL_W);
    float h_px  = t->h[wc & TMASK];
    float ax_px = (float)(bot_sc * CELL_W);
    float ax_py = h_px - WHEEL_R;

    /* wheels (offset slightly along slope) */
    float lx = ax_px - AXLE_HW*cosf(b->alpha);
    float ly = ax_py + AXLE_HW*sinf(b->alpha);
    float rx = ax_px + AXLE_HW*cosf(b->alpha);
    float ry = ax_py - AXLE_HW*sinf(b->alpha);
    draw_wheel(lx, ly, b->spin_angle, rows, cols);
    draw_wheel(rx, ry, b->spin_angle, rows, cols);

    /* axle bar */
    {
        int ar=px_to_cy(ax_py), cl=px_to_cx(lx)+3, cr=px_to_cx(rx)-3;
        attron(COLOR_PAIR(CP_CHASSIS)|A_BOLD);
        for(int c=cl;c<=cr&&c<cols;c++) if(ar>=0&&ar<rows) mvaddch(ar,c,'=');
        attroff(COLOR_PAIR(CP_CHASSIS)|A_BOLD);
    }

    /* chassis box */
    float p0x,p0y,p1x,p1y;
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,  0,   -BODY_HW,&p0x,&p0y);
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,BODY_H,-BODY_HW,&p1x,&p1y);
    draw_seg(p0x,p0y,p1x,p1y, A_BOLD, CP_CHASSIS, rows, cols);

    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,  0,   +BODY_HW,&p0x,&p0y);
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,BODY_H,+BODY_HW,&p1x,&p1y);
    draw_seg(p0x,p0y,p1x,p1y, A_BOLD, CP_CHASSIS, rows, cols);

    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,  0,-BODY_HW,&p0x,&p0y);
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,  0,+BODY_HW,&p1x,&p1y);
    draw_seg(p0x,p0y,p1x,p1y, A_NORMAL, CP_CHASSIS, rows, cols);

    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,BODY_H,-BODY_HW,&p0x,&p0y);
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,BODY_H,+BODY_HW,&p1x,&p1y);
    draw_seg(p0x,p0y,p1x,p1y, A_NORMAL, CP_CHASSIS, rows, cols);

    /* spine */
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,   4, 0,&p0x,&p0y);
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,BODY_H-4,0,&p1x,&p1y);
    draw_seg(p0x,p0y,p1x,p1y, A_DIM, CP_WHEEL, rows, cols);

    /* beacon */
    bot_body_pt(ax_px,ax_py,b->theta,b->alpha,BODY_H,0,&p1x,&p1y);
    attr_t ba=(((int)(b->spin_angle*2))&1)?A_BOLD:A_DIM;
    put_c(px_to_cy(p1y),px_to_cx(p1x),'*',ba,CP_WARN,rows,cols);

    /* lean readout */
    {
        float deg = b->theta_eff * 57.3f;
        int ar=px_to_cy(ax_py), ac=px_to_cx(ax_px);
        int cp=(fabsf(deg)>20.f)?CP_WARN:CP_HUD;
        attron(COLOR_PAIR(cp));
        mvprintw(ar, ac-4, "%+.1f\xc2\xb0", deg);
        attroff(COLOR_PAIR(cp));
    }

    if (b->fallen) {
        int mr=rows/2, mc=cols/2-9;
        put_s(mr-1,mc,"   !! FALLEN !!   ",A_BOLD|A_BLINK,CP_WARN,rows,cols);
        put_s(mr,  mc,"  g=preset  r=reset",A_BOLD,CP_DIM,rows,cols);
    }
}

/* ── §13  VIEW A — TELEMETRY ────────────────────────────────────── */
static void draw_telemetry(const Bot *b, int rows, int cols,
                           int fps, int c0) {
    int bw=14, bc=c0+18, r=0;

    /* separator helper (nested function — GCC extension) */
    auto void sep(void);
    void sep(void){
        attron(COLOR_PAIR(CP_DIM));
        for(int i=0;i<31;i++)
            put_c(r,c0+i,(i==0||i==30)?'+':'-',0,CP_DIM,rows,cols);
        attroff(COLOR_PAIR(CP_DIM)); r++;
    }

    put_s(r++,c0," TELEMETRY              ",A_BOLD,CP_TITLE,rows,cols);
    sep();

    { float deg=b->theta_eff*57.3f;
      int cp=fabsf(deg)>20.f?CP_WARN:CP_HUD;
      char buf[32]; snprintf(buf,sizeof buf," \xce\xb8  %+6.2f\xc2\xb0 ",deg);
      put_s(r,c0,buf,A_BOLD,cp,rows,cols);
      draw_bar(r,bc,bw,deg/35.f,CP_BARP,CP_BARN,rows,cols); r++; }

    { char buf[32]; snprintf(buf,sizeof buf," \xcf\x89  %+6.2f   ",b->theta_dot);
      put_s(r,c0,buf,A_NORMAL,CP_HUD,rows,cols);
      draw_bar(r,bc,bw,b->theta_dot/6.f,CP_BARP,CP_BARN,rows,cols); r++; }

    { float sdeg=b->alpha*57.3f;
      int cp=fabsf(sdeg)>15.f?CP_WARN:CP_HUD;
      char buf[32]; snprintf(buf,sizeof buf," \xce\xb1  %+6.2f\xc2\xb0 ",sdeg);
      put_s(r,c0,buf,A_NORMAL,cp,rows,cols);
      draw_bar(r,bc,bw,sdeg/25.f,CP_BARN,CP_BARP,rows,cols); r++; }

    { float mps=b->drive_spd/PIX_PER_M;
      char buf[32]; snprintf(buf,sizeof buf," spd  %+5.2fm/s ",mps);
      put_s(r++,c0,buf,A_BOLD,CP_SPD,rows,cols); }

    { char buf[32]; snprintf(buf,sizeof buf," dist %7.1fm   ",b->dist_m);
      put_s(r++,c0,buf,A_NORMAL,CP_HUD,rows,cols); }

    sep();
    put_s(r++,c0," PID OUTPUT             ",A_BOLD,CP_TITLE,rows,cols);

    if (b->pid_on) {
        char buf[32];
        snprintf(buf,sizeof buf," P   %+8.2f  ",b->pid_p);
        put_s(r,c0,buf,A_NORMAL,CP_HUD,rows,cols);
        draw_bar(r,bc,bw,b->pid_p/MAX_FORCE,CP_BARP,CP_BARN,rows,cols); r++;

        snprintf(buf,sizeof buf," I   %+8.2f  ",b->pid_i);
        put_s(r++,c0,buf,A_DIM,CP_HUD,rows,cols);

        snprintf(buf,sizeof buf," D   %+8.2f  ",b->pid_d);
        put_s(r,c0,buf,A_NORMAL,CP_HUD,rows,cols);
        draw_bar(r,bc,bw,b->pid_d/(MAX_FORCE*0.4f),CP_BARP,CP_BARN,rows,cols); r++;

        snprintf(buf,sizeof buf," \xce\xa3   %+8.2f  ",b->pid_out);
        int cp=fabsf(b->pid_out)>MAX_FORCE*0.85f?CP_WARN:CP_HUD;
        put_s(r,c0,buf,A_BOLD,cp,rows,cols);
        draw_bar(r,bc,bw,b->pid_out/MAX_FORCE,CP_BARP,CP_BARN,rows,cols); r++;
    } else {
        put_s(r++,c0," PID  DISABLED          ",A_BOLD,CP_WARN,rows,cols);
        r+=3;
    }

    sep();

    { char buf[32]; snprintf(buf,sizeof buf," Kp:%.0f Ki:%.2f Kd:%.0f  ",b->kp,b->ki,b->kd);
      put_s(r++,c0,buf,A_NORMAL,CP_DIM,rows,cols); }
    { char buf[32]; snprintf(buf,sizeof buf," %dfps  %dHz              ",fps,b->tick_hz);
      put_s(r++,c0,buf,A_NORMAL,CP_DIM,rows,cols); }

    sep();
    put_s(r++,c0," q:quit SPC:pause r:reset",A_BOLD,CP_HUD,rows,cols);
    put_s(r++,c0," \xe2\x86\x91\xe2\x86\x93:speed  p:PID     ",A_BOLD,CP_HUD,rows,cols);
    put_s(r++,c0," m:view  g:preset +/-:Kp ",A_BOLD,CP_HUD,rows,cols);
}

/* ── §14  VIEW B — EQUATIONS ────────────────────────────────────── */
/*
 * Shows the Lagrangian cart-pole equations with live values substituted.
 * Goal: reader can trace exactly how the physics produces the numbers.
 */
static void draw_equations(const Bot *b, int rows, int cols, int c0) {
    int r = 0;
    char buf[64];

    auto void hdr(const char *s);
    void hdr(const char *s) {
        put_s(r++,c0,s,A_BOLD,CP_TITLE,rows,cols);
    }
    auto void eq(const char *label, const char *expr);
    void eq(const char *label, const char *expr) {
        put_s(r,  c0+1, label, A_BOLD,   CP_EQ,  rows,cols);
        put_s(r++,c0+10, expr,  A_NORMAL, CP_VAL, rows,cols);
    }
    auto void sep(void);
    void sep(void){
        attron(COLOR_PAIR(CP_DIM));
        for(int i=0;i<31;i++)
            put_c(r,c0+i,(i==0||i==30)?'+':'-',0,CP_DIM,rows,cols);
        attroff(COLOR_PAIR(CP_DIM)); r++;
    }
    auto void note(const char *s);
    void note(const char *s){ put_s(r++,c0+1,s,A_DIM,CP_DIM,rows,cols); }

    hdr(" INVERTED PENDULUM EQ   ");
    sep();

    /* Effective lean */
    note("Effective lean from vertical:");
    snprintf(buf,sizeof buf,"%.3f+%.3f=%+.3f rad",
             b->theta, b->alpha, b->theta_eff);
    eq("\xce\xb8_eff=\xce\xb8+\xce\xb1", buf);
    snprintf(buf,sizeof buf,"(%+.1f\xc2\xb0)", b->theta_eff*57.3f);
    put_s(r++,c0+12,buf,A_BOLD,
          fabsf(b->theta_eff)>0.35f?CP_WARN:CP_GOOD, rows,cols);

    sep();
    note("Effective inertia:");
    snprintf(buf,sizeof buf,"%.1f+%.1f*%.4f=%.3f kg",
             MASS_CART, MASS_POLE,
             sinf(b->theta_eff)*sinf(b->theta_eff), b->M_eff);
    eq("M_eff   ", buf);

    note("Horizontal accel (F drives cart):");
    snprintf(buf,sizeof buf,"%+.3f m/s\xc2\xb2", b->x_ddot);
    eq("\xc3\xbc (x\xcc\x88)  ", buf);

    note("Angular accel (g tips, x\xcc\x88 corrects):");
    snprintf(buf,sizeof buf,"%+.3f rad/s\xc2\xb2", b->theta_ddot);
    eq("\xce\xb8\xcc\x88      ", buf);

    sep();
    hdr(" PID CONTROL            ");
    sep();

    note("Setpoint: lean into slope:");
    snprintf(buf,sizeof buf,"-%.2f*%.3f=%+.3f rad",
             SLOPE_FEED, b->alpha, b->theta_ref);
    eq("\xce\xb8_ref   ", buf);

    note("Error drives all three terms:");
    snprintf(buf,sizeof buf,"%+.4f - (%+.4f) = %+.4f",
             b->theta, b->theta_ref, b->theta - b->theta_ref);
    eq("error   ", buf);

    note("P — stiffness (instant correction)");
    snprintf(buf,sizeof buf,"%.0f * %+.4f = %+.2f N",
             b->kp, b->theta-b->theta_ref, b->pid_p);
    eq("Kp*e    ", buf);

    note("I — drift removal (slope offset)");
    snprintf(buf,sizeof buf,"%.2f * integ = %+.2f N", b->ki, b->pid_i);
    eq("Ki*\xe2\x88\xab""e    ", buf);

    note("D — damping (brakes oscillation)");
    snprintf(buf,sizeof buf,"%.0f * \xce\xb8\xcc\x87 = %+.2f N", b->kd, b->pid_d);
    eq("Kd*\xce\xb8\xcc\x87    ", buf);

    note("Total motor force:");
    snprintf(buf,sizeof buf,"%+.2f N  (max \xc2\xb1%.0f N)", b->pid_out, MAX_FORCE);
    int cp=fabsf(b->pid_out)>MAX_FORCE*0.85f?CP_WARN:CP_GOOD;
    put_s(r,c0+1,"F total ", A_BOLD, CP_EQ, rows,cols);
    put_s(r++,c0+10,buf, A_BOLD, cp, rows,cols);

    sep();
    /* Stability margin */
    {
        float margin = FALL_ANGLE - fabsf(b->theta_eff);
        float frac   = margin / FALL_ANGLE;
        int   bar_w  = 20;
        int   filled = (int)(frac * bar_w);
        int   cp     = frac > 0.5f ? CP_GOOD : (frac > 0.25f ? CP_SPD : CP_WARN);
        note("Safety margin to fall (60\xc2\xb0 limit):");
        attron(COLOR_PAIR(cp)|A_BOLD);
        mvprintw(r, c0+1, "margin %+.1f\xc2\xb0 [", (margin)*57.3f);
        attroff(COLOR_PAIR(cp)|A_BOLD);
        attron(COLOR_PAIR(cp));
        int sc = c0+16;
        for(int i=0;i<bar_w;i++)
            put_c(r,sc+i, i<filled?'|':' ', 0,cp,rows,cols);
        put_c(r,sc+bar_w,']',0,CP_DIM,rows,cols);
        attroff(COLOR_PAIR(cp)); r++;
    }
    sep();
    put_s(r++,c0," m:next view  g:preset  ",A_BOLD,CP_HUD,rows,cols);
}

/* ── §15  VIEW C — PHASE PORTRAIT ──────────────────────────────── */
/*
 * Phase space: X-axis = θ (lean), Y-axis = ω (angular velocity).
 * A stable controller draws a spiral converging to (0,0).
 * Underdamped → wide circles.  Overdamped → slow straight line.
 */
static void draw_phase(const Bot *b, int rows, int cols, int c0) {
    int r = 0;
    char buf[64];

    auto void sep(void);
    void sep(void){
        attron(COLOR_PAIR(CP_DIM));
        for(int i=0;i<31;i++)
            put_c(r,c0+i,(i==0||i==30)?'+':'-',0,CP_DIM,rows,cols);
        attroff(COLOR_PAIR(CP_DIM)); r++;
    }

    put_s(r++,c0," PHASE PORTRAIT         ",A_BOLD,CP_TITLE,rows,cols);
    sep();
    put_s(r++,c0,"  \xce\xb8 (lean) vs \xcf\x89 (spin rate) ",A_DIM,CP_DIM,rows,cols);

    /* plot area */
    int plot_r0 = r;
    int plot_h  = 12;
    int plot_w  = 29;
    int mid_r   = plot_r0 + plot_h/2;
    int mid_c   = c0 + 1 + plot_w/2;

    /* axes */
    for (int i=0;i<plot_h;i++)
        put_c(plot_r0+i, mid_c, '|', A_DIM, CP_DIM, rows, cols);
    for (int i=0;i<plot_w;i++)
        put_c(mid_r, c0+1+i, '-', A_DIM, CP_DIM, rows, cols);
    put_c(mid_r,mid_c,'+',A_DIM,CP_DIM,rows,cols);

    /* axis labels */
    put_s(mid_r-1, c0+1,  "-\xce\xb8", A_DIM, CP_DIM, rows, cols);
    put_s(mid_r-1, c0+28, "+\xce\xb8", A_DIM, CP_DIM, rows, cols);
    put_s(plot_r0,   mid_c-2, "+\xcf\x89", A_DIM, CP_DIM, rows, cols);
    put_s(plot_r0+plot_h-1, mid_c-2, "-\xcf\x89", A_DIM, CP_DIM, rows, cols);

    /* map rad to plot cell */
    float theta_scale = (plot_w/2) / 0.6f;  /* 0.6 rad full half */
    float omega_scale = (plot_h/2) / 5.0f;  /* 5 rad/s full half */

    /* draw trail */
    int n = b->ph_fill;
    for (int i = 0; i < n; i++) {
        int idx = (b->ph_head - n + i + HIST_LEN) % HIST_LEN;
        float th = b->ph_theta[idx];
        float om = b->ph_omega[idx];
        int pc = mid_c + (int)(th * theta_scale);
        int pr = mid_r - (int)(om * omega_scale);
        float age = (float)i / (float)n;
        int cp = (age > 0.85f) ? CP_WARN :
                 (age > 0.5f)  ? CP_SPD  : CP_DIM;
        put_c(pr, pc, '.', A_NORMAL, cp, rows, cols);
    }

    /* current point (bright) */
    {
        int pc = mid_c + (int)(b->theta * theta_scale);
        int pr = mid_r - (int)(b->theta_dot * omega_scale);
        put_c(pr, pc, '@', A_BOLD, CP_WARN, rows, cols);
    }

    r = plot_r0 + plot_h;
    sep();

    /* regime classification */
    {
        float abs_err = fabsf(b->theta - b->theta_ref);
        const char *regime;
        int cp;
        if (!b->pid_on) {
            regime = "PID OFF  (free fall)"; cp = CP_WARN;
        } else if (b->kd < 1.f) {
            regime = "UNDERDAMPED (Kd too low)"; cp = CP_SPD;
        } else if (b->kd > 50.f) {
            regime = "OVERDAMPED (Kd too high)"; cp = CP_SPD;
        } else if (abs_err < 0.03f) {
            regime = "STABLE — converged"; cp = CP_GOOD;
        } else {
            regime = "SETTLING — correcting"; cp = CP_HUD;
        }
        put_s(r++,c0+1,regime,A_BOLD,cp,rows,cols);
    }

    snprintf(buf,sizeof buf," \xce\xb8=%+.3f  \xcf\x89=%+.3f ",
             b->theta, b->theta_dot);
    put_s(r++,c0,buf,A_NORMAL,CP_VAL,rows,cols);

    sep();

    /* active gain preset lesson */
    const GainPreset *pr_ = &PRESETS[b->preset_idx];
    snprintf(buf,sizeof buf," [%s]",pr_->name);
    put_s(r++,c0,buf,A_BOLD,CP_SPD,rows,cols);
    for(int i=0;i<3;i++)
        put_s(r++,c0+1,pr_->lesson[i],A_DIM,CP_DIM,rows,cols);

    sep();
    put_s(r++,c0," m:view  g:next preset  ",A_BOLD,CP_HUD,rows,cols);
}

/* ── §16  APP ────────────────────────────────────────────────────── */
int main(void) {
    struct sigaction sa = {0};
    sa.sa_handler = sig_handler;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGWINCH,&sa, NULL);

    perlin_init((unsigned int)time(NULL));

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Terrain terrain;
    terrain_init(&terrain, rows, cols);

    Bot bot;
    bot_init(&bot);

    int bot_sc = cols / 3;   /* bot stays at this screen column */

    long long frame_ns = 1000000000LL / FPS_TARGET;
    long long tick_ns  = 1000000000LL / bot.tick_hz;
    long long sim_acc  = 0;
    long long t_prev   = now_ns();
    long long fps_acc  = 0;
    int fps_cnt=0, fps_disp=0;

    while (running) {
        if (need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            bot_sc = cols / 3;
            terrain.rows = rows; terrain.cols = cols;
            need_resize = 0;
        }

        long long t_now = now_ns();
        long long dt_ns = t_now - t_prev;
        t_prev = t_now;
        if (dt_ns > 50000000LL) dt_ns = 50000000LL;

        terrain_ensure(&terrain, (int)(bot.world_x/CELL_W) + cols + 32);

        if (!bot.paused && !bot.fallen) {
            sim_acc += dt_ns;
            tick_ns  = 1000000000LL / bot.tick_hz;
            while (sim_acc >= tick_ns) {
                bot_tick(&bot, &terrain, (float)tick_ns * 1e-9f);
                sim_acc -= tick_ns;
            }
        }

        fps_acc += dt_ns; fps_cnt++;
        if (fps_acc >= 1000000000LL) {
            fps_disp = fps_cnt; fps_cnt = 0;
            fps_acc -= 1000000000LL;
        }

        long long el = now_ns() - t_now;
        long long sl = frame_ns - el;
        if (sl > 0) { struct timespec ts={0,sl}; nanosleep(&ts,NULL); }

        /* draw */
        getmaxyx(stdscr, rows, cols);
        erase();

        int wc = (int)(bot.world_x / CELL_W);
        draw_terrain(&terrain, wc, fmodf(bot.world_x,CELL_W), bot_sc, rows, cols);
        draw_bot(&bot, &terrain, bot_sc, rows, cols);

        /* right panel — active view */
        int c0 = cols - 32;
        if (c0 < 1) c0 = 1;

        /* view mode badge */
        {
            char badge[32];
            snprintf(badge,sizeof badge,"[%s]",VIEW_NAMES[bot.view]);
            put_s(rows-1, c0, badge, A_BOLD, CP_SPD, rows, cols);
        }

        switch (bot.view) {
        case VIEW_TELE:  draw_telemetry(&bot, rows, cols, fps_disp, c0); break;
        case VIEW_EQ:    draw_equations(&bot, rows, cols, c0);            break;
        case VIEW_PHASE: draw_phase    (&bot, rows, cols, c0);            break;
        default: break;
        }

        /* status bar */
        {
            float margin_deg = (FALL_ANGLE - fabsf(bot.theta_eff)) * 57.3f;
            char sb[80];
            snprintf(sb,sizeof sb,"  %s%s  \xce\xb1=%+.1f\xc2\xb0  margin=%.1f\xc2\xb0  dist=%.1fm",
                     bot.paused?"PAUSED ":"",
                     bot.pid_on?"":" NO-PID",
                     bot.alpha*57.3f, margin_deg, bot.dist_m);
            put_s(rows-1,0,sb,A_DIM,CP_DIM,rows,cols);
        }

        wnoutrefresh(stdscr); doupdate();

        /* input */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: running = 0; break;
            case ' ': bot.paused = !bot.paused; break;
            case 'p':
                bot.pid_on = !bot.pid_on;
                bot.pid_int = 0.f;
                break;
            case 'r':
                bot_reset(&bot);
                terrain_init(&terrain, rows, cols);
                break;
            case 'm':
                bot.view = (ViewMode)((bot.view + 1) % VIEW_COUNT);
                break;
            case 'g':
                bot.preset_idx = (bot.preset_idx + 1) % N_PRESETS;
                bot_apply_preset(&bot);
                break;
            case KEY_UP:
                bot.drive_spd += DRIVE_STEP;
                if (bot.drive_spd > DRIVE_MAX) bot.drive_spd = DRIVE_MAX;
                break;
            case KEY_DOWN:
                bot.drive_spd -= DRIVE_STEP;
                if (bot.drive_spd < 0.f) bot.drive_spd = 0.f;
                break;
            case '+': case '=':
                bot.kp += 5.f; if (bot.kp > 400.f) bot.kp = 400.f;
                break;
            case '-':
                bot.kp -= 5.f; if (bot.kp < 0.f) bot.kp = 0.f;
                break;
            case '[':
                bot.tick_hz -= 10; if (bot.tick_hz < 10) bot.tick_hz = 10;
                break;
            case ']':
                bot.tick_hz += 10; if (bot.tick_hz > 600) bot.tick_hz = 600;
                break;
            }
        }
    }

    endwin();
    return 0;
}
