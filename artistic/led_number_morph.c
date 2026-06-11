/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * led_number_morph.c — Particle LED Number Morphing
 *
 * 336 particles form 7-segment-style digits 0 → 1 → 2 → … → 9 → 0.
 * The display is modelled as seven segments (top/mid/bot horizontal,
 * four vertical halves).  Particles belong permanently to one segment;
 * when the segment is active they spring to evenly-spaced targets on it,
 * when it goes dark they drift slowly back toward centre.
 *
 * Formed particles use orientation-aware characters: horizontal segments
 * render as '-' and vertical segments as '|', so the result looks like a
 * large LED display.  Active particles still in flight show '+'; passive
 * (idle) particles show '.' — so the lit number always stands out, even
 * though every particle shares one colour.
 *
 * Digit size scales with the terminal so it always fills ≈65% of height.
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   n         next digit immediately
 *   ] / [     morph faster / slower
 *   t / T     next / prev color theme  (Neon / Fire / Ice / Plasma / Mono)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra led_number_morph.c -o led_number_morph -lncurses -lm
 *
 * §1  config            §2  performance      §3  logic (7-seg geometry)
 * §4  simulation        §5  render           §6  app / main
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Spring-mass particle morphing — each particle has a
 *                  target position on the active digit's 7-segment layout.
 *                  Per tick: apply spring force F = k·(target − pos),
 *                  integrate velocity with damping, update position.
 *                  Particles assigned to inactive segments drift toward
 *                  segment centre (aggregation behaviour).
 *
 * Math           : 7-segment encoding: each digit 0–9 activates a subset
 *                  of 7 named segments (top, tl, tr, mid, bl, br, bot).
 *                  Segment lookup table maps digit → bitmask.  Particles are
 *                  permanently bound to one segment and distributed evenly
 *                  along its length when that segment is active.
 *
 * Physics        : Overdamped spring: critical damping chosen so particles
 *                  reach their targets smoothly without overshoot.
 *                  Damping coefficient b chosen near 2√(k·m) (critical
 *                  damping condition).
 *
 * Rendering      : Particles in horizontal segments drawn as '-', vertical
 *                  as '|'; active in-flight particles as '+', passive (idle)
 *                  particles as '.'.
 *                  Digit size scales with terminal height so the display fills
 *                  ~65% of available rows — recomputed on each resize.
 *
 * References     :
 *   Particle dynamics & spring-to-target morphing (§4 parts_update)
 *     [1] Reeves, "Particle Systems — A Technique for Modeling a Class of
 *         Fuzzy Objects," ACM TOG 2(2) (1983) — the foundational particle-
 *         system paper (a pool of independent points with per-step forces).
 *     [2] Witkin & Baraff, "Physically Based Modeling: Principles and
 *         Practice" (SIGGRAPH course notes) — the damped-spring force
 *         F = k·(target − x) − b·v and its explicit integration.
 *     [3] Lowe, "Critically Damped Ease-In/Ease-Out Smoothing," Game
 *         Programming Gems 4 (2004) — choosing b ≈ 2√(k·m) so particles reach
 *         their targets fast and smoothly without overshoot (DAMP vs SPRING_K).
 *   7-segment digit encoding (§3 k_segs / seg_targets)
 *     [4] Horowitz & Hill, "The Art of Electronics" — the standard A–G
 *         seven-segment layout and per-digit segment bitmasks.
 *   ASCII rendering substrate (§5 parts_draw)
 *     [5] Bourke, "Character representation of grey scale images" (1997) — the
 *         glyph-by-intensity idea behind the '-'/'|' > '+' > '.' tiering.
 *     [6] Padala, "NCURSES Programming HOWTO" (TLDP) — colour pairs and the
 *         erase → draw → refresh frame model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * Four real layers; two are intentionally absent.
 *
 *   LAYER        SECTION  MUTATES
 *   ────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — clock primitives (frame cap + fps in §6)
 *   LOGIC        §3       nothing — pure: seg_targets (digit/segment → target
 *                           xy) and the k_segs digit table.  Reads the scene
 *                           geometry, writes only caller-provided arrays.
 *   SIMULATION   §4       g_parts[] (pos/vel/target/active) and the scene
 *                           geometry g_dh/g_dw (dig_size, at init/resize);
 *                           advances the PRNG g_lcg.
 *   RENDER       §5       the terminal + colour pairs only; reads particles /
 *                           geometry, never writes them.
 *
 *   EFFECTS  : ABSENT — a particle's glyph and brightness are derived at render
 *              time from its distance-to-target / speed (parts_draw), never
 *              stored as cosmetic state.
 *   DELAYS   : the digit HOLD timer (hold_tick → hold_max) and the pause flag
 *              are inline counters in the main loop (§6), not a module; there
 *              are no other timers.
 *
 * LOGIC (§3) is provably uncorruptable from RENDER: it does no mutation and no
 * I/O, so deleting or reordering any draw cannot change seg_targets.
 *
 * Per-tick combine order — main() (§6) is the only place that advances state:
 *     1. PERFORMANCE  measure frame_start                          §6
 *     2. SIMULATION   parts_update(fixed dt)   [skipped if paused]  §4
 *     3. DELAYS       hold_tick++ → digit_assign(next) at hold_max  §6 + §4
 *     4. RENDER       parts_draw + hud_draw                         §5
 *     5. PERFORMANCE  fps tally + sleep to RENDER_NS                §6
 *
 * User events (keys: quit/pause/next/speed/theme; SIGWINCH resize) mutate
 * state but are NOT part of the tick — handled before it (§6 input drain and
 * the resize block, which re-run dig_size + digit_assign).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define RENDER_FPS  30
#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/*
 * Scene geometry (set by dig_size in §4 at init/resize; read by §3 logic and
 * §5 render).  g_dh = digit height (rows), g_dw = digit width (cols).
 * 7-segment ratio: physical_width / physical_height ≈ 0.55
 * Terminal cell is ~2× taller than wide, so:
 *   g_dw ≈ g_dh × 0.55 × 2  ≈  g_dh × 1.1
 */
static int g_rows, g_cols;
static int g_dh, g_dw;   /* digit bounding box in terminal cells */

/* Segments A-G: A=top, B=top-right, C=bot-right, D=bot,
 *              E=bot-left, F=top-left, G=middle */
enum { SA=0, SB, SC, SD, SE, SF, SG, N_SEGS };

/* Segment orientation: 0=horizontal, 1=vertical */
static const int k_seg_horiz[N_SEGS] = { 1,0,0,1,0,0,1 };

/* Characters for each orientation at different phases */
static const char k_ch_h = '-';    /* formed horizontal segment */
static const char k_ch_v = '|';    /* formed vertical segment   */

#define N_PER_SEG  48                      /* particles per segment — dense enough
                                            * that even the long top/mid/bot bars
                                            * render solid, not dotted */
#define N_PARTS   (N_SEGS * N_PER_SEG)    /* 336 total */

/* Spring physics constants */
#define SPRING_K   9.0f   /* stiffness — active particles   */
#define DAMP       5.5f   /* damping   — active             */
#define DRIFT_K    0.3f   /* stiffness — idle drift to centre */
#define DRIFT_DAMP 1.5f   /* damping   — idle               */
#define JITTER     0.06f  /* random nudge for idle particles */

/* Initial scatter (parts_init) */
#define PARTS_SCATTER_SPAN 1.5f  /* initial scatter box = this × digit box      */
#define PARTS_INIT_SPEED   1.5f  /* initial random velocity range (cells/frame) */

/* Render glyph thresholds (parts_draw) — squared distance / speed in cells² */
#define ARRIVED_D2      0.64f    /* dist² below which an active particle is "formed" */
#define FLYING_NEAR_D2  6.0f     /* dist² below which an in-flight particle is bold  */
#define IDLE_VISIBLE_V2 0.04f    /* speed² above which an idle particle is drawn     */

/* Timing */
#define HOLD_MIN   35     /* min ticks before digit advance */
#define HOLD_DEF  100     /* default (~3.3 s at 30 fps)     */
#define HOLD_MAX  270

#define N_THEMES   5

/* Color pair IDs */
enum {
    CP_HUD  = 1,    /* top HUD: data readout (yellow)               */
    CP_D0   = 2,    /* CP_D0 + digit (0-9) → particle color         */
    CP_IDLE = 12,   /* inactive particles (same theme colour)       */
    CP_MOV  = 13,   /* in-flight active particles                   */
    CP_HINT = 14,   /* bottom HUD: action keys (cyan)               */
};

/* ===================================================================== */
/* §2  performance — timing primitives (frame cap applied in §6)          */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + (int64_t)ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  logic — pure 7-segment geometry: no mutation, no I/O               */
/* ===================================================================== */

/*
 * 7-segment encoding:  bit 0=A 1=B 2=C 3=D 4=E 5=F 6=G
 */
static const uint8_t k_segs[10] = {
    0x3F, /* 0: ABCDEF  */
    0x06, /* 1: BC      */
    0x5B, /* 2: ABDEG   */
    0x4F, /* 3: ABCDG   */
    0x66, /* 4: BCFG    */
    0x6D, /* 5: ACDFG   */
    0x7D, /* 6: ACDEFG  */
    0x07, /* 7: ABC     */
    0x7F, /* 8: ABCDEFG */
    0x6F, /* 9: ABCDFG  */
};

/*
 * seg_endpoints — the two end coordinates of segment s for a digit centred at
 * (cx, cy), within a g_dw × g_dh box.  The ±1 offsets leave a 1-cell corner gap
 * so adjacent segments don't overlap at junctions.
 */
static void seg_endpoints(int s, float cx, float cy,
                          float *x0, float *y0, float *x1, float *y1)
{
    float hw = g_dw * 0.5f;
    float hh = g_dh * 0.5f;
    switch (s) {
    case SA: /* top horizontal */
        *x0 = cx-hw+1; *y0 = cy-hh;   *x1 = cx+hw-1; *y1 = cy-hh;   break;
    case SB: /* top-right vertical */
        *x0 = cx+hw;   *y0 = cy-hh+1; *x1 = cx+hw;   *y1 = cy-1;    break;
    case SC: /* bottom-right vertical */
        *x0 = cx+hw;   *y0 = cy+1;    *x1 = cx+hw;   *y1 = cy+hh-1; break;
    case SD: /* bottom horizontal */
        *x0 = cx-hw+1; *y0 = cy+hh;   *x1 = cx+hw-1; *y1 = cy+hh;   break;
    case SE: /* bottom-left vertical */
        *x0 = cx-hw;   *y0 = cy+1;    *x1 = cx-hw;   *y1 = cy+hh-1; break;
    case SF: /* top-left vertical */
        *x0 = cx-hw;   *y0 = cy-hh+1; *x1 = cx-hw;   *y1 = cy-1;    break;
    case SG: /* middle horizontal */
        *x0 = cx-hw+1; *y0 = cy;      *x1 = cx+hw-1; *y1 = cy;      break;
    default:
        *x0 = cx; *y0 = cy; *x1 = cx; *y1 = cy; break;
    }
}

/*
 * seg_targets — fill tx[],ty[] with n evenly-spaced positions along segment s
 * (linear interpolation between the segment's two endpoints).
 */
static void seg_targets(int s, float cx, float cy,
                         float *tx, float *ty, int n)
{
    float x0, y0, x1, y1;
    seg_endpoints(s, cx, cy, &x0, &y0, &x1, &y1);
    for (int i = 0; i < n; i++) {
        float t = (n > 1) ? (float)i / (float)(n - 1) : 0.5f;  /* fraction along seg */
        tx[i] = x0 + t * (x1 - x0);
        ty[i] = y0 + t * (y1 - y0);
    }
}

/* ===================================================================== */
/* §4  simulation — advances particle state + derives scene geometry      */
/* ===================================================================== */

/* PRNG — mutates g_lcg, so NOT pure (not §3 logic). */
static uint32_t g_lcg = 12345u;
static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1 << 24);
}

/* Scene-geometry setup (init / resize): derive the digit box from the
 * terminal size.  See the ratio note in §1. */
static void dig_size(void)
{
    g_dh = (int)(g_rows * 0.65f);
    if (g_dh < 8)  g_dh = 8;
    if (g_dh > 40) g_dh = 40;
    g_dw = (int)(g_dh * 1.1f);
    if (g_dw > g_cols - 4) g_dw = g_cols - 4;
    if (g_dw < 6) g_dw = 6;
}

/* Particle — one point mass in the morphing cloud; the digit is an EMERGENT
 * shape, never drawn directly.
 *
 * WHY a particle system (Reeves 1983, ref [1]): rather than draw each digit,
 * 336 independent points each chase a target and the LED shape emerges from
 * their collective settling.  Morphing 5 → 6 then needs no shape interpolation —
 * just hand every particle a new target and let the springs animate the change.
 *
 * WHY permanent segment binding: particle i belongs forever to segment
 * i / N_PER_SEG (implicit in its index, never stored).  This sidesteps the
 * assignment problem (which point goes to which new target?) — a particle only
 * ever springs to its OWN segment or drifts to centre, so successive digits
 * never fight over the same points and the motion stays coherent.
 *
 * PHYSICS (parts_update): an overdamped spring toward the target,
 *   v += (SPRING_K·(target − pos) − DAMP·v)·dt ,   pos += v·dt
 * with DAMP near-critical (≈ 2√(SPRING_K·m)) so it arrives fast without
 * overshoot or oscillation (refs [2][3]).  Idle particles use the gentler
 * DRIFT_K / DRIFT_DAMP pull toward centre plus a little JITTER.
 *
 * UNITS: x/y/tx/ty in terminal cells; vx/vy in cells per frame. */
typedef struct {
    float x, y;    /* position in terminal cell coordinates (float)             */
    float vx, vy;  /* velocity, cells per frame (integrated each tick)          */
    float tx, ty;  /* spring target on this particle's home segment (if active) */
    bool  active;  /* true  = home segment lit → spring to (tx,ty);
                    * false = segment dark → drift toward the digit centre       */
} Particle;

static Particle g_parts[N_PARTS];

static void parts_init(float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        /* Scatter randomly within the digit bounding box */
        g_parts[i].x  = cx + (lcg_f() - 0.5f) * (float)g_dw * PARTS_SCATTER_SPAN;
        g_parts[i].y  = cy + (lcg_f() - 0.5f) * (float)g_dh * PARTS_SCATTER_SPAN;
        g_parts[i].vx = (lcg_f() - 0.5f) * PARTS_INIT_SPEED;
        g_parts[i].vy = (lcg_f() - 0.5f) * PARTS_INIT_SPEED;
        g_parts[i].tx = cx;
        g_parts[i].ty = cy;
        g_parts[i].active = false;
    }
}

/*
 * digit_assign — update spring targets for a new digit.
 * Particle i always belongs to segment (i / N_PER_SEG) so particles
 * never need to cross between segments; they either go to their home
 * segment or drift to centre.
 */
static void digit_assign(int digit, float cx, float cy)
{
    uint8_t segs = k_segs[digit];
    for (int s = 0; s < N_SEGS; s++) {
        bool on = (segs >> s) & 1;
        float stx[N_PER_SEG], sty[N_PER_SEG];
        if (on) seg_targets(s, cx, cy, stx, sty, N_PER_SEG);
        for (int j = 0; j < N_PER_SEG; j++) {
            int i = s * N_PER_SEG + j;
            g_parts[i].active = on;
            if (on) { g_parts[i].tx = stx[j]; g_parts[i].ty = sty[j]; }
        }
    }
}

/* Overdamped spring pulling an active particle toward its segment target. */
static void particle_spring(Particle *p, float dt)
{
    float dx = p->tx - p->x, dy = p->ty - p->y;
    p->vx += (SPRING_K * dx - DAMP * p->vx) * dt;
    p->vy += (SPRING_K * dy - DAMP * p->vy) * dt;
}

/* Gentle pull of an idle particle toward the digit centre, plus turbulence. */
static void particle_drift(Particle *p, float cx, float cy, float dt)
{
    float dx = cx - p->x, dy = cy - p->y;
    p->vx += (DRIFT_K * dx - DRIFT_DAMP * p->vx) * dt;
    p->vy += (DRIFT_K * dy - DRIFT_DAMP * p->vy) * dt;
    p->vx += (lcg_f() - 0.5f) * JITTER;
    p->vy += (lcg_f() - 0.5f) * JITTER;
}

/* Explicit-Euler position update. */
static void particle_integrate(Particle *p, float dt)
{
    p->x += p->vx * dt;
    p->y += p->vy * dt;
}

static void parts_update(float dt, float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        Particle *p = &g_parts[i];
        if (p->active) particle_spring(p, dt);
        else           particle_drift(p, cx, cy, dt);
        particle_integrate(p, dt);
    }
}

/* ===================================================================== */
/* §5  render — state → screen; reads only, never mutates sim state        */
/* ===================================================================== */

/* Theme — one named colour preset for the whole display.
 *
 * WHY a single colour (not a per-digit gradient): with hundreds of overlapping
 * particles, multi-tier colours made the lit number hard to read.  One bright
 * hue keeps every particle visible, and the THREE roles are separated by GLYPH
 * + attribute instead of colour, so depth comes from intensity within one hue:
 *   formed digit  '-' / '|'   A_BOLD        (brightest)
 *   in flight     '+'         A_BOLD/normal (assembling)
 *   idle space    '.'         A_DIM         (faintest)
 * The t/T keys cycle the presets. */
typedef struct {
    const char *name;      /* label shown in the HUD                           */
    short       color;     /* single bright xterm-256 fg for the whole display */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Neon",   51  },     /* bright cyan     */
    { "Fire",   208 },     /* bright orange   */
    { "Ice",    117 },     /* bright sky-blue */
    { "Plasma", 201 },     /* bright magenta  */
    { "Mono",   231 },     /* white           */
};

static void theme_apply(int t)
{
    short c = (COLORS >= 256) ? k_themes[t].color : COLOR_WHITE;
    for (int d = 0; d < 10; d++)
        init_pair((short)(CP_D0 + d), c, COLOR_BLACK);
    init_pair(CP_IDLE, c, COLOR_BLACK);
    init_pair(CP_MOV,  c, COLOR_BLACK);
}

static void colors_init(int theme)
{
    start_color();
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT, (COLORS >= 256) ? 51  : COLOR_CYAN,   COLOR_BLACK);
    theme_apply(theme);
}

/* Draw one particle: its glyph + brightness encode its role (formed segment bar,
 * in-flight '+', or faint idle '.') — colour is the single theme hue (cpair for
 * formed, CP_MOV in flight, CP_IDLE idle).  i selects the home-segment glyph. */
static void draw_particle(const Particle *p, int i, int cpair)
{
    int col = (int)(p->x + 0.5f);
    int row = (int)(p->y + 0.5f);
    if (row < 0 || row >= g_rows - 1) return;
    if (col < 0 || col >= g_cols) return;

    if (p->active) {
        float dx = p->tx - p->x, dy = p->ty - p->y;
        float dist2 = dx*dx + dy*dy;          /* squared distance to target */
        chtype ch;
        attr_t attr;
        if (dist2 < ARRIVED_D2) {
            /* Arrived — use segment-appropriate char */
            int seg = i / N_PER_SEG;
            ch   = (chtype)(unsigned char)(k_seg_horiz[seg] ? k_ch_h : k_ch_v);
            attr = (attr_t)COLOR_PAIR(cpair) | A_BOLD;
        } else {
            /* Active but still in flight — always '+' (never '.'), so an active
             * particle is never confused with a passive one. */
            ch   = '+';
            attr = (attr_t)COLOR_PAIR(CP_MOV) | (dist2 < FLYING_NEAR_D2 ? A_BOLD : 0);
        }
        attron(attr);
        mvaddch(row, col, ch);
        attroff(attr);
    } else {
        /* Passive (idle) negative space — only visible while drifting, and dimmed
         * (same theme hue, A_DIM) so the lit digit stands out. */
        float spd2 = p->vx*p->vx + p->vy*p->vy;   /* squared speed */
        if (spd2 < IDLE_VISIBLE_V2) return;
        attron(COLOR_PAIR(CP_IDLE) | A_DIM);
        mvaddch(row, col, '.');
        attroff(COLOR_PAIR(CP_IDLE) | A_DIM);
    }
}

static void parts_draw(int digit)
{
    int cpair = CP_D0 + digit;
    for (int i = 0; i < N_PARTS; i++)
        draw_particle(&g_parts[i], i, cpair);
}

static void hud_draw(int digit, int hold_max, int theme, bool paused, double fps)
{
    /* Top-right: data readout */
    char buf[160];
    snprintf(buf, sizeof buf,
             " digit:%d  theme:%s  hold:%d  %.0f fps  %s ",
             digit, k_themes[theme].name, hold_max, fps,
             paused ? "PAUSED " : "running");
    int x = g_cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Bottom-left: action keys */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(g_rows - 1, 0, " q:quit  p:pause  n:next  ]/[:speed  t/T:theme ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §6  app / main — combine point + user events + frame cap               */
/* ===================================================================== */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_exit(int s)   { (void)s; g_running     = 0; }
static void sig_resize(int s) { (void)s; g_need_resize = 1; }
static void cleanup(void)     { endwin(); }

int main(void)
{
    g_lcg = (uint32_t)clock_ns();

    atexit(cleanup);
    signal(SIGINT,   sig_exit);
    signal(SIGTERM,  sig_exit);
    signal(SIGWINCH, sig_resize);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);

    int theme = 0;
    colors_init(theme);

    getmaxyx(stdscr, g_rows, g_cols);
    dig_size();

    float cx = (float)g_cols * 0.5f;
    float cy = (float)g_rows * 0.5f;

    parts_init(cx, cy);

    int  cur_digit  = 0;
    int  hold_tick  = 0;
    int  hold_max   = HOLD_DEF;
    bool paused     = false;

    digit_assign(cur_digit, cx, cy);

    double  fps       = 0.0;
    int     frame_cnt = 0;
    int64_t fps_clock = clock_ns();

    while (g_running) {

        /* USER EVENT (out of tick): resize → re-derive geometry + targets */
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            dig_size();
            cx = (float)g_cols * 0.5f;
            cy = (float)g_rows * 0.5f;
            digit_assign(cur_digit, cx, cy);
        }

        /* 1. PERFORMANCE — frame start timestamp */
        int64_t frame_start = clock_ns();

        /* USER EVENT (out of tick): input — quit/pause/next/speed/theme */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': case ' ': paused = !paused; break;
            case 'n': case 'N':
                hold_tick = hold_max;   /* force advance on next tick */
                break;
            case ']':
                hold_max -= 10;
                if (hold_max < HOLD_MIN) hold_max = HOLD_MIN;
                break;
            case '[':
                hold_max += 10;
                if (hold_max > HOLD_MAX) hold_max = HOLD_MAX;
                break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                theme_apply(theme);
                break;
            case 'T':
                theme = (theme + N_THEMES - 1) % N_THEMES;
                theme_apply(theme);
                break;
            default: break;
            }
        }

        if (!paused) {
            /* 2. SIMULATION — physics with fixed dt = 1/FPS */
            parts_update(1.0f / (float)RENDER_FPS, cx, cy);

            /* 3. DELAYS — digit hold timer; advance when it expires */
            hold_tick++;
            if (hold_tick >= hold_max) {
                hold_tick  = 0;
                cur_digit  = (cur_digit + 1) % 10;
                digit_assign(cur_digit, cx, cy);
            }
        }

        /* 4. RENDER — read-only */
        erase();
        parts_draw(cur_digit);
        hud_draw(cur_digit, hold_max, theme, paused, fps);
        wnoutrefresh(stdscr);
        doupdate();

        /* 5. PERFORMANCE — fps tally + sleep to the frame cap */
        frame_cnt++;
        int64_t now = clock_ns();
        if (now - fps_clock >= 500LL * NS_PER_MS) {
            fps       = (double)frame_cnt
                      / ((double)(now - fps_clock) / (double)NS_PER_SEC);
            frame_cnt = 0;
            fps_clock = now;
        }

        clock_sleep_ns(RENDER_NS - (clock_ns() - frame_start));
    }

    return 0;
}
