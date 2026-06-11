/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * particle_number_morph.c — Solid Particle Number Morphing (LERP)
 *
 * Up to 500 particles densely fill every pixel of a large bitmap font,
 * cycling digits 0 → 1 → 2 → … → 9 → 0.
 *
 * On each digit change a greedy nearest-neighbour match assigns every
 * particle the closest target in the new digit.  Positions are then
 * linearly interpolated (smoothstep easing) from their snapshot at the
 * moment of change to the new target — no spring forces, no velocity.
 * The result is a clean, deterministic glide between shapes.
 *
 * Idle particles (excess over the new digit's count) glide back to the
 * digit centre and become invisible, then rejoin the swarm on future
 * digits that need more particles.
 *
 * Font: 9-row × 7-col bitmap, each '#' expanded to a sub-grid of
 * particles scaled to the terminal (up to 3 rows × 4 cols per pixel).
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   n         next digit immediately
 *   ] / [     hold time longer / shorter
 *   f / F     morph faster / slower
 *   t / T     next / prev theme  (Neon/Fire/Ice/Plasma/Mono)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_number_morph.c \
 *       -o particle_number_morph -lncurses -lm
 *
 * §1  config & font     §2  performance    §3  logic
 * §4  simulation        §5  render          §6  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Greedy nearest-neighbour bipartite matching + LERP morph.
 *                  On digit change: build target set for new digit; for each
 *                  particle, find closest unassigned target (O(P·T) greedy
 *                  scan); then linearly interpolate position from snapshot
 *                  to target over MORPH_FRAMES frames using smoothstep easing.
 *
 * Math           : Smoothstep easing: f(t) = 3t² − 2t³, t ∈ [0,1].
 *                  Produces S-curve acceleration at start and deceleration at
 *                  end — more visually pleasing than linear interpolation.
 *                  Bitmap font: 9-row × 7-col bitmap; each '#' pixel expanded
 *                  to a sub-grid of particles scaled to terminal dimensions.
 *
 * Performance    : Greedy matching is O(P·T) per digit change but only runs
 *                  once per transition (not per frame).  Per-frame cost is
 *                  O(P) position update + draw — cheap at P≤500.
 *
 * Rendering      : Particle colour based on proximity to target: settled
 *                  particles show theme colour; particles in flight fade
 *                  through a gradient based on interpolation progress t.
 *
 * References     :
 *   Particle→target matching & morph (§4 digit_assign, parts_update)
 *     [1] Kuhn, "The Hungarian Method for the Assignment Problem," Naval
 *         Research Logistics Quarterly 2 (1955) — the optimal particle↔target
 *         assignment; this file uses the cheap O(P·T) greedy approximation.
 *     [2] Reeves, "Particle Systems — A Technique for Modeling a Class of
 *         Fuzzy Objects," ACM TOG 2(2) (1983) — the pool-of-points model whose
 *         collective motion forms the digit.
 *   Easing (§3 smoothstep)
 *     [3] Penner, "Robert Penner's Easing Functions" (2002) — the S-curve
 *         ease-in/ease-out used to glide particles between shapes.
 *     [4] Perlin, "Improving Noise," ACM SIGGRAPH (2002) — the cubic Hermite
 *         smoothstep 3t²−2t³ (and its smootherstep cousin) behind the easing.
 *   ASCII rendering substrate (§5 parts_draw)
 *     [5] Bourke, "Character representation of grey scale images" (1997) — the
 *         progress → glyph ramp ('.' → '+' → '#' → '@').
 *     [6] Padala, "NCURSES Programming HOWTO" (TLDP) — colour pairs and the
 *         erase → draw → refresh frame model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * No physics, no springs. Each digit transition is a one-shot global
 * problem: "given 500 particles in their current poses, snap a snapshot,
 * pair every particle with the nearest unclaimed pixel of the new digit,
 * then slide everyone there in lockstep over MORPH_FRAMES frames using a
 * smoothstep curve." Once the snapshot is taken, particles are just
 * interpolating along straight lines — predictable, jitter-free, fast.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of it as a camera-track shot rather than a flock. At t=0 the
 * camera (the global lerp parameter) is at 0% and every particle stands at
 * its old pose. By t=1 every particle is parked exactly on a target pixel
 * of the new digit. Smoothstep gives the shot a soft start and soft end so
 * eyes follow comfortably. Idle particles (excess from a sparser digit
 * like "1") are sent to the digit centre and fade out together.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Precompute targets: for each digit 0..9, walk the 9×7 bitmap, expand
 *     every '#' pixel into a sub-grid of g_suby × g_subx points, store in
 *     g_targets[d].x/.y[0..g_targets[d].n-1].
 *  2. On digit change (digit_assign):
 *       a. Snapshot every particle's current (x,y) into (ox,oy).
 *       b. For each of n=g_targets[digit].n targets, scan all unclaimed
 *          particles, pick the one with smallest squared distance, mark
 *          claimed and write target.
 *       c. Unclaimed particles are sent to (cx,cy) and marked inactive.
 *       d. Reset g_morph_t = 0.
 *  3. Per frame (parts_update): t += 1/morph_frames, st = 3t²−2t³,
 *     x = ox + st·(tx − ox).
 *  4. Render: glyph escalates with st (.→+→#→@); inactive particles fade
 *     out once g_morph_t hits 1.
 *  5. Hold timer increments only after the morph completes; on overflow
 *     trigger digit_assign(next).
 *
 * KEY FORMULAS
 * ────────────
 *  smoothstep(t) = 3t² − 2t³        S-curve ease, derivatives 0 at 0 and 1
 *  x(t) = ox + smoothstep(t)·(tx−ox)  per-axis lerp from snapshot to target
 *  greedy match cost: argmin_p ‖p − target_t‖²    O(P·T) per transition
 *  digit width  = FONT_C × g_sx     pixels in cells
 *  particle target count: g_targets[d].n = (#-pixels) × g_suby × g_subx
 *
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
 *   LOGIC        §3       nothing — pure smoothstep (eased t)
 *   SIMULATION   §4       scene geometry (g_rows/g_cols, the scale & sub-grid
 *                           sizes), the digit target tables (g_targets[]),
 *                           the particle pool
 *                           g_parts[] and the morph clock g_morph_t; advances
 *                           the PRNG g_lcg.
 *   RENDER       §5       the terminal + colour pairs only; READS the particles,
 *                           never writes them.
 *
 *   EFFECTS  : ABSENT — a particle's glyph ('.'→'+'→'#'→'@') and the idle fade
 *              are derived at render time from the morph progress
 *              (smoothstep(g_morph_t)), never stored as cosmetic state.
 *   DELAYS   : the morph clock g_morph_t and the post-morph hold timer
 *              (hold_tick → hold_max) plus the pause flag are inline counters in
 *              §4 / §6, not a module.
 *
 * LOGIC (§3) is provably uncorruptable from RENDER: it does no mutation and no
 * I/O, so deleting or reordering any draw cannot change smoothstep.
 *
 * Per-tick combine order — main() (§6) is the only place that advances state:
 *     1. PERFORMANCE  frame_start timestamp                       §6
 *     2. SIMULATION   parts_update()  (lerp)  [skipped if paused]  §4
 *     3. DELAYS       after the morph completes, hold_tick++ →
 *                       digit_assign(next) when it overflows        §6 + §4
 *     4. RENDER       parts_draw + hud_draw                        §5
 *     5. PERFORMANCE  fps tally + sleep to RENDER_NS               §6
 *
 * User events (keys: quit/pause/next/hold/speed/theme; SIGWINCH resize) mutate
 * state but are NOT part of the tick — handled before it (§6 input drain and the
 * g_need_resize block, which re-run scale_compute / targets_precompute and
 * digit_assign).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  config & font                                                      */
/* ===================================================================== */

#define RENDER_FPS  30
#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* Bitmap font — '#' = filled, ' ' = empty.  9 rows × 7 cols. */
#define FONT_R  9
#define FONT_C  7

static const char k_font[10][FONT_R][FONT_C + 1] = {
    { /* 0 */  " ##### ", "##   ##", "##   ##", "##   ##",
               "##   ##", "##   ##", "##   ##", "##   ##", " ##### " },
    { /* 1 */  "   ##  ", "  ###  ", "   ##  ", "   ##  ",
               "   ##  ", "   ##  ", "   ##  ", "   ##  ", " ##### " },
    { /* 2 */  " ##### ", "##   ##", "     ##", "     ##",
               "  #### ", " ###   ", "###    ", "##     ", "#######" },
    { /* 3 */  " ##### ", "##   ##", "     ##", "     ##",
               "  #### ", "     ##", "     ##", "##   ##", " ##### " },
    { /* 4 */  "##   ##", "##   ##", "##   ##", "##   ##",
               "#######", "     ##", "     ##", "     ##", "     ##" },
    { /* 5 */  "#######", "##     ", "##     ", "###### ",
               "     ##", "     ##", "     ##", "##   ##", " ##### " },
    { /* 6 */  " ##### ", "##   ##", "##     ", "##     ",
               "###### ", "##   ##", "##   ##", "##   ##", " ##### " },
    { /* 7 */  "#######", "##   ##", "     ##", "    ## ",
               "    ## ", "   ##  ", "   ##  ", "  ##   ", "  ##   " },
    { /* 8 */  " ##### ", "##   ##", "##   ##", "##   ##",
               " ##### ", "##   ##", "##   ##", "##   ##", " ##### " },
    { /* 9 */  " ##### ", "##   ##", "##   ##", "##   ##",
               " ######", "     ##", "     ##", "##   ##", " ##### " },
};

/* Particle pool — digit 8 has 39 '#' pixels, max sub = 3×4 = 12 → 468 < 500 */
#define N_PARTS     500
#define SUB_Y_MAX   3
#define SUB_X_MAX   4

/* Lerp timing */
#define MORPH_FRAMES_MIN   10   /* ~0.33 s */
#define MORPH_FRAMES_DEF   40   /* ~1.33 s */
#define MORPH_FRAMES_MAX  120   /* ~4 s    */
#define HOLD_MIN    20
#define HOLD_DEF    60    /* frames to hold after morph completes */
#define HOLD_MAX   200

/* Layout / scatter */
#define FONT_HEIGHT_FRAC 0.65f  /* digit fills this fraction of screen height */
#define SCATTER_SPAN     1.5f   /* initial scatter box = this × digit box     */

/* Render glyph ramp by morph progress st (parts_draw) */
#define GLYPH_SETTLED    0.92f  /* st > this: '@' settled                     */
#define GLYPH_NEAR       0.55f  /* st > this: '#' nearly there                */
#define GLYPH_MID        0.20f  /* st > this: '+' mid-flight; else '.'        */

#define N_THEMES  5

/* Color pairs */
enum {
    CP_HUD  = 1,    /* top HUD: data readout (yellow)   */
    CP_D0   = 2,    /* CP_D0 + digit → per-digit color  */
    CP_IDLE = 12,   /* in-transit idle particles        */
    CP_HINT = 14,   /* bottom HUD: action keys (cyan)    */
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
/* §3  logic — pure decisions: no mutation, no I/O                        */
/* ===================================================================== */

/* Smoothstep easing: starts and ends gently, fast through the middle */
static float smoothstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* ===================================================================== */
/* §4  simulation — advances state (geometry, targets, particles, morph)  */
/* ===================================================================== */

/* PRNG — mutates g_lcg, so NOT pure (not §3 logic). */
static uint32_t g_lcg = 12345u;
static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1u << 24);
}

/* Scene geometry, derived from the terminal at init/resize. */
static int g_rows, g_cols;
static int g_sy, g_sx;
static int g_suby, g_subx;

static void scale_compute(void)
{
    g_sy = (int)((float)g_rows * FONT_HEIGHT_FRAC / (float)FONT_R);
    if (g_sy < 1) g_sy = 1;
    if (g_sy > 8) g_sy = 8;
    g_sx = g_sy * 2;
    if (g_sx > 16) g_sx = 16;
    g_suby = (g_sy < SUB_Y_MAX) ? g_sy : SUB_Y_MAX;
    g_subx = (g_sx < SUB_X_MAX) ? g_sx : SUB_X_MAX;
}

/* DigitTargets — the set of target pixel positions for one digit: n points at
 * (x[i], y[i]).  Precomputed once per scale by expanding the bitmap font;
 * digit_assign matches particles onto these points.  Struct-of-arrays, one per
 * digit 0..9. */
typedef struct {
    float x[N_PARTS];       /* target x per point                          */
    float y[N_PARTS];       /* target y per point                          */
    int   n;                /* valid point count (#pixels × sub-grid size) */
} DigitTargets;

static DigitTargets g_targets[10];

static void targets_precompute(float cx, float cy)
{
    float orig_x = cx - (float)(FONT_C * g_sx) * 0.5f;
    float orig_y = cy - (float)(FONT_R * g_sy) * 0.5f;
    float step_x = (g_subx > 1) ? (float)g_sx / (float)g_subx : (float)g_sx * 0.5f;
    float step_y = (g_suby > 1) ? (float)g_sy / (float)g_suby : (float)g_sy * 0.5f;
    float off_x  = step_x * 0.5f;
    float off_y  = step_y * 0.5f;

    for (int d = 0; d < 10; d++) {
        int n = 0;
        for (int fr = 0; fr < FONT_R; fr++) {
            for (int fc = 0; fc < FONT_C; fc++) {
                if (k_font[d][fr][fc] != '#') continue;
                float bx = orig_x + (float)(fc * g_sx);
                float by = orig_y + (float)(fr * g_sy);
                for (int py = 0; py < g_suby && n < N_PARTS; py++) {
                    for (int px = 0; px < g_subx && n < N_PARTS; px++) {
                        g_targets[d].x[n] = bx + off_x + (float)px * step_x;
                        g_targets[d].y[n] = by + off_y + (float)py * step_y;
                        n++;
                    }
                }
            }
        }
        g_targets[d].n = n;
    }
}

/*
 * Each particle stores:
 *   ox, oy — position snapshot taken the moment a new digit is triggered
 *   tx, ty — target for this digit (or centre for idle particles)
 *   x,  y  — current position = lerp(o, t, smoothstep(morph_t))
 *   active — true = assigned to a digit target; false = gliding to centre
 */
typedef struct {
    float x,  y;    /* current position  */
    float ox, oy;   /* origin at morph start */
    float tx, ty;   /* target */
    bool  active;
} Particle;

static Particle g_parts[N_PARTS];
static float    g_morph_t     = 1.0f;   /* 0 = just triggered, 1 = done */
static int      g_morph_frames = MORPH_FRAMES_DEF;

static void parts_init(float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        g_parts[i].x  = cx + (lcg_f() - 0.5f) * (float)(FONT_C * g_sx) * SCATTER_SPAN;
        g_parts[i].y  = cy + (lcg_f() - 0.5f) * (float)(FONT_R * g_sy) * SCATTER_SPAN;
        g_parts[i].ox = g_parts[i].x;
        g_parts[i].oy = g_parts[i].y;
        g_parts[i].tx = cx;
        g_parts[i].ty = cy;
        g_parts[i].active = false;
    }
}

/* Greedy nearest-neighbour: index of the closest particle not yet claimed by a
 * target (by squared distance), or -1 if none remain. */
static int nearest_unclaimed(float tx, float ty, const bool *used)
{
    float best_d2 = 1e18f;
    int   best_p  = -1;
    for (int p = 0; p < N_PARTS; p++) {
        if (used[p]) continue;
        float dx = g_parts[p].x - tx;
        float dy = g_parts[p].y - ty;
        float d2 = dx*dx + dy*dy;
        if (d2 < best_d2) { best_d2 = d2; best_p = p; }
    }
    return best_p;
}

/*
 * digit_assign — snapshot current positions, match particles to new
 * digit targets (greedy nearest-neighbour), send idle ones to centre,
 * then reset morph_t to 0 so the lerp restarts.
 */
static void digit_assign(int digit, float cx, float cy)
{
    const DigitTargets *tg = &g_targets[digit];
    bool  used[N_PARTS];
    memset(used, 0, sizeof used);

    /* Mark all inactive first */
    for (int i = 0; i < N_PARTS; i++) g_parts[i].active = false;

    /* Greedy match: each target claims its nearest free particle */
    for (int t = 0; t < tg->n; t++) {
        int p = nearest_unclaimed(tg->x[t], tg->y[t], used);
        if (p < 0) continue;
        used[p] = true;
        g_parts[p].active = true;
        g_parts[p].tx = tg->x[t];
        g_parts[p].ty = tg->y[t];
    }

    /* Unclaimed particles glide to centre; snapshot every origin for the lerp */
    for (int i = 0; i < N_PARTS; i++) {
        if (!used[i]) {
            g_parts[i].active = false;
            g_parts[i].tx = cx;
            g_parts[i].ty = cy;
        }
        g_parts[i].ox = g_parts[i].x;
        g_parts[i].oy = g_parts[i].y;
    }

    g_morph_t = 0.0f;   /* kick off lerp */
}

/*
 * parts_update — advance lerp by one frame.
 * All particles (active and idle) move together from their origin
 * snapshot toward their target using the same eased t value.
 */
static void parts_update(void)
{
    if (g_morph_t >= 1.0f) return;

    g_morph_t += 1.0f / (float)g_morph_frames;
    if (g_morph_t > 1.0f) g_morph_t = 1.0f;

    float st = smoothstep(g_morph_t);   /* eased t */

    for (int i = 0; i < N_PARTS; i++) {
        g_parts[i].x = g_parts[i].ox + st * (g_parts[i].tx - g_parts[i].ox);
        g_parts[i].y = g_parts[i].oy + st * (g_parts[i].ty - g_parts[i].oy);
    }
}

/* ===================================================================== */
/* §5  render — state → screen; reads only, never mutates sim state        */
/* ===================================================================== */

/* Theme — one named colour preset: a per-digit palette (dig[0..9]) plus the
 * colour for in-transit idle particles.  t/T cycle the presets. */
typedef struct {
    const char *name;       /* label shown in the HUD                     */
    short       dig[10];    /* foreground colour per digit 0..9           */
    short       idle;       /* idle (gliding-to-centre) particle colour   */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Neon",   { 51,231,226,208,46,201,196,117,255,220 }, 237 },
    { "Fire",   { 124,196,202,208,214,220,226,228,231,222 }, 52 },
    { "Ice",    { 17,21,27,33,39,44,51,87,123,159 }, 18 },
    { "Plasma", { 54,93,129,165,201,207,213,219,225,231 }, 53 },
    { "Mono",   { 255,255,255,255,255,255,255,255,255,255 }, 237 },
};

static void theme_apply(int t)
{
    for (int d = 0; d < 10; d++)
        init_pair((short)(CP_D0 + d),
                  (COLORS >= 256) ? k_themes[t].dig[d] : COLOR_WHITE,
                  COLOR_BLACK);
    init_pair(CP_IDLE, (COLORS >= 256) ? k_themes[t].idle : COLOR_BLACK, COLOR_BLACK);
}

static void colors_init(int theme)
{
    start_color();
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT, (COLORS >= 256) ? 51  : COLOR_CYAN,   COLOR_BLACK);
    theme_apply(theme);
}

/* Glyph for a settling particle by morph progress: dot → plus → hash → solid. */
static char morph_glyph(float st)
{
    if (st > GLYPH_SETTLED) return '@';
    if (st > GLYPH_NEAR)    return '#';
    if (st > GLYPH_MID)     return '+';
    return '.';
}

static void parts_draw(int digit)
{
    attr_t a_bright = (attr_t)COLOR_PAIR(CP_D0 + digit) | A_BOLD;
    attr_t a_idle   = (attr_t)COLOR_PAIR(CP_IDLE);

    /* Character tracks lerp progress (same for all); colour is the digit colour */
    char glyph = morph_glyph(smoothstep(g_morph_t));

    for (int i = 0; i < N_PARTS; i++) {
        Particle *p = &g_parts[i];
        int col = (int)(p->x + 0.5f);
        int row = (int)(p->y + 0.5f);
        if (row < 0 || row >= g_rows - 1) continue;
        if (col < 0 || col >= g_cols)     continue;

        if (p->active) {
            attron(a_bright);
            mvaddch(row, col, (chtype)(unsigned char)glyph);
            attroff(a_bright);
        } else {
            /* Idle: visible only during the lerp, invisible once at centre */
            if (g_morph_t >= 1.0f) continue;
            attron(a_idle);
            mvaddch(row, col, '.');
            attroff(a_idle);
        }
    }
}

static void hud_draw(int digit, int hold_max, int theme, bool paused, double fps)
{
    /* Top-right: data readout */
    char buf[160];
    snprintf(buf, sizeof buf,
             " digit:%d  theme:%s  morph:%d  hold:%d  %.0f fps  %s ",
             digit, k_themes[theme].name, g_morph_frames, hold_max, fps,
             paused ? "PAUSED " : "running");
    int x = g_cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Bottom-left: action keys */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(g_rows - 1, 0,
             " q:quit  p:pause  n:next  ]/[:hold  f/F:speed  t/T:theme ");
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

static void do_resize(float *cx, float *cy)
{
    endwin(); refresh();
    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows < 5)  g_rows = 5;
    if (g_cols < 10) g_cols = 10;
    *cx = (float)g_cols * 0.5f;
    *cy = (float)g_rows * 0.5f;
    scale_compute();
    targets_precompute(*cx, *cy);
    g_need_resize = 0;
}

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
    float cx = (float)g_cols * 0.5f;
    float cy = (float)g_rows * 0.5f;
    scale_compute();
    targets_precompute(cx, cy);

    parts_init(cx, cy);

    int  cur_digit    = 0;
    int  hold_tick    = 0;
    int  hold_max     = HOLD_DEF;
    bool paused       = false;

    digit_assign(cur_digit, cx, cy);

    double  fps       = 0.0;
    int     frame_cnt = 0;
    int64_t fps_clock = clock_ns();

    while (g_running) {

        /* USER EVENT (out of tick): resize → re-derive geometry + targets */
        if (g_need_resize) {
            do_resize(&cx, &cy);
            digit_assign(cur_digit, cx, cy);
        }

        /* 1. PERFORMANCE — frame start timestamp */
        int64_t frame_start = clock_ns();

        /* USER EVENT (out of tick): input — quit/pause/next/hold/speed/theme */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': case ' ': paused = !paused; break;
            case 'n': case 'N':
                /* skip to next digit immediately */
                hold_tick = hold_max;
                g_morph_t = 1.0f;
                break;
            case ']':
                hold_max += 10;
                if (hold_max > HOLD_MAX) hold_max = HOLD_MAX;
                break;
            case '[':
                hold_max -= 10;
                if (hold_max < HOLD_MIN) hold_max = HOLD_MIN;
                break;
            case 'f':
                /* faster morph: fewer frames */
                g_morph_frames -= 5;
                if (g_morph_frames < MORPH_FRAMES_MIN) g_morph_frames = MORPH_FRAMES_MIN;
                break;
            case 'F':
                /* slower morph: more frames */
                g_morph_frames += 5;
                if (g_morph_frames > MORPH_FRAMES_MAX) g_morph_frames = MORPH_FRAMES_MAX;
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
            /* 2. SIMULATION — advance the lerp every frame */
            parts_update();

            /* 3. DELAYS — hold timer runs only after the morph completes */
            if (g_morph_t >= 1.0f) {
                hold_tick++;
                if (hold_tick >= hold_max) {
                    hold_tick = 0;
                    cur_digit = (cur_digit + 1) % 10;
                    digit_assign(cur_digit, cx, cy);
                }
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
