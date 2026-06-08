/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* dune_sandworm.c — Arrakis sandworm: swim underground, breach, open mouth, dive
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/dune_sandworm.c -o dune_sandworm -lncurses -lm
 *
 * Keys: q quit | p pause | r reset | +/- speed | Space breach | w/W worms | t theme
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Segmented worm body using a chain-of-circles (Conga)
 *                  follower model.  Each segment follows the previous:
 *                    if dist(seg[i], seg[i-1]) > SEG_LEN, move seg[i]
 *                    toward seg[i-1] until separation == SEG_LEN.
 *                  The head drives; the body follows with O(N_SEGS) updates.
 *
 * Math           : Underground path: sinusoidal oscillation
 *                    y = surface_row + SWIM_DEPTH + SWIM_AMP·sin(SWIM_FREQ·x)
 *                  Breach arc: parabolic trajectory above the surface —
 *                    apex at BREACH_HEIGHT rows, spanning BREACH_SPAN cols.
 *                  Terrain surface: procedural height map built from a sum
 *                  of low-frequency cosine waves to mimic sand dunes.
 *
 * Rendering      : Body segments drawn with direction-dependent ASCII glyphs
 *                  (| ! vertical, O o 0 horizontal, \ / diagonal; see seg_char).
 *                  Head has a multi-row open-mouth sprite during breach.  Sand
 *                  ripples expand radially at RIPPLE_SPEED cols/s after a breach.
 *
 * References     :
 *   Body chain / follower (the segment distance constraint in relax_chain):
 *   [1] Jakobsen, "Advanced Character Physics" (GDC 2001) — Verlet + stick
 *       constraints; relaxing each segment to SEG_LEN is exactly his distance
 *       stick with the leading endpoint pinned (the chain loop here).
 *   [2] Müller, Heidelberger, Hennix & Ratcliff, "Position Based Dynamics"
 *       (J. Vis. Commun. & Image Repr. 2007) — formal framework for the
 *       per-segment distance projection that keeps spacing == SEG_LEN.
 *   [3] Aristidou & Lasenby, "FABRIK: A fast, iterative solver for the
 *       Inverse Kinematics problem" (Graphical Models 2011) — the
 *       follow-the-leader chain-of-points this Conga body is built on.
 *
 *   Procedural terrain & ballistic motion:
 *   [4] Perlin, "An Image Synthesizer" (SIGGRAPH 1985) — gradient noise; the
 *       dune surface is its cheap cousin, a sum of low-frequency sines.
 *   [5] Ebert, Musgrave, Peachey, Perlin & Worley, "Texturing & Modeling:
 *       A Procedural Approach" (3rd ed. 2003) — fBm / sum-of-octaves terrain.
 *   [6] Witkin & Baraff, "Physically Based Modeling" (SIGGRAPH course notes)
 *       — explicit Euler for the sand-spray particles; the breach arc is the
 *       same projectile integration written in closed form (parabola in t).
 *
 *   Particles & ASCII rendering:
 *   [7] Reeves, "Particle Systems — A Technique for Modeling a Class of Fuzzy
 *       Objects" (ACM TOG 1983) — origin of the spray / ripple particle model.
 *   [8] Bourke, "Character representation of grey scale images" (1997) —
 *       luminance->ASCII ramp behind the intensity / glyph choices.
 *   [9] Padala, "NCURSES Programming HOWTO" — color pairs and erase/doupdate
 *       double-buffering used in the §8 draw layer.
 *
 * ─────────────────────────────────────────────────────────────────────── */


/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into LAYERS by concern. All simulation DATA hangs off one
 * Scene aggregate (§3); functions take the NARROWEST slice they need —
 * const DomainType* for a pure read, DomainType* for a mutator — so the layers
 * never re-couple. Only the whole-tick orchestrators (scene_reset / scene_tick)
 * take Scene*.
 *
 *   Layer        Section            Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2 timing          nothing (reads OS clock, sleeps)
 *   DATA         §3 data            — types + the Scene aggregate —
 *   LOGIC        §4 logic           nothing (pure reads: returns values)
 *   EFFECTS      §5 effects         scene.ripples, scene.spray (cosmetic)
 *   SIMULATION   §6 simulation      scene.worms (and spawns into EFFECTS)
 *   INIT/RESET   §7 init            ALL of Scene
 *   RENDER       §8 render          the screen only — never Scene
 *   EVENTS       §9 events          scene.{paused,speed,n_worms,theme,worms},
 *                                    g_running, g_need_resize
 *   —            §10 main           the per-frame driver (combines layers)
 *
 * EFFECTS is real here: ripples and spray are cosmetic-only particle pools
 * that advance over time but are never read back by the worm simulation, so
 * deleting them cannot change worm motion. SIMULATION spawns into them
 * (ripple_spawn / spray_burst) but never reads them.
 *
 * LOGIC (terrain_at, seg_char, count_active) does no mutation and no I/O, so
 * reordering or deleting RENDER/EFFECTS cannot change its results.
 *
 * No DELAYS layer: pause is a single flag (scene.paused) tested once in main
 * before the tick; the per-worm timers (swim_timer, ripple_timer,
 * respawn_timer) are owned by and advanced inside the SIMULATION state. There
 * are no global holds.
 *
 * PER-TICK COMBINE — scene_tick (§6) is the ONLY place sim state advances,
 * in order:
 *   1. per worm: if inactive, worm_try_respawn (ages respawn_timer, may
 *      worm_reset); else worm_update — which delegates to worm_swim /
 *      worm_breach (these may ripple_spawn / spray_burst) then relax_chain
 *   2. ripple_update   (EFFECTS)
 *   3. spray_update    (EFFECTS)
 *
 * User events (quit, pause, reset r, breach space, speed +/-, worms w/W,
 * theme t, resize) DO mutate state but are NOT part of the tick — they run in
 * main's input/resize handling, outside and before the gated
 * `if (!scene.paused) scene_tick(&scene, ...)`. Reset r and resize re-invoke
 * INIT (§7). The signal flags g_running/g_need_resize stay global so the
 * handlers can reach them.
 *
 * ─────────────────────────────────────────────────────────────────────── */


/* ── §1 config ───────────────────────────────────────────────────────────── */
#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TICK_NS        33333333L   /* 30 Hz */

/* body chain */
#define N_SEGS         50          /* number of body segments */
#define SEG_LEN        1.8f        /* cells between consecutive segments */

/* worms */
#define MAX_WORMS      8          /* hard upper limit */
#define DEFAULT_WORMS  2

/* swimming */
#define SWIM_DEPTH     9.f         /* rows below terrain surface */
#define SWIM_AMP       2.8f        /* sinusoidal vertical swing */
#define SWIM_FREQ      0.06f       /* radians per col advanced */
#define WORM_SPEED0    14.f        /* default cols/sec */
#define SWIM_TIME_MIN  3.5f        /* seconds between breaches */
#define SWIM_TIME_MAX  7.f

/* breach arc */
#define BREACH_HEIGHT  13.f        /* rows above surface at apex */
#define BREACH_SPAN    58.f        /* horizontal cols for one full arc */

/* ripples */
#define MAX_RIPPLES    8
#define RIPPLE_SPEED   9.f         /* cols/sec */
#define RIPPLE_LIFE    1.8f

/* sand spray */
#define MAX_SPRAY      36
#define SPRAY_GRAVITY  9.f         /* downward accel on grains (cells/sec^2) */
#define SPRAY_BURST_N  24          /* grains flung per breach */
#define SPRAY_T_LO     0.02f       /* breach-arc window where the head breaks the */
#define SPRAY_T_HI     0.13f       /*   surface → fire the one-shot burst in [LO,HI) */

/* ripple shedding while skimming near the surface */
#define RIPPLE_SHED_FRAC 0.65f     /* shed while head is within this fraction of SWIM_DEPTH of the surface */

/* breach mouth sprite */
#define MOUTH_HALF_MAX 3.5f        /* mouth half-width in chars at full open */
#define MOUTH_MIN_OPEN 0.08f       /* below this openness the lips are not drawn */
#define MOUTH_APEX_WIN 0.22f       /* mouth is open within this much of the arc apex (t units) */

/* body chain & lifecycle */
#define TAIL_SEGS      8           /* last N segments drawn as the tapering tail */
#define THICKEN_RATIO  1.3f        /* |dx| past this*|dy| → draw a 2nd row (fat horizontal body) */
#define CHAIN_EPS      0.001f      /* skip the constraint when two nodes nearly coincide */
#define RETIRE_MARGIN  10.f        /* extra cells beyond body length before a worm retires off-screen */
#define RESPAWN_DELAY_S 2.0f       /* seconds a dead worm slot waits before respawning */
#define WORM_STAGGER_S 1.8f        /* extra swim seconds per worm index at reset (staggers entries) */

/* terrain, sky & glyph thresholds */
#define DUNE_CLAMP     3           /* dunes stay within +/- this many rows of the surface centre */
#define SLOPE_AXIS_RATIO 1.73f     /* ~tan(60deg): one axis dominates → straight glyph, else diagonal */
#define SAND_SPECKLE_PERIOD 13     /* 2-in-this-many sub-surface cells get a speckle dot */
#define STAR_DENSITY   6           /* ~this/256 of sky cells get a star */
#define SKY_GAP        4           /* sky stops this many rows above the surface centre */
#define FADE_HOT       0.6f        /* particle life-fraction above this → brightest glyph */
#define FADE_MID       0.3f        /* … above this → mid glyph; below → faintest */

#define MAX_COLS 512

/* ── §2 PERFORMANCE — timing primitives ──────────────────────────────────── *
 * Monotonic clock + sleep. Mutates nothing. The frame cap and dt clamp that use
 * these live in main's loop (variable-dt + frame cap, not a fixed-timestep
 * accumulator).
 * ────────────────────────────────────────────────────────────────────────── */
static long long clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 DATA — types & the Scene aggregate ────────────────────────────────── *
 * Declarations only; no behaviour. State is mutated by EFFECTS (§5) /
 * SIMULATION (§6) / INIT (§7) / EVENTS (§9) and read by LOGIC (§4) / RENDER
 * (§8). The signal flags live with their handlers in §9.
 * ────────────────────────────────────────────────────────────────────────── */

/* color-pair identifiers (RENDER). ncurses pairs are 1-BASED — pair 0 is the
 * reserved default (white-on-black) and cannot be redefined, so the first real
 * id starts at 1. Each id names ONE visually-distinct element; theme_apply (§8)
 * rebinds CP_STAR..CP_SPRAY from the active Theme on every theme switch, while
 * CP_HUD/CP_HINT are fixed (yellow/cyan) per the project HUD standard. Ordered
 * back-to-front by draw order: sky, then terrain, then worm, then effects. */
enum {
    CP_STAR = 1,                                       /* night-sky speckle      */
    CP_GROUND, CP_SAND,                                /* dune crest, sub-surface */
    CP_WORM_TOP, CP_WORM_SUB, CP_WORM_HEAD, CP_MOUTH,  /* dorsal, submerged, head, maw */
    CP_RIPPLE, CP_SPRAY,                               /* sand ripple, sand spray */
    CP_HUD, CP_HINT                                    /* top data bar, bottom action bar */
};

/* Theme — one named RENDER palette, fully DATA-DRIVEN so the whole look swaps at
 * runtime ('t' cycles k_themes) with zero code change. WHY a flat table of ints:
 * each field is an xterm-256 FOREGROUND index (16..231 = the 6x6x6 colour cube,
 * 232..255 = the gray ramp); the background is always -1 (the terminal default)
 * so a theme never fights the user's wallpaper. Indices are picked from the
 * BRIGHT half of the cube so even the darkest tier stays legible on black
 * (project palette-brightness rule). k_themes is the fixed table; the live
 * choice is Scene.theme, and one row maps 1:1 onto the CP_* pairs above.
 *   name                 : label shown in the HUD.
 *   star                 : night-sky speckle               (CP_STAR).
 *   ground, sand         : dune crest glyph, sub-surface fill (CP_GROUND/SAND).
 *   worm_top, worm_sub   : dorsal body, submerged-head tint (CP_WORM_TOP/SUB).
 *   worm_head, mouth     : breaching head, open-maw cavity  (CP_WORM_HEAD/MOUTH).
 *   ripple, spray        : the two sand particle effects    (CP_RIPPLE/SPRAY). */
typedef struct {
    const char *name;
    int star, ground, sand, worm_top, worm_sub, worm_head, mouth, ripple, spray;
} Theme;

#define N_THEMES 10
static const Theme k_themes[N_THEMES] = {
/*          name        star  gnd  sand  top  sub  head  mth  rpl  spr */
/* 0 */ { "Desert",    250, 136,  94,  214, 130,  226, 196, 220, 230 },
/* 1 */ { "Midnight",  147,  67,  17,   51,  23,  123, 201, 111, 159 },
/* 2 */ { "Crimson",   253, 160,  88,  202, 124,  220, 231, 167, 210 },
/* 3 */ { "Arctic",    255, 195, 153,   51,  23,  123, 200, 159, 231 },
/* 4 */ { "Toxic",     248, 148,  22,  118,  28,  154, 196,  82, 155 },
/* 5 */ { "Volcanic",  240,  88,  52,  202, 124,  226, 231, 166, 208 },
/* 6 */ { "Cosmic",    141,  93,  54,  177,  55,  207, 196, 141, 183 },
/* 7 */ { "Sunset",    223, 209, 130,  208,  94,  226, 160, 215, 229 },
/* 8 */ { "Neon",      201,  46,  22,  201,  53,  226, 196,  51, 231 },
/* 9 */ { "Ghost",     252, 240, 235,  255, 244,  231, 238, 248, 253 },
};

/* HeightField — the Arrakis dune surface as a 1-D HEIGHT FIELD: exactly one
 * ground row per screen column (NOT a 2-D grid — the "…Field" suffix marks that
 * lesson). WHY 1-D: the desert is drawn one glyph per column and the worm only
 * ever asks "how deep is the sand at x?", so a row-per-column array answers in
 * O(1) and costs MAX_COLS ints instead of a whole screen grid. INTENT of the
 * profile (sampled by dune_height, laid into row[] by terrain_init): a sum of
 * three sine octaves with FALLING amplitude (1.5 / 0.8 / 0.4) and RISING
 * frequency (2.1 / 5.3 / 11.7) — a cheap fractal-Brownian (fBm) stand-in for
 * true Perlin gradient noise [4][5], seamless because every term is periodic
 * across the width. The sum is clamped to +/-DUNE_CLAMP rows of the centre so
 * dunes never swallow the sky or the bottom HUD.
 *   row[c]      : sand-surface screen ROW at column c. Larger = LOWER on screen
 *                 (row 0 is the top). Valid c in [0,cols); each value clamped to
 *                 [surface_row-3, surface_row+3].
 *   surface_row : baseline centre row (= rows/2) the dune profile varies around,
 *                 and the depth datum worms swim SWIM_DEPTH rows below. */
typedef struct {
    int row[MAX_COLS];
    int surface_row;
} HeightField;

/* Ripple — one concentric sand ring racing outward from the spot where a worm
 * nears or breaks the surface; a coarse particle effect in the spirit of Reeves
 * [7]. WHY just an origin + radius (not a list of points): the ring is perfectly
 * symmetric, so draw_ripples reconstructs it each frame as two mirrored points
 * at ox +/- radius riding the terrain crest — O(1) state per ring. Cosmetic-only
 * (EFFECTS): the worm simulation NEVER reads it back, so a ripple may be added,
 * dropped, or reordered with no effect on motion.
 *   ox     : origin column in cells — fixed at spawn, the ring's centre.
 *   radius : current half-width (cells); grows by RIPPLE_SPEED each tick.
 *   life   : seconds remaining, counts down from RIPPLE_LIFE; the life/RIPPLE_LIFE
 *            fraction selects the glyph ('~' -> '.' -> ' ') as it fades.
 *   active : 1 while this fixed-pool slot is in use (no per-frame allocation). */
typedef struct {
    float ox, radius, life;
    int   active;
} Ripple;

/* Spray — one ballistic sand grain thrown up as the worm's head punches through
 * the surface. A textbook particle [7] integrated with explicit Euler under
 * CONSTANT gravity [6]: each tick vy += g·dt, then pos += vel·dt (spray_update),
 * no inter-particle forces. INTENT: spray_one emits into the UPPER hemisphere
 * only (angle 0..π, vy negative = up) so grains arc up and outward, never down
 * into the dune. Cosmetic-only (EFFECTS): never read back by the simulation.
 *   x, y     : position in cells.
 *   vx, vy   : velocity in cells/sec; vy starts negative (upward) and is bent
 *              back down by gravity each tick, tracing the parabolic toss.
 *   life     : seconds of life remaining, counts down to 0 = dead.
 *   max_life : lifetime captured at spawn; the life/max_life ratio (1 -> 0) is
 *              the ONLY age signal kept and drives the glyph fade ('*' '+' '.').
 *   active   : 1 while this fixed-pool slot is in use. */
typedef struct {
    float x, y, vx, vy, life, max_life;
    int   active;
} Spray;

/* WState — the worm's two-phase lifecycle, a tiny state machine that selects
 * which trajectory the head follows (worm_update branches on it):
 *   WS_SWIM   : cruising UNDER the dunes on a sine path (depth SWIM_DEPTH, swing
 *               SWIM_AMP); a countdown (swim_timer) eventually triggers a breach.
 *   WS_BREACH : arcing ABOVE the surface along a parabola (apex BREACH_HEIGHT,
 *               span BREACH_SPAN); on finishing the arc it returns to WS_SWIM.
 * Only these two phases exist — entering/leaving the field off-screen is handled
 * by the worm's `active` flag, not by a third state. */
typedef enum { WS_SWIM, WS_BREACH } WState;

/* Seg — one body node: a single point on the worm's follower chain (the
 * "circle" in the chain-of-circles / Conga model [1][3]). WHY only (x,y): a
 * segment carries no velocity or mass of its own — it is repositioned purely by
 * the distance constraint against the node ahead of it (relax_chain), so a bare
 * position is all the state it needs. N_SEGS of these, spaced SEG_LEN apart,
 * make one worm body.
 *   x, y : position in cells (fractional; rounded only at draw time). */
typedef struct { float x, y; } Seg;

/* Worm — one sandworm built as a HEAD that drives a SEG_LEN-spaced FOLLOWER
 * CHAIN (the chain-of-circles / Conga model). INTENT / how it moves: only the
 * head is integrated — it rides a swim sinusoid (WS_SWIM) or a breach parabola
 * (WS_BREACH); the body is then relaxed in ONE pass, tail-from-head, by a
 * one-sided DISTANCE CONSTRAINT (relax_chain) — if a segment sits farther than
 * SEG_LEN from the node ahead, slide it straight in until the gap == SEG_LEN.
 * That single relaxation IS Jakobsen's Verlet "stick" with the lead end pinned
 * [1], the
 * simplest case of Position-Based Dynamics [2] and the FABRIK follow-the-leader
 * pass [3]; it is O(N_SEGS) and needs no per-segment velocity. Worms live in a
 * fixed object pool — `active` marks a live slot, so there is no allocation on
 * the hot path.
 *
 *   segs[]        : the body chain; segs[0] is pinned to the head each tick and
 *                   the rest follow by the SEG_LEN constraint.
 *   hx, hy        : head position in CELLS (fractional; rounded at draw time).
 *   dir           : travel sense — +1 = rightward, -1 = leftward.
 *   speed         : head speed (cells/sec); per-worm = base speed * U[0.85,1.15)
 *                   sampled at reset, so worms never march in lock-step.
 *   state         : WState — WS_SWIM (below ground) or WS_BREACH (arcing over).
 *   swim_timer    : seconds of swimming left before the next breach fires; set
 *                   to a random SWIM_TIME_MIN..SWIM_TIME_MAX at each reset/arc.
 *   swim_phase    : accumulating sine phase (radians) for the underground path.
 *   breach_x0     : head x captured at breach start; the arc is parametric in
 *                   t = (hx-breach_x0)/(dir*BREACH_SPAN), with t in [0,1].
 *   mouth_open    : 0 = shut .. 1 = fully open; peaks at the breach apex (t≈0.5)
 *                   and drives how wide the head sprite's maw is drawn.
 *   ripple_timer  : countdown gating how often near-surface swimming sheds a
 *                   ripple (an EFFECTS cadence, not a sim input).
 *   respawn_timer : counts UP while inactive; at 2 s the slot respawns. Preserved
 *                   across worm_reset so the delay survives the reset.
 *   sprayed       : one-shot latch so the breach sand-burst fires exactly once.
 *   active        : 1 while this pool slot holds a live worm. */
typedef struct {
    Seg    segs[N_SEGS];
    float  hx, hy;          /* head position (float cells) */
    float  dir;             /* +1 right / -1 left */
    float  speed;
    WState state;
    float  swim_timer;      /* seconds until next breach */
    float  swim_phase;      /* sinusoidal phase accumulator */
    float  breach_x0;       /* hx at start of breach */
    float  mouth_open;      /* 0=closed 1=fully open */
    float  ripple_timer;
    float  respawn_timer;   /* counts up when inactive */
    int    sprayed;         /* sand spray fired this breach? */
    int    active;
} Worm;

/* Scene — the ENTIRE simulation in one aggregate, laid out like a table of
 * contents so a reader sees WHAT exists, HOW it is tuned, and WHERE in its run
 * it is. WHY one aggregate: state lives in exactly one place, yet functions
 * still take the NARROWEST slice they need (const X* to read, X* to mutate) —
 * only scene_reset / scene_tick take Scene* — so "everything hangs off Scene"
 * never re-couples the layers. worms / ripples / spray are FIXED-SIZE OBJECT
 * POOLS: an `active` flag per slot, a linear scan for a free one, nothing
 * allocated after start-up (no malloc on the hot path).
 *
 *   WHAT  — the domain objects:
 *     terrain   : the dune height field (HeightField).
 *     worms[]   : pool of up to MAX_WORMS sandworms; n_worms of them are managed.
 *   EFFECTS — cosmetic pools the sim writes but never reads back:
 *     ripples[] : up to MAX_RIPPLES expanding sand rings.
 *     spray[]   : up to MAX_SPRAY ballistic sand grains.
 *   HOW   — the user-tunable simulation knobs:
 *     speed     : base head speed (cells/sec); +/- adjust it within [4,40], and
 *                 each worm samples speed*U[0.85,1.15) at reset.
 *     n_worms   : how many worm slots are managed; w/W adjust it in [1,MAX_WORMS].
 *   WHERE — run position / render selector:
 *     paused    : 1 freezes scene_tick (RENDER keeps running).
 *     theme     : index into k_themes of the active palette — a RENDER choice
 *                 (cycled by 't'), kept on Scene so reset/resize preserve it. */
typedef struct {
    HeightField terrain;
    Worm        worms[MAX_WORMS];
    Ripple      ripples[MAX_RIPPLES];
    Spray       spray[MAX_SPRAY];
    float       speed;
    int         n_worms;
    int         paused;
    int         theme;
} Scene;

static Scene g_scene = { .speed = WORM_SPEED0, .n_worms = DEFAULT_WORMS };

/* ── §4 LOGIC — pure reads (no mutation, no I/O) ──────────────────────────── *
 * Each reads its inputs and returns a value. Deleting or reordering
 * RENDER/EFFECTS cannot change their results.
 * ────────────────────────────────────────────────────────────────────────── */

/* PRNG utilities — the one impurity in §4: they advance the shared rand()
 * stream, but hold no sim state and touch no screen. They reproduce the file's
 * original 0.01-step jitter exactly, so swapping them in leaves visuals
 * unchanged. */

/* uniform random float in [lo, hi) in 0.01 steps */
static float frand_range(float lo, float hi) {
    int steps = (int)((hi - lo) * 100.f + 0.5f);
    return lo + (float)(rand() % steps) * 0.01f;
}

/* symmetric random offset about 0: (rand()%n - n/2) * scale */
static float rand_centered(int n, float scale) {
    int off = rand() % n - n / 2;
    return (float)off * scale;
}

/* random +1 or -1 — a coin flip for which side a worm enters from */
static float rand_sign(void) {
    return (rand() % 2 == 0) ? 1.f : -1.f;
}

static int terrain_at(const HeightField *hf, float x, int cols) {
    int c = (int)(x + 0.5f);
    if (c < 0)    c = 0;
    if (c >= cols) c = cols - 1;
    return hf->row[c];
}

/* segment character: direction toward head gives slope → ring char */
static char seg_char(float dx, float dy, int idx) {
    float ax = fabsf(dx), ay = fabsf(dy);
    if (ay > ax * SLOPE_AXIS_RATIO)
        return (idx % 2 == 0) ? '|' : '!';
    if (ax > ay * SLOPE_AXIS_RATIO)
        return (idx % 3 == 0) ? 'O' : (idx % 3 == 1) ? 'o' : '0';
    return (dx * dy > 0.f) ? '\\' : '/';
}

static int count_active(const Worm *worms, int n_worms) {
    int n = 0;
    for (int i = 0; i < n_worms; i++) if (worms[i].active) n++;
    return n;
}

/* true once the whole body has cleared the screen edge plus a margin */
static int worm_off_field(const Worm *w, int cols) {
    float margin = (float)N_SEGS * SEG_LEN + RETIRE_MARGIN;
    return (w->dir > 0.f && w->hx > (float)cols + margin) ||
           (w->dir < 0.f && w->hx < -margin);
}

/* normalized progress along the breach arc: t in [0,1) as the head advances
   BREACH_SPAN cols from breach_x0 (clamped non-negative). */
static float breach_t(const Worm *w) {
    float t = (w->hx - w->breach_x0) / (w->dir * BREACH_SPAN);
    return t < 0.f ? 0.f : t;
}

/* height above the surface at arc progress t — a parabola peaking at t=0.5
   (4t(1-t) hits 1 at the apex), scaled to BREACH_HEIGHT rows. */
static float breach_height(float t) {
    return BREACH_HEIGHT * 4.f * t * (1.f - t);
}

/* mouth openness 0..1: fully open at the apex (t=0.5), ramping shut within
   MOUTH_APEX_WIN of it, closed beyond. */
static float mouth_openness(float t) {
    float d_apex = fabsf(t - 0.5f);
    return (d_apex < MOUTH_APEX_WIN) ? 1.f - d_apex / MOUTH_APEX_WIN : 0.f;
}

/* one sample of the dune profile at normalized x in [0,1]: a 3-octave fBm with
   falling amplitude and rising frequency — a cheap periodic stand-in for Perlin
   gradient noise [4][5]. */
static float dune_height(float x) {
    static const float freq [3] = { 2.1f, 5.3f, 11.7f };
    static const float amp  [3] = { 1.5f, 0.8f,  0.4f };
    static const float phase[3] = { 0.f,  0.9f,  0.4f };
    float h = 0.f;
    for (int k = 0; k < 3; k++)
        h += sinf(x * (float)M_PI * freq[k] + phase[k]) * amp[k];
    return h;
}

/* 2-D integer hash → a well-mixed pseudo-random word (MurmurHash-style
   finalizer); scatters a stable star field across the sky. */
static unsigned int star_hash(int c, int r) {
    unsigned int h = (unsigned int)(c * 1234597u ^ r * 987659u);
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}

/* ── §5 EFFECTS — cosmetic-only particle pools ────────────────────────────── *
 * Ripples and spray advance over time but are NEVER read back by the worm
 * simulation, so they cannot influence worm motion. SIMULATION spawns into
 * them (ripple_spawn / spray_burst); scene_tick advances them each tick.
 * ────────────────────────────────────────────────────────────────────────── */

static void ripple_spawn(Ripple *ripples, float ox) {
    for (int i = 0; i < MAX_RIPPLES; i++) {
        if (ripples[i].active) continue;
        ripples[i] = (Ripple){ ox, 0.f, RIPPLE_LIFE, 1 };
        return;
    }
}

static void ripple_update(Ripple *ripples, float dt) {
    for (int i = 0; i < MAX_RIPPLES; i++) {
        if (!ripples[i].active) continue;
        ripples[i].life   -= dt;
        ripples[i].radius += RIPPLE_SPEED * dt;
        if (ripples[i].life <= 0.f) ripples[i].active = 0;
    }
}

static void spray_one(Spray *spray, float x, float y) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        if (spray[i].active) continue;
        /* upward hemisphere: angle 0..π → left/up/right burst */
        float ang = (float)(rand() % 180) * (float)M_PI / 180.f;
        float spd = frand_range(3.f, 10.f);     /* grain speed (cells/sec) */
        spray[i].active   = 1;
        spray[i].x        = x + rand_centered(7, 0.5f);   /* small spawn scatter */
        spray[i].y        = y;
        spray[i].vx       =  cosf(ang) * spd;
        spray[i].vy       = -sinf(ang) * spd;   /* negative = upward */
        spray[i].life     = frand_range(0.4f, 1.0f);
        spray[i].max_life = spray[i].life;
        return;
    }
}

static void spray_burst(Spray *spray, float x, float y, int n) {
    for (int i = 0; i < n; i++) spray_one(spray, x, y);
}

static void spray_update(Spray *spray, float dt) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        if (!spray[i].active) continue;
        spray[i].life -= dt;
        if (spray[i].life <= 0.f) { spray[i].active = 0; continue; }
        spray[i].vy += SPRAY_GRAVITY * dt;
        spray[i].x  += spray[i].vx * dt;
        spray[i].y  += spray[i].vy * dt;
    }
}

/* ── §6 SIMULATION — worm physics & the per-tick combine ───────────────────── *
 * worm_update advances one worm by delegating to the phase helpers worm_swim /
 * worm_breach (which read terrain via LOGIC §4 and spawn into EFFECTS §5), then
 * relax_chain pulls the body after the head. scene_tick is the SINGLE per-tick
 * combine — nothing else in the program advances simulation state. worm_reset
 * re-seeds one worm and is reused by INIT (§7) and worm_try_respawn here.
 * ────────────────────────────────────────────────────────────────────────── */

static void worm_reset(Worm *w, const HeightField *hf, float speed,
                       int cols, int rows, float dir) {
    float respawn = w->respawn_timer;  /* preserve across reset */
    memset(w, 0, sizeof *w);
    w->respawn_timer = respawn;
    w->active      = 1;
    w->dir         = dir;
    w->speed       = speed * frand_range(0.85f, 1.15f);   /* +/-15% so worms desync */
    w->state       = WS_SWIM;
    w->swim_timer  = frand_range(SWIM_TIME_MIN, SWIM_TIME_MAX);
    w->swim_phase  = frand_range(0.f, 6.28f);             /* random start phase [0,2pi) */
    w->ripple_timer = 0.6f;

    /* start off-screen on the side matching dir */
    float start_x = (dir > 0.f)
                  ? -(float)(N_SEGS) * SEG_LEN
                  : (float)cols + (float)(N_SEGS) * SEG_LEN;
    float start_y = (float)(hf->surface_row + (int)SWIM_DEPTH);

    for (int i = 0; i < N_SEGS; i++) {
        w->segs[i].x = start_x - dir * (float)i * SEG_LEN;
        w->segs[i].y = start_y;
    }
    w->hx = start_x;
    w->hy = start_y;
    (void)rows;
}

/* WS_SWIM: ride the underground sine path, shedding ripples while skimming the
   surface, until the breach countdown fires. */
static void worm_swim(Worm *w, const HeightField *hf, Ripple *ripples,
                      float dt, int cols) {
    float surf = (float)terrain_at(hf, w->hx, cols);
    w->hy          = surf + SWIM_DEPTH + sinf(w->swim_phase) * SWIM_AMP;
    w->swim_phase += SWIM_FREQ * w->speed * dt;
    w->mouth_open  = 0.f;

    /* shed ripples while skimming near the surface */
    float depth = w->hy - surf;
    if (depth < SWIM_DEPTH * RIPPLE_SHED_FRAC) {
        w->ripple_timer -= dt;
        if (w->ripple_timer <= 0.f) {
            ripple_spawn(ripples, w->hx);
            w->ripple_timer = frand_range(0.45f, 0.85f);
        }
    }

    /* countdown to the next breach */
    w->swim_timer -= dt;
    if (w->swim_timer <= 0.f) {
        w->state     = WS_BREACH;
        w->breach_x0 = w->hx;
        w->sprayed   = 0;
        /* big ripple cluster at the breach point */
        ripple_spawn(ripples, w->hx);
        ripple_spawn(ripples, w->hx);
        ripple_spawn(ripples, w->hx + w->dir * 3.f);
    }
}

/* WS_BREACH: arc over the surface on a parabola, opening the mouth near the
   apex and flinging a one-shot sand burst as the head breaks through; returns
   to WS_SWIM when the arc completes. */
static void worm_breach(Worm *w, const HeightField *hf, Spray *spray, int cols) {
    float t = breach_t(w);
    if (t >= 1.f) {
        /* completed arc — resume swimming */
        w->state      = WS_SWIM;
        w->swim_timer = frand_range(SWIM_TIME_MIN, SWIM_TIME_MAX);
        w->mouth_open = 0.f;
    } else {
        float surf    = (float)terrain_at(hf, w->hx, cols);
        w->hy         = surf - breach_height(t);
        w->mouth_open = mouth_openness(t);

        /* one-shot sand spray as the head breaks the surface */
        if (!w->sprayed && t > SPRAY_T_LO && t < SPRAY_T_HI) {
            spray_burst(spray, w->hx, surf, SPRAY_BURST_N);
            w->sprayed = 1;
        }
    }
}

/* pin segs[0] to the head, then pull each following node straight in until it
   sits exactly SEG_LEN from the node ahead — one tail-from-head pass of
   Jakobsen's pinned distance constraint [1] (the chain-of-circles follower). */
static void relax_chain(Worm *w) {
    w->segs[0].x = w->hx;
    w->segs[0].y = w->hy;
    for (int i = 1; i < N_SEGS; i++) {
        float dx   = w->segs[i-1].x - w->segs[i].x;
        float dy   = w->segs[i-1].y - w->segs[i].y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > SEG_LEN && dist > CHAIN_EPS) {
            float s = (dist - SEG_LEN) / dist;
            w->segs[i].x += dx * s;
            w->segs[i].y += dy * s;
        }
    }
}

static void worm_update(Worm *w, const HeightField *hf,
                        Ripple *ripples, Spray *spray,
                        float dt, int cols, int rows) {
    if (!w->active) return;

    w->hx += w->dir * w->speed * dt;                 /* advance head along its lane */
    if (w->state == WS_SWIM) worm_swim(w, hf, ripples, dt, cols);
    else                     worm_breach(w, hf, spray, cols);

    if (worm_off_field(w, cols)) { w->active = 0; return; }

    relax_chain(w);                                  /* body follows the head */
    (void)rows;
}

/* a dead slot ages its respawn timer and re-enters from a random side */
static void worm_try_respawn(Worm *w, const HeightField *hf, float speed,
                             int cols, int rows, float dt) {
    w->respawn_timer += dt;
    if (w->respawn_timer >= RESPAWN_DELAY_S)
        worm_reset(w, hf, speed, cols, rows, rand_sign());
}

/* the per-tick combine: the ONLY place sim state advances (see ARCHITECTURE) */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
    for (int i = 0; i < s->n_worms; i++) {
        Worm *w = &s->worms[i];
        if (!w->active)
            worm_try_respawn(w, &s->terrain, s->speed, cols, rows, dt);
        else
            worm_update(w, &s->terrain, s->ripples, s->spray, dt, cols, rows);
    }
    ripple_update(s->ripples, dt);
    spray_update(s->spray, dt);
}

/* ── §7 INIT/RESET — builds Scene state (setup, NOT part of the tick) ───────── *
 * Mutates terrain + worm + effect state wholesale. Called at startup, on reset
 * (r), and on resize — all outside scene_tick.
 * ────────────────────────────────────────────────────────────────────────── */

static void terrain_init(HeightField *hf, int cols, int rows) {
    hf->surface_row = rows / 2;                          /* surface sits at mid-screen */
    for (int c = 0; c < cols; c++) {
        float x  = (float)c / (float)(cols > 1 ? cols - 1 : 1);
        int   gr = hf->surface_row + (int)(dune_height(x) + 0.5f);
        if (gr < hf->surface_row - DUNE_CLAMP) gr = hf->surface_row - DUNE_CLAMP;
        if (gr > hf->surface_row + DUNE_CLAMP) gr = hf->surface_row + DUNE_CLAMP;
        hf->row[c] = gr;
    }
}

static void scene_reset(Scene *s, int cols, int rows) {
    memset(s->worms,   0, sizeof s->worms);
    memset(s->ripples, 0, sizeof s->ripples);
    memset(s->spray,   0, sizeof s->spray);
    terrain_init(&s->terrain, cols, rows);
    /* spawn worms alternating sides, stagger their breach timers */
    for (int i = 0; i < s->n_worms; i++) {
        float dir = (i % 2 == 0) ? 1.f : -1.f;
        worm_reset(&s->worms[i], &s->terrain, s->speed, cols, rows, dir);
        s->worms[i].swim_timer += (float)i * WORM_STAGGER_S;   /* stagger entries */
    }
}

/* ── §8 RENDER — Scene → screen (reads only, never mutates sim state) ──────── *
 * color_init / theme_apply set up pairs; the draw_* functions translate the
 * current state into characters and take the narrowest const sub-type. Particle
 * FADE is derived here from life fraction, never stored.
 * ────────────────────────────────────────────────────────────────────────── */

static void theme_apply(int t) {
    const Theme *th = &k_themes[t];
    init_pair(CP_STAR,      th->star,     -1);
    init_pair(CP_GROUND,    th->ground,   -1);
    init_pair(CP_SAND,      th->sand,     -1);
    init_pair(CP_WORM_TOP,  th->worm_top, -1);
    init_pair(CP_WORM_SUB,  th->worm_sub, -1);
    init_pair(CP_WORM_HEAD, th->worm_head,-1);
    init_pair(CP_MOUTH,     th->mouth,    -1);
    init_pair(CP_RIPPLE,    th->ripple,   -1);
    init_pair(CP_SPRAY,     th->spray,    -1);
}

static void color_init(int theme) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, -1);  /* bright yellow on default bg — top data bar */
        init_pair(CP_HINT,  51, -1);  /* bright cyan   on default bg — bottom action bar */
        theme_apply(theme);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);  /* HUD fallback */
        init_pair(CP_HINT, COLOR_CYAN,   -1);  /* HINT fallback */
        init_pair(CP_STAR,      COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_GROUND,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_SAND,      COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_WORM_TOP,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_WORM_SUB,  COLOR_RED,    COLOR_BLACK);
        init_pair(CP_WORM_HEAD, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_MOUTH,     COLOR_RED,    COLOR_BLACK);
        init_pair(CP_RIPPLE,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_SPRAY,     COLOR_WHITE,  COLOR_BLACK);
    }
}

static void draw_stars(const HeightField *hf, int cols, int rows) {
    (void)rows;
    int sky_bottom = hf->surface_row - SKY_GAP;
    attron(COLOR_PAIR(CP_STAR) | A_DIM);
    for (int r = 0; r < sky_bottom; r++) {
        for (int c = 0; c < cols; c++) {
            unsigned int h = star_hash(c, r);
            if ((h & 0xFF) < STAR_DENSITY)            /* ~STAR_DENSITY/256 of cells */
                mvaddch(r, c, (h >> 9 & 1) ? '*' : '.');
        }
    }
    attroff(COLOR_PAIR(CP_STAR) | A_DIM);
}

static void draw_terrain(const HeightField *hf, int cols, int rows) {
    for (int c = 0; c < cols; c++) {
        int gr = hf->row[c];
        if (gr < 0 || gr >= rows) continue;

        int   prev_gr = (c > 0)        ? hf->row[c-1] : gr;
        int   next_gr = (c < cols - 1) ? hf->row[c+1] : gr;
        int   slope   = next_gr - prev_gr;
        char  sch     = (slope < -1) ? '/' : (slope > 1) ? '\\' : '_';

        attron(COLOR_PAIR(CP_GROUND) | A_BOLD);
        mvaddch(gr, c, (chtype)(unsigned char)sch);
        attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);

        attron(COLOR_PAIR(CP_SAND));
        for (int r = gr + 1; r < rows - 1; r++) {
            char fc = ((c * 3 + r * 7) % SAND_SPECKLE_PERIOD < 2) ? '.' : ' ';
            mvaddch(r, c, (chtype)(unsigned char)fc);
        }
        attroff(COLOR_PAIR(CP_SAND));
    }
}

/* the follower chain, drawn tail-to-head so the head writes last (on top); each
   segment is a slope glyph above ground, a faint dot showing through the sand. */
static void draw_worm_body(const Worm *w, const HeightField *hf, int cols, int rows) {
    for (int i = N_SEGS - 1; i >= 1; i--) {
        int sr = (int)(w->segs[i].y + 0.5f);
        int sc = (int)(w->segs[i].x + 0.5f);
        if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) continue;

        /* direction of this segment toward head */
        float dx = w->segs[i-1].x - w->segs[i].x;
        float dy = w->segs[i-1].y - w->segs[i].y;
        char  ch = seg_char(dx, dy, i);

        int is_tail      = (i > N_SEGS - TAIL_SEGS);
        int surf         = terrain_at(hf, w->segs[i].x, cols);
        int above_ground = (sr < surf);

        if (above_ground) {
            attr_t at = is_tail ? A_DIM : A_BOLD;
            attron(COLOR_PAIR(CP_WORM_TOP) | at);
            mvaddch(sr, sc, (chtype)(unsigned char)ch);
            /* thicken: draw a second row when the segment is mostly horizontal */
            if (!is_tail && fabsf(dx) > fabsf(dy) * THICKEN_RATIO && sr + 1 < surf && sr + 1 < rows)
                mvaddch(sr + 1, sc, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(CP_WORM_TOP) | at);
        } else {
            /* underground: bright dorsal theme colour so the body still shows
               clearly through the sand (no A_DIM — that washes out against it) */
            attr_t at = is_tail ? A_NORMAL : A_BOLD;
            attron(COLOR_PAIR(CP_WORM_TOP) | at);
            mvaddch(sr, sc, (chtype)(unsigned char)(i % 3 == 0 ? 'o' : '.'));
            attroff(COLOR_PAIR(CP_WORM_TOP) | at);
        }
    }
}

/* the head: a multi-row open-mouth sprite when breaching above ground (top lip /
   maw cavity / bottom lip, width from mouth_open), or a single bold dot when
   still under the sand. */
static void draw_worm_head(const Worm *w, const HeightField *hf, int cols, int rows) {
    int hr = (int)(w->hy + 0.5f);
    int hc = (int)(w->hx + 0.5f);
    if (hc < 0 || hc >= cols || hr < 0 || hr >= rows) return;

    int  surf        = terrain_at(hf, w->hx, cols);
    int  above_ground = (hr < surf);

    if (above_ground) {
        /* open_w: how many chars the mouth extends each side */
        int open_w = (int)(w->mouth_open * MOUTH_HALF_MAX);

        attron(COLOR_PAIR(CP_WORM_HEAD) | A_BOLD);

        /* top lip */
        if (w->mouth_open > MOUTH_MIN_OPEN && hr - 1 >= 0) {
            for (int j = -open_w; j <= open_w; j++) {
                int mc = hc + j;
                if (mc < 0 || mc >= cols) continue;
                mvaddch(hr - 1, mc, (j == -open_w || j == open_w) ? '|' : '_');
            }
        }

        /* centre row — mouth cavity */
        if (open_w == 0) {
            mvaddch(hr, hc, '@');
        } else {
            int lo = hc - open_w - 1, hi = hc + open_w + 1;
            if (lo >= 0)    mvaddch(hr, lo, '(');
            if (hi < cols)  mvaddch(hr, hi, ')');
            attron(COLOR_PAIR(CP_MOUTH) | A_BOLD);
            for (int j = -open_w; j <= open_w; j++) {
                int mc = hc + j;
                if (mc < 0 || mc >= cols) continue;
                mvaddch(hr, mc, j == 0 ? '@' : ' ');
            }
            attroff(COLOR_PAIR(CP_MOUTH) | A_BOLD);
            attron(COLOR_PAIR(CP_WORM_HEAD) | A_BOLD);
        }

        /* bottom lip */
        if (w->mouth_open > MOUTH_MIN_OPEN && hr + 1 < surf && hr + 1 < rows) {
            for (int j = -open_w; j <= open_w; j++) {
                int mc = hc + j;
                if (mc < 0 || mc >= cols) continue;
                mvaddch(hr + 1, mc, (j == -open_w || j == open_w) ? '|' : '-');
            }
        }

        attroff(COLOR_PAIR(CP_WORM_HEAD) | A_BOLD);

    } else {
        /* head underground: bold dot */
        attron(COLOR_PAIR(CP_WORM_SUB) | A_BOLD);
        mvaddch(hr, hc, 'O');
        attroff(COLOR_PAIR(CP_WORM_SUB) | A_BOLD);
    }
}

static void draw_worm(const Worm *w, const HeightField *hf, int cols, int rows) {
    if (!w->active) return;
    draw_worm_body(w, hf, cols, rows);
    draw_worm_head(w, hf, cols, rows);
}

static void draw_ripples(const Ripple *ripples, const HeightField *hf,
                         int cols, int rows) {
    for (int i = 0; i < MAX_RIPPLES; i++) {
        const Ripple *rp = &ripples[i];
        if (!rp->active) continue;
        float frac = rp->life / RIPPLE_LIFE;
        int   r    = (int)(rp->radius + 0.5f);

        for (int side = -1; side <= 1; side += 2) {
            /* outer ring */
            int c = (int)(rp->ox + (float)(side * r) + 0.5f);
            if (c >= 0 && c < cols) {
                int row = terrain_at(hf, (float)c, cols) - 1;
                if (row >= 0 && row < rows) {
                    char rch = (frac > FADE_HOT) ? '~' : (frac > FADE_MID) ? '.' : ' ';
                    attron(COLOR_PAIR(CP_RIPPLE) | A_BOLD);
                    mvaddch(row, c, (chtype)(unsigned char)rch);
                    attroff(COLOR_PAIR(CP_RIPPLE) | A_BOLD);
                }
            }
            /* inner half-radius ring */
            int c2 = (int)(rp->ox + (float)(side * (r / 2.0f)) + 0.5f);
            if (c2 >= 0 && c2 < cols) {
                int row = terrain_at(hf, (float)c2, cols) - 1;
                if (row >= 0 && row < rows) {
                    attron(COLOR_PAIR(CP_RIPPLE));
                    mvaddch(row, c2, '.');
                    attroff(COLOR_PAIR(CP_RIPPLE));
                }
            }
        }
    }
}

static void draw_spray(const Spray *spray, int cols, int rows) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        if (!spray[i].active) continue;
        int sr = (int)(spray[i].y + 0.5f);
        int sc = (int)(spray[i].x + 0.5f);
        if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) continue;
        float frac = spray[i].life / spray[i].max_life;
        char  ch   = (frac > FADE_HOT) ? '*' : (frac > FADE_MID) ? '+' : '.';
        int   pair = (frac > FADE_HOT) ? CP_SPRAY : CP_RIPPLE;
        attron(COLOR_PAIR(pair));
        mvaddch(sr, sc, (chtype)(unsigned char)ch);
        attroff(COLOR_PAIR(pair));
    }
}

/*
 * draw_hud — two fixed bars (HUD Standard). Takes the few scene values it shows
 * (not Scene*), so RENDER stays decoupled from the aggregate:
 *   top    row 0      DATA    — title (left) + speed / worm count / theme / run
 *                              state (right-aligned, yellow).
 *   bottom row rows-1 ACTIONS — the key legend (cyan).
 * Both bars are filled first, then text is clipped with "%.*s" so a narrow
 * terminal can neither overflow nor wrap.
 */
static void draw_hud(float speed, int n_active, int n_total,
                     const char *theme, int paused, int rows, int cols) {
    if (cols < 1 || rows < 1) return;

    /* ── Top row 0: DATA. ── */
    char left[24], right[72];
    snprintf(left,  sizeof left,  " DUNE SANDWORM ");
    snprintf(right, sizeof right, " spd:%.0f  worms:%d/%d  theme:%s  %s ",
             speed, n_active, n_total, theme,
             paused ? "PAUSED" : "running");
    int rx = cols - (int)strlen(right);          /* right-aligned stats column */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(0, c, ' ');
    if (rx >= 0) {
        mvprintw(0, 0,  "%.*s", rx, left);       /* title clipped clear of stats */
        mvprintw(0, rx, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, right);    /* too narrow: data only */
    }
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* ── Bottom row: ACTIONS — every interactive key. ── */
    int brow = rows - 1;
    if (brow > 0) {
        attron(COLOR_PAIR(CP_HINT) | A_BOLD);
        for (int c = 0; c < cols; c++) mvaddch(brow, c, ' ');
        mvprintw(brow, 0, "%.*s", cols,
                 " q:quit  p:pause  r:reset  +/-:speed  spc:breach  w/W:worms  t:theme ");
        attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
    }
}

/* ── §9 EVENTS — signals & user actions (mutate state, NOT part of the tick) ─ *
 * Signal handlers set volatile flags read by main; they stay global so the
 * handlers can reach them without a Scene pointer. The key handling itself
 * lives inline in main's input loop. These run outside scene_tick.
 * ────────────────────────────────────────────────────────────────────────── */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;
static void on_sigint(int s)   { (void)s; g_running = 0; }
static void on_sigwinch(int s) { (void)s; g_need_resize = 1; }

/* ── §10 main ────────────────────────────────────────────────────────────── */
int main(void) {
    srand((unsigned)time(NULL));
    signal(SIGINT,   on_sigint);
    signal(SIGTERM,  on_sigint);
    signal(SIGWINCH, on_sigwinch);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    color_init(g_scene.theme);

    int rows = LINES, cols = COLS;
    scene_reset(&g_scene, cols, rows);

    long long prev = clock_ns();

    while (g_running) {
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': g_scene.paused = !g_scene.paused; break;
            case 'r': case 'R': scene_reset(&g_scene, cols, rows); break;
            case ' ':
                /* force the next swimming worm to breach now */
                for (int i = 0; i < MAX_WORMS; i++) {
                    Worm *w = &g_scene.worms[i];
                    if (w->active && w->state == WS_SWIM) {
                        w->swim_timer = 0.f;
                        break;
                    }
                }
                break;
            case '+': case '=':
                g_scene.speed = fminf(g_scene.speed + 2.f, 40.f);
                for (int i = 0; i < MAX_WORMS; i++)
                    if (g_scene.worms[i].active) g_scene.worms[i].speed = g_scene.speed;
                break;
            case '-':
                g_scene.speed = fmaxf(g_scene.speed - 2.f, 4.f);
                for (int i = 0; i < MAX_WORMS; i++)
                    if (g_scene.worms[i].active) g_scene.worms[i].speed = g_scene.speed;
                break;
            case 'w': case 'W':
                /* w = add worm, W = remove worm */
                if (ch == 'w' || ch == 'W') {
                    if (ch == 'w' && g_scene.n_worms < MAX_WORMS) {
                        int idx = g_scene.n_worms++;
                        float dir = rand_sign();
                        memset(&g_scene.worms[idx], 0, sizeof g_scene.worms[idx]);
                        worm_reset(&g_scene.worms[idx], &g_scene.terrain, g_scene.speed,
                                   cols, rows, dir);
                    } else if (ch == 'W' && g_scene.n_worms > 1) {
                        g_scene.n_worms--;
                        g_scene.worms[g_scene.n_worms].active = 0;
                    }
                }
                break;
            case 't': case 'T':
                if (COLORS >= 256) {
                    g_scene.theme = (g_scene.theme + 1) % N_THEMES;
                    theme_apply(g_scene.theme);
                }
                break;
            case KEY_RESIZE: g_need_resize = 1; break;
            }
        }

        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            scene_reset(&g_scene, cols, rows);
        }

        long long now = clock_ns();
        float dt = (float)(now - prev) * 1e-9f;
        if (dt > 0.1f) dt = 0.1f;
        prev = now;

        if (!g_scene.paused) scene_tick(&g_scene, dt, cols, rows);

        erase();
        draw_stars(&g_scene.terrain, cols, rows);
        draw_terrain(&g_scene.terrain, cols, rows);
        draw_ripples(g_scene.ripples, &g_scene.terrain, cols, rows);
        draw_spray(g_scene.spray, cols, rows);
        for (int i = 0; i < MAX_WORMS; i++)
            draw_worm(&g_scene.worms[i], &g_scene.terrain, cols, rows);
        draw_hud(g_scene.speed, count_active(g_scene.worms, g_scene.n_worms),
                 g_scene.n_worms, k_themes[g_scene.theme].name, g_scene.paused,
                 rows, cols);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }

    endwin();
    return 0;
}
