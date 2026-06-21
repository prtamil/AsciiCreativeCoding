/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* dune_sandworm.c — Arrakis sandworms that swim under the dunes, breach with
 * an open mouth, then dive again. The body is a chain of points that follow
 * the head. Refs: Jakobsen "Advanced Character Physics" (GDC 2001) for the
 * follower chain; Perlin/fBm noise for the dune profile. */

/* ── §1 config — all the tuning knobs ────────────────────────────────────── */
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

/* ── §2 timing — read the clock, sleep to cap the frame rate ─────────────── */
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

/* ── §3 data — the types and the one Scene that holds everything ──────────── */

/* Names for the color pairs. ncurses pair numbers start at 1 (pair 0 is the
 * fixed default and can't be redefined), so the first id is 1. theme_apply (§8)
 * recolors CP_STAR..CP_SPRAY when you switch themes; CP_HUD/CP_HINT stay fixed.
 * Listed in draw order: sky, then ground, then worm, then effects. */
enum {
    CP_STAR = 1,                                       /* night-sky speckle      */
    CP_GROUND, CP_SAND,                                /* dune crest, sub-surface */
    CP_WORM_TOP, CP_WORM_SUB, CP_WORM_HEAD, CP_MOUTH,  /* dorsal, submerged, head, maw */
    CP_RIPPLE, CP_SPRAY,                               /* sand ripple, sand spray */
    CP_HUD, CP_HINT                                    /* top data bar, bottom action bar */
};

/* Theme — one named color palette. The whole look is just a row of numbers, so
 * pressing 't' swaps every color at once with no code change. Each field is an
 * xterm-256 foreground color number; the background is always the terminal
 * default (-1) so a theme never clashes with the user's wallpaper. Colors are
 * picked from the bright half of the palette so even the darkest one stays
 * readable on black. One row lines up 1:1 with the CP_* pairs above.
 *   name                 : label shown in the HUD.
 *   star                 : night-sky speckle.
 *   ground, sand         : dune-crest glyph, the sand fill below it.
 *   worm_top, worm_sub   : body color above ground, submerged-head tint.
 *   worm_head, mouth     : breaching head, the open mouth's inside.
 *   ripple, spray        : the two sand particle effects. */
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

/* HeightField — the dune surface stored as one ground row per screen column,
 * not a full 2-D grid. The desert is drawn a column at a time and the worm only
 * ever asks "where's the sand surface at this x?", so a single row-per-column
 * array answers instantly and costs far less memory. The shape comes from
 * adding three sine waves of shrinking height and rising frequency (built in
 * terrain_init) — a cheap stand-in for Perlin noise that gives rolling dunes.
 * The result is kept within DUNE_CLAMP rows of center so dunes never swallow
 * the sky or the bottom HUD.
 *   row[c]      : screen row of the sand surface at column c. Bigger means lower
 *                 on screen (row 0 is the top). c ranges over [0,cols).
 *   surface_row : the center row (= rows/2) the dunes wave around, and the line
 *                 worms swim SWIM_DEPTH rows beneath. */
typedef struct {
    int row[MAX_COLS];
    int surface_row;
} HeightField;

/* Ripple — one sand ring that races outward from where a worm nears or breaks
 * the surface. We store only a center and a radius (not a list of points)
 * because the ring is symmetric: draw_ripples just redraws two mirrored points
 * at center +/- radius each frame. Purely cosmetic — the worm motion never
 * reads it, so adding or dropping a ripple changes nothing about the worms.
 *   ox     : center column where the ring started.
 *   radius : how far the ring has spread (cells); grows each tick.
 *   life   : seconds left, counting down from RIPPLE_LIFE; its fraction picks
 *            the fading glyph ('~' then '.' then blank).
 *   active : 1 while this slot in the fixed pool is in use. */
typedef struct {
    float ox, radius, life;
    int   active;
} Ripple;

/* Spray — one sand grain flung up when the worm's head punches the surface.
 * Each tick gravity pulls its upward speed back down, then it moves by its
 * speed, tracing a thrown-rock arc (spray_update). Grains are launched only
 * upward and outward, never down into the dune. Purely cosmetic — the worm
 * motion never reads it.
 *   x, y     : position in cells.
 *   vx, vy   : speed in cells/sec; vy starts negative (upward) and gravity bends
 *              it back down over time.
 *   life     : seconds left before it dies at 0.
 *   max_life : the life it started with; life/max_life (1 down to 0) is how we
 *              fade the glyph ('*' then '+' then '.').
 *   active   : 1 while this slot in the fixed pool is in use. */
typedef struct {
    float x, y, vx, vy, life, max_life;
    int   active;
} Spray;

/* WState — which of the worm's two phases it is in; picks the path the head
 * follows (worm_update checks it):
 *   WS_SWIM   : cruising under the dunes on a wavy path until a timer fires.
 *   WS_BREACH : arcing up over the surface, then dropping back to WS_SWIM. */
typedef enum { WS_SWIM, WS_BREACH } WState;

/* Seg — one bead on the worm's body chain. It has no speed or weight of its
 * own; relax_chain just drags it to stay SEG_LEN behind the bead ahead, so a
 * plain position is all it needs. N_SEGS of these make one worm.
 *   x, y : position in cells (can be fractional; rounded at draw time). */
typedef struct { float x, y; } Seg;

/* Worm — one sandworm: a head that leads and a chain of beads that follow it.
 * Only the head actually moves on its own (along a swim wave or a breach arc);
 * the body then catches up in one pass from head to tail, each bead pulled in
 * to sit SEG_LEN behind the one ahead (relax_chain). This follower trick is
 * Jakobsen's pinned-stick chain. Worms live in a fixed array — `active` marks a
 * live slot — so nothing is allocated while the program runs.
 *
 *   segs[]        : the body beads; segs[0] is snapped to the head each tick.
 *   hx, hy        : head position in cells.
 *   dir           : travel direction — +1 right, -1 left.
 *   speed         : head speed (cells/sec); each worm gets a slightly different
 *                   one at reset so they don't move in lock-step.
 *   state         : swimming (WS_SWIM) or breaching (WS_BREACH).
 *   swim_timer    : seconds of swimming left before the next breach.
 *   swim_phase    : how far along the swim wave the head is.
 *   breach_x0     : head x where the current breach began; the arc measures its
 *                   progress from here.
 *   mouth_open    : 0 shut .. 1 fully open; widest at the top of the arc and
 *                   sets how wide the mouth is drawn.
 *   ripple_timer  : countdown for how often a near-surface swim sheds a ripple.
 *   respawn_timer : counts up while dead; at RESPAWN_DELAY_S the slot respawns.
 *                   Kept across worm_reset so the wait survives the reset.
 *   sprayed       : flag so the breach sand-burst fires exactly once per breach.
 *   active        : 1 while this slot holds a live worm. */
typedef struct {
    Seg    segs[N_SEGS];
    float  hx, hy;          /* head position in cells */
    float  dir;             /* +1 right / -1 left */
    float  speed;
    WState state;
    float  swim_timer;      /* seconds until next breach */
    float  swim_phase;      /* how far along the swim wave */
    float  breach_x0;       /* head x where this breach began */
    float  mouth_open;      /* 0=closed 1=fully open */
    float  ripple_timer;
    float  respawn_timer;   /* counts up while dead */
    int    sprayed;         /* did this breach already throw sand? */
    int    active;
} Worm;

/* Scene — the whole simulation in one place. Keeping all state here means it
 * lives in exactly one spot, while individual functions still take only the
 * piece they need. The worms, ripples, and spray are fixed-size arrays with an
 * `active` flag per slot, so nothing is allocated once the program is running.
 *   terrain   : the dune surface.
 *   worms[]   : up to MAX_WORMS sandworms; n_worms of them are in play.
 *   ripples[] : up to MAX_RIPPLES expanding sand rings (cosmetic).
 *   spray[]   : up to MAX_SPRAY flung sand grains (cosmetic).
 *   speed     : base head speed; +/- changes it, each worm varies slightly.
 *   n_worms   : how many worm slots are in play; w/W change it.
 *   paused    : 1 freezes the simulation (drawing keeps going).
 *   theme     : which palette is active; 't' cycles it, kept here so reset and
 *               resize don't lose your choice. */
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

/* ── §4 logic — small helpers that only read and return a value ───────────── */

/* random float somewhere in [lo, hi) */
static float frand_range(float lo, float hi) {
    int steps = (int)((hi - lo) * 100.f + 0.5f);
    return lo + (float)(rand() % steps) * 0.01f;
}

/* random offset spread evenly above and below 0 */
static float rand_centered(int n, float scale) {
    int off = rand() % n - n / 2;
    return (float)off * scale;
}

/* coin flip: +1 or -1, used to pick which side a worm comes from */
static float rand_sign(void) {
    return (rand() % 2 == 0) ? 1.f : -1.f;
}

/* sand-surface row at column x (clamped to the screen) */
static int terrain_at(const HeightField *hf, float x, int cols) {
    int c = (int)(x + 0.5f);
    if (c < 0)    c = 0;
    if (c >= cols) c = cols - 1;
    return hf->row[c];
}

/* pick the body glyph for a segment from which way it leans toward the head:
   nearly vertical -> '|'/'!', nearly flat -> 'O'/'o'/'0', else a diagonal */
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

/* true once the whole worm has slid past a screen edge, body and all */
static int worm_off_field(const Worm *w, int cols) {
    float margin = (float)N_SEGS * SEG_LEN + RETIRE_MARGIN;
    return (w->dir > 0.f && w->hx > (float)cols + margin) ||
           (w->dir < 0.f && w->hx < -margin);
}

/* how far the worm is through its breach arc: 0 at the start, 1 at the end */
static float breach_t(const Worm *w) {
    float t = (w->hx - w->breach_x0) / (w->dir * BREACH_SPAN);
    return t < 0.f ? 0.f : t;
}

/* how high above the surface the head sits partway through the arc; a smooth
   hump that peaks halfway and reaches BREACH_HEIGHT rows there */
static float breach_height(float t) {
    return BREACH_HEIGHT * 4.f * t * (1.f - t);
}

/* mouth openness 0..1: fully open at the top of the arc, closing on either side */
static float mouth_openness(float t) {
    float d_apex = fabsf(t - 0.5f);
    return (d_apex < MOUTH_APEX_WIN) ? 1.f - d_apex / MOUTH_APEX_WIN : 0.f;
}

/* one sample of the dune height: three sine waves of shrinking size added up,
   a cheap stand-in for Perlin noise that gives rolling dunes */
static float dune_height(float x) {
    static const float freq [3] = { 2.1f, 5.3f, 11.7f };
    static const float amp  [3] = { 1.5f, 0.8f,  0.4f };
    static const float phase[3] = { 0.f,  0.9f,  0.4f };
    float h = 0.f;
    for (int k = 0; k < 3; k++)
        h += sinf(x * (float)M_PI * freq[k] + phase[k]) * amp[k];
    return h;
}

/* turn a (column, row) pair into a scrambled number; lets us scatter stars that
   stay put frame to frame instead of flickering randomly */
static unsigned int star_hash(int c, int r) {
    unsigned int h = (unsigned int)(c * 1234597u ^ r * 987659u);
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}

/* ── §5 effects — sand ripples and spray (cosmetic only) ──────────────────── */

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
        /* aim somewhere in the upper half-circle so grains fly up and out */
        float ang = (float)(rand() % 180) * (float)M_PI / 180.f;
        float spd = frand_range(3.f, 10.f);     /* grain speed (cells/sec) */
        spray[i].active   = 1;
        spray[i].x        = x + rand_centered(7, 0.5f);   /* scatter the start a bit */
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

/* ── §6 simulation — move the worms, then their bodies, once per tick ─────── */

/* put one worm back at the start: pick a side, a slightly random speed, and
   line its body up off-screen ready to swim in */
static void worm_reset(Worm *w, const HeightField *hf, float speed,
                       int cols, int rows, float dir) {
    float respawn = w->respawn_timer;  /* keep the respawn countdown across the wipe */
    memset(w, 0, sizeof *w);
    w->respawn_timer = respawn;
    w->active      = 1;
    w->dir         = dir;
    w->speed       = speed * frand_range(0.85f, 1.15f);   /* vary it so worms drift apart */
    w->state       = WS_SWIM;
    w->swim_timer  = frand_range(SWIM_TIME_MIN, SWIM_TIME_MAX);
    w->swim_phase  = frand_range(0.f, 6.28f);             /* random spot on the swim wave */
    w->ripple_timer = 0.6f;

    /* start off-screen on the side it's heading from */
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

/* swimming phase: ride the underground wave, leave a ripple when skimming near
   the surface, and count down to the next breach */
static void worm_swim(Worm *w, const HeightField *hf, Ripple *ripples,
                      float dt, int cols) {
    float surf = (float)terrain_at(hf, w->hx, cols);
    w->hy          = surf + SWIM_DEPTH + sinf(w->swim_phase) * SWIM_AMP;
    w->swim_phase += SWIM_FREQ * w->speed * dt;
    w->mouth_open  = 0.f;

    /* close to the surface? shed a ripple now and then */
    float depth = w->hy - surf;
    if (depth < SWIM_DEPTH * RIPPLE_SHED_FRAC) {
        w->ripple_timer -= dt;
        if (w->ripple_timer <= 0.f) {
            ripple_spawn(ripples, w->hx);
            w->ripple_timer = frand_range(0.45f, 0.85f);
        }
    }

    /* time to breach? switch phases and burst a cluster of ripples */
    w->swim_timer -= dt;
    if (w->swim_timer <= 0.f) {
        w->state     = WS_BREACH;
        w->breach_x0 = w->hx;
        w->sprayed   = 0;
        ripple_spawn(ripples, w->hx);
        ripple_spawn(ripples, w->hx);
        ripple_spawn(ripples, w->hx + w->dir * 3.f);
    }
}

/* breaching phase: arc up over the surface, open the mouth near the top, throw
   sand once as the head breaks through, and drop back to swimming at the end */
static void worm_breach(Worm *w, const HeightField *hf, Spray *spray, int cols) {
    float t = breach_t(w);
    if (t >= 1.f) {
        /* arc done — go back under */
        w->state      = WS_SWIM;
        w->swim_timer = frand_range(SWIM_TIME_MIN, SWIM_TIME_MAX);
        w->mouth_open = 0.f;
    } else {
        float surf    = (float)terrain_at(hf, w->hx, cols);
        w->hy         = surf - breach_height(t);
        w->mouth_open = mouth_openness(t);

        /* throw sand once, right as the head breaks the surface */
        if (!w->sprayed && t > SPRAY_T_LO && t < SPRAY_T_HI) {
            spray_burst(spray, w->hx, surf, SPRAY_BURST_N);
            w->sprayed = 1;
        }
    }
}

/* drag the body to follow the head: snap the first bead to the head, then walk
   the rest, pulling each one in until it sits SEG_LEN behind the bead ahead */
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

    w->hx += w->dir * w->speed * dt;                 /* move the head forward */
    if (w->state == WS_SWIM) worm_swim(w, hf, ripples, dt, cols);
    else                     worm_breach(w, hf, spray, cols);

    if (worm_off_field(w, cols)) { w->active = 0; return; }

    relax_chain(w);                                  /* body follows the head */
    (void)rows;
}

/* a dead worm waits a moment, then comes back in from a random side */
static void worm_try_respawn(Worm *w, const HeightField *hf, float speed,
                             int cols, int rows, float dt) {
    w->respawn_timer += dt;
    if (w->respawn_timer >= RESPAWN_DELAY_S)
        worm_reset(w, hf, speed, cols, rows, rand_sign());
}

/* one simulation step: move every worm, then advance the ripples and spray */
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

/* ── §7 init — build the scene fresh (startup, reset, and resize) ─────────── */

static void terrain_init(HeightField *hf, int cols, int rows) {
    hf->surface_row = rows / 2;                          /* surface sits mid-screen */
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
    /* worms enter from alternating sides, each a bit later than the last */
    for (int i = 0; i < s->n_worms; i++) {
        float dir = (i % 2 == 0) ? 1.f : -1.f;
        worm_reset(&s->worms[i], &s->terrain, s->speed, cols, rows, dir);
        s->worms[i].swim_timer += (float)i * WORM_STAGGER_S;   /* stagger their breaches */
    }
}

/* ── §8 render — turn the scene into characters on screen ─────────────────── */

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

/* draw the body, tail first so the head ends up on top; segments above ground
   get a slope glyph, those still buried show as a faint dot */
static void draw_worm_body(const Worm *w, const HeightField *hf, int cols, int rows) {
    for (int i = N_SEGS - 1; i >= 1; i--) {
        int sr = (int)(w->segs[i].y + 0.5f);
        int sc = (int)(w->segs[i].x + 0.5f);
        if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) continue;

        /* which way this segment leans toward the head, for picking the glyph */
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
            /* mostly-flat segment? add a row below so the body looks thick */
            if (!is_tail && fabsf(dx) > fabsf(dy) * THICKEN_RATIO && sr + 1 < surf && sr + 1 < rows)
                mvaddch(sr + 1, sc, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(CP_WORM_TOP) | at);
        } else {
            /* still buried: use the bright body color (not dim) so it shows
               through the sand instead of washing out */
            attr_t at = is_tail ? A_NORMAL : A_BOLD;
            attron(COLOR_PAIR(CP_WORM_TOP) | at);
            mvaddch(sr, sc, (chtype)(unsigned char)(i % 3 == 0 ? 'o' : '.'));
            attroff(COLOR_PAIR(CP_WORM_TOP) | at);
        }
    }
}

/* draw the head: above ground while breaching it's a little open-mouth sprite
   (top lip, mouth, bottom lip, width from mouth_open); buried it's just a dot */
static void draw_worm_head(const Worm *w, const HeightField *hf, int cols, int rows) {
    int hr = (int)(w->hy + 0.5f);
    int hc = (int)(w->hx + 0.5f);
    if (hc < 0 || hc >= cols || hr < 0 || hr >= rows) return;

    int  surf        = terrain_at(hf, w->hx, cols);
    int  above_ground = (hr < surf);

    if (above_ground) {
        /* how many chars the mouth spreads to each side */
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

/* draw the two status bars: stats along the top, the key legend along the
   bottom. Each bar is filled blank first, then text is clipped to fit so a
   narrow terminal can't overflow or wrap. */
static void draw_hud(float speed, int n_active, int n_total,
                     const char *theme, int paused, int rows, int cols) {
    if (cols < 1 || rows < 1) return;

    /* top row: title on the left, stats on the right */
    char left[24], right[72];
    snprintf(left,  sizeof left,  " DUNE SANDWORM ");
    snprintf(right, sizeof right, " spd:%.0f  worms:%d/%d  theme:%s  %s ",
             speed, n_active, n_total, theme,
             paused ? "PAUSED" : "running");
    int rx = cols - (int)strlen(right);          /* column where the stats start */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(0, c, ' ');
    if (rx >= 0) {
        mvprintw(0, 0,  "%.*s", rx, left);       /* title, clipped before the stats */
        mvprintw(0, rx, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, right);    /* too narrow: show stats only */
    }
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* bottom row: every key you can press */
    int brow = rows - 1;
    if (brow > 0) {
        attron(COLOR_PAIR(CP_HINT) | A_BOLD);
        for (int c = 0; c < cols; c++) mvaddch(brow, c, ' ');
        mvprintw(brow, 0, "%.*s", cols,
                 " q:quit  p:pause  r:reset  +/-:speed  spc:breach  w/W:worms  t:theme ");
        attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
    }
}

/* ── §9 events — signal handlers (key handling lives in main) ─────────────── */
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
