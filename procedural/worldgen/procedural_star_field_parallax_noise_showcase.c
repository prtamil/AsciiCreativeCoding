/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_star_field_parallax_noise_showcase.c
 *   — Procedural infinite star field with depth-cued parallax scroll
 *     and a fractional-Brownian-motion nebula backdrop.
 *
 * DEMO: Four star "sheets" stacked at different virtual depths scroll
 *       across the terminal at four different speeds. The closest sheet
 *       (foreground) flies past quickly with bright '*' / 'O' glyphs;
 *       the deepest sheet creeps along almost imperceptibly with faint
 *       '.' specks. The brain reads the speed differential as DEPTH —
 *       the scene appears three-dimensional even though every glyph is
 *       drawn at integer cell coordinates with no z-buffer.
 *
 *       No stars are stored. Each on-screen cell asks a hash function
 *       "is there a star at world coordinates (wx, wy) on layer L?",
 *       and the answer is deterministic and infinite — fly the camera
 *       for an hour and the same hash returns the same star at the
 *       same world coord every time. Cycle through four patterns:
 *
 *         STARFIELD  pure parallax stars
 *         TWINKLE    each star pulses with its own hashed phase
 *         NEBULA     stars over a slow-drifting fBm cloud backdrop
 *         WARP       stars stretched into trailing streaks (5x speed)
 *
 * Study alongside: ../fields/perin_noise_flow_showcase.c — that file
 *       uses a Perlin field to STEER particles. This file uses a hash
 *       to PLACE stars and uses Perlin/fBm only for the nebula. The
 *       distinction is "noise as motion" vs "noise as content".
 *
 * Section map:
 *   §1 config    — N_LAYERS, layer speeds & density, glyphs, themes
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — HUD reserved + 10 themes (4 star + 4 nebula tints)
 *   §5 starfield — int hash, star_at(wx,wy,L), perlin2d + fbm
 *   §6 scene     — Camera, Scene state, scene_tick (camera advance)
 *   §7 screen    — per-cell layer scan, pattern dispatch, HUD
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset camera to origin, reseed
 *   n / N      next pattern  (STARFIELD → TWINKLE → NEBULA → WARP → ...)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster scroll
 *   -          slower scroll
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/procedural_star_field_parallax_noise_showcase.c \
 *     -o star_field -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two pieces composed together.
 *
 *                  (1) Parallax scrolling — the depth illusion.
 *                      Stack N independent "sheets" of stars at virtual
 *                      depths z_0 < z_1 < ... < z_{N-1}. Each sheet
 *                      scrolls horizontally at a speed proportional to
 *                      1/z (closer sheets sweep faster). Render every
 *                      sheet onto the same 2-D screen, foreground last
 *                      (or first, with early-out — see §7). The visual
 *                      cortex interprets the speed differential as
 *                      depth even with no actual 3-D geometry. This is
 *                      the same trick old side-scrolling video games
 *                      used for sky / mountains / trees / ground.
 *
 *                  (2) Procedural star placement — hash-based content.
 *                      Stars are NOT stored. For each (world_x, world_y,
 *                      layer) triple we compute h = hash3(wx, wy, L)
 *                      and ask "is h % LAYER_DENSITY[L] == 0?" If yes,
 *                      a star exists there; the same hash also encodes
 *                      its glyph, colour tint, and twinkle phase.
 *                      Because hash is a pure function of its inputs,
 *                      the same world cell always answers the same way,
 *                      so stars appear nailed to their world positions
 *                      as the camera scrolls past — the world feels
 *                      infinite and consistent, with O(0) memory.
 *
 *                  Plus an optional Perlin/fBm nebula backdrop: every
 *                  empty cell samples a slow-scrolling 4-octave fBm
 *                  field, mapped through a density ramp to ASCII glyphs.
 *
 * Data-structure : NONE for the star field itself — that is the whole
 *                  point of procedural generation. The only state is:
 *                    - 256-entry permutation table for Perlin (nebula)
 *                    - Camera position (cx, cy) and velocity (vx, vy)
 *                    - Pattern / theme / speed selectors
 *                  No grid array, no entity pool, no spatial index.
 *
 * Rendering      : ASCII only. Per cell, scan layers front-to-back and
 *                  break on first hit (foreground wins). Glyph and
 *                  colour are derived from the SAME hash that decided
 *                  the star exists, so they're stable as the camera
 *                  scrolls. Layer index drives glyph "size":
 *                    layer 0 (close)  : '*' 'O' '+' 'o'   (bright, BOLD)
 *                    layer 1          : '*' '+' 'o' '.'   (medium)
 *                    layer 2          : '.' '+' '.' ','   (dim)
 *                    layer 3 (far)    : '.' '`' '.' '\''  (faint, A_DIM)
 *                  TWINKLE multiplies brightness by sin(2π·t·F + phase);
 *                  WARP swaps to streak glyphs '=' '-' '~' and bumps
 *                  scroll speed; NEBULA paints empty cells from fBm.
 *
 * Performance    : O(W·H·N_LAYERS) hashes per frame — for 240×80 with
 *                  4 layers that's 76 800 hash3 calls per frame, each
 *                  about 6 multiplies + 4 xors. At 60 fps that's ≈
 *                  4.6 M hash ops/sec — ~0.1% of a modern CPU. NEBULA
 *                  adds W·H Perlin evaluations per frame (~1.2 M
 *                  ops/sec) — still inconsequential. There is no
 *                  per-tick allocation and no I/O inside the loop;
 *                  the renderer is a pure function of camera state.
 *
 * References     : • Perlin, K. (1985)  An Image Synthesizer, SIGGRAPH
 *                    https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *                  • Perlin, K. (2002)  Improving Noise (quintic fade)
 *                    https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *                  • Wikipedia — Parallax scrolling
 *                    https://en.wikipedia.org/wiki/Parallax_scrolling
 *                  • Wikipedia — Perlin noise
 *                    https://en.wikipedia.org/wiki/Perlin_noise
 *                  • Inigo Quilez — Hash without Sine, smooth integer noise
 *                    https://iquilezles.org/articles/morenoise/
 *                  • Red Blob Games — Noise functions and map generation
 *                    https://www.redblobgames.com/articles/noise/introduction.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two ideas, combined: depth-from-speed and content-from-hash. Stars at
 * different "depths" scroll at different speeds — the eye reads the
 * speed difference as 3-D, even though the screen is flat. AND those
 * stars are not stored anywhere; they are RECOMPUTED every frame from
 * a hash function, so the world is infinite, deterministic, and free.
 * Memory cost is constant; world size is unbounded.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine you are looking out the side window of a moving train. The
 * fence right next to the tracks streaks past so fast it blurs. The
 * trees in the middle distance pass by at a comfortable speed. The
 * mountains on the horizon barely move. SAME train, SAME velocity —
 * the visual speed of a thing depends purely on its distance.
 *
 * Now replace the fence/trees/mountains with four invisible sheets of
 * star stickers, all parallel to the screen. Slide each sheet sideways
 * at its own speed. The closest sheet has fewer, brighter stars
 * (because near things look big); the farthest sheet has many, faint
 * stars (because far things blur into a sprinkle). That is the
 * algorithm. Every visible "depth" effect comes from the speed and
 * brightness differences between sheets.
 *
 * Where do the stars on a sheet live? Nowhere — they are summoned on
 * demand. For each sheet and each (x, y) in WORLD coordinates, ask a
 * hash function "is there a star here?" and the hash either says yes
 * or no in O(1) time. Same (x, y, sheet) → same answer, every time,
 * forever. The world is a function, not a database.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Pick LAYER_SPEEDS[N_LAYERS] (decreasing — closest layer
 *     first). Pick LAYER_DENSITY[N_LAYERS] (higher = sparser; "1 in
 *     LAYER_DENSITY[L] world cells of layer L hosts a star").
 *  2. EACH FRAME:
 *     a. Advance the camera: cam.x += cam.vx · dt; cam.y += cam.vy · dt.
 *     b. Erase the screen.
 *     c. For each on-screen cell (sx, sy):
 *          For each layer L from 0 (front) up to N_LAYERS-1 (back):
 *            wx = floor(sx + cam.x · LAYER_SPEEDS[L])
 *            wy = floor(sy + cam.y · LAYER_SPEEDS[L])
 *            h  = hash3(wx, wy, L)         // 32-bit deterministic
 *            if (h % LAYER_DENSITY[L]) == 0:
 *              draw the star — glyph, colour, brightness all encoded
 *              in different bits of h. Break (foreground wins).
 *          If still empty AND pattern == NEBULA:
 *            sample fBm at slow-scrolling world coords; map to glyph.
 *  3. PATTERN MODIFIERS (applied during step 2c):
 *     - TWINKLE : multiply per-star brightness by 0.5 + 0.5·sin(2π·t·f
 *                 + phase), where phase is bits [24..31] of h.
 *     - WARP    : 5× scroll speed, swap glyphs to streak set.
 *     - NEBULA  : draw fBm in empty cells (steps within 2c above).
 *  4. RENDER HUD, present, sleep until next frame.
 *
 * KEY FORMULAS
 * ────────────
 *  Screen → world (per layer L):
 *    world_x = floor(screen_x + camera_x · layer_speed[L])
 *    world_y = floor(screen_y + camera_y · layer_speed[L])
 *
 *  Star existence test (layer L, world cell (wx, wy)):
 *    h = hash3(wx, wy, L)
 *    star_present = (h mod LAYER_DENSITY[L]) == 0
 *
 *  Star attributes (decoded from same hash):
 *    glyph_idx = (h >>  8) & 3        // 0..3 — index into LAYER_GLYPHS[L]
 *    color_idx = (h >> 16) & 3        // 0..3 — index into theme palette
 *    phase     = (h >> 24) / 255 · 2π // twinkle offset
 *
 *  Twinkle (PATTERN_TWINKLE):
 *    brightness = 0.5 + 0.5 · sin(2π · t · TWINKLE_HZ + phase)
 *    where t is wall-clock seconds. brightness in [0, 1] selects:
 *      brightness < 0.30 : skip cell (off cycle)
 *      brightness < 0.65 : A_DIM glyph
 *      else              : A_BOLD glyph
 *
 *  Hash function (any 3-int → 32-bit, mix-of-multiplies):
 *    h  = wx · 73856093 ^ wy · 19349663 ^ L · 83492791
 *    h ^= h >> 16; h *= 0x85ebca6b
 *    h ^= h >> 13; h *= 0xc2b2ae35
 *    h ^= h >> 16
 *
 *  Nebula (PATTERN_NEBULA), per empty cell:
 *    nx = sx + cam.x · NEBULA_SCROLL
 *    ny = sy + cam.y · NEBULA_SCROLL
 *    n  = fbm(nx · NEBULA_SCALE, ny · NEBULA_SCALE, 4 octaves)
 *    map n through a 4-step density ramp to glyph + tint.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • HASH QUALITY. The 3 large odd primes (73856093, 19349663, 83492791)
 *    are the standard "spatial-hash" trio from Teschner et al. 2003.
 *    Replacing them with arbitrary numbers tends to introduce visible
 *    diagonal stripes — the hash output becomes correlated along
 *    wx + wy. The avalanching multiply-shift after the XOR is what
 *    breaks the residual structure.
 *
 *  • LAYER ORDER. Layer 0 is the FRONT (fastest, brightest). Iterate
 *    front-to-back with early-out: the first star we find wins. If you
 *    iterate back-to-front and overwrite, slower hidden stars "leak
 *    through" foreground gaps and ruin the depth cue.
 *
 *  • FLOOR ON NEGATIVE. As the camera moves toward +x, world_x becomes
 *    progressively LARGER. If the camera ever moves toward -x (or the
 *    screen y exceeds cam.y), world coordinates can go negative. Use
 *    floorf() not (int) cast — the latter truncates toward zero and
 *    introduces a 1-cell discontinuity at the origin. floorf is correct
 *    for all signs.
 *
 *  • CAMERA OVERFLOW. (int)floorf(huge_float) is undefined when the
 *    float exceeds INT_MAX. With cam.x growing at 8 cells/sec, INT_MAX
 *    is reached after ~8.5 years of continuous run. Acceptable for an
 *    interactive demo; not for a saved-state simulator.
 *
 *  • LAYER SPEED CHOICE. If two layers have the SAME speed, they paint
 *    at the same rate and no parallax happens between them — they look
 *    like one fat layer. Speeds should be roughly geometric, e.g.
 *    1.0 / 0.45 / 0.18 / 0.06 — each layer noticeably slower than the
 *    one in front of it.
 *
 *  • NEBULA SCROLL VS LAYER SCROLL. The nebula is conceptually behind
 *    layer 3 (deepest). Its scroll speed must therefore be the SLOWEST.
 *    Default NEBULA_SCROLL is 0.03 — half of the deepest star layer.
 *
 *  • INFINITE-LOOK BREAKS AT 256. Our Perlin permutation table is
 *    256 entries (with a duplicate copy → 512). The noise pattern
 *    REPEATS every 256 noise-coord units. With NEBULA_SCALE=0.04 that
 *    is a 6 400-cell period — far larger than any terminal, but visible
 *    if you fly for hours. To extend the period, multiply coords by an
 *    irrational so they never realign.
 *
 *  • WARP STREAK OVERSHOOT. In WARP mode the streak length is fixed at
 *    3 cells. If a streak extends past the right edge, the truncated
 *    chars vanish — that is correct; do not wrap or clip explicitly,
 *    the bounds check inside the draw loop handles it.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). The image freezes. Press space again. The image
 *    resumes from EXACTLY where it stopped — no jump. Verifies the
 *    fixed-step accumulator.
 *
 *  • Press 'r' at any time. The camera snaps to the origin and the
 *    SAME star pattern that was visible at startup re-appears (as long
 *    as you have not changed the seed). Verifies hash determinism.
 *
 *  • In STARFIELD pattern, count the rough star density on the
 *    foreground layer's flow — should be ≈ 1 / LAYER_DENSITY[0] ≈
 *    1 in 22 cells, i.e. about 4–5% of the screen. The deepest layer
 *    should be denser (≈ 1 in 7), but its glyphs are dim '.', so it
 *    looks like a faint sprinkle. If layer 0 looks DENSER than layer
 *    3, your front-to-back scan order is reversed.
 *
 *  • Switch to TWINKLE. Watch one bright star for 3 seconds. It
 *    should fade in and out smoothly with a period of ≈ 2 s and a
 *    different phase from its neighbours. If every star pulses in
 *    sync, the per-star phase isn't being read from the hash.
 *
 *  • Switch to NEBULA. The empty regions fill with smoothly-shaded
 *    cloud glyphs that drift slowly leftward (camera goes right, world
 *    appears to slide left). The clouds should be SLOWER than even
 *    the deepest star layer. If they look faster, NEBULA_SCROLL is
 *    too high.
 *
 *  • Switch to WARP. The closest stars become 3-cell streaks; deeper
 *    layers stay as small marks. Press 'r' — the streaks vanish (no
 *    history kept; re-derived every frame). This proves the streaks
 *    are not painted into a buffer that survives reset.
 *
 *  • At any pattern, scroll speed +/-: with '+' the parallax should
 *    grow more dramatic (foreground blurs while background creeps);
 *    with '-' the whole field nearly freezes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    /* Hard upper bound on screen dimensions we are willing to render
     * stars across. Anything beyond is clipped. The arrays below do
     * not depend on these — there is no per-cell storage at all. */
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,

    /* Number of parallax sheets. Four is the visual sweet spot — each
     * additional sheet adds depth perception but also clutter. */
    N_LAYERS            =   4,

    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* Scroll speed in CELLS PER SECOND for the FOREGROUND layer.
     * Other layers scale it down by LAYER_SPEEDS[L]. Default 8 makes
     * the foreground sweep across an 80-col screen in ~10 seconds. */
    SCROLL_SPEED_MIN    =   1,
    SCROLL_SPEED_DEF    =   8,
    SCROLL_SPEED_MAX    =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_STAR_BASE      =   3,    /* +0..+3 = 4 star tints           */
    PAIR_NEBULA_BASE    =   7,    /* +0..+3 = 4 nebula tints         */
    PAIR_FLASH          =  11,    /* reset flash                     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/*
 * LAYER_SPEEDS — fraction of the user's scroll-speed knob that this
 * layer actually scrolls at. Layer 0 is the FRONT (closest, fastest),
 * layer N_LAYERS-1 is the BACK. A roughly geometric falloff (each
 * layer ~0.4× the previous) gives the cleanest depth illusion.
 *
 * If you flatten this — say, set every entry to 1.0 — the parallax
 * disappears and all layers look like one indistinguishable cloud.
 */
static const float LAYER_SPEEDS[N_LAYERS] = {
    1.00f,   /* layer 0: foreground — full speed                      */
    0.45f,   /* layer 1: mid-near                                     */
    0.18f,   /* layer 2: mid-far                                      */
    0.06f,   /* layer 3: deep background — barely moves               */
};

/*
 * LAYER_DENSITY — "one in N world cells of this layer hosts a star."
 * Smaller = denser. Foreground sparser (fewer big bright stars), back-
 * ground denser (many faint specks) — this matches how galactic depth
 * actually compresses: more distant stars accumulate per solid angle.
 */
static const int LAYER_DENSITY[N_LAYERS] = {
    22,    /* foreground: one star per ~22 cells                      */
    16,
    11,
     7,    /* deep field: one star per ~7 cells (heavy sprinkle)      */
};

/*
 * LAYER_GLYPHS — four glyph variants per layer, picked by 2 bits of
 * the per-star hash. Foreground uses larger / more visually-weighted
 * glyphs; deeper layers use lighter marks. The eye reads "size" from
 * glyph weight and uses it as an additional depth cue on top of the
 * speed differential.
 */
static const char LAYER_GLYPHS[N_LAYERS][4] = {
    { '*', 'O', '+', 'o' },     /* layer 0: bright, BOLD             */
    { '*', '+', 'o', '.' },     /* layer 1                            */
    { '.', '+', '.', ',' },     /* layer 2: dim                       */
    { '.', '`', '.', '\'' },    /* layer 3: A_DIM                     */
};

/*
 * WARP-pattern glyph swap. Foreground stars become streak chars;
 * deeper layers keep their normal glyphs (they barely move so a streak
 * would be misleading).
 */
static const char WARP_GLYPHS[4] = { '=', '-', '~', '>' };

#define WARP_SPEED_MULT     5.0f   /* scroll-speed multiplier in WARP */
#define WARP_STREAK_LEN     3      /* trailing chars per fg star      */

/*
 * Twinkle period. Smaller = faster pulsing. 2 s feels like real
 * atmospheric scintillation; below 0.5 s it looks like a strobe.
 */
#define TWINKLE_HZ          0.5f
#define TWINKLE_OFF_THRESH  0.30f
#define TWINKLE_BOLD_THRESH 0.65f

/*
 * Nebula (fBm) parameters.
 *   NEBULA_SCALE  — noise period in world units. Smaller value = bigger,
 *                   smoother clouds. 0.025 → ~40-cell features.
 *   NEBULA_SCROLL — how fast the nebula scrolls relative to the camera.
 *                   Smaller than the deepest star layer (0.06) so the
 *                   clouds read as the FARTHEST element in the scene.
 *   NEBULA_*_THRESH — density-ramp cutoffs, mapping fbm output to glyph.
 *   FBM_OCTAVES   — octaves stacked; 4 is the standard sweet spot.
 */
#define NEBULA_SCALE        0.025f
#define NEBULA_SCROLL       0.03f
#define FBM_OCTAVES         4
#define NEBULA_DOT_THRESH   0.50f
#define NEBULA_MID_THRESH   0.62f
#define NEBULA_HI_THRESH    0.74f

/*
 * Pattern — four ways of rendering the same parallax field. Cycle
 * with n / p.
 */
typedef enum {
    PATTERN_STARFIELD = 0,
    PATTERN_TWINKLE   = 1,
    PATTERN_NEBULA    = 2,
    PATTERN_WARP      = 3,
    N_PATTERNS        = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_STARFIELD: return "STARFIELD";
    case PATTERN_TWINKLE:   return "TWINKLE  ";
    case PATTERN_NEBULA:    return "NEBULA   ";
    case PATTERN_WARP:      return "WARP     ";
    default:                return "?        ";
    }
}

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 * Each theme provides 4 STAR tints (one per star colour bucket) and
 * 4 NEBULA tints (one per density band). PAIR_HUD/HINT/FLASH stay
 * theme-independent.
 */
typedef struct {
    const char *name;
    short       star  [4];
    short       nebula[4];
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name      star{0,1,2,3}              nebula{0,1,2,3}            */
    { "DEFAULT", { 231, 226, 117,  33 }, {  17,  19,  62, 105 } },
    { "MATRIX",  { 230, 226, 118,  46 }, {  22,  28,  34,  40 } },
    { "NOVA",    { 231, 219, 201, 129 }, {  53,  54,  91, 165 } },
    { "MONO",    { 254, 250, 244, 240 }, { 234, 236, 238, 240 } },
    { "OCEAN",   { 231, 159,  51,  39 }, {  17,  18,  24,  31 } },
    { "FIRE",    { 231, 226, 208, 196 }, {  52,  88, 124, 160 } },
    { "EARTH",   { 230, 222, 173, 100 }, {  58,  64, 100, 137 } },
    { "FOREST",  { 231, 156, 118,  64 }, {  22,  28,  29,  65 } },
    { "DESERT",  { 230, 222, 173, 130 }, {  94,  95, 130, 137 } },
    { "ARCTIC",  { 231, 195, 159,  39 }, {  17,  18,  19,  24 } },
};

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

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_STAR_BASE   + i), t->star  [i], -1);
            init_pair((short)(PAIR_NEBULA_BASE + i), t->nebula[i], -1);
        }
    } else {
        /* 8-color fallback — distinct hues that work on every term. */
        static const short fb_star[4]   = { COLOR_WHITE,   COLOR_YELLOW,
                                            COLOR_CYAN,    COLOR_BLUE };
        static const short fb_nebula[4] = { COLOR_BLUE,    COLOR_BLUE,
                                            COLOR_MAGENTA, COLOR_CYAN };
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_STAR_BASE   + i), fb_star  [i], -1);
            init_pair((short)(PAIR_NEBULA_BASE + i), fb_nebula[i], -1);
        }
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  starfield — hash, star_at, perlin/fbm                              */
/* ===================================================================== */

/*
 * hash3 — stateless 3-int → 32-bit avalanche hash.
 *
 * The three multipliers (73856093, 19349663, 83492791) are the spatial-
 * hash primes from Teschner et al. (2003), specifically chosen to
 * decorrelate adjacent integer triples. The post-XOR multiply-shift
 * sequence is the public-domain "splitmix64-lite" finaliser, which
 * removes residual structure introduced by the additive XOR.
 *
 * Why not just rand()? rand() has STATE — calling it for "is there a
 * star at (wx, wy, L)" would force you to seed on every query and
 * permute, which is both slower and not stable across reruns. A pure
 * function gives the same answer every time for free.
 *
 * Equivalent in spirit to Inigo Quilez's "hash without sine" idea.
 */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/*
 * star_at — does layer L have a star at world cell (wx, wy)?
 *
 * Returns true iff the hash-modulo test passes. On true, fills *out_h
 * with the full hash so the caller can decode glyph / colour / phase
 * from the SAME bits — this guarantees a star's appearance is stable
 * as the camera moves: same world coord → same hash → same look.
 */
static inline bool star_at(int wx, int wy, int L, uint32_t *out_h)
{
    uint32_t h = hash3(wx, wy, L);
    *out_h = h;
    return (h % (uint32_t)LAYER_DENSITY[L]) == 0u;
}

/*
 * Perlin-noise scaffolding for the NEBULA backdrop. The same code as
 * in perin_noise_flow_showcase.c — copied inline per the project's
 * self-contained-file rule. See that file for the full derivation.
 *
 * perm[]   : 256-entry permutation, duplicated to 512 to avoid mod
 * fade(t)  : 6t⁵-15t⁴+10t³ — Perlin's 2002 quintic improvement
 * grad     : 8 lattice gradients chosen by hash low bits
 * perlin2d : returns roughly [-1, 1]
 */
static uint8_t perm[512];

static void perm_shuffle(void)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }

static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    float u = fade_q(x);
    float v = fade_q(y);

    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;

    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);

    return lerp_f(
        lerp_f(n00, n10, u),
        lerp_f(n01, n11, u),
        v);
}

/*
 * fbm — fractional Brownian motion. Sums FBM_OCTAVES of Perlin noise,
 * each octave doubling the frequency and halving the amplitude. The
 * result is normalised to roughly [0, 1] by dividing by the cumulative
 * amplitude — keeping the 4-step density ramp's thresholds meaningful.
 */
static float fbm2(float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* ===================================================================== */
/* §6  scene — camera + state + tick                                      */
/* ===================================================================== */

/*
 * Camera — continuous (float) position in world units, plus a velocity
 * that the user can scale up / down with +/-. The default direction is
 * pure +x ("flying right through the field"); a small +y component is
 * blended in so the parallax is visible on BOTH axes.
 *
 *   x, y   : current camera position; advanced by velocity each tick
 *   vx, vy : velocity in cells/sec — multiplied by LAYER_SPEEDS[L] at
 *            sample time
 */
typedef struct {
    float x, y;
    float vx, vy;
} Camera;

/*
 * Scene — all the state that survives across ticks. There is no per-
 * cell array: the rendering function reads camera + pattern + theme
 * and computes everything else from the hash / Perlin functions.
 *
 *   cam              : where the camera is
 *   paused           : freeze advance
 *   speed            : 1..SCROLL_SPEED_MAX, multiplies cam.v on tick
 *   current_theme    : 0..N_THEMES-1
 *   current_pattern  : STARFIELD / TWINKLE / NEBULA / WARP
 *   time_secs        : accumulator for TWINKLE phase
 *   flash_t          : 0..1 reset-flash intensity, decays each tick
 */
typedef struct {
    Camera  cam;
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   time_secs;
    float   flash_t;
} Scene;

/*
 * scene_reset — snap the camera to the origin, reseed the Perlin
 * permutation, raise the flash. This is the "press r" behaviour and
 * is also the one-time init-the-world call.
 */
static void scene_reset(Scene *s)
{
    s->cam.x = 0.0f;
    s->cam.y = 0.0f;
    /* Default velocity: dominantly rightward, with a small downward
     * component so vertical parallax is also visible. The user's speed
     * knob is applied later as a SCALAR multiplier of this direction. */
    s->cam.vx = 1.00f;
    s->cam.vy = 0.18f;
    s->time_secs = 0.0f;
    s->flash_t   = 1.0f;
    perm_shuffle();
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SCROLL_SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_STARFIELD;
    scene_reset(s);
}

/*
 * scene_tick — advance the camera and any time-driven state. No
 * rendering and no per-cell work happens here; this is just the
 * continuous-state update step of the fixed-timestep loop.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Effective scroll multiplier: knob × WARP boost when applicable. */
    float speed_mul = (float)s->speed;
    if (s->current_pattern == PATTERN_WARP) speed_mul *= WARP_SPEED_MULT;

    s->cam.x += s->cam.vx * speed_mul * dt;
    s->cam.y += s->cam.vy * speed_mul * dt;

    s->time_secs += dt;
    s->flash_t   *= expf(-4.0f * dt);   /* ~0.25 s decay */
}

/* ===================================================================== */
/* §7  screen — per-cell layer scan, pattern dispatch, HUD                */
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

/*
 * draw_star_at — paint the glyph + colour for ONE detected star. Fac-
 * tored out so the four patterns share the colour/glyph encoding logic.
 *
 *   sy, sx       : where on the screen
 *   layer        : 0..N_LAYERS-1
 *   h            : the full 32-bit hash returned by star_at()
 *   pattern      : determines glyph table + brightness modifier
 *   time_secs    : for TWINKLE
 *
 * Returns false if the star ended up "off" (TWINKLE pattern,
 * brightness below TWINKLE_OFF_THRESH) — the caller can then continue
 * scanning deeper layers behind it.
 */
static bool draw_star_at(int sy, int sx, int layer, uint32_t h,
                         Pattern pattern, float time_secs,
                         int rows, int cols)
{
    int color_idx = (int)((h >> 16) & 3u);
    int glyph_idx = (int)((h >>  8) & 3u);

    /* Default attribute by layer — depth cue via brightness. */
    int attr = A_NORMAL;
    if      (layer == 0) attr = A_BOLD;
    else if (layer == 3) attr = A_DIM;

    /* Glyph: pattern-specific. */
    char glyph;
    if (pattern == PATTERN_WARP && layer == 0) {
        glyph = WARP_GLYPHS[glyph_idx];
    } else {
        glyph = LAYER_GLYPHS[layer][glyph_idx];
    }

    /* TWINKLE: per-star phase modulates brightness. Off-cycle stars
     * return false so layers behind them can show through. */
    if (pattern == PATTERN_TWINKLE) {
        float phase = ((float)((h >> 24) & 0xFFu) / 255.0f) * 2.0f * (float)M_PI;
        float b = 0.5f + 0.5f * sinf(2.0f * (float)M_PI * TWINKLE_HZ
                                     * time_secs + phase);
        if (b < TWINKLE_OFF_THRESH)        return false;
        else if (b < TWINKLE_BOLD_THRESH)  attr = A_DIM;
        else                                attr = A_BOLD;
    }

    int pair = PAIR_STAR_BASE + color_idx;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);

    /* WARP: paint a 3-cell trailing streak behind the foreground star.
     * The streak extends in the +x direction (the camera is moving in
     * +x, so the world appears to slide in -x — the star's previous
     * positions are at higher screen-x). Fades over the streak length. */
    if (pattern == PATTERN_WARP && layer == 0) {
        for (int d = 1; d <= WARP_STREAK_LEN; d++) {
            int tx = sx + d;
            if (tx >= cols) break;
            char tg = (d == 1) ? '-' : (d == 2) ? '~' : '.';
            int  ta = (d == 1) ? A_BOLD : (d == 2) ? A_NORMAL : A_DIM;
            attron(COLOR_PAIR(pair) | ta);
            mvaddch(sy, tx, (chtype)(unsigned char)tg);
            attroff(COLOR_PAIR(pair) | ta);
        }
    }
    (void)rows;
    return true;
}

/*
 * draw_nebula_cell — paint the fBm backdrop at one empty cell. Sampled
 * at slow-scroll world coords so the clouds drift behind the deepest
 * star layer. 4-step density ramp:
 *
 *   n < NEBULA_DOT_THRESH : skip   (transparent)
 *   n < NEBULA_MID_THRESH : '.'    (faintest tint)
 *   n < NEBULA_HI_THRESH  : '*'
 *   else                  : '#'    (densest tint)
 */
static void draw_nebula_cell(int sy, int sx, float wx, float wy)
{
    float n = fbm2(wx * NEBULA_SCALE, wy * NEBULA_SCALE);

    char glyph;
    int  color_idx;
    int  attr = A_NORMAL;

    if (n < NEBULA_DOT_THRESH) return;

    if      (n < NEBULA_MID_THRESH) { glyph = '.'; color_idx = 0; attr = A_DIM;  }
    else if (n < NEBULA_HI_THRESH ) { glyph = '*'; color_idx = 1;                }
    else                            { glyph = '#'; color_idx = 2; attr = A_BOLD; }

    /* Use the fourth nebula tint on the very brightest peaks. */
    if (n > 0.86f) { color_idx = 3; attr = A_BOLD; glyph = '#'; }

    int pair = PAIR_NEBULA_BASE + color_idx;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * scene_draw — the heart of the renderer. For every visible cell,
 * scan layers front-to-back; the first hit wins. Empty cells in
 * NEBULA pattern fall through to draw_nebula_cell.
 *
 * Why front-to-back with break? Two reasons:
 *   1. Correctness — foreground stars must occlude deeper ones.
 *   2. Speed — typical cell terminates on layer 0 or 1 (foreground
 *      density ~5%, layer-1 density ~6%; combined ~11% of cells
 *      hit before reaching layer 2). The deeper, denser layers are
 *      only reached for the empty ~89% of cells, where they fill in
 *      the sprinkle.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    /* Reserve top 2 rows for HUD, bottom 1 for hint. */
    int top    = 2;
    int bottom = rows - 1;
    int width  = cols;
    if (width  > MAP_W_MAX) width  = MAP_W_MAX;
    if (bottom - top > MAP_H_MAX) bottom = top + MAP_H_MAX;

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {

            bool drew = false;

            for (int L = 0; L < N_LAYERS; L++) {
                /* Per-layer world coord — the FLOOR is essential here:
                 * (int) cast would discontinuity-jump at zero. */
                float wxf = (float)sx + s->cam.x * LAYER_SPEEDS[L];
                float wyf = (float)sy + s->cam.y * LAYER_SPEEDS[L];
                int   wx  = (int)floorf(wxf);
                int   wy  = (int)floorf(wyf);

                uint32_t h;
                if (!star_at(wx, wy, L, &h)) continue;

                if (draw_star_at(sy, sx, L, h,
                                 s->current_pattern, s->time_secs,
                                 rows, cols)) {
                    drew = true;
                    break;          /* foreground wins */
                }
                /* TWINKLE off-cycle: continue scanning behind. */
            }

            /* Empty cells: optional nebula backdrop. */
            if (!drew && s->current_pattern == PATTERN_NEBULA) {
                float wxf = (float)sx + s->cam.x * NEBULA_SCROLL;
                float wyf = (float)sy + s->cam.y * NEBULA_SCROLL;
                draw_nebula_cell(sy, sx, wxf, wyf);
            }
        }
    }

    /* Reset flash — sparse '*' overlay that fades quickly. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->time_secs * 1000.0f);
        attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
        for (int sy = top; sy < bottom; sy += 2) {
            for (int sx = 0; sx < width; sx += 2) {
                if (((sx ^ sy ^ seed) & 7) == 0)
                    mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    }
}

/*
 * screen_draw — clears, draws scene, draws HUD.
 *
 *   row 0 : title (left) + fps/Hz/state/speed (right)
 *   row 1 : pattern + theme + 4-colour palette swatch + camera coords
 *   bottom: key-hint strip (PAIR_HINT, A_BOLD)
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const char *state_str = s->paused
                          ? "PAUSED   "
                          : pattern_name(s->current_pattern);

    /* Row 0 right — primary status. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " STAR FIELD / PARALLAX ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — pattern, theme, palette swatches, camera. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 20;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " stars:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 7;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_STAR_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '*');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " neb:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 5;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_NEBULA_BASE + i;
        attron(COLOR_PAIR(p));
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p));
        x++;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  cam:(%7.1f,%6.1f)  layers:%d ",
             (double)s->cam.x, (double)s->cam.y, N_LAYERS);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
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

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset(s);                                break;

    case '=': case '+':
        if (s->speed < SCROLL_SPEED_MAX) s->speed *= 2;
        if (s->speed > SCROLL_SPEED_MAX) s->speed  = SCROLL_SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SCROLL_SPEED_MIN) s->speed  = SCROLL_SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

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
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* Fixed-step accumulator — same idiom as every other demo. */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
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
