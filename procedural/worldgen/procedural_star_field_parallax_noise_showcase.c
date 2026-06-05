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
 *       same world coord every time. Cycle through FIFTEEN modes — the
 *       first four render the parallax field, the rest add new motion,
 *       timed rings, meteors, a galaxy band, and full-screen field effects:
 *
 *         STARFIELD  pure parallax stars
 *         TWINKLE    each star pulses with its own hashed phase
 *         NEBULA     stars over a slow-drifting fBm cloud backdrop
 *         WARP       stars stretched into trailing streaks (5x speed)
 *         TUNNEL     radial fly-through — streaks shoot from the centre
 *         STARFALL   stars rain downward with vertical trails
 *         WORMHOLE   the whole field spirals into the centre
 *         REDSHIFT   stars tinted by depth (a near→far Doppler gradient)
 *         PULSAR     concentric brightness rings march outward
 *         SUPERNOVA  a few sites detonate into fading shock rings
 *         METEORS    sparse fast diagonal streaks over a calm field
 *         MILKYWAY   a luminous diagonal dust band of dense stars
 *         PLASMA     full-screen animated interference field
 *         AURORA     flowing vertical noise curtains
 *         QUASAR     a brilliant nucleus with two vertical jets
 *
 * Study alongside: ../fields/perin_noise_flow_showcase.c — that file
 *       uses a Perlin field to STEER particles. This file uses a hash
 *       to PLACE stars and uses Perlin/fBm only for the nebula. The
 *       distinction is "noise as motion" vs "noise as content".
 *
 * Section map (re-cut by CONCERN — see ARCHITECTURE block below):
 *   §1 CONFIG       — constants, data tables (layers, glyphs, themes), Pattern
 *   §2 PERFORMANCE  — timing primitives (throttle policy lives in main)
 *   §3 LOGIC        — pure: int hash, star_at, perlin2d/fbm, name map
 *   §4 SIMULATION   — scene_tick (camera + clock advance) + reset/reseed
 *   §5 EFFECTS      — cosmetic-only state (one-line note: none stored)
 *   §6 DELAYS       — pauses, holds, timers (one-line note: pause only)
 *   §7 RENDER       — pattern dispatch: layer scan / field modes / overlays + HUD
 *   §8 APP          — user events + per-tick combine + main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset camera to origin, reseed
 *   n / N      next / prev pattern  (cycles all 15 modes)
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
 * References     :
 *
 *   Procedural placement & noise (the concepts) —
 *   • Teschner, M. et al. (2003) — "Optimized Spatial Hashing for Collision
 *     Detection of Deformable Objects", VMV. Source of hash3's large-prime
 *     multipliers and the whole placement trick: a star exists at (wx,wy,L)
 *     iff hash3(wx,wy,L) mod density == 0.
 *   • Perlin, K. (1985) — "An Image Synthesizer", SIGGRAPH. Gradient noise.
 *   • Perlin, K. (2002) — "Improving Noise" (the quintic fade used in fade_q)
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   • Ebert, Musgrave, Peachey, Perlin & Worley — "Texturing & Modeling: A
 *     Procedural Approach". fBm (stacked octaves) → NEBULA / AURORA / MILKYWAY.
 *   • Wikipedia — "Perlin noise"  https://en.wikipedia.org/wiki/Perlin_noise
 *
 *   Rendering & real-time effects —
 *   • Wikipedia — "Parallax scrolling" (depth from per-layer scroll speed)
 *     https://en.wikipedia.org/wiki/Parallax_scrolling
 *   • Lode Vandevenne — "Lode's Computer Graphics Tutorial" (plasma, tunnel,
 *     starfield)  https://lodev.org/cgtutor/  — the PLASMA / TUNNEL / AURORA modes.
 *   • Inigo Quilez — "Hash without Sine" / integer noise
 *     https://iquilezles.org/articles/morenoise/
 *   • Red Blob Games — "Noise functions and map generation"
 *     https://www.redblobgames.com/articles/noise/introduction.html
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

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into separated concern-layers (a SEPARATION
 * pass: RELOCATE + LABEL only — every function body is byte-identical, nothing
 * renamed). Like the galaxy showcase, this is a pure-function renderer: the
 * picture is a stateless function of a camera + a clock + a mode selector, so
 * §3 LOGIC is large and §4 SIMULATION is tiny. Layer → section → what it mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time constants + const data tables
 *                    (LAYER_SPEEDS/DENSITY/GLYPHS, themes) + the Pattern enum.
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers; the
 *                    frame cap + fixed-timestep accumulator are POLICY in main.
 *   LOGIC        §3  nothing — the spatial hash, star_at (a star exists iff
 *                    hash3(wx,wy,L) mod density == 0), the Perlin/fBm samplers,
 *                    pattern_name, and the small pure math (twinkle_brightness,
 *                    swirl_cell). All pure; perm[] is a stable table
 *                    reseeded only from §4, so no RENDER/EFFECTS reorder can
 *                    corrupt a LOGIC result.
 *   SIMULATION   §4  Scene.{cam,paused,speed,current_theme,current_pattern,
 *                    time_secs} + the global perm[] noise table. scene_tick
 *                    advances the camera + clock; scene_reset/init/perm_shuffle
 *                    seed. The ONLY writers of sim state.
 *   EFFECTS      §5  (none) — no stored cosmetic buffer; trails / twinkle /
 *                    rings are derived at render time. One line, not a layer.
 *   DELAYS       §6  (none) — only Scene.paused (early-returns scene_tick); the
 *                    field scrolls continuously, no holds/dwells.
 *   RENDER       §7  ncurses back buffer + colour-pair table only (theme_apply,
 *                    color_init, the draw_ / overlay_ family, scene_draw dispatch,
 *                    screen_draw). Reads §4 state, re-evaluates §3 per cell;
 *                    never writes simulation state.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     while (sim_accum >= tick_ns)        // PERFORMANCE: fixed timestep
 *         scene_tick()                    //   SIMULATION (camera + clock)
 *     screen_draw() ; screen_present()    // RENDER (reads only; field re-derived)
 *     getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick() advances simulation state. User events
 * (app_handle_key / app_do_resize) mutate Scene/Screen on a keypress or
 * SIGWINCH — and 'r' reseeds the noise via perm_shuffle — but they run once per
 * frame OUTSIDE the accumulator loop, not as part of the tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants, data tables (layers, glyphs, themes), Pattern*/
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
 * Cell aspect (terminal cells are ~2× taller than wide) — multiply a vertical
 * cell-distance by this when measuring radius/angle so circles, rings and
 * radial streaks look round instead of squashed.
 */
#define SF_ASPECT           2.0f

/* TUNNEL — radial fly-through: foreground stars streak outward from the centre,
 * longer toward the edges (the vanishing-point illusion). */
#define TUNNEL_STREAK_MAX   7

/* STARFALL — stars rain downward with a short vertical trail. */
#define FALL_STREAK_LEN     4

/* WORMHOLE — spiral swirl: how hard the field twists as it nears the centre. */
#define WORM_SWIRL          5.0f
#define WORM_SPIN           0.30f      /* swirl rotation speed (rad/sec)     */
#define WORM_FALLOFF        0.08f      /* how fast the twist decays with radius */

/* PULSAR — concentric brightness rings marching outward from the centre. */
#define PULSAR_SPEED        24.0f      /* ring expansion, cells/sec          */
#define PULSAR_SPACING      14.0f      /* gap between rings, cells           */
#define PULSAR_RING_W        1.6f      /* ring half-thickness, cells         */

/* SUPERNOVA — a few sites that detonate into an expanding, fading shock ring. */
#define NOVA_SITES           3
#define NOVA_PERIOD          3.6f      /* seconds between re-detonations     */
#define NOVA_SPEED          22.0f      /* shock expansion, cells/sec         */
#define NOVA_RING_W          1.8f

/* METEORS — sparse fast diagonal streaks over a calm field. */
#define METEOR_COUNT         6
#define METEOR_LEN          12
#define METEOR_PERIOD        2.2f      /* seconds per meteor cycle           */

/* MILKYWAY — a luminous diagonal dust band across the field. */
#define BAND_HALF_W          6.0f      /* band half-thickness, aspect cells  */
#define BAND_SLOPE           0.5f      /* band tilt (dy per dx)              */

/* PLASMA — full-screen animated interference field (sum of sines). */
#define PLASMA_SCALE         0.13f
#define PLASMA_SPEED         1.4f

/* AURORA — flowing horizontal noise curtains. */
#define AURORA_SCALE         0.07f
#define AURORA_SPEED         0.6f

/* QUASAR — brilliant nucleus with two vertical relativistic jets. */
#define QUASAR_CORE_R        4.0f      /* bright core radius, aspect cells   */
#define QUASAR_JET_HALF_W    1.5f      /* jet beam half-width, cells         */

/*
 * Pattern — fifteen ways of rendering (or replacing) the parallax field, each a
 * distinct cosmic phenomenon. Cycle with n / p. The first four are the original
 * field modes (kept exactly); the rest add radial/vertical/swirl motion,
 * timed pulse + shock rings, meteors, a Milky-Way band, and full-screen field
 * effects (plasma, aurora, quasar) that stand in for the star scan.
 */
typedef enum {
    PATTERN_STARFIELD = 0,    /* classic parallax scroll                  */
    PATTERN_TWINKLE,          /* stars pulse in/out                       */
    PATTERN_NEBULA,           /* drifting fBm dust clouds + stars         */
    PATTERN_WARP,             /* linear hyperspace streaks                */
    PATTERN_TUNNEL,           /* radial fly-through (streaks from centre) */
    PATTERN_STARFALL,         /* vertical star rain                       */
    PATTERN_WORMHOLE,         /* spiral swirl of the whole field          */
    PATTERN_REDSHIFT,         /* stars tinted by depth (near→far gradient)*/
    PATTERN_PULSAR,           /* expanding concentric brightness rings    */
    PATTERN_SUPERNOVA,        /* detonating, fading shock rings           */
    PATTERN_METEORS,          /* sparse fast diagonal meteor streaks      */
    PATTERN_MILKYWAY,         /* luminous diagonal dust band              */
    PATTERN_PLASMA,           /* full-screen interference plasma          */
    PATTERN_AURORA,           /* flowing horizontal noise curtains        */
    PATTERN_QUASAR,           /* nucleus + vertical relativistic jets     */
    N_PATTERNS,
} Pattern;

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 * Each theme provides 4 STAR tints (one per star colour bucket) and
 * 4 NEBULA tints (one per density band). PAIR_HUD/HINT stay
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
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The 60 fps frame cap and the fixed-timestep
 * accumulator that decide how many scene_tick()s run per frame are POLICY,
 * applied in main() (§8). */

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
/* §3  LOGIC  -- pure decisions: hash, star test, perlin/fbm, name map   */
/* ===================================================================== */

/* Pure functions — the bulk of the algorithm. Given their arguments (and the
 * read-only noise table) each returns a value with NO mutation and NO I/O: the
 * spatial hash, the star-existence test (a star is at (wx,wy,L) iff
 * hash3 mod density == 0), the Perlin/fBm samplers, the enum→name map, and the
 * small per-cell math (twinkle_brightness, the WORMHOLE swirl_cell).
 *
 * The one piece of data here, perm[], is a stable lookup table reseeded ONLY
 * by perm_shuffle (§4, on reset) — never by RENDER/EFFECTS — so a frame's
 * render order cannot change any LOGIC result. star_at / the noise are
 * evaluated fresh per cell by the renderer; they are decisions, not state. */

/* 9-char padded names so the HUD field width stays fixed. */
static const char *pattern_name(Pattern p)
{
    static const char *names[N_PATTERNS] = {
        "STARFIELD", "TWINKLE  ", "NEBULA   ", "WARP     ", "TUNNEL   ",
        "STARFALL ", "WORMHOLE ", "REDSHIFT ", "PULSAR   ", "SUPERNOVA",
        "METEORS  ", "MILKYWAY ", "PLASMA   ", "AURORA   ", "QUASAR   ",
    };
    return ((int)p >= 0 && (int)p < N_PATTERNS) ? names[p] : "?        ";
}

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

/* twinkle_brightness — per-star scintillation in [0,1]: a sine driven by the
 * star's OWN hashed phase, so neighbouring stars pulse out of sync. */
static inline float twinkle_brightness(uint32_t h, float time_secs)
{
    float phase = ((float)((h >> 24) & 0xFFu) / 255.0f) * 2.0f * (float)M_PI;
    return 0.5f + 0.5f * sinf(2.0f * (float)M_PI * TWINKLE_HZ * time_secs + phase);
}

/* swirl_cell — WORMHOLE distortion: rotate a screen cell around the centre by
 * an angle that grows toward the centre (and drifts with time), so the sampled
 * field spirals inward. Writes the swirled sample position to *esx, *esy. */
static inline void swirl_cell(int sx, int sy, float cx, float cy, float time_secs,
                              float *esx, float *esy)
{
    float dx = (float)sx - cx, dy = ((float)sy - cy) * SF_ASPECT;
    float rr = sqrtf(dx * dx + dy * dy);
    float ang = atan2f(dy, dx)
              + WORM_SWIRL / (rr * WORM_FALLOFF + 1.0f)
              + time_secs * WORM_SPIN;
    *esx = cx + rr * cosf(ang);
    *esy = cy + rr * sinf(ang) / SF_ASPECT;
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (only writers of sim state)         */
/* ===================================================================== */

/* The ONLY writers of simulation state. scene_tick advances the camera
 * (cam.x/y by velocity × speed × dt) and the wall clock (time_secs) once per
 * tick; scene_reset / scene_init (with perm_shuffle, the noise reseed) set them
 * on reset. Mutates: Scene.{cam,paused,speed,current_theme,current_pattern,
 * time_secs} and the global perm[] noise table. scene_tick() is the single
 * per-tick entry point (called only from main, §8); user events (key/resize)
 * also mutate Scene but are NOT part of the tick -- see §8. */

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
 * Scene — all the state that survives across ticks, as a table of contents.
 * There is no per-cell array: the renderer reads camera + pattern + clock and
 * computes every pixel from the hash / Perlin functions. Fields group by
 * concept, not by which key changes them. scene_tick() (§4) is the only
 * per-tick writer; user events set the knobs / selections.
 */
typedef struct {
    /* WHAT is simulated — the moving viewpoint flying through the field */
    Camera  cam;              /* position + velocity (world units)        */
    /* HOW the user drives it — the tunable simulation knob */
    int     speed;            /* scroll-speed multiplier, 1..SCROLL_SPEED_MAX */
    /* WHAT we are looking at — RENDER selections, not simulation */
    Pattern current_pattern;  /* active render mode (n/p)                 */
    int     current_theme;    /* index into themes[] (t/T)                */
    /* WHEN we are — wall clock + run-state */
    float   time_secs;        /* accumulating clock (drives twinkle/effects) */
    bool    paused;           /* freeze camera + clock; rendering continues */
} Scene;

/*
 * scene_reset — snap the camera to the origin and reseed the Perlin
 * permutation. This is the "press r" behaviour and is also the one-time
 * init-the-world call.
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
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. Nothing cosmetic is stored: the WARP / TUNNEL / STARFALL
 * motion trails, the TWINKLE pulse, and the pulsar/supernova/meteor overlays
 * are all DERIVED at render time from the star position + time_secs (§7) —
 * there is no trail/glow buffer to advance. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer. The only timing control is the pause toggle
 * (Scene.paused), which early-returns scene_tick (§4). There are no holds or
 * dwells — the camera scrolls (and the effects animate) continuously. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. scene_draw dispatches the active pattern: full-screen field
 * modes (plasma/aurora/quasar) replace the scan, the rest render the parallax
 * star field (draw_star_field → star_at + draw_star_at, with NEBULA/MILKYWAY
 * dust) then layer a timed overlay (pulsar/supernova/meteors); screen_draw lays
 * the HUD over it. Reads Scene/Screen + §3 LOGIC; writes ONLY the ncurses back
 * buffer and the colour-pair table (theme_apply/color_init). Never mutates sim. */

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
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

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

/* draw_warp_trail — WARP: a fading linear streak in +x behind a foreground
 * star (the world slides -x as the camera moves +x). */
static void draw_warp_trail(int sy, int sx, int pair, int cols)
{
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

/* draw_tunnel_trail — TUNNEL: a radial streak pointing OUTWARD from the centre,
 * lengthening toward the edges (the fly-through vanishing-point illusion). */
static void draw_tunnel_trail(int sy, int sx, int pair, float cx, float cy,
                              int rows, int cols)
{
    float dx = (float)sx - cx, dy = (float)sy - cy;
    float len = sqrtf(dx * dx + dy * dy);
    if (len <= 0.5f) return;
    float ux = dx / len, uy = dy / len;
    float maxlen = sqrtf(cx * cx + cy * cy) + 1.0f;
    int slen = 1 + (int)(len / maxlen * TUNNEL_STREAK_MAX);
    for (int d = 1; d <= slen; d++) {
        int tx = sx + (int)lroundf(ux * (float)d);
        int ty = sy + (int)lroundf(uy * (float)d);
        if (tx < 0 || tx >= cols || ty < 2 || ty >= rows - 1) break;
        char tg = (d <= 1) ? '-' : (d <= 3) ? ':' : '.';
        int  ta = (d <= 1) ? A_BOLD : (d <= 3) ? A_NORMAL : A_DIM;
        attron(COLOR_PAIR(pair) | ta);
        mvaddch(ty, tx, (chtype)(unsigned char)tg);
        attroff(COLOR_PAIR(pair) | ta);
    }
}

/* draw_fall_trail — STARFALL: a short downward trail (stars raining). */
static void draw_fall_trail(int sy, int sx, int pair, int rows)
{
    for (int d = 1; d <= FALL_STREAK_LEN; d++) {
        int ty = sy + d;
        if (ty >= rows - 1) break;
        char tg = (d == 1) ? '|' : (d == 2) ? ':' : '.';
        int  ta = (d == 1) ? A_NORMAL : A_DIM;
        attron(COLOR_PAIR(pair) | ta);
        mvaddch(ty, sx, (chtype)(unsigned char)tg);
        attroff(COLOR_PAIR(pair) | ta);
    }
}

/*
 * draw_star_at — paint ONE detected star: decode its tint/glyph from the hash,
 * apply the per-mode modifier (REDSHIFT depth-tint, WARP glyph, TWINKLE
 * brightness), draw it, then add the mode's motion trail. Returns false only
 * when TWINKLE has this star off-cycle, so the caller scans the layers behind.
 *
 *   sy, sx : screen cell    layer : 0..N_LAYERS-1    h : the star's hash
 */
static bool draw_star_at(int sy, int sx, int layer, uint32_t h,
                         Pattern pattern, float time_secs,
                         int rows, int cols, float cx, float cy)
{
    int color_idx = (int)((h >> 16) & 3u);
    int glyph_idx = (int)((h >>  8) & 3u);

    /* Depth cue: brightness by layer; REDSHIFT instead tints by depth. */
    int attr = A_NORMAL;
    if      (layer == 0) attr = A_BOLD;
    else if (layer == 3) attr = A_DIM;
    if (pattern == PATTERN_REDSHIFT) color_idx = layer;

    /* Glyph: WARP foreground stars become streak heads. */
    char glyph = (pattern == PATTERN_WARP && layer == 0)
               ? WARP_GLYPHS[glyph_idx] : LAYER_GLYPHS[layer][glyph_idx];

    /* TWINKLE: off-cycle stars return false so layers behind show through. */
    if (pattern == PATTERN_TWINKLE) {
        float b = twinkle_brightness(h, time_secs);
        if (b < TWINKLE_OFF_THRESH)       return false;
        else if (b < TWINKLE_BOLD_THRESH) attr = A_DIM;
        else                              attr = A_BOLD;
    }

    int pair = PAIR_STAR_BASE + color_idx;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);

    /* Foreground motion trail — direction depends on the mode. */
    if      (pattern == PATTERN_WARP     && layer == 0) draw_warp_trail(sy, sx, pair, cols);
    else if (pattern == PATTERN_TUNNEL   && layer == 0) draw_tunnel_trail(sy, sx, pair, cx, cy, rows, cols);
    else if (pattern == PATTERN_STARFALL && layer <= 1) draw_fall_trail(sy, sx, pair, rows);

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

/* draw_band_cell — MILKYWAY dust confined to a tilted diagonal band: fBm dust
 * that fades out away from the band centre-line. */
static void draw_band_cell(int sy, int sx, float wx, float wy, float cx, float cy)
{
    float dband = ((float)sy - cy) * SF_ASPECT - BAND_SLOPE * ((float)sx - cx);
    if (fabsf(dband) > BAND_HALF_W) return;
    float falloff = 1.0f - fabsf(dband) / BAND_HALF_W;           /* 1 at centre */
    float n = fbm2(wx * NEBULA_SCALE * 1.6f, wy * NEBULA_SCALE * 1.6f) * falloff;
    if (n < 0.28f) return;

    char glyph; int color_idx, attr = A_NORMAL;
    if      (n < 0.42f) { glyph = '.'; color_idx = 0; attr = A_DIM;  }
    else if (n < 0.55f) { glyph = '*'; color_idx = 1;               }
    else                { glyph = '#'; color_idx = 3; attr = A_BOLD; }
    int pair = PAIR_NEBULA_BASE + color_idx;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * draw_star_field — the parallax scan shared by every star-based mode. For
 * each cell, scan layers front-to-back; the first hit wins (foreground stars
 * occlude deeper ones, and most cells terminate on layer 0/1). WORMHOLE twists
 * the sample position around the centre (swirl_cell); NEBULA / MILKYWAY fill
 * empty cells with dust. For the plain modes it is just the parallax scan.
 */
static void draw_star_field(const Camera *cam, Pattern pattern, float time_secs,
                            int top, int bottom, int width, int rows, int cols)
{
    Pattern p = pattern;
    float cx = (float)width * 0.5f;
    float cy = (float)(top + bottom) * 0.5f;

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {

            /* The cell we sample the field at — WORMHOLE swirls it inward. */
            float esx = (float)sx, esy = (float)sy;
            if (p == PATTERN_WORMHOLE)
                swirl_cell(sx, sy, cx, cy, time_secs, &esx, &esy);

            bool drew = false;
            for (int L = 0; L < N_LAYERS; L++) {
                /* Per-layer world coord — FLOOR (not cast) to avoid a jump at 0. */
                float wxf = esx + cam->x * LAYER_SPEEDS[L];
                float wyf = esy + cam->y * LAYER_SPEEDS[L];
                int   wx  = (int)floorf(wxf);
                int   wy  = (int)floorf(wyf);

                uint32_t h;
                if (!star_at(wx, wy, L, &h)) continue;

                if (draw_star_at(sy, sx, L, h, p, time_secs,
                                 rows, cols, cx, cy)) {
                    drew = true;
                    break;          /* foreground wins */
                }
                /* TWINKLE off-cycle: continue scanning behind. */
            }

            if (!drew && p == PATTERN_NEBULA) {
                float wxf = (float)sx + cam->x * NEBULA_SCROLL;
                float wyf = (float)sy + cam->y * NEBULA_SCROLL;
                draw_nebula_cell(sy, sx, wxf, wyf);
            } else if (!drew && p == PATTERN_MILKYWAY) {
                float wxf = (float)sx + cam->x * NEBULA_SCROLL;
                float wyf = (float)sy + cam->y * NEBULA_SCROLL;
                draw_band_cell(sy, sx, wxf, wyf, cx, cy);
            }
        }
    }
}

/* overlay_pulsar — concentric brightness rings marching outward from centre. */
static void overlay_pulsar(float time_secs, int top, int bottom, int width)
{
    float cx = (float)width * 0.5f, cy = (float)(top + bottom) * 0.5f;
    float march = fmodf(time_secs * PULSAR_SPEED, PULSAR_SPACING);
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float dx = (float)sx - cx, dy = ((float)sy - cy) * SF_ASPECT;
            float rr = sqrtf(dx * dx + dy * dy);
            float ph = fmodf(rr - march, PULSAR_SPACING);
            if (ph < 0.0f) ph += PULSAR_SPACING;
            if (ph < PULSAR_RING_W || ph > PULSAR_SPACING - PULSAR_RING_W) {
                int  pair  = PAIR_STAR_BASE + ((int)(rr / PULSAR_SPACING) & 3);
                char glyph = (ph < PULSAR_RING_W * 0.5f
                              || ph > PULSAR_SPACING - PULSAR_RING_W * 0.5f) ? 'O' : 'o';
                attron(COLOR_PAIR(pair) | A_BOLD);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(pair) | A_BOLD);
            }
        }
    }
}

/* overlay_supernova — a few hashed sites, each detonating into an expanding,
 * fading shock ring on its own clock. */
static void overlay_supernova(float time_secs, int top, int bottom, int width)
{
    int span_w = width > 1 ? width : 1;
    int span_h = (bottom - top) > 1 ? (bottom - top) : 1;
    for (int k = 0; k < NOVA_SITES; k++) {
        uint32_t hs = hash3(k * 97 + 13, k * 61 + 7, 1234);
        float sxc = (float)(hs % (uint32_t)span_w);
        float syc = (float)(top + (int)((hs >> 8) % (uint32_t)span_h));
        float age = fmodf(time_secs + (float)(hs & 0xFFu) / 255.0f * NOVA_PERIOD,
                          NOVA_PERIOD);
        float ring_r = age * NOVA_SPEED;
        float fade   = 1.0f - age / NOVA_PERIOD;       /* 1 (fresh) → 0 (faded) */
        if (fade < 0.10f) continue;

        for (int sy = top; sy < bottom; sy++) {
            for (int sx = 0; sx < width; sx++) {
                float dx = (float)sx - sxc, dy = ((float)sy - syc) * SF_ASPECT;
                float rr = sqrtf(dx * dx + dy * dy);
                if (fabsf(rr - ring_r) >= NOVA_RING_W) continue;
                int  attr  = fade > 0.6f ? A_BOLD : fade > 0.3f ? A_NORMAL : A_DIM;
                char glyph = fade > 0.6f ? '#'    : fade > 0.3f ? '*'      : '.';
                int  pair  = PAIR_NEBULA_BASE + (k & 3);
                attron(COLOR_PAIR(pair) | attr);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(pair) | attr);
            }
        }
    }
}

/* overlay_meteors — sparse fast diagonal streaks, each on its own phase clock. */
static void overlay_meteors(float time_secs, int top, int bottom, int width)
{
    int span_h = (bottom - top) > 1 ? (bottom - top) : 1;
    for (int m = 0; m < METEOR_COUNT; m++) {
        uint32_t hm = hash3(m * 131 + 5, m * 89 + 3, 77);
        float phase   = (float)(hm & 0xFFFFu) / 65535.0f;
        float t       = fmodf(time_secs / METEOR_PERIOD + phase, 1.0f);  /* 0..1 */
        float head_x  = t * (float)(width + METEOR_LEN) - (float)METEOR_LEN;
        float start_y = (float)(top + (int)((hm >> 16) % (uint32_t)span_h));
        float head_y  = start_y + t * 6.0f;                                  /* slight fall */
        for (int d = 0; d < METEOR_LEN; d++) {
            int tx = (int)(head_x - (float)d);
            int ty = (int)(head_y - (float)d * 0.4f);
            if (tx < 0 || tx >= width || ty < top || ty >= bottom) continue;
            char glyph = d == 0 ? '@' : d < 3 ? '*' : d < 6 ? '-' : '.';
            int  attr  = d == 0 ? A_BOLD : d < 6 ? A_NORMAL : A_DIM;
            int  pair  = PAIR_STAR_BASE + (m & 3);
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(ty, tx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* draw_plasma — full-screen animated interference field (sum of sines). Fills
 * every cell, so there is no star scan; colour + glyph track the field value. */
static void draw_plasma(float time_secs, int top, int bottom, int width)
{
    float t = time_secs * PLASMA_SPEED;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float fx = (float)sx * PLASMA_SCALE, fy = (float)sy * PLASMA_SCALE;
            float v = sinf(fx + t)
                    + sinf(fy * 1.3f + t * 0.9f)
                    + sinf((fx + fy) * 0.7f + t * 1.1f)
                    + sinf(sqrtf(fx * fx + fy * fy) * 0.9f + t);
            float u = (v + 4.0f) / 8.0f;                     /* → [0,1] */
            int  idx   = (int)(u * 3.999f);
            char glyph = u < 0.25f ? '.' : u < 0.50f ? '*' : u < 0.75f ? '#' : '@';
            int  attr  = u < 0.30f ? A_DIM : u > 0.70f ? A_BOLD : A_NORMAL;
            int  pair  = PAIR_NEBULA_BASE + (idx & 3);
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* draw_aurora — flowing vertical curtains: fBm stretched tall and drifting
 * upward, modulated by a horizontal shimmer. Dark sky shows between curtains. */
static void draw_aurora(float time_secs, int top, int bottom, int width)
{
    float t = time_secs * AURORA_SPEED;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float n = fbm2((float)sx * AURORA_SCALE,
                           (float)sy * AURORA_SCALE * 3.0f - t);
            float shimmer = 0.5f + 0.5f * sinf((float)sx * 0.08f + t * 2.0f + n * 4.0f);
            float v = n * shimmer;
            if (v < 0.18f) continue;
            int  idx   = v < 0.30f ? 0 : v < 0.45f ? 1 : v < 0.60f ? 2 : 3;
            char glyph = v < 0.30f ? '.' : v < 0.45f ? ':' : v < 0.60f ? '|' : '#';
            int  attr  = v < 0.30f ? A_DIM : v > 0.60f ? A_BOLD : A_NORMAL;
            int  pair  = PAIR_NEBULA_BASE + idx;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* draw_quasar — a brilliant nucleus with two vertical pulsing jets, a faint
 * accretion sprinkle filling the rest. */
static void draw_quasar(float time_secs, int top, int bottom, int width)
{
    float cx = (float)width * 0.5f, cy = (float)(top + bottom) * 0.5f;
    float span = (float)(bottom - top);
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < width; sx++) {
            float dx = (float)sx - cx, dy = ((float)sy - cy) * SF_ASPECT;
            float rr = sqrtf(dx * dx + dy * dy);

            if (rr < QUASAR_CORE_R) {                         /* nucleus */
                int  attr  = rr < QUASAR_CORE_R * 0.5f ? A_BOLD : A_NORMAL;
                char glyph = rr < QUASAR_CORE_R * 0.5f ? '@' : '#';
                attron(COLOR_PAIR(PAIR_STAR_BASE) | attr);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(PAIR_STAR_BASE) | attr);
            } else if (fabsf((float)sx - cx) < QUASAR_JET_HALF_W) {   /* jets */
                float jf    = 1.0f - rr / span;
                float pulse = 0.5f + 0.5f * sinf(rr * 0.3f - time_secs * 4.0f);
                float b     = jf * pulse;
                if (b > 0.08f) {
                    int  attr  = b > 0.4f ? A_BOLD : b > 0.2f ? A_NORMAL : A_DIM;
                    char glyph = b > 0.4f ? '|' : ':';
                    attron(COLOR_PAIR(PAIR_NEBULA_BASE + 3) | attr);
                    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                    attroff(COLOR_PAIR(PAIR_NEBULA_BASE + 3) | attr);
                }
            } else {                                          /* faint sprinkle */
                uint32_t h = hash3(sx, sy, 909);
                if ((h % 40u) == 0u) {
                    int pair = PAIR_STAR_BASE + (int)((h >> 8) & 3u);
                    attron(COLOR_PAIR(pair) | A_DIM);
                    mvaddch(sy, sx, '.');
                    attroff(COLOR_PAIR(pair) | A_DIM);
                }
            }
        }
    }
}

/*
 * scene_draw — dispatch the active pattern. Full-screen field modes (plasma /
 * aurora / quasar) replace the star scan entirely; everything else renders the
 * parallax star field, then layers a timed overlay (pulsar / supernova /
 * meteors) on top.
 */
static void scene_draw(const Camera *cam, Pattern pattern, float time_secs,
                       int cols, int rows)
{
    /* Reserve top 2 rows for HUD, bottom 1 for hint. */
    int top    = 2;
    int bottom = rows - 1;
    int width  = cols;
    if (width  > MAP_W_MAX) width  = MAP_W_MAX;
    if (bottom - top > MAP_H_MAX) bottom = top + MAP_H_MAX;

    switch (pattern) {
    case PATTERN_PLASMA: draw_plasma(time_secs, top, bottom, width); return;
    case PATTERN_AURORA: draw_aurora(time_secs, top, bottom, width); return;
    case PATTERN_QUASAR: draw_quasar(time_secs, top, bottom, width); return;
    default: break;
    }

    draw_star_field(cam, pattern, time_secs, top, bottom, width, rows, cols);

    switch (pattern) {
    case PATTERN_PULSAR:    overlay_pulsar   (time_secs, top, bottom, width); break;
    case PATTERN_SUPERNOVA: overlay_supernova(time_secs, top, bottom, width); break;
    case PATTERN_METEORS:   overlay_meteors  (time_secs, top, bottom, width); break;
    default: break;
    }
}

/* Draw a 4-tint palette swatch at row 1, col x; returns the next free column. */
static int draw_swatch(int x, int base_pair, char glyph, int attr)
{
    for (int i = 0; i < 4; i++) {
        attron(COLOR_PAIR(base_pair + i) | attr);
        mvaddch(1, x, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(base_pair + i) | attr);
        x++;
    }
    return x;
}

/* Row 0 right — primary status: fps, sim Hz, mode/pause, speed. Right-aligned. */
static void draw_status_line(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED   " : pattern_name(s->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern (n/N counter), theme, the star + nebula tint swatches, and
 * the camera position. Fixed left-aligned layout. */
static void draw_param_line(const Scene *s)
{
    int x = 1;
    char pbuf[40];
    snprintf(pbuf, sizeof pbuf, " pattern:%s %d/%d ",
             pattern_name(s->current_pattern),
             (int)s->current_pattern + 1, (int)N_PATTERNS);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, "%s", pbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += (int)strlen(pbuf);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " stars:");  attroff(COLOR_PAIR(PAIR_HUD));
    x = draw_swatch(x + 7, PAIR_STAR_BASE, '*', A_BOLD);
    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " neb:");    attroff(COLOR_PAIR(PAIR_HUD));
    x = draw_swatch(x + 5, PAIR_NEBULA_BASE, '#', A_NORMAL);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  cam:(%7.1f,%6.1f)  layers:%d ",
             (double)s->cam.x, (double)s->cam.y, N_LAYERS);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_draw — clear, draw the scene, then lay the HUD over it (status, title,
 * params, hint).
 *
 * The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, camera, run-state. A
 * const read can't re-couple the layers; scene_draw and the renderers stay narrow.
 */
static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(&s->cam, s->current_pattern, s->time_secs, sc->cols, sc->rows);

    draw_status_line(sc, s, fps, sim_fps);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " STAR FIELD / PARALLAX ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    draw_param_line(s);
    draw_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main
 * loop. main() is the ONE place that combines the layers per tick, in fixed
 * order:  scene_tick (SIM) -> screen_draw (RENDER) -> screen_present -> input.
 * app_handle_key() / app_do_resize() mutate state on USER EVENTS (a keypress or
 * SIGWINCH; 'r' also reseeds the noise) and are deliberately OUTSIDE the tick. */

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
