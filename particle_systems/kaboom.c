/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * kaboom.c  —  ncurses ASCII blast / kaboom effect
 *
 * Original algorithm by the kaboom.c author, rewritten in the
 * fireworks/matrix_rain framework:
 *   - Single stdscr, ncurses internal double buffer — no flicker
 *   - typeahead(-1) — no mid-flush input polling, no tearing
 *   - HUD written into stdscr after blast (always on top)
 *   - dt (delta-time) loop drives playback speed independently of CPU, render capped at 60 fps
 *   - SIGWINCH resize: rebuilds scene + restarts blast
 *   - Speed control:   ] = faster   [ = slower
 *   - Restart:         r = replay current theme+shape from frame 0
 *   - Theme cycle:     t / T = next / previous theme (resets blast)
 *   - Shape cycle:     n / N = next / previous shape (resets blast)
 *   - Clean signal / atexit teardown — terminal always restored
 *
 * Keys:
 *   q / ESC   quit
 *   ]  [      speed up / slow down
 *   r         replay (same theme + shape)
 *   t / T     next / previous theme  — resets blast immediately
 *   n / N     next / previous shape  — resets blast immediately
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra kaboom.c -o kaboom -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config  — every tunable constant in one block
 *   §2  clock   — monotonic nanosecond clock, portable sleep
 *   §3  color   — color pairs; 256-color with 8-color fallback
 *   §4  blob    — one 3-D debris particle (original algorithm)
 *   §5  blast   — frame buffer + blob pool + tick + draw
 *   §6  screen  — single stdscr, ncurses internal double buffer
 *   §7  app     — dt loop, input, resize, cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two-layer 3-D explosion drawn into a single 2-D
 *                  Cell back-buffer. Layer A (wave_layer_render) is a
 *                  scalar field over the screen grid: an aspect-corrected
 *                  radius modulated by a cos(petal_n·θ) lobe term, sampled
 *                  per frame to pick a glyph from flash_chars/wave_chars.
 *                  Layer B (blob_layer_render) is NUM_BLOBS=800 unit
 *                  directions on a sphere, projected through a pinhole
 *                  camera. Blobs carry no velocity — position(t) is
 *                  synthesised as direction · (frame-6) · blob_speed,
 *                  so the blob array is read-only after init.
 *
 * Math           : Pinhole perspective: cx = cols/2 + bx·P/(bz+P)
 *                  (perspective_project). Aspect-corrected radius
 *                  √(x² + 4y²) so circles look circular on 2:1 cells
 *                  (aspect_radius). Petal modulation 1+ripple·cos(n·θ)
 *                  with a smooth-ring guard at petal_n ≤ 0 (petal_lobe).
 *                  No depth sorting — last-writer-wins in the cell
 *                  buffer (blob_layer runs after wave_layer) is enough
 *                  for an explosion since fragments don't occlude.
 *
 * Performance    : Fixed NUM_BLOBS=800-entry pool reused every frame
 *                  (no per-frame allocation). The only malloc/calloc in
 *                  the hot path is Blast.cells, sized cols×rows and
 *                  released on resize/reset. dt loop: sim runs at
 *                  SIM_FPS (5–60 Hz), render capped at 60 fps.
 *
 * Rendering      : Drop glyph picked by depth bucket (blob_depth_bucket):
 *                  near '@' COL_BLOB_N, mid 'o' COL_BLOB_M, far '.' COL_BLOB_F.
 *                  Wave glyph picked from the per-shape ramp by
 *                  (frame − r − 7). All writes go into Cell[] first;
 *                  blast_draw walks the array and emits one ncurses
 *                  call per painted cell — empty cells (ch==0) are
 *                  skipped so the black background needs no fill.
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  The fixed-size particle pool, the
 *       stochastic spawn distribution, and the per-particle screen
 *       projection are all from this paper.  We collapse Reeves'
 *       per-tick Euler integration into a closed-form position(t) =
 *       direction · (frame-6) · blob_speed because the only force is
 *       constant outward velocity — no need to carry vy/dt state.
 *       §4 of the paper covers fire/explosion specifically.
 *
 *     Sims, K. (1990)
 *       "Particle Animation and Rendering Using Data Parallel Computation"
 *       SIGGRAPH '90 Proceedings: 405-413.
 *       Extends Reeves with parallel update + rendering; the
 *       per-particle independent update we use is the (trivial-case)
 *       data-parallel pattern Sims formalises.
 *
 *   BOOKS
 *     Knuth, D. E. — "The Art of Computer Programming, Vol. 2:
 *       Seminumerical Algorithms" (3rd ed, Addison-Wesley, 1997).
 *       §3.2.1 — analysis of linear congruential generators; the
 *       multiplier 1488248101 + increment 981577151 used by prng()
 *       are an LCG of exactly the form Knuth studies.
 *
 *     Foley, J. D., van Dam, A., Feiner, S. K. & Hughes, J. F. —
 *       "Computer Graphics: Principles and Practice" (3rd ed,
 *       Addison-Wesley, 2013).  §6.5 — pinhole-camera / perspective
 *       projection; the cx = cols/2 + bx · P/(bz + P) formula is the
 *       discrete-pixel form of the textbook division-by-z.
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. —
 *       "Real-Time Rendering" (4th ed, CRC Press, 2018).
 *       §4.7 covers projection matrices end-to-end; §13.7 covers
 *       point-sprite particle rendering, the depth-bucket glyph
 *       selection (`.` / `o` / `@`) here is the ASCII analogue of
 *       size-by-depth point sprites.
 *
 *     Press, W. H., Teukolsky, S. A., Vetterling, W. T. & Flannery,
 *       B. P. — "Numerical Recipes" (3rd ed, Cambridge UP, 2007).
 *       §7.1 — quality criteria for random number generators and the
 *       LCG family in particular; useful for understanding why this
 *       simple prng() is adequate for visual noise but unsuitable
 *       for Monte-Carlo work.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A blast is two layers stacked.  Layer A (wave_layer_render) is a
 * 2-D shockwave: at each cell (x,y) compute r = √(x² + 4y²)
 * (aspect_radius), apply a petal modulation 1 + ripple·cos(petal_n·θ)
 * (petal_lobe), and shade the cell from wave_chars / flash_chars
 * indexed by (frame − r − 7).  Layer B (blob_layer_render) is
 * NUM_BLOBS=800 3-D point blobs pre-distributed on a unit sphere,
 * flying outward at direction · (frame-6) · blob_speed, projected to
 * the screen by pinhole perspective cx = bx·P/(bz+P)
 * (perspective_project).  Both layers paint into a single Cell[]
 * back-buffer; blast_draw blits the non-empty entries.  A cycle lasts
 * NUM_FRAMES=150 ticks.  Auto-end of cycle advances theme + shape;
 * key 'r' replays the same theme+shape; t / n advance one of them
 * independently and reset the blast immediately.
 *
 * ALGORITHM IN STEPS  (each step = one helper in §4 / §5)
 * ──────────────────
 *  Initial setup (per blast_init / app_reset_blast):
 *    1. blob_init_pool → blob_sample_unit_direction (×NUM_BLOBS).
 *       For each blob pick (bx,by,bz) ∈ [-1,1]³, divide by norm,
 *       multiply y by 0.5 (oblate squish), scale by 1.3 + 0.2·rand.
 *    2. Read pp = k_themes[theme_idx], sh = k_shapes[shape_idx].
 *
 *  Per frame (blast_render_frame):
 *    3. wave_layer_render: sweep (x,y) over [-cols/2, +cols/2] ×
 *       [-rows/2, +rows/2], cell_clear each slot, then branch on frame:
 *    4.   frame == 0   → cell_paint_origin_flash: single '*' FLASH
 *                        at (0,0).
 *    5.   frame  < 8   → cell_paint_disc: aspect_radius < frame·disc_speed
 *                        → '@' FLASH (the initial fireball).
 *    6.   frame ≥ 8   → cell_paint_shockwave:
 *           lobe = petal_lobe(x, y, petal_n, ripple)
 *           r    = aspect_radius(x,y) · (0.5 + prng/3 · lobe · 0.3)
 *           v    = frame − r − 7
 *           v < 0      → flash_chars[frame-8] (INNER colour)
 *           v < waveN  → wave_chars[v]; INNER half / WAVE half.
 *    7. blob_layer_render (only when frame > 6): for each blob,
 *       blob_paint_projected:
 *           bx,by,bz = blob.{x,y·y_squash,z} · (frame-6) · blob_speed
 *           skip if bz outside [5-persp, persp]
 *           cx,cy = cols/2,rows/2 + perspective_project(b{x,y}, bz, P)
 *           skip if outside the screen rectangle
 *           blob_depth_bucket: bz > 0.8·P → '.' COL_BLOB_F,
 *                              bz > -0.4·P → 'o' COL_BLOB_M,
 *                              else       → '@' COL_BLOB_N.
 *
 *  Per frame (after blast_tick advances `frame`):
 *    8. blast_draw → cell_blit per painted cell. Empty cells (ch==0)
 *       are skipped so the black background needs no fill pass.
 *    9. At frame == NUM_FRAMES, blast_tick returns false; the main
 *       loop bumps theme_idx + shape_idx (each mod its COUNT) and
 *       calls app_reset_blast which goes back to step 1.
 *
 * KEY FORMULAS
 * ────────────
 *  Aspect-corrected radius    (aspect_radius):
 *      r  = sqrt(x² + 4·y²)            terminal cells 2× tall as wide
 *
 *  Disc growth (frames 1..7)  (cell_paint_disc):
 *      r  < frame · disc_speed         filled '@' fireball
 *
 *  Angular petal modulation   (petal_lobe + cell_paint_shockwave):
 *      angle = atan2(2y + ε, x + ε)
 *      lobe  = 1 + ripple · cos(petal_n · angle)
 *      r     = base_r · (0.5 + prng/3 · lobe · 0.3)
 *      v     = frame − r − 7           ramp index into wave_chars
 *
 *  Pinhole projection         (perspective_project):
 *      cx = cols/2 + bx · P / (bz + P)
 *      cy = rows/2 + by · P / (bz + P)
 *      with P = sh.persp ∈ {25..80}
 *
 *  Blob outward sweep         (blob_paint_projected, no per-tick state):
 *      bx = blob.x · (frame−6) · blob_speed
 *      by = blob.y · (frame−6) · blob_speed · y_squash
 *      bz = blob.z · (frame−6) · blob_speed
 *
 *  Depth → glyph              (blob_depth_bucket):
 *      bz > 0.8·P  → '.' COL_BLOB_F   (far, small)
 *      bz > -0.4·P → 'o' COL_BLOB_M   (middle)
 *      else        → '@' COL_BLOB_N   (near, big)
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#ifndef M_1_PI
#  define M_1_PI (1.0 / M_PI)
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    NUM_FRAMES      = 150,
    NUM_BLOBS       = 800,

    HUD_COLS        =  28,
    FPS_UPDATE_MS   = 500,
};

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(fps)  (NS_PER_SEC / (fps))

#define PERSPECTIVE   50.0

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

typedef enum {
    COL_FLASH  = 1,
    COL_INNER  = 2,
    COL_WAVE   = 3,
    COL_BLOB_F = 4,
    COL_BLOB_M = 5,
    COL_BLOB_N = 6,
    COL_HUD    = 7,    /* top-row status bar — bright yellow on black  */
    COL_HINT   = 8,    /* bottom-row key hints — bright cyan on black  */
} ColorID;

/*
 * BlastTheme — one colour palette for an entire blast cycle.
 *
 * Each theme defines six foreground colours (FLASH, INNER, WAVE,
 * BLOB_F, BLOB_M, BLOB_N) as 256-colour xterm indices, plus an
 * 8-colour fallback set so terminals without 256-colour support
 * still get a sensible look (chosen automatically via the COLORS >=
 * 256 branch in color_theme_apply). Background is always COLOR_BLACK
 * so blast glyphs read cleanly against the dark frame.
 *
 * Roles, from hottest to coolest:
 *   FLASH   = the initial '@'/'*' fireball — usually pure white so
 *             A_BOLD makes it punch.
 *   INNER   = the pre-wave bright region rendered with flash_chars.
 *   WAVE    = the trailing shockwave rendered with wave_chars.
 *   BLOB_F  = far 3-D debris (depth glyph '.') — usually washed out.
 *   BLOB_M  = mid-depth 3-D debris (depth glyph 'o').
 *   BLOB_N  = near 3-D debris (depth glyph '@') — usually saturated.
 *
 * Members:
 *   name      : short label shown in the HUD. ≤ 8 chars to fit the
 *               %-7s column without truncation.
 *   flash     : 256-colour index for COL_FLASH (the initial fireball
 *               and any cell painted in the pre-wave disc phase).
 *               White (231) for almost every theme — the bright punch
 *               at frame 0 reads best as pure white regardless of
 *               theme so the eye is drawn to the centre.
 *   inner     : 256-colour index for COL_INNER (the bright body of
 *               the wave). The "theme colour" most viewers will
 *               identify the blast by — green for MATRIX, orange for
 *               FIRE, etc.
 *   wave      : 256-colour index for COL_WAVE (the trailing edge of
 *               the shockwave). Usually a DARKER, COOLER variant of
 *               `inner` so the wave reads as fading outward.
 *   blob_f    : COL_BLOB_F — far-depth debris glyph. Often white or
 *               pale so distant chunks look bright + tiny against
 *               the dark.
 *   blob_m    : COL_BLOB_M — middle-depth debris.
 *   blob_n    : COL_BLOB_N — near-depth debris. Usually the
 *               saturated/dark end of the palette so close debris
 *               reads as heavy and 'in the foreground'.
 *   f8_flash  : 8-colour fallback for FLASH (typically COLOR_WHITE).
 *   f8_inner  : 8-colour fallback for INNER.
 *   f8_wave   : 8-colour fallback for WAVE.
 *   f8_bm     : 8-colour fallback for BLOB_M.
 *   f8_bn     : 8-colour fallback for BLOB_N. (BLOB_F reuses
 *               f8_flash in color_theme_apply since the 8-colour
 *               palette has no "pale" variant.)
 *
 * Themes (cycle order on `t`; `T` reverses):
 *   0 MATRIX   — digital green rain: white flash, lime inner, dark green wave
 *   1 FIRE     — classic orange/red: white flash, orange inner, amber wave
 *   2 OCEANIC  — deep sea: white flash, pale aqua inner, teal wave
 *   3 NEON     — retro arcade: white flash, hot pink inner, purple wave
 *   4 MONO     — grayscale: white flash through gray ramp
 *   5 ICE      — frozen: white flash, bright cyan inner, blue wave
 *   6 NOVA     — supernova: white flash, yellow inner, orange-red wave
 *   7 FOREST   — woodland: cream flash, lime inner, dark olive wave
 *   8 DESERT   — sand storm: cream flash, sandy peach inner, brown wave
 *   9 ECLIPSE  — bloodmoon: white flash, orange inner, dark red wave
 */
typedef struct {
    const char *name;
    int flash, inner, wave, blob_f, blob_m, blob_n;    /* 256-color    */
    int f8_flash, f8_inner, f8_wave, f8_bm, f8_bn;     /* 8-color      */
} BlastTheme;

static const BlastTheme k_themes[] = {
    /* name       flash inner wave  blob_f blob_m blob_n   8-color: flash       inner          wave           bm             bn */
    { "MATRIX",   231,  118,   40,  250,   154,    46,
      COLOR_WHITE, COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   },
    { "FIRE",     231,  214,   94,  250,   220,   196,
      COLOR_WHITE, COLOR_YELLOW,  COLOR_RED,     COLOR_YELLOW,  COLOR_RED     },
    { "OCEANIC",  231,  159,   31,  195,    87,    39,
      COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE,    COLOR_CYAN,    COLOR_BLUE    },
    { "NEON",     231,  201,   93,  219,   207,   165,
      COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA },
    { "MONO",     231,  253,  245,  251,   247,   244,
      COLOR_WHITE, COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE   },
    { "ICE",      231,   51,   27,  195,   123,    39,
      COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE,    COLOR_CYAN,    COLOR_BLUE    },
    { "NOVA",     231,  226,  202,  255,   220,   208,
      COLOR_WHITE, COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  },
    { "FOREST",   230,  154,   64,  230,   184,    70,
      COLOR_WHITE, COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_GREEN   },
    { "DESERT",   230,  223,  130,  230,   215,   172,
      COLOR_WHITE, COLOR_YELLOW,  COLOR_RED,     COLOR_YELLOW,  COLOR_RED     },
    { "ECLIPSE",  231,  208,   52,  250,   202,    88,
      COLOR_WHITE, COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/*
 * color_theme_apply() — reload blast color pairs for one theme.
 * Safe to call mid-run; takes effect on the next rendered frame.
 * COL_HUD stays yellow in every theme for consistent readability.
 */
static void color_theme_apply(int t)
{
    const BlastTheme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(COL_FLASH,  th->flash,  COLOR_BLACK);
        init_pair(COL_INNER,  th->inner,  COLOR_BLACK);
        init_pair(COL_WAVE,   th->wave,   COLOR_BLACK);
        init_pair(COL_BLOB_F, th->blob_f, COLOR_BLACK);
        init_pair(COL_BLOB_M, th->blob_m, COLOR_BLACK);
        init_pair(COL_BLOB_N, th->blob_n, COLOR_BLACK);
        init_pair(COL_HUD,    226,        COLOR_BLACK);
        init_pair(COL_HINT,    51,        COLOR_BLACK);
    } else {
        init_pair(COL_FLASH,  th->f8_flash, COLOR_BLACK);
        init_pair(COL_INNER,  th->f8_inner, COLOR_BLACK);
        init_pair(COL_WAVE,   th->f8_wave,  COLOR_BLACK);
        init_pair(COL_BLOB_F, th->f8_flash, COLOR_BLACK);
        init_pair(COL_BLOB_M, th->f8_bm,   COLOR_BLACK);
        init_pair(COL_BLOB_N, th->f8_bn,   COLOR_BLACK);
        init_pair(COL_HUD,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT,   COLOR_CYAN,   COLOR_BLACK);
    }
}

static void color_init(int theme)
{
    start_color();
    color_theme_apply(theme);
}

/* ===================================================================== */
/* §4  blob                                                               */
/* ===================================================================== */

/*
 * Blob — one piece of 3-D debris in the explosion's blob cloud.
 *
 * (x, y, z) is a UNIT DIRECTION from the blast centre, pre-sampled
 * once per cycle in blob_init_pool (rejection-free sphere sampling:
 * pick from [-1,1]³, divide by norm, scale by 1.3 + 0.2·rand). The
 * Blob itself is never mutated after init — outward motion is
 * synthesised every frame as position(t) = blob * (frame-6) *
 * blob_speed, so the same pool feeds every tick at zero cost.
 *
 * Three doubles even though each blob ends up as a single cell,
 * because the visible (cx, cy) comes from perspective division
 * cx = bx · P/(bz + P) — both bx and bz need full precision. The z
 * axis also drives the depth-bucket glyph (bz > 0.8·P → '.' far,
 * > -0.4·P → 'o' mid, else → '@' near), so without it every blob
 * would project at the same rate and the burst would read as flat
 * 2-D fireworks. The y component is multiplied by sh.y_squash at
 * render time, which lets the SAME sphere read as flat disc, sphere,
 * or tall column per shape — no per-shape pool needed.
 */
typedef struct {
    double x, y, z;
} Blob;

static double prng(void)
{
    static long long s = 1;
    s = s * 1488248101LL + 981577151LL;
    return ((s % 65536) - 32768) / 32768.0;
}

/*
 * blob_sample_unit_direction — sample one (oblate) unit direction.
 *
 *   1. Pick a random point (bx, by, bz) ∈ [-1, 1]³ from the LCG.
 *   2. Divide by ‖(bx, by, bz)‖ to project onto the unit sphere.
 *      This is rejection-free — accepts the cube-corner bias since
 *      visually it's indistinguishable for an explosion.
 *   3. Multiply y by 0.5 BEFORE storing → the cloud is squashed
 *      vertically into an oblate spheroid, which renders better on
 *      a 2:1-cell-aspect terminal where pure spheres look stretched.
 *   4. Scale each axis by 1.3 + 0.2·rand so the cloud has a slight
 *      radial thickness instead of every blob leaving at exactly
 *      the same rate.
 */
static void blob_sample_unit_direction(Blob *out)
{
    double bx = prng();
    double by = prng();
    double bz = prng();
    double br = sqrt(bx*bx + by*by + bz*bz);
    out->x = (bx / br)         * (1.3 + 0.2 * prng());
    out->y = (0.5 * by / br)   * (1.3 + 0.2 * prng());
    out->z = (bz / br)         * (1.3 + 0.2 * prng());
}

/* Fill the pool with NUM_BLOBS independent direction samples. */
static void blob_init_pool(Blob *blobs)
{
    for (int i = 0; i < NUM_BLOBS; i++)
        blob_sample_unit_direction(&blobs[i]);
}

/*
 * BlastShape — physical + visual knobs that distinguish one blast
 * "shape" from another. blast_render_frame reads these every tick;
 * the rendering algorithm itself never branches on shape index. Same
 * code, six very different blasts — change shape via the n/N keys.
 *
 *   name        : short label shown in the HUD so the viewer can see
 *                 which shape is active.
 *   petal_n     : angular frequency of the cos(petal_n · atan2(2y,x))
 *                 lobe modulation. INTEGER counts of lobes:
 *                 0 = smooth ring (no angular modulation; cos drops out
 *                     via the petal_n > 0 guard in blast_render_frame);
 *                 1 = asymmetric teardrop; 4 = cross; 6 = hex-star;
 *                 8 = classic; 12 = nova; 16 = spiky.
 *                 Why a double? — passed to libm's cos() which wants
 *                 a double anyway, and fractional values can give
 *                 chaotic non-symmetric shapes if ever needed.
 *   ripple      : amplitude of the angular ripple. 0 = perfectly
 *                 smooth ring (the ring shape); 0.3 = subtle classic
 *                 lobes; 0.6 = very jagged pulse. Multiplies the cos
 *                 term so it controls how DEEPLY the lobes cut.
 *   disc_speed  : cells/frame at which the initial fireball disc
 *                 grows during frames 1..7 (the pre-wave bloom).
 *                 1.5 = slow puff (star), 3.5 = explosive flash (nova).
 *                 Picked together with the wave_chars length so the
 *                 disc-to-wave handoff at frame 8 looks smooth.
 *   y_squash    : vertical scale applied to blob.y at render time.
 *                 0.3 = flat puck (ring); 1.0 = true sphere; 1.6 =
 *                 tall column (star). Lets every shape reuse the
 *                 same pre-sampled Blob pool but read as a different
 *                 silhouette.
 *   persp       : "P" in the pinhole projection cx = cols/2 + bx·P/(bz+P).
 *                 SMALL persp (25 = pulse) flattens the cloud — every
 *                 blob looks roughly the same size; LARGE persp (80
 *                 = nova) opens up depth — near blobs balloon, far
 *                 blobs shrink to dots. Also the z-cull bounds use
 *                 persp directly (bz < 5-persp or bz > persp).
 *   blob_speed  : outward velocity scale for the 3-D blob cloud.
 *                 Multiplied against (frame - 6) so each blob travels
 *                 at blob * blob_speed * (frame-6). 0.7 = slow drift
 *                 (pulse); 1.6 = blast outward (nova). Tune together
 *                 with persp so blobs don't all rocket off-screen
 *                 before the shockwave catches up.
 *   flash_chars : character ramp for the INNER pre-wave region
 *                 (frames 8..8+len, color = COL_INNER). First char
 *                 hottest, last char coolest; the index into this
 *                 string is (frame - 8). Length controls how long
 *                 the bright core lingers before the wave takes over.
 *   wave_chars  : character ramp for the EXPANDING shockwave. Each
 *                 cell looks up wave_chars[frame - r - 7]; the index
 *                 grows with frame so the same cell cycles through
 *                 the whole ramp as the wave sweeps past. First
 *                 char = leading edge (faint), last char = trailing
 *                 (densest). Length controls wave THICKNESS in cells.
 */
typedef struct {
    const char *name;
    double      petal_n;
    double      ripple;
    double      disc_speed;
    double      y_squash;
    double      persp;
    double      blob_speed;
    const char *flash_chars;
    const char *wave_chars;
} BlastShape;

static const BlastShape k_shapes[] = {
    {   /* 0  classic — original algorithm */
        "classic",
        16.0, 0.3, 2.0, 0.5, 50.0, 1.0,
        "T%@W#H=+~-:.",
        " .:!HIOMW#%$&@08O=+-"
    },
    {   /* 1  star — 6-pointed, slow thick wave, tall blob column */
        "star",
        6.0,  0.45, 1.5, 1.6, 35.0, 0.8,
        "*+oO0@#%&$!^~",
        " `.-:=+*oO0#@%$"
    },
    {   /* 2  ring — smooth sphere, fast thin ring, flat disc blobs */
        "ring",
        0.0,  0.0,  3.0, 0.3, 70.0, 1.4,
        "o0OQ@#%&$()[]{}",
        " .,:;!|/\\-=+~*oO"
    },
    {   /* 3  cross — 4 lobes, medium speed, medium depth */
        "cross",
        4.0,  0.5,  2.5, 0.8, 45.0, 1.1,
        "#@WMH+|=~-:.",
        " :-=+|H#@WM0O%$"
    },
    {   /* 4  nova — 12 lobes, very fast, deep 3D blobs */
        "nova",
        12.0, 0.35, 3.5, 1.0, 80.0, 1.6,
        "%$&#@!*+~-:.",
        " .`'^-~=+*#@$%&!"
    },
    {   /* 5  pulse — asymmetric teardrop, slow, very flat blobs */
        "pulse",
        3.0,  0.6,  1.2, 0.25, 25.0, 0.7,
        "~-:.+=#@*oO0Q",
        " ..,::==++##@@%%"
    },
};

#define SHAPE_COUNT (int)(sizeof k_shapes / sizeof k_shapes[0])

/* ===================================================================== */
/* §5  blast                                                              */
/* ===================================================================== */

/*
 * Cell — one slot in the staging frame buffer Cell[cols*rows].
 *
 * The blast renders into this back-buffer first; only after every
 * layer (disc, wave, blobs) has written does blast_draw blit the
 * non-empty cells to stdscr. This decouples the layer-overlap logic
 * ("last writer wins" — blobs paint on top of the wave because they
 * loop second) from the ncurses I/O, so overlap resolution is plain
 * memory writes. `ch == 0` is the empty sentinel: blast_draw tests
 * `!c.ch` to skip cells no layer wrote, which is how the black
 * background stays black without an explicit fill pass. `color` is
 * a ColorID picked at write time; COL_FLASH additionally gets
 * A_BOLD in blast_draw so the initial fireball pops brighter than
 * everything else on screen.
 */
typedef struct {
    char    ch;
    ColorID color;
} Cell;

/*
 * Blast — the entire simulation state for ONE explosion cycle.
 * Allocated once at startup; torn down and rebuilt on resize and on
 * theme/shape change (see app_reset_blast). blast_tick advances
 * `frame`; blast_render_frame writes the wave + blobs into cells[];
 * blast_draw blits the non-empty entries to stdscr.
 *
 *   blobs[NUM_BLOBS]
 *           Pre-distributed 3-D debris cloud (see Blob). 800
 *           unit-direction vectors, generated once per blast in
 *           blob_init_pool. The LCG inside prng() never resets, so
 *           successive blasts within one run get DIFFERENT
 *           distributions — every cycle looks fresh.
 *
 *   cells   Pointer to a calloc'd Cell[cols*rows] back-buffer.
 *           Allocated once per blast (one of the very few mallocs
 *           in the program — see "Memory Allocation" in CLAUDE.md).
 *           blast_render_frame fills it; blast_draw drains it. Kept
 *           separate from stdscr so the layer-overlap logic
 *           (wave-then-blobs, last-writer-wins) is plain memory
 *           writes, not ncurses calls.
 *
 *   cols    Cached terminal column count at the moment of init/resize.
 *           Used everywhere as the basis for the centred coordinate
 *           system [-cols/2, +cols/2] in which the algorithm thinks.
 *           Cached locally so the inner render loop never calls
 *           getmaxyx() — that would re-read the terminfo state.
 *
 *   rows    Cached terminal row count (same role as cols, vertical).
 *           The aspect-correction factor `4·y²` (terminal cells are
 *           ~2× tall as wide) is applied at the radius computation
 *           so a circle in cell-space looks circular on screen.
 *
 *   frame   Tick counter, 0..NUM_FRAMES-1. Drives the THREE stages
 *           of the visual algorithm:
 *             frame == 0     → single '*' FLASH at origin
 *             frame  < 8     → filled disc fireball, radius
 *                              = frame · disc_speed
 *             frame >= 8     → angular shockwave + 3-D blobs
 *           Also feeds the blob outward velocity formula (multiplied
 *           by frame-6). When frame reaches NUM_FRAMES the cycle
 *           ends; the main loop advances theme + shape and calls
 *           app_reset_blast.
 *
 *   theme   Index into k_themes — selects the FLASH / INNER / WAVE /
 *           BLOB_F / BLOB_M / BLOB_N colour-pair set. Mutated by
 *           t / T keys or by the auto-end path; the value here is
 *           "the palette currently being painted". Also displayed
 *           in the HUD so the viewer can identify the active theme.
 *
 *   shape   Index into k_shapes — selects every per-shape parameter
 *           (petal_n, ripple, disc_speed, y_squash, persp,
 *           blob_speed, flash_chars, wave_chars). Mutated by n / N
 *           keys or by the auto-end path. Decoupling shape from
 *           theme lets the viewer e.g. watch all 10 colour palettes
 *           on the same "classic" silhouette.
 *
 *   done    Sticky one-shot end-of-cycle flag set when frame reaches
 *           NUM_FRAMES. blast_tick checks it on entry and returns
 *           false immediately, signalling the main loop to schedule
 *           an app_reset_blast (which clears `done` along with
 *           everything else).
 */
typedef struct {
    Blob  blobs[NUM_BLOBS];
    Cell *cells;
    int   cols;
    int   rows;
    int   frame;
    int   theme;
    int   shape;
    bool  done;
} Blast;

static void blast_alloc_cells(Blast *b)
{
    b->cells = calloc((size_t)(b->cols * b->rows), sizeof(Cell));
}

static void blast_init(Blast *b, int cols, int rows, int theme, int shape)
{
    b->cols  = cols;
    b->rows  = rows;
    b->frame = 0;
    b->theme = theme;
    b->shape = shape;
    b->done  = false;
    blast_alloc_cells(b);
    blob_init_pool(b->blobs);
    color_theme_apply(theme);
}

static void blast_free(Blast *b)
{
    free(b->cells);
    *b = (Blast){0};
}

/* ── Pure-math helpers ───────────────────────────────────────────── */

/* Aspect-corrected radius — terminal cells are ~2× tall as wide, so
 * y² is weighted ×4 to keep a circle in cell-space looking circular
 * on screen. Used by both the disc layer (frames 1-7) and the
 * shockwave layer (frames ≥ 8). */
static inline double aspect_radius(int x, int y)
{
    return sqrt((double)(x * x) + 4.0 * (double)(y * y));
}

/* Angular lobe modulation: 1 + ripple·cos(petal_n · θ), with θ
 * computed in the same aspect-corrected coordinate frame (2·y, x).
 * The tiny + 0.01 offsets keep atan2 well-defined at the origin.
 * petal_n ≤ 0 is the "smooth ring" guard — cos(0) = 1 would still
 * add a constant amplitude bump, so we short-circuit to 1.0 to keep
 * the ring perfectly round. */
static inline double petal_lobe(int x, int y, double petal_n, double ripple)
{
    if (petal_n <= 0.0) return 1.0;
    double angle = atan2((double)y * 2.0 + 0.01, (double)x + 0.01);
    return 1.0 + ripple * cos(petal_n * angle);
}

/* Pinhole projection along one axis: b_screen = b · P / (bz + P).
 * Called twice per blob, once each for x and y. The same P also
 * appears in the z-cull bounds and the depth-bucket cutoffs. */
static inline double perspective_project(double b, double bz, double persp)
{
    return b * persp / (bz + persp);
}

/* ── Per-cell painters (one cell, one frame phase) ───────────────── */

/* Reset a cell to "empty". ch == 0 is the sentinel blast_draw uses to
 * skip cells no layer wrote, which is how the black background stays
 * black without an explicit fill pass. Default colour is COL_WAVE so
 * callers only assign colour when they paint a different layer. */
static inline void cell_clear(Cell *c)
{
    c->ch    = 0;
    c->color = COL_WAVE;
}

/* Frame 0: plant a single bright '*' at the blast origin so the eye is
 * drawn there before the disc bloom and shockwave take over. */
static inline void cell_paint_origin_flash(Cell *c, int x, int y)
{
    if (x == 0 && y == 0) {
        c->ch    = '*';
        c->color = COL_FLASH;
    }
}

/* Frames 1..7: filled '@' fireball disc growing at disc_speed cells
 * per frame. Uniform colour, no angular modulation. The disc-to-wave
 * handoff happens implicitly at frame == 8 when the caller switches
 * to cell_paint_shockwave. */
static inline void cell_paint_disc(Cell *c, int x, int y,
                                   int frame, double disc_speed)
{
    if (aspect_radius(x, y) < (double)frame * disc_speed) {
        c->ch    = '@';
        c->color = COL_FLASH;
    }
}

/* Frames ≥ 8: angular shockwave with petal-modulated radius.
 *
 *   r = aspect_radius · (0.5 + prng/3 · lobe · 0.3)
 *   v = frame − r − 7              ramp index into the wave
 *
 *     v < 0          → INNER bright core, glyph = flash_chars[frame-8]
 *     0 ≤ v < waveN  → wave_chars[v], INNER colour first half,
 *                                     WAVE colour second half
 *     v ≥ waveN      → cell stays empty (wave has passed this cell)
 *
 * The per-cell prng() call jitters the radius so the wave is not a
 * perfectly regular curve — gives the explosion a noisy, organic edge. */
static inline void cell_paint_shockwave(Cell *c, int x, int y, int frame,
                                        const BlastShape *sh,
                                        int flash_len, int wave_len)
{
    double lobe = petal_lobe(x, y, sh->petal_n, sh->ripple);
    double r    = aspect_radius(x, y) * (0.5 + (prng() / 3.0) * lobe * 0.3);
    int    v    = frame - (int)r - 7;

    if (v < 0) {
        int fi = frame - 8;
        if (fi >= 0 && fi < flash_len) {
            c->ch    = sh->flash_chars[fi];
            c->color = COL_INNER;
        }
    } else if (v < wave_len) {
        c->ch    = sh->wave_chars[v];
        c->color = (v < wave_len / 2) ? COL_INNER : COL_WAVE;
    }
}

/* ── Wave-layer driver (the (x, y) double loop) ──────────────────── */

/* Sweep every cell in the screen-centred grid and paint whichever
 * phase-of-blast applies for this frame. The frame-phase decision
 * happens once per cell (cheap branch the predictor nails), so a
 * single loop fans out into three distinct visual stages. */
static void wave_layer_render(Blast *b)
{
    const int cols  = b->cols;
    const int rows  = b->rows;
    const int frame = b->frame;
    const BlastShape *sh = &k_shapes[b->shape];

    const int minx = -(cols / 2);
    const int maxx = cols + minx - 1;
    const int miny = -(rows / 2);
    const int maxy = rows + miny - 1;

    const int flash_len = (int)strlen(sh->flash_chars);
    const int wave_len  = (int)strlen(sh->wave_chars);

    Cell *p = b->cells;
    for (int y = miny; y <= maxy; y++) {
        for (int x = minx; x <= maxx; x++) {
            cell_clear(p);
            if      (frame == 0) cell_paint_origin_flash(p, x, y);
            else if (frame  < 8) cell_paint_disc       (p, x, y, frame, sh->disc_speed);
            else                 cell_paint_shockwave  (p, x, y, frame, sh,
                                                        flash_len, wave_len);
            p++;
        }
    }
}

/* ── Per-blob painter and blob-layer driver ──────────────────────── */

/* Resolve the depth-bucket glyph and colour for one blob based on its
 * z-position relative to the perspective depth P.
 *   bz >  0.8·P → '.' COL_BLOB_F  (far — distant dot)
 *   bz > -0.4·P → 'o' COL_BLOB_M  (middle distance)
 *   else        → '@' COL_BLOB_N  (near — bold lump)
 */
static inline void blob_depth_bucket(double bz, double persp,
                                     char *out_glyph, ColorID *out_color)
{
    if      (bz >  persp * 0.8) { *out_glyph = '.'; *out_color = COL_BLOB_F; }
    else if (bz > -persp * 0.4) { *out_glyph = 'o'; *out_color = COL_BLOB_M; }
    else                        { *out_glyph = '@'; *out_color = COL_BLOB_N; }
}

/* Project one 3-D blob to a 2-D Cell, or cull it.
 *   outward sweep:  position(t) = blob · (frame - 6) · blob_speed
 *   y-squash:       by *= sh->y_squash  (per-shape silhouette)
 *   z-cull:         keep only blobs with bz ∈ [5 - persp, persp]
 *   projection:     cx = cols/2 + bx·P/(bz + P)  (likewise cy)
 *   x/y bounds:     skip if the projected pixel falls off screen
 *   depth glyph:    via blob_depth_bucket
 * The single Cell write at the end overwrites whatever the wave layer
 * left in that slot — this is the "blobs on top" layer ordering rule. */
static void blob_paint_projected(const Blob *blob, Cell *cells,
                                 int cols, int rows,
                                 int frame, const BlastShape *sh)
{
    const double persp  = sh->persp;
    const double bspeed = sh->blob_speed;
    const int    t      = frame - 6;

    double bx = blob->x * t * bspeed;
    double by = blob->y * t * bspeed * sh->y_squash;
    double bz = blob->z * t * bspeed;

    if (bz < 5.0 - persp || bz > persp) return;

    int cx = cols / 2 + (int)perspective_project(bx, bz, persp);
    int cy = rows / 2 + (int)perspective_project(by, bz, persp);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    Cell *c = &cells[cy * cols + cx];
    blob_depth_bucket(bz, persp, &c->ch, &c->color);
}

/* Sweep the whole blob cloud and project each into the cell buffer.
 * Called only when frame > 6 — earlier ticks belong entirely to the
 * disc/origin-flash layers. Runs AFTER wave_layer_render so blobs
 * overdraw the shockwave at any cell they both touch. */
static void blob_layer_render(Blast *b)
{
    const BlastShape *sh = &k_shapes[b->shape];
    for (int j = 0; j < NUM_BLOBS; j++) {
        blob_paint_projected(&b->blobs[j], b->cells,
                             b->cols, b->rows, b->frame, sh);
    }
}

/* ── Driver — pseudocode for one frame ──────────────────────────── */

/* Render one frame into the cell buffer:
 *   1. wave layer  — origin spark / disc / angular shockwave
 *   2. blob layer  — 3-D debris (frame > 6), overdraws the wave
 */
static void blast_render_frame(Blast *b)
{
    wave_layer_render(b);
    if (b->frame > 6) blob_layer_render(b);
}

static bool blast_tick(Blast *b)
{
    if (b->done) return false;

    blast_render_frame(b);
    b->frame++;

    if (b->frame >= NUM_FRAMES) {
        b->done = true;
        return false;
    }

    return true;
}

/* Blit one cell to the window if it was painted (ch != 0).
 * COL_FLASH additionally turns on A_BOLD so the initial fireball
 * and disc reads brighter than the wave / blob layers. */
static inline void cell_blit(WINDOW *w, int x, int y, Cell c)
{
    if (!c.ch) return;
    attr_t attr = COLOR_PAIR(c.color);
    if (c.color == COL_FLASH) attr |= A_BOLD;
    wattron(w, attr);
    mvwaddch(w, y, x, (chtype)(unsigned char)c.ch);
    wattroff(w, attr);
}

/* Walk the back-buffer and emit ncurses writes for every painted cell.
 * Takes WINDOW* so the same routine drives stdscr or any sub-window. */
static void blast_draw(const Blast *b, WINDOW *w)
{
    const int cols = b->cols;
    const int rows = b->rows;
    for (int i = 0; i < cols * rows; i++)
        cell_blit(w, i % cols, i / cols, b->cells[i]);
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

/*
 * Screen — single stdscr, ncurses' internal double buffer.
 *
 * erase()             — clear newscr (back buffer), no terminal I/O
 * blast_draw(stdscr)  — write blast into newscr
 * mvprintw / attron   — write HUD into newscr after blast (on top)
 * wnoutrefresh()      — mark newscr ready, still no terminal I/O
 * doupdate()          — ONE atomic write: diff newscr vs curscr → terminal
 *
 * typeahead(-1) prevents ncurses interrupting output mid-flush to poll
 * stdin, eliminating tearing at high tick rates.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_blast(Screen *s, const Blast *b)
{
    erase();
    blast_draw(b, stdscr);
}

/*
 * screen_draw_hud — paint a two-layer HUD over the blast:
 *
 *   Row 0          — STATUS LINE.  Bright yellow COL_HUD + A_BOLD.
 *                    Shows theme, shape, frame counter, render fps, sim Hz.
 *   Row rows-1     — KEY HINT LINE.  Bright cyan COL_HINT + A_BOLD.
 *                    Lists every interactive key the demo accepts.
 *
 * Both rows are cleared with their pair colour first so the coloured
 * background spans the full width. Drawn AFTER blast_draw so blast
 * glyphs never bleed through.
 */
static void screen_draw_hud(Screen *s, double fps, int sim_fps,
                              int frame, int theme, int shape)
{
    /* ── Top row: status ──────────────────────────────────────── */
    char status[200];
    snprintf(status, sizeof status,
             " KABOOM   theme:%-7s   shape:%-8s   frame:%3d/%3d   "
             "%4.1f fps  %2d Hz ",
             k_themes[theme].name, k_shapes[shape].name,
             frame, NUM_FRAMES, fps, sim_fps);

    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    for (int x = 0; x < s->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%s", status);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);

    /* ── Bottom row: key hints (every interactive key) ────────── */
    const char *hints =
        " q:quit  r:replay  t/T:theme  n/N:shape  ]/[:speed ";

    int hint_row = s->rows - 1;
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    for (int x = 0; x < s->cols; x++) mvaddch(hint_row, x, ' ');
    mvprintw(hint_row, 0, "%s", hints);
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

typedef struct {
    Blast                 blast;
    Screen                screen;
    int                   sim_fps;
    int                   theme_idx;  /* live theme; t/T mutate, auto-end bumps */
    int                   shape_idx;  /* live shape; n/N mutate, auto-end bumps */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Restart the blast from frame 0 with the current theme + shape indices.
 * Called by r, t, T, n, N, and the auto-end-of-cycle path. */
static void app_reset_blast(App *app)
{
    blast_free(&app->blast);
    blast_init(&app->blast, app->screen.cols, app->screen.rows,
               app->theme_idx, app->shape_idx);
}

static void app_do_resize(App *app)
{
    blast_free(&app->blast);
    screen_resize(&app->screen);
    blast_init(&app->blast, app->screen.cols, app->screen.rows,
               app->theme_idx, app->shape_idx);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 'r': case 'R':
        app_reset_blast(app);
        break;

    case 't':
        app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
        app_reset_blast(app);
        break;
    case 'T':
        app->theme_idx = (app->theme_idx + THEME_COUNT - 1) % THEME_COUNT;
        app_reset_blast(app);
        break;

    case 'n':
        app->shape_idx = (app->shape_idx + 1) % SHAPE_COUNT;
        app_reset_blast(app);
        break;
    case 'N':
        app->shape_idx = (app->shape_idx + SHAPE_COUNT - 1) % SHAPE_COUNT;
        app_reset_blast(app);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app       = &g_app;
    app->running   = 1;
    app->sim_fps   = SIM_FPS_DEFAULT;
    app->theme_idx = 0;
    app->shape_idx = 0;

    screen_init(&app->screen);
    blast_init(&app->blast, app->screen.cols, app->screen.rows,
               app->theme_idx, app->shape_idx);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            if (!blast_tick(&app->blast)) {
                app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
                app->shape_idx = (app->shape_idx + 1) % SHAPE_COUNT;
                app_reset_blast(app);
            }
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── HUD counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        int64_t budget  = NS_PER_SEC / 60;
        clock_sleep_ns(budget - elapsed);

        /* ── render + HUD ────────────────────────────────────────── */
        screen_draw_blast(&app->screen, &app->blast);
        screen_draw_hud(&app->screen, fps_display,
                         app->sim_fps, app->blast.frame,
                         app->blast.theme, app->blast.shape);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    blast_free(&app->blast);
    screen_free(&app->screen);
    return 0;
}
