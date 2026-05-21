/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun_rain.c — radial matrix-rain sun corona
 *
 * DEMO: A single '@' burns at the screen centre. 180 independent
 *       radial streams of matrix glyphs shoot outward from it in
 *       all directions — a continuous solar wind with no circle,
 *       no disc, no border. Each stream has its own random speed
 *       and a stagger offset so they appear at different distances
 *       from the core at any given moment, producing a stochastic
 *       field of beams rather than a single synchronised burst.
 *
 *       Sketch (one frozen frame; 180 rays, only 12 drawn for
 *       clarity, with their head positions × and tails →):
 *
 *                  ×
 *               × ←  ← ←
 *           ×   ←
 *           ×       ←
 *        ×             ←
 *           ×              ×
 *                @           ×
 *           ×              ×
 *        ×             ←
 *           ×       ←
 *           ×   ←
 *               × ←  ← ←
 *                  ×
 *
 *       The core '@' is drawn LAST, every frame, so no ray ever
 *       overwrites it. Each ray's trail fades from a bright white
 *       head through five themed shades to a barely-visible tail.
 *
 * Study alongside:
 *   matrix_rain/matrix_rain.c       — same shimmer-cache trick on
 *                                     plain vertical falling streams.
 *   matrix_rain/pulsar_rain.c       — the rotating sibling: rays go
 *                                     SIDEWAYS instead of OUTWARD;
 *                                     this file is what you get if
 *                                     you spin omega = 0 and let the
 *                                     rays slide along their angle.
 *   matrix_rain/fireworks_rain.c    — same cache-shimmer trick on
 *                                     parabolic-arc spark trails.
 *   matrix_rain/matrix_snowflake.c  — rain accumulating into snow.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 5 themed 5-shade palettes
 *   §4  scene        — Ray type (§4.2-4.5); RayField + SunGeometry +
 *                       SimControls sub-structs; Scene root (field +
 *                       geom + sim + theme_idx); scene_init / _reset /
 *                       _tick / _draw + named helpers
 *                       (compute_sun_geometry, spawn_rays_evenly_spaced,
 *                       advance_one_ray_with_recycle);
 *                       ray_tick split into shimmer_reroll_ray_cache +
 *                       tail_tip_inside_recycle_horizon;
 *                       ray_draw split into trail_cell_radius_from_head +
 *                       polar_to_screen_cell + paint_ray_trail_cell;
 *                       sun_draw_core (the '@'); input helper
 *                       (scene_cycle_theme)
 *   §5  screen       — ncurses init / present; screen_draw_hud split
 *                       into hud_paint_text / format_hud_status /
 *                       draw_hud_status / draw_hud_hint
 *   §6  app          — FpsCounter + App; signals, resize, key dispatch,
 *                       variable-dt main loop (7 numbered phases);
 *                       scene_reinit_preserving_knobs (SIGWINCH helper)
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume
 *   r                reset (re-stagger all 180 rays)
 *   t                cycle theme (5 themes)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/sun_rain.c \
 *       -o sun_rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Polar-coordinate radial sweep with shimmer cache.
 *
 *                 (A) RAYS    — N_RAYS independent streams, one per
 *                     evenly-spaced angle θ_i = 2π · i / N. Each ray
 *                     owns a head distance r_off (cells from centre,
 *                     float for sub-cell smoothness), a speed
 *                     (cells/sec), and a per-ray cache of RAY_TRAIL
 *                     random ASCII glyphs that reroll every frame.
 *                     When the tail clears max_r, the ray "dies"
 *                     and is reset with a new stagger and speed.
 *
 *                 (B) MOTION  — variable-dt integration: each frame,
 *                     `r_off += speed_cps · dt`. No fixed-step
 *                     accumulator, no alpha interpolation: the float
 *                     r_off gives smooth sub-cell motion at any
 *                     frame rate. Variable dt is unconditionally
 *                     stable here because the sim is non-stiff
 *                     (constant per-ray speed, no springs, no
 *                     contacts).
 *
 *                 (C) SHIMMER — for each cache slot, with probability
 *                     1 − 1/SHIMMER_KEEP_ONE_IN, reroll the glyph.
 *                     KEEP_ONE_IN = 4 → 75 % rerolled each frame —
 *                     the classic Matrix-rain shimmer rate.
 *
 * Data-structure: Scene = RayField + SunGeometry + SimControls +
 *                 theme_idx, each a named sub-struct (see §4 for the
 *                 layered-ownership diagram).  RayField wraps a
 *                 fixed 180-slot Ray array (no heap allocation
 *                 post-init).  Each Ray carries pre-baked
 *                 cos_a/sin_a direction components (set once at
 *                 ray_init), the float r_off head distance, the
 *                 speed (cells/sec), and a 16-byte glyph cache.
 *                 SunGeometry caches the screen centre (cx, cy) and
 *                 the max_r recycle threshold; both are derived
 *                 ONCE from the screen extent in scene_init and
 *                 again in app_do_resize.
 *
 * Rendering     : For each ray, walk i = 0..RAY_TRAIL-1 from head
 *                 backward (i = 0 is the head, i = RAY_TRAIL-1 is
 *                 the tail tip). At each step compute
 *                     ri = r_off - i
 *                 and break if ri < CORE_RESERVED_RADIUS so the trail
 *                 never overwrites the central '@'. The remaining
 *                 cells project to (col, row) via the pre-baked
 *                 (cos_a, sin_a · ASPECT) direction. ASPECT (≈ 0.45)
 *                 is BAKED INTO sin_a so terminal-cell anisotropy is
 *                 corrected exactly once.
 *
 * Performance   : Per frame: O(N_RAYS · RAY_TRAIL) cell paints. With
 *                 defaults (180 · 16 = 2880 mvaddch + 180 · 16 cache
 *                 rerolls), microseconds. ncurses redraw is the
 *                 dominant cost. Pre-baked direction vectors mean
 *                 zero trig in the inner loop.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Astrophysics: what the visual is emulating ─────────────────
 *   [1] Parker, E. N. (1958), "Dynamics of the Interplanetary Gas
 *       and Magnetic Fields", Astrophys. J. 128, pp. 664-676 — the
 *       canonical solar-wind paper.  Hot coronal plasma streaming
 *       radially outward from the Sun is exactly what each Ray in
 *       this demo emulates: a stream of bright "particles" leaving
 *       the core (cx, cy) along a fixed direction, fading with
 *       distance.  Real coronal plasma accelerates with altitude;
 *       our rays use CONSTANT speed for visual simplicity (see
 *       "Why constant speed" note in Ray struct doc, §4).
 *   [2] Wikipedia, "Solar wind" + "Corona (Sun)" — quick online
 *       references for the lighthouse-model physics.
 *       https://en.wikipedia.org/wiki/Solar_wind
 *
 *   ── Per-ray particle model ─────────────────────────────────────
 *   [3] Reeves, W. T. (1983), "Particle Systems — A Technique for
 *       Modeling a Class of Fuzzy Objects", ACM SIGGRAPH '83 / ACM
 *       TOG 2(2), pp. 91-108 — canonical particle-systems paper.
 *       Each Ray in §4 is a Reeves particle confined to a fixed
 *       radial direction (cos_a, sin_a·ASPECT).  The spawn → fly
 *       → recycle lifecycle is Reeves's "expire → recycle" pattern
 *       with the tail-clears-screen event substituting for
 *       off-screen expiry.
 *
 *   ── Polar sampling + cell aspect ───────────────────────────────
 *   [4] Bresenham, J. E. (1965), "Algorithm for computer control
 *       of a digital plotter", IBM Sys. J. 4(1).  Bresenham's
 *       integer line/circle algorithms are the lineage for our
 *       discrete radial sampling at (cx + r·cosθ, cy + r·sinθ).
 *       We use float roundf instead of integer DDA because the
 *       glyph cache breaks the constant-line assumption Bresenham
 *       relies on — but the sampling pattern (radial lines from a
 *       common origin) is the same idea.
 *   [5] Cell-aspect correction reference: terminal cells are
 *       ~1 column wide × 2 rows tall.  We multiply sin(θ) by
 *       ASPECT to make rays appear evenly spaced around a circle
 *       instead of vertically squished into an ellipse.  Same trick
 *       used across this project — see CLAUDE.md §"Coordinate /
 *       Physics" + the inline note in ray_init.
 *
 *   ── Shimmer cache (the visual heritage) ────────────────────────
 *   [6] *The Matrix* (1999, dir. Wachowski) — the rerolling-glyph
 *       rain aesthetic that the per-ray head + trail borrows.
 *       Original used mirror-image katakana; this file uses ASCII
 *       for portability across non-UTF-8 terminals (CLAUDE.md
 *       §"ASCII-Only Rendering").
 *   [7] Wikipedia, "Matrix digital rain" — Whiteley font + demoscene
 *       precedents.
 *       https://en.wikipedia.org/wiki/Matrix_digital_rain
 *
 *   ── Variable-timestep game loop ────────────────────────────────
 *   [8] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       canonical case for FIXED-step physics.  This file
 *       deliberately uses VARIABLE dt: each ray's `r_off += speed·dt`
 *       update is non-stiff and unconditionally stable.  Fiedler's
 *       DT_CAP rule (100 ms hard cap, DT_CAP_SEC here) is the only
 *       piece we adopt.
 *
 *   ── ncurses rendering substrate ────────────────────────────────
 *   [9] Raymond, E. S., "NCURSES Programming HOWTO" — §6 (colour
 *       pairs) for the theme-cycling design, §11 (output options)
 *       for the wnoutrefresh + doupdate diff-only write pattern
 *       that keeps the field flicker-free at 60 fps.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     matrix_rain/matrix_rain.c     — same shimmer-cache pattern
 *       on plain VERTICAL streams (no radial geometry).  Read this
 *       first if the per-ray glyph-cache trick is unfamiliar.
 *     matrix_rain/pulsar_rain.c     — same polar-coordinate
 *       machinery with rays that ROTATE instead of translate
 *       outward.  Trade between this file and pulsar_rain.c shows
 *       the radial-translate vs angular-rotate dichotomy with
 *       otherwise-identical infrastructure (Scene-rooted state,
 *       cell-aspect correction, shimmer cache).
 *     matrix_rain/fireworks_rain.c  — same shimmer on ARC
 *       trajectories (rocket bursts) — another radial-fan variant.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take The Matrix's vertical green rain, replace gravity with a
 * centre, and let each drop choose its own outward angle. With 180
 * angles spaced 2° apart, each carrying its own speed and stagger,
 * you get a continuous solar-corona effect: characters streaming
 * radially in every direction from a single '@' core, never
 * repeating, never pausing. The whole thing is the same per-stream
 * cache + fade-by-distance trick from matrix_rain.c, just using
 * polar coordinates instead of (x, y).
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. dt = wall-clock seconds since last frame, capped at DT_CAP_SEC.
 *  2. scene_tick(dt):
 *     For each ray:
 *       if ray_tick(ray, dt) returns false → ray_reset(ray)
 *  3. erase()
 *  4. scene_draw():
 *       For each ray:
 *         For slot_i = 0..RAY_TRAIL-1:
 *           radius = ray->r_off - slot_i
 *           break if radius < CORE_RESERVED_RADIUS
 *           col = cx + round(radius · cos_a)
 *           row = cy + round(radius · sin_a)
 *           paint ray->cache[slot_i] in dist_attr(slot_i)
 *       Draw '@' core at (cx, cy) — drawn LAST so it always wins.
 *  5. HUD: yellow status row 0; cyan hint strip row last.
 *  6. doupdate; sleep to TARGET_FPS cap.
 *
 * KEY FORMULAS
 * ────────────
 *   θ_i       = 2π · i / N_RAYS                  ray angle
 *   cos_a     = cos(θ)                            horizontal direction
 *   sin_a     = sin(θ) · ASPECT                   vertical, anisotropy-corrected
 *   col, row  = cx + round(ri · cos_a),
 *               cy + round(ri · sin_a)
 *   ri        = r_off - i                         distance for trail slot i
 *   r_off(t)  = r_off(0) + speed_cps · t          smooth radial motion
 *   max_r     = cols + rows / ASPECT              farthest corner from centre
 *   stagger   = -uniform([0, STAGGER_FRAC · max_r])
 *
 * Background you need
 * ───────────────────
 *   - matrix_rain T1-T5 (stream-per-X, shimmer, bands).
 *   - pulsar_rain T1-T3 (polar geometry, ASPECT correction).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Solar physics. The core is decorative; we don't simulate
 *     thermonuclear fusion.
 *   - Phong / radiometric shading. The "brightness" is just the
 *     6-band ramp from matrix_rain T4.
 *   - Per-ray rotation. Rays are FIXED in angle; only their
 *     head distance changes.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum {
    TARGET_FPS = 60,
};

/* ── §1.2 ray geometry ────────────────────────────────────────────── */
enum {
    /* N_RAYS — number of radial streams emanating from the core.
     * 180 rays = 2° angular spacing — dense enough to read as a
     * continuous corona; smaller leaves visible gaps. */
    N_RAYS    = 180,

    /* RAY_TRAIL — characters per ray tail. Also the array bound on
     * each ray's glyph cache. */
    RAY_TRAIL = 16,

    /* Brightness band boundaries (used by dist_attr). */
    TRAIL_HOT_END   = 2,                /* [0..2]      → BOLD          */
    TRAIL_WARM_END  = RAY_TRAIL / 2,    /* [3..N/2-1]  → NORMAL        */
                                        /* [N/2..N-1]  → DIM           */
};

/*
 * ASPECT — terminal cell height/width ratio correction.
 * Baked into sin_a (= sin θ · ASPECT) once per ray at init, so every
 * inner-loop position formula is just:
 *     col = cx + ri · cos_a
 *     row = cy + ri · sin_a
 * Without ASPECT, vertical rays walk twice as fast as horizontal ones
 * (terminal cells are ~2× tall as they are wide) and the sun becomes
 * a tall ellipse instead of round.
 */
#define ASPECT  0.45f

/*
 * CORE_RESERVED_RADIUS — distance from centre at which ray trails
 * stop drawing. The '@' core is at radius 0; we leave a 1-cell
 * guard so no trail can overwrite it. ray_draw breaks the inner
 * loop as soon as ri < this.
 */
#define CORE_RESERVED_RADIUS  1.0f

/* ── §1.3 ray motion (cells/sec — physical units) ─────────────────── */
/*
 * Each ray's speed is uniform-sampled in [SPEED_MIN_CPS, SPEED_MAX_CPS]
 * at spawn. Rays at different speeds produce the natural-looking
 * variance — fast rays sprint to the rim while slow ones still hug
 * the core.
 */
#define SPEED_MIN_CPS  30.0f
#define SPEED_MAX_CPS  80.0f

/*
 * STAGGER_FRAC — at spawn, each ray's r_off is uniform in
 * [-STAGGER_FRAC · max_r, 0]. Negative = "pre-emerged"; the ray's
 * tail is below the core for stagger/speed seconds before the head
 * actually crosses the core. With 0.55 the screen fills smoothly
 * over the first ~1-2 seconds rather than popping fully populated.
 */
#define STAGGER_FRAC  0.55f

/* ── §1.4 shimmer ─────────────────────────────────────────────────── */
/* Per-frame, each glyph has a 1-in-KEEP_ONE_IN chance of surviving;
 * the rest reroll. KEEP_ONE_IN = 4 → 75 % rerolled per frame. */
#define SHIMMER_KEEP_ONE_IN  4

/* ── §1.5 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..5 — trail bands (theme-controlled) */
    SHADE_FADE     = 1,
    SHADE_DARK,
    SHADE_MID,
    SHADE_BRIGHT,
    SHADE_HOT,

    /* 6 — ray head, always white (theme-independent) */
    SHADE_HEAD,

    /* 7 — sun core '@', always white */
    SHADE_CORE,

    /* 8..9 — HUD spec, theme-independent */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.6 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.7 timing primitives ───────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.8 HUD ─────────────────────────────────────────────────────── */
#define HUD_BUF_LEN  72

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

/*
 * Theme — one named 5-colour brightness palette for the ray trail.
 *
 * Intent
 *   Each ray draws a HEAD cell + RAY_TRAIL-1 trailing cells along
 *   its frozen radial direction.  As the trail index `i` increases
 *   (0 = head, larger i = farther back, dimmer), the cell renders
 *   in a dimmer brightness tier.  The mapping (i → tier) happens in
 *   §4 dist_attr; this struct holds the per-theme fg colour for each
 *   tier (SHADE_FADE..SHADE_HOT, 5 tiers total).
 *
 *   The `t` key cycles through THEME_COUNT presets — theme_apply()
 *   re-registers all the colour pairs in one O(SHADE_COUNT) call.
 *   Each ray's PAIR ID stays the same across theme switches; only
 *   the pair's fg colour changes, so existing rays instantly adopt
 *   the new look without re-spawn.
 *
 * Why two parallel palettes (fg[] vs fg_8[])
 *   256-colour terminals can use a fine-grained brightness ramp
 *   (5 distinct cube indices per theme); 8-colour terminals can't —
 *   fg_8[] picks the closest ANSI primary, accepting that adjacent
 *   shades collapse into the same equivalence class.
 *
 * Why HEAD + CORE are theme-INDEPENDENT
 *   SHADE_HEAD (the bright leading cell of each ray) and SHADE_CORE
 *   (the central '@' glyph at cx, cy) are always WHITE regardless
 *   of theme.  Theme-tinting them would wash out the contrast
 *   against the trail tail and make the sun's core dissolve into
 *   the radiating rays.  Hardcoded white keeps both legible across
 *   every theme.
 *
 * Themes (preset table)
 *   solar   — amber / gold (the eponymous look)
 *   green   — Matrix green
 *   nova    — cool blue → white burst
 *   plasma  — purple / magenta corona
 *   fire    — red / orange flame
 *
 * Members
 *   name    HUD label ("solar", "green", …) shown in the status row.
 *   fg[]    5 xterm-256 fg indices ordered SHADE_FADE..SHADE_HOT
 *           (dimmest → brightest).  Indexed by the brightness tier
 *           that dist_attr maps each trail position to.
 *   fg_8[]  5 ANSI-8 fallback indices, same ordering.
 *
 * Invariants
 *   Every fg[i] ∈ [24, 255] per CLAUDE.md "Theme Palette Brightness"
 *   (cube 16–23 and gray 232–239 render as black on default-black
 *   terminals, so they'd make the trail tail invisible).  Values
 *   24–29 used only as the LOWEST ramp tier (oldest trail cells).
 *   fg[i] perceptually ≤ fg[i+1] (monotone bright→dim along array).
 *   name != NULL.
 *
 * References
 *   CLAUDE.md §"Theme Palette Brightness" for the cube-floor rule.
 *   Same theme-cycling design as pulsar_rain.c / matrix_rain.c —
 *   a reader who knows one knows the rest.
 */
typedef struct {
    const char *name;
    int         fg  [5];
    int         fg_8[5];
} Theme;

static const Theme k_themes[] = {
    { "solar",
      { 130, 166, 202, 214, 220 },
      { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  } },
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   } },
    { "nova",
      {  24,  33,  51, 159, 255 },
      { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE   } },
    { "plasma",
      {  53,  57,  93, 129, 201 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "fire",
      {  52,  88, 124, 160, 196 },
      { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — bind wake bands and head/core for the chosen theme.
 * PAIR_HUD and PAIR_HINT are NEVER touched here — they carry semantic
 * meaning that must not change with theme.
 */
static void theme_apply(int idx)
{
    const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
    init_pair(SHADE_CORE,   COLOR_WHITE, COLOR_BLACK);
}

/*
 * hud_pairs_init — bind PAIR_HUD and PAIR_HINT once at startup. Both
 * use the default terminal background (-1) so the HUD sits on the
 * user's real background instead of a forced black box.
 */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

/*
 * dist_attr — map distance from ray head to ncurses attr.
 *
 *   i=0           HEAD     white     BOLD
 *   i=1           HOT      theme[4]  BOLD
 *   i=2           BRIGHT   theme[3]  BOLD
 *   i=3..N/2      MID      theme[2]  NORMAL
 *   i=N/2+1..N-2  DARK     theme[1]  NORMAL
 *   i=N-1..       FADE     theme[0]  DIM
 */
static attr_t dist_attr(int i)
{
    if (i == 0)              return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (i == 1)              return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (i <= TRAIL_HOT_END)  return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (i <= TRAIL_WARM_END) return COLOR_PAIR(SHADE_MID);
    if (i <= RAY_TRAIL - 2)  return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* ===================================================================== */
/* §4  scene — Ray + RayField + SunGeometry + SimControls; tick + draw    */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utility ─────────────────────────── */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01   (void) { return (float)rand() / (float)RAND_MAX; }

/* ── §4.2 Ray type ────────────────────────────────────────────────── */

/*
 * Ray — one outward-radiating stream from the sun's core.
 *
 * Intent
 *   The atomic unit of the sun rendering: a single radial line from
 *   (cx, cy) outward along a frozen direction.  Each Ray is a 1-D
 *   Reeves [3] particle constrained to its assigned direction —
 *   only the head DISTANCE FROM CENTRE evolves over time; the
 *   direction is BAKED at spawn into (cos_a, sin_a·ASPECT) and never
 *   changes for the ray's lifetime.
 *
 *   Lifecycle:
 *     1. SPAWN    — ray_init sets the direction (cos_a, sin_a)
 *                   from the assigned angle.  ray_reset randomises
 *                   r_off ∈ [−STAGGER_FRAC·max_r, 0] so the visible
 *                   emergence is staggered (rays don't all appear
 *                   from the centre at the same instant).
 *     2. FLY      — ray_tick advances r_off += speed · dt.
 *                   ~75 % of cache[] slots reroll per tick (the
 *                   matrix shimmer applied along the radial trail).
 *     3. RECYCLE  — when the TAIL TIP (r_off − RAY_TRAIL) clears
 *                   max_r, ray_tick returns false and the caller
 *                   (scene_tick) immediately ray_resets it.  Direction
 *                   is preserved across recycle; only r_off, speed,
 *                   and cache[] are re-randomised.
 *
 *   Different rays fly at different speeds (re-randomised at every
 *   recycle), so the radial fan looks organic instead of pulsing
 *   in unison.
 *
 * Why pre-baked (cos_a, sin_a) instead of recomputing each frame
 *   ray_draw paints RAY_TRAIL cells per ray per frame.  At N_RAYS =
 *   180 and 60 fps that's ~170 k radial positions/sec.  Doing
 *   cos/sin in the inner loop would cost ~340 k trig calls/sec; one
 *   trig pair at spawn time costs 360/recycle (~6/sec under defaults).
 *   The cache-baked approach makes the inner loop ZERO TRIG.
 *
 * Why ASPECT baked into sin_a (not multiplied at draw time)
 *   Terminal cells are ~1 col × 2 rows.  Without correction, sin(θ)·r
 *   gives a y-offset in CELL units that looks elliptical instead of
 *   circular.  Baking ASPECT into sin_a means the draw loop's
 *   `row = cy + roundf(ri * sin_a)` does the correction for free —
 *   no per-cell multiplication.  See CLAUDE.md §"Coordinate /
 *   Physics" + [5].
 *
 * Why constant speed (not Parker's accelerating wind)
 *   Real solar-wind plasma accelerates with altitude (Parker [1]).
 *   We use CONSTANT speed because:
 *     • Visual: at terminal-cell resolution the acceleration over a
 *       single ray's lifetime is sub-cell, invisible.
 *     • Simplicity: makes ray_tick a one-line `r_off += speed · dt`
 *       instead of an integrator.  Stability is also trivial — no
 *       stiff terms, variable-dt is fine ([8] Fiedler).
 *
 * Members
 *   ── direction (set ONCE at spawn, never mutated) ────────────────
 *   cos_a    Pre-baked cos(θ); horizontal direction component.
 *   sin_a    Pre-baked sin(θ) · ASPECT; vertical direction with
 *            cell-aspect correction baked in.
 *
 *   ── per-tick mutable state ──────────────────────────────────────
 *   r_off    Head distance from centre (cells, float).
 *              < 0 → pre-emergence (stagger delay; not yet visible).
 *              ≥ 1 → head visible at distance r_off.
 *            Advances by `speed · dt` each tick.
 *   speed    Cells per second.  Constant for one ray's lifetime,
 *            re-randomised at every ray_reset to give the fan
 *            organic variation.
 *   cache[]  Per-ray random ASCII glyphs.  cache[0] is the HEAD
 *            (drawn brightest); cache[1..RAY_TRAIL-1] is the tail.
 *            ~75 % reroll per frame produces the Matrix shimmer.
 *
 * Invariants
 *   cos_a² + (sin_a/ASPECT)² == 1  (unit vector pre-aspect-correction).
 *   speed > 0.
 *
 * Memory footprint
 *   4 floats + RAY_TRAIL chars = 32 bytes (RAY_TRAIL = 16).
 *   N_RAYS × Ray = 180 × 32 = 5760 bytes — comfortably in BSS,
 *   no malloc anywhere in this file.
 *
 * References
 *   [1] Parker 1958 — solar-wind paper; rays are 1-D particles in
 *       Parker's radial-outflow geometry (without the acceleration).
 *   [3] Reeves 1983 — particle-systems lifecycle (spawn / fly /
 *       expire-recycle).
 *   [5] Cell-aspect correction (CLAUDE.md + ray_init inline note).
 *   [6] Wachowski film — the shimmer cache visual heritage.
 */
typedef struct {
    float cos_a, sin_a;
    float r_off;
    float speed;
    char  cache[RAY_TRAIL];
} Ray;

/* ── §4.3 ray_init / ray_reset ────────────────────────────────────── */

/*
 * ray_reset — re-stagger an already-initialised ray. Called when the
 * tail clears max_r so the ray can "loop" back to a fresh start.
 * Direction (cos_a, sin_a) is left untouched — only r_off, speed,
 * and the glyph cache are randomised.
 */
static void ray_reset(Ray *r, float max_r)
{
    r->r_off = -urand01() * STAGGER_FRAC * max_r;
    r->speed = SPEED_MIN_CPS + urand01() * (SPEED_MAX_CPS - SPEED_MIN_CPS);
    for (int i = 0; i < RAY_TRAIL; i++)
        r->cache[i] = rand_glyph();
}

/*
 * ray_init — set this ray's direction (cos_a, sin_a baked from
 * angle), then call ray_reset for fresh r_off/speed/cache.
 */
static void ray_init(Ray *r, float angle, float max_r)
{
    r->cos_a = cosf(angle);
    r->sin_a = sinf(angle) * ASPECT;       /* ASPECT BAKED IN */
    ray_reset(r, max_r);
}

/* ── §4.4 ray_tick — advance head, shimmer cache ─────────────────── */

/*
 * One per-frame physics step. Returns true while the ray is still
 * partially on screen, false once even the tail tip has cleared
 * max_r (caller respawns via ray_reset).
 */
/* shimmer_reroll_ray_cache — per-tick reroll.  Each cache slot has a
 * (SHIMMER_KEEP_ONE_IN − 1)/SHIMMER_KEEP_ONE_IN probability of being
 * re-rolled to a fresh random glyph (KEEP_ONE_IN = 4 → ~75 %
 * rerolled per tick).  Slots that survive look "still" as the ray
 * flies outward; rerolled slots produce the matrix shimmer along
 * the radial trail. */
static inline void shimmer_reroll_ray_cache(Ray *r) {
    for (int i = 0; i < RAY_TRAIL; i++)
        if (rand() % SHIMMER_KEEP_ONE_IN != 0)
            r->cache[i] = rand_glyph();
}

/* tail_tip_inside_recycle_horizon — true while the FARTHEST-BACK
 * trail cell (head minus full RAY_TRAIL length) is still inside the
 * recycle horizon at max_r.  Once the tail tip clears max_r, the
 * caller respawns the ray with a fresh stagger. */
static inline bool tail_tip_inside_recycle_horizon(const Ray *r, float max_r) {
    float tail_tip_distance = r->r_off - (float)RAY_TRAIL;
    return tail_tip_distance < max_r;
}

static bool ray_tick(Ray *r, float dt, float max_r)
{
    r->r_off += r->speed * dt;             /* advance head outward */
    shimmer_reroll_ray_cache(r);            /* per-tick matrix shimmer */
    return tail_tip_inside_recycle_horizon(r, max_r);
}

/* ── §4.5 ray_draw — paint head + trail ──────────────────────────── */

/* trail_cell_radius_from_head — radius of trail slot `i`.
 * Slot 0 is the head (at r_off); slot i sits i cells closer to the
 * centre (r_off − i).  Once this radius drops below the core
 * reservation, the caller stops painting to leave the central '@'
 * legible. */
static inline float trail_cell_radius_from_head(const Ray *r, int slot_i) {
    return r->r_off - (float)slot_i;
}

/* polar_to_screen_cell — map (radius, baked direction) to a discrete
 * screen cell.  Direction is (cos_a, sin_a·ASPECT) — sin_a already
 * has the cell-aspect correction baked in at ray_init, so the
 * inner loop is two multiplies + two adds + two rounds, ZERO trig.
 * Bresenham [4] would do this with integer DDA; we use float roundf
 * because each polar point is INDEPENDENT (no constant-line
 * assumption — different `i` slots can hit non-collinear cells in
 * general, even though for a single ray they're collinear in
 * isotropic space). */
static inline void polar_to_screen_cell(int cx, int cy,
                                         float radius,
                                         float cos_a, float sin_a,
                                         int *out_col, int *out_row) {
    *out_col = cx + (int)roundf(radius * cos_a);
    *out_row = cy + (int)roundf(radius * sin_a);
}

/* paint_ray_trail_cell — paint ONE trail cell with the correct attr,
 * dropping it silently if it falls off-screen. */
static inline void paint_ray_trail_cell(const Ray *r, int slot_i,
                                         int col, int row,
                                         int cols, int rows) {
    if (col < 0 || col >= cols || row < 0 || row >= rows) return;
    attr_t attr = dist_attr(slot_i);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)r->cache[slot_i]);
    attroff(attr);
}

/*
 * ray_draw — paint one ray's head + trail.
 *
 *   Walk slot_i = 0..RAY_TRAIL-1.  Slot 0 is the head (brightest,
 *   at radius r_off); each successive slot sits 1 cell closer to
 *   the centre (radius r_off − slot_i).  Stop painting once the
 *   radius drops below CORE_RESERVED_RADIUS — this preserves the
 *   central '@' core from being stamped over by trail glyphs.
 *
 *   Inner loop: zero trig (cos_a/sin_a were baked at ray_init).
 *   Two multiplies + two rounds + two adds + bounds check + paint.
 */
static void ray_draw(const Ray *r, int cx, int cy, int cols, int rows)
{
    for (int slot_i = 0; slot_i < RAY_TRAIL; slot_i++) {
        float radius = trail_cell_radius_from_head(r, slot_i);
        if (radius < CORE_RESERVED_RADIUS) break;   /* leave the '@' core alone */

        int col, row;
        polar_to_screen_cell(cx, cy, radius, r->cos_a, r->sin_a, &col, &row);
        paint_ray_trail_cell(r, slot_i, col, row, cols, rows);
    }
}

/* ── §4.6 Scene sub-structs ───────────────────────────────────────── */

/*
 * RayField — the fixed-capacity array of N_RAYS outward beams.
 *
 * Intent
 *   N_RAYS rays evenly spaced around the unit circle at scene_init
 *   time.  After init, each ray's direction (cos_a, sin_a·ASPECT) is
 *   FROZEN; only (r_off, speed, cache[]) mutate per tick.  Bundling
 *   the array into a named type makes the symmetry with other
 *   project demos explicit (StreamPool in matrix_snowflake.c has the
 *   same "one beam per slot, never resized" shape).
 *
 * Members
 *   rays[N_RAYS]   Inline array; every slot is populated by
 *                  scene_init.  Index is the ray number; the angle
 *                  for ray i is (i / N_RAYS) · 2π.
 *
 * Invariants
 *   For every i ∈ [0, N_RAYS): rays[i] is in a valid state
 *   (cos_a + sin_a baked from its assigned angle).
 */
typedef struct {
    Ray  rays[N_RAYS];
} RayField;

/*
 * SunGeometry — the radial recycle geometry, derived from screen
 * extent at scene_init time.
 *
 * Intent
 *   Each ray flies outward along its direction vector until its
 *   tail clears `max_r`, then ray_reset re-staggers it inside [0,
 *   STAGGER_FRAC·max_r] and it starts again.  cx/cy are the visible
 *   sun centre (= cols/2, rows/2 at init).  max_r is sized to a
 *   conservative upper bound so even the longest diagonal trail
 *   fully exits before recycle.
 *
 *   Recomputed in scene_init (and again in app_do_resize) from the
 *   current Screen extent.  Frozen for the duration of one screen size.
 *
 * Why max_r = cols + rows/ASPECT (not √(half² + half²))
 *   That formula is a deliberate OVER-estimate.  The tight bound
 *   is √((cols/2)² + (rows/2/ASPECT)²) for the diagonal corner, but
 *   we pay for the loose bound only in extra ticks-per-recycle
 *   (microseconds) and gain a guarantee that trails fully exit
 *   regardless of (cos_a, sin_a) magnitude.
 *
 * Members
 *   cx, cy   Screen-cell coordinates of the sun centre
 *            (= cols/2, rows/2 at init).
 *   max_r    Upper-bound recycle distance.  When a ray's tail tip
 *            (r_off − RAY_TRAIL) exceeds max_r, the ray respawns.
 *
 * Invariants
 *   max_r > 0 after init.
 *   cx ∈ [0, screen.cols), cy ∈ [0, screen.rows) at init time.
 *
 * References
 *   The ASPECT correction trick is project-wide — see CLAUDE.md
 *   §"Coordinate / Physics" and the inline note in ray_init.
 */
typedef struct {
    int   cx, cy;
    float max_r;
} SunGeometry;

/*
 * SimControls — user-facing playback knob.
 *
 * Intent
 *   This demo has ONE pure playback knob (paused).  Other user-
 *   controllable parameters live on Scene directly (theme_idx)
 *   because they're tied to specific semantic domains.
 *
 * Members
 *   paused   true → scene_tick early-returns; all rays freeze in
 *            place but rendering continues (HUD stays live).
 *            Toggled by SPACE/p.
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * Scene — owns ALL simulation + render state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── field      : RayField      ← rays[N_RAYS] (radial fan)
 *       ├── geom       : SunGeometry   ← cx/cy + max_r
 *       ├── sim        : SimControls   ← paused
 *       └── theme_idx  : int           ← active colour palette
 *
 *   Every persistent value reachable from one `Scene *s`.  Helpers
 *   that touch only one sub-domain take just that sub-struct (e.g.
 *   ray_tick takes `Ray *` + `max_r` from geom), keeping the
 *   "what does this function touch" question answerable from the
 *   signature.
 *
 * Why no World sub-struct (unlike matrix_snowflake.c)
 *   Sun geometry is centre + radii, NOT a cell grid.  The Screen
 *   extent is read ONCE in scene_init to compute (cx, cy, max_r) —
 *   after that, the sim doesn't need cols/rows; only the renderer's
 *   clip check does, and it gets them from app->screen via
 *   scene_draw's parameters.  Same pattern as pulsar_rain.c.
 */
typedef struct {
    RayField    field;
    SunGeometry geom;
    SimControls sim;
    int         theme_idx;
} Scene;

/* ── §4.7 compute_sun_geometry — derive (cx, cy, max_r) from screen ── */

/*
 * compute_sun_geometry — derive the radial recycle geometry from the
 * screen extent.
 *
 *   centre   (cols/2, rows/2)
 *   max_r    cols + rows / ASPECT  (conservative upper bound so
 *            trails fully exit before recycle; see SunGeometry doc
 *            for why we use this loose bound instead of the tight
 *            corner distance)
 */
static void compute_sun_geometry(SunGeometry *geom, int cols, int rows) {
    geom->cx    = cols / 2;
    geom->cy    = rows / 2;
    geom->max_r = (float)cols + (float)rows / ASPECT;
}

/* ── §4.8 scene_init / scene_reset ────────────────────────────────── */

/*
 * spawn_rays_evenly_spaced — seed RayField with N_RAYS rays whose
 * directions are evenly distributed around the unit circle:
 *
 *     angle_i = (i / N_RAYS) · 2π   for i ∈ [0, N_RAYS)
 *
 * ray_init bakes (cos_a, sin_a·ASPECT) at this angle, so each ray's
 * direction is frozen for the rest of its life.
 */
static void spawn_rays_evenly_spaced(RayField *field, float max_r) {
    const float ANGLE_STEP_PER_RAY = 2.0f * (float)M_PI / (float)N_RAYS;
    for (int i = 0; i < N_RAYS; i++) {
        float angle = (float)i * ANGLE_STEP_PER_RAY;
        ray_init(&field->rays[i], angle, max_r);
    }
}

/*
 * scene_init — full reset: geometry + ray placement + UI defaults.
 * Called at startup AND on resize (app_do_resize preserves user
 * knobs across the call).
 */
static void scene_init(Scene *s, int cols, int rows)
{
    /* (1) SunGeometry — derived from screen extent */
    compute_sun_geometry(&s->geom, cols, rows);

    /* (2) RayField — N_RAYS rays evenly spaced around the circle */
    spawn_rays_evenly_spaced(&s->field, s->geom.max_r);

    /* (3) SimControls + theme defaults */
    s->sim.paused = false;
    s->theme_idx  = 0;
}

/* scene_reset — re-stagger every ray without recomputing angles or
 * geometry.  Used by the 'r' key to bloom a fresh sun.  Each ray's
 * direction (cos_a, sin_a) is preserved; only (r_off, speed, cache[])
 * are re-randomised. */
static void scene_reset(Scene *s)
{
    for (int i = 0; i < N_RAYS; i++)
        ray_reset(&s->field.rays[i], s->geom.max_r);
}

/* ── §4.9 scene_tick — advance every ray ──────────────────────────── */

/*
 * advance_one_ray_with_recycle — advance a single ray; if its tail
 * has cleared the screen, respawn it with a fresh stagger and speed
 * (preserving its angle). */
static inline void advance_one_ray_with_recycle(Ray *r, float dt, float max_r) {
    bool still_on_screen = ray_tick(r, dt, max_r);
    if (!still_on_screen)
        ray_reset(r, max_r);
}

/*
 * scene_tick — advance every ray by one frame.
 *
 *   When paused, returns early — rays freeze in place but the
 *   screen continues to redraw, so the HUD stays live.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;

    for (int i = 0; i < N_RAYS; i++)
        advance_one_ray_with_recycle(&s->field.rays[i], dt, s->geom.max_r);
}

/* ── §4.10 sun_draw_core — '@' on top of everything ───────────────── */

static void sun_draw_core(const Scene *s, int cols, int rows)
{
    int cx = s->geom.cx, cy = s->geom.cy;
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    attron(COLOR_PAIR(SHADE_CORE) | A_BOLD);
    mvaddch(cy, cx, '@');
    attroff(COLOR_PAIR(SHADE_CORE) | A_BOLD);
}

/* ── §4.11 scene_draw — orchestrator: N rays + core last ──────────── */

static void scene_draw(const Scene *s, int cols, int rows)
{
    /* Pass 1 — every ray's head + trail. */
    for (int i = 0; i < N_RAYS; i++)
        ray_draw(&s->field.rays[i], s->geom.cx, s->geom.cy, cols, rows);

    /* Pass 2 — core LAST, always wins overlap. */
    sun_draw_core(s, cols, rows);
}

/* ── §4.12 input helpers (used by app_handle_key) ─────────────────── */

static void scene_cycle_theme(Scene *s)
{
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/* ===================================================================== */
/* §5  screen — ncurses init / present / HUD                              */
/* ===================================================================== */

/*
 * Screen — terminal cell extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Tracks the TERMINAL side of the demo: cell dimensions for HUD
 *   placement and field clipping.  ncurses owns the back buffer;
 *   this struct is the source-of-truth for cell-space dimensions,
 *   refreshed via getmaxyx in screen_init / screen_resize.
 *
 *   Scene.geom.{cx, cy, max_r} ARE DERIVED from these dimensions
 *   (see compute_sun_geometry in §4.7).  After init, the sim
 *   doesn't need cols/rows anymore — only the renderer does, for
 *   its clip-to-screen check.  Cols/rows flow into scene_draw and
 *   ray_draw as explicit parameters.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_draw_hud touch ncurses' initscr / endwin /
 *     mvprintw.  They all take `Screen *` to make this layer
 *     explicit at the type level.
 *   • Symmetry with every other demo in this project — `{cols, rows}`
 *     is the canonical Screen shape.
 *
 * Members
 *   cols   Terminal width in CELLS (getmaxyx).
 *   rows   Terminal height in CELLS.  Bottom row index is rows − 1
 *          (where the key hint paints).  Row 0 is the status row.
 *
 * Invariants
 *   cols > 0, rows > 0 after screen_init.
 *   Both refreshed on every SIGWINCH via screen_resize +
 *   app_do_resize (which also re-runs compute_sun_geometry at the
 *   new extent).
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    start_color();
    use_default_colors();       /* lets HUD pairs use -1 background       */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* hud_paint_text — attron / mvprintw / attroff sandwich shared by
 * both HUD rows.  Centralises the colour-pair setup. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* format_hud_status — write the top-row status string into `buf`.
 * Reports fps, ray count, active theme, paused state. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    snprintf(buf, buflen, " %5.1f fps  rays:%d  [%s] %s ",
             fps, N_RAYS, k_themes[s->theme_idx].name,
             s->sim.paused ? "PAUSED " : "running");
}

/* draw_hud_status — right-aligned status row 0 (PAIR_HUD yellow). */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_BUF_LEN];
    format_hud_status(s, fps, buf, sizeof buf);
    int right_col = sc->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan). */
static void draw_hud_hint(const Screen *sc) {
    static const char *KEY_HINT =
        " q:quit  spc:pause  r:reset  t:theme ";
    hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/*
 * screen_draw_hud — required HUD per CLAUDE.md spec.
 *
 *   Row 0       PAIR_HUD  + A_BOLD  (yellow) — fps + state + theme
 *   Bottom row  PAIR_HINT + A_BOLD  (cyan)   — full key list
 *
 * Both pairs sit on default background (-1) so they stay legible
 * regardless of theme. theme_apply() never touches them.
 */
static void screen_draw_hud(const Screen *sc, double fps, const Scene *s)
{
    draw_hud_status(sc, s, fps);
    draw_hud_hint  (sc);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §6  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Per-frame fps would jitter; this accumulates frame_count + elapsed
 * nanoseconds over a 500 ms window and emits a smoothed `display`
 * value each time the window fills.  Same shape as the FpsCounter on
 * every other file in this project.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 *   scene         simulation + render state (RayField + SunGeometry
 *                  + SimControls + theme_idx)
 *   screen        terminal cell extent + ncurses lifecycle
 *   fps           rolling-window fps estimator for HUD
 *   running       sig_atomic_t flag cleared by SIGINT / SIGTERM + 'q'
 *   need_resize   sig_atomic_t flag set by SIGWINCH; main reacts at
 *                 the top of the next iteration
 *
 * `g_app` is the program's only file-scope mutable state.  Signal
 * handlers reach the flags through it; everything else flows via
 * `App *app` parameter.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * scene_reinit_preserving_knobs — re-run scene_init at a new extent,
 * preserving the user-tuned knobs that SHOULD survive a re-init
 * (theme_idx, paused).
 *
 * Used by app_do_resize (SIGWINCH: new screen size, same knobs).
 * scene_reset (the 'r' key) deliberately does NOT use this — the
 * user wants ray re-stagger only, not geometry recomputation.
 */
static void scene_reinit_preserving_knobs(Scene *s, int new_cols, int new_rows) {
    /* (1) snapshot user-tuned knobs */
    int  saved_theme  = s->theme_idx;
    bool saved_paused = s->sim.paused;

    /* (2) full re-init (resets EVERYTHING including knobs) */
    scene_init(s, new_cols, new_rows);

    /* (3) restore preserved knobs */
    s->theme_idx  = saved_theme;
    s->sim.paused = saved_paused;
}

/*
 * app_do_resize — handle a pending SIGWINCH.
 *
 *   (1) refresh the Screen extent (ncurses getmaxyx)
 *   (2) re-init the Scene at the new extent, preserving user knobs
 *   (3) clear the resize flag so main resumes normal frames
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);                                 /* (1) */
    scene_reinit_preserving_knobs(&app->scene,                   /* (2) */
                                   app->screen.cols,
                                   app->screen.rows);
    app->need_resize = 0;                                        /* (3) */
}

/*
 * app_handle_key — process one keystroke.  Returns false on quit.
 *
 *   q / Q / ESC      quit
 *   SPACE / p / P    toggle paused
 *   r / R            reset (re-stagger every ray; keep theme + paused)
 *   t / T            cycle theme
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;   break;
    case 'r': case 'R':           scene_reset(s);                   break;
    case 't': case 'T':           scene_cycle_theme(s);             break;

    default: break;
    }
    return true;
}

/*
 * main — variable-dt render loop.
 *
 *   (1) RNG + atexit + signal handlers
 *   (2) bring up Screen + App; init Scene at current terminal size
 *   (3) loop:
 *       (a) handle pending SIGWINCH
 *       (b) measure dt (capped to prevent spiral-of-death)
 *       (c) drain non-blocking input
 *       (d) advance every ray (scene_tick)
 *       (e) rolling-window fps counter
 *       (f) draw + present
 *       (g) frame cap: sleep so we don't burn the next slot's budget
 */
int main(void)
{
    /* (1) RNG + atexit + signal handlers */
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* (2) Bring up Screen + App; init Scene */
    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    scene_init(scene, app->screen.cols, app->screen.rows);
    theme_apply(scene->theme_idx);
    hud_pairs_init();

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* (3a) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* (3b) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3c) drain input */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (3d) advance the scene (every ray) */
        scene_tick(scene, dt);

        /* (3e) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt_ns);

        /* (3f) draw + present */
        erase();
        scene_draw(scene, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, app->fps.display, scene);
        screen_present();

        /* (3g) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
