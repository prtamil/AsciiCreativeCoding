/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_pendulum.c
 *   — A damped pendulum hangs above three equal magnets arranged at
 *     the vertices of an equilateral triangle.  The bob is attracted
 *     by each magnet with a softened 1/r² force.  Starting from any
 *     position with zero velocity, the bob ALWAYS eventually settles
 *     on ONE of the three magnets — but WHICH magnet wins is a
 *     chaotic function of the starting position.  The set of starting
 *     points that all converge to the same magnet is that magnet's
 *     "basin of attraction"; the BOUNDARY between basins is a
 *     FRACTAL when damping is light.
 *
 * DEMO: SIDE-VIEW animation of the apparatus.  A fixed pivot '+' at
 *       top-centre of the screen; a faint rod sweeps from the pivot
 *       to a single bob '@' swinging chaotically over the three
 *       magnets '[1]' '[2]' '[3]' (numbered for identification).  A
 *       long fading trail records the recent swing path.  Once the
 *       bob is absorbed by a magnet it briefly holds, then auto-
 *       respawns from a fresh random release point — the animation
 *       never stops.
 *
 *       2 presets (n/p):
 *         1 "WEAK"   — γ = 0.05  → long chaotic swings, sensitive
 *                                   dependence on release point
 *                                   (FRACTAL-boundary regime)
 *         2 "STRONG" — γ = 0.30  → bob barely overshoots once and
 *                                   drops to the nearest magnet
 *                                   (clean Voronoi regime)
 *
 * Study alongside:
 *   ../fractals/newton_fractal.c — basins of attraction for Newton's
 *                                  method (same idea, different system).
 *   ./double_pendulum.c          — same physics intuition (damped,
 *                                  driven, gravity) without magnets.
 *
 * Section map:
 *   §1  config   — constants, presets, themes
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — HUD pairs + tier-based theme apply
 *   §5  physics  — Magnet + MagnetField + PendulumSystem + BobState
 *                  + Pendulum (composite) + named-stage RK4
 *   §6  trail    — ring buffer of recent bob positions + age→tier helper
 *   §7  orbit    — ActiveOrbit (single trajectory + bookkeeping) +
 *                  PresetState + PaletteState typed wrappers
 *   §8  scene    — Scene composes orbit + preset + palette + dims
 *   §9  screen   — Viewport, layered painters (anchor → trail → rod
 *                  → bob → magnets), HUD writers
 *   §10 app     — signals, resize, key dispatch table, FrameClock,
 *                  main pseudocode driver + named loop helpers
 *
 * Keys: q/ESC quit | space pause | r:reseed | n/p:preset
 *       t/T:theme | ]/[:Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/magnetic_pendulum.c \
 *       -o magnetic_pendulum -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Damped pendulum bob in 2-D plane.  Three fixed
 *                 magnets at unit-circle vertices (angles 90°, 210°,
 *                 330°).  Three-component force law:
 *
 *                   F = −k·r                        ← spring to origin
 *                       − γ·v                        ← viscous damping
 *                       − Σᵢ (r − mᵢ) / (|r − mᵢ|² + ε²)^{3/2}
 *                                                    ← softened 1/r²
 *                                                       attractions
 *
 *                 RK4 at fixed SIM_DT, LIVE_BOB_SUBSTEPS_PER_TICK
 *                 substeps per render frame.  The bob is "absorbed"
 *                 when |v|² < CONV_V_SQ AND it is within CONV_R of
 *                 a magnet — at which point the orbit holds for
 *                 LIVE_BOB_RESPAWN_FRAMES, then a fresh random
 *                 release point starts a new orbit.
 *
 * Data-struct   : §5 Pendulum splits into PendulumSystem (k, γ, ε,
 *                 MagnetField) and BobState (x, y, vx, vy ODE coords).
 *                 §6 Trail is a LIVE_TRAIL_MAX ring buffer of recent
 *                 bob positions, drawn as a fading sweep.
 *                 §7 ActiveOrbit bundles Pendulum + Trail + per-orbit
 *                 bookkeeping (step counter, winner, respawn timer,
 *                 seed point).
 *
 * Rendering     : ASCII-only side-view.  Layered back-to-front:
 *                   ANCHOR   — '+' at top-centre pivot (fixed)
 *                   TRAIL    — 4-tier glyph + colour fade (oldest →
 *                              newest: ','  '.'  ':'  '+')
 *                   ROD      — Bresenham line from pivot to bob, with
 *                              direction-aware glyph at each cell
 *                   BOB      — '@' (moving) or 'O' (settled), bright
 *                   MAGNETS  — '[1]' '[2]' '[3]' bracketed markers
 *                              in brightest tier, drawn LAST so the
 *                              bob visibly merges INTO one on absorb.
 *                 All apparatus elements share one accent hue cycled
 *                 by t/T (theme); brightness tier conveys the role.
 *
 * References    : THE PROBLEM  (where the equation comes from)
 *                 [1] Strogatz, S. H. (2015) — "Nonlinear Dynamics and
 *                     Chaos" (2nd ed.), Westview Press, §7.2.  The
 *                     pedagogical entry point: defines basins of
 *                     attraction in 2-D dissipative systems and walks
 *                     through how multiple stable fixed points partition
 *                     phase space.
 *                 [2] Moon, F. C. (1992) — "Chaotic and Fractal Dynamics",
 *                     Wiley, Ch. 4 §4.7.  The magnetic pendulum as
 *                     the classic experimental demonstration of
 *                     fractal basins.  Includes photographs of physical
 *                     apparatus and the predicted basin maps.
 *                 [3] Aguirre, J., Sanjuán, M. A. F. (2003) —
 *                     "Limit of small exponents in the magnetic
 *                     pendulum's basin entropy", Physical Review E
 *                     67(5).  Quantitative study of THIS specific
 *                     system (3 magnets, spring + damping, 1/r²
 *                     attractions) and its fractal-boundary entropy.
 *
 *                 FRACTAL BASIN BOUNDARIES  (the underlying chaos)
 *                 [4] Grebogi, C., Ott, E., Yorke, J. A. (1983) —
 *                     "Fractal Basin Boundaries", Physica D 7.
 *                     The seminal paper introducing the concept that
 *                     basin boundaries can themselves be fractal —
 *                     directly relevant to the WEAK_DAMP preset.
 *                 [5] Ott, E. (2002) — "Chaos in Dynamical Systems"
 *                     (2nd ed.), Cambridge University Press, Ch. 5.
 *                     Pedagogical chapter on basin structures, the
 *                     transition to fractal boundaries as a system
 *                     parameter (here: γ damping) changes.
 *
 *                 NUMERICAL INTEGRATION
 *                 [6] Press, W. H., Teukolsky, S. A., Vetterling, W. T.,
 *                     Flannery, B. P. (2007) — "Numerical Recipes"
 *                     (3rd ed.), Cambridge, Ch. 17.1.  RK4 derivation,
 *                     stability, and the Butcher tableau implemented
 *                     in pendulum_rk4 / rk4_butcher_weighted_average.
 *                 [7] Fiedler, G. (2004) — "Fix Your Timestep!",
 *                     gafferongames.com.  Accumulator pattern in
 *                     app_drain_fixed_timestep — keeps the orbit
 *                     deterministic across frame rates.
 *
 *                 SIMULATION ARCHITECTURE & RENDERING
 *                 [8] Sussman, G. J., Wisdom, J. (2014) — "Structure
 *                     and Interpretation of Classical Mechanics"
 *                     (2nd ed.), MIT Press.  The "system / state /
 *                     observable" architecture pattern that Scene
 *                     mirrors at the code level (PendulumSystem,
 *                     BobState, Trail).
 *                 [9] Bresenham, J. E. (1965) — "Algorithm for
 *                     computer control of a digital plotter", IBM
 *                     Systems Journal 4(1).  The integer-only line-
 *                     stepping algorithm used by paint_pendulum_rod
 *                     to render the rod from pivot to bob.
 *                 [10] Gookin, D. (2007) — "Programmer's Guide to
 *                     NCurses", Wiley.  Double-buffered rendering
 *                     model, color-pair theory, and SIGWINCH handling
 *                     used in §3 and §9-10.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Three magnets sit at the corners of a triangle.  A pendulum bob
 * swings over them with gravity pulling it gently toward the centre.
 * Friction slowly bleeds energy away.  The bob ALWAYS ends up on
 * one magnet — but which?  The answer is a wildly intricate map.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Strong damping → bob barely moves → it just falls to the nearest
 * magnet → clean Voronoi-like basins, no surprises.
 * Weak damping → bob bounces around between magnets for a long
 * transient → tiny shifts in initial position route it differently
 * → basin BOUNDARIES become fractal (self-similar tendrils at
 * every magnification).
 *
 * KEY FORMULAS
 * ────────────
 *   r̈ = −k·r − γ·ṙ − Σ_i (r − m_i) / (|r − m_i|² + ε²)^{3/2}
 *
 *   k = 0.5    (gentle spring pulling bob toward (0,0))
 *   γ = 0.05 or 0.30  (preset-controlled damping)
 *   m_i = (cos(90° + i·120°), sin(90° + i·120°))   for i = 0,1,2
 *
 * HOW TO VERIFY  (observe the live animation)
 * ─────────────
 *  • STRONG (γ=0.30): bob takes the direct route to whichever magnet
 *    is closest to the release point.  Short trails, almost-straight
 *    paths; the orbit settles in ~1–2 seconds.
 *  • WEAK   (γ=0.05): bob loops around multiple magnets in long
 *    chaotic patterns before settling.  Two release points just a
 *    cell apart can lead to DIFFERENT magnets — watch by pressing
 *    'r' repeatedly from similar visible spots and seeing how often
 *    the winner number changes in the HUD.  Spirograph-like dense
 *    trails accumulate near magnets.
 *  • Reset (r) drops a fresh bob from a random release point.
 *    Animation never stops — orbits auto-respawn on settle.
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
    MAP_W_MAX        = 200, MAP_H_MAX = 56,
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    /* Pendulum visual uses ONE accent hue per theme split across 4
     * brightness tiers (oldest-trail → newest-trail / apparatus).  No
     * per-magnet colour: all 3 magnets render in the same accent so
     * the visual is clean and the dynamics of the bob are the focus. */
    PAIR_TIER_BASE   =   3,  /* 4 tiers: +0 dim … +3 bright (apparatus)  */
};
#define N_TIERS                   4
#define PAIR_TIER(i)              (PAIR_TIER_BASE + (i))
#define PAIR_TIER_BRIGHTEST       PAIR_TIER(N_TIERS - 1)
#define PAIR_TIER_DIM             PAIR_TIER(1)

#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                   1000000LL
#define TICK_NS(f)               (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET        60
#define RENDER_FRAME_BUDGET_NS   (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS      100

/* Physics */
#define N_MAGNETS                 3
#define MAGNET_RADIUS             1.0f    /* magnets on unit circle */
#define SPRING_K                  0.50f
#define MAGNET_EPS                0.10f
#define SIM_DT                    0.05f
#define MAX_STEPS              2000        /* per-cell integration budget */
#define CONV_V_SQ                 0.001f   /* |v|² convergence threshold */
#define CONV_R_SQ                 0.05f    /* |r - magnet|² threshold */

/* Phase-space viewport (each starting cell maps to a point here) */
#define X_MIN                    -1.7f
#define X_MAX                     1.7f
#define Y_MIN                    -1.5f
#define Y_MAX                     1.5f

/* ── pendulum-animation settings ────────────────────────────────── */
/* The viewer sees the dynamics of a real magnetic pendulum from a
 * SIDE-VIEW perspective: a single bob swinging from a fixed pivot
 * at the top of the screen, over three magnets below.  A long fading
 * trail records the recent swing path; once the bob is absorbed by
 * a magnet, a brief pause, then a new orbit is released from a fresh
 * random seed.  The animation never stops. */
#define LIVE_TRAIL_MAX              700     /* long sweep for the orbit pattern */
#define LIVE_TRAIL_BAND_COUNT         4     /* 4 colour tiers oldest→newest     */
/* LIVE_BOB_SUBSTEPS_PER_TICK — RK4 substeps executed per render tick.
 * Per-frame bob displacement = SIM_DT × this value ≈ 0.1 sim-units
 * (at SIM_DT=0.05).  Across the viewport's ~3.4 sim-unit width that's
 * about 3 % per frame — the swing reads as smooth, watchable motion
 * rather than a teleport every frame.  Total wall-clock sim rate is
 * sim_fps × SUBSTEPS × SIM_DT ≈ 6 sim-sec / real-sec at 60 Hz, so a
 * typical orbit takes ~10–20 real seconds to settle — long enough to
 * watch the chaotic swing, short enough to keep respawning. */
#define LIVE_BOB_SUBSTEPS_PER_TICK    2
#define LIVE_BOB_RESPAWN_FRAMES     120     /* ~2 sec settled-pause before
                                             * reseeding — gives the user
                                             * time to register the result. */
#define LIVE_BOB_WINNER_NONE         -1     /* sentinel: still moving           */

/* Anchor + rod (the pendulum apparatus, SIDE VIEW) — the spring force
 * in the physics is -k·r, i.e. it pulls the bob toward (0, 0).  In a
 * top-down rendering the spring origin is drawn at the centre of the
 * viewport.  In a SIDE-VIEW rendering we want the pendulum to read as
 * "hanging from a fixed pivot at the TOP of the screen, swinging
 * around DOWN below it" — so we draw a VIRTUAL pivot at top-centre of
 * the viewport and draw the rod from THAT virtual pivot down to the
 * bob's actual 2D physics position.  The bob's chaotic 2D motion is
 * still visible inside the viewport; the visible rod just gives the
 * eye a fixed "pendulum tied to the ceiling" reference point. */
#define ANCHOR_GLYPH               '+'
#define ANCHOR_TOP_OFFSET          1        /* pivot sits this many cells
                                             * below the top of the
                                             * viewport (so it isn't
                                             * flush against the HUD). */
#define ROD_GLYPH_HORIZ            '-'      /* horizontal step */
#define ROD_GLYPH_VERT             '|'      /* vertical step   */
#define ROD_GLYPH_DIAG_DOWNRIGHT   '\\'     /* diagonal step where sx==sy  */
#define ROD_GLYPH_DIAG_UPRIGHT     '/'      /* diagonal step where sx!=sy  */

/* Glyphs for the bob marker. */
#define LIVE_BOB_GLYPH_MOVING      '@'      /* bright @ while in motion       */
#define LIVE_BOB_GLYPH_SETTLED     'O'      /* O once absorbed into a magnet  */

/* Trail glyph ramp: 4 tiers oldest → newest.  Each glyph picked for
 * its ink weight: , (tiny) → . → : → + (heavy).  Combined with a
 * 4-tier colour ramp this gives 16 effective levels of fade. */
#define LIVE_TRAIL_GLYPH_TIER0     ','
#define LIVE_TRAIL_GLYPH_TIER1     '.'
#define LIVE_TRAIL_GLYPH_TIER2     ':'
#define LIVE_TRAIL_GLYPH_TIER3     '+'

/* Magnet markers — 3-cell wide '[N]' brackets where N is the magnet's
 * 1-based index ('1', '2', '3').  All in the same accent colour, but
 * the numbered centre self-labels each marker as "magnet N" so the
 * user can READ the visualisation directly (the HUD also reports
 * "absorbed → magnet N" using the same numbering). */
#define MAGNET_GLYPH_LEFT          '['
#define MAGNET_GLYPH_RIGHT         ']'
/* Per-magnet centre glyph: '1' + magnet index → '1', '2', '3'. */
#define MAGNET_GLYPH_CENTRE_FOR(i) ((char)('1' + (i)))

/*
 * Preset — index into the 2-entry presets[] table.  The two presets
 * pick qualitatively distinct regimes:
 *
 *   WEAK_DAMP   (γ=0.05) — bob oscillates for many swings before
 *                          settling.  Tiny shifts in release point
 *                          route it to wildly different magnets →
 *                          the FRACTAL boundary regime.
 *   STRONG_DAMP (γ=0.30) — bob barely overshoots once.  It just falls
 *                          toward the nearest magnet → CLEAN VORONOI
 *                          basins, no fractal sensitivity.
 *
 * The contrast between these two is the pedagogical payoff of the
 * demo — same equation, same magnets, totally different long-term
 * sensitivity, controlled by ONE parameter (γ).
 *
 * REFERENCES (numbered in file-header References block)
 *   [4] Grebogi, Ott, Yorke (1983) — the original fractal-basin-
 *       boundary paper.  Discusses exactly this damping-dependence.
 */
typedef enum {
    PRESET_WEAK_DAMP = 0,
    PRESET_STRONG_DAMP,
    N_PRESETS
} Preset;

/*
 * PendPreset — one row of the presets[] table: a named damping value.
 *
 * INTENT
 *   The magnetic-pendulum apparatus has only ONE knob the user can
 *   meaningfully turn: the damping coefficient γ.  Everything else
 *   (spring strength, magnet positions, softening) is fixed by the
 *   apparatus design.  This struct names a specific γ value so the
 *   user can cycle between regimes with n/p without having to
 *   remember numbers.
 *
 * CONTEXT
 *   Lives only in the const-table presets[] below.  Looked up by §7's
 *   PresetState wrapper via preset_state_active().  Consumed by
 *   scene_load_active_preset() which copies .gamma into
 *   orbit.pendulum.system.damping.  Never mutated at runtime.
 *
 * MEMBER LOGIC
 *   name  : 8-char label shown in HUD (%-8s format).  Picked to evoke
 *           the visual regime ("WEAK" = fractal tendrils; "STRONG"
 *           = clean Voronoi-like basins).
 *   gamma : damping coefficient γ ≥ 0.  Holmes-canon range is roughly
 *           γ ∈ [0.05, 0.5]; below 0.05 the bob essentially never
 *           settles, above 0.5 the dynamics become trivially
 *           dissipative.  This file's two presets bracket the
 *           interesting middle.
 */
typedef struct {
    const char *name;
    float       gamma;
} PendPreset;
static const PendPreset presets[N_PRESETS] = {
    { "WEAK    ", 0.05f },
    { "STRONG  ", 0.30f },
};

/*
 * Theme — N_TIERS shades of ONE accent hue, dim → bright.
 *
 * INTENT
 *   The pendulum-animation renderer uses FOUR brightness levels for
 *   the trail fade plus one for the bright apparatus (anchor / magnets
 *   / bob).  All five render roles use the SAME hue family — only the
 *   shade changes.  This keeps the visual minimal: the animation
 *   reads as "one accent colour, multiple intensities".  Cycling
 *   themes with t/T just rotates the HUE family — DEFAULT (greys),
 *   MATRIX (greens), OCEAN (cyan), FIRE (orange), etc.
 *
 * CONTEXT
 *   Lives only in the const-table themes[] of N_THEMES entries.
 *   Selected by §7's PaletteState wrapper via palette_state_active().
 *   Pushed into ncurses by §3 theme_apply() which calls init_pair()
 *   on each tier index.  Never mutated at runtime.
 *
 * MEMBER LOGIC
 *   name        : 7-char label shown on row-1 HUD as the current theme.
 *   tier[0]     : dimmest tier — oldest trail samples (the tail end
 *                 of the fade).
 *   tier[1]     : dim tier — trail mid-old; also the pendulum rod.
 *   tier[2]     : mid tier — trail mid-new.
 *   tier[N-1]   : brightest tier — newest trail, anchor '+', magnet
 *                 '[N]' brackets, and the live '@' bob.  Always the
 *                 most saturated colour in the theme.
 *
 *   Every entry must sit at xterm-256 index ≥ 24 per CLAUDE.md
 *   "Theme Palette Brightness" so the dimmest tier stays legible
 *   against default-black with A_BOLD.
 *
 * REFERENCES
 *   • CLAUDE.md "Theme Palette Brightness" — the ≥ 24 rule that
 *     keeps tier[0] legible against default-black.
 *   • CLAUDE.md "HUD Standard" — PAIR_HUD / PAIR_HINT use bright
 *     yellow + cyan and are NOT per-theme; only the apparatus tiers
 *     in this struct cycle with t/T.
 */
typedef struct {
    const char *name;
    short       tier[N_TIERS];
} Theme;

#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    /*                  dim →                              → bright */
    { "DEFAULT",   { 240, 244, 248, 252 } },   /* greyscale          */
    { "MATRIX",    {  28,  34,  82, 118 } },   /* greens             */
    { "NOVA",      {  90, 126, 162, 207 } },   /* magenta            */
    { "MONO",      { 240, 245, 250, 255 } },   /* pure greyscale     */
    { "OCEAN",     {  24,  31,  39,  51 } },   /* cyans              */
    { "FIRE",      {  94, 130, 208, 220 } },   /* warm orange ramp   */
    { "EARTH",     {  94, 130, 173, 215 } },   /* terracotta         */
    { "FOREST",    {  22,  28,  64, 156 } },   /* deep green ramp    */
    { "DESERT",    {  94, 130, 180, 222 } },   /* tan / dust          */
    { "ARCTIC",    {  24,  31,  81, 195 } },   /* pale blue          */
};

/* ===================================================================== */
/* §2 clock + §3 color                                                    */
/* ===================================================================== */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* theme_apply — push the N_TIERS shades of the active theme into
 * ncurses.  The 4 tier pairs are then used by every painter:
 *   tier 0 (dim)    — oldest trail samples
 *   tier 1          — older trail
 *   tier 2 (mid)    — newer trail, rod
 *   tier 3 (bright) — newest trail, anchor, magnets, bob marker  */
/* PAIR_HUD_FG_256 / PAIR_HINT_FG_256 — bright yellow / bright cyan in
 * the xterm-256 cube.  Named so the HUD setup doesn't carry magic
 * literals inline. */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

/* theme_apply_tiers_256color — push the N_TIERS shades of the active
 * theme into ncurses pair table.  Each tier index maps to one
 * xterm-256 colour from the Theme row.  All foregrounds use the
 * terminal's default background (-1). */
static void theme_apply_tiers_256color(const Theme *t)
{
    for (int i = 0; i < N_TIERS; i++)
        init_pair(PAIR_TIER(i), t->tier[i], -1);
}

/* theme_apply_tiers_8color_fallback — 8-colour fallback: all tiers
 * collapse to white.  Keeps the apparatus legible on legacy terminals
 * but loses the brightness gradient (and thus the trail fade). */
static void theme_apply_tiers_8color_fallback(void)
{
    for (int i = 0; i < N_TIERS; i++)
        init_pair(PAIR_TIER(i), COLOR_WHITE, -1);
}

/* theme_apply — top-level dispatcher: clamp the index, branch on
 * terminal capability, delegate to the appropriate helper. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_tiers_256color(&themes[idx]);
    else               theme_apply_tiers_8color_fallback();
}

/* color_init_hud_pairs — install the project-standard HUD + HINT
 * pairs per CLAUDE.md's "HUD Standard".  These pairs are NOT
 * per-theme — they stay constant as t/T cycles so the status row
 * never loses contrast. */
static void color_init_hud_pairs(void)
{
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  PAIR_HUD_FG_256,  -1);
        init_pair(PAIR_HINT, PAIR_HINT_FG_256, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* color_init — bring ncurses colour subsystem online and load theme 0.
 *
 *   STEP 1 — enable colour + default-bg passthrough
 *   STEP 2 — install the constant HUD pairs (project standard)
 *   STEP 3 — push the first theme's tier colours
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    color_init_hud_pairs();
    theme_apply(0);
}

/* ===================================================================== */
/* §5  physics — damped pendulum + magnetic attractions                   */
/* ===================================================================== */

/*
 * The equation of motion for the bob (mass = 1, 2-D plane) is:
 *
 *   r̈ = F_spring(r) + F_damping(v) + F_magnets(r)
 *
 *     F_spring(r)    = -k · r                       central restoring spring
 *     F_damping(v)   = -γ · v                       viscous damping
 *     F_magnets(r)   = -Σᵢ (r - mᵢ) / (|r - mᵢ|² + ε²)^{3/2}
 *                       └──── inverse-cube attraction to each magnet ────┘
 *
 * The naming reflects this decomposition explicitly — pendulum_deriv()
 * builds the acceleration as spring_term + damping_term + magnet_term.
 */

/*
 * Magnet — one fixed attractor in the 2-D plane.
 *
 * INTENT
 *   The bob feels an inverse-cube (softened) attraction toward each
 *   magnet.  Each magnet's STRENGTH is folded into the universal
 *   prefactor of the force law — so a Magnet only needs to know
 *   WHERE it sits.  Keeping the struct minimal makes the magnet field
 *   a pure spatial concept.
 *
 * MEMBER LOGIC
 *   x, y : magnet position in physics units.  By default the three
 *          magnets sit on a unit circle at angles 90°, 210°, 330°
 *          forming an equilateral triangle around the origin —
 *          matches the canonical apparatus from ref [2] Moon §4.7.
 */
typedef struct {
    float x, y;
} Magnet;

/*
 * MagnetField — the set of attractors that pull on the bob.
 *
 * INTENT
 *   A magnetic-pendulum experiment can have any number of magnets in
 *   any layout — but our pedagogical demo uses the classic 3-magnet
 *   equilateral triangle (refs [2] [3]).  Bundling the magnet set
 *   as one struct means:
 *     • the force-summation loop has a single "for each magnet" to
 *       iterate over;
 *     • magnet positions live inside PendulumSystem, not as file-scope
 *       globals — so the apparatus is a SELF-CONTAINED object;
 *     • a future "add a 4th magnet" experiment touches just this
 *       struct, not the physics code.
 *
 * CONTEXT
 *   One instance lives inside PendulumSystem.  Built by
 *   magnet_field_equilateral_triangle() at scene_init time.  Read by
 *   magnet_field_net_attraction() during every RK4 substep (so ~120
 *   reads/sec) and by pendulum_test_convergence() each substep.
 *   Const during a run — magnets don't move.
 *
 * MEMBER LOGIC
 *   count : number of magnets actually in use (≤ N_MAGNETS, the
 *           static capacity from §1).  Letting count be data (not a
 *           compile-time fixed loop bound) means the same code paths
 *           work for any 1..N_MAGNETS configuration.
 *   m     : magnet positions.  Indices 0..count-1 are live; indices
 *           count..N_MAGNETS-1 are unused space.
 *
 * REFERENCES (numbered in file-header References block)
 *   [2] Moon §4.7 — describes the 3-magnet equilateral-triangle
 *                   layout as the canonical configuration.
 */
typedef struct {
    int    count;
    Magnet m[N_MAGNETS];
} MagnetField;

/*
 * PendulumSystem — the INVARIANT parameters of the apparatus:
 *
 *   spring_k    : central spring constant (force per unit displacement
 *                 toward origin).  Holds the bob's nominal "rest"
 *                 position at (0, 0) — the bottom of the bowl.
 *   damping     : viscous damping coefficient γ.  Energy SINK per unit
 *                 velocity.  The ONLY parameter the user actually
 *                 varies — n/p swaps between WEAK (γ=0.05) and STRONG
 *                 (γ=0.30) presets.
 *   softening   : ε in the magnet-attraction denominator.  Caps the
 *                 1/r² singularity so the bob can't blow up if it
 *                 passes "through" a magnet during numerical
 *                 integration.
 *   magnets     : §5 MagnetField of attractor positions.
 *
 * REFERENCES (numbered in file-header References block)
 *   [2] Moon (1992) Ch.4 §4.7 — describes the SAME apparatus (3
 *                                magnets + spring + damping) as the
 *                                experimental setup whose dynamics
 *                                this struct parameterises.
 *   [3] Aguirre & Sanjuán (2003) — formal study of the same triple
 *                                  (k, γ, ε) parameterisation;
 *                                  basin-entropy curves vs damping.
 */
typedef struct {
    float       spring_k;
    float       damping;
    float       softening;
    MagnetField magnets;
} PendulumSystem;

/*
 * BobState — the 4-D state of the ODE: position (x, y) + velocity
 * (vx, vy).
 *
 * INTENT
 *   The state vector y for the ODE dy/dt = f(y, sys).  "Where we are
 *   in iterating the system" — what evolves under pendulum_rk4().
 *   Kept as its own type so it can be passed by value into
 *   pendulum_deriv() and combined with state_add() (k₁..k₄ are
 *   themselves BobState values) without dragging system parameters
 *   along.
 *
 *   Flat by-value struct so RK4 stage states (slope_start … slope_end)
 *   can be created and combined cheaply.  No pointer aliasing risk.
 *
 * CONTEXT
 *   One instance lives inside Pendulum.state.  Lifetime:
 *     • pendulum_state_release_at writes initial (x₀, y₀, 0, 0) on
 *       every orbit release.
 *     • pendulum_rk4 mutates ~120 times/sec (60 frames × 2 substeps).
 *     • pendulum_deriv reads at every RK4 stage; pendulum_test_
 *       convergence reads at every substep.
 *
 * MEMBER LOGIC
 *   x, y   : configuration-space position.  Spring pulls toward (0, 0);
 *           magnets pull toward their fixed positions.  No bound —
 *           the bob can in principle escape if energy is high enough,
 *           but in practice the spring + damping keep it well inside
 *           the viewport.
 *   vx, vy : velocity dr/dt.  Sign convention matches position: +x
 *            velocity = motion in +x direction.  Damping continually
 *            shrinks |v| → 0; settling threshold is CONV_V_SQ.
 *
 * REFERENCES (numbered in file-header References block)
 *   [6] Numerical Recipes Ch. 17.1 — explains why an ODE state
 *       vector is best as a FLAT BY-VALUE type for RK4: k_i need
 *       cheap structural copies, not pointer aliasing.
 */
typedef struct {
    float x, y;
    float vx, vy;
} BobState;

/*
 * Pendulum — composite: SYSTEM + STATE.
 *
 * INTENT
 *   Bundle the parameters and the dynamic state under one named
 *   type so callers that need "the whole pendulum" take one pointer
 *   instead of two.  RK4 mutates only .state; preset cycling mutates
 *   only .system.damping.  The split mirrors the textbook treatment:
 *   PARAMETERS / COORDINATES.
 *
 * CONTEXT
 *   One instance lives inside ActiveOrbit.pendulum.  Sub-structs are
 *   touched separately:
 *     .system : written ONCE by pendulum_system_init_default() at
 *               scene_init.  .system.damping is updated on each
 *               preset change.
 *     .state  : written ~120 times/sec by pendulum_rk4 substeps;
 *               reset by pendulum_state_release_at() on every
 *               orbit release.
 *
 * MEMBER LOGIC
 *   system : §5 PendulumSystem — k, γ, ε, magnets.
 *   state  : §5 BobState — (x, y, vx, vy) ODE coordinate.
 *
 * REFERENCES (numbered in file-header References block)
 *   [8] Sussman & Wisdom Ch. 3 — the modern functional-style
 *       PARAMETER / STATE separation that this struct's two-way
 *       decomposition reproduces in code form.
 */
typedef struct {
    PendulumSystem system;
    BobState       state;
} Pendulum;

/* magnet_field_equilateral_triangle — the canonical 3-magnet layout:
 * vertices of an equilateral triangle of radius MAGNET_RADIUS centred
 * on the origin, with magnet 0 at the TOP (angle 90°) and the other
 * two at 210° and 330°.  Standard textbook configuration. */
static MagnetField magnet_field_equilateral_triangle(void)
{
    MagnetField mf;
    mf.count = N_MAGNETS;
    for (int i = 0; i < N_MAGNETS; i++) {
        float angle_deg = 90.0f + 120.0f * (float)i;
        float angle_rad = angle_deg * (float)M_PI / 180.0f;
        mf.m[i].x = MAGNET_RADIUS * cosf(angle_rad);
        mf.m[i].y = MAGNET_RADIUS * sinf(angle_rad);
    }
    return mf;
}

/* pendulum_system_init_default — build the canonical PendulumSystem
 * with SPRING_K / MAGNET_EPS from §1 and the triangle of magnets.
 * Damping is filled separately by scene_load_active_preset. */
static PendulumSystem pendulum_system_init_default(void)
{
    PendulumSystem sys;
    sys.spring_k  = SPRING_K;
    sys.damping   = 0.0f;                        /* set per-preset */
    sys.softening = MAGNET_EPS;
    sys.magnets   = magnet_field_equilateral_triangle();
    return sys;
}

/* magnet_field_net_attraction — accumulate the net attraction force
 *   F = -Σᵢ (r - mᵢ) / (|r - mᵢ|² + ε²)^{3/2}
 * onto (*ax, *ay).  ε² is passed precomputed for efficiency.  Each
 * magnet contributes an inverse-cube attractive term softened by ε. */
static inline void magnet_field_net_attraction(const MagnetField *mf,
                                               float x, float y,
                                               float eps_sq,
                                               float *ax, float *ay)
{
    *ax = 0.0f;
    *ay = 0.0f;
    for (int i = 0; i < mf->count; i++) {
        float dx = x - mf->m[i].x;
        float dy = y - mf->m[i].y;
        float r2_softened = dx * dx + dy * dy + eps_sq;
        float inv_r3      = 1.0f / (r2_softened * sqrtf(r2_softened));
        *ax -= dx * inv_r3;
        *ay -= dy * inv_r3;
    }
}

/* pendulum_deriv — evaluate dy/dt = f(state, system).
 *
 *   STEP 1 — SPRING term  -k·r        (central restoring force)
 *   STEP 2 — DAMPING term -γ·v        (viscous energy sink)
 *   STEP 3 — MAGNET term  -Σ(r-mᵢ)/…  (attractive net force)
 *   STEP 4 — assemble derivative: kinematic identity for (x, y),
 *            sum of three force terms for (vx, vy)
 *
 * Equation form: header ref [1] Strogatz §7.2 gives the general 2-D
 * dissipative-system framework; ref [2] Moon §4.7 specialises to the
 * three-magnet pendulum.  The magnet attraction is the SOFTENED
 * inverse-cube law (ε² added to r²) used to keep the integrator
 * stable when the bob is close to a magnet — ref [3] uses the
 * same form. */
static inline BobState pendulum_deriv(const BobState *s,
                                      const PendulumSystem *sys)
{
    /* STEP 1 — spring (central restoring force toward origin) */
    float spring_ax  = -sys->spring_k * s->x;
    float spring_ay  = -sys->spring_k * s->y;

    /* STEP 2 — damping (viscous, opposite to velocity) */
    float damping_ax = -sys->damping  * s->vx;
    float damping_ay = -sys->damping  * s->vy;

    /* STEP 3 — net magnet attraction (sum over all magnets) */
    float magnet_ax, magnet_ay;
    magnet_field_net_attraction(&sys->magnets, s->x, s->y,
                                sys->softening * sys->softening,
                                &magnet_ax, &magnet_ay);

    /* STEP 4 — assemble (kinematic + dynamic) */
    BobState dy;
    dy.x  = s->vx;                                       /* dx/dt  = vx */
    dy.y  = s->vy;                                       /* dy/dt  = vy */
    dy.vx = spring_ax + damping_ax + magnet_ax;          /* dvx/dt = ΣFx */
    dy.vy = spring_ay + damping_ay + magnet_ay;          /* dvy/dt = ΣFy */
    return dy;
}

/* state_add — y + h·k, returned as a new BobState.  Builds the
 * intermediate stage states inside RK4. */
static inline BobState state_add(const BobState *a, float h, const BobState *k)
{
    BobState r;
    r.x  = a->x  + h * k->x;
    r.y  = a->y  + h * k->y;
    r.vx = a->vx + h * k->vx;
    r.vy = a->vy + h * k->vy;
    return r;
}

/* RK4 Butcher tableau constants. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* rk4_butcher_weighted_average — (k₁ + 2k₂ + 2k₃ + k₄) / 6.  The
 * Simpson-rule combination of the four slope estimates; weights from
 * the classical 4th-order Butcher tableau (Numerical Recipes 17.1). */
static inline BobState rk4_butcher_weighted_average(
    const BobState *k1, const BobState *k2,
    const BobState *k3, const BobState *k4)
{
    BobState avg;
    avg.x  = (k1->x  + 2.0f*k2->x  + 2.0f*k3->x  + k4->x ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y  = (k1->y  + 2.0f*k2->y  + 2.0f*k3->y  + k4->y ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.vx = (k1->vx + 2.0f*k2->vx + 2.0f*k3->vx + k4->vx) / RK4_BUTCHER_WEIGHT_SUM;
    avg.vy = (k1->vy + 2.0f*k2->vy + 2.0f*k3->vy + k4->vy) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* pendulum_rk4 — one fixed-step RK4 update of the bob's BobState
 * against its PendulumSystem.
 *
 *   STAGE 1 — slope_start = f(y)
 *   STAGE 2 — slope_mid_1 = f(y + ½dt · slope_start)
 *   STAGE 3 — slope_mid_2 = f(y + ½dt · slope_mid_1)
 *   STAGE 4 — slope_end   = f(y +  dt · slope_mid_2)
 *   UPDATE  — y ← y + dt · (k₁ + 2k₂ + 2k₃ + k₄) / 6
 *
 * See header ref [6] Numerical Recipes Ch. 17.1 for the full Butcher
 * tableau, O(dt⁵) local truncation, and stability domain.  This
 * system is dissipative (γ > 0) so RK4's energy non-conservation is
 * irrelevant — the bob LOSES energy regardless. */
static void pendulum_rk4(Pendulum *p, float dt)
{
    const PendulumSystem *sys = &p->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    BobState slope_start    = pendulum_deriv(&p->state, sys);
    BobState midpoint_1     = state_add(&p->state, half_dt, &slope_start);
    BobState slope_mid_1    = pendulum_deriv(&midpoint_1, sys);
    BobState midpoint_2     = state_add(&p->state, half_dt, &slope_mid_1);
    BobState slope_mid_2    = pendulum_deriv(&midpoint_2, sys);
    BobState endpoint       = state_add(&p->state, dt, &slope_mid_2);
    BobState slope_end      = pendulum_deriv(&endpoint, sys);

    BobState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    p->state = state_add(&p->state, dt, &effective_slope);
}

/* pendulum_state_release_at — set the bob's position to (x, y) with
 * zero velocity.  This is the standard "drop the bob and let it
 * fall" initial condition used in the classical basin-of-attraction
 * literature (refs [2] Moon, [3] Aguirre & Sanjuán) — so the magnet
 * the bob eventually settles on is the canonical basin assignment
 * for that release point. */
static inline void pendulum_state_release_at(BobState *s, float x, float y)
{
    s->x  = x;
    s->y  = y;
    s->vx = 0.0f;
    s->vy = 0.0f;
}

/* pendulum_test_convergence — has the bob settled on a magnet?
 *   TRUE conditions:
 *     |v|² < CONV_V_SQ         (the bob is essentially at rest)
 *     |r - mᵢ|² < CONV_R_SQ    (the bob is within capture radius)
 * Returns the magnet index 0..count-1, or LIVE_BOB_WINNER_NONE. */
static int pendulum_test_convergence(const BobState *s, const MagnetField *mf)
{
    if (s->vx * s->vx + s->vy * s->vy >= CONV_V_SQ)
        return LIVE_BOB_WINNER_NONE;
    for (int i = 0; i < mf->count; i++) {
        float dx = s->x - mf->m[i].x;
        float dy = s->y - mf->m[i].y;
        if (dx * dx + dy * dy < CONV_R_SQ) return i;
    }
    return LIVE_BOB_WINNER_NONE;
}

/* ===================================================================== */
/* §6  trail — ring buffer of recent bob positions                        */
/* ===================================================================== */

/*
 * Trail — fixed-capacity ring buffer of recent (x, y) positions of
 * the bob.  Drawn by §9 as a fading swing-path behind the bob.
 *
 * INTENT
 *   The orbit's RECENT HISTORY tells the viewer "where the bob has
 *   been" — the curving sweep around magnets, the lurches near
 *   boundaries, the spiral as it settles.  A static frame would show
 *   one '@' character — useless.  This struct stores LIVE_TRAIL_MAX
 *   samples so the renderer can paint a fading tail that conveys
 *   both DIRECTION (newest is brightest) and HISTORY (oldest fades
 *   to dim) without any extra motion cue.
 *
 *   The trail also makes the chaotic regime LEGIBLE: with WEAK
 *   damping the bob's path winds densely around magnets producing
 *   beautiful spirograph patterns — none of which are visible from
 *   a single snapshot of the bob position.
 *
 * CONTEXT
 *   One instance lives inside ActiveOrbit.trail.  Lifetime:
 *     • trail_reset clears head/count on each orbit release.
 *     • trail_push appends one sample per RK4 substep (~120 pushes/sec).
 *     • paint_trail walks oldest→newest each render frame, applying
 *       the 4-tier brightness fade via trail_tier_for_age().
 *
 * DATA-STRUCTURE LOGIC
 *   Ring buffer chosen over a growable list for:
 *     1. ZERO ALLOCATION in the hot path — per CLAUDE.md, scene_tick
 *        must not malloc/free.
 *     2. CONSTANT cost per frame — push and full-buffer iteration are
 *        both O(LIVE_TRAIL_MAX), no resize step.
 *   Newest sample at .head; oldest at trail_oldest_index().  push
 *   advances head FIRST then writes, so .head is always the just-
 *   written slot.
 *
 * MEMBER LOGIC
 *   x[i], y[i] : physical (x, y) of the i-th stored sample, in the
 *                same units as BobState — no conversion at push time
 *                so the trail inherits the integrator's sub-cell
 *                precision.  Conversion to screen cells happens
 *                ONCE in viewport_phys_to_cell at paint time.
 *   head       : index of MOST RECENT sample, wraps mod LIVE_TRAIL_MAX.
 *   count      : valid-sample count, saturates at LIVE_TRAIL_MAX
 *                (so a long-running orbit drops oldest samples).
 *
 * REFERENCES
 *   • Sedgewick "Algorithms" 4th ed. §1.3 — canonical reference for
 *     ring-buffer / circular-queue implementation.
 */
typedef struct {
    float x[LIVE_TRAIL_MAX];
    float y[LIVE_TRAIL_MAX];
    int   head;
    int   count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }

static void trail_push(Trail *t, float x, float y)
{
    t->head = (t->head + 1) % LIVE_TRAIL_MAX;
    t->x[t->head] = x;
    t->y[t->head] = y;
    if (t->count < LIVE_TRAIL_MAX) t->count++;
}

/* trail_oldest_index — index of the OLDEST still-valid sample in the
 * ring.  Same formula works in both "filling" and "full" cases. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + LIVE_TRAIL_MAX) % LIVE_TRAIL_MAX;
}

/* trail_tier_for_age — map sample age to one of LIVE_TRAIL_BAND_COUNT
 * brightness tiers.  Linear partition: age=0 (newest) → highest tier,
 * age=count-1 (oldest) → tier 0.  Used by the painter for the fade. */
static inline int trail_tier_for_age(int age, int count)
{
    int tier = (LIVE_TRAIL_BAND_COUNT - 1)
             - (age * LIVE_TRAIL_BAND_COUNT) / count;
    if (tier < 0)                         tier = 0;
    if (tier > LIVE_TRAIL_BAND_COUNT - 1) tier = LIVE_TRAIL_BAND_COUNT - 1;
    return tier;
}

/* ===================================================================== */
/* §7  active orbit + state wrappers                                      */
/* ===================================================================== */

/* BOB_TIMEOUT — sentinel value stored in ActiveOrbit.winner when an
 * orbit fails to converge within MAX_STEPS substeps.  Negative so it
 * can't be mistaken for a valid magnet index 0..N_MAGNETS-1. */
#define BOB_TIMEOUT  (-2)

/*
 * ActiveOrbit — the single live trajectory being animated.
 *
 * INTENT
 *   Bundles everything that describes "the one orbit currently
 *   swinging" — the pendulum (system + state), the trail, the
 *   per-orbit bookkeeping (step counter, winner state, respawn
 *   timer, seed position).  When the bob is absorbed by a magnet
 *   the countdown ticks down, and at zero a fresh random seed
 *   restarts the cycle — keeping the animation perpetual.
 *
 * CONTEXT
 *   One instance lives on Scene.  Lifetime per orbit:
 *     • active_orbit_release_random_seed — drop bob at a fresh
 *       random (x, y); zero velocity; clear trail; winner=NONE.
 *     • active_orbit_advance — RK4-advance LIVE_BOB_SUBSTEPS_PER_TICK
 *       substeps per render frame; push to trail; test convergence;
 *       on settle start the respawn countdown.
 *     • Once respawn_countdown reaches 0 → release a new random seed.
 *
 * MEMBER LOGIC
 *   pendulum          : the physical body (§5 Pendulum: system + state).
 *   trail             : §6 Trail of recent positions for the fade.
 *   step_count        : substeps elapsed since release (timeout check).
 *   winner            : LIVE_BOB_WINNER_NONE while moving,
 *                       0..N_MAGNETS-1 once converged,
 *                       BOB_TIMEOUT if it ran out of substeps.
 *   respawn_countdown : frames left in the "absorbed" pause before
 *                       reseeding.  Gives the viewer time to register
 *                       which magnet won.
 *   seed_x, seed_y    : the release position.  Drawn as a dim 'o' so
 *                       the viewer can see where the orbit STARTED.
 */
typedef struct {
    Pendulum pendulum;
    Trail    trail;
    int      step_count;
    int      winner;
    int      respawn_countdown;
    float    seed_x, seed_y;
} ActiveOrbit;

/* viewport_random_phys_point — pick a uniformly-random (x, y) in the
 * full viewport bounds [X_MIN, X_MAX] × [Y_MIN, Y_MAX]. */
static void viewport_random_phys_point(float *x, float *y)
{
    float fx = (float)rand() / (float)RAND_MAX;
    float fy = (float)rand() / (float)RAND_MAX;
    *x = X_MIN + fx * (X_MAX - X_MIN);
    *y = Y_MIN + fy * (Y_MAX - Y_MIN);
}

/* active_orbit_release_at — initialise the orbit with the bob at
 * (x0, y0), zero velocity, fresh trail, winner=NONE. */
static void active_orbit_release_at(ActiveOrbit *orb, float x0, float y0)
{
    pendulum_state_release_at(&orb->pendulum.state, x0, y0);
    trail_reset(&orb->trail);
    orb->step_count        = 0;
    orb->winner            = LIVE_BOB_WINNER_NONE;
    orb->respawn_countdown = 0;
    orb->seed_x            = x0;
    orb->seed_y            = y0;
}

/* active_orbit_release_random_seed — pick a random (x, y) and release
 * the bob there.  Called on r-press and on every auto-respawn. */
static void active_orbit_release_random_seed(ActiveOrbit *orb)
{
    float x0, y0;
    viewport_random_phys_point(&x0, &y0);
    active_orbit_release_at(orb, x0, y0);
}

/* active_orbit_step_once — one RK4 substep + trail push + termination
 * test.  Returns false the moment the bob settles or times out.
 *
 *   STEP 0 — short-circuit if already settled
 *   STEP 1 — advance physics by ONE RK4 substep of size SIM_DT
 *   STEP 2 — record the new position in the fading trail
 *   STEP 3 — test convergence (slow AND near a magnet → absorbed)
 *   STEP 4 — test timeout (too many steps without settling)
 */
static bool active_orbit_step_once(ActiveOrbit *orb)
{
    /* STEP 0 — early-exit if the bob is already absorbed */
    if (orb->winner != LIVE_BOB_WINNER_NONE) return false;

    /* STEP 1 — RK4 advance */
    pendulum_rk4(&orb->pendulum, SIM_DT);

    /* STEP 2 — record in the trail ring buffer */
    trail_push(&orb->trail, orb->pendulum.state.x, orb->pendulum.state.y);
    orb->step_count++;

    /* STEP 3 — convergence test (slow + near magnet) */
    int absorbed_magnet = pendulum_test_convergence(&orb->pendulum.state,
                                                    &orb->pendulum.system.magnets);
    if (absorbed_magnet != LIVE_BOB_WINNER_NONE) {
        orb->winner            = absorbed_magnet;
        orb->respawn_countdown = LIVE_BOB_RESPAWN_FRAMES;
        return false;
    }

    /* STEP 4 — timeout (orbit never settled within budget) */
    if (orb->step_count >= MAX_STEPS) {
        orb->winner            = BOB_TIMEOUT;
        orb->respawn_countdown = LIVE_BOB_RESPAWN_FRAMES;
        return false;
    }
    return true;
}

/* active_orbit_tick — advance LIVE_BOB_SUBSTEPS_PER_TICK substeps if
 * the bob is moving; otherwise tick the respawn countdown and reseed
 * at zero.  The animation never stops — one orbit ends, another
 * starts.  Called once per scene_tick. */
static void active_orbit_tick(ActiveOrbit *orb)
{
    if (orb->winner == LIVE_BOB_WINNER_NONE) {
        for (int i = 0; i < LIVE_BOB_SUBSTEPS_PER_TICK; i++)
            if (!active_orbit_step_once(orb)) break;
        return;
    }
    if (orb->respawn_countdown > 0) {
        orb->respawn_countdown--;
        return;
    }
    active_orbit_release_random_seed(orb);
}

/*
 * PresetState — typed wrapper around "which damping preset is loaded".
 *
 * INTENT
 *   The Preset enum is a bare int — wrapping it in a struct + cycle
 *   helpers lifts modular arithmetic out of the key-binding table so
 *   app_handle_key reads as
 *
 *       case 'n': preset_state_cycle_next(&s->preset); scene_reset(...);
 *
 *   This is the "Replace Primitive with Object" pattern — a one-field
 *   struct exists not to hold data but to ATTACH a named API to that
 *   data so the call sites read as intentions.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_preset_next /
 *   prev when n/p are pressed.  Read by scene_load_active_preset
 *   (via preset_state_active_gamma → orbit.pendulum.system.damping)
 *   and by HUD writers for the row-1 "preset:NAME" label.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table (§1).  Must be in
 *             [0, N_PRESETS) — cycle helpers guarantee this by
 *             modular arithmetic.  Initial value is PRESET_WEAK_DAMP
 *             (set by scene_init) — the dramatic fractal regime
 *             that demonstrates the system best.
 *
 * REFERENCES
 *   • Fowler "Refactoring" (2nd ed.), "Replace Primitive with Object".
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)        { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)        { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const PendPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }
static float            preset_state_active_gamma(const PresetState *p) { return presets[p->current].gamma; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState (one-int wrapper + cycle helpers) but
 *   kept as a SEPARATE type because their tables differ (presets[]
 *   vs themes[]).  Distinct types let scene_apply_theme() take
 *   exactly the right wrapper and refuse anything else — eliminates
 *   the class of bug where a "next" key on the wrong wrapper would
 *   silently index past one table's bounds into the other.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_theme_next /
 *   prev when t/T are pressed.  After every mutation the caller
 *   invokes palette_state_apply() which forwards to §3 theme_apply()
 *   — the only place that touches ncurses init_pair() calls.
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table (§1).  Must be in
 *             [0, N_THEMES).  Initial value is 0 (DEFAULT, set by
 *             scene_init) — pure greyscale, theme-neutral starting
 *             point.
 *
 * REFERENCES
 *   • Fowler "Refactoring" — same "Replace Primitive with Object"
 *     pattern as PresetState.  The deliberate type-distinction
 *     between PresetState and PaletteState despite identical shape
 *     is the DDD "Tiny Types" idea: values from different domains
 *     shouldn't share a type just because the bits match.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)        { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)        { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p) { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)        { theme_apply(p->current); }

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — composite owner of all mutable simulation state.
 *
 * INTENT
 *   Bundles every piece of mutable state under one named struct so
 *   the App layer owns ONE Scene (no loose globals) and the main loop
 *   drives it through a tiny named-method API.  Each sub-field is a
 *   NAMED CONCEPT — the orbit currently swinging, which preset is
 *   loaded, which palette is active, and the drawable extent.
 *
 * MEMBER LOGIC
 *   orbit    : §7 ActiveOrbit — the single live trajectory.  Mutated
 *              by scene_tick(); reset on r-press / preset change.
 *   preset   : §7 PresetState — which damping is loaded.  Cycled by
 *              n/p; scene_load_active_preset() propagates γ into the
 *              orbit's PendulumSystem.
 *   palette  : §7 PaletteState — which theme is active.  Cycled by
 *              t/T; scene_apply_theme() pushes the change to ncurses.
 *   map_w/h  : grid dims (cells) for the renderer's viewport.
 *   paused   : when true, scene_tick early-returns.
 */
typedef struct {
    ActiveOrbit  orbit;
    PresetState  preset;
    PaletteState palette;
    int          map_w, map_h;
    bool         paused;
} Scene;

/* scene_load_active_preset — propagate the active preset's damping
 * value into the orbit's PendulumSystem.  This is the ONLY thing
 * preset cycling changes about the physics — magnets, spring, and
 * softening are fixed across presets. */
static void scene_load_active_preset(Scene *s)
{
    s->orbit.pendulum.system.damping = preset_state_active_gamma(&s->preset);
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s, int w, int h)
{
    s->map_w = w;
    s->map_h = h;
    scene_load_active_preset(s);
    active_orbit_release_random_seed(&s->orbit);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->paused = false;

    /* Sub-state init — each wrapper owns its own default. */
    preset_state_init (&s->preset,  PRESET_WEAK_DAMP);
    palette_state_init(&s->palette, 0);

    /* Build the pendulum system once: invariant magnet positions +
     * spring + softening; damping is set per-preset by scene_reset. */
    s->orbit.pendulum.system = pendulum_system_init_default();
    scene_reset(s, w, h);
}

/* scene_tick — advance the orbit's physics by LIVE_BOB_SUBSTEPS_PER_TICK
 * substeps; auto-respawn the bob whenever it settles or times out
 * (handled inside active_orbit_tick).  The animation never stops. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    active_orbit_tick(&s->orbit);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-adapter handle: ncurses lifecycle + current
 * window dimensions.
 *
 * INTENT
 *   ncurses owns the terminal mode and back-buffer; this struct is
 *   the THIN OWNERSHIP HANDLE the program passes around.  Bundling
 *   (cols, rows) here lets every painter take ONE Screen parameter
 *   instead of calling getmaxyx() at every draw site.  Funnelling
 *   all ncurses init/teardown through screen_init / screen_free
 *   keeps the dependency at a single boundary.
 *
 * CONTEXT
 *   One instance lives on App.  Initialised by screen_init() during
 *   app_bootstrap().  Refreshed by screen_resize() inside
 *   app_handle_pending_resize() whenever SIGWINCH fires.  Read by
 *   every painter that needs to clip against terminal bounds and by
 *   the HUD writers for right-anchored layout.  Freed at exit;
 *   endwin() also runs from atexit(cleanup) as a safety net.
 *
 * MEMBER LOGIC
 *   cols : current terminal width in CHARACTER CELLS.  Updated only
 *          by screen_init / screen_resize via getmaxyx().
 *   rows : current terminal height in CHARACTER CELLS.  Row 0 is the
 *          top of the terminal; row (rows - 1) is the bottom.  The
 *          HUD reserves HUD_TOP_ROWS at the top and HUD_BOTTOM_ROWS
 *          at the bottom (see §1).
 *
 * REFERENCES (numbered in file-header References block)
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 2 — the initscr
 *        / endwin / getmaxyx / wnoutrefresh / doupdate API surface
 *        the screen_* helpers wrap.
 */
typedef struct {
    int cols;
    int rows;
} Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/*
 * Viewport — the centred (gx0, gy0, w, h) rectangle that holds the
 * pendulum's 2-D motion field on screen.
 *
 * INTENT
 *   The bob lives in continuous physical coordinates (x, y) bounded
 *   by [X_MIN, X_MAX] × [Y_MIN, Y_MAX].  The viewport bundles the
 *   FOUR numbers (origin cell + cell dimensions) that every painter
 *   needs to map physical (x, y) → screen cells.  Without this
 *   struct each painter would take 4 loose ints and the conversion
 *   would be scattered.  Built once per frame in scene_paint.
 *
 *   The viewport also defines the SIDE-VIEW pivot (top-centre cell)
 *   via viewport_virtual_pivot_cell — the visible "this is where the
 *   pendulum is suspended from" anchor.
 *
 * MEMBER LOGIC
 *   gx0, gy0 : cell coordinates of the viewport's TOP-LEFT corner on
 *              screen.  Centred inside the drawable band by
 *              viewport_build; clipped against the HUD.
 *   w, h     : viewport size in cells.  Matches the Scene's map_w /
 *              map_h modulo clipping.
 *
 * REFERENCES (numbered in file-header References block)
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 4 — the mvaddch
 *        / wnoutrefresh model that every painter funnels through.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} Viewport;

static Viewport viewport_build(int map_w, int map_h, int cols, int rows)
{
    Viewport vp;
    vp.w   = map_w;
    vp.h   = map_h;
    vp.gx0 = (cols - vp.w) / 2;
    vp.gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - vp.h) / 2 + HUD_TOP_ROWS;
    if (vp.gx0 < 0)            vp.gx0 = 0;
    if (vp.gy0 < HUD_TOP_ROWS) vp.gy0 = HUD_TOP_ROWS;
    return vp;
}

/* VIEWPORT_ROUND_OFFSET — the 0.5f added before float→int truncation
 * to convert it into nearest-integer rounding.  Without it the cell
 * assignment skews low (every fractional cell index gets floored). */
#define VIEWPORT_ROUND_OFFSET   0.5f

/* viewport_phys_to_cell — convert a physical (x, y) into the
 * corresponding screen cell.
 *
 *   STEP 1 — bounds check; reject points outside the viewport
 *   STEP 2 — normalise (x, y) into fractional [0, 1] across the viewport
 *   STEP 3 — quantise to integer cell index (with rounding)
 *   STEP 4 — clip to [0, w-1] / [0, h-1] for safety
 *   STEP 5 — shift to screen coordinates, flipping Y so positive
 *            physics y maps to higher rows on screen */
static bool viewport_phys_to_cell(const Viewport *vp, float x, float y,
                                  int *sx, int *sy)
{
    /* STEP 1 — bounds */
    if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX) return false;

    /* STEP 2 — normalise to [0, 1] */
    float fx_norm = (x - X_MIN) / (X_MAX - X_MIN);
    float fy_norm = (y - Y_MIN) / (Y_MAX - Y_MIN);

    /* STEP 3 — quantise (with rounding) */
    int cell_x = (int)(fx_norm * (float)(vp->w - 1) + VIEWPORT_ROUND_OFFSET);
    int cell_y = (int)(fy_norm * (float)(vp->h - 1) + VIEWPORT_ROUND_OFFSET);

    /* STEP 4 — clip */
    if (cell_x < 0)         cell_x = 0;
    if (cell_x >= vp->w)    cell_x = vp->w - 1;
    if (cell_y < 0)         cell_y = 0;
    if (cell_y >= vp->h)    cell_y = vp->h - 1;

    /* STEP 5 — shift to screen coords, flip Y */
    *sx = vp->gx0 + cell_x;
    *sy = vp->gy0 + (vp->h - 1 - cell_y);
    return true;
}

/* paint_magnet_markers — 3-cell '[N]' brackets at each magnet's
 * physical position, where N is the magnet's 1-based number.  All
 * three magnets use the SAME brightest-tier pair (no per-magnet
 * hue) — the digit identifies WHICH magnet, the bracket frame says
 * "this is a magnet". */
static void paint_magnet_markers(const MagnetField *mf, const Viewport *vp)
{
    attron(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
    for (int i = 0; i < mf->count; i++) {
        int sx, sy;
        if (!viewport_phys_to_cell(vp, mf->m[i].x, mf->m[i].y, &sx, &sy))
            continue;
        if (sx > vp->gx0 && sx < vp->gx0 + vp->w - 1) {
            mvaddch(sy, sx - 1, (chtype)(unsigned char)MAGNET_GLYPH_LEFT);
            mvaddch(sy, sx + 1, (chtype)(unsigned char)MAGNET_GLYPH_RIGHT);
        }
        mvaddch(sy, sx, (chtype)(unsigned char)MAGNET_GLYPH_CENTRE_FOR(i));
    }
    attroff(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
}

/* trail_tier_glyph — 4-tier glyph ramp ',' '.' ':' '+'.  Combined
 * with the tier colour pair this gives 16 visual fade levels. */
static char trail_tier_glyph(int tier)
{
    switch (tier) {
    case 0:  return LIVE_TRAIL_GLYPH_TIER0;
    case 1:  return LIVE_TRAIL_GLYPH_TIER1;
    case 2:  return LIVE_TRAIL_GLYPH_TIER2;
    default: return LIVE_TRAIL_GLYPH_TIER3;
    }
}

/* paint_trail — render the trail as a 4-tier fade.  EACH SAMPLE uses
 * the tier-colour-pair matching its age (oldest → PAIR_TIER(0) dim,
 * newest → PAIR_TIER_BRIGHTEST). */
static void paint_trail(const Trail *t, const Viewport *vp)
{
    if (t->count == 0) return;
    int oldest = trail_oldest_index(t);
    for (int i = 0; i < t->count; i++) {
        int idx  = (oldest + i) % LIVE_TRAIL_MAX;
        int age  = t->count - 1 - i;
        int tier = trail_tier_for_age(age, t->count);
        int sx, sy;
        if (!viewport_phys_to_cell(vp, t->x[idx], t->y[idx], &sx, &sy))
            continue;
        int pair     = PAIR_TIER(tier);
        chtype glyph = (chtype)(unsigned char)trail_tier_glyph(tier);
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sy, sx, glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* paint_bob_marker — '@' (in motion) or 'O' (settled) at the bob's
 * current position.  Always in the brightest tier so the bob is the
 * visually dominant element of the apparatus. */
static void paint_bob_marker(const ActiveOrbit *orb, const Viewport *vp)
{
    int sx, sy;
    if (!viewport_phys_to_cell(vp, orb->pendulum.state.x,
                                   orb->pendulum.state.y, &sx, &sy)) return;
    chtype glyph = (orb->winner == LIVE_BOB_WINNER_NONE)
                 ? (chtype)(unsigned char)LIVE_BOB_GLYPH_MOVING
                 : (chtype)(unsigned char)LIVE_BOB_GLYPH_SETTLED;
    attron(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
    mvaddch(sy, sx, glyph);
    attroff(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
}

/* viewport_virtual_pivot_cell — screen cell of the SIDE-VIEW pendulum
 * pivot.  Always at the TOP-CENTRE of the viewport (one cell below
 * the top edge), regardless of where physical (0, 0) is.  This is the
 * "pendulum is tied to the ceiling here" anchor that gives the side-
 * view its visual identity. */
static void viewport_virtual_pivot_cell(const Viewport *vp,
                                        int *pivot_x, int *pivot_y)
{
    *pivot_x = vp->gx0 + vp->w / 2;
    *pivot_y = vp->gy0 + ANCHOR_TOP_OFFSET;
}

/* paint_anchor_marker — '+' at the SIDE-VIEW pivot (top-centre of
 * viewport), in bright white + bold so it reads as a fixed pendulum
 * pivot.  This is DIFFERENT from the spring origin (0, 0) — that's a
 * physics concept; this is the visual representation of "the
 * apparatus hangs from this point". */
static void paint_anchor_marker(const Viewport *vp)
{
    int pivot_x, pivot_y;
    viewport_virtual_pivot_cell(vp, &pivot_x, &pivot_y);
    attron(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
    mvaddch(pivot_y, pivot_x, (chtype)(unsigned char)ANCHOR_GLYPH);
    attroff(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
}

/* rod_glyph_for_step — direction-aware ASCII glyph for ONE Bresenham
 * step.  Same trick double_pendulum.c uses on its rods: pure
 * horizontal step → '-', pure vertical → '|', diagonal → '/' or '\'.
 * The line reads as a coherent rod at any angle, not a chain of dots. */
static chtype rod_glyph_for_step(bool step_x, bool step_y, int sx_dir, int sy_dir)
{
    if (step_x && step_y) return (sx_dir == sy_dir) ? ROD_GLYPH_DIAG_DOWNRIGHT
                                                    : ROD_GLYPH_DIAG_UPRIGHT;
    if (step_x)           return ROD_GLYPH_HORIZ;
    return ROD_GLYPH_VERT;
}

/* rod_cell_should_skip — TRUE when the current Bresenham cell is
 * EITHER the pivot ('+') OR the bob ('@') — those will be drawn by
 * their dedicated painters, and the rod glyph would overwrite them. */
static inline bool rod_cell_should_skip(int x, int y,
                                        int pivot_x, int pivot_y,
                                        int bob_x,   int bob_y)
{
    return (x == pivot_x && y == pivot_y)
        || (x == bob_x   && y == bob_y);
}

/* paint_pendulum_rod — Bresenham line from the virtual PIVOT
 * (top-centre of viewport) down to the bob's current screen cell.
 * Drawn in PAIR_TIER_DIM.  This is the SIDE-VIEW pendulum-rod —
 * always reads as "rod hanging from pivot at top, tied to the bob
 * below" regardless of where the bob's 2D physics position is.
 * Skipped entirely if the bob is off-viewport.
 *
 *   STEP 1 — locate the pivot and bob screen cells
 *   STEP 2 — initialise Bresenham state (deltas, step signs, error)
 *   STEP 3 — walk pixel-by-pixel toward the bob:
 *              • plot a direction-aware glyph (rod_glyph_for_step)
 *                UNLESS the cell is the pivot or the bob (skipped)
 *              • advance error + (x, y) by one Bresenham step
 *
 * See header ref [9] Bresenham (1965) for the integer-only line-
 * stepping algorithm.  The direction-aware glyph picker
 * rod_glyph_for_step adds an ASCII extension: the glyph reflects the
 * angle of the NEXT step, so the rod reads as a coherent line at any
 * orientation. */
static void paint_pendulum_rod(const ActiveOrbit *orb, const Viewport *vp)
{
    /* STEP 1 — pivot + bob screen cells */
    int pivot_x, pivot_y;
    viewport_virtual_pivot_cell(vp, &pivot_x, &pivot_y);
    int bob_x, bob_y;
    if (!viewport_phys_to_cell(vp, orb->pendulum.state.x,
                                   orb->pendulum.state.y, &bob_x, &bob_y))
        return;

    /* STEP 2 — Bresenham state (deltas, step signs, error accumulator) */
    int cursor_x      = pivot_x;
    int cursor_y      = pivot_y;
    int delta_x       = abs(bob_x - cursor_x);
    int delta_y       = abs(bob_y - cursor_y);
    int step_dir_x    = (cursor_x < bob_x) ?  1 : -1;
    int step_dir_y    = (cursor_y < bob_y) ?  1 : -1;
    int error_accum   = delta_x - delta_y;

    /* STEP 3 — walk from pivot toward bob */
    attron(COLOR_PAIR(PAIR_TIER_DIM));
    while (1) {
        if (!rod_cell_should_skip(cursor_x, cursor_y,
                                  pivot_x, pivot_y, bob_x, bob_y)) {
            int doubled_error  = 2 * error_accum;
            bool will_step_x   = (doubled_error > -delta_y);
            bool will_step_y   = (doubled_error <  delta_x);
            chtype glyph = rod_glyph_for_step(will_step_x, will_step_y,
                                              step_dir_x, step_dir_y);
            mvaddch(cursor_y, cursor_x, glyph);
        }
        if (cursor_x == bob_x && cursor_y == bob_y) break;

        /* Bresenham advance: pick which axes to step on this iteration */
        int doubled_error = 2 * error_accum;
        if (doubled_error > -delta_y) { error_accum -= delta_y; cursor_x += step_dir_x; }
        if (doubled_error <  delta_x) { error_accum += delta_x; cursor_y += step_dir_y; }
    }
    attroff(COLOR_PAIR(PAIR_TIER_DIM));
}

/* paint_pendulum_apparatus — render the single active orbit as a
 * proper side-view apparatus:
 *
 *   STEP 1 — anchor '+' at top-centre pivot
 *   STEP 2 — fading trail (recent swing history)
 *   STEP 3 — pendulum rod (pivot → current bob position)
 *   STEP 4 — bob marker ('@' moving / 'O' settled) at the rod's tip
 *
 * Order: rod over trail (so the current geometry is visible against
 * the historical sweep); bob over rod (so the bob always sits at the
 * rod's tip). */
static void paint_pendulum_apparatus(const ActiveOrbit *orb, const Viewport *vp)
{
    paint_anchor_marker(vp);
    paint_trail        (&orb->trail, vp);
    paint_pendulum_rod (orb, vp);
    paint_bob_marker   (orb, vp);
}

/* scene_paint — render the apparatus.
 *
 *   STEP 1 — build the viewport from the scene's map dims
 *   STEP 2 — paint pendulum apparatus (anchor + trail + rod + bob)
 *   STEP 3 — paint the magnet '[N]' markers on top so the attractors
 *            are always crisp and the bob visibly terminates INTO
 *            one when absorbed.
 */
static void scene_paint(const Scene *s, int cols, int rows)
{
    Viewport vp = viewport_build(s->map_w, s->map_h, cols, rows);
    paint_pendulum_apparatus(&s->orbit, &vp);
    paint_magnet_markers    (&s->orbit.pendulum.system.magnets, &vp);
}

/* hud_pendulum_status — concise status for the bob.  Kept ≤ 32 chars
 * so it composes cleanly into the fixed-width top-right HUD slot
 * without truncation. */
static void hud_pendulum_status(char *buf, size_t bufsz, const Scene *s)
{
    const ActiveOrbit *orb = &s->orbit;
    if (orb->winner == LIVE_BOB_WINNER_NONE)
        snprintf(buf, bufsz, "swinging %4d/%d", orb->step_count, MAX_STEPS);
    else if (orb->winner == BOB_TIMEOUT)
        snprintf(buf, bufsz, "TIMEOUT @ %d", orb->step_count);
    else
        snprintf(buf, bufsz, "absorbed→magnet %d", orb->winner + 1);
}

/* hud_paint_top_left_title — fixed title in row 0 column HUD_LEFT_MARGIN. */
static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " MAGNETIC PENDULUM ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_format_top_right_status — format the dynamic status string
 * (fps, Hz, preset name + index, bob state) into a stack buffer.
 * Pure formatter — no ncurses calls.  Pulled out so hud_top reads
 * as "format then paint", not a tangled snprintf+positioning block. */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    char pend_status[32];
    hud_pendulum_status(pend_status, sizeof pend_status, s);
    const PendPreset *active_preset = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  %s ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active_preset->name,
             s->preset.current + 1, N_PRESETS, pend_status);
}

/* hud_paint_top_right — paint a right-anchored string at row 0.
 * Right-anchor formula: column = cols - strlen, clipped at 0 so a
 * narrow terminal doesn't push the start negative. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_top — row-0 HUD driver.
 *
 *   STEP 1 — top-left title cell (constant text)
 *   STEP 2 — format the dynamic status into a buffer
 *   STEP 3 — paint the buffer right-anchored to `cols`
 */
static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_paint_top_left_title();

    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}
/* HUD_PARAM_CELL_WIDTH_* — printed widths of the row-1 cells.  Each
 * cell uses a fixed-width format specifier so the layout doesn't
 * shift as preset / theme names change. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */
#define HUD_PARAM_CELL_WIDTH_GAMMA    13   /* " γ:0.05  k:0.50 " (γ is 1 col) */

/* hud_param — row-1 of the HUD: data cells only.
 *
 *   STEP 1 — PRESET name (which damping value is active)
 *   STEP 2 — THEME  name (which accent ramp is active)
 *   STEP 3 — physics params (γ damping, k spring)
 *   STEP 4 — symbol legend so the viewer can READ the apparatus:
 *            [N]=magnet, @=bob, +=pivot
 */
static void hud_param(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    /* STEP 1 — preset */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " preset:%-8s ", preset_state_active(&s->preset)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    /* STEP 2 — theme */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    /* STEP 3 — γ damping + k spring */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " γ:%.2f  k:%.2f ",
             (double)s->orbit.pendulum.system.damping,
             (double)s->orbit.pendulum.system.spring_k);
    attroff(COLOR_PAIR(PAIR_HUD));
    cursor_x += HUD_PARAM_CELL_WIDTH_GAMMA;

    /* STEP 4 — symbol legend */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " [N]=magnet  @=bob  +=pivot ");
    attroff(COLOR_PAIR(PAIR_HUD));
}
/* hud_hint — bottom row of the HUD: ACTIONS only.  No data, no
 * legend — those live in the top two rows.  This row is reserved
 * for "what KEY does what". */
static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reseed  n/p:preset  t/T:theme  ]/[:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_paint(s, sc->cols, sc->rows);
    hud_top(sc->cols, fps, sim_fps, s);
    hud_param(s);
    hud_hint(sc->rows);
}

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — top-level composition root.
 *
 * INTENT
 *   Owns the Scene (mutable simulation state), the Screen (terminal
 *   adapter), the sim tick rate (sim_fps), the chosen map size, and
 *   the two volatile flags written from signal handlers.  Everything
 *   else in the program is reached through this struct — there is no
 *   other mutable global state.  This makes the program read top-
 *   down: main() = "drive the App"; each helper = "do one named
 *   thing to the App".
 *
 *   Concentrating mutable state in one struct disciplines signal
 *   handling: the SIGWINCH and SIGINT handlers may only touch the
 *   volatile sig_atomic_t fields on g_app — every other piece of
 *   state is reachable only through main-loop functions.
 *
 * CONTEXT
 *   Exactly ONE instance exists: file-scope g_app.  Signal handlers
 *   need a callable-from-signal target, and POSIX async-signal-safety
 *   requires that to be a global; hence g_app.  All non-signal
 *   accesses pass an App* explicitly so the rest of the file remains
 *   free of global lookups.
 *
 *   Lifetime:
 *     app_bootstrap    → fills scene + screen + sim_fps + map dims
 *     main loop        → mutates everything except 'running', which
 *                        is read in the while-condition
 *     atexit(cleanup)  → endwin() safety net
 *
 * MEMBER LOGIC
 *   scene       : §8 Scene — orbit, preset, palette, dims, paused.
 *                 The simulation; "what the user sees".
 *   screen      : §9 Screen — cols / rows + ncurses lifecycle handle.
 *   sim_fps     : current PHYSICS tick rate in Hz.  Distinct from
 *                 RENDER_FPS_TARGET — physics can step at any rate
 *                 while the renderer stays locked at 60 fps.
 *                 Adjusted by ] / [ keys.
 *   map_w       : cells of horizontal SIMULATION area.
 *   map_h       : cells of vertical SIMULATION area (= screen.rows -
 *                 HUD_BAND_RESERVED_ROWS).  Cached on App so
 *                 scene_paint has stable inputs across the frame;
 *                 recomputed by app_pick_map_size on resize.
 *   running     : 0 → main loop exits.  Set by SIGINT/SIGTERM handler
 *                 and by app_handle_key on q/ESC.  Declared
 *                 volatile sig_atomic_t for async-signal safety per
 *                 POSIX.1-2017 §2.4.3.
 *   need_resize : 1 → next frame triggers a SIGWINCH rebuild via
 *                 app_handle_pending_resize.  Set by the SIGWINCH
 *                 handler.  Same volatile sig_atomic_t requirement.
 *
 * REFERENCES (numbered in file-header References block)
 *   [7]  Fiedler "Fix Your Timestep!" — explains why sim_fps and
 *        RENDER_FPS_TARGET are kept as INDEPENDENT knobs on App.
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 11 — SIGWINCH
 *        pattern reproduced by .need_resize + app_handle_pending_resize.
 *   • POSIX.1-2017 §2.4.3 — async-signal safety rules that justify
 *     the volatile sig_atomic_t typing of running / need_resize.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* main_install_signal_handlers — wire SIGINT/TERM → quit, SIGWINCH → resize. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_pick_map_size — clamp terminal dims minus HUD reservation. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16)        mw = 16;
    if (mh < 8)         mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* app_bootstrap — first-time initialisation. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* app_handle_pending_resize — rebuild screen + scene on SIGWINCH. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* app_compute_frame_dt — wall-clock since last frame, capped to
 * prevent the spiral-of-death after a long pause. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler's accumulator pattern: pull
 * fixed-size physics steps until what's left is less than one tick.
 *
 * See header ref [7] Fiedler "Fix Your Timestep!".  Variable-dt
 * would make the orbit NON-DETERMINISTIC across machine speeds —
 * different render rates would sample the magnet attractions at
 * different times, so a slow terminal might land the bob on
 * magnet 2 where a fast terminal lands it on magnet 1.  Fixed dt
 * keeps the visualisation reproducible. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* app_update_fps_meter — refresh fps every FPS_UPDATE_MS. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* app_throttle_to_render_target — sleep so render stays at 60 fps. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — paint scene + HUD and flip the back buffer. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Clamped sim-rate mutators. */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* One-line mutators — named so the binding table reads as intentions. */
static void app_toggle_pause      (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reseed_orbit      (App *app) { active_orbit_release_random_seed(&app->scene.orbit); }

static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
}
static void app_cycle_theme_next(App *app)
{
    palette_state_cycle_next(&app->scene.palette);
    scene_apply_theme(&app->scene);
}
static void app_cycle_theme_prev(App *app)
{
    palette_state_cycle_prev(&app->scene.palette);
    scene_apply_theme(&app->scene);
}

static bool app_handle_key(App *app, int ch);

/* app_poll_keyboard — non-blocking getch + dispatch.  false = quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* app_handle_key — key-binding table; each case calls ONE named mutator. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reseed_orbit     (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    case 'n': case 'N':  app_cycle_preset_next(app); break;
    case 'p': case 'P':  app_cycle_preset_prev(app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — wall-clock + accumulator state threaded through the
 * per-frame helpers.
 *
 * INTENT
 *   Bundle the 5 timing locals that main() would otherwise carry
 *   loose (frame_time, sim_accum, fps_accum, frame_count, fps_display)
 *   into one named concept.  Mirrors the App / Scene pattern: every
 *   piece of mutable state belongs to a NAMED OBJECT, not to main's
 *   stack frame.  The main loop becomes a ~25-line pseudocode driver
 *   over FrameClock + App.
 *
 *   The clock separates TWO independent timescales:
 *     • a wall-clock RENDER timeline (frame_time, fps_*) for what
 *       the user sees,
 *     • a PHYSICS accumulator (sim_accum) for the fixed-step Fiedler
 *       drain inside the loop.
 *   Both update from the same dt sample each frame; the helpers
 *   advance them coherently.
 *
 * CONTEXT
 *   One instance lives on main()'s stack.  Lifetime:
 *     frame_clock_init                 — once before the loop
 *     frame_clock_reset_after_resize   — after each SIGWINCH rebuild
 *     frame_clock_advance              — once per frame
 *     app_drain_fixed_timestep         — reads / mutates .sim_accum
 *     app_update_fps_meter             — reads / mutates .fps_*
 *     app_throttle_to_render_target    — reads .frame_time
 *     app_present_frame                — reads .fps_display
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock timestamp (ns) at the boundary of the
 *                 LAST frame.  app_compute_frame_dt subtracts the
 *                 current clock from this to get dt.
 *   sim_accum   : leftover ns NOT YET consumed by a physics tick.
 *                 The Fiedler fixed-step accumulator (header ref [7]).
 *   fps_accum   : ns of frame time accumulated since the last fps
 *                 readout refresh.
 *   frame_count : number of frames in the current fps window.
 *   fps_display : most recent fps figure, smoothed by the
 *                 FPS_UPDATE_MS window length.  Displayed in HUD row 0.
 *
 * REFERENCES (numbered in file-header References block)
 *   [7] Fiedler "Fix Your Timestep!" — the accumulator pattern that
 *       .sim_accum implements.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}

static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}

static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* main — the whole simulation as a pseudocode driver.
 *
 *   install signal handlers
 *   bootstrap (ncurses + scene)
 *   init frame clock
 *   while running:
 *     handle pending resize
 *     dt ← wall-clock since last frame
 *     advance clock; drain fixed-timestep physics; refresh fps
 *     throttle to render target; present frame
 *     poll keyboard (may flip running)
 *   tear down
 */
int main(void)
{
    main_install_signal_handlers();

    App *app = &g_app;
    app_bootstrap(app);

    FrameClock clk;
    frame_clock_init(&clk);

    while (app->running) {
        if (app->need_resize) {
            app_handle_pending_resize(app);
            frame_clock_reset_after_resize(&clk);
        }

        int64_t dt = app_compute_frame_dt(&clk.frame_time);
        frame_clock_advance(&clk, dt);
        app_drain_fixed_timestep(app, &clk.sim_accum);
        app_update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display);

        app_throttle_to_render_target(clk.frame_time, dt);
        app_present_frame(app, clk.fps_display);

        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
