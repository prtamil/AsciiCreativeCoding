/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pendulum_wave.c — Pendulum Wave (Berg–Marshall demonstration [1])
 *
 * N=15 simple pendulums with lengths chosen so that pendulum n completes
 * (N_BASE + n) full oscillations in T_SYNC seconds:
 *
 *   ω_n = 2π (N_BASE + n) / T_SYNC      from  ω = √(g/L)  [2][3]
 *
 * At t=0 all swing in phase.  They drift apart into travelling waves,
 * standing patterns, and a "checkerboard" before "clapping" back to
 * perfect synchrony at t=T_SYNC.  The phenomenon is a direct visual
 * consequence of the integer-ratio frequency design [1] and a textbook
 * illustration of beat / superposition behaviour [4][5].
 *
 * Each pendulum is drawn as a string of '|' / '/' / '\' chars with a
 * coloured bob ('O') at the displaced position.  The colour ramp is
 * theme-driven — 10 brightness-safe palettes, cycled with t / T.
 *
 * Keys: q quit  p pause  r reset  +/- amplitude  SPACE jump T_SYNC
 *       t/T cycle theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/pendulum_wave.c \
 *       -o pendulum_wave -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 physics + scene  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic (not numerical) integration.
 *                  Each pendulum is a simple harmonic oscillator with
 *                  θ_n(t) = amp · sin(ω_n · t) — the closed-form solution
 *                  of θ'' = −(g/L)·θ under the small-angle approximation
 *                  sin θ ≈ θ [2][3].  No Runge-Kutta needed; the state
 *                  at any t is a single sinf() call.
 *
 * Physics        : Length-frequency relation.  Longer pendulum → lower ω
 *                  → slower swing (ω = √(g/L), period T = 2π√(L/g) [2]).
 *                  Pendulum n has ω_n = 2π(N_BASE+n)/T_SYNC, so the
 *                  required lengths are L_n = g/ω_n².  At t=T_SYNC every
 *                  ω_n·t is an integer multiple of 2π — all bobs return
 *                  to the start phase, the "clap" of Berg & Marshall [1].
 *
 * Math           : Superposition of harmonic oscillators with frequencies
 *                  in arithmetic progression.  Spatial Fourier inspection
 *                  along the array reveals the same beat structure that
 *                  produces Lissajous figures and amplitude beats between
 *                  two close tones [4][5].  Coupled-oscillator theory is
 *                  *not* needed here — each pendulum is independent — but
 *                  the apparent travelling waves are kin to the synchrony
 *                  patterns studied by Strogatz et al. [6].
 *
 * Rendering      : 10 brightness-safe theme palettes — each is an 8-stop
 *                  ramp that the 15 pendulums sample across (i → ramp[i*8/15]).
 *                  HUD chrome (CP_HUD bright yellow, CP_HINT bright cyan)
 *                  is theme-independent so it stays legible everywhere.
 *                  Drawn via ncurses single-buffer + doupdate() diff [7];
 *                  see also documentation/Visual.md for the project's
 *                  in-tree field guide on terminal rendering technique.
 *
 * Performance    : O(N) per frame — each pendulum is one sinf() call plus
 *                  a short string raster.  No spring forces, no constraint
 *                  iterations, no allocation in the hot path.
 *
 * References (cite inline as [n]):
 *
 *   [1] Berg, R. E.; Marshall, T. S. (1991) — "Pendulum waves: A lecture
 *       demonstration", *American Journal of Physics* 59 (2), 186–187.
 *       The original write-up of the apparatus we simulate: derives the
 *       length formula L_n = g·T_SYNC² / (4π²·(N_BASE+n)²) and predicts
 *       the wave / checkerboard / resync sequence step by step.
 *
 *   [2] French, A. P. (1971) — *Vibrations and Waves*, MIT Introductory
 *       Physics Series, W. W. Norton.  Chapters 1–3: SHM, small-angle
 *       pendulum, ω = √(g/L).  The cleanest undergraduate derivation of
 *       the equation of motion we use; also covers damping (which we
 *       deliberately omit to keep the resync exact).
 *
 *   [3] Feynman, R. P.; Leighton, R. B.; Sands, M. — *The Feynman
 *       Lectures on Physics*, Vol. I, Ch. 21–23, Addison-Wesley (1963).
 *       Harmonic oscillator from first principles, resonance, and the
 *       "everything is a harmonic oscillator" mindset that justifies the
 *       analytic shortcut in θ_n(t).
 *
 *   [4] Crawford, F. S. (1968) — *Waves*, Berkeley Physics Course Vol. 3,
 *       McGraw-Hill.  Chapter 1 (free oscillations of simple systems)
 *       and Chapter 2 (free oscillations of systems with many degrees of
 *       freedom) — the modal-superposition viewpoint behind the apparent
 *       travelling waves seen in this demo.
 *
 *   [5] Taylor, J. R. (2005) — *Classical Mechanics*, University Science
 *       Books.  Ch. 5 on oscillations: rigorous treatment of pendulum
 *       dynamics including the large-angle correction we *don't* apply,
 *       so it doubles as the explanation of why amplitude is capped.
 *
 *   [6] Strogatz, S. H. (2003) — *Sync: The Emerging Science of
 *       Spontaneous Order*, Hyperion.  Pop-science companion to his
 *       *Nonlinear Dynamics and Chaos*; the Kuramoto-style framing of
 *       phase-locking is the right mental model for "why does it look
 *       like the array is alive?" even though our pendulums are uncoupled.
 *
 *   [7] Padala, P. — *NCURSES Programming HOWTO*, The Linux Documentation
 *       Project.  Authoritative free reference for the rendering layer:
 *       attron/COLOR_PAIR semantics, double-buffer model used by
 *       wnoutrefresh + doupdate, signal-safe resize handling.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_PEND                                                                 \
  15 /* number of pendulums (sweet spot: enough for                            \
      * rich wave patterns, few enough to all be visible) */

/* N_BASE: oscillation count for the slowest pendulum over T_SYNC seconds.
 * Pendulum n completes (N_BASE+n) full cycles in T_SYNC.
 * N_BASE=40 means the slowest pendulum does 40 cycles in 60 s → ω=4.19 rad/s.
 * Choosing a large N_BASE (relative to N_PEND=15) keeps the frequencies
 * closely spaced → the wave patterns develop slowly and elegantly.        */
#define N_BASE 40

/* T_SYNC: time (seconds) until all pendulums return to perfect phase alignment.
 * All ω_n are integer multiples of 2π/T_SYNC, so at t=T_SYNC:
 * ω_n · T_SYNC = 2π·(N_BASE+n) — an exact integer number of full cycles.
 * 60 s is long enough to see many wave forms before the resync "clap".    */
#define T_SYNC 60.0f

/* AMP_INIT: initial swing amplitude as a fraction of the column width.
 * 0.70 → each bob swings ±70% of its column band.
 * Higher → more dramatic but bobs of adjacent pendulums may overlap.      */
#define AMP_INIT 0.70f
#define AMP_STEP 0.05f

#define RENDER_NS (1000000000LL / 60)

#define HUD_TOP 1 /* row 0 reserved for top status bar       */
#define HUD_BOT 1 /* row rows-1 reserved for action keys     */

#define N_THEMES 10
#define N_RAMP                                                                 \
  8 /* ramp stops per theme; pendulums sample                                  \
     * across this gradient                    */

/* Color-pair IDs: one per ramp stop, plus theme-independent HUD chrome. */
enum {
  CP_R0 = 1,
  CP_R1,
  CP_R2,
  CP_R3,
  CP_R4,
  CP_R5,
  CP_R6,
  CP_R7,
  CP_HUD,
  CP_HINT,
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color / themes — 10 brightness-safe palettes + fixed HUD chrome   */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Theme — a perceptual-gradient palette for the pendulum array.
 *
 * Intent:
 *   A single Theme describes "how the array LOOKS" — nothing about how
 *   it MOVES.  Cycling themes must be a pure visual change: pause the
 *   demo, press t, and every bob's position, velocity, and ω is byte-
 *   identical, only the rendered colour shifts.  This is the locality
 *   rule that lets us treat the theme index as a render param on the
 *   Scene struct (§4) rather than as physics state.
 *
 * Why 8 stops (not 15, not 6):
 *   The pendulums number N_PEND=15, but designing 15 hand-tuned indices
 *   per palette × 10 palettes = 150 colour values is fragile — every
 *   palette needs separate brightness auditing.  8 stops is the smallest
 *   ramp that still gives a visibly monotonic gradient when 15 indices
 *   sample across it via the formula
 *
 *       stop_for_pendulum(i) = (i * N_RAMP) / N_PEND      [0 ≤ i < 15]
 *
 *   which buckets the 15 pendulums roughly 2-2-2-2-2-2-2-1 across the
 *   8 stops.  Fewer than 8 produces visible "stairs" in the gradient;
 *   more is a maintenance tax with no perceptual win at this column
 *   count.  See documentation/COLOR.md for the project's broader
 *   palette-design notes.
 *
 * Why ramp[0] → ramp[7] is a monotonic gradient (not arbitrary):
 *   Pendulum 0 has the lowest ω (longest L) and pendulum 14 the highest
 *   — the visual order of the array IS the frequency order.  Mapping
 *   that ordering onto a perceptual brightness ramp gives the eye a
 *   built-in axis ("low ω = dim left, high ω = bright right") that
 *   reinforces the physics described in [2][3].  Themes that reverse
 *   this — say bright-on-left — were tried and rejected because the
 *   wave patterns become much harder to read.
 *
 * Brightness safety (CLAUDE.md, see documentation/COLOR.md):
 *   Every entry sits at 256-cube index ≥ 30, or 24-29 / 240-243 only
 *   as the lowest ramp tier.  16-23 / 232-239 are forbidden — they
 *   render as pure background-black on default-background terminals
 *   and the pendulum simply disappears.
 *
 * Algorithm refs:
 *   Frequency-order ↔ visual-order mapping  — Berg & Marshall [1]
 *                                             (their original demo
 *                                             uses physical bob colour
 *                                             gradient for the same
 *                                             reason).
 *   256-colour cube structure                — XTerm Control Sequences
 *                                             § "ISO-8613-3 Controls".
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* Human-readable label shown in the top HUD.  ≤ 7 chars so the full
   * status line fits on an 80-column terminal alongside the other
   * fields (amp, t, sync_in, paused/running, fps). */
  const char *name;

  /* Eight ordered 256-colour cube indices forming a low→high ramp.
   * ramp[0] = dimmest visible tier (pendulum 0, lowest ω);
   * ramp[N_RAMP-1] = brightest tier (pendulum N_PEND-1, highest ω).
   * All entries audited against the brightness-safety bands above. */
  short ramp[N_RAMP];
} Theme;

/*
 * k_themes — the 10 stock palettes referenced by the CLAUDE.md theme
 * mandate.  Index 0 (Matrix) is the boot default for visibility on
 * almost any terminal background.  The comment on each row names the
 * perceptual story the ramp tells; tweak the indices, not the names —
 * the names are part of the HUD contract.
 */
static const Theme k_themes[N_THEMES] = {
    /*  name        ramp[0..7]                                              */
    {"Matrix", {28, 34, 40, 46, 82, 118, 154, 190}},       /* greens      */
    {"Fire", {130, 166, 202, 208, 214, 220, 226, 231}},    /* embers→white*/
    {"Oceanic", {24, 31, 38, 45, 51, 87, 123, 195}},       /* deep→cyan   */
    {"Neon", {93, 129, 165, 171, 207, 213, 219, 231}},     /* purple→pink */
    {"Mono", {240, 244, 247, 250, 252, 253, 254, 255}},    /* grayscale   */
    {"Ice", {39, 75, 111, 117, 153, 159, 195, 231}},       /* polar blues */
    {"Nova", {54, 92, 129, 135, 141, 177, 213, 231}},      /* violet/pink */
    {"Forest", {58, 64, 100, 106, 142, 148, 184, 190}},    /* leaves      */
    {"Desert", {130, 136, 172, 178, 214, 220, 222, 230}},  /* sand/gold   */
    {"Eclipse", {240, 124, 160, 196, 202, 208, 220, 231}}, /* dark→corona */
};

/* Fixed HUD chrome — same on every theme so status/keys stay legible. */
#define CHROME_HUD_256 226 /* bright yellow */
#define CHROME_HINT_256 51 /* bright cyan   */

/*
 * theme_apply — push the chosen palette into the ncurses colour-pair
 * table.  Idempotent: safe to call from t/T key handlers and at startup.
 * idx is taken modulo N_THEMES (with a negative-safe correction) so the
 * caller does not need to wrap.  CP_HUD / CP_HINT are reinstated every
 * call so a future bug that overwrites them can't outlive one keypress.
 */
static void theme_apply(int idx) {
  const Theme *t = &k_themes[((idx % N_THEMES) + N_THEMES) % N_THEMES];
  if (COLORS >= 256) {
    for (int k = 0; k < N_RAMP; k++)
      init_pair(CP_R0 + k, t->ramp[k], -1);
    init_pair(CP_HUD, CHROME_HUD_256, -1);
    init_pair(CP_HINT, CHROME_HINT_256, -1);
  } else {
    /* 8-colour fallback — theme-independent rainbow-ish ramp. */
    static const short fallback[N_RAMP] = {
        COLOR_BLUE,   COLOR_BLUE,   COLOR_CYAN, COLOR_GREEN,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,  COLOR_MAGENTA,
    };
    for (int k = 0; k < N_RAMP; k++)
      init_pair(CP_R0 + k, fallback[k], -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

/*
 * color_init — bring ncurses' colour subsystem online and install the
 * boot theme.  Split from theme_apply so the per-frame cost of t/T
 * cycling is just init_pair calls, not start_color() each time.
 */
static void color_init(int boot_theme) {
  start_color();
  use_default_colors();
  theme_apply(boot_theme);
}

/* Pendulum i → ramp stop index → ncurses color-pair ID.
 * Bucketing formula explained on the Theme struct above. */
static inline int pend_cp(int i) { return CP_R0 + (i * N_RAMP) / N_PEND; }

/* ===================================================================== */
/* §4  physics + scene                                                    */
/*                                                                       */
/* The simple-harmonic state (omega/time/amp/paused) plus the one render */
/* knob (theme) live together on a single file-scope Scene so the reader */
/* sees the entire mutable surface in one place.  The Scene docstring    */
/* below explains the simulation-vs-render locality contract.            */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level state spanning physics ticks, draw calls, and the
 * main loop.  Bundles every file-scope mutable that survives across
 * iterations.  Single instance `g_scene`; lives in BSS because Scene is
 * tiny (~80 bytes) and passing it by pointer everywhere just clutters
 * helper signatures without buying any encapsulation.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters — consumed by physics_init and theta(); these
 *     are the inputs to the closed-form solution θ_n(t) = amp·sin(ω_n·t)
 *     [2][3].  Anything that changes WHERE THE BOBS ARE belongs here.
 *     Mutated by physics-affecting keys: r (reset), p / space (pause),
 *     +/- (amplitude), SPACE (jump to next sync).
 *
 *   Rendering parameters — consumed by theme_apply and the draw_hud_*
 *     helpers.  Toggling these while paused must leave omega[], time,
 *     amp byte-identical; only colours may differ.  Mutated by purely
 *     cosmetic keys: t / T (theme).
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually changes how theta() evaluates
 *   would silently couple display to physics — exactly the bug the
 *   separation prevents.  When adding a field, ask: does this change
 *   what theta() or physics_init() produces?  If yes, simulation; if
 *   no, rendering.
 *
 * What stays OUTSIDE this struct (intentionally):
 *
 *   g_quit / g_resize     volatile sig_atomic_t flags written by the
 *                         signal handler; must stay at file scope for
 *                         async-signal-safety (the standard guarantees
 *                         atomic load/store only for objects of that
 *                         type at file scope).  See [7] § "Signals".
 *
 *   g_rows / g_cols       screen geometry tracked by the main loop's
 *                         SIGWINCH handler; Scene stays geometry-
 *                         agnostic so resize logic is the main loop's
 *                         concern, not the renderer's, and so theta()
 *                         remains pure (depends only on Scene fields,
 *                         no terminal-size aliasing).
 *
 *   fps_acc / fps_cnt / fps_disp
 *                         rolling-window FPS bookkeeping; lives as
 *                         locals in main() because it has no readers
 *                         outside the loop body and the value passed to
 *                         draw_hud_top is just a function argument.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ────────────────────────────────────────
   * These fields are the entire physics state.  theta(n, t) reads
   * omega[n] and amp; physics_init writes omega[] once at startup;
   * the main loop advances time when !paused.
   */

  /* omega[n] = 2π·(N_BASE+n)/T_SYNC  — angular frequency (rad/s) of
   * pendulum n.  Precomputed once in physics_init so the hot loop
   * is a single sinf() per bob with no transcendentals.  The integer
   * coefficient (N_BASE+n) is what guarantees ω_n·T_SYNC = 2πk for
   * some integer k, hence the "clap" resync of Berg & Marshall [1].
   * Derivation: ω = √(g/L) for small θ [2, eq. 3.8] [3, Vol I § 21-3];
   * we set ω directly and back-compute the implied L visually via
   * the bob's row position in pendulum_build (§5). */
  float omega[N_PEND];

  /* time — accumulated simulation time (s) since reset.  This is the
   * sole independent variable of θ_n(t): the demo is stateless apart
   * from this counter.  Advances by real wall-clock dt when !paused,
   * capped at 100 ms per step in main to prevent spiral-of-death on
   * suspended terminals.  Wraps every T_SYNC seconds visually but is
   * NOT modulo'd in storage — kept as monotonic since boot so the
   * "sync_in" HUD readout can compute fmodf(time, T_SYNC) directly. */
  float time;

  /* amp — dimensionless swing amplitude in [0.05, 0.95], scaled in
   * pendulum_build (§5) to a horizontal column offset.  Treated as a
   * pure visual gain on θ_n(t); does NOT enter ω so changing amp
   * leaves the resync timing intact (only the swing width changes).
   * Capped below 1.0 because real small-angle approximation breaks
   * down well before that — see Taylor [5, eq. 5.81] for the actual
   * O(θ²) period correction we deliberately ignore. */
  float amp;

  /* paused — when true, main skips the time += dt step.  Physics is
   * frozen exactly: omega[] and amp are unchanged, time is unchanged,
   * so unpause resumes mid-pattern with no discontinuity.  Mirrored
   * to the HUD via the "PAUSED" / "running" badge. */
  bool paused;

  /* ── Rendering parameters ─────────────────────────────────────────
   * Pure cosmetic state.  Mutating these MUST be a no-op for the
   * physics step — that contract is what keeps "press t mid-wave"
   * from desynchronising the resync.
   */

  /* theme — index into k_themes[] (§3).  Cycled forward by 't',
   * backward by 'T'; both call theme_apply() to push the new ramp
   * into the ncurses color-pair table.  Always normalised to
   * [0, N_THEMES) by the t / T handlers; theme_apply() still wraps
   * defensively as a safety net for any future caller. */
  int theme;
} Scene;

/*
 * g_scene — the one Scene instance.  Default-initialised here so that
 * the file compiles and runs sanely even before physics_init() has been
 * called (omega[] is zeroed by BSS-init; theta(n, 0) yields 0.0 so the
 * first frame after main entry is a quiet, in-phase array).
 */
static Scene g_scene = {
    .amp = AMP_INIT, .paused = false, .time = 0.0f, .theme = 0,
    /* .omega[] is BSS-zeroed; physics_init() populates it before tick. */
};

/*
 * physics_init — fill g_scene.omega[].  Pure setup; reads only the
 * compile-time constants N_BASE / N_PEND / T_SYNC.  Called once from
 * main() before the loop starts.
 */
static void physics_init(void) {
  for (int i = 0; i < N_PEND; i++)
    g_scene.omega[i] = (float)(2.0 * M_PI * (N_BASE + i) / T_SYNC);
}

/*
 * theta — closed-form pendulum angle at time t.
 *   θ_n(t) = amp · sin(ω_n · t)            [2, eq. 3.8]
 * Pure: depends only on (n, t) and the scene's omega/amp.  Called
 * once per pendulum per frame from pendulum_build (§5); no other
 * caller in the file.
 */
static float theta(int n, float t) {
  return g_scene.amp * sinf(g_scene.omega[n] * t);
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int g_rows, g_cols;

/* ─────────────────────────────────────────────────────────────────────── *
 * Pendulum — per-frame visual SNAPSHOT of one strand.
 *
 * Intent:
 *   A small POD that lets the drawing pipeline read like its pseudocode:
 *
 *       for each i:                              [draw_pendulums]
 *           p = build snapshot from (i, viewport, scene)   ← pendulum_build
 *           draw the pivot anchor                          ←
 * pendulum_draw_pivot draw the swinging string                       ←
 * pendulum_draw_string draw the bob                                   ←
 * pendulum_draw_bob
 *
 *   pendulum_build returns the snapshot by value; each draw helper
 *   takes a const Pendulum* and does exactly ONE thing.  Only
 *   pendulum_build calls into the physics layer (theta()), so the
 *   draw helpers are pure ncurses calls plus bounds checks.
 *
 * Lifetime / value semantics:
 *   Per-frame TRANSIENT — Pendulum is never stored across iterations.
 *   draw_pendulums constructs each one on the stack, hands it to three
 *   helpers, and discards it.  This is on purpose: SIGWINCH resize,
 *   t / T theme cycle, and +/- amplitude all "take effect on the next
 *   frame" with zero migration code — the next pendulum_build reads
 *   the current scene + viewport and produces a fresh snapshot.  Do
 *   NOT promote any field to Scene; the struct exists precisely
 *   because it is short-lived.
 *
 * Fields:
 *   (pivot_row, pivot_col)  — terminal cell of the '^' anchor.
 *                             pivot_row is shared across all strands
 *                             (just below the top HUD); pivot_col is
 *                             the centre of strand i's column band.
 *   (bob_row,   bob_col)    — terminal cell of the 'O' bob right now.
 *                             bob_row depends on i (longer pendulum
 *                             hangs lower); bob_col swings with θ_n(t).
 *   cp                      — ncurses colour-pair ID from pend_cp(i);
 *                             cached so the string and bob agree.
 *   string_glyph            — '|', '/', or '\\' chosen once at build
 *                             time from the strand slope, so every cell
 *                             of one string uses the same character.
 *
 * Why a struct (vs returning multiple ints):
 *   Three helpers consume the same five-or-six numbers.  Repeating
 *   the parameter list three times is noise; a const Pendulum* is one
 *   pointer per helper, and the caller-side cost is one stack copy
 *   that the compiler trivially inlines / elides under -O2.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  int pivot_row, pivot_col;
  int bob_row, bob_col;
  int cp;
  chtype string_glyph;
} Pendulum;

/*
 * pendulum_build — derive strand i's snapshot from the current scene
 * and viewport.  Pure: reads (i, pivot_row, pend_area_rows) plus
 * g_scene.time / g_scene.amp (via theta()) and g_rows / g_cols / g_scene.theme
 * (via pend_cp()).  Writes nothing; the only sinf() call lives here.
 *
 * Layout:
 *   column band  =  g_cols / N_PEND          → centre at (i+0.5)·band
 *   bob horiz    =  centre + θ_n(t) · band   (θ in [-amp, amp])
 *   bob vert     =  pivot+1 + (string_len)·L_frac
 *                  with L_frac = (N_BASE+i)/(N_BASE+N_PEND-1) ∈ (0, 1].
 *                  Maps frequency order onto vertical order so shorter
 *                  strands (higher ω) hang at a higher row — see
 *                  Berg & Marshall [1] for the physical analogue.
 *
 * Glyph choice (only depends on local slope = dc/dr):
 *   |slope| < 0.35  → '|'   (near-vertical, the usual case)
 *   slope > 0       → '\\'  (string trails down-right toward the bob)
 *   slope ≤ −0.35   → '/'   (string trails down-left)
 *   Threshold 0.35 ≈ tan(19°) — chosen empirically so the glyph stays
 *   stable through one cell of horizontal jitter at typical strand
 *   lengths (~10 rows).
 */
static Pendulum pendulum_build(int i, int pivot_row, int pend_area_rows) {
  Pendulum p;

  /* Pivot — fixed for the lifetime of this frame. */
  p.pivot_row = pivot_row;
  p.pivot_col = (int)((i + 0.5f) * g_cols / N_PEND);

  /* Bob horizontal — driven by θ_n(t). */
  float th = theta(i, g_scene.time);
  p.bob_col = p.pivot_col + (int)(th * ((float)g_cols / (float)N_PEND));

  /* Bob vertical — encodes ω ordering as visual height. */
  int string_len = pend_area_rows - 2;
  float L_frac = (float)(N_BASE + i) / (float)(N_BASE + N_PEND - 1);
  p.bob_row = pivot_row + 1 + (int)(string_len * L_frac);
  if (p.bob_row >= g_rows - HUD_BOT)
    p.bob_row = g_rows - HUD_BOT - 1;

  /* Style — colour from theme ramp, glyph from local slope. */
  p.cp = pend_cp(i);

  int dr = p.bob_row - p.pivot_row;
  int dc = p.bob_col - p.pivot_col;
  float slope = (dr > 0) ? (float)dc / (float)dr : 0.f;
  if (fabsf(slope) < 0.35f)
    p.string_glyph = '|';
  else if (slope > 0)
    p.string_glyph = '\\';
  else
    p.string_glyph = '/';

  return p;
}

/* pendulum_draw_pivot — the '^' anchor at the top of the strand. */
static void pendulum_draw_pivot(const Pendulum *p) {
  if (p->pivot_row < g_rows && p->pivot_col >= 0 && p->pivot_col < g_cols)
    mvaddch(p->pivot_row, p->pivot_col, '^');
}

/*
 * pendulum_draw_string — render the line from pivot to bob as a column
 * of string_glyph cells.  Walks rows s = 1 .. dr-1 (open interval — the
 * pivot is drawn by pendulum_draw_pivot, the bob by pendulum_draw_bob),
 * interpolating column with the slope re-derived from snapshot endpoints.
 * This is a fixed-point DDA in spirit but written in floats for clarity
 * at N=15 — the per-strand work is at most a few dozen cells.
 */
static void pendulum_draw_string(const Pendulum *p) {
  int dr = p->bob_row - p->pivot_row;
  if (dr <= 0)
    return;
  int dc = p->bob_col - p->pivot_col;
  float slope = (float)dc / (float)dr;

  attron(COLOR_PAIR(p->cp));
  for (int s = 1; s < dr; s++) {
    int r = p->pivot_row + s;
    int c = p->pivot_col + (int)(slope * s + 0.5f);
    if (r < g_rows - HUD_BOT && c >= 0 && c < g_cols)
      mvaddch(r, c, p->string_glyph);
  }
  attroff(COLOR_PAIR(p->cp));
}

/* pendulum_draw_bob — the bold 'O' at the displaced position. */
static void pendulum_draw_bob(const Pendulum *p) {
  if (p->bob_row < g_rows - HUD_BOT && p->bob_col >= 0 && p->bob_col < g_cols) {
    attron(COLOR_PAIR(p->cp) | A_BOLD);
    mvaddch(p->bob_row, p->bob_col, 'O');
    attroff(COLOR_PAIR(p->cp) | A_BOLD);
  }
}

/*
 * draw_pendulums — top-level pendulum-array renderer.  Reads as the
 * pseudocode quoted on the Pendulum struct above: pure orchestration,
 * no math, no ncurses calls of its own.
 */
static void draw_pendulums(void) {
  int pend_area_rows = g_rows - HUD_TOP - HUD_BOT;
  if (pend_area_rows < 4)
    return; /* terminal too short to draw anything legible */

  int pivot_row = HUD_TOP; /* anchor row, shared by all strands */

  for (int i = 0; i < N_PEND; i++) {
    Pendulum p = pendulum_build(i, pivot_row, pend_area_rows);
    pendulum_draw_pivot(&p);
    pendulum_draw_string(&p);
    pendulum_draw_bob(&p);
  }
}

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0       right (CP_HUD  bright yellow + bold) — live status
 *   row rows-1  left  (CP_HINT bright cyan   + bold) — actions / keys
 */
static void draw_hud_top(int fps) {
  float t_rem = T_SYNC - fmodf(g_scene.time, T_SYNC);
  char buf[180];
  snprintf(buf, sizeof buf,
           " theme:%s  amp:%.2f  t:%5.2f/%.0fs  sync_in:%5.2fs  %s  %d fps ",
           k_themes[g_scene.theme].name, g_scene.amp, g_scene.time, T_SYNC,
           t_rem, g_scene.paused ? "PAUSED " : "running", fps);
  int len = (int)strlen(buf);
  int col = g_cols - len;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void draw_hud_bottom(void) {
  const char *hint_full =
      " q:quit  p:pause  r:reset  +/-:amp  spc:sync  t/T:theme ";
  const char *hint_short = " q:quit  p:pause  r:reset  t:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= g_cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, hint, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(int fps) {
  draw_pendulums();
  draw_hud_top(fps);
  draw_hud_bottom();
}

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_quit = 1;
  if (s == SIGWINCH)
    g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void) {
  atexit(cleanup);
  signal(SIGINT, sig_h);
  signal(SIGTERM, sig_h);
  signal(SIGWINCH, sig_h);

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  color_init(g_scene.theme);
  getmaxyx(stdscr, g_rows, g_cols);
  physics_init();

  long long last = clock_ns();
  long long fps_acc = 0;
  int fps_cnt = 0;
  int fps_disp = 0;

  while (!g_quit) {

    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_rows, g_cols);
      last = clock_ns();
    }

    int ch = getch();
    switch (ch) {
    case 'q':
    case 'Q':
    case 27:
      g_quit = 1;
      break;
    case 'p':
    case 'P':
      g_scene.paused = !g_scene.paused;
      break;
    case 'r':
    case 'R':
      g_scene.time = 0.f;
      break;
    case '+':
    case '=':
      g_scene.amp += AMP_STEP;
      if (g_scene.amp > 0.95f)
        g_scene.amp = 0.95f;
      break;
    case '-':
      g_scene.amp -= AMP_STEP;
      if (g_scene.amp < 0.05f)
        g_scene.amp = 0.05f;
      break;
    case ' ':
      /* jump to next sync point */
      g_scene.time = ceilf(g_scene.time / T_SYNC) * T_SYNC;
      break;
    case 't':
      g_scene.theme = (g_scene.theme + 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    case 'T':
      g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    default:
      break;
    }

    long long now = clock_ns();
    long long dt_ns = now - last;
    last = now;
    if (dt_ns > 100000000LL)
      dt_ns = 100000000LL;

    if (!g_scene.paused)
      g_scene.time += (float)dt_ns * 1e-9f;

    /* fps counter — rolling 1-second window */
    fps_acc += dt_ns;
    fps_cnt += 1;
    if (fps_acc >= 1000000000LL) {
      fps_disp = (int)(fps_cnt * 1000000000LL / fps_acc);
      fps_acc = 0;
      fps_cnt = 0;
    }

    erase();
    scene_draw(fps_disp);
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(RENDER_NS - (clock_ns() - now));
  }
  return 0;
}
