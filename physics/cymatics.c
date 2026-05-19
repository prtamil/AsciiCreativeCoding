/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cymatics.c — Chladni Figures (2-D Standing-Wave Nodal Patterns)
 *
 * A Chladni figure is the pattern of sand on a plate vibrating at a
 * resonant frequency.  Sand collects on the nodal lines where the
 * displacement is zero.
 *
 * Formula for mode (m, n):
 *   Z(x,y) = cos(m·π·x)·cos(n·π·y)  −  cos(n·π·x)·cos(m·π·y)
 *   x, y ∈ [0,1]
 *
 * Rendering:
 *   |Z| ≈ 0  → nodal line  (bright chars: @ # * + . depending on closeness)
 *   Z  > 0   → positive antinode  (dim, theme colour A)
 *   Z  < 0   → negative antinode  (dim, theme colour B)
 *
 * Animation: the figure slowly morphs to the next (m,n) mode by blending
 * Z values, then holds the complete figure before advancing.
 *
 * Themes (t key):
 *   Classic   white nodal / cyan+ / red−
 *   Ocean     white nodal / blue+ / teal−
 *   Ember     white nodal / orange+ / red−
 *   Neon      white nodal / green+ / magenta−
 *
 * Keys:
 *   n / p     next / previous mode
 *   t         next colour theme
 *   a         toggle auto-advance
 *   + / -     shorten / lengthen hold time (showcase ↔ slow)
 *   space     pause / resume
 *   q / Q     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/cymatics.c -o cymatics \
 *       -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 field math
 *           §5 scene (state + mutators)  §6 rendering  §7 screen + app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic Chladni figure computation — no simulation, no PDE.
 *                  The nodal pattern is computed by evaluating the 2D mode
 * shape function at every cell and testing if |Z| < threshold.
 *
 * Physics        : Chladni figures (Ernst Chladni, 1787): when a plate vibrates
 *                  at a resonant frequency, sand on the plate migrates to nodal
 *                  lines (where displacement = 0).  The mode (m,n) describes
 *                  how many half-wavelengths fit in each direction.
 *
 * Math           : Square plate mode shape function:
 *                    Z(x,y) = cos(m·π·x)·cos(n·π·y) − cos(n·π·x)·cos(m·π·y)
 *                  Nodal lines are where Z(x,y) = 0.  The cos−cos structure
 *                  comes from satisfying Neumann boundary conditions (free
 * edge). Resonant frequency: f_mn ∝ √(m² + n²) — Pythagorean relationship.
 *                  Modes with m=n are degenerate: Z=0 everywhere (trivial
 * solution); interesting patterns require m ≠ n.
 *
 * Rendering      : Analytic band-thresholding — evaluate Z at every cell,
 *                  bucket |Z| into 5 bands (0.04, 0.10, 0.18, 0.28, 0.40),
 *                  pick a glyph from the density ramp '@#*+.' (closest to
 *                  nodal → faintest antinode), colour by sign(Z) for the
 *                  outer bands and white for the inner nodal core.  No
 *                  marching-squares, no contour-line tracing — we just
 *                  shade the field directly, and the human eye reads the
 *                  resulting bands as nodal curves.
 *
 * References     :
 *   • Chladni, E. F. F. (1787) "Entdeckungen über die Theorie des
 *     Klanges" (Discoveries Concerning the Theory of Music), Leipzig.
 *     — The original.  Sand-on-plate experiments that named the
 *       figures.  Demonstrated to Napoleon in 1809.
 *
 *   • Kirchhoff, G. (1850) "Über das Gleichgewicht und die Bewegung
 *     einer elastischen Scheibe", J. Reine Angew. Math. 40, 51-88.
 *     — First rigorous PDE for plate vibration: the biharmonic
 *       equation  ∇⁴w + (ρh/D) ∂²w/∂t² = 0  with free-edge BCs.  The
 *       real physics our analytic cos·cos formula approximates.
 *
 *   • Rayleigh, Lord (J. W. Strutt) (1894-1896) "The Theory of Sound"
 *     (2 vols), Macmillan.
 *     — The classical encyclopedia of acoustic vibration.  Vol. I
 *       §195-227 treats plate modes in detail.
 *
 *   • Leissa, A. W. (1969) "Vibration of Plates", NASA SP-160.
 *     — The reference compendium.  Tables of mode shapes for every
 *       plate geometry and boundary condition; you can look up (m,n)
 *       and read off the expected figure.
 *
 *   • Graff, K. F. (1991) "Wave Motion in Elastic Solids", Dover.
 *     — Clean pedagogical treatment.  Chapter on plate waves is the
 *       cleanest derivation of the mode-shape formula we use here.
 *
 *   • Morse, P. M. & Ingard, K. U. (1968) "Theoretical Acoustics",
 *     McGraw-Hill (Princeton paperback reprint, 1986).
 *     — Formal solution of the 2D wave equation on bounded domains.
 *       Where the cos(mπx)·cos(nπy) eigenfunctions come from.
 *
 *   • Jenny, H. (1967) "Cymatics: The Study of Wave Phenomena and
 *     Vibration", MACROmedia.
 *     — The book that named the field "cymatics".  Photo-rich; less
 *       rigorous, more visual — every figure on the screen has a
 *       photographic twin in here.
 *
 *   • Gander, M. J. & Kwok, F. (2012) "Chladni Figures and the
 *     Tacoma Bridge: Motivating PDE Eigenvalue Problems via
 *     Vibrations", SIAM Review 54(3), 573-596.
 *     — Modern pedagogical paper using Chladni to teach PDE
 *       eigenvalue problems.  Great bridge from this demo's analytic
 *       formula to the underlying biharmonic eigenvalue problem.
 *
 *   • Tuan, P. H., Wen, C. P., Chiang, P. Y., Yu, Y. T., Liang, H. C.,
 *     Huang, K. F., Chen, Y. F. (2015) "Exploring the resonant
 *     vibration of thin plates: reconstruction of Chladni patterns
 *     and determination of resonant wave numbers", J. Acoust. Soc.
 *     Am. 137(4), 2113-2123.
 *     — Modern experimental reconstruction.  Reconciles the analytic
 *       cos·cos approximation with high-resolution measurements.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A Chladni figure is just a contour plot of one analytic function:
 * Z(x, y) = cos(mπx)·cos(nπy) - cos(nπx)·cos(mπy).  Wherever Z ≈ 0 the
 * sand piles up; wherever Z is positive or negative the plate is
 * vibrating up or down (an antinode).  No simulation, no PDE solver,
 * no time-stepping for the physics — we just evaluate Z at every cell
 * each frame.  The animation is a simple linear blend between the
 * current mode's Z and the next mode's Z, with t walking from 0 to 1.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Each tick (scene_tick):
 *       if paused → return.
 *       if state == ST_HOLD and auto: hold_ctr++; if ≥ hold_ticks,
 *         enter ST_MORPH with t=0.
 *       if state == ST_MORPH: t += MORPH_SPEED.  When t ≥ 1, advance
 *         mode_idx by 1 (mod N_MODES), reset t = 0, return to HOLD.
 *  2. Each frame, for every cell (r, c) inside the bowl region:
 *       Map (r, c) → (x, y) ∈ [0, 1]².
 *       z1 = chladni_z(x, y, m_cur, n_cur)
 *       z  = z1   if t == 0
 *          = (1-t)·z1 + t·z2  otherwise (z2 from next mode)
 *  3. Compute az = |z|.  Five threshold bands:
 *       az < 0.04 → '@' (nodal core, white, A_BOLD)
 *       az < 0.10 → '#' (close to nodal, A_BOLD)
 *       az < 0.18 → '*' (penumbra, A_DIM, theme colour)
 *       az < 0.28 → '+' (faint antinode glow, A_DIM)
 *       az < 0.40 → '.' (very faint, A_DIM)
 *       else      → ' ' (blank — pure antinode)
 *  4. Glyphs in the inner 2 bands paint with CP_NODE (white).  Outer
 *     bands paint with CP_POS or CP_NEG depending on sign(z).
 *  5. Top HUD (row 0): current → next (m, n), state, t, hold seconds,
 *     theme, auto/manual, paused/running.  Bottom HUD (row -1): action
 *     key list (q / spc / n / p / t / a / +/-).
 *
 * KEY FORMULAS
 * ────────────
 *  Mode shape       Z(x,y) = cos(mπx)·cos(nπy) - cos(nπx)·cos(mπy)
 *  Symmetry         Z(y,x) = -Z(x,y)         (anti-symmetric in x↔y)
 *  Resonance        f_{m,n} ∝ √(m² + n²)
 *  Cell→plate       x = c / (cols - 1),  y = r / (rows - 1)
 *  Morph blend      Z(x,y,t) = (1-t)·Z_cur + t·Z_next     t ∈ [0,1]
 *  Nodal test       |Z| < NODAL_THRESH  → near-zero displacement
 *  Band thresholds  0.04, 0.10, 0.18, 0.28, 0.40 → glyphs @ # * + .
 *  Hold then morph  state oscillates HOLD (hold_ticks default 120 ≈ 4 s,
 *                   tunable [0..300] via +/-) → MORPH (1/MORPH_SPEED =
 *                   40 ticks ≈ 1.3 s) → HOLD …
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS 33333333LL
#define HOLD_DEFAULT 120   /* ~4 s default hold before morphing   */
#define HOLD_MIN 0         /* +/- floor — continuous showcase     */
#define HOLD_MAX 300       /* +/- ceiling — ~10 s dwell per mode  */
#define HOLD_STEP 15       /* +/- increment, ~0.5 s per press     */
#define MORPH_SPEED 0.025f /* t increment per tick (~1.3 s morph) */
#define NODAL_THRESH 0.04f /* |Z| < this → centre nodal char      */

enum { ST_HOLD = 0, ST_MORPH };
enum { CP_POS = 1, CP_NEG, CP_NODE, CP_HUD, CP_HINT };

/* ─────────────────────────────────────────────────────────────────────
 * MODES — table of Chladni mode pairs (m, n) the demo cycles through.
 *
 * WHAT IS A MODE?
 * ───────────────
 * On a vibrating square plate, every resonant frequency excites one
 * specific standing-wave shape.  The shape for one mode is given by
 *
 *     Z(x, y) = cos(mπx)·cos(nπy)  −  cos(nπx)·cos(mπy)
 *
 * where m and n are positive integers and (x, y) ∈ [0,1]² spans the
 * plate.  m counts the number of half-wavelengths from edge to edge
 * along x (m=3 → three pressure maxima); n does the same along y.
 * Sand poured on the vibrating plate falls away from cells where |Z|
 * is large (the antinodes, vibrating up/down) and collects on the
 * nodal lines where Z ≈ 0.  Hence "a Chladni figure for mode (m, n)"
 * = the curve set of Z(x, y) = 0.
 *
 * WHY 1 ≤ m < n  (no m = n, no m > n)
 * ────────────────────────────────────
 *   m = n :  Z(x,y) = cos(mπx)cos(mπy) − cos(mπx)cos(mπy) = 0
 *            EVERYWHERE.  Degenerate — would render a blank plate.
 *            Skipped.
 *
 *   m > n :  Swap labels.  The minus sign in the formula gives
 *            Z(m,n) = −Z(n,m), and since sand cares only about |Z|,
 *            the nodal pattern of (m, n) is IDENTICAL to (n, m) — same
 *            figure modulo a sign flip on the antinode tints.  We
 *            list only m < n to avoid drawing the same figure twice.
 *
 * WHICH PAIRS ARE IN THE TABLE (40 figures)
 * ──────────────────────────────────────────
 * Every (m, n) with 1 ≤ m < n ≤ 9, plus four pairs from n = 10:
 *
 *     n = 2 :  (1,2)                                                1
 *     n = 3 :  (1,3) (2,3)                                          2
 *     n = 4 :  (1,4) (2,4) (3,4)                                    3
 *     n = 5 :  (1,5) (2,5) (3,5) (4,5)                              4
 *     n = 6 :  (1,6) (2,6) (3,6) (4,6) (5,6)                        5
 *     n = 7 :  (1,7) (2,7) (3,7) (4,7) (5,7) (6,7)                  6
 *     n = 8 :  (1,8) (2,8) (3,8) (4,8) (5,8) (6,8) (7,8)            7
 *     n = 9 :  (1,9) (2,9) (3,9) (4,9) (5,9) (6,9) (7,9) (8,9)      8
 *     n = 10:  (1,10) (2,10) (3,10) (4,10)                          4
 *                                                                  ──
 *                                                         total  =  40
 *
 * The n = 2..9 triangle gives 1+2+...+8 = 36 pairs (the full Pascal
 * column m < n for n ≤ 9); four entries from n = 10 round the count
 * to 40 and give the demo a stripe of fine-lattice "showcase" modes.
 *
 * VISUAL CHARACTER PER RANGE
 * ──────────────────────────
 *   n = 2 - 4   Classic figures.  Few nodal curves, easy to read.
 *               (1,2) → a single + cross (one horizontal + one vertical
 *                       nodal line);
 *               (2,3) → wavy intersections, three nodal arcs;
 *               (3,4) → the iconic "star" Chladni figure.
 *
 *   n = 5 - 7   Moderately intricate.  Five to twelve nodal arcs; the
 *               pattern starts to feel "woven".  Press 'n' here to see
 *               the patterns Chladni's original 1787 plates produced.
 *
 *   n = 8 - 10  Fine lattice.  Twenty to thirty nodal curves on a
 *               typical terminal.  Drop hold time with '+' for a
 *               continuous showcase morph through these.
 *
 * RESONANT FREQUENCY RELATIONSHIP
 * ───────────────────────────────
 * For the simply-supported approximation (the cos·cos analytic form
 * used here), the resonant frequency of mode (m, n) scales as
 *
 *     f_{m,n} ∝ √(m² + n²)
 *
 * so the (3, 4) figure rings at f ∝ 5 (a Pythagorean 3-4-5 triple),
 * (1, 2) at f ∝ √5 ≈ 2.24, and the fine-lattice (4, 10) at f ∝ √116
 * ≈ 10.77.  The demo doesn't make sound — but each figure on screen
 * corresponds to a real audible pitch a real plate would emit if
 * vibrated at that frequency.  Real Kirchhoff plates have slightly
 * different frequencies because their PDE is ∇⁴w + ∂²w/∂t² = 0 (not
 * the simpler ∇²w form this approximation uses); the (m, n) labelling
 * carries over but the spacing of frequencies differs slightly.
 * ───────────────────────────────────────────────────────────────────── */
static const int MODES[][2] = {
    /* n = 2..6 (the lower-mode classics) */
    {1, 2},
    {1, 3},
    {2, 3},
    {1, 4},
    {2, 4},
    {3, 4},
    {1, 5},
    {2, 5},
    {3, 5},
    {4, 5},
    {1, 6},
    {2, 6},
    {3, 6},
    {4, 6},
    {5, 6},
    /* n = 7..9 (intricate mid-range) */
    {1, 7},
    {2, 7},
    {3, 7},
    {4, 7},
    {5, 7},
    {6, 7},
    {1, 8},
    {2, 8},
    {3, 8},
    {4, 8},
    {5, 8},
    {6, 8},
    {7, 8},
    {1, 9},
    {2, 9},
    {3, 9},
    {4, 9},
    {5, 9},
    {6, 9},
    {7, 9},
    {8, 9},
    /* n = 10 (fine-lattice showcase) */
    {1, 10},
    {2, 10},
    {3, 10},
    {4, 10},
};
#define N_MODES (int)(sizeof(MODES) / sizeof(MODES[0]))

#define N_THEMES 4
/* theme[theme][0]=pos fg, [1]=neg fg */
static const short THEMES_256[N_THEMES][2] = {
    {51, 196},  /* Classic:  cyan / red      */
    {45, 30},   /* Ocean:    blue / teal      */
    {202, 160}, /* Ember:  orange / dark-red  */
    {82, 201},  /* Neon:    green / magenta   */
};
static const short THEMES_8[N_THEMES][2] = {
    {COLOR_CYAN, COLOR_RED},
    {COLOR_BLUE, COLOR_CYAN},
    {COLOR_YELLOW, COLOR_RED},
    {COLOR_GREEN, COLOR_MAGENTA},
};
static const char *THEME_NAMES[N_THEMES] = {"Classic", "Ocean", "Ember",
                                            "Neon"};

/* density chars: distance from nodal line → char brightness */
static const char NODAL_CHARS[] = "@#*+."; /* closest → farthest  */
#define N_NCHARS (int)(sizeof(NODAL_CHARS) - 1)

/* ── §2 clock ────────────────────────────────────────────────────────── */

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

/* ── §3 color ────────────────────────────────────────────────────────── */

/*
 * color_apply — install the colour palette for theme index t.
 *
 * CP_POS / CP_NEG vary per theme.  CP_NODE / CP_HUD / CP_HINT are
 * canonical chrome — white nodal core, bright yellow top status bar,
 * bright cyan bottom action bar — fixed across themes so the figure
 * highlights and the HUD stay legible whichever palette is active.
 */
static void color_apply(int t) {
  short pos, neg;
  if (COLORS >= 256) {
    pos = THEMES_256[t][0];
    neg = THEMES_256[t][1];
  } else {
    pos = THEMES_8[t][0];
    neg = THEMES_8[t][1];
  }
  init_pair(CP_POS, pos, -1);
  init_pair(CP_NEG, neg, -1);
  init_pair(CP_NODE, (COLORS >= 256) ? 231 : COLOR_WHITE, -1);
  init_pair(CP_HUD, (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_HINT, (COLORS >= 256) ? 51 : COLOR_CYAN, -1);
}

/* ── §4 field math (pure functions) ──────────────────────────────────── */

/*
 * chladni_z — analytic mode-shape value at (x, y) for mode (m, n).
 *     Z(x, y) = cos(mπx)·cos(nπy) − cos(nπx)·cos(mπy)
 * Nodal lines are where Z = 0.  See CONCEPTS header for derivation.
 */
static float chladni_z(float x, float y, int m, int n) {
  return cosf((float)m * (float)M_PI * x) * cosf((float)n * (float)M_PI * y) -
         cosf((float)n * (float)M_PI * x) * cosf((float)m * (float)M_PI * y);
}

/* ── §5 scene — state struct + mutators ──────────────────────────────── */

/* ─────────────────────────────────────────────────────────────────────
 * Scene — the entire cymatics demo state.
 *
 * One Scene IS the demo.  Every non-trivial function takes a Scene* and
 * does its work on it; the only mutable state living outside Scene is
 * the two signal-handler flags (g_running, g_need_resize), which must
 * be globals because the C signal API (signal(3)) has no context
 * pointer.
 *
 * Three semantically distinct groups of members, organised below by
 * where they live in the algorithm rather than by type:
 *
 *   1. CHLADNI MODE STATE  — which standing-wave figure is shown, plus
 *                            the finite state machine that animates
 *                            between consecutive figures.
 *   2. UI STATE            — user-controlled toggles for the demo's
 *                            three switches (auto, pause, theme).
 *   3. SCREEN DIMENSIONS   — cell-space size, refreshed on SIGWINCH.
 *
 * The animation is a 2-state machine — HOLD (freeze on current figure)
 * alternating with MORPH (cross-fade to the next):
 *
 *       ┌─────────────────────── auto_advance toggle controls entry ─┐
 *       │                                                            │
 *       ▼            hold_ctr ≥ hold_ticks         t increments      │
 *    ST_HOLD ────────────────────────────────► ST_MORPH ─────────────┘
 *       ▲                                          │  (t reaches 1.0:
 *       │                                          ▼   mode_idx++,
 *       └──────── reset hold_ctr; new figure ──────┘   reset to HOLD)
 *
 * References (full list in the CONCEPTS header at the top of the file):
 *
 *   • Chladni, E. F. F. (1787) "Entdeckungen über die Theorie des
 *     Klanges" — coined the figures and described the sand-on-plate
 *     experiment that the (m, n) pairs in MODES[] parameterise.
 *   • Kirchhoff, G. (1850) — the biharmonic PDE for plate vibration
 *     whose eigenmodes are the (m, n) shapes Scene::mode_idx selects.
 *   • Rayleigh, Lord (1894-96) "The Theory of Sound" §195-227 — modal
 *     decomposition of plates: every Chladni figure is one eigenmode.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ─ CHLADNI MODE STATE ─────────────────────────────────────────── *
   *
   * One Chladni figure is fully specified by an integer pair (m, n)
   * from MODES[]; mode_idx is the index into that table.  scene_tick
   * advances mode_idx by one each time a MORPH cycle completes, and
   * the n/p keys jump it manually (via scene_advance_mode).
   *
   * The state machine (state, hold_ctr, hold_ticks, t) sequences the
   * animation:
   *
   *   ST_HOLD : dwell on the current figure.  If auto_advance is on,
   *             hold_ctr ticks up to hold_ticks, then enter ST_MORPH.
   *
   *   ST_MORPH: cross-fade toward MODES[(mode_idx+1) % N_MODES].  The
   *             renderer's z_at_cell blends Z_current and Z_next as
   *             (1−t)·Z_cur + t·Z_next.  When t ≥ 1.0, mode_idx
   *             advances and the machine returns to ST_HOLD.
   *
   * hold_ticks is user-tunable via +/- (scene_adjust_hold) so the
   * dwell can run from 0 ticks (continuous showcase morphing) up to
   * HOLD_MAX (slow study).                                            */
  int mode_idx;   /* 0..N_MODES-1 — index into MODES[] (40 figures)*/
  int state;      /* ST_HOLD (0) or ST_MORPH (1)                   */
  int hold_ctr;   /* dwell counter, 0..hold_ticks-1; resets per HOLD*/
  int hold_ticks; /* dwell duration, clamped to [HOLD_MIN, HOLD_MAX]*/
  float t;        /* morph cross-fade ∈ [0, 1); 0 ⇒ pure current
                     figure, → 1 ⇒ pure next figure                */

  /* ─ UI STATE ───────────────────────────────────────────────────── *
   *
   * Set only by app_handle_key (which calls the scene_toggle_* /
   * scene_cycle_* / scene_adjust_* mutators) so the keypress switch
   * stays a declarative dispatch table.  Read by scene_tick (paused,
   * auto_advance) and by the renderers (theme).                       */
  int auto_advance; /* 'a': true → HOLD auto-promotes to MORPH;
                       false → freeze on current figure forever      */
  int paused;       /* spc: true → scene_tick is a no-op (everything
                       frozen, including any in-progress morph)      */
  int theme;        /* 't'/'T': 0..N_THEMES-1 — index into THEMES_*
                       tables; scene_cycle_theme calls color_apply   */

  /* ─ SCREEN DIMENSIONS ──────────────────────────────────────────── *
   *
   * Cell-space dimensions of the terminal, including the HUD chrome
   * rows.  The chladni field renderer (render_field) draws rows
   * 1..rows-2 — leaving row 0 for the top HUD and row rows-1 for the
   * bottom action bar.  Refreshed only by scene_resize, which the
   * main loop calls after the SIGWINCH signal sets g_need_resize.    */
  int rows; /* total terminal rows (HUD chrome included)    */
  int cols; /* total terminal columns                        */
} Scene;

/* scene_init — boot defaults: first mode, hold state, auto on, theme 0. */
static void scene_init(Scene *sc) {
  sc->mode_idx = 0;
  sc->state = ST_HOLD;
  sc->hold_ctr = 0;
  sc->hold_ticks = HOLD_DEFAULT;
  sc->t = 0.0f;
  sc->auto_advance = 1;
  sc->paused = 0;
  sc->theme = 0;
  sc->rows = 0;
  sc->cols = 0;
}

/* scene_resize — read the current terminal size into the scene. */
static void scene_resize(Scene *sc) {
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  sc->rows = rows;
  sc->cols = cols;
  erase();
}

/*
 * scene_tick — one tick of the HOLD ↔ MORPH state machine.
 *
 *   HOLD : if auto_advance, increment hold_ctr.  Cross hold_ticks
 *          → enter MORPH at t = 0.
 *   MORPH: advance t by MORPH_SPEED each tick.  Cross 1.0
 *          → advance mode_idx, reset to HOLD.
 *
 * If paused, do nothing.
 */
static void scene_tick(Scene *sc) {
  if (sc->paused)
    return;

  if (sc->state == ST_HOLD) {
    if (sc->auto_advance) {
      if (++sc->hold_ctr >= sc->hold_ticks) {
        sc->state = ST_MORPH;
        sc->t = 0.0f;
      }
    }
    return;
  }

  /* ST_MORPH */
  sc->t += MORPH_SPEED;
  if (sc->t >= 1.0f) {
    sc->t = 0.0f;
    sc->mode_idx = (sc->mode_idx + 1) % N_MODES;
    sc->state = ST_HOLD;
    sc->hold_ctr = 0;
  }
}

/* scene_advance_mode — manual n/p jump.  Aborts any in-progress morph. */
static void scene_advance_mode(Scene *sc, int dir) {
  sc->mode_idx = ((sc->mode_idx + dir) % N_MODES + N_MODES) % N_MODES;
  sc->state = ST_HOLD;
  sc->hold_ctr = 0;
  sc->t = 0.0f;
}

static void scene_toggle_pause(Scene *sc) { sc->paused ^= 1; }
static void scene_toggle_auto(Scene *sc) { sc->auto_advance ^= 1; }

/* scene_cycle_theme — bump theme index and re-install the palette. */
static void scene_cycle_theme(Scene *sc) {
  sc->theme = (sc->theme + 1) % N_THEMES;
  color_apply(sc->theme);
}

/*
 * scene_adjust_hold — change auto-cycle dwell by `delta` ticks.
 * Positive delta = slower; negative = faster.  Clamped to [HOLD_MIN,
 * HOLD_MAX]; floor = continuous showcase morphing with no dwell.
 */
static void scene_adjust_hold(Scene *sc, int delta) {
  sc->hold_ticks += delta;
  if (sc->hold_ticks < HOLD_MIN)
    sc->hold_ticks = HOLD_MIN;
  if (sc->hold_ticks > HOLD_MAX)
    sc->hold_ticks = HOLD_MAX;
}

/*
 * z_at_cell — Z value the renderer wants at cell (r, c).
 *
 * Linear blend of the current mode's Z and the next mode's Z, with the
 * morph parameter t doing the cross-fade.  At t=0 only the current
 * mode is evaluated (one chladni_z call instead of two).
 */
static float z_at_cell(const Scene *sc, int r, int c) {
  float x = (sc->cols > 1) ? (float)c / (float)(sc->cols - 1) : 0.5f;
  float y = (sc->rows > 1) ? (float)r / (float)(sc->rows - 1) : 0.5f;

  int cm = MODES[sc->mode_idx][0], cn = MODES[sc->mode_idx][1];
  float z1 = chladni_z(x, y, cm, cn);
  if (sc->t <= 0.0f)
    return z1;

  int nm = MODES[(sc->mode_idx + 1) % N_MODES][0];
  int nn = MODES[(sc->mode_idx + 1) % N_MODES][1];
  float z2 = chladni_z(x, y, nm, nn);
  return (1.0f - sc->t) * z1 + sc->t * z2;
}

/* ── §6 rendering — band-threshold pipeline + composers ──────────────── */

/*
 * Band thresholds — five tiers of "how close is |Z| to a nodal line".
 *   < 0.04  band 0 — nodal core    (@ white,           BOLD)
 *   < 0.10  band 1 — close to node (# pos/neg colour,  BOLD)
 *   < 0.18  band 2 — penumbra      (* pos/neg colour,  DIM )
 *   < 0.28  band 3 — faint glow    (+ pos/neg colour,  DIM )
 *   < 0.40  band 4 — antinode hint (. pos/neg colour,  DIM )
 *   else    band -1 → blank cell
 */
static const float BAND_THRESH[] = {0.04f, 0.10f, 0.18f, 0.28f, 0.40f};

/* band_for_amplitude — quantise |Z| into one of the five bands, or -1. */
static int band_for_amplitude(float az) {
  for (int b = 0; b < N_NCHARS; b++)
    if (az < BAND_THRESH[b])
      return b;
  return -1;
}

/* pair_for_band — nodal core white; outer bands tinted by sign(z). */
static int pair_for_band(int band, float z) {
  if (band == 0)
    return CP_NODE;
  return (z > 0.0f) ? CP_POS : CP_NEG;
}

/* attr_for_band — inner two bands BOLD (bright sand grain); rest DIM. */
static attr_t attr_for_band(int band, int pair) {
  return (band <= 1) ? (COLOR_PAIR(pair) | A_BOLD) : (COLOR_PAIR(pair) | A_DIM);
}

/*
 * paint_field_cell — band-threshold one cell and paint its glyph.
 * Cells outside the outermost band stay blank.
 */
static void paint_field_cell(const Scene *sc, int r, int c) {
  float z = z_at_cell(sc, r, c);
  float az = fabsf(z);

  int band = band_for_amplitude(az);
  if (band < 0) {
    mvaddch(r, c, ' ');
    return;
  }
  int pair = pair_for_band(band, z);
  attr_t attr = attr_for_band(band, pair);
  attron(attr);
  mvaddch(r, c, (chtype)(unsigned char)NODAL_CHARS[band]);
  attroff(attr);
}

/*
 * render_field — paint the chladni field everywhere except the HUD rows.
 * Row 0 (top HUD) and row rows-1 (bottom HUD) are skipped so chrome
 * stays intact.
 */
static void render_field(const Scene *sc) {
  for (int r = 1; r < sc->rows - 1; r++)
    for (int c = 0; c < sc->cols - 1; c++)
      paint_field_cell(sc, r, c);
}

/*
 * render_hud_top — canonical top status bar (row 0).
 * Right-aligned: current → next mode, state, t, hold time, theme,
 * auto/manual, paused/running.  CP_HUD (bright yellow + bold).
 */
static void render_hud_top(const Scene *sc) {
  int cm = MODES[sc->mode_idx][0], cn = MODES[sc->mode_idx][1];
  int nm = MODES[(sc->mode_idx + 1) % N_MODES][0];
  int nn = MODES[(sc->mode_idx + 1) % N_MODES][1];
  const char *state_str = (sc->state == ST_MORPH) ? "morphing" : "hold";

  /* Hold time in seconds (TICK_NS is 33.3 ms → 30 fps). */
  double hold_sec = (double)sc->hold_ticks / 30.0;

  char buf[200];
  snprintf(buf, sizeof buf,
           " mode(%d,%d)->(%d,%d)  [%d/%d]  %s t=%.2f  hold:%.1fs  "
           "theme:%s  %s  %s ",
           cm, cn, nm, nn, sc->mode_idx + 1, N_MODES, state_str, (double)sc->t,
           hold_sec, THEME_NAMES[sc->theme], sc->auto_advance ? "auto" : "man.",
           sc->paused ? "PAUSED" : "running");
  int len = (int)strlen(buf);
  int col = sc->cols - len;
  if (col < 0)
    col = 0;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, sc->cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * render_hud_bottom — canonical bottom action bar (row -1).
 * Left-aligned key list; short fallback when the terminal is narrow.
 * CP_HINT (bright cyan + bold).
 */
static void render_hud_bottom(const Scene *sc) {
  const char *hint_full =
      " q:quit  spc:pause  n/p:mode  t:theme  a:auto  +/-:hold ";
  const char *hint_short = " q:quit  n:mode  t:theme  +/-:hold ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= sc->cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(sc->rows - 1, 0, hint, sc->cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §7 screen + app ─────────────────────────────────────────────────── */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  start_color();
  use_default_colors();
  color_apply(0); /* boot palette; scene_cycle_theme rotates it later */
}

/* Signal-handler flags must be globals — the C signal API has no
 * context pointer, so any per-Scene state can't be reached from here.  */
static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig) {
  if (sig == SIGWINCH)
    g_need_resize = 1;
  else
    g_running = 0;
}
static void cleanup(void) { endwin(); }

/*
 * app_handle_key — route one keypress to the matching scene mutator.
 * Returns 0 on quit, 1 otherwise.
 */
static int app_handle_key(Scene *sc, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
    return 0;
  case ' ':
    scene_toggle_pause(sc);
    break;
  case 'a':
  case 'A':
    scene_toggle_auto(sc);
    break;
  case 'n':
    scene_advance_mode(sc, +1);
    break;
  case 'p':
    scene_advance_mode(sc, -1);
    break;
  case 't':
  case 'T':
    scene_cycle_theme(sc);
    break;
  case '+':
  case '=':
    scene_adjust_hold(sc, -HOLD_STEP);
    break;
  case '-':
  case '_':
    scene_adjust_hold(sc, +HOLD_STEP);
    break;
  }
  return 1;
}

int main(void) {
  signal(SIGINT, sig_handler);
  signal(SIGTERM, sig_handler);
  signal(SIGWINCH, sig_handler);
  atexit(cleanup);

  screen_init();

  Scene scene;
  scene_init(&scene);
  scene_resize(&scene);

  long long next = clock_ns();

  while (g_running) {
    if (g_need_resize) {
      g_need_resize = 0;
      endwin();
      refresh();
      scene_resize(&scene);
    }

    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(&scene, ch)) {
        g_running = 0;
        break;
      }
    }

    scene_tick(&scene);
    erase();
    render_field(&scene);
    render_hud_top(&scene);
    render_hud_bottom(&scene);
    wnoutrefresh(stdscr);
    doupdate();

    next += TICK_NS;
    clock_sleep_ns(next - clock_ns());
  }
  return 0;
}
