/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bat.c  —  formations of ASCII bats erupting from the screen centre into the dark
 *
 * Three formations of bats burst outward from the screen centre, each heading
 * in a different direction.  Every formation is a V-wedge — one leader at the
 * tip with follower rows fanning out behind.  Each bat flaps through a 4-frame
 * wing cycle.
 *
 * Formation shape — filled triangle (Pascal rows), n_rows = 3 shown:
 *
 *            /o\                 ← row 0: 1 bat  (leader, bold)
 *         /o\   /o\              ← row 1: 2 bats
 *      /o\   /o\   /o\           ← row 2: 3 bats
 *   /o\   /o\   /o\   /o\        ← row 3: 4 bats
 *
 * Row r has r+1 bats evenly spaced.  Total bats = (n+1)(n+2)/2.
 *   n_rows 1 → 3 bats    n_rows 4 → 15 bats
 *   n_rows 2 → 6 bats    n_rows 5 → 21 bats
 *   n_rows 3 → 10 bats   n_rows 6 → 28 bats
 *
 * Pressing + / - grows or shrinks the triangle live.  Flying formations gain
 * or lose bats immediately, placed at the correct position in the formation.
 *
 * Wing frames (triangle wave):
 *   0  /o\   wings up
 *   1  -o-   wings level
 *   2  \o/   wings down
 *   3  -o-   wings level
 *
 * When the leader of a formation exits the screen the formation holds briefly
 * then re-launches from the centre in a new direction, cycling through 6
 * presets.
 *
 * Formation colours:
 *   Formation 0 — light purple  (xterm 141) / magenta fallback
 *   Formation 1 — electric cyan (xterm  87) / cyan fallback
 *   Formation 2 — pink-magenta  (xterm 213) / white fallback
 *
 * Keys:
 *   q / ESC   quit
 *   + / -     add / remove one row (1–6 rows, 3–28 bats per formation)
 *   r         reset all formations to centre
 *   p / spc   pause / resume
 *   ] [       faster / slower simulation
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra Artistic/bat.c -o bat -lncurses -lm
 *
 * Sections  (re-cut into single-concern layers — see ARCHITECTURE below)
 * --------
 *   §1  config       — constants, numeric palette data, timing macros
 *   §2  clock        — PERFORMANCE: monotonic clock + sleep
 *   §3  color        — RENDER setup: theme palettes, colour-pair binding
 *   §4  coords        — LOGIC: pure pixel ↔ cell conversions
 *   §5  data         — type definitions (Bat, Formation, Scene, Screen)
 *   §6  logic        — pure decisions (formation geometry, leader-exit test)
 *   §7  simulation   — advances state; scene_tick is the per-tick combine
 *   §8  render       — state → screen (reads only)
 *   §9  screen       — terminal lifecycle (init / resize / teardown)
 *   §10 app          — input events + fixed-timestep main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : V-formation particle system with triangular row layout.
 *                  Row r contains r+1 bats; total bats = (n+1)(n+2)/2
 *                  (triangular number).  Each bat's position is computed
 *                  analytically from formation heading angle, row index, and
 *                  column index — no physics integration, pure kinematics.
 *
 * Data-structure : Fixed-size formation array; each formation stores heading
 *                  (angle), velocity, flight state (FLYING / HOLDING), and
 *                  n_rows.  Bats within a formation are reconstructed each
 *                  frame from the formation's leader, avoiding per-bat storage
 *                  for the wedge shape.
 *
 * Math           : Row r, column c bat offset from leader:
 *                    dx = c·SPACING_X − r·SPACING_X/2  (fan-out)
 *                    dy = r·SPACING_Y                   (depth behind leader)
 *                  Rotation matrix applied to (dx,dy) using formation heading θ.
 *
 * Rendering      : 4-frame wing cycle (0→up, 1→level, 2→down, 3→level)
 *                  implemented as a triangle wave index: frame%4 maps to
 *                  one of 3 ASCII bat shapes.  Leader drawn bold.
 *
 * References     : Formation flight & group motion —
 *  • Reynolds, C.W. "Flocks, Herds, and Schools: A Distributed Behavioral
 *    Model," SIGGRAPH 1987 — the boids model; canonical reference for animated
 *    groups of creatures. (Here the formation is SCRIPTED geometry, not emergent
 *    flocking, but the leader/follower idea descends from this line.)
 *  • Reynolds, C.W. "Steering Behaviors For Autonomous Characters," GDC 1999 —
 *    leader following, offset pursuit and formation motion: directly the leader
 *    + follower-row layout each formation flies in.
 *  • Lissaman, P.B.S. & Shollenberger, C.A. "Formation Flight of Birds,"
 *    Science 168 (1970) — the aerodynamics behind echelon / V formations; the
 *    real-world inspiration for the triangular flight wedge.
 *                  Animation, kinematics & rendering —
 *  • Williams, R. *The Animator's Survival Kit* (Faber, 2001) — keyframe cycles;
 *    the 4-frame wing flap (WING_CYCLE) is a looping flap cycle.
 *  • Lengyel, E. *Mathematics for 3D Game Programming and Computer Graphics*
 *    (3rd ed., 2011) — 2D rotation matrices + parametric constant-velocity
 *    motion: the heading-rotation of each bat's (dx,dy) offset and formation flight.
 *  • Fiedler, G. "Fix Your Timestep!" (gafferongames.com, 2004) — the fixed-
 *    timestep accumulator that decouples bat kinematics from frame rate.
 *  • Graham, Knuth & Patashnik, *Concrete Mathematics* (§1.2, figurate numbers)
 *    — the triangular number (n+1)(n+2)/2 that sizes each formation.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A V-formation isn't simulated bat-by-bat — it's a RIGID JIG that rides
 * on top of the leader.  Every follower's position is the leader's
 * position plus a fixed (along, perp) offset rotated into the formation's
 * heading.  Simulation is a single point per formation; everything else is
 * cosmetic geometry recomputed each frame.  The wing flap is a four-frame
 * cycle indexed by a per-bat phase counter, and the formation is a
 * triangular Pascal layout where row r holds r+1 bats.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of a paper-cutout V glued to the tip of an arrow.  The arrow
 * flies; the V flies with it, never deforming.  When the arrow flips to
 * a new heading, the V instantly re-orients — there is no inertia, no
 * per-bat physics, no flocking.  The "swarm" feel comes purely from
 * three independent arrows leaving the same origin at staggered times
 * along six preset compass directions.
 *
 * ALGORITHM IN STEPS  (per formation, per tick)
 * ──────────────────
 *  1. If FLIGHT_HOLDING, decrement timer; on zero, advance angle_idx, pick
 *     the next preset angle from k_angles[6], call formation_launch().
 *  2. formation_launch sets vx,vy = SPEED·(cos θ, sin θ) and re-places every
 *     bat at (cx,cy) using the formation jig.
 *  3. If FLIGHT_FLYING, advance every bat by (vx·dt, vy·dt) — same delta
 *     for all bats, so the formation is rigid by construction.
 *  4. Increment each bat's phase by 1; wrap at WING_CYCLE (=36 ticks).
 *  5. If the LEADER (bat 0) leaves the screen rect, switch to FLIGHT_HOLDING
 *     for HOLD_TICKS.
 *  6. Render: for each bat compute frame = phase·4/WING_CYCLE ∈ {0..3}
 *     and stamp the 3-cell glyph (lw, body, rw); leader uses A_BOLD.
 *
 * KEY FORMULAS
 * ────────────
 *  Pascal row of bat k :  largest r with r·(r+1)/2 <= k
 *  Position in row     :  pos = k − r·(r+1)/2          ∈ {0..r}
 *  Bat count           :  n_bats = (n_rows+1)·(n_rows+2)/2  (triangular)
 *  Flight-frame offset :  along = −r · LAG_PX
 *                         perp  = (pos − r/2) · SPREAD_PX
 *  World rotation      :  wx = lx + along·cos θ − perp·sin θ
 *                         wy = ly + along·sin θ + perp·cos θ
 *  Velocity            :  (vx, vy) = SPEED·(cos θ, sin θ)
 *  Wing frame index    :  frame = ⌊phase · 4 / WING_CYCLE⌋   (∈ 0..3)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Leader-exit detection uses bat[0] only — followers can briefly
 *    linger off-screen after the leader trips the bound; the cell-clip
 *    in formation_draw skips out-of-bounds glyphs so this is safe.
 *  • WING_CYCLE must be divisible by 4, else `frame` skips frames at
 *    the wrap point.  36 / 4 = 9, fine.
 *  • Growing rows mid-flight: new bats inherit current leader pos and
 *    velocity, so the formation snaps in rigidly without trailing.
 *  • Shrinking rows: tail bats are simply not iterated — old pixel
 *    positions remain in the slots but are never drawn.
 *  • Initial stagger uses (k_init_angle_idx − 1) so the post-hold
 *    increment lands on the intended first-launch heading.
 *  • Resize triggers full scene_init; current formation state is discarded
 *    intentionally rather than trying to re-anchor mid-flight.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press `+` six times: row count must climb 3→4→5→6 then clamp.
 *    Bat counts per formation: 6, 10, 15, 21, 28.
 *  • Pause with space; the leaders sit at the centre and three V's
 *    fan out behind in their respective heading.  Count rows: should
 *    equal n_rows + 1 (leader is row 0).
 *  • Watch one bat through one flap: '/o\' → '-o-' → '\o/' → '-o-' →
 *    '/o\' over WING_CYCLE ticks (~0.6 s at 60 fps).
 *  • All three formations should never collide at t=0 — the STAGGER_TICKS
 *    delay puts them on the screen one at a time.
 *  • After leader exits, the formation reappears at centre after
 *    HOLD_TICKS ticks (≈0.9 s at 60 fps) heading 60° later in the
 *    six-preset cycle.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation + abstraction) ────────────────────── *
 *
 * Single-concern layers; the table says what each layer MUTATES.  The
 * narrowest-type signature convention is now applied: a pure read takes
 * `const DomainType *`, a mutator takes `DomainType *`, and only the few
 * orchestrators that drive a whole tick/reset take `Scene *` — so although the
 * simulation DATA aggregates on Scene, the layers never re-couple through it.
 *
 *   LAYER        SECTION         MUTATES
 *   ─────        ───────         ───────
 *   PERFORMANCE  §2  clock       timing only — sim_accum / fps_accum / frame
 *                §10 app loop    timer / fps_display / sim_fps.  Never sim state.
 *   LOGIC        §4  coords      nothing — pure px↔cell conversions
 *                §6  logic       nothing — pure (bat_form_offset, leader_exited)
 *   SIMULATION   §7  simulation  Formation{ bats[].px/py/phase, vx, vy, angle,
 *                                 state, timer, angle_idx, n_rows, n_bats },
 *                                 Scene{ formations, n_rows, cols, rows, paused }
 *   RENDER       §3  color       ncurses colour pairs (setup only)
 *                §8  render      the ncurses frame buffer ONLY; reads sim state
 *                                 through const pointers, never writes it
 *
 * EFFECTS  — no standalone layer.  The only cosmetic state is each bat's wing
 *            `phase`; it is advanced inline in formation_tick (SIMULATION) and
 *            the 4-frame flap index + leader A_BOLD are DERIVED read-only at
 *            render time (formation_draw).  Nothing cosmetic is stored or
 *            updated apart from the sim tick, so there is no effects state to
 *            layer.
 * DELAYS   — no standalone layer.  The inter-flight hold and the launch
 *            stagger are the single `Formation.timer` field: set by
 *            formation_hold / scene_init, counted down inside formation_tick.
 *            Folded into SIMULATION because the function that advances flight
 *            is the same one that counts the hold.
 *
 * PER-TICK COMBINE — scene_tick (§7) is the ONE place that advances state.
 * It calls formation_tick for each formation; per formation, in explicit order:
 *     HOLDING state: DELAY countdown → on expiry SIM formation_launch (new heading)
 *     FLYING  state: SIM advance positions → advance wing phase
 *                  → LOGIC leader_exited? → DELAY formation_hold
 * Nothing else advances simulation state.
 *
 * USER EVENTS (NOT part of the tick) — these mutate state from §10 in response
 * to input, OUTSIDE scene_tick:
 *     scene_init        boot / 'r' reset / SIGWINCH resize
 *     scene_set_rows    '+' / '-'  (grow / shrink formation)
 *     paused toggle     'p' / space
 *     apply_theme       't' / 'T'   (App.theme — a RENDER setting, not sim state)
 *     sim_fps           '[' / ']'
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MIN     =  10,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,

    /*
     * CELL_W × CELL_H — physical pixels per terminal cell.
     * Physics lives in pixel space so that diagonal motion is isotropic
     * regardless of the non-square terminal cell aspect.
     */
    CELL_W = 8,
    CELL_H = 16,

    N_FORMATIONS = 3,

    /*
     * n_rows — number of follower rows behind the leader (1 = leader only pair,
     * 6 = six rows deep).  n_bats = (n_rows+1)*(n_rows+2)/2.
     * MAX_BATS covers the largest possible formation.
     */
    ROWS_DEFAULT =  3,
    ROWS_MIN     =  1,
    ROWS_MAX     =  6,   /* row 6 → 7 bats wide; comfortable on any terminal */

    /*
     * STAGGER_TICKS — how many ticks each successive formation waits before
     * launching.  Prevents all three formations overlapping at t=0.
     */
    STAGGER_TICKS = 30,
    HOLD_TICKS    = 55,  /* ticks to hold at centre before re-launch */

    /*
     * WING_CYCLE  — ticks for one complete flap.  36 @ 60 fps ≈ 0.6 s ≈ 1.7 Hz.
     * WING_FRAMES — keyframes in that flap (up→level→down→level).  WING_CYCLE
     *               must be divisible by WING_FRAMES so frame selection is even
     *               (36 / 4 = 9 ticks per frame).
     */
    WING_CYCLE    = 36,
    WING_FRAMES   =  4,

    FPS_UPDATE_MS = 500,

    /*
     * Render-loop timing (consumed in main()).  Distinct from the SIM_FPS_* set:
     * those set how often the sim TICKS; these bound the FRAME the user sees.
     */
    FRAME_CAP_FPS = 60,   /* present at most this many frames/sec       */
    MAX_FRAME_MS  = 100,  /* clamp a long dt to this, so a stall can't  */
                          /* spiral the sim into a catch-up death loop  */

    HUD_TITLE_COLS = 18,  /* min column for the stats block, so it never */
                          /* collides with the left-justified title bar  */
};

/*
 * MAX_BATS — worst-case bat count per formation (n_rows = ROWS_MAX).
 * Triangular number: (ROWS_MAX+1)*(ROWS_MAX+2)/2.
 * ROWS_MAX=6 → 7*8/2 = 28.
 */
#define MAX_BATS ((ROWS_MAX + 1) * (ROWS_MAX + 2) / 2)   /* = 28 */

/*
 * FORMATION_SPEED — physical pixels per second.
 * LAG_PX          — along-flight gap between successive rows (pixels).
 * SPREAD_PX       — lateral gap per rank unit (pixels).
 *                   Row k sits at ±k × SPREAD_PX from the centreline.
 */
#define FORMATION_SPEED  260.0f
#define LAG_PX            56.0f
#define SPREAD_PX         32.0f

/*
 * SECONDS_PER_TICK — sim time advanced per tick.  Deliberately tied to
 * SIM_FPS_DEFAULT, NOT the live sim_fps: changing Hz changes how OFTEN the sim
 * ticks (accumulator in main), while each tick always advances a fixed 1/60 s
 * of motion — so faster Hz reads as faster flight, by design.
 * PARK_PX — off-screen parking position for a held (invisible) formation's bats.
 */
#define SECONDS_PER_TICK (1.0f / (float)SIM_FPS_DEFAULT)
#define PARK_PX          (-99999.0f)

/*
 * 6 preset flight angles (radians, y-down screen coordinates).
 * Screen y-down: sin > 0 = downward, sin < 0 = upward.
 *
 *   330° → upper-right    210° → upper-left    90° → straight down
 *    45° → lower-right    135° → lower-left   270° → straight up
 *
 * WHY presets, not random: scripted headings keep the three formations visually
 * separated and reproducible.  Each formation holds an `angle_idx` into this
 * table and advances it +1 mod N_ANGLES on every relaunch, so its successive
 * flights walk these six headings in order (see Formation.angle_idx).
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(d)  ((d) * (float)M_PI / 180.0f)

static const float k_angles[6] = {
    DEG2RAD(330.0f), DEG2RAD(210.0f), DEG2RAD( 90.0f),
    DEG2RAD( 45.0f), DEG2RAD(135.0f), DEG2RAD(270.0f),
};
#define N_ANGLES 6

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                       PERFORMANCE — monotonic time + sleep   */
/* ===================================================================== */
/* Mutates nothing.  Read by the §10 fixed-timestep loop only. */

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
/* §3  color                       RENDER setup — palettes + pair binding */
/* ===================================================================== */
/* Mutates ncurses colour pairs only (setup / 't'-'T' key).  No sim state. */

/*
 * ColorID — names for the ncurses colour-pair slots this program allocates.
 *
 * WHY: ncurses addresses colours by small integer "pair" handles (defined with
 * init_pair, selected with COLOR_PAIR); naming them keeps every call site
 * readable instead of scattering magic 1..5.  The values ARE the literal pair
 * slots — they start at 1 because ncurses reserves pair 0 for the terminal's
 * default foreground/background, which cannot be redefined.
 *
 * Two groups, split by concern:
 *   COL_G0..COL_G2  — one slot per formation (3 = N_FORMATIONS).  RENDER-themed:
 *                     apply_theme() rebinds these to the active palette (t/T).
 *   COL_HUD/COL_HINT— fixed HUD slots, never themed, so the overlay stays
 *                     legible against any palette (project HUD standard).
 */
typedef enum {
    COL_G0   = 1,   /* formation 0 — themed (rebound by apply_theme)        */
    COL_G1   = 2,   /* formation 1 — themed                                 */
    COL_G2   = 3,   /* formation 2 — themed                                 */
    COL_HUD  = 4,   /* HUD data line (top): fixed bright white, never themed */
    COL_HINT = 5,   /* HUD action line (bottom): fixed bright cyan           */
} ColorID;

/*
 * Colour themes — the palette table, indexed by App.theme (0..N_THEMES-1, the
 * value t/T cycles).  Each ROW gives the N_FORMATIONS formation colours (one
 * per wedge); the columns line up with COL_G0/G1/G2.
 *
 * Value logic: entries are xterm-256 cube indices, all chosen from the BRIGHT
 * half of the cube — bottom-of-palette colours go invisible against black under
 * A_DIM, so even a theme's darkest hue stays legible (project palette rule).
 * Each row's three hues are kept distinct so the three formations read apart.
 * theme_names[] is the parallel HUD label for each row.  On an 8-colour
 * terminal apply_theme() ignores this table and falls back to magenta/cyan/white.
 */
#define N_THEMES 5
static const char *const theme_names[N_THEMES] = {
    "NEON  ", "SUNSET", "EMBER ", "FOREST", "CANDY ",
};
static const short bat_themes[N_THEMES][N_FORMATIONS] = {
    { 141,  87, 213 },   /* NEON   — purple / cyan / pink   */
    { 208, 197, 226 },   /* SUNSET — orange / rose / gold   */
    { 196, 208, 220 },   /* EMBER  — red / orange / amber   */
    {  40,  84, 159 },   /* FOREST — green / lime / aqua    */
    { 213, 226,  51 },   /* CANDY  — pink / yellow / cyan   */
};

/* apply_theme — (re)bind the formation colour pairs to theme t. Safe at
 * runtime (t/T key); the next refresh shows the new palette. 8-colour fallback
 * uses fixed magenta/cyan/white regardless of theme. */
static void apply_theme(int t)
{
    if (t < 0 || t >= N_THEMES) t = 0;
    if (COLORS >= 256) {
        init_pair(COL_G0, bat_themes[t][0], COLOR_BLACK);
        init_pair(COL_G1, bat_themes[t][1], COLOR_BLACK);
        init_pair(COL_G2, bat_themes[t][2], COLOR_BLACK);
    } else {
        init_pair(COL_G0, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_G1, COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_G2, COLOR_WHITE,   COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_HUD,  231, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_WHITE, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,  COLOR_BLACK);
    }
    apply_theme(0);   /* default theme */
}

/* ===================================================================== */
/* §4  coords                      LOGIC — pure pixel ↔ cell conversions  */
/* ===================================================================== */
/* Pure: no mutation, no I/O.  Used by both LOGIC (§6) and RENDER (§8). */

static inline int   px_to_col(float px) { return (int)(px / CELL_W); }
static inline int   px_to_row(float py) { return (int)(py / CELL_H); }
static inline float col_to_px(int col)  { return (float)col * CELL_W + CELL_W * 0.5f; }
static inline float row_to_px(int row)  { return (float)row * CELL_H + CELL_H * 0.5f; }

/* ===================================================================== */
/* §5  data                        type definitions (no behaviour)        */
/* ===================================================================== */
/*
 * All structs/enums live here so LOGIC (§6), SIMULATION (§7) and RENDER (§8)
 * can each reference them without one layer's types living inside another.
 * Defining the data has no read/mutate role of its own.
 */

/*
 * Bat — one animated bat: its body-cell position and where it is in the flap.
 *
 * WHY pixel space: position is held in sub-cell "pixels" (CELL_W×CELL_H per
 * character cell, §1) rather than integer row/col, so diagonal flight is
 * isotropic on a non-square terminal cell.  The single px→cell conversion
 * happens in the coords layer (§4), at draw time only.  `phase` is the program's
 * ONLY cosmetic state; everything else about a bat is derived from its formation.
 *
 * Refs: Lengyel, *Math for 3D Game Programming* — constant-velocity parametric
 * motion (each bat advances by the shared formation velocity); Williams,
 * *Animator's Survival Kit* — `phase` drives the looping keyframe flap.
 */
typedef struct {
    float px, py;   /* body-cell centre, pixel space (CELL_W×CELL_H per cell)    */
    float phase;    /* flap position in ticks, [0, WING_CYCLE); the draw frame   */
                    /* is derived by wing_frame() — the only cosmetic state      */
} Bat;

/*
 * Flight — the two states of a formation's launch cycle (a 2-state machine).
 *
 * WHY: a formation does not live forever on screen.  When its leader crosses
 * the edge it vanishes, waits, then relaunches in a new direction.  That wait
 * is FLIGHT_HOLDING, and it is also where the DELAYS concern lives (the
 * countdown is Formation.timer).  States are compared by name, so their numeric
 * order carries no meaning.
 *
 * Ref: Reynolds, "Steering Behaviors for Autonomous Characters" (GDC 1999) —
 * launch / re-spawn of a leader-led group.
 */
typedef enum {
    FLIGHT_HOLDING,   /* at centre between flights: invisible, timer counting down */
    FLIGHT_FLYING,    /* airborne: bats advance each tick until the leader exits    */
} Flight;

/*
 * Formation — one V-wedge of bats sharing a single heading and velocity.
 *
 * WHY one struct, not N independent bats: the wedge is a RIGID JIG.  Only the
 * leader (bat 0, the tip) has a real trajectory; every follower's world
 * position is the leader's plus a fixed (along, perp) offset rotated into the
 * heading (bat_form_offset + formation_place_bat, §6/§7).  So the struct stores
 * ONE motion state (vx, vy, angle) shared by all bats and recomputes the shape
 * from it each frame — no per-follower physics, no inertia, no flocking.
 *
 * Shape: a filled Pascal triangle, row r holding r+1 bats; the live count is
 * therefore the triangular number (n_rows+1)(n_rows+2)/2.
 *
 * Refs: Reynolds (SIGGRAPH 1987 / GDC 1999) — leader/follower & formation
 * motion; Lissaman & Shollenberger (Science 1970) — the V / echelon wedge;
 * Lengyel (2011) — the 2D rotation of each offset into the heading;
 * Graham/Knuth/Patashnik, *Concrete Mathematics* §1.2 — the triangular number.
 */
typedef struct {
    Bat     bats[MAX_BATS];   /* fixed pool (28 = worst case); only [0,n_bats) used; */
                              /* bat 0 = leader at the tip.  No malloc (mem rule).   */
    int     n_rows;           /* Pascal-triangle depth: follower rows, 1..ROWS_MAX(6) */
    int     n_bats;           /* live count; invariant (n_rows+1)(n_rows+2)/2 ⇒ 3..28 */
    float   vx, vy;           /* px/sec; = FORMATION_SPEED·(cosθ,sinθ); SAME for all  */
                              /* bats — that shared delta is what keeps the wedge rigid */
    float   angle;            /* heading θ, radians, y-down; always one of k_angles    */
    ColorID color;            /* this wedge's colour-pair slot; set once from k_colors */
    Flight  state;            /* FLIGHT_HOLDING (waiting) or FLIGHT_FLYING (airborne)  */
    int     timer;            /* DELAYS: ticks left while HOLDING; reaches ≤0 ⇒ relaunch */
    int     angle_idx;        /* cursor into k_angles[N_ANGLES]; +1 mod 6 per relaunch  */
                              /* so successive flights cycle the six compass presets    */
} Formation;

/*
 * Scene — the whole simulated world; reads top-to-bottom as a table of contents.
 *
 * WHY one aggregate: it is the single value the orchestrators (scene_init /
 * scene_tick / scene_set_rows, §7) own and advance, while every other function
 * takes the narrowest sub-type it needs (a Formation*, or a const view) — so
 * "all sim data hangs off Scene" WITHOUT the layers re-coupling through it.
 * Holds ONLY simulation state: the colour theme is a RENDER setting and lives on
 * App, deliberately not here, so a reset/resize that rebuilds Scene cannot
 * disturb the palette.
 */
typedef struct {
    Formation formations[N_FORMATIONS];  /* WHAT:  the 3 flying wedges             */
    int       n_rows;                    /* HOW:   shared wedge depth, 1..6 (+/-)  */
    int       cols, rows;                /* WHERE: viewport in cells — Scene's OWN */
                                         /*        copy of the terminal size (decoupled */
                                         /*        from Screen; see below)         */
    bool      paused;                    /* WHEN:  true ⇒ scene_tick is a no-op    */
} Scene;

/*
 * Screen — cached terminal size in character cells, for the render / I-O layer.
 *
 * WHY separate from Scene's cols/rows: this is the terminal geometry as ncurses
 * reports it (getmaxyx at init and on every SIGWINCH).  Scene keeps its OWN
 * copy so the simulation never reaches into the I/O layer for its bounds; the
 * two are synced at scene_init time, then read independently.
 */
typedef struct { int cols, rows; } Screen;   /* cells, ≥1; refreshed on resize */

/* ===================================================================== */
/* §6  logic                       LOGIC — pure decisions, no mutation    */
/* ===================================================================== */
/*
 * Provably uncorruptible from EFFECTS or RENDER: these functions do no
 * mutation and no I/O, so reordering or deleting any render/effects code
 * cannot change their result.  bat_form_offset writes only its caller-owned
 * out-params; leader_exited reads through a const pointer and returns a bool.
 */

/*
 * bat_form_offset — formation position of bat k in flight-frame coordinates.
 *
 * Bats are laid out in a filled triangle (Pascal rows):
 *   Row r contains r+1 bats: indices r*(r+1)/2 … (r+1)*(r+2)/2 − 1.
 *
 * To find row r from k, we walk up until the next row would overshoot k.
 * Position within row:  pos = k − r*(r+1)/2,  ranging 0 … r.
 *
 * along = −r × LAG_PX          (behind the leader)
 * perp  = (pos − r/2) × SPREAD_PX  (centred on the leader's lateral axis)
 *
 *   r=0, pos=0:  perp = 0                           (leader)
 *   r=1, pos=0:  perp = −½S   pos=1: +½S            (2 bats)
 *   r=2, pos=0:  perp = −S    pos=1:  0   pos=2: +S (3 bats)
 *   r=3, pos=0:  perp = −3/2S … pos=3: +3/2S        (4 bats)
 */
static void bat_form_offset(int k, float *along, float *perp)
{
    /* Determine row r: largest r with r*(r+1)/2 <= k */
    int r = 0;
    while ((r + 1) * (r + 2) / 2 <= k) r++;
    int pos = k - r * (r + 1) / 2;

    *along = -(float)r * LAG_PX;
    *perp  = ((float)pos - (float)r * 0.5f) * SPREAD_PX;
}

static bool leader_exited(const Formation *f, int cols, int rows)
{
    int c = px_to_col(f->bats[0].px);
    int r = px_to_row(f->bats[0].py);
    return (c < -2 || c > cols + 1 || r < -2 || r > rows + 1);
}

/* bat_count — how many bats a wedge of n_rows follower rows holds: the
 * triangular number (n_rows+1)(n_rows+2)/2 (Concrete Mathematics §1.2).  One
 * name for a formula the sim and the HUD both need. */
static int bat_count(int n_rows)
{
    return (n_rows + 1) * (n_rows + 2) / 2;
}

/* ===================================================================== */
/* §7  simulation                  SIMULATION — the only state-advancing  */
/* ===================================================================== */
/*
 * Mutates Formation{ bats[].px/py/phase, vx, vy, angle, state, timer,
 * angle_idx, n_rows, n_bats } and Scene{ formations, n_rows, cols, rows,
 * paused }.
 *
 * scene_tick is the ONE per-tick combine point (see ARCHITECTURE).  The other
 * mutating functions here are either its callees or USER-EVENT entry points
 * (scene_init, scene_set_rows) invoked from §10, NOT from the tick.
 *
 * The wing `phase` advance (the only cosmetic / EFFECTS state) and the hold
 * `timer` countdown (the only DELAYS state) are both folded in here rather
 * than split out — see ARCHITECTURE for why neither earns its own layer.
 */

/*
 * Per-formation init tables, indexed by formation number (0..N_FORMATIONS-1):
 *   k_init_angle_idx — each formation's STARTING heading, as an index into
 *                      k_angles.  {0,1,2} = three different presets, so the
 *                      formations diverge from t=0 instead of overlapping
 *                      (paired with the STAGGER_TICKS launch delay in scene_init).
 *   k_colors         — each formation's colour-pair slot, assigned once in
 *                      scene_init and never reassigned (a theme change remaps
 *                      what the slot LOOKS like, not which slot a formation owns).
 */
static const int     k_init_angle_idx[N_FORMATIONS] = { 0, 1, 2 };
static const ColorID k_colors[N_FORMATIONS]         = { COL_G0, COL_G1, COL_G2 };

/*
 * staggered_wing_phase — the initial flap phase for bat k of n_bats.
 * Spreads the bats evenly around the cycle (k·WING_CYCLE/n_bats) plus a small
 * random jitter (< one frame), so the wings do NOT all flap in unison — the
 * formation reads as a living swarm rather than a rigid stamp.  Wrapped into
 * [0, WING_CYCLE).
 */
static float staggered_wing_phase(int k, int n_bats)
{
    float phase = (float)k * (float)WING_CYCLE / (float)n_bats
                + (float)(rand() % (WING_CYCLE / WING_FRAMES));
    if (phase >= (float)WING_CYCLE) phase -= (float)WING_CYCLE;
    return phase;
}

/*
 * formation_place_bat — write bat k's world position and starting flap phase,
 * given the leader at (lx, ly) and the formation's heading.
 */
static void formation_place_bat(Formation *f, int k, float lx, float ly)
{
    float along, perp;
    bat_form_offset(k, &along, &perp);

    /*
     * Rotate flight-frame offset into world pixel space.
     * Flight frame axes:
     *   along → (cos_a,  sin_a)   forward
     *   perp  → (−sin_a, cos_a)   left-normal
     */
    float cos_a = cosf(f->angle);
    float sin_a = sinf(f->angle);
    f->bats[k].px = lx + along * cos_a - perp * sin_a;
    f->bats[k].py = ly + along * sin_a + perp * cos_a;

    f->bats[k].phase = staggered_wing_phase(k, f->n_bats);
}

static void formation_launch(Formation *f, float cx, float cy)
{
    f->vx    = FORMATION_SPEED * cosf(f->angle);
    f->vy    = FORMATION_SPEED * sinf(f->angle);
    f->state = FLIGHT_FLYING;
    f->timer = 0;
    for (int k = 0; k < f->n_bats; k++)
        formation_place_bat(f, k, cx, cy);
}

static void formation_hold(Formation *f, int ticks)
{
    f->state = FLIGHT_HOLDING;
    f->timer = ticks;
    for (int k = 0; k < f->n_bats; k++) {
        f->bats[k].px = PARK_PX;
        f->bats[k].py = PARK_PX;
    }
}

/*
 * formation_set_rows — resize the wedge to new_rows while preserving motion.
 *
 * Growing (new_rows > f->n_rows):
 *   New bats are placed at the correct formation positions relative to the
 *   leader's current pixel location and given random wing phases.  They
 *   immediately inherit the formation velocity so the wedge stays rigid.
 *
 * Shrinking (new_rows < f->n_rows):
 *   Simply decrement n_bats/n_rows — the tail bats are no longer iterated
 *   or drawn.  No memory needs clearing; the slots are just ignored.
 */
static void formation_set_rows(Formation *f, int new_rows)
{
    if (new_rows < ROWS_MIN) new_rows = ROWS_MIN;
    if (new_rows > ROWS_MAX) new_rows = ROWS_MAX;
    if (new_rows == f->n_rows) return;

    int old_n_bats = f->n_bats;
    f->n_rows = new_rows;
    f->n_bats = bat_count(new_rows);

    if (f->state == FLIGHT_FLYING && f->n_bats > old_n_bats) {
        float lx = f->bats[0].px;
        float ly = f->bats[0].py;
        for (int k = old_n_bats; k < f->n_bats; k++)
            formation_place_bat(f, k, lx, ly);
    }
}

/* advance_to_next_heading — step angle_idx to the next compass preset and
 * adopt that heading (the +1-mod-N_ANGLES cycle through k_angles). */
static void advance_to_next_heading(Formation *f)
{
    f->angle_idx = (f->angle_idx + 1) % N_ANGLES;
    f->angle     = k_angles[f->angle_idx];
}

/* formation_advance — move every bat one tick along the shared velocity and
 * step its wing flap (wrapping the phase).  The whole wedge translates rigidly
 * because the delta is identical for all bats. */
static void formation_advance(Formation *f)
{
    for (int k = 0; k < f->n_bats; k++) {
        Bat *b = &f->bats[k];
        b->px    += f->vx * SECONDS_PER_TICK;
        b->py    += f->vy * SECONDS_PER_TICK;
        b->phase += 1.0f;                                   /* one tick of flap */
        if (b->phase >= (float)WING_CYCLE) b->phase -= (float)WING_CYCLE;
    }
}

static void formation_tick(Formation *f, float cx, float cy, int cols, int rows)
{
    if (f->state == FLIGHT_HOLDING) {                       /* waiting to relaunch */
        if (--f->timer <= 0) {
            advance_to_next_heading(f);
            formation_launch(f, cx, cy);
        }
        return;
    }

    formation_advance(f);                                  /* fly + flap          */
    if (leader_exited(f, cols, rows))                      /* gone? park & wait    */
        formation_hold(f, HOLD_TICKS);
}

/*
 * formation_init — place formation `slot` in its starting state at centre
 * (cx, cy): give it the shared depth, its preset colour and heading, then
 * either launch it (slot 0, flies immediately) or hold it for a staggered
 * entrance.  Held formations rewind angle_idx by one so the increment applied
 * on their first relaunch lands back on k_init_angle_idx[slot].
 */
static void formation_init(Formation *f, int slot, int n_rows, float cx, float cy)
{
    f->color     = k_colors[slot];
    f->n_rows    = n_rows;
    f->n_bats    = bat_count(n_rows);
    f->angle_idx = k_init_angle_idx[slot];
    f->angle     = k_angles[f->angle_idx];

    if (slot == 0) {
        formation_launch(f, cx, cy);
    } else {
        formation_hold(f, slot * STAGGER_TICKS);
        f->angle_idx = (k_init_angle_idx[slot] - 1 + N_ANGLES) % N_ANGLES;
    }
}

/* scene_init — USER-EVENT entry point (boot / 'r' reset / SIGWINCH resize).
 * NOT part of the per-tick combine. */
static void scene_init(Scene *s, int cols, int rows)
{
    s->cols   = cols;
    s->rows   = rows;
    s->paused = false;
    /* Preserve n_rows across resets; first call sets default. */
    if (s->n_rows < ROWS_MIN || s->n_rows > ROWS_MAX)
        s->n_rows = ROWS_DEFAULT;

    float cx = col_to_px(cols / 2);
    float cy = row_to_px(rows / 2);

    for (int i = 0; i < N_FORMATIONS; i++)
        formation_init(&s->formations[i], i, s->n_rows, cx, cy);
}

/*
 * scene_set_rows — resize all formations by delta (+1 or −1).
 * Clamps to [ROWS_MIN, ROWS_MAX].  Applied immediately: flying formations gain
 * or lose bats without waiting for the next reset.
 *
 * USER-EVENT entry point ('+' / '-'); NOT part of the per-tick combine.
 */
static void scene_set_rows(Scene *s, int delta)
{
    int new_rows = s->n_rows + delta;
    if (new_rows < ROWS_MIN || new_rows > ROWS_MAX) return;
    s->n_rows = new_rows;
    for (int i = 0; i < N_FORMATIONS; i++)
        formation_set_rows(&s->formations[i], new_rows);
}

/* scene_tick — THE per-tick combine.  The single place that advances
 * simulation state; nothing else does.  Per formation it runs the order
 * documented in the ARCHITECTURE block (HOLDING countdown→launch, or FLYING
 * advance→phase→leader-exit→hold). */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    float cx = col_to_px(s->cols / 2);
    float cy = row_to_px(s->rows / 2);
    for (int i = 0; i < N_FORMATIONS; i++)
        formation_tick(&s->formations[i], cx, cy, s->cols, s->rows);
}

/* ===================================================================== */
/* §8  render                      RENDER — state → screen, reads only    */
/* ===================================================================== */
/*
 * Mutates the ncurses frame buffer ONLY.  Every function reads simulation
 * state through const pointers (formation_draw / scene_draw) or recomputes
 * derived values (the flap `frame`, leader A_BOLD) at draw time — it never
 * writes Formation/Scene fields.  Driven once per FRAME from §10, separate
 * from the per-tick combine.
 */

/*
 * Wing glyphs — the four keyframes of one flap, as a triangle wave:
 *   frame   0    1    2    3     (then repeats)
 *   shape  /o\  -o-  \o/  -o-    up → level → down → level
 * k_bat_lw / k_bat_rw are the left / right wing chars for each frame; drawn on
 * either side of BAT_BODY they read as a flapping bat.  wing_frame() (below)
 * maps a phase to its frame index.
 * Body 'o' is direction-neutral so it looks right on any heading.  All glyphs
 * are plain ASCII (project ASCII-only rule).
 * Ref: Williams, *Animator's Survival Kit* — a looping keyframe cycle.
 */
static const char k_bat_lw[WING_FRAMES] = { '/', '-', '\\', '-' };  /* left wing, per frame  */
static const char k_bat_rw[WING_FRAMES] = { '\\', '-', '/', '-' };  /* right wing, per frame */
#define BAT_BODY 'o'                                                /* direction-neutral body */

/* wing_frame — which of the WING_FRAMES keyframes a flap phase falls in
 * (0..WING_FRAMES-1): ⌊phase·WING_FRAMES/WING_CYCLE⌋.  Clamped defensively so a
 * stray phase can never index past the glyph tables. */
static int wing_frame(float phase)
{
    int frame = (int)phase * WING_FRAMES / WING_CYCLE;
    if (frame < 0 || frame > WING_FRAMES - 1) frame = 0;
    return frame;
}

/* bat_glyph_on_screen — true when the 3-cell glyph (col-1 .. col+1 on row) sits
 * wholly inside the viewport, so drawing it clips nothing at an edge. */
static bool bat_glyph_on_screen(int col, int row, int cols, int rows)
{
    return row >= 0 && row < rows && col - 1 >= 0 && col + 1 < cols;
}

/* stamp_bat — paint one bat as left-wing / body / right-wing across three cells
 * in the formation's colour.  The leader (tip of the V) is bold to stand out. */
static void stamp_bat(WINDOW *w, int row, int col, int frame, ColorID color, bool leader)
{
    attr_t attr = COLOR_PAIR((int)color);
    if (leader) attr |= A_BOLD;
    wattron(w, attr);
    mvwaddch(w, row, col - 1, (chtype)(unsigned char)k_bat_lw[frame]);
    mvwaddch(w, row, col,     (chtype)BAT_BODY);
    mvwaddch(w, row, col + 1, (chtype)(unsigned char)k_bat_rw[frame]);
    wattroff(w, attr);
}

static void formation_draw(const Formation *f, WINDOW *w, int cols, int rows)
{
    if (f->state == FLIGHT_HOLDING) return;   /* parked off-screen: nothing to draw */

    for (int k = 0; k < f->n_bats; k++) {
        const Bat *b = &f->bats[k];
        int col = px_to_col(b->px);
        int row = px_to_row(b->py);
        if (!bat_glyph_on_screen(col, row, cols, rows)) continue;
        stamp_bat(w, row, col, wing_frame(b->phase), f->color, k == 0);
    }
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    for (int i = 0; i < N_FORMATIONS; i++)
        formation_draw(&s->formations[i], w, s->cols, s->rows);
}

/* hud_bar — fill a HUD row with spaces so it reads as a clean bar over the
 * animation (the bats are drawn first; the bar then covers that row). */
static void hud_bar(int row, int cols)
{
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
}

/* hud_title_stats — row 0: title (left) + live fps / Hz / run-state (right).
 * The stats block is right-justified but clamped to HUD_TITLE_COLS so it never
 * overruns the title on a narrow terminal. */
static void hud_title_stats(int cols, double fps, int sim_fps, bool paused)
{
    hud_bar(0, cols);
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, 1, " BAT FORMATIONS ");
    char rbuf[48];
    snprintf(rbuf, sizeof rbuf, " %5.1f fps  %3d Hz  %s ",
             fps, sim_fps, paused ? "PAUSED " : "running");
    int hx = cols - (int)strlen(rbuf);
    if (hx < HUD_TITLE_COLS) hx = HUD_TITLE_COLS;
    mvprintw(0, hx, "%s", rbuf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

/* hud_params — row 1: the tunable parameters (active theme, formation depth and
 * the bat count it implies).  No bold, so row 0 stays the dominant line. */
static void hud_params(int cols, int theme, int n_rows)
{
    hud_bar(1, cols);
    char pbuf[64];
    snprintf(pbuf, sizeof pbuf, " theme:%s %d/%d   rows:%d (%d bats) ",
             theme_names[theme], theme + 1, N_THEMES, n_rows,
             N_FORMATIONS * bat_count(n_rows));
    attron(COLOR_PAIR(COL_HUD));
    mvprintw(1, 1, "%s", pbuf);
    attroff(COLOR_PAIR(COL_HUD));
}

/* hud_keys — bottom row: the action legend, listing every interactive key. */
static void hud_keys(int cols, int rows)
{
    int brow = rows - 1;
    hud_bar(brow, cols);
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    mvprintw(brow, 1,
             "+/-:rows   p:pause   r:reset   t/T:theme   [/]:Hz   q:quit");
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

/*
 * screen_draw — render one frame: the bats, then the HUD overlaid in three
 * bands.  Bars are filled first so the animation never shows through the text.
 * Scene is read const, never mutated; `theme` is passed in (it lives on App, a
 * render setting).
 */
static void screen_draw(const Screen *s, const Scene *sc, double fps,
                        int sim_fps, int theme)
{
    erase();
    scene_draw(sc, stdscr);
    hud_title_stats(s->cols, fps, sim_fps, sc->paused);
    hud_params(s->cols, theme, sc->n_rows);
    hud_keys(s->cols, s->rows);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  screen                      terminal lifecycle (init/resize/free)  */
/* ===================================================================== */
/* Mutates terminal mode + the Screen{cols,rows} cache.  USER-EVENT / setup
 * (boot, SIGWINCH); not part of the per-tick combine. */

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

/* ===================================================================== */
/* §10  app                        input events + fixed-timestep loop     */
/* ===================================================================== */
/*
 * PERFORMANCE: the main loop owns the fixed-timestep accumulator, the 60 fps
 * frame cap and the fps display — it advances timing state, never sim state
 * directly.  It drives the layers in this per-FRAME order:
 *   resize (USER EVENT) → SIM combine (scene_tick, N× per frame) → fps calc
 *   → sleep → RENDER (screen_draw/present) → input (USER EVENT, app_handle_key)
 * USER EVENTS (resize, keys) mutate state OUTSIDE scene_tick — see ARCHITECTURE.
 */

/*
 * App — the running program: its sub-systems plus everything that lives for the
 * whole session rather than for a single scene.
 *
 * WHY these fields sit together (three concerns, one owner):
 *   scene, screen        — the two sub-systems the loop drives each frame.
 *   sim_fps, theme       — session SETTINGS that must survive a scene reset ('r')
 *                          and a resize; held here, off Scene, so the rebuild
 *                          literally cannot touch them.  theme is a RENDER choice;
 *                          both start at their zero/default via static init of
 *                          g_app (theme 0 = NEON; sim_fps is set in main()).
 *   running, need_resize — signal flags.  volatile sig_atomic_t because they are
 *                          WRITTEN from signal handlers (about the only thing a
 *                          handler may safely do) and READ in the main loop;
 *                          the qualifiers stop the compiler caching/ tearing them.
 *
 * Ref: Fiedler, "Fix Your Timestep!" — the fixed-timestep loop in main() that
 * consumes sim_fps.
 */
typedef struct {
    Scene                 scene;       /* the simulated world (§5/§7)              */
    Screen                screen;      /* terminal geometry cache (§9)             */
    int                   sim_fps;     /* tick rate (Hz), SIM_FPS_MIN..MAX, [ / ]  */
    int                   theme;       /* palette index 0..N_THEMES-1, t / T       */
    volatile sig_atomic_t running;     /* 0 ⇒ leave the loop (SIGINT/SIGTERM, q)   */
    volatile sig_atomic_t need_resize; /* 1 ⇒ rebuild on next frame (SIGWINCH)     */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case 'p': case 'P': case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    /* Formation size — + / = to grow, - to shrink */
    case '+': case '=':
        scene_set_rows(&app->scene, +1);
        break;
    case '-': case '_':
        scene_set_rows(&app->scene, -1);
        break;

    case ']':
        if (app->sim_fps < SIM_FPS_MAX) app->sim_fps += SIM_FPS_STEP;
        break;
    case '[':
        if (app->sim_fps > SIM_FPS_MIN) app->sim_fps -= SIM_FPS_STEP;
        break;

    case 't':
        app->theme = (app->theme + 1) % N_THEMES;
        apply_theme(app->theme);
        break;
    case 'T':
        app->theme = (app->theme + N_THEMES - 1) % N_THEMES;
        apply_theme(app->theme);
        break;

    default: break;
    }
    return true;
}

/* app_init — one-time start-up: seed the RNG, install signal handlers, bring up
 * the terminal, then build the scene at the current size.  scene is zeroed
 * before scene_init so its n_rows starts at 0 and picks up ROWS_DEFAULT. */
static void app_init(App *app)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    memset(&app->scene, 0, sizeof app->scene);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
}

int main(void)
{
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        /* 1. apply any pending resize before this frame is measured */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 2. measure elapsed time, clamped so a stall can't spiral the sim */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        /* 3. advance the sim: one fixed tick per accumulated tick-interval */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

        /* 4. refresh the fps readout a couple of times a second */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. cap the frame rate, then draw the frame */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps,
                    app->theme);
        screen_present();

        /* 6. drain one input event (may flip running or mutate the scene) */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
