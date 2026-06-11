/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * jellyfish.c — bioluminescent jellyfish simulation
 *
 * Physics-based pulse locomotion:
 *   IDLE (sinks) → CONTRACT (jet thrust) → GLIDE (coasts) → EXPAND (blooms)
 *
 * Keys:
 *   q / ESC   quit
 *   spc       pause / resume
 *   r         reset
 *   + / -     add / remove jellyfish (1–8)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/jellyfish.c -o jellyfish -lncurses -lm
 *
 * §1 config       §2 performance  §3 logic
 * §4 simulation   §5 render        §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : State-machine locomotion — each jellyfish cycles through
 *                  4 states: IDLE (passive sinking), CONTRACT (jet thrust),
 *                  GLIDE (coast on momentum), EXPAND (bell re-opens).
 *                  Bell shape is a parametric ellipse with time-varying
 *                  radii; tentacles follow the segmented-chain follower model.
 *
 * Physics        : Jet propulsion — a one-shot upward impulse (vy = -JET_THRUST)
 *                  lands at the END of CONTRACT; GRAVITY pulls down only while
 *                  IDLE (the bell sinks between pulses); exponential drag
 *                  (DRAG_VY) bleeds vy every frame so each pulse rises, coasts,
 *                  then settles.  Net effect: jet up, rest down.
 *
 * Math           : Bell outline is a half-ellipse — width Rx = BELL_RX_BASE *
 *                  head_f scales with the contraction factor bell_open ∈
 *                  [MIN_OPEN, 1.0]; height Ry = BELL_RY_BASE is fixed.  The
 *                  cell-aspect correction is baked into the 1:2 radius ratio
 *                  (CELL_W=8, CELL_H=16) so the bell reads round despite tall
 *                  character cells.
 *
 * Rendering      : Bell drawn as a scan-line half-ellipse banded by height —
 *                  roof '_', shoulders '( )', steep walls '/ \\', flat rim '-',
 *                  dim '.' interior.  Tentacles are travelling-wave chains
 *                  tapered by depth ('|' near the bell → ':' → '.' at the tips).
 *                  Each jelly owns a distinct bright colour pair for the glow.
 *
 * References     :
 *   Biology / propulsion physics (the IDLE→CONTRACT→GLIDE→EXPAND cycle)
 *     [1] Dabiri, Colin, Costello & Gharib (2005), "Flow patterns generated
 *         by oblate medusan jellyfish," J. Exp. Biol. 208:1257-1265 — the
 *         contract-jet / relax-refill pulse this state machine imitates.
 *     [2] Gemmell, Costello, Colin et al. (2013), "Passive energy recapture
 *         in jellyfish...," PNAS 110(44):17904 — why the GLIDE/coast phase
 *         carries the animal so far on one pulse (momentum + drag, §4).
 *   Motion math (bell open-fraction curves, §3 curve_contract/expand)
 *     [3] Penner, R. (2002), "Robert Penner's Easing Functions" — the
 *         ease-out (snap-shut) vs ease-in (bloom) families used here.
 *     [4] Perlin, K. (2002), "Improving Noise," ACM SIGGRAPH — origin of the
 *         3t²−2t³ smoothstep used for the EXPAND bloom.
 *   Tentacle chain & inertia (§5 draw_tentacles)
 *     [5] Jakobsen, T. (2001), "Advanced Character Physics," GDC — segmented
 *         constraint chains and the trailing-lag idea behind tentacle inertia.
 *   Loop & geometry
 *     [6] Fiedler, G., "Fix Your Timestep!", gafferongames.com — the capped
 *         fixed-dt integration loop in §6.
 *     [7] Lengyel, E., "Mathematics for 3D Game Programming and Computer
 *         Graphics" — parametric ellipse r(θ) and the cell-aspect correction.
 *   Rendering substrate
 *     [8] Padala, P., "NCURSES Programming HOWTO" (TLDP) — windows, colour
 *         pairs, and the erase→draw→refresh frame model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * The program has FOUR real layers; two are intentionally absent.
 *
 *   LAYER        SECTION  MUTATES
 *   ────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — timing primitives (frame cap lives in §6)
 *   LOGIC        §3       nothing — pure functions of their arguments only
 *   SIMULATION   §4       Jellyfish.{cx,cy,vy,bell_open,phase,phase_t,idle_dur,
 *                           wave_scale,drift_*,tent_phase,crown_f,head_f}
 *                           and Scene.{n_jellies,paused,max_px,max_py}
 *                           (also advances the global PRNG via randf)
 *   RENDER       §5       the terminal only (ncurses); READS sim state,
 *                           never writes it.  Screen.{cols,rows} set here too.
 *
 *   EFFECTS  : ABSENT — glow / translucency / colour are derived at render
 *              time from animation state (bell_open, segment depth, vy), never
 *              stored as separate cosmetic state.
 *   DELAYS   : ABSENT — the only pause is Scene.paused (user-toggled, checked
 *              once at the combine point); inter-pulse holds are encoded as
 *              durations inside the SIMULATION state machine (idle_dur, DUR_*).
 *
 * LOGIC (§3) is provably uncorruptable from RENDER/EFFECTS: it does no
 * mutation and no I/O, so deleting or reordering any draw call cannot change
 * what curve_contract/curve_expand/smoothstep return.
 *
 * Per-tick combine order — main() (§6) is the ONLY place that advances state,
 * and scene_tick() is the ONLY mutator of simulation state:
 *     1. PERFORMANCE  measure dt (capped at 100 ms)        §6
 *     2. SIMULATION   scene_tick(dt)   ← sole sim mutator  §4
 *     3. PERFORMANCE  accumulate fps counter               §6
 *     4. RENDER       screen_render()  (read-only)         §5
 *     5. PERFORMANCE  sleep to frame cap                   §6
 *
 * User events (key, reset, resize, signal) MAY mutate Scene/Screen but are
 * NOT part of the tick — they run after render, outside the combine order:
 * app_key / screen_resize / on_signal (§6).
 *
 * TYPE GLOSSARY (every name is a noun a reader could look up):
 *   PulsePhase  §4  one phase of the medusan swim cycle (jet → coast → refill)
 *   Jellyfish   §4  one simulated medusa (the WHAT)
 *   Scene       §4  the whole simulated world (WHAT + HOW knobs + WHERE bounds)
 *   Screen      §5  the ncurses display surface (terminal cells)
 *   App         §5  process-level glue: world + display + signal flags
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

#define CELL_W   8
#define CELL_H  16

enum {
    N_JELLIES_DEFAULT = 1,
    N_JELLIES_MAX     = 8,
    N_TENTACLES       = 3,     /* left, center, right                     */
    TENTACLE_SEGS     = 20,
    FPS_UPDATE_MS     = 500,
    TARGET_FPS        = 60,
};

/* Bell geometry (pixels) */
#define BELL_RX_BASE    72.0f
#define BELL_RY_BASE   144.0f

/* Minimum open fraction during contraction — needs room to produce thrust */
#define MIN_OPEN         0.85f  /* crown height minimum (height axis)     */
#define HEAD_MIN_OPEN    0.85f  /* body width minimum — squeezes harder   */

/* Bell silhouette bands, keyed by |ny| (normalized dome height: 1=apex, 0=rim).
 * draw_dome_row classifies each scan-row into one band and emits its glyphs. */
#define BELL_NY_ROOF      0.94f  /* |ny|>this: flat closing roof  '_'         */
#define BELL_NY_CROWN     0.88f  /* roof..this: rounded shoulders '(' ')'     */
#define BELL_CROWN_SPAN   0.12f  /* vertical span of the crown (1-BELL_NY_CROWN) */
#define BELL_NY_SHOULDER  0.40f  /* crown..this: steep side walls '/' '\\'    */
#define BELL_NY_RIM       0.14f  /* below this: flat rim line     '-'         */
#define BELL_INTERIOR_R2  0.60f  /* r²>this inside body: translucent dots '.' */

/* ── Physics-based pulse locomotion ─────────────────────────────────── */
#define GRAVITY          30.0f  /* downward accel px/s² while idle (sinking) */
#define JET_THRUST      190.0f  /* upward velocity kick at peak contraction   */
#define DRAG_VY           2.4f  /* exponential drag on vertical velocity      */
#define IDLE_DUR_MIN      0.55f /* min time bell stays open between pulses    */
#define IDLE_DUR_MAX      1.10f /* max time bell stays open between pulses    */
#define DUR_CONTRACT      0.19f /* bell snap-shut  (fast ease-out)            */
#define DUR_GLIDE         0.38f /* coasting phase after thrust                */
#define DUR_EXPAND        0.68f /* bell bloom-open (slow ease-in)             */
#define TENT_LAG          0.09f /* tentacle inertia — trails on fast move     */

/* Tentacle sway-amplitude (wave_scale) levels across the pulse cycle */
#define WAVE_FULL          1.00f /* full sway — IDLE / fully bloomed bell      */
#define WAVE_GLIDE         0.12f /* near-straight streaming during GLIDE       */
#define WAVE_CONTRACT_END  0.15f /* sway remaining at the end of CONTRACT      */
#define WAVE_CONTRACT_DROP 0.85f /* sway lost over CONTRACT (WAVE_FULL→0.15)   */
#define WAVE_EXPAND_RISE   0.88f /* sway regained over EXPAND (0.12→WAVE_FULL) */

/* Tentacle wave dynamics */
#define TENT_SEG_PX     14.0f
#define TENT_WAVE_AMP   19.0f
#define TENT_WAVE_K      0.30f
#define TENT_WAVE_SPD    3.2f
#define TENT_PHASE_OFF   0.80f

/* Tentacle taper bands by depth (0=root, 1=tip): glyph + intensity per band */
#define TENT_DEPTH_NEAR   0.20f  /* root..near: bold  '|'                    */
#define TENT_DEPTH_MID    0.45f  /* near..mid:  plain '|'                    */
#define TENT_DEPTH_FAR    0.70f  /* mid..far:   dim   ':' ; beyond: dim '.'  */

/* Horizontal oscillation */
#define JELLY_DRIFT_A   15.0f
#define JELLY_DRIFT_HZ   0.30f

/* Spawn / respawn layout — fractions of the world box (px = fraction*max_*). */
#define SPAWN_HERO_CY     0.55f  /* lone hero jelly starts just below mid       */
#define SPAWN_ZONE_MARGIN 0.10f  /* left inset within each fleet lane           */
#define SPAWN_ZONE_FILL   0.80f  /* usable width of a lane                      */
#define SPAWN_BAND_TOP    0.85f  /* topmost spread row (fraction of height)     */
#define SPAWN_BAND_RANGE  0.70f  /* vertical spread span across the fleet       */
#define SPAWN_JITTER      0.25f  /* random cy jitter as fraction of a lane      */
#define SPAWN_BELOW       0.40f  /* extra below-screen offset when entering     */
#define RESPAWN_CY        0.85f  /* recycle drop-in height (fraction)           */
#define RESPAWN_LANE_LO   0.15f  /* recycle lane low bound (fraction)           */
#define RESPAWN_LANE_SPAN 0.70f  /* recycle lane span (fraction)               */

#define TAU  6.28318530f
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Colour-pair identifiers + per-jelly palette table.
 * Shared constants: SIMULATION (§4 jelly_spawn) assigns j->cp from jelly_cp[];
 * RENDER (§5 color_init) binds the actual colours.  No state, no I/O here. */
enum {
    CP_J0 = 1, CP_J1, CP_J2, CP_J3,
    CP_J4,     CP_J5, CP_J6, CP_J7,
    CP_HUD, CP_HINT,
};

static const int jelly_cp[N_JELLIES_MAX] = {
    CP_J0, CP_J1, CP_J2, CP_J3, CP_J4, CP_J5, CP_J6, CP_J7
};

/* ===================================================================== */
/* §2  performance — timing primitives (frame cap applied in §6)          */
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
/* §3  logic — pure decisions: no mutation, no I/O, readable in isolation */
/* ===================================================================== */

static float smoothstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* Bell open-fraction curves:
 *   contract: ease-out (fast at start, slow at end) — snap shut
 *   expand:   ease-in  (slow at start, fast at end) — gentle bloom    */
static float curve_contract(float t) /* t in [0,1] */
{
    float inv = 1.0f - t;
    return 1.0f - inv * inv;   /* ease-out quadratic */
}
static float curve_expand(float t)
{
    return smoothstep(t);      /* smoothstep: ease-in-out but starts slow */
}

/* ===================================================================== */
/* §4  simulation — advances state (sole mutator of Jellyfish / Scene)    */
/* ===================================================================== */

/* PulsePhase — the four phases of the medusan jet-propulsion swim cycle.
 *
 * WHY a state machine: a real jellyfish cannot thrust continuously — the bell
 * is essentially one muscle that can only CONTRACT (eject water = thrust) and
 * then passively RELAX (refill).  Locomotion is therefore inherently cyclic,
 * so a phase machine reproduces the pulse-glide-sink rhythm for free: each
 * phase owns a different physics rule (thrust vs. gravity vs. drag) AND a
 * different bell shape, and a single enum selects both at once in
 * jelly_pulse_tick — far simpler than a continuous force field.
 *
 * CONTEXT / REFS: the contract-jet then relax-refill sequence follows Dabiri,
 * Colin, Costello & Gharib (2005) [1]; the long coasting GLIDE that lets one
 * pulse carry the animal far is the "passive energy recapture" of Gemmell et
 * al. (2013) [2].  Order is fixed and wraps forever:
 *   IDLE → CONTRACT → GLIDE → EXPAND → IDLE
 * Per-phase durations are config literals in §1 (IDLE_DUR_*, DUR_*).         */
typedef enum {
    PHASE_IDLE,      /* bell fully open (1.0); passive sink under GRAVITY;
                      * waits a randomised idle_dur, then fires a contraction  */
    PHASE_CONTRACT,  /* bell snaps 1.0→MIN_OPEN (ease-out, DUR_CONTRACT); a
                      * JET_THRUST upward velocity kick lands at its very end   */
    PHASE_GLIDE,     /* bell held at MIN_OPEN; coast upward on momentum while
                      * DRAG_VY bleeds velocity off — the free-distance phase   */
    PHASE_EXPAND,    /* bell blooms MIN_OPEN→1.0 (ease-in, DUR_EXPAND) as it
                      * refills; sway returns, then loops back to PHASE_IDLE    */
} PulsePhase;

/* Jellyfish — the complete state of one simulated medusa.
 *
 * WHY one flat struct: the animal is a point-mass bell (a parametric half-
 * ellipse, ref [7]) trailing a few sine-driven tentacle chains (ref [5]).  All
 * of its appearance is driven by ONE scalar — bell_open, the muscle
 * contraction — so the struct is deliberately flat rather than split into
 * sub-objects: every other shape value is a cheap derivation of bell_open,
 * recomputed each tick.  Splitting it would scatter that single cause.
 *
 * DATA-FLOW (the field groups below are ordered along this chain):
 *   phase + phase_t  ──drive──▶  bell_open + wave_scale   (jelly_pulse_tick)
 *   bell_open        ──drive──▶  crown_f, head_f          (render fractions)
 *   vy (physics)     ──drive──▶  cy (position) + tentacle inertia lag
 * The ONLY real integrators are vy and cy; bell_open is a function of the
 * phase clock; crown_f/head_f/wave_scale are pure functions stored once so the
 * renderer (§5) never recomputes them nor knows the phase machine exists.
 *
 * UNITS: all lengths are PIXELS (sub-cell units; CELL_W=8, CELL_H=16 px per
 * terminal cell).  The single pixel→cell conversion happens at draw time.    */
typedef struct {
    /* ── Position & lateral drift — pixel space (integrated in jelly_tick) ─ *
     * Vertical motion is true physics; horizontal motion is purely cosmetic:
     * cx is a fixed-amplitude sine wobble about drift_cx, not a force.       */
    float cx, cy;            /* bell-centre position (px); only cy integrates  */
    float drift_cx;          /* midpoint that cx oscillates about (px)         */
    float drift_phase;       /* phase of the horizontal drift sine (rad)       */

    /* ── Pulse-cycle locomotion — the swim engine (jelly_pulse_tick) ─────── *
     * phase/phase_t are the PulsePhase state machine; vy is the lone velocity
     * integrator; bell_open is the muscle signal all shape values derive from. */
    float      vy;           /* vertical velocity (px/s); NEGATIVE = upward.
                              * Set to -JET_THRUST at the end of CONTRACT, then
                              * decayed every frame by exp(-DRAG_VY*dt)         */
    float      bell_open;    /* master contraction ∈ [MIN_OPEN..1]: 1=relaxed/
                              * open, MIN_OPEN=fully contracted. Sole driver of
                              * crown_f & head_f                                */
    float      idle_dur;     /* this IDLE's hold time (s), re-rolled in
                              * [IDLE_DUR_MIN..MAX] each cycle so separate
                              * jellies never lock into a synchronised pulse    */
    PulsePhase phase;        /* current swim phase — selects the physics rule  */
    float      phase_t;      /* seconds elapsed in `phase`; compared against
                              * that phase's duration to trigger the next one   */

    /* ── Tentacle wave — sway kinematics (read by draw_tentacles) ───────── *
     * Tentacles are NOT simulated as masses; each is a travelling sine wave
     * whose amplitude is gated by wave_scale, so sway dies during the jet.   */
    float tent_phase[N_TENTACLES];  /* travelling-wave phase per tentacle (rad),
                                     * advanced at TENT_WAVE_SPD*wave_scale     */
    float wave_scale;        /* sway-amplitude gate ∈ [~0.12..1]: 1=full sway
                              * (IDLE), ~0.12=streaming straight back (GLIDE)   */

    /* ── Render shape fractions — pure functions of bell_open (draw_bell) ── *
     * Cached at the tail of jelly_pulse_tick so the renderer stays oblivious
     * to the phase machine — it just scales geometry by these.               */
    float crown_f;           /* dome HEIGHT scale = bell_open; apex descends
                              * as it shrinks (crown squashes vertically)       */
    float head_f;            /* dome WIDTH scale: bell_open remapped into
                              * [HEAD_MIN_OPEN..1] so width squeezes harder     */

    /* ── Bioluminescent colour ──────────────────────────────────────────── */
    int cp;                  /* ncurses colour-pair id, chosen from jelly_cp[]
                              * by spawn index → each jelly a distinct hue      */
} Jellyfish;

/* Utility: advances the global PRNG — NOT pure, hence not in §3 logic. */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* Place a freshly spawned jelly in the world.  idx 0 with spread = the lone
 * "hero" centred near mid-height; otherwise lane the fleet across the width and
 * either stagger it in height (spread) or start it just below the screen
 * (entering, so the new jelly swims up into view). */
static void jelly_seed_position(Jellyfish *j, int idx,
                                float max_px, float max_py, bool spread)
{
    if (idx == 0 && spread) {
        j->drift_cx = max_px * 0.5f;
        j->cx       = j->drift_cx;
        j->cy       = max_py * SPAWN_HERO_CY;
        return;
    }

    /* Horizontal: a random x inside this jelly's vertical lane */
    float zone_w  = max_px / (float)N_JELLIES_MAX;
    float zone_lo = zone_w * (idx % N_JELLIES_MAX) + zone_w * SPAWN_ZONE_MARGIN;
    float zone_hi = zone_lo + zone_w * SPAWN_ZONE_FILL;
    j->drift_cx   = zone_lo + randf() * (zone_hi - zone_lo);
    j->cx         = j->drift_cx;

    /* Vertical: spread fans the fleet down the screen; entering starts below */
    float slot   = (float)(idx % N_JELLIES_MAX) / (float)(N_JELLIES_MAX - 1);
    float jitter = (max_py / (float)N_JELLIES_MAX) * SPAWN_JITTER;
    j->cy = spread
          ? max_py * (SPAWN_BAND_TOP - SPAWN_BAND_RANGE * slot)
            + (randf() - 0.5f) * jitter
          : max_py + TENTACLE_SEGS * TENT_SEG_PX * SPAWN_BELOW + BELL_RY_BASE;
}

static void jelly_spawn(Jellyfish *j, int idx,
                        float max_px, float max_py, bool spread)
{
    jelly_seed_position(j, idx, max_px, max_py, spread);

    /* Randomise sway / drift phases and pick this jelly's bioluminescent hue */
    j->drift_phase = randf() * TAU;
    for (int k = 0; k < N_TENTACLES; k++)
        j->tent_phase[k] = randf() * TAU;
    j->cp = jelly_cp[idx % N_JELLIES_MAX];

    /* Begin a fresh idle cycle, with phase_t randomised so pulses don't sync */
    j->vy         = 0.0f;
    j->bell_open  = 1.0f;
    j->wave_scale = WAVE_FULL;
    j->phase      = PHASE_IDLE;
    j->phase_t    = randf() * IDLE_DUR_MAX;   /* stagger so not all sync  */
    j->idle_dur   = IDLE_DUR_MIN + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);

    j->crown_f = j->head_f = 1.0f;
}

/* Map the muscle signal bell_open → the geometry fractions the renderer reads.
 * crown_f scales dome HEIGHT 1:1; head_f remaps openness into [HEAD_MIN_OPEN..1]
 * so the body squeezes width-wise harder than the crown drops.  Pure cache:
 * called once at the tail of jelly_pulse_tick, never by the renderer. */
static void bell_set_render_fractions(Jellyfish *j)
{
    float open01 = (j->bell_open - MIN_OPEN) / (1.0f - MIN_OPEN); /* 0=shut 1=open */
    j->crown_f = j->bell_open;
    j->head_f  = HEAD_MIN_OPEN + (1.0f - HEAD_MIN_OPEN) * open01;
}

/* Advance the swim-phase state machine one step (sets bell_open + wave_scale +
 * vy thrust per phase), then derive the render fractions from bell_open. */
static void jelly_pulse_tick(Jellyfish *j, float dt)
{
    j->phase_t += dt;

    switch (j->phase) {

    case PHASE_IDLE:
        /* Bell fully open; passive sinking; tentacles sway freely */
        j->bell_open  = 1.0f;
        j->wave_scale = WAVE_FULL;
        j->vy += GRAVITY * dt;
        if (j->phase_t >= j->idle_dur) {
            j->phase   = PHASE_CONTRACT;
            j->phase_t = 0.0f;
        }
        break;

    case PHASE_CONTRACT: {
        /* Bell snaps shut — ease-out (fast at start).
         * Tentacles pull inward as bell contracts: sway shrinks to ~15%. */
        float frac = j->phase_t / DUR_CONTRACT;
        if (frac > 1.0f) frac = 1.0f;
        j->bell_open  = MIN_OPEN + (1.0f - MIN_OPEN) * (1.0f - curve_contract(frac));
        j->wave_scale = WAVE_FULL - WAVE_CONTRACT_DROP * frac;   /* 1.0 → 0.15 */
        if (j->phase_t >= DUR_CONTRACT) {
            j->bell_open  = MIN_OPEN;
            j->wave_scale = WAVE_CONTRACT_END;
            j->vy         = -JET_THRUST;
            j->phase      = PHASE_GLIDE;
            j->phase_t    = 0.0f;
        }
        break;
    }

    case PHASE_GLIDE:
        /* Coasting upward — tentacles stream nearly straight behind */
        j->bell_open  = MIN_OPEN;
        j->wave_scale = WAVE_GLIDE;
        if (j->phase_t >= DUR_GLIDE) {
            j->phase   = PHASE_EXPAND;
            j->phase_t = 0.0f;
        }
        break;

    case PHASE_EXPAND: {
        /* Bell blooms open slowly; tentacle sway gradually returns */
        float frac = j->phase_t / DUR_EXPAND;
        if (frac > 1.0f) frac = 1.0f;
        j->bell_open  = MIN_OPEN + (1.0f - MIN_OPEN) * curve_expand(frac);
        j->wave_scale = WAVE_GLIDE + WAVE_EXPAND_RISE * smoothstep(frac); /* 0.12→1.0 */
        if (j->phase_t >= DUR_EXPAND) {
            j->bell_open  = 1.0f;
            j->wave_scale = WAVE_FULL;
            j->phase      = PHASE_IDLE;
            j->phase_t    = 0.0f;
            j->idle_dur   = IDLE_DUR_MIN
                          + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);
        }
        break;
    }
    }

    bell_set_render_fractions(j);   /* bell_open → crown_f / head_f */
}

/* Recycle a jelly that has fully swum off the top of the screen: drop it back
 * to mid-height in a fresh horizontal lane and restart its idle cycle, so the
 * fleet keeps streaming upward forever (wrap-around respawn). */
static void jelly_wrap_offscreen(Jellyfish *j, float max_px, float max_py)
{
    float tent_extent = TENTACLE_SEGS * TENT_SEG_PX;
    if (j->cy + tent_extent >= 0.0f) return;   /* still (partly) on screen */

    j->cy          = max_py * RESPAWN_CY;
    j->drift_cx    = max_px * (RESPAWN_LANE_LO + RESPAWN_LANE_SPAN * randf());
    j->drift_phase = randf() * TAU;
    j->vy          = 0.0f;
    j->phase       = PHASE_IDLE;
    j->phase_t     = 0.0f;
    j->idle_dur    = IDLE_DUR_MIN + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);
    j->bell_open   = 1.0f;
    j->wave_scale  = WAVE_FULL;
    j->crown_f = j->head_f = 1.0f;
}

static void jelly_tick(Jellyfish *j, float dt, float max_px, float max_py)
{
    jelly_pulse_tick(j, dt);                       /* advance swim phase + shape */

    j->vy *= expf(-DRAG_VY * dt);                  /* exponential vertical drag  */

    /* Advance oscillator phases (drift sine + per-tentacle sway) */
    j->drift_phase += TAU * JELLY_DRIFT_HZ * dt;
    for (int k = 0; k < N_TENTACLES; k++)
        j->tent_phase[k] += TENT_WAVE_SPD * j->wave_scale * dt;

    /* Integrate position: vertical from velocity, horizontal from drift sine */
    j->cy += j->vy * dt;
    j->cx  = j->drift_cx + JELLY_DRIFT_A * sinf(j->drift_phase);

    /* Clamp horizontal so drift can't walk off screen */
    if (j->cx < BELL_RX_BASE)            j->cx = BELL_RX_BASE;
    if (j->cx > max_px - BELL_RX_BASE)   j->cx = max_px - BELL_RX_BASE;

    jelly_wrap_offscreen(j, max_px, max_py);       /* recycle if it left the top */
}

/* Scene — the entire simulated world in one aggregate (the WHAT + HOW + WHERE).
 *
 * WHY a fixed pool: jellies[] is allocated to N_JELLIES_MAX up front and is
 * NEVER grown or freed at runtime — n_jellies is merely how many of those
 * fixed slots are currently live.  This honours the project's no-malloc-in-the-
 * hot-path rule: the '+' key spawns into an already-allocated slot and '-'
 * just lowers the count; nothing allocates after init.
 *
 * WHERE / units: max_px/max_py are the world extent in PIXELS, derived from the
 * terminal size (cols*CELL_W, rows*CELL_H) at init and on every resize.  The
 * simulation clamps and wraps against them, so behaviour is resolution-
 * independent — the same pulse looks the same in any terminal size.          */
typedef struct {
    Jellyfish jellies[N_JELLIES_MAX]; /* WHAT: fixed pool; only the first
                                       * n_jellies are simulated and drawn      */
    int       n_jellies;     /* HOW: live count ∈ [1..N_JELLIES_MAX] (+/- keys) */
    bool      paused;        /* HOW: freezes scene_tick; rendering still runs   */
    float     max_px, max_py;/* WHERE: world bounds in px (cols*CELL_W,
                              * rows*CELL_H); reset at init and on every resize  */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->max_px    = (float)(cols * CELL_W);
    s->max_py    = (float)(rows * CELL_H);
    s->n_jellies = N_JELLIES_DEFAULT;
    s->paused    = false;
    for (int i = 0; i < N_JELLIES_MAX; i++)
        jelly_spawn(&s->jellies[i], i, s->max_px, s->max_py, /*spread=*/true);
}

/* The ONE per-tick simulation advance. Honours the paused delay-guard. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    for (int i = 0; i < s->n_jellies; i++)
        jelly_tick(&s->jellies[i], dt, s->max_px, s->max_py);
}

/* ===================================================================== */
/* §5  render — state → screen; reads only, never mutates sim state        */
/* ===================================================================== */

/* coords: one-way pixel→cell mapping used only at draw time */
static inline int px_to_col(float px) { return (int)floorf(px / (float)CELL_W); }
static inline int px_to_row(float py) { return (int)floorf(py / (float)CELL_H); }

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_J0,  51, -1);
        init_pair(CP_J1, 105, -1);
        init_pair(CP_J2, 213, -1);
        init_pair(CP_J3,  86, -1);
        init_pair(CP_J4, 159, -1);
        init_pair(CP_J5, 141, -1);
        init_pair(CP_J6, 219, -1);
        init_pair(CP_J7, 123, -1);
        init_pair(CP_HUD,  226, -1);   /* bright yellow on default bg */
        init_pair(CP_HINT,  51, -1);   /* bright cyan   on default bg */
    } else {
        init_pair(CP_J0, COLOR_CYAN,    -1);
        init_pair(CP_J1, COLOR_BLUE,    -1);
        init_pair(CP_J2, COLOR_MAGENTA, -1);
        init_pair(CP_J3, COLOR_GREEN,   -1);
        init_pair(CP_J4, COLOR_WHITE,   -1);
        init_pair(CP_J5, COLOR_CYAN,    -1);
        init_pair(CP_J6, COLOR_MAGENTA, -1);
        init_pair(CP_J7, COLOR_BLUE,    -1);
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

/* Render one scan-row of the bell dome: classify it into a silhouette band by
 * |ny| (BELL_NY_* thresholds, §1) and emit that band's glyphs between the
 * ellipse's left/right edges.  Returning early = "this row is done". */
static void draw_dome_row(WINDOW *win, const Jellyfish *j,
                          int r, float ny, int cols)
{
    float abs_ny  = fabsf(ny);
    float nx_edge = sqrtf(1.0f - ny * ny);          /* ellipse half-width at ny */
    float Rx      = BELL_RX_BASE * j->head_f;        /* width pulses with head_f */
    int   cl      = px_to_col(j->cx - Rx * nx_edge);
    int   cr      = px_to_col(j->cx + Rx * nx_edge);

    /* closing roof */
    if (abs_ny > BELL_NY_ROOF) {
        wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
        for (int c = cl; c <= cr; c++)
            if (c >= 0 && c < cols) mvwaddch(win, r, c, '_');
        wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
        return;
    }

    /* rounded crown */
    if (abs_ny > BELL_NY_CROWN) {
        wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
        if (cl >= 0 && cl < cols) mvwaddch(win, r, cl, '(');
        if (cr >= 0 && cr < cols && cr != cl) mvwaddch(win, r, cr, ')');
        for (int c = cl + 1; c < cr; c++)
            if (c >= 0 && c < cols) mvwaddch(win, r, c, '_');
        wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
        return;
    }

    /* edge characters — steep walls vs. gentle shoulders vs. rim */
    chtype lch, rch;
    attr_t ea;
    if (abs_ny > BELL_NY_SHOULDER) {
        lch = '/';  rch = '\\'; ea = A_BOLD;
    } else if (abs_ny > BELL_NY_RIM) {
        lch = '(';  rch = ')';  ea = A_BOLD;
    } else {
        lch = '~';  rch = '~';  ea = A_NORMAL;
    }

    if (cl >= 0 && cl < cols) {
        wattron(win, COLOR_PAIR(j->cp) | ea);
        mvwaddch(win, r, cl, lch);
        wattroff(win, COLOR_PAIR(j->cp) | ea);
    }
    if (cr >= 0 && cr < cols && cr != cl) {
        wattron(win, COLOR_PAIR(j->cp) | ea);
        mvwaddch(win, r, cr, rch);
        wattroff(win, COLOR_PAIR(j->cp) | ea);
    }

    /* rim line */
    if (abs_ny <= BELL_NY_RIM) {
        wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
        for (int c = cl; c <= cr; c++)
            if (c >= 0 && c < cols) mvwaddch(win, r, c, '-');
        wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
        return;
    }

    /* interior translucent dots — only the part of the body outside the core */
    for (int c = cl + 1; c < cr; c++) {
        if (c < 0 || c >= cols) continue;
        float pcx = c * (float)CELL_W + (float)CELL_W * 0.5f;
        float nx  = (pcx - j->cx) / (BELL_RX_BASE * j->head_f + 0.001f);
        float r2  = nx * nx + ny * ny;
        if (r2 < BELL_INTERIOR_R2) continue;
        wattron(win, COLOR_PAIR(j->cp) | A_DIM);
        mvwaddch(win, r, c, '.');
        wattroff(win, COLOR_PAIR(j->cp) | A_DIM);
    }
}

/*
 * draw_bell() — scan-line render of the dome (a parametric half-ellipse, ref [7]).
 * Reads as: clip the crown for the contraction, find the rows the bell spans,
 * then hand each row to draw_dome_row to classify and draw its band.
 */
static void draw_bell(WINDOW *win, const Jellyfish *j, int cols, int rows)
{
    float Ry = BELL_RY_BASE;                          /* dome height (px) */

    /* Contracted crown: apex descends → clip rows above this |ny| */
    float crown_ny_limit = BELL_NY_CROWN + BELL_CROWN_SPAN * j->crown_f;

    int row_top = px_to_row(j->cy - Ry) - 1;
    int row_bot = px_to_row(j->cy)     + 1;
    if (row_top < 0)      row_top = 0;
    if (row_bot >= rows)  row_bot = rows - 1;

    for (int r = row_top; r <= row_bot; r++) {
        float py = r * (float)CELL_H + (float)CELL_H * 0.5f;
        float ny = (py - j->cy) / Ry;                /* normalized height, -1..0 */
        if (ny > 0.0f || ny < -1.0f) continue;       /* outside the dome         */
        if (fabsf(ny) > crown_ny_limit) continue;    /* above the contracted apex */
        draw_dome_row(win, j, r, ny, cols);          /* emit this row's band     */
    }
}

/* Lateral sway of one tentacle segment: a travelling sine down the chain
 * (phase advances with time, retards with segment depth), gated by wave_scale
 * so the sway dies during the jet and returns as the bell re-blooms (ref [5]). */
static float tentacle_wave_offset(const Jellyfish *j, int k, int seg)
{
    return TENT_WAVE_AMP * j->wave_scale
         * sinf(j->tent_phase[k] - seg * TENT_WAVE_K + (float)k * TENT_PHASE_OFF);
}

/* Taper glyph from root to tip: solid bars near the bell fading to dim dots at
 * the tips.  Returns the char and writes its intensity attribute via *attr. */
static char tentacle_glyph(float depth, attr_t *attr)
{
    if (depth < TENT_DEPTH_NEAR) { *attr = A_BOLD;   return '|'; }
    if (depth < TENT_DEPTH_MID)  { *attr = A_NORMAL; return '|'; }
    if (depth < TENT_DEPTH_FAR)  { *attr = A_DIM;    return ':'; }
    *attr = A_DIM; return '.';
}

/*
 * draw_tentacles() — N_TENTACLES travelling-wave chains hanging from the rim.
 * Each segment = root-x + sway offset, dropped by seg spacing plus an inertia
 * lag (when vy<0 the chain streams downward behind the rising bell), drawn with
 * a depth-tapered glyph.
 */
static void draw_tentacles(WINDOW *win, const Jellyfish *j, int cols, int rows)
{
    float Rx        = BELL_RX_BASE * j->head_f;
    float lag_total = -j->vy * TENT_LAG;   /* inertia: trail opposite to motion */

    for (int k = 0; k < N_TENTACLES; k++) {
        float t_frac = (N_TENTACLES > 1)
                     ? (float)k / (float)(N_TENTACLES - 1) : 0.5f;
        float base_x = j->cx + (t_frac - 0.5f) * 2.0f * Rx;   /* root on the rim */

        for (int seg = 0; seg < TENTACLE_SEGS; seg++) {
            float depth = (float)seg / (float)(TENTACLE_SEGS - 1);  /* 0=root 1=tip */
            float px = base_x + tentacle_wave_offset(j, k, seg);    /* sway        */
            float py = j->cy + seg * TENT_SEG_PX + lag_total * depth;/* drop + lag */

            int c = px_to_col(px);
            int r = px_to_row(py);
            if (c < 0 || c >= cols || r < 0 || r >= rows) continue;

            attr_t at;
            char   ch = tentacle_glyph(depth, &at);
            wattron(win, COLOR_PAIR(j->cp) | at);
            mvwaddch(win, r, c, ch);
            wattroff(win, COLOR_PAIR(j->cp) | at);
        }
    }
}

/* HUD: data readout top-right (fps / count / run-state), key hints bottom-left.
 * Bright + A_BOLD per the project HUD standard so it stays legible over the art. */
static void draw_hud(WINDOW *win, const Scene *s, int cols, int rows, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  jellies:%d  %s ",
             fps, s->n_jellies, s->paused ? "PAUSED " : "running");
    int x = cols - (int)strlen(buf);
    if (x < 0) x = 0;
    wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvwprintw(win, 0, x, "%s", buf);
    wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);

    wattron(win, COLOR_PAIR(CP_HINT) | A_BOLD);
    mvwprintw(win, rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:count ");
    wattroff(win, COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(const Scene *s, WINDOW *win,
                       int cols, int rows, double fps)
{
    for (int i = 0; i < s->n_jellies; i++) {
        draw_tentacles(win, &s->jellies[i], cols, rows);
        draw_bell(win, &s->jellies[i], cols, rows);
    }
    draw_hud(win, s, cols, rows, fps);
}

/* Screen — the ncurses display surface, measured in terminal CELLS (chars).
 *
 * WHY only two fields: ncurses owns the real frame buffer (stdscr) and its
 * internal double-buffering, so this struct caches just the current dimensions
 * — the hot path then never calls getmaxyx().  cols×rows are the bridge
 * between the cell world the renderer writes into and the pixel world
 * (×CELL_W / CELL_H) the simulation lives in.  Owns the init/resize/render
 * lifecycle below; dimensions are refreshed by screen_init and screen_resize. */
typedef struct {
    int cols, rows;          /* terminal width/height in character cells,
                              * re-read from getmaxyx at init and on resize     */
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

static void screen_render(const Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §6  app — combine point (per-tick order) + user events + frame cap     */
/* ===================================================================== */

/* App — process-level glue binding the simulation to its terminal.
 *
 * WHY it exists and is global (g_app): POSIX signal handlers take no user-data
 * argument, so the SIGINT/SIGTERM/SIGWINCH handler can only reach program
 * state through a file-scope object.  The two flags are the ONLY channel
 * between the async handler and the main loop, hence volatile sig_atomic_t —
 * the one type the C standard guarantees is safe to write inside a handler and
 * read in the loop without tearing.  scene/screen are touched by the main
 * thread only, never the handler.                                            */
typedef struct {
    Scene                 scene;       /* the simulated world (§4)             */
    Screen                screen;      /* the display surface (§5)             */
    volatile sig_atomic_t running;     /* cleared by SIGINT/SIGTERM → loop ends */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH → loop re-reads size  */
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

/* User event — mutates Scene, but NOT part of the per-tick combine order. */
static bool app_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':
        sc->paused = !sc->paused;
        break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;
    case '+': case '=':
        if (sc->n_jellies < N_JELLIES_MAX) {
            int i = sc->n_jellies++;
            jelly_spawn(&sc->jellies[i], i,
                        sc->max_px, sc->max_py, /*spread=*/false);
        }
        break;
    case '-':
        if (sc->n_jellies > 1) sc->n_jellies--;
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t prev      = clock_ns();
    int64_t fps_acc   = 0;
    int     fps_count = 0;
    double  fps_disp  = 0.0;

    while (app->running) {

        /* USER EVENT (out of tick): resize is applied before measuring dt */
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->scene.max_px = (float)(app->screen.cols * CELL_W);
            app->scene.max_py = (float)(app->screen.rows * CELL_H);
            app->need_resize  = 0;
            prev = clock_ns();
        }

        /* 1. PERFORMANCE — measure dt, capped to avoid spiral-of-death */
        int64_t now   = clock_ns();
        int64_t dt_ns = now - prev;
        prev = now;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        /* 2. SIMULATION — the sole state advance */
        scene_tick(&app->scene, dt);

        /* 3. PERFORMANCE — fps accumulation */
        fps_count++;
        fps_acc += dt_ns;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_acc / (double)NS_PER_SEC);
            fps_count = 0;
            fps_acc   = 0;
        }

        /* 4. RENDER — read-only */
        screen_render(&app->screen, &app->scene, fps_disp);

        /* 5. PERFORMANCE — sleep to the frame cap */
        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* USER EVENT (out of tick): key handling */
        int ch = getch();
        if (ch != ERR && !app_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
