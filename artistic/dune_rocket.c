/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* dune_rocket.c — Harkonnen siege: homing missiles pound the Arrakis desert
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/dune_rocket.c -o dune_rocket
 * -lncurses -lm
 *
 * Keys: q quit | p pause | r reset | +/- launch rate | Space salvo
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Homing missile simulation with proportional navigation.
 *                  Each rocket steers toward a target using a turn rate
 *                  (TURN_RATE) that bends the velocity vector each tick.
 *                  Lateral wobble (WOBBLE_AMP × sin(WOBBLE_FREQ × t)) adds
 *                  visual realism to the flight path.
 *
 * Physics        : Rocket speed increases from ROCKET_SPEED0 toward
 *                  ROCKET_SPEEDMAX each frame (acceleration phase).
 *                  Trajectory is integrated with explicit Euler:
 *                    pos += vel × dt;  vel direction bent by TURN_RATE.
 *
 * Rendering      : Rocket orientation mapped to one of 8 ASCII direction
 *                  glyphs (> \ v / < ^) by heading octant; trail stored as a
 *                  ring buffer (TRAIL_LEN) of past positions drawn at
 *                  decreasing brightness to show motion blur.
 *                  Explosion sparks are simple ballistic particles with
 *                  gravity applied each tick.
 *
 * References     :
 *   Guidance / homing (TURN_RATE, proportional navigation):
 *   [1] Zarchan, "Tactical and Strategic Missile Guidance" (AIAA, 6th ed.)
 *       — the standard text on proportional navigation & pursuit laws.
 *   [2] Shneydor, "Missile Guidance and Pursuit" (Horwood, 1998)
 *       — concise treatment of pure-pursuit / lead-pursuit geometry.
 *   [3] Reynolds, "Steering Behaviors for Autonomous Characters" (GDC 1999)
 *       — seek/pursue as a turn-rate-limited velocity bend (this sim's model).
 *
 *   Integration & particles (Euler step, ballistic sparks):
 *   [4] Witkin & Baraff, "Physically Based Modeling" (SIGGRAPH course notes)
 *       — explicit Euler, particle systems, why dt must stay small.
 *   [5] Reeves, "Particle Systems — A Technique for Modeling a Class of
 *       Fuzzy Objects" (ACM TOG 1983) — origin of fire/explosion particles.
 *   [6] Millington, "Game Physics Engine Development" (2nd ed.)
 *       — practical projectile/gravity integration for real-time loops.
 *
 *   Rendering (8-way direction glyphs, trail brightness ramp):
 *   [7] Bourke, "Character representation of grey scale images" (1997)
 *       — luminance→ASCII ramp, the basis for trail fade.
 *   [8] ncurses HOWTO / "Programming with ncurses" (Padala) — color pairs,
 *       erase/doupdate double-buffering used in the screen layer.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into LAYERS, not entities. State is aggregated on one Scene
 * (§3) that reads like a table of contents; functions take the NARROWEST type
 * they need (const DomainType* to read, DomainType* to mutate) so "everything
 * hangs off Scene" never re-couples the layers. Only the per-tick orchestrators
 * (scene_reset / scene_tick / scene_salvo) take Scene*.
 *
 *   Layer        Section            Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2 timing          nothing (reads OS clock, sleeps)
 *   DATA         §3 state           — type declarations + the Scene instance —
 *   LOGIC        §4 logic           nothing (pure: reads, returns values)
 *   SIMULATION   §5 simulation      scene.rockets, scene.explosions,
 *                                    scene.terrain (scorch), scene.carrier
 *                                    (port_flash), scene.launch_timer
 *   INIT/RESET   §6 init            ALL sim state (every §3 object)
 *   RENDER       §7 render          the screen only — never sim state
 *   EVENTS       §8 events          g_running, g_need_resize, + new launches
 *
 * No EFFECTS layer: cosmetic-only state DOES exist (rocket trail ring buffer,
 * carrier port_flash, terrain scorch) but each is written inline by the
 * SIMULATION function that owns it — rocket_step records the trail (via
 * trail_push), scene_tick decays the port flash, terrain_scorch/terrain_step
 * manage scorch. The
 * trail/spark brightness FADE is derived at render time from age fraction, not
 * stored. A standalone EFFECTS layer would mean splitting those functions.
 *
 * No DELAYS layer: pause is a single flag (scene.paused) tested once in main
 * before the tick; the per-entity timers (launch cadence scene.launch_timer,
 * port_flash, explosion life, scorch_decay) are owned by and advanced inside
 * the SIMULATION state they belong to. There are no global holds.
 *
 * PER-TICK COMBINE — scene_tick is the ONLY place sim state advances, in order:
 *   1. decay carrier port-flash timers  (cosmetic state)
 *   2. auto-launch if timer elapsed
 *   3. rocket_step each rocket; on RS_DONE → explosion_spawn, deactivate
 *   4. explosion_step each explosion
 *   5. terrain_step (scorch decay)
 *
 * User events (reset r, salvo space, rate +/-, resize, quit) DO mutate state
 * but are NOT part of the tick — they run in main's input/resize handling,
 * outside and before the gated `if (!paused) scene_tick(...)`.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── §1 config ───────────────────────────────────────────────────────────── */
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

#define TICK_NS 33333333L /* 30 Hz */
#define MAX_ROCKETS 28
#define MAX_EXPLOSIONS 16
#define N_SPARKS 22
#define TRAIL_LEN 30
#define MAX_PORTS 8
#define SHIP_TOP_ROW 1 /* topmost ship row */
#define SHIP_ROWS 5    /* ship height in rows */

#define ROCKET_SPEED0 15.0f   /* launch speed (cells/sec) */
#define ROCKET_SPEEDMAX 44.0f /* terminal speed */
#define TURN_RATE 3.0f        /* steering sharpness */
#define WOBBLE_AMP 0.28f      /* lateral wobble (radians equiv.) */
#define WOBBLE_FREQ 4.0f      /* oscillations per second */
#define LAUNCH_RATE0 1.1f     /* seconds between auto-launches */
#define LAUNCH_RATE_STEP                                                       \
  0.2f /* +/- adjusts the auto-launch interval by this (sec) */
#define LAUNCH_RATE_MIN                                                        \
  0.2f /* fastest auto-launch interval the user can set (sec) */
#define LAUNCH_RATE_MAX                                                        \
  5.0f              /* slowest auto-launch interval the user can set (sec) */
#define DT_MAX 0.1f /* clamp on frame dt — avoids spiral-of-death (sec) */

/* rocket flight & guidance */
#define ROCKET_ACCEL 20.0f   /* speed ramp toward terminal (cells/sec^2) */
#define ROCKET_ARRIVE 1.5f   /* dist to target counted as a hit (cells) */
#define TURN_BLEND_MAX 0.99f /* cap on per-tick heading blend fraction */
#define WOBBLE_FADE_DIST                                                       \
  10.0f /* wobble fades to zero within this range (cells) */

/* explosion, sparks & shockwave ring */
#define SPARK_GRAVITY 7.0f /* downward accel on debris sparks (cells/sec^2) */
#define EXPLO_LIFE 2.0f    /* explosion lifetime (sec) */
#define RING_SPEED 14.0f   /* shockwave expansion rate (cells/sec) */
#define RING_MAX_R 22.0f   /* radius past which the ring stops drawing (cells) \
                            */
#define RING_ASPECT 0.45f  /* vertical squash so the ring reads as a circle */

/* terrain scorch & launch ports */
#define SCORCH_RADIUS 5.5f /* blast scorch radius on the dunes (cells) */
#define SCORCH_MAX 4 /* scorch intensity at blast centre (0..SCORCH_MAX) */
#define SCORCH_COOL_S 3.0f /* seconds between scorch cool-down steps */
#define PORT_FLASH_S 0.15f /* muzzle-flash duration after a port fires (sec)   \
                            */

/* launch & flight jitter (every jitter helper preserves the original RNG
 * stream) */
#define VEL_EPS 0.01f /* speeds below this read as "no heading" (cells/sec) */
#define LAUNCH_LEAN_SPAN                                                       \
  201 /* discrete steps of the sideways launch shove (symmetric) */
#define LAUNCH_LEAN_SCALE 0.018f /* cells/sec per lean step (peak ~ +/-1.8) */
#define WOBBLE_FREQ_JITTER                                                     \
  0.35f /* per-rocket wobble frequency varies +/-this around WOBBLE_FREQ */
#define WOBBLE_AMP_JITTER                                                      \
  0.45f /* per-rocket wobble amplitude varies +/-this around WOBBLE_AMP */
#define LAUNCH_JITTER                                                          \
  0.6f /* auto-launch interval varies +/-this around launch_rate */
#define FIRST_LAUNCH_DELAY                                                     \
  0.3f /* seconds before the first launch after a reset */

/* spark emission — one ember, in spark_emit/explosion_spawn */
#define SPARK_SPEED_MIN 4.0f  /* slowest ember (cells/sec) */
#define SPARK_SPEED_MAX 13.0f /* fastest ember (cells/sec) */
#define SPARK_LIFE_MIN 0.35f  /* shortest ember lifetime (sec) */
#define SPARK_LIFE_MAX 1.05f  /* longest ember lifetime (sec) */
#define SPARK_UP_CLAMP                                                         \
  0.25f /* cap on sin(angle) so embers burst up/out, not down */
#define SPARK_UP_KICK                                                          \
  3.0f /* extra upward velocity on every ember (cells/sec) */
#define SPARK_SCATTER_SPAN                                                     \
  5 /* discrete spawn-scatter positions about the blast centre */
#define SPARK_SCATTER_SCALE 0.3f /* cells per scatter step (peak ~ +/-0.6) */
#define SPARK_ANGLE_SPAN                                                       \
  100 /* discrete angle-jitter steps on the even spark fan */
#define SPARK_ANGLE_SCALE                                                      \
  0.01f /* radians per angle-jitter step (peak ~ +/-0.5) */

/* targeting & launch cadence */
#define TARGET_MARGIN                                                          \
  2 /* keep impact targets this many columns off each edge */
#define TARGET_ABOVE_GROUND                                                    \
  0.3f /* aim this far above the sand surface (cells) */
#define AUTO_LAUNCH_RESERVE                                                    \
  2 /* auto-launch only while this many pool slots stay free */

/* dune height field — surface row measured up from the bottom HUD */
#define GROUND_BASE_OFF                                                        \
  7 /* nominal surface sits this many rows above the bottom */
#define GROUND_TOP_OFF                                                         \
  12 /* highest the surface may rise (rows above bottom) — clears ship */
#define GROUND_BOT_OFF                                                         \
  4 /* lowest the surface may fall (rows above bottom) — clears HUD */

/* carrier hull geometry */
#define HULL_W_MIN 30 /* narrowest hull (columns) */
#define HULL_W_MAX 82 /* widest hull (columns) */
#define PORT_INSET                                                             \
  4 /* launch ports inset this many columns from each hull end */

/* render — brightness-ramp breakpoints (fraction of life/age, freshest = 1) */
#define TRAIL_HOT                                                              \
  0.70f /* trail above this age-fraction -> bright orange + bold */
#define TRAIL_MID                                                              \
  0.38f /* ... above this -> mid orange; below -> dim dark-red */
#define EXP_RING_HOT 0.55f /* ring above this life-fraction -> white '*' */
#define EXP_RING_MID                                                           \
  0.25f /* ... above this -> yellow '+'; below -> orange '.' */
#define EXP_CORE_FRAC                                                          \
  0.65f                /* draw the bright core sprite while fresher than this */
#define SPARK_HOT 0.6f /* ember above this life-fraction -> white '*' */
#define SPARK_MID 0.3f /* ... above this -> yellow '+'; below -> orange '.' */

/* render — shockwave ring sampling & sand texture */
#define RING_PTS_PER_R 6.28f /* ~2*pi: ring sample count ~ circumference */
#define RING_MIN_PTS 12 /* floor on ring samples so small rings still read */
#define SAND_SPECKLE_PERIOD                                                    \
  11 /* 2-in-this-many sub-surface cells get a speckle dot */
#define SLOPE_AXIS_RATIO                                                       \
  1.73f /* ~tan(60deg): trail slope counts as axis-aligned past this */

#define MAX_COLS 512

/* ── §2 PERFORMANCE — timing primitives ──────────────────────────────────── *
 * Monotonic clock + sleep. Mutates nothing. The frame cap and dt clamp that
 * use these live in main's loop (variable-dt + frame cap, NOT a fixed-timestep
 * accumulator — unlike the framework default).
 * ────────────────────────────────────────────────────────────────────────── */
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

/* ── §3 DATA — domain types & the Scene aggregate ─────────────────────────── *
 * No behaviour here: these types are mutated by SIMULATION (§5) / INIT (§6) and
 * read by LOGIC (§4) / RENDER (§7). Color-pair ids are render-side identifiers.
 * ────────────────────────────────────────────────────────────────────────── */

/* color-pair identifiers (RENDER) */
enum {
  CP_SHIP_HULL = 1,
  CP_SHIP_PORT,
  CP_RKT_HEAD,
  CP_RKT_HOT,
  CP_RKT_MID,
  CP_RKT_DIM,
  CP_EXP_CORE,
  CP_EXP_MID,
  CP_EXP_DIM,
  CP_GROUND,
  CP_SAND,
  CP_SCORCH,
  CP_HUD,
  CP_HINT
};

/* Terrain — the Arrakis dune surface: a 1-D HEIGHT FIELD plus a decaying scorch
 * heatmap. WHY a height field, not a 2-D grid: the desert is drawn one glyph
 * per screen column, so the surface only needs a single ground row per column —
 * a full grid would waste memory, and the slope glyph (/ \ _) is just the
 * discrete derivative of neighbouring heights. INTENT of the profile: the dunes
 * are a sum of four sine octaves with FALLING amplitude (2.0/1.3/0.7/0.3) and
 * RISING frequency (2.3/5.7/12.1/27.3) — a crude 1/f fractal-Brownian look, far
 * cheaper than Perlin noise and seamless because every term is periodic over
 * the screen width (see terrain_init). Scorch is a tiny integer heatmap that
 * impacts stamp and one shared clock slowly cools — "blast scars heal over
 * time."
 *
 *   ground[col]  : HEIGHT FIELD. Screen row of the sand surface at column col.
 *                  Valid for col in [0,cols); clamped to
 *                  [rows-GROUND_TOP_OFF, rows-GROUND_BOT_OFF] so dunes never
 *                  reach the ship or the bottom HUD. Larger value = lower on
 *                  screen (row 0 is the top).
 *   scorch[col]  : burn intensity, integer 0..SCORCH_MAX. 0 = clean sand,
 *                  SCORCH_MAX = fresh blast centre. Written by terrain_scorch
 *                  with a linear radial falloff (s = SCORCH_MAX - dist/radius *
 *                  SCORCH_MAX); read by draw_terrain to override the surface
 *                  glyph (>= SCORCH_MAX-1 -> '*', else '.').
 *   scorch_decay : seconds accumulated toward the next cool-down. ONE shared
 *                  clock for all columns (not a timer per scar): when it passes
 *                  SCORCH_COOL_S every scorched column drops one level and it
 *                  resets — O(cols) once per SCORCH_COOL_S instead of MAX_COLS
 *                  live timers. */
typedef struct {
  int ground[MAX_COLS];
  int scorch[MAX_COLS];
  float scorch_decay;
} Terrain;

/* Carrier — the Harkonnen ship spanning the top of the screen, the source of
 * every rocket. WHY these fields: the hull is purely decorative geometry, so it
 * stores only its column span and the spawn row; the meaningful part is the row
 * of launch ports, held as PARALLEL ARRAYS — port_x is fixed geometry,
 * port_flash is transient cosmetic state (see the step-5 note on why these stay
 * fields rather than a LaunchPort type). CONTEXT: the hull width auto-scales to
 * the terminal as clamp(cols*3/4, HULL_W_MIN, HULL_W_MAX), centred, so the ship
 * fills a sensible fraction of any screen (see carrier_init).
 *
 *   left, right  : hull's leftmost / rightmost screen column (border included).
 *   launch_row   : row rockets spawn from = SHIP_TOP_ROW + SHIP_ROWS, i.e. just
 *                  under the hull so missiles emerge from the belly.
 *   n_ports      : live launch-port count (= MAX_PORTS); also the valid length
 *                  of both arrays below.
 *   port_x[i]    : screen column of port i, spread evenly across the hull
 *                  interior (inset PORT_INSET from each end). FIXED at init.
 *   port_flash[i]: muzzle-flash countdown in SECONDS — set to PORT_FLASH_S
 *                  when port i fires, decayed each tick. >0 -> red '*', else
 *                  a gray '|'. The only mutable cosmetic field per port. */
typedef struct {
  int left, right;
  int launch_row;
  int n_ports;
  int port_x[MAX_PORTS];
  float port_flash[MAX_PORTS];
} Carrier;

/* RState — rocket lifecycle. RS_FLYING: live, steering and integrating each
 * tick. RS_DONE: terminal — it reached its target, struck the dunes, or left
 * the field. scene_tick turns a DONE rocket into an explosion and frees its
 * pool slot the SAME tick, so a DONE rocket is never rendered (it is a
 * one-frame "please reap me" signal, not a drawable state). */
typedef enum { RS_FLYING = 0, RS_DONE } RState;

/* Rocket — one homing missile flying PURE-PURSUIT guidance toward a fixed
 * ground target. INTENT / how it steers: each tick the unit heading is blended
 * a fraction (TURN_RATE*dt, capped at TURN_BLEND_MAX) toward the unit vector
 * pointing at the target, then re-normalised — a turn-rate-limited "seek"
 * (Reynolds [3]), which is the simplest pursuit case of proportional navigation
 * (Zarchan [1], Shneydor [2]). Motion is explicit Euler — pos += vel*dt, vel =
 * heading*speed, speed ramping toward ROCKET_SPEEDMAX (Witkin & Baraff [4],
 * Millington [6]). A lateral SINE wobble perpendicular to the heading makes the
 * flight look powered rather than rail-guided; it fades out near the target so
 * the final dive is straight. Recent positions are kept in a ring buffer to
 * draw a fading motion-blur trail (brightness ramp in the spirit of Bourke
 * [7]).
 *
 *   x, y        : position in CELLS (fractional; rounded only at draw time).
 *   vx, vy      : velocity in CELLS/SEC = heading * speed, rebuilt every tick.
 *   tx, ty      : the FIXED ground impact point chosen at launch; guidance aims
 *                 here until dist < ROCKET_ARRIVE, then state -> RS_DONE.
 *   wobble_ph   : current wobble phase in radians; advances by wobble_freq*dt.
 *   wobble_freq : per-rocket angular rate (rad/s), WOBBLE_FREQ scaled by
 *                 (1 +/- WOBBLE_FREQ_JITTER) so rockets never lock-step.
 *   wobble_amp  : per-rocket lateral amplitude, WOBBLE_AMP scaled by
 *                 (1 +/- WOBBLE_AMP_JITTER), then by
 *                 min(1, dist/WOBBLE_FADE_DIST) so it vanishes on the dive.
 *   speed       : scalar speed (cells/sec), kept apart from |vx,vy| because
 *                 the heading is re-normalised each tick; speed alone
 *                 integrates ROCKET_ACCEL up to ROCKET_SPEEDMAX.
 *   state       : RState (see above).
 *   tbx[],tby[] : trail RING BUFFER holding the last <=TRAIL_LEN positions.
 *   t_head      : index of the NEXT slot to overwrite (wraps mod TRAIL_LEN).
 *   t_len       : filled-slot count, saturating at TRAIL_LEN. A ring buffer
 *                 gives O(1) append, no per-frame allocation or shifting.
 *   active      : 1 if this slot of the fixed rocket pool is in use
 *                 (object-pool pattern, no malloc on the hot path). */

typedef struct {
  float x, y;      /* position (cells) */
  float vx, vy;    /* velocity (cells/sec) */
  float tx, ty;    /* ground target */
  float wobble_ph; /* wobble phase (radians) */
  float wobble_freq;
  float wobble_amp;
  float speed; /* current speed magnitude */
  RState state;
  /* trail ring buffer */
  float tbx[TRAIL_LEN];
  float tby[TRAIL_LEN];
  int t_head; /* next-write index */
  int t_len;  /* filled entries 0..TRAIL_LEN */
  int active;
} Rocket;

/* Spark — one ballistic ember thrown by an explosion. A textbook particle
 * (Reeves [5]) integrated with explicit Euler under CONSTANT downward gravity
 * (SPARK_GRAVITY, applied in explosion_step), with no inter-particle forces —
 * cheap and visually sufficient. Emitted radially from the blast with the
 * downward velocity component clamped at spawn so debris flies up and out, not
 * into the ground.
 *
 *   x, y      : position in cells;  vx, vy : velocity in cells/sec.
 *   life      : seconds of life remaining; decremented by dt, dead at <= 0.
 *   max_life  : life at spawn. The ratio life/max_life (1 -> 0) is the ONLY age
 *               signal stored, and drives the spark's colour/glyph fade.
 *   active    : 1 while alive — slot flag in the explosion's fixed spark pool.
 */
typedef struct {
  float x, y, vx, vy;
  float life, max_life;
  int active;
} Spark;

/* Explosion — an impact burst built from three superimposed effects: a brief
 * bright CORE, an expanding shockwave RING, and a cloud of ballistic SPARKS
 * (particle system, Reeves [5]). WHY one struct: all three share a birthplace
 * and lifetime, so they age off a single life clock. The ring is drawn as an
 * aspect-corrected ellipse (y radius scaled by RING_ASPECT) so it reads as a
 * circle despite character cells being roughly twice as tall as they are wide.
 *
 *   x, y      : blast centre in cells.
 *   life      : seconds remaining (starts at EXPLO_LIFE); the burst dies at <=
 * 0. max_life  : life at spawn. frac = life/max_life (1 = fresh -> 0 = dying)
 *               selects the ring/core colour pair and glyph (* -> + -> .).
 *   ring_r    : shockwave radius in cells; grows at RING_SPEED, stops drawing
 *               past RING_MAX_R. Kept SEPARATE from life so the ring can
 *               expand and vanish while the core/sparks still fade.
 *   sparks    : embedded fixed pool of N_SPARKS embers, all emitted at spawn.
 *   active    : 1 while this slot of the explosion pool is in use. */
typedef struct {
  float x, y;
  float life, max_life;
  float ring_r; /* expanding shockwave radius */
  Spark sparks[N_SPARKS];
  int active;
} Explosion;

/* Scene — the entire simulation in one aggregate so each function can take the
 * NARROWEST sub-object it needs (see ARCHITECTURE), never the whole Scene
 * unless it orchestrates a tick. Reads as a table of contents: WHAT exists, HOW
 * the user tunes it, WHERE in time we are. CONTEXT: rockets[] and explosions[]
 * are FIXED-SIZE OBJECT POOLS — no allocation after start-up; an `active` flag
 * marks each live slot and rocket_launch / explosion_spawn linearly scan for a
 * free one (a full pool simply drops the launch).
 *
 *   WHAT  - the domain objects:
 *     terrain      : the dune height field + scorch heatmap.
 *     carrier      : the launching ship and its ports.
 *     rockets[]    : pool of up to MAX_ROCKETS in-flight missiles.
 *     explosions[] : pool of up to MAX_EXPLOSIONS active bursts.
 *   HOW   - the one user-tunable knob:
 *     launch_rate  : seconds between auto-launches; +/- adjust within
 *                    [LAUNCH_RATE_MIN, LAUNCH_RATE_MAX]. PERSISTS across
 *                    reset/resize (set once at init) while scene_reset
 *                    zeroes everything else.
 *   WHERE - run position in time:
 *     launch_timer : seconds counting DOWN to the next auto-launch; rearmed
 *                    to launch_rate * (1 +/- LAUNCH_JITTER) so fire timing
 *                    never feels mechanical.
 *     paused       : 1 freezes scene_tick (rendering keeps running). */
typedef struct {
  Terrain terrain;
  Carrier carrier;
  Rocket rockets[MAX_ROCKETS];
  Explosion explosions[MAX_EXPLOSIONS];
  float launch_rate;
  float launch_timer;
  int paused;
} Scene;

/* the single simulation instance (BSS); launch_rate persists across resets */
static Scene g_scene = {.launch_rate = LAUNCH_RATE0};

/* ── §4 LOGIC — pure decisions (no mutation, no I/O) ──────────────────────── *
 * These read inputs and return a value. Deleting or reordering RENDER/EFFECTS
 * cannot change their results, because they hold no state and touch no screen.
 * ────────────────────────────────────────────────────────────────────────── */

/* Random jitter idioms. These are the ONE impurity in §4 — they advance the
 * shared PRNG — but they hold no sim state and touch no screen, so they read as
 * value-returning decisions. Both reproduce the file's original 0.01-step
 * granularity exactly, so swapping them in leaves the RNG sequence untouched.
 */

/* uniform random in [lo, hi) in 0.01 steps (== lo + rand()%steps * 0.01) */
static float frand_range(float lo, float hi) {
  int steps = (int)((hi - lo) * 100.f + 0.5f);
  return lo + (float)(rand() % steps) * 0.01f;
}

/* symmetric random offset about 0 (== (rand()%n - n/2) * scale) */
static float rand_centered(int n, float scale) {
  int off = rand() % n - n / 2; /* integer span centred on 0 */
  return (float)off * scale;
}

/* scale a vector to unit length in place (leaves a near-zero vector alone) */
static void normalize(float *x, float *y) {
  float len = sqrtf(*x * *x + *y * *y);
  if (len > 0.0001f) {
    *x /= len;
    *y /= len;
  }
}

/* ring-buffer index of the t-th trail point, oldest (t=0) → newest */
static int trail_index(const Rocket *r, int t) {
  return (r->t_head - r->t_len + t + TRAIL_LEN * 2) % TRAIL_LEN;
}

/* has the rocket left the playfield or buried itself in the dunes? */
static int rocket_off_field(const Rocket *r, const Terrain *terrain, int cols,
                            int rows) {
  int ic = (int)(r->x + 0.5f);
  if (ic < 0 || ic >= cols)
    return 1; /* off the sides */
  if (r->y >= (float)terrain->ground[ic])
    return 1; /* into the sand */
  if (r->y >= (float)(rows - 2))
    return 1; /* below the field */
  return 0;
}

/* velocity → slope character for the rocket nose: classify the heading into one
   of 8 compass octants (45deg wide, boundaries at the odd multiples of 22.5deg)
 */
static char dir_char_rocket(float vx, float vy) {
  float a = atan2f(vy, vx) * (180.f / (float)M_PI);
  if (a > -22.5f && a <= 22.5f)
    return '>';
  if (a > 22.5f && a <= 67.5f)
    return '\\';
  if (a > 67.5f && a <= 112.5f)
    return 'v';
  if (a > 112.5f && a <= 157.5f)
    return '/';
  if (a > 157.5f || a <= -157.5f)
    return '<';
  if (a > -157.5f && a <= -112.5f)
    return '\\';
  if (a > -112.5f && a <= -67.5f)
    return '^';
  return '/'; /* -67.5 .. -22.5 → up-right */
}

/* velocity → slope character for trail segments */
static char dir_char_trail(float vx, float vy) {
  float ax = fabsf(vx), ay = fabsf(vy);
  if (ay > ax * SLOPE_AXIS_RATIO)
    return '|';
  if (ax > ay * SLOPE_AXIS_RATIO)
    return '-';
  return (vx * vy > 0.f) ? '\\' : '/';
}

/* count rockets currently in flight (pure read) */
static int count_active(const Rocket *rockets) {
  int n = 0;
  for (int i = 0; i < MAX_ROCKETS; i++)
    if (rockets[i].active)
      n++;
  return n;
}

/* ── §5 SIMULATION — advances state ───────────────────────────────────────── *
 * Every function here mutates sim state (see ARCHITECTURE table); each takes a
 * non-const pointer to exactly the object it changes. scene_tick is the single
 * per-tick combine — nothing else in the program advances state.
 * ────────────────────────────────────────────────────────────────────────── */

static void terrain_scorch(Terrain *terrain, float cx, float radius) {
  int c0 = (int)(cx - radius);
  int c1 = (int)(cx + radius + 1.f);
  for (int c = c0; c <= c1; c++) {
    if (c < 0 || c >= MAX_COLS)
      continue;
    float d = fabsf((float)c - cx) / radius;
    if (d >= 1.f)
      continue;
    int s = SCORCH_MAX - (int)(d * SCORCH_MAX); /* linear radial falloff */
    if (s > terrain->scorch[c])
      terrain->scorch[c] = s;
  }
}

static void terrain_step(Terrain *terrain, float dt, int cols) {
  terrain->scorch_decay += dt;
  if (terrain->scorch_decay > SCORCH_COOL_S) {
    terrain->scorch_decay = 0.f;
    for (int c = 0; c < cols; c++)
      if (terrain->scorch[c] > 0)
        terrain->scorch[c]--;
  }
}

static void rocket_launch(Rocket *rockets, Carrier *carrier, int port, float tx,
                          float ty) {
  for (int i = 0; i < MAX_ROCKETS; i++) {
    if (rockets[i].active)
      continue;
    Rocket *r = &rockets[i];
    memset(r, 0, sizeof *r);
    r->active = 1;
    r->x = (float)carrier->port_x[port];
    r->y = (float)carrier->launch_row + 0.5f;
    /* slight random sideways lean at launch */
    float lean = rand_centered(LAUNCH_LEAN_SPAN, LAUNCH_LEAN_SCALE);
    r->vx = lean;
    r->vy = ROCKET_SPEED0;
    r->tx = tx;
    r->ty = ty;
    r->wobble_ph = frand_range(0.f, 6.28f); /* random start phase [0,2pi) */
    r->wobble_freq = WOBBLE_FREQ * frand_range(1.f - WOBBLE_FREQ_JITTER,
                                               1.f + WOBBLE_FREQ_JITTER);
    r->wobble_amp = WOBBLE_AMP * frand_range(1.f - WOBBLE_AMP_JITTER,
                                             1.f + WOBBLE_AMP_JITTER);
    r->speed = ROCKET_SPEED0;
    r->state = RS_FLYING;
    carrier->port_flash[port] = PORT_FLASH_S;
    return;
  }
}

/* append the current position to the trail ring buffer (O(1), wraps) */
static void trail_push(Rocket *r) {
  r->tbx[r->t_head] = r->x;
  r->tby[r->t_head] = r->y;
  r->t_head = (r->t_head + 1) % TRAIL_LEN;
  if (r->t_len < TRAIL_LEN)
    r->t_len++;
}

static void rocket_step(Rocket *r, const Terrain *terrain, float dt, int cols,
                        int rows) {
  if (!r->active || r->state == RS_DONE)
    return;
  trail_push(r);

  /* range to target; close enough counts as a hit */
  float dx = r->tx - r->x;
  float dy = r->ty - r->y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < ROCKET_ARRIVE) {
    r->state = RS_DONE;
    return;
  }

  /* desired heading (unit vector pointing at the target) */
  float ddx = dx / dist, ddy = dy / dist;

  /* current heading (unit vector along velocity) */
  float spd = sqrtf(r->vx * r->vx + r->vy * r->vy);
  float cdx = (spd > VEL_EPS) ? r->vx / spd : 0.f;
  float cdy = (spd > VEL_EPS) ? r->vy / spd : 1.f;

  /* steer: blend current heading a turn-rate-limited fraction toward desired */
  float tr = fminf(TURN_RATE * dt, TURN_BLEND_MAX);
  cdx += (ddx - cdx) * tr;
  cdy += (ddy - cdy) * tr;
  normalize(&cdx, &cdy);

  /* lateral wobble, fading to zero as the rocket nears its target */
  float wscale = fminf(1.f, dist / WOBBLE_FADE_DIST);
  float wobble = r->wobble_amp * wscale * sinf(r->wobble_ph);
  r->wobble_ph += r->wobble_freq * dt;

  /* nudge heading along the perpendicular (-cdy, cdx) by wobble, re-unit so
     speed stays constant and wobble is purely a direction perturbation */
  float wdx = cdx - cdy * wobble;
  float wdy = cdy + cdx * wobble;
  normalize(&wdx, &wdy);

  /* accelerate toward terminal speed, then integrate position (explicit Euler)
   */
  r->speed = fminf(r->speed + ROCKET_ACCEL * dt, ROCKET_SPEEDMAX);
  r->vx = wdx * r->speed;
  r->vy = wdy * r->speed;
  r->x += r->vx * dt;
  r->y += r->vy * dt;

  if (rocket_off_field(r, terrain, cols, rows)) {
    r->state = RS_DONE;
    return;
  }
}

/* emit one ballistic ember from (ox,oy) along base angle ang, bursting up/out
 */
static void spark_emit(Spark *sp, float ox, float oy, float ang) {
  float spd = frand_range(SPARK_SPEED_MIN, SPARK_SPEED_MAX);
  float up = sinf(ang);
  if (up > SPARK_UP_CLAMP)
    up = SPARK_UP_CLAMP; /* clamp so debris flies up, not down */
  sp->vx = cosf(ang) * spd;
  sp->vy = up * spd - SPARK_UP_KICK;
  sp->x = ox + rand_centered(SPARK_SCATTER_SPAN, SPARK_SCATTER_SCALE);
  sp->y = oy;
  sp->life = frand_range(SPARK_LIFE_MIN, SPARK_LIFE_MAX);
  sp->max_life = sp->life;
  sp->active = 1;
}

static void explosion_spawn(Explosion *explosions, Terrain *terrain, float x,
                            float y) {
  terrain_scorch(terrain, x, SCORCH_RADIUS);
  for (int i = 0; i < MAX_EXPLOSIONS; i++) {
    if (explosions[i].active)
      continue;
    Explosion *e = &explosions[i];
    memset(e, 0, sizeof *e);
    e->active = 1;
    e->x = x;
    e->y = y;
    e->life = EXPLO_LIFE;
    e->max_life = EXPLO_LIFE;
    e->ring_r = 0.f;
    /* fan sparks evenly around the circle, each angle randomly perturbed */
    for (int s = 0; s < N_SPARKS; s++) {
      float ang = (float)s / (float)N_SPARKS * 2.f * (float)M_PI +
                  rand_centered(SPARK_ANGLE_SPAN, SPARK_ANGLE_SCALE);
      spark_emit(&e->sparks[s], x, y, ang);
    }
    return;
  }
}

static void explosion_step(Explosion *e, float dt) {
  if (!e->active)
    return;
  e->life -= dt;
  if (e->life <= 0.f) {
    e->active = 0;
    return;
  }
  e->ring_r += RING_SPEED * dt;
  for (int s = 0; s < N_SPARKS; s++) {
    Spark *sp = &e->sparks[s];
    if (!sp->active)
      continue;
    sp->life -= dt;
    if (sp->life <= 0.f) {
      sp->active = 0;
      continue;
    }
    sp->vy += SPARK_GRAVITY * dt; /* gravity pulls sparks down */
    sp->x += sp->vx * dt;
    sp->y += sp->vy * dt;
  }
}

/* pick a random ground column and the impact point just above its surface
   (TARGET_ABOVE_GROUND sits the target on top of the sand, not buried in it) */
static void random_ground_target(const Terrain *terrain, int cols, float *tx,
                                 float *ty) {
  int tc = TARGET_MARGIN + rand() % (cols - 2 * TARGET_MARGIN);
  *tx = (float)tc;
  *ty = (float)terrain->ground[tc] - TARGET_ABOVE_GROUND;
}

/* fire one rocket from a random port at a random target, then re-arm the
   cadence timer with jitter so launches never feel mechanical */
static void scene_auto_launch(Scene *s, int cols) {
  int port = rand() % s->carrier.n_ports;
  float tx, ty;
  random_ground_target(&s->terrain, cols, &tx, &ty);
  rocket_launch(s->rockets, &s->carrier, port, tx, ty);
  s->launch_timer =
      s->launch_rate * frand_range(1.f - LAUNCH_JITTER, 1.f + LAUNCH_JITTER);
}

/* the per-tick combine: the ONLY place sim state advances (see ARCHITECTURE) */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  /* 1. cool the muzzle-flash timers */
  for (int p = 0; p < s->carrier.n_ports; p++)
    if (s->carrier.port_flash[p] > 0.f)
      s->carrier.port_flash[p] -= dt;

  /* 2. auto-launch once the cadence timer elapses and the pool has room */
  s->launch_timer -= dt;
  if (s->launch_timer <= 0.f &&
      count_active(s->rockets) < MAX_ROCKETS - AUTO_LAUNCH_RESERVE)
    scene_auto_launch(s, cols);

  /* 3. step rockets; a finished rocket becomes an explosion and frees its slot
   */
  for (int i = 0; i < MAX_ROCKETS; i++) {
    Rocket *r = &s->rockets[i];
    if (!r->active)
      continue;
    rocket_step(r, &s->terrain, dt, cols, rows);
    if (r->state == RS_DONE) {
      explosion_spawn(s->explosions, &s->terrain, r->x, r->y);
      r->active = 0;
    }
  }

  /* 4. step explosions */
  for (int i = 0; i < MAX_EXPLOSIONS; i++)
    explosion_step(&s->explosions[i], dt);

  /* 5. cool the terrain scorch */
  terrain_step(&s->terrain, dt, cols);
}

/* ── §6 INIT/RESET — builds sim state (setup, NOT part of the tick) ──────────
 * * Mutates terrain/carrier/rocket/explosion state wholesale. Called at
 * startup, on reset (r), and on resize — all outside scene_tick.
 * ────────────────────────────────────────────────────────────────────────── */

static void terrain_init(Terrain *terrain, int cols, int rows) {
  /* dune profile = sum of four sine octaves: FALLING amplitude + RISING
     frequency → a crude 1/f (fractal-Brownian) ridge, seamless over the width
   */
  static const float DUNE_FREQ[4] = {2.3f, 5.7f, 12.1f, 27.3f};
  static const float DUNE_AMP[4] = {2.0f, 1.3f, 0.7f, 0.3f};
  static const float DUNE_PHASE[4] = {0.f, 0.73f, 1.47f, 0.23f};
  for (int c = 0; c < cols; c++) {
    float x = (float)c / (float)(cols > 1 ? cols - 1 : 1);
    float h = 0.f;
    for (int k = 0; k < 4; k++)
      h += sinf(x * (float)M_PI * DUNE_FREQ[k] + DUNE_PHASE[k]) * DUNE_AMP[k];
    int gr = (rows - GROUND_BASE_OFF) + (int)(h + 0.5f);
    if (gr < rows - GROUND_TOP_OFF)
      gr = rows - GROUND_TOP_OFF;
    if (gr > rows - GROUND_BOT_OFF)
      gr = rows - GROUND_BOT_OFF;
    terrain->ground[c] = gr;
  }
  memset(terrain->scorch, 0, sizeof terrain->scorch);
  terrain->scorch_decay = 0.f;
}

static void carrier_init(Carrier *carrier, int cols) {
  int w = (cols * 3) / 4; /* hull spans 3/4 of the screen width */
  if (w > HULL_W_MAX)
    w = HULL_W_MAX;
  if (w < HULL_W_MIN)
    w = HULL_W_MIN;
  int cx = cols / 2;
  carrier->left = cx - w / 2;
  carrier->right = carrier->left + w;
  carrier->launch_row = SHIP_TOP_ROW + SHIP_ROWS;

  carrier->n_ports = MAX_PORTS;
  int span = w - 2 * PORT_INSET; /* port-bearing width inside the inset */
  int div = carrier->n_ports > 1 ? carrier->n_ports - 1 : 1;
  for (int i = 0; i < carrier->n_ports; i++) {
    carrier->port_x[i] = carrier->left + PORT_INSET + span * i / div;
    carrier->port_flash[i] = 0.f;
  }
}

static void scene_reset(Scene *s, int cols, int rows) {
  memset(s->rockets, 0, sizeof s->rockets);
  memset(s->explosions, 0, sizeof s->explosions);
  terrain_init(&s->terrain, cols, rows);
  carrier_init(&s->carrier, cols);
  s->launch_timer = FIRST_LAUNCH_DELAY;
}

/* ── §7 RENDER — state → screen (reads only, never mutates sim state) ────────
 * * color_init sets up pairs once; the draw_* functions translate the current
 * state into characters and take const sub-types. Trail and spark brightness
 * FADE is computed here from age fraction — never stored (see ARCHITECTURE).
 * ────────────────────────────────────────────────────────────────────────── */

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(CP_SHIP_HULL, 250, 236); /* light gray / dark gray */
    init_pair(CP_SHIP_PORT, 196, 236); /* bright red / dark gray */
    init_pair(CP_RKT_HEAD, 231, -1);   /* bright white */
    init_pair(CP_RKT_HOT, 214, -1);    /* orange */
    init_pair(CP_RKT_MID, 202, -1);    /* red-orange */
    init_pair(CP_RKT_DIM, 88, -1);     /* dark red */
    init_pair(CP_EXP_CORE, 231, -1);   /* white */
    init_pair(CP_EXP_MID, 226, -1);    /* yellow */
    init_pair(CP_EXP_DIM, 208, -1);    /* orange */
    init_pair(CP_GROUND, 136, -1);     /* sandy gold */
    init_pair(CP_SAND, 94, -1);        /* dark sand */
    init_pair(CP_SCORCH, 52, -1);      /* very dark red */
    init_pair(CP_HUD, 226, -1);        /* bright yellow — top data bar */
    init_pair(CP_HINT, 51, -1);        /* bright cyan   — bottom action bar */
  } else {
    init_pair(CP_SHIP_HULL, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_SHIP_PORT, COLOR_RED, COLOR_BLACK);
    init_pair(CP_RKT_HEAD, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_RKT_HOT, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_RKT_MID, COLOR_RED, COLOR_BLACK);
    init_pair(CP_RKT_DIM, COLOR_RED, COLOR_BLACK);
    init_pair(CP_EXP_CORE, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_EXP_MID, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_EXP_DIM, COLOR_RED, COLOR_BLACK);
    init_pair(CP_GROUND, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_SAND, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_SCORCH, COLOR_RED, COLOR_BLACK);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void draw_terrain(const Terrain *terrain, int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    int gr = terrain->ground[c];
    if (gr < 0 || gr >= rows)
      continue;

    /* surface character from neighbour slope */
    int prev_gr = (c > 0) ? terrain->ground[c - 1] : gr;
    int next_gr = (c < cols - 1) ? terrain->ground[c + 1] : gr;
    int slope = next_gr - prev_gr;
    char sch = (slope < -1) ? '/' : (slope > 1) ? '\\' : '_';

    if (terrain->scorch[c] > 0) {
      /* scorch overrides surface */
      attron(COLOR_PAIR(CP_SCORCH) | A_BOLD);
      mvaddch(gr, c, terrain->scorch[c] >= SCORCH_MAX - 1 ? '*' : '.');
      attroff(COLOR_PAIR(CP_SCORCH) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_GROUND) | A_BOLD);
      mvaddch(gr, c, (chtype)(unsigned char)sch);
      attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);
    }

    /* sand fill below surface line */
    attron(COLOR_PAIR(CP_SAND));
    for (int r = gr + 1; r < rows - 1; r++) {
      char fc = ((c + r * 3) % SAND_SPECKLE_PERIOD < 2) ? '.' : ' ';
      mvaddch(r, c, (chtype)(unsigned char)fc);
    }
    attroff(COLOR_PAIR(CP_SAND));
  }
}

static void draw_rockets(const Rocket *rockets) {
  for (int i = 0; i < MAX_ROCKETS; i++) {
    const Rocket *r = &rockets[i];
    if (!r->active || r->state == RS_DONE)
      continue;

    /* trail: iterate oldest → newest */
    for (int t = 0; t < r->t_len; t++) {
      int idx = trail_index(r, t);
      int tr = (int)(r->tby[idx] + 0.5f);
      int tc = (int)(r->tbx[idx] + 0.5f);
      if (tr < 0 || tc < 0)
        continue;

      /* direction from this point to the next (gives correct slope char) */
      float dvx, dvy;
      if (t < r->t_len - 1) {
        int nxt = trail_index(r, t + 1);
        dvx = r->tbx[nxt] - r->tbx[idx];
        dvy = r->tby[nxt] - r->tby[idx];
      } else {
        dvx = r->vx;
        dvy = r->vy;
      }
      char tch = dir_char_trail(dvx, dvy);

      /* age fraction 0=oldest 1=newest — drives brightness */
      float af = (r->t_len > 1) ? (float)t / (float)(r->t_len - 1) : 1.f;
      int pair;
      attr_t attr = 0;
      if (af > TRAIL_HOT) {
        pair = CP_RKT_HOT;
        attr = A_BOLD;
      } else if (af > TRAIL_MID) {
        pair = CP_RKT_MID;
      } else {
        pair = CP_RKT_DIM;
        attr = A_DIM;
      }

      attron(COLOR_PAIR(pair) | attr);
      mvaddch(tr, tc, (chtype)(unsigned char)tch);
      attroff(COLOR_PAIR(pair) | attr);
    }

    /* rocket head */
    int hr = (int)(r->y + 0.5f);
    int hc = (int)(r->x + 0.5f);
    if (hr >= 0 && hc >= 0) {
      attron(COLOR_PAIR(CP_RKT_HEAD) | A_BOLD);
      mvaddch(hr, hc, (chtype)(unsigned char)dir_char_rocket(r->vx, r->vy));
      attroff(COLOR_PAIR(CP_RKT_HEAD) | A_BOLD);
    }
  }
}

static void draw_explosions(const Explosion *explosions, int cols, int rows) {
  for (int i = 0; i < MAX_EXPLOSIONS; i++) {
    const Explosion *e = &explosions[i];
    if (!e->active)
      continue;
    float frac = e->life / e->max_life; /* 1=fresh 0=dying */

    /* expanding shockwave ring (aspect-corrected ellipse) */
    if (e->ring_r < RING_MAX_R) {
      int n_pts = (int)(e->ring_r * RING_PTS_PER_R) + RING_MIN_PTS;
      int pair = (frac > EXP_RING_HOT)   ? CP_EXP_CORE
                 : (frac > EXP_RING_MID) ? CP_EXP_MID
                                         : CP_EXP_DIM;
      char rch = (frac > EXP_RING_HOT)   ? '*'
                 : (frac > EXP_RING_MID) ? '+'
                                         : '.';
      for (int p = 0; p < n_pts; p++) {
        float a = (float)p / (float)n_pts * 2.f * (float)M_PI;
        int rc = (int)(e->x + cosf(a) * e->ring_r + 0.5f);
        /* squash y so the ring looks circular despite tall cells */
        int rr = (int)(e->y + sinf(a) * e->ring_r * RING_ASPECT + 0.5f);
        if (rr < 0 || rr >= rows || rc < 0 || rc >= cols)
          continue;
        attron(COLOR_PAIR(pair));
        mvaddch(rr, rc, (chtype)(unsigned char)rch);
        attroff(COLOR_PAIR(pair));
      }
    }

    /* bright core flash on fresh explosions */
    if (frac > EXP_CORE_FRAC) {
      int er = (int)(e->y + 0.5f);
      int ec = (int)(e->x + 0.5f);
      attron(COLOR_PAIR(CP_EXP_CORE) | A_BOLD);
      if (er >= 0 && er < rows && ec >= 0 && ec < cols)
        mvaddch(er, ec, '#');
      if (er - 1 >= 0) {
        mvaddch(er - 1, ec, '*');
        if (ec - 1 >= 0)
          mvaddch(er - 1, ec - 1, '+');
        if (ec + 1 < cols)
          mvaddch(er - 1, ec + 1, '+');
      }
      if (ec - 1 >= 0 && er < rows)
        mvaddch(er, ec - 1, '*');
      if (ec + 1 < cols && er < rows)
        mvaddch(er, ec + 1, '*');
      attroff(COLOR_PAIR(CP_EXP_CORE) | A_BOLD);
    }

    /* spark particles */
    for (int s = 0; s < N_SPARKS; s++) {
      const Spark *sp = &e->sparks[s];
      if (!sp->active)
        continue;
      int sr = (int)(sp->y + 0.5f);
      int sco = (int)(sp->x + 0.5f);
      if (sr < 0 || sr >= rows || sco < 0 || sco >= cols)
        continue;
      float sf = sp->life / sp->max_life;
      int pair = (sf > SPARK_HOT)   ? CP_EXP_CORE
                 : (sf > SPARK_MID) ? CP_EXP_MID
                                    : CP_EXP_DIM;
      char sch = (sf > SPARK_HOT) ? '*' : (sf > SPARK_MID) ? '+' : '.';
      attron(COLOR_PAIR(pair));
      mvaddch(sr, sco, (chtype)(unsigned char)sch);
      attroff(COLOR_PAIR(pair));
    }
  }
}

static void draw_carrier(const Carrier *carrier) {
  int L = carrier->left, R = carrier->right;
  int row = SHIP_TOP_ROW;

  /* title row */
  const char *title = "HARKONNEN  CARRIER";
  int tx = (L + R) / 2 - (int)strlen(title) / 2;
  attron(COLOR_PAIR(CP_SHIP_HULL) | A_BOLD);
  mvprintw(row, tx, "%s", title);
  row++;

  /* top border */
  mvaddch(row, L, ACS_ULCORNER);
  for (int c = L + 1; c < R; c++)
    mvaddch(row, c, ACS_HLINE);
  mvaddch(row, R, ACS_URCORNER);
  row++;

  /* hull body — hatched armor texture */
  attroff(A_BOLD);
  mvaddch(row, L, ACS_VLINE);
  for (int c = L + 1; c < R; c++) {
    char bc = (((c - L) * 3 + row) % 7 < 2) ? '#' : '=';
    mvaddch(row, c, (chtype)(unsigned char)bc);
  }
  mvaddch(row, R, ACS_VLINE);
  row++;

  /* bottom border: launch port notches with ACS_TTEE */
  attron(A_BOLD);
  mvaddch(row, L, ACS_LLCORNER);
  for (int c = L + 1; c < R; c++) {
    int is_port = 0;
    for (int p = 0; p < carrier->n_ports; p++)
      if (c == carrier->port_x[p]) {
        is_port = 1;
        break;
      }
    mvaddch(row, c, is_port ? ACS_TTEE : ACS_HLINE);
  }
  mvaddch(row, R, ACS_LRCORNER);
  row++;

  /* port shafts: flash red on launch, otherwise gray */
  attroff(A_BOLD);
  for (int p = 0; p < carrier->n_ports; p++) {
    if (carrier->port_flash[p] > 0.f) {
      attron(COLOR_PAIR(CP_SHIP_PORT) | A_BOLD);
      mvaddch(row, carrier->port_x[p], '*');
      attroff(COLOR_PAIR(CP_SHIP_PORT) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_SHIP_HULL));
      mvaddch(row, carrier->port_x[p], ACS_VLINE);
      attroff(COLOR_PAIR(CP_SHIP_HULL));
    }
  }

  attroff(COLOR_PAIR(CP_SHIP_HULL) | A_BOLD);
}

/*
 * draw_hud — two fixed bars over the scene (the ship occupies rows 1+, so row 0
 * is free for the data bar):
 *   top    row 0      DATA    — title (left) + launch rate / rocket count / run
 *                              state (right-aligned).
 *   bottom row rows-1 ACTIONS — the key legend.
 * Both bars are filled first, then text is clipped with "%.*s" and the left
 * title is kept clear of the right block, so a narrow terminal can neither
 * overflow nor wrap.
 */
static void draw_hud(float rate, int n_active, int paused, int rows, int cols) {
  if (cols < 1 || rows < 1)
    return;

  /* ── Top row 0: DATA. ── */
  char left[24], right[64];
  snprintf(left, sizeof left, " DUNE ROCKET ");
  snprintf(right, sizeof right, " rate:%.1fs   rockets:%d/%d   %s ", rate,
           n_active, MAX_ROCKETS, paused ? "PAUSED" : "running");
  int rx = cols - (int)strlen(right); /* right-aligned stats column */
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  for (int c = 0; c < cols; c++)
    mvaddch(0, c, ' ');
  if (rx >= 0) {
    mvprintw(0, 0, "%.*s", rx, left); /* title clipped clear of stats */
    mvprintw(0, rx, "%s", right);
  } else {
    mvprintw(0, 0, "%.*s", cols, right); /* too narrow: data only */
  }
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── Bottom row: ACTIONS — every interactive key. ── */
  int brow = rows - 1;
  if (brow > 0) {
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    for (int c = 0; c < cols; c++)
      mvaddch(brow, c, ' ');
    mvprintw(brow, 0, "%.*s", cols,
             " q:quit  p:pause  r:reset  +/-:rate  spc:salvo ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
  }
}

/* ── §8 EVENTS — signals & user actions (mutate state, NOT part of the tick) ─
 * * Signal handlers set volatile flags read by main. scene_salvo is a
 * key-driven launch command that coordinates carrier + rockets, so it takes
 * Scene*. These run in main's input/resize handling, outside scene_tick.
 * ────────────────────────────────────────────────────────────────────────── */
static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_need_resize = 0;
static void on_sigint(int s) {
  (void)s;
  g_running = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_need_resize = 1;
}

/* fire all ports at once — one rocket per port toward random targets */
static void scene_salvo(Scene *s, int cols) {
  for (int p = 0;
       p < s->carrier.n_ports && count_active(s->rockets) < MAX_ROCKETS - 1;
       p++) {
    float tx, ty;
    random_ground_target(&s->terrain, cols, &tx, &ty);
    rocket_launch(s->rockets, &s->carrier, p, tx, ty);
  }
}

/* ── §9 main — orchestrates EVENTS → PERFORMANCE → SIMULATION → RENDER ───────
 * * The one place the layers are combined per frame, in explicit named order.
 * User events (keys, resize) are handled first and are NOT part of the tick;
 * scene_tick runs only when not paused.
 * ────────────────────────────────────────────────────────────────────────── */
int main(void) {
  srand((unsigned)time(NULL));
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);

  initscr();
  noecho();
  cbreak();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  color_init();

  int rows = LINES, cols = COLS;
  scene_reset(&g_scene, cols, rows);

  long long prev = clock_ns();

  while (g_running) {
    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 'Q':
      case 27:
        g_running = 0;
        break;
      case 'p':
      case 'P':
        g_scene.paused = !g_scene.paused;
        break;
      case 'r':
      case 'R':
        scene_reset(&g_scene, cols, rows);
        break;
      case ' ':
        scene_salvo(&g_scene, cols);
        break;
      case '+':
      case '=':
        g_scene.launch_rate =
            fmaxf(LAUNCH_RATE_MIN, g_scene.launch_rate - LAUNCH_RATE_STEP);
        break;
      case '-':
        g_scene.launch_rate =
            fminf(LAUNCH_RATE_MAX, g_scene.launch_rate + LAUNCH_RATE_STEP);
        break;
      case KEY_RESIZE:
        g_need_resize = 1;
        break;
      }
    }

    if (g_need_resize) {
      g_need_resize = 0;
      endwin();
      refresh();
      rows = LINES;
      cols = COLS;
      scene_reset(&g_scene, cols, rows);
    }

    long long now = clock_ns();
    float dt = (float)(now - prev) * 1e-9f;
    if (dt > DT_MAX)
      dt = DT_MAX;
    prev = now;

    if (!g_scene.paused)
      scene_tick(&g_scene, dt, cols, rows);

    erase();
    draw_terrain(&g_scene.terrain, cols, rows);
    draw_explosions(g_scene.explosions, cols, rows);
    draw_rockets(g_scene.rockets);
    draw_carrier(&g_scene.carrier);
    draw_hud(g_scene.launch_rate, count_active(g_scene.rockets), g_scene.paused,
             rows, cols);
    wnoutrefresh(stdscr);
    doupdate();

    clock_sleep_ns(TICK_NS - (clock_ns() - now));
  }

  endwin();
  return 0;
}
