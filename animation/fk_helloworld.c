/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_helloworld.c — two-link arm driven by joint angles (forward kinematics)
 *
 * DEMO: The textbook hello-world of FORWARD kinematics, the
 *       counterpart to ik_helloworld.c. A two-link arm hangs off
 *       a fixed shoulder. The user controls TWO joint angles
 *       directly with the arrow keys; the elbow and hand
 *       positions are COMPUTED from those angles by composing
 *       two rotations. The hand is no longer commanded — it is
 *       wherever the angles place it.
 *
 *           S ──────────────── E ──────────────── H
 *           @       L1         O       L2         *
 *        shoulder            elbow              hand
 *         (fixed)         (computed)         (computed)
 *
 *       ←/→ rotate θ1 (the shoulder angle).
 *       ↑/↓ rotate θ2 (the elbow angle, relative to the upper arm).
 *       Hold a key and watch the hand sweep through space —
 *       that traced path is exactly what ik_helloworld.c has to
 *       INVERT to recover the angles from a hand position.
 *
 * FK vs IK in one paragraph
 *   FK (this file): user inputs JOINT ANGLES → compute positions.
 *                   Closed-form. One unique answer for every input.
 *                   No reachability check, no two-solution
 *                   ambiguity. Just two rotations composed.
 *   IK (partner)  : user inputs the HAND POSITION → back-solve
 *                   for the angles via law of cosines. TWO valid
 *                   solutions per reachable target (elbow up/down),
 *                   reachability check needed. The hard
 *                   mathematical inverse of FK.
 *   FK is the EASY direction. Every IK solver is essentially
 *   asking "what FK input produced THIS output?" — so studying
 *   FK first makes IK intelligible.
 *
 * Symbol meanings — note how '*' flips role between the two files:
 *
 *               IK (ik_helloworld.c)        FK (this file)
 *     '@'  S    fixed (same)                fixed (same)
 *     'O'  E    computed (IK output)        computed (FK output)
 *     '*'  ─    *target* — user INPUT        *hand* — FK OUTPUT
 *
 * Study alongside:
 *   animation/ik_helloworld.c             — the IK partner: same
 *                                           geometry, opposite
 *                                           input/output flow.
 *   animation/snake_forward_kinematics.c  — FK on a long chain.
 *   animation/snake_inverse_kinematics.c  — FABRIK on a long chain.
 *
 * Section map:
 *   §1  config       — every tunable in one place
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-link / per-marker pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  fk           — Vec2 + compute_fk(S, θ1, θ2) → (E, H)
 *   §6  scene        — Scene state, input, draw
 *   §7  screen       — ncurses init / present / HUD
 *   §8  app          — signals, resize, main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume angle input
 *   r            reset angles to zero (arm horizontal)
 *   ← / →        rotate θ1  (shoulder)
 *   ↑ / ↓        rotate θ2  (elbow, relative to upper arm)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/fk_helloworld.c \
 *       -o fk_helloworld -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Forward Kinematics — given joint angles θ1, θ2
 *                  and link lengths L1, L2, compute joint positions
 *                  by composing two rotations from the fixed
 *                  shoulder S:
 *                      E = S + L1·(cos θ1,        sin θ1)
 *                      H = E + L2·(cos(θ1 + θ2),  sin(θ1 + θ2))
 *                  No equations to solve, no triangle to construct,
 *                  no ambiguity. Hand position is a deterministic
 *                  function of the joint angles.
 *
 * Data-structure : One Vec2 for the fixed shoulder, two scalar
 *                  angles (theta1, theta2) for user input, and two
 *                  cached Vec2 outputs (elbow, hand) in the scene.
 *                  compute_fk(S, θ1, θ2) is a pure function returning
 *                  an ArmPose struct; the main loop assigns its
 *                  result into scene.{elbow, hand} so renderer and
 *                  HUD can read the positions without recomputing.
 *
 * Rendering      : Identical to ik_helloworld.c — two coloured
 *                  lines (S→E cyan, E→H orange) plus three markers
 *                  ('@' shoulder, 'O' elbow, '*' hand). The visual
 *                  is the same; only the data flow direction
 *                  differs. In IK '*' is INPUT (you place the
 *                  hand); in FK '*' is OUTPUT (the hand is wherever
 *                  the angles put it).
 *
 * Performance    : O(1) per frame. Two cosf and two sinf per
 *                  frame; nothing else of consequence.
 *
 * Why FK before IK?
 *   FK is the easy direction. Given angles, finding positions is
 *   one matrix multiplication (or, in 2-D, two trig calls). IK is
 *   the inverse problem: given the desired hand position, what
 *   angles produce it? That inverse is non-trivial — multiple
 *   solutions exist, the problem may be unsolvable, and the math
 *   branches into case analysis. Studying FK first makes IK
 *   intelligible: every IK solver is essentially asking "what FK
 *   input produced this output?".
 *
 * References
 *   Wikipedia, "Forward kinematics" — derivation for serial chains.
 *     https://en.wikipedia.org/wiki/Forward_kinematics
 *   Wikipedia, "Denavit–Hartenberg parameters" — the standard
 *     formal notation for multi-joint robot geometries.
 *     https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
 *   Craig, "Introduction to Robotics: Mechanics and Control"
 *     (3rd ed., 2005) — the canonical robotics textbook treatment
 *     of FK and IK side by side.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Position is what falls out when angles are given. Rotate the
 * shoulder by θ1 and the upper arm points in that direction;
 * walk L1 along it — there's the elbow. Apply a second rotation θ2
 * RELATIVE to the upper arm and walk L2 along the resulting
 * direction — there's the hand. That's it. Two rotations composed
 * in series. There is no solver, no inverse, no ambiguity: FK is
 * the FORWARD direction of the kinematic chain.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Sit at a desk and lay your right arm flat. Slowly rotate your
 * right SHOULDER — the upper arm swings, and your elbow draws an
 * arc of radius L1 around the shoulder. Now hold the shoulder
 * fixed and bend your ELBOW — the forearm pivots, and your hand
 * draws an arc of radius L2 around the elbow. Combine the two:
 * any reachable hand position is some θ1 (shoulder) plus some θ2
 * (elbow). That is all FK is.
 *
 *   shoulder    @          ← S, fixed, never moves
 *                ╲ θ1
 *                 ╲   L1   ← upper arm (constant length)
 *                  ╲
 *           elbow   O      ← E = S + L1·(cos θ1, sin θ1)
 *                   ╲ θ2   (relative to the upper arm direction)
 *                    ╲ L2
 *                     ╲
 *                hand  *   ← H = E + L2·(cos(θ1+θ2), sin(θ1+θ2))
 *
 * ALGORITHM
 * ─────────
 *   E = S + L1 · (cos θ1,        sin θ1)
 *   H = E + L2 · (cos(θ1 + θ2),  sin(θ1 + θ2))
 *
 * Two assignments, four trig calls. Read compute_fk in §5
 * line-by-line and it matches exactly.
 *
 * KEY FORMULAS
 * ────────────
 *   Forward step    : a link of length L hanging from base B at
 *                     angle θ from +X has its tip at
 *                         B + L · (cos θ, sin θ)
 *                     This is THE FK primitive. Stack two of them
 *                     in series — second one using the SUM of
 *                     angles — and you have a 2-link planar arm.
 *
 *   Angle convention: θ1 is absolute (measured from +X, math
 *                     convention so positive θ rotates CCW in
 *                     math space, CW in screen space because y
 *                     points down). θ2 is RELATIVE to the upper
 *                     arm, so the forearm's absolute angle is
 *                     (θ1 + θ2).
 *
 *   Hand reach      : |H − S| ranges over [|L1−L2|, L1+L2] depending
 *                     on θ2 alone. With L1 = L2 the minimum is 0
 *                     (forearm folds back exactly onto the upper
 *                     arm) and the maximum is L1+L2 (arm fully
 *                     extended).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Angle wrap-around. We normalise θ1, θ2 into (−π, π] every
 *     frame so the HUD readout doesn't drift to ±10000°. Wrapping
 *     is purely cosmetic — the trig calls work fine on raw values.
 *
 *   • No reachability check. The hand can land anywhere within the
 *     reach disc; there's no "out of reach" because the user isn't
 *     TARGETING anything — they're rotating joints. This is one
 *     of the major ways FK is simpler than IK.
 *
 *   • No two-solution ambiguity. Each (θ1, θ2) pair maps to a
 *     unique (E, H). Compare with IK, where any reachable target
 *     has TWO valid configurations (elbow up / elbow down).
 *
 *   • Resize. SIGWINCH re-anchors S to the new screen centre.
 *     Angles are unaffected; the new positions just recompute
 *     from the same θ1, θ2.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press 'r'. θ1 = θ2 = 0; arm horizontal pointing right,
 *     hand at distance L1+L2 from the shoulder.
 *
 *   • Hold →. θ1 increases; the WHOLE arm rotates around the
 *     shoulder. The elbow traces a circle of radius L1; the hand
 *     traces a circle of radius L1+L2 (because θ2 stayed at 0).
 *
 *   • Press 'r', then hold ↓. θ2 increases; only the FOREARM
 *     rotates around the elbow, while the upper arm stays put.
 *     The hand traces a circle of radius L2 around the elbow.
 *
 *   • With θ2 = π (180°) exactly, the forearm folds back along
 *     the upper arm. With L1 = L2 the hand sits ON the shoulder;
 *     the HUD shows |H − S| = 0.
 *
 *   • With θ2 = 0 exactly, the forearm extends straight from the
 *     upper arm; the arm is fully extended and |H − S| = L1+L2.
 *
 *   • Try to reproduce IK behaviour by hand: pick a hand position
 *     somewhere, then twiddle θ1, θ2 until '*' lands on it. THAT
 *     manual search is what IK automates.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. This is THE smallest possible FK demo; everything
 *      else in animation/ is some elaboration of compute_fk().
 *   2. §5 fk — the entire algorithm is one function: compute_fk.
 *      Read AFTER tutorials T1-T4. It mirrors the formula in
 *      MENTAL MODEL line for line.
 *   3. §6 scene — input and rendering. Two arrow-key handlers
 *      (one per joint angle), a draw routine that plots two
 *      lines + three markers. Mostly UI plumbing.
 *   4. §1-§4 + §7-§8 — infrastructure (config, clock, colour,
 *      screen, app loop). Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   theta1, theta2     joint angles. theta1 is absolute (from +X);
 *                      theta2 is RELATIVE to the upper arm.
 *   L1, L2             link lengths (in pixels). Constant.
 *   S, E, H            shoulder / elbow / hand positions (Vec2).
 *                      S fixed; E and H computed every frame.
 *   ArmPose            return type bundling (E, H) so compute_fk
 *                      stays a pure function (no out-params).
 *
 * Background you need
 * ───────────────────
 *   - Trigonometric circle: (cos θ, sin θ) is a unit vector at
 *     angle θ from +X.
 *   - Vector addition: walk a length L along a direction d̂ →
 *     base + L · d̂.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Matrices. The standard FK formulation in robotics uses 4×4
 *     transforms; in 2-D with two joints, two trig calls beat
 *     any matrix.
 *   - Denavit-Hartenberg parameters. Worth knowing for serial
 *     robots; overkill for this two-link arm.
 *   - Quaternions. 2-D angles compose by addition; 3-D needs them.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build forward kinematics from first principles.
 *
 *   T1  What is a kinematic chain?
 *   T2  One link — angle to position
 *   T3  Two links — relative angles compose by addition
 *   T4  Why FK is the EASY direction
 *   T5  How to translate this code into IK
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS A KINEMATIC CHAIN?
 * ──────────────────────────────
 * A KINEMATIC CHAIN is a sequence of rigid links connected by
 * joints. Each joint has a small set of "degrees of freedom" —
 * the parameters that pin down its current pose.
 *
 *   In 2-D each joint is a HINGE: one degree of freedom (the
 *   angle around the joint pivot).
 *   In 3-D a joint can be a hinge (1 DOF), a universal joint
 *   (2 DOF), a ball joint (3 DOF), or fancier.
 *
 * "FORWARD" kinematics walks the chain from the FIXED END (root)
 * toward the FREE END (tip), using the joint angles to decide
 * each link's orientation. The output is the WORLD POSITION (and
 * orientation, in 3-D) of every joint and link.
 *
 *      ┌─────────────────────────────────────────────────┐
 *      │  fixed root               free tip              │
 *      │   @  ──────  O  ──────  O  ──────  *            │
 *      │   S  link 1  E  link 2  J3 link 3  H            │
 *      │                                                 │
 *      │  given angles → walk forward → tip position     │
 *      └─────────────────────────────────────────────────┘
 *
 * Our arm is the simplest non-trivial chain: two links, two
 * hinge joints, one fixed root.
 *
 * T2  ONE LINK — ANGLE TO POSITION
 * ────────────────────────────────
 * Start with one link of length L attached to a base B at angle
 * θ measured from +X. The tip of that link is at:
 *
 *     tip = B + L · (cos θ, sin θ)
 *
 * Why? (cos θ, sin θ) is the UNIT VECTOR pointing in direction
 * θ. Multiplying by L scales it to length L; adding to B places
 * the tail at B and the head at the tip.
 *
 * That's the ENTIRE FK primitive. Every other FK in this folder
 * (snake, centipede, spider) is just this primitive run in a loop.
 *
 *      ┌────────────────────────────────────────────┐
 *      │      tip                                   │
 *      │       *                                    │
 *      │      ╱                                     │
 *      │     ╱  L                                   │
 *      │    ╱  ↗                                    │
 *      │   ╱ θ                                      │
 *      │  ●─────────────  +X                        │
 *      │  B                                         │
 *      └────────────────────────────────────────────┘
 *
 * T3  TWO LINKS — RELATIVE ANGLES COMPOSE BY ADDITION
 * ───────────────────────────────────────────────────
 * Now add a second link of length L₂ hinged at the tip of the
 * first. Question: at what ABSOLUTE angle does link 2 point?
 *
 * Answer: the ABSOLUTE angle of link 2 is the absolute angle of
 * link 1 plus the RELATIVE angle θ₂ (measured at the elbow,
 * between link 1 and link 2):
 *
 *     link 1 absolute angle = θ₁
 *     link 2 absolute angle = θ₁ + θ₂
 *
 * That's why "angles compose by addition" — in 2-D, rotation is
 * a scalar that adds.
 *
 * Apply the T2 primitive twice:
 *
 *     elbow E = S + L₁ · (cos θ₁,        sin θ₁)
 *     hand  H = E + L₂ · (cos(θ₁ + θ₂),  sin(θ₁ + θ₂))
 *
 * That's compute_fk in §5 in two lines. Notice θ₂ being RELATIVE
 * is what lets you pose the elbow naturally — "bend the elbow
 * 30°" rather than "make the forearm point at 47°."
 *
 * In 3-D you'd compose two ROTATION MATRICES (or quaternions)
 * instead of adding scalars, but the structure is identical.
 *
 * T4  WHY FK IS THE EASY DIRECTION
 * ────────────────────────────────
 * FK gives you positions FROM angles. IK gives you angles FROM
 * a target position. The two share geometry but differ wildly
 * in difficulty:
 *
 *               FK                   IK
 *   inputs    angles                 target position
 *   outputs   positions              angles
 *   solver    closed form            non-trivial: trig + cases
 *   solutions ONE                    ZERO, ONE, or TWO
 *   reach     not asked              reachability check needed
 *   cost      O(N) trig calls        O(N) per joint × iterations
 *
 * FK ALWAYS SUCCEEDS. Every (θ₁, θ₂) maps to one unique (E, H).
 *
 * IK can FAIL: the target may be outside the reach disc
 * [|L₁−L₂|, L₁+L₂]. Even when reachable, two valid arm poses
 * exist (elbow-up / elbow-down) so the solver must PICK one.
 *
 * Doing FK in your head: rotate, walk, rotate, walk. Trivial.
 * Doing IK in your head: imagine the target, find the elbow
 * position via the law of cosines, then solve for both angles.
 * That mental work is exactly what ik_helloworld.c does for you.
 *
 * T5  HOW TO TRANSLATE THIS CODE INTO IK
 * ──────────────────────────────────────
 * Open ik_helloworld.c side-by-side with this file. The DEMO
 * is identical: same shoulder, same two coloured links, same
 * three markers. The DATA-FLOW is reversed:
 *
 *   FK input pipeline:    arrow keys  →  θ₁, θ₂
 *                         compute_fk  →  E, H
 *                         draw          (E, H)
 *
 *   IK input pipeline:    arrow keys  →  H (target)
 *                         solve_ik    →  θ₁, θ₂  (and back to E)
 *                         draw          (E, H)
 *
 * Reading the IK file with FK already in your head:
 *   - The 2-link IK solver applies the LAW OF COSINES to the
 *     triangle (S, E, H) to find θ₂.
 *   - Then atan2 finds the angle from S to H, and another trig
 *     adjustment splits that into θ₁.
 *   - The "elbow up vs elbow down" choice is a single sign flip.
 *
 * The IK file is bigger because it has to handle reachability
 * (clamp the target to the reach disc), pick a branch (which
 * elbow), and undo the inverse mapping. The math is harder; the
 * geometry is the same triangle.
 *
 * Once you can read both files, every other animation/ file is
 * "FK or IK on a longer chain or a more interesting pose."
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
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

enum {
    TARGET_FPS = 60,

    /* ncurses pair IDs. PAIR_HUD/PAIR_HINT carry the project HUD
     * convention (bright yellow / bright cyan, both A_BOLD). */
    PAIR_HUD          = 1,
    PAIR_HINT         = 2,
    PAIR_LINK_UPPER   = 3,
    PAIR_LINK_FORE    = 4,
    PAIR_JOINT        = 5,
    PAIR_SHOULDER     = 6,
    PAIR_HAND         = 7,
};

/* Equal upper arm and forearm: the hand reaches all the way back
 * to the shoulder when the elbow is fully folded (θ2 = π).
 * 80 px = 10 cells per link → maximum reach 20 cells. */
#define L1_PX  80.0f                  /* upper arm length, pixels */
#define L2_PX  80.0f                  /* forearm  length, pixels */

/* Angle conversions. */
#define DEG_TO_RAD      ((float)M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / (float)M_PI)

/* 2° per arrow keypress. With typical key-repeat ~30 Hz this gives
 * a comfortable ~60°/sec sweep — fast enough to feel responsive,
 * slow enough to land on a target angle by eye. */
#define ANGLE_STEP_RAD  (2.0f * DEG_TO_RAD)

/* Terminal cell dimensions — the aspect-ratio bridge. CELL_W=8
 * sub-pixels per column, CELL_H=16 per row, giving the 2:1 ratio
 * that matches a typical terminal font. All math is in pixel space
 * (isotropic); cell conversion happens only at draw time. */
#define CELL_W   8
#define CELL_H  16

#define NS_PER_SEC 1000000000LL

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
 * One fixed semantic colour per visual role:
 *   upper arm   cyan
 *   forearm     orange
 *   elbow       white
 *   shoulder    lime    (anchor — never moves)
 *   hand        red     (FK output — wherever the angles place it)
 */
static void color_init(void)
{
    if (!has_colors()) return;
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,         226, -1);
        init_pair(PAIR_HINT,         51, -1);
        init_pair(PAIR_LINK_UPPER,   45, -1);
        init_pair(PAIR_LINK_FORE,   208, -1);
        init_pair(PAIR_JOINT,       255, -1);
        init_pair(PAIR_SHOULDER,    118, -1);
        init_pair(PAIR_HAND,        196, -1);
    } else {
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_UPPER,  COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_FORE,   COLOR_RED,     -1);
        init_pair(PAIR_JOINT,       COLOR_WHITE,   -1);
        init_pair(PAIR_SHOULDER,    COLOR_GREEN,   -1);
        init_pair(PAIR_HAND,        COLOR_RED,     -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio bridge                           */
/* ===================================================================== */

/*
 * Positions live in square pixel space. These helpers convert to
 * cell coordinates only at draw time, undoing the 8:16 cell aspect
 * ratio. Doing geometry in pixel space means perpendicular vectors
 * are genuinely perpendicular — a property that would break in
 * cell space.
 */
static inline int   px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline float cells_to_px_w(int cols){ return (float)cols * CELL_W; }
static inline float cells_to_px_h(int rows){ return (float)rows * CELL_H; }

/* ===================================================================== */
/* §5  fk — Vec2 + compute_fk(S, θ1, θ2) → (E, H)                         */
/* ===================================================================== */

typedef struct { float x, y; } Vec2;

static inline Vec2  vec2(float x, float y)    { return (Vec2){x, y}; }
static inline Vec2  vec2_sub(Vec2 a, Vec2 b)  { return (Vec2){a.x-b.x, a.y-b.y}; }
static inline float vec2_len(Vec2 v)          { return sqrtf(v.x*v.x + v.y*v.y); }
static inline float vec2_dist(Vec2 a, Vec2 b) { return vec2_len(vec2_sub(a, b)); }

/* ArmPose — the two outputs of one FK evaluation. */
typedef struct { Vec2 elbow, hand; } ArmPose;

/*
 * compute_fk — forward kinematics for a 2-link planar arm.
 *
 * Pure function: given a fixed shoulder S and two joint angles,
 * return the resulting elbow and hand positions. No branches, no
 * iteration, four trig calls.
 *
 *     theta1   absolute, measured from +X axis.
 *     theta2   RELATIVE to the upper-arm direction, so the
 *              forearm's absolute angle is (theta1 + theta2).
 *
 * The body matches the ALGORITHM block in MENTAL MODEL exactly —
 * E walks L1 from S along θ1; H walks L2 from E along (θ1 + θ2).
 */
static ArmPose compute_fk(Vec2 S, float theta1, float theta2)
{
    Vec2 E = vec2(S.x + L1_PX * cosf(theta1),
                  S.y + L1_PX * sinf(theta1));
    Vec2 H = vec2(E.x + L2_PX * cosf(theta1 + theta2),
                  E.y + L2_PX * sinf(theta1 + theta2));
    return (ArmPose){E, H};
}

/* ===================================================================== */
/* §6  scene — Scene state, input, draw                                   */
/* ===================================================================== */

typedef struct {
    Vec2  shoulder;       /* S — pinned                              */
    float theta1;         /* shoulder angle, radians (abs from +X)   */
    float theta2;         /* elbow angle, radians (rel to upper arm) */
    Vec2  elbow;          /* E — refreshed each frame by compute_fk  */
    Vec2  hand;           /* H — refreshed each frame by compute_fk  */
    bool  paused;
    int   rows, cols;
} Scene;

/*
 * wrap_pi — fold an angle into (−π, π]. Cosmetic only: keeps the
 * HUD readout stable as the user holds an arrow key for a long
 * time. The underlying trig works fine on any real angle.
 */
static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/*
 * scene_init — anchor S at screen centre, set angles to zero
 * (arm horizontal pointing right), run compute_fk once so the
 * cached elbow and hand are valid on frame zero.
 */
static void scene_init(Scene *s, int rows, int cols)
{
    s->rows   = rows;
    s->cols   = cols;
    s->paused = false;
    s->theta1 = 0.0f;
    s->theta2 = 0.0f;

    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);

    ArmPose pose = compute_fk(s->shoulder, s->theta1, s->theta2);
    s->elbow = pose.elbow;
    s->hand  = pose.hand;
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);

    ArmPose pose = compute_fk(s->shoulder, s->theta1, s->theta2);
    s->elbow = pose.elbow;
    s->hand  = pose.hand;
}

/*
 * scene_input — translate one keypress.
 *   ←/→     rotate shoulder θ1
 *   ↑/↓     rotate elbow    θ2
 *   space   toggle pause
 *   r       reset angles to zero
 * No key for moving the elbow or the hand directly — both are
 * FK output. Only angles are user input.
 */
static void scene_input(Scene *s, int ch)
{
    const float k = ANGLE_STEP_RAD;

    if (!s->paused) {
        switch (ch) {
            case KEY_LEFT:  s->theta1 -= k; break;
            case KEY_RIGHT: s->theta1 += k; break;
            case KEY_UP:    s->theta2 -= k; break;
            case KEY_DOWN:  s->theta2 += k; break;
            default: break;
        }
        s->theta1 = wrap_pi(s->theta1);
        s->theta2 = wrap_pi(s->theta2);
    }

    switch (ch) {
        case ' ': s->paused = !s->paused; break;
        case 'r': scene_init(s, s->rows, s->cols); break;
        default: break;
    }
}

/* ---- rendering ------------------------------------------------------- */

static inline bool in_screen(int row, int col, int rows, int cols)
{
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

/*
 * draw_line_px — parametric line from a to b in pixel space, sampled
 * at half-cell spacing along the dominant axis so no cell on the
 * path is skipped at any angle.
 */
static void draw_line_px(Vec2 a, Vec2 b, chtype glyph, int rows, int cols)
{
    float dx = b.x - a.x, dy = b.y - a.y;
    float steps_f = fmaxf(fabsf(dx) / (CELL_W * 0.5f),
                          fabsf(dy) / (CELL_H * 0.5f));
    int   steps   = (int)steps_f + 1;
    for (int i = 0; i <= steps; i++) {
        float t   = (float)i / (float)steps;
        float px  = a.x + t * dx;
        float py  = a.y + t * dy;
        int   col = px_to_cell_x(px);
        int   row = px_to_cell_y(py);
        if (in_screen(row, col, rows, cols))
            mvaddch(row, col, glyph);
    }
}

static void draw_line(Vec2 a, Vec2 b, int color_pair, int rows, int cols)
{
    chtype glyph = (chtype)((unsigned char)'#') | COLOR_PAIR(color_pair);
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    draw_line_px(a, b, glyph, rows, cols);
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

static void draw_point(Vec2 p, char glyph, int color_pair, int rows, int cols)
{
    int row = px_to_cell_y(p.y);
    int col = px_to_cell_x(p.x);
    if (!in_screen(row, col, rows, cols)) return;
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    mvaddch(row, col, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

/*
 * scene_draw — the five draw calls. Identical to ik_helloworld.c
 * except that '*' is now the COMPUTED hand instead of the
 * user-controlled target. Lines first, markers on top; '@' last so
 * the pinned shoulder always wins the cell contest at S.
 */
static void scene_draw(const Scene *s)
{
    Vec2 S = s->shoulder, E = s->elbow, H = s->hand;

    draw_line (S, E, PAIR_LINK_UPPER, s->rows, s->cols);
    draw_line (E, H, PAIR_LINK_FORE,  s->rows, s->cols);

    draw_point(H, '*', PAIR_HAND,     s->rows, s->cols);
    draw_point(E, 'O', PAIR_JOINT,    s->rows, s->cols);
    draw_point(S, '@', PAIR_SHOULDER, s->rows, s->cols);
}

/* ===================================================================== */
/* §7  screen — ncurses init / present / HUD                              */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    color_init();
}

static void screen_cleanup(void)
{
    if (!isendwin()) endwin();
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * screen_hud — required HUD: status row 0, hint row last. Row 1
 * shows the live values of θ1 and θ2 in degrees plus the resulting
 * hand reach |H − S|. Holding a single arrow key produces a
 * smooth sweep through both readouts — that's FK doing its job.
 */
static void screen_hud(const Scene *s, float fps)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    char  buf[112];
    float deg1  = s->theta1 * RAD_TO_DEG;
    float deg2  = s->theta2 * RAD_TO_DEG;
    float reach = vec2_dist(s->shoulder, s->hand);

    snprintf(buf, sizeof buf, " %5.1f fps  forward kinematics  %s ",
             fps, s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    snprintf(buf, sizeof buf,
             " L1:%.0f  L2:%.0f  theta1:%+7.1f°  theta2:%+7.1f°  |H-S|:%5.1fpx ",
             L1_PX, L2_PX, deg1, deg2, reach);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  L/R:rotate theta1  U/D:rotate theta2 ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §8  app — signals, resize, main loop                                   */
/* ===================================================================== */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running     = 0;
}

static void install_signals(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGINT,   &sa, NULL);
    sigaction(SIGTERM,  &sa, NULL);
    sigaction(SIGWINCH, &sa, NULL);
}

int main(void)
{
    install_signals();
    atexit(screen_cleanup);
    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene scene;
    scene_init(&scene, rows, cols);

    int64_t       t_prev       = clock_ns();
    int64_t       fps_accum_ns = 0;
    int           fps_frames   = 0;
    float         fps          = 0.0f;
    const int64_t TICK_NS      = NS_PER_SEC / TARGET_FPS;

    while (g_running) {
        int64_t t_now = clock_ns();
        fps_accum_ns += t_now - t_prev;
        t_prev = t_now;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC) {
            fps          = (float)fps_frames * 1e9f / (float)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        if (g_need_resize) {
            g_need_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&scene, rows, cols);
        }

        /* (1) input — angles, NOT a target */
        for (int ch; (ch = getch()) != ERR; ) {
            if (ch == 'q' || ch == 27 /*ESC*/) { g_running = 0; break; }
            scene_input(&scene, ch);
        }

        /* (2) compute_fk(S, θ1, θ2) → (E, H), cached in scene */
        ArmPose pose = compute_fk(scene.shoulder, scene.theta1, scene.theta2);
        scene.elbow  = pose.elbow;
        scene.hand   = pose.hand;

        /* (3) clear / draw / present (same five draws as IK) */
        erase();
        scene_draw(&scene);
        screen_hud(&scene, fps);
        screen_present();

        int64_t elapsed = clock_ns() - t_now;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    return 0;
}
