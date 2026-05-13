# Authoring — Phase 1 File Templates and Code Structure

Reference doc for Phase-1 authoring rules. CLAUDE.md keeps the one-line summary of each rule; this file holds the full templates and examples. Read this when starting a new file.

Every new file is a teaching artifact. A reader must understand the physics, the algorithm, and the framework decisions without leaving the file.

---

# File Header (mandatory)

```c
/*
 * <filename>.c — <one-line description of what it visually does>
 *
 * DEMO: <2-3 sentences. What does the user see? What does it demonstrate?>
 *
 * Study alongside: <the most relevant other file in the project>
 *
 * Section map:
 *   §1 config   — all tunable constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses color pair setup
 *   §4 helpers  — small math helpers — coords (pixel↔cell), vec3 /
 *                 Mat4 for 3-D, Spherical for fractals, etc.  Pick
 *                 whatever the renderer needs.  Omit entirely for
 *                 cell-space sims that don't need any helper math.
 *   §5 <entity> — simulation state + tick logic (rename to the
 *                 actual entity: §5 fluid, §5 boids, §5 torus, etc.)
 *   §6 scene    — entity pool, scene_tick, scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   space pause   r reset   ...
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra <file>.c -o <name> -lncurses -lm
 */
```

---

# CONCEPTS Block (mandatory)

After the file header, before §1. Answers "what is this and where do I read more?"

```c
/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : <Name the algorithm. What problem does it solve?
 *                   One paragraph. No code.>
 *
 * Data-structure : <What structure drives the simulation? Why this one?>
 *
 * Rendering      : <How does the physics map to terminal characters?
 *                   Mention alpha interpolation if used.>
 *
 * Performance    : <What makes it fast enough for real-time? Fixed-step,
 *                   spatial hash, precompute, etc.>
 *
 * References     : <2 minimum, 5 maximum — Wikipedia, papers, books.
 *                   A learner reading this file should know where to go
 *                   next to understand the math more deeply.>
 *
 * ─────────────────────────────────────────────────────────────────────── */
```

References mandatory: **2-5 per file**. Examples: Wikipedia article, original paper (Reynolds 1987 boids, Stam 1999 fluids), textbook chapter (Millington §12), web resource (Inigo Quilez SDFs, Red Blob Games).

---

# MENTAL MODEL Block (mandatory)

After CONCEPTS, before §1. Answers "how do I think in this algorithm?" Six fixed sub-headings, in order:

```c
/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * <One paragraph. The single sentence a learner should walk away with —
 *  the "aha" that makes the rest fall into place. No code.>
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * <Analogy or metaphor that anchors the algorithm in everyday intuition.
 *  Graph paper, cellular automaton, conveyor belt, billiard balls — pick
 *  the closest physical or visual object the learner already understands.>
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 * <Numbered steps the algorithm follows each frame or each iteration.
 *  Plain English, not pseudocode. A learner reading only this section
 *  should be able to reimplement the algorithm from scratch.>
 *
 * KEY FORMULAS
 * ────────────
 * <Every non-trivial equation used in the file, in math-like form with a
 *  one-line gloss. Forward AND inverse mappings if both exist.>
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 * <Bulleted list of subtle things that bite. Boundary conditions, off-
 *  by-one traps, resize handling, what happens at min/max parameter
 *  values, contradictions/contradictory states, etc.>
 *
 * HOW TO VERIFY
 * ─────────────
 * <Concrete sanity checks: counts a learner can do by hand, expected
 *  values at sentinel inputs, what should happen when a parameter is
 *  doubled. The "if I'm not sure it's working, here's how I'd tell."
 *  section.>
 *
 * ─────────────────────────────────────────────────────────────────────── */
```

Canonical reference: `grids/rect_grids/01_uniform_rect.c`. CONCEPTS and MENTAL MODEL are mandatory and distinct — do not collapse.

---

# Named Constants — No Magic Numbers

Every literal carrying meaning belongs in `§1 config` with a context-describing name and units.

```c
/* BAD  */  if (age > 120) die();           vel *= 0.98f;
/* GOOD */  if (age > PARTICLE_LIFETIME_TICKS) die();
            vel *= DRAG_COEFFICIENT;   /* air resistance per tick */

/* Group related constants with a context comment */
#define SEPARATION_RADIUS   40.0f   /* pixels */
#define SEPARATION_FORCE   180.0f   /* pixels/sec² */
```

**Not magic numbers** (don't extract): `0`, `1`, `−1`, `2`, `0.5f` (halving), `2.0f * M_PI` inline, loop bounds from struct fields, index arithmetic (`i+1`), array dimensions implied by name (`LUMI_GLYPHS[8]`).

---

# Function Comments — WHY, not WHAT

Every non-trivial function: (1) what it does in one line, (2) **why** the approach was chosen, (3) cross-reference to equivalent function in another file if any.

```c
/*
 * boid_separate() — push boid away from neighbours within SEPARATION_RADIUS.
 *
 * Uses inverse-distance weighting: closer neighbour = stronger push.
 * Linear (1/d) not quadratic (1/d²) — quadratic causes violent jitter
 * when boids momentarily overlap.
 *
 * Equivalent to ball_collide() in bounce_ball.c for "push apart", but
 * here the force is continuous, not an impulse.
 */
```

For §4 coords, §7 ncurses double-buffer, §8 main loop, fixed-step accumulator: copy explanatory comments from `framework.c` verbatim — these are the canonical explanations.

---

# §5 Entity — Name for the Concept

Struct + functions named after the simulation concept, not generics: `Boid`/`boid_tick()` not `Entity`/`entity_update()`. Fields commented with units.

```c
typedef struct {
    float px, py;   /* position — pixels                        */
    float vx, vy;   /* velocity — pixels / second               */
    float age;      /* seconds alive; dies at PARTICLE_LIFETIME */
    int   color;    /* ncurses color pair index (1–N_COLORS)    */
} Particle;
```

---

# Learner Checkpoints

At each section boundary: `/* ── end §5 — to understand rendering, read §6 scene_draw() ── */`. For non-obvious physics formulas, add the derivation or a named reference inline.

---

# Learner-Friendly Code Structure

How to organize functions, control flow, data, naming so a reader can pick up the file cold.

**Test:** read top-to-bottom. If you ever stop and ask "wait, where is this defined?" — the code failed, regardless of comment thoroughness.

## One concept per function

If a function name needs `_and_` to describe it, split it.

```c
/* BAD  */ void update_and_draw(Boid *b, WINDOW *w);
/* GOOD */ void boid_tick(Boid *b, float dt);
           void boid_draw(const Boid *b, WINDOW *w);
```

## Separate pure from mutating

Pure helpers take `const` and return values; mutating functions take a non-const pointer. The signature alone shows whether state can change.

```c
static int  body_seg_pair (int i);                       /* pure   */
static Vec2 trail_sample  (const Centipede *c, float s); /* pure   */
static void trail_push    (Centipede *c, Vec2 pos);      /* mutate */
```

## Mirror the math in the code

If KEY FORMULAS lists steps 1–4, the function body has four lines, in order, one per step.

```c
/* BAD — four math steps fused into one cryptic expression */
foot.x = hip.x + UPPER_LEN*cosf(body_dir + LEG_SPLAY +
       SWING_AMP*sinf(wave_time*GAIT_FREQ + i*M_PI/N_LEGS));

/* GOOD — one line per step in KEY FORMULAS */
float phi   = wave_time * GAIT_FREQ + i * (M_PI / N_LEGS);
float upper = body_dir + LEG_SPLAY + SWING_AMP * sinf(phi);
Vec2  knee  = { hip.x + UPPER_LEN * cosf(upper),
                hip.y + UPPER_LEN * sinf(upper) };
```

## Linear flow inside functions

Top-to-bottom matches conceptual order. Guard clauses first; main work after. Three nesting levels = split or use early returns.

```c
static void rocket_tick(Rocket *r, ...) {
    if (!r->alive)             return;
    if (r->age >= LIFESPAN) {  r->alive = 0; return; }

    push_trail(r);
    apply_gene(r);
    advance_position(r);
    check_target_hit(r);
    check_offscreen(r);
}
```

## Function length discipline

- **≤ 30 lines** target.
- **Up to ~60** for orchestrators (`scene_tick`, `main`) where structure IS the documentation.
- **Past 60** = doing more than one thing — split.

## Group struct fields with one-line headers

15 ungrouped fields = a cliff. Group with one-line headers:

```c
typedef struct {
    /* trail buffer — circular head-position log */
    Vec2 trail[TRAIL_CAP];
    int  trail_head, trail_count;

    /* body joints — placed by arc-length sampling each tick */
    Vec2 joint[BODY_SEGS + 1];

    /* motion state */
    float heading, wave_time, turn_phase;
    float move_speed, turn_amp, turn_freq;

    /* legs — computed each frame from body joints */
    Vec2 leg_left [N_LEGS][3];
    Vec2 leg_right[N_LEGS][3];
} Centipede;
```

## Name for the concept, not the type

```c
/* BAD  */  float f, x1, x2;       int n;     int  flag;
/* GOOD */  float fitness;         int n_pop; bool hit_target;
            float target_x, launch_x;
```

Single-letter names (`i`, `dx`, `dy`, `t`) fine inside tight loops where context is one line away. Outside that: spell it out.

## Don't reach across §-sections

§5 entity must not call §7 ncurses. §7 screen must not mutate §5 state. §3 color must not know about entity geometry. Cross layers via parameters (`const` where read-only), not globals — exception: `g_app` / `g_running` flags written by signal handlers.

## Cleverness needs a one-line justification

```c
/* +TRAIL_CAP before % avoids C's negative-modulo trap */
return c->trail[(c->trail_head + TRAIL_CAP - k) % TRAIL_CAP];

/* dy negated so atan2f returns angle in math (y-up) coords, not
 * terminal (y-down) coords — matches what the eye sees */
float ang = atan2f(-dy, dx);
```

If you can't justify the trick in one line, write the slow obvious way.

## Acceptance test: a beginner can grok the algorithm from the code alone

A competent C programmer who has never seen the algorithm should — within ~10 minutes of top-to-bottom reading — name the algorithm, sketch it on paper, and predict each function from its signature alone. The four levers:

1. **Algorithm in plain English before any code.** CONCEPTS + MENTAL MODEL must teach the algorithm without function bodies. Test: delete every body — does the prose still teach? If not, expand ALGORITHM IN STEPS.
2. **Data joined when it travels together; separated when it doesn't.** `Vec2 pos` joins x+y because they always move together; `Scene` separates pools because they evolve on different rules. 15 ungrouped fields = failed separation; either group with headers or split.
3. **Names that explain themselves.** `wrap_pi(angle)` self-explains; `fix_a(x)` doesn't. Verb+noun for functions, entity name for structs, physical quantity (with units) for fields.
4. **Worked examples threaded through comments.** "At 60 Hz × 0.992 damping, a rope segment loses 38% of its speed in one second" beats "exponential decay applies". KEY FORMULAS has equations; HOW TO VERIFY plugs in defaults; EDGE CASES shows limits.

If a reader goes blank: fix prose first, then structure, then — last — code. Most learner-friendliness problems are explanation problems.
