# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## Modes of Operation

This file describes THREE operating modes.  Different rules apply to each;
when rules appear to contradict, the contradiction is resolved by which
mode is active.

| Mode | Trigger | Goal | Latitude |
|------|---------|------|----------|
| **Surgical edit** | bug fix, single feature, "change X to Y" | minimum diff that solves the request | rules in §3 *Surgical Changes* dominate; do **not** restructure or relabel beyond the request |
| **New file** | "write a new simulation", "add a demo for X" | a complete teaching artefact in one file | rules in *Learner-Friendly Code Standards / Structure* dominate; targets in *New Simulation Workflow* apply (250-450 lines) |
| **Pedagogical refactor** | trigger phrases listed in *Pedagogical Refactor Recipe* | turn an existing file into an embedded textbook | rules in the *Pedagogical Refactor Recipe* dominate; deliberately overrides "match existing style" and the 250-450-line target (1.5×-2× growth is expected) |

When in doubt about which mode applies, name it explicitly before
acting: *"This is a Surgical edit, so I'll only touch X."* / *"This is a
Pedagogical refactor, so I'll rewrite the whole file from first
principles."*

## Three-Phase Workflow

The natural lifecycle of a file in this codebase is a three-phase
loop.  Each phase activates a different mode (above) and a different
set of rules.  Knowing which phase you are in resolves most "what
should I do here?" questions.

### Phase 1 — Author (new code request)

**Trigger:** *"write a new X"*, *"add a demo for Y"*, *"create a
sphere-traced Z"*.  Anything that asks for a NEW file.

**Goal:** a working production version, validated by clean compile
and visual inspection.  Target 250-450 lines.

**Produce:**

- File header (DEMO + Section map + Keys + Build).
- CONCEPTS block — short paragraph per subsection, 2 references.
- MENTAL MODEL block — short paragraph per subheading, ≥ 1 ASCII diagram.
- §1..§N code following the *framework.c* template.
- HUD (yellow status row 0, cyan hint last row, both `A_BOLD`).
- Themes only if the user asks for them.

**Do NOT produce in phase 1:**

- No HOW TO READ THIS FILE block.
- No GUIDED TUTORIAL block.
- No debug overlays unless the user asks.
- No long-name expansion (write `vr` for now, not `velocity_radial`).
- No per-function teaching blocks beyond a one-line *what + why*.

The aim is FAST.  Ship a clean, working, framework-conformant file.
Pedagogy is phase 3's job, after the algorithm has been validated.

### Phase 2 — Iterate (execute → observe → fix)

**Trigger:** implicit — the file exists, the user is running it.

**Goal:** validated visual behaviour.  The user runs the program,
reports what they SEE, and we converge via surgical edits.

**What I do:**

- Diagnose visual symptoms (*"top half flickers, bottom half doesn't"*):
  trace the rendering pipeline, identify cause, propose minimum-diff fix.
- Make surgical edits per the rules in *§3 Surgical Changes*.
- One concern per turn.

**Do NOT do in phase 2:**

- No restructuring beyond the immediate request.
- No "while we're here, let me also…".
- No pedagogy upgrades — phase 3 hasn't started yet.

**End of phase 2** = explicit user approval (*"looks good"* / *"ship
it"* / *"perfect"*).  After approval the file is **validated**: the
algorithm produces what the user wanted to see.

### Phase 3 — Refactor for learnability (refactor request)

**Trigger:** any phrase from *Pedagogical Refactor Recipe*.

**Goal:** convert the validated production file into an embedded
textbook.  Expected file growth 1.5×-2×.

**Add (on top of the phase-1 file):**

- HOW TO READ THIS FILE block (15-25 lines).
- GUIDED TUTORIAL block (6-12 mini-tutorials with diagrams +
  pseudocode).
- Per-function teaching block (purpose / pseudocode / mental model /
  inputs+outputs+units / why-it-exists).
- Debug overlays (`d` / `D` cycles them).
- Long-name expansion across the file (per the recipe's variable-name
  table).
- Inline pedagogy where non-obvious concepts appear.

The pedagogical pass works **because** phase 2 has validated the
algorithm.  Without phase 2, the tutorials risk teaching something
incorrect.

### Orthogonal track: surgical edits to an existing file

**Trigger:** bug fix, theme addition, dead-code removal, *"remove
X"*, *"add themes to Y"*.

**Goal:** minimum diff that solves the request.  Surgical edits do
NOT move a file between phases — a phase-3 textbook file can still
take a phase-2-style surgical edit (e.g. a bug fix) without losing
its textbook status.

### When the workflow gets violated

- If **phase 2 hasn't ended** (no approval) and the user asks for
  phase 3: ask whether the algorithm is validated first.  Pedagogy
  on a buggy algorithm wastes effort.
- If the user asks for *"phase 1 with phase 3 quality"* (full
  textbook in the first pass): warn that the textbook may need
  rewriting after phase 2 finds bugs, and offer to do a minimal
  phase 1 first.
- If a file has never been validated by the user (e.g. an old file
  the user wrote alone), treat it as still phase-2-pending: ask
  whether the algorithm is correct before refactoring.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

---

# Learner-Friendly Code Standards

Every new file in this project is a teaching artifact, not just a program.
A reader should be able to understand the physics, the algorithm, and the
framework decisions without leaving the file. Follow these rules on every
new file and every significant addition.

## File Header (mandatory)

Every file opens with a block comment in this exact structure:

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

## CONCEPTS Block (mandatory, after includes, before §1)

Immediately after the file header, add a `/* ── CONCEPTS ── */` block:

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

References are mandatory: **minimum 2, maximum 5** per file.  Pick the
ones a learner would actually open next; quality beats quantity.
Examples of good references:
- Wikipedia article on the algorithm
- Original paper (Reynolds 1987 for boids, Stam 1999 for stable fluids, etc.)
- A textbook chapter (e.g. "Game Physics Engine Development — Millington §12")
- A web resource (e.g. Inigo Quilez SDF functions, Red Blob Games pathfinding)

## MENTAL MODEL Block (mandatory, after CONCEPTS, before §1)

CONCEPTS names the algorithm; MENTAL MODEL teaches a learner to *think* in
it. After the CONCEPTS block, add a `/* ── MENTAL MODEL ── */` block with
these six fixed sub-headings, in this order:

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

The canonical reference for this block is `grids/rect_grids/01_uniform_rect.c`
— study its MENTAL MODEL block before writing your own. It demonstrates the
right level of detail: enough that a learner can reimplement, not so much
that the actual code feels redundant.

CONCEPTS vs MENTAL MODEL — the distinction:
- **CONCEPTS** answers "what is this and where do I read more?" — algorithm
  name, data structure, references. Skim-friendly, citation-style.
- **MENTAL MODEL** answers "how do I think in this algorithm?" — analogies,
  steps, formulas, traps. The "if you read only one comment block, read this
  one" section.

Both are mandatory in every new file. Do not collapse them.

## Named Constants — No Magic Numbers

Every literal that carries meaning belongs in `§1 config` as a named constant.
The name must describe the context, not the value.

```c
/* BAD  */  float r = 4.0f;
/* GOOD */  #define BOID_SEPARATION_RADIUS  4.0f

/* BAD  */  if (age > 120) die();
/* GOOD */  if (age > PARTICLE_LIFETIME_TICKS) die();

/* BAD  */  vel *= 0.98f;
/* GOOD */  vel *= DRAG_COEFFICIENT;   /* air resistance per tick */
```

Group related constants together with a comment explaining the context:

```c
/* Separation behaviour — how far apart boids try to stay.
 * Increase SEPARATION_RADIUS to make the flock more spread out. */
#define SEPARATION_RADIUS   40.0f   /* pixels */
#define SEPARATION_FORCE   180.0f   /* pixels/sec² */
```

**What is NOT a magic number** (do not bother extracting these):

- `0`, `1`, `−1`, `2`            mathematical identity / unit values.
- `0.5f`                         halving (e.g. centring `(a+b)*0.5f`,
                                 a midpoint computation).
- `2.0f * M_PI`                  the constant 2π, when used inline
                                 (extracting to `TWO_PI` is fine but
                                 not required).
- Loop bounds that come from a struct field: `for (i = 0; i < n; i++)`.
- Index arithmetic: `i + 1`, `j - 1`, etc.
- Array dimensions when the array's name already encodes the size:
  `LUMI_GLYPHS[8]` does not need an `LUMI_GLYPHS_LEN = 8` constant.

Everything else with a meaning beyond the literal value belongs in
§1 with a unit-bearing name.  When unsure, extract.

## Function Comments — WHY, not WHAT

Every non-trivial function gets a comment block. Explain:
1. What this function does (one line)
2. **Why** the approach was chosen — what would break if done differently
3. Cross-reference the equivalent function in another file if one exists

```c
/*
 * boid_separate() — push boid away from neighbours within SEPARATION_RADIUS.
 *
 * Uses inverse-distance weighting: the closer a neighbour, the stronger
 * the push. Linear distance (1/d) is used instead of quadratic (1/d²)
 * because quadratic causes violent jitter when boids momentarily overlap.
 *
 * Equivalent to ball_collide() in bounce_ball.c for the "push apart" idea,
 * but here the force is continuous, not an impulse.
 */
```

For the §4 coordinate system, §7 ncurses double-buffer, §8 main loop, and
the fixed-step accumulator: copy the full explanatory comments from
`framework.c` verbatim — these are the canonical explanations and a learner
should see them in every file.

## §5 Entity — the simulation heart

Name the struct and functions after the simulation concept, not generics:
- `Boid` / `boid_tick()` not `Entity` / `entity_update()`
- `FluidCell` / `fluid_advect()` not `Cell` / `step()`

The struct fields must be commented with units and purpose:

```c
typedef struct {
    float px, py;   /* position — pixels                        */
    float vx, vy;   /* velocity — pixels / second               */
    float age;      /* seconds alive; dies at PARTICLE_LIFETIME */
    int   color;    /* ncurses color pair index (1–N_COLORS)    */
} Particle;
```

## Learner Checkpoints

At each section boundary, add a one-line comment pointing the learner forward:

```c
/* ── end §5 — to understand the rendering side, read §6 scene_draw() ── */
```

For any non-obvious physics formula, add the derivation or a named reference:

```c
/* Euler angle integration: θ += ω·dt  (small-angle approximation valid
 * when |θ| < 0.3 rad; see Goldstein "Classical Mechanics" §1.4) */
```

---

# Learner-Friendly Code Structure

The previous section covers *comments and documentation*. This section
covers *the code itself* — how to organize functions, control flow, data,
and naming so a reader can pick up the file cold and follow the thread
without backtracking.

**Test:** open the file and read top-to-bottom. If you ever stop and ask
"wait, where is this defined?" or "what is this doing?" — the code has
failed the readability test, regardless of how thorough the comments are.

## One concept per function

If a function name needs `_and_` to describe it, split it. The §5/§6/§7
sectioning is the same idea at file scale; apply it at function scale too.

```c
/* BAD  */ void update_and_draw(Boid *b, WINDOW *w);
/* GOOD */ void boid_tick(Boid *b, float dt);
           void boid_draw(const Boid *b, WINDOW *w);
```

A reader scanning the function list should be able to predict what each
one does from the name alone. If two functions could plausibly do the
same thing, one of them is misnamed or one of them shouldn't exist.

## Separate pure from mutating

Pure helpers take `const` and return values; mutating functions take a
non-const pointer to the entity. The signature alone tells the reader
whether state can change.

```c
static int  body_seg_pair (int i);                       /* pure   */
static Vec2 trail_sample  (const Centipede *c, float s); /* pure   */
static void trail_push    (Centipede *c, Vec2 pos);      /* mutate */
static void move_head     (Centipede *c, float dt, ...); /* mutate */
```

This makes data flow obvious: glance at the signatures and you know
which functions touch the world.

## Mirror the math in the code

If KEY FORMULAS lists steps 1–4, the function body has four lines, in
order, one per step. Don't fold steps to save lines — a learner needs
to read the formula and the code side-by-side and have them match.

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

Each intermediate variable is a label for a concept named in the math.
Names like `phi`, `upper`, `knee` map 1:1 onto the formulas.

## Linear flow inside functions

Top-to-bottom reading order should match conceptual order. Guard
clauses first; main work after. If you find yourself with three levels
of nesting (loop-inside-loop-inside-if), the function wants to be split
or have early returns added.

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

The reader holds the whole function in their head at once. No backwards
jumps, no "wait, what state are we in here?"

## Function length discipline

- **≤ 30 lines** is the target.
- **Up to ~60** is fine for orchestration functions (`scene_tick`,
  `compute_legs`, `main`) where the structure itself is the documentation.
- **Past 60 lines**, the function is doing more than one thing — split it.

## Group struct fields with one-line headers

A struct with 15 ungrouped fields is a cliff. Group related fields
together; one comment per group is enough.

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

A reader can scan the group headers and find the field they need
without parsing every line.

## Name for the concept, not the type

Names describe the simulation meaning, not the C type or the
mechanical action.

```c
/* BAD  */  float f, x1, x2;        int n;     int  flag;
/* GOOD */  float fitness;          int n_pop; bool hit_target;
            float target_x, launch_x;
```

Single-letter names (`i`, `dx`, `dy`, `t`) are fine *inside* tight
loops where the context is one line away. Outside tight context: spell
the concept out.

## Don't reach across §-sections

§5 entity must not call §7 ncurses. §7 screen must not mutate §5 state.
§3 color must not know about entity geometry. The section boundaries
name layers of abstraction; reaching across collapses the layering.

If data needs to cross a layer, pass it as a parameter (`const` where
read-only) rather than touching globals from another section. The
exception is `g_app` / `g_running` flags written by signal handlers —
those are unavoidable in C.

## Cleverness needs a one-line justification

If a line uses bit tricks, ternary chains, pointer arithmetic, or
non-obvious math identities, leave a comment proving the trick earns
its keep:

```c
/* +TRAIL_CAP before % avoids C's negative-modulo trap */
return c->trail[(c->trail_head + TRAIL_CAP - k) % TRAIL_CAP];

/* dy negated so atan2f returns angle in math (y-up) coords, not
 * terminal (y-down) coords — matches what the eye sees */
float ang = atan2f(-dy, dx);
```

If you can't justify the trick in one line, rewrite it the slow
obvious way. Cell-resolution terminal animation is rarely the place
where a clever trick matters more than readability.

## The reader's mental budget

A learner has limited working memory while reading. Every concept you
introduce — every variable, every helper, every macro — costs a slot.
Spend the budget on concepts the simulation genuinely requires, not on
abbreviations or premature optimizations. When in doubt: write the slow
obvious version, then measure before sacrificing clarity for speed.

## A beginner must be able to grok the algorithm from the code alone

This is the **acceptance test** for every other principle in this
section. Imagine a competent C programmer who has never seen the
algorithm before — say, a CS undergraduate who knows pointers but has
never heard of FABRIK, or boids, or position-Verlet. Open the file.
Read top-to-bottom. Within ~10 minutes, that reader should be able to
**name the algorithm, sketch it on paper, and predict what each
function does before reading its body**. If they can't, the file fails
this test.

The four levers that decide whether a file passes:

**1. Algorithm explanation in plain English, before any code.**
The CONCEPTS block names the algorithm; the MENTAL MODEL block teaches
the reader to *think in it*. By the time they reach §1 they should
already know — in words — what the file is going to compute, why it
works, and what the steps are. Code without that prelude is a maze; a
reader has to reverse-engineer the algorithm before they can study it.
**Test:** delete every function body. Does the prose alone teach the
algorithm? If not, expand MENTAL MODEL → ALGORITHM IN STEPS until it
does.

**2. Data joined when it travels together; separated when it doesn't.**
A `Boid` carries position + velocity + colour together because they
*move together* through the simulation. A `Scene` separates the boid
pool from the threat from the queue geometry because those evolve on
different rules. The shape of the data is the shape of the concept.

- *Join* fields that are always read or written in the same step
  (e.g. `pos.x, pos.y` → `Vec2 pos` because nothing ever uses one
  without the other).
- *Separate* concerns that change at different rates or for different
  reasons (e.g. simulation state vs. render snapshots vs. config —
  three different structs, even if they share field names).
- A struct with 15 ungrouped fields is a sign of failed separation: the
  reader cannot see at a glance which fields belong together. Add the
  one-line group headers (see "Group struct fields with one-line
  headers" above) or split the struct.

**Test:** for each struct field, ask "what other field do I always
touch alongside this one?" If the answer is always *the same group*,
the join is correct. If the answer differs by field, the struct should
be split.

**3. Names that explain themselves to a stranger.**
A reader should be able to read a function signature and predict what
it does. `wrap_pi(angle)` is self-explanatory; `fix_a(x)` is not.
`steer_separate(people, count, self)` describes a concept; `f1(p, n,
s)` describes nothing. This is the difference between code a learner
can study and code they have to *decipher* before they can study.

- Function names: verb + noun describing the simulation concept
  (`fabrik_forward_pass`, `apply_platform_collisions`, not `step`).
- Struct names: the entity name, not the data layout
  (`Centipede`, not `EntityArray`).
- Field names: the physical or conceptual quantity, with units in the
  comment (`heading /* radians */`, not `h`).

**Test:** show a function signature to someone who has not read the
file. Can they predict what it does? If not, rename.

**4. Worked example threaded through the comments.**
At every non-obvious step, anchor the explanation with concrete
numbers. "At 60 Hz × 0.992 damping, a rope segment loses 38 % of its
speed in one second" is teachable; "exponential decay applies" is
filler. KEY FORMULAS shows the equation; HOW TO VERIFY shows what
plugging in default values produces; EDGE CASES shows what happens at
the limits. A learner reading the file should never have to *imagine*
what the simulation does — the comments should walk them through one
specific run.

**The grok test.** When you finish a file, hand it to someone who has
never seen the algorithm. Tell them: "Read this for ten minutes. Then
without referring to the code, sketch on paper what happens each tick
and what each function does." If they can do it, the file passes. If
they go blank, fix the prose first, then the structure, then — only as
a last resort — the code. Most learner-friendliness problems are not
code problems; they are explanation problems.

---

# Pedagogical Refactor Recipe

When the user asks for a **"refactor for learnability"**, **"pedagogical
refactor"**, **"learnability rewrite"**, **"learning-friendly rewrite"**,
**"rewrite from first principles for learning"**, or any phrase combining
*refactor / rewrite* with *learn / teach / pedagogy / first principles*,
apply this recipe to the named file.

**This is NOT a production cleanup.** It is a deep rewrite that turns
the file into an embedded textbook: prose teaches the algorithm
step-by-step, and the working code is the worked exercises.

## Mindset

- Clarity over cleverness.
- Intuition over performance.
- Explicitness over compactness.
- Mental models over abstraction layers.

Rebuild from first principles. Don't tidy the existing code —
reconstruct the implementation as if explaining it to a beginner who
knows basic C syntax but nothing about the domain. Define the core
problem, explain why the technique exists, explain the
mathematical/geometric/system meaning, explain how information flows,
and build the implementation incrementally like a guided tutorial.

## Output structure (in addition to existing CLAUDE.md standards)

The rewritten file must contain, in order:

1. **File header** (per "File Header (mandatory)" above).
2. **HOW TO READ THIS FILE** — 15-25 lines explaining reading order,
   the long-name convention, and what background the reader needs.
3. **CONCEPTS block** (5 subsections, ≥ 2 references).
4. **MENTAL MODEL block** (all 6 subheadings, with at least one ASCII
   diagram).
5. **GUIDED TUTORIAL** — 6 to 12 numbered mini-tutorials that build the
   algorithm from first principles. Each tutorial:
     - opens with a question ("What is X?", "Why does Y happen?")
     - explains the idea in plain English first
     - includes an ASCII diagram or a worked example
     - ends with simplified pseudocode
   Tutorials should cover, in order: the core problem, the data layout,
   each per-step transformation, coordinate-system bridges, and the
   render/sim interface (where both exist).
6. **§1..§N actual code**, broken into many small sections (≥ 15 for
   non-trivial files), each ≤ ~100 lines and teaching one concept.

## For every section

Open with an educational preamble: what problem is being solved here,
why this logic exists, what assumptions are made, what inputs flow in,
what outputs flow out. The preamble is the reader's running orientation
— when they get lost, they should re-anchor by reading the section
header.

## For every function

A comment block above the signature describing:

- **Purpose** (one sentence).
- **Pseudocode** that mirrors the body 1:1.
- **Mental model** — analogy or visualisation that makes the
  operation concrete.
- **Inputs / outputs / units** — including coordinate systems if
  relevant (`world space`, `grid index space`, `screen pixels`, etc.).
- **Why this function exists architecturally** — what would break if
  it were merged with its caller.

The function body must mirror its pseudocode line-for-line, with a
short inline comment naming each math/physics concept it represents.
Length follows the canonical *Function length discipline* rule (≤ 30
target, ≤ 60 for orchestrators).

## Variable naming

Long descriptive names everywhere. Long names add visual weight but
turn every line into self-documentation. Examples of the conversion:

| Before              | After                                   |
|---------------------|-----------------------------------------|
| `vr`, `vy`          | `velocity_radial`, `velocity_vertical`  |
| `T`, `rho`          | `temperature`, `density`                |
| `dt`                | `step_seconds`                          |
| `T_view`            | `transmittance`                         |
| `dtau`              | `optical_depth_step`                    |
| `L`, `L_hot`        | `total_luminance`, `hot_luminance`      |
| `ro`, `rd`          | `cam.origin`, `direction`               |
| `p`, `div`          | `pressure`, `divergence`                |
| `buf1`, `buf2`      | `scratch_a`, `scratch_b`                |

Forbidden: vague names, hidden state, magic numbers (every literal
goes in §1 with a unit-bearing name), compressed math, premature
optimisation, overly generic abstractions, expert-only shorthand.

## Inline pedagogy

Where a non-obvious concept appears (geometry, physics intuition,
rendering pipelines, simulation flow, interpolation, coordinate
systems, state transitions, numerical approximation, memory layout,
signal flow), pause and **teach** the concept inline. Don't just name
it. The reader should never need an external textbook open while
reading.

## ASCII diagrams

Required wherever a spatial, temporal, or data-flow relationship is
hard to describe in prose. Common cases:

- Memory layout of a struct.
- Coordinate transformations and projections.
- Stencils on a grid (Laplacian, advection backward-trace).
- Pipeline stages with arrows.
- Ray paths through a volume.
- Time-step state transitions.
- Mesh winding or face orientation.
- Interpolation weighting (bilinear corners, trilinear cube).

## Educational debug helpers

Add optional debug overlays toggled by `d` / `D`. Each overlay
visualises one piece of intermediate state and teaches a specific
mental model. Examples by domain:

- **Fluid sim:** density / temperature / velocity-arrows / pressure /
  divergence views.
- **Raymarcher:** depth / normal / step-count / curvature / orbit-trap
  views.
- **Fractal:** iteration-count / escape-ratio heatmaps.
- **Rasteriser:** per-stage intermediate buffers (z-buffer, normal
  buffer, AO buffer, light buffer).
- **Cellular automaton:** rule-rate, density, age-since-flip overlays.

The debug helper's source code is itself part of the lesson — a 30-line
`render_debug_density` function teaches what "density field" means
more vividly than any prose comment. Document each overlay in the
section preamble: what it shows, what mental model it builds.

## Stacks on top of existing standards

This recipe **adds to** the existing CLAUDE.md standards, it never
overrides them. The pedagogical rewrite must still pass:

- The `── §N name ──` divider style.
- HUD spec (yellow status row 0, cyan hint last row, both `A_BOLD`).
- ASCII-only rendering at runtime.
- Theme palette brightness rules.
- *Function length discipline* (canonical rule above).
- Self-contained file rule (one `.c` file, no shared headers).
- Compile clean with `gcc -std=c11 -O2 -Wall -Wextra ... -lncurses -lm`,
  zero warnings.

## Expected outcome

Total file length typically 1.5× to 2× the production version. The
growth is in comments, tutorials, and debug helpers — never in the
algorithm itself.

## Acceptance test

Hand the rewritten file to a competent C programmer who has never seen
the technique. Within ten minutes of top-to-bottom reading they should
be able to:

- Name the algorithm and the problem it solves.
- Sketch the data flow on paper.
- Predict what each function does from its signature alone.
- Identify which knob in §1 controls which visual effect.

If they can't, the fix is almost always more prose, more diagrams,
more pseudocode — not more code.

---

# Working on This Project

## New Simulation Workflow

When asked to write a new simulation, clarify these before writing any code:

**Step 1 — Name the algorithm.** State what it computes and why it produces something worth watching.

**Step 2 — Choose coordinate space.** This is the first architectural decision:

| Use cell-space (skip §4) | Use pixel-space (include §4) |
|---|---|
| Grid/CA simulations — each cell IS one character | Continuous motion — balls, pendulums, particles, flocking |
| fire, sand, reaction-diffusion, flowfield, matrix_rain | bounce_ball, lorenz, nbody, cloth, boids |
| Position stored as `int row, col` | Position stored as `float px, py` in pixel units |

**Step 3 — Estimate scope.** A typical *new-file* (Surgical edit / new
demo) sits at 250-450 lines. State if it will be longer and why. If it
approaches 600+, the §5 physics is probably doing too much — split
into sub-functions.

> **Note (Pedagogical refactor mode):** the 250-450 target does NOT
> apply to pedagogical refactors.  The Pedagogical Refactor Recipe
> deliberately grows the file 1.5×-2× via tutorials, diagrams, and
> debug helpers — that growth is in comments and pedagogy, not in the
> algorithm.  See *Modes of Operation* at the top of this file.

**Step 4 — Write in this order:** §1 config → §5 entity/physics → §6 scene → §3 color → §7 screen → §8 app. Config first forces all magic numbers to be named before any logic is written.

---

## Verification (no tests — compile + visual = done)

There is no test suite. "Done" means all five of these:

1. `gcc -std=c11 -O2 -Wall -Wextra <file>.c -o name -lncurses -lm` — **zero warnings**
2. Runs at stable ~60 fps — no busy-loop, no spiral-of-death on slow terminals
3. HUD shows fps + sim parameters + PAUSED/running state
4. `q` / `ESC` exits cleanly — terminal restored, cursor visible
5. Resize (`SIGWINCH`) doesn't crash or corrupt display

**Debugging visual bugs:** When Tamil describes a visual problem ("particles going wrong direction", "rope not moving with bob"), treat the description as ground truth. Debug `scene_draw()` in §6 first — the mapping from physics state to screen coordinates is the most common source, not the physics math itself.

---

## HUD Standard

Every program has two fixed UI elements. Follow this layout and these
colours exactly — the HUD must remain legible against ANY animation
behind it (dark or bright, sparse or dense), so it always uses
**high-contrast bright colours and `A_BOLD`. Never `A_DIM`.**

Required colour pairs (define these in `§3 color_init()` for every demo):

```c
/* §3 color_init() — HUD pairs are reserved IDs across all demos.
 * Use 256-colour codes when COLORS >= 256, fall back to basic-8 below. */
init_pair(PAIR_HUD,  226, -1);   /* bright yellow on default bg */
init_pair(PAIR_HINT,  51, -1);   /* bright cyan   on default bg */
/* 8-colour fallback: COLOR_YELLOW for HUD, COLOR_CYAN for HINT.   */
```

Required draw block — the hint strip uses `A_BOLD` (NOT `A_DIM`) so it
stays readable when a bright animation flashes behind it:

```c
/* §7 screen_draw() — top-right: fps + params + state */
char buf[80];
snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ",
         fps, sim_fps, paused ? "PAUSED " : "running");
attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
mvprintw(0, cols - (int)strlen(buf), "%s", buf);
attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

/* bottom-left: one-line key hint strip — A_BOLD, never A_DIM */
attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
mvprintw(rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:<param> ");
attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
```

Why these specific colours:
- **226 (bright yellow)** for the status — pops against blue/red/green
  animations, dim enough to not glare on a black background.
- **51 (bright cyan)** for the hint — distinct hue from any common
  particle/fire/plasma palette, so the hint never camouflages.
- **`A_BOLD` on both** — many terminals render bold as a brighter shade
  of the foreground; combined with the already-bright 256-colour codes
  the HUD becomes near-white-bright, readable on every animation.

The key hint at the bottom must list every interactive key the program
uses. If the demo has a SECOND status line (e.g. parameter readouts), it
sits at `row 1` using `PAIR_HUD` without `A_BOLD` so the primary status
on `row 0` remains the dominant element.

---

## Theme Palette Brightness

**Every theme entry must sit in the BRIGHT HALF of the 256-colour space —
even the "deepest / darkest" tier of the ramp.** Avoid the bottom of the
palette: colours below ~24 in the 6×6×6 RGB cube and below ~240 in the
grayscale strip become invisible against a default-black terminal,
especially when the renderer applies `A_DIM`.

**Concrete rule of thumb:**

| Range                | Status                          |
|----------------------|---------------------------------|
| 16-23 (cube)         | NEVER use — too close to black, A_DIM = invisible |
| 232-239 (grayscale)  | NEVER use — same problem in gray    |
| 24-29 / 240-243      | Use sparingly; OK as the lowest ramp tier only |
| 30+ / 244+           | Safe everywhere                 |

**Why:** `A_DIM` typically renders at half intensity. Color 17 (a near-
black blue) with `A_DIM` is effectively invisible. Color 24 stays a
barely-visible navy that still reads as "dark theme tint". The trick is
that theme CHARACTER comes from the *relative gradient* — readers see a
clear progression from low to high — not from the absolute "darkness" of
the lowest entry. As long as ramp[0] < ramp[1] < … < ramp[N-1], the
theme works; pushing ramp[0] from 17 to 24 keeps the gradient and gains
visibility.

**Anti-pattern (do NOT do this):**
```c
{ "OCEAN", { 17,  18,  24,  31,  39,  51, 117, 195 }, 196,  21 },
//          ^^   ^^                                  ↑ both invisible with A_DIM
```

**Correct:**
```c
{ "OCEAN", { 24,  25,  31,  38,  45,  51, 117, 195 }, 196,  21 },
//          ^^^^^^                                    ↑ all visibly tinted
```

The hot/cold accent colours used for boundary highlighting, peak
flashes, etc. follow the same rule — pick from the bright half so the
accent reads regardless of `A_DIM` / `A_NORMAL` / `A_BOLD` rendering.

This applies to ALL theme palettes — biome ramps, plate tints, building
tints, star tints, line colours, backdrop sprinkles. Every theme cell
the user can land on through `t/T` cycling must be legible.

---

## ASCII-Only Rendering

**All runtime glyphs must be ASCII (codepoints 0x20–0x7E). No UTF-8
box-drawing, no Unicode block elements, no multi-byte characters in
`mvaddch` / `mvaddstr` output.**

Why: Unicode rendering depends on the user's terminal locale, font, and
ncurses link variant. On non-UTF-8 locales the multi-byte sequences
arrive as garbled bytes (e.g. `─` becomes `MMM`-style mojibake). ASCII
renders identically on every terminal.

The classic ASCII vocabulary covers every common simulation:

| Need | Use | Don't use |
|---|---|---|
| Horizontal line / wall  | `-`              | `─` `━` `═` |
| Vertical line / wall    | `\|`              | `│` `┃` `║` |
| Corners and junctions   | `+`              | `┌┐└┘├┤┬┴┼` |
| Filled cell / particle  | `#` `*` `@` `O`  | `█` `▓` `▒` |
| Trail / dot             | `.` `,` `'` `` ` `` | `·` `•` |
| Diagonal hint           | `/` `\`          | `╱` `╲` |

Distinguish similar tile types by **colour, intensity, and animation**,
not by glyph variety. wfc_showcase.c demonstrates the pattern: 33
junction tiles all render as `+`, with weight classes encoded in three
distinct colour pairs (cyan / pink / gold).

**Comment dividers** like `── §1 config ──` are exempt — they're source
text, not runtime output, and are part of the established project style.

**`setlocale(LC_ALL, "")`** in `screen_init()` is no longer required for
new files. Leave it out unless a specific reason demands it; if a future
file does use UTF-8 output, justify the choice in a comment and add the
build-time fallback (8-colour locale-C path).

---

## Common ncurses Bugs

These are the mistakes most likely to appear in generated code. Verify each before presenting:

| Bug | Correct form |
|---|---|
| `mvaddch(y, x, ch)` without cast | `mvaddch(y, x, (chtype)(unsigned char)ch)` — prevents sign-extension on chars > 127 |
| `clear()` each frame | `erase()` — `clear()` retransmits the full screen every frame, causes flicker |
| `refresh()` to flush | `wnoutrefresh(stdscr); doupdate();` — one diff write, no partial frames |
| Missing `typeahead(-1)` in screen_init | ncurses silently interrupts output to peek stdin — causes tearing |
| No `SIGWINCH` handler | Terminal resize corrupts display permanently |
| Missing `atexit(cleanup)` / `endwin()` on all exit paths | Terminal left in raw mode after crash or signal |

---

## Self-Contained File Rule

Every program is one `.c` file. Hard rules:

- No `#include "local_header.h"` — no shared headers of any kind
- No multi-file compilation — one `gcc` command, one binary
- No libraries beyond `ncurses` and `libm`
- If a function from another file is needed, copy it inline and note the source in a comment

---

# Project: Terminal Demos — ncurses C (C11)

## Build Pattern

```bash
gcc -std=c11 -O2 -Wall -Wextra <dir>/<file>.c -o <name> -lncurses [-lm]
```

Most files need `-lm`. A few cell-space sims (sandpile, hex_grid, bsp_tree, quadtree) omit it.

---

## Core Architecture

### Coordinate / Physics Model
- Physics lives in **pixel space** — `CELL_W=8`, `CELL_H=16` sub-pixels per cell
- **One conversion point**: `px_to_cell_x/y()` in `scene_draw()` — nowhere else
- Cell-space sims (fire, sand, matrix_rain, flowfield) omit `§4 coords` entirely

### Simulation Loop
- Fixed-timestep accumulator: `sim_accum += dt; while (sim_accum >= TICK_NS) { tick(); sim_accum -= TICK_NS; }`
- `dt` capped at 100 ms to prevent spiral-of-death
- Sleep **before** terminal I/O — stable frame cap regardless of write time

### Render Interpolation
- `alpha = sim_accum / TICK_NS` ∈ [0.0, 1.0)
- Constant velocity: `draw_pos = pos + vel * alpha * dt`
- Non-linear forces: `draw_pos = prev + (cur - prev) * alpha`

### ncurses Rendering
- Single `stdscr` — ncurses double-buffers internally; no manual WINDOW pair needed
- Frame sequence: `erase() → scene → HUD → wnoutrefresh(stdscr) → doupdate()`
- `typeahead(-1)` prevents input from interrupting diff write
- `erase()` not `clear()` — avoids full-screen retransmit every frame

### Signal Handling
- `SIGINT/SIGTERM` → `running = 0`; `SIGWINCH` → `need_resize = 1`
- All signal-written flags are `volatile sig_atomic_t`
- `atexit(cleanup)` calls `endwin()` — terminal always restored

---

## Raster Pipeline (`raster/*.c`)

```
tessellate → scene_tick (MVP) → pipeline_draw_mesh → fb_blit
               per triangle: vert shader → clip/NDC/screen → back-face cull
                             → rasterize (barycentric) → z-test → frag shader
                             → luma → Bayer dither → Bourke char → cbuf[]
```

- `ShaderProgram` splits `vert_uni` / `frag_uni` — prevents segfault when shaders need different uniform types
- `cbuf[]` decouples all rendering math from ncurses; `fb_blit()` is the only I/O boundary
- `zbuf[]` float depth buffer, init to `FLT_MAX`, z-tested per cell

---

## Memory Allocation

No dynamic allocation after init.  Allocate everything you need in
the init phase (or place it in BSS via static / global storage); the
hot path must not call `malloc` / `free`.  A few exceptions exist
where the cost of pre-sizing for the worst case would be silly
(`tessellate`, `flowfield`, `sand` — initial mesh / particle pool
allocations); document those at the call site.

The other rules that used to live in this section are stated
elsewhere — see *Function Comments — WHY, not WHAT*, *Named
Constants*, *Common ncurses Bugs*, *Signal Handling*, and
*Verification*.

---

## Documentation

| File | Contents |
|---|---|
| `documentation/Architecture.md` | Framework design, loop mechanics, coordinate model, per-subsystem deep dives |
| `documentation/Master.md` | Long-form essays on algorithms, physics, and visual techniques |
| `documentation/Visual.md` | ncurses field guide — V1–V9, Quick-Reference Matrix, Technique Index |
| `documentation/COLOR.md` | Color tricks — palettes, escape-time coloring, density coloring, 256-color patterns |
