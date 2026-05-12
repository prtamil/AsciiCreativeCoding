# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## Modes & Phases

A file in this codebase passes through three phases. Each activates a different mode with different rules — when rules appear to contradict, the active mode wins.

| Phase / Mode | Trigger | Goal | Rules dominate |
|---|---|---|---|
| **Phase 1 — Author** (new file) | "write a new X", "add a demo for Y" | working production version, validated by clean compile + visual inspection. 250-450 lines. | *Learner-Friendly Code Standards / Structure*; *New Simulation Workflow* |
| **Phase 2 — Iterate** (surgical edit) | bug fix, "change X to Y", visual symptom report | minimum diff that solves the request | §3 *Surgical Changes* |
| **Phase 3 — LITERATE refactor** | the word `LITERATE`, `/literate <file>`, or any phrase combining *refactor/rewrite* with *learn/teach/pedagogy/first principles* | turn validated file into embedded textbook via the LITERATE 3-step procedure. **Don't read existing prose** — read code only, inventory the algorithm in your own words, write fresh prose from the inventory. Length is whatever the acceptance test requires (typically 1.5×–2× on Phase-1 files; less on already-pedagogical ones). | *LITERATE Refactor Doctrine*; overrides "match existing style" and the 250-450 target |
| **Phase 3b — UPDATE_LITERATE** (comment-only) | the word `UPDATE_LITERATE`, `/update-literate <file>`, "redo the comments on X" | assumes Step 0 (color/HUD/themes) and Step 1 (code) already done; **read code only**, erase all narrative prose, rewrite the comment layer from scratch via LITERATE Step 2 | *UPDATE_LITERATE Procedure* |

When unsure which mode applies, name it explicitly: *"This is a Surgical edit, so I'll only touch X."*

A surgical edit (bug fix, theme add) does NOT move a file between phases — a phase-3 textbook can take a phase-2 fix without losing textbook status.

### Phase 1 — Author

**Produce:** file header, CONCEPTS, MENTAL MODEL, §1..§N code following framework.c, HUD. Themes only if requested.

**Do NOT:** add HOW TO READ THIS FILE, GUIDED TUTORIAL, debug overlays, long-name expansion, or per-function teaching blocks. Pedagogy is phase 3's job.

### Phase 2 — Iterate

The user runs the program, reports what they see; converge via surgical edits. One concern per turn. No restructuring beyond the request, no "while we're here".

**End** = explicit user approval ("looks good" / "ship it"). The file is then **validated**.

### Phase 3 — LITERATE refactor

Invoked by the trigger word `LITERATE` (or `/literate <filename>`). Turns a validated phase-1 file into an embedded textbook via a fixed 3-step procedure:

```
  Step 0 ─── color / HUD / themes / debug overlay   (visual scaffolding)
  Step 1 ─── refactor code from scratch             (pseudocode-shaped bodies)
  Step 2 ─── refactor comments                      (the textbook around the code)
```

**STRICTLY clean-slate.** Even when 80–90 % of the existing themes/HUD/code/prose is good and reusable, throw it all out and rebuild from zero. The point is to force a fresh teaching pass at every layer; preserving "good enough" pieces leaks stale framing and prevents the doctrine from doing its job. If the rebuild ends up identical to what was there, that confirms the original was already correct — but the work was still done.

Full procedure: **LITERATE Refactor Doctrine** at the bottom of this file. Phase 3 works **because** phase 2 has validated the algorithm.

### Phase 3b — UPDATE_LITERATE (comment-only refresh)

Invoked by `UPDATE_LITERATE` (or `/update-literate <filename>`). The comment-only sibling of LITERATE — useful when Step 0 (color/HUD/themes/debug) and Step 1 (code structure) are already in good shape but the comment layer is missing, stale, or pre-LITERATE.

Procedure: scan the code to inventory in your own words, **erase EVERY existing prose comment block**, then apply LITERATE Step 2 (all ten subsections) from zero. Code bodies are not touched; only narrative comments are erased and rewritten.

Full procedure: **UPDATE_LITERATE Procedure** at the bottom of this file.

### Phase violations

- LITERATE requested before phase 2 ended → ask whether the algorithm is validated. Pedagogy on a buggy algorithm wastes effort.
- UPDATE_LITERATE requested on a file whose code is NOT pseudocode-shaped (no helpers, no named locals, no layer functions) → push back: run LITERATE Step 1 first, then UPDATE_LITERATE.
- "Phase 1 with phase 3 quality" → warn that the textbook may need rewriting after phase 2 finds bugs; offer minimal phase 1 first.
- Old file the user wrote alone, never validated → treat as phase-2-pending.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

- State assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them — don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- Notice unrelated dead code? Mention it — don't delete.
- Remove imports/variables YOUR changes orphaned; don't remove pre-existing dead code unless asked.

The test: every changed line traces directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan with a `verify:` clause per step. Strong success criteria let you loop independently.

---

# Learner-Friendly Code Standards

Every new file is a teaching artifact. A reader must understand the physics, the algorithm, and the framework decisions without leaving the file.

## File Header (mandatory)

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

## CONCEPTS Block (mandatory)

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

## MENTAL MODEL Block (mandatory)

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

Canonical reference: `grids/rect_grids/01_uniform_rect.c`. Both blocks are mandatory and distinct — do not collapse.

## Named Constants — No Magic Numbers

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

## Function Comments — WHY, not WHAT

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

## §5 Entity — Name for the Concept

Struct + functions named after the simulation concept, not generics: `Boid`/`boid_tick()` not `Entity`/`entity_update()`. Fields commented with units.

```c
typedef struct {
    float px, py;   /* position — pixels                        */
    float vx, vy;   /* velocity — pixels / second               */
    float age;      /* seconds alive; dies at PARTICLE_LIFETIME */
    int   color;    /* ncurses color pair index (1–N_COLORS)    */
} Particle;
```

## Learner Checkpoints

At each section boundary: `/* ── end §5 — to understand rendering, read §6 scene_draw() ── */`. For non-obvious physics formulas, add the derivation or a named reference inline.

---

# Learner-Friendly Code Structure

The previous section covers comments. This one covers the code itself — how to organize functions, control flow, data, naming so a reader can pick up the file cold.

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

---

# LITERATE Refactor Doctrine

> **Trigger:** the word `LITERATE`, `/literate <filename>`, or any phrase combining *refactor/rewrite* with *learn/teach/pedagogy/first principles*.

A LITERATE pass turns a validated Phase-2 file into an embedded textbook. The reader learns the algorithm by reading the file top-to-bottom — no external docs required. Canonical reference: `particle_systems/comet.c`.

## Mindset

- Clarity over cleverness.
- Intuition over performance — but never tank performance (Step 1 has explicit guardrails).
- Explicitness over compactness.
- Mental models over abstraction layers.

## Preconditions

- File passed Phase 2: algorithm validated by visual inspection, no known bugs.
- Compiles clean with `-Wall -Wextra`.

## Clean-slate rule — read code, never existing prose

**Do NOT read existing prose blocks.** Skip past them on the way to the code. Specifically: file header `/* ... */`, HOW TO READ block, CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL, per-§ section preambles, per-function comment blocks, and multi-line pedagogical comments inside function bodies — all of these are off-limits during a LITERATE pass.

What you DO read: function signatures, struct fields, function bodies, key constants in §1, the call tree implied by who-calls-whom. Build your inventory of the algorithm in your own words from that. Then write fresh prose against the inventory.

Three reasons this rule matters:
1. **Eliminates stale framing.** Existing prose may carry framing that's wrong, partial, or domain-jargon-heavy. Not reading it removes the temptation to lightly edit it.
2. **Eliminates phrase leakage.** Reading the original then "rewriting from scratch" still leaves trace borrowings. Skipping it entirely is mechanically simpler and bigger reduction in token cost.
3. **Forces the cognitive lift.** The work IS re-deriving understanding from code. Skipping that step defeats the doctrine.

The rule applies to LITERATE first-pass AND refinement-pass AND UPDATE_LITERATE. The output may end up similar to what was there — that's fine; what matters is the fresh pass was done from the code.

## Up-front clarifying questions

Before reading the file, pause with one tool call asking 2–3 scope questions. Don't burn tokens reading code you're going to skip, or skipping work the user wants done.

**Q1 (always): Scope.**
- Full LITERATE (Step 0 + 1 + 2).
- Skip Step 0 — themes / HUD / debug are already correct; refactor code + comments only.
- Comments only — switch to UPDATE_LITERATE (cheaper, no code touched).

**Q2 (when Step 0 is in scope): Add features not yet present?**
- Yes: 10 doctrine themes + debug overlay.
- Themes only.
- Skip — file already has everything it needs.

**Q3 (when uncertain): Section structure?**
- Keep current section count if it's natural for the file.
- Match the canonical breakdown (one section per concept, typically 8–12 for a 1000-line file).

Don't make this a long interrogation — the goal is to narrow scope in 30 seconds before doing real work.

## The procedure — three sequential steps

```
  Step 0  ─── color / HUD / themes / debug overlay   (visual scaffolding)
  Step 1  ─── refactor code from scratch             (pseudocode-shaped bodies)
  Step 2  ─── refactor comments                      (the textbook around the code)
```

Each step must compile clean and visually verify before the next begins. No step is skipped.

---

## STEP 0 — Color, HUD, themes, debug overlay

Visual scaffolding the textbook hangs off. Done first so Steps 1–2 reference real symbols.

### Color pairs

All ramps + HUD + hints initialised in `color_init()`. `PAIR_HUD` = bright yellow + `A_BOLD`; `PAIR_HINT` = bright cyan + `A_BOLD`. Every palette colour in the BRIGHT half of the 256-cube (see *Theme Palette Brightness*).

### HUD

Two fixed UI lines, both `A_BOLD`:

- Top row (right-aligned): `fps · sim:Hz · STATE`.
- Bottom row: every interactive key, plus the active theme name.

### Themes — 10 by default

Override only on explicit user request.

| # | Name    | Feel                                       |
|---|---------|--------------------------------------------|
| 1 | matrix  | green-on-black, terminal classic            |
| 2 | neon    | hot magenta + cyan, 80s arcade              |
| 3 | nova    | white-hot core, red rim                     |
| 4 | ocean   | deep blue → cyan → white                    |
| 5 | fire    | red → orange → yellow                       |
| 6 | toxic   | acid green → yellow                         |
| 7 | gold    | warm browns → cream                         |
| 8 | ice     | dark blue → pale cyan                       |
| 9 | aurora  | green / cyan / magenta multi-hue            |
| 10| plasma  | purple → pink → cyan                        |

Each theme is an 8-step `ramp[0..7]` (cool/dim → hot/bright) plus dedicated `head`/`halo`/`accent` colours. Cycled with `t` / `T`; theme name shown in HUD.

### Debug overlay

Toggled with `d` / `D`. Cycles through visualisations of intermediate simulation state — ONE piece per overlay. Domain examples:

| Domain         | Overlays                                                       |
|----------------|----------------------------------------------------------------|
| Fluid          | density / temperature / velocity / pressure / divergence        |
| Raymarcher     | depth / normal / step-count / curvature / orbit-trap            |
| Particle       | spawn-rate / per-pool counts / age histogram                    |
| Fractal        | iteration-count / escape-ratio                                  |
| Rasteriser     | per-stage buffers (z / normal / AO / light)                     |
| Cellular auto. | rule-rate / density / age-since-flip                            |

The debug overlay's source IS part of the lesson — write it as cleanly as the main render.

---

## STEP 1 — Refactor code (pseudocode-shaped bodies)

The body of every function should READ like its pseudocode block. Lever: **helper extraction by activity**, not by line count. Performance preserved by guardrails below.

### Helper categories

Every helper falls in one of these slots. Render (`scene_draw`, `*_draw_*`) and tick (`scene_tick`, `phase*_*`) code in particular should use helpers HEAVILY — that is where readers lose the thread.

| Category         | Purpose                                  | Examples (from `comet.c`)                                        |
|------------------|------------------------------------------|------------------------------------------------------------------|
| Calculation      | Pure: inputs → derived value             | `round_to_cell`, `trail_freshness`, `ramp_slot_from_freshness`   |
| Predicate        | Pure, returns bool                       | `cell_visible`, `comet_off_screen`                               |
| Update / mutate  | Modifies ONE struct in place             | `comet_apply_plasma_kick`, `comet_emit_trail_particles`          |
| Algorithmic step | Performs one named activity              | `paint_cell`, `scene_emit_trail`, `blast_ignite`                 |
| Layer / phase    | Orchestrator over a pool or render stage | `scene_draw_trail_layer`, `phase2_advance_all_comets`            |

### Pattern A — Pull intermediate expressions into named locals

Every non-trivial expression gets a name BEFORE it is used.

```c
/* BAD — context hides inside expressions */
int ix = (int)(p->x + 0.5f);
if (ix < 0 || ix >= cols) continue;
float f = 1.0f - p->age / p->life;
int slot = (int)(f * 7.999f);

/* GOOD — each step is a named noun */
int   cell_x    = round_to_cell(p->x);
int   cell_y    = round_to_cell(p->y);
if (!cell_visible(cell_x, cell_y, cols, rows_playable)) continue;
float freshness = trail_freshness(p->age, p->life);
int   ramp_slot = ramp_slot_from_freshness(freshness);
```

A helper pays its way the first time it removes ambiguity at the call site, not the third time it deduplicates code.

### Pattern B — Orchestrators as tables of contents

Long functions become call trees. The reader descends to the level of detail they need.

```c
static void scene_draw(const Scene *s) {
    int rows_playable = s->rows - 1;
    scene_draw_trail_layer (s, rows_playable);   /* background */
    scene_draw_blast_layer (s, rows_playable);   /* mid-ground */
    scene_draw_comet_layer (s, rows_playable);   /* foreground */
}
```

The §-section preamble (written in Step 2) lists the full call tree so the reader has a roadmap.

### Pattern C — Step-labelled comments inside bodies

When a function's pseudocode block lists N steps, repeat those labels inside the body:

```c
static void scene_emit_trail(Scene *s, const Comet *c) {
    /* Step 1 — find an inactive TrailParticle slot; bail if pool full. */
    int slot_index = trail_pool_find_inactive(s);
    if (slot_index < 0) return;
    /* Step 2 — perpendicular unit vector to the comet's flight direction. */
    float comet_speed = sqrtf(c->vx * c->vx + c->vy * c->vy);
    ...
}
```

### Performance guardrails

Clarity-over-performance is bounded. Pedagogy must not regress frame rate.

- Small helpers (≤ 10 lines) → `static inline`.
- No malloc in the hot path. All pools allocated in init.
- Don't recompute a value in two helpers if one helper can pass it as argument.
- Don't add a 3-layer call chain around a 3-line core — that is over-engineered.
- If a helper changes a measured benchmark by > 5 %, inline it back and accept the longer function.

### Exception — keep loop ephemera short

`i`, `j`, `dx`, `dy`, `t` remain OK inside ≤ 5-line loop bodies. The naming table in Step 2 applies to *scope-significant* values, not iteration counters. Renaming `i` to `loop_index` is noise.

### Acceptance for Step 1

Read each function body aloud, one line at a time. If a line requires re-reading or mental arithmetic to understand its purpose, it deserves a named helper or a named local. After extraction every function reads as one verb per line. Compile + visual unchanged.

---

## STEP 2 — Refactor comments (the textbook)

Only after Step 1 compiles clean. Each block below is mandatory. Pattern derived from `particle_systems/comet.c` — that file is the canonical template; consult it for full examples of every block.

### 2.1  File header

```c
/*
 * <filename>.c — <one-line visual description>
 *
 * DEMO: <2-4 sentences. What does the user see? What techniques,
 *        algorithms, and tricks does it demonstrate?>
 *
 * Study alongside:
 *   <2-3 sibling files. For each: 1 line on what is shared and
 *    what differs.>
 *
 * Section map:
 *   §1 config   — constants, themes, per-pattern parameters
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — palette pairs + theme cycle
 *   §4 <entity> — primary struct + spawn + tick
 *   §5 <entity> — secondary struct(s)
 *   §6 scene    — pools, tick, draw orchestration
 *   §7 screen   — ncurses init / draw / resize
 *   §8 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC     quit              spc        pause / resume
 *   r           reseed            d / D      cycle debug overlay
 *   n / N p / P next / prev pattern
 *   t / T       next / prev theme
 *   + / -       faster / slower   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra <path>/<file>.c -o <name> -lncurses -lm
 */
```

### 2.2  HOW TO READ THIS FILE

~10–15 lines. TWO subsections (the section map in the file header already gives the reading order, so don't duplicate it):

- **NAMING** — one-line glossary for every significant identifier (struct, key constant, hot helper).
- **BACKGROUND ASSUMED** — bullet list of prerequisite concepts (object pools, explicit Euler, ncurses double-buffer, etc.).

If a non-obvious section should be read first or last, mark it inline in the file header's section map with `(start here)` / `(skip on first read)`.

### 2.3  CONCEPTS

Four subsections (Rendering is dropped — the section map + per-§ preambles already cover it). 2–5 references.

| Subsection      | Contents                                                          |
|-----------------|-------------------------------------------------------------------|
| Algorithm       | One line — name + what it computes                                |
| Data-structure  | What holds the state and why                                       |
| Performance     | Why it fits in real time. Big-O per tick                           |
| References      | 2–5 of: original paper, Wikipedia, textbook chapter, sibling file  |

### 2.4  MENTAL MODEL

Six fixed subheadings IN ORDER:

1. **CORE IDEA** — one paragraph. The single sentence the learner walks away with.
2. **HOW TO THINK ABOUT IT** — analogy from everyday intuition.
3. **ALGORITHM IN STEPS** — numbered, plain English, no pseudocode syntax.
4. **KEY FORMULAS** — every non-trivial equation with a one-line gloss.
5. **EDGE CASES TO WATCH** — what bites: boundary conditions, off-by-one traps.
6. **HOW TO VERIFY** — sanity checks the reader can do at runtime.

≥ 1 ASCII diagram somewhere in the file.

### 2.5  GUIDED TUTORIAL

**6–7 numbered mini-lessons.** Tight target — if a lesson overlaps an adjacent one, merge them. Each lesson: opens with a question, plain English first, ASCII diagram or worked example where it helps, ends with the pseudocode or struct that maps onto a real symbol below.

Cover IN ORDER, one lesson each:

1. The core problem (what does the algorithm compute, and why is the obvious approach wrong).
2. Data layout (what struct holds what, why N structs not one).
3. The per-tick algorithm phases (one lesson, NOT one-per-phase).
4. Coordinate-system bridges (cell ↔ pixel, polar ↔ Cartesian) — skip if there are none.
5. Render / sim interface (the painter's algorithm, layer order).
6. Themes + debug overlay (what they expose, why they exist).

Lessons 4 and 6 are optional depending on the file. Default landing zone: 6 lessons.

### 2.6  Per-§ section preamble

Length proportional to section size. Don't write a 15-line preamble for a 30-line section.

| Section size                       | Preamble                                                        |
|------------------------------------|-----------------------------------------------------------------|
| Small (< 40 lines, 1 struct or 3 small functions) | 2–3 lines: name what's in it, done.                 |
| Medium (40–100 lines)              | 5–8 lines: what problem it solves + named members.              |
| Large (orchestrator / CA / pipeline) | 10–15 lines: problem, why-here, inputs/outputs, call tree.    |

### 2.7  Per-function comment block — TIERED

Per-function blocks come in three sizes. Most functions get Tier 1 or Tier 2. **Tier 3 caps at ~10 functions per file** — only the orchestrators and non-obvious math.

**Tier 1 — trivial helper (≤ 10-line body).** One-line comment above the signature.
```c
/* round_to_cell — float pos → integer cell index. */
static inline int round_to_cell(float v) { ... }
```

**Tier 2 — default for algorithmic steps.** Two fields: Purpose + Pseudocode.
```c
/*
 * paint_cell — wrap mvaddch in attron/off so callers stay one-line-clear.
 *
 *   attron(pair | attr); mvaddch(y, x, glyph); attroff(...).
 */
```

**Tier 3 — orchestrators, coordinate transforms, non-obvious math.** Four fields:
- **Purpose** — one sentence.
- **Pseudocode** — mirrors body 1:1.
- **Inputs / outputs / units** — with coordinate systems where applicable.
- **Why it exists** — what would break if merged with caller.

(The previous 5-field block had a "Mental model" sub-field; it's dropped. The file-level MENTAL MODEL section already handles analogies. Per-function analogies were almost always forced restatements of Purpose.)

**Upgrade-to-Tier-3 triggers (any one):**
- Function owns a call tree (scene_tick, scene_draw, bitmap_draw, main).
- Function does coordinate-system transformation.
- Function implements non-obvious math (decay table, FS dithering, gamma, polar emission).

Body mirrors pseudocode line-for-line (Step 1 already enforced this).

### 2.8  Variable naming — long descriptive names

Scope-significant names get expanded (loop ephemera exempt per Step 1):

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
| `ix`, `iy`          | `cell_x`, `cell_y`                      |
| `f`, `slot`         | `freshness`, `ramp_slot`                |

Forbidden: vague names, hidden state, magic numbers, compressed math, premature optimisation, generic abstractions, expert shorthand.

### 2.9  Inline pedagogy + ASCII diagrams

Pause and teach inline where non-obvious concepts appear: geometry, physics, rendering pipelines, interpolation, coordinate systems, state transitions, numerical approximation, memory layout, signal flow.

ASCII diagrams REQUIRED for: struct memory layout, coordinate transformations, grid stencils (Laplacian, advection backward-trace), pipeline stages, ray paths through volumes, time-step state transitions, mesh winding, interpolation weighting.

### 2.10  Delete-then-write order

Before writing any new prose: delete every existing prose block from the file (file header, HOW TO READ, CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL, per-§ preambles, per-function comment blocks, multi-line pedagogical comments inside function bodies). Then write fresh prose against the inventory you built from the code.

What stays:
- `static const` data-table column header comments (label data layout, not narrative).
- `/* Step N — ... */` labels inside function bodies (part of pseudocode-shaped code from Step 1).
- One-line clarifications next to single tricky expressions.

The delete-first order matters: keeping the old prose visible while writing the new prose invites lightly-edited rewrites and phrase leakage. See the *Clean-slate rule* at the top of this doctrine.

---

## Length discipline

| Scope                | Target           | Hard cap |
|----------------------|------------------|----------|
| Function             | ≤ 30 lines       | ≤ 60 lines (orchestrators only) |
| §-section            | ≤ 100 lines      | —        |

**Section count: proportional, not absolute.** Roughly 1 section per 80–120 lines of code, with a floor of 5 for non-trivial files. A 400-line file lands at 5–6 sections; a 1500-line file at 10–15. Don't force splits that don't pay for themselves.

**File growth: NOT a target.** The acceptance test drives length. Most first-pass LITERATE runs land at 1.5×–2× the production version; refinement passes land at 1.05×–1.2×. If the read-aloud test passes at 1.1×, ship at 1.1×. If it fails at 2×, keep going past 2×. The multiplier is a sanity check, not a goal.

## Acceptance test (whole file)

A C programmer new to the technique should, within ten minutes of top-to-bottom reading:

1. Name the algorithm.
2. Sketch the data flow on paper.
3. Predict each function from its signature alone.
4. Identify which knob in §1 controls which visible effect.

If they can't, the failure path in order: **prose → structure → code**. Most learner-friendliness problems are explanation problems, not code problems.

**This test is the length governor.** Length is whatever it takes to pass the test, no more.

---

# UPDATE_LITERATE Procedure

> **Trigger:** the word `UPDATE_LITERATE`, `/update-literate <filename>`, or "redo the comments on X".

UPDATE_LITERATE is the comment-only sibling of LITERATE. It assumes Step 0 (color/HUD/themes/debug overlay) and Step 1 (pseudocode-shaped code with extracted helpers) are already done; only the prose layer is rebuilt — from zero.

## When to use

- Code is already pseudocode-shaped from a prior LITERATE pass, but comments are missing, stale, or pre-LITERATE conventions.
- After a Phase 2 bug fix changed an algorithm — comments now lie.
- After a Step 1 structural refactor that introduced new helpers/layers — old per-function blocks don't cover the new symbols.

## When NOT to use

- File code structure is monolithic (no helpers, no named locals, no layer functions). Push back: run LITERATE Step 1 first, then UPDATE_LITERATE.
- Phase 2 not yet validated. Pedagogy on a buggy algorithm wastes effort.

## Procedure

### 1. Scan to inventory — CODE ONLY

Read the file's CODE only. Skip past every existing prose block — file header `/* ... */`, HOW TO READ, CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL, per-§ preambles, per-function comment blocks, multi-line pedagogical comments inside bodies. Do not read them. Do not quote them. Do not paraphrase them.

What you DO read: function signatures, struct fields, function bodies, key constants in §1, the call tree implied by who-calls-whom. In working memory (NOT in a file), build:

- Every `§` section and what it owns.
- Every struct and its field meanings (from field declarations, not from comments).
- Every function (signature + 1-sentence summary in your own words).
- Every key constant in §1.
- The full call tree of the per-tick + per-frame orchestrators.

If you find yourself reading existing prose to understand the code, that's a signal the code is not pseudocode-shaped and you should be running LITERATE Step 1 first, not UPDATE_LITERATE.

### 2. Erase ALL existing prose

Delete every narrative comment block. Specifically:

- File header `/* ... */` block at top of file.
- `HOW TO READ THIS FILE` block.
- `CONCEPTS` block.
- `MENTAL MODEL` block.
- `GUIDED TUTORIAL` block.
- Per-§ section preamble comment blocks.
- Per-function comment blocks above signatures.
- Multi-line pedagogical comments inside function bodies.

### 3. Preserve (do NOT erase)

- `#include`s and forward declarations.
- All function bodies — including `/* Step N — ... */` labels and short inline trick justifications inside bodies. Those are part of Step 1's pseudocode-shaped code, not prose.
- `static const` data tables (themes, pattern_params, glyph ramps). One-line column-header comments labelling the table layout stay.
- Signal handlers, `main()` loop skeleton, key handler.

### 4. Apply LITERATE Step 2 from zero

Write all ten Step 2 subsections (2.1 file header → 2.10 clean-slate compliance) against your inventory from step 1, NOT against the deleted prose. Canonical reference: `particle_systems/comet.c`.

## Acceptance

Same 10-minute read-aloud test as LITERATE. Code is unchanged; verify compile + visual are unchanged.

## Length budget

Growth is whatever it takes — typically the file doubles if it had thin comments to start, or grows 1.5× if it had pre-LITERATE prose. The byte count is not the target; the read-aloud test is.

---

# Working on This Project

## New Simulation Workflow

Before writing code, clarify:

1. **Name the algorithm** — what it computes, why it produces something worth watching.
2. **Choose coordinate space** — first architectural decision:

| Cell-space (skip §4) | Pixel-space (include §4) |
|---|---|
| Grid/CA — each cell IS one character | Continuous motion |
| fire, sand, reaction-diffusion, flowfield, matrix_rain | bounce_ball, lorenz, nbody, cloth, boids |
| `int row, col` | `float px, py` in pixel units |

3. **Estimate scope** — typical phase-1 file sits at 250-450 lines. State if longer and why. Approaching 600+ = §5 is doing too much; split. (Phase 3 pedagogical refactors deliberately grow 1.5×-2×.)
4. **Write order:** §1 config → §5 entity/physics → §6 scene → §3 color → §7 screen → §8 app. Config first forces all magic numbers to be named before any logic.

## Verification (no tests — compile + visual = done)

"Done" means all five:

1. `gcc -std=c11 -O2 -Wall -Wextra <file>.c -o name -lncurses -lm` — **zero warnings**
2. Stable ~60 fps — no busy-loop, no spiral-of-death on slow terminals
3. HUD shows fps + sim parameters + PAUSED/running state
4. `q`/`ESC` exits cleanly — terminal restored, cursor visible
5. Resize (`SIGWINCH`) doesn't crash or corrupt display

**Debugging visual bugs:** when Tamil describes a visual problem, treat the description as ground truth. Debug `scene_draw()` in §6 first — the physics→screen mapping is the most common source, not the physics math itself.

## HUD Standard

Two fixed UI elements, **bright high-contrast colours + `A_BOLD`, never `A_DIM`** so they remain legible against any animation.

```c
/* §3 color_init() — HUD pairs reserved across all demos */
init_pair(PAIR_HUD,  226, -1);   /* bright yellow on default bg */
init_pair(PAIR_HINT,  51, -1);   /* bright cyan   on default bg */
/* 8-colour fallback: COLOR_YELLOW for HUD, COLOR_CYAN for HINT */

/* §7 screen_draw() */
char buf[80];
snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ",
         fps, sim_fps, paused ? "PAUSED " : "running");
attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
mvprintw(0, cols - (int)strlen(buf), "%s", buf);
attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
mvprintw(rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:<param> ");
attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
```

The bottom hint must list every interactive key. A second status line (parameter readouts) sits at row 1 with `PAIR_HUD` without `A_BOLD` so row 0 stays dominant.

## Theme Palette Brightness

Every theme entry must sit in the **bright half** of the 256-colour space — even the "darkest" tier. Bottom-of-palette colours become invisible against default-black with `A_DIM`.

| Range | Status |
|---|---|
| 16-23 (cube), 232-239 (gray) | NEVER use — A_DIM = invisible |
| 24-29 / 240-243 | OK as lowest ramp tier only |
| 30+ / 244+ | Safe everywhere |

Theme character comes from the *relative gradient*, not absolute darkness. Pushing ramp[0] from 17→24 keeps the gradient and gains visibility.

```c
/* BAD  */ { "OCEAN", { 17, 18, 24, 31, 39, 51, 117, 195 }, ... },  /* 17,18 invisible */
/* GOOD */ { "OCEAN", { 24, 25, 31, 38, 45, 51, 117, 195 }, ... },
```

Applies to ALL theme palettes — biome ramps, plate tints, line colours, accents. Every cell reachable via `t/T` cycling must be legible.

## ASCII-Only Rendering

All runtime glyphs ASCII (0x20–0x7E). No UTF-8 box-drawing, no Unicode block elements. Multi-byte sequences become mojibake on non-UTF-8 locales; ASCII renders identically everywhere.

| Need | Use | Don't |
|---|---|---|
| Horizontal line / wall | `-` | `─` `━` `═` |
| Vertical line / wall | `\|` | `│` `┃` `║` |
| Corners / junctions | `+` | `┌┐└┘├┤┬┴┼` |
| Filled cell / particle | `#` `*` `@` `O` | `█` `▓` `▒` |
| Trail / dot | `.` `,` `'` `` ` `` | `·` `•` |
| Diagonal | `/` `\` | `╱` `╲` |

Distinguish similar tile types by **colour, intensity, animation** — not glyph variety. wfc_showcase.c: 33 junction tiles all render as `+`, weight classes encoded in three colour pairs.

Comment dividers (`── §1 config ──`) are exempt — source text, not runtime output. `setlocale(LC_ALL, "")` no longer required for new files; justify in a comment if a future file needs UTF-8 output.

## Common ncurses Bugs

| Bug | Correct form |
|---|---|
| `mvaddch(y, x, ch)` without cast | `mvaddch(y, x, (chtype)(unsigned char)ch)` — prevents sign-extension on chars > 127 |
| `clear()` each frame | `erase()` — `clear()` retransmits full screen, causes flicker |
| `refresh()` to flush | `wnoutrefresh(stdscr); doupdate();` — one diff write |
| Missing `typeahead(-1)` in screen_init | ncurses interrupts output to peek stdin — causes tearing |
| No `SIGWINCH` handler | Resize permanently corrupts display |
| Missing `atexit(cleanup)` / `endwin()` | Terminal left in raw mode after crash |

## Self-Contained File Rule

One `.c` file per program:
- No `#include "local_header.h"` — no shared headers
- No multi-file compilation — one `gcc`, one binary
- No libraries beyond `ncurses` and `libm`
- Need a function from another file? Copy inline, note the source.

---

# Project: Terminal Demos — ncurses C (C11)

## Build

```bash
gcc -std=c11 -O2 -Wall -Wextra <dir>/<file>.c -o <name> -lncurses [-lm]
```

Most files need `-lm`. A few cell-space sims (sandpile, hex_grid, bsp_tree, quadtree) omit it.

## Core Architecture

**Coordinate / Physics**
- Physics lives in **pixel space** — `CELL_W=8`, `CELL_H=16` sub-pixels per cell
- **One conversion point**: `px_to_cell_x/y()` in `scene_draw()` — nowhere else
- Cell-space sims (fire, sand, matrix_rain, flowfield) omit `§4 coords` entirely

**Simulation Loop**
- Fixed-timestep accumulator: `sim_accum += dt; while (sim_accum >= TICK_NS) { tick(); sim_accum -= TICK_NS; }`
- `dt` capped at 100 ms to prevent spiral-of-death
- Sleep **before** terminal I/O — stable frame cap regardless of write time

**Render Interpolation**
- `alpha = sim_accum / TICK_NS` ∈ [0.0, 1.0)
- Constant velocity: `draw_pos = pos + vel * alpha * dt`
- Non-linear forces: `draw_pos = prev + (cur - prev) * alpha`

**ncurses Rendering**
- Single `stdscr` — ncurses double-buffers internally
- Frame: `erase() → scene → HUD → wnoutrefresh(stdscr) → doupdate()`
- `typeahead(-1)` prevents input from interrupting diff write

**Signal Handling**
- `SIGINT/SIGTERM` → `running = 0`; `SIGWINCH` → `need_resize = 1`
- Signal-written flags are `volatile sig_atomic_t`
- `atexit(cleanup)` calls `endwin()`

## Raster Pipeline (`raster/*.c`)

```
tessellate → scene_tick (MVP) → pipeline_draw_mesh → fb_blit
               per triangle: vert shader → clip/NDC/screen → back-face cull
                             → rasterize (barycentric) → z-test → frag shader
                             → luma → Bayer dither → Bourke char → cbuf[]
```

- `ShaderProgram` splits `vert_uni`/`frag_uni` — prevents segfault when shaders need different uniform types
- `cbuf[]` decouples rendering math from ncurses; `fb_blit()` is the only I/O boundary
- `zbuf[]` float depth buffer, init to `FLT_MAX`, z-tested per cell

## Memory Allocation

No dynamic allocation after init. Allocate everything in init phase (or BSS via static/global storage); the hot path must not call `malloc`/`free`. Documented exceptions: `tessellate`, `flowfield`, `sand` (initial mesh / particle pool allocations).

## Documentation

| File | Contents |
|---|---|
| `documentation/Architecture.md` | Framework design, loop mechanics, coordinate model, per-subsystem deep dives |
| `documentation/Master.md` | Long-form essays on algorithms, physics, visual techniques |
| `documentation/Visual.md` | ncurses field guide — V1–V9, Quick-Reference Matrix, Technique Index |
| `documentation/COLOR.md` | Color tricks — palettes, escape-time/density coloring, 256-color patterns |
