---
name: novice-refactor
description: Trigger for the NOVICE_REFACTOR doctrine on a validated phase-2 C file — the textbook treatment for true beginners. Invoke when the user types `NOVICE_REFACTOR`, `COMPLETE_LITERATE` / `LITERATE` (legacy aliases), `/novice-refactor <filename>`, or asks to refactor a file so a first-time reader (no domain background) can learn the algorithm from zero. Reserve for the 5–10 canonical algorithm references in the project. Turns a working file into an embedded textbook via 3 sequential steps — (Step 0) color/HUD/themes, (Step 1) pseudocode-shaped code with extracted helpers, (Step 2) the full prose layer (file header + CONCEPTS + MENTAL MODEL with worked example + GUIDED TUTORIAL with construction-sequence lessons + Tier-1 function labels). Strictly clean-slate — read code only, never existing prose. For competent-learner files (the default case), use `default-refactor` instead.
---

# NOVICE_REFACTOR Doctrine

A NOVICE_REFACTOR pass turns a validated Phase-2 file into an embedded textbook for a true beginner — someone who hasn't met the algorithm before and needs analogies, worked examples, and a step-by-step construction sequence. Reserve for the **5–10 canonical algorithm references** in the project (the files someone would arrive at to LEARN a technique). For competent-learner files (the working middle case), use `default-refactor` instead — concept density + pseudocode, ~30 % less prose. For variants and already-known-audience demos, `default-refactor` also fits with reduced content depth. Canonical reference: `particle_systems/comet.c`.

## Mindset

- Clarity over cleverness.
- Intuition over performance — but never tank performance (Step 1 has explicit guardrails).
- Explicitness over compactness.
- Mental models over abstraction layers.

## Preconditions

- File passed Phase 2: algorithm validated by visual inspection, no known bugs.
- Compiles clean with `-Wall -Wextra`.

## Clean-slate rule — read code, never existing prose

**Do NOT read existing prose blocks.** Skip past them on the way to the code. Off-limits during a LITERATE pass: file header `/* ... */`, HOW TO READ block, CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL, per-§ section preambles, per-function comment blocks, and multi-line pedagogical comments inside function bodies.

What you DO read: function signatures, struct fields, function bodies, key constants in §1, the call tree implied by who-calls-whom. Build your inventory of the algorithm in your own words from that. Then write fresh prose against the inventory.

Why: stale framing leaks; "rewriting from scratch" after reading still leaves trace borrowings; the work IS re-deriving understanding from code, and skipping that defeats the doctrine. The rule applies to LITERATE first-pass, refinement-pass, AND UPDATE_LITERATE.

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

Goal: narrow scope in 30 seconds before doing real work.

## Token-efficient execution

The clean-slate rule says "don't read existing prose," but `Edit`'s `old_string` requires you to quote prose verbatim to delete it — so a naive workflow ends up reading hundreds of lines of prose anyway. These four patterns cut LITERATE's context cost roughly in half while preserving the doctrine.

### 1. Inventory by signatures, not bodies

Two greps give the whole map:

```bash
grep -nE '^(static|typedef|enum|int main|^#define)' <file>
grep -n  '^/\* §' <file>
```

Use the output to plan the Tier-3 list (5–10 orchestrators / non-obvious math). Read **full bodies only for those Tier-3 functions** — for Tier 1/2 the signature plus a glance at body length is enough; you're writing a one-liner anyway. Skip 200-line `Read` calls over spans you haven't earmarked for Tier 3.

### 2. Strip top-of-file prose with `sed`, don't quote it back

The file-top region (file header → HOW TO READ → CONCEPTS → MENTAL MODEL → GUIDED TUTORIAL) is one contiguous 400–500-line block where everything gets discarded. Delete it without loading it:

```bash
sed -i '1,/^#define _POSIX_C_SOURCE/{/^#define _POSIX_C_SOURCE/!d}' <file>
```

(adjust the closing anchor if the file uses a different first-code line — `#include` etc.)

Then ONE `Edit` anchored on `#define _POSIX_C_SOURCE` inserts the fresh prose above it. The `old_string` is one line; only the `new_string` (your new prose) costs context. Saves ~400 lines vs an `Edit` that quotes the whole old block.

### 3. One Edit per region, not per block

The five file-top blocks are contiguous and all discarded — bundle into ONE `Edit` (after the `sed` strip), not five.

Same for §-section preambles + their per-function comment blocks: if §3 has a preamble plus three function docs to rewrite, one `Edit` spanning all four beats four small ones.

Exception: when adjacent code BETWEEN comment blocks must stay verbatim, smaller Edits are safer. Judgment call.

### 4. Cache boundary lookups; minimise inter-Edit narration

Run the section/function grep ONCE per Step, then trust it. Re-grepping between every Edit is waste.

User-facing text: one sentence per Step ("rewriting §1–§4 prose") is enough. Avoid a "compile clean, moving on" line after each Edit — they add up. Save the compile-check for the end of each Step.

### Risk + mitigation

Scripted prose-stripping is mildly riskier than `Edit` (a misanchored `sed` could eat code). For our standard `/* ... */` top blocks ending at `#define _POSIX_C_SOURCE`, the regex is the same every file — but ALWAYS run `wc -l` and a compile immediately after the `sed` to confirm the strip landed where expected.

## The procedure — three sequential steps

```
  Step 0  ─── color / HUD / themes / debug overlay   (visual scaffolding)
  Step 1  ─── refactor code from scratch             (pseudocode-shaped bodies)
  Step 2  ─── refactor comments                      (the textbook around the code)
```

Each step must compile clean and visually verify before the next begins. No step is skipped.

**Clean-slate rule, restated:** Even when 80–90 % of existing themes/HUD/code/prose is good and reusable, throw it all out and rebuild from zero. If the rebuild ends up identical to what was there, that confirms the original was already correct — but the work was still done.

---

## STEP 0 — Color, HUD, themes

Visual scaffolding the textbook hangs off. Done first so Steps 1–2 reference real symbols.

### Color pairs

All ramps + HUD + hints initialised in `color_init()`. `PAIR_HUD` = bright yellow + `A_BOLD`; `PAIR_HINT` = bright cyan + `A_BOLD`. Every palette colour in the BRIGHT half of the 256-cube (see CLAUDE.md → *Theme Palette Brightness*).

### HUD

Canonical HUD spec is in CLAUDE.md → *HUD Standard*. Step 0 just verifies the file conforms.

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

---

## STEP 1 — Refactor code (pseudocode-shaped bodies)

The body of every function should READ like its pseudocode block. Lever: **helper extraction by activity**, not by line count. Performance preserved by guardrails below.

### Helper categories

Render (`scene_draw`, `*_draw_*`) and tick (`scene_tick`, `phase*_*`) code in particular should use helpers HEAVILY — that is where readers lose the thread.

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

## STEP 2 — Refactor comments (the slim textbook layer)

Only after Step 1 compiles clean. Step 2 produces FOUR prose blocks plus Tier-1 function labels — that's it. Everything else (HOW TO READ, per-§ preambles, Tier 2/3 per-function blocks, scattered inline pedagogy) was dropped as low ROI: token-expensive, redundant with the four required blocks.

The four required blocks, in order:

1. **§2.1 File header** — identifier metadata.
2. **§2.2 CONCEPTS** — what the algorithm IS, in 4 lines.
3. **§2.3 MENTAL MODEL** — the static photograph: 6 angles on the finished algorithm.
4. **§2.4 GUIDED TUTORIAL** — the construction sequence: how the algorithm gets BUILT, lesson by lesson.

Plus **§2.5 Tier-1 one-liners** above every function signature.

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
 *   r           reseed
 *   n / N p / P next / prev pattern
 *   t / T       next / prev theme
 *   + / -       faster / slower   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra <path>/<file>.c -o <name> -lncurses -lm
 */
```

### 2.2  CONCEPTS

Four subsections. 2–5 references.

| Subsection      | Contents                                                          |
|-----------------|-------------------------------------------------------------------|
| Algorithm       | One line — name + what it computes                                |
| Data-structure  | What holds the state and why                                       |
| Performance     | Why it fits in real time. Big-O per tick                           |
| References      | 2–5 of: original paper, Wikipedia, textbook chapter, sibling file  |

### 2.3  MENTAL MODEL

Six fixed subheadings IN ORDER. The static photograph of the finished algorithm — what it IS, from six complementary angles:

1. **CORE IDEA** — one paragraph. The single sentence the learner walks away with.
2. **HOW TO THINK ABOUT IT** — analogy from everyday intuition.
3. **ALGORITHM IN STEPS** — numbered, plain English, no pseudocode syntax.
4. **KEY FORMULAS** — every non-trivial equation with a one-line gloss.
5. **EDGE CASES TO WATCH** — what bites: boundary conditions, off-by-one traps.
6. **HOW TO VERIFY** — sanity checks the reader can do at runtime.

**≥ 1 ASCII diagram is required**, placed inside HOW TO THINK ABOUT IT (or another subsection where it earns its lines). One diagram is the minimum and usually the maximum — if you're tempted to add a second, ask whether the first is doing its job.

### 2.4  GUIDED TUTORIAL — incremental construction

**Distinct from MENTAL MODEL.** MENTAL MODEL photographs the finished algorithm; GUIDED TUTORIAL walks through how it gets BUILT. Each lesson EXPANDS the working program one mechanical step.

**4–6 lessons total.** Each lesson is a 3-part triplet, hard cap ~15 lines:

```
LESSON N — <one-line title naming the failure and the fix>

PROBLEM    The naive approach + the concrete failure mode it produces.
           Two or three lines.  Engage the reader's intuition; let them
           PREDICT the failure before the fix arrives.

FIX        The small mechanical change. Pseudocode where helpful.
           Two or three lines.

WHY/WHERE  One-line invariant the fix preserves + code anchor.
           (e.g. "→ scene_emit_trail() at §6")
```

**Lesson sequence — each lesson EXPANDS the working algorithm:**

1. Simplest viable version + its first failure.
2. Data layout that solves a constraint.
3. Per-tick algorithm (one lesson, NOT one-per-phase).
4. Coordinate bridges OR render/sim interface — skip if neither applies.
5. The polish that makes the visual work (lerp, brightness buckets, themes).

**Hard rules:**
- Cap each lesson at ~15 lines. If you can't fit, the lesson is doing two things — split.
- If you can't name a PROBLEM + FIX + INVARIANT, it isn't a lesson, it's exposition — remove it.
- Avoid duplicating MENTAL MODEL content. GT is a CONSTRUCTION SEQUENCE, not a re-explanation.
- Every lesson ends with a code anchor (`→ function_name() at §X`).

**Worked example (from comet.c):**

```
LESSON 1 — Trail without inheriting velocity → the moving-emitter trick

PROBLEM    A naive moving emitter spawns particles WITH the emitter's
           velocity.  Result: the particles fly along with the comet —
           the trail moves with the comet and no streak appears.

FIX        Spawn each particle with its OWN small velocity (zero, or a
           perpendicular drift), NOT the comet's velocity:
               p->vx = perp_x · kick · drift_factor;   (NOT c->vx)
               p->vy = perp_y · kick · drift_factor;

WHY/WHERE  Decoupling actor velocity from particle velocity is what makes
           the trail "fall behind" naturally.
           → scene_emit_trail() at §6.
```

### 2.5  Per-function comment — Tier 1 one-liners only

One short line above each function signature, naming what the function does in one phrase:

```c
/* round_to_cell — float pos → integer cell index. */
static inline int round_to_cell(float v) { ... }

/* scene_emit_trail — drop ONE trail particle at the comet's position. */
static void scene_emit_trail(Scene *s, const Comet *c) { ... }
```

That's it. **No PURPOSE / PSEUDOCODE / INPUTS / WHY blocks above functions** — the slim Step 2 explicitly drops Tier 2 and Tier 3.

If a function genuinely needs more (non-obvious math, an intricate orchestrator), put the explanation in MENTAL MODEL → KEY FORMULAS, or as a GUIDED TUTORIAL lesson, or as a `/* Step N — ... */` label inside the body (which is Step-1 code structure, not Step-2 prose) — never as an above-signature block.

### 2.6  Variable naming — long descriptive names

This rule belongs to Step 1's "named locals" pattern but is repeated here as a cross-reference. Scope-significant names get expanded; loop ephemera exempt:

| Before              | After                                   |
|---------------------|-----------------------------------------|
| `vr`, `vy`          | `velocity_radial`, `velocity_vertical`  |
| `T`, `rho`          | `temperature`, `density`                |
| `dt`                | `step_seconds`                          |
| `L`, `L_hot`        | `total_luminance`, `hot_luminance`      |
| `ro`, `rd`          | `cam.origin`, `direction`               |
| `p`, `div`          | `pressure`, `divergence`                |
| `ix`, `iy`          | `cell_x`, `cell_y`                      |
| `f`, `slot`         | `freshness`, `ramp_slot`                |

Forbidden: vague names, hidden state, magic numbers, compressed math, premature optimisation, generic abstractions, expert shorthand.

### 2.7  Delete-then-write order

Before writing any new prose: delete every existing prose block from the file (file header, HOW TO READ if present, CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL, per-§ preambles if present, per-function comment blocks, multi-line pedagogical comments inside function bodies). Then write fresh prose against the inventory you built from the code.

What stays:
- `static const` data-table column header comments (label data layout, not narrative).
- `/* Step N — ... */` labels inside function bodies (part of pseudocode-shaped code from Step 1).
- One-line clarifications next to single tricky expressions.

The delete-first order matters: keeping the old prose visible while writing the new prose invites lightly-edited rewrites and phrase leakage. See *Clean-slate rule* at top.

For the mechanics of deleting cheaply (without loading the prose into context), see *Token-efficient execution* above.

### Note on canonical reference

`particle_systems/comet.c` and `particle_systems/constellation.c` were produced under the OLD (full) doctrine — they include HOW TO READ blocks, per-§ preambles, and Tier 2/3 per-function blocks that the slim profile drops. Use them as references for the STRUCTURE of the four required blocks (file header, CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL), not as targets for total prose volume. Files written under the slim profile should run roughly 1.05×–1.3× the production line count, not 1.5×–2× as the old canonicals do.

---

## Length discipline

| Scope                | Target           | Hard cap |
|----------------------|------------------|----------|
| Function             | ≤ 30 lines       | ≤ 60 lines (orchestrators only) |
| §-section            | ≤ 100 lines      | —        |

**Section count: proportional, not absolute.** Roughly 1 section per 80–120 lines of code, with a floor of 5 for non-trivial files. A 400-line file lands at 5–6 sections; a 1500-line file at 10–15. Don't force splits that don't pay for themselves.

**File growth: NOT a target.** The acceptance test drives length. Under the slim profile, first-pass LITERATE runs land at roughly 1.05×–1.3× the production version (vs 1.5×–2× under the old full doctrine). If the read-aloud test passes at 1.1×, ship at 1.1×. If it fails at 1.5×, keep going. The multiplier is a sanity check, not a goal.

## Acceptance test (whole file)

A C programmer new to the technique should, within ten minutes of top-to-bottom reading:

1. Name the algorithm.
2. Sketch the data flow on paper.
3. Predict each function from its signature alone.
4. Identify which knob in §1 controls which visible effect.

If they can't, the failure path in order: **prose → structure → code**. Most learner-friendliness problems are explanation problems, not code problems.

**This test is the length governor.** Length is whatever it takes to pass the test, no more.
