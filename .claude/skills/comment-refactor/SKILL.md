---
name: comment-refactor
description: Trigger for the COMMENT_REFACTOR procedure on a phase-2-validated C file whose CODE is already in good shape (pseudocode-shaped from a prior refactor pass) but whose COMMENTS / PROSE need to be rebuilt. Invoke when the user types `COMMENT_REFACTOR`, `UPDATE_LITERATE` (legacy alias), `/comment-refactor <filename>`, or asks to "redo the comments on X". Defaults to the DEFAULT_REFACTOR profile (the competent-learner standard); pass `novice` as an arg (`/comment-refactor novice <file>`) to target NOVICE_REFACTOR instead. Erases every existing narrative comment block, then writes the chosen profile from zero against a code-only inventory. Code bodies are NOT touched.
---

# COMMENT_REFACTOR Procedure

COMMENT_REFACTOR is the comment-only sibling of `default-refactor` and `novice-refactor`. Use it when **the code is already good** (pseudocode-shaped from a prior refactor) but **the comments are missing, stale, or out-of-date**. Code bodies stay untouched; only the prose layer is rebuilt — from zero.

## When to use

- Code is already pseudocode-shaped (named locals, helpers extracted, `/* Step N */` labels), but comments are missing, stale, or pre-doctrine conventions.
- After a Phase 2 bug fix changed an algorithm — comments now lie.
- After a Step 1 structural refactor that introduced new helpers/layers — old per-function blocks don't cover the new symbols.
- After a doctrine update — switching a file from one profile (e.g. NOVICE) to another (e.g. DEFAULT).

## When NOT to use

- File code structure is monolithic (no helpers, no named locals, no layer functions). Push back: run `default-refactor` (or `novice-refactor`) Step 1 first, then COMMENT_REFACTOR.
- Phase 2 not yet validated. Pedagogy on a buggy algorithm wastes effort.

## Target profile

**Default: `default-refactor` profile** (concept density + pseudocode + heavy doc on data structures + drivers).

**Override: `novice-refactor` profile** — invoked by passing `novice` as the first argument:
```
/comment-refactor novice signal/dft_helloworld.c
```

Use `novice` only for the 5–10 canonical algorithm references where a true beginner needs the textbook (analogies, worked examples, GUIDED TUTORIAL with construction-sequence lessons).

The friction-free default is intentional — DEFAULT is the right answer for ~80 % of files, and the explicit override avoids an extra question per invocation.

## Token-efficient execution

The "delete every existing prose block" rule below would normally require quoting that prose verbatim into `Edit`'s `old_string` — which forces you to load it. These patterns sidestep that, preserving the code-only-inventory rule while cutting context cost roughly in half.

### Inventory by signatures, not bodies

```bash
grep -nE '^(static|typedef|enum|int main|^#define)' <file>
grep -n  '^/\* §' <file>
```

Use the output to plan the call tree. Read **full bodies only for drivers** (the functions that earn Tier-3 prose); for everything else, the signature is enough.

### Strip top-of-file prose with `sed`, don't quote it back

The contiguous top-of-file prose region is entirely discarded:

```bash
sed -i '1,/^#define _POSIX_C_SOURCE/{/^#define _POSIX_C_SOURCE/!d}' <file>
```

(adjust the closing anchor if the file's first code line is different — `#include`, etc.)

Then ONE `Edit` anchored on `#define _POSIX_C_SOURCE` inserts the fresh prose above it.

### One Edit per region, not per block

Bundle adjacent prose deletions+rewrites: a `§3` preamble + its three function blocks go in one `Edit`, not four.

### Cache greps; minimise inter-Edit narration

Run the section/function grep ONCE per phase, then trust it. One sentence per phase ("rewriting top-of-file prose") is enough — no per-Edit "compile clean, moving on" lines.

### Risk + mitigation

`sed` mis-anchoring could eat code. Run `wc -l` and a compile immediately after every strip to confirm.

## Procedure

### 1. Lock in the target profile

If the user invocation contained `novice`, target `novice-refactor`. Otherwise target `default-refactor`. Note the choice in your first user-facing sentence so they can override before any work begins.

### 2. Scan to inventory — CODE ONLY

Read the file's CODE only. Skip past every existing prose block — file header `/* ... */`, any HOW TO READ / CONCEPTS / MENTAL MODEL / GUIDED TUTORIAL / OVERALL PSEUDOCODE / DRIVER PSEUDOCODE block, per-§ preambles, per-function comment blocks, multi-line pedagogical comments inside bodies. Do not read them. Do not quote them. Do not paraphrase them.

What you DO read: function signatures, struct fields, function bodies, key constants in `§1`, the call tree implied by who-calls-whom. In working memory (NOT in a file), build:

- Every `§` section and what it owns.
- Every struct and its field meanings (from field declarations, not from comments).
- Every function (signature + 1-sentence summary in your own words).
- Every key constant in `§1`.
- The full call tree of the per-tick + per-frame drivers.

If you find yourself reading existing prose to understand the code, that's a signal the code is not pseudocode-shaped and you should be running `default-refactor` Step 1 first, not COMMENT_REFACTOR.

### 3. Erase ALL existing prose

Delete every narrative comment block:

- File header `/* ... */` block at top of file.
- Any HOW TO READ THIS FILE block.
- Any CONCEPTS / CONCEPTS & ALGORITHMS block.
- Any MENTAL MODEL block.
- Any GUIDED TUTORIAL block.
- Any OVERALL PSEUDOCODE / DRIVER PSEUDOCODE block.
- Per-§ section preamble comment blocks.
- Per-function comment blocks above signatures.
- Multi-line pedagogical comments inside function bodies.

For the file-top region, use the `sed` strip from *Token-efficient execution* above so you never load the prose into context. Per-§ and per-function blocks are usually short enough that bundled `Edit`s are fine.

### 4. Preserve (do NOT erase)

- `#include`s and forward declarations.
- All function bodies — including `/* Step N — ... */` labels and short inline trick justifications inside bodies. Those are part of Step 1's pseudocode-shaped code, not prose.
- `static const` data tables (themes, pattern_params, glyph ramps). One-line column-header comments labelling the table layout stay.
- Signal handlers, `main()` loop skeleton, key handler.

### 5. Apply the chosen profile from zero

Write the chosen Step 2 against your inventory from step 2, NOT against the deleted prose. Full specs live in the two target skills.

#### If target = DEFAULT_REFACTOR

See `.claude/skills/default-refactor/SKILL.md` for the full spec. Produces SEVEN pieces:

- **2.1  ABSTRACT** — 3–5 sentences. What the program computes / shows.
- **2.2  SECTION MAP** — one line per section, no preamble.
- **2.3  CONCEPTS & ALGORITHMS** — dense bullet list with references (5–10 entries).
- **2.4  OVERALL PSEUDOCODE** — the whole program in 10–20 lines of pseudocode.
- **2.5  DRIVER PSEUDOCODE** — for each driver (tick / draw / update / headline algorithm fn), 5–15 lines of pseudocode at top of file.
- **2.6  KEYS + BUILD** — standard.
- **2.7  REFERENCES** — full citations grouped into PAPERS + BOOKS only (no ONLINE / sibling files). 2–6 entries per category, ≤ 30 lines total.
- **2.8  Inline per-element prose**:
  - Tier 3 above each main DATA STRUCTURE (5–12 lines per struct).
  - Tier 3 above each DRIVER function (10–25 lines, repeats the top-of-file driver pseudocode for in-place reading).
  - Tier 1 (one-line) above ALGORITHMIC HELPERS only.
  - NOTHING above plumbing functions (signal handlers, screen wrappers, key dispatch, init/cleanup).
  - Inline switch-case comments where variants exist (2–4 lines per case).

If you find yourself wanting to add MENTAL MODEL / GUIDED TUTORIAL / HOW TO READ — stop. The default profile drops them on purpose. Either the file should be NOVICE (re-invoke `/comment-refactor novice <file>`) or the content belongs in CONCEPTS & ALGORITHMS / inline switch-case / a `/* Step N — ... */` label inside the body.

#### If target = NOVICE_REFACTOR

See `.claude/skills/novice-refactor/SKILL.md` for the full spec. Produces FIVE pieces:

- **2.1  File header** — identifier metadata (DEMO, Study alongside, Section map, Keys, Build).
- **2.2  CONCEPTS** — Algorithm / Data-structure / Performance / References (4 lines + 2–5 refs).
- **2.3  MENTAL MODEL** — six fixed subheadings (CORE IDEA, HOW TO THINK, ALGORITHM IN STEPS, KEY FORMULAS, EDGE CASES, HOW TO VERIFY) + ≥1 ASCII diagram.
- **2.4  GUIDED TUTORIAL** — 4–6 lessons, each a PROBLEM / FIX / WHY+WHERE triplet capped at ~15 lines. Construction sequence, NOT a re-explanation of MENTAL MODEL.
- **2.5  Tier-1 one-liners** above each function signature.

Drop everything not on this list (HOW TO READ blocks, per-§ preambles, Tier 2/3 per-function blocks, scattered ASCII diagrams).

## Acceptance

| Target | Test |
|---|---|
| DEFAULT_REFACTOR | 5-minute read: name algorithm + cite reference, trace OVERALL PSEUDOCODE, predict drivers from DRIVER PSEUDOCODE, identify each main data structure. |
| NOVICE_REFACTOR  | 10-minute read: name algorithm, sketch data flow on paper, predict each function from its signature alone, identify which knob in `§1` controls which visible effect. |

Code is unchanged in both modes; verify compile + visual are unchanged.

## Length budget

Depends on the target picked at invocation:

| Target | Multiplier (vs production line count) |
|---|---|
| DEFAULT_REFACTOR | 1.10×–1.25× |
| NOVICE_REFACTOR  | 1.20×–1.35× |

If the source file was previously written under an OLD verbose doctrine (HOW TO READ, per-§ preambles, Tier 2/3 on every function, scattered diagrams), COMMENT_REFACTOR will SHRINK the file in both target modes — that's expected and correct. The byte count is not the target; the relevant acceptance test is.
