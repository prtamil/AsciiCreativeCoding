---
name: default-refactor
description: Trigger for the DEFAULT_REFACTOR doctrine on a validated phase-2 C file — the default choice for competent learners. Invoke when the user types `DEFAULT_REFACTOR`, `/default-refactor <filename>`, or asks to refactor a file for learning where the reader is a competent programmer (knows C, knows general programming, may not know the domain). Concept density + pseudocode-first instead of analogies and worked examples. Heavy doc on data structures + driver functions; light everywhere else. ~30% less prose than `novice-refactor` and better suited to anyone who isn't a true beginner. For absolute beginners who need analogies and worked examples, use `novice-refactor` instead.
---

# DEFAULT_REFACTOR Doctrine

A DEFAULT_REFACTOR pass turns a validated Phase-2 file into a competent-learner artefact. The reader knows C, knows general programming, and either knows the domain or wants to learn it from concept references + dense pseudocode — NOT from analogies, worked numerical examples, or step-by-step tutorials.

This is the **default** choice for ~80 % of files. Use `novice-refactor` only for the 5–10 canonical algorithm references where a true beginner needs the textbook.

## Mindset

- **Concept density over prose volume.** Reference theorems, papers, and Wikipedia by name; don't re-derive them.
- **Pseudocode replaces mental model.** Show the algorithm as code — competent readers reason about code faster than analogies.
- **Heavy where it earns the lines: data structures and driver functions.** Light everywhere else.
- **No bullshit.** No "let me explain again from a different angle". One pass at each idea, in the right place.

## Preconditions

- File passed Phase 2: algorithm validated by visual inspection, no known bugs.
- Compiles clean with `-Wall -Wextra`.

## Up-front clarifying questions

Before reading the file, pause with one tool call asking 1–2 scope questions.

**Q1 (always): Scope.**
- Full DEFAULT_REFACTOR (Step 0 + 1 + 2).
- Skip Step 0 — themes/HUD already correct; do Step 1 + Step 2.
- Comments only — switch to `comment-refactor` (cheaper, no code touched).

**Q2 (when scope includes Step 1+2): Drivers.**
Which functions are the drivers — the per-frame state-mutation + state-render orchestrators? Usually `tick`, `update`, `step`, `draw`, `render`, plus any headline algorithm function (`convolve`, `dft`, `nbody_step`, etc.). 1–4 functions total. These get heavy refactor in Step 1 and Tier-3 prose in Step 2.

If the file has no obvious drivers — pure library code, no main loop — push back: this skill expects an animated demo with at least one per-frame driver.

## Token-efficient execution

Same patterns as `novice-refactor` (sed-strip top prose, signature-first inventory, one Edit per region, cache greps, minimise narration). Re-stated tightly:

```bash
# Inventory
grep -nE '^(static|typedef|enum|int main|^#define)' <file>
grep -n  '^/\* §' <file>

# Strip top-of-file prose without loading it
sed -i '1,/^#define _POSIX_C_SOURCE/{/^#define _POSIX_C_SOURCE/!d}' <file>
```

Then ONE `Edit` anchored on `#define _POSIX_C_SOURCE` inserts the fresh prose.

DEFAULT is naturally token-efficient — moderate prose budget (~225 lines for a 1200-line file vs ~330 for novice).

## The procedure — three sequential steps

```
  Step 0  ─── color / HUD / themes                           (visual scaffolding)
  Step 1  ─── refactor code, focused on drivers              (heavy on drivers; light on plumbing)
  Step 2  ─── 7 fixed prose pieces, top-of-file + inline     (concept density + pseudocode)
```

---

## STEP 0 — Color, HUD, themes

Identical to `novice-refactor` Step 0:

- 7–10 themes cycled with `t` / `T`.
- HUD bar (top-right, `PAIR_HUD` + `A_BOLD`) + HINT bar (bottom-left, `PAIR_HINT` + `A_BOLD`).
- Theme-invariant pairs (HUD/HINT) sit OUTSIDE the theme-rebound range.

Skip if Step 0 is already in place.

---

## STEP 1 — Refactor code (focused on drivers)

Effort distribution:

| Code region | Treatment |
|---|---|
| **Drivers** (tick / draw / update / step / headline algorithm function from Q2) | Heavy. Named locals everywhere. Helpers extracted by activity. `/* Step N — ... */` labels matching the pseudocode you'll write in Step 2. Body reads as one verb per line. |
| **Algorithmic helpers** (the things drivers call) | Clean: named locals, sensible signatures, but minimal extraction. Don't decompose a 10-line helper into three 3-line helpers. |
| **Plumbing** (signal handlers, key dispatch, init/cleanup, screen wrappers, clock primitives) | Leave alone. No refactor unless there's a real bug. |

The principle: refactor effort = concentrated where the algorithm lives.

### Helper categories (for driver decomposition)

| Category | Purpose | Example |
|---|---|---|
| Calculation | Pure: inputs → derived value | `round_to_cell`, `compute_lerp_positions` |
| Predicate | Pure, returns bool | `cell_visible`, `comet_off_screen` |
| Update | Mutates ONE struct in place | `comet_apply_plasma_kick` |
| Algorithmic step | Performs one named activity | `scene_emit_trail`, `blast_ignite` |
| Layer / phase | Orchestrator over a pool or render stage | `scene_draw_trail_layer` |

### Performance guardrails

- Small helpers (≤ 10 lines) → `static inline`.
- No malloc in the hot path. All pools allocated in init.
- Don't recompute a value in two helpers if one helper can pass it as argument.
- Don't add a 3-layer call chain around a 3-line core — over-engineered.
- If a helper changes a measured benchmark by > 5 %, inline it back.

### Acceptance for Step 1

Read each driver body aloud, one line at a time. If a line requires re-reading or mental arithmetic, it deserves a named helper or a named local. After extraction every driver reads as one verb per line. Compile + visual unchanged.

---

## STEP 2 — Prose layer (8 fixed pieces, in order)

Only after Step 1 compiles clean. Step 2 produces EIGHT pieces. Top-of-file pieces (#1–#7) are written as one bundled Edit after the sed-strip. Inline pieces (#8) are per-region.

### 2.1  ABSTRACT (3–5 sentences)

What the program computes / shows. No analogies, no manual-style controls walkthrough. The gist.

```c
/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * <filename>.c — <one-line>
 *
 * <3-5 sentences. What this program computes, what algorithm it uses,
 * what the user sees on screen.  No analogies, no worked examples,
 * no step-by-step controls walkthrough — the keys block at #6 covers
 * controls.>
 */
```

### 2.2  SECTION MAP

```c
/*
 * Section map
 *   §1 config       constants
 *   §2 clock        monotonic-ns timer + sleep
 *   §3 color        palette pairs + theme cycle
 *   §4 <entity>     primary struct + spawn + tick
 *   ...
 */
```

One line per section. No preamble under any section divider — the section map IS the contents page.

### 2.3  CONCEPTS & ALGORITHMS

Dense bullet list. Names of theorems, papers, Wikipedia entries, sibling files.

```c
/* ── CONCEPTS & ALGORITHMS ───────────────────────────────────────────── *
 *
 *   <concept name>     <one-line description>
 *                      → <reference: paper / wiki / sibling file>
 *   <algorithm name>   <one-line description>
 *                      → <reference>
 *   <data-structure>   <one-line description>
 *                      → <reference>
 *
 * ─────────────────────────────────────────────────────────────────────── */
```

5–10 entries. Each is two lines max. Reader walks away knowing the names of the concepts and where to read more — replacing the pages of CONCEPTS / MENTAL MODEL prose that NOVICE_REFACTOR writes.

### 2.4  OVERALL PSEUDOCODE

The WHOLE program in 10–20 lines of pseudocode.

```c
/* ── OVERALL PSEUDOCODE ──────────────────────────────────────────────── *
 *
 *   init:
 *     install signals; start ncurses; init palettes; init scene
 *
 *   loop while running:
 *     handle resize if pending
 *     dt = clock_now - last_frame_time   (capped)
 *     while sim_accum >= tick_ns:        fixed-step physics
 *       scene_tick(dt_sec)
 *     alpha = sim_accum / tick_ns        sub-tick fractional for lerp
 *     scene_draw(alpha)
 *     handle one keystroke (non-blocking)
 *     sleep to ~60 fps
 *
 *   cleanup:
 *     restore terminal
 *
 * ─────────────────────────────────────────────────────────────────────── */
```

This is the elevator pitch as code. Reader who reads only this knows the file's whole shape.

### 2.5  DRIVER PSEUDOCODE (per driver)

For each driver from Q2, 5–15 lines of pseudocode at top-of-file (NOT yet above the function — that comes in #7). Lives at top so the reader has the map BEFORE descending into code.

```c
/* ── DRIVER PSEUDOCODE ───────────────────────────────────────────────── *
 *
 * scene_tick(dt):
 *   if paused:  return
 *   for each star in stars[0..n-1]:
 *     prev_pos = pos                   ; save for lerp
 *     v += uniform(±WANDER_ACCEL) · dt ; bounded random walk
 *     if |v| > SPEED_CAP: clamp magnitude
 *     pos += v · dt                    ; explicit Euler
 *     for each wall: snap + reflect    ; elastic collision
 *
 * scene_draw(alpha):
 *   lerp every star to (dpx, dpy) pixel + (dcx, dcy) cell
 *   pair scan over i < j:
 *     cull on dist_sq ≥ CONNECT_DIST²
 *     pick brightness bucket from ratio
 *     draw_line(thin Bresenham + slope glyph + stipple + first-writer-wins)
 *   paint stars LAST (foreground always wins)
 *
 * ─────────────────────────────────────────────────────────────────────── */
```

For each driver: 1–2 lines on WHY this order / approach (e.g. "step 1 must precede 2-4 so prev tracks the LAST tick"). Keep tight — the pseudocode IS the explanation.

### 2.6  KEYS + BUILD

Standard.

```c
/*
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   <every interactive key>
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra <path>/<file>.c \
 *       -o <name> -lncurses -lm
 */
```

### 2.7  REFERENCES — full citations, grouped

Two categories only: **PAPERS** and **BOOKS**. No ONLINE / Wikipedia / sibling-files sections — those are noise; if a reference matters, it's in print and citable.

```c
/* ── REFERENCES ──────────────────────────────────────────────────────── *
 *
 * PAPERS
 *   Author (year)
 *     "Title"
 *     Journal Vol(Issue): pages.   (one-line note if non-obvious)
 *   ...
 *
 * BOOKS
 *   Author — "Title"  (Publisher, year)
 *     Chapter / section pointer + one-line note.
 *   ...
 *
 * ─────────────────────────────────────────────────────────────────────── */
```

Inline refs in §2.3 (CONCEPTS & ALGORITHMS) stay full-citation as before — slight duplication is acceptable; the inline serves the reader scanning concepts, the REFERENCES block serves the reader hunting for the source.

If the file genuinely has no papers and no books to cite (rare for an algorithm file), omit the block. If it has only one citation total, ask whether the file should be DEFAULT or could survive as a no-prose code-only file.

### 2.8  Inline per-element prose

Inside the code body:

#### Tier 3 above each main DATA STRUCTURE

```c
/*
 * Star — the actor.  Drift across the screen with bounded random walk;
 * connection lines are derived from pairs at draw time, not stored.
 *
 *   px, py            position THIS tick      (pixels, float)
 *   prev_px, prev_py  position at LAST tick   (pixels) — for render lerp
 *   vx, vy            velocity                (pixels/sec)
 *   color             colour-pair id          1..N_STAR_COLORS
 *   ch                glyph chosen at spawn from "*+o@."
 *
 * Lifecycle:
 *   spawn at scene_init / 'r' / on resize for out-of-bounds slots.
 *   no destruction — the active count `Scene.n` defines the live slice.
 */
typedef struct {
    float px, py;
    float prev_px, prev_py;
    float vx, vy;
    int   color;
    char  ch;
} Star;
```

5–12 lines per main struct. Cover: what it represents, fields with units, lifecycle.

#### Tier 3 above each DRIVER function

Repeat the pseudocode from #2.5 above the function, plus inputs / outputs / units / why-it-exists. Deliberately repeated so the reader doesn't scroll back to the top.

```c
/*
 * scene_tick — advance every active star by dt.
 *
 *   if paused:  return
 *   max_px ← pw(cols) - 1
 *   max_py ← ph(rows) - 1
 *   for i = 0..n-1:  star_tick(stars[i], dt, max_px, max_py)
 *
 * Stars do NOT see each other here — that's a render-time concern, handled
 * in scene_draw_connections via the pair scan.  Tick is pure physics;
 * encapsulating it here keeps main()'s loop at orchestrator level.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows) { ... }
```

#### Tier 1 above ALGORITHMIC HELPERS only

The functions that drivers call. One short line above each:

```c
/* compute_lerp_positions — fill dpx/dpy/dcx/dcy caches for n active stars. */
static void compute_lerp_positions(...) { ... }

/* pick_connect_style — three brightness buckets from ratio ∈ [0, 1). */
static void pick_connect_style(...) { ... }
```

#### NOTHING above plumbing functions

Signal handlers, `cleanup`, `screen_init` / `screen_free` / `screen_resize`, `clock_ns`, `clock_sleep_ns`, `app_install_signals`, key dispatch — no comment block. The function name speaks for itself.

#### Inline switch-case comments

Where data-table switches exist (`generate_signal`, `generate_kernel`, `theme_apply`, `pattern_select`), each case gets 2–4 lines of inline comment explaining what variant + why:

```c
case SIG_IMPULSE: {
    /* Zero everywhere except a single 1 at the centre.
     * Convolving any kernel with this gives the kernel back —
     * the impulse-response identity. */
    for (int n = 0; n < N; n++) output_signal[n] = 0.0f;
    output_signal[N / 2] = 1.0f;
    break;
}
```

These are where most variant-specific teaching happens, after the driver Tier-3 blocks.

---

## What the finished file looks like

Reader experience top-to-bottom:

| Lines (approx) | What's there |
|---|---|
| 1–40 | ABSTRACT + SECTION MAP |
| 40–60 | CONCEPTS & ALGORITHMS (dense; one ref per concept inline) |
| 60–85 | OVERALL PSEUDOCODE (whole program in code form) |
| 85–115 | DRIVER PSEUDOCODE (per driver, 5–15 lines each) |
| 115–125 | KEYS + BUILD |
| 125–155 | REFERENCES (PAPERS + BOOKS, full citations) |
| 155+ | Code begins. §1 config, §2 clock, ... |
| ~430–630 | First driver function with Tier-3 block (pseudocode repeated for in-place reading) |
| Throughout | Tier-1 one-liners on algorithmic helpers; data-struct Tier-3 blocks above each main typedef; inline switch-case comments in variant generators; nothing on plumbing |

**Total prose budget for a 1200-line file: ~260 lines** (~22 % of file size).

vs NOVICE_REFACTOR on the same file: ~330 lines (~28 %).

## Length discipline

| Scope | Target |
|---|---|
| ABSTRACT | 3–5 sentences |
| CONCEPTS & ALGORITHMS | 5–10 entries, ≤ 25 lines |
| OVERALL PSEUDOCODE | 10–20 lines |
| DRIVER PSEUDOCODE (per driver) | 5–15 lines |
| REFERENCES (PAPERS + BOOKS) | 2–6 entries each, ≤ 30 lines total |
| Tier-3 above data struct | 5–12 lines per struct |
| Tier-3 above driver | 10–25 lines (pseudocode + why) |
| Tier-1 above algorithmic helper | 1 line |
| Above plumbing functions | none |
| Inline switch-case | 2–4 lines per case |

**File growth target: 1.10×–1.30× the production line count.**

## Acceptance test

A C programmer with general programming background but possibly NEW to the domain should, within **5 minutes** of reading the file top-to-bottom:

1. Name the algorithm and cite at least one reference for it.
2. Trace data flow through the OVERALL PSEUDOCODE.
3. Predict what `tick()` and `draw()` do from the DRIVER PSEUDOCODE alone.
4. Identify each main data structure and what it represents.
5. Locate the headline algorithmic helper functions (called by drivers).

If they can't, the failure path: **ABSTRACT → DRIVER PSEUDOCODE → Tier-3 driver blocks**. Most learnability problems trace to one of these three pieces being too thin.

If the reader needs MORE than this to learn the algorithm — analogies, worked examples, step-by-step construction — the file should be NOVICE_REFACTOR, not DEFAULT.
