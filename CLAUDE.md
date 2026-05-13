# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## Modes & Phases

A file in this codebase passes through three phases. Each activates a different mode with different rules — when rules appear to contradict, the active mode wins.

| Phase / Mode | Trigger | Goal | Rules dominate |
|---|---|---|---|
| **Phase 1 — Author** (new file) | "write a new X", "add a demo for Y" | working production version, validated by clean compile + visual inspection. 250-450 lines. | `documentation/Authoring.md`; *New Simulation Workflow* below |
| **Phase 2 — Iterate** (surgical edit) | bug fix, "change X to Y", visual symptom report | minimum diff that solves the request | §3 *Surgical Changes* |
| **Phase 3a — DEFAULT_REFACTOR** (default for ~80 % of files) | `DEFAULT_REFACTOR` / `/default-refactor <file>`, or *refactor* on any file where the reader is a competent programmer (knows C, may not know the domain) | concept density + pseudocode-first; Tier-3 on data structures + drivers; light everywhere else | `default-refactor` skill (`.claude/skills/default-refactor/SKILL.md`); overrides "match existing style" and the 250-450 target |
| **Phase 3b — NOVICE_REFACTOR** (textbook for true beginners) | `NOVICE_REFACTOR` / `COMPLETE_LITERATE` / `LITERATE` (legacy aliases) / `/novice-refactor <file>`, or *refactor* on a CANONICAL algorithm reference where the reader has no domain background | embedded textbook with analogies, worked examples, GUIDED TUTORIAL with construction-sequence lessons | `novice-refactor` skill (`.claude/skills/novice-refactor/SKILL.md`); reserve for the 5–10 canonical references |
| **Phase 3c — COMMENT_REFACTOR** (comment-only) | `COMMENT_REFACTOR` / `UPDATE_LITERATE` (legacy alias) / `/comment-refactor <file>`, "redo the comments on X" | rewrite the comment layer from zero against a code-only inventory; defaults to DEFAULT profile, pass `novice` arg to override; code untouched | `comment-refactor` skill (`.claude/skills/comment-refactor/SKILL.md`) |

When unsure which mode applies, name it explicitly: *"This is a Surgical edit, so I'll only touch X."*

A surgical edit (bug fix, theme add) does NOT move a file between phases — a phase-3 textbook can take a phase-2 fix without losing textbook status.

### Phase 1 — Author

**Produce:** file header, CONCEPTS, MENTAL MODEL, §1..§N code following framework.c, HUD. Themes only if requested. Full templates and examples in `documentation/Authoring.md`.

**Do NOT:** add HOW TO READ THIS FILE, GUIDED TUTORIAL, debug overlays, long-name expansion, or per-function teaching blocks. Pedagogy is phase 3's job.

### Phase 2 — Iterate

User runs the program, reports what they see; converge via surgical edits. One concern per turn. No restructuring beyond the request, no "while we're here".

**End** = explicit user approval ("looks good" / "ship it"). The file is then **validated**.

### Phase 3a — DEFAULT_REFACTOR (the default for ~80 % of files)

Trigger words `DEFAULT_REFACTOR` / `/default-refactor <filename>` auto-invoke the `default-refactor` skill. Same 3-step structure as the others (Step 0 themes/HUD, Step 1 pseudocode-shaped code, Step 2 prose layer), but Step 2 is built for a **competent learner** (knows C, knows general programming, may not know the domain). Step 2 produces SEVEN pieces in fixed order: ABSTRACT (3-5 sentences), SECTION MAP, CONCEPTS & ALGORITHMS (dense bullet list with refs), OVERALL PSEUDOCODE (whole program in 10-20 lines), DRIVER PSEUDOCODE (per driver, top-of-file), KEYS+BUILD, plus inline Tier-3 above each main DATA STRUCTURE and DRIVER function. Tier-1 one-liners only on algorithmic helpers; nothing on plumbing. **Concept density + pseudocode-first** instead of analogies and worked examples.

### Phase 3b — NOVICE_REFACTOR (textbook for true beginners)

Trigger words `NOVICE_REFACTOR` / `COMPLETE_LITERATE` / `LITERATE` (legacy aliases) / `/novice-refactor <filename>` auto-invoke the `novice-refactor` skill. The **textbook** treatment: Step 2 produces file header + CONCEPTS + MENTAL MODEL (with worked example + ASCII diagram) + GUIDED TUTORIAL (4–6 PROBLEM/FIX/WHY+WHERE lessons) + Tier-1 function labels. Reserve for the **5–10 canonical algorithm references** in the project where a true beginner with no domain background needs to learn from zero.

### Phase 3c — COMMENT_REFACTOR (comment-only refresh)

Trigger word `COMMENT_REFACTOR` (or legacy `UPDATE_LITERATE` / `/comment-refactor <filename>`) auto-invokes the `comment-refactor` skill. Comment-only sibling of both DEFAULT and NOVICE — assumes Step 0 and Step 1 are already done; rewrites only the prose layer. **Defaults to DEFAULT_REFACTOR profile silently**; pass `novice` as an arg (`/comment-refactor novice <file>`) to target NOVICE instead.

### Phase violations

- DEFAULT_REFACTOR / NOVICE_REFACTOR requested before phase 2 ended → ask whether the algorithm is validated. Pedagogy on a buggy algorithm wastes effort.
- COMMENT_REFACTOR requested on a file whose code is NOT pseudocode-shaped (no helpers, no named locals, no layer functions) → push back: run `default-refactor` (or `novice-refactor`) Step 1 first, then COMMENT_REFACTOR.
- "Phase 1 with phase 3 quality" → warn that the textbook may need rewriting after phase 2 finds bugs; offer minimal phase 1 first.
- Old file the user wrote alone, never validated → treat as phase-2-pending.
- NOVICE requested on a non-canonical file (variant, polish demo) → suggest DEFAULT instead; CONFIRM the user really wants the textbook treatment before committing the prose budget.

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

# Phase 1 Authoring — Rule Summary

Full templates and examples: **`documentation/Authoring.md`**. Read it before authoring a new file. One-line summary of each rule:

- **File header** — `<file> — <one-line>`, DEMO, Study alongside, Section map §1..§N, Keys, Build command. Mandatory.
- **CONCEPTS block** — Algorithm / Data-structure / Rendering / Performance / References (2-5). Mandatory.
- **MENTAL MODEL block** — CORE IDEA, HOW TO THINK ABOUT IT, ALGORITHM IN STEPS, KEY FORMULAS, EDGE CASES TO WATCH, HOW TO VERIFY. Six fixed sub-headings. Mandatory.
- **No magic numbers** — every meaningful literal goes in §1 config with units. Exempt: `0 1 -1 2 0.5f`, `2.0f * M_PI` inline, loop bounds, index arithmetic.
- **Function comments — WHY, not WHAT** — one-line purpose + why this approach + cross-ref if any.
- **§5 entity naming** — name struct + functions after the concept (`Boid`/`boid_tick`), not generics (`Entity`/`entity_update`). Fields commented with units.
- **Learner checkpoints** — at each §-boundary, point to the next conceptual section.
- **One concept per function** — `_and_` in a name = split it.
- **Pure vs mutating** — pure helpers `const`-return; mutating take non-const pointer. Signature reveals state-change.
- **Mirror math in code** — N steps in KEY FORMULAS = N lines in the body, in order.
- **Linear flow** — guard clauses first, main work after. Three nesting levels = split or early-return.
- **Function length** — ≤30 lines target, ≤60 for orchestrators, past 60 = doing more than one thing.
- **Group struct fields** — 15 ungrouped = a cliff; group with one-line headers.
- **Name for the concept** — `fitness` not `f`, `n_pop` not `n`. Single letters OK only in tight loops.
- **No cross-§ reaches** — §5 entity must not call §7 ncurses. Cross layers via parameters, not globals.
- **Cleverness needs one-line justification** — if you can't, write the slow obvious way.

Acceptance: a competent C programmer should grok the algorithm in 10 minutes from top-to-bottom reading. Failure path: prose → structure → code.

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

3. **Estimate scope** — typical phase-1 file sits at 250-450 lines. State if longer and why. Approaching 600+ = §5 is doing too much; split. (Phase 3 pedagogical refactors grow the file: NOVICE_REFACTOR 1.20×–1.35×, DEFAULT_REFACTOR 1.10×–1.25×.)
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

Canonical for ALL demos, including DEFAULT_REFACTOR / NOVICE_REFACTOR-treated files. Two fixed UI elements, **bright high-contrast colours + `A_BOLD`, never `A_DIM`** so they remain legible against any animation.

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

Canonical for ALL theme work, including DEFAULT_REFACTOR / NOVICE_REFACTOR Step 0. Every theme entry must sit in the **bright half** of the 256-colour space — even the "darkest" tier. Bottom-of-palette colours become invisible against default-black with `A_DIM`.

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
| `documentation/Authoring.md` | Phase-1 file templates: header, CONCEPTS, MENTAL MODEL, code structure rules |
| `documentation/Architecture.md` | Framework design, loop mechanics, coordinate model, per-subsystem deep dives |
| `documentation/Master.md` | Long-form essays on algorithms, physics, visual techniques |
| `documentation/Visual.md` | ncurses field guide — V1–V9, Quick-Reference Matrix, Technique Index |
| `documentation/COLOR.md` | Color tricks — palettes, escape-time/density coloring, 256-color patterns |
| `.claude/skills/default-refactor/SKILL.md` | DEFAULT_REFACTOR 3-step doctrine — competent-learner standard; the default for ~80 % of files (auto-invoked by trigger word) |
| `.claude/skills/novice-refactor/SKILL.md` | NOVICE_REFACTOR 3-step doctrine — full textbook for true beginners; reserve for the 5–10 canonical algorithm references (auto-invoked by trigger word) |
| `.claude/skills/comment-refactor/SKILL.md` | COMMENT_REFACTOR comment-only procedure; defaults to DEFAULT profile, pass `novice` arg to override (auto-invoked by trigger word) |
