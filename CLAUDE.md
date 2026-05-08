# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## Modes & Phases

A file in this codebase passes through three phases. Each activates a different mode with different rules — when rules appear to contradict, the active mode wins.

| Phase / Mode | Trigger | Goal | Rules dominate |
|---|---|---|---|
| **Phase 1 — Author** (new file) | "write a new X", "add a demo for Y" | working production version, validated by clean compile + visual inspection. 250-450 lines. | *Learner-Friendly Code Standards / Structure*; *New Simulation Workflow* |
| **Phase 2 — Iterate** (surgical edit) | bug fix, "change X to Y", visual symptom report | minimum diff that solves the request | §3 *Surgical Changes* |
| **Phase 3 — Pedagogical refactor** | any phrase combining *refactor/rewrite* with *learn/teach/pedagogy/first principles* | turn validated file into embedded textbook. 1.5×-2× growth. | *Pedagogical Refactor Recipe*; overrides "match existing style" and the 250-450 target |

When unsure which mode applies, name it explicitly: *"This is a Surgical edit, so I'll only touch X."*

A surgical edit (bug fix, theme add) does NOT move a file between phases — a phase-3 textbook can take a phase-2 fix without losing textbook status.

### Phase 1 — Author

**Produce:** file header, CONCEPTS, MENTAL MODEL, §1..§N code following framework.c, HUD. Themes only if requested.

**Do NOT:** add HOW TO READ THIS FILE, GUIDED TUTORIAL, debug overlays, long-name expansion, or per-function teaching blocks. Pedagogy is phase 3's job.

### Phase 2 — Iterate

The user runs the program, reports what they see; converge via surgical edits. One concern per turn. No restructuring beyond the request, no "while we're here".

**End** = explicit user approval ("looks good" / "ship it"). The file is then **validated**.

### Phase 3 — Pedagogical refactor

Add (on top of phase-1 file): HOW TO READ THIS FILE block, GUIDED TUTORIAL, per-function teaching blocks, debug overlays (`d`/`D`), long-name expansion, inline pedagogy. Phase 3 works **because** phase 2 has validated the algorithm.

### Phase violations

- Phase 3 requested before phase 2 ended → ask whether the algorithm is validated. Pedagogy on a buggy algorithm wastes effort.
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

# Pedagogical Refactor Recipe

Triggered by phrases combining *refactor/rewrite* with *learn/teach/pedagogy/first principles*. NOT a production cleanup — a deep rewrite that turns the file into an embedded textbook.

## Mindset

- Clarity over cleverness.
- Intuition over performance.
- Explicitness over compactness.
- Mental models over abstraction layers.

Rebuild from first principles. Don't tidy existing code — reconstruct as if explaining to a beginner who knows basic C but nothing about the domain.

## Output structure (in order)

1. **File header** (per the standard above).
2. **HOW TO READ THIS FILE** — 15-25 lines: reading order, long-name convention, required background.
3. **CONCEPTS block** (5 subsections, ≥2 references).
4. **MENTAL MODEL block** (all 6 subheadings, ≥1 ASCII diagram).
5. **GUIDED TUTORIAL** — 6-12 numbered mini-tutorials building the algorithm from first principles. Each: opens with a question, plain English first, ASCII diagram or worked example, ends with simplified pseudocode. Cover (in order): core problem, data layout, each transformation step, coordinate-system bridges, render/sim interface.
6. **§1..§N actual code**, broken into many small sections (≥15 for non-trivial files), each ≤~100 lines, one concept each.

## For every section

Open with educational preamble: problem being solved, why this logic exists, assumptions, inputs, outputs. The preamble is the reader's running orientation.

## For every function

Comment block above signature: **Purpose** (one sentence), **Pseudocode** mirroring the body 1:1, **Mental model** (analogy), **Inputs/outputs/units** (with coordinate systems), **Why it exists** (what would break if merged with caller). Body mirrors pseudocode line-for-line with inline concept names. Same length discipline (≤30 target, ≤60 for orchestrators).

## Variable naming — long descriptive names everywhere

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

Forbidden: vague names, hidden state, magic numbers, compressed math, premature optimisation, generic abstractions, expert shorthand.

## Inline pedagogy + ASCII diagrams

Where non-obvious concepts appear (geometry, physics, rendering pipelines, interpolation, coordinate systems, state transitions, numerical approximation, memory layout, signal flow), pause and **teach** inline.

ASCII diagrams required for: struct memory layout, coordinate transformations, grid stencils (Laplacian, advection backward-trace), pipeline stages, ray paths through volumes, time-step state transitions, mesh winding, interpolation weighting.

## Educational debug helpers

Optional overlays toggled by `d`/`D`. Each visualises one piece of intermediate state. Examples: fluid → density/temperature/velocity-arrows/pressure/divergence; raymarcher → depth/normal/step-count/curvature/orbit-trap; fractal → iteration-count/escape-ratio; rasteriser → per-stage buffers (z, normal, AO, light); CA → rule-rate/density/age-since-flip. The debug helper's source IS part of the lesson.

## Expected outcome + acceptance test

File length 1.5×-2× the production version — growth is comments, tutorials, debug helpers, never the algorithm itself.

A C programmer new to the technique should, within ten minutes of top-to-bottom reading: name the algorithm, sketch the data flow, predict each function from signature alone, identify which knob in §1 controls which visual effect. If they can't — more prose, more diagrams, more pseudocode. Not more code.

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
