# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

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
 *   §4 coords   — pixel↔cell conversion  (omit if cell-space sim)
 *   §5 <entity> — simulation state + tick logic
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
 * References     : <2-5 links or citations — Wikipedia, papers, books.
 *                   A learner reading this file should know where to go
 *                   next to understand the math more deeply.>
 *
 * ─────────────────────────────────────────────────────────────────────── */
```

References are not optional. Every CONCEPTS block must have at least two.
Examples of good references:
- Wikipedia article on the algorithm
- Original paper (Reynolds 1987 for boids, Stam 1999 for stable fluids, etc.)
- A textbook chapter (e.g. "Game Physics Engine Development — Millington §12")
- A web resource (e.g. Inigo Quilez SDF functions, Red Blob Games pathfinding)

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

**Step 3 — Estimate scope.** A typical file is 250–450 lines. State if it will be longer and why. If it approaches 600+, the §5 physics is probably doing too much — split into sub-functions.

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

Every program has two fixed UI elements. Follow this layout exactly:

```c
/* §7 screen_draw() — top-right: fps + params + state */
char buf[64];
snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ",
         fps, sim_fps, paused ? "PAUSED " : "running");
attron(COLOR_PAIR(3) | A_BOLD);
mvprintw(0, cols - (int)strlen(buf), "%s", buf);
attroff(COLOR_PAIR(3) | A_BOLD);

/* bottom-left: one-line key hint strip */
attron(COLOR_PAIR(6) | A_DIM);
mvprintw(rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:<param> ");
attroff(COLOR_PAIR(6) | A_DIM);
```

The key hint at the bottom must list every interactive key the program uses.

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

### Section Layout (every animation file)
- `§1 config` — all `#define` / enum constants, `TICK_NS`, `CELL_W/H`
- `§2 clock` — `clock_ns()` + `clock_sleep_ns()` (`CLOCK_MONOTONIC`)
- `§3 color` — `color_init()`, 256-color with 8-color fallback
- `§4 coords` — `pw/ph/px_to_cell_x/y` (omitted in cell-space sims)
- `§5 physics` — simulation struct + tick (no ncurses here)
- `§6 scene` — owns physics objects; `scene_tick` + `scene_draw(alpha)`
- `§7 screen` — `screen_init/draw/present/resize`
- `§8 app` — `App` struct, signal flags, main loop

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

## Coding Conventions
- Comments explain **why** — non-obvious physics, rounding choices, workarounds only
- Every tunable constant in `§1 config` as `#define` or enum
- No dynamic allocation after init (except initial `malloc` in tessellate/flowfield/sand)
- `(chtype)(unsigned char)ch` double cast on every `mvaddch` — prevents sign-extension corruption
- `sig_atomic_t` for all signal-written flags
- C11, `-Wall -Wextra` clean

---

## Documentation

| File | Contents |
|---|---|
| `documentation/Architecture.md` | Framework design, loop mechanics, coordinate model, per-subsystem deep dives |
| `documentation/Master.md` | Long-form essays on algorithms, physics, and visual techniques |
| `documentation/Visual.md` | ncurses field guide — V1–V9, Quick-Reference Matrix, Technique Index |
| `documentation/COLOR.md` | Color tricks — palettes, escape-time coloring, density coloring, 256-color patterns |
