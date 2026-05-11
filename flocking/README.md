# flocking — Reynolds boids, steering behaviours, agent swarms

A reference for the **collective-motion backbone** of the project. This
folder contains **7 self-contained C programs** that all answer the same
question:

> **How does a crowd of N independent agents organise itself without a
> central planner?**

Every file builds on Reynolds' 1987 boids — three local rules (separation,
alignment, cohesion) applied to every agent in isolation produce emergent
group behaviour. The folder spreads outward from that base: same-flock
**plus** a leader, **plus** a predator, **plus** a state machine, **plus** a
target shape, **plus** a chemical trail. Five rules become ten; one species
becomes two; and at no point does any agent see the global picture.

If you read **only one file**, read
[`flocking.c`](flocking.c) — five switchable algorithms (Reynolds boids,
leader chase, Vicsek, orbit, predator-prey) on the same Boid struct.
Everything else in the folder is a thematic extension of it.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive — per-agent local rules](#the-unifying-primitive--per-agent-local-rules)
3. [The five canonical forces](#the-five-canonical-forces)
4. [Force composition diagram](#force-composition-diagram)
5. [File index](#file-index)
6. [Building and running](#building-and-running)
7. [Adding a new flock](#adding-a-new-flock)
8. [Cross-folder pointers](#cross-folder-pointers)

---

## How to read this folder

```
   1. flocking.c              ← 5-mode primer: boids / chase / Vicsek / orbit / predator
              │
              ▼
   2. crowd.c                 ← 6 steering behaviours on a Person pool (single flock)
              │
              ▼
   3. murmuration.c           ← 800 starlings + density-field renderer + hawk predator
   3'. shepherd.c             ← Strömbom border collie herds sheep back to pen
              │
              ▼
   4. war.c                   ← two-faction battle: melee + archers + FSM per warrior
   4'. swarm_gen_numbers.c    ← swarm self-assembles into ASCII digits (10 strategies)
              │
              ▼
   5. slime_mold.c            ← Physarum: agents leave a chemical trail they then sense
```

**Prerequisites.** Read `flocking.c` first — it states the three Reynolds
rules and the toroidal-delta neighbour metric that the other six files
inherit. Then read `crowd.c` for the **steering-behaviours expansion** of
the same primitives (seek, flee, arrive, wander) on top of separation /
alignment / cohesion.

From there the folder forks:

* **More agents, richer rendering** → `murmuration.c` (density-field
  glyph ramp at 800 birds).
* **Higher-level controllers on top of boids** → `shepherd.c` (Strömbom
  collect/patrol), `war.c` (per-warrior FSM), `swarm_gen_numbers.c`
  (greedy slot assignment).
* **A different sensing primitive entirely** → `slime_mold.c` (stigmergic
  trail rather than peer-position polling).

---

## The unifying primitive — per-agent local rules

Every file in this folder is built around the same agent struct:

```c
typedef struct {
    Vec2  pos, prev_pos, vel;   /* pixel-space position, last position, velocity */
    int   color, glyph;         /* per-agent appearance */
    /* + per-file extras: state, faction, target_idx, HP, etc. */
} Agent;

Agent pool[N_MAX];              /* fixed-capacity pool, only first `count` active */
```

The tick is **two-staged** in every file:

```
# stage 1 — read everyone's current state, compute everyone's NEW velocity
for each agent a:
    force = weighted sum of (separation, alignment, cohesion, seek, flee, ...)
    new_vel[a] = clamp_speed(a.vel + force · dt)

# stage 2 — apply all the new velocities and integrate position
for each agent a:
    a.prev_pos = a.pos
    a.vel      = new_vel[a]
    a.pos     += a.vel · dt
    wrap_or_bounce(a.pos)
```

The two-stage split matters: stage 1 is read-only on the agent array so no
agent reacts to a neighbour that has already moved this tick. Stage 2 is
write-only. This is the **one rule** that prevents the simulation from
becoming order-dependent.

**Why O(N²) is fine.** Every file polls every pair of agents — N(N-1)/2
distance checks per tick. At N ≤ 150 that's < 11 000 ops at 60 Hz, well
below the noise floor of one ncurses redraw. `murmuration.c` (N = 800)
keeps the same O(N²) loop and still hits 60 fps because the inner body is
pure float math. No spatial hash is needed for terminal-sized worlds.

---

## The five canonical forces

| Force         | Definition                                                                                                | Used in (all 7 unless noted) |
|---------------|-----------------------------------------------------------------------------------------------------------|------------------------------|
| **Separation**| For each neighbour within `R_sep`, sum the unit vector *away* from the neighbour, scaled by `1/distance`. | every flock file             |
| **Alignment** | Sum velocities of neighbours within `R_align`; steer toward the mean direction.                          | every flock file             |
| **Cohesion**  | Compute the centroid of neighbours within `R_coh`; steer toward it.                                       | every flock file             |
| **Seek**      | Unit vector from `pos` to `target`, scaled to `max_speed`; subtract current velocity.                    | `crowd.c`, `flocking.c`, `swarm_gen_numbers.c`, `shepherd.c`, `war.c` |
| **Flee**      | Negated Seek, with a falloff radius beyond which the force is zero.                                       | `crowd.c`, `flocking.c`, `shepherd.c`, `murmuration.c` (hawk), `war.c` |

The whole intelligence of every agent is a **weighted sum** of those five
vectors. No rule hierarchy, no finite-state machine at the boid level (the
FSM lives one level up, in `war.c` and `shepherd.c`). Different per-agent
behaviours are different *weight vectors* applied to the same five primitives.

---

## Force composition diagram

```
                ┌────────────────────────────────────────────┐
       agent a  │  neighbours within R_sep, R_align, R_coh    │
                └──────┬──────────┬───────────┬───────────────┘
                       │          │           │
                       ▼          ▼           ▼
                 ┌──────────┐ ┌─────────┐ ┌──────────┐
                 │ SEPARATE │ │  ALIGN  │ │  COHERE  │      ┌──────────┐
                 │   1/d    │ │   v̄    │ │ centroid │      │   SEEK   │
                 └────┬─────┘ └────┬────┘ └─────┬────┘      │  target  │
                      │            │            │           └────┬─────┘
                      │ ×w_sep     │ ×w_align   │ ×w_coh         │ ×w_seek
                      └──────┬─────┴─────┬──────┘                │
                             │           │                       │
                             ▼           ▼                       ▼
                          ┌─────────────────────────────────────────┐
                          │   force = Σ weight · force_vector       │
                          └────────────────────┬────────────────────┘
                                               │
                                               ▼
                                  Euler integrate (vel, pos)
                                               │
                                               ▼
                                    wrap toroidally / bounce
```

The diagram is **the entire architecture** of every file in this folder. A
new behaviour is just a new column in the sum.

---

## File index

| File                       | Description                                                                       | Forces in play                            | What it adds vs. previous |
|----------------------------|-----------------------------------------------------------------------------------|-------------------------------------------|----------------------------|
| `flocking.c`               | Five switchable algorithms across three flocks sharing the screen                 | sep + align + coh + leader-seek + flee    | base: 1-5 to switch boids / chase / Vicsek / orbit / predator |
| `crowd.c`                  | Six switchable steering-behaviour crowds (wander / flock / panic / gather / follow / queue) | sep + align + coh + seek + flee + arrive  | adds *seek* / *arrive* / *wander* on a single-flock pool |
| `murmuration.c`            | 800-bird starling flock rendered as a 2-D density field with diving hawk          | sep + align + coh + hawk-flee             | density-glyph ramp `.,:;oO*#@` + PATROL/DIVE hawk FSM |
| `shepherd.c`               | Autonomous border collie herds scattered sheep back into a circular pen           | sep + coh + dog-flee + pen-inward         | Strömbom collect/patrol controller — emergent herding |
| `war.c`                    | Two-faction battle (Gondor vs Mordor) with melee, archers, real arrow projectiles | sep + align + faction-seek + flee         | per-warrior 4-state FSM (ADVANCE/COMBAT/FLEE/DEAD) + arrow pool |
| `swarm_gen_numbers.c`      | 120-agent swarm self-organises into ASCII digits 0-9 with 10 strategies           | sep + align + coh + slot-arrive + spring  | greedy slot assignment + 10 strategy presets |
| `slime_mold.c`             | Jeff Jones Physarum model: agents sense + deposit on a diffusing trail grid       | sense-rotate (no peer polling)            | replaces peer polling with **stigmergic trail** — agents communicate via the grid |

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra <path>.c -o <name> -lncurses -lm
```

**Universal keys** (present in every file):

| Key             | Action                              |
|-----------------|-------------------------------------|
| `q` / `ESC`     | quit                                |
| `space`         | pause / resume                      |
| `r` / `R`       | reset positions                     |
| `+` / `-`       | add / remove agents                 |
| `t` / `T`       | next / previous theme (most files)  |

**Behaviour-switching keys** (per file):

| File                  | Keys                                                  |
|-----------------------|-------------------------------------------------------|
| `flocking.c`          | `1`-`5` switch algorithm; `n/m` Vicsek noise          |
| `crowd.c`             | `1`-`6` wander / flock / panic / gather / follow / queue |
| `murmuration.c`       | `space` hawk dive; `h` auto-dive toggle               |
| `shepherd.c`          | `space` scatter; `S` mega-scatter; `c` chaos          |
| `war.c`               | `1`-`6` switch strategy; `g/m` add Gondor / Mordor    |
| `swarm_gen_numbers.c` | `0`-`9` form digit; `n/p` strategy; `a` auto-cycle    |
| `slime_mold.c`        | `n/N` preset; `d/D` diffusion; `e/E` decay; `f` food  |

---

## Adding a new flock

1. **State the local rule.** What does one agent see, and what one
   summed-vector force does it produce? If the rule is *"average heading
   of neighbours within R + noise"*, that's Vicsek. If it's *"flee any
   threat within R, otherwise wander"*, that's panic-mode crowd.
2. **Copy the closest existing file** as a template:
   * Multiple algorithms on one screen → `flocking.c`
   * Single behaviour, single flock → `crowd.c`
   * Predator + prey + density rendering → `murmuration.c`
   * Controller + flock (one master, N followers) → `shepherd.c`
   * Per-agent FSM on top of steering → `war.c`
   * Target-shape assembly → `swarm_gen_numbers.c`
   * No peer polling, only environment → `slime_mold.c`
3. **Replace one function — the per-agent `force()` computation.**
   Everything else (clock, color, scene, screen, app, signals) carries
   over unchanged. The two-stage tick stays.
4. **Pick a topology.** Toroidal wrap (`flocking.c`, `murmuration.c`) for
   uniform behaviour everywhere; bounded box (`crowd.c`, `war.c`) for
   stage-like containment; soft pen attractor (`shepherd.c`) for partial
   containment.
5. **Add CONCEPTS + MENTAL MODEL blocks** per the project's
   [CLAUDE.md](../CLAUDE.md) template.
6. **Verify**: `gcc -Wall -Wextra` clean, stable 60 fps even at N_MAX,
   `q` / `ESC` exits cleanly, `SIGWINCH` doesn't crash, HUD shows fps +
   active behaviour.

---

## Cross-folder pointers

* **[`animation/`](../animation/)** integrates **link constraints** per
  tick (FK / IK / Verlet); this folder integrates **social-force vectors**
  per tick. Both use the same fixed-timestep accumulator and the same
  pixel-space `Vec2` math — only the per-tick force computation differs.
  The two-stage tick pattern here is identical to the prev/cur snapshot
  pattern in `ragdoll_figure.c`.
* **[`robots/`](../robots/)** are *individual* agents with internal
  controllers (PID, state machine, kinematic chain). A flocking agent has
  five lines of force math; a robot has hundreds of lines of body
  kinematics — but both share the same outer loop and the same pixel-space
  integrator.
* **[`grids/`](../grids/)** is the discrete-coordinate counterpart; this
  folder lives in continuous pixel space because flocking forces need
  isotropic distance metrics. Only `slime_mold.c` straddles both — its
  agents live in continuous space but read/write a discrete trail grid.
