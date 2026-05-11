# animation — articulated motion: FK, IK, Verlet ragdolls, gaits

A reference for the **articulated-body backbone** of the project. This folder
contains **14 self-contained C programs** that all answer one question in
two flavours:

> **Where do the joints of a chain go this frame?**
>
> * given **angles** (forward kinematics — push joints outward from a fixed root)
> * given a **target** (inverse kinematics — bend the chain so the tip lands there)
> * given **forces** (Verlet integration + distance constraints — let physics place them)

Every file uses **one chain primitive** — an ordered list of joint positions
in pixel space — and differs only in how those positions are recomputed each
tick. Once you can read a 16-segment FABRIK tentacle, an 8-bone Verlet
ragdoll is the same data structure with constraints replacing the IK solve.

If you read **only one file**, read
[`fk_helloworld.c`](fk_helloworld.c) — two links, two angles, twelve lines
of math. Everything else in the folder is downstream of it.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive — a joint chain](#the-unifying-primitive--a-joint-chain)
3. [Three engines, one chain](#three-engines-one-chain)
   * [Engine A — Forward Kinematics (angles → positions)](#engine-a--forward-kinematics-angles--positions)
   * [Engine B — Inverse Kinematics (target → angles)](#engine-b--inverse-kinematics-target--angles)
   * [Engine C — Verlet + constraints (forces → positions)](#engine-c--verlet--constraints-forces--positions)
4. [File index](#file-index)
5. [Building and running](#building-and-running)
6. [Adding a new chain](#adding-a-new-chain)
7. [Cross-folder pointers](#cross-folder-pointers)

---

## How to read this folder

```
   1. fk_helloworld.c ─────────► ik_helloworld.c    ← 2-link textbook pair
              │                          │
              ▼                          ▼
   2. fk_ik_helloworld.c          (both modes, 3-link with wrist constraint)
              │
              ▼
   3. fk_tentacle_forest.c        ← stateless FK on a stack of chains
              │
              ▼
   4. snake_forward_kinematics.c  ← trail-buffer FK (path-following body)
   4'. snake_inverse_kinematics.c ← FABRIK head + trail-buffer body
              │
              ▼
   5. ik_arm_reach.c              ← FABRIK on a 4-link arm
   5'. ik_tentacle_seek.c         ← FABRIK + per-joint angle constraints
              │
              ▼
   6. ik_spider.c                 ← body FK + 6 legs of 2-joint IK + gait
   6'. ik_scorpin.c               ← spider + curving FK tail
   6''. hexpod_tripod.c           ← rigid-body chassis + tripod gait
              │
              ▼
   7. fk_centipede.c              ← 24-segment FK body + stateless leg gait
              │
              ▼
   8. ragdoll_figure.c            ← Verlet humanoid + slanted platforms
   8'. ragdoll_ropes.c            ← seven Verlet ropes in wind
```

**Prerequisites.** Each file's header carries *Study alongside* pointers —
follow them in either direction. The pedagogical pairs are deliberate:

* `fk_helloworld.c` ↔ `ik_helloworld.c` — same 2-link geometry, **opposite
  input/output flow**. Read them as a pair before anything else.
* `snake_forward_kinematics.c` ↔ `snake_inverse_kinematics.c` — same 32-
  segment body, sinusoidal-wander vs. FABRIK head.
* `ik_arm_reach.c` ↔ `ik_tentacle_seek.c` — FABRIK without vs. with
  per-joint angle clamps.
* `ik_spider.c` ↔ `hexpod_tripod.c` — trail-buffer body vs. rigid chassis,
  same 2-joint analytical IK in both leg models.
* `ragdoll_figure.c` ↔ `ragdoll_ropes.c` — same Verlet engine, branched
  skeleton vs. linear chains.

---

## The unifying primitive — a joint chain

Every file in this folder centres on the same abstraction:

```c
typedef struct {
    float x, y;     /* pixel-space position */
} Vec2;

Vec2 joint[N_JOINTS];   /* ordered, joint[0] is root, joint[N-1] is tip */
```

The chain may be a 2-link arm, a 24-segment centipede body, a 4-link FABRIK
arm, a Verlet rope, or a 15-particle humanoid — the data layout is the
same. What changes between files is **how `joint[]` is recomputed each
tick.** That is the single axis of variation across all 14 files.

A second array often sits alongside `joint[]`:

```c
Vec2 prev_joint[N_JOINTS];    /* last frame's positions */
float link_len[N_JOINTS - 1]; /* fixed distance between consecutive joints */
```

`prev_joint[]` either drives Verlet integration (`pos − prev_pos` IS the
velocity) or feeds sub-tick alpha lerp for smooth rendering. `link_len[]`
encodes the **kinematic constraint**: every solver in this folder works to
preserve these distances.

The whole folder is then a study of **three engines** that all produce the
same `joint[]` output from different inputs.

---

## Three engines, one chain

```
                ┌──────────────────────────────────────────────────┐
                │  joint[N_JOINTS]   — the universal output         │
                └─────────▲──────────────▲──────────────▲──────────┘
                          │              │              │
                  ENGINE A          ENGINE B        ENGINE C
                 forward kin       inverse kin     Verlet + constraints
                  (angles)          (target)         (forces)
                          │              │              │
              ┌───────────┴──┐    ┌──────┴──────┐  ┌────┴────────┐
              │ θ₁..θₙ        │   │ target_pos  │  │ pos, old_pos │
              │ link lengths  │   │ link lengths│  │ link lengths │
              └───────────────┘   └─────────────┘  └─────────────┘
```

### Engine A — Forward Kinematics (angles → positions)

Push joints outward from a fixed root by accumulating per-segment rotations:

```
cumulative = root_angle
joint[0]   = root
for i in 0 .. N-1:
    cumulative += delta_theta[i]
    joint[i+1]  = joint[i] + link_len[i] * (cos cumulative, sin cumulative)
```

Closed-form. One unique answer for every input. No reachability check, no
ambiguity. **FK is the easy direction** — every IK solver in this folder is
essentially asking *"what FK input produced this output?"*

Three FK flavours appear in the folder:

| Flavour                  | Where it lives                                       | What drives the angle |
|--------------------------|------------------------------------------------------|-----------------------|
| **User-driven**          | `fk_helloworld.c`, `fk_ik_helloworld.c` (FK mode)    | arrow keys            |
| **Stateless sinusoidal** | `fk_tentacle_forest.c`, `fk_centipede.c` (legs)      | `sin(ω·t + phase)`    |
| **Trail-buffer**         | `snake_forward_kinematics.c`, `fk_centipede.c` (body), `ik_spider.c` (body), `ik_scorpin.c` (body) | the head's own path history |

Trail-buffer FK is the project's signature trick: store the head's past
pixel positions in a circular `trail[TRAIL_CAP]` ring buffer, then place
each body joint by walking that buffer until accumulated arc length equals
`i * SEG_LEN`. The body follows the exact path the head carved — no
per-segment angle math at all.

### Engine B — Inverse Kinematics (target → angles)

Given the desired **tip position**, back-solve where the joints must go.
Two variants appear:

**B1 — Analytical 2-link (law of cosines).** Used wherever a leg or arm
has exactly two links:

```
d  = |target − shoulder|
α  = acos( (d² + L₁² − L₂²) / (2·d·L₁) )      ← angle at shoulder
φ  = atan2(target.y − shoulder.y, target.x − shoulder.x)
elbow = shoulder + L₁ · (cos(φ ± α), sin(φ ± α))    ← ± picks elbow side
```

One sqrt, one acos, one atan2 — O(1). **Two valid solutions** for every
reachable target (elbow-up / elbow-down) — that two-solution ambiguity is
the conceptual heart of IK. Used in: `ik_helloworld.c`, `fk_ik_helloworld.c`
(IK mode), `ik_spider.c`, `ik_scorpin.c`, `hexpod_tripod.c`.

**B2 — FABRIK (Forward And Backward Reaching IK).** For chains longer than
two links, the analytical approach fails. FABRIK iterates two geometric
passes:

```
repeat until |tip − target| < EPSILON:
    FORWARD  pass:  snap tip to target;  walk root-ward re-stretching links
    BACKWARD pass:  snap root to anchor; walk tip-ward re-stretching links
```

No matrix inverse, no Jacobian, no singularities. Converges in 3–5
iterations for a 4-link arm. Used in: `ik_arm_reach.c`,
`ik_tentacle_seek.c` (with per-joint angle clamps),
`snake_inverse_kinematics.c`.

### Engine C — Verlet + constraints (forces → positions)

Position-Verlet integration: each particle stores `pos` and `old_pos`;
velocity is implicit in the difference. Gravity and wind accelerate;
distance-constraint projection (Jakobsen) restores rigid-link lengths after
integration:

```
# integrate
for each particle p:
    accel    = gravity + wind
    new_pos  = 2·p.pos − p.old_pos + accel · dt²
    p.old_pos = p.pos
    p.pos     = new_pos

# project — repeat 6..8 times
for each constraint (a, b, rest_len):
    delta = b.pos − a.pos
    diff  = (|delta| − rest_len) / |delta|
    a.pos += 0.5 · diff · delta
    b.pos -= 0.5 · diff · delta
```

The two-file Verlet pair (`ragdoll_figure.c`, `ragdoll_ropes.c`) is the
project's bridge to physics-driven animation. **Wind**, **slanted platform
reflection** and **anchor pinning** are surgical additions on top of the
same engine.

---

## File index

| File                              | Description                                                                 | Engine | What it adds vs. previous |
|-----------------------------------|-----------------------------------------------------------------------------|--------|----------------------------|
| `fk_helloworld.c`                 | 2-link arm driven by joint angles (forward kinematics textbook)             | A      | base case: angles in, positions out |
| `ik_helloworld.c`                 | 2-link arm reaches a target via analytical IK (law of cosines)              | B1     | inverts FK — two solutions, elbow flip |
| `fk_ik_helloworld.c`              | 3-link arm with two modes: FK angles, IK target + wrist-aim constraint      | A, B1  | 3-link IK underdetermined → add a constraint |
| `fk_tentacle_forest.c`            | Eight ocean tentacles sway in a current — stateless FK chains               | A      | per-segment cumulative angle from sin(ω·t) |
| `snake_forward_kinematics.c`      | 32-segment FK snake bouncing inside the screen on a sinusoidal S-curve      | A      | trail-buffer FK: body samples the head's path |
| `snake_inverse_kinematics.c`      | 32-segment snake chasing a wandering target (FABRIK head + trail body)      | A, B2  | IK head added to the FK body |
| `ik_arm_reach.c`                  | 4-link FABRIK arm tracking a Lissajous figure-8 + reach-horizon overlay     | B2     | first FABRIK file — iterative IK |
| `ik_tentacle_seek.c`              | 16-link FABRIK tentacle with per-joint angle constraints                    | B2     | adds bend-angle clamp inside the FABRIK pass |
| `ik_spider.c`                     | 6-leg crawler: trail-buffer body + 2-joint IK legs + autonomous step gait   | A, B1  | first multi-leg gait — drift-triggered swings |
| `ik_scorpin.c`                    | Scorpion: spider model + 7-segment curving FK tail with travelling wave     | A, B1  | adds a second FK chain on top of the spider |
| `hexpod_tripod.c`                 | 2-D hexapod walker with alternating-tripod gait + 2-joint IK                | B1     | rigid-body chassis contrast to spider's flexible body |
| `fk_centipede.c`                  | 24-segment centipede with 10 leg pairs in contralateral antiphase gait      | A      | gait without IK — pure phase-offset sinusoids |
| `ragdoll_figure.c`                | Verlet humanoid (15 particles, 17 bones) tumbling down slanted platforms    | C      | first Verlet file — constraints replace IK |
| `ragdoll_ropes.c`                 | Seven Verlet ropes swaying in sinusoidal wind, anchored to ceiling          | C      | linear chains + wind forcing |

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra <path>.c -o <name> -lncurses -lm
```

**Universal keys** (present in every file):

| Key             | Action                  |
|-----------------|-------------------------|
| `q` / `ESC`     | quit                    |
| `space`         | pause / resume          |
| `r`             | reset                   |
| `t` / `T`       | next / previous theme   |
| `[` / `]`       | sim Hz − / +            |

**Chain-specific keys** (where applicable):

| Key             | Action                                              |
|-----------------|-----------------------------------------------------|
| arrows          | move target (IK files) / rotate joint (FK files)    |
| `f`             | flip elbow to other valid 2-link solution           |
| `m`             | toggle FK ↔ IK mode (`fk_ik_helloworld.c` only)     |
| `w/s`, `a/d`    | wave / amplitude / speed parameter trim             |

---

## Adding a new chain

1. **Pick an engine.** Driven by user angles → FK. Driven by a target →
   IK (analytical if 2-link, FABRIK if more). Driven by forces → Verlet.
2. **Copy the closest existing file** as a template:
   * 2-link arm → `fk_helloworld.c` or `ik_helloworld.c`
   * long stateless chain → `fk_tentacle_forest.c`
   * body that follows a path → `snake_forward_kinematics.c`
   * multi-link IK chain → `ik_arm_reach.c`
   * multi-leg creature → `ik_spider.c` (or `hexpod_tripod.c` for rigid chassis)
   * physics-driven body → `ragdoll_figure.c`
3. **Define the chain.** `Vec2 joint[N]`, `float link_len[N-1]`, and (for
   Verlet) `Vec2 old_pos[N]`. Keep them in §5 as the canonical *entity*.
4. **Replace one function — the per-tick `compute_joints()`.** Everything
   else (clock, color, scene, screen, app, signals) carries over unchanged.
5. **Add CONCEPTS + MENTAL MODEL blocks** per the project's
   [CLAUDE.md](../CLAUDE.md) template.
6. **Verify**: `gcc -Wall -Wextra` clean, stable 60 fps, `q` / `ESC` exits
   cleanly, `SIGWINCH` doesn't crash, HUD shows fps + state.

---

## Cross-folder pointers

* **[`flocking/`](../flocking/)** uses the same per-tick integration pattern
  but with **populations** instead of chains — N independent agents whose
  positions are driven by Reynolds steering forces (separation, alignment,
  cohesion). The animation chain integrates link constraints; the flocking
  agent integrates social-force vectors.
* **[`robots/`](../robots/)** is where the kinematics primitives become
  whole organisms. `robots/walking_robot.c` uses **FK during swing,
  analytical IK during stance** — both engines alternating in one gait
  cycle. `robots/perlin_terrain_bot.c` adds a **PID controller** on top of
  rigid-body integration to balance an inverted pendulum.
* **[`grids/`](../grids/)** is the discrete-coordinate counterpart — chains
  here live in continuous pixel space (`float px, py`), grids live in
  integer cell space (`int r, int c`).
