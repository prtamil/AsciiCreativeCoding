# Ai — small AI / ML visualisations

A starter folder for **AI/ML demos** rendered in the terminal: 2
self-contained C programs that take a concept from the classical
AI/ML toolbox (evolutionary computation, neural network architecture)
and turn it into an animated visual you can poke at in real time. This
folder is deliberately small; everything else in the repository — fluid
solvers, fractals, raymarchers, particle systems — is built around
algorithms with a closed-form objective. The Ai folder is for demos
whose objective is **learning over time** (a population improves, a
particle traverses a network) rather than instantaneous physics.

If you read **only one file**, read [`genetic_rocket.c`](genetic_rocket.c)
— Holland's 1975 genetic algorithm in its most accessible form (Daniel
Shiffman's "Smart Rockets" formulation), with every step of selection
and breeding visible on screen as the population evolves.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
3. [File index](#file-index)
4. [Building and running](#building-and-running)

---

## How to read this folder

The two files are independent — there is no chain. Pick by intent:

```
   "How does a GA find a solution?"          ──▶  genetic_rocket.c
   "What does a feed-forward network        ──▶  neural_net_vis.c
    look like, geometrically?"
```

**Background you need.** Comfort with `float` arithmetic, the
framework conventions (fixed-step accumulator, `erase()/doupdate()`)
from [`../grids/README.md`](../grids/README.md), and a willingness to
read the CONCEPTS + MENTAL MODEL block at the top of each file. No
prior AI/ML background is assumed; both files teach the relevant idea
from scratch.

The folder is **provisional**: a third Ai file would tip the balance
toward Q-learning, a tiny backprop trainer, or a hill-climber gallery.
Until then this folder is two examples of two complementary themes:
**learning by population** (genetic_rocket) and **structure without
learning** (neural_net_vis).

---

## The unifying primitive

What ties the two files together is not a shared algorithm but a shared
*shape of computation*: **discrete agents traversing a state space,
with the visual output showing the trajectory in real time.**

| File                | Agent                | State space                                       | Driving rule                                                  |
|---------------------|----------------------|---------------------------------------------------|---------------------------------------------------------------|
| `genetic_rocket.c`  | Rocket (POP_SIZE of them) | 2-D screen + per-rocket genome (LIFESPAN forces) | per-tick `vel += genes[i]`; per-generation breed + mutate     |
| `neural_net_vis.c`  | Particle (one per input neuron) | Network graph (layers × neurons + edges)         | per-frame edge-by-edge hop forward, wrap when reaching output |

The pattern is:

```
    AGENT POOL (fixed size, no allocation in hot path)
        │
        │   per tick / frame
        ▼
    UPDATE RULE (genome step, edge hop, force vector, ...)
        │
        ▼
    DRAW (positions / trails / connections on the terminal)
```

This is the same pool-tick-draw pattern as `particle_systems/` and the
agent loops in `flocking/`, but the **update rule** is closer in spirit
to learning algorithms:

* `genetic_rocket.c` — genome-driven update + Darwinian breeding over
  generations. Holland 1975. The *population* learns; individual
  rockets do not.
* `neural_net_vis.c` — graph-traversal update without any learning.
  Stage 2 of an evolving file; later stages will add weighted edges,
  pulses on neuron arrival, and back-propagation visualisation.

Reading the two files side by side, the contrast is the point: GA
*has* an objective (hit the target) and improves over time; NN-vis is
pure layout (draw the architecture). Both are valid stations on the
ML pedagogy line; both look similar at the per-frame level.

---

## File index

| File                | Lines  | Subject                                              | DEMO line — what it visually does                                                                |
|---------------------|--------|------------------------------------------------------|--------------------------------------------------------------------------------------------------|
| `genetic_rocket.c`  | ~1500  | Genetic algorithm (Holland 1975, Smart Rockets)      | Population of rockets evolves to hit a target; chaos early, convergence after ~30-80 generations.|
| `neural_net_vis.c`  | ~1450  | Feed-forward network architecture (no learning)      | Columns of `(O)` neurons + full connectivity; one particle per input drifts forward to output.   |

Line counts include the header, CONCEPTS, MENTAL MODEL, and inline
teaching prose — the algorithm itself is 300-500 lines per file.

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra Ai/<file>.c -o <name> -lncurses -lm
```

Both files use `-lm` (forces, distances, layout). The project is strict
about `-Wall -Wextra` clean — both files compile with zero warnings.

**Universal keys** (always present):

| Key             | Action                                  |
|-----------------|-----------------------------------------|
| `q` / `ESC`     | quit                                    |
| `space` / `p`   | pause                                   |
| `r`             | reset                                   |
| `t` / `T`       | next / previous theme                   |

**File-specific keys:**

`genetic_rocket.c`:

| Key             | Action                                                  |
|-----------------|---------------------------------------------------------|
| `f`             | toggle fast-forward (1 generation per frame)            |
| `s`             | toggle trails                                           |
| `[` / `]`       | decrease / increase population (20..100, step 5)        |
| `-` / `+`       | decrease / increase mutation rate (0.001..0.10)         |

`neural_net_vis.c`:

| Key             | Action                                                  |
|-----------------|---------------------------------------------------------|
| `[` / `]`       | decrease / increase layer count (2..12)                 |
| `-` / `+`       | decrease / increase neurons per layer (2..16)           |
| `<` / `>`       | thinner / thicker connection glyphs (dot / thin / bold / heavy) |

---

## Cross-references

* [`../flocking/`](../flocking/) — similar agent + force model
  (Reynolds boids); `genetic_rocket.c` lists it as study-alongside.
* [`../particle_systems/`](../particle_systems/) — particle pools
  without inter-agent forces; baseline for the rocket pool design.
* [`../artistic/galaxy.c`](../artistic/), `artistic/graph_search.c`
  — dot-cluster + edge patterns; `neural_net_vis.c` cites both as
  study-alongside.
* [`../grids/README.md`](../grids/README.md) — `GridCtx` mapping;
  both files use cell-space rendering (no §4 coords block).
* [`../documentation/Master.md`](../documentation/Master.md) —
  long-form essays; relevant chapters when adding a third Ai demo.
