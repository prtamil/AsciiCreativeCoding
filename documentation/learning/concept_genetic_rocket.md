# Concept: Genetic Algorithm Rockets

## Pass 1 — Understanding

### Core Idea
A population of rockets evolves to hit a target. Each rocket carries a fixed-length genome of force vectors that it applies one per simulation tick — its entire flight is genetically determined. After every generation, fitness-proportional selection picks parents and single-point crossover with mutation breeds the next generation. Initial rockets fly chaotically; after 30–80 generations most reliably hit the target.

### Mental Model
The genome is an instruction tape: at tick `i` the rocket adds `genes[i]` to its velocity. Run all rockets for `LIFESPAN` ticks. Score by proximity to the target (with bonuses for hits, penalties for crashes). Fittest genomes propagate; weakest die out. Repeat. The selection pressure rewards genomes that produce trajectories ending close to the target — over generations the population converges toward "always hit."

### Key Equations
```
fitness(r) = (1 / (dist(r, target) + 1))²
           × 10  if hit_target
           × 0.1 if crashed

pool weights: each rocket appears k = 100·f/f_max times in mating pool
crossover: child.genes[i] = (i < mid) ? parentA.genes[i] : parentB.genes[i]
mutation:  if frand() < MUTATION_RATE: replace genes[i] with random unit force
```

### Non-Obvious Decisions
- **Fitness squared**: Amplifies selection pressure without going so extreme that the population collapses to one solution.
- **Hit bonus ×10, crash penalty ×0.1**: Crashers still contribute occasional good genes (their early flight may be useful).
- **Single-point crossover, not uniform**: Preserves contiguous "manoeuvre patterns" in the genome rather than scrambling them.
- **`t = 0` reset on shape change**: All particles snap to launch, makes the moment of restart visible.
- **Two spawn paths (`bomb_spawn` and `bomb_spawn_burst`)**: Burst uses 1.5× speed and wider cone so it's visually unmistakable even when pool is full.

### Key Constants
| Name | Role |
|------|------|
| `POP_DEFAULT` | 50 rockets per generation |
| `LIFESPAN` | 100 ticks per rocket lifetime |
| `MUTATION_DEFAULT` | 0.01 — fraction of genes mutated per child |
| `MAX_FORCE` | 0.05 cells/tick² per gene |
| `MAX_VEL` | 0.6 cells/tick velocity cap |

### Open Questions
- What if you used uniform crossover (per-gene parent pick) instead of single-point?
- How does mutation rate interact with population size? Higher pop tolerates lower mutation.
- Could you let rocks (obstacles) appear between launch and target — does the GA still solve it?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — POP_SIZE, LIFESPAN, MAX_FORCE, MUTATION_RATE
§4 random   — frand, rand_unit (rejection sampling on disc)
§5 rocket   — Rocket struct, init_genome, launch, tick, fitness
§6 ga       — World struct, ga_breed (crossover + mutation), ga_evolve
§7 scene    — draw_target + draw_rockets + draw_trails + HUD
§9 app      — main loop, fast-forward toggle, key handling
```

### Data Flow
```
init: random genomes for every rocket → launch all
tick: for each alive rocket: vy += gene[age]; pos += vel; check target/crash
end of generation (LIFESPAN ticks or all dead):
    compute fitness for every rocket
    build mating pool weighted by fitness
    breed new generation: crossover(rand parent, rand parent) + mutate
    relaunch all rockets
    increment generation counter
```

### References
- Holland, "Adaptation in Natural and Artificial Systems" (1975)
- Shiffman, "The Nature of Code" ch. 9 — Smart Rockets formulation
- Goldberg, "Genetic Algorithms in Search, Optimization, and Machine Learning" (1989)
