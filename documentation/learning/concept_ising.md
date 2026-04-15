# Concept: Ising Model (Statistical Mechanics)

## Pass 1 — Understanding

### Core Idea
A grid of magnetic spins (+1 or -1). At each step, flip a random spin if it lowers energy, or sometimes flip it even if it raises energy (with probability e^{-ΔE/kT}). This Metropolis algorithm samples the equilibrium distribution. Near the critical temperature Tc≈2.269, the system exhibits a phase transition from ordered (ferromagnet) to disordered (paramagnet).

### Mental Model
Imagine a grid of compass needles, each trying to align with its neighbors (low energy). But thermal noise randomly flips needles. At low temperature, almost all needles align — ordered. At high temperature, random flips dominate — disordered. Near Tc, the system fluctuates between large ordered patches and disorder — critical phenomenon.

### Key Equations
Energy of the system:
```
E = -J · Σ_{<ij>} s_i · s_j    (J=1: ferromagnet)
```
Energy change when flipping spin k:
```
ΔE = 2·J·s_k · Σ_{neighbors} s_n
```

Metropolis acceptance:
```
if ΔE ≤ 0: always flip
if ΔE > 0: flip with probability exp(-ΔE / (k_B · T))
```

Critical temperature (2D Ising): `Tc = 2J / (k_B · ln(1+√2)) ≈ 2.269 J/k_B`

### Data Structures
- `spin[H][W]`: int8, values ±1
- Total magnetization M = Σ s_i (running sum, update ±2 on each flip)
- Total energy E (update ΔE on each flip — don't recompute from scratch)

### Non-Obvious Decisions
- **Precompute acceptance table**: ΔE ∈ {-8,-4,0,4,8} (only 5 values for square lattice). Precompute `exp(-ΔE/T)` for these 5 values. Avoids calling exp() in the hot loop.
- **Checkerboard update**: Update all black squares, then all white squares of the checkerboard. This allows parallelism and avoids reading/writing the same cell simultaneously.
- **Visualization**: Color +1 spins white, -1 spins black. Near Tc, you see fractal-like domain boundaries.
- **Order parameter**: Plot magnetization |M|/N vs. time. Near Tc it fluctuates dramatically — this is a signature of criticality.
- **Temperature sweep**: Start at high T (random), slowly cool. Watch phase transition happen.

### Key Constants
| Name | Value | Role |
|------|-------|------|
| J | 1.0 | coupling constant |
| T_INIT | 3.0 | initial temperature (above Tc) |
| T_CRIT | 2.269 | critical temperature |
| STEPS_PER_FRAME | H*W | one Monte Carlo sweep |

### Open Questions
- What is the divergence of the correlation length at Tc?
- Show the specific heat C = <E²>-<E>² / (kT²) as a function of T — does it peak at Tc?
- Try the Ising model on a triangular lattice — different Tc?

## From the Source

**Algorithm:** Metropolis Monte Carlo (MCMC). Randomly propose a spin flip; accept with probability `P = min(1, exp(−ΔE / kT))`. Over many steps this samples the Boltzmann distribution at temperature T. Not a physics simulation in real-time — it is a statistical sampler converging toward thermal equilibrium.

**Physics:** 2D Ising model of ferromagnetism. Each spin s = ±1 interacts with nearest neighbours. `ΔE = 2·s·(sum of 4 neighbours)` — the energy cost of flipping one spin given its environment. Below T_crit ≈ 2.269 (in units where J=k_B=1), large aligned domains spontaneously appear — a textbook example of a 2nd-order phase transition. T_CRIT = 2 / ln(1 + √2) ≈ 2.2692 — exact analytical result by Lars Onsager (1944).

**Math:** ΔE formula exploits the fact that the Hamiltonian is a sum of pairwise products: H = −J·Σ s_i·s_j. Flipping spin i changes H by `2·J·s_i·Σ_nbr s_j`. For J=1 this is ΔE = 2·s·Σ_nbr (values ±4, ±8, ±16 depending on alignment with 4 neighbours).

**Performance:** FLIPS_PER_CELL attempts per grid cell per frame. At 200×60=12000 cells, 50 attempts each = 600k flips/frame at 30 fps ≈ 18M flip attempts/s — fast enough to see domain formation in real-time.

---

## Pass 2 — Implementation

### Pseudocode
```
# Precompute acceptance probabilities
for dE in {-8,-4,0,4,8}:
    accept_prob[dE/4 + 2] = min(1.0, exp(-dE / T))

metropolis_step():
    i = random(H); j = random(W)
    neighbor_sum = spin[(i+1)%H][j] + spin[(i-1+H)%H][j]
                 + spin[i][(j+1)%W] + spin[i][(j-1+W)%W]
    dE = 2 * spin[i][j] * neighbor_sum
    if dE <= 0 or random() < accept_prob[dE/4 + 2]:
        spin[i][j] *= -1
        E += dE
        M += 2 * spin[i][j]

one_sweep():
    for _ in range(H*W): metropolis_step()

draw():
    for i in 0..H:
        for j in 0..W:
            if spin[i][j] > 0: mvaddch(i, j, '#' | COLOR_WHITE)
            else:               mvaddch(i, j, ' ')
    draw_HUD(T, M/N, E/N)
```

### Module Map
```
§1 config    — H, W, J, T_INIT, STEPS_PER_FRAME
§2 init      — random spins, compute initial E and M
§3 physics   — precompute_table(), metropolis_step(), one_sweep()
§4 draw      — spin grid + magnetization bar + energy
§5 app       — main loop, keys (temperature +/-, reset, cold/hot start)
```

### Data Flow
```
spin[H][W] → random site → ΔE calculation → Metropolis accept/reject
→ flip spin (update E,M) → draw spin grid → screen
```
