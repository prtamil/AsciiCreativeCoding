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
