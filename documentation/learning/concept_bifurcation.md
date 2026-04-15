# Concept: Bifurcation Diagram (Logistic Map)

## Pass 1 — Understanding

### Core Idea
The logistic map `x_{n+1} = r·x_n·(1-x_n)` models population growth. For small r, the population reaches a fixed point. As r increases, it period-doubles (2-cycle, 4-cycle, 8-cycle...), then suddenly becomes chaotic. The bifurcation diagram plots all eventual values of x versus r, revealing the route to chaos.

### Mental Model
Imagine a rabbit population. At low growth rate r, population settles to one value. At higher r, it oscillates between two values each year. Higher still — four values, eight, sixteen, then suddenly unpredictable chaos. The Feigenbaum constant δ≈4.669 governs the ratio of each period-doubling step.

### Key Equations
```
x_{n+1} = r · x_n · (1 - x_n)
```
- r ∈ [0,4] for bounded behavior
- x ∈ [0,1]
- Onset of chaos: r ≈ 3.57

Feigenbaum constant: ratio of consecutive period-doubling intervals converges to δ≈4.669201...

### Data Structures
- Screen column = r value
- For each r: iterate 1000 warmup steps, then plot next 200 values as dots in that column
- No array needed — draw points directly

### Non-Obvious Decisions
- **Warmup iterations**: Discard first 500–1000 iterations so transients die out. Only plot the attractor.
- **Many r values**: Sweep r over [2.5,4.0] with one column per pixel column. This gives ~100–200 distinct r values in a terminal.
- **Zoom into Feigenbaum region**: The first bifurcation is at r≈3, next at r≈3.449, next at r≈3.544. Zoom into [3.5,4.0] to see the detail.
- **Color by period**: Track cycle length and color each branch differently to see period-doubling structure.

### Key Constants
| Name | Role |
|------|------|
| R_MIN, R_MAX | range of r to display (e.g. 2.5–4.0) |
| WARMUP | iterations to discard (500–1000) |
| PLOT_ITER | iterations to plot per r value (100–300) |

### Open Questions
- Measure the ratio r_{n+1}-r_n / r_{n+2}-r_{n+1} for successive bifurcations. Does it approach 4.669?
- The logistic map in the chaotic region has periodic windows. Find the period-3 window.
- Does the same bifurcation structure appear in the sine map `x' = r·sin(π·x)`?

---

## Pass 2 — Implementation

### Pseudocode
```
for col in 0..cols:
    r = R_MIN + col * (R_MAX-R_MIN)/cols
    x = 0.5       # initial condition
    for n in 0..WARMUP: x = r*x*(1-x)     # discard transient
    for n in 0..PLOT_ITER:
        x = r*x*(1-x)
        row = (int)((1-x) * rows)          # x=1 at top, x=0 at bottom
        if row in [0,rows): mvaddch(row, col, '.')
```

### Module Map
```
§1 config    — R_MIN, R_MAX, WARMUP, PLOT_ITER
§2 compute   — logistic_map(), period_detect()
§3 draw      — render() sweeps r, plots attractor points
§4 app       — main (initial draw, keys: zoom, shift, reset)
```

### Data Flow
```
column → r value → iterate (warmup then plot)
→ x values → screen row → mvaddch
```

## From the Source

**Algorithm:** Logistic map iteration with parameter scanning. For each column, r is set to a distinct value from a range. WARMUP=500 transient iterations are discarded; the next PLOT=300 values are plotted — these are on the attractor.

**Math:** The logistic map `x_{n+1} = r·xₙ·(1−xₙ)` models population dynamics. For r < 3: converges to a fixed point. r ≈ 3.0: period-2 bifurcation. Each bifurcation point r_n satisfies `r_{n+1} − r_n → 1/δ` where `δ = 4.669...` is Feigenbaum's constant (universal across all unimodal maps, not just the logistic map). At r ≈ 3.5699: accumulation point — onset of chaos. For r > 4: all trajectories diverge. The constant `FEIGENBAUM_R = 3.5699456718695f` is embedded in the source.

**Performance:** O(W × (WARMUP + PLOT)) per diagram redraw. Each column is independent (embarrassingly parallel). Auto-zoom scrolls r toward r∞ = 3.5699 so self-similar copies of the bifurcation diagram are continuously revealed.
