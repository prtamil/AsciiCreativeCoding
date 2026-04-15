# Pass 1 — wave_interference.c: Analytic Wave Interference

## Core Idea

Each point source `i` emits a sinusoidal cylindrical wave. The total field at every terminal cell is the **superposition** of all N source waves, computed analytically each frame:

```
u_i(x,y,t) = sin( ω_i · t  −  k_i · r_i  +  φ_i )
u_total     = Σ u_i
```

where `r_i` is the physical pixel-space distance from source `i` to the cell. No FDTD grid, no CFL stability condition, no transient warm-up — just analytic sinusoids evaluated per pixel.

## The Mental Model

### Why analytic rather than FDTD

FDTD (Finite Difference Time Domain) evolves the wave equation `∂²u/∂t² = c²∇²u` on a discrete grid. It needs a CFL stability condition (`c·dt/dx ≤ 1`), requires many cells for long-wavelength accuracy, and needs many frames to reach steady state.

The analytic approach gives the **steady-state interference pattern immediately** on frame 1. The trade-off: FDTD handles boundaries, reflections, and dispersion naturally; analytic does not (no reflections, no boundaries, infinite medium).

For a visual demonstration of interference phenomena the analytic approach is superior — the pattern appears instantly and every parameter change takes effect the next frame.

### Phase precomputation

The term `k·r_i` depends only on source position and wavenumber, not on time. It is precomputed into `g_phase[row][col]` for each source whenever a source moves:

```c
for each cell (row, col):
    dx = col - src.col;   dy = row - src.row;
    r  = sqrtf((dx*CELL_W)² + (dy*CELL_H)²);   /* pixel-space distance */
    g_phase[s][row][col] = k * r;               /* k = 2π / (λ·CELL_W) */
```

Per-frame cost per cell: `N` `sinf()` calls + N adds + `sinf()` output scaling. No `sqrt()` per frame.

### Terminal aspect ratio in distance

Terminal cells are `CELL_W=8 px` wide and `CELL_H=16 px` tall. A source at `(sc, sr)` and a cell at `(col, row)` have physical pixel-space distance:

```
r = sqrt( (dx·CELL_W)² + (dy·CELL_H)² )
```

Without this correction, circular wavefronts would appear elliptical on screen (stretched vertically). With it, wavefronts appear circular even though the terminal grid is rectangular.

### The wavenumber k

```
k = 2π / λ_physical  where  λ_physical = λ_terminal × CELL_W
```

`λ_terminal` is the user-visible wavelength in terminal columns (e.g. 20 cols). Multiplying by `CELL_W=8` gives the wavelength in pixels, which matches the pixel-space `r`.

### 8-level signed colour ramp

```
u_total / N  ∈ [-1, 1]  (normalised by source count)

u < -0.75     CP_N3   very deep trough (cold)
u < -0.50     CP_N2
u < -0.25     CP_N1
u <  0        CP_N0   shallow trough
u <  0.25     CP_P0   shallow crest (warm)
u <  0.50     CP_P1
u <  0.75     CP_P2
u ≥  0.75     CP_P3   very deep crest (hot)
```

Near-zero amplitudes (the four middle levels) use sparse characters (`·`, `:`) so the background shows through. Deep amplitudes use dense characters (`@`, `#`) with bold.

### The four presets

| Preset | Sources | Effect |
|--------|---------|--------|
| Double Slit | 2 in-phase, same ω | Hyperbolic nodal lines — Young's double-slit |
| Ripple Tank | 4 at corners | Cross-hatched square fringes |
| Beat | 2 sources, Δω ≠ 0 | Moving envelope (beating waves) |
| Radial | 6 on circle | 6-fold star pattern |

**Beat preset mechanics:** two sources with slightly different ω (ω and ω+Δω) produce a wave whose amplitude envelope oscillates at Δω with spatial wavelength = harmonic mean of the two λ. This is the audio analogue of two guitar strings slightly out of tune.

### Source selection and interactive editing

`g_sel` tracks the selected source (0..N_SRC-1). Arrow keys move it by 1 cell, triggering `phases_rebuild()`. `n` adds a source at centre; `x` removes the selected. `+/-` and `f/F` change λ and ω for the selected source only.

Two colour pairs distinguish selected (`CP_SRC`) from unselected (`CP_SRC2`) source markers, drawn as `*` on the field.

## Data Structures

```c
typedef struct {
    float col, row;   /* source position (terminal coordinates) */
    float omega;      /* angular frequency (rad/frame) */
    float lambda;     /* wavelength (terminal columns) */
    float k;          /* wavenumber in pixel space */
    float phi;        /* initial phase offset */
} Source;

Source g_src[N_SRC_MAX];
int    g_nsrc;
float  g_phase[N_SRC_MAX][GRID_H_MAX][GRID_W_MAX];  /* k·r precomputed */
float  g_t;   /* simulation time (frames) */
```

## The Main Loop

```
phases_rebuild(s):                       ← called on source change
    for each cell (row, col):
        r = sqrt((dx*CELL_W)² + (dy*CELL_H)²)
        g_phase[s][row][col] = src[s].k * r

scene_draw() per frame:
    g_t += 1.0f
    for each cell (row, col):
        u = 0
        for s in 0..nsrc-1:
            u += sinf(src[s].omega * g_t - g_phase[s][row][col] + src[s].phi)
        u /= nsrc                            ← normalise
        level = signed_level(u)
        draw char and color for level
    draw source markers, HUD
```

## Non-Obvious Decisions

### Why normalise by source count

With N sources each contributing ±1, the raw sum `u` ∈ [-N, N]. Dividing by N keeps `u ∈ [-1, 1]` regardless of how many sources exist, so the same color thresholds work for 1 source and for 8.

The trade-off: constructive interference between N sources has amplitude N (full reinforcement), but after normalisation it shows as 1.0 — the same as a single source. The normalisation preserves relative pattern but not absolute intensity.

### Why `CELL_W=8, CELL_H=16` specifically

These are the standard dimensions of an 80×25 VT100 terminal at common font sizes (9×16 or 8×16 pixels). They are project-wide constants used in all physics simulations for consistent aspect-ratio correction. A 20-column wavelength at CELL_W=8 means 160 pixels — a comfortable number of full cycles visible on screen.

### Phase stored per source, not per cell

The alternative — storing one `g_phase[row][col]` array per cell and summing sources there — would require rebuilding the entire grid every time *any* source changes. The current layout `g_phase[s][row][col]` allows rebuilding only source `s` on source `s`'s change, leaving other sources' phase tables intact.

### Static preallocation (N_SRC_MAX=8, GRID_H_MAX=100, GRID_W_MAX=300)

`g_phase` would be `8 × 100 × 300 × 4 = 960 KB` of floats — fits comfortably in static storage. Dynamic allocation would save memory for fewer sources but add malloc complexity. The static approach follows the project convention of fixed upper bounds.

## Open Questions for Pass 3

- Add **reflection** from a wall: a source and its mirror image across the wall produce the correct reflected wavefronts analytically.
- Implement **smooth contour drawing**: instead of per-cell filled characters, draw only cells where `|u| ≈ threshold` to show nodal lines (zeros of the wave field).
- The beat pattern (Δω preset) shows a moving envelope. Does the envelope speed match the theoretical group velocity `v_g = dω/dk`? Measure it.
- Add a **phase slider** for source i to demonstrate coherence effects: shifting one source's φ by π should flip constructive/destructive interference.

## From the Source

**Algorithm:** Fully analytic (closed-form) superposition — no PDE integration, no grid state, no stability condition. Unlike FDTD wave.c, the result is exact and can be paused/resumed without losing transient warmup time. Each frame recomputes u(x,y,t) = Σ sin(ωt − kr + φ) directly.

**Physics/References:** Cylindrical waves (Huygens' principle). Constructive interference (crests adding): u ≈ N·A. Destructive (cancellation): u ≈ 0.

**Math:** Phase precomputation: k·r computed once per source per grid cell. Per-frame cost = N_sources × W × H × sinf() calls. Aspect correction uses CELL_W=8, CELL_H=16 (standard 80×25 VT100 terminal pixel dimensions) so waves appear circular on screen.

**Performance:** Phase stored per source (g_phase[s][row][col]) allows rebuilding only one source's table when that source changes, leaving other sources' tables intact. Static preallocation: g_phase[8][100][300] × 4 bytes = 960 KB — fits in static storage without malloc complexity.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_src[N_SRC_MAX]` | `Source[8]` | ~128 B | source positions, ω, λ, phase, active flag |
| `g_kp[N_SRC_MAX][GRID_H_MAX][GRID_W_MAX]` | `float[8][100][300]` | ~960 KB | precomputed spatial phase table k·r − φ per source per cell |
| `g_time` | `float` | 4 B | simulation time (advances each frame when unpaused) |
