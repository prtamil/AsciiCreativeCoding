# Concept File: `grids/polar_grids_placement/04_polar_scatter.c`

> Polar scatter placement — four strategies for distributing objects in polar space: uniform-area, radial-Gaussian, angular wedge, and ring-snap.

---

## Pass 1 — First Read

### Core Idea

Scatter placement is about choosing **where** to put N random objects according to a distribution law. Four distinct laws are implemented, each producing a visually and mathematically different pattern. The key challenge is that naïve scatter (uniform in r and θ) concentrates objects near the origin — fixing this requires the **uniform-area transform**.

### Mental Model

Imagine throwing darts at a circular dartboard:
- **Uniform-area (U)**: Equal probability per square centimetre — darts spread evenly over the annulus.
- **Radial-Gaussian (G)**: Most darts cluster near a target ring, falling off like a bell curve outward.
- **Wedge (W)**: Darts confined to a pie-slice angle range; uniform within the wedge.
- **Ring-snap (D)**: Darts snap to discrete ring tracks with small random jitter — like a vinyl record.

### Key Equations

**Uniform-area** (why sqrt matters):
```
The area element in polar is r dr dθ.
Naive: r = r₀ + rand*(r₁ - r₀)  → concentrates near r₀
Correct (equal area): r = sqrt(r₀² + rand*(r₁² - r₀²))
```
This linearises the CDF of the area element. The sqrt comes from integrating `∫r dr = r²/2`.

**Radial-Gaussian** (Box-Muller):
```
u1 = uniform in (0, 1)   (avoid 0 for log)
u2 = uniform in [0, 1)
z  = sqrt(-2 × ln(u1)) × cos(2π × u2)   /* standard normal */
r  = |cursor_r + sigma × z|              /* fold to positive */
θ  = uniform in [0, 2π)
```
Box-Muller transforms two uniform samples into one Normal sample. The `|...|` fold prevents negative radii.

**Wedge**:
```
θ = cursor_theta - wedge_half + rand * (2 × wedge_half)
r = sqrt(r_inner² + rand*(r_outer² - r_inner²))   /* uniform area */
```

**Ring-snap**:
```
k = rand_int in [1, N_RINGS]
r = k × RING_SNAP_SP + rand * RING_JITTER - RING_JITTER/2
θ = uniform in [0, 2π)
```

### Non-Obvious Decisions

- **`last_key` tracks context for `[`/`]`**: Pressing `[`/`]` means "decrease/increase the last-used parameter" — sigma for `G`, wedge for `W`. A single `char last_key` is enough. After `U` or `D`, `[`/`]` has no effect.
- **`srand()` in `screen_init`**: Calling `srand(time(NULL))` at startup gives different scatter each run. Forgetting this causes identical scatter every run (seed 1 by default in C).
- **`R_OUTER_FACTOR = 0.42`**: The scatter radius ceiling is `0.42 × half-screen-diagonal`. This keeps scatter contained within the visible terminal. 0.5 would touch corners; 0.42 leaves a visible margin.
- **Box-Muller `u1` uses `+1.0` / `+2.0` offset**: `log(0)` is undefined. Shifting u1 to `(rand+1)/(RAND_MAX+2)` ensures it's always in `(0, 1)`.

### Open Questions

- Could you add a spiral-density scatter — more objects along a spiral track, falling off with distance from it?
- What distribution law would produce a uniform scatter on the surface of a sphere projected to a polar plane (gnomonic projection)?

---

## From the Source

```c
static double gauss(double mean, double sigma) {
    double u1 = ((double)rand()+1.0) / ((double)RAND_MAX+2.0);
    double u2 = (double)rand() / ((double)RAND_MAX+1.0);
    double z  = sqrt(-2.0*log(u1)) * cos(2.0*M_PI*u2);
    return mean + sigma * z;
}

static void scatter_uniform(ObjPool *pool, int n, double r0, double r1,
                             int rows, int cols, int ox, int oy)
{
    double r0sq = r0*r0, r1sq = r1*r1;
    for (int i = 0; i < n; i++) {
        double r = sqrt(r0sq + ((double)rand()/RAND_MAX) * (r1sq - r0sq));
        double theta = ((double)rand()/RAND_MAX) * 2.0*M_PI;
        int c, row; polar_to_screen(r, theta, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}
```

---

## Pass 2 — After Running It

### Pseudocode

```
sigma = SIGMA_R_DEFAULT, wedge = WEDGE_DEFAULT, n_scat = N_SCATTER_DEFAULT
last_key = ' '
loop:
  key = getch()
  if arrows: move cursor; sync polar
  if 'U': scatter_uniform(n_scat); last_key = 'U'
  if 'G': scatter_gauss(n_scat, cur_r, sigma); last_key = 'G'
  if 'W': scatter_wedge(n_scat, cur_r, cur_theta, wedge); last_key = 'W'
  if 'D': scatter_ringsnap(n_scat); last_key = 'D'
  if '+'/'-': adjust n_scat
  if '[': if G: sigma--; if W: wedge--
  if ']': if G: sigma++; if W: wedge++
  if 'C': pool_clear()
  draw: bg → objects → cursor → HUD
```

### Module Map

| Function | Strategy |
|----------|---------|
| `scatter_uniform` | Uniform-area: r=sqrt(r₀²+rand×(r₁²-r₀²)) |
| `scatter_gauss` | Radial-Gaussian: Box-Muller around cursor_r |
| `scatter_wedge` | Angular wedge + uniform-area radial |
| `scatter_ringsnap` | Quantised r = k×RING_SNAP_SP + jitter |
| `gauss` | Box-Muller transform: 2 uniforms → 1 Normal |

### Data Flow

```
Scatter key → scatter_*(pool, n, params)
  → for each of n points: sample (r, θ) from distribution
  → polar_to_screen(r, θ) → pool_place
pool_draw renders all accumulated objects
'['/']' → adjust sigma or wedge based on last_key context
```
