# Pass 1 — galaxy.c: Spiral Galaxy Simulation

## Core Idea

3000 stars orbit a galactic centre in exact circular orbits. The spiral arms are not a physical structure — they are a **pattern** created by the initial placement of stars on logarithmic spirals and the subsequent differential rotation. The same mechanism is active in real spiral galaxies.

Two principles drive everything:
1. **Flat rotation curve** — every star orbits at the same tangential speed regardless of radius, so `ω(r) = v₀/r`
2. **Differential rotation** — because inner stars have higher ω, they complete orbits faster and gradually overtake outer stars, winding the arms up

## The Mental Model

### Why flat rotation curve, not Keplerian?

Keplerian orbits (solar system, one dominant mass) give `v ∝ 1/√r`, so `ω ∝ r^(−3/2)`.

Real spiral galaxies have dark matter halos that add mass at large radii. The combined mass enclosed grows fast enough that `v_circ ≈ const` at all observable radii. This is the famous **rotation curve problem** — galaxies rotate "too fast" at large radii if you only count visible matter, implying unseen mass.

The code uses `ω = V0 / r` which is exactly the flat-rotation-curve condition. Inner stars still orbit faster (smaller r → larger ω), but the differential is less extreme than Keplerian.

### Why logarithmic spirals?

A **logarithmic spiral** has the property that the angle increases linearly with `ln(r)`:
```
θ = θ_start + WINDING × ln(r / r_min)
```
Or equivalently: `r = r_min × exp((θ − θ_start) / WINDING)`.

Key property: the spiral intersects all radii at the same angle (self-similar). This is why galaxy arms look geometrically similar at all scales.

The `WINDING` parameter controls tightness:
- `WINDING = 0.5` — loose, open arms (like M33 Triangulum)
- `WINDING = 1.0` — moderate (like the Milky Way, code default)
- `WINDING = 2.0` — tight, wound arms

### Differential rotation winds up the arms

After the simulation starts, each star advances its angle independently:
```
θ(t) = θ₀ + ω(r) × t = θ_start + WINDING × ln(r) + (V0/r) × t
```

The term `(V0/r) × t` grows faster for small r than large r. The arm shape deforms from a logarithmic spiral into a more tightly wound structure. After many orbits the arms become indistinguishable from rings — this is called **wind-up** and is a real problem in galaxy formation theory (solved by density waves, not implemented here).

### Brightness accumulator as a display technique

Rather than drawing individual star positions, each frame:
1. Advance all 3000 star angles
2. For each star, add weight to `g_bright[sy][sx]`
3. Multiply entire grid by `DECAY = 0.82`
4. Render: bright cells → dense chars (`@`), dim cells → sparse chars (`.`)

The **steady-state brightness** for a cell receiving `f` stars per frame:
```
B_ss = f / (1 − DECAY) = f × 5.56
```
Normalising by `b_max` maps this to [0,1] for character selection. This self-calibrates regardless of star density or speed settings.

The decay creates a **motion blur trail** automatically: stars that occupied a cell in previous frames still contribute, exponentially fading. Trail length ≈ `1 / (1 − DECAY) = 5.6` frames.

### Radial colour zones (not per-star colour)

Colour is determined by **screen distance from centre**, not by which star occupies the cell. This is physically motivated: core has old red/yellow stars, disc has young blue-white stars, halo has metal-poor dim stars.

```c
float rn = sqrt(dx²/rx² + dy²/ry²);   /* normalised galactic radius */
cp = (rn < 0.10) ? CP_CORE : (rn < 0.65) ? CP_DISK : CP_HALO;
```

The threshold `0.10` for the core corresponds to 10% of the display radius — roughly matching the visible bulge fraction in real galaxies.

### Aspect ratio correction

Terminal cells are ~2× taller than wide. Plotting `(r·cos θ, r·sin θ)` directly maps to pixels, but the galaxy appears as an ellipse stretched vertically.

Fix: scale `ry = rx × 0.5`. The star plots as:
```c
sx = cx + (int)(r * cos(θ) * rx);    /* unscaled x */
sy = cy + (int)(r * sin(θ) * ry);    /* half-scale y */
```
This produces an oval in grid coordinates that appears circular on screen.

### Box-Muller for the bulge

The bulge needs stars concentrated near r=0 with a smooth falloff. A half-Gaussian works:
```c
r = |gauss(0, 0.07)|   /* radius drawn from positive half of N(0, σ) */
```
Box-Muller transform: given two uniform samples u₁, u₂:
```c
z = sqrt(-2 × ln(u₁)) × cos(2π × u₂)   /* standard normal */
r = |z × 0.07|
```
68% of bulge stars land within `r = 0.07`, 95% within `r = 0.14`. This creates the bright concentrated core naturally.

## Data Structures

```c
typedef struct {
    float r;       /* normalised orbital radius 0..1 */
    float theta;   /* current angle (radians) */
    float omega;   /* angular velocity: V0 / r */
} Star;

Star  g_stars[N_STARS];            /* 3000 stars */
float g_bright[ROWS_MAX][COLS_MAX]; /* brightness accumulator */
```

No velocity vector needed — orbits are exact circular paths computed analytically from `r` and `theta`.

## Galaxy Population

| Group | Count | r range | theta |
|---|---|---|---|
| Bulge | 600 | 0..0.20 (half-Gaussian σ=0.07) | uniform |
| Arms  | 2100 | 0.08..0.95 (uniform) | log-spiral + ±0.25 rad scatter |
| Halo  | 300  | 0.35..1.05 (uniform) | uniform |

## The Main Loop

```
galaxy_init(narms):
    for bulge stars: r ~ |gauss(0,0.07)|, theta random, omega = V0/r
    for arm stars: r ~ uniform(0.08,0.95)
                   theta = arm_offset + WINDING × ln(r/r_min) + scatter
                   omega = V0/r
    for halo stars: r ~ uniform(0.35,1.05), theta random, omega = V0/r

scene_draw():                          /* once per frame */
    g_bright[r][c] *= DECAY            /* decay all cells */

galaxy_step():                         /* g_steps times per frame */
    for each star:
        theta += omega × g_speed
        sx = cx + r × cos(theta) × rx
        sy = cy + r × sin(theta) × ry
        g_bright[sy][sx] += 1.0 / g_steps

scene_draw() (continued):
    b_max = max(g_bright)
    for each cell (r,c):
        t = g_bright[r][c] / b_max
        ch  = '. , : o O 0 @'[level(t)]
        cp  = zone(screen_dist(r,c))
        draw ch with COLOR_PAIR(cp)
```

## Non-Obvious Decisions

### Why divide by g_steps when accumulating

`g_bright[sy][sx] += 1.0f / g_steps`

If STEPS=4 and we add 1.0 per step, one frame contributes 4× as much as STEPS=1. The brightness scale changes with speed settings. Dividing by g_steps keeps total contribution per frame at 1.0 per star regardless of g_steps — the display looks the same at all speeds.

### Why normalise by b_max rather than a fixed scale

The steady-state brightness depends on DECAY, g_steps, g_speed, and star distribution. Normalising by b_max makes the display always use the full character range regardless of parameter settings. The galaxy is always visually dense at its densest point.

### Why colour by screen position, not star type

Stars from different populations (bulge, arm, halo) mix after many orbits. A bulge star can orbit out to disk radii. Tagging colour by star origin would produce wrong colours for displaced stars. Screen-radius colouring always matches the physical expectation (inner = hot/bright, outer = cool/dim).

## Open Questions for Pass 3

- What is the arm **winding time** — how many ticks until the arms decohere into rings? Measure by computing the azimuthal entropy of the star distribution.
- Does the simulation show the correct **rotation curve**? Measure average angular displacement per tick as a function of r — should be linear in 1/r.
- What happens with `N_ARMS = 3` or `4`? Real galaxies have 2 or 4 arms most commonly — why?
- Add a **density wave**: a rotating oval potential that stars feel as they pass through. This would maintain arm coherence indefinitely. What minimum amplitude prevents wind-up?
- The `WINDING` parameter sets the initial spiral geometry. What values of WINDING produce the most visually stable arms (slowest to wind up)?

---

## From the Source

**Algorithm:** Kinematic circular orbit simulation — no N-body gravity. Each star is placed on a logarithmic spiral arm at init with a fixed angular speed ω = v_tan / radius. Each tick: θ += ω × dt. Position = (r·cos θ, r·sin θ).

**Physics:** Flat rotation curve: all stars orbit at the same tangential speed v_tan regardless of radius (like real spiral galaxies, explained by dark matter halos). Differential rotation: ω ∝ 1/r → inner stars orbit faster → arms wind up over time (the classic winding problem).

**Math:** Logarithmic spiral: r = a·exp(b·θ), so θ = ln(r/a)/b. Stars seeded along N_ARMS arms at equal r-intervals; their initial θ values are staggered by 2π/N_ARMS.

**Rendering:** Stars projected to screen (r, θ) → (col, row) with aspect correction. Density → glyph ramp (`. , : o O 0 @`). Radial zone (CORE/DISK/HALO) determines colour pair.
