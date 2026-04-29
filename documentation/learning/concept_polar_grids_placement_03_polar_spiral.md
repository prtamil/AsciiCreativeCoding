# Concept File: `grids/polar_grids_placement/03_polar_spiral.c`

> Parametric spiral placement — walk the curve analytically and convert each point to a cell, instead of screen-sweeping.

---

## Pass 1 — First Read

### Core Idea

Instead of testing every screen cell ("is this on a spiral?"), we **walk the spiral parametrically**: increment the parameter t, compute (r, θ) from the spiral formula, and convert to (col, row) via `polar_to_screen`. This is the inverse of how the background grids in `polar_grids/` render spirals — and it's far better for *placement* because it places objects exactly on the curve at any density.

### Mental Model

Imagine rolling a ball along a spiral track — you always know where the ball is, and you can adjust how fast it rolls (density) and how many laps it takes (turns). Screen-sweep would instead ask "is each pixel on the track?" — correct but inefficient and hard to control.

### Key Equations

**Archimedean spiral** (`l` key):
```
a = pitch / (2π)          /* radial advance per radian */
r(t) = r₀ + a × t
θ(t) = θ₀ + t
t ∈ [0, n_turns × 2π]
```
Gap between successive arms: constant = `pitch` pixels.

**Logarithmic spiral** (`o` key):
```
r(t) = r₀ × e^(growth × t)
θ(t) = θ₀ + t
t ∈ [0, n_turns × 2π]
```
Gap grows exponentially. The **golden spiral** is the special case where `growth = 2×ln(φ)/π ≈ 0.3065` — each quarter-turn scales r by the golden ratio φ ≈ 1.618.

### Non-Obvious Decisions

- **`density` controls step in radians**: Smaller = more objects placed per turn. Default 0.08 rad ≈ 4.6° — about one object per cell at medium radius. Too small and objects overlap; too large and the curve looks dotted.
- **Golden spiral constant from φ**: `GROWTH_GOLDEN = 2*log(PHI)/M_PI`. This is derived from the equiangular property: the constant crossing angle α satisfies `cot(α) = growth`, and for the golden ratio `α = arctan(2π/ln(φ)) ≈ 17.1°`. The formula `2ln(φ)/π` is a compact way to encode this — worth knowing for any logarithmic spiral work.
- **Cursor sets r₀ and θ₀**: The spiral starts at the cursor, making placement intuitive. Moving the cursor repositions the starting point of the next draw.

### Open Questions

- Can you draw multiple interleaved spirals (N arms) from the same starting point? (Yes — run the placement N times with θ₀ offset by `2π/N` each time.)
- Why does Archimedean pitch feel "correct" for even coverage but logarithmic growth gives a more organic look?

---

## From the Source

```c
static void spiral_place_archim(ObjPool *pool, double r0, double theta0,
                                 double pitch, int n_turns, double density,
                                 int rows, int cols, int ox, int oy)
{
    double a = pitch / (2.0 * M_PI);
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r = r0 + a * t;
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}

static void spiral_place_log(ObjPool *pool, double r0, double theta0,
                              double growth, int n_turns, double density,
                              int rows, int cols, int ox, int oy)
{
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = r0 * exp(growth * t);
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}
```

---

## Pass 2 — After Running It

### Pseudocode

```
mode = ARCHIM, pitch = PITCH_DEFAULT, n_turns = 3, density = 0.08
loop:
  key = getch()
  if arrows: move cursor; sync polar
  if 'l': mode = ARCHIM
  if 'o': mode = LOG
  if 'd': draw spiral from (cur_r, cur_theta)
  if '+'/'-': adjust pitch (ARCHIM) or growth (LOG)
  if '['/']': adjust n_turns
  if ','/'.': adjust density
  draw: bg → objects → cursor → HUD
```

### Module Map

| Function | Role |
|----------|------|
| `spiral_place_archim` | Walk Archimedean curve, place objects |
| `spiral_place_log` | Walk log-spiral curve, place objects |
| `polar_to_screen` | Convert (r, θ) → (col, row) |
| `pool_place` | Add object if in bounds and pool not full |

### Data Flow

```
'd' key → spiral_place_*(pool, cur_r, cur_theta, params)
  → for each t: compute (r, θ) → polar_to_screen → pool_place
pool_draw → renders all accumulated objects
parameter keys → adjust pitch/growth/turns/density for next draw
```
