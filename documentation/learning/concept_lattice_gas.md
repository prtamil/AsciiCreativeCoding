# Pass 1 — lattice_gas.c: FHP-I Lattice Gas Automaton

## Core Idea

A hexagonal grid where each cell holds 0–6 particles, one per direction, encoded as bits of a `uint8_t`. Every tick, two local operations are applied to all cells simultaneously:

1. **Collision** — particles in the same cell interact via a lookup table that conserves particle count (mass) and momentum, but may change individual directions
2. **Streaming** — each particle moves to the adjacent cell in its direction

From these two rules, the macroscopic behaviour of the fluid (density, velocity, pressure) obeys the **Navier-Stokes equations** at large scales.

Reference: Frisch, Hasslacher & Pomeau, *Physical Review Letters* 56(14):1505–1508, 1986.

## From the Source

**Algorithm:** FHP-I Lattice Gas Automaton. Each cell holds 6 bits — one per hexagonal lattice direction (0–5). Per tick: (1) COLLISION — particles in same cell swap via lookup table conserving mass and momentum; (2) STREAMING — each particle hops one cell in its direction. One lookup per cell; very cache-friendly since each cell is a single byte.

**Physics/References:** The H-theorem guarantees that many FHP particles, averaged over many cells, obey the Navier-Stokes equations. This is a proof that macroscopic fluid behaviour arises purely from microscopic conservation laws — no explicit PDEs. The hexagonal grid removes velocity-space anisotropy that afflicted earlier square-lattice gas models.

**Rendering:** 3×3 spatial averaging before display. Individual cells are binary (particle or no particle) — too noisy to show macroscopic flow. Averaging over 9 cells gives local particle density ρ and mean momentum ⟨p⟩, which map to color (velocity direction) and character (density level) for smooth visualisation.

**Performance:** O(W×H) per step — one lookup per cell. The collision table is precomputed at init.

## The Mental Model

### Why hex, not square?

On a square grid with 4 directions (N,S,E,W), the momentum flux tensor has different coefficients for horizontal and vertical flow. This anisotropy means the emergent fluid equations are also anisotropic — you get different viscosities in different directions. This is unphysical.

On a hexagonal grid with 6 directions (60° apart), the momentum flux tensor is isotropic. The lattice has the minimum rotational symmetry needed for the Navier-Stokes equations to hold. This is why Frisch, Hasslacher and Pomeau chose the hex grid.

### Why collision conserving mass and momentum is enough

At any one cell, the particles interact and change directions. The rule is designed so that:
1. **Mass conservation:** the number of particles before = number after (no creation or annihilation)
2. **Momentum conservation:** the sum of direction unit vectors before = sum after

These two local conservation laws are sufficient for the macroscopic equations to be continuity + Navier-Stokes. The viscosity, pressure, and flow speed emerge from the statistics — you don't set them directly.

### The two non-trivial collision patterns (FHP-I)

Only 5 of the 64 possible 6-bit states produce a non-identity collision:

**Head-on 2-particle (ambiguous):**
```
0x09 = bit0+bit3 = E+W   → NE+SW (0x12) or NW+SE (0x24)
0x12 = bit1+bit4 = NE+SW → NW+SE (0x24) or E+W   (0x09)
0x24 = bit2+bit5 = NW+SE → E+W   (0x09) or NE+SW (0x12)
```
Each head-on pair has two equally valid post-collision states (rotate CW or CCW). Both conserve mass and momentum. The choice is arbitrary — but it must be made consistently to avoid introducing chirality.

**3-particle symmetric (unique):**
```
0x15 = E+NW+SW (3 directions 120° apart) → NE+W+SE (0x2A)
0x2A = NE+W+SE (3 directions 120° apart) → E+NW+SW (0x15)
```
These have only one possible post-collision state (rotating the triplet by 60° in either direction gives the same result because of the 120° symmetry).

All 59 remaining patterns are unchanged (identity).

### Why the spatial parity trick works for head-on pairs

If every cell always chose CW rotation for head-on pairs, the simulation would develop a global CCW spin — the fluid would spontaneously rotate. This violates isotropy.

The fix: resolve the ambiguity by `(row+col)&1`. Even cells use CCW, odd cells use CW. On a checkerboard pattern, every CW rotation at one cell is adjacent to a CCW rotation. The local chiralities cancel statistically, and the bulk fluid has zero net rotation.

This is not a physical assumption — it is purely a numerical trick to prevent a lattice artifact from appearing in the simulation.

### Streaming: why the hex offset matters

On screen, the grid is displayed as a rectangular array. But the connectivity is hexagonal: each cell has 6 neighbours, not 4. The neighbour offsets depend on whether the row is even or odd (the "offset row" layout):

```
Even row: E=(0,+1) NE=(-1,0)  NW=(-1,-1) W=(0,-1) SW=(+1,-1) SE=(+1,0)
Odd row:  E=(0,+1) NE=(-1,+1) NW=(-1,0)  W=(0,-1) SW=(+1,0)  SE=(+1,+1)
```

The row-parity offset is essential. Without it, the streaming phase would connect wrong neighbours and violate the hexagonal topology, producing incorrect momentum flux.

### Bounce-back walls

When a particle reaches a wall cell, it reverses direction (`(d+3)%6`) and stays at the source cell. This is the **no-slip boundary condition** — wall cells reflect all momentum back, equivalent to the fluid having zero velocity at the wall. The bounce-back rule is the correct physical boundary condition for incompressible flow in lattice methods.

### Emergent viscosity

The kinematic viscosity ν of the FHP-I fluid depends on the mean occupation d per direction:
```
ν ≈ 1 / (6·d·(1−d))  − 1/8
```
At d=0.18 (default), ν ≈ 0.8. Higher density → lower viscosity (more particle-particle interactions damp momentum differences faster). The Reynolds number Re = U·L/ν determines whether flow is laminar or turbulent.

## Data Structures

### Bit-packed cell (§5)
```c
uint8_t g_grid[ROWS_MAX][COLS_MAX]   /* bits 0..5: particle in dir 0..5 */
uint8_t g_buf [ROWS_MAX][COLS_MAX]   /* streaming write buffer */
bool    g_wall[ROWS_MAX][COLS_MAX]   /* solid obstacle map */
```

### Collision table (§4)
```c
uint8_t g_col[2][64]   /* [parity][6-bit state] → post-collision 6-bit state */
```
Built once at startup by `build_collision_tables()`. All 64 entries initialised to identity; 5 entries overridden for the non-trivial collisions.

### Hex direction tables (§4)
```c
int DIR_DR[2][6]   /* row offset for each direction in even/odd rows */
int DIR_DC[2][6]   /* col offset for each direction in even/odd rows */
```

## The Main Loop

1. `inject_inlet()` — for each non-wall cell in column 0, set bit 0 (East) with probability `g_inlet_prob`
2. `grid_collide()` — for each non-wall cell: `g_grid[r][c] = g_col[(r+c)&1][g_grid[r][c] & 0x3F]`
3. `grid_stream()` — for each non-wall cell, for each set bit d: compute hex neighbour `(nr,nc)`; if wall: set `g_buf[r][c] |= (1<<(d+3)%6)`; else: `g_buf[nr][nc] |= (1<<d)`; then `memcpy(g_grid, g_buf)`

## Non-Obvious Decisions

### Collision before streaming, not after

The order matters. Doing collision first, then streaming, corresponds to the standard FHP update. Swapping the order (stream then collide) produces a different numerical scheme — not wrong, but it changes the effective viscosity and time-step semantics. The rule is: particles interact at their current position first, then move.

### Initialising g_col to identity for all 64 entries

`for (int s = 0; s < 64; s++) g_col[0][s] = g_col[1][s] = s;`

This means all the uninteresting states (0 particles, 1 particle, asymmetric 2-particle encounters, 4-5-6 particle states) pass through unchanged without any branching in the hot loop. The lookup table handles all cases with a single array access.

### `__builtin_popcount` for density display

The particle count per cell is `__builtin_popcount((unsigned)s)` — one instruction on x86. This is called for every visible cell every frame. Using a loop of 6 bit-checks would be 6× slower.

### Horizontal momentum ×2 (integer display)

The x-component of hex direction unit vectors:
`E=+1, NE=+0.5, NW=-0.5, W=-1, SW=-0.5, SE=+0.5`

Multiply all by 2 to get integers: `E=+2, NE=+1, NW=-1, W=-2, SW=-1, SE=+1`. Then:
```c
int mx2 = 2*((s>>0)&1) + ((s>>1)&1) - ((s>>2)&1) - 2*((s>>3)&1) - ((s>>4)&1) + ((s>>5)&1);
```
Range: −4 to +4. This maps to 5 colour pairs without any floating-point arithmetic per cell.

### Aspect ratio correction for cylinder

Terminal cells are 2× taller than wide (CELL_H=16, CELL_W=8). A circular obstacle in grid coordinates would appear as an ellipse on screen (elongated vertically). The fix:

```c
float dx = (float)(c - cx);
float dy = (float)(r - cy) * ASPECT_R;   /* ASPECT_R = 2.0 = CELL_H/CELL_W */
if (dx*dx + dy*dy < (float)(R*R)) ...
```

This creates a grid ellipse with vertical semi-axis = R/2 rows, which appears as a circle (R columns × R rows in pixel space) on screen.

### Toroidal boundary + inlet

Rather than open inflow/outflow boundaries (which require special logic), the grid wraps toroidally. The inlet continuously forces East-facing particles at the left column. Particles that exit the right column re-enter the left column — but immediately mix with fresh inlet particles. After ~100 ticks, the flow reaches a quasi-steady state driven entirely by the inlet.

## Preset Designs

| # | Name | Structure | What to observe |
|---|---|---|---|
| 0 | Cylinder | Circular obstacle at 2/5 from left | Deflection, pressure build-up on upstream face, low-density wake |
| 1 | DoubleSlit | Vertical barrier at 1/3, two gaps | Spreading after slits (classical diffraction, not quantum) |
| 2 | Channel | Solid top/bottom walls, open sides | Poiseuille-like parabolic velocity profile develops |
| 3 | Free | No walls, 40% random initial | Watch equilibration toward Maxwell-Boltzmann distribution |

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `INLET_PROB_DEF` | 0.55 | Higher → stronger flow, higher Re, possibly turbulent wake |
| `STEPS_DEF` | 6 | More steps/frame → faster time evolution per second of wall time |
| `ASPECT_R` | 2.0 | Must equal CELL_H/CELL_W; wrong value → oval cylinder |
| `d` (init density) | 0.18/direction | Higher → lower viscosity, higher Re |

## Themes

5 themes cycle with `t`/`T` — each assigns 256-color foreground values for the 5 momentum levels:

| # | Name | West (cold) | East (warm) | Effect |
|---|---|---|---|---|
| 0 | Flow | Blue→cyan | Orange→red | Classic velocity colormap |
| 1 | Ocean | Navy→white | White→white | Depth-like gradient |
| 2 | Plasma | Purple→pink | Yellow→white | High contrast |
| 3 | Matrix | Dark green→bright | Green | Monochromatic |
| 4 | Heat | Dark red→orange | Gold→yellow | Thermal map |

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_grid[ROWS_MAX][COLS_MAX]` | `uint8_t[128][512]` | 64 KB | 6-bit particle state per cell (one bit per hex direction) |
| `g_buf[ROWS_MAX][COLS_MAX]` | `uint8_t[128][512]` | 64 KB | streaming double-buffer; written during stream, swapped after |
| `g_wall[ROWS_MAX][COLS_MAX]` | `bool[128][512]` | 64 KB | solid obstacle mask; walls bounce particles back |
| `g_col[2][64]` | `uint8_t[2][64]` | 128 B | precomputed FHP-I collision lookup tables (parity 0 and 1) |

---

# Pass 2 — lattice_gas: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | ROWS_MAX, COLS_MAX, direction bits, STEPS_DEF, INLET_PROB_DEF |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5 themes × 7 colour pairs (5 momentum + wall + HUD) |
| §4 hex | `DIR_DR[2][6]`, `DIR_DC[2][6]`, `g_col[2][64]`, `build_collision_tables()`, `cell_mx2()`, `mx2_to_cp()` |
| §5 grid | `g_grid`, `g_buf`, `g_wall`, `inject_inlet()`, `grid_collide()`, `grid_stream()`, `grid_step()`, `scene_init()` |
| §6 draw | `scene_draw()` |
| §7 screen | ncurses init, `screen_resize()` |
| §8 app | main loop, input, signal handlers |

## Data Flow

```
build_collision_tables():
  for s in 0..63: g_col[0][s] = g_col[1][s] = s   (identity)
  /* head-on 2-particle: CCW parity 0, CW parity 1 */
  g_col[0][0x09] = 0x12;  g_col[1][0x09] = 0x24;
  g_col[0][0x12] = 0x24;  g_col[1][0x12] = 0x09;
  g_col[0][0x24] = 0x09;  g_col[1][0x24] = 0x12;
  /* 3-particle symmetric: unique */
  g_col[*][0x15] = 0x2A;  g_col[*][0x2A] = 0x15;

grid_step():
  inject_inlet():
    for each non-wall r in col 0:
      if rng_float() < g_inlet_prob: g_grid[r][0] |= 0x01 (East bit)

  grid_collide():
    for each non-wall cell (r,c):
      par = (r+c) & 1
      g_grid[r][c] = g_col[par][ g_grid[r][c] & 0x3F ]

  grid_stream():
    memset(g_buf, 0)
    for each non-wall cell (r,c) with non-zero state:
      par = r & 1
      for d in 0..5:
        if bit d set:
          nr = (r + DIR_DR[par][d] + g_rows) % g_rows
          nc = (c + DIR_DC[par][d] + g_cols) % g_cols
          if g_wall[nr][nc]:
            g_buf[r][c] |= (1 << (d+3)%6)   // bounce back
          else:
            g_buf[nr][nc] |= (1 << d)         // stream
    memcpy(g_grid, g_buf)

scene_draw():
  for each cell (r,c):
    if wall: draw '#' CP_WALL
    else:
      n = popcount(g_grid[r][c])
      if n > 0:
        cp = mx2_to_cp(cell_mx2(g_grid[r][c]))
        draw k_dens[n] with COLOR_PAIR(cp), A_BOLD if n>=4
  draw HUD
```

## Open Questions for Pass 3

- Does the flow past cylinder develop a **Kármán vortex street** at higher inlet densities? What inlet_prob produces visible oscillating wake?
- The FHP model has a known **spurious invariant**: the checkerboard density (sum over even cells minus sum over odd cells) is conserved. Does this create visible artifacts at long run times?
- The kinematic viscosity ν ≈ 1/(6·d·(1-d)) − 1/8. Measure the actual viscosity by fitting the channel velocity profile to a parabola and computing ν = U_max·h²/(2·dP/dx). Does it match theory?
- For the double-slit preset: do the two wakes from the slits create an interference-like fringe pattern? (This is classical diffraction, not quantum — it should appear.)
- FHP-II and FHP-III add additional collision rules (4-particle collisions, rest particles) that reduce viscosity. What is the minimum viscosity achievable with FHP-I?
- What is the effective Reynolds number for the cylinder preset at default settings? Re = U·R/ν where U ≈ inlet_prob/6 and R is the cylinder radius in cells.
