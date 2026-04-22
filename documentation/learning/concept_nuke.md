# Pass 1 — physics/nuke.c: 2-D Shockwave Propagation

## From the Source

**Algorithm:** Discrete 5-point Laplacian on a 2-D pressure field, second-order
in space and time, integrated as a leapfrog so cell `u` at the next tick depends
only on the current and previous tick. Two substeps per frame let the wave speed
go above the single-step CFL ceiling without losing stability.

**Physics/References:** Damped scalar wave equation
`∂²u/∂t² = c² ∇²u − γ ∂u/∂t`. Initial condition: tight Gaussian "tap" centred
just above the terrain. Energy decays via two independent mechanisms — geometric
spreading (`amp ∝ 1/√r` in 2-D cylindrical geometry) plus exponential damping
(`exp(−γ t)`).

**CFL stability:** `c · dt · √2 ≤ 1`. With 2 substeps per 60 Hz frame
(effective `dt = 1/120 s`), the ceiling is `c ≤ 84` cells/s. Default `c = 28`
gives CFL ≈ 0.33 — well inside the safe envelope.

---

## Core Idea

A blast detonates above a heightmap terrain. The pressure wave radiates outward
as a circular ring; positive peak in front, rarefaction trough behind — the
classic N-wave shape of a real shock. The same field drives every visual:

- **Wave glow** — the field itself, mapped to chars by `|u|` and to colour pairs
  by sign (positive overpressure vs. negative rarefaction).
- **Terrain ripples** — the wave value sampled at each terrain-surface row gets
  injected into a per-column displacement that decays with a spring restoring
  force, giving heave-and-settle.
- **Debris** — radial particle burst from the blast point, gravity pulls down.
- **Dust** — slow ground particles spawned as the ring sweeps past each
  column's terrain row.
- **Screen shake** — integer cell offset applied at the `mvaddch` boundary,
  amplitude `SHAKE_AMP × exp(−SHAKE_DECAY × t) × sin(2π SHAKE_FREQ t)`.
- **Flash** — full-screen white that fades over ~0.2 s.

---

## The Wave Step

```
u_new[y][x] = 2 u[y][x] − u_old[y][x]
            + (c·dt)² · ( u[y][x+1] + u[y][x−1] + u[y+1][x] + u[y−1][x] − 4 u[y][x] )
            − γ · dt · ( u[y][x] − u_old[y][x] )
```

This is the standard explicit second-order leapfrog: only three buffers are
needed, rotated each substep. The damping term `γ · dt · (u − u_old)` is an
implicit-velocity backward Euler approximation — stable at any γ.

---

## Energy Decay — Two Mechanisms

| Mechanism | Formula | Source |
|---|---|---|
| Geometric (spreading) | amp ∝ 1/√r | 2-D cylindrical geometry — total energy in the ring is conserved while the circumference grows |
| Physical (damping) | amp ∝ exp(−γ t) | γ converts wave energy to heat |

A ring 30 cells out has roughly 40 % of its initial peak amplitude under the
default constants.

---

## Terrain Ripple Coupling

`terrain_d[col]` is a per-column displacement that the rendered terrain row uses
as a vertical offset. Two forces drive it:

```
terrain_d[col] += k_inject · u[surface_row][col] · dt
terrain_d[col] *= decay                         /* spring restoring force */
```

The decay coefficient is chosen so the ripple settles in ≈ 1.5 s. The result is
a brief upward heave under the wave peak, a downward trough as the rarefaction
passes, then slow oscillation back to rest.

---

## Particle System

| Type | Spawn | Forces |
|---|---|---|
| DEBRIS | At blast centre, radial velocity from `ANG_RAND × R_RAND` | Gravity downward, no drag |
| DUST | At `(col, surface_row)` when ring passes, drifting laterally + slow upward | Slow gravity, lifetime fade |

A single `Particle` struct serves both — `type` selects the integration branch.

---

## Screen Shake

Integer cell offset applied at the rendering boundary only:

```
shake_amp = SHAKE_AMP × exp(−SHAKE_DECAY × t_since_blast)
shake_r   = (int)( shake_amp × sin(2π SHAKE_FREQ t) )
shake_c   = (int)( shake_amp × cos(2π SHAKE_FREQ t) )
mvaddch(row + shake_r, col + shake_c, ...)
```

The simulation never sees the shake — physics is on the unshaken grid.

---

## Themes (6 total)

Each theme is a triple: wave colour ramp + terrain colour + particle palette.
`t` cycles, `init_pair()` re-binds the same colour-pair indices each switch so
the renderer never branches on theme.

| # | Name | Wave | Mood |
|---|---|---|---|
| 0 | Atomic | white → yellow → orange → red | classic mushroom sky |
| 1 | Plasma | cyan → magenta → purple | high-energy pulse |
| 2 | Inferno | red → orange → black | hellfire |
| 3 | Acid | lime → green → forest | toxic sky |
| 4 | Arctic | white → ice → steel-blue | cold detonation |
| 5 | Mono | white → grey → black | clean greyscale |

---

## Non-Obvious Decisions

### Why two substeps per frame instead of one larger `c`?

CFL caps `c · dt` at `1/√2`. Halving `dt` (two substeps) doubles the maximum
sustainable `c` without changing visual smoothness, since rendering is still
once per frame.

### Why a Gaussian tap, not a delta?

A pixel-sharp delta excites every wavenumber up to Nyquist, including the
unstable ones. A Gaussian tap of half-width 2 cells filters out the
high-frequency content the discrete Laplacian cannot represent.

### Why integer-cell shake instead of sub-pixel jitter?

ncurses cannot draw sub-cell — fractional offsets quantize anyway. Integer
shake at the `mvaddch` site is one add per character with zero artifacts.

### Why decay terrain ripples with a spring, not free oscillation?

A free spring would oscillate forever. The exponential decay envelope models
internal damping in the soil layer and gives the visual a clear "settle to
quiet" endpoint without an explicit timer.

---

## Open Questions for Pass 3

1. Replace the 5-point Laplacian with a 9-point isotropic stencil. Does the
   ring become noticeably more circular at low resolution?
2. Add reflective walls at the screen edges (current behaviour: open boundary,
   wave dies at the border). What pattern emerges for repeated detonations?
3. Couple the ash particles to the wave field — let them be advected by the
   pressure gradient, not just gravity. Does this look like a real fireball?
4. Replace the screen-space shake with a real camera shake (offset every
   simulation read). Is the difference visible at 60 Hz?

---

# Pass 2 — physics/nuke.c: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | grid size, c, γ, blast amp, particle counts, theme palettes |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 6 themes × 8 pairs; `theme_apply()` rebinds pairs |
| §4 wave | three buffers (u_old/u/u_new), `wave_step()` (leapfrog) |
| §5 terrain | heightmap generator, per-column ripple displacement |
| §6 particles | debris + dust pools, sharing one Particle struct |
| §7 simulation | `SimState`, `blast_trigger()`, `sim_tick()` |
| §8 render | wave + terrain + particles + flash + shake → ncurses |
| §9 screen | ncurses init/teardown/resize |
| §10 app | main loop |

## Data Flow

```
sim_tick(dt):
  for n in 0..SUBSTEPS:
    wave_step(u_old, u, u_new, c, dt/SUBSTEPS, γ)
    rotate buffers
    for col in 0..cols:
      terrain_d[col] += K_INJECT · u[surface_row(col)][col] · dt
      terrain_d[col] *= DECAY
    spawn dust where ring sweeps surface
  particles_tick(dt)
  shake_amp = SHAKE_AMP · exp(−SHAKE_DECAY · t)
  flash *= FLASH_DECAY

render():
  for each (row, col):
    ch, pair = wave_to_glyph(u[row][col])
    mvaddch(row + shake_r, col + shake_c, ch | pair)
  draw_terrain(terrain_d)
  draw_particles()
  if flash > 0: overlay white at flash alpha
```
