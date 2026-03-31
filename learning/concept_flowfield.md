# Pass 1 — flowfield: ASCII Flow Field Simulation

## Core Idea

A 2D vector field computed from layered Perlin noise drives hundreds of tracer particles that leave fading trails. The noise evolves slowly over time, so the field and the particle paths change continuously — producing motion that looks like wind, smoke, or ocean currents rendered in ASCII arrows and trail characters.

---

## The Mental Model

Imagine every terminal cell has a tiny arrow showing which direction the "wind" is blowing at that point. The arrows change slowly over time (the noise field evolves). Hundreds of tracer particles follow the wind, leaving fading trails behind them: `.,+~*` from dim-to-bright, with a directional arrow character at the head.

The result: a living, flowing, swirling pattern that looks organic because it is driven by multi-scale Perlin noise.

---

## Data Structures

### `Field` struct
| Field | Meaning |
|---|---|
| `angles[rows*cols]` | Flow angle in radians for each cell |
| `cols, rows` | Grid dimensions |
| `time` | Current noise time axis value |
| `speed` | How fast time advances per tick |

### `Particle` struct
| Field | Meaning |
|---|---|
| `x, y` | Continuous position (not snapped to cell) |
| `speed` | Per-particle speed in [0.5, 1.3] cells/tick |
| `angle` | Current movement direction (from field sample) |
| `trail_x[], trail_y[]` | Ring buffer of MAX_TRAIL=20 past positions |
| `head` | Ring buffer write index |
| `trail_fill` | How many trail slots are used (ramps up from 0) |
| `trail_len` | Active trail length (configurable, ≤ MAX_TRAIL) |
| `life` | Countdown to auto-respawn |
| `color_pair` | Color pair index 1..8 (updated each tick in rainbow mode) |
| `alive` | False = slot free for respawn |

### `Scene` struct
| Field | Meaning |
|---|---|
| `field` | The vector field |
| `pool[PARTICLES_MAX]` | All particle slots |
| `n_particles` | How many are active |
| `trail_len` | Current trail length setting |
| `theme` | 0=rainbow, 1=cyan, 2=green, 3=white |
| `show_field` | Toggle background arrow display |
| `paused` | Simulation paused flag |

---

## The Main Loop

Standard fixed-timestep accumulator:
1. `field_tick()` — advance noise time, recompute all cell angles from 3-octave Perlin
2. For each live particle: `particle_tick()` — sample field, move, wrap, age
3. Auto-respawn dead particles
4. `scene_draw()` — optionally draw background arrows, then all particle trails

---

## Non-Obvious Decisions

### 3-octave fBm instead of single Perlin
A single noise octave gives very smooth, large-scale flow — it looks bland. Three octaves (1.0 + 0.5 + 0.25 at frequencies 1×/2×/4×) add fine-detail swirls layered over the large-scale flow. This creates the "multi-scale wind" appearance.

### Two independent noise channels → `atan2`
The X and Y components of the flow vector come from two completely separate noise evaluations (spatially offset by 100.3/200.7). If you just used one noise value as an angle directly, the result would have a bias toward certain directions. Using `atan2(ny, nx)` ensures all directions are equally likely.

### Bilinear field sampling
Particles move at continuous floating-point positions. The field is on a discrete cell grid. Simple nearest-neighbor would cause particles to jump as they cross cell boundaries. Bilinear interpolation between the 4 surrounding cells gives smooth continuous direction.

### Ring buffer trail + proportional ramp
The trail uses a circular buffer (head index advances, wraps at trail_len). The visual ramp `.,+~*` is mapped **proportionally** across the actual filled length — a 3-slot trail and a 20-slot trail both show the full gradient. The old naive approach (raw index into ramp) made long trails look mostly dim, like a caterpillar.

### Per-particle speed jitter
All particles at the same speed would synchronize — they'd pile up and thin out in waves. Each particle gets a random speed in [0.5, 1.3]. This prevents lock-step behavior and makes the flow look natural.

### Toroidal wrapping
Particles wrap around screen edges (left↔right, top↔bottom) rather than bouncing or dying at the boundary. This makes the flow look infinite and avoids the density thinning you'd get near walls.

### Direction-aware head character
The head of each trail shows an arrow matching the particle's current movement direction (`- / | \`). This makes each particle visually read as "going somewhere" rather than being a generic dot.

---

## State Machines

No state machine. Continuous simulation. Only boolean: `paused`. Dead particles auto-respawn immediately at a random position.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `PARTICLES_DEFAULT` | 300 | More streaks, denser field coverage |
| `TRAIL_LEN_DEFAULT` | 14 | Longer streaks per particle |
| `FIELD_SPD_DEFAULT` | 0.008 | Faster field evolution → more turbulent motion |
| `PARTICLE_SPEED` | 0.9 | Faster particles → longer streaks per second |
| `NOISE_SCALE_X/Y` | 0.04/0.07 | Smaller = smoother large-scale flow |
| `LIFE_BASE` | 100 | Longer particle lifetime before respawn |

---

## Open Questions for Pass 3

1. Remove bilinear sampling — use nearest-neighbor — does it look choppy?
2. Change wrap to bounce — how does particle behavior change at edges?
3. What happens if you use only 1 octave of noise vs 3 octaves? (add octaves one at a time)
4. What if all particles had the same speed? Does lock-step actually appear?
5. Visualize the field arrows while particles move — does the flow direction match the particle paths?
6. Try making field speed proportional to local noise gradient — faster flow in high-gradient zones.
