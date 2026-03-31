# Pass 1 — fireworks: Rocket Fireworks with State Machine

## Core Idea

Each rocket is an independent state machine that cycles through three states: waiting to launch (IDLE), rising toward its apex (RISING), then exploding into a burst of particles (EXPLODED). Multiple rockets run in parallel. The Show struct owns a pool of rocket slots; each rocket owns its own particle array.

---

## The Mental Model

Imagine a pool of `N` rockets. Each rocket:
1. **IDLE**: waits for a fuse countdown. Staggered initial fuses prevent all rockets from launching simultaneously.
2. **RISING**: moves upward each tick, with gravity slowing it. When vertical velocity crosses zero (apex reached), it explodes — spawning 80 particles in all directions.
3. **EXPLODED**: waits for all particles to die. Each particle is drawn with brightness based on its remaining life (bold = fresh, dim = fading). When all particles dead, sets a new random fuse and returns to IDLE.

The rocket is drawn as `|` with an `'` exhaust trail below it. Particles are drawn as random ASCII symbols (`*+.,` etc.), each with a random color.

---

## Data Structures

### `Particle` struct
| Field | Meaning |
|---|---|
| `x, y` | Current position in terminal columns/rows |
| `vx, vy` | Velocity; vy increased by GRAVITY each tick |
| `life` | 0.6–1.0 at birth, decremented by decay each tick |
| `decay` | How fast life drains (random per particle) |
| `symbol` | Random ASCII character from k_particle_symbols |
| `color` | Random ColorID (each particle has its own color) |
| `active` | False when life <= 0 |

### `Rocket` struct
| Field | Meaning |
|---|---|
| `x, y` | Position in terminal coordinates |
| `vy` | Vertical velocity (negative = upward) |
| `color` | Color for the rocket body |
| `state` | RS_IDLE / RS_RISING / RS_EXPLODED |
| `fuse` | Countdown ticks until next launch from IDLE |
| `particles[80]` | Fixed array — no heap allocation |

### `Show` struct
| Field | Meaning |
|---|---|
| `rockets[MAX_ROCKETS]` | Fixed pool of all rocket slots |
| `active_rockets` | How many slots are actually ticked |

---

## The Main Loop

Standard fixed-timestep:
1. `dt` → `sim_accum`
2. While `sim_accum >= tick_ns`: `show_tick()` (ticks each active rocket)
3. Render: `erase()` → `show_draw()` → HUD → `doupdate()`
4. Input: add/remove rockets, speed, quit

---

## Non-Obvious Decisions

### Staggered initial fuses
At init, rocket `i` gets `fuse = i * 8`. Without this, all rockets would launch at tick 0 — one big simultaneous blast instead of staggered shows.

### vy >= 0 as apex detection
Gravity adds to vy each tick. The rocket starts with negative vy (upward). When gravity has accumulated enough, vy crosses zero — this is exactly the apex. No need to track max height.

### Vertical velocity squash: `vy = sinf(angle) * speed * 0.5f`
The particle spread is a 2D circle in physics, but the terminal is non-square (cells are taller than wide). Squashing vy by 0.5 makes the burst look circular on screen.

### Multi-color bursts
Every particle gets an independent random color. This is different from having one color per rocket — the multi-color explosion looks richer.

### Brightness from life
`life > 0.6f → A_BOLD` (freshly born, glowing bright)
`life < 0.2f → A_DIM` (dying embers, fading out)
Middle range: normal brightness

### No heap allocation
Each rocket owns `particles[PARTICLES_PER_BURST]` directly. `Show` owns `rockets[MAX_ROCKETS]` directly. Everything is in the App struct on the (conceptual) stack. No malloc/free in the hot path.

---

## State Machine

```
IDLE ──(fuse reaches 0)──→ RISING
         ↑                     │
         │                     │ vy crosses 0 (apex) or y < 2
         │                     ↓
         └──(all particles dead) EXPLODED
                (set new random fuse)
```

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `PARTICLES_PER_BURST` | 80 | More sparks per explosion; heavier CPU |
| `ROCKETS_DEFAULT` | 6 | More simultaneous rockets |
| `LAUNCH_SPEED_MIN/MAX` | 3–8 rows/s | Higher = rockets reach greater heights |
| `GRAVITY` | 9.8 rows/s² | Higher = quicker apex, shorter flights |
| fuse range | 0.5–2.5s | Longer wait between launches |

---

## Open Questions for Pass 3

1. What happens if you set `GRAVITY = 0`? Particles would fly in straight lines.
2. Can you make all particles in one explosion share one color (matching the rocket) but vary in brightness?
3. What would the stagger look like with `fuse = rand() % 30` instead of `i * 8`?
4. How would you add a rocket that doesn't explode but instead transforms into a falling star?
5. The physics uses terminal rows/cols directly — no pixel space. How does this compare to bounce_ball's approach?
