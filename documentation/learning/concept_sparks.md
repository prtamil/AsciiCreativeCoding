# Pass 1 — sparks: Fast Bouncing Particles with Motion-Blur Trails

## Core Idea

Each spark is an independent fast-moving projectile that **remembers its last few positions** and **bounces elastically off the floor**. The screen renders the memory as a fading streak (motion-blur trail) behind a bright head, and gravity pulls the whole arc downward until restitution gradually drains the bounce energy. Distinct from `embers.c`: embers RISE slowly and have no trail or bounce; sparks SHOOT FAST, drag a streak, and bounce off the floor with a coefficient of restitution.

---

## The Mental Model

Imagine a tracer bullet on a 2-D billiard table that has only a floor (no side walls), pulled down by gravity. The bullet leaves a short, fading streak behind it. When it hits the floor it bounces back up, but only as high as `e²` of the previous height (for restitution `e`). After a few bounces it has so little vertical energy left that drag stops it. Sparks from a real welder behave exactly like this — which is why the visual reads correctly.

The four patterns are different (emitter, cone) configurations of the same physics:

| Pattern | Emitter | Cone direction | Speed | Gravity | Restitution |
|---|---|---|---|---|---|
| WELDING | left wall, mid-height | right (±0.55 rad) | 55–90 | 78 | 0.55 |
| GRINDER | bottom-left | up-right (-1.3 to -0.4 rad) | 72–110 | 92 | 0.50 |
| CAMPFIRE | bottom-centre | upward broad (-2.2 to -0.94 rad) | 34–56 | 36 | 0.40 |
| TESLA | screen centre | full circle (-π to π) | 55–85 | 26 | 0.65 |

---

## Data Structures

### `Spark` struct
| Field | Meaning |
|---|---|
| `x, y` | Current position (cells); +y is downward on screen |
| `vx, vy` | Velocity (cells/sec). Gravity adds positive vy each tick |
| `age` | Seconds since spawn |
| `life` | Seconds until death (random ∈ [life_min, life_max] at birth) |
| `trail_x[TRAIL_LEN]` | Ring of previous x positions, oldest at index 0 |
| `trail_y[TRAIL_LEN]` | Ring of previous y positions, oldest at index 0 |
| `active` | Pool-slot occupancy |

### `Scene` struct
| Field | Meaning |
|---|---|
| `sparks[MAX_SPARKS]` | Fixed pool, `MAX_SPARKS = 800` |
| `current_pattern` | `WELDING` / `GRINDER` / `CAMPFIRE` / `TESLA` |
| `current_theme` | Index into 8-theme table (heat ramps) |
| `emitter_offset_x` | User-shift of the emitter via `w/W` keys |
| `paused, speed, rng` | Standard scene controls |

### `PatternParams` struct
Pattern table (`pattern_params[N_PATTERNS]`) holds **physics + emission cone + visuals** as one row per pattern. Fields: `target_sparks`, `emitter` (anchor enum), `emit_x_jitter`, `emit_y_jitter`, `speed_min/max`, `angle_min/max` (radians), `gravity`, `drag_coeff`, `restitution`, `floor_friction`, `life_min/max`. Switching pattern is a single index change — physics adapts instantly.

---

## The Main Loop

Standard fixed-timestep, identical skeleton to `embers.c`:

1. `dt` → `sim_accum`
2. While `sim_accum >= tick_ns`: `scene_tick()`
   - Top up pool to `target_sparks` (with per-tick spawn cap)
   - For each active spark:
     1. Shift trail history (drop oldest, push current `(x, y)` as newest prev)
     2. Integrate: `vy += gravity·dt`, `v *= exp(−drag·dt)`, `x += vx·dt`, `y += vy·dt`, `age += dt`
     3. Floor bounce check
     4. Death checks (age >= life, off-screen, settled)
3. Render: `erase()` → trails (dim, all sparks) → heads (bright, all sparks) → HUD → `doupdate()`
4. Input: pattern, theme, speed, source-shift, pause, quit

---

## Non-Obvious Decisions

### Trail history initialised at spawn to current position
Without this, a fresh spark would draw a streak from a random previous occupant's last position the moment it was born. Filling all `TRAIL_LEN` slots with the spawn `(x, y)` makes the trail invisible (drawn on top of the head) for the first `TRAIL_LEN` ticks, then naturally stretches out as the spark moves.

### Trail-first / heads-second draw passes
If trails were drawn alongside heads, spark A's trail could overwrite spark B's head when they crossed. Two passes — all trails first, then all heads — guarantees the bright dot you actually track with your eye is never shadowed by a passing streak.

### `drag_factor = expf(−drag_coeff · dt)` (not `v *= 0.99`)
The exponential form is **frame-rate-independent** — applying it across two `dt/2` substeps is identical to applying it once across `dt`. The naïve `v *= 0.99` form gives different damping at different sim Hz, breaking the visual "feel" when the user changes `]/[`.

### Settle-kill after low-energy bounces
With finite restitution, a spark with very small `|vy|` would be re-bounced on every tick by gravity → tiny `vy` → bounce → repeat, infinitely. The kill check after bounce — `if |vy| < SETTLE_VY && |vx| < SETTLE_VX → active = false` — frees the slot the moment the spark has too little energy to make a visible bounce.

### Cone emission via polar `(angle, speed)`
Unlike embers (which sample velocity as "mostly upward + lateral jitter rectangle"), sparks sample `angle ∈ [angle_min, angle_max]` then `speed ∈ [speed_min, speed_max]` and decompose into `(vx, vy)`. Setting `angle_max − angle_min ≥ 2π` gives full omnidirectional emission for free — no special TESLA code path needed.

### Floor at `rows - 2`
The HUD lives on the bottom row (`rows - 1`); sparks bounce off the row above. This places visible bounces just above the HUD bar, framing the simulation in the top portion of the screen.

### Reflection formula `y' = 2·floor_y − y`
Standard mirror reflection about the floor line. After reflection the spark sits the same overshoot distance above the floor as it did below. Without this, a spark that crossed the floor by 1.3 cells in one tick would teleport back to exactly `floor_y`, losing the sub-tick energy that should carry it back upward.

### Two glyph ramps: `HEAD_GLYPHS` and `TRAIL_GLYPHS`
The head ramp uses dense punctuation (`*` `+` `#` `@`) to read as a hot point. The trail ramp uses sparse glyphs (`,` `.` `:`) to read as a fading streak BEHIND the head. Both indexed cool→hot so the same heat-ramp slot picks visually consistent colour + density.

---

## Bounce Physics

```
y'  = 2·floor_y − y                  // reflect about floor line
vy' = −vy · restitution              // flip + lose energy
vx' = vx · floor_friction            // tangential drag (per bounce)
```

Restitution `e` controls how high subsequent bounces go: bounce `n` reaches a height proportional to `e^(2n)` of the original. With `e = 0.55` (WELDING), the third bounce reaches ~9% of the initial height — visually it dies after 2–3 visible bounces.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `MAX_SPARKS` | 800 | More density; CPU cost grows linearly |
| `TRAIL_LEN` | 3 | Longer streak (CPU + render cost grow linearly) |
| `pattern.target_sparks` | 220–380 | Steady-state pool occupancy |
| `pattern.gravity` | 26–92 | Steeper arcs, faster floor contact |
| `pattern.restitution` | 0.40–0.65 | Higher bounces, longer-lived bounces |
| `pattern.drag_coeff` | 0.20–0.55 | Faster velocity decay; shorter trails |
| `SETTLE_VY` | 6.0 | Higher = sparks die earlier on floor |
| `pattern.angle_min/max` | varies | Wider cone = more spread, sparser per-direction |

---

## Open Questions for Pass 3

1. What if `restitution = 1.0`? Sparks would bounce forever — the settle-kill threshold becomes the only thing limiting their lifetime.
2. How would adding side walls (with their own restitution) change the visual? TESLA would become a "fully enclosed crackle" — sparks ricochet around the box.
3. Could a single spark fire its own miniature radial burst on impact? (Sub-spark recursion — closer to lightning.c branching.)
4. What replaces the trail history if `TRAIL_LEN = 0`? You get bare points at high speed — looks like rain, not sparks.
5. The drag is isotropic (same on `vx` and `vy`). Air resistance is more accurately quadratic in speed. What changes if `drag = drag_coeff · |v|`?

---

# Pass 2 — sparks: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | Pattern enum + `pattern_params[N_PATTERNS]`, theme heat ramps, `TRAIL_LEN`, `SETTLE_VY/VX`, `EMITTER_SHIFT_STEP`, `TICK_NS` |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` (`CLOCK_MONOTONIC`) |
| §3 color | 8-pair heat ramp, 256-colour with 8-colour fallback |
| §4 spark | `Spark` struct (incl. trail history) + LCG |
| §5 scene | Pool, `scene_emitter_xy()`, `scene_spawn_spark()`, `scene_tick()`, `scene_draw()` |
| §6 screen | ncurses init/draw/resize, HUD on bottom row |
| §7 app | `App`, signals, fixed-step main loop |

---

## Data Flow Diagram

```
scene_tick(dt):
  1. Top up pool:
       active = count of active sparks
       to_spawn = min(target − active, spawn_cap)
       for k = 0..to_spawn−1:
         scene_spawn_spark()
           idx = first inactive slot
           cx, cy = emitter_xy()
           angle = U(angle_min, angle_max)
           speed = U(speed_min, speed_max)
           vx, vy = speed·cos/sin(angle)
           x, y = cx ± jitter, cy ± jitter
           life = U(life_min, life_max)
           trail[k] = (x, y) for all k
           active = true

  2. For each active spark:
       shift trail: trail[k] = trail[k+1]; trail[LEN-1] = (x, y)
       vy += gravity·dt
       v  *= exp(−drag·dt)
       x  += vx·dt
       y  += vy·dt
       age += dt

       if y >= floor_y and vy > 0:
         y  = 2·floor_y − y
         vy = −vy · restitution
         vx *=  floor_friction
         if |vy| < SETTLE_VY and |vx| < SETTLE_VX:
           active = false
           continue

       if age >= life or off-screen:
         active = false

scene_draw():
  erase()

  // Pass 1: trails (dim, drawn first so heads can overwrite)
  for each active spark:
    head_slot = floor((1 − age/life) · 7.999)
    for k = 0..TRAIL_LEN−1:
      slot = head_slot − (TRAIL_LEN − k)
      if slot < 0: continue
      mvaddch(trail_y[k], trail_x[k], TRAIL_GLYPHS[slot])

  // Pass 2: heads (bright, drawn last so they sit on top)
  for each active spark:
    slot  = head_slot
    glyph = HEAD_GLYPHS[slot]
    attr  = A_BOLD if slot >= 6 else A_DIM if slot <= 1 else A_NORMAL
    mvaddch(y, x, glyph)

  HUD on bottom row
  wnoutrefresh + doupdate
```

---

## State Diagram (per spark)

```
LIVE (in flight)
  │
  ├── (age >= life)        ──→ DEAD
  ├── (off-screen)         ──→ DEAD
  ├── (settled at floor)   ──→ DEAD
  │
  └── (y crosses floor)    ──→ BOUNCE ──→ LIVE  (vy flipped, energy reduced)
```

No multi-state lifecycle (unlike `fireworks.c` rockets). Sparks are a one-shot LIVE → DEAD transition with bounces as in-place velocity edits.

---

## Function Breakdown

### `scene_emitter_xy(scene, *cx, *cy)`
Switch on `pattern.emitter` (`EMIT_LEFT_MID` / `EMIT_BOT_LEFT` / `EMIT_BOT_CENTER` / `EMIT_CENTER`) and write the screen-centred anchor + user shift. Each pattern has its own emitter so the four scenes look distinct.

### `scene_spawn_spark(scene)`
Find first inactive pool slot. Sample `(angle, speed)` from pattern cone; decompose to `(vx, vy)`. Position = emitter + jitter. Initialise all `TRAIL_LEN` slots to spawn position so the renderer never sees stale history. Set `age = 0`, `life = U(life_min, life_max)`, `active = true`.

### `scene_tick(scene, dt)`
Apply speed multiplier. Top up pool (with per-tick spawn cap so a long pause doesn't dump everything on resume). For each active spark: shift trail, semi-implicit Euler integrate, bounce check, death check.

### `scene_draw(scene)`
Two passes (trails first, heads second). For each spark, compute `head_slot` from remaining-life fraction; trail slots are `head_slot − (TRAIL_LEN − k)` so older history points use cooler ramp slots and fade behind the spark.

### `app_handle_key(app, ch)`
`q/ESC` quit; `space` pause; `r` reseed; `n/N` `p/P` pattern cycle; `t/T` theme cycle; `+/-` speed; `]/[` sim Hz; `w/W` emitter shift.

---

## Key Files

- `particle_systems/sparks.c` — implementation (~1100 lines)
- `particle_systems/embers.c` — sibling demo. Read first; sparks.c is the FAST-and-BOUNCY counterpart to embers' SLOW-and-BUOYANT.
- `particle_systems/fountain.c` — also gravity + bouncing particles.
- `particle_systems/fireworks.c` — TESLA's omnidirectional emission is closest to fireworks' radial burst.

---

## References

- Reeves, W. T. (1983). "Particle Systems: A Technique for Modelling a Class of Fuzzy Objects". *ACM TOG* 2(2):91–108. Foundational paper on stochastic particle systems.
- Wikipedia. [Coefficient of restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution). The `e` in `vy' = −e · vy` is exactly the COR.
- Millington, I. *Game Physics Engine Development*, §6 ("Particle Physics") and §7 ("Particle Contacts"). Plain-English derivation of Euler integration + impulse bounce.
- Bourke, P. [Character density ramps](http://paulbourke.net/dataformats/asciiart/). Source of the cool→hot glyph ramps used for trail/head.
