# Pass 1 — gear.c: Wireframe rotating gear with themed spark emission

## Core Idea

A single gear rotates at the center of the screen, drawn entirely as a wireframe using ASCII line characters. Every tooth tip continuously emits sparks that carry the gear's tangential surface velocity — so at high speed the sparks don't just fly radially outward but sweep in large arcs that follow the gear's rotation direction. Sparks cool through a 7-stage color gradient from white-hot to ember. Ten named color themes change both the gear wireframe color and the entire spark palette simultaneously.

## The Mental Model

Think of an angle grinder or a lathe cutting metal. At low speed, chips fall nearly straight down with a small radial kick. At high speed, the surface is moving so fast that chips carry huge tangential momentum — they fly off in dramatic sweeping arcs that wrap around the tool. The terminal gear replicates this: `tang_vx = -sin(θ) × ω × R × TANG_SCALE`. At `rot_speed = 20 rad/s`, the surface velocity dominates over the radial kick, and sparks spiral dramatically away.

The wireframe is drawn by scanning every terminal cell in the gear's bounding box and testing proximity to three geometric features:
- **Circular arcs** (hub ring, inner arc, outer tooth arc) — tangential character `line_char(ang + π/2)`
- **Radial edges** (spokes, tooth sides) — radial character `line_char(ang)`

No rasterization, no polygon fill — pure distance-to-curve proximity tests.

## Data Structures

### Spark
```
px, py   — position in pixel space
vx, vy   — velocity (px/s)
life     — 1.0 (born) → 0.0 (dead); drives cooling stage
```

### Gear
```
cx, cy         — center position
angle          — current rotation angle (radians)
rot_speed      — angular velocity (rad/s), user-controlled
spark_density  — emission rate multiplier (user-controlled)
sparks[]       — pool of MAX_SPARKS=1500 Spark structs
emit_acc       — fractional spark accumulator
```

### Theme
```
name           — display string ("FIRE", "MATRIX", ...)
gear_fg        — 256-color index for bright wireframe
gear_dim_fg    — 256-color index for dim wireframe
spark_fg[7]    — color per cooling stage, freshest→dead
spark_ch[7]    — ASCII char per stage
spark_at[7]    — bold/normal/dim per stage (0/1/2)
```

## Wireframe Edge Detection

Each terminal cell center `(c×CELL_W + 4, r×CELL_H + 8)` is tested against the gear geometry in local polar coordinates `(rad, ang_local)`:

**Hub ring:** `|rad - R_HUB| < THRESH_CIRC` → tangential char
**Spokes:** `rad ∈ (R_HUB, R_INNER)` and `arc_spoke < THRESH_SPOKE` → radial char
**Tooth sides:** `rad ∈ (R_INNER, R_OUTER)` and `arc_side < THRESH_SIDE` → radial char
**Inner arc:** `!in_tooth` and `|rad - R_INNER| < THRESH_CIRC` → tangential char (dim)
**Outer arc:** `in_tooth` and `|rad - R_OUTER| < THRESH_CIRC` → tangential char

Where `in_tooth = (phase < TOOTH_DUTY)`, `phase = fmod(ang_local × N_TEETH / 2π, 1)`.

**`line_char(ang)` maps angle to ASCII:**
```
ang ≈ 0     → '-'   (rightward)
ang ≈ π/4   → '\'
ang ≈ π/2   → '|'   (downward)
ang ≈ 3π/4  → '/'
```
For tangential edges, `ang + π/2` is passed so the char runs along the circle, not radially.

## Spark Physics

**Emission:**
```
total_rate = SPARK_BASE_RATE × (rot_speed / ROT_BASE) × spark_density
```
Each tick, `emit_acc += total_rate × dt`; one spark fires per integer crossed. The emitting tooth is chosen randomly from all N_TEETH — uniform all-around coverage.

**Velocity at birth:**
```
tang_vx = -sin(tip_ang) × rot_speed × R_OUTER × TANG_SCALE
tang_vy =  cos(tip_ang) × rot_speed × R_OUTER × TANG_SCALE
kick    = random in [KICK_MIN, KICK_MAX]  (radial outward)
scatter = random × SCATTER               (uniform spread)
vx = tang_vx + cos(tip_ang)×kick + scatter_x
vy = tang_vy + sin(tip_ang)×kick + scatter_y
```

**Per-tick integration:**
```
vy += GRAVITY × dt                        (downward pull)
vx += random × TURB × dt                 (flame wiggle)
vy += random × TURB × 0.4 × dt
vx *= exp(-DRAG × dt)                     (exponential drag — exact solution)
vy *= exp(-DRAG × dt)
px += vx × dt
py += vy × dt
life -= dt / SPARK_LIFE
```

## The 7-Stage Cooling Gradient

Life runs from 1.0 (born) to 0.0 (dead). Stage thresholds: `0.85, 0.70, 0.55, 0.38, 0.22, 0.10, 0.00`.

Each theme maps these 7 stages to its own color, character, and brightness. The FIRE theme is:

| Stage | Life | Char | Color | Description |
|-------|------|------|-------|-------------|
| 0 | >0.85 | `*` | 231 white | white-hot |
| 1 | >0.70 | `*` | 226 yellow | very hot |
| 2 | >0.55 | `+` | 220 amber | hot |
| 3 | >0.38 | `+` | 214 orange | warm |
| 4 | >0.22 | `.` | 202 red-orange | cooling |
| 5 | >0.10 | `.` | 196 red | cool |
| 6 | ≥0.00 | `,` | 160 ember | nearly dead |

## The 10 Themes

| # | Name | Gear color | Spark palette | Character style |
|---|------|-----------|---------------|-----------------|
| 0 | FIRE | Steel blue 153 | white→yellow→amber→orange→red→ember | `* * + + . . ,` |
| 1 | MATRIX | Dark green 34 | white→lime→green→mid→dark | `@ # * + ; : .` |
| 2 | PLASMA | Purple 99 | white→pink→hot-magenta→purple→violet | `* * + + . . ,` |
| 3 | NOVA | Blue 69 | white→bright-cyan→cyan→sky→deep-blue | `* * + + . . ,` |
| 4 | POISON | Olive 64 | white→bright-yellow→chartreuse→lime→dark | `* * + + . . ,` |
| 5 | OCEAN | Teal 31 | white→ice→cyan→ocean→deep | `~ o ~ + . , .` |
| 6 | GOLD | Copper 136 | white→pale-gold→gold→ochre→bronze→copper | `* * + + . . ,` |
| 7 | NEON | Hot pink 201 | white→light-pink→hot-pink→magenta→deep | `* * + + . . ,` |
| 8 | ARCTIC | Ice 153 | white→pale-blue→ice→steel→grey | `* * + . . , `` |
| 9 | LAVA | Crimson 88 | white→amber→deep-orange→red→dark-red→black | `* * + + . . ,` |

Switching theme calls `init_pair()` again for all color pairs — ncurses repaints immediately.

## Non-Obvious Decisions

### Tangential velocity dominates at high speed
At `rot_speed = 20 rad/s`, `tang_v = 20 × 88 × 0.5 = 880 px/s`. The radial kick is only 35–120 px/s. So fast sparks fly almost purely tangentially — they spiral and arc around the gear rather than spraying outward. This emerges automatically from the physics, no special-casing needed.

### `exp(-DRAG × dt)` for drag
`dv/dt = -k×v` has exact solution `v(t) = v₀ × exp(-k×t)`. Using `vx *= exp(-DRAG × dt)` per frame is frame-rate independent — doubling frame rate gives identical visual results. The common alternative `vx *= (1 - DRAG × dt)` is only a first-order approximation that diverges at low frame rates.

### `init_pair()` for theme switching
ncurses `init_pair()` can be called repeatedly on the same pair index. Calling it for all 9 pairs when `t` is pressed remaps the entire color system immediately — no need to re-draw sparks with new attributes.

### Pool allocation for sparks
Rather than dynamic allocation, a fixed array of 1500 Spark structs is scanned for `life <= 0` slots. At 1200 sparks/sec × 1.9s lifetime = 2280 needed at saturation — capped at 1500, so the pool fills before `emit_acc` would overflow.

## Open Questions to Explore

1. What if sparks had an angular momentum component that curved their path — spiral trails instead of straight lines?
2. Can multiple meshing gears share spark pools, with sparks emitting only at the contact point between teeth?
3. What if each tooth had a slightly different emission color, creating banded arcs at high speed?
4. Can the gear react to load — spark density increases when rotation is slowing (simulated resistance)?

---

## From the Source

**Algorithm:** Analytic polar gear geometry — no mesh storage. The gear outline is computed each frame by iterating polar angle steps and testing proximity to three geometric features (hub ring, spokes, tooth sides) using THRESH_CIRC=7.5 px, THRESH_SIDE=4.0 px, THRESH_SPOKE=3.8 px. Geometry is regenerated each frame from the current rotation angle.

**Math:** Involute tooth approximated as a trapezoid in polar coordinates. `phase = fmod(ang_local × N_TEETH / 2π, 1.0)`; `in_tooth = (phase < TOOTH_DUTY=0.42)`. Tangential surface velocity: `v_tip = GEAR_R_OUTER × ω` (ω in rad/s). Spark initial velocity ≈ v_tip direction + KICK_MIN=35 to KICK_MAX=120 px/s radial + SCATTER=55 px/s. Exponential drag: `v *= exp(-DRAG × dt)` — exact solution to `dv/dt = -k·v`, frame-rate independent.

**Data-structure:** Spark pool — fixed array of MAX_SPARKS=1500 structs scanned for `life <= 0` slots. O(N) per frame. Emission rate scales with ω: `total_rate = SPARK_BASE_RATE × (rot_speed / ROT_BASE) × spark_density`. GEAR_R_OUTER=88 px, GEAR_R_INNER=64 px, GEAR_R_HUB=22 px, N_TEETH=10.

---

# Pass 2 — Pseudocode, Module Map, Data Flow

## Module Map

```
§1 config      — geometry constants, physics constants, theme count
§2 clock       — clock_ns() / clock_sleep_ns()
§3 themes      — Theme struct, THEMES[10] array, STAGE_THRESH[], spark_stage()
§4 color       — color_apply_theme(), color_init()  — maps theme to init_pair calls
§5 coords      — px_col() / px_row()
§6 entity      — Spark struct, Gear struct
               — gear_init(), spark_emit(), gear_tick()
§7 draw        — line_char(), draw_gear(), draw_sparks()
§8 screen      — Scene (gear + paused + max_px/py + theme), screen_init/render/resize
§9 app         — App, signal handlers, app_key(), main loop
```

## Data Flow Diagram

```
gear_init(g)
    │  cx=center, angle=0, rot_speed=ROT_BASE, spark_density=1.0
    ▼
gear_tick(g, dt, max_px, max_py)
    │  angle += rot_speed × dt
    │  total_rate = BASE × (rot_speed/ROT_BASE) × spark_density
    │  emit_acc += total_rate × dt
    │  while emit_acc ≥ 1: pick random tooth → spark_emit(g, tip_ang)
    │
    │  spark_emit(tip_ang):
    │    find dead slot in sparks[]
    │    px/py = gear center + R_OUTER × (cos, sin)(tip_ang)
    │    vx/vy = tang_velocity + radial_kick + scatter
    │    life  = 0.85 + rand × 0.15
    │
    │  for each live spark:
    │    life -= dt/SPARK_LIFE
    │    vy += GRAVITY×dt  ;  vx/vy += turbulence×dt
    │    vx/vy *= exp(-DRAG×dt)
    │    px/py += vx/vy×dt
    │    kill if out of bounds
    ▼
draw_sparks(win, g, cols, rows, theme_idx)
    │  for each live spark:
    │    st = spark_stage(life)   ← STAGE_THRESH lookup
    │    ch = THEMES[theme_idx].spark_ch[st]
    │    at = ATTR_DEC[THEMES[theme_idx].spark_at[st]]
    │    cp = CP_S0 + st
    │    mvwaddch(r, c, ch | COLOR_PAIR(cp) | at)
    ▼
draw_gear(win, g, cols, rows)
    │  scan bounding box cells
    │  for each cell: compute (rad, ang_local, phase, in_tooth)
    │    test proximity to hub/spokes/sides/inner-arc/outer-arc
    │    if match: ch = line_char(ang), mvwaddch with CP_GEAR or CP_GEAR_DIM
    ▼
wnoutrefresh → doupdate
```

## Core Loop Pseudocode

```
init gear (center, ROT_BASE speed, density=1.0)
apply theme 0 (FIRE)

while running:
    if SIGWINCH: resize, recentre gear
    dt = elapsed, clamped 100ms

    gear_tick(dt)               — rotate, emit, tick sparks

    erase()
    draw_sparks(theme)          — sparks first (behind gear)
    draw_gear()                 — wireframe on top
    draw_HUD(fps, speed, density, sparks_live, theme_name)
    doupdate()

    handle keys:
        q/ESC  → quit
        spc    → pause/resume
        r      → reset (keep theme)
        +/-    → rot_speed ± 0.6, clamp [0.2, 20]
        ]/[    → spark_density ± 0.3, clamp [0.2, 6.0]
        t/T    → theme ±1 mod 10, color_apply_theme()
        1-5    → rot_speed presets (0.4/2/6/12/20)

    sleep to 60 fps
```

## Key Equations

**Tooth phase (which fraction of tooth cycle are we at):**
```
phase = fmod(ang_local × N_TEETH / 2π,  1.0)    -- ∈ [0, 1)
in_tooth = (phase < TOOTH_DUTY)                  -- TOOTH_DUTY = 0.42
```

**Arc distance to tooth side edge:**
```
dp0 = min(phase, 1-phase)          -- distance to tooth start
dpT = |phase - TOOTH_DUTY|         -- distance to tooth end
arc_side = min(dp0, dpT) × (2π/N_TEETH) × rad
```

**Spark tangential velocity:**
```
v_surface = rot_speed × R_OUTER              -- surface speed (px/s)
tang_vx   = -sin(tip_ang) × v_surface × TANG_SCALE
tang_vy   =  cos(tip_ang) × v_surface × TANG_SCALE
```

**Exponential drag (frame-rate independent):**
```
vx(t+dt) = vx(t) × exp(-DRAG × dt)
vy(t+dt) = vy(t) × exp(-DRAG × dt)
```
