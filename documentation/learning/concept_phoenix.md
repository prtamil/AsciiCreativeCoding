# Concept: Perched Phoenix — Self-Immolation and Rebirth

## Pass 1 — Understanding

### Core Idea
A perched owl-shaped phoenix sits still on a branch, breathes softly, then ignites itself in a multi-stage lifecycle: it heats up, becomes a roaring fire, dissolves into falling embers, smoulders as ash, then regrows from a single spark at the head outward until it is whole again, eyes glowing — and the cycle repeats. One full cycle ≈ 26 seconds.

The bird is not a flying entity. It is a **silhouette painted by a swarm of particles** that snap to a fixed anchor template each frame. The lifecycle phase decides the rules of the snap: snap always (PERCH), snap with rising heat (IGNITE), snap at white-hot (BLAZE), stop snapping piece by piece (COLLAPSE), don't snap and float away (ASH), then snap back from the inside out (REBIRTH).

### Mental Model
Imagine fireflies trained to fly into a pose. At rest they land in formation tracing an owl. Heat them up: they glow brighter. Heat them more: the formation breaks; some leave the pose and rise as embers. Wait long enough and the heat fades to nothing. Drop one bright firefly back where the owl's eye should be, and call the others home — they return one by one, nearest pose-points first, and the silhouette grows back from a single point.

That's the architectural intuition. Two cooperating particle systems share the screen:
- **BODY particles** are anchor-bound by default. Each has a `released` bit; when `false` they snap to a randomly-picked anchor every frame, when `true` they integrate fire physics like a normal ember. Phase logic is what flips the bit.
- **SPARK particles** are pure free-flight embers — emitted from random anchors at phase-dependent rates, integrate the same fire physics as released body particles, recycled when life or temperature expires.

### Lifecycle FSM

| Phase | Duration | Body behaviour | Spark rate | Visual |
|-------|----------|----------------|------------|--------|
| `PERCH`    | 12 s | All BOUND. Temp = anchor.base_temp (0.14–0.55). | 0/s | Dim owl, eyes brightest, gentle vertical breath bob. |
| `IGNITE`   |  2 s | All BOUND. Temp lerps from `base_temp → 1.0` over the phase. | 20/s | Silhouette warms from feather tints to flame. Sparks start. |
| `BLAZE`    |  3 s | All BOUND. Temp ∈ [0.85, 1.00] random. | 120/s | Owl-shaped fire, bright `@` and `%` glyphs, white-hot core. |
| `COLLAPSE` |  2 s | Each frame, prob `frac · 6.0 · dt` of flipping BOUND→FREE. FREE particles get downward+lateral velocity, integrate physics. | 60/s | Silhouette dissolves, embers fall and drift. |
| `ASH`      |  4 s | All FREE. Smoulder, drift, cool toward 0. | 8/s | No silhouette. Smoke (bucket 0, dim grey) dominates. |
| `REBIRTH`  |  3 s | Each frame, prob `5.0 · dt` of capturing onto an alive anchor. Bird rebuilds from head outward. | 15/s | Single spark at eye-centre, expanding fire-circle becomes owl. |

### Key Equations

**Heat-ramp bucket** (used everywhere — body and sparks):
```
b = clamp(floor(temp · 6), 0, 5)
b=0 '.' A_DIM   smoke (bucket 0, theme[0])
b=1 '+'         dim feather/dark theme tint
b=2 '*'         mid theme tint
b=3 '#'         bright theme tint (ember)
b=4 '@' A_BOLD  hot accent
b=5 '%' A_BOLD  white-hot core
```

**Anchor `alive_at` (radial wake order during REBIRTH):**
```
seed = head centre (0, -3)
d_i      = sqrt((dx_i − seed_x)² + (dy_i − seed_y)²)
d_max    = max_i d_i
alive_at = d_i / d_max                       ∈ [0, 1]
anchor i is alive when growth_frac ≥ alive_at
```
Eyes are close to the seed (small `alive_at`) so they wake up first; ear-tufts are far (alive_at ≈ 1.0) so they appear last. This produces the visual "bird grows from the eyes outward".

**Phase temperature modifier for BOUND particles:**
```
PERCH    : T = base_temp + jitter
IGNITE   : T = lerp(base_temp, 1.0, frac) + jitter
BLAZE    : T = 0.85 + 0.15 · rand
COLLAPSE : T = lerp(1.0, 0.4, frac)
REBIRTH  : T = 0.95 − 0.30 · alive_at_of_anchor + jitter
            (centre hot, edges cooler — outward heat front)
```

**Free-flight fire physics** (FREE body particles AND sparks share `fire_step()`):
```
shx, shy  = sin/cos shear (FIRE_SHEAR_HZ · t + 0.30·x ± 0.45·y)
v        += FIRE_SHEAR_AMP · (shx, 0.6·shy) · dt
vy       -= FIRE_BUOYANCY · (0.20 + 0.80·T) · dt
v        *= 1 − FIRE_DRAG · dt
x, y     += v · dt
T        *= exp(−FIRE_COOL · dt)
```
The buoyancy floor (`0.20 + 0.80·T`) means even smoke (T → 0) still rises slowly; that's why ash drifts upward instead of falling.

### Owl Silhouette Anchors

| Region | Count | Weight | base_temp | Notes |
|--------|-------|--------|-----------|-------|
| Head outline | 12 | 1.4 | 0.18 | Ellipse around `(0, −3)`, rx=4, ry=3 |
| Body outline | 14 | 1.2 | 0.16 | Ellipse around `(0, +2.5)`, rx=3, ry=3.5 |
| Body fill | 5 | 0.7 | 0.14 | Inner ellipse for visible mass |
| Eyes | 2 | **5.0** | **0.55** | Always-bright pupils, dominate the silhouette |
| Beak | 1 | 2.0 | 0.40 | Below eyes |
| Ear tufts | 2 | 1.5 | 0.22 | Pointed tips above head |
| **Total** | **~37** | | | |

Eyes get high weight AND high base_temp — that's why a perched owl reads as "two glowing yellow eyes on a dim feather-coloured body" rather than a uniform blob.

### Non-Obvious Decisions

- **Two pools, not one.** Body particles stay roughly constant in count and snap to anchors; sparks recycle through a separate pool with life-cap and emission accumulator. Tried merging: a single pool needed a "is this currently a body particle or a spark?" flag and the FSM logic became unreadable. Splitting by lifetime model (snap vs. free) was cleaner.

- **`alive_at` as anchor metadata, not a separate sort.** Computed once at init from `dist(anchor, seed) / max_dist`. REBIRTH simply filters anchors with `alive_at ≤ growth_frac`. This sidesteps any per-frame sorting and naturally gives the radial wake-up effect.

- **PERCH still uses the heat ramp.** The owl at rest renders at temp ≈ 0.16–0.55 — buckets 1–2 of the same ramp BLAZE uses at 0.85–1.0. Lower buckets are dim feather-brown; higher are white-hot. One ramp does both jobs; no separate "body tint vs fire" code path.

- **REBIRTH→PERCH snap.** When the phase advances, any still-FREE body particle is reassigned to a random anchor immediately so the next PERCH frame is a complete owl, not a half-rebuilt one. This is one frame of catch-up but reads cleaner than waiting for slow capture rates to finish during PERCH.

- **`released` field instead of two pools.** Each body particle carries its own state. Cheaper than maintaining two arrays and lets the same draw loop handle both BOUND and FREE.

- **Anchor weight × phase rate, not per-anchor release order.** During COLLAPSE every BOUND particle has the same per-frame release probability `frac · COLLAPSE_RELEASE_RATE · dt`. We don't release "head first" or "feet first" — but because anchors with lower weight have fewer particles bound to them, sparser regions empty visibly first in practice.

### Free-Flight Fire Physics — Why Each Term

| Term | Formula | Why |
|------|---------|-----|
| Buoyancy | `vy -= BUOYANCY · (0.20 + 0.80·T) · dt` | Boussinesq approximation: hot air rises, density gradient drives lift. The 0.20 floor means even cold smoke rises slowly (matches real wisps). |
| Turbulence | `v += SHEAR · sin/cos(t·hz + a·x + b·y) · dt` | Cheap stand-in for proper velocity advection. Position-dependent so neighbouring particles get different forces (otherwise the column sways as a block). |
| Drag | `v *= 1 − DRAG · dt` | Momentum decay to surrounding air. Without this, sparks accelerate to absurd speeds. |
| Cooling | `T *= exp(−COOL · dt)` | Stefan–Boltzmann small-perturbation form. Exponential decay — no negative temps, no linear-cooling artefact. |

### Themes (4)
| # | Name | Heat ramp (cool→hot) | Mood |
|---|------|----------------------|------|
| 0 | CLASSIC | 244 → 130 → 166 → 202 → 214 → 231 | brown owl, red/orange/yellow fire, white core |
| 1 | IRIS    | 244 →  96 → 134 → 170 → 213 → 231 | purple/pink phoenix |
| 2 | JADE    | 244 →  28 →  70 → 112 → 154 → 231 | green-jade phoenix |
| 3 | GOLD    | 244 →  94 → 130 → 172 → 214 → 231 | copper/gold phoenix |

All entries sit in the bright half of the 256-cube per the CLAUDE.md "Theme Palette Brightness" rule — even bucket 0 (244) renders cleanly under `A_DIM`.

### Open Questions
- Could IGNITE spread non-uniformly — fire seeded at the beak, propagating outward via the same `alive_at` mechanism? Framework already supports it; current build keeps uniform heating for legibility.
- Multiple phoenixes side-by-side, phase-offset? Each is independent state; trivial to instantiate.
- A "rage" mode where the BLAZE phase emits sparks at a much higher rate (500/s+)?

---

## Pass 2 — Implementation

### Module Map
```
§1  config    — sizes, phase timings, fire physics constants, palette
§2  clock     — monotonic timer + sleep
§3  color     — 6-bucket heat ramp by theme + perch tint + HUD
§4  random    — frand / frand_signed
§5  anchor    — owl anchor table (one-shot build) + alive_at + pickers
§6  body      — BodyParticle (BOUND ↔ FREE), rebind, release, draw
§7  spark     — Spark (free-flight only), emit, tick, draw
§8  phoenix   — Phoenix state, lifecycle FSM, emission rules
§9  scene     — composition, perch glyphs, HUD
§10 screen    — ncurses init / cleanup
§11 app       — signals, dt tracking, key handling, main loop
```

### Data Flow
```
phoenix_init():
    anchors = anchor_table_build()          # one-shot, ~37 anchors
    foreach body[i]: snap to random anchor (BOUND, base_temp)
    sparks = inactive

main loop:
    dt = wall-clock since last frame (capped at 100 ms)
    phoenix_tick(dt):
        world_t += dt; phase_t += dt
        if phase_t >= duration(phase): advance_phase
        bob = sin(world_t · breath_hz) · breath_amp     [PERCH only]
        phoenix_tick_body(dt, bob):              # branches on phase
            PERCH/IGNITE/BLAZE  → all BOUND, rebind to full anchor pool
            COLLAPSE            → probabilistic BOUND→FREE flip, mixed
            ASH                 → all FREE, integrate physics
            REBIRTH             → probabilistic FREE→BOUND, alive-only
        phoenix_tick_sparks(dt):                 # phase-dependent rate
            emit_acc += dt · spark_hz_for_phase
            spawn floor(emit_acc) sparks (cap per frame)
            integrate every active spark
    scene_draw():
        erase
        draw_perch (static '-' strip + cap glyphs)
        body BOUND first (silhouette base layer)
        body FREE next (released embers on top)
        sparks (free-flight pool above everything)
        HUD
```

### Key Data Structures

```c
typedef struct {
    float dx, dy;       /* body-local cell coords (head at origin)     */
    float weight;       /* draw probability                              */
    float base_temp;    /* PERCH temperature                             */
    float alive_at;     /* REBIRTH wake threshold (0=seed, 1=far)        */
    AnchorKind kind;    /* AK_HEAD / BODY / EYE / BEAK / TUFT           */
} Anchor;

typedef struct {
    float x, y;         /* world cell coords                             */
    float vx, vy;       /* used while FREE                               */
    float temp;         /* drives heat-ramp bucket                       */
    int   anchor_idx;   /* current anchor when BOUND                     */
    bool  released;     /* false=BOUND, true=FREE                        */
} BodyParticle;

typedef struct { float x, y, vx, vy, temp, life; bool active; } Spark;
```

### Default Constants (one-line each)

| Name | Value | Role |
|------|-------|------|
| `N_BODY_DEFAULT` | 360 | particles painting the silhouette |
| `N_SPARK_DEFAULT` | 220 | free-flight ember pool |
| `T_PERCH/IGNITE/BLAZE/COLLAPSE/ASH/REBIRTH` | 12/2/3/2/4/3 s | phase durations |
| `BODY_JITTER_R` | 0.55 cells | rebind jitter radius |
| `BREATH_HZ` | 0.25 | perch chest rise (cycles/sec) |
| `COLLAPSE_RELEASE_RATE` | 6.0 | per-frame BOUND→FREE coefficient |
| `REBIRTH_CAPTURE_RATE` | 5.0 | per-frame FREE→BOUND coefficient |
| `FIRE_BUOYANCY` | 11.0 | upward acceleration scaled by temp |
| `FIRE_DRAG` | 1.4 | velocity damping per second |
| `FIRE_COOL` | 0.85 | exponential temp decay rate |
| `FIRE_SHEAR_AMP` | 6.0 | turbulence amplitude |
| `FIRE_SHEAR_HZ` | 1.5 | turbulence base frequency |
| `SPARK_HZ_BLAZE` | 120 | sparks/sec at peak emission |

### References
- Reeves, "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects", SIGGRAPH 1983 — emit/integrate/cool/recycle pattern.
- Stam, "Real-Time Fluid Dynamics for Games", GDC 2003 — sin/cos-shear turbulence as a cheap stand-in for proper velocity advection.
- Boussinesq buoyancy approximation — any combustion textbook; here `vy -= BUOYANCY · temp · dt`.
- Phoenix mythology — Bulfinch's *Mythology* (1855), Egyptian Bennu, Persian Simurgh.

### Cross-references in this codebase
- `artistic/fire_tornado.c` — same heat-ramp + buoyancy on a vertical column.
- `artistic/volcano.c` — free-flight cooling embers with ambient bursts.
- `artistic/ant_colony.c` — anchor-template silhouette pattern from a different domain (foragers around food).
