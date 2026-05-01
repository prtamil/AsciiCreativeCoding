# Concept: Phoenix in Flight

## Pass 1 — Understanding

### Core Idea
A bird-shaped formation of fire particles soars across the screen. The body is a fixed array of anchor points (head, neck, body, two wings, tail feathers); each anchor carries a cloud of jittery particles, each frame the particles rebind to their anchor's current world position. Wings flap via a sinusoid in time. The bird follows a four-phase lifecycle FSM — `FLY → DIE → ASH → BIRTH → FLY` — taking ~20 seconds per cycle.

### Mental Model
Anchors are body skeleton in body-local coordinates. Each frame: advance the head along its path, compute every anchor's world position by translating + applying wing-flap and head-bob, then re-bind every particle to a random anchor with a small jitter radius. The bird is *redrawn fresh* each frame — particles don't carry memory. The lifecycle FSM controls how the binding happens (FLY: bind to live anchors; ASH: release bindings, free flight + gravity; BIRTH: gradually rebind).

### Key Equations
```
anchor_world(idx, head_x, head_y, world_time, dir) =
    dx = anchor.dx · (-dir) · ASPECT_X     /* mirror x when flying left  */
    dy = anchor.dy + flap_term + bob_term

flap_term = (type ∈ {WING_L, WING_R}) ?
              sin(t · 2π · WING_FLAP_HZ) · WING_FLAP_AMP · wing_frac · sign : 0
bob_term  = sin(t · 2π · WING_FLAP_HZ) · HEAD_BOB_AMP · 0.3

particle_bind(p):
    (ax, ay) = anchor_world(p.anchor_idx, ...)
    p.x = ax + jitter_signed() · JITTER_R · ASPECT_X
    p.y = ay + jitter_signed() · JITTER_R
    p.temp = base_t(anchor.type) + small_jitter   /* heat gradient by body part */
```

### Non-Obvious Decisions
- **Particles rebind every frame** rather than physical motion: cleaner for a body that moves *as a whole* (the bird translates; particles don't fly independently except in ASH phase).
- **Heat varies by anchor type**: wing tips white-hot, body warm, tail orange. Without this, every particle was bucket 4 (white) — theme cycling produced no visible change.
- **Single-direction wing flap**: both wings up, both down, in unison — like real birds, not in opposition.
- **Mirror `dx` on `dir` change**: when flying right-to-left after rebirth, the bird's nose still points the way it's flying (head leads, tail trails).
- **ASH releases bindings**: particles get explicit velocity (radial outward burst) and free-flight physics, so the death feels like a real explosion rather than a shape collapsing.

### Key Constants
| Name | Role |
|------|------|
| `N_PART_DEFAULT` | 280 particles forming the body |
| `N_ANCHORS` | ~21 anchor points (computed from BODY_ANCHORS array) |
| `WING_FLAP_HZ` | 1.5 — flaps per second |
| `T_FLY / T_DIE / T_ASH / T_BIRTH` | 10 / 2.5 / 5 / 2.5 seconds per phase |
| `JITTER_R` | 0.7 cells — anchor jitter radius |

### Open Questions
- Could a 3-D body template projected to 2-D give convincing depth?
- What if the bird followed a Bezier path instead of a straight line + sinusoid?
- A flock of phoenixes? Each with phase-offset wing flaps?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — pool sizes, body geometry, lifecycle timings
§5 anchor    — Anchor struct + BODY_ANCHORS table + weighted picker
§6 particle  — Particle struct + draw
§7 phoenix   — Phoenix state + lifecycle FSM (4 phases) + emission
§8 scene     — particle render + HUD
§10 app      — signals, dt tracking, key handling
```

### Data Flow
```
init: pick anchor for each particle weighted by anchor.weight; bind
tick FSM:
  FLY   — head advances; particles rebind every frame
  DIE   — head decelerates; particles bind tighter, temps rise
  ASH   — bindings released; particles free-fly with gravity, cool fast
  BIRTH — head reaches start; particles rebind incrementally over time
draw: erase → particle_draw (heat-ramp by temp) → HUD
```

### References
- Reeves, "Particle Systems" SIGGRAPH (1983).
- Reynolds, "Steering Behaviors for Autonomous Characters" GDC (1999) — formation-flight inspires the anchor model.
