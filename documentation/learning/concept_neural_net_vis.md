# Concept: Neural Network Visualisation

## Pass 1 — Understanding

### Core Idea
A feed-forward neural network drawn as columns of `(O)` neurons fully connected by slope-character lines. One particle per input neuron drifts forward through the network — at each layer it picks a random target in the next layer and travels along that edge — looping back to the input layer when it reaches the output. Layer count, neurons-per-layer, and connection thickness are all interactive. Two address spaces share the screen: a stateless geometry derived per-pixel, and a small particle pool with addressed (layer, idx, t) state.

### Mental Model
Imagine the network as an infinite address book — every neuron has a 3-tuple `(layer, idx, side)`. Connections are edges between adjacent layers; the particle pool walks the graph one edge at a time. The drawing is a side-view: x is layer index, y is neuron index, both stretched to fit the terminal. Stateful structure is *minimal* — just sizes and a small particle pool. Geometry recomputes from terminal dims each frame.

### Key Equations
```
neuron_cell(layer, idx) = (
    col = (layer + 1) · cols     / (n_layers     + 1),
    row = (idx   + 1) · (rows-1) / (n_in_layer   + 1)
)

particle position = lerp(source_neuron, target_neuron, t)
particle hop      = on t ≥ 1: from_layer++, from_idx ← to_idx, to_idx ← rand(N[from+1])
                    if reached output: loop to random input neuron
```

### Non-Obvious Decisions
- **No stored neuron positions**: `neuron_cell()` recomputes them each frame from terminal size, so resize is automatic.
- **Connection per-cell slope chars**: `\` `/` `-` `|` chosen by the local step (`dsr`, `dsc`), not the global delta — avoids stairstep artefacts that read as multiple parallel lines.
- **Particle reset to `t = 0` on shape change**: makes the moment of restart visibly obvious.
- **Speed jitter ±40%**: particles desync within a few hops without explicit phase tracking.
- **Thickness ladder (dot/thin/bold/heavy)**: each level uses a different glyph set; level 3 swaps to UTF-8 `═ ║ ╲ ╱` for genuinely thicker strokes.

### Key Constants
| Name | Role |
|------|------|
| `MAX_PARTICLES` | One per input neuron (max 16) |
| `PARTICLE_SPEED` | 0.6 edges/sec base |
| `PARTICLE_JITTER` | 0.4 (±40% per particle) |
| `MAX_LAYERS` | 12 |
| `MAX_NEURONS` | 16 per layer |

### Open Questions
- Could particle target selection be biased by a learned weight matrix?
- Forward + backward particles on different colours = a "training step" visualisation. Stage 4?
- Can the same code render a recurrent network by adding self-loop connections?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — MIN/MAX/DEFAULT layers & neurons & thickness
§3 color     — per-theme triplet: neuron / connection / particle
§4 layout    — neuron_cell() coord seam + LineStyle table for thickness
§5 net       — Net struct (sizes, thickness, theme)
§6 particle  — Particle pool: reset / tick / draw + edge-by-edge hop
§7 scene     — draw_connections + particle_draw + draw_neurons + HUD
§9 app       — signals, dt tracking, key handling, main loop
```

### Data Flow
```
arrow / [ ] / -+ → modify Net struct → reseed particles
tick: for each particle: t += speed·dt; on t≥1, hop to next layer
draw: erase → connections → particles (over edges) → neurons (on top) → HUD
```

### References
- Goodfellow, Bengio & Courville, "Deep Learning" (2016) — feed-forward notation.
- Schmidhuber, "Deep Learning in Neural Networks: An Overview" *Neural Networks* 61 (2015).
