# Concept: Network Simulation

## Pass 1 — Understanding

### Core Idea
Simulate dynamics on a graph: nodes with states, edges with weights or capacities. Models include: epidemic spreading (SIR model), information diffusion, power grid cascades, random graph evolution. The graph structure fundamentally shapes how things propagate.

### Mental Model
Imagine diseases spreading through a social network. Each person is a node: Susceptible, Infected, or Recovered. Infected people infect their neighbors with some probability each timestep. The network topology — who knows whom — determines whether an epidemic spreads or dies out.

### Key Equations
SIR epidemic model on network:
```
S → I: probability β per infected neighbor per timestep
I → R: probability γ per timestep
R → S: probability δ (immunity loss, optional)
```

Basic reproduction number: `R₀ = β·⟨k⟩/γ`
- Epidemic spreads if R₀ > 1
- ⟨k⟩ = mean node degree

Random graph (Erdős-Rényi): each edge exists with probability p.
Scale-free (Barabási-Albert): new nodes attach preferentially to high-degree nodes.

### Data Structures
- `node[N]`: {state, x, y, degree}
- `edge[M]`: {src, dst, weight}
- Adjacency list for efficient neighbor iteration
- State counts: S, I, R for statistics

### Non-Obvious Decisions
- **Layout algorithm**: Place nodes with spring-force layout (Fruchterman-Reingold): edges pull nodes together, node pairs repel. Run for 100 iterations.
- **Epidemic threshold**: On random graphs, R₀ = β·⟨k⟩/γ. On scale-free networks, epidemics spread for any β>0 because hubs have infinite effective degree in the limit.
- **Cascade simulation**: When a node fails, it can trigger neighbors to fail. Model power grid failures or financial contagion.
- **Animate timesteps slowly**: Show state transitions one step at a time so the user sees waves propagating.

### Key Constants
| Name | Role |
|------|------|
| N | number of nodes |
| β | infection probability per contact |
| γ | recovery probability |
| p | edge probability (Erdős-Rényi) |
| INIT_INFECTED | number of initial infection seeds |

### Open Questions
- What is the epidemic threshold for a scale-free network vs. random graph?
- How does clustering coefficient affect epidemic spread?
- Can you identify the "super-spreader" nodes (highest degree)?

## From the Source

**Algorithm:** SIR (Susceptible-Infected-Recovered) epidemic model on a Watts-Strogatz small-world network. Per tick: each I node infects each S neighbour with probability β; each I node recovers with probability γ.

**Math:** Basic reproduction number: R₀ = β·⟨k⟩/γ where ⟨k⟩ is the mean degree. Epidemic threshold R₀=1 marks the phase transition between extinction and outbreak. Node positions on ring: θ_i = 2πi/N, placed in a circle.

**Data-structure:** Watts-Strogatz construction: start with a K=4 ring graph (each node connected to K/2 nearest neighbours on each side); rewire each edge with probability p=0.15, replacing the target with a random node. Rewired "shortcut" edges create the small-world property: short average path length + high clustering coefficient.

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_adj[N_NODES][N_NODES]` | `bool[40][40]` | ~1.6 KB | adjacency matrix of the Watts-Strogatz graph |
| `g_rewired[N_NODES][N_NODES]` | `bool[40][40]` | ~1.6 KB | flags edges that were rewired (shortcut edges) |
| `g_node_px[N_NODES]` | `float[40]` | 160 B | x pixel positions of nodes on ring layout |
| `g_node_py[N_NODES]` | `float[40]` | 160 B | y pixel positions of nodes on ring layout |
| `g_state[N_NODES]` | `SIR[40]` | 160 B | current S/I/R state per node |
| `g_flash[N_NODES]` | `int[40]` | 160 B | countdown ticks for newly-infected flash |
| `g_hist_s[HIST_LEN]` | `int[500]` | ~2 KB | rolling S count history for epidemic curve |
| `g_hist_i[HIST_LEN]` | `int[500]` | ~2 KB | rolling I count history for epidemic curve |
| `g_hist_r[HIST_LEN]` | `int[500]` | ~2 KB | rolling R count history for epidemic curve |

---

## Pass 2 — Implementation

### Pseudocode
```
build_erdos_renyi(N, p):
    nodes = N random positions
    for i,j in pairs: if random()<p: add_edge(i,j)

spring_layout(nodes, edges, iterations):
    for _ in range(iterations):
        for each node i:
            force = (0,0)
            # repulsion from all nodes
            for j != i: force += K²/dist(i,j) * direction(j→i)
            # attraction along edges
            for each neighbor j: force += dist(i,j)²/K * direction(i→j)
            nodes[i].pos += force * dt

sir_step(nodes, edges, beta, gamma):
    new_states = copy(states)
    for each infected node i:
        if random() < gamma: new_states[i] = RECOVERED
        for each neighbor j:
            if states[j]==SUSCEPTIBLE and random()<beta:
                new_states[j] = INFECTED
    states = new_states

draw():
    for each edge: draw_line(nodes[src], nodes[dst], dim)
    for each node: draw_char(node.x, node.y, state_char(node.state), state_color)
    draw_SIR_histogram()
```

### Module Map
```
§1 config    — N, p, BETA, GAMMA, layout type
§2 graph     — build_random(), build_scalefree(), adj list
§3 layout    — spring_layout() for visual placement
§4 dynamics  — sir_step(), cascade_step()
§5 draw      — nodes + edges + state overlay + statistics
§6 app       — main loop, keys (seed infection, speed, reset, model)
```

### Data Flow
```
graph structure → spring layout → node positions
states → SIR step → new states → draw nodes (color by state) + statistics
```
