# Pass 1 — fluid/reaction_diffusion.c: Gray-Scott Turing patterns

## Core Idea

Two chemicals, U and V, diffuse through the terminal grid and react with each other. U is a substrate (continuously replenished by the feed rate). V is a catalyst (continuously removed by the kill rate). Where they overlap, they react: V converts U into more V. This feedback loop — combined with V diffusing more slowly than U — produces stable self-organising patterns: spots, stripes, coral, worms, mazes.

## The Mental Model

Think of a lawn where rabbits (V) eat grass (U). Grass grows back everywhere (feed rate). Rabbits die off naturally (kill rate). Where there are many rabbits, grass gets eaten. Where grass recovers faster than rabbits spread, the rabbits cluster into isolated groups. The balance between how fast grass grows back (f) and how fast rabbits die (k) determines whether you get isolated rabbit colonies (spots), long rabbit trails (stripes), or a uniform lawn (no rabbits).

Small changes in f and k produce radically different visual patterns — this is the core surprise of the Gray-Scott model.

## The Equations

    dU/dt = Du·∇²U  −  U·V²  +  f·(1−U)
    dV/dt = Dv·∇²V  +  U·V²  −  (f+k)·V

- `Du`, `Dv` — diffusion rates (Du > Dv; substrate spreads faster than catalyst)
- `U·V²` — the reaction term: V converts U into more V (autocatalytic)
- `f·(1−U)` — feed: U is replenished toward 1
- `(f+k)·V` — kill: V is removed

## 9-Point Isotropic Laplacian

    ∇²u ≈ 0.20·(N+S+E+W) + 0.05·(NE+NW+SE+SW) − u

This weights cardinal neighbours more than diagonal neighbours and is more rotationally symmetric than the 4-point stencil. The result is rounder spots and smoother curves.

## Dual-Grid Ping-Pong

Two grids `grid[0]` and `grid[1]` alternate as read and write each step:

    this step: read grid[cur], write grid[1-cur]
    next step: flip cur = 1 - cur

This ensures all cells read from the same generation simultaneously — the reaction at cell A does not use the already-updated value of neighbouring cell B. Without ping-pong the update order would bias the pattern.

## Preset Parameters

| Preset | f | k | Pattern |
|---|---|---|---|
| Mitosis | 0.0367 | 0.0649 | Spots that divide and multiply |
| Coral | 0.0545 | 0.0620 | Branching coral-like growth |
| Stripes | 0.0300 | 0.0570 | Zebra-like alternating bands |
| Worms | 0.0780 | 0.0610 | Writhing worm-like structures |
| Maze | 0.0290 | 0.0570 | Labyrinth-like channels |
| Bubbles | 0.0980 | 0.0570 | Expanding rings and holes |
| Solitons | 0.0250 | 0.0500 | Stable moving blobs |

## Non-Obvious Decisions

### Why 600 warm-up steps?
Patterns start from a nearly-uniform state (U≈1, V≈0) with a small seeded perturbation. At t=0 the screen would be nearly blank. Running 600 steps before the first frame means interesting structure is already visible when the program starts.

### Why is Du > Dv?
If V diffused faster than U, the activator would spread before the substrate could sustain it — patterns would wash out. The slower diffusion of V is what enables localised pattern formation (Turing's original insight).

### Why drop a seed blob with 's'?
Starting from a uniform state takes many seconds before patterns form. A seed blob (patch of high V at the centre) kickstarts the reaction immediately, letting you see how patterns nucleate and spread.

## Key Constants

| Constant | Effect |
|---|---|
| Du, Dv | Diffusion rates; ratio Du/Dv controls pattern scale |
| f | Feed rate; too high → U oversaturates; too low → V dies out |
| k | Kill rate; tiny shifts move between completely different pattern regimes |
| STEPS_PER_FRAME | Controls simulation vs display speed tradeoff |
