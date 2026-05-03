# Concept — `procedural_constellation.c`: Star-Anchor Graph + Bresenham

(Distinct from the older `concept_constellation.md`, which covers the proximity-link drifting-stars demo. This file is the substitution / graph variant.)

## Core Idea

A constellation is two things and only two things: a small set of points scattered in the sky, and a set of edges that someone — long ago, around a fire — drew between those points to make a picture. The points come first; the edges come second; the picture is imaginary. Procedurally, we generate the points (jittered grid), pick the edges (one of four rules), draw the lines (Bresenham), and slap on a name. There is nothing else.

---

## The Mental Model

Imagine punching ten holes through a sheet of black paper, then drawing pencil lines BETWEEN selected pairs of holes. The holes are the stars; the pencil lines are the constellation. Different constellations differ ONLY in (a) where you punched the holes and (b) which pairs you chose to connect with pencil. Both are choices; neither is intrinsic to the stars.

Now: replace "punch holes" with "place jittered points in a grid", and replace "draw pencil lines" with one of four small algorithms. That is the entire generator. The reason the result LOOKS like a real constellation is that real constellations were drawn by humans subject to the same constraints: stars roughly evenly distributed across the visible sky, and edges chosen for visual simplicity (no crossings, no long jumps, every star connected).

Why FOUR topologies? Real constellations cluster into morphological families:
- branchy ones (Orion, Hercules) → MST
- linear ones (Big Dipper, Cassiopeia's W) → CHAIN
- closed ones (Pegasus square, Corona) → LOOP
- radial ones (Crux, the Cross) → SPOKE

Picking one and applying it consistently produces a recognisable constellation flavour every time.

---

## Algorithm in Steps

1. **PICK N.** Random integer in `[5, 10]` depending on pattern.

2. **PLACE N STARS** in a region of size `REG_W × REG_H`:
   - `cols = round(√N)`, `rows = ⌈N / cols⌉`
   - `cell_w = REG_W / cols`, `cell_h = REG_H / rows`
   - For each cell `(c, r)` in row-major order, until N stars:
     ```
     h = hash3(c, r, seed)
     jitter_x = ((h        & 0x3FF) − 512) · cell_w / 2048
     jitter_y = ((h >> 10) & 0x3FF) − 512) · cell_h / 2048
     star.x = c·cell_w + cell_w/2 + jitter_x
     star.y = r·cell_h + cell_h/2 + jitter_y
     ```

3. **BUILD EDGES** per pattern:
   - **TREE**: Prim's. Maintain `in_tree[]`; repeatedly add the cheapest cross-edge until N-1 edges placed.
   - **CHAIN**: sort stars by x; emit edges 0-1, 1-2, …, (N-2)-(N-1).
   - **LOOP**: compute centroid; sort stars by `atan2(y-cy, x-cx)`; emit edges 0-1, …, (N-1)-0.
   - **SPOKE**: compute centroid; pick `hub` = nearest star to centroid; emit one edge from hub to every other star.

4. **NAME the constellation:**
   ```
   prefix    = PREFIXES[hash(seed, 1) mod N_P]
   suffix    = SUFFIXES[hash(seed, 2) mod N_S]
   modifier  = MODIFIERS[hash(seed, 3) mod N_M]
   name      = prefix ++ suffix ++ (" " ++ modifier  if non-empty)
   ```

5. **ANIMATE** in four phases:
   - PHASE_DRAW_STARS: reveal stars one by one, fade-in over `PHASE_STARS_TOTAL` seconds.
   - PHASE_DRAW_EDGES: reveal edges in order, each over `PHASE_EDGE_TIME` seconds. Per-edge `reveal_t ∈ [0,1]` drives the partial Bresenham plot.
   - PHASE_HOLD: freeze the figure, fade in the name, dim underline grows under it.
   - PHASE_FADE: sky-wide flash, then regenerate.

---

## Key Formulas

**Bresenham** (one cell per step, integer-only):
```
dx, dy = |x1-x0|, |y1-y0|
sx, sy = sign(x1-x0), sign(y1-y0)
err    = dx − dy
each step:
  plot(x, y)
  if 2·err >= -dy: err -= dy; x += sx
  if 2·err <=  dx: err += dx; y += sy
stop when (x, y) == (x1, y1)
```

**Animated partial Bresenham:**
```
total_steps = max(|dx|, |dy|) + 1
step_limit  = ⌈reveal_t · total_steps⌉
```

**Line-segment glyph** from segment direction (dx, dy):
```
|dx| ≥ 2·|dy|              → '-'
|dy| ≥ 2·|dx|              → '|'
sign(dx) == sign(dy)       → '\\'
sign(dx) != sign(dy)       → '/'
```

**Prim's MST** (best-cross-edge until tree spans all N):
```
while tree.size < N:
  pick (i, j) with i in tree, j not in tree, minimising d(i,j)²
  add j to tree; emit edge (i, j)
```

**Polar sort key for LOOP topology:**
```
θ_i = atan2(y_i − cy, x_i − cx)         // centroid (cx, cy)
sort stars[] ascending by θ_i           // counterclockwise cycle
```

---

## Edge Cases and Pitfalls

- **REGION TOO SMALL.** If the constellation region is narrower than cols, the jittered-grid placement collapses (`cell_w < 1`) and multiple stars overlap. Clamp `REG_W ≥ 16` and `REG_H ≥ 8` in the layout step.

- **DUPLICATE STAR POSITIONS.** With heavy jitter two stars in adjacent cells COULD land on the same `(x, y)`. The MST is robust to that (zero-distance edges just fold the duplicates), but the rendering double-draws which looks weird. Reduce `JITTER_FRAC` below 0.5 so adjacent cells don't overlap.

- **PRIM'S TIE-BREAKING.** When two cross-edges have identical distance², the loop picks the first one found. That makes the MST seed-dependent in subtle ways (insertion order). It's deterministic — same seed, same MST.

- **LOOP WITH N=2.** The polygon collapses to two points joined by two coincident edges. Skip LOOP for very small N.

- **SORTING IN PLACE.** CHAIN sorts `stars[]` by x, LOOP sorts by angle. The edge indices reference positions IN THE SORTED array, so the order matters. Don't re-sort after edge construction.

- **LINE OFF-SCREEN.** Bresenham can step into cells that are out of the renderable region (HUD rows, off-screen). Always bounds-check before `mvaddch` — the algorithm doesn't know about the HUD.

- **REVEAL_T = 1.0 OFF-BY-ONE.** `⌈1·total_steps⌉` might be one fewer than `total_steps` when `total_steps` is small and `reveal_t` is very slightly < 1. Let the fully-reached state plot ALL cells by using `> 0.999` as the "complete" guard.

- **NAME OVERFLOW.** `snprintf` truncates if `prefix+suffix+modifier` is longer than the name buffer. Size the buffer to comfortably hold the longest combination of the largest fragment lengths.

---

## How to Verify

- **Pause** (space) during DRAW_EDGES: a partially-traced edge freezes mid-draw. Resume: the edge continues from exactly where it paused.

- Press **`r`**: sky flashes, regenerate. The new constellation has a DIFFERENT name and DIFFERENT star arrangement, but the topology family is the same as the active pattern.

- **TREE** pattern: count edges. Should be exactly `(n_stars − 1)`. The edge graph should be CONNECTED but ACYCLIC.

- **CHAIN** pattern: edges should run left-to-right with no crossings, each edge connecting horizontally-adjacent stars in the sort order.

- **LOOP** pattern: count edges = `n_stars` exactly. The edges should form a closed polygon (no break, no fork, every star is endpoint of exactly two edges).

- **SPOKE** pattern: one star is the endpoint of EVERY edge. That hub star should be visually near the centroid of the cluster.

- **NAMING**: trying many seeds should produce many different names with constellation flavour ("Aurelia", "Lyrenus", "Pegonor") rather than gibberish or repeats. With ~30 prefixes × 16 suffixes × 10 modifiers, expect <0.05% repeat rate over 100 trials.

---

## References

- Bresenham, J. (1965) — "Algorithm for computer control of a digital plotter", IBM Systems Journal 4(1):25-30.
- Prim, R. (1957) — "Shortest connection networks and some generalizations", Bell System Tech. J. 36(6).
- Wikipedia — [Constellation](https://en.wikipedia.org/wiki/Constellation).
- Wikipedia — Poisson-disk sampling (intuition for jittered placement).

---

*Source: `procedural/worldgen/procedural_constellation.c`*
