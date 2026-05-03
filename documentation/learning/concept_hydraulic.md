# Concept — `hydraulic.c`: Particle-Based Hydraulic Erosion

## Core Idea

To carve a realistic landscape, do not draw the rivers — let them draw themselves. Drop a tiny pebble of water onto a smooth height field; gravity pulls it downhill; whenever the slope steepens it digs out a bit of dirt; whenever the slope flattens it drops the dirt back. Repeat with thousands of pebbles. The first ones cut shallow scratches; later ones, biased to follow the existing scratches because those are the steepest paths, deepen them into channels. The channels merge into rivers, the rivers carve valleys, the valleys break into deltas at the lowlands. **Nobody designed any of it** — every feature is a consequence of "water flows down and carries dirt".

---

## The Mental Model

Picture a sheet of putty. Sprinkle marbles on top. Each marble:
- Rolls in the direction of steepest descent (gravity).
- Has a little scoop attached underneath.
- When rolling fast (steep slope), the scoop digs a little groove into the putty.
- When rolling slow (flat), the scoop is full and dribbles its load back out, raising the putty.
- When it leaves the table, it disappears and a new marble is placed somewhere random.

That is the algorithm. The putty is the heightmap, the marbles are the droplets, the scoop is the carrying capacity formula, and the grooves are the rivers. The marbles do not know about each other, but they collectively build the drainage network because each marble is steered by the grooves left by previous marbles.

---

## Algorithm in Steps

1. **INITIALISE the heightmap** with Perlin fBm (4 octaves of noise summed at halved amplitude / doubled frequency). Save a copy into `initial[]` for the cut/fill diff later.

2. **SPAWN a droplet** at a random cell with: `v = (0, 0)`, `speed = 1`, `water = 1`, `sediment = 0`.

3. **STEP** the droplet up to MAX_STEPS times:
   ```
   xi, yi = floor(droplet.x, droplet.y)
   h00, h10, h01, h11 = height of 4 surrounding cells
   ∇h = ((h10 − h00)(1 − fy) + (h11 − h01)·fy,
         (h01 − h00)(1 − fx) + (h11 − h10)·fx)
   v ← inertia·v − (1 − inertia)·∇h    [steering]
   v ← v / |v|                          [unit length]
   new_pos ← pos + v.  Sample h_new bilinearly.
   Δh = h_new − h_old.
   C = max(−Δh·speed·water·K, C_min).
   if (sediment > C || Δh > 0):
     DEPOSIT  — split (sediment−C)·rate over the four corners
                of the current cell.
   else:
     ERODE    — pull up min((C−sediment)·rate, |Δh|) using a
                disc-weighted brush of radius BRUSH_R.
   v² ← max(0, v² − Δh·g);  water ← water·(1 − e_rate).
   pos ← new_pos.
   ```

4. After a budget `DROPLETS_PER_GEN` of droplets, **hold for HOLD_SECONDS** and then go back to step 1 with a new seed.

---

## Key Formulas

**Bilinear height** at fractional `(x, y)`:
```
fx, fy = x − ⌊x⌋, y − ⌊y⌋
h = (h00·(1−fx) + h10·fx)·(1−fy) + (h01·(1−fx) + h11·fx)·fy
```

**Bilinear gradient** at the same point:
```
∂h/∂x = (h10 − h00)·(1 − fy) + (h11 − h01)·fy
∂h/∂y = (h01 − h00)·(1 − fx) + (h11 − h10)·fx
```

**Carrying capacity** (sediment a droplet CAN hold):
```
C = max(−Δh · speed · water · CAPACITY_K,  C_min)
```

**Erosion brush** (disc-weighted; `ω(d) = 1 − d/R` for `d ≤ R`):
```
total_w = Σ_disc ω(d_i)
height_i ← height_i − amount · ω(d_i) / total_w     for i in disc
```

**Energetics:**
```
speed² ← max(0, speed² − Δh · gravity)
water  ← water · (1 − evaporate_rate)
```

**Biome buckets** (same as `tectonic.c`):
```
e<0.15 DEEP_OCEAN  | <0.30 OCEAN   | <0.40 COAST
| <0.50 PLAINS     | <0.62 HILLS   | <0.75 MOUNTAINS
| <0.85 HIGHLANDS  | else PEAKS
```

---

## Edge Cases and Pitfalls

- **SINGLE-CELL EROSION = ZIG-ZAGS.** If you erode JUST the cell the droplet is in, every droplet cuts a 1-pixel channel and the terrain develops a stripey checkerboard look. Always erode over a SMALL DISC (`BRUSH_R = 2` cells) — the resulting valleys are smoothly rounded.

- **UNNORMALISED VELOCITY.** After `v ← inertia·v − (1−inertia)·∇h` the magnitude drifts; un-normalised, droplets in flat regions crawl to a halt and never leave. Always normalise to unit length so the droplet keeps moving until it walks off the map.

- **UPHILL SAFETY.** If a droplet steers uphill (`Δh > 0`) it must always deposit, capped at `Δh`, even if the capacity test would say "erode" — otherwise droplets dig holes UPHILL of where they came from, which is unphysical and visually awful.

- **LIMIT EROSION TO Δh.** Without this guard, a steep cell + a near-empty droplet can pull up more height than the slope actually offers, producing pits that water can never escape. Cap `erode_amount` at `|Δh|`.

- **EVAPORATION VS CAPACITY.** As water evaporates the carrying capacity drops; eventually the droplet must deposit anything it's carrying. This is what produces deltas at the foot of rivers. Don't make `EVAPORATE_RATE` too small (sediment carried to map edge) or too large (rivers deposit before they reach lowlands).

- **BOUNDARY HANDLING.** Bilinear sampling reads four corners `(xi, yi)..(xi+1, yi+1)`. On the right and bottom edges the +1 index is out of bounds; terminate the droplet rather than reading garbage. The simulation still produces erosion all the way to the edge because earlier steps reach there.

- **TRAIL DECAY.** `WATER_TRAIL_DECAY` is per-TICK, not per-second; if you raise the sim_fps, decay-per-second changes proportionally. For a fixed visual decay length, multiply by `exp(-K·dt)` instead of a constant. Here we accept the tick-coupling for simplicity.

---

## How to Verify

- **Pause** (space). Heightmap and trails freeze. Resume: simulation continues from exactly where it stopped.

- Switch to **EROSION** on a FRESH world. The map is uniform "no change" (no red, no blue) — initial == current. Watch for a few seconds; red lines appear (erosion in valleys) and blue blobs appear at the lowlands (deposition in deltas). The two should roughly balance — total cut ≈ total fill.

- Switch to **DROPLETS** during active erosion. You should see a glowing dendritic NETWORK of channels — recently-walked cells light up and fade. The network should match the valleys visible in TERRAIN — the channels follow the same valleys that have eroded.

- Switch to **SLOPE**. Steep cells (along ridges and valley walls) are bright; flat cells (plains and lake bottoms) are dim. Arrow glyphs at every cell point downhill — water flows from high to low.

- Press **`r`**. Flash, regen. The HUD's "droplets" counter resets to 0; the world is fresh; erosion starts over.

- Run with `speed = 64` (much faster); the simulation reaches 8 000 droplets in a couple of seconds and holds. Drop speed to 1 — droplets crawl and you can WATCH each erosion step individually as a single bright water trail.

---

## References

- Beyer, H. (2015) — "Implementation of a method for hydraulic erosion", Bachelor thesis (TU München) — the canonical reference for the particle method.
- Lague, Sebastian (2019) — "Coding Adventure: Hydraulic Erosion" (YouTube).
- Mei, X., Decaudin, P., Hu, B-G. (2007) — "Fast Hydraulic Erosion Simulation and Visualization on GPU", Pacific Graphics — the grid-based "virtual pipes" alternative.
- Wikipedia — [Erosion / Stream power law](https://en.wikipedia.org/wiki/Erosion).
- Perlin, K. (2002) — "Improving Noise" (the fBm scaffold).

---

*Source: `procedural/worldgen/hydraulic.c`*
