# Concept — `procedural_star_field_parallax_noise_showcase.c`: Parallax Star Field

## Core Idea

Stack N flat **sheets of stars** at different virtual depths and scroll each one at a different speed. The closest sheet flies past quickly with bright `*` and `O` glyphs; the deepest sheet creeps along almost imperceptibly with faint `.` specks. The brain reads the speed differential as DEPTH — the scene appears three-dimensional even though every glyph is drawn at integer cell coordinates with no z-buffer.

No stars are stored. Each on-screen cell asks a hash function "is there a star at world coordinates (wx, wy) on layer L?", and the answer is deterministic and infinite — fly the camera for an hour and the same hash returns the same star at the same world coord every time. Memory cost is constant; world size is unbounded.

---

## The Mental Model

Imagine you are looking out the side window of a moving train. The fence right next to the tracks streaks past so fast it blurs. The trees in the middle distance pass by at a comfortable speed. The mountains on the horizon barely move. SAME train, SAME velocity — the visual speed of a thing depends purely on its distance.

Now replace the fence/trees/mountains with four invisible sheets of star stickers, all parallel to the screen. Slide each sheet sideways at its own speed. The closest sheet has fewer, brighter stars (because near things look big); the farthest sheet has many, faint stars (because far things blur into a sprinkle). That is the algorithm. Every visible "depth" effect comes from the speed and brightness differences between sheets.

Where do the stars on a sheet live? **Nowhere — they are summoned on demand.** For each sheet and each (x, y) in WORLD coordinates, ask a hash function "is there a star here?" and the hash either says yes or no in O(1) time. Same `(x, y, sheet)` → same answer, every time, forever. The world is a function, not a database.

---

## Algorithm in Steps

1. **INIT.** Pick `LAYER_SPEEDS[N_LAYERS]` (decreasing — closest layer first). Pick `LAYER_DENSITY[N_LAYERS]` (higher = sparser; "1 in `LAYER_DENSITY[L]` world cells of layer L hosts a star").

2. **EACH FRAME:**
   a. Advance the camera: `cam.x += cam.vx · dt; cam.y += cam.vy · dt`.
   b. Erase the screen.
   c. For each on-screen cell `(sx, sy)`:
      For each layer `L` from 0 (front) up to `N_LAYERS-1` (back):
      - `wx = floor(sx + cam.x · LAYER_SPEEDS[L])`
      - `wy = floor(sy + cam.y · LAYER_SPEEDS[L])`
      - `h  = hash3(wx, wy, L)`
      - if `(h % LAYER_DENSITY[L]) == 0`: draw the star — glyph, colour, brightness all encoded in different bits of `h`. Break (foreground wins).
      If still empty AND pattern == NEBULA: sample fBm at slow-scrolling world coords; map to glyph.

3. **PATTERN MODIFIERS** (applied during step 2c):
   - **TWINKLE**: multiply per-star brightness by `0.5 + 0.5·sin(2π·t·f + phase)`, where phase is bits [24..31] of `h`.
   - **WARP**: 5× scroll speed, swap glyphs to streak set.
   - **NEBULA**: draw fBm in empty cells (steps within 2c above).

4. **HUD**, present, sleep until next frame.

---

## Key Formulas

**Screen → world** (per layer L):
```
world_x = floor(screen_x + camera_x · layer_speed[L])
world_y = floor(screen_y + camera_y · layer_speed[L])
```

**Star existence test** (layer L, world cell (wx, wy)):
```
h = hash3(wx, wy, L)
star_present = (h mod LAYER_DENSITY[L]) == 0
```

**Star attributes** (decoded from same hash):
```
glyph_idx = (h >>  8) & 3        // index into LAYER_GLYPHS[L]
color_idx = (h >> 16) & 3        // index into theme palette
phase     = (h >> 24) / 255 · 2π // twinkle offset
```

**Twinkle** (PATTERN_TWINKLE):
```
brightness = 0.5 + 0.5 · sin(2π · t · TWINKLE_HZ + phase)
```
brightness in `[0, 1]` selects: < 0.30 skip, < 0.65 A_DIM, else A_BOLD.

**Hash function** (any 3-int → 32-bit, mix-of-multiplies):
```
h  = wx · 73856093 ^ wy · 19349663 ^ L · 83492791
h ^= h >> 16; h *= 0x85ebca6b
h ^= h >> 13; h *= 0xc2b2ae35
h ^= h >> 16
```

**Nebula** (PATTERN_NEBULA), per empty cell:
```
nx = sx + cam.x · NEBULA_SCROLL
ny = sy + cam.y · NEBULA_SCROLL
n  = fbm(nx · NEBULA_SCALE, ny · NEBULA_SCALE, 4 octaves)
```

---

## Edge Cases and Pitfalls

- **HASH QUALITY.** The 3 large odd primes (73856093, 19349663, 83492791) are the standard "spatial-hash" trio from Teschner et al. 2003. Replacing them with arbitrary numbers introduces visible diagonal stripes — the hash output becomes correlated along `wx + wy`. The avalanching multiply-shift after the XOR breaks the residual structure.

- **LAYER ORDER.** Layer 0 is the FRONT (fastest, brightest). Iterate **front-to-back** with early-out: the first star found wins. Iterating back-to-front and overwriting lets slower hidden stars "leak through" foreground gaps, ruining the depth cue.

- **FLOOR ON NEGATIVE.** As the camera moves toward +x, `world_x` becomes progressively LARGER. If the camera ever moves toward −x (or screen y exceeds `cam.y`), world coordinates can go negative. Use `floorf()` not `(int)` cast — the latter truncates toward zero and introduces a 1-cell discontinuity at the origin.

- **CAMERA OVERFLOW.** `(int)floorf(huge_float)` is undefined when the float exceeds `INT_MAX`. With `cam.x` growing at 8 cells/sec, `INT_MAX` is reached after ~8.5 years of continuous run. Acceptable for a demo; not for a saved-state simulator.

- **LAYER SPEED CHOICE.** If two layers have the SAME speed, they paint at the same rate and no parallax happens between them — they look like one fat layer. Speeds should be roughly geometric, e.g. `1.0 / 0.45 / 0.18 / 0.06` — each layer noticeably slower than the one in front.

- **NEBULA SCROLL VS LAYER SCROLL.** The nebula is conceptually behind layer 3 (deepest). Its scroll speed must therefore be the SLOWEST. Default `NEBULA_SCROLL` is 0.03 — half of the deepest star layer.

- **INFINITE-LOOK BREAKS AT 256.** The Perlin permutation table is 256 entries (duplicated → 512). The noise pattern REPEATS every 256 noise-coord units. With `NEBULA_SCALE=0.04` that is a 6,400-cell period — far larger than any terminal, but visible if you fly for hours.

- **WARP STREAK OVERSHOOT.** In WARP mode the streak length is fixed at 3 cells. If a streak extends past the right edge, the truncated chars vanish — that is correct; the bounds check inside the draw loop handles it.

---

## How to Verify

- **Pause** (space). The image freezes. Press space again. The image resumes from EXACTLY where it stopped — no jump. Verifies the fixed-step accumulator.

- Press **`r`** at any time. The camera snaps to the origin and the SAME star pattern that was visible at startup re-appears (as long as you have not changed the seed). Verifies hash determinism.

- In **STARFIELD** pattern, count the rough star density on the foreground layer's flow — should be ≈ `1 / LAYER_DENSITY[0]` ≈ 1 in 22 cells (about 4–5% of the screen). The deepest layer should be denser (≈ 1 in 7), but its glyphs are dim `.`, so it looks like a faint sprinkle. If layer 0 looks DENSER than layer 3, your front-to-back scan order is reversed.

- Switch to **TWINKLE**. Watch one bright star for 3 seconds. It should fade in and out smoothly with a period of ≈ 2 s and a different phase from its neighbours. If every star pulses in sync, the per-star phase isn't being read from the hash.

- Switch to **NEBULA**. The empty regions fill with smoothly-shaded cloud glyphs that drift slowly leftward (camera goes right, world appears to slide left). The clouds should be SLOWER than even the deepest star layer.

- Switch to **WARP**. The closest stars become 3-cell streaks; deeper layers stay as small marks. Press `r` — the streaks vanish (no history kept; re-derived every frame). This proves the streaks are not painted into a buffer that survives reset.

---

## References

- Teschner et al. (2003) — "Optimized Spatial Hashing for Collision Detection of Deformable Objects", VMV 2003 (the three primes used in `hash3`).
- Wikipedia — [Parallax scrolling](https://en.wikipedia.org/wiki/Parallax_scrolling).
- Inigo Quilez — "Hash without Sine", smooth integer noise (https://iquilezles.org/articles/morenoise/).
- Red Blob Games — Noise functions and map generation introduction.
- Perlin, K. (1985) — "An Image Synthesizer", SIGGRAPH (the fBm scaffold used for the NEBULA backdrop).

---

*Source: `procedural/worldgen/procedural_star_field_parallax_noise_showcase.c`*
