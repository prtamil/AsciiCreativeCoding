# Pass 1 — geometry/string_art.c: Mathematical String Art

## Core Idea

N nails are arranged in a circle. Threads connect nail `i` to nail `round(i × k) mod N`. As `k` drifts continuously from 2 upward, the pattern morphs through a sequence of named mathematical curves: cardioid (k=2), nephroid (k=3), deltoid (k=4), astroid (k=5). The drift slows near integer k values so the user can see each shape clearly, then accelerates between integers.

## The Mental Model

Imagine putting 60 nails in a circle. Take a thread, start at nail 0, then go to nail 0×2=0. Then from nail 1 go to nail 2. From nail 2 go to nail 4. From nail 3 go to nail 6. Continuing, nail 30 goes to nail 60 = nail 0. When you're done, every nail has exactly two threads meeting at it, and the envelope of all the threads forms a heart shape — a cardioid. Change the rule to "go to 3× the nail number" and you get a nephroid (two-lobed). The k value controls the number of lobes.

## Thread Mathematics

For multiplier `k` and `N` nails, thread `i` connects:
- Start nail: `i`
- End nail: `round(i × k) mod N`

For integer k, this creates exact cusped curves. For non-integer k, the pattern looks like a smeared transition between the surrounding integers.

**Named shapes at integer k:**
| k | Shape | Cusps |
|---|---|---|
| 2 | Cardioid | 1 |
| 3 | Nephroid | 2 |
| 4 | Deltoid | 3 |
| 5 | Astroid | 4 |
| n | n-1 cusps | n-1 |

## Speed Modulation

The k drift speed is modulated to slow near integers:
```c
float dist = fabsf(g_k - roundf(g_k));   // 0 at integer k, 0.5 midway
float mult = 0.15f + 1.70f * (dist * dist * 4.0f);
g_k += K_SPEED * mult;
```

At `dist = 0` (integer k): `mult = 0.15` — very slow, shape holds for ~2 s.  
At `dist = 0.5` (midway): `mult = 0.15 + 1.70·1.0 = 1.85` — 12× faster, transition is brief.

This means the user sees each named shape clearly while the transitions between shapes are quick.

## Slope-Based Thread Characters

Each thread is a straight line from nail `i` to nail `j`. To draw it, the slope determines which character to use:

```c
float vs = fabsf(dy * 2.0f / dx);   // vertical/horizontal ratio, ×2 for aspect
if      (vs < 0.577f) ch = '-';     // nearly horizontal
else if (vs > 1.732f) ch = '|';     // nearly vertical
else if (sign matches) ch = '/';    // diagonal
else                  ch = '\\';    // other diagonal
```

The `×2` factor compensates for terminal cells being approximately twice as tall as wide. Without aspect correction, a 45° geometric line would appear nearly vertical on screen.

## Visual Scaffolding

**Circle rim:** `RIM_STEPS = 2000` sample points on the circle are drawn as `·` characters, giving a clear circular boundary.

**Nail markers:** each nail position is drawn as bold `o`, providing visible landmarks that match the geometric concept of "nails on a board."

**Rainbow threads:** 12 fixed 256-color indices cover the spectrum. Thread `i` uses `CP_T0 + (i % 12)`. With N=60 nails and 12 colors, every 5th nail shares a hue; with N=200 the colors repeat ~16× around the circle.

## Non-Obvious Decisions

### Why speed modulation rather than just stopping at integers?
Stopping and restarting would feel jerky. Continuous modulation keeps the animation fluid while still giving long pauses at the interesting shapes.

### Why `round(i × k) mod N` rather than `(i × k) mod N`?
For non-integer k, `i × k` is a float. Rounding to the nearest nail gives a clean thread endpoint. Without rounding, floating-point thread endpoints would be at fractional nail positions, requiring interpolation.

### Why `×2` in the aspect correction?
Most terminals have cells approximately 2:1 height/width ratio. A pixel-space slope of 1.0 (45° geometrically) appears at 2.0 in screen coordinates. The correction maps: geometric 1:1 → screen 1:2, so the character thresholds (0.577 and 1.732 = tan 30° and tan 60°) define equal 60° sectors of apparent angle.

## From the Source

**Algorithm:** String-art envelope construction using modular arithmetic. N nails evenly spaced on a circle. Thread `i` connects nail `i` to nail `round(i × k) mod N`. As `i` sweeps `0..N-1`, the set of N chords forms the envelope of a family of lines.

**Math:** The envelope of chords `nail_i → nail_{k·i mod N}` is a hypocycloid/epicycloid depending on k:
- k=2: cardioid (1 cusp), k=3: nephroid (2 cusps), k=4: deltoid (3 cusps), k=5: astroid (4 cusps)
Fourier series connection: the cardioid appears in complex power series, and the Mandelbrot set's main body is bounded by the same curve. Non-integer k creates transitional (blended) shapes between named curves, giving the smooth drift animation.

**Rendering:** Chord endpoints mapped from polar (`angle = 2πi/N`) to terminal coordinates with aspect correction. Character for each chord selected by slope: `|dy/dx| < 0.5` → `'-'`, `> 2` → `'|'`, else `'/'` or `'\\'`.

---

## Key Constants

| Constant | Effect |
|---|---|
| N (nail count) | More nails → denser pattern, more detail at large k |
| K_SPEED | Base drift speed of k per frame |
| RIM_STEPS | More → smoother circle rim |
| N_TCOLS (12) | Color cycle length; fewer → more visible repetition |
