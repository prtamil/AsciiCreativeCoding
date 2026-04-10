# Pass 1 — artistic/epicycles.c: Fourier Epicycles

## Core Idea

Any closed curve can be approximated by a chain of rotating circles (epicycles). The sizes and speeds of the circles come from the Discrete Fourier Transform (DFT) of the curve. The program samples a shape into 256 complex points, computes the DFT, sorts the resulting "arms" from largest to smallest, and then animates the chain rotating — the tip of the last arm traces the original shape. One arm is added automatically every few frames, showing how the approximation builds up from a blurry blob to the exact shape.

## The Mental Model

Imagine a clock hand (one rotating circle). Its tip traces a perfect circle. Now attach a smaller hand to the tip of the first — now the tip traces an epicycle (a circle tracing a circle). With enough hands of the right lengths and speeds, the tip can trace any closed curve. This is the core idea of Fourier series: any shape is just the right combination of circular motions at different frequencies.

## Algorithm

1. **Sample the shape:** compute `N_SAMPLES = 256` complex points `z[k] = x(t) + i·y(t)` for parameter `t ∈ [0, 2π]`, scaled to fit the terminal.

2. **DFT:** `Z[n] = Σ_{k=0}^{N-1} z[k] · exp(−2πi·n·k/N)`. Each `Z[n]` gives a complex coefficient. `|Z[n]|/N` is the arm radius; `arg(Z[n])` is the starting angle; `n` (or its wrapped frequency) is the angular speed.

3. **Sort by amplitude:** the largest-radius arms are added first, so the approximation builds from the dominant shape to fine details.

4. **Animate:** at angle `φ`, the arm chain position is:
   ```
   tip = Σ_{j=0}^{n_arms-1} (|Z[j]|/N) · exp(i · (freq_j · φ + arg Z[j]))
   ```
   Each arm rotates at its own frequency. The pivot of arm `j+1` is the tip of arm `j`.

5. **Auto-add:** every `AUTO_ADD_FRAMES = 12` frames, `n_arms` increments, adding the next DFT term. Starting with 1 arm shows how badly a single circle approximates the shape; reaching all 256 shows exact reconstruction.

## Five Shapes

| Shape | Parametric form | Key feature |
|---|---|---|
| Heart | `x = 16sin³t`, `y = 13cos t − 5cos 2t − 2cos 3t − cos 4t` | Cusp at bottom |
| Star | `r = 1 + 0.5·cos(5t)`, polar | 5-fold symmetry |
| Trefoil | `r = cos(3t)`, polar with offset | 3-lobed shape |
| Figure-8 | `x = sin t`, `y = sin 2t / 2` | Lemniscate |
| Butterfly | `r = exp(sin t) − 2cos 4t + sin⁵((2t−π)/24)` | Irregular |

## Subpixel Coordinates

Arm positions are stored in pixel space (terminal cells × `CELL_W = 8`). This is the same aspect-correction scheme as flocking.c. At draw time, pixel coordinates are divided by `CELL_W` and `CELL_H` to get terminal cell indices. Without this, circles would appear as ellipses on the terminal (cells are taller than wide).

## Non-Obvious Decisions

### Why sort by amplitude?
If arms were sorted by frequency, the first arm added would be the fastest-rotating (highest frequency), which contributes fine detail. The resulting shape would look wrong until almost all arms were added. Sorting by amplitude means the first arm captures the dominant shape, giving a recognisable approximation from the very first few arms.

### Why complex number representation?
A complex number `x + i·y` represents a 2-D point. Multiplying by `exp(iθ)` rotates the point. This makes the DFT and arm-chain formulas compact and natural — no separate sin/cos arrays needed.

### Why 256 samples?
256 is a power of 2, making FFT computation efficient if needed. For the visual output, 256 points gives enough resolution for smooth curve reconstruction at typical terminal sizes.

## Key Constants

| Constant | Effect |
|---|---|
| N_SAMPLES | More samples → finer DFT resolution, more epicycles available |
| TRAIL_LEN | Longer trail shows more of the traced path before fading |
| AUTO_ADD_FRAMES | Fewer → faster convergence demo; more → more time to appreciate each stage |
| SHAPE_SCALE | Fraction of terminal size for the shape; too large → epicycles go off-screen |
| N_CIRCLES | How many orbit circles are drawn when toggled on |
