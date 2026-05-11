# signal — DSP from the bottom up, animated

A reference folder for **discrete-time signal processing**: 10
self-contained C programs that take a 1-D buffer of samples, transform
it in some way, and paint the result live as bar charts or waterfalls.
Every file is built around the same primitive — **a time-series buffer
and a transform that maps it to either another time-series or a
spectrum** — and the folder's job is to teach the foundational moves
(DFT, FFT, IDFT, convolution, FIR filtering, windowing, the sampling
theorem) with side-by-side visual proof.

The pedagogical bet of the folder: **DSP is one short list of
operations**, and seeing each one drawn on the terminal makes the math
stick in a way that a textbook chapter cannot. The visual sub-genre at
play is `signal/epicycles.c`, `signal/fourier_draw.c` and
`signal/fourier_shapes.c` — the rotating-arm reconstructions popularised
by Daniel Shiffman's "Nature of Code" and 3Blue1Brown's Fourier video,
implemented from scratch.

If you read **only one file**, read [`dft_helloworld.c`](dft_helloworld.c)
— the textbook O(N²) DFT with no FFT trickery, side by side with the
input signal. Every other file in the folder is downstream of it.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
3. [File index](#file-index)
4. [Building and running](#building-and-running)
5. [Adding a new DSP demo](#adding-a-new-dsp-demo)

---

## How to read this folder

The recommended path goes from *sampling* up to *2-D Fourier drawing*,
with three side-quests into time-domain filtering. Each step changes one
piece of the previous file.

```
                       SAMPLING LAYER
                       ──────────────
   1.  aliasing.c                 — what is a sample?  Nyquist & fold-back

                       FREQUENCY DOMAIN
                       ────────────────
   2.  dft_helloworld.c           — textbook O(N²) DFT, the keystone file
            │
            ▼
   3.  fft_helloworld.c           — same DFT, computed via Cooley-Tukey
            │                       (verified against #2 every frame)
            ▼
   4.  fft_vis.c                  — scaled-up FFT: 7 modes, 4 windows,
                                     spectrogram waterfall
            │
            ▼
   5.  idft_helloworld.c          — inverse DFT + spectrum manipulation
                                     (low-pass, high-pass, mag-only, phase-only)

                       TIME-DOMAIN FILTERING
                       ─────────────────────
   6.  convolution_helloworld.c   — the sliding multiply-accumulate operation
            │
            ▼
   7.  fir_filter.c               — design a filter (windowed sinc), apply it
                                     by convolution; linear-phase property

                       FOURIER IN 2-D
                       ──────────────
   8.  epicycles.c                — rotating-arm reconstruction of 20 preset shapes
            │
            ├──▶ 9.  fourier_shapes.c   — same engine, 21 shapes + Parseval energy bar
            │
            └──▶ 10. fourier_draw.c     — same engine, user-drawn path
```

**Prerequisites graph.** Each file's header lists *Study alongside:*
pointers. The two non-obvious dependencies:

* `idft_helloworld.c` (file 5) teaches the **convolution-multiplication
  duality** in tutorial T9, which is the bridge to the time-domain
  filtering files (6, 7). Don't skip it.
* `epicycles.c` (file 8) is the pedagogical workhorse for the 2-D
  Fourier files. `fourier_shapes.c` and `fourier_draw.c` re-use its
  arm-chain machinery — read 8 first, then 9, then 10.

**Background you need.** Comfort with `float` arithmetic, complex
numbers (every file in this folder builds its own tiny complex-number
helper — no `<complex.h>`), and the framework conventions
(`erase()/doupdate()`, fixed-step accumulator) from
[`../grids/README.md`](../grids/README.md). No prior DSP background
assumed; every file's MENTAL MODEL block teaches the relevant idea from
scratch.

---

## The unifying primitive

Every file in this folder operates on the same two-step pipeline:

```
   sample buffer                  TRANSFORM                    output buffer
   ─────────────                  ─────────                    ──────────────
   x[0], x[1], ... , x[N-1]   ──▶  DFT / FFT / IDFT /     ──▶  y[0], y[1], ...
   (real or complex)              convolution / filter         (spectrum or signal)
```

`N` is small in this folder (32–256) so even O(N²) runs effortlessly at
60 fps. The buffer is **reset and recomputed every frame** — there is
no streaming, no DSP pipeline state. The whole point is that each frame
is a self-contained transform, drawn and labelled so you can see what
the math did.

The five transforms used across the folder:

| Transform      | Definition (informal)                              | Files                                                       |
|----------------|----------------------------------------------------|-------------------------------------------------------------|
| **Sampling**   | `x[n] = continuous_x(n / fs)`                      | `aliasing.c`                                                |
| **DFT**        | `X[k] = Σ_n x[n] · exp(-j·2π·kn/N)`                | `dft_helloworld.c`, `epicycles.c`, `fourier_shapes.c`, `fourier_draw.c` |
| **FFT**        | same as DFT, computed in O(N log N) via butterflies | `fft_helloworld.c`, `fft_vis.c`                             |
| **IDFT**       | `x[n] = (1/N) · Σ_k X[k] · exp(+j·2π·kn/N)`        | `idft_helloworld.c`                                         |
| **Convolution**| `y[n] = Σ_i h[i] · x[n - i]`                       | `convolution_helloworld.c`, `fir_filter.c`                  |

The two transforms are linked by the **convolution-multiplication
duality**, derived inline in `idft_helloworld.c` (T9):

```
   convolution in time  ⇔  multiplication in frequency

       y = h * x       ⇔     Y[k] = H[k] · X[k]
```

This is why the FIR-filter file can choose its filter shape in the
*frequency* domain (windowed sinc) and apply it in the *time* domain
(slide-and-multiply convolution).

**The FFT butterfly**, as ASCII. The radix-2 DIT butterfly is the
operation in `fft_helloworld.c` (§8) and `fft_vis.c` (§4):

```
       stage s (block of 2^s pairs)

       a   ────────●──────────► A
                   │ +
                   │
                   │ -
       b   ──×W────●──────────► B

       A = a + W·b
       B = a − W·b           W = exp(-j·2π·k / 2^s)
```

`log₂(N)` such stages, preceded by one bit-reversal permutation, gives
the full FFT. `fft_helloworld.c` verifies the output bin-by-bin against
the textbook O(N²) DFT every single frame — when the HUD says
"`FFT == DFT`" it's not metaphor.

---

## File index

| File                        | Lines  | Subsystem   | DEMO line — what it visually does                                                                       |
|-----------------------------|--------|-------------|---------------------------------------------------------------------------------------------------------|
| `aliasing.c`                | ~1400  | sampling    | True continuous sine vs sampled version; as `f` sweeps past Nyquist the sampled wave folds back.        |
| `dft_helloworld.c`          | ~1500  | DFT         | Cosine `cos(2πfn/N)` on top, magnitude spectrum `|X[k]|` below; sweep `f` and watch the spike walk.     |
| `fft_helloworld.c`          | ~1650  | FFT         | Same demo as above, but spectrum computed by Cooley-Tukey FFT and verified against DFT live every frame.|
| `fft_vis.c`                 | ~2200  | FFT         | Three-panel: time waveform / spectrum / scrolling spectrogram. 7 signals, 4 window functions.            |
| `idft_helloworld.c`         | ~1800  | IDFT        | Round-trip / low-pass / high-pass / magnitude-only / phase-only reconstruction. Five modes via `m`.      |
| `convolution_helloworld.c`  | ~1500  | filtering   | Kernel slides across input; watch each output sample being computed live. 6 kernels, 5 signals.         |
| `fir_filter.c`              | ~1700  | filtering   | Build a windowed-sinc low/high/band-pass filter, apply by convolution, show impulse + magnitude + phase.|
| `epicycles.c`               | ~2300  | DFT in 2-D  | Chain of rotating arms traces 20 preset shapes (heart, star, butterfly, trefoil knot, ...).             |
| `fourier_draw.c`            | ~2100  | DFT in 2-D  | Scribble a path with arrows, press ENTER, watch the epicycle chain redraw it from scratch.              |
| `fourier_shapes.c`          | ~2000  | DFT in 2-D  | 21 preset shapes, Parseval energy bar showing cumulative power captured by the active arms.             |

Line counts include the pedagogical block (HOW TO READ, GUIDED
TUTORIAL, inline teaching prose) — the algorithm itself is typically
200-500 lines per file.

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra signal/<file>.c -o <name> -lncurses -lm
```

All ten files use `-lm` (every file needs `sinf`, `cosf`, complex
arithmetic). The project is strict about `-Wall -Wextra` clean — every
file compiles with zero warnings.

**Universal keys** (always present):

| Key             | Action                                  |
|-----------------|-----------------------------------------|
| `q` / `ESC`     | quit                                    |
| `space`         | pause                                   |
| `r`             | reset                                   |
| `t` / `T`       | next / previous theme                   |
| `+` / `-`       | tune the headline parameter (freq, cutoff, arms, ...) |
| `,` / `.`       | fine-tune the same parameter (× 0.1)    |
| `a`             | toggle auto-sweep / manual mode         |
| `d`             | toggle small debug overlay              |
| `D`             | toggle large numeric debug overlay      |

**File-specific keys** (vary):

| Key             | Where                                                | Action                                              |
|-----------------|------------------------------------------------------|-----------------------------------------------------|
| `s`             | `convolution_helloworld.c`, `fir_filter.c`           | cycle input signal                                  |
| `k`             | `convolution_helloworld.c`                           | cycle filter kernel                                 |
| `w`             | `fir_filter.c`, `fft_vis.c`                          | cycle window function                               |
| `f`             | `fir_filter.c`                                       | cycle filter type (low / high / band)               |
| `m`             | `idft_helloworld.c`                                  | cycle reconstruction mode (round-trip / LP / HP / mag / phase) |
| `n` / `p`       | `epicycles.c`, `fourier_shapes.c`                    | next / previous shape                               |
| `o`             | `fourier_draw.c`                                     | toggle auto-close                                   |
| ENTER           | `fourier_draw.c`                                     | commit scribble → switch to PLAY mode               |

---

## Adding a new DSP demo

If you want to add (say) a 2-D FFT, a Discrete Cosine Transform, a
spectrogram synthesiser, or a phase vocoder:

1. **Pick the transform.** Linear (DFT / DCT / Hadamard), non-linear
   (CQT / wavelet), or operation on a different state (autocorrelation,
   cepstrum)?  All fit the buffer-in / buffer-out template.
2. **Decide on the visual primitive.** Bar chart (spectrum), waterfall
   (time-varying spectrum), pair of stacked panels (time + freq), or
   rotating-arm chain (2-D Fourier). 7 of the 10 files here use bar
   charts; pick the closest existing template.
3. **Compute with `N` small.** Keep `N` ≤ 256 for any O(N²) transform;
   even at 60 fps the math is microseconds. The terminal is the
   bottleneck.
4. **Verify against a slow reference.** `fft_helloworld.c` runs the
   slow DFT alongside the fast FFT every frame and prints the max
   bin-by-bin error. Copy this pattern for any optimised transform.
5. **Write the complex-number helper inline.** Every file in this
   folder defines its own `ComplexNumber` struct + ~6 helpers. The
   project rule is no shared headers; the cost of the duplication is
   ~30 lines per file in exchange for full self-containment.
6. **Copy the closest template:**
   * Magnitude-spectrum bar chart → copy `dft_helloworld.c`
   * Spectrogram waterfall → copy `fft_vis.c`
   * Time-domain sliding filter → copy `convolution_helloworld.c`
   * 2-D arm chain → copy `epicycles.c`
7. **Add CONCEPTS + MENTAL MODEL** per the project's
   [CLAUDE.md](../CLAUDE.md) template. Both blocks are mandatory.
   References: 2-5 per file. Standard references for DSP: Smith's
   "Scientist and Engineer's Guide to DSP", Oppenheim & Schafer
   "Discrete-Time Signal Processing", 3Blue1Brown's Fourier video.
8. **Verify:** `-Wall -Wextra` clean, stable 60 fps, `q`/`ESC` exits
   cleanly, `SIGWINCH` doesn't crash, HUD shows fps + parameters + state.

---

## Cross-references

* [`../grids/README.md`](../grids/README.md) — `GridCtx` and the
  cell ↔ screen mapping; the bar-chart and waterfall primitives reuse
  the same pixel-to-cell quantisation logic.
* [`../animation/`](../animation/) — framework conventions
  (fixed-step accumulator, `erase()/doupdate()`).
* [`../procedural/`](../procedural/) — Perlin / curl-noise field
  generators; useful prior to writing a 2-D FFT demo since 2-D noise is
  the canonical test input.
* [`../raster/`](../raster/) — shares the bar-rendering vocabulary
  (`mvaddch` of `'#'`, `'|'`, `' '`); same idea generalised to triangles.
* [`../documentation/Master.md`](../documentation/Master.md) — long-form
  essays on selected algorithms; chapters on Fourier and DSP relevant
  here.
