# Concept: FFT Visualiser (Cooley-Tukey)

## Pass 1 — Understanding

### Core Idea
Visualise the Cooley-Tukey radix-2 DIT (Decimation-In-Time) FFT side-by-side with the signal it analyses. The top panel shows the time-domain input x[n] as a bar chart; the bottom panel shows the frequency-domain magnitudes |X[k]| after the transform. You compose a signal from 3 sine waves and watch peaks appear/disappear in real time as you tune frequency and amplitude.

### Mental Model
The DFT asks: "which sine/cosine frequencies are present in this signal, and how strongly?" The FFT gives the exact same answer as the O(N²) DFT but exploits the symmetry of the complex exponentials to do it in O(N log N). The trick is that the DFT of an N-point signal can be split into two DFTs of N/2 points (even/odd samples), combined with "twiddle factors" — phase-rotation terms W_N^k. Recurse until N=1 (trivial DFT). That recursion is the butterfly diagram.

### Key Equations

**DFT definition:**
```
X[k] = Σ_{n=0}^{N-1} x[n] · exp(−2πi·k·n/N)    k = 0 … N−1
```

**Twiddle factor:**
```
W_N^k = exp(−2πi·k/N) = cos(2πk/N) − i·sin(2πk/N)
```

**Cooley-Tukey radix-2 butterfly:**
```
X[k]     = E[k] + W_N^k · O[k]       k = 0 … N/2−1
X[k+N/2] = E[k] − W_N^k · O[k]
```
where E[k] = DFT of even-indexed samples, O[k] = DFT of odd-indexed samples.

**Bit-reversal permutation:** before the butterfly passes, reorder x[n] so that index n appears at position bit_reverse(n). This is what "DIT" means: the input is in bit-reversed order, output is in natural order.

**Magnitude:** `|X[k]| = sqrt(Re(X[k])² + Im(X[k])²)`

### Implementation-Specific Notes
- `N = 256` (power of 2 — required for radix-2)
- Signal built as sum of up to 3 sine waves with independent freq/amp
- Bit-reversal done with `__builtin_clz` / manual count
- Iterative bottom-up butterfly (no recursion — avoids stack overhead)
- Magnitude normalised by N/2 for display scaling
- Only N/2 unique frequency bins shown (bins N/2 … N−1 are symmetric conjugates)
- Frame rate: recompute FFT every display tick; input signal is regenerated from scratch each frame

### Data Structures
```
complex double buf[N];   // work buffer (in-place transform)
double x[N];             // time-domain signal
double X_mag[N/2];       // frequency-domain magnitudes
```

### Non-Obvious Design Decisions
- **Why iterative, not recursive FFT?** Recursive FFT has O(log N) stack depth and function-call overhead. Iterative bottom-up avoids both.
- **Why N=256?** Fits comfortably on a terminal and gives 128 frequency bins — enough resolution to see individual sine peaks clearly.
- **Frequency axis:** bin k corresponds to frequency k · (sample_rate / N). With sample_rate = N (artificial), bin k = frequency k. So the 3 sine components appear at exactly their frequency index.

### Open Questions to Explore
1. What happens to the frequency spectrum if you add a DC offset to the signal?
2. Try a square wave (sum of odd harmonics with 1/k amplitude). What does the spectrum look like?
3. The FFT assumes the signal is periodic. What happens when the signal frequency doesn't divide evenly into N? (spectral leakage)
4. How would you implement a real-time audio FFT using this same butterfly?
5. What is the Nyquist limit and why can you only display N/2 bins?

## From the Source

**Algorithm:** Cooley-Tukey radix-2 DIT (Decimation-In-Time) FFT. Recursively splits the N-point DFT into two N/2-point DFTs of even- and odd-indexed samples, then combines with twiddle factors W_N^k = exp(-2πi·k/N). Complexity: O(N log₂ N) multiplications vs O(N²) for the naive DFT — for N=128: 896 vs 16,384 operations.

**Math:** DFT linearity: sum of sinusoids → sum of delta spikes in the frequency domain. Parseval's theorem: Σ|x[n]|² = (1/N)·Σ|X[k]|² — total power is conserved between time and frequency domains.

**Performance:** N_FFT must be a power of 2 for the radix-2 butterfly. In-place bit-reversal permutation reorders samples before the butterfly stages, requiring no auxiliary buffer.

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_noise[N_FFT]` | `float[128]` | 512 B | white noise burst added to signal |
| `a.re[N_FFT]` | `float[128]` | 512 B | FFT real part (in-place) |
| `a.im[N_FFT]` | `float[128]` | 512 B | FFT imaginary part (in-place) |
| `a.sig[N_FFT]` | `float[128]` | 512 B | composite time-domain signal |
| `a.mag[N_FFT/2]` | `float[64]` | 256 B | magnitude spectrum (Nyquist bins) |
| `a.comps[N_COMPS]` | `Comp[3]` | 24 B | sine component frequency/amplitude/toggle |

---

## Pass 2 — Implementation

### Pseudocode

```
// 1. Build signal
for n in 0..N:
    x[n] = Σ (amp[i] * sin(2π * freq[i] * n / N))  for active components

// 2. Bit-reversal permutation
for n in 0..N:
    r = bit_reverse(n, log2(N))
    if r > n: swap(buf[n], buf[r])

// 3. Iterative butterfly
for s = 1 to log2(N):           // stage s
    m = 2^s                      // butterfly span
    W_m = exp(-2πi / m)          // twiddle factor base for this stage
    for k = 0 to N-1 step m:    // group start
        w = 1.0 + 0.0i
        for j = 0 to m/2 - 1:
            t = w * buf[k + j + m/2]
            u = buf[k + j]
            buf[k + j]       = u + t
            buf[k + j + m/2] = u - t
            w *= W_m

// 4. Magnitude
for k in 0..N/2:
    X_mag[k] = |buf[k]| / (N/2)

// 5. Draw
draw_bar_chart(x,     TOP_PANEL)
draw_bar_chart(X_mag, BOT_PANEL)
```

### Module Map
```
§1 config      — N=256, panel heights, freq/amp ranges
§2 clock       — fixed-timestep display loop
§3 signal      — sine wave generation, component toggle
§4 fft         — bit-reversal + iterative butterfly
§5 draw        — two-panel bar chart renderer
§6 app         — input handling, main loop
```

### Data Flow
```
components[] (freq, amp, active)
      │
      ▼
  build_signal() → x[N]
      │
      ▼
  fft_inplace()  → buf[N] (complex)
      │
      ▼
  magnitudes()   → X_mag[N/2]
      │
      ▼
  draw_panels()  → terminal
```

### Core Loop
```c
while (running) {
    handle_input();
    build_signal(x, N, components);
    memcpy(buf, x, N * sizeof(complex double));
    fft_inplace(buf, N);
    for (int k = 0; k < N/2; k++)
        X_mag[k] = cabs(buf[k]) / (N / 2.0);
    erase();
    draw_time_panel(x, N);
    draw_freq_panel(X_mag, N/2);
    draw_hud(components);
    doupdate();
    frame_sleep();
}
```

