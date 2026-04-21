# Concept: STFT Spectrogram Visualizer

## Pass 1 — Understanding

### Core Idea
Visualise the time-frequency content of a signal using the Short-Time Fourier Transform (STFT). A window slides over the signal; each window position yields one FFT column. The columns are assembled into a 2-D image: x=time, y=frequency, brightness=magnitude. The result is a spectrogram — simultaneously showing when and at what frequency energy is present.

### Mental Model
The FFT of the whole signal tells you all the frequencies present, but not when. The STFT chops the signal into overlapping short windows and FFTs each one. Each window gives a spectrum "snapshot" at that moment. Stack these snapshots side by side and you have a spectrogram. Sinusoids appear as horizontal bright lines. A chirp appears as a diagonal line (frequency rising over time). A drum hit appears as a vertical bright stripe (impulse energy across all frequencies, concentrated in time).

### Key Equations
```
STFT: X(t, f) = Σₙ x[n + t·H] · w[n] · e^{−j2πfn/N}

Hann window: w[n] = 0.5 · (1 − cos(2πn/N))

Cooley-Tukey butterfly:
  a' = a + W·b
  b' = a − W·b
  where W = e^{−j2πk/M} for stage of size M

Magnitude in dB: L = 20·log10(|X| / ref)
```

### Data Structures
- `float buf[FFT_SIZE]`: windowed signal buffer
- `complex float fft[FFT_SIZE]`: complex FFT output
- `float spec[MAX_COLS][FFT_SIZE/2]`: ring buffer of magnitude columns
- `int spec_head`: next write position
- `SigGen`: phase accumulators + signal parameters

### Non-Obvious Decisions
- **Ring buffer display**: Store FFT columns in a circular buffer. Write new column at head; display reads (head+col)%total from oldest to newest. O(1) update vs O(N) column shift.
- **dB scale not linear**: Linear magnitude compresses 99% of the dynamic range into the bottom 1% of the display. dB = 20·log₁₀(|X|) spreads the range evenly. DB_FLOOR=−80 dB = 10,000:1 amplitude ratio.
- **Window choice tradeoff**: Wide main lobe (Blackman) cannot distinguish two close frequencies. Narrow main lobe (Rect) has high sidelobes that mask weak signals near strong ones. Match window to the analysis goal.
- **Phase accumulator not table lookup**: `phase += 2π·f/SR` wraps naturally at 2π. A large table would need integer modulo; accumulator just uses `fmodf`. For very long signals (>10⁶ samples) float precision erodes — reset the phase periodically.
- **75% overlap**: HOP_SIZE = FFT_SIZE/4. Each new column shares 75% of samples with the previous. This over-samples in time to avoid "streaks" in the spectrogram when events span fewer samples than one hop.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| SAMPLE_RATE | 8000 | samples/sec (Nyquist = 4000 Hz) |
| FFT_SIZE | 512 | must be power of 2; Δf = SR/N = 15.6 Hz |
| HOP_DIV | 4 | hop = FFT_SIZE/HOP_DIV (75% overlap) |
| DB_FLOOR | −80 | minimum dB shown (80 dB range) |
| SMOOTH_TAU | 200 | peak smoothing time constant (columns) |

### Open Questions
- What happens to frequency resolution if you halve FFT_SIZE?
- Why must FFT_SIZE be a power of 2 for Cooley-Tukey?
- What does the uncertainty principle say about a Gaussian window?

## From the Source

**Algorithm:** STFT with Cooley-Tukey iterative radix-2 DIT FFT. Bit-reversal permutation. Four window functions. Ring-buffer scrolling spectrogram. Signal generator: sine, multi-tone, AM, FM, chirp, noise.

**Physics/References:** FFT: Cooley & Tukey (1965). Windowing: Harris "On the Use of Windows for Harmonic Analysis with the DFT" (1978). STFT: Gabor (1946). AM/FM sidebands: Carson (1922) for AM; Armstrong (1936) for FM; Bessel functions for FM sidebands.

**Math:** Cooley-Tukey reduces N-point DFT from O(N²) to O(N·log₂N). For N=512: 512²=262,144 vs 512×9=4,608 multiplies — 57× speedup. Hann window sidelobe: −31 dB vs Rect −13 dB — 6.3× better sidelobe suppression.

**Performance:** O(N·log₂N) per column. N=512: ~4k complex multiplies. At HOP=128 samples, SR=8000: 62.5 columns/sec, one FFT per 16 ms — trivial at 60 fps.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `buf[FFT_SIZE]` | `float[N]` | ~2 KB | windowed signal |
| `fft[FFT_SIZE]` | `complex float[N]` | ~4 KB | FFT input/output |
| `spec[COLS][N/2]` | `float[C×N/2]` | ~512 KB | ring buffer of magnitudes |
| `win[FFT_SIZE]` | `float[N]` | ~2 KB | precomputed window |
| `sig` | `SigGen` | 1 struct | signal generator state |

---

## Pass 2 — Implementation

### Pseudocode
```
each frame:
    # generate HOP_SIZE new samples
    for i in 0..HOP_SIZE:
        sample = siggen_next(&sig)
        shift_buffer(buf)    # slide window by 1

    # window and FFT the current buffer
    windowed[n] = buf[n] * win[n]
    fft_inplace(windowed)    # Cooley-Tukey in-place

    # compute magnitude column
    for k in 0..N/2:
        mag = 2*|windowed[k]| / N   # two-sided → one-sided
        db  = 20*log10f(mag + EPS)
        spec[spec_head][k] = db
    spec_head = (spec_head + 1) % spec_cols

fft_inplace(x, N):
    # Step 1: bit-reversal permutation
    for i in 0..N:
        j = bitrev(i, log2(N))
        if j > i: swap(x[i], x[j])

    # Step 2: butterfly stages
    for s in 1..log2(N):
        M = 1 << s          # butterfly block size
        W_step = exp(-j*2π/M)
        W = 1+0j
        for k in 0..M/2:
            for m = k, k+M, k+2M, ...:
                t = W * x[m + M/2]
                x[m + M/2] = x[m] - t
                x[m]       = x[m] + t
            W *= W_step
```

### Module Map
```
§1 config      — SAMPLE_RATE, FFT_SIZE, HOP_DIV, DB_FLOOR, window type
§2 window      — build_window() Rect/Hann/Hamming/Blackman coefficients
§3 FFT         — fft_inplace() bit-reversal + butterfly stages
§4 SigGen      — siggen_init/next: sine, AM, FM, chirp, noise
§5 spectrogram — compute column, ring buffer head advance
§6 render      — ring buffer read, dB → color, frequency grid lines
§7 HUD         — dominant freq, window type, signal mode, scale
§8 app         — main loop, signal/window/FFT-size controls
```

### Data Flow
```
SigGen → samples → shift into buf
buf × win → windowed → fft_inplace → complex spectrum
|spectrum|² → magnitude → dB → spec[spec_head]
spec ring → render columns → screen (time scrolls left→right)
```
