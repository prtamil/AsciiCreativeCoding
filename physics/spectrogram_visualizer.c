/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * spectrogram_visualizer.c
 *
 * Sliding-Window FFT Spectrogram Visualizer
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  A scrolling time-frequency heatmap (spectrogram).
 *    X axis — time: newest data at right, oldest at left, scrolls left.
 *    Y axis — frequency: DC at bottom, Nyquist (SR/2) at top.
 *    Color  — intensity: black=silent, white=loud.
 *
 *  Six test signals (keys 1–6):
 *    SINE     — single pure tone: one bright horizontal line
 *    CHIRP    — linear frequency sweep: a diagonal bright streak
 *    MIXTURE  — tone + white noise: bright line on speckled background
 *    MULTI    — three simultaneous tones: three parallel horizontal lines
 *    AM       — amplitude modulation: carrier + two symmetric sidebands
 *    FM       — frequency modulation: carrier + multiple Bessel sidebands
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — TIME-FREQUENCY TRADEOFF  (Heisenberg Uncertainty)
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  You CANNOT simultaneously know exactly WHEN and at exactly WHAT
 *  FREQUENCY an event occurs.  This is not a limitation of the algorithm
 *  — it is a fundamental constraint of any time-frequency representation:
 *
 *       Δt · Δf  ≥  1 / (4π)     (Gabor / Heisenberg inequality)
 *
 *  In the Short-Time Fourier Transform (STFT):
 *    Time resolution:       Δt = hop_size / sample_rate   (seconds / column)
 *    Frequency resolution:  Δf = sample_rate / fft_size   (Hz / bin)
 *
 *  Their product:   Δt · Δf = hop_size / fft_size  (constant overlap fraction).
 *
 *  INCREASING FFT SIZE  (press N):
 *    → Δf shrinks: frequency lines become sharper, closely-spaced tones
 *      can be resolved (visible as two distinct lines instead of one blur).
 *    → Each column covers more time: a fast frequency sweep (CHIRP)
 *      appears smeared horizontally rather than a clean diagonal line.
 *
 *  DECREASING FFT SIZE  (press n):
 *    → Δt shrinks: transients appear pinpointed in time.
 *    → Δf grows: frequency lines thicken; tonal signals look fuzzy
 *      because many bins contribute to one tone.
 *
 *  Try CHIRP signal: large FFT gives a crisp line but blurry edges.
 *  Try MULTI signal: large FFT separates the three tones clearly.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — WINDOWING EFFECT
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  The FFT operates on a finite block of N samples.  This is equivalent
 *  to multiplying the infinite signal by a rectangular window w[n]=1.
 *  In the frequency domain, multiplication becomes convolution:
 *
 *       X_windowed(f)  =  X_true(f)  ⊛  W(f)
 *
 *  where W(f) is the DTFT of the window function.
 *
 *  RECTANGULAR  (no window): W(f) = sinc — narrow main lobe (+good freq
 *    resolution) but sidelobes only −13 dB below main peak (+bad leakage).
 *
 *  HANN:  w[n] = ½(1 − cos(2πn/(N−1)))
 *    Sidelobes at −31 dB.  Main lobe 2× wider.  Best general-purpose
 *    choice: good leakage rejection without sacrificing too much resolution.
 *
 *  HAMMING:  w[n] = 0.54 − 0.46·cos(2πn/(N−1))
 *    First sidelobe at −41 dB (better near-peak suppression than Hann).
 *    Outer sidelobes decay slightly slower.
 *
 *  BLACKMAN:  w[n] = 0.42 − 0.5·cos + 0.08·cos(4πn/(N−1))
 *    Sidelobes at −57 dB: excellent for detecting weak tones near strong
 *    ones.  Widest main lobe (~3× rect).
 *
 *  Press w to cycle windows.  On MULTI signal with RECT, you will see
 *  "skirts" (leakage smearing) below and above each tone; switch to
 *  HANN or BLACKMAN and they vanish.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — SPECTRAL LEAKAGE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Leakage occurs when a sinusoid's true frequency does NOT fall exactly
 *  on a DFT bin (i.e., the frequency is not an integer multiple of Δf).
 *
 *  Example:  N=256, SR=8000 Hz  →  Δf = 31.25 Hz.
 *    f=800 Hz → bin k = 800/31.25 = 25.6  (NOT an integer)
 *    The DFT "sees" a discontinuity where the periodic extension of the
 *    block doesn't match — like slicing a waveform mid-cycle and pasting
 *    it end-to-end.  The jump smears energy across all bins.
 *
 *  WHY WINDOWING HELPS:
 *    Tapering to zero at both edges removes the discontinuity.
 *    The smoother the window's edge, the smaller the sidelobes of W(f),
 *    and the less energy leaks into neighboring bins.
 *
 *  COST:
 *    Wider window sidelobes → two closely-spaced tones that appear as
 *    one blob rather than two resolved peaks.  Blackman's 3× wider main
 *    lobe means minimum resolvable spacing is 3·Δf instead of 1·Δf.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  Keys:
 *    q/ESC  quit          space   pause/resume
 *    1-6    signal type  (sine / chirp / mixture / multi / AM / FM)
 *    w      cycle window (rect / hann / hamming / blackman)
 *    n      decrease FFT size  →  better time resolution
 *    N      increase FFT size  →  better frequency resolution
 *    g      toggle frequency grid (500 / 1k / 2k / 4k Hz)
 *    r      reset
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/spectrogram_visualizer.c \
 *        -o spectrogram -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config  §2 clock  §3 color  §4 fft
 *  §5 window  §6 signal  §7 spectrogram
 *  §8 render_panel  §9 render_overlay
 *  §10 scene  §11 screen  §12 app
 * ─────────────────────────────────────────────────────────────────────
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * SAMPLE_RATE — simulated audio sample rate in Hz.
 *
 * This determines the real-world meaning of the frequency axis.
 * The Nyquist frequency (highest representable frequency) = SR/2 = 4000 Hz.
 * 8 kHz is the standard telephone audio bandwidth — every frequency
 * you see on the Y axis maps into the range 0..4000 Hz.
 * Does NOT affect wall-clock speed — the simulator runs as fast as the
 * display allows, compressing or stretching simulation time.
 * Changing this constant scales all displayed frequencies proportionally:
 * doubling SR doubles the Nyquist limit and halves the per-bin Hz value.
 */
#define SAMPLE_RATE  8000.0f    /* Hz */

/*
 * FFT_SIZES — the set of FFT window lengths available (keys n / N cycle through).
 *
 * ALL entries must be exact powers of 2.  The Cooley-Tukey radix-2 algorithm
 * (used in §4) requires this: it recursively splits the problem in half, and
 * that recursion only terminates cleanly when N is a power of 2.
 *
 * Frequency resolution at SR=8000 Hz:
 *   N=  64 → Δf = 8000/64   = 125.0 Hz per bin  (coarse, good time res)
 *   N= 128 → Δf = 8000/128  =  62.5 Hz per bin
 *   N= 256 → Δf = 8000/256  =  31.25 Hz per bin  (default — good balance)
 *   N= 512 → Δf = 8000/512  =  15.6 Hz per bin
 *   N=1024 → Δf = 8000/1024 =   7.8 Hz per bin  (fine, poor time res)
 *
 * As N doubles, frequency resolution improves by 2× but time resolution
 * halves (each FFT window covers twice as many samples = more simulation time).
 */
static const int FFT_SIZES[]  = { 64, 128, 256, 512, 1024 };
#define N_FFT_SIZES  5
#define FFT_IDX_DEF  2          /* default: FFT_SIZE=256 */

/*
 * HOP_DIV — controls the overlap between successive FFT windows.
 *
 * hop_size = FFT_SIZE / HOP_DIV
 * overlap fraction = 1 − 1/HOP_DIV
 *
 * With HOP_DIV=4:  hop = N/4  →  75% overlap (each new column shares
 * 75% of its samples with the previous column).
 *
 * Why 75%?  Higher overlap produces smoother scrolling on the time axis
 * because the spectrogram advances in small steps.  50% (÷2) is the
 * minimum needed to avoid gaps in the STFT; 75% is the standard choice
 * for audio spectrograms.  Increasing HOP_DIV further gives diminishing
 * returns and costs more CPU (more FFT calls per render frame).
 */
#define HOP_DIV      4          /* 75% overlap — good balance */

/*
 * STEPS_PER_FRAME — number of FFT columns computed per render frame.
 *
 * Each call to scene_tick() generates hop_size new samples and computes
 * one FFT column.  STEPS_PER_FRAME=2 means 2 new columns are pushed into
 * the ring buffer before the screen is redrawn, so the spectrogram scrolls
 * at 2 × (hop_size / SR) seconds of simulated audio per real frame.
 * Increase this to make the display scroll faster (more simulation time
 * per wall-clock second).
 */
#define STEPS_PER_FRAME  2

/* Grid size caps: keep display real-time on large terminals */
#define GRID_MAX_COLS  220
#define GRID_MAX_ROWS   55

/*
 * DB_FLOOR — the lowest dB value displayed (the "noise floor" of the display).
 *
 * −80 dB corresponds to a magnitude ratio of 10^(−80/20) = 10^−4 = 1/10000.
 * A 10000:1 dynamic range means a tone at full amplitude (0 dB) will still
 * show the noise floor of white noise spread across all bins.
 * Everything below DB_FLOOR is treated as silence (drawn black).
 * Making this more negative (e.g. −100) shows weaker signals but makes the
 * background noisier; less negative (e.g. −40) hides the noise floor but
 * compresses the dynamic range of interesting features.
 */
#define DB_FLOOR  -80.0f

/* Signal types */
#define SIG_SINE     0   /* pure tone at SINE_FREQ */
#define SIG_CHIRP    1   /* linear sweep CHIRP_F0 → CHIRP_F1 */
#define SIG_MIXTURE  2   /* tone + white noise */
#define SIG_MULTI    3   /* three simultaneous tones */
#define SIG_AM       4   /* amplitude modulation */
#define SIG_FM       5   /* frequency modulation */
#define SIG_COUNT    6

/* Window functions */
#define WIN_RECT     0
#define WIN_HANN     1
#define WIN_HAMMING  2
#define WIN_BLACKMAN 3
#define WIN_COUNT    4

/*
 * Signal parameters (Hz) — all frequencies are in simulated Hz relative to
 * SAMPLE_RATE.  They determine which FFT bins light up in the spectrogram.
 *
 * SINE_FREQ = 800 Hz
 *   → bin k = 800 / (8000/256) = 800 / 31.25 = 25.6  (non-integer!)
 *   → intentionally off-bin to demonstrate spectral leakage: the energy
 *     spreads into neighboring bins instead of appearing in a single bin.
 *   Switch windows (press w) to see leakage shrink with Hann/Blackman.
 *
 * CHIRP_F0 = 100 Hz, CHIRP_F1 = 3800 Hz, CHIRP_DUR = 3.0 s
 *   → sweeps from near-DC all the way to near-Nyquist over 3 simulated seconds.
 *   → produces a diagonal streak across the full height of the spectrogram.
 *   → With large FFT: streak is crisp but slightly blurred horizontally.
 *   → With small FFT: streak is sharp in time but blurry in frequency.
 *
 * MIX_TONE = 1200 Hz, noise amplitude = 40%
 *   → The tone appears as one bright horizontal line (energy concentrated).
 *   → White noise fills ALL bins at a low, uniform level.
 *   → Demonstrates how noise and tones look different in the spectrogram.
 *
 * MULTI_F1/F2/F3 = 440 / 1000 / 2800 Hz
 *   → Chosen to be NOT harmonically related (no common fundamental period).
 *   → Well-spaced so that even small FFT sizes can resolve them as three
 *     distinct lines rather than one merged blob.
 *
 * AM_CARRIER = 2000 Hz, AM_MOD_FREQ = 200 Hz
 *   → Amplitude modulation creates sidebands at carrier ± modulation freq:
 *     2000 − 200 = 1800 Hz  and  2000 + 200 = 2200 Hz.
 *   → Modulation index m = 0.8 → sideband amplitude = m/2 = 0.4 of carrier.
 *   → Visible in the spectrogram as exactly 3 bright horizontal lines.
 *
 * FM_CARRIER = 1500 Hz, FM_DEV = 600 Hz, FM_RATE = 8 Hz
 *   → Modulation index β = FM_DEV / FM_RATE = 600 / 8 = 75.
 *   → β >> 1 means WIDE bandwidth: sidebands at 1500 ± n×8 Hz for many
 *     integer values of n (governed by Bessel functions J_n(β)).
 *   → Visible as a dense cluster of lines around 1500 Hz; a striking
 *     contrast to the clean 3-line pattern of AM.
 */
#define SINE_FREQ    800.0f
#define CHIRP_F0     100.0f
#define CHIRP_F1    3800.0f
#define CHIRP_DUR     3.0f    /* sweep duration in simulation seconds */
#define MIX_TONE    1200.0f
#define MULTI_F1     440.0f
#define MULTI_F2    1000.0f
#define MULTI_F3    2800.0f
#define AM_CARRIER  2000.0f
#define AM_MOD_FREQ  200.0f
#define FM_CARRIER  1500.0f
#define FM_DEV       600.0f
#define FM_RATE        8.0f   /* modulation rate (Hz) */

/* Color pair IDs */
#define CP_S0    1   /* level 0: silence  (black)       */
#define CP_S1    2   /* level 1: very dim (dark blue)   */
#define CP_S2    3   /* level 2: dim      (blue)        */
#define CP_S3    4   /* level 3: medium   (cyan)        */
#define CP_S4    5   /* level 4: moderate (green)       */
#define CP_S5    6   /* level 5: bright   (yellow)      */
#define CP_S6    7   /* level 6: loud     (orange-red)  */
#define CP_S7    8   /* level 7: peak     (white)       */
#define CP_HUD   9   /* HUD / overlay (cyan)            */
#define CP_DOM  10   /* dominant-frequency marker       */
#define CP_GRID 11   /* frequency grid lines            */

/* ASCII density ramp for the 8 magnitude levels */
static const char SPEC_CH[8] = { ' ', '.', ':', '+', 'x', 'X', '#', '@' };

/* Human-readable names */
static const char *SIG_NAMES[SIG_COUNT] = {
    "SINE", "CHIRP", "MIXTURE", "MULTI-TONE", "AM", "FM" };
static const char *WIN_NAMES[WIN_COUNT] = {
    "RECT", "HANN", "HAMMING", "BLACKMAN" };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        /* Spectral ramp: black → dark-blue → blue → cyan → green → yellow → red → white */
        init_pair(CP_S0,  16,  COLOR_BLACK);   /* black (silence)    */
        init_pair(CP_S1,  17,  COLOR_BLACK);   /* dark navy          */
        init_pair(CP_S2,  27,  COLOR_BLACK);   /* blue               */
        init_pair(CP_S3,  51,  COLOR_BLACK);   /* cyan               */
        init_pair(CP_S4,  46,  COLOR_BLACK);   /* green              */
        init_pair(CP_S5, 226,  COLOR_BLACK);   /* yellow             */
        init_pair(CP_S6, 208,  COLOR_BLACK);   /* orange             */
        init_pair(CP_S7, 231,  COLOR_BLACK);   /* white (peak)       */
        /* Misc */
        init_pair(CP_HUD,  51, COLOR_BLACK);   /* cyan HUD           */
        init_pair(CP_DOM, 226, COLOR_BLACK);   /* yellow dom-freq    */
        init_pair(CP_GRID, 240, COLOR_BLACK);  /* dim grey grid      */
    } else {
        init_pair(CP_S0, COLOR_BLACK,   COLOR_BLACK);
        init_pair(CP_S1, COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_S2, COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_S3, COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_S4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_S5, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_S6, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_S7, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_DOM,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_GRID, COLOR_WHITE,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  fft — iterative Cooley-Tukey DIT radix-2                          */
/* ===================================================================== */

/*
 * fft_inplace — compute the DFT of n complex samples in-place.
 *
 * Input:  re[] contains the n real signal samples; im[] must be all zeros.
 * Output: re[] and im[] together hold the complex frequency spectrum.
 *         Only bins 0 .. n/2−1 are useful (the upper half is a mirror
 *         because the input is real-valued).
 *
 * Algorithm: iterative Decimation-In-Time (DIT) Cooley-Tukey radix-2 FFT.
 *
 * ── Step 1: Bit-reversal permutation ────────────────────────────────────
 *   The DIT butterfly network reads its inputs in bit-reversed index order.
 *   For example with N=8: index 3 (binary 011) maps to position 6 (binary 110).
 *   We swap pairs re[i] ↔ re[j] and im[i] ↔ im[j] where j = bit_reverse(i).
 *   The standard iterative technique (j-variable with XOR) does this in O(N)
 *   without a separate bit-count function.
 *   After this step the data is in the correct order for the butterfly passes.
 *
 * ── Step 2: log₂(N) butterfly stages ────────────────────────────────────
 *   We process stages len = 2, 4, 8, ..., N.
 *   Each stage takes pairs of DFTs of size len/2 and combines them into
 *   one DFT of size len using the butterfly operation:
 *
 *       u = upper element (index i+j)
 *       v = lower element (index i+j+len/2) multiplied by twiddle factor
 *       u_new = u + v       (constructive combination)
 *       v_new = u − v       (destructive combination)
 *
 *   Twiddle factor W_len = e^(−2πi/len).  Rather than calling cosf/sinf
 *   inside the inner loop, we precompute W_len = (wr0, wi0) once per stage
 *   and accumulate:  (wr, wi) ← (wr, wi) × (wr0, wi0) each j step.
 *   This is equivalent to e^(−2πij/len) but uses only multiply-add operations,
 *   making the inner loop much faster.
 *
 * ── Complexity ───────────────────────────────────────────────────────────
 *   O(N log₂ N).  For N = 256: log₂(256) = 8 stages, each with 128
 *   butterflies → 1024 complex multiply-adds.
 *   Compare to the naive DFT: N² = 65536 operations — 64× slower.
 *   For N = 1024: FFT needs ~5120 operations vs DFT's 1,048,576.
 */
static void fft_inplace(float *re, float *im, int n)
{
    /* ── Step 1: Bit-reversal permutation ──────────────────────────────
     * Reorder input samples so each sample[i] moves to position bit_reverse(i).
     * The j-variable technique: for each i, compute j = bit_reverse(i)
     * incrementally using XOR bit manipulation, then swap re[i]↔re[j].
     * Only swap when i < j to avoid swapping pairs twice (which undoes the work). */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    /* ── Step 2: Butterfly stages — len = 2, 4, 8, ..., n ──────────────
     * Each iteration of the outer loop doubles the DFT block size.
     * ang = −2π/len is the angle of the base twiddle factor W_len.
     * (wr0, wi0) = (cos(ang), sin(ang)) = W_len^1; computed once per stage. */
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wr0 = cosf(ang), wi0 = sinf(ang);
        /* Process each block of size len within the array */
        for (int i = 0; i < n; i += len) {
            /* (wr, wi) is the running twiddle factor W_len^j.
             * Starts at (1, 0) = W_len^0 and multiplies by (wr0, wi0) each step.
             * This avoids sin/cos in the inner loop — only multiplications needed. */
            float wr = 1.0f, wi = 0.0f;
            for (int j = 0; j < len/2; j++) {
                /* u = upper butterfly input (no twiddle) */
                float ur = re[i+j],         ui = im[i+j];
                /* v = lower butterfly input scaled by twiddle factor W_len^j */
                float vr = re[i+j+len/2]*wr - im[i+j+len/2]*wi;
                float vi = re[i+j+len/2]*wi + im[i+j+len/2]*wr;
                /* Butterfly output: u_new = u + v,  v_new = u − v */
                re[i+j]         = ur + vr;
                im[i+j]         = ui + vi;
                re[i+j+len/2]   = ur - vr;
                im[i+j+len/2]   = ui - vi;
                /* Advance twiddle factor: (wr,wi) ← (wr,wi) × (wr0,wi0)
                 * This is complex multiplication: e^(−2πi(j+1)/len) = e^(−2πij/len) × e^(−2πi/len) */
                float nwr = wr*wr0 - wi*wi0;
                wi = wr*wi0 + wi*wr0;
                wr = nwr;
            }
        }
    }
}

/* ===================================================================== */
/* §5  window — analysis window functions                                 */
/* ===================================================================== */

/*
 * build_window — fill win[0..n-1] with the chosen window function weights.
 *
 * A window function w[n] tapers the analysis block to zero (or near zero)
 * at both its edges before the FFT is applied.  This reduces spectral
 * leakage (see the theory section at the top).
 *
 * Normalized argument: x = n / (N−1)  ∈ [0, 1].
 * Using (N−1) in the denominator ensures:
 *   x[0]   = 0 / (N−1) = 0.0   → window starts at or near zero
 *   x[N−1] = (N−1) / (N−1) = 1.0 → window ends at or near zero
 * This is correct regardless of what N is — the window shape scales
 * to fit any FFT size without any special-casing.
 *
 * Window type summary:
 *
 * WIN_RECT (rectangular): w[n] = 1 for all n.
 *   Equivalent to no window at all — just multiplying by 1.
 *   Sharpest possible frequency resolution: one bin wide main lobe.
 *   Worst leakage: sinc-shaped sidelobes reach only −13 dB below the peak.
 *   Use when: you know the tone frequency falls exactly on a bin (rare),
 *   or you need maximum frequency resolution and leakage doesn't matter.
 *
 * WIN_HANN (Hann / "von Hann"): w[n] = 0.5 × (1 − cos(2πx)).
 *   Smooth raised-cosine taper to zero at both ends.
 *   Sidelobes at −31 dB; main lobe 2× wider than RECT.
 *   Best general-purpose window: a good balance between leakage rejection
 *   and frequency resolution.  The default choice for most audio work.
 *
 * WIN_HAMMING: w[n] = 0.54 − 0.46 × cos(2πx).
 *   Like Hann but the endpoints are 0.08 instead of 0.0 (doesn't taper
 *   completely to zero).  The non-zero endpoints give a slightly narrower
 *   main lobe and a lower first sidelobe (−41 dB vs Hann's −31 dB), making
 *   it better for detecting one strong tone near a weaker one.
 *   However, the outer sidelobes don't decay as fast as Hann's.
 *
 * WIN_BLACKMAN: w[n] = 0.42 − 0.5×cos(2πx) + 0.08×cos(4πx).
 *   Three-term cosine window.  Sidelobes at −57 dB — excellent suppression.
 *   Main lobe is 3× wider than RECT.  Best for detecting a weak tone that
 *   sits next to a strong one (minimal leakage from the strong tone).
 *   Worst frequency resolution: two nearby tones need 3×Δf separation
 *   to appear as distinct peaks rather than one merged blob.
 */
static void build_window(float *win, int n, int type)
{
    float pi2 = 2.0f * (float)M_PI;
    for (int i = 0; i < n; i++) {
        /* x ∈ [0,1]: normalized position within the window.
         * Dividing by (n−1) makes x=0 at the first sample and x=1 at the last,
         * so the window shape is independent of n. */
        float x = (float)i / (float)(n - 1);
        switch (type) {
        case WIN_RECT:
            /* w[n] = 1: no tapering, passes all samples unchanged.
             * Sharpest frequency resolution (1 bin) but worst leakage (sinc sidelobes −13 dB). */
            win[i] = 1.0f;
            break;
        case WIN_HANN:
            /* w[n] = 0.5×(1 − cos(2πx)): smooth raised-cosine taper.
             * Sidelobes −31 dB; main lobe 2× wider than RECT; best general-purpose choice. */
            win[i] = 0.5f * (1.0f - cosf(pi2 * x));
            break;
        case WIN_HAMMING:
            /* w[n] = 0.54 − 0.46×cos(2πx): first sidelobe −41 dB; doesn't taper fully to zero.
             * Better near-peak sidelobe suppression than Hann; outer sidelobes decay slightly slower. */
            win[i] = 0.54f - 0.46f * cosf(pi2 * x);
            break;
        case WIN_BLACKMAN:
            /* w[n] = 0.42 − 0.5×cos(2πx) + 0.08×cos(4πx): three-term cosine window.
             * Sidelobes −57 dB; main lobe 3× wider; best for detecting weak tones near strong ones. */
            win[i] = 0.42f - 0.5f*cosf(pi2*x) + 0.08f*cosf(2.0f*pi2*x);
            break;
        default:
            win[i] = 1.0f;
        }
    }
}

/* ===================================================================== */
/* §6  signal — test signal generators                                    */
/* ===================================================================== */

/*
 * SigGen — state for the test signal oscillators.
 *
 * sig_phase / sig_phase2 / sig_phase3 — phase accumulators for up to three
 * simultaneous oscillators.  Each sample, the corresponding phase advances by:
 *   Δφ = 2π × frequency / SAMPLE_RATE   (radians per sample)
 * The oscillator output is then sin(phase).  Because the phase is carried
 * across calls, the waveform is phase-continuous: there are no discontinuities
 * between successive generate_sample() calls.
 *
 * Why wrap phase mod 2π each sample?
 *   After n samples, the raw accumulated phase would be:
 *     φ = 2π × freq × n / SR
 *   At n = 10^6 samples (125 seconds at 8 kHz) with freq = 800 Hz:
 *     φ ≈ 2π × 800 × 10^6 / 8000 ≈ 6.28 × 10^5  (large number!)
 *   A 32-bit float has only ~7 significant decimal digits.  At φ ≈ 6×10^5,
 *   the fractional bits (which determine the waveform's fine detail) are lost.
 *   Wrapping keeps φ in [0, 2π] at all times so the fractional precision is
 *   always preserved, regardless of how long the simulation runs.
 *
 * inst_freq — instantaneous frequency accumulator used only by SIG_CHIRP.
 *   It advances linearly from CHIRP_F0 to CHIRP_F1 sample-by-sample.
 *   The phase is the running integral of inst_freq, giving a quadratic-phase
 *   chirp: a waveform whose frequency increases linearly with time.
 *   Because phase = ∫f dt is continuous even when f changes, there are no
 *   discontinuities in the chirp waveform — you get a smooth sweep.
 *
 * AM waveform in the frequency domain:
 *   s(t) = (1 + m·sin(2π·f_mod·t)) × cos(2π·f_c·t)
 *   Expanding: carrier at f_c, plus sidebands at f_c ± f_mod.
 *   With m = 0.8: sideband amplitude = m/2 = 0.4 relative to carrier.
 *   Visible in the spectrogram as exactly 3 bright horizontal lines.
 *
 * FM waveform in the frequency domain:
 *   Instantaneous frequency = f_c + FM_DEV × sin(2π·FM_RATE·t)
 *   Phase = integral of inst freq = 2π·f_c·t − (FM_DEV/FM_RATE)·cos(2π·FM_RATE·t)
 *   Spectrum: sidebands at f_c + n×FM_RATE Hz for integer n, with amplitudes
 *   given by Bessel functions J_n(β) where β = FM_DEV / FM_RATE = 75.
 *   β >> 1 → many sidebands with significant amplitude → wide bandwidth.
 *
 * White noise in generate_sample():
 *   rand() & 0x7FFF  produces a uniform integer in [0, 32767].
 *   Dividing by 16383.5 maps it to approximately [0, 2].
 *   Subtracting 1 centers it to approximately [−1, +1].
 *   This is uniform white noise: it contributes equal power to all frequency bins.
 */
typedef struct {
    float  sig_phase;    /* primary phase accumulator   */
    float  sig_phase2;   /* secondary phase             */
    float  sig_phase3;   /* tertiary phase              */
    float  inst_freq;    /* chirp instantaneous freq    */
    int    signal_type;
} SigGen;

static void siggen_reset(SigGen *g, int signal_type)
{
    g->sig_phase  = 0.0f;
    g->sig_phase2 = 0.0f;
    g->sig_phase3 = 0.0f;
    g->inst_freq  = CHIRP_F0;
    g->signal_type = signal_type;
}

static float generate_sample(SigGen *g)
{
    float out  = 0.0f;
    float pi2  = 2.0f * (float)M_PI;
    float isr  = 1.0f / SAMPLE_RATE;

    switch (g->signal_type) {

    case SIG_SINE:
        /* Phase accumulator: advances 2π×SINE_FREQ/SR radians per sample.
         * Output: sin(phase) — a pure tone at SINE_FREQ Hz.
         * SINE_FREQ=800 → bin k = 800/(8000/256) = 25.6 (off-bin: shows leakage). */
        g->sig_phase += pi2 * SINE_FREQ * isr;
        out = sinf(g->sig_phase);
        break;

    case SIG_CHIRP:
        /* Phase-coherent linear chirp via instantaneous frequency.
         * inst_freq starts at CHIRP_F0 and increases by (F1−F0)/(dur×SR) each sample.
         * Phase = integral of frequency → continuous, no jumps when freq changes.
         * When inst_freq reaches CHIRP_F1, it resets to CHIRP_F0 (sawtooth in freq).
         * Result: a diagonal streak across the full height of the spectrogram. */
        g->sig_phase += pi2 * g->inst_freq * isr;
        g->inst_freq += (CHIRP_F1 - CHIRP_F0) / (CHIRP_DUR * SAMPLE_RATE);
        if (g->inst_freq >= CHIRP_F1) g->inst_freq = CHIRP_F0;
        out = sinf(g->sig_phase);
        break;

    case SIG_MIXTURE:
        /* sig_phase accumulates at MIX_TONE = 1200 Hz. */
        g->sig_phase += pi2 * MIX_TONE * isr;
        /* Tone (0.6) + white noise (0.4):
         * rand()&0x7FFF → uniform integer [0, 32767] → /16383.5 → [0,2] → −1 → [−1,1].
         * The tone shows as one bright line; noise fills all bins at a low level. */
        out = 0.6f * sinf(g->sig_phase)
            + 0.4f * ((float)(rand() & 0x7FFF) / 16383.5f - 1.0f);
        break;

    case SIG_MULTI:
        /* Three independent phase accumulators for three simultaneous tones.
         * F1=440, F2=1000, F3=2800 Hz — not harmonically related, well spaced.
         * Amplitudes 0.50 + 0.33 + 0.17 = 1.0 (sum to 1 to prevent clipping).
         * Shows as 3 distinct horizontal lines; demonstrates frequency resolution. */
        g->sig_phase  += pi2 * MULTI_F1 * isr;
        g->sig_phase2 += pi2 * MULTI_F2 * isr;
        g->sig_phase3 += pi2 * MULTI_F3 * isr;
        out = 0.50f*sinf(g->sig_phase)
            + 0.33f*sinf(g->sig_phase2)
            + 0.17f*sinf(g->sig_phase3);
        break;

    case SIG_AM: {
        /* sig_phase  = carrier phase at AM_CARRIER = 2000 Hz.
         * sig_phase2 = modulator phase at AM_MOD_FREQ = 200 Hz.
         * Envelope = (1 + m×sin(f_mod)) where m = 0.8.
         * AM output = 0.5 × envelope × carrier.
         * In frequency domain: carrier at 2000 Hz + sidebands at 1800 and 2200 Hz.
         * Sideband amplitude = m/2 = 0.4 relative to the carrier. */
        g->sig_phase  += pi2 * AM_CARRIER  * isr;
        g->sig_phase2 += pi2 * AM_MOD_FREQ * isr;
        /* Envelope = 1 + m·sin(f_mod) where m=0.8 */
        float env = 1.0f + 0.8f * sinf(g->sig_phase2);
        out = 0.5f * env * sinf(g->sig_phase);
        break;
    }
    case SIG_FM: {
        /* sig_phase2 = modulator phase at FM_RATE = 8 Hz.
         * Carrier instantaneous frequency = FM_CARRIER + FM_DEV×sin(modulator).
         * Each sample, the carrier phase advances by 2π×inst_freq/SR.
         * Modulation index β = FM_DEV/FM_RATE = 600/8 = 75 → many Bessel sidebands.
         * Spectrum: lines at 1500 ± n×8 Hz for n = 1,2,3,...; β>>1 means wide bandwidth. */
        /* Modulator phase */
        g->sig_phase2 += pi2 * FM_RATE * isr;
        /* Carrier with instantaneous freq deviation */
        float dp = pi2 * (FM_CARRIER + FM_DEV * sinf(g->sig_phase2)) * isr;
        g->sig_phase += dp;
        out = sinf(g->sig_phase);
        break;
    }
    }

    /* Wrap phases mod 2π to maintain float precision.
     * Without wrapping, phase grows without bound and loses fractional precision:
     * at n=10^6 samples, phase ≈ 6×10^5 radians; a float has ~7 digits so the
     * sine argument loses accuracy. Subtracting 2π when phase ≥ 2π keeps it
     * in [0, 2π] at all times — sin() result is identical, precision is preserved. */
    float p2 = 2.0f * (float)M_PI;
    if (g->sig_phase  >= p2) g->sig_phase  -= p2;
    if (g->sig_phase2 >= p2) g->sig_phase2 -= p2;
    if (g->sig_phase3 >= p2) g->sig_phase3 -= p2;

    return out;
}

/* ===================================================================== */
/* §7  spectrogram — state, allocation, FFT → display pipeline            */
/* ===================================================================== */

/*
 * Spec — all state for the live spectrogram.
 *
 * FFT buffers:
 *   fft_re / fft_im   — real and imaginary parts of the FFT in/out buffer.
 *   win_func          — precomputed window weights; multiplied onto samples
 *                       before the FFT to reduce spectral leakage.
 *   magnitude         — |FFT output| for bins 0..fft_size/2−1 (positive freqs).
 *
 * Signal ring buffer:
 *   sig_buf[fft_size] — circular buffer holding the most recent fft_size samples.
 *   sig_pos           — index of the NEXT write position (wraps mod fft_size).
 *   Each hop_size new samples are appended; compute_fft() reads the last
 *   fft_size samples (with 75% overlap) for the windowed FFT.
 *
 * Spectrogram display grid:
 *   spec[spec_cols × spec_rows] — the 2D ring buffer of normalized dB values [0,1].
 *   spec_cols — width in characters = number of time steps visible.
 *   spec_rows — height in rows = number of frequency bins displayed.
 *   spec_head — ring buffer write pointer: points to the NEXT column to write,
 *               which is also the OLDEST currently visible column.
 *               Reading from spec_head leftward (wrapping) gives time-ordered data.
 *
 * Diagnostics:
 *   mag_peak — smoothed global magnitude peak, updated every FFT column.
 *     Update rule: mag_peak = 0.995 × old + 0.005 × new_col_max.
 *     The 0.5% blend rate means the peak adapts over τ = 1/0.005 = 200 columns,
 *     preventing the color scale from jumping on every transient but still
 *     tracking long-term level changes.  The display stays stable.
 *   dominant_freq — Hz of the bin with the highest magnitude in the latest column.
 *     Computed as: dom_bin × (SAMPLE_RATE / fft_size).
 *     Displayed in the HUD and marked with '>' on the right edge of the panel.
 *
 * dB normalization formula (in compute_fft):
 *   db  = 20 × log10(mag / mag_peak)     [range: DB_FLOOR..0 dB]
 *   val = (db − DB_FLOOR) / (−DB_FLOOR)  [mapped to 0..1]
 *   At mag = mag_peak (loudest bin): db = 0, val = (0 − (−80)) / 80 = 1.0.
 *   At mag = mag_peak × 10^(−4) (−80 dB): db = −80, val = (−80 − (−80)) / 80 = 0.0.
 *   Any bin quieter than DB_FLOOR dB below the peak is clamped to val=0.0 (black).
 */
typedef struct {
    /* FFT parameters */
    int    fft_idx;       /* index into FFT_SIZES[]                     */
    int    fft_size;      /* N (power of 2)                             */
    int    hop_size;      /* new samples per spectrogram column         */
    float *fft_re;        /* FFT input/output real  [fft_size]          */
    float *fft_im;        /* FFT input/output imag  [fft_size]          */
    float *win_func;      /* analysis window weights [fft_size]         */
    float *magnitude;     /* magnitude spectrum      [fft_size/2]       */

    /* Signal ring buffer */
    float *sig_buf;       /* circular sample buffer  [fft_size]         */
    int    sig_pos;       /* next write position (mod fft_size)         */

    /* Generator */
    SigGen siggen;

    /* Spectrogram display grid */
    float *spec;          /* [spec_cols * spec_rows] normalized dB [0,1]*/
    int    spec_cols;     /* columns = time axis width                  */
    int    spec_rows;     /* rows    = frequency axis height            */
    int    spec_head;     /* ring-buffer write head (oldest data pos)   */

    /* Diagnostics */
    float  mag_peak;      /* smoothed global magnitude peak             */
    float  dominant_freq; /* Hz of highest-magnitude bin                */

    /* Config */
    int    signal_type;
    int    window_type;
    bool   paused;
    bool   show_grid;     /* overlay freq grid lines                    */
} Spec;

#define SPEC_IDX(s, col, row) ((col)*(s)->spec_rows + (row))

static int spec_alloc(Spec *s, int spec_cols, int spec_rows, int fft_idx)
{
    s->fft_idx  = fft_idx;
    s->fft_size = FFT_SIZES[fft_idx];
    s->hop_size = s->fft_size / HOP_DIV;
    s->spec_cols = spec_cols;
    s->spec_rows = spec_rows;

    int n = s->fft_size;
    s->fft_re   = calloc(n,               sizeof(float));
    s->fft_im   = calloc(n,               sizeof(float));
    s->win_func = malloc(n *              sizeof(float));
    s->magnitude = calloc(n/2,            sizeof(float));
    s->sig_buf  = calloc(n,               sizeof(float));
    s->spec     = calloc(spec_cols * spec_rows, sizeof(float));

    return (s->fft_re && s->fft_im && s->win_func && s->magnitude
            && s->sig_buf && s->spec) ? 0 : -1;
}

static void spec_free(Spec *s)
{
    free(s->fft_re);  free(s->fft_im);  free(s->win_func);
    free(s->magnitude); free(s->sig_buf); free(s->spec);
}

static void spec_init(Spec *s)
{
    memset(s->sig_buf, 0, s->fft_size * sizeof(float));
    memset(s->spec,    0, s->spec_cols * s->spec_rows * sizeof(float));
    s->sig_pos       = 0;
    s->spec_head     = 0;
    s->mag_peak      = 1e-6f;
    s->dominant_freq = 0.0f;
    siggen_reset(&s->siggen, s->signal_type);
    build_window(s->win_func, s->fft_size, s->window_type);
}

/*
 * compute_fft — window the latest N samples, run the FFT, store one
 * column of normalized dB values into the spectrogram ring buffer.
 *
 * Pipeline:
 *   1. Copy the last fft_size samples from sig_buf (ring buffer) into
 *      fft_re[], multiplied by win_func[].  fft_im[] is zeroed.
 *   2. Run fft_inplace() — O(N log₂ N) in-place complex DFT.
 *   3. Compute magnitude[k] = sqrt(re[k]² + im[k]²) for k = 1..N/2−1.
 *      (DC bin 0 is zeroed to prevent it dominating the display.)
 *   4. Track the column maximum (col_max) and dominant bin (dom_bin).
 *   5. Update smoothed global peak:
 *        mag_peak = 0.995 × mag_peak + 0.005 × col_max
 *      τ = 1/0.005 = 200 column updates → slow, stable adaptation.
 *      Prevents the color scale from jumping on every transient.
 *   6. Compute dominant_freq = dom_bin × (SR / N) Hz.
 *   7. For each display row, map it to a frequency bin:
 *        fbin = (spec_rows − 1 − row) × (N/2 − 1) / (spec_rows − 1)
 *        row=0 (top of screen) → fbin = N/2−1 = Nyquist
 *        row=spec_rows−1 (bottom) → fbin = 0 = DC
 *   8. Convert magnitude to normalized dB:
 *        db  = 20 × log10(magnitude[bin] / mag_peak)   (0 dB = peak)
 *        val = (db − DB_FLOOR) / (−DB_FLOOR)           (0.0 = floor, 1.0 = peak)
 *      val is clamped to [0, 1] and stored in spec[col][row].
 *   9. Advance spec_head (ring buffer write pointer) by 1 mod spec_cols.
 */
static void compute_fft(Spec *s)
{
    int n = s->fft_size;

    /* Copy windowed samples from ring buffer into FFT input.
     * sig_pos points one past the most recent sample, so sample (n−1) steps
     * back in the ring is at index (sig_pos − n + i + n) & (n−1). */
    for (int i = 0; i < n; i++) {
        int idx = (s->sig_pos - n + i + n) & (n - 1); /* ring read, n power of 2 */
        s->fft_re[i] = s->sig_buf[idx] * s->win_func[i];
        s->fft_im[i] = 0.0f;
    }

    fft_inplace(s->fft_re, s->fft_im, n);

    /* Compute magnitude spectrum, find dominant bin.
     * DC bin (k=0) is set to zero — DC offset would dominate the display
     * and is not musically meaningful.  Loop starts at k=1. */
    float col_max = 1e-10f;
    int   dom_bin = 1;
    s->magnitude[0] = 0.0f;  /* zero DC */
    for (int k = 1; k < n/2; k++) {
        float mag = sqrtf(s->fft_re[k]*s->fft_re[k] + s->fft_im[k]*s->fft_im[k]);
        s->magnitude[k] = mag;
        if (mag > col_max) { col_max = mag; dom_bin = k; }
    }

    /* Smooth global peak for stable display normalization.
     * mag_peak decays slowly (τ ≈ 200 columns) so sudden loud transients
     * don't cause a flash by collapsing the entire color scale to near-zero. */
    s->mag_peak = s->mag_peak * 0.995f + col_max * 0.005f;
    if (s->mag_peak < 1e-9f) s->mag_peak = 1e-9f;
    /* dominant_freq: convert bin index to Hz using the bin-to-frequency formula */
    s->dominant_freq = (float)dom_bin * SAMPLE_RATE / (float)n;

    /* Map magnitude to normalized dB and store in spectrogram column.
     * spec_head is the current write column; it will be advanced at the end. */
    int   col     = s->spec_head;
    float inv_pk  = 1.0f / s->mag_peak;
    int   half    = n / 2;

    for (int row = 0; row < s->spec_rows; row++) {
        /* row 0 = top = high freq (Nyquist), row spec_rows-1 = bottom = DC.
         * fbin is a float bin index; we round to the nearest integer bin. */
        float fbin = (float)(s->spec_rows - 1 - row) * (float)(half - 1)
                     / (float)(s->spec_rows - 1);
        int bin = (int)(fbin + 0.5f);
        if (bin < 0)    bin = 0;
        if (bin >= half) bin = half - 1;

        /* dB relative to smoothed peak: 0 dB = peak, DB_FLOOR = silence.
         * val maps [DB_FLOOR, 0] dB linearly to [0, 1]:
         *   val = (db − DB_FLOOR) / (0 − DB_FLOOR) = (db − DB_FLOOR) / (−DB_FLOOR) */
        float db  = 20.0f * log10f(s->magnitude[bin] * inv_pk + 1e-9f);
        float val = (db - DB_FLOOR) / (-DB_FLOOR);  /* [0,1]: 0=floor, 1=peak */
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;

        s->spec[SPEC_IDX(s, col, row)] = val;
    }

    /* Advance ring-buffer write head: next call writes to the next column,
     * overwriting the oldest visible column (which will scroll off-screen). */
    s->spec_head = (s->spec_head + 1) % s->spec_cols;
}

/*
 * update_spectrogram — generate hop_size new samples then compute one FFT.
 *
 * Generates samples into the circular sig_buf, then calls compute_fft
 * which reads the last fft_size samples (with 75% overlap) and pushes
 * one new column into the spec ring buffer.
 */
static void update_spectrogram(Spec *s)
{
    if (s->paused) return;

    int mask = s->fft_size - 1;  /* works because fft_size is power of 2 */
    for (int i = 0; i < s->hop_size; i++) {
        s->sig_buf[s->sig_pos & mask] = generate_sample(&s->siggen);
        s->sig_pos++;
    }

    compute_fft(s);
}

/* ===================================================================== */
/* §8  render_panel + render_overlay                                      */
/* ===================================================================== */

/*
 * render_panel — draws the scrolling time-frequency heatmap.
 *
 * Ring-buffer read order:
 *   spec_head = NEXT write position = oldest currently visible data.
 *   To display data in time order (oldest left, newest right):
 *     screen column sx=0       (left,  oldest) → ring col = spec_head
 *     screen column sx=sc-1    (right, newest) → ring col = (spec_head + sc−1) % sc
 *     General: ring_col = (spec_head + sx) % spec_cols
 *   This way the rightmost column is always the freshest FFT result,
 *   and the display scrolls left as new columns overwrite spec_head.
 *
 * Intensity → character mapping:
 *   val ∈ [0,1] is multiplied by 8 and truncated to level lv ∈ [0,7].
 *   SPEC_CH[lv] picks the density character: ' ' for silence, '@' for peak.
 *   COLOR_PAIR(CP_S0 + lv) maps lv to the spectral color ramp (black → white).
 *   Levels 5–7 get A_BOLD to boost brightness on terminals that support it.
 *
 * attrset change-detection:
 *   cur_attr tracks the currently-set ncurses attribute.  attrset() is
 *   only called when the new attribute differs from the previous one.
 *   In smooth gradient regions many adjacent cells share the same level,
 *   so this reduces ncurses I/O calls from ~3×(rows×cols) to ~1×(rows×cols).
 *
 * Frequency grid lines (show_grid):
 *   Grid lines at 500, 1000, 2000, 4000 Hz.
 *   Row position from frequency:
 *     grow = (1 − f/nyquist) × (sr − 1)
 *     f=0 (DC)      → grow = (1−0)×(sr−1) = sr−1 = bottom row
 *     f=nyquist     → grow = (1−1)×(sr−1) = 0    = top row
 *   The label (e.g. "1000") is printed at column 0; dashes fill the rest.
 *
 * Dominant frequency marker:
 *   The '>' character is drawn at the right edge of the display (sc−1)
 *   on the row corresponding to dominant_freq.  Same row formula as grid:
 *     dom_row = (1 − dominant_freq/nyquist) × (sr−1)
 *   Only drawn when dominant_freq > 10 Hz (ignores near-DC noise).
 */
static void render_panel(const Spec *s, int cols, int rows)
{
    int sc = s->spec_cols < cols ? s->spec_cols : cols;
    int sr = s->spec_rows < rows ? s->spec_rows : rows;

    chtype cur_attr = A_NORMAL;
    attrset(cur_attr);

    for (int sy = 0; sy < sr; sy++) {
        for (int sx = 0; sx < sc; sx++) {
            /* Map screen column sx to ring buffer column.
             * sx=0 → spec_head (oldest data), sx=sc-1 → spec_head−1 mod sc (newest). */
            int ring_col = (s->spec_head + sx) % s->spec_cols;
            float v = s->spec[SPEC_IDX(s, ring_col, sy)];

            /* Quantize normalized value [0,1] to 8 discrete display levels. */
            int    lv = (int)(v * 8.0f);
            if (lv < 0) lv = 0;
            if (lv > 7) lv = 7;

            attr_t at = (lv >= 5) ? A_BOLD : A_NORMAL;
            chtype a  = (chtype)COLOR_PAIR(CP_S0 + lv) | at;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvaddch(sy + 1, sx, (chtype)SPEC_CH[lv]);
        }
    }

    /* Frequency grid overlay.
     * grow maps frequency to display row: grow = (1 − f/nyquist) × (sr−1).
     * f=0 (DC) → bottom row; f=nyquist → top row. */
    if (s->show_grid) {
        static const float GRID_HZ[] = { 500.0f, 1000.0f, 2000.0f, 4000.0f };
        float nyquist = SAMPLE_RATE * 0.5f;
        for (int gi = 0; gi < 4; gi++) {
            float f = GRID_HZ[gi];
            if (f >= nyquist) continue;
            int grow = (int)((1.0f - f/nyquist) * (float)(sr - 1) + 0.5f);
            if (grow < 0 || grow >= sr) continue;
            attron(COLOR_PAIR(CP_GRID) | A_DIM);
            for (int sx = 4; sx < sc; sx++) mvaddch(grow+1, sx, (chtype)'-');
            mvprintw(grow+1, 0, "%4.0f", (double)f);
            attroff(COLOR_PAIR(CP_GRID) | A_DIM);
        }
    }

    /* Dominant-frequency marker: '>' drawn at right edge on the dominant freq row.
     * dom_row uses the same frequency-to-row formula as the grid lines:
     *   dom_row = (1 − dominant_freq/nyquist) × (sr−1)
     * Only shown when dominant_freq > 10 Hz to suppress noise near DC. */
    if (s->dominant_freq > 10.0f) {
        float nyquist = SAMPLE_RATE * 0.5f;
        int dom_row = (int)((1.0f - s->dominant_freq/nyquist) * (float)(sr-1) + 0.5f);
        if (dom_row >= 0 && dom_row < sr) {
            attron(COLOR_PAIR(CP_DOM) | A_BOLD);
            mvaddch(dom_row+1, sc-1, (chtype)'>');
            attroff(COLOR_PAIR(CP_DOM) | A_BOLD);
        }
    }

    attrset(A_NORMAL);
}

/*
 * render_overlay — HUD top and bottom bars.
 *
 * Top bar shows:
 *   signal name, window name, FFT size, frequency resolution (Δf),
 *   time resolution per column (Δt), dominant frequency, fps.
 *
 * Resolution estimate explanation:
 *   Δf = SR / FFT_SIZE  (Hz per bin — minimum separable frequency difference)
 *   Δt = hop_size / SR  (seconds per column — time between successive frames)
 *   Δt · Δf = hop / fft_size = 1 / HOP_DIV  (constant = 0.25 here, 75% overlap)
 */
static void render_overlay(const Spec *s, int cols, int rows, double fps)
{
    float delta_f  = SAMPLE_RATE / (float)s->fft_size;
    float delta_t  = (float)s->hop_size / SAMPLE_RATE * 1000.0f;  /* ms */
    float nyquist  = SAMPLE_RATE * 0.5f;

    char buf[350];
    snprintf(buf, sizeof buf,
        " SPECTROGRAM [%s | %s] | FFT=%d  Df=%.1fHz  Dt=%.1fms"
        " | fdom=%.0fHz | NY=%.0fHz | %.0ffps ",
        SIG_NAMES[s->signal_type], WIN_NAMES[s->window_type],
        s->fft_size, (double)delta_f, (double)delta_t,
        (double)s->dominant_freq, (double)nyquist, fps);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(rows-1, 0,
        " q:quit  spc:pause  1-6:signal  w:window  n/N:FFT-  g:grid  r:reset ");
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    if (s->paused) {
        attron(COLOR_PAIR(CP_S6) | A_BOLD);
        mvprintw(rows/2, cols/2-4, " PAUSED ");
        attroff(COLOR_PAIR(CP_S6) | A_BOLD);
    }
}

/* ===================================================================== */
/* §9  scene                                                              */
/* ===================================================================== */

typedef struct { Spec spec; } Scene;

static void scene_init(Scene *sc, int cols, int rows)
{
    Spec *s = &sc->spec;
    memset(s, 0, sizeof *s);
    s->signal_type = SIG_SINE;
    s->window_type = WIN_HANN;
    s->show_grid   = true;

    int sc_cols = cols   < GRID_MAX_COLS ? cols   : GRID_MAX_COLS;
    int sc_rows = rows-2 < GRID_MAX_ROWS ? rows-2 : GRID_MAX_ROWS;
    if (sc_rows < 4) sc_rows = 4;

    if (spec_alloc(s, sc_cols, sc_rows, FFT_IDX_DEF) != 0) return;
    spec_init(s);
}

static void scene_free(Scene *sc) { spec_free(&sc->spec); }

static void scene_rebuild(Scene *sc, int spec_cols, int spec_rows, int fft_idx)
{
    Spec *s = &sc->spec;
    int sig = s->signal_type, win = s->window_type;
    bool grid = s->show_grid;
    spec_free(s);
    memset(s, 0, sizeof *s);
    s->signal_type = sig;
    s->window_type = win;
    s->show_grid   = grid;
    if (spec_alloc(s, spec_cols, spec_rows, fft_idx) != 0) return;
    spec_init(s);
}

static void scene_resize(Scene *sc, int cols, int rows)
{
    Spec *s = &sc->spec;
    int sc_cols = cols   < GRID_MAX_COLS ? cols   : GRID_MAX_COLS;
    int sc_rows = rows-2 < GRID_MAX_ROWS ? rows-2 : GRID_MAX_ROWS;
    if (sc_rows < 4) sc_rows = 4;
    scene_rebuild(sc, sc_cols, sc_rows, s->fft_idx);
}

static void scene_tick(Scene *sc)
{
    update_spectrogram(&sc->spec);
}

static void scene_draw(const Scene *sc, int cols, int rows, double fps)
{
    erase();
    render_panel(&sc->spec, cols, rows - 2);
    render_overlay(&sc->spec, cols, rows, fps);
}

/* ===================================================================== */
/* §10  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak();
    curs_set(0); nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)  { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols);
}

/* ===================================================================== */
/* §11  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit(int s)   { (void)s; g_app.running    = 0; }
static void on_resize(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup(void)    { endwin(); }

static bool handle_key(App *app, int ch)
{
    Spec *s = &app->scene.spec;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':  s->paused = !s->paused; break;
    case 'g': case 'G': s->show_grid = !s->show_grid; break;
    case 'r': case 'R':
        scene_resize(&app->scene, app->screen.cols, app->screen.rows);
        break;
    case '1': case '2': case '3': case '4': case '5': case '6': {
        int t = ch - '1';
        if (t < SIG_COUNT) {
            s->signal_type = t;
            siggen_reset(&s->siggen, t);
        }
        break;
    }
    case 'w': case 'W':
        s->window_type = (s->window_type + 1) % WIN_COUNT;
        build_window(s->win_func, s->fft_size, s->window_type);
        break;
    case 'n': {
        /* Decrease FFT size: better time resolution, worse frequency */
        int new_idx = s->fft_idx - 1;
        if (new_idx < 0) new_idx = 0;
        if (new_idx != s->fft_idx)
            scene_rebuild(&app->scene, s->spec_cols, s->spec_rows, new_idx);
        break;
    }
    case 'N': {
        /* Increase FFT size: better frequency resolution, worse time */
        int new_idx = s->fft_idx + 1;
        if (new_idx >= N_FFT_SIZES) new_idx = N_FFT_SIZES - 1;
        if (new_idx != s->fft_idx)
            scene_rebuild(&app->scene, s->spec_cols, s->spec_rows, new_idx);
        break;
    }
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time = clock_ns();
    int64_t fps_accum  = 0;
    int     fps_count  = 0;
    double  fps_disp   = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time = clock_ns();
        }

        /* ── wall-clock dt ───────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;

        /* ── physics: advance spectrogram ────────────── */
        for (int step = 0; step < STEPS_PER_FRAME; step++)
            scene_tick(&app->scene);

        /* ── fps tracking ────────────────────────────── */
        fps_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_accum / (double)NS_PER_SEC);
            fps_count = 0;
            fps_accum = 0;
        }

        /* ── sleep to target 30 fps ──────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 30 - elapsed);

        /* ── render ──────────────────────────────────── */
        scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ───────────────────────────────────── */
        int key;
        while ((key = getch()) != ERR)
            if (!handle_key(app, key)) { app->running = 0; break; }
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
