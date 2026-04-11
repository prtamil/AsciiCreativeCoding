# Concept: Pendulum Wave

## Pass 1 — Understanding

### Core Idea
N pendulums with lengths chosen so pendulum n completes (N_BASE + n) full oscillations in T_SYNC seconds. At t=0 all are in phase. They drift apart into mesmerizing wave patterns, then re-synchronize exactly at t=T_SYNC.

### Mental Model
Imagine 15 metronomes, each ticking slightly faster than the last. They start together, slowly go out of step, form traveling waves and standing waves, then snap back into unison. The interval between sync events is exactly T_SYNC.

### Key Equations
```
ω_n = 2π · (N_BASE + n) / T_SYNC    ← angular frequency
θ_n(t) = A · sin(ω_n · t)           ← small-angle motion
```

The length is implicit: `L_n = g / ω_n²` (not needed if using ω_n directly).

### Data Structures
- `g_omega[N_PEND]`: precomputed angular frequencies
- No integration needed — analytic formula gives θ(t) directly
- `g_time`: continuously accumulated float

### Non-Obvious Decisions
- **Analytic, not numerical**: Because each pendulum is independent with a fixed frequency, we can use `sin(ω·t)` directly — no RK4 needed.
- **String as interpolated line**: The string from pivot to bob must be drawn as a sloped line (`/` `\` `|`), not a vertical column. Otherwise the string appears motionless while only the bob moves.
- **Bob displacement in columns**: `bob_col = center_col + θ · (cols_per_pendulum)`. Scale correctly — integer division kills amplitude.
- **Jump to sync**: SPACE key sets `t = ceil(t/T_SYNC)*T_SYNC` to instantly show the re-sync moment.

### Key Constants
| Name | Value | Role |
|------|-------|------|
| N_PEND | 15 | number of pendulums |
| N_BASE | 40 | lowest oscillation count in T_SYNC |
| T_SYNC | 60 s | period until full re-synchronization |
| AMP_INIT | 0.70 | initial amplitude (fraction of column width) |

### Open Questions
- What phase patterns appear at t = T_SYNC/2? T_SYNC/3? T_SYNC/4?
- What if N_BASE varied non-linearly? (frequency ratios change pattern)
- Can you show the pendulum lengths visually as different string lengths?

---

## Pass 2 — Implementation

### Pseudocode
```
physics_init():
    for n in 0..N_PEND:
        omega[n] = 2*PI*(N_BASE + n) / T_SYNC

draw():
    for n in 0..N_PEND:
        th = amp * sin(omega[n] * t)
        center_col = (n + 0.5) * cols / N_PEND
        bob_col = center_col + th * (cols / N_PEND)
        bob_row = pivot_row + string_len * length_fraction(n)

        # draw string as slope from (pivot_row, center_col) to (bob_row, bob_col)
        dr = bob_row - pivot_row
        dc = bob_col - center_col
        slope = dc / dr
        for s in 1..dr:
            r = pivot_row + s
            c = center_col + round(slope * s)
            mvaddch(r, c, slope_char(slope))

        mvaddch(bob_row, bob_col, 'O')   # bob
```

### Module Map
```
§1 config    — N_PEND, N_BASE, T_SYNC, AMP_INIT
§2 clock     — clock_ns(), clock_sleep_ns()
§3 color     — 15 rainbow colors + HUD pair
§4 physics   — omega[] precompute, theta(n,t) inline
§5 draw      — scene_draw(): strings + bobs + HUD
§6 app       — main loop, signal handlers, key input
```

### Data Flow
```
t accumulates each frame → theta(n,t) per pendulum
→ (center_col, bob_col, bob_row) → slope-interpolated string
→ colored bob at displaced position
```

### Core Loop
```c
for each frame:
    handle keys (q p r +/- space)
    dt_ns = now - last; last = now
    if !paused: t += dt_ns * 1e-9
    erase()
    scene_draw()   // computes theta analytically, draws strings
    doupdate()
    sleep to hit 60 fps
```
