# Pass 1 — led_number_morph.c: Particle LED Number Morphing

## Core Idea

A large 7-segment LED digit is rendered entirely from 168 particles. Each of the 7 segments owns exactly 24 particles permanently. When a segment is **active** for the current digit, its particles spring toward evenly spaced target positions along the segment. When inactive, they drift back toward the digit centre. The result looks like a large illuminated display constructed from glowing dots.

## The Mental Model

### 7-segment encoding

A classic 7-segment display uses segments A–G:

```
 _        A = top horizontal
|_|       B = top-right vertical
|_|       C = bot-right vertical
          D = bottom horizontal
          E = bot-left vertical
          F = top-left vertical
          G = middle horizontal
```

Each digit 0–9 is a bitmask:

```c
static const uint8_t k_seg_mask[10] = {
    0x3F,  /* 0: ABCDEF   */
    0x06,  /* 1: BC       */
    0x5B,  /* 2: ABDEG    */
    0x4F,  /* 3: ABCDG    */
    0x66,  /* 4: BCFG     */
    0x6D,  /* 5: ACDFG    */
    0x7D,  /* 6: ACDEFG   */
    0x07,  /* 7: ABC      */
    0x7F,  /* 8: ABCDEFG  */
    0x6F,  /* 9: ABCDFG   */
};
```

### Permanent segment ownership

168 particles are divided evenly: 24 per segment (N_SEGS = 7). Ownership is set at initialisation and never changes. This avoids reassignment when digits change — a particle always belongs to the same segment and knows its orientation.

### Spring physics

Each particle has position `(x, y)`, velocity `(vx, vy)`, and a target `(tx, ty)`:

```
F = SPRING_K * (tx - x) - DAMP * vx
vx += F * dt
x  += vx * dt
```

`SPRING_K = 9.0`, `DAMP = 5.5`. The critical damping ratio `ζ = DAMP / (2√SPRING_K) ≈ 0.92` — slightly underdamped, so particles arrive at the target with a tiny overshoot, then settle. This gives a lively, bouncy motion rather than a stiff snap.

When a segment is **inactive**, `(tx, ty)` is set to the digit centre `(cx, cy)`. The particle drifts inward slowly and eventually clusters at the centre, becoming invisible at the background character.

### Target generation

For each segment, `N_PER_SEG = 24` targets are spaced evenly along the segment line:

```c
for (int k = 0; k < N_PER_SEG; k++) {
    float t = (float)k / (float)(N_PER_SEG - 1);
    tx = seg_x0 + t * (seg_x1 - seg_x0);
    ty = seg_y0 + t * (seg_y1 - seg_y0);
}
```

Horizontal segments span the digit width at the top, middle, or bottom row. Vertical segments span the top or bottom half of the digit height on the left or right side.

### Scaling with terminal size

```c
g_dh = (int)(g_rows * 0.65f);   /* 65% of screen height */
g_dw = (int)(g_dh * 1.1f);      /* ≈7-segment aspect ratio, corrected for cells */
```

The factor 1.1 comes from the physical ratio: a 7-segment display is ≈0.55 times as wide as tall, but terminal cells are ≈2× taller than wide, so `effective_width = 0.55 × 2 = 1.1 × height`. This makes the digit look proportional on screen.

### Orientation-aware rendering

```c
/* Segment orientation: 0=horizontal, 1=vertical */
static const int k_seg_orient[N_SEGS] = { 0,1,1,0,1,1,0 };   /* A B C D E F G */
```

When a segment is formed (particle at target), the character depends on orientation:
- Horizontal segment → `'-'` (or `ACS_HLINE` if available)
- Vertical segment → `'|'` (or `ACS_VLINE`)

Particles in flight (more than 1.5 cells from target) show `'+'` or `'.'` depending on speed. This makes the formed digit look like actual LED segments rather than a blob of identical characters.

### Hold timer and digit advance

```c
if (all particles settled):
    hold_tick++;
    if hold_tick >= HOLD_FRAMES:
        advance digit 0→1→…→9→0
        assign new segment targets
```

"Settled" means `|pos - target| < SETTLE_THRESH` for all particles. The hold gives the viewer time to read the digit before it morphs.

## Data Structures

```c
typedef struct {
    float x, y;    /* current position */
    float vx, vy;  /* velocity */
    float tx, ty;  /* current target */
    int   seg;     /* permanent segment index 0..6 */
} Particle;

Particle g_parts[N_PARTS];   /* N_PARTS = 168 = 7 segs × 24 */
```

## The Main Loop

```
init_particles():
    for each segment, for k in 0..23:
        particle.seg = segment index
        particle.x/y = centre (starting position)
        particle.tx/ty = first target position

digit_set(d):
    for each segment s:
        active = (k_seg_mask[d] >> s) & 1
        if active: compute 24 evenly spaced targets along segment
        else:      tx=cx, ty=cy  (centre target)

per frame:
    for each particle:
        Fx = SPRING_K*(tx-x) - DAMP*vx
        Fy = SPRING_K*(ty-y) - DAMP*vy
        vx += Fx * DT;  x += vx * DT
        vy += Fy * DT;  y += vy * DT

draw:
    for each particle:
        dist = hypot(x-tx, y-ty)
        if dist < 0.8: ch = orientation_char   (formed)
        elif dist < 2:  ch = '+'
        else:           ch = '.'
        color = per-digit theme color
```

## Non-Obvious Decisions

### Why 24 particles per segment, not fewer or more

24 particles can cover the longest segment (height ≈ 0.5 × dh ≈ 20 rows) with one particle per row, leaving room for the shorter horizontal segments to use fewer with no gaps. With 12 particles some long segments would show visible gaps. With 36 particles the segment would appear over-dense and slow to settle.

### Why permanent ownership rather than reassignment

Reassigning particles on each digit change would require a matching algorithm (like greedy nearest-neighbor in particle_number_morph.c). With permanent ownership, each particle always knows its segment, its orientation, and its set of possible targets. No matching, no index tables, simpler code.

The cost: when a segment turns off, all 24 of its particles must migrate to the centre. With high damping (DAMP=5.5) this is fast, but it means 24 particles always exist at the centre for inactive segments rather than being hidden.

### Why the particle char changes by distance to target, not speed

Speed `sqrt(vx²+vy²)` is momentarily zero at the start of movement (before the spring pulls), which would incorrectly show the particle as "formed" during the first frame. Distance `|pos - target|` is always meaningful: large = in flight, small = formed.

## Open Questions for Pass 3

- What SPRING_K / DAMP ratio produces the most aesthetically pleasing morph? Plot settling time vs `ζ = DAMP / (2√SPRING_K)` for `ζ` in [0.5, 1.5].
- Could particles be assigned fractional targets at different densities — one particle per 2 cells on long segments, one per 1 cell on short segments — for uniform apparent density?
- What would the display look like with a glow falloff: nearby cells rendered with half-characters or dimmer colors as a simulated point-source glow radius?
- Can you add a **clock mode**: read system time and cycle the display showing HH:MM instead of 0–9?
