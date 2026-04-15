# Pass 1 — particle_number_morph.c: Solid Particle Number Morphing (LERP)

## Core Idea

Up to 500 particles densely fill the interior of a large bitmap font digit. On each digit change, a **greedy nearest-neighbour assignment** routes each particle to the closest target in the new digit. Positions are then **linearly interpolated with smoothstep easing** from their current location to the target — no spring forces, no velocity. The result is a clean, deterministic glide between digit shapes.

## The Mental Model

### Bitmap font expansion

The digit is defined in a 9-row × 7-column binary font where `'#'` means filled:

```c
/* digit 0 */
" ##### "
"##   ##"
"##   ##"
...
" ##### "
```

Each `'#'` cell is expanded to a sub-grid of size `(ppr × ppc)` particles, where `ppr` and `ppc` scale with terminal size (up to 3×4 per font pixel). This produces a target grid of `n_targets` positions per digit.

```
max pixels for digit 8: 39 font '#' × 12 sub-pixels = 468 ≤ N_PARTS=500
```

All 10 digits' target grids are precomputed once at startup into `g_targets[10][N_PARTS]`.

### Greedy nearest-neighbour assignment

On digit change, each active particle must be assigned a target. The assignment determines which particle travels where:

```c
for each target t in new digit:
    find closest unassigned particle p
    assign p → t
```

This is O(n_targets × N_PARTS) — for 468 targets × 500 particles ≈ 234,000 distance comparisons. At 500 particles this runs in a fraction of a millisecond.

**Why greedy nearest-neighbour?** It minimises total travel distance approximately (not optimally — that requires the Hungarian algorithm at O(n³) cost). Greedy NN produces visually pleasing, short trajectories: particles barely move when digit shapes are similar (e.g. 3→9), and only particles far from any new target travel long distances.

### Smoothstep LERP

```c
static float smoothstep(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);  /* 3t² − 2t³ */
}
```

The morph parameter `g_morph_t` advances by `1/g_morph_frames` per frame. `smoothstep` maps it to a curve that:
- Has zero derivative at t=0 and t=1 (ease-in and ease-out)
- Is symmetric around t=0.5
- Passes through (0,0) and (1,1)

Position update:
```c
float st = smoothstep(g_morph_t);
p.x = p.ox + st * (p.tx - p.ox);   /* ox = snapshot at morph start */
p.y = p.oy + st * (p.ty - p.oy);
```

`ox, oy` are snapshotted at the moment `digit_assign()` is called — wherever the particle currently is (which could be mid-morph from a previous digit). This means the ease-out of the previous morph flows directly into the ease-in of the next.

### Origin snapshot

```c
/* In digit_assign(): */
g_parts[i].ox = g_parts[i].x;
g_parts[i].oy = g_parts[i].y;
```

Without snapshotting, interpolation would be from the *original* target of the previous digit (which could be far from where the particle actually is if it was interrupted mid-morph). The snapshot makes the motion always start from the current position regardless of history.

### Idle particles

Particles with no target in the new digit (because the new digit has fewer pixels than the old) are marked `active = false`, assigned `tx = cx, ty = cy` (digit centre), and lerp to the centre. Once there they become invisible. When the next digit needs more particles, they are reassigned from the centre — they appear to "emerge" from the centre and glide to their new positions.

### Color during morph

All active particles always use the same bright color pair regardless of morph progress:

```c
attr_t a_bright = COLOR_PAIR(CP_D0 + digit) | A_BOLD;
```

The character changes to indicate morph stage:
```
st > 0.92  →  '@'   (nearly arrived, dense)
st > 0.55  →  '#'   (mid-morph, settled)
st > 0.20  →  '+'   (early, sparse)
else       →  '.'   (just started)
```

This makes the morph visible in character density without dimming the color. Earlier versions dimmed `a_mid` / `a_far` but the user corrected this — bright color throughout gives a more luminous effect.

### Hold timer

```c
if (g_morph_t >= 1.0f) {
    hold_tick++;
    if (hold_tick >= hold_max) {
        cur_digit = (cur_digit + 1) % 10;
        digit_assign(cur_digit, cx, cy);
        hold_tick = 0;
    }
}
```

The hold timer only advances when the morph is fully complete (`morph_t ≥ 1.0`). This guarantees the digit is fully formed before starting the hold, so the viewer always sees the complete digit, never an in-progress morph that prematurely disappears.

## Data Structures

```c
typedef struct {
    float x,  y;    /* current position */
    float ox, oy;   /* snapshot position at morph start */
    float tx, ty;   /* target position */
    bool  active;   /* true if this particle fills a digit pixel */
} Particle;

Particle g_parts[N_PARTS];   /* N_PARTS = 500 */

typedef struct { float x, y; } Target;
Target g_targets[10][N_PARTS];  /* precomputed target grids */
int    g_n_targets[10];          /* number of filled pixels per digit */
```

## The Main Loop

```
startup:
    precompute_targets()   ← expand font '#' pixels to sub-grid, scale to terminal
    init particles at centre, all active=false
    digit_assign(0, cx, cy)

digit_assign(d):
    greedy NN: assign each target[d][t] to closest unassigned particle
    for assigned particles: set tx/ty, active=true
    for unassigned particles: set tx=cx, ty=cy, active=false
    snapshot ox/oy = current x/y for all particles
    g_morph_t = 0.0

per frame:
    if !paused:
        g_morph_t = min(1.0, g_morph_t + 1/g_morph_frames)
        st = smoothstep(g_morph_t)
        for each particle:
            x = ox + st*(tx - ox)
            y = oy + st*(ty - oy)
    if g_morph_t >= 1.0:
        hold_tick++
        if hold_tick >= hold_max: advance digit

draw:
    for each particle:
        if active: render char based on st
        (inactive particles at centre → not rendered)
```

## Non-Obvious Decisions

### Why N_PARTS = 500 rather than exactly matching digit 8

Digit 8 needs up to 468 particles (39 pixels × 12 sub-grid). Using 500 gives a 7% buffer for rounding in sub-grid calculation (which uses integer division). If the sub-grid were slightly larger at certain terminal sizes, 468 could exceed. 500 is a round number safely above any realistic maximum.

### Why precompute all 10 digit target grids at startup

The alternative is computing targets on demand when each digit is assigned. But `precompute_targets()` scales and positions targets based on terminal size, which changes only on resize. Precomputing avoids repeated scaling arithmetic during the fast digit_assign path. The cost is `10 × 500 × 2 × 4 = 40 KB` of static storage — negligible.

### Why greedy NN rather than optimal assignment

The Hungarian algorithm gives minimum total travel distance but is O(n³) ≈ 500³ = 125M operations per digit change. Greedy NN is O(n²) ≈ 250K operations and produces routes that are visually indistinguishable — the extra travel distance is tiny and all particles travel simultaneously, so "long" routes don't visually dominate.

### Smoothstep vs linear lerp

Linear lerp (`x = ox + t*(tx-ox)`) has a visible jerk at t=0 (instant velocity change from 0 to max) and t=1 (instant stop). Smoothstep starts and ends with zero velocity, matching the physical intuition of a particle that accelerates then decelerates. The morph looks natural and continuous.

## Open Questions for Pass 3

- What assignment algorithm produces the most visually pleasing morph (not necessarily minimum distance)? For example, a **spiral assignment** that routes particles radially outward from the digit centre might create a bloom effect.
- Can the morph be **interrupted mid-transition** and redirected to a third digit? The snapshot mechanism already supports this — the `ox/oy` captures the current position, so a new `digit_assign()` would restart from wherever particles currently are.
- What font resolution (ppr × ppc sub-grid) is needed before the particle grid becomes indistinguishable from a solid filled region? At what terminal size does 500 particles become insufficient?
- Add a **velocity-based character selection**: use the instantaneous velocity `|dx/dt| = |tx-ox| × smoothstep'(t) / morph_frames` to select characters — fast-moving particles show `'.'`, slow ones show `'@'`, reversing the current scheme to simulate motion blur.

## From the Source

**Algorithm:** Greedy nearest-neighbour bipartite matching + LERP morph. On digit change: build target set for new digit; for each particle, find closest unassigned target (O(P·T) greedy scan); then linearly interpolate position from snapshot to target over MORPH_FRAMES frames using smoothstep easing.

**Math:** Smoothstep easing: f(t) = 3t² − 2t³, t ∈ [0,1]. Produces S-curve acceleration at start and deceleration at end — more visually pleasing than linear interpolation. Bitmap font: 9-row × 7-col bitmap; each '#' pixel expanded to a sub-grid of particles scaled to terminal dimensions.

**Performance:** Greedy matching is O(P·T) per digit change but only runs once per transition (not per frame). Per-frame cost is O(P) position update + draw — cheap at P≤500.
