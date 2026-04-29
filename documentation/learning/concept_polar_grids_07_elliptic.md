# Concept: Elliptic Polar Grid (07_elliptic.c)

## Pass 1 — Understanding

### Core Idea
Replace circular rings with concentric ellipses by stretching the x and y axes independently.  The "elliptic radius" `e_r = sqrt((dx/A)² + (dy/B)²)` generalises the standard polar radius to elliptic geometry.  Pressing 'h' overlays the orthogonal family — confocal hyperbolae — which cross every ellipse at a right angle.

### Mental Model
Take the circular polar grid (01) and stretch the x-axis by A and the y-axis by B.  Every circle becomes an ellipse; every ring-test fmod still works because we've already absorbed the stretch into the e_r formula.  When A = B the grid is a perfect circle — this is the 01 special case.

The hyperbola family: in electrostatics, if two parallel elliptic conductors are at opposite voltages, the electric field lines are confocal hyperbolae and the equipotential lines are confocal ellipses.  Pressing 'h' reveals both families simultaneously.

### Key Equations

**Elliptic radius:**
```
dx_px = (col − ox) × CELL_W    ← pixel-space coordinates
dy_px = (row − oy) × CELL_H
e_r   = sqrt((dx_px/A)² + (dy_px/B)²)
```
When A = B: `e_r = sqrt(dx_px² + dy_px²) / A = r_px / A` — standard polar radius (scaled).

**Elliptic ring detection:**
```
u    = e_r / RING_SPACING
frac = u − floor(u)
on_ring: frac < RING_W_U || frac > 1 − RING_W_U
```

**Character direction** (tangent to ellipse at each point):
```
ell_theta = atan2(dy_px/B, dx_px/A)    ← angle in "normalised" space
angle_char(ell_theta)                  ← tangent character aligned with ellipse
```
Using `ell_theta` instead of `atan2(dy_px, dx_px)` ensures the character aligns with the ellipse tangent, not the screen radial direction.

**Hyperbola overlay:**
```
cv = |cos(ell_theta)|              ∈ [0, 1]
hfrac = fmod(cv, HYPER_STEP)
on_hyper: hfrac < HYPER_W || hfrac > HYPER_STEP − HYPER_W
```
Level sets of `cos(ell_theta)` are the confocal hyperbolae.

### Non-Obvious Decisions

- **Two color pairs**: Ellipses and hyperbolae drawn in different colors (PAIR_GRID vs PAIR_HYPER) make the orthogonality visually clear.  A single color would merge the two families.
- **`ell_theta` for angle_char, not raw screen theta**: If we used `atan2(dy_px, dx_px)`, the characters would align with circular tangents, not elliptic tangents.  The stretched angle `atan2(dy/B, dx/A)` gives the correct tangent direction.
- **`fabs(cos(ell_theta))`**: cos(ell_theta) ∈ [−1, 1].  Taking absolute value folds to [0, 1] so the hyperbola test sees both left and right branches — a full hyperbola, not half.
- **AXIS_STEP = 0.1**: Fine enough that A/B can be tuned smoothly from circles to very elongated ellipses.  The range [0.5, 4.0] covers from 8:1 to 1:8 aspect ratios.

### Open Questions

- What does the grid look like when A >> B?  (Answer: nearly horizontal lines — the ellipses are very flat, almost like parallel horizontal rails.)
- Are the hyperbolae and ellipses truly orthogonal at every intersection?  (Answer: yes, by the confocal conic theorem — the tangent to a confocal ellipse is perpendicular to the tangent of the confocal hyperbola at every point of intersection.)
- If A = B and you show only hyperbolae (no ellipses), what do you see?  (Answer: straight radial lines at equal angular intervals — the hyperbolae collapse to spokes.)

---

## From the Source

**Algorithm:** Screen-sweep with e_r replacing r_px in the ring test.  PAIR_HYPER adds a second color pass for the hyperbola family.

**Math:** Elliptic coordinate system.  e_r = sqrt((x/A)²+(y/B)²) generalises polar radius.  Confocal hyperbolae are level sets of cos(ell_theta).

**Physical context:** Equipotential lines of an elliptic cylinder capacitor; acoustic resonances of an elliptic drum.

---

## Pass 2 — Implementation

### Pseudocode

```
cell_to_elliptic(col, row, ox, oy, A, B → e_r, ell_theta):
    dx_px = (col − ox) × CELL_W
    dy_px = (row − oy) × CELL_H
    u = dx_px / A;  v = dy_px / B
    e_r = sqrt(u² + v²)
    ell_theta = atan2(v, u)

grid_draw(rows, cols, ox, oy, A, B, ring_spacing, show_hyper):
    for each cell (row, col):
        (e_r, ell_theta) = cell_to_elliptic(…)
        if e_r < E_R_MIN: skip
        u = e_r / ring_spacing; frac = u − floor(u)
        on_ring = frac < RING_W_U || frac > 1 − RING_W_U
        if show_hyper:
            cv = |cos(ell_theta)|
            on_hyper = fmod(cv, HYPER_STEP) < HYPER_W || …
        char = angle_char(ell_theta)
        if on_ring && on_hyper: draw '+' in PAIR_HYPER
        elif on_hyper:          draw char in PAIR_HYPER
        elif on_ring:           draw char in PAIR_GRID
```

### Module Map

```
§1 config   — AXIS_A/B_DEFAULT/MIN/MAX/STEP, RING_SPACING, RING_W_U,
               HYPER_STEP/W, PAIR_HYPER (4th color pair)
§3 color    — color_init with THEME_HFG for PAIR_HYPER
§4 coords   — cell_to_elliptic (new: replaces cell_to_polar), angle_char
§5 draw     — grid_draw (elliptic ring + hyperbola overlay)
§6 scene    — scene_draw (HUD shows A, B, spacing; +hyper label)
§8 app      — 'h' toggles show_hyper; a/z adjust A; s/x adjust B
```

### Data Flow

```
getch('h')   → show_hyper = !show_hyper
getch('a/z') → axis_a ± AXIS_STEP
getch('s/x') → axis_b ± AXIS_STEP
scene_draw → grid_draw →
    cell_to_elliptic → ring test (PAIR_GRID) + optional hyper test (PAIR_HYPER)
    → mvaddch with appropriate color pair
```
