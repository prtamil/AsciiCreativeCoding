# Concept: Penrose Substitution (Robinson Triangles)

## Pass 1 — Understanding

### Core Idea
Penrose's classical aperiodic tiling implemented as a recursive substitution on two prototiles — the Robinson triangles. An "acute" isoceles (apex 36°) and an "obtuse" isoceles (apex 108°), both with golden-ratio side ratios, substitute into smaller copies of themselves. Pressing `+/-` changes recursion depth (0..10); leaf count grows roughly as 2^N. The resulting figure is a fragment of the Penrose P3 tiling — aperiodic with 5-fold rotational symmetry on average and self-similar at scale 1/φ.

### The Two Prototiles
```
Acute  (A): isoceles, apex angle 36°, base angles 72° each;  leg = φ, base = 1
Obtuse (B): isoceles, apex angle 108°, base angles 36° each; leg = 1, base = φ
```
where φ = (1 + √5) / 2 ≈ 1.618 — the golden ratio.

### Substitution Rules
Given an acute A with vertices (apex, base-left, base-right):
```
Split point P on the LEFT leg at distance 1 from apex (so apex→P = 1 and P→base-left = φ−1 = 1/φ).
Emit:
  acute child  (P, base-left, P_apex)        // shrunk acute, ratio 1/φ
  obtuse child (apex, base-right, P)         // a new obtuse from upper portion
```
Given an obtuse B with vertices (apex, base-left, base-right):
```
Split point Q on the BASE at distance 1 from base-left.
Emit:
  obtuse child (Q, apex, base-right)         // a new obtuse, ratio 1/φ
  acute child  (base-left, apex, Q)          // a new acute
```
Each substitution divides each prototile into ~2 children at scale 1/φ.

### Why Aperiodic
The substitution rules mix golden-ratio splits with rotations of 36° and 72°. Because φ is the unique positive root of x² = x + 1, any periodic tiling would require some integer linear relation between leg lengths — none exists. Equivalently, the Robinson matchings on the prototiles enforce non-periodic placement at every scale.

### 5-Fold Symmetry on Average
Local fragments of the tiling display 5-fold rotational symmetry. Globally there's no exact 5-fold centre, but the diffraction pattern (long-range order) exhibits 5-fold symmetry — the famous "quasi-crystalline" property.

### Connection to Other Files
- `10_pinwheel.c` — another aperiodic substitution (5-way split with √5 scaling).
- `09_sierpinski.c` — periodic 3-way self-similar split.

### Non-Obvious Decisions
- **Two prototiles, not one**: the substitution couples them — A produces one A and one B; B produces one B and one A. Single-prototile recursive splits cannot give aperiodic tilings (Berger's theorem implies the minimal aperiodic set requires multiple prototiles).
- **Depth vs visual quality**: 2^10 = 1024 leaves — beyond depth 8 the leaves drop below 1 character on a typical terminal. The default starts at 5 or 6.
- **Golden ratio constant**: PHI = (1.0 + sqrt(5.0)) / 2.0 is computed once at startup; all subdivisions use this same value.

### Key Constants
| Name | Role |
|------|------|
| `DEPTH` | Recursion depth (0..10) |
| `SIZE_FRAC` | Seed triangle size relative to screen |
| `PHI` | Golden ratio (1 + √5) / 2 |

### Open Questions
- What is the relationship between the Robinson triangles and the more famous Penrose kite + dart tiles?
- Why does no periodic tiling exist using these two prototiles? (Berger's theorem hint.)

---

## Pass 2 — Implementation

### Module Map
```
§1 config     — DEPTH, SIZE_FRAC, PHI
§3 color      — type-keyed palette (acute vs obtuse)
§4 formula    — slope_char + Bresenham line_draw
§5 substitute — golden-ratio split for each Robinson triangle
§6 scene      — seed_triangle (acute or obtuse) + scene_draw
§7 screen     — ncurses init / cleanup
§8 app        — signals, main loop
```

### Data Flow
```
seed_triangle (type A or B) → substitute(depth) → ~2^depth leaves
                                       ↓
                          line_draw 3 edges per leaf, colour by leaf type
```
