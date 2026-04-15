# Concept: L-System (Lindenmayer System)

## Pass 1 — Understanding

### Core Idea
A formal string rewriting system. Start with an axiom string. Each generation, replace every symbol according to production rules simultaneously. Interpret the resulting string as turtle graphics instructions. Used to model plant branching, fractals, and space-filling curves.

### Mental Model
Imagine a grammar: "F" means "draw forward," "+" means "turn right," "[" means "save position," "]" means "restore position." The axiom is a seed plant. The rules say how each part grows. After 5 generations, what started as "F" becomes a string millions of characters long that draws an entire tree.

### Key Equations
No numeric equations — it's symbolic:
```
Axiom: F
Rule:  F → F[+F]F[-F]F
```
After 1 step: F[+F]F[-F]F
After 2 steps: F[+F]F[-F]F [+ F[+F]F[-F]F] F[+F]F[-F]F [-F[+F]F[-F]F] F[+F]F[-F]F

Turtle interpretation:
- F: move forward `step` units, draw line
- +: rotate by +ANGLE
- -: rotate by -ANGLE
- [: push (x,y,angle) onto stack
- ]: pop (x,y,angle) from stack

### Data Structures
- String buffer (grows exponentially — limit to 6–8 generations)
- Turtle state: (x, y, angle)
- Stack: for branching [ and ]
- Production rules table: char → string

### Non-Obvious Decisions
- **String length explosion**: Each generation multiplies length by production rule length. Cap at generation 6–8 or the string becomes gigabytes.
- **Bounding box pre-pass**: Walk the turtle without drawing to find extent, then scale and center.
- **Stochastic L-systems**: Each symbol can have multiple productions chosen randomly. Gives natural-looking (non-symmetric) trees.
- **Parametric L-systems**: Symbols carry numeric parameters like F(length,width). Much more expressive, much more complex to implement.

### Classic Examples
| Name | Axiom | Rules | Angle |
|------|-------|-------|-------|
| Koch | F | F→F+F--F+F | 60° |
| Sierpinski | F+G+G | F→F+G-F-G+F, G→GG | 120° |
| Plant | X | X→F+[[X]-X]-F[-FX]+X, F→FF | 25° |
| Dragon | FX | X→X+YF, Y→FX-Y | 90° |

### Open Questions
- Why does increasing ORDER by 1 exactly double or square the string length?
- Can you render with variable line thickness based on branch depth?
- Implement a 3D L-system using 3D turtle (roll, pitch, yaw)?

---

## Pass 2 — Implementation

### Pseudocode
```
rewrite(s, rules) → new_string:
    result = ""
    for char in s:
        if char in rules: result += rules[char]
        else:             result += char
    return result

generate(axiom, rules, order):
    s = axiom
    for _ in range(order): s = rewrite(s, rules)
    return s

turtle_bounds(s, angle_deg):
    x=y=0, θ=0, stack=[]
    min_x=max_x=min_y=max_y=0
    for char in s:
        if char=='F': x+=cos(θ); y+=sin(θ); update bounds
        if char=='+': θ+=angle_deg*PI/180
        if char=='-': θ-=angle_deg*PI/180
        if char=='[': stack.push(x,y,θ)
        if char==']': x,y,θ=stack.pop()
    return (min_x,min_y,max_x,max_y)

draw_turtle(s, angle_deg, scale, offset):
    x=y=cx; θ=90°; stack=[]
    for char in s:
        if char=='F':
            x2=x+cos(θ)*scale, y2=y+sin(θ)*scale
            draw_line((x,y),(x2,y2))
            x=x2; y=y2
        ...
```

### Module Map
```
§1 config    — preset rules (Koch, plant, dragon, sierpinski)
§2 generate  — rewrite(), generate()
§3 bounds    — turtle_bounds() for auto-scale
§4 draw      — draw_turtle() with Bresenham lines
§5 app       — main, keys (order +/-, preset select, angle adjust)
```

### Data Flow
```
axiom + rules + ORDER → generate string
→ turtle_bounds → scale/offset
→ draw_turtle → line segments → screen
```

## From the Source

**Algorithm:** String-rewriting L-system with turtle-graphics rendering. Each generation replaces every variable in the string using a production rule table. The resulting string is then "executed" by a turtle: F=forward, +=turn_left, -=turn_right, [=push_state, ]=pop_state.

**Math:** L-systems (Lindenmayer, 1968) were invented to model plant cell division. The string length grows exponentially with generation: `|L_n| = |L_{n-1}| × avg_expansion_factor`. For the Dragon Curve rule F→F+G, G→F-G: `|L_n| = 2ⁿ`. The limit object (infinite generation) of space-filling L-systems (Hilbert, Peano) has Hausdorff dimension = 2 (fills area).

**Performance:** String length limit MAX_STR=1 MB (covers Koch gen 6 ~490 K, Hilbert gen 6 ~490 K, Branching Plant gen 7). The string is stored in a dynamic buffer; each rewrite step O(|string| × max_rule_length) copies characters. Aspect correction: forward step is (STEP_PX_COL, STEP_PX_ROW) accounting for CELL_W/CELL_H ratio to keep branches isotropic. STACK_MAX=128 for maximum branch nesting depth.

**References:** Lindenmayer, A. (1968) — original L-system paper modeling plant cell division.
