/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * beam_bending.c — Euler-Bernoulli Beam Bending + Vibration Simulator
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  PHYSICS OVERVIEW
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  The Euler-Bernoulli beam model assumes:
 *    1. Cross-sections remain plane and perpendicular to the neutral axis.
 *    2. Deflections are small (linear theory — no geometric nonlinearity).
 *    3. Material is linearly elastic (stress proportional to strain).
 *
 *  Governing equation (beam axis = x, transverse deflection = w):
 *
 *      EI · d⁴w/dx⁴ = q(x)
 *
 *  where:
 *    E  = Young's modulus   [Pa]   — material stiffness
 *    I  = second moment of area [m⁴] — cross-section shape resistance
 *    EI = flexural rigidity — the single stiffness parameter that matters
 *    q  = distributed transverse load [N/m]
 *    w  = transverse deflection [m]   (positive = downward here)
 *
 *  Bending moment — what you feel as a beam bends:
 *      M(x) = EI · d²w/dx²    (curvature × stiffness)
 *
 *  Bending stress at distance y from neutral axis:
 *      σ(x,y) = M(x)·y / I
 *  → High |M| = high stress = higher risk of yielding/fracture.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  BOUNDARY CONDITIONS — what makes each beam type unique
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Simply Supported (SS):   w(0)=0, M(0)=0, w(L)=0, M(L)=0
 *    Both ends pinned. Beam can rotate at supports. Moment is zero at
 *    the pins — the support can only push up/down, not apply a couple.
 *    → Symmetric deflection under symmetric loads.
 *    → Max deflection = PL³/(48EI) at midspan for centre point load.
 *
 *  Cantilever (Fixed-Free): w(0)=0, w'(0)=0, M(L)=0, V(L)=0
 *    Left end fully clamped — cannot deflect or rotate. Right end free.
 *    The wall provides both a reaction force AND a reaction moment.
 *    → Largest tip deflection: PL³/(3EI) — 16× more than SS midspan!
 *    → Moment is maximum at the wall and zero at the free tip.
 *
 *  Fixed-Fixed (FF):        w(0)=0, w'(0)=0, w(L)=0, w'(L)=0
 *    Both ends fully clamped. Both ends provide moment reactions.
 *    → Stiffest configuration: midspan deflection PL³/(192EI).
 *    → Hogging moments (tension at top) develop at both clamped ends.
 *    → Sagging moment at midspan. Moment diagram crosses zero twice.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  DYNAMIC MODE — modal superposition
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Free vibration: each mode n has a natural frequency ωₙ and a mode
 *  shape φₙ(x). The response is a sum over all modes:
 *
 *      w(x,t) = Σ qₙ(t) · φₙ(x)
 *
 *  Each modal coordinate qₙ(t) obeys an independent damped oscillator:
 *
 *      q̈ₙ + 2ζωₙ q̇ₙ + ωₙ² qₙ = Fₙ(t)
 *
 *  where ζ = damping ratio, Fₙ = modal load = ∫ q(x)φₙ(x)dx / Mₙ.
 *
 *  Stepped with exact transition matrix (unconditionally stable).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  VISUALIZATION DESIGN
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Main area:  deflected beam shape, curvature shading, support symbols,
 *              load arrows. Character density encodes |M(x)|/M_max.
 *              Color: green (low M) → yellow (mid) → red (high M).
 *  Side panel: bending moment diagram, horizontal bars ±M(x)/M_max.
 *  HUD row 0:  BC type, load type, fps.
 *  HUD row -2: max deflection, max moment, load P, exaggeration factor.
 *  HUD row -1: key reference.
 *
 * Keys:
 *   q/ESC  quit       space   pause/resume    d  toggle dynamic mode
 *   b      cycle BC   l       cycle load      r  reset
 *   +/-    load P     e/E     deflection exag
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/beam_bending.c -o beam_bending -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config   §2 clock    §3 color
 *  §4 beam     §5 solver   §6 dynamics   §7 render   §8 scene
 *  §9 screen   §10 app
 * ─────────────────────────────────────────────────────────────────────
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_NODES       200    /* beam discretisation points                 */
#define BEAM_HEIGHT     3    /* visible cross-section height in rows (odd) */
#define PANEL_W        22    /* right-side moment diagram width (cols)     */
#define BEAM_X_MARGIN   4    /* empty cols on each side of beam span       */
#define LOAD_ARROW_H    4    /* rows of load arrow above beam top          */
#define TARGET_FPS     30    /* render frame cap                           */
#define RAMP_SPEED    0.7f   /* load ramp: full load in ~1.4 s             */
#define N_MODES         4    /* eigenmodes used in dynamic superposition   */
#define MODAL_DAMP    0.025f /* ζ — damping ratio (2.5 % of critical)      */

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL

/*
 * Color pairs — role-named so color choices don't spread through code.
 * Rule: assign roles first; pick colors second. Changing a palette entry
 * here affects every place that role is used.
 */
#define CP_LO     1   /* low curvature zone  — green   */
#define CP_MED    2   /* mid curvature zone  — yellow  */
#define CP_HI     3   /* high curvature zone — red     */
#define CP_MOM_P  4   /* positive (sagging) moment bar — lime   */
#define CP_MOM_N  5   /* negative (hogging) moment bar — magenta*/
#define CP_LOAD   6   /* load arrow / label            — gold   */
#define CP_SUPP   7   /* support symbols               — white  */
#define CP_HUD    8   /* overlay text                  — cyan   */
#define CP_DIM    9   /* reference lines / dim text    — grey   */
#define CP_HDR   10   /* panel header                  — orange */

typedef enum { BC_SS=0, BC_CANT, BC_FF,  BC_COUNT  } BCType;
typedef enum { LD_CENTER=0, LD_UDL, LD_OFFSET, LD_COUNT } LDType;

static const char *bc_names[BC_COUNT] = {
    "Simply Supported",
    "Cantilever (Fixed-Free)",
    "Fixed-Fixed",
};
static const char *ld_names[LD_COUNT] = {
    "Center Point Load",
    "Uniform Distributed Load",
    "Offset Point (3/4 span)",
};

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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
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
        init_pair(CP_LO,    46, COLOR_BLACK);  /* bright green  */
        init_pair(CP_MED,  226, COLOR_BLACK);  /* yellow        */
        init_pair(CP_HI,   196, COLOR_BLACK);  /* red           */
        init_pair(CP_MOM_P, 82, COLOR_BLACK);  /* lime green    */
        init_pair(CP_MOM_N,201, COLOR_BLACK);  /* magenta       */
        init_pair(CP_LOAD, 220, COLOR_BLACK);  /* gold          */
        init_pair(CP_SUPP, 255, COLOR_BLACK);  /* bright white  */
        init_pair(CP_HUD,   51, COLOR_BLACK);  /* cyan          */
        init_pair(CP_DIM,  240, COLOR_BLACK);  /* dark grey     */
        init_pair(CP_HDR,  214, COLOR_BLACK);  /* orange        */
    } else {
        init_pair(CP_LO,   COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_MED,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_HI,   COLOR_RED,     COLOR_BLACK);
        init_pair(CP_MOM_P,COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_MOM_N,COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_LOAD, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_SUPP, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HUD,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_DIM,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HDR,  COLOR_YELLOW,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  beam entity                                                        */
/* ===================================================================== */

/*
 * DynBeam — state for the free-vibration dynamic mode.
 *
 * Eigenvalue problem derivation (sketch):
 *   Assuming w(x,t) = φ(x)·T(t), separating variables gives:
 *     EI·φ'''' = ρA·ω²·φ     (spatial ODE)
 *     T̈ + ω²T = 0            (temporal ODE)
 *
 *   Solutions to the spatial ODE: φ(x) = C₁cosh(βx) + C₂sinh(βx)
 *                                       + C₃cos(βx) + C₄sin(βx)
 *   where β⁴ = ρA·ω²/(EI).  Applying boundary conditions yields the
 *   characteristic equation whose roots give the eigenvalues.
 *
 * Tabulated β·L values (ρA=EI=1):
 *   SS:    βₙL = nπ              (exact — characteristic eq: sin(βL)=0)
 *   Cant:  characteristic eq: cos(βL)·cosh(βL) + 1 = 0
 *          roots: 1.87510, 4.69409, 7.85476, 10.99554
 *   FF:    characteristic eq: cos(βL)·cosh(βL) - 1 = 0
 *          roots: 4.73004, 7.85321, 10.99561, 14.13717
 *
 * High-order cantilever/FF mode shapes involve large cosh values that
 * nearly cancel with cos — computed in double to preserve accuracy.
 *
 * ref_max_w: the static deflection scale captured at mode activation,
 *   held fixed for the whole run so the beam size does not jump as
 *   oscillation amplitude changes.
 */
typedef struct {
    float phi[N_MODES][N_NODES]; /* mode shapes φₙ(xᵢ) at N_NODES stations */
    float omega[N_MODES];        /* natural frequencies ωₙ [rad/s]          */
    float modal_mass[N_MODES];   /* Mₙ = ρA·∫φₙ²dx  (ρA=1 here)            */
    float q[N_MODES];            /* modal coordinates qₙ(t)                 */
    float qdot[N_MODES];         /* modal velocities q̇ₙ(t)                  */
    float Fn[N_MODES];           /* generalised modal forces                */
    float ref_max_w;             /* fixed visual amplitude reference        */
    bool  active;
} DynBeam;

/*
 * Beam — complete simulator state.
 *
 * Coordinates are normalised: L=1.0, EI=1.0.  Real units are embedded
 * in P (load magnitude), and results are only shown normalised on-screen.
 * This makes the solver formulas clean and dimensionless.
 *
 * w[i]          deflection at node i (positive = downward)
 * M[i]          bending moment at node i (positive = sagging)
 * x[i]          normalised position = i/(N_NODES-1) ∈ [0,1]
 * load_anim     ∈ [0,1] — ramp factor applied during load application
 * exag          visual multiplier so hairline deflections become visible
 * max_deflection used to normalise the y-axis of the deflection display
 * max_moment     used to normalise the moment diagram
 */
typedef struct {
    float w[N_NODES];
    float M[N_NODES];
    float x[N_NODES];

    float L, EI, P;
    float exag;
    float load_anim;
    bool  paused;

    BCType bc;
    LDType load;

    float max_deflection;
    float max_moment;

    DynBeam dyn;
} Beam;

/* apply_load — set BC and load type, reset animation ramp */
static void apply_load(Beam *b, BCType bc, LDType ld, float P)
{
    b->bc   = bc;
    b->load = ld;
    b->P    = P;
}

static void init_beam(Beam *b)
{
    memset(b, 0, sizeof *b);
    b->L         = 1.0f;
    b->EI        = 1.0f;
    b->exag      = 1.0f;
    b->load_anim = 0.0f;
    b->paused    = false;
    for (int i = 0; i < N_NODES; i++)
        b->x[i] = (float)i / (float)(N_NODES - 1);
    apply_load(b, BC_SS, LD_CENTER, 1.0f);
}

/* ===================================================================== */
/* §5  solver — closed-form Euler-Bernoulli solutions                     */
/* ===================================================================== */

/*
 * solve_beam — fill w[] and M[] with the analytical static solution.
 *
 * All formulas derived by integrating EI·w'''' = q(x) four times and
 * applying boundary conditions.  Source: Roark's Formulas for Stress
 * and Strain, 8th edition.
 *
 * Load is scaled by load_anim so we can animate the ramp from zero.
 * Sign convention: downward deflection positive, sagging moment positive.
 *
 * ── Simply Supported, centre point load P at a=L/2 ──────────────────
 *   Reactions: Ra = Rb = P/2.
 *   For x ≤ a:
 *     M(x) = P·b·x/L           (linear rise to midspan)
 *     w(x) = P·b·x(L²-b²-x²)/(6EIL)
 *   where b = L-a.  Mirror for x > a.
 *   Physical insight: the beam sags symmetrically; moment peaks at midspan.
 *
 * ── Simply Supported, UDL q over full span ──────────────────────────
 *     M(x) = q·x·(L-x)/2      (parabola, peak at midspan)
 *     w(x) = q·x(L³-2Lx²+x³)/(24EI)
 *   Physical insight: 4th-order polynomial deflection, symmetric.
 *
 * ── Cantilever, tip load P ───────────────────────────────────────────
 *     M(x) = P·(L-x)           (linear, max at wall, zero at tip)
 *     w(x) = P·x²·(3L-x)/(6EI)
 *   Tip deflection: δ_tip = PL³/(3EI) — 16× a SS beam midspan!
 *   Physical insight: the wall carries all shear AND moment reaction.
 *
 * ── Fixed-Fixed, centre load ─────────────────────────────────────────
 *     Fixed-end moments: Ma = Mb = PL/8 (hogging, shown as negative here)
 *     For x ≤ L/2:
 *       M(x) = Px/2 - PL/8
 *       w(x) = P·x²·(3L-4x)/(48EI)
 *   Physical insight: clamped ends build up negative (hogging) moments,
 *   partially counteracting midspan sag → stiffest of the three BCs.
 */
static void solve_beam(Beam *b)
{
    float L  = b->L;
    float EI = b->EI;
    float P  = b->P * b->load_anim;   /* animated load magnitude */

    float max_w = 0.0f, max_M = 0.0f;

    for (int i = 0; i < N_NODES; i++) {
        float x = b->x[i] * L;
        float w = 0.0f, M = 0.0f;

        switch (b->bc) {

        /* ── Simply Supported ──────────────────────────────────── */
        case BC_SS:
            switch (b->load) {

            case LD_CENTER: {
                float a = 0.5f*L, bv = 0.5f*L;
                if (x <= a) {
                    M = P*bv*x / L;
                    w = P*bv*x*(L*L - bv*bv - x*x) / (6.0f*EI*L);
                } else {
                    float xs = L - x;         /* mirror symmetry */
                    M = P*a*xs / L;
                    w = P*a*xs*(L*L - a*a - xs*xs) / (6.0f*EI*L);
                }
                break;
            }

            case LD_UDL: {
                float q = P;
                M = q*x*(L - x) / 2.0f;
                w = q*x*(L*L*L - 2.0f*L*x*x + x*x*x) / (24.0f*EI);
                break;
            }

            case LD_OFFSET: {   /* point load at a = 3L/4 */
                float a = 0.75f*L, bv = 0.25f*L;
                if (x <= a) {
                    M = P*bv*x / L;
                    w = P*bv*x*(L*L - bv*bv - x*x) / (6.0f*EI*L);
                } else {
                    float xs = L - x;
                    M = P*a*xs / L;
                    w = P*a*xs*(L*L - a*a - xs*xs) / (6.0f*EI*L);
                }
                break;
            }

            default: break;
            }
            break;

        /* ── Cantilever (fixed left, free right) ───────────────── */
        case BC_CANT:
            switch (b->load) {

            case LD_CENTER: {   /* "centre" = tip for cantilever */
                M = P*(L - x);
                w = P*x*x*(3.0f*L - x) / (6.0f*EI);
                break;
            }

            case LD_UDL: {
                float q = P;
                M = q*(L - x)*(L - x) / 2.0f;
                w = q*x*x*(6.0f*L*L - 4.0f*L*x + x*x) / (24.0f*EI);
                break;
            }

            case LD_OFFSET: {   /* point load at mid-span a = L/2 */
                float a = 0.5f*L;
                if (x <= a) {
                    M = P*(a - x);
                    w = P*x*x*(3.0f*a - x) / (6.0f*EI);
                } else {
                    /* Beyond load point: M=0, w follows cubic from a */
                    M = 0.0f;
                    w = P*a*a*(3.0f*x - a) / (6.0f*EI);
                }
                break;
            }

            default: break;
            }
            break;

        /* ── Fixed-Fixed ────────────────────────────────────────── */
        case BC_FF:
            switch (b->load) {

            case LD_CENTER: {
                /* Fixed-end moment = PL/8 (hogging, sign kept explicit) */
                if (x <= 0.5f*L) {
                    M = P*x/2.0f - P*L/8.0f;
                    w = P*x*x*(3.0f*L - 4.0f*x) / (48.0f*EI);
                } else {
                    float xs = L - x;
                    M = P*xs/2.0f - P*L/8.0f;
                    w = P*xs*xs*(3.0f*L - 4.0f*xs) / (48.0f*EI);
                }
                break;
            }

            case LD_UDL: {
                float q = P;
                /* Fixed-end moments = qL²/12 each */
                M = q*x*(L-x)/2.0f - q*L*L/12.0f;
                w = q*x*x*(L-x)*(L-x) / (24.0f*EI);
                break;
            }

            case LD_OFFSET: {   /* load at a = 3L/4 */
                float a  = 0.75f*L;
                float bv = L - a;
                /* Stiffness-method fixed-end moments: Ma = Pab²/L², Mb = Pa²b/L² */
                float MA = P*a*bv*bv / (L*L);
                float RA = P*bv*bv*(3.0f*a + bv) / (L*L*L);
                M = -MA + RA*x - (x > a ? P*(x - a) : 0.0f);
                if (x <= a)
                    w = P*bv*bv*x*x*(3.0f*a*L - (3.0f*a+bv)*x) / (6.0f*EI*L*L*L);
                else {
                    float xs = L - x;
                    w = P*a*a*xs*xs*(3.0f*bv*L - (3.0f*bv+a)*xs) / (6.0f*EI*L*L*L);
                }
                break;
            }

            default: break;
            }
            break;

        default: break;
        }

        b->w[i] = w;
        b->M[i] = M;
        if (fabsf(w) > max_w) max_w = fabsf(w);
        if (fabsf(M) > max_M) max_M = fabsf(M);
    }

    b->max_deflection = max_w;
    b->max_moment     = max_M;
}

/* ===================================================================== */
/* §6  dynamics — modal superposition                                     */
/* ===================================================================== */

/*
 * dyn_setup — precompute mode shapes φₙ(x) and natural frequencies ωₙ.
 *
 * For Simply Supported beams the mode shapes are pure sines — no
 * cancellation issues.  For Cantilever and Fixed-Fixed the mode shapes
 * are combinations of hyperbolic and trigonometric functions:
 *
 *   φₙ(x) = cosh(βₙx) - cos(βₙx) - σₙ·[sinh(βₙx) - sin(βₙx)]
 *
 * where σₙ is determined by applying the boundary conditions at x=L.
 * For cantilever: σₙ = (cos βₙL + cosh βₙL) / (sin βₙL + sinh βₙL)
 *
 * WHY double precision: cosh(βₙL) grows as e^(βₙL)/2.  For mode 4 of
 * a cantilever βₙL≈11, so cosh≈30,000.  When subtracting the nearly-
 * equal cos and sinh terms, float loses 4-5 significant digits — the
 * mode shape collapses to noise.  Double keeps 15 digits, enough to
 * survive the cancellation.  Store as float once computed (display only).
 *
 * Natural frequency from β: ωₙ² = (βₙ)⁴·EI/(ρA) = (βₙ)⁴  (EI=ρA=1)
 */
static void dyn_setup(Beam *b)
{
    DynBeam *d  = &b->dyn;
    double   L  = (double)b->L;
    double   dx = L / (double)(N_NODES - 1);

    if (b->bc == BC_SS) {
        /* Exact sine modes — characteristic equation sin(βL)=0 → βₙ=nπ/L */
        for (int n = 0; n < N_MODES; n++) {
            double beta  = (double)(n + 1) * 3.14159265358979323846 / L;
            d->omega[n]  = (float)(beta * beta);   /* ωₙ = βₙ² for EI=ρA=1 */
            for (int i = 0; i < N_NODES; i++)
                d->phi[n][i] = (float)sin(beta * b->x[i] * L);
        }
    } else if (b->bc == BC_CANT) {
        /* Tabulated roots of: cos(βL)·cosh(βL) + 1 = 0 */
        static const double lam[N_MODES] = {1.87510, 4.69409, 7.85476, 10.99554};
        for (int n = 0; n < N_MODES; n++) {
            double bn  = lam[n] / L;
            double sig = (cos(lam[n]) + cosh(lam[n]))
                       / (sin(lam[n]) + sinh(lam[n]));
            d->omega[n] = (float)(bn * bn);
            for (int i = 0; i < N_NODES; i++) {
                double x = b->x[i] * L;
                d->phi[n][i] = (float)(
                    cosh(bn*x) - cos(bn*x) - sig*(sinh(bn*x) - sin(bn*x)));
            }
        }
    } else {  /* BC_FF — roots of: cos(βL)·cosh(βL) - 1 = 0 */
        static const double lam[N_MODES] = {4.73004, 7.85321, 10.99561, 14.13717};
        for (int n = 0; n < N_MODES; n++) {
            double bn  = lam[n] / L;
            double den = sin(lam[n]) - sinh(lam[n]);
            /* Guard: den→0 near degenerate roots (shouldn't occur for these λ) */
            double sig = (fabs(den) > 1e-10)
                       ? (cos(lam[n]) - cosh(lam[n])) / den : -1.0;
            d->omega[n] = (float)(bn * bn);
            for (int i = 0; i < N_NODES; i++) {
                double x = b->x[i] * L;
                d->phi[n][i] = (float)(
                    cosh(bn*x) - cos(bn*x) - sig*(sinh(bn*x) - sin(bn*x)));
            }
        }
    }

    /*
     * Modal mass Mₙ = ρA·∫φₙ²dx  (ρA=1, trapezoidal integration).
     * Normalises the mode shape so Fₙ has consistent units.
     * If a mode numerically degenerates (Mₙ≈0), clamp to 1 to avoid /0.
     */
    for (int n = 0; n < N_MODES; n++) {
        double mm = 0.0;
        for (int i = 0; i < N_NODES; i++)
            mm += (double)d->phi[n][i] * d->phi[n][i] * dx;
        d->modal_mass[n] = (float)(mm > 1e-9 ? mm : 1.0);
    }
}

/*
 * dyn_load — project load q(x) onto each mode to get generalised forces.
 *
 *   Fₙ = (1/Mₙ) · ∫ q(x)·φₙ(x) dx
 *
 * For point loads at position xf, the Dirac-delta integral gives:
 *   Fₙ = P·φₙ(xf) / Mₙ
 *
 * For UDL, numerical integration (rectangle rule) over all nodes.
 */
static void dyn_load(Beam *b)
{
    DynBeam *d  = &b->dyn;
    float    L  = b->L;
    float    P  = b->P;
    float    dx = L / (float)(N_NODES - 1);

    for (int n = 0; n < N_MODES; n++) {
        float Fn = 0.0f;
        if (b->load == LD_UDL) {
            for (int i = 0; i < N_NODES; i++)
                Fn += P * d->phi[n][i] * dx;
        } else {
            float xf = (b->load == LD_CENTER) ? 0.5f : 0.75f;
            int   ni = (int)(xf * (float)(N_NODES - 1) + 0.5f);
            Fn = P * d->phi[n][ni];
        }
        d->Fn[n] = Fn / d->modal_mass[n];
    }
}

/*
 * dyn_activate — switch from static to dynamic oscillation mode.
 *
 * Initial conditions: q=0, qdot=0 (beam starts from rest at zero
 * deflection, then load is suddenly applied as a step function).
 *
 * Dynamic magnification: under a suddenly-applied constant load, the
 * response overshoots the static deflection by up to 2× on the first
 * half-cycle (dynamic magnification factor = 2 for undamped system).
 * Damping reduces this; ζ=0.025 gives visible but quickly-decaying
 * oscillation — representative of lightly-damped structural steel.
 */
static void dyn_activate(Beam *b)
{
    b->load_anim = 1.0f;
    solve_beam(b);                        /* get static solution + max_w */

    DynBeam *d   = &b->dyn;
    d->ref_max_w = b->max_deflection;     /* lock display scale          */

    dyn_setup(b);
    dyn_load(b);

    for (int n = 0; n < N_MODES; n++) {
        d->q[n]    = 0.0f;
        d->qdot[n] = 0.0f;
    }
    d->active = true;
}

/*
 * dyn_tick_modes — advance all modal oscillators by one timestep dt.
 *
 * Each mode: q̈ + 2ζω·q̇ + ω²·q = F
 *
 * WHY NOT explicit Euler: Euler approximates e^(iωdt)≈1+iωdt, which
 * has magnitude > 1.  Energy grows each step → numerical explosion when
 * ω·dt > 2.  At ω₄≈120 rad/s and dt=1/30 s, ω·dt≈4 — Euler diverges
 * within a few steps.
 *
 * Exact solution for constant F over [t, t+dt]:
 *   Let η = q - q_s   where q_s = F/ω² (static equilibrium offset)
 *   η(t) is a homogeneous damped oscillator:
 *     η(t+dt) = e^{-ζωdt} · [A·cos(ωd·dt) + B·sin(ωd·dt)]
 *   where ωd = ω·√(1-ζ²)  (damped natural frequency)
 *         A = η(t)
 *         B = (η̇(t) + ζω·η(t)) / ωd
 *
 * This is unconditionally stable for any dt and any ω.  The eigenvalues
 * of the transition matrix all have magnitude exactly e^{-ζωdt} < 1.
 */
static void dyn_tick_modes(Beam *b, float dt)
{
    DynBeam *d    = &b->dyn;
    float    zeta = MODAL_DAMP;

    for (int n = 0; n < N_MODES; n++) {
        float omega = d->omega[n];
        float q_s   = (omega > 1e-6f) ? d->Fn[n] / (omega * omega) : 0.0f;
        float wd    = omega * sqrtf(1.0f - zeta * zeta);
        if (wd < 1e-6f) wd = 1e-6f;  /* clamp overdamped edge case */

        float eta  = d->q[n] - q_s;
        float etad = d->qdot[n];
        float A    = eta;
        float B    = (etad + zeta * omega * A) / wd;
        float e    = expf(-zeta * omega * dt);
        float c    = cosf(wd * dt);
        float s    = sinf(wd * dt);

        d->q[n]    = q_s + e * (A*c + B*s);
        d->qdot[n] = e * ((-zeta*omega*A + wd*B)*c
                        + (-zeta*omega*B - wd*A)*s);
    }

    /* Reconstruct physical deflection: w(x) = Σₙ qₙ(t)·φₙ(x) */
    float max_w = 0.0f;
    for (int i = 0; i < N_NODES; i++) {
        float w = 0.0f;
        for (int n = 0; n < N_MODES; n++)
            w += d->q[n] * d->phi[n][i];
        b->w[i] = w;
        if (fabsf(w) > max_w) max_w = fabsf(w);
    }
    /* Only expand scale; never shrink (avoids visual jitter near zero crossing) */
    if (max_w > d->ref_max_w * 1.05f) d->ref_max_w = max_w;
    b->max_deflection = d->ref_max_w;
}

/* beam_tick — per-frame update dispatcher */
static void beam_tick(Beam *b, float dt)
{
    if (b->paused) return;

    if (b->dyn.active) {
        dyn_tick_modes(b, dt);
    } else {
        /* Static mode: animate the load ramp, then re-solve analytically */
        if (b->load_anim < 1.0f) {
            b->load_anim += dt * RAMP_SPEED;
            if (b->load_anim > 1.0f) b->load_anim = 1.0f;
        }
        solve_beam(b);
    }
}

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/* node_col — map beam node index to screen column */
static int node_col(int i, int x0, int x1)
{
    return x0 + i * (x1 - x0) / (N_NODES - 1);
}

/*
 * defl_row — convert deflection w to row offset from the neutral axis.
 *
 * The base_rows range is rows/6 (a fraction of screen height), scaled
 * by the exaggeration factor so even tiny deflections become visible.
 * Clamped to 3×base_rows so the beam never exits the visible area.
 */
static int defl_row(float w, float max_w, int base_rows, float exag)
{
    if (max_w < 1e-9f) return 0;
    int dr  = (int)(w / max_w * (float)base_rows * exag + 0.5f);
    int cap = base_rows * 3;
    return (dr >  cap) ?  cap
         : (dr < -cap) ? -cap
         : dr;
}

/*
 * draw_curvature_column — render one column of the beam cross-section.
 *
 * Character density encodes |M|/M_max (bending stress proxy):
 *   " .-~=*#@"  — 8 levels, sparse = low stress, dense = high stress.
 *
 * Color encodes the same quantity coarsely:
 *   green (< 35 %) → yellow (35–70 %) → red (> 70 %)
 *
 * The top and bottom flanges are always '=' to hint at the cross-section
 * shape; the web is the density character for interior cells.
 *
 * WHY this mapping: the visible character density intuitively reads as
 * "how hard is the material working here?" — dense chars look stressed.
 * Color reinforces it with traffic-light semantics (green = safe).
 */
static void draw_curvature_column(int col, int mid_row, float kn,
                                  int half_h, int rows)
{
    static const char pal[] = " .-~=*#@";
    int npal = (int)(sizeof pal - 1);

    int pi   = (int)(kn * (float)(npal - 1) + 0.5f);
    if (pi < 0) pi = 0;
    if (pi >= npal) pi = npal - 1;
    chtype fill = (chtype)pal[pi];

    int cp = (kn < 0.35f) ? CP_LO : (kn < 0.70f) ? CP_MED : CP_HI;

    for (int rr = mid_row - half_h; rr <= mid_row + half_h; rr++) {
        if (rr < 2 || rr >= rows - 2) continue;
        bool is_flange = (rr == mid_row - half_h || rr == mid_row + half_h);
        chtype ch   = is_flange ? '=' : fill;
        attr_t attr = (chtype)COLOR_PAIR(cp) | (rr == mid_row ? A_BOLD : 0);
        attron(attr);
        mvaddch(rr, col, ch);
        attroff(attr);
    }
}

/*
 * draw_load_arrows — draw load application symbols above the beam.
 *
 * Point loads: a vertical stem '|' and downward arrowhead 'v'.
 * UDL: repeated '|' + 'v' arrows at every 3rd column across the span.
 */
static void draw_load_arrows(const Beam *b, int x0, int x1,
                              int neutral_row, int base_rows, int half_h,
                              int cols, int rows)
{
    (void)cols;
    attron(COLOR_PAIR(CP_LOAD) | A_BOLD);

    switch (b->load) {

    case LD_CENTER:
    case LD_OFFSET: {
        float xfrac = (b->load == LD_CENTER) ? 0.5f : 0.75f;
        int   lc    = x0 + (int)(xfrac * (float)(x1 - x0) + 0.5f);
        int   ni    = (int)(xfrac * (float)(N_NODES - 1));
        int   dr    = defl_row(b->w[ni], b->max_deflection, base_rows, b->exag);
        int   top   = neutral_row + dr - half_h;

        for (int rr = top - LOAD_ARROW_H; rr < top - 1; rr++)
            if (rr >= 2 && rr < rows - 2) mvaddch(rr, lc, '|');
        if (top - LOAD_ARROW_H >= 2)
            mvprintw(top - LOAD_ARROW_H, lc - 1, "P");
        if (top - 1 >= 2) mvaddch(top - 1, lc, 'v');
        break;
    }

    case LD_UDL: {
        for (int c = x0; c <= x1; c += 3) {
            int ni = (c - x0) * (N_NODES - 1) / (x1 - x0);
            if (ni < 0) ni = 0;
            if (ni >= N_NODES) ni = N_NODES - 1;
            int dr  = defl_row(b->w[ni], b->max_deflection, base_rows, b->exag);
            int top = neutral_row + dr - half_h;
            if (top - 1 >= 2 && top - 1 < rows - 2) mvaddch(top - 1, c, 'v');
            if (top - 2 >= 2 && top - 2 < rows - 2) mvaddch(top - 2, c, '|');
        }
        int mc = (x0 + x1) / 2;
        if (neutral_row - half_h - 3 >= 2)
            mvprintw(neutral_row - half_h - 3, mc - 3, "q=%.2f", b->P * b->load_anim);
        break;
    }

    default: break;
    }

    attroff(COLOR_PAIR(CP_LOAD) | A_BOLD);
}

/*
 * draw_supports — draw boundary condition symbols.
 *
 * SS:  'A' triangles + '/_%c\' base line (standard pin-on-roller symbol).
 * Cantilever: hatch '#' on left wall; "free" label at right tip.
 * Fixed-Fixed: hatch '#' walls on both ends.
 */
static void draw_supports(BCType bc, int x0, int x1,
                          int neutral_row, int half_h,
                          int cols, int rows)
{
    attron(COLOR_PAIR(CP_SUPP) | A_BOLD);

    switch (bc) {

    case BC_SS: {
        int sr = neutral_row + half_h + 1;
        if (sr < rows - 2) {
            mvaddch(sr, x0, 'A');
            mvaddch(sr, x1, 'A');
        }
        if (sr + 1 < rows - 2) {
            mvprintw(sr + 1, x0 - 1, "/_%c", '\\');
            mvprintw(sr + 1, x1 - 1, "/_%c", '\\');
        }
        break;
    }

    case BC_CANT: {
        for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1; rr++) {
            if (rr < 1 || rr >= rows - 2) continue;
            if (x0 - 1 >= 0)   mvaddch(rr, x0 - 1, '#');
            if (x0 - 2 >= 0)   mvaddch(rr, x0 - 2, '#');
        }
        if (neutral_row >= 1 && neutral_row < rows - 2 && x1 + 1 < cols)
            mvprintw(neutral_row, x1 + 1, "free");
        break;
    }

    case BC_FF: {
        for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1; rr++) {
            if (rr < 1 || rr >= rows - 2) continue;
            if (x0 - 1 >= 0)    mvaddch(rr, x0 - 1, '#');
            if (x1 + 1 < cols)  mvaddch(rr, x1 + 1, '#');
        }
        break;
    }

    default: break;
    }

    attroff(COLOR_PAIR(CP_SUPP) | A_BOLD);
}

/*
 * render_beam — compose the deflected beam: neutral axis, body, loads, supports.
 *
 * Abstraction layers:
 *   draw_curvature_column  — one column of beam cross-section
 *   draw_load_arrows       — load application markers
 *   draw_supports          — boundary condition symbols
 *
 * All three layers use the same base geometry (x0/x1/neutral_row/base_rows).
 */
static void render_beam(const Beam *b, int cols, int rows)
{
    int x0 = BEAM_X_MARGIN;
    int x1 = cols - PANEL_W - BEAM_X_MARGIN - 1;
    if (x1 <= x0 + 10) return;

    int neutral_row = rows / 2;
    int base_rows   = rows / 6;
    if (base_rows < 3) base_rows = 3;
    int half_h = BEAM_HEIGHT / 2;

    /* Neutral axis reference line — where w=0 */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for (int c = x0; c <= x1; c++) mvaddch(neutral_row, c, '-');
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    /* Beam body — one column per node */
    for (int i = 0; i < N_NODES; i++) {
        int   c   = node_col(i, x0, x1);
        if (c < 0 || c >= cols) continue;

        /* kn = normalised curvature proxy |M(x)|/M_max ∈ [0,1] */
        float kn  = (b->max_moment > 1e-9f)
                    ? fabsf(b->M[i]) / b->max_moment : 0.0f;

        int   dr  = defl_row(b->w[i], b->max_deflection, base_rows, b->exag);
        draw_curvature_column(c, neutral_row + dr, kn, half_h, rows);
    }

    draw_load_arrows(b, x0, x1, neutral_row, base_rows, half_h, cols, rows);
    draw_supports(b->bc, x0, x1, neutral_row, half_h, cols, rows);
}

/*
 * render_moment_panel — right-side bending moment diagram.
 *
 * Layout: each screen row maps to a beam station; horizontal bars show
 * M / M_max.  Positive (sagging) bars extend rightward with '>' chars,
 * negative (hogging) bars extend leftward with '<' chars.
 *
 * WHY two colors: sign of M determines whether the top or bottom fibre
 * is in tension.  Sagging = tension at bottom (green — common/OK).
 * Hogging = tension at top (magenta — less obvious, more dangerous in
 * concrete which is weak in tension at top).
 */
static void render_moment_panel(const Beam *b, int cols, int rows)
{
    int px   = cols - PANEL_W;
    int half = PANEL_W / 2 - 1;
    int zero = px + half;           /* column of the zero-moment axis */
    if (rows - 4 < 3 || px < 1) return;

    /* Panel separator */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for (int r = 1; r < rows - 1; r++) mvaddch(r, px - 1, '|');
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    attron(COLOR_PAIR(CP_HDR) | A_BOLD);
    mvprintw(1, px, " MOMENT  +/- ");
    attroff(COLOR_PAIR(CP_HDR) | A_BOLD);

    /* Zero-moment vertical axis */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for (int r = 2; r < rows - 2; r++) mvaddch(r, zero, '|');
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    if (b->max_moment < 1e-9f) return;

    for (int r = 2; r < rows - 2; r++) {
        /* Map screen row → beam node (linear interpolation) */
        int   ni  = (r - 2) * (N_NODES - 1) / (rows - 4);
        if (ni < 0) ni = 0;
        if (ni >= N_NODES) ni = N_NODES - 1;

        float mn  = b->M[ni] / b->max_moment;   /* -1..1 */
        int   len = (int)(fabsf(mn) * (float)(half - 1) + 0.5f);
        if (len > half - 1) len = half - 1;
        if (len == 0) continue;

        int  cp  = (mn >= 0.0f) ? CP_MOM_P : CP_MOM_N;
        char bch = (mn >= 0.0f) ? '>' : '<';

        attron(COLOR_PAIR(cp));
        int dir = (mn >= 0.0f) ? 1 : -1;
        for (int k = 1; k <= len; k++) {
            int bc2 = zero + dir * k;
            if (bc2 >= px && bc2 < cols) mvaddch(r, bc2, (chtype)bch);
        }
        attroff(COLOR_PAIR(cp));
    }

    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(rows - 2, px, " sag+ hog- ");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/*
 * render_overlay — HUD rows at top and bottom.
 *
 * Row 0:  BC name | load name | [DYNAMIC badge] | fps
 * Row 1:  curvature color legend
 * Row -2: physics values (max deflection, max moment, load, exaggeration)
 * Row -1: key reference
 */
static void render_overlay(const Beam *b, int cols, int rows)
{
    /* Top row — mode names */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, " BC: %-24s  Load: %-26s",
             bc_names[b->bc], ld_names[b->load]);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Curvature legend */
    int lx = 1;
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(1, lx, "Stress: ");  lx += 8;
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    attron(COLOR_PAIR(CP_LO));  mvprintw(1, lx, "Low "); lx += 4; attroff(COLOR_PAIR(CP_LO));
    attron(COLOR_PAIR(CP_MED)); mvprintw(1, lx, "Med "); lx += 4; attroff(COLOR_PAIR(CP_MED));
    attron(COLOR_PAIR(CP_HI));  mvprintw(1, lx, "High");           attroff(COLOR_PAIR(CP_HI));

    if (b->dyn.active) {
        attron(COLOR_PAIR(CP_HI) | A_BOLD);
        mvprintw(1, cols - PANEL_W - 14, " VIBRATING ");
        attroff(COLOR_PAIR(CP_HI) | A_BOLD);
    }

    /* Physics values */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 2, 0,
        " Deflect: %7.5f   Moment: %7.5f   P=%.2f   Exag:%.1fx%s",
        b->max_deflection, b->max_moment,
        b->P * b->load_anim, b->exag,
        b->paused ? "   [PAUSED]" : "           ");
    attroff(COLOR_PAIR(CP_HUD));

    /* Key reference */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(rows - 1, 0,
        " [b]BC  [l]load  [+/-]P  [e/E]exag  [d]vibrate  [r]reset  [spc]pause  [q]quit");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

typedef struct { Beam beam; } Scene;

static void scene_init(Scene *s)
{
    init_beam(&s->beam);
    solve_beam(&s->beam);
}

static void scene_tick(Scene *s, float dt)  { beam_tick(&s->beam, dt); }

static void scene_draw(const Scene *s, int cols, int rows)
{
    erase();
    render_beam(&s->beam, cols, rows);
    render_moment_panel(&s->beam, cols, rows);
    render_overlay(&s->beam, cols, rows);
}

/* ===================================================================== */
/* §9  screen                                                             */
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

static void screen_free(Screen *s)   { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int s)   { (void)s; g_app.running    = 0; }
static void on_resize_signal(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup(void)           { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    Beam *b = &app->scene.beam;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        b->paused = !b->paused;
        break;

    case 'b': case 'B':
        b->dyn.active = false;
        b->bc = (BCType)((b->bc + 1) % BC_COUNT);
        b->load_anim = 0.0f;
        solve_beam(b);
        break;

    case 'l': case 'L':
        b->dyn.active = false;
        b->load = (LDType)((b->load + 1) % LD_COUNT);
        b->load_anim = 0.0f;
        solve_beam(b);
        break;

    case 'd': case 'D':
        if (b->dyn.active) {
            /* Toggle back to static */
            b->dyn.active = false;
            b->load_anim  = 1.0f;
            solve_beam(b);
        } else {
            dyn_activate(b);
        }
        break;

    case '+': case '=':
        b->P = (b->P * 1.25f > 100.0f) ? 100.0f : b->P * 1.25f;
        solve_beam(b);
        break;

    case '-':
        b->P = (b->P / 1.25f < 0.05f) ? 0.05f : b->P / 1.25f;
        solve_beam(b);
        break;

    case 'e':
        b->exag = (b->exag * 1.5f > 20.0f) ? 20.0f : b->exag * 1.5f;
        break;

    case 'E':
        b->exag = (b->exag / 1.5f < 0.5f) ? 0.5f : b->exag / 1.5f;
        break;

    case 'r': case 'R':
        scene_init(&app->scene);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t tick_ns     = NS_PER_SEC / 30;
    float   dt_sec      = 1.0f / 30.0f;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            screen_resize(&app->screen);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* Fixed-timestep accumulator */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;  /* spiral-of-death guard */

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* fps rolling average (update every 0.5 s) */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Frame cap — sleep before rendering to bound CPU usage */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        scene_draw(&app->scene, app->screen.cols, app->screen.rows);

        attron(COLOR_PAIR(CP_DIM) | A_DIM);
        mvprintw(0, app->screen.cols - PANEL_W - 12, "%.1f fps", fps_display);
        attroff(COLOR_PAIR(CP_DIM) | A_DIM);

        wnoutrefresh(stdscr);
        doupdate();

        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
