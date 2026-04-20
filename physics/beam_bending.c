/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * beam_bending.c — Euler-Bernoulli Beam Bending Simulator
 *
 * Physics:
 *   EI·w'''' = q(x)      beam equation
 *   κ = w'' = M/(EI)     curvature proportional to bending moment
 *   Solved analytically for 3 BC types × 3 load types (9 combinations).
 *
 * Visualization:
 *   Main panel  — deflected beam with curvature-shaded body + load symbols
 *   Side panel  — bending moment diagram (right 22 cols, horizontal bars)
 *   Overlay     — max deflection, load magnitude, BC type, moment legend
 *
 * Keys:
 *   q / ESC    quit            space   pause / resume
 *   b          cycle BC        l       cycle load type
 *   + / -      load magnitude  e / E   deflection exaggeration
 *   r          reset to defaults
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/beam_bending.c -o beam_bending -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *  §1  config      §2  clock       §3  color
 *  §4  beam        §5  solver      §6  render
 *  §7  scene       §8  screen      §9  app
 * ─────────────────────────────────────────────────────────────────────
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Curvature relation:
 *   κ(x) = d²w/dx² = M(x)/(EI)
 *   High |κ| → high bending stress → dense shading ('*','#','@') and red.
 *   Low  |κ| → green, sparse chars (' ','.','-').
 *   Shading palette: ' ' · '.' · '-' · '~' · '=' · '*' · '#' · '@'
 *   maps linearly from |κ|=0 to |κ|=|κ|_max.
 *
 * Boundary constraints:
 *   Simply Supported  w(0)=0, w(L)=0, M(0)=0, M(L)=0
 *     Both ends pinned; beam free to rotate at supports.
 *     Moment diagram: triangular (point load) or parabolic (UDL).
 *   Cantilever        w(0)=0, w'(0)=0, M(L)=0, V(L)=0
 *     Left end fully fixed; maximum moment at wall, zero at free tip.
 *     Tip deflection δ = PL³/(3EI) — largest of the three BC types.
 *   Fixed-Fixed       w(0)=0, w'(0)=0, w(L)=0, w'(L)=0
 *     Both ends clamped; hogging moments develop at supports.
 *     Mid-span deflection ≈ 5× less than simply supported — much stiffer.
 *
 * Load response behavior:
 *   Center point load  symmetric V-shaped moment, cubic deflection.
 *   Uniform dist. load parabolic moment, 4th-order deflection curve.
 *   Offset point load  asymmetric; larger reaction at near support;
 *                      moment peak closer to applied load position.
 * ─────────────────────────────────────────────────────────────────────── */

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

#define N_NODES       200    /* discretisation points along beam          */
#define BEAM_HEIGHT     3    /* rows of beam cross-section (odd)          */
#define PANEL_W        22    /* right-side moment diagram panel width      */
#define BEAM_X_MARGIN   4    /* cols of padding on each beam end           */
#define LOAD_ARROW_H    4    /* rows of load arrow above beam              */
#define TARGET_FPS     30    /* render cap                                 */
#define RAMP_SPEED    0.7f   /* load animation: full load reached in ~1.4s */
#define N_MODES        4     /* modal superposition: first 4 eigenmodes    */
#define MODAL_DAMP    0.025f /* modal damping ratio ζ (2.5% critical)      */

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL

/* Color pair IDs */
#define CP_LO     1   /* low curvature  — green              */
#define CP_MED    2   /* mid curvature  — yellow             */
#define CP_HI     3   /* high curvature — red                */
#define CP_MOM_P  4   /* positive moment bar — green         */
#define CP_MOM_N  5   /* negative moment bar — magenta       */
#define CP_LOAD   6   /* load arrows — bold gold             */
#define CP_SUPP   7   /* support symbols — white             */
#define CP_HUD    8   /* overlay text — cyan                 */
#define CP_DIM    9   /* dim reference lines — grey          */
#define CP_HDR   10   /* panel header — orange               */

typedef enum { BC_SS=0, BC_CANT, BC_FF, BC_COUNT } BCType;
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
        init_pair(CP_LO,    46, COLOR_BLACK);   /* bright green  */
        init_pair(CP_MED,  226, COLOR_BLACK);   /* yellow        */
        init_pair(CP_HI,   196, COLOR_BLACK);   /* red           */
        init_pair(CP_MOM_P, 82, COLOR_BLACK);   /* lime green    */
        init_pair(CP_MOM_N,201, COLOR_BLACK);   /* magenta       */
        init_pair(CP_LOAD, 220, COLOR_BLACK);   /* gold          */
        init_pair(CP_SUPP, 255, COLOR_BLACK);   /* bright white  */
        init_pair(CP_HUD,   51, COLOR_BLACK);   /* cyan          */
        init_pair(CP_DIM,  240, COLOR_BLACK);   /* dark grey     */
        init_pair(CP_HDR,  214, COLOR_BLACK);   /* orange        */
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
 * DynBeam — modal superposition state for free-vibration mode.
 *
 * Mode shapes φₙ(x) and natural frequencies ωₙ are precomputed from the
 * beam BC type using tabulated eigenvalues:
 *   SS:   βₙ = nπ/L,  cos/cosh characteristic eq. not needed (exact)
 *   Cant: cos(λ)cosh(λ) + 1 = 0  →  λ = [1.875, 4.694, 7.855, 10.996]
 *   FF:   cos(λ)cosh(λ) = 1      →  λ = [4.730, 7.853, 10.996, 14.137]
 *
 * Each modal equation:  q̈ₙ + 2ζωₙq̇ₙ + ωₙ²qₙ = Fₙ
 * Stepped exactly per frame using the damped oscillator transition matrix
 * — no stability limit on dt.
 *
 * ref_max_w: static max deflection captured at activation; used as a fixed
 * display scale so the beam doesn't jump during oscillation.
 */
typedef struct {
    float phi[N_MODES][N_NODES]; /* mode shapes at all beam nodes       */
    float omega[N_MODES];        /* natural frequencies ωₙ (rad/s)      */
    float modal_mass[N_MODES];   /* Mₙ = ρA ∫ φₙ² dx                   */
    float q[N_MODES];            /* modal coordinates                   */
    float qdot[N_MODES];         /* modal velocities                    */
    float Fn[N_MODES];           /* modal forces (constant while loaded) */
    float ref_max_w;             /* fixed visual scale reference         */
    bool  active;
} DynBeam;

/*
 * Beam — complete state for the simulator.
 *
 * w[i]   deflection at node i (positive = downward, in normalised units)
 * M[i]   bending moment at node i (positive = sagging)
 *
 * After solve_beam(), max_deflection and max_moment hold the absolute
 * maxima used for normalising the display.
 *
 * load_anim ∈ [0,1]: ramp applied during init to animate load application.
 * exag      : visual multiplier — max deflection occupies exag × base rows.
 */
typedef struct {
    float w[N_NODES];
    float M[N_NODES];
    float x[N_NODES];        /* normalised positions [0, 1] */

    float L;                 /* beam length (always 1.0)    */
    float EI;                /* flexural rigidity (1.0)     */
    float P;                 /* load magnitude              */
    float exag;              /* deflection exaggeration     */
    float load_anim;         /* ramp factor [0, 1]          */
    bool  paused;

    BCType bc;
    LDType load;

    float max_deflection;
    float max_moment;

    DynBeam dyn;
} Beam;

/* apply_load — update load parameters; solve_beam() does the computation */
static void apply_load(Beam *b, BCType bc, LDType ld, float P)
{
    b->bc    = bc;
    b->load  = ld;
    b->P     = P;
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
/* §5  solver — analytical Euler-Bernoulli solutions                     */
/* ===================================================================== */

/*
 * solve_beam() — fill w[] and M[] using closed-form solutions.
 *
 * All coordinates are normalised: L=1, EI=1.  Actual load magnitude
 * is P * load_anim so the on-screen deflection ramps in from zero.
 *
 * Sign convention: downward deflection positive, sagging moment positive.
 *
 * Reference formulas (Roark's Formulas for Stress and Strain, 8th ed.):
 *   SS + point load at a:  w = Pb²x(L²-b²-x²)/(6EIL) for x≤a
 *   SS + UDL q:            w = qx(L³-2Lx²+x³)/(24EI)
 *   Cantilever + tip load: w = Px²(3L-x)/(6EI)
 *   Cantilever + UDL:      w = qx²(6L²-4Lx+x²)/(24EI)
 *   Fixed-fixed + center:  w = Px²(3L-4x)/(48EI) for x≤L/2
 *   Fixed-fixed + UDL:     w = qx²(L-x)²/(24EI)
 */
static void solve_beam(Beam *b)
{
    float L  = b->L;
    float EI = b->EI;
    float P  = b->P * b->load_anim;   /* animated load */

    float max_w = 0.0f, max_M = 0.0f;

    for (int i = 0; i < N_NODES; i++) {
        float x  = b->x[i] * L;
        float w  = 0.0f;
        float M  = 0.0f;

        switch (b->bc) {

        /* ── Simply Supported ─────────────────────────────────────── */
        case BC_SS:
            switch (b->load) {

            case LD_CENTER: {
                float a = 0.5f * L, bv = 0.5f * L;
                if (x <= a) {
                    M = P * bv * x / L;
                    w = P * bv * x * (L*L - bv*bv - x*x) / (6.0f*EI*L);
                } else {
                    float xs = L - x;
                    M = P * a * xs / L;
                    w = P * a  * xs * (L*L - a*a - xs*xs) / (6.0f*EI*L);
                }
                break;
            }

            case LD_UDL: {
                float q = P;
                M = q * x * (L - x) / 2.0f;
                w = q * x * (L*L*L - 2.0f*L*x*x + x*x*x) / (24.0f*EI);
                break;
            }

            case LD_OFFSET: {
                float a = 0.75f * L, bv = 0.25f * L;
                if (x <= a) {
                    M = P * bv * x / L;
                    w = P * bv * x * (L*L - bv*bv - x*x) / (6.0f*EI*L);
                } else {
                    float xs = L - x;
                    M = P * a  * xs / L;
                    w = P * a  * xs * (L*L - a*a - xs*xs) / (6.0f*EI*L);
                }
                break;
            }

            default: break;
            }
            break;

        /* ── Cantilever (fixed left, free right) ──────────────────── */
        case BC_CANT:
            switch (b->load) {

            case LD_CENTER: {   /* point load at free tip */
                M = P * (L - x);
                w = P * x*x * (3.0f*L - x) / (6.0f*EI);
                break;
            }

            case LD_UDL: {
                float q = P;
                M = q * (L - x) * (L - x) / 2.0f;
                w = q * x*x * (6.0f*L*L - 4.0f*L*x + x*x) / (24.0f*EI);
                break;
            }

            case LD_OFFSET: {   /* point load at mid-span */
                float a = 0.5f * L;
                if (x <= a) {
                    M = P * (a - x);
                    w = P * x*x * (3.0f*a - x) / (6.0f*EI);
                } else {
                    M = 0.0f;
                    w = P * a*a * (3.0f*x - a) / (6.0f*EI);
                }
                break;
            }

            default: break;
            }
            break;

        /* ── Fixed-Fixed ──────────────────────────────────────────── */
        case BC_FF:
            switch (b->load) {

            case LD_CENTER: {
                /* Fixed end moment = PL/8; mid-span moment = PL/8 */
                if (x <= 0.5f*L) {
                    M = P*x/2.0f - P*L/8.0f;
                    w = P * x*x * (3.0f*L - 4.0f*x) / (48.0f*EI);
                } else {
                    float xs = L - x;
                    M = P*xs/2.0f - P*L/8.0f;
                    w = P * xs*xs * (3.0f*L - 4.0f*xs) / (48.0f*EI);
                }
                break;
            }

            case LD_UDL: {
                float q = P;
                M = q*x*(L-x)/2.0f - q*L*L/12.0f;
                w = q * x*x * (L-x)*(L-x) / (24.0f*EI);
                break;
            }

            case LD_OFFSET: {
                /* load at a=3L/4 */
                float a  = 0.75f * L;
                float bv = L - a;
                float MA = P * a * bv*bv / (L*L);
                float RA = P * bv*bv * (3.0f*a + bv) / (L*L*L);
                M = -MA + RA*x - (x > a ? P*(x - a) : 0.0f);
                if (x <= a)
                    w = P * bv*bv * x*x * (3.0f*a*L - (3.0f*a+bv)*x)
                        / (6.0f*EI*L*L*L);
                else {
                    float xs = L - x;
                    w = P * a*a * xs*xs * (3.0f*bv*L - (3.0f*bv+a)*xs)
                        / (6.0f*EI*L*L*L);
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

/* ── dynamic modal superposition ─────────────────────────────────────── */

/*
 * dyn_setup — precompute mode shapes and natural frequencies for current BC.
 *
 * SS (exact):
 *   φₙ(x) = sin(nπx/L),  ωₙ = (nπ/L)² (EI=ρA=1)
 *
 * Cantilever and Fixed-Fixed use cosh/sinh/cos/sin combinations with
 * tabulated eigenvalues; computed in double to avoid cancellation error
 * in the large-argument cosh terms of higher modes.
 */
static void dyn_setup(Beam *b)
{
    DynBeam *d  = &b->dyn;
    double   L  = (double)b->L;
    double   dx = L / (double)(N_NODES - 1);

    if (b->bc == BC_SS) {
        for (int n = 0; n < N_MODES; n++) {
            double beta  = (double)(n + 1) * 3.14159265358979 / L;
            d->omega[n]  = (float)(beta * beta);
            for (int i = 0; i < N_NODES; i++)
                d->phi[n][i] = (float)sin(beta * b->x[i] * L);
        }
    } else if (b->bc == BC_CANT) {
        static const double lam[N_MODES] = {1.87510, 4.69409, 7.85476, 10.99554};
        for (int n = 0; n < N_MODES; n++) {
            double bn  = lam[n] / L;
            double sig = (cos(lam[n]) + cosh(lam[n]))
                       / (sin(lam[n]) + sinh(lam[n]));
            d->omega[n] = (float)(bn * bn);
            for (int i = 0; i < N_NODES; i++) {
                double x = b->x[i] * L;
                d->phi[n][i] = (float)(cosh(bn*x) - cos(bn*x)
                                      - sig*(sinh(bn*x) - sin(bn*x)));
            }
        }
    } else { /* BC_FF — symmetric modes only */
        static const double lam[N_MODES] = {4.73004, 7.85321, 10.99561, 14.13717};
        for (int n = 0; n < N_MODES; n++) {
            double bn  = lam[n] / L;
            double den = sin(lam[n]) - sinh(lam[n]);
            double sig = (fabs(den) > 1e-10)
                       ? (cos(lam[n]) - cosh(lam[n])) / den : -1.0;
            d->omega[n] = (float)(bn * bn);
            for (int i = 0; i < N_NODES; i++) {
                double x = b->x[i] * L;
                d->phi[n][i] = (float)(cosh(bn*x) - cos(bn*x)
                                      - sig*(sinh(bn*x) - sin(bn*x)));
            }
        }
    }

    /* Modal masses: Mₙ = ρA ∫ φₙ² dx  (ρA=1, trapezoidal rule) */
    for (int n = 0; n < N_MODES; n++) {
        double mm = 0.0;
        for (int i = 0; i < N_NODES; i++)
            mm += (double)d->phi[n][i] * d->phi[n][i] * dx;
        d->modal_mass[n] = (float)(mm > 1e-9 ? mm : 1.0);
    }
}

/*
 * dyn_load — project external load onto each mode.
 *
 *   Fₙ = (1/Mₙ) ∫ q(x) φₙ(x) dx
 *
 * Point loads are treated as Dirac delta: Fₙ = P·φₙ(x_load)/Mₙ.
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
 * dyn_activate — switch to dynamic mode: full load applied as a step.
 *
 * Starting from rest (q=0, qdot=0) with suddenly applied load gives the
 * classic dynamic magnification: beam overshoots to 2× static deflection
 * on first half-cycle, then rings down to the static solution.
 */
static void dyn_activate(Beam *b)
{
    b->load_anim = 1.0f;
    solve_beam(b);                    /* compute static solution + max_w */

    DynBeam *d   = &b->dyn;
    d->ref_max_w = b->max_deflection; /* fix visual scale for the run    */

    dyn_setup(b);
    dyn_load(b);

    for (int n = 0; n < N_MODES; n++) {
        d->q[n]    = 0.0f;            /* start from undeflected position */
        d->qdot[n] = 0.0f;
    }
    d->active = true;
}

/*
 * dyn_tick_modes — advance each modal oscillator by one timestep.
 *
 * Uses the exact damped-oscillator transition for arbitrary dt:
 *   η = q - q_static,   q_static = Fₙ/ωₙ²
 *   η(t+dt) = e^{-ζωdt} [A cos(ωd·dt) + B sin(ωd·dt)]
 *   where A = η(t),  B = (η̇(t) + ζω·η(t)) / ωd
 *
 * No stability limit — exact stepping regardless of dt or ω.
 */
static void dyn_tick_modes(Beam *b, float dt)
{
    DynBeam *d    = &b->dyn;
    float    zeta = MODAL_DAMP;

    for (int n = 0; n < N_MODES; n++) {
        float omega = d->omega[n];
        float q_s   = (omega > 1e-6f) ? d->Fn[n] / (omega * omega) : 0.0f;
        float wd    = omega * sqrtf(1.0f - zeta * zeta);
        if (wd < 1e-6f) wd = 1e-6f;

        float A  = d->q[n] - q_s;
        float B  = (d->qdot[n] + zeta * omega * A) / wd;
        float e  = expf(-zeta * omega * dt);
        float c  = cosf(wd * dt);
        float s  = sinf(wd * dt);

        d->q[n]    = q_s + e * (A*c + B*s);
        d->qdot[n] = e * ((-zeta*omega*A + wd*B)*c
                        + (-zeta*omega*B - wd*A)*s);
    }

    /* Reconstruct deflection w[i] = Σ qₙ φₙ(i) */
    float max_w = 0.0f;
    for (int i = 0; i < N_NODES; i++) {
        float w = 0.0f;
        for (int n = 0; n < N_MODES; n++)
            w += d->q[n] * d->phi[n][i];
        b->w[i] = w;
        if (fabsf(w) > max_w) max_w = fabsf(w);
    }
    /* Hold display scale fixed; only update if clearly larger (avoids jitter) */
    if (max_w > d->ref_max_w * 1.05f) d->ref_max_w = max_w;
    b->max_deflection = d->ref_max_w;
}

/* ── tick dispatcher ──────────────────────────────────────────────────── */

/* Advance one fixed-step tick: ramp the load, re-solve */
static void beam_tick(Beam *b, float dt)
{
    if (b->paused) return;
    if (b->dyn.active) {
        dyn_tick_modes(b, dt);
    } else {
        if (b->load_anim < 1.0f) {
            b->load_anim += dt * RAMP_SPEED;
            if (b->load_anim > 1.0f) b->load_anim = 1.0f;
        }
        solve_beam(b);
    }
}

/* ===================================================================== */
/* §6  render                                                             */
/* ===================================================================== */

/* Map node index → screen column within [x0, x1] */
static int node_col(int i, int x0, int x1)
{
    return x0 + i * (x1 - x0) / (N_NODES - 1);
}

/* Map deflection to row offset; exag scales the visual amplitude */
static int defl_row(float w, float max_w, int base_rows, float exag)
{
    if (max_w < 1e-9f) return 0;
    int dr = (int)(w / max_w * (float)base_rows * exag + 0.5f);
    int cap = base_rows * 3;
    if (dr >  cap) dr =  cap;
    if (dr < -cap) dr = -cap;
    return dr;
}

/*
 * render_beam — draw deflected shape with curvature shading and supports.
 *
 * Curvature shading:
 *   Each column of the beam cross-section is filled with a character
 *   chosen by mapping |M[i]| / max_moment → [0,7] into the palette:
 *     0→' ', 1→'.', 2→'-', 3→'~', 4→'=', 5→'*', 6→'#', 7→'@'
 *   Color cycles green→yellow→red as curvature increases.
 */
static void render_beam(const Beam *b, int cols, int rows)
{
    int x0 = BEAM_X_MARGIN;
    int x1 = cols - PANEL_W - BEAM_X_MARGIN - 1;
    if (x1 <= x0 + 10) return;

    int neutral_row  = rows / 2;
    int base_rows    = rows / 6;
    if (base_rows < 3) base_rows = 3;
    int half_h = BEAM_HEIGHT / 2;

    /* Neutral axis reference line */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for (int c = x0; c <= x1; c++) mvaddch(neutral_row, c, '-');
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    /* Curvature character palette (density increases with curvature) */
    static const char pal[] = " .-~=*#@";
    int npal = (int)(sizeof pal - 1);

    /* Beam body */
    for (int i = 0; i < N_NODES; i++) {
        int c = node_col(i, x0, x1);
        if (c < 0 || c >= cols) continue;

        float kn = (b->max_moment > 1e-9f)
                   ? fabsf(b->M[i]) / b->max_moment
                   : 0.0f;

        int pi = (int)(kn * (float)(npal - 1) + 0.5f);
        if (pi < 0) pi = 0;
        if (pi >= npal) pi = npal - 1;
        chtype fill = (chtype)pal[pi];

        int cp = (kn < 0.35f) ? CP_LO : (kn < 0.70f) ? CP_MED : CP_HI;

        int dr      = defl_row(b->w[i], b->max_deflection, base_rows, b->exag);
        int mid_row = neutral_row + dr;

        for (int rr = mid_row - half_h; rr <= mid_row + half_h; rr++) {
            if (rr < 2 || rr >= rows - 2) continue;
            chtype ch = (rr == mid_row - half_h || rr == mid_row + half_h)
                        ? '='
                        : fill;
            attr_t attr = COLOR_PAIR(cp) | (rr == mid_row ? A_BOLD : 0);
            attron(attr);
            mvaddch(rr, c, ch);
            attroff(attr);
        }
    }

    /* Load indicators */
    attron(COLOR_PAIR(CP_LOAD) | A_BOLD);
    switch (b->load) {

    case LD_CENTER:
    case LD_OFFSET: {
        float xfrac = (b->load == LD_CENTER) ? 0.5f : 0.75f;
        int lc = x0 + (int)(xfrac * (float)(x1 - x0) + 0.5f);
        int ni = (int)(xfrac * (float)(N_NODES - 1));
        int dr = defl_row(b->w[ni], b->max_deflection, base_rows, b->exag);
        int top = neutral_row + dr - half_h;

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
            if (top - 1 >= 2 && top - 1 < rows - 2)
                mvaddch(top - 1, c, 'v');
            if (top - 2 >= 2 && top - 2 < rows - 2)
                mvaddch(top - 2, c, '|');
        }
        int mc = (x0 + x1) / 2;
        if (neutral_row - half_h - 3 >= 2)
            mvprintw(neutral_row - half_h - 3, mc - 3, "q=%.2f", b->P * b->load_anim);
        break;
    }

    default: break;
    }
    attroff(COLOR_PAIR(CP_LOAD) | A_BOLD);

    /* Support symbols */
    attron(COLOR_PAIR(CP_SUPP) | A_BOLD);
    switch (b->bc) {

    case BC_SS: {
        /* Triangular pin supports at both ends */
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
        /* Wall on left */
        for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1; rr++) {
            if (rr < 1 || rr >= rows - 2) continue;
            if (x0 - 1 >= 0) mvaddch(rr, x0 - 1, '#');
            if (x0 - 2 >= 0) mvaddch(rr, x0 - 2, '#');
        }
        int fr = neutral_row;
        if (fr >= 1 && fr < rows - 2 && x1 + 1 < cols)
            mvprintw(fr, x1 + 1, "free");
        break;
    }

    case BC_FF: {
        /* Walls on both ends */
        for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1; rr++) {
            if (rr < 1 || rr >= rows - 2) continue;
            if (x0 - 1 >= 0) mvaddch(rr, x0 - 1, '#');
            if (x1 + 1 < cols) mvaddch(rr, x1 + 1, '#');
        }
        break;
    }

    default: break;
    }
    attroff(COLOR_PAIR(CP_SUPP) | A_BOLD);
}

/*
 * render_moment_panel — right-side bending moment diagram.
 *
 * Each row maps to a beam node; horizontal bars show M[node]/max_moment.
 * Positive (sagging) bars extend right, negative (hogging) extend left.
 * The panel is separated from the beam area by a dim vertical line.
 */
static void render_moment_panel(const Beam *b, int cols, int rows)
{
    int px    = cols - PANEL_W;
    int half  = PANEL_W / 2 - 1;
    int zero  = px + half;
    int pr    = rows - 4;
    if (pr < 3 || px < 1) return;

    /* Separator */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for (int r = 1; r < rows - 1; r++) mvaddch(r, px - 1, '|');
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    /* Header */
    attron(COLOR_PAIR(CP_HDR) | A_BOLD);
    mvprintw(1, px, " MOMENT  +/- ");
    attroff(COLOR_PAIR(CP_HDR) | A_BOLD);

    /* Zero axis */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for (int r = 2; r < rows - 2; r++) mvaddch(r, zero, '|');
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    if (b->max_moment < 1e-9f) return;

    for (int r = 2; r < rows - 2; r++) {
        int ni = (r - 2) * (N_NODES - 1) / (rows - 4);
        if (ni < 0) ni = 0;
        if (ni >= N_NODES) ni = N_NODES - 1;

        float mn = b->M[ni] / b->max_moment;   /* -1..1 */
        int blen = (int)(fabsf(mn) * (float)(half - 1) + 0.5f);
        if (blen > half - 1) blen = half - 1;
        if (blen == 0) continue;

        int cp   = (mn >= 0.0f) ? CP_MOM_P : CP_MOM_N;
        char bch = (mn >= 0.0f) ? '>' : '<';

        attron(COLOR_PAIR(cp));
        if (mn >= 0.0f) {
            for (int k = 1; k <= blen; k++) {
                int bc2 = zero + k;
                if (bc2 < cols) mvaddch(r, bc2, (chtype)bch);
            }
        } else {
            for (int k = 1; k <= blen; k++) {
                int bc2 = zero - k;
                if (bc2 >= px) mvaddch(r, bc2, (chtype)bch);
            }
        }
        attroff(COLOR_PAIR(cp));
    }

    /* Legend labels */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(rows - 2, px, " sag+ hog- ");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/*
 * render_overlay — HUD text: BC type, load, max deflection, key hints,
 *                  and curvature color legend.
 */
static void render_overlay(const Beam *b, int cols, int rows)
{
    (void)cols;

    /* Row 0: BC and load type */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, " BC: %-24s  Load: %-26s",
             bc_names[b->bc], ld_names[b->load]);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Row 1: curvature color legend */
    int lx = 1;
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(1, lx, "Curvature: ");
    lx += 11;
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    attron(COLOR_PAIR(CP_LO));
    mvprintw(1, lx, "Low ");  lx += 4;
    attroff(COLOR_PAIR(CP_LO));
    attron(COLOR_PAIR(CP_MED));
    mvprintw(1, lx, "Med ");  lx += 4;
    attroff(COLOR_PAIR(CP_MED));
    attron(COLOR_PAIR(CP_HI));
    mvprintw(1, lx, "High");
    attroff(COLOR_PAIR(CP_HI));

    /* Dynamic mode badge */
    if (b->dyn.active) {
        attron(COLOR_PAIR(CP_HI) | A_BOLD);
        mvprintw(1, cols - PANEL_W - 14, " DYNAMIC ");
        attroff(COLOR_PAIR(CP_HI) | A_BOLD);
    }

    /* Second-to-last row: physics values */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 2, 0,
        " Max deflect: %7.5f   Max moment: %7.5f   P=%.2f   Exag: %.1fx%s",
        b->max_deflection, b->max_moment,
        b->P * b->load_anim, b->exag,
        b->paused ? "   [PAUSED]" : "           ");
    attroff(COLOR_PAIR(CP_HUD));

    /* Last row: key hints */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(rows - 1, 0,
        " [b]BC  [l]load  [+/-]P  [e/E]exag  [d]dynamic  [r]reset  [spc]pause  [q]quit");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct { Beam beam; } Scene;

static void scene_init(Scene *s)
{
    init_beam(&s->beam);
    solve_beam(&s->beam);
}

static void scene_tick(Scene *s, float dt)
{
    beam_tick(&s->beam, dt);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    erase();
    render_beam(&s->beam, cols, rows);
    render_moment_panel(&s->beam, cols, rows);
    render_overlay(&s->beam, cols, rows);
}

/* ===================================================================== */
/* §8  screen                                                             */
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

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ===================================================================== */
/* §9  app                                                                */
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
            b->dyn.active = false;
            b->load_anim  = 1.0f;
            solve_beam(b);
        } else {
            dyn_activate(b);
        }
        break;

    case '+': case '=':
        b->P *= 1.25f;
        if (b->P > 100.0f) b->P = 100.0f;
        solve_beam(b);
        break;

    case '-':
        b->P /= 1.25f;
        if (b->P < 0.05f) b->P = 0.05f;
        solve_beam(b);
        break;

    case 'e':
        b->exag *= 1.5f;
        if (b->exag > 20.0f) b->exag = 20.0f;
        break;

    case 'E':
        b->exag /= 1.5f;
        if (b->exag < 0.5f) b->exag = 0.5f;
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

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

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
