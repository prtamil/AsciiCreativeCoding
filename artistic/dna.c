/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dna.c — 10 DNA/RNA structure visualisations in 2D ASCII art
 *
 * DNA TYPES (n/p to cycle):
 *   0  B-DNA Double Helix   — classic Watson-Crick, right-handed
 *   1  A-DNA Double Helix   — compressed, wide-grooved form
 *   2  Z-DNA                — left-handed zigzag
 *   3  Triple Helix         — Hoogsteen third strand
 *   4  G-Quadruplex         — four-strand G4 telomere structure
 *   5  RNA Hairpin          — stem-loop fold
 *   6  Replication Fork     — Y-shape parent→daughter unwinding
 *   7  Cruciform            — cross-shaped inverted-repeat
 *   8  Plasmid              — closed circular bacterial DNA
 *   9  DNA Ladder           — educational unrolled base-pair ladder
 *
 * COLOR THEMES (t):
 *   0 BioLab  1 Neon  2 Ocean  3 Fire  4 Cosmic  5 Mono
 *   Each theme supplies 5 complementary colors:
 *   strand1, strand2, strand3, bond, label
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   n         next DNA type
 *   p         previous DNA type
 *   t         next color theme
 *   ] / [     fps up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/dna.c -o dna -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 draw  §5 scene  §6 screen  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Helix rendering:
 *   Backbone positions are parametric sinusoids scrolling in t:
 *     x1(row) = cx + A·cos(2π·row/P + t)
 *     x2(row) = cx − A·cos(2π·row/P + t)
 *   Depth  z  = sin(phase) determines front/back order.
 *   Front strand is drawn bold, back strand dim — gives 3-D illusion.
 *
 * CELL_AR = 0.5 aspect correction:
 *   Terminal cells are ~2× taller than wide.  Horizontal helix arm
 *   uses pitch_h = pitch_v / CELL_AR so both arms look the same scale.
 *   Plasmid uses ry = rx × CELL_AR so the ellipse appears circular.
 *
 * Theme system:
 *   Each of 6 themes supplies 5 xterm-256 indices (s1, s2, s3, bond, label).
 *   color_apply_theme() registers 5 ncurses pairs once at theme change.
 *   Falls back to 8 basic colors on terminals with COLORS < 256.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

#define CELL_AR     0.5f   /* col_width / row_height terminal aspect ratio */
#define TAU         6.28318530f

enum {
    N_DNA_TYPES   = 10,
    N_THEMES      = 6,
    THEME_COLS    = 5,     /* s1, s2, s3, bond, label                     */
    HUD_ROWS      = 2,     /* top title bar + bottom key-hint bar          */

    SIM_FPS_MIN   =  5,
    SIM_FPS_DEF   = 30,
    SIM_FPS_MAX   = 60,
    SIM_FPS_STEP  =  5,

    NS_PER_SEC_I  = 1,     /* placeholder for enum; real value is #define  */
};

#define NS_PER_SEC  1000000000LL

static const char *dna_names[N_DNA_TYPES] = {
    "B-DNA Double Helix",
    "A-DNA Double Helix",
    "Z-DNA (Left-handed)",
    "Triple Helix",
    "G-Quadruplex",
    "RNA Hairpin",
    "Replication Fork",
    "Cruciform",
    "Plasmid (Circular DNA)",
    "DNA Ladder",
};

static const char *theme_names[N_THEMES] = {
    "BioLab", "Neon", "Ocean", "Fire", "Cosmic", "Mono",
};

/* xterm-256 color indices: s1, s2, s3, bond, label */
static const int theme_256[N_THEMES][THEME_COLS] = {
    {  46, 118,  51, 250, 243 },   /* BioLab: green, lime, cyan, lgray, dgray */
    { 201, 213, 226, 255, 245 },   /* Neon:   magenta, pink, yellow, white, gray */
    {  27,  51, 159, 195, 240 },   /* Ocean:  blue, cyan, sky, pale, dim       */
    { 196, 208, 226, 255, 243 },   /* Fire:   red, orange, yellow, white, gray */
    {  93, 129, 171, 219, 240 },   /* Cosmic: purple, violet, lavender, pink   */
    { 255, 231, 195, 244, 236 },   /* Mono:   white, pale, cream, mid, dark    */
};

static const int theme_8[N_THEMES][THEME_COLS] = {
    { COLOR_GREEN,   COLOR_CYAN,    COLOR_GREEN,   COLOR_WHITE, COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_YELLOW,  COLOR_RED,     COLOR_WHITE, COLOR_WHITE },
    { COLOR_BLUE,    COLOR_CYAN,    COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE },
    { COLOR_RED,     COLOR_YELLOW,  COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_BLUE,    COLOR_CYAN,    COLOR_WHITE, COLOR_WHITE },
    { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

enum { CP_S1 = 1, CP_S2, CP_S3, CP_BD, CP_LB, CP_HUD };

static void color_apply_theme(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < THEME_COLS; i++) {
        int fg = (COLORS >= 256) ? theme_256[theme][i] : theme_8[theme][i];
        init_pair(CP_S1 + i, fg, -1);
    }
    init_pair(CP_HUD, (COLORS >= 256) ? 240 : COLOR_WHITE, -1);
}

/* ===================================================================== */
/* §4  draw                                                               */
/* ===================================================================== */

/* safe cell write — silently clamps to drawable interior */
static inline void sput(int r, int c, chtype ch, int rows, int cols)
{
    if (r >= 1 && r < rows - 1 && c >= 0 && c < cols)
        mvaddch(r, c, ch);
}

/*
 * helix_draw — shared backbone for B-DNA, A-DNA, Z-DNA
 *   cx, amp, pitch, hand (+1 right / -1 left), t, base_step_div
 */
static void helix_draw(int cx, float amp, float pitch, float hand,
                       float t, int rows, int cols, bool zigzag)
{
    float k = TAU / pitch * hand;
    int   bstep = (int)(pitch / 5.f + 0.5f);
    if (bstep < 2) bstep = 2;
    int draw_rows = rows - HUD_ROWS - 1;

    for (int row = 1; row < draw_rows; row++) {
        float phase = k * (float)row + t;
        float prev  = k * (float)(row - 1) + t;
        int   x1 = (int)roundf((float)cx + amp * cosf(phase));
        int   x2 = (int)roundf((float)cx - amp * cosf(phase));
        float z1 = sinf(phase);
        bool  s1f = (z1 >= 0.f);
        bool  bp  = ((row % bstep) == 0);

        /* base pair connector */
        if (bp) {
            int lo = (x1 < x2 ? x1 : x2) + 1;
            int hi = (x1 > x2 ? x1 : x2) - 1;
            attrset(COLOR_PAIR(CP_BD));
            for (int c = lo; c <= hi; c++)
                sput(row, c, '-', rows, cols);
        }

        /* choose characters */
        chtype ch1, ch2;
        if (zigzag) {
            int px1 = (int)roundf((float)cx + amp * cosf(prev));
            ch1 = (x1 > px1) ? '/' : (x1 < px1 ? '\\' : '|');
            ch2 = (ch1 == '/') ? '\\' : (ch1 == '\\' ? '/' : '|');
        } else {
            ch1 = ch2 = bp ? 'O' : 'o';
        }

        /* back strand dim, front strand bright */
        attrset(COLOR_PAIR(s1f ? CP_S2 : CP_S1) | A_DIM);
        sput(row, s1f ? x2 : x1, s1f ? ch2 : ch1, rows, cols);

        attrset(COLOR_PAIR(s1f ? CP_S1 : CP_S2) | (bp ? A_BOLD : A_NORMAL));
        sput(row, s1f ? x1 : x2, s1f ? ch1 : ch2, rows, cols);
    }
    attrset(A_NORMAL);
}

/* 0: B-DNA — classic right-handed, standard pitch */
static void draw_bdna(int cx, float t, int rows, int cols)
{
    helix_draw(cx, 9.f, 20.f, 1.f, t, rows, cols, false);
}

/* 1: A-DNA — right-handed, wide amplitude, compressed pitch */
static void draw_adna(int cx, float t, int rows, int cols)
{
    helix_draw(cx, 12.f, 13.f, 1.f, t, rows, cols, false);
}

/* 2: Z-DNA — left-handed zigzag */
static void draw_zdna(int cx, float t, int rows, int cols)
{
    helix_draw(cx, 6.f, 24.f, -1.f, t, rows, cols, true);
}

/* 3: Triple Helix — three strands at 0, 2π/3, 4π/3 */
static void draw_triple(int cx, float t, int rows, int cols)
{
    float pitch = 21.f;
    float amp   = 9.f;
    float k     = TAU / pitch;
    int   bstep = (int)(pitch / 5.f + 0.5f);
    int   draw_rows = rows - HUD_ROWS - 1;

    static const float ph3[3] = { 0.f, 2.094395f, 4.188790f };
    static const int   cp3[3] = { CP_S1, CP_S2, CP_S3 };

    for (int row = 1; row < draw_rows; row++) {
        float base_phase = k * (float)row + t;
        bool  bp = ((row % bstep) == 0);

        /* bond between strand 0 and strand 1 */
        if (bp) {
            int xa = (int)roundf((float)cx + amp * cosf(base_phase + ph3[0]));
            int xb = (int)roundf((float)cx + amp * cosf(base_phase + ph3[1]));
            int lo = (xa < xb ? xa : xb) + 1;
            int hi = (xa > xb ? xa : xb) - 1;
            attrset(COLOR_PAIR(CP_BD));
            for (int c = lo; c <= hi; c++)
                sput(row, c, '-', rows, cols);
        }

        /* sort 3 strands by depth (back-to-front) */
        typedef struct { int x; float z; int cp; } Sv;
        Sv sv[3];
        for (int i = 0; i < 3; i++) {
            float ph  = base_phase + ph3[i];
            sv[i].x   = (int)roundf((float)cx + amp * cosf(ph));
            sv[i].z   = sinf(ph);
            sv[i].cp  = cp3[i];
        }
        /* simple insertion sort by z ascending */
        for (int a = 1; a < 3; a++) {
            Sv key = sv[a];
            int b = a - 1;
            while (b >= 0 && sv[b].z > key.z) { sv[b+1] = sv[b]; b--; }
            sv[b+1] = key;
        }

        chtype bc = bp ? 'O' : 'o';
        for (int i = 0; i < 3; i++) {
            attr_t at = (i == 2) ? (bp ? A_BOLD : A_NORMAL) : A_DIM;
            attrset(COLOR_PAIR(sv[i].cp) | at);
            sput(row, sv[i].x, bc, rows, cols);
        }
    }
    attrset(A_NORMAL);
}

/* 4: G-Quadruplex — four parallel strands + periodic G-tetrad squares */
static void draw_gquad(int cx, float t, int rows, int cols)
{
    int draw_rows = rows - HUD_ROWS - 1;
    int sp = (cols > 60) ? 9 : 5;
    int xs[4] = { cx - sp - sp/2, cx - sp/2, cx + sp/2, cx + sp + sp/2 };
    static const int cp4[4] = { CP_S1, CP_S2, CP_S3, CP_S2 };
    int tetrad_step = 7;

    for (int row = 1; row < draw_rows; row++) {
        bool tetrad = ((row % tetrad_step) == 0);
        float pulse_ph = TAU * (float)row / 30.f + t;
        attr_t pulse   = (sinf(pulse_ph) > 0.f) ? A_BOLD : A_NORMAL;

        for (int i = 0; i < 4; i++) {
            attrset(COLOR_PAIR(cp4[i]) | (tetrad ? A_BOLD : pulse));
            sput(row, xs[i], tetrad ? 'G' : '|', rows, cols);
        }

        if (tetrad) {
            /* outer rectangle connecting all 4 strands */
            attrset(COLOR_PAIR(CP_BD) | A_BOLD);
            for (int c = xs[0] + 1; c < xs[3]; c++)
                sput(row, c, '-', rows, cols);
            /* central potassium ion marker */
            int mid = (xs[1] + xs[2]) / 2;
            attrset(COLOR_PAIR(CP_LB) | A_BOLD);
            sput(row, mid, 'K', rows, cols);
        }
    }
    attrset(A_NORMAL);
    (void)t;
}

/* 5: RNA Hairpin — stem-loop structure */
static void draw_hairpin(int cx, float t, int rows, int cols)
{
    int draw_rows = rows - HUD_ROWS - 1;
    int gap       = 6;
    int lx        = cx - gap;
    int rx        = cx + gap;
    int stem_rows = (draw_rows - 6) * 3 / 4;

    static const char nt_l[] = "AUGCAUGCAUGC";
    static const char nt_r[] = "UACGUACGUACG";

    /* 5' / 3' end labels */
    attrset(COLOR_PAIR(CP_LB) | A_BOLD);
    if (lx - 3 >= 0) mvprintw(1, lx - 3, "5'");
    if (rx + 2 < cols) mvprintw(1, rx + 2, "3'");

    /* stem */
    for (int i = 0; i < stem_rows; i++) {
        int row = 2 + i;
        if (row >= draw_rows) break;

        float ph = TAU * (float)i / 10.f + t * 0.4f;
        attr_t pulse = (sinf(ph) > 0.f) ? A_BOLD : A_NORMAL;

        attrset(COLOR_PAIR(CP_S1) | A_DIM);
        sput(row, lx, '|', rows, cols);
        attrset(COLOR_PAIR(CP_S2) | A_DIM);
        sput(row, rx, '|', rows, cols);

        /* base pair every 2 rows */
        if ((i % 2) == 0) {
            int ni = (i / 2) % (int)(sizeof(nt_l) - 1);
            attrset(COLOR_PAIR(CP_S1) | A_BOLD);
            sput(row, lx + 1, (chtype)nt_l[ni], rows, cols);
            attrset(COLOR_PAIR(CP_S2) | A_BOLD);
            sput(row, rx - 1, (chtype)nt_r[ni], rows, cols);
            char bch = (nt_l[ni] == 'G' || nt_l[ni] == 'C') ? '=' : '-';
            attrset(COLOR_PAIR(CP_BD) | pulse);
            for (int c = lx + 2; c < rx - 1; c++)
                sput(row, c, (chtype)bch, rows, cols);
        }
    }

    /* loop: semicircle arc from lx to rx below stem */
    int loop_top = 2 + stem_rows;
    int n_arc    = gap * 8;
    for (int i = 0; i <= n_arc; i++) {
        float a  = (float)M_PI * (float)i / (float)n_arc;
        int   lc = cx - (int)roundf((float)gap * cosf(a));
        int   lr = loop_top + (int)roundf((float)gap * 0.5f * sinf(a));
        float ph = TAU * (float)i / (float)n_arc + t;
        int   cp = (sinf(ph) > 0.f) ? CP_S1 : CP_S2;
        attrset(COLOR_PAIR(cp) | A_BOLD);
        sput(lr, lc, '*', rows, cols);
    }

    /* loop nucleotide labels at arc midpoint */
    {
        static const char loop_seq[] = "ACGU";
        int mid_row = loop_top + gap / 2;
        for (int i = 0; i < 4; i++) {
            int mc = cx - 1 + i;
            attrset(COLOR_PAIR(CP_LB) | A_BOLD);
            sput(mid_row, mc, (chtype)loop_seq[i], rows, cols);
        }
    }
    attrset(A_NORMAL);
}

/* 6: Replication Fork — Y-shape with animated fork point */
static void draw_rfork(int cx, float t, int rows, int cols)
{
    int   draw_rows = rows - HUD_ROWS - 1;
    float fork_frac = 0.35f + 0.15f * sinf(t * 0.25f);
    int   fork_row  = 1 + (int)(fork_frac * (float)(draw_rows - 2));
    float pitch = 18.f;
    float amp   = 8.f;
    float k     = TAU / pitch;

    /* parent double helix above fork */
    for (int row = 1; row < fork_row && row < draw_rows; row++) {
        float phase = k * (float)row + t;
        int   x1 = (int)roundf((float)cx + amp * cosf(phase));
        int   x2 = (int)roundf((float)cx - amp * cosf(phase));
        float z1 = sinf(phase);
        bool  s1f = (z1 >= 0.f);
        bool  bp  = ((row % 4) == 0);

        if (bp) {
            int lo=(x1<x2?x1:x2)+1, hi=(x1>x2?x1:x2)-1;
            attrset(COLOR_PAIR(CP_BD));
            for (int c=lo; c<=hi; c++) sput(row,c,'-',rows,cols);
        }
        attrset(COLOR_PAIR(s1f?CP_S2:CP_S1)|A_DIM);
        sput(row, s1f?x2:x1, bp?'O':'o', rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S1:CP_S2)|(bp?A_BOLD:A_NORMAL));
        sput(row, s1f?x1:x2, bp?'O':'o', rows, cols);
    }

    /* fork point indicator */
    if (fork_row >= 1 && fork_row < draw_rows) {
        attrset(COLOR_PAIR(CP_LB) | A_BOLD);
        sput(fork_row, cx, 'Y', rows, cols);
        sput(fork_row, cx - 2, '<', rows, cols);
        sput(fork_row, cx - 1, '<', rows, cols);
        sput(fork_row, cx + 1, '>', rows, cols);
        sput(fork_row, cx + 2, '>', rows, cols);
    }

    /* two daughter strands diverging below fork */
    for (int row = fork_row + 1; row < draw_rows; row++) {
        float depth = (float)(row - fork_row);
        float div   = 0.55f * depth;

        /* leading strand — left */
        float ph_l = k * (float)row + t;
        int   xl   = (int)roundf((float)cx - div + amp * 0.5f * cosf(ph_l));
        attrset(COLOR_PAIR(CP_S1) | A_NORMAL);
        sput(row, xl, 'o', rows, cols);

        /* lagging strand — right, with Okazaki fragment gaps */
        float ph_r  = k * (float)row + t + (float)M_PI;
        int   xr    = (int)roundf((float)cx + div + amp * 0.5f * cosf(ph_r));
        int   frag  = (int)(depth) % 10;
        attr_t fattr = (frag < 7) ? A_NORMAL : A_DIM;
        chtype fch   = (frag < 7) ? 'o' : '.';
        attrset(COLOR_PAIR(CP_S2) | fattr);
        sput(row, xr, fch, rows, cols);
    }

    /* labels */
    if (fork_row + 3 < draw_rows) {
        attrset(COLOR_PAIR(CP_LB));
        if (cx - 16 >= 0) mvprintw(fork_row + 3, cx - 16, "Leading");
        if (cx + 6 < cols) mvprintw(fork_row + 3, cx + 6,  "Lagging");
    }
    attrset(A_NORMAL);
}

/* 7: Cruciform — vertical + horizontal double helices crossing */
static void draw_cruciform(int cx, float t, int rows, int cols)
{
    int   draw_rows = rows - HUD_ROWS - 1;
    int   cy        = draw_rows / 2;
    float pitch_v   = 18.f;
    float kv        = TAU / pitch_v;
    float amp_v     = 7.f;
    /* horizontal arm: compensate aspect ratio */
    float pitch_h   = pitch_v / CELL_AR;
    float kh        = TAU / pitch_h;
    float amp_h     = amp_v * CELL_AR;

    /* vertical arms — skip 3 rows near the junction */
    for (int row = 1; row < draw_rows; row++) {
        if (abs(row - cy) < 3) continue;
        float phase = kv * (float)row + t;
        int   x1 = (int)roundf((float)cx + amp_v * cosf(phase));
        int   x2 = (int)roundf((float)cx - amp_v * cosf(phase));
        float z1 = sinf(phase);
        bool  s1f = (z1 >= 0.f);
        bool  bp  = ((row % 4) == 0);

        if (bp) {
            int lo=(x1<x2?x1:x2)+1, hi=(x1>x2?x1:x2)-1;
            attrset(COLOR_PAIR(CP_BD));
            for (int c=lo; c<=hi; c++) sput(row,c,'-',rows,cols);
        }
        attrset(COLOR_PAIR(s1f?CP_S2:CP_S1)|A_DIM);
        sput(row, s1f?x2:x1, 'o', rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S1:CP_S2)|(bp?A_BOLD:A_NORMAL));
        sput(row, s1f?x1:x2, 'o', rows, cols);
    }

    /* horizontal arms — skip 6 cols near junction */
    int arm_len = cols / 2 - 2;
    for (int dc = -arm_len; dc <= arm_len; dc++) {
        int col = cx + dc;
        if (col < 1 || col >= cols - 1) continue;
        if (abs(dc) < 6) continue;

        float phase = kh * (float)dc + t + (float)M_PI * 0.5f;
        int   r1 = cy + (int)roundf(amp_h * cosf(phase));
        int   r2 = cy - (int)roundf(amp_h * cosf(phase));
        float z1 = sinf(phase);
        bool  s1f = (z1 >= 0.f);
        bool  bp  = ((abs(dc) % 5) == 0);

        attrset(COLOR_PAIR(s1f?CP_S3:CP_S2)|A_DIM);
        sput(s1f?r2:r1, col, 'o', rows, cols);
        attrset(COLOR_PAIR(s1f?CP_S2:CP_S3)|(bp?A_BOLD:A_NORMAL));
        sput(s1f?r1:r2, col, 'o', rows, cols);
    }

    /* junction cross */
    attrset(COLOR_PAIR(CP_LB) | A_BOLD);
    sput(cy,     cx,     '+', rows, cols);
    sput(cy - 1, cx,     '|', rows, cols);
    sput(cy + 1, cx,     '|', rows, cols);
    sput(cy,     cx - 1, '-', rows, cols);
    sput(cy,     cx + 1, '-', rows, cols);
    attrset(A_NORMAL);
}

/* 8: Plasmid — closed circular DNA with gene regions */
static void draw_plasmid(int cx, float t, int rows, int cols)
{
    int   draw_rows = rows - HUD_ROWS - 1;
    int   cy        = draw_rows / 2;
    float max_r     = (float)((cols < draw_rows * 2 ? cols : draw_rows * 2)) * 0.27f;
    float rx        = max_r;
    float ry        = rx * CELL_AR;

    static const char *gene_names[4] = { "GeneA", "GeneB", "Ori", "GeneC" };
    static const float gene_angles[4] = { 0.78f, 2.36f, 3.93f, 5.50f };
    static const int   gene_cp[4]     = { CP_S1, CP_S2, CP_S3, CP_BD };

    /* outer strand (bold) + inner strand (dim) */
    int n_pts = 360;
    for (int i = 0; i < n_pts; i++) {
        float a = TAU * (float)i / (float)n_pts + t * 0.08f;

        /* which gene region? */
        float norm_a = fmodf(a - t * 0.08f, TAU);
        if (norm_a < 0.f) norm_a += TAU;
        int sector = (int)(norm_a / TAU * 4.f) % 4;

        int r_outer = cy + (int)roundf(ry * sinf(a));
        int c_outer = cx + (int)roundf(rx * cosf(a));
        int r_inner = cy + (int)roundf(ry * 0.82f * sinf(a));
        int c_inner = cx + (int)roundf(rx * 0.82f * cosf(a));

        attrset(COLOR_PAIR(gene_cp[sector]) | A_BOLD);
        sput(r_outer, c_outer, 'o', rows, cols);
        attrset(COLOR_PAIR(gene_cp[sector]) | A_DIM);
        sput(r_inner, c_inner, '.', rows, cols);
    }

    /* gene region labels inside ring */
    for (int g = 0; g < 4; g++) {
        float a  = gene_angles[g];
        int   lr = cy + (int)roundf(ry * 0.5f * sinf(a));
        int   lc = cx + (int)roundf(rx * 0.5f * cosf(a));
        int   len = (int)strlen(gene_names[g]);
        lc -= len / 2;
        if (lr >= 1 && lr < draw_rows && lc >= 1 && lc + len < cols - 1) {
            attrset(COLOR_PAIR(gene_cp[g]) | A_BOLD);
            mvprintw(lr, lc, "%s", gene_names[g]);
        }
    }

    /* centre label */
    attrset(COLOR_PAIR(CP_LB));
    if (cy >= 1 && cy < draw_rows) {
        int lc = cx - 3;
        if (lc >= 1 && lc + 7 < cols - 1)
            mvprintw(cy, lc, "PLASMID");
    }
    attrset(A_NORMAL);
}

/* 9: DNA Ladder — educational unrolled flat view, scrolling */
static void draw_ladder(int cx, float t, int rows, int cols)
{
    int draw_rows = rows - HUD_ROWS - 1;
    int gap       = 8;
    int lx        = cx - gap;
    int rx        = cx + gap;
    int rung_step = 3;   /* rows between base pairs */

    static const char nt_l[] = "ATGCTAGCATGCATGC";
    static const char nt_r[] = "TACGATCGTACGTACG";
    int n_seq = (int)(sizeof(nt_l) - 1);

    /* scrolling offset */
    int scroll = (int)(t * 1.8f) % (rung_step * n_seq);

    /* backbone labels */
    attrset(COLOR_PAIR(CP_S1) | A_BOLD);
    sput(1, lx - 3, '5', rows, cols);
    sput(1, lx - 2, '\'', rows, cols);
    attrset(COLOR_PAIR(CP_S2) | A_BOLD);
    sput(1, rx + 2, '3', rows, cols);
    sput(1, rx + 3, '\'', rows, cols);

    for (int row = 2; row < draw_rows; row++) {
        /* backbone lines */
        attrset(COLOR_PAIR(CP_S1) | A_NORMAL);
        sput(row, lx, '|', rows, cols);
        attrset(COLOR_PAIR(CP_S2) | A_NORMAL);
        sput(row, rx, '|', rows, cols);

        /* base pair rungs */
        int adj = (row + scroll) % (rung_step * n_seq);
        if ((adj % rung_step) == 0) {
            int ni = (adj / rung_step) % n_seq;
            float ph = TAU * (float)row / (float)(n_seq * rung_step) + t * 0.3f;
            attr_t pulse = (sinf(ph) > 0.f) ? A_BOLD : A_NORMAL;

            /* nucleotide chars at strand edges */
            attrset(COLOR_PAIR(CP_S1) | A_BOLD);
            sput(row, lx + 1, (chtype)nt_l[ni], rows, cols);
            attrset(COLOR_PAIR(CP_S2) | A_BOLD);
            sput(row, rx - 1, (chtype)nt_r[ni], rows, cols);

            /* bond type: G-C triple bond, A-T double bond */
            bool strong = (nt_l[ni] == 'G' || nt_l[ni] == 'C');
            char bch    = strong ? '=' : '-';
            attrset(COLOR_PAIR(CP_BD) | pulse);
            for (int c = lx + 2; c < rx - 1; c++)
                sput(row, c, (chtype)bch, rows, cols);

            /* bond label at centre */
            if (rx - lx > 6) {
                attrset(COLOR_PAIR(CP_LB) | pulse);
                sput(row, cx, strong ? '#' : ':', rows, cols);
            }
        }
    }

    /* 3' / 5' bottom labels */
    attrset(COLOR_PAIR(CP_S1) | A_BOLD);
    sput(draw_rows - 1, lx - 3, '3', rows, cols);
    sput(draw_rows - 1, lx - 2, '\'', rows, cols);
    attrset(COLOR_PAIR(CP_S2) | A_BOLD);
    sput(draw_rows - 1, rx + 2, '5', rows, cols);
    sput(draw_rows - 1, rx + 3, '\'', rows, cols);
    attrset(A_NORMAL);
}

/* dispatch table */
typedef void (*DrawFn)(int cx, float t, int rows, int cols);
static const DrawFn draw_fns[N_DNA_TYPES] = {
    draw_bdna, draw_adna, draw_zdna, draw_triple, draw_gquad,
    draw_hairpin, draw_rfork, draw_cruciform, draw_plasmid, draw_ladder,
};

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

typedef struct {
    int   dna_type;
    int   theme;
    float t;
    int   rows, cols;
} Scene;

static void scene_init(Scene *s, int rows, int cols)
{
    s->dna_type = 0;
    s->theme    = 0;
    s->t        = 0.f;
    s->rows     = rows;
    s->cols     = cols;
    color_apply_theme(s->theme);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
}

static void scene_tick(Scene *s, float dt)
{
    s->t += dt;
}

static void scene_draw(const Scene *s)
{
    int cx = s->cols / 2;
    draw_fns[s->dna_type](cx, s->t, s->rows, s->cols);
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
}

static void screen_hud(const Scene *s, int fps, float actual_fps)
{
    int cols = s->cols;

    /* top bar */
    attrset(COLOR_PAIR(CP_HUD));
    mvhline(0, 0, ' ', cols);
    mvprintw(0, 1, "DNA: %s", dna_names[s->dna_type]);
    char right_buf[48];
    snprintf(right_buf, sizeof(right_buf), "Theme: %s  %d/%d fps  %.0f",
             theme_names[s->theme], s->dna_type + 1, N_DNA_TYPES, actual_fps);
    int rlen = (int)strlen(right_buf);
    if (cols - rlen - 1 > 0)
        mvprintw(0, cols - rlen - 1, "%s", right_buf);

    /* bottom bar */
    mvhline(s->rows - 1, 0, ' ', cols);
    mvprintw(s->rows - 1, 1,
             "n/p:type  t:theme  spc:pause  q:quit  ]/[:fps(%d)", fps);
    attrset(A_NORMAL);
}

static void screen_present(void)
{
    refresh();
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void sig_winch(int s) { (void)s; g_resize = 1; }
static void sig_int(int s)   { (void)s; g_quit   = 1; }

int main(void)
{
    signal(SIGWINCH, sig_winch);
    signal(SIGINT,   sig_int);

    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene s;
    scene_init(&s, rows, cols);

    bool    paused     = false;
    int     fps        = SIM_FPS_DEF;
    int64_t tick_ns    = NS_PER_SEC / fps;
    int64_t last_ns    = clock_ns();
    int64_t fps_accum  = 0;
    int     fps_frames = 0;
    float   actual_fps = (float)fps;
    float   dt         = 1.f / (float)fps;

    while (!g_quit) {
        /* handle resize */
        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&s, rows, cols);
        }

        /* input */
        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_quit = 1;    break;
        case ' ':          paused = !paused; break;
        case 'n':
            s.dna_type = (s.dna_type + 1) % N_DNA_TYPES;
            s.t = 0.f;
            break;
        case 'p':
            s.dna_type = (s.dna_type + N_DNA_TYPES - 1) % N_DNA_TYPES;
            s.t = 0.f;
            break;
        case 't':
            s.theme = (s.theme + 1) % N_THEMES;
            color_apply_theme(s.theme);
            break;
        case ']':
            fps = fps + SIM_FPS_STEP <= SIM_FPS_MAX ? fps + SIM_FPS_STEP : SIM_FPS_MAX;
            tick_ns = NS_PER_SEC / fps;
            dt      = 1.f / (float)fps;
            break;
        case '[':
            fps = fps - SIM_FPS_STEP >= SIM_FPS_MIN ? fps - SIM_FPS_STEP : SIM_FPS_MIN;
            tick_ns = NS_PER_SEC / fps;
            dt      = 1.f / (float)fps;
            break;
        default: break;
        }

        if (!paused)
            scene_tick(&s, dt);

        /* draw */
        erase();
        scene_draw(&s);
        screen_hud(&s, fps, actual_fps);
        screen_present();

        /* sleep to hit target fps */
        int64_t now     = clock_ns();
        int64_t elapsed = now - last_ns;
        clock_sleep_ns(tick_ns - elapsed);
        last_ns = clock_ns();

        /* actual fps tracking */
        fps_accum += clock_ns() - now + (tick_ns - elapsed > 0 ? tick_ns - elapsed : 0);
        fps_frames++;
        if (fps_accum >= NS_PER_SEC / 2) {
            actual_fps  = (float)fps_frames * (float)NS_PER_SEC / (float)fps_accum;
            fps_accum   = 0;
            fps_frames  = 0;
        }
    }

    endwin();
    return 0;
}
