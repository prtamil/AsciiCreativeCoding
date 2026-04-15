/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * network_sim.c — SIR epidemic on a Watts-Strogatz small-world network
 *
 * SPLIT DISPLAY
 *   Left  ~60% : network ring — nodes coloured by SIR state
 *   Right ~40% : scrolling stacked epidemic curve (S/I/R over time)
 *
 * N=40 nodes (half the original; ring is readable).
 * Watts-Strogatz: K=4 ring neighbours, 15% rewiring probability.
 *
 * Node symbols
 *   S  grey  ·  susceptible — small, unobtrusive
 *   I  red   *  newly infected (flashes FLASH_TICKS ticks after transition)
 *   I  red   @  infected, settled
 *   R  green +  recovered / immune
 *
 * Edge colours
 *   dim grey      S–S and R–* edges   (background structure)
 *   bright red    any edge touching I  (shows where disease is active)
 *   bright yellow rewired shortcut edge touching I
 *
 * Epidemic curve (right panel)
 *   stacked bar per tick: R (bottom green) → I (middle red) → S (top grey)
 *   scrolls left as time advances; Y axis = node count 0..N
 *
 * R0 = β · <k> / γ — when R0 > 1 epidemic spreads; < 1 it dies
 *
 * Keys:  q quit   ↑↓ β   ←→ γ   r reset   i inject   spc pause
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/network_sim.c \
 *       -o network_sim -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 graph  §5 SIR  §6 draw  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : SIR (Susceptible-Infected-Recovered) epidemic model on
 *                  a Watts-Strogatz small-world network.
 *                  Per tick: each I node infects each S neighbour with
 *                  probability β; each I node recovers with probability γ.
 *                  R0 = β·⟨k⟩/γ: epidemic spreads when R0 > 1.
 *
 * Data-structure : Watts-Strogatz construction: start with a K=4 ring graph
 *                  (each node connected to K/2 nearest neighbours on each
 *                  side); rewire each edge with probability p=0.15, replacing
 *                  target with a random node.  Rewired "shortcut" edges create
 *                  the small-world property: short average path length + high
 *                  clustering coefficient.
 *
 * Math           : SIR basic reproduction number: R0 = β·⟨k⟩/γ where
 *                  ⟨k⟩ is the mean degree.  Epidemic threshold R0=1 marks
 *                  the phase transition between extinction and outbreak.
 *                  Node positions on ring: θ_i = 2πi/N, placed in a circle.
 *
 * Rendering      : Split display: left panel shows network ring with node
 *                  colours (S=grey, I=red, R=green) and edges; right panel
 *                  shows scrolling stacked epidemic curve bar chart.
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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_NODES     40        /* ring nodes — readable without hairball    */
#define WS_K         4        /* ring degree (2 neighbours each side)      */
#define WS_P         0.15f    /* Watts-Strogatz rewiring probability       */

#define BETA_INIT    0.040f   /* infection prob per S-I edge per tick      */
#define GAMMA_INIT   0.025f   /* recovery prob per I node per tick         */
#define BETA_STEP    0.005f
#define GAMMA_STEP   0.005f

#define FLASH_TICKS  6        /* ticks a newly-infected node shows as '*'  */
#define HIST_LEN   500        /* rolling history for epidemic curve        */

#define CELL_W       8        /* pixels per terminal column                */
#define CELL_H      16        /* pixels per terminal row                   */
#define HUD_ROWS     3        /* rows reserved at top for HUD              */
#define NET_FRAC     0.58f    /* fraction of screen width for network      */
#define RING_FRAC    0.44f    /* ring radius / min(half-width, half-height)*/
#define FPS         15

typedef enum { S_STATE, I_STATE, R_STATE } SIR;

enum {
    CP_S=1, CP_I, CP_I_FLASH, CP_R,
    CP_EDGE_DIM, CP_EDGE_HOT, CP_EDGE_REWIRE,
    CP_HUD,
    CP_BAR_S, CP_BAR_I, CP_BAR_R,
    CP_DIVIDER,
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns/1000000000LL, ns%1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_S,          246,  -1);  /* grey — susceptible           */
        init_pair(CP_I,          196,  -1);  /* red  — infected              */
        init_pair(CP_I_FLASH,    226,  -1);  /* bright yellow — just infected*/
        init_pair(CP_R,           46,  -1);  /* green — recovered            */
        init_pair(CP_EDGE_DIM,   236,  -1);  /* very dark — background edges */
        init_pair(CP_EDGE_HOT,   196,  -1);  /* red  — edges touching I      */
        init_pair(CP_EDGE_REWIRE,220,  -1);  /* yellow — rewired + hot       */
        init_pair(CP_HUD,        244,  -1);  /* grey HUD                     */
        init_pair(CP_BAR_S,      246,  -1);
        init_pair(CP_BAR_I,      196,  -1);
        init_pair(CP_BAR_R,       46,  -1);
        init_pair(CP_DIVIDER,    238,  -1);
    } else {
        init_pair(CP_S,         COLOR_WHITE,   -1);
        init_pair(CP_I,         COLOR_RED,     -1);
        init_pair(CP_I_FLASH,   COLOR_YELLOW,  -1);
        init_pair(CP_R,         COLOR_GREEN,   -1);
        init_pair(CP_EDGE_DIM,  COLOR_WHITE,   -1);
        init_pair(CP_EDGE_HOT,  COLOR_RED,     -1);
        init_pair(CP_EDGE_REWIRE,COLOR_YELLOW, -1);
        init_pair(CP_HUD,       COLOR_WHITE,   -1);
        init_pair(CP_BAR_S,     COLOR_WHITE,   -1);
        init_pair(CP_BAR_I,     COLOR_RED,     -1);
        init_pair(CP_BAR_R,     COLOR_GREEN,   -1);
        init_pair(CP_DIVIDER,   COLOR_WHITE,   -1);
    }
}

/* ===================================================================== */
/* §4  graph — Watts-Strogatz                                             */
/* ===================================================================== */

static bool  g_adj[N_NODES][N_NODES];
static bool  g_rewired[N_NODES][N_NODES];
static float g_node_px[N_NODES], g_node_py[N_NODES];

static void graph_build(void)
{
    memset(g_adj,     0, sizeof g_adj);
    memset(g_rewired, 0, sizeof g_rewired);

    /* Step 1: ring lattice */
    for (int i = 0; i < N_NODES; i++)
        for (int k = 1; k <= WS_K/2; k++) {
            int j = (i+k) % N_NODES;
            g_adj[i][j] = g_adj[j][i] = true;
        }

    /* Step 2: rewire each rightward edge with probability WS_P */
    for (int i = 0; i < N_NODES; i++) {
        for (int k = 1; k <= WS_K/2; k++) {
            if ((float)rand()/(float)RAND_MAX >= WS_P) continue;
            int old_j = (i+k) % N_NODES;
            int new_j, tries = 0;
            do { new_j = rand() % N_NODES; tries++; }
            while ((new_j==i || g_adj[i][new_j]) && tries < N_NODES);
            if (tries >= N_NODES) continue;
            g_adj[i][old_j] = g_adj[old_j][i] = false;
            g_adj[i][new_j] = g_adj[new_j][i] = true;
            g_rewired[i][new_j] = g_rewired[new_j][i] = true;
        }
    }
}

static float mean_degree(void)
{
    int tot = 0;
    for (int i=0;i<N_NODES;i++)
        for (int j=0;j<N_NODES;j++)
            if (g_adj[i][j]) tot++;
    return (float)tot/(float)N_NODES;
}

/* Place nodes on a ring inside the left network panel */
static void layout_ring(int rows, int cols)
{
    int net_w = (int)(cols * NET_FRAC);
    float pw  = (float)(net_w  * CELL_W);
    float ph  = (float)((rows - HUD_ROWS) * CELL_H);
    float cx  = pw * 0.5f;
    float cy  = ph * 0.5f + (float)(HUD_ROWS * CELL_H);
    float r   = RING_FRAC * (pw < ph ? pw : ph) * 0.5f;
    for (int i = 0; i < N_NODES; i++) {
        float a = 2.f*(float)M_PI*(float)i/(float)N_NODES - (float)M_PI/2.f;
        g_node_px[i] = cx + r*cosf(a);
        g_node_py[i] = cy + r*sinf(a);
    }
}

static int px_col(float px) { return (int)(px/(float)CELL_W + 0.5f); }
static int px_row(float py) { return (int)(py/(float)CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  SIR dynamics                                                       */
/* ===================================================================== */

static SIR g_state[N_NODES];
static int  g_flash[N_NODES];   /* countdown: shows '*' while > 0         */
static float g_beta  = BETA_INIT;
static float g_gamma = GAMMA_INIT;
static bool  g_paused = false;
static int   g_tick   = 0;

/* rolling history for epidemic curve */
static int g_hist_s[HIST_LEN], g_hist_i[HIST_LEN], g_hist_r[HIST_LEN];
static int g_hist_head = 0, g_hist_n = 0;
static int g_peak_i    = 0;   /* all-time peak infected count             */

static int count_s(void){ int n=0; for(int i=0;i<N_NODES;i++) n+=(g_state[i]==S_STATE); return n; }
static int count_i(void){ int n=0; for(int i=0;i<N_NODES;i++) n+=(g_state[i]==I_STATE); return n; }
static int count_r(void){ int n=0; for(int i=0;i<N_NODES;i++) n+=(g_state[i]==R_STATE); return n; }

static void sir_reset(void)
{
    for (int i=0;i<N_NODES;i++) { g_state[i]=S_STATE; g_flash[i]=0; }
    int seed = rand() % N_NODES;
    g_state[seed] = I_STATE;
    g_flash[seed] = FLASH_TICKS;
    g_tick = 0; g_hist_head = 0; g_hist_n = 0; g_peak_i = 0;
}

static void sir_inject(void)
{
    for (int tries=0; tries < N_NODES*2; tries++) {
        int i = rand() % N_NODES;
        if (g_state[i]==S_STATE) {
            g_state[i]=I_STATE; g_flash[i]=FLASH_TICKS; return;
        }
    }
}

static void sir_tick(void)
{
    if (g_paused) return;

    /* decrement flash counters */
    for (int i=0;i<N_NODES;i++) if (g_flash[i]>0) g_flash[i]--;

    /* synchronous SIR update */
    SIR nxt[N_NODES];
    memcpy(nxt, g_state, sizeof g_state);

    for (int i=0;i<N_NODES;i++) {
        if (g_state[i] != I_STATE) continue;
        /* recovery */
        if ((float)rand()/(float)RAND_MAX < g_gamma)
            nxt[i] = R_STATE;
        /* transmission */
        for (int j=0;j<N_NODES;j++) {
            if (!g_adj[i][j] || g_state[j]!=S_STATE) continue;
            if ((float)rand()/(float)RAND_MAX < g_beta) {
                nxt[j] = I_STATE;
                g_flash[j] = FLASH_TICKS;
            }
        }
    }
    memcpy(g_state, nxt, sizeof g_state);
    g_tick++;

    /* record history */
    int ci = count_i();
    g_hist_s[g_hist_head] = count_s();
    g_hist_i[g_hist_head] = ci;
    g_hist_r[g_hist_head] = count_r();
    g_hist_head = (g_hist_head+1) % HIST_LEN;
    if (g_hist_n < HIST_LEN) g_hist_n++;
    if (ci > g_peak_i) g_peak_i = ci;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

/* Bresenham line with slope-matched ASCII chars */
static void draw_line(int x0,int y0,int x1,int y1,attr_t attr,int cols,int rows)
{
    int dx=abs(x1-x0),dy=abs(y1-y0),sx=x0<x1?1:-1,sy=y0<y1?1:-1,err=dx-dy;
    for(;;){
        if(x0>=0&&x0<cols&&y0>=0&&y0<rows){
            int e2=2*err; bool bx=(e2>-dy),by=(e2<dx);
            chtype ch=(bx&&by)?(sx==sy?'\\':'/'):(bx?'-':'|');
            attron(attr); mvaddch(y0,x0,ch); attroff(attr);
        }
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>-dy){err-=dy;x0+=sx;}
        if(e2< dx){err+=dx;y0+=sy;}
    }
}

/* ── left panel: network ─────────────────────────────────────────────── */
static void draw_network(int rows, int cols)
{
    int net_w = (int)(cols * NET_FRAC);

    /* edges — drawn first so nodes sit on top */
    for (int i=0;i<N_NODES;i++) {
        for (int j=i+1;j<N_NODES;j++) {
            if (!g_adj[i][j]) continue;

            bool hot = (g_state[i]==I_STATE || g_state[j]==I_STATE);
            attr_t attr;
            if (!hot) {
                attr = COLOR_PAIR(CP_EDGE_DIM);          /* dim background  */
            } else if (g_rewired[i][j]) {
                attr = COLOR_PAIR(CP_EDGE_REWIRE)|A_BOLD; /* shortcut active */
            } else {
                attr = COLOR_PAIR(CP_EDGE_HOT)|A_BOLD;   /* ring edge active*/
            }

            draw_line(px_col(g_node_px[i]), px_row(g_node_py[i]),
                      px_col(g_node_px[j]), px_row(g_node_py[j]),
                      attr, net_w, rows);
        }
    }

    /* nodes — drawn on top of edges */
    for (int i=0;i<N_NODES;i++) {
        int c = px_col(g_node_px[i]);
        int r = px_row(g_node_py[i]);
        if (c<0||c>=net_w||r<HUD_ROWS||r>=rows) continue;

        int    cp;
        chtype ch;
        attr_t ex = 0;

        switch (g_state[i]) {
        case S_STATE:
            cp = CP_S; ch = '.'; break;
        case I_STATE:
            if (g_flash[i] > 0) {
                cp = CP_I_FLASH; ch = '*'; ex = A_BOLD;
            } else {
                cp = CP_I;       ch = '@'; ex = A_BOLD;
            }
            break;
        case R_STATE:
            cp = CP_R; ch = '+'; break;
        default:
            cp = CP_S; ch = '.'; break;
        }
        attron(COLOR_PAIR(cp)|ex);
        mvaddch(r, c, ch);
        attroff(COLOR_PAIR(cp)|ex);
    }
}

/* ── right panel: epidemic curve ────────────────────────────────────── */
/*
 * Stacked bar chart, newest tick at right edge, scrolls left.
 * Each column:  bottom = R (green)
 *               middle = I (red)
 *               top    = S (grey)
 */
static void draw_chart(int rows, int cols)
{
    int net_w   = (int)(cols * NET_FRAC);
    int chart_x = net_w + 1;          /* leave one col for divider          */
    int chart_w = cols - chart_x;
    if (chart_w < 8) return;

    int chart_top = HUD_ROWS;
    int chart_h   = rows - chart_top;
    if (chart_h < 4) return;

    /* vertical divider */
    attron(COLOR_PAIR(CP_DIVIDER));
    for (int r=HUD_ROWS; r<rows; r++) mvaddch(r, net_w, ACS_VLINE);
    attroff(COLOR_PAIR(CP_DIVIDER));

    /* chart title */
    attron(COLOR_PAIR(CP_HUD)|A_BOLD);
    mvprintw(chart_top, chart_x, " EPIDEMIC CURVE");
    attroff(COLOR_PAIR(CP_HUD)|A_BOLD);

    /* Y-axis label column + data area */
    int y_lbl_w = 3;                  /* "40 " "20 " " 0 "                  */
    int data_x  = chart_x + y_lbl_w;
    int data_w  = chart_w - y_lbl_w;
    if (data_w < 4) return;

    int data_top = chart_top + 1;
    int data_h   = chart_h   - 1;    /* one row for title                   */
    if (data_h < 2) return;

    /* Y axis labels */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(data_top,            chart_x, "%2d", N_NODES);
    mvprintw(data_top+data_h/4,   chart_x, "%2d", N_NODES*3/4);
    mvprintw(data_top+data_h/2,   chart_x, "%2d", N_NODES/2);
    mvprintw(data_top+data_h*3/4, chart_x, "%2d", N_NODES/4);
    mvprintw(data_top+data_h-1,   chart_x, " 0");
    attroff(COLOR_PAIR(CP_HUD));

    /* stacked bars (most recent at right, older scrolling left) */
    int n_show = g_hist_n < data_w ? g_hist_n : data_w;
    if (n_show == 0) return;

    for (int cx = 0; cx < n_show; cx++) {
        /* index into circular buffer: oldest shown at left, newest at right */
        int bi = (g_hist_head - n_show + cx + HIST_LEN) % HIST_LEN;
        int hs = g_hist_s[bi];
        int hi = g_hist_i[bi];
        int hr = g_hist_r[bi];

        int scol = data_x + (data_w - n_show) + cx;
        if (scol < data_x || scol >= cols) continue;

        /* map counts to row heights in the data area */
        int rh = hr * data_h / N_NODES;
        int ih = hi * data_h / N_NODES;
        int sh = hs * data_h / N_NODES;

        /* draw each row of this column from bottom up */
        for (int rb = 0; rb < data_h; rb++) {
            int srow = data_top + (data_h - 1 - rb);
            if (srow < 0 || srow >= rows) continue;

            int    cp;
            chtype ch;
            attr_t ex = 0;

            if      (rb < rh)          { cp=CP_BAR_R; ch='-';            }
            else if (rb < rh+ih)       { cp=CP_BAR_I; ch='#'; ex=A_BOLD; }
            else if (rb < rh+ih+sh)    { cp=CP_BAR_S; ch='=';            }
            else                       { continue;                        }

            attron(COLOR_PAIR(cp)|ex);
            mvaddch(srow, scol, ch);
            attroff(COLOR_PAIR(cp)|ex);
        }
    }

    /* peak I marker */
    if (g_peak_i > 0) {
        int peak_row = data_top + data_h - 1 - g_peak_i*data_h/N_NODES;
        if (peak_row >= data_top && peak_row < data_top+data_h) {
            attron(COLOR_PAIR(CP_I)|A_DIM);
            mvprintw(peak_row, chart_x+y_lbl_w-3, "pk");
            attroff(COLOR_PAIR(CP_I)|A_DIM);
        }
    }
}

/* ── HUD (top 3 rows, full width) ────────────────────────────────────── */
static void draw_hud(int cols)
{
    int ns=count_s(), ni=count_i(), nr=count_r();
    float mk = mean_degree();
    float r0 = (g_gamma>0.f) ? g_beta*mk/g_gamma : 0.f;

    /* detect epidemic phase */
    const char *phase;
    if (ni == 0 && nr == 0)         phase = "READY  ";
    else if (ni == 0)               phase = "EXTINCT";
    else if (g_hist_n >= 2) {
        int prev_i = g_hist_i[(g_hist_head-2+HIST_LEN)%HIST_LEN];
        phase = (ni > prev_i) ? "GROWING" : (ni < prev_i) ? "WANING " : "PLATEAU";
    } else                          phase = "SEEDED ";

    /* row 0: controls */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " network_sim  q:quit  spc:pause  r:reset  i:inject  "
        "↑↓:β  ←→:γ");
    attroff(COLOR_PAIR(CP_HUD));

    /* row 1: live parameters */
    attron(COLOR_PAIR(CP_HUD)|A_BOLD);
    mvprintw(1, 0, " β=%.3f  γ=%.3f ", g_beta, g_gamma);
    attroff(COLOR_PAIR(CP_HUD)|A_BOLD);

    /* R0 coloured by threshold */
    attr_t r0_attr = (r0 > 1.f) ? (COLOR_PAIR(CP_I)|A_BOLD) : (COLOR_PAIR(CP_R)|A_BOLD);
    attron(r0_attr);
    mvprintw(1, 18, "R0=%-4.2f", r0);
    attroff(r0_attr);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(1, 27, " <k>=%.1f  tick=%-4d  S=%-3d I=%-3d R=%-3d  %s%s",
             mk, g_tick, ns, ni, nr,
             phase,
             g_paused ? "  [PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD));

    /* row 2: SIR proportion bar */
    {
        int bar_w = cols - 22;
        if (bar_w < 6) bar_w = 6;

        mvprintw(2, 0, " ");
        int bx = 1;

        /* S segment */
        int fill_s = ns * bar_w / N_NODES;
        attron(COLOR_PAIR(CP_BAR_S));
        for (int i=0;i<fill_s&&bx+i<cols;i++) mvaddch(2, bx+i, '=');
        attroff(COLOR_PAIR(CP_BAR_S));
        bx += fill_s;

        /* I segment */
        int fill_i = ni * bar_w / N_NODES;
        attron(COLOR_PAIR(CP_BAR_I)|A_BOLD);
        for (int i=0;i<fill_i&&bx+i<cols;i++) mvaddch(2, bx+i, '#');
        attroff(COLOR_PAIR(CP_BAR_I)|A_BOLD);
        bx += fill_i;

        /* R segment */
        int fill_r = nr * bar_w / N_NODES;
        attron(COLOR_PAIR(CP_BAR_R));
        for (int i=0;i<fill_r&&bx+i<cols;i++) mvaddch(2, bx+i, '-');
        attroff(COLOR_PAIR(CP_BAR_R));
        bx += fill_r;

        /* legend */
        attron(COLOR_PAIR(CP_BAR_S)); mvprintw(2, bx+2, "S"); attroff(COLOR_PAIR(CP_BAR_S));
        attron(COLOR_PAIR(CP_BAR_I)|A_BOLD); mvprintw(2, bx+4, "I"); attroff(COLOR_PAIR(CP_BAR_I)|A_BOLD);
        attron(COLOR_PAIR(CP_BAR_R)); mvprintw(2, bx+6, "R"); attroff(COLOR_PAIR(CP_BAR_R));

        /* node symbol key */
        attron(COLOR_PAIR(CP_HUD));
        mvprintw(2, bx+9, " key: ");
        attroff(COLOR_PAIR(CP_HUD));
        attron(COLOR_PAIR(CP_S));       mvprintw(2, bx+15, ".=S "); attroff(COLOR_PAIR(CP_S));
        attron(COLOR_PAIR(CP_I)|A_BOLD);mvprintw(2, bx+19, "@=I "); attroff(COLOR_PAIR(CP_I)|A_BOLD);
        attron(COLOR_PAIR(CP_R));       mvprintw(2, bx+23, "+=R");  attroff(COLOR_PAIR(CP_R));
    }
}

static void scene_draw(int rows, int cols)
{
    draw_hud(cols);
    draw_network(rows, cols);
    draw_chart(rows, cols);
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s==SIGINT||s==SIGTERM) g_quit=1;
    if (s==SIGWINCH)           g_resize=1;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,  sig_h);
    signal(SIGTERM, sig_h);
    signal(SIGWINCH,sig_h);

    initscr();
    cbreak(); noecho(); keypad(stdscr,TRUE);
    nodelay(stdscr,TRUE); curs_set(0); typeahead(-1);
    color_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    graph_build();
    layout_ring(rows, cols);
    sir_reset();

    long long frame_ns = 1000000000LL / FPS;

    while (!g_quit) {
        if (g_resize) {
            g_resize=0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            layout_ring(rows, cols);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit=1;        break;
        case ' ':  g_paused=!g_paused;                 break;
        case 'r': case 'R': sir_reset();               break;
        case 'i': case 'I': sir_inject();              break;
        case KEY_UP:
            g_beta+=BETA_STEP; if(g_beta>1.f)g_beta=1.f; break;
        case KEY_DOWN:
            g_beta-=BETA_STEP; if(g_beta<0.f)g_beta=0.f; break;
        case KEY_RIGHT:
            g_gamma+=GAMMA_STEP; if(g_gamma>1.f)g_gamma=1.f; break;
        case KEY_LEFT:
            g_gamma-=GAMMA_STEP; if(g_gamma<0.f)g_gamma=0.f; break;
        default: break;
        }

        long long t0 = clock_ns();
        sir_tick();
        erase();
        scene_draw(rows, cols);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(frame_ns - (clock_ns()-t0));
    }
    return 0;
}
