/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * network_sim.c — a disease spreading through a small-world social network.
 *
 * Left panel: the network as a ring of people (nodes), coloured by health
 * state (grey susceptible, red infected, green recovered). Right panel: a
 * scrolling chart of how many people are in each state over time.
 *
 * The disease model is SIR (Susceptible -> Infected -> Recovered); the
 * network is a Watts-Strogatz "small world" (a ring where a few links are
 * randomly rewired into long-range shortcuts). See Watts & Strogatz 1998,
 * Nature 393; Kermack & McKendrick 1927 (the original SIR model).
 * Sister files: algorithms/graph_search.c (same graph layout idea).
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/network_sim.c \
 *            -o network_sim -lncurses -lm
 */


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

/* ── §1 config — tunable constants: network size, disease rates, layout ── */

#define N_NODES     40        /* people in the network — small enough to see */
#define WS_K         4        /* each person starts linked to 4 ring neighbours (2 per side) */
#define WS_P         0.15f    /* chance each link gets rewired into a shortcut */

#define BETA_INIT    0.040f   /* chance an infected person infects a susceptible neighbour, per tick */
#define GAMMA_INIT   0.025f   /* chance an infected person recovers, per tick */
#define BETA_STEP    0.005f   /* how much arrow keys nudge beta  */
#define GAMMA_STEP   0.005f   /* how much arrow keys nudge gamma */

#define FLASH_TICKS  6        /* how long a freshly-infected node flashes bright */
#define HIST_LEN   500        /* how many past ticks the chart remembers */

#define CELL_W       8        /* sub-cell width: terminal cells are taller than wide, */
#define CELL_H      16        /*   so we work in fake pixels and divide down when drawing */
#define NET_FRAC     0.58f    /* left fraction of the screen given to the network */
#define RING_FRAC    0.44f    /* ring radius as a fraction of the panel half-size */
#define FPS         15        /* simulation/redraw rate, frames per second */

/* Rows fenced off at top and bottom for status bars: the top bar shows the
 * live numbers (rates, R0, counts), the bottom bar shows the key hints. The
 * network and chart only draw in the band between them. */
#define HUD_TOP_ROWS 2        /* row 0: numbers; row 1: S/I/R proportion bar */
#define HUD_BOT_ROWS 1        /* last row: key hints                          */

/* The three health states every person is in, exactly one at a time:
 * Susceptible (healthy, can catch it), Infected (has it, can pass it on),
 * Recovered (had it, now immune forever in this basic model). The whole
 * simulation is just the rules for moving S->I->R. An enum (not bare 0/1/2)
 * so comparisons like `state[i] == I_STATE` read in English and typos are
 * caught at compile time. Classic SIR model: Kermack & McKendrick 1927. */
typedef enum {
    S_STATE,    /* Susceptible — healthy, can be infected by a sick neighbour */
    I_STATE,    /* Infected    — sick; spreads it, may recover each tick       */
    R_STATE,    /* Recovered   — immune; never changes again                   */
} SIR;

/* Names for the ncurses colour-pair slots (slot 0 is reserved, so start at 1). */
enum {
    CP_S=1, CP_I, CP_I_FLASH, CP_R,   /* node colours by state            */
    CP_EDGE_DIM, CP_EDGE_HOT, CP_EDGE_REWIRE,  /* link colours            */
    CP_HUD,                /* top numbers bar — bright yellow            */
    CP_HINT,               /* bottom key-hint bar — bright cyan          */
    CP_BAR_S, CP_BAR_I, CP_BAR_R,     /* the three chart bands           */
    CP_DIVIDER,            /* the line splitting the two panels          */
};

/* ── §2 clock — monotonic time + sleep, for the frame timer ── */

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

/* ── §3 color — ncurses colour pairs for nodes, edges, chart, HUD ── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    /* Prefer the 256-colour palette; fall back to the 8 basic colours. */
    if (COLORS >= 256) {
        init_pair(CP_S,          246,  -1);  /* grey — susceptible            */
        init_pair(CP_I,          196,  -1);  /* red  — infected               */
        init_pair(CP_I_FLASH,    226,  -1);  /* bright yellow — just infected */
        init_pair(CP_R,           46,  -1);  /* green — recovered             */
        init_pair(CP_EDGE_DIM,   244,  -1);  /* very dark — background edges  */
        init_pair(CP_EDGE_HOT,   196,  -1);  /* red  — edges touching I       */
        init_pair(CP_EDGE_REWIRE,220,  -1);  /* yellow — rewired + hot        */
        init_pair(CP_HUD,        226,  -1);  /* bright yellow — top data bar  */
        init_pair(CP_HINT,        51,  -1);  /* bright cyan — bottom hint bar */
        init_pair(CP_BAR_S,      246,  -1);
        init_pair(CP_BAR_I,      196,  -1);
        init_pair(CP_BAR_R,       46,  -1);
        init_pair(CP_DIVIDER,    246,  -1);
    } else {
        init_pair(CP_S,         COLOR_WHITE,   -1);
        init_pair(CP_I,         COLOR_RED,     -1);
        init_pair(CP_I_FLASH,   COLOR_YELLOW,  -1);
        init_pair(CP_R,         COLOR_GREEN,   -1);
        init_pair(CP_EDGE_DIM,  COLOR_WHITE,   -1);
        init_pair(CP_EDGE_HOT,  COLOR_RED,     -1);
        init_pair(CP_EDGE_REWIRE,COLOR_YELLOW, -1);
        init_pair(CP_HUD,       COLOR_YELLOW,  -1);
        init_pair(CP_HINT,      COLOR_CYAN,    -1);
        init_pair(CP_BAR_S,     COLOR_WHITE,   -1);
        init_pair(CP_BAR_I,     COLOR_RED,     -1);
        init_pair(CP_BAR_R,     COLOR_GREEN,   -1);
        init_pair(CP_DIVIDER,   COLOR_WHITE,   -1);
    }
}

/* ── §4 data types — graph, disease state, history, and the Scene that holds it all ── */

/* A point on the screen, in fake "pixels" (CELL_W x CELL_H pixels per cell).
 * Float, not int, so the ring trig doesn't accumulate rounding and make nodes
 * jitter when the layout is recomputed on resize. y grows downward, the usual
 * terminal convention. */
typedef struct {
    float x;        /* rightward; bigger x is further right */
    float y;        /* downward;  bigger y is further down  */
} Vec2;

/* Who is connected to whom. Built once at startup and never changed after
 * that — resetting ('r') or injecting ('i') only touches the disease, never
 * the wiring. That lets you re-run the same outbreak on the same network and
 * see how much of the difference is pure luck versus the layout.
 *
 * Two square tables, both symmetric (if i links j, then j links i):
 *   adj[i][j]      is there a link between person i and person j?
 *   rewired[i][j]  is that link one of the random long-range shortcuts?
 *                  (always a subset of adj.) Kept separate only so the
 *                  drawing code can paint shortcuts bright yellow — the
 *                  most telling feature of a small-world network.
 * A full table (not an adjacency list) because N is tiny (40) and "are i
 * and j linked?" is asked constantly; the table answers it instantly.
 * Watts & Strogatz 1998 for the construction. */
typedef struct {
    bool adj    [N_NODES][N_NODES];     /* link present? symmetric, no self-links */
    bool rewired[N_NODES][N_NODES];     /* link is a shortcut? (subset of adj)    */
} Topology;

/* The disease side of each person: their current state, plus a short
 * countdown so freshly-infected nodes flash bright for a few ticks before
 * settling into the steady infected look. state[] picks the colour, flash[]
 * picks the glyph ('*' while flashing, '@' once settled). Reset on 'r',
 * nudged by 'i' (infect a random healthy person).
 *
 * flash is a separate countdown rather than an extra state so the spreading
 * rules in sir_tick stay simple — they never have to remember to "un-flash" a
 * node. It's an int, not a bool, because the flash lasts several ticks. */
typedef struct {
    SIR state[N_NODES];     /* S / I / R for each person                       */
    int flash[N_NODES];     /* ticks left flashing after infection (0 = done)  */
} EpiState;

/* The data behind the scrolling chart: how many people were in each state at
 * each past tick. It's a ring buffer — after HIST_LEN ticks it wraps and
 * overwrites the oldest sample, so memory stays bounded while the chart keeps
 * showing the most recent window. Three parallel arrays (not one array of
 * triples) so `h->i[k]` literally reads "infected count at tick k".
 *
 * head is the next slot to write; n is how many slots are filled so far.
 * peak_i remembers the worst-ever infected count so the chart can mark it
 * even after the outbreak has died down. Always s+i+r == N_NODES, since every
 * person is in exactly one state. */
typedef struct {
    int s[HIST_LEN];        /* susceptible count per recorded tick      */
    int i[HIST_LEN];        /* infected count per recorded tick         */
    int r[HIST_LEN];        /* recovered count per recorded tick        */
    int head;               /* next slot to write (0..HIST_LEN-1)       */
    int n;                  /* how many slots filled (0..HIST_LEN)      */
    int peak_i;             /* highest infected count seen all run      */
} EpiHistory;

/* Everything one running simulation needs, in one bag passed by pointer to
 * every function. Splitting it out of globals means each function's signature
 * shows what it reads and writes. (The only true globals are the signal-handler
 * flags in §8, which signals can't reach any other way.)
 *
 * Its parts have three different lifetimes: the graph and node positions are
 * built once (positions recomputed on resize); the disease fields get wiped on
 * 'r' so you can re-run on the same graph; rows/cols are refreshed every frame.
 *
 * Note: <k> (mean links per node), R0, and the live S/I/R counts are NOT
 * stored — they're recomputed each frame so they can never go stale. */
typedef struct {
    Topology    topology;       /* the network (fixed once built)           */
    Vec2        pos[N_NODES];   /* where each node sits on the ring         */

    EpiState    epi;            /* per-person disease state + flash         */
    EpiHistory  history;        /* past counts, for the chart               */
    float       beta;           /* infection chance per sick-healthy contact, 0..1 */
    float       gamma;          /* recovery chance per sick person, 0..1    */
    int         tick;           /* ticks since the last reset               */
    bool        paused;         /* true = disease frozen (SPACE toggles)    */

    int         rows;           /* terminal height this frame               */
    int         cols;           /* terminal width  this frame               */
} Scene;

/* ── shared helpers ── */

/* The one coin flip behind every random event here: returns true with
 * probability p (rewiring a link, recovering, infecting). Caller keeps p in
 * 0..1; the input handler clamps it. */
static inline bool prob_roll(float p)
{
    return (float)rand() / (float)RAND_MAX < p;
}

/* Turn a pixel position into a terminal column/row, rounding to nearest. */
static inline int px_col(float px) { return (int)(px / (float)CELL_W + 0.5f); }
static inline int px_row(float py) { return (int)(py / (float)CELL_H + 0.5f); }

/* ── §5 graph — build the small-world network and place nodes on a circle ── */

/* ── building the network ── */

/* Wipe both link tables back to empty — first step of any (re)build. */
static void topology_clear(Topology *t)
{
    memset(t->adj,     0, sizeof t->adj);
    memset(t->rewired, 0, sizeof t->rewired);
}

/* Step 1: seat everyone in a circle and link each person to their K nearest
 * neighbours (K/2 on each side). A tidy, regular ring — neighbours all know
 * each other, but getting across the circle takes many hops. rewire_edges
 * then adds the shortcuts that make it a "small world". */
static void connect_ring_neighbours(Topology *t)
{
    for (int i = 0; i < N_NODES; i++)
        for (int k = 1; k <= WS_K / 2; k++) {
            int j = (i + k) % N_NODES;
            t->adj[i][j] = t->adj[j][i] = true;
        }
}

/* Unplug one of person i's links (to old_j) and re-plug it to a random other
 * person — a long-range shortcut. Picks a random partner that isn't i and
 * isn't already linked; gives up after a bounded number of tries if the graph
 * is too dense to find one (rare at our settings). */
static void rewire_one_edge(Topology *t, int i, int old_j)
{
    for (int tries = 0; tries < N_NODES; tries++) {
        int new_j = rand() % N_NODES;
        if (new_j == i || t->adj[i][new_j]) continue;
        t->adj[i][old_j] = t->adj[old_j][i] = false;
        t->adj[i][new_j] = t->adj[new_j][i] = true;
        t->rewired[i][new_j] = t->rewired[new_j][i] = true;
        return;
    }
    /* couldn't find a free partner — keep the original ring link */
}

/* Step 2: walk every ring link and, with probability WS_P, turn it into a
 * shortcut. Visiting the same pairs Step 1 made, so each link gets one shot. */
static void rewire_edges(Topology *t)
{
    for (int i = 0; i < N_NODES; i++)
        for (int k = 1; k <= WS_K / 2; k++)
            if (prob_roll(WS_P))
                rewire_one_edge(t, i, (i + k) % N_NODES);
}

/* Build the whole small-world network: empty it, make the ring, add shortcuts.
 * (Watts & Strogatz 1998.) */
static void topology_build_ws(Topology *t)
{
    topology_clear(t);
    connect_ring_neighbours(t);
    rewire_edges(t);
}

/* Average number of links per person. Stays at K even after rewiring (each
 * unplug is paired with a re-plug). Feeds the R0 number in the HUD. */
static float topology_mean_degree(const Topology *t)
{
    int total = 0;
    for (int i = 0; i < N_NODES; i++)
        for (int j = 0; j < N_NODES; j++)
            if (t->adj[i][j]) total++;
    return (float)total / (float)N_NODES;
}

/* ── placing nodes on the ring ── */

/* Work out where the ring sits: its centre and radius. The network gets the
 * left NET_FRAC of the screen and the rows between the two HUD bars; the ring
 * is centred in that box and sized to fit with a margin. */
static void compute_ring_geometry(int rows, int cols,
                                  Vec2 *centre, float *radius)
{
    int   net_w = (int)(cols * NET_FRAC);
    float pw    = (float)(net_w * CELL_W);
    float ph    = (float)((rows - HUD_TOP_ROWS - HUD_BOT_ROWS) * CELL_H);
    centre->x   = pw * 0.5f;
    centre->y   = ph * 0.5f + (float)(HUD_TOP_ROWS * CELL_H);
    *radius     = RING_FRAC * (pw < ph ? pw : ph) * 0.5f;
}

/* Where node i goes on the circle: spread the N nodes evenly around it. The
 * -pi/2 twist starts node 0 at the top (12 o'clock) instead of the right. */
static Vec2 place_node_on_ring(int i, int n, Vec2 centre, float radius)
{
    float angle = 2.f * (float)M_PI * (float)i / (float)n
                - (float)M_PI / 2.f;
    Vec2  p = { centre.x + radius * cosf(angle),
                centre.y + radius * sinf(angle) };
    return p;
}

/* Place every node on the ring. Run at startup and on every resize so the
 * picture refits the terminal; the links themselves never change. */
static void layout_ring(Scene *sc)
{
    Vec2  centre;
    float radius;
    compute_ring_geometry(sc->rows, sc->cols, &centre, &radius);
    for (int i = 0; i < N_NODES; i++)
        sc->pos[i] = place_node_on_ring(i, N_NODES, centre, radius);
}

/* ── §6 SIR dynamics — one tick of the disease spreading and people recovering ── */

/* How many people are currently in a given state (S, I, or R). */
static int count_state(const EpiState *epi, SIR target)
{
    int n = 0;
    for (int i = 0; i < N_NODES; i++)
        if (epi->state[i] == target) n++;
    return n;
}

/* Start a fresh outbreak: everyone healthy except one random patient zero,
 * history cleared, clock to zero. The network is left alone, so each 'r'
 * re-runs the disease on the same wiring. */
static void epi_reset(Scene *sc)
{
    for (int i = 0; i < N_NODES; i++) {
        sc->epi.state[i] = S_STATE;
        sc->epi.flash[i] = 0;
    }
    int seed = rand() % N_NODES;
    sc->epi.state[seed] = I_STATE;
    sc->epi.flash[seed] = FLASH_TICKS;
    sc->tick    = 0;
    sc->history = (EpiHistory){ 0 };
}

/* Infect one random healthy person (the 'i' key) — restarts spread after an
 * outbreak fizzles, without wiping the chart. Gives up if nobody's healthy. */
static void epi_inject(EpiState *epi)
{
    for (int tries = 0; tries < N_NODES * 2; tries++) {
        int i = rand() % N_NODES;
        if (epi->state[i] == S_STATE) {
            epi->state[i] = I_STATE;
            epi->flash[i] = FLASH_TICKS;
            return;
        }
    }
}

/* Tick down each node's flash timer. When it hits 0 the glyph settles from
 * '*' to '@'. Runs before the spread step so this tick's new infections start
 * with a full flash. */
static void decrement_flash_counters(EpiState *epi)
{
    for (int i = 0; i < N_NODES; i++)
        if (epi->flash[i] > 0) epi->flash[i]--;
}

/* Give one sick person a chance (gamma) to recover this tick. Staged into
 * nxt[] rather than applied immediately (see snapshot/commit below). */
static inline void try_recovery(int i, SIR *nxt, float gamma_rate)
{
    if (prob_roll(gamma_rate))
        nxt[i] = R_STATE;
}

/* Give a sick->healthy contact a chance (beta) to pass the disease along; on
 * success the healthy neighbour j becomes infected and starts flashing. */
static inline void try_transmission(int j, EpiState *epi, SIR *nxt,
                                    float beta_rate)
{
    if (prob_roll(beta_rate)) {
        nxt[j]        = I_STATE;
        epi->flash[j] = FLASH_TICKS;
    }
}

/* Everything one sick person does in a tick: maybe recover, and roll to
 * infect each healthy neighbour. Whether a neighbour is healthy is read from
 * this tick's snapshot, so the result doesn't depend on loop order. */
static void tick_one_infected(int i, const Scene *sc, EpiState *epi,
                              SIR *nxt)
{
    try_recovery(i, nxt, sc->gamma);
    for (int j = 0; j < N_NODES; j++) {
        if (!sc->topology.adj[i][j])  continue;
        if (epi->state[j] != S_STATE) continue;
        try_transmission(j, epi, nxt, sc->beta);
    }
}

/* Record this tick's (S, I, R) counts into the ring buffer and update the
 * worst-ever infected count. Wraps around to overwrite the oldest sample. */
static void history_record(EpiHistory *h, int s, int i, int r)
{
    h->s[h->head] = s;
    h->i[h->head] = i;
    h->r[h->head] = r;
    h->head = (h->head + 1) % HIST_LEN;
    if (h->n < HIST_LEN) h->n++;
    if (i > h->peak_i)   h->peak_i = i;
}

/* The infected count one tick ago. The HUD compares it to "now" to decide
 * whether the outbreak is growing, fading, or flat. Returns 0 before there's
 * a previous tick to look at. */
static int history_prev_infected(const EpiHistory *h)
{
    if (h->n < 2) return 0;
    return h->i[(h->head - 2 + HIST_LEN) % HIST_LEN];
}

/* Copy the current states into a scratch buffer. Everyone decides their next
 * move by looking at this frozen photo, so two people changing in the same
 * tick can't trip over each other and the result is order-independent. */
static inline void snapshot_state(const EpiState *epi, SIR *out)
{
    memcpy(out, epi->state, sizeof epi->state);
}

/* Once every decision is staged in nxt[], apply them all at once. Pairs with
 * snapshot_state above. */
static inline void commit_state(EpiState *epi, const SIR *in)
{
    memcpy(epi->state, in, sizeof epi->state);
}

/* Work out everyone's next state into nxt[]. Only sick people act (recover,
 * infect neighbours); the healthy and recovered just sit there until a sick
 * neighbour reaches them. */
static void compute_next_state(Scene *sc, SIR *nxt)
{
    for (int i = 0; i < N_NODES; i++) {
        if (sc->epi.state[i] != I_STATE) continue;
        tick_one_infected(i, sc, &sc->epi, nxt);
    }
}

/* Tally S/I/R right now and push them onto the chart history. Called after the
 * states are committed, so the numbers reflect this tick. */
static void record_current_counts(Scene *sc)
{
    int s = count_state(&sc->epi, S_STATE);
    int i = count_state(&sc->epi, I_STATE);
    int r = count_state(&sc->epi, R_STATE);
    history_record(&sc->history, s, i, r);
}

/* Advance the disease one tick: age the flashes, photograph the current
 * states, decide everyone's next move from that photo, apply them all at once,
 * then record the new counts. The photo-then-apply order is what keeps the
 * result fair regardless of who is processed first. Does nothing while paused. */
static void sir_tick(Scene *sc)
{
    if (sc->paused) return;

    decrement_flash_counters(&sc->epi);

    SIR nxt[N_NODES];
    snapshot_state    (&sc->epi, nxt);
    compute_next_state(sc,       nxt);
    commit_state      (&sc->epi, nxt);

    sc->tick++;
    record_current_counts(sc);
}

/* ── §7 draw — paint the network, the epidemic chart, and the HUD ── */

/* Draw a straight line of characters between two cells, picking a glyph that
 * matches its slope ('-' flat, '|' upright, '/' or '\' diagonal). Integer-only
 * line stepping (Bresenham 1965). */
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

/* ── left panel: the network ── */

/* How one node should look: which colour, which character, and any extra
 * attribute (bold). Bundling the three keeps the "what does state X look like"
 * decision in one place (pick_node_glyph) and out of the drawing loop. */
typedef struct {
    int    cp;      /* colour-pair index (CP_S, CP_I, ...) */
    chtype ch;      /* the character to draw               */
    attr_t extra;   /* A_BOLD, or 0 for none               */
} NodeGlyph;

/* Pick the look for node i: grey '.' if healthy, bright '*' if just infected,
 * red '@' if settled-infected, green '+' if recovered. This is the on-screen
 * legend the HUD spells out. */
static NodeGlyph pick_node_glyph(const EpiState *epi, int i)
{
    switch (epi->state[i]) {
    case S_STATE: return (NodeGlyph){ CP_S, '.', 0 };
    case I_STATE: return epi->flash[i] > 0
                       ? (NodeGlyph){ CP_I_FLASH, '*', A_BOLD }
                       : (NodeGlyph){ CP_I,       '@', A_BOLD };
    case R_STATE: return (NodeGlyph){ CP_R, '+', 0 };
    default:      return (NodeGlyph){ CP_S, '.', 0 };
    }
}

/* Colour a link by how "live" it is. If either end is infected the link is
 * active: bright yellow if it's a shortcut, red if it's an ordinary ring link.
 * Otherwise dim grey background. Yellow shortcuts jumping across the ring are
 * the visible sign of the disease taking a long-range hop. */
static attr_t pick_edge_attr(const Scene *sc, int i, int j)
{
    bool hot = (sc->epi.state[i] == I_STATE || sc->epi.state[j] == I_STATE);
    if (!hot)                          return COLOR_PAIR(CP_EDGE_DIM);
    if (sc->topology.rewired[i][j])    return COLOR_PAIR(CP_EDGE_REWIRE) | A_BOLD;
    return                                    COLOR_PAIR(CP_EDGE_HOT)    | A_BOLD;
}

/* Draw all the links, before the nodes so the nodes sit on top. Only j > i so
 * each link is drawn once. */
static void draw_network_edges(const Scene *sc)
{
    int net_w = (int)(sc->cols * NET_FRAC);
    for (int i = 0; i < N_NODES; i++)
        for (int j = i + 1; j < N_NODES; j++) {
            if (!sc->topology.adj[i][j]) continue;
            draw_line(px_col(sc->pos[i].x), px_row(sc->pos[i].y),
                      px_col(sc->pos[j].x), px_row(sc->pos[j].y),
                      pick_edge_attr(sc, i, j),
                      net_w, sc->rows - HUD_BOT_ROWS);
        }
}

/* Draw each node on top of the links, skipping any that fall outside the
 * left panel or behind the HUD bars. */
static void draw_network_nodes(const Scene *sc)
{
    int net_w = (int)(sc->cols * NET_FRAC);
    for (int i = 0; i < N_NODES; i++) {
        int c = px_col(sc->pos[i].x);
        int r = px_row(sc->pos[i].y);
        if (c < 0 || c >= net_w)                              continue;
        if (r < HUD_TOP_ROWS || r >= sc->rows - HUD_BOT_ROWS) continue;
        NodeGlyph g = pick_node_glyph(&sc->epi, i);
        attron(COLOR_PAIR(g.cp) | g.extra);
        mvaddch(r, c, g.ch);
        attroff(COLOR_PAIR(g.cp) | g.extra);
    }
}

/* The whole left panel: links first, then nodes over them. */
static void draw_network(const Scene *sc)
{
    draw_network_edges(sc);
    draw_network_nodes(sc);
}

/* ── right panel: the epidemic chart ── */

/* Pre-computed screen rectangles for this frame's chart, worked out once and
 * handed to all the chart-drawing helpers so they don't each redo the math (or
 * disagree about it). The screen reads left to right: network panel | divider |
 * Y-axis labels | the bars. The `valid` flag is the polite handling of a too-
 * small terminal: if the chart can't fit, draw_chart just skips it and the
 * network panel still draws. */
typedef struct {
    int  net_w;        /* column of the divider (right edge of the network) */
    int  chart_x;      /* first column past the divider                     */
    int  chart_top;    /* first row of the chart (just below the top HUD)   */
    int  chart_h;      /* chart height in rows                              */
    int  data_x;       /* first column of the bars (past the axis labels)   */
    int  data_w;       /* width of the bar area in columns                  */
    int  data_top;     /* first row of the bars (below the title row)       */
    int  data_h;       /* height of the bar area in rows                    */
    bool valid;        /* false = terminal too small; skip the chart        */
} ChartLayout;

/* Split the screen left/right: network on the left, chart on the right. Bails
 * (returns false) if the chart would be under 8 columns wide. */
static bool split_panels_horizontally(ChartLayout *L, int cols)
{
    L->net_w   = (int)(cols * NET_FRAC);
    L->chart_x = L->net_w + 1;
    int chart_w = cols - L->chart_x;
    return chart_w >= 8;
}

/* Fit the chart into the rows between the two HUD bars. Bails if fewer than 4
 * rows are left (no room for title + bars). */
static bool set_chart_vertical_extent(ChartLayout *L, int rows)
{
    L->chart_top = HUD_TOP_ROWS;
    L->chart_h   = rows - L->chart_top - HUD_BOT_ROWS;
    return L->chart_h >= 4;
}

/* Reserve 3 columns at the left of the chart for the Y-axis numbers; what's
 * left is the bar area. Bails if under 4 columns remain for bars. */
static bool reserve_y_axis_columns(ChartLayout *L, int cols)
{
    const int y_lbl_w = 3;          /* "40 ", "20 ", " 0" */
    L->data_x = L->chart_x + y_lbl_w;
    L->data_w = (cols - L->chart_x) - y_lbl_w;
    return L->data_w >= 4;
}

/* Reserve one row at the top for the title; the rest is bar height. Bails if
 * under 2 rows remain for bars. */
static bool reserve_title_row(ChartLayout *L)
{
    L->data_top = L->chart_top + 1;
    L->data_h   = L->chart_h - 1;
    return L->data_h >= 2;
}

/* Run the four sizing steps in order. If any step says "doesn't fit", we leave
 * valid = false (its initial value) and stop, and draw_chart skips the panel. */
static ChartLayout compute_chart_layout(int rows, int cols)
{
    ChartLayout L = { .valid = false };
    if (!split_panels_horizontally(&L, cols))   return L;
    if (!set_chart_vertical_extent(&L, rows))   return L;
    if (!reserve_y_axis_columns   (&L, cols))   return L;
    if (!reserve_title_row        (&L))         return L;
    L.valid = true;
    return L;
}

/* The vertical line splitting the network panel from the chart panel. */
static void draw_chart_divider(const Scene *sc, const ChartLayout *L)
{
    attron(COLOR_PAIR(CP_DIVIDER));
    for (int r = L->chart_top; r < sc->rows - HUD_BOT_ROWS; r++)
        mvaddch(r, L->net_w, ACS_VLINE);
    attroff(COLOR_PAIR(CP_DIVIDER));
}

/* The "EPIDEMIC CURVE" heading above the bars. */
static void draw_chart_title(const ChartLayout *L)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(L->chart_top, L->chart_x, " EPIDEMIC CURVE");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* The Y-axis tick numbers (0 up to N), marking node counts. */
static void draw_chart_axis_labels(const ChartLayout *L)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(L->data_top,                  L->chart_x, "%2d", N_NODES);
    mvprintw(L->data_top + L->data_h/4,    L->chart_x, "%2d", N_NODES*3/4);
    mvprintw(L->data_top + L->data_h/2,    L->chart_x, "%2d", N_NODES/2);
    mvprintw(L->data_top + L->data_h*3/4,  L->chart_x, "%2d", N_NODES/4);
    mvprintw(L->data_top + L->data_h - 1,  L->chart_x, " 0");
    attroff(COLOR_PAIR(CP_HUD));
}

/* The bars themselves: one column per past tick, newest on the right. Each
 * column is stacked recovered (green, bottom), infected (red, middle),
 * susceptible (grey, top), with each band's height scaled to the node count. */
static void draw_chart_stacked_bars(const Scene *sc, const ChartLayout *L)
{
    int n_show = sc->history.n < L->data_w ? sc->history.n : L->data_w;
    if (n_show == 0) return;

    for (int cx = 0; cx < n_show; cx++) {
        int bi = (sc->history.head - n_show + cx + HIST_LEN) % HIST_LEN;
        int hs = sc->history.s[bi];
        int hi = sc->history.i[bi];
        int hr = sc->history.r[bi];

        int scol = L->data_x + (L->data_w - n_show) + cx;
        if (scol < L->data_x || scol >= sc->cols) continue;

        int rh = hr * L->data_h / N_NODES;
        int ih = hi * L->data_h / N_NODES;
        int sh = hs * L->data_h / N_NODES;

        for (int rb = 0; rb < L->data_h; rb++) {
            int srow = L->data_top + (L->data_h - 1 - rb);
            if (srow < 0 || srow >= sc->rows) continue;

            int    cp;
            chtype ch;
            attr_t ex = 0;
            if      (rb < rh)             { cp=CP_BAR_R; ch='-';            }
            else if (rb < rh + ih)        { cp=CP_BAR_I; ch='#'; ex=A_BOLD; }
            else if (rb < rh + ih + sh)   { cp=CP_BAR_S; ch='=';            }
            else                          { continue;                       }

            attron(COLOR_PAIR(cp) | ex);
            mvaddch(srow, scol, ch);
            attroff(COLOR_PAIR(cp) | ex);
        }
    }
}

/* A small "pk" tag at the height of the worst-ever infected count, so you can
 * see how bad the peak got even after it passes. Dim so it reads as a note. */
static void draw_chart_peak_marker(const Scene *sc, const ChartLayout *L)
{
    if (sc->history.peak_i <= 0) return;
    int peak_row = L->data_top + L->data_h - 1
                 - sc->history.peak_i * L->data_h / N_NODES;
    if (peak_row < L->data_top || peak_row >= L->data_top + L->data_h) return;
    attron(COLOR_PAIR(CP_I) | A_DIM);
    mvprintw(peak_row, L->chart_x, "pk");
    attroff(COLOR_PAIR(CP_I) | A_DIM);
}

/* The whole right panel: lay it out, then (if it fits) draw divider, title,
 * axis, bars, and peak marker. */
static void draw_chart(const Scene *sc)
{
    ChartLayout L = compute_chart_layout(sc->rows, sc->cols);
    if (!L.valid) return;
    draw_chart_divider     (sc, &L);
    draw_chart_title       (&L);
    draw_chart_axis_labels (&L);
    draw_chart_stacked_bars(sc, &L);
    draw_chart_peak_marker (sc, &L);
}

/* ── HUD (top rows: live numbers, bottom row: key hints) ── */

/* A one-word label for where the outbreak is right now: READY (not started),
 * SEEDED (just started), GROWING / WANING / PLATEAU (by comparing the infected
 * count to last tick's), or EXTINCT (everyone's recovered). */
static const char *detect_epidemic_phase(const Scene *sc, int ni, int nr)
{
    if (ni == 0 && nr == 0) return "READY  ";
    if (ni == 0)            return "EXTINCT";
    if (sc->history.n < 2)  return "SEEDED ";
    int prev_i = history_prev_infected(&sc->history);
    if (ni > prev_i) return "GROWING";
    if (ni < prev_i) return "WANING ";
    return                  "PLATEAU";
}

/* The two rates the user dials with the arrow keys: infection (beta) and
 * recovery (gamma). Everything else on the top row is computed from these. */
static void draw_hud_rates(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, " β=%.3f  γ=%.3f ", sc->beta, sc->gamma);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* R0: roughly how many people one sick person infects before recovering. Above
 * 1 the disease takes off (shown red), at-or-below 1 it dies out (green). Drag
 * the rates and watch the colour flip as R0 crosses 1 — that's the tipping
 * point. (Anderson & May 1991 for the formula behind it.) */
static void draw_hud_r0(float r0)
{
    attr_t a = (r0 > 1.f) ? (COLOR_PAIR(CP_I) | A_BOLD)
                          : (COLOR_PAIR(CP_R) | A_BOLD);
    attron(a);
    mvprintw(0, 18, "R0=%-4.2f", r0);
    attroff(a);
}

/* The rest of the top row: average links per person, tick count, the S/I/R
 * tallies, the phase label, and a PAUSED marker. */
static void draw_hud_summary(const Scene *sc, float mk, int ns, int ni, int nr,
                             const char *phase)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 27, " <k>=%.1f  tick=%-4d  S=%-3d I=%-3d R=%-3d  %s%s",
             mk, sc->tick, ns, ni, nr,
             phase,
             sc->paused ? "  [PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Draw one coloured chunk of the horizontal proportion bar, its length set by
 * count's share of the total. Returns where the next chunk should start, so
 * the caller can lay them end to end. */
static int draw_bar_segment(int row, int bx, int count, int total, int bar_w,
                            int max_col, int cp, attr_t extra, chtype glyph)
{
    int fill = count * bar_w / total;
    attron(COLOR_PAIR(cp) | extra);
    for (int i = 0; i < fill && bx + i < max_col; i++)
        mvaddch(row, bx + i, glyph);
    attroff(COLOR_PAIR(cp) | extra);
    return bx + fill;
}

/* The coloured "S I R" key that says which colour is which in the bar. */
static void draw_segment_legend(int row, int bx)
{
    attron(COLOR_PAIR(CP_BAR_S));        mvprintw(row, bx+2, "S"); attroff(COLOR_PAIR(CP_BAR_S));
    attron(COLOR_PAIR(CP_BAR_I)|A_BOLD); mvprintw(row, bx+4, "I"); attroff(COLOR_PAIR(CP_BAR_I)|A_BOLD);
    attron(COLOR_PAIR(CP_BAR_R));        mvprintw(row, bx+6, "R"); attroff(COLOR_PAIR(CP_BAR_R));
}

/* The ".=S @=I +=R" key explaining the node glyphs in the network panel
 * (separate from the bar-colour key above). */
static void draw_glyph_key(int row, int bx)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(row, bx+9, " key: ");
    attroff(COLOR_PAIR(CP_HUD));
    attron(COLOR_PAIR(CP_S));        mvprintw(row, bx+15, ".=S "); attroff(COLOR_PAIR(CP_S));
    attron(COLOR_PAIR(CP_I)|A_BOLD); mvprintw(row, bx+19, "@=I "); attroff(COLOR_PAIR(CP_I)|A_BOLD);
    attron(COLOR_PAIR(CP_R));        mvprintw(row, bx+23, "+=R");  attroff(COLOR_PAIR(CP_R));
}

/* Row 1: a single horizontal bar split into S / I / R chunks showing what
 * fraction of the population is in each state right now, with both keys after
 * it. */
static void draw_hud_proportion_bar(const Scene *sc, int ns, int ni, int nr)
{
    int bar_w = sc->cols - 22;
    if (bar_w < 6) bar_w = 6;

    mvprintw(1, 0, " ");
    int bx = 1;
    bx = draw_bar_segment(1, bx, ns, N_NODES, bar_w, sc->cols, CP_BAR_S, 0,      '=');
    bx = draw_bar_segment(1, bx, ni, N_NODES, bar_w, sc->cols, CP_BAR_I, A_BOLD, '#');
    bx = draw_bar_segment(1, bx, nr, N_NODES, bar_w, sc->cols, CP_BAR_R, 0,      '-');

    draw_segment_legend(1, bx);
    draw_glyph_key     (1, bx);
}

/* The whole top status area: compute the live numbers, then draw the rates,
 * R0, the summary line, and the proportion bar. */
static void draw_hud_top(const Scene *sc)
{
    int ns = count_state(&sc->epi, S_STATE);
    int ni = count_state(&sc->epi, I_STATE);
    int nr = count_state(&sc->epi, R_STATE);
    float mk = topology_mean_degree(&sc->topology);
    float r0 = (sc->gamma > 0.f) ? sc->beta * mk / sc->gamma : 0.f;
    const char *phase = detect_epidemic_phase(sc, ni, nr);

    draw_hud_rates          (sc);
    draw_hud_r0             (r0);
    draw_hud_summary        (sc, mk, ns, ni, nr, phase);
    draw_hud_proportion_bar (sc, ns, ni, nr);
}

/* The bottom row: the list of keys you can press. */
static void draw_hud_bottom(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HINT)|A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  i:inject  up/dn:β  lt/rt:γ ");
    attroff(COLOR_PAIR(CP_HINT)|A_BOLD);
}

/* Paint one whole frame: top HUD, then the network, the chart, and the key
 * hints. */
static void scene_draw(const Scene *sc)
{
    draw_hud_top   (sc);
    draw_network   (sc);
    draw_chart     (sc);
    draw_hud_bottom(sc);
}

/* ── §8 app — startup, the main loop, input handling, cleanup ── */

/* Set by signal handlers, read by the main loop. volatile sig_atomic_t is the
 * only type a handler may safely touch. */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s==SIGINT||s==SIGTERM) g_quit=1;
    if (s==SIGWINCH)           g_resize=1;
}
static void cleanup(void) { endwin(); }

/* Keep a value between 0 and 1 — beta and gamma are probabilities. */
static inline float clamp01(float v)
{
    if (v < 0.f) return 0.f;
    if (v > 1.f) return 1.f;
    return v;
}

/* Act on one keypress: quit, pause, reset, inject, or nudge a rate. Quit goes
 * through g_quit so 'q' and Ctrl-C take the same exit path. */
static void handle_input(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit       = 1;             break;
    case ' ':                    sc->paused   = !sc->paused;   break;
    case 'r': case 'R':          epi_reset(sc);                break;
    case 'i': case 'I':          epi_inject(&sc->epi);         break;
    case KEY_UP:    sc->beta  = clamp01(sc->beta  + BETA_STEP);  break;
    case KEY_DOWN:  sc->beta  = clamp01(sc->beta  - BETA_STEP);  break;
    case KEY_RIGHT: sc->gamma = clamp01(sc->gamma + GAMMA_STEP); break;
    case KEY_LEFT:  sc->gamma = clamp01(sc->gamma - GAMMA_STEP); break;
    default: break;
    }
}

/* ── startup and main loop ── */

/* Seed the random number generator from the clock so each run differs. */
static void init_random_seed(void)
{
    srand((unsigned)(clock_ns() & 0xFFFFFFFF));
}

/* Hook up the signals: Ctrl-C / kill ask the loop to quit, a window resize
 * asks it to relayout, and atexit makes sure the terminal is restored even on
 * a crash. */
static void register_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);
}

/* Start ncurses the way this demo wants it: raw unbuffered keys, no echo,
 * arrow keys recognised, non-blocking input, hidden cursor, and colour on.
 * typeahead(-1) stops ncurses pausing the redraw to peek at input (avoids
 * tearing). */
static void init_ncurses_session(void)
{
    initscr();
    cbreak(); noecho(); keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE); curs_set(0); typeahead(-1);
    color_init();
}

/* Set the starting rates, build the network once, place it, and seed the first
 * infection. The network lasts the whole run; only the layout is redone on
 * resize and only the disease is wiped on reset. */
static void init_scene(Scene *sc)
{
    sc->beta   = BETA_INIT;
    sc->gamma  = GAMMA_INIT;
    sc->paused = false;
    sc->tick   = 0;
    getmaxyx(stdscr, sc->rows, sc->cols);
    topology_build_ws(&sc->topology);
    layout_ring(sc);
    epi_reset(sc);
}

/* On a window resize, tell ncurses the new size and replace the node positions
 * to match. The disease keeps running; only the picture is refitted. */
static void handle_resize(Scene *sc)
{
    g_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
    layout_ring(sc);
}

/*
 * Draw and show one frame. erase clears the back buffer, scene_draw paints it,
 * and wnoutrefresh+doupdate push a single minimal update to the terminal (one
 * write per frame, the standard ncurses way to avoid flicker).
 */
static void render_one_frame(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Wait out whatever's left of this frame's time budget to hold the target FPS.
 * If the frame already ran long, the sleep is zero — no catch-up spiral. */
static void sleep_to_frame_deadline(long long t0, long long frame_ns)
{
    clock_sleep_ns(frame_ns - (clock_ns() - t0));
}

/* Set everything up, then loop until quit: handle resize, read a key, advance
 * the disease one tick, draw, and sleep to keep a steady frame rate. */
int main(void)
{
    init_random_seed();
    register_signal_handlers();
    init_ncurses_session();

    Scene scene = { 0 };
    init_scene(&scene);

    long long frame_ns = 1000000000LL / FPS;

    while (!g_quit) {
        if (g_resize) handle_resize(&scene);
        handle_input(&scene, getch());

        long long t0 = clock_ns();
        sir_tick(&scene);
        render_one_frame(&scene);
        sleep_to_frame_deadline(t0, frame_ns);
    }
    return 0;
}
