/*
 * bonsai.c  —  growing bonsai tree in the terminal
 *
 * A feature-complete cbonsai-style program built on the same ncurses
 * rendering framework as bounce.c and matrix_rain.c:
 *   - dt loop, CLOCK_MONOTONIC, fixed-timestep accumulator
 *   - single stdscr / erase / wnoutrefresh / doupdate flush
 *   - SIGWINCH resize, SIGINT/SIGTERM clean exit, atexit cleanup
 *
 * Features
 * --------
 *   Live growth animation  — watch the tree grow step by step
 *   Infinite mode          — regrow a new tree after each one finishes
 *   Screensaver mode       — live + infinite, any key quits
 *   5 tree types           — t key cycles: random / dwarf / weeping /
 *                            sparse / bamboo
 *   2 pot styles           — b key cycles: big / small / none
 *   Message box            — m key toggles a centred message panel
 *   Leaves                 — configurable leaf chars, random per branch
 *   Branch multiplier      — M / N keys (more / fewer branches)
 *   Life                   — L / K keys (longer / shorter tree)
 *   Speed                  — ] / [ keys (faster / slower growth)
 *   Seed control           — r key re-seeds and resets
 *   Pause                  — space
 *   Colour themes          — 256-colour with 8-colour fallback
 *
 * Runtime keys
 * ------------
 *   t          cycle tree type
 *   b          cycle pot style
 *   m          toggle message panel
 *   M / N      more / fewer branches (multiplier)
 *   L / K      longer / shorter tree (life)
 *   ] / [      faster / slower growth
 *   r          new random seed → restart
 *   space      pause / resume
 *   q / ESC    quit
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra bonsai.c -o bonsai -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config        — all tunable constants
 *   §2  clock         — monotonic ns clock + sleep
 *   §3  color         — color pairs, branch/leaf palette
 *   §4  pot           — ASCII art pot rendering
 *   §5  branch        — branch struct, step algorithm, char selection
 *   §6  tree          — branch pool, growth tick, leaf scatter
 *   §7  message       — bordered message box
 *   §8  scene         — owns tree + settings, drives render
 *   §9  screen        — ncurses init/resize/present
 *   §10 app           — dt loop, input, SIGWINCH, cleanup
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

enum {
    /* Growth simulation */
    LIFE_DEFAULT       = 120,   /* steps a trunk branch lives          */
    LIFE_MIN           =  10,
    LIFE_MAX           = 200,
    MULT_DEFAULT       =   8,   /* branching multiplier 0-20           */
    MULT_MIN           =   0,
    MULT_MAX           =  20,

    /* Speed: growth steps per second */
    GROW_FPS_DEFAULT   =  30,
    GROW_FPS_MIN       =   2,
    GROW_FPS_MAX       = 120,
    GROW_FPS_STEP      =   5,

    /* Branch pool */
    BRANCH_MAX         = 1024,

    /* Infinite / screensaver wait after tree finishes (ms) */
    INFINITE_WAIT_MS   = 4000,

    /* Message box */
    MSG_PAD            =   2,   /* padding inside message border       */
    MSG_MAX            = 128,

    /* HUD */
    HUD_COLS           =  50,
    FPS_UPDATE_MS      = 500,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

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
    struct timespec r = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Color pair assignments:
 *   1  dark wood    (trunk, thick branches)
 *   2  light wood   (thin branches)
 *   3  dark leaf    (dense foliage)
 *   4  light leaf   (sparse foliage, tips)
 *   5  pot / base   (ceramic colour)
 *   6  message bg   (message box border + text)
 *   7  HUD          (status bar)
 */

static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        /* 256-colour: rich brown trunk, varied greens */
        init_pair(1, 130, -1);   /* dark brown  — trunk         */
        init_pair(2, 172, -1);   /* amber brown — branches      */
        init_pair(3,  28, -1);   /* dark green  — dense leaves  */
        init_pair(4,  82, -1);   /* lime green  — light leaves  */
        init_pair(5, 180, -1);   /* tan         — pot           */
        init_pair(6, 250, -1);   /* light grey  — message       */
        init_pair(7, 226, -1);   /* yellow      — HUD           */
    } else {
        /* 8-colour fallback */
        init_pair(1, COLOR_RED,     -1);
        init_pair(2, COLOR_YELLOW,  -1);
        init_pair(3, COLOR_GREEN,   -1);
        init_pair(4, COLOR_GREEN,   -1);
        init_pair(5, COLOR_YELLOW,  -1);
        init_pair(6, COLOR_WHITE,   -1);
        init_pair(7, COLOR_YELLOW,  -1);
    }
}

/* ===================================================================== */
/* §4  pot                                                                */
/* ===================================================================== */

typedef enum { POT_BIG=0, POT_SMALL, POT_NONE, POT_COUNT } PotType;
static const char *k_pot_names[] = { "big", "small", "none" };

/*
 * Pot art — drawn centred at (cx, base_row).
 *
 * All strings in each array share the same maxw so draw_pot's
 * left-edge calculation (lx = cx - maxw/2) keeps every line
 * visually centred.  Each row's wall characters sit at the
 * same column positions as the rows above and below.
 *
 * POT_BIG  (17-char max, walls at col 1 and col 16):
 *   row 0  __ rim top  __ (cols 2-15, inset 1 from walls)
 *   row 1  /  rim arc  \  (cols 1-16, outermost)
 *   row 2  |  body     |  (cols 1-16, same as rim arc)
 *   row 3  |  body     |
 *   row 4  \_ base arc_/  (cols 2-15, inset 1, tapers inward)
 *
 * POT_SMALL (10-char max, walls at col 0 and col 9):
 *   row 0   _ rim top _   (cols 1-8, inset 1)
 *   row 1  /  body    \   (cols 0-9, outermost)
 *   row 2  |  body    |   (cols 0-9)
 *   row 3  \_ base   _/   (cols 1-8, inset 1)
 */
static const char *k_pot_big[] = {
    "  ______________",
    " /              \\",
    " |              |",
    " |              |",
    "  \\____________/",
    NULL
};

static const char *k_pot_small[] = {
    " ________",
    "/        \\",
    "|        |",
    " \\______/",
    NULL
};

/*
 * draw_pot — render the pot centred horizontally at cx, bottom at row.
 * Returns the row of the topmost pot line (= where trunk starts).
 */
static int draw_pot(int cx, int rows, PotType type)
{
    if (type == POT_NONE) return rows - 1;

    const char **lines = (type == POT_BIG) ? k_pot_big : k_pot_small;
    int n = 0;
    while (lines[n]) n++;

    /* measure widest line for centering */
    int maxw = 0;
    for (int i = 0; i < n; i++) {
        int w = (int)strlen(lines[i]);
        if (w > maxw) maxw = w;
    }

    int top_row = rows - n;
    attron(COLOR_PAIR(5) | A_BOLD);
    for (int i = 0; i < n; i++) {
        int lx = cx - maxw/2;
        if (lx < 0) lx = 0;
        mvprintw(top_row + i, lx, "%s", lines[i]);
    }
    attroff(COLOR_PAIR(5) | A_BOLD);

    return top_row;   /* trunk initialises here; first draw lands at top_row-1 (rim-adjacent) */
}

/* ===================================================================== */
/* §5  branch                                                             */
/* ===================================================================== */

typedef enum {
    BR_TRUNK  = 0,
    BR_LEFT   = 1,
    BR_RIGHT  = 2,
    BR_DYING  = 3,
    BR_DEAD   = 4,
} BranchType;

/*
 * Tree types — control the growth algorithm weights:
 *
 *   TREE_RANDOM   — cbonsai default: balanced trunk + spread branches
 *   TREE_DWARF    — very short life, dense branching, compact
 *   TREE_WEEPING  — branches strongly prefer downward drift
 *   TREE_SPARSE   — low multiplier, long life, open canopy
 *   TREE_BAMBOO   — trunk grows almost straight up, thin side shoots
 */
typedef enum {
    TREE_RANDOM=0, TREE_DWARF, TREE_WEEPING, TREE_SPARSE, TREE_BAMBOO,
    TREE_COUNT
} TreeType;
static const char *k_tree_names[] = {
    "random","dwarf","weeping","sparse","bamboo"
};

/*
 * Branch — one growing segment of the tree.
 *
 * x, y      current cell position
 * life      steps remaining before this branch dies
 * type      trunk / left / right / dying
 * dx, dy    current direction (−1/0/+1 each)
 * age       steps taken so far (used for character selection)
 */
typedef struct {
    int        x, y;
    int        life;
    BranchType type;
    int        dx, dy;
    int        age;
    bool       alive;
} Branch;

/*
 * branch_char — choose the ASCII character for a branch segment.
 *
 * Based on direction and type, matching cbonsai's character choices:
 *   trunk rising straight   → |  (pipe)
 *   trunk leaning           → /\ (slashes)
 *   thin diagonal branch    → /\ or ~  (wiggle for dying)
 *   thick horizontal        → _ (underscore)
 *   dying / very thin       → ~ (tilde)
 */
static char branch_char(int dx, int dy, BranchType type, int age)
{
    if (type == BR_DYING) {
        /* dying: sparse, wiggly */
        const char *dying = "~";
        return dying[0];
    }

    /* trunk near base: thick chars */
    if (type == BR_TRUNK && age < 4) {
        if      (dy == -1 && dx ==  0) return '|';
        else if (dy == -1 && dx ==  1) return '\\';
        else if (dy == -1 && dx == -1) return '/';
        else if (dy ==  0 && dx ==  1) return '_';
        else if (dy ==  0 && dx == -1) return '_';
    }

    if (dy == -1) {
        if      (dx ==  0) return '|';
        else if (dx ==  1) return '\\';
        else if (dx == -1) return '/';
    } else if (dy == 0) {
        if      (dx ==  1) return '_';
        else if (dx == -1) return '_';
        else               return '-';
    } else {
        /* downward (weeping) */
        if      (dx ==  1) return '\\';
        else if (dx == -1) return '/';
        else               return '|';
    }
    return '?';
}

/*
 * branch_color — color pair for a branch segment.
 *
 * Young trunk → dark wood (pair 1, bold).
 * Older trunk + thick branches → light wood (pair 2).
 * Thin / dying → light wood, no bold.
 */
static int branch_color(BranchType type, int life, int life_max)
{
    if (type == BR_TRUNK)
        return (life > life_max * 2/3) ? 1 : 2;
    if (type == BR_DYING) return 2;
    return (life > 8) ? 2 : 2;
}

static bool branch_bold(BranchType type, int life, int life_max)
{
    if (type == BR_TRUNK && life > life_max * 2/3) return true;
    if (type == BR_DYING) return false;
    return (life > life_max / 2);
}

/* ===================================================================== */
/* §6  tree                                                               */
/* ===================================================================== */

/*
 * Leaf characters — randomly chosen when a branch dies.
 * Custom sets can be cycled with the 'l' key.
 */
static const char *k_leaf_sets[] = {
    "&",
    "*",
    "@",
    "#",
    "~",
    "&*@",
    "+",
};
#define LEAF_SETS  (int)(sizeof k_leaf_sets / sizeof k_leaf_sets[0])

typedef struct {
    Branch  pool[BRANCH_MAX];
    int     n;              /* active branch count               */
    bool    growing;        /* false = tree finished             */
    int     shoots;         /* live branch count this generation */
    int     shoot_max;      /* max shoots = multiplier           */
    int     leaf_set;       /* index into k_leaf_sets            */
    TreeType tree_type;
    int     life_start;     /* life given to trunk               */
    int     multiplier;
} Tree;

static void tree_reset(Tree *t, int cx, int cy,
                       TreeType type, int life, int mult)
{
    memset(t, 0, sizeof *t);
    t->tree_type  = type;
    t->life_start = life;
    t->multiplier = mult;
    t->shoot_max  = mult;
    t->growing    = true;

    /* seed trunk branch */
    t->pool[0] = (Branch){
        .x     = cx,
        .y     = cy,
        .life  = life,
        .type  = BR_TRUNK,
        .dx    = 0,
        .dy    = -1,
        .age   = 0,
        .alive = true,
    };
    t->n = 1;
}

/*
 * tree_new_branch — spawn a child branch from parent position.
 */
static void tree_new_branch(Tree *t, int x, int y,
                             int life, BranchType type, int dx, int dy)
{
    if (t->n >= BRANCH_MAX) return;
    t->pool[t->n++] = (Branch){
        .x=x,.y=y,.life=life,.type=type,
        .dx=dx,.dy=dy,.age=0,.alive=true
    };
    t->shoots++;
}

/*
 * next_dir — advance direction based on tree type and current state.
 *
 * Returns new (dx, dy).  The algorithm mirrors cbonsai's original
 * heuristic: biased random walk, type-specific weights.
 */
static void next_dir(TreeType type, BranchType btype,
                     int age, int life,
                     int cur_dx, int cur_dy,
                     int *out_dx, int *out_dy)
{
    int dx = cur_dx;
    int dy = cur_dy;

    /* random perturbation — direction drifts each step */
    int r = rand() % 10;

    switch (type) {

    case TREE_BAMBOO:
        /*
         * Bamboo: trunk almost always goes straight up.
         * Very slight horizontal drift (1-in-10 chance per step).
         */
        if (btype == BR_TRUNK) {
            dy = -1;
            dx = (r == 0) ? 1 : (r == 1) ? -1 : 0;
        } else {
            /* side shoots spread quickly then flatten */
            dx = (age < 3) ? cur_dx : 0;
            dy = (age < 2) ? -1 : 0;
        }
        break;

    case TREE_WEEPING:
        /*
         * Weeping: trunk goes up normally, but branches strongly
         * prefer to droop downward as they age.
         */
        if (btype == BR_TRUNK) {
            dy = -1;
            dx = (r < 4) ? 0 : (r < 7) ? 1 : -1;
        } else {
            /* branches droop: after a few steps go sideways then down */
            dy = (age < 2) ? -1 : (age < 5) ? 0 : 1;
            dx = cur_dx + (r < 3 ? 1 : r < 6 ? -1 : 0);
            if (dx >  1) dx =  1;
            if (dx < -1) dx = -1;
        }
        break;

    case TREE_DWARF:
        /*
         * Dwarf: compact, dense. Branches spread wide quickly.
         */
        if (btype == BR_TRUNK) {
            dy = -1;
            dx = (r < 5) ? 0 : (r < 8) ? 1 : -1;
        } else {
            dx = cur_dx + (r < 4 ? 1 : r < 8 ? -1 : 0);
            dy = (r < 6) ? -1 : 0;
            if (dx >  1) dx =  1;
            if (dx < -1) dx = -1;
        }
        break;

    case TREE_SPARSE:
        /*
         * Sparse: branches grow long and mostly upward.
         * Very little horizontal drift on trunk.
         */
        if (btype == BR_TRUNK) {
            dy = -1;
            dx = (r < 2) ? 1 : (r < 4) ? -1 : 0;
        } else {
            dx = cur_dx + (r < 2 ? 1 : r < 4 ? -1 : 0);
            dy = (r < 7) ? -1 : 0;
            if (dx >  1) dx =  1;
            if (dx < -1) dx = -1;
        }
        break;

    default: /* TREE_RANDOM — cbonsai default */
        if (btype == BR_TRUNK) {
            /* trunk drifts slowly: mostly up, slight horizontal wobble */
            if (r <= 2)      { dy = -1; dx =  0; }
            else if (r <= 4) { dy = -1; dx =  1; }
            else if (r <= 6) { dy = -1; dx = -1; }
            else if (r == 7) { dy =  0; dx =  1; }
            else             { dy =  0; dx = -1; }
        } else {
            /* branches: weighted toward spreading outward */
            int bias = (btype == BR_LEFT) ? -1 : 1;
            dx = cur_dx + bias * (r < 5 ? 0 : r < 8 ? 1 : -1);
            dy = (r < 5) ? -1 : (r < 8) ? 0 : (rand()%3==0 ? 1 : -1);
            if (dx >  2) dx =  2;
            if (dx < -2) dx = -2;
        }
        break;
    }

    *out_dx = dx;
    *out_dy = dy;
}

/*
 * tree_step — advance one growth tick.
 *
 * For each live branch:
 *   1. Compute next direction.
 *   2. Move position.
 *   3. Determine if it should spawn children.
 *   4. Decrement life; on death scatter leaves.
 *   5. Draw the character at the new position.
 *
 * Returns false when no branches are alive (tree done).
 */
static bool tree_step(Tree *t, int cols, int rows, int trunk_base_y)
{
    bool any_alive = false;
    int n_this_tick = t->n;   /* only step branches that existed at start */

    for (int i = 0; i < n_this_tick; i++) {
        Branch *b = &t->pool[i];
        if (!b->alive) continue;
        any_alive = true;

        /* advance direction */
        int nx, ny;
        next_dir(t->tree_type, b->type, b->age, t->life_start,
                 b->dx, b->dy, &nx, &ny);
        b->dx = nx;
        b->dy = ny;

        b->x  += b->dx;
        b->y  += b->dy;
        b->age++;
        b->life--;

        /* clamp to screen */
        if (b->x < 0)      b->x = 0;
        if (b->x >= cols)  b->x = cols - 1;
        if (b->y < 0)      b->y = 1;
        if (b->y >= rows)  b->y = rows - 1;

        /* draw branch character */
        BranchType draw_type = (b->life <= 3) ? BR_DYING : b->type;
        char ch = branch_char(b->dx, b->dy, draw_type, b->age);
        int  cp = branch_color(b->type, b->life, t->life_start);
        bool bo = branch_bold (b->type, b->life, t->life_start);

        attr_t attr = COLOR_PAIR(cp) | (bo ? A_BOLD : 0);
        attron(attr);
        mvaddch(b->y, b->x, (chtype)(unsigned char)ch);
        attroff(attr);

        /*
         * Branching logic — mirrors cbonsai:
         * Trunk re-branches when not close to ground AND either randomly
         * or upon every <multiplier> steps.
         */
        bool should_branch = false;

        if (b->type == BR_TRUNK) {
            int dist_from_base = trunk_base_y - b->y;
            if (dist_from_base > 3) {
                /* random branch: 1-in-multiplier chance per step */
                if (t->multiplier > 0 && (rand() % (MULT_MAX - t->multiplier + 2)) == 0)
                    should_branch = true;
                /* periodic branch every multiplier steps */
                if (t->multiplier > 0 && (b->age % (t->multiplier + 1)) == 0)
                    should_branch = true;
            }
        } else {
            /* side branches spawn near-tips if shoots budget allows */
            if (b->life > 3 && t->shoots < t->shoot_max * 3)
                if ((rand() % 8) == 0)
                    should_branch = true;
        }

        if (should_branch && t->n < BRANCH_MAX - 2) {
            int child_life = b->life / 2;
            if (child_life < 2) child_life = 2;
            /* spawn left and right children */
            tree_new_branch(t, b->x, b->y, child_life, BR_LEFT,  -1, -1);
            tree_new_branch(t, b->x, b->y, child_life, BR_RIGHT,  1, -1);
        }

        /* die */
        if (b->life <= 0) {
            b->alive = false;
            b->type  = BR_DEAD;

            /* scatter leaves at death position */
            const char *lset = k_leaf_sets[t->leaf_set % LEAF_SETS];
            int llen = (int)strlen(lset);

            int n_leaves = 3 + rand() % 4;
            for (int li = 0; li < n_leaves; li++) {
                int lx = b->x + (rand() % 5) - 2;
                int ly = b->y + (rand() % 3) - 1;
                if (lx < 0 || lx >= cols) continue;
                if (ly < 1 || ly >= rows) continue;

                char lch = lset[rand() % llen];
                int  lcp = (rand() & 1) ? 3 : 4;  /* dark or light leaf */
                attron(COLOR_PAIR(lcp) | A_BOLD);
                mvaddch(ly, lx, (chtype)(unsigned char)lch);
                attroff(COLOR_PAIR(lcp) | A_BOLD);
            }
        }
    }

    return any_alive;
}

/* ===================================================================== */
/* §7  message                                                            */
/* ===================================================================== */

/*
 * draw_message — draw a bordered message box.
 *
 * Positioned at (4, 2) top-left, matching cbonsai's default placement.
 * Wraps long messages at a fixed column width.
 */
static void draw_message(const char *msg, int cols, int rows)
{
    if (!msg || !msg[0]) return;

    int mlen = (int)strlen(msg);
    int max_w = (cols / 2) - 4;
    if (max_w < 10) max_w = 10;
    if (mlen < max_w) max_w = mlen;

    int box_w = max_w + MSG_PAD * 2 + 2;  /* +2 for border chars */
    int box_h = 3;                         /* border + text + border */
    int bx = 4, by = 2;
    if (bx + box_w >= cols) bx = 0;
    if (by + box_h >= rows) by = 0;

    attron(COLOR_PAIR(6) | A_BOLD);

    /* top border */
    mvaddch(by, bx, ACS_ULCORNER);
    for (int i = 1; i < box_w - 1; i++) mvaddch(by, bx+i, ACS_HLINE);
    mvaddch(by, bx + box_w - 1, ACS_URCORNER);

    /* middle row with message */
    mvaddch(by+1, bx, ACS_VLINE);
    for (int i = 0; i < MSG_PAD; i++) mvaddch(by+1, bx+1+i, ' ');
    int tx = bx + 1 + MSG_PAD;
    for (int i = 0; i < max_w && msg[i]; i++)
        mvaddch(by+1, tx+i, (chtype)(unsigned char)msg[i]);
    for (int i = 0; i < MSG_PAD; i++) mvaddch(by+1, tx+max_w+i, ' ');
    mvaddch(by+1, bx + box_w - 1, ACS_VLINE);

    /* bottom border */
    mvaddch(by+2, bx, ACS_LLCORNER);
    for (int i = 1; i < box_w - 1; i++) mvaddch(by+2, bx+i, ACS_HLINE);
    mvaddch(by+2, bx + box_w - 1, ACS_LRCORNER);

    attroff(COLOR_PAIR(6) | A_BOLD);
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — owns all runtime state.
 *
 * The tree grows on treeWin-equivalent: stdscr with erase each regrow.
 * We repaint the static elements (pot, message, HUD) every render frame
 * and only call tree_step() when the growth accumulator fires.
 */
typedef struct {
    Tree      tree;
    TreeType  tree_type;
    PotType   pot_type;
    int       multiplier;
    int       life;
    int       grow_fps;
    int       leaf_set;

    bool      live_mode;    /* animate growth step by step              */
    bool      infinite;     /* regrow when tree finishes                */
    bool      screensaver;  /* live+infinite, any key quits             */
    bool      show_msg;     /* message box visible                      */
    char      message[MSG_MAX];

    unsigned int seed;
    bool      paused;
    bool      done;         /* tree finished growing                    */
    bool      growing;

    /* regrow wait state */
    int64_t   wait_accum;   /* ns accumulated waiting after tree done   */

    /* trunk init row = pot's top row; first drawn char lands one row above */
    int       trunk_base_y;
    int       trunk_cx;     /* centre x */
} Scene;

static void scene_plant(Scene *s, int cols, int rows)
{
    /*
     * Plant a new tree.
     * trunk_cx: horizontal centre of screen.
     * trunk_base_y: row just above pot, computed after drawing pot.
     */
    s->trunk_cx     = cols / 2;
    s->trunk_base_y = draw_pot(s->trunk_cx, rows, s->pot_type);

    tree_reset(&s->tree, s->trunk_cx, s->trunk_base_y,
               s->tree_type, s->life, s->multiplier);
    s->tree.leaf_set = s->leaf_set;
    s->done    = false;
    s->growing = true;
    s->wait_accum = 0;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->tree_type  = TREE_RANDOM;
    s->pot_type   = POT_BIG;
    s->multiplier = MULT_DEFAULT;
    s->life       = LIFE_DEFAULT;
    s->grow_fps   = GROW_FPS_DEFAULT;
    s->leaf_set   = 0;
    s->live_mode  = true;
    s->infinite   = false;
    s->screensaver= false;
    s->show_msg   = false;
    strncpy(s->message, "bonsai!", MSG_MAX - 1);
    s->seed       = (unsigned int)clock_ns();
    srand(s->seed);

    scene_plant(s, cols, rows);
}

/*
 * scene_tick — called each growth step from the accumulator.
 * Returns false when tree has just finished.
 */
static bool scene_tick(Scene *s, int cols, int rows)
{
    if (s->done || s->paused) return true;

    bool alive = tree_step(&s->tree, cols, rows, s->trunk_base_y);
    if (!alive) {
        s->done    = true;
        s->growing = false;
        return false;
    }
    return true;
}

/*
 * scene_draw_static — repaint elements that don't change each growth step:
 * pot, message, HUD.  Called every render frame.
 */
static void scene_draw_static(Scene *s, int cols, int rows, double fps)
{
    /* pot */
    draw_pot(s->trunk_cx, rows, s->pot_type);

    /* message */
    if (s->show_msg)
        draw_message(s->message, cols, rows);

    /* HUD — top right */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps [%s][%s] M:%d L:%d ",
             fps,
             k_tree_names[s->tree_type],
             k_pot_names[s->pot_type],
             s->multiplier,
             s->life);
    int hx = cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(7) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(7) | A_BOLD);

    /* key hints bottom */
    attron(COLOR_PAIR(6));
    mvprintw(rows - 1, 0,
        "t=type  b=pot  m=msg  M/N=branch  L/K=life  ]/[=speed  r=seed  q=quit");
    attroff(COLOR_PAIR(6));

    /* status indicator */
    const char *status = s->done    ? (s->infinite ? "waiting..." : "done")
                       : s->paused  ? "PAUSED"
                       : "growing";
    attron(COLOR_PAIR(7));
    mvprintw(0, 0, " [%s] seed:%u ", status, s->seed);
    attroff(COLOR_PAIR(7));
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s){ (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
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

static void on_exit_signal(int sig)  { (void)sig; g_app.running    = 0; }
static void on_resize_signal(int sig){ (void)sig; g_app.need_resize = 1; }
static void cleanup(void)            { endwin(); }

static void app_do_resize(App *app, int64_t *frame_time, int64_t *grow_accum)
{
    screen_resize(&app->screen);
    /* redraw from scratch on resize */
    erase();
    scene_plant(&app->scene, app->screen.cols, app->screen.rows);
    srand(app->scene.seed);
    *frame_time  = clock_ns();
    *grow_accum  = 0;
    app->need_resize = 0;
}

static void app_regrow(App *app, int64_t *frame_time, int64_t *grow_accum)
{
    app->scene.seed = (unsigned int)clock_ns();
    srand(app->scene.seed);
    erase();
    scene_plant(&app->scene, app->screen.cols, app->screen.rows);
    *frame_time = clock_ns();
    *grow_accum = 0;
}

/*
 * app_handle_key — return false to quit.
 *
 * For screensaver mode any key quits.
 */
static bool app_handle_key(App *app, int ch,
                            int64_t *frame_time, int64_t *grow_accum)
{
    Scene  *s    = &app->scene;
    int     cols = app->screen.cols;
    int     rows = app->screen.rows;

    if (s->screensaver) return false;   /* any key quits screensaver */

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        /* new seed → full restart */
        s->seed = (unsigned int)clock_ns();
        srand(s->seed);
        erase();
        scene_plant(s, cols, rows);
        *frame_time = clock_ns();
        *grow_accum = 0;
        break;

    case 't': case 'T':
        s->tree_type = (TreeType)((s->tree_type + 1) % TREE_COUNT);
        erase();
        scene_plant(s, cols, rows);
        *frame_time = clock_ns();
        *grow_accum = 0;
        break;

    case 'b': case 'B':
        s->pot_type = (PotType)((s->pot_type + 1) % POT_COUNT);
        erase();
        scene_plant(s, cols, rows);
        *frame_time = clock_ns();
        *grow_accum = 0;
        break;

    case 'm':
        s->show_msg = !s->show_msg;
        break;

    case 'M':
        s->multiplier++;
        if (s->multiplier > MULT_MAX) s->multiplier = MULT_MAX;
        break;

    case 'N':
        s->multiplier--;
        if (s->multiplier < MULT_MIN) s->multiplier = MULT_MIN;
        break;

    case 'L':
        s->life += 10;
        if (s->life > LIFE_MAX) s->life = LIFE_MAX;
        break;

    case 'K':
        s->life -= 10;
        if (s->life < LIFE_MIN) s->life = LIFE_MIN;
        break;

    case 'l':   /* cycle leaf character set */
        s->leaf_set = (s->leaf_set + 1) % LEAF_SETS;
        s->tree.leaf_set = s->leaf_set;
        break;

    case ']':
        s->grow_fps += GROW_FPS_STEP;
        if (s->grow_fps > GROW_FPS_MAX) s->grow_fps = GROW_FPS_MAX;
        break;

    case '[':
        s->grow_fps -= GROW_FPS_STEP;
        if (s->grow_fps < GROW_FPS_MIN) s->grow_fps = GROW_FPS_MIN;
        break;

    case 'i': case 'I':
        s->infinite = !s->infinite;
        break;

    case 'S':
        s->screensaver = true;
        s->live_mode   = true;
        s->infinite    = true;
        break;

    default: break;
    }
    return true;
}

int main(int argc, char *argv[])
{
    /* minimal CLI: -S screensaver, -i infinite, -m "msg", -s seed */
    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* parse simple CLI args */
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-S") || !strcmp(argv[i], "--screensaver")) {
            app->scene.screensaver = true;
            app->scene.live_mode   = true;
            app->scene.infinite    = true;
        } else if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "--infinite")) {
            app->scene.infinite = true;
        } else if (!strcmp(argv[i], "-l") || !strcmp(argv[i], "--live")) {
            app->scene.live_mode = true;
        } else if ((!strcmp(argv[i], "-m") || !strcmp(argv[i], "--message"))
                   && i+1 < argc) {
            strncpy(app->scene.message, argv[++i], MSG_MAX-1);
            app->scene.show_msg = true;
        } else if ((!strcmp(argv[i], "-s") || !strcmp(argv[i], "--seed"))
                   && i+1 < argc) {
            app->scene.seed = (unsigned int)atoi(argv[++i]);
            srand(app->scene.seed);
            /* replant with new seed */
            erase();
            scene_plant(&app->scene,
                        app->screen.cols, app->screen.rows);
        } else if ((!strcmp(argv[i], "-M") || !strcmp(argv[i], "--multiplier"))
                   && i+1 < argc) {
            app->scene.multiplier = atoi(argv[++i]);
            if (app->scene.multiplier < MULT_MIN) app->scene.multiplier = MULT_MIN;
            if (app->scene.multiplier > MULT_MAX) app->scene.multiplier = MULT_MAX;
        } else if ((!strcmp(argv[i], "-L") || !strcmp(argv[i], "--life"))
                   && i+1 < argc) {
            app->scene.life = atoi(argv[++i]);
            if (app->scene.life < LIFE_MIN) app->scene.life = LIFE_MIN;
            if (app->scene.life > LIFE_MAX) app->scene.life = LIFE_MAX;
        } else if ((!strcmp(argv[i], "-t") || !strcmp(argv[i], "--type"))
                   && i+1 < argc) {
            app->scene.tree_type = (TreeType)(atoi(argv[++i]) % TREE_COUNT);
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            endwin();
            printf("Usage: bonsai [OPTIONS]\n\n");
            printf("  -S, --screensaver      screensaver mode (live+infinite, any key quits)\n");
            printf("  -i, --infinite         regrow tree after each one finishes\n");
            printf("  -l, --live             animate growth step by step\n");
            printf("  -m, --message STR      show message box next to tree\n");
            printf("  -s, --seed INT         seed the random number generator\n");
            printf("  -M, --multiplier INT   branch multiplier 0-20 [default: %d]\n", MULT_DEFAULT);
            printf("  -L, --life INT         tree life 10-200 [default: %d]\n", LIFE_DEFAULT);
            printf("  -t, --type INT         tree type 0=random 1=dwarf 2=weeping 3=sparse 4=bamboo\n");
            printf("  -h, --help             show this help\n");
            printf("\nRuntime keys:\n");
            printf("  t = cycle type    b = cycle pot    m = toggle message\n");
            printf("  M/N = more/fewer branches    L/K = longer/shorter\n");
            printf("  ]/[ = faster/slower    r = new seed    space = pause    q = quit\n");
            return 0;
        }
    }

    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
    atexit(cleanup);

    int64_t frame_time  = clock_ns();
    int64_t grow_accum  = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    /*
     * Render loop.
     *
     * Two accumulators:
     *   grow_accum  — drives tree_step() at grow_fps Hz
     *   fps_accum   — measures actual render fps for HUD
     *
     * The tree is drawn incrementally — we do NOT erase() each frame.
     * Each tree_step() writes characters directly to stdscr and they
     * persist until the next erase() (which only happens on regrow).
     * Static elements (pot, HUD, message) are redrawn every frame so
     * they stay on top of any branch characters that may land there.
     */
    while (app->running) {

        /* ── resize ──────────────────────────────────────────── */
        if (app->need_resize)
            app_do_resize(app, &frame_time, &grow_accum);

        /* ── dt ──────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;

        /* ── growth accumulator ──────────────────────────────── */
        Scene *s = &app->scene;

        if (s->done && (s->infinite || s->screensaver)) {
            /*
             * Tree finished — wait INFINITE_WAIT_MS then regrow.
             */
            s->wait_accum += dt;
            if (s->wait_accum >= INFINITE_WAIT_MS * NS_PER_MS)
                app_regrow(app, &frame_time, &grow_accum);
        } else if (!s->done) {
            int64_t tick_ns = NS_PER_SEC / s->grow_fps;
            grow_accum += dt;
            while (grow_accum >= tick_ns && !s->done) {
                scene_tick(s, app->screen.cols, app->screen.rows);
                grow_accum -= tick_ns;
            }
        }

        /* ── FPS counter ─────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── static elements (redrawn every frame, on top) ───── */
        scene_draw_static(s, app->screen.cols, app->screen.rows, fps_display);

        /* ── flush ───────────────────────────────────────────── */
        screen_present();

        /* ── input ───────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR) {
            if (!app_handle_key(app, ch, &frame_time, &grow_accum))
                app->running = 0;
        }

        /* ── frame cap ───────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
