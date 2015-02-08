


#include "treewriter.h"

static void cost(
    int *const C,
    vp8_tree T,
    const vp8_prob *const P,
    int i,
    int c
)
{
    const vp8_prob p = P [i>>1];

    do
    {
        const vp8_tree_index j = T[i];
        const int d = c + vp8_cost_bit(p, i & 1);

        if (j <= 0)
            C[-j] = d;
        else
            cost(C, T, P, j, d);
    }
    while (++i & 1);
}
void vp8_cost_tokens(int *c, const vp8_prob *p, vp8_tree t)
{
    cost(c, t, p, 0, 0);
}
