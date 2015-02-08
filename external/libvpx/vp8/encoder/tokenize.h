


#ifndef tokenize_h
#define tokenize_h

#include "entropy.h"
#include "block.h"

void vp8_tokenize_initialize();

typedef struct
{
    short Token;
    short Extra;
} TOKENVALUE;

typedef struct
{
    int Token;
    int Extra;
    const vp8_prob *context_tree;
    int skip_eob_node;
    int section;
} TOKENEXTRA;

int rd_cost_mby(MACROBLOCKD *);

#ifdef ENTROPY_STATS
void init_context_counters();
void print_context_counters();

extern _int64 context_counters[BLOCK_TYPES] [COEF_BANDS] [PREV_COEF_CONTEXTS] [vp8_coef_tokens];
#endif

extern const int *vp8_dct_value_cost_ptr;
extern const TOKENVALUE *vp8_dct_value_tokens_ptr;

#endif  /* tokenize_h */
