


#ifndef __INC_TREECODER_H
#define __INC_TREECODER_H

typedef unsigned char vp8bc_index_t; /* probability index */


typedef unsigned char vp8_prob;

#define vp8_prob_half ( (vp8_prob) 128)

typedef signed char vp8_tree_index;
struct bool_coder_spec;

typedef struct bool_coder_spec bool_coder_spec;
typedef struct bool_writer bool_writer;
typedef struct bool_reader bool_reader;

typedef const bool_coder_spec c_bool_coder_spec;
typedef const bool_writer c_bool_writer;
typedef const bool_reader c_bool_reader;



# define vp8_complement( x) (255 - x)



typedef const vp8_tree_index vp8_tree[], *vp8_tree_p;


typedef const struct vp8_token_struct
{
    int value;
    int Len;
} vp8_token;

/* Construct encoding array from tree. */

void vp8_tokens_from_tree(struct vp8_token_struct *, vp8_tree);
void vp8_tokens_from_tree_offset(struct vp8_token_struct *, vp8_tree,
                                 int offset);



void vp8_tree_probs_from_distribution(
    int n,                      /* n = size of alphabet */
    vp8_token tok               [ /* n */ ],
    vp8_tree tree,
    vp8_prob probs          [ /* n-1 */ ],
    unsigned int branch_ct       [ /* n-1 */ ] [2],
    const unsigned int num_events[ /* n */ ],
    unsigned int Pfactor,
    int Round
);

/* Variant of above using coder spec rather than hardwired 8-bit probs. */

void vp8bc_tree_probs_from_distribution(
    int n,                      /* n = size of alphabet */
    vp8_token tok               [ /* n */ ],
    vp8_tree tree,
    vp8_prob probs          [ /* n-1 */ ],
    unsigned int branch_ct       [ /* n-1 */ ] [2],
    const unsigned int num_events[ /* n */ ],
    c_bool_coder_spec *s
);


#endif
