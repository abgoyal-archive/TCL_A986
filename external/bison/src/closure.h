
#ifndef CLOSURE_H_
# define CLOSURE_H_

# include "gram.h"


void new_closure (unsigned int n);



void closure (item_number *items, size_t n);


/* Frees ITEMSET, RULESET and internal data.  */

void free_closure (void);

extern item_number *itemset;
extern size_t nritemset;

#endif /* !CLOSURE_H_ */
