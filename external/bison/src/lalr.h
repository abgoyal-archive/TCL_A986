
#ifndef LALR_H_
# define LALR_H_

# include <bitset.h>
# include <bitsetv.h>

/* Import the definition of RULE_T. */
# include "gram.h"

/* Import the definition of CORE, TRANSITIONS and REDUCTIONS. */
# include "state.h"


void lalr (void);


void lalr_free (void);


/* lalr() builds these data structures. */


typedef size_t goto_number;
# define GOTO_NUMBER_MAXIMUM ((goto_number) -1)

extern goto_number *goto_map;
extern state_number *from_state;
extern state_number *to_state;


#endif /* !LALR_H_ */
