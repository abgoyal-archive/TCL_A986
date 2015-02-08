


#ifndef STATE_H_
# define STATE_H_

# include <bitset.h>

# include "gram.h"
# include "symtab.h"



typedef int state_number;
# define STATE_NUMBER_MAXIMUM INT_MAX

/* Be ready to map a state_number to an int.  */
static inline int
state_number_as_int (state_number s)
{
  return s;
}


typedef struct state state;


typedef struct
{
  int num;
  state *states[1];
} transitions;



#define TRANSITION_SYMBOL(Transitions, Num) \
  (Transitions->states[Num]->accessing_symbol)

/* Is the TRANSITIONS->states[Num] a shift? (as opposed to gotos).  */

#define TRANSITION_IS_SHIFT(Transitions, Num) \
  (ISTOKEN (TRANSITION_SYMBOL (Transitions, Num)))

/* Is the TRANSITIONS->states[Num] a goto?. */

#define TRANSITION_IS_GOTO(Transitions, Num) \
  (!TRANSITION_IS_SHIFT (Transitions, Num))

/* Is the TRANSITIONS->states[Num] labelled by the error token?  */

#define TRANSITION_IS_ERROR(Transitions, Num) \
  (TRANSITION_SYMBOL (Transitions, Num) == errtoken->number)


#define TRANSITION_DISABLE(Transitions, Num) \
  (Transitions->states[Num] = NULL)

#define TRANSITION_IS_DISABLED(Transitions, Num) \
  (Transitions->states[Num] == NULL)


/* Iterate over each transition over a token (shifts).  */
#define FOR_EACH_SHIFT(Transitions, Iter)			\
  for (Iter = 0;						\
       Iter < Transitions->num					\
	 && (TRANSITION_IS_DISABLED (Transitions, Iter)		\
	     || TRANSITION_IS_SHIFT (Transitions, Iter));	\
       ++Iter)							\
    if (!TRANSITION_IS_DISABLED (Transitions, Iter))


struct state *transitions_to (transitions *shifts, symbol_number sym);



typedef struct
{
  int num;
  symbol *symbols[1];
} errs;

errs *errs_new (int num, symbol **tokens);



typedef struct
{
  int num;
  bitset *look_ahead_tokens;
  rule *rules[1];
} reductions;




struct state
{
  state_number number;
  symbol_number accessing_symbol;
  transitions *transitions;
  reductions *reductions;
  errs *errs;

  /* Nonzero if no look-ahead is needed to decide what to do in state S.  */
  char consistent;

  /* If some conflicts were solved thanks to precedence/associativity,
     a human readable description of the resolution.  */
  const char *solved_conflicts;

  /* Its items.  Must be last, since ITEMS can be arbitrarily large.
     */
  size_t nitems;
  item_number items[1];
};

extern state_number nstates;
extern state *final_state;

/* Create a new state with ACCESSING_SYMBOL for those items.  */
state *state_new (symbol_number accessing_symbol,
		  size_t core_size, item_number *core);

/* Set the transitions of STATE.  */
void state_transitions_set (state *s, int num, state **trans);

/* Set the reductions of STATE.  */
void state_reductions_set (state *s, int num, rule **reds);

int state_reduction_find (state *s, rule *r);

/* Set the errs of STATE.  */
void state_errs_set (state *s, int num, symbol **errors);

void state_rule_look_ahead_tokens_print (state *s, rule *r, FILE *out);

/* Create/destroy the states hash table.  */
void state_hash_new (void);
void state_hash_free (void);

state *state_hash_lookup (size_t core_size, item_number *core);

/* Insert STATE in the state hash table.  */
void state_hash_insert (state *s);

/* All the states, indexed by the state number.  */
extern state **states;

/* Free all the states.  */
void states_free (void);
#endif /* !STATE_H_ */
