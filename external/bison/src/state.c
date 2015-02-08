
#include <config.h>
#include "system.h"

#include <hash.h>

#include "complain.h"
#include "gram.h"
#include "state.h"


			/*-------------------.
			| Shifts and Gotos.  |
			`-------------------*/



static transitions *
transitions_new (int num, state **the_states)
{
  size_t states_size = num * sizeof *the_states;
  transitions *res = xmalloc (offsetof (transitions, states) + states_size);
  res->num = num;
  memcpy (res->states, the_states, states_size);
  return res;
}



state *
transitions_to (transitions *shifts, symbol_number sym)
{
  int j;
  for (j = 0; ; j++)
    {
      assert (j < shifts->num);
      if (TRANSITION_SYMBOL (shifts, j) == sym)
	return shifts->states[j];
    }
}


			/*--------------------.
			| Error transitions.  |
			`--------------------*/



errs *
errs_new (int num, symbol **tokens)
{
  size_t symbols_size = num * sizeof *tokens;
  errs *res = xmalloc (offsetof (errs, symbols) + symbols_size);
  res->num = num;
  memcpy (res->symbols, tokens, symbols_size);
  return res;
}




			/*-------------.
			| Reductions.  |
			`-------------*/



static reductions *
reductions_new (int num, rule **reds)
{
  size_t rules_size = num * sizeof *reds;
  reductions *res = xmalloc (offsetof (reductions, rules) + rules_size);
  res->num = num;
  res->look_ahead_tokens = NULL;
  memcpy (res->rules, reds, rules_size);
  return res;
}



			/*---------.
			| States.  |
			`---------*/


state_number nstates = 0;
state *final_state = NULL;



state *
state_new (symbol_number accessing_symbol,
	   size_t nitems, item_number *core)
{
  state *res;
  size_t items_size = nitems * sizeof *core;

  assert (nstates < STATE_NUMBER_MAXIMUM);

  res = xmalloc (offsetof (state, items) + items_size);
  res->number = nstates++;
  res->accessing_symbol = accessing_symbol;
  res->transitions = NULL;
  res->reductions = NULL;
  res->errs = NULL;
  res->consistent = 0;
  res->solved_conflicts = NULL;

  res->nitems = nitems;
  memcpy (res->items, core, items_size);

  state_hash_insert (res);

  return res;
}



static void
state_free (state *s)
{
  free (s->transitions);
  free (s->reductions);
  free (s->errs);
  free (s);
}



void
state_transitions_set (state *s, int num, state **trans)
{
  assert (!s->transitions);
  s->transitions = transitions_new (num, trans);
}



void
state_reductions_set (state *s, int num, rule **reds)
{
  assert (!s->reductions);
  s->reductions = reductions_new (num, reds);
}


int
state_reduction_find (state *s, rule *r)
{
  int i;
  reductions *reds = s->reductions;
  for (i = 0; i < reds->num; ++i)
    if (reds->rules[i] == r)
      return i;
  return -1;
}



void
state_errs_set (state *s, int num, symbol **tokens)
{
  assert (!s->errs);
  s->errs = errs_new (num, tokens);
}




void
state_rule_look_ahead_tokens_print (state *s, rule *r, FILE *out)
{
  /* Find the reduction we are handling.  */
  reductions *reds = s->reductions;
  int red = state_reduction_find (s, r);

  /* Print them if there are.  */
  if (reds->look_ahead_tokens && red != -1)
    {
      bitset_iterator biter;
      int k;
      char const *sep = "";
      fprintf (out, "  [");
      BITSET_FOR_EACH (biter, reds->look_ahead_tokens[red], k, 0)
	{
	  fprintf (out, "%s%s", sep, symbols[k]->tag);
	  sep = ", ";
	}
      fprintf (out, "]");
    }
}



/* Initial capacity of states hash table.  */
#define HT_INITIAL_CAPACITY 257

static struct hash_table *state_table = NULL;

/* Two states are equal if they have the same core items.  */
static inline bool
state_compare (state const *s1, state const *s2)
{
  size_t i;

  if (s1->nitems != s2->nitems)
    return false;

  for (i = 0; i < s1->nitems; ++i)
    if (s1->items[i] != s2->items[i])
      return false;

  return true;
}

static bool
state_comparator (void const *s1, void const *s2)
{
  return state_compare (s1, s2);
}

static inline size_t
state_hash (state const *s, size_t tablesize)
{
  /* Add up the state's item numbers to get a hash key.  */
  size_t key = 0;
  size_t i;
  for (i = 0; i < s->nitems; ++i)
    key += s->items[i];
  return key % tablesize;
}

static size_t
state_hasher (void const *s, size_t tablesize)
{
  return state_hash (s, tablesize);
}



void
state_hash_new (void)
{
  state_table = hash_initialize (HT_INITIAL_CAPACITY,
				 NULL,
				 state_hasher,
				 state_comparator,
				 NULL);
}



void
state_hash_free (void)
{
  hash_free (state_table);
}



void
state_hash_insert (state *s)
{
  hash_insert (state_table, s);
}



state *
state_hash_lookup (size_t nitems, item_number *core)
{
  size_t items_size = nitems * sizeof *core;
  state *probe = xmalloc (offsetof (state, items) + items_size);
  state *entry;

  probe->nitems = nitems;
  memcpy (probe->items, core, items_size);
  entry = hash_lookup (state_table, probe);
  free (probe);
  return entry;
}

/* All the decorated states, indexed by the state number.  */
state **states = NULL;



void
states_free (void)
{
  state_number i;
  for (i = 0; i < nstates; ++i)
    state_free (states[i]);
  free (states);
}
