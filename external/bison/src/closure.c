
#include <config.h>
#include "system.h"

#include <bitset.h>
#include <bitsetv-print.h>
#include <bitsetv.h>
#include <quotearg.h>

#include "closure.h"
#include "derives.h"
#include "getargs.h"
#include "gram.h"
#include "reader.h"
#include "symtab.h"

/* NITEMSET is the size of the array ITEMSET.  */
item_number *itemset;
size_t nritemset;

static bitset ruleset;

/* internal data.  See comments before set_fderives and set_firsts.  */
static bitsetv fderives = NULL;
static bitsetv firsts = NULL;

/* Retrieve the FDERIVES/FIRSTS sets of the nonterminals numbered Var.  */
#define FDERIVES(Var)   fderives[(Var) - ntokens]
#define   FIRSTS(Var)   firsts[(Var) - ntokens]



static void
print_closure (char const *title, item_number *array, size_t size)
{
  size_t i;
  fprintf (stderr, "Closure: %s\n", title);
  for (i = 0; i < size; ++i)
    {
      item_number *rp;
      fprintf (stderr, "  %2d: .", array[i]);
      for (rp = &ritem[array[i]]; *rp >= 0; ++rp)
	fprintf (stderr, " %s", symbols[*rp]->tag);
      fprintf (stderr, "  (rule %d)\n", -*rp - 1);
    }
  fputs ("\n\n", stderr);
}


static void
print_firsts (void)
{
  symbol_number i, j;

  fprintf (stderr, "FIRSTS\n");
  for (i = ntokens; i < nsyms; i++)
    {
      bitset_iterator iter;
      fprintf (stderr, "\t%s firsts\n", symbols[i]->tag);
      BITSET_FOR_EACH (iter, FIRSTS (i), j, 0)
	{
	  fprintf (stderr, "\t\t%s\n",
		   symbols[j + ntokens]->tag);
	}
    }
  fprintf (stderr, "\n\n");
}


static void
print_fderives (void)
{
  int i;
  rule_number r;

  fprintf (stderr, "FDERIVES\n");
  for (i = ntokens; i < nsyms; i++)
    {
      bitset_iterator iter;
      fprintf (stderr, "\t%s derives\n", symbols[i]->tag);
      BITSET_FOR_EACH (iter, FDERIVES (i), r, 0)
	{
	  fprintf (stderr, "\t\t%3d ", r);
	  rule_rhs_print (&rules[r], stderr);
	}
    }
  fprintf (stderr, "\n\n");
}


static void
set_firsts (void)
{
  symbol_number i, j;

  firsts = bitsetv_create (nvars, nvars, BITSET_FIXED);

  for (i = ntokens; i < nsyms; i++)
    for (j = 0; derives[i - ntokens][j]; ++j)
      {
	item_number sym = derives[i - ntokens][j]->rhs[0];
	if (ISVAR (sym))
	  bitset_set (FIRSTS (i), sym - ntokens);
      }

  if (trace_flag & trace_sets)
    bitsetv_matrix_dump (stderr, "RTC: Firsts Input", firsts);
  bitsetv_reflexive_transitive_closure (firsts);
  if (trace_flag & trace_sets)
    bitsetv_matrix_dump (stderr, "RTC: Firsts Output", firsts);

  if (trace_flag & trace_sets)
    print_firsts ();
}


static void
set_fderives (void)
{
  symbol_number i, j;
  rule_number k;

  fderives = bitsetv_create (nvars, nrules, BITSET_FIXED);

  set_firsts ();

  for (i = ntokens; i < nsyms; ++i)
    for (j = ntokens; j < nsyms; ++j)
      if (bitset_test (FIRSTS (i), j - ntokens))
	for (k = 0; derives[j - ntokens][k]; ++k)
	  bitset_set (FDERIVES (i), derives[j - ntokens][k]->number);

  if (trace_flag & trace_sets)
    print_fderives ();

  bitsetv_free (firsts);
}



void
new_closure (unsigned int n)
{
  itemset = xnmalloc (n, sizeof *itemset);

  ruleset = bitset_create (nrules, BITSET_FIXED);

  set_fderives ();
}



void
closure (item_number *core, size_t n)
{
  /* Index over CORE. */
  size_t c;

  /* A bit index over RULESET. */
  rule_number ruleno;

  bitset_iterator iter;

  if (trace_flag & trace_sets)
    print_closure ("input", core, n);

  bitset_zero (ruleset);

  for (c = 0; c < n; ++c)
    if (ISVAR (ritem[core[c]]))
      bitset_or (ruleset, ruleset, FDERIVES (ritem[core[c]]));

  nritemset = 0;
  c = 0;
  BITSET_FOR_EACH (iter, ruleset, ruleno, 0)
    {
      item_number itemno = rules[ruleno].rhs - ritem;
      while (c < n && core[c] < itemno)
	{
	  itemset[nritemset] = core[c];
	  nritemset++;
	  c++;
	}
      itemset[nritemset] = itemno;
      nritemset++;
    };

  while (c < n)
    {
      itemset[nritemset] = core[c];
      nritemset++;
      c++;
    }

  if (trace_flag & trace_sets)
    print_closure ("output", itemset, nritemset);
}


void
free_closure (void)
{
  free (itemset);
  bitset_free (ruleset);
  bitsetv_free (fderives);
}
