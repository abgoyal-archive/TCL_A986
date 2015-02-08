
#include <config.h>
#include "system.h"

#include "getargs.h"

#include "derives.h"
#include "gram.h"
#include "reader.h"
#include "symtab.h"

/* Linked list of rule numbers.  */
typedef struct rule_list
{
  struct rule_list *next;
  rule *value;
} rule_list;

rule ***derives;

static void
print_derives (void)
{
  int i;

  fputs ("DERIVES\n", stderr);

  for (i = ntokens; i < nsyms; i++)
    {
      rule **rp;
      fprintf (stderr, "\t%s derives\n", symbols[i]->tag);
      for (rp = derives[i - ntokens]; *rp; ++rp)
	{
	  fprintf (stderr, "\t\t%3d ", (*rp)->user_number);
	  rule_rhs_print (*rp, stderr);
	}
    }

  fputs ("\n\n", stderr);
}


void
derives_compute (void)
{
  symbol_number i;
  rule_number r;
  rule **q;

  /* DSET[NTERM - NTOKENS] -- A linked list of the numbers of the rules
     whose LHS is NTERM.  */
  rule_list **dset = xcalloc (nvars, sizeof *dset);

  /* DELTS[RULE] -- There are NRULES rule number to attach to nterms.
     Instead of performing NRULES allocations for each, have an array
     indexed by rule numbers.  */
  rule_list *delts = xnmalloc (nrules, sizeof *delts);

  for (r = nrules - 1; r >= 0; --r)
    {
      symbol_number lhs = rules[r].lhs->number;
      rule_list *p = &delts[r];
      /* A new LHS is found.  */
      p->next = dset[lhs - ntokens];
      p->value = &rules[r];
      dset[lhs - ntokens] = p;
    }

  /* DSET contains what we need under the form of a linked list.  Make
     it a single array.  */

  derives = xnmalloc (nvars, sizeof *derives);
  q = xnmalloc (nvars + nrules, sizeof *q);

  for (i = ntokens; i < nsyms; i++)
    {
      rule_list *p = dset[i - ntokens];
      derives[i - ntokens] = q;
      while (p)
	{
	  *q++ = p->value;
	  p = p->next;
	}
      *q++ = NULL;
    }

  if (trace_flag & trace_sets)
    print_derives ();

  free (dset);
  free (delts);
}


void
derives_free (void)
{
  free (derives[0]);
  free (derives);
}
