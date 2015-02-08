
#include <config.h>
#include "system.h"

#include <bitset.h>

#include "LR0.h"
#include "complain.h"
#include "conflicts.h"
#include "files.h"
#include "getargs.h"
#include "gram.h"
#include "lalr.h"
#include "reader.h"
#include "state.h"
#include "symtab.h"

/* -1 stands for not specified. */
int expected_sr_conflicts = -1;
int expected_rr_conflicts = -1;
static char *conflicts;
static struct obstack solved_conflicts_obstack;

static bitset shift_set;
static bitset look_ahead_set;



enum conflict_resolution
  {
    shift_resolution,
    reduce_resolution,
    left_resolution,
    right_resolution,
    nonassoc_resolution
  };



static inline void
log_resolution (rule *r, symbol_number token,
		enum conflict_resolution resolution)
{
  if (report_flag & report_solved_conflicts)
    {
      /* The description of the resolution. */
      switch (resolution)
	{
	case shift_resolution:
	case right_resolution:
	  obstack_fgrow2 (&solved_conflicts_obstack,
			  _("\
    Conflict between rule %d and token %s resolved as shift"),
			  r->number,
			  symbols[token]->tag);
	  break;
	case reduce_resolution:
	case left_resolution:
	  obstack_fgrow2 (&solved_conflicts_obstack,
			  _("\
    Conflict between rule %d and token %s resolved as reduce"),
			  r->number,
			  symbols[token]->tag);
	  break;
	case nonassoc_resolution:
	  obstack_fgrow2 (&solved_conflicts_obstack,
			  _("\
    Conflict between rule %d and token %s resolved as an error"),
			  r->number,
			  symbols[token]->tag);
	  break;
	}

      /* The reason. */
      switch (resolution)
	{
	case shift_resolution:
	  obstack_fgrow2 (&solved_conflicts_obstack,
			  " (%s < %s)",
			  r->prec->tag,
			  symbols[token]->tag);
	  break;

	case reduce_resolution:
	  obstack_fgrow2 (&solved_conflicts_obstack,
			  " (%s < %s)",
			  symbols[token]->tag,
			  r->prec->tag);
	  break;

	case left_resolution:
	  obstack_fgrow1 (&solved_conflicts_obstack,
			  " (%%left %s)",
			  symbols[token]->tag);
	  break;

	case right_resolution:
	  obstack_fgrow1 (&solved_conflicts_obstack,
			  " (%%right %s)",
			  symbols[token]->tag);
	  break;
	case nonassoc_resolution:
	  obstack_fgrow1 (&solved_conflicts_obstack,
			  " (%%nonassoc %s)",
			  symbols[token]->tag);
	  break;
	}
      obstack_sgrow (&solved_conflicts_obstack, ".\n");
    }
}



static void
flush_shift (state *s, int token)
{
  transitions *trans = s->transitions;
  int i;

  bitset_reset (look_ahead_set, token);
  for (i = 0; i < trans->num; i++)
    if (!TRANSITION_IS_DISABLED (trans, i)
	&& TRANSITION_SYMBOL (trans, i) == token)
      TRANSITION_DISABLE (trans, i);
}



static void
flush_reduce (bitset look_ahead_tokens, int token)
{
  bitset_reset (look_ahead_tokens, token);
}



static void
resolve_sr_conflict (state *s, int ruleno, symbol **errors)
{
  symbol_number i;
  reductions *reds = s->reductions;
  /* Find the rule to reduce by to get precedence of reduction.  */
  rule *redrule = reds->rules[ruleno];
  int redprec = redrule->prec->prec;
  bitset look_ahead_tokens = reds->look_ahead_tokens[ruleno];
  int nerrs = 0;

  for (i = 0; i < ntokens; i++)
    if (bitset_test (look_ahead_tokens, i)
	&& bitset_test (look_ahead_set, i)
	&& symbols[i]->prec)
      {
	/* Shift-reduce conflict occurs for token number i
	   and it has a precedence.
	   The precedence of shifting is that of token i.  */
	if (symbols[i]->prec < redprec)
	  {
	    log_resolution (redrule, i, reduce_resolution);
	    flush_shift (s, i);
	  }
	else if (symbols[i]->prec > redprec)
	  {
	    log_resolution (redrule, i, shift_resolution);
	    flush_reduce (look_ahead_tokens, i);
	  }
	else
	  /* Matching precedence levels.
	     For left association, keep only the reduction.
	     For right association, keep only the shift.
	     For nonassociation, keep neither.  */

	  switch (symbols[i]->assoc)
	    {
	    default:
	      abort ();

	    case right_assoc:
	      log_resolution (redrule, i, right_resolution);
	      flush_reduce (look_ahead_tokens, i);
	      break;

	    case left_assoc:
	      log_resolution (redrule, i, left_resolution);
	      flush_shift (s, i);
	      break;

	    case non_assoc:
	      log_resolution (redrule, i, nonassoc_resolution);
	      flush_shift (s, i);
	      flush_reduce (look_ahead_tokens, i);
	      /* Record an explicit error for this token.  */
	      errors[nerrs++] = symbols[i];
	      break;
	    }
      }

  if (nerrs)
    {
      /* Some tokens have been explicitly made errors.  Allocate a
	 permanent errs structure for this state, to record them.  */
      state_errs_set (s, nerrs, errors);
    }

  if (obstack_object_size (&solved_conflicts_obstack))
    {
      obstack_1grow (&solved_conflicts_obstack, '\0');
      s->solved_conflicts = obstack_finish (&solved_conflicts_obstack);
    }
}



static void
set_conflicts (state *s, symbol **errors)
{
  int i;
  transitions *trans = s->transitions;
  reductions *reds = s->reductions;

  if (s->consistent)
    return;

  bitset_zero (look_ahead_set);

  FOR_EACH_SHIFT (trans, i)
    bitset_set (look_ahead_set, TRANSITION_SYMBOL (trans, i));

  /* Loop over all rules which require look-ahead in this state.  First
     check for shift-reduce conflict, and try to resolve using
     precedence.  */
  for (i = 0; i < reds->num; ++i)
    if (reds->rules[i]->prec && reds->rules[i]->prec->prec
	&& !bitset_disjoint_p (reds->look_ahead_tokens[i], look_ahead_set))
      resolve_sr_conflict (s, i, errors);

  /* Loop over all rules which require look-ahead in this state.  Check
     for conflicts not resolved above.  */
  for (i = 0; i < reds->num; ++i)
    {
      if (!bitset_disjoint_p (reds->look_ahead_tokens[i], look_ahead_set))
	conflicts[s->number] = 1;

      bitset_or (look_ahead_set, look_ahead_set, reds->look_ahead_tokens[i]);
    }
}



void
conflicts_solve (void)
{
  state_number i;
  /* List of look-ahead tokens on which we explicitly raise a syntax error.  */
  symbol **errors = xnmalloc (ntokens + 1, sizeof *errors);

  conflicts = xcalloc (nstates, sizeof *conflicts);
  shift_set = bitset_create (ntokens, BITSET_FIXED);
  look_ahead_set = bitset_create (ntokens, BITSET_FIXED);
  obstack_init (&solved_conflicts_obstack);

  for (i = 0; i < nstates; i++)
    {
      set_conflicts (states[i], errors);

      /* For uniformity of the code, make sure all the states have a valid
	 `errs' member.  */
      if (!states[i]->errs)
	states[i]->errs = errs_new (0, 0);
    }

  free (errors);
}



static int
count_sr_conflicts (state *s)
{
  int i;
  int src_count = 0;
  transitions *trans = s->transitions;
  reductions *reds = s->reductions;

  if (!trans)
    return 0;

  bitset_zero (look_ahead_set);
  bitset_zero (shift_set);

  FOR_EACH_SHIFT (trans, i)
    bitset_set (shift_set, TRANSITION_SYMBOL (trans, i));

  for (i = 0; i < reds->num; ++i)
    bitset_or (look_ahead_set, look_ahead_set, reds->look_ahead_tokens[i]);

  bitset_and (look_ahead_set, look_ahead_set, shift_set);

  src_count = bitset_count (look_ahead_set);

  return src_count;
}



static int
count_rr_conflicts (state *s, bool one_per_token)
{
  int i;
  reductions *reds = s->reductions;
  int rrc_count = 0;

  for (i = 0; i < ntokens; i++)
    {
      int count = 0;
      int j;
      for (j = 0; j < reds->num; ++j)
	if (bitset_test (reds->look_ahead_tokens[j], i))
	  count++;

      if (count >= 2)
	rrc_count += one_per_token ? 1 : count-1;
    }

  return rrc_count;
}



static void
conflict_report (FILE *out, int src_num, int rrc_num)
{
  if (src_num && rrc_num)
    fprintf (out, _("conflicts: %d shift/reduce, %d reduce/reduce\n"),
	     src_num, rrc_num);
  else if (src_num)
    fprintf (out, _("conflicts: %d shift/reduce\n"), src_num);
  else if (rrc_num)
    fprintf (out, _("conflicts: %d reduce/reduce\n"), rrc_num);
}



void
conflicts_output (FILE *out)
{
  bool printed_sth = false;
  state_number i;
  for (i = 0; i < nstates; i++)
    {
      state *s = states[i];
      if (conflicts[i])
	{
	  fprintf (out, _("State %d "), i);
	  conflict_report (out, count_sr_conflicts (s),
			   count_rr_conflicts (s, true));
	  printed_sth = true;
	}
    }
  if (printed_sth)
    fputs ("\n\n", out);
}


int
conflicts_total_count (void)
{
  state_number i;
  int count;

  /* Conflicts by state.  */
  count = 0;
  for (i = 0; i < nstates; i++)
    if (conflicts[i])
      {
	count += count_sr_conflicts (states[i]);
	count += count_rr_conflicts (states[i], false);
      }
  return count;
}



void
conflicts_print (void)
{
  /* Is the number of SR conflicts OK?  Either EXPECTED_CONFLICTS is
     not set, and then we want 0 SR, or else it is specified, in which
     case we want equality.  */
  bool src_ok;
  bool rrc_ok;

  int src_total = 0;
  int rrc_total = 0;
  int src_expected;
  int rrc_expected;

  /* Conflicts by state.  */
  {
    state_number i;

    for (i = 0; i < nstates; i++)
      if (conflicts[i])
	{
	  src_total += count_sr_conflicts (states[i]);
	  rrc_total += count_rr_conflicts (states[i], true);
	}
  }

  if (! glr_parser && rrc_total > 0 && expected_rr_conflicts != -1)
    {
      warn (_("%%expect-rr applies only to GLR parsers"));
      expected_rr_conflicts = -1;
    }

  src_expected = expected_sr_conflicts == -1 ? 0 : expected_sr_conflicts;
  rrc_expected = expected_rr_conflicts == -1 ? 0 : expected_rr_conflicts;
  src_ok = src_total == src_expected;
  rrc_ok = rrc_total == rrc_expected;

  /* If there are as many RR conflicts and SR conflicts as
     expected, then there is nothing to report.  */
  if (rrc_ok & src_ok)
    return;

  /* Report the total number of conflicts on STDERR.  */
  if (src_total | rrc_total)
    {
      if (! yacc_flag)
	fprintf (stderr, "%s: ", current_file);
      conflict_report (stderr, src_total, rrc_total);
    }

  if (expected_sr_conflicts != -1 || expected_rr_conflicts != -1)
    {
      if (! src_ok)
	complain (ngettext ("expected %d shift/reduce conflict",
			    "expected %d shift/reduce conflicts",
			    src_expected),
		  src_expected);
      if (! rrc_ok)
	complain (ngettext ("expected %d reduce/reduce conflict",
			    "expected %d reduce/reduce conflicts",
			    rrc_expected),
		  rrc_expected);
    }
}


void
conflicts_free (void)
{
  free (conflicts);
  bitset_free (shift_set);
  bitset_free (look_ahead_set);
  obstack_free (&solved_conflicts_obstack, NULL);
}
