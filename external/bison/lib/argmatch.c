

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

/* Specification.  */
#include "argmatch.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gettext.h"
#define _(msgid) gettext (msgid)

#include "error.h"
#include "exit.h"
#include "quotearg.h"
#include "quote.h"

#if USE_UNLOCKED_IO
# include "unlocked-io.h"
#endif

#ifndef ARGMATCH_QUOTING_STYLE
# define ARGMATCH_QUOTING_STYLE locale_quoting_style
#endif

/* Non failing version of argmatch call this function after failing. */
#ifndef ARGMATCH_DIE
# include "exitfail.h"
# define ARGMATCH_DIE exit (exit_failure)
#endif

#ifdef ARGMATCH_DIE_DECL
ARGMATCH_DIE_DECL;
#endif

static void
__argmatch_die (void)
{
  ARGMATCH_DIE;
}

argmatch_exit_fn argmatch_die = __argmatch_die;



ptrdiff_t
argmatch (const char *arg, const char *const *arglist,
	  const char *vallist, size_t valsize)
{
  size_t i;			/* Temporary index in ARGLIST.  */
  size_t arglen;		/* Length of ARG.  */
  ptrdiff_t matchind = -1;	/* Index of first nonexact match.  */
  bool ambiguous = false;	/* If true, multiple nonexact match(es).  */

  arglen = strlen (arg);

  /* Test all elements for either exact match or abbreviated matches.  */
  for (i = 0; arglist[i]; i++)
    {
      if (!strncmp (arglist[i], arg, arglen))
	{
	  if (strlen (arglist[i]) == arglen)
	    /* Exact match found.  */
	    return i;
	  else if (matchind == -1)
	    /* First nonexact match found.  */
	    matchind = i;
	  else
	    {
	      /* Second nonexact match found.  */
	      if (vallist == NULL
		  || memcmp (vallist + valsize * matchind,
			     vallist + valsize * i, valsize))
		{
		  /* There is a real ambiguity, or we could not
		     disambiguate. */
		  ambiguous = true;
		}
	    }
	}
    }
  if (ambiguous)
    return -2;
  else
    return matchind;
}


void
argmatch_invalid (const char *context, const char *value, ptrdiff_t problem)
{
  char const *format = (problem == -1
			? _("invalid argument %s for %s")
			: _("ambiguous argument %s for %s"));

  error (0, 0, format, quotearg_n_style (0, ARGMATCH_QUOTING_STYLE, value),
	 quote_n (1, context));
}

void
argmatch_valid (const char *const *arglist,
		const char *vallist, size_t valsize)
{
  size_t i;
  const char *last_val = NULL;

  /* We try to put synonyms on the same line.  The assumption is that
     synonyms follow each other */
  fprintf (stderr, _("Valid arguments are:"));
  for (i = 0; arglist[i]; i++)
    if ((i == 0)
	|| memcmp (last_val, vallist + valsize * i, valsize))
      {
	fprintf (stderr, "\n  - `%s'", arglist[i]);
	last_val = vallist + valsize * i;
      }
    else
      {
	fprintf (stderr, ", `%s'", arglist[i]);
      }
  putc ('\n', stderr);
}


ptrdiff_t
__xargmatch_internal (const char *context,
		      const char *arg, const char *const *arglist,
		      const char *vallist, size_t valsize,
		      argmatch_exit_fn exit_fn)
{
  ptrdiff_t res = argmatch (arg, arglist, vallist, valsize);
  if (res >= 0)
    /* Success. */
    return res;

  /* We failed.  Explain why. */
  argmatch_invalid (context, arg, res);
  argmatch_valid (arglist, vallist, valsize);
  (*exit_fn) ();

  return -1; /* To please the compilers. */
}

const char *
argmatch_to_argument (const char *value,
		      const char *const *arglist,
		      const char *vallist, size_t valsize)
{
  size_t i;

  for (i = 0; arglist[i]; i++)
    if (!memcmp (value, vallist + valsize * i, valsize))
      return arglist[i];
  return NULL;
}

#ifdef TEST
char *program_name;

/* When to make backup files.  */
enum backup_type
{
  /* Never make backups.  */
  no_backups,

  /* Make simple backups of every file.  */
  simple_backups,

  /* Make numbered backups of files that already have numbered backups,
     and simple backups of the others.  */
  numbered_existing_backups,

  /* Make numbered backups of every file.  */
  numbered_backups
};

static const char *const backup_args[] =
{
  "no", "none", "off",
  "simple", "never",
  "existing", "nil",
  "numbered", "t",
  0
};

static const enum backup_type backup_vals[] =
{
  no_backups, no_backups, no_backups,
  simple_backups, simple_backups,
  numbered_existing_backups, numbered_existing_backups,
  numbered_backups, numbered_backups
};

int
main (int argc, const char *const *argv)
{
  const char *cp;
  enum backup_type backup_type = no_backups;

  program_name = (char *) argv[0];

  if (argc > 2)
    {
      fprintf (stderr, "Usage: %s [VERSION_CONTROL]\n", program_name);
      exit (1);
    }

  if ((cp = getenv ("VERSION_CONTROL")))
    backup_type = XARGMATCH ("$VERSION_CONTROL", cp,
			     backup_args, backup_vals);

  if (argc == 2)
    backup_type = XARGMATCH (program_name, argv[1],
			     backup_args, backup_vals);

  printf ("The version control is `%s'\n",
	  ARGMATCH_TO_ARGUMENT (backup_type, backup_args, backup_vals));

  return 0;
}
#endif
