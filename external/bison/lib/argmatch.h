

#ifndef ARGMATCH_H_
# define ARGMATCH_H_ 1

# include <stddef.h>

# include "verify.h"

# define ARRAY_CARDINALITY(Array) (sizeof (Array) / sizeof *(Array))


# define ARGMATCH_VERIFY(Arglist, Vallist) \
    verify (ARRAY_CARDINALITY (Arglist) == ARRAY_CARDINALITY (Vallist) + 1)


ptrdiff_t argmatch (char const *arg, char const *const *arglist,
		    char const *vallist, size_t valsize);

# define ARGMATCH(Arg, Arglist, Vallist) \
  argmatch (Arg, Arglist, (char const *) (Vallist), sizeof *(Vallist))

typedef void (*argmatch_exit_fn) (void);
extern argmatch_exit_fn argmatch_die;

/* Report on stderr why argmatch failed.  Report correct values. */

void argmatch_invalid (char const *context, char const *value,
		       ptrdiff_t problem);

/* Left for compatibility with the old name invalid_arg */

# define invalid_arg(Context, Value, Problem) \
  argmatch_invalid (Context, Value, Problem)



/* Report on stderr the list of possible arguments.  */

void argmatch_valid (char const *const *arglist,
		     char const *vallist, size_t valsize);

# define ARGMATCH_VALID(Arglist, Vallist) \
  argmatch_valid (Arglist, (char const *) (Vallist), sizeof *(Vallist))




ptrdiff_t __xargmatch_internal (char const *context,
				char const *arg, char const *const *arglist,
				char const *vallist, size_t valsize,
				argmatch_exit_fn exit_fn);

/* Programmer friendly interface to __xargmatch_internal. */

# define XARGMATCH(Context, Arg, Arglist, Vallist)		\
  ((Vallist) [__xargmatch_internal (Context, Arg, Arglist,	\
				    (char const *) (Vallist),	\
				    sizeof *(Vallist),		\
				    argmatch_die)])

/* Convert a value into a corresponding argument. */

char const *argmatch_to_argument (char const *value,
				  char const *const *arglist,
				  char const *vallist, size_t valsize);

# define ARGMATCH_TO_ARGUMENT(Value, Arglist, Vallist)			\
  argmatch_to_argument (Value, Arglist,					\
			(char const *) (Vallist), sizeof *(Vallist))

#endif /* ARGMATCH_H_ */
