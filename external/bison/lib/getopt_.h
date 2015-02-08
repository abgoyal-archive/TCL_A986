
#ifndef _GETOPT_H

#ifndef __need_getopt
# define _GETOPT_H 1
#endif

#if defined __GETOPT_PREFIX && !defined __need_getopt
# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# undef __need_getopt
# undef getopt
# undef getopt_long
# undef getopt_long_only
# undef optarg
# undef opterr
# undef optind
# undef optopt
# define __GETOPT_CONCAT(x, y) x ## y
# define __GETOPT_XCONCAT(x, y) __GETOPT_CONCAT (x, y)
# define __GETOPT_ID(y) __GETOPT_XCONCAT (__GETOPT_PREFIX, y)
# define getopt __GETOPT_ID (getopt)
# define getopt_long __GETOPT_ID (getopt_long)
# define getopt_long_only __GETOPT_ID (getopt_long_only)
# define optarg __GETOPT_ID (optarg)
# define opterr __GETOPT_ID (opterr)
# define optind __GETOPT_ID (optind)
# define optopt __GETOPT_ID (optopt)
#endif

#if !defined __need_getopt
# if defined __GETOPT_PREFIX
#  define __getopt_argv_const /* empty */
# else
#  define __getopt_argv_const const
# endif
#endif

#if !defined __GNU_LIBRARY__
# include <ctype.h>
#endif

#ifndef __THROW
# ifndef __GNUC_PREREQ
#  define __GNUC_PREREQ(maj, min) (0)
# endif
# if defined __cplusplus && __GNUC_PREREQ (2,8)
#  define __THROW	throw ()
# else
#  define __THROW
# endif
#endif

#ifdef	__cplusplus
extern "C" {
#endif


extern char *optarg;


extern int optind;


extern int opterr;

/* Set to an option character which was unrecognized.  */

extern int optopt;

#ifndef __need_getopt

struct option
{
  const char *name;
  /* has_arg can't be an enum because some compilers complain about
     type mismatches in all the code that assumes it is an int.  */
  int has_arg;
  int *flag;
  int val;
};

/* Names for the values of the `has_arg' field of `struct option'.  */

# define no_argument		0
# define required_argument	1
# define optional_argument	2
#endif	/* need getopt */



extern int getopt (int ___argc, char *const *___argv, const char *__shortopts)
       __THROW;

#ifndef __need_getopt
extern int getopt_long (int ___argc, char *__getopt_argv_const *___argv,
			const char *__shortopts,
		        const struct option *__longopts, int *__longind)
       __THROW;
extern int getopt_long_only (int ___argc, char *__getopt_argv_const *___argv,
			     const char *__shortopts,
		             const struct option *__longopts, int *__longind)
       __THROW;

#endif

#ifdef	__cplusplus
}
#endif

/* Make sure we later can get all the definitions and declarations.  */
#undef __need_getopt

#endif /* getopt.h */
