
#ifndef BISON_SYSTEM_H
#define BISON_SYSTEM_H

#if ! defined __STDC_VERSION__ || __STDC_VERSION__ < 199901
# undef INT8_MIN
# undef INT16_MIN
# undef INT32_MIN
# undef INT8_MAX
# undef INT16_MAX
# undef UINT8_MAX
# undef INT32_MAX
# undef UINT16_MAX
# undef UINT32_MAX
#endif

#include <limits.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "unlocked-io.h"

#if HAVE_SYS_TYPES_H
# include <sys/types.h>
#endif

#if HAVE_UNISTD_H
# include <unistd.h>
#endif

#if HAVE_INTTYPES_H
# include <inttypes.h>
#endif
#if HAVE_STDINT_H
# include <stdint.h>
#endif

#if ! HAVE_UINTPTR_T
typedef size_t uintptr_t;
#endif

#include <verify.h>
#include <xalloc.h>



#include <stpcpy.h>

/* From lib/basename.c. */
char *base_name (char const *name);



#ifdef lint
# define IF_LINT(Code) Code
#else
# define IF_LINT(Code) /* empty */
#endif

#ifndef __attribute__
/* This feature is available in gcc versions 2.5 and later.  */
# if (! defined __GNUC__ || __GNUC__ < 2 \
      || (__GNUC__ == 2 && __GNUC_MINOR__ < 5) || __STRICT_ANSI__)
#  define __attribute__(Spec) /* empty */
# endif
#endif

#if __GNUC__ < 2 || (__GNUC__ == 2 && __GNUC_MINOR__ < 7)
# define __format__ format
# define __printf__ printf
#endif

#ifndef ATTRIBUTE_NORETURN
# define ATTRIBUTE_NORETURN __attribute__ ((__noreturn__))
#endif

#ifndef ATTRIBUTE_UNUSED
# define ATTRIBUTE_UNUSED __attribute__ ((__unused__))
#endif


#include <locale.h>

#include <gettext.h>
#define _(Msgid)  gettext (Msgid)
#define N_(Msgid) (Msgid)



#ifndef LOCALEDIR
# define LOCALEDIR "/usr/local/share/locale"
#endif



#include <stdbool.h>



#define obstack_chunk_alloc xmalloc
#define obstack_chunk_free  free
#include <obstack.h>

#define obstack_sgrow(Obs, Str) \
  obstack_grow (Obs, Str, strlen (Str))

#define obstack_fgrow1(Obs, Format, Arg1)	\
do {						\
  char buf[4096];				\
  sprintf (buf, Format, Arg1);			\
  obstack_grow (Obs, buf, strlen (buf));	\
} while (0)

#define obstack_fgrow2(Obs, Format, Arg1, Arg2)	\
do {						\
  char buf[4096];				\
  sprintf (buf, Format, Arg1, Arg2);		\
  obstack_grow (Obs, buf, strlen (buf));	\
} while (0)

#define obstack_fgrow3(Obs, Format, Arg1, Arg2, Arg3)	\
do {							\
  char buf[4096];					\
  sprintf (buf, Format, Arg1, Arg2, Arg3);		\
  obstack_grow (Obs, buf, strlen (buf));		\
} while (0)

#define obstack_fgrow4(Obs, Format, Arg1, Arg2, Arg3, Arg4)	\
do {								\
  char buf[4096];						\
  sprintf (buf, Format, Arg1, Arg2, Arg3, Arg4);		\
  obstack_grow (Obs, buf, strlen (buf));			\
} while (0)




#ifndef OUTPUT_EXT
# define OUTPUT_EXT ".output"
#endif

#ifndef TAB_EXT
# define TAB_EXT ".tab"
#endif

#ifndef DEFAULT_TMPDIR
# define DEFAULT_TMPDIR "/tmp"
#endif




#define LIST_FREE(Type, List)			\
do {						\
  Type *_node, *_next;				\
  for (_node = List; _node; _node = _next)	\
    {						\
      _next = _node->next;			\
      free (_node);				\
    }						\
} while (0)


#define assert(x) ((void) ((x) || (abort (), 0)))



# if WITH_DMALLOC
#  define DMALLOC_FUNC_CHECK
#  include <dmalloc.h>
# endif /* WITH_DMALLOC */

#endif  /* ! BISON_SYSTEM_H */
