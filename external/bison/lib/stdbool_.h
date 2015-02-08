
#ifndef _STDBOOL_H
#define _STDBOOL_H

/* ISO C 99 <stdbool.h> for platforms that lack it.  */



/* 7.16. Boolean type and values */

#ifdef __BEOS__
# include <OS.h> /* defines bool but not _Bool */
# undef false
# undef true
#endif

#if defined __cplusplus || defined __BEOS__
  /* A compiler known to have 'bool'.  */
  /* If the compiler already has both 'bool' and '_Bool', we can assume they
     are the same types.  */
# if !@HAVE__BOOL@
typedef bool _Bool;
# endif
#else
# if !defined __GNUC__
   /* If @HAVE__BOOL@:
        Some HP-UX cc and AIX IBM C compiler versions have compiler bugs when
        the built-in _Bool type is used.  See
          http://gcc.gnu.org/ml/gcc-patches/2003-12/msg02303.html
          http://lists.gnu.org/archive/html/bug-coreutils/2005-11/msg00161.html
          http://lists.gnu.org/archive/html/bug-coreutils/2005-10/msg00086.html
        Similar bugs are likely with other compilers as well; this file
        wouldn't be used if <stdbool.h> was working.
        So we override the _Bool type.
      If !@HAVE__BOOL@:
        Need to define _Bool ourselves. As 'signed char' or as an enum type?
        Use of a typedef, with SunPRO C, leads to a stupid
          "warning: _Bool is a keyword in ISO C99".
        Use of an enum type, with IRIX cc, leads to a stupid
          "warning(1185): enumerated type mixed with another type".
        The only benefit of the enum type, debuggability, is not important
        with these compilers.  So use 'signed char' and no typedef.  */
#  define _Bool signed char
enum { false = 0, true = 1 };
# else
   /* With this compiler, trust the _Bool type if the compiler has it.  */
#  if !@HAVE__BOOL@
typedef enum { _Bool_must_promote_to_int = -1, false = 0, true = 1 } _Bool;
#  endif
# endif
#endif
#define bool _Bool

/* The other macros must be usable in preprocessor directives.  */
#define false 0
#define true 1
#define __bool_true_false_are_defined 1

#endif /* _STDBOOL_H */
