
#ifndef COMPLAIN_H_
# define COMPLAIN_H_ 1

# include "location.h"

# ifdef	__cplusplus
extern "C" {
# endif

/* Informative messages, but we proceed.  */

void warn (char const *format, ...)
  __attribute__ ((__format__ (__printf__, 1, 2)));

void warn_at (location loc, char const *format, ...)
  __attribute__ ((__format__ (__printf__, 2, 3)));

/* Something bad happened, but let's continue and die later.  */

void complain (char const *format, ...)
  __attribute__ ((__format__ (__printf__, 1, 2)));

void complain_at (location loc, char const *format, ...)
  __attribute__ ((__format__ (__printf__, 2, 3)));

/* Something bad happened, and let's die now.  */

void fatal (char const *format, ...)
  __attribute__ ((__noreturn__, __format__ (__printf__, 1, 2)));

void fatal_at (location loc, char const *format, ...)
  __attribute__ ((__noreturn__, __format__ (__printf__, 2, 3)));

/* This variable is set each time `warn' is called.  */
extern bool warning_issued;

/* This variable is set each time `complain' is called.  */
extern bool complaint_issued;

# ifdef	__cplusplus
}
# endif

#endif /* !COMPLAIN_H_ */
