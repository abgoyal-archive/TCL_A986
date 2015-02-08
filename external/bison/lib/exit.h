
#ifndef _EXIT_H
#define _EXIT_H

/* Get exit() declaration.  */
#include <stdlib.h>

/* Some systems do not define EXIT_*, even with STDC_HEADERS.  */
#ifndef EXIT_SUCCESS
# define EXIT_SUCCESS 0
#endif
#ifndef EXIT_FAILURE
# define EXIT_FAILURE 1
#endif

#endif /* _EXIT_H */
