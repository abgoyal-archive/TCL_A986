
#if defined __EMX__
# include "os2compat.c"
#else
/* Avoid AIX compiler warning.  */
typedef int dummy;
#endif
