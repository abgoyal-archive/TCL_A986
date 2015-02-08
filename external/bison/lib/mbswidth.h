
#include <stddef.h>

#if HAVE_DECL_MBSWIDTH_IN_WCHAR_H
# include <wchar.h>
#endif


#ifdef	__cplusplus
extern "C" {
#endif


/* Optional flags to influence mbswidth/mbsnwidth behavior.  */

#define MBSW_REJECT_INVALID 1

#define MBSW_REJECT_UNPRINTABLE	2


/* Returns the number of screen columns needed for STRING.  */
#define mbswidth gnu_mbswidth  /* avoid clash with UnixWare 7.1.1 function */
extern int mbswidth (const char *string, int flags);

extern int mbsnwidth (const char *buf, size_t nbytes, int flags);


#ifdef	__cplusplus
}
#endif
