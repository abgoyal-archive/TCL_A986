
#ifndef	__XFS_SUPPORT_DEBUG_H__
#define	__XFS_SUPPORT_DEBUG_H__

#include <stdarg.h>

#define CE_DEBUG        7               /* debug        */
#define CE_CONT         6               /* continuation */
#define CE_NOTE         5               /* notice       */
#define CE_WARN         4               /* warning      */
#define CE_ALERT        1               /* alert        */
#define CE_PANIC        0               /* panic        */

extern void cmn_err(int, char *, ...)
	__attribute__ ((format (printf, 2, 3)));
extern void assfail(char *expr, char *f, int l);

#define ASSERT_ALWAYS(expr)	\
	(unlikely(expr) ? (void)0 : assfail(#expr, __FILE__, __LINE__))

#ifndef DEBUG
#define ASSERT(expr)	((void)0)

#ifndef STATIC
# define STATIC static noinline
#endif

#else /* DEBUG */

#define ASSERT(expr)	\
	(unlikely(expr) ? (void)0 : assfail(#expr, __FILE__, __LINE__))

#ifndef STATIC
# define STATIC noinline
#endif

#endif /* DEBUG */
#endif  /* __XFS_SUPPORT_DEBUG_H__ */
