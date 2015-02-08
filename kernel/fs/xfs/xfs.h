
#ifndef __XFS_H__
#define __XFS_H__

#ifdef CONFIG_XFS_DEBUG
#define STATIC
#define DEBUG 1
#define XFS_BUF_LOCK_TRACKING 1
/* #define QUOTADEBUG 1 */
#endif

#include <linux-2.6/xfs_linux.h>
#endif	/* __XFS_H__ */
