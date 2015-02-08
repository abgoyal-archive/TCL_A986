
#ifndef __XFS_DMAPI_H__
#define __XFS_DMAPI_H__


#define DMATTR_PREFIXLEN	8
#define DMATTR_PREFIXSTRING	"SGI_DMI_"

typedef enum {
	DM_EVENT_INVALID	= -1,
	DM_EVENT_CANCEL		= 0,		/* not supported */
	DM_EVENT_MOUNT		= 1,
	DM_EVENT_PREUNMOUNT	= 2,
	DM_EVENT_UNMOUNT	= 3,
	DM_EVENT_DEBUT		= 4,		/* not supported */
	DM_EVENT_CREATE		= 5,
	DM_EVENT_CLOSE		= 6,		/* not supported */
	DM_EVENT_POSTCREATE	= 7,
	DM_EVENT_REMOVE		= 8,
	DM_EVENT_POSTREMOVE	= 9,
	DM_EVENT_RENAME		= 10,
	DM_EVENT_POSTRENAME	= 11,
	DM_EVENT_LINK		= 12,
	DM_EVENT_POSTLINK	= 13,
	DM_EVENT_SYMLINK	= 14,
	DM_EVENT_POSTSYMLINK	= 15,
	DM_EVENT_READ		= 16,
	DM_EVENT_WRITE		= 17,
	DM_EVENT_TRUNCATE	= 18,
	DM_EVENT_ATTRIBUTE	= 19,
	DM_EVENT_DESTROY	= 20,
	DM_EVENT_NOSPACE	= 21,
	DM_EVENT_USER		= 22,
	DM_EVENT_MAX		= 23
} dm_eventtype_t;
#define HAVE_DM_EVENTTYPE_T

typedef enum {
	DM_RIGHT_NULL,
	DM_RIGHT_SHARED,
	DM_RIGHT_EXCL
} dm_right_t;
#define HAVE_DM_RIGHT_T

/* Defines for determining if an event message should be sent. */
#ifdef HAVE_DMAPI
#define	DM_EVENT_ENABLED(ip, event) ( \
	unlikely ((ip)->i_mount->m_flags & XFS_MOUNT_DMAPI) && \
		( ((ip)->i_d.di_dmevmask & (1 << event)) || \
		  ((ip)->i_mount->m_dmevmask & (1 << event)) ) \
	)
#else
#define DM_EVENT_ENABLED(ip, event)	(0)
#endif

#define DM_XFS_VALID_FS_EVENTS		( \
	(1 << DM_EVENT_PREUNMOUNT)	| \
	(1 << DM_EVENT_UNMOUNT)		| \
	(1 << DM_EVENT_NOSPACE)		| \
	(1 << DM_EVENT_DEBUT)		| \
	(1 << DM_EVENT_CREATE)		| \
	(1 << DM_EVENT_POSTCREATE)	| \
	(1 << DM_EVENT_REMOVE)		| \
	(1 << DM_EVENT_POSTREMOVE)	| \
	(1 << DM_EVENT_RENAME)		| \
	(1 << DM_EVENT_POSTRENAME)	| \
	(1 << DM_EVENT_LINK)		| \
	(1 << DM_EVENT_POSTLINK)	| \
	(1 << DM_EVENT_SYMLINK)		| \
	(1 << DM_EVENT_POSTSYMLINK)	| \
	(1 << DM_EVENT_ATTRIBUTE)	| \
	(1 << DM_EVENT_DESTROY)		)


#define	DM_XFS_VALID_FILE_EVENTS	( \
	(1 << DM_EVENT_ATTRIBUTE)	| \
	(1 << DM_EVENT_DESTROY)		)


#define	DM_XFS_VALID_DIRECTORY_EVENTS	( \
	(1 << DM_EVENT_CREATE)		| \
	(1 << DM_EVENT_POSTCREATE)	| \
	(1 << DM_EVENT_REMOVE)		| \
	(1 << DM_EVENT_POSTREMOVE)	| \
	(1 << DM_EVENT_RENAME)		| \
	(1 << DM_EVENT_POSTRENAME)	| \
	(1 << DM_EVENT_LINK)		| \
	(1 << DM_EVENT_POSTLINK)	| \
	(1 << DM_EVENT_SYMLINK)		| \
	(1 << DM_EVENT_POSTSYMLINK)	| \
	(1 << DM_EVENT_ATTRIBUTE)	| \
	(1 << DM_EVENT_DESTROY)		)

/* Events supported by the XFS filesystem. */
#define	DM_XFS_SUPPORTED_EVENTS		( \
	(1 << DM_EVENT_MOUNT)		| \
	(1 << DM_EVENT_PREUNMOUNT)	| \
	(1 << DM_EVENT_UNMOUNT)		| \
	(1 << DM_EVENT_NOSPACE)		| \
	(1 << DM_EVENT_CREATE)		| \
	(1 << DM_EVENT_POSTCREATE)	| \
	(1 << DM_EVENT_REMOVE)		| \
	(1 << DM_EVENT_POSTREMOVE)	| \
	(1 << DM_EVENT_RENAME)		| \
	(1 << DM_EVENT_POSTRENAME)	| \
	(1 << DM_EVENT_LINK)		| \
	(1 << DM_EVENT_POSTLINK)	| \
	(1 << DM_EVENT_SYMLINK)		| \
	(1 << DM_EVENT_POSTSYMLINK)	| \
	(1 << DM_EVENT_READ)		| \
	(1 << DM_EVENT_WRITE)		| \
	(1 << DM_EVENT_TRUNCATE)	| \
	(1 << DM_EVENT_ATTRIBUTE)	| \
	(1 << DM_EVENT_DESTROY)		)



#define DM_FLAGS_NDELAY		0x001	/* return EAGAIN after dm_pending() */
#define DM_FLAGS_UNWANTED	0x002	/* event not in fsys dm_eventset_t */
#define DM_FLAGS_IMUX		0x004	/* thread holds i_mutex */
#define DM_FLAGS_IALLOCSEM_RD	0x010	/* thread holds i_alloc_sem rd */
#define DM_FLAGS_IALLOCSEM_WR	0x020	/* thread holds i_alloc_sem wr */

#include "xfs_dmapi_priv.h"


#define FILP_DELAY_FLAG(filp) ((filp->f_flags&(O_NDELAY|O_NONBLOCK)) ? \
			DM_FLAGS_NDELAY : 0)
#define AT_DELAY_FLAG(f) ((f & XFS_ATTR_NONBLOCK) ? DM_FLAGS_NDELAY : 0)

#endif  /* __XFS_DMAPI_H__ */
