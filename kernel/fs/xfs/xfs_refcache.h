
#ifndef __XFS_REFCACHE_H__
#define __XFS_REFCACHE_H__

#ifdef HAVE_REFCACHE
#define XFS_REFCACHE_SIZE_MAX	512

struct xfs_inode;
struct xfs_mount;

extern void xfs_refcache_insert(struct xfs_inode *);
extern void xfs_refcache_purge_ip(struct xfs_inode *);
extern void xfs_refcache_purge_mp(struct xfs_mount *);
extern void xfs_refcache_purge_some(struct xfs_mount *);
extern void xfs_refcache_resize(int);
extern void xfs_refcache_destroy(void);

extern void xfs_refcache_iunlock(struct xfs_inode *, uint);

#else

#define xfs_refcache_insert(ip)		do { } while (0)
#define xfs_refcache_purge_ip(ip)	do { } while (0)
#define xfs_refcache_purge_mp(mp)	do { } while (0)
#define xfs_refcache_purge_some(mp)	do { } while (0)
#define xfs_refcache_resize(size)	do { } while (0)
#define xfs_refcache_destroy()		do { } while (0)

#define xfs_refcache_iunlock(ip, flags)	xfs_iunlock(ip, flags)

#endif

#endif	/* __XFS_REFCACHE_H__ */
