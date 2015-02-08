
#ifndef __XFS_SYSCTL_H__
#define __XFS_SYSCTL_H__

#include <linux/sysctl.h>


typedef struct xfs_sysctl_val {
	int min;
	int val;
	int max;
} xfs_sysctl_val_t;

typedef struct xfs_param {
	xfs_sysctl_val_t sgid_inherit;	/* Inherit S_ISGID if process' GID is
					 * not a member of parent dir GID. */
	xfs_sysctl_val_t symlink_mode;	/* Link creat mode affected by umask */
	xfs_sysctl_val_t panic_mask;	/* bitmask to cause panic on errors. */
	xfs_sysctl_val_t error_level;	/* Degree of reporting for problems  */
	xfs_sysctl_val_t syncd_timer;	/* Interval between xfssyncd wakeups */
	xfs_sysctl_val_t stats_clear;	/* Reset all XFS statistics to zero. */
	xfs_sysctl_val_t inherit_sync;	/* Inherit the "sync" inode flag. */
	xfs_sysctl_val_t inherit_nodump;/* Inherit the "nodump" inode flag. */
	xfs_sysctl_val_t inherit_noatim;/* Inherit the "noatime" inode flag. */
	xfs_sysctl_val_t xfs_buf_timer;	/* Interval between xfsbufd wakeups. */
	xfs_sysctl_val_t xfs_buf_age;	/* Metadata buffer age before flush. */
	xfs_sysctl_val_t inherit_nosym;	/* Inherit the "nosymlinks" flag. */
	xfs_sysctl_val_t rotorstep;	/* inode32 AG rotoring control knob */
	xfs_sysctl_val_t inherit_nodfrg;/* Inherit the "nodefrag" inode flag. */
	xfs_sysctl_val_t fstrm_timer;	/* Filestream dir-AG assoc'n timeout. */
} xfs_param_t;


enum {
	/* XFS_REFCACHE_SIZE = 1 */
	/* XFS_REFCACHE_PURGE = 2 */
	/* XFS_RESTRICT_CHOWN = 3 */
	XFS_SGID_INHERIT = 4,
	XFS_SYMLINK_MODE = 5,
	XFS_PANIC_MASK = 6,
	XFS_ERRLEVEL = 7,
	XFS_SYNCD_TIMER = 8,
	/* XFS_PROBE_DMAPI = 9 */
	/* XFS_PROBE_IOOPS = 10 */
	/* XFS_PROBE_QUOTA = 11 */
	XFS_STATS_CLEAR = 12,
	XFS_INHERIT_SYNC = 13,
	XFS_INHERIT_NODUMP = 14,
	XFS_INHERIT_NOATIME = 15,
	XFS_BUF_TIMER = 16,
	XFS_BUF_AGE = 17,
	/* XFS_IO_BYPASS = 18 */
	XFS_INHERIT_NOSYM = 19,
	XFS_ROTORSTEP = 20,
	XFS_INHERIT_NODFRG = 21,
	XFS_FILESTREAM_TIMER = 22,
};

extern xfs_param_t	xfs_params;

#ifdef CONFIG_SYSCTL
extern int xfs_sysctl_register(void);
extern void xfs_sysctl_unregister(void);
#else
# define xfs_sysctl_register()		(0)
# define xfs_sysctl_unregister()	do { } while (0)
#endif /* CONFIG_SYSCTL */

#endif /* __XFS_SYSCTL_H__ */
