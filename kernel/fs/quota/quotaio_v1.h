
#ifndef _LINUX_QUOTAIO_V1_H
#define _LINUX_QUOTAIO_V1_H

#include <linux/types.h>

#define MAX_IQ_TIME  604800	/* (7*24*60*60) 1 week */
#define MAX_DQ_TIME  604800	/* (7*24*60*60) 1 week */

struct v1_disk_dqblk {
	__u32 dqb_bhardlimit;	/* absolute limit on disk blks alloc */
	__u32 dqb_bsoftlimit;	/* preferred limit on disk blks */
	__u32 dqb_curblocks;	/* current block count */
	__u32 dqb_ihardlimit;	/* absolute limit on allocated inodes */
	__u32 dqb_isoftlimit;	/* preferred inode limit */
	__u32 dqb_curinodes;	/* current # allocated inodes */
	time_t dqb_btime;	/* time limit for excessive disk use */
	time_t dqb_itime;	/* time limit for excessive inode use */
};

#define v1_dqoff(UID)      ((loff_t)((UID) * sizeof (struct v1_disk_dqblk)))

#endif	/* _LINUX_QUOTAIO_V1_H */
