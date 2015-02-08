

#ifndef _LINUX_QUOTA_TREE_H
#define _LINUX_QUOTA_TREE_H

#include <linux/types.h>
#include <linux/quota.h>

struct qt_disk_dqdbheader {
	__le32 dqdh_next_free;	/* Number of next block with free entry */
	__le32 dqdh_prev_free;	/* Number of previous block with free entry */
	__le16 dqdh_entries;	/* Number of valid entries in block */
	__le16 dqdh_pad1;
	__le32 dqdh_pad2;
};

#define QT_TREEOFF	1		/* Offset of tree in file in blocks */

#define q_warn(fmt, args...) \
do { \
	if (printk_ratelimit()) \
		printk(fmt, ## args); \
} while(0)

#endif /* _LINUX_QUOTAIO_TREE_H */
