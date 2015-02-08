

#ifndef _NILFS_DIRECT_H
#define _NILFS_DIRECT_H

#include <linux/types.h>
#include <linux/buffer_head.h>
#include "bmap.h"


struct nilfs_direct;

struct nilfs_direct_node {
	__u8 dn_flags;
	__u8 pad[7];
};

struct nilfs_direct {
	struct nilfs_bmap d_bmap;
};


#define NILFS_DIRECT_NBLOCKS	(NILFS_BMAP_SIZE / sizeof(__le64) - 1)
#define NILFS_DIRECT_KEY_MIN	0
#define NILFS_DIRECT_KEY_MAX	(NILFS_DIRECT_NBLOCKS - 1)


int nilfs_direct_init(struct nilfs_bmap *);
int nilfs_direct_delete_and_convert(struct nilfs_bmap *, __u64, __u64 *,
				    __u64 *, int);


#endif	/* _NILFS_DIRECT_H */
