

#include <linux/namei.h>

#include "exofs.h"

static void *exofs_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct exofs_i_info *oi = exofs_i(dentry->d_inode);

	nd_set_link(nd, (char *)oi->i_data);
	return NULL;
}

const struct inode_operations exofs_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
};

const struct inode_operations exofs_fast_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= exofs_follow_link,
};
