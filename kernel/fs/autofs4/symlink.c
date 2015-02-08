

#include "autofs_i.h"

static void *autofs4_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct autofs_info *ino = autofs4_dentry_ino(dentry);
	nd_set_link(nd, (char *)ino->u.symlink);
	return NULL;
}

const struct inode_operations autofs4_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= autofs4_follow_link
};
