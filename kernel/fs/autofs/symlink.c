

#include "autofs_i.h"

/* Nothing to release.. */
static void *autofs_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	char *s=((struct autofs_symlink *)dentry->d_inode->i_private)->data;
	nd_set_link(nd, s);
	return NULL;
}

const struct inode_operations autofs_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= autofs_follow_link
};
