

#include <linux/fs.h>
#include <linux/namei.h>
#include "jfs_incore.h"
#include "jfs_inode.h"
#include "jfs_xattr.h"

static void *jfs_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	char *s = JFS_IP(dentry->d_inode)->i_inline;
	nd_set_link(nd, s);
	return NULL;
}

const struct inode_operations jfs_fast_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= jfs_follow_link,
	.setattr	= jfs_setattr,
	.setxattr	= jfs_setxattr,
	.getxattr	= jfs_getxattr,
	.listxattr	= jfs_listxattr,
	.removexattr	= jfs_removexattr,
};

const struct inode_operations jfs_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
	.setattr	= jfs_setattr,
	.setxattr	= jfs_setxattr,
	.getxattr	= jfs_getxattr,
	.listxattr	= jfs_listxattr,
	.removexattr	= jfs_removexattr,
};

