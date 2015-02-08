

#include <linux/fs.h>
#include <linux/jbd.h>
#include <linux/ext3_fs.h>
#include <linux/namei.h>
#include "xattr.h"

static void * ext3_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct ext3_inode_info *ei = EXT3_I(dentry->d_inode);
	nd_set_link(nd, (char*)ei->i_data);
	return NULL;
}

const struct inode_operations ext3_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
	.setattr	= ext3_setattr,
#ifdef CONFIG_EXT3_FS_XATTR
	.setxattr	= generic_setxattr,
	.getxattr	= generic_getxattr,
	.listxattr	= ext3_listxattr,
	.removexattr	= generic_removexattr,
#endif
};

const struct inode_operations ext3_fast_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= ext3_follow_link,
	.setattr	= ext3_setattr,
#ifdef CONFIG_EXT3_FS_XATTR
	.setxattr	= generic_setxattr,
	.getxattr	= generic_getxattr,
	.listxattr	= ext3_listxattr,
	.removexattr	= generic_removexattr,
#endif
};
