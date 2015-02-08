

#include "ext2.h"
#include "xattr.h"
#include <linux/namei.h>

static void *ext2_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct ext2_inode_info *ei = EXT2_I(dentry->d_inode);
	nd_set_link(nd, (char *)ei->i_data);
	return NULL;
}

const struct inode_operations ext2_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
	.setattr	= ext2_setattr,
#ifdef CONFIG_EXT2_FS_XATTR
	.setxattr	= generic_setxattr,
	.getxattr	= generic_getxattr,
	.listxattr	= ext2_listxattr,
	.removexattr	= generic_removexattr,
#endif
};
 
const struct inode_operations ext2_fast_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= ext2_follow_link,
	.setattr	= ext2_setattr,
#ifdef CONFIG_EXT2_FS_XATTR
	.setxattr	= generic_setxattr,
	.getxattr	= generic_getxattr,
	.listxattr	= ext2_listxattr,
	.removexattr	= generic_removexattr,
#endif
};
