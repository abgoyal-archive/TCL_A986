

#include <linux/fs.h>
#include <linux/namei.h>

#include "ufs_fs.h"
#include "ufs.h"


static void *ufs_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct ufs_inode_info *p = UFS_I(dentry->d_inode);
	nd_set_link(nd, (char*)p->i_u1.i_symlink);
	return NULL;
}

const struct inode_operations ufs_fast_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= ufs_follow_link,
	.setattr	= ufs_setattr,
};

const struct inode_operations ufs_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
	.setattr	= ufs_setattr,
};
