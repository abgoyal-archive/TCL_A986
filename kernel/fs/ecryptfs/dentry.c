

#include <linux/dcache.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/fs_stack.h>
#include <linux/slab.h>
#include "ecryptfs_kernel.h"

static int ecryptfs_d_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	struct dentry *lower_dentry = ecryptfs_dentry_to_lower(dentry);
	struct vfsmount *lower_mnt = ecryptfs_dentry_to_lower_mnt(dentry);
	struct dentry *dentry_save;
	struct vfsmount *vfsmount_save;
	int rc = 1;

	if (!lower_dentry->d_op || !lower_dentry->d_op->d_revalidate)
		goto out;
	dentry_save = nd->path.dentry;
	vfsmount_save = nd->path.mnt;
	nd->path.dentry = lower_dentry;
	nd->path.mnt = lower_mnt;
	rc = lower_dentry->d_op->d_revalidate(lower_dentry, nd);
	nd->path.dentry = dentry_save;
	nd->path.mnt = vfsmount_save;
	if (dentry->d_inode) {
		struct inode *lower_inode =
			ecryptfs_inode_to_lower(dentry->d_inode);

		fsstack_copy_attr_all(dentry->d_inode, lower_inode);
	}
out:
	return rc;
}

struct kmem_cache *ecryptfs_dentry_info_cache;

static void ecryptfs_d_release(struct dentry *dentry)
{
	if (ecryptfs_dentry_to_private(dentry)) {
		if (ecryptfs_dentry_to_lower(dentry)) {
			dput(ecryptfs_dentry_to_lower(dentry));
			mntput(ecryptfs_dentry_to_lower_mnt(dentry));
		}
		kmem_cache_free(ecryptfs_dentry_info_cache,
				ecryptfs_dentry_to_private(dentry));
	}
	return;
}

const struct dentry_operations ecryptfs_dops = {
	.d_revalidate = ecryptfs_d_revalidate,
	.d_release = ecryptfs_d_release,
};
