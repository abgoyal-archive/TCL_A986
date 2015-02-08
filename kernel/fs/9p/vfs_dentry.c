

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/pagemap.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/inet.h>
#include <linux/namei.h>
#include <linux/idr.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <net/9p/9p.h>
#include <net/9p/client.h>

#include "v9fs.h"
#include "v9fs_vfs.h"
#include "fid.h"


static int v9fs_dentry_delete(struct dentry *dentry)
{
	P9_DPRINTK(P9_DEBUG_VFS, " dentry: %s (%p)\n", dentry->d_name.name,
									dentry);

	return 1;
}


static int v9fs_cached_dentry_delete(struct dentry *dentry)
{
	struct inode *inode = dentry->d_inode;
	P9_DPRINTK(P9_DEBUG_VFS, " dentry: %s (%p)\n", dentry->d_name.name,
									dentry);

	if(!inode)
		return 1;

	return 0;
}


void v9fs_dentry_release(struct dentry *dentry)
{
	struct v9fs_dentry *dent;
	struct p9_fid *temp, *current_fid;

	P9_DPRINTK(P9_DEBUG_VFS, " dentry: %s (%p)\n", dentry->d_name.name,
									dentry);
	dent = dentry->d_fsdata;
	if (dent) {
		list_for_each_entry_safe(current_fid, temp, &dent->fidlist,
									dlist) {
			p9_client_clunk(current_fid);
		}

		kfree(dent);
		dentry->d_fsdata = NULL;
	}
}

const struct dentry_operations v9fs_cached_dentry_operations = {
	.d_delete = v9fs_cached_dentry_delete,
	.d_release = v9fs_dentry_release,
};

const struct dentry_operations v9fs_dentry_operations = {
	.d_delete = v9fs_dentry_delete,
	.d_release = v9fs_dentry_release,
};
