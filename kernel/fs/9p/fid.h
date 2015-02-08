

#include <linux/list.h>

struct v9fs_dentry {
	spinlock_t lock; /* protect fidlist */
	struct list_head fidlist;
};

struct p9_fid *v9fs_fid_lookup(struct dentry *dentry);
struct p9_fid *v9fs_fid_clone(struct dentry *dentry);
int v9fs_fid_add(struct dentry *dentry, struct p9_fid *fid);
