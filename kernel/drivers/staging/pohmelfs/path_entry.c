

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/fs_struct.h>
#include <linux/pagemap.h>
#include <linux/writeback.h>
#include <linux/mount.h>
#include <linux/mm.h>

#include "netfs.h"

#define UNHASHED_OBSCURE_STRING_SIZE		sizeof(" (deleted)")

int pohmelfs_construct_path_string(struct pohmelfs_inode *pi, void *data, int len)
{
	struct path path;
	struct dentry *d;
	char *ptr;
	int err = 0, strlen, reduce = 0;

	d = d_find_alias(&pi->vfs_inode);
	if (!d) {
		printk("%s: no alias, list_empty: %d.\n", __func__, list_empty(&pi->vfs_inode.i_dentry));
		return -ENOENT;
	}

	read_lock(&current->fs->lock);
	path.mnt = mntget(current->fs->root.mnt);
	read_unlock(&current->fs->lock);

	path.dentry = d;

	if (!IS_ROOT(d) && d_unhashed(d))
		reduce = 1;

	ptr = d_path(&path, data, len);
	if (IS_ERR(ptr)) {
		err = PTR_ERR(ptr);
		goto out;
	}

	if (reduce && len >= UNHASHED_OBSCURE_STRING_SIZE) {
		char *end = data + len - UNHASHED_OBSCURE_STRING_SIZE;
		*end = '\0';
	}

	strlen = len - (ptr - (char *)data);
	memmove(data, ptr, strlen);
	ptr = data;

	err = strlen;

	dprintk("%s: dname: '%s', len: %u, maxlen: %u, name: '%s', strlen: %d.\n",
			__func__, d->d_name.name, d->d_name.len, len, ptr, strlen);

out:
	dput(d);
	mntput(path.mnt);

	return err;
}

int pohmelfs_path_length(struct pohmelfs_inode *pi)
{
	struct dentry *d, *root, *first;
	int len = 1; /* Root slash */

	first = d = d_find_alias(&pi->vfs_inode);
	if (!d) {
		dprintk("%s: ino: %llu, mode: %o.\n", __func__, pi->ino, pi->vfs_inode.i_mode);
		return -ENOENT;
	}

	read_lock(&current->fs->lock);
	root = dget(current->fs->root.dentry);
	read_unlock(&current->fs->lock);

	spin_lock(&dcache_lock);

	if (!IS_ROOT(d) && d_unhashed(d))
		len += UNHASHED_OBSCURE_STRING_SIZE; /* Obscure " (deleted)" string */

	while (d && d != root && !IS_ROOT(d)) {
		len += d->d_name.len + 1; /* Plus slash */
		d = d->d_parent;
	}
	spin_unlock(&dcache_lock);

	dput(root);
	dput(first);

	return len + 1; /* Including zero-byte */
}
