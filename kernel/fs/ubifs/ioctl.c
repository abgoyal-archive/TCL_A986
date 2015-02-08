

/* This file implements EXT2-compatible extended attribute ioctl() calls */

#include <linux/compat.h>
#include <linux/mount.h>
#include "ubifs.h"

void ubifs_set_inode_flags(struct inode *inode)
{
	unsigned int flags = ubifs_inode(inode)->flags;

	inode->i_flags &= ~(S_SYNC | S_APPEND | S_IMMUTABLE | S_DIRSYNC);
	if (flags & UBIFS_SYNC_FL)
		inode->i_flags |= S_SYNC;
	if (flags & UBIFS_APPEND_FL)
		inode->i_flags |= S_APPEND;
	if (flags & UBIFS_IMMUTABLE_FL)
		inode->i_flags |= S_IMMUTABLE;
	if (flags & UBIFS_DIRSYNC_FL)
		inode->i_flags |= S_DIRSYNC;
}

static int ioctl2ubifs(int ioctl_flags)
{
	int ubifs_flags = 0;

	if (ioctl_flags & FS_COMPR_FL)
		ubifs_flags |= UBIFS_COMPR_FL;
	if (ioctl_flags & FS_SYNC_FL)
		ubifs_flags |= UBIFS_SYNC_FL;
	if (ioctl_flags & FS_APPEND_FL)
		ubifs_flags |= UBIFS_APPEND_FL;
	if (ioctl_flags & FS_IMMUTABLE_FL)
		ubifs_flags |= UBIFS_IMMUTABLE_FL;
	if (ioctl_flags & FS_DIRSYNC_FL)
		ubifs_flags |= UBIFS_DIRSYNC_FL;

	return ubifs_flags;
}

static int ubifs2ioctl(int ubifs_flags)
{
	int ioctl_flags = 0;

	if (ubifs_flags & UBIFS_COMPR_FL)
		ioctl_flags |= FS_COMPR_FL;
	if (ubifs_flags & UBIFS_SYNC_FL)
		ioctl_flags |= FS_SYNC_FL;
	if (ubifs_flags & UBIFS_APPEND_FL)
		ioctl_flags |= FS_APPEND_FL;
	if (ubifs_flags & UBIFS_IMMUTABLE_FL)
		ioctl_flags |= FS_IMMUTABLE_FL;
	if (ubifs_flags & UBIFS_DIRSYNC_FL)
		ioctl_flags |= FS_DIRSYNC_FL;

	return ioctl_flags;
}

static int setflags(struct inode *inode, int flags)
{
	int oldflags, err, release;
	struct ubifs_inode *ui = ubifs_inode(inode);
	struct ubifs_info *c = inode->i_sb->s_fs_info;
	struct ubifs_budget_req req = { .dirtied_ino = 1,
					.dirtied_ino_d = ui->data_len };

	err = ubifs_budget_space(c, &req);
	if (err)
		return err;

	/*
	 * The IMMUTABLE and APPEND_ONLY flags can only be changed by
	 * the relevant capability.
	 */
	mutex_lock(&ui->ui_mutex);
	oldflags = ubifs2ioctl(ui->flags);
	if ((flags ^ oldflags) & (FS_APPEND_FL | FS_IMMUTABLE_FL)) {
		if (!capable(CAP_LINUX_IMMUTABLE)) {
			err = -EPERM;
			goto out_unlock;
		}
	}

	ui->flags = ioctl2ubifs(flags);
	ubifs_set_inode_flags(inode);
	inode->i_ctime = ubifs_current_time(inode);
	release = ui->dirty;
	mark_inode_dirty_sync(inode);
	mutex_unlock(&ui->ui_mutex);

	if (release)
		ubifs_release_budget(c, &req);
	if (IS_SYNC(inode))
		err = write_inode_now(inode, 1);
	return err;

out_unlock:
	ubifs_err("can't modify inode %lu attributes", inode->i_ino);
	mutex_unlock(&ui->ui_mutex);
	ubifs_release_budget(c, &req);
	return err;
}

long ubifs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int flags, err;
	struct inode *inode = file->f_path.dentry->d_inode;

	switch (cmd) {
	case FS_IOC_GETFLAGS:
		flags = ubifs2ioctl(ubifs_inode(inode)->flags);

		dbg_gen("get flags: %#x, i_flags %#x", flags, inode->i_flags);
		return put_user(flags, (int __user *) arg);

	case FS_IOC_SETFLAGS: {
		if (IS_RDONLY(inode))
			return -EROFS;

		if (!is_owner_or_cap(inode))
			return -EACCES;

		if (get_user(flags, (int __user *) arg))
			return -EFAULT;

		if (!S_ISDIR(inode->i_mode))
			flags &= ~FS_DIRSYNC_FL;

		/*
		 * Make sure the file-system is read-write and make sure it
		 * will not become read-only while we are changing the flags.
		 */
		err = mnt_want_write(file->f_path.mnt);
		if (err)
			return err;
		dbg_gen("set flags: %#x, i_flags %#x", flags, inode->i_flags);
		err = setflags(inode, flags);
		mnt_drop_write(file->f_path.mnt);
		return err;
	}

	default:
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
long ubifs_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case FS_IOC32_GETFLAGS:
		cmd = FS_IOC_GETFLAGS;
		break;
	case FS_IOC32_SETFLAGS:
		cmd = FS_IOC_SETFLAGS;
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return ubifs_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif
