

#include <linux/kernel.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/syscalls.h>
#include <linux/linkage.h>
#include <linux/pagemap.h>
#include <linux/quotaops.h>
#include <linux/buffer_head.h>
#include <linux/backing-dev.h>
#include "internal.h"

#define VALID_FLAGS (SYNC_FILE_RANGE_WAIT_BEFORE|SYNC_FILE_RANGE_WRITE| \
			SYNC_FILE_RANGE_WAIT_AFTER)

static int __sync_filesystem(struct super_block *sb, int wait)
{
	/*
	 * This should be safe, as we require bdi backing to actually
	 * write out data in the first place
	 */
	if (!sb->s_bdi || sb->s_bdi == &noop_backing_dev_info)
		return 0;

	if (sb->s_qcop && sb->s_qcop->quota_sync)
		sb->s_qcop->quota_sync(sb, -1, wait);

	if (wait)
		sync_inodes_sb(sb);
	else
		writeback_inodes_sb(sb);

	if (sb->s_op->sync_fs)
		sb->s_op->sync_fs(sb, wait);
	return __sync_blockdev(sb->s_bdev, wait);
}

int sync_filesystem(struct super_block *sb)
{
	int ret;

	/*
	 * We need to be protected against the filesystem going from
	 * r/o to r/w or vice versa.
	 */
	WARN_ON(!rwsem_is_locked(&sb->s_umount));

	/*
	 * No point in syncing out anything if the filesystem is read-only.
	 */
	if (sb->s_flags & MS_RDONLY)
		return 0;

	ret = __sync_filesystem(sb, 0);
	if (ret < 0)
		return ret;
	return __sync_filesystem(sb, 1);
}
EXPORT_SYMBOL_GPL(sync_filesystem);

static void sync_one_sb(struct super_block *sb, void *arg)
{
	if (!(sb->s_flags & MS_RDONLY) && sb->s_bdi)
		__sync_filesystem(sb, *(int *)arg);
}
static void sync_filesystems(int wait)
{
	iterate_supers(sync_one_sb, &wait);
}

SYSCALL_DEFINE0(sync)
{
	wakeup_flusher_threads(0);
	sync_filesystems(0);
	sync_filesystems(1);
	if (unlikely(laptop_mode))
		laptop_sync_completion();
	return 0;
}

static void do_sync_work(struct work_struct *work)
{
	/*
	 * Sync twice to reduce the possibility we skipped some inodes / pages
	 * because they were temporarily locked
	 */
	sync_filesystems(0);
	sync_filesystems(0);
	printk("Emergency Sync complete\n");
	kfree(work);
}

void emergency_sync(void)
{
	struct work_struct *work;

	work = kmalloc(sizeof(*work), GFP_ATOMIC);
	if (work) {
		INIT_WORK(work, do_sync_work);
		schedule_work(work);
	}
}

int file_fsync(struct file *filp, int datasync)
{
	struct inode *inode = filp->f_mapping->host;
	struct super_block * sb;
	int ret, err;

	/* sync the inode to buffers */
	ret = write_inode_now(inode, 0);

	/* sync the superblock to buffers */
	sb = inode->i_sb;
	if (sb->s_dirt && sb->s_op->write_super)
		sb->s_op->write_super(sb);

	/* .. finally sync the buffers to disk */
	err = sync_blockdev(sb->s_bdev);
	if (!ret)
		ret = err;
	return ret;
}
EXPORT_SYMBOL(file_fsync);

int vfs_fsync_range(struct file *file, loff_t start, loff_t end, int datasync)
{
	struct address_space *mapping = file->f_mapping;
	int err, ret;

	if (!file->f_op || !file->f_op->fsync) {
		ret = -EINVAL;
		goto out;
	}

	ret = filemap_write_and_wait_range(mapping, start, end);

	/*
	 * We need to protect against concurrent writers, which could cause
	 * livelocks in fsync_buffers_list().
	 */
	mutex_lock(&mapping->host->i_mutex);
	err = file->f_op->fsync(file, datasync);
	if (!ret)
		ret = err;
	mutex_unlock(&mapping->host->i_mutex);

out:
	return ret;
}
EXPORT_SYMBOL(vfs_fsync_range);

int vfs_fsync(struct file *file, int datasync)
{
	return vfs_fsync_range(file, 0, LLONG_MAX, datasync);
}
EXPORT_SYMBOL(vfs_fsync);

static int do_fsync(unsigned int fd, int datasync)
{
	struct file *file;
	int ret = -EBADF;

	file = fget(fd);
	if (file) {
		ret = vfs_fsync(file, datasync);
		fput(file);
	}
	return ret;
}

SYSCALL_DEFINE1(fsync, unsigned int, fd)
{
	return do_fsync(fd, 0);
}

SYSCALL_DEFINE1(fdatasync, unsigned int, fd)
{
	return do_fsync(fd, 1);
}

int generic_write_sync(struct file *file, loff_t pos, loff_t count)
{
	if (!(file->f_flags & O_DSYNC) && !IS_SYNC(file->f_mapping->host))
		return 0;
	return vfs_fsync_range(file, pos, pos + count - 1,
			       (file->f_flags & __O_SYNC) ? 0 : 1);
}
EXPORT_SYMBOL(generic_write_sync);

SYSCALL_DEFINE(sync_file_range)(int fd, loff_t offset, loff_t nbytes,
				unsigned int flags)
{
	int ret;
	struct file *file;
	struct address_space *mapping;
	loff_t endbyte;			/* inclusive */
	int fput_needed;
	umode_t i_mode;

	ret = -EINVAL;
	if (flags & ~VALID_FLAGS)
		goto out;

	endbyte = offset + nbytes;

	if ((s64)offset < 0)
		goto out;
	if ((s64)endbyte < 0)
		goto out;
	if (endbyte < offset)
		goto out;

	if (sizeof(pgoff_t) == 4) {
		if (offset >= (0x100000000ULL << PAGE_CACHE_SHIFT)) {
			/*
			 * The range starts outside a 32 bit machine's
			 * pagecache addressing capabilities.  Let it "succeed"
			 */
			ret = 0;
			goto out;
		}
		if (endbyte >= (0x100000000ULL << PAGE_CACHE_SHIFT)) {
			/*
			 * Out to EOF
			 */
			nbytes = 0;
		}
	}

	if (nbytes == 0)
		endbyte = LLONG_MAX;
	else
		endbyte--;		/* inclusive */

	ret = -EBADF;
	file = fget_light(fd, &fput_needed);
	if (!file)
		goto out;

	i_mode = file->f_path.dentry->d_inode->i_mode;
	ret = -ESPIPE;
	if (!S_ISREG(i_mode) && !S_ISBLK(i_mode) && !S_ISDIR(i_mode) &&
			!S_ISLNK(i_mode))
		goto out_put;

	mapping = file->f_mapping;
	if (!mapping) {
		ret = -EINVAL;
		goto out_put;
	}

	ret = 0;
	if (flags & SYNC_FILE_RANGE_WAIT_BEFORE) {
		ret = filemap_fdatawait_range(mapping, offset, endbyte);
		if (ret < 0)
			goto out_put;
	}

	if (flags & SYNC_FILE_RANGE_WRITE) {
		ret = filemap_fdatawrite_range(mapping, offset, endbyte);
		if (ret < 0)
			goto out_put;
	}

	if (flags & SYNC_FILE_RANGE_WAIT_AFTER)
		ret = filemap_fdatawait_range(mapping, offset, endbyte);

out_put:
	fput_light(file, fput_needed);
out:
	return ret;
}
#ifdef CONFIG_HAVE_SYSCALL_WRAPPERS
asmlinkage long SyS_sync_file_range(long fd, loff_t offset, loff_t nbytes,
				    long flags)
{
	return SYSC_sync_file_range((int) fd, offset, nbytes,
				    (unsigned int) flags);
}
SYSCALL_ALIAS(sys_sync_file_range, SyS_sync_file_range);
#endif

SYSCALL_DEFINE(sync_file_range2)(int fd, unsigned int flags,
				 loff_t offset, loff_t nbytes)
{
	return sys_sync_file_range(fd, offset, nbytes, flags);
}
#ifdef CONFIG_HAVE_SYSCALL_WRAPPERS
asmlinkage long SyS_sync_file_range2(long fd, long flags,
				     loff_t offset, loff_t nbytes)
{
	return SYSC_sync_file_range2((int) fd, (unsigned int) flags,
				     offset, nbytes);
}
SYSCALL_ALIAS(sys_sync_file_range2, SyS_sync_file_range2);
#endif
