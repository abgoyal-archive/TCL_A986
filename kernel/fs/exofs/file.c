

#include <linux/buffer_head.h>

#include "exofs.h"

static int exofs_release_file(struct inode *inode, struct file *filp)
{
	return 0;
}

static int exofs_file_fsync(struct file *filp, int datasync)
{
	int ret;
	struct address_space *mapping = filp->f_mapping;
	struct inode *inode = mapping->host;
	struct super_block *sb;

	ret = filemap_write_and_wait(mapping);
	if (ret)
		return ret;

	/* sync the inode attributes */
	ret = write_inode_now(inode, 1);

	/* This is a good place to write the sb */
	/* TODO: Sechedule an sb-sync on create */
	sb = inode->i_sb;
	if (sb->s_dirt)
		exofs_sync_fs(sb, 1);

	return ret;
}

static int exofs_flush(struct file *file, fl_owner_t id)
{
	exofs_file_fsync(file, 1);
	/* TODO: Flush the OSD target */
	return 0;
}

const struct file_operations exofs_file_operations = {
	.llseek		= generic_file_llseek,
	.read		= do_sync_read,
	.write		= do_sync_write,
	.aio_read	= generic_file_aio_read,
	.aio_write	= generic_file_aio_write,
	.mmap		= generic_file_mmap,
	.open		= generic_file_open,
	.release	= exofs_release_file,
	.fsync		= exofs_file_fsync,
	.flush		= exofs_flush,
	.splice_read	= generic_file_splice_read,
	.splice_write	= generic_file_splice_write,
};

const struct inode_operations exofs_file_inode_operations = {
	.truncate	= exofs_truncate,
	.setattr	= exofs_setattr,
};
